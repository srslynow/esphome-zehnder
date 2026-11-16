#include "zehnder_fan.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cinttypes>

#ifdef USE_ESP32
#include "nvs.h"
#endif

namespace esphome {
namespace zehnder_fan {

static const char *const TAG = "zehnder_fan";

// nRF905 SPI instructions
static constexpr uint8_t NRF905_W_CONFIG = 0x00;
static constexpr uint8_t NRF905_R_CONFIG = 0x10;
static constexpr uint8_t NRF905_W_TX_PAYLOAD = 0x20;
static constexpr uint8_t NRF905_W_TX_ADDRESS = 0x22;
static constexpr uint8_t NRF905_R_RX_PAYLOAD = 0x24;

// Distinguishes the pairing preference from the fan state preference of the same entity.
static constexpr uint32_t PAIRING_PREFERENCE_VERSION = 0x5A464E31;  // "ZFN1"

static constexpr int SPEED_COUNT = 4;

static std::array<uint8_t, 4> to_little_endian(uint32_t value) {
    return {static_cast<uint8_t>(value), static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value >> 16),
            static_cast<uint8_t>(value >> 24)};
}

// =========================================================================
// Frame
// =========================================================================

Frame::Frame(uint8_t rx_type, uint8_t rx_id, uint8_t tx_type, uint8_t tx_id, FrameType type,
             std::initializer_list<uint8_t> parameters) {
    const size_t count = std::min(parameters.size(), MAX_PARAMETERS);
    this->bytes_[0] = rx_type;
    this->bytes_[1] = rx_id;
    this->bytes_[2] = tx_type;
    this->bytes_[3] = tx_id;
    this->bytes_[4] = FRAME_TTL;
    this->bytes_[5] = type;
    this->bytes_[6] = static_cast<uint8_t>(count);
    std::copy_n(parameters.begin(), count, this->bytes_.begin() + 7);
}

uint32_t Frame::network_id() const {
    return static_cast<uint32_t>(this->parameter(0)) | (static_cast<uint32_t>(this->parameter(1)) << 8) |
           (static_cast<uint32_t>(this->parameter(2)) << 16) | (static_cast<uint32_t>(this->parameter(3)) << 24);
}

// =========================================================================
// 1. NRF905Controller Implementation
// =========================================================================

bool NRF905Controller::setup() {
    this->spi_setup();

    this->pwr_pin_->setup();
    this->ce_pin_->setup();
    this->txen_pin_->setup();
    this->dr_pin_->setup();

    this->write_config_();
    if (!this->verify_config_()) {
        ESP_LOGE(TAG, "nRF905 does not respond; check the wiring.");
        return false;
    }

    ESP_LOGD(TAG, "nRF905 initialized.");
    return true;
}

void NRF905Controller::dump_config() {
    LOG_PIN("  CS Pin: ", this->cs_);
    LOG_PIN("  PWR Pin: ", this->pwr_pin_);
    LOG_PIN("  CE Pin: ", this->ce_pin_);
    LOG_PIN("  TXEN Pin: ", this->txen_pin_);
    LOG_PIN("  DR Pin: ", this->dr_pin_);
}

void NRF905Controller::set_mode_idle() {
    this->pwr_pin_->digital_write(true);
    this->ce_pin_->digital_write(false);
    this->txen_pin_->digital_write(false);
}

void NRF905Controller::set_mode_receive() {
    this->pwr_pin_->digital_write(true);
    this->txen_pin_->digital_write(false);
    this->ce_pin_->digital_write(true);
}

void NRF905Controller::set_mode_transmit() {
    this->pwr_pin_->digital_write(true);
    this->ce_pin_->digital_write(false); // Drop to standby before enabling TX
    delayMicroseconds(100);
    this->txen_pin_->digital_write(true);
    this->ce_pin_->digital_write(true);
}

void NRF905Controller::set_network_address(uint32_t address) {
    const auto bytes = to_little_endian(address);

    // The receive address lives in the config registers...
    std::copy(bytes.begin(), bytes.end(), this->config_.begin() + CONFIG_RX_ADDRESS_OFFSET);
    this->write_config_();

    // ...while the transmit address has its own instruction.
    this->enable();
    this->write_byte(NRF905_W_TX_ADDRESS);
    this->write_array(bytes.data(), bytes.size());
    this->disable();
}

void NRF905Controller::write_tx_payload(const Frame &frame) {
    this->set_mode_idle();
    this->enable();
    this->write_byte(NRF905_W_TX_PAYLOAD);
    this->write_array(frame.bytes().data(), frame.bytes().size());
    this->disable();
}

std::optional<Frame> NRF905Controller::read_rx_payload() {
    if (!this->dr_pin_->digital_read()) {
        return std::nullopt;
    }

    // Go to standby first, so a new packet cannot overwrite the payload while it is clocked out.
    this->set_mode_idle();

    Frame frame;
    this->enable();
    this->write_byte(NRF905_R_RX_PAYLOAD);
    this->read_array(frame.bytes().data(), frame.bytes().size());
    this->disable();
    return frame;
}

void NRF905Controller::write_config_() {
    this->set_mode_idle();
    this->enable();
    this->write_byte(NRF905_W_CONFIG);
    this->write_array(this->config_.data(), this->config_.size());
    this->disable();
}

bool NRF905Controller::verify_config_() {
    Config read_back{};
    this->enable();
    this->write_byte(NRF905_R_CONFIG);
    this->read_array(read_back.data(), read_back.size());
    this->disable();
    return read_back == this->config_;
}


// =========================================================================
// 2. ZehnderFanProtocol Implementation
// =========================================================================

bool ZehnderFanProtocol::start_pairing() {
    if (this->is_busy()) {
        return false;
    }

    this->peer_ = {};
    // Our own id for this network; 0x00 and 0xFF are reserved.
    this->peer_.my_device_id = static_cast<uint8_t>(1 + random_uint32() % 254);

    ESP_LOGD(TAG, "Searching for a unit that is open for pairing (my device ID: 0x%02X)...", this->peer_.my_device_id);

    this->radio_.set_network_address(NETWORK_LINK_ID);
    const auto link_id = to_little_endian(NETWORK_LINK_ID);
    // The receiver fields of this frame are fixed values rather than a real address.
    this->begin_step_(Step::PAIRING_DISCOVER,
                      Frame(0x04, 0x00, DEVICE_TYPE_REMOTE_CONTROL, this->peer_.my_device_id, FRAME_NETWORK_JOIN_ACK,
                            {link_id[0], link_id[1], link_id[2], link_id[3]}));
    return true;
}

bool ZehnderFanProtocol::start_set_speed(const FanPairingInfo &pairing_info, FanSpeed speed, uint8_t timer_minutes) {
    if (this->is_busy()) {
        return false;
    }

    this->peer_ = pairing_info;
    this->radio_.set_network_address(pairing_info.network_id);

    const Frame frame = timer_minutes > 0
        ? Frame(DEVICE_TYPE_MAIN_UNIT, pairing_info.main_unit_id, DEVICE_TYPE_REMOTE_CONTROL,
                pairing_info.my_device_id, FRAME_SET_TIMER, {speed, timer_minutes})
        : Frame(DEVICE_TYPE_MAIN_UNIT, pairing_info.main_unit_id, DEVICE_TYPE_REMOTE_CONTROL,
                pairing_info.my_device_id, FRAME_SET_SPEED, {speed});
    this->begin_step_(Step::SET_SPEED, frame);
    return true;
}

std::optional<OperationResult> ZehnderFanProtocol::loop() {
    if (!this->is_busy()) {
        return std::nullopt;
    }

    const uint32_t elapsed = millis() - this->phase_start_;

    if (this->phase_ == Phase::TRANSMITTING) {
        // The radio repeats the frame for as long as it stays in transmit mode.
        if (elapsed >= TX_WINDOW_MS) {
            this->radio_.set_mode_receive();
            this->phase_ = Phase::LISTENING;
            this->phase_start_ = millis();
        }
        return std::nullopt;
    }

    if (auto frame = this->radio_.read_rx_payload()) {
        auto result = this->handle_frame_(*frame);
        if (this->is_busy() && this->phase_ == Phase::LISTENING) {
            // Not the frame we were waiting for; keep listening until the timeout.
            this->radio_.set_mode_receive();
        }
        return result;
    }

    if (elapsed >= REPLY_TIMEOUT_MS) {
        return this->handle_timeout_();
    }
    return std::nullopt;
}

void ZehnderFanProtocol::begin_step_(Step step, const Frame &frame, uint8_t max_attempts) {
    this->step_ = step;
    this->tx_frame_ = frame;
    this->attempt_ = 0;
    this->max_attempts_ = max_attempts;
    this->transmit_();
}

void ZehnderFanProtocol::transmit_() {
    this->radio_.write_tx_payload(this->tx_frame_);
    this->radio_.set_mode_transmit();
    this->phase_ = Phase::TRANSMITTING;
    this->phase_start_ = millis();
}

std::optional<OperationResult> ZehnderFanProtocol::handle_frame_(const Frame &frame) {
    const bool for_me = frame.is_addressed_to(DEVICE_TYPE_REMOTE_CONTROL, this->peer_.my_device_id);

    switch (this->step_) {
        case Step::SET_SPEED:
            // The unit answers a speed command with its current settings. Other frames on the
            // network (such as units relaying our own command) do not tell us it was applied.
            if (frame.type() == FRAME_FAN_SETTINGS && for_me) {
                ESP_LOGD(TAG, "Unit acknowledged: speed %u, voltage %u%%.", frame.parameter(0), frame.parameter(1));
                return this->finish_(true);
            }
            break;

        case Step::PAIRING_DISCOVER:
            if (frame.type() == FRAME_NETWORK_JOIN_OPEN && frame.network_id() != 0) {
                this->peer_.main_unit_type = frame.tx_type();
                this->peer_.main_unit_id = frame.tx_id();
                this->peer_.network_id = frame.network_id();
                ESP_LOGD(TAG, "Found unit 0x%02X on network 0x%08" PRIX32 ". Requesting to join...",
                         this->peer_.main_unit_id, this->peer_.network_id);

                this->radio_.set_network_address(this->peer_.network_id);
                const auto network_id = to_little_endian(this->peer_.network_id);
                this->begin_step_(Step::PAIRING_JOIN,
                                  Frame(DEVICE_TYPE_MAIN_UNIT, this->peer_.main_unit_id, DEVICE_TYPE_REMOTE_CONTROL,
                                        this->peer_.my_device_id, FRAME_NETWORK_JOIN_REQUEST,
                                        {network_id[0], network_id[1], network_id[2], network_id[3]}));
                return std::nullopt;
            }
            break;

        case Step::PAIRING_JOIN:
            // Most units accept with 0x0B; the ComfoAir E300 RF is reported to answer with 0x04.
            if ((frame.type() == FRAME_NETWORK_JOIN_ACCEPTED || frame.type() == FRAME_NETWORK_JOIN_REQUEST) && for_me &&
                frame.is_from(this->peer_.main_unit_type, this->peer_.main_unit_id)) {
                ESP_LOGD(TAG, "Unit accepted the join request. Confirming...");
                this->begin_step_(Step::PAIRING_CONFIRM,
                                  Frame(DEVICE_TYPE_MAIN_UNIT, this->peer_.main_unit_id, DEVICE_TYPE_REMOTE_CONTROL,
                                        this->peer_.my_device_id, FRAME_NETWORK_JOIN_ACCEPTED),
                                  PAIRING_CONFIRM_ATTEMPTS);
                return std::nullopt;
            }
            break;

        case Step::PAIRING_CONFIRM:
            // The unit closes the handshake with a frame that it addresses to itself.
            if (frame.type() == FRAME_QUERY_NETWORK &&
                frame.is_addressed_to(this->peer_.main_unit_type, this->peer_.main_unit_id) &&
                frame.is_from(this->peer_.main_unit_type, this->peer_.main_unit_id)) {
                return this->finish_(true);
            }
            break;

        case Step::IDLE:
            break;
    }

    this->log_ignored_(frame);
    return std::nullopt;
}

std::optional<OperationResult> ZehnderFanProtocol::handle_timeout_() {
    this->attempt_++;
    if (this->attempt_ < this->max_attempts_) {
        ESP_LOGD(TAG, "No reply, retrying (%u/%u)", this->attempt_, this->max_attempts_);
        this->transmit_();
        return std::nullopt;
    }

    if (this->step_ == Step::PAIRING_CONFIRM) {
        // The unit accepted our join request in the previous step, so we are paired even
        // though we did not hear its final confirmation.
        ESP_LOGW(TAG, "Unit did not confirm the pairing; assuming it succeeded.");
        return this->finish_(true);
    }

    ESP_LOGW(TAG, "No reply after %u attempts.", this->max_attempts_);
    return this->finish_(false);
}

OperationResult ZehnderFanProtocol::finish_(bool success) {
    OperationResult result{};
    result.type = this->step_ == Step::SET_SPEED ? OperationType::SET_SPEED : OperationType::PAIRING;
    result.success = success;
    if (success && result.type == OperationType::PAIRING) {
        result.pairing_info = this->peer_;
    }

    this->step_ = Step::IDLE;
    this->radio_.set_mode_idle();
    return result;
}

void ZehnderFanProtocol::log_ignored_(const Frame &frame) const {
    ESP_LOGD(TAG, "Ignoring frame type 0x%02X from %02X:%02X to %02X:%02X", frame.type(), frame.tx_type(),
             frame.tx_id(), frame.rx_type(), frame.rx_id());
}


// =========================================================================
// 3. ZehnderFanComponent Implementation
// =========================================================================

void ZehnderFanComponent::setup() {
    if (!this->radio_->setup()) {
        this->mark_failed();
        return;
    }

    this->load_pairing_info_();

    // The unit keeps running at its last speed while we reboot, so only restore what we
    // know about it; nothing is transmitted here.
    if (auto restore = this->restore_state_()) {
        restore->apply(*this);
    }
}

void ZehnderFanComponent::loop() {
    if (auto result = this->protocol_.loop()) {
        this->handle_result_(*result);
    }
    if (!this->protocol_.is_busy()) {
        this->start_next_operation_();
    }
}

void ZehnderFanComponent::dump_config() {
    LOG_FAN("", "Zehnder Fan", this);
    this->radio_->dump_config();
    if (this->pairing_info_.has_value()) {
        ESP_LOGCONFIG(TAG, "  Paired Network ID: 0x%08" PRIX32, this->pairing_info_->network_id);
        ESP_LOGCONFIG(TAG, "  Paired Unit ID: 0x%02X", this->pairing_info_->main_unit_id);
        ESP_LOGCONFIG(TAG, "  My Device ID: 0x%02X", this->pairing_info_->my_device_id);
    } else {
        ESP_LOGCONFIG(TAG, "  Not paired.");
    }
    if (this->is_failed()) {
        ESP_LOGE(TAG, "  Communication with the nRF905 failed!");
    }
}

fan::FanTraits ZehnderFanComponent::get_traits() {
    // Speed levels: Low, Medium, High, Max. No oscillation or direction.
    return fan::FanTraits(false, true, false, SPEED_COUNT);
}

void ZehnderFanComponent::control(const fan::FanCall &call) {
    if (!this->pairing_info_.has_value()) {
        ESP_LOGE(TAG, "Cannot control fan: not paired.");
        return;
    }

    // Build on the most recent request, so calls that only change the state or only the speed
    // combine correctly with a request that is still on its way to the unit.
    Target target{this->state, this->speed};
    if (this->queued_target_.has_value() && this->queued_target_->timer_minutes == 0) {
        target = *this->queued_target_;
    } else if (this->active_target_.has_value() && this->active_target_->timer_minutes == 0) {
        target = *this->active_target_;
    }

    if (call.get_state().has_value()) {
        target.state = *call.get_state();
    }
    if (call.get_speed().has_value()) {
        target.speed = *call.get_speed();
    }
    target.speed = std::clamp(target.speed, 1, SPEED_COUNT);

    // Sent from loop() as soon as the radio is free; replaces any request that is still waiting.
    this->queued_target_ = target;
}

void ZehnderFanComponent::start_pairing() {
    ESP_LOGI(TAG, "Pairing requested.");
    this->pairing_requested_ = true;
}

void ZehnderFanComponent::clear_pairing() {
    // Preferences cannot be deleted, so store an empty record instead.
    this->save_pairing_info_(FanPairingInfo{});
    this->pairing_info_.reset();
    this->queued_target_.reset();
    ESP_LOGI(TAG, "Pairing cleared.");
}

void ZehnderFanComponent::set_speed_with_timer(int speed, uint8_t minutes) {
    if (!this->pairing_info_.has_value()) {
        ESP_LOGE(TAG, "Cannot control fan: not paired.");
        return;
    }
    if (minutes == 0) {
        ESP_LOGW(TAG, "Ignoring timer of 0 minutes.");
        return;
    }
    this->queued_target_ = Target{true, std::clamp(speed, 1, SPEED_COUNT), minutes};
}

void ZehnderFanComponent::start_next_operation_() {
    if (this->pairing_requested_) {
        this->pairing_requested_ = false;
        if (this->protocol_.start_pairing()) {
            this->high_freq_.start();
            return;
        }
    }

    if (this->queued_target_.has_value() && this->pairing_info_.has_value()) {
        const Target target = *this->queued_target_;
        this->queued_target_.reset();

        // "Off" is the unit's lowest setting: a ventilation unit never stops completely.
        const FanSpeed speed = target.state ? static_cast<FanSpeed>(target.speed) : FAN_SPEED_AUTO;
        ESP_LOGD(TAG, "Setting fan speed to %u (timer: %u min)", speed, target.timer_minutes);
        if (this->protocol_.start_set_speed(*this->pairing_info_, speed, target.timer_minutes)) {
            this->active_target_ = target;
            this->high_freq_.start();
            return;
        }
    }

    this->high_freq_.stop();
}

void ZehnderFanComponent::handle_result_(const OperationResult &result) {
    switch (result.type) {
        case OperationType::SET_SPEED: {
            const std::optional<Target> target = this->active_target_;
            this->active_target_.reset();

            if (!result.success) {
                ESP_LOGW(TAG, "Failed to set fan speed.");
            } else if (target.has_value() && target->timer_minutes == 0) {
                this->state = target->state;
                this->speed = target->speed;
            }
            // Also published on failure, so Home Assistant falls back to the last known state.
            this->publish_state();
            break;
        }

        case OperationType::PAIRING:
            if (result.success && result.pairing_info.has_value()) {
                this->pairing_info_ = result.pairing_info;
                this->save_pairing_info_(*result.pairing_info);
                ESP_LOGI(TAG, "Paired with unit 0x%02X on network 0x%08" PRIX32 " (my device ID: 0x%02X).",
                         result.pairing_info->main_unit_id, result.pairing_info->network_id,
                         result.pairing_info->my_device_id);
            } else {
                ESP_LOGE(TAG, "Pairing failed. Is the unit open for pairing (just powered on)?");
            }
            break;
    }
}

void ZehnderFanComponent::load_pairing_info_() {
    this->pairing_pref_ = this->make_entity_preference<FanPairingInfo>(PAIRING_PREFERENCE_VERSION);

    FanPairingInfo info{};
    if (this->pairing_pref_.load(&info)) {
        // An empty record means the pairing was cleared.
        if (info.network_id != 0) {
            this->pairing_info_ = info;
        }
    } else if (auto legacy = this->load_legacy_pairing_info_()) {
        ESP_LOGI(TAG, "Migrated pairing info from the previous storage format.");
        this->pairing_info_ = legacy;
        this->save_pairing_info_(*legacy);
    }

    if (this->pairing_info_.has_value()) {
        ESP_LOGI(TAG, "Paired with unit 0x%02X on network 0x%08" PRIX32 ".", this->pairing_info_->main_unit_id,
                 this->pairing_info_->network_id);
    } else {
        ESP_LOGW(TAG, "Not paired. Power cycle the ventilation unit and press the pair button.");
    }
}

/// Earlier versions of this component stored the pairing info directly in NVS. Read it once,
/// so existing installations do not have to pair again after updating.
std::optional<FanPairingInfo> ZehnderFanComponent::load_legacy_pairing_info_() {
#ifdef USE_ESP32
    nvs_handle_t handle;
    if (nvs_open("zehnder_fan", NVS_READONLY, &handle) != ESP_OK) {
        return std::nullopt;
    }

    FanPairingInfo info{};
    size_t size = sizeof(info);
    const esp_err_t err = nvs_get_blob(handle, "pairing_info", &info, &size);
    nvs_close(handle);

    if (err == ESP_OK && size == sizeof(info) && info.network_id != 0) {
        return info;
    }
#endif
    return std::nullopt;
}

void ZehnderFanComponent::save_pairing_info_(const FanPairingInfo &info) {
    if (!this->pairing_pref_.save(&info) || !global_preferences->sync()) {
        ESP_LOGE(TAG, "Could not store the pairing info; it will be lost after a restart.");
    }
}

} // namespace zehnder_fan
} // namespace esphome
