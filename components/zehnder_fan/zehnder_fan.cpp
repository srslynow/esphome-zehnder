#include "zehnder_fan.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "nvs_flash.h"
#include "nvs.h"

namespace esphome {
namespace zehnder_fan {

static const char *const TAG = "zehnder_fan";
static const char *const NVS_NAMESPACE = "zehnder_fan";
static const char *const NVS_PAIRING_KEY = "pairing_info";

// =========================================================================
// 1. NRF905Controller Implementation
// =========================================================================

void NRF905Controller::setup_pins(GPIOPin *pwr_pin, GPIOPin *ce_pin, GPIOPin *txen_pin, GPIOPin *dr_pin) {
    this->pwr_pin_ = pwr_pin;
    this->ce_pin_ = ce_pin;
    this->txen_pin_ = txen_pin;
    this->dr_pin_ = dr_pin;
}

bool NRF905Controller::init() {
    // Initialize SPI device
    this->spi_setup();
    
    this->pwr_pin_->setup();
    this->ce_pin_->setup();
    this->txen_pin_->setup();
    this->dr_pin_->setup();

    this->set_mode_idle();

    // Zehnder nRF905 configuration profile from fan.h
    const uint8_t zehnder_config[] = {0x76, 0x2E, 0x44, 0x10, 0x10, 0xA5, 0x5A, 0x5A, 0xA5, 0xDB};
    this->write_config_registers(zehnder_config, sizeof(zehnder_config));

    ESP_LOGD(TAG, "NRF905 initialized.");
    return true;
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

void NRF905Controller::set_tx_address(uint32_t address) {
    this->enable();
    this->write_byte(0x22); // W_TX_ADDRESS
    this->write_byte((address >> 0) & 0xFF);
    this->write_byte((address >> 8) & 0xFF);
    this->write_byte((address >> 16) & 0xFF);
    this->write_byte((address >> 24) & 0xFF);
    this->disable();
}

void NRF905Controller::set_rx_address(uint32_t address) {
    // RX address is set via config registers, not a direct command
    const uint8_t config_update[] = {
        0x76, 0x2E, 0x44, 0x10, 0x10, // Keep first 5 bytes
        (uint8_t)((address >> 0) & 0xFF),
        (uint8_t)((address >> 8) & 0xFF),
        (uint8_t)((address >> 16) & 0xFF),
        (uint8_t)((address >> 24) & 0xFF),
        0xDB, // Keep last byte
    };
    this->write_config_registers(config_update, sizeof(config_update));
}


void NRF905Controller::write_tx_payload(const uint8_t *payload, size_t size) {
    this->enable();
    this->write_byte(0x20); // W_TX_PAYLOAD
    this->write_array(payload, size);
    this->disable();
}

bool NRF905Controller::read_rx_payload(uint8_t *buffer, size_t size) {
    if (!this->is_data_ready()) {
        return false;
    }
    this->enable();
    this->write_byte(0x24); // R_RX_PAYLOAD
    this->read_array(buffer, size);
    this->disable();
    return true;
}

void NRF905Controller::write_config_registers(const uint8_t *config, size_t size) {
    this->set_mode_idle();
    this->enable();
    this->write_byte(0x00); // W_CONFIG
    this->write_array(config, size);
    this->disable();
}


// =========================================================================
// 2. ZehnderFanProtocol Implementation
// =========================================================================

ZehnderFanProtocol::ZehnderFanProtocol(NRF905Controller *radio) : radio_(radio) {
    // Initialize pending operation to idle state
    pending_op_.type = RadioOperationType::NONE;
    pending_op_.state = RadioOperationState::IDLE;
}

void ZehnderFanProtocol::start_pairing() {
    if (pending_op_.state != RadioOperationState::IDLE) {
        ESP_LOGW(TAG, "Cannot start pairing: Radio operation already in progress");
        return;
    }
    
    ESP_LOGD(TAG, "Starting fan pairing discovery...");
    
    // Initialize pairing operation
    pending_op_.type = RadioOperationType::PAIRING_DISCOVER;
    pending_op_.state = RadioOperationState::IDLE; // Will be set to TRANSMITTING by setup_pairing_discover
    pending_op_.data.pairing.my_device_id = random_uint32() & 0xFE; // Avoid 0xFF
    if (pending_op_.data.pairing.my_device_id == 0x00) 
        pending_op_.data.pairing.my_device_id = 1;
    pending_op_.data.pairing.pairing_step = 0;
    
    setup_pairing_discover();
}

void ZehnderFanProtocol::start_set_speed(const FanPairingInfo &pairing_info, uint8_t speed, uint8_t timer_minutes) {
    if (pending_op_.state != RadioOperationState::IDLE) {
        ESP_LOGW(TAG, "Cannot set speed: Radio operation already in progress");
        return;
    }
    
    // Initialize set speed operation
    pending_op_.type = RadioOperationType::SET_SPEED;
    pending_op_.data.set_speed.pairing_info = pairing_info;
    pending_op_.data.set_speed.speed = speed;
    pending_op_.data.set_speed.timer_minutes = timer_minutes;
    pending_op_.max_retries = FAN_TX_RETRIES;
    pending_op_.retry_count = 0;
    pending_op_.timeout_ms = FAN_REPLY_TIMEOUT_MS;
    
    // Setup radio for this network
    radio_->set_mode_idle();
    radio_->set_tx_address(pairing_info.network_id);
    radio_->set_rx_address(pairing_info.network_id);
    
    // Prepare payload
    memset(pending_op_.tx_payload, 0, FAN_FRAMESIZE);
    pending_op_.tx_payload[0] = FAN_TYPE_MAIN_UNIT;
    pending_op_.tx_payload[1] = pairing_info.main_unit_id;
    pending_op_.tx_payload[2] = FAN_TYPE_REMOTE_CONTROL;
    pending_op_.tx_payload[3] = pairing_info.my_device_id;
    pending_op_.tx_payload[4] = 0xFA; // TTL
    pending_op_.tx_payload[5] = (timer_minutes > 0) ? FAN_FRAME_SETTIMER : FAN_FRAME_SETSPEED;
    pending_op_.tx_payload[6] = (timer_minutes > 0) ? 0x02 : 0x01; // Number of parameters
    pending_op_.tx_payload[7] = speed;
    pending_op_.tx_payload[8] = timer_minutes;
    
    start_transmit();
}

void ZehnderFanProtocol::process() {
    switch (pending_op_.state) {
        case RadioOperationState::IDLE:
            // Nothing to do
            break;
            
        case RadioOperationState::TRANSMITTING:
            // Check if we can move to receive mode (transmission should be quick)
            pending_op_.state = RadioOperationState::WAITING_RESPONSE;
            pending_op_.start_time = millis();
            radio_->set_mode_receive();
            break;
            
        case RadioOperationState::WAITING_RESPONSE:
            // Check for received data
            if (radio_->read_rx_payload(rx_buffer_, FAN_FRAMESIZE)) {
                handle_response();
            } else {
                // Check for timeout
                uint32_t elapsed = millis() - pending_op_.start_time;
                if (elapsed >= pending_op_.timeout_ms) {
                    retry_or_fail();
                }
            }
            break;
            
        case RadioOperationState::OPERATION_COMPLETE:
            // Operation finished, waiting for external reset
            break;
    }
}

void ZehnderFanProtocol::start_transmit() {
    radio_->write_tx_payload(pending_op_.tx_payload, FAN_FRAMESIZE);
    pending_op_.state = RadioOperationState::TRANSMITTING;
    radio_->set_mode_transmit();
    // Note: We'll move to WAITING_RESPONSE in the next process() call
}

void ZehnderFanProtocol::handle_response() {
    if (pending_op_.type == RadioOperationType::SET_SPEED) {
        // For set speed, any response is considered success
        ESP_LOGD(TAG, "Set speed command acknowledged.");
        complete_operation(true);
        
    } else if (pending_op_.type >= RadioOperationType::PAIRING_DISCOVER && 
               pending_op_.type <= RadioOperationType::PAIRING_ACK) {
        handle_pairing_response();
    }
}

void ZehnderFanProtocol::retry_or_fail() {
    pending_op_.retry_count++;
    
    if (pending_op_.retry_count < pending_op_.max_retries) {
        ESP_LOGD(TAG, "Radio timeout, retrying (%d/%d)", pending_op_.retry_count, pending_op_.max_retries);
        start_transmit();
    } else {
        ESP_LOGW(TAG, "Radio operation failed after %d retries", pending_op_.max_retries);
        complete_operation(false);
    }
}

void ZehnderFanProtocol::complete_operation(bool success) {
    pending_op_.state = RadioOperationState::OPERATION_COMPLETE;
    last_operation_success_ = success;
    radio_->set_mode_idle();
}

std::optional<FanPairingInfo> ZehnderFanProtocol::get_pairing_result() {
    if (pending_op_.type == RadioOperationType::PAIRING_ACK && 
        pending_op_.state == RadioOperationState::OPERATION_COMPLETE &&
        last_operation_success_) {
        return pairing_result_;
    }
    return std::nullopt;
}

void ZehnderFanProtocol::reset_operation_state() {
    pending_op_.state = RadioOperationState::IDLE;
    pending_op_.type = RadioOperationType::NONE;
    radio_->set_mode_idle();
}

// Pairing state machine implementation
void ZehnderFanProtocol::setup_pairing_discover() {
    radio_->set_mode_idle();
    radio_->set_tx_address(NETWORK_LINK_ID);
    radio_->set_rx_address(NETWORK_LINK_ID);
    
    pending_op_.max_retries = FAN_TX_RETRIES;
    pending_op_.retry_count = 0;
    pending_op_.timeout_ms = FAN_REPLY_TIMEOUT_MS;
    
    // Prepare discovery payload
    memset(pending_op_.tx_payload, 0, FAN_FRAMESIZE);
    pending_op_.tx_payload[0] = 0x04;
    pending_op_.tx_payload[1] = 0x00;
    pending_op_.tx_payload[2] = FAN_TYPE_REMOTE_CONTROL;
    pending_op_.tx_payload[3] = pending_op_.data.pairing.my_device_id;
    pending_op_.tx_payload[4] = 0xFA;
    pending_op_.tx_payload[5] = FAN_NETWORK_JOIN_ACK;
    pending_op_.tx_payload[6] = 0x04;
    pending_op_.tx_payload[7] = 0xa5;
    pending_op_.tx_payload[8] = 0x5a;
    pending_op_.tx_payload[9] = 0x5a;
    pending_op_.tx_payload[10] = 0xa5;
    
    start_transmit();
}

void ZehnderFanProtocol::setup_pairing_join() {
    auto &info = pending_op_.data.pairing.current_info;
    
    radio_->set_tx_address(info.network_id);
    radio_->set_rx_address(info.network_id);
    
    pending_op_.type = RadioOperationType::PAIRING_JOIN;
    pending_op_.retry_count = 0;
    
    // Prepare join payload
    memset(pending_op_.tx_payload, 0, FAN_FRAMESIZE);
    pending_op_.tx_payload[0] = FAN_TYPE_MAIN_UNIT;
    pending_op_.tx_payload[1] = info.main_unit_id;
    pending_op_.tx_payload[2] = FAN_TYPE_REMOTE_CONTROL;
    pending_op_.tx_payload[3] = pending_op_.data.pairing.my_device_id;
    pending_op_.tx_payload[4] = 0xFA;
    pending_op_.tx_payload[5] = FAN_NETWORK_JOIN_REQUEST;
    pending_op_.tx_payload[7] = (info.network_id >> 0) & 0xFF;
    pending_op_.tx_payload[8] = (info.network_id >> 8) & 0xFF;
    pending_op_.tx_payload[9] = (info.network_id >> 16) & 0xFF;
    pending_op_.tx_payload[10] = (info.network_id >> 24) & 0xFF;
    
    start_transmit();
}

void ZehnderFanProtocol::setup_pairing_ack() {
    auto &info = pending_op_.data.pairing.current_info;
    
    pending_op_.type = RadioOperationType::PAIRING_ACK;
    pending_op_.retry_count = 0;
    pending_op_.max_retries = 1; // Fire and forget
    
    // Prepare ack payload
    memset(pending_op_.tx_payload, 0, FAN_FRAMESIZE);
    pending_op_.tx_payload[0] = FAN_TYPE_MAIN_UNIT;
    pending_op_.tx_payload[1] = info.main_unit_id;
    pending_op_.tx_payload[2] = FAN_TYPE_REMOTE_CONTROL;
    pending_op_.tx_payload[3] = pending_op_.data.pairing.my_device_id;
    pending_op_.tx_payload[4] = 0xFA;
    pending_op_.tx_payload[5] = FAN_FRAME_0B;
    
    start_transmit();
}

void ZehnderFanProtocol::handle_pairing_response() {
    if (pending_op_.type == RadioOperationType::PAIRING_DISCOVER) {
        if (rx_buffer_[5] != FAN_NETWORK_JOIN_OPEN) {
            ESP_LOGW(TAG, "Pairing failed: Received unexpected frame type 0x%02X.", rx_buffer_[5]);
            complete_operation(false);
            return;
        }
        
        // Extract pairing info from response
        auto &info = pending_op_.data.pairing.current_info;
        info.main_unit_type = rx_buffer_[2];
        info.main_unit_id = rx_buffer_[3];
        info.network_id = (uint32_t)rx_buffer_[7] | ((uint32_t)rx_buffer_[8] << 8) | 
                         ((uint32_t)rx_buffer_[9] << 16) | ((uint32_t)rx_buffer_[10] << 24);
        info.my_device_id = pending_op_.data.pairing.my_device_id;
        
        ESP_LOGD(TAG, "Found fan unit ID 0x%02X on network 0x%08X. Requesting to join...", 
                 info.main_unit_id, info.network_id);
        
        // Move to join phase
        setup_pairing_join();
        
    } else if (pending_op_.type == RadioOperationType::PAIRING_JOIN) {
        // Join acknowledged, send final ack
        ESP_LOGD(TAG, "Join request acknowledged, sending final ack...");
        setup_pairing_ack();
        
    } else if (pending_op_.type == RadioOperationType::PAIRING_ACK) {
        // Pairing complete!
        auto &info = pending_op_.data.pairing.current_info;
        pairing_result_ = info;
        
        ESP_LOGI(TAG, "Pairing successful! Network ID: 0x%08X, Fan ID: 0x%02X, My Device ID: 0x%02X",
                 info.network_id, info.main_unit_id, info.my_device_id);
        
        complete_operation(true);
    }
}


// =========================================================================
// 3. ZehnderFanComponent Implementation
// =========================================================================

void ZehnderFanComponent::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Zehnder Fan...");

    // Initialize nRF905 SPI device
    this->nrf_radio_.set_spi_parent(this->spi_parent_);
    this->nrf_radio_.set_cs_pin(this->cs_pin_);
    this->nrf_radio_.setup_pins(this->pwr_pin_, this->ce_pin_, this->txen_pin_, this->dr_pin_);
    this->nrf_radio_.init();

    this->fan_protocol_ = make_unique<ZehnderFanProtocol>(&this->nrf_radio_);
    
    // Initialize NVS
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    if (this->load_pairing_info()) {
        ESP_LOGI(TAG, "Loaded pairing info from NVS.");
    } else {
        ESP_LOGW(TAG, "No pairing info found. Fan needs to be paired.");
    }
}

void ZehnderFanComponent::loop() {
    // Process async radio operations
    this->fan_protocol_->process();
    
    // Handle operation completion
    if (this->fan_protocol_->is_operation_complete()) {
        this->handle_operation_complete();
    }
}

void ZehnderFanComponent::update() {
    // Required by PollingComponent, but not needed for this implementation
}

void ZehnderFanComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Zehnder Fan Component:");
    LOG_PIN("  PWR Pin: ", this->pwr_pin_);
    LOG_PIN("  CE Pin: ", this->ce_pin_);
    LOG_PIN("  TXEN Pin: ", this->txen_pin_);
    LOG_PIN("  DR Pin: ", this->dr_pin_);
    if (this->pairing_info_.has_value()) {
        ESP_LOGCONFIG(TAG, "  Paired Network ID: 0x%08X", this->pairing_info_->network_id);
        ESP_LOGCONFIG(TAG, "  Paired Fan ID: 0x%02X", this->pairing_info_->main_unit_id);
    } else {
        ESP_LOGCONFIG(TAG, "  Device is not paired.");
    }
}

fan::FanTraits ZehnderFanComponent::get_traits() {
    // The fan supports Off, Low, Medium, High, Max speeds.
    return fan::FanTraits(false, true, false, 4);
}

void ZehnderFanComponent::control(const fan::FanCall &call) {
    if (!this->pairing_info_.has_value()) {
        ESP_LOGE(TAG, "Cannot control fan: Not paired.");
        return;
    }
    
    // Check if radio is busy
    if (this->component_state_ != ComponentOperationState::IDLE) {
        ESP_LOGW(TAG, "Cannot control fan: Radio operation in progress, ignoring request.");
        return;
    }

    // Store pending state changes
    if (call.get_state().has_value()) {
        this->pending_fan_state_ = *call.get_state();
        this->pending_state_change_ = true;
    }
    if (call.get_speed().has_value()) {
        this->pending_fan_speed_ = *call.get_speed();
        this->pending_state_change_ = true;
    }

    uint8_t fan_speed = FAN_SPEED_AUTO; // Off
    if (this->pending_fan_state_) {
        switch (this->pending_fan_speed_) {
            case 1: fan_speed = FAN_SPEED_LOW; break;
            case 2: fan_speed = FAN_SPEED_MEDIUM; break;
            case 3: fan_speed = FAN_SPEED_HIGH; break;
            case 4: fan_speed = FAN_SPEED_MAX; break;
        }
    }
    
    // For now, timer is not implemented via Home Assistant fan model. Could be a separate service.
    uint8_t timer = 0; 
    
    ESP_LOGD(TAG, "Setting fan speed to level %d", this->pending_fan_speed_);
    
    // Start async operation
    this->component_state_ = ComponentOperationState::SETTING_SPEED;
    this->fan_protocol_->start_set_speed(this->pairing_info_.value(), fan_speed, timer);
}

void ZehnderFanComponent::start_pairing() {
    ESP_LOGI(TAG, "Pairing service called. Attempting to discover and pair with fan...");
    
    // Check if radio is busy
    if (this->component_state_ != ComponentOperationState::IDLE) {
        ESP_LOGW(TAG, "Cannot start pairing: Radio operation in progress.");
        return;
    }
    
    // Start async pairing operation
    this->component_state_ = ComponentOperationState::PAIRING;
    this->fan_protocol_->start_pairing();
}

void ZehnderFanComponent::handle_operation_complete() {
    bool success = this->fan_protocol_->last_operation_successful();
    
    if (this->component_state_ == ComponentOperationState::SETTING_SPEED) {
        if (success) {
            // Apply the pending state changes
            if (this->pending_state_change_) {
                this->state = this->pending_fan_state_;
                this->speed = this->pending_fan_speed_;
                this->pending_state_change_ = false;
                this->publish_state();
                ESP_LOGD(TAG, "Fan speed set successfully");
            }
        } else {
            ESP_LOGW(TAG, "Failed to set fan speed");
        }
        
    } else if (this->component_state_ == ComponentOperationState::PAIRING) {
        if (success) {
            auto result = this->fan_protocol_->get_pairing_result();
            if (result.has_value()) {
                this->save_pairing_info(result.value());
                this->load_pairing_info(); // Reload into component state
                ESP_LOGI(TAG, "Pairing successful and info saved to flash.");
            }
        } else {
            ESP_LOGE(TAG, "Pairing failed.");
        }
    }
    
    // Reset operation state and radio protocol state
    this->component_state_ = ComponentOperationState::IDLE;
    this->fan_protocol_->reset_operation_state();
}

void ZehnderFanComponent::save_pairing_info(const FanPairingInfo &info) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle!", esp_err_to_name(err));
        return;
    }

    err = nvs_set_blob(nvs_handle, NVS_PAIRING_KEY, &info, sizeof(FanPairingInfo));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) writing pairing info to NVS!", esp_err_to_name(err));
    } else {
        err = nvs_commit(nvs_handle);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error (%s) committing NVS!", esp_err_to_name(err));
        } else {
            ESP_LOGD(TAG, "Pairing info saved to NVS successfully.");
        }
    }

    nvs_close(nvs_handle);
}

bool ZehnderFanComponent::load_pairing_info() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Error (%s) opening NVS handle for reading!", esp_err_to_name(err));
        this->pairing_info_ = std::nullopt;
        return false;
    }

    FanPairingInfo loaded_info;
    size_t required_size = sizeof(FanPairingInfo);
    err = nvs_get_blob(nvs_handle, NVS_PAIRING_KEY, &loaded_info, &required_size);
    nvs_close(nvs_handle);

    if (err == ESP_OK && required_size == sizeof(FanPairingInfo)) {
        ESP_LOGI(TAG, "Loaded pairing info: Network ID 0x%08X, Fan ID 0x%02X, My Device ID 0x%02X",
                 loaded_info.network_id, loaded_info.main_unit_id, loaded_info.my_device_id);
        this->pairing_info_ = loaded_info;
        return true;
    } else {
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGW(TAG, "No pairing info found in NVS. Device is not paired.");
        } else {
            ESP_LOGW(TAG, "Error (%s) reading pairing info from NVS!", esp_err_to_name(err));
        }
        this->pairing_info_ = std::nullopt;
        return false;
    }
}

void ZehnderFanComponent::clear_pairing_info() {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS handle for clearing!", esp_err_to_name(err));
        return;
    }

    err = nvs_erase_key(nvs_handle, NVS_PAIRING_KEY);
    if (err == ESP_OK) {
        err = nvs_commit(nvs_handle);
        if (err == ESP_OK) {
            ESP_LOGD(TAG, "Pairing info cleared from NVS successfully.");
        } else {
            ESP_LOGE(TAG, "Error (%s) committing NVS after clearing!", esp_err_to_name(err));
        }
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGD(TAG, "No pairing info to clear in NVS.");
    } else {
        ESP_LOGE(TAG, "Error (%s) clearing pairing info from NVS!", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    this->pairing_info_ = std::nullopt;
}

} // namespace zehnder_fan
} // namespace esphome