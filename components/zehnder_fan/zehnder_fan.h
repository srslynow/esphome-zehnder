#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/core/preferences.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan.h"

#include <array>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <optional>

namespace esphome {
namespace zehnder_fan {

// =========================================================================
// Protocol constants
// =========================================================================

inline constexpr size_t FRAME_SIZE = 16;
inline constexpr uint8_t FRAME_TTL = 0xFA;
/// Network every unit listens on while it is open for pairing.
inline constexpr uint32_t NETWORK_LINK_ID = 0xA55A5AA5;

/// How long the radio keeps (auto-re)transmitting one frame. A frame takes ~3.7 ms on air
/// (186 bits at 50 kbps), so this window sends it about four times.
inline constexpr uint32_t TX_WINDOW_MS = 16;
/// How long to listen for a reply before transmitting again.
inline constexpr uint32_t REPLY_TIMEOUT_MS = 500;
/// Number of transmit attempts. High on purpose: the antenna circuit of common nRF905 modules
/// is not tuned for 868 MHz, so many frames are lost.
inline constexpr uint8_t TX_ATTEMPTS = 50;
/// Attempts for the last pairing frame. The unit has already accepted us at that point, so
/// its confirmation is nice to have rather than required.
inline constexpr uint8_t PAIRING_CONFIRM_ATTEMPTS = 5;

enum DeviceType : uint8_t {
    DEVICE_TYPE_BROADCAST = 0x00,
    DEVICE_TYPE_MAIN_UNIT = 0x01,
    DEVICE_TYPE_REMOTE_CONTROL = 0x03,
};

enum FrameType : uint8_t {
    FRAME_SET_SPEED = 0x02,
    FRAME_SET_TIMER = 0x03,
    FRAME_NETWORK_JOIN_REQUEST = 0x04,
    FRAME_SET_SPEED_REPLY = 0x05,
    FRAME_NETWORK_JOIN_OPEN = 0x06,
    FRAME_FAN_SETTINGS = 0x07,  // Current settings, sent by the unit in reply to a speed command
    FRAME_NETWORK_JOIN_ACCEPTED = 0x0B,
    FRAME_NETWORK_JOIN_ACK = 0x0C,
    FRAME_QUERY_NETWORK = 0x0D,
};

enum FanSpeed : uint8_t {
    FAN_SPEED_AUTO = 0x00,
    FAN_SPEED_LOW = 0x01,
    FAN_SPEED_MEDIUM = 0x02,
    FAN_SPEED_HIGH = 0x03,
    FAN_SPEED_MAX = 0x04,
};

/// Everything needed to talk to a paired unit. Stored in flash as-is, so keep the layout stable.
struct FanPairingInfo {
    uint32_t network_id;
    uint8_t main_unit_id;
    uint8_t main_unit_type;
    uint8_t my_device_id;
};

/// A 16 byte Zehnder RF frame:
/// [0] receiver type  [1] receiver id  [2] sender type  [3] sender id
/// [4] ttl            [5] frame type   [6] parameter count  [7..15] parameters
class Frame {
public:
    using Bytes = std::array<uint8_t, FRAME_SIZE>;
    static constexpr size_t MAX_PARAMETERS = FRAME_SIZE - 7;

    Frame() = default;
    Frame(uint8_t rx_type, uint8_t rx_id, uint8_t tx_type, uint8_t tx_id, FrameType type,
          std::initializer_list<uint8_t> parameters = {});

    uint8_t rx_type() const { return this->bytes_[0]; }
    uint8_t rx_id() const { return this->bytes_[1]; }
    uint8_t tx_type() const { return this->bytes_[2]; }
    uint8_t tx_id() const { return this->bytes_[3]; }
    uint8_t type() const { return this->bytes_[5]; }
    uint8_t parameter(size_t index) const { return this->bytes_[7 + index]; }
    /// The first four parameters interpreted as a little-endian network id.
    uint32_t network_id() const;

    bool is_addressed_to(uint8_t type, uint8_t id) const { return this->rx_type() == type && this->rx_id() == id; }
    bool is_from(uint8_t type, uint8_t id) const { return this->tx_type() == type && this->tx_id() == id; }

    Bytes &bytes() { return this->bytes_; }
    const Bytes &bytes() const { return this->bytes_; }

private:
    Bytes bytes_{};
};

// =========================================================================
// 1. Low-Level nRF905 Radio Controller
// =========================================================================
class NRF905Controller : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                              spi::CLOCK_POLARITY_LOW,
                                              spi::CLOCK_PHASE_LEADING,
                                              spi::DATA_RATE_4MHZ> {
public:
    void set_pwr_pin(GPIOPin *pin) { this->pwr_pin_ = pin; }
    void set_ce_pin(GPIOPin *pin) { this->ce_pin_ = pin; }
    void set_txen_pin(GPIOPin *pin) { this->txen_pin_ = pin; }
    void set_dr_pin(GPIOPin *pin) { this->dr_pin_ = pin; }

    /// Initialise the pins and the radio. Returns false if the radio does not respond.
    bool setup();
    void dump_config();

    void set_mode_idle();
    void set_mode_receive();
    void set_mode_transmit();

    /// Set the address used for both transmitting and receiving. Leaves the radio idle.
    void set_network_address(uint32_t address);

    void write_tx_payload(const Frame &frame);
    /// Read a received frame, if there is one. Leaves the radio idle when a frame was read.
    std::optional<Frame> read_rx_payload();

private:
    static constexpr size_t CONFIG_SIZE = 10;
    static constexpr size_t CONFIG_RX_ADDRESS_OFFSET = 5;
    using Config = std::array<uint8_t, CONFIG_SIZE>;

    void write_config_();
    bool verify_config_();

    // Zehnder profile: 868.4 MHz, +10 dBm, auto retransmit, 4 byte addresses, 16 byte payloads,
    // 16 MHz crystal, CRC16. Bytes 5..8 hold the receive address.
    Config config_{0x76, 0x2E, 0x44, 0x10, 0x10, 0xA5, 0x5A, 0x5A, 0xA5, 0xDB};

    GPIOPin *pwr_pin_{nullptr};
    GPIOPin *ce_pin_{nullptr};
    GPIOPin *txen_pin_{nullptr};
    GPIOPin *dr_pin_{nullptr};
};


// =========================================================================
// 2. High-Level Fan Communication Protocol
// =========================================================================

enum class OperationType : uint8_t {
    SET_SPEED,
    PAIRING,
};

struct OperationResult {
    OperationType type;
    bool success;
    /// Only set for a successful pairing.
    std::optional<FanPairingInfo> pairing_info;
};

/// Non-blocking implementation of the Zehnder RF protocol. Start an operation, then call
/// loop() until it hands back the result.
class ZehnderFanProtocol {
public:
    explicit ZehnderFanProtocol(NRF905Controller &radio) : radio_(radio) {}

    bool is_busy() const { return this->step_ != Step::IDLE; }

    /// Both return false (and do nothing) when another operation is still in progress.
    bool start_pairing();
    bool start_set_speed(const FanPairingInfo &pairing_info, FanSpeed speed, uint8_t timer_minutes = 0);

    /// Advance the state machine. Returns the result once, when an operation finishes.
    std::optional<OperationResult> loop();

private:
    enum class Step : uint8_t {
        IDLE,
        SET_SPEED,
        PAIRING_DISCOVER,  // Ask units that are open for pairing to announce themselves
        PAIRING_JOIN,      // Ask the unit that answered to let us join its network
        PAIRING_CONFIRM,   // Acknowledge that we joined
    };
    enum class Phase : uint8_t {
        TRANSMITTING,
        LISTENING,
    };

    void begin_step_(Step step, const Frame &frame, uint8_t max_attempts = TX_ATTEMPTS);
    void transmit_();
    std::optional<OperationResult> handle_frame_(const Frame &frame);
    std::optional<OperationResult> handle_timeout_();
    OperationResult finish_(bool success);
    void log_ignored_(const Frame &frame) const;

    NRF905Controller &radio_;
    Step step_{Step::IDLE};
    Phase phase_{Phase::TRANSMITTING};
    uint32_t phase_start_{0};
    uint8_t attempt_{0};
    uint8_t max_attempts_{TX_ATTEMPTS};
    Frame tx_frame_;
    /// The unit we are talking to; filled in step by step while pairing.
    FanPairingInfo peer_{};
};


// =========================================================================
// 3. ESPHome Component
// =========================================================================

class ZehnderFanComponent : public fan::Fan, public Component {
public:
    explicit ZehnderFanComponent(NRF905Controller *radio) : radio_(radio), protocol_(*radio) {}

    void setup() override;
    void loop() override;
    void dump_config() override;

    fan::FanTraits get_traits() override;

    /// Discover a unit that is open for pairing (just powered on) and join its network.
    void start_pairing();
    /// Forget the unit we are paired with.
    void clear_pairing();
    /// Run at `speed` (1-4) for `minutes`, after which the unit returns to its previous speed
    /// by itself. The state of the fan entity is left untouched.
    void set_speed_with_timer(int speed, uint8_t minutes);

protected:
    /// What we want the unit to do.
    struct Target {
        bool state;
        int speed;
        uint8_t timer_minutes{0};
    };

    void control(const fan::FanCall &call) override;

    void start_next_operation_();
    void handle_result_(const OperationResult &result);

    void load_pairing_info_();
    std::optional<FanPairingInfo> load_legacy_pairing_info_();
    void save_pairing_info_(const FanPairingInfo &info);

    NRF905Controller *radio_;
    ZehnderFanProtocol protocol_;
    HighFrequencyLoopRequester high_freq_;

    ESPPreferenceObject pairing_pref_;
    std::optional<FanPairingInfo> pairing_info_;

    /// Most recent request that still has to be sent; newer requests replace older ones.
    std::optional<Target> queued_target_;
    /// Request that is being sent right now.
    std::optional<Target> active_target_;
    bool pairing_requested_{false};
};

} // namespace zehnder_fan
} // namespace esphome
