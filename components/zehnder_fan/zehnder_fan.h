#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan.h"

#include <optional>

namespace esphome {
namespace zehnder_fan {

// Constants extracted from the original fan.h and config.h
static const uint32_t NRF905_XTAL_FREQUENCY = 16000000;
static const uint8_t FAN_FRAMESIZE = 16;
static const uint8_t FAN_TX_FRAMES = 4;
static const uint8_t FAN_TX_RETRIES = 50;
static const uint32_t FAN_REPLY_TIMEOUT_MS = 500;
static const uint32_t NETWORK_LINK_ID = 0xA55A5AA5;

// Fan device types and commands
enum {
    FAN_TYPE_BROADCAST = 0x00,
    FAN_TYPE_MAIN_UNIT = 0x01,
    FAN_TYPE_REMOTE_CONTROL = 0x03,
};
enum {
    FAN_FRAME_SETSPEED = 0x02,
    FAN_FRAME_SETTIMER = 0x03,
    FAN_NETWORK_JOIN_REQUEST = 0x04,
    FAN_FRAME_SETSPEED_REPLY = 0x05,
    FAN_NETWORK_JOIN_OPEN = 0x06,
    FAN_TYPE_FAN_SETTINGS = 0x07,
    FAN_FRAME_0B = 0x0B,
    FAN_NETWORK_JOIN_ACK = 0x0C,
};
enum {
    FAN_SPEED_AUTO = 0x00,
    FAN_SPEED_LOW = 0x01,
    FAN_SPEED_MEDIUM = 0x02,
    FAN_SPEED_HIGH = 0x03,
    FAN_SPEED_MAX = 0x04,
};


struct FanPairingInfo {
    uint32_t network_id;
    uint8_t main_unit_id;
    uint8_t main_unit_type;
    uint8_t my_device_id;
};

// =========================================================================
// 1. Low-Level nRF905 Radio Controller
// =========================================================================
class NRF905Controller : public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                              spi::CLOCK_POLARITY_LOW,
                                              spi::CLOCK_PHASE_LEADING,
                                              spi::DATA_RATE_4MHZ> {
public:
    void setup_pins(GPIOPin *pwr_pin, GPIOPin *ce_pin, GPIOPin *txen_pin, GPIOPin *dr_pin);
    void set_cs_pin(GPIOPin *cs_pin) { this->cs_ = cs_pin; }
    bool init();
    void set_mode_idle();
    void set_mode_receive();
    void set_mode_transmit();

    void set_tx_address(uint32_t address);
    void set_rx_address(uint32_t address);

    void write_tx_payload(const uint8_t *payload, size_t size);
    bool read_rx_payload(uint8_t *buffer, size_t size);

    bool is_data_ready() { return this->dr_pin_->digital_read(); }

private:
    void write_config_registers(const uint8_t *config, size_t size);
    
    GPIOPin *pwr_pin_{nullptr};
    GPIOPin *ce_pin_{nullptr};
    GPIOPin *txen_pin_{nullptr};
    GPIOPin *dr_pin_{nullptr};
};


// =========================================================================
// 2. High-Level Fan Communication Protocol
// =========================================================================

enum class RadioOperationState {
    IDLE,
    TRANSMITTING,
    WAITING_RESPONSE,
    OPERATION_COMPLETE
};

enum class RadioOperationType {
    NONE,
    SET_SPEED,
    PAIRING_DISCOVER,
    PAIRING_JOIN,
    PAIRING_ACK
};

struct PendingOperation {
    RadioOperationType type;
    RadioOperationState state;
    uint32_t start_time;
    uint8_t retry_count;
    uint8_t max_retries;
    uint32_t timeout_ms;
    uint8_t tx_payload[FAN_FRAMESIZE];
    
    // Operation-specific data
    union {
        struct {
            FanPairingInfo pairing_info;
            uint8_t speed;
            uint8_t timer_minutes;
        } set_speed;
        
        struct {
            uint8_t my_device_id;
            FanPairingInfo current_info;
            uint8_t pairing_step; // 0=discover, 1=join, 2=ack
        } pairing;
    } data;
};

class ZehnderFanProtocol {
public:
    ZehnderFanProtocol(NRF905Controller *radio);

    // Async interface - returns immediately
    void start_pairing();
    void start_set_speed(const FanPairingInfo &pairing_info, uint8_t speed, uint8_t timer_minutes);
    
    // Process state machine - call from loop()
    void process();
    
    // Check if operation is complete
    bool is_operation_complete() const { return pending_op_.state == RadioOperationState::OPERATION_COMPLETE; }
    bool last_operation_successful() const { return last_operation_success_; }
    
    // Reset state machine for next operation
    void reset_operation_state();
    
    // Get pairing result if available
    std::optional<FanPairingInfo> get_pairing_result();

private:
    void start_transmit();
    void handle_response();
    void retry_or_fail();
    void complete_operation(bool success);
    
    // Pairing state machine helpers
    void setup_pairing_discover();
    void setup_pairing_join();
    void setup_pairing_ack();
    void handle_pairing_response();
    
    NRF905Controller *radio_;
    uint8_t rx_buffer_[FAN_FRAMESIZE]{0};
    PendingOperation pending_op_{};
    bool last_operation_success_{false};
    std::optional<FanPairingInfo> pairing_result_;
};


// =========================================================================
// 3. ESPHome Component
// =========================================================================

enum class ComponentOperationState {
    IDLE,
    SETTING_SPEED,
    PAIRING
};

class ZehnderFanComponent : public fan::Fan, public PollingComponent {
public:
    void setup() override;
    void loop() override;
    void dump_config() override;
    void update() override;

    fan::FanTraits get_traits() override;
    void control(const fan::FanCall &call) override;

    // Service function to initiate pairing
    void start_pairing();

    // Pin Setters from YAML
    void set_pwr_pin(GPIOPin *pin) { this->pwr_pin_ = pin; }
    void set_ce_pin(GPIOPin *pin) { this->ce_pin_ = pin; }
    void set_txen_pin(GPIOPin *pin) { this->txen_pin_ = pin; }
    void set_dr_pin(GPIOPin *pin) { this->dr_pin_ = pin; }
    void set_cs_pin(GPIOPin *pin) { this->cs_pin_ = pin; }
    void set_spi_parent(spi::SPIComponent *parent) { this->spi_parent_ = parent; }

protected:
    void save_pairing_info(const FanPairingInfo &info);
    bool load_pairing_info();
    void clear_pairing_info();
    
    void handle_operation_complete();

    NRF905Controller nrf_radio_;
    std::unique_ptr<ZehnderFanProtocol> fan_protocol_;
    
    // Pins from YAML
    GPIOPin *pwr_pin_;
    GPIOPin *ce_pin_;
    GPIOPin *txen_pin_;
    GPIOPin *dr_pin_;
    GPIOPin *cs_pin_;
    spi::SPIComponent *spi_parent_;

    std::optional<FanPairingInfo> pairing_info_;
    ComponentOperationState component_state_{ComponentOperationState::IDLE};
    
    // Pending fan call data
    bool pending_state_change_{false};
    bool pending_fan_state_{false};
    int pending_fan_speed_{1};
};

} // namespace zehnder_fan
} // namespace esphome