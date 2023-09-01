#pragma once

#include <AP_HAL/I2CDevice.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Parachute/AP_Parachute.h>

#define MAX_CAPACITORS      4

class AP_Parachute_I2C
{
public:
    // static detection function
    static AP_Parachute_I2C *detect(AP_Parachute::_i2c_params_t& params);

    // release the parachute
    bool release();

    // prepare to release the parachute (enable motor_cutoff and buzzer)
    bool release_prepare();

    // pre flight tests
    bool preflight_tests(char *failure_msg, const uint8_t failure_msg_len);

    // is i2c_parachute healthy
    bool healthy() { return _healthy; }

    bool is_in_backup_mode() { return _backup_mode; }

    bool get_inflight_tests_results() { return _inflight_tests_results; }

    bool buzzer(bool on_off);

    void play_tone(uint32_t duration_ms);

    /// reads data from i2c. should be called at about 10hz.
    void update();

    // void debug_i2c_channels(int8_t channels);

private:
    typedef enum adc_input_e {
        AIN0 = 0x04,
        AIN1 = 0x05,
        AIN2 = 0x06,
        AIN3 = 0x07,
    } adc_input_t;

    AP_Parachute_I2C(AP_HAL::OwnPtr<AP_HAL::I2CDevice> adc_dev, AP_HAL::OwnPtr<AP_HAL::I2CDevice> io_expander_dev, AP_HAL::OwnPtr<AP_HAL::I2CDevice> supercap_dev, AP_HAL::OwnPtr<AP_HAL::I2CDevice> extender_dev, AP_Parachute::_i2c_params_t& params) :
                     _adc_dev(std::move(adc_dev)), _io_expander_dev(std::move(io_expander_dev)), _supercap_dev(std::move(supercap_dev)), _extender_dev(std::move(extender_dev)), _params(params), _backup_mode(false),
                     _bus_voltage(0), _pyro_current(0), _pyro_short_current(0), _inflight_tests_results(false), _state{} { }

    bool init();
    bool init_io_expander();

    void write_to_log(MAV_SEVERITY severity, const char *fmt, va_list arg_list, bool force_write=false);
    void write_to_log(MAV_SEVERITY severity, const char *fmt, ...);
    void write_to_log(bool force_write, MAV_SEVERITY severity, const char *fmt, ...);

    bool write_reg(AP_HAL::OwnPtr<AP_HAL::I2CDevice>& dev, const uint8_t reg, const uint8_t* value, const uint8_t value_size);
    bool read_reg(AP_HAL::OwnPtr<AP_HAL::I2CDevice>& dev, const uint8_t reg, const uint8_t value_size, uint8_t* value);

    bool io_expander_write_value(const uint8_t value);
    bool io_expander_read_value(uint8_t& value);
    bool supercap_write_reg(const uint8_t reg, const int16_t value);
    bool supercap_read_reg(const uint8_t reg, int16_t& value);
    bool adc_write_config(const int16_t value);
    bool adc_read_reg(const uint8_t reg, int16_t& value);
    bool adc_read_config(int16_t& value);
    bool adc_read_value(int16_t& value);
    bool adc_read_input(adc_input_t in, int16_t& value);

    bool loopback_tests();
    bool supercap_capacity_tests();
    uint8_t get_capacitors_number();
    bool start_cap_esr_test();
    bool stop_cap_esr_test();
    bool pyro_tests();
    bool extender_tests();
    bool is_supercap_status_valid();

    // in flight tests wrapper
    void inflight_tests_wrapper();

    // in flight tests
    bool inflight_tests();

    void write_log();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _adc_dev;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _io_expander_dev;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _supercap_dev;
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _extender_dev;

    uint8_t _io_expander_current_value;

    HAL_Semaphore _log_write_semaphore;

    bool _cap_esr_test_started;
    bool _healthy;

    bool _backup_mode;

    bool _inflight_tests_results;

    float _bus_voltage;
    float _pyro_current;
    float _pyro_short_current;

    struct parachute_i2c_state {
        float vcap_single[MAX_CAPACITORS];
        float gpi;
        float vin;
        float vcap_total;
        float vout;
        float iin;
        float ichg;
        float dtemp;
        uint64_t frozen_since_us;
        uint64_t last_frozen_error_us;
        uint64_t last_updated_us;
    };

    struct parachute_i2c_state _state;          // current i2c regisater state

    uint32_t    _play_tone_start_ms;            // time that the last tone play has started
    uint32_t    _play_tone_duration_ms;         // duration to play the tone for (0 means disabled)

    AP_Parachute::_i2c_params_t& _params;
};
