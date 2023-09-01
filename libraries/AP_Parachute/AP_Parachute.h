/// @file   AP_Parachute.h
/// @brief  Parachute release library
#pragma once

#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_Common.h>
#include <AP_Relay/AP_Relay.h>
#include <GCS_MAVLink/GCS.h>

#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_0       0
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_1       1
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_2       2
#define AP_PARACHUTE_TRIGGER_TYPE_RELAY_3       3
#define AP_PARACHUTE_TRIGGER_TYPE_SERVO         10
#define AP_PARACHUTE_TRIGGER_TYPE_I2C           20

#define AP_PARACHUTE_RELEASE_DELAY_MS           0      // delay in milliseconds between call to release() and when servo or relay actually moves.  Allows for warning to user
#define AP_PARACHUTE_RELEASE_DURATION_MS       2000    // when parachute is released, servo or relay stay at their released position/value for 2000ms (2seconds)

#define AP_PARACHUTE_SERVO_ON_PWM_DEFAULT      1300    // default PWM value to move servo to when shutter is activated
#define AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT     1100    // default PWM value to move servo to when shutter is deactivated

#define AP_PARACHUTE_ALT_MIN_DEFAULT            10     // default min altitude the vehicle should have before parachute is released

#define AP_PARACHUTE_CRITICAL_SINK_DEFAULT           5    // default critical sink speed in m/s to trigger emergency parachute
#define AP_PARACHUTE_CRITICAL_SINK_TIME_DEFAULT      250  // default critical sink time in ms to trigger emergency parachute
#define AP_PARACHUTE_CRITICAL_FLIP_DEFAULT           60   // default critical flip in degrees to trigger emergency parachute
#define AP_PARACHUTE_CRITICAL_FLIP_TIME_DEFAULT      250  // default critical flip time in ms to trigger emergency parachute
#define AP_PARACHUTE_CRITICAL_YAW_DEFAULT            300  // default critical yaw rate in degrees/second to trigger emergency parachute
#define AP_PARACHUTE_CRITICAL_YAW_RATE_TIME_DEFAULT  250  // default critical yaw rate time in ms to trigger emergency parachute

#ifndef HAL_PARACHUTE_ENABLED
// default to parachute enabled to match previous configs
#define HAL_PARACHUTE_ENABLED 1
#endif

#if HAL_PARACHUTE_ENABLED

class AP_Parachute_I2C;

/// @class  AP_Parachute
/// @brief  Class managing the release of a parachute
class AP_Parachute {

public:
    /// Constructor
    AP_Parachute(AP_Relay &relay)
        : _relay(relay), _i2c_parachute(nullptr)
    {
        // setup parameter defaults
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        if (_singleton != nullptr) {
            AP_HAL::panic("Parachute must be singleton");
        }
#endif
        _singleton = this;
        AP_Param::setup_object_defaults(this, var_info);
    }

    /* Do not allow copies */
    AP_Parachute(const AP_Parachute &other) = delete;
    AP_Parachute &operator=(const AP_Parachute&) = delete;

    /// safety_switch - enable or disable parachute safety switch. if (and only if) safety switch is on - parachute is allowed to be released
    void safety_switch(bool on_off);

    /// is_safety_switch_on - returns true if safety switch is on. if (and only if) safety switch is on - parachute is allowed to be released
    bool is_safety_switch_on() const { return (_safety_switch != 0 ? true : false);  /*return _safety_switch;*/ }

    /// enabled - enable or disable parachute release
    void enabled(bool on_off);

    /// enabled - returns true if parachute release is enabled
    bool enabled() const { return _enabled; }

    // reset the parachute to unreleased state
    void reset();

    // output the current yaw_rate, pitch and roll
    void get_axes_values(int32_t &yaw_rate, int32_t &pitch, int32_t &roll);

    bool is_reboot_necessary();

    /// release - release parachute
    void release();

    /// released - true if the parachute has been released (or release is in progress)
    bool released() const { return _released; }

    /// release_initiated - true if the parachute release sequence has been initiated (may wait before actual release)
    bool release_initiated() const { return _release_initiated; }

    /// release_in_progress - true if the parachute release sequence is in progress
    bool release_in_progress() const { return _release_in_progress; }

    /// update - shuts off the trigger should be called at about 10hz
    void update();

    /// alt_min - returns the min altitude above home the vehicle should have before parachute is released
    ///   0 = altitude check disabled
    int16_t alt_min() const { return _alt_min; }

    /// set_is_flying - accessor to the is_flying flag
    void set_is_flying(const bool is_flying) { _is_flying = is_flying; }

    // set_sink_rate - set vehicle sink rate
    void set_sink_rate(float sink_rate);

     // parachute tests running after arming
    bool inflight_tests();

    void play_tone(uint32_t duration_ms);

    // prearm checks
    bool prearm_healthy(char *failure_msg, const uint8_t failure_msg_len);

    // A bitfield for use for parachute mode
    // bit index 0 - safety switch, bit index 1 - FTS healthy, bit index 2 - parachute triggered
    uint32_t mode();

    // check settings are valid
    bool arming_checks(size_t buflen, char *buffer) const;
    
    static const struct AP_Param::GroupInfo        var_info[];

    // get singleton instance
    static AP_Parachute *get_singleton() { return _singleton; }

private:
    static AP_Parachute *_singleton;
    // Parameters
    AP_Int8     _enabled;       // 1 if parachute release is enabled
    AP_Int8     _release_type;  // 0:Servo,1:Relay
    AP_Int16    _servo_on_pwm;  // PWM value to move servo to when shutter is activated
    AP_Int16    _servo_off_pwm; // PWM value to move servo to when shutter is deactivated
    AP_Int16    _alt_min;       // min altitude the vehicle should have before parachute is released
    AP_Int16    _delay_ms;      // delay before chute release for motors to stop
    AP_Float    _critical_sink;      // critical sink rate to trigger emergency parachute
    AP_Float    _critical_sink_time; // critical sink time to trigger emergency parachute
    AP_Float    _critical_flip;      // critical flip rate to trigger emergency parachute
    AP_Float    _critical_flip_time; // critical flip time to trigger emergency parachute
    AP_Float    _critical_yaw_rate;       // critical yaw rate to trigger emergency parachute
    AP_Float    _critical_yaw_rate_time;  // critical yaw time to trigger emergency parachute

    // internal variables
    AP_Relay   &_relay;         // pointer to relay object from the base class Relay.
    uint32_t    _release_time;  // system time that parachute is ordered to be released (actual release will happen 0.5 seconds later)
    bool        _release_initiated:1;    // true if the parachute release initiated (may still be waiting for engine to be suppressed etc.)
    bool        _release_in_progress:1;  // true if the parachute release is in progress
    bool        _released:1;             // true if the parachute has been released
    bool        _is_flying:1;            // true if the vehicle is flying
    float       _sink_rate;              // vehicle sink rate in m/s
    uint32_t    _sink_time_ms;           // system time that the vehicle exceeded critical sink rate

    enum class Options : uint8_t {
        HoldOpen = (1U<<0),
    };

    AP_Int32    _options;
    uint32_t    _pitch_time;              // time that the vehicle exceeded critical pitch rate
    uint32_t    _roll_time;              // time that the vehicle exceeded critical roll rate
    uint32_t    _yaw_time;               // time that the vehicle exceeded critical yaw rate
    AP_Int8     _safety_switch;          // true if parachute safety switch is on (aka parachute allowed to be released)

    AP_Parachute_I2C *_i2c_parachute;

    struct _i2c_params_t
    {
        AP_Int32    bus_clock;
        AP_Int8     capacitors;
        AP_Float    min_bus_voltage;
        AP_Float    ain_test_expected_voltage;
        AP_Float    ain_test_expected_tolerance;
        AP_Float    rsnsc;
        AP_Float    rt;
        AP_Float    rtst;
        AP_Int8     simulate;
        AP_Int8     ignore_cap_tests;
    } _i2c_params;

    friend class AP_Parachute_I2C;

    // get a condition critical value (yaw, tilt, sink, etc.), it's max time allowed in critical condition, it's current value and it's time in critical condition
    // returns true if parachute is released by condition and false otherwise
    bool _release_by_condition(const AP_Float& critical_condition, const AP_Float& critical_condition_time, const float& current_condition_value, uint32_t& condition_time);

    // get pointer to i2c parachute if availiable (nullptr otherwise)
    AP_Parachute_I2C *get_i2c_parachute();
};

namespace AP {
    AP_Parachute *parachute();
};

#endif // HAL_PARACHUTE_ENABLED
