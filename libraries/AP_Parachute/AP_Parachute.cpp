#include "AP_Parachute.h"
#include <cstdlib>

#if HAL_PARACHUTE_ENABLED

#include <AP_Relay/AP_Relay.h>
#include <AP_Math/AP_Math.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_HAL/utility/Socket.h>
#include "AP_Parachute_I2C.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_Parachute::var_info[] = {

    // @Param: ENABLED
    // @DisplayName: Parachute release enabled or disabled
    // @Description: Parachute release enabled or disabled
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO_FLAGS("ENABLED", 0, AP_Parachute, _enabled, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: TYPE
    // @DisplayName: Parachute release mechanism type (relay or servo)
    // @Description: Parachute release mechanism type (relay or servo)
    // @Values: 0:First Relay,1:Second Relay,2:Third Relay,3:Fourth Relay,10:Servo,20:I2C
    // @User: Standard
    AP_GROUPINFO("TYPE", 1, AP_Parachute, _release_type, AP_PARACHUTE_TRIGGER_TYPE_RELAY_0),

    // @Param: SERVO_ON
    // @DisplayName: Parachute Servo ON PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_ON", 2, AP_Parachute, _servo_on_pwm, AP_PARACHUTE_SERVO_ON_PWM_DEFAULT),

    // @Param: SERVO_OFF
    // @DisplayName: Servo OFF PWM value
    // @Description: Parachute Servo PWM value in microseconds when parachute is not released
    // @Range: 1000 2000
    // @Units: PWM
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SERVO_OFF", 3, AP_Parachute, _servo_off_pwm, AP_PARACHUTE_SERVO_OFF_PWM_DEFAULT),

    // @Param: ALT_MIN
    // @DisplayName: Parachute min altitude in meters above home
    // @Description: Parachute min altitude above home.  Parachute will not be released below this altitude.  0 to disable alt check.
    // @Range: 0 32000
    // @Units: m
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("ALT_MIN", 4, AP_Parachute, _alt_min, AP_PARACHUTE_ALT_MIN_DEFAULT),

    // @Param: DELAY_MS
    // @DisplayName: Parachute release delay
    // @Description: Delay in millseconds between motor stop and chute release
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("DELAY_MS", 5, AP_Parachute, _delay_ms, AP_PARACHUTE_RELEASE_DELAY_MS),
    
    // @Param: SINK_CRT
    // @DisplayName: Critical sink speed rate in m/s to trigger emergency parachute
    // @Description: Release parachute when critical sink rate is reached
    // @Range: 0 15
    // @Units: m/s
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SINK_CRT", 6, AP_Parachute, _critical_sink, AP_PARACHUTE_CRITICAL_SINK_DEFAULT),

    // @Param: OPTIONS
    // @DisplayName: Parachute options
    // @Description: Optional behaviour for parachute
    // @Bitmask: 0:hold open forever after release
    // @User: Standard
    AP_GROUPINFO("OPTIONS", 7, AP_Parachute, _options, 0),

    // @Param: SINK_TIME
    // @DisplayName: When critical sink speed rate for more than critical sink time - trigger emergency parachute
    // @Description: Release parachute when critical sink rate is reached for more than critical sink time
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SINK_TIME", 8, AP_Parachute, _critical_sink_time, AP_PARACHUTE_CRITICAL_SINK_TIME_DEFAULT),

    // @Param: FLIP_CRT
    // @DisplayName: Critical flip degree to trigger emergency parachute
    // @Description: Release parachute when critical flip rate is reached
    // @Range: 0 180
    // @Units: degrees
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLIP_CRT", 9, AP_Parachute, _critical_flip, AP_PARACHUTE_CRITICAL_FLIP_DEFAULT),

    // @Param: FLIP_TIME
    // @DisplayName: When in critical flip for more than critical flip time - trigger emergency parachute
    // @Description: Release parachute when critical flip rate is reached for more than critical flip time
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("FLIP_TIME", 10, AP_Parachute, _critical_flip_time, AP_PARACHUTE_CRITICAL_FLIP_TIME_DEFAULT),

    // @Param: YAW_CRT
    // @DisplayName: Critical yaw rate in degrees/second to trigger emergency parachute
    // @Description: Release parachute when critical yaw rate is reached
    // @Range: 0 720
    // @Units: degrees/second
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_CRT", 11, AP_Parachute, _critical_yaw_rate, AP_PARACHUTE_CRITICAL_YAW_DEFAULT),

    // @Param: YAW_TIME
    // @DisplayName: When in critical yaw for more than critical yaw time - trigger emergency parachute
    // @Description: Release parachute when critical yaw rate is reached for more than critical yaw time
    // @Range: 0 5000
    // @Units: ms
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("YAW_TIME", 12, AP_Parachute, _critical_yaw_rate_time, AP_PARACHUTE_CRITICAL_YAW_RATE_TIME_DEFAULT),

    // @Param: SAFETY
    // @DisplayName: Parachute safety Switch
    // @Description: Parachute safety Switch
    // @Range: 0 1
    // @Units: bool
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("SAFETY", 13, AP_Parachute, _safety_switch, false),  // TODO: it's a temporary parameter for testing only. should be removed.

    // @Param: I2C_BUS_CLK
    // @DisplayName: I2C bus clock
    // @Description: I2C bus clock
    // @User: Standard
    AP_GROUPINFO("I2C_BUS_CLK", 14, AP_Parachute, _i2c_params.bus_clock, 100000),

    // @Param: I2C_CAP_NUM
    // @DisplayName: I2C number of capacitors
    // @Description: I2C number of capacitors
    // @User: Standard
    AP_GROUPINFO("I2C_CAP_NUM", 15, AP_Parachute, _i2c_params.capacitors, 2),

    // @Param: I2C_BUS_V
    // @DisplayName: I2C minimum bus voltage
    // @Description: I2C minimum bus voltage
    // @User: Standard
    AP_GROUPINFO("I2C_BUS_V", 16, AP_Parachute, _i2c_params.min_bus_voltage, 4.8),

    // @Param: TST_V
    // @DisplayName: I2C AIN0/1 test expected voltage
    // @Description: I2C AIN0/1 test expected voltage
    // @User: Standard
    AP_GROUPINFO("I2C_TST_V", 17, AP_Parachute, _i2c_params.ain_test_expected_voltage, 2.2),

    // @Param: TST_TRL
    // @DisplayName: I2C AIN0/1 test expected tolerance
    // @Description: I2C AIN0/1 test expected tolerance
    // @User: Standard
    AP_GROUPINFO("I2C_TST_TLR", 18, AP_Parachute, _i2c_params.ain_test_expected_tolerance, 0.3),

    // @Param: RSNSC
    // @DisplayName: The value of the sense resistor
    // @Description: The value of the sense resistor
    // @Units: Ohm
    // @User: Standard
    AP_GROUPINFO("I2C_RSNSC", 19, AP_Parachute, _i2c_params.rsnsc, 0.006),

    // @Param: CAP_RT
    // @DisplayName: The resistor on the RT pin
    // @Description: The resistor on the RT pin
    // @Units: KOhm
    // @User: Standard
    AP_GROUPINFO("I2C_CAP_RT", 20, AP_Parachute, _i2c_params.rt, 71.5),

    // @Param: CAP_RTST
    // @DisplayName: The resistor on the ITST pin
    // @Description: The resistor on the ITST pin
    // @Units: KOhm
    // @User: Standard
    AP_GROUPINFO("I2C_CAP_RTST", 21, AP_Parachute, _i2c_params.rtst, 0.121),

    // @Param: I2C_SIMULATE
    // @DisplayName: I2C parachute simulation only. will not really release parachute. use with caution.
    // @Description: I2C parachute simulation only. will not really release parachute. use with caution.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("I2C_SIMULATE", 22, AP_Parachute, _i2c_params.simulate, 0),

    // @Param: I2C_SIMULATE
    // @DisplayName: ignore capacitance tests.
    // @Description: ignore capacitance tests.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("I2C_CAP_IGNR", 23, AP_Parachute, _i2c_params.ignore_cap_tests, 0),

    
    AP_GROUPEND
};

/// enabled - enable or disable parachute release
void AP_Parachute::enabled(bool on_off)
{
#ifdef FTS
    if (!on_off) {
        GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "FTS mode. keeping the parachute %s", _enabled ? "ENABLED" : "DISABLED");
        return;
    }
#endif
    _enabled = on_off;

    // clear release_time
    _release_time = 0;

    AP::logger().Write_Event(_enabled ? LogEvent::PARACHUTE_ENABLED : LogEvent::PARACHUTE_DISABLED);
}

void AP_Parachute::safety_switch(bool on_off)
{
    if (is_safety_switch_on() == on_off) {
        return;  // nothing need to be changed
    }

    _safety_switch = on_off ? 1 : 0;  // _safety_switch = on_off;
    gcs().send_text(MAV_SEVERITY_INFO, "Safety Switch: %s", on_off ? "ENGAGED" : "DISENGAGED");
}

/// release - release parachute
void AP_Parachute::release()
{
    // exit immediately if safety switch is off
    if (!is_safety_switch_on()) {
        return;
    }

    // exit immediately if not enabled
    if (_enabled <= 0) {
        return;
    }

    if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_I2C) {
        if (!get_i2c_parachute()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "should release I2C parachute, but it's uninitilized");
            return;
        }
        if (!get_i2c_parachute()->release_prepare()) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "failed to trigger the parachute");
            return;  // we failed to trigger the parachute so we'll try again in the next call to update()
        }
    }


#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    struct pwm_packet {
        uint16_t pwm[16];
    } pwm_pkt = { 0 };

    char *rc_port = getenv("SITL_RCIN_PORT");
    if (rc_port != NULL) {
        SocketAPM device { true };
        pwm_pkt.pwm[15] = 2000;  // channel 16 need to be HIGH (> 1800)
        device.sendto((const uint8_t *)&pwm_pkt, sizeof(pwm_pkt), "127.0.0.1", (uint16_t) atoi(rc_port));
    }
#endif

    gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Released");
    AP::logger().Write_Event(LogEvent::PARACHUTE_RELEASED);

    // set release time to current system time
    if (_release_time == 0) {
        _release_time = AP_HAL::millis();
    }

    _release_initiated = true;

    // update AP_Notify
    AP_Notify::flags.parachute_release = 1;
}

// move servo to off position
void AP_Parachute::reset()
{
    if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
        // move servo back to off position
        SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_off_pwm);
    }
    else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
        // set relay back to zero volts
        _relay.off(_release_type);
    }
    // reset released flag and release_time
    _release_in_progress = false;
    _release_time = 0;
    _release_initiated = false;
    // update AP_Notify
    AP_Notify::flags.parachute_release = 0;
}

bool AP_Parachute::_release_by_condition(const AP_Float& critical_condition, const AP_Float& critical_condition_time, const float& current_condition_value, uint32_t& condition_time)
{
    uint32_t time = AP_HAL::millis();
    if ((critical_condition > 0) && (current_condition_value > critical_condition) && !_release_initiated) {
        if (condition_time == 0) {
            condition_time = time;
        }
        if ((time - condition_time) >= critical_condition_time) {
            release();
            return true;
        }
    } else {
        condition_time = 0;
    }
    return false;
}

void AP_Parachute::get_axes_values(int32_t &yaw_rate, int32_t &pitch, int32_t &roll)
{
    const AP_AHRS &ahrs_yb = AP::ahrs();
    yaw_rate = labs(roundf(ToDeg(ahrs_yb.get_yaw_rate_earth())));
    pitch = labs(roundf(ahrs_yb.pitch_sensor / 100.0)); // attitude pitch in degrees
    roll = labs(roundf(ahrs_yb.roll_sensor / 100.0));   // attitude roll in degrees
    const AP_AHRS &ahrs = AP::ahrs();
    const AP_InertialSensor &ins = AP::ins();
    const uint8_t active_cores = ahrs.get_active_core_count();
    if (active_cores > 1) { 
        int32_t yaw_rate_values[active_cores] = { 0 };
        int32_t pitch_values[active_cores] = { 0 };
        int32_t roll_values[active_cores] = { 0 };
        for (uint8_t core = 0; core < active_cores; ++core) {
            yaw_rate_values[core] = INT32_MIN;
            pitch_values[core] = INT32_MIN;
            roll_values[core] = INT32_MIN;
            if (core < ins.get_gyro_count()) {
                yaw_rate_values[core] = labs(roundf(ToDeg(ins.get_gyro(core) * ahrs_yb.get_rotation_body_to_ned().c)));
            }
            Vector3f eulers;
            ahrs.get_euler_angles(core, eulers);
            pitch_values[core] = labs(roundf(degrees(eulers.y)));
            roll_values[core] = labs(roundf(degrees(eulers.x)));               
        }

        auto int32_compare = [](const void* a, const void* b) -> int
        {
            int32_t arg1 = *static_cast<const int32_t*>(a);
            int32_t arg2 = *static_cast<const int32_t*>(b);
            return (arg1 > arg2) - (arg1 < arg2);
        };
        std::qsort(yaw_rate_values, ARRAY_SIZE(yaw_rate_values), sizeof(yaw_rate_values[0]), int32_compare);
        std::qsort(pitch_values, ARRAY_SIZE(pitch_values), sizeof(pitch_values[0]), int32_compare);
        std::qsort(roll_values, ARRAY_SIZE(roll_values), sizeof(roll_values[0]), int32_compare);

        if (yaw_rate_values[ARRAY_SIZE(yaw_rate_values) - 2] > INT32_MIN) {
            yaw_rate = yaw_rate_values[ARRAY_SIZE(yaw_rate_values) - 2];
        }
        if (pitch_values[ARRAY_SIZE(pitch_values) - 2] > INT32_MIN) {
            pitch = pitch_values[ARRAY_SIZE(pitch_values) - 2];
        }
        if (roll_values[ARRAY_SIZE(roll_values) - 2] > INT32_MIN) {
            roll = roll_values[ARRAY_SIZE(roll_values) - 2];
        }
    }
}


/// update - shuts off the trigger should be called at about 10hz
void AP_Parachute::update()
{
    // exit immediately if safety switch is off
    if (!is_safety_switch_on()) {
        return;
    }

    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0) {
        return;
    }

    if (!_release_initiated) {
        const uint32_t time = AP_HAL::millis();
        int32_t yaw_rate, pitch, roll;
        get_axes_values(yaw_rate, pitch, roll);

        if (_release_by_condition(_critical_yaw_rate, _critical_yaw_rate_time, yaw_rate, _yaw_time)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "yaw_rate %" PRId32 ", time %" PRId32 " ms", yaw_rate, time - _yaw_time);
        }

        if (_release_by_condition(_critical_flip, _critical_flip_time, pitch, _pitch_time)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pitch %" PRId32 ", critical angle %d, time %" PRIu32 " ms", pitch, (int)_critical_flip, time - _pitch_time);
        }

        if (_release_by_condition(_critical_flip, _critical_flip_time, roll, _roll_time)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "roll %" PRId32 ", critical angle %d, time %" PRId32 " ms", roll, (int)_critical_flip, time - _roll_time);
        }

        if (_release_by_condition(_critical_sink, _critical_sink_time, _sink_rate, _sink_time_ms)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "sink %f, time %" PRId32 " ms", _sink_rate, time - _sink_time_ms);
        }
    }

    // calc time since release
    uint32_t time_diff = AP_HAL::millis() - _release_time;
    uint32_t delay_ms = _delay_ms<=0 ? 0: (uint32_t)_delay_ms;

    bool hold_forever = (_options.get() & uint32_t(Options::HoldOpen)) != 0;

    // check if we should release parachute
    if ((_release_time != 0) && !_release_in_progress) {
        if (time_diff >= delay_ms) {
            if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
                // move servo
                SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_on_pwm);
            } else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
                // set relay
                _relay.on(_release_type);
            } else if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_I2C) {
                if (!get_i2c_parachute()) {
                    gcs().send_text(MAV_SEVERITY_CRITICAL, "should release I2C parachute, but it's uninitilized");
                    return;
                }
                if (!get_i2c_parachute()->release()) {
                    return;  // we failed to trigger the parachute so we'll try again in the next call to update()
                }
            }
            _release_in_progress = true;
            _released = true;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL,"Parachute: Released");
        }
    } else if ((_release_time == 0) ||
               (!hold_forever && time_diff >= delay_ms + AP_PARACHUTE_RELEASE_DURATION_MS)) {
        if (_release_time == 0) {
            reset();
        }

        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            // move servo back to off position
            SRV_Channels::set_output_pwm(SRV_Channel::k_parachute_release, _servo_off_pwm);
        } else if (_release_type <= AP_PARACHUTE_TRIGGER_TYPE_RELAY_3) {
            // set relay back to zero volts
            _relay.off(_release_type);
        }
        // reset released flag and release_time
        _release_in_progress = false;
        _release_time = 0;
        // update AP_Notify
        AP_Notify::flags.parachute_release = 0;
    }
}

bool AP_Parachute::is_reboot_necessary()
{
    static bool should_reboot_when_externally_powered = false;

    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0) {
        should_reboot_when_externally_powered = false;
        return false;
    }

    // exit immediately if parachute is not i2c
    if (_release_type != AP_PARACHUTE_TRIGGER_TYPE_I2C) {
        should_reboot_when_externally_powered = false;
        return false;
    }

    // exit immediately if safety switch is on
    if (is_safety_switch_on()) {
        should_reboot_when_externally_powered = false;
        return false;
    }

    if (get_i2c_parachute()) {
        if (get_i2c_parachute()->is_in_backup_mode()) {
            should_reboot_when_externally_powered = true;
        }
        if ((!get_i2c_parachute()->is_in_backup_mode()) && (should_reboot_when_externally_powered)) {
            return true;
        }
    }

    return false;
}

void AP_Parachute::play_tone(uint32_t duration_ms)
{
    if (get_i2c_parachute()) {
        get_i2c_parachute()->play_tone(duration_ms);
    }
}

bool AP_Parachute::inflight_tests()
{
    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0) {
        return true;
    }

    // exit immediately if parachute is not i2c
    if (_release_type != AP_PARACHUTE_TRIGGER_TYPE_I2C) {
        return true;
    }

    if (get_i2c_parachute()) {
        if (get_i2c_parachute()->is_in_backup_mode() && is_safety_switch_on()) {
            get_i2c_parachute()->buzzer(true);
        }

        if (is_reboot_necessary()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Externally powered now. rebooting.");
            hal.scheduler->delay(1000);
            hal.scheduler->reboot(false);
        }

        return get_i2c_parachute()->get_inflight_tests_results();
    }

    gcs().send_text(MAV_SEVERITY_ERROR, "parachute uninitialized");
    return false;
}

bool AP_Parachute::prearm_healthy(char *failure_msg, const uint8_t failure_msg_len)
{
    // exit immediately if not enabled or parachute not to be released
    if (_enabled <= 0) {
        return true;
    }

    // exit immediately if parachute is not i2c
    if (_release_type != AP_PARACHUTE_TRIGGER_TYPE_I2C) {
        return true;
    }

    if (!get_i2c_parachute()) {
        strncpy(failure_msg, "Can't get i2c_parachute", failure_msg_len);
        return false;
    }

    return get_i2c_parachute()->preflight_tests(failure_msg, failure_msg_len);
}

// bit index 0 - safety switch, bit index 1 - FTS healthy, bit index 2 - parachute triggered
uint32_t AP_Parachute::mode()
{
    uint32_t _mode = 0;
    if (is_safety_switch_on()) {
        _mode |= 0x01;
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    _mode |= 0x02;
#endif
    if (get_i2c_parachute() && get_i2c_parachute()->healthy()) {
        _mode |= 0x02;
    }
    if (_release_initiated) {
        _mode |= 0x04;
    }
    return _mode;
}

AP_Parachute_I2C *AP_Parachute::get_i2c_parachute()
{
    // Initializing I2C parachute if needed
    if((!_i2c_parachute) && (_release_type == AP_PARACHUTE_TRIGGER_TYPE_I2C)) {
        _i2c_parachute = AP_Parachute_I2C::detect(_i2c_params);
    }
    return _i2c_parachute;
}

// set_sink_rate - set vehicle sink rate
void AP_Parachute::set_sink_rate(float sink_rate)
{
    _sink_rate = sink_rate;
}

// check settings are valid
bool AP_Parachute::arming_checks(size_t buflen, char *buffer) const
{
    if (_enabled > 0) {
        if (_release_type == AP_PARACHUTE_TRIGGER_TYPE_SERVO) {
            if (!SRV_Channels::function_assigned(SRV_Channel::k_parachute_release)) {
                hal.util->snprintf(buffer, buflen, "Chute has no channel");
                return false;
            }
        } else if (!_relay.enabled(_release_type)) {
            hal.util->snprintf(buffer, buflen, "Chute invalid relay %d", int(_release_type));
            return false;
        }
        if (_release_initiated) {
            hal.util->snprintf(buffer, buflen, "Chute is released");
            return false;
        }
    }
    return true;
}

// singleton instance
AP_Parachute *AP_Parachute::_singleton;

namespace AP {

AP_Parachute *parachute()
{
    return AP_Parachute::get_singleton();
}

}
#endif // HAL_PARACHUTE_ENABLED
