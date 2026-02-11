/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "AP_Periph.h"

#if AP_PERIPH_FSO_POWERSTACK_ENABLED

#include <dronecan_msgs.h>

#define FSO_BATT_VOLTS_DIFF_MAX             1.5     // Maximum difference in voltage between batteries to allow power on
#define FSO_PAYLOAD_1_VOLT_DEFAULT          12.0    // Default voltage setting of Payload BEC 1
#define FSO_PAYLOAD_2_VOLT_DEFAULT          5.0     // Default voltage setting of Payload BEC 2
#define FSO_FAN_ERROR_HZ_MIN                100.0   // Minimum tachometer reading for the standard fan
#define FSO_OPTIONS_DEFAULT                 4       // Default options: PAYLOAD_HV OFF PAYLOAD_BEC ON

#define FSO_OVER_CURRENT_TC                 1.0     // Maximum current of the HV payload output
#define FSO_PAYLOAD_BEC_CURRENT_MAX         10.0    // Maximum current of the payload BEC's
#define FSO_PAYLOAD_BEC_CURRENT_FUSE        11.25   // Maximum current of the payload BEC's
#define FSO_PAYLOAD_HV_CURRENT_MAX          25.0    // Maximum current of the HV payload output
#define FSO_PAYLOAD_HV_CURRENT_FUSE         27.5    // Maximum current of the HV payload output
#define FSO_INTERNAL_BEC_HC_CURRENT_MAX     5.0     // Maximum current of the high current internal BEC
#define FSO_INTERNAL_BEC_CURRENT_MAX        1.75    // Maximum current of the low current internal BEC's

#define FSO_MAIN_TEMPERATURE_MAX            95.0    // Maximum temperature of the main power disribution board
#define FSO_BEC_HC_TEMPERATURE_MAX          90.0    // Maximum temperature of the high current BEC's
#define FSO_INTERNAL_BEC_TEMPERATURE_MAX    75.0    // Maximum temperature of the payload BEC's

#define FSO_C1_DEFAULT                      24.5314 // Payload BEC calibration coefficient 1
#define FSO_C2_DEFAULT                      0.16154 // Payload BEC calibration coefficient 2

#define FSO_OUT_VOLTS_DIFF_MAX              2.0     // Maximum difference between output and batteries before turn on (check this)
#define FSO_PRECHARGE_TIMEOUT_MS            500     // Maximum pre-charge time before turn on is aborted
#define FSO_SWITCH_ON_TIME_MS               500     // Minimum press time to turn on
#define FSO_SWITCH_OFF_TIME_MS              1000    // Minimum press time to turn off
#define FSO_LOOP_TIME_MS                    100     // Loop time in ms, must be the same as battery read period

#define FSO_MAIN_ERROR_MSG_INTERVAL         5000    // Interval of main temperature error messages
#define FSO_ERROR_FAN_MSG_INTERVAL          10000   // Interval of fan RPM error messages
#define FSO_ERROR_MSG_INTERVAL              15000   // Interval of current and temperature error messages

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo FSOPowerStack::var_info[] {

    // 1 was DAC_ - that is now using DAC from AP_Periph's Parameters.cpp

    // @Param: _OPTIONS
    // @DisplayName: FSO Options
    // @Description: FSO Options
    // @Bitmask: 0:Debug, 1:PAYLOAD_HV_ON, 2:PAYLOAD_BEC_ON
    AP_GROUPINFO("_OPTIONS", 2, FSOPowerStack, options, FSO_OPTIONS_DEFAULT),

    // @Param: _BAT_OFF_MAX
    // @DisplayName: Payload 1 voltage
    // @Description: Payload 1 BEC voltage setting
    // @Range: 1.0 2.0
    AP_GROUPINFO("_BAT_OFF_MAX", 3, FSOPowerStack, battery_diff_max, FSO_BATT_VOLTS_DIFF_MAX),

    // @Param: _BEC1_VOLT
    // @DisplayName: Payload 1 voltage
    // @Description: Payload 1 BEC voltage setting
    // @Range: 0 100
    AP_GROUPINFO("_BEC1_VOLT", 4, FSOPowerStack, payload_1_voltage, FSO_PAYLOAD_1_VOLT_DEFAULT),

    // @Param: _BEC2_VOLT
    // @DisplayName: Payload 2 voltage
    // @Description: Payload 2 BEC voltage setting
    // @Range: 0 100
    AP_GROUPINFO("_BEC2_VOLT", 5, FSOPowerStack, payload_2_voltage, FSO_PAYLOAD_2_VOLT_DEFAULT),

    // @Param: _BEC1_AMP_LIM
    // @DisplayName: Payload 1 current limit
    // @Description: Payload 1 current limit in Amps
    // @Range: 0 10
    AP_GROUPINFO("_BEC1_AMP_LIM", 6, FSOPowerStack, payload_1_current_max, FSO_PAYLOAD_BEC_CURRENT_MAX),

    // @Param: _BEC2_AMP_LIM
    // @DisplayName: Payload 2 current limit
    // @Description: Payload 2 current limit in Amps
    // @Range: 0 10
    AP_GROUPINFO("_BEC2_AMP_LIM", 7, FSOPowerStack, payload_2_current_max, FSO_PAYLOAD_BEC_CURRENT_MAX),

    // @Param: _HV_AMP_LIM
    // @DisplayName: Payload HV current limit
    // @Description: Payload HV current limit in Amps
    // @Range: 0 25
    AP_GROUPINFO("_HV_AMP_LIM", 8, FSOPowerStack, payload_HV_current_max, FSO_PAYLOAD_HV_CURRENT_MAX),

    // @Param: _BEC_TEMP_LIM
    // @DisplayName: Maximum temperature of BEC circuits
    // @Description: Maximum temperature of BEC circuits before shutdown or warning
    // @Range: 0 100
    AP_GROUPINFO("_BEC_TEMP_LIM", 9, FSOPowerStack, bec_temperature_max, FSO_BEC_HC_TEMPERATURE_MAX),

    // @Param: _FAN_1_MIN
    // @DisplayName: Fan 1 tachometer Alarm
    // @Description: Alarm threshold for minimum fan 1 tachometer in Hz
    // @Range: 0 200
    AP_GROUPINFO("_FAN_1_MIN", 10, FSOPowerStack, fan_1_min_Hz, FSO_FAN_ERROR_HZ_MIN),

    // @Param: _FAN_2_MIN
    // @DisplayName: Fan 2 tachometer Alarm
    // @Description: Alarm threshold for minimum fan 2 tachometer in Hz
    // @Range: 0 200
    AP_GROUPINFO("_FAN_2_MIN", 11, FSOPowerStack, fan_2_min_Hz, 0.0),

    // @Param: _FAN_3_MIN
    // @DisplayName: Fan 3 tachometer Alarm
    // @Description: Alarm threshold for minimum fan 3 tachometer in Hz
    // @Range: 0 200
    AP_GROUPINFO("_FAN_3_MIN", 12, FSOPowerStack, fan_3_min_Hz, 0.0),

    // @Param: _FAN_4_MIN
    // @DisplayName: Fan 4 tachometer Alarm
    // @Description: Alarm threshold for minimum fan 4 tachometer in Hz
    // @Range: 0 200
    AP_GROUPINFO("_FAN_4_MIN", 13, FSOPowerStack, fan_4_min_Hz, FSO_FAN_ERROR_HZ_MIN),

    // @Param: _CAL_P1C1
    // @DisplayName: Payload 1 Coefficient 1
    // @Description: Payload 1 Coefficient 1
    // @Range: 0 10
    AP_GROUPINFO("_CAL_P1C1", 14, FSOPowerStack, cal_payload_P1c1, FSO_C1_DEFAULT),

    // @Param: _CAL_P1C2
    // @DisplayName: Payload 1 Coefficient 2
    // @Description: Payload 1 Coefficient 2
    // @Range: 0 10
    AP_GROUPINFO("_CAL_P1C2", 15, FSOPowerStack, cal_payload_P1c2, FSO_C2_DEFAULT),

    // @Param: _CAL_P2C1
    // @DisplayName: Payload 2 Coefficient 1
    // @Description: Payload 2 Coefficient 1
    // @Range: 0 10
    AP_GROUPINFO("_CAL_P2C1", 16, FSOPowerStack, cal_payload_P2c1, FSO_C1_DEFAULT),

    // @Param: _CAL_P2C2
    // @DisplayName: Payload 2 Coefficient 2
    // @Description: Payload 2 Coefficient 2
    // @Range: 0 10
    AP_GROUPINFO("_CAL_P2C2", 17, FSOPowerStack, cal_payload_P2c2, FSO_C2_DEFAULT),

    AP_GROUPEND
};

FSOPowerStack::FSOPowerStack(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  handle interrupt for a fan
 */
void FSOPowerStack::Fan::handler(uint8_t _pin, bool pin_state, uint32_t timestamp)
{
    if (_pin != pin) {
        return;
    }
    if (last_pulse_us != 0) {
        const uint32_t dt = timestamp - last_pulse_us;
        dt_sum += dt;
        dt_count++;
    }
    last_pulse_us = timestamp;
}

/*
  initialise a fan interrupt
 */
void FSOPowerStack::Fan::init()
{
    hal.gpio->pinMode(pin, HAL_GPIO_INPUT);
    hal.gpio->attach_interrupt(
        pin,
        FUNCTOR_BIND_MEMBER(&FSOPowerStack::Fan::handler, void, uint8_t, bool, uint32_t),
        AP_HAL::GPIO::INTERRUPT_RISING);
    last_error_ms = AP_HAL::millis() - FSO_ERROR_FAN_MSG_INTERVAL + 30000; // delay first fan error message by 30 seconds
}

void FSOPowerStack::Fan::report_errors()
{
    const auto now_ms = AP_HAL::millis();
    if (now_ms - last_error_ms < FSO_ERROR_FAN_MSG_INTERVAL) {
        return;
    }
    if (freq_hz >= min_Hz) {
        // no error
        return;
    }
    last_error_ms = now_ms;

    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Fan %u failure; %uHz < %uHz", instance, unsigned(freq_hz), unsigned(min_Hz.get()));
}


void FSOPowerStack::init()
{
    uint32_t now_ms = AP_HAL::millis();
    last_report_errors_ms = now_ms;

    for (auto & fan : fans) {
        fan.init();
    }

    float over_current_tc = FSO_OVER_CURRENT_TC;
    payload_HV_current_filter.set_cutoff_frequency(1.0/over_current_tc);
    payload_1_current_filter.set_cutoff_frequency(1.0/over_current_tc);
    payload_2_current_filter.set_cutoff_frequency(1.0/over_current_tc);

    set_internal_HC_on();
}

/*
  init called after CAN init
 */
void FSOPowerStack::late_init()
{
    periph.dac.init();
    if (option_is_set(Option::PAYLOAD_HV_ON)) {
        set_HV_payload_on();
    } else {
        set_HV_payload_off();
    }
    if (option_is_set(Option::PAYLOAD_BEC_ON)) {
        set_payload_BEC_1_on();
        set_payload_BEC_2_on();
    } else {
        set_payload_BEC_1_off();
        set_payload_BEC_2_off();
    }
}

/*
  update fan frequency reading
 */
void FSOPowerStack::update_fans(void)
{
    uint32_t now_ms = AP_HAL::millis();
    // update fans at 1Hz
    if (now_ms - last_fan_ms < 1000) {
        return;
    }
    last_fan_ms = now_ms;

    for (auto &fan : fans) {
        fan.update();
    }
}

void FSOPowerStack::Fan::update(void)
{
    if (dt_count == 0) {
        freq_hz = 0;
        return;
    }
    float dt_avg;
    {
        void *irqstate = hal.scheduler->disable_interrupts_save();
        dt_avg = float(dt_sum) / dt_count;
        dt_sum = 0;
        dt_count = 0;
        hal.scheduler->restore_interrupts(irqstate);
    }
    freq_hz = 1.0/(dt_avg*1.0e-6);
}

void FSOPowerStack::debug_msg(void)
{
    if (!option_is_set(Option::DEBUG)) {
        return;
    }

    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_debug_msg_ms < 5000) {
        return;
    }
    last_debug_msg_ms = now_ms;

    auto &batt = AP::battery();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Volt - B1:%.1f, B2:%.1f, PHV:%.1f, P1:%.2f, P2:%.2f, IHC:%.2f, I1:%.2f, I2:%.2f, O:%.2f",
                  batt.voltage(0), batt.voltage(1), batt.voltage(2), batt.voltage(3),
                  batt.voltage(4), batt.voltage(5), batt.voltage(6), batt.voltage(7), batt.voltage(8));

    float B1_C = nanf("");
    float B2_C = nanf("");
    float PHV_C = nanf("");
    float P1_C = nanf("");
    float P2_C = nanf("");
    float IHC_C = nanf("");
    float I1_C = nanf("");
    float I2_C = nanf("");
    if (batt.current_amps(B1_C, 0) || batt.current_amps(B2_C, 1) || batt.current_amps(PHV_C, 2) || batt.current_amps(P1_C, 3) || batt.current_amps(P2_C, 4) || batt.current_amps(IHC_C, 5) || batt.current_amps(I1_C, 6) || batt.current_amps(I2_C, 7)) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Current - B1:%.4f, B2:%.4f, PHV:%.2f, P1:%.2f, P2:%.2f, IHC:%.2f, I1:%.2f, I2:%.2f",
                  B1_C, B2_C, PHV_C, P1_C, P2_C, IHC_C, I1_C, I2_C);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Current Limit - PHV:%.2f, P1:%.2f, P2:%.2f,  IHC:%.2f, I1:%.2f, I2:%.2f",
                  payload_HV_current_filter.get(), payload_1_current_filter.get(), payload_2_current_filter.get(),
                  internal_HC_current_filter.get(), internal_1_current_filter.get(), internal_2_current_filter.get());

    float main_temp;
    float PHV_temp;
    float P1_temp;
    float P2_temp;
    float IHC_temp;
    float I1_temp;
    float I2_temp;
    if (batt.get_temperature(main_temp, 9) && batt.get_temperature(PHV_temp, 2) && batt.get_temperature(P1_temp, 3) && batt.get_temperature(P2_temp, 4) && batt.get_temperature(IHC_temp, 5) && batt.get_temperature(I1_temp, 6) && batt.get_temperature(I2_temp, 7)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Temp - M:%.1f, PHV:%.1f P1:%.1f, P2:%.1f, IHC:%.1f, I1:%.1f, I2:%.1f",
                    main_temp, PHV_temp, P1_temp, P2_temp, IHC_temp, I1_temp, I2_temp);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Fans - F1: %.0f, F2: %.0f, F3: %.0f, F4: %.0f",
                    fans[0].freq_hz, fans[1].freq_hz, fans[2].freq_hz, fans[3].freq_hz);
}

void FSOPowerStack::report_errors(void)
{
    uint32_t now_ms = AP_HAL::millis();
    auto &batt = AP::battery();

    for (auto &fan : fans) {
        fan.report_errors();
    }

    if (now_ms - last_main_report_errors_ms > FSO_MAIN_ERROR_MSG_INTERVAL) {
        float main_temp;
        if (batt.get_temperature(main_temp, 9)) {
            if (main_temp > FSO_MAIN_TEMPERATURE_MAX) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "POWER STACK OVER TEMPERATURE: %.2f deg", main_temp);
                last_main_report_errors_ms = now_ms;
            }
        }
    }

    if (now_ms - last_report_errors_ms < FSO_ERROR_MSG_INTERVAL) {
        return;
    }

    float   payload_1_temp;
    if (batt.get_temperature(payload_1_temp, 3)) {
        if ((payload_1_temp > MIN(bec_temperature_max, FSO_BEC_HC_TEMPERATURE_MAX) - 5.0)
                && payload_BEC_1_on == true) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "BEC 1 temp warning: %.2f deg", payload_1_temp);
            last_report_errors_ms = now_ms;
        }
    }

    float   payload_2_temp;
    if (batt.get_temperature(payload_2_temp, 4)) {
        if ((payload_2_temp > MIN(bec_temperature_max, FSO_BEC_HC_TEMPERATURE_MAX) - 5.0)
                && payload_BEC_2_on == true) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "BEC 2 temp warning: %.2f deg", payload_2_temp);
            last_report_errors_ms = now_ms;
        }
    }

    float internal_HC_temp;
    if (batt.get_temperature(internal_HC_temp, 5)) {
        if (internal_HC_temp > FSO_BEC_HC_TEMPERATURE_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC HC over temp: %.2f deg", internal_HC_temp);
            last_report_errors_ms = now_ms;
        }
    }

    float internal_1_temp;
    if (batt.get_temperature(internal_1_temp, 6)) {
        if (internal_1_temp > FSO_INTERNAL_BEC_TEMPERATURE_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC 1 over temp: %.2f deg", internal_1_temp);
            last_report_errors_ms = now_ms;
        }
    }

    float internal_2_temp;
    if (batt.get_temperature(internal_2_temp, 7)) {
        if (internal_2_temp > FSO_INTERNAL_BEC_TEMPERATURE_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC 2 over temp: %.2f deg", internal_2_temp);
            last_report_errors_ms = now_ms;
        }
    }

    float internal_HC_current;
    if (batt.current_amps(internal_HC_current, 5)) {
        internal_HC_current_filter.apply(internal_HC_current, 0.001 * FSO_LOOP_TIME_MS);
        if (internal_HC_current_filter.get() > FSO_INTERNAL_BEC_HC_CURRENT_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC HC over current fault: %.2f A", internal_HC_current_filter.get());
            last_report_errors_ms = now_ms;
        }
    }
    
    float internal_1_current;
    if (batt.current_amps(internal_1_current, 6)) {
        internal_1_current_filter.apply(internal_1_current, 0.001 * FSO_LOOP_TIME_MS);
        if (internal_1_current_filter.get() > FSO_INTERNAL_BEC_CURRENT_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC 1 over current fault: %.2f A", internal_1_current_filter.get());
            last_report_errors_ms = now_ms;
        }
    }
    
    float internal_2_current;
    if (batt.current_amps(internal_2_current, 7)) {
        internal_2_current_filter.apply(internal_2_current, 0.001 * FSO_LOOP_TIME_MS);
        if (internal_2_current_filter.get() > FSO_INTERNAL_BEC_CURRENT_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC 2 over current fault: %.2f A", internal_2_current_filter.get());
            last_report_errors_ms = now_ms;
        }
    }
}


void FSOPowerStack::update_switches()
{
    uint32_t now_ms = AP_HAL::millis();

    if (!switch_1_pressed()) {
        switch_1_press_time_ms = now_ms;
        switch_1_switch_released = true;
    }
    
    if (switch_1_is_off()){
        if (switch_1_switch_released && (now_ms - switch_1_press_time_ms > FSO_SWITCH_ON_TIME_MS)) {
            set_switch_1_on();
            switch_1_switch_released = false;
        }
    } else {
        if (switch_1_switch_released && (now_ms - switch_1_press_time_ms > FSO_SWITCH_OFF_TIME_MS)) {
            set_switch_1_off();
            switch_1_switch_released = false;
        }
    }

    if (!switch_2_pressed()) {
        switch_2_press_time_ms = now_ms;
        switch_2_switch_released = true;
    }
    
    if (switch_2_on == false){
        if (switch_2_switch_released && (now_ms - switch_2_press_time_ms > FSO_SWITCH_ON_TIME_MS)) {
            set_switch_2_on();
            switch_2_switch_released = false;
        }
    } else {
        if (switch_2_switch_released && (now_ms - switch_2_press_time_ms > FSO_SWITCH_OFF_TIME_MS)) {
            set_switch_2_off();
            switch_2_switch_released = false;
        }
    }
}

void FSOPowerStack::update_main_power()
{
    uint32_t now_ms = AP_HAL::millis();
    auto &batt = AP::battery();
    
    // Main Turn On State Machine
    switch (main_state) {

    case TurnOnState::Off:
        if (main_on == true) {
            main_state = TurnOnState::PreChargeStart;
            if (!version_displayed) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "PowerStack Version: 2.0");
                version_displayed = true;
            }
        }
        break;

    case TurnOnState::PreChargeStart:
        if (fabsf(batt.voltage(0) - batt.voltage(1)) < battery_diff_max) {
            set_main_PC_on();
            start_main_precharge_ms = now_ms;
            main_state = TurnOnState::PreCharge;
            break;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Unbalanced batteries B1: %.2f V, B2: %.2f V", batt.voltage(0), batt.voltage(1));
            set_main_off();
            set_switch_1_off();
            main_state = TurnOnState::ShutDown;
        }
        break;

    case TurnOnState::PreCharge:
        if (MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(8) < FSO_OUT_VOLTS_DIFF_MAX) {
            // Turn off Pre-Charge
            set_main_PC_off();
            // Turn on main battery switches
            set_bat_1_SW_on();
            set_bat_2_SW_on();
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Main Power On");
            main_state = TurnOnState::On;
        } else if (now_ms - start_main_precharge_ms > FSO_PRECHARGE_TIMEOUT_MS) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Main pre-charge failure, dV: %.2f", MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(8));
            main_state = TurnOnState::ShutDown;
        }
        break;

    case TurnOnState::On:
        if (main_on == false) {
            main_state = TurnOnState::ShutDown;
        }
        break;

    case TurnOnState::ShutDown:
        // Turn off Pre-Charge
        set_main_PC_off();
        set_bat_1_SW_off();
        set_bat_2_SW_off();
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Main Power Off");
        main_state = TurnOnState::Off;
        break;
    }
}

void FSOPowerStack::update_payload_HV_power()
{
    uint32_t now_ms = AP_HAL::millis();
    auto &batt = AP::battery();
    
    float payload_HV_current;
    if (batt.current_amps(payload_HV_current, 2)) {
        if ((payload_HV_current > FSO_PAYLOAD_HV_CURRENT_FUSE)
                && (payload_HV_state != ShutDown)) {
            payload_HV_state = ShutDown;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "HV fuse shutdown: %.2f A", payload_HV_current);
        }
        payload_HV_current_filter.apply(payload_HV_current, 0.001 * FSO_LOOP_TIME_MS);
        if ((payload_HV_current_filter.get() > MIN(payload_HV_current_max, FSO_PAYLOAD_HV_CURRENT_MAX))
                && (payload_HV_state != ShutDown)) {
            payload_HV_state = ShutDown;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "HV over current shutdown: %.2f A", payload_HV_current_filter.get());
        }
    }
    
    // Payload Turn On State Machine
    switch (payload_HV_state) {

    case TurnOnState::Off:
        if (payload_HV_on == true) {
            payload_HV_state = TurnOnState::PreChargeStart;
        }
        break;

    case TurnOnState::PreChargeStart:
        // Turn on Payload HV pre-charge
        set_payload_HV_PC_on();
        start_payload_HV_precharge_ms = now_ms;
        payload_HV_state = TurnOnState::PreCharge;
        break;

    case TurnOnState::PreCharge:
        if (MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(2) < FSO_OUT_VOLTS_DIFF_MAX) {
            // Turn off payload HV pre-charge
            set_payload_HV_PC_off();
            // Turn on payload HV switch
            set_payload_HV_SW_on();
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HV Power On");
            payload_HV_state = TurnOnState::On;
        } else if (now_ms - start_payload_HV_precharge_ms > FSO_PRECHARGE_TIMEOUT_MS) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "HV pre-charge failure, dV: %.2f", MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(2));
            payload_HV_state = ShutDown;
        }
        break;

    case TurnOnState::On:
        if (payload_HV_on == false) {
            payload_HV_state = TurnOnState::ShutDown;
        }
        break;

    case TurnOnState::ShutDown:
        // Turn off payload HV pre-charge
        set_payload_HV_PC_off();
        // Turn off payload HV switch
        set_payload_HV_SW_off();
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "HV Power Off");
        payload_HV_state = TurnOnState::Off;
        break;
    }
}

void FSOPowerStack::update_payload_BEC()
{
    auto &batt = AP::battery();

    float payload_1_current;
    if (batt.current_amps(payload_1_current, 3)) {
        if ((payload_1_current > FSO_PAYLOAD_BEC_CURRENT_MAX)
                && payload_BEC_1_on == true) {
            set_payload_BEC_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "BEC 1 fuse shutdown: %.2f A", payload_1_current);
        }
        payload_1_current_filter.apply(payload_1_current, 0.001 * FSO_LOOP_TIME_MS);
        if ((payload_1_current_filter.get() > MIN(payload_1_current_max, FSO_PAYLOAD_BEC_CURRENT_MAX))
                && payload_BEC_1_on == true) {
            set_payload_BEC_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "BEC 1 over current shutdown: %.2f A", payload_1_current_filter.get());
        }
    }
    
    float payload_2_current;
    if (batt.current_amps(payload_2_current, 4)) {
        if ((payload_2_current > FSO_PAYLOAD_BEC_CURRENT_MAX)
                && payload_BEC_2_on == true) {
            set_payload_BEC_2_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "BEC 2 fuse shutdown: %.2f A", payload_2_current);
        }
        payload_2_current_filter.apply(payload_2_current, 0.001 * FSO_LOOP_TIME_MS);
        if ((payload_2_current_filter.get() > MIN(payload_2_current_max, FSO_PAYLOAD_BEC_CURRENT_MAX))
                && payload_BEC_2_on == true) {
            set_payload_BEC_2_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "BEC 2 over current shutdown: %.2f A", payload_2_current_filter.get());
        }
    }

    float   payload_1_temp;
    if (batt.get_temperature(payload_1_temp, 3)) {
        if ((payload_1_temp > MIN(bec_temperature_max, FSO_BEC_HC_TEMPERATURE_MAX))
                && payload_BEC_1_on == true) {
            set_payload_BEC_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "BEC 1 over temp shutdown: %.2f deg", payload_1_temp);
        }
    }

    float   payload_2_temp;
    if (batt.get_temperature(payload_2_temp, 4)) {
        if ((payload_2_temp > MIN(bec_temperature_max, FSO_BEC_HC_TEMPERATURE_MAX))
                && payload_BEC_2_on == true) {
            set_payload_BEC_2_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "BEC 2 over temp shutdown: %.2f deg", payload_2_temp);
        }
    }
}

/*
  update DACs
 */
void FSOPowerStack::update_DAC()
{
    const float v1 = (cal_payload_P1c1 - payload_1_voltage) * cal_payload_P1c2;
    if (!periph.dac.set_voltage(0, 0, v1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "set voltage %u %.3f failed", unsigned(0), v1);
    }
    const float v2 = (cal_payload_P2c1 - payload_2_voltage) * cal_payload_P2c2;
    if (!periph.dac.set_voltage(0, 3, v2)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "set voltage %u %.3f failed", unsigned(3), v2);
    }
}



/*
  update FSOPowerStack
 */
void FSOPowerStack::update(bool battery_read)
{
    if (!done_late_init) {
        done_late_init = true;
        late_init();
    }

    update_switches();

    update_main_power();

    update_payload_HV_power();

    if (!battery_read) {
        // run at 10Hz after battery read
        return;
    }

    if (switch_1_is_on()) {
        set_LED_1_on();
        set_main_on();
    } else {
        set_LED_1_off();
        set_main_off();
    }
    if (!option_is_set(Option::PAYLOAD_HV_ON)) {
        if (main_state == TurnOnState::On) {
            set_HV_payload_on();
        } else {
            set_HV_payload_off();
        }
    }
    if (!option_is_set(Option::PAYLOAD_BEC_ON)) {
        if (main_state == TurnOnState::On) {
            set_payload_BEC_1_on();
            set_payload_BEC_2_on();
        } else {
            set_payload_BEC_1_off();
            set_payload_BEC_2_off();
        }
    }

    if (switch_2_is_on()) {
        set_LED_2_on();
    } else {
        set_LED_2_off();
    }

    update_DAC();

    update_fans();

    update_payload_BEC();

    debug_msg();
    report_errors();
}

#endif  // AP_PERIPH_FSO_POWERSTACK_ENABLED

