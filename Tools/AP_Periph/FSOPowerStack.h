#pragma once

#if AP_PERIPH_FSO_POWERSTACK_ENABLED

extern const AP_HAL::HAL &hal;

class FSOPowerStack {
public:
    friend class AP_Periph_FW;
    FSOPowerStack(void);

    static const struct AP_Param::GroupInfo var_info[];

    void init(void);
    void update(bool battery_read);

private:

    enum class Option : uint32_t {
        DEBUG               = 0,
        PAYLOAD_BEC_ON      = 1
    };

    bool option_is_set(Option option) {
        return (options & (1U << uint32_t(option))) != 0;
    }

    AP_Int32 options;
    AP_Float battery_diff_max;
    AP_Float payload_1_voltage;
    AP_Float payload_2_voltage;
    AP_Float payload_1_current_max;
    AP_Float payload_2_current_max;
    AP_Float bec_temperature_max;
    AP_Float fan_1_min_Hz;
    AP_Float fan_2_min_Hz;
    AP_Float fan_3_min_Hz;
    AP_Float fan_4_min_Hz;
    AP_Float cal_payload_P1c1;
    AP_Float cal_payload_P1c2;
    AP_Float cal_payload_P2c1;
    AP_Float cal_payload_P2c2;
    
    uint32_t last_update_ms;
    bool version_displayed = false;
    

    class Fan {
    public:
        Fan(uint8_t _instance, uint8_t _pin, const AP_Float &_min_Hz) :
            instance{_instance},
            pin{_pin},
            min_Hz{_min_Hz}
            { }
        void init();
        void update();
        void handler(uint8_t pin,
                     bool pin_state,
                     uint32_t timestamp);
        uint8_t get_instance_number() const { return instance; }
        void report_errors();
        float freq_hz;
    private:
        uint8_t instance;
        uint8_t pin;
        const AP_Float &min_Hz;
        uint32_t last_pulse_us;
        uint32_t dt_sum;
        uint32_t dt_count;
        uint32_t last_error_ms;
    } fans[4] {
        { 0, FSO_FAN_TACH1_PIN, fan_1_min_Hz },
        { 1, FSO_FAN_TACH2_PIN, fan_2_min_Hz },
        { 2, FSO_FAN_TACH3_PIN, fan_3_min_Hz },
        { 3, FSO_FAN_TACH4_PIN, fan_4_min_Hz },
    };

    uint32_t last_fan_ms;
    void update_fans();

    // States used during turn on
    enum TurnOnState {
        Off,
        PreChargeStart,
        PreCharge,
        On,
        ShutDown
    };

    void update_switches();
    void update_main_power();
    void update_payload_BEC();
    void update_DAC();

    void set_main_on(){main_on = true;}
    void set_main_off(){main_on = false;}

    bool main_is_on(){return main_state == TurnOnState::On;}
    bool main_is_off(){return main_state == TurnOnState::Off;}

    void set_switch_1_on(){switch_1_on = true;}
    void set_switch_1_off(){switch_1_on = false;}

    bool switch_1_is_on(){return switch_1_on == true;}
    bool switch_1_is_off(){return switch_1_on == false;}

    void set_LED_1_on(){hal.gpio->write(FSO_LED_MAIN_PIN, 1);}
    void set_LED_1_off(){hal.gpio->write(FSO_LED_MAIN_PIN, 0);}

    void set_main_PC_on(){hal.gpio->write(FSO_MAIN_PC_PIN, 1);}
    void set_main_PC_off(){hal.gpio->write(FSO_MAIN_PC_PIN, 0);}

    void set_bat_1_SW_on(){hal.gpio->write(FSO_BAT_1_EN_PIN, 1);}
    void set_bat_1_SW_off(){hal.gpio->write(FSO_BAT_1_EN_PIN, 0);}

    void set_bat_2_SW_on(){hal.gpio->write(FSO_BAT_2_EN_PIN, 1);}
    void set_bat_2_SW_off(){hal.gpio->write(FSO_BAT_2_EN_PIN, 0);}

    void set_payload_BEC_1_on(){hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 1); payload_BEC_1_on = true;}
    void set_payload_BEC_1_off(){hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 0); payload_BEC_1_on = false;}

    void set_payload_BEC_2_on(){hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 1); payload_BEC_2_on = true;}
    void set_payload_BEC_2_off(){hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 0); payload_BEC_2_on = false;}

    void set_internal_HC_on(){hal.gpio->write(FSO_INTERNAL_HC_EN_PIN, 1); payload_internal_HC_on = true;}
    void set_internal_HC_off(){hal.gpio->write(FSO_INTERNAL_HC_EN_PIN, 0); payload_internal_HC_on = false;}

    bool switch_1_pressed(){return hal.gpio->read(FSO_SWITCH_MAIN_PIN);}

    uint32_t last_debug_msg_ms;
    void debug_msg();
    uint32_t last_report_errors_ms;
    uint32_t last_main_report_errors_ms;
    void report_errors();


    bool switch_1_on;
    bool switch_1_switch_released;
    uint32_t switch_1_press_time_ms;

    bool main_on;
    bool payload_BEC_1_on;
    bool payload_BEC_2_on;
    bool payload_internal_HC_on;
    uint32_t start_main_precharge_ms;
    LowPassFilterFloat  payload_1_current_filter;   // Payload BEC 1 current input filter
    LowPassFilterFloat  payload_2_current_filter;   // Payload BEC 2 current input filter
    LowPassFilterFloat  internal_HC_current_filter;  // Internal BEC HC current input filter
    LowPassFilterFloat  internal_1_current_filter;   // Internal BEC 1 current input filter
    LowPassFilterFloat  internal_2_current_filter;   // Internal BEC 2 current input filter

    TurnOnState main_state = Off;

    // init called after CAN init
    void late_init(void);

    bool done_late_init;
};

#endif // AP_PERIPH_FSO_POWERSTACK_ENABLED


