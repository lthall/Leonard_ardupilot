#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include "AP_BattMonitor_Params.h"

// maximum number of battery monitors
#ifndef AP_BATT_MONITOR_MAX_INSTANCES
#define AP_BATT_MONITOR_MAX_INSTANCES       9
#endif

// first monitor is always the primary monitor
#define AP_BATT_PRIMARY_INSTANCE            0

#define AP_BATT_SERIAL_NUMBER_DEFAULT       -1

#define AP_BATT_MONITOR_TIMEOUT             5000

#define AP_BATT_MONITOR_RES_EST_TC_1        0.5f
#define AP_BATT_MONITOR_RES_EST_TC_2        0.1f

#if !HAL_MINIMIZE_FEATURES && BOARD_FLASH_SIZE > 1024
#define AP_BATT_MONITOR_CELLS_MAX           14
#else
#define AP_BATT_MONITOR_CELLS_MAX           12
#endif

#define AP_BATT_MONITOR_SERIAL_LEN          14
#define AP_BATT_MONITOR_MODEL_NAME_LEN      32
#define AP_BATT_MONITOR_FIRMWARE_ID_LEN     15

#ifndef AP_BATTMON_SMBUS_ENABLE
#define AP_BATTMON_SMBUS_ENABLE 1
#endif

#ifndef AP_BATTMON_FUEL_ENABLE
#define AP_BATTMON_FUEL_ENABLE 1
#endif

// declare backend class
class AP_BattMonitor_Backend;
class AP_BattMonitor_Analog;
class AP_BattMonitor_SMBus;
class AP_BattMonitor_SMBus_Solo;
class AP_BattMonitor_SMBus_Generic;
class AP_BattMonitor_SMBus_Maxell;
class AP_BattMonitor_SMBus_Rotoye;
class AP_BattMonitor_UAVCAN;
class AP_BattMonitor_Generator;
class AP_BattMonitor_INA2XX;
class AP_BattMonitor_LTC2946;
class AP_BattMonitor_Torqeedo;

class AP_BattMonitor
{
    friend class AP_BattMonitor_Backend;
    friend class AP_BattMonitor_Analog;
    friend class AP_BattMonitor_SMBus;
    friend class AP_BattMonitor_SMBus_Solo;
    friend class AP_BattMonitor_SMBus_Generic;
    friend class AP_BattMonitor_SMBus_Maxell;
    friend class AP_BattMonitor_SMBus_Rotoye;
    friend class AP_BattMonitor_UAVCAN;
    friend class AP_BattMonitor_Sum;
    friend class AP_BattMonitor_FuelFlow;
    friend class AP_BattMonitor_FuelLevel_PWM;
    friend class AP_BattMonitor_Generator;
    friend class AP_BattMonitor_INA2XX;
    friend class AP_BattMonitor_LTC2946;

    friend class AP_BattMonitor_Torqeedo;

public:

    // battery failsafes must be defined in levels of severity so that vehicles wont fall backwards
    enum class Failsafe : uint8_t {
        None = 0,
        HighCurrent,
        HighTemperature,
        Low,
        Critical,
    };

    // Battery monitor driver types
    enum class Type {
        NONE                       = 0,
        ANALOG_VOLTAGE_ONLY        = 3,
        ANALOG_VOLTAGE_AND_CURRENT = 4,
        SOLO                       = 5,
        BEBOP                      = 6,
        SMBus_Generic              = 7,
        UAVCAN_BatteryInfo         = 8,
        BLHeliESC                  = 9,
        Sum                        = 10,
        FuelFlow                   = 11,
        FuelLevel_PWM              = 12,
        SUI3                       = 13,
        SUI6                       = 14,
        NeoDesign                  = 15,
        MAXELL                     = 16,
        GENERATOR_ELEC             = 17,
        GENERATOR_FUEL             = 18,
        Rotoye                     = 19,
        // 20 was MPPT_PacketDigital
        INA2XX                     = 21,
        LTC2946                    = 22,
        Torqeedo                   = 23,
        UAVCAN_FlyhawkSmartBattery = 24
    };

    FUNCTOR_TYPEDEF(battery_failsafe_handler_fn_t, void, const char *, const int8_t);

    AP_BattMonitor(uint32_t log_battery_bit, battery_failsafe_handler_fn_t battery_failsafe_handler_fn, const int8_t *failsafe_priorities);

    /* Do not allow copies */
    AP_BattMonitor(const AP_BattMonitor &other) = delete;
    AP_BattMonitor &operator=(const AP_BattMonitor&) = delete;

    static AP_BattMonitor *get_singleton() {
        return _singleton;
    }

    // cell voltages in millivolts
    struct cells {
        uint16_t cells[AP_BATT_MONITOR_CELLS_MAX];
    };

    // Battery info and (yet) unknown fields
    struct BattMonitor_Info {
        uint16_t    capacity_initial_mah;      // Capacity when full according to manufacturer
        uint16_t    capacity_current_mah;      // Capacity when full (accounting for battery degradation)
        uint16_t    cycles;                    // Charge/discharge cycle count
        char        serial_num[AP_BATT_MONITOR_SERIAL_LEN];
        char        model_name[AP_BATT_MONITOR_MODEL_NAME_LEN];
        char        firmware_id[AP_BATT_MONITOR_FIRMWARE_ID_LEN];
        uint8_t     lifetime_pct;
        uint16_t    made_year;
        uint8_t     unk1;
        uint8_t     unk2;
        uint16_t    unk3;
        uint8_t     unk4;
        uint8_t     unk5;
        uint8_t     info_unk1;
        uint16_t    info_unk2;
        uint16_t    info_unk3;
        uint32_t    info_serial;
        uint8_t     info_unk4[8];
        uint8_t     info_unk5[5];
        uint16_t    version_unk1;
        uint8_t     version_unk2[6];
    };

    // The BattMonitor_State structure is filled in by the backend driver
    struct BattMonitor_State {
        uint8_t                     battery_id;                // identifies the battery within this vehicle
        cells                       cell_voltages;             // battery cell voltages in millivolts, 10 cells matches the MAVLink spec
        float                       voltage;                   // voltage in volts
        float                       current_amps;              // current in amperes
        uint8_t                     capacity_pct;              // capacity % direct from battery (0% to 100%)
        float                       consumed_mah;              // total current draw in milliamp hours since start-up
        float                       consumed_wh;               // total energy consumed in Wh since start-up
        float                       remaining_capacity_wh;     // capacity in watt hours direct from battery
        uint32_t                    last_time_micros;          // time when voltage and current was last read in microseconds
        uint32_t                    low_voltage_start_ms;      // time when voltage dropped below the minimum in milliseconds
        uint32_t                    critical_voltage_start_ms; // critical voltage failsafe start timer in milliseconds
        uint32_t                    high_temperature_start_ms; // high temperature failsafe start time in milliseconds
        uint32_t                    high_current_start_ms;     // high current failsafe start time in milliseconds
        float                       temperature;               // battery temperature in degrees Celsius
        uint32_t                    temperature_time;          // timestamp of the last received temperature message
        float                       voltage_resting_estimate;  // voltage with sag removed based on current and resistance estimate in Volt
        float                       resistance;                // resistance, in Ohms, calculated by comparing resting voltage vs in flight voltage
        Failsafe                    failsafe;                  // stage failsafe the battery is in
        bool                        healthy;                   // battery monitor is communicating correctly
        bool                        is_powering_off;           // true when power button commands power off
        bool                        powerOffNotified;          // only send powering off notification once
        uint32_t                    time_remaining;            // remaining battery time
        bool                        has_time_remaining;        // time_remaining is only valid if this is true
        const struct AP_Param::GroupInfo *var_info;
        float                       current2_amps;             // current2 in amperes
        bool                        is_on;                     // true when the battery is powered on
        MAV_BATTERY_CHARGE_STATE    charge_state;              // charge state of the battery
        float                       average_power_10sec;       // average power consumption over the last 10 seconds in watt

        struct BattMonitor_Info info;    
    };

    static const struct AP_Param::GroupInfo *backend_var_info[AP_BATT_MONITOR_MAX_INSTANCES];

    // Return the number of battery monitor instances
    uint8_t num_instances(void) const { return _num_instances; }

    // detect and initialise any available battery monitors
    void init();

    /// Read the battery voltage and current for all batteries.  Should be called at 10hz
    void read();

    // healthy - returns true if monitor is functioning
    bool healthy(uint8_t instance) const;

    // return true if all configured battery monitors are healthy
    bool healthy() const;

    /// voltage - returns battery voltage in volts
    float voltage(uint8_t instance) const;
    float voltage() const { return voltage(AP_BATT_PRIMARY_INSTANCE); }

    // voltage for a GCS, may be resistance compensated
    float gcs_voltage(uint8_t instance) const;
    float gcs_voltage(void) const { return gcs_voltage(AP_BATT_PRIMARY_INSTANCE); }

    /// get voltage with sag removed (based on battery current draw and resistance)
    /// this will always be greater than or equal to the raw voltage
    float voltage_resting_estimate(uint8_t instance) const;
    float voltage_resting_estimate() const { return voltage_resting_estimate(AP_BATT_PRIMARY_INSTANCE); }

    /// current_amps - returns the instantaneous current draw in amperes
    bool current_amps(float &current, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /// current2_amps - returns the instantaneous current draw for the extra current battery report, in amperes
    bool current2_amps(float &current2, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /// consumed_mah - returns total current drawn since start-up in milliampere.hours
    bool consumed_mah(float &mah, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /// consumed_wh - returns total energy drawn since start-up in watt.hours
    bool consumed_wh(float&wh, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
    virtual bool capacity_remaining_pct(uint8_t &percentage, uint8_t instance) const WARN_IF_UNUSED;
    bool capacity_remaining_pct(uint8_t &percentage) const WARN_IF_UNUSED;

    /// time_remaining - returns remaining battery time
    bool time_remaining(uint32_t &seconds, const uint8_t instance = AP_BATT_PRIMARY_INSTANCE) const WARN_IF_UNUSED;

    /// pack_capacity_mah - returns the capacity of the battery pack in mAh when the pack is full
    int32_t pack_capacity_mah(uint8_t instance) const;
    int32_t pack_capacity_mah() const { return pack_capacity_mah(AP_BATT_PRIMARY_INSTANCE); }
 
    /// returns true if a battery failsafe has ever been triggered
    bool has_failsafed(void) const { return _has_triggered_failsafe; };

    /// returns the highest failsafe action that has been triggered
    int8_t get_highest_failsafe_priority(void) const { return _highest_failsafe_priority; };

    /// get_type - returns battery monitor type
    enum Type get_type() const { return get_type(AP_BATT_PRIMARY_INSTANCE); }
    enum Type get_type(uint8_t instance) const {
        return (Type)_params[instance]._type.get();
    }

    /// get_serial_number - returns battery serial number
    int32_t get_serial_number() const { return get_serial_number(AP_BATT_PRIMARY_INSTANCE); }
    int32_t get_serial_number(uint8_t instance) const {
        return _params[instance]._serial_number;
    }

    /// true when (voltage * current) > watt_max
    bool overpower_detected() const;
    bool overpower_detected(uint8_t instance) const;

    // cell voltages in millivolts
    bool has_cell_voltages() const { return has_cell_voltages(AP_BATT_PRIMARY_INSTANCE); }
    bool has_cell_voltages(const uint8_t instance) const;
    const cells &get_cell_voltages() const { return get_cell_voltages(AP_BATT_PRIMARY_INSTANCE); }
    const cells &get_cell_voltages(const uint8_t instance) const;

    bool get_battery_info(BattMonitor_Info &info, const uint8_t instance) const;

    // temperature
    bool get_temperature(float &temperature) const { return get_temperature(temperature, AP_BATT_PRIMARY_INSTANCE); }
    bool get_temperature(float &temperature, const uint8_t instance) const;

    // cycle count
    bool get_cycle_count(uint8_t instance, uint16_t &cycles) const;

    // get battery resistance estimate in ohms
    float get_resistance() const { return get_resistance(AP_BATT_PRIMARY_INSTANCE); }
    float get_resistance(uint8_t instance) const { return state[instance].resistance; }

    // set is_powering_off to true when the battery instance is powring off, false otherwise. return false if out of range.
    bool is_powering_off(bool &is_powering_off, uint8_t instance) const;

    // set is_battery_on to true when the battery instance is currently on, false otherwise. return false if out of range.
    bool is_battery_on(bool &is_battery_on, uint8_t instance) const;

    // returns false if we fail arming checks, in which case the buffer will be populated with a failure message
    bool arming_checks(size_t buflen, char *buffer) const;

    // sends powering off mavlink broadcasts and sets notify flag
    void checkPoweringOff(void);

    // reset battery remaining percentage
    bool reset_remaining_mask(uint16_t battery_mask, float percentage);
    bool reset_remaining(uint8_t instance, float percentage) { return reset_remaining_mask(1U<<instance, percentage);}

    // Returns mavlink charge state
    MAV_BATTERY_CHARGE_STATE get_mavlink_charge_state(const uint8_t instance) const;

    // Returns mavlink fault state
    uint32_t get_mavlink_fault_bitmask(const uint8_t instance) const;

    static const struct AP_Param::GroupInfo var_info[];
    static const struct AP_Param::GroupInfo var_info_global[];  // for parameters that not belong to a specific battery monitor instance

protected:

    /// parameters
    AP_BattMonitor_Params _params[AP_BATT_MONITOR_MAX_INSTANCES];

private:
    static AP_BattMonitor *_singleton;

    BattMonitor_State state[AP_BATT_MONITOR_MAX_INSTANCES];
    AP_BattMonitor_Backend *drivers[AP_BATT_MONITOR_MAX_INSTANCES];
    uint32_t    _log_battery_bit;
    uint8_t     _num_instances;                                     /// number of monitors

    void convert_dynamic_param_groups(uint8_t instance);

    /// returns the failsafe state of the battery
    Failsafe check_failsafe(const uint8_t instance);
    void check_failsafes(void); // checks all batteries failsafes

    battery_failsafe_handler_fn_t _battery_failsafe_handler_fn;
    const int8_t *_failsafe_priorities; // array of failsafe priorities, sorted highest to lowest priority, -1 indicates no more entries

    int8_t      _highest_failsafe_priority; // highest selected failsafe action level (used to restrict what actions we move into)
    bool        _has_triggered_failsafe;  // true after a battery failsafe has been triggered for the first time

    uint32_t    _battery_power_diff_started_ms; // battery power diff start time in milliseconds

    AP_Int16 _power_diff_min_pwr;       /// minimal power (in Watt) of any of the batts to check for battery power diff
    AP_Int8  _power_diff_timeout;       /// timeout (in seconds) before we report battery power diff
    AP_Int8  _power_diff_percent;       /// the maximum power diff between batteries allowed
    
    uint32_t    _last_log_write_us[AP_BATT_MONITOR_MAX_INSTANCES];  // last time battery messages were written to the log

};

namespace AP {
    AP_BattMonitor &battery();
};
