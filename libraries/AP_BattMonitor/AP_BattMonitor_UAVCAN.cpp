#include <AP_HAL/AP_HAL.h>

#if HAL_ENABLE_LIBUAVCAN_DRIVERS

#include "AP_BattMonitor.h"
#include "AP_BattMonitor_UAVCAN.h"

#include <AP_CANManager/AP_CANManager.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Math/AP_Math.h>
#include <AP_UAVCAN/AP_UAVCAN.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_Motors/AP_Motors.h>

#include <uavcan/equipment/power/BatteryInfo.hpp>
#include <uavcan/equipment/power/BatteryInfoEx.hpp>
#include <ardupilot/equipment/power/BatteryInfoAux.hpp>
#include <mppt/Stream.hpp>
#include <mppt/OutputEnable.hpp>
#include <uavcan/equipment/power/SmartBatteryInfo.hpp>
#include <uavcan/equipment/power/SmartBatteryStatus.hpp>

#include <GCS_MAVLink/GCS.h>

#define LOG_TAG "BattMon"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_UAVCAN::var_info[] = {

    // @Param: CURR_MULT
    // @DisplayName: Scales reported power monitor current
    // @Description: Multiplier applied to all current related reports to allow for adjustment if no UAVCAN param access or current splitting applications
    // @Range: .1 10
    // @User: Advanced
    AP_GROUPINFO("CURR_MULT", 30, AP_BattMonitor_UAVCAN, _curr_mult, 1.0),

    // Param indexes must be between 30 and 39 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

UC_REGISTRY_BINDER(BattInfoCb, uavcan::equipment::power::BatteryInfo);
UC_REGISTRY_BINDER(BattInfoExCb, uavcan::equipment::power::BatteryInfoEx);
UC_REGISTRY_BINDER(BattInfoAuxCb, ardupilot::equipment::power::BatteryInfoAux);
UC_REGISTRY_BINDER(MpptStreamCb, mppt::Stream);
UC_REGISTRY_BINDER(SmartBattInfoCb, uavcan::equipment::power::SmartBatteryInfo);
UC_REGISTRY_BINDER(SmartBattStatusCb, uavcan::equipment::power::SmartBatteryStatus);

/// Constructor
AP_BattMonitor_UAVCAN::AP_BattMonitor_UAVCAN(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, BattMonitor_UAVCAN_Type type, AP_BattMonitor_Params &params, uint8_t instance_number) :
    AP_BattMonitor_Backend(mon, mon_state, params, instance_number),
    _type(type)
{
    AP_Param::setup_object_defaults(this,var_info);
    _state.var_info = var_info;

    // starts with not healthy
    _state.healthy = false;
}

void AP_BattMonitor_UAVCAN::subscribe_msgs(AP_UAVCAN* ap_uavcan)
{
    if (ap_uavcan == nullptr) {
        return;
    }

    auto* node = ap_uavcan->get_node();

    uavcan::Subscriber<uavcan::equipment::power::BatteryInfo, BattInfoCb> *battinfo_listener;
    battinfo_listener = new uavcan::Subscriber<uavcan::equipment::power::BatteryInfo, BattInfoCb>(*node);
    // Backend Msg Handler
    const int battinfo_listener_res = battinfo_listener->start(BattInfoCb(ap_uavcan, &handle_battery_info_trampoline));
    if (battinfo_listener_res < 0) {
        AP_BoardConfig::allocation_error("UAVCAN BatteryInfo subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<uavcan::equipment::power::BatteryInfoEx, BattInfoExCb> *battinfo_ex_listener;
    battinfo_ex_listener = new uavcan::Subscriber<uavcan::equipment::power::BatteryInfoEx, BattInfoExCb>(*node);
    // Backend Msg Handler
    const int battinfo_ex_listener_res = battinfo_ex_listener->start(BattInfoExCb(ap_uavcan, &handle_battery_info_extended_trampoline));
    if (battinfo_ex_listener_res < 0) {
        AP_BoardConfig::allocation_error("UAVCAN BatteryInfoEx subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<ardupilot::equipment::power::BatteryInfoAux, BattInfoAuxCb> *battinfo_aux_listener;
    battinfo_aux_listener = new uavcan::Subscriber<ardupilot::equipment::power::BatteryInfoAux, BattInfoAuxCb>(*node);
    // Backend Msg Handler
    const int battinfo_aux_listener_res = battinfo_aux_listener->start(BattInfoAuxCb(ap_uavcan, &handle_battery_info_aux_trampoline));
    if (battinfo_aux_listener_res < 0) {
        AP_BoardConfig::allocation_error("UAVCAN BatteryInfoAux subscriber start problem");
        return;
    }
    
    uavcan::Subscriber<mppt::Stream, MpptStreamCb> *mppt_stream_listener;
    mppt_stream_listener = new uavcan::Subscriber<mppt::Stream, MpptStreamCb>(*node);
    // Backend Msg Handler
    const int mppt_stream_listener_res = mppt_stream_listener->start(MpptStreamCb(ap_uavcan, &handle_mppt_stream_trampoline));
    if (mppt_stream_listener_res < 0) {
        AP_BoardConfig::allocation_error("UAVCAN Mppt::Stream subscriber start problem");
        return;
    }

    uavcan::Subscriber<uavcan::equipment::power::SmartBatteryInfo, SmartBattInfoCb> *smartbattinfo_listener;
    smartbattinfo_listener = new uavcan::Subscriber<uavcan::equipment::power::SmartBatteryInfo, SmartBattInfoCb>(*node);
    // Backend Msg Handler
    const int smartbattinfo_listener_res = smartbattinfo_listener->start(SmartBattInfoCb(ap_uavcan, &handle_smart_battery_info_trampoline));
    if (smartbattinfo_listener_res < 0) {
        AP_HAL::panic("UAVCAN SmartBatteryInfo subscriber start problem\n\r");
        return;
    }

    uavcan::Subscriber<uavcan::equipment::power::SmartBatteryStatus, SmartBattStatusCb> *smartbattstatus_listener;
    smartbattstatus_listener = new uavcan::Subscriber<uavcan::equipment::power::SmartBatteryStatus, SmartBattStatusCb>(*node);
    // Backend Msg Handler
    const int smartbattstatus_listener_res = smartbattstatus_listener->start(SmartBattStatusCb(ap_uavcan, &handle_smart_battery_status_trampoline));
    if (smartbattstatus_listener_res < 0) {
        AP_HAL::panic("UAVCAN SmartBatteryStatus subscriber start problem\n\r");
        return;
    }
}

bool AP_BattMonitor_UAVCAN::is_type_uavcan(uint8_t i)
{
    if (AP::battery().drivers[i] == nullptr) {
        return false;
    }

    switch (AP::battery().get_type(i)) {
        case AP_BattMonitor::Type::UAVCAN_BatteryInfo:
            FALLTHROUGH;
        case AP_BattMonitor::Type::UAVCAN_FlyhawkSmartBattery:
            return true;
        default:
            return false;
            break;
    }
}

AP_BattMonitor_UAVCAN* AP_BattMonitor_UAVCAN::get_uavcan_backend(AP_UAVCAN* ap_uavcan, uint8_t node_id, uint8_t battery_id)
{
    if (ap_uavcan == nullptr) {
        return nullptr;
    }
    for (uint8_t i = 0; i < AP::battery()._num_instances; i++) {
        if (!is_type_uavcan(i)) {
            continue;
        }
        AP_BattMonitor_UAVCAN* driver = (AP_BattMonitor_UAVCAN*)AP::battery().drivers[i];
        if (driver->_ap_uavcan == ap_uavcan && driver->_node_id == node_id && match_battery_id(i, battery_id)) {
            return driver;
        }
    }
    // find empty uavcan driver
    for (uint8_t i = 0; i < AP::battery()._num_instances; i++) {
        if (is_type_uavcan(i) && 
            match_battery_id(i, battery_id)) {

            AP_BattMonitor_UAVCAN* batmon = (AP_BattMonitor_UAVCAN*)AP::battery().drivers[i];
            if(batmon->_ap_uavcan != nullptr || batmon->_node_id != 0) {
                continue;
            }
            batmon->_ap_uavcan = ap_uavcan;
            batmon->_node_id = node_id;
            batmon->_instance = i;
            batmon->_node = ap_uavcan->get_node();
            batmon->init();
            AP::can().log_text(AP_CANManager::LOG_INFO,
                            LOG_TAG,
                            "Registered BattMonitor Node %d on Bus %d\n",
                            node_id,
                            ap_uavcan->get_driver_index());
            return batmon;
        }
    }
    return nullptr;
}

void AP_BattMonitor_UAVCAN::handle_smart_battery_info(const SmartBattInfoCb &cb)
{
    WITH_SEMAPHORE(_sem_battmon);

    _interim_state.info.capacity_initial_mah = cb.msg->capacity_initial;
    _interim_state.info.capacity_current_mah = cb.msg->capacity_current;
    _interim_state.info.cycles = cb.msg->cycles;

    memset(_interim_state.info.serial_num, 0, ARRAY_SIZE(_interim_state.info.serial_num));
    memcpy(_interim_state.info.serial_num, cb.msg->serial_num.begin(), MIN(ARRAY_SIZE(_interim_state.info.serial_num), cb.msg->serial_num.size()));

    memset(_interim_state.info.model_name, 0, ARRAY_SIZE(_interim_state.info.model_name));
    memcpy(_interim_state.info.model_name, cb.msg->model_name.begin(), MIN(ARRAY_SIZE(_interim_state.info.model_name), cb.msg->model_name.size()));

    memset(_interim_state.info.firmware_id, 0, ARRAY_SIZE(_interim_state.info.firmware_id));
    memcpy(_interim_state.info.firmware_id, cb.msg->firmware_id.begin(), MIN(ARRAY_SIZE(_interim_state.info.firmware_id), cb.msg->firmware_id.size()));

    _interim_state.info.lifetime_pct = cb.msg->lifetime;
    _interim_state.info.made_year = cb.msg->made_year;
    _interim_state.info.info_unk1 = cb.msg->info_unk1;
    _interim_state.info.info_unk2 = cb.msg->info_unk2;
    _interim_state.info.info_unk3 = cb.msg->info_unk3;
    _interim_state.info.info_serial = cb.msg->info_serial;

    memset(_interim_state.info.info_unk4, 0, ARRAY_SIZE(_interim_state.info.info_unk4));
    memcpy(_interim_state.info.info_unk4, cb.msg->info_unk4.begin(), MIN(ARRAY_SIZE(_interim_state.info.info_unk4), cb.msg->info_unk4.size()));

    memset(_interim_state.info.info_unk5, 0, ARRAY_SIZE(_interim_state.info.info_unk5));
    memcpy(_interim_state.info.info_unk5, cb.msg->info_unk5.begin(), MIN(ARRAY_SIZE(_interim_state.info.info_unk5), cb.msg->info_unk5.size()));

    _interim_state.info.version_unk1 = cb.msg->version_unk1;

    memset(_interim_state.info.version_unk2, 0, ARRAY_SIZE(_interim_state.info.version_unk2));
    memcpy(_interim_state.info.version_unk2, cb.msg->version_unk2.begin(), MIN(ARRAY_SIZE(_interim_state.info.version_unk2), cb.msg->version_unk2.size()));
}


void AP_BattMonitor_UAVCAN::handle_smart_battery_status(const SmartBattStatusCb &cb)
{
    WITH_SEMAPHORE(_sem_battmon);
    _interim_state.battery_id = cb.msg->battery_id;
    _interim_state.voltage = (float)(cb.msg->Vpack / 1000.0);
    _interim_state.current_amps = (float)(cb.msg->current1 / 1000.0);
    _interim_state.current2_amps = (float)(cb.msg->current2 / 1000.0);
    _interim_state.capacity_pct = cb.msg->state_of_charge_pct;

    for (size_t cell = 0; cell < cb.msg->Vcell.size(); ++cell) {
        _interim_state.cell_voltages.cells[cell] = cb.msg->Vcell[cell];
    }
    for (size_t cell = cb.msg->Vcell.size(); cell < ARRAY_SIZE(_interim_state.cell_voltages.cells); ++cell) {
        _interim_state.cell_voltages.cells[cell] = UINT16_MAX;
    }
    _has_cell_voltages = true;
    
    _interim_state.is_powering_off = cb.msg->is_shutting_down;
    _interim_state.is_on = cb.msg->is_on;

    _interim_state.info.unk1 = cb.msg->unk1;
    _interim_state.info.unk2 = cb.msg->unk2;
    _interim_state.info.unk3 = cb.msg->unk3;
    _interim_state.info.unk4 = cb.msg->unk4;
    _interim_state.info.unk5 = cb.msg->unk5;

    if (!isnanf(cb.msg->temperature) && cb.msg->temperature > 0) {
        // Temperature reported from battery in kelvin and stored internally in Celsius.
        _interim_state.temperature = KELVIN_TO_C(cb.msg->temperature / 10.0);
        _interim_state.temperature_time = AP_HAL::millis();
    }

    _interim_state.last_time_micros = AP_HAL::micros();

    _interim_state.healthy = true;
}

void AP_BattMonitor_UAVCAN::report_charging_status(const BattInfoCb &cb)
{
    static uint16_t last_status_flags[AP_BATT_MONITOR_MAX_INSTANCES] = { 0 };
    const char *charging_debug_message = nullptr;
    bool healthy;
    if (cb.msg->status_flags & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_IN_USE) {
        charging_debug_message = "IN_USE";
        _interim_state.charge_state = MAV_BATTERY_CHARGE_STATE_OK;
        healthy = true;
    } else {
        healthy = false;
    }
    if (cb.msg->status_flags & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_CHARGING) {
        charging_debug_message = "CHARGING";
        _interim_state.charge_state = MAV_BATTERY_CHARGE_STATE_CHARGING;
    }
    if (cb.msg->status_flags & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_CHARGED) {
        charging_debug_message = "CHARGED";
        _interim_state.charge_state = MAV_BATTERY_CHARGE_STATE_CHARGING;
    }
    if (cb.msg->status_flags & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_NEED_SERVICE) {
        if (!(last_status_flags[_instance_number] & uavcan::equipment::power::BatteryInfo::STATUS_FLAG_NEED_SERVICE)) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Battery %u need service!", _instance_number);
        }
        if (!AP::motors()->armed()) {
            healthy = false;
        }
    }
    if (cb.msg->status_flags & (uavcan::equipment::power::BatteryInfo::STATUS_FLAG_BMS_ERROR |
                                uavcan::equipment::power::BatteryInfo::STATUS_FLAG_TEMP_HOT |
                                uavcan::equipment::power::BatteryInfo::STATUS_FLAG_TEMP_COLD |
                                uavcan::equipment::power::BatteryInfo::STATUS_FLAG_OVERLOAD |
                                uavcan::equipment::power::BatteryInfo::STATUS_FLAG_BAD_BATTERY)) {
        _interim_state.charge_state = MAV_BATTERY_CHARGE_STATE_UNHEALTHY;
        if (last_status_flags[_instance_number] != cb.msg->status_flags) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "battery %u status: %u", cb.msg->status_flags, _instance_number);
        }
        healthy = false;
    }
    _interim_state.healthy = healthy;
    if ((!healthy) && AP::motors()->armed()) {
        AP::vehicle()->set_battery_failure();
    }
    if (last_status_flags[_instance_number] != cb.msg->status_flags) {
        last_status_flags[_instance_number] = cb.msg->status_flags;
        if (charging_debug_message != nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "battery %u status: %s", _instance_number, charging_debug_message);
        }
    }
}

void AP_BattMonitor_UAVCAN::handle_battery_info(const BattInfoCb &cb)
{
    update_interim_state(cb.msg->voltage, cb.msg->current, cb.msg->temperature, cb.msg->state_of_charge_pct, cb.msg->remaining_capacity_wh, cb.msg->average_power_10sec); 

    WITH_SEMAPHORE(_sem_battmon);
    _remaining_capacity_wh = cb.msg->remaining_capacity_wh;
    _full_charge_capacity_wh = cb.msg->full_charge_capacity_wh;

    report_charging_status(cb);
}

void AP_BattMonitor_UAVCAN::handle_battery_info_extended(const BattInfoExCb &cb)
{
    const struct log_BIX pkt{
        LOG_PACKET_HEADER_INIT(LOG_BIX_MSG),
        time_us                 : AP_HAL::micros64(),
        instance                : get_batt_log_id(),
        charge_intergrator_ah   : cb.msg->charge_intergrator_ah,
        cfet_temp_dc            : cb.msg->cfet_temp_dc,
        dfet_temp_dc            : cb.msg->dfet_temp_dc,
        pcb_temp_bq_dc          : cb.msg->pcb_temp_bq_dc,
        rear_cell_temp_dc       : cb.msg->rear_cell_temp_dc,
        cell_board_temp_dc      : cb.msg->cell_board_temp_dc,
        front_cell_temp_dc      : cb.msg->front_cell_temp_dc,
        load_voltage_v          : cb.msg->load_voltage_v,
        charger_voltage_v       : cb.msg->charger_voltage_v,
        non_clamped_soc         : cb.msg->non_clamped_soc,
        active_faults           : cb.msg->active_faults,
        balanced_cells          : cb.msg->balanced_cells,
        lifetime_charge_ah      : cb.msg->lifetime_charge_ah
    };
    AP::logger().WriteBlock(&pkt, sizeof(pkt));

    WITH_SEMAPHORE(_sem_battmon);
    uint8_t cell_count = MIN(ARRAY_SIZE(_interim_state.cell_voltages.cells), cb.msg->cell_voltage_v.size());
    for (uint8_t i = 0; i < cell_count; i++) {
        _interim_state.cell_voltages.cells[i] = cb.msg->cell_voltage_v[i] * 1000;
    }
    _has_cell_voltages = true;
}

void AP_BattMonitor_UAVCAN::update_interim_state(const float voltage, const float current, const float temperature_K, const uint8_t soc)
{
    WITH_SEMAPHORE(_sem_battmon);

    _interim_state.voltage = voltage;
    _interim_state.current_amps = _curr_mult * current;
    _interim_state.capacity_pct = soc;

    if (!isnanf(temperature_K) && temperature_K > 0) {
        // Temperature reported from battery in kelvin and stored internally in Celsius.
        _interim_state.temperature = KELVIN_TO_C(temperature_K);
        _interim_state.temperature_time = AP_HAL::millis();
    }

    const uint32_t tnow = AP_HAL::micros();

    if (!_has_battery_info_aux || _mppt.is_detected) {
        const uint32_t dt_us = tnow - _interim_state.last_time_micros;

        // update total current drawn since startup
        update_consumed(_interim_state, dt_us);
    }

    // record time
    _interim_state.last_time_micros = tnow;
}

void AP_BattMonitor_UAVCAN::update_interim_state(const float voltage, const float current, const float temperature_K, const uint8_t soc, const float remaining_capacity_wh, const float average_power_10sec)
{
    _interim_state.remaining_capacity_wh = remaining_capacity_wh;
    _interim_state.average_power_10sec = average_power_10sec;
    update_interim_state(voltage, current, temperature_K, soc);
}

void AP_BattMonitor_UAVCAN::handle_battery_info_aux(const BattInfoAuxCb &cb)
{
    WITH_SEMAPHORE(_sem_battmon);
    uint8_t cell_count = MIN(ARRAY_SIZE(_interim_state.cell_voltages.cells), cb.msg->voltage_cell.size());
    float remaining_capacity_ah = _remaining_capacity_wh / cb.msg->nominal_voltage;
    float full_charge_capacity_ah = _full_charge_capacity_wh / cb.msg->nominal_voltage;

    _cycle_count = cb.msg->cycle_count;
    for (uint8_t i = 0; i < cell_count; i++) {
        _interim_state.cell_voltages.cells[i] = cb.msg->voltage_cell[i] * 1000;
    }
    _interim_state.is_powering_off = cb.msg->is_powering_off;
    _interim_state.consumed_mah = (full_charge_capacity_ah - remaining_capacity_ah) * 1000;
    _interim_state.consumed_wh = _full_charge_capacity_wh - _remaining_capacity_wh;
    _interim_state.time_remaining =  is_zero(_interim_state.current_amps) ? 0 : (remaining_capacity_ah / _interim_state.current_amps * 3600);
    _interim_state.has_time_remaining = true;

    _has_cell_voltages = true;
    _has_time_remaining = true;
    _has_consumed_energy = true;
    _has_battery_info_aux = true;
}

void AP_BattMonitor_UAVCAN::handle_mppt_stream(const MpptStreamCb &cb)
{
    const bool use_input_value = (uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::MPPT_Use_Input_Value)) != 0;
    const float voltage = use_input_value ? cb.msg->input_voltage : cb.msg->output_voltage;
    const float current = use_input_value ? cb.msg->input_current : cb.msg->output_current;

    // use an invalid soc so we use the library calculated one
    const uint8_t soc = 127;

    // convert C to Kelvin
    const float temperature_K = isnanf(cb.msg->temperature) ? 0 : C_TO_KELVIN(cb.msg->temperature);

    update_interim_state(voltage, current, temperature_K, soc); 

    if (!_mppt.is_detected) {
        // this is the first time the mppt message has been received
        // so set powered up state
        _mppt.is_detected = true;
        mppt_set_bootup_powered_state();
    }

    mppt_check_and_report_faults(cb.msg->fault_flags);
}

void AP_BattMonitor_UAVCAN::handle_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, cb.msg->battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info(cb);
}

void AP_BattMonitor_UAVCAN::handle_battery_info_extended_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoExCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, 0);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info_extended(cb);
}

void AP_BattMonitor_UAVCAN::handle_smart_battery_info_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const SmartBattInfoCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, cb.msg->battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_smart_battery_info(cb);
}

void AP_BattMonitor_UAVCAN::handle_battery_info_aux_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const BattInfoAuxCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, cb.msg->battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_battery_info_aux(cb);
}

void AP_BattMonitor_UAVCAN::handle_mppt_stream_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const MpptStreamCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, node_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_mppt_stream(cb);
}

void AP_BattMonitor_UAVCAN::handle_smart_battery_status_trampoline(AP_UAVCAN* ap_uavcan, uint8_t node_id, const SmartBattStatusCb &cb)
{
    AP_BattMonitor_UAVCAN* driver = get_uavcan_backend(ap_uavcan, node_id, cb.msg->battery_id);
    if (driver == nullptr) {
        return;
    }
    driver->handle_smart_battery_status(cb);
}

// read - read the voltage and current
void AP_BattMonitor_UAVCAN::read()
{
    static uint32_t last_timeout_log[AP_BATT_MONITOR_MAX_INSTANCES] = { };
    uint32_t tnow = AP_HAL::micros();

    // timeout after 5 seconds
    if ((tnow - _interim_state.last_time_micros) > _params._batt_uavcan_timeout_secs * 1e6) {
        if ((tnow - last_timeout_log[_instance_number]) > _params._batt_uavcan_timeout_secs * 1e6) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "battery monitor %u uavcan timeout (%lu)",
                            _instance_number, (long unsigned int)(tnow - _interim_state.last_time_micros));
            last_timeout_log[_instance_number] = tnow;
        }
        _interim_state.healthy = false;
        if (AP::motors()->armed()) {
            AP::vehicle()->set_battery_failure();
        }
    }
    // Copy over relevant states over to main state
    WITH_SEMAPHORE(_sem_battmon);
    _state.battery_id = _interim_state.battery_id;
    _state.temperature = _interim_state.temperature;
    _state.temperature_time = _interim_state.temperature_time;
    _state.voltage = _interim_state.voltage;
    _state.current_amps = _interim_state.current_amps;
    _state.current2_amps = _interim_state.current2_amps;
    _state.consumed_mah = _interim_state.consumed_mah;
    _state.capacity_pct = _interim_state.capacity_pct;
    _state.consumed_wh = _interim_state.consumed_wh;
    _state.remaining_capacity_wh = _interim_state.remaining_capacity_wh;
    _state.is_powering_off = _interim_state.is_powering_off;
    _state.is_on = _interim_state.is_on;
    _state.last_time_micros = _interim_state.last_time_micros;
    _state.healthy = _interim_state.healthy;
    _state.time_remaining = _interim_state.time_remaining;
    _state.has_time_remaining = _interim_state.has_time_remaining;
    _state.is_powering_off = _interim_state.is_powering_off;
    memcpy(_state.cell_voltages.cells, _interim_state.cell_voltages.cells, sizeof(_state.cell_voltages));
    _state.charge_state = _interim_state.charge_state;
    _state.average_power_10sec = _interim_state.average_power_10sec;

    for (size_t cell = 0; cell < ARRAY_SIZE(_interim_state.cell_voltages.cells); ++cell) {
        _state.cell_voltages.cells[cell] = _interim_state.cell_voltages.cells[cell];
    }

    _has_temperature = (AP_HAL::millis() - _state.temperature_time) <= AP_BATT_MONITOR_TIMEOUT;

    // check if MPPT should be powered on/off depending upon arming state
    if (_mppt.is_detected) {
        mppt_set_armed_powered_state();
    }
    memcpy(&(_state.info), &(_interim_state.info), sizeof(_state.info));
}

/// capacity_remaining_pct - returns true if the percentage is valid and writes to percentage argument
bool AP_BattMonitor_UAVCAN::capacity_remaining_pct(uint8_t &percentage) const
{
    if ((uint32_t(_params._options.get()) & uint32_t(AP_BattMonitor_Params::Options::Ignore_UAVCAN_SoC)) ||
        _mppt.is_detected ||
        _state.capacity_pct > 100) {
        // a UAVCAN battery monitor may not be able to supply a state of charge. If it can't then
        // the user can set the option to use current integration in the backend instead.
        return AP_BattMonitor_Backend::capacity_remaining_pct(percentage);
    }

    percentage = _state.capacity_pct;
    return true;
}

/// get_cycle_count - return true if cycle count can be provided and fills in cycles argument
bool AP_BattMonitor_UAVCAN::get_cycle_count(uint16_t &cycles) const
{
    if (_has_battery_info_aux) {
        cycles = _cycle_count;
        return true;
    }
    return false;
}

// request MPPT board to power on/off at boot as specified by BATT_OPTIONS
void AP_BattMonitor_UAVCAN::mppt_set_bootup_powered_state()
{
    const uint32_t options = uint32_t(_params._options.get());
    const bool on_at_boot = (options & uint32_t(AP_BattMonitor_Params::Options::MPPT_Power_On_At_Boot)) != 0;
    const bool off_at_boot = (options & uint32_t(AP_BattMonitor_Params::Options::MPPT_Power_Off_At_Boot)) != 0;

    if (on_at_boot) {
        mppt_set_powered_state(true, true);
    } else if (off_at_boot) {
        mppt_set_powered_state(false, true);
    }
}

// request MPPT board to power on/off depending upon vehicle arming state as specified by BATT_OPTIONS
void AP_BattMonitor_UAVCAN::mppt_set_armed_powered_state()
{
    // check if vehicle armed state has changed
    const bool vehicle_armed = hal.util->get_soft_armed();
    if (vehicle_armed == _mppt.vehicle_armed_last) {
        return;
    }
    _mppt.vehicle_armed_last = vehicle_armed;

    // check options for arming state change events
    const uint32_t options = uint32_t(_params._options.get());
    const bool power_on_at_arm = (options & uint32_t(AP_BattMonitor_Params::Options::MPPT_Power_On_At_Arm)) != 0;
    const bool power_off_at_disarm = (options & uint32_t(AP_BattMonitor_Params::Options::MPPT_Power_Off_At_Disarm)) != 0;

    if (vehicle_armed && power_on_at_arm) {
        mppt_set_powered_state(true, false);
    } else if (!vehicle_armed && power_off_at_disarm) {
        mppt_set_powered_state(false, false);
    }
}

// request MPPT board to power on or off
// power_on should be true to power on the MPPT, false to power off
// force should be true to force sending the state change request to the MPPT
void AP_BattMonitor_UAVCAN::mppt_set_powered_state(bool power_on, bool force)
{
    if (_ap_uavcan == nullptr || _node == nullptr || !_mppt.is_detected) {
        return;
    }

    // return immediately if already desired state and not forced
    if ((_mppt.powered_state == power_on) && !force) {
        return;
    }
    _mppt.powered_state = power_on;
    _mppt.powered_state_changed = true;

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: powering %s", (unsigned)_instance+1, _mppt.powered_state ? "ON" : "OFF");

    mppt::OutputEnable::Request request;
    request.enable = _mppt.powered_state;
    request.disable = !request.enable;

    uavcan::ServiceClient<mppt::OutputEnable> client(*_node);
    client.setCallback([](const uavcan::ServiceCallResult<mppt::OutputEnable>& handle_mppt_enable_output_response){});
    client.call(_node_id, request);
}

// report changes in MPPT faults
void AP_BattMonitor_UAVCAN::mppt_check_and_report_faults(uint8_t fault_flags)
{
    // return immediately if no changes
    if (_mppt.fault_flags == fault_flags) {
        return;
    }
    _mppt.fault_flags = fault_flags;

    // handle recovery
    if (_mppt.fault_flags == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: OK", (unsigned)_instance+1);
        return;
    }

    // send battery faults via text messages
    for (uint8_t fault_bit=0x01; fault_bit <= 0x08; fault_bit <<= 1) {
        // this loop is to generate multiple messages if there are multiple concurrent faults, but also run once if there are no faults
        if ((fault_bit & fault_flags) != 0) {
            const MPPT_FaultFlags err = (MPPT_FaultFlags)fault_bit;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Battery %u: %s", (unsigned)_instance+1, mppt_fault_string(err));
        }
    }
}

// returns string description of MPPT fault bit. Only handles single bit faults
const char* AP_BattMonitor_UAVCAN::mppt_fault_string(MPPT_FaultFlags fault)
{
    switch (fault) {
        case MPPT_FaultFlags::OVER_VOLTAGE:
            return "over voltage";
        case MPPT_FaultFlags::UNDER_VOLTAGE:
            return "under voltage";
        case MPPT_FaultFlags::OVER_CURRENT:
            return "over current";
        case MPPT_FaultFlags::OVER_TEMPERATURE:
            return "over temp";
    }
    return "unknown";
}

// return mavlink fault bitmask (see MAV_BATTERY_FAULT enum)
uint32_t AP_BattMonitor_UAVCAN::get_mavlink_fault_bitmask() const
{
    // return immediately if not mppt or no faults
    if (!_mppt.is_detected || (_mppt.fault_flags == 0)) {
        return 0;
    }

    // convert mppt fault bitmask to mavlink fault bitmask
    uint32_t mav_fault_bitmask = 0;
    if ((_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_VOLTAGE) || (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::UNDER_VOLTAGE)) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_INCOMPATIBLE_VOLTAGE;
    }
    if (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_CURRENT) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_OVER_CURRENT;
    }
    if (_mppt.fault_flags & (uint8_t)MPPT_FaultFlags::OVER_TEMPERATURE) {
        mav_fault_bitmask |= MAV_BATTERY_FAULT_OVER_TEMPERATURE;
    }
    return mav_fault_bitmask;
}

#endif
