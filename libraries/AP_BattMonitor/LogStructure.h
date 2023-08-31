#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_BATTMONITOR \
    LOG_BAT_MSG, \
    LOG_BCL_MSG, \
    LOG_BIX_MSG

// @LoggerMessage: BAT
// @Description: Gathered battery data
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: Volt: measured voltage
// @Field: VoltR: estimated resting voltage
// @Field: Curr: measured current
// @Field: CurrTot: consumed Ah, current * time
// @Field: EnrgTot: consumed Wh, energy this battery has expended
// @Field: RemW: remaining Wh
// @Field: T: measured temperature
// @Field: Res: estimated battery resistance
// @Field: RemP: remaining percentage
struct PACKED log_BAT {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    voltage;
    float    voltage_resting;
    float    current_amps;
    float    current2_amps;
    float    current_total;
    float    consumed_wh;
    float    remaining_capacity_wh;
    int16_t  temperature; // degrees C * 100
    float    resistance;
    uint8_t  rem_percent;
    uint8_t  is_powering_off;
    uint8_t  is_on;
};

// @LoggerMessage: BCL
// @Description: Battery cell voltage information
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: Volt: battery voltage
// @Field: V1: first cell voltage
// @Field: V2: second cell voltage
// @Field: V3: third cell voltage
// @Field: V4: fourth cell voltage
// @Field: V5: fifth cell voltage
// @Field: V6: sixth cell voltage
// @Field: V7: seventh cell voltage
// @Field: V8: eighth cell voltage
// @Field: V9: ninth cell voltage
// @Field: V10: tenth cell voltage
// @Field: V11: eleventh cell voltage
// @Field: V12: twelfth cell voltage
struct PACKED log_BCL {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    voltage;
    uint16_t cell_voltages[12]; // the format does not support more than 12 cells, the remaining cells are reported in the BCL2 message
};

// @LoggerMessage: BIX
// @Description: Battery Info Extended
// @Field: TimeUS: Time since system startup
// @Field: Instance: battery instance number
// @Field: ChgI: charge_intergrator_ah
// @Field: CfT: cfet_temp_dc
// @Field: DT: dfet_temp_dc
// @Field: PT: pcb_temp_bq_dc
// @Field: RCT: rear_cell_temp_dc
// @Field: CBT: cell_board_temp_dc
// @Field: FCT: front_cell_temp_dc
// @Field: LoadV: load_voltage_v
// @Field: ChgV: charger_voltage_v
// @Field: NCSOC: non_clamped_soc
// @Field: AF: active_faults
// @Field: Blnc: balanced_cells
// @Field: LT: lifetime_charge_ah
struct PACKED log_BIX {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    charge_intergrator_ah;
    float    cfet_temp_dc;
    float    dfet_temp_dc;
    float    pcb_temp_bq_dc;
    float    rear_cell_temp_dc;
    float    cell_board_temp_dc;
    float    front_cell_temp_dc;
    float    load_voltage_v;
    float    charger_voltage_v;
    float    non_clamped_soc;
    uint64_t active_faults;
    uint16_t balanced_cells;
    float    lifetime_charge_ah;
};

#define LOG_STRUCTURE_FROM_BATTMONITOR        \
    { LOG_BAT_MSG, sizeof(log_BAT), \
        "BAT", "QBfffffffcfBBB", "TimeUS,Inst,V,VR,Cur,Cur2,CurTot,EnrgTot,RemW,T,Res,RemP,Shut,On", "s#vvAAaXXOw%--", "F-0000C00?00--" },  \
    { LOG_BCL_MSG, sizeof(log_BCL), \
        "BCL", "QBfHHHHHHHHHHHH", "TimeUS,Instance,Volt,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12", "s#vvvvvvvvvvvvv", "F-0CCCCCCCCCCCC" , true }, \
    { LOG_BIX_MSG, sizeof(log_BIX), \
        "BIX", "QBffffffffffQHf", "TimeUS,I,ChgI,CfT,DT,PT,RCT,CBT,FCT,LoadV,ChgV,NCSOC,AF,Blnc,LT", "s#aOOOOOOvv---a", "F-0??????00----" },
