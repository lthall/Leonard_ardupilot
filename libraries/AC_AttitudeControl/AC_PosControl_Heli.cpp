

#include "AC_PosControl_Heli.h"

// table of user settable parameters
const AP_Param::GroupInfo AC_PosControl_Heli::var_info[] = {
    // parameters from parent vehicle
    AP_NESTEDGROUPINFO(AC_PosControl, 0),

    AP_GROUPEND
};

//
// throttle functions
//

void AC_PosControl_Heli::set_throttle_out(float throttle_in, bool apply_angle_boost, float filter_cutoff)
{
    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    _motors.set_throttle(throttle_in);
    // Clear angle_boost for logging purposes
    _angle_boost = 0.0f;
}

// Update Alt_Hold angle maximum
void AC_PosControl_Heli::update_althold_lean_angle_max(float throttle_in)
{
    float althold_lean_angle_max = acosf(constrain_float(throttle_in / AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX, 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}
