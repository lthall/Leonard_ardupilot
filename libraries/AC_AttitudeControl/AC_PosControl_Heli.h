#pragma once

#include "AC_PosControl.h"
#include <AP_Motors/AP_MotorsHeli.h>

#define AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX   0.95f    // Heli's use 95% of max collective before limiting frame angle

class AC_PosControl_Heli : public AC_PosControl {
public:

    /// Constructor
    AC_PosControl_Heli(AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       AP_MotorsHeli& motors,
                       AC_AttitudeControl& attitude_control,
                       float dt):
        AC_PosControl(ahrs,inav,motors,attitude_control,dt)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    // Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
    float get_althold_lean_angle_max_cd() const override { return 0; }

    // get_actuator_accel_target - convert aircraft current attitude and actuator settings to an expected acceleration
    virtual Vector2f get_actuator_accel_target_xy() const override { Vector2f var2f; return var2f; }

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

};
