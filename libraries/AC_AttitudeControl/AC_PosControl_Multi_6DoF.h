#pragma once

#include "AC_PosControl_Multi.h"
#include "AC_AttitudeControl_Multi_6DoF.h"

class AC_PosControl_Multi_6DoF : public AC_PosControl_Multi {
public:

    /// Constructor
    AC_PosControl_Multi_6DoF(AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       AP_MotorsMulticopter& motors,
                       AC_AttitudeControl& attitude_control,
                       float dt):
        AC_PosControl_Multi(ahrs,inav,motors,attitude_control,dt)
        {
        }


    // limiting lean angle based on throttle makes no sense for 6DoF, always allow 90 deg, return in centi-degrees
    float get_althold_lean_angle_max_cd() const override { return 9000.0f; }


protected:

};
