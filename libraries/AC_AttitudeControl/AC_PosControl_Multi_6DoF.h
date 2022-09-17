#pragma once

#include "AC_PosControl_Multi.h"

class AC_PosControl_Multi_6DoF : public AC_PosControl_Multi {
public:

    /// Constructor
    AC_PosControl_Multi_6DoF(AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       const class AP_Motors& motors,
                       AC_AttitudeControl& attitude_control,
                       float dt):
        AC_PosControl_Multi(ahrs,inav,motors,attitude_control,dt)
        {
        }


protected:

};
