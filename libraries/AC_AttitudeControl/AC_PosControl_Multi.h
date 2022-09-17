#pragma once

#include "AC_PosControl.h"

class AC_PosControl_Multi : public AC_PosControl {
public:

    /// Constructor
    AC_PosControl_Multi(AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       const class AP_Motors& motors,
                       AC_AttitudeControl& attitude_control,
                       float dt):
        AC_PosControl(ahrs,inav,motors,attitude_control,dt)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }


protected:

};
