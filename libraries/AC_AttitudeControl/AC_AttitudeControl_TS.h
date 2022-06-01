#pragma once

/// @file    AC_AttitudeControl_TVBS.h
/// @brief   ArduCopter attitude control library

#include "AC_AttitudeControl_Multi.h"


class AC_AttitudeControl_TS : public AC_AttitudeControl_Multi
{
public:
    AC_AttitudeControl_TS(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt);

    // empty destructor to suppress compiler warning
    virtual ~AC_AttitudeControl_TS() {}
};
