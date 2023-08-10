#include "Copter.h"

#if MODE_GUIDED_NOGPS_ENABLED == ENABLED

/*
 * Init and run calls for guided_nogps flight mode
 */

// initialise guided_nogps controller
bool ModeGuidedNoGPS::init(bool ignore_checks)
{
    // start in angle control mode
    ModeGuided::angle_control_start();
    return true;
}

// guided_run - runs the guided controller
// should be called at 100hz or more
void ModeGuidedNoGPS::run()
{

    // call the correct auto controller

    switch (ModeGuided::guided_mode) {

    case SubMode::VelAccel:
        ModeGuided::accel_control_run_nogps();
        break;

    case SubMode::Angle:
        ModeGuided::angle_control_run();
        break;
        
    default:
        // this will only run once
        ModeGuided::angle_control_start();
        ModeGuided::angle_control_run();
        break;
    }
}

#endif
