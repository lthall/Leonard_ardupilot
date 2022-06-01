#ifdef ENABLE_SCRIPTING

#include "AC_AttitudeControl_Multi_6DoF.h"
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

// 6DoF control is extracted from the existing copter code by treating desired angles as thrust angles rather than vehicle attitude.
// Vehicle attitude is then set separately, typically the vehicle would matain 0 roll and pitch.
// rate commands result in the vehicle behaving as a ordinary copter.

/*
    override all input to the attitude controller and convert desired angles into thrust angles and substitute
*/

// Command an euler roll and pitch angle and an euler yaw rate with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_euler_rate_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_rate_cds) {

    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);

    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_euler_rate_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_rate_cds);
}

// Command an euler roll, pitch and yaw angle with angular velocity feedforward and smoothing
void AC_AttitudeControl_Multi_6DoF::input_euler_angle_roll_pitch_yaw(float euler_roll_angle_cd, float euler_pitch_angle_cd, float euler_yaw_angle_cd, bool slew_yaw) {

    set_forward_lateral(euler_pitch_angle_cd, euler_roll_angle_cd);

    AC_AttitudeControl_Multi::input_euler_angle_roll_pitch_yaw(euler_roll_angle_cd, euler_pitch_angle_cd, euler_yaw_angle_cd, slew_yaw);
}

void AC_AttitudeControl_Multi_6DoF::set_forward_lateral(float &euler_pitch_angle_cd, float &euler_roll_angle_cd)
{
    // pitch/forward
    if (forward_enable) {
        _motors.set_forward(-sinf(radians(euler_pitch_angle_cd * 0.01f)));
        euler_pitch_angle_cd = pitch_offset_deg * 100.0f;
    } else {
        _motors.set_forward(0.0f);
        euler_pitch_angle_cd += pitch_offset_deg * 100.0f;
    }
    euler_pitch_angle_cd = wrap_180_cd(euler_pitch_angle_cd);

    // roll/lateral
    if (lateral_enable) {
        _motors.set_lateral(sinf(radians(euler_roll_angle_cd * 0.01f)));
        euler_roll_angle_cd = roll_offset_deg * 100.0f;
    } else {
        _motors.set_lateral(0.0f);
        euler_roll_angle_cd += roll_offset_deg * 100.0f;
    }
    euler_roll_angle_cd = wrap_180_cd(euler_roll_angle_cd);
}

/*
    all other input functions should zero thrust vectoring
*/



AC_AttitudeControl_Multi_6DoF *AC_AttitudeControl_Multi_6DoF::_singleton = nullptr;

#endif // ENABLE_SCRIPTING
