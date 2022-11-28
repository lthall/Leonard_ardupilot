#pragma once

#include "AC_PosControl.h"
#include <AP_Motors/AP_MotorsMulticopter.h>

#define POSCONTROL_CONTROL_ANGLE_LIMIT_THROTTLE_MAX    0.8f    // Max throttle used to limit lean angle so that vehicle does not lose altitude

class AC_PosControl_Multi : public AC_PosControl {
public:

    /// Constructor
    AC_PosControl_Multi(AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       AP_MotorsMulticopter& motors,
                       AC_AttitudeControl& attitude_control,
                       float dt) :
        AC_PosControl(ahrs,inav,motors,attitude_control,dt),
        _motors_multi(motors)
        {
            AP_Param::setup_object_defaults(this, var_info);
        }

    // Set output throttle
    void set_throttle_out(float throttle_in, bool apply_angle_boost, float filt_cutoff) override;

    // calculate total body frame throttle required to produce the given earth frame throttle
    float get_throttle_boosted(float throttle_in);

    /// update_xy_controller - runs the horizontal position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_xy_controller();

    // relax_z_controller - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
    void relax_z_controller(float throttle_setting);

    // init_z_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function is private and contains all the shared z axis initialisation functions
    void init_z_controller();

    /// update_z_controller - runs the vertical position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_z_controller();

    // Update Alt_Hold angle maximum
    void update_althold_lean_angle_max(float throttle_in) override;

    // set desired throttle vs attitude mixing (actual mix is slewed towards this value over 1~2 seconds)
    //  low values favour pilot/autopilot throttle over attitude control, high values favour attitude control over throttle
    //  has no effect when throttle is above hover throttle
    void set_throttle_mix_min() override { _throttle_rpy_mix_desired = _thr_mix_min; }
    void set_throttle_mix_man() override { _throttle_rpy_mix_desired = _thr_mix_man; }
    void set_throttle_mix_max(float ratio) override;
    void set_throttle_mix_value(float value) override { _throttle_rpy_mix_desired = _throttle_rpy_mix = value; }
    float get_throttle_mix(void) const override { return _throttle_rpy_mix; }

    // are we producing min throttle?
    bool is_throttle_mix_min() const override { return (_throttle_rpy_mix < 1.25f * _thr_mix_min); }

    /// standby_xyz_reset - resets I terms and removes position error
    ///     This function will let Loiter and Alt Hold continue to operate
    ///     in the event that the flight controller is in control of the
    ///     aircraft when in standby.
    void standby_xyz_reset();

    void input_ned_accel_rate_heading(const Vector3f& thrust_vector, AC_AttitudeControl::HeadingCommand heading) override;

    // sanity check parameters.  should be called once before take-off
    void parameter_sanity_check() override;

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

    // get_actuator_accel_target - convert aircraft current attitude and actuator settings to an expected acceleration
    virtual Vector2f get_actuator_accel_target_xy() const override;

    // Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
    float get_althold_lean_angle_max_cd() const override;

protected:

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    Vector2f lean_angles_to_accel_xy() const;

    // update_throttle_rpy_mix - updates thr_low_comp value towards the target
    void update_throttle_rpy_mix();

    // get maximum value throttle can be raised to based on throttle vs attitude prioritisation
    float get_throttle_avg_max(float throttle_in);

    // get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain)
    float get_throttle_with_vibration_override();

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const;

    AP_MotorsMulticopter& _motors_multi;

    // Enable/Disable angle boost
    AP_Int8               _angle_boost_enabled;

    AP_Float              _thr_mix_man;     // throttle vs attitude control prioritisation used when using manual throttle (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_min;     // throttle vs attitude control prioritisation used when landing (higher values mean we prioritise attitude control over throttle)
    AP_Float              _thr_mix_max;     // throttle vs attitude control prioritisation used during active flight (higher values mean we prioritise attitude control over throttle)
};
