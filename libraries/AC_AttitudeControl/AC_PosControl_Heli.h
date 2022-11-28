#pragma once

#include "AC_PosControl.h"
#include <AP_Motors/AP_MotorsHeli.h>

#define AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX   0.95f    // Heli's use 95% of max collective before limiting frame angle
#define AC_POSCON_HELI_COMPOUND_ACCEL_X_MAX       5.0f

class AC_PosControl_Heli : public AC_PosControl {
public:

    /// Constructor
    AC_PosControl_Heli(AP_AHRS_View& ahrs,
                       const AP_InertialNav& inav,
                       AP_MotorsHeli& motors,
                       AC_AttitudeControl& attitude_control,
                       float dt):
        AC_PosControl(ahrs,inav,motors,attitude_control,dt),
        _motors_heli(motors)
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

    // get_actuator_accel_target - convert aircraft current attitude and actuator settings to an expected acceleration
    virtual Vector2f get_actuator_accel_target_xy() const override;

    // Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
    float get_althold_lean_angle_max_cd() const override;

    /// standby_xyz_reset - resets I terms and removes position error
    ///     This function will let Loiter and Alt Hold continue to operate
    ///     in the event that the flight controller is in control of the
    ///     aircraft when in standby.
    void standby_xyz_reset();

    void input_ned_accel_rate_heading(const Vector3f& thrust_vector, Orientation heading) override;

    // Set use forward flight collective flag
    void set_use_ff_collective(bool ff_collective) { use_ff_collective = ff_collective; };

    // user settable parameters
    static const struct AP_Param::GroupInfo var_info[];

protected:

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    Vector2f lean_angles_to_accel_xy() const;

    // get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain)
    float get_throttle_with_vibration_override();

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const;

    bool use_ff_collective;  // true if the forward flight collective is set

    bool current_ff_flt_coll;  // holds current forward flight collective status

    LowPassFilterFloat pitch_cd_lpf;

    AP_MotorsHeli& _motors_heli;

};
