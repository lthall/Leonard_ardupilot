

#include "AC_PosControl_Heli.h"

// vibration compensation gains
#define POSCONTROL_VIBE_COMP_P_GAIN 0.250f
#define POSCONTROL_VIBE_COMP_I_GAIN 0.125f

# define POSCONTROL_CONTROL_ANGLE_LIMIT_MIN    10.0    // Min lean angle so that vehicle can maintain limited control

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

    if (strcmp(_motors_heli._get_frame(), "HELI_COMPOUND") == 0) {
        if (use_ff_collective) {
            // smoothly set collective to forward flight collective
            throttle_in = _motors_heli.get_fwd_flt_coll();
            filter_cutoff = 0.5f;
        }
    }

    _throttle_in = throttle_in;
    update_althold_lean_angle_max(throttle_in);
    _motors.set_throttle_filter_cutoff(filter_cutoff);
    _motors.set_throttle(throttle_in);
    // Clear angle_boost for logging purposes
    _angle_boost = 0.0f;
}

/// update_xy_controller - runs the horizontal position controller correcting position, velocity and acceleration errors.
///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
///     Desired velocity and accelerations are added to these corrections as they are calculated
///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
void AC_PosControl_Heli::update_xy_controller()
{
    AC_PosControl::update_xy_controller();

    // update angle targets that will be passed to stabilize controller
    accel_to_lean_angles(_accel_target.x, _accel_target.y, _roll_target, _pitch_target);
}

void AC_PosControl_Heli::relax_z_controller(float throttle_setting)
{
    AC_PosControl::relax_z_controller(0.0f);

    // init_z_controller has set the accel PID I term to generate the current throttle set point
    // Use relax_integrator to decay the throttle set point to throttle_setting
    _pid_accel_z.relax_integrator((throttle_setting - _motors.get_throttle_hover()) * 1000.0f, POSCONTROL_RELAX_TC);
}

void AC_PosControl_Heli::init_z_controller()
{
    AC_PosControl::init_z_controller();

    _pid_accel_z.reset_filter();

    // Set accel PID I term based on the current throttle
    // Remove the expected P term due to _accel_desired.z being constrained to _accel_max_z_cmss
    // Remove the expected FF term due to non-zero _accel_target.z
    _pid_accel_z.set_integrator((get_throttle_in() - _motors.get_throttle_hover()) * 1000.0f
        - _pid_accel_z.kP() * (_accel_target.z - get_z_accel_cmss())
        - _pid_accel_z.ff() * _accel_target.z);
}

void AC_PosControl_Heli::update_z_controller()
{
    AC_PosControl::update_z_controller();

    // Acceleration Controller

    // Calculate vertical acceleration
    const float z_accel_meas = get_z_accel_cmss();

    // ensure imax is always large enough to overpower hover throttle
    if (_motors.get_throttle_hover() * 1000.0f > _pid_accel_z.imax()) {
        _pid_accel_z.imax(_motors.get_throttle_hover() * 1000.0f);
    }
    float thr_out;
    if (_vibe_comp_enabled) {
        thr_out = get_throttle_with_vibration_override();
    } else {
        thr_out = _pid_accel_z.update_all(_accel_target.z, z_accel_meas, (_motors.limit.throttle_lower || _motors.limit.throttle_upper)) * 0.001f;
        thr_out += _pid_accel_z.get_ff() * 0.001f;
    }
    thr_out += _motors.get_throttle_hover();

    // Actuator commands

    // send throttle to attitude controller with angle boost
    set_throttle_out(thr_out, true, POSCONTROL_THROTTLE_CUTOFF_FREQ_HZ);

    // Check for vertical controller health

    // _speed_down_cms is checked to be non-zero when set
    float error_ratio = _pid_vel_z.get_error() / _vel_max_down_cms;
    _vel_z_control_ratio += _dt * 0.1f * (0.5 - error_ratio);
    _vel_z_control_ratio = constrain_float(_vel_z_control_ratio, 0.0f, 2.0f);

    // set vertical component of the limit vector
    if (_motors.limit.throttle_upper) {
        _limit_vector.z = 1.0f;
    } else if (_motors.limit.throttle_lower) {
        _limit_vector.z = -1.0f;
    } else {
        _limit_vector.z = 0.0f;
    }
}

// get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain)
float AC_PosControl_Heli::get_throttle_with_vibration_override()
{
    const float thr_per_accelz_cmss = _motors.get_throttle_hover() / (GRAVITY_MSS * 100.0f);
    // during vibration compensation use feed forward with manually calculated gain
    // ToDo: clear pid_info P, I and D terms for logging
    if (!(_motors.limit.throttle_lower || _motors.limit.throttle_upper) || ((is_positive(_pid_accel_z.get_i()) && is_negative(_pid_vel_z.get_error())) || (is_negative(_pid_accel_z.get_i()) && is_positive(_pid_vel_z.get_error())))) {
        _pid_accel_z.set_integrator(_pid_accel_z.get_i() + _dt * thr_per_accelz_cmss * 1000.0f * _pid_vel_z.get_error() * _pid_vel_z.kP() * POSCONTROL_VIBE_COMP_I_GAIN);
    }
    return POSCONTROL_VIBE_COMP_P_GAIN * thr_per_accelz_cmss * _accel_target.z + _pid_accel_z.get_i() * 0.001f;
}

void AC_PosControl_Heli::standby_xyz_reset()
{
    AC_PosControl::standby_xyz_reset();

    // Set _pid_accel_z integrator to zero.
    _pid_accel_z.set_integrator(0.0f);
}

// get_actuator_accel_target - convert aircraft current attitude and actuator settings to an expected acceleration
Vector2f AC_PosControl_Heli::get_actuator_accel_target_xy() const
{
    Vector2f accel_target;
    // limit acceleration using maximum lean angles
    float angle_max = MIN(get_althold_lean_angle_max_cd(), get_lean_angle_max_cd());
    float accel_max = angle_to_accel(angle_max * 0.01) * 100.0;
    accel_target.limit_length(accel_max);
    return accel_target;
}

// Return tilt angle limit for pilot input that prioritises altitude hold over lean angle
float AC_PosControl_Heli::get_althold_lean_angle_max_cd() const
{
    // convert to centi-degrees for public interface
    return MAX(ToDeg(_althold_lean_angle_max), POSCONTROL_CONTROL_ANGLE_LIMIT_MIN) * 100.0f;
}

// lean_angles_to_accel_xy - convert roll, pitch lean target angles to NE frame accelerations in cm/s/s
// todo: this should be based on thrust vector attitude control
Vector2f AC_PosControl_Heli::lean_angles_to_accel_xy() const
{
    // rotate our roll, pitch angles into lat/lon frame
    Vector3f att_target_euler = _attitude_control.get_att_target_euler_rad();
    att_target_euler.z = _ahrs.yaw;
    Vector3f accel_cmss = lean_angles_to_accel(att_target_euler);

    return accel_cmss.xy();
}

// get_lean_angles_to_accel - convert roll, pitch lean angles to NE frame accelerations in cm/s/s
void AC_PosControl_Heli::accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const
{
    // rotate accelerations into body forward-right frame
    const float accel_forward = accel_x_cmss * _ahrs.cos_yaw() + accel_y_cmss * _ahrs.sin_yaw();
    const float accel_right = -accel_x_cmss * _ahrs.sin_yaw() + accel_y_cmss * _ahrs.cos_yaw();

    // update angle targets that will be passed to stabilize controller
    pitch_target = accel_to_angle(-accel_forward * 0.01) * 100;
    float cos_pitch_target = cosf(pitch_target * M_PI / 18000.0f);
    roll_target = accel_to_angle((accel_right * cos_pitch_target)*0.01) * 100;
}

// Update Alt_Hold angle maximum
void AC_PosControl_Heli::update_althold_lean_angle_max(float throttle_in)
{
    float althold_lean_angle_max = acosf(constrain_float(throttle_in / AC_ATTITUDE_HELI_ANGLE_LIMIT_THROTTLE_MAX, 0.0f, 1.0f));
    _althold_lean_angle_max = _althold_lean_angle_max + (_dt / (_dt + _angle_limit_tc)) * (althold_lean_angle_max - _althold_lean_angle_max);
}

void AC_PosControl_Heli::input_ned_accel_rate_heading(const Vector3f& ned_accel, float heading_rate_cds, bool slew_yaw)
{

    bool print_gcs = false;
    float pitch_cd = 0.0f;

    if (current_ff_flt_coll != use_ff_collective) {
        current_ff_flt_coll = use_ff_collective;
        print_gcs = true;
    }

    if (strcmp(_motors_heli._get_frame(), "HELI_COMPOUND") == 0) {

        // rotate accelerations into body forward-right frame
        float accel_forward = ned_accel.x * _ahrs.cos_yaw() + ned_accel.y * _ahrs.sin_yaw();
        float accel_right = -ned_accel.x * _ahrs.sin_yaw() + ned_accel.y * _ahrs.cos_yaw();

        // update angle targets that will be passed to stabilize controller
        float pitch_target = accel_to_angle(-accel_forward * 0.01) * 100;
        float cos_pitch_target = cosf(pitch_target * M_PI / 18000.0f);
        float roll_target = accel_to_angle((accel_right * cos_pitch_target)*0.01) * 100;

        float accel_x_target = constrain_float(accel_forward / AC_POSCON_HELI_COMPOUND_ACCEL_X_MAX, -1.0f, 1.0f);
        _motors_heli.set_forward(accel_x_target);
        if (use_ff_collective) {
            pitch_cd = constrain_float(_accel_target.z / 500.0f, -1.0f, 1.0f) * 3000.0f;
            pitch_cd_lpf.reset(pitch_cd);
        } else {
            // smoothly set pitch_cd to zero
            pitch_cd = pitch_cd_lpf.apply(0.0f, _dt);
        }
        _attitude_control.input_euler_angle_roll_pitch_euler_rate_yaw(roll_target, pitch_cd, heading_rate_cds);

    } else {
        _attitude_control.input_thrust_vector_rate_heading(ned_accel, heading_rate_cds, slew_yaw);
    }
    if (print_gcs) {
        gcs().send_text(MAV_SEVERITY_NOTICE,"use ff coll; %s pitch_cd: %f", (use_ff_collective)?"true ":"false ", pitch_cd);
        print_gcs = false;
    }



}
