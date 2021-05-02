#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include <AC_PID/AC_P.h>            // P library
#include <AC_PID/AC_PID.h>          // PID library
#include <AC_PID/AC_P_1D.h>         // P library (1-axis)
#include <AC_PID/AC_P_2D.h>         // P library (2-axis)
#include <AC_PID/AC_PI_2D.h>        // PI library (2-axis)
#include <AC_PID/AC_PID_Basic.h>    // PID library (1-axis)
#include <AC_PID/AC_PID_2D.h>       // PID library (2-axis)
#include <AP_InertialNav/AP_InertialNav.h>  // Inertial Navigation library
#include "AC_AttitudeControl.h"     // Attitude control library
#include <AP_Motors/AP_Motors.h>    // motors library
#include <AP_Vehicle/AP_Vehicle.h>  // common vehicle parameters


// position controller default definitions
#define POSCONTROL_ACCEL_XY                     100.0f  // default horizontal acceleration in cm/s/s.  This is overwritten by waypoint and loiter controllers
#define POSCONTROL_STOPPING_DIST_UP_MAX         300.0f  // max stopping distance (in cm) vertically while climbing
#define POSCONTROL_STOPPING_DIST_DOWN_MAX       200.0f  // max stopping distance (in cm) vertically while descending

#define POSCONTROL_SPEED                        500.0f  // default horizontal speed in cm/s
#define POSCONTROL_SPEED_DOWN                  -150.0f  // default descent rate in cm/s
#define POSCONTROL_SPEED_UP                     250.0f  // default climb rate in cm/s

#define POSCONTROL_ACCEL_Z                      250.0f  // default vertical acceleration in cm/s/s.

#define POSCONTROL_VEL_ERROR_CUTOFF_FREQ        4.0f    // low-pass filter on velocity error (unit: hz)
#define POSCONTROL_THROTTLE_CUTOFF_FREQ         2.0f    // low-pass filter on accel error (unit: hz)
#define POSCONTROL_ACCEL_FILTER_HZ              2.0f    // low-pass filter on acceleration (unit: hz)

#define POSCONTROL_Z_SHAPER_TC                  0.25f    // low-pass filter on acceleration (unit: hz)

#define POSCONTROL_OVERSPEED_GAIN_Z             2.0f    // gain controlling rate at which z-axis speed is brought back within SPEED_UP and SPEED_DOWN range

#define POSCONTROL_RELAX_TC                     0.16f

class AC_PosControl
{
public:

    /// Constructor
    AC_PosControl(AP_AHRS_View& ahrs, const AP_InertialNav& inav,
                  const AP_Motors& motors, AC_AttitudeControl& attitude_control, float dt);

    /// set_dt - sets time delta in seconds for all position controllers (i.e. 100hz = 0.01, 400hz = 0.0025)
    void set_dt(float delta_sec);

    /// get_dt - gets time delta in seconds for all position controllers
    float get_dt() const { return _dt; }


    ///
    /// Lateral position controller
    ///

    /// set_max_speed_accel_xy - set the maximum horizontal speed in cm/s and acceleration in cm/s/s
    void set_max_speed_accel_xy(float speed_cms, float accel_cmss);

    /// get_max_speed_xy - get the maximum horizontal speed in cm/s
    float get_max_speed_xy() const { return _vel_max_xy_cms; }

    /// get_max_accel_xy - get the maximum horizontal acceleration in cm/s/s
    float get_max_accel_xy() const { return _accel_max_xy_cms; }

    // set the maximum horizontal position error that will be allowed in the horizontal plane
    void set_pos_error_max_xy(float error_max) { _p_pos_xy.set_limits(error_max, _vel_max_xy_cms, _accel_max_xy_cms, 0.0f); }
    float get_pos_error_max_xy() { return _p_pos_xy.get_error_max(); }

    /// init_xy_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    void init_xy_controller();

    /// init_xy_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
    ///     This function should be used when the expected kinimatic path assumes a stationary initial condition but does not specify a specific starting position.
    ///     The starting position can be retrieved by getting the position target using get_pos_target() after calling this function.
    void init_xy_controller_stopping_point();

    // relax_velocity_controller_xy - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
    void relax_velocity_controller_xy();

    /// input_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by:
    ///         accel_max : maximum acceleration
    ///         tc : time constant
    ///     The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
    ///     The time constant also defines the time taken to achieve the maximum acceleration.
    ///     The time constant must be positive.
    ///     The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
    void input_vel_accel_xy(Vector3f& vel, const Vector3f& accel);

    /// input_pos_vel_xy calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by:
    ///         vel_max : maximum velocity
    ///         accel_max : maximum acceleration
    ///         tc : time constant
    ///     The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
    ///     The time constant also defines the time taken to achieve the maximum acceleration.
    ///     The time constant must be positive.
    ///     The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
    void input_pos_vel_accel_xy(Vector3f& pos, Vector3f& vel, const Vector3f& accel);

    // is_active_xy - returns true if the xy position controller has been run in the previous 5 loop times
    bool is_active_xy() const;

    /// update_xy_controller - runs the horizontal position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_xy_controller();

    ///
    /// Vertical position controller
    ///

    /// set_max_speed_accel_z - set the maximum horizontal speed in cm/s and acceleration in cm/s/s
    ///     speed_down can be positive or negative but will always be interpreted as a descent speed
    void set_max_speed_accel_z(float speed_down, float speed_up, float accel_cmss);

    /// get_max_speed_up_z - get the maximum vertical speed up in cm/s
    float get_max_speed_up_z() const { return _vel_max_up_cms; }

    /// get_max_speed_down_z - get the maximum vertical speed down in cm/s
    float get_max_speed_down_z() const { return _vel_max_down_cms; }

    /// get_max_accel_z - get the maximum vertical acceleration in cm/s/s
    float get_max_accel_z() const { return _accel_max_z_cms; }

    // set_pos_error_max_z - set the maximum vertical position error that will be allowed
    void set_pos_error_max_z(float error_min, float error_max) { _p_pos_z.set_limits(error_min, error_max, -fabsf(_vel_max_down_cms), _vel_max_up_cms, _accel_max_z_cms, 0.0f); }

    // get_pos_error_max_z - get the maximum vertical position error that will be allowed
    float get_pos_error_max_z() { return _p_pos_z.get_error_max(); }
    float get_pos_error_min_z() { return _p_pos_z.get_error_min(); }

    /// get_max_speed_up - accessors for current maximum up speed in cm/s
    float get_max_speed_up() const { return _vel_max_up_cms; }

    /// get_max_speed_down - accessors for current maximum down speed in cm/s.  Will be a negative number
    float get_max_speed_down() const { return _vel_max_down_cms; }

    /// init_z_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    void init_z_controller();

    /// init_z_controller - initialise the position controller to the current position, velocity, acceleration and attitude.
    ///     This function is the default initialisation for any position control that provides position, velocity and acceleration.
    ///     This function does not allow any negative velocity or acceleration
    void init_z_controller_no_descent();

    /// init_z_controller_stopping_point - initialise the position controller to the stopping point with zero velocity and acceleration.
    ///     This function should be used when the expected kinimatic path assumes a stationary initial condition but does not specify a specific starting position.
    ///     The starting position can be retrieved by getting the position target using get_pos_target() after calling this function.
    void init_z_controller_stopping_point();

    // relax_z_controller - initialise the position controller to the current position and velocity with decaying acceleration.
    ///     This function decays the output acceleration by 95% every half second to achieve a smooth transition to zero requested acceleration.
    void relax_z_controller(float throttle_setting);

    /// input_vel_z calculate a jerk limited path from the current position, velocity and acceleration to an input velocity.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by:
    ///         accel_max : maximum acceleration
    ///         tc : time constant
    ///     The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
    ///     The time constant also defines the time taken to achieve the maximum acceleration.
    ///     The time constant must be positive.
    ///     The function alters the input velocity to be the velocity that the system could reach zero acceleration in the minimum time.
    virtual void input_vel_accel_z(Vector3f& vel, const Vector3f& accel, bool force_descend);

    /// set_alt_target_from_climb_rate_ff - adjusts target up or down using a climb rate in cm/s
    ///     using the default position control kinimatic path.
    void set_alt_target_from_climb_rate(const float& vel, bool force_descend);

    /// input_pos_vel_z calculate a jerk limited path from the current position, velocity and acceleration to an input position and velocity.
    ///     The function takes the current position, velocity, and acceleration and calculates the required jerk limited adjustment to the acceleration for the next time dt.
    ///     The kinematic path is constrained by:
    ///         vel_up_max, vel_down_max : maximum velocity
    ///         accel_max : maximum acceleration
    ///         tc : time constant
    ///     The time constant defines the acceleration error decay in the kinematic path as the system approaches constant acceleration.
    ///     The time constant also defines the time taken to achieve the maximum acceleration.
    ///     The time constant must be positive.
    ///     The function alters the input position to be the closest position that the system could reach zero acceleration in the minimum time.
    void input_pos_vel_accel_z(Vector3f& pos, Vector3f& vel, const Vector3f& accel);

    /// set_alt_target_with_slew - adjusts target up or down using a commanded altitude in cm
    ///     using the default position control kinimatic path.
    void set_alt_target_with_slew(const float& pos);

    // is_active_z - returns true if the z position controller has been run in the previous 5 loop times
    bool is_active_z() const;

    /// update_z_controller - runs the vertical position controller correcting position, velocity and acceleration errors.
    ///     Position and velocity errors are converted to velocity and acceleration targets using PID objects
    ///     Desired velocity and accelerations are added to these corrections as they are calculated
    ///     Kinematically consistent target position and desired velocity and accelerations should be provided before calling this function
    void update_z_controller();



    ///
    /// Assessors
    ///

    /// set commanded position, velocity and acceleration inputs when the path is created externally.
    void set_pos_vel_accel(const Vector3f& pos, const Vector3f& vel, const Vector3f& accel);
    void set_pos_vel_accel_xy(const Vector2f& pos, const Vector2f& vel, const Vector2f& accel);


    /// Position

    /// set_pos_target_xy in cm from home
    void set_pos_target_xy(float pos_x, float pos_y) {_pos_target.x = pos_x; _pos_target.y = pos_y; }

    /// get_pos_target - get target as position vector (from home in cm)
    const Vector3f& get_pos_target() const { return _pos_target; }

    /// set_pos_target_z - set altitude target in cm above home
    void set_pos_target_z(float pos_target) { _pos_target.z = pos_target; }

    /// get_pos_target_z - get desired altitude (in cm above home)
    // This should be used only for logging and telemetry
    float get_pos_target_z() const { return _pos_target.z; }

    /// get_stopping_point_xy - calculates stopping point based on current position, velocity, vehicle acceleration
    void get_stopping_point_xy(Vector3f &stopping_point) const;

    /// get_stopping_point_z - calculates stopping point based on current position, velocity, vehicle acceleration
    void get_stopping_point_z(Vector3f& stopping_point) const;

    /// get_pos_error - get position error vector between the current and target position
    const Vector3f get_pos_error() const { return _pos_target - _inav.get_position(); }

    /// get_pos_error_xy - get the length of the position error vector in the xy plane
    float get_pos_error_xy() const { return norm(_pos_target.x - _inav.get_position().x, _pos_target.y - _inav.get_position().y); }

    /// get_alt_error - returns altitude error in cm
    float get_pos_error_z() const { return (_pos_target.z - _inav.get_position().z); }


    /// Velocity

    /// set_vel_desired - sets desired velocity in cm/s in all 3 axis
    ///     when update_vel_controller_xyz is next called the position target is moved based on the desired velocity
    void set_vel_desired(const Vector3f &des_vel) { _vel_desired = des_vel; }

    /// get_vel_desired - returns xy desired velocity (i.e. feed forward) in cm/s in lat and lon direction
    const Vector3f& get_vel_desired() { return _vel_desired; }

    /// set_vel_desired_xy - sets desired velocity in cm/s in lat and lon directions
    ///     when update_xy_controller is next called the position target is moved based on the desired velocity and
    ///     the desired velocities are fed forward into the rate_to_accel step
    void set_vel_desired_xy(float vel_x, float vel_y) {_vel_desired.x = vel_x; _vel_desired.y = vel_y; }

    /// set_vel_desired_z - sets desired velocity in cm/s in z axis
    void set_vel_desired_z(float vel_z_cms) {_vel_desired.z = vel_z_cms;}

    /// get_vel_target_z - returns current vertical speed in cm/s
    // This should be used only for logging and telemetry
    float get_vel_target_z() const { return _vel_target.z; }

    /// get_lean_angle_max_cd - returns the maximum lean angle the autopilot may request
    float get_lean_angle_max_cd() const;


    /// Acceleration

    // set desired acceleration in cm/s in xy axis
    void set_accel_desired_xy(float accel_lat_cms, float accel_lon_cms) { _accel_desired.x = accel_lat_cms; _accel_desired.y = accel_lon_cms; }


    /// Outputs

    /// get desired roll and pitch to be passed to the attitude controller
    float get_roll() const { return _roll_target; }
    float get_pitch() const { return _pitch_target; }

    /// get desired yaw to be passed to the attitude controller
    float get_yaw_cd() const { return _yaw_target; }

    /// get desired yaw rate to be passed to the attitude controller
    float get_yaw_rate_cds() const { return _yaw_rate_target; }

    /// get desired roll and pitch to be passed to the attitude controller
    Vector3f get_thrust_vector() const;

    /// get_bearing_to_target - get bearing to target position in centi-degrees
    int32_t get_bearing_to_target() const;


    /// Other

    /// get pid controllers
    AC_P_2D& get_pos_xy_p() { return _p_pos_xy; }
    AC_P_1D& get_pos_z_p() { return _p_pos_z; }
    AC_PID_2D& get_vel_xy_pid() { return _pid_vel_xy; }
    AC_PID_Basic& get_vel_z_pid() { return _pid_vel_z; }
    AC_PID& get_accel_z_pid() { return _pid_accel_z; }

    /// set_limit_accel_xy - mark that accel has been limited
    ///     this prevents integrator buildup
    void set_limit_accel_xy(void) { _limit.accel_xy = true; }

    // overrides the velocity process variable for one timestep
    void override_vehicle_velocity_xy(const Vector2f& vel_xy) { _vehicle_horiz_vel = vel_xy; _flags.vehicle_horiz_vel_override = true; }

    /// accessors for reporting
    const Vector3f& get_vel_target() const { return _vel_target; }
    const Vector3f& get_accel_target() const { return _accel_target; }

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    Vector3f lean_angles_to_accel(const Vector3f& att_target_euler) const;

    void write_log();

    // provide feedback on whether arming would be a good idea right now:
    bool pre_arm_checks(const char *param_prefix,
                        char *failure_msg,
                        const uint8_t failure_msg_len);

    // enable or disable high vibration compensation
    void set_vibe_comp(bool on_off) { _vibe_comp_enabled = on_off; }

    /// get_vel_z_error_ratio - returns the proportion of error relative to the maximum request
    float get_vel_z_control_ratio() const { return constrain_float(_vel_z_control_ratio, 0.0f, 1.0f); }

    /// standby_xyz_reset - resets I terms and removes position error
    ///     This function will let Loiter and Alt Hold continue to operate
    ///     in the event that the flight controller is in control of the
    ///     aircraft when in standby.
    void standby_xyz_reset();

    static const struct AP_Param::GroupInfo var_info[];

protected:

    // general purpose flags
    struct poscontrol_flags {
            uint16_t vehicle_horiz_vel_override : 1; // 1 if we should use _vehicle_horiz_vel as our velocity process variable for one timestep
    } _flags;

    // limit flags structure
    struct poscontrol_limit_flags {
        bool pos_xy;        // true if we have hit a horizontal position limit
        bool pos_up;        // true if we have hit a vertical position limit while going up
        bool pos_down;      // true if we have hit a vertical position limit while going down
        bool accel_xy;      // true if we have hit the horizontal accel limit
        bool accel_up;      // true if we have hit the vertical velocity limit going up
        bool accel_down;    // true if we have hit the vertical velocity limit going down
    } _limit;

    /// init_xy - initialise the position controller to the current position, velocity and acceleration.
    ///     This function is private and contains all the shared xy axis intialisaion functions
    void init_xy();

    /// init_z - initialise the position controller to the current position, velocity and acceleration.
    ///     This function is private and contains all the shared z axis intialisaion functions
    void init_z();

    // get throttle using vibration-resistant calculation (uses feed forward with manually calculated gain)
    float get_throttle_with_vibration_override();

    // get earth-frame Z-axis acceleration with gravity removed in cm/s/s with +ve being up
    float get_z_accel_cmss() const { return -(_ahrs.get_accel_ef_blended().z + GRAVITY_MSS) * 100.0f; }

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void accel_to_lean_angles(float accel_x_cmss, float accel_y_cmss, float& roll_target, float& pitch_target) const;

    // lean_angles_to_accel - convert roll, pitch lean angles to lat/lon frame accelerations in cm/s/s
    void lean_angles_to_accel_xy(float& accel_x_cmss, float& accel_y_cmss) const;

    // calculate_yaw_and_rate_yaw - calculate the vehicle yaw and rate of yaw.
    bool calculate_yaw_and_rate_yaw();

    /// initialise and check for ekf position resets
    void init_ekf_xy_reset();
    void check_for_ekf_xy_reset();
    void init_ekf_z_reset();
    void check_for_ekf_z_reset();

    // references to inertial nav and ahrs libraries
    AP_AHRS_View&           _ahrs;
    const AP_InertialNav&   _inav;
    const AP_Motors&        _motors;
    AC_AttitudeControl&     _attitude_control;

    // parameters
    AP_Float        _lean_angle_max;    // Maximum autopilot commanded angle (in degrees). Set to zero for Angle Max
    AP_Float        _shaping_tc;        // This is the time constant used to determine how quickly the aircraft varies the acceleration target
    AC_P_2D         _p_pos_xy;          // XY axis position controller to convert distance error to desired velocity
    AC_P_1D         _p_pos_z;           // Z axis position controller to convert altitude error to desired climb rate
    AC_PID_2D       _pid_vel_xy;        // XY axis velocity controller to convert velocity error to desired acceleration
    AC_PID_Basic    _pid_vel_z;         // Z axis velocity controller to convert climb rate error to desired acceleration
    AC_PID          _pid_accel_z;       // Z axis acceleration controller to convert desired acceleration to throttle output

    // internal variables
    float       _dt;                    // time difference (in seconds) between calls from the main program
    uint64_t    _last_update_xy_us;     // system time (in microseconds) since last update_xy_controller call
    uint64_t    _last_update_z_us;      // system time (in microseconds) since last update_z_controller call
    float       _vel_max_down_cms;      // max descent rate in cm/s
    float       _vel_max_up_cms;        // max climb rate in cm/s
    float       _vel_max_xy_cms;        // max horizontal speed in cm/s
    float       _accel_max_z_cms;       // max vertical acceleration in cm/s/s
    float       _accel_max_xy_cms;      // max horizontal acceleration in cm/s/s
    float       _vel_z_control_ratio = 2.0f;    // confidence that we have control in the vertical axis

    // output from controller
    float       _roll_target;           // desired roll angle in centi-degrees calculated by position controller
    float       _pitch_target;          // desired roll pitch in centi-degrees calculated by position controller
    float       _yaw_target;            // desired yaw in centi-degrees calculated by position controller
    float       _yaw_rate_target;       // desired yaw rate in centi-degrees per second calculated by position controller

    // position controller internal variables
    Vector3f    _pos_target;            // target location in cm from home
    Vector3f    _vel_desired;           // desired velocity in cm/s
    Vector3f    _vel_target;            // velocity target in cm/s calculated by pos_to_rate step
    Vector3f    _vel_error;             // error between desired and actual acceleration in cm/s
    Vector3f    _accel_desired;         // desired acceleration in cm/s/s (feed forward)
    Vector3f    _accel_target;          // acceleration target in cm/s/s
    Vector3f    _accel_error;           // acceleration error in cm/s/s
    Vector2f    _vehicle_horiz_vel;     // velocity to use if _flags.vehicle_horiz_vel_override is set
    LowPassFilterFloat _vel_error_filter;   // low-pass-filter on z-axis velocity error

    // ekf reset handling
    uint32_t    _ekf_xy_reset_ms;      // system time of last recorded ekf xy position reset
    uint32_t    _ekf_z_reset_ms;       // system time of last recorded ekf altitude reset

    // high vibration handling
    bool        _vibe_comp_enabled;     // true when high vibration compensation is on
};
