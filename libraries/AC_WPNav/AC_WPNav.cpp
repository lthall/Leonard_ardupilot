#include <AP_HAL/AP_HAL.h>
#include "AC_WPNav.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_WPNav::var_info[] = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 20 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 5 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 10 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WPNav, _wp_speed_up_cms, WPNAV_WP_SPEED_UP),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 10 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WPNav, _wp_speed_down_cms, WPNAV_WP_SPEED_DOWN),

    // @Param: ACCEL
    // @DisplayName: Waypoint Acceleration 
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL",       5, AC_WPNav, _wp_accel_cmss, WPNAV_ACCELERATION),

    // @Param: ACCEL_Z
    // @DisplayName: Waypoint Vertical Acceleration
    // @Description: Defines the vertical acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",     6, AC_WPNav, _wp_accel_z_cmss, WPNAV_WP_ACCEL_Z_DEFAULT),

    // @Param: RFND_USE
    // @DisplayName: Waypoint missions use rangefinder for terrain following
    // @Description: This controls if waypoint missions use rangefinder for terrain following
    // @Values: 0:Disable,1:Enable
    // @User: Advanced
    AP_GROUPINFO("RFND_USE",   10, AC_WPNav, _rangefinder_use, 1),

    // @Param: JERK
    // @DisplayName: Waypoint Jerk
    // @Description: Defines the horizontal jerk in m/s/s used during missions
    // @Units: m/s/s
    // @Range: 1 20
    // @User: Standard
    AP_GROUPINFO("JERK",   11, AC_WPNav, _wp_jerk, 1),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WPNav::AC_WPNav(const AP_InertialNav& inav, const AP_AHRS_View& ahrs, AC_PosControl& pos_control, const AC_AttitudeControl& attitude_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _attitude_control(attitude_control)
{
    AP_Param::setup_object_defaults(this, var_info);

    // init flags
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;
    _flags.segment_type = SEGMENT_STRAIGHT;

    // sanity check some parameters
    _wp_accel_cmss = MIN(_wp_accel_cmss, GRAVITY_MSS * 100.0f * tanf(ToRad(_attitude_control.lean_angle_max() * 0.01f)));
    _wp_radius_cm = MAX(_wp_radius_cm, WPNAV_WP_RADIUS_MIN);
}

// get expected source of terrain data if alt-above-terrain command is executed (used by Copter's ModeRTL)
AC_WPNav::TerrainSource AC_WPNav::get_terrain_source() const
{
    // use range finder if connected
    if (_rangefinder_available && _rangefinder_use) {
        return AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER;
    }
#if AP_TERRAIN_AVAILABLE
    if ((_terrain != nullptr) && _terrain->enabled()) {
        return AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE;
    } else {
        return AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE;
    }
#else
    return AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE;
#endif
}

///
/// waypoint navigation
///

/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void AC_WPNav::wp_and_spline_init()
{
    // check _wp_accel_cmss is reasonable
    if (_wp_accel_cmss <= 0) {
        _wp_accel_cmss.set_and_save(WPNAV_ACCELERATION);
    }

    // initialise position controller
    _pos_control.set_desired_accel_xy(0.0f,0.0f);
    _pos_control.init_xy_controller();

    // initialise feed forward velocity to zero
    _pos_control.set_desired_velocity_xy(0.0f, 0.0f);

    // initialise position controller speed and acceleration
    _pos_control.set_max_speed_xy(_wp_speed_cms);
    _pos_control.set_max_accel_xy(_wp_accel_cmss);
    _pos_control.set_max_speed_z(-_wp_speed_down_cms, _wp_speed_up_cms);
    _pos_control.set_max_accel_z(_wp_accel_z_cmss);
    _pos_control.calc_leash_length_xy();
    _pos_control.calc_leash_length_z();

    float jerk = MIN(_attitude_control.get_ang_vel_roll_max() * GRAVITY_MSS, _attitude_control.get_ang_vel_pitch_max() * GRAVITY_MSS);
    if (is_zero(jerk)) {
        jerk = _wp_jerk;
    } else {
        jerk = MIN(jerk, _wp_jerk);
    }
    float jounce = MIN(_attitude_control.get_accel_roll_max() * GRAVITY_MSS, _attitude_control.get_accel_pitch_max() * GRAVITY_MSS);

    // time constant may also be a useful parameter
    float tc;
    if (is_positive(jounce)) {
        tc = MAX(_attitude_control.get_input_tc(), 0.5 * jerk * M_PI / jounce);
    } else {
        tc = MAX(_attitude_control.get_input_tc(), 0.1f);
    }
//    float tc = MAX(0.25 * 2.0, 0.5 * jerk * M_PI / jounce);
    _scurve_last_leg = scurves(tc * 2.0, jerk * 100.0, _wp_accel_cmss, _wp_speed_cms);
    _scurve_this_leg = scurves(tc * 2.0, jerk * 100.0, _wp_accel_cmss, _wp_speed_cms);
    _scurve_next_leg = scurves(tc * 2.0, jerk * 100.0, _wp_accel_cmss, _wp_speed_cms);

    // initialise yaw heading to current heading target
    _flags.wp_yaw_set = false;

    _scurve_this_leg.Cal_Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    _scurve_last_leg.Cal_Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

/// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
void AC_WPNav::set_speed_xy(float speed_cms)
{
    // range check new target speed and update position controller
    if (speed_cms >= WPNAV_WP_SPEED_MIN) {
        _pos_control.set_max_speed_xy(speed_cms);
    }
}

/// set current target climb rate during wp navigation
void AC_WPNav::set_speed_up(float speed_up_cms)
{
    _pos_control.set_max_speed_z(_pos_control.get_max_speed_down(), speed_up_cms);
}

/// set current target descent rate during wp navigation
void AC_WPNav::set_speed_down(float speed_down_cms)
{
    _pos_control.set_max_speed_z(speed_down_cms, _pos_control.get_max_speed_up());
}

bool AC_WPNav::get_wp_destination_loc(Location& destination) const
{
    Vector3f dest = get_wp_destination();
    if (!AP::ahrs().get_origin(destination)) {
        return false;
    }
    destination.offset(dest.x*0.01f, dest.y*0.01f);
    destination.alt += dest.z;
    return true;
}

/// set_wp_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav::set_wp_destination_loc(const Location& destination)
{
    Vector3f destination_NEU;

    // convert destination location to vector
    if (!destination.get_vector_from_origin_NEU(destination_NEU)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_wp_destination(destination_NEU);
}

/// set_wp_destination waypoint using location class
///     provide the next_destination if known
///     returns false if conversion from location to vector from ekf origin cannot be calculated
bool AC_WPNav::set_wp_destination_loc_next(const Location& destination)
{
    Vector3f destination_NEU;

    // convert destination location to vector
    if (!destination.get_vector_from_origin_NEU(destination_NEU)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_wp_destination_next(destination_NEU);
}

/// set waypoint destination using NED position vector from ekf origin in meters
bool AC_WPNav::set_wp_destination_NED(const Vector3f& destination_NED)
{
    // convert NED to NEU and do not use terrain following
    return set_wp_destination(Vector3f(destination_NED.x * 100.0f, destination_NED.y * 100.0f, -destination_NED.z * 100.0f));
}

/// set waypoint destination using NED position vector from ekf origin in meters
bool AC_WPNav::set_wp_destination_NED(const Vector3f& destination_NED, const Vector3f& next_destination_NED)
{
    // convert NED to NEU and do not use terrain following
    if (!set_wp_destination(Vector3f(destination_NED.x * 100.0f, destination_NED.y * 100.0f, -destination_NED.z * 100.0f))) {
        return false;
    }
    return set_wp_destination_next(Vector3f(destination_NED.x * 100.0f, destination_NED.y * 100.0f, -destination_NED.z * 100.0f));
}

/// set_wp_destination - set destination waypoints using position vectors (distance from ekf origin in cm)
///     terrain_alt should be true if origin.z and destination.z are desired altitudes above terrain (false if these are alt-above-ekf-origin)
///     returns false on failure (likely caused by missing terrain data)
bool AC_WPNav::set_wp_destination(const Vector3f& destination)
{
    hal.console->printf("set_wp_destination \n");
    _origin = _destination;

    // initialise intermediate point to the origin
    _flags.reached_destination = false;
    _flags.wp_yaw_set = false;

    // store destination location
    _destination = destination;
    _scurve_last_leg = _scurve_this_leg;
    if (_flags.fast_waypoint) {
        _scurve_this_leg = _scurve_next_leg;
    } else {
        _scurve_this_leg.calculate_leg(_origin, _destination);
    }
    _flags.fast_waypoint = false;   // default waypoint back to slow
    _scurve_next_leg.Cal_Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    return true;
}

/// set next destination using position vector (distance from ekf origin in cm)
///     terrain_alt should be true if destination.z is a desired altitude above terrain
///     provide next_destination
bool AC_WPNav::set_wp_destination_next(const Vector3f& destination)
{
    hal.console->printf("set_wp_destination_next \n");

    _scurve_next_leg.calculate_leg(_destination, destination);

    // next destination provided so fast waypoint
    _flags.fast_waypoint = true;

    return true;
}

///
/// spline methods
///

/// set_spline_destination waypoint using location class
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     stopped_at_start should be set to true if vehicle is stopped at the origin
///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
bool AC_WPNav::set_spline_destination_loc(const Location& destination, Location next_destination, bool spline_next)
{
    // convert destination location to vector
    Vector3f destination_NEU;
    if (!destination.get_vector_from_origin_NEU(destination_NEU)) {
        return false;
    }
    Vector3f next_destination_NEU;
    if (!next_destination.get_vector_from_origin_NEU(next_destination_NEU)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_spline_destination(destination_NEU, next_destination_NEU, spline_next);
}

bool AC_WPNav::set_spline_destination_next_loc(const Location& destination, Location next_destination, bool spline_next)
{
    // convert destination location to vector
    Vector3f destination_NEU;
    if (!destination.get_vector_from_origin_NEU(destination_NEU)) {
        return false;
    }
    Vector3f next_destination_NEU;
    if (!next_destination.get_vector_from_origin_NEU(next_destination_NEU)) {
        return false;
    }

    // set target as vector from EKF origin
    return set_spline_destination_next(destination_NEU, next_destination_NEU, spline_next);
}

/// set_spline_destination waypoint using position vector (distance from home in cm)
///     returns false if conversion from location to vector from ekf origin cannot be calculated
///     terrain_alt should be true if destination.z is a desired altitudes above terrain (false if its desired altitudes above ekf origin)
///     stopped_at_start should be set to true if vehicle is stopped at the origin
///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
bool AC_WPNav::set_spline_destination(const Vector3f& destination, const Vector3f& next_destination, bool spline_next)
{
    hal.console->printf("set_spline_destination \n");

    Vector3f origin_vector, destination_vector;
    origin_vector = _destination - _origin;
    if (spline_next) {
        destination_vector = next_destination - _destination;
    } else {
        destination_vector = next_destination - destination;
    }

    _origin = _destination;

    // initialise intermediate point to the origin
    _flags.reached_destination = false;
    _flags.wp_yaw_set = false;

    // store destination location
    _destination = destination;
    _scurve_last_leg = _scurve_this_leg;
    if (_flags.fast_waypoint) {
        _scurve_this_leg = _scurve_next_leg;
    } else {
        _scurve_this_leg.calculate_spline_leg(_origin, _destination, origin_vector, destination_vector);
    }
    _flags.fast_waypoint = false;   // default waypoint back to slow
    _scurve_next_leg.Cal_Init(0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

    return true;
}

bool AC_WPNav::set_spline_destination_next(const Vector3f& destination, const Vector3f& next_destination, bool spline_next)
{
    hal.console->printf("set_spline_destination_next \n");

    Vector3f origin_vector, destination_vector;
    if (_scurve_this_leg.is_streight()) {
        origin_vector = _destination - _origin;
    } else {
        origin_vector = destination - _origin;
    }
    if (spline_next) {
        destination_vector = next_destination - _destination;
    } else {
        destination_vector = next_destination - destination;
    }

    _scurve_next_leg.calculate_spline_leg(_destination, destination, origin_vector, destination_vector);

    // next destination provided so fast waypoint
    _flags.fast_waypoint = true;

    return true;
}

/// shift_wp_origin_to_current_pos - shifts the origin and destination so the origin starts at the current position
///     used to reset the position just before takeoff
///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
void AC_WPNav::shift_wp_origin_to_current_pos()
{
    // return immediately if vehicle is not at the origin
    if (_track_desired > 0.0f) {
        return;
    }

    // get current and target locations
    const Vector3f &curr_pos = _inav.get_position();
    const Vector3f pos_target = _pos_control.get_pos_target();

    // calculate difference between current position and target
    Vector3f pos_diff = curr_pos - pos_target;

    // shift origin and destination
    _origin += pos_diff;
    _destination += pos_diff;

    // move pos controller target and disable feed forward
    _pos_control.set_pos_target(curr_pos);
}

/// get_wp_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
void AC_WPNav::get_wp_stopping_point_xy(Vector3f& stopping_point) const
{
	_pos_control.get_stopping_point_xy(stopping_point);
}

/// get_wp_stopping_point - returns vector to stopping point based on 3D position and velocity
void AC_WPNav::get_wp_stopping_point(Vector3f& stopping_point) const
{
    _pos_control.get_stopping_point_xy(stopping_point);
    _pos_control.get_stopping_point_z(stopping_point);
}

/// advance_wp_target_along_track - move target location along track from origin to destination
bool AC_WPNav::advance_wp_target_along_track(float dt)
{

    // get current location
    const Vector3f &curr_pos = _inav.get_position();
//    const Vector3f &curr_vel = _inav.get_velocity();
    const Vector3f &desired_vel = _pos_control.get_desired_velocity();
    float track_desired_vel = desired_vel.length();

//    float nav_tc = 1.0f;
//    if (is_positive(track_desired_vel)) {
//        float track_vel = (curr_vel.x * desired_vel.x + curr_vel.y * desired_vel.y + curr_vel.z * desired_vel.z) / track_desired_vel;
//        _track_scaler_dt += (track_vel/(_track_scaler_dt*track_desired_vel)-_track_scaler_dt) * (dt / (dt + nav_tc));
//        _track_scaler_dt = constrain_float(_track_scaler_dt, 0.1f, 1.0f);
//    } else {
//        _track_scaler_dt = 1.0f;
//    }
    _track_scaler_dt = 1.0f;

    Vector3f target_pos, target_vel, target_accel;
    target_pos = _origin;
    _scurve_last_leg.move_from_pos_vel_accel(dt, _track_scaler_dt, target_pos, target_vel, target_accel);
    bool s_finish = _scurve_this_leg.move_to_pos_vel_accel(dt, _track_scaler_dt, target_pos, target_vel, target_accel);

    _pos_control.set_pos_vel_accel(target_pos, target_vel, target_accel);

    if (_flags.fast_waypoint && _scurve_this_leg.breaking()) {
        float time_to_destination = _scurve_this_leg.time_to_end();
        Vector3f turn_pos, turn_vel, turn_accel;
        turn_pos = -_scurve_this_leg.get_pos_end();
        _scurve_this_leg.move_to_time_pos_vel_accel(_scurve_this_leg.time_now() + time_to_destination/2.0, 1.0, turn_pos, turn_vel, turn_accel);
        _scurve_next_leg.move_to_time_pos_vel_accel(time_to_destination/2.0, _track_scaler_dt, turn_pos, turn_vel, turn_accel);
//      s_finish = s_finish || ((_scurve_this_leg.time_to_end() < _scurve_next_leg.time_end()/2.0) && (turn_pos.length() < _wp_radius_cm) && (Vector2f(turn_vel.x, turn_vel.y).length() < _wp_speed_cms));
        s_finish = s_finish || ((_scurve_this_leg.time_to_end() < _scurve_next_leg.time_end()/2.0) && (turn_pos.length() < _wp_radius_cm) && (Vector2f(turn_vel.x, turn_vel.y).length() < _wp_speed_cms) && (Vector2f(turn_accel.x, turn_accel.y).length() < 2*_wp_accel_cmss));
        AP::logger().Write("LENF",
                "TimeUS,td,te,ne2,tpl,tvl,ws,tal,wa",
                "Qffffffff",
                AP_HAL::micros64(),
                (double)time_to_destination,
                (double)_scurve_this_leg.time_to_end(),
                (double)_scurve_next_leg.time_end()/2.0,
                (double)turn_pos.length(),
                (double)Vector2f(turn_vel.x, turn_vel.y).length(),
                (double)_wp_speed_cms,
                (double)Vector2f(turn_accel.x, turn_accel.y).length(),
                (double)2*_wp_accel_cmss);
    }

    // check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        if( s_finish ) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            }else{
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest = curr_pos - _destination;
                if( dist_to_dest.length() <= _wp_radius_cm ) {
                    _flags.reached_destination = true;
                }
            }
        }
    }

    float turn_rate, accel_forward;
    Vector3f accel_turn;
    if (is_positive(track_desired_vel) && is_positive(target_vel.length())) {
        accel_forward = (target_accel.x * target_vel.x + target_accel.y * target_vel.y + target_accel.z * target_vel.z)/target_vel.length();
        accel_turn = target_accel - target_vel * accel_forward / target_vel.length();
        turn_rate = accel_turn.length() / (_track_scaler_dt*target_vel.length());
        if(accel_turn.y * target_vel.x - accel_turn.x * target_vel.y < 0.0 ) {
            turn_rate *= -1.0f;
        }
    } else {
        turn_rate = 0.0f;
    }

    // update the target yaw if origin and destination are at least 2m apart horizontally
    float heading_vel = 0.0f;
    if (_scurve_this_leg.pos_end() >= WPNAV_YAW_DIST_MIN) {
        heading_vel = get_bearing_cd(Vector3f(), target_vel);
        // todo: add feed forward yaw for coordinated turns
        set_yaw_cd(heading_vel);
        set_yaw_cds(turn_rate*degrees(100.0f));
    }

//    AP::logger().Write("LEN2",
//            "TimeUS,tvx,tvy,tax,tay,ltx,lty,tr,ta,ts",
//            "Qfffffffff",
//            AP_HAL::micros64(),
//            (double)target_vel.x,
//            (double)target_vel.y,
//            (double)target_accel.x,
//            (double)target_accel.y,
//            (double)accel_turn.x,
//            (double)accel_turn.y,
//            (double)degrees(turn_rate),
//            (double)get_yaw()/100.0f,
//            (double)_track_scaler_dt);

    float tt, tp, tv, ta, tj, lt, lp, lv, la, lj;
    tt = _scurve_this_leg.time_now();
    _scurve_this_leg.runme(tt, tj, ta, tv, tp);
    lt = _scurve_last_leg.time_now();
    _scurve_last_leg.runme(lt, lj, la, lv, lp);
    AP::logger().Write("LENS",
            "TimeUS,tt,tp,tv,ta,lt,lp,lv,la",
            "Qffffffff",
            AP_HAL::micros64(),
            (double)tt,
            (double)tp,
            (double)tv,
            (double)ta,
            (double)lt,
            (double)lp,
            (double)lv,
            (double)la);

    // successfully advanced along track
    return true;
}

/// get_wp_distance_to_destination - get horizontal distance to destination in cm
float AC_WPNav::get_wp_distance_to_destination() const
{
    // get current location
    const Vector3f &curr = _inav.get_position();
    return norm(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_wp_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t AC_WPNav::get_wp_bearing_to_destination() const
{
    return get_bearing_cd(_inav.get_position(), _destination);
}

/// update_wpnav - run the wp controller - should be called at 100hz or higher
bool AC_WPNav::update_wpnav()
{
    bool ret = true;

    // get dt from pos controller
    float dt = _pos_control.get_dt();

    // allow the accel and speed values to be set without changing
    // out of auto mode. This makes it easier to tune auto flight
    _pos_control.set_max_accel_xy(_wp_accel_cmss);
    _pos_control.set_max_accel_z(_wp_accel_z_cmss);

    // advance the target if necessary
    if (!advance_wp_target_along_track(dt)) {
        // To-Do: handle inability to advance along track (probably because of missing terrain data)
        ret = false;
    }

    _pos_control.update_xy_controller();

    _wp_last_update = AP_HAL::millis();

    return ret;
}

// returns target yaw in centi-degrees (used for wp and spline navigation)
float AC_WPNav::get_yaw() const
{
    return _yaw;
    if (_flags.wp_yaw_set) {
        return _yaw;
    } else {
        // if yaw has not been set return attitude controller's current target
        return _attitude_control.get_att_target_euler_cd().z;
    }
}

// returns target yaw in centi-degrees (used for wp and spline navigation)
float AC_WPNav::get_yaw_rate() const
{
    if (_flags.wp_yaw_set) {
        return _yaw_rate;
    } else {
        // if yaw has not been set return attitude controller's current target
        return 0.0f;
    }
}

// set heading used for spline and waypoint navigation
void AC_WPNav::set_yaw_cd(float heading_cd)
{
    _yaw = heading_cd;
    _flags.wp_yaw_set = true;
}

// set heading used for spline and waypoint navigation
void AC_WPNav::set_yaw_cds(float heading_cds)
{
    _yaw_rate = heading_cds;
}

// get terrain's altitude (in cm above the ekf origin) at the current position (+ve means terrain below vehicle is above ekf origin's altitude)
bool AC_WPNav::get_terrain_offset(float& offset_cm)
{
    // calculate offset based on source (rangefinder or terrain database)
    switch (get_terrain_source()) {
    case AC_WPNav::TerrainSource::TERRAIN_UNAVAILABLE:
        return false;
    case AC_WPNav::TerrainSource::TERRAIN_FROM_RANGEFINDER:
        if (_rangefinder_healthy) {
            offset_cm = _inav.get_altitude() - _rangefinder_alt_cm;
            return true;
        }
        return false;
    case AC_WPNav::TerrainSource::TERRAIN_FROM_TERRAINDATABASE:
#if AP_TERRAIN_AVAILABLE
        float terr_alt = 0.0f;
        if (_terrain != nullptr && _terrain->height_above_terrain(terr_alt, true)) {
            offset_cm = _inav.get_altitude() - (terr_alt * 100.0f);
            return true;
        }
#endif
        return false;
    }

    // we should never get here but just in case
    return false;
}

// convert location to vector from ekf origin.  terrain_alt is set to true if resulting vector's z-axis should be treated as alt-above-terrain
//      returns false if conversion failed (likely because terrain data was not available)
bool AC_WPNav::get_vector_NEU(const Location &loc, Vector3f &vec, bool &terrain_alt)
{
    // convert location to NE vector2f
    Vector2f res_vec;
    if (!loc.get_vector_xy_from_origin_NE(res_vec)) {
        return false;
    }

    // convert altitude
    if (loc.get_alt_frame() == Location::AltFrame::ABOVE_TERRAIN) {
        int32_t terr_alt;
        if (!loc.get_alt_cm(Location::AltFrame::ABOVE_TERRAIN, terr_alt)) {
            return false;
        }
        vec.z = terr_alt;
        terrain_alt = true;
    } else {
        terrain_alt = false;
        int32_t temp_alt;
        if (!loc.get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, temp_alt)) {
            return false;
        }
        vec.z = temp_alt;
        terrain_alt = false;
    }

    // copy xy (we do this to ensure we do not adjust vector unless the overall conversion is successful
    vec.x = res_vec.x;
    vec.y = res_vec.y;

    return true;
}
