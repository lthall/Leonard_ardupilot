#include "Copter.h"

#if MODE_SHIP_OPS_ENABLED == ENABLED

// parameters for ship operation
const AP_Param::GroupInfo ModeShipOperation::var_info[] = {

    // @Param: PCH_ANG
    // @DisplayName: Angle from the bow of the ship of the perch position
    // @Description: Angle from the bow of the ship of the perch position where the aircraft will wait until it is commanded to land.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("PCH_ANG", 1, ModeShipOperation, ship_perch_angle, 180.0f),

    // @Param: PCH_RAD
    // @DisplayName: Distance of the perch position from the ship
    // @Description: Distance in m of the perch position from the ship where the aircraft will wait until it is commanded to land.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("PCH_RAD", 2, ModeShipOperation, ship_perch_radius, 25.0f),

    // @Param: PCH_ALT
    // @DisplayName: Altitude of the perch position from the ship
    // @Description: Altitude in m of the perch position relative to the ship where the aircraft will wait until it is commanded to land.
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("PCH_ALT", 3, ModeShipOperation, ship_perch_altitude, 25.0f),

    // @Param: ACCELXY
    // @DisplayName: Acceleration limit for the horizontal kinematic input shaping
    // @Description: Acceleration limit of the horizontal kinematic path generation used to determine how quickly the ship varies in velocity
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ACCELXY", 4, ModeShipOperation, ship_accel_xy, 2.5f),

    // @Param: JERKXY
    // @DisplayName: Jerk limit for the horizontal kinematic input shaping
    // @Description: Jerk limit of the horizontal kinematic path generation used to determine how quickly the ship varies in acceleration
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("JERKXY", 5, ModeShipOperation, ship_jerk_xy, 5.0f),

    // @Param: ACCELZ
    // @DisplayName: Acceleration limit for the vertical kinematic input shaping
    // @Description: Acceleration limit of the vertical kinematic path generation used to determine how quickly the ship varies in velocity
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ACCELZ", 6, ModeShipOperation, ship_accel_z, 2.5f),

    // @Param: JERKZ
    // @DisplayName: Jerk limit for the vertical kinematic input shaping
    // @Description: Jerk limit of the vertical kinematic path generation used to determine how quickly the ship varies in acceleration
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("JERKZ", 7, ModeShipOperation, ship_jerk_z, 5.0f),

    // @Param: ACCELH
    // @DisplayName: Angular acceleration limit for the heading kinematic input shaping
    // @Description: Angular acceleration limit of the heading kinematic path generation used to determine how quickly the ship varies in angular velocity
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("ACCELH", 8, ModeShipOperation, ship_accel_h, 90.0f),

    // @Param: JERKH
    // @DisplayName: Angular jerk limit for the heading kinematic input shaping
    // @Description: Angular jerk limit of the heading kinematic path generation used to determine how quickly the ship varies in angular acceleration
    // @Range: 0 1
    // @User: Advanced
    AP_GROUPINFO("JERKH", 9, ModeShipOperation, ship_jerk_h, 360.0f),

    AP_GROUPEND
};

ModeShipOperation::ModeShipOperation(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}


/*
 * mode_ship_ops.cpp - Ship take off and landing referenced to a mavlink-enabled system id
 */

// initialise ship operations mode
bool ModeShipOperation::init(const bool ignore_checks)
{
    if (!g2.follow.enabled()) {
        // follow not enabled
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

    if (!g2.follow.have_target()) {
        // follow does not have a target
        gcs().send_text(MAV_SEVERITY_WARNING, "No beacon detected");
        return false;
    }
    
    // if (!g2.follow.offsets_are_set()) {
    //     // follow does not have a target
    //     gcs().send_text(MAV_SEVERITY_WARNING, "FOLL_OFS X, Y, Z not set");
    //     return false;
    // }

    Vector3f pos_delta;  // vector to lead vehicle + offset
    Vector3f pos_delta_with_ofs;  // vector to lead vehicle + offset
    Vector3f pos_with_ofs;  // vector to lead vehicle + offset
    Vector3f vel_ned;  // velocity of lead vehicle
    const Vector3f &curr_pos = inertial_nav.get_position_neu_cm();
    
    if (!g2.follow.get_target_dist_and_vel_ned(pos_delta, pos_delta_with_ofs, vel_ned)) {
        // follow does not have a target
        gcs().send_text(MAV_SEVERITY_WARNING, "Beacon distance larger than FOLL_DIST_MAX");
        return false;
    }
    // change NED to NEU
    pos_delta_with_ofs.z = - pos_delta_with_ofs.z;
    pos_with_ofs = curr_pos + pos_delta_with_ofs * 100.0f;
    
    float target_heading_deg = 0.0f;
    g2.follow.get_target_heading_deg(target_heading_deg);

    ship_pos = pos_with_ofs.topostype();
    ship_vel = vel_ned;
    ship_accel.zero();
    ship_heading = radians(target_heading_deg);
    ship_heading_rate = 0.0;
    ship_heading_accel = 0.0;
    ship_available = true;

    offset.zero();
    offset.xy() = curr_pos.xy() - ship_pos.xy().tofloat();

    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    // initialise yaw
    // auto_yaw.set_mode_to_default(false);

    if (get_alt_hold_state(0.0f) == AltHold_Flying) {
        // we are flying so we will initialise at the climb to RTL altitude.
        _state = SubMode::CLIMB_TO_RTL;
        gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: ClimbToRTL");
        ship_takeoff = false;
    } else {
        // we are landed so we will initialise in the Final state.
        _state = SubMode::LAUNCH_RECOVERY;
        gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: LaunchRecovery");
        ship_takeoff = true;
    }

    return true;
}

void ModeShipOperation::run()
{
    float yaw_cd = attitude_control->get_att_target_euler_cd().z;
    float yaw_rate_cds = 0.0f;

    Vector2f perch_offset = Vector2f(ship_perch_radius * 100.0, 0.0f);
    perch_offset.rotate(radians(ship_perch_angle));
    float perch_height = ship_perch_altitude * 100.0;

    // get pilot desired climb rate if enabled
    bool valid_pilot_input = !copter.failsafe.radio && copter.ap.rc_receiver_present && g.land_repositioning;
    if (valid_pilot_input) {
        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // take off and landing will be conducted automaticaly  
        if (ship_takeoff) {
            // take off and move to perch
            target_climb_rate = g.pilot_speed_up;
        } else {
            // return to perch and continue to landing
            target_climb_rate = -get_pilot_speed_dn();
        }
    }

    Vector3f pos_delta;  // vector to lead vehicle + offset
    Vector3f pos_delta_with_ofs;  // vector to lead vehicle + offset
    Vector3f pos_with_ofs;  // vector to lead vehicle + offset
    Vector3f vel_ned;  // velocity of lead vehicle
    Vector3f accel_ned;  // accel of lead vehicle
    
    const Vector3f &curr_pos = inertial_nav.get_position_neu_cm();
    // define target location
    if (g2.follow.get_target_dist_and_vel_ned(pos_delta, pos_delta_with_ofs, vel_ned)) {
        // vel_ned does not include the change in heading + offset radius
        pos_with_ofs.xy() = curr_pos.xy() + pos_delta_with_ofs.xy() * 100.0;
        pos_with_ofs.z = -curr_pos.z + pos_delta_with_ofs.z * 100.0;
        vel_ned *= 100.0f;

        float target_heading_deg = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);

        if (!ship_available) {
            // reset ship pos, vel, accel to current value when detected.
            ship_pos = pos_with_ofs.topostype();
            ship_vel = vel_ned;
            ship_accel.zero();
            ship_heading = radians(target_heading_deg);
            ship_heading_rate = 0.0;
            ship_heading_accel = 0.0;
            ship_available = true;
        }

        shape_pos_vel_accel_xy(pos_with_ofs.xy().topostype(), vel_ned.xy(), accel_ned.xy(),
                            ship_pos.xy(), ship_vel.xy(), ship_accel.xy(),
                            0.0, ship_accel_xy * 100.0,
                            ship_jerk_xy * 100.0, G_Dt, false);
        shape_pos_vel_accel(pos_with_ofs.z, vel_ned.z, accel_ned.z,
                            ship_pos.z, ship_vel.z, ship_accel.z,
                            0.0, 0.0, 
                            -ship_accel_z * 100.0, ship_accel_z * 100.0,
                            ship_jerk_z * 100.0, G_Dt, false);
        update_pos_vel_accel_xy(ship_pos.xy(), ship_vel.xy(), ship_accel.xy(), G_Dt, Vector2f(), Vector2f(), Vector2f());
        update_pos_vel_accel(ship_pos.z, ship_vel.z, ship_accel.z, G_Dt, 0.0, 0.0, 0.0);

        shape_angle_vel_accel(radians(target_heading_deg), 0.0, 0.0,
                            ship_heading, ship_heading_rate, ship_heading_accel,
                            0.0, radians(ship_accel_h),
                            radians(ship_jerk_h), G_Dt, false);
        postype_t ship_heading_p = ship_heading;
        update_pos_vel_accel(ship_heading_p, ship_heading_rate, ship_heading_accel, G_Dt, 0.0, 0.0, 0.0);
        ship_heading = wrap_PI(ship_heading_p);

        // transform offset and perch to earth frame
        perch_offset.rotate(ship_heading);
    } else {
        _state = SubMode::CLIMB_TO_RTL;
        ship_available = false;
    }

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        // initialise all controllers
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pos_control->init_xy_controller();   // forces attitude target to decay to zero
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        offset.zero();
        offset.xy() = curr_pos.xy() - ship_pos.xy().tofloat();
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();

        // disarm when the landing detector says we've landed and no RC input
        if (!valid_pilot_input && is_negative(target_climb_rate) && copter.ap.land_complete && motors->get_spool_state() == AP_Motors::SpoolState::GROUND_IDLE) {
            copter.arming.disarm(AP_Arming::Method::LANDED);
        }
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_velocity_controller_xy();   // forces attitude target to decay to zero
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            // initialise no-navigation altitude
            if (is_positive(g2.wp_navalt_min)) {
                auto_takeoff_no_nav_active = true;
            } else {
                auto_takeoff_no_nav_active = false;
            }
            // initialise takeoff variables
            takeoff.start(constrain_float(g.pilot_takeoff_alt, 0.0f, 1000.0f));
            offset.zero();
            offset.xy() = curr_pos.xy() - ship_pos.xy().tofloat();
        }

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // check if we are not navigating because of low altitude
        if (auto_takeoff_no_nav_active) {
            // check if vehicle has reached no_nav_alt threshold
            if (inertial_nav.get_position_z_up_cm() >= takeoff.take_off_start_alt + g2.wp_navalt_min) {
                auto_takeoff_no_nav_active = false;
            }
            pos_control->relax_velocity_controller_xy();
        } else {
            Vector2f accel;
            pos_control->input_vel_accel_xy(ship_vel.xy(), accel);
        }
        break;

    case AltHold_Flying:

        // Calculate target location
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // climb to RTL altitude
            offset.z = MIN(-pos_control->get_pos_target_z_cm(), -(float)g.rtl_altitude) - ship_pos.tofloat().z;
            break;
        case SubMode::RETURN_TO_PERCH:
            // move to Perch location at RTL altitude
            offset.x = perch_offset.x;
            offset.y = perch_offset.y;
            offset.z = MIN(-pos_control->get_pos_target_z_cm(), -(float)g.rtl_altitude) - ship_pos.tofloat().z;
            break;
            // FALLTHROUGH
        case SubMode::PERCH:
            // move to Perch location and altitude
            offset.x = perch_offset.x;
            offset.y = perch_offset.y;
            offset.z = -perch_height;
            break;
        case SubMode::OVER_SPOT:
            // FALLTHROUGH
        case SubMode::PAYLOAD_PLACE:
            // move over landing Spot at Perch altitude
            offset.zero();
            offset.z = -perch_height;
            break;
        case SubMode::LAUNCH_RECOVERY:
            // rotate offset with ship
            offset.xy().rotate(ship_heading_rate * G_Dt);
            // ascend to Perch altitude when throttle high
            // descend to deck when throttle is low
            if( is_zero(target_climb_rate) && valid_pilot_input) {
                // convert pilot input to reposition velocity
                // use half maximum acceleration as the maximum velocity to ensure aircraft will
                // stop from full reposition speed in less than 1 second.
                Vector2f vel_correction = get_pilot_desired_velocity(wp_nav->get_wp_acceleration() * 0.5);
                // set reposition state
                copter.ap.land_repo_active = !vel_correction.is_zero();
                // this should use direct velocity control with shaped follow input to remove integration errors.
                offset.xy() += vel_correction * G_Dt;
            }
            offset.z = -perch_height;
            break;
        }

        // horizontal navigation
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // slow to zero velocity and climb to RTL altitude using velocity control
            vel_ned.zero();
            accel_ned.zero();
            pos_control->input_vel_accel_xy(vel_ned.xy(), accel_ned.xy());
            break;
        case SubMode::RETURN_TO_PERCH:
            // FALLTHROUGH
        case SubMode::PERCH:
            // FALLTHROUGH
        case SubMode::OVER_SPOT:
            // FALLTHROUGH
        case SubMode::LAUNCH_RECOVERY:
            // FALLTHROUGH
        case SubMode::PAYLOAD_PLACE:
            // move to target position and velocity
            Vector3p pos = ship_pos.topostype();
            pos += offset.topostype();
            Vector2f zero;
            // relax stop target if we might be landed
            if (copter.ap.land_complete_maybe) {
                pos_control->soften_for_landing_xy();
            }
            pos_control->input_pos_vel_accel_xy(pos.xy(), ship_vel.xy(), zero);
            break;
        }

        // vertical navigation
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // FALLTHROUGH
        case SubMode::RETURN_TO_PERCH:
            // FALLTHROUGH
        case SubMode::PERCH:
            // FALLTHROUGH
        case SubMode::OVER_SPOT:
// include vertical offset
            pos_control->set_alt_target_with_slew(-(ship_pos.z + offset.z));
            break;
        case SubMode::LAUNCH_RECOVERY:
            bool enforce_descent_limit;
            if( is_positive(target_climb_rate) ) {
                // move to perch altitude
                // reduce decent rate to land_speed when below land_alt_low
                float cmb_rate = constrain_float(sqrt_controller(-(ship_pos.z + offset.z) - pos_control->get_pos_target_cm().z, pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt), 0.0, g.pilot_speed_up);
                target_climb_rate = constrain_float(target_climb_rate, 0.0, cmb_rate);

                enforce_descent_limit = true;
            } else {
                // decend at pilots commanded rate constrained by landing speed limits
                float max_land_descent_velocity;
                if (g.land_speed_high > 0) {
                    max_land_descent_velocity = -g.land_speed_high;
                } else {
                    max_land_descent_velocity = pos_control->get_max_speed_down_cms();
                }
                float alt_above_deck = MAX(0.0f, pos_control->get_pos_target_cm().z - ship_pos.z);
                if (copter.rangefinder_alt_ok()) {
                    // check if range finder detects the deck is closer than expected
                    alt_above_deck = MIN(alt_above_deck, copter.rangefinder_state.alt_cm_filt.get());
                }
                // reduce decent rate to land_speed when below land_alt_low
                float cmb_rate = constrain_float(sqrt_controller(MAX(g2.land_alt_low, 100) - alt_above_deck, pos_control->get_pos_z_p().kP(), 0.5 * pos_control->get_max_accel_z_cmss(), G_Dt), max_land_descent_velocity, -abs(g.land_speed));
                target_climb_rate = constrain_float(target_climb_rate, cmb_rate, g.pilot_speed_up);
                
                // do not ignore limits until we have slowed down for landing
                enforce_descent_limit = (MAX(g2.land_alt_low,100) > alt_above_deck) && !copter.ap.land_complete_maybe;
            }
            pos_control->land_at_climb_rate_cm(target_climb_rate, enforce_descent_limit);
            break;
        case SubMode::PAYLOAD_PLACE:
            payload_place.run_vertical_control();
            break;
        }

        // calculate vehicle heading
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // Do Nothing
            break;
        case SubMode::RETURN_TO_PERCH:
            yaw_cd = pos_control->get_yaw_cd();
            yaw_rate_cds = pos_control->get_yaw_rate_cds();
            break;
        case SubMode::PERCH:
            // FALLTHROUGH
        case SubMode::OVER_SPOT:
            // FALLTHROUGH
        case SubMode::PAYLOAD_PLACE:
            switch (g2.follow.get_yaw_behave()) {
                case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                    if (ship_pos.xy().length() > 1.0f) {
                        yaw_cd = get_bearing_cd(curr_pos.xy(), ship_pos.xy().tofloat());
                    }
                    break;
                }

                case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                    yaw_cd = degrees(ship_heading) * 100.0;
                    yaw_rate_cds = degrees(ship_heading_rate) * 100.0;
                    break;
                }

                case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                    if (ship_vel.length() > 100.0f) {
                        yaw_cd = pos_control->get_yaw_cd();
                        yaw_rate_cds = pos_control->get_yaw_rate_cds();
                    }
                    break;
                }

                case AP_Follow::YAW_BEHAVE_NONE:
                default:
                    // do nothing
                break;
            }
            break;
        case SubMode::LAUNCH_RECOVERY:
            // pilot has yaw control during landing.
            if (valid_pilot_input) {
                yaw_rate_cds = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
                yaw_rate_cds += degrees(ship_heading_rate) * 100.0;
            }
            break;
        }

        // update state of Ship Operations

        Vector3f pos_error = ship_pos.tofloat() + offset - pos_control->get_pos_target_cm().tofloat();
        bool pos_check;
        // altitude is less than 5% of the Perch height
        bool alt_check = fabsf(-(ship_pos.z + offset.z) - pos_control->get_pos_target_cm().z) < perch_height * 0.05f;
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // check altitude is within 5% of perch_height from RTL altitude
            if (ship_available && alt_check) {
                _state = SubMode::RETURN_TO_PERCH;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: ReturnToPerch");
            }
            break;
        case SubMode::RETURN_TO_PERCH:
            // check that position is within 10% of the Perch radius in x and y
            // if throttle is low then reduce altitude to Perch altitude
            pos_check = pos_error.xy().length() < ship_perch_radius * 10.0f;
            if (pos_check) {
                _state = SubMode::PERCH;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: Perch");
            }
            break;
        case SubMode::PERCH:
            // if altitude is correct and throttle is low then continue landing
            if (alt_check && is_negative(target_climb_rate)) {
                _state = SubMode::OVER_SPOT;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: OverSpot");
            }
            break;
        case SubMode::OVER_SPOT:
            // check position is within 10 percent of the Perch height
            // if accent requested then move back to Perch location
            // if decent requested then continue recovery
            pos_check = pos_error.xy().length() < perch_height * 0.1f;
            if (pos_check && is_negative(target_climb_rate)) {
                switch (_approach_mode) {
                case ApproachMode::LAUNCH_RECOVERY:
                    gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: LaunchRecovery");
                    _state = SubMode::LAUNCH_RECOVERY;
#if AP_LANDINGGEAR_ENABLED
                    // optionally deploy landing gear
                    copter.landinggear.deploy_for_landing();
#endif
                    break;
                case ApproachMode::PAYLOAD_PLACE:
                    gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: Payload Place");
                    payload_place.init(0.0);
                    _state = SubMode::PAYLOAD_PLACE;
                    break;
                }
            } else if (alt_check && is_positive(target_climb_rate)) {
                _state = SubMode::PERCH;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: ReturnToPerch");
            }
            break;
        case SubMode::LAUNCH_RECOVERY:
            // if accent requested and altitude has reached or exceeded the perch altitude then move to Perch
            if (alt_check && is_positive(target_climb_rate)) {
                _state = SubMode::PERCH;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: ReturnToPerch");
#if AP_LANDINGGEAR_ENABLED
                // optionally retract landing gear
                copter.landinggear.retract_after_takeoff();
#endif
            }
            break;
        case SubMode::PAYLOAD_PLACE:
            if(payload_place.verify()) {
                _state = SubMode::OVER_SPOT;
                set_approach_mode(ApproachMode::LAUNCH_RECOVERY);
                gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: OverSpot");
            }
            if(is_positive(target_climb_rate) || _approach_mode != ApproachMode::PAYLOAD_PLACE) {
                _state = SubMode::OVER_SPOT;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: OverSpot");
            }
            break;
        }
        break;
    }

    // update the position controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    if (_state == SubMode::LAUNCH_RECOVERY) {
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), yaw_rate_cds);
    } else {
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), yaw_cd, yaw_rate_cds);
    }
}

bool ModeShipOperation::is_landing() const
{
    return is_negative(target_climb_rate);
}

uint32_t ModeShipOperation::wp_distance() const
{
    return g2.follow.get_distance_to_target() * 100;
}

int32_t ModeShipOperation::wp_bearing() const
{
    return g2.follow.get_bearing_to_target() * 100;
}

/*
  get target position for mavlink reporting
 */
bool ModeShipOperation::get_wp(Location &loc) const
{
    float dist = g2.follow.get_distance_to_target();
    float bearing = g2.follow.get_bearing_to_target();
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

void ModeShipOperation::set_approach_mode(ApproachMode approach_mode)
{
    _approach_mode = approach_mode;
    switch (_approach_mode) {
    case ApproachMode::LAUNCH_RECOVERY:
        gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: Mode Launch/Recovery");
        break;
    case ApproachMode::PAYLOAD_PLACE:
        gcs().send_text(MAV_SEVERITY_INFO, "ShipOps: Mode Payload Place");
        break;
    }

}

#endif // MODE_SHIP_OPS_ENABLED == ENABLED
