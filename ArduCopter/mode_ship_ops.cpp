#include "Copter.h"

#if MODE_SHIP_OPS_ENABLED == ENABLED

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

    Vector3f posit;  // vector to lead vehicle + offset
    Vector3f pos_with_ofs;  // vector to lead vehicle + offset
    Vector3f vel_ned;  // velocity of lead vehicle
    
    if (!g2.follow.get_target_dist_and_vel_ned(posit, pos_with_ofs, vel_ned)) {
        // follow does not have a target
        gcs().send_text(MAV_SEVERITY_WARNING, "Beacon distance larger than FOLL_DIST_MAX");
        return false;
    }
    const Vector3f &curr_pos = inertial_nav.get_position_neu_cm();
    pos_with_ofs = curr_pos + pos_with_ofs * 100.0f;
// use landing offsets provided in ship parameters

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
        gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_ClimbToRTL");
        ship_takeoff = false;
    } else {
        // we are landed so we will initialise in the Final state.
        _state = SubMode::LAUNCH_RECOVERY;
        gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_LaunchRecovery");
        ship_takeoff = true;
    }

    offset.zero();
    offset.xy() = curr_pos.xy() - pos_with_ofs.xy();

    return true;
}

void ModeShipOperation::run()
{
    float yaw_cd = attitude_control->get_att_target_euler_cd().z;
    float yaw_rate = 0.0f;

    Vector2f perch_offset = Vector2f(g2.ship_perch_radius * 100.0, 0.0f);
// don't rotate perch offset
    perch_offset.rotate(radians(g2.ship_perch_angle));
    float perch_height = g2.ship_perch_altitude * 100.0;

    // get pilot desired climb rate if enabled
    bool valid_pilot_input = !copter.failsafe.radio && copter.ap.rc_receiver_present && g.land_repositioning;
    if (valid_pilot_input) {
        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // take off and landing will be conducted automaticaly  
        if (ship_takeoff) {
            // take off and move to purch
            target_climb_rate = g.pilot_speed_up;
        } else {
            // return to purch and continue to landing
            target_climb_rate = -get_pilot_speed_dn();
        }
    }

    Vector3f posit;  // vector to lead vehicle + offset
    Vector3f pos_with_ofs;  // vector to lead vehicle + offset
    Vector3f vel_ned;  // velocity of lead vehicle
    Vector3f accel_ned;  // accel of lead vehicle
// input shape position target
    bool ship_availible = g2.follow.get_target_dist_and_vel_ned(posit, pos_with_ofs, vel_ned);
    const Vector3f &curr_pos = inertial_nav.get_position_neu_cm();
    if (ship_availible) {
        // g2.follow.get_target_pos_and_vel_ned(pos_with_ofs, vel_ned);
        // change NED to NEU
        pos_with_ofs.z = - pos_with_ofs.z;
        pos_with_ofs = curr_pos + pos_with_ofs * 100.0f;
        vel_ned *= 100.0f;
    } else {
        _state = SubMode::CLIMB_TO_RTL;
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
        offset.xy() = curr_pos.xy() - pos_with_ofs.xy();
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
            offset.xy() = curr_pos.xy() - pos_with_ofs.xy();
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
            pos_control->input_vel_accel_xy(vel_ned.xy(), accel);
        }
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_velocity_controller_xy();   // forces attitude target to decay to zero
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Flying:
        // define target location
        float target_heading_deg = 0.0f;
        if (g2.follow.get_target_heading_deg(target_heading_deg)) {
// provide yaw rate using input shaping
            perch_offset.rotate(radians(target_heading_deg));
        }

        // Calculate target location
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // climb to RTL altitude
// use maximum of current altitude and RTL altitude
            offset.z = -(float)g.rtl_altitude;
            break;
        case SubMode::RETURN_TO_PERCH:
            // move to Perch location at RTL altitude
            offset.x = perch_offset.x;
            offset.y = perch_offset.y;
            offset.z = -(float)g.rtl_altitude;
            break;
            // FALLTHROUGH
        case SubMode::PERCH:
            // move to Perch location and altitude
            offset.x = perch_offset.x;
            offset.y = perch_offset.y;
            offset.z = -perch_height;
            break;
        case SubMode::OVER_SPOT:
            // move over landing Spot at Perch altitude
            offset.zero();
            offset.z = -perch_height;
            break;
        case SubMode::LAUNCH_RECOVERY:
            // ascend to Perch altitude when throttle high
            // descend to deck when throttle is low
            if( is_zero(target_climb_rate) && valid_pilot_input) {
                // convert pilot input to reposition velocity
                // use half maximum acceleration as the maximum velocity to ensure aircraft will
                // stop from full reposition speed in less than 1 second.
                Vector2f vel_correction = get_pilot_desired_velocity(wp_nav->get_wp_acceleration() * 0.5);
                // set reposition state
                copter.ap.land_repo_active = !vel_correction.is_zero();
// rotate vel_correction
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
            // move to target position and velocity
// use .xy
            Vector3p pos = pos_with_ofs.topostype();
            pos += offset.topostype();
            Vector2f zero;
            // relax stop target if we might be landed
            if (copter.ap.land_complete_maybe) {
                pos_control->soften_for_landing_xy();
            }
            pos_control->input_pos_vel_accel_xy(pos.xy(), vel_ned.xy(), zero);
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
// include vertical offseet
            pos_control->set_alt_target_with_slew(-offset.z);
            break;
        case SubMode::LAUNCH_RECOVERY:
            if( is_positive(target_climb_rate) ) {
                // move to directly to perch altitude
// use vertical rate
// need to add rate limit to stop at -offset.z
                pos_control->set_alt_target_with_slew(-offset.z);
            } else {
                // decend at pilots commanded rate constrained by landing speed limits
                float max_land_descent_velocity;
                if (g.land_speed_high > 0) {
// and less than land height
// probably need to use square root controller and combine the two
                    max_land_descent_velocity = -g.land_speed_high;
                } else {
                    max_land_descent_velocity = pos_control->get_max_speed_down_cms();
                }
                float alt_above_deck = MAX(0.0f, pos_control->get_pos_target_cm().z - pos_with_ofs.z);
                if (copter.rangefinder_alt_ok()) {
                    // check if range finder detects the deck is closer than expected
                    alt_above_deck = MIN(alt_above_deck, copter.rangefinder_state.alt_cm_filt.get());
                }
                // reduce decent rate to land_speed when below land_alt_low
                float cmb_rate = constrain_float(sqrt_controller(MAX(g2.land_alt_low, 100) - alt_above_deck, pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt), max_land_descent_velocity, -abs(g.land_speed));
                target_climb_rate = constrain_float(target_climb_rate, cmb_rate, g.pilot_speed_up);
                pos_control->land_at_climb_rate_cm(target_climb_rate, true);
            }
            break;
        }

        // calculate vehicle heading
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // Do Nothing
            break;
        case SubMode::RETURN_TO_PERCH:
            yaw_cd = pos_control->get_yaw_cd();
            yaw_rate = pos_control->get_yaw_rate_cds();
            break;
        case SubMode::PERCH:
            // FALLTHROUGH
        case SubMode::OVER_SPOT:
            switch (g2.follow.get_yaw_behave()) {
                case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                    if (pos_with_ofs.xy().length() > 1.0f) {
                        yaw_cd = get_bearing_cd(Vector2f(), pos_with_ofs.xy());
                    }
                    break;
                }

                case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                    float target_hdg = 0.0f;
                    if (g2.follow.get_target_heading_deg(target_hdg)) {
                        yaw_cd = target_hdg * 100.0f;
                    }
                    break;
                }

                case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                    if (vel_ned.length() > 100.0f) {
                        yaw_cd = pos_control->get_yaw_cd();
                        yaw_rate = pos_control->get_yaw_rate_cds();
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
                yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
            }
            break;
        }

        // update state of Ship Operations

        Vector3f pos_error = pos_with_ofs + offset - pos_control->get_pos_target_cm().tofloat();
        bool pos_check;
        // altitude is less than 5% of the Perch height
        bool alt_check = fabsf(-offset.z - pos_control->get_pos_target_cm().z) < perch_height * 0.05f;
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // check altitude is within 5% of perch_height from RTL altitude
            if (ship_availible && alt_check) {
                _state = SubMode::RETURN_TO_PERCH;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_ReturnToPerch");
            }
            break;
        case SubMode::RETURN_TO_PERCH:
            // check that position is within 10% of the Perch radius in x and y
            // if throttle is low then reduce altitude to Perch altitude
            pos_check = Vector2f(pos_error.x, pos_error.y).length() < g2.ship_perch_radius * 10.0f;
            if (pos_check) {
                _state = SubMode::PERCH;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_Perch");
            }
            break;
        case SubMode::PERCH:
            // if altitude is correct and throttle is low then continue landing
            if (alt_check && is_negative(target_climb_rate)) {
                _state = SubMode::OVER_SPOT;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_OverSpot");
            }
            break;
        case SubMode::OVER_SPOT:
            // check position is within 10 percent of the Perch height
            // if accent requested then move back to Perch location
            // if decent requested then continue recovery
            pos_check = Vector2f(pos_error.x, pos_error.y).length() < perch_height * 0.1f;
            if (pos_check && is_negative(target_climb_rate)) {
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_LaunchRecovery");
                _state = SubMode::LAUNCH_RECOVERY;
            } else if (is_positive(target_climb_rate)) {
                _state = SubMode::PERCH;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_ReturnToPerch");
                // optionally deploy landing gear
                copter.landinggear.retract_after_takeoff();
            }
            break;
        case SubMode::LAUNCH_RECOVERY:
            // if accent requested and altitude has reached or exceeded the perch altitude then move to Perch
            if (alt_check && is_positive(target_climb_rate)) {
                _state = SubMode::PERCH;
                gcs().send_text(MAV_SEVERITY_INFO, "ShipLand: ShipOps_ReturnToPerch");
                // optionally deploy landing gear
                copter.landinggear.deploy_for_landing();
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
        attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), yaw_rate);
    } else {
        attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), yaw_cd, yaw_rate);
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

#endif // MODE_SHIP_OPS_ENABLED == ENABLED