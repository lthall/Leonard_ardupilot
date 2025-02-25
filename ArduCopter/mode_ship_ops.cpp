#include "Copter.h"

#if MODE_SHIP_OPS_ENABLED == ENABLED

#define MESSAGE_PERIOD 5000

// parameters for ship operation
const AP_Param::GroupInfo ModeShipOperation::var_info[] = {

    // @Param: HTL_ANG
    // @DisplayName: Angle from the bow of the ship of the Hotel position
    // @Description: Angle from the bow of the ship of the Hotel position where the aircraft will wait until it is commanded to land.
    // @Range: -180 360
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("HTL_ANG", 1, ModeShipOperation, hotel_angle, 180.0),

    // @Param: HTL_RAD
    // @DisplayName: Distance of the Hotel position from the ship
    // @Description: Distance in m of the Hotel position from the ship where the aircraft will wait until it is commanded to land.
    // @Range: 0 100
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("HTL_RAD", 2, ModeShipOperation, hotel_radius, 25.0),

    // @Param: HTL_ALT
    // @DisplayName: Altitude of the Hotel position from the ship
    // @Description: Altitude in m of the Hotel position relative to the ship where the aircraft will wait until it is commanded to land.
    // @Range: 0 100
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("HTL_ALT", 3, ModeShipOperation, hotel_altitude, 25.0),

    // @Param: ACCELXY
    // @DisplayName: Acceleration limit for the horizontal kinematic input shaping
    // @Description: Acceleration limit of the horizontal kinematic path generation used to determine how quickly the ship varies in velocity
    // @Range: 0 5
    // @Units: m/s/s
    // @User: Advanced
    AP_GROUPINFO("ACCELXY", 4, ModeShipOperation, ship_max_accel_xy, 2.5),

    // @Param: JERKXY
    // @DisplayName: Jerk limit for the horizontal kinematic input shaping
    // @Description: Jerk limit of the horizontal kinematic path generation used to determine how quickly the ship varies in acceleration
    // @Range: 0 20
    // @Units: m/s/s/s
    // @User: Advanced
    AP_GROUPINFO("JERKXY", 5, ModeShipOperation, ship_max_jerk_xy, 5.0),

    // @Param: ACCELZ
    // @DisplayName: Acceleration limit for the vertical kinematic input shaping
    // @Description: Acceleration limit of the vertical kinematic path generation used to determine how quickly the ship varies in velocity
    // @Range: 0 2.5
    // @Units: m/s/s
    // @User: Advanced
    AP_GROUPINFO("ACCELZ", 6, ModeShipOperation, ship_max_accel_z, 2.5),

    // @Param: JERKZ
    // @DisplayName: Jerk limit for the vertical kinematic input shaping
    // @Description: Jerk limit of the vertical kinematic path generation used to determine how quickly the ship varies in acceleration
    // @Range: 0 5
    // @Units: m/s/s/s
    // @User: Advanced
    AP_GROUPINFO("JERKZ", 7, ModeShipOperation, ship_max_jerk_z, 5.0),

    // @Param: ACCELH
    // @DisplayName: Angular acceleration limit for the heading kinematic input shaping
    // @Description: Angular acceleration limit of the heading kinematic path generation used to determine how quickly the ship varies in angular velocity
    // @Range: 0 45
    // @Units: deg/s/s
    // @User: Advanced
    AP_GROUPINFO("ACCELH", 8, ModeShipOperation, ship_max_accel_h, 90.0),

    // @Param: JERKH
    // @DisplayName: Angular jerk limit for the heading kinematic input shaping
    // @Description: Angular jerk limit of the heading kinematic path generation used to determine how quickly the ship varies in angular acceleration
    // @Range: 0 180
    // @Units: deg/s/s/s
    // @User: Advanced
    AP_GROUPINFO("JERKH", 9, ModeShipOperation, ship_max_jerk_h, 360.0),

    // @Param: KOZ_CW
    // @DisplayName: CW angular offset in degrees from ship heading for Keep Out Zone
    // @Description: CW angular offset in degrees from ship heading for Keep Out Zone
    // @Range: -180 360
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("KOZ_CW", 10, ModeShipOperation, keep_out_CW, 90.0),

    // @Param: KOZ_CCW
    // @DisplayName: CCW angular offset in degrees from ship heading for Keep Out Zone
    // @Description: CCW angular offset in degrees from ship heading for Keep Out Zone
    // @Range: -180 360
    // @Units: deg
    // @User: Advanced
    AP_GROUPINFO("KOZ_CCW", 11, ModeShipOperation, keep_out_CCW, -90.0),

    // @Param: KOZ_RAD
    // @DisplayName: Radius in meters of the Keep Out Zone
    // @Description: Radius in meters of the Keep Out Zone
    // @Range: 0 5000
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("KOZ_RAD", 12, ModeShipOperation, keep_out_radius, 250.0),

    // @Param: KOZ_DKR
    // @DisplayName: Radius in meters of the deck
    // @Description: Radius in meters of the deck before the Keep Out Zone becomes effective
    // @Range: 0 100
    // @Units: m
    // @User: Advanced
    AP_GROUPINFO("KOZ_DKR", 13, ModeShipOperation, deck_radius, 10.0),

    // @Param: HTL_VXY
    // @DisplayName: Velocity limit of the aircraft during movement between Hotel and High Hover
    // @Description: Velocity limit of the aircraft during movement between Hotel and High Hover
    // @Range: 0 5
    // @Units: m/s/s
    // @User: Advanced
    AP_GROUPINFO("HTL_VXY", 14, ModeShipOperation, hotel_max_vel_xy, 5.0),

    // @Param: HTL_AXY
    // @DisplayName: Acceleration limit of the aircraft during movement between Hotel and High Hover
    // @Description: Acceleration limit of the aircraft during movement between Hotel and High Hover
    // @Range: 0 5
    // @Units: m/s/s
    // @User: Advanced
    AP_GROUPINFO("HTL_AXY", 15, ModeShipOperation, hotel_max_accel_xy, 2.0),

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
    last_msg_ms = AP_HAL::millis();

    if (!g2.follow.enabled()) {
        // follow not enabled
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: Set FOLL_ENABLE = 1");
        ship.available = false;
        if (!ignore_checks) {
            return false;
        }
    }

    float keep_out_CCW_rad = wrap_2PI(radians(keep_out_CCW));
    float keep_out_angle_rad = wrap_2PI(radians(keep_out_CW) - keep_out_CCW_rad);
    float keep_out_CW_rad = keep_out_angle_rad + keep_out_CCW_rad;
    float keep_out_center_rad = (keep_out_CW_rad + keep_out_CCW_rad) / 2.0;
    if (is_positive(keep_out_radius)) {
        // KOZ is being used if keep_out_radius is positive
        bool deck_radius_valid = is_positive(deck_radius);
        bool approach_arc_valid = fabsf(wrap_PI(radians(hotel_angle) - keep_out_center_rad)) >=  keep_out_angle_rad / 2.0;
        if (!deck_radius_valid) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: KOZ_DKR must be positive");
            if (!ignore_checks) {
                return false;
            }
        }
        if (!approach_arc_valid) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: Hotel in KOZ");
            if (!ignore_checks) {
                return false;
            }
        }
    }

    // TODO: Must check Hotel angle is in approach cone.
    
    // if (!g2.follow.offsets_are_set()) {
    //     // follow does not have a target
    //     GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "FOLL_OFS X, Y, Z not set");
    //     return false;
    // }

    Vector3f pos_delta_ned_m;  // vector to lead vehicle + offset
    Vector3f pos_delta_with_ofs_ned_m;  // vector to lead vehicle + offset
    Vector3f vel_ned_ms;  // velocity of lead vehicle
    offset.zero();
    
    if (!AP::ahrs().home_is_set()) {
        ship.available = false;
        if (!ignore_checks) {
            return false;
        }
    } else if (!g2.follow.have_target()) {
        // follow does not have a target
        ship.available = false;
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: No Valid Beacon");
        if (!ignore_checks) {
            return false;
        }
    } else if (!g2.follow.get_target_dist_and_vel_ned(pos_delta_ned_m, pos_delta_with_ofs_ned_m, vel_ned_ms)) {
        // follow does not have a target
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: No valid beacon");
        ship.available = false;
        if (!ignore_checks) {
            return false;
        }
    } else {
        const Vector3f &curr_pos_neu_cm = inertial_nav.get_position_neu_cm();

        Vector3f pos_with_ofs_ned;  // vector to lead vehicle + offset
        pos_with_ofs_ned.xy() = curr_pos_neu_cm.xy() + pos_delta_with_ofs_ned_m.xy() * 100.0;
        pos_with_ofs_ned.z = -curr_pos_neu_cm.z + pos_delta_with_ofs_ned_m.z * 100.0;
        vel_ned_ms *= 100.0f;
        
        float target_heading_deg = 0.0f;
        g2.follow.get_target_heading_deg(target_heading_deg);

        ship.reset(g2.follow.get_target_sysid(), pos_with_ofs_ned, vel_ned_ms, target_heading_deg);

        offset.xy() = curr_pos_neu_cm.xy() - ship.pos_ned.xy().tofloat();
    }

    // do not permit horizontal movement until we have met an altitude
    // threshold:
    xy_position_control_permitted = false;

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
        set_state(SubMode::CLIMB_TO_RTL);
    } else {
        // we are landed so we will initialise in the Final state.
        set_state(SubMode::LAUNCH_RECOVERY);
    }

    return true;
}

// returns a user-readable string for supplied submode
const char *ModeShipOperation::state_name(SubMode mode) const
{
    switch (mode) {
    case SubMode::CLIMB_TO_RTL:
        return "ClimbToRTL";
    case SubMode::RETURN_TO_HOTEL:
        return "ReturnToHotel";
    case SubMode::HOTEL:
        return "Hotel";
    case SubMode::HIGH_HOVER:
        return "HighHover";
    case SubMode::LAUNCH_RECOVERY:
        return "LaunchRecovery";
    case SubMode::PAYLOAD_PLACE:
        return "PayloadPlace";
    }
    return "?";
}

void ModeShipOperation::set_state(SubMode mode)
{
    if (_state == mode) {
        // nothing to do
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: %s", state_name(mode));
    _state = mode;
    switch (_state) {
    case SubMode::CLIMB_TO_RTL:
        FALLTHROUGH;
    case SubMode::RETURN_TO_HOTEL:
        pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
        break;
    case SubMode::HOTEL:
        FALLTHROUGH;
    case SubMode::HIGH_HOVER:
        FALLTHROUGH;
    case SubMode::LAUNCH_RECOVERY:
        FALLTHROUGH;
    case SubMode::PAYLOAD_PLACE:
        pos_control->set_max_speed_accel_xy(hotel_max_vel_xy * 100.0, hotel_max_accel_xy * 100.0);
        break;
    }
}

void ModeShipOperation::set_keep_out_zone_mode(KeepOutZoneMode new_keep_out_zone_mode)
{
    if (keep_out_zone_mode == new_keep_out_zone_mode) {
        // nothing to do
        return;
    }
    keep_out_zone_mode = new_keep_out_zone_mode;
    switch (keep_out_zone_mode) {
    case KeepOutZoneMode::NO_ACTION:
        break;
    case KeepOutZoneMode::AVOID_KOZ:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: Avoiding KOZ");
        break;
    case KeepOutZoneMode::EXIT_KOZ:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: Exiting KOZ");
        break;
    }
}

void ModeShipOperation::set_approach_mode(ApproachMode new_approach_mode)
{
    if (approach_mode == new_approach_mode) {
        // nothing to do
        return;
    }
    approach_mode = new_approach_mode;
    switch (approach_mode) {
    case ApproachMode::LAUNCH_RECOVERY:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: Mode Launch/Recovery");
        break;
    case ApproachMode::PAYLOAD_PLACE:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "ShipOps: Mode Payload Place");
        break;
    }

}

void ModeShipOperation::Ship::reset(uint8_t sys_id, const Vector3f &pos_with_ofs_ned, const Vector3f &vel_ned_ms, float target_heading_deg)
{
    sysid = sys_id;
    pos_ned = pos_with_ofs_ned.topostype();
    vel_ned = vel_ned_ms;
    accel_ned.zero();
    heading = radians(target_heading_deg);
    heading_rate = 0.0;
    heading_accel = 0.0;
    available = true;
}

void ModeShipOperation::run()
{
    const uint32_t now_ms = AP_HAL::millis();

    float yaw_cd = attitude_control->get_att_target_euler_cd().z;
    float yaw_rate_cds = 0.0f;

    Vector2f hotel_offset = { hotel_radius * 100.0f, 0.0f };
    hotel_offset.rotate(radians(hotel_angle));
    const float hotel_height = hotel_altitude * 100.0;

    // get pilot desired climb rate if enabled
    bool valid_pilot_input = rc().has_valid_input();
    if (valid_pilot_input) {
        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    } else {
        // return to Hotel and continue to landing
        // set_approach_mode(ApproachMode::LAUNCH_RECOVERY);
        target_climb_rate = -get_pilot_speed_dn();
    }

    // check that Keep Out Zone is valid
    // if not then don't take-off or approach ship
    float keep_out_CCW_rad = wrap_2PI(radians(keep_out_CCW));
    float keep_out_angle_rad = wrap_2PI(radians(keep_out_CW) - keep_out_CCW_rad);
    float keep_out_CW_rad = keep_out_angle_rad + keep_out_CCW_rad;
    float keep_out_center_rad = (keep_out_CW_rad + keep_out_CCW_rad) / 2.0;
    bool keep_out_zone_valid = true; // true if the KOZ is valid
    bool deck_radius_valid = is_positive(deck_radius);
    bool approach_arc_valid = fabsf(wrap_PI(radians(hotel_angle) - keep_out_center_rad)) >=  keep_out_angle_rad / 2.0;
    if (is_positive(keep_out_radius)) {
        // KOZ is being used if keep_out_radius is positive
        keep_out_zone_valid = deck_radius_valid && approach_arc_valid;
    }
    
    bool deck_takeoff_check = true; // To ensure we are on the deck before taking off
    int32_t alt_home_above_origin_cm = 0; // To calculate RTL altitude
    const Vector3f &curr_pos_neu_cm = inertial_nav.get_position_neu_cm();
    if (!AP::ahrs().home_is_set()) {
        alt_home_above_origin_cm = 0;
        ship.available = false;
    } else {
        if (!AP::ahrs().get_home().get_alt_cm(Location::AltFrame::ABOVE_ORIGIN, alt_home_above_origin_cm)) {
            alt_home_above_origin_cm = 0;
        }
        // define target location
        Vector3f pos_delta_ned_m;  // vector to lead vehicle + offset
        Vector3f pos_delta_with_ofs_ned_m;  // vector to lead vehicle + offset
        Vector3f vel_ned_ms;  // velocity of lead vehicle
        Vector3f accel_ned_mss;  // accel of lead vehicle
        if (g2.follow.get_target_dist_and_vel_ned(pos_delta_ned_m, pos_delta_with_ofs_ned_m, vel_ned_ms)) {
            vel_ned_ms *= 100.0f;
            accel_ned_mss.zero(); // follow me should include acceleration so it is kept here for future functionality.
            // vel_ned_ms does not include the change in heading + offset radius
            Vector3f pos_with_ofs_ned;  // vector to lead vehicle + offset
            pos_with_ofs_ned.xy() = curr_pos_neu_cm.xy() + pos_delta_with_ofs_ned_m.xy() * 100.0;
            pos_with_ofs_ned.z = -curr_pos_neu_cm.z + pos_delta_with_ofs_ned_m.z * 100.0;
    
            float target_heading_deg = 0.0f;
            g2.follow.get_target_heading_deg(target_heading_deg);
    
            if (!ship.available || ship.sysid != g2.follow.get_target_sysid()) {
                // reset ship pos, vel, accel to current value when detected.
                ship.reset(g2.follow.get_target_sysid(), pos_with_ofs_ned, vel_ned_ms, target_heading_deg);
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Beacon %i detected", g2.follow.get_target_sysid() );
                if (copter.ap.land_complete) {
                    set_state(SubMode::LAUNCH_RECOVERY);
                } else {
                    set_state(SubMode::CLIMB_TO_RTL);
                }
            }
    
            shape_pos_vel_accel_xy(pos_with_ofs_ned.xy().topostype(), vel_ned_ms.xy(), accel_ned_mss.xy(),
                ship.pos_ned.xy(), ship.vel_ned.xy(), ship.accel_ned.xy(),
                0.0, ship_max_accel_xy * 100.0,
                ship_max_jerk_xy * 100.0, G_Dt, false);
            shape_pos_vel_accel(pos_with_ofs_ned.z, vel_ned_ms.z, accel_ned_mss.z,
                ship.pos_ned.z, ship.vel_ned.z, ship.accel_ned.z,
                0.0, 0.0, 
                -ship_max_accel_z * 100.0, ship_max_accel_z * 100.0,
                ship_max_jerk_z * 100.0, G_Dt, false);
            update_pos_vel_accel_xy(ship.pos_ned.xy(), ship.vel_ned.xy(), ship.accel_ned.xy(), G_Dt, Vector2f(), Vector2f(), Vector2f());
            update_pos_vel_accel(ship.pos_ned.z, ship.vel_ned.z, ship.accel_ned.z, G_Dt, 0.0, 0.0, 0.0);
    
            shape_angle_vel_accel(radians(target_heading_deg), 0.0, 0.0,
                ship.heading, ship.heading_rate, ship.heading_accel,
                0.0, radians(ship_max_accel_h),
                radians(ship_max_jerk_h), G_Dt, false);
            postype_t ship_heading_p = ship.heading;
            update_pos_vel_accel(ship_heading_p, ship.heading_rate, ship.heading_accel, G_Dt, 0.0, 0.0, 0.0);
            ship.heading = wrap_PI(ship_heading_p);
    
            // transform offset and Hotel to earth frame
            hotel_offset.rotate(ship.heading);
        } else {
            ship.available = false;
        }

        // Don't allow take-off if outside of deck radius
        if (ship.available && copter.ap.land_complete && deck_radius_valid && is_positive(target_climb_rate)) {
            Vector2f offset_xy = curr_pos_neu_cm.xy() - ship.pos_ned.xy().tofloat();
            if (offset_xy.length() > deck_radius * 100.0) {
                deck_takeoff_check = false;
                // prevent takeoff being triggered
                target_climb_rate = -get_pilot_speed_dn();
            }
        }
    }
    
    if (!keep_out_zone_valid || !ship.available) {
        if (copter.ap.land_complete) {
            // prevent takeoff being triggered
            target_climb_rate = -get_pilot_speed_dn();
            set_state(SubMode::LAUNCH_RECOVERY);
        } else {
            target_climb_rate = 0.0;
            set_state(SubMode::CLIMB_TO_RTL);
        }
    }

    // Handle error messages
    if (!keep_out_zone_valid || !ship.available || !deck_takeoff_check) {
        if (now_ms - last_msg_ms > MESSAGE_PERIOD) {
            last_msg_ms = now_ms;
            if (!ship.available) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Beacon %i not available", g2.follow.get_target_sysid() );
            }
            if (!deck_takeoff_check) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Take-off refused - off deck");
            }
            if (is_positive(keep_out_radius)) {
                // KOZ is being used if keep_out_radius is positive
                if (!deck_radius_valid) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Invalid KOZ: KOZ_DKR must be positive");
                }
                if (!approach_arc_valid) {
                    GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Invalid KOZ: Hotel in KOZ");
                }
            }
        }
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
        offset.xy() = curr_pos_neu_cm.xy() - ship.pos_ned.xy().tofloat();
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
        offset.zero();
        offset.xy() = curr_pos_neu_cm.xy() - ship.pos_ned.xy().tofloat();
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            // initialise takeoff variables
            // takeoff is between 110% of wp_navalt_min and 50% hotel_height
            takeoff.start(constrain_float(g.pilot_takeoff_alt, g2.wp_navalt_min * 110, hotel_height * 0.5));
            offset.zero();
            offset.xy() = curr_pos_neu_cm.xy() - ship.pos_ned.xy().tofloat();
            xy_position_control_permitted = false;
        }

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);

        // check if we are not navigating because of low altitude
        if (!xy_position_control_permitted) {
            // check if vehicle has reached no_nav_alt threshold.
            // Note we are e-using this WP parameter when we possibly
            // should not.
            if (inertial_nav.get_position_z_up_cm() >= takeoff.take_off_start_alt + g2.wp_navalt_min * 100) {
                xy_position_control_permitted = true;
            }
            offset.xy() = curr_pos_neu_cm.xy() - ship.pos_ned.xy().tofloat();
            pos_control->relax_velocity_controller_xy();
        } else {
            Vector2f accel;
            pos_control->input_vel_accel_xy(ship.vel_ned.xy(), accel);
        }
        break;

    case AltHold_Flying:

        // Calculate target location
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // climb to RTL altitude
            offset.z = MIN(-pos_control->get_pos_target_z_cm(),  -(float)(alt_home_above_origin_cm + g.rtl_altitude)) - ship.pos_ned.tofloat().z;
            break;
        case SubMode::RETURN_TO_HOTEL: {
            // move to Hotel location at RTL altitude while avoiding ship   
            offset.zero();
            offset.z = MIN(-pos_control->get_pos_target_z_cm(),  -(float)(alt_home_above_origin_cm + g.rtl_altitude)) - ship.pos_ned.tofloat().z;

            Vector2f aircraft_vector_cm = curr_pos_neu_cm.xy() - ship.pos_ned.xy().tofloat();
            float aircraft_bearing_rad = aircraft_vector_cm.angle();
            float koz_center_heading_rad = wrap_2PI(ship.heading + keep_out_center_rad);
            float extension_distance_cm = stopping_distance(wp_nav->get_default_speed_xy() + ship.vel_ned.xy().length(), 0.5 * pos_control->get_shaping_jerk_xy_cmsss() / wp_nav->get_wp_acceleration(), 0.5 * wp_nav->get_wp_acceleration());
            // We don't want the length to be greater than the gap in the approach zone.
            extension_distance_cm = MIN(extension_distance_cm, keep_out_radius * 100.0 * 0.5 * (2 * M_PI - keep_out_angle_rad));
            // We go to purch at 10% Hotel radius so we must make sure our extension is always more than that.
            extension_distance_cm = MAX(extension_distance_cm, hotel_radius * 12.5 );
            Vector2f extension_cm = { extension_distance_cm, 0.0 };

            if (!is_positive(keep_out_radius) || aircraft_vector_cm.length() < deck_radius * 100.0) {
                // I did this to ensure I can't get a divide by zero but I suspect I could have just use || and been fine.
                set_keep_out_zone_mode(KeepOutZoneMode::NO_ACTION);
            } else if (fabsf(wrap_PI(aircraft_bearing_rad - koz_center_heading_rad)) >  keep_out_angle_rad / 2.0 - safe_asin(deck_radius * 100.0 / aircraft_vector_cm.length())) {
                set_keep_out_zone_mode(KeepOutZoneMode::NO_ACTION);
            } else if (aircraft_vector_cm.length() < keep_out_radius * 100.0) {
                set_keep_out_zone_mode(KeepOutZoneMode::EXIT_KOZ);
            } else {
                set_keep_out_zone_mode(KeepOutZoneMode::AVOID_KOZ);
            }

            switch (keep_out_zone_mode) {
            case KeepOutZoneMode::NO_ACTION:
                // direct line of sight to the Hotel point
                offset.xy() = hotel_offset;
                break;
            case KeepOutZoneMode::AVOID_KOZ: {
                // avoiding Keep Out Zone
                float aircraft_tangent_angle = acosf(keep_out_radius * 100.0 / aircraft_vector_cm.length());
                float intercept_point_angle = 0.0;
                // this needs to account for forward facing purch should also use 
                if (is_positive(wrap_PI(aircraft_bearing_rad - koz_center_heading_rad))) {
                    // this can be simplified now
                    intercept_point_angle = constrain_float(wrap_PI(aircraft_bearing_rad - koz_center_heading_rad + aircraft_tangent_angle), -keep_out_angle_rad / 2.0, keep_out_angle_rad / 2.0) + koz_center_heading_rad; 
                    extension_cm.rotate(intercept_point_angle + M_PI / 2);
                } else {
                    // this can be simplified now
                    intercept_point_angle = constrain_float(wrap_PI(aircraft_bearing_rad - koz_center_heading_rad - aircraft_tangent_angle), -keep_out_angle_rad / 2.0, keep_out_angle_rad / 2.0) + koz_center_heading_rad;
                    extension_cm.rotate(intercept_point_angle - M_PI / 2);
                }
                offset.xy().zero();
                offset.xy().x = keep_out_radius * 100.0;
                offset.xy().rotate(intercept_point_angle);
                offset.xy() += extension_cm;
                break;
            }
            case KeepOutZoneMode::EXIT_KOZ: {
                // exiting Keep Out Zone
                Vector2f ship_heading_unit = { 1.0, 0.0 };
                ship_heading_unit.rotate(ship.heading);
                float exit_angle_avoid = acosf((aircraft_vector_cm * ship_heading_unit) / (keep_out_radius * 100.0));
                if (is_negative(wrap_PI(aircraft_bearing_rad - ship.heading))) {
                    exit_angle_avoid *= -1.0;
                }
                exit_angle_avoid += ship.heading;
                // keep_out_radius is in meters so 0.9 * 100 = 90.
                float turn_ratio_1 = constrain_float((aircraft_vector_cm.length() - keep_out_radius * 50.0) / (keep_out_radius * 50.0), 0.0, 1.0);
                float exit_angle = turn_ratio_1 * aircraft_bearing_rad + (1.0 - turn_ratio_1) * exit_angle_avoid;
                // this only sends the aircraft away from the boat.
                // we need this to move the aircraft to the approach vector.
                float turn_ratio_2 = constrain_float((aircraft_vector_cm.length() - keep_out_radius * 90.0) / (keep_out_radius * 10.0), 0.0, 1.0);
                float transition_angle;
                if (is_positive(wrap_PI(aircraft_bearing_rad - koz_center_heading_rad))) {
                    transition_angle = exit_angle + turn_ratio_2 * M_PI / 2;
                } else {
                    transition_angle = exit_angle - turn_ratio_2 * M_PI / 2;
                }
                extension_cm.rotate(transition_angle);
                offset.xy().zero();
                offset.xy().x = keep_out_radius * 100.0;
                offset.xy().rotate(exit_angle);
                offset.xy() += extension_cm;
                break;
            }
            }
            break;
        }
        case SubMode::HOTEL:
            // move to Hotel location and altitude
            offset.x = hotel_offset.x;
            offset.y = hotel_offset.y;
            offset.z = -hotel_height;
            break;
        case SubMode::HIGH_HOVER:
            FALLTHROUGH;
        case SubMode::PAYLOAD_PLACE:
            // move to High Hover
            offset.zero();
            offset.z = -hotel_height;
            break;
        case SubMode::LAUNCH_RECOVERY:
            // rotate offset with ship
            offset.xy().rotate(ship.heading_rate * G_Dt);
            // ascend to Hotel altitude when throttle high
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
            offset.z = -hotel_height;
            break;
        }

        // horizontal navigation
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:{
            // slow to zero velocity and climb to RTL altitude using velocity control
            Vector2f vel_ned;
            Vector2f accel_ned;
            pos_control->input_vel_accel_xy(vel_ned, accel_ned);
            break;
        }
        case SubMode::RETURN_TO_HOTEL:
            FALLTHROUGH;
        case SubMode::HOTEL:
            FALLTHROUGH;
        case SubMode::HIGH_HOVER:
            FALLTHROUGH;
        case SubMode::LAUNCH_RECOVERY:
            // relax horizontal position target if we might be landed
            // Do not put this in payload place as it does this already
            if (copter.ap.land_complete_maybe) {
                pos_control->soften_for_landing_xy();
            }
            FALLTHROUGH;
        case SubMode::PAYLOAD_PLACE:
            // move to target position and velocity
            Vector3p pos = ship.pos_ned.topostype();
            pos += offset.topostype();
            Vector2f zero;
            pos_control->input_pos_vel_accel_xy(pos.xy(), ship.vel_ned.xy(), zero, false);
            break;
        }

        // vertical navigation
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            FALLTHROUGH;
        case SubMode::RETURN_TO_HOTEL:
            FALLTHROUGH;
        case SubMode::HOTEL:
            FALLTHROUGH;
        case SubMode::HIGH_HOVER:
// include vertical offset
            pos_control->set_alt_target_with_slew(-(ship.pos_ned.z + offset.z));
            break;
        case SubMode::LAUNCH_RECOVERY:
            bool enforce_descent_limit;
            if( is_positive(target_climb_rate) ) {
                // move to Hotel altitude
                // reduce decent rate to land_speed when below land_alt_low
                float cmb_rate = constrain_float(sqrt_controller(-(ship.pos_ned.z + offset.z) - pos_control->get_pos_target_cm().z, pos_control->get_pos_z_p().kP(), pos_control->get_max_accel_z_cmss(), G_Dt), 0.0, g.pilot_speed_up);
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
                float alt_above_deck = MAX(0.0f, pos_control->get_pos_target_cm().z - ship.pos_ned.z);
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
        case SubMode::RETURN_TO_HOTEL:
            yaw_cd = pos_control->get_yaw_cd();
            yaw_rate_cds = pos_control->get_yaw_rate_cds();
            break;
        case SubMode::HOTEL:
            FALLTHROUGH;
        case SubMode::HIGH_HOVER:
            FALLTHROUGH;
        case SubMode::PAYLOAD_PLACE:
            switch (g2.follow.get_yaw_behave()) {
                case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                    if (ship.pos_ned.xy().length() > 1.0f) {
                        yaw_cd = get_bearing_cd(curr_pos_neu_cm.xy(), ship.pos_ned.xy().tofloat());
                    }
                    break;
                }

                case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                    yaw_cd = degrees(ship.heading) * 100.0;
                    yaw_rate_cds = degrees(ship.heading_rate) * 100.0;
                    break;
                }

                case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                    if (ship.vel_ned.length() > 100.0f) {
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
                yaw_rate_cds += degrees(ship.heading_rate) * 100.0;
            }
            break;
        }

        // update state of Ship Operations

        Vector3f pos_error = ship.pos_ned.tofloat() + offset - pos_control->get_pos_target_cm().tofloat();
        bool pos_check;
        // altitude is less than 5% of the Perch height
        bool alt_check = fabsf(-(ship.pos_ned.z + offset.z) - pos_control->get_pos_target_cm().z) < hotel_height * 0.05f;
        switch (_state) {
        case SubMode::CLIMB_TO_RTL:
            // check altitude is within 5% of hotel_height from RTL altitude
            if (ship.available && alt_check && keep_out_zone_valid) {
                set_state(SubMode::RETURN_TO_HOTEL);
            }
            break;
        case SubMode::RETURN_TO_HOTEL:
            // check that position is within 10% of the Hotel radius in x and y
            // if throttle is low then reduce altitude to Hotel altitude
            pos_check = pos_error.xy().length() < hotel_radius * 10.0f;
            if (pos_check) {
                set_state(SubMode::HOTEL);
            }
            break;
        case SubMode::HOTEL:
            // if altitude is correct and throttle is low then continue landing
            if (alt_check && is_negative(target_climb_rate)) {
                set_state(SubMode::HIGH_HOVER);
            }
            break;
        case SubMode::HIGH_HOVER:
            // check position is within 10 percent of the Hotel height
            // if accent requested then move back to Hotel location
            // if decent requested then continue recovery
            pos_check = pos_error.xy().length() < hotel_height * 0.1f;
            if (pos_check && is_negative(target_climb_rate)) {
                switch (approach_mode) {
                case ApproachMode::LAUNCH_RECOVERY:
                    set_state(SubMode::LAUNCH_RECOVERY);
#if AP_LANDINGGEAR_ENABLED
                    // optionally deploy landing gear
                    copter.landinggear.deploy_for_landing();
#endif
                    break;
                case ApproachMode::PAYLOAD_PLACE:
                    if (AP::gripper() != nullptr && AP::gripper()->valid() && AP::gripper()->grabbed()) {
                        payload_place.init(0.0);
                        set_state(SubMode::PAYLOAD_PLACE);
                    } else {
                        if (now_ms - last_msg_ms > MESSAGE_PERIOD) {
                            last_msg_ms = now_ms;
                            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Gripper already dropped");
                        }
                    }
                    break;
                }
            } else if (alt_check && is_positive(target_climb_rate)) {
                set_state(SubMode::HOTEL);
            }
            break;
        case SubMode::LAUNCH_RECOVERY:
            // if accent requested and altitude has reached or exceeded the Hotel altitude then move to Hotel
            if (alt_check && is_positive(target_climb_rate)) {
                set_state(SubMode::HOTEL);
#if AP_LANDINGGEAR_ENABLED
                // optionally retract landing gear
                copter.landinggear.retract_after_takeoff();
#endif
            }
            break;
        case SubMode::PAYLOAD_PLACE:
             if(payload_place.verify() || is_positive(target_climb_rate) || approach_mode != ApproachMode::PAYLOAD_PLACE) {
                set_state(SubMode::HIGH_HOVER);
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
    if (!AP::ahrs().home_is_set()) {
        return false;
    }
    float dist = g2.follow.get_distance_to_target();
    float bearing = g2.follow.get_bearing_to_target();
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_SHIP_OPS_ENABLED == ENABLED
