#include "Copter.h"

#if MODE_FOLLOW_ENABLED == ENABLED

/*
 * mode_follow.cpp - follow another mavlink-enabled vehicle by system id
 *
 * TODO: stick control to move around on sphere
 * TODO: stick control to change sphere diameter
 * TODO: "channel 7 option" to lock onto "pointed at" target
 * TODO: do better in terms of loitering around the moving point; may need a PID?  Maybe use loiter controller somehow?
 * TODO: extrapolate target vehicle position using its velocity and acceleration
 * TODO: ensure AC_AVOID_ENABLED is true because we rely on it velocity limiting functions
 */

// initialise follow mode
bool ModeFollow::init(const bool ignore_checks)
{
    if (!g2.follow.enabled()) {
        gcs().send_text(MAV_SEVERITY_WARNING, "Set FOLL_ENABLE = 1");
        return false;
    }

#if HAL_MOUNT_ENABLED
    AP_Mount *mount = AP_Mount::get_singleton();
    // follow the lead vehicle using sysid
    if (g2.follow.option_is_enabled(AP_Follow::Option::MOUNT_FOLLOW_ON_ENTER) && mount != nullptr) {
        mount->set_target_sysid(g2.follow.get_target_sysid());
    }
#endif

    // initialise horizontal speed, acceleration
    pos_control->set_max_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());
    pos_control->set_correction_speed_accel_xy(wp_nav->get_default_speed_xy(), wp_nav->get_wp_acceleration());

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    return true;
}

// perform cleanup required when leaving follow mode
void ModeFollow::exit()
{
    g2.follow.clear_offsets_if_required();
}

void ModeFollow::FollowTarget::reset(uint8_t sys_id, const Vector3f &target_pos_ned_cm, const Vector3f &target_vel_ned_cms, const Vector3f &target_accel_ned_cmss, float target_heading_deg, float target_heading_rate_degs)
{
    sysid = sys_id;
    pos_ned_cm = target_pos_ned_cm.topostype();
    vel_ned_cms = target_vel_ned_cms;
    accel_ned_cmss = target_accel_ned_cmss;
    heading_rad = radians(target_heading_deg);
    heading_rate_rads = radians(target_heading_rate_degs);
    heading_accel_radss = 0.0;
    available = true;
}

void ModeFollow::run()
{
    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_ground_handling();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    float yaw_cd = attitude_control->get_att_target_euler_cd().z;
    float yaw_rate_cds = 0.0f;
    // these should be parameters
    float target_max_accel_xy_cmss = pos_control->get_max_accel_xy_cmss();
    float target_max_jerk_xy_cmsss = pos_control->get_max_jerk_xy_cmsss();
    float target_max_accel_z_cmss = pos_control->get_max_accel_z_cmss();
    float target_max_jerk_z_cmsss = pos_control->get_max_jerk_z_cmsss();
    float target_max_accel_h_cdss = attitude_control->get_accel_yaw_max_cdss();
    float target_max_jerk_h_cdsss = attitude_control->get_accel_yaw_max_cdss() / attitude_control->get_input_tc();

    
    Vector3f pos_ned_m;  // vector to lead vehicle + offset_ned_cm
    if (!AP::ahrs().home_is_set()) {
        target.available = false;
    } else {
        // define target location
        Vector3f pos_with_ofs_ned_m;  // vector to lead vehicle + offset_ned_cm
        Vector3f vel_ned_ms;  // velocity of lead vehicle
        Vector3f accel_ned_mss;  // accel of lead vehicle
        if (g2.follow.get_target_pos_vel_accel_ned(pos_ned_m, pos_with_ofs_ned_m, vel_ned_ms, accel_ned_mss)) {
            Vector3f pos_with_ofs_ned_cm = pos_with_ofs_ned_m * 100.0;
            // vel_ned_ms does not include the change in heading_rad + offset_ned_cm radius
            Vector3f vel_ned_cms = vel_ned_ms * 100.0;
            Vector3f accel_ned_cmss = accel_ned_mss * 100.0;
    
            float target_heading_deg = 0.0f;
            float target_heading_rate_degs = 0.0f;
            g2.follow.get_target_heading_deg(target_heading_deg);
            g2.follow.get_target_heading_rate_degs(target_heading_rate_degs);
    
            if (!target.available || target.sysid != g2.follow.get_target_sysid()) {
                // reset target pos, vel, accel to current value when detected.
                target.reset(g2.follow.get_target_sysid(), pos_with_ofs_ned_cm, vel_ned_cms, accel_ned_cmss, target_heading_deg, target_heading_rate_degs);
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Follow Target %i detected", g2.follow.get_target_sysid() );
            }
    
            shape_pos_vel_accel_xy(pos_with_ofs_ned_cm.xy().topostype(), vel_ned_cms.xy(), accel_ned_cmss.xy(),
                target.pos_ned_cm.xy(), target.vel_ned_cms.xy(), target.accel_ned_cmss.xy(),
                0.0, target_max_accel_xy_cmss,
                target_max_jerk_xy_cmsss, G_Dt, false);
            shape_pos_vel_accel(pos_with_ofs_ned_cm.z, vel_ned_cms.z, accel_ned_cmss.z,
                target.pos_ned_cm.z, target.vel_ned_cms.z, target.accel_ned_cmss.z,
                0.0, 0.0, 
                -target_max_accel_z_cmss, target_max_accel_z_cmss,
                target_max_jerk_z_cmsss, G_Dt, false);
            update_pos_vel_accel_xy(target.pos_ned_cm.xy(), target.vel_ned_cms.xy(), target.accel_ned_cmss.xy(), G_Dt, Vector2f(), Vector2f(), Vector2f());
            update_pos_vel_accel(target.pos_ned_cm.z, target.vel_ned_cms.z, target.accel_ned_cmss.z, G_Dt, 0.0, 0.0, 0.0);
    
            shape_angle_vel_accel(radians(target_heading_deg), radians(target_heading_rate_degs), 0.0,
                target.heading_rad, target.heading_rate_rads, target.heading_accel_radss,
                0.0, radians(target_max_accel_h_cdss * 0.01),
                radians(target_max_jerk_h_cdsss * 0.01), G_Dt, false);
            postype_t target_heading_p = target.heading_rad;
            update_pos_vel_accel(target_heading_p, target.heading_rate_rads, target.heading_accel_radss, G_Dt, 0.0, 0.0, 0.0);
            target.heading_rad = wrap_PI(target_heading_p);
        } else {
            target.available = false;
        }
    }

    if (target.available) {
        Vector2p pos_ned_cm_xy = target.pos_ned_cm.xy();
        Vector2f vel_ned_cms_xy = target.vel_ned_cms.xy();
        Vector2f accel_ned_cmss_xy = target.accel_ned_cmss.xy();
        pos_control->input_pos_vel_accel_xy(pos_ned_cm_xy, vel_ned_cms_xy, accel_ned_cmss_xy, false);

        float pos_ned_cm_z = -target.pos_ned_cm.tofloat().z;
        float vel_ned_cms_z = -target.vel_ned_cms.z;
        float accel_ned_cmss_z = -target.accel_ned_cmss.z;
        pos_control->input_pos_vel_accel_z(pos_ned_cm_z, vel_ned_cms_z, accel_ned_cmss_z, false);

        // calculate vehicle heading
        switch (g2.follow.get_yaw_behave()) {
            case AP_Follow::YAW_BEHAVE_FACE_LEAD_VEHICLE: {
                if (pos_ned_m.xy().length_squared() > 1.0) {
                    yaw_cd = get_bearing_cd(pos_control->get_pos_target_cm().xy().tofloat(), pos_ned_m.xy());
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_SAME_AS_LEAD_VEHICLE: {
                yaw_cd = degrees(target.heading_rad) * 100.0;
                yaw_rate_cds = degrees(target.heading_rate_rads) * 100.0;
                break;
            }

            case AP_Follow::YAW_BEHAVE_DIR_OF_FLIGHT: {
                if (vel_ned_cms_xy.length_squared() > (100.0 * 100.0)) {
                    yaw_cd = get_bearing_cd(Vector2f{}, vel_ned_cms_xy);
                }
                break;
            }

            case AP_Follow::YAW_BEHAVE_NONE:
            default:
                // do nothing
            break;

        }
    } else {
        Vector2f vel_zero;
        Vector2f accel_zero;
        pos_control->input_vel_accel_xy(vel_zero, accel_zero, false);
        float velz = 0.0;
        pos_control->input_vel_accel_z(velz, 0.0, false);
        yaw_rate_cds = 0.0f;
    }

    // update the position controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_heading(pos_control->get_thrust_vector(), yaw_cd, yaw_rate_cds);
}

uint32_t ModeFollow::wp_distance() const
{
    return g2.follow.get_distance_to_target() * 100;
}

int32_t ModeFollow::wp_bearing() const
{
    return g2.follow.get_bearing_to_target() * 100;
}

/*
  get target position for mavlink reporting
 */
bool ModeFollow::get_wp(Location &loc) const
{
    float dist = g2.follow.get_distance_to_target();
    float bearing = g2.follow.get_bearing_to_target();
    loc = copter.current_loc;
    loc.offset_bearing(bearing, dist);
    return true;
}

#endif // MODE_FOLLOW_ENABLED == ENABLED
