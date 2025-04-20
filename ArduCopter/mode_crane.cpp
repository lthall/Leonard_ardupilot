#include "Copter.h"

#if MODE_CRANE_ENABLED == ENABLED

// parameters for crane mode
const AP_Param::GroupInfo ModeCrane::var_info[] = {

    // @Param: VEL_HIGH
    // @DisplayName: Maximum velocity high setting
    // @Description: Maximum velocity high setting.
    // @Range: 1.0 15.0
    // @Units: m/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VEL_HIGH", 1, ModeCrane, vel_high_ms, 12.0),

    // @Param: ACCEL_HIGH
    // @DisplayName: Maximum acceleration high setting
    // @Description: Maximum acceleration high setting.
    // @Range: 1.0 5.0
    // @Units: m/s/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ACCEL_HIGH", 2, ModeCrane, accel_high_mss, 3.0),

    // @Param: VEL_MED
    // @DisplayName: Maximum velocity medium setting
    // @Description: Maximum velocity medium setting.
    // @Range: 1.0 15.0
    // @Units: m/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VEL_MED", 3, ModeCrane, vel_med_ms, 6.0),

    // @Param: ACCEL_MED
    // @DisplayName: Maximum acceleration medium setting
    // @Description: Maximum acceleration medium setting.
    // @Range: 1.0 5.0
    // @Units: m/s/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ACCEL_MED", 4, ModeCrane, accel_med_mss, 2.0),

    // @Param: VEL_LOW
    // @DisplayName: Maximum velocity low setting
    // @Description: Maximum velocity low setting.
    // @Range: 1.0 15.0
    // @Units: m/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("VEL_LOW", 5, ModeCrane, vel_low_ms, 2.0),

    // @Param: ACCEL_LOW
    // @DisplayName: Maximum acceleration low setting
    // @Description: Maximum acceleration low setting.
    // @Range: 1.0 5.0
    // @Units: m/s/s
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("ACCEL_LOW", 6, ModeCrane, accel_low_mss, 1.0),

    AP_GROUPEND
};

ModeCrane::ModeCrane(void) : Mode()
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
 * Init and run calls for crane flight mode
 */

// crane_init - initialise crane controller
bool ModeCrane::init(bool ignore_checks)
{
    // initialise horizontal speed, acceleration
    float vel_max_cms = wp_nav->get_default_speed_xy();
    float accel_max_cmss = wp_nav->get_wp_acceleration();
    get_speed_mode_vel_accel(vel_max_cms, accel_max_cmss);
    pos_control->set_max_speed_accel_xy(vel_max_cms, accel_max_cmss);
    pos_control->set_correction_speed_accel_xy(vel_low_ms * 100.0, accel_low_mss * 100.0);

    // initialize vertical speeds and acceleration
    pos_control->set_max_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());
    pos_control->set_correction_speed_accel_z(wp_nav->get_default_speed_down(), wp_nav->get_default_speed_up(), wp_nav->get_accel_z());

    // initialise velocity controller
    pos_control->init_z_controller();
    pos_control->init_xy_controller();

    return true;
}

void ModeCrane::set_speed_mode(SpeedMode new_speed_mode)
{
    if (speed_mode == new_speed_mode) {
        // nothing to do
        return;
    }
    speed_mode = new_speed_mode;
    speed_mode_message();
}

void ModeCrane::speed_mode_message()
{
    switch (speed_mode) {
    case SpeedMode::HIGH:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Crane: Speed Mode High");
        break;
    case SpeedMode::MEDIUM:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Crane: Speed Mode Medium");
        break;
    case SpeedMode::LOW:
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Crane: Speed Mode Low");
        break;
    }
}

void ModeCrane::get_speed_mode_vel_accel(float &vel_cms, float &accel_cmss)
{
    float accel_new_cmss = accel_cmss;
    switch (speed_mode) {
    case SpeedMode::HIGH:
        vel_cms = vel_high_ms * 100.0;
        accel_new_cmss = accel_high_mss * 100.0;
        break;
    case SpeedMode::MEDIUM:
        vel_cms = vel_med_ms * 100.0;
        accel_new_cmss = accel_med_mss * 100.0;
        break;
    case SpeedMode::LOW:
        vel_cms = vel_low_ms * 100.0;
        accel_new_cmss = accel_low_mss * 100.0;
        break;
    }
    if (pos_control->get_vel_desired_cms().xy().length() <= vel_cms) {
        accel_cmss = accel_new_cmss;
    }
}

// crane_run - runs the loiter controller
// should be called at 100hz or more
void ModeCrane::run()
{
    Vector2f target_velocity_cms;
    float target_climb_rate_cms = 0.0f;
    float target_yaw_rate_degs = 0.0f;

    // get speed and acceleration limits
    float vel_max_cms = wp_nav->get_default_speed_xy();
    float accel_max_cmss = wp_nav->get_wp_acceleration();
    get_speed_mode_vel_accel(vel_max_cms, accel_max_cmss);
    pos_control->set_max_speed_accel_xy(vel_max_cms, accel_max_cmss);

    Vector3p pos_target_old_neu_cm = pos_control->get_pos_target_cm();

    // process pilot inputs unless we are in radio failsafe
    if (!copter.failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to reposition velocity
        target_velocity_cms = get_pilot_desired_velocity(vel_max_cms);

        // get pilot desired climb rate
        target_climb_rate_cms = get_pilot_desired_climb_rate(channel_throttle->get_control_in());

        // get pilot's desired yaw rate
        target_yaw_rate_degs = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());
    }

    // relax loiter target if we might be landed
    if (copter.ap.land_complete_maybe) {
        pos_control->soften_for_landing_xy();
    }

    // Crane State Machine Determination
    AltHoldModeState crane_state = get_alt_hold_state(target_climb_rate_cms);

    // Crane State Machine
    switch (crane_state) {

    case AltHold_MotorStopped:
        // initialise all controllers
        attitude_control->reset_yaw_target_and_rate();
        attitude_control->reset_rate_controller_I_terms();
        pos_control->init_xy_controller();   // forces attitude target to decay to zero
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_velocity_controller_xy();   // forces attitude target to decay to zero
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate_cms);

        // check if we are not navigating because of low altitude
        if (!xy_position_control_permitted) {
            // check if vehicle has reached no_nav_alt threshold.
            // Note we are e-using this WP parameter when we possibly
            // should not.
            if (inertial_nav.get_position_z_up_cm() >= takeoff.take_off_start_alt + g2.wp_navalt_min * 100) {
                xy_position_control_permitted = true;
            }
            pos_control->relax_velocity_controller_xy();
        } else {
            Vector2f accel;
            pos_control->input_vel_accel_xy(target_velocity_cms, accel);
        }
        break;

    case AltHold_Flying:
        // set motors to full range
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

        // Send the commanded velocity to the position controller
        Vector2f accel;
        pos_control->input_vel_accel_xy(target_velocity_cms, accel);

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate_cms);
        break;
    }

    Vector2f vel_desired_unit = pos_control->get_vel_desired_cms().xy().normalized();
    if (!vel_desired_unit.is_zero()) {
        // prevent position controller target from moving in front of our current position
        Vector3p pos_target_new_neu_cm = pos_control->get_pos_target_cm();
        Vector2f delta_pos_target = (pos_target_new_neu_cm.xy() - pos_target_old_neu_cm.xy()).tofloat();
        Vector2f delta_pos_current = pos_target_new_neu_cm.xy().tofloat() - pos_control->get_pos_neu_cm().xy();
        float delta_pos_target_dir = vel_desired_unit * delta_pos_target;
        float delta_pos_current_dir = vel_desired_unit * delta_pos_current;
        delta_pos_target_dir = constrain_float(delta_pos_target_dir-delta_pos_current_dir, 0.0, delta_pos_target_dir);
        pos_control->set_pos_target_xy_cm(pos_target_old_neu_cm.x + delta_pos_target_dir * vel_desired_unit.x,
                                          pos_target_old_neu_cm.y + delta_pos_target_dir * vel_desired_unit.y);
    }

    // update the position controller
    pos_control->update_xy_controller();
    pos_control->update_z_controller();

    // call attitude controller
    attitude_control->input_thrust_vector_rate_heading(pos_control->get_thrust_vector(), target_yaw_rate_degs);
}

uint32_t ModeCrane::wp_distance() const
{
    return pos_control->get_pos_error_xy_cm();;
}

int32_t ModeCrane::wp_bearing() const
{
    return pos_control->get_bearing_to_target_cd();
}

#endif
