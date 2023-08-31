#include "Copter.h"

#include <cstdlib>

// Code to detect a crash main ArduCopter code
#define CRASH_CHECK_TRIGGER_SEC         2       // 2 seconds inverted indicates a crash
#define CRASH_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees beyond target angle is signal we are out of control
#define CRASH_CHECK_ANGLE_MIN_DEG       15.0f   // vehicle must be leaning at least 15deg to trigger crash check
#define CRASH_CHECK_SPEED_MAX           10.0f   // vehicle must be moving at less than 10m/s to trigger crash check
#define CRASH_CHECK_ACCEL_MAX           3.0f    // vehicle must be accelerating less than 3m/s/s to be considered crashed

// Code to detect a thrust loss main ArduCopter code
#define THRUST_LOSS_CHECK_TRIGGER_SEC         1     // 1 second descent while level and high throttle indicates thrust loss
#define THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD  1500  // we can't expect to maintain altitude beyond 15 degrees on all aircraft
#define THRUST_LOSS_CHECK_MINIMUM_THROTTLE    0.9f  // we can expect to maintain altitude above 90 % throttle

#define TIME_BETWEEN_ARM_ATTEMPTS_MS          5000

// Yaw imbalance check
#define YAW_IMBALANCE_IMAX_THRESHOLD 0.75f
#define YAW_IMBALANCE_WARN_MS 10000

// crash_check - disarms motors if a crash has been detected
// crashes are detected by the vehicle being more than 20 degrees beyond it's angle limits continuously for more than 1 second
// called at MAIN_LOOP_RATE
void Copter::crash_check()
{
    static uint16_t crash_counter;  // number of iterations vehicle may have been crashed

    // return immediately if disarmed, or crash checking disabled
    if (!motors->armed() || ap.land_complete || g.fs_crash_check == 0) {
        crash_counter = 0;
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        crash_counter = 0;
        return;
    }

    // exit immediately if in force flying
    if (force_flying && !flightmode->is_landing()) {
        crash_counter = 0;
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (flightmode->mode_number() == Mode::Number::ACRO || flightmode->mode_number() == Mode::Number::FLIP) {
        crash_counter = 0;
        return;
    }

#if MODE_AUTOROTATE_ENABLED == ENABLED
    //return immediately if in autorotation mode
    if (flightmode->mode_number() == Mode::Number::AUTOROTATE) {
        crash_counter = 0;
        return;
    }
#endif

    // vehicle not crashed if 1hz filtered acceleration is more than 3m/s (1G on Z-axis has been subtracted)
    const float filtered_acc = land_accel_ef_filter.get().length();
    if (filtered_acc >= CRASH_CHECK_ACCEL_MAX) {
        crash_counter = 0;
        return;
    }

    // check for lean angle over 15 degrees
    const float lean_angle_deg = degrees(acosf(ahrs.cos_roll()*ahrs.cos_pitch()));
    if (lean_angle_deg <= CRASH_CHECK_ANGLE_MIN_DEG) {
        crash_counter = 0;
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        crash_counter = 0;
        return;
    }

    // check for speed under 10m/s (if available)
    Vector3f vel_ned;
    if (ahrs.get_velocity_NED(vel_ned) && (vel_ned.length() >= CRASH_CHECK_SPEED_MAX)) {
        crash_counter = 0;
        return;
    }

    // we may be crashing
    crash_counter++;

    // check if crashing for 2 seconds
    if (crash_counter >= (CRASH_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        AP::logger().Write_Error(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_CRASH);
        // send message to gcs
        gcs().send_text(MAV_SEVERITY_EMERGENCY,"Crash: Disarming: AngErr=%.0f>%.0f, Accel=%.1f<%.1f", angle_error, CRASH_CHECK_ANGLE_DEVIATION_DEG, filtered_acc, CRASH_CHECK_ACCEL_MAX);
        // disarm motors
        copter.arming.disarm(AP_Arming::Method::CRASH);
    }
}

// check for loss of thrust and trigger thrust boost in motors library
void Copter::thrust_loss_check()
{
    static uint16_t thrust_loss_counter;  // number of iterations vehicle may have been crashed

    // no-op if suppresed by flight options param
    if ((copter.g2.flight_options & uint32_t(FlightOptions::DISABLE_THRUST_LOSS_CHECK)) != 0) {
        return;
    }

    // exit immediately if thrust boost is already engaged
    if (motors->get_thrust_boost()) {
        return;
    }

    // return immediately if disarmed
    if (!motors->armed() || ap.land_complete) {
        thrust_loss_counter = 0;
        return;
    }

    // return immediately if in arming delay
    if (ap.in_arming_delay) {
        thrust_loss_counter = 0;
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        return;
    }

    // check for desired angle over 15 degrees
    // todo: add thrust angle to AC_AttitudeControl
    const Vector3f angle_target = attitude_control->get_att_target_euler_cd();
    if (sq(angle_target.x) + sq(angle_target.y) > sq(THRUST_LOSS_CHECK_ANGLE_DEVIATION_CD)) {
        thrust_loss_counter = 0;
        return;
    }

    // check for throttle over 90% or throttle saturation
    if ((attitude_control->get_throttle_in() < THRUST_LOSS_CHECK_MINIMUM_THROTTLE) && (!motors->limit.throttle_upper)) {
        thrust_loss_counter = 0;
        return;
    }

    // check throttle is over 25% to prevent checks triggering from thrust limitations caused by low commanded throttle
    if ((attitude_control->get_throttle_in() < 0.25f)) {
        thrust_loss_counter = 0;
        return;
    }

    // check for descent
    if (!is_negative(inertial_nav.get_velocity_z_up_cms())) {
        thrust_loss_counter = 0;
        return;
    }

    // check for angle error over 30 degrees to ensure the aircraft has attitude control
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error >= CRASH_CHECK_ANGLE_DEVIATION_DEG) {
        thrust_loss_counter = 0;
        return;
    }

    // the aircraft is descending with low requested roll and pitch, at full available throttle, with attitude control
    // we may have lost thrust
    thrust_loss_counter++;

    // check if thrust loss for 1 second
    if (thrust_loss_counter >= (THRUST_LOSS_CHECK_TRIGGER_SEC * scheduler.get_loop_rate_hz())) {
        // reset counter
        thrust_loss_counter = 0;
        AP::logger().Write_Error(LogErrorSubsystem::THRUST_LOSS_CHECK, LogErrorCode::FAILSAFE_OCCURRED);
        // send message to gcs
        gcs().send_text(MAV_SEVERITY_EMERGENCY, "Potential Thrust Loss (%d)", (int)motors->get_lost_motor() + 1);
        // enable thrust loss handling
        motors->set_thrust_boost(true);
        // the motors library disables this when it is no longer needed to achieve the commanded output

#if GRIPPER_ENABLED == ENABLED
        if ((copter.g2.flight_options & uint32_t(FlightOptions::RELEASE_GRIPPER_ON_THRUST_LOSS)) != 0) {
            copter.g2.gripper.release();
        }
#endif
    }
}

// check for a large yaw imbalance, could be due to badly calibrated ESC or misaligned motors
void Copter::yaw_imbalance_check()
{
    // no-op if suppresed by flight options param
    if ((copter.g2.flight_options & uint32_t(FlightOptions::DISABLE_YAW_IMBALANCE_WARNING)) != 0) {
        return;
    }

    // If I is disabled it is unlikely that the issue is not obvious
    if (!is_positive(attitude_control->get_rate_yaw_pid().kI())) {
        return;
    }

    // thrust loss is trigerred, yaw issues are expected
    if (motors->get_thrust_boost()) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // return immediately if disarmed
    if (!motors->armed() || ap.land_complete) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // exit immediately if in standby
    if (standby_active) {
        yaw_I_filt.reset(0.0f);
        return;
    }

    // magnitude of low pass filtered I term
    const float I_term = attitude_control->get_rate_yaw_pid().get_pid_info().I;
    const float I = fabsf(yaw_I_filt.apply(attitude_control->get_rate_yaw_pid().get_pid_info().I,G_Dt));
    if (I > fabsf(I_term)) {
        // never allow to be larger than I
        yaw_I_filt.reset(I_term);
    }

    const float I_max = attitude_control->get_rate_yaw_pid().imax();
    if ((is_positive(I_max) && ((I > YAW_IMBALANCE_IMAX_THRESHOLD * I_max) || (is_equal(I_term,I_max))))) {
        // filtered using over precentage of I max or unfiltered = I max
        // I makes up more than precentage of total available control power
        const uint32_t now = millis();
        if (now - last_yaw_warn_ms > YAW_IMBALANCE_WARN_MS) {
            last_yaw_warn_ms = now;
            gcs().send_text(MAV_SEVERITY_EMERGENCY, "Yaw Imbalance %0.0f%%", I *100);
        }
    }
}

#if PARACHUTE == ENABLED

// Code to detect a crash main ArduCopter code
#define PARACHUTE_CHECK_TRIGGER_SEC         1       // 1 second of loss of control triggers the parachute
#define PARACHUTE_CHECK_ANGLE_DEVIATION_DEG 30.0f   // 30 degrees off from target indicates a loss of control

// returns the sink rate in m/s
float Copter::calculate_sink_rate()
{
    float sink_rate = -inertial_nav.get_velocity_z_up_cms() * 0.01;
    uint8_t active_ekf_cores = AP::ahrs().get_active_core_count();

    if (active_ekf_cores == 0) {
        return sink_rate;
    }

    // we take the second highest velocity from all cores, because the highest might be wrong
    if (active_ekf_cores > 1) { 
        float vertical_velocities[active_ekf_cores] = { 0 };
        Vector3f velNED;
        for (uint8_t core = 0; core < active_ekf_cores; ++core) {
            vertical_velocities[core] = -FLT_MAX;
            if (!AP::ahrs().get_velocity_NED(velNED, core)) {
                continue;
            }
            vertical_velocities[core] = velNED.z;
        }
        std::qsort(vertical_velocities, ARRAY_SIZE(vertical_velocities), sizeof(vertical_velocities[0]), [](const void* a, const void* b) -> int
        {
            float arg1 = *static_cast<const float*>(a);
            float arg2 = *static_cast<const float*>(b);
            return (arg1 > arg2) - (arg1 < arg2);
        });

        float vertical_velocity_candidate = vertical_velocities[ARRAY_SIZE(vertical_velocities) - 2];   // second highest
        if (vertical_velocity_candidate > -FLT_MAX) {
            sink_rate = vertical_velocity_candidate;
        }
    }

    return sink_rate;
}

#ifdef FTS
bool Copter::can_arm_fts()
{
    uint32_t now = AP_HAL::millis();
    static uint32_t last_unhealthy = now;
    if ((now - last_unhealthy) < TIME_BETWEEN_ARM_ATTEMPTS_MS) {
        return false;
    }
    if (AP::compass().is_calibrating()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Compass calibration running");
        last_unhealthy = now;
        return false;
    }
    if (AP::compass().compass_cal_requires_reboot()) {
        gcs().send_text(MAV_SEVERITY_INFO, "Compass calibrated requires reboot");
        last_unhealthy = now;
        return false;
    }
    if (!AP::compass().healthy()) {
        gcs().send_text(MAV_SEVERITY_INFO, "compass isn't ready yet");
        last_unhealthy = now;
        return false;
    }
    uint8_t gps_sensors = AP::gps().num_sensors();
    if (gps_sensors == 0) {
        gcs().send_text(MAV_SEVERITY_INFO, "no GPS sensors detected yet");
        last_unhealthy = now;
        return false;
    }
    for (uint8_t i = 0; i < gps_sensors; ++i) {
        if (!AP::gps().is_healthy(i)) {
            gcs().send_text(MAV_SEVERITY_INFO, "GPS %u isn't ready yet", i);
            last_unhealthy = now;
            return false;
        }
    }

    if (!ahrs.healthy()) {
        gcs().send_text(MAV_SEVERITY_INFO, "AHRS isn't ready yet");
        last_unhealthy = now;
        return false;
    }

    return true;
}
#endif

// parachute_check - disarms motors and triggers the parachute if serious loss of control has been detected
// vehicle is considered to have a "serious loss of control" by the vehicle being more than 30 degrees off from the target roll and pitch angles continuously for 1 second
// called at MAIN_LOOP_RATE
void Copter::parachute_check()
{
    static uint16_t control_loss_count; // number of iterations we have been out of control
    static int32_t baro_alt_start;

#ifdef FTS
    if(!motors->armed()) {
        parachute.safety_switch(false);  // we don't want the parachute to fire on the ground
        parachute.reset();  // always sending pwm off signal when not armed

        if (parachute.is_reboot_necessary()) {
            gcs().send_text(MAV_SEVERITY_INFO, "Externally powered now. rebooting.");
            hal.scheduler->delay(1000);
            hal.scheduler->reboot(false);
        }

        if (!can_arm_fts()) {
            return;
        }

        // always check if inertial nav has started and is ready
        if (parachute.enabled() && AP::arming().pre_arm_checks(false)) {
            AP::arming().arm(AP_Arming::Method::MAVLINK, false);  // after we set the safety_switch to disengaged state (false) - we can arm the FTS
        }
    }
#endif

    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // pass is_flying to parachute library
    parachute.set_is_flying(!ap.land_complete);

    // pass sink rate to parachute library
    parachute.set_sink_rate(-inertial_nav.get_velocity_z_up_cms() * 0.01f);

    // exit immediately if in standby
    if (standby_active) {
        return;
    }

    // call update to give parachute a chance to move servo or relay back to off position
    parachute.update();

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!motors->armed()) {
        control_loss_count = 0;
        return;
    }

#ifdef FTS
    parachute.inflight_tests();

    // exit immediately if safety switch is off
    if (!parachute.is_safety_switch_on()) {
        return;
    }

    // pass sink rate to parachute library
    parachute.set_sink_rate(calculate_sink_rate());

    // call update to give parachute a chance to move servo or relay back to off position
    parachute.update();
#endif

    // exit immediately if in standby
    if (standby_active) {
        return;
    }

    // return immediately if motors are not armed or pilot's throttle is above zero
    if (!motors->armed()) {
        control_loss_count = 0;
        return;
    }

    if (parachute.release_initiated()) {
        copter.arming.disarm(AP_Arming::Method::PARACHUTE_RELEASE);
        return;
    }

    // return immediately if we are not in an angle stabilize flight mode or we are flipping
    if (flightmode->mode_number() == Mode::Number::ACRO || flightmode->mode_number() == Mode::Number::FLIP) {
        control_loss_count = 0;
        return;
    }

    // ensure we are flying or crash checking enabled
    if (ap.land_complete || g.fs_crash_check == 0) {
        control_loss_count = 0;
        return;
    }

    // ensure the first control_loss event is from above the min altitude
    if (control_loss_count == 0 && parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100)) {
        return;
    }

    // check for angle error over 30 degrees
    const float angle_error = attitude_control->get_att_error_angle_deg();
    if (angle_error <= PARACHUTE_CHECK_ANGLE_DEVIATION_DEG) {
        if (control_loss_count > 0) {
            control_loss_count--;
        }
        return;
    }

    // increment counter
    if (control_loss_count < (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        control_loss_count++;
    }

    // record baro alt if we have just started losing control
    if (control_loss_count == 1) {
        baro_alt_start = baro_alt;

    // exit if baro altitude change indicates we are not falling
    } else if (baro_alt >= baro_alt_start) {
        control_loss_count = 0;
        return;

    // To-Do: add check that the vehicle is actually falling

    // check if loss of control for at least 1 second
    } else if (control_loss_count >= (PARACHUTE_CHECK_TRIGGER_SEC*scheduler.get_loop_rate_hz())) {
        // reset control loss counter
        control_loss_count = 0;
        AP::logger().Write_Error(LogErrorSubsystem::CRASH_CHECK, LogErrorCode::CRASH_CHECK_LOSS_OF_CONTROL);
        // release parachute
        parachute_release();
    }
}

// parachute_release - trigger the release of the parachute, disarm the motors and notify the user
void Copter::parachute_release()
{
#ifndef FTS
    // disarm motors
    arming.disarm(AP_Arming::Method::PARACHUTE_RELEASE);
#endif

    // release parachute
    parachute.release();

#ifndef FTS
#if LANDING_GEAR_ENABLED == ENABLED
    // deploy landing gear
    landinggear.set_position(AP_LandingGear::LandingGear_Deploy);
#endif
#endif
}

// parachute_manual_release - trigger the release of the parachute, after performing some checks for pilot error
//   checks if the vehicle is landed 
void Copter::parachute_manual_release()
{
    // exit immediately if safety switch is off
    if (!parachute.is_safety_switch_on()) {
        return;
    }

    // exit immediately if parachute is not enabled
    if (!parachute.enabled()) {
        return;
    }

    // if we are in FTS mode, we shouldn't test anything but FTS is enabled and the safety is engaged
#ifndef FTS
    // do not release if vehicle is landed
    // do not release if we are landed or below the minimum altitude above home
    if (ap.land_complete) {
        // warn user of reason for failure
        gcs().send_text(MAV_SEVERITY_INFO,"Parachute: Landed");
        AP::logger().Write_Error(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_LANDED);
        return;
    }

    // do not release if we are landed or below the minimum altitude above home
    if ((parachute.alt_min() != 0 && (current_loc.alt < (int32_t)parachute.alt_min() * 100))) {
        // warn user of reason for failure
        gcs().send_text(MAV_SEVERITY_ALERT,"Parachute: Too low");
        AP::logger().Write_Error(LogErrorSubsystem::PARACHUTES, LogErrorCode::PARACHUTE_TOO_LOW);
        return;
    }
#endif

    // if we get this far release parachute
    parachute_release();
}

#endif // PARACHUTE == ENABLED
