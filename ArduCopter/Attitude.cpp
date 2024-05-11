#include "Copter.h"

/*************************************************************
 *  Attitude Rate controllers and timing
 ****************************************************************/

/*
  thread for rate control
*/
void Copter::rate_controller_thread()
{
    HAL_BinarySemaphore rate_sem;
    ins.set_rate_loop_sem(&rate_sem);

    uint32_t last_run_us = AP_HAL::micros();
    float max_dt = 0.0;
    float min_dt = 1.0;
    float dt_avg = 0.0;
    uint32_t now_ms = AP_HAL::millis();
    uint32_t last_report_ms = now_ms;
    uint32_t last_rtdt_log_us = last_run_us;
    uint32_t last_notch_sample_ms = now_ms;
    uint32_t loop_delay_ns = 125000;
    bool was_using_rate_thread = false;

    while (true) {
        // allow changing option at runtime
        if (!flight_option_is_set(FlightOptions::USE_RATE_LOOP_THREAD) ||
            ap.motor_test) {
            using_rate_thread = false;
            if (was_using_rate_thread) {
                // if we were using the rate thread, we need to
                // setup the notch filter sample rate
                const float loop_rate_hz = AP::scheduler().get_filtered_loop_rate_hz();
                attitude_control->set_notch_sample_rate(loop_rate_hz);
                motors->set_dt(1.0/loop_rate_hz);
                was_using_rate_thread = false;
            }
            hal.scheduler->delay_microseconds(500);
            last_run_us = AP_HAL::micros();
            continue;
        }

        using_rate_thread = true;

        // wait for an IMU sample
        rate_sem.wait_blocking();
        if (ap.motor_test) {
            continue;
        }

        const uint32_t now_us = AP_HAL::micros();
        const uint32_t dt_us = now_us - last_run_us;
        const float dt = dt_us * 1.0e-6;
        last_run_us = now_us;


        if (is_zero(dt_avg)) {
            dt_avg = dt;
        } else {
            dt_avg = 0.99f * dt_avg + 0.01f * dt;
        }

        max_dt = MAX(dt, max_dt);
        min_dt = MIN(dt, min_dt);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: RTDT
// @Description: Attitude controller time deltas
// @Field: TimeUS: Time since system startup
// @Field: dt: current time delta
// @Field: dtAvg: current time delta average
// @Field: dtMax: Max time delta since last log output
// @Field: dtMin: Min time delta since last log output

        if (now_us - last_rtdt_log_us >= 2500) {    // 400 Hz
            AP::logger().WriteStreaming("RTDT", "TimeUS,dt,dtAvg,dtMax,dtMin", "Qffff",
                                                AP_HAL::micros64(),
                                                dt, dt_avg, max_dt, min_dt);
            max_dt = dt_avg;
            min_dt = dt_avg;
            last_rtdt_log_us = now_us;
        }
#endif

        /*
          run the rate controller. We pass in the long term average dt
          not the per loop dt, as most of the timing jitter is from
          the timing of the FIFO reads on the SPI bus, which does not
          reflect the actual time between IMU samples, which is steady
        */
        motors->set_dt(dt_avg);
        attitude_control->set_dt(dt_avg);
        attitude_control->rate_controller_run();

        /*
          immediately output the new motor values
         */
        motors_output();

        /*
          update the center frequencies of notch filters
         */
        update_dynamic_notch_at_specified_rate();

#if CONFIG_HAL_BOARD != HAL_BOARD_SITL
        if (AP::scheduler().get_extra_loop_us() > 0) {
            // if the main loop is slow, slowly lower the attitude rate
            loop_delay_ns = MIN(loop_delay_ns + 100, 500000);    // min rate 2Khz
        } else {
            loop_delay_ns = MAX(loop_delay_ns - 100, 125000);    // max rate 8Khz
        }
        // ensure we give at least some CPU to other threads
        // don't sleep on SITL where small sleeps are not possible
        hal.scheduler->delay_microseconds(loop_delay_ns * 1e-3);
#endif

        now_ms = AP_HAL::millis();

        if (now_ms - last_notch_sample_ms >= 1000 || !was_using_rate_thread) {
            // update the PID notch sample rate at 1Hz if if we are
            // enabled at runtime
            last_notch_sample_ms = now_ms;
            attitude_control->set_notch_sample_rate(1.0 / dt_avg);
        }
        
        if (now_ms - last_report_ms >= 200) {
            last_report_ms = now_ms;
            gcs().send_named_float("LRATE", 1.0/dt_avg);
        }

        was_using_rate_thread = true;
    }
}

/*
  update rate controller when run from main thread (normal operation)
*/
void Copter::run_rate_controller_main()
{
    // set attitude and position controller loop time
    const float last_loop_time_s = AP::scheduler().get_last_loop_time_s();
    pos_control->set_dt(last_loop_time_s);

    if (!using_rate_thread) {
        motors->set_dt(last_loop_time_s);
        attitude_control->set_dt(last_loop_time_s);
        // only run the rate controller if we are not using the rate thread
        attitude_control->rate_controller_run();
    }
}

/*************************************************************
 *  throttle control
 ****************************************************************/

// update estimated throttle required to hover (if necessary)
//  called at 100hz
void Copter::update_throttle_hover()
{
    // if not armed or landed or on standby then exit
    if (!motors->armed() || ap.land_complete || standby_active) {
        return;
    }

    // do not update in manual throttle modes or Drift
    if (flightmode->has_manual_throttle() || (copter.flightmode->mode_number() == Mode::Number::DRIFT)) {
        return;
    }

    // do not update while climbing or descending
    if (!is_zero(pos_control->get_vel_desired_cms().z)) {
        return;
    }

    // get throttle output
    float throttle = motors->get_throttle();

    // calc average throttle if we are in a level hover.  accounts for heli hover roll trim
    if (throttle > 0.0f && fabsf(inertial_nav.get_velocity_z_up_cms()) < 60 &&
        fabsf(ahrs.roll_sensor-attitude_control->get_roll_trim_cd()) < 500 && labs(ahrs.pitch_sensor) < 500) {
        // Can we set the time constant automatically
        motors->update_throttle_hover(0.01f);
#if HAL_GYROFFT_ENABLED
        gyro_fft.update_freq_hover(0.01f, motors->get_throttle_out());
#endif
    }
}

// get_pilot_desired_climb_rate - transform pilot's throttle input to climb rate in cm/s
// without any deadzone at the bottom
float Copter::get_pilot_desired_climb_rate(float throttle_control)
{
    // throttle failsafe check
    if (failsafe.radio || !ap.rc_receiver_present) {
        return 0.0f;
    }

#if TOY_MODE_ENABLED == ENABLED
    if (g2.toy_mode.enabled()) {
        // allow throttle to be reduced after throttle arming and for
        // slower descent close to the ground
        g2.toy_mode.throttle_adjust(throttle_control);
    }
#endif

    // ensure a reasonable throttle value
    throttle_control = constrain_float(throttle_control,0.0f,1000.0f);

    // ensure a reasonable deadzone
    g.throttle_deadzone.set(constrain_int16(g.throttle_deadzone, 0, 400));

    float desired_rate = 0.0f;
    const float mid_stick = get_throttle_mid();
    const float deadband_top = mid_stick + g.throttle_deadzone;
    const float deadband_bottom = mid_stick - g.throttle_deadzone;

    // check throttle is above, below or in the deadband
    if (throttle_control < deadband_bottom) {
        // below the deadband
        desired_rate = get_pilot_speed_dn() * (throttle_control-deadband_bottom) / deadband_bottom;
    } else if (throttle_control > deadband_top) {
        // above the deadband
        desired_rate = g.pilot_speed_up * (throttle_control-deadband_top) / (1000.0f-deadband_top);
    } else {
        // must be in the deadband
        desired_rate = 0.0f;
    }

    return desired_rate;
}

// get_non_takeoff_throttle - a throttle somewhere between min and mid throttle which should not lead to a takeoff
float Copter::get_non_takeoff_throttle()
{
    return MAX(0,motors->get_throttle_hover()/2.0f);
}

// set_accel_throttle_I_from_pilot_throttle - smoothes transition from pilot controlled throttle to autopilot throttle
void Copter::set_accel_throttle_I_from_pilot_throttle()
{
    // get last throttle input sent to attitude controller
    float pilot_throttle = constrain_float(attitude_control->get_throttle_in(), 0.0f, 1.0f);
    // shift difference between pilot's throttle and hover throttle into accelerometer I
    pos_control->get_accel_z_pid().set_integrator((pilot_throttle-motors->get_throttle_hover()) * 1000.0f);
}

// rotate vector from vehicle's perspective to North-East frame
void Copter::rotate_body_frame_to_NE(float &x, float &y)
{
    float ne_x = x*ahrs.cos_yaw() - y*ahrs.sin_yaw();
    float ne_y = x*ahrs.sin_yaw() + y*ahrs.cos_yaw();
    x = ne_x;
    y = ne_y;
}

// It will return the PILOT_SPEED_DN value if non zero, otherwise if zero it returns the PILOT_SPEED_UP value.
uint16_t Copter::get_pilot_speed_dn() const
{
    if (g2.pilot_speed_dn == 0) {
        return abs(g.pilot_speed_up);
    } else {
        return abs(g2.pilot_speed_dn);
    }
}
