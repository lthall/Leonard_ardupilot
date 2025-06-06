/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  parent class for aircraft simulators
*/

#include "SIM_Aircraft.h"

#include <stdio.h>
#include <sys/time.h>
#include <unistd.h>

#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_JSON/AP_JSON.h>
#include <AP_Filesystem/AP_Filesystem.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL_SITL/HAL_SITL_Class.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

// the SITL HAL can add information about pausing the simulation and its effect on the UART.  Not present when we're compiling for simulation-on-hardware
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
extern const HAL_SITL& hal_sitl;
#endif

Aircraft *Aircraft::instances[MAX_SIM_INSTANCES];

/*
  parent class for all simulator types
 */

Aircraft::Aircraft(const char *frame_str) :
    frame(frame_str)
{
    // make the SIM_* variables available to simulator backends
    sitl = AP::sitl();

    set_speedup(1.0f);

    last_wall_time_us = get_wall_time_us();

    // allow for orientation settings, such as with tailsitters
    enum ap_var_type ptype;
    ahrs_orientation = (AP_Int8 *)AP_Param::find("AHRS_ORIENTATION", &ptype);

    // ahrs_orientation->get() returns ROTATION_NONE here, regardless of the actual value
    enum Rotation imu_rotation = ahrs_orientation?(enum Rotation)ahrs_orientation->get():ROTATION_NONE;
    last_imu_rotation = imu_rotation;
    // sitl is null if running example program
    if (sitl) {
        sitl->ahrs_rotation.from_rotation(imu_rotation);
        sitl->ahrs_rotation_inv = sitl->ahrs_rotation.transposed();
    }

    // init rangefinder array to NaN to signify no data
    for (uint8_t i = 0; i < ARRAY_SIZE(rangefinder_m); i++){
        rangefinder_m[i] = nanf("");
    }
}

void Aircraft::set_start_location(const Location &start_loc, const float start_yaw)
{
    home = start_loc;
    origin = home;
    position.xy().zero();
    home_yaw = start_yaw;
    home_is_set = true;

    ::printf("Home: %f %f alt=%fm hdg=%f\n",
             home.lat*1e-7,
             home.lng*1e-7,
             home.alt*0.01,
             home_yaw);

    location = home;
    ground_level = home.alt * 0.01f;

#if 0
    // useful test for home position being very different from origin
    home.offset(-3000*1000, 1800*1000);
#endif

    dcm.from_euler(0.0f, 0.0f, radians(home_yaw));
}

/*
   return difference in altitude between home position and current loc
*/
float Aircraft::ground_height_difference() const
{
#if AP_TERRAIN_AVAILABLE
    AP_Terrain *terrain = AP::terrain();
    float h1, h2;
    if (sitl &&
        terrain != nullptr &&
        sitl->terrain_enable &&
        terrain->height_amsl(home, h1, false) &&
        terrain->height_amsl(location, h2, false)) {
        h2 += local_ground_level;
        return h2 - h1;
    }
#endif
    return local_ground_level;
}

float Aircraft::ambient_temperature_degC() const
{
    // FIXME: AP_Baro_SITL should be getting temperature from the
    // simulated aircraft, not the other way around!
#if !APM_BUILD_TYPE(APM_BUILD_AP_Periph)    // Periph does not instantiate Baro
    return AP::baro().get_temperature();
#endif
    return 25.0;
}

void Aircraft::set_precland(SIM_Precland *_precland) {
    precland = _precland;
    precland->set_default_location(home.lat * 1.0e-7f, home.lng * 1.0e-7f, static_cast<int16_t>(get_home_yaw()));
}

/*
   return current height above ground level (metres)
*/
float Aircraft::hagl() const
{
    return (-position.z) + home.alt * 0.01f - ground_level - frame_height - ground_height_difference();
}

/*
   return true if we are on the ground
*/
bool Aircraft::on_ground() const
{
    return hagl() <= 0.001f;  // prevent bouncing around ground
}

/*
   update location from position
*/
void Aircraft::update_position(void)
{
    location = origin;
    location.offset(position.x, position.y);

    location.alt  = static_cast<int32_t>(home.alt - position.z * 100.0f);

#if 0
    Vector3d pos_home = position;
    pos_home.xy() += home.get_distance_NE_double(origin);

    // logging of raw sitl data
    Vector3f accel_ef = dcm * accel_body;
// @LoggerMessage: SITL
// @Description: Simulation data
// @Field: TimeUS: Time since system startup
// @Field: VN: Velocity - North component
// @Field: VE: Velocity - East component
// @Field: VD: Velocity - Down component
// @Field: AN: Acceleration - North component
// @Field: AE: Acceleration - East component
// @Field: AD: Acceleration - Down component
// @Field: PN: Position - North component
// @Field: PE: Position - East component
// @Field: PD: Position - Down component
    AP::logger().WriteStreaming("SITL", "TimeUS,VN,VE,VD,AN,AE,AD,PN,PE,PD", "Qfffffffff",
                                           AP_HAL::micros64(),
                                           velocity_ef.x, velocity_ef.y, velocity_ef.z,
                                           accel_ef.x, accel_ef.y, accel_ef.z,
                                           pos_home.x, pos_home.y, pos_home.z);
#endif

    if (!disable_origin_movement) {
        uint32_t now = AP_HAL::millis();
        if (now - last_one_hz_ms >= 1000) {
            // shift origin of position at 1Hz to current location
            // this prevents spherical errors building up in the GPS data
            last_one_hz_ms = now;
            Vector2d diffNE = origin.get_distance_NE_double(location);
            position.xy() -= diffNE;
            smoothing.position.xy() -= diffNE;
            origin.lat = location.lat;
            origin.lng = location.lng;
        }
    }
}

/*
   update body magnetic field from position and rotation
*/
void Aircraft::update_mag_field_bf()
{
    // get the magnetic field intensity and orientation
    float intensity;
    float declination;
    float inclination;
    AP_Declination::get_mag_field_ef(location.lat * 1e-7f, location.lng * 1e-7f, intensity, declination, inclination);

    // create a field vector and rotate to the required orientation
    Vector3f mag_ef(1e3f * intensity, 0.0f, 0.0f);
    Matrix3f R;
    R.from_euler(0.0f, -radians(inclination), radians(declination));
    mag_ef = R * mag_ef;

    // calculate frame height above ground
    const float frame_height_agl = fmaxf((-position.z) + home.alt * 0.01f - ground_level, 0.0f);

    if (!sitl) {
        // running example program
        return;
    }

    // calculate scaling factor that varies from 1 at ground level to 1/8 at sitl->mag_anomaly_hgt
    // Assume magnetic anomaly strength scales with 1/R**3
    float anomaly_scaler = (sitl->mag_anomaly_hgt / (frame_height_agl + sitl->mag_anomaly_hgt));
    anomaly_scaler = anomaly_scaler * anomaly_scaler * anomaly_scaler;

    // add scaled anomaly to earth field
    mag_ef += sitl->mag_anomaly_ned.get() * anomaly_scaler;

    // Rotate into body frame
    mag_bf = dcm.transposed() * mag_ef;

    // add motor interference
    mag_bf += sitl->mag_mot.get() * battery_current;
}

/* advance time by deltat in seconds */
void Aircraft::time_advance()
{
    // we only advance time if it hasn't been advanced already by the
    // backend
    if (last_time_us == time_now_us) {
        time_now_us += frame_time_us;
    }
    last_time_us = time_now_us;
    if (use_time_sync) {
        sync_frame_time();
    }
}

/* setup the frame step time */
void Aircraft::setup_frame_time(float new_rate, float new_speedup)
{
    rate_hz = new_rate;
    target_speedup = new_speedup;
    frame_time_us = uint64_t(1.0e6f/rate_hz);

    last_wall_time_us = get_wall_time_us();
}

/* adjust frame_time calculation */
void Aircraft::adjust_frame_time(float new_rate)
{
    frame_time_us = uint64_t(1.0e6f/new_rate);
    rate_hz = new_rate;
}

/*
   try to synchronise simulation time with wall clock time, taking
   into account desired speedup
   This tries to take account of possible granularity of
   get_wall_time_us() so it works reasonably well on windows
*/
void Aircraft::sync_frame_time(void)
{
    frame_counter++;
    uint64_t now = get_wall_time_us();
    uint64_t dt_us = now - last_wall_time_us;

    const float target_dt_us = 1.0e6/(rate_hz*target_speedup);

    // accumulate sleep debt if we're running too fast
    sleep_debt_us += target_dt_us - dt_us;

    if (sleep_debt_us < -1.0e5) {
        // don't let a large negative debt build up
        sleep_debt_us = -1.0e5;
    }
    if (sleep_debt_us > min_sleep_time) {
        // sleep if we have built up a debt of min_sleep_tim
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        usleep(sleep_debt_us);
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        hal.scheduler->delay_microseconds(sleep_debt_us);
#else
        // ??
#endif
        sleep_debt_us -= (get_wall_time_us() - now);
    }
    last_wall_time_us = get_wall_time_us();

    uint32_t now_ms = last_wall_time_us / 1000ULL;
    float dt_wall = (now_ms - last_fps_report_ms) * 0.001;
    if (dt_wall > 0.01) {  // 0.01s average
        achieved_rate_hz = (frame_counter - last_frame_count) / dt_wall;
#if 0
        ::printf("Rate: target:%.1f achieved:%.1f speedup %.1f/%.1f\n",
                 rate_hz*target_speedup, achieved_rate_hz,
                 achieved_rate_hz/rate_hz, target_speedup);
#endif
        last_frame_count = frame_counter;
        last_fps_report_ms = now_ms;
    }
}

/* add noise based on throttle level (from 0..1) */
void Aircraft::add_noise(float throttle)
{
    gyro += Vector3f(rand_normal(0, 1),
                     rand_normal(0, 1),
                     rand_normal(0, 1)) * gyro_noise * fabsf(throttle);
    accel_body += Vector3f(rand_normal(0, 1),
                           rand_normal(0, 1),
                           rand_normal(0, 1)) * accel_noise * fabsf(throttle);
}

/*
  normal distribution random numbers
  See
  http://en.literateprograms.org/index.php?title=Special:DownloadCode/Box-Muller_transform_%28C%29&oldid=7011
*/
double Aircraft::rand_normal(double mean, double stddev)
{
    static double n2 = 0.0;
    static int n2_cached = 0;
    if (!n2_cached) {
        double x, y, r;
        do
        {
            x = 2.0 * rand()/RAND_MAX - 1;
            y = 2.0 * rand()/RAND_MAX - 1;
            r = x*x + y*y;
        } while (is_zero(r) || r > 1.0);
        const double d = sqrt(-2.0 * log(r)/r);
        const double n1 = x * d;
        n2 = y * d;
        const double result = n1 * stddev + mean;
        n2_cached = 1;
        return result;
    } else {
        n2_cached = 0;
        return n2 * stddev + mean;
    }
}




/*
   fill a sitl_fdm structure from the simulator state
*/
void Aircraft::fill_fdm(struct sitl_fdm &fdm)
{
    bool is_smoothed = false;
    if (use_smoothing) {
        smooth_sensors();
        is_smoothed = true;
    }
    fdm.timestamp_us = time_now_us;
    if (fdm.home.lat == 0 && fdm.home.lng == 0) {
        // initialise home
        fdm.home = home;
    }
    fdm.is_lock_step_scheduled = lock_step_scheduled;
    fdm.latitude  = location.lat * 1.0e-7;
    fdm.longitude = location.lng * 1.0e-7;
    fdm.altitude  = location.alt * 1.0e-2;
    fdm.heading   = degrees(atan2f(velocity_ef.y, velocity_ef.x));
    fdm.speedN    = velocity_ef.x;
    fdm.speedE    = velocity_ef.y;
    fdm.speedD    = velocity_ef.z;
    fdm.xAccel    = accel_body.x;
    fdm.yAccel    = accel_body.y;
    fdm.zAccel    = accel_body.z;
    fdm.rollRate  = degrees(gyro.x);
    fdm.pitchRate = degrees(gyro.y);
    fdm.yawRate   = degrees(gyro.z);
    float r, p, y;
    dcm.to_euler(&r, &p, &y);
    fdm.rollDeg  = degrees(r);
    fdm.pitchDeg = degrees(p);
    fdm.yawDeg   = degrees(y);
    fdm.quaternion.from_rotation_matrix(dcm);
    fdm.airspeed = airspeed_pitot;
    fdm.velocity_air_bf = velocity_air_bf;
    fdm.battery_voltage = battery_voltage;
    fdm.battery_current = battery_current;
    fdm.motor_mask = motor_mask | sitl->vibe_motor_mask;
    memcpy(fdm.rpm, rpm, sizeof(fdm.rpm));
    fdm.rcin_chan_count = rcin_chan_count;
    fdm.range = rangefinder_range();
    memcpy(fdm.rcin, rcin, rcin_chan_count * sizeof(float));
    fdm.bodyMagField = mag_bf;

    fdm.bodyMagField.rotate(sitl->imu_orientation);

    // copy laser scanner results
    fdm.scanner.points = scanner.points;
    fdm.scanner.ranges = scanner.ranges;

    // copy rangefinder
    memcpy(fdm.rangefinder_m, rangefinder_m, sizeof(fdm.rangefinder_m));

    fdm.wind_vane_apparent.direction = wind_vane_apparent.direction;
    fdm.wind_vane_apparent.speed = wind_vane_apparent.speed;

    fdm.wind_ef = wind_ef;

    if (is_smoothed) {
        fdm.xAccel = smoothing.accel_body.x;
        fdm.yAccel = smoothing.accel_body.y;
        fdm.zAccel = smoothing.accel_body.z;
        fdm.rollRate  = degrees(smoothing.gyro.x);
        fdm.pitchRate = degrees(smoothing.gyro.y);
        fdm.yawRate   = degrees(smoothing.gyro.z);
        fdm.speedN    = smoothing.velocity_ef.x;
        fdm.speedE    = smoothing.velocity_ef.y;
        fdm.speedD    = smoothing.velocity_ef.z;
        fdm.latitude  = smoothing.location.lat * 1.0e-7;
        fdm.longitude = smoothing.location.lng * 1.0e-7;
        fdm.altitude  = smoothing.location.alt * 1.0e-2;
    }


    if (ahrs_orientation != nullptr) {
        enum Rotation imu_rotation = (enum Rotation)ahrs_orientation->get();
        if (imu_rotation != last_imu_rotation) {
            sitl->ahrs_rotation.from_rotation(imu_rotation);
            sitl->ahrs_rotation_inv = sitl->ahrs_rotation.transposed();
            last_imu_rotation = imu_rotation;
        }
        if (imu_rotation != ROTATION_NONE) {
            Matrix3f m = dcm;
            m = m * sitl->ahrs_rotation_inv;

            m.to_euler(&r, &p, &y);
            fdm.rollDeg  = degrees(r);
            fdm.pitchDeg = degrees(p);
            fdm.yawDeg   = degrees(y);
            fdm.quaternion.from_rotation_matrix(m);
        }
    }

    // in the first call here, if a speedup option is specified, overwrite it
    if (is_equal(last_speedup, -1.0f) && !is_equal(get_speedup(), 1.0f)) {
        sitl->speedup.set(get_speedup());
    }
    
    if (!is_equal(last_speedup, float(sitl->speedup)) && sitl->speedup > 0) {
        set_speedup(sitl->speedup);
        last_speedup = sitl->speedup;
    }

#if HAL_LOGGING_ENABLED
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // the SITL HAL can add information about pausing the simulation
    // and its effect on the UART.  Not present when we're compiling
    // for simulation-on-hardware
    const uint32_t full_count = hal_sitl.get_uart_output_full_queue_count();
#else
    const uint32_t full_count = 0;
#endif
    // for EKF comparison log relhome pos and velocity at loop rate
    static uint16_t last_ticks;
    // FIXME: stop using AP_Scheduler here!  It's from "the other side"
    const auto *scheduler = AP_Scheduler::get_singleton();
    if (scheduler == nullptr) {
        // not instantiated in examples
        return;
    }
    uint16_t ticks = scheduler->ticks();
    if (last_ticks != ticks) {
        last_ticks = ticks;
// @LoggerMessage: SIM2
// @Description: Additional simulator state
// @Field: TimeUS: Time since system startup
// @Field: PN: North position from home
// @Field: PE: East position from home
// @Field: PD: Down position from home
// @Field: VN: Velocity north
// @Field: VE: Velocity east
// @Field: VD: Velocity down
// @Field: As: Airspeed
// @Field: ASpdU: Achieved simulation speedup value
// @Field: UFC: Number of times simulation paused for serial0 output
        Vector3d pos = get_position_relhome();
        Vector3f vel = get_velocity_ef();
        AP::logger().WriteStreaming(
            "SIM2",
            "TimeUS,PN,PE,PD,VN,VE,VD,As,ASpdU,UFC",
            "QdddfffffI",
            AP_HAL::micros64(),
            pos.x, pos.y, pos.z,
            vel.x, vel.y, vel.z,
            airspeed_pitot,
            achieved_rate_hz/rate_hz,
            full_count
        );
    }
#endif
}

/*
  rover and copter have special handling for horizontal rangefinders
  as distance to obstacles - this takes effect for yaw-only
  orientations
 */
#define SITL_RANGEFINDER_AS_OBJECT_SENSOR (APM_BUILD_TYPE(APM_BUILD_ArduCopter) || APM_BUILD_TYPE(APM_BUILD_Rover))
#define SITL_RANGEFINDER_IS_YAW_ONLY(orientation) (orientation <= ROTATION_YAW_315)

// returns perpendicular height to surface rangefinder is bouncing off
float Aircraft::perpendicular_distance_to_rangefinder_surface() const
{
#if SITL_RANGEFINDER_AS_OBJECT_SENSOR
    const auto orientation = (Rotation)sitl->sonar_rot.get();
    if (SITL_RANGEFINDER_IS_YAW_ONLY(orientation)) {
        // assume these are avoidance sensors
        return sitl->measure_distance_at_angle_bf(location, sitl->sonar_rot.get()*45);
    }
#endif

    // default is ground sensing rangefinders
    return sitl->state.height_agl;
}

float Aircraft::rangefinder_range() const
{
    float altitude = perpendicular_distance_to_rangefinder_surface();

    // sensor position offset in body frame
    const Vector3f relPosSensorBF = sitl->rngfnd_pos_offset;

    // n.b. the following code is assuming rotation-pitch-270:
    // adjust altitude for position of the sensor on the vehicle if position offset is non-zero
    if (!relPosSensorBF.is_zero()) {
        // get a rotation matrix following DCM conventions (body to earth)
        Matrix3f rotmat;
        sitl->state.quaternion.rotation_matrix(rotmat);
        // rotate the offset into earth frame
        const Vector3f relPosSensorEF = rotmat * relPosSensorBF;
        // correct the altitude at the sensor
        altitude -= relPosSensorEF.z;
    }

    const auto orientation = (Rotation)sitl->sonar_rot.get();
#if SITL_RANGEFINDER_AS_OBJECT_SENSOR

    float roll = sitl->state.rollDeg;
    float pitch = sitl->state.pitchDeg;

    if (roll > 0) {
        roll -= rangefinder_beam_width();
        if (roll < 0) {
            roll = 0;
        }
    } else {
        roll += rangefinder_beam_width();
        if (roll > 0) {
            roll = 0;
        }
    }
    if (pitch > 0) {
        pitch -= rangefinder_beam_width();
        if (pitch < 0) {
            pitch = 0;
        }
    } else {
        pitch += rangefinder_beam_width();
        if (pitch > 0) {
            pitch = 0;
        }
    }

    /*
      rover and copter using SITL rangefinders as obstacle sensors
     */
    if (SITL_RANGEFINDER_IS_YAW_ONLY(orientation)) {
        if (fabs(roll) >= 90.0 || fabs(pitch) >= 90.0) {
            // not going to hit the ground....
            return INFINITY;
        }
        altitude /= cosf(radians(roll)) * cosf(radians(pitch));
    } else
#endif
    {
        // adjust for rotation based on orientation of the sensor
        Matrix3f rotmat;
        sitl->state.quaternion.rotation_matrix(rotmat);
        Vector3f v{1, 0, 0};
        v.rotate(orientation);
        v = rotmat * v;

        if (!is_positive(v.z)) {
            return INFINITY;
        }
        altitude /= v.z;

        // this is awful, but there are drawbacks to assuming an
        // infinite plane.  If we don't do this here then we end up
        // with a ridiculous rangefinder range, and that can cause
        // floating point exceptions when we return a distance in cm
        // from the AP_RangeFinder_SITL.
        if (altitude > 100000) {
            return INFINITY;
        }
    }

    // Add some noise on reading
    altitude += sitl->sonar_noise * rand_float();

    // our starting positions can disagree with the terrain database:
    if (altitude < 0) {
        altitude = 0;
    }

    return altitude;
}

#if defined(__CYGWIN__) || defined(__CYGWIN64__)
extern "C" { uint32_t timeGetTime(); }
#endif

// potentially replace this with a call to AP_HAL::Util::get_hw_rtc
uint64_t Aircraft::get_wall_time_us() const
{
#if defined(__CYGWIN__) || defined(__CYGWIN64__)
    static uint32_t tPrev;
    static uint64_t last_ret_us;
    if (tPrev == 0) {
        tPrev = timeGetTime();
        return 0;
    }
    uint32_t now = timeGetTime();
    last_ret_us += (uint64_t)((now - tPrev)*1000UL);
    tPrev = now;
    return last_ret_us;
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint64_t(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
#else
    return AP_HAL::micros64();
#endif
}

/*
  set simulation speedup
 */
void Aircraft::set_speedup(float speedup)
{
    setup_frame_time(rate_hz, speedup);
}

void Aircraft::update_home()
{
    if (!home_is_set) {
        if (sitl == nullptr) {
            return;
        }
        const Location loc{
            int32_t(sitl->opos.lat.get() * 1.0e7),
            int32_t(sitl->opos.lng.get() * 1.0e7),
            int32_t(sitl->opos.alt.get() * 1.0e2),
            Location::AltFrame::ABSOLUTE
        };
        set_start_location(loc, sitl->opos.hdg.get());
    }
}

void Aircraft::update_model(const struct sitl_input &input)
{
    local_ground_level = 0.0f;
    if (sitl != nullptr) {
        update(input);
    } else {
        time_advance();
    }
}

/*
  update the simulation attitude and relative position
 */
void Aircraft::update_dynamics(const Vector3f &rot_accel)
{
    WITH_SEMAPHORE(pose_sem);

    // update eas2tas and air density
#if AP_AHRS_ENABLED
    eas2tas = AP::ahrs().get_EAS2TAS();
#endif
    air_density = SSL_AIR_DENSITY / sq(eas2tas);

    const float delta_time = frame_time_us * 1.0e-6f;

    // update eas2tas and air density
    eas2tas = AP_Baro::get_EAS2TAS_for_alt_amsl(location.alt*0.01);
    air_density = AP_Baro::get_air_density_for_alt_amsl(location.alt*0.01);

    // update rotational rates in body frame
    gyro += rot_accel * delta_time;

    gyro.x = constrain_float(gyro.x, -radians(2000.0f), radians(2000.0f));
    gyro.y = constrain_float(gyro.y, -radians(2000.0f), radians(2000.0f));
    gyro.z = constrain_float(gyro.z, -radians(2000.0f), radians(2000.0f));

    // limit body accel to 64G
    const float accel_limit = 64*GRAVITY_MSS;
    accel_body.x = constrain_float(accel_body.x, -accel_limit, accel_limit);
    accel_body.y = constrain_float(accel_body.y, -accel_limit, accel_limit);
    accel_body.z = constrain_float(accel_body.z, -accel_limit, accel_limit);

    // update attitude
    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    Vector3f accel_earth = dcm * accel_body;
    accel_earth += Vector3f(0.0f, 0.0f, GRAVITY_MSS);

    // if we're on the ground, then our vertical acceleration is limited
    // to zero. This effectively adds the force of the ground on the aircraft
    if (on_ground() && accel_earth.z > 0) {
        accel_earth.z = 0;
    }

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0.0f, 0.0f, -GRAVITY_MSS));

    // new velocity vector
    velocity_ef += accel_earth * delta_time;

    const bool was_on_ground = on_ground();
    // new position vector
    position += (velocity_ef * delta_time).todouble();

    // velocity relative to air mass, in earth frame
    velocity_air_ef = velocity_ef - wind_ef;

    // velocity relative to airmass in body frame
    velocity_air_bf = dcm.transposed() * velocity_air_ef;

    // airspeed
    update_eas_airspeed();

    // constrain height to the ground
    if (on_ground()) {
        if (!was_on_ground && AP_HAL::millis() - last_ground_contact_ms > 1000) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SIM Hit ground at %f m/s", velocity_ef.z);
            last_ground_contact_ms = AP_HAL::millis();
        }
        position.z = -(ground_level + frame_height - home.alt * 0.01f + ground_height_difference());

        // get speed of ground movement (for ship takeoff/landing)
        float yaw_rate = 0;
#if AP_SIM_SHIP_ENABLED
        const Vector2f ship_movement = sitl->models.shipsim.get_ground_speed_adjustment(location, yaw_rate);
        const Vector3f gnd_movement(ship_movement.x, ship_movement.y, 0);
#else
        const Vector3f gnd_movement;
#endif
        switch (ground_behavior) {
        case GROUND_BEHAVIOR_NONE:
            break;
        case GROUND_BEHAVIOR_NO_MOVEMENT: {
            // zero roll/pitch, but keep yaw
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            y = y + yaw_rate * delta_time;
            dcm.from_euler(0.0f, 0.0f, y);
            // X, Y movement tracks ground movement
            velocity_ef.x = gnd_movement.x;
            velocity_ef.y = gnd_movement.y;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            use_smoothing = true;
            break;
        }
        case GROUND_BEHAVIOR_FWD_ONLY: {
            // zero roll/pitch, but keep yaw
            float r, p, y;
            dcm.to_euler(&r, &p, &y);
            if (velocity_ef.length() < 5) {
                // at high speeds don't constrain pitch, otherwise we
                // can get stuck in takeoff
                p = 0;
            } else {
                p = MAX(p, 0);
            }
            y = y + yaw_rate * delta_time;
            dcm.from_euler(0.0f, p, y);
            // only fwd movement
            Vector3f v_bf = dcm.transposed() * velocity_ef;
            v_bf.y = 0.0f;
            if (v_bf.x < 0.0f) {
                v_bf.x = 0.0f;
            }

            Vector3f gnd_movement_bf = dcm.transposed() * gnd_movement;

            // lateral speed equals ground movement
            v_bf.y = gnd_movement_bf.y;

            if (!gnd_movement_bf.is_zero()) {
                // fwd speed slowly approaches ground movement to simulate wheel friction
                const float tconst = 20; // seconds
                const float alpha = delta_time/(delta_time+tconst/M_2PI);
                v_bf.x += (gnd_movement.x - v_bf.x) * alpha;
            }

            velocity_ef = dcm * v_bf;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            gyro.z = yaw_rate;
            use_smoothing = true;
            break;
        }
        case GROUND_BEHAVIOR_TAILSITTER: {
            // rotate normal refernce frame to get yaw angle, then rotate back
            Matrix3f rot;
            rot.from_rotation(ROTATION_PITCH_270);
            float r, p, y;
            (dcm * rot).to_euler(&r, &p, &y);
            y = y + yaw_rate * delta_time;
            dcm.from_euler(0.0, 0.0, y);
            rot.from_rotation(ROTATION_PITCH_90);
            dcm *= rot;
            // X, Y movement tracks ground movement
            velocity_ef.x = gnd_movement.x;
            velocity_ef.y = gnd_movement.y;
            if (velocity_ef.z > 0.0f) {
                velocity_ef.z = 0.0f;
            }
            gyro.zero();
            gyro.x = yaw_rate;
            use_smoothing = true;
            break;
        }
        }
    }

    // update slung payload
#if AP_SIM_SLUNGPAYLOAD_ENABLED
    sitl->models.slung_payload_sim.update(get_position_relhome(), velocity_ef, accel_earth, wind_ef);
#endif

    // update tether
#if AP_SIM_TETHER_ENABLED
    sitl->models.tether_sim.update(location);
#endif

    // allow for changes in physics step
    adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));
}

/*
  update wind vector
*/
void Aircraft::update_wind(const struct sitl_input &input)
{
    // wind vector in earth frame
    wind_ef = Vector3f(cosf(radians(input.wind.direction))*cosf(radians(input.wind.dir_z)), 
                       sinf(radians(input.wind.direction))*cosf(radians(input.wind.dir_z)), 
                       sinf(radians(input.wind.dir_z))) * input.wind.speed;

    wind_ef.z += get_local_updraft(position + home.get_distance_NED_double(origin));

    const float wind_turb = input.wind.turbulence * 10.0f;  // scale input.wind.turbulence to match standard deviation when using iir_coef=0.98
    const float iir_coef = 0.98f;  // filtering high frequencies from turbulence

    if (wind_turb > 0 && !on_ground()) {

        turbulence_azimuth = turbulence_azimuth + (2 * rand());

        turbulence_horizontal_speed =
                static_cast<float>(turbulence_horizontal_speed * iir_coef+wind_turb * rand_normal(0, 1) * (1 - iir_coef));

        turbulence_vertical_speed = static_cast<float>((turbulence_vertical_speed * iir_coef) + (wind_turb * rand_normal(0, 1) * (1 - iir_coef)));

        wind_ef += Vector3f(
            cosf(radians(turbulence_azimuth)) * turbulence_horizontal_speed,
            sinf(radians(turbulence_azimuth)) * turbulence_horizontal_speed,
            turbulence_vertical_speed);
    }

    // the AHRS wants wind with opposite sense
    wind_ef = -wind_ef;
}

/*
  smooth sensors for kinematic consistancy when we interact with the ground
 */
void Aircraft::smooth_sensors(void)
{
    uint64_t now = time_now_us;
    Vector3d delta_pos = position - smoothing.position;
    if (smoothing.last_update_us == 0 || delta_pos.length() > 10) {
        smoothing.position = position;
        smoothing.rotation_b2e = dcm;
        smoothing.accel_body = accel_body;
        smoothing.velocity_ef = velocity_ef;
        smoothing.gyro = gyro;
        smoothing.last_update_us = now;
        smoothing.location = location;
        printf("Smoothing reset at %.3f\n", now * 1.0e-6f);
        return;
    }
    const float delta_time = (now - smoothing.last_update_us) * 1.0e-6f;
    if (delta_time < 0 || delta_time > 0.1) {
        return;
    }

    // calculate required accel to get us to desired position and velocity in the time_constant
    const float time_constant = 0.1f;
    Vector3f dvel = (velocity_ef - smoothing.velocity_ef) + (delta_pos / time_constant).tofloat();
    Vector3f accel_e = dvel / time_constant + (dcm * accel_body + Vector3f(0.0f, 0.0f, GRAVITY_MSS));
    const float accel_limit = 14 * GRAVITY_MSS;
    accel_e.x = constrain_float(accel_e.x, -accel_limit, accel_limit);
    accel_e.y = constrain_float(accel_e.y, -accel_limit, accel_limit);
    accel_e.z = constrain_float(accel_e.z, -accel_limit, accel_limit);
    smoothing.accel_body = smoothing.rotation_b2e.transposed() * (accel_e + Vector3f(0.0f, 0.0f, -GRAVITY_MSS));

    // calculate rotational rate to get us to desired attitude in time constant
    Quaternion desired_q, current_q, error_q;
    desired_q.from_rotation_matrix(dcm);
    desired_q.normalize();
    current_q.from_rotation_matrix(smoothing.rotation_b2e);
    current_q.normalize();
    error_q = desired_q / current_q;
    error_q.normalize();

    Vector3f angle_differential;
    error_q.to_axis_angle(angle_differential);
    smoothing.gyro = gyro + angle_differential / time_constant;

    float R, P, Y;
    smoothing.rotation_b2e.to_euler(&R, &P, &Y);
    float R2, P2, Y2;
    dcm.to_euler(&R2, &P2, &Y2);

#if 0
// @LoggerMessage: SMOO
// @Description: Smoothed sensor data fed to EKF to avoid inconsistencies
// @Field: TimeUS: Time since system startup
// @Field: AEx: Angular Velocity (around x-axis)
// @Field: AEy: Angular Velocity (around y-axis)
// @Field: AEz: Angular Velocity (around z-axis)
// @Field: DPx: Velocity (along x-axis)
// @Field: DPy: Velocity (along y-axis)
// @Field: DPz: Velocity (along z-axis)
// @Field: R: Roll
// @Field: P: Pitch
// @Field: Y: Yaw
// @Field: R2: DCM Roll
// @Field: P2: DCM Pitch
// @Field: Y2: DCM Yaw
    AP::logger().WriteStreaming("SMOO", "TimeUS,AEx,AEy,AEz,DPx,DPy,DPz,R,P,Y,R2,P2,Y2",
                                           "Qffffffffffff",
                                           AP_HAL::micros64(),
                                           degrees(angle_differential.x),
                                           degrees(angle_differential.y),
                                           degrees(angle_differential.z),
                                           delta_pos.x, delta_pos.y, delta_pos.z,
                                           degrees(R), degrees(P), degrees(Y),
                                           degrees(R2), degrees(P2), degrees(Y2));
#endif


    // integrate to get new attitude
    smoothing.rotation_b2e.rotate(smoothing.gyro * delta_time);
    smoothing.rotation_b2e.normalize();

    // integrate to get new position
    smoothing.velocity_ef += accel_e * delta_time;
    smoothing.position += (smoothing.velocity_ef * delta_time).todouble();

    smoothing.location = origin;
    smoothing.location.offset(smoothing.position.x, smoothing.position.y);
    smoothing.location.alt  = static_cast<int32_t>(home.alt - smoothing.position.z * 100.0f);

    smoothing.last_update_us = now;
}

/*
  return a filtered servo input as a value from -1 to 1
  servo is assumed to be 1000 to 2000, trim at 1500
 */
float Aircraft::filtered_servo_angle(const struct sitl_input &input, uint8_t idx)
{
    uint16_t pwm = input.servos[idx] == 0 ? 1500 : input.servos[idx];
    return servo_filter[idx].filter_angle(pwm, frame_time_us * 1.0e-6);
}

/*
  return a filtered servo input as a value from 0 to 1
  servo is assumed to be 1000 to 2000
 */
float Aircraft::filtered_servo_range(const struct sitl_input &input, uint8_t idx)
{
    return servo_filter[idx].filter_range(input.servos[idx], frame_time_us * 1.0e-6);
}

// setup filtering for servo
void Aircraft::filtered_servo_setup(uint8_t idx, uint16_t pwm_min, uint16_t pwm_max, float deflection_deg)
{
    servo_filter[idx].set_pwm_range(pwm_min, pwm_max);
    servo_filter[idx].set_deflection(deflection_deg);
}

// extrapolate sensors by a given delta time in seconds
void Aircraft::extrapolate_sensors(float delta_time)
{
    Vector3f accel_earth = dcm * accel_body;
    accel_earth.z += GRAVITY_MSS;

    dcm.rotate(gyro * delta_time);
    dcm.normalize();

    // work out acceleration as seen by the accelerometers. It sees the kinematic
    // acceleration (ie. real movement), plus gravity
    accel_body = dcm.transposed() * (accel_earth + Vector3f(0,0,-GRAVITY_MSS));

    // new velocity and position vectors
    velocity_ef += accel_earth * delta_time;
    position += (velocity_ef * delta_time).todouble();
    velocity_air_ef = velocity_ef - wind_ef;
    velocity_air_bf = dcm.transposed() * velocity_air_ef;
}

bool Aircraft::Clamp::clamped(Aircraft &aircraft, const struct sitl_input &input)
{
    const auto clamp_ch = AP::sitl()->clamp_ch;
    if (clamp_ch < 1) {
        return false;
    }
    const uint32_t clamp_idx = clamp_ch - 1;
    if (clamp_idx > ARRAY_SIZE(input.servos)) {
        return false;
    }
    const uint16_t servo_pos = input.servos[clamp_idx];
    bool new_clamped = currently_clamped;
    if (servo_pos < 1200) {
        if (currently_clamped) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL: Clamp: released vehicle");
            new_clamped = false;
        }
        grab_attempted = false;
    } else {
        // re-clamp if < 10cm from home
        if (servo_pos > 1800 && !grab_attempted) {
            const Vector3d pos = aircraft.get_position_relhome();
            const float distance_from_home = pos.length();
            // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL: Clamp: dist=%f", distance_from_home);
            if (distance_from_home < 0.5) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL: Clamp: grabbed vehicle");
                new_clamped = true;
            } else if (!grab_attempted) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "SITL: Clamp: missed vehicle");
            }
            grab_attempted = true;
        }
    }

    currently_clamped = new_clamped;

    return currently_clamped;
}

void Aircraft::update_external_payload(const struct sitl_input &input)
{
    external_payload_mass = 0;

    // update sprayer
    if (sprayer && sprayer->is_enabled()) {
        sprayer->update(input);
        external_payload_mass += sprayer->payload_mass();
    }

    {
        const float range = rangefinder_range();
        if (!isinf(range) && range > 100000) {
            AP_HAL::panic("Bad rangefinder calculation");
        }
        for (uint8_t i=0; i<ARRAY_SIZE(rangefinder_m); i++) {
            rangefinder_m[i] = range;
        }
    }

    // update i2c
    if (i2c) {
        i2c->update(*this);
    }

    // update buzzer
    if (buzzer && buzzer->is_enabled()) {
        buzzer->update(input);
    }

    // update grippers
    if (gripper && gripper->is_enabled()) {
        gripper->set_alt(hagl());
        gripper->update(input);
        external_payload_mass += gripper->payload_mass();
    }
    if (gripper_epm && gripper_epm->is_enabled()) {
        gripper_epm->update(input);
        external_payload_mass += gripper_epm->payload_mass();
    }

    // update parachute
    if (parachute && parachute->is_enabled()) {
        parachute->update(input);
        // TODO: add drag to vehicle, presumably proportional to velocity
    }

    if (precland && precland->is_enabled()) {
        precland->update(get_location());
        if (precland->_over_precland_base) {
            local_ground_level += precland->_device_height;
        }
    }

    // update RichenPower generator
    if (richenpower) {
        richenpower->update(input);
    }

#if AP_SIM_LOWEHEISER_ENABLED
    // update Loweheiser generator
    if (loweheiser) {
        loweheiser->update();
    }
#endif

    if (fetteconewireesc) {
        fetteconewireesc->update(*this);
    }

#if AP_SIM_VOLZ_ENABLED
    if (volz) {
        volz->update(*this);
    }
#endif  // AP_SIM_VOLZ_ENABLED

#if AP_SIM_SHIP_ENABLED
    sitl->models.shipsim.update();
#endif

    // update IntelligentEnergy 2.4kW generator
    if (ie24) {
        ie24->update(input);
    }

#if AP_TEST_DRONECAN_DRIVERS
    if (dronecan) {
        dronecan->update();
    }
#endif

#if AP_SIM_GPIO_LED_1_ENABLED
    sim_led1.update(*this);
#endif
#if AP_SIM_GPIO_LED_2_ENABLED
    sim_led2.update(*this);
#endif
#if AP_SIM_GPIO_LED_3_ENABLED
    sim_led3.update(*this);
#endif
#if AP_SIM_GPIO_LED_RGB_ENABLED
    sim_ledrgb.update(*this);
#endif
}

void Aircraft::add_shove_forces(Vector3f &rot_accel, Vector3f &body_accel)
{
    const uint32_t now = AP_HAL::millis();
    if (sitl == nullptr) {
        return;
    }
    if (sitl->shove.t == 0) {
        return;
    }
    if (sitl->shove.start_ms == 0) {
        sitl->shove.start_ms = now;
    }
    if (now - sitl->shove.start_ms < uint32_t(sitl->shove.t)) {
        // FIXME: can we get a vector operation here instead?
        body_accel.x += sitl->shove.x;
        body_accel.y += sitl->shove.y;
        body_accel.z += sitl->shove.z;
    } else {
        sitl->shove.start_ms = 0;
        sitl->shove.t.set(0);
    }
}

float Aircraft::get_local_updraft(const Vector3d &currentPos)
{
    int scenario = sitl->thermal_scenario;

    #define MAX_THERMALS 10

    float thermals_w[MAX_THERMALS];
    float thermals_r[MAX_THERMALS];
    float thermals_x[MAX_THERMALS];
    float thermals_y[MAX_THERMALS];

    int n_thermals = 0;

    switch (scenario) {
        case 0:
            return 0;
        case 1:
            n_thermals = 1;
            thermals_w[0] =  2.0;
            thermals_r[0] =  80.0;
            thermals_x[0] = -180.0;
            thermals_y[0] = -260.0;
            break;
        case 2:
            n_thermals = 1;
            thermals_w[0] =  4.0;
            thermals_r[0] =  30.0;
            thermals_x[0] = -180.0;
            thermals_y[0] = -260.0;
            break;
        case 3:
            n_thermals = 1;
            thermals_w[0] =  2.0;
            thermals_r[0] =  30.0;
            thermals_x[0] = -180.0;
            thermals_y[0] = -260.0;
            break;
        case 4:
            n_thermals = 1;
            thermals_w[0] =  5.0;
            thermals_r[0] =  30.0;
            thermals_x[0] =  0;
            thermals_y[0] =  0;
            break;
        default:
            AP_BoardConfig::config_error("Bad thermal scenario");
    }

    // Wind drift at this altitude
    float driftX = sitl->wind_speed * (currentPos.z+100) * cosf(sitl->wind_direction * DEG_TO_RAD);
    float driftY = sitl->wind_speed * (currentPos.z+100) * sinf(sitl->wind_direction * DEG_TO_RAD);

    int iThermal;
    float w = 0.0f;
    float r2;
    for (iThermal=0;iThermal<n_thermals;iThermal++) {
        Vector3d thermalPos(thermals_x[iThermal] + driftX/thermals_w[iThermal],
                            thermals_y[iThermal] + driftY/thermals_w[iThermal],
                            0);
        Vector3d relVec = currentPos - thermalPos;
        r2 = relVec.x*relVec.x + relVec.y*relVec.y;
        w += thermals_w[iThermal]*exp(-r2/(thermals_r[iThermal]*thermals_r[iThermal]));
    }

    return w;
}

void Aircraft::add_twist_forces(Vector3f &rot_accel)
{
    if (sitl == nullptr) {
        return;
    }
    if (sitl->gnd_behav != -1) {
        ground_behavior = (GroundBehaviour)sitl->gnd_behav.get();
    }
    const uint32_t now = AP_HAL::millis();
    if (sitl == nullptr) {
        return;
    }
    if (sitl->twist.t == 0) {
        return;
    }
    if (sitl->twist.start_ms == 0) {
        sitl->twist.start_ms = now;
    }
    if (now - sitl->twist.start_ms < uint32_t(sitl->twist.t)) {
        // FIXME: can we get a vector operation here instead?
        rot_accel.x += sitl->twist.x;
        rot_accel.y += sitl->twist.y;
        rot_accel.z += sitl->twist.z;
    } else {
        sitl->twist.start_ms = 0;
        sitl->twist.t.set(0);
    }
}

// add body-frame force due to slung payload and tether
void Aircraft::add_external_forces(Vector3f &body_accel)
{
    Vector3f total_force;
#if AP_SIM_SLUNGPAYLOAD_ENABLED
    Vector3f forces_ef_slung;
    sitl->models.slung_payload_sim.get_forces_on_vehicle(forces_ef_slung);
    total_force += forces_ef_slung;
#endif

#if AP_SIM_TETHER_ENABLED
    Vector3f forces_ef_tether;
    sitl->models.tether_sim.get_forces_on_vehicle(forces_ef_tether);
    total_force += forces_ef_tether;
#endif

    // convert ef forces to body-frame accelerations (acceleration = force / mass)
    const Vector3f accel_bf_tether = dcm.transposed() * total_force / mass;
    body_accel += accel_bf_tether;
}

/*
  get position relative to home
 */
Vector3d Aircraft::get_position_relhome() const
{
    Vector3d pos = position;
    pos.xy() += home.get_distance_NE_double(origin);
    return pos;
}

// get air density in kg/m^3
float Aircraft::get_air_density(float alt_amsl) const
{
    return AP_Baro::get_air_density_for_alt_amsl(alt_amsl);
}

/*
  update EAS airspeed and pitot speed
 */
void Aircraft::update_eas_airspeed()
{
    airspeed = velocity_air_ef.length() / eas2tas;

    /*
      airspeed as seen by a fwd pitot tube (limited to 120m/s)
    */
    airspeed_pitot = airspeed;

    // calculate angle between the local flow vector and a pitot tube aligned with the X body axis
    const float pitot_aoa =  atan2f(sqrtf(sq(velocity_air_bf.y) + sq(velocity_air_bf.z)), velocity_air_bf.x);

    /*
      assume the pitot can correctly capture airspeed up to 20 degrees off the nose
      and follows a cose law outside that range
    */
    const float max_pitot_aoa = radians(20);
    if (pitot_aoa > radians(90)) {
        airspeed_pitot = 0;
    } else if (pitot_aoa > max_pitot_aoa) {
        const float gain_factor = M_PI_2 / (radians(90) - max_pitot_aoa);
        airspeed_pitot *= cosf((pitot_aoa - max_pitot_aoa) * gain_factor);
    }
}

/*
  set pose on the aircraft, called from scripting
 */
bool Aircraft::set_pose(uint8_t instance, const Location &loc, const Quaternion &quat, const Vector3f &velocity_ef, const Vector3f &gyro_rads)
{
    if (instance >= MAX_SIM_INSTANCES || instances[instance] == nullptr) {
        return false;
    }
    auto &aircraft = *instances[instance];
    WITH_SEMAPHORE(aircraft.pose_sem);

    quat.rotation_matrix(aircraft.dcm);
    aircraft.velocity_ef = velocity_ef;
    aircraft.location = loc;
    aircraft.position = aircraft.home.get_distance_NED_double(loc);
    aircraft.smoothing.position = aircraft.position;
    aircraft.smoothing.rotation_b2e = aircraft.dcm;
    aircraft.smoothing.velocity_ef = velocity_ef;
    aircraft.smoothing.location = loc;
    aircraft.gyro = gyro_rads;

    return true;
}

/*
  wrapper for scripting access
 */
bool SITL::SIM::set_pose(uint8_t instance, const Location &loc, const Quaternion &quat, const Vector3f &velocity_ef, const Vector3f &gyro_rads)
{
    return Aircraft::set_pose(instance, loc, quat, velocity_ef, gyro_rads);
}

