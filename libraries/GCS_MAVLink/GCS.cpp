#include "GCS.h"

#include <AC_Fence/AC_Fence.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Arming/AP_Arming.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <AP_Vehicle/AP_Vehicle.h>

extern const AP_HAL::HAL& hal;

// if this assert fails then fix it and the comment in GCS.h where
// _statustext_queue is declared
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
assert_storage_size<GCS::statustext_t, 58> _assert_statustext_t_size;
#endif

void GCS::get_sensor_status_flags(uint32_t &present,
                                  uint32_t &enabled,
                                  uint32_t &health,
                                  uint32_t &present_extension,
                                  uint32_t &enabled_extension,
                                  uint32_t &health_extension)
{
    update_sensor_status_flags();

    present = control_sensors_present;
    enabled = control_sensors_enabled;
    health = control_sensors_health;
    present_extension = control_sensors_extension_present;
    enabled_extension = control_sensors_extension_enabled;
    health_extension = control_sensors_extension_health;
}

MissionItemProtocol_Waypoints *GCS::_missionitemprotocol_waypoints;
MissionItemProtocol_Rally *GCS::_missionitemprotocol_rally;
MissionItemProtocol_Fence *GCS::_missionitemprotocol_fence;
MissionItemProtocol_Fallback *GCS::_missionitemprotocol_fallback;

const MAV_MISSION_TYPE GCS_MAVLINK::supported_mission_types[] = {
    MAV_MISSION_TYPE_MISSION,
    MAV_MISSION_TYPE_RALLY,
    MAV_MISSION_TYPE_FALLBACK,
    MAV_MISSION_TYPE_FENCE,
};

void GCS::init()
{
    mavlink_system.sysid = sysid_this_mav();
}

/*
 * returns a mask of channels that statustexts should be sent to
 */
uint8_t GCS::statustext_send_channel_mask() const
{
    uint8_t ret = 0;
    ret |= GCS_MAVLINK::active_channel_mask();
    ret |= GCS_MAVLINK::streaming_channel_mask();
    ret &= ~GCS_MAVLINK::private_channel_mask();
    return ret;
}

/*
  send a text message to all GCS
 */
void GCS::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list)
{
    uint8_t mask = statustext_send_channel_mask();
    if (!update_send_has_been_called) {
        // we have not yet initialised the streaming-channel-mask,
        // which is done as part of the update() call.  So just send
        // it to all channels:
        mask = (1<<_num_gcs)-1;
    }
    send_textv(severity, fmt, arg_list, mask);
}

void GCS::send_text(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    send_textv(severity, fmt, arg_list);
    va_end(arg_list);
}

void GCS::send_to_active_channels(uint32_t msgid, const char *pkt)
{
    const mavlink_msg_entry_t *entry = mavlink_get_msg_entry(msgid);
    if (entry == nullptr) {
        return;
    }
    for (uint8_t i=0; i<num_gcs(); i++) {
        GCS_MAVLINK &c = *chan(i);
        if (c.is_private()) {
            continue;
        }
        if (!c.is_active()) {
            continue;
        }
        if (entry->max_msg_len + c.packet_overhead() > c.txspace()) {
            // no room on this channel
            continue;
        }
        c.send_message(pkt, entry);
    }
}

void GCS::send_named_float(const char *name, float value) const
{

    mavlink_named_value_float_t packet {};
    packet.time_boot_ms = AP_HAL::millis();
    packet.value = value;
    memcpy(packet.name, name, MIN(strlen(name), (uint8_t)MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN));

    gcs().send_to_active_channels(MAVLINK_MSG_ID_NAMED_VALUE_FLOAT,
                                  (const char *)&packet);
}

/*
  install an alternative protocol handler. This allows another
  protocol to take over the link if MAVLink goes idle. It is used to
  allow for the AP_BLHeli pass-thru protocols to run on hal.serial(0)
 */
bool GCS::install_alternative_protocol(mavlink_channel_t c, GCS_MAVLINK::protocol_handler_fn_t handler)
{
    if (c >= num_gcs()) {
        return false;
    }
    if (chan(c)->alternative.handler && handler) {
        // already have one installed - we may need to add support for
        // multiple alternative handlers
        return false;
    }
    chan(c)->alternative.handler = handler;
    return true;
}

bool GCS::is_escs_healthy()
{
#if (!HAL_WITH_ESC_TELEM) || (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
    if (AP::vehicle()->get_motor_failure()) {
        return false;
    }
    return true;
#else
    AP_Motors *motors = AP_Motors::get_singleton();
    AP_ESC_Telem& telem = AP::esc_telem();

    if (!motors) {
        return false;
    }

    if (AP::vehicle()->get_motor_failure()) {
        return false;
    }

    static bool wrong_motors_num_detected = false;
    
    int8_t num_expected_motors = telem.get_num_expected_motors();

    if (num_expected_motors < 0) {
        return false;
    }

    uint8_t num_active_escs = telem.get_num_active_escs();

    if (num_active_escs != num_expected_motors) {
        if (!wrong_motors_num_detected) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "Wrong number of UAVCAN ESCs detected (%u/%u)",
                                                num_active_escs, num_expected_motors);
            if (motors->armed() && (!AP_Notify::flags.in_arming_delay)) {
                AP::motors()->set_thrust_boost(true);
                AP::vehicle()->set_motor_failure();
                return false;
            }
        }
        wrong_motors_num_detected = true;
        return false;
    }
    wrong_motors_num_detected = false;
#endif
    return true;
}

void GCS::update_sensor_status_flags()
{
    control_sensors_present = 0;
    control_sensors_enabled = 0;
    control_sensors_health = 0;
    control_sensors_extension_present = 0;
    control_sensors_extension_enabled = 0;
    control_sensors_extension_health = 0;

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_AHRS)
    AP_AHRS &ahrs = AP::ahrs();
    const AP_InertialSensor &ins = AP::ins();

    control_sensors_present |= MAV_SYS_STATUS_AHRS;
    if (ahrs.initialised()) {
        control_sensors_enabled |= MAV_SYS_STATUS_AHRS;
        if (ahrs.healthy()) {
            if (!ahrs.have_inertial_nav() || ins.accel_calibrated_ok_all()) {
                control_sensors_health |= MAV_SYS_STATUS_AHRS;
            }
        }
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_MAG)
    const Compass &compass = AP::compass();
    if (compass.available()) {
        if (compass.get_count() > 0) {
            control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG;
            if (compass.use_for_yaw(0) && compass.is_configured(0)) {
                control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_MAG;
                if (compass.compass_checks(0)) {
                    control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG;
                }
            }
        }
        if (compass.get_count() > 1) {
            control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_MAG2;
            if (compass.use_for_yaw(1)&& compass.is_configured(1)) {
                control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_MAG2;
                if(compass.compass_checks(1)) {
                    control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_MAG2;
                }
            }
        }
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_BARO)
    const AP_Baro &barometer = AP::baro();
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if (barometer.all_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_GPS)
    const AP_GPS &gps = AP::gps();
    if ((gps.num_sensors() > 0) && (gps.status(0) > AP_GPS::NO_GPS)) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    }
    if ((gps.num_sensors() > 0) && (gps.is_healthy(0)) && (gps.status(0) >= min_status_for_gps_healthy())) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
#endif

    if ((gps.num_sensors() > 1) && (gps.status(1) > AP_GPS::NO_GPS)) {
        control_sensors_extension_present |= MAV_SYS_STATUS_SENSOR_GPS2;
        control_sensors_extension_enabled |= MAV_SYS_STATUS_SENSOR_GPS2;
    }
    if ((gps.num_sensors() > 1) && (gps.is_healthy(1)) && (gps.status(1) >= min_status_for_gps_healthy())) {
        control_sensors_extension_health |= MAV_SYS_STATUS_SENSOR_GPS2;
    }

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_BATTERY)
    const AP_BattMonitor &battery = AP::battery();
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
    if (battery.num_instances() > 0) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
    if (battery.healthy() && !battery.has_failsafed()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_BATTERY;
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_AHRS)
    if (ins.get_gyro_count() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
        if ((!ins.calibrating()) && ins.use_gyro(0)) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
            if (ins.get_gyro_health(0) && ins.gyro_calibrated_ok(0)) {
                control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
            }
        }
    }
    if (ins.get_accel_count() > 0) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
        if ((!ins.calibrating()) && ins.use_accel(0)) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
            if (ins.get_accel_health(0)) {
                control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
            }
        }
    }
    if (ins.get_gyro_count() > 1) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_GYRO2;
        if ((!ins.calibrating()) && ins.use_gyro(1)) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_GYRO2;
            if (ins.get_gyro_health(1) && ins.gyro_calibrated_ok(1)) {
                control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_GYRO2;
            }
        }
    }
    if (ins.get_accel_count() > 1) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
        if ((!ins.calibrating()) && ins.use_accel(1)) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
            if (ins.get_accel_health(1)) {
                control_sensors_health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL2;
            }
        }
    }
    if (ins.get_gyro_count() > 2) {
        control_sensors_extension_present |= MAV_SYS_STATUS_SENSOR_3D_GYRO3;
        if ((!ins.calibrating()) && ins.use_gyro(2)) {
            control_sensors_extension_enabled |= MAV_SYS_STATUS_SENSOR_3D_GYRO3;
            if (ins.get_gyro_health(2) && ins.gyro_calibrated_ok(2)) {
                control_sensors_extension_health |= MAV_SYS_STATUS_SENSOR_3D_GYRO3;
            }
        }
    }
    if (ins.get_accel_count() > 2) {
        control_sensors_extension_present |= MAV_SYS_STATUS_SENSOR_3D_ACCEL3;
        if ((!ins.calibrating()) && ins.use_accel(2)) {
            control_sensors_extension_enabled |= MAV_SYS_STATUS_SENSOR_3D_ACCEL3;
            if (ins.get_accel_health(2)) {
                control_sensors_extension_health |= MAV_SYS_STATUS_SENSOR_3D_ACCEL3;
            }
        }
    }
#endif

    control_sensors_extension_present |= MAV_SYS_STATUS_VIBRATIONS;
    control_sensors_extension_enabled |= MAV_SYS_STATUS_VIBRATIONS;
    if (!ins.is_high_vibrations()) {
        control_sensors_extension_health |= MAV_SYS_STATUS_VIBRATIONS;
    }

#if HAL_LOGGING_ENABLED
    const AP_Logger &logger = AP::logger();
    bool logging_present = logger.logging_present();
    bool logging_enabled = logger.logging_enabled();
    bool logging_healthy = !logger.logging_failed();
#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_GPS)
    // some GPS units do logging, so they have to be healthy too:
    logging_present |= gps.logging_present();
    logging_enabled |= gps.logging_enabled();
    logging_healthy &= !gps.logging_failed();
#endif
    if (logging_present) {
        control_sensors_present |= MAV_SYS_STATUS_LOGGING;
    }
    if (logging_enabled) {
        control_sensors_enabled |= MAV_SYS_STATUS_LOGGING;
    }
    if (logging_healthy) {
        control_sensors_health |= MAV_SYS_STATUS_LOGGING;
    }
#endif  // HAL_LOGGING_ENABLED

    // set motors outputs as enabled if safety switch is not disarmed (i.e. either NONE or ARMED)
#if !defined(HAL_BUILD_AP_PERIPH)
    control_sensors_present |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED) {
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }
    if (is_escs_healthy()) {
        control_sensors_health |= MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
    }
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (ahrs.get_ekf_type() == 10) {
        // always show EKF type 10 as healthy. This prevents spurious error
        // messages in xplane and other simulators that use EKF type 10
        control_sensors_health |= MAV_SYS_STATUS_AHRS | MAV_SYS_STATUS_SENSOR_GPS | MAV_SYS_STATUS_SENSOR_3D_ACCEL | MAV_SYS_STATUS_SENSOR_3D_GYRO;
    }
#endif

#if !defined(HAL_BUILD_AP_PERIPH)
    const AC_Fence *fence = AP::fence();
    if (fence != nullptr) {
        if (fence->sys_status_enabled()) {
            control_sensors_enabled |= MAV_SYS_STATUS_GEOFENCE;
        }
        if (fence->sys_status_present()) {
            control_sensors_present |= MAV_SYS_STATUS_GEOFENCE;
        }
        if (!fence->sys_status_failed()) {
            control_sensors_health |= MAV_SYS_STATUS_GEOFENCE;
        }
    }
#endif

    // airspeed
#if AP_AIRSPEED_ENABLED
    const AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed && airspeed->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        const bool use = airspeed->use();
        const bool enabled = AP::ahrs().airspeed_sensor_enabled();
        if (use) {
            control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        }
        if (airspeed->all_healthy() && (!use || enabled)) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE;
        }
    }
#endif

#if HAL_VISUALODOM_ENABLED
    const AP_VisualOdom *visual_odom = AP::visualodom();
    if (visual_odom && visual_odom->enabled()) {
        control_sensors_present |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        control_sensors_enabled |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        if (visual_odom->healthy()) {
            control_sensors_health |= MAV_SYS_STATUS_SENSOR_VISION_POSITION;
        }
    }
#endif

    // give GCS status of prearm checks. This is enabled if any arming checks are enabled.
    // it is healthy if armed or checks are passing
#if !defined(HAL_BUILD_AP_PERIPH)
    control_sensors_present |= MAV_SYS_STATUS_PREARM_CHECK;
    if (AP::arming().get_enabled_checks()) {
        control_sensors_enabled |= MAV_SYS_STATUS_PREARM_CHECK;
        if (hal.util->get_soft_armed() || AP_Notify::flags.pre_arm_check) {
            control_sensors_health |= MAV_SYS_STATUS_PREARM_CHECK;
        }
    }
#endif

    update_vehicle_sensor_status_flags();
}

bool GCS::out_of_time() const
{
#if defined(HAL_BUILD_AP_PERIPH)
    // we are never out of time for AP_Periph
    // as we don't have concept of AP_Scheduler in AP_Periph
    return false;
#endif
    // while we are in the delay callback we are never out of time:
    if (hal.scheduler->in_delay_callback()) {
        return false;
    }

    // we always want to be able to send messages out while in the error loop:
    if (AP_BoardConfig::in_config_error()) {
        return false;
    }

    if (min_loop_time_remaining_for_message_send_us() <= AP::scheduler().time_available_usec()) {
        return false;
    }

    return true;
}

void gcs_out_of_space_to_send_count(mavlink_channel_t chan)
{
    gcs().chan(chan)->out_of_space_to_send();
}
