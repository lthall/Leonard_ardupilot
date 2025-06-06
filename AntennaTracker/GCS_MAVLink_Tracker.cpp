#include "GCS_MAVLink_Tracker.h"
#include "Tracker.h"

MAV_TYPE GCS_Tracker::frame_type() const
{
    return MAV_TYPE_ANTENNA_TRACKER;
}

uint8_t GCS_MAVLINK_Tracker::base_mode() const
{
    uint8_t _base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    // work out the base_mode. This value is not very useful
    // for APM, but we calculate it as best we can so a generic
    // MAVLink enabled ground station can work out something about
    // what the MAV is up to. The actual bit values are highly
    // ambiguous for most of the APM flight modes. In practice, you
    // only get useful information from the custom_mode, which maps to
    // the APM flight mode and has a well defined meaning in the
    // ArduPlane documentation
    switch (tracker.mode->number()) {
    case Mode::Number::MANUAL:
        _base_mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        break;

    case Mode::Number::STOP:
        break;

    case Mode::Number::SCAN:
    case Mode::Number::SERVOTEST:
    case Mode::Number::AUTO:
    case Mode::Number::GUIDED:
        _base_mode |= MAV_MODE_FLAG_GUIDED_ENABLED |
            MAV_MODE_FLAG_STABILIZE_ENABLED;
        // note that MAV_MODE_FLAG_AUTO_ENABLED does not match what
        // APM does in any mode, as that is defined as "system finds its own goal
        // positions", which APM does not currently do
        break;

    case Mode::Number::INITIALISING:
        break;
    }

    // we are armed if safety switch is not disarmed
    if (hal.util->safety_switch_state() != AP_HAL::Util::SAFETY_DISARMED &&
        tracker.mode != &tracker.mode_initialising &&
        hal.util->get_soft_armed()) {
        _base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    return _base_mode;
}

uint32_t GCS_Tracker::custom_mode() const
{
    return (uint32_t)tracker.mode->number();
}

MAV_STATE GCS_MAVLINK_Tracker::vehicle_system_status() const
{
    if (tracker.mode == &tracker.mode_initialising) {
        return MAV_STATE_CALIBRATING;
    }
    return MAV_STATE_ACTIVE;
}

void GCS_MAVLINK_Tracker::send_nav_controller_output() const
{
	float alt_diff = (tracker.g.alt_source == ALT_SOURCE_BARO) ? tracker.nav_status.alt_difference_baro : tracker.nav_status.alt_difference_gps;

    mavlink_msg_nav_controller_output_send(
        chan,
        0,
        tracker.nav_status.pitch,
        tracker.nav_status.bearing,
        tracker.nav_status.bearing,
        MIN(tracker.nav_status.distance, UINT16_MAX),
        alt_diff,
        0,
        0);
}

void GCS_MAVLINK_Tracker::handle_set_attitude_target(const mavlink_message_t &msg)
{
    // decode packet
    mavlink_set_attitude_target_t packet;
    mavlink_msg_set_attitude_target_decode(&msg, &packet);

    // exit if vehicle is not in Guided mode
    if (tracker.mode != &tracker.mode_guided) {
        return;
    }

    // sanity checks:
    if (!is_zero(packet.body_roll_rate)) {
        return;
    }
    if (!(packet.type_mask & (1<<0))) {
        // not told to ignore body roll rate
        return;
    }
    if (!(packet.type_mask & (1<<6))) {
        // not told to ignore throttle
        return;
    }
    if (packet.type_mask & (1<<7)) {
        // told to ignore attitude (we don't allow continuous motion yet)
        return;
    }
    if ((packet.type_mask & (1<<3)) && (packet.type_mask&(1<<4))) {
        // told to ignore both pitch and yaw rates - nothing to do?!
        return;
    }

    const bool use_yaw_rate = !(packet.type_mask & (1<<2));

    tracker.mode_guided.set_angle(
        Quaternion(packet.q[0],packet.q[1],packet.q[2],packet.q[3]),
        use_yaw_rate,
        packet.body_yaw_rate);
}

/*
  send PID tuning message
 */
void GCS_MAVLINK_Tracker::send_pid_tuning()
{
    const Parameters &g = tracker.g;

    // Pitch PID
    if (g.gcs_pid_mask & 1) {
        const AP_PIDInfo *pid_info = &g.pidPitch2Srv.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_PITCH,
                                    pid_info->target,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }

    // Yaw PID
    if (g.gcs_pid_mask & 2) {
        const AP_PIDInfo *pid_info = &g.pidYaw2Srv.get_pid_info();
        mavlink_msg_pid_tuning_send(chan, PID_TUNING_YAW,
                                    pid_info->target,
                                    pid_info->actual,
                                    pid_info->FF,
                                    pid_info->P,
                                    pid_info->I,
                                    pid_info->D,
                                    pid_info->slew_rate,
                                    pid_info->Dmod);
        if (!HAVE_PAYLOAD_SPACE(chan, PID_TUNING)) {
            return;
        }
    }
}

/*
  We eavesdrop on MAVLINK_MSG_ID_GLOBAL_POSITION_INT and
  MAVLINK_MSG_ID_SCALED_PRESSUREs
*/
void GCS_MAVLINK_Tracker::packetReceived(const mavlink_status_t &status,
                                         const mavlink_message_t &msg)
{
    // return immediately if sysid doesn't match our target sysid
    if ((tracker.g.sysid_target != 0) && (tracker.g.sysid_target != msg.sysid)) {
        GCS_MAVLINK::packetReceived(status, msg);
        return;
    }

    switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
        mavlink_check_target(msg);
        break;
    }

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
    {
        // decode
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);
        tracker.tracking_update_position(packet);
        break;
    }
    
    case MAVLINK_MSG_ID_SCALED_PRESSURE:
    {
        // decode
        mavlink_scaled_pressure_t packet;
        mavlink_msg_scaled_pressure_decode(&msg, &packet);
        tracker.tracking_update_pressure(packet);
        break;
    }
    }
    GCS_MAVLINK::packetReceived(status, msg);
}

bool GCS_MAVLINK_Tracker::try_send_message(enum ap_message id)
{
    switch(id) {
    case MSG_WIND: // other vehicles do something custom with wind:
        return true;
    default:
        return GCS_MAVLINK::try_send_message(id);
    }
    return true;
}

// locks onto a particular target sysid and sets it's position data stream to at least 1hz
void GCS_MAVLINK_Tracker::mavlink_check_target(const mavlink_message_t &msg)
{
    // exit immediately if the target has already been set
    if (tracker.target_set) {
        return;
    }

    // decode
    mavlink_heartbeat_t packet;
    mavlink_msg_heartbeat_decode(&msg, &packet);

    // exit immediately if this is not a vehicle we would track
    if ((packet.type == MAV_TYPE_ANTENNA_TRACKER) ||
        (packet.type == MAV_TYPE_GCS) ||
        (packet.type == MAV_TYPE_ONBOARD_CONTROLLER) ||
        (packet.type == MAV_TYPE_GIMBAL)) {
        return;
    }

    // set our sysid to the target, this ensures we lock onto a single vehicle
    if (tracker.g.sysid_target == 0) {
        tracker.g.sysid_target.set(msg.sysid);
    }

    // send data stream request to target on all channels
    //  Note: this doesn't check success for all sends meaning it's not guaranteed the vehicle's positions will be sent at 1hz
    tracker.gcs().request_datastream_position(msg.sysid, msg.compid);
    tracker.gcs().request_datastream_airpressure(msg.sysid, msg.compid);

    // flag target has been set
    tracker.target_set = true;
}

MAV_RESULT GCS_MAVLINK_Tracker::_handle_command_preflight_calibration_baro(const mavlink_message_t &msg)
{
    MAV_RESULT ret = GCS_MAVLINK::_handle_command_preflight_calibration_baro(msg);
    if (ret == MAV_RESULT_ACCEPTED) {
        // zero the altitude difference on next baro update
        tracker.nav_status.need_altitude_calibration = true;
    }
    return ret;
}

MAV_RESULT GCS_MAVLINK_Tracker::handle_command_component_arm_disarm(const mavlink_command_int_t &packet)
{
    if (is_equal(packet.param1,1.0f)) {
        tracker.arm_servos();
        return MAV_RESULT_ACCEPTED;
    }
    if (is_zero(packet.param1))  {
        tracker.disarm_servos();
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_UNSUPPORTED;
}

MAV_RESULT GCS_MAVLINK_Tracker::handle_command_int_packet(const mavlink_command_int_t &packet, const mavlink_message_t &msg)
{
    switch(packet.command) {

    case MAV_CMD_DO_SET_SERVO:
        // ensure we are in servo test mode
        tracker.set_mode(tracker.mode_servotest, ModeReason::SERVOTEST);

        if (!tracker.mode_servotest.set_servo(packet.param1, packet.param2)) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;

        // mavproxy/mavutil sends this when auto command is entered 
    case MAV_CMD_MISSION_START:
        tracker.set_mode(tracker.mode_auto, ModeReason::GCS_COMMAND);
        return MAV_RESULT_ACCEPTED;

    default:
        return GCS_MAVLINK::handle_command_int_packet(packet, msg);
    }
}

void GCS_MAVLINK_Tracker::handle_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {

    case MAVLINK_MSG_ID_SET_ATTITUDE_TARGET:
        handle_set_attitude_target(msg);
        break;

#if AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED
    // When mavproxy 'wp sethome' 
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
        handle_message_mission_write_partial_list(msg);
        break;

    // XXX receive a WP from GCS and store in EEPROM if it is HOME
    case MAVLINK_MSG_ID_MISSION_ITEM:
        handle_message_mission_item(msg);
        break;
#endif

    case MAVLINK_MSG_ID_MANUAL_CONTROL:
        handle_message_manual_control(msg);
        break;

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        handle_message_global_position_int(msg);
        break;

    case MAVLINK_MSG_ID_SCALED_PRESSURE:
        handle_message_scaled_pressure(msg);
        break;
    }

    GCS_MAVLINK::handle_message(msg);
}


#if AP_TRACKER_SET_HOME_VIA_MISSION_UPLOAD_ENABLED
void GCS_MAVLINK_Tracker::handle_message_mission_write_partial_list(const mavlink_message_t &msg)
{
        // decode
        mavlink_mission_write_partial_list_t packet;
        mavlink_msg_mission_write_partial_list_decode(&msg, &packet);
        if (packet.start_index == 0)
        {
            // New home at wp index 0. Ask for it
            waypoint_receiving = true;
            send_message(MSG_NEXT_MISSION_REQUEST_WAYPOINTS);
        }
}

void GCS_MAVLINK_Tracker::handle_message_mission_item(const mavlink_message_t &msg)
{
        mavlink_mission_item_t packet;
        MAV_MISSION_RESULT result = MAV_MISSION_ACCEPTED;

        mavlink_msg_mission_item_decode(&msg, &packet);

        Location tell_command;

        switch (packet.frame)
        {
        case MAV_FRAME_MISSION:
        case MAV_FRAME_GLOBAL:
        {
            tell_command = Location{
                int32_t(1.0e7f*packet.x), // in as DD converted to * t7
                int32_t(1.0e7f*packet.y), // in as DD converted to * t7
                int32_t(packet.z*1.0e2f), // in as m converted to cm
                Location::AltFrame::ABSOLUTE
            };
            break;
        }

#ifdef MAV_FRAME_LOCAL_NED
        case MAV_FRAME_LOCAL_NED:                         // local (relative to home position)
        {
            tell_command = Location{
                int32_t(1.0e7f*degrees(packet.x/(RADIUS_OF_EARTH*cosf(radians(home.lat/1.0e7f)))) + home.lat),
                int32_t(1.0e7f*degrees(packet.y/RADIUS_OF_EARTH) + home.lng),
                int32_t(-packet.z*1.0e2f),
                Location::AltFrame::ABOVE_HOME
            };
            break;
        }
#endif

#ifdef MAV_FRAME_LOCAL
        case MAV_FRAME_LOCAL:                         // local (relative to home position)
        {
            tell_command = {
                int32_t(1.0e7f*degrees(packet.x/(RADIUS_OF_EARTH*cosf(radians(home.lat/1.0e7f)))) + home.lat),
                int32_t(1.0e7f*degrees(packet.y/RADIUS_OF_EARTH) + home.lng),
                int32_t(packet.z*1.0e2f),
                Location::AltFrame::ABOVE_HOME
            };
            break;
        }
#endif

        case MAV_FRAME_GLOBAL_RELATIVE_ALT:                         // absolute lat/lng, relative altitude
        {
            tell_command = {
                int32_t(1.0e7f * packet.x), // in as DD converted to * t7
                int32_t(1.0e7f * packet.y), // in as DD converted to * t7
                int32_t(packet.z * 1.0e2f),
                Location::AltFrame::ABOVE_HOME
            };
            break;
        }

        default:
            result = MAV_MISSION_UNSUPPORTED_FRAME;
            break;
        }

        if (result != MAV_MISSION_ACCEPTED) goto mission_failed;

        // Check if receiving waypoints (mission upload expected)
        if (!waypoint_receiving) {
            result = MAV_MISSION_ERROR;
            goto mission_failed;
        }

        // check if this is the HOME wp
        if (packet.seq == 0) {
            if (!tracker.set_home(tell_command, false)) {
                result = MAV_MISSION_ERROR;
                goto mission_failed;
            }
            send_text(MAV_SEVERITY_INFO,"New HOME received");
            waypoint_receiving = false;
        }

mission_failed:
        // send ACK (including in success case)
        mavlink_msg_mission_ack_send(
            chan,
            msg.sysid,
            msg.compid,
            result,
            MAV_MISSION_TYPE_MISSION);
}
#endif

void GCS_MAVLINK_Tracker::handle_message_manual_control(const mavlink_message_t &msg)
{
        mavlink_manual_control_t packet;
        mavlink_msg_manual_control_decode(&msg, &packet);
        tracker.tracking_manual_control(packet);
}

void GCS_MAVLINK_Tracker::handle_message_global_position_int(const mavlink_message_t &msg)
{
        // decode
        mavlink_global_position_int_t packet;
        mavlink_msg_global_position_int_decode(&msg, &packet);
        tracker.tracking_update_position(packet);
}

void GCS_MAVLINK_Tracker::handle_message_scaled_pressure(const mavlink_message_t &msg)
{
        mavlink_scaled_pressure_t packet;
        mavlink_msg_scaled_pressure_decode(&msg, &packet);
        tracker.tracking_update_pressure(packet);
}

// send position tracker is using
void GCS_MAVLINK_Tracker::send_global_position_int()
{
    if (!tracker.stationary) {
        GCS_MAVLINK::send_global_position_int();
        return;
    }

    mavlink_msg_global_position_int_send(
        chan,
        AP_HAL::millis(),
        tracker.current_loc.lat,  // in 1E7 degrees
        tracker.current_loc.lng,  // in 1E7 degrees
        tracker.current_loc.alt,  // millimeters above ground/sea level
        0,                        // millimeters above home
        0,                        // X speed cm/s (+ve North)
        0,                        // Y speed cm/s (+ve East)
        0,                        // Z speed cm/s (+ve Down)
        tracker.ahrs.yaw_sensor); // compass heading in 1/100 degree
}

// Send the mode with the given index (not mode number!) return the total number of modes
// Index starts at 1
uint8_t GCS_MAVLINK_Tracker::send_available_mode(uint8_t index) const
{
    const Mode* modes[] {
        &tracker.mode_manual,
        &tracker.mode_stop,
        &tracker.mode_scan,
        &tracker.mode_guided,
        &tracker.mode_servotest,
        &tracker.mode_auto,
        &tracker.mode_initialising,
    };

    const uint8_t mode_count = ARRAY_SIZE(modes);

    // Convert to zero indexed
    const uint8_t index_zero = index - 1;
    if (index_zero >= mode_count) {
        // Mode does not exist!?
        return mode_count;
    }

    // Ask the mode for its name and number
    const char* name = modes[index_zero]->name();
    const uint8_t mode_number = (uint8_t)modes[index_zero]->number();

    mavlink_msg_available_modes_send(
        chan,
        mode_count,
        index,
        MAV_STANDARD_MODE::MAV_STANDARD_MODE_NON_STANDARD,
        mode_number,
        0, // MAV_MODE_PROPERTY bitmask
        name
    );

    return mode_count;
}
