/*
   Simulator for an AVT CM62 MAVLink (Gimbal Protocol v2) gimbal
*/

#include "SIM_config.h"

#if AP_SIM_AVT_CM62_ENABLED

#include "SIM_AVT_CM62.h"
#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

namespace SITL {

// The capability flags the real EO camera advertises.  It under-reports: its
// vendor documents photo, video and focus commands it does not flag, and it
// answers them, as this camera does.
static constexpr uint32_t CAMERA_CAP_FLAGS =
    CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM |
    CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM |
    CAMERA_CAP_FLAGS_HAS_TRACKING_POINT |
    CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE;

// how fast a continuous zoom or focus drive moves, in percent of full range
// per second, and how far one step moves
static constexpr float ZOOM_RATE_PCT_PER_S  = 20.0f;
static constexpr float FOCUS_RATE_PCT_PER_S = 25.0f;
static constexpr float LENS_STEP_PCT        = 10.0f;

void AVT_CM62::set_instance(uint8_t instance)
{
    MAVLinkGimbalv2::set_instance(instance);
    // a second simulated payload takes the next camera id, as its gimbal
    // takes the next gimbal id
    _camera_compid = MAV_COMP_ID_CAMERA + instance;
}

void AVT_CM62::handle_peripheral_message(const mavlink_message_t &msg)
{
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_COMMAND_LONG: {
        mavlink_command_long_t cmd;
        mavlink_msg_command_long_decode(&msg, &cmd);
        camera_handle_command(msg, cmd.command, cmd.target_system, cmd.target_component,
                              cmd.param1, cmd.param2);
        break;
    }
    case MAVLINK_MSG_ID_COMMAND_INT: {
        mavlink_command_int_t cmd;
        mavlink_msg_command_int_decode(&msg, &cmd);
        camera_handle_command(msg, cmd.command, cmd.target_system, cmd.target_component,
                              cmd.param1, cmd.param2);
        break;
    }
    default:
        break;
    }
}

void AVT_CM62::camera_handle_command(const mavlink_message_t &msg, uint16_t command,
                                     uint8_t target_system, uint8_t target_component,
                                     float p1, float p2)
{
    if (target_system != vehicle_system_id() || target_component != _camera_compid) {
        return;
    }

    MAV_RESULT result = MAV_RESULT_ACCEPTED;
    switch (command) {
    case MAV_CMD_REQUEST_MESSAGE:
        // answer first, so the reply is there when the ACK says it was sent
        switch ((uint32_t)p1) {
        case MAVLINK_MSG_ID_CAMERA_INFORMATION:
            camera_send_information();
            break;
        case MAVLINK_MSG_ID_CAMERA_SETTINGS:
            camera_send_settings();
            break;
        case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS:
            camera_send_capture_status();
            break;
        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }
        break;

    // the deprecated per-message requests, which ground stations still send
    case MAV_CMD_REQUEST_CAMERA_INFORMATION:
        camera_send_information();
        break;
    case MAV_CMD_REQUEST_CAMERA_SETTINGS:
        camera_send_settings();
        break;
    case MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS:
        camera_send_capture_status();
        break;

    case MAV_CMD_SET_CAMERA_ZOOM:
        switch ((uint8_t)p1) {
        case ZOOM_TYPE_STEP:
            _zoom_rate = 0;
            _zoom_pct = constrain_float(_zoom_pct + LENS_STEP_PCT * (p2 > 0 ? 1 : -1), 0, 100);
            break;
        case ZOOM_TYPE_CONTINUOUS:
            _zoom_rate = (p2 > 0) ? 1 : (p2 < 0) ? -1 : 0;
            break;
        case ZOOM_TYPE_RANGE:
            _zoom_rate = 0;
            _zoom_pct = constrain_float(p2, 0, 100);
            break;
        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }
        break;

    case MAV_CMD_SET_CAMERA_FOCUS:
        switch ((uint8_t)p1) {
        case FOCUS_TYPE_STEP:
            _focus_rate = 0;
            _focus_pct = constrain_float(_focus_pct + LENS_STEP_PCT * (p2 > 0 ? 1 : -1), 0, 100);
            break;
        case FOCUS_TYPE_CONTINUOUS:
            _focus_rate = (p2 > 0) ? 1 : (p2 < 0) ? -1 : 0;
            break;
        case FOCUS_TYPE_RANGE:
            _focus_rate = 0;
            _focus_pct = constrain_float(p2, 0, 100);
            break;
        default:
            result = MAV_RESULT_UNSUPPORTED;
            break;
        }
        break;

    case MAV_CMD_IMAGE_START_CAPTURE:
        _image_count++;
        break;
    case MAV_CMD_IMAGE_STOP_CAPTURE:
        break;

    case MAV_CMD_VIDEO_START_CAPTURE:
        if (!_recording) {
            _recording = true;
            _recording_start_ms = AP_HAL::millis();
        }
        break;
    case MAV_CMD_VIDEO_STOP_CAPTURE:
        _recording = false;
        break;

    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }

    camera_send_ack(msg, command, result);

    // report a change at once rather than at the next periodic status
    if (result == MAV_RESULT_ACCEPTED) {
        if (command == MAV_CMD_SET_CAMERA_ZOOM || command == MAV_CMD_SET_CAMERA_FOCUS) {
            camera_send_settings();
        } else if (command == MAV_CMD_IMAGE_START_CAPTURE ||
                   command == MAV_CMD_VIDEO_START_CAPTURE ||
                   command == MAV_CMD_VIDEO_STOP_CAPTURE) {
            camera_send_capture_status();
        }
    }
}

void AVT_CM62::camera_update_lens(uint32_t now_ms)
{
    const float dt = (now_ms - _camera_last_lens_ms) * 0.001f;
    _camera_last_lens_ms = now_ms;
    if (_zoom_rate != 0) {
        _zoom_pct = constrain_float(_zoom_pct + _zoom_rate * ZOOM_RATE_PCT_PER_S * dt, 0, 100);
        if (_zoom_pct <= 0 || _zoom_pct >= 100) {
            _zoom_rate = 0;    // the lens is at the end of its travel
        }
    }
    if (_focus_rate != 0) {
        _focus_pct = constrain_float(_focus_pct + _focus_rate * FOCUS_RATE_PCT_PER_S * dt, 0, 100);
        if (_focus_pct <= 0 || _focus_pct >= 100) {
            _focus_rate = 0;
        }
    }
}

void AVT_CM62::update_peripherals(uint32_t now_ms)
{
    camera_update_lens(now_ms);

    if (now_ms - _camera_last_heartbeat_ms >= 1000) {
        _camera_last_heartbeat_ms = now_ms;
        camera_send_heartbeat();
    }

    // faster while the lens moves, so a ground station's readout follows it
    const bool lens_moving = _zoom_rate != 0 || _focus_rate != 0;
    if (now_ms - _camera_last_settings_ms >= (lens_moving ? 200U : 1000U)) {
        camera_send_settings();
    }

    if (now_ms - _camera_last_capture_status_ms >= 1000) {
        camera_send_capture_status();
    }
}

void AVT_CM62::camera_send_heartbeat()
{
    mavlink_heartbeat_t hb {};
    hb.type            = MAV_TYPE_CAMERA;
    hb.autopilot       = MAV_AUTOPILOT_INVALID;
    hb.system_status   = MAV_STATE_ACTIVE;
    hb.mavlink_version = 3;

    mavlink_message_t msg;
    mavlink_msg_heartbeat_encode_status(vehicle_system_id(), _camera_compid,
                                        &_camera_status, &msg, &hb);
    send_mavlink_message(msg);
}

void AVT_CM62::camera_send_information()
{
    mavlink_camera_information_t info {};
    info.time_boot_ms     = AP_HAL::millis();
    info.firmware_version = get_firmware_version();
    info.focal_length     = NAN;
    info.sensor_size_h    = NAN;
    info.sensor_size_v    = NAN;
    info.flags            = CAMERA_CAP_FLAGS;
    info.resolution_h     = 1920;
    info.resolution_v     = 1080;
    info.gimbal_device_id = gimbal_compid();
    strncpy_noterm((char *)info.vendor_name, "AVTA", sizeof(info.vendor_name));
    strncpy_noterm((char *)info.model_name,  "EO1",  sizeof(info.model_name));

    mavlink_message_t msg;
    mavlink_msg_camera_information_encode_status(vehicle_system_id(), _camera_compid,
                                                 &_camera_status, &msg, &info);
    send_mavlink_message(msg);
}

void AVT_CM62::camera_send_settings()
{
    _camera_last_settings_ms = AP_HAL::millis();

    mavlink_camera_settings_t settings {};
    settings.time_boot_ms = _camera_last_settings_ms;
    settings.mode_id      = _recording ? CAMERA_MODE_VIDEO : CAMERA_MODE_IMAGE;
    settings.zoomLevel    = _zoom_pct;
    settings.focusLevel   = _focus_pct;

    mavlink_message_t msg;
    mavlink_msg_camera_settings_encode_status(vehicle_system_id(), _camera_compid,
                                              &_camera_status, &msg, &settings);
    send_mavlink_message(msg);
}

void AVT_CM62::camera_send_capture_status()
{
    _camera_last_capture_status_ms = AP_HAL::millis();

    mavlink_camera_capture_status_t status {};
    status.time_boot_ms       = _camera_last_capture_status_ms;
    status.image_status       = 0;
    status.video_status       = _recording ? 1 : 0;
    status.recording_time_ms  = _recording ? (_camera_last_capture_status_ms - _recording_start_ms) : 0;
    status.available_capacity = 32000;    // MiB: a card with room to spare
    status.image_count        = _image_count;

    mavlink_message_t msg;
    mavlink_msg_camera_capture_status_encode_status(vehicle_system_id(), _camera_compid,
                                                    &_camera_status, &msg, &status);
    send_mavlink_message(msg);
}

void AVT_CM62::camera_send_ack(const mavlink_message_t &request, uint16_t command, MAV_RESULT result)
{
    mavlink_command_ack_t ack {};
    ack.command          = command;
    ack.result           = (uint8_t)result;
    ack.progress         = 255;
    ack.target_system    = request.sysid;
    ack.target_component = request.compid;

    mavlink_message_t msg;
    mavlink_msg_command_ack_encode_status(vehicle_system_id(), _camera_compid,
                                          &_camera_status, &msg, &ack);
    send_mavlink_message(msg);
}

}  // namespace SITL

#endif  // AP_SIM_AVT_CM62_ENABLED
