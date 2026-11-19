/*
   Simulator for an AVT CM62 MAVLink (Gimbal Protocol v2) gimbal

   The real payload presents several MAVLink components on one link: the
   gimbal device (MAV_COMP_ID_GIMBAL) and its EO camera (MAV_COMP_ID_CAMERA),
   which reports vendor "AVTA" and model "EO1".  This simulator presents both.
   The camera answers the camera-protocol requests and commands a ground
   station sends it (zoom, focus, photo, video) and streams CAMERA_SETTINGS
   and CAMERA_CAPTURE_STATUS so the ground station sees the result.

   The gimbal device accepts AVT's object-detection custom command (a vendor
   DO_SET_PARAMETER, param1 1006).  Not simulated: the IR camera the real
   payload also presents, and its EO/IR swap (param1 1000), whose effect on the
   wire has not been captured; the swap is refused as unsupported.

./Tools/autotest/sim_vehicle.py -v ArduCopter -A --serial5=sim:avt_cm62_gimbal

param set MNT1_TYPE 6          # MAVLink
param set CAM1_TYPE 4          # mount
param set SERIAL5_PROTOCOL 2   # MAVLink2
reboot
*/

#pragma once

#include "SIM_config.h"

#if AP_SIM_AVT_CM62_ENABLED

#include "SIM_MAVLinkGimbalv2.h"
#include <AP_Math/AP_Math.h>

namespace SITL {

class AVT_CM62 : public MAVLinkGimbalv2 {
public:
    void set_instance(uint8_t instance) override;

protected:
    const char *get_vendor_name()      const override { return "AVTA"; }
    const char *get_model_name()       const override { return "SIM_AVTA"; }
    // firmware_version encoding: major=1 | (minor=2)<<8 | (patch=3)<<16
    uint32_t    get_firmware_version() const override { return (3U << 16) | (2U << 8) | 1U; }
    uint16_t    get_cap_flags()        const override {
        return GIMBAL_DEVICE_CAP_FLAGS_HAS_PITCH_AXIS |
               GIMBAL_DEVICE_CAP_FLAGS_HAS_YAW_AXIS;
    }
    float get_pitch_min_rad() const override { return radians(-45.0f); }
    float get_pitch_max_rad() const override { return radians( 45.0f); }
    float get_yaw_min_rad()   const override { return radians(-180.0f); }
    float get_yaw_max_rad()   const override { return radians( 180.0f); }

    void handle_peripheral_message(const mavlink_message_t &msg) override;
    void update_peripherals(uint32_t now_ms) override;

private:
    // AVT custom commands (DO_SET_PARAMETER to the gimbal device)
    void gimbal_handle_command(const mavlink_message_t &msg, uint16_t command,
                               uint8_t target_system, uint8_t target_component,
                               float p1, float p2);
    bool _object_detection;

    // the EO camera
    void camera_handle_command(const mavlink_message_t &msg, uint16_t command,
                               uint8_t target_system, uint8_t target_component,
                               float p1, float p2);
    void camera_update_lens(uint32_t now_ms);
    void camera_send_heartbeat();
    void camera_send_information();
    void camera_send_settings();
    void camera_send_capture_status();
    void camera_send_ack(const mavlink_message_t &request, uint16_t command, MAV_RESULT result);

    uint8_t _camera_compid {MAV_COMP_ID_CAMERA};
    // the camera's own sequence numbers, apart from the gimbal's
    mavlink_status_t _camera_status;

    uint32_t _camera_last_heartbeat_ms;
    uint32_t _camera_last_settings_ms;
    uint32_t _camera_last_capture_status_ms;
    uint32_t _camera_last_lens_ms;

    // zoom and focus, as percentages of their full range, and the direction
    // each is being driven in (-1, 0 or 1)
    float  _zoom_pct;
    int8_t _zoom_rate;
    float  _focus_pct {50.0f};
    int8_t _focus_rate;

    bool     _recording;
    uint32_t _recording_start_ms;
    int32_t  _image_count;
};

}  // namespace SITL

#endif  // AP_SIM_AVT_CM62_ENABLED
