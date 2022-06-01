#pragma once

#include <AP_HAL/utility/Socket.h>
#include "SIM_Aircraft.h"

namespace SITL {

/*
       agentsim Simulator
*/

struct sensdat_type {
    double bd_tot_accel_fpss[3];
    double bd_ang_rates_rps[3];
    double ned_vel_fps[3];
    double lat_deg;
    double long_deg;
    double alt_ft;
    double euler_att_rad[3];
    double simtime_sec;
};

struct initdat_packet {
    double init_lat;
    double init_long;
    double init_alt;
    double init_hdg;
};

class agentsim : public Aircraft {
public:
       agentsim(const char *frame_str);

       /* update model by one time step */
       void update(const struct sitl_input &input) override;

       /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new agentsim(frame_str);
    }

  /*  Create and set in/out socket for agentsim simulator */
  void set_interface_ports(const char* address, const int port_in, const int port_out) override;

private:

  sensdat_type sensdat;
  initdat_packet initdat;
  int recv_initmsg_confirm = 0;

  // Servo packet struct sent to agentsim (SITL -> agentsim)
       static const int num_pwm_sig = 16;

       struct servo_packet {
               uint16_t pwm[num_pwm_sig];
               double hot;
       };

       // default connection_info_.ip_address
       const char *agentsim_ip = "127.0.0.1";

       // connection_info_.ip_port
       uint16_t agentsim_sensor_port = 9003;

       // connection_info_.sitl_ip_port
       uint16_t agentsim_control_port = 9002;

       SocketAPM sock;

       void send_servos(const struct sitl_input &input);
       void recv_fdm();
       void send_initdat();

  uint8_t sensor_buffer[65000];
  uint32_t sensor_buffer_len;

};

}
