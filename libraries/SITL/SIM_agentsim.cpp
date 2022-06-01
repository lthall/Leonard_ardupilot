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
       Simulator Connector for agentsim
*/

#include "SIM_agentsim.h"

#include <stdio.h>
#include <arpa/inet.h>
#include <errno.h>

#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/replace.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

static const float ft_to_m = 0.3048f;

agentsim::agentsim(const char *frame_str) :
       Aircraft(frame_str),
       sock(true)
{
    rate_hz = 1000.0f;
    set_speedup(1.0f);
       printf("Starting SITL agentsim\n");
}

/*
       Create & set in/out socket
*/
void agentsim::set_interface_ports(const char* address, const int port_in, const int port_out)
{
       if (!sock.bind("0.0.0.0", port_in)) {
               printf("Unable to bind agentsim sensor_in socket at port %u - Error: %s\n",
                                port_in, strerror(errno));
               return;
       }
       printf("Bind SITL sensor input at %s:%u\n", "127.0.0.1", port_in);
       sock.set_blocking(false);
       sock.reuseaddress();

       agentsim_ip = address;
       agentsim_control_port = port_out;
       agentsim_sensor_port = port_in;

       printf("agentsim control interface set to %s:%u\n", agentsim_ip, agentsim_control_port);
}

/*
       Decode and send servos
*/
void agentsim::send_servos(const struct sitl_input &input)
{
       servo_packet pkt{0};
    float hot;

       for (uint8_t i=0; i<num_pwm_sig; i++) {
               pkt.pwm[i] = input.servos[i];
       }

    terrain->height_amsl(location, hot, false);
    pkt.hot = (double)hot;

       ssize_t send_ret = sock.sendto(&pkt, sizeof(pkt), agentsim_ip, agentsim_control_port);
       if (send_ret != sizeof(pkt)) {
               if (send_ret <= 0) {
                       printf("Unable to send servo output to %s:%u - Error: %s, Return value: %ld\n",
                                        agentsim_ip, agentsim_control_port, strerror(errno), send_ret);
               } else {
                       printf("Sent %ld bytes instead of %ld bytes\n", send_ret, sizeof(pkt));
               }
       }
}

/*
       Receive new sensor data from simulator
       This is a blocking function (via polling)
*/
void agentsim::recv_fdm()
{
    // Receive sensor packet
    ssize_t ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, 100);
    while (ret <= 0) {
        printf("No sensor message received - %s\n", strerror(errno));
        ret = sock.recv(&sensor_buffer[sensor_buffer_len], sizeof(sensor_buffer)-sensor_buffer_len, 100);
    }
    memcpy(&sensdat,sensor_buffer,sizeof(sensdat_type));
    recv_initmsg_confirm = 1;

    accel_body = Vector3f(ft_to_m * (float)sensdat.bd_tot_accel_fpss[0],
                          ft_to_m * (float)sensdat.bd_tot_accel_fpss[1],
                          ft_to_m * (float)sensdat.bd_tot_accel_fpss[2]);


    gyro = Vector3f((float)sensdat.bd_ang_rates_rps[0],
                    (float)sensdat.bd_ang_rates_rps[1],
                    (float)sensdat.bd_ang_rates_rps[2]);

    velocity_ef = Vector3f(ft_to_m * (float)sensdat.ned_vel_fps[0],
                           ft_to_m * (float)sensdat.ned_vel_fps[1],
                           ft_to_m * (float)sensdat.ned_vel_fps[2]);

    location.lat = (float)sensdat.lat_deg * 1.0e7;
    location.lng = (float)sensdat.long_deg * 1.0e7;
    location.alt = ft_to_m * (float)sensdat.alt_ft * 100.0f;

    dcm.from_euler((float)sensdat.euler_att_rad[0], (float)sensdat.euler_att_rad[1], (float)sensdat.euler_att_rad[2]);

    time_now_us = (uint64_t)(sensdat.simtime_sec * 1000000.0);


    //rcin_chan_count = state.rc.rc_channels.length < 8 ? state.rc.rc_channels.length : 8;
    for (uint8_t i=0; i < 8; i++) {
        rcin[i] = 1000;
    }

    rcin[1] = 2000;


#if 0
    AP::logger().Write("ASM1", "TimeUS,TUS,R,P,Y,GX,GY,GZ",
                       "QQffffff",
                       AP_HAL::micros64(),
                       state.timestamp,
                       degrees(state.pose.roll),
                       degrees(state.pose.pitch),
                       degrees(state.pose.yaw),
                       degrees(gyro.x),
                       degrees(gyro.y),
                       degrees(gyro.z));

    Vector3f velocity_bf = dcm.transposed() * velocity_ef;
    position = home.get_distance_NED(location);

    AP::logger().Write("ASM2", "TimeUS,AX,AY,AZ,VX,VY,VZ,PX,PY,PZ,Alt,SD",
                       "Qfffffffffff",
                       AP_HAL::micros64(),
                       accel_body.x,
                       accel_body.y,
                       accel_body.z,
                       velocity_bf.x,
                       velocity_bf.y,
                       velocity_bf.z,
                       position.x,
                       position.y,
                       position.z,
                       state.gps.alt,
                       velocity_ef.z);
#endif

}

/*
  update the agentsim simulation by one time step
*/
void agentsim::update(const struct sitl_input &input)
{
    if (recv_initmsg_confirm == 0)
    {
        send_initdat();
    }
    else
    {
           send_servos(input);
    }
    recv_fdm();

    // update magnetic field
    update_mag_field_bf();
}

void agentsim::send_initdat()
{
    initdat_packet initdat_pkt;
    initdat_pkt.init_hdg = (double)home_yaw;
    initdat_pkt.init_lat = (double)home.lat*1e-7;
    initdat_pkt.init_long = (double)home.lng*1e-7;
    initdat_pkt.init_alt = (double)home.alt*0.01;

       ssize_t send_ret = sock.sendto(&initdat_pkt, sizeof(initdat_pkt), agentsim_ip, agentsim_control_port);
       if (send_ret != sizeof(initdat_pkt)) {
               if (send_ret <= 0) {
                       printf("Unable to send init data output to %s:%u - Error: %s, Return value: %ld\n",
                                        agentsim_ip, agentsim_control_port, strerror(errno), send_ret);
               } else {
                       printf("Sent %ld bytes instead of %ld bytes\n", send_ret, sizeof(initdat_pkt));
               }
       }
}
