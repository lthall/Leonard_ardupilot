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
  simulator connection for another AP as simulator
*/

#include "SIM_AP.h"

#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include <sched.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

#define TIMESTAMPS_DIFF_US 1000000

extern const AP_HAL::HAL& hal;

namespace SITL {

AP_Vehicle::AP_Vehicle(const char *_frame_str) :
    Aircraft(_frame_str),
    last_timestamp_us(0),
    sock(true)
{
    fprintf(stdout, "Preparing AP_Vehicle socket\n");
    if (!sock.reuseaddress()) {
        fprintf(stderr, "SITL: socket reuseaddress failed on AP_Vehicle in port: %d - %s\n", fdm_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    // try to bind to a specific port so that if we restart ArduPilot
    // AP_Vehicle keeps sending us packets. Not strictly necessary but
    // useful for debugging
    if (!sock.bind("0.0.0.0", fdm_port)) {

        fprintf(stderr, "SITL: socket bind failed on AP_Vehicle in port : %d - %s\n", fdm_port, strerror(errno));
    sock.reuseaddress();
        fprintf(stderr, "Aborting launch...\n");
    sock.set_blocking(false);
        exit(1);
    }
    if (!sock.set_blocking(false)) {
        fprintf(stderr, "SITL: socket set_blocking(false) failed on AP_Vehicle in port: %d - %s\n", fdm_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
    if (!sock.set_cloexec()) {
        fprintf(stderr, "SITL: socket set_cloexec() failed on AP_Vehicle in port: %d - %s\n", fdm_port, strerror(errno));
        fprintf(stderr, "Aborting launch...\n");
        exit(1);
    }
}

/*
  receive an update from the FDM
  This is a blocking function
 */
void AP_Vehicle::recv_fdm()
{
    struct sitl_fdm pkt;

    while (sock.recv(&pkt, sizeof(pkt), 100) != sizeof(pkt)) {
        sched_yield();
    }

    received_fdm = pkt;
}

void AP_Vehicle::fill_fdm(struct sitl_fdm &fdm)
{
    if (fdm.timestamp_us > received_fdm.timestamp_us + TIMESTAMPS_DIFF_US) {
        // The old timestamp is more than 1 second older than the new timestamp
        // Probably because of a sender reboot. we should also reboot in this state.
        gcs().send_text(MAV_SEVERITY_WARNING, "Received large timestamp diff. rebooting.");
        hal.scheduler->reboot(false);
    }
    fdm = received_fdm;
}

/*
  update the AP vehicle simulation by one time step
 */
void AP_Vehicle::update(const struct sitl_input &input)
{
    recv_fdm();
}

} // namespace SITL
