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

#pragma once

#include <AP_HAL/utility/Socket.h>

#include "SIM_Aircraft.h"

namespace SITL {

/*
  another AP as  simulator
 */
class AP_Vehicle : public Aircraft {
public:
    AP_Vehicle(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
        return new AP_Vehicle(frame_str);
    }

    virtual void fill_fdm(struct sitl_fdm &fdm) override;

private:
    static const uint16_t fdm_port = PASSENGER_PORT;

    void recv_fdm();

    struct sitl_fdm received_fdm;

    uint64_t last_timestamp_us;
    SocketAPM sock;
};

} // namespace SITL
