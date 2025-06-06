/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "DeviceBus.h"

#include <AP_HAL/AP_HAL.h>
#include <stdio.h>

#include "Scheduler.h"
#include "Semaphores.h"

using namespace QURT;

extern const AP_HAL::HAL& hal;

DeviceBus::DeviceBus(AP_HAL::Scheduler::priority_base _thread_priority) :
    semaphore(),
    thread_priority(_thread_priority)
{
}

/*
  per-bus callback thread
*/
void DeviceBus::bus_thread(void)
{
    while (true) {
        uint64_t now = AP_HAL::micros64();
        DeviceBus::callback_info *callback;

        // find a callback to run
        for (callback = callbacks; callback; callback = callback->next) {
            if (now >= callback->next_usec) {
                while (now >= callback->next_usec) {
                    callback->next_usec += callback->period_usec;
                }
                // call it with semaphore held
                WITH_SEMAPHORE(semaphore);
                callback->cb();
            }
        }

        // work out when next loop is needed
        uint64_t next_needed = 0;
        now = AP_HAL::micros64();

        for (callback = callbacks; callback; callback = callback->next) {
            if (next_needed == 0 ||
                callback->next_usec < next_needed) {
                next_needed = callback->next_usec;
                if (next_needed < now) {
                    next_needed = now;
                }
            }
        }

        // delay for at most 50ms, to handle newly added callbacks
        uint32_t delay = 50000;
        if (next_needed >= now && next_needed - now < delay) {
            delay = next_needed - now;
        }
        // don't delay for less than 100usec, so one thread doesn't
        // completely dominate the CPU
        if (delay < 100) {
            delay = 100;
        }
        hal.scheduler->delay_microseconds(delay);
    }
    return;
}

AP_HAL::Device::PeriodicHandle DeviceBus::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb, AP_HAL::Device *_hal_device)
{
    if (!thread_started) {
        thread_started = true;
        hal_device = _hal_device;
        // setup a name for the thread
        char name[30];
        switch (hal_device->bus_type()) {
        case AP_HAL::Device::BUS_TYPE_I2C:
            snprintf(name, sizeof(name), "APM_I2C:%u",
                     hal_device->bus_num());
            break;

        case AP_HAL::Device::BUS_TYPE_SPI:
            snprintf(name, sizeof(name), "APM_SPI:%u",
                     hal_device->bus_num());
            break;
        default:
            break;
        }
#ifdef BUSDEBUG
        printf("%s:%d Thread Start\n", __PRETTY_FUNCTION__, __LINE__);
#endif
        hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&DeviceBus::bus_thread, void),
                                     name, HAL_QURT_DEVICE_STACK_SIZE, thread_priority, 0);
    }
    DeviceBus::callback_info *callback = new DeviceBus::callback_info;
    if (callback == nullptr) {
        return nullptr;
    }
    callback->cb = cb;
    callback->period_usec = period_usec;
    callback->next_usec = AP_HAL::micros64() + period_usec;

    // add to linked list of callbacks on thread
    callback->next = callbacks;
    callbacks = callback;

    return callback;
}

/*
 * Adjust the timer for the next call: it needs to be called from the bus
 * thread, otherwise it will race with it
 */
bool DeviceBus::adjust_timer(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    // TODO: add timer adjustment
    return true;
}
