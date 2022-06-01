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

   This class inherits from AC_AttitudeControl_Multi and provides functionality
   specific to tailsitter quadplanes.
   1) "body-frame" roll control mode for all tailsitters
   2) a relax_attitude_controller method needed for coping with vectored tailsitters
 */
#include "AC_AttitudeControl_TS.h"


AC_AttitudeControl_TS::AC_AttitudeControl_TS(AP_AHRS_View &ahrs, const AP_Vehicle::MultiCopter &aparm, AP_MotorsMulticopter& motors, float dt) :
    AC_AttitudeControl_Multi(ahrs, aparm, motors, dt)
{
}
