/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "io/motor_and_servo.h"
#include "io/rc_controls.h"
#include "flight/pid.h"
#include "sensors/barometer.h"

// Variables globales
extern int32_t G_altitude_altEst;
extern int32_t G_altitude_altCmd;
extern int32_t G_altitude_vzEst;

// Corp
void altitudeCalculate(uint32_t currentTime);
void altitudeApplyAltHold(void);
void altitudeUpdateBaroAltHoldState(void);
void altitudeUpdateSonarAltHoldState(void);
