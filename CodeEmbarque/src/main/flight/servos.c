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

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include <platform.h>
#include "debug.h"

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"
#include "common/filter.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"
#include "config/config_reset.h"

#include "drivers/system.h"
#include "drivers/pwm_output.h"
#include "drivers/pwm_mapping.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/system.h"

#include "rx/rx.h"
#include "io/gimbal.h"
#include "io/motor_and_servo.h"
#include "io/rc_controls.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"

#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/failsafe.h"
#include "flight/pid.h"
#include "flight/imu.h"

//VARIABLES-------------------------------------------------------------
uint8_t       useServo;
int16_t       servoCmd[MAX_SUPPORTED_SERVOS];

servoParam_t  servoConfVTOL[MAX_SUPPORTED_SERVOS];
servoMixer_t  servoMixerVTOL[MAX_SUPPORTED_SERVOS];
biquad_t      servoFilterState[MAX_SUPPORTED_SERVOS];

// FONCTIONS -----------------------------------------------------------

void initServos(){
    servoMixer_t servoMixertmp[MAX_SUPPORTED_SERVOS] = {
        // format servo, input, rate, speed, min, max, box
        { SERVO_FLAPPERON, INPUT_STABILIZED_ROLL,  100, 0, 0, 100, 0 },
        { SERVO_RUDDER,    INPUT_STABILIZED_YAW,   100, 0, 0, 100, 0 },
        { SERVO_ELEVATOR,  INPUT_STABILIZED_PITCH, 100, 0, 0, 100, 0 },
        { SERVO_AUX1,      INPUT_RC_AUX1,          100, 0, 0, 100, 0 }};
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++)
        servoMixerVTOL[i] = servoMixertmp[i];

    // enable servos for mixes that require them. note, this shifts motor counts.
    useServo = true;

    // give all servos a default command
    for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++)
        servoCmd[i] = DEFAULT_SERVO_MIDDLE;

    for(uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++){
        servoConfVTOL[i].min                = DEFAULT_SERVO_MIN;
        servoConfVTOL[i].max                = DEFAULT_SERVO_MAX;
        servoConfVTOL[i].middle             = DEFAULT_SERVO_MIDDLE;
        servoConfVTOL[i].rate               = 100;
        servoConfVTOL[i].angleAtMin         = DEFAULT_SERVO_MIN_ANGLE;
        servoConfVTOL[i].angleAtMax         = DEFAULT_SERVO_MAX_ANGLE;
        servoConfVTOL[i].forwardFromChannel = CHANNEL_FORWARDING_DISABLED;
    }
}

void initServoFilter(uint32_t targetLooptime){
    if (mixerConfigVTOL.servo_lowpass_enable) {
        for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            BiQuadNewLpf(mixerConfigVTOL.servo_lowpass_freq, &servoFilterState[i], targetLooptime);
        }
    }
}

void servoMixer(uint8_t phaseDeVol){
    if (phaseDeVol == VOL_AVION) {
        static int16_t output[MAX_SERVO_RULES];
               int16_t input[INPUT_SOURCE_COUNT]; // Range [-500:+500]

        if (FLIGHT_MODE(PASSTHRU_MODE)) {
            // Direct passthru from RX
            input[INPUT_STABILIZED_ROLL]  = rcCommand[ROLL];
            input[INPUT_STABILIZED_PITCH] = rcCommand[PITCH];
            input[INPUT_STABILIZED_YAW]   = rcCommand[YAW];
        } else {
            // Assisted modes (gyro only or gyro+acc according to AUX configuration in Gui
            input[INPUT_STABILIZED_ROLL]  = axisPID[FD_ROLL];
            input[INPUT_STABILIZED_PITCH] = axisPID[FD_PITCH];
            input[INPUT_STABILIZED_YAW]   = axisPID[FD_YAW];
        }

        // center the RC input value around the RC middle value
        // by subtracting the RC middle value from the RC input value, we get:
        // data - middle = input
        // 2000 - 1500 = +500
        // 1500 - 1500 = 0
        // 1000 - 1500 = -500
        input[INPUT_RC_ROLL]     = rcData[ROLL]     - rxConfig()->midrc;
        input[INPUT_RC_PITCH]    = rcData[PITCH]    - rxConfig()->midrc;
        input[INPUT_RC_YAW]      = rcData[YAW]      - rxConfig()->midrc;
        input[INPUT_RC_THROTTLE] = rcData[THROTTLE] - rxConfig()->midrc;
        input[INPUT_RC_AUX1]     = rcData[AUX1]     - rxConfig()->midrc;
        input[INPUT_RC_AUX2]     = rcData[AUX2]     - rxConfig()->midrc;
        input[INPUT_RC_AUX3]     = rcData[AUX3]     - rxConfig()->midrc;
        input[INPUT_RC_AUX4]     = rcData[AUX4]     - rxConfig()->midrc;

        //initialisation des commandes aux servos
        for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++){
            //Init cmd
            servoCmd[i] = 0;

            // mix servos
            uint8_t  target = servoMixerVTOL[i].targetChannel;
            uint8_t  from   = servoMixerVTOL[i].inputSource;
            uint16_t width  = servoConfVTOL[target].max - servoConfVTOL[target].min;
            int16_t  min    = servoMixerVTOL[i].min * width / 100 - width / 2;
            int16_t  max    = servoMixerVTOL[i].max * width / 100 - width / 2;

            //Calcul de la sortie (nombre entier)
            if (servoMixerVTOL[i].speed == 0)
                //Vitesse des servo infinie : sortie - entrée
                output[i] = input[from];
            else {
                //Vitesse des servo finie (v)
                if (output[i] < input[from]){
                    // Sortie (o) en vitesse : o = o+v contraint dans [o, i]
                    output[i] = constrain(output[i] + servoMixerVTOL[i].speed, output[i], input[from]);
                }else if (output[i] > input[from]){
                    // Sortie (o) en vitesse : o = o-v contraint dans [o, i]
                    output[i] = constrain(output[i] - servoMixerVTOL[i].speed, input[from], output[i]);
                }
            }

            //Commande c = c +/- o*(%vmax)
            servoCmd[target] += servoDirection(target, from) * constrain(((int32_t)output[i] * servoMixerVTOL[i].rate) / 100, min, max);
            //Taux de commande (c) à prendre en compte : c = c*(%cmax)
            servoCmd[i]  = ((int32_t)servoConfVTOL[i].rate * servoCmd[i]) / 100L;
            //La commande est relative a la position "milieu" du servo
            servoCmd[i] += servoConfVTOL[i].middle;
            //On borne la commande
            servoCmd[i] = constrain(servoCmd[i], servoConfVTOL[i].min, servoConfVTOL[i].max); // limit the values
        }
    }
    else{
        for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++){
            //Init cmd
            servoCmd[i] = 0;
        }
    }
}

void filterServos(void){
#if defined(MIXER_DEBUG)
    uint32_t startTime = micros();

#endif
    if (mixerConfigVTOL.servo_lowpass_enable) {
        for (uint8_t i = 0; i < MAX_SUPPORTED_SERVOS; i++) {
            servoCmd[i] = lrintf(applyBiQuadFilter((float) servoCmd[i], &servoFilterState[i]));

            // Sanity check
            servoCmd[i] = constrain(servoCmd[i], servoConfVTOL[i].min, servoConfVTOL[i].max);
        }
    }
#if defined(MIXER_DEBUG)
    debug[0] = (int16_t)(micros() - startTime);
#endif
}

void writeServos(void){
    pwmWriteServo(1, servoCmd[SERVO_ELEVATOR]);
    pwmWriteServo(2, servoCmd[SERVO_FLAPPERON]);
    pwmWriteServo(3, servoCmd[SERVO_RUDDER]);
    pwmWriteServo(4, servoCmd[SERVO_AUX1]);
}

bool isMixerUsingServos(void){
    return useServo;
}

int servoDirection(int servoIndex, int inputSource){
    // determine the direction (reversed or not) from the direction bitfield of the servo
    if (servoConfVTOL[servoIndex].reversedSources & (1 << inputSource))
        return -1;
    else
        return 1;
}

