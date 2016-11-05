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

#include "config/parameter_group_ids.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"
#include "config/config_reset.h"

mixerConfig_t mixerConfigVTOL;
motorMixer_t  motorMixerVTOL[MAX_SUPPORTED_MOTORS];

int16_t       motorsThrottle[MAX_SUPPORTED_MOTORS];
int16_t       motorDisarmed[MAX_SUPPORTED_MOTORS];
bool          motorLimitReached;

// FONCTIONS STATIC ----------------------------------------------------
static uint16_t mixConstrainMotorForFailsafeCondition(uint8_t motorIndex){
    return constrain(motorsThrottle[motorIndex], motorAndServoConfig()->mincommand, motorAndServoConfig()->maxthrottle);
}

static uint8_t getPhaseDeVol(){
    //comande auxiliaire pour transition phase de vol
    int16_t cmdAux = rcData[AUX1];

    //phases de vol
    if (cmdAux <= AUX_QUAD){
      return VOL_QUAD;
    }
    else if (cmdAux >= AUX_AVION){
      return VOL_AVION;
    }
    else{
      return VOL_TRANS;
    }
}

//FONCTIONS ------------------------------------------------------------
void initMixer(){
    //motors config
    motorMixer_t motorMixertmp[MAX_SUPPORTED_MOTORS] = {
        /*throttle, roll, pitch,  yaw                     4CW   2CCW  */
        { 1.0f, -1.0f,  1.0f, -1.0f }, /* REAR_R  (M1)       \ /      */
        { 1.0f, -1.0f, -1.0f,  1.0f }, /* FRONT_R (M2)        X       */
        { 1.0f,  1.0f,  1.0f,  1.0f }, /* REAR_L  (M3)       / \      */
        { 1.0f,  1.0f, -1.0f, -1.0f }  /* FRONT_L (M4)    3CCW  1CW   */};
    for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++){
        motorMixerVTOL[i] = motorMixertmp[i];
    }

    //mixer config
    mixerConfigVTOL.pid_at_min_throttle       = 1;
    mixerConfigVTOL.yaw_motor_direction       = 1;
    mixerConfigVTOL.yaw_jump_prevention_limit = 200;
    mixerConfigVTOL.tri_unarmed_servo         = 1;
    mixerConfigVTOL.servo_lowpass_freq        = 400.0f;
}

/* fonction pour initialiser la valeur des moteur en etat "desarme" */
void mixerResetDisarmedMotors(void){
    // set disarmed motorsThrottle values
    for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motorDisarmed[i] = motorAndServoConfig()->mincommand;
}

/*fonction ecriture des moteurs avec pwm*/
void writeMotors(void){
    for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        pwmWriteMotor(i, motorsThrottle[i]);
    if (feature(FEATURE_ONESHOT125))
        pwmCompleteOneshotMotorUpdate(MAX_SUPPORTED_MOTORS);
}

/*fonction affectation de la vitesse moteur*/
void writeAllMotors(int16_t mc){
    // Sends commands to all motorsThrottle
    for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++)
        motorsThrottle[i] = mc;
    writeMotors();
}

/*fonction arret des moteurs - affectation vitesse au minimum*/
void stopMotors(void){
    writeAllMotors(motorAndServoConfig()->mincommand);
    delay(50); // give the timers and ESCs a chance to react.
}

/*fonction arret signaux pwm pour les moteurs */
void StopPwmAllMotors(){
    pwmShutdownPulsesForAllMotors(MAX_SUPPORTED_MOTORS);
}

void mixTable(void){
    // type de phase de vol
    uint8_t phaseDeVol       = getPhaseDeVol();

    /*Disarmed motors*/
    if (!ARMING_FLAG(ARMED)) {
        for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
            motorsThrottle[i] = motorDisarmed[i];
        }
    }
    /* Armed motors */
    else{
        // indicateur "failsafe"
        bool    isFailsafeActive = failsafeIsActive();

        if (phaseDeVol == VOL_QUAD){
            if( (mixerConfigVTOL.yaw_jump_prevention_limit < YAW_JUMP_PREVENTION_LIMIT_HIGH)) {
                // prevent "yaw jump" during yaw correction
                int16_t axisPIDMax =   mixerConfigVTOL.yaw_jump_prevention_limit
                                     + ABS(rcCommand[YAW]);
                int16_t axisPIDMin = - mixerConfigVTOL.yaw_jump_prevention_limit
                                     - ABS(rcCommand[YAW]);
                //borne entre min et max
                axisPID[FD_YAW] = constrain(axisPID[FD_YAW], axisPIDMin, axisPIDMax);
            }

            //Code repris de boxairmode
            //Initial mixerVTOL concept by bdoiron74 reused and optimized for Air Mode
            int16_t rollPitchYawMix[MAX_SUPPORTED_MOTORS];
            int16_t rollPitchYawMixMax = 0; // assumption: symetrical about zero.
            int16_t rollPitchYawMixMin = 0;

            // Find roll/pitch/yaw desired output
            for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
              rollPitchYawMix[i] =   (axisPID[FD_PITCH] * motorMixerVTOL[i].pitch)  //pitch contrbution
                                   + (axisPID[FD_ROLL]  * motorMixerVTOL[i].roll)   //roll conribution
                                   - (axisPID[FD_YAW]   * motorMixerVTOL[i].yaw)    //yaw contrbution
                                      * mixerConfigVTOL.yaw_motor_direction ;       //yaw direction
              // Valeur max commandee
              if (rollPitchYawMix[i] > rollPitchYawMixMax)
                rollPitchYawMixMax = rollPitchYawMix[i];
              // Valeur min commandee
              if (rollPitchYawMix[i] < rollPitchYawMixMin)
                rollPitchYawMixMin = rollPitchYawMix[i];
            }

            // Scale roll/pitch/yaw uniformly to fit within throttle range
            int16_t rollPitchYawMixRange = rollPitchYawMixMax - rollPitchYawMixMin;

            // Find min and max throttle based on condition. Use rcData for 3D to prevent loss of power due to min_check
            int16_t throttleMin   = motorAndServoConfig()->minthrottle;
            int16_t throttleMax   = motorAndServoConfig()->maxthrottle;
            int16_t throttleRange = throttleMax - throttleMin;

            //test si poussee demandee hors limite
            if (rollPitchYawMixRange > throttleRange) {
                // limite en poussee atteinte
                motorLimitReached = true;
                //facteur de reduction de poussee
                float mixReduction = (float) (throttleRange / rollPitchYawMixRange);
                //reduction poussee
                for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
                    rollPitchYawMix[i] =  lrintf((float) (rollPitchYawMix[i] * mixReduction));
                }
                // Get the maximum correction by setting throttle offset to center.
                throttleMin = throttleMax = throttleMin + (throttleRange / 2);
            }
            else {
                //poussee limite non atteinte
                motorLimitReached = false;
                //poussee min et max
                throttleMin = throttleMin + (rollPitchYawMixRange / 2);
                throttleMax = throttleMax - (rollPitchYawMixRange / 2);
            }

            // Now add in the desired throttle, but keep in a range that doesn't clip adjusted
            // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
            for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {

                //commabde en poussee de moteur
                if (isFailsafeActive) {
                    //en failsafe : pousse failsafe
                    motorsThrottle[i] = mixConstrainMotorForFailsafeCondition(i);
                }
                else {
                    motorsThrottle[i] = rcCommand[THROTTLE] * motorMixerVTOL[i].throttle;
                    motorsThrottle[i] = constrain(motorsThrottle[i], throttleMin, throttleMax);
                    motorsThrottle[i] = motorsThrottle[i] + rollPitchYawMix[i];
                    motorsThrottle[i] = constrain(motorsThrottle[i], motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);
                }
            }
        }
        else{
            //Now add in the desired throttle, but keep in a range that doesn't clip adjusted
            // roll/pitch/yaw. This could move throttle down, but also up for those low throttle flips.
            for (uint32_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
                //commabde en poussee de moteur
                if (isFailsafeActive) {
                    //en failsafe : pousse failsafe
                    motorsThrottle[i] = mixConstrainMotorForFailsafeCondition(i);
                }
                else {
                    motorsThrottle[i] = rcCommand[THROTTLE] * motorMixerVTOL[i].throttle;
                    motorsThrottle[i] = constrain(motorsThrottle[i], motorAndServoConfig()->minthrottle, motorAndServoConfig()->maxthrottle);
                }
            }
        }
    }
    // motorsThrottle outputs are used as sources for servo mixing, so motorsThrottle must be calculated before servos.
    servoMixer(phaseDeVol);
}

