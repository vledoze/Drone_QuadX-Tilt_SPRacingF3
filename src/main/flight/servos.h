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

#pragma once

//Ennumerator ----------------------------------------------------------
// These must be consecutive, see 'reversedSources'
enum {
    INPUT_STABILIZED_ROLL = 0,
    INPUT_STABILIZED_PITCH,
    INPUT_STABILIZED_YAW,
    INPUT_STABILIZED_THROTTLE,
    INPUT_RC_ROLL,
    INPUT_RC_PITCH,
    INPUT_RC_YAW,
    INPUT_RC_THROTTLE,
    INPUT_RC_AUX1,
    INPUT_RC_AUX2,
    INPUT_RC_AUX3,
    INPUT_RC_AUX4,
    INPUT_GIMBAL_PITCH,
    INPUT_GIMBAL_ROLL,
    INPUT_SOURCE_COUNT
} inputSource_e;

// target servo channels
typedef enum {
    SERVO_ELEVATOR  = 0,
    SERVO_FLAPPERON = 1,
    SERVO_RUDDER    = 2,
    SERVO_AUX1      = 3     // Servo pour la conversion phase de vol
} servoChannel_e;

//DEFINE ---------------------------------------------------------------
#define MAX_SUPPORTED_SERVOS  4
#define MAX_SERVO_RULES       4
#define MAX_SERVO_BOXES       3
#define SERVO_PLANE_INDEX_MIN SERVO_ELEVATOR
#define MAX_SERVO_SPEED       UINT8_MAX

//STRUCTURES -----------------------------------------------------------
typedef struct servoMixer_s {
    uint8_t targetChannel;  // servo that receives the output of the rule
    uint8_t inputSource;    // input channel for this rule
    int8_t  rate;           // range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
    uint8_t speed;          // reduces the speed of the rule, 0=unlimited speed
    int8_t  min;            // lower bound of rule range [0;100]% of servo max-min
    int8_t  max;            // lower bound of rule range [0;100]% of servo max-min
    uint8_t box;            // active rule if box is enabled, range [0;3], 0=no box, 1=BOXSERVO1, 2=BOXSERVO2, 3=BOXSERVO3
} servoMixer_t;

typedef struct servoParam_s {
    int16_t  min;                // servo min
    int16_t  max;                // servo max
    int16_t  middle;             // servo middle
    int8_t   rate;               // range [-125;+125] ; can be used to adjust a rate 0-125% and a direction
    uint8_t  angleAtMin;         // range [0;180] the measured angle in degrees from the middle when the servo is at the 'min' value.
    uint8_t  angleAtMax;         // range [0;180] the measured angle in degrees from the middle when the servo is at the 'max' value.
    int8_t   forwardFromChannel; // RX channel index, 0 based.  See CHANNEL_FORWARDING_DISABLED
    uint32_t reversedSources;    // the direction of servo movement for each input source of the servo mixer, bit set=inverted
} __attribute__ ((__packed__)) servoParam_t;

//DECLARATIONS ---------------------------------------------------------
struct motorAndServoConfig_s;
struct rxConfig_s;

extern servoParam_t servoConfVTOL[MAX_SUPPORTED_SERVOS];
extern servoMixer_t servoMixerVTOL[MAX_SUPPORTED_SERVOS];
extern int16_t      servoCmd[MAX_SUPPORTED_SERVOS];

//FONCTIONS ------------------------------------------------------------
void initServos(void);
void initServoFilter(uint32_t targetLooptime);

void servoMixer(uint8_t phaseDeVol);
void filterServos(void);
void writeServos(void);

int  servoDirection(int servoIndex, int fromChannel);
bool isMixerUsingServos(void);
