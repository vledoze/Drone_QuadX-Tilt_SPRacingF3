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

#define MAX_SUPPORTED_MOTORS            4
#define YAW_JUMP_PREVENTION_LIMIT_LOW   80
#define YAW_JUMP_PREVENTION_LIMIT_HIGH  500
#define CHANNEL_FORWARDING_DISABLED     (uint8_t)0xFF

#define AUX_QUAD                        100
#define AUX_AVION                       1000

//Enumeration ----------------------------------------------------------
// type de vol
typedef enum {
    VOL_QUAD = 0,
    VOL_AVION,
    VOL_TRANS
} typeVol_e;

//Structures -----------------------------------------------------------
// Custom mixer data per motor
typedef struct motorMixer_s{
    float throttle;
    float roll;
    float pitch;
    float yaw;
} motorMixer_t;

typedef struct mixerConfig_s {
    int8_t   yaw_motor_direction;
    int8_t   servo_lowpass_enable;        // enable/disable lowpass filter
    uint8_t  pid_at_min_throttle;         // when enabled pids are used at minimum throttle
    uint8_t  tri_unarmed_servo;           // send tail servo correction pulses even when unarmed
    uint16_t yaw_jump_prevention_limit;   // make limit configurable (original fixed value was 100)
    float    servo_lowpass_freq;          // lowpass servo filter frequency selection; 1/1000ths of loop freq
} mixerConfig_t;

// Variables locales ---------------------------------------------------
extern motorMixer_t  motorMixerVTOL[MAX_SUPPORTED_MOTORS];
extern mixerConfig_t mixerConfigVTOL;

extern int16_t       motorsThrottle[MAX_SUPPORTED_MOTORS];
extern int16_t       motor_disarmed[MAX_SUPPORTED_MOTORS];
extern bool          motorLimitReached;

// Fonctions -----------------------------------------------------------
void initMixer(void);
void mixerResetDisarmedMotors(void);

void writeMotors(void);
void writeAllMotors(int16_t mc);
void stopMotors(void);
void StopPwmAllMotors(void);

void mixTable(void);
