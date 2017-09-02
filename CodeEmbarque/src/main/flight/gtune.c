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
#include <string.h>
#include <math.h>

#include <platform.h>

#ifdef GTUNE

#include "build_config.h"

#include "common/axis.h"
#include "common/maths.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/feature.h"
#include "config/config_reset.h"

#include "drivers/system.h"
#include "drivers/sensor.h"
#include "drivers/accgyro.h"

#include "sensors/sensors.h"
#include "sensors/gyro.h"
#include "sensors/acceleration.h"

#include "flight/pid.h"
#include "flight/imu.h"

#include "blackbox/blackbox.h"

#include "io/rc_controls.h"

#include "gtune.h"

extern uint16_t cycleTime;

/*
****************************************************************************
***                    G_Tune                                            ***
****************************************************************************
G_Tune Mode
This is the multiwii implementation of ZERO-PID Algorithm
http://technicaladventure.blogspot.com/2014/06/zero-pids-tuner-for-multirotors.html
The algorithm has been originally developed by Mohammad Hefny (mohammad.hefny@gmail.com)

You may use/modify this algorithm on your own risk, kindly refer to above link in any future distribution.
*/

/*
version 1.0.0: MIN & Maxis & Tuned Band
version 1.0.1:
            a. error is gyro reading not rc - gyro.
            b. gtune_errorOld = Error no averaging.
            c. No Min Maxis BOUNDRY
version 1.0.2:
            a. no boundaries
            b. I - Factor tune.
            c. gtune_timeSkip

Crashpilot: Reduced to just P tuning in a predefined range - so it is not "zero pid" anymore.
Tuning is limited to just work when stick is centered besides that YAW is tuned in non Acro as well.
See also:
http://diydrones.com/profiles/blogs/zero-pid-tunes-for-multirotors-part-2
http://www.multiwii.com/forum/viewtopic.php?f=8&t=5190
Gyrosetting 2000DPS
GyroScale = (1 / 16,4 ) * RADX(see board.h) = 0,001064225154 digit per rad/s

pidProfile->gtune_lolimP[ROLL]   = 10;  [0..200] Lower limit of ROLL P during G tune.
pidProfile->gtune_lolimP[PITCH]  = 10;  [0..200] Lower limit of PITCH P during G tune.
pidProfile->gtune_lolimP[YAW]    = 10;  [0..200] Lower limit of YAW P during G tune.
pidProfile->gtune_hilimP[ROLL]   = 100; [0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axisis.
pidProfile->gtune_hilimP[PITCH]  = 100; [0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axisis.
pidProfile->gtune_hilimP[YAW]    = 100; [0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axisis.
pidProfile->gtune_pwr            = 0;   [0..10] Strength of adjustment
pidProfile->gtune_settle_time    = 450; [200..1000] Settle time in ms
pidProfile->gtune_average_cycles = 16;  [8..128] Number of looptime cycles used for gyro average calculation
*/

/* Variables ================================================================ */
/* Static ------------------------------------------------------------------- */
static int16_t gtune_delayCycles;
static int16_t gtune_timeSkip[3];
static int16_t gtune_errorOld[3];
static int16_t gtune_resultP64[3];
static int32_t gtune_avgGyro[3];
static bool gtune_floatPID;

/* Global ------------------------------------------------------------------- */
PG_REGISTER_PROFILE_WITH_RESET_TEMPLATE(TYP_gtune_config, PG_gtune_config, PG_GTUNE_CONFIG_ID, 0);
PG_RESET_TEMPLATE(TYP_gtune_config, PG_gtune_config,
    .gtune_lolimP[FD_ROLL] = 10,          // [0..200] Lower limit of ROLL P during G tune.
    .gtune_lolimP[FD_PITCH] = 10,         // [0..200] Lower limit of PITCH P during G tune.
    .gtune_lolimP[FD_YAW] = 10,           // [0..200] Lower limit of YAW P during G tune.
    .gtune_hilimP[FD_ROLL] = 100,         // [0..200] Higher limit of ROLL P during G tune. 0 Disables tuning for that axis.
    .gtune_hilimP[FD_PITCH] = 100,        // [0..200] Higher limit of PITCH P during G tune. 0 Disables tuning for that axis.
    .gtune_hilimP[FD_YAW] = 100,          // [0..200] Higher limit of YAW P during G tune. 0 Disables tuning for that axis.
    .gtune_pwr = 0,                       // [0..10] Strength of adjustment
    .gtune_settle_time = 450,             // [200..1000] Settle time in ms
    .gtune_average_cycles = 16,           // [8..128] Number of looptime cycles used for gyro average calculation
);

/* Fonctions ================================================================ */
/* Static ------------------------------------------------------------------- */
static void gtuneUpdateDelayCycles(void)
{
    gtune_delayCycles = -(((int32_t)PG_gtune_config()->gtune_settle_time * 1000) / cycleTime);
}

/* Corp --------------------------------------------------------------------- */
void gtuneInit(void)
{
    if (pidProfile()->pidController == PID_CONTROLLER_LUX_FLOAT) {
        gtune_floatPID = true;       // LuxFloat is using float values for PID settings
    } else {
        gtune_floatPID = false;
    }

    gtuneUpdateDelayCycles();
    for (uint8_t i = 0; i < 3; i++) {
        // Protection sur la valeur limite haute
        if ( (PG_gtune_config()->gtune_hilimP[i])
           &&(PG_gtune_config()->gtune_lolimP[i] > PG_gtune_config()->gtune_hilimP[i]) ){
            // User config error disable axisis for tuning
            // Disable YAW tuning for everything below a quadcopter
            PG_gtune_config()->gtune_hilimP[i] = 0;
        }
        // On borne la valeur de P a la valeur min
        if (pidProfile()->P8[i] < PG_gtune_config()->gtune_lolimP[i]) {
            pidProfile()->P8[i] = PG_gtune_config()->gtune_lolimP[i];
        }
        // 6 bit extra resolution for P.
        gtune_resultP64[i] = (int16_t)pidProfile()->P8[i] << 6;
        gtune_errorOld[i] = 0;
        gtune_timeSkip[i] = gtune_delayCycles;
    }
}

void gtuneCalculate(uint8_t axis)
{
    int16_t error;
    int16_t diff_G;
    int16_t threshP;

    if ( (rcCommand[axis])               /* Block tuning on stick input */
      || ( (axis != FD_YAW)              /* Always allow G-Tune on YAW */
        && ( (FLIGHT_MODE(ANGLE_MODE))   /* G-Tune on ROLL & PITCH only in acromode */
          || (FLIGHT_MODE(HORIZON_MODE)) ) ) ){
        gtune_errorOld[axis] = 0;
        // Some settle time after stick center. default 450ms
        gtune_timeSkip[axis] = gtune_delayCycles;
    } else {
        // Premier passage
        if (!gtune_timeSkip[axis]) gtune_avgGyro[axis] = 0;
        // Incrementation
        gtune_timeSkip[axis]++;
        // Passage suivants
        if (gtune_timeSkip[axis] > 0) {
            // On moyenne les mesures gyro
            if (axis == FD_YAW) {
                gtune_avgGyro[axis] += 32 * ((int16_t)gyroADC[axis] / 32);           // sur 32 mesures en YAW
            } else {
                gtune_avgGyro[axis] += 128 * ((int16_t)gyroADC[axis] / 128);         // sur 128 mesures en ROLL & PITCH
            }
        }
        // Dernier passage
        if (gtune_timeSkip[axis] == PG_gtune_config()->gtune_average_cycles) {  // Looptime cycles for gyro average calculation. default 16.
            gtune_avgGyro[axis] /= gtune_timeSkip[axis];   // gtune_avgGyro[axis] has now very clean gyrodata
            gtune_timeSkip[axis] = 0;                      // RAZ compteur
            if (axis == FD_YAW) {
                threshP = 20;    //Palier en P
                error = -gtune_avgGyro[axis];
            } else {
                threshP = 10;    //Palier en P
                error = gtune_avgGyro[axis];
            }
            // Don't run when not needed or pointless to do so
            if ( (PG_gtune_config()->gtune_hilimP[axis])
              && (error)
              && (gtune_errorOld[axis])
              && (error != gtune_errorOld[axis]) ) {
                // Evolution de l'erreur
                diff_G = ABS(error) - ABS(gtune_errorOld[axis]);
                // Si l'erreur a gardé le meme signe
                if ( ((error > 0) && (gtune_errorOld[axis] > 0))
                  || ((error < 0) && (gtune_errorOld[axis] < 0))) {
                    // Si l'evolution de l'erreur est superieur au palier en P
                    if (diff_G > threshP) {
                        // On augmente P
                        if (axis == FD_YAW) {
                            // YAW ends up at low limit on float PID, give it some more to work with.
                            gtune_resultP64[axis] += 256 + PG_gtune_config()->gtune_pwr;
                        } else {
                            // Shift balance a little on the plus side.
                            gtune_resultP64[axis] += 64 + PG_gtune_config()->gtune_pwr;
                        }
                    // Si l'evolution de l'erreur est superieur en valeur absolue au palier en P
                    } else if (diff_G < -threshP) {
                        // On diminue P
                        if (axis == FD_YAW) {
                            gtune_resultP64[axis] -= 64 + PG_gtune_config()->gtune_pwr;
                        } else {
                            gtune_resultP64[axis] -= 32;
                        }
                    }
                // Si l'erreur a changé de signe
                } else {
                    // On diminue pour les axes ROLL & PITCH si la variation d'erreur est
                    // Superieur en valeur absolu au palier en P
                    if ((ABS(diff_G) > threshP) && (axis != FD_YAW)) {
                        gtune_resultP64[axis] -= 32;    // Don't use antiwobble for YAW
                    }
                }
                // On retourne a la resolution originale
                // On contraint le resultat entre les valeurs min et max de P
                // On caste en uint8_t
                // On enregistre la nouvelle valeur de reglage pour le PID
                pidProfile()->P8[axis] =
                    (uint8_t) constrain(
                        (gtune_resultP64[axis] >> 6),
                        (int16_t)PG_gtune_config()->gtune_lolimP[axis],
                        (int16_t)PG_gtune_config()->gtune_hilimP[axis]);
                // Enregistrement de la modification dans la Blackbox
                #ifdef BLACKBOX
                    if (feature(FEATURE_BLACKBOX)) {
                        flightLogEvent_gtuneCycleResult_t eventData;
                        eventData.gtuneAxis = axis;
                        eventData.gtuneGyroAVG = gtune_avgGyro[axis];
                        // for float PID the logged P value is still mutiplyed by 10
                        eventData.gtuneNewP = pidProfile()->P8[axis];
                        blackboxLogEvent(FLIGHT_LOG_EVENT_GTUNE_RESULT, (flightLogEventData_t*)&eventData);
                    }
                #endif
            }
            // On garde l'erreur en memoire
            gtune_errorOld[axis] = error;
        }
    }
}
#endif
