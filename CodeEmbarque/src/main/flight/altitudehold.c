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
#include <math.h>


#include <platform.h>

#include "debug.h"

#include "common/maths.h"
#include "common/axis.h"

#include "config/runtime_config.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/sensor.h"
#include "drivers/accgyro.h"
#include "drivers/sonar_hcsr04.h"

#include "sensors/sensors.h"
#include "sensors/acceleration.h"
#include "sensors/barometer.h"
#include "sensors/sonar.h"

#include "rx/rx.h"

#include "io/rc_controls.h"
#include "io/motor_and_servo.h"

#include "flight/mixer.h"
#include "flight/pid.h"
#include "flight/imu.h"

#include "flight/altitudehold.h"

/* Constantes ======================================================= */
#define BARO_UPDATE_FREQUENCY_40HZ (1000 * 25) // 40hz update rate (20hz LPF on acc)
#define DEGREES_80_IN_DECIDEGREES 800
#define THROTTLE_2_VZ   0.5                   //velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
#define P_ALT_QUANTUM 128     // eq. 2^7
#define P_VEL_QUANTUM 32      // eq. 2^5
#define I_VEL_QUANTUM 8192    // eq. 2^15
#define D_VEL_QUANTUM 512     // eq. 2^9


/* Variables ======================================================== */
/* Static ----------------------------------------------------------- */
static uint8_t altitude_altCommandedChanged = 0;
static int16_t altitude_rcDataThrottleHold = 0;
static int16_t altitude_rcCommandThrottleHold = 0;
static int32_t altitude_rcCommandThrottleAdjustment = 0;
static int32_t altitude_sumErrVel = 0;
static int32_t altitude_vzCommand = 0;              // in cm

/* Global ----------------------------------------------------------- */
int32_t G_altitude_altEst = 0;
int32_t G_altitude_altCmd = 0;                   // in cm
int32_t G_altitude_vzEst = 0;             // variometer in cm/s


/* Fonctions ======================================================== */
/* Static ----------------------------------------------------------- */

/* Fonction    : altitude_isThrustFacingDownwards
 * Description : Verifie que le drone est relativement a plat
                    roll in [-80deg; 80deg]
                    pitch in [-80deg; 80deg]
 * Entrees     :
        angles d'euler du drone (attitude)
 * Sorties     :
        drone à plat (OUI: true, NON: false)
*/
static bool altitude_isThrustFacingDownwards(attitudeEulerAngles_t *attitude)
{
    return (    (ABS(attitude->values.roll) < DEGREES_80_IN_DECIDEGREES)
             && (ABS(attitude->values.pitch) < DEGREES_80_IN_DECIDEGREES) );
}

/* Fonction    : altitudeCalculateThrottleAdjustement
 * Description : Calcul la variation de commande a envoyer pour garder l'altitude
 * Entrees     :
        Vitesse verticale (G_altitude_vzEst)
        Acceleration verticale (acczMesIMU)
        Acceleration verticale precedemment utilisee (acc_old)
 * Sorties     :
        Variation de commande THROTTLE (altitude_rcCommandThrottleAdjustment)
 */
static void altitudeCalculateThrottleAdjustement(float acczMesIMU, float acczMesIMUOld)
{
    int32_t altError;
    int32_t vzError;
    int32_t vzCommand;

    if (!altitude_isThrustFacingDownwards(&attitude)) {
        altitude_rcCommandThrottleAdjustment = 0;
    }
    else
    {
        // Altitude P-Controller
        if (!altitude_vzCommand) {
            altError = constrain(G_altitude_altCmd - G_altitude_altEst, -500, 500);
            altError = applyDeadband(altError, 10); // remove small P parameter to reduce noise near zero position
            vzCommand = constrain((pidProfile()->P8[PIDALT] * altError / P_ALT_QUANTUM), -300, +300); // limit velocity to +/- 3 m/s
        } else {
            vzCommand = altitude_vzCommand;
        }
        // Velocity PID-Controller
        // P
        vzError = vzCommand - G_altitude_vzEst;
        altitude_rcCommandThrottleAdjustment = constrain((pidProfile()->P8[PIDVEL] * vzError / D_VEL_QUANTUM), -300, +300);
        // I
        altitude_sumErrVel += (pidProfile()->I8[PIDVEL] * vzError);
        altitude_sumErrVel = constrain(altitude_sumErrVel, -(I_VEL_QUANTUM * 200), (I_VEL_QUANTUM * 200));
        altitude_rcCommandThrottleAdjustment += altitude_sumErrVel / I_VEL_QUANTUM;     // I in range +/-200
        // D
        altitude_rcCommandThrottleAdjustment -= constrain(pidProfile()->D8[PIDVEL] * (acczMesIMU + acczMesIMUOld) / D_VEL_QUANTUM, -150, 150);
    }
}

/* Corp ------------------------------------------------------------- */

/* Fonction    : altitudeUpdateBaroAltHoldState
 * Description : TODO
 * Entrees     :
        TODO
 * Sorties     :
        TODO
*/
void altitudeUpdateBaroAltHoldState(void)
{
    // Baro alt hold activate
    if (!rcModeIsActive(BOXBARO)) {
        DISABLE_FLIGHT_MODE(BARO_MODE);
        return;
    }

    if (!FLIGHT_MODE(BARO_MODE)) {
        ENABLE_FLIGHT_MODE(BARO_MODE);
        G_altitude_altCmd = G_altitude_altEst;
        altitude_rcDataThrottleHold = rcData[THROTTLE];
        altitude_rcCommandThrottleHold = rcCommand[THROTTLE];
        altitude_sumErrVel = 0;
        altitude_rcCommandThrottleAdjustment = 0;
    }
}

/* Fonction    : altitudeUpdateSonarAltHoldState
 * Description : TODO
 * Entrees     :
        TODO
 * Sorties     :
        TODO
*/
void altitudeUpdateSonarAltHoldState(void)
{
    // Sonar alt hold activate
    if (!rcModeIsActive(BOXSONAR)) {
        DISABLE_FLIGHT_MODE(SONAR_MODE);
        return;
    }

    if (!FLIGHT_MODE(SONAR_MODE)) {
        ENABLE_FLIGHT_MODE(SONAR_MODE);
        G_altitude_altCmd = G_altitude_altEst;
        altitude_rcDataThrottleHold = rcData[THROTTLE];
        altitude_rcCommandThrottleHold = rcCommand[THROTTLE];
        altitude_sumErrVel = 0;
        altitude_rcCommandThrottleAdjustment = 0;
    }
}

/* Fonction    : altitudeApplyAltHold
 * Description : Pilotage du drone pour garder une altitude
 * Entrees     :
        TODO
 * Sorties     :
        Commande THROTTLE (rcCommand[THROTTLE])
 */
void altitudeApplyAltHold(void)
{
    // rapid alt changes
    if (rcControlsConfig()->alt_hold_fast_change) {
        if (ABS(rcData[THROTTLE] - altitude_rcDataThrottleHold) > rcControlsConfig()->alt_hold_deadband) {
            altitude_sumErrVel = 0;
            rcCommand[THROTTLE] += (rcData[THROTTLE] > altitude_rcDataThrottleHold) ?
                -rcControlsConfig()->alt_hold_deadband :
                 rcControlsConfig()->alt_hold_deadband;
        } else {
            G_altitude_altCmd = G_altitude_altEst;
            rcCommand[THROTTLE] = constrain(
                altitude_rcCommandThrottleHold + altitude_rcCommandThrottleAdjustment,
                motorAndServoConfig()->minthrottle,
                motorAndServoConfig()->maxthrottle);
        }
    // slow alt changes, mostly used for aerial photography
    } else {
        if (ABS(rcData[THROTTLE] - altitude_rcDataThrottleHold) > rcControlsConfig()->alt_hold_deadband) {
            // set velocity proportional to stick movement +100 throttle gives ~ +50 cm/s
            altitude_vzCommand = (rcData[THROTTLE] - altitude_rcDataThrottleHold) * THROTTLE_2_VZ;
        } else if (altitude_altCommandedChanged) {
            altitude_vzCommand = 0;
            G_altitude_altCmd = G_altitude_altEst;
        }
        rcCommand[THROTTLE] = constrain(
            altitude_rcCommandThrottleHold + altitude_rcCommandThrottleAdjustment,
            motorAndServoConfig()->minthrottle,
            motorAndServoConfig()->maxthrottle);
    }
}

/* Fonction    : altitudeCalculate
 * Description : Calcul l'altitude estimée
 * Entrees     : TODO
        G_imu_accSum (imu.c)
 * Sorties     : TODO
*/
void altitudeCalculate(uint32_t time)
{
    float acczMesIMU;
    float sonarTransition;
    float vzEstBaro;
    float vzEstIMU;
    float dtIMU;
    uint32_t dtBaro;

    static float altEstBaro = 0.0f;
    static float altEstIMU = 0.0f;
    static float sumVzEstIMU = 0.0f;
    static float acczMesIMUOld = 0.0f;
    static int32_t altMesBaroOld = 0;
    static int32_t baroAltOffset = 0;  //Ecart de mesure sur le barometre / sonar
    static uint32_t timeOld = 0;

    /* On verifie que le barometre a eu le temps de faire une nouvelle mesure */
    dtBaro = time - timeOld;
    if (dtBaro < BARO_UPDATE_FREQUENCY_40HZ)
        return;
    timeOld = time;

    /* Infos issues de l'IMU */
    // Acceleration IMU
    if (abs(accSumCount) > 0) {
        // Acceleration normee suivant z
        acczMesIMU = (float)G_imu_accSum[2] / (float)accSumCount;
    } else {
        // Acceleration nulle suivant z : protection div0
        acczMesIMU = 0;
    }

    /* Infos issu du barometre */
    // Calibration
    if (!isBaroCalibrationComplete()) {
        performBaroCalibrationCycle();
        sumVzEstIMU = 0.0f;
        altEstBaro = 0.0f;
    }

    // Calcul de G_baro_altMes : mesure du barometre
    baroCalculateAltitude();

    /* Infos issues du sonar */
    // Calcul de G_sonar_altMes : mesure du sonar
    sonarCalculateAltitude(getCosTiltAngle());

    /* Traitements */
    // Recalage des mesures du barometre avec les mesure du sonar
    if (G_sonar_altMes > 0 && G_sonar_altMes < sonarCfAltCm) {
        //On ecrase la mesure du barometre avec le sonar
        baroAltOffset = G_baro_altMes - G_sonar_altMes;
        G_baro_altMes = G_sonar_altMes;    //FIXME : variable externe -> barometre
    } else {
        //On compense la mesure du barometre
        G_baro_altMes -= baroAltOffset;
        if (G_sonar_altMes > 0  && G_sonar_altMes <= sonarMaxAltWithTiltCm) {
            // SONAR in range, so use complementary filter
            sonarTransition = (float)(sonarMaxAltWithTiltCm - G_sonar_altMes)
                                   / (sonarMaxAltWithTiltCm - sonarCfAltCm);
            G_baro_altMes += sonarTransition * (G_sonar_altMes - G_baro_altMes);
            //G_baro_altMes = G_sonar_altMes * sonarTransition + G_baro_altMes * (1.0f - sonarTransition);
        }
    }

    // vitesse estimee IMU : cm/sec (integration de l'acceleration)
    vzEstIMU = acczMesIMU * accVelScale * (float)accTimeSum;   //TODO recherche unite de temps

    // Estimation de l'Altitude in cm
    dtIMU = accTimeSum * 1e-6f; // delta acc reading time in seconds
    altEstIMU += (vzEstIMU * 0.5f) * dtIMU + sumVzEstIMU * dtIMU;      // integrate velocity to get distance (x= a/2 * t^2)
    altEstBaro = altEstIMU * barometerConfig()->baro_cf_alt + (float)G_baro_altMes * (1.0f - barometerConfig()->baro_cf_alt);    // complementary filter for altitude estimation (baro & acc)

    // Somme des vitesses verticales IMU (cm/s)
    // Permet de prendre en compte l'historique lors du calcul de altEstIMU
    sumVzEstIMU += vzEstIMU;

    //On remet a zero la somme des acceleration IMU
    imuResetAccelerationSum();

    //Si le barometre n'est toujours pas calibre : stop
    if (!isBaroCalibrationComplete()) {
        return;
    }

    //Vitesse verticale estimee a partir des mesures du barometre
    vzEstBaro = (G_baro_altMes - altMesBaroOld) * 1000000.0f / dtBaro;
    vzEstBaro = constrain(vzEstBaro, -1500, 1500);  // constrain baro velocity +/- 1500cm/s
    vzEstBaro = applyDeadband(vzEstBaro, 10);       // to reduce noise near zero

    /* Mise a jour des infos a transmettre au drone */
    // Altitude G_altitude_altEst
    if (G_sonar_altMes > 0 && G_sonar_altMes < sonarCfAltCm) {
        // the sonar has the best range
        G_altitude_altEst = G_sonar_altMes;
    } else {
        G_altitude_altEst = altEstBaro;
    }

    // Vitesse G_altitude_vzEst
    // apply Complimentary Filter to keep the calculated velocity based on baro velocity (i.e. near real velocity).
    // By using CF it's possible to correct the drift of integrated accZ (velocity) without loosing the phase, i.e without delay
    G_altitude_vzEst = lrintf(vzEstIMU * barometerConfig()->baro_cf_vel + vzEstBaro * (1.0f - barometerConfig()->baro_cf_vel));
    G_altitude_vzEst = applyDeadband(G_altitude_vzEst, 5);

    // Calcul de la variation de pousse a mettre pour tenir l'altitude
    altitudeCalculateThrottleAdjustement(acczMesIMU, acczMesIMUOld);

    //Enregistrement des mesures IMU et BARO
    altMesBaroOld = G_baro_altMes;
    acczMesIMUOld = acczMesIMU;
}
