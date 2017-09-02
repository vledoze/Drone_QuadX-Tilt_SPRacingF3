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

#include <platform.h>

#include "debug.h"

#include "common/axis.h"

#include "config/parameter_group.h"

#include "drivers/system.h"

#include "rx/rx.h"

#include "io/beeper.h"
#include "io/motor_and_servo.h"
#include "io/rc_controls.h"

#include "config/parameter_group_ids.h"
#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_reset.h"

#include "flight/failsafe.h"

/*
 * Usage:
 *
 * failsafeInit() and failsafeSetConfig() must be called before the other methods are used.
 *
 * failsafeInit() and failsafeSetConfig() can be called in any order.
 * failsafeInit() should only be called once.
 *
 * enable() should be called after system initialisation.
 */

/* Variables ================================================================ */
/* Static ------------------------------------------------------------------- */
typedef struct STR_failsafe_state {
    int16_t events;
    bool monitoring;
    bool active;
    uint32_t rxDataFailurePeriod;
    uint32_t validRxDataReceivedAt;
    uint32_t validRxDataFailedAt;
    uint32_t throttleLowPeriod;             // throttle stick must have been below 'min_check' for this period
    uint32_t landingShouldBeFinishedAt;
    uint32_t receivingRxDataPeriod;         // period for the required period of valid rxData
    uint32_t receivingRxDataPeriodPreset;   // preset for the required period of valid rxData
    ENUM_failsafe_phase phase;
    ENUM_failsafe_RxLinkState rxLinkState;
} TYP_failsafe_state;
static TYP_failsafe_state failsafe_state;

/* Global ------------------------------------------------------------------- */
PG_REGISTER_WITH_RESET_TEMPLATE(TYP_failsafe_config, PG_failsafe_config, PG_FAILSAFE_CONFIG_ID, 0);
PG_RESET_TEMPLATE(TYP_failsafe_config, PG_failsafe_config,
    .failsafe_delay = 10,              // 1sec
    .failsafe_off_delay = 200,         // 20sec
    .failsafe_throttle = 1000,         // default throttle off.
    .failsafe_throttle_low_delay = 100, // default throttle low delay for "just disarm" on failsafe condition
);

/* Fonctions ================================================================ */
/* Static ------------------------------------------------------------------- */
static void failsafeReset(void)
{
    failsafe_state.rxDataFailurePeriod = PERIOD_RXDATA_FAILURE + PG_failsafe_config()->failsafe_delay * MILLIS_PER_TENTH_SECOND;
    failsafe_state.validRxDataReceivedAt = 0;
    failsafe_state.validRxDataFailedAt = 0;
    failsafe_state.throttleLowPeriod = 0;
    failsafe_state.landingShouldBeFinishedAt = 0;
    failsafe_state.receivingRxDataPeriod = 0;
    failsafe_state.receivingRxDataPeriodPreset = 0;
    failsafe_state.phase = FAILSAFE_PHASE___IDLE;
    failsafe_state.rxLinkState = FAILSAFE_RXLINKSTATE___DOWN;
}

static bool failsafeShouldHaveCausedLandingByNow(void)
{
    return (millis() > failsafe_state.landingShouldBeFinishedAt);
}

static void failsafeActivate(void)
{
    // Lorsque le failsafe est active on cherche a atterir
    failsafe_state.active = true;
    failsafe_state.phase = FAILSAFE_PHASE___LANDING;
    ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
    failsafe_state.landingShouldBeFinishedAt = millis() + PG_failsafe_config()->failsafe_off_delay * MILLIS_PER_TENTH_SECOND;
    failsafe_state.events++;
}

static void failsafeApplyControlInput(void)
{
    // Prise de control du drone
    // - Stabilisation
    for (int i = 0; i < 3; i++) {
        rcData[i] = rxConfig()->midrc;
    }
    // - Descente
    rcData[THROTTLE] = PG_failsafe_config()->failsafe_throttle;
}

/* Getter ------------------------------------------------------------------- */
ENUM_failsafe_phase failsafePhase(void)
{
    return failsafe_state.phase;
}

bool failsafeIsMonitoring(void)
{
    return failsafe_state.monitoring;
}

bool failsafeIsActive(void)
{
    return failsafe_state.active;
}

bool failsafeIsReceivingRxData(void)
{
    return (failsafe_state.rxLinkState == FAILSAFE_RXLINKSTATE___UP);
}

/* Corp --------------------------------------------------------------------- */
void failsafeSetConfig(void)
{
    failsafeReset();
}

void failsafeInit(void)
{
    failsafe_state.events = 0;
    failsafe_state.monitoring = false;
    return;
}

void failsafeStartMonitoring(void)
{
    failsafe_state.monitoring = true;
}

void failsafeOnRxSuspend(uint32_t usSuspendPeriod)
{
    failsafe_state.validRxDataReceivedAt += (usSuspendPeriod / 1000);    // / 1000 to convert micros to millis
}

void failsafeOnRxResume(void)
{
    failsafe_state.validRxDataReceivedAt = millis();                     // prevent RX link down trigger, restart rx link up
    failsafe_state.rxLinkState = FAILSAFE_RXLINKSTATE___UP;                     // do so while rx link is up
}

void failsafeOnValidDataReceived(void)
{
    failsafe_state.validRxDataReceivedAt = millis();
    if ((failsafe_state.validRxDataReceivedAt - failsafe_state.validRxDataFailedAt) > PERIOD_RXDATA_RECOVERY) {
        failsafe_state.rxLinkState = FAILSAFE_RXLINKSTATE___UP;
    }
}

void failsafeOnValidDataFailed(void)
{
    failsafe_state.validRxDataFailedAt = millis();
    if ((failsafe_state.validRxDataFailedAt - failsafe_state.validRxDataReceivedAt) > failsafe_state.rxDataFailurePeriod) {
        failsafe_state.rxLinkState = FAILSAFE_RXLINKSTATE___DOWN;
    }
}

void failsafeUpdateState(void)
{
    // Si pas de monitoring on sort
    if (!failsafe_state.monitoring) {
        return;
    }

    // Variables locales
    bool reprocessState;
    bool receivingRxData;
    bool armed;
    bool failsafeSwitchIsOn;
    beeperMode_e beeperMode;

    // On beep si le signal est perdu
    receivingRxData = failsafeIsReceivingRxData();
    beeperMode = BEEPER_SILENCE;;
    if (!receivingRxData) {
        beeperMode = BEEPER_RX_LOST;
    }

    // On boucle tant que reprocessState est vrai
    do {
        reprocessState = false;
        armed = ARMING_FLAG(ARMED);
        failsafeSwitchIsOn = rcModeIsActive(BOXFAILSAFE);;

        // Phases du failsafe
        switch (failsafe_state.phase) {

            // PHASE : Au repos
            case FAILSAFE_PHASE___IDLE:
                // Si le drone est armé
                if (armed) {
                    // Track throttle command below minimum time
                    if (THROTTLE_HIGH == calculateThrottleStatus(rxConfig())) {
                        failsafe_state.throttleLowPeriod = millis() + PG_failsafe_config()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                    }
                    // Kill switch logic (must be independent of receivingRxData to skip PERIOD_RXDATA_FAILURE delay before disarming)
                    if (failsafeSwitchIsOn && PG_failsafe_config()->failsafe_kill_switch) {
                        // KillswitchEvent: failsafe switch is configured as KILL switch and is switched ON
                        failsafeActivate();
                        failsafe_state.phase = FAILSAFE_PHASE___LANDED;      // skip auto-landing procedure
                        failsafe_state.receivingRxDataPeriodPreset = PERIOD_OF_1_SECONDS;    // require 1 seconds of valid rxData
                        reprocessState = true;
                    } else if (!receivingRxData) {
                        if (millis() > failsafe_state.throttleLowPeriod) {
                            // JustDisarm: throttle was LOW for at least 'failsafe_throttle_low_delay' seconds
                            failsafeActivate();
                            failsafe_state.phase = FAILSAFE_PHASE___LANDED;      // skip auto-landing procedure
                            failsafe_state.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // require 3 seconds of valid rxData
                        } else {
                            failsafe_state.phase = FAILSAFE_PHASE___RX_LOSS_DETECTED;
                        }
                        reprocessState = true;
                    }
                // Si le drone n'est pas arme
                } else {
                    // When NOT armed, show rxLinkState of failsafe switch in GUI (failsafe mode)
                    if (failsafeSwitchIsOn) {
                        ENABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    } else {
                        DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                    }
                    // Throttle low period expired (= low long enough for JustDisarm)
                    failsafe_state.throttleLowPeriod = 0;
                }
                break;

            // PHASE : Signal radio perdu
            case FAILSAFE_PHASE___RX_LOSS_DETECTED:
                // On recupere le signal
                if (receivingRxData) {
                    failsafe_state.phase = FAILSAFE_PHASE___RX_LOSS_RECOVERED;
                // On n'a pas recuperé le signal
                } else {
                    // Choix de la procedure d'atterissage
                    switch (PG_failsafe_config()->failsafe_procedure) {
                        default:
                        case FAILSAFE_PROCEDURE___AUTO_LANDING:
                            // Stabilize, and set Throttle to specified level
                            failsafeActivate();
                            break;
                        case FAILSAFE_PROCEDURE___DROP_IT:
                            // Drop the craft
                            failsafeActivate();
                            failsafe_state.phase = FAILSAFE_PHASE___LANDED;      // skip auto-landing procedure
                            failsafe_state.receivingRxDataPeriodPreset = PERIOD_OF_3_SECONDS; // require 3 seconds of valid rxData
                            break;
                    }
                }
                reprocessState = true;
                break;

            // PHASE : Atterissage du drone
            case FAILSAFE_PHASE___LANDING:
                // On recupere le signal
                if (receivingRxData) {
                    failsafe_state.phase = FAILSAFE_PHASE___RX_LOSS_RECOVERED;
                    reprocessState = true;
                }
                // Si le drone est arme - on prend le control
                if (armed) {
                    failsafeApplyControlInput();
                    beeperMode = BEEPER_RX_LOST_LANDING;
                }
                // Drone consideré comme etant au sol
                if (failsafeShouldHaveCausedLandingByNow() || !armed) {
                    failsafe_state.receivingRxDataPeriodPreset = PERIOD_OF_30_SECONDS; // require 30 seconds of valid rxData
                    failsafe_state.phase = FAILSAFE_PHASE___LANDED;
                    reprocessState = true;
                }
                break;

            // PHASE : Drone au sol
            case FAILSAFE_PHASE___LANDED:
                ENABLE_ARMING_FLAG(PREVENT_ARMING); // To prevent accidently rearming by an intermittent rx link
                mwDisarm();
                failsafe_state.receivingRxDataPeriod = millis() + failsafe_state.receivingRxDataPeriodPreset; // set required period of valid rxData
                failsafe_state.phase = FAILSAFE_PHASE___RX_LOSS_MONITORING;
                reprocessState = true;
                break;

            // PHASE : Dignostic etat de la liaison radio (autorisation de repartir seulement si liaison radio OK)
            case FAILSAFE_PHASE___RX_LOSS_MONITORING:
                // Monitoring the rx link to allow rearming when it has become good for > `receivingRxDataPeriodPreset` time.
                if (receivingRxData) {
                    if (millis() > failsafe_state.receivingRxDataPeriod) {
                        // rx link is good now, when arming via ARM switch, it must be OFF first
                        if (!(!isUsingSticksForArming() && rcModeIsActive(BOXARM))) {
                            DISABLE_ARMING_FLAG(PREVENT_ARMING);
                            failsafe_state.phase = FAILSAFE_PHASE___RX_LOSS_RECOVERED;
                            reprocessState = true;
                        }
                    }
                } else {
                    failsafe_state.receivingRxDataPeriod = millis() + failsafe_state.receivingRxDataPeriodPreset;
                }
                break;

            // PHASE : Liaison radio recuperee
            case FAILSAFE_PHASE___RX_LOSS_RECOVERED:
                // Entering IDLE with the requirement that throttle first must be at min_check for failsafe_throttle_low_delay period.
                // This is to prevent that JustDisarm is activated on the next iteration.
                // Because that would have the effect of shutting down failsafe handling on intermittent connections.
                failsafe_state.throttleLowPeriod = millis() + PG_failsafe_config()->failsafe_throttle_low_delay * MILLIS_PER_TENTH_SECOND;
                failsafe_state.phase = FAILSAFE_PHASE___IDLE;
                failsafe_state.active = false;
                DISABLE_FLIGHT_MODE(FAILSAFE_MODE);
                reprocessState = true;
                break;

            default:
                break;
        }
    } while (reprocessState);

    if (beeperMode != BEEPER_SILENCE) {
        beeper(beeperMode);
    }
}
