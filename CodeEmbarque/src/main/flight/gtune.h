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

#pragma once

#ifdef GTUNE
    // Variables
    typedef struct STR_gtune_config {
        uint8_t  gtune_lolimP[FD_INDEX_COUNT];  // [0..200] Lower limit of P during G tune
        uint8_t  gtune_hilimP[FD_INDEX_COUNT];  // [0..200] Higher limit of P during G tune. 0 Disables tuning for that axis.
        uint8_t  gtune_pwr;                     // [0..10] Strength of adjustment
        uint16_t gtune_settle_time;             // [200..1000] Settle time in ms
        uint8_t  gtune_average_cycles;          // [8..128] Number of looptime cycles used for gyro average calculation
    } TYP_gtune_config;
    PG_DECLARE_PROFILE(TYP_gtune_config, PG_gtune_config);

    // Corp
    void gtuneInit(void);
    void gtuneCalculate(uint8_t axis);
#endif
