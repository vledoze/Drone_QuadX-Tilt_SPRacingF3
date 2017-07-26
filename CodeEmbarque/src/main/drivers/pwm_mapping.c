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
#include <stdlib.h>

#include <platform.h>

#include "config/parameter_group.h"

#include "gpio.h"
#include "timer.h"
#include "drivers/bus_i2c.h"

#include "pwm_output.h"
#include "pwm_rx.h"
#include "pwm_mapping.h"

#include "serial_uart_stm32f30x.h"

void pwmBrushedMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse);
void pwmBrushlessMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex, uint16_t motorPwmRate, uint16_t idlePulse);
void pwmOneshotMotorConfig(const timerHardware_t *timerHardware, uint8_t motorIndex);
void pwmServoConfig(const timerHardware_t *timerHardware, uint8_t servoIndex, uint16_t servoPwmRate, uint16_t servoCenterPulse);

/*
    Configuration maps

    Note: this documentation is only valid for STM32F10x, for STM32F30x please read the code itself.

    1) multirotor PPM input
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    2) multirotor PPM input with more servos
    PWM1 used for PPM
    PWM5..8 used for motors
    PWM9..10 used for servo or else motors
    PWM11..14 used for servos

    2) multirotor PWM input
    PWM1..8 used for input
    PWM9..10 used for servo or else motors
    PWM11..14 used for motors

    3) airplane / flying wing w/PWM
    PWM1..8 used for input
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos

    4) airplane / flying wing with PPM
    PWM1 used for PPM
    PWM5..8 used for servos
    PWM9 used for motor throttle +PWM10 for 2nd motor
    PWM11.14 used for servos
*/

enum {
    MAP_TO_PPM_INPUT = 1,
    MAP_TO_PWM_INPUT,
    MAP_TO_MOTOR_OUTPUT,
    MAP_TO_SERVO_OUTPUT,
};

static const uint16_t multiPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT    << 8), // PPM input

    PWM9  | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM10 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM13 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM14 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM15 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM16 | (MAP_TO_MOTOR_OUTPUT << 8),
    PWM5  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM6  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM7  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    PWM8  | (MAP_TO_MOTOR_OUTPUT << 8),      // Swap to servo if needed
    0xFFFF
};

static const uint16_t multiPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),    //ENTREE RC 
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), //MOTORS
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM11 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM12 | (MAP_TO_MOTOR_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8), //SERVOS
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8),
    0xFFFF
};

static const uint16_t airPPM[] = {
    PWM1  | (MAP_TO_PPM_INPUT << 8),     // PPM input
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM5  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM6  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM7  | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM8  | (MAP_TO_SERVO_OUTPUT  << 8), // servo #10
    0xFFFF
};

static const uint16_t airPWM[] = {
    PWM1  | (MAP_TO_PWM_INPUT << 8),     // input #1
    PWM2  | (MAP_TO_PWM_INPUT << 8),
    PWM3  | (MAP_TO_PWM_INPUT << 8),
    PWM4  | (MAP_TO_PWM_INPUT << 8),
    PWM5  | (MAP_TO_PWM_INPUT << 8),
    PWM6  | (MAP_TO_PWM_INPUT << 8),
    PWM7  | (MAP_TO_PWM_INPUT << 8),
    PWM8  | (MAP_TO_PWM_INPUT << 8),     // input #8
    PWM9  | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #1
    PWM10 | (MAP_TO_MOTOR_OUTPUT  << 8), // motor #2
    PWM11 | (MAP_TO_SERVO_OUTPUT  << 8), // servo #1
    PWM12 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM13 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM14 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM15 | (MAP_TO_SERVO_OUTPUT  << 8),
    PWM16 | (MAP_TO_SERVO_OUTPUT  << 8), // server #6
    0xFFFF
};

static const uint16_t * const hardwareMaps[] = {
    multiPWM,
    multiPPM,
    airPWM,
    airPPM,
};

static pwmIOConfiguration_t pwmIOConfiguration;

pwmIOConfiguration_t *pwmGetOutputConfiguration(void){
    return &pwmIOConfiguration;
}

pwmIOConfiguration_t *pwmInit(drv_pwm_config_t *init)
{
    int i = 0;
    const uint16_t *setup;

    int channelIndex = 0;


    memset(&pwmIOConfiguration, 0, sizeof(pwmIOConfiguration));

    setup = hardwareMaps[i];

    for (i = 0; i < USABLE_TIMER_CHANNEL_COUNT && setup[i] != 0xFFFF; i++) {
        uint8_t timerIndex = setup[i] & 0x00FF;
        uint8_t type = (setup[i] & 0xFF00) >> 8;

        const timerHardware_t *timerHardwarePtr = &timerHardware[timerIndex];

#ifdef OLIMEXINO_UNCUT_LED2_E_JUMPER
        // PWM2 is connected to LED2 on the board and cannot be connected unless you cut LED2_E
        if (timerIndex == PWM2)
            continue;
#endif

        // skip UART ports
#ifdef USE_UART2
        if (init->useUART2 && timerHardwarePtr->gpio == UART2_GPIO && (timerHardwarePtr->pin == UART2_TX_PIN || timerHardwarePtr->pin == UART2_RX_PIN))
            continue;
#endif

#ifdef USE_UART3
        if (init->useUART3 && timerHardwarePtr->gpio == UART3_GPIO && (timerHardwarePtr->pin == UART3_TX_PIN || timerHardwarePtr->pin == UART3_RX_PIN))
            continue;
#endif

#ifdef USE_UART4
        if (init->useUART4 && timerHardwarePtr->gpio == UART4_GPIO && (timerHardwarePtr->pin == UART4_TX_PIN || timerHardwarePtr->pin == UART4_RX_PIN))
            continue;
#endif

#ifdef USE_UART5
        if (init->useUART5 &&
            (
                (timerHardwarePtr->gpio == UART5_GPIO_TX && timerHardwarePtr->pin == UART5_TX_PIN)
                || (timerHardwarePtr->gpio == UART5_GPIO_RX && timerHardwarePtr->pin == UART5_RX_PIN)
            )
        )
            continue;
#endif

#ifdef SOFTSERIAL_1_TIMER
        if (init->useSoftSerial && timerHardwarePtr->tim == SOFTSERIAL_1_TIMER)
            continue;
#endif
#ifdef SOFTSERIAL_2_TIMER
        if (init->useSoftSerial && timerHardwarePtr->tim == SOFTSERIAL_2_TIMER)
            continue;
#endif

#ifdef LED_STRIP_TIMER
        // skip LED Strip output
        if (init->useLEDStrip) {
            if (timerHardwarePtr->tim == LED_STRIP_TIMER)
                continue;
#if defined(WS2811_GPIO) && defined(WS2811_PIN_SOURCE)
            if (timerHardwarePtr->gpio == WS2811_GPIO && timerHardwarePtr->gpioPinSource == WS2811_PIN_SOURCE)
                continue;
#endif
        }

#endif

#ifdef VBAT_ADC_GPIO
        if (init->useVbat && timerHardwarePtr->gpio == VBAT_ADC_GPIO && timerHardwarePtr->pin == VBAT_ADC_GPIO_PIN) {
            continue;
        }
#endif

#ifdef RSSI_ADC_GPIO
        if (init->useRSSIADC && timerHardwarePtr->gpio == RSSI_ADC_GPIO && timerHardwarePtr->pin == RSSI_ADC_GPIO_PIN) {
            continue;
        }
#endif

#ifdef CURRENT_METER_ADC_GPIO
        if (init->useCurrentMeterADC && timerHardwarePtr->gpio == CURRENT_METER_ADC_GPIO && timerHardwarePtr->pin == CURRENT_METER_ADC_GPIO_PIN) {
            continue;
        }
#endif

#ifdef SONAR
        if (init->sonarGPIOConfig && timerHardwarePtr->gpio == init->sonarGPIOConfig->gpio &&
            (
                timerHardwarePtr->pin == init->sonarGPIOConfig->triggerPin ||
                timerHardwarePtr->pin == init->sonarGPIOConfig->echoPin
            )
        ) {
            continue;
        }
#endif

        if (type == MAP_TO_PWM_INPUT && !init->useParallelPWM)
            continue;
        if (type == MAP_TO_PPM_INPUT && !init->usePPM)
            continue;

        if (type == MAP_TO_PPM_INPUT) {
            ppmInConfig(timerHardwarePtr);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PPM;
            pwmIOConfiguration.ppmInputCount++;
        } else if (type == MAP_TO_PWM_INPUT) {
            pwmInConfig(timerHardwarePtr, channelIndex);
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_PWM;
            pwmIOConfiguration.pwmInputCount++;
            channelIndex++;
        } else if (type == MAP_TO_MOTOR_OUTPUT) {
            if (init->useOneshot) {

                pwmOneshotMotorConfig(timerHardwarePtr, pwmIOConfiguration.motorCount);
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_ONESHOT|PWM_PF_OUTPUT_PROTOCOL_PWM;

            } else if (isMotorBrushed(init->motorPwmRate)) {

                pwmBrushedMotorConfig(timerHardwarePtr, pwmIOConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_MOTOR_MODE_BRUSHED | PWM_PF_OUTPUT_PROTOCOL_PWM;

            } else {

                pwmBrushlessMotorConfig(timerHardwarePtr, pwmIOConfiguration.motorCount, init->motorPwmRate, init->idlePulse);
                pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_MOTOR | PWM_PF_OUTPUT_PROTOCOL_PWM ;
            }

            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.motorCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;

            pwmIOConfiguration.motorCount++;

        } else if (type == MAP_TO_SERVO_OUTPUT) {
            pwmServoConfig(timerHardwarePtr, pwmIOConfiguration.servoCount, init->servoPwmRate, init->servoCenterPulse);

            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].flags = PWM_PF_SERVO | PWM_PF_OUTPUT_PROTOCOL_PWM;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].index = pwmIOConfiguration.servoCount;
            pwmIOConfiguration.ioConfigurations[pwmIOConfiguration.ioCount].timerHardware = timerHardwarePtr;

            pwmIOConfiguration.servoCount++;
        } else {
            continue;
        }

        pwmIOConfiguration.ioCount++;
    }

    return &pwmIOConfiguration;
}
