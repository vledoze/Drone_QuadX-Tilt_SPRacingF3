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

#include <platform.h>

#include "build_config.h"
#include "debug.h"

#include "common/axis.h"
#include "common/color.h"
#include "common/atomic.h"
#include "common/maths.h"
#include "common/printf.h"
#include "common/streambuf.h"

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "drivers/nvic.h"

#include "drivers/sensor.h"
#include "drivers/system.h"
#include "drivers/dma.h"
#include "drivers/gpio.h"
#include "drivers/light_led.h"
#include "drivers/sound_beeper.h"
#include "drivers/timer.h"
#include "drivers/serial.h"
#include "drivers/serial_softserial.h"
#include "drivers/serial_uart.h"
#include "drivers/accgyro.h"
#include "drivers/compass.h"
#include "drivers/pwm_mapping.h"
#include "drivers/pwm_rx.h"
#include "drivers/adc.h"
#include "drivers/bus_i2c.h"
#include "drivers/bus_spi.h"
#include "drivers/inverter.h"
#include "drivers/flash_m25p16.h"
#include "drivers/sonar_hcsr04.h"
#include "drivers/sdcard.h"
#include "drivers/usb_io.h"
#include "drivers/transponder_ir.h"
#include "drivers/gyro_sync.h"

#include "rx/rx.h"
#include "rx/spektrum.h"

#include "io/serial.h"
#include "io/flashfs.h"
#include "io/gps.h"
#include "io/motor_and_servo.h"
#include "io/rc_controls.h"
#include "io/gimbal.h"
#include "io/ledstrip.h"
#include "io/display.h"
#include "io/asyncfatfs/asyncfatfs.h"
#include "io/transponder_ir.h"
#include "io/msp.h"
#include "io/serial_msp.h"
#include "io/serial_cli.h"

#include "sensors/sensors.h"
#include "sensors/sonar.h"
#include "sensors/barometer.h"
#include "sensors/compass.h"
#include "sensors/acceleration.h"
#include "sensors/gyro.h"
#include "sensors/battery.h"
#include "sensors/boardalignment.h"
#include "sensors/initialisation.h"

#include "telemetry/telemetry.h"
#include "blackbox/blackbox.h"

#include "flight/pid.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "flight/servos.h"
#include "flight/failsafe.h"
#include "flight/navigation.h"

#include "config/runtime_config.h"
#include "config/config.h"
#include "config/config_system.h"
#include "config/feature.h"

#ifdef USE_HARDWARE_REVISION_DETECTION
  #include "hardware_revision.h"
#endif

#include "scheduler.h"

//VARIABLES GLOBALES -----------------------------------------------------------
extern uint8_t motorControlEnable;

#ifdef SOFTSERIAL_LOOPBACK
    serialPort_t *loopbackPort;
#endif

//DECLARATION FONCTIONS --------------------------------------------------------
//Déclaration fonction initialisation RX
void rxInit(modeActivationCondition_t *modeActivationConditions);

//Déclaration fonction initialisation Navigation
void navigationInit(pidProfile_t *pidProfile);

//Déclaration fonction initialisation Sonar
const sonarHardware_t *sonarGetHardwareConfiguration(currentSensor_e  currentMeterType);
void  sonarInit(const sonarHardware_t *sonarHardware);

//Déclaration fonction initialisation horloge interne
#ifdef STM32F303xC
    // from system_stm32f30x.c
    void SetSysClock(void);
#endif //STM32F303xC

#ifdef STM32F10X
    // from system_stm32f10x.c
    void SetSysClock(bool overclock);
#endif //STM32F10X

//Déclaration fonctions initialisation Systeme
// -- Fonctions initialisations pour un type "systemConfig_t"
// -- (REGISTER+RESET, REGISTER, RESET)
PG_REGISTER_WITH_RESET_TEMPLATE(systemConfig_t, systemConfig, PG_SYSTEM_CONFIG, 0);
PG_REGISTER(pwmRxConfig_t, pwmRxConfig, PG_DRIVER_PWM_RX_CONFIG, 0);
PG_RESET_TEMPLATE(systemConfig_t, systemConfig,
    .i2c_highspeed = 1,
);
// -- Etats possibles de la variable "systemState"
typedef enum {
    SYSTEM_STATE_INITIALISING        = 0,
    SYSTEM_STATE_CONFIG_LOADED       = (1 << 0),
    SYSTEM_STATE_SENSORS_READY       = (1 << 1),
    SYSTEM_STATE_MOTORS_READY        = (1 << 2),
    SYSTEM_STATE_TRANSPONDER_ENABLED = (1 << 3),
    SYSTEM_STATE_READY               = (1 << 7)
} systemState_e;

//VARIABLES LOCALES ------------------------------------------------------------
// -- Initialisation de la variable "systemeState"
static uint8_t systemState = SYSTEM_STATE_INITIALISING;

//IMPLEMENTATION FONCTIONS -----------------------------------------------------
//Fonction    : flashLedsAndBeep
//Description : -
void flashLedsAndBeep(void)
{
    LED1_ON;
    LED0_OFF;
    for (uint8_t i = 0; i < 10; i++) {
        LED1_TOGGLE;
        LED0_TOGGLE;
        delay(25);
        BEEP_ON;
        delay(25);
        BEEP_OFF;
    }
    LED0_OFF;
    LED1_OFF;
}

#ifdef BUTTONS
    //Fonction    : buttonsInit
    //Description : initialisation Bouttons (GPIO)
    void buttonsInit(void)
    {
        gpio_config_t buttonAGpioConfig = {
            BUTTON_A_PIN,
            Mode_IPU,
            Speed_2MHz
        };
        gpioInit(BUTTON_A_PORT, &buttonAGpioConfig);

        gpio_config_t buttonBGpioConfig = {
            BUTTON_B_PIN,
            Mode_IPU,
            Speed_2MHz
        };
        gpioInit(BUTTON_B_PORT, &buttonBGpioConfig);

        delayMicroseconds(10);  // allow GPIO configuration to settle
    }

    //Fonction    : buttonsHandleColdBootButtonPresses
    //Description : fonction prise en compte appui sur bouton "Boot"
    void buttonsHandleColdBootButtonPresses(void)
    {
        uint8_t secondsRemaining = 10;
        bool bothButtonsHeld;
        do {
            bothButtonsHeld = !digitalIn(BUTTON_A_PORT, BUTTON_A_PIN) && !digitalIn(BUTTON_B_PORT, BUTTON_B_PIN);
            if (bothButtonsHeld) {
                if (--secondsRemaining == 0) {
                    resetEEPROM();
                    systemReset();
                }

                if (secondsRemaining > 5) {
                    delay(1000);
                } else {
                    // flash quicker after a few seconds
                    delay(500);
                    LED0_TOGGLE;
                    delay(500);
                }
                LED0_TOGGLE;
            }
        } while (bothButtonsHeld);

        // buttons released between 5 and 10 seconds
        if (secondsRemaining < 5) {
            usbGenerateDisconnectPulse();
            flashLedsAndBeep();
            systemResetToBootloader();
        }
    }
#endif //BUTTONS

//Fonction    : init
//Description : Initialisation globale materiel + systeme
void init(void)
{
    //Parametres PWM
    drv_pwm_config_t pwm_params;

    //Print ? TODO quoi
    printfSupportInit();

    //Initialisation EEPROM : TODO c'est quoi EEPROM ?
    initEEPROM();
    ensureEEPROMContainsValidData();
    readEEPROM();

    //TODO a quoi ca sert ??
    systemState |= SYSTEM_STATE_CONFIG_LOADED;

    //TODO quoi ?
    #ifdef STM32F303
        SCB->CPACR = (0x3 << (10*2)) | (0x3 << (11*2));  // start fpu
    #endif // STM32F10X

    //Mise en place horloge systeme
    #ifdef STM32F303xC
        SetSysClock();
    #endif //STM32F303xC
    #ifdef STM32F10X
        // Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers
        // Configure the Flash Latency cycles and enable prefetch buffer
        SetSysClock(systemConfig()->emf_avoidance);
    #endif //STM32F10X
    //Mise en place over-clock horloge systeme
    i2cSetOverclock(systemConfig()->i2c_highspeed);

    //Initialisation du système
    systemInit();

    //Detection de la version du materiel
    #ifdef USE_HARDWARE_REVISION_DETECTION
        detectHardwareRevision();
    #endif //USE_HARDWARE_REVISION_DETECTION

    // Latch active features to be used for feature() in the remainder of init().
    latchActiveFeatures();

    //Initialisation des LEDs
    #ifdef ALIENFLIGHTF3
        if (hardwareRevision == AFF3_REV_1) {
            ledInit(false);
        } else {
            ledInit(true);
        }
    #else
        ledInit(false);
    #endif //ALIENFLIGHTF3

    //Initialisation du BEEPER
    #ifdef BEEPER
        //Configuration BEEPER
        beeperConfig_t beeperConfig = {
            .gpioPeripheral = BEEP_PERIPHERAL,
            .gpioPin        = BEEP_PIN,
            .gpioPort       = BEEP_GPIO,
        #ifdef BEEPER_INVERTED
                .gpioMode        = Mode_Out_PP,
                .isInverted      = true
        #else
                .gpioMode        = Mode_Out_OD,
                .isInverted      = false
        #endif //BEEPER_INVERTED
        };
        //Forcage configuration BEEPER : cas spéciaux NAZE
        #ifdef NAZE
            if (hardwareRevision >= NAZE32_REV5) {
                // naze rev4 and below used opendrain to PNP for buzzer.
                // Rev5 and above use PP to NPN.
                beeperConfig.gpioMode   = Mode_Out_PP;
                beeperConfig.isInverted = true;
            }
        #endif //NAZE
        //Initialisation BEEPER avec la configuration choisie
        beeperInit(&beeperConfig);
    #endif //BEEPER

    //Initialisation des BUTTONS
    #ifdef BUTTONS
        //Initialisation
        buttonsInit();
        //Initialisation prise en compte appui sec sur bouton Boot
        if (!isMPUSoftReset()) {
            buttonsHandleColdBootButtonPresses();
        }
    #endif //BUTTONS

    //TODO : qu'est ce que c'est ???
    #ifdef SPEKTRUM_BIND
        if (feature(FEATURE_RX_SERIAL)) {
            switch (rxConfig()->serialrx_provider) {
                case SERIALRX_SPEKTRUM1024:
                case SERIALRX_SPEKTRUM2048:
                    // Spektrum satellite binding if enabled on startup.
                    // Must be called before that 100ms sleep so that we don't lose satellite's binding window after startup.
                    // The rest of Spektrum initialization will happen later - via spektrumInit()
                    spektrumBind(rxConfig());
                    break;
            }
        }
    #endif //SPEKTRUM_BIND

    //Delais pour s'assurer que les initialisation précédentes
    //ont eu le temps de se mettre en place
    delay(100);

    //Initialisation du Timer
    timerInit();  // timer must be initialized before any channel is allocated

    //Initialisation du DMA : TODO quoi ?
    dmaInit();

    //Initialisation serial : TODO quoi ?
    serialInit(feature(FEATURE_SOFTSERIAL));

    //Initialisation configuration moteur+servos
    initMixer();
    initServos();

    //Initilisation des paramètres PWM a 0
    //Préparation pour les initialisation des sorties analogique
    memset(&pwm_params, 0, sizeof(pwm_params));

    //Initialisation SONAR
    #ifdef SONAR
        const sonarHardware_t *sonarHardware = NULL;
        if (feature(FEATURE_SONAR)) {
            //Configuration signal SONAR
            sonarHardware = sonarGetHardwareConfiguration(batteryConfig()->currentMeterType);
            sonarGPIOConfig_t sonarGPIOConfig = {
                .gpio       = SONAR_GPIO,
                .triggerPin = sonarHardware->echo_pin,
                .echoPin    = sonarHardware->trigger_pin,
            };
            //configuration SONAR -> configuration PWM
            pwm_params.sonarGPIOConfig = &sonarGPIOConfig;
        }
    #endif

    //TODO quoi ?
    pwm_params.airplane = false;

    //configuration ports UART -> configuration PWM
    #if defined(USE_UART2) && defined(STM32F10X)
        pwm_params.useUART2 = doesConfigurationUsePort(SERIAL_PORT_UART2);
    #endif
    #if defined(USE_UART3)
        pwm_params.useUART3 = doesConfigurationUsePort(SERIAL_PORT_UART3);
    #endif
    #if defined(USE_UART4)
        pwm_params.useUART4 = doesConfigurationUsePort(SERIAL_PORT_UART4);
    #endif
    #if defined(USE_UART5)
        pwm_params.useUART5 = doesConfigurationUsePort(SERIAL_PORT_UART5);
    #endif

    //Fonctionnalités (entrée/sorties) supportees -> configuration PWM
    pwm_params.useVbat        = feature(FEATURE_VBAT);
    pwm_params.useSoftSerial  = feature(FEATURE_SOFTSERIAL);
    pwm_params.useParallelPWM = feature(FEATURE_RX_PARALLEL_PWM);
    pwm_params.useRSSIADC     = feature(FEATURE_RSSI_ADC);
    pwm_params.useCurrentMeterADC = (
        feature(FEATURE_CURRENT_METER)
        && batteryConfig()->currentMeterType == CURRENT_SENSOR_ADC
    );
    pwm_params.useLEDStrip    = feature(FEATURE_LED_STRIP);
    pwm_params.usePPM         = feature(FEATURE_RX_PPM);
    pwm_params.useSerialRx    = feature(FEATURE_RX_SERIAL);
    #ifdef SONAR
        pwm_params.useSonar = feature(FEATURE_SONAR);
    #endif

    //Signal servo-moteur -> configuration PWM
    pwm_params.useServos            = isMixerUsingServos();
    pwm_params.useChannelForwarding = feature(FEATURE_CHANNEL_FORWARDING);
    pwm_params.servoCenterPulse     = motorAndServoConfig()->servoCenterPulse;
    pwm_params.servoPwmRate         = motorAndServoConfig()->servo_pwm_rate;

    //Signal moteur -> configuration PWM
    pwm_params.useOneshot           = feature(FEATURE_ONESHOT125);
    pwm_params.motorPwmRate         = motorAndServoConfig()->motor_pwm_rate;
    pwm_params.idlePulse            = motorAndServoConfig()->mincommand;
    if (pwm_params.motorPwmRate > 500)
        pwm_params.idlePulse = 0; // brushed motors

    //TODO quoi ??
    pwmRxInit();

    //Initialisation PWM avec la configuration choisie
    pwmInit(&pwm_params); // pwmInit() needs to be called as soon as possible for ESC compatibility reasons

    //On envoi le signal PWM necessaire pour desarmer les moteurs (Securité)
    mixerResetDisarmedMotors();

    //Enregistrement signal PWM pour debogue
    #ifdef DEBUG_PWM_CONFIGURATION
        debug[2] = pwmIOConfiguration->pwmInputCount;
        debug[3] = pwmIOConfiguration->ppmInputCount;
    #endif //DEBUG_PWM_CONFIGURATION

    //On peut maintenant activer le controle des moteurs
    if (!feature(FEATURE_ONESHOT125)){
        motorControlEnable = true;
    }
    systemState |= SYSTEM_STATE_MOTORS_READY;

    //TODO quoi ??
    #ifdef INVERTER
        initInverter();
    #endif //INVERTER

    //Initialisation signal SPI
    #ifdef USE_SPI
        spiInit(SPI1);
        spiInit(SPI2);
        #ifdef STM32F303xC
            #ifdef ALIENFLIGHTF3
                if (hardwareRevision == AFF3_REV_2) {
                    spiInit(SPI3);
                }
            #else
                spiInit(SPI3);
            #endif //ALIENFLIGHTF3
        #endif //STM32F303xC
    #endif //USE_SPI

    //Mise à jour version materiel
    #ifdef USE_HARDWARE_REVISION_DETECTION
        updateHardwareRevision();
    #endif //USE_HARDWARE_REVISION_DETECTION

    //Cas particuliers : tri des ports inutilisés / version materiel
    #ifdef NAZE
        if (hardwareRevision == NAZE32_SP) {
            serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
        } else  {
            serialRemovePort(SERIAL_PORT_UART3);
        }
    #endif //NAZE
    #if defined(SPRACINGF3) && defined(SONAR) && defined(USE_SOFTSERIAL2)
        if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
            serialRemovePort(SERIAL_PORT_SOFTSERIAL2);
        }
    #endif //(SPRACINGF3)&&(SONAR)&&(USE_SOFTSERIAL2)
    #if defined(SPRACINGF3MINI) && defined(SONAR) && defined(USE_SOFTSERIAL1)
        if (feature(FEATURE_SONAR) && feature(FEATURE_SOFTSERIAL)) {
            serialRemovePort(SERIAL_PORT_SOFTSERIAL1);
        }
    #endif //(SPRACINGF3MINI)&&(SONAR)&&(USE_SOFTSERIAL2)

    //Initilisation signaux I2C
    #ifdef USE_I2C
        #if defined(NAZE)
            if (hardwareRevision != NAZE32_SP) {
                i2cInit(I2C_DEVICE);
            } else {
                if (!doesConfigurationUsePort(SERIAL_PORT_UART3)) {
                    i2cInit(I2C_DEVICE);
                }
            }
        #elif defined(CC3D)
            if (!doesConfigurationUsePort(SERIAL_PORT_UART3)) {
                i2cInit(I2C_DEVICE);
            }
        #else
            i2cInit(I2C_DEVICE);
        #endif //(NAZE)||(CC3D)
    #endif //USE_I2C

    //Initialisation signal ADC
    #ifdef USE_ADC
        //Parametres ADC
        drv_adc_config_t adc_params;
        adc_params.enableVBat = feature(FEATURE_VBAT);
        adc_params.enableRSSI = feature(FEATURE_RSSI_ADC);
        adc_params.enableCurrentMeter = feature(FEATURE_CURRENT_METER);
        adc_params.enableExternal1 = false;
        #ifdef OLIMEXINO
            adc_params.enableExternal1 = true;
        #endif //OLIMEXINO
        #ifdef NAZE
            // optional ADC5 input on rev.5 hardware
            adc_params.enableExternal1 = (hardwareRevision >= NAZE32_REV5);
        #endif //NAZE
        //Initialisation signal ADC avec le parametrage choisi
        adcInit(&adc_params);
    #endif //USE_ADC

    //Initialisation de l'alignement de la carte
    initBoardAlignment();

    //Initialisation Display : TODO quoi ?
    #ifdef DISPLAY
        if (feature(FEATURE_DISPLAY)) {
            displayInit();
        }
    #endif //DISPLAY

    //Configuration de l'echantillonage Gyrometre
    // -- Set gyro sampling rate divider before initialization
    gyroSetSampleRate(imuConfig()->looptime,
                      gyroConfig()->gyro_lpf,
                      imuConfig()->gyroSync,
                      imuConfig()->gyroSyncDenominator);

    //Verification présence IMU
    if (!sensorsAutodetect()) {
        // if gyro was not detected due to whatever reason, we give up now.
        failureMode(FAILURE_MISSING_ACC);
    }

    //OK initilisation Materiel
    systemState |= SYSTEM_STATE_SENSORS_READY;
    flashLedsAndBeep();

    //Initialisation des filtres appliqués aux Servo-moteurs
    initServoFilter(targetLooptime);

    //Initilisation magnetomètres
    #ifdef MAG
        if (sensors(SENSOR_MAG))
            compassInit();
    #endif

    //Initilisation IMU
    imuInit();

    //Initialisation interface MSP
    mspInit();
    mspSerialInit();

    //Initilisation interface CLI
    #ifdef USE_CLI
        cliInit();
    #endif

    //Initilisation du Fail-safe
    failsafeInit();

    //Initialisation RX : TODO pourquoi la ?
    rxInit(modeActivationProfile()->modeActivationConditions);

    //Initilisation NAVIGATION
    #ifdef GPS
        if (feature(FEATURE_GPS)) {
            //Initilisation GPS
            gpsInit();
            //Initilisation algos NAVIGATION : TODO Pourquoi ici ??
            navigationInit(pidProfile());
        }
    #endif //GPS
    #ifdef SONAR
        if (feature(FEATURE_SONAR)) {
            sonarInit(sonarHardware);
        }
    #endif //SONAR

    //Initilisation de la bande de LED
    #ifdef LED_STRIP
        ledStripInit();
        if (feature(FEATURE_LED_STRIP)) {
            ledStripEnable();
        }
    #endif //LED_STRIP

    //Initilisation de la TELEMETRIE
    #ifdef TELEMETRY
        if (feature(FEATURE_TELEMETRY)) {
            telemetryInit();
        }
    #endif //TELEMETRY

    //test : cable USB connecté ???
    #ifdef USB_CABLE_DETECTION
        usbCableDetectInit();
    #endif //USB_CABLE_DETECTION

    //Initilisation du transpondeur : TODO quoi ???
    #ifdef TRANSPONDER
        if (feature(FEATURE_TRANSPONDER)) {
            transponderInit(transponderConfig()->data);
            transponderEnable();
            transponderStartRepeating();
            systemState |= SYSTEM_STATE_TRANSPONDER_ENABLED;
        }
    #endif //TRANSPONDER

    //Initilisation FLASH Carte
    #ifdef USE_FLASHFS
        #ifdef NAZE
            if (hardwareRevision == NAZE32_REV5) {
                m25p16_init();
            }
        #elif defined(USE_FLASH_M25P16)
            m25p16_init();
        #endif //(NAZE)||(USE_FLASH_M25P16)
        flashfsInit();
    #endif

    //Initilisation lecture SD-CARD
    #ifdef USE_SDCARD
        bool sdcardUseDMA = false;
        sdcardInsertionDetectInit();
        #ifdef SDCARD_DMA_CHANNEL_TX
            #if defined(LED_STRIP) && defined(WS2811_DMA_CHANNEL)
                // Ensure the SPI Tx DMA doesn't overlap with the led strip
                sdcardUseDMA = !feature(FEATURE_LED_STRIP) || SDCARD_DMA_CHANNEL_TX != WS2811_DMA_CHANNEL;
            #else
                sdcardUseDMA = true;
            #endif //(LED_STRIP)&&(WS2811_DMA_CHANNEL)
        #endif //SDCARD_DMA_CHANNEL_TX
        sdcard_init(sdcardUseDMA);
        afatfs_init();
    #endif //SD-CARD

    //Initilisation boite-noire embarquée
    #ifdef BLACKBOX
        initBlackbox();
    #endif //BLACKBOX

    //Calibration IMU
    accSetCalibrationCycles(CALIBRATING_ACC_CYCLES);
    gyroSetCalibrationCycles(CALIBRATING_GYRO_CYCLES);

    //Calibration Barometre
    #ifdef BARO
        baroSetCalibrationCycles(CALIBRATING_BARO_CYCLES);
    #endif //BARO

    // start all timers
    // TODO - not implemented yet
    timerStart();

    ENABLE_STATE(SMALL_ANGLE);
    DISABLE_ARMING_FLAG(PREVENT_ARMING);

    //Boucle de retour ??? TODO quoi ??
    #ifdef SOFTSERIAL_LOOPBACK
        // FIXME this is a hack, perhaps add a FUNCTION_LOOPBACK to support it properly
        loopbackPort = (serialPort_t*)&(softSerialPorts[0]);
        if (!loopbackPort->vTable) {
            loopbackPort = openSoftSerial(0, NULL, 19200, SERIAL_NOT_INVERTED);
        }
        serialPrint(loopbackPort, "LOOPBACK\r\n");
    #endif //SOFTSERIAL_LOOPBACK

    // Now that everything has powered up the voltage and cell count be determined.

    //Initilisation suivi niveau de batterie
    if (feature(FEATURE_VBAT | FEATURE_CURRENT_METER)){
        batteryInit();
    }

    //Fixed page GPS pour debug ??? TODO quoi
    #ifdef DISPLAY
        if (feature(FEATURE_DISPLAY)) {
            #ifdef USE_OLED_GPS_DEBUG_PAGE_ONLY
                displayShowFixedPage(PAGE_GPS);
            #else
                displayResetPageCycling();
                displayEnablePageCycling();
            #endif //USE_OLED_GPS_DEBUG_PAGE_ONLY
        }
    #endif //DISPLAY

    //TODO quoi ???
    #ifdef CJMCU
        LED2_ON;
    #endif //CJMCU

    // Latch active features AGAIN since some may be modified by init().
    latchActiveFeatures();

    //OK - le systeme est prèt
    motorControlEnable = true;
    systemState |= SYSTEM_STATE_READY;
}

//Fonction    : processLoopback
//Description : Traitement de la boucle de retour ?? TODO quoi ???
#ifdef SOFTSERIAL_LOOPBACK
    void processLoopback(void) {
        if (loopbackPort) {
            uint8_t bytesWaiting;
            while ((bytesWaiting = serialRxBytesWaiting(loopbackPort))) {
                uint8_t b = serialRead(loopbackPort);
                serialWrite(loopbackPort, b);
            };
        }
    }
#else
    #define processLoopback()
#endif //SOFTSERIAL_LOOPBACK

//Fonction    : main
//Description : Fonction principale du programme
int main(void) {
    //Initialisation globale
    init();
    //Initilisation du Scheluder
    schedulerInit();
    //Creation et gestion des taches planififiées Scheluder
    setTaskEnabled(TASK_GYROPID, true);
    rescheduleTask(TASK_GYROPID, imuConfig()->gyroSync ? targetLooptime - INTERRUPT_WAIT_TIME : targetLooptime);
    setTaskEnabled(TASK_ACCEL, sensors(SENSOR_ACC));
    setTaskEnabled(TASK_SERIAL, true);
    #ifdef BEEPER
        setTaskEnabled(TASK_BEEPER, true);
    #endif //BEEPER
    setTaskEnabled(TASK_BATTERY, feature(FEATURE_VBAT) || feature(FEATURE_CURRENT_METER));
    setTaskEnabled(TASK_RX, true);
    #ifdef GPS
        setTaskEnabled(TASK_GPS, feature(FEATURE_GPS));
    #endif //GPS
    #ifdef MAG
        setTaskEnabled(TASK_COMPASS, sensors(SENSOR_MAG));
        #if defined(MPU6500_SPI_INSTANCE) && defined(USE_MAG_AK8963)
            // fixme temporary solution for AK6983 via slave I2C on MPU9250
            rescheduleTask(TASK_COMPASS, 1000000 / 40);
        #endif
    #endif //MAG
    #ifdef BARO
        setTaskEnabled(TASK_BARO, sensors(SENSOR_BARO));
    #endif //BARO
    #ifdef SONAR
        setTaskEnabled(TASK_SONAR, sensors(SENSOR_SONAR));
    #endif //SONAR
    #if defined(BARO) || defined(SONAR)
        setTaskEnabled(TASK_ALTITUDE, sensors(SENSOR_BARO) || sensors(SENSOR_SONAR));
    #endif //(BARO)||(SONAR)
    #ifdef DISPLAY
        setTaskEnabled(TASK_DISPLAY, feature(FEATURE_DISPLAY));
    #endif //DISPLAY
    #ifdef TELEMETRY
        setTaskEnabled(TASK_TELEMETRY, feature(FEATURE_TELEMETRY));
    #endif //TELEMETRY
    #ifdef LED_STRIP
        setTaskEnabled(TASK_LEDSTRIP, feature(FEATURE_LED_STRIP));
    #endif //LED_STRIP
    #ifdef TRANSPONDER
        setTaskEnabled(TASK_TRANSPONDER, feature(FEATURE_TRANSPONDER));
    #endif //TRANSPONDER

    //Boucle systeme
    // 1 - MAJ SCHELUDER
    // 2 - Boucle de retour
    while (true) {
        scheduler();
        processLoopback();
    }
}

//Fonction    : HardFault_Handler
//Description : Gestion des default au cours de l'utilisation ?? TODO
void HardFault_Handler(void)
{
    // fall out of the sky
    uint8_t requiredStateForMotors = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_MOTORS_READY;
    if ((systemState & requiredStateForMotors) == requiredStateForMotors) {
        stopMotors();
    }
    #ifdef TRANSPONDER
        // prevent IR LEDs from burning out.
        uint8_t requiredStateForTransponder = SYSTEM_STATE_CONFIG_LOADED | SYSTEM_STATE_TRANSPONDER_ENABLED;
        if ((systemState & requiredStateForTransponder) == requiredStateForTransponder) {
            transponderIrDisable();
        }
    #endif //TRANSPONDER

    //On se met en defaut - Attente sans rien faire
    while (1);
}
