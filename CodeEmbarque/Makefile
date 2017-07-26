###############################################################################
# "THE BEER-WARE LICENSE" (Revision 42):
# <msmith@FreeBSD.ORG> wrote this file. As long as you retain this notice you
# can do whatever you want with this stuff. If we meet some day, and you think
# this stuff is worth it, you can buy me a beer in return
###############################################################################
#
# Makefile for building the cleanflight firmware.
#
# Invoke this with 'make help' to see the list of supported targets.
#

###############################################################################
# Things that the user might override on the commandline
#

# Compile-time options
OPTIONS = FAIL_ON_WARNINGS

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE ?= $(firstword $(wildcard /dev/ttyUSB*) no-port-found)

# Configure default flash sizes for the targets
FLASH_SIZE ?= 256

###############################################################################
# Things that need to be maintained as the source changes
###############################################################################
FORKNAME  = master
REVISION := $(shell git log -1 --format="%h")

# Working directories
ROOT := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))

# USB Drivers for SPRACINGF3
USBPERIPH_DIR = $(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC = $(notdir $(wildcard $(USBPERIPH_DIR)/src/*.c))

# STD Drivers for SPRACINGF3
STDPERIPH_DIR = $(ROOT)/lib/main/STM32F30x_StdPeriph_Driver
STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

# CMSIS for SPRACINGF3
# Note : CMSIS - Cortex Microcontroller Software Interface Standard
CMSIS_DIR = $(ROOT)/lib/main/CMSIS
CMSIS_SRC = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

# Systems files for SPRACINGF3
SYS_DIR = $(ROOT)/src/main/target/SPRACINGF3
SYS_SRC = $(notdir $(wildcard $(SYS_DIR)/*.c))

# Search Path for Sources files
SRC_DIR = $(ROOT)/src/main
VPATH := \
$(SRC_DIR):\
$(SRC_DIR)/startup:\
$(CMSIS_DIR)/CM1/CoreSupport:\
$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x:\
$(SYS_DIR):\
$(STDPERIPH_DIR)/src

#Sources files
STARTUP_STM32F30x_SRC = \
startup_stm32f30x_md_gcc.S

DRIVERS_SRC = \
drivers/adc.c \
drivers/adc_stm32f30x.c \
drivers/bus_i2c_stm32f30x.c \
drivers/bus_spi.c \
drivers/display_ug2864hsweg01.h \
drivers/gpio_stm32f30x.c \
drivers/light_led_stm32f30x.c \
drivers/pwm_mapping.c \
drivers/pwm_output.c \
drivers/pwm_rx.c \
drivers/serial_uart.c \
drivers/serial_uart_stm32f30x.c \
drivers/sound_beeper_stm32f30x.c \
drivers/system_stm32f30x.c \
drivers/timer.c \
drivers/timer_stm32f30x.c \
drivers/accgyro_mpu.c \
drivers/accgyro_mpu6050.c \
drivers/barometer_ms5611.c \
drivers/compass_ak8975.c \
drivers/compass_hmc5883l.c \
drivers/display_ug2864hsweg01.h \
drivers/flash_m25p16.c \
drivers/light_ws2811strip.c \
drivers/light_ws2811strip_stm32f30x.c \
drivers/serial_softserial.c \
drivers/sonar_hcsr04.c 

FLASH_SRC = \
io/flashfs.c  

INIT_SRC = \
flight/gtune.c \
flight/navigation.c \
flight/gps_conversion.c \
common/colorconversion.c \
io/gps.c \
io/ledstrip.c \
io/display.c 

TELEMETRY_SRC = \
telemetry/telemetry.c \
telemetry/frsky.c \
telemetry/hott.c \
telemetry/smartport.c \
telemetry/ltm.c \
telemetry/mavlink.c \
sensors/sonar.c \
sensors/barometer.c

BLACKBOX_SRC = \
blackbox/blackbox.c \
blackbox/blackbox_io.c

BUILD_SRC = \
build_config.c \
debug.c \
version.c 

CONFIG_SRC = \
config/config.c \
config/runtime_config.c \
config/config_streamer.c \
config/config_eeprom.c \
config/parameter_group.c \
config/feature.c \
config/profile.c

COMMON_SRC = \
common/maths.c \
common/printf.c \
common/typeconversion.c \
common/encoding.c \
common/filter.c \
common/streambuf.c 

MAIN_SRC = \
scheduler.c \
scheduler_tasks.c \
main.c \
mw.c 

FLIGHT_SRC = \
flight/altitudehold.c \
flight/failsafe.c \
flight/pid.c \
flight/pid_luxfloat.c \
flight/pid_mwrewrite.c \
flight/pid_mw23.c \
flight/imu.c \
flight/mixer.c \
flight/servos.c \
drivers/bus_i2c_soft.c \
drivers/serial.c \
drivers/sound_beeper.c \
drivers/system.c \
drivers/dma.c \
drivers/buf_writer.c \
drivers/gyro_sync.c 

IO_SRC = \
io/beeper.c \
io/gimbal.c \
io/motor_and_servo.c \
io/rate_profile.c \
io/rc_adjustments.c \
io/rc_controls.c \
io/rc_curves.c \
io/serial.c \
io/serial_4way.c \
io/serial_4way_avrootloader.c \
io/serial_4way_stk500v2.c \
io/serial_cli.c \
io/serial_msp.c \
io/statusindicator.c \
io/msp.c \

RX_SRC = \
rx/rx.c \
rx/pwm.c \
rx/msp.c \
rx/sbus.c \
rx/sumd.c \
rx/sumh.c \
rx/spektrum.c \
rx/xbus.c \
rx/ibus.c

SENSORS_SRC = \
sensors/sensors.c \
sensors/acceleration.c \
sensors/battery.c \
sensors/boardalignment.c \
sensors/compass.c \
sensors/gyro.c \
sensors/initialisation.c 

SPRACINGF3_SRC = \
$(STARTUP_STM32F30x_SRC) \
$(DRIVERS_SRC) \
$(FLASH_SRC) \
$(INIT_SRC) \
$(TELEMETRY_SRC) \
$(BLACKBOX_SRC) \
$(BUILD_SRC) \
$(SYS_SRC) \
$(CONFIG_SRC) \
$(COMMON_SRC) \
$(MAIN_SRC) \
$(FLIGHT_SRC) \
$(IO_SRC) \
$(RX_SRC) \
$(SENSORS_SRC) \
$(CMSIS_SRC) \
$(STDPERIPH_SRC) 

# Includes directories
INCLUDE_DIRS := \
$(STDPERIPH_DIR)/inc \
$(CMSIS_DIR)/CM1/CoreSupport \
$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x \
$(SYS_DIR) \
$(SRC_DIR)

# Load script for SPRACINGF3
LINKER_DIR = $(ROOT)/src/main/target
LD_SCRIPT = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC = arm-none-eabi-gcc
OBJCOPY = arm-none-eabi-objcopy
SIZE = arm-none-eabi-size

# Tool options.
ifeq ($(DEBUG),GDB)
	OPTIMIZE = -O0
	LTO_FLAGS = $(OPTIMIZE)
else
	OPTIMIZE = -Os
	LTO_FLAGS = -flto -fuse-linker-plugin $(OPTIMIZE)
endif

# Flags
ARCH_FLAGS = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
ifneq ($(filter $(OPTIONS),FAIL_ON_WARNINGS),)
	WARN_FLAGS += -Werror
endif
DEBUG_FLAGS = -ggdb3 -DDEBUG
DEVICE_FLAGS = -DSTM32F303xC -DSTM32F303 -DFLASH_SIZE=$(FLASH_SIZE) -DSPRACINGF3

# Compilation Flags
CFLAGS = \
$(ARCH_FLAGS) \
$(LTO_FLAGS) \
$(WARN_FLAGS) \
$(addprefix -D,$(OPTIONS)) \
$(addprefix -I,$(INCLUDE_DIRS)) \
$(DEBUG_FLAGS) \
-std=gnu99 \
-Wall \
-Wextra \
-Wunsafe-loop-optimizations \
-Wdouble-promotion \
-Wundef \
-ffunction-sections \
-fdata-sections \
$(DEVICE_FLAGS) \
-DUSE_STDPERIPH_DRIVER \
-D'__FORKNAME__="$(FORKNAME)"' \
-D'__TARGET__="SPRACINGF3"' \
-D'__REVISION__="$(REVISION)"' \
-fverbose-asm \
-ffat-lto-objects \
-save-temps=obj \
-MMD \
-MP

# Assembly flags
ASFLAGS = \
$(ARCH_FLAGS) \
$(WARN_FLAGS) \
-x assembler-with-cpp \
$(addprefix -I,$(INCLUDE_DIRS)) \
-MMD -MP

# Link flags
LDFLAGS = \
-lm \
-nostartfiles \
--specs=nano.specs \
-lc \
-lnosys \
$(ARCH_FLAGS) \
$(LTO_FLAGS) \
$(WARN_FLAGS) \
$(DEBUG_FLAGS) \
-static \
-Wl,-gc-sections,-Map,$(MAP) \
-Wl,-L$(LINKER_DIR) \
-Wl,--cref \
-T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

#Sources
CSOURCES := $(shell find $(SRC_DIR) -name '*.c')

#Check
CPPCHECK_OPT = --enable=all --platform=unix64 --std=c99 --inline-suppr --quiet --force
CPPCHECK_INC = $(addprefix -I,$(INCLUDE_DIRS)) -I/usr/include -I/usr/include/linux
CPPCHECK = cppcheck $(CSOURCES) $(CPPCHECK_OPT) $(CPPCHECK_INC)

# Things we will build
OBJ_DIR = $(ROOT)/obj/main
BIN_DIR = $(ROOT)/obj
BIN = $(BIN_DIR)/$(FORKNAME)_SPRACINGF3.bin
HEX = $(BIN_DIR)/$(FORKNAME)_SPRACINGF3.hex
ELF = $(OBJ_DIR)/$(FORKNAME)_SPRACINGF3.elf
OBJS = $(addsuffix .o,$(addprefix $(OBJ_DIR)/SPRACINGF3/,$(basename $(SPRACINGF3_SRC))))
DEPS = $(addsuffix .d,$(addprefix $(OBJ_DIR)/SPRACINGF3/,$(basename $(SPRACINGF3_SRC))))
MAP = $(OBJ_DIR)/$(FORKNAME)_SPRACINGF3.map

## Default make goal:
## hex         : Make filetype hex only
.DEFAULT_GOAL := hex

## Optional make goals:
## all         : Make all filetypes, binary and hex
all: hex bin

## binary      : Make binary filetype
## bin         : Alias of 'binary'
## hex         : Make hex filetype
bin:    $(BIN)
binary: $(BIN)
hex:    $(HEX)

## clean       : clean up all temporary / machine-generated files
clean:
	rm -f $(BIN) $(HEX) $(ELF) $(OBJS) $(MAP)
	rm -rf $(OBJ_DIR)/SPRACINGF3

## flash       : flash firmware (.hex) onto flight controller
flash: $(HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## st-flash    : flash firmware (.bin) onto flight controller
st-flash: $(BIN)
	st-flash --reset write $< 0x08000000

## unbrick     : unbrick flight controller
unbrick: $(HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## cppcheck    : run static analysis on C source code
cppcheck: $(CSOURCES)
	$(CPPCHECK)

cppcheck-result.xml: $(CSOURCES)
	$(CPPCHECK) --xml-version=2 2> cppcheck-result.xml

## help        : print this help message and exit
help: Makefile
	@echo ""
	@echo "Makefile for the $(FORKNAME)_$(SPRACINGF3) firmware"
	@echo ""
	@sed -n 's/^## //p' $<

## test        : run the cleanflight test suite (not implemented yet)
## junittest   : run the cleanflight test suite, producing Junit XML result files. (not implemented yet)
# test junittest:  (TODO les sources tests n'existent pas sur Git)
#      cd src/test && $(MAKE) $@

# rebuild everything when makefile changes
$(OBJS) : Makefile

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.
$(HEX): $(ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(BIN): $(ELF)
	$(OBJCOPY) -O binary $< $@

$(ELF):  $(OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(ELF)

# Compile
$(OBJ_DIR)/SPRACINGF3/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJ_DIR)/SPRACINGF3/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

$(OBJ_DIR)/SPRACINGF3/%.o: %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

# include auto-generated dependencies
-include $(DEPS)
