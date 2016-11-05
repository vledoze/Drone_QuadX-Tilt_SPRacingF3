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

# The target to build, see VALID_TARGETS below
TARGET		= SPRACINGF3

# Compile-time options
OPTIONS		= FAIL_ON_WARNINGS
export OPTIONS

# Debugger optons, must be empty or GDB
DEBUG ?=

# Serial port/Device for flashing
SERIAL_DEVICE	?= $(firstword $(wildcard /dev/ttyUSB*) no-port-found)

# Configure default flash sizes for the targets
FLASH_SIZE = 256

###############################################################################
# Things that need to be maintained as the source changes
#

FORKNAME   = cleanflight

REVISION := $(shell git log -1 --format="%h")

# Working directories
ROOT		 := $(patsubst %/,%,$(dir $(lastword $(MAKEFILE_LIST))))
SRC_DIR		 = $(ROOT)/src/main
OBJECT_DIR	 = $(ROOT)/obj/main
BIN_DIR		 = $(ROOT)/obj
CMSIS_DIR	 = $(ROOT)/lib/main/CMSIS
INCLUDE_DIRS	 = $(SRC_DIR)
LINKER_DIR	 = $(ROOT)/src/main/target

# Search path for sources
VPATH		:= $(SRC_DIR):$(SRC_DIR)/startup
USBFS_DIR	= $(ROOT)/lib/main/STM32_USB-FS-Device_Driver
USBPERIPH_SRC = $(notdir $(wildcard $(USBFS_DIR)/src/*.c))

CSOURCES        := $(shell find $(SRC_DIR) -name '*.c')

# VALID TARGETS = F3 TARGET
STDPERIPH_DIR	= $(ROOT)/lib/main/STM32F30x_StdPeriph_Driver

STDPERIPH_SRC = $(notdir $(wildcard $(STDPERIPH_DIR)/src/*.c))

EXCLUDES	= stm32f30x_crc.c \
		        stm32f30x_can.c

STDPERIPH_SRC := $(filter-out ${EXCLUDES}, $(STDPERIPH_SRC))

DEVICE_STDPERIPH_SRC = \
		 $(STDPERIPH_SRC)


VPATH		  := $(VPATH):$(CMSIS_DIR)/CM1/CoreSupport:$(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x
CMSIS_SRC	 = $(notdir $(wildcard $(CMSIS_DIR)/CM1/CoreSupport/*.c \
			       $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
		   $(STDPERIPH_DIR)/inc \
		   $(CMSIS_DIR)/CM1/CoreSupport \
		   $(CMSIS_DIR)/CM1/DeviceSupport/ST/STM32F30x


LD_SCRIPT	 = $(LINKER_DIR)/stm32_flash_f303_$(FLASH_SIZE)k.ld

ARCH_FLAGS	 = -mthumb -mcpu=cortex-m4 -mfloat-abi=hard -mfpu=fpv4-sp-d16 -fsingle-precision-constant -Wdouble-promotion
DEVICE_FLAGS = -DSTM32F303xC -DSTM32F303
TARGET_FLAGS = -D$(TARGET)


ifneq ($(FLASH_SIZE),)
DEVICE_FLAGS := $(DEVICE_FLAGS) -DFLASH_SIZE=$(FLASH_SIZE)
endif

TARGET_DIR = $(ROOT)/src/main/target/$(TARGET)
TARGET_SRC = $(notdir $(wildcard $(TARGET_DIR)/*.c))

INCLUDE_DIRS := $(INCLUDE_DIRS) \
								$(TARGET_DIR)

VPATH		:= $(VPATH):$(TARGET_DIR)

COMMON_SRC = build_config.c \
						 debug.c \
						 version.c \
						 $(TARGET_SRC) \
						 config/config.c \
						 config/runtime_config.c \
						 config/config_streamer.c \
						 config/config_eeprom.c \
						 config/parameter_group.c \
						 config/feature.c \
						 config/profile.c \
						 common/maths.c \
						 common/printf.c \
						 common/typeconversion.c \
						 common/encoding.c \
						 common/filter.c \
						 common/streambuf.c \
						 scheduler.c \
             scheduler_tasks.c \
						 main.c \
						 mw.c \
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
						 drivers/gyro_sync.c \
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
						 rx/rx.c \
						 rx/pwm.c \
						 rx/msp.c \
						 rx/sbus.c \
						 rx/sumd.c \
						 rx/sumh.c \
						 rx/spektrum.c \
						 rx/xbus.c \
						 rx/ibus.c \
						 sensors/sensors.c \
						 sensors/acceleration.c \
						 sensors/battery.c \
						 sensors/boardalignment.c \
						 sensors/compass.c \
						 sensors/gyro.c \
						 sensors/initialisation.c \
						 $(CMSIS_SRC) \
						 $(DEVICE_STDPERIPH_SRC)

HIGHEND_SRC = \
						 flight/gtune.c \
						 flight/navigation.c \
						 flight/gps_conversion.c \
						 common/colorconversion.c \
						 io/gps.c \
						 io/ledstrip.c \
						 io/display.c \
						 telemetry/telemetry.c \
						 telemetry/frsky.c \
						 telemetry/hott.c \
						 telemetry/smartport.c \
						 telemetry/ltm.c \
						 telemetry/mavlink.c \
						 sensors/sonar.c \
						 sensors/barometer.c \
						 blackbox/blackbox.c \
						 blackbox/blackbox_io.c

STM32F30x_COMMON_SRC = \
						 startup_stm32f30x_md_gcc.S \
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
						 drivers/timer_stm32f30x.c

SPRACINGF3_SRC = \
						 $(STM32F30x_COMMON_SRC) \
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
						 drivers/sonar_hcsr04.c \
						 io/flashfs.c \
						 $(HIGHEND_SRC) \
						 $(COMMON_SRC)


# Search path and source files for the ST stdperiph library
VPATH		:= $(VPATH):$(STDPERIPH_DIR)/src

###############################################################################
# Things that might need changing to use different tools
#

# Tool names
CC		 = arm-none-eabi-gcc
OBJCOPY		 = arm-none-eabi-objcopy
SIZE		 = arm-none-eabi-size

#
# Tool options.
#

ifeq ($(DEBUG),GDB)
OPTIMIZE	 = -O0
LTO_FLAGS	 = $(OPTIMIZE)
else
OPTIMIZE	 = -Os
LTO_FLAGS	 =  -flto -fuse-linker-plugin $(OPTIMIZE)
endif

ifneq ($(filter $(OPTIONS),FAIL_ON_WARNINGS),)
WARN_FLAGS      += -Werror
endif

DEBUG_FLAGS	 = -ggdb3 -DDEBUG

CFLAGS		 = $(ARCH_FLAGS) \
						 $(LTO_FLAGS) \
						 $(WARN_FLAGS) \
						 $(addprefix -D,$(OPTIONS)) \
						 $(addprefix -I,$(INCLUDE_DIRS)) \
						 $(DEBUG_FLAGS) \
						 -std=gnu99 \
						 -Wall -Wextra -Wunsafe-loop-optimizations -Wdouble-promotion -Wundef \
						 -ffunction-sections \
						 -fdata-sections \
						 $(DEVICE_FLAGS) \
						 -DUSE_STDPERIPH_DRIVER \
						 $(TARGET_FLAGS) \
						 -D'__FORKNAME__="$(FORKNAME)"' \
						 -D'__TARGET__="$(TARGET)"' \
		   -D'__REVISION__="$(REVISION)"' \
						 -fverbose-asm -ffat-lto-objects \
						 -save-temps=obj \
						 -MMD -MP

ASFLAGS		 = $(ARCH_FLAGS) \
						 $(WARN_FLAGS) \
						 -x assembler-with-cpp \
						 $(addprefix -I,$(INCLUDE_DIRS)) \
						-MMD -MP

LDFLAGS		 = -lm \
						 -nostartfiles \
						 --specs=nano.specs \
						 -lc \
						 -lnosys \
						 $(ARCH_FLAGS) \
						 $(LTO_FLAGS) \
						 $(WARN_FLAGS) \
						 $(DEBUG_FLAGS) \
						 -static \
						 -Wl,-gc-sections,-Map,$(TARGET_MAP) \
						 -Wl,-L$(LINKER_DIR) \
						 -Wl,--cref \
						 -T$(LD_SCRIPT)

###############################################################################
# No user-serviceable parts below
###############################################################################

CPPCHECK   = cppcheck $(CSOURCES) --enable=all --platform=unix64 \
					 --std=c99 --inline-suppr --quiet --force \
					 $(addprefix -I,$(INCLUDE_DIRS)) \
					 -I/usr/include -I/usr/include/linux

#
# Things we will build
#

TARGET_BIN	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).bin
TARGET_HEX	 = $(BIN_DIR)/$(FORKNAME)_$(TARGET).hex
TARGET_ELF	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).elf
TARGET_OBJS	 = $(addsuffix .o,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_DEPS	 = $(addsuffix .d,$(addprefix $(OBJECT_DIR)/$(TARGET)/,$(basename $($(TARGET)_SRC))))
TARGET_MAP	 = $(OBJECT_DIR)/$(FORKNAME)_$(TARGET).map


## Default make goal:
## hex         : Make filetype hex only
.DEFAULT_GOAL := hex

## Optional make goals:
## all         : Make all filetypes, binary and hex
all: hex bin

## binary      : Make binary filetype
## bin         : Alias of 'binary'
## hex         : Make hex filetype
bin:    $(TARGET_BIN)
binary: $(TARGET_BIN)
hex:    $(TARGET_HEX)

# rule to reinvoke make with TARGET= parameter
# rules that should be handled in toplevel Makefile, not dependent on TARGET
GLOBAL_GOALS	= all_targets cppcheck test

.PHONY: $(TARGETS)
$(TARGETS):
	$(MAKE) $(filter-out $(TARGETS) $(GLOBAL_GOALS), $(MAKECMDGOALS))

## clean       : clean up all temporary / machine-generated files
clean:
	rm -f $(TARGET_BIN) $(TARGET_HEX) $(TARGET_ELF) $(TARGET_OBJS) $(TARGET_MAP)
	rm -rf $(OBJECT_DIR)/$(TARGET)
	cd src/test && $(MAKE) clean || true

flash_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	echo -n 'R' >$(SERIAL_DEVICE)
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## flash       : flash firmware (.hex) onto flight controller
flash: flash_$(TARGET)

st-flash_$(TARGET): $(TARGET_BIN)
	st-flash --reset write $< 0x08000000

## st-flash    : flash firmware (.bin) onto flight controller
st-flash: st-flash_$(TARGET)

unbrick_$(TARGET): $(TARGET_HEX)
	stty -F $(SERIAL_DEVICE) raw speed 115200 -crtscts cs8 -parenb -cstopb -ixon
	stm32flash -w $(TARGET_HEX) -v -g 0x0 -b 115200 $(SERIAL_DEVICE)

## unbrick     : unbrick flight controller
unbrick: unbrick_$(TARGET)

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

## test        : run the cleanflight test suite
## junittest   : run the cleanflight test suite, producing Junit XML result files.
test junittest:
	cd src/test && $(MAKE) $@

# rebuild everything when makefile changes
$(TARGET_OBJS) : Makefile

# List of buildable ELF files and their object dependencies.
# It would be nice to compute these lists, but that seems to be just beyond make.

$(TARGET_HEX): $(TARGET_ELF)
	$(OBJCOPY) -O ihex --set-start 0x8000000 $< $@

$(TARGET_BIN): $(TARGET_ELF)
	$(OBJCOPY) -O binary $< $@

$(TARGET_ELF):  $(TARGET_OBJS)
	$(CC) -o $@ $^ $(LDFLAGS)
	$(SIZE) $(TARGET_ELF)

# Compile
$(OBJECT_DIR)/$(TARGET)/%.o: %.c
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(CFLAGS) $<

# Assemble
$(OBJECT_DIR)/$(TARGET)/%.o: %.s
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

$(OBJECT_DIR)/$(TARGET)/%.o: %.S
	@mkdir -p $(dir $@)
	@echo %% $(notdir $<)
	@$(CC) -c -o $@ $(ASFLAGS) $<

# include auto-generated dependencies
-include $(TARGET_DEPS)
