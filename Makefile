PROJECT_NAME     := raktrack
TARGETS          := raktrack
OUTPUT_DIRECTORY := _build

SDK_ROOT := ../../..
PROJ_DIR := .

$(OUTPUT_DIRECTORY)/$(PROJECT_NAME).out: \
  LINKER_SCRIPT  := $(PROJECT_NAME)_gcc_nrf52.ld

# Source files common to all targets
SRC_FILES += \
  $(SDK_ROOT)/modules/nrfx/mdk/gcc_startup_nrf52840.S \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_rtt.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_backend_serial.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_default_backends.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_frontend.c \
  $(SDK_ROOT)/components/libraries/log/src/nrf_log_str_formatter.c \
  \
  $(SDK_ROOT)/components/boards/boards.c \
  \
  $(SDK_ROOT)/components/ble/common/ble_advdata.c \
  $(SDK_ROOT)/components/ble/nrf_ble_scan/nrf_ble_scan.c \
  $(SDK_ROOT)/components/libraries/atomic_fifo/nrf_atfifo.c \
  $(SDK_ROOT)/components/libraries/atomic/nrf_atomic.c \
  $(SDK_ROOT)/components/libraries/balloc/nrf_balloc.c \
  $(SDK_ROOT)/components/libraries/block_dev/empty/nrf_block_dev_empty.c \
  $(SDK_ROOT)/components/libraries/block_dev/qspi/nrf_block_dev_qspi.c \
  $(SDK_ROOT)/components/libraries/block_dev/qspi/nrf_serial_flash_params.c \
  $(SDK_ROOT)/components/libraries/block_dev/ram/nrf_block_dev_ram.c \
  $(SDK_ROOT)/components/libraries/block_dev/sdc/nrf_block_dev_sdc.c \
  $(SDK_ROOT)/components/libraries/experimental_section_vars/nrf_section_iter.c \
  $(SDK_ROOT)/components/libraries/fifo/app_fifo.c \
  $(SDK_ROOT)/components/libraries/hardfault/hardfault_implementation.c \
  $(SDK_ROOT)/components/libraries/hardfault/nrf52/handler/hardfault_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/memobj/nrf_memobj.c \
  $(SDK_ROOT)/components/libraries/pwr_mgmt/nrf_pwr_mgmt.c \
  $(SDK_ROOT)/components/libraries/queue/nrf_queue.c \
  $(SDK_ROOT)/components/libraries/ringbuf/nrf_ringbuf.c \
  $(SDK_ROOT)/components/libraries/scheduler/app_scheduler.c \
  $(SDK_ROOT)/components/libraries/sdcard/app_sdcard.c \
  $(SDK_ROOT)/components/libraries/sortlist/nrf_sortlist.c \
  $(SDK_ROOT)/components/libraries/strerror/nrf_strerror.c \
  $(SDK_ROOT)/components/libraries/timer/app_timer2.c \
  $(SDK_ROOT)/components/libraries/timer/drv_rtc.c \
  $(SDK_ROOT)/components/libraries/usbd/app_usbd.c \
  $(SDK_ROOT)/components/libraries/usbd/app_usbd_core.c \
  $(SDK_ROOT)/components/libraries/usbd/app_usbd_string_desc.c \
  $(SDK_ROOT)/components/libraries/usbd/class/msc/app_usbd_msc.c \
  $(SDK_ROOT)/components/libraries/util/app_error.c \
  $(SDK_ROOT)/components/libraries/util/app_error_handler_gcc.c \
  $(SDK_ROOT)/components/libraries/util/app_error_weak.c \
  $(SDK_ROOT)/components/libraries/util/app_util_platform.c \
  $(SDK_ROOT)/components/libraries/util/nrf_assert.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ant.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_ble.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh.c \
  $(SDK_ROOT)/components/softdevice/common/nrf_sdh_soc.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf.c \
  $(SDK_ROOT)/external/fprintf/nrf_fprintf_format.c \
  $(SDK_ROOT)/external/utf_converter/utf.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_clock.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_power.c \
  $(SDK_ROOT)/integration/nrfx/legacy/nrf_drv_spi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_clock.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_gpiote.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_power.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_ppi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_qspi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_rtc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_saadc.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_spim.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_timer.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twi.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_twim.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_uarte.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/nrfx_usbd.c \
  $(SDK_ROOT)/modules/nrfx/drivers/src/prs/nrfx_prs.c \
  $(SDK_ROOT)/modules/nrfx/soc/nrfx_atomic.c \
  \
  $(SDK_ROOT)/components/ant/ant_channel_config/ant_channel_config.c \
  $(SDK_ROOT)/components/ant/ant_key_manager/ant_key_manager.c \
  $(SDK_ROOT)/components/ant/ant_search_config/ant_search_config.c \
  \
  $(PROJ_DIR)/main.c \
  $(PROJ_DIR)/gps.c \
  $(PROJ_DIR)/oled.c \
  $(PROJ_DIR)/sensors.c \
  $(PROJ_DIR)/ant.c \
  \
  $(SDK_ROOT)/external/fatfs/port/diskio_blkdev.c \
  $(SDK_ROOT)/external/fatfs/src/ff.c \
  $(SDK_ROOT)/modules/nrfx/mdk/system_nrf52840.c \
  $(SDK_ROOT)/external/segger_rtt/SEGGER_RTT.c \
\

## Include folders common to all targets
INC_FOLDERS += \
  config \
  $(PROJ_DIR) \
  $(SDK_ROOT)/components \
  $(SDK_ROOT)/components/ant/ant_channel_config \
  $(SDK_ROOT)/components/ant/ant_key_manager \
  $(SDK_ROOT)/components/ant/ant_key_manager/config \
  $(SDK_ROOT)/components/ant/ant_profiles/ant_hrm \
  $(SDK_ROOT)/components/ant/ant_profiles/ant_hrm/utils \
  $(SDK_ROOT)/components/ant/ant_search_config \
  $(SDK_ROOT)/components/ant/ant_state_indicator \
  $(SDK_ROOT)/components/ble/common \
  $(SDK_ROOT)/components/ble/nrf_ble_scan \
  $(SDK_ROOT)/components/boards \
  $(SDK_ROOT)/components/libraries/atomic \
  $(SDK_ROOT)/components/libraries/atomic_fifo \
  $(SDK_ROOT)/components/libraries/balloc \
  $(SDK_ROOT)/components/libraries/block_dev \
  $(SDK_ROOT)/components/libraries/block_dev/ram \
  $(SDK_ROOT)/components/libraries/block_dev/sdc \
  $(SDK_ROOT)/components/libraries/block_dev/empty \
  $(SDK_ROOT)/components/libraries/delay \
  $(SDK_ROOT)/components/libraries/experimental_section_vars \
  $(SDK_ROOT)/components/libraries/fifo \
  $(SDK_ROOT)/components/libraries/log \
  $(SDK_ROOT)/components/libraries/log/src \
  $(SDK_ROOT)/components/libraries/memobj \
  $(SDK_ROOT)/components/libraries/mutex \
  $(SDK_ROOT)/components/libraries/pwr_mgmt \
  $(SDK_ROOT)/components/libraries/queue \
  $(SDK_ROOT)/components/libraries/ringbuf \
  $(SDK_ROOT)/components/libraries/scheduler \
  $(SDK_ROOT)/components/libraries/sdcard \
  $(SDK_ROOT)/components/libraries/block_dev/qspi \
  $(SDK_ROOT)/components/libraries/sensorsim \
  $(SDK_ROOT)/components/libraries/sortlist \
  $(SDK_ROOT)/components/libraries/strerror \
  $(SDK_ROOT)/components/libraries/timer \
  $(SDK_ROOT)/components/libraries/usbd \
  $(SDK_ROOT)/components/libraries/usbd/class/msc \
  $(SDK_ROOT)/components/libraries/util \
  $(SDK_ROOT)/components/softdevice/common \
  $(SDK_ROOT)/components/softdevice/s340/headers \
  $(SDK_ROOT)/components/softdevice/s340/headers/nrf52 \
  $(SDK_ROOT)/components/toolchain/cmsis/include \
  $(SDK_ROOT)/external/fatfs/port \
  $(SDK_ROOT)/external/fatfs/src \
  $(SDK_ROOT)/external/fprintf \
  $(SDK_ROOT)/external/protothreads \
  $(SDK_ROOT)/external/protothreads/pt-1.4 \
  $(SDK_ROOT)/external/segger_rtt \
  $(SDK_ROOT)/external/utf_converter \
  $(SDK_ROOT)/integration/nrfx \
  $(SDK_ROOT)/modules/nrfx \
  $(SDK_ROOT)/modules/nrfx/drivers/include \
  $(SDK_ROOT)/modules/nrfx/hal \
  $(SDK_ROOT)/modules/nrfx/mdk \
  $(SDK_ROOT)/integration/nrfx/legacy \


# Libraries common to all targets
LIB_FILES += \

# Optimization flags
#OPT = -O3 -g3
OPT = -Og -g3 

#-E -dM  # show macros
#-H      # show headers

# Uncomment the line below to enable link time optimization
#OPT += -flto

# C flags common to all targets
CFLAGS += $(OPT)
CFLAGS += -DDEBUG -DDEBUG_NRF
CFLAGS += -DAPP_TIMER_V2
CFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
CFLAGS += -DBOARD_PCA10056
CFLAGS += -DNRF52_PAN_74
#app timer
#CFLAGS += -DSWI_DISABLE0
#sd
#CFLAGS += -DSWI_DISABLE1 
#sdrad
#CFLAGS += -DSWI_DISABLE2 
CFLAGS += -DBSP_DEFINES_ONLY
CFLAGS += -DCONFIG_GPIO_AS_PINRESET
CFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
CFLAGS += -DFLOAT_ABI_HARD
CFLAGS += -DINITIALIZE_USER_SECTIONS
CFLAGS += -DNRF52840_XXAA
CFLAGS += -DS340
CFLAGS += -DSOFTDEVICE_PRESENT
CFLAGS += -mcpu=cortex-m4
CFLAGS += -mthumb -mabi=aapcs
CFLAGS += -Wall -Werror -Wno-unused-function -Wno-unused-const-variable -Wno-pointer-sign
CFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
# keep every function in a separate section, this allows linker to discard unused ones
CFLAGS += -ffunction-sections -fdata-sections -fno-strict-aliasing
CFLAGS += -fno-builtin -fshort-enums

# C++ flags common to all targets
CXXFLAGS += $(OPT)

# Assembler flags common to all targets
ASMFLAGS += -g3
ASMFLAGS += -DDEBUG -DDEBUG_NRF
ASMFLAGS += -mcpu=cortex-m4
ASMFLAGS += -mthumb -mabi=aapcs
ASMFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16
ASMFLAGS += -DAPP_TIMER_V2
ASMFLAGS += -DAPP_TIMER_V2_RTC1_ENABLED
ASMFLAGS += -DBOARD_PCA10056
ASMFLAGS += -DNRF52_PAN_74
#ASMFLAGS += -DSWI_DISABLE0
#ASMFLAGS += -DSWI_DISABLE1
#ASMFLAGS += -DSWI_DISABLE2
ASMFLAGS += -DBSP_DEFINES_ONLY
ASMFLAGS += -DCONFIG_GPIO_AS_PINRESET
ASMFLAGS += -DCONFIG_NFCT_PINS_AS_GPIOS
ASMFLAGS += -DFLOAT_ABI_HARD
ASMFLAGS += -DNRF52840_XXAA
ASMFLAGS += -DS340
ASMFLAGS += -DSOFTDEVICE_PRESENT

# Linker flags
LDFLAGS += $(OPT)
LDFLAGS += -mthumb -mabi=aapcs -L$(SDK_ROOT)/modules/nrfx/mdk -T$(LINKER_SCRIPT)
LDFLAGS += -mcpu=cortex-m4
LDFLAGS += -mfloat-abi=hard -mfpu=fpv4-sp-d16

LDFLAGS += -u _printf_float

# let linker dump unused sections
LDFLAGS += -Wl,--gc-sections
# use newlib in nano version
LDFLAGS += --specs=nano.specs

$(PROJECT_NAME): CFLAGS += -D__HEAP_SIZE=8192
$(PROJECT_NAME): CFLAGS += -D__STACK_SIZE=8192
$(PROJECT_NAME): ASMFLAGS += -D__HEAP_SIZE=8192
$(PROJECT_NAME): ASMFLAGS += -D__STACK_SIZE=8192

# Add standard libraries at the very end of the linker input, after all objects
# that may need symbols provided by these libraries.
LIB_FILES += -lc -lnosys -lm

.PHONY: default help

# Default target - first one defined
default: $(PROJECT_NAME)

# Print all targets that can be built
help:
	@echo following targets are available:
	@echo		$(PROJECT_NAME)
	@echo		sdk_config - starting external tool for editing sdk_config.h
	@echo		flash      - flashing binary

TEMPLATE_PATH := $(SDK_ROOT)/components/toolchain/gcc


include $(TEMPLATE_PATH)/Makefile.common

$(foreach target, $(TARGETS), $(call define_target, $(target)))

.PHONY: flash erase

flash: default
	@echo Flashing: $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex
	openocd -f interface/stlink-dap.cfg -f target/nrf52.cfg \
		-c "program $(OUTPUT_DIRECTORY)/$(PROJECT_NAME).hex verify reset exit"

erase:
	openocd -f interface/stlink-dap.cfg -f target/nrf52.cfg \
		-c "init" \
		-c "nrf5 mass_erase"

SDK_CONFIG_FILE := config/sdk_config.h
CMSIS_CONFIG_TOOL := $(SDK_ROOT)/external_tools/cmsisconfig/CMSIS_Configuration_Wizard.jar
sdk_config:
	java -jar $(CMSIS_CONFIG_TOOL) $(SDK_CONFIG_FILE)
