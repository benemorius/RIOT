# Raspberry Pi GPIO debug adapter
export SWCLK ?= 21
export SWDIO ?= 20
export RST ?= 16

ifeq (0,$(shell test $$(free -m | grep Mem | awk '{print $$2}') -le 512; echo $$?))
  RPI_CONF = raspberrypi-native.cfg
else
  RPI_CONF = raspberrypi2-native.cfg
endif

export OPENOCD_ADAPTER_INIT ?= \
  -f 'interface/$(RPI_CONF)' \
  -c 'bcm2835gpio_swd_nums $(SWCLK) $(SWDIO)' \
  -c 'bcm2835gpio_srst_num $(RST)' \
  -c 'reset_config srst_only srst_open_drain' \
  -c 'transport select swd' \
  -c 'adapter_nsrst_delay 100' \
  -c 'adapter_nsrst_assert_width 100'

# if no openocd specific configuration file, check for default locations:
# 1. Using the default dist/openocd.cfg (automatically set by openocd.sh)
# 2. Using the common cpu specific config file
ifeq (,$(OPENOCD_CONFIG))
  # if no openocd default configuration is provided by the board,
  # use the STM32 common one
  ifeq (0,$(words $(wildcard $(RIOTBOARD)/$(BOARD)/dist/openocd.cfg)))
    export OPENOCD_CONFIG := $(RIOTBASE)/boards/common/stm32/dist/$(CPU).cfg
  endif
endif
