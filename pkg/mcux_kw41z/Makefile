PKG_NAME=mcux_kw41z
PKG_URL=https://github.com/benemorius/mcux_kw41z.git
PKG_VERSION=master
PKG_LICENSE=???

include $(RIOTBASE)/pkg/pkg.mk

CFLAGS += -DMANUFACTURER_NAME=\"openlabs.co\"
CFLAGS += -DBOARD_NAME=\"kw41z-mini\"

CFLAGS += -DSDK_DEBUGCONSOLE=0
CFLAGS += -Dasm=__asm__ -DFSL_RTOS_RIOT
CFLAGS += -imacros $(TDIR)/contrib/include/app_preinclude.h
CFLAGS += -Wno-extra -Wno-strict-prototypes -Wno-old-style-definition

export TDIR = $(RIOTPKG)/$(PKG_NAME)
export MDIR = $(PKG_SOURCE_DIR)/MKW41Z4
export FDIR = $(PKG_SOURCE_DIR)/framework_5.3.8
export BDIR = $(PKG_SOURCE_DIR)/bluetooth_1.2.8

SUBMODS := $(filter mcux_kw41z_%,$(USEMODULE))

all: $(SUBMODS) $(VENDOR_LIBS)

mcux_kw41z_riot_contrib:
	"$(MAKE)" -C $(TDIR)/contrib

mcux_kw41z_vendor_ble:
	"$(MAKE)" -C $(TDIR)/vendor

mcux_kw41z_vendor_seclib:
	"$(MAKE)" -C $(TDIR)/vendor/seclib

mcux_kw41z_vendor_controller:
	"$(MAKE)" -C $(TDIR)/vendor/controller

mcux_kw41z_system:
	"$(MAKE)" -C $(MDIR) -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_drivers:
	"$(MAKE)" -C $(MDIR)/drivers -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_xcvr:
	"$(MAKE)" -C $(FDIR)/XCVR/MKW41Z4 -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_xcvr_configs:
	"$(MAKE)" -C $(FDIR)/XCVR/MKW41Z4/cfgs_kw4x_3x_2x -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_common:
	"$(MAKE)" -C $(FDIR)/Common -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_dcdc:
	"$(MAKE)" -C $(FDIR)/DCDC/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_fsci:
	"$(MAKE)" -C $(FDIR)/FSCI/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_flash:
	"$(MAKE)" -C $(FDIR)/Flash/Internal -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_functionlib:
	"$(MAKE)" -C $(FDIR)/FunctionLib -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_gpio:
	"$(MAKE)" -C $(FDIR)/GPIO -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_keyboard:
	"$(MAKE)" -C $(FDIR)/Keyboard/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_led:
	"$(MAKE)" -C $(FDIR)/LED/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_lists:
	"$(MAKE)" -C $(FDIR)/Lists -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_lowpower:
	"$(MAKE)" -C $(FDIR)/LowPower/Source/MKW41Z -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_memmanager:
	"$(MAKE)" -C $(FDIR)/MemManager/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_messaging:
	"$(MAKE)" -C $(FDIR)/Messaging/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_mwscoexistence:
	"$(MAKE)" -C $(FDIR)/MWSCoexistence/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_nvm:
	"$(MAKE)" -C $(FDIR)/NVM/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_osabstraction:
	"$(MAKE)" -C $(FDIR)/OSAbstraction/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_otasupport:
	"$(MAKE)" -C $(FDIR)/OtaSupport/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_panic:
	"$(MAKE)" -C $(FDIR)/Panic/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_rng:
	"$(MAKE)" -C $(FDIR)/RNG/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_serialmanager:
	"$(MAKE)" -C $(FDIR)/SerialManager/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_shell:
	"$(MAKE)" -C $(FDIR)/Shell/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_timersmanager:
	"$(MAKE)" -C $(FDIR)/TimersManager/Source/ -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_application_common:
	"$(MAKE)" -C $(BDIR)/application/common -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_host_config:
	"$(MAKE)" -C $(BDIR)/host/config -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_application_gatt_db:
	"$(MAKE)" -C $(BDIR)/application/common/gatt_db -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_profile_temperature:
	"$(MAKE)" -C $(BDIR)/profiles/temperature -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_profile_battery:
	"$(MAKE)" -C $(BDIR)/profiles/battery -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_profile_device_info:
	"$(MAKE)" -C $(BDIR)/profiles/device_info -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_hci_transport:
	"$(MAKE)" -C $(BDIR)/hci_transport/source -f $(RIOTBASE)/Makefile.base MODULE=$@

mcux_kw41z_seclib:
	"$(MAKE)" -C $(FDIR)/SecLib -f $(RIOTBASE)/Makefile.base MODULE=$@
