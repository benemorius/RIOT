# name of your application
APPLICATION = strain-gauge

# If no BOARD is found in the environment, use this default:
BOARD ?= openlabs-kw41z-mini

USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
USEMODULE += gnrc_ipv6_default
USEMODULE += gnrc_icmpv6_echo
# USEMODULE += gnrc_icmpv6_error
USEMODULE += netstats_l2

USEMODULE += util
USEMODULE += util_shell
# USEMODULE += util_ota_server
# USEMODULE += telnet_server

USEMODULE += gnrc_sock_udp
USEMODULE += emcute

# Optimize network stack for use with a single network interface
CFLAGS += -DGNRC_NETIF_SINGLE

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
DEVELHELP ?= 1

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

DEFAULT_CHANNEL ?= 26
CFLAGS += -DCONFIG_IEEE802154_DEFAULT_CHANNEL=$(DEFAULT_CHANNEL)

# Use LED to indicate packet TX/RX
CFLAGS += -DKW41ZRF_ENABLE_LEDS=1

# Use internal voltage reference
CFLAGS += -DADC_REF_SETTING=0

# Use baud rate <=57600 for low power modes
CFLAGS += -DSTDIO_UART_BAUDRATE=57600

# Disable retries and use a short timeout so we can go back to sleep quickly
CFLAGS += -DEMCUTE_N_RETRY=0
CFLAGS += -DEMCUTE_T_RETRY=0.02

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..
include $(RIOTBASE)/Makefile.include
