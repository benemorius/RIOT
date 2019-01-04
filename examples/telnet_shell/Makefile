# name of your application
APPLICATION = telnet_shell

# If no BOARD is found in the environment, use this default:
BOARD ?= kw41z-mini

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Include packages that pull up and auto-init the link layer.
# NOTE: 6LoWPAN will be included if IEEE802.15.4 devices are present
USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
# Activate ICMPv6 error messages
USEMODULE += gnrc_icmpv6_error
USEMODULE += gnrc_icmpv6_echo
# Specify the mandatory networking modules for IPv6 and UDP
# USEMODULE += gnrc_ipv6_router_default
USEMODULE += gnrc_ipv6_default

USEMODULE += gnrc_udp
USEMODULE += gnrc_tcp
# USEMODULE += gnrc_sock_udp
# USEMODULE += gnrc_sock_tcp
# USEMODULE += posix_sockets

USEMODULE += vfs
USEMODULE += posix
USEMODULE += posix_time

USEMODULE += shell
USEMODULE += shell_commands
USEMODULE += ps
USEMODULE += netstats_l2
USEMODULE += netstats_ipv6
USEMODULE += netstats_rpl

USEMODULE += od
USEMODULE += od_string
USEMODULE += netopt
USEMODULE += sntp
USEMODULE += gnrc_pktbuf_cmd

USEMODULE += log_dmesg

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

# Uncomment the following 2 lines to specify static link local IPv6 address
# this might be useful for testing, in cases where you cannot or do not want to
# run a shell with ifconfig to get the real link local address.
#IPV6_STATIC_LLADDR ?= '"fe80::cafe:cafe:cafe:1"'
#CFLAGS += -DGNRC_IPV6_STATIC_LLADDR=$(IPV6_STATIC_LLADDR)

# Uncomment this to join RPL DODAGs even if DIOs do not contain
# DODAG Configuration Options (see the doc for more info)
# CFLAGS += -DGNRC_RPL_DODAG_CONF_OPTIONAL_ON_JOIN

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 1

include $(RIOTBASE)/Makefile.include

# Set a custom channel if needed
DEFAULT_CHANNEL ?= 26
CFLAGS += -DIEEE802154_DEFAULT_CHANNEL=$(DEFAULT_CHANNEL)

# CFLAGS += -Wno-all

CFLAGS += -DLOG_LEVEL=LOG_ALL
