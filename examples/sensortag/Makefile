# name of your application
APPLICATION = sensortag

# If no BOARD is found in the environment, use this default:
BOARD = srf06-cc26xx

# stm32f0discovery objects are too big with ARM Embedded Toolchain v4.9.3 20141119
# (used currently by travis)
BOARD_INSUFFICIENT_RAM=stm32f0discovery

# This has to be the absolute path to the RIOT base directory:
RIOTBASE ?= $(CURDIR)/../..

# Uncomment these lines if you want to use platform support from external
# repositories:
#RIOTCPU ?= $(CURDIR)/../../../thirdparty_cpu
#RIOTBOARD ?= $(CURDIR)/../../../thirdparty_boards

# Uncomment this to enable scheduler statistics for ps:
#CFLAGS += -DSCHEDSTATISTICS

# If you want to use native with valgrind, you should recompile native
# with the target all-valgrind instead of all:
# make -B clean all-valgrind

# Comment this out to disable code in RIOT that does safety checking
# which is not needed in a production environment but helps in the
# development process:
CFLAGS += -DDEVELHELP

# Change this to 0 show compiler invocation lines by default:
QUIET ?= 0

# Features required
# FEATURES_REQUIRED += cpp

# If you want to add some extra flags when compile c++ files, add these flags
# to CXXEXFLAGS variable
CXXEXFLAGS +=
CFLAGS +=


CPPMIX = 1

# USEMODULE += vtimer
# USEMODULE += transceiver
# USEMODULE += defaulttransceiver
# USEMODULE += at86rf231
# USEMODULE += rpl
# USEMODULE += udp

USEMODULE += shell
USEMODULE += shell_commands
# USEMODULE += uart0
USEMODULE += ps
# USEMODULE += xtimer

USEMODULE += at45
USEMODULE += si70xx
USEMODULE += bmp180

USEMODULE += hashes

USEMODULE += cc26x0_rf
USEMODULE += netif
USEMODULE += ieee802154
USEMODULE += netdev2_ieee802154
USEMODULE += gnrc_netdev2

USEMODULE += gnrc_netdev_default
USEMODULE += auto_init_gnrc_netif
# Specify the mandatory networking modules for IPv6 and UDP
USEMODULE += gnrc_ipv6_default
USEMODULE += gnrc_udp
# Add a routing protocol
USEMODULE += gnrc_rpl
USEMODULE += auto_init_gnrc_rpl
# This application dumps received packets to STDIO using the pktdump module
# USEMODULE += gnrc_pktdump
# Additional networking modules that can be dropped if not needed
USEMODULE += gnrc_icmpv6_echo
# Add also the shell, some shell commands
USEMODULE += netstats_l2
USEMODULE += netstats_ipv6


# DISABLE_MODULE = auto_init

# USEMODULE += gnrc_netdev_default
# USEMODULE += gnrc_netdev2
# USEMODULE += auto_init_gnrc_netif
# USEMODULE += gnrc_ipv6

# We use only the lower layers of the GNRC network stack, hence, we can
# reduce the size of the packet buffer a bit
CFLAGS += -DGNRC_PKTBUF_SIZE=512


export INCLUDES += -I$(RIOTBASE)/sys/net/include \
-I$(RIOTBASE)/sys/net/routing/aodvv2 \
-I$(RIOTBASE)/pkg/oonf_api/oonf_api/src-api/

include $(RIOTBASE)/Makefile.include
