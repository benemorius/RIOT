# define the module that is build
MODULE = cpu

# add a list of directories that should also be built
DIRS += periph $(RIOTCPU)/cortexm_common $(KINETIS_COMMON)
DIRS += vendor/XCVR/MKW41Z4

# (file triggers compiler bug. see #5775)
SRC_NOLTO += vectors.c

include $(RIOTBASE)/Makefile.base
