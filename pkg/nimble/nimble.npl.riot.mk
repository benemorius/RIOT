MODULE = nimble_npl_riot

# these files aren't needed if we aren't building controller
ifeq (,$(filter nimble_controller,$(USEMODULE)))
  IGNORE := nrf5x_isr.c
endif

SRC := $(filter-out $(IGNORE),$(wildcard *.c))

include $(RIOTBASE)/Makefile.base
