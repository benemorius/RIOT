MODULE = nimble_porting_nimble

# these files aren't needed if we aren't building controller
ifeq (,$(filter nimble_controller,$(USEMODULE)))
  IGNORE := hal_timer.c
endif

SRC := $(filter-out $(IGNORE),$(wildcard *.c))

include $(RIOTBASE)/Makefile.base
