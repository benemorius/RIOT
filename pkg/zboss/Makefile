PKG_NAME=zboss
PKG_URL=https://github.com/benemorius/zboss.git
PKG_VERSION=16badda7fdf9df3974fe531db23f1a4bacaf8620
PKG_LICENSE=GPL-2

# CFLAGS +=

.PHONY: all

all: git-download
	cp $(RIOTBASE)/pkg/zboss/src/zboss-riot.c $(PKG_BUILDDIR)/osif/unix/zboss-riot.c
	"$(MAKE)" -C $(PKG_BUILDDIR)

include $(RIOTBASE)/pkg/pkg.mk
