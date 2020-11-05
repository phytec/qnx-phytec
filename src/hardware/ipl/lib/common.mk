ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)

define PINFO
PINFO DESCRIPTION=ipl library
endef

ASFLAGS_x86 += -x assembler-with-cpp
ASFLAGS+=$(ASFLAGS_$(CPU))

NAME = ipl

## DEFFILE = asmoff.def

INSTALLDIR = usr/lib

EXTRA_SRCVPATH += $(PROJECT_ROOT)/fs
EXTRA_SRCVPATH += $(PROJECT_ROOT)/flexspi
EXTRA_SRCVPATH += $(PROJECT_ROOT)/sdmmc
EXTRA_INCVPATH += $(PROJECT_ROOT)/private
EXTRA_SRCVPATH += $(PROJECT_ROOT)/secureboot
EXTRA_SRCVPATH += $(PROJECT_ROOT)/usbboot
EXTRA_SRCVPATH += $(PROJECT_ROOT)/usbboot/udc
EXTRA_INCVPATH += $(PROJECT_ROOT)/usbboot
EXTRA_SRCVPATH += $(PROJECT_ROOT)/uecc
EXTRA_INCVPATH += $(PROJECT_ROOT)/uecc


#####AUTO-GENERATED by packaging script... do not checkin#####
   INSTALL_ROOT_nto = $(PROJECT_ROOT)/../../../../install
   USE_INSTALL_ROOT=1
##############################################################

include $(MKFILES_ROOT)/qtargets.mk


-include $(PROJECT_ROOT)/roots.mk

CCFLAGS_aarch64 +=-mgeneral-regs-only -mstrict-align -fno-store-merging
CCFLAGS += $(CCFLAGS_$(CPU))
CCFLAGS+=-fno-PIE
ASFLAGS+=-fno-PIE

GCCVER:= $(if $(GCC_VERSION),$(GCC_VERSION), $(shell qcc -V 2>&1 | grep default | sed -e 's/,.*//'))
ifneq ($(filter 4.4.%, $(strip $(GCCVER))),)
       CCFLAGS+=-fno-dwarf2-cfi-asm
endif

CCFLAGS+=-fno-stack-protector -fno-unwind-tables
CCOPTS:=$(filter-out -fstack-protector%,$(CCOPTS))

#
# This particular little kludge is to stop GCC from using F.P. instructions
# to move 8 byte quantities around.
#
CC_nto_ppc_gcc += -msoft-float
CC_nto_ppc_gcc_qcc += -Wc,-msoft-float
