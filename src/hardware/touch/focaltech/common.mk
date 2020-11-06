ifndef QCONFIG
QCONFIG=qconfig.mk
endif
include $(QCONFIG)


LIBS = drvrS cacheS m

NAME = libmtouch-focaltech

define PINFO
PINFO DESCRIPTION=Driver for FocalTech FT5x06 line of chips
endef


#####AUTO-GENERATED by packaging script... do not checkin#####
   INSTALL_ROOT_nto = $(PROJECT_ROOT)/../../../../install
   USE_INSTALL_ROOT=1
##############################################################

include $(MKFILES_ROOT)/qtargets.mk

-include $(PROJECT_ROOT)/roots.mk
