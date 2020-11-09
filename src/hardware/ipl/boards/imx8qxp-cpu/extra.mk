# Uncomment 'IMX_ARM_TRUSTED_FW' symbol to enable Arm Trusted Firmware
# For enabling ATF, please comment line with the 'SCFW_FLAGS' variable in the images/Makefile file
# (SCFW going to move all resources to secure partition)
IMX_ARM_TRUSTED_FW = 1

ifdef IMX_ARM_TRUSTED_FW
    LDFLAGS := $(filter-out -nostdlib $(LDF_$(COMPILER_DRIVER)),$(LDFLAGS))
    LDFLAGS := $(filter-out -fuse-ld=bfd -nopie,$(LDFLAGS))
    LDF_ := $(filter-out -T$(PROJECT_ROOT)/$(SECTION)/$(SECTION).lnk,$(LDF_))
    LDF_qcc := $(filter-out -M -nostartup -Wl,-T$(PROJECT_ROOT)/$(SECTION)/$(SECTION).lnk,$(LDF_qcc))
    LDF_ = -T$(PROJECT_ROOT)/$(SECTION)/$(SECTION)-atf.lnk
    LDF_qcc = -M -nostartup -Wl,-T$(PROJECT_ROOT)/$(SECTION)/$(SECTION)-atf.lnk -static
    LDFLAGS += -nostdlib $(LDF_$(COMPILER_DRIVER))
    LDFLAGS += -fuse-ld=bfd -nopie
    CCFLAGS += -DIMX_ARM_TRUSTED_FW
    ASFLAGS += -DIMX_ARM_TRUSTED_FW
endif

LDF_ += -static
LDF_qcc += -static
LIBS += nxp_imx8_sci
ASFLAGS += -DIMX_ASM_MACROS_EN
