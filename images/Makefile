ROOT_DIR := $(notdir $(CURDIR))
ifndef QCONFIG
QCONFIG=qconfig.mk
endif

# Uncomment for add CM4 firmware to the IPL bootable image.
#CM4_FIRMWARE := imx-mkimage/cm4_hello_world_tcm.bin
# CM4 core start configuration
ifdef CM4_FIRMWARE
    CM4_PARAMETER = -m4 $(CM4_FIRMWARE) 0 0x34FE0000
else
    CM4_PARAMETER =
endif

# For ATF using please comment line below (SCFW going to move all resources to secure partition)
SCFW_FLAGS = -flags 0x00010000

# IPL bootable image is created under Linux OS only.
# For other OS IPL binary is created only and mingw utility is needed for creating imx bootable image
# (see UserGuide documantation).
IPL_IMAGE :=
ifeq ($(OS),Windows_NT)
	MKIMAGE_TOOL := mkimage_imx8.exe
else
	MKIMAGE_TOOL := mkimage_imx8
	ifeq ($(shell uname -s),Linux)
		IPL_IMAGE := ipl-imx
	endif
endif

include $(QCONFIG)

SRC := $(CURDIR)/../src
BOARD_ROOT := $(SRC)/hardware/startup/boards
HOST_MKIFS := mkifs
BOARD_NAME := imx8qxp-cpu
IPL_START_ADDR := 0x80000000
IFS_LOAD_ADDR := 0x88000000
SUFFIXES := .build .ifs .raw

.PHONY: all clean ipl-bin $(IPL_IMAGE)

default:
	echo Use \'make all\' after obtaining required binaries from NXP

all: ifs-imx8qxp-cpu.bin ifs-imx8qxp-cpu-graphics.bin ipl-bin $(IPL_IMAGE)

clean:
	$(RM_HOST) ifs-imx8qxp-cpu.bin *.bin *.imx
	$(RM_HOST) *.sym

ifs-imx8qxp-cpu.bin: imx8qxp-cpu.build
	$(HOST_MKIFS) -vvv -r../install $(MKIFSFLAGS) $^ $@

ifs-imx8qxp-cpu-graphics.bin: imx8qxp-cpu-graphics.build
	$(HOST_MKIFS) -vvv -r../install $(MKIFSFLAGS) $^ $@

ipl-bin:
	${QNX_HOST}/usr/bin/ntoaarch64-objcopy --input-target=elf64-littleaarch64 --output-target=binary ../src/hardware/ipl/boards/$(BOARD_NAME)/aarch64/le/ipl-$(BOARD_NAME) ./ipl-$(BOARD_NAME).bin
ipl-imx:

	# B0 silicon version
	imx-mkimage/$(MKIMAGE_TOOL) -soc QX -rev B0 -append imx-mkimage/mx8qxb0-ahab-container.img -c $(SCFW_FLAGS) -scfw imx-mkimage/scfw_tcm_b0.bin -ap ipl-$(BOARD_NAME).bin a35 $(IPL_START_ADDR) $(CM4_PARAMETER) -out ipl-$(BOARD_NAME)_b0.imx
	imx-mkimage/$(MKIMAGE_TOOL) -soc QX -rev B0 -append imx-mkimage/mx8qxb0-ahab-container.img -dev flexspi -c $(SCFW_FLAGS) -scfw imx-mkimage/scfw_tcm_b0.bin -ap ipl-$(BOARD_NAME).bin a35 $(IPL_START_ADDR) $(CM4_PARAMETER) -out ipl-$(BOARD_NAME)_flash_b0.imx

	# C0 silicon version
	# B0 SC firmware binary includes C0 support
	imx-mkimage/$(MKIMAGE_TOOL) -soc QX -rev B0 -append imx-mkimage/mx8qxc0-ahab-container.img -c $(SCFW_FLAGS) -scfw imx-mkimage/scfw_tcm_b0.bin -ap ipl-$(BOARD_NAME).bin a35 $(IPL_START_ADDR) $(CM4_PARAMETER) -out ipl-$(BOARD_NAME)_c0.imx
	imx-mkimage/$(MKIMAGE_TOOL) -soc QX -rev B0 -append imx-mkimage/mx8qxc0-ahab-container.img -dev flexspi -c $(SCFW_FLAGS) -scfw imx-mkimage/scfw_tcm_b0.bin -ap ipl-$(BOARD_NAME).bin a35 $(IPL_START_ADDR) $(CM4_PARAMETER) -out ipl-$(BOARD_NAME)_flash_c0.imx

	# Uncomment for add "containter" to ifs-imx8qxp-cpu.imx image (used for AHAB authentication)
	#imx-mkimage/$(MKIMAGE_TOOL) -soc QX -rev B0 -c -ap ifs-imx8qxp-cpu.bin a35 $(IFS_LOAD_ADDR) -out ifs-imx8qxp-cpu.imx
	#imx-mkimage/$(MKIMAGE_TOOL) -soc QX -rev B0 -c -ap ifs-imx8qxp-cpu-graphics.bin a35 $(IFS_LOAD_ADDR) -out ifs-imx8qxp-cpu-graphics.imx

	$(RM_HOST) ipl-$(BOARD_NAME).bin
