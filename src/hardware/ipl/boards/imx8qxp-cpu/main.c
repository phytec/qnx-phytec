/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
 * Copyright 2018, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"). You
 * may not reproduce, modify or distribute this software except in
 * compliance with the License. You may obtain a copy of the License
 * at: http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" basis,
 * WITHOUT WARRANTIES OF ANY KIND, either express or implied.
 *
 * This file may contain contributions from others, either as
 * contributors under the License or as licensors under other terms.
 * Please review this entire file for other proprietary rights or license
 * notices, as well as the QNX Development Suite License Guide at
 * http://licensing.qnx.com/license-guide/ for other information.
 * $
 */

#include "ipl.h"
#include "board.h"
#include "imx_ipl.h"
#include <hw/inout.h>
#include "private/fat-fs.h"
#include <private/imx8_sdhc.h>
#include <private/imx8_flexspi_ipl.h>
#include <sys/srcversion.h>
#include <hw/nxp/imx8/sci/sci.h>
#include <aarch64/mx8xp.h>
#include <aarch64/mx8x_smc_call.h>

#if (IMX_HAB_FUSE_CONFIG == 1)
#define IMX_PRINT_FUSE(word, val)   ser_putstr("    Fuse["); \
                                    ser_putdec(word); \
                                    ser_putstr("]: 0x"); \
                                    ser_puthex(val); \
                                    ser_putstr("\n");

#define IMX_READ_AND_PRINT_FUSE(word, val) \
                                    ser_putstr("    Fuse["); \
                                    ser_putdec(word); \
                                    ser_putstr("]: 0x"); \
                                    (void)sc_misc_otp_fuse_read(ipc, word, &val); \
                                    ser_puthex(val); \
                                    ser_putstr("\n");


const uint32_t hab_oem_srk_hash[][2] = {IMX_HAB_FUSE_VALUES};
#endif

/* The buffer used by fat-fs.c as the common buffer */
static unsigned char fat_buf2[FAT_COMMON_BUF_SIZE];
/* The buffer used by fat-fs.c as the FAT info */
static unsigned char fat_buf1[FAT_FS_INFO_BUF_SIZE];

#if (IMX_CACHE_EN == 1)
    extern void imx_enable_mmu(void);
    extern void imx_disable_mmu(void);
#endif
/* imx8 chip information */
uint32_t chip_type = IMX_CHIP_TYPE_QUAD_X_PLUS;
uint32_t chip_rev = IMX_CHIP_REV_A;

#ifdef IMX8_DEBUG
/**
 * Software delay loop.
 *
 * @param dly Delay loop count.
 */
static void delay(unsigned dly)
{
    volatile int j;

    while (dly--) {
        for (j = 0; j < 32; j++) {}
    }
}
#endif

#if (IMX_QSPI_BOOT == 1)
/**
 * Load QNX image file from flash memory to DDR memory.
 *
 * @param dest_addr     QNX image DDR memory address.
 * @param source_addr   QNX image flash memory address.
 *
 * @return              Execution status.
 */
static inline int flexspi_load_file(unsigned long dest_addr, unsigned source_addr)
{
    flexspi_conf_t flexspi_conf = {
            .rx_fifo_size = 512,        /* Rx FIFO size */
            .pads = 8,                  /* Pads number */
            .smpl = 3,                  /* Sample clock source */
            .lut0 = 0x0B2004CCU,        /* Lookup table 0 */
            .lut1 = 0x27043310U,        /* Lookup table 1 */
            .lut2 = 0U,                 /* Lookup table 2 */
            .lut3 = 0U,                 /* Lookup table 3 */
    };
    flexspi_t           flexspi;
    int                 status;
#if (IMX_HAB_AUTHENTICATE_CONFIG == 1)
    imx_flash_header_t  flash_header;
#endif
    unsigned long       startup_hdr_addr;
    unsigned int        image_size = 0;

    flexspi.conf = &flexspi_conf;
    imx_flexspi_init(&flexspi, IMX_FLEXSPI0_BASE, 0);

#if (IMX_HAB_AUTHENTICATE_CONFIG == 1)
    status = imx_flexspi_read(&flexspi, source_addr,
                              (unsigned long)&flash_header, sizeof(imx_flash_header_t));
    if (status == 0) {
            if (flash_header.tag == IMX_IVT_HEADER_TAG) {
                image_size = flash_header.img_array[0].offset;
            } else {
                ser_putstr("IVT header check failed!\n");
                return (-1);
            }
        } else {
        return (-1);
    }
#endif

    status = imx_flexspi_read(&flexspi, (source_addr + image_size),
                             dest_addr, sizeof(startup_hdr) + IMX_QNX_IMAGE_SCAN_SIZE);

    if (status == 0) {
        /* Check loaded image in DDR RAM memory */
        startup_hdr_addr = image_scan_2(dest_addr, dest_addr + IMX_QNX_IMAGE_SCAN_SIZE, 0);
        if (startup_hdr_addr != (unsigned long)-1) {
            ser_putstr("QNX image detected, size: ");
            image_size += ((struct startup_header *)(startup_hdr_addr))->stored_size;
#if (IMX_HAB_AUTHENTICATE_CONFIG == 1)
            /* Align QNX-IFS image size to 1024. */
            image_size = IMX_ALIGN(image_size, 1024);
#endif
            ser_putdec(image_size);
            ser_putstr(" bytes\n");
            status = imx_flexspi_read(&flexspi, source_addr, dest_addr, image_size);
        } else {
            ser_putstr("QNX-IFS image format is wrong!\n");
        }
    }
    return status;
}
#endif

/**
 *  Load QNX image file from SD, eMMC memory to DDR memory.
 *
 * @param card_type Memory device type (eSDC, eMMC).
 * @param address   QNX image load address.
 * @param file_name Image file name (e.g. QNX-IFS).
 *
 * @return          QNX image file load status:  SDMMC_OK - no error; SDMMC_ERROR - SDMMC error
 */
static inline int usdhc_load_file(card_type_t card_type, long address, const char *file_name)
{
    sdmmc_t sdmmc;
    int status;
    fat_sdmmc_t fat = {
        .ext = &sdmmc,
        .buf1 = fat_buf1,
        .buf1_len = FAT_FS_INFO_BUF_SIZE,
        .buf2 = fat_buf2,
        .buf2_len = FAT_COMMON_BUF_SIZE,
        .verbose = 0
    };

    if (card_type == eSDC) {
        /* Initialize the sdmmc interface and card, 198MHz clk */
        imx_sdmmc_init_hc(&sdmmc, IMX_USDHC1_BASE, 198000000, SDMMC_VERBOSE_LVL_0);
        if (sdmmc_init_sd(&sdmmc)) {
            ser_putstr("SDMMC card init failed\n");
            status = SDMMC_ERROR;
            goto done;
        }
    } else if (card_type == eMMC) {
        /* Initialize the emmc interface and card, 400MHz clk */
        imx_sdmmc_init_hc(&sdmmc, IMX_USDHC0_BASE, 400000000, SDMMC_VERBOSE_LVL_0);
        if (sdmmc_init_mmc(&sdmmc)) {
            ser_putstr("eMMC flash init failed\n");
            status = SDMMC_ERROR;
            goto done;
        }
    } else {
        ser_putstr("Unknown card type\n");
        status = SDMMC_ERROR;
        goto done;
    }

    if (fat_init(&fat)) {
        ser_putstr("Failed to init fat-fs\n");
        status = SDMMC_ERROR;
        goto done;
    }

    status = fat_copy_named_file((unsigned char *)address, (char *)file_name);

done:
    sdmmc_fini(&sdmmc);

    return status;
}

/**
 * Get information about chip i.MX8 (chip type, chip revision).
 *
 * @param ipc  IPC handle.
 * @param type Pointer to i.MX8 MCU type variable.
 * @param rev  Pointer to i.MX8 MCU revision variable.
 */
static void imx_get_chip_info(sc_ipc_t ipc, uint32_t *type, uint32_t *rev)
{
    uint32_t chip_info = 0U, val;
    sc_err_t sc_status;

    /* Get i.MX8 chip information */
    sc_status = sc_misc_get_control(ipc, SC_R_SYSTEM, SC_C_ID, &chip_info);
    if (sc_status != SC_ERR_NONE) {
        ser_putstr("Get chip information failed.\n");
    }
    /* Get i.MX8 chip variant - used for fused i.MX8QXP chip only */
    (void)sc_misc_otp_fuse_read(ipc, 6U, &val);
    if ((val & 0x0CU) == 0x0CU) {
        *type = IMX_CHIP_TYPE_DUAL_TYPE;
    } else {
        *type = (chip_info & 0x1FU);
    }
    *rev = ((chip_info >> 5) & 0x0FU);
}

/**
 * Initial Program Loader main function. Initializes board (AIPS, clocks, pin routing, console,...),
 * loads QNX image to DDR RAM and start it.
 *
 * @return  Always 0 (never reach, jump to startup code).
 */
int main(void)
{
    paddr_t image;
    char BootKey = IMX_QNX_IMAGE_LOAD_FROM;
    sc_err_t sc_status;
    sc_ipc_t ipc;
    uint32_t scfw_build, scfw_commit;
    uint32_t seco_version, seco_commit;
#if (IMX_ARM_TRUSTED_FW == 1)
    uint64_t atf_commit = 0;
#endif
#if (IMX_HAB_FUSE_CONFIG == 1)
    uint32_t idx;
    uint32_t val;
#endif
#if (IMX_USDHC_FUSE_CONFIG == 1)
    uint32_t emmc_fuse_word = 0;
#endif

    /* Open IPC channel */
    do {
        sc_status = sc_ipc_open(&ipc, IMX_SCU_IPC_MU);
    } while (sc_status != SC_ERR_NONE);

    /* Initialize IPL serial console */
    imx_init_console(ipc);
    /* Get chip information */
    imx_get_chip_info(ipc, &chip_type, &chip_rev);

    ser_putstr("\nWelcome to QNX Neutrino Initial Program Loader for NXP ");
    ser_putstr(IS_IMX8QXP_MCU_TYPE(chip_type) ? "i.MX8QXP" : "i.MX8DXP/DX");
    ser_putstr(" CPU Board (ARM Cortex-A35)\n");

    /* Get SCFW build info */
    sc_misc_build_info(ipc, &scfw_build, &scfw_commit);
    ser_putstr("SCFW build version: ");
    ser_putdec(scfw_build);
    ser_putstr(", SCFW commit: ");
    ser_puthex(scfw_commit);
    ser_putchar('\n');

    if (IS_CHIP_REV_A(chip_rev)) {
        ser_putstr("\n!!! i.MX8QXP A0 CPU in no longer supported by this BSP !!!\n");
        /* coverity[no_escape] - Suppress coverity INFINITE_LOOP error */
        /* For now don't stop A0 CPU from booting */
        //while (1) {}
    } else {
        /* Get SECO FW information (for B0 silicon only) */
        sc_seco_build_info(ipc, &seco_version, &seco_commit);
        ser_putstr("SECO version: ");
        ser_putdec((seco_version >> 16) & 0x3FUL);   ser_putstr(".");
        ser_putdec((seco_version >>  4) & 0x0FFFUL); ser_putstr(".");
        ser_putdec((seco_version >>  0) & 0x0FUL);
        ser_putstr(", commit: "); ser_puthex(seco_commit);
        ser_putchar('\n');
    }

    /* Initialize the system clocks */
    imx_init_clocks(ipc);
    /* Initialize PINs routing needed for IPL */
    imx_init_pinmux(ipc);
#ifndef IMX_ARM_TRUSTED_FW
    /* Start all cores + initialize CCI */
    imx_init_cores(ipc);
#endif
    /* Close IPC channel */
    sc_ipc_close(ipc);

#if (IMX_ARM_TRUSTED_FW == 1)
    atf_commit = imx_sec_firmware_psci(IMX_FSL_SIP_BUILDINFO, IMX_FSL_SIP_BUILDINFO_GET_COMMITHASH, 0x00, 0x00, 0x00);
    if (atf_commit != (uint64_t)IMX_PSCI_NOT_SUPPORTED) {
        ser_putstr("ATF commit: ");
        if (atf_commit != 0U) {
        ser_putstr((char *)&atf_commit);
        ser_putchar('\n');
        } else {
            ser_putstr("Information is not available\n");
        }
    } else {
        ser_putstr("Failed to get ARM commit\n");
    }
#endif


    while (1) {
        image = IMX_QNX_LOAD_ADDR;
        if (BootKey == 0) {
            ser_putstr("Command:\n");
            ser_putstr("Press 'D' for serial download, using the 'sendnto' utility\n");
            ser_putstr("Press 'M' for eSD download, IFS filename MUST be 'QNX-IFS'.\n");
            ser_putstr("Press 'E' for eMMC download, IFS filename MUST be 'QNX-IFS'.\n");
#if (IMX_QSPI_BOOT == 1)
            ser_putstr("Press 'F' for NOR flash download, IFS file MUST be at 0x100000 offset.\n");
#endif
#if (IMX_HAB_FUSE_CONFIG == 1)
            ser_putstr("Press 'R' for HAB fuse word read operation.\n");
            ser_putstr("Press 'W' for HAB fuse word write operation.\n");
#endif
#if (IMX_USDHC_FUSE_CONFIG == 1)
            ser_putstr("Press 'Y' for USDHC fuse word read operation.\n");
            ser_putstr("Press 'X' for USDHC fuse word write operation.\n");
#endif
            BootKey = ser_getchar();
        }
        switch (BootKey) {
            case 'D':
            case 'd':
                ser_putstr("send image now...\n");
                if (image_download_ser(image)) {
                    ser_putstr("download failed...\n");
                    BootKey = 0;
                } else {
                    ser_putstr("download OK...\n");
                }
                break;
            case 'M':
            case 'm':
                ser_putstr("eSDC download...\n");
                if (usdhc_load_file(eSDC, image, "QNX-IFS") == 0) {
                    ser_putstr("load image done.\n");
                    /* Proceed to image scan */
                } else {
                    ser_putstr("Load image failed.\n");
                    BootKey = 0;
                }
                break;
            case 'e':
            case 'E':
                ser_putstr("eMMC download...\n");
                if (usdhc_load_file(eMMC, image, "QNX-IFS") == 0) {
                    ser_putstr("load image done.\n");
                    /* Proceed to image scan */
                } else {
                    ser_putstr("Load image failed.\n");
                    BootKey = 0;
                }
                break;
#if (IMX_QSPI_BOOT == 1)
            case 'F':
            case 'f':
                ser_putstr("NOR flash download...\n");
                if (flexspi_load_file(image, IMX_FLASH_IMAGE_ADDR) == 0) {
                    ser_putstr("Load image done.\n");
                    /* Proceed to image scan */
                } else {
                    ser_putstr("Load image failed.\n");
                    BootKey = 0;
                }
                break;
#endif
#if (IMX_HAB_FUSE_CONFIG == 1)
            case 'R':
            case 'r':
                ser_putstr("\nHAB fuse word values: \n");
                sc_status = sc_ipc_open(&ipc, IMX_SCU_IPC_MU);
                if (sc_status == SC_ERR_NONE) {
                    for (idx = 0; idx <  (sizeof(hab_oem_srk_hash) / sizeof(hab_oem_srk_hash[0])); idx++) {
                        IMX_READ_AND_PRINT_FUSE(hab_oem_srk_hash[idx][0], val);
                    }
                    ser_putstr("\n\n");
                    sc_ipc_close(ipc);
                } else {
                    ser_putstr("Open SC failed!\n");
                }
                    BootKey = 0;
                break;
            case 'W':
            case 'w':
                ser_putstr("\nAre you sure to write predefined HAB fuse word values? \n");
                for (idx = 0; idx < (sizeof(hab_oem_srk_hash) / sizeof(hab_oem_srk_hash[0])); idx++) {
                    IMX_PRINT_FUSE(hab_oem_srk_hash[idx][0], hab_oem_srk_hash[idx][1]);
                }
                ser_putstr("\nConfirm write operation 'y'es/'n'o/'s'kip): ");
                BootKey = ser_getchar();
                ser_putchar(BootKey);
                ser_putstr("\n");
                if ((BootKey == 'y') || (BootKey == 's')) {
                    sc_status = sc_ipc_open(&ipc, IMX_SCU_IPC_MU);
                    if (sc_status == SC_ERR_NONE) {
                        if (BootKey == 'y') {
                            for (idx = 0; idx < (sizeof(hab_oem_srk_hash) / sizeof(hab_oem_srk_hash[0])); idx++) {
                                sc_status = sc_misc_otp_fuse_write(ipc, hab_oem_srk_hash[idx][0], hab_oem_srk_hash[idx][1]);
                                if (sc_status != SC_ERR_NONE) {
                                    ser_putstr("Write fuse word '");
                                    ser_putdec(hab_oem_srk_hash[idx][0]);
                                    ser_putstr("' operation failed!\n");
                                    break;
                                }
                            }
                        }
                        if (sc_status == SC_ERR_NONE) {
                            ser_putstr("Write fuse word operation complete.\n\n");
                            ser_putstr("Configure SECO lifecycle to '"); ser_putdec(IMX_SECOLIFE_CYCLE); ser_putstr("' value?\n");
                            ser_putstr("Confirm write operation ('y'/'n'): ");
                            BootKey = ser_getchar();
                            ser_putchar(BootKey);
                            ser_putstr("\n");
                            if (BootKey == 'y') {
                                sc_status = sc_seco_forward_lifecycle(ipc, IMX_SECOLIFE_CYCLE);
                                if (sc_status == SC_ERR_NONE) {
                                    ser_putstr("seco_forward_lifecycle operation complete.\n");
                                } else {
                                    ser_putstr("seco_forward_lifecycle operation failed!\n");
                                }
                            } else if (BootKey == 'n') {
                                ser_putstr("Write operation was canceled.\n");
                            } else {
                                ser_putstr("Unknown command.\n");
                            }
                        }
                    } else {
                        ser_putstr("Open SC failed!\n");
                    }
                } else if (BootKey == 'n') {
                    ser_putstr("Write operation was canceled.\n");
                } else {
                    ser_putstr("Unknown command.\n");
                }
                ser_putstr("\n");
                BootKey = 0;
                break;
#endif
#if (IMX_USDHC_FUSE_CONFIG == 1)
            case 'Y':
            case 'y':
                ser_putstr("\nRead USDHC fuse values\n");
                sc_status = sc_ipc_open(&ipc, IMX_SCU_IPC_MU);
                if (sc_status == SC_ERR_NONE) {
                    emmc_fuse_word = 0;
                    sc_status = sc_misc_otp_fuse_read(ipc, IMX_USDHC_FUSE_ROW_INDEX, &emmc_fuse_word);
                    if (sc_status != SC_ERR_NONE) {
                        ser_putstr("USDHC read fuse fail!\n");
                        sc_ipc_close(ipc);
                        break;
                    }
                    ser_putstr("Fuse value: 0x");
                    ser_puthex(emmc_fuse_word);
                    ser_putstr("\n");
                    sc_ipc_close(ipc);
                } else {
                    ser_putstr("Open SC failed!\n");
                }
                BootKey = 0;
                break;
            case 'X':
            case 'x':
                ser_putstr("\nWrite USDHC fuse values\n");
                sc_status = sc_ipc_open(&ipc, IMX_SCU_IPC_MU);
                if (sc_status == SC_ERR_NONE) {
                    sc_status = sc_misc_otp_fuse_write(ipc, IMX_USDHC_FUSE_ROW_INDEX, IMX_USDHC_FUSE_VALUE);
                    if (sc_status != SC_ERR_NONE) {
                        ser_putstr("USDHC write fuse fail!\n");
                        sc_ipc_close(ipc);
                        break;
                    }
                    ser_putstr("USDHC write fuse OK.\n");
                    sc_ipc_close(ipc);
                } else {
                    ser_putstr("Open SC failed!\n");
                }
                BootKey = 0;
                break;
#endif
            default:
                ser_putstr("Unknown command.\n");
                BootKey = 0;
        }
        if (BootKey == 0) {
            continue;
        }
#if (IMX_CACHE_EN == 1)
        /* Enable DCache and MMU */
        imx_enable_mmu();
#endif
#if (IMX_HAB_AUTHENTICATE_CONFIG == 1)
        ser_putstr("Authenticate QNX-IFS image...\n");
        if (imx_ahab_authenticate_image(image) != 0) {
    #if (IMX_CACHE_EN == 1)
            /* Disable DCache and MMU */
            imx_disable_mmu();
    #endif
            continue;
        }
        ser_putstr("QNX-IFS image authenticated.\n");
#endif
        /* Scan loaded image in DDR RAM memory */
        image = image_scan_2(image, image + 0x200, 1);
#if (IMX_CACHE_EN == 1)
        /* Disable DCache and MMU */
        imx_disable_mmu();
#endif
        if (image != (paddr_t)-1) {
            ser_putstr("Found image               @ 0x");
            ser_puthex(image);
            ser_putstr("\n");
            image_setup(image);

            ser_putstr("Jumping to startup        @ 0x");
            ser_puthex(startup_hdr.startup_vaddr);
            ser_putstr("\n\n");
#ifndef IMX_ARM_TRUSTED_FW
            /* Wake secondary cores, jump to startup code */
            wake_secondary_core((paddr_t)startup_hdr.startup_vaddr);
#endif
            image_start(image);

            /* Never reach here */
            return 0;
        }
        ser_putstr("Image_scan failed...\n");
    }
    return 0;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/boards/imx8qxp-cpu/main.c $ $Rev: 904597 $")
#endif
