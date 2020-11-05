/*
 * $QNXLicenseC:
 * Copyright 2017-2018 NXP
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


#ifndef BOARD_H_
#define BOARD_H_

/**
 * i.MX startup source file.
 *
 * @file       imx8qxp-cpu/board.h
 * @addtogroup ipl
 * @{
 */

/*!
 * @name IPL console UART configuration.
 */
/*@{*/
/** UART console base address */
#define IMX_CONSOLE_UART_BASE           IMX_LPUART0_BASE
/** UART console baud rate */
#define IMX_CONSOLE_UART_BAUD_RATE      115200U
/*@}*/

/** 8MHz from CPclk */
#define	IMX_COUNTER_FREQUENCY           8000000

/**
 * QNX-IFS image position in NOR flash memory.
 */
#define IMX_FLASH_IMAGE_ADDR            0x100000

/**
 * Load QNX image from:\n
 *   0  - Boot source defined by key from debug console.\n
 *  'D' - serial download, using the 'sendnto' utility.\n
 *  'M' - SDMMC download, IFS filename MUST be 'QNX-IFS'.\n
 *  'E' - eMMC download, IFS filename MUST be 'QNX-IFS'.\n
 *  'F' - NOR flash download, IFS filename be at 0x100000 offset.
 */
#define IMX_QNX_IMAGE_LOAD_FROM         0
#define IMX_QNX_IMAGE_SCAN_SIZE         0x1000

/**
 * Setting IMX_QSPI_BOOT to 1 enables QNX-IFS image download
 * from NOR flash memory.
 */
#define IMX_QSPI_BOOT                   1

/**
 * Enables USDHC fuse word configuration in boot menu:
 * Press 'Y' for USDHC fuse word read operation.
 * Press 'X' for USDHC fuse word write operation.
 */
#define IMX_USDHC_FUSE_CONFIG              0
/**
 * USDHC fuse row index.
 */
#define IMX_USDHC_FUSE_ROW_INDEX           19
/**
 * MMC Bus Width: 8-bit DDR
 * Boot Speed: High
 * Fast Boot Acknowledge: Enabled
 * Fast Boot: Enabled
 */
#define IMX_USDHC_FUSE_VALUE               0x27

/*!
 * @name AHAB configuration.
 */
/*@{*/
/**
 * Setting IMX_HAB_AUTHENTICATE_CONFIG to 1
 * enables QNX-IFS image HAB authenticate process.
 */
#define IMX_HAB_AUTHENTICATE_CONFIG     0

/**
 * Enable HAB fuses configuration in boot menu:
 * Press 'R' for HAB fuse word read operation.
 * Press 'W' for HAB fuse word write operation.
 */
#define IMX_HAB_FUSE_CONFIG             0
/**
 * Predefined HAB fuse values (example fuse values)
 * format: {fuse_row_index, fuse_value}
 */
#define IMX_HAB_FUSE_VALUES             {730, 0x8ED0A37D}, \
                                        {731, 0x084E7DB7}, \
                                        {732, 0x2B529253}, \
                                        {733, 0x872C3D4B}, \
                                        {734, 0xA7E3409E}, \
                                        {735, 0x6164F88B}, \
                                        {736, 0xF279B732}, \
                                        {737, 0x978A7B68}, \
                                        {738, 0xBC56312A}, \
                                        {739, 0x31CB64A6}, \
                                        {740, 0xA35B7653}, \
                                        {741, 0x79964BDB}, \
                                        {742, 0xD3C4291B}, \
                                        {743, 0x0E01A8A5}, \
                                        {744, 0xA743CB0B}, \
                                        {745, 0xC364E7A7}
/** SECO life cycle value: SCU FW Closed
 *      SECO_LIFECYCLE_DEFAULT      = (1U << 0)    Default fab mode (early_fuses_pgrm not blown)
 *      SECO_LIFECYCLE_FAB          = (1U << 1)    Fab mode
 *      SECO_LIFECYCLE_NO_SECRETS   = (1U << 2)    No secrets
 *      SECO_LIFECYCLE_SECRETS      = (1U << 3)    With Secrets
 *      SECO_LIFECYCLE_SC_FW_CLSD   = (1U << 4)    SCU FW Closed
 *      SECO_LIFECYCLE_SECO_FW_CLSD = (1U << 5)    SECO FW Closed
 *      SECO_LIFECYCLE_CLOSED       = (1U << 6)    Closed
 *      SECO_LIFECYCLE_CLOSED_FW    = (1U << 7)    Closed with FW
 *      SECO_LIFECYCLE_PART_RTN     = (1U << 8)    Partial field return
 *      SECO_LIFECYCLE_RTN          = (1U << 9)    Field return
 *      SECO_LIFECYCLE_NO_RTN       = (1U << 10)   No Return
 */
#define IMX_SECOLIFE_CYCLE              (1U << 4)
/*@}*/

#if (IMX_ARM_TRUSTED_FW == 1)
    #define IMX_ATF_LENGTH              0x020000
    #define IMX_ARM_TRUSTED_FW_BINARY   "bl31.bin"
#else
    #define IMX_ATF_LENGTH              0x000000
#endif

/** Message Unit used for SCFW IPC calls */
#if (IMX_ARM_TRUSTED_FW == 1)
    #define IMX_SCU_IPC_MU              IMX_MU1_BASE
#else
    #define IMX_SCU_IPC_MU              IMX_MU0_BASE
#endif

/**
 * Setting IMX_CACHE_EN to 1 enables I/D caches feature
 * (improve QNX-IFS scan or authentication time.
 */
#define IMX_CACHE_EN                    1

/** IPL memory base address */
#define IMX_IPL_MEM_BASE                0x80000000

/** IPL stack size */
#define IMX_IPL_STACK_SIZE              0x5000;

/** SDRAM base address (used for MMU configuration) */
#define IMX_SDRAM_BASE                  0x80000000
/** SDRAM size (used for MMU configuration) */
#define IMX_SDRAM_SIZE                  (2048 * 1024 * 1024)

/**
 *  The address into which you load the IFS image MUST NOT
 *  overlap the area it will be run from.  The image will
 *  be placed into the correct location by 'image_setup'.
 *
 *  The image is configured to be uncompressed to, and
 *  run from, 0x80800000.  The compressed image is loaded
 *  higher up, and we must leave a big enough gap so they
 *  do not overlap when being decompressed.
 *  Loading the image to 0x88000000 leaves a 120MB gap.
 */
#define IMX_QNX_LOAD_ADDR               0x88000000

#endif


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
#ifdef __ASM__
__SRCVERSION "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/boards/imx8qxp-cpu/board.h $ $Rev: 886382 $"
#else
__SRCVERSION( "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/boards/imx8qxp-cpu/board.h $ $Rev: 886382 $" )
#endif
#endif
