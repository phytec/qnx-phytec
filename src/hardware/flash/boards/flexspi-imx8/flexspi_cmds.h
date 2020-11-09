/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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

#ifndef FLEXSPI_CMDS_H_
#define FLEXSPI_CMDS_H_

#include <stdint.h>

#include "imx_fc_flexspi.h"

/**
 * @file       flexspi-imx8/flexspi_cmds.h
 * @addtogroup ffs3_cmds Commands
 * @{
 */

/* Used address range */
#define ADDR_MODE_4BYTE             32

/** @name Device status register common definition */
#define DEVICE_SR_WIP               (1 << 0) /**< Write in Progress (WIP) bit */
#define DEVICE_SR_WEL               (1 << 1) /**< Write Enable Latch (WEL) bit */
#define DEVICE_SR_SRWD              (1 << 7) /**< Status Register Write Disable (SRWD) bit */
/** @} */

/** @name Device flag status register common definition */
#define DEVICE_FSR_ERASE_ERR        (1 << 5) /**< ERASE operation has succeeded (0) or failed (1). */
/** @} */

/** @name Flash commands */
/* Erase */
#define FLASH_OPCODE_SE4B           0xDC     /**< Sector erase 4B */
/* Read */
#define FLASH_OPCODE_MAIN_DRV_READ  FLASH_OPCODE_8READ4B
#define FLASH_OPCODE_READ4B         0x13     /**< Read data bytes 4B */
#define FLASH_OPCODE_FREAD4B        0x0C     /**< Fast Read data bytes 4B */
#define FLASH_OPCODE_4READ4B        0xEC     /**< 4-BYTE QUAD I/O FAST READ */
#define FLASH_OPCODE_8READ4B        0xCC     /**< 4-BYTE OCTAL I/O FAST READ */
#define FLASH_OPCODE_8READ4B_DDR    0xFD     /**< 4-BYTE DDR OCTAL I/O FAST READ */
/* DDR command is not used since it is causing miss alignment read issues */
/* Write */
#define FLASH_OPCODE_MAIN_DRV_WRITE FLASH_OPCODE_8PP_EXT_4B
#define FLASH_OPCODE_PP4B           0x12     /**< Page program 4B */
#define FLASH_OPCODE_4PP_EXT_4B     0x3E     /**< 4-BYTE QUAD INPUT EXTENDED FAST PROGRAM */
#define FLASH_OPCODE_8PP_EXT_4B     0x8E     /**< 4-BYTE OCTAL INPUT EXTENDED FAST PROGRAM */
/* Other */
#define FLASH_OPCODE_ENQIO          0x35     /**< Enter Quad IO mode */
#define FLASH_OPCODE_RESQIO         0xF5     /**< Reset Quad IO mode */
#define FLASH_OPCODE_RDID           0x9F     /**< Read JEDEC ID in SPI mode */
#define FLASH_OPCODE_WREN           0x06     /**< Write enable */
#define FLASH_OPCODE_WRSR           0x01     /**< Write status register */
#define FLASH_OPCODE_RDSR           0x05     /**< Read status register */
#define FLASH_OPCODE_EN4B           0xB7     /**< Enable 4 Byte address mode */
#define FLASH_OPCODE_EXIT4B         0xE9     /**< Exit 4 Byte address mode */
/* Specific */
/* Macronix */
#define FLASH_OPCODE_RDSCUR         0x2B     /**< Read security register */
/* Micron */
#define FLASH_OPCODE_RFLAGR         0x70     /**< Read flag status register */
#define FLASH_OPCODE_RNVCR          0xB5     /**< Read Non-Volatile Configuration Register */
#define FLASH_OPCODE_WNVCR          0xB1     /**< Write Non-Volatile Configuration Register */
#define FLASH_OPCODE_RVCR           0x85     /**< Read Volatile Configuration Register */
#define FLASH_OPCODE_WVCR           0x81     /**< Write Volatile Configuration Register */
/** @} */

#define FLASH_PROGRAM_WAIT_TIME     (1000 * 1000 * 20)   /**< 20ms */

int iswriting(imx_fspi_t *dev);

int wait_for_completion(imx_fspi_t *dev);

int write_status(imx_fspi_t *dev, uint8_t *stat_reg);

int read_status(imx_fspi_t *dev, uint8_t *stat_reg);

int read_ident(imx_fspi_t *dev, int *manufact_id, int *device_id, uint32_t *size);

int post_ident_cfg(imx_fspi_t *dev);

int pre_ident_cfg(imx_fspi_t *dev);

int sector_erase(imx_fspi_t *dev, const int offset);

int page_program(imx_fspi_t *dev, int offset, int len, uint8_t *data);

int read_from(imx_fspi_t *dev, int offset, int len, uint8_t *buffer);

int init_lut(imx_fspi_t *dev);

/** @} */

#endif /* FLEXSPI_CMDS_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/flexspi_cmds.h $ $Rev: 876662 $")
#endif
