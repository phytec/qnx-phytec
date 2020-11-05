/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
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

#ifndef IMX_IPL_H_
#define IMX_IPL_H_

#include <aarch64/mx8xp.h>
#include "variant.h"

#ifndef __ASM__
#include <hw/nxp/imx8/sci/sci.h>
#endif

/* i.MX8 chip revision list */
#define IMX_CHIP_REV_A              0x00
#define IMX_CHIP_REV_B              0x01

/* i.MX8 chip type list */
#define IMX_CHIP_TYPE_QUAD_MAX      0x01
#define IMX_CHIP_TYPE_QUAD_X_PLUS   0x02
#define IMX_CHIP_TYPE_DUAL_TYPE     0x10

#define IMX_LE_2_BE_32(l) \
    ((((l) & 0x000000FF) << 24) | \
    (((l) & 0x0000FF00) << 8)  | \
    (((l) & 0x00FF0000) >> 8)  | \
    (((l) & 0xFF000000) >> 24))

#define IMX_ALIGN(val, align_size) ((val + (align_size - 1)) & ~(align_size - 1))

/* Check imx8 MCU type macros */
#define IS_IMX8QXP_MCU_TYPE(val)    (val == IMX_CHIP_TYPE_QUAD_X_PLUS)
#define IS_IMX8DUAL_MCU_TYPE(val)   (val == IMX_CHIP_TYPE_DUAL_TYPE)
/* Check imx8 MCU revision macros */
#define IS_CHIP_REV_A(val)          (val == IMX_CHIP_REV_A)
#define IS_CHIP_REV_B(val)          (val == IMX_CHIP_REV_B)

#if defined(IMX_ASM_MACROS_EN)
/* Branch code according the exception level */
.macro  branch_el_mode, reg, label4el3, label4el2, label4el1
    mrs     \reg, CurrentEL
    cmp     \reg, 0x0C
    b.eq    \label4el3
    cmp     \reg, 0x08
    b.eq    \label4el2
    cmp     \reg, 0x04
    b.eq    \label4el1
.endm
#endif

#ifndef __ASM__
void imx_init_console(sc_ipc_t ipc);
void imx_init_clocks(sc_ipc_t ipc);
void imx_init_pinmux(sc_ipc_t ipc);
void imx_init_lpuart(uintptr_t port, uint32_t baud, uint32_t clk, uint32_t osr);
#ifndef IMX_ARM_TRUSTED_FW
    extern void imx_init_cores(sc_ipc_t ipc);
    extern void wake_secondary_core(paddr_t addr);
#endif
#if (IMX_HAB_AUTHENTICATE_CONFIG == 1)
    int imx_ahab_authenticate_image(paddr_t start);
#endif
#endif

#define IMX_IVT_HEADER_TAG      0x87
#define IMX_HASH_MAX_LEN        64
#define IMX_IV_MAX_LEN          32
#define IMX_MAX_IMG_NUM         6

#if !defined(IMX_ASM_MACROS_EN)
typedef struct {
    uint32_t    offset;
    uint32_t    size;
    uint64_t    load_addr;
    uint64_t    entry;
    uint32_t    flags;
    uint32_t    meta_data;
    uint8_t     hash[IMX_HASH_MAX_LEN];
    uint8_t     iv[IMX_IV_MAX_LEN];
} __attribute__((packed)) imx_image_array_t;

typedef struct {
    uint8_t     version;
    uint16_t    length;
    uint8_t     tag;
    uint32_t    flags;
    uint16_t    sw_version;
    uint8_t     fuse_version;
    uint8_t     num_images;
    uint16_t    signature_block_offset;
    uint16_t    reserved;
    imx_image_array_t img_array[IMX_MAX_IMG_NUM];

} __attribute__((packed)) imx_flash_header_t;
#endif

#endif /* IMX_IPL_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
#ifdef __ASM__
__SRCVERSION "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/boards/imx8qxp-cpu/imx_ipl.h $ $Rev: 904597 $"
#else
__SRCVERSION( "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/boards/imx8qxp-cpu/imx_ipl.h $ $Rev: 904597 $" )
#endif
#endif
