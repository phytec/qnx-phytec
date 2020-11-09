/*
 * $QNXLicenseC:
 * Copyright 2020, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017, 2019 NXP
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/slog.h>
#include <sys/slogcodes.h>
#include <sys/mman.h>
#include <pthread.h>
#include <string.h>

#include "imx_fc_flexspi.h"
#include "flexspi_cmds.h"
#include "f3s_flexspi_s26hs.h"

//#define DEBUG

#include "aarch64/imx8_common/imx_gpio.h"
extern uintptr_t gpio_base;

/**
 * @file       flexspi-imx8/f3s_flexspi_s26hs.c
 * @addtogroup ffs3_fc Flash Controller
 * @{
 */


/** @name RX/TX constants */
#define FLEXSPI_RX_BUF_SIZE  (512)
#define FLEXSPI_MAX_BURST_TX (16)
#define FLEXSPI_MAX_BURST_RX (128)

/** @name LUT device command indexes */
#define IMX_FLEXSPI_LUT_AUTOSELECT     1 /* 2 sequences */
#define IMX_FLEXSPI_LUT_READ_STATUS    3
#define IMX_FLEXSPI_LUT_READ           4
#define IMX_FLEXSPI_LUT_ENTER_SFDP_ASO 5
#define IMX_FLEXSPI_LUT_SW_RESET       6
#define IMX_FLEXSPI_LUT_PRE_ERASE      7
#define IMX_FLEXSPI_LUT_CHIP_ERASE     8
#define IMX_FLEXSPI_LUT_PROGRAM_WORD   9
#define IMX_FLEXSPI_LUT_WRITE          10
#define IMX_FLEXSPI_LUT_READ_VCONFIG2  11
#define IMX_FLEXSPI_LUT_WRITE_VCONFIG2 12
#define IMX_FLEXSPI_LUT_SECTOR_ERASE   13
#define IMX_FLEXSPI_LUT_CONFIG_ERASE   14

#define IMX_FLEXSPI_LUT_SIZE           15 /**< Number of commands in LUT */

/** @name Additional flexspi instructions */
#define IMX_FSPI_INSTR_RADDR_DDR            0x22
#define IMX_FSPI_INSTR_CADDR_DDR            0x23
#define IMX_FSPI_INSTR_DUMMY_RWDS_DDR       0x2D

/** @name Other macros */
#define SA_TO_OFFSET(_sa)      ((_sa) << 1)
#define DEVICE_ID_TABLE_OFFSET SA_TO_OFFSET(0x800)
#define DEVICE_ID_TABLE_SIZE   (32)


static int flexspi_s26hs_write_data(imx_fspi_t *dev, uint32_t offset, uint8_t *addr, uint32_t data_size);
static int flexspi_s26hs_read_data(imx_fspi_t *dev, uint32_t offset, uint8_t *buffer, uint32_t size);
static int flexspi_s26hs_send_ip_cmd(imx_fspi_t *dev, uint8_t lut_index, uint8_t num_seq, uint32_t data_size_override);
static int flexspi_s26hs_wait_for_standby(imx_fspi_t *dev);


/**
 * Read non-volatile configuration register 2.
 *
 * @param[in]  dev      Low level driver handle.
 * @param[out] cfg      Configuration register 2 value. (2 bytes)
 *
 * @return 0 on success, -1 on failure.
 */
static int flexspi_s26hs_read_nvconfig2(imx_fspi_t *dev, uint8_t *cfg)
{
    int ret;

    ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_AUTOSELECT, 2, 0);
    if (ret >= 0) {
        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_READ_VCONFIG2, 1, 0);
    }
    if (ret >= 0) {
        ret = flexspi_s26hs_read_data(dev, 0, cfg, 2);
    }
    if (ret < 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "IP command failed.\n");
        return -1;
    }
    return 0;

}

/**
 * Write to non-volatile configuration register 2.
 *
 * @param[in]  dev      Low level driver handle.
 * @param[in]  cfg      Configuration register 2 value. (2 bytes)
 *
 * @return 0 on success, -1 on failure.
 */
static int flexspi_s26hs_write_nvconfig2(imx_fspi_t *dev, uint8_t *cfg)
{
    int ret;

    out32(dev->vbase + IMX_FLEXSPI_IPCR0, 0x0); /* Write address to controller */

    /* Clear Tx FIFO */
    out32(dev->vbase + IMX_FLEXSPI_IPTXFCR, in32(dev->vbase + IMX_FLEXSPI_IPTXFCR) | IMX_FLEXSPI_IPTXFCR_CLRIPTXF_MASK);

    /* Set interrupt empty space threshold */
    imx_flexspi_set_tx_watermark(dev, 2);

    /* Wait for available space (watermark) */
    while (!(in32(dev->vbase + IMX_FLEXSPI_INTR) & IMX_FLEXSPI_INTR_IPTXWE_MASK ));

    /* Fill-up initial data */
    memcpy((void*)(dev->vbase + IMX_FLEXSPI_TFDR0), cfg, 2);

    ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_AUTOSELECT, 2, 0);
    if (ret >= 0) {
        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_WRITE_VCONFIG2, 1, 0);
    }
    if (ret >= 0) {
        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_WRITE, 1, 2);
    }
    if (ret >= 0) {
        /* Clear IP and TXWE to push data into device */
        out32(dev->vbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPTXWE_MASK);
    }
    if (ret < 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "IP command failed.\n");
        return -1;
    }
    if (flexspi_s26hs_wait_for_standby(dev) < 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Performing soft reset of flash device.\n");
        flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_SW_RESET, 1, 0);
        return -1;
    }
    return 0;
}

/**
 * Read status register of the memory device.
 *
 * @param[in]  dev      Low level driver handle.
 * @param[out] buffer   Memory status register.
 *
 * @return 0 always.
 */
static int flexspi_s26hs_read_status(imx_fspi_t *dev, uint8_t *buffer)
{
    flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_READ_STATUS, 1, 0/*bytes*/);
    flexspi_s26hs_read_data(dev, 0x0, buffer, 2/*bytes*/);
    return 0;
}

/**
 * Second part of controller initialization.
 *
 * @param dev Low level driver handle.
 *
 * @return 0 on success, -1 on error.
 */
static int flexspi_s26hs_reconfig(imx_fspi_t* dev)
{
    int ret;
    uint8_t cfg2[2];

    /* Select 1-bit ECC instead of 2-bit hardware ECC.
     * This allows multiple writes to same 16-byte halfpage that is required by ffs3 lib.
     * Notes:
     * - devf should be used with -x, to enable software ECC.
     * - Writing more than once to a particular halfpage disables 1-bit hardware ECC.
     * - Only FFS3 extents are written more than once. So extents are protected only by software ECC.
     * - Other halfpages not belonging to extents will have 1-bit hardware ECC + software ECC.
     */
    ret = flexspi_s26hs_read_nvconfig2(dev, cfg2);
    if (ret >= 0) {
        /* CFR2V[5]: ECC12S:
         *     0 = 1-bit ECC Error Detection/Correction and 2-bit ECC error detection
         *     1 = 1-bit ECC Error Detection/Correction
         */
        slogf(_SLOGC_FS_FFS, _SLOG_INFO, "Read: nonvolatile config2=0x%02x%02x", cfg2[0], cfg2[1]);

        if ((cfg2[1] & 0x20) == 0) {
            slogf(_SLOGC_FS_FFS, _SLOG_WARNING, "Updating: nonvolatile config2...");
            // Erase nonvolatile cfg2
            ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_AUTOSELECT, 2, 0);
            if (ret >= 0) {
                ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_CONFIG_ERASE, 1, 0);
            }
            if (ret >= 0) {
                cfg2[1] |= 0x20;
                ret = flexspi_s26hs_write_nvconfig2(dev, cfg2);
            }
            if (ret >= 0) {
                /* Check volatile config2 */
                ret = flexspi_s26hs_read_nvconfig2(dev, cfg2);
                if ((ret == 0) && (cfg2[1] & 0x20)) {
                    slogf(_SLOGC_FS_FFS, _SLOG_WARNING, "Updated: volatile config2=0x%02x%02x", cfg2[0], cfg2[1]);
                }
                else {
                    slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Failed to update volatile config2=0x%02x%02x", cfg2[0], cfg2[1]);
                }
            }
        }
    }
    if (ret < 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Failed to read/erase/write volatile config2!");
        return -1;
    }

    return 0;
}

/**
 * FLEXSPI interrupt handler.
 *
 * @param area Low level driver.
 * @param id   Unused parameter.
 *
 * @return NULL or EVENT
 */
const struct sigevent *flexspi_s26hs_qspi_intr(void *area,  __attribute__((unused)) int id)
{
    imx_fspi_t *dev = area;

    if (dev->irq_requested) {
        dev->irq_requested = 0;
        if (dev->rx_data_len > 0) {
            out32(dev->vbase + IMX_FLEXSPI_INTEN, IMX_FLEXSPI_INTEN_IPRXWAEN_MASK); /* Enable RX watermark interrupt */
        }
        else {
            out32(dev->vbase + IMX_FLEXSPI_INTEN, 0x0); /* Disable all interrupts */
        }
        return &dev->fspievent;
    }
    if (dev->rx_data_len > 0) {
        /* Read data from hw */
        memcpy_isr(dev->buf, (void*)(dev->vbase + IMX_FLEXSPI_RFDR0), dev->rx_watermark);
        /* FIFO shift */
        out32(dev->vbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPRXWA_MASK);
        /* Check if more data is expected */
        dev->rx_data_len = dev->rx_data_len - dev->rx_watermark;
        if (dev->rx_data_len == 0) {
            /* Return from interrupt */
            return &dev->fspievent;
        }
        /* Update pointer value */
        dev->buf = dev->buf + dev->rx_watermark;
        /* Update watermark value */
        dev->rx_watermark = min(dev->rx_data_len, IMX_FSPI_MAX_RX_FIFO_WINDOW);
        /* Write updated watermark value to hw */
        imx_flexspi_set_rx_watermark(dev, dev->rx_watermark);
    }
    return NULL;
}

/**
 * Creates look-up table entry.
 *
 * @param dev     Low level driver handle.
 * @param index   Index in look-up table.
 * @param lutcmd0 Record 0.
 * @param lutcmd1 Record 1.
 * @param lutcmd2 Record 2.
 * @param lutcmd3 Record 3.
 *
 * @return EOK always.
 */
static int flexspi_s26hs_write_lut(imx_fspi_t *dev, uint8_t index, imx_fspi_lut_t *lutcmd)
{
    uint8_t inner_index = index * 4;

    out32(dev->vbase + IMX_FLEXSPI_LUTa(inner_index),   lutcmd[0].U);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(++inner_index), lutcmd[1].U);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(++inner_index), lutcmd[2].U);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(++inner_index), lutcmd[3].U);
    return EOK;
}

/**
 * Command look-up table initialization.
 *
 * @param dev Low level driver handle.
 */
static int flexspi_s26hs_init_lut(imx_fspi_t *dev)
{
    imx_fspi_lut_t lutAutoSelect[8];
    imx_fspi_lut_t lutIDSFE1_3_1[4];
    imx_fspi_lut_t lutRDVSTR_2_0[4];
    imx_fspi_lut_t lutRead[4];
    imx_fspi_lut_t lutASOEXT_1_1[4];
    imx_fspi_lut_t lutERCHIP_6_0a[4];
    imx_fspi_lut_t lutERCHIP_6_0b[4];
    imx_fspi_lut_t lutPGWORD_4_0[4];
    imx_fspi_lut_t lutWrite[4];
    imx_fspi_lut_t lutRDNCR2_4_0[4];
    imx_fspi_lut_t lutPGNCR2_4_0[4];
    imx_fspi_lut_t lutERSCTR_6_0[4];
    imx_fspi_lut_t lutERNC12_3_0[4];

    lutAutoSelect[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutAutoSelect[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutAutoSelect[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutAutoSelect[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Data */ );

    lutAutoSelect[4] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutAutoSelect[5] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x55  /* Row Address */ );
    lutAutoSelect[6] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x02  /* Col Address */ ); /* Total Addr = 0x2AA */
    lutAutoSelect[7] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x55  /* Data */ );

    /* Read ID */
    lutIDSFE1_3_1[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutIDSFE1_3_1[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutIDSFE1_3_1[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutIDSFE1_3_1[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x90  /* Data */ );

    /* Read status register */
    lutRDVSTR_2_0[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutRDVSTR_2_0[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutRDVSTR_2_0[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutRDVSTR_2_0[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x70  /* Data */ ); /* Data = 0x0070 */

    /* Read */
    lutRead[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR,        IMX_FSPI_PAD_8, 0xA0, /* 1:Read, 0:NU_Target_Mem, 1:Linear Burst, 00000:NU */
                                               IMX_FSPI_INSTR_RADDR_DDR,      IMX_FSPI_PAD_8, 0x18  /* =24 bits */ );
    lutRead[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CADDR_DDR,      IMX_FSPI_PAD_8, 0x10, /* =16 bits */
                                               IMX_FSPI_INSTR_DUMMY_RWDS_DDR, IMX_FSPI_PAD_8, dev->dummy  /* Latency count */ );
    lutRead[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_READ_DDR,       IMX_FSPI_PAD_8, 0x01, /* 1 byte */
                                               IMX_FSPI_INSTR_STOP,           0x00,           0x00  /* STOP */ );

    /* SW Reset / ASO Exit */
    lutASOEXT_1_1[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutASOEXT_1_1[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutASOEXT_1_1[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutASOEXT_1_1[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xF0  /* Data */ );

    /* Pre-erase */
    lutERCHIP_6_0a[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                      IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutERCHIP_6_0a[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                      IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutERCHIP_6_0a[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                      IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutERCHIP_6_0a[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                      IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x80  /* Data */ );

    /* Chip Erase */
    lutERCHIP_6_0b[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                      IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutERCHIP_6_0b[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                      IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutERCHIP_6_0b[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                      IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutERCHIP_6_0b[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                      IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x10  /* Data */ );

    /* Pre-write */
    lutPGWORD_4_0[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutPGWORD_4_0[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutPGWORD_4_0[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutPGWORD_4_0[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xA0  /* Data */ );

    /* Write */
    lutWrite[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR,        IMX_FSPI_PAD_8, 0x20, /* 0:Write, 0:NU_Target_Mem, 1:Linear_Burst, 00000:NU */
                                                IMX_FSPI_INSTR_RADDR_DDR,      IMX_FSPI_PAD_8, 0x18  /* =24 bits */ );
    lutWrite[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CADDR_DDR,      IMX_FSPI_PAD_8, 0x10, /* =16 bits */
                                                IMX_FSPI_INSTR_WRITE_DDR,      IMX_FSPI_PAD_8, 0x10  /* bytes */ );
    lutWrite[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_STOP,           0x00,           0x00, /* STOP */
                                                IMX_FSPI_INSTR_STOP,           0x00,           0x00  /* STOP */ );

    /* Read non-volatile config register 2 */
    lutRDNCR2_4_0[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutRDNCR2_4_0[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutRDNCR2_4_0[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutRDNCR2_4_0[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xCA  /* Data */ );

    /* Write volatile config register 2 */
    lutPGNCR2_4_0[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutPGNCR2_4_0[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutPGNCR2_4_0[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutPGNCR2_4_0[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x3B  /* Data */ );

    /* Sector Erase */
    lutERSCTR_6_0[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR,   IMX_FSPI_PAD_8, 0x20, /* 0:Write, 0:NU_Target_Mem, 1:Linear Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_RADDR_DDR, IMX_FSPI_PAD_8, 0x18  /* 24-bits */ );
    lutERSCTR_6_0[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CADDR_DDR, IMX_FSPI_PAD_8, 0x10, /* 16-bits */
                                                     IMX_FSPI_INSTR_CMD_DDR,   IMX_FSPI_PAD_8, 0x00  /* Data */ );
    lutERSCTR_6_0[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR,   IMX_FSPI_PAD_8, 0x30, /* Data */
                                                     IMX_FSPI_INSTR_STOP,      0x00,           0x00  /* Stop */ );

    /* Erase non-volatile configuration registers 1 and 2 */
    lutERNC12_3_0[0] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* 0:Write, 0:NU_Target_Mem, 0:Wrapped Burst, 00000:NU */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00  /* Row Address */ );
    lutERNC12_3_0[1] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Row Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xAA  /* Row Address */ );
    lutERNC12_3_0[2] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Col Address */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x05  /* Col Address */ ); /* Total Addr = 0x555 */
    lutERNC12_3_0[3] = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0x00, /* Data */
                                                     IMX_FSPI_INSTR_CMD_DDR, IMX_FSPI_PAD_8, 0xC8  /* Data */ );

    /* Setup LUT begin */
    imx_flexspi_unlock_lut(dev);

    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_AUTOSELECT,     lutAutoSelect);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_AUTOSELECT+1,   lutAutoSelect+4);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_READ_STATUS,    lutRDVSTR_2_0);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_READ,           lutRead);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_ENTER_SFDP_ASO, lutIDSFE1_3_1);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_SW_RESET,       lutASOEXT_1_1);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_PRE_ERASE,      lutERCHIP_6_0a);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_CHIP_ERASE,     lutERCHIP_6_0b);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_PROGRAM_WORD,   lutPGWORD_4_0);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_WRITE,          lutWrite);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_READ_VCONFIG2,  lutRDNCR2_4_0);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_WRITE_VCONFIG2, lutPGNCR2_4_0);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_SECTOR_ERASE,   lutERSCTR_6_0);
    flexspi_s26hs_write_lut(dev, IMX_FLEXSPI_LUT_CONFIG_ERASE,   lutERNC12_3_0);

    /* Setup LUT end */
    imx_flexspi_lock_lut(dev);

    return EOK;
}


/**
 * First part of controller initialization.
 *
 * @return File descriptor.
 */
static int flexspi_s26hs_reopen(imx_fspi_t *dev)
{
    uint32_t reg;

    /*
     * NOTES:
     * - The following init sequence generally follows recommendation from:
     *       IMX8DQXPRM-RevE-06-2019, section 18.2.5.1 Flexspi Initialization
     * - Must disable MDIS (i.e. enable flexspi) before assigning LUT
     * - Watermark must be multiple of 8.
     * - IMX_FLEXSPI_LUTa() #define does not put param in parenthesis!!!
     * - DLLACR needs to be set accg to "18.2.4.14.4 DLL configuration for sampling" to fix the first-byte-issue.
     * - See iMX8 RM section 18.2.4.6 - Otherwise, by default, offset will refer to row (i.e. half-page; 16-byte) access only.
     */

    /* Enter stop mode */
    reg = in32(dev->vbase + IMX_FLEXSPI_MCR0);
    reg |= IMX_FLEXSPI_MCR0_MDIS_MASK;
    out32(dev->vbase + IMX_FLEXSPI_MCR0, reg);

    /* Configure MCR0, MCR1, and MCR2 */
    reg = in32(dev->vbase + IMX_FLEXSPI_MCR0);
    reg &= ~(IMX_FLEXSPI_MCR0_ARDFEN_MASK | IMX_FLEXSPI_MCR0_ATDFEN_MASK | IMX_FLEXSPI_MCR0_RXCLKSRC_MASK |
                 IMX_FLEXSPI_MCR0_COMBINATIONEN_MASK);
    if (dev->pads == 8) {
        reg |= IMX_FLEXSPI_MCR0_COMBINATIONEN_MASK;
    }
    reg |= (dev->smpl << IMX_FLEXSPI_MCR0_RXCLKSRC_SHIFT);
    out32(dev->vbase + IMX_FLEXSPI_MCR0, reg);

    /* Set max SEQ and AHB timeout */
    reg = 0xffffffff;
    out32(dev->vbase + IMX_FLEXSPI_MCR1, reg);

    reg = IMX_FLEXSPI_MCR2_RESUMEWAIT_MASK |
          IMX_FLEXSPI_MCR2_SCKBDIFFOPT_MASK |
          IMX_FLEXSPI_MCR2_CLRLEARNPHASE_MASK;
    out32(dev->vbase + IMX_FLEXSPI_MCR2, reg);

    /* Configure FLSHxCR0,FLSHxCR1,FLSHxCR2 */

    /* TODO:
     * During testing, read/write of the last 16 bytes of the specified size always fails.
     * Informing the SOC the device has additional space seems to work around the problem.
     * Need more investigation, or inquire NXP.
     */
    reg = in32(dev->vbase + IMX_FLEXSPI_FLSHA1CR0);
    reg = (reg & ~IMX_FLEXSPI_FLSHA1CR0_FLSHSZ_MASK) | ((dev->size >> 11) + 0x0f); /* In KBs: /1024 /2 */
    out32(dev->vbase + IMX_FLEXSPI_FLSHA1CR0, reg);

    /* See iMX8 RM section 18.2.4.6 - Otherwise, by default, offset will
       refer only to row accesses (i.e. in half-page increments; 16-byte increments). */
    reg = in32(dev->vbase + IMX_FLEXSPI_FLSHA1CR1);
    reg &= ~(IMX_FLEXSPI_FLSHA1CR1_CAS_MASK | IMX_FLEXSPI_FLSHA1CR1_WA_MASK);
    reg |= (3 << IMX_FLEXSPI_FLSHA1CR1_CAS_SHIFT) |
            IMX_FLEXSPI_FLSHA1CR2_CLRINSTRPTR_MASK /* | IMX_FLEXSPI_FLSHA1CR1_WA_MASK */;
    out32(dev->vbase + IMX_FLEXSPI_FLSHA1CR1, reg);

    /* Configure DLL control register DLLxCR */
    reg = (IMX_FLEXSPI_DLLACR_SLVDLYTARGET_MASK) | IMX_FLEXSPI_DLLACR_DLLEN_MASK;
    out32(dev->vbase + IMX_FLEXSPI_DLLACR, reg);

    reg = ((0xf << IMX_FLEXSPI_DLLBCR_SLVDLYTARGET_SHIFT) & IMX_FLEXSPI_DLLBCR_SLVDLYTARGET_MASK) |
            IMX_FLEXSPI_DLLACR_DLLEN_MASK;
    out32(dev->vbase + IMX_FLEXSPI_DLLBCR, reg);

    /* Exit module stop mode */
    reg = in32(dev->vbase + IMX_FLEXSPI_MCR0);
    reg &= ~IMX_FLEXSPI_MCR0_MDIS_MASK;
    out32(dev->vbase + IMX_FLEXSPI_MCR0, reg);

    /* Configure LUT */
    flexspi_s26hs_init_lut(dev);

    /* Reset controller */
    reg = in32(dev->vbase + IMX_FLEXSPI_MCR0);
    reg |= IMX_FLEXSPI_MCR0_SWRESET_MASK;
    out32(dev->vbase + IMX_FLEXSPI_MCR0, reg);

    /* Wait 1ms for domains reset */
    delay(1);

    /* Clear RX and TX FIFOs */
    out32(dev->vbase + IMX_FLEXSPI_IPTXFCR, IMX_FLEXSPI_IPTXFCR_CLRIPTXF_MASK);
    out32(dev->vbase + IMX_FLEXSPI_IPRXFCR, IMX_FLEXSPI_IPRXFCR_CLRIPRXF_MASK);
    /* Disable all interrupts */
    out32(dev->vbase + IMX_FLEXSPI_INTEN, 0x0);
    /* Clear all interrupts */
    out32(dev->vbase + IMX_FLEXSPI_INTR, 0xFFF);

    /* Interrupt handling initialization */
    if (dev->iid != -1) {
        InterruptDetach(dev->iid);
    }
    dev->iid = InterruptAttach(dev->irq, flexspi_s26hs_qspi_intr, dev, 0, _NTO_INTR_FLAGS_TRK_MSK);
    if (dev->iid == -1) {
        return -1;
    }
    return 0;
}

/**
 * Wait routine.
 *
 * @param dev Low level driver handle.
 *
 * @retval 0 on success, -1 on failure
 */
static int flexspi_s26hs_intr_wait(imx_fspi_t *dev)
{
    struct _pulse   pulse;
    uint64_t        ntime = 1e9;

    for (;;) {
        TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE, NULL, &ntime, NULL);
        if (MsgReceivePulse(dev->chid, &pulse, sizeof(pulse), NULL) == -1) {
            out32(dev->vbase + IMX_FLEXSPI_INTEN, 0x0); // Disable all interrupts
            dev->irq_requested = 0;
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "(devf t%d::%s:%d) timeout\n", pthread_self(), __func__, __LINE__);
            return -1;
        }
        if (pulse.code == IMX_FSPI_EVENT) {
            return 0;
        }
    }
    return -1;
}

/**
 * Sends command to connected device. General IP command. Waits for cmd finish.
 *
 * @param dev                Low level driver handle.
 * @param lut_index          Look-up table index.
 * @param data_size_override Optional parameter that will override default value of data size in LUT.
 *
 * @retval 0 on success, -1 on error
 */
static int flexspi_s26hs_send_ip_cmd(imx_fspi_t *dev, uint8_t lut_index, uint8_t num_seq, uint32_t data_size_override)
{
    uint32_t intr;

    /* Wait for idle */
    while (!(in32(dev->vbase + IMX_FLEXSPI_STS0) & 0x3));

    out32(dev->vbase + IMX_FLEXSPI_FLSHA1CR2, in32(dev->vbase + IMX_FLEXSPI_FLSHA1CR2) | IMX_FLEXSPI_FLSHA1CR2_CLRINSTRPTR_MASK);

    /* Clear all interrupts */
    out32(dev->vbase + IMX_FLEXSPI_INTR, 0xFFF);

    /* Flag interrupt handler that we will be waiting for an interrupt */
    dev->irq_requested = 1;

    /* Enable the required interrupts */
    out32(dev->vbase + IMX_FLEXSPI_INTEN,
            IMX_FLEXSPI_INTEN_IPCMDDONEEN_MASK | /* Cmd done */
            IMX_FLEXSPI_INTEN_IPCMDERREN_MASK  | /* Cmd error */
            IMX_FLEXSPI_INTEN_IPCMDGEEN_MASK);   /* Grant timeout */

    /* Set IP command sequence index */
    out32(dev->vbase + IMX_FLEXSPI_IPCR1, (lut_index << IMX_FLEXSPI_IPCR1_ISEQID_SHIFT) |
                                          ((num_seq-1) << IMX_FLEXSPI_IPCR1_ISEQNUM_SHIFT) |
                                          data_size_override);

    /* Trigger IP command */
    out32(dev->vbase + IMX_FLEXSPI_IPCMD, 0x1);

    if (flexspi_s26hs_intr_wait(dev) != 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "IP command failed!");
        return -1;
    }

    intr = in32(dev->vbase + IMX_FLEXSPI_INTR);
    out32(dev->vbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPCMDDONE_MASK);

    if (intr & 0xB0A) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "  ipcmd error: intr=0x%08x\n", intr);
        return -1;
    }

    return 0;
}

/**
 * Wait until program is done.
 *
 * @param dev Low level driver handle.
 *
 * @retval -1 If any error occur.
 * @retval 0 Everything is fine, -1 Error encountered
 */
static int flexspi_s26hs_wait_for_standby(imx_fspi_t *dev)
{
    uint8_t status[2];
    while (1) {
        status[0]=status[1]=0;
        flexspi_s26hs_read_status(dev, status);
        if (status[1] & 0x20 /* ERSERR */) {
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Erase error detected! (status=0x%02x%02x)\n", status[0], status[1]);
            return -1;
        }
        if (status[1] & 0x10 /* PRGERR */) {
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Write error detected! (status=0x%02x%02x)\n", status[0], status[1]);
            return -1;
        }
        if (status[1] & 0x80 /* RDYBSY */) {
            break;
        }
    }
    return 0;
}

/**
 * Writes data to FLEXSPI Tx FIFO.
 *
 * @param dev       Low level driver handle.
 * @param addr      Address of the write data buffer.
 * @param data_size Size of data to write.
 *
 * @retval EIO Data size exceed Tx FIFO size or no data to send.
 * @retval EOK Everything is fine.
 */
static int flexspi_s26hs_write_data(imx_fspi_t *dev, uint32_t offset, uint8_t *addr, uint32_t data_size)
{
    int ret;
    uint32_t watermark;
    uint32_t remaining = data_size;
    uint32_t run_offset = offset;

#ifdef DEBUG
    slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Write: offset=0x%08x, data_size=%d", offset, data_size);
#endif

    if ((offset & 0x01) || (data_size & 0x01)) {
        slogf(_SLOGC_FS_FFS, _SLOG_WARNING, "WARNING! Write offset=0x%08x or size=%d is not 2-byte aligned!", offset, data_size);
    }

    if (!data_size) {
        return -1;
    }

    /* If the start address is not aligned to halfpage(16-byte) boundary,
     * write data until the next halfpage boundary.
     */
    uint32_t extra = (offset & (FLEXSPI_MAX_BURST_TX-1));
    if (extra) {
        watermark = min(data_size, FLEXSPI_MAX_BURST_TX-extra);
    }
    else {
        watermark = min(data_size, FLEXSPI_MAX_BURST_TX);
    }

    /* Divide by 2, since addressable unit is 2 bytes */
    run_offset >>= 1;

    /* Disable DMA mode */
    out32(dev->vbase + IMX_FLEXSPI_IPTXFCR, in32(dev->vbase + IMX_FLEXSPI_IPTXFCR) & ~IMX_FLEXSPI_IPTXFCR_TXDMAEN_MASK);

    while (remaining > 0) {
        /* Start address: interpretation of IPCR0 depends on FLSHAxCR1[CAS] and [WA] */
        out32(dev->vbase + IMX_FLEXSPI_IPCR0, run_offset);

        /* Increment run_offset. Divide cur_size by 2, since addressable unit is 2 bytes */
        run_offset += (watermark >> 1);

        /* Clear Tx FIFO */
        out32(dev->vbase + IMX_FLEXSPI_IPTXFCR, in32(dev->vbase + IMX_FLEXSPI_IPTXFCR) | IMX_FLEXSPI_IPTXFCR_CLRIPTXF_MASK);

        /* Flag interrupt handler that we will be waiting for an interrupt */
        dev->irq_requested = 1;

        imx_flexspi_clear_fifo(dev, tx);

        /* Set interrupt empty space threshold */
        imx_flexspi_set_tx_watermark(dev, watermark);

        /* Enable TX watermark empty space available interrupt */
        out32(dev->vbase + IMX_FLEXSPI_INTEN, IMX_FLEXSPI_INTEN_IPTXWEEN_MASK);

        /* Wait for available space (watermark) */
        if (flexspi_s26hs_intr_wait(dev) != 0) {
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "No TX buffer! data_size=%u, offset=0x%08x, remaining=%u, run_offset=0x%08x\n",
                   data_size, offset, remaining, run_offset);
            goto fail;
        }

        /* Fill-up initial data */
        memcpy((void*)(dev->vbase + IMX_FLEXSPI_TFDR0), addr, watermark);
        addr += watermark;
        remaining -= watermark;

        /* Write */
        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_AUTOSELECT, 2, 0);
        if (ret >= 0) {
            ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_PROGRAM_WORD, 1, 0);
        }
        if (ret >= 0) {
            ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_WRITE, 1, watermark);
        }
        if (ret < 0) {
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Write command failed! data_size=%u, offset=0x%08x, remaining=%u, run_offset=0x%08x\n",
                   data_size, offset, remaining, run_offset);
            goto fail;
        }

        /* Clear IP and TXWE to PUSH DATA into device */
        out32(dev->vbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPTXWE_MASK);

        if (flexspi_s26hs_wait_for_standby(dev) < 0) {
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Write did not finish! data_size=%u, offset=0x%08x, remaining=%u, run_offset=0x%08x\n",
                   data_size, offset, remaining, run_offset);
            goto fail;
        }

        // Compute next watermark
        watermark = min(remaining, FLEXSPI_MAX_BURST_TX);
    }
    return data_size;

fail:
    slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Performing soft reset.");
    flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_SW_RESET, 1, 0);
    return -1;
}

/**
 * Reads data from FLEXSPI Rx FIFO.
 *
 * @param dev    Low level driver handle.
 * @param buffer Pointer where to copy Rx data.
 * @param size   Expected amount of data.
 *
 * @return Read data size. -1 if any error.
 */
static int flexspi_s26hs_read_data(imx_fspi_t *dev, uint32_t offset, uint8_t *buffer, uint32_t size)
{
    int ret;
    uint32_t cur_size;
    uint32_t watermark;
    uint32_t remaining = size;
    uint32_t run_offset = offset;

#ifdef DEBUG
    slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Read: offset=0x%08x, size=%d", offset, size);
#endif

    /* 1 read every fifo size, 1 copy every watermark */

    /* This is just for detection.
       Device also does this anyway. */
    if ((offset & 0x1) || (size & 0x1)) {
        slogf(_SLOGC_FS_FFS, _SLOG_WARNING, "WARNING! Read offset=0x%08x or size=%d is not 2-byte aligned!", offset, size);
    }

    /* Divide by 2, since addressable unit is 2 bytes */
    run_offset >>= 1;

    while (remaining > 0) {
        cur_size = min(remaining, FLEXSPI_RX_BUF_SIZE);

        /* Make sure cur_size is always divisible by FLEXSPI_MAX_BURST_RX,
         * except for the final read.
         * Code assumes FLEXSPI_MAX_BURST_RX is a power-of-2 value.
         */
        if (cur_size > FLEXSPI_MAX_BURST_RX) {
            cur_size -= (cur_size & (FLEXSPI_MAX_BURST_RX-1));
        }
        watermark = min(cur_size, FLEXSPI_MAX_BURST_RX);

        /* Read */

        /* Start address */
        out32(dev->vbase + IMX_FLEXSPI_IPCR0, run_offset);

        /* Increment run_offset. Divide cur_size by 2, since addressable unit is 2 bytes */
        run_offset += (cur_size >> 1);

        /* Clear Rx FIFO */
        out32(dev->vbase + IMX_FLEXSPI_IPRXFCR, in32(dev->vbase + IMX_FLEXSPI_IPRXFCR) | IMX_FLEXSPI_IPRXFCR_CLRIPRXF_MASK);

        /* Flag interrupt handler the amount of data we are waiting for */
        dev->rx_data_len = cur_size;
        dev->buf = buffer;
        dev->rx_watermark = watermark;

        imx_flexspi_clear_fifo(dev, tx);

        /* Set interrupt data threshold */
        imx_flexspi_set_rx_watermark(dev, watermark);

        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_READ, 1, cur_size);
        if (ret < 0) {
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Read command failed! offset=%u, size=%u, remaining=%u, cur_size=%u, run_offset=%u",
                  offset, size, remaining, cur_size, run_offset);
            goto fail;
        }

        if (flexspi_s26hs_intr_wait(dev) != 0) {
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Read did not finish! offset=%u, size=%u, remaining=%u, cur_size=%u, rx_data_len=%d, run_offset=%u",
                  offset, size, remaining, cur_size, dev->rx_data_len, run_offset);
            goto fail;
        }

        remaining -= cur_size;
        buffer += cur_size;
    }

    dev->rx_data_len = 0;
    dev->buf = 0;
    dev->rx_watermark = 0;

    return size;

fail:
    dev->rx_data_len = 0;
    dev->buf = 0;
    dev->rx_watermark = 0;

    slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Performing soft reset.");
    flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_SW_RESET, 1, 0);
    return -1;
}

/**
 * Called before device parameter investigation.
 *
 * @param dev Low level driver handle.
 *
 * @retval EOK Everything is OK.
 */
static int flexspi_s26hs_pre_ident_cfg(imx_fspi_t *dev)
{
    if (flexspi_s26hs_reopen(dev) != 0) {
        return -1;
    }
    return 0;
}

/**
 * Called after device parameter investigation.
 *
 * @param dev Low level driver handle
 *
 * @return Execution status.
 */
static int flexspi_s26hs_post_ident_cfg(imx_fspi_t *dev, uint8_t cfg_ecc)
{
    if (cfg_ecc) {
        if (flexspi_s26hs_reconfig(dev) != 0) {
            return -1;
        }
        flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_SW_RESET, 1, 0);
    }
    return 0;
}

/**
 * Read Device ID table and copy the mid, did, and device capacity.
 *
 * @param[in] dev          Low level driver handle.
 * @param[out] manufact_id Manufacturer identification.
 * @param[out] device_id   Device memory identification.
 * @param[out] size        Memory size.
 *
 * @return 0 on success, -1 of failure.
 */
static int flexspi_s26hs_parse_sfdp(imx_fspi_t *dev, int *manufact_id, int *device_id, uint32_t *size)
{
    int ret;
    uint8_t ident[DEVICE_ID_TABLE_SIZE] = {0};

    ret = flexspi_s26hs_read_data(dev, DEVICE_ID_TABLE_OFFSET, ident, DEVICE_ID_TABLE_SIZE);
    if (ret < 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Failed to read Device ID table.");
        return -1;
    }

    *manufact_id = (((uint32_t)ident[0]) << 8) | ident[1];
    *device_id = (((uint32_t)ident[8]) << 8) | ident[9];

    *size = 0;
    if (ident[4] == 0) {
        switch(ident[5]) {
            case 0x19:
                *size = (32 * 1024 * 1024); /* 256 Mbits */
                break;
            case 0x1A:
                *size = (64 * 1024 * 1024); /* 512 Mbits */
                break;
            case 0x1B:
                *size = (128 * 1024 * 1024); /* 1024 Mbits */
                break;
            default:
                break;
        }
    }
    if (*size == 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Unrecognized flash size 0x%02x %02x!", ident[4], ident[5]);
        return -1;
    }
    return 0;
}

/**
 * Read ID-CFI table and copy the mid, did, and device capacity.
 *
 * @param[in] dev          Low level driver handle.
 * @param[out] manufact_id Manufacturer identification.
 * @param[out] device_id   Device memory identification.
 * @param[out] size        Memory size.
 *
 * @return 0 on success, -1 of failure.
 */
static int flexspi_s26hs_parse_idcfi(imx_fspi_t *dev, int *manufact_id, int *device_id, uint32_t *size)
{
    int ret;
    uint8_t ident[DEVICE_ID_TABLE_SIZE] = {0};

    ret = flexspi_s26hs_read_data(dev, 0, ident, DEVICE_ID_TABLE_SIZE);
    if (ret < 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Failed to read ID-CFI table.");
        return -1;
    }

    *manufact_id = (((uint32_t)ident[0]) << 8) | ident[1];
    *device_id = (((uint32_t)ident[2]) << 8) | ident[3];

    *size = 0;
    if (ident[28] == 0) {
        switch(ident[29]) {
            case 0x6F: /* 3.0V */
            case 0x70: /* 1.8V */
                *size = (64 * 1024 * 1024); /* 512 Mbits */
                break;
            case 0x71: /* 3.0V */
            case 0x72: /* 1.8V */
                *size = (32 * 1024 * 1024); /* 256 Mbits */
                break;
            case 0x73: /* 3.0V */
            case 0x74: /* 1.8V */
                *size = (16 * 1024 * 1024); /* 128 Mbits */
                break;
            default:
                break;
        }
    }
    if (*size == 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Unrecognized flash size 0x%02x %02x!", ident[28], ident[29]);
        return -1;
    }
    return 0;
}

/**
 * Reads identification string of the memory device.
 *
 * @param[in] dev          Low level driver handle.
 * @param[out] manufact_id Manufacturer identification.
 * @param[out] device_id   Device memory identification.
 * @param[out] size        Memory size.
 *
 * @return 0 on success, -1 on failure.
 */
static int flexspi_s26hs_read_ident(imx_fspi_t *dev, int *manufact_id, int *device_id, uint32_t *size, uint8_t *cfg_ecc)
{
    uint8_t offset0[4] = {0};
    int ret;

    ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_SW_RESET, 1, 0);
    if (ret >= 0) {
        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_AUTOSELECT, 2, 0);
    }
    if (ret >= 0) {
        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_ENTER_SFDP_ASO, 1, 0);
    }
    if (ret >= 0) {
        ret = flexspi_s26hs_read_data(dev, 0, offset0, 4);
    }
    if (ret < 0) {
        return -1;
    }

    slogf(_SLOGC_FS_FFS, _SLOG_DEBUG2, "offset0=0x%02x %02x %02x %02x.\n", offset0[0], offset0[1], offset0[2], offset0[3]);

    if ( (offset0[1] == 'S') && (offset0[0] == 'F') && (offset0[3] == 'D') && (offset0[2] == 'P')) {
        slogf(_SLOGC_FS_FFS, _SLOG_INFO, "SFDP detected.");
        ret = flexspi_s26hs_parse_sfdp(dev, manufact_id, device_id, size);
        *cfg_ecc = 1;
    }
    else {
        slogf(_SLOGC_FS_FFS, _SLOG_INFO, "Standard ID-CFI detected.");
        ret = flexspi_s26hs_parse_idcfi(dev, manufact_id, device_id, size);
        *cfg_ecc = 0;
    }

    flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_SW_RESET, 1, 0);
    return (ret);
}

/**
 * Erase sector at given offset.
 *
 * @param dev     Low level driver handle.
 * @param offset  Memory offset.
 *
 * @retval EOK If OK.
 * @retval -1  If previous erase fail @see read_erase_status.
 */
static int flexspi_s26hs_sector_erase(imx_fspi_t *dev, int offset)
{
    int ret;

    slogf(_SLOGC_FS_FFS, _SLOG_INFO, "Erase: offset=0x%08x", offset);

    /* Divide by 2, since addressable unit is 2 bytes */
    offset >>= 1;

    out32(dev->vbase + IMX_FLEXSPI_IPCR0, offset); /* Write address to controller */

    ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_AUTOSELECT, 2, 0);
    if (ret >= 0) {
        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_PRE_ERASE, 1, 0);
    }
    if (ret >= 0) {
        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_AUTOSELECT, 2, 0);
    }
    if (ret >= 0) {
        ret = flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_SECTOR_ERASE, 1, 0);
    }
    if (ret < 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Erase: IP command failed.\n");
        return -1;
    }
    if (flexspi_s26hs_wait_for_standby(dev) < 0) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Erase: Performing soft reset of flash device.\n");
        flexspi_s26hs_send_ip_cmd(dev, IMX_FLEXSPI_LUT_SW_RESET, 1, 0);
        return -1;
    }
    return EOK;
}

/**
 * Program page at given offset.
 *
 * @param dev     Low level driver handle.
 * @param offset  Memory offset.
 * @param len     Size of data to write.
 * @param data    Pointer to data write buffer.
 *
 * @retval nbytes Size of written data.
 * @retval -1     If error.
 */
static int flexspi_s26hs_page_program(imx_fspi_t *dev, int offset, int len, uint8_t *data)
{
    return flexspi_s26hs_write_data(dev, offset, data, len);
}

/**
 * Read data from given offset.
 *
 * @param dev     Low level driver handle.
 * @param offset  Memory offset.
 * @param len     Size of data to read.
 * @param buffer  Pointer to data read buffer.
 *
 * @return Size of read data.
 */
static int flexspi_s26hs_read_from(imx_fspi_t *dev, int offset, int len, uint8_t *buffer)
{
    return flexspi_s26hs_read_data(dev, offset, buffer, len);
}



static const f3s_dbase_t s26hs_supported_devices[] = {
    {
        sizeof(f3s_dbase_t),   /* Size of complete structure with geometries */
        0,                     /* Status of structure */
        0x34,                  /* Jedec high byte - manufacturer ID */
        0x90,                  /* Jedec low byte - device ID */
        "Cypress S26HS Semper Flash with HyperBus Interface",
        0,                     /* Flags for capabilities */
        2,                     /* Interleave for chips on bus */
        2,                     /* Width of chip */
        120000U,               /* Typical write time for cell (ns) */
        200000000U,            /* Typical erase time for unit (ns) */
        0,                     /* Read mode voltage needed */
        0,                     /* Program mode voltage needed */
        0,                     /* Number of erase cycles */
        0,                     /* Poll count timeout */
        0,                     /* Depth of erase queue per chip */
        0,                     /* Number of write buffers per chip */
        16,                    /* Size of write buffers */
        1,                     /* Number of geometries in vector */
        {{256, 18}}            /* Number of erase units for geometry; power 2 size of a unit (256kB sector size) */
    },
    {
        sizeof(f3s_dbase_t),   /* Size of complete structure with geometries */
        0,                     /* Status of structure */
        0x01,                  /* Jedec high byte - manufacturer ID */
        0x7E,                  /* Jedec low byte - device ID */
        "Cypress S26KS HyperFlash Memory",
        0,                     /* Flags for capabilities */
        2,                     /* Interleave for chips on bus */
        2,                     /* Width of chip */
        120000U,               /* Typical write time for cell (ns) */
        200000000U,            /* Typical erase time for unit (ns) */
        0,                     /* Read mode voltage needed */
        0,                     /* Program mode voltage needed */
        0,                     /* Number of erase cycles */
        0,                     /* Poll count timeout */
        0,                     /* Depth of erase queue per chip */
        0,                     /* Number of write buffers per chip */
        16,                    /* Size of write buffers */
        1,                     /* Number of geometries in vector */
        {{256, 18}}            /* Number of erase units for geometry; power 2 size of a unit (256kB sector size) */
    },
    {0, 0xFFFF, 0, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
};

/**
 * This is the identification callout for FLEXSPI NOR flash driver.
 *
 * @param dbase  Flash Services Database.
 * @param access Access Super Structure.
 * @param flags  Flags.
 * @param offset Memory offset.
 *
 * @retval EOK    Device detection was successful.
 * @retval ENOENT Device detection fail.
 */
int f3s_flexspi_ident_s26hs(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags, 
                            __attribute__((unused)) uint32_t offset)
{
    static const f3s_dbase_t *probe = NULL;
    int                      mid = 0;  /* Manufacturer id */
    int                      did = 0;  /* Device id */
    uint32_t                 size = 0; /* Chip total size in bytes */
    uint8_t                  i;
    int                      k;
    unsigned                 unit_size;
    imx_fspi_t               *dev = (imx_fspi_t *) access->socket.socket_handle;
    uint8_t                  cfg_ecc = 0;

    /* Check listing flag */
    if (flags & F3S_LIST_ALL) {
        /* Check if first pass */
        if (!probe) {
            probe = &s26hs_supported_devices[0];
        }
        if (!probe->struct_size) {
            return (ENOENT);
        }
        *dbase = *probe++;  /* Update database entry */
        return (EOK);
    }

    /* Flash device spec calls for a 30 us delay after
     * release from power down. For simplicity, we just sleep 1 ms.
     */
    delay(1);

    if (flexspi_s26hs_pre_ident_cfg(dev)) {
        return ENOENT;
    }
    flexspi_s26hs_read_ident(dev, &mid, &did, &size, &cfg_ecc);

    slogf(_SLOGC_FS_FFS, _SLOG_INFO, "Ident: mid=0x%02x, did=0x%02x, size=%u", mid, did, size);

    if (size == 0) { /* The device ident does return the size */
        size = dev->size * dev->die;
    }

    for (i = 0; i < ARRAY_SIZE(s26hs_supported_devices); i++) {
        const f3s_dbase_t *sd = &s26hs_supported_devices[i];

        if  (((mid == sd->jedec_hi) && (did == sd->jedec_lo)) &&
             (!sd->chip_inter || (sd->chip_inter == flashcfg.chip_inter)) &&
             (!sd->chip_width || (sd->chip_width == flashcfg.bus_width))) {
            for (k = 0; k < sd->geo_num; k++) {
                const struct f3s_geo_s *geo_vect = &sd->geo_vect[k];
                unit_size = 1 << geo_vect->unit_pow2;
                /* since size is a command line option it must be checked here*/

                if( (size == geo_vect->unit_num * unit_size)
                    ||
                    (size == dev->size ) ) {
                    *dbase = *sd;  /* Update database entry */
                    /* ensure the number of erase units is set right*/
                    dbase->geo_vect[0].unit_num = size / unit_size;
                    dbase->chip_size = size;
                    access->socket.unit_size = unit_size;
                    if (flexspi_s26hs_post_ident_cfg(dev, cfg_ecc)) {
                        return ENOENT;
                    }
                    slogf(_SLOGC_FS_FFS, _SLOG_INFO, "(devf  t%d::%s:%d) NOR flash detected: mid=0x%X did=0x%X size=%d",
                          pthread_self(), __func__, __LINE__, mid, did, size);
                    return (EOK);
                }
            }
        }
    }

    return (ENOENT);
}

/**
 * This is the read callout for FLEXSPI NOR flash driver.
 *
 * @param dbase       Flash Services Database.
 * @param access      Access Super Structure
 * @param flags       Flags.
 * @param text_offset Offset of memory to read data.
 * @param buffer_size Buffer size.
 * @param buffer      Pointer to buffer.
 *
 * @return Returns number of bytes read or -1 in case of an error.
 */
int32_t f3s_flexspi_read_s26hs(f3s_dbase_t *dbase, f3s_access_t *access,  __attribute__((unused)) uint32_t flags,
                               uint32_t text_offset, int32_t buffer_size, uint8_t *buffer)
{
    int rc;

#ifdef DEBUG
    slogf(_SLOGC_FS_FFS, _SLOG_DEBUG2, "(devf  t%d::%s:%d) offset=0x%x", pthread_self(), __func__, __LINE__, text_offset);
#endif

    /* Check if offset does not fit in array */
    if (text_offset >= access->socket.window_size) {
        errno = ERANGE;
        return -1;
    }
    /* Ensure that offset + size is not out of bounds */
    buffer_size = min(buffer_size, access->socket.window_size - text_offset);
    rc = flexspi_s26hs_read_from((imx_fspi_t *) access->socket.socket_handle, text_offset, buffer_size, buffer);
    if (-1 == rc) {
        errno = EIO;
        return -1;
    }

    return rc; /* Return number of bytes read */
}

/**
 * This is the write callout for FLEXSPI NOR flash driver.
 *
 * @param dbase  Flash Services Database.
 * @param access Access Super Structure.
 * @param flags  Flags.
 * @param offset Offset where to write data.
 * @param size   Size of data.
 * @param buffer Buffer to write to memory.
 *
 * @return Size of written data if everything is fine. EIO otherwise.
 */
int32_t f3s_flexspi_write_s26hs(f3s_dbase_t *dbase, f3s_access_t *access,  __attribute__((unused)) uint32_t flags,
                                uint32_t offset, int32_t size, uint8_t *buffer)
{
    int     rc,i;
    int     nbytes;
    int32_t xfer;
    imx_fspi_t *dev = (imx_fspi_t *) access->socket.socket_handle;
    uint8_t *verify=dev->verify; /* Buffer for verification */

#ifdef DEBUG
    slogf(_SLOGC_FS_FFS, _SLOG_DEBUG2, "(devf  t%d::%s:%d) offset=0x%x size=%d", pthread_self(), __func__, __LINE__,
          offset, size);
#endif
    if (access->service->page(&access->socket, 0, offset, &size) == NULL) {
        return -1;
    }
    rc = flexspi_s26hs_page_program(dev, offset, size, buffer);
    if (-1 == rc) {
        errno = EIO;
        return -1;
    }
    if (!(flags & F3S_VERIFY_WRITE)) {
        return rc;
    }
    /* Verify data was written correctly */
    size = rc;
    xfer = rc;

    while (xfer) {
        nbytes = min(dev->page_size, xfer);

        rc = flexspi_s26hs_read_from(dev, offset, nbytes, verify);
        if (-1 == rc) {
            errno = EIO;
            return -1;
        }

        if (memcmp(verify, buffer, rc)) {
            for (i = 0; i < rc; i++) {
                slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "EXPECTED %d val: 0x%x", i, buffer[i]);
                slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "READ     %d val: 0x%x", i, verify[i]);
            }
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "(devf  t%d::%s:%d) program verify error\n"
                  "between offset 0x%x and 0x%x, size = %d", pthread_self(),
                  __func__, __LINE__, offset, offset + rc, rc);
            errno = EIO;
            return -1;
        }
        xfer -= rc;
        offset += rc;
        buffer += rc;
    }
    /* Verification successful */
    return size;
}

/**
 * This is the sync callout for FLEXSPI NOR flash driver. Called together with erase
 * function to check erase progress.
 *
 * @param dbase       Flash Services Database.
 * @param access      Access Super Structure.
 * @param flags       Flags.
 * @param text_offset Memory offset.
 *
 * @retval EOK    Everything is fine. Memory is ready.
 * @retval ERANGE Offset out of bounds.
 * @retval EIO    This return code currently not supported.
 * @retval EAGAIN Erase still in progress.
 */
int32_t f3s_flexspi_sync_s26hs(f3s_dbase_t *dbase, f3s_access_t *access,  __attribute__((unused)) uint32_t flags, uint32_t text_offset)
{
    uint8_t status[2];
    imx_fspi_t *dev = (imx_fspi_t *) access->socket.socket_handle;
    flexspi_s26hs_read_status(dev, status);
    if (status[1] & 0x20 /* ERSERR */) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Sync: Erase error detected!\n");
        return EIO;
    }
    if (status[1] & 0x10 /* PRGERR */) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "Sync: Write error detected!\n");
        return EIO;
    }
    if (status[1] & 0x80 /* RDYBSY */) {
        return EOK;
    }
    return EAGAIN;
}

/**
 * This is the erase callout for FLEXSPI NOR flash driver.
 *
 * @param dbase  Flash Services Database.
 * @param access Access Super Structure.
 * @param flags  Flags.
 * @param offset Memory offset.
 *
 * @retval EOK    Everything is OK.
 * @retval ERANGE The offset is out of bounds.
 * @retval EIO    Previous erase fail.
 */
int f3s_flexspi_erase_s26hs(f3s_dbase_t *dbase, f3s_access_t *access,  __attribute__((unused)) uint32_t flags, uint32_t offset)
{
    int rc;

#ifdef DEBUG
    slogf(_SLOGC_FS_FFS, _SLOG_DEBUG2, "(devf  t%d::%s:%d) offset=0x%x", pthread_self(), __func__, __LINE__, offset);
#endif
    if (access->service->page(&access->socket, 0, offset, NULL) == NULL) {
        return (ERANGE);
    }
    rc = flexspi_s26hs_sector_erase((imx_fspi_t *) access->socket.socket_handle, offset);
    if (rc < 0) {
        return (EIO);
    }

    return (EOK);
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_s26hs.c $ $Rev: 910001 $")
#endif
