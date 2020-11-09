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

#ifndef IMX_FC_FLEXSPI_H_
#define IMX_FC_FLEXSPI_H_

#include <aarch64/inout.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/f3s_mtd.h>

#include <aarch64/imx8_common/imx_flexspi.h>

/**
 * @file       flexspi-imx8/imx_fc_flexspi.h
 * @addtogroup ffs3_fc Flash Controller
 * @{
 */

/* IP Block IO space size */
#ifndef IMX_FLEXSPI_SIZE
#define IMX_FLEXSPI_SIZE                    0x10000
#endif

/* Driver Default values */
#define IMX_FLEXSPI_DEFAULT_BASE_ADDR           0x5D120000
#define IMX_FLEXSPI_DEFAULT_IRQ                 124

/** LUT entry */
typedef union {
    uint32_t U;
    struct {
        unsigned OPRND0: 8;
        unsigned PAD0: 2;
        unsigned INSTR0: 6;
        unsigned OPRND1: 8;
        unsigned PAD1: 2;
        unsigned INSTR1: 6;
    } B;
} imx_fspi_lut_t;

#define IMX_FSPI_MAX_RX_FIFO_WINDOW         (IMX_FLEXSPI_RFDRa_NUM * 4)    /**< RX/TX maximum FIFO window (bytes) */
#define IMX_FSPI_MAX_TX_FIFO_WINDOW         (IMX_FLEXSPI_TFDRa_NUM * 4)    /**< RX/TX maximum FIFO window (bytes) */

/** @name IP RX FIFO parameters */
#define IMX_FSPI_RX_FIFO_WIDTH              8           /**< RX FIFO width (bytes) */
#define IMX_FSPI_RX_FIFO_DEPTH              64          /**< RX FIFO depth (entries) */
#define IMX_FSPI_RX_FIFO_SIZE               (IMX_FSPI_RX_FIFO_DEPTH * IMX_FSPI_RX_FIFO_WIDTH) /**< RX FIFO size */

/** @name IP TX FIFO parameters */
#define IMX_FSPI_TX_FIFO_WIDTH              8           /**< TX FIFO width (bytes) */
#define IMX_FSPI_TX_FIFO_DEPTH              128         /**< TX FIFO depth (entries) */
#define IMX_FSPI_TX_FIFO_SIZE               (IMX_FSPI_TX_FIFO_DEPTH * IMX_FSPI_TX_FIFO_WIDTH) /**< TX FIFO size */

/** @} */

/** @name LUT commands */
#define IMX_FSPI_PAD_1                      0x00        /**< Communication on 1 pad  (Single mode) */
#define IMX_FSPI_PAD_2                      0x01        /**< Communication on 2 pads (Dual mode) */
#define IMX_FSPI_PAD_4                      0x02        /**< Communication on 4 pads (Quad mode) */
#define IMX_FSPI_PAD_8                      0x03        /**< Communication on 8 pads (Octal mode) */

#define IMX_FSPI_INSTR_CMD                  0x01        /**< FC command */
#define IMX_FSPI_INSTR_CMD_DDR              0x21        /**< FC command DDR */
#define IMX_FSPI_INSTR_ADDR                 0x02        /**< FC address */
#define IMX_FSPI_INSTR_ADDR_DDR             0x22        /**< FC address DDR */
#define IMX_FSPI_INSTR_DUMMY                0x0C        /**< FC dummy operation */
#define IMX_FSPI_INSTR_DUMMY_DDR            0x2C        /**< FC dummy operation DDR */
#define IMX_FSPI_INSTR_READ                 0x09        /**< FC read */
#define IMX_FSPI_INSTR_READ_DDR             0x29        /**< FC read DDR */
#define IMX_FSPI_INSTR_WRITE                0x08        /**< FC write */
#define IMX_FSPI_INSTR_WRITE_DDR            0x28        /**< FC write DDR */
#define IMX_FSPI_INSTR_JMP_ON_CS            0x1F        /**< FC jump on cs */
#define IMX_FSPI_INSTR_STOP                 0x00        /**< FC stop*/

/** @name LUT device command indexes */
#define NOT_USED                            0           /**< Enter Quad IO mode */
#define IMX_FSPI_LUT_CMD_IDX_RDIR           1           /**< Read identification */
#define IMX_FSPI_LUT_CMD_IDX_WREN           2           /**< Write enable */
#define IMX_FSPI_LUT_CMD_IDX_SE             3           /**< Sector erase */
#define IMX_FSPI_LUT_CMD_IDX_RDSR           4           /**< Read status register */
#define IMX_FSPI_LUT_CMD_IDX_RFLAGR         5           /**< Read flag register */
#define IMX_FSPI_LUT_CMD_IDX_MAIN_DRV_READ  6           /**< Main read command */
#define IMX_FSPI_LUT_CMD_IDX_MAIN_DRV_WRITE 7           /**< Main write command */
#define IMX_FSPI_LUT_CMD_IDX_WRSR           8           /**< Write status register (Normal mode) */
#define IMX_FSPI_LUT_CMD_IDX_EN4B           9           /**< Enter 4-byte addressing mode */
#define IMX_FSPI_LUT_CMD_IDX_EXIT4B         10          /**< Exit 4-byte addressing mode */

#define IMX_FSPI_LUT_SIZE                   11          /**< Number of commands in LUT */
/** @} */
/** @} */

#define ARRAY_SIZE(array) (sizeof(array)/sizeof((array)[0]))

typedef enum _imx_fspi_speed_t {
    full  = 0,
    half = 2,
} imx_fspi_speed_t;

typedef enum _imx_fspi_bufclr_t {
    tx = 1,
    rx = 2,
    both = 3,
} imx_fspi_bufclr_t;

#define IMX_FSPI_EVENT 1
#define IMX_FSPI_PRIORITY 21

/** Low level driver handle */
typedef struct _imx_fspi_t {
    unsigned              pbase;
    uintptr_t             vbase;
    unsigned              irq;
    unsigned              page_size;        /**< Page size in Bytes */
    unsigned              size;             /**< Device capacity in Bytes */
    unsigned              die;              /**< Number of die */
    unsigned              dummy;            /**< Dummy cycles for read command(s) */
    unsigned              pads;             /**< Device mode Octal/Quad */
    unsigned              smpl;             /**< Rx sampling mode */
    int                   iid;
    int                   coid;
    int                   chid;
    struct sigevent       fspievent;
    uint32_t              mema1;          /**< Start address of device connected to A1 */
    uint32_t              memb1;          /**< Start address of device connected to B1 */
    uint32_t              mema2;          /**< Start address of device connected to A2 */
    uint32_t              memb2;          /**< Start address of device connected to B1 */
    int                   irq_requested;
    /* Variables for RX FIFO handling in interrupt */
    uint32_t              rx_data_len;
    uint32_t              rx_watermark;
    uint8_t               *buf;
    int                   fd;
    uint8_t               *verify;
} imx_fspi_t;

int flexspi_intr_wait(imx_fspi_t *dev);

int imx_flexspi_setcfg(imx_fspi_t *dev);

imx_fspi_t *imx_flexspi_open(void);

int imx_flexspi_close(imx_fspi_t *dev);

int imx_flexspi_write_data(imx_fspi_t *dev, uint8_t *addr, uint32_t data_size);

int imx_flexspi_read_data(imx_fspi_t *dev, uint8_t *buffer, uint32_t size);

int imx_flexspi_clear_fifo(imx_fspi_t *dev, imx_fspi_bufclr_t mask);

void imx_flexspi_set_rx_watermark(imx_fspi_t *dev, uint32_t data_transfer_size);

void imx_flexspi_set_tx_watermark(imx_fspi_t *dev, uint32_t data_transfer_size);

int imx_flexspi_send_ip_cmd(imx_fspi_t *dev, uint8_t lut_index, uint32_t data_size_override);

int imx_flexspi_unlock_lut(imx_fspi_t *dev);

int imx_flexspi_lock_lut(imx_fspi_t *dev);

imx_fspi_lut_t imx_flexspi_create_lut_record(uint8_t instr0, uint8_t pad0, uint8_t opr0,
                                             uint8_t instr1, uint8_t pad1, uint8_t opr1);

int imx_flexspi_write_lut(imx_fspi_t* dev, uint8_t index, imx_fspi_lut_t *lutcmd0, imx_fspi_lut_t *lutcmd1,
                          imx_fspi_lut_t *lutcmd2, imx_fspi_lut_t *lutcmd3);

void imx_flexspi_set_speed(imx_fspi_t *dev, imx_fspi_speed_t speed);

/** @} */

#endif /* IMX_FC_FLEXSPI_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/imx_fc_flexspi.h $ $Rev: 885792 $")
#endif
