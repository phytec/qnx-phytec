/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
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

#include <unistd.h>
#include <assert.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <time.h>

#include "imx_fc_flexspi.h"
#include "flexspi_cmds.h"

/**
 * @file       flexspi-imx8/flexspi_cmds.c
 * @addtogroup ffs3_cmds Commands
 * @{
 */

/**
 * Check WIP bit of the status register of memory and returns 0/1.
 *
 * @param[in] dev Low level driver handle.
 *
 * @retval 1  Writing in progress.
 * @retval 0  No write in progress.
 * @retval -1 If error.
 */
int iswriting(imx_fspi_t *dev)
{
    uint32_t stat_reg = 0;

    imx_flexspi_clear_fifo(dev, rx); /* Clear Rx FIFO */
    imx_flexspi_set_rx_watermark(dev, sizeof(stat_reg));
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_RDSR, sizeof(stat_reg));
    if (imx_flexspi_read_data(dev, (uint8_t *)&stat_reg, sizeof(stat_reg)) == -1) {
        /* Interrupt handling problem occur */
        return -1;
    }

    return (stat_reg & DEVICE_SR_WIP);
}

/**
 * Wait until program is done.
 *
 * @param dev Low level driver handle.
 *
 * @retval -1 If any error occur.
 * @retval EOK Everything is fine.
 */
int wait_for_completion(imx_fspi_t *dev)
{
    int count = FLASH_PROGRAM_WAIT_TIME;
    int rc = 0;
    do {
        rc = iswriting(dev);
        if (-1 == rc) {
            return rc;
        }
        if (rc & DEVICE_SR_WIP) {
            nanospin_ns(1);
        }
    } while (rc && --count);
    if (count == 0) {
        return -1;
    }
    return EOK;
}

/**
 * Read status of the memory erase operation.
 *
 * @param[in] dev Low level driver.
 *
 * @retval EOK If erase succeed.
 * @retval EIO If erase failed.
 */
int read_erase_status(imx_fspi_t *dev)
{
    uint32_t flag_reg = 0;

    imx_flexspi_clear_fifo(dev, both);                                           /* Clear Rx Tx FIFOs */
    imx_flexspi_set_rx_watermark(dev, sizeof(flag_reg));                         /* Set watermark*/
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_RFLAGR, sizeof(flag_reg)); /* Send command */
    imx_flexspi_read_data(dev, (uint8_t*)&flag_reg, sizeof(flag_reg));           /* Read data */

    return ((flag_reg & DEVICE_FSR_ERASE_ERR) ? EIO : EOK);
}

/**
 * Read status register of the memory device.
 *
 * @param[in]  dev      Low level driver handle.
 * @param[out] stat_reg Memory status register.
 *
 * @return EOK always.
 */
int read_status(imx_fspi_t *dev, uint8_t *stat_reg)
{
    imx_flexspi_clear_fifo(dev, both);                                          /* Clear Rx Tx FIFOs */
    imx_flexspi_set_rx_watermark(dev, sizeof(*stat_reg));                       /* Set watermark */
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_RDSR, sizeof(*stat_reg)); /* Send command */
    imx_flexspi_read_data(dev, stat_reg, sizeof(*stat_reg));                    /* Read data */

    return (EOK);
}

/**
 * Called before device parameter investigation.
 *
 * @param dev Low level driver handle.
 *
 * @retval EOK Everything is OK.
 */
int pre_ident_cfg(imx_fspi_t *dev)
{
    return EOK;
}

/**
 * Called after device parameter investigation.
 *
 * @param dev Low level driver handle
 *
 * @return Execution status.
 */
int post_ident_cfg(imx_fspi_t *dev)
{
    return EOK;
}

/**
 * Writes status register of the memory.
 *
 * @param[in] dev      Low level driver handle.
 * @param[in] stat_reg Status register value.
 *
 * @return EOK always.
 */
int write_status(imx_fspi_t *dev, uint8_t *stat_reg)
{
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_WREN, 0);                 /* Write enable */
    imx_flexspi_clear_fifo(dev, both);                                          /* Clear Rx Tx FIFOs */
    imx_flexspi_write_data(dev, stat_reg, sizeof(*stat_reg));                   /* Write data */
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_WRSR, sizeof(*stat_reg)); /* Send command */

    return (EOK);
}

/**
 * Reads identification string of the memory device.
 *
 * @param[in] dev          Low level driver handle.
 * @param[out] manufact_id Manufacturer identification.
 * @param[out] device_id   Device memory identification.
 * @param[out] size        Memory size.
 *
 * @return EOK always.
 */
int read_ident(imx_fspi_t *dev, int *manufact_id, int* device_id, uint32_t *size)
{
    uint8_t ident[4] = {0};

    imx_flexspi_clear_fifo(dev, both);                                      /* Clear Rx Tx FIFOs */
    imx_flexspi_set_rx_watermark(dev, sizeof(ident));                       /* Set Rx watermark */
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_RDIR, sizeof(ident)); /* Send command */
    imx_flexspi_read_data(dev, ident, sizeof(ident));                       /* Read data */

    *manufact_id = ident[0];
    *device_id = ident[1];
    if (ident[2] == 0) {
        *size = 0;
    } else {
        *size = (1 << ident[2]) * dev->die;
    }

    return (EOK);
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
int sector_erase(imx_fspi_t *dev, const int offset)
{
    int status;

    out32(dev->vbase + IMX_FLEXSPI_IPCR0, offset); /* Write address to controller */
    /* Check erase status */
    status = read_erase_status(dev);
    if (status != EOK) {
        return -1;
    }
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_WREN, 0); /* Write enable */
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_SE, 0);   /* Sector erase */

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
int page_program(imx_fspi_t *dev, int offset, int len, uint8_t *data)
{
    int nbytes = len;
    int tx_watermark;
    int tx_data_len;

    /* If writing all nbytes crosses a page boundary, then we reduce nbytes so that we write to the
     * end of the current page, but not beyond. */
    nbytes = min(nbytes, (offset & ~(dev->page_size - 1)) + dev->page_size - offset);

    dev->buf = data;
    tx_watermark = min(nbytes, IMX_FSPI_MAX_TX_FIFO_WINDOW);
    tx_data_len = nbytes;

    out32(dev->vbase + IMX_FLEXSPI_IPCR0, offset);                              /* Write address to controller */
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_WREN, 0);                 /* Write enable */
    imx_flexspi_clear_fifo(dev, tx);                                            /* Clear Tx FIFO */
    imx_flexspi_set_tx_watermark(dev, tx_watermark);                            /* Set water-mark */
    while (tx_data_len > 0) {
        /* Write data */
        memcpy((void*)(dev->vbase + IMX_FLEXSPI_TFDR0), dev->buf, tx_watermark);
        /* Push data to Tx FIFO */
        out32(dev->vbase + IMX_FLEXSPI_INTR, in32(dev->vbase + IMX_FLEXSPI_INTR) | IMX_FLEXSPI_INTR_IPTXWE_MASK);
        /* Update variables with new values */
        dev->buf = dev->buf + tx_watermark;
        tx_data_len = tx_data_len - tx_watermark;
        tx_watermark = min(tx_data_len, IMX_FSPI_MAX_TX_FIFO_WINDOW);
        imx_flexspi_set_tx_watermark(dev, tx_watermark);
    }
    imx_flexspi_send_ip_cmd(dev, IMX_FSPI_LUT_CMD_IDX_MAIN_DRV_WRITE, nbytes);  /* Send Command */
    if (wait_for_completion(dev)) {                                             /* Wait for interrupt */
        return -1;
    }

    return nbytes;
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
int read_from(imx_fspi_t *dev, int offset, int len, uint8_t *buffer)
{
    if (len == 0) {
        return 0;
    }
    len = min(len, IMX_FSPI_RX_FIFO_SIZE);                                      /* Reduce transfer size */
    dev->buf = buffer;                                                          /* Buffer pointer for interrupt */
    dev->rx_data_len = len;                                                     /* Data length for interrupt */
    dev->rx_watermark = min(len, IMX_FSPI_MAX_RX_FIFO_WINDOW);                  /* Set Rx watermark for interrupt */

    out32(dev->vbase + IMX_FLEXSPI_IPCR0, offset);                              /* Write address to controller */
    imx_flexspi_clear_fifo(dev, rx);                                            /* Clear Rx FIFO */
    imx_flexspi_set_rx_watermark(dev, dev->rx_watermark);                       /* Set Rx watermark */
    out32(dev->vbase + IMX_FLEXSPI_INTEN, IMX_FLEXSPI_INTEN_IPRXWAEN_MASK);     /* Enable Rx interrupt */
    /* Set IP command sequence index */
    out32(dev->vbase + IMX_FLEXSPI_IPCR1, (IMX_FSPI_LUT_CMD_IDX_MAIN_DRV_READ << IMX_FLEXSPI_IPCR1_ISEQID_SHIFT) |
                                          dev->rx_data_len);
    /* Trigger IP command */
    out32(dev->vbase + IMX_FLEXSPI_IPCMD, 0x1);
    /* Wait for interrupt */
    if (flexspi_intr_wait(dev)) {
        return -1;
    }

    return len;
}

/**
 * Command look-up table initialization.
 *
 * @param dev Low level driver handle.
 */
int init_lut(imx_fspi_t *dev)
{
    imx_fspi_lut_t lutRDIR;                         /* RDIR - Read identification */
    imx_fspi_lut_t lutSE;                           /* SE - sector erase */
    imx_fspi_lut_t lutMAIN_READ1, lutMAIN_READ2;    /* MAIN READ - see MAIN_DRV_READ define for details  */
    imx_fspi_lut_t lutMAIN_WRITE1, lutMAIN_WRITE2;  /* MAIN WRITE - see MAIN_DRV_WRITE define for details */
    imx_fspi_lut_t lutEN4B;                         /* Enable 4 byte addressing mode (4B) */
    imx_fspi_lut_t lutEXIT4B;                       /* Exit 4 byte addressing mode (4B) */
    uint8_t main_read = 0, main_write = 0, pads = 0;

    if (dev->pads == 8) {
        main_read = FLASH_OPCODE_8READ4B;
        main_write = FLASH_OPCODE_8PP_EXT_4B;
        pads = IMX_FSPI_PAD_8;
    } else if (dev->pads == 4) {
        main_read = FLASH_OPCODE_4READ4B;
        main_write = FLASH_OPCODE_4PP_EXT_4B;
        pads = IMX_FSPI_PAD_4;
    } else {
        return ENOTSUP;
    }
    /* READ ID */
    lutRDIR = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD, IMX_FSPI_PAD_1, FLASH_OPCODE_RDID,
                                            IMX_FSPI_INSTR_READ, IMX_FSPI_PAD_1, 4);
    /* SECTOR ERASE */
    lutSE = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD, IMX_FSPI_PAD_1, FLASH_OPCODE_SE4B,
                                          IMX_FSPI_INSTR_ADDR, IMX_FSPI_PAD_1, ADDR_MODE_4BYTE);
    /* ENABLE 4 BYTE ADDRESSING MODE */
    lutEN4B = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD, IMX_FSPI_PAD_1, FLASH_OPCODE_EN4B,
                                            0, 0, 0);
    /* EXIT 4 BYTE ADDRESSING MODE */
    lutEXIT4B = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD, IMX_FSPI_PAD_1, FLASH_OPCODE_EXIT4B,
                                              0, 0, 0);
    /* READ */
    lutMAIN_READ1 = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD, IMX_FSPI_PAD_1, main_read,
                                                  IMX_FSPI_INSTR_ADDR, pads, ADDR_MODE_4BYTE);
    lutMAIN_READ2 = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_DUMMY, pads, dev->dummy, /* x2 for DDR */
                                                  IMX_FSPI_INSTR_READ, pads, 4);
    /* WRITE */
    lutMAIN_WRITE1 = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_CMD, IMX_FSPI_PAD_1, main_write,
                                                   IMX_FSPI_INSTR_ADDR, pads, ADDR_MODE_4BYTE);
    lutMAIN_WRITE2 = imx_flexspi_create_lut_record(IMX_FSPI_INSTR_WRITE, pads, 4,
                                                   0, 0, 0);

    /* Setup LUT begin */
    imx_flexspi_unlock_lut(dev);
    /* ##Seq0## : NOT USED */
    out32(dev->vbase + IMX_FLEXSPI_LUTa(0), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(1), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(2), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(3), 0);
    /* ##Seq1## : Read id */
    imx_flexspi_write_lut(dev, IMX_FSPI_LUT_CMD_IDX_RDIR, &lutRDIR, NULL, NULL, NULL);
    /* ##Seq2## : Write enable */
    out32(dev->vbase + IMX_FLEXSPI_LUTa(8), (IMX_FSPI_INSTR_CMD << IMX_FLEXSPI_LUT_OPCODE0_SHIFT) |
                                            (IMX_FSPI_PAD_1 << IMX_FLEXSPI_LUT_NUM_PADS0_SHIFT) |
                                            FLASH_OPCODE_WREN);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(9), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(10), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(11), 0);
    /* ##Seq3## : Sector erase 4 byte addressing mode (4B) */
    imx_flexspi_write_lut(dev, IMX_FSPI_LUT_CMD_IDX_SE, &lutSE, NULL, NULL, NULL);
    /* ##Seq4## : Read status */
    out32(dev->vbase + IMX_FLEXSPI_LUTa(16), (IMX_FSPI_INSTR_READ << IMX_FLEXSPI_LUT_OPCODE1_SHIFT) |
                                             (IMX_FSPI_PAD_1 << IMX_FLEXSPI_LUT_NUM_PADS1_SHIFT) |
                                             (1 << IMX_FLEXSPI_LUT_OPERAND1_SHIFT) |
                                             (IMX_FSPI_INSTR_CMD << IMX_FLEXSPI_LUT_OPCODE0_SHIFT) |
                                             (IMX_FSPI_PAD_1 << IMX_FLEXSPI_LUT_NUM_PADS0_SHIFT) |
                                             FLASH_OPCODE_RDSR);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(17), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(18), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(19), 0);
    /* ##Seq5## : Read flag status register */
    out32(dev->vbase + IMX_FLEXSPI_LUTa(20), (IMX_FSPI_INSTR_READ << IMX_FLEXSPI_LUT_OPCODE1_SHIFT) |
                                             (IMX_FSPI_PAD_1 << IMX_FLEXSPI_LUT_NUM_PADS1_SHIFT) |
                                             (1 << IMX_FLEXSPI_LUT_OPERAND1_SHIFT) |
                                             (IMX_FSPI_INSTR_CMD << IMX_FLEXSPI_LUT_OPCODE0_SHIFT) |
                                             (IMX_FSPI_PAD_1 << IMX_FLEXSPI_LUT_NUM_PADS0_SHIFT) | FLASH_OPCODE_RFLAGR);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(21), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(22), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(23), 0);
    /* ##Seq6## : MAIN DRV READ */
    imx_flexspi_write_lut(dev, IMX_FSPI_LUT_CMD_IDX_MAIN_DRV_READ, &lutMAIN_READ1, &lutMAIN_READ2, NULL, NULL);
    /* ##Seq7## : MAIN DRV WRITE */
    imx_flexspi_write_lut(dev, IMX_FSPI_LUT_CMD_IDX_MAIN_DRV_WRITE, &lutMAIN_WRITE1, &lutMAIN_WRITE2, NULL, NULL);
    /* ##Seq8## : Write Status (Normal mode) */
    out32(dev->vbase + IMX_FLEXSPI_LUTa(32), (IMX_FSPI_INSTR_WRITE << IMX_FLEXSPI_LUT_OPCODE1_SHIFT) |
                                             (IMX_FSPI_PAD_1 << IMX_FLEXSPI_LUT_NUM_PADS1_SHIFT) |
                                             (2 << IMX_FLEXSPI_LUT_OPERAND1_SHIFT) |
                                             (IMX_FSPI_INSTR_CMD << IMX_FLEXSPI_LUT_OPCODE0_SHIFT) |
                                             (IMX_FSPI_PAD_1 << IMX_FLEXSPI_LUT_NUM_PADS0_SHIFT) |
                                             FLASH_OPCODE_WRSR);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(33), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(34), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(35), 0);
    /* ##Seq9## : Enable 4 byte addressing mode (4B) */
    imx_flexspi_write_lut(dev, IMX_FSPI_LUT_CMD_IDX_EN4B, &lutEN4B, NULL, NULL, NULL);
    /* ##Seq10## : Exit 4 byte addressing mode (4B) */
    imx_flexspi_write_lut(dev, IMX_FSPI_LUT_CMD_IDX_EXIT4B, &lutEXIT4B, NULL, NULL, NULL);

    /* Setup LUT end */
    imx_flexspi_lock_lut(dev);

    return EOK;
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/flexspi_cmds.c $ $Rev: 889622 $")
#endif
