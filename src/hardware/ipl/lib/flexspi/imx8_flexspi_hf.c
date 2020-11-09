/*
 * $QNXLicenseC:
 * Copyright 2020, QNX Software Systems.
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

#include "ipl.h"
#include <hw/inout.h>
#include "private/fat-fs.h"
#include <private/imx8_flexspi_hf_ipl.h>
#include <aarch64/imx8_common/imx_flexspi.h>

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
#define IMX_FSPI_INSTR_RADDR_DDR            0x22        /**< FC row address */
#define IMX_FSPI_INSTR_CADDR_DDR            0x23        /**< FC column address */
#define IMX_FSPI_INSTR_DUMMY_RWDS_DDR       0x2D        /**< FC dummy cycles */

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

#define IMX_FLEXSPI_LUT_AUTOSELECT     1 /* 2 sequences */
#define IMX_FLEXSPI_LUT_READ_STATUS    3
#define IMX_FLEXSPI_LUT_READ           4
#define IMX_FLEXSPI_LUT_ENTER_SFDP_ASO 5
#define IMX_FLEXSPI_LUT_SW_RESET       6

#define SA_TO_OFFSET(_sa)      ((_sa) << 1)
#define DEVICE_ID_TABLE_OFFSET SA_TO_OFFSET(0x800)
#define DEVICE_ID_TABLE_SIZE   (32)


int imx_flexspi_read_hf(flexspi_hf_t *flexspi, unsigned offset, unsigned long buffer, unsigned size);

/**
 * Create a 32-bit sequence code. (One 32-bit sequence contains two instructions.)
 *
 * @param[in]  instr0     instruction opcode 0
 * @param[in]  pad0       number of pads 0 (single/dual/quad/octal mode)
 * @param[in]  opr0       operand 0
 * @param[in]  instr1     instruction opcode 1
 * @param[in]  pad1       number of pads 1 (single/dual/quad/octal mode)
 * @param[in]  opr1       operand 1
 *
 * @return none
 */
imx_fspi_lut_t imx_flexspi_create_lut_record(uint8_t instr0, uint8_t pad0, uint8_t opr0,
                                             uint8_t instr1, uint8_t pad1, uint8_t opr1)
{
    imx_fspi_lut_t lut_rec;

    lut_rec.B.INSTR0 = instr0;
    lut_rec.B.PAD0 = pad0;
    lut_rec.B.OPRND0 = opr0;
    lut_rec.B.INSTR1 = instr1;
    lut_rec.B.PAD1 = pad1;
    lut_rec.B.OPRND1 = opr1;

    return lut_rec;
}

/**
 * Write one 4*32-bit sequence (one index) to LUT table.
 *
 * @param[in]  flexspi     flexspi handle.
 * @param[in]  index       command index
 * @param[in]  lutcmd      sequence codes
 *
 * @return none
 */
void imx_flexspi_write_lut(flexspi_hf_t *flexspi, uint8_t index,
                           imx_fspi_lut_t *lutcmd)
{
    uint8_t inner_index = index * 4;

    out32(flexspi->pbase + IMX_FLEXSPI_LUTa(inner_index),   lutcmd[0].U);
    out32(flexspi->pbase + IMX_FLEXSPI_LUTa(++inner_index), lutcmd[1].U);
    out32(flexspi->pbase + IMX_FLEXSPI_LUTa(++inner_index), lutcmd[2].U);
    out32(flexspi->pbase + IMX_FLEXSPI_LUTa(++inner_index), lutcmd[3].U);
}

/**
 * Setup the flexspi look-up table.
 *
 * @param[in]  flexspi     flexspi handle
 *
 * @return none
 */
static void imx_flexspi_setup_lut(flexspi_hf_t *flexspi)
{
    /* NOTE:
     * This could be simplified by directly setting the various 32-bit values needed for the LUT sequences,
     * instead of using imx_flexspi_create_lut_record().
     * However, for maintainability, using imx_flexspi_create_lut_record() is more practical.
     */
    imx_fspi_lut_t lutAutoSelect[8];
    imx_fspi_lut_t lutIDSFE1_3_1[4];
    imx_fspi_lut_t lutRDVSTR_2_0[4];
    imx_fspi_lut_t lutRead[4];
    imx_fspi_lut_t lutASOEXT_1_1[4];

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
                                               IMX_FSPI_INSTR_DUMMY_RWDS_DDR, IMX_FSPI_PAD_8, 0x0B  /* Latency count */ );
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

    /* Unlock LUT */
    out32(flexspi->pbase + IMX_FLEXSPI_LUTKEY, 0x5AF05AF0);
    out32(flexspi->pbase + IMX_FLEXSPI_LUTCR, 2);

    imx_flexspi_write_lut(flexspi, IMX_FLEXSPI_LUT_AUTOSELECT,     lutAutoSelect);
    imx_flexspi_write_lut(flexspi, IMX_FLEXSPI_LUT_AUTOSELECT+1,   lutAutoSelect+4);
    imx_flexspi_write_lut(flexspi, IMX_FLEXSPI_LUT_READ_STATUS,    lutRDVSTR_2_0);
    imx_flexspi_write_lut(flexspi, IMX_FLEXSPI_LUT_READ,           lutRead);
    imx_flexspi_write_lut(flexspi, IMX_FLEXSPI_LUT_ENTER_SFDP_ASO, lutIDSFE1_3_1);
    imx_flexspi_write_lut(flexspi, IMX_FLEXSPI_LUT_SW_RESET,       lutASOEXT_1_1);

    /* Lock LUT */
    out32(flexspi->pbase + IMX_FLEXSPI_LUTKEY, 0x5AF05AF0);
    out32(flexspi->pbase + IMX_FLEXSPI_LUTCR, 1);
}

/**
 * Set the RX watermark register.
 *
 * @param[in]  flexspi           flexspi handle
 * @param[in]  wtmk              watermark value
 *
 * @return actual watermark value set.
 */
static uint32_t imx_flexspi_set_rx_watermark(flexspi_hf_t *flexspi, uint32_t wtmk)
{
    /* Data transfer size logic must be dword aligned */
    while ((wtmk % 8) != 0) {
        wtmk++;
    }
    /* Set watermark */
    out32(flexspi->pbase + IMX_FLEXSPI_IPRXFCR, (wtmk / 8 - 1) << IMX_FLEXSPI_IPRXFCR_RXWMRK_SHIFT);
    return wtmk;
}

/**
 * Send an IP command.
 *
 * @param[in]  flexspi               flexspi handle
 * @param[in]  lut_index             index of the command
 * @param[in]  num_seq               number of sequential lut sequences of the command
 * @param[in]  data_size_override    size of data following the command 
 *
 * @return 0 on success, -1 on failure.
 */
static int imx_flexspi_send_ip_cmd(flexspi_hf_t *flexspi, uint8_t lut_index, uint8_t num_seq, uint32_t data_size_override)
{
    uint32_t intr;

    /* Wait for idle */
    while (!(in32(flexspi->pbase + IMX_FLEXSPI_STS0) & 0x3));

    out32(flexspi->pbase + IMX_FLEXSPI_FLSHA1CR2,
          in32(flexspi->pbase + IMX_FLEXSPI_FLSHA1CR2) | IMX_FLEXSPI_FLSHA1CR2_CLRINSTRPTR_MASK);

    /* Clear all interrupts */
    out32(flexspi->pbase + IMX_FLEXSPI_INTR, 0xFFF);

    /* Set IP command sequence index */
    out32(flexspi->pbase + IMX_FLEXSPI_IPCR1,
          (lut_index << IMX_FLEXSPI_IPCR1_ISEQID_SHIFT) | ((num_seq-1) << IMX_FLEXSPI_IPCR1_ISEQNUM_SHIFT) | data_size_override);

    /* Trigger IP command */
    out32(flexspi->pbase + IMX_FLEXSPI_IPCMD, 0x1);

    /* Wait for IP command done interrupt */
    while (!(in32(flexspi->pbase + IMX_FLEXSPI_INTR) & IMX_FLEXSPI_INTR_IPCMDDONE_MASK));

    intr = in32(flexspi->pbase + IMX_FLEXSPI_INTR);
    out32(flexspi->pbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPCMDDONE_MASK);

    if (intr & 0xB0A) {
        ser_putstr("  ipcmd error: 0x");
        ser_puthex(intr);
        ser_putstr("\n");
        return -1;
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
static int imx_flexspi_parse_sfdp(flexspi_hf_t *flexspi, uint32_t *manufact_id, uint32_t *device_id, uint32_t *size)
{
    int ret;
    uint8_t ident[DEVICE_ID_TABLE_SIZE] = {0};

    ret = imx_flexspi_read_hf(flexspi, DEVICE_ID_TABLE_OFFSET, (unsigned long)ident, DEVICE_ID_TABLE_SIZE);
    if (ret < 0) {
        ser_putstr("Failed to read Device ID table.\n");
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
        ser_putstr("Unrecognized flash size!\n");
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
static int imx_flexspi_parse_idcfi(flexspi_hf_t *flexspi, uint32_t *manufact_id, uint32_t *device_id, uint32_t *size)
{
    int ret;
    uint8_t ident[DEVICE_ID_TABLE_SIZE] = {0};

    ret = imx_flexspi_read_hf(flexspi, 0, (unsigned long)ident, DEVICE_ID_TABLE_SIZE);
    if (ret < 0) {
        ser_putstr("Failed to read ID-CFI table.\n");
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
        ser_putstr("Unrecognized flash size!\n");
        return -1;
    }
    return 0;
}

/**
 * Reads identification string of the memory device.
 *
 * @param[in]  flexspi     flexspi handle.
 * @param[out] manufact_id manufacturer identification.
 * @param[out] device_id   device memory identification.
 *
 * @return 0 on success, -1 on failure.
 */
static int imx_flexspi_read_ident(flexspi_hf_t *flexspi, uint32_t *manufact_id, uint32_t *device_id, uint32_t *size)
{
    uint8_t offset0[4] = {0};
    int ret;

    ret = imx_flexspi_send_ip_cmd(flexspi, IMX_FLEXSPI_LUT_SW_RESET, 1, 0);
    if (ret >= 0) {
        ret = imx_flexspi_send_ip_cmd(flexspi, IMX_FLEXSPI_LUT_AUTOSELECT, 2, 0);
    }
    if (ret >= 0) {
        ret = imx_flexspi_send_ip_cmd(flexspi, IMX_FLEXSPI_LUT_ENTER_SFDP_ASO, 1, 0);
    }
    if (ret >= 0) {
        ret = imx_flexspi_read_hf(flexspi, 0, (unsigned long)offset0, 4);
    }
    if (ret < 0) {
        return -1;
    }

    ser_putstr("offset0=0x");
    ser_puthex(offset0[0]); ser_putstr(" ");
    ser_puthex(offset0[1]); ser_putstr(" ");
    ser_puthex(offset0[2]); ser_putstr(" ");
    ser_puthex(offset0[3]); ser_putstr("\n");

    if ( (offset0[1] == 'S') && (offset0[0] == 'F') && (offset0[3] == 'D') && (offset0[2] == 'P')) {
        ser_putstr("SFDP found.\n");
        ret = imx_flexspi_parse_sfdp(flexspi, manufact_id, device_id, size);
    }
    else {
        ser_putstr("Standard ID-CFI found.\n");
        ret = imx_flexspi_parse_idcfi(flexspi, manufact_id, device_id, size);
    }

    imx_flexspi_send_ip_cmd(flexspi, IMX_FLEXSPI_LUT_SW_RESET, 1, 0);
    return (ret);
}

/**
 * Initialize flexspi and check hyperflash device identification.
 *
 * @param[in]  flexspi     flexspi handle
 * @param[in]  base        register base address of module
 * @param[out] mid         manufacturer identification.
 * @param[out] did         device memory identification.
 *
 * @return 0 on success, -1 on failure.
 */
int imx_flexspi_init_hf(flexspi_hf_t *flexspi, unsigned base, int verbose)
{
    int status;
    uint32_t reg;
    uint32_t manufact_id = 0, device_id = 0, size = 0;

    flexspi->pbase = base;
    flexspi->verbose = verbose;

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
    reg = in32(flexspi->pbase + IMX_FLEXSPI_MCR0);
    reg |= IMX_FLEXSPI_MCR0_MDIS_MASK;
    out32(flexspi->pbase + IMX_FLEXSPI_MCR0, reg);

    /* Configure MCR0, MCR1, and MCR2 */
    reg = in32(flexspi->pbase + IMX_FLEXSPI_MCR0);
    reg &= ~(IMX_FLEXSPI_MCR0_ARDFEN_MASK | IMX_FLEXSPI_MCR0_ATDFEN_MASK | IMX_FLEXSPI_MCR0_RXCLKSRC_MASK | IMX_FLEXSPI_MCR0_COMBINATIONEN_MASK);
    reg |= ((flexspi->conf->pads == 8) ? IMX_FLEXSPI_MCR0_COMBINATIONEN_MASK : 0) |
           (flexspi->conf->smpl << IMX_FLEXSPI_MCR0_RXCLKSRC_SHIFT);
    out32(flexspi->pbase + IMX_FLEXSPI_MCR0, reg);

    /* Set max SEQ and AHB timeout */
    reg = 0xffffffff;
    out32(flexspi->pbase + IMX_FLEXSPI_MCR1, reg);

    reg = IMX_FLEXSPI_MCR2_RESUMEWAIT_MASK |
              IMX_FLEXSPI_MCR2_SCKBDIFFOPT_MASK |
              IMX_FLEXSPI_MCR2_CLRLEARNPHASE_MASK;
    out32(flexspi->pbase + IMX_FLEXSPI_MCR2, reg);

    /* Configure FLSHxCR0,FLSHxCR1,FLSHxCR2 */

    /* TODO:
     * During testing, read/write of the last 16 bytes of the specified size always fails.
     * Informing the SOC the device has additional space seems to work around the problem.
     * Need more investigation, or inquire NXP.
     */
    reg = in32(flexspi->pbase + IMX_FLEXSPI_FLSHA1CR0);
    reg = (reg & ~IMX_FLEXSPI_FLSHA1CR0_FLSHSZ_MASK) | ((flexspi->conf->size >> 11) + 0xf); /* In KBs: /1024 /2 */
    out32(flexspi->pbase + IMX_FLEXSPI_FLSHA1CR0, reg);

    /* See iMX8 RM section 18.2.4.6 - Otherwise, by default, offset will
       refer only to row accesses (i.e. in half-page increments; 16-byte increments). */
    reg = in32(flexspi->pbase + IMX_FLEXSPI_FLSHA1CR1);
    reg &= ~(IMX_FLEXSPI_FLSHA1CR1_CAS_MASK | IMX_FLEXSPI_FLSHA1CR1_WA_MASK);
    reg |= (flexspi->conf->col_width << IMX_FLEXSPI_FLSHA1CR1_CAS_SHIFT) |
           IMX_FLEXSPI_FLSHA1CR2_CLRINSTRPTR_MASK |
           (flexspi->conf->word_addr ? 1 : 0);
    out32(flexspi->pbase + IMX_FLEXSPI_FLSHA1CR1, reg);

    /* Configure DLL control register DLLxCR */
    reg = (IMX_FLEXSPI_DLLACR_SLVDLYTARGET_MASK) | IMX_FLEXSPI_DLLACR_DLLEN_MASK;
    out32(flexspi->pbase + IMX_FLEXSPI_DLLACR, reg);

    reg = ((0xf << IMX_FLEXSPI_DLLBCR_SLVDLYTARGET_SHIFT) & IMX_FLEXSPI_DLLBCR_SLVDLYTARGET_MASK) |
            IMX_FLEXSPI_DLLACR_DLLEN_MASK;
    out32(flexspi->pbase + IMX_FLEXSPI_DLLBCR, reg);

    /* Exit module stop mode */
    reg = in32(flexspi->pbase + IMX_FLEXSPI_MCR0);
    reg &= ~IMX_FLEXSPI_MCR0_MDIS_MASK;
    out32(flexspi->pbase + IMX_FLEXSPI_MCR0, reg);

    /* Setup LUT */
    imx_flexspi_setup_lut(flexspi);

    /* Clear RX and TX FIFOs */
    out32(flexspi->pbase + IMX_FLEXSPI_IPTXFCR, IMX_FLEXSPI_IPTXFCR_CLRIPTXF_MASK);
    out32(flexspi->pbase + IMX_FLEXSPI_IPRXFCR, IMX_FLEXSPI_IPRXFCR_CLRIPRXF_MASK);

    /* Disable all interrupts */
    out32(flexspi->pbase + IMX_FLEXSPI_INTEN, 0x0);

    /* Clear all interrupts */
    out32(flexspi->pbase + IMX_FLEXSPI_INTR, 0xFFF);

    /* Begin read */
    status = imx_flexspi_read_ident(flexspi, &manufact_id, &device_id, &size);

    if (status >= 0) {
        flexspi_conf_hf_t *conf = flexspi->conf;

        ser_putstr("mid=0x");
        ser_puthex(manufact_id);
        ser_putstr(", did=0x");
        ser_puthex(device_id);
        ser_putstr(", size=0x");
        ser_puthex(size);
        ser_putstr("\n");

        status = -1;
        while (conf->name != (char*)0) {
            if ((conf->mid == manufact_id) &&
                (conf->did == device_id)) {
                ser_putstr("Detected: ");
                ser_putstr(conf->name);
                ser_putstr("\n\n");
                status = 0;
                break;
            }
            conf++;
        }
    }

    if (status != 0) {
        ser_putstr("Flash device not supported!\n");
        return -1;
    }
    return 0;
}

/**
 * Read data from the hyperflash device offset into the specified buffer.
 *
 * @param[in]  flexspi     flexspi handle
 * @param[in]  offset      offset to read data
 * @param[out] buffer      destination buffer
 * @param[in]  size        size of data to read
 *
 * @return 0 on success, -1 on failure.
 */
int imx_flexspi_read_hf(flexspi_hf_t *flexspi, unsigned offset, unsigned long buffer, unsigned size)
{
    uint32_t cur_size;
    uint32_t watermark;
    uint32_t remaining = size;

    /* Send 1 read command every fifo size, and perform 1 copy every watermark. */

    /* Divide by 2, since addressable unit is 2 bytes */
    offset >>= 1;

    while (remaining > 0) {
        cur_size = MIN(remaining, flexspi->conf->rx_fifo_size);

        /* Make sure cur_size is always divisible by FLEXSPI_MAX_BURST_RX,
         * except for the very final read.
         * Code assumes FLEXSPI_MAX_BURST_RX is a power-of-2 value.
         */
        if (cur_size > FLEXSPI_MAX_BURST_RX) {
            cur_size -= (cur_size & (FLEXSPI_MAX_BURST_RX-1));
        }
        watermark = MIN(cur_size, FLEXSPI_MAX_BURST_RX);

        /* Read */

        /* Start address */
        out32(flexspi->pbase + IMX_FLEXSPI_IPCR0, offset);

        /* Increment offset. Divide cur_size by 2, since addressable unit is 2 bytes */
        offset += (cur_size >> 1);

        /* Clear Rx FIFO */
        out32(flexspi->pbase + IMX_FLEXSPI_IPRXFCR, in32(flexspi->pbase + IMX_FLEXSPI_IPRXFCR) | IMX_FLEXSPI_IPRXFCR_CLRIPRXF_MASK);

        /* Set interrupt data threshold */
        imx_flexspi_set_rx_watermark(flexspi, watermark);

        if (imx_flexspi_send_ip_cmd(flexspi, IMX_FLEXSPI_LUT_READ, 1, cur_size) < 0) {
            ser_putstr("IP command failure! Aborting. remaining=");
            ser_puthex(remaining);
            ser_putstr(", offset=0x");
            ser_puthex(offset);
            ser_putstr(", cur_size=0x");
            ser_puthex(cur_size);
            ser_putstr("\n");
            return -1;
        }
        while (cur_size > 0) {

            /* Wait for watermark amount of data to be available */
            while (!(in32(flexspi->pbase + IMX_FLEXSPI_INTR) & IMX_FLEXSPI_INTR_IPRXWA_MASK));

            /* Read data from hw */
            copy((unsigned long)buffer, (flexspi->pbase + IMX_FLEXSPI_RFDR0), watermark);

            /* Clear IP and Rx flags */
            out32(flexspi->pbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPRXWA_MASK | IMX_FLEXSPI_INTR_IPCMDDONE_MASK);

            cur_size -= watermark;
            remaining -= watermark;
            buffer += watermark;
        }
    }
    return 0;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/lib/flexspi/imx8_flexspi_hf.c $ $Rev: 910001 $")
#endif
