/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
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

#include <hw/inout.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include "ipl.h"
#include "private/rcar_gen3_sdhi.h"
#include "private/rcar_gen3_delay.h"
#include <aarch64/r-car-gen3.h>

#define TUNING_4BIT_BLK_SIZE        64
#define TUNING_8BIT_BLK_SIZE        128

static sdmmc_hc_t rcar_gen3_hc;
static rcar_gen3_sdmmc_bs_t *rcar_gen3_bs;

/*
 * SDMMC command table
 */
static const struct cmd_str
{
    int         cmd;
    unsigned    sdmmc_cmd;
} cmdtab[] =
{
    // MMC_GO_IDLE_STATE
    { 0,            MMC_CMD0 },
    // MMC_SEND_OP_COND (R3)
    { 1,            MMC_CMD1 },
    // MMC_ALL_SEND_CID (R2)
    { 2,            MMC_CMD2 },
    // MMC_SET_RELATIVE_ADDR (R6)
    { 3,            MMC_CMD3 },
    // MMC_SWITCH (R1b)
    { 6,            MMC_CMD6 },
    // MMC_SEL_DES_CARD (R1b)
    { 7,            MMC_CMD7 },
    // MMC_IF_COND (R7)
    { 8,            MMC_CMD8 },
    // MMC_SEND_CSD (R2)
    { 9,            MMC_CMD9 },
    // MMC_SEND_STATUS (R1)
    { 13,           MMC_CMD13 },
    // MMC_SET_BLOCKLEN (R1)
    { 16,           MMC_CMD16 },
    // MMC_READ_SINGLE_BLOCK (R1)
    { 17,           MMC_CMD17 },
    // MMC_READ_MULTIPLE_BLOCK (R1)
    { 18,           MMC_CMD18 },
    // MMC_APP_CMD (R1)
    { 55,           MMC_CMD55 },
    // SD_SET_BUS_WIDTH (R1)
    { (55 << 8) | 6,  MMC_ACMD6 },
    // SD_SEND_OP_COND (R3)
    { (55 << 8) | 41,  MMC_ACMD41 },
    // end of command list
    { -1 },
};

/* searches for a command in the command table */
static struct cmd_str *get_cmd(int cmd)
{
    struct cmd_str *command = (struct cmd_str *)&cmdtab[0];

    while (command->cmd != -1) {
        if (command->cmd == cmd) {
            return command;
        }
        command++;
    }

    return 0;
}

const uint8_t sdio_tbp_4bit[] = {
  0xff, 0x0f, 0xff, 0x00, 0xff, 0xcc, 0xc3, 0xcc,
  0xc3, 0x3c, 0xcc, 0xff, 0xfe, 0xff, 0xfe, 0xef,
  0xff, 0xdf, 0xff, 0xdd, 0xff, 0xfb, 0xff, 0xfb,
  0xbf, 0xff, 0x7f, 0xff, 0x77, 0xf7, 0xbd, 0xef,
  0xff, 0xf0, 0xff, 0xf0, 0x0f, 0xfc, 0xcc, 0x3c,
  0xcc, 0x33, 0xcc, 0xcf, 0xff, 0xef, 0xff, 0xee,
  0xff, 0xfd, 0xff, 0xfd, 0xdf, 0xff, 0xbf, 0xff,
  0xbb, 0xff, 0xf7, 0xff, 0xf7, 0x7f, 0x7b, 0xde,
};

// tuning block pattern for 8 bit mode
const uint8_t sdio_tbp_8bit[] = {
  0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00,
  0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc, 0xcc,
  0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff, 0xff,
  0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee, 0xff,
  0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd, 0xdd,
  0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff, 0xbb,
  0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff, 0xff,
  0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee, 0xff,
  0xff, 0xff, 0xff, 0x00, 0xff, 0xff, 0xff, 0x00,
  0x00, 0xff, 0xff, 0xcc, 0xcc, 0xcc, 0x33, 0xcc,
  0xcc, 0xcc, 0x33, 0x33, 0xcc, 0xcc, 0xcc, 0xff,
  0xff, 0xff, 0xee, 0xff, 0xff, 0xff, 0xee, 0xee,
  0xff, 0xff, 0xff, 0xdd, 0xff, 0xff, 0xff, 0xdd,
  0xdd, 0xff, 0xff, 0xff, 0xbb, 0xff, 0xff, 0xff,
  0xbb, 0xbb, 0xff, 0xff, 0xff, 0x77, 0xff, 0xff,
  0xff, 0x77, 0x77, 0xff, 0x77, 0xbb, 0xdd, 0xee,
};

/*------------------------------------------*/
/* R-Car Gen3 SDMMC generic driver code     */
/*------------------------------------------*/

/* This function (with magic numbers) was reused from devb driver:
 * http://svn.ott.qnx.com/view/product/mainline/hardware/devb/sdmmc/sdiodi/hc/rcar_gen3.c?view=markup&pathrev=836542
 */
static uint8_t clock_div(int hclk, int *clock) {
    uint32_t clk;
    int      new_clock;


    for (new_clock = hclk/512, clk = 0x80000080; *clock >= (new_clock * 2); clk >>= 1) {
        new_clock <<= 1;
    }

    *clock = new_clock;

    if ((clk >> 22) & 1) {
        clk |= 0xff;
    }

    return (uint8_t)clk;
}

/* sets the sdmmc clock frequency */
static void rcar_gen3_set_frq(sdmmc_t *sdmmc, unsigned frq) {
    sdmmc_hc_t  *hc = sdmmc->hc;
    unsigned    base = hc->sdmmc_pbase;
    uint8_t     clkctl;
    int         clock;
    uint32_t    info2;
    int         cnt;

    for (cnt = SDHI_TMOUT; cnt >= 0; --cnt) {
        info2 = sdhi_read(base, MMC_SD_INFO2);
        if (!(info2 & SDH_INFO2_CBSY) && (info2 & SDH_INFO2_SCLKDIVEN))
            break;
    }

    if (cnt <= 0) {
        if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
            ser_putstr("set_frq: Busy state! Cannot change the clock!\n");
        }
        return;
    }

    frq = frq * 1000;

    /* stop clock */
    sdhi_write(base, MMC_SD_CLK_CTRL, sdhi_read(base, MMC_SD_CLK_CTRL) & ~SDH_CLKCTRL_SCLKEN);

    if (frq > hc->clock) {
        frq = hc->clock;
    }

    clock = frq;

    clkctl = clock_div(hc->clock, &clock);

    if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
        ser_putstr("Set clock to ");
        ser_putdec(frq/1000);
        ser_putstr("KHz ref clock ");
        ser_putdec(hc->clock/1000);
        ser_putstr(" clkctl 0x");
        ser_puthex(clkctl);
        ser_putstr("\n");
    }

    sdhi_write(base, MMC_SD_CLK_CTRL, clkctl);

    for (cnt = SDHI_TMOUT; cnt >= 0; --cnt) {
        if (((sdhi_read(base, MMC_SD_INFO2) & SDH_INFO2_SCLKDIVEN)))
            break;
    }

    if (cnt <= 0) {
        if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
            ser_putstr("set_frq: time out waiting for CLKDIVEN\n");
        }
        return;
    }


    sdhi_write(base, MMC_SD_CLK_CTRL, sdhi_read(base, MMC_SD_CLK_CTRL) | SDH_CLKCTRL_SCLKEN);
}

/* sets the data bus mode */
static void rcar_gen3_set_bus_mode(sdmmc_t *sdmmc, int mode) {
    return;
}

/* sets the data bus width */
static void rcar_gen3_set_bus_width(sdmmc_t *sdmmc, int width) {
    unsigned    base = sdmmc->hc->sdmmc_pbase;
    uint32_t    hctl;
    int         i, status;

    if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
        ser_putstr("Bus width ");
        ser_putdec(width);
        ser_putstr("\n");
    }

    for (i = 0; i < SDHI_TMOUT; i++) {
        status =  sdhi_read(base, MMC_SD_INFO2);
        if (!(status & SDH_INFO2_CBSY) && (status & SDH_INFO2_SCLKDIVEN))
            break;
    }

    if (i >= SDHI_TMOUT) {
        if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
            ser_putstr("set_bus_width: Busy state! Cannot change the bus width!\n");
        }
        return;
    }

    hctl = sdhi_read(base, MMC_SD_OPTION);

    if (width == 8) {
        hctl &= ~(SDH_OPTION_WIDTH_1);
        hctl |=  (SDH_OPTION_WIDTH_8);
    }
    else if (width == 4) {
        hctl &= ~(SDH_OPTION_WIDTH_1 | SDH_OPTION_WIDTH_8);
    }
    else {
        hctl |=  (SDH_OPTION_WIDTH_1);
        hctl &= ~(SDH_OPTION_WIDTH_8);
    }

    sdhi_write(base, MMC_SD_OPTION, hctl);
}

static void rcar_gen3_set_timing(sdmmc_t *sdmmc, int timing) {
    unsigned base = sdmmc->hc->sdmmc_pbase;

    sdhi_write(base, MMC_SD_CLK_CTRL, sdhi_read(base, MMC_SD_CLK_CTRL) & ~SDH_CLKCTRL_SCLKEN);

    /* Reset HS4000 mode */
    sdhi_write(base, MMC_SDIF_MODE, sdhi_read(base, MMC_SDIF_MODE) & ~SDIF_MODE_HS400);
    sdhi_write(base, MMC_SCC_TMPPORT2,  sdhi_read(base, MMC_SCC_TMPPORT2) &
            ~(RCAR_SDHI_SCC_TMPPORT2_HS400OSEL | RCAR_SDHI_SCC_TMPPORT2_HS400EN));

    sdhi_write(base, MMC_SD_CLK_CTRL, sdhi_read(base, MMC_SD_CLK_CTRL) | SDH_CLKCTRL_SCLKEN);
}

/* initializes the controller */
static int rcar_gen3_init_ctrl(sdmmc_t *sdmmc) {
    unsigned base = sdmmc->hc->sdmmc_pbase;
    int i, status;

    sdmmc->icr  = 0x000001aa;             /* 2.7 V - 3.6 V */
    sdmmc->ocr  = 0x00300000;             /* 3.2 V - 3.4 V */

    sdhi_write(base, MMC_SOFT_RST, SOFT_RST_ON);
    sdhi_write(base, MMC_SOFT_RST, SOFT_RST_OFF);

    for (i = 0; i < SDHI_TMOUT; i++) {
        status =  sdhi_read(base, MMC_SD_INFO2);
        if (!(status & SDH_INFO2_CBSY)) {
            break;
        }
    }

    if (i >= SDHI_TMOUT) {
        if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
            ser_putstr("init_ctrl: failed waiting for INFO2_CBUSY\n");
        }
        return EIO;
    }

    rcar_gen3_set_frq(sdmmc, 400);
    sdhi_write(base, MMC_SD_OPTION, SDH_OPTION_RESERVED | SDH_OPTION_WIDTH_1 |
        SDH_OPTION_CTOP_MAX | SDH_OPTION_TOP_MAX);
    sdhi_write(base, MMC_HOST_MODE, MMC_HOST_MODE_BUS_WIDTH_32 | MMC_HOST_MODE_BUF_WIDTH_32);

    if(rcar_gen3_bs && rcar_gen3_bs->init) {
        rcar_gen3_bs->init(sdmmc);
    } else {
        rcar_gen3_usec_delay(10000);
    }

    return SDMMC_OK;
}

/* clean up the SDMMC controller */
int rcar_gen3_sdmmc_fini(sdmmc_t *sdmmc) {
    return 0;
}

static void rcar_sdmmc_dma_start(sdmmc_t *sdmmc) {
    unsigned        base = sdmmc->hc->sdmmc_pbase;

    // Clear status
    sdhi_write(base, MMC_DM_CM_INFO1, 0x00000000);
    sdhi_write(base, MMC_DM_CM_INFO2, 0x00000000);

    // Start DMA
    sdhi_write(base, MMC_DM_CM_DTRAN_CTRL, DM_START);
}

static void rcar_sdmmc_dma_cmplt(sdmmc_t *sdmmc) {
    unsigned        base = sdmmc->hc->sdmmc_pbase;

    /* Clear DMA info */
    sdhi_write(base, MMC_DM_CM_INFO1, 0x00000000);
    sdhi_write(base, MMC_DM_CM_INFO2, 0x00000000);

    /* The SD_BUF read/write DMA transfer is disabled */
    sdhi_write(base, MMC_CC_EXT_MODE, BUF_ACC_DMAWDIS);
}

static int rcar_sdmmc_dma_setup(sdmmc_t *sdmmc) {
    unsigned        base = sdmmc->hc->sdmmc_pbase;
    sdmmc_cmd_t     *cmd  = &sdmmc->cmd;

    /* switch host mode to 64-bit */
    sdhi_write(base, MMC_HOST_MODE, MMC_HOST_MODE_BUF_WIDTH_64);

    /* Enable read/write by DMA */
    sdhi_write(base, MMC_CC_EXT_MODE, BUF_ACC_DMAWEN);

    /* Set the address mode
     * SDMMC lib doesn't seem to send write commands
     */
    sdhi_write(base, MMC_DM_CM_DTRAN_MODE, CH_NUM_UPSTREAM | BUS_WID_64BIT | INCREMENT_ADDRESS);

    /* Set the SDMA address (assuming well-aligned address from sdmmc lib */
    sdhi_write(base, MMC_DM_DTRAN_ADDR, (uintptr_t)cmd->dbuf);

    return (EOK);
}


/* issues a command on the SDMMC bus */
static int rcar_gen3_send_cmd(sdmmc_t *sdmmc) {
    struct cmd_str  *command;
    unsigned        base = sdmmc->hc->sdmmc_pbase;
    sdmmc_cmd_t     *cmd  = &sdmmc->cmd;
    int             status, status2, i;
    int             data_txf = 0;

    if (0 == (command = get_cmd (cmd->cmd))) {
        ser_putstr("Failed to get SDMMC CMD");ser_putdec(cmd->cmd);ser_putstr("\n");
        return SDMMC_ERROR;
    }

    if (sdmmc->verbose > SDMMC_VERBOSE_LVL_2) {
        ser_putstr("rcar_gen3_send_cmd(), CMD");
        ser_putdec(command->cmd);
        ser_putstr(" (");
        ser_puthex(command->sdmmc_cmd);
        ser_putstr(") ARG: 0x");
        ser_puthex(cmd->arg);
        ser_putstr("\n");
    }

    for (i = 0; i < SDHI_TMOUT; i++) {
        status =  sdhi_read(base, MMC_SD_INFO2);
        if (!(status & SDH_INFO2_CBSY) && (status & SDH_INFO2_SCLKDIVEN)) {
            break;
        }
    }

    if (i >= SDHI_TMOUT) {
        if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
            ser_putstr("send_cmd: Busy state! Cannot execute because BUS busy\n");
        }
        return -1;
    }

    /* Clear Status */
    sdhi_write(base, MMC_SD_INFO1, sdhi_read(base, MMC_SD_INFO1) & ~(SDH_INFO1_AE | SDH_INFO1_RE));
    sdhi_write(base, MMC_SD_INFO2, sdhi_read(base, MMC_SD_INFO2) & ~SDH_INFO2_ALL_ERR);

    /* check if need data transfer */
    data_txf = command->sdmmc_cmd & SDH_CMD_ADTC;

    if (data_txf) {
        /* block size */
        sdhi_write(base, MMC_SD_SIZE, cmd->bsize);

        /* only valid for multi-block transfer */
        if (cmd->bcnt > 1) {
            sdhi_write(base, MMC_SD_SECCNT, cmd->bcnt);
            sdhi_write(base, MMC_SD_STOP, SDH_STOP_SEC);
        } else {
            sdhi_write(base, MMC_SD_STOP, SDH_STOP_DEFAULT);
        }

        rcar_sdmmc_dma_setup(sdmmc);
    }

    /* Write command and argument */
    sdhi_write(base, MMC_SD_ARG, cmd->arg);
    sdhi_write(base, MMC_SD_CMD, command->sdmmc_cmd);

    /* wait for command finish */
    for (i = 0; i < SDHI_TMOUT; i++) {
        status =  sdhi_read(base, MMC_SD_INFO1);
        status2 = sdhi_read(base, MMC_SD_INFO2);
        if (status & SDH_INFO1_RE) {
            sdhi_write(base, MMC_SD_INFO1, ~SDH_INFO1_RE);
            if (data_txf) {
                if (status2 & SDH_INFO2_ALL_ERR) {
                    ser_putstr("send_cmd: error detected with response to data transfer\n");
                } else {
                    rcar_sdmmc_dma_start(sdmmc);
                }
            }
            break;
        }

        rcar_gen3_usec_delay(10);
    }

    if (i >= SDHI_TMOUT) {
        if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
            ser_putstr("send_cmd: timeout waiting for a response after sending a command\n");
        }
        return SDMMC_ERROR;
    }

    /* check error status */
    unsigned temp = sdhi_read(base, MMC_SD_INFO2);
    if (temp & SDH_INFO2_ALL_ERR) {
        cmd->erintsts = temp;
        /* We know some expected errors in IDLE state */
        if (sdmmc->card.state != MMC_IDLE) {
            ser_putstr("send_cmd() failed at CMD");
            ser_putdec(command->cmd);
            ser_putstr(" MMC_SD_INFO2: 0x");
            ser_puthex(temp);
            ser_putstr("\n");
        }
        return SDMMC_ERROR;
    }

    if (data_txf) {
        /* wait for end of data transfer */
        while(1) {
            status =  sdhi_read(base, MMC_SD_INFO1);
            if (status & SDH_INFO1_AE) {
                sdhi_write(base, MMC_SD_INFO1, ~SDH_INFO1_AE & ~SDH_INFO1_RE);
                rcar_sdmmc_dma_cmplt(sdmmc);
                break;
            }

            status = sdhi_read(base, MMC_SD_INFO2);

            if (status & SDH_INFO2_ALL_ERR) {
                rcar_sdmmc_dma_cmplt(sdmmc);
                break;
            }

            rcar_gen3_usec_delay(10);
        }
    }

    /* get command response */
    if (cmd->rsp != 0) {
        if ((command->sdmmc_cmd & SDH_CMD_RSPR2)) {
            uint32_t    *resp = &cmd->rsp[0];
            resp[3] = sdhi_read(base, MMC_SD_RSP76);
            resp[2] = sdhi_read(base, MMC_SD_RSP54);
            resp[1] = sdhi_read(base, MMC_SD_RSP32);
            resp[0] = sdhi_read(base, MMC_SD_RSP10);

            /*
            * CRC is not included in the response register,
            * we have to left shift 8 bit to match the 128 CID/CSD structure
            */
            resp[3] = (resp[3] << 8) | (resp[2] >> 24);
            resp[2] = (resp[2] << 8) | (resp[1] >> 24);
            resp[1] = (resp[1] << 8) | (resp[0] >> 24);
            resp[0] = (resp[0] << 8);
        } else {
            cmd->rsp[0] = sdhi_read(base, MMC_SD_RSP10);
        }
    }

    return SDMMC_OK;
}

static int rcar_gen3_pio_read(sdmmc_t *sdmmc, void *buf, unsigned len) {
    unsigned        base = sdmmc->hc->sdmmc_pbase;
    sdmmc_cmd_t     *cmd  = &sdmmc->cmd;
    int             status;
    int             i;
    unsigned        *pbuf = (unsigned *)buf;
    uint32_t        sdmmc_cmd = cmd->cmd;

    /* wait for ready */
    for (i = 0; i < SDHI_TMOUT; i++) {
        status =  sdhi_read(base, MMC_SD_INFO2);
        if (!(status & SDH_INFO2_CBSY) && (status & SDH_INFO2_SCLKDIVEN))
            break;
    }

    if (i >= SDHI_TMOUT) {
        if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
            ser_putstr("pio_read: Busy state! Cannot execute because BUS busy\n");
        }
        return -1;
    }

    /* setup PIO read */
    sdhi_write(base, MMC_SD_SIZE, len);

    /* Clear Status */
    sdhi_write(base, MMC_SD_INFO1, sdhi_read(base, MMC_SD_INFO1) & ~(SDH_INFO1_AE | SDH_INFO1_RE));
    sdhi_write(base, MMC_SD_INFO2, sdhi_read(base, MMC_SD_INFO2) & ~SDH_INFO2_ALL_ERR);
    sdhi_write(base, MMC_SD_INFO2, sdhi_read(base, MMC_SD_INFO2) & ~SDH_INFO2_BRE);

    switch (cmd->cmd) {
        case MMC_SEND_EXT_CSD:
        case MMC_SEND_TUNING_BLOCK:
            sdmmc_cmd |= SDH_CMD_RSPR1;
            break;
        case MMC_SEND_SCR:
            sdmmc_cmd |= SDH_CMD_ACMD;
            break;
        default:
            break;
    }

    /* setup the argument register and send command */
    sdhi_write(base, MMC_SD_ARG, cmd->arg);
    sdhi_write(base, MMC_SD_CMD, sdmmc_cmd | SDH_CMD_DAT_READ | SDH_CMD_ADTC);

    /* wait for command finish */
    while (!(sdhi_read(base, MMC_SD_INFO2) & (SDH_INFO2_BRE  | SDH_INFO2_ALL_ERR )))
        ;

    /* check error status */
    if (sdhi_read(base, MMC_SD_INFO2) & SDH_INFO2_ALL_ERR) {
        cmd->erintsts = sdhi_read(base, MMC_SD_INFO2);

        if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
            ser_putstr("rcar_gen3_pio_read() failed. MMC_SD_INFO2: 0x");
            ser_puthex(cmd->erintsts);
            ser_putstr("\n");
        }

        return SDMMC_ERROR;
    }

    /* get command response */
    if (cmd->rsp != 0) {
        cmd->rsp[0] = sdhi_read(base, MMC_SD_RSP10);
    }

    /* now read from FIFO */
    for (; len > 0; len -= 4){
        sdhi_read(base, MMC_SD_SIZE);
        sdhi_write(base, MMC_SD_INFO2, ~SDH_INFO2_BRE);
        *pbuf++ = sdhi_read(base, MMC_SD_BUF0);

        if (sdhi_read(base, MMC_SD_INFO1) & SDH_INFO1_AE) {
            sdhi_write(base, MMC_SD_INFO1, sdhi_read(base, MMC_SD_INFO1) & ~(SDH_INFO1_AE| SDH_INFO1_RE));
            break;
        }
    }

    sdhi_write(base, MMC_SD_INFO1, sdhi_read(base, MMC_SD_INFO1) & ~(SDH_INFO1_AE| SDH_INFO1_RE));

    return SDMMC_OK;
}

static int rcar_gen3_dma_mxblks(sdmmc_t *sdmmc) {
    /* No limit */
    return ~0;
}


static int rcar_gen3_signal_voltage( sdmmc_t *sdmmc, int signal_voltage )
{
    return(SDMMC_OK);
}


static int rcar_sdmmc_select_tuning(sdmmc_t *sdmmc, int tap_num, int *taps, int *smpcmp)
{
    sdmmc_hc_t  *hc = sdmmc->hc;
    unsigned    base = hc->sdmmc_pbase;
    uint32_t    tap_cnt;    /* counter of tuning success */
    uint32_t    tap_set;    /* tap position */
    uint32_t    tap_start;  /* start position of tuning success */
    uint32_t    tap_end;    /* end position of tuning success */
    uint32_t    ntap;       /* temporary counter of tuning success */
    uint32_t    match_cnt;  /* counter of matching data */
    int         i;
    int         select = 0;

    /* Clear SCC_RVSREQ */
    sdhi_write(base, MMC_SCC_RVSREQ, 0x00000000);

    /* Merge the results */
    for (i = 0; i < tap_num * 2; i++) {
        if (!taps[i]) {
            taps[i % tap_num] = 0;
            taps[(i % tap_num) + tap_num] = 0;
        }
        if (!smpcmp[i]) {
            smpcmp[i % tap_num] = 0;
            smpcmp[(i % tap_num) + tap_num] = 0;
        }
    }

    /*
     * Find the longest consecutive run of successful probes.  If that
     * is more than RCAR_SDHI_MAX_TAP probes long then use the
     * center index as the tap.
     */
    tap_cnt   = 0;
    ntap      = 0;
    tap_start = 0;
    tap_end = 0;
    for (i = 0; i < tap_num * 2; i++) {
        if (taps[i]) {
            ntap++;
        } else {
            if (ntap > tap_cnt) {
                tap_start = i - ntap;
                tap_end = i - 1;
                tap_cnt = ntap;
            }
            ntap = 0;
        }
    }

    if (ntap > tap_cnt) {
        tap_start = i - ntap;
        tap_end = i - 1;
        tap_cnt = ntap;
    }

    /*
     * If all of the TAP is OK, the sampling clock position is selected by
     * identifying the change point of data.
     */
    if (tap_cnt == tap_num * 2) {
        match_cnt = 0;
        ntap = 0;
        tap_start = 0;
        tap_end = 0;
        for (i = 0; i < tap_num * 2; i++) {
            if (smpcmp[i]) {
                ntap++;
            } else {
                if (ntap > match_cnt) {
                    tap_start = i - ntap;
                    tap_end = i - 1;
                    match_cnt = ntap;
                }
                ntap = 0;
            }
        }
        if (ntap > match_cnt) {
            tap_start = i - ntap;
            tap_end = i - 1;
            match_cnt = ntap;
        }
        if (match_cnt)
            select = 1;
    } else if (tap_cnt >= RCAR_SDHI_MAX_TAP) {
        select = 1;
    }

    if (select) {
        tap_set = ((tap_start + tap_end) / 2) % tap_num;
    } else {
        return (EIO);
    }

    /* Set SCC */
    sdhi_write(base, MMC_SCC_TAPSET, tap_set);

    /* Enable auto re-tuning */
    sdhi_write(base, MMC_SCC_RVSCNTL, (1 << 1) | RCAR_SDHI_SCC_RVSCNTL_RVSEN | sdhi_read(base, MMC_SCC_RVSCNTL));

    return (EOK);
}

static int rcar_sdmmc_init_tuning(sdmmc_t *sdmmc)
{
    sdmmc_hc_t  *hc = sdmmc->hc;
    unsigned    base = hc->sdmmc_pbase;
    int         scc_tapnum;
    int         taps_num;

    scc_tapnum  = 8;

    /* Initialize SCC */
    sdhi_write(base, MMC_SD_INFO1, 0x0000);
    sdhi_write(base, MMC_SD_INFO2, 0x0000);

    sdhi_write(base, MMC_SD_CLK_CTRL, sdhi_read(base, MMC_SD_CLK_CTRL) & ~SDH_CLKCTRL_SCLKEN); //Stop clock

    /* set sampling clock selection range */
    sdhi_write(base, MMC_SCC_DTCNTL, RCAR_SDHI_SCC_DTCNTL_TAPEN | (scc_tapnum << 16));

    sdhi_write(base, MMC_SCC_CKSEL, RCAR_SDHI_SCC_CKSEL_DTSEL | sdhi_read(base, MMC_SCC_CKSEL));

    sdhi_write(base, MMC_SCC_RVSCNTL, (1 << 1) | (~RCAR_SDHI_SCC_RVSCNTL_RVSEN & sdhi_read(base, MMC_SCC_RVSCNTL)));

    sdhi_write(base, MMC_SCC_DT2FF, 0x00000300);

    sdhi_write(base, MMC_SD_CLK_CTRL, sdhi_read(base, MMC_SD_CLK_CTRL) | SDH_CLKCTRL_SCLKEN); //Start clock

    /* Read TAPNUM */
    taps_num = (sdhi_read(base, MMC_SCC_DTCNTL) >> 16) & 0xFF;

    return (taps_num);
}

static int rcar_sdmmc_prepare_tuning(sdmmc_t *sdmmc, uint32_t tap)
{
    sdmmc_hc_t  *hc = sdmmc->hc;
    unsigned    base = hc->sdmmc_pbase;

    /* Set sampling clock position */
    sdhi_write(base, MMC_SCC_TAPSET, tap);

    return (EOK);
}

static unsigned int rcar_sdmmc_compare_scc_data(sdmmc_t *sdmmc)
{
    unsigned    base = sdmmc->hc->sdmmc_pbase;

    /* Get comparison of sampling data */
    return sdhi_read(base, MMC_SCC_SMPCMP);
}

static void rcar_sdmmc_scc_reset(sdmmc_t *sdmmc)
{
    unsigned    base = sdmmc->hc->sdmmc_pbase;

    sdhi_write(base, MMC_SD_CLK_CTRL, sdhi_read(base, MMC_SD_CLK_CTRL) & ~SDH_CLKCTRL_SCLKEN);  //Stop

    /* Reset SCC */
    sdhi_write(base, MMC_SCC_CKSEL, ~RCAR_SDHI_SCC_CKSEL_DTSEL & sdhi_read(base, MMC_SCC_CKSEL));

    sdhi_write(base, MMC_SCC_DTCNTL, ~RCAR_SDHI_SCC_DTCNTL_TAPEN & sdhi_read(base, MMC_SCC_DTCNTL));

    sdhi_write(base, MMC_SCC_RVSCNTL, ~RCAR_SDHI_SCC_RVSCNTL_RVSEN & sdhi_read(base, MMC_SCC_RVSCNTL));

    sdhi_write(base, MMC_SD_CLK_CTRL, sdhi_read(base, MMC_SD_CLK_CTRL) | SDH_CLKCTRL_SCLKEN);  //Start
}

static unsigned rcar_sdmmc_read_bus_width(sdmmc_t *sdmmc)
{
    unsigned    base = sdmmc->hc->sdmmc_pbase;
    uint32_t    mmc_sd_option;

    mmc_sd_option = sdhi_read(base,MMC_SD_OPTION);

    if (mmc_sd_option & SDH_OPTION_WIDTH_8) {
        return 8;
    } else if ((mmc_sd_option & (SDH_OPTION_WIDTH_8)) == 0 &&
               (mmc_sd_option & (SDH_OPTION_WIDTH_1)) == 0 ) {
        return 4;
    } else {
        return 1;
    }
}

static int taps_cmp(const uint8_t *p1, const uint8_t *p2, size_t len)
{
    int i;
    for(i=0; i<len; i++)
    {
        if(*(p1 +i ) != *(p2 +i )) return(1);
    }
    return(0);
}

static int rcar_gen3_tune(sdmmc_t *sdmmc, int op)
{
    unsigned        rsp[4];
    uint8_t         td[TUNING_8BIT_BLK_SIZE];
    const uint8_t   *tuning_block_pattern;
    int             tap_num;
    int             taps[RCAR_SDHI_TUNING_RETRIES];
    int             smpcmp[RCAR_SDHI_TUNING_RETRIES];
    int             tlen;
    int             status;
    int             i;

    tap_num = rcar_sdmmc_init_tuning(sdmmc);

    if (rcar_sdmmc_read_bus_width(sdmmc) == 8) {
        tlen = TUNING_8BIT_BLK_SIZE;
        tuning_block_pattern = sdio_tbp_8bit;
    } else {
        tlen = TUNING_4BIT_BLK_SIZE;
        tuning_block_pattern = sdio_tbp_4bit;
    }

    memset(taps, 0, RCAR_SDHI_TUNING_RETRIES);
    memset(smpcmp, 0, RCAR_SDHI_TUNING_RETRIES);

    /* Issue CMD19/CMD21 twice for each tap */
    for (i = 0; i < 2 * tap_num; i++) {
        /* clear tuning data buffer to avoid comparing old data after unsuccessful transfer */
        memset(td, 0, tlen);

        rcar_sdmmc_prepare_tuning(sdmmc, i % tap_num );

        /* setup tuning command */
        CMD_CREATE (sdmmc->cmd, MMC_SEND_TUNING_BLOCK, 0, rsp, TUNING_8BIT_BLK_SIZE, 1, td);
        /* execute tuning command */
        if( 0 != (status = rcar_gen3_pio_read(sdmmc, td, TUNING_8BIT_BLK_SIZE))) {
            if (sdmmc->verbose > SDMMC_VERBOSE_LVL_0) {
                ser_putstr("tuning rcar_gen3_pio_read error\n");
            }
        } else {
            /* use to determine largest timing window where data transfer is working */
            if (!(taps_cmp(td, tuning_block_pattern, tlen))) {
                taps[i] = 1;
            }
        }

        if (!rcar_sdmmc_compare_scc_data(sdmmc)) {
            smpcmp[i] = 1;
        } else {
            smpcmp[i] = 0;
        }

        rcar_gen3_usec_delay(100);
    }
    status = rcar_sdmmc_select_tuning(sdmmc, tap_num, taps, smpcmp);

    if (status != EOK) {
        rcar_sdmmc_scc_reset(sdmmc);
    }

    return (status);
}

/*--------------------------------------*/
/* R-Car Gen 3 Host Controller          */
/*--------------------------------------*/
void rcar_gen3_sdmmc_init_hc(sdmmc_t *sdmmc, unsigned base, unsigned clock, int verbose) {
    rcar_gen3_hc.set_clk        = rcar_gen3_set_frq;
    rcar_gen3_hc.set_bus_width  = rcar_gen3_set_bus_width;
    rcar_gen3_hc.set_bus_mode   = rcar_gen3_set_bus_mode;
    rcar_gen3_hc.set_timing     = rcar_gen3_set_timing;
    rcar_gen3_hc.send_cmd       = rcar_gen3_send_cmd;
    rcar_gen3_hc.pio_read       = rcar_gen3_pio_read;
    rcar_gen3_hc.init_hc        = rcar_gen3_init_ctrl;
    rcar_gen3_hc.dinit_hc       = rcar_gen3_sdmmc_fini;
    rcar_gen3_hc.signal_voltage = rcar_gen3_signal_voltage;
    rcar_gen3_hc.tuning         = rcar_gen3_tune;
    rcar_gen3_hc.sdmmc_pbase    = base;
    rcar_gen3_hc.clock          = clock;

    sdmmc->hc            = &rcar_gen3_hc;
    sdmmc->verbose       = verbose;
    sdmmc->hc->max_blks  = rcar_gen3_dma_mxblks;
    sdmmc->caps          = 0;
}

void rcar_gen3_sdmmc_init_bs(rcar_gen3_sdmmc_bs_t *bs) {
    rcar_gen3_bs                = bs;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/lib/aarch64/rcar_gen3_sdmmc.c $ $Rev: 887585 $")
#endif
