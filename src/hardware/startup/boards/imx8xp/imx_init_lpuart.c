/*
 * $QNXLicenseC:
 * Copyright 2018, QNX Software Systems.
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

#include "startup.h"
#include <aarch64/imx8_common/imx_lpuart.h>

/**
 * Parse UART initialization options (base address, input clock, baudrate).
 *
 * @param channel Debug device index (in debug_devices structure)
 * @param line    String line to parse.
 * @param baud    Pointer to baudrate variable.
 * @param clk     Pointer to input peripheral clock variable.
 */
static void parse_line(unsigned channel, const char *line, unsigned *baud, unsigned *clk)
{
    /* Get device base address and register stride */
    if ((*line != '.') && (*line != '\0')) {
        dbg_device[channel].base = strtoul(line, (char **)&line, 16);
        if (*line == '^') {
            dbg_device[channel].shift = strtoul(line + 1, (char **)&line, 0);
        }
    }

    /* Get baud rate value */
    if (*line == '.') {
        ++line;
    }
    if ((*line != '.') && (*line != '\0')) {
        *baud = strtoul(line, (char **)&line, 0);
    }

    /* Get input device clock rate value */
    if (*line == '.') {
        ++line;
    }
    if ((*line != '.') && (*line != '\0')) {
        *clk = strtoul(line, (char **)&line, 0);
    }
}


/**
 * Initialize one of the serial ports.
 *
 * @param channel  Debug device index (in debug_devices structure)
 * @param init     String line with configuration parameters.
 * @param defaults String line with configuration parameters.
 */
void imx_init_lpuart(unsigned channel, const char *init, const char *defaults)
{
    uint32_t baud, clk, reg32, osr, idx, temp_diff, calculated_baud, baud_diff;
    uint16_t sbr, sbr_temp;
    uintptr_t base;

    parse_line(channel, defaults, &baud, &clk);
    parse_line(channel, init, &baud, &clk);
    base = dbg_device[channel].base;

    if (baud == 0u) {
        return;
    }

    /* Wait for UART to finish transmitting */
    while (!(in32(base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TDRE_MASK)) {}

    /* Disable LPUART receiver & transmitter */
    reg32 = in32(base + IMX_LPUART_CTRL) & (~(IMX_LPUART_CTRL_RE_MASK | IMX_LPUART_CTRL_TE_MASK));
    out32(base + IMX_LPUART_CTRL, reg32);

    /* Set to default POR state */
    out32(base + IMX_LPUART_GLOBAL, IMX_LPUART_GLOBAL_RST_MASK);
    while ((in32(base + IMX_LPUART_GLOBAL) & IMX_LPUART_GLOBAL_RST_MASK) == 0u) {}
    out32(base + IMX_LPUART_GLOBAL, 0x00);
    while ((in32(base + IMX_LPUART_GLOBAL) & IMX_LPUART_GLOBAL_RST_MASK) != 0u) {}

    /* Set to 8N1, no parity */
    reg32 = in32(base + IMX_LPUART_CTRL);
    reg32 &= ~(IMX_LPUART_CTRL_M_MASK | IMX_LPUART_CTRL_PE_MASK | IMX_LPUART_CTRL_PT_MASK);
    out32(base + IMX_LPUART_CTRL, reg32);

    /* Calculate LPUART BaudRate register setting */
    osr = 4u;
    sbr = (clk / (baud * osr));
    calculated_baud = (clk / (osr * sbr));
    baud_diff = (calculated_baud > baud) ? (calculated_baud - baud) : (baud - calculated_baud);
    /* Loop to find the best OSR value possible */
    for (idx = 5u; idx <= 32u; idx++) {
        /* Calculate the temporary SBR value */
        sbr_temp = (clk / (baud * idx));
        /* Calculate the baud rate based on the temporary OSR and SBR values */
        calculated_baud = (clk / (idx * sbr_temp));
        temp_diff = (calculated_baud > base) ? (calculated_baud - baud) : (baud - calculated_baud);
        if (temp_diff <= baud_diff) {
            baud_diff = temp_diff;
            osr = idx;                /* Update and store the best OSR value calculated */
            sbr = sbr_temp;           /* Update store the best SBR value calculated */
        }
    }
    /* Set calculated BaudRate */
    reg32 = (in32(base + IMX_LPUART_BAUD) & ~(IMX_LPUART_BAUD_OSR_MASK | IMX_LPUART_BAUD_SBR_MASK));
    reg32 |= (((osr - 1u) << IMX_LPUART_BAUD_OSR_SHIFT) | (sbr & IMX_LPUART_BAUD_SBR_MASK));
    out32(base + IMX_LPUART_BAUD, reg32);

    /* Clear status flags */
    reg32 = (in32(base + IMX_LPUART_STAT) |
             (IMX_LPUART_STAT_LBKDIF_MASK |
              IMX_LPUART_STAT_RXEDGIF_MASK |
              IMX_LPUART_STAT_IDLE_MASK |
              IMX_LPUART_STAT_OR_MASK |
              IMX_LPUART_STAT_NF_MASK |
              IMX_LPUART_STAT_FE_MASK |
              IMX_LPUART_STAT_PF_MASK |
              IMX_LPUART_STAT_MA1F_MASK |
              IMX_LPUART_STAT_MA2F_MASK));
    out32(base + IMX_LPUART_STAT, reg32);

    /* Enable LPUART receiver & transmitter */
    reg32 = in32(base + IMX_LPUART_CTRL);
    reg32 |= (IMX_LPUART_CTRL_RE_MASK | IMX_LPUART_CTRL_TE_MASK);
    out32(base + IMX_LPUART_CTRL, reg32);
}

/**
 * Send a character over LPUART device.
 *
 * @param data Character to send.
 */
void imx_lpuart_put_char(int data)
{
    paddr_t base = dbg_device[0].base;

    while ((in32(base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TDRE_MASK) == 0) {}
    out32(base + IMX_LPUART_DATA, data);
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/startup/boards/imx8xp/imx_init_lpuart.c $ $Rev: 869196 $")
#endif
