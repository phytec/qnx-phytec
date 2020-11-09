/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
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
#include <aarch64/imx8_common/imx_lpuart.h>

/* LPUART function prototype */
static uint8_t imx_lpuart_pollkey(void);
static uint8_t imx_lpuart_getchar(void);
static void    imx_lpuart_putchar(uint8_t data);

static const ser_dev imx_dev = {
    .get_byte = imx_lpuart_getchar,
    .put_byte = imx_lpuart_putchar,
    .poll = imx_lpuart_pollkey
};

static uintptr_t imx_lpuart_base;

/**
 *  Initializes LPUART device, enable transmitter and receiver.
 *
 * @param port    Base address of initialized LPUART device.
 * @param baud    Baud rate.
 * @param clk     LPUART input module clock.
 * @param osr     LPUART receiver oversampling ratio (OSR 4-32).
 */
void imx_init_lpuart(uintptr_t port, uint32_t baud, uint32_t clk, uint32_t osr)
{
    uint32_t reg32;

    /* Wait for UART to finish transmitting */
    while (!(in32(port + IMX_LPUART_STAT) & IMX_LPUART_STAT_TDRE_MASK)) {}

    /* Disable LPUART receiver & transmitter */
    reg32 = in32(port + IMX_LPUART_CTRL) & (~(IMX_LPUART_CTRL_RE_MASK | IMX_LPUART_CTRL_TE_MASK));
    out32(port + IMX_LPUART_CTRL, reg32);

    /* Set to default POR state */
    out32(port + IMX_LPUART_GLOBAL, IMX_LPUART_GLOBAL_RST_MASK);
    while ((in32(port + IMX_LPUART_GLOBAL) & IMX_LPUART_GLOBAL_RST_MASK) == 0) {}
    out32(port + IMX_LPUART_GLOBAL, 0x00);
    while ((in32(port + IMX_LPUART_GLOBAL) & IMX_LPUART_GLOBAL_RST_MASK) != 0) {}

    /* Set to 8N1, no parity */
    reg32 = in32(port + IMX_LPUART_CTRL);
    reg32 &= ~(IMX_LPUART_CTRL_M_MASK | IMX_LPUART_CTRL_PE_MASK | IMX_LPUART_CTRL_PT_MASK);
    out32(port + IMX_LPUART_CTRL, reg32);

    /* Set LPUART BaudRate */
    reg32 = (in32(port + IMX_LPUART_BAUD) & ~(IMX_LPUART_BAUD_OSR_MASK | IMX_LPUART_BAUD_SBR_MASK));
    /* SBR = (LPUART clock / baud rate) / (OSR + 1) */
    reg32 |= (((osr - 1) << IMX_LPUART_BAUD_OSR_SHIFT) |
              ((clk / (baud * osr)) & IMX_LPUART_BAUD_SBR_MASK));
    out32(port + IMX_LPUART_BAUD, reg32);

    /* Clear status flags */
    reg32 = in32(port + IMX_LPUART_STAT) |
            (IMX_LPUART_STAT_LBKDIF_MASK |
             IMX_LPUART_STAT_RXEDGIF_MASK |
             IMX_LPUART_STAT_IDLE_MASK |
             IMX_LPUART_STAT_OR_MASK |
             IMX_LPUART_STAT_NF_MASK |
             IMX_LPUART_STAT_FE_MASK |
             IMX_LPUART_STAT_PF_MASK |
             IMX_LPUART_STAT_MA1F_MASK |
             IMX_LPUART_STAT_MA2F_MASK);
    out32(port + IMX_LPUART_STAT, reg32);

    /* Enable LPUART receiver & transmitter */
    reg32 = in32(port + IMX_LPUART_CTRL);
    reg32 |= (IMX_LPUART_CTRL_RE_MASK | IMX_LPUART_CTRL_TE_MASK);
    out32(port + IMX_LPUART_CTRL, reg32);

    imx_lpuart_base = port;

    /* Register our debug functions */
    init_serdev((ser_dev *)&imx_dev);
}

/**
 * Indicates that at least 1 character is received and
 * written to the Rx FIFO
 *
 * @return 1 - Receive data ready; 0 - No receive data ready.
 */
static uint8_t imx_lpuart_pollkey(void)
{
    if (in32(imx_lpuart_base + IMX_LPUART_STAT) & IMX_LPUART_STAT_RDRF_MASK) {
        return 1;
    } else {
        return 0;
    }
}

/**
 * Wait for a receive data ready, read char from Rx FIFO.
 *
 * @return Received char.
 */
static uint8_t imx_lpuart_getchar(void)
{
    while (imx_lpuart_pollkey() == 0) {}
    return ((uint8_t)(in32(imx_lpuart_base + IMX_LPUART_DATA) & IMX_LPUART_DATA_RT_MASK));
}

/**
 * Write char to the UART Tx FIFO.
 *
 * @param data1 Data char to write.
 */
static void imx_lpuart_putchar(uint8_t data)
{
    while (!(in32(imx_lpuart_base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TDRE_MASK)) {}
    out32(imx_lpuart_base + IMX_LPUART_DATA, (uint32_t) data);
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/boards/imx8qxp-cpu/imx_lpuart.c $ $Rev: 869196 $")
#endif
