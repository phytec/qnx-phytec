/*
 * $QNXLicenseC:
 * Copyright 2015,2017, QNX Software Systems.
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
#include <aarch64/r-car-gen3.h>

/* serscif code adapted from http://community.qnx.com/sf/frs/do/viewRelease/projects.renesas_drivers/frs.renesas_r_car_h2_archives_and_do.renesas_r_car_m3_reference_bsp_0 */

#define RCAR_GEN3_SCIF_SCSCR_CKE_IN             (2 << 0)
#define SCBRR_VALUE(bps, clk)                   (clk / bps / 32 - 1) /* Internal Clock */

static unsigned char    serscif_pollkey();
static unsigned char    serscif_getchar();
static void             serscif_putchar(unsigned char);

static const ser_dev scif_dev = {
    serscif_getchar,
    serscif_putchar,
    serscif_pollkey
};

static unsigned scif_base;

static void my_delay(int dly) {
    volatile int i = dly;

    while (i--)
        ;
}

static void serscif_putchar(unsigned char c) {
    while (!(in16(scif_base + RCAR_GEN3_SCIF_SCFSR) & (RCAR_GEN3_SCIF_SCFSR_TDFE | RCAR_GEN3_SCIF_SCFSR_TEND)))
        ;
	out8(scif_base + RCAR_GEN3_SCIF_SCFTDR, c);
	out16(scif_base + RCAR_GEN3_SCIF_SCFSR, in16(scif_base + RCAR_GEN3_SCIF_SCFSR) & ~(RCAR_GEN3_SCIF_SCFSR_TDFE | RCAR_GEN3_SCIF_SCFSR_TEND));
}

static unsigned char serscif_pollkey(void) {
    if (in16(scif_base + RCAR_GEN3_SCIF_SCFSR) & (RCAR_GEN3_SCIF_SCFSR_ER | RCAR_GEN3_SCIF_SCFSR_BRK))
        out16(scif_base + RCAR_GEN3_SCIF_SCFSR, in32(scif_base + RCAR_GEN3_SCIF_SCFSR) & ~(RCAR_GEN3_SCIF_SCFSR_ER | RCAR_GEN3_SCIF_SCFSR_BRK));

    if (RCAR_GEN3_SCIF_SCFDR_RX(in16(scif_base + RCAR_GEN3_SCIF_SCFDR)))
        return 1;
    else
        return 0;
}

static unsigned char serscif_getchar(void) {
    unsigned char c;

    while (serscif_pollkey() == 0)
        ;

    c = in8(scif_base + RCAR_GEN3_SCIF_SCFRDR);

    out16(scif_base + RCAR_GEN3_SCIF_SCFSR, in32(scif_base + RCAR_GEN3_SCIF_SCFSR) & ~(RCAR_GEN3_SCIF_SCFSR_DR | RCAR_GEN3_SCIF_SCFSR_RDF));

    return c;
}

void init_serscif(unsigned address, unsigned baud, unsigned clk, unsigned extclk) {
    unsigned scscr = 0;

    in16(address + RCAR_GEN3_SCIF_SCLSR);             /* dummy read */
    out16(address + RCAR_GEN3_SCIF_SCLSR, 0x0000);    /* clear ORER bit */
    out16(address + RCAR_GEN3_SCIF_SCFSR, 0x0000);    /* clear all error bit */

    out16(address + RCAR_GEN3_SCIF_SCSCR, 0x0000);    /* clear SCR.TE & SCR.RE*/
    out16(address + RCAR_GEN3_SCIF_SCFCR, RCAR_GEN3_SCIF_SCFCR_TFRST | RCAR_GEN3_SCIF_SCFCR_RFRST);
    out16(address + RCAR_GEN3_SCIF_SCSMR, 0x0000);    /* 8bit data, no-parity, 1 stop, Po/1 */

    if (!extclk) {
        out16(address + RCAR_GEN3_SCIF_SCBRR, SCBRR_VALUE(baud, clk));
        scscr = RCAR_GEN3_SCIF_SCSCR_TE | RCAR_GEN3_SCIF_SCSCR_RE; /* enable TE, RE; SC_CLK=internal */
    } else {
        my_delay(100);
        out16(address + RCAR_GEN3_SCIF_BRG_DL, clk / (16 * baud));
        out16(address + RCAR_GEN3_SCIF_BRG_CKS, 0x0000);      /* select scif_clk	 */
        scscr = RCAR_GEN3_SCIF_SCSCR_TE | RCAR_GEN3_SCIF_SCSCR_RE | RCAR_GEN3_SCIF_SCSCR_CKE_IN;
    }

    my_delay(100);
    out16(address + RCAR_GEN3_SCIF_SCFCR, 0x0000);    /* reset-off tx-fifo, rx-fifo. */
    out16(address + RCAR_GEN3_SCIF_SCSCR, scscr);
    my_delay(100);
    scif_base = address;

    /*
     * Register our debug functions
     */
    init_serdev((ser_dev *)&scif_dev);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/lib/aarch64/serscif.c $ $Rev: 841337 $")
#endif
