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
#include <aarch64/r-car-gen3.h>
#include "ipl.h"
#include "private/rcar_gen3_delay.h"

void rcar_gen3_usec_delay(uint32_t usec)
{
    /* Stop TMU0 before changing settings */
    uint8_t start_reg = in8(RCAR_GEN3_TMU_BASE + RCAR_GEN3_TMU_TSTR0);
    out8(RCAR_GEN3_TMU_BASE + RCAR_GEN3_TMU_TSTR0, start_reg & ~RCAR_GEN3_TMU_START0);

    /*
     * Input-clock for TMU0 is CP = EXTAL/2 = 16.66MHz/2
     * TPSC(RCAR_GEN3_TMU_TCR[2:0]) = 0: clock_div = 4
     * Each cycle = 1/(CP/clock_div) = 480ns
     * 2 cycles = 960ns ~= 1us
     */
    if(usec & (1 << 31)){
        ser_putstr("usec_delay: warning - invalid delay value\n");
    }
    usec <<= 1;

    out16(RCAR_GEN3_TMU0_BASE + RCAR_GEN3_TMU_TCR, RCAR_GEN3_TMU_UNIE);
    out32(RCAR_GEN3_TMU0_BASE + RCAR_GEN3_TMU_TCOR, usec);
    out32(RCAR_GEN3_TMU0_BASE + RCAR_GEN3_TMU_TCNT, usec);

    /* Start TMU0 */
    out8(RCAR_GEN3_TMU_BASE + RCAR_GEN3_TMU_TSTR0, start_reg | RCAR_GEN3_TMU_START0);

    while(!(in16(RCAR_GEN3_TMU0_BASE + RCAR_GEN3_TMU_TCR) & RCAR_GEN3_TMU_UNF));
    out8(RCAR_GEN3_TMU_BASE + RCAR_GEN3_TMU_TSTR0, start_reg & ~RCAR_GEN3_TMU_START0);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/lib/aarch64/rcar_gen3_delay.c $ $Rev: 885792 $")
#endif
