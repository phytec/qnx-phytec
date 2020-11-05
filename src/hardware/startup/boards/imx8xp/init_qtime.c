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
#include "board.h"
#include <aarch64/gic_v3.h>
#include <aarch64/mx8xp.h>

#define ARM_CNTV_IRQ            27


static const struct callout_slot    timer_callouts[] = {
    { CALLOUT_SLOT(timer_load, _armv8) },
    { CALLOUT_SLOT(timer_value, _armv8) },
    { CALLOUT_SLOT(timer_reload, _armv8) },
};

/**
 * Initialize timer and specify timer timing, interrupt vector properties,
 * start timer.
 */
void init_qtime(void)
{
    struct qtime_entry    *qtime;
    uint64_t    mpidr;
    uint32_t    cpu_aff, gicr_aff;
    uint32_t    gic_core_gicr = IMX_GIC_GICR_BASE;

    qtime = alloc_qtime();

    /*
     * Set up qtime timer rate/scale
     */
    if (timer_freq == 0) {
#if defined(IMX_CNTV_CLOCK_FREQ)
        timer_freq = IMX_CNTV_CLOCK_FREQ;
#else
        /* Get input CNTV frequency from cntfrq register */
        __asm__ __volatile__("mrs    %0, cntfrq_el0" : "=r"(timer_freq));
#endif
    }
    invert_timer_freq(qtime, timer_freq);


    /* Enable PPI CNTV interrupt in GIC redistributor */
    /* Get core affinity */
    __asm__ __volatile__("mrs    %0, MPIDR_EL1" : "=r"(mpidr));
    cpu_aff = (uint32_t)((mpidr & 0xFFF) | ((mpidr >> 8) & 0xFF000000));

    do {
        /* Get GICR affinity bits (offset + 4, [32 - 63] bits of TYPER register) */
        gicr_aff = in32(gic_core_gicr + ARM_GICR_TYPER + 4);
        if (cpu_aff == gicr_aff) {
            break;
        }
        gic_core_gicr += ARM_GICR_SIZE_PER_CORE;
    } while (1);
    /* SGI base address */
    gic_core_gicr += ARM_GICR_SGI_BASE_OFFSET;
    /* Enable PPI CNTV interrupt */
    out32(gic_core_gicr + ARM_GICR_ISENABLER0, (in32(gic_core_gicr + ARM_GICR_ISENABLER0) |
                                                (0x01 << ARM_CNTV_IRQ)));

    /*
     * Each core has its own generic timer registers with the timer interrupt
     * routed to GIC PPI #27, so we force the system timer to cpu0's timer.
     */
    qtime->flags |= QTIME_FLAG_TIMER_ON_CPU0;
    qtime->cycles_per_sec = timer_freq;
    qtime->intr = ARM_CNTV_IRQ;

    /* Disable CNTV interrupt */
    __asm__ __volatile__("msr    cntv_ctl_el0, %0" : : "r"(0));

    add_callout_array(timer_callouts, sizeof(timer_callouts));
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/init_qtime.c $ $Rev: 850159 $")
#endif
