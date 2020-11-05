/*
 * $QNXLicenseC:
 * Copyright 2018-2019, QNX Software Systems.
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

#include <stdbool.h>
#include "startup.h"
#include "aarch64/aarch64_tlb.h"
#include "imx_startup.h"

/*
 * Enable/disable the various system controls.
 * This code is hardware dependant and may have to be changed
 * by end users.
 */

#define SDRAM_BASE_ADDR        0x80000000
#ifndef STARTUP_SDRAM_SIZE
#define STARTUP_SDRAM_SIZE    0x40000000
#endif

uint64_t aarch64_tlb[TLB_SIZE] __attribute__ ((aligned(64 * 1024)));
bool cache_always_on = true; /* It is safe to always enable cache */

aarch64_tlb_t board_tlb[] = {
    {
        .start = SDRAM_BASE_ADDR,
        .len   = STARTUP_SDRAM_SIZE,
        .attr  = (0x4 << 2),
    },
    {
        -1, -1, 0,
    },
};

void
board_mmu_enable()
{
    aarch64_setup_tlb(board_tlb, aarch64_tlb);
    aarch64_enable_mmu((uint64_t)aarch64_tlb);

    if(cache_always_on) {
        board_icache_enable();
        board_dcache_enable();
    }
}

void
board_mmu_disable()
{
    if(cache_always_on) {
        board_icache_disable();
        board_dcache_disable();
    }

    aarch64_disable_mmu();
}

void
board_alignment_check_enable()
{
    aarch64_alignment_check_enable();
}

void
board_alignment_check_disable()
{
    aarch64_alignment_check_disable();
}

void
board_dcache_enable()
{
    aarch64_dcache_enable();
}

void
board_dcache_disable()
{
    aarch64_dcache_disable();
}

void
board_icache_enable()
{
    aarch64_icache_enable();
}

void
board_icache_disable()
{
    aarch64_icache_disable();
}

void
board_enable_caches()
{
    if(!cache_always_on) {  // check cache status
        board_icache_enable();
        board_dcache_enable();
    }
}

void
board_disable_caches()
{
    if(!cache_always_on) {  // check cache status
        board_icache_disable();
        board_dcache_disable();
    }
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/qxp-cpu/board_sysctl.c $ $Rev: 884416 $")
#endif
