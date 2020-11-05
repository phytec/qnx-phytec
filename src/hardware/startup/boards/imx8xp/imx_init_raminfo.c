/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * Copyright 2018, QNX Software Systems.
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

/**
 * i.MX startup source file.
 *
 * @file       imx_init_raminfo.c
 * @addtogroup startup
 * @{
 */

enum mem_layout_elems{
    START_ADDR = 0,
    END_ADDR,
    POPULATE
};

/* Memory layout table below 4G
 * Warning: two SYSRAM regions must not be neighbors if
 * Start addr and End addr are not specified.
 * Last region must have end of ram specified ever.
 */
const uint64_t mem_layout_below4G[][3] =
{
/*  { Start addr, End addr  , populate } */
    { IMX_SDRAM0_BASE, 0x800FFFFF, 0},  /* IPL_BASE */
#if IMX_VPU_INIT_ENABLED
    { 0x80100000, 0x820FFFFF, 0},  /* DECODER_BOOT */
    { 0x82100000, 0x822FFFFF, 0},  /* ENCODER_BOOT */
    { 0x00000000, 0x00000000, 1},  /* SYSRAM */
    { 0x8D000000, 0x8D1FFFFF, 0},  /* DECODER_RPC */
    { 0x8D200000, 0x8D3FFFFF, 0},  /* ENCODER_RPC */
    { 0x8D400000, 0x8DBFFFFF, 0},  /* ENCODER_RSRVD */
    { 0x8DC00000, 0x8DFFFFFF, 0},  /* FILLING */
#endif
#if IMX_DSP_INIT_ENABLED
    { 0x00000000, 0x00000000, 1},  /* SYSRAM */
    { 0x8E000000, 0x9000FFFF, 0},  /* DSP */
#endif
    { 0x00000000, IMX_SDRAM0_BASE + MEG(IMX_SDRAM0_SIZE) - 1, 1}  /* SYSRAM */
};

#if defined(IMX_SDRAM1_SIZE)
/* Memory layout table above 4G
 * Warning: two SYSRAM regions must not be neighbors if
 * Start addr and End addr are not specified.
 * Last region must have end of ram specified ever.
 */
const uint64_t mem_layout_above4G[][3] =
{
/*  { Start addr, End addr, populate } */
    { IMX_SDRAM1_BASE, IMX_SDRAM1_BASE + MEG(IMX_SDRAM1_SIZE) - 1, 1}  /* SYSRAM */
};
#endif

/**
 * Add sysram memory regions based on memory layout table.
 *
 * @param mem_layout  Memory layout table
 * @param layout_size sizeof Memory layout table
 */
void add_ram_regions(const uint64_t mem_layout[][3], size_t layout_size){

    int      region;
    int      regions = layout_size / 3 / sizeof(uint64_t);
    uint64_t curr_addr = mem_layout[0][START_ADDR];
    uint64_t size;

    /* For each memory region */
    for (region = 0; region < regions; region++) {
        if (((mem_layout[region][END_ADDR]) - mem_layout[region][START_ADDR]) == 0) {
            /* We don't know size of region! => Calculate size from start address of next element. */
            size = mem_layout[region + 1][START_ADDR] - curr_addr;
        } else {
            /* We know region size from mem_layout table */
            /* Use region size from mem_layout table */
            size = (mem_layout[region][END_ADDR] + 1) - curr_addr;
        }

        /* Populate memory region */
        if ((mem_layout[region][POPULATE]) && (size != 0)) {
            add_ram(curr_addr, size);
        }
        /* Update next region start address */
        curr_addr += size;
    }
}

/**
 * Add RAM area information to the system page.
 */
void imx_init_raminfo(void)
{
   add_ram_regions(mem_layout_below4G, sizeof(mem_layout_below4G));

#if defined(IMX_SDRAM1_SIZE)
   add_ram_regions(mem_layout_above4G, sizeof(mem_layout_above4G));
#endif
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_init_raminfo.c $ $Rev: 893864 $")
#endif
