/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
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
#include "imx_startup.h"
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
#include "aarch64/psci.h"
#endif

/* Startup global data */
extern imx_startup_data_t startup_data;

#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
    const uint64_t imx8x_core_affinity[4] = {
            0x0000,
            0x0001,
            0x0002,
            0x0003
    };
#endif

uintptr_t   secondary_start;      /* Start address for cores waiting in cstart.S */
long        secondary_cpu;        /* CPU being woken up */

/**
 * Return CPU core number.
 *
 * @return CPU core number.
 */
unsigned board_smp_num_cpu(void)
{
    unsigned board_smp_max_cpu = (startup_data.imx8dual_type) ?
                                 IMX_DXx_MCU_CORES_NUMBER : IMX_QXP_MCU_CORES_NUMBER;

    kprintf("board_smp_num_cpu: %d cores\n", board_smp_max_cpu);
    return board_smp_max_cpu;
}

/**
 * Perform any board specific SMP initialisation.
 *
 * @param smp      Pointer to smp_entry structure.
 * @param num_cpus CPU cores number.
 */
void board_smp_init(struct smp_entry *smp, unsigned num_cpus)
{
    smp->send_ipi = (void *)&sendipi_gic_v3_sr;
}

/**
 * Initialize and start secondary CPU core.
 *
 * @param cpu   CPU core index.
 * @param start CPU reset address.
 *
 * @return  CPU core start status.
 * @retval  0   CPU core start failed.
 * @retval  1   Success, OK.
 */
int board_smp_start(unsigned cpu, void (*start)(void))
{
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
    int32_t status;
#endif

    board_mmu_disable();

    /* Secondary cores will be spinning in _start.S */
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
    status = psci_cpu_on(cpu, (uint64_t)start, 0);
    if (status != PSCI_SUCCESS) {
        crash("Cortex-A core %d start failed! Status: 0x%x. \n", cpu, status);
    }
#else
    secondary_start = (uintptr_t)start;
    secondary_cpu = cpu;

    __asm__ __volatile__(
        "dsb sy\n"
        "sev\n"
        : : : "memory"
    );
#endif
    return 1;
}

/**
 * Perform any board/cpu-specific actions required to adjust the cpu number.
 *
 * @param cpu CPU core number.
 *
 * @return    Adjusted CPU core number.
 */
unsigned board_smp_adjust_num(unsigned cpu)
{
    return cpu;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/startup/boards/imx8xp/board_smp.c $ $Rev: 887148 $")
#endif
