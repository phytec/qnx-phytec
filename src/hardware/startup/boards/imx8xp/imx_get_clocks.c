/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
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
#include "imx_startup.h"
#include <hw/nxp/imx8/sci/sci.h>

/**
 * Return Cortex-A35 clock frequency.
 *
 * @param ipc      IPC handle.
 * @param resource CPU core index.
 *
 * @return         Cortex-A35 clock frequency (in herz).
 */
uint32_t imx_get_cpu_clk(sc_ipc_t ipc, sc_rsrc_t resource)
{
    sc_pm_clock_rate_t rate;

    (void)sc_pm_get_clock_rate(ipc, resource, SC_PM_CLK_CPU, &rate);

    return (uint32_t)rate;
}

/**
 * Print chip information and input clock module speed.
 *
 * @param startup_data  Pointer to the startup data.
 */
void imx_dump_clocks(imx_startup_data_t * startup_data)
{
    sc_pm_clock_rate_t rate;
    sc_pm_power_mode_t mode;

    if (debug_flag) {
        kprintf("\n");
        print_chip_info(startup_data->ipc);
        kprintf("\n");
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_A35, SC_PM_CLK_CPU, &rate);
        kprintf("Cortex-A35    : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        if (sc_rm_is_resource_owned(startup_data->ipc, SC_R_GPU_0_PID0) != 0) {
             (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_GPU_0_PID0, SC_PM_CLK_PER, &rate);
             kprintf("GPU clock     : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
             (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_GPU_0_PID0, SC_PM_CLK_MISC, &rate);
             kprintf("Shader clock  : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        }
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_DRC_0, SC_PM_CLK_MISC0, &rate);
        kprintf("DDR speed     : %dkHz\n", SAFE_DIVIDE((rate * 2), ONE_KHZ));
        kprintf("UART0 clock   : %dkHz\n", SAFE_DIVIDE(startup_data->imx_uart_clock[0], ONE_KHZ));
        kprintf("UART1 clock   : %dkHz\n", SAFE_DIVIDE(startup_data->imx_uart_clock[1], ONE_KHZ));
        kprintf("UART2 clock   : %dkHz\n", SAFE_DIVIDE(startup_data->imx_uart_clock[2], ONE_KHZ));
        kprintf("UART3 clock   : %dkHz\n", SAFE_DIVIDE(startup_data->imx_uart_clock[3], ONE_KHZ));
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_GPT_0, SC_PM_CLK_PER, &rate);
        kprintf("GPT1 clock    : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_FSPI_0 , SC_PM_CLK_PER, &rate);
        kprintf("FlexSPI0 clock: %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_SDHC_0, SC_PM_CLK_PER, &rate);
        (void)sc_pm_get_resource_power_mode(startup_data->ipc, SC_R_SDHC_0, &mode);
        kprintf("USDHC0 clock  : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        kprintf("USDHC0 power  : %d\n", mode);
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_SDHC_1, SC_PM_CLK_PER, &rate);
        (void)sc_pm_get_resource_power_mode(startup_data->ipc, SC_R_SDHC_1, &mode);
        kprintf("USDHC1 clock  : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        kprintf("USDHC1 power  : %d\n", mode);
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_SDHC_2, SC_PM_CLK_PER, &rate);
        (void)sc_pm_get_resource_power_mode(startup_data->ipc, SC_R_SDHC_2, &mode);
        kprintf("USDHC2 clock  : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        kprintf("USDHC2 power  : %d\n", mode);
        kprintf("LPSPI0 clock  : %dkHz\n", SAFE_DIVIDE(startup_data->imx_spi_clk[0], ONE_KHZ));
        kprintf("LPSPI1 clock  : %dkHz\n", SAFE_DIVIDE(startup_data->imx_spi_clk[1], ONE_KHZ));
        kprintf("LPSPI2 clock  : %dkHz\n", SAFE_DIVIDE(startup_data->imx_spi_clk[2], ONE_KHZ));
        kprintf("LPSPI3 clock  : %dkHz\n", SAFE_DIVIDE(startup_data->imx_spi_clk[3], ONE_KHZ));
        /* FlexCAN0,1,2 shares the same peripheral clock */
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_CAN_0, SC_PM_CLK_PER, &rate);
        kprintf("FLEXCAN0 clock : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_CAN_0, SC_PM_CLK_PER, &rate);
        kprintf("FLEXCAN1 clock : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        (void)sc_pm_get_clock_rate(startup_data->ipc, SC_R_CAN_0, SC_PM_CLK_PER, &rate);
        kprintf("FLEXCAN2 clock : %dkHz\n", SAFE_DIVIDE(rate, ONE_KHZ));
        kprintf("\n");
    }
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_get_clocks.c $ $Rev: 886382 $")
#endif
