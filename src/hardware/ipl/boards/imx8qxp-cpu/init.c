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

#include <sys/srcversion.h>
#include <hw/inout.h>
#include <hw/nxp/imx8/sci/sci.h>
#include <hw/nxp/imx8/sci/imx8qxp_pads.h>
#include <aarch64/mx8xp.h>
#include "ipl.h"
#include "imx_ipl.h"
#include "board.h"

#ifndef NUM_ELTS
#define NUM_ELTS(__ary) ((unsigned)(sizeof(__ary)/sizeof(*(__ary))))
#endif

/* Definition of PADS alternate functions */
#define IMX_PAD_ALT_0   0   /* PAD alternate function 0 */
#define IMX_PAD_ALT_1   1   /* PAD alternate function 1 */
#define IMX_PAD_ALT_2   2   /* PAD alternate function 2 */
#define IMX_PAD_ALT_3   3   /* PAD alternate function 3 */

/* Structure encapsulating pin and mux configuration */
typedef struct {
    sc_pad_t             pad;     /* PAD name */
    uint8_t              mux;     /* PAD mux (pad alternate function) */
    sc_pad_config_t      config;  /* PAD config */
    sc_pad_28fdsoi_dse_t dse;     /* PAD drive strength */
    sc_pad_28fdsoi_ps_t  ps;      /* PAD pull select */
} imx_pad_t;

/* Function prototypes */
static void imx_init_usdhc0(sc_ipc_t ipc);
static void imx_init_usdhc1(sc_ipc_t ipc);
#ifdef IMX_QSPI_BOOT
    static void imx_init_fspi(sc_ipc_t ipc);
#endif
extern uint32_t chip_type;

/* Code performed if SCI error occurs */
static void sci_error_occurred(void) {
    /* coverity[no_escape] - Suppress coverity INFINITE_LOOP error */
    while (1) {}
}

/**
 * Print error information about unsuccessful power up.
 *
 * @param resource   Resource number to print.
 * @param sc_status  SC status information to print.
 */
static void imx_print_power_up_fail(sc_rsrc_t resource, sc_err_t sc_status)
{
    ser_putstr(sc_rsrc2str(resource));
    ser_putstr(" power up failed. SC status: ");
    ser_putstr(sc_status2str(sc_status));
    ser_putstr("\n");
}

/**
 * Print error information about unsuccessful clock setup.
 *
 * @param resource  Resource number to print.
 * @param freq      Frequency in Hz to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_set_clock_fail(sc_rsrc_t resource, uint32_t freq, sc_err_t sc_status)
{
    ser_putstr(sc_rsrc2str(resource));
    ser_putstr(" set clock root to ");
    ser_putdec(freq);
    ser_putstr(" Hz failed. SC status: ");
    ser_putstr(sc_status2str(sc_status));
    ser_putstr("\n");
}

/**
 * Print error information about unsuccessful clock enable.
 *
 * @param resource  Resource number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_clock_enable_fail(sc_rsrc_t resource, sc_err_t sc_status)
{
    ser_putstr(sc_rsrc2str(resource));
    ser_putstr(" enable of clock root failed. SC status: ");
    ser_putstr(sc_status2str(sc_status));
    ser_putstr("\n");
}

/**
 * Print error information about unsuccessful clock disable.
 *
 * @param resource  Resource number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_clock_disable_fail(sc_rsrc_t resource, sc_err_t sc_status)
{
    ser_putstr(sc_rsrc2str(resource));
    ser_putstr(" disable of clock root failed. SC status: ");
    ser_putstr(sc_status2str(sc_status));
    ser_putstr("\n");
}

/**
 * Print error information about unsuccessful setup of mux.
 *
 * @param pad       Pad number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_set_mux_fail(sc_pad_t pad, sc_err_t sc_status)
{
    ser_putstr(" sc_pad_set_mux failed for pad ");
    ser_putdec(pad);
    ser_putstr(". SC status: ");
    ser_putstr(sc_status2str(sc_status));
    ser_putstr("\n");
}

/**
 * Print error information about unsuccessful setup of electric features.
 *
 * @param pad       Pad number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_set_gp_fail(sc_pad_t pad, sc_err_t sc_status)
{
    ser_putstr(" sc_pad_set_gp_28fdsoi failed for pad ");
    ser_putdec(pad);
    ser_putstr(". SC status: ");
    ser_putstr(sc_status2str(sc_status));
    ser_putstr("\n");
}

#ifndef IMX_ARM_TRUSTED_FW
/**
 * Print error information about unsuccessful CPU start.
 *
 * @param resource   Resource number to print.
 * @param sc_status  SC status information to print.
 */
static void imx_print_cpu_start_fail(sc_rsrc_t resource, sc_err_t sc_status)
{
    ser_putstr(sc_rsrc2str(resource));
    ser_putstr(" cpu start failed. SC status: ");
    ser_putstr(sc_status2str(sc_status));
    ser_putstr("\n");
}
#endif

/**
 * Initialize serial console.
 *
 * @param ipc IPC handle.
 */
void imx_init_console(sc_ipc_t ipc)
{
    sc_err_t sci_err;
    /* Set UART0 clock root to 80 MHz */
    sc_pm_clock_rate_t rate = 80000000U;

    /* Initialize UART input clock */
    {
        /* Power up UART0 */
        sci_err = sc_pm_set_resource_power_mode(ipc, SC_R_UART_0, SC_PM_PW_MODE_ON);
        if (sci_err != SC_ERR_NONE) {
            sci_error_occurred();
        }
        sci_err = sc_pm_set_clock_rate(ipc, SC_R_UART_0, 2, &rate);
        if (sci_err != SC_ERR_NONE) {
            sci_error_occurred();
        }
        /* Enable UART0 clock root */
        sci_err = sc_pm_clock_enable(ipc, SC_R_UART_0, 2, true, false);
        if (sci_err != SC_ERR_NONE) {
            sci_error_occurred();
        }
    }
    /* Configure UART0 pads */
    {
        /* UART0_RX: DMA.UART0.RX, LSIO.GPIO0.IO20 = mux 0*/
        sci_err = sc_pad_set_mux(ipc, SC_P_UART0_RX, 0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF);
        sci_err |= sc_pad_set_gp_28fdsoi(ipc, SC_P_UART0_RX, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU);
        /* UART0_TX: DMA.UART0.TX, LSIO.GPIO0.IO21 = mux 0 */
        sci_err |= sc_pad_set_mux(ipc, SC_P_UART0_TX, 0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF);
        sci_err |= sc_pad_set_gp_28fdsoi(ipc, SC_P_UART0_TX, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE);
        if (sci_err != SC_ERR_NONE) {
            sci_error_occurred();
        }
    }
    /*
     * Initialize serial interface,
     * oversampling ratio = 7 */
    imx_init_lpuart(IMX_CONSOLE_UART_BASE, IMX_CONSOLE_UART_BAUD_RATE, rate, 7U);
}

/**
 * Enable clock gate and configure clock path for devices used by IPL.
 *
 * @param ipc IPC handle.
 */
void imx_init_clocks(sc_ipc_t ipc)
{
    sc_err_t sci_err;
    sc_pm_clock_rate_t rate;

#ifdef IMX_QSPI_BOOT
    /* ********************************* FLEXSPI ********************************* */
    sci_err = sc_pm_set_resource_power_mode(ipc, SC_R_FSPI_0, SC_PM_PW_MODE_ON);
    if (sci_err != SC_ERR_NONE) {
        imx_print_power_up_fail(SC_R_FSPI_0, sci_err);
    }
    /* Set FlexSPI0 clock root to 166 MHz */
    rate = 166000000U;
    sci_err = sc_pm_set_clock_rate(ipc, SC_R_FSPI_0, SC_PM_CLK_PER, &rate);
    if (sci_err != SC_ERR_NONE) {
        imx_print_set_clock_fail(SC_R_FSPI_0, rate, sci_err);
    }
    /* Enable FlexSPI0 peripheral clock */
    sci_err = sc_pm_clock_enable(ipc, SC_R_FSPI_0, SC_PM_CLK_PER, true, false);
    if (sci_err != SC_ERR_NONE) {
        imx_print_clock_enable_fail(SC_R_FSPI_0, sci_err);
    }
#endif
    /* ********************************* uSDHC0 ********************************* */
    /* Power up uSDHC0 */
    sci_err = sc_pm_set_resource_power_mode(ipc, SC_R_SDHC_0, SC_PM_PW_MODE_ON);
    if (sci_err != SC_ERR_NONE) {
        imx_print_power_up_fail(SC_R_SDHC_0, sci_err);
    }
    /* Disable uSDHC0 peripheral clock */
    sci_err = sc_pm_clock_enable(ipc, SC_R_SDHC_0, SC_PM_CLK_PER, false, false);
    if (sci_err != SC_ERR_NONE) {
        imx_print_clock_disable_fail(SC_R_SDHC_0, sci_err);
    }
    /* Set uSDHC0 clock root to 400 MHz */
    rate = 400000000;
    sci_err = sc_pm_set_clock_rate(ipc, SC_R_SDHC_0, SC_PM_CLK_PER, &rate);
    if (sci_err != SC_ERR_NONE) {
        imx_print_set_clock_fail(SC_R_SDHC_0, rate, sci_err);
    }
    /* Enable uSDHC0 peripheral clock */
    sci_err = sc_pm_clock_enable(ipc, SC_R_SDHC_0, SC_PM_CLK_PER, true, true);
    if (sci_err != SC_ERR_NONE) {
        imx_print_clock_enable_fail(SC_R_SDHC_0, sci_err);
    }
    /* ********************************* uSDHC1 ********************************* */
    /* Power up uSDHC1 */
    sci_err = sc_pm_set_resource_power_mode(ipc, SC_R_SDHC_1, SC_PM_PW_MODE_ON);
    if (sci_err != SC_ERR_NONE) {
        imx_print_power_up_fail(SC_R_SDHC_1, sci_err);
    }
    /* Disable uSDHC1 peripheral clock */
    sci_err = sc_pm_clock_enable(ipc, SC_R_SDHC_1, SC_PM_CLK_PER, false, false);
    if (sci_err != SC_ERR_NONE) {
        imx_print_clock_disable_fail(SC_R_SDHC_1, sci_err);
    }
    /* Set uSDHC1 clock root to 198 MHz */
    rate = 198000000;
    sci_err = sc_pm_set_clock_rate(ipc, SC_R_SDHC_1, SC_PM_CLK_PER, &rate);
    if (sci_err != SC_ERR_NONE) {
        imx_print_set_clock_fail(SC_R_SDHC_1, rate, sci_err);
    }
    /* Enable uSDHC0 peripheral clock */
    sci_err = sc_pm_clock_enable(ipc, SC_R_SDHC_1, SC_PM_CLK_PER, true, false);
    if (sci_err != SC_ERR_NONE) {
        imx_print_clock_enable_fail(SC_R_SDHC_1, sci_err);
    }
}

/**
 * Initialize UART0, SD0(1 - eMMC), SD1(2 - SD card) PINs routing and electric PIN properties.
 *
 * @param ipc IPC handle.
 */
void imx_init_pinmux(sc_ipc_t ipc)
{
#ifdef IMX_QSPI_BOOT
    imx_init_fspi(ipc);
#endif
    imx_init_usdhc0(ipc);
    imx_init_usdhc1(ipc);
}

#ifdef IMX_QSPI_BOOT
/**
 * Initialize FlexSPI electric properties of PINs for NOR device.
 *
 * @param ipc IPC handle.
 */
static void imx_init_fspi(sc_ipc_t ipc)
{
    sc_err_t sci_err;
    uint8_t i;

    imx_pad_t pads[] = {
            {SC_P_QSPI0A_DATA0, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0A_DATA1, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0A_DATA2, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0A_DATA3, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0A_DQS,   IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0A_SS0_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0A_SCLK,  IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0B_SCLK,  IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0B_DATA0, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0B_DATA1, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0B_DATA2, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
            {SC_P_QSPI0B_DATA3, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
    };

    for (i = 0; i < NUM_ELTS(pads); i++) {
        sci_err = sc_pad_set_mux(ipc, pads[i].pad, pads[i].mux, pads[i].config, SC_PAD_ISO_OFF);
        if (sci_err != SC_ERR_NONE) {
            imx_print_set_mux_fail(pads[i].pad, sci_err);
        }
        sci_err = sc_pad_set_gp_28fdsoi(ipc, pads[i].pad, pads[i].dse, pads[i].ps);
        if (sci_err != SC_ERR_NONE) {
            imx_print_set_gp_fail(pads[i].pad, sci_err);
        }
    }
}
#endif

/**
 * Initialize uSDHC0 electric properties of PINs for eMMC device.
 *
 * @param ipc IPC handle.
 */
static void imx_init_usdhc0(sc_ipc_t ipc)
{
    sc_err_t sci_err;
    uint8_t i;

    imx_pad_t pads[] = {
        /* Pad configuration: pad, mux, config, dse, ps */
        {SC_P_EMMC0_CLK,     IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* CLK */
        {SC_P_EMMC0_CMD,     IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* CMD */
        {SC_P_EMMC0_DATA0,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* DATA0 */
        {SC_P_EMMC0_DATA1,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* DATA1 */
        {SC_P_EMMC0_DATA2,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* DATA2 */
        {SC_P_EMMC0_DATA3,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* DATA3 */
        {SC_P_EMMC0_DATA4,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* DATA4 */
        {SC_P_EMMC0_DATA5,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* DATA5 */
        {SC_P_EMMC0_DATA6,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* DATA6 */
        {SC_P_EMMC0_DATA7,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* DATA7 */
        {SC_P_EMMC0_STROBE,  IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* STROBE */
        {SC_P_EMMC0_RESET_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_LOW,  SC_PAD_28FDSOI_PS_NONE}  /* RESET_B */
    };

    for (i = 0; i < NUM_ELTS(pads); i++) {
        sci_err = sc_pad_set_mux(ipc, pads[i].pad, pads[i].mux, pads[i].config, SC_PAD_ISO_OFF);
        if (sci_err != SC_ERR_NONE) {
            imx_print_set_mux_fail(pads[i].pad, sci_err);
        }
        sci_err = sc_pad_set_gp_28fdsoi(ipc, pads[i].pad, pads[i].dse, pads[i].ps);
        if (sci_err != SC_ERR_NONE) {
            imx_print_set_gp_fail(pads[i].pad, sci_err);
        }
    }
}

/**
 * Initialize uSDHC1 electric properties of PINs for SD card.
 *
 * @param ipc IPC handle.
 */
static void imx_init_usdhc1(sc_ipc_t ipc)
{
    sc_err_t sci_err;
    uint8_t i;

    imx_pad_t pads[] = {
        /* Pad configuration: pad, mux, config, dse, ps */
        {SC_P_USDHC1_CLK,     IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* CLK */
        {SC_P_USDHC1_CMD,     IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* CMD */
        {SC_P_USDHC1_DATA0,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* DATA0 */
        {SC_P_USDHC1_DATA1,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* DATA1 */
        {SC_P_USDHC1_DATA2,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* DATA2 */
        {SC_P_USDHC1_DATA3,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* DATA3 */
        {SC_P_USDHC1_WP,      IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_LOW,  SC_PAD_28FDSOI_PS_PU}, /* WP */
        {SC_P_USDHC1_CD_B,    IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_LOW,  SC_PAD_28FDSOI_PS_PU}, /* CD_B */
        {SC_P_USDHC1_RESET_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_LOW,  SC_PAD_28FDSOI_PS_PU}, /* RESET_B */
        {SC_P_USDHC1_VSELECT, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_28FDSOI_DSE_DV_LOW,  SC_PAD_28FDSOI_PS_PU}, /* VSELECT */
    };

    for (i = 0; i < NUM_ELTS(pads); i++) {
        sci_err = sc_pad_set_mux(ipc, pads[i].pad, pads[i].mux, pads[i].config, SC_PAD_ISO_OFF);
        if (sci_err != SC_ERR_NONE) {
            imx_print_set_mux_fail(pads[i].pad, sci_err);
        }
        sci_err = sc_pad_set_gp_28fdsoi(ipc, pads[i].pad, pads[i].dse, pads[i].ps);
        if (sci_err != SC_ERR_NONE) {
            imx_print_set_gp_fail(pads[i].pad, sci_err);
        }
    }
}

#ifndef IMX_ARM_TRUSTED_FW
/**
 * Initialize secondary cores.
 *
 * @param ipc IPC handle.
 */
void imx_init_cores(sc_ipc_t ipc)
{
    sc_rsrc_t cpu;
    sc_err_t sci_err;

    /* Start Cortex-A35 cores 1 - 2 || 4 */
    for (cpu = SC_R_A35_1; cpu <= (IS_IMX8QXP_MCU_TYPE(chip_type) ? SC_R_A35_3 : SC_R_A35_1); cpu++) {
        /* IPC to power up and boot other cores */
        if ((sci_err = sc_pm_set_resource_power_mode(ipc, cpu, SC_PM_PW_MODE_ON)) != SC_ERR_NONE) {
            imx_print_power_up_fail(cpu, sci_err);
        }
        if ((sci_err = sc_pm_cpu_start(ipc, cpu, false, 0x80000000)) != SC_ERR_NONE) {
            imx_print_cpu_start_fail(cpu, sci_err);
        }
        if ((sci_err = sc_pm_cpu_start(ipc, cpu, true, 0x80000000)) != SC_ERR_NONE) {
            imx_print_cpu_start_fail(cpu, sci_err);
        }
    }
}
#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/boards/imx8qxp-cpu/init.c $ $Rev: 893260 $")
#endif
