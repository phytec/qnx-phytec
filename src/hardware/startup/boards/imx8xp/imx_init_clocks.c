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
#include <aarch64/mx8xp.h>
#include <hw/nxp/imx8/sci/sci.h>
#include "board.h"
#include "imx_acm_drv.h"

/** Structure encapsulating peripheral initialization */
typedef struct {
    sc_rsrc_t          per;    /**< Peripheral index */
    uint32_t           clk;    /**< Requested clock */
    sc_pm_power_mode_t pwr;    /**< Power mode */
} imx_peripheral_t;

void imx_init_console(imx_startup_data_t * startup_data, struct debug_device * debug_devices);

/**
 * Print error information about unsuccessful power up.
 *
 * @param func_name Pointer to a string containing name of function to print.
 * @param resource  Resource number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_power_up_fail(const char * func_name, sc_rsrc_t resource, sc_err_t sc_status)
{

    kprintf("%s: %s power up failed. SC status: %s\n", func_name, sc_rsrc2str(resource), sc_status2str(sc_status));
}

/**
 * Print error information about unsuccessful clock setup.
 *
 * @param func_name Pointer to a string containing name of function to print.
 * @param resource  Resource number to print.
 * @param freq      Frequency in Hz to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_set_clock_fail(const char * func_name, sc_rsrc_t resource, uint32_t freq, sc_err_t sc_status)
{
    kprintf("%s: %s set clock root to %u Hz failed. SC status: %s\n", func_name, sc_rsrc2str(resource), freq,
            sc_status2str(sc_status));
}

/**
 * Print error information about unsuccessful read of clock.
 *
 * @param func_name Pointer to a string containing name of function to print.
 * @param resource  Resource number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_get_clock_fail(const char * func_name, sc_rsrc_t resource, sc_err_t sc_status)
{
    kprintf("%s: %s get clock root frequency failed. SC status: %s\n", func_name, sc_rsrc2str(resource),
            sc_status2str(sc_status));
}

/**
 * Print error information about unsuccessful clock enable.
 *
 * @param func_name Pointer to a string containing name of function to print.
 * @param resource  Resource number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_clock_enable_fail(const char * func_name, sc_rsrc_t resource, sc_err_t sc_status)
{
    kprintf("%s: %s enable of clock root failed. SC status: %s\n", func_name, sc_rsrc2str(resource),
            sc_status2str(sc_status));
}

/**
 * Print error information about unsuccessful clock disable.
 *
 * @param func_name Pointer to a string containing name of function to print.
 * @param resource  Resource number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_clock_disable_fail(const char * func_name, sc_rsrc_t resource, sc_err_t sc_status)
{
    kprintf("%s: %s disable of clock root failed. SC status: %s\n", func_name, sc_rsrc2str(resource),
            sc_status2str(sc_status));
}

/**
 * Configures peripheral clock settings.
 *
 * @param ipc       IPC handle.
 * @param resource  Resource (peripheral or bus master) to be configured.
 * @param clk       Clock to be configured.
 * @param rate      Optional parameter. When not NULL, sc_pm_set_clock_rate() is called.
 * @param func_name Pointer to function name string.
 *
 * @return Execution status.
 */
static sc_err_t imx_init_periph_clock(sc_ipc_t ipc, sc_rsrc_t resource, sc_pm_clk_t clk, sc_pm_clock_rate_t * rate,
                               const char * func_name)
{
    sc_err_t sc_status = SC_ERR_NONE;
    sc_pm_clock_rate_t val;

    do {
        /* Power up */
        sc_status = sc_pm_set_resource_power_mode(ipc, resource, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(func_name, resource, sc_status);
            break;
        }
        if (rate != NULL) {
            sc_status = sc_pm_clock_enable(ipc, resource, clk, false, false);
            if (sc_status != SC_ERR_NONE) {
                imx_print_clock_disable_fail(func_name, resource, sc_status);
                break;
            }
            /* Set clock */
            sc_status = sc_pm_set_clock_rate(ipc, resource, clk, rate);
            if (sc_status != SC_ERR_NONE) {
                imx_print_set_clock_fail(func_name, resource, *rate, sc_status);
                break;
            }
        }
        sc_status = sc_pm_clock_enable(ipc, resource, clk, true, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(func_name, resource, sc_status);
            break;
        }
        if (debug_flag > 1) {
            sc_status = sc_pm_get_clock_rate(ipc, resource, clk, &val);
            /* For unavailable clocks return SC_ERR_NONE */
            if (sc_status == SC_ERR_UNAVAILABLE) {
                sc_status = SC_ERR_NONE;
                break;
            }
            if (sc_status != SC_ERR_NONE) {
                imx_print_get_clock_fail(func_name, resource, sc_status);
                break;
            }
            kprintf("%s: %s clock rate get %u Hz\n", func_name, sc_rsrc2str(resource), val);
        }
    } while (0);
    return sc_status;
}

/**
 * Code performed if SCI error occurs
 *
 * @param ipc IPC handle.
 */
static void imx_sci_error_occurred(sc_ipc_t ipc)
{
    sc_err_t sc_status;

    /* Reset board */
    do {
        sc_status = sc_pm_reset(ipc, SC_PM_RESET_TYPE_BOARD);
    } while (sc_status != SC_ERR_NONE);
    /* coverity[no_escape] - Suppress coverity INFINITE_LOOP error */
    while (1) {}
}

/**
 * Initialize console UART clock and startup console/QNX debug driver structure.
 *
 * @param ipc           Pointer to the startup data.
 * @param debug_devices Pointer to debug console structure.
 */
void imx_init_console(imx_startup_data_t * startup_data, struct debug_device * debug_devices)
{
    sc_pm_clock_rate_t rate;
    sc_err_t sc_status;

    /* Power up LPUART0 */
    sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_UART_0, SC_PM_PW_MODE_ON);
    if (sc_status != SC_ERR_NONE) {
        imx_sci_error_occurred(startup_data->ipc);
    }
    /* Disable LPUART0 peripheral clock */
    sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_0, SC_PM_CLK_PER, false, false);
    if (sc_status != SC_ERR_NONE) {
        imx_sci_error_occurred(startup_data->ipc);
    }
    /* Set LPUART0 clock root to 80 MHz */
    rate = 80000000;
    sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_UART_0, SC_PM_CLK_PER, &rate);
    if (sc_status != SC_ERR_NONE) {
        imx_sci_error_occurred(startup_data->ipc);
    }
    startup_data->imx_uart_clock[0] = rate;
    /* Enable LPUART0 peripheral clock  */
    sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_0, SC_PM_CLK_PER, true, false);
    if (sc_status != SC_ERR_NONE) {
        imx_sci_error_occurred(startup_data->ipc);
    }
    /* Set debug_devices->defaults variable in "0x5A060000^0.115200.80000000.16" format */
    ksprintf((char *)debug_devices->defaults[0], "0x%x^0.%d.%d.16", IMX_LPUART0_BASE, 115200U, rate);
}

/**
 * Initialize USB clocks.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_usb_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;
    uint32_t val;

    do {
        /* *********************************  USB_OTG1 ***************************** */
        /* Power up USB_OTG1 */
        startup_data->usb_features |= IMX_CHIP_USB2_AVAILABLE;
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_USB_0, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_USB_0, sc_status);
            break;
        }

        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_USB_0_PHY, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_USB_0_PHY, sc_status);
            break;
        }
        /* *********************************  USB3_OTG2 ***************************** */

        if (sc_rm_is_resource_owned(startup_data->ipc, SC_R_USB_2) != 0) {
            startup_data->usb_features |= IMX_CHIP_USB3_AVAILABLE;
            /* Get USB processor type specific features */
            (void)sc_misc_otp_fuse_read(startup_data->ipc, 8U, &val);
            if ((val & 0x800U) == 0x800U) {
                startup_data->usb_features |= IMX_CHIP_USB3_LIMIT_TO_USB2;
                kprintf("USB3 limit to USB2 detected \n");
            }
            /* Power up USB_OTG2 */
            sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_USB_2, SC_PM_PW_MODE_ON);
            if (sc_status != SC_ERR_NONE) {
                imx_print_power_up_fail(__FUNCTION__, SC_R_USB_2, sc_status);
                break;
            }
            sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_USB_2_PHY, SC_PM_PW_MODE_ON);
            if (sc_status != SC_ERR_NONE) {
                imx_print_power_up_fail(__FUNCTION__, SC_R_USB_2_PHY, sc_status);
                break;
            }
            /* Enable USB_OTG2 peripheral clock */
            sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_USB_2, SC_PM_CLK_MISC, true, false);
            if (sc_status != SC_ERR_NONE) {
                imx_print_clock_enable_fail(__FUNCTION__, SC_R_USB_2, sc_status);
                break;
            }
            /* Enable USB_OTG2 peripheral clock */
            sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_USB_2, SC_PM_CLK_MST_BUS , true, false);
            if (sc_status != SC_ERR_NONE) {
                imx_print_clock_enable_fail(__FUNCTION__, SC_R_USB_2, sc_status);
                break;
            }
            /* Enable USB_OTG2 peripheral clock */
            sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_USB_2, SC_PM_CLK_PER , true, false);
            if (sc_status != SC_ERR_NONE) {
                imx_print_clock_enable_fail(__FUNCTION__, SC_R_USB_2, sc_status);
                break;
            }
        } else {
            kprintf("USB3_OTG2 resource not available \n");
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize ENET clock.
 *
 * @param startup_data  Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_enet_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;
    sc_pm_clock_rate_t rate;
    sc_rsrc_t instance[2] = {SC_R_ENET_0, SC_R_ENET_1};
    uint32_t i;

    do {
        for (i = 0; i < sizeof(instance) / sizeof(instance[0]); i++) {
            /* Power up ENET */
            sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, instance[i], SC_PM_PW_MODE_ON);
            if (sc_status != SC_ERR_NONE) {
                imx_print_power_up_fail(__FUNCTION__, instance[i], sc_status);
            }

            /* Disable ENET clock roots */
            sc_status = sc_pm_clock_enable(startup_data->ipc, instance[i], SC_PM_CLK_MISC0, false, false);   /* 0 */
            sc_status |= sc_pm_clock_enable(startup_data->ipc, instance[i], SC_PM_CLK_PER, false, false);    /* 2 */
            sc_status |= sc_pm_clock_enable(startup_data->ipc, instance[i], SC_PM_CLK_BYPASS, false, false); /* 4 */

            if (sc_status != SC_ERR_NONE) {
                imx_print_clock_disable_fail(__FUNCTION__, instance[i], sc_status);
                break;
            }

            /* Set ENET clock root to 250 MHz */
            rate = 250000000U;
            sc_status = sc_pm_set_clock_rate(startup_data->ipc, instance[i], SC_PM_CLK_PER, &rate);
            if (sc_status != SC_ERR_NONE) {
                imx_print_set_clock_fail(__FUNCTION__, instance[i], rate, sc_status);
                break;
            }

            /* Set 25M Time clock root to 25 MHz */
            rate = 25000000U;
            sc_status = sc_pm_set_clock_rate(startup_data->ipc, instance[i], SC_PM_CLK_BYPASS, &rate);
            if (sc_status != SC_ERR_NONE) {
                imx_print_set_clock_fail(__FUNCTION__, instance[i], rate, sc_status);
                break;
            }
            /* Set clock divider to reach 125MHz clock. 250MHz / 2 = 125MHz */
            sc_status = sc_misc_set_control(startup_data->ipc,instance[i], SC_C_CLKDIV, 1U);
            if (sc_status != SC_ERR_NONE) {
                imx_print_set_clock_fail(__FUNCTION__, instance[i], rate, sc_status);
                break;
            }
            /* Enable ENET clock root */
            sc_status = sc_pm_clock_enable(startup_data->ipc, instance[i], SC_PM_CLK_MISC0, true, true);
            sc_status |= sc_pm_clock_enable(startup_data->ipc, instance[i], SC_PM_CLK_PER, true, true);
            sc_status |= sc_pm_clock_enable(startup_data->ipc, instance[i], SC_PM_CLK_BYPASS, true, true);
            if (sc_status != SC_ERR_NONE) {
                imx_print_clock_enable_fail(__FUNCTION__, instance[i], sc_status);
                break;
            }
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize GPIO clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_gpio_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;
    sc_rsrc_t index;

    for (index = SC_R_GPIO_0; index <= SC_R_GPIO_5; index++) {
        /* Power up all GPIO[0-5] devices */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, index, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, index, sc_status);
            return -1;
        }
    }

    for (index = SC_R_BOARD_R0; index <= SC_R_BOARD_R1; index++) {
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, index, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, index, sc_status);
            return -1;
        }
    }
    return 0;
}

/**
 * Initialize GPT clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_gpt_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;
    sc_pm_clock_rate_t rate;

    do {
        /* Power up GPT1 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_GPT_1, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_GPT_1, sc_status);
            break;
        }
        /* Set GPT1 clock root */
        rate = 24000000U;
        sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_GPT_1, SC_PM_CLK_PER, &rate);
        if (sc_status != SC_ERR_NONE) {
            imx_print_set_clock_fail(__FUNCTION__, SC_R_GPT_1, rate, sc_status);
            break;
        }
        /* Enable GPT1 peripheral clock */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_GPT_1, SC_PM_CLK_PER, true, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(__FUNCTION__, SC_R_GPT_1, sc_status);
            break;
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize DC clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_dc_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;

    do {
        /* Power up display controllers, so the I2C belonging to the DCs can be configured */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_DC_0,
                SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_DC_0, sc_status);
            return -1;
        }
        /* Power up display controllers, so the I2C belonging to the DCs can be configured */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_LVDS_0,
                SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_LVDS_0, sc_status);
            return -1;
        }
        /* Power up display controllers, so the I2C belonging to the DCs can be configured */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_MIPI_0,
                SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_MIPI_0, sc_status);
            return -1;
        }
        /* Power up display controllers, so the I2C belonging to the DCs can be configured */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_MIPI_1,
                SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_MIPI_1, sc_status);
            return -1;
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize I2C clock.
 *
 * @param ipc Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_i2c_clock(imx_startup_data_t * startup_data)
{
    uint32_t i;
    sc_err_t sc_status;
    typedef struct {
        sc_rsrc_t rsrc;
        uint8_t  index;
    } imx_i2c_clk_t;
    imx_i2c_clk_t clks[] = {
                             {SC_R_I2C_0, 0},
                             {SC_R_I2C_1, 1},
                             {SC_R_I2C_2, 2},
                             {SC_R_I2C_3, 3},
                             {SC_R_MIPI_1_I2C_0, 4},
                             {SC_R_MIPI_1_I2C_1, 5},
                             {SC_R_CSI_0_I2C_0,  6},
                             {SC_R_MIPI_0_I2C_0, 7},
                             {SC_R_MIPI_0_I2C_1, 8},
                             /* PI I2C disabled , 9 */
                             {SC_R_M4_0_I2C, 10},
                           };

    /* Enable INTMUX peripheral to MUX IRQs from M4 subsystem to GIC */
    sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_M4_0_INTMUX, SC_PM_PW_MODE_ON);
    if (sc_status != SC_ERR_NONE) {
        imx_print_power_up_fail(__FUNCTION__, SC_R_M4_0_INTMUX, sc_status);
        return -1;
    }
    /* Currently just enable I2C clocks with default clock frequency */
    for (i = 0; i < (sizeof(clks) / sizeof(imx_i2c_clk_t)); i++) {
        if (imx_init_periph_clock(startup_data->ipc, clks[i].rsrc, SC_PM_CLK_PER, NULL, __FUNCTION__) != SC_ERR_NONE) {
            return -1;
        }
        /* Save clock source frequency for HWI table */
        if ((sizeof(startup_data->imx_i2c_clock)/sizeof(uint32_t)) == i) {
            /* Array out of bounds */
            kprintf("%s array out of bounds\n", __FUNCTION__);
            return -1;
        }
        sc_status = sc_pm_get_clock_rate(startup_data->ipc,  clks[i].rsrc, SC_PM_CLK_PER, &startup_data->imx_i2c_clock[clks[i].index]);
        if (sc_status != SC_ERR_NONE) {
            imx_print_get_clock_fail(__FUNCTION__, clks[i].rsrc, sc_status);
            return -1;
        }
    }
    return 0;
}

/**
 * Initialize LPSPI clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_lpspi_clock(imx_startup_data_t * startup_data)
{
    sc_rsrc_t i;
    sc_pm_clock_rate_t rate;

    /* Power up lpspi0 - 3 */
    for (i = SC_R_SPI_0; i <= SC_R_SPI_3 ; i++) {
        rate = 160000000U;
        if (startup_data->chip_rev == IMX_CHIP_REV_A) {
            /* 120 MHz is used only for revA since revB has different PLL value of clock source. */
            rate = 120000000U;
        }
        if (imx_init_periph_clock(startup_data->ipc, i, SC_PM_CLK_PER, &rate, __FUNCTION__) != SC_ERR_NONE) {
            return -1;
        }
        startup_data->imx_spi_clk[i - SC_R_SPI_0] = rate;
    }
    return 0;
}

/**
 * Initialize FLEXSPI clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_flexspi_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;
    sc_pm_clock_rate_t rate;
    do {
        /* Power up FlexSPI0 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_FSPI_0, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_FSPI_0, sc_status);
            break;
        }
        /* Set FlexSPI0 clock root to 166 MHz */
        rate = 166000000U;
        sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_FSPI_0, SC_PM_CLK_PER, &rate);
        if (sc_status != SC_ERR_NONE) {
            imx_print_set_clock_fail(__FUNCTION__, SC_R_FSPI_0, rate, sc_status);
            break;
        }
        /* Enable FlexSPI0 peripheral clock */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_FSPI_0, SC_PM_CLK_PER, true, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(__FUNCTION__, SC_R_FSPI_0, sc_status);
            break;
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize USDHC clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_usdhc_clock(imx_startup_data_t * startup_data)
{
    sc_err_t            sc_status;
    sc_pm_clock_rate_t  rate;
    uint32_t            i;
    imx_peripheral_t periphs[] = {
            {SC_R_SDHC_0, 400000000, SC_PM_PW_MODE_ON},   /**< EMMC on CPU board */
            {SC_R_SDHC_1, 200000000, SC_PM_PW_MODE_ON},   /**< SD on CPU board */
    };

    /* Power up usdhc0 - 2 */
    for (i = 0; i < (sizeof(periphs) / sizeof(imx_peripheral_t)); i++) {
        /* Power up uSDHCx */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, periphs[i].per, periphs[i].pwr);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, periphs[i].per, sc_status);
            return -1;
        }
        if (periphs[i].pwr == SC_PM_PW_MODE_ON) {
            /* Disable uSDHCx peripheral clock */
            sc_status = sc_pm_clock_enable(startup_data->ipc, periphs[i].per, SC_PM_CLK_PER, false, false);
            if (sc_status != SC_ERR_NONE) {
                imx_print_clock_disable_fail(__FUNCTION__, periphs[i].per, sc_status);
                return -1;
            }
            if (startup_data->chip_rev == IMX_CHIP_REV_A) {
                /*
                 * IMX8QXP USDHC_CLK_ROOT default source from DPLL, but this DPLL
                 * is not stable, will cause usdhc data transfer crc error. So here
                 * is a workaround, let USDHC_CLK_ROOT source from AVPLL. Due to
                 * AVPLL is fixed to 1000MHz, so here config USDHC1_CLK_ROOT to 333MHz,
                 * USDHC2_CLK_ROOT to 200MHz, make eMMC HS400ES work at 166MHz, and SD
                 * SDR104 work at 200MHz. Not needed for B.0.
                 */
                sc_status = sc_pm_set_clock_parent(startup_data->ipc, periphs[i].per, SC_PM_CLK_PER, SC_PM_PARENT_PLL1);
                if (sc_status != SC_ERR_NONE) {
                    imx_print_set_clock_fail(__FUNCTION__, periphs[i].per, SC_PM_PARENT_PLL1, sc_status);
                    return -1;
                }
            }
            /* Set uSDHCx clock root */
            rate = periphs[i].clk;
            sc_status = sc_pm_set_clock_rate(startup_data->ipc, periphs[i].per, SC_PM_CLK_PER, &rate);
            if (sc_status != SC_ERR_NONE) {
                imx_print_set_clock_fail(__FUNCTION__, periphs[i].per, rate, sc_status);
                return -1;
            }
            startup_data->imx_usdhc_clk[i] = rate;
            /* Enable uSDHCx peripheral clock */
            sc_status = sc_pm_clock_enable(startup_data->ipc, periphs[i].per, SC_PM_CLK_PER, true, false);
            if (sc_status != SC_ERR_NONE) {
                imx_print_clock_enable_fail(__FUNCTION__, periphs[i].per, sc_status);
                return -1;
            }
        }
    }
    return 0;
}

/**
 * Initialize Audio clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_audio_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;
    uint32_t i;
    sc_pm_clock_rate_t rate;
    /* Power on all peripherals so ACM registers will be accessible
     * by acm-imx driver. Otherwise bus error may occur.
     */
    sc_rsrc_t pm_en[] = {
                         SC_R_MCLK_OUT_0,
                         SC_R_MCLK_OUT_1,
                         SC_R_AUDIO_CLK_0,
                         SC_R_AUDIO_CLK_1,
                         SC_R_SAI_0,
                         SC_R_SAI_1,
                         SC_R_SAI_2,
                         SC_R_SAI_3,
                         SC_R_SAI_4,
                         SC_R_SAI_5,
                         SC_R_ESAI_0,
                         SC_R_ASRC_0,
                         SC_R_ASRC_1,
                         SC_R_GPT_5,
                         SC_R_GPT_6,
                         SC_R_GPT_7,
                         SC_R_GPT_8,
                         SC_R_GPT_9,
                         SC_R_GPT_10,
                         SC_R_SPDIF_0,
                         SC_R_MQS_0
                        };
    do {
        /* Power up Audio PLL0 and configure for sample rates 48, 96, 192 kHz */
        rate = 786432000U;
        if (imx_init_periph_clock(startup_data->ipc, SC_R_AUDIO_PLL_0, SC_PM_CLK_PLL, &rate, __FUNCTION__) != SC_ERR_NONE) {
            break;
        }
        /* Configure AUD_PLL_DIV_CLK0 to 24.576 MHz */
        rate = 24576000U;
        if (imx_init_periph_clock(startup_data->ipc, SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC0, &rate, __FUNCTION__) != SC_ERR_NONE) {
            break;
        }
        /* Configure AUD_REC_CLK0 to 24.576 MHz, this clock is mainly used for MCLK Out pins */
        rate = 24576000U;
        if (imx_init_periph_clock(startup_data->ipc, SC_R_AUDIO_PLL_0, SC_PM_CLK_MISC1, &rate, __FUNCTION__) != SC_ERR_NONE) {
            break;
        }
        /* Power up Audio PLL1 and configure for sample rates 44.1, 88.2, 176.4 kHz */
        rate = 812851200U;
        if (imx_init_periph_clock(startup_data->ipc, SC_R_AUDIO_PLL_1, SC_PM_CLK_PLL, &rate, __FUNCTION__) != SC_ERR_NONE) {
            break;
        }
        /* Configure AUD_PLL_DIV_CLK1 to 22.5792 MHz */
        rate = 22579200U;
        if (imx_init_periph_clock(startup_data->ipc, SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC0, &rate, __FUNCTION__) != SC_ERR_NONE) {
            break;
        }
        /* Configure AUD_REC_CLK1 to 22.5792 MHz, this clock is mainly used for MCLK Out pins */
        rate = 22579200U;
        if (imx_init_periph_clock(startup_data->ipc, SC_R_AUDIO_PLL_1, SC_PM_CLK_MISC1, &rate, __FUNCTION__) != SC_ERR_NONE) {
            break;
        }

        for (i = 0; i < (sizeof(pm_en) / sizeof(sc_rsrc_t)); i++) {
            sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, pm_en[i], SC_PM_PW_MODE_ON);
            if (sc_status != SC_ERR_NONE) {
                imx_print_power_up_fail(__FUNCTION__, pm_en[i], sc_status);
                return -1;
            }
        }
        if (startup_data->chip_rev == IMX_CHIP_REV_A) {
            /* Configures GPT0 and GPT1 from audio subsystem for trigger by ESAI TX FIFO */
            imx_acm_set_gpt_event(0, 0x0E);
            imx_acm_set_gpt_event(1, 0x0E);
            /* Configures GPT2 and GPT3 from audio subsystem for trigger by ESAI RX FIFO */
            imx_acm_set_gpt_event(2, 0x0D);
            imx_acm_set_gpt_event(3, 0x0D);
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize FlexCAN clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_flexcan_clock(imx_startup_data_t * startup_data)
{
    sc_err_t           sc_status;
    sc_pm_clock_rate_t rate;

    do {
        /* ********************************* Power up FLEXCAN0,1,2 ********************************* */
        /* Power up FLEXCAN0 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_CAN_0, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_CAN_0, sc_status);
            break;
        }
        /* Power up FLEXCAN1 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_CAN_1, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_CAN_1, sc_status);
            break;
        }
        /* Power up FLEXCAN2 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_CAN_2, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_CAN_2, sc_status);
            break;
        }
        /* ************************ Disable FLEXCAN0,1,2 peripheral clocks ************************* */
        /* Disable FLEXCAN0,1,2 peripheral clock - shared for all FlexCAN modules */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_CAN_0, SC_PM_CLK_PER, false, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_disable_fail(__FUNCTION__, SC_R_CAN_0, sc_status);
            break;
        }
        /* ****************************** Clock root for FLEXCAN0,1,2 ****************************** */
        /* Set FLEXCAN0,1,2 clock root to 80 MHz - shared for all FlexCAN modules */
        rate = 80000000U;
        sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_CAN_0, SC_PM_CLK_PER, &rate);
        if (sc_status != SC_ERR_NONE) {
            imx_print_set_clock_fail(__FUNCTION__, SC_R_CAN_0, rate, sc_status);
            break;
        }

        /* *********************** Enable peripheral clocks for FLEXCAN0,1,2 *********************** */
        /* Enable FLEXCAN0,1,2 peripheral clock - shared for all FlexCAN modules */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_CAN_0, SC_PM_CLK_PER, true, true);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(__FUNCTION__, SC_R_CAN_0, sc_status);
            break;
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize LPUART clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_lpuart_clock(imx_startup_data_t * startup_data)
{
    sc_err_t           sc_status;
    sc_pm_clock_rate_t rate;

    do {
        /* LPUART0 initialized in imx_init_console() function */
#if 0
        /* ********************************* LPUART0 ********************************* */
        /* Power up LPUART0 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_UART_0, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_UART_0, sc_status);
        }
        /* Disable LPUART0 peripheral clock */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_0, SC_PM_CLK_PER, false, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_disable_fail(__FUNCTION__, SC_R_UART_0, sc_status);
        }
        /* Set LPUART0 clock root to 80 MHz */
        rate = 80000000;
        sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_UART_0, SC_PM_CLK_PER, &rate);
        if (sc_status != SC_ERR_NONE) {
            imx_print_set_clock_fail(__FUNCTION__, SC_R_UART_0, rate, sc_status);
        }
        startup_data->imx_uart_clock[0] = rate;
        /* Enable LPUART0 peripheral clock  */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_0, SC_PM_CLK_PER, true, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(__FUNCTION__, SC_R_UART_0, sc_status);
        }
#endif
        /* ********************************* LPUART1 ********************************* */
        /* Power up LPUART1 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_UART_1, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_UART_1, sc_status);
        }
        /* Disable LPUART1 peripheral clock */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_1, SC_PM_CLK_PER, false, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_disable_fail(__FUNCTION__, SC_R_UART_1, sc_status);
        }
        /* Set LPUART1 clock root to 80 MHz */
        rate = 80000000;
        sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_UART_1, SC_PM_CLK_PER, &rate);
        if (sc_status != SC_ERR_NONE) {
            imx_print_set_clock_fail(__FUNCTION__, SC_R_UART_1, rate, sc_status);
        }
        startup_data->imx_uart_clock[1] = rate;
        /* Enable LPUART1 peripheral clock  */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_1, SC_PM_CLK_PER, true, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(__FUNCTION__, SC_R_UART_1, sc_status);
        }
        /* ********************************* LPUART2 ********************************* */
        /* Power up LPUART2 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_UART_2, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_UART_2, sc_status);
        }
        /* Disable LPUART2 peripheral clock */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_2, SC_PM_CLK_PER, false, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_disable_fail(__FUNCTION__, SC_R_UART_2, sc_status);
        }
        /* Set LPUART2 clock root to 80 MHz */
        rate = 80000000;
        sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_UART_2, SC_PM_CLK_PER, &rate);
        if (sc_status != SC_ERR_NONE) {
            imx_print_set_clock_fail(__FUNCTION__, SC_R_UART_2, rate, sc_status);
        }
        startup_data->imx_uart_clock[2] = rate;
        /* Enable LPUART2 peripheral clock  */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_2, SC_PM_CLK_PER, true, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(__FUNCTION__, SC_R_UART_2, sc_status);
        }
        /* ********************************* LPUART3 ********************************* */
        /* Power up LPUART3 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_UART_3, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_UART_3, sc_status);
        }
        /* Disable LPUART3 peripheral clock */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_3, SC_PM_CLK_PER, false, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_disable_fail(__FUNCTION__, SC_R_UART_3, sc_status);
        }
        /* Set LPUART3 clock root to 80 MHz */
        rate = 80000000;
        sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_UART_3, SC_PM_CLK_PER, &rate);
        if (sc_status != SC_ERR_NONE) {
            imx_print_set_clock_fail(__FUNCTION__, SC_R_UART_3, rate, sc_status);
        }
        startup_data->imx_uart_clock[3] = rate;
        /* Enable LPUART3 peripheral clock  */
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_UART_3, SC_PM_CLK_PER, true, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(__FUNCTION__, SC_R_UART_3, sc_status);
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize PCI express clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_pcie_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;

    do {
        /* ********************************* PCIe B ********************************* */
        /* Power up PCIe (HSIO) SERDES 1 */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_SERDES_1, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_SERDES_1, sc_status);
            break;
        }
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_PCIE_B, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_PCIE_B, sc_status);
            break;
        }
        /* Power up PCIe (HSIO) GPIO */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_HSIO_GPIO, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_HSIO_GPIO, sc_status);
            break;
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize GPU clock.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_gpu_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;
    uint32_t gpu_freq_fuse;
    sc_pm_clock_rate_t gpu_freq, shader_freq;
    sc_pm_clock_rate_t gpu_freq_table[4] = {0U, SC_250MHZ, SC_372MHZ, SC_600MHZ};

    do {
        /* ********************************** GPU0 *********************************** */
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_GPU_0_PID0, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, SC_R_GPU_0_PID0, sc_status);
            break;
        }

        /* Calculate GPU and shader frequency */
        (void)sc_misc_otp_fuse_read(startup_data->ipc, 7U, &gpu_freq_fuse);
        gpu_freq_fuse = (gpu_freq_fuse >> 4) & 0x0F;
        if (gpu_freq_fuse != 0U) {
            if ((gpu_freq_fuse & 0x0CU) == 0x0CU) {
                gpu_freq = SC_372MHZ;
                shader_freq = SC_372MHZ;
            } else {
                gpu_freq = gpu_freq_table[(gpu_freq_fuse & 0x03U)];
                shader_freq = gpu_freq_table[(gpu_freq_fuse & 0x03U)];
            }
        } else {
            gpu_freq = SC_700MHZ;
            shader_freq = SC_850MHZ;
        }

        sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_GPU_0_PID0, SC_PM_CLK_PER, &gpu_freq);
        if (sc_status != SC_ERR_NONE) {
            imx_print_set_clock_fail(__FUNCTION__, SC_R_GPU_0_PID0, gpu_freq, sc_status);
            break;
        }

        sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_GPU_0_PID0, SC_PM_CLK_MISC, &shader_freq);
        if (sc_status != SC_ERR_NONE) {
            imx_print_set_clock_fail(__FUNCTION__, SC_R_GPU_0_PID0, shader_freq, sc_status);
            break;
        }

        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_GPU_0_PID0, SC_PM_CLK_MISC, true, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(__FUNCTION__, SC_R_GPU_0_PID0, sc_status);
            break;
        }
        sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_GPU_0_PID0, SC_PM_CLK_PER, true, false);
        if (sc_status != SC_ERR_NONE) {
            imx_print_clock_enable_fail(__FUNCTION__, SC_R_GPU_0_PID0, sc_status);
            break;
        }
        sc_status = sc_misc_set_control(startup_data->ipc, SC_R_GPU_0_PID0, SC_C_ID, 0);
        if (sc_status != SC_ERR_NONE) {
            kprintf("ERROR: GPU0: Failed to set GPU ID! sc_status = %d\n", sc_status);
            break;
        }
        return 0;
    } while (0);
    return -1;
}

/**
 * Initialize Imaging subsystem clocks.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_isi_csi_clock(imx_startup_data_t * startup_data)
{
    sc_err_t sc_status;
    sc_pm_clock_rate_t rate;
    sc_rsrc_t i;

    /* **************************** ISI Channels 0-7 ***************************** */
    for (i = SC_R_ISI_CH0; i <= SC_R_ISI_CH7; i++) {
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, i, SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, i, sc_status);
            return -1;
        }
    }
    /* ******************************** CSI2RX ********************************* */
    sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_CSI_0, SC_PM_PW_MODE_ON);
    if (sc_status != SC_ERR_NONE) {
        imx_print_power_up_fail(__FUNCTION__, SC_R_CSI_0, sc_status);
        return -1;
    }
    /* Set clock */
    rate = 360000000U;
    sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_CSI_0, SC_PM_CLK_PER, &rate);
    if (sc_status != SC_ERR_NONE) {
        imx_print_set_clock_fail(__FUNCTION__, SC_R_CSI_0, rate, sc_status);
        return -1;
    }
    /* Set clock */
    rate = 72000000U;
    sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_CSI_0, SC_PM_CLK_MISC, &rate);
    if (sc_status != SC_ERR_NONE) {
        imx_print_set_clock_fail(__FUNCTION__, SC_R_CSI_0, rate, sc_status);
        return -1;
    }
    sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_CSI_0, SC_PM_CLK_PER, true, false);
    if (sc_status != SC_ERR_NONE) {
        imx_print_clock_enable_fail(__FUNCTION__, SC_R_CSI_0, sc_status);
        return -1;
    }
    sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_CSI_0, SC_PM_CLK_MISC, true, false);
    if (sc_status != SC_ERR_NONE) {
        imx_print_clock_enable_fail(__FUNCTION__, SC_R_CSI_0, sc_status);
        return -1;
    }

    /* Set RXCDRP(LP-RX threshold voltage ) and RXLPRP(LP-CD threshold voltage) values for MIPI CSI0 */
    sc_status = sc_misc_set_control(startup_data->ipc, SC_R_CSI_0, SC_C_CALIB1 , 2);
    if (sc_status != SC_ERR_NONE) {
        kprintf("ERROR: SC_R_CSI_0: Cannot set RXCDRP! sc_status = %d\n", sc_status);
        return -1;
    }
    sc_status = sc_misc_set_control(startup_data->ipc, SC_R_CSI_0, SC_C_CALIB2 , 2);
    if (sc_status != SC_ERR_NONE) {
        kprintf("ERROR: SC_R_CSI_0: Cannot set RXLPRP! sc_status = %d\n", sc_status);
        return -1;
    }

    /* Reset PHY */
    sc_status = sc_misc_set_control(startup_data->ipc, SC_R_CSI_0, SC_C_MIPI_RESET, 1);
    if (sc_status != SC_ERR_NONE) {
        kprintf("ERROR: SC_R_CSI_0: Cannot reset PHY! sc_status = %d\n", sc_status);
        return -1;
    }

    /* ********************************** CI_PI ********************************** */
    /* Change source for pixel clock */
    sc_status = sc_pm_set_clock_parent(startup_data->ipc, SC_R_PI_0, SC_PM_CLK_MISC2, 1);
    if (sc_status != SC_ERR_NONE) {
        imx_print_clock_enable_fail(__FUNCTION__, SC_R_PI_0, sc_status);
        return -1;
    }
    /* Enable power for Parallel capture interface */
    sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, SC_R_PI_0, SC_PM_PW_MODE_ON);
    if (sc_status != SC_ERR_NONE) {
        imx_print_power_up_fail(__FUNCTION__, SC_R_PI_0, sc_status);
        return -1;
    }
    /* Set rate for pixel clock */
    rate = 160000000U;
    sc_status = sc_pm_set_clock_rate(startup_data->ipc, SC_R_PI_0, SC_PM_CLK_MISC2, &rate);
    if (sc_status != SC_ERR_NONE) {
        imx_print_set_clock_fail(__FUNCTION__, SC_R_PI_0, rate, sc_status);
        return -1;
    }
    /* Pixel clock enable */
    sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_PI_0, SC_PM_CLK_PER, true, false);
    if (sc_status != SC_ERR_NONE) {
        imx_print_clock_enable_fail(__FUNCTION__, SC_R_PI_0, sc_status);
        return -1;
    }
    /* 24MHz MCLK reference clock */
    sc_status = sc_pm_clock_enable(startup_data->ipc, SC_R_PI_0, SC_PM_CLK_MISC0, true, false);
    if (sc_status != SC_ERR_NONE) {
        imx_print_clock_enable_fail(__FUNCTION__, SC_R_PI_0, sc_status);
        return -1;
    }
    return 0;
}

/**
 * Power up DMA channels.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_dma_clock(imx_startup_data_t * startup_data)
{
    uint32_t  i;
    sc_err_t  sc_status;
    sc_rsrc_t pm_en[] = {
                         SC_R_DMA_0_CH0  ,
                         SC_R_DMA_0_CH1  ,
                         SC_R_DMA_0_CH2  ,
                         SC_R_DMA_0_CH3  ,
                         SC_R_DMA_0_CH4  ,
                         SC_R_DMA_0_CH5  ,
                         SC_R_DMA_0_CH6  ,
                         SC_R_DMA_0_CH7  ,
                         SC_R_DMA_0_CH8  ,
                         SC_R_DMA_0_CH9  ,
                         SC_R_DMA_0_CH10 ,
                         SC_R_DMA_0_CH11 ,
                         SC_R_DMA_0_CH12 ,
                         SC_R_DMA_0_CH13 ,
                         SC_R_DMA_0_CH14 ,
                         SC_R_DMA_0_CH15 ,
                         SC_R_DMA_0_CH16 ,
                         SC_R_DMA_0_CH17 ,
                         SC_R_DMA_0_CH18 ,
                         SC_R_DMA_0_CH19 ,
                         SC_R_DMA_0_CH20 ,
                         SC_R_DMA_0_CH21 ,
                         SC_R_DMA_0_CH22 ,
                         SC_R_DMA_0_CH23 ,
                         SC_R_DMA_0_CH24 ,
                         SC_R_DMA_0_CH25 ,
                         SC_R_DMA_0_CH26 ,
                         SC_R_DMA_0_CH27 ,
                         SC_R_DMA_0_CH28 ,
                         SC_R_DMA_0_CH29 ,
                         SC_R_DMA_0_CH30 ,
                         SC_R_DMA_0_CH31 ,
                         SC_R_DMA_1_CH0  ,
                         SC_R_DMA_1_CH1  ,
                         SC_R_DMA_1_CH2  ,
                         SC_R_DMA_1_CH3  ,
                         SC_R_DMA_1_CH4  ,
                         SC_R_DMA_1_CH5  ,
                         SC_R_DMA_1_CH6  ,
                         SC_R_DMA_1_CH7  ,
                         SC_R_DMA_1_CH8  ,
                         SC_R_DMA_1_CH9  ,
                         SC_R_DMA_1_CH10 ,
                         SC_R_DMA_1_CH11 ,
                         SC_R_DMA_1_CH12 ,
                         SC_R_DMA_1_CH13 ,
                         SC_R_DMA_1_CH14 ,
                         SC_R_DMA_1_CH15 ,
                         SC_R_DMA_2_CH0  ,
                         SC_R_DMA_2_CH1  ,
                         SC_R_DMA_2_CH2  ,
                         SC_R_DMA_2_CH3  ,
                         SC_R_DMA_2_CH4  ,
                         SC_R_DMA_2_CH5  ,
                         SC_R_DMA_2_CH6  ,
                         SC_R_DMA_2_CH7  ,
                         SC_R_DMA_2_CH8  ,
                         SC_R_DMA_2_CH9  ,
                         SC_R_DMA_2_CH10 ,
                         SC_R_DMA_2_CH11 ,
                         SC_R_DMA_2_CH12 ,
                         SC_R_DMA_2_CH13 ,
                         SC_R_DMA_2_CH14 ,
                         SC_R_DMA_2_CH15 ,
                         SC_R_DMA_2_CH16 ,
                         SC_R_DMA_2_CH17 ,
                         SC_R_DMA_2_CH18 ,
                         SC_R_DMA_2_CH19 ,
                         SC_R_DMA_2_CH20 ,
                         SC_R_DMA_2_CH21 ,
                         SC_R_DMA_2_CH22 ,
                         SC_R_DMA_2_CH23 ,
                         SC_R_DMA_2_CH24 ,
                         SC_R_DMA_2_CH25 ,
                         SC_R_DMA_2_CH26 ,
                         SC_R_DMA_2_CH27 ,
                         SC_R_DMA_2_CH28 ,
                         SC_R_DMA_2_CH29 ,
                         SC_R_DMA_2_CH30 ,
                         SC_R_DMA_2_CH31 ,
                         SC_R_DMA_3_CH0  ,
                         SC_R_DMA_3_CH1  ,
                         SC_R_DMA_3_CH2  ,
                         SC_R_DMA_3_CH3  ,
                         SC_R_DMA_3_CH4  ,
                         SC_R_DMA_3_CH5  ,
                         SC_R_DMA_3_CH6  ,
                         SC_R_DMA_3_CH7  ,
                         SC_R_DMA_3_CH8  ,
                         SC_R_DMA_3_CH9  ,
                         SC_R_DMA_3_CH10 ,
                         SC_R_DMA_3_CH11 ,
                         SC_R_DMA_3_CH12 ,
                         SC_R_DMA_3_CH13 ,
                         SC_R_DMA_3_CH14 ,
                         SC_R_DMA_3_CH15 };

    for (i = 0; i < (sizeof(pm_en) / sizeof(sc_rsrc_t)); i++) {
        sc_status = sc_pm_set_resource_power_mode(startup_data->ipc, pm_en[i], SC_PM_PW_MODE_ON);
        if (sc_status != SC_ERR_NONE) {
            imx_print_power_up_fail(__FUNCTION__, pm_en[i], sc_status);
            return -1;
        }
    }
    return 0;
}
/**
 * Initialize peripheral clocks.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
int imx_init_clocks(imx_startup_data_t * startup_data)
{
    do {
#if IMX_DMA_INIT_ENABLED
        if (imx_init_dma_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_GPIO_INIT_ENABLED
        if (imx_init_gpio_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_GPT_INIT_ENABLED
        if (imx_init_gpt_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_ENET_INIT_ENABLED
        if (imx_init_enet_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_LPUART_INIT_ENABLED
        if (imx_init_lpuart_clock(startup_data) != 0) {
             break;
        }
#endif
#if IMX_ISI_CSI_INIT_ENABLED
        /* The ISI and MIPI-CSI clocks needs to be enabled so the i2c devices owned be the CSI2
         * can be setup. */
        if (imx_init_isi_csi_clock(startup_data) != 0) {
             break;
         }
#endif
#if IMX_DC_INIT_ENABLED
        if (sc_rm_is_resource_owned(startup_data->ipc, SC_R_DC_0) != 0) {
            startup_data->dc_features |= IMX_CHIP_DC0_PORT0_AVAIL;
            startup_data->dc_features |= IMX_CHIP_DC0_PORT1_AVAIL;
            if (imx_init_dc_clock(startup_data) != 0) {
                break;
            }
        }
#endif
#if IMX_I2C_INIT_ENABLED
        if (imx_init_i2c_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_LPSPI_INIT_ENABLED
        if (imx_init_lpspi_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_FLEXSPI_INIT_ENABLED
        if (imx_init_flexspi_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_USDHC_INIT_ENABLED
        if (imx_init_usdhc_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_USB_INIT_ENABLED
        if (imx_init_usb_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_AUDIO_INIT_ENABLED
        if (imx_init_audio_clock(startup_data) != 0) {
            break;
        }
#endif
#if IMX_FLEXCAN_INIT_ENABLED
        if (imx_init_flexcan_clock(startup_data) != 0) {
             break;
        }
#endif
#if IMX_PCIE_INIT_ENABLED
        if (imx_init_pcie_clock(startup_data) != 0) {
             break;
        }
#endif
#if IMX_GPU_INIT_ENABLED
        if (sc_rm_is_resource_owned(startup_data->ipc, SC_R_GPU_0_PID0) != 0) {
            if (imx_init_gpu_clock(startup_data) != 0) {
                break;
            }
        }
#endif
        return 0;
    } while (0);
    return -1;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_init_clocks.c $ $Rev: 891625 $")
#endif
