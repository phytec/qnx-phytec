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
#include <aarch64/imx8_common/imx_gpio.h>
#include <hw/nxp/imx8/sci/sci.h>
#include <hw/nxp/imx8/sci/imx8qxp_pads.h>
#include "board.h"
#include "qxp_cpu.h"
#include "imx_startup.h"

/** Definition of PADS alternate functions */
#define IMX_PAD_ALT_0   0   /**< PAD alternate function 0 */
#define IMX_PAD_ALT_1   1   /**< PAD alternate function 1 */
#define IMX_PAD_ALT_2   2   /**< PAD alternate function 2 */
#define IMX_PAD_ALT_3   3   /**< PAD alternate function 3 */
#define IMX_PAD_ALT_4   4   /**< PAD alternate function 3 */

/** Structure encapsulating all pad configuration */
typedef struct {
    sc_pad_t                pad;    /**< PAD name */
    uint8_t                 mux;    /**< PAD mux (pad alternate function) */
    sc_pad_config_t         cfg;    /**< PAD configuration - Normal, Open drain, ... */
    sc_pad_iso_t            iso;    /**< PAD isolation */
    sc_pad_28fdsoi_dse_t    dse;    /**< PAD drive strength */
    sc_pad_28fdsoi_ps_t     ps;     /**< PAD pull selection */
} imx_pad_t;

#ifdef IMX_DELAY_ENABLE
/**
 * Delay function.
 *
 * @param   timeout - number of NOP instructions to be executed.
 */
static void imx_delay(uint32_t timeout)
{
    while (timeout--) {
        __asm__ __volatile__("nop");
    }
}
#endif

/**
 * Print error information about unsuccessful setup of mux.
 *
 * @param func_name Pointer to a string containing name of function to print.
 * @param pad       Pad number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_set_mux_fail(const char * func_name, sc_pad_t pad, sc_err_t sc_status)
{
    kprintf("%s: sc_pad_set_mux failed for pad %u. SC status: %s\n", func_name, pad, sc_status2str(sc_status));
}

/**
 * Print error information about unsuccessful setup of electric features.
 *
 * @param func_name Pointer to a string containing name of function to print.
 * @param pad       Pad number to print.
 * @param sc_status SC status information to print.
 */
static void imx_print_set_gp_fail(const char * func_name, sc_pad_t pad, sc_err_t sc_status)
{
    kprintf("%s: sc_pad_set_gp_28fdsoi failed for pad %u. SC status: %s\n", func_name, pad, sc_status2str(sc_status));
}

/**
 * PAD configuration routine.
 *
 * @param ipc  IPC handle.
 * @param pads Pointer to PAD configuration array.
 * @param size Number of pads for configuration.
 *
 * @retval 0  PAD configuration OK.
 * @retval -1 Error in PAD configuration.
 */
static int set_28fdsoi_pad(sc_ipc_t ipc, imx_pad_t *pads, uint32_t size)
{
    uint32_t i;
    sc_err_t err;

    for (i = 0; i < size; i++) {
        err = sc_pad_set_mux(ipc, pads[i].pad, pads[i].mux, pads[i].cfg, pads[i].iso);
        if (err != SC_ERR_NONE) {
            imx_print_set_mux_fail(__FUNCTION__, pads[i].pad, err);
            return -1;
        }
        err = sc_pad_set_gp_28fdsoi(ipc, pads[i].pad, pads[i].dse, pads[i].ps);
        if (err != SC_ERR_NONE) {
            imx_print_set_gp_fail(__FUNCTION__, pads[i].pad, err);
            return -1;
        }
    }
    return 0;
}

#if IMX_USDHC_INIT_ENABLED
/**
 * Routes and configures electrical properties of PADs for uSDHC0 device.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_usdhc0_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        {SC_P_EMMC0_CLK,      IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* CLK */
        {SC_P_EMMC0_CMD,      IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* CMD */
        {SC_P_EMMC0_DATA0,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA0 */
        {SC_P_EMMC0_DATA1,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA1 */
        {SC_P_EMMC0_DATA2,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA2 */
        {SC_P_EMMC0_DATA3,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA3 */
        {SC_P_EMMC0_DATA4,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA4 */
        {SC_P_EMMC0_DATA5,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA5 */
        {SC_P_EMMC0_DATA6,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA6 */
        {SC_P_EMMC0_DATA7,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA7 */
        {SC_P_EMMC0_STROBE,   IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PD},   /* STROBE */
        /* RESET_B not connected to the controller */
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}

/**
 * Routes and configures electrical properties of PADs for uSDHCx device(s).
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_usdhc1_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        /* USDHC1 */
        {SC_P_USDHC1_CLK,     IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* CLK */
        {SC_P_USDHC1_CMD,     IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* CMD */
        {SC_P_USDHC1_DATA0,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA0 */
        {SC_P_USDHC1_DATA1,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA1 */
        {SC_P_USDHC1_DATA2,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA2 */
        {SC_P_USDHC1_DATA3,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},   /* DATA3 */
        {SC_P_USDHC1_WP,      IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW,  SC_PAD_28FDSOI_PS_PU},   /* WP */
        {SC_P_USDHC1_CD_B,    IMX_PAD_ALT_4, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW,  SC_PAD_28FDSOI_PS_PU},   /* CD_B - route as GPIO */
        {SC_P_USDHC1_RESET_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW,  SC_PAD_28FDSOI_PS_PU},   /* RESET_B */
        {SC_P_USDHC1_VSELECT, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW,  SC_PAD_28FDSOI_PS_PU},   /* VSELECT */
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_USDHC_INIT_ENABLED */

#if IMX_FLEXSPI_INIT_ENABLED
/**
 * FLEXSPI mux and electrical properties.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_flexspi_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        /* QSPI0A */
        {SC_P_QSPI0A_DATA0, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0A_DATA1, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0A_DATA2, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0A_DATA3, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0A_DQS,   IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0A_SS0_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0A_SS1_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0A_SCLK,  IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},

        /* QSPI0B */
        {SC_P_QSPI0B_DATA0, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0B_DATA1, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0B_DATA2, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0B_DATA3, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0B_SCLK,  IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0B_DQS,   IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0B_SS0_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_QSPI0B_SS1_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_FLEXSPI_INIT_ENABLED */

#if IMX_USB_INIT_ENABLED
/**
 * Route and configure electrical properties of PINs for USB devices.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_usb_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        /* OTG power*/
        {SC_P_USB_SS3_TC0, IMX_PAD_ALT_1, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_33V_4MA, SC_PAD_28FDSOI_PS_KEEPER},
        /* LSIO_GPIO0_IO30 for Type-A/OTG select */
        {SC_P_SAI1_RXC, IMX_PAD_ALT_4, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_USB_INIT_ENABLED */

#if IMX_I2C_INIT_ENABLED
/**
 * Route and configure electrical properties of pads for I2C devices.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_i2c_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        {SC_P_USB_SS3_TC1, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_33V_2MA, SC_PAD_28FDSOI_PS_PU},       /* ADMA.I2C1.SCL (SS3_I2C1_SCL) */
        {SC_P_USB_SS3_TC3, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_33V_2MA, SC_PAD_28FDSOI_PS_PU},       /* ADMA.I2C1.SDA (SS3_I2C1_SDA) */

        {SC_P_MIPI_DSI0_I2C0_SCL, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* MIPI_DSI0.I2C0.SCL */
        {SC_P_MIPI_DSI0_I2C0_SDA, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* MIPI_DSI0.I2C0.SDA */

        {SC_P_MIPI_DSI1_I2C0_SCL, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* MIPI_DSI1.I2C0.SCL */
        {SC_P_MIPI_DSI1_I2C0_SDA, IMX_PAD_ALT_0, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU}, /* MIPI_DSI1.I2C0.SDA */
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_I2C_INIT_ENABLED */

#if IMX_AUDIO_INIT_ENABLED
/**
 * Route and configure electrical properties of PADs for audio devices.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_audio_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
	/* SAI1_TXFS on FLEXCAN0_TX */
        {SC_P_FLEXCAN0_TX,   IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
	/* SAI1_TXCLK on FLEXCAN0_RX */
        {SC_P_FLEXCAN0_RX,   IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
	/* SAI1_RXD on FLEXCAN1_TX */
        {SC_P_FLEXCAN1_TX,   IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
	/* SAI1_TXD on FLEXCAN1_RX */
        {SC_P_FLEXCAN1_RX,   IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},

        {SC_P_MCLK_OUT0,     IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
    };

    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_AUDIO_INIT_ENABLED */

#if IMX_ENET_INIT_ENABLED
/**
 * Function performs configuration of the ENET0 pads.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_enet0_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        {SC_P_ENET0_RGMII_RX_CTL, IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXD0,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXD1,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXD2,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXD3,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_RXC,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_TX_CTL, IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_TXD0,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_TXD1,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_TXD2,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_TXD3,   IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_RGMII_TXC,    IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_MDC,          IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_ENET0_MDIO,         IMX_PAD_ALT_0, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}


/**
 * Function performs configuration of the ENET1 pads.
 */
int imx_init_enet1_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        {SC_P_SPDIF0_TX,     IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},    /* CONN.ENET1.RGMII_RX_CTL */
        {SC_P_SPDIF0_RX,     IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},    /* CONN.ENET1.RGMII_RXD0 */
        {SC_P_ESAI0_TX3_RX2, IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},    /* CONN.ENET1.RGMII_RXD1 */
        {SC_P_ESAI0_TX2_RX3, IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},    /* CONN.ENET1.RGMII_RXD2 */
        {SC_P_ESAI0_TX1,     IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},    /* CONN.ENET1.RGMII_RXD3 */
        {SC_P_ESAI0_TX0,     IMX_PAD_ALT_3, SC_PAD_CONFIG_OD_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE},    /* CONN.ENET1.RGMII_RXC */
        {SC_P_ESAI0_SCKR,    IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* CONN.ENET1.RGMII_TX_CTL */
        {SC_P_ESAI0_TX4_RX1, IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* CONN.ENET1.RGMII_TXD0 */
        {SC_P_ESAI0_TX5_RX0, IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* CONN.ENET1.RGMII_TXD1 */
        {SC_P_ESAI0_FST,     IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* CONN.ENET1.RGMII_TXD2 */
        {SC_P_ESAI0_SCKT,    IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* CONN.ENET1.RGMII_TXD3 */
        {SC_P_ESAI0_FSR,     IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* CONN.ENET1.RGMII_TXC */
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* IMX_ENET_INIT_ENABLED */

#if IMX_FLEXCAN_INIT_ENABLED
/**
 * Routes and configures electrical properties of PADs for FlexCAN devices.
 * Note that these lines are shared with UART2.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_flexcan_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        {SC_P_UART2_RX, IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
        {SC_P_UART2_TX, IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_PU},
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif

#if IMX_LPUART_INIT_ENABLED
/**
 * Routes and configures electrical properties of PADs for LPUART devices.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_lpuart_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        /* UART0 */
        {SC_P_UART0_RX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU},   /* UART0_RX */
        {SC_P_UART0_TX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* UART0_TX */

        /* UART1 */
        {SC_P_UART1_RX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU},   /* UART1_RX */
        {SC_P_UART1_TX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* UART1_TX */

#if (IMX_FLEXCAN_INIT_ENABLED == 1)
        /* UART2 -- shared with FlexCAN */
        {SC_P_UART2_RX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU},   /* UART2_RX */
        {SC_P_UART2_TX, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* UART2_TX */
#endif

    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_LPUART_INIT_ENABLED */

#if IMX_GPIO_INIT_ENABLED
/**
 * Routes and configures electrical properties of PADs for GPIO.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_gpio_pads(imx_startup_data_t * startup_data)
{
    int err;

    imx_pad_t pads[] = {
	/* LVDS0 DSI0_nSPI_IRQ (GPIO1_IO19) */
        {SC_P_FLEXCAN2_RX, IMX_PAD_ALT_4, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU},
	/* LVDS1 DSI1_nSPI_IRQ (GPIO1_IO05) */
        {SC_P_SPI0_SDI, IMX_PAD_ALT_4, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU},
	/* LVDS0 DISP_ENA (LSIO GPIO1_IO28) */
        {SC_P_MIPI_DSI0_GPIO0_01, IMX_PAD_ALT_4, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU},
	/* LVDS1 DISP_ENA (LSIO GPIO2_IO00)*/
        {SC_P_MIPI_DSI1_GPIO0_01, IMX_PAD_ALT_4, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU},
	/* LVDS0 DISP_BRIGHT/MIPI_DSI0_PWM0_OUT */
        {SC_P_MIPI_DSI0_GPIO0_00, IMX_PAD_ALT_2, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU},
	/* LVDS1 DISP_BRIGHT/MIPI_DSI1_PWM0_OUT */
        {SC_P_MIPI_DSI0_GPIO0_00, IMX_PAD_ALT_2, SC_PAD_CONFIG_OUT_IN, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_PU},
    };

    err = set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));

    /* make LVDS0 DSI0_nSPI_IRQ an input (GPIO1_IO19) */
    out32(IMX_GPIO1_BASE + IMX_GPIO_GDIR,
		(in32(IMX_GPIO1_BASE + IMX_GPIO_GDIR) & (~(0x01 << 19))));

    /* make LVDS1 DSI0_nSPI_IRQ an input (GPIO1_IO05) */
    out32(IMX_GPIO1_BASE + IMX_GPIO_GDIR,
		(in32(IMX_GPIO1_BASE + IMX_GPIO_GDIR) & (~(0x01 << 5))));

    /* force LVDS0 DISP_ENA on (GPIO1_IO28)*/
    out32(IMX_GPIO1_BASE + IMX_GPIO_GDIR,
		(in32(IMX_GPIO1_BASE + IMX_GPIO_GDIR) | (0x01 << 28)));
    out32(IMX_GPIO1_BASE + IMX_GPIO_DR,
		(in32(IMX_GPIO1_BASE + IMX_GPIO_DR) | (0x01 << 28)));

    /* force LVDS1 DISP_ENA on (GPIO2_IO00)*/
    out32(IMX_GPIO2_BASE + IMX_GPIO_GDIR,
		(in32(IMX_GPIO2_BASE + IMX_GPIO_GDIR) | (0x01 << 0)));
    out32(IMX_GPIO2_BASE + IMX_GPIO_DR,
		(in32(IMX_GPIO2_BASE + IMX_GPIO_DR) | (0x01 << 0)));

    return err;
}
#endif	/* if IMX_GPIO_INIT_ENABLED */


#if IMX_PCIE_INIT_ENABLED
/**
 * Routes and configures electrical properties of PADs for PCIe.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_pcie_pads(imx_startup_data_t * startup_data)
{
    /* Most PCIe pads do not need a mux setting */
    imx_pad_t pads[] = {
        {SC_P_PCIE_CTRL0_CLKREQ_B, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* HSIO.PCIE0.CLKREQ_B for PCIE_CLKREQ_N */
        {SC_P_PCIE_CTRL0_WAKE_B,   IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* HSIO.PCIE0.WAKE_B for PCIE_WAKE_N */
        {SC_P_PCIE_CTRL0_PERST_B,  IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* HSIO.PCIE0.PERST_B for PCIE_RST_N */
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_PCIE_INIT_ENABLED */

#if IMX_MIPI_CSI_INIT_ENABLED
/**
 * Routes and configures electrical properties of PADs for MIPI CSI.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_mipi_csi_pads(imx_startup_data_t * startup_data)
{

    imx_pad_t pads[] = {
        {SC_P_MIPI_CSI0_MCLK_OUT, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_2MA, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_MIPI_CSI0_I2C0_SDA, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_2MA, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_MIPI_CSI0_I2C0_SCL, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_2MA, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_MIPI_CSI0_GPIO0_01, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_2MA, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_MIPI_CSI0_GPIO0_00, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_2MA, SC_PAD_28FDSOI_PS_NONE},
        {SC_P_MIPI_CSI0_GPIO0_00, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_18V_2MA, SC_PAD_28FDSOI_PS_NONE},
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_MIPI_CSI_INIT_ENABLED */

#if IMX_PARALLEL_CSI_INIT_ENABLED
/**
 * Routes and configures electrical properties of PADs for Parallel CSI.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_init_parallel_csi_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        {SC_P_CSI_MCLK, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* CSI_MCLK */
        {SC_P_CSI_PCLK, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* CAM0_PCLK */
        {SC_P_CSI_HSYNC, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* CAM0_HSYNC */
        {SC_P_CSI_VSYNC, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* CAM0_VSYNC */

        {SC_P_CSI_D00, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* X_CSI_D00 / CAM0_PD2 */
        {SC_P_CSI_D01, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* X_CSI_D01 / CAM0_PD3 */
        {SC_P_CSI_D02, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* X_CSI_D02 / CAM0_PD4 */
        {SC_P_CSI_D03, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* X_CSI_D03 / CAM0_PD5 */
        {SC_P_CSI_D04, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* X_CSI_D04 / CAM0_PD6 */
        {SC_P_CSI_D05, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* X_CSI_D05 / CAM0_PD7 */
        {SC_P_CSI_D06, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* X_CSI_D06 / CAM0_PD8 */
        {SC_P_CSI_D07, IMX_PAD_ALT_0, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_LOW, SC_PAD_28FDSOI_PS_NONE}, /* X_CSI_D07 / CAM0_PD9 */
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_PARALLEL_CSI_INIT_ENABLED */

#if IMX_PARALLEL_LCD_INIT_ENABLED
/**
 * Routes and configures electrical properties of PADs for parallel LCD.
 *
 * @param ipc IPC handle.
 *
 * @return Execution status.
 */
static int imx_init_parallel_lcd_pads(imx_startup_data_t * startup_data)
{
    imx_pad_t pads[] = {
        {SC_P_ESAI0_FSR,      IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D00_B0 */
        {SC_P_ESAI0_FST,      IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D01_B1 */
        {SC_P_ESAI0_SCKR,     IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D02_B2 */
        {SC_P_ESAI0_SCKT,     IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D03_B3 */
        {SC_P_ESAI0_TX0,      IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D04_B4 */
        {SC_P_ESAI0_TX1,      IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D04_B5 */
        {SC_P_ESAI0_TX2_RX3,  IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D06_B6 */
        {SC_P_ESAI0_TX3_RX2,  IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D07_B7 */

        {SC_P_ESAI0_TX4_RX1,  IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D08_G0 */
        {SC_P_ESAI0_TX5_RX0,  IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D09_G1 */
        {SC_P_SPDIF0_RX,      IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D10_G2 */
        {SC_P_SPDIF0_TX,      IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D11_G3 */
        {SC_P_SPDIF0_EXT_CLK, IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D12_G4 */
        {SC_P_SPI3_SCK,       IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D13_G5 */
        {SC_P_SPI3_SDO,       IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D14_G6 */
        {SC_P_SPI3_SDI,       IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D15_G7 */

        {SC_P_UART1_RTS_B,    IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D16_R0 */
        {SC_P_UART1_CTS_B,    IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D17_R1 */
	{SC_P_SAI0_TXD,       IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D18_R2 */
        {SC_P_SAI0_TXC,       IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D19_R3 */
        {SC_P_SAI0_RXD,       IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D20_R4 */
        {SC_P_SAI1_RXD,       IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D21_R5 */
        {SC_P_SAI1_RXC,       IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D22_R6 */
        {SC_P_SAI1_RXFS,      IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD_D23_R7 */

        {SC_P_SPI3_CS0,       IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD.HSYNC */
        {SC_P_SPI3_CS1,       IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD.RESET */
        {SC_P_MCLK_IN0,       IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD.VSYNC */
        {SC_P_MCLK_IN1,       IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD.EN */
        {SC_P_MCLK_OUT0,      IMX_PAD_ALT_2, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD.CLK */
        {SC_P_SPI0_CS1,       IMX_PAD_ALT_3, SC_PAD_CONFIG_NORMAL, SC_PAD_ISO_OFF, SC_PAD_28FDSOI_DSE_DV_HIGH, SC_PAD_28FDSOI_PS_NONE}, /* LCD.PWM */
    };
    return set_28fdsoi_pad(startup_data->ipc, &pads[0], sizeof(pads) / sizeof(imx_pad_t));
}
#endif	/* if IMX_PARALLEL_LCD_INIT_ENABLED */

/**
 * Function overrides 2V5 logic to 1V8/3V3 logic for ESAI0/ENET1 pads.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
static int imx_set_esai_enet_2V5_override(imx_startup_data_t * startup_data)
{
    int status;
    uint8_t compen, rasrcp, rasrcn, nasrc;
    sc_bool_t fastfrz, nasrc_sel, compok, psw_ovr;

    kprintf("Overriding 2V5 logic for ESAI0/ENET1 pads\n");
    status = sc_pad_get_gp_28fdsoi_comp(startup_data->ipc, SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHB,
                                        &compen, &fastfrz, &rasrcp, &rasrcn,
                                        &nasrc_sel, &compok, &nasrc, &psw_ovr);
    if (status != SC_ERR_NONE) {
        kprintf("sc_pad_get_gp_28fdsoi_comp returned error %i\n", status );
    }
    /* Set psw_ovr to low to choose 1.8/3.3V IO for ESAI */
    psw_ovr = 0;
    status = sc_pad_set_gp_28fdsoi_comp(startup_data->ipc, SC_P_COMP_CTL_GPIO_1V8_3V3_GPIORHB,
                                        compen, fastfrz, rasrcp, rasrcn,
                                        nasrc_sel, psw_ovr);
    if (status != SC_ERR_NONE) {
        kprintf("sc_pad_set_gp_28fdsoi_comp returned error %i\n", status );
    }
    return status;
}

/**
 * Initialize peripheral pads.
 *
 * @param startup_data Pointer to the startup data.
 *
 * @return Execution status.
 */
int imx_init_pads(imx_startup_data_t * startup_data)
{
    do {
        if (startup_data->chip_rev != IMX_CHIP_REV_A) {
            /* Switch from 2V5 to 1V8/3V3 logic for ESAI0/ENET1 pads */
            if (imx_set_esai_enet_2V5_override(startup_data) != 0) {
                break;
            }
        }

#if IMX_ENET_INIT_ENABLED
        /* Configure ENET0 pads */
        if (imx_init_enet0_pads(startup_data) != 0) {
            break;
        }
        /* Configure ENET1 pads */
        if (imx_init_enet1_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_I2C_INIT_ENABLED
        /* Configure I2C pads */
        if (imx_init_i2c_pads(startup_data) != 0) {
            break;
        }
#endif
/* MUST be enabled for fix of I2C1 conflict */
#if IMX_USB_INIT_ENABLED
        /* Configure USB pads */
        if (imx_init_usb_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_USDHC_INIT_ENABLED
        /* Configure USDHC pads */
        if (imx_init_usdhc0_pads(startup_data) != 0) {
            break;
        }
        /* Configure USDHC pads */
        if (imx_init_usdhc1_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_FLEXSPI_INIT_ENABLED
        /* Configure FLEXSPI pads */
        if (imx_init_flexspi_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_FLEXCAN_INIT_ENABLED
        /* Configure FlexCAN pads */
        if (imx_init_flexcan_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_LPUART_INIT_ENABLED
        /* Configure LPUART pads */
        if (imx_init_lpuart_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_AUDIO_INIT_ENABLED
        /* Configure Audio pads */
        if (imx_init_audio_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_GPIO_INIT_ENABLED
        if (imx_init_gpio_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_PCIE_INIT_ENABLED
        /* Configure PCIe pads */
        if (imx_init_pcie_pads(startup_data) != 0) {
            break;
        }
#endif

#if IMX_MIPI_CSI_INIT_ENABLED
        /* Configure PCIe pads */
        if (imx_init_mipi_csi_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_PARALLEL_CSI_INIT_ENABLED
        /* Configure PCIe pads */
        if (imx_init_parallel_csi_pads(startup_data) != 0) {
            break;
        }
#endif
#if IMX_PARALLEL_LCD_INIT_ENABLED
        /* Configure parallel lcd pads */
        if (imx_init_parallel_lcd_pads(startup_data) != 0) {
            break;
        }
#endif
        return 0;
    } while (0);
    return -1;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/startup/boards/imx8xp/qxp-cpu/init_pads.c $ $Rev: 887148 $")
#endif
