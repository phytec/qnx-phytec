/*
* $QNXLicenseC:
* Copyright 2017-2018 NXP
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

/*==========================================================================*/
/*!
 * @file  ipc.c
 *
 * Implementation of the IPC functions using MUs (client side).
 */
/*==========================================================================*/

/* Includes */

#include <hw/nxp/imx8/sci/scfw.h>
#include <hw/nxp/imx8/sci/ipc.h>
#include <hw/nxp/imx8/sci/sci.h>
#include <rpc.h>
#include <fsl_mu.h>

/* Local Defines */

/* Local Types */

/* Local Functions */

/* Local Variables */

const char * sc_rsrc_str[] = {
    "SC_R_A53",
    "SC_R_A53_0",
    "SC_R_A53_1",
    "SC_R_A53_2",
    "SC_R_A53_3",
    "SC_R_A72",
    "SC_R_A72_0",
    "SC_R_A72_1",
    "SC_R_A72_2",
    "SC_R_A72_3",
    "SC_R_CCI",
    "SC_R_DB",
    "SC_R_DRC_0",
    "SC_R_DRC_1",
    "SC_R_GIC_SMMU",
    "SC_R_IRQSTR_M4_0",
    "SC_R_IRQSTR_M4_1",
    "SC_R_SMMU",
    "SC_R_GIC",
    "SC_R_DC_0_BLIT0",
    "SC_R_DC_0_BLIT1",
    "SC_R_DC_0_BLIT2",
    "SC_R_DC_0_BLIT_OUT",
    "SC_R_DC_0_CAPTURE0",
    "SC_R_DC_0_CAPTURE1",
    "SC_R_DC_0_WARP",
    "SC_R_DC_0_INTEGRAL0",
    "SC_R_DC_0_INTEGRAL1",
    "SC_R_DC_0_VIDEO0",
    "SC_R_DC_0_VIDEO1",
    "SC_R_DC_0_FRAC0",
    "SC_R_DC_0_FRAC1",
    "SC_R_DC_0",
    "SC_R_GPU_2_PID0",
    "SC_R_DC_0_PLL_0",
    "SC_R_DC_0_PLL_1",
    "SC_R_DC_1_BLIT0",
    "SC_R_DC_1_BLIT1",
    "SC_R_DC_1_BLIT2",
    "SC_R_DC_1_BLIT_OUT",
    "SC_R_DC_1_CAPTURE0",
    "SC_R_DC_1_CAPTURE1",
    "SC_R_DC_1_WARP",
    "SC_R_DC_1_INTEGRAL0",
    "SC_R_DC_1_INTEGRAL1",
    "SC_R_DC_1_VIDEO0",
    "SC_R_DC_1_VIDEO1",
    "SC_R_DC_1_FRAC0",
    "SC_R_DC_1_FRAC1",
    "SC_R_DC_1",
    "SC_R_GPU_3_PID0",
    "SC_R_DC_1_PLL_0",
    "SC_R_DC_1_PLL_1",
    "SC_R_SPI_0",
    "SC_R_SPI_1",
    "SC_R_SPI_2",
    "SC_R_SPI_3",
    "SC_R_UART_0",
    "SC_R_UART_1",
    "SC_R_UART_2",
    "SC_R_UART_3",
    "SC_R_UART_4",
    "SC_R_EMVSIM_0",
    "SC_R_EMVSIM_1",
    "SC_R_DMA_0_CH0",
    "SC_R_DMA_0_CH1",
    "SC_R_DMA_0_CH2",
    "SC_R_DMA_0_CH3",
    "SC_R_DMA_0_CH4",
    "SC_R_DMA_0_CH5",
    "SC_R_DMA_0_CH6",
    "SC_R_DMA_0_CH7",
    "SC_R_DMA_0_CH8",
    "SC_R_DMA_0_CH9",
    "SC_R_DMA_0_CH10",
    "SC_R_DMA_0_CH11",
    "SC_R_DMA_0_CH12",
    "SC_R_DMA_0_CH13",
    "SC_R_DMA_0_CH14",
    "SC_R_DMA_0_CH15",
    "SC_R_DMA_0_CH16",
    "SC_R_DMA_0_CH17",
    "SC_R_DMA_0_CH18",
    "SC_R_DMA_0_CH19",
    "SC_R_DMA_0_CH20",
    "SC_R_DMA_0_CH21",
    "SC_R_DMA_0_CH22",
    "SC_R_DMA_0_CH23",
    "SC_R_DMA_0_CH24",
    "SC_R_DMA_0_CH25",
    "SC_R_DMA_0_CH26",
    "SC_R_DMA_0_CH27",
    "SC_R_DMA_0_CH28",
    "SC_R_DMA_0_CH29",
    "SC_R_DMA_0_CH30",
    "SC_R_DMA_0_CH31",
    "SC_R_I2C_0",
    "SC_R_I2C_1",
    "SC_R_I2C_2",
    "SC_R_I2C_3",
    "SC_R_I2C_4",
    "SC_R_ADC_0",
    "SC_R_ADC_1",
    "SC_R_FTM_0",
    "SC_R_FTM_1",
    "SC_R_CAN_0",
    "SC_R_CAN_1",
    "SC_R_CAN_2",
    "SC_R_DMA_1_CH0",
    "SC_R_DMA_1_CH1",
    "SC_R_DMA_1_CH2",
    "SC_R_DMA_1_CH3",
    "SC_R_DMA_1_CH4",
    "SC_R_DMA_1_CH5",
    "SC_R_DMA_1_CH6",
    "SC_R_DMA_1_CH7",
    "SC_R_DMA_1_CH8",
    "SC_R_DMA_1_CH9",
    "SC_R_DMA_1_CH10",
    "SC_R_DMA_1_CH11",
    "SC_R_DMA_1_CH12",
    "SC_R_DMA_1_CH13",
    "SC_R_DMA_1_CH14",
    "SC_R_DMA_1_CH15",
    "SC_R_DMA_1_CH16",
    "SC_R_DMA_1_CH17",
    "SC_R_DMA_1_CH18",
    "SC_R_DMA_1_CH19",
    "SC_R_DMA_1_CH20",
    "SC_R_DMA_1_CH21",
    "SC_R_DMA_1_CH22",
    "SC_R_DMA_1_CH23",
    "SC_R_DMA_1_CH24",
    "SC_R_DMA_1_CH25",
    "SC_R_DMA_1_CH26",
    "SC_R_DMA_1_CH27",
    "SC_R_DMA_1_CH28",
    "SC_R_DMA_1_CH29",
    "SC_R_DMA_1_CH30",
    "SC_R_DMA_1_CH31",
    "SC_R_UNUSED1",
    "SC_R_UNUSED2",
    "SC_R_UNUSED3",
    "SC_R_UNUSED4",
    "SC_R_GPU_0_PID0",
    "SC_R_GPU_0_PID1",
    "SC_R_GPU_0_PID2",
    "SC_R_GPU_0_PID3",
    "SC_R_GPU_1_PID0",
    "SC_R_GPU_1_PID1",
    "SC_R_GPU_1_PID2",
    "SC_R_GPU_1_PID3",
    "SC_R_PCIE_A",
    "SC_R_SERDES_0",
    "SC_R_MATCH_0",
    "SC_R_MATCH_1",
    "SC_R_MATCH_2",
    "SC_R_MATCH_3",
    "SC_R_MATCH_4",
    "SC_R_MATCH_5",
    "SC_R_MATCH_6",
    "SC_R_MATCH_7",
    "SC_R_MATCH_8",
    "SC_R_MATCH_9",
    "SC_R_MATCH_10",
    "SC_R_MATCH_11",
    "SC_R_MATCH_12",
    "SC_R_MATCH_13",
    "SC_R_MATCH_14",
    "SC_R_PCIE_B",
    "SC_R_SATA_0",
    "SC_R_SERDES_1",
    "SC_R_HSIO_GPIO",
    "SC_R_MATCH_15",
    "SC_R_MATCH_16",
    "SC_R_MATCH_17",
    "SC_R_MATCH_18",
    "SC_R_MATCH_19",
    "SC_R_MATCH_20",
    "SC_R_MATCH_21",
    "SC_R_MATCH_22",
    "SC_R_MATCH_23",
    "SC_R_MATCH_24",
    "SC_R_MATCH_25",
    "SC_R_MATCH_26",
    "SC_R_MATCH_27",
    "SC_R_MATCH_28",
    "SC_R_LCD_0",
    "SC_R_LCD_0_PWM_0",
    "SC_R_LCD_0_I2C_0",
    "SC_R_LCD_0_I2C_1",
    "SC_R_PWM_0",
    "SC_R_PWM_1",
    "SC_R_PWM_2",
    "SC_R_PWM_3",
    "SC_R_PWM_4",
    "SC_R_PWM_5",
    "SC_R_PWM_6",
    "SC_R_PWM_7",
    "SC_R_GPIO_0",
    "SC_R_GPIO_1",
    "SC_R_GPIO_2",
    "SC_R_GPIO_3",
    "SC_R_GPIO_4",
    "SC_R_GPIO_5",
    "SC_R_GPIO_6",
    "SC_R_GPIO_7",
    "SC_R_GPT_0",
    "SC_R_GPT_1",
    "SC_R_GPT_2",
    "SC_R_GPT_3",
    "SC_R_GPT_4",
    "SC_R_KPP",
    "SC_R_MU_0A",
    "SC_R_MU_1A",
    "SC_R_MU_2A",
    "SC_R_MU_3A",
    "SC_R_MU_4A",
    "SC_R_MU_5A",
    "SC_R_MU_6A",
    "SC_R_MU_7A",
    "SC_R_MU_8A",
    "SC_R_MU_9A",
    "SC_R_MU_10A",
    "SC_R_MU_11A",
    "SC_R_MU_12A",
    "SC_R_MU_13A",
    "SC_R_MU_5B",
    "SC_R_MU_6B",
    "SC_R_MU_7B",
    "SC_R_MU_8B",
    "SC_R_MU_9B",
    "SC_R_MU_10B",
    "SC_R_MU_11B",
    "SC_R_MU_12B",
    "SC_R_MU_13B",
    "SC_R_ROM_0",
    "SC_R_FSPI_0",
    "SC_R_FSPI_1",
    "SC_R_IEE",
    "SC_R_IEE_R0",
    "SC_R_IEE_R1",
    "SC_R_IEE_R2",
    "SC_R_IEE_R3",
    "SC_R_IEE_R4",
    "SC_R_IEE_R5",
    "SC_R_IEE_R6",
    "SC_R_IEE_R7",
    "SC_R_SDHC_0",
    "SC_R_SDHC_1",
    "SC_R_SDHC_2",
    "SC_R_ENET_0",
    "SC_R_ENET_1",
    "SC_R_MLB_0",
    "SC_R_DMA_2_CH0",
    "SC_R_DMA_2_CH1",
    "SC_R_DMA_2_CH2",
    "SC_R_DMA_2_CH3",
    "SC_R_DMA_2_CH4",
    "SC_R_USB_0",
    "SC_R_USB_1",
    "SC_R_USB_0_PHY",
    "SC_R_USB_2",
    "SC_R_USB_2_PHY",
    "SC_R_DTCP",
    "SC_R_NAND",
    "SC_R_LVDS_0",
    "SC_R_LVDS_0_PWM_0",
    "SC_R_LVDS_0_I2C_0",
    "SC_R_LVDS_0_I2C_1",
    "SC_R_LVDS_1",
    "SC_R_LVDS_1_PWM_0",
    "SC_R_LVDS_1_I2C_0",
    "SC_R_LVDS_1_I2C_1",
    "SC_R_LVDS_2",
    "SC_R_LVDS_2_PWM_0",
    "SC_R_LVDS_2_I2C_0",
    "SC_R_LVDS_2_I2C_1",
    "SC_R_M4_0_PID0",
    "SC_R_M4_0_PID1",
    "SC_R_M4_0_PID2",
    "SC_R_M4_0_PID3",
    "SC_R_M4_0_PID4",
    "SC_R_M4_0_RGPIO",
    "SC_R_M4_0_SEMA42",
    "SC_R_M4_0_TPM",
    "SC_R_M4_0_PIT",
    "SC_R_M4_0_UART",
    "SC_R_M4_0_I2C",
    "SC_R_M4_0_INTMUX",
    "SC_R_M4_0_SIM",
    "SC_R_M4_0_WDOG",
    "SC_R_M4_0_MU_0B",
    "SC_R_M4_0_MU_0A0",
    "SC_R_M4_0_MU_0A1",
    "SC_R_M4_0_MU_0A2",
    "SC_R_M4_0_MU_0A3",
    "SC_R_M4_0_MU_1A",
    "SC_R_M4_1_PID0",
    "SC_R_M4_1_PID1",
    "SC_R_M4_1_PID2",
    "SC_R_M4_1_PID3",
    "SC_R_M4_1_PID4",
    "SC_R_M4_1_RGPIO",
    "SC_R_M4_1_SEMA42",
    "SC_R_M4_1_TPM",
    "SC_R_M4_1_PIT",
    "SC_R_M4_1_UART",
    "SC_R_M4_1_I2C",
    "SC_R_M4_1_INTMUX",
    "SC_R_M4_1_SIM",
    "SC_R_M4_1_WDOG",
    "SC_R_M4_1_MU_0B",
    "SC_R_M4_1_MU_0A0",
    "SC_R_M4_1_MU_0A1",
    "SC_R_M4_1_MU_0A2",
    "SC_R_M4_1_MU_0A3",
    "SC_R_M4_1_MU_1A",
    "SC_R_SAI_0",
    "SC_R_SAI_1",
    "SC_R_SAI_2",
    "SC_R_IRQSTR_SCU2",
    "SC_R_IRQSTR_DSP",
    "SC_R_UNUSED5",
    "SC_R_UNUSED6",
    "SC_R_AUDIO_PLL_0",
    "SC_R_PI_0",
    "SC_R_PI_0_PWM_0",
    "SC_R_PI_0_PWM_1",
    "SC_R_PI_0_I2C_0",
    "SC_R_PI_0_PLL",
    "SC_R_PI_1",
    "SC_R_PI_1_PWM_0",
    "SC_R_PI_1_PWM_1",
    "SC_R_PI_1_I2C_0",
    "SC_R_PI_1_PLL",
    "SC_R_SC_PID0",
    "SC_R_SC_PID1",
    "SC_R_SC_PID2",
    "SC_R_SC_PID3",
    "SC_R_SC_PID4",
    "SC_R_SC_SEMA42",
    "SC_R_SC_TPM",
    "SC_R_SC_PIT",
    "SC_R_SC_UART",
    "SC_R_SC_I2C",
    "SC_R_SC_MU_0B",
    "SC_R_SC_MU_0A0",
    "SC_R_SC_MU_0A1",
    "SC_R_SC_MU_0A2",
    "SC_R_SC_MU_0A3",
    "SC_R_SC_MU_1A",
    "SC_R_SYSCNT_RD",
    "SC_R_SYSCNT_CMP",
    "SC_R_DEBUG",
    "SC_R_SYSTEM",
    "SC_R_SNVS",
    "SC_R_OTP",
    "SC_R_VPU_PID0",
    "SC_R_VPU_PID1",
    "SC_R_VPU_PID2",
    "SC_R_VPU_PID3",
    "SC_R_VPU_PID4",
    "SC_R_VPU_PID5",
    "SC_R_VPU_PID6",
    "SC_R_VPU_PID7",
    "SC_R_VPU_UART",
    "SC_R_VPUCORE",
    "SC_R_VPUCORE_0",
    "SC_R_VPUCORE_1",
    "SC_R_VPUCORE_2",
    "SC_R_VPUCORE_3",
    "SC_R_DMA_4_CH0",
    "SC_R_DMA_4_CH1",
    "SC_R_DMA_4_CH2",
    "SC_R_DMA_4_CH3",
    "SC_R_DMA_4_CH4",
    "SC_R_ISI_CH0",
    "SC_R_ISI_CH1",
    "SC_R_ISI_CH2",
    "SC_R_ISI_CH3",
    "SC_R_ISI_CH4",
    "SC_R_ISI_CH5",
    "SC_R_ISI_CH6",
    "SC_R_ISI_CH7",
    "SC_R_MJPEG_DEC_S0",
    "SC_R_MJPEG_DEC_S1",
    "SC_R_MJPEG_DEC_S2",
    "SC_R_MJPEG_DEC_S3",
    "SC_R_MJPEG_ENC_S0",
    "SC_R_MJPEG_ENC_S1",
    "SC_R_MJPEG_ENC_S2",
    "SC_R_MJPEG_ENC_S3",
    "SC_R_MIPI_0",
    "SC_R_MIPI_0_PWM_0",
    "SC_R_MIPI_0_I2C_0",
    "SC_R_MIPI_0_I2C_1",
    "SC_R_MIPI_1",
    "SC_R_MIPI_1_PWM_0",
    "SC_R_MIPI_1_I2C_0",
    "SC_R_MIPI_1_I2C_1",
    "SC_R_CSI_0",
    "SC_R_CSI_0_PWM_0",
    "SC_R_CSI_0_I2C_0",
    "SC_R_CSI_1",
    "SC_R_CSI_1_PWM_0",
    "SC_R_CSI_1_I2C_0",
    "SC_R_HDMI",
    "SC_R_HDMI_I2S",
    "SC_R_HDMI_I2C_0",
    "SC_R_HDMI_PLL_0",
    "SC_R_HDMI_RX",
    "SC_R_HDMI_RX_BYPASS",
    "SC_R_HDMI_RX_I2C_0",
    "SC_R_ASRC_0",
    "SC_R_ESAI_0",
    "SC_R_SPDIF_0",
    "SC_R_SPDIF_1",
    "SC_R_SAI_3",
    "SC_R_SAI_4",
    "SC_R_SAI_5",
    "SC_R_GPT_5",
    "SC_R_GPT_6",
    "SC_R_GPT_7",
    "SC_R_GPT_8",
    "SC_R_GPT_9",
    "SC_R_GPT_10",
    "SC_R_DMA_2_CH5",
    "SC_R_DMA_2_CH6",
    "SC_R_DMA_2_CH7",
    "SC_R_DMA_2_CH8",
    "SC_R_DMA_2_CH9",
    "SC_R_DMA_2_CH10",
    "SC_R_DMA_2_CH11",
    "SC_R_DMA_2_CH12",
    "SC_R_DMA_2_CH13",
    "SC_R_DMA_2_CH14",
    "SC_R_DMA_2_CH15",
    "SC_R_DMA_2_CH16",
    "SC_R_DMA_2_CH17",
    "SC_R_DMA_2_CH18",
    "SC_R_DMA_2_CH19",
    "SC_R_DMA_2_CH20",
    "SC_R_DMA_2_CH21",
    "SC_R_DMA_2_CH22",
    "SC_R_DMA_2_CH23",
    "SC_R_DMA_2_CH24",
    "SC_R_DMA_2_CH25",
    "SC_R_DMA_2_CH26",
    "SC_R_DMA_2_CH27",
    "SC_R_DMA_2_CH28",
    "SC_R_DMA_2_CH29",
    "SC_R_DMA_2_CH30",
    "SC_R_DMA_2_CH31",
    "SC_R_ASRC_1",
    "SC_R_ESAI_1",
    "SC_R_SAI_6",
    "SC_R_SAI_7",
    "SC_R_AMIX",
    "SC_R_MQS_0",
    "SC_R_DMA_3_CH0",
    "SC_R_DMA_3_CH1",
    "SC_R_DMA_3_CH2",
    "SC_R_DMA_3_CH3",
    "SC_R_DMA_3_CH4",
    "SC_R_DMA_3_CH5",
    "SC_R_DMA_3_CH6",
    "SC_R_DMA_3_CH7",
    "SC_R_DMA_3_CH8",
    "SC_R_DMA_3_CH9",
    "SC_R_DMA_3_CH10",
    "SC_R_DMA_3_CH11",
    "SC_R_DMA_3_CH12",
    "SC_R_DMA_3_CH13",
    "SC_R_DMA_3_CH14",
    "SC_R_DMA_3_CH15",
    "SC_R_DMA_3_CH16",
    "SC_R_DMA_3_CH17",
    "SC_R_DMA_3_CH18",
    "SC_R_DMA_3_CH19",
    "SC_R_DMA_3_CH20",
    "SC_R_DMA_3_CH21",
    "SC_R_DMA_3_CH22",
    "SC_R_DMA_3_CH23",
    "SC_R_DMA_3_CH24",
    "SC_R_DMA_3_CH25",
    "SC_R_DMA_3_CH26",
    "SC_R_DMA_3_CH27",
    "SC_R_DMA_3_CH28",
    "SC_R_DMA_3_CH29",
    "SC_R_DMA_3_CH30",
    "SC_R_DMA_3_CH31",
    "SC_R_AUDIO_PLL_1",
    "SC_R_AUDIO_CLK_0",
    "SC_R_AUDIO_CLK_1",
    "SC_R_MCLK_OUT_0",
    "SC_R_MCLK_OUT_1",
    "SC_R_PMIC_0",
    "SC_R_PMIC_1",
    "SC_R_SECO",
    "SC_R_CAAM_JR1",
    "SC_R_CAAM_JR2",
    "SC_R_CAAM_JR3",
    "SC_R_SECO_MU_2",
    "SC_R_SECO_MU_3",
    "SC_R_SECO_MU_4",
    "SC_R_HDMI_RX_PWM_0",
    "SC_R_A35",
    "SC_R_A35_0",
    "SC_R_A35_1",
    "SC_R_A35_2",
    "SC_R_A35_3",
    "SC_R_DSP",
    "SC_R_DSP_RAM",
    "SC_R_CAAM_JR1_OUT",
    "SC_R_CAAM_JR2_OUT",
    "SC_R_CAAM_JR3_OUT",
    "SC_R_VPU_DEC_0",
    "SC_R_VPU_ENC_0",
    "SC_R_CAAM_JR0",
    "SC_R_CAAM_JR0_OUT",
    "SC_R_PMIC_2",
    "SC_R_DBLOGIC",
    "SC_R_HDMI_PLL_1",
    "SC_R_BOARD_R0",
    "SC_R_BOARD_R1",
    "SC_R_BOARD_R2",
    "SC_R_BOARD_R3",
    "SC_R_BOARD_R4",
    "SC_R_BOARD_R5",
    "SC_R_BOARD_R6",
    "SC_R_BOARD_R7",
    "SC_R_MJPEG_DEC_MP",
    "SC_R_MJPEG_ENC_MP",
    "SC_R_VPU_TS_0",
    "SC_R_VPU_MU_0",
    "SC_R_VPU_MU_1",
    "SC_R_VPU_MU_2",
    "SC_R_VPU_MU_3",
    "SC_R_VPU_ENC_1",
    "SC_R_VPU",
    "SC_R_DMA_5_CH0",
    "SC_R_DMA_5_CH1",
    "SC_R_DMA_5_CH2",
    "SC_R_DMA_5_CH3",
    "SC_R_ATTESTATION",
    "SC_R_PERF",
    "SC_R_LAST"
};

#ifdef HAS_TEST_PTIM
    SC_PTIM_EXTERN(testProf);
#endif

char * sc_status2str(sc_err_t status)
{
    switch (status) {
        case SC_ERR_NONE:
            return "SC_ERR_NONE";
            break;
        case SC_ERR_VERSION:
            return "SC_ERR_VERSION";
            break;
        case SC_ERR_CONFIG:
            return "SC_ERR_CONFIG";
            break;
        case SC_ERR_PARM:
            return "SC_ERR_PARM";
            break;
        case SC_ERR_NOACCESS:
            return "SC_ERR_NOACCESS";
            break;
        case SC_ERR_LOCKED:
            return "SC_ERR_LOCKED";
            break;
        case SC_ERR_UNAVAILABLE:
            return "SC_ERR_UNAVAILABLE";
            break;
        case SC_ERR_NOTFOUND:
            return "SC_ERR_NOTFOUND";
            break;
        case SC_ERR_NOPOWER:
            return "SC_ERR_NOPOWER";
            break;
        case SC_ERR_IPC:
            return "SC_ERR_IPC";
            break;
        case SC_ERR_BUSY:
            return "SC_ERR_BUSY";
            break;
        case SC_ERR_FAIL:
            return "SC_ERR_FAIL";
            break;
        default:
            return "UNKNOWN";
            break;
    }
}

const char * sc_rsrc2str(sc_rsrc_t resource)
{
    if (resource < SC_R_LAST) {
        return sc_rsrc_str[resource];
    }
    return sc_rsrc_str[SC_R_LAST];
}
/*----------------------------------------------------------------------*/
/* RPC command/response                                                 */
/*----------------------------------------------------------------------*/
void sc_call_rpc(sc_ipc_t ipc, sc_rpc_msg_t *msg, sc_bool_t no_resp)
{
    sc_ipc_write(ipc, msg);
    if (!no_resp) {
        sc_ipc_read(ipc, msg);
    }
}

/*--------------------------------------------------------------------------*/
/* Open an IPC channel                                                      */
/*--------------------------------------------------------------------------*/
sc_err_t sc_ipc_open(sc_ipc_t *ipc, sc_ipc_id_t id)
{
    MU_Type *base = (MU_Type*) id;

    /* Get MU base associated with IPC channel */
    if ((ipc == NULL) || (base == NULL)) {
        return SC_ERR_IPC;
    }

    /* Init MU */
    MU_DisableInterrupts(base, kMU_Tx0EmptyInterruptEnable
                         | kMU_Tx1EmptyInterruptEnable | kMU_Tx2EmptyInterruptEnable
                         | kMU_Tx3EmptyInterruptEnable | kMU_Rx0FullInterruptEnable
                         | kMU_Rx1FullInterruptEnable | kMU_Rx2FullInterruptEnable
                         | kMU_Rx3FullInterruptEnable | kMU_GenInt0InterruptEnable
                         | kMU_GenInt1InterruptEnable | kMU_GenInt2InterruptEnable
                         | kMU_GenInt3InterruptEnable | kMU_GenInt0InterruptTrigger
                         | kMU_GenInt1InterruptTrigger | kMU_GenInt2InterruptTrigger
                         | kMU_GenInt3InterruptTrigger);
    MU_ClearStatusFlags(base, kMU_GenInt0Flag | kMU_GenInt1Flag
                        | kMU_GenInt2Flag | kMU_GenInt3Flag);

    /* Enable all RX interrupts */
    MU_EnableInterrupts(base, kMU_Rx0FullInterruptEnable
                        | kMU_Rx1FullInterruptEnable | kMU_Rx2FullInterruptEnable
                        | kMU_Rx3FullInterruptEnable);

    /* Return MU address as handle */
    *ipc = (sc_ipc_t) id;

    return SC_ERR_NONE;
}

/*--------------------------------------------------------------------------*/
/* Close an IPC channel                                                     */
/*--------------------------------------------------------------------------*/
void sc_ipc_close(sc_ipc_t ipc)
{
    MU_Type *base = (MU_Type*) ipc;

    if (base != NULL) {
        MU_DisableInterrupts(base, kMU_Tx0EmptyInterruptEnable
                             | kMU_Tx1EmptyInterruptEnable | kMU_Tx2EmptyInterruptEnable
                             | kMU_Tx3EmptyInterruptEnable | kMU_Rx0FullInterruptEnable
                             | kMU_Rx1FullInterruptEnable | kMU_Rx2FullInterruptEnable
                             | kMU_Rx3FullInterruptEnable | kMU_GenInt0InterruptEnable
                             | kMU_GenInt1InterruptEnable | kMU_GenInt2InterruptEnable
                             | kMU_GenInt3InterruptEnable | kMU_GenInt0InterruptTrigger
                             | kMU_GenInt1InterruptTrigger | kMU_GenInt2InterruptTrigger
                             | kMU_GenInt3InterruptTrigger);
        MU_ClearStatusFlags(base, kMU_GenInt0Flag | kMU_GenInt1Flag
                            | kMU_GenInt2Flag | kMU_GenInt3Flag);
    }
}

/*--------------------------------------------------------------------------*/
/* Read message from an IPC channel                                         */
/*--------------------------------------------------------------------------*/
void sc_ipc_read(sc_ipc_t ipc, void *data)
{
    MU_Type *base = (MU_Type*) ipc;
    sc_rpc_msg_t *msg = (sc_rpc_msg_t*) data;
    uint8_t count = 0;

    /* Check parms */
    if ((base == NULL) || (msg == NULL)) {
        return;
    }

    /* Read first word */
    *((uint32_t*)msg) = MU_ReceiveMsg(base, 0);
    count++;

    /* Check size */
    if (msg->size > SC_RPC_MAX_MSG) {
        *((uint32_t*) msg) = 0;
        return;
    }

    /* Read remaining words */
    while (count < msg->size) {
        msg->DATA.u32[count - 1] = MU_ReceiveMsg(base,
                                                 count % MU_RR_COUNT);
        count++;
    }
}

/*--------------------------------------------------------------------------*/
/* Write a message to an IPC channel                                        */
/*--------------------------------------------------------------------------*/
void sc_ipc_write(sc_ipc_t ipc, const void *data)
{
    MU_Type *base = (MU_Type*) ipc;
    sc_rpc_msg_t *msg = (sc_rpc_msg_t*) data;
    uint8_t count = 0;

    /* Check parms */
    if ((base == NULL) || (msg == NULL)) {
        return;
    }

    /* Check size */
    if (msg->size > SC_RPC_MAX_MSG) {
        return;
    }

    /* Write first word */
    MU_SendMsg(base, 0, *((uint32_t*) msg));
    count++;

    /* Write remaining words */
    while (count < msg->size) {
        MU_SendMsg(base, count % MU_TR_COUNT, msg->DATA.u32[count - 1]);
        count++;
    }
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/ipc.c $ $Rev: 904590 $")
#endif
