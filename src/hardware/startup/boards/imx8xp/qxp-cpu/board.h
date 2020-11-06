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


#ifndef BOARD_H_
#define BOARD_H_

#include <aarch64/mx8xp.h>

/** Enables initialization of peripheral clocks and pads */
#define IMX_GPT_INIT_ENABLED               1
#define IMX_ENET_INIT_ENABLED              1
#define IMX_LPUART_INIT_ENABLED            1
#define IMX_ISI_CSI_INIT_ENABLED           0
#define IMX_I2C_INIT_ENABLED               1
#define IMX_LPSPI_INIT_ENABLED             0
#define IMX_FLEXSPI_INIT_ENABLED           1
#define IMX_USDHC_INIT_ENABLED             1
#define IMX_USB_INIT_ENABLED               1
#define IMX_DC_INIT_ENABLED                1
#define IMX_AUDIO_INIT_ENABLED             1
#define IMX_FLEXCAN_INIT_ENABLED           0
#define IMX_GPIO_INIT_ENABLED              1
#define IMX_PCIE_INIT_ENABLED              1
#define IMX_MIPI_CSI_INIT_ENABLED          0
#define IMX_PARALLEL_CSI_INIT_ENABLED      0
#define IMX_GPU_INIT_ENABLED               1
#define IMX_ENET_GET_MAC_ENABLED           1
#define IMX_PCA9557_INIT_ENABLED           0
#define IMX_PCA6416_INIT_ENABLED           0
#define IMX_DMA_INIT_ENABLED               1
#define IMX_VPU_INIT_ENABLED               1
#define IMX_DSP_INIT_ENABLED               1
/* Disabled by default. Conflicts with audio pads routing. */
#define IMX_PARALLEL_LCD_INIT_ENABLED      0

/** Enables ARM Trusted Firmware support */
#define IMX_ARM_TRUSTED_FIRMWARE_ENABLED

/** Message Unit used for SCFW IPC calls */
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
    #define IMX_SCU_IPC_MU                 IMX_MU1_BASE
#else
    #define IMX_SCU_IPC_MU                 IMX_MU0_BASE
#endif

/** i.MX8QXP MCU cores number */
#define IMX_QXP_MCU_CORES_NUMBER           4
/** i.MX8DXx MCU cores number */
#define IMX_DXx_MCU_CORES_NUMBER           2

/** SDRAM memory information */
#define IMX_SDRAM0_BASE                    0x80000000
#define IMX_SDRAM0_SIZE                    2048
#define IMX_SDRAM1_BASE                    0x880000000

/** Comment for disable memory address above 4GB */
#define IMX_SDRAM1_SIZE                    1024
/** Startup MMU configuration */
#define STARTUP_SDRAM_BASE                 IMX_SDRAM0_BASE
#define STARTUP_SDRAM_SIZE                 IMX_SDRAM0_SIZE

/** Base address for GPT driver */
#define IMX_GPT_DRV_BASE                   IMX_GPT1_BASE
/** GPT imx timer information (in MHz) */
#define IMX_GPT_CLOCK_FREQ                 24000000UL

/** Core counter input clock (in MHz) */
#define IMX_CNTV_CLOCK_FREQ                8000000UL

/**
 * Board information:
 * -n = board name.
 */
#define IMX_BOARD_INFO                     "-n cpu"
/** I2C expander information */
#define PORT_EXP_RST_GPIO                  IMX_GPIO1_BASE
#define PORT_EXP_RST_PIN                   1

#define I2C_SWITCH_ADDR                    0x71
#define EXP_A_ADDR                         0x1A
#define EXP_B_ADDR                         0x1D
#define EXP_CAN_ADDR                       0x20

/** PCA9646 channel configuration */
#define I2C_SWITCH_CONTROL_SCL_DIRECTION   0x80
#define I2C_SWITCH_CONTROL_B3              0x08
#define I2C_SWITCH_CONTROL_B2              0x04
#define I2C_SWITCH_CONTROL_B1              0x02
#define I2C_SWITCH_CONTROL_B0              0x01

/** PCA9557 registers */
#define EXP_INPUT_PORT_REGISTER            0x00
#define EXP_OUTPUT_PORT_REGISTER           0x01
#define EXP_POLARITY_INVERSION_REGISTER    0x02
#define EXP_CONFIGURATION_REGISTER         0x03

/* PCA6416 registers */
#define EXP_INPUT_PORT_0_REGISTER               0x00
#define EXP_INPUT_PORT_1_REGISTER               0x01
#define EXP_OUTPUT_PORT_0_REGISTER              0x02
#define EXP_OUTPUT_PORT_1_REGISTER              0x03
#define EXP_POLARITY_INVERSION_PORT_0_REGISTER  0x04
#define EXP_POLARITY_INVERSION_PORT_1_REGISTER  0x05
#define EXP_CONFIGURATION_PORT_0_REGISTER       0x06
#define EXP_CONFIGURATION_PORT_1_REGISTER       0x07

/* Expander A output pins */
#define EXP_A_USB_TYPEC_EN                 0x80
#define EXP_A_MIPI_DSI0_EN                 0x40
#define EXP_A_SENS_RST_B                   0x20
#define EXP_A_CPU_ENET_PHY_RST_B           0x10
#define EXP_A_PER_RST_B                    0x08
#define EXP_A_WIFI_DISABLE_B               0x04
#define EXP_A_BT_DISABLE_B                 0x02
#define EXP_A_WIFI_EN                      0x01

/** Expander B output pins */
#define EXP_B_MIPI_DSI1_EN                 0x80
#define EXP_B_BB_GPIO_EXP3                 0x40
#define EXP_B_BB_GPIO_EXP2                 0x20
#define EXP_B_BB_GPIO_EXP1                 0x10
#define EXP_B_BB_ARD_MIK_RST_B             0x08
#define EXP_B_BB_USB_OTG1_PWR_ON           0x04
#define EXP_B_BB_AUDIN_RST_B               0x02
#define EXP_B_TP36                         0x01

/* IO CAN Expander output pins - port0 - Base Board */
#define BB_CAN_EXP_MLB_PWDN                0x80
#define BB_CAN_EXP_CAN2_STB_B              0x40
#define BB_CAN_EXP_CAN01_STB_B             0x20
#define BB_CAN_EXP_CAN2_EN                 0x10
#define BB_CAN_EXP_CAN01_EN                0x08
#define BB_CAN_EXP_CAN2_WAKE_3V3_B         0x04
#define BB_CAN_EXP_CAN1_WAKE_3V3_B         0x02
#define BB_CAN_EXP_CAN0_WAKE_3V3_B         0x01

/* IO CAN Expander output pins - port1 - Base Board */
#define BB_CAN_EXP_QM_MERC_EN              0x80
#define BB_CAN_EXP_AUD_SW1_SEL             0x40
#define BB_CAN_EXP_CAN2_INH3V3             0x20
#define BB_CAN_EXP_CAN1_INH3V3             0x10
#define BB_CAN_EXP_CAN0_INH3V3             0x08
#define BB_CAN_EXP_CAN2_ERR_B              0x04
#define BB_CAN_EXP_CAN1_ERR_B              0x02
#define BB_CAN_EXP_CAN0_ERR_B              0x01

#endif


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
#ifdef __ASM__
__SRCVERSION "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/qxp-cpu/board.h $ $Rev: 893864 $"
#else
__SRCVERSION( "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/qxp-cpu/board.h $ $Rev: 893864 $" )
#endif
#endif
