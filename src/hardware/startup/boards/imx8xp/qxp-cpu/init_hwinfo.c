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
#include "hwinfo_private.h"
#include <hw/hwinfo_imx8x.h>
#include <drvr/hwinfo.h>                /* For hwi support routines in libdrvr */
#include <aarch64/mx8xp.h>
#include "board.h"
#include "qxp_cpu.h"
#include "imx_startup.h"
#include <aarch64/imx8_common/imx_edma_requests.h>
#include <aarch64/imx8_common/imx_flexcan.h>

/* instead of hwibus_add_can() function use the following function to avoid empty tag creation
 * that happens inside hwibus_add_can(), as facing difficulty to fill up that empty tag
 */
static unsigned imx8x_hwibus_add_can(unsigned parent_hwi_off, hwiattr_can_t *attr);

/**
 * Add bus device to the hwinfo table.
 *
 * @param dev     Device name in lowercase to substitute ##dev## eg. i2c
 * @param DEV     Device name in uppercase to substitute ##DEV## eg. I2C
 * @param base    Hardware base address
 * @param size    Size of device address space
 * @param irq     Interrupt vector number
 * @param clock   Clock frequency in Hz
 * @param divider Clock divider
 */
#define IMX_HWI_ADD_BUS(dev, DEV, base, size, irq, clock, divider)       {\
                                            unsigned hwi_off;\
                                            hwiattr_##dev##_t attr = HWIATTR_##DEV##_T_INITIALIZER;\
                                            struct hwi_inputclk clksrc = {.clk = clock, .div = divider};\
                                            HWIATTR_##DEV##_SET_NUM_CLK(&attr, 1);\
                                            if (irq!=-1) {\
                                                HWIATTR_##DEV##_SET_NUM_IRQ(&attr, 1);\
                                            } else {\
                                                HWIATTR_##DEV##_SET_NUM_IRQ(&attr, 0);\
                                            }\
                                            HWIATTR_##DEV##_SET_LOCATION(&attr, base, size, 0, hwi_find_as(base, 1));\
                                            hwi_off = hwibus_add_##dev(hwi_bus_internal, &attr);\
                                            ASSERT(hwi_off != HWI_NULL_OFF);\
                                            if (irq!=-1) {\
                                                hwitag_set_avail_ivec(hwi_off, 0, irq);\
                                            }\
                                            hwitag_set_inputclk(hwi_off, 0, &clksrc);\
                                        }

#define IMX_HWI_ADD_DEV(dev,DEV,base,size,irq)       {\
                                            unsigned hwi_off;\
                                            hwiattr_##dev##_t attr = HWIATTR_##DEV##_T_INITIALIZER;\
                                            if (irq!=-1) {\
                                                HWIATTR_##DEV##_SET_NUM_IRQ(&attr, 1);\
                                            } else {\
                                                HWIATTR_##DEV##_SET_NUM_IRQ(&attr, 0);\
                                            }\
                                            HWIATTR_##DEV##_SET_LOCATION(&attr, base, size, 0, hwi_find_as(base, 1));\
                                            hwi_off = hwidev_add_##dev(IMX_HWI_##DEV, &attr,hwi_bus_internal);\
                                            ASSERT(hwi_off != HWI_NULL_OFF);\
                                            if (irq!=-1) {\
                                                hwitag_set_avail_ivec(hwi_off, 0, irq);\
                                            }\
                                        }

#if IMX_AUDIO_INIT_ENABLED
/**
 * Add common device to the hwinfo table.
 *
 * @param device_name Name of a device used for find the device in hwinfo table
 * @param base        Hardware base address
 * @param size        Size of device address space
 * @param irq         Pointer to a variable or array with interrupt vector numbers
 * @param irq_count   Number of interrupt vectors in array
 * @param dma         Pointer to a variable or array with DMA requests
 * @param dma_count   Number of DMA requests in array
 *
 * @return            Always returns 0. Reserved for future use.
 */
static int imx_add_common_device(const char *device_name, paddr_t base, uint32_t size, uint32_t *irq, uint32_t irq_count,
                          imx_edma_request_source_t *dma, uint32_t dma_count, uint32_t * errata, uint32_t errata_count)
{
    unsigned hwi_off;
    unsigned hwi_bus_internal = 0;
    unsigned i;
    hwiattr_common_t attr = HWIATTR_COMMON_INITIALIZER;
    HWIATTR_SET_NUM_IRQ(&attr, irq_count);
    HWIATTR_SET_NUM_DMA(&attr, dma_count);
    HWIATTR_SET_NUM_ERRATA(&attr, errata_count);
    HWIATTR_SET_LOCATION(&attr, base, size, 0, hwi_find_as(base, 1));
    /* Create a device with device_name */
    hwi_off = hwidev_add(device_name, 0, hwi_bus_internal);
    ASSERT(hwi_off != HWI_NULL_OFF);
    /* Assign attribute structure */
    hwitag_add_common(hwi_off, &attr);
    /* Assign IRQ numbers */
    for (i = 0; i < irq_count; i++) {
        if (hwitag_set_ivec(hwi_off, i, irq[i]) == 0) {
            crash("%s hwitag_set_ivec failed for IRQ index %u\n", device_name, i);
        }
    }
    /* Assign DMA requests */
    for (i = 0; i < dma_count; i++) {
        if (hwitag_set_dma(hwi_off, i, dma[i]) == 0) {
            crash("%s hwitag_set_dma failed for DMA index %u\n", device_name, i);
        }
    }
    /* Assign ERRATA */
    for (i = 0; i < errata_count; i++) {
        if (hwitag_set_errata(hwi_off, i, errata[i]) == 0) {
            crash("%s hwitag_set_errata failed for index %u\n", device_name, i);
        }
    }
    return 0;
}
#endif

/**
 * Add imx8 devices information (e.g.: base address, interrupt vector,...)
 * to the hardware info section of the syspage.
 *
 * @param startup_data Pointer to the startup data.
 */
void imx_init_hwinfo(imx_startup_data_t * startup_data)
{
    unsigned hwi_bus_internal = 0;

    /* Add  LPUART0 */
    {
        unsigned hwi_off;
        hwiattr_uart_t attr = HWIATTR_UART_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_uart_clock[0], .div = 1};
        HWIATTR_UART_SET_NUM_IRQ(&attr, 1);
        HWIATTR_UART_SET_NUM_CLK(&attr, 1);
        HWIATTR_UART_SET_NUM_DMA(&attr, 2);
        /* Create LPUART0 */
        HWIATTR_UART_SET_LOCATION(&attr, IMX_LPUART0_BASE, IMX_LPUART_SIZE, 0, hwi_find_as(IMX_LPUART0_BASE, 1));
        hwi_off = hwidev_add_uart(IMX_HWI_UART, &attr, hwi_bus_internal);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_LPUART0_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwitag_set_dma(hwi_off, 0, IMX_DMA_REQ_LPUART0_RX_MX8XP);
        hwitag_set_dma(hwi_off, 1, IMX_DMA_REQ_LPUART0_TX_MX8XP);
    }
    /* Add  LPUART1 */
    {
        unsigned hwi_off;
        hwiattr_uart_t attr = HWIATTR_UART_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_uart_clock[1], .div = 1};
        HWIATTR_UART_SET_NUM_IRQ(&attr, 1);
        HWIATTR_UART_SET_NUM_CLK(&attr, 1);
        HWIATTR_UART_SET_NUM_DMA(&attr, 2);
        /* Create LPUART1 */
        HWIATTR_UART_SET_LOCATION(&attr, IMX_LPUART1_BASE, IMX_LPUART_SIZE, 0, hwi_find_as(IMX_LPUART1_BASE, 1));
        hwi_off = hwidev_add_uart(IMX_HWI_UART, &attr, hwi_bus_internal);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_LPUART1_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwitag_set_dma(hwi_off, 0, IMX_DMA_REQ_LPUART1_RX_MX8XP);
        hwitag_set_dma(hwi_off, 1, IMX_DMA_REQ_LPUART1_TX_MX8XP);
    }
    /* Add  LPUART2 */
    {
        unsigned hwi_off;
        hwiattr_uart_t attr = HWIATTR_UART_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_uart_clock[2], .div = 1};
        HWIATTR_UART_SET_NUM_IRQ(&attr, 1);
        HWIATTR_UART_SET_NUM_CLK(&attr, 1);
        HWIATTR_UART_SET_NUM_DMA(&attr, 2);
        /* Create LPUART2 */
        HWIATTR_UART_SET_LOCATION(&attr, IMX_LPUART2_BASE, IMX_LPUART_SIZE, 0, hwi_find_as(IMX_LPUART2_BASE, 1));
        hwi_off = hwidev_add_uart(IMX_HWI_UART, &attr, hwi_bus_internal);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_LPUART2_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwitag_set_dma(hwi_off, 0, IMX_DMA_REQ_LPUART2_RX_MX8XP);
        hwitag_set_dma(hwi_off, 1, IMX_DMA_REQ_LPUART2_TX_MX8XP);
    }
    /* Add  LPUART3 */
    {
        unsigned hwi_off;
        hwiattr_uart_t attr = HWIATTR_UART_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_uart_clock[3], .div = 1};
        HWIATTR_UART_SET_NUM_IRQ(&attr, 1);
        HWIATTR_UART_SET_NUM_CLK(&attr, 1);
        HWIATTR_UART_SET_NUM_DMA(&attr, 2);
        /* Create LPUART3 */
        HWIATTR_UART_SET_LOCATION(&attr, IMX_LPUART3_BASE, IMX_LPUART_SIZE, 0, hwi_find_as(IMX_LPUART3_BASE, 1));
        hwi_off = hwidev_add_uart(IMX_HWI_UART, &attr, hwi_bus_internal);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_LPUART3_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwitag_set_dma(hwi_off, 0, IMX_DMA_REQ_LPUART3_RX_MX8XP);
        hwitag_set_dma(hwi_off, 1, IMX_DMA_REQ_LPUART3_TX_MX8XP);
    }

    /* Add  LPSPI0 */
    {
        unsigned hwi_off;
        hwiattr_spi_t attr = HWIATTR_SPI_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_spi_clk[0], .div = 1};
        HWIATTR_SPI_SET_NUM_IRQ(&attr, 1);
        HWIATTR_SPI_SET_NUM_CLK(&attr, 1);
        /* Create SPI0 */
        HWIATTR_SPI_SET_LOCATION(&attr, IMX_LPSPI0_BASE, IMX_LPSPI_SIZE, 0, hwi_find_as(IMX_LPSPI0_BASE, 1));
        hwi_off = hwibus_add_spi(hwi_bus_internal, &attr);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_LPSPI0_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);

        hwi_add_synonym(hwi_off, "spi0");
    }
    /* Add  LPSPI1 */
    {
        unsigned hwi_off;
        hwiattr_spi_t attr = HWIATTR_SPI_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_spi_clk[1], .div = 1};
        HWIATTR_SPI_SET_NUM_IRQ(&attr, 1);
        HWIATTR_SPI_SET_NUM_CLK(&attr, 1);
        /* Create SPI1 */
        HWIATTR_SPI_SET_LOCATION(&attr, IMX_LPSPI1_BASE, IMX_LPSPI_SIZE, 0, hwi_find_as(IMX_LPSPI1_BASE, 1));
        hwi_off = hwibus_add_spi(hwi_bus_internal, &attr);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_LPSPI1_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwi_add_synonym(hwi_off, "spi1");
    }
    /* Add  LPSPI2 */
    {
        unsigned hwi_off;
        hwiattr_spi_t attr = HWIATTR_SPI_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_spi_clk[2], .div = 1};
        HWIATTR_SPI_SET_NUM_IRQ(&attr, 1);
        HWIATTR_SPI_SET_NUM_CLK(&attr, 1);
        /* Create SPI2 */
        HWIATTR_SPI_SET_LOCATION(&attr, IMX_LPSPI2_BASE, IMX_LPSPI_SIZE, 0, hwi_find_as(IMX_LPSPI2_BASE, 1));
        hwi_off = hwibus_add_spi(hwi_bus_internal, &attr);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_LPSPI2_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwi_add_synonym(hwi_off, "spi2");
    }
    /* Add  LPSPI3 */
    {
        unsigned hwi_off;
        hwiattr_spi_t attr = HWIATTR_SPI_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_spi_clk[3], .div = 1};
        HWIATTR_SPI_SET_NUM_IRQ(&attr, 1);
        HWIATTR_SPI_SET_NUM_CLK(&attr, 1);
        /* Create SPI3 */
        HWIATTR_SPI_SET_LOCATION(&attr, IMX_LPSPI3_BASE, IMX_LPSPI_SIZE, 0, hwi_find_as(IMX_LPSPI3_BASE, 1));
        hwi_off = hwibus_add_spi(hwi_bus_internal, &attr);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_LPSPI3_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwi_add_synonym(hwi_off, "spi3");
    }

    /* Add FEC (ENET0 & ENET1 peripherals) */
    {
        unsigned hwi_off;
        hwiattr_enet_t attr = HWIATTR_ENET_T_INITIALIZER;
        HWIATTR_ENET_SET_NUM_IRQ(&attr, 1);

        /* Create fec0 */
        HWIATTR_ENET_SET_LOCATION(&attr, IMX_ENET0_BASE, IMX_ENET0_MEM_SIZE, 0, hwi_find_as(IMX_ENET0_BASE, 1));
        hwi_off = hwidev_add_enet(IMX_HWI_LEGACY_FEC, &attr, hwi_bus_internal);
        ASSERT(hwi_find_unit(hwi_off) == 0);
        hwitag_set_avail_ivec(hwi_off, 0, IMX_ENET0_IRQ); /* Add ENET0 irq number */
        hwitag_add_nicphyaddr(hwi_off, 0);
        {
            hwi_tag *tag_gpt, *tag_gpt_irq;
            unsigned hwi_off_gpt;

            hwi_off_gpt = hwidev_add("fec0_gpt", 0, hwi_bus_internal);
            ASSERT(hwi_off_gpt != HWI_NULL_OFF);
            if (hwi_off_gpt != HWI_NULL_OFF) {
                tag_gpt = hwi_alloc_tag(HWI_TAG_INFO(location));
                tag_gpt->location.base = IMX_GPT1_BASE;
                tag_gpt->location.len = IMX_GPT_SIZE;
                tag_gpt->location.regshift = 0;
                tag_gpt->location.addrspace = hwi_find_as(IMX_GPT1_BASE, 1);
                tag_gpt_irq = hwi_alloc_tag(HWI_TAG_INFO(irq));
                tag_gpt_irq->irq.vector = IMX_GPT1_IRQ;
            }
        }

        /* Create fec1 */
        HWIATTR_ENET_SET_LOCATION(&attr, IMX_ENET1_BASE, IMX_ENET1_MEM_SIZE, 0, hwi_find_as(IMX_ENET1_BASE, 1));
        hwi_off = hwidev_add_enet(IMX_HWI_LEGACY_FEC, &attr, hwi_bus_internal);
        ASSERT(hwi_find_unit(hwi_off) == 1);
        hwitag_set_avail_ivec(hwi_off, 0, IMX_ENET1_IRQ); /* Add ENET1 irq number */
        hwitag_add_location(hwi_off, IMX_ENET0_BASE, IMX_ENET0_MEM_SIZE, 0, 0); /* Add memory address of MDC/MDIO registers associated with the second PHY */
        hwitag_add_nicphyaddr(hwi_off, 1);
        {
            hwi_tag *tag_gpt, *tag_gpt_irq;
            unsigned hwi_off_gpt;

            hwi_off_gpt = hwidev_add("fec0_gpt", 0, hwi_bus_internal);
            ASSERT(hwi_off_gpt != HWI_NULL_OFF);
            if (hwi_off_gpt != HWI_NULL_OFF) {
                tag_gpt = hwi_alloc_tag(HWI_TAG_INFO(location));
                tag_gpt->location.base = IMX_GPT2_BASE;
                tag_gpt->location.len = IMX_GPT_SIZE;
                tag_gpt->location.regshift = 0;
                tag_gpt->location.addrspace = hwi_find_as(IMX_GPT2_BASE, 1);
                tag_gpt_irq = hwi_alloc_tag(HWI_TAG_INFO(irq));
                tag_gpt_irq->irq.vector = IMX_GPT2_IRQ;
            }
        }
    }

#if IMX_I2C_INIT_ENABLED
    /* Add  LPI2C */
    {
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_LSIO_I2C0_BASE,      IMX_LPI2C_SIZE, IMX_LSIO_I2C0_IRQ,      startup_data->imx_i2c_clock[0], 1); /* I2C0 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_LSIO_I2C1_BASE,      IMX_LPI2C_SIZE, IMX_LSIO_I2C1_IRQ,      startup_data->imx_i2c_clock[1], 1); /* I2C1 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_LSIO_I2C2_BASE,      IMX_LPI2C_SIZE, IMX_LSIO_I2C2_IRQ,      startup_data->imx_i2c_clock[2], 1); /* I2C2 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_LSIO_I2C3_BASE,      IMX_LPI2C_SIZE, IMX_LSIO_I2C3_IRQ,      startup_data->imx_i2c_clock[3], 1); /* I2C3 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_DC0_MIPI1_I2C0_BASE, IMX_LPI2C_SIZE, IMX_DC0_MIPI1_I2C0_IRQ, startup_data->imx_i2c_clock[4], 1); /* I2C4 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_DC0_MIPI1_I2C1_BASE, IMX_LPI2C_SIZE, IMX_DC0_MIPI1_I2C1_IRQ, startup_data->imx_i2c_clock[5], 1); /* I2C5 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_MIPI_CSI0_I2C0_BASE, IMX_LPI2C_SIZE, IMX_MIPI_CSI0_I2C0_IRQ, startup_data->imx_i2c_clock[6], 1); /* I2C6 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_DC0_MIPI0_I2C0_BASE, IMX_LPI2C_SIZE, IMX_DC0_MIPI0_I2C0_IRQ, startup_data->imx_i2c_clock[7], 1); /* I2C7 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_DC0_MIPI0_I2C1_BASE, IMX_LPI2C_SIZE, IMX_DC0_MIPI0_I2C1_IRQ, startup_data->imx_i2c_clock[8], 1); /* I2C8 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_PARALEL_I2C0_BASE,   IMX_LPI2C_SIZE, IMX_PARALEL_I2C0_IRQ,   startup_data->imx_i2c_clock[9], 1); /* I2C9 */
        IMX_HWI_ADD_BUS(i2c, I2C, IMX_M40_I2C0_BASE,       IMX_LPI2C_SIZE, IMX_M40_I2C0_IRQ,       startup_data->imx_i2c_clock[10], 1); /* I2C10 */
    }
#endif

#if IMX_AUDIO_INIT_ENABLED
    /* Add ASRC0 */
    {
        uint32_t irq = IMX_ASRC0_IRQ;
        imx_edma_request_source_t dma[] = { IMX_DMA_REQ_ASRC0_PAIR_A_INPUT,
                                            IMX_DMA_REQ_ASRC0_PAIR_A_OUTPUT,
                                            IMX_DMA_REQ_ASRC0_PAIR_B_INPUT,
                                            IMX_DMA_REQ_ASRC0_PAIR_B_OUTPUT,
                                            IMX_DMA_REQ_ASRC0_PAIR_C_INPUT,
                                            IMX_DMA_REQ_ASRC0_PAIR_C_OUTPUT,
                                          };
        imx_add_common_device(IMX_HWI_ASRC, IMX_ASRC0_BASE, IMX_ASRC_SIZE, &irq, 1, dma,
                              sizeof(dma) / sizeof(imx_edma_request_source_t), NULL, 0);
    }
    /* Add ASRC1 */
    {
        uint32_t irq = IMX_ASRC1_IRQ;
        imx_edma_request_source_t dma[] = { IMX_DMA_REQ_ASRC1_PAIR_A_INPUT,
                                            IMX_DMA_REQ_ASRC1_PAIR_A_OUTPUT,
                                            IMX_DMA_REQ_ASRC1_PAIR_B_INPUT,
                                            IMX_DMA_REQ_ASRC1_PAIR_B_OUTPUT,
                                            IMX_DMA_REQ_ASRC1_PAIR_C_INPUT,
                                            IMX_DMA_REQ_ASRC1_PAIR_C_OUTPUT,
                                          };
        imx_add_common_device(IMX_HWI_ASRC, IMX_ASRC1_BASE, IMX_ASRC_SIZE, &irq, 1, dma,
                              sizeof(dma) / sizeof(imx_edma_request_source_t), NULL, 0);
    }
    /* Add SAI0 */
    {
        uint32_t irq = IMX_SAI0_IRQ;
        imx_edma_request_source_t dma[] = { IMX_DMA_REQ_SAI0_RX,
                                            IMX_DMA_REQ_SAI0_TX,
                                          };
        imx_add_common_device(IMX_HWI_SAI, IMX_SAI0_BASE, IMX_SAI_SIZE, &irq, 1, dma,
                              sizeof(dma) / sizeof(imx_edma_request_source_t), NULL, 0);
    }
    /* Add SAI1 */
    {
        uint32_t irq = IMX_SAI1_IRQ;
        imx_edma_request_source_t dma[] = { IMX_DMA_REQ_SAI1_RX,
                                            IMX_DMA_REQ_SAI1_TX,
                                          };
        imx_add_common_device(IMX_HWI_SAI, IMX_SAI1_BASE, IMX_SAI_SIZE, &irq, 1, dma,
                              sizeof(dma) / sizeof(imx_edma_request_source_t), NULL, 0);
    }
    /* Add SAI2 */
    {
        uint32_t irq = IMX_SAI2_IRQ;
        imx_edma_request_source_t dma[] = { IMX_DMA_REQ_SAI2_RX,
                                            IMX_DMA_REQ_NONE
                                          };
        imx_add_common_device(IMX_HWI_SAI, IMX_SAI2_BASE, IMX_SAI_SIZE, &irq, 1, dma,
                              sizeof(dma) / sizeof(imx_edma_request_source_t), NULL, 0);
    }
    /* Add SAI3 */
    {
        uint32_t irq = IMX_SAI3_IRQ;
        imx_edma_request_source_t dma[] = { IMX_DMA_REQ_SAI3_RX,
                                            IMX_DMA_REQ_NONE
                                          };
        imx_add_common_device(IMX_HWI_SAI, IMX_SAI3_BASE, IMX_SAI_SIZE, &irq, 1, dma,
                              sizeof(dma) / sizeof(imx_edma_request_source_t), NULL, 0);
    }
    /* Add SAI4 */
    {
        uint32_t irq = IMX_SAI4_IRQ;
        imx_edma_request_source_t dma[] = { IMX_DMA_REQ_SAI4_RX_MX8XP,
                                            IMX_DMA_REQ_SAI4_TX_MX8XP
                                          };
        imx_add_common_device(IMX_HWI_SAI, IMX_SAI4_BASE, IMX_SAI_SIZE, &irq, 1, dma,
                              sizeof(dma) / sizeof(imx_edma_request_source_t), NULL, 0);
    }
    /* Add SAI5 */
    {
        uint32_t irq = IMX_SAI5_IRQ;
        imx_edma_request_source_t dma[] = { IMX_DMA_REQ_NONE,
                                            IMX_DMA_REQ_SAI5_TX_MX8XP
                                          };
        imx_add_common_device(IMX_HWI_SAI, IMX_SAI5_BASE, IMX_SAI_SIZE, &irq, 1, dma,
                              sizeof(dma) / sizeof(imx_edma_request_source_t), NULL, 0);
    }
    /* Add ESAI0 */
    {
        uint32_t irq = IMX_ESAI0_IRQ;
        imx_edma_request_source_t dma[] = { IMX_DMA_REQ_ESAI0_RX,
                                            IMX_DMA_REQ_ESAI0_TX
                                          };
        imx_add_common_device(IMX_HWI_ESAI, IMX_ESAI0_BASE, IMX_ESAI_SIZE, &irq, 1, dma,
                              sizeof(dma) / sizeof(imx_edma_request_source_t), NULL, 0);
    }
#endif

    /* Add the WATCHDOG device */
    {
        unsigned hwi_off;
        hwiattr_timer_t attr = HWIATTR_TIMER_T_INITIALIZER;
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
        const char *optstr = "smc_call=yes";
#else
        const char *optstr = NULL;
#endif
        HWIATTR_TIMER_SET_NUM_CLK(&attr, 1);
        HWIATTR_SET_OPTSTR(&attr.common, optstr);
        hwi_off = hwidev_add_timer(IMX_HWI_WDOG, &attr, HWI_NULL_OFF );
        ASSERT(hwi_off != HWI_NULL_OFF);
    }

/* Add  USDHC0 */
    {
        unsigned hwi_off;
        hwiattr_sdio_t attr = HWIATTR_SDIO_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_usdhc_clk[0], .div = 1};
        HWIATTR_SDIO_SET_NUM_IRQ(&attr, 1);
        HWIATTR_SDIO_SET_NUM_CLK(&attr, 1);
        HWIATTR_SDIO_SET_DLL(&attr, "imx");
        /* Create USDHC0 */
        HWIATTR_SDIO_SET_LOCATION(&attr, IMX_USDHC0_BASE, IMX_USDHC_SIZE, 0, hwi_find_as(IMX_USDHC0_BASE, 1));
        hwi_off = hwibus_add_sdio(hwi_bus_internal, &attr);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_USDHC0_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwi_add_synonym(hwi_off, "usdhc0");
    }
    /* Add  USDHC1 */
    {
        unsigned hwi_off;
        hwiattr_sdio_t attr = HWIATTR_SDIO_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_usdhc_clk[1], .div = 1};
        HWIATTR_SDIO_SET_NUM_IRQ(&attr, 1);
        HWIATTR_SDIO_SET_NUM_CLK(&attr, 1);
        HWIATTR_SDIO_SET_DLL(&attr, "imx");
        /* Create USDHC1 */
        HWIATTR_SDIO_SET_LOCATION(&attr, IMX_USDHC1_BASE, IMX_USDHC_SIZE, 0, hwi_find_as(IMX_USDHC1_BASE, 1));
        hwi_off = hwibus_add_sdio(hwi_bus_internal, &attr);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_USDHC1_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwi_add_synonym(hwi_off, "usdhc1");
    }
    /* Add  USDHC2 */
    {
        unsigned hwi_off;
        hwiattr_sdio_t attr = HWIATTR_SDIO_T_INITIALIZER;
        struct hwi_inputclk clksrc = {.clk = startup_data->imx_usdhc_clk[2], .div = 1};
        HWIATTR_SDIO_SET_NUM_IRQ(&attr, 1);
        HWIATTR_SDIO_SET_NUM_CLK(&attr, 1);
        HWIATTR_SDIO_SET_DLL(&attr, "imx");
        /* Create USDHC2 */
        HWIATTR_SDIO_SET_LOCATION(&attr, IMX_USDHC2_BASE, IMX_USDHC_SIZE, 0, hwi_find_as(IMX_USDHC2_BASE, 1));
        hwi_off = hwibus_add_sdio(hwi_bus_internal, &attr);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_set_ivec(hwi_off, 0, IMX_USDHC2_IRQ);
        hwitag_set_inputclk(hwi_off, 0, &clksrc);
        hwi_add_synonym(hwi_off, "usdhc2");
    }
    /* Add information for SC driver */
    {
        unsigned hwi_off;
        hwiattr_common_t attr = HWIATTR_COMMON_INITIALIZER;
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
        paddr_t sc_mu_base = IMX_MU1_BASE;
        uint32_t sc_mu_vector = IMX_MU1_IRQ;
        const char *optstr = "smc_call=yes";
#else
        paddr_t sc_mu_base = IMX_MU0_BASE;
        uint32_t sc_mu_vector = IMX_MU0_IRQ;
        const char *optstr = "smc_call=no";
#endif
        HWIATTR_SET_NUM_IRQ(&attr, 1);
        /* Create SC */
        HWIATTR_SET_LOCATION(&attr, sc_mu_base, IMX_MU_SIZE, 0, hwi_find_as(sc_mu_base, 1));
        hwi_off = hwidev_add("sc", hwi_devclass_NONE, hwi_bus_internal);
        ASSERT(hwi_off != HWI_NULL_OFF);
        hwitag_add_common(hwi_off, &attr);
        hwitag_add_optstr(hwi_off, optstr);
        hwitag_set_ivec(hwi_off, 0, sc_mu_vector);
    }
    /* add imx8 board and type and silicon version */
    {
        hwi_tag *tag;
        unsigned off;
        unsigned hwi_off;

        hwi_off = hwidev_add("board", 0, 0);
        ASSERT(hwi_off != HWI_NULL_OFF);
        if (hwi_off != HWI_NULL_OFF) {
            off = add_string(IMX_BOARD_INFO);
            tag = hwi_alloc_tag(HWI_TAG_INFO(hwversion));
            tag->hwversion.hclass = (_Uint8t)(startup_data->chip_type); /* hclass = imx chip type */
            tag->hwversion.version = (_Uint8t)(startup_data->chip_rev); /* version = imx chip version */
            tag->hwversion.name = (_Uint16t)off;
        }
    }
#if IMX_FLEXCAN_INIT_ENABLED
    /* Add CAN devices */
    {
        unsigned hwi_off;
        hwiattr_can_t attr = HWIATTR_CAN_T_INITIALIZER;
        HWIATTR_CAN_SET_NUM_IRQ(&attr, 1);
        HWIATTR_CAN_SET_NUM_MEMADDR(&attr, 1);

        /* create CAN0 */
        HWIATTR_CAN_SET_LOCATION(&attr, IMX_FLEXCAN0_REG_BASE, IMX_FLEXCAN_REG_SIZE, 0, hwi_find_as(IMX_FLEXCAN0_REG_BASE, 1));
        hwi_off = imx8x_hwibus_add_can(hwi_bus_internal, &attr);
        hwitag_add_location(hwi_off, IMX_FLEXCAN0_MEM_BASE, IMX_FLEXCAN_MEM_SIZE, 0, 0);
        ASSERT(hwi_find_unit(hwi_off) == 0);
        hwitag_set_ivec(hwi_off, 0, IMX_FLEXCAN0_IRQ);

        /* create CAN1 */
        HWIATTR_CAN_SET_LOCATION(&attr, IMX_FLEXCAN1_REG_BASE, IMX_FLEXCAN_REG_SIZE, 0, hwi_find_as(IMX_FLEXCAN1_REG_BASE, 1));
        hwi_off = imx8x_hwibus_add_can(hwi_bus_internal, &attr);
        hwitag_add_location(hwi_off, IMX_FLEXCAN1_MEM_BASE, IMX_FLEXCAN_MEM_SIZE, 0, 0);
        ASSERT(hwi_find_unit(hwi_off) == 1);
        hwitag_set_ivec(hwi_off, 0, IMX_FLEXCAN1_IRQ);

        /* create CAN2 */
        HWIATTR_CAN_SET_LOCATION(&attr, IMX_FLEXCAN2_REG_BASE, IMX_FLEXCAN_REG_SIZE, 0, hwi_find_as(IMX_FLEXCAN2_REG_BASE, 1));
        hwi_off = imx8x_hwibus_add_can(hwi_bus_internal, &attr);
        hwitag_add_location(hwi_off, IMX_FLEXCAN2_MEM_BASE, IMX_FLEXCAN_MEM_SIZE, 0, 0);
        ASSERT(hwi_find_unit(hwi_off) == 2);
        hwitag_set_ivec(hwi_off, 0, IMX_FLEXCAN2_IRQ);
    }
#endif
#if IMX_DMA_INIT_ENABLED
    {
        /* Add eDMA periphs */
        {
            uint32_t errata =  IMX_DMA0_ERRATA;
            uint32_t irq[] = { IMX_DMA0_IRQ,
                               IMX_DMA0_ERROR_IRQ,
                             };
            imx_add_common_device(IMX_HWI_DMA, IMX_DMA0_BASE, IMX_DMA_DEVICE_SIZE, irq, 2, (imx_edma_request_source_t *)imx8xp_edma0_requests,
                    IMX_DMA0_CH_NUM, &errata, 1);
        }
        {
            uint32_t errata =  IMX_DMA1_ERRATA;
            uint32_t irq[] = { IMX_DMA1_IRQ,
                               IMX_DMA1_ERROR_IRQ,
                             };
            imx_add_common_device(IMX_HWI_DMA, IMX_DMA1_BASE, IMX_DMA_DEVICE_SIZE, irq, 2, (imx_edma_request_source_t *)imx8xp_edma1_requests,
                    IMX8XP_DMA1_CH_NUM, &errata, 1);
        }
        {
            uint32_t errata =  IMX_DMA2_ERRATA;
            uint32_t irq[] = { IMX_DMA2_IRQ,
                               IMX_DMA2_ERROR_IRQ,
                             };
            imx_add_common_device(IMX_HWI_DMA, IMX_DMA2_BASE, IMX_DMA_DEVICE_SIZE, irq, 2, (imx_edma_request_source_t *)imx8xp_edma2_requests,
                    IMX_DMA2_CH_NUM, &errata, 1);
        }
        {
            uint32_t errata =  IMX_DMA3_ERRATA;
            uint32_t irq[] = { IMX_DMA3_IRQ,
                               IMX_DMA3_ERROR_IRQ,
                             };
            imx_add_common_device(IMX_HWI_DMA, IMX_DMA3_BASE, IMX_DMA_DEVICE_SIZE, irq, 2, (imx_edma_request_source_t *)imx8xp_edma3_requests,
                    IMX8XP_DMA3_CH_NUM, &errata, 1);
        }
    }
#endif
    /* Add RTC devices */
    {
        hwi_add_rtc("mx8sc", 0, 0, 0, 1, -1);
    }

    /* Add ATF status */
    {
        unsigned hwi_off;

        hwi_off = hwidev_add("smc_call", 0, HWI_NULL_OFF);
#ifdef IMX_ARM_TRUSTED_FIRMWARE_ENABLED
        const char *optstr = "smc_call=yes";
#else
        const char *optstr = "smc_call=no";
#endif
        hwitag_add_optstr (hwi_off, optstr);
    }
    /* Add imx8 board and type and silicon version */
    {
        hwi_tag *tag;
        unsigned off;
        unsigned hwi_off;

        hwi_off = hwidev_add("board", 0, 0);
        ASSERT(hwi_off != HWI_NULL_OFF);
        if (hwi_off != HWI_NULL_OFF) {
            off = add_string(IMX_BOARD_INFO);
            tag = hwi_alloc_tag(HWI_TAG_INFO(hwversion));
            tag->hwversion.hclass = (_Uint8t)(startup_data->chip_type); /* hclass = imx chip type */
            tag->hwversion.version = (_Uint8t)(startup_data->chip_rev); /* version = imx chip version */
            tag->hwversion.name = (_Uint16t)off;
        }
    }
}

static unsigned imx8x_hwibus_add_can(unsigned parent_hwi_off, hwiattr_can_t *attr)
{
    unsigned hwi_off = hwibus_add(HWI_ITEM_BUS_CAN, parent_hwi_off);
    if ((hwi_off != HWI_NULL_OFF) && (attr != NULL))
    {
        unsigned i;
        hwitag_add_common(hwi_off, &attr->common);
        for (i=0; i<attr->num_clks; i++)
            hwitag_add_inputclk(hwi_off, 0, 1);
    }
    return hwi_off;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/qxp-cpu/init_hwinfo.c $ $Rev: 891625 $")
#endif
