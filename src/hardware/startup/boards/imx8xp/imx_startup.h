/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2019 NXP
 * Copyright 2018, QNX Software Systems
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


#ifndef IMX_STARTUP_H_
#define IMX_STARTUP_H_

#include <hw/nxp/imx8/sci/sci.h>
#include <aarch64/mx8xp.h>

typedef uint64_t   imx_base_t;
#define IMX_IN     in32
#define IMX_OUT    out32
#define ONE_MHZ                 1000000
#define ONE_KHZ                 1000
/* Round last digit during division */
#define SAFE_DIVIDE(A,B)         ((A + (B - 1)) / B)

/* i.MX8 chip revision list */
#define IMX_CHIP_REV_A              0x00
#define IMX_CHIP_REV_B              0x01
#define IMX_CHIP_REV_C              0x02

/* i.MX8 chip type list */
#define IMX_CHIP_TYPE_QUAD_MAX      0x01
#define IMX_CHIP_TYPE_QUAD_X_PLUS   0x02
#define IMX_CHIP_TYPE_DUAL_X_PLUS   0x03

/* USB processor type specific features */
#define IMX_CHIP_USB3_LIMIT_TO_USB2 (1 << 0)
#define IMX_CHIP_USB3_AVAILABLE     (1 << 1)
#define IMX_CHIP_USB2_AVAILABLE     (1 << 2)
/* DC processor type specific features */
#define IMX_CHIP_DC0_PORT0_AVAIL    (1 << 0)
#define IMX_CHIP_DC0_PORT1_AVAIL    (1 << 1)
typedef struct {
    sc_ipc_t    ipc;                                /**< IPC handle */
    uint32_t    chip_rev;                           /**< Processor revision */
    uint32_t    chip_type;                          /**< Processor type */
    uint32_t    imx_spi_clk[IMX_LPSPI_COUNT];       /**< Array of SPI clocks used in HWI */
    uint32_t    imx_i2c_clock[IMX_LPI2C_COUNT];     /**< Array of I2C clocks used in HWI */
    uint32_t    imx_uart_clock[IMX_LPUART_COUNT];   /**< Array of LPUART clocks used in HWI */
    uint32_t    imx_usdhc_clk[IMX_USDHC_COUNT];     /**< Array of USDHC clocks used in HWI */
    bool        imx8dual_type;                      /**< TRUE if imx8 dual MCU type detected */
    uint32_t    usb_features;                       /**< USB processor type specific features */
    uint32_t    enet_features;                      /**< ENET processor type specific features */
    uint32_t    dc_features;                        /**< DC processor type specific features */
} imx_startup_data_t;

/*
 * IPG CLK = 24MHz, therefore period = 41.67ns = 0.042us
 * Therefore 24 clocks per us
 */
#define IPG_CLK                     24000000
#define IPG_CLKS_IN_ONE_US          24

#ifndef TRUE
    #define TRUE 1
#endif
#ifndef FALSE
    #define FALSE 0
#endif

/* Startup command line arguments */
#define IMX_WDOG_ENABLE            (1 << 0)
#define IMX_CAN1_ENABLE            (1 << 1)
#define IMX_AUDIO_CAPTURE_ENABLE   (1 << 2)
#define IMX_SDMA_ENABLE            (1 << 3)

extern void imx_init_lpuart(unsigned channel, const char *init, const char *defaults);
extern void imx_lpuart_put_char(int);

uint32_t imx_get_cpu_clk(sc_ipc_t ipc, sc_rsrc_t resource);

extern struct callout_rtn imx_lpuart_display_char;
extern struct callout_rtn imx_lpuart_poll_key;
extern struct callout_rtn imx_lpuart_break_detect;

void imx_timer_init(void);
unsigned int imx_get_timer_val(void);
unsigned int imx_get_timer_delta(unsigned int t_first, unsigned int t_second);
void imx_timer_print_delta(unsigned int t_first, unsigned int t_second);
void imx_usleep(uint32_t sleep_duration);
void imx_mmu_disable_flushcache(void);
void imx_init_cacheattr(imx_startup_data_t * startup_data);
void imx_init_raminfo(void);
void init_qtime(void);
int imx_init_clocks(imx_startup_data_t * startup_data);
int imx_init_pads(imx_startup_data_t * startup_data);
void imx_dump_clocks(imx_startup_data_t * startup_data);
void imx_init_usdhc_clk(void);
void imx_wdg_enable(sc_ipc_t ipc);
void imx_init_usb_otg1(void);
void imx_init_usb3_otg2(void);
void imx_init_usb_host1(void);
void imx_usb2_otg1_host_init(void);
void imx_usb3_otg2_host_init(void);


uint32_t imx_set_gpio_output(uint32_t base, uint32_t pin, uint32_t level);
uint32_t imx_set_gpio_input(uint32_t base, uint32_t pin);
uint32_t imx_reset_gpio_pin(uint32_t base, uint32_t pin,  uint32_t level);
uint32_t imx_reset_gpio_pin_fin(uint32_t dur);
uint32_t imx_get_gpio_value(uint32_t base, uint32_t pin, uint32_t *gpio_val);
uint32_t imx_gpio_set_irq_mode(uint32_t base, uint32_t pin, uint32_t irq_mode);
uint32_t imx_get_chip_rev(sc_ipc_t ipc);
uint32_t imx_get_chip_type(sc_ipc_t ipc);
void print_chip_info(sc_ipc_t ipc);

#endif /* IMX_STARTUP_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_startup.h $ $Rev: 912196 $")
#endif
