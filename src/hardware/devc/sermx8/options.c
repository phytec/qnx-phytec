/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
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


/*
#ifdef __USAGE
%C - Serial driver for i.MX UARTs

%C [options] [port[,irq]] &
Options:
 -b number    Define initial baud rate (default 115200)
 -c clk       Set the input clock rate (default 0). If this parameter is omitted,
              information from HWI table is used.
 -C number    Size of canonical input buffer (default 256)
 -e           Set options to "edit" mode
 -E           Set options to "raw" mode (default)
 -I number    Size of raw input buffer (default 2048)
 -f           Enable hardware flow control (default)
 -F           Disable hardware flow control
 -l number    Number of IDLE characters (default 4). Used only when DMA mode is enabled. Possible values: 1, 2, 4, 8, 16, 32, 64, 128.
 -O number    Size of output buffer (default 2048)
 -s           Enable software flow control
 -S           Disable software flow control (default)
 -t number    Set receive FIFO trigger level (default 1)
 -T number    Set number of characters to send to transmit FIFO (default 32)
 -u unit      Set serial unit number (default 1)
 -U uid:gid   Set the user id and group id
 -d           Use DMA (if DMA capability is available). devc-sermx8-dma is automatically built with DMA option enabled
              and should be used instead of devc-sermx8.
 -i (0|1)     Interrupt mode (0 = event, 1 = isr) (default = 1)

Examples:

# Disable HW and SW flow control for an UART device with the base address 0x5A080000 and the IRQ 253
# and enabled edited mode - useful for a debug port.
devc-sermx8 -e -F -S 0x5A080000,253

# Enable DMA and HW Flow Control for an UART device with the base address 0x5A080000 and the IRQ 253.
devc-sermx8-dma -d -E -f -S 0x5A080000,253

#endif
*/

#include "externs.h"

/**
 * @file       src/hardware/devc/sermx8/options.c
 * @addtogroup devc
 * @{
 */

TTYINIT_UART devinit_defaults = {
        /* tty */
        {
            0,             // port
            0,             // port_shift
            0,             // intr
            115200,        // baud
            2048,          // isize
            2048,          // osize
            256,           // csize
            0,             // c_cflag
            0,             // c_iflag
            0,             // c_lflag
            0,             // c_oflag
            0,             // fifo
            0,             // pfclk
            16,            // div
            "/dev/ser",    // name
            NULL,          //reserved1
            0,             //reserved2
            0,             //verbose
            0,             //highwater
            "",            //logging_path
            0              //lflags
        },
        { -1, -1},         // intr
#if defined(USE_DMA)
        1,                 // usedma (DMA enabled when USE_DMA is defined)
#else
        0,                 // usedma (DMA disabled when USE_DMA is not defined)
#endif
        -1,                // rx_dma_evt
        -1,                // tx_dma_evt
        1,                 // isr
        2,                 // rx_idle_cnt = 4 characters
        0,                 // usesmmu
    };

/**
 * Specify parameters for default devices from hwi_info tags.
 *
 * @param dip  Pointer to a tty device structure.
 * @param unit Index of the device.
 *
 * @return Execution status.
 */
static int query_hwi_device(TTYINIT_UART *dip, unsigned unit)
{
    unsigned hwi_off = hwi_find_device("uart", unit);
#if defined(USE_DMA)
    unsigned tag_idx = 0;
#endif

    if (hwi_off != HWI_NULL_OFF) {
        /* Peripheral input clock - update if not already set */
        hwi_tag *tag_inputclk = hwi_tag_find(hwi_off, HWI_TAG_NAME_inputclk, 0);
        if ((tag_inputclk != NULL) && (dip->tty.clk == 0)) {
            dip->tty.clk = tag_inputclk->inputclk.clk;
        }
        hwi_tag *tag_location = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, 0);
        /* Peripheral base address - update if not already set*/
        if ((tag_location != NULL) && (dip->tty.port == 0)) {
            dip->tty.port = tag_location->location.base;
        }
        /* IRQ vector number - update if not already set*/
        hwi_tag *tag_irq = hwi_tag_find(hwi_off, HWI_TAG_NAME_irq, 0);
        if ((tag_irq != NULL) && (dip->intr[0] == -1)) {
            dip->intr[0] = tag_irq->irq.vector;
        }
#if defined(USE_DMA)
        /* RX DMA channel code - update if not already set */
        hwi_tag *tag_dma = hwi_tag_find(hwi_off, HWI_TAG_NAME_dma, &tag_idx);
        if ((tag_dma != NULL) && (dip->rx_dma_evt == -1)) {
            dip->rx_dma_evt = tag_dma->dma.chnl;
        }
        /* Get TX DMA channel code from hwi table. Update a device structure only if it has not been not set yet. */
        tag_dma = hwi_tag_find(hwi_off, HWI_TAG_NAME_dma, &tag_idx);
        if ((tag_dma != NULL) && (dip->tx_dma_evt == -1)) {
            dip->tx_dma_evt = tag_dma->dma.chnl;
        }
#endif
        return 1;
    }
    /*
    * No default device, the base address and irq have been specified
    */
    return 0;
}

/**
 * Specify parameters for default devices from hwi_info tags. dev_base arguments is used
 * to search a device in hwi info table.
 *
 * @param dip      Pointer to a tty device structure.
 * @param dev_base Base address of an LPUART device.
 *
 * @return Execution status.
 */
static int query_hwi_device_by_base_address(TTYINIT_UART *dip, unsigned dev_base)
{
    int       hwi_dev_num = hwi_find_num_units("uart");
    unsigned  hwi_off;
    int       idx;
#if defined(USE_DMA)
    unsigned  tag_idx = 0;
#endif

    for (idx = 0; idx < hwi_dev_num; idx++) {
        hwi_off = hwi_find_device("uart", idx);
        if (hwi_off != HWI_NULL_OFF) {
            hwi_tag *tag_location = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, 0);
            if ((tag_location != NULL) && (tag_location->location.base == dev_base)) {
                /* Found a device with a base address specified by the argument dev_base.
                 * Update device info structure by values from hwi table - update if not already set.
                 */
                /* Peripheral input clock - update if not already set */
                hwi_tag *tag_inputclk = hwi_tag_find(hwi_off, HWI_TAG_NAME_inputclk, 0);
                if ((tag_inputclk != NULL) && (dip->tty.clk == 0)) {
                    dip->tty.clk = tag_inputclk->inputclk.clk;
                }
                /* Peripheral base address - update if not already set */
                if (dip->tty.port == 0) {
                    dip->tty.port = tag_location->location.base;
                }
                /* IRQ vector number - update if not already set */
                hwi_tag *tag_irq = hwi_tag_find(hwi_off, HWI_TAG_NAME_irq, 0);
                if ((tag_irq != NULL) && (dip->intr[0] == -1)) {
                    dip->intr[0] = tag_irq->irq.vector;
                }
#if defined(USE_DMA)
                /* RX DMA channel code - update if not already set*/
                hwi_tag *tag_dma = hwi_tag_find(hwi_off, HWI_TAG_NAME_dma, &tag_idx);
                if ((tag_dma != NULL) && (dip->rx_dma_evt == -1)) {
                    dip->rx_dma_evt = tag_dma->dma.chnl;
                }
                /* Get TX DMA channel code from hwi table. Update a device structure only if it has not been not set yet. */
                tag_dma = hwi_tag_find(hwi_off, HWI_TAG_NAME_dma, &tag_idx);
                if ((tag_dma != NULL) && (dip->tx_dma_evt == -1)) {
                    dip->tx_dma_evt = tag_dma->dma.chnl;
                }
#endif
                return 1;
            }
        }
    }

    /* No device found with a base address specified by the argument dev_base found. */
    return 0;
}


/**
 * Parse input options and set device parameters.
 *
 * @param argc The count of total command line arguments passed to executable on execution.
 * @param argv The array of character string of each command line argument passed to executable on execution.
 * *
 * @return Number of ports.
 */
unsigned options(int argc, char *argv[], uint8_t *smmu_ports_created)
{
    int         opt;
    int         numports = 0;
    void        *link;
    unsigned    rx_idle_chars;
    unsigned    unit;
    unsigned    rx_fifo_wm = RX_FIFO_WM_DEF;
    unsigned    tx_fifo_wm = TX_FIFO_WM_DEF;

    UserParm = NULL;

    TTYINIT_UART devinit;

    int found_hwi_device = -1;

    /*
     * Initialize the devinit to raw mode
     */
    ttc(TTC_INIT_RAW, &devinit_defaults, 0);

    unit = 0;

    while (optind < argc) {
        /* Load defaults for each device */
        memcpy(&devinit, &devinit_defaults, sizeof(TTYINIT_UART));

        /*
         * Process dash options.
         * Options already used by io-char (do not use these!): b,e,E,f,F,s,S,C,I,O,o,v
         */
        while ((opt = getopt(argc, argv, IO_CHAR_SERIAL_OPTIONS "t:T:c:u:U:di:l:")) != -1) {
            switch (ttc(TTC_SET_OPTION, &devinit, opt)) {
                case 't':
                    rx_fifo_wm = strtoul(optarg, NULL, 0);
                    if ((rx_fifo_wm >= MAX_FIFO_SIZE) || (rx_fifo_wm == 0)) {
                        fprintf(stderr, "Rx FIFO Watermark must be less than MAX FIFO size %d and greater than 0.\n", MAX_FIFO_SIZE);
                        fprintf(stderr, "Using Rx FIFO size of %d\n", RX_FIFO_WM_DEF);
                        rx_fifo_wm = RX_FIFO_WM_DEF;
                    }
                    break;

                case 'T':
                    tx_fifo_wm = strtoul(optarg, NULL, 0);
                    if (tx_fifo_wm >= MAX_FIFO_SIZE) {
                        fprintf(stderr, "Tx FIFO Watermark must be less than MAX FIFO size %d.\n", MAX_FIFO_SIZE);
                        fprintf(stderr, "Using Tx FIFO size of %d\n", TX_FIFO_WM_DEF);
                        tx_fifo_wm = TX_FIFO_WM_DEF;
                    }

                    break;

                case 'c':
                    devinit.tty.clk = strtoul(optarg, &optarg, 0);
                    break;

                case 'u':
                    unit = strtoul(optarg, NULL, 0);
                    break;

                case 'U':
                    UserParm = strdup(optarg);
                    break;

                case 'd':
#if defined(USE_DMA)
                    devinit.usedma = 1;
#else
                    fprintf(stderr, "DMA is not supported. Build the serial driver with the USE_DMA option enabled.\n");
#endif
                    break;

                case 'i':
                    devinit.isr = strtoul(optarg, NULL, 0);
                    break;

                case 'l':
                    rx_idle_chars = strtoul(optarg, NULL, 0);

                    if(rx_idle_chars == 0 || rx_idle_chars > 128) {
                        fprintf(stderr, "Idle characters is out of range(%d) - default to (%d)\n", rx_idle_chars, devinit.rx_idle_cnt);
                    } else if(__builtin_popcount(rx_idle_chars) == 1) { /*Make sure it is a power of 2 */
                        devinit.rx_idle_cnt =__builtin_ctz(rx_idle_chars);
                    } else {
                        fprintf(stderr, "Unsupported number of idle characters (%d) - default to (%d)\n", rx_idle_chars, devinit.rx_idle_cnt);
                    }
                    break;
            }
        }

        devinit.tty.fifo = (rx_fifo_wm << RX_FIFO_SHIFT) | (tx_fifo_wm << TX_FIFO_SHIFT);

        /*
         * Process ports and interrupts.
         */
        while (optind < argc  &&  *(optarg = argv[optind]) != '-') {
            devinit.tty.port = strtoul(optarg, &optarg, 16);
            if (*optarg == ',') {
                devinit.intr[0] = strtoul(optarg + 1, &optarg, 0);
                if (*optarg == ',') {
                    devinit.intr[1] = strtoul(optarg + 1, &optarg, 0);
                }
            }

            if (devinit.tty.port != 0 && devinit.intr[0] != -1) {
#if defined(USE_DMA)
                /* Set/clear the SMMU flag */
                if ( ttyctrl.flags & USE_SMMU ) {
                    slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "%s: SMMU ON", __FUNCTION__);
                    devinit.usesmmu = 1;
                } else {
                    slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "%s: SMMU OFF", __FUNCTION__);
                    devinit.usesmmu = 0;
                }
#else
                if ( ttyctrl.flags & USE_SMMU ) {
                    fprintf(stderr, "SMMU not support in non-DMA mode. Build the serial driver with the USE_DMA option enabled.\n");
                }
#endif
                /* Get information from hwi table and update the device init structure */
                query_hwi_device_by_base_address(&devinit, devinit.tty.port);
                /* Create device */
                create_device(&devinit, unit++, smmu_ports_created);
                ++numports;
            }
            ++optind;
        }
    }

    if (numports == 0) {
        unit = 0;
        link = NULL;

        /* Load defaults for each device */
        memcpy(&devinit, &devinit_defaults, sizeof(TTYINIT_UART));

        devinit.tty.fifo = (rx_fifo_wm << RX_FIFO_SHIFT) | (tx_fifo_wm << TX_FIFO_SHIFT);
        while (1) {
            found_hwi_device = query_hwi_device(&devinit, unit);
            if (!found_hwi_device) {
                break;
            }
            /* Get information from hwi table and update the device init structure */
            query_hwi_device_by_base_address(&devinit, devinit.tty.port);
            /* Create device */
            create_device(&devinit, unit++, smmu_ports_created);
            ++numports;
        }
        while (1) {
            link = query_default_device(&devinit, link);
            if (link == NULL) {
                break;
            }
            /* Create device */
            create_device(&devinit, unit++, smmu_ports_created);
            ++numports;
        }
    }

    return numports;
}


/** @} */ /* end of devc */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/devc/sermx8/options.c $ $Rev: 893169 $")
#endif
