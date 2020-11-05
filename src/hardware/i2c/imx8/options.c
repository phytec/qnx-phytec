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

#include "proto.h"


/**
 * Finds I2C device in hwi table.
 *
 * @param dev  Pointer to I2C driver device structure.
 * @param unit I2C unit number.
 *
 * @return Execution status.
 * @retval 1 Success.
 * @retval 0 Not found.
 */
static int query_hwi_device(imx_dev_t *dev, unsigned unit)
{
    unsigned hwi_off = hwi_find_bus(HWI_ITEM_BUS_I2C, unit);
    hwi_tag *tag;
    if (hwi_off != HWI_NULL_OFF) {
        tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, 0);
        if (tag) {
            dev->physbase = tag->location.base;
            dev->intr = hwitag_find_ivec(hwi_off, NULL);
            tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_inputclk, NULL);
            if (tag) {
                dev->input_clk = tag->inputclk.clk;
            }
        }
        return 1;
    }
    /*
     * No default device, the base address and irq have been specified
     */
    return 0;
}

/**
 * Parses I2C driver specific options.
 *
 * @param dev  Pointer to I2C driver device structure.
 * @param argc Number of arguments.
 * @param argv Pointer to arguments.
 *
 * @return Execution status.
 * @retval  0 Success.
 * @retval -1 Fail.
 */
int imx_options(imx_dev_t *dev, int argc, char *argv[])
{
    int c;
    int prev_optind;
    int done = 0;
    unsigned interface;

    /* defaults */
    dev->intr = -1;
    dev->iid = -1;
    dev->physbase = 0;
    dev->slave_addr = 0;
    dev->input_clk = IMX_I2C_INPUT_CLOCK;
    dev->verbose = 0;
    dev->int_mode = 1;
    dev->nack = 0;
    /* Set default timeout to 1000 ms to allow transfer of large data blocks eg. 4 KB */
    dev->timeout = 1000 * 1000 * 1000;
    dev->max_retries = 0;

    while (!done) {
        prev_optind = optind;
        c = getopt(argc, argv, "I:c:p:i:t:r:vb");
        switch (c) {
            case 'I':
                interface = strtoul(optarg, &optarg, 0);
                /* Getting the I2C Base addresss and irq from the Hwinfo Section if available */
                if (!query_hwi_device(dev, interface)) {
                    printf("%s - interface %i not found in HWI table\n", __FUNCTION__, interface);
                    /* Device not found in HWI table */
                    return -1;
                }
                break;
            case 'c':
                dev->input_clk = strtoul(optarg, &optarg, 0);
                break;
            case 'p':
                dev->physbase = strtoul(optarg, &optarg, 0);
                break;
            case 'i':
                dev->intr = strtol(optarg, &optarg, 0);
                break;
            case 'v':
                dev->verbose = 1;
                break;
            case 'b':
                /* Driver blocking mode */
                dev->int_mode = 0;
                break;
            case 't':
                /* Timeout in ns  = ms * 1000 * 1000 */
                dev->timeout = strtol(optarg, &optarg, 0) * 1000 * 1000;
                break;
            case 'r':
                /* Max retries when error is detected */
                dev->max_retries = strtol(optarg, &optarg, 0);
                break;
            case '?':
                if (optopt == '-') {
                    ++optind;
                    break;
                }
                return -1;
            case -1:
                if (prev_optind < optind) { /* -- */
                    return -1;
                }

                if (argv[optind] == NULL) {
                    done = 1;
                    break;
                }
                if (*argv[optind] != '-') {
                    ++optind;
                    break;
                }
                return -1;

            case ':':
            default:
                return -1;
        }
    }
    /* Disable interrupt in case of unknown IRQ */
    if (dev->intr == -1) {
        dev->int_mode = 0;
    }
    return 0;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/i2c/imx8/options.c $ $Rev: 893360 $")
#endif
