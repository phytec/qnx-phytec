/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * *
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

#include <stdlib.h>
#include <stdio.h>
#include <errno.h>
#include <malloc.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>
#include <hw/inout.h>
#include <sys/syspage.h>
#include <inttypes.h>
#include <hw/sysinfo.h>
#include <drvr/hwinfo.h>
#include <login.h>
#include <sys/procmgr.h>

#include "canmx7x.h"
#include "proto.h"

// Default FlexCAN IRQ's
#define FLEXCAN0_SYSINTR             (32 + 110)
#define FLEXCAN1_SYSINTR             (32 + 111)

// Default for FlexCAN 2
#define FLEXCAN2_REG_BASE            0x5A8F0000
#define FLEXCAN2_MEM_BASE            0x5A8F0080
#define FLEXCAN2_SYSINTR             269

// Define default bitrate settings for i.MX7 board
// based on CAN Bit Rate Calculation and rules from
// FlexCAN Reference Guide.
// The PSEG1, PSEG2, RJW and PROPSEG values are held constant and the
// PRESDIV value is changed for the different default bitrates.

// The FlexCAN module uses CTRL register to set-up the bit timing parameters required by the CAN protocol.
// Control register (CANCTRL) contains the PROPSEG = PROP_SEG(Bit 0-3),
// PSEG1 = PHASE_SEG1 (Bit 19-21), PSEG2 = PHASE_SEG2 (Bit 16-18),
// and the RJW (Bit 22-23) fields which allow the user to configure the bit timing parameters.
// The prescaler divide register (PRESDIV) allows the user to select the ratio used to derive the clock from the system clock.
// For the position of the sample point only the relation (SYNC_SEG + PROP_SEG + PHASE_SEG1) / (PHASE_SEG2) is important.
// The values for PRESDIV, PROPSEG, PSEG1 and PSEG2 are as given below.

/* The number of time quanta is 16 */
#define CAN_RJW                      0
#define CAN_PROPSEG                  0x04
#define CAN_PSEG1                    0x07
#define CAN_PSEG2                    0x01

/* Bitrate values for XTAL 24.5 MHz, desired Sample Point at 87.5% */
//TODO CAN - Support 24M XTAL as a clock source for FlexCAN module.
#define CAN_PRESDIV_10K_XTAL         0xAE
#define CAN_PRESDIV_50K_XTAL         0x22
#define CAN_PRESDIV_125K_XTAL        0x0d
#define CAN_PRESDIV_250K_XTAL        0x06

/* Bitrate table for CAN_CLK 80 MHz, desired Sample Point at 87.5% */
#define CAN_PRESDIV_50K_PLL          0x63   /* (((30000000) / (50000UL * 16Tq)) - 1) */
#define CAN_PRESDIV_125K_PLL         0x27   /* (((30000000) / (125000UL * 16Tq)) - 1) */
#define CAN_PRESDIV_250K_PLL         0x13   /* (((30000000) / (250000UL * 16Tq)) - 1) */
#define CAN_PRESDIV_500K_PLL         0x09   /* (((30000000) / (500000UL * 16Tq)) - 1) */

//#define DEBUG_DRVR

// Function prototypes
void device_init(int argc, char *argv[]);
void create_device(CANDEV_FLEXCAN_INIT *devinit);

typedef struct candev_hwinfo
{
    paddr_t regbase;
    paddr_t reglen;
    paddr_t membase;
    paddr_t memlen;
    unsigned irqvector;
} CANDEV_HWINFO;


/**
 * Get CAN device structure.
 *
 * @param candev  Pointer to a device info structure.
 * @param unit   CAN device index.
 *
 * @return Execution status
 * @retval 0  OK
 * @retval -1 Error
 */
static int get_can_hwinfo(CANDEV_HWINFO *candev, unsigned unit)
{
    unsigned hwi_off;

    hwi_off = hwi_find_bus(HWI_ITEM_BUS_CAN, unit);
    if (hwi_off != HWI_NULL_OFF ) {
        hwiattr_can_t attr;

        hwiattr_get_can(hwi_off, &attr);

        candev->regbase = attr.common.location.base;

        if (attr.common.num_irq > 0) {
            candev->irqvector = hwitag_find_ivec(hwi_off, NULL);
        }

        if (attr.num_memaddr > 0) {
            unsigned instance = 1;
            hwi_tag *tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, &instance);
            if (tag != NULL) candev->membase = tag->location.base;
        }

        return 0;
    } else {
        return -1;
    }
}

/**
 * CAN driver main function.
 *
 * @param argc The count of total command line arguments passed to executable on execution.
 * @param argv The array of character string of each command line argument passed to executable on execution.
 *
 * @retval  0 In case of success.
 * @retval -1 In case of error.
 */
int main(int argc, char *argv[])
{
    // Driver implemented functions called by CAN library
    can_drvr_funcs_t drvr_funcs = {can_drvr_transmit, can_drvr_devctl, NULL, NULL};

    // Get I/O privity - new flags introduced in 6.6 must be set up to access the CAN register space
    ThreadCtl(PRIVITY_FLAGS, 0);
    if (ThreadCtl(PRIVITY_FLAGS, 0) == -1) {
        perror("ThreadCtl()");
        return EXIT_FAILURE;
    }

    // Initialize Resource Manager
    can_resmgr_init(&drvr_funcs);

    // Process options and create devices
    device_init(argc, argv);

    /* Drop Root */
    if (UserParm != NULL) {
        if(procmgr_ability( 0,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_KEYDATA,
                            PROCMGR_AOP_DENY  | PROCMGR_ADN_NONROOT | PROCMGR_AOP_LOCK            | PROCMGR_AID_EOL)
                             != EOK){
            fprintf(stderr, "%s:Unable to gain procmgr abilities for nonroot operation.\n", argv[0]);
            return EXIT_FAILURE;
        }
        if(set_ids_from_arg(UserParm) != EOK){
            fprintf(stderr, "%s: Unable to drop to user %s: %s\n", argv[0], UserParm, strerror(errno));
            return EXIT_FAILURE;
        }
        free(UserParm);
    }

    // Start Handling Clients
    can_resmgr_start();

    return EXIT_SUCCESS;
}

/**
 * Initialize CAN device.
 *
 * @param argc Parameter passed from the main function.
 * @param argv Parameter passed from the main function.
 */
void device_init(int argc, char *argv[])
{
    int  opt, hwi_can0, hwi_can1, hwi_can2;
    int  numcan = 0;
    char *cp;
    CANDEV_HWINFO can0, can1, can2;

    // Set default options
    CANDEV_FLEXCAN_INIT devinit = {
        {
            CANDEV_TYPE_RX,                 /* devtype */
            0,                              /* can_unit - set this later */
            0,                              /* dev_unit - set this later*/
            100,                            /* msgq_size  - max number of queued CAN messages per mailbox */
            16,                             /* waitq_size - max number of clients blocked wait for receive messages */
            0                               /* mode - CAN driver mode - I/O or raw frames */
        },
#ifndef VARIANT_mx8x
        IMX7_FLEXCAN0_REG_BASE,              /* port */
        IMX7_FLEXCAN0_MEM_BASE,              /* mem */
#else
        IMX_FLEXCAN0_REG_BASE,              /* port */
        IMX_FLEXCAN0_MEM_BASE,              /* mem */
#endif
        FLEXCAN_CLK_PLL,                    /* clk */
        0,                                  /* bitrate */
        CAN_PRESDIV_50K_PLL,                /* br_presdiv */
        CAN_PROPSEG,                        /* br_propseg */
        CAN_RJW,                            /* br_rjw */
        CAN_PSEG1,                          /* br_pseg1 */
        CAN_PSEG2,                          /* br_pseg2 */
        FLEXCAN0_SYSINTR,                   /* irqsys for i.MX7 board */
        INIT_FLAGS_MDRIVER_INIT,            /* flags */
        /* numtx - number of transmit mailboxes */
        IMX7_FLEXCAN_NUM_MAILBOX / 2,
        /* numrx - number of receive mailboxes */
        IMX7_FLEXCAN_NUM_MAILBOX / 2,
        0x100C0000,                         /* midrx */
        0x100C0000,                         /* midtx */
        0x0,                                /* timestamp */
    };

    hwi_can0 = hwi_can1 = hwi_can2 = 1;
    hwi_can0 = get_can_hwinfo(&can0, 0);
    hwi_can1 = get_can_hwinfo(&can1, 1);
    hwi_can2 = get_can_hwinfo(&can2, 2);

    UserParm = NULL;

    // Process command line options and create associated devices
    while (optind < argc) {
        // Process dash options
        while ((opt = getopt(argc, argv, "ab:B:c:Di:l:m:Mn:pRsStu:U:vwxz")) != -1) {
            switch (opt) {
                case 'a':
                    devinit.flags |= INIT_FLAGS_AUTOBUS;
                    break;
                case 'b':
                    // Set CANCTRL params to default
                    devinit.br_propseg = CAN_PROPSEG;
                    devinit.br_pseg1 = CAN_PSEG1;
                    devinit.br_pseg2 = CAN_PSEG2;
                    devinit.br_rjw = CAN_RJW;

                    // Determine BRP value for desired bitrate
                    if (strncmp(optarg, "50K", 3) == 0) {
                        devinit.br_presdiv = CAN_PRESDIV_50K_PLL;
                    } else if (strncmp(optarg, "125K", 4) == 0) {
                        devinit.br_presdiv = CAN_PRESDIV_125K_PLL;
                    } else if (strncmp(optarg, "250K", 4) == 0) {
                        devinit.br_presdiv = CAN_PRESDIV_250K_PLL;
                    } else if (strncmp(optarg, "500K", 4) == 0) {
                        devinit.br_presdiv = CAN_PRESDIV_500K_PLL;
                    } else {
                        // Set default to 50K
                        devinit.br_presdiv = CAN_PRESDIV_50K_PLL;
                    }
                    break;

                case 'B':
                    // Values to program bitrate manually
                    devinit.br_presdiv = strtoul(optarg, &optarg, 0);

                    if ((cp = strchr(optarg, ','))) {
                        cp += 1;    // Skip over the ','
                        devinit.br_propseg = strtoul(cp, &cp, 0);
                    }
                    if (cp && (cp = strchr(cp, ','))) {
                        cp += 1;    // Skip over the ','
                        devinit.br_pseg1 = strtoul(cp, &cp, 0);
                    }
                    if (cp && (cp = strchr(cp, ','))) {
                        cp += 1;    // Skip over the ','
                        devinit.br_pseg2 = strtoul(cp, &cp, 0);
                    }
                    if (cp && (cp = strchr(cp, ','))) {
                        cp += 1;    // Skip over the ','
                        devinit.br_rjw = strtoul(cp, &cp, 0);
                    }

                    // Check for valid bitrate settings
                    if (devinit.br_rjw > IMX7_FLEXCAN_CTRL1_RJW_MAXVAL
                            || devinit.br_pseg1 > IMX7_FLEXCAN_CTRL1_PSEG1_MAXVAL
                            || devinit.br_pseg2 > IMX7_FLEXCAN_CTRL1_PSEG2_MAXVAL
                            || devinit.br_pseg2 == 0) {
                        fprintf(stderr, "Invalid manual bitrate settings\n");
                        exit(EXIT_FAILURE);
                    }
                    break;
                case 'c':
                    if (strncmp(optarg, "24M", 3) == 0)
                        devinit.clk = FLEXCAN_CLK_EXTAL;
                    else if (strncmp(optarg, "30M", 3) == 0)
                        devinit.clk = FLEXCAN_CLK_PLL;
                    else
                        devinit.clk = strtoul(optarg, NULL, 0);
                    break;
                case 'D':
                    devinit.flags &= ~INIT_FLAGS_MDRIVER_INIT;
                    break;
                case 'i':
                    devinit.midrx = strtoul(optarg, &optarg, 16);
                    if ((cp = strchr(optarg, ','))) {
                        devinit.midtx = strtoul(cp + 1, NULL, 0);
                    }
                    break;
                case 'm':
                    devinit.flags |= INIT_FLAGS_TIMESTAMP;
                    devinit.timestamp = strtoul(optarg, NULL, 16);
                    break;
                case 'n':
                    devinit.cinit.msgq_size = strtoul(optarg, NULL, 0);
                    break;
                case 'p':
                    devinit.flags |= INIT_FLAGS_CLKSRC;
                    break;
                case 'q':
                    devinit.cinit.waitq_size = strtoul(optarg, NULL, 0);
                    break;
                case 'R':
                    devinit.cinit.mode = CANDEV_MODE_RAW_FRAME;
                    devinit.numrx = RAW_MODE_RX_NUM_MBOX;
                    devinit.numtx = RAW_MODE_TX_NUM_MBOX;
                    break;
                case 's':
                    devinit.flags |= INIT_FLAGS_BITRATE_SAM;
                    break;
                case 'S':
                    devinit.flags |= INIT_FLAGS_MDRIVER_SORT;
                    break;
                case 't':
                    devinit.flags |= INIT_FLAGS_LOOPBACK;
                    break;
                case 'u':
                    devinit.cinit.can_unit = strtoul(optarg, NULL, 0);
                    break;
                case 'U':
                    UserParm = strdup(optarg);
                    break;
                case 'v':
                    devinit.flags |= INIT_FLAGS_LOM;
                    break;
                case 'w':
                    devinit.flags |= INIT_FLAGS_LBUF;
                    break;
                case 'x':
                    devinit.flags |= INIT_FLAGS_EXTENDED_MID;
                    break;
                case 'z':
                    devinit.flags |= INIT_FLAGS_TSYN;
                    break;
                default:
                    break;
            }
        }

        // Ensure message ID is valid
        if (devinit.flags & INIT_FLAGS_EXTENDED_MID) {
            devinit.midrx &= FLEXCAN_MID_MASK_EXT;
            devinit.midtx &= FLEXCAN_MID_MASK_EXT;
        } else {
            devinit.midrx &= FLEXCAN_MID_MASK_STD;
            devinit.midtx &= FLEXCAN_MID_MASK_STD;
        }

        // Process ports and interrupt
        while (optind < argc && *(optarg = argv[optind]) != '-') {
            if (strncmp(optarg, "can0", 4) == 0) {
                // Set default port for CAN 0
                if (0 == hwi_can0) {
                    devinit.port = can0.regbase;
                    devinit.mem = can0.membase;
                    devinit.irqsys = can0.irqvector;
                } else { // default values, They should be removed once all startups have been changed
                #ifndef VARIANT_mx8x
                    devinit.port = IMX7_FLEXCAN0_REG_BASE;
                    devinit.mem = IMX7_FLEXCAN0_MEM_BASE;
                #else
                    devinit.port = IMX_FLEXCAN0_REG_BASE;
                    devinit.mem = IMX_FLEXCAN0_MEM_BASE;
                #endif
                    // Set defaults even though user may override them
                    devinit.irqsys = FLEXCAN0_SYSINTR;
                }
                // Set default can unit number
                if (!devinit.cinit.can_unit) {
                    devinit.cinit.can_unit = 0;
                }
                // Increment optarg
                optarg += 4; // skip "can0" string
            } else if (strncmp(optarg, "can1", 4) == 0) {
                // Set default port for CAN 1
                if (0 == hwi_can1) {
                    devinit.port = can1.regbase;
                    devinit.mem = can1.membase;
                    devinit.irqsys = can1.irqvector;
                } else { // default values, They should be removed once all startups have been changed
                #ifndef VARIANT_mx8x
                    devinit.port = IMX7_FLEXCAN1_REG_BASE;
                    devinit.mem = IMX7_FLEXCAN1_MEM_BASE;
                #else
                    devinit.port = IMX_FLEXCAN1_REG_BASE;
                    devinit.mem = IMX_FLEXCAN1_MEM_BASE;
                #endif
                    // Set defaults even though user may override them
                    devinit.irqsys = FLEXCAN1_SYSINTR;
                }
                // Set default can unit number
                if (!devinit.cinit.can_unit) {
                    devinit.cinit.can_unit = 1;
                }
                // Increment optarg
                optarg += 4; //skip "can1" string
            } else if (strncmp(optarg, "can2", 4) == 0) {
                // Set default port for CAN 2
                if (0 == hwi_can2) {
                    devinit.port = can2.regbase;
                    devinit.mem = can2.membase;
                    devinit.irqsys = can2.irqvector;
                } else { // default values, They should be removed once all startups have been changed
                #ifndef VARIANT_mx8x
                    devinit.port = FLEXCAN2_REG_BASE;
                    devinit.mem = FLEXCAN2_MEM_BASE;
                #else
                    devinit.port = IMX_FLEXCAN2_REG_BASE;
                    devinit.mem = IMX_FLEXCAN2_MEM_BASE;
                #endif
                    // Set defaults even though user may override them
                    devinit.irqsys = FLEXCAN2_SYSINTR;
                }
                // Set default can unit number
                if (!devinit.cinit.can_unit) {
                    devinit.cinit.can_unit = 2;
                }
                // Increment optarg
                optarg += 4; //skip "can2" string
            } else {
                fprintf(stderr, "Invalid options\n");
                exit(EXIT_FAILURE);
            }
            // Set system interrupt vector
            if (*optarg == ',') {
                devinit.irqsys = strtoul(optarg + 1, NULL, 0);
            }
            ++optind;

            // Create the CAN device
            create_device(&devinit);
            // Reset unit number for next device
            devinit.cinit.can_unit = 0;
            numcan++;
        }
    }

    // If no devices have been created yet, create the default device
    if (numcan == 0) {
        // Create the default CAN device
        devinit.cinit.can_unit = 1;
        create_device(&devinit);
    }
}

/**
 * Create CAN device.
 *
 * @param devinit Pointer to a device init structure.
 */
void create_device(CANDEV_FLEXCAN_INIT *devinit)
{
    CANDEV_FLEXCAN_INFO *devinfo;
    CANDEV_FLEXCAN *devlist;
    int mdriver_intr = -1;
    unsigned int i;
    unsigned num_mailboxes;

#ifdef DEBUG_DRVR
    fprintf(stderr, "port = 0x%X\n", devinit->port);
    fprintf(stderr, "mem = 0x%X\n", devinit->mem);
    fprintf(stderr, "clk = %d\n", devinit->clk);
    fprintf(stderr, "bitrate = %d\n", devinit->bitrate);
    fprintf(stderr, "presdiv = %d\n", devinit->br_presdiv);
    fprintf(stderr, "propseg = %d\n", devinit->br_propseg);
    fprintf(stderr, "rjw = %d\n", devinit->br_rjw);
    fprintf(stderr, "pseg1 = %d\n", devinit->br_pseg1);
    fprintf(stderr, "pseg2 = %d\n", devinit->br_pseg2);
    fprintf(stderr, "irqsys = %d\n", devinit->irqsys);
    fprintf(stderr, "unit = %u\n", devinit->cinit.can_unit);
    fprintf(stderr, "flags = %u\n", devinit->flags);
    fprintf(stderr, "numrx = %u\n", devinit->numrx);
    fprintf(stderr, "numtx = %u\n", devinit->numtx);
    fprintf(stderr, "midrx = 0x%X\n", devinit->midrx);
    fprintf(stderr, "midtx = 0x%X\n", devinit->midtx);
#endif

    // Allocate device info
    devinfo = (void *)_smalloc(sizeof(*devinfo));
    if (!devinfo) {
        fprintf(stderr, "devinfo: _smalloc failed\n");
        exit(EXIT_FAILURE);
    }
    memset(devinfo, 0, sizeof(*devinfo));

    // Set up CAN operation mode - single RX and TX mailboxes using raw frames
    // or multi-mailbox, I/O based communications
    devinfo->mode = devinit->cinit.mode;

    // Setup the RX and TX mailbox sizes
    devinfo->numrx = devinit->numrx;
    devinfo->numtx = devinit->numtx;
    num_mailboxes = devinit->numrx + devinit->numtx;

    // Allocate an array of devices - one for each mailbox
    devlist = (void *)_smalloc(sizeof(*devlist) * num_mailboxes);
    if (!devlist) {
        fprintf(stderr, "devlist: _smalloc failed\n");
        exit(EXIT_FAILURE);
    }
    memset(devlist, 0, sizeof(*devlist) * num_mailboxes);

    // Map device registers
    devinfo->base = mmap_device_io(IMX7_FLEXCAN_REG_SIZE, devinit->port);
    if (devinfo->base == MAP_DEVICE_FAILED ) {
        perror("CAN REG: Can't map device I/O");
        exit(EXIT_FAILURE);
    }

    // Determine if there is an active mini-driver and initialize driver to support it
    if (devinit->flags & INIT_FLAGS_MDRIVER_INIT) {
        mdriver_intr = mdriver_init(devinfo, devinit);
    }

    // Map device message memory
    devinfo->canmsg = mmap_device_memory(NULL, IMX7_FLEXCAN_MEM_SIZE,
            PROT_READ | PROT_WRITE | PROT_NOCACHE, 0, devinit->mem);
    if (devinfo->canmsg == MAP_FAILED ) {
        perror("CAN MSG: Can't map device memory");
        exit(EXIT_FAILURE);
    }
    // Clear the mailbox memory if there is no mini-driver
    if (!(devinit->flags & INIT_FLAGS_MDRIVER_INIT) || (mdriver_intr == -1)) {
        memset(devinfo->canmsg, 0, IMX7_FLEXCAN_MEM_SIZE);
    }

    // Map CANLAM memory
    devinfo->canlam = mmap_device_io(IMX7_FLEXCAN_LAM_MEM_SIZE, devinit->port + IMX7_FLEXCAN_RXIMR0);
    if (devinfo->canlam == MAP_DEVICE_FAILED ) {
        perror("CAN IMR: Can't map device I/O");
        exit(EXIT_FAILURE);
    }

    // Setup device info
    devinfo->devlist = devlist;
    strcpy(devinfo->initinfo.description, "MX7X FlexCAN");
    devinfo->initinfo.msgq_size = devinit->cinit.msgq_size;
    devinfo->initinfo.waitq_size = devinit->cinit.waitq_size;
    devinfo->initinfo.bitrate = devinit->bitrate;
    devinfo->initinfo.br_brp = devinit->br_presdiv;
    devinfo->initinfo.br_rjw = devinit->br_rjw;
    devinfo->initinfo.br_pseg1 = devinit->br_pseg1;
    devinfo->initinfo.br_pseg2 = devinit->br_pseg2;

    // Initialize flags
    if (devinit->flags & INIT_FLAGS_RX_FULL_MSG) {
        devinfo->iflags |= INFO_FLAGS_RX_FULL_MSG;
    }

    devinfo->iflags |= INFO_FLAGS_ENDIAN_SWAP;

    // Initialize all device mailboxes
    for (i = 0; i < num_mailboxes; i++) {
        // Set index into device mailbox memory
        devlist[i].mbxid = i;
        // Store a pointer to the device info
        devlist[i].devinfo = devinfo;

        // Set device mailbox unit number
        devinit->cinit.dev_unit = i;
        // Set device mailbox as transmit or receive
        if (i < devinfo->numrx) {
            devinit->cinit.devtype = CANDEV_TYPE_RX;
        } else {
            devinit->cinit.devtype = CANDEV_TYPE_TX;
        }

        // Initialize the CAN device
        can_resmgr_init_device(&devlist[i].cdev, (CANDEV_INIT *)devinit);

        // Create the resmgr device
        can_resmgr_create_device(&devlist[i].cdev);
    }

    if (!(devinit->flags & INIT_FLAGS_MDRIVER_INIT) ||( mdriver_intr == -1)) {
        can_init_hw(devinfo, devinit);
    }

#ifdef DEBUG_DRVR
    can_print_reg(devinfo);
    can_print_mailbox(devinfo);
#endif

    // Initialize interrupts and attach interrupt handler
    can_init_intr(devinfo, devinit, mdriver_intr);

}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/can/mx7x/driver.c $ $Rev: 893475 $")
#endif
