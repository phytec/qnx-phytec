/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
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

#ifndef MX7_CAN_H_
#define MX7_CAN_H_

#include <hw/libcan.h>
#include <hw/mini_driver.h>
#include <arm/imx7/imx7_flexcan.h>

#ifdef VARIANT_mx8x
#include <aarch64/mx8x.h>
#endif

#define RAW_MODE_RX_NUM_MBOX                    1
#define RAW_MODE_TX_MBOX                        1
#define RAW_MODE_TX_NUM_MBOX                    1
#define RAW_MODE_RX_IRQ                         (0x01 << 0)
#define RAW_MODE_TX_IRQ                         (0x01 << 1)

/*
 * The crystal oscillator clock should be selected whenever a tight
 * tolerance (up to 0.1%) is required in the CAN bus timing.
 * The crystal oscillator clock has better jitter performance
 * than PLL generated clocks.
 */
#define FLEXCAN_CLK_EXTAL                       24576000
#define FLEXCAN_CLK_PLL                         80000000

/*** Receive message object codes ***/
#define MSG_BUF_CODE_MASK                       0x0F000000
#define MSG_BUF_CODE_SHIFT                      24

/* Message buffer is not active */
#define REC_CODE_NOT_ACTIVE                     0x0
/* Message buffer is full */
#define REC_CODE_FULL                           0x2
/* Message buffer is active and empty */
#define REC_CODE_EMPTY                          0x4
/* Second frame was received into a full buffer before CPU read the first */
#define REC_CODE_OVERRUN                        0x6

/* Message buffer is now being filled with a new receive frame.
 * This condition will be cleared within 20 cycles.
 */
#define REC_CODE_BUSY                           0x1

/*** Transmit message object codes ***/
/* Message buffer not ready for transmit */
#define TRANS_CODE_NOT_READY                    0x8
/* Message buffer was configured as Tx and ARM aborted the transmission */
#define TRANS_CODE_ABORT                        0x9
/* Data frame to be transmitted once, unconditionally */
#define TRANS_CODE_TRANSMIT_ONCE                0xC
/* Remote frame to be transmitted once, and message buffer becomes
 * an RX message buffer for data frames;
 * RTR bit must be set
 */
#define TRANS_CODE_TRANSMIT_RTR_ONCE            0xC
/* Data frame to be transmitted only as a response to a remote frame, always */
#define TRANS_CODE_TRANSMIT_ONLY_RTR            0xA
/* Data frame to be transmitted once, unconditionally
 * and then only as a response to remote frame always
 */
#define TRANS_CODE_TRANSMIT_ONCE_RTR_ALWAYS     0xE

/* Definitions for FlexCAN Message Control Field */
#define FLEXCAN_MCF_DLC_BYTE0                   0x0
#define FLEXCAN_MCF_DLC_BYTE1                   0x1
#define FLEXCAN_MCF_DLC_BYTE2                   0x2
#define FLEXCAN_MCF_DLC_BYTE3                   0x3
#define FLEXCAN_MCF_DLC_BYTE4                   0x4
#define FLEXCAN_MCF_DLC_BYTE5                   0x5
#define FLEXCAN_MCF_DLC_BYTE6                   0x6
#define FLEXCAN_MCF_DLC_BYTE7                   0x7
#define FLEXCAN_MCF_DLC_BYTE8                   0x8
#define FLEXCAN_MCF_RTR                         0x00100000
#define FLEXCAN_MCF_TPL_SHIFT                   29
#define FLEXCAN_MCF_TPL_MAXVAL                  0x7
#define FLEXCAN_MCF_TPL_MASK                    0xE0000000
#define FLEXCAN_MCF_IDE                         0x200000

/* Definitions for FlexCAN Message Identifier */
#define FLEXCAN_MID_MASK_STD                    0x1FFC0000
#define FLEXCAN_MID_MASK_EXT                    0x1FFFFFFF

#define CANDEV_TX_ENABLED                       0x00000001

#define INIT_FLAGS_LOOPBACK                     0x00000001    /* Enable self-test/loopback mode */
#define INIT_FLAGS_EXTENDED_MID                 0x00000002    /* Enable 29 bit extended message ID */
#define INIT_FLAGS_AUTOBUS                      0x00000004    /* Disable auto bus on */
#define INIT_FLAGS_TIMESTAMP                    0x00000008    /* Set initial local network time */
#define INIT_FLAGS_RX_FULL_MSG                  0x00000010    /* Receiver should store message ID, timestamp, etc. */
#define INIT_FLAGS_BITRATE_SAM                  0x00000020    /* Enable Bitrate Triple Sample Mode */
#define INIT_FLAGS_MDRIVER_INIT                 0x00000040    /* Initialize from mini-driver (if it exists and is running) */
#define INIT_FLAGS_MDRIVER_SORT                 0x00000080    /* Sort buffered mdriver message based on MID */
#define INIT_FLAGS_CLKSRC                       0x00000100    /* External reference clock source */
#define INIT_FLAGS_TSYN                         0x00000200    /* Enable Timer Sync feature */
#define INIT_FLAGS_LOM                          0x00000400    /* Listen Only Mode */
#define INIT_FLAGS_LBUF                         0x00000800    /* Lowest number buffer is transmitted first */

#define INFO_FLAGS_RX_FULL_MSG                  0x00000001    /* Receiver should store message ID, timestamp, etc. */
#define INFO_FLAGS_ENDIAN_SWAP                  0x00000002    /* Data is TX/RX'd MSB, need to perform ENDIAN conversions */


struct candev_flexcan_entry;

/* Initialization and Options Structure */
typedef struct candev_flexcan_init_entry
{
    CANDEV_INIT      cinit;                    /* Generic CAN Device Init Params */
    _Paddr32t        port;                     /* Device Physical Register Address */
    _Paddr32t        mem;                      /* CAN Message Object Physical Memory Address */
    uint32_t         clk;                      /* CAN Clock */
    /* Bitrate related parameters */
    uint32_t         bitrate;                  /* Bitrate */
    uint8_t          br_presdiv;               /* Bitrate Prescaler */
    uint8_t          br_propseg;               /* Propagation Segment Time */
    uint8_t          br_rjw;                   /* Bitrate Resync Jump Width */
    uint8_t          br_pseg1;                 /* Bitrate Phase Buffer Segment 1 */
    uint8_t          br_pseg2;                 /* Bitrate Phase Buffer Segment 2 */
    int              irqsys;                   /* Device Message System Vector */
    uint32_t         flags;                    /* Initialization flags */
    uint32_t         numtx;                    /* Number of TX Mailboxes */
    uint32_t         numrx;                    /* Number of RX Mailboxes */
    uint32_t         midrx;                    /* RX Message ID */
    uint32_t         midtx;                    /* TX Message ID */
    uint32_t         timestamp;                /* Initial value for local network time */
} CANDEV_FLEXCAN_INIT;

typedef struct candev_flexcan_init_info
{
    char             description[64];          /* CAN device description */
    uint32_t         msgq_size;                /* Number of messages */
    uint32_t         waitq_size;               /* Length of CAN message data */
    uint32_t         bitrate;                  /* Bitrate */
    uint16_t         br_brp;                   /* Bitrate Prescaler */
    uint8_t          br_rjw;                   /* Bitrate Sync Jump Width */
    uint8_t          br_pseg1;                 /* Bitrate Time Segment 1 */
    uint8_t          br_pseg2;                 /* Bitrate Time Segment 2 */
} CANDEV_FLEXCAN_INIT_INFO;

/* General Device Information Structure */
typedef struct candev_flexcan_info_entry
{
    uintptr_t                        base;     /* Device Virtual Register Mapping */
    can_msg_obj_t                    *canmsg;  /* Array of CAN message objects */
    CANDEV_MODE                      mode;     /* Driver mode:  raw frames or I/O */
    uintptr_t                        canlam;   /* Array of CAN message object local area masks */
    uint32_t                         timer;    /* CAN message time stamps */
    volatile uint32_t                canestat; /* Keep track of CANESR register status for devctl */
    struct candev_flexcan_entry      *devlist; /* Array of all device mailboxes */
    int                              iidsys;   /* Return iid from InterruptAttach */
    uint32_t                         numtx;    /* Number of TX Mailboxes */
    uint32_t                         numrx;    /* Number of RX Mailboxes */
    uint32_t                         iflags;   /* Info flags */
    struct can_devctl_stats          stats;
    minican_data_t                   *mdata;   /* Mini-driver data */
    can_msg_obj_t                    *mcanmsg; /* Mini-driver buffered CAN messages */
    struct candev_flexcan_init_info  initinfo; /* Initialization info */
} CANDEV_FLEXCAN_INFO;

/* Device specific extension of CANDEV struct */
typedef struct candev_flexcan_entry
{
    CANDEV                        cdev;        /* CAN Device - MUST be first entry */
    int                           mbxid;       /* Index into mailbox memory */
    volatile uint32_t             dflags;      /* Device specific flags */
    CANDEV_FLEXCAN_INFO           *devinfo;    /* Common device information */
} CANDEV_FLEXCAN;


#endif


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/can/public/hw/mx7_can.h $ $Rev: 893378 $")
#endif
