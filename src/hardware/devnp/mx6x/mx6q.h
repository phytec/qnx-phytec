/*
 * $QNXLicenseC:
 * Copyright 2010, QNX Software Systems.
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

#ifndef NIC_MX6Q_H_INCLUDED
#define NIC_MX6Q_H_INCLUDED

// Freescale i.mx 6Q ENET

#include <io-pkt/iopkt_driver.h>

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

#include <sys/syslog.h>
#include <sys/slogcodes.h>
#include <sys/cache.h>
#include <sys/param.h>
#include <sys/syspage.h>
#include <sys/malloc.h>
#include <sys/mman.h>
#include <sys/io-pkt.h>
#include <sys/callout.h>
#include <sys/mbuf.h>
#include <sys/ioctl.h>
#include <sys/neutrino.h>
#include <sys/netmgr.h>
#include <sys/slog2.h>

#include <netdrvr/mdi.h>
#include <netdrvr/nicsupport.h>
#include <netdrvr/common.h>
#include <netdrvr/ptp.h>
#include <netdrvr/smmu.h>

#include <hw/nicinfo.h>

#include <net/if.h>
#include <net/if_dl.h>
#include <net/if_ether.h>
#include <net/if_types.h>

#include <quiesce.h>
#include <siglock.h>
#include <nw_thread.h>

#include <sys/device.h>
#include <net/if_media.h>
#include <dev/mii/miivar.h>
#include <sys/hwinfo.h>
#include <drvr/hwinfo.h>
#include <sys/kthread.h>
#if (_NTO_VERSION < 660)
#include <drvr/nic_mutex.h>
#else
#include <netdrvr/nic_mutex.h>
#endif
#include "nxp.h"
#include "dp83867.h"

#ifdef MX8XP
#include "aarch64/mx8x.h"
#endif

#ifdef MX7D
    #define MX6Q_DEVICE_DESCRIPTION "i.MX7"
    #define MX6Q_RX_THREAD_NAME     "mx7 Rx"
#elif MX8XP
    #define MX6Q_DEVICE_DESCRIPTION "i.MX8"
    #define MX6Q_RX_THREAD_NAME     "mx8 Rx"
#elif S32V
    #define MX6Q_DEVICE_DESCRIPTION "S32V"
    #define MX6Q_RX_THREAD_NAME     "s32v Rx"
#else
    #define MX6Q_DEVICE_DESCRIPTION "i.MX6"
    #define MX6Q_RX_THREAD_NAME     "mx6 Rx"
#endif

#define MX6Q_MAP_SIZE       4096

#define MX6Q_QUIESCE_PULSE  _PULSE_CODE_MINAVAIL
#define MX6Q_RX_PULSE       (MX6Q_QUIESCE_PULSE + 1)

/*
 * Constants used with ENET1 and ENET2 to control the Phy Address search
 */

/* metadata for control */
#define HWITAG_PHY_ADDR_METADATA_MASK   0xffff0000
#define HWITAG_IS_AN_ON_BOARD_PHY_ADDR  0x000f0000

#define CONFIG_PHY_ADDR_METADATA_MASK   0xf0000000
#define CONFIG_HAS_DEFINED_PHY_ADDR     0x50000000

/* address mask for explicit defintion of phy address */
#define HWITAG_PHY_ADDR_MASK            0x0000001f

/*
 * Timer registers
 */
#ifdef MX8XP
#define TICK_TIME       41U /* 24MHz Peripheral Clock, so 41ns per ticks */
#define GPT_INT         IMX_GPT1_IRQ
#define GPT_BASE        IMX_GPT1_BASE
#else
#define TICK_TIME       15 /* 66MHz Peripheral Clock, so 15ns ticks */
#define GPT_INT         87
#define GPT_BASE        0x2098000
#endif

#define GPT_CR              (0x00 >> 2)
    #define GPT_CR_SWR          (1 << 15)
    #define GPT_CR_FRR          (1 << 9)
    #define GPT_CR_CLKSRC_PERI  (1 << 6)
    #define GPT_CR_STOPEN       (1 << 5)
    #define GPT_CR_DOZEEN       (1 << 4)
    #define GPT_CR_WAITEN       (1 << 3)
    #define GPT_CR_DBGEN        (1 << 2)
    #define GPT_CR_ENMOD        (1 << 1)
    #define GPT_CR_EN           (1 << 0)

#define GPT_PR              (0x04 >> 2)

#define GPT_SR              (0x08 >> 2)

#define GPT_IR              (0x0c >> 2)
    #define GPT_IR_ROVIE            (1 << 5)
    #define GPT_IR_IF2IE            (1 << 4)
    #define GPT_IR_IF1IE            (1 << 3)
    #define GPT_IR_OF3IE            (1 << 2)
    #define GPT_IR_OF2IE            (1 << 1)
    #define GPT_IR_OF1IE            (1 << 0)

#define GPT_OCR1            (0x10 >> 2)

#define GPT_OCR2            (0x14 >> 2)

#define GPT_OCR3            (0x18 >> 2)

#define GPT_ICR1            (0x1c >> 2)

#define GPT_ICR2            (0x20 >> 2)

#define GPT_CNT             (0x24 >> 2)

/*
 * Note that in general a TCP stream sends packets in 3 descriptors:
 * header, last part of previous cluster, first part of next cluster.
 * Extending this to a jumbo frame of 9k gives a header + 6 clusters.
 * Allow a few extras to try and avoid defragging all the time.
 */
#define MX6Q_MAX_FRAGS          10

/*
 * Freescale have confirmed the spec is wrong, only bits 6-10 should be set.
 * N.B. This is used as a mask as well as a max.
 */
#define MX6Q_MAX_RBUFF_SIZE     1984

/*
 * io-pkt defaults to 8192 clusters of 2048 bytes each.
 * On an mx6x-slx we have two interfaces each with 3 queues of Tx and Rx.
 * We need to ensure that we don't use all the clusters here.
 * Defaults of 256 seem reasonable.
 */

#define MIN_NUM_RX_DESCRIPTORS      16
#define MIN_NUM_TX_DESCRIPTORS      64

#define DEFAULT_NUM_RX_DESCRIPTORS  256
#define DEFAULT_NUM_TX_DESCRIPTORS  256

#define MAX_NUM_RX_DESCRIPTORS      2048
#define MAX_NUM_TX_DESCRIPTORS      2048

#if defined MX6XSLX || defined MX7D || defined MX8XP
    #define MAX_NUM_QUEUES              3
#else
    #define MAX_NUM_QUEUES              1
#endif

#define DEFAULT_NUM_RX_QUEUES       1
#define DEFAULT_NUM_TX_QUEUES       1

#define RX_SECTION_FULL_THRESHOLD   0x10
#define RX_SECTION_EMPTY_THRESHOLD  0x82
#define RX_ALMOST_EMPTY_THRESHOLD   0x08
#define RX_ALMOST_FULL_THRESHOLD    0x08

#if defined MX6XSLX || defined MX8XP
/*
 * Set the default values used for the interrupt coalescence
 * feature of the SoloX
 */
#define RX_FRAME_DEFAULT        120
#define RX_DELAY_DEFAULT        240
#define TX_FRAME_DEFAULT        255
#define TX_DELAY_DEFAULT        240
#endif

#define MPC_TIMEOUT     1000
#define STOP_TIMEOUT	100

/* ENET General Control and Status Registers */
/* Control/Status Registers */

#define MX6Q_ENET_ID                (0x0000)
#define MX6Q_IEVENT                 (0x0004 >> 2)
        #define IEVENT_BABR         (1 << 30)
        #define IEVENT_BABT         (1 << 29)
        #define IEVENT_GRA          (1 << 28)
        #define IEVENT_TFINT        (1 << 27)
        #define IEVENT_TXB          (1 << 26)
        #define IEVENT_RFINT        (1 << 25)
        #define IEVENT_RXB          (1 << 24)
        #define IEVENT_MII          (1 << 23)
        #define IEVENT_EBERR        (1 << 22)
        #define IEVENT_LATE_COL     (1 << 21)
        #define IEVENT_COL_RET_LIM  (1 << 20)
        #define IEVENT_XFIFO_UN     (1 << 19)
        #define IEVENT_PLR          (1 << 18)
        #define IEVENT_WAKEUP       (1 << 17)
    #define IEVENT_TS_AVAIL         (1 << 16)
    #define IEVENT_TS_TIMER         (1 << 15)
    #define IEVENT_RXFLUSH2         (1 << 14)
    #define IEVENT_RXFLUSH1         (1 << 13)
    #define IEVENT_RXFLUSH0         (1 << 12)
    #define IEVENT_TXF2             (1 << 7)
    #define IEVENT_TXB2             (1 << 6)
    #define IEVENT_RXF2             (1 << 5)
    #define IEVENT_RXB2             (1 << 4)
    #define IEVENT_TXF1             (1 << 3)
    #define IEVENT_TXB1             (1 << 2)
    #define IEVENT_RXF1             (1 << 1)
    #define IEVENT_RXB1             (1 << 0)


#define MX6Q_IMASK                  (0x0008 >> 2)   /* Interrupt Mask Register */
        #define IMASK_BREN          (1 << 30)
        #define IMASK_BTEN          (1 << 29)
        #define IMASK_GRAEN         (1 << 28)
        #define IMASK_TFIEN         (1 << 27)
        #define IMASK_TBIEN         (1 << 26)
        #define IMASK_RFIEN         (1 << 25)
        #define IMASK_RBIEN         (1 << 24)
        #define IMASK_MIIEN         (1 << 23)
        #define IMASK_EBERREN       (1 << 22)
        #define IMASK_LCEN          (1 << 21)
        #define IMASK_CRLEN         (1 << 20)
        #define IMASK_XFUNEN        (1 << 19)
        #define IMASK_XFERREN       (1 << 18)
        #define IMASK_RFERREN       (1 << 17)
        #define IMASK_TS_AVAIL      (1 << 16)
        #define IMASK_TS_TIMER      (1 << 15)
    #define IMASK_RXFLUSH2EN        (1 << 14)
    #define IMASK_RXFLUSH1EN        (1 << 13)
    #define IMASK_RXFLUSH0EN        (1 << 12)
    #define IMASK_TXF2EN            (1 << 7)
    #define IMASK_TXB2EN            (1 << 6)
    #define IMASK_RXF2EN            (1 << 5)
    #define IMASK_RXB2EN            (1 << 4)
    #define IMASK_TXF1EN            (1 << 3)
    #define IMASK_TXB1EN            (1 << 2)
    #define IMASK_RXF1EN            (1 << 1)
    #define IMASK_RXB1EN            (1 << 0)

#define MX6Q_R_DES_ACTIVE               (0x0010 >> 2)
        #define R_DES_ACTIVE            (1 << 24)

#define MX6Q_X_DES_ACTIVE               (0x0014 >> 2)
        #define X_DES_ACTIVE            (1 << 24)

#define MX6Q_ECNTRL                 (0x0024 >> 2)
    #define ECNTRL_SVLANDBL         (1 << 11)
    #define ECNTRL_VLANUSE2ND       (1 << 10)
    #define ECNTRL_SVLANEN          (1 << 9)
    #define ECNTRL_DBSWP            (1 << 8)
    #define ECNTRL_DBG_EN           (1 << 6)
    #define ECNTRL_ETH_SPEED        (1 << 5)
    #define ECNTRL_ENA_1588         (1 << 4)
    #define ECNTRL_SLEEP            (1 << 3)
        #define ECNTRL_ENET_OE      (1 << 2)
        #define ECNTRL_ETHER_EN     (1 << 1)
        #define ECNTRL_RESET        (1 << 0)

#define MX6Q_MII_DATA               (0x0040 >> 2)
        #define MAC_MMFR_ST         (1 << 30)
        #define MAC_MMFR_OP         (1 << 28)
        #define MAC_MMFR_PA         (1 << 23)
        #define MAC_MMFR_RA         (1 << 18)
        #define MAC_MMFR_TA         (1 << 16)
        #define MAC_MMFR_DATA       (1 << 0)

#define MX6Q_MII_SPEED              (0x0044 >> 2)
        #define DIS_RSRVD0          (1 << 11)
        #define DIS_HOLDTIME        (1 << 8)
        #define DIS_PREAMBLE        (1 << 7)
        #define DIS_SPEED           (1 << 1)
        #define DIS_RSRVD1          (1 << 0)
#ifdef MX8XP
        #define MII_SPEED_SHIFT     (24 << 1)
        #define MII_HOLDTIME_SHIFT  (1 << 8)
#else
        #define MII_SPEED_SHIFT     (13 << 1)
        #define MII_HOLDTIME_SHIFT  (1 << 8)
#endif

#define MX6Q_MIB_CONTROL            (0x0064 >> 2)
        #define MIB_DISABLE         (1 << 31)
        #define MIB_IDLE            (1 << 30)
        #define MIB_CLEAR           (1 << 29)

#define MX6Q_R_CNTRL                            (0x0084 >> 2)
        #define RCNTRL_GRS                      (1 << 31)
        #define RCNTRL_NO_LGTH_CHECK            (1 << 30)
        #define RCNTRL_MAX_FL                   (1 << 16)
        #define RCNTRL_FRM_ENA                  (1 << 15)
        #define RCNTRL_CRC_FWD                  (1 << 14)
        #define RCNTRL_PAUSE_FWD                (1 << 13)
        #define RCNTRL_PAD_EN                   (1 << 12)
        #define RCNTRL_RMII_ECHO                (1 << 11)
        #define RCNTRL_RMII_LOOP                (1 << 10)
        #define RCNTRL_RMII_10T                 (1 << 9)
        #define RCNTRL_RMII_MODE                (1 << 8)
        #define RCNTRL_SGMII_ENA                (1 << 7)
        #define RCNTRL_RGMII_ENA                (1 << 6)
        #define RCNTRL_FCE                      (1 << 5)
        #define RCNTRL_BC_REJ                   (1 << 4)
        #define RCNTRL_PROM                     (1 << 3)
        #define RCNTRL_MII_MODE                 (1 << 2)
        #define RCNTRL_DRT                      (1 << 1)
        #define RCNTRL_LOOP                     (1 << 0)

#define MX6Q_X_CNTRL                            (0x00c4 >> 2)
        #define XCNTRL_TX_CRC_FWD               (1 << 9)
        #define XCNTRL_TX_ADDR_INS              (1 << 8)
        #define XCNTRL_TX_ADDR_SEL              (1 << 5)
        #define XCNTRL_RFC_PAUSE                (1 << 4)
        #define XCNTRL_TFC_PAUSE                (1 << 3)
        #define XCNTRL_FDEN                     (1 << 2)
        #define XCNTRL_HBC                      (1 << 1)
        #define XCNTRL_GTS                      (1 << 0)

#define MX6Q_TIMER_CTRLR                        (0x0400 >> 2)
        #define MX6Q_TIMER_CTRL_SLAVE           (1 << 13)
        #define MX6Q_TIMER_CTRL_CAPTURE         (1 << 11)
        #define MX6Q_TIMER_CTRL_RESTART         (1 << 9)
        #define MX6Q_TIMER_CTRL_PINPER          (1 << 7)
        #define MX6Q_TIMER_CTRL_PEREN           (1 << 4)
        #define MX6Q_TIMER_CTRL_OFFRST          (1 << 3)
        #define MX6Q_TIMER_CTRL_OFFEN           (1 << 2)
        #define MX6Q_TIMER_CTRL_EN              (1 << 0)
#define MX6Q_TIMER_VALUER                       (0x0404 >> 2)
#define MX6Q_TIMER_OFFSETR                      (0x0408 >> 2)
#define MX6Q_TIMER_PERR                         (0x040c >> 2)
#define MX6Q_TIMER_CORR                         (0x0410 >> 2)
#define MX6Q_TIMER_INCR                         (0x0414 >> 2)
#define MX6Q_TIMER_TSTMP                        (0x0418 >> 2)
#define MX6Q_TIMER_INCR_MASK                    0x0000007f
#define MX6Q_TIMER_INCR_CORR_OFF                8
#define MX6Q_TIMER_PER1SEC                      1000000000  // Periodic event interval(ns)

#define MX6Q_TGSR				(0x604 >> 2)
	#define MX6Q_TGSR_TF0				(1 << 31)
	#define MX6Q_TGSR_TF1				(1 << 30)
	#define MX6Q_TGSR_TF2				(1 << 29)
	#define MX6Q_TGSR_TF3				(1 << 28)
#define MX6Q_TCSR0				(0x608 >> 2)
#define MX6Q_TCSR1				(0x610 >> 2)
#define MX6Q_TCSR2				(0x618 >> 2)
#define MX6Q_TCSR3				(0x620 >> 2)
	#define MX6Q_TCSR_TDRE				(1 << 0)
	#define MX6Q_TCSR_MODE_MASK			0x3c
	#define MX6Q_TCSR_MODE_SHIFT			2
	#define MX6Q_TCSR_MODE_OUT_SW			4
	#define MX6Q_TCSR_TIE				(1 << 6)
	#define MX6Q_TCSR_TF				(1 << 7)
#define MX6Q_TCCR0				(0x60c >> 2)
#define MX6Q_TCCR1				(0x614 >> 2)
#define MX6Q_TCCR2				(0x61c >> 2)
#define MX6Q_TCCR3				(0x624 >> 2)
	#define MX6Q_TIMER_PER05SEC                      500000000  // Half rollover

#define MX6Q_PADDR1                             (0x00e4 >> 2)
#define MX6Q_PADDR2                             (0x00e8 >> 2)
#define MX6Q_OP_PAUSE                           (0x00ec >> 2)

#define MX6Q_X_ITRPT_COALESCING0                (0x00F0 >> 2)
#define MX6Q_X_ITRPT_COALESCING1                (0x00F4 >> 2)
#define MX6Q_X_ITRPT_COALESCING2                (0x00F8 >> 2)
    #define ITRPT_COALESCING_EN                 (1 << 31)
    #define ITRPT_COALESCING_CLK_ENET           (1 << 30)

#define MX6Q_R_ITRPT_COALESCING0                (0x0100 >> 2)
#define MX6Q_R_ITRPT_COALESCING1                (0x0104 >> 2)
#define MX6Q_R_ITRPT_COALESCING2                (0x0108 >> 2)

#define MX6Q_IADDR1                             (0x0118 >> 2)
#define MX6Q_IADDR2                             (0x011c >> 2)
#define MX6Q_GADDR1                             (0x0120 >> 2)
#define MX6Q_GADDR2                             (0x0124 >> 2)

#define MX6Q_X_WMRK                             (0x0144 >> 2)
        #define X_WMRK_STR_FWD                  (1 << 8)
        #define X_WMRK_TFWR                     (1 << 0)

#define MX6Q_R_DES_START1                       (0x0160 >> 2)
#define MX6Q_X_DES_START1                       (0x0164 >> 2)
#define MX6Q_R_BUFF_SIZE1                       (0x0168 >> 2)
#define MX6Q_R_DES_START2                       (0x016C >> 2)
#define MX6Q_X_DES_START2                       (0x0170 >> 2)
#define MX6Q_R_BUFF_SIZE2                       (0x0174 >> 2)
#define MX6Q_R_DES_START                        (0x0180 >> 2)
#define MX6Q_X_DES_START                        (0x0184 >> 2)
#define MX6Q_R_BUFF_SIZE                        (0x0188 >> 2)
#define MX6Q_R_SECTION_FULL_ADDR                (0x0190 >> 2)
#define MX6Q_R_SECTION_EMPTY_ADDR               (0x0194 >> 2)
#define MX6Q_R_ALMOST_EMPTY_ADDR                (0x0198 >> 2)
#define MX6Q_R_ALMOST_FULL_ADDR                 (0x019c >> 2)
#define MX6Q_T_SECTION_EMPTY_ADDR               (0x01a0 >> 2)
#define MX6Q_T_ALMOST_EMPTY_ADDR                (0x01a4 >> 2)
#define MX6Q_T_ALMOST_FULL_ADDR                 (0x01a8 >> 2)
#define MX6Q_IPG_LENGTH_ADDR                    (0x01ac >> 2)
#define MX6Q_TRUNC_FL_ADDR                      (0x01b0 >> 2)
#define MX6Q_IPACCTXCONF_ADDR                   (0x01c0 >> 2)
#define MX6Q_IPACCRXCONF_ADDR                   (0x01c4 >> 2)

#define MX6Q_R_CLS_MATCH1           (0x01C8 >> 2)
#define MX6Q_R_CLS_MATCH2           (0x01CC >> 2)
    #define RCV_CLS_MATCHEN         (1 << 16)
    #define RCV_CMP0_SHIFT           0
    #define RCV_CMP1_SHIFT           4
    #define RCV_CMP2_SHIFT           8
    #define RCV_CMP3_SHIFT           12

#define MX6Q_DMACFG1                (0x01D8 >> 2)
#define MX6Q_DMACFG2                (0x01DC >> 2)
    #define DMACFG_CALC_NOIPGEN     (1 << 17)
    #define DMACFG_DMA_CLASSEN      (1 << 16)

#define MX6Q_R_DES_ACTIVE1          (0x01E0 >> 2)
#define MX6Q_X_DES_ACTIVE1          (0x01E4 >> 2)
#define MX6Q_R_DES_ACTIVE2          (0x01E8 >> 2)
#define MX6Q_X_DES_ACTIVE2          (0x01EC >> 2)

#define MX6Q_QOS_SCHEME             (0x01F0 >> 2)
    #define QOS_SCHEME_RX_FLUSH2    (1 <<  5)
    #define QOS_SCHEME_RX_FLUSH1    (1 <<  4)
    #define QOS_SCHEME_RX_FLUSH0    (1 <<  3)

#if defined MX6XSLX || defined MX8XP

#define MX6SLX_TXIC0                (0x00F0 >> 2)
#define MX6SLX_TXIC1                (0x00F4 >> 2)
#define MX6SLX_TXIC2                (0x00F8 >> 2)
    #define TXIC_ICEN               (1 << 31)
    #define TXIC_ICFT(x)            ((x & 0xff) << 20)
    #define TXIC_ICTT(x)            (x & 0xffff)
#define MX6SLX_RXIC0                (0x0100 >> 2)
#define MX6SLX_RXIC1                (0x0104 >> 2)
#define MX6SLX_RXIC2                (0x0108 >> 2)
    #define RXIC_ICEN               TXIC_ICEN
    #define RXIC_ICFT(x)            TXIC_ICFT(x)
    #define RXIC_ICTT(x)            TXIC_ICTT(x)
#endif

/* MIB Block Counters */

#define MX6Q_RMON_T_DROP        (0x0200 >> 2)
#define MX6Q_RMON_T_PACKETS     (0x0204 >> 2)
#define MX6Q_RMON_T_BC_PKT      (0x0208 >> 2)
#define MX6Q_RMON_T_MC_PKT      (0x020c >> 2)
#define MX6Q_RMON_T_CRC_ALIGN   (0x0210 >> 2)
#define MX6Q_RMON_T_UNDERSIZE   (0x0214 >> 2)
#define MX6Q_RMON_T_OVERSIZE    (0x0218 >> 2)
#define MX6Q_RMON_T_FRAG        (0x021c >> 2)
#define MX6Q_RMON_T_JAB         (0x0220 >> 2)
#define MX6Q_RMON_T_COL         (0x0224 >> 2)
#define MX6Q_RMON_T_P64         (0x0228 >> 2)
#define MX6Q_RMON_T_P65TO127    (0x022c >> 2)
#define MX6Q_RMON_T_P128TO255   (0x0230 >> 2)
#define MX6Q_RMON_T_P256TO511   (0x0234 >> 2)
#define MX6Q_RMON_T_P512TO1023  (0x0238 >> 2)
#define MX6Q_RMON_T_P1024TO2047 (0x023c >> 2)
#define MX6Q_RMON_T_P_GTE2048   (0x0240 >> 2)
#define MX6Q_RMON_T_OCTETS      (0x0244 >> 2)
#define MX6Q_IEEE_T_DROP        (0x0248 >> 2)
#define MX6Q_IEEE_T_FRAME_OK    (0x024c >> 2)
#define MX6Q_IEEE_T_1COL        (0x0250 >> 2)
#define MX6Q_IEEE_T_MCOL        (0x0254 >> 2)
#define MX6Q_IEEE_T_DEF         (0x0258 >> 2)
#define MX6Q_IEEE_T_LCOL        (0x025c >> 2)
#define MX6Q_IEEE_T_EXCOL       (0x0260 >> 2)
#define MX6Q_IEEE_T_MACERR      (0x0264 >> 2)
#define MX6Q_IEEE_T_CSERR       (0x0268 >> 2)
#define MX6Q_IEEE_T_SQE         (0x026c >> 2)
#define MX6Q_T_FDXFC            (0x0270 >> 2)
#define MX6Q_IEEE_T_OCTETS_OK   (0x0274 >> 2)
#define MX6Q_RMON_R_PACKETS     (0x0284 >> 2)
#define MX6Q_RMON_R_BC_PKT      (0x0288 >> 2)
#define MX6Q_RMON_R_MC_PKT      (0x028c >> 2)
#define MX6Q_RMON_R_CRC_ALIGN   (0x0290 >> 2)
#define MX6Q_RMON_R_UNDERSIZE   (0x0294 >> 2)
#define MX6Q_RMON_R_OVERSIZE    (0x0298 >> 2)
#define MX6Q_RMON_R_FRAG        (0x029c >> 2)
#define MX6Q_RMON_R_JAB         (0x02a0 >> 2)
#define MX6Q_RMON_R_P64         (0x02a8 >> 2)
#define MX6Q_RMON_R_P65TO127    (0x02ac >> 2)
#define MX6Q_RMON_R_P128TO255   (0x02b0 >> 2)
#define MX6Q_RMON_R_P256TO511   (0x02b4 >> 2)
#define MX6Q_RMON_R_P512TO1023  (0x02b8 >> 2)
#define MX6Q_RMON_R_P1024TO2047 (0x02bc >> 2)
#define MX6Q_RMON_R_P_GTE2048   (0x02c0 >> 2)
#define MX6Q_RMON_R_OCTETS      (0x02c4 >> 2)
#define MX6Q_IEEE_R_DROP        (0x02c8 >> 2)
#define MX6Q_IEEE_R_FRAME_OK    (0x02cc >> 2)
#define MX6Q_IEEE_R_CRC         (0x02d0 >> 2)
#define MX6Q_IEEE_R_ALIGN       (0x02d4 >> 2)
#define MX6Q_IEEE_R_MACERR      (0x02d8 >> 2)
#define MX6Q_R_FDXFC            (0x02dc >> 2)
#define MX6Q_IEEE_OCTETS_OK     (0x02e0 >> 2)

/* Transmit/receive buffer descriptor */

typedef struct {
    uint16_t   length;             // Data length
    uint16_t   status;             // Status field
    uint32_t   buffer;             // Data buffer
    uint32_t   estatus;            // Enhanced status
    uint16_t   payload_chksum;     // Payload checksum
    uint8_t    ptype;              // Protocol type
    uint8_t    header_length;      // Header length
    uint32_t   bdu;                // BDU field
    uint32_t   timestamp;          // Timestamp of the frame
    uint16_t   reserved[4];
} __attribute__((__packed__)) mpc_bd_t;

#define TXBD_R                      (1 << 15)           /* Ready */
#define TXBD_TO1                    (1 << 14)           /* Transmit Ownership */
#define TXBD_W                      (1 << 13)           /* Wrap */
#define TXBD_TO2                    (1 << 12)           /* Transmit Ownership */
#define TXBD_L                      (1 << 11)           /* Last */
#define TXBD_TC                     (1 << 10)           /* Tx CRC */
#define TXBD_ABC                    (1 << 9)            /* Append bad CRC */

#define RXBD_E                      (1 << 15)           /* Empty */
#define RXBD_RO1                    (1 << 14)           /* Receive software ownership bit */
#define RXBD_W                      (1 << 13)           /* Wrap */
#define RXBD_RO2                    (1 << 12)           /* Receive Ownership */
#define RXBD_L                      (1 << 11)           /* Last in frame */
#define RXBD_M                      (1 << 8)            /* Miss */
#define RXBD_BC                     (1 << 7)            /* Broadcast */
#define RXBD_MC                     (1 << 6)            /* Multicast */
#define RXBD_LG                     (1 << 5)            /* Rx frame length violation */
#define RXBD_NO                     (1 << 4)            /* Rx non-octet aligned frame */
#define RXBD_SH                     (1 << 3)            /* Short frame */
#define RXBD_CR                     (1 << 2)            /* Rx CRC error */
#define RXBD_OV                     (1 << 1)            /* Overrun */
#define RXBD_TR                     (1 << 0)            /* Truncation */
#define RXBD_ERR                    (RXBD_TR | RXBD_OV | RXBD_CR | RXBD_SH | RXBD_NO | RXBD_LG)

#define RXBD_ESTATUS_ME             (1 << 31)           /* MAC Error */
#define RXBD_ESTATUS_PE             (1 << 26)           /* PHY Error */
#define RXBD_ESTATUS_CE             (1 << 25)           /* Collision detected */
#define RXBD_ESTATUS_UC             (1 << 24)           /* Unicast frame */
#define RXBD_ESTATUS_INT            (1 << 23)           /* Generate RXB/RXF interrupt */
#define RXBD_ESTATUS_ICE            (1 << 5)            /* IP header checksum error */
#define RXBD_ESTATUS_PCR            (1 << 4)            /* Protocol checksum error */
#define RXBD_ESTATUS_VLAN           (1 << 2)            /* Frame has a VLAN tag */
#define RXBD_ESTATUS_IPV6           (1 << 1)            /* Frame has a IPv6 frame type */
#define RXBD_ESTATUS_FRAG           (1 << 0)            /* Frame is an IPv4 fragment frame */

#define TXBD_ESTATUS_INT            (1 << 30)           /* Generate interrupt */
#define TXBD_ESTATUS_TS             (1 << 29)           /* Generate timestamp frame */
#define TXBD_ESTATUS_PINS           (1 << 28)           /* Insert protocol checksum */
#define TXBD_ESTATUS_IINS           (1 << 27)           /* Insert IP header checksum */
#define TXBD_ESTATUS_TXE            (1 << 15)           /* Transmit error occured */
#define TXBD_ESTATUS_UE             (1 << 13)           /* Underflow error */
#define TXBD_ESTATUS_EE             (1 << 12)           /* Excess Collision error */
#define TXBD_ESTATUS_FE             (1 << 11)           /* Frame with error */
#define TXBD_ESTATUS_LCE            (1 << 10)           /* Late collision error */
#define TXBD_ESTATUS_OE             (1 << 9)            /* Overflow error */
#define TXBD_ESTATUS_TSE            (1 << 8)            /* Timestamp error */

#define BD_BDU              (1 << 31)       /* Buffer Descriptor Update done */

#define NEXT_TX(x)              ((x + 1) % mx6q->num_tx_descriptors)
#define NEXT_RX(x)              ((x + 1) % mx6q->num_rx_descriptors)
#define PREV_TX(x)              ((x == 0) ? mx6q->num_tx_descriptors - 1 : x - 1)

#define MX6Q_TX_TIMESTAMP_BUF_SZ    16      // Amount of timestamps of transmitted packets
#define MX6Q_RX_TIMESTAMP_BUF_SZ    64      // Amount of timestamps of received packets

#define MX6Q_SQI_SAMPLING_INTERVAL  1       // SQI sampling interval, in second

#ifdef MX7D
    #define CLOCK_FREQ  25000000
#else
    #define CLOCK_FREQ  0
#endif

typedef enum {
  MX6_FLOW_AUTO = -1,
  MX6_FLOW_NONE,
  MX6_FLOW_BOTH,
  MX6_FLOW_TX,
  MX6_FLOW_RX
} mx6_flow_t;

typedef enum {
  MX6_FUNC_MODE_DEFAULT = 0,
  MX6_FUNC_MODE_AVB
} mx6_func_mode_t;

typedef struct _nic_mx6q_ext {
    struct device       dev;
    struct ethercom     ecom;

    int                 tid;
    int                 chid;
    int                 coid;
    struct sigevent     isr_event[MAX_NUM_QUEUES];
    struct _iopkt_inter inter;
    struct _iopkt_inter inter_queue;
    int                 iid_rx;
    int                 iid_tx;
    int                 iid_err;
    int                 iid_1588;
    void               *sdhook;

    nic_config_t        cfg;
    nic_stats_t         stats;
    nic_stats_t         old_stats;

    struct _iopkt_self *iopkt;

    //
    // rx
    //
    int                 rxd_pkts;
    int                 num_rx_descriptors;
    mpc_bd_t           *rx_bd;
    int                 rx_cidx[MAX_NUM_QUEUES];
    struct mbuf       **rx_pkts;
    pthread_mutex_t     rx_mutex;
    int                 rx_running;
    int                 rx_full;
    struct ifqueue      rx_queue;

    struct mbuf        *rpkt[MAX_NUM_QUEUES];
    struct mbuf        *rpkt_tail[MAX_NUM_QUEUES];
    uint32_t            length[MAX_NUM_QUEUES];
    uint32_t            num_rx_queues;

    //
    // cmd line args and state variables
    //
    int                 clock_freq_set;
    uint32_t            clock_freq;
    int                 dying;

    // Note: Most of the time phy_base == iobase, and phy_reg == reg,
    //       but on some board the MDIO interface is shared. In this
    //       case phy_base and phy_reg is a window into another nic's
    //       registers.
    uintptr_t           iobase;
    uintptr_t           phy_base;
    uintptr_t           tbase;
    uint32_t           *reg;
    uint32_t           *phy_reg;
    uint32_t           *treg;
    int                 tirq;

#if defined MX6XSLX | defined MX8XP
    //
    // interrupt coalescence
    //
    uint32_t            rx_frame;
    uint32_t            rx_delay;
    uint32_t            tx_frame;
    uint32_t            tx_delay;
#endif

    //
    // mii
    //
    struct callout      mii_callout;
    mdi_t              *mdi;
    uint32_t            probe_phy;
    struct mii_data     bsd_mii;        // for media devctls
    int			set_speed;
    int			set_duplex;
    mx6_flow_t		set_flow;
    mx6_flow_t		flow_status;
    int                 mii;
    int                 rmii;
    int                 brmast; // Sets BroadrReach role as master (1) or slave (0)
    int                 iout;   // Settings for BCM89811 PHY
    int                 slew;   // As above
    int                 cl45;   // use clause 45 to access the PHY

    struct cache_ctrl   cachectl;
    int (*stack_output)(struct ifnet *, struct mbuf *,
                struct sockaddr *, struct rtentry *);
    //
    // tx variables - hopefully a cache line or two away from
    // the rx variables above
    //
    int                 num_tx_descriptors; // per queue
    mpc_bd_t           *tx_bd;
    int                 tx_descr_inuse[MAX_NUM_QUEUES];
    int                 tx_pidx[MAX_NUM_QUEUES]; // descr producer index
    int                 tx_cidx[MAX_NUM_QUEUES]; // descr consumer index
    volatile bool       tx_full;
    struct mbuf       **tx_pkts;
    int                 tx_reaped;      // flag for periodic descr ring cleaning
    uint32_t            num_tx_queues;
    // Real Time Clock value (sec.)
    volatile uint32_t   rtc;
    volatile uint32_t	rtc_half; /* Half rollover flag */
    //sqi
    uint8_t             sqi;
    struct callout      sqi_callout;
    intrspin_t          spinlock;
	nic_mutex_t			mii_mutex;

	mx6_func_mode_t     mode;
    int                 fd;    // to use io-pkt typed memory for descriptors

    // physical addresses for smmu man
    paddr_t             tx_descriptor_area_phys;
    paddr_t             rx_descriptor_area_phys;
} mx6q_dev_t;

// event.c
const struct sigevent * mx6q_isr(void *, int);
int mx6q_enable_interrupt(void *arg);
const struct sigevent * mx6q_isr_rx(void *, int);
int mx6q_enable_rx(void *arg);
const struct sigevent * mx6q_1588_isr(void *, int);
int mx6q_process_interrupt(void *arg, struct nw_work_thread *);
int mx6q_process_interrupt_rx(void *arg, struct nw_work_thread *);
int mx6q_enable_queue(void *arg);
int mx6q_process_queue(void *arg, struct nw_work_thread *);

// multicast.c
void mx6q_set_multicast(mx6q_dev_t *);

// detect.c
void dump_mbuf(struct mbuf *, uint32_t);
int mx6q_detect(void *dll_hdl, struct _iopkt_self *iopkt, char *options);
void mx6q_speeduplex(mx6q_dev_t *);

/* devctl.c */
int mx6q_ioctl(struct ifnet *, unsigned long, caddr_t);
int mx6q_sqi_ioctl(mx6q_dev_t *, struct ifdrv *);
int mx6q_mii_read_ioctl(mx6q_dev_t *, struct ifdrv *);
int mx6q_mii_write_ioctl(mx6q_dev_t *, struct ifdrv *);
int mx6q_br_lowpower_enable_ioctl(mx6q_dev_t *mx6q, struct ifdrv *ifd);
int mx6q_br_lowpower_disable_ioctl(mx6q_dev_t *mx6q, struct ifdrv *ifd);
int mx6q_mii_op_ioctl(mx6q_dev_t *, struct ifdrv *);
int mx6q_phy_funcs(mx6q_dev_t *mx6q, struct ifdrv *ifd);

// transmit.c
void mx6q_start(struct ifnet *);
void mx6q_transmit_complete(mx6q_dev_t *, uint8_t);
int mx6q_set_tx_bw (mx6q_dev_t *mx6q, struct ifdrv *ifd);
int mx6q_output (struct ifnet *, struct mbuf *,
         struct sockaddr *, struct rtentry *);
// receive.c
int mx6q_receive(mx6q_dev_t *, struct nw_work_thread *, uint8_t);

// mii.c
void mx6q_MDI_MonitorPhy(void *);
int mx6q_init_phy(mx6q_dev_t *);
void mx6q_fini_phy(mx6q_dev_t *);
int mx6_is_br_phy (mx6q_dev_t *);
uint16_t mx6q_mii_read (void *handle, uint8_t phy_add, uint8_t reg_add);
void mx6q_mii_write (void *handle, uint8_t phy_add, uint8_t reg_add, uint16_t data);
void mx6_mii_sqi(mx6q_dev_t *mx6q);
void mx6q_BRCM_SQI_Monitor(void *);
void mx6q_ksz8081_phy_init(mx6q_dev_t *mx6q);

// sqi.c
void mx6_clear_sample();
int mx6_calculate_sqi(unsigned val);


// stats.c
void mx6q_update_stats(mx6q_dev_t *);
void mx6q_clear_stats(mx6q_dev_t *);

// bsd_media.c
void bsd_mii_initmedia(mx6q_dev_t *);
int bsd_mii_mediachange(struct ifnet *ifp);
void bsd_mii_finimedia(mx6q_dev_t *);

// ptp.c
void mx6q_ptp_start(mx6q_dev_t *);
int mx6q_ptp_is_eventmsg(struct mbuf *, ptpv2hdr_t **);
void mx6q_ptp_add_rx_timestamp(mx6q_dev_t *, ptpv2hdr_t *, mpc_bd_t *);
void mx6q_ptp_add_tx_timestamp(mx6q_dev_t *, ptpv2hdr_t *, mpc_bd_t *);
int mx6q_ptp_get_rx_timestamp(mx6q_dev_t *, ptp_extts_t *);
int mx6q_ptp_get_tx_timestamp(mx6q_dev_t *, ptp_extts_t *);
int mx6q_ptp_ioctl(mx6q_dev_t *, struct ifdrv *);
void mx6q_ptp_get_cnt(mx6q_dev_t *, ptp_time_t *);
void mx6q_ptp_set_cnt(mx6q_dev_t *, ptp_time_t);
void mx6q_ptp_set_compensation(mx6q_dev_t *, ptp_comp_t);

// timer.c
int mx6q_timer_delay (mx6q_dev_t *, struct ifdrv *);

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devnp/mx6x/mx6q.h $ $Rev: 910691 $")
#endif
