/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
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


#ifdef DEFN
    #define EXT
    #define INIT1(a)                = { a }
#else
    #define EXT extern
    #define INIT1(a)
#endif

#ifndef TRUE
    #define TRUE 1
#endif
#ifndef FALSE
    #define FALSE 0
#endif

#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <malloc.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/neutrino.h>
#include <termios.h>
#include <devctl.h>
#include <sys/dcmd_chr.h>
#include <sys/iomsg.h>
#include <atomic.h>
#include <hw/inout.h>
#include <aarch64/mx8x.h>
#include <aarch64/imx8_common/imx_lpuart.h>
#include <sys/io-char.h>
#include <sys/hwinfo.h>
#include <drvr/hwinfo.h>
#include <pthread.h>
#include <sys/rsrcdbmgr.h>
#include <sys/dispatch.h>
#include <sys/slog.h>
#include <sys/slogcodes.h>

#if defined(USE_DMA)
    #include <hw/dma.h>
#endif
#include <smmu.h>

/**
 * @file       src/hardware/devc/sermx8/externs.h
 * @addtogroup devc
 * @{
 */

#define FIFO_SIZE       64
#define MAX_FIFO_SIZE   256
#define RX_FIFO_MASK    0x000001FF
#define RX_FIFO_SHIFT   0
#define TX_FIFO_MASK    0x01FF0000
#define TX_FIFO_SHIFT   16
#define RX_FIFO_WM_DEF  1
#define TX_FIFO_WM_DEF  32

/* LPUART error mask */
#define DEVC_LPUART_ERROR_MASK (IMX_LPUART_STAT_OR_MASK | IMX_LPUART_STAT_NF_MASK | IMX_LPUART_STAT_FE_MASK | IMX_LPUART_STAT_PF_MASK)

typedef struct mx53_dma {
    char            *buf;
    off64_t         phys_addr;
    uint32_t        xfer_size;
    struct sigevent sdma_event;
    void            *pulse;
    void            *dma_chn;
    int             bytes_read;
    int             buffer0;
    int             status;
    unsigned        key;
} mx8_dma_t;

typedef struct dev_uart {
    TTYDEV          tty;
    int             intr[2];
    int             iid[2];
    unsigned        clk;
    unsigned        div;
    unsigned        fifo;
    uintptr_t       base;            /**< Virtual base address of LPUART registers */
    uintptr_t       phys_base;       /**< Physical base address of LPUART registers */
    unsigned        fcr;
    unsigned        ctrl;
    unsigned        global;
    unsigned        baud;
    unsigned        modir;
    unsigned        bir;
    /* EDMA */
    int             fd;              /**< File descriptor used for internal buffer allocation for systems with more than 4GB RAM */
    mx8_dma_t       rx_dma;
    mx8_dma_t       tx_dma;
    unsigned        usedma;
    pthread_mutex_t mutex;            /**< Tx DMA mutex */
    int             rx_dma_evt;
    int             tx_dma_evt;
    int             isr;
    int             rx_idle_cnt;     /**< Number of IDLE character - EOP detection when DMA is used */
    unsigned        usesmmu;
#if defined(USE_DMA)
    struct          smmu_object *smmu_obj;
    dma_functions_t     sdmafuncs;
    volatile unsigned   tx_xfer_active;
#endif
} DEV_UART;

typedef struct ttyinit_uart {
    TTYINIT     tty;
    int         intr[2];   /* Interrupts */
    unsigned    usedma;
    int         rx_dma_evt;
    int         tx_dma_evt;
    int         isr;
    int         rx_idle_cnt;
    unsigned    usesmmu;
} TTYINIT_UART;

EXT TTYCTRL        ttyctrl;

#define DMA_XFER_SIZE       512

extern _Paddrt mphys(void *);
void dinit(void);

char *UserParm;

#include "proto.h"


/** @} */ /* end of devc */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/devc/sermx8/externs.h $ $Rev: 893169 $")
#endif
