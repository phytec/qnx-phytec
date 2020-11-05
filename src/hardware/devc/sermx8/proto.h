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

/**
 * @file       src/hardware/devc/sermx8/proto.h
 * @addtogroup devc
 * @{
 */


void        create_device(TTYINIT_UART *dip, unsigned unit, uint8_t *smmu_ports_created);
void        ser_stty(DEV_UART *dev);
void        ser_ctrl(DEV_UART *dev, unsigned flags);
void        ser_attach_intr(DEV_UART *dev);
void *      query_default_device(TTYINIT_UART *dip, void *link);
unsigned    options(int argc, char *argv[], uint8_t *smmu_ports_created);

/* pulse.c */
#if defined(USE_DMA)
int my_attach_pulse(void **x , struct sigevent *event , void (*handler)(DEV_UART *dev , struct sigevent *event),
                    DEV_UART *dev);
int my_detach_pulse(void **x);
void lpuart_rx_pulse_hdlr(DEV_UART *dev, struct sigevent *event);
void lpuart_tx_pulse_hdlr(DEV_UART *dev, struct sigevent *event);


/**
 * This function prepares information for RX DMA transfer.
 *
 * @param dev         Device info structure.
 * @src_dma_addr_info Structure containing a source address information.
 * @dst_dma_addr_info Structure containing a destination address information.
 * @dst_dma_addr      Destination address for RX DMA transfer.
 *
 */
#define lpuart_prep_rx_dma_transfer(dev, tinfo, src_dma_addr_info, dst_dma_addr_info, dst_dma_addr) ({\
        /* Schedule an RX DMA transfer (upto MAX DMA SIZE) */\
        memset(&tinfo, 0, sizeof(tinfo));\
        tinfo.xfer_bytes = ((dev->fifo & RX_FIFO_MASK) >> RX_FIFO_SHIFT) + 1;\
        tinfo.xfer_unit_size = 8;\
        /* DMA RX source address */\
        src_dma_addr_info.len = dev->rx_dma.xfer_size;\
        src_dma_addr_info.paddr = (uint32_t)dev->phys_base + IMX_LPUART_DATA;\
        tinfo.src_addrs= &src_dma_addr_info;\
        tinfo.src_fragments = 1;\
        tinfo.src_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;\
        /* DMA RX destination address */\
        dst_dma_addr_info.len = dev->rx_dma.xfer_size;\
        dst_dma_addr_info.paddr = dst_dma_addr;\
        tinfo.dst_addrs = &dst_dma_addr_info;\
        tinfo.dst_fragments = 1;\
    })

#endif


/**
 * This function prepares information for TX DMA transfer.
 *
 * @param dev         Device info structure.
 * @src_dma_addr_info Structure containing a source address information.
 * @dst_dma_addr_info Structure containing a destination address information.
 *
 */
#define lpuart_prep_tx_dma_transfer(dev, tinfo, src_dma_addr_info, dst_dma_addr_info, data_num) ({\
        /* Schedule a TX DMA transfer */\
        memset(&tinfo, 0, sizeof(tinfo));\
        tinfo.xfer_bytes = 1;\
        tinfo.xfer_unit_size = 8;\
        /* DMA TX source address */\
        src_dma_addr.len = data_num;\
        src_dma_addr.paddr = dev->tx_dma.phys_addr;\
        tinfo.src_addrs = &src_dma_addr;\
        tinfo.src_fragments = 1;\
        /* DMA TX destination address */\
        dst_dma_addr.len = data_num;\
        dst_dma_addr.paddr = (uint32_t)dev->phys_base + IMX_LPUART_DATA;\
        tinfo.dst_addrs = &dst_dma_addr;\
        tinfo.dst_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;\
        tinfo.dst_fragments = 1;\
    })

/** @} */ /* end of devc */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/devc/sermx8/proto.h $ $Rev: 883564 $")
#endif
