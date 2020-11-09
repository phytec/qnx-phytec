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


#include "externs.h"

/**
 * @file       src/hardware/devc/sermx8/intr.c
 * @addtogroup devc
 * @{
 */

#define IMX_RXERR (IMX_UART_URXD_ERR | IMX_UART_URXD_OVERRUN | IMX_UART_URXD_FRMERR | IMX_UART_URXD_BRK | IMX_UART_URXD_PRERR)

#if defined(USE_DMA)
/*
 * TX Pulse Handler - Gets notified once TX DMA is done
 */
void lpuart_tx_pulse_hdlr(DEV_UART *dev, struct sigevent *event)
{
    int status = 0;

    dev->sdmafuncs.xfer_complete(dev->tx_dma.dma_chn);
    pthread_mutex_lock(&dev->mutex);
    dev->tx_xfer_active = FALSE;
    dev->tty.un.s.tx_tmr = 0;
    pthread_mutex_unlock(&dev->mutex);

    /* Send event to io-char, tto() will be processed at thread time */
    atomic_set(&dev->tty.flags, EVENT_TTO);
    status |= 1;

    if (status) {
        iochar_send_event (&dev->tty);
    }
}

/*
 * RX Pulse Handler - Get notified once RX DMA is complete
 */
void lpuart_rx_pulse_hdlr(DEV_UART *dev, struct sigevent *event)
{
    uintptr_t    base = dev->base;
    int error = 0;
    uint32_t stat_reg;
    dma_transfer_t tinfo;
    dma_addr_t src_dma_addr, dst_dma_addr;

    dev->rx_dma.bytes_read = dev->sdmafuncs.bytes_left(dev->rx_dma.dma_chn);
    error = dev->sdmafuncs.xfer_complete(dev->rx_dma.dma_chn);
    dev->rx_dma.key = 0;
    dev->rx_dma.status = 0;

    /* Read status register */
    stat_reg = in32(base + IMX_LPUART_STAT);

    if (error || ((stat_reg & DEVC_LPUART_ERROR_MASK) != 0 )) {
        /* DMA error or LPUART error detected */
        atomic_set(&dev->tty.flags, OBAND_DATA);
        /* Check LPUART errors */
        if ((stat_reg & IMX_LPUART_STAT_FE_MASK) && (stat_reg & IMX_LPUART_STAT_RDRF_MASK)) {
            /* Break detected */
            dev->rx_dma.key |= TTI_BREAK;
            slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Break detected", __FUNCTION__);
        }
        if (stat_reg & IMX_LPUART_STAT_FE_MASK) {
            /* Framing error detected */
            dev->rx_dma.key |= TTI_FRAME;
            slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Framing error detected", __FUNCTION__);
        }
        if (stat_reg & IMX_LPUART_STAT_PF_MASK) {
            /* Parity error detected */
            dev->rx_dma.key |= TTI_PARITY;
            slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Parity error detected", __FUNCTION__);
        }
        if (stat_reg & IMX_LPUART_STAT_OR_MASK) {
            /* Overrun error detected */
            dev->rx_dma.key |= TTI_OVERRUN;
            slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Overrun error detected", __FUNCTION__);
        }
        /* Clear error status masks in the status register */
        out32(base + IMX_LPUART_STAT, (stat_reg | DEVC_LPUART_ERROR_MASK));
    }

    if ((dev->tty.c_cflag & OHFLOW) && ((dev->tty.ibuf.size - dev->tty.ibuf.cnt) < dev->rx_dma.bytes_read)) {
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "rx no buf\n");
        /* RTS flow control enabled and there is not enough space in io-char buffer */
        /* No enough space in ibuf. Return!
         * This is okay because RTS/CTS is enabled and another transfer will
         * get scheduled once we receive the signal in tto.c
         */

        /* Set IHW_PAGED otherwise io-char flow control code will not call tto */
        atomic_set (&dev->tty.flags, IHW_PAGED);

        /* Force READ to make more room in io-char buffer */
        atomic_set (&dev->tty.flags, EVENT_READ);
        iochar_send_event (&dev->tty);

        return;
    }

    if (dev->rx_dma.buffer0) {
        /* Prepare RX DMA info structure using RX DMA buffer 1 */
        lpuart_prep_rx_dma_transfer(dev, tinfo, src_dma_addr, dst_dma_addr, dev->rx_dma.phys_addr + dev->rx_dma.xfer_size + 1);
        /* Set up DMA RX transfer */
        dev->sdmafuncs.setup_xfer(dev->rx_dma.dma_chn, &tinfo);
        /* Start DMA RX transfer using RX DMA buffer 1 */
        dev->sdmafuncs.xfer_start(dev->rx_dma.dma_chn);

        if ((dev->tty.ibuf.size - dev->tty.ibuf.cnt) >= dev->rx_dma.bytes_read) {    /* Enough space in io-char buffer? */
            /* Transfer data from RX DMA buffer 0 to RX software fifo in case there is enough space for data */
            dev->rx_dma.status |= tti2(&dev->tty, (unsigned char *)dev->rx_dma.buf, dev->rx_dma.bytes_read, dev->rx_dma.key);
        } else {
            /* When there is not enough space in the io-char buffer, and RTS flow control is disabled, received data is lost */
        }
    } else {
        /* Prepare RX DMA info structure using RX DMA buffer 0 */
        lpuart_prep_rx_dma_transfer(dev, tinfo, src_dma_addr, dst_dma_addr, dev->rx_dma.phys_addr);
        /* Set up DMA RX transfer */
        dev->sdmafuncs.setup_xfer(dev->rx_dma.dma_chn, &tinfo);
        /* Start DMA RX transfer using RX DMA buffer 0 */
        dev->sdmafuncs.xfer_start(dev->rx_dma.dma_chn);

        if ((dev->tty.ibuf.size - dev->tty.ibuf.cnt) >= dev->rx_dma.bytes_read) {    /* Enough space in io-char buffer? */
            /* Transfer data from RX DMA buffer 1 to RX software FIFO in case there is enough space data */
            dev->rx_dma.status |= tti2(&dev->tty, (unsigned char *)(dev->rx_dma.buf + dev->rx_dma.xfer_size + 1), dev->rx_dma.bytes_read, dev->rx_dma.key);
        } else {
            /* When there is not enough space in the io-char buffer, and RTS flow control is disabled, received data is lost and new DMA transfer is initiated */
        }
    }
    dev->rx_dma.buffer0 ^= 1;

    if (dev->rx_dma.status) {
        iochar_send_event (&dev->tty);
    }
}
#endif
static inline int ms_interrupt(DEV_UART *dev)
{
    int status = 0;
#if 0
    uintptr_t       base = dev->base;
    int stat_reg;

    stat_reg = in32(base + IMX_UART_USR1);
    out32(base + IMX_UART_USR1, IMX_UART_USR1_RTSD);
    if (dev->tty.c_cflag & OHFLOW) {
        status |= tti(&dev->tty,
                      (stat_reg & IMX_UART_USR1_RTSS) ? TTI_OHW_CONT : TTI_OHW_STOP);
    }
#endif

    return (status);
}

static inline int tx_interrupt(DEV_UART *dev)
{
    int        status = 0;
    uintptr_t  base = dev->base;
    int        ctrl_reg;

    /* Disable interrupt */
    ctrl_reg = in32(base + IMX_LPUART_CTRL);
    out32(base + IMX_LPUART_CTRL, (ctrl_reg & ~(IMX_LPUART_CTRL_TIE_MASK)));
    dev->tty.un.s.tx_tmr = 0;
    /* Send event to io-char, tto() will be processed at thread time */
    atomic_set(&dev->tty.flags, EVENT_TTO);
    status |= 1;

    return (status);
}

static inline int rx_interrupt(DEV_UART *dev)
{
    int       status = 0;
    int       byte_count = 0;
    unsigned  key, rxdata;
    uintptr_t base = dev->base;
    uint32_t  stat_reg;

    /* Limit loop iterations by FIFO size to prevent ISR from running too long */
    while (!(in32(base + IMX_LPUART_FIFO) & IMX_LPUART_FIFO_RXEMPT_MASK) && (byte_count < FIFO_SIZE)) {
        /* Read next character from FIFO */
        rxdata = in32(base + IMX_LPUART_DATA) & IMX_LPUART_DATA_RT_MASK;
        switch(dev->tty.c_cflag & CSIZE) {
            case CS7:
                key = rxdata & 0x7F;
                break;
            case CS8:
            default:
                key = rxdata & 0xFF;
                break;
        }
        /* Read status register and check LPUART errors */
        stat_reg = in32(base + IMX_LPUART_STAT);
        if ((stat_reg & DEVC_LPUART_ERROR_MASK) !=0 ) {
            /*
             * Save error as out-of-band data which can be read via devctl()
             */
            dev->tty.oband_data |= key;
            atomic_set(&dev->tty.flags, OBAND_DATA);
            /* Check LPUART errors */
            if ((stat_reg & IMX_LPUART_STAT_FE_MASK) && (stat_reg & IMX_LPUART_STAT_RDRF_MASK)) {
                /* Break detected */
                key |= TTI_BREAK;
            }
            if (stat_reg & IMX_LPUART_STAT_FE_MASK) {
                /* Framing error detected */
                key |= TTI_FRAME;
            }
            if (stat_reg & IMX_LPUART_STAT_PF_MASK) {
                /* Parity error detected */
                key |= TTI_PARITY;
            }
            if (stat_reg & IMX_LPUART_STAT_OR_MASK) {
                /* Overrun error detected */
                key |= TTI_OVERRUN;
            }
            /* Clear error status masks in the status register */
            out32(base + IMX_LPUART_STAT, (stat_reg | DEVC_LPUART_ERROR_MASK));
        }

        status |= tti(&dev->tty, key);
        byte_count++;
    }

    return status;
}

static inline int do_interrupt(DEV_UART *dev, int id)
{
    int      sts=0;
    uint32_t stat_reg, ctrl_reg;

    if (!dev->usedma) {  /* Do not need to process tx and rx_interrupt in DMA mode */
        /*
         * If the interrupt was caused by the RX FIFO fill level
         * being reached or the aging timer interrupt fired process the RX interrupt
         */
        stat_reg = in32(dev->base + IMX_LPUART_STAT);    /* Read status register */
        ctrl_reg = in32(dev->base + IMX_LPUART_CTRL);    /* Read control register */
        if ((stat_reg & IMX_LPUART_STAT_RDRF_MASK) && (ctrl_reg & IMX_LPUART_CTRL_RIE_MASK)) {
            sts = rx_interrupt(dev);
        }

        if ((stat_reg & IMX_LPUART_STAT_TDRE_MASK) && (ctrl_reg & IMX_LPUART_CTRL_TIE_MASK)) {
            sts |= tx_interrupt(dev);
        }
    }

    if (in32(dev->base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TDRE_MASK) {
        sts |= ms_interrupt(dev);
    }

    return sts;
}

/*
 * Serial interrupt handler
 */
static const struct sigevent * ser_intr(void *area, int id)
{
    DEV_UART    *dev = area;

    if (do_interrupt(dev,id) && (dev->tty.flags & EVENT_QUEUED) == 0) {
        dev_lock(&ttyctrl);
        ttyctrl.event_queue[ttyctrl.num_events++] = &dev->tty;
        atomic_set(&dev->tty.flags, EVENT_QUEUED);
        dev_unlock(&ttyctrl);
        return &ttyctrl.event;
    }

    return 0;
}

int interrupt_event_handler (message_context_t * msgctp, int code, unsigned flags, void *handle)
{
    uint32_t status;
    DEV_UART *dev = (DEV_UART *) handle;

    status = do_interrupt (dev, dev->iid[0]);

    if (status) {
        iochar_send_event (&dev->tty);
    }

    InterruptUnmask (dev->intr[0], dev->iid[0]);
    return (EOK);
}

void ser_attach_intr(DEV_UART *dev)
{
    struct sigevent event;
    if (dev->isr) {
        dev->iid[0] = InterruptAttach(dev->intr[0], ser_intr, dev, 0, _NTO_INTR_FLAGS_TRK_MSK);
        if (dev->intr[1] != -1) {
            dev->iid[1] = InterruptAttach(dev->intr[1], ser_intr, dev, 0, _NTO_INTR_FLAGS_TRK_MSK);
        }
    } else {
        /* Associate a pulse which will call the event handler. */
        if ((event.sigev_code =
                pulse_attach (ttyctrl.dpp, MSG_FLAG_ALLOC_PULSE, 0, &interrupt_event_handler,
                    dev)) == -1) {
            fprintf (stderr, "Unable to attach event pulse.%s\n", strerror(errno));
            return;
        }

        /* Init the pulse for interrupt event */
        event.sigev_notify = SIGEV_PULSE;
        event.sigev_coid = ttyctrl.coid;

        /*
         * If the interrupt is being handled by an event, then set the event priority
         * to the io-char event priority+1. The io-char event priority can be configured
         * by the "-o priority=X" parameter.
         */
        event.sigev_priority = ttyctrl.event.sigev_priority + 1;
        event.sigev_value.sival_int = 0;

        dev->iid[0] = InterruptAttachEvent (dev->intr[0], &event, _NTO_INTR_FLAGS_TRK_MSK);
        if (dev->iid[0] == -1) {
            fprintf (stderr, "Unable to attach InterruptEvent. %s\n", strerror(errno));
        }
    }

    /* Enable Receive Ready DMA Interrupt Enable, Transmitter Ready DMA Enable */
    if (dev->usedma) {
        out32(dev->base + IMX_LPUART_BAUD, in32(dev->base + IMX_LPUART_BAUD) | IMX_LPUART_BAUD_RDMAE_MASK | IMX_LPUART_BAUD_TDMAE_MASK);
    } else {
        /* Ensure that DMA interrupts are all disabled */
        out32(dev->base + IMX_LPUART_BAUD, in32(dev->base + IMX_LPUART_BAUD) & ~(IMX_LPUART_BAUD_RDMAE_MASK | IMX_LPUART_BAUD_TDMAE_MASK));
    }
}


void ser_detach_intr(DEV_UART *dev)
{
    /* Set a device to a default POR state */
    out32(dev->base + IMX_LPUART_GLOBAL, IMX_LPUART_GLOBAL_RST_MASK);
    while ((in32(dev->base + IMX_LPUART_GLOBAL) & IMX_LPUART_GLOBAL_RST_MASK) == 0) {}
    out32(dev->base + IMX_LPUART_GLOBAL, 0x00);
    while ((in32(dev->base + IMX_LPUART_GLOBAL) & IMX_LPUART_GLOBAL_RST_MASK) != 0) {}

    InterruptDetach(dev->iid[0]);
    dev->intr[0] = -1;
    if (dev->intr[1] != -1) {
        InterruptDetach(dev->iid[1]);
        dev->intr[1] = -1;
    }
}


/** @} */ /* end of devc */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devc/sermx8/intr.c $ $Rev: 871081 $")
#endif
