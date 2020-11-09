
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


#include "externs.h"
#include <sys/mman.h>
#include <string.h>
#if defined(USE_DMA)
    #include <hw/edma.h>
    #include <sys/siginfo.h>
#endif
#include <smmu.h>

/**
 * @file       src/hardware/devc/sermx8/init.c
 * @addtogroup devc
 * @{
 */

/*
 * Specify parameters for default devices.
 */
void * query_default_device(TTYINIT_UART *dip, void *link)
{
    /*
     * No default device, the base address and irq have be be specified
     */
    return NULL;
}

void create_device(TTYINIT_UART *dip, unsigned unit, uint8_t *smmu_ports_created)
{
    DEV_UART *dev;          /* Device info structure */
    uint32_t reg32;
    uint32_t rx_fifo_wm, tx_fifo_wm;
    uint32_t rxFifoSize, txFifoSize;

#if defined(USE_DMA)
    dma_transfer_t tinfo;
    dma_addr_t src_dma_addr, dst_dma_addr;
    struct smmu_map_entry smmu_buffer_entries[2];
#endif
    /*
     * Get a device entry and the input/output buffers for it.
     */
    dev = calloc(1, sizeof(*dev));

    if (dev == NULL) {
        perror("UART: Unable to allocate device entry\n");
        exit(1);
    }

    if (dev->usedma && dip->tty.isize < (DMA_XFER_SIZE * 2)) {
        perror("UART: Invalid input buffer size\n");
        dip->tty.isize = DMA_XFER_SIZE * 2;
    }

    /*
     * Get buffers.
     */
    dev->tty.ibuf.head = dev->tty.ibuf.tail = dev->tty.ibuf.buff = malloc(dev->tty.ibuf.size = dip->tty.isize);
    dev->tty.obuf.head = dev->tty.obuf.tail = dev->tty.obuf.buff = malloc(dev->tty.obuf.size = dip->tty.osize);
    dev->tty.cbuf.head = dev->tty.cbuf.tail = dev->tty.cbuf.buff = malloc(dev->tty.cbuf.size = dip->tty.csize);
    if (dip->usedma) {
        dev->tty.highwater = dev->tty.ibuf.size + 1;    // when DMA is enabled never reach the RX FIFO highwater mark.
    } else {
        dev->tty.highwater = dev->tty.ibuf.size - ((dev->tty.ibuf.size < 128) ? (dev->tty.ibuf.size / 4) : 100);
    }

    strlcpy(dev->tty.name, dip->tty.name, sizeof(dev->tty.name));

    dev->tty.baud    = dip->tty.baud;

    /*
     * The i.MX SOCs don't technically require the LOSES_TX_INTR flag,
     * but the timer mechanism acts as a fail safe in case we ever miss a TX interrupt.
     */
    dev->tty.flags   = EDIT_INSERT | LOSES_TX_INTR;
    dev->tty.c_cflag = dip->tty.c_cflag;
    dev->tty.c_iflag = dip->tty.c_iflag;
    dev->tty.c_lflag = dip->tty.c_lflag;
    dev->tty.c_oflag = dip->tty.c_oflag;
    dev->tty.verbose = dip->tty.verbose;
    dev->tty.fifo    = dip->tty.fifo;

    dev->fifo        = dip->tty.fifo;
    dev->intr[0]     = dip->intr[0];
    dev->intr[1]     = dip->intr[1];
    dev->clk         = dip->tty.clk;
    dev->div         = dip->tty.div;
    dev->phys_base   = dip->tty.port;

    dev->usedma      = dip->usedma;
    dev->rx_dma_evt  = dip->rx_dma_evt;
    dev->tx_dma_evt  = dip->tx_dma_evt;
    dev->isr         = dip->isr;
    dev->rx_idle_cnt = dip->rx_idle_cnt;
    dev->rx_dma.buffer0 = 1;
    dev->rx_dma.status = 0;
    dev->rx_dma.bytes_read = 0;
    dev->rx_dma.key = 0;
    dev->usesmmu      = dip->usesmmu;

    /*
     * Currently io-char has a limitation that the TX timeout is hard coded to 150ms.
     * At low baud rates the timer could potentially expire before the DMA transfer
     * naturally completes. So when DMA is enabled we disable the LOSES_TX_INTR flag
     * and let the driver specify the timeout value in tto().
     */
#if defined(USE_DMA)
    if (dev->usedma) {
        dev->tty.flags &= ~LOSES_TX_INTR;
    }
#endif

    /* Map device registers */
    dev->base = mmap_device_io(IMX_LPUART_SIZE, dip->tty.port);
    if (dev->base == (uintptr_t)MAP_FAILED) {
        perror("UART: MAP_FAILED\n");
        goto fail1;
    }
    pthread_mutex_init(&dev->mutex, NULL);
#if defined(USE_DMA)
    if (dev->usedma) {
        dev->fd = posix_typed_mem_open("/memory/below4G", O_RDWR, POSIX_TYPED_MEM_ALLOCATE_CONTIG);
        /* Allocation of TX DMA buffer */
        dev->tx_dma.xfer_size = DMA_XFER_SIZE;
        if (dev->fd == -1) {
            /* If no fd exists try to allocate buffer
             * using mmap (system with less than 4GB RAM)*/
            dev->tx_dma.buf = mmap(NULL, dev->tx_dma.xfer_size,
                                   PROT_READ | PROT_WRITE | PROT_NOCACHE,
                                   MAP_ANON | MAP_PHYS | MAP_PRIVATE, NOFD, 0);
        } else {
            /* Allocate memory under 4GB RAM on systems with
             * more than 4GB. DMA supports 32-bit address range.*/
            dev->tx_dma.buf = mmap(NULL, dev->tx_dma.xfer_size,
                                   PROT_READ | PROT_WRITE | PROT_NOCACHE,
                                   MAP_SHARED, dev->fd, 0);
        }
        if (dev->tx_dma.buf == MAP_FAILED) {
            perror("Unable to allocate DMA memory\n");
            goto fail2;
        }

        if (mem_offset64(dev->tx_dma.buf, NOFD, 1, &dev->tx_dma.phys_addr, 0) == -1) {
            perror("LPUART: Error in address conversion of tx buffer\n");
            goto fail3;
        }
        msync(dev->tx_dma.buf, dev->tx_dma.xfer_size, MS_INVALIDATE);


        /* Allocation of RX DMA buffer */
        /* Allocate 2x the transfer size for ping-pong buffer. Additional two bytes are required by
         * the DMA engine. DMA engine stores 0xFF byte after each transfer when the end of packet detection is used.
         */
        dev->rx_dma.xfer_size = DMA_XFER_SIZE;
        if (dev->fd == -1) {
            /* If no fd exists try to allocate buffer
             * using mmap (system with less than 4GB RAM)*/
            dev->rx_dma.buf = mmap(NULL, ((dev->rx_dma.xfer_size + 1) * 2),
                                   PROT_READ | PROT_WRITE | PROT_NOCACHE,
                                   MAP_ANON | MAP_PHYS | MAP_PRIVATE, NOFD, 0);
        } else {
            /* Allocate memory under 4GB RAM on systems with
             * more than 4GB. DMA supports 32-bit address range.*/
            dev->rx_dma.buf = mmap(NULL, ((dev->rx_dma.xfer_size + 1) * 2),
                                   PROT_READ | PROT_WRITE | PROT_NOCACHE,
                                   MAP_SHARED, dev->fd, 0);
        }
        if (dev->rx_dma.buf == MAP_FAILED) {
            perror("Unable to allocate DMA memory\n");
            goto fail3;
        }
        if (mem_offset64(dev->rx_dma.buf, NOFD, 1, &dev->rx_dma.phys_addr, 0) == -1) {
            perror("LPUART: Error in address conversion of rx buffer\n");
            goto fail4;
        }
        msync(dev->rx_dma.buf, (dev->rx_dma.xfer_size + 1) * 2, MS_INVALIDATE);

        my_attach_pulse(&dev->rx_dma.pulse, &dev->rx_dma.sdma_event, lpuart_rx_pulse_hdlr, dev);
        my_attach_pulse(&dev->tx_dma.pulse, &dev->tx_dma.sdma_event, lpuart_tx_pulse_hdlr, dev);

        if (get_dmafuncs(&dev->sdmafuncs, sizeof(dma_functions_t)) == -1) {
            perror("UART: Failed to get DMA lib function\n");
            goto fail4;
        }

        if (dev->sdmafuncs.init(NULL) == -1) {
            perror("UART: DMA init failed\n");
            goto fail4;
        }

	    /* SMMU Mappings Enabled */
	    if(dip->usesmmu) {
		    slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "UART SMMU Mappings Enabled for port %p", (void*)dip->tty.port);

		    /* Track how many ports are being opened in this process */
		    (*smmu_ports_created)++;

		    dev->smmu_obj = smmu_obj_create(SOCF_NONE);
		    if (dev->smmu_obj == NULL) {
		        perror("UART: Failed to create SMMU object\n");
		        goto fail6;
		    }

	        /* Attach RX DMA channel w/ SMMU */
	        if ((dev->rx_dma.dma_chn = dev->sdmafuncs.channel_attach_smmu("eeop=1", &dev->rx_dma.sdma_event, (unsigned *) &dev->rx_dma_evt,
	            DMA_ATTACH_PRIORITY_HIGHEST, DMA_ATTACH_EVENT_ON_COMPLETE, dev->smmu_obj)) == NULL) {
	            perror("UART: Unable to create rx dma channel\n");
	            goto fail7;
	        }

	        /* Attach RX DMA channel w/ SMMU */
	        if ((dev->tx_dma.dma_chn = dev->sdmafuncs.channel_attach_smmu(NULL, &dev->tx_dma.sdma_event, (unsigned *) &dev->tx_dma_evt,
	            DMA_ATTACH_PRIORITY_HIGHEST, DMA_ATTACH_EVENT_ON_COMPLETE, dev->smmu_obj)) == NULL) {
	            perror("UART: Unable to create tx dma channel\n");
	            goto fail8;
	        }

            smmu_buffer_entries[0].phys = (uintptr64_t)dev->rx_dma.phys_addr;
            smmu_buffer_entries[0].len = (dev->rx_dma.xfer_size + 1) * 2;
            smmu_buffer_entries[1].phys = (uintptr64_t)dev->tx_dma.phys_addr;
            smmu_buffer_entries[1].len = dev->tx_dma.xfer_size;

            if ( smmu_mapping_add(dev->smmu_obj, SMF_READ|SMF_WRITE, 0, 2, smmu_buffer_entries, 0) == -1 ) {
                perror("UART: Failed to add buffer mappings\n");
                goto fail9;
            }

        } else {

	        /* Attach RX DMA channel */
	        if ((dev->rx_dma.dma_chn = dev->sdmafuncs.channel_attach("eeop=1", &dev->rx_dma.sdma_event, (unsigned *) &dev->rx_dma_evt,
	            DMA_ATTACH_PRIORITY_HIGHEST, DMA_ATTACH_EVENT_ON_COMPLETE)) == NULL) {
	            perror("UART: Unable to create rx dma channel\n");
	            goto fail5;
	        }

	        /* Attach TX DMA channel */
	        if ((dev->tx_dma.dma_chn = dev->sdmafuncs.channel_attach(NULL, &dev->tx_dma.sdma_event, (unsigned *) &dev->tx_dma_evt,
	            DMA_ATTACH_PRIORITY_HIGHEST, DMA_ATTACH_EVENT_ON_COMPLETE)) == NULL) {
	            perror("UART: Unable to create tx dma channel\n");
	            goto fail8;
	        }
	    }

        dev->tx_xfer_active = FALSE;
    }
#endif
    /*
    * Initialize termios cc codes to an ANSI terminal.
    */
    ttc(TTC_INIT_CC, &dev->tty, 0);

    /*
    * Initialize the device's name.
    * Assume that the basename is set in device name.  This will attach
    * to the path assigned by the unit number/minor number combination
    */
    unit = SET_NAME_NUMBER(unit) | NUMBER_DEV_FROM_USER;
    ttc(TTC_INIT_TTYNAME, &dev->tty, unit);

    /* Wait for UART to finish transmitting */
    while (!(in32(dev->base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TDRE_MASK)) {}

    /* Disable LPUART receiver & transmitter */
    reg32 = in32(dev->base + IMX_LPUART_CTRL) & (~(IMX_LPUART_CTRL_RE_MASK | IMX_LPUART_CTRL_TE_MASK));
    out32(dev->base + IMX_LPUART_CTRL, reg32);

    /* Set a device to a default POR state */
    out32(dev->base + IMX_LPUART_GLOBAL, IMX_LPUART_GLOBAL_RST_MASK);
    while ((in32(dev->base + IMX_LPUART_GLOBAL) & IMX_LPUART_GLOBAL_RST_MASK) == 0) {}
    out32(dev->base + IMX_LPUART_GLOBAL, 0x00);
    while ((in32(dev->base + IMX_LPUART_GLOBAL) & IMX_LPUART_GLOBAL_RST_MASK) != 0) {}

   /* Clear status flags */
    reg32 = in32(dev->base + IMX_LPUART_STAT) |
            (IMX_LPUART_STAT_LBKDIF_MASK |
             IMX_LPUART_STAT_RXEDGIF_MASK |
             IMX_LPUART_STAT_IDLE_MASK |
             IMX_LPUART_STAT_OR_MASK |
             IMX_LPUART_STAT_NF_MASK |
             IMX_LPUART_STAT_FE_MASK |
             IMX_LPUART_STAT_PF_MASK |
             IMX_LPUART_STAT_MA1F_MASK |
             IMX_LPUART_STAT_MA2F_MASK);
   out32(dev->base + IMX_LPUART_STAT, reg32);

    /*
    * Only setup IRQ handler for non-pcmcia devices.
    * Pcmcia devices will have this done later when card is inserted.
    */
    if ((dip->tty.port != 0) && (dev->intr[0] != -1)) {
        ser_stty(dev);
        if (!dev->usedma) {
            ser_attach_intr(dev);
        }
    }

#if defined(USE_DMA)
    if (dev->usedma) {
        /* FIFO configuration */
        /* Disable Receiver Idle Empty bit - similar functionality is enabled by setting RIDMAE, enable FIFO */
        reg32 = in32(dev->base + IMX_LPUART_FIFO);
        reg32 &= ~IMX_LPUART_FIFO_RXIDEN_MASK;
        reg32 |= IMX_LPUART_FIFO_RXFE_MASK |
                       IMX_LPUART_FIFO_TXFE_MASK;
    } else {
#endif
    /* Set the "timeout" to 2 characters, enable FIFO */
    reg32 = in32(dev->base + IMX_LPUART_FIFO);
    reg32 &= ~IMX_LPUART_FIFO_RXIDEN_MASK;
    reg32 |= (2 << IMX_LPUART_FIFO_RXIDEN_SHIFT) |
                   IMX_LPUART_FIFO_RXFE_MASK |
                   IMX_LPUART_FIFO_TXFE_MASK;
#if defined(USE_DMA)
    }
#endif
    out32(dev->base + IMX_LPUART_FIFO, reg32);
    /* RX FIFO water mark */
    rxFifoSize = 2 << ((reg32 & IMX_LPUART_FIFO_RXFIFOSIZE_MASK) >> IMX_LPUART_FIFO_RXFIFOSIZE_SHIFT);
    txFifoSize = 2 << ((reg32 & IMX_LPUART_FIFO_TXFIFOSIZE_MASK) >> IMX_LPUART_FIFO_TXFIFOSIZE_SHIFT);
    rx_fifo_wm = (dev->fifo & RX_FIFO_MASK) >> RX_FIFO_SHIFT;
    tx_fifo_wm = (dev->fifo & TX_FIFO_MASK) >> TX_FIFO_SHIFT;
    if (rx_fifo_wm >= rxFifoSize) {
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "serial: Rx FIFO water mark must less than Rx FIFO size %d.\n", rxFifoSize);
        rx_fifo_wm = rxFifoSize >> 1;
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "serial: Set Rx FIFO water mark to half of Rx FIFO size (%d).\n", rx_fifo_wm);
    }
    if (tx_fifo_wm >= txFifoSize) {
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "serial: Tx FIFO water mark must less than Tx FIFO size %d.\n", txFifoSize);
        tx_fifo_wm = txFifoSize >> 1;
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "serial: Set Tx FIFO water mark to half of Tx FIFO size (%d).\n", tx_fifo_wm);
    }
    out32(dev->base + IMX_LPUART_WATER, ((rx_fifo_wm << IMX_LPUART_WATER_RXWATER_SHIFT) | (tx_fifo_wm << IMX_LPUART_WATER_TXWATER_SHIFT)));
    slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "Rx water %d, Tx water %d", rx_fifo_wm, tx_fifo_wm);

    /* Set RTS water mark to a half of the RX FIFO size */
    reg32 = in32(dev->base + IMX_LPUART_MODIR) & ~IMX_LPUART_MODIR_RTSWATER_MASK;
    reg32 |= (rxFifoSize >> 1) << IMX_LPUART_MODIR_RTSWATER_SHIFT;
    out32(dev->base + IMX_LPUART_MODIR, reg32);

#if defined(USE_DMA)
    if (dev->usedma) {
        /* Enable RX DMA, TX DMA and RX IDLE DMA */
        reg32 = in32(dev->base + IMX_LPUART_BAUD);
        reg32 |= (IMX_LPUART_BAUD_RDMAE_MASK | IMX_LPUART_BAUD_TDMAE_MASK | IMX_LPUART_BAUD_RIDMAE_MASK);
        out32(dev->base + IMX_LPUART_BAUD, reg32);
        /* Set number of IDLE characters */
        reg32 = (in32(dev->base + IMX_LPUART_CTRL) & ~IMX_LPUART_CTRL_IDLECFG_MASK) | (dev->rx_idle_cnt << IMX_LPUART_CTRL_IDLECFG_SHIFT);

        out32(dev->base + IMX_LPUART_CTRL, reg32);
    }
#endif

    /* Enable Transmitter and Receiver */
    out32(dev->base + IMX_LPUART_CTRL,
          in32(dev->base + IMX_LPUART_CTRL) | (IMX_LPUART_CTRL_RE_MASK | IMX_LPUART_CTRL_TE_MASK));

    /* Attach the resource manager */
    ttc(TTC_INIT_ATTACH, &dev->tty, 0);
#if defined(USE_DMA)
    if (dev->usedma) {
        /* Prepare RX DMA info structure */
        lpuart_prep_rx_dma_transfer(dev, tinfo, src_dma_addr, dst_dma_addr, dev->rx_dma.phys_addr);
        /* Set up DMA RX transfer */
        dev->sdmafuncs.setup_xfer(dev->rx_dma.dma_chn, &tinfo);
        /* Start DMA RX transfer */
        dev->sdmafuncs.xfer_start(dev->rx_dma.dma_chn);

        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "serial: DMA is enabled for device %s", dev->tty.name);
    } else {
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "serial: DMA is disabled for device %s", dev->tty.name);
    }
#endif
    return;
#if defined(USE_DMA)
fail9:
    dev->sdmafuncs.channel_release(dev->tx_dma.dma_chn);
fail8:
    dev->sdmafuncs.channel_release(dev->rx_dma.dma_chn);
fail7:
    if(dip->usesmmu) {
        smmu_obj_destroy(dev->smmu_obj);
    }
fail6:
    if (dip->usesmmu) {
        (*smmu_ports_created)--;
        if (*smmu_ports_created == 0) {
            smmu_fini();
        }
    }
fail5:
    dev->sdmafuncs.fini();
fail4:
    my_detach_pulse(&dev->rx_dma.pulse);
    my_detach_pulse(&dev->tx_dma.pulse);
    munmap(dev->rx_dma.buf, dev->rx_dma.xfer_size);
fail3:
    munmap(dev->tx_dma.buf, dev->tx_dma.xfer_size);
fail2:
    munmap_device_io(dev->base, IMX_LPUART_SIZE);


#endif
fail1:
    free(dev->tty.obuf.buff);
    free(dev->tty.ibuf.buff);
    free(dev->tty.cbuf.buff);
    free (dev);
    exit(1);

}

void dinit()
{
}


/** @} */ /* end of devc */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devc/sermx8/init.c $ $Rev: 894855 $")
#endif
