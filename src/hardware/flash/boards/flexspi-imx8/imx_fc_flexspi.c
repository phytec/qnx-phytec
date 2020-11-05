/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017, 2019 NXP
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

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/slog.h>
#include <sys/slogcodes.h>
#include <sys/mman.h>
#include <pthread.h>
#include <string.h>

#include "imx_fc_flexspi.h"
#include "flexspi_cmds.h"

/**
 * @file       flexspi-imx8/imx_fc_flexspi.c
 * @addtogroup ffs3_fc Flash Controller
 * @{
 */

/* Driver command line options */
char *drv_opts = NULL;
/* Supported driver command line options */
static char *supported_opts[] = {
    "base",
    "irq",
    "smpl",
    "pads",
    "page_size",    /* Page size in Bytes */
    "size",         /* Device capacity in Mbit */
    "die",          /* Number of die */
    "dummy",        /* Dummy cycles for read command(s) */
    NULL
};


/**
 * Second part of controller initialization.
 *
 * @param dev Low level driver handle.
 *
 * @return EOK always.
 */
int imx_flexspi_setcfg(imx_fspi_t* dev)
{
    int status;

    /* IP command address */
    out32(dev->vbase + IMX_FLEXSPI_IPCR0, 0x0);

    dev->mema1 = 0;
    dev->mema2 = 0;
    dev->memb1 = 0;
    dev->memb2 = 0;

    /* Individual mode
     * FLSHA1CRx/FLSHA2CRx/FLSHB1CRx/FLSHB2CRx setting will be applied to Flash A1/A2/B1/B2 separately
     */
    out32(dev->vbase + IMX_FLEXSPI_MCR2, in32(dev->vbase + IMX_FLEXSPI_MCR2) & ~IMX_FLEXSPI_MCR2_SAMEDEVICEEN_MASK);

    /* Setup address space */
    switch (dev->die) {
        case 2    :
            dev->mema1 = dev->size / 1024 / 2; /* Size of the memory in KB */
            dev->memb1 = dev->mema1;
            break;
        default   :
            dev->mema1 = dev->size / 1024; /* Size of the memory in KB */
    }
    out32(dev->vbase + IMX_FLEXSPI_FLSHA1CR0, dev->mema1);
    out32(dev->vbase + IMX_FLEXSPI_FLSHA2CR0, dev->mema2);
    out32(dev->vbase + IMX_FLEXSPI_FLSHB1CR0, dev->memb1);
    out32(dev->vbase + IMX_FLEXSPI_FLSHB2CR0, dev->memb2);

    /* Initialize command look-up table */
    status = init_lut(dev);

    return status;
}

/**
 * FLEXSPI interrupt handler.
 *
 * @param area Low level driver.
 * @param id   Unused parameter.
 *
 * @return NULL or EVENT
 */
static const struct sigevent *qspi_intr(void *area, __attribute__((unused)) int id)
{
    imx_fspi_t *dev = area;

    if (dev->irq_requested) {
        dev->irq_requested = 0;
        out32(dev->vbase + IMX_FLEXSPI_INTEN, 0x0); /* Disable all interrupts */
        return &dev->fspievent;
    }
    if (dev->rx_data_len > 0) {
        /* Read data from hw */
        memcpy_isr(dev->buf, (void*)(dev->vbase + IMX_FLEXSPI_RFDR0), dev->rx_watermark);
        /* FIFO shift */
        out32(dev->vbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPRXWA_MASK);
        /* Check if more data is expected */
        dev->rx_data_len = dev->rx_data_len - dev->rx_watermark;
        if (dev->rx_data_len == 0) {
            /* Return from interrupt */
            return &dev->fspievent;
        }
        /* Update pointer value */
        dev->buf = dev->buf + dev->rx_watermark;
        /* Update watermark value */
        dev->rx_watermark = min(dev->rx_data_len, IMX_FSPI_MAX_RX_FIFO_WINDOW);
        /* Write updated watermark value to hw */
        imx_flexspi_set_rx_watermark(dev, dev->rx_watermark);
    }

    return NULL;
}

/**
 * Parse driver command line options.
 *
 * @param dev Low level driver handle.
 */
static void imx_flexspi_driver_option(imx_fspi_t *dev)
{
    char    *value, *freeptr, *options;
    int     opt;

    if (drv_opts == NULL)
        return;

    freeptr = strdup(drv_opts);

    options = freeptr;

    while (options && *options != '\0') {
        opt = getsubopt(&options, supported_opts, &value);
        switch (opt) {
            case 0:
                dev->pbase  = strtoul(value, 0, 0);
                break;
            case 1:
                dev->irq = strtoul(value, 0, 0);
                break;
            case 2:
                dev->smpl = strtoul(value, 0, 0);
                break;
            case 3:
                dev->pads = strtoul(value, 0, 0);
                break;
            case 4:
                dev->page_size = strtoul(value, 0, 0);
                break;
            case 5:
            /* same as in the datasheet, size command line option is in Mbits
             * convert to bytes
             */
                dev->size = ( strtoul(value, 0, 0) * 1024 * 1024) / 8 ;
                break;
            case 6:
                dev->die = strtoul(value, 0, 0);
                break;
            case 7:
                dev->dummy = strtoul(value, 0, 0);
                break;
        }
    }

    free(freeptr);
}

/**
 * First part of controller initialization.
 *
 * @return File descriptor.
 */
imx_fspi_t *imx_flexspi_open(void)
{
    static imx_fspi_t *dev;
    uint32_t mcr_reg;

    if (ThreadCtl(_NTO_TCTL_IO_PRIV, 0) == -1) {
        fprintf(stderr, "norqspi: ThreadCtl Failed\n");
        return NULL;
    }
    if ((dev = calloc(1, sizeof(imx_fspi_t))) == NULL) {
        fprintf(stderr, "norqspi: Could not allocate imx_qspi_t memory\n");
        return NULL;
    }
    /* Load defaults */
    dev->pbase = IMX_FLEXSPI_DEFAULT_BASE_ADDR;        /* Physical base address */
    dev->irq = IMX_FLEXSPI_DEFAULT_IRQ;                /* IRQ */
    dev->smpl = IMX_FLEXSPI_MCR0_RXCLKSRC_BV_FLSH_DQS; /* Flash provided Read strobe and input from DQS pad */
    dev->pads = 8;                                     /* Bus width in bits */
    dev->page_size = 256;                              /* Flash Page size */
    dev->size = 64 * 1024 * 1024;                      /* Flash size in bytes */
    dev->die = 1;                                      /* Number of dies in flash */
    dev->dummy = 16;                                   /* Dummy cycles for read command */
    /* Load driver command line options */
    if (drv_opts != NULL) {
        imx_flexspi_driver_option(dev);
    }
    /* Map io space */
    dev->vbase = mmap_device_io(IMX_FLEXSPI_SIZE, dev->pbase);
    if ((dev->vbase) == (uintptr_t) MAP_FAILED) {
        goto fail0;
    }
    /* Enable FLEXSPI clock */
    mcr_reg = in32(dev->vbase + IMX_FLEXSPI_MCR0);
    mcr_reg &= ~IMX_FLEXSPI_MCR0_MDIS_MASK;
    out32(dev->vbase + IMX_FLEXSPI_MCR0, mcr_reg);
    /* Configure FLEXSPI to use IP access and COMBINATION mode
     * RXCLKSRC - Flash provided Read strobe and input from DQS pad */
    mcr_reg = in32(dev->vbase + IMX_FLEXSPI_MCR0);
    mcr_reg &= ~(IMX_FLEXSPI_MCR0_ARDFEN_MASK | IMX_FLEXSPI_MCR0_ATDFEN_MASK | IMX_FLEXSPI_MCR0_RXCLKSRC_MASK |
                 IMX_FLEXSPI_MCR0_COMBINATIONEN_MASK);
    if (dev->pads == 8) {
        mcr_reg |= IMX_FLEXSPI_MCR0_COMBINATIONEN_MASK;
    }
    mcr_reg |= (dev->smpl << IMX_FLEXSPI_MCR0_RXCLKSRC_SHIFT);
    out32(dev->vbase + IMX_FLEXSPI_MCR0, mcr_reg);
    /* Reset FLEXSPI */
    mcr_reg = in32(dev->vbase + IMX_FLEXSPI_MCR0);
    mcr_reg |= IMX_FLEXSPI_MCR0_SWRESET_MASK;
    out32(dev->vbase + IMX_FLEXSPI_MCR0, mcr_reg);
    /* Wait 1ms for domains reset */
    delay(1);
    /* Enable delay line calibration (Flash provided read strobe) */
    out32(dev->vbase + IMX_FLEXSPI_DLLACR, IMX_FLEXSPI_DLLACR_DLLEN_MASK);
    /* Clear RX and TX FIFOs */
    out32(dev->vbase + IMX_FLEXSPI_IPTXFCR, IMX_FLEXSPI_IPTXFCR_CLRIPTXF_MASK);
    out32(dev->vbase + IMX_FLEXSPI_IPRXFCR, IMX_FLEXSPI_IPRXFCR_CLRIPRXF_MASK);
    /* Disable all interrupts */
    out32(dev->vbase + IMX_FLEXSPI_INTEN, 0x0);
    /* Clear all interrupts */
    out32(dev->vbase + IMX_FLEXSPI_INTR, 0xFFF);
    /* Interrupt handling initialization */
    dev->chid = ChannelCreate(_NTO_CHF_DISCONNECT | _NTO_CHF_UNBLOCK);
    if (dev->iid == -1) {
        goto fail1;
    }
    dev->coid = ConnectAttach(0, 0, dev->chid, _NTO_SIDE_CHANNEL, 0);
    if (dev->coid == -1) {
        goto fail2;
    }
    /* Event initialization */
    SIGEV_PULSE_INIT(&dev->fspievent, dev->coid, IMX_FSPI_PRIORITY, IMX_FSPI_EVENT, NULL);
    /* Interrupt initialization */
    dev->iid = InterruptAttach(dev->irq, qspi_intr, dev, 0, _NTO_INTR_FLAGS_TRK_MSK);
    if (dev->iid == -1) {
        goto fail3;
    }

    /* Verify buffer */
    dev->fd = posix_typed_mem_open("/memory/below4G", O_RDWR, POSIX_TYPED_MEM_ALLOCATE_CONTIG);
    if (dev->fd == -1) {
        /* If no fd exists try to allocate buffer
         * using mmap (system with less than 4GB RAM)*/
        dev->verify = mmap(0, dev->page_size,
                           PROT_READ | PROT_WRITE | PROT_NOCACHE,
                           MAP_ANON | MAP_PHYS | MAP_PRIVATE, NOFD, 0);
    } else {
        /* Allocate memory under 4GB RAM on systems with
         * more than 4GB. DMA supports 32-bit address range.*/
        dev->verify = mmap(NULL, dev->page_size,
                           PROT_READ | PROT_WRITE | PROT_NOCACHE,
                           MAP_SHARED, dev->fd, 0);
    }
    if (dev->verify == MAP_FAILED) {
        goto fail4;
    }

    return dev;

fail4:
    InterruptDetach(dev->iid);
fail3:
    ConnectDetach(dev->coid);
fail2:
    ChannelDestroy(dev->chid);
fail1:
    munmap_device_io(dev->vbase, IMX_FLEXSPI_SIZE);
fail0:
    free(dev);
    return NULL;
}

/**
 * De-init method of the controller.
 *
 * @param dev Low level driver handle.
 *
 * @return EOK always.
 */
int imx_flexspi_close(imx_fspi_t *dev)
{
    out32(dev->vbase + IMX_FLEXSPI_MCR0, IMX_FLEXSPI_MCR0_MDIS_MASK);
    ConnectDetach(dev->coid);
    ChannelDestroy(dev->chid);
    InterruptDetach(dev->iid);
    munmap_device_io(dev->vbase, IMX_FLEXSPI_SIZE);
    munmap(dev->verify, dev->page_size);
    free(dev);

    return EOK;
}

/**
 * Locks look-up table content.
 *
 * @param dev Low level driver handle.
 *
 * @return EOK always.
 */
int imx_flexspi_lock_lut(imx_fspi_t *dev)
{
    out32(dev->vbase + IMX_FLEXSPI_LUTKEY, IMX_FLEXSPI_LUT_KEY_VAL);
    out32(dev->vbase + IMX_FLEXSPI_LUTCR, IMX_FLEXSPI_LUTCR_LOCK_MASK);

    return EOK;
}

/**
 * Unlocks look-up table content.
 *
 * @param dev Low level driver handle.
 *
 * @return EOK always.
 */
int imx_flexspi_unlock_lut(imx_fspi_t *dev)
{
    out32(dev->vbase + IMX_FLEXSPI_LUTKEY, IMX_FLEXSPI_LUT_KEY_VAL);
    out32(dev->vbase + IMX_FLEXSPI_LUTCR, IMX_FLEXSPI_LUTCR_UNLOCK_MASK);

    return EOK;
}

/**
 * Set RX water-mark level according to data transfer size.
 *
 * @param[in] dev                Low level driver handle.
 * @param[in] data_transfer_size Transfer data size.
 */
void imx_flexspi_set_rx_watermark(imx_fspi_t *dev, uint32_t data_transfer_size)
{
    /* Data transfer size logic must be dword aligned */
    while ((data_transfer_size % 8) != 0) {
        data_transfer_size++;
    }
    out32(dev->vbase + IMX_FLEXSPI_IPRXFCR, (data_transfer_size / 8 - 1) << IMX_FLEXSPI_IPRXFCR_RXWMRK_SHIFT);
}

/**
 * Set TX water-mark level according to data transfer size.
 *
 * @param[in] dev                Low level driver handle.
 * @param[in] data_transfer_size Transfer data size.
 */
void imx_flexspi_set_tx_watermark(imx_fspi_t *dev, uint32_t data_transfer_size)
{
    /* Data transfer size logic must be dword aligned */
    while ((data_transfer_size % 8) != 0) {
        data_transfer_size++;
    }
    out32(dev->vbase + IMX_FLEXSPI_IPTXFCR, (data_transfer_size / 8 - 1) << IMX_FLEXSPI_IPTXFCR_TXWMRK_SHIFT);
}

/**
 * Creates look-up record. One look-up entry contains up to 4 records. See imx_flexspi_write_lut for details.
 *
 * @param instr0 Controller command.
 * @param pad0   Number of pins used for communication.
 * @param opr0   Device command, data size, etc.
 * @param instr1 Controller command.
 * @param pad1   Number of pins used for communication.
 * @param opr1   Device command, data size, etc.
 *
 * @return created record
 */
imx_fspi_lut_t imx_flexspi_create_lut_record(uint8_t instr0, uint8_t pad0, uint8_t opr0,
                                             uint8_t instr1, uint8_t pad1, uint8_t opr1)
{
    imx_fspi_lut_t lut_rec;

    lut_rec.B.INSTR0 = instr0;
    lut_rec.B.PAD0 = pad0;
    lut_rec.B.OPRND0 = opr0;
    lut_rec.B.INSTR1 = instr1;
    lut_rec.B.PAD1 = pad1;
    lut_rec.B.OPRND1 = opr1;

    return lut_rec;
}

/**
 * Creates look-up table entry.
 *
 * @param dev     Low level driver handle.
 * @param index   Index in look-up table.
 * @param lutcmd0 Record 0.
 * @param lutcmd1 Record 1.
 * @param lutcmd2 Record 2.
 * @param lutcmd3 Record 3.
 *
 * @return EOK always.
 */
int imx_flexspi_write_lut(imx_fspi_t *dev, uint8_t index, imx_fspi_lut_t *lutcmd0, imx_fspi_lut_t *lutcmd1,
                          imx_fspi_lut_t *lutcmd2, imx_fspi_lut_t *lutcmd3)
{
    uint8_t inner_index = index * 4;

    out32(dev->vbase + IMX_FLEXSPI_LUTa(inner_index), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(++inner_index), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(++inner_index), 0);
    out32(dev->vbase + IMX_FLEXSPI_LUTa(++inner_index), 0);

    inner_index = index * 4;

    if (lutcmd0 != NULL) {
        out32(dev->vbase + IMX_FLEXSPI_LUTa(inner_index), lutcmd0->U);
    }
    if (lutcmd1 != NULL) {
        out32(dev->vbase + IMX_FLEXSPI_LUTa(++inner_index), lutcmd1->U);
    }
    if (lutcmd2 != NULL) {
        out32(dev->vbase + IMX_FLEXSPI_LUTa(++inner_index), lutcmd2->U);
    }
    if (lutcmd3 != NULL) {
        out32(dev->vbase + IMX_FLEXSPI_LUTa(++inner_index), lutcmd3->U);
    }

    return EOK;
}

/**
 * Wait routine.
 *
 * @param dev Low level driver handle.
 *
 * @retval EIO Returned in case of error.
 * @retval EOK Everything is fine.
 */
int flexspi_intr_wait(imx_fspi_t *dev)
{
    struct _pulse   pulse;
    uint64_t        ntime = 1e9;

    for (;;) {
        TimerTimeout(CLOCK_REALTIME, _NTO_TIMEOUT_RECEIVE, NULL, &ntime, NULL);
        if (MsgReceivePulse(dev->chid, &pulse, sizeof(pulse), NULL) == -1) {
            dev->irq_requested = 0;
            slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "(devf t%d::%s:%d) timeout\n", pthread_self(), __func__, __LINE__);
            return (EIO);
        }
        if (pulse.code == IMX_FSPI_EVENT) {
            return EOK;
        }
    }
    return EIO;
}

/**
 * Sends command to connected device. General IP command. Waits for cmd finish.
 *
 * @param dev                Low level driver handle.
 * @param lut_index          Look-up table index.
 * @param data_size_override Optional parameter that will override default value of data size in LUT.
 *
 * @retval EOK - Everything is fine.
 * @retval EIO - Error in command processing.
 */
int imx_flexspi_send_ip_cmd(imx_fspi_t *dev, uint8_t lut_index, uint32_t data_size_override)
{
    out32(dev->vbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPCMDDONE_MASK |
          IMX_FLEXSPI_INTR_IPCMDERR_MASK |
          IMX_FLEXSPI_INTR_IPCMDGE_MASK); /* Clear IP cmd done flag */
    dev->irq_requested = 1;
    out32(dev->vbase + IMX_FLEXSPI_INTEN, IMX_FLEXSPI_INTEN_IPCMDDONEEN_MASK |
          IMX_FLEXSPI_INTEN_IPCMDERREN_MASK |
          IMX_FLEXSPI_INTEN_IPCMDGEEN_MASK); /* Enable IP cmd done interrupt */
    /* Set IP command sequence index */
    out32(dev->vbase + IMX_FLEXSPI_IPCR1, (lut_index << IMX_FLEXSPI_IPCR1_ISEQID_SHIFT) | data_size_override);
    /* Trigger IP command */
    out32(dev->vbase + IMX_FLEXSPI_IPCMD, 0x1);
    /* Wait for interrupt */
    if (flexspi_intr_wait(dev)) {
        return EIO;
    }
    /* Clear IP cmd done flag */
    out32(dev->vbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPCMDDONE_MASK |
          IMX_FLEXSPI_INTR_IPCMDERR_MASK |
          IMX_FLEXSPI_INTR_IPCMDGE_MASK);
    return EOK;
}

/**
 * Clear either Rx or Tx FIFO or both. Depends on mask parameter.
 *
 * @param dev     Low level driver handle.
 * @param mask    Mask of the FIFO(s).
 *
 * @return EOK always.
 */
int imx_flexspi_clear_fifo(imx_fspi_t *dev, imx_fspi_bufclr_t mask)
{
    uint32_t ipr_reg;

    if (mask & rx) {
        ipr_reg = in32(dev->vbase + IMX_FLEXSPI_IPRXFCR);
        ipr_reg |= IMX_FLEXSPI_IPRXFCR_CLRIPRXF_MASK;
        out32(dev->vbase + IMX_FLEXSPI_IPRXFCR, ipr_reg);
    }
    if (mask & tx) {
        ipr_reg = in32(dev->vbase + IMX_FLEXSPI_IPTXFCR);
        ipr_reg |= IMX_FLEXSPI_IPTXFCR_CLRIPTXF_MASK;
        out32(dev->vbase + IMX_FLEXSPI_IPTXFCR, ipr_reg);
    }

    return EOK;
}

/**
 * Writes data to FLEXSPI Tx FIFO.
 *
 * @param dev       Low level driver handle.
 * @param addr      Address of the write data buffer.
 * @param data_size Size of data to write.
 *
 * @retval EIO Data size exceed Tx FIFO size or no data to send.
 * @retval EOK Everything is fine.
 */
int imx_flexspi_write_data(imx_fspi_t *dev, uint8_t *addr, uint32_t data_size)
{
    if (data_size > IMX_FSPI_MAX_TX_FIFO_WINDOW) {
        slogf(_SLOGC_FS_FFS, _SLOG_ERROR, "(devf  t%d::%s:%d) data_size=%x exceeded write window %d",
              pthread_self(), __func__, __LINE__, data_size, IMX_FSPI_MAX_TX_FIFO_WINDOW);
        return EIO;
    }
    if (!(data_size > 0)) {
        return EIO;
    }
    /* Clear Tx FIFO */
    imx_flexspi_clear_fifo(dev, tx);
    /* Set water-mark */
    imx_flexspi_set_tx_watermark(dev, data_size);
    /* Write data */
    memcpy((void*)(dev->vbase + IMX_FLEXSPI_TFDR0), addr, data_size);
    /* Push data to Tx FIFO */
    out32(dev->vbase + IMX_FLEXSPI_INTR, in32(dev->vbase + IMX_FLEXSPI_INTR) | IMX_FLEXSPI_INTR_IPTXWE_MASK);

    return EOK;
}

/**
 * Reads data from FLEXSPI Rx FIFO.
 *
 * @param dev    Low level driver handle.
 * @param buffer Pointer where to copy Rx data.
 * @param size   Expected amount of data.
 *
 * @return Read data size. -1 if any error.
 */
int imx_flexspi_read_data(imx_fspi_t *dev, uint8_t *buffer, uint32_t size)
{
    /* Enable Rx interrupt */
    dev->irq_requested = 1;
    out32(dev->vbase + IMX_FLEXSPI_INTEN, IMX_FLEXSPI_INTEN_IPRXWAEN_MASK);
    /* Wait for interrupt */
    if (flexspi_intr_wait(dev)) {
        return -1;
    }
    /* Read data from hw */
    memcpy(buffer, (void*)(dev->vbase + IMX_FLEXSPI_RFDR0), size);
    /* Clear IP and Rx flags */
    out32(dev->vbase + IMX_FLEXSPI_INTR, IMX_FLEXSPI_INTR_IPRXWA_MASK | IMX_FLEXSPI_INTR_IPCMDDONE_MASK);

    return (size);
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/flash/boards/flexspi-imx8/imx_fc_flexspi.c $ $Rev: 893539 $")
#endif
