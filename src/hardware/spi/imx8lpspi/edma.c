/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
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

#include <fcntl.h>
#include <sys/mman.h>
#ifdef EDMA_SUPPORT
#include <aarch64/imx8_common/imx_edma.h>
#endif
#include "mx8lpspi.h"
#include "smmu.h"

/**
 * @file       imx8lpsi/edma.c
 * @addtogroup spi
 * @{
 */

/**
 * Create buffer for DMA functionality.
 *
 * @param dev   Low level driver handle.
 * @param pbuff Physical buffer address.
 * @param size  Size of the buffer.
 *
 * @return Virtual buffer address.
 */
void *imxspi_edma_create_buffer(imx_spi_t *dev, uint32_t *pbuff, uint32_t size)
{
    off64_t           offset;
    void              *buff;

    /* DMA buffer */
    dev->fd = posix_typed_mem_open("/memory/below4G", O_RDWR, POSIX_TYPED_MEM_ALLOCATE_CONTIG);
    if (dev->fd == -1) {
        /* If no fd exists try to allocate buffer
         * using mmap (system with less than 4GB RAM)*/
        buff = mmap(0, size,
                           PROT_READ | PROT_WRITE | PROT_NOCACHE,
                           MAP_ANON | MAP_PHYS | MAP_PRIVATE, NOFD, 0);
    } else {
        /* Allocate memory under 4GB RAM on systems with
         * more than 4GB. DMA supports 32-bit address range.*/
        buff = mmap(NULL, size,
                           PROT_READ | PROT_WRITE | PROT_NOCACHE,
                           MAP_SHARED, dev->fd, 0);
    }
    if (buff != MAP_FAILED) {
        if (mem_offset64(buff, NOFD, 1, &offset, 0) == -1) {
            LOG_ERROR("(spi  t%d::%s:%d) Error in address conversion", pthread_self(), __func__, __LINE__);
            munmap(buff, size);
            return NULL;
        }
        *pbuff = offset;
    }
    return buff;
}

/**
 * Function responsible for correct DMA pulse/API initialization.
 *
 * @param dev Low level driver handle.
 *
 * @return Execution status. EOK or -1 otherwise.
 */
int imxspi_attach_edma(imx_spi_t *dev)
{
#ifdef EDMA_SUPPORT
    dma_driver_info_t sdma_info;
    struct smmu_map_entry smmu_buffer_entry;

    unsigned          rxchannel, txchannel;

    /* DMA internal buffer */
    if ((dev->dmabuf = imxspi_edma_create_buffer(dev, &dev->pdmabuf, IMX_SPI_DRV_DMA_BUF_SIZE)) == NULL) {
        goto attach_edma_fail1;
    }
    /* EDMA API registration */
    if (get_dmafuncs(&dev->dmafuncs, sizeof(dma_functions_t)) == -1) {
        goto attach_edma_fail2;
    }
    dev->dmafuncs.driver_info(&sdma_info);
    if (dev->dmafuncs.init(NULL) == -1) {
        goto attach_edma_fail2;
    }

    /* RX EDMA pulse initialization */
    if ((dev->erxcoid = ConnectAttach(0, 0, dev->chid, _NTO_SIDE_CHANNEL, 0)) == -1) {
        goto attach_edma_fail2;
    }
    SIGEV_PULSE_INIT(&dev->spi_rxedma_event, dev->erxcoid, IMX_SPI_PRIORITY, IMX_SPI_RX_DMA_EVENT, NULL);

    /* TX EDMA pulse initialization */
    if ((dev->etxcoid = ConnectAttach(0, 0, dev->chid, _NTO_SIDE_CHANNEL, 0)) == -1) {
        goto attach_edma_fail3;
    }
    SIGEV_PULSE_INIT(&dev->spi_txedma_event, dev->etxcoid, IMX_SPI_PRIORITY, IMX_SPI_TX_DMA_EVENT, NULL);


    /* SMMU Mappings Enabled */
    if(dev->use_smmu) {
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "SPI SMMU mappings enabled for port %p", &dev->pbase);

        /* Create connection to SMMU and add mappings */
        if (dev->smmu_init == 0) {
	        /* Make sure to only call this function once per process */
	        if (smmu_init(0) == -1) {
	            LOG_ERROR("(spi  t%d::%s:%d) Failed to connect to SMMU", pthread_self(), __func__, __LINE__);
                goto attach_edma_fail4;
	        }
        }

        /* Track how many ports are being opened */
        dev->smmu_init++;

        dev->smmu_obj = smmu_obj_create(SOCF_NONE);
        if (dev->smmu_obj == NULL) {
            LOG_ERROR("(spi  t%d::%s:%d) Failed to create SMMU object", pthread_self(), __func__, __LINE__);
            goto attach_edma_fail5;
        }

        /* RX EDMA Channel attach */
        rxchannel = imx_edma_define_request(dev->rxedma_vect.edmanum, dev->rxedma_vect.channelnum, dev->rxedma_vect.irqnum);
        if ((dev->edma_rx_handler = dev->dmafuncs.channel_attach_smmu(NULL,
                                                                 &dev->spi_rxedma_event,
                                                                 &rxchannel,
                                                                 sdma_info.max_priority,
                                                                 dev->slave ? DMA_ATTACH_EVENT_PER_SEGMENT : DMA_ATTACH_EVENT_ON_COMPLETE,
                                                                 dev->smmu_obj)) == NULL) {
            goto attach_edma_fail6;
        }

        /* Tx EDMA Channel attach */
        txchannel = imx_edma_define_request(dev->txedma_vect.edmanum, dev->txedma_vect.channelnum, dev->txedma_vect.irqnum);
        if ((dev->edma_tx_handler = dev->dmafuncs.channel_attach_smmu(NULL,
                                                                 &dev->spi_txedma_event,
                                                                 &txchannel,
                                                                 sdma_info.max_priority,
                                                                 DMA_ATTACH_EVENT_ON_COMPLETE, dev->smmu_obj)) == NULL) {
            goto attach_edma_fail6;
        }

        smmu_buffer_entry.phys = (uintptr64_t)dev->dmabuf;
        smmu_buffer_entry.len = IMX_SPI_DRV_DMA_BUF_SIZE;

        if ( smmu_mapping_add(dev->smmu_obj, SMF_READ|SMF_WRITE, 0, 1, &smmu_buffer_entry, 0) == -1 ) {
            LOG_ERROR("(spi  t%d::%s:%d) Failed to add buffer mappings", pthread_self(), __func__, __LINE__);
            goto attach_edma_fail6;
        }

    } else {

        /* RX EDMA Channel attach */
        rxchannel = imx_edma_define_request(dev->rxedma_vect.edmanum, dev->rxedma_vect.channelnum, dev->rxedma_vect.irqnum);
        if ((dev->edma_rx_handler = dev->dmafuncs.channel_attach(NULL,
                                                                 &dev->spi_rxedma_event,
                                                                 &rxchannel,
                                                                 sdma_info.max_priority,
                                                                 dev->slave ? DMA_ATTACH_EVENT_PER_SEGMENT : DMA_ATTACH_EVENT_ON_COMPLETE)) == NULL) {
            goto attach_edma_fail4;
        }

        /* Tx EDMA Channel attach */
        txchannel = imx_edma_define_request(dev->txedma_vect.edmanum, dev->txedma_vect.channelnum, dev->txedma_vect.irqnum);
        if ((dev->edma_tx_handler = dev->dmafuncs.channel_attach(NULL,
                                                                 &dev->spi_txedma_event,
                                                                 &txchannel,
                                                                 sdma_info.max_priority,
                                                                 DMA_ATTACH_EVENT_ON_COMPLETE)) == NULL) {
            goto attach_edma_fail4;
        }
    }

    return EOK;


attach_edma_fail6:
    smmu_obj_destroy(dev->smmu_obj);

attach_edma_fail5:
    dev->smmu_init--;
    if ( dev->smmu_init == 0 ) {
        smmu_fini();
    }

attach_edma_fail4:
    ConnectDetach(dev->etxcoid);

attach_edma_fail3:
    ConnectDetach(dev->erxcoid);

attach_edma_fail2:
    munmap(dev->dmabuf, IMX_SPI_DRV_DMA_BUF_SIZE);

attach_edma_fail1:
    return -1;

#else
    return -1;
#endif
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/spi/imx8lpspi/edma.c $ $Rev: 894855 $")
#endif
