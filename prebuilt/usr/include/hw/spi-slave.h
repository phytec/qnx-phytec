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


#ifndef _SPI_SLAVE_LIB_H_INCLUDED
#define _SPI_SLAVE_LIB_H_INCLUDED

#include <sys/iofunc.h>
#include <sys/dispatch.h>

#ifndef	__TYPES_H_INCLUDED
#include <sys/types.h>
#endif

#ifndef	_INTTYPES_H_INCLUDED
#include <inttypes.h>
#endif

#include <hw/spi-master.h>
#include <sys/iomsg.h>

#define SPI_SLAVE_HW_ERROR (SPI_SLAVE_HW_TX_FIFO_UNDERRUN | SPI_SLAVE_HW_RX_FIFO_OVERFLOW)

#define SPI_SLAVE_RX_DMA_OVERFLOW          0x0001
#define SPI_SLAVE_RX_DMA_ABORT             0x0002
#define SPI_SLAVE_TX_DMA_UNDERRUN          0x0004
#define SPI_SLAVE_TX_DMA_ABORT             0x0008
#define SPI_SLAVE_HW_TX_FIFO_UNDERRUN      0x0800
#define SPI_SLAVE_HW_RX_FIFO_OVERFLOW      0x1000

/*
 * The following devctls are used by a client application
 * to control the SPI interface.
 */
#define DCMD_SPI_GET_ERROR    __DIOF (_DCMD_SPI, 0x14, uint32_t)

/*
 * Hardware interface for low level driver
 */
typedef struct {
	/* size of this structure */
	size_t	size;

	/*
	 * Initialize master interface.
	 * Returns a handle associated with this driver
	 * Returns:
	 * !NULL    success
	 * NULL     failure
	 */
	void*	(*init)(void *hdl, char *options);

	/*
	 * Clean up driver.
	 * Frees memory associated with "hdl".
	 */
	void	(*fini)(void *hdl);

	/*
	 * Get driver information
	 */
	int		(*drvinfo)(void *hdl, spi_drvinfo_t *info);

	/*
	 * Get device information
	 */
	int		(*devinfo)(void *hdl, uint32_t device, spi_devinfo_t *info);

	/*
	 * Set SPI configuration
	 */
	int		(*setcfg)(void *hdl, uint16_t device, spi_cfg_t *cfg);

	/*
	 * Unsupported: xfer function calls
	 */
	void*	(*xfer)(void *hdl, uint32_t device, uint8_t *buf, int *len);

	/*
	 * Unsupported: DMA xfer function calls
	 */
	int		(*dma_xfer)(void *hdl, uint32_t device, spi_dma_paddr_t *paddr, int len);

	/*
	 * Read function calls
	 */
	void*	(*read)(void *hdl, uint32_t device, uint8_t *buf, int *len);

	/*
	 * Write function calls
	 */
	void*	(*write)(void *hdl, uint32_t device, uint8_t *buf, int *len);

	/*
	 * Get status function calls
	 */
	void*	(*status)(void *hdl, uint32_t device, int *status);

    /*
     * Get error function calls
     */
    int    (*error)(void *hdl, int *error);
} spi_slave_funcs_t;


#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/spi/slave/public/hw/spi-slave.h $ $Rev: 863314 $")
#endif
