/*
 * $QNXLicenseC:
 * Copyright 2016 QNX Software Systems.
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

#ifndef FASTBOOT_H
#define FASTBOOT_H

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <sys/types.h>
#include <errno.h>
#include <hw/inout.h>

#define USB_STR_VENDOR_ID		0
#define USB_STR_PRODUCT_ID		1
#define USB_STR_SERIAL_ID		2
#define USB_STR_CONFIG_ID		3
#define USB_STR_INTERFACE_ID	4
#define USB_STR_MANUFACTURER_ID	5

#define CACHE_OP_INVAL			1
#define CACHE_OP_FLUSH			2

#define USBBOOT_VERSION		"0.4"

typedef struct _usb_string {
	uint8_t			id;
	const char		*str;
}usb_string_t;

typedef struct _usbboot_context {
	uint8_t		*download_buffer;
	size_t		download_buffer_size;
	paddr64_t	download_buffer_phys;
	uint8_t		*malloc_buffer;
	size_t		malloc_size;
	paddr64_t	malloc_buffer_phys;

	uint16_t		dev_vid;
	uint16_t		dev_did;
	uint16_t		langid;

	usb_string_t	*dev_strings;

	uint8_t		*base; //based address of the device controller
	uint32_t		ioport;
	int			verbose;

	int				(*dc_init)(void *);

	int				(*bs_init)(void **, uint32_t);
	int				(*bs_fini)(void *, uint32_t);
	int				(*bs_phy_reset)(void *, uint32_t);

	/*board specific utility routine*/
	void				(*bs_delay_ms)(uint32_t);
	void				(*bs_delay_ns)(uint32_t);
	void				(*bs_bus_sync)(void *, uint32_t);

	//command implementations and board specific callback
	int				(*reboot)(void);
	int				(*flash)(char *, char *);
	int				(*oem)(char *, char *);
	int				(*erase)(char *, char *);
	int				(*boot)(uint8_t *, size_t);
	int				(*getvar)(char *, char *, unsigned);
	int				(*dcache_op)(unsigned, unsigned, int);
	int				(*dcache_op_all)(int);
}usbboot_context_t;


extern int usb_boot_init(usbboot_context_t *cfg);
extern void usb_boot_shutdown(void);
extern int usb_boot_process(void);
extern int usb_boot_unsparse(void *hdl, unsigned long addr, unsigned blkno, unsigned isize, int (*blk_func)(void *, void *, unsigned, unsigned), void *fillBuf, int fillBufSz);
extern unsigned usb_boot_get_downloaded_bytes(void);
extern void usb_boot(unsigned image);

extern void strultostr ( char *cp, unsigned long n);
extern int strcmp_first(const char *s1, const char *s2);

//controller entry point we supported.
extern int dwcotg_init(void *udc);
extern int chip_idea_init(void *udc);

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/lib/usbboot/usbboot.h $ $Rev: 913227 $")
#endif
