/*
 * $QNXLicenseC:
 * Copyright 2009, QNX Software Systems.
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

#include <stdint.h>
#include <sys/types.h>
#include <hw/inout.h>

#ifndef __PIO_PCI_H_INCLUDED
#define __PIO_PCI_H_INCLUDED

/*
 * Standard PCI configuration space registers and offsets
 */
#define PCI_COMMAND 0x4
#define PCI_REVISION_ID 0x8
#define PCI_SUBSYSTEM_VENDOR_ID 0x2C
#define PCI_SUBSYSTEM_ID 0x2E

#define PCI_CFG_ADDR(b, d, f) ( 0x80000000          | \
		                       (((b) & 0xff) << 16) | \
		                       (((d) & 0x1f) << 11) | \
		                       (((f) & 0x07) <<  8))

#define PCIEX_CFG_ADDR(__b, __d, __f, __r) ( 0xE0000000 + ((((__b) &  0xff) << 20) | \
                                                  (((__d) &  0x1f) << 15) | \
                                                  (((__f) &  0x0f) << 12) | \
                                                  (((__r) & 0xfff))))

/*
 * Utility functions for reading within the PCI configuration space
 */
static inline uint32_t
pci_rd_cfg32(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
	if ((reg & 3) == 0) {
	  out32(0xcf8, PCI_CFG_ADDR(bus, dev, func) + reg);
	  return in32 (0xcfc);
	}

	return 0;
}

static inline uint16_t
pci_rd_cfg16(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
	uint8_t shift;

	if ((reg & 1) == 0) {
		shift = (reg & 3) * 8;
		out32(0xcf8, PCI_CFG_ADDR(bus, dev, func) + reg);
		return (uint16_t)((in32(0xcfc) >> shift) & 0xffff);
	}

	return 0;
}

static inline uint8_t
pci_rd_cfg8(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg)
{
	uint8_t shift = (reg & 3) * 8;
	out32(0xcf8, PCI_CFG_ADDR(bus, dev, func) + reg);
	return (uint8_t)((in32(0xcfc) >> shift) & 0xff);
}

/*
 * Utility functions for writing to the PCI configuration space
 */
static inline void
pci_wr_cfg32(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint32_t val)
{
	if ((reg & 3) == 0) {
		out32(0xcf8, PCI_CFG_ADDR(bus, dev, func) + reg);
		out32(0xcfc, val);
	}
}

static inline void
pci_wr_cfg16(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint16_t val)
{
	if ((reg & 1) == 0) {
		out32(0xcf8, PCI_CFG_ADDR(bus, dev, func) + reg);
		out16(0xcfc + (reg & 3), val);
	}
}

static inline void
pci_wr_cfg8(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint8_t val)
{
	out32(0xcf8, PCI_CFG_ADDR(bus, dev, func) + reg);
	out8 (0xcfc + (reg & 3), val);
}

/*
 * Utility functions for masked writing to PCI configuration space registers
 */
static inline void
pci_reg_wr_cfg32(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint32_t mask, uint32_t val)
{
	pci_wr_cfg32(bus, dev, func, reg, ((pci_rd_cfg32(bus, dev, func, reg) & ~mask) | val));
}

static inline void
pci_reg_wr_cfg16(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint16_t mask, uint16_t val)
{
	pci_wr_cfg16(bus, dev, func, reg, ((pci_rd_cfg16(bus, dev, func, reg) & ~mask) | val));
}

static inline void
pci_reg_wr_cfg8(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint8_t mask, uint8_t val)
{
	pci_wr_cfg8(bus, dev, func, reg, ((pci_rd_cfg8(bus, dev, func, reg) & ~mask) | val));
}

/*
 * Utility functions for setting bits in the PCI configuration space
 */
static inline void
pci_set_cfg32(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint32_t bits)
{
	pci_wr_cfg32(bus, dev, func, reg, pci_rd_cfg32(bus, dev, func, reg) | bits);
}

static inline void
pci_set_cfg16(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint16_t bits)
{
	pci_wr_cfg16(bus, dev, func, reg, pci_rd_cfg16(bus, dev, func, reg) | bits);
}

static inline void
pci_set_cfg8(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint8_t bits)
{
	pci_wr_cfg8(bus, dev, func, reg, pci_rd_cfg8(bus, dev, func, reg) | bits);
}

/*
 * Utility functions for clearing bits in the PCI configuration space
 */
static inline void
pci_clear_cfg32(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint32_t bits)
{
	pci_wr_cfg32(bus, dev, func, reg, pci_rd_cfg32(bus, dev, func, reg) & ~bits);
}

static inline void
pci_clear_cfg16(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint16_t bits)
{
	pci_wr_cfg16(bus, dev, func, reg, pci_rd_cfg16(bus, dev, func, reg) & ~bits);
}

static inline void
pci_clear_cfg8(uint8_t bus, uint8_t dev, uint8_t func, uint8_t reg, uint8_t bits)
{
	pci_wr_cfg8(bus, dev, func, reg, pci_rd_cfg8(bus, dev, func, reg) & ~bits);
}

/*
 * Utility functions for reading from the PCI extended configuration space
 */
static inline uint8_t
pciex_rd_cfg8(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg)
{
	return *(volatile uint8_t*)(uintptr_t)PCIEX_CFG_ADDR(bus, dev, func, reg);
}

static inline uint16_t
pciex_rd_cfg16(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg)
{
	return *(volatile uint16_t*)(uintptr_t)PCIEX_CFG_ADDR(bus, dev, func, reg);
}

static inline uint32_t
pciex_rd_cfg32(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg)
{
	return *(volatile uint32_t*)(uintptr_t)PCIEX_CFG_ADDR(bus, dev, func, reg);
}

/*
 * Utility functions for writing to the PCI extended configuration space
 */
static inline void
pciex_wr_cfg8(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint8_t val)
{
	*(volatile uint8_t*)(uintptr_t)PCIEX_CFG_ADDR(bus, dev, func, reg) = val;
}

static inline void
pciex_wr_cfg16(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint16_t val)
{
	*(volatile uint16_t*)(uintptr_t)PCIEX_CFG_ADDR(bus, dev, func, reg) = val;
}

static inline void
pciex_wr_cfg32(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint32_t val)
{
	*(volatile uint32_t*)(uintptr_t)PCIEX_CFG_ADDR(bus, dev, func, reg) = val;
}

/*
 * Utility functions for setting bits in the PCI extended configuration space
 */
static inline void
pciex_set_cfg8(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint8_t bits)
{
	pciex_wr_cfg8(bus, dev, func, reg, pciex_rd_cfg8(bus, dev, func, reg) | bits);
}

static inline void
pciex_set_cfg16(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint16_t bits)
{
	pciex_wr_cfg16(bus, dev, func, reg, pciex_rd_cfg16(bus, dev, func, reg) | bits);
}

static inline void
pciex_set_cfg32(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint32_t bits)
{
	pciex_wr_cfg32(bus, dev, func, reg, pciex_rd_cfg32(bus, dev, func, reg) | bits);
}

/*
 * Utility functions for clearing bits in the PCI extended configuration space
 */
static inline void
pciex_clear_cfg8(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint8_t bits)
{
	pciex_wr_cfg8(bus, dev, func, reg, pciex_rd_cfg8(bus, dev, func, reg) & ~bits);
}

static inline void
pciex_clear_cfg16(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint16_t bits)
{
	pciex_wr_cfg16(bus, dev, func, reg, pciex_rd_cfg16(bus, dev, func, reg) & ~bits);
}

static inline void
pciex_clear_cfg32(uint8_t bus, uint8_t dev, uint8_t func, uint16_t reg, uint32_t bits)
{
	pciex_wr_cfg32(bus, dev, func, reg, pciex_rd_cfg32(bus, dev, func, reg) & ~bits);
}

#undef PCIEX_CFG_ADDR

#endif /* __PIO_PCI_H_INCLUDED */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/lib/public/x86/pio_pci.h $ $Rev: 814164 $")
#endif
