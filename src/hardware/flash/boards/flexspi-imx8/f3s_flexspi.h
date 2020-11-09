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

/*
 * This file contains definitions for FLEXSPI NOR flash driver callouts.
 */

#ifndef F3S_FLEXSPI_H_
#define F3S_FLEXSPI_H_

#include <sys/f3s_mtd.h>

#include "flexspi_cmds.h"

/**
 * @file       flexspi-imx8/f3s_flexspi.h
 * @addtogroup ffs3
 * @{
 */

int32_t f3s_flexspi_open(f3s_socket_t *socket, uint32_t flags);

uint8_t *f3s_flexspi_page(f3s_socket_t *socket, uint32_t flags, uint32_t offset, int32_t *size);

int32_t f3s_flexspi_read(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags,
                         uint32_t text_offset, int32_t buffer_size, uint8_t *buffer);

int32_t f3s_flexspi_status(f3s_socket_t *socket, uint32_t flags);

void f3s_flexspi_close(f3s_socket_t *socket, uint32_t flags);

void f3s_flexspi_reset(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags, uint32_t offset);

int32_t f3s_flexspi_ident(f3s_dbase_t * dbase, f3s_access_t * access, uint32_t flags, uint32_t offset);

int32_t f3s_flexspi_write(f3s_dbase_t * dbase, f3s_access_t * access, uint32_t flags,
                          uint32_t offset, int32_t size, uint8_t * buffer);

int f3s_flexspi_erase(f3s_dbase_t * dbase, f3s_access_t * access, uint32_t flags, uint32_t offset);

int32_t f3s_flexspi_sync(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags, uint32_t text_offset);

/** @} */

#endif /* F3S_FLEXSPI_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi.h $ $Rev: 876662 $")
#endif
