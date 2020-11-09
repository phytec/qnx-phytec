/*
 * $QNXLicenseC:
 * Copyright 2020, QNX Software Systems.
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

#ifndef _F3S_FLEXSPI_S26HS_H
#define _F3S_FLEXSPI_S26HS_H

#include <aarch64/inout.h>
#include <pthread.h>
#include <stdint.h>
#include <sys/f3s_mtd.h>
#include <imx_fc_flexspi.h>

#include <aarch64/imx8_common/imx_flexspi.h>

/**
 * @file       flexspi-imx8/f3s_flexspi_s26hs.h
 * @addtogroup ffs3_fc Flash Controller for semperflash
 * @{
 */

int f3s_flexspi_ident_s26hs(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags, uint32_t offset);
int32_t f3s_flexspi_read_s26hs(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags,
                               uint32_t text_offset, int32_t buffer_size, uint8_t *buffer);
int32_t f3s_flexspi_write_s26hs(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags,
                                uint32_t offset, int32_t size, uint8_t *buffer);
int32_t f3s_flexspi_sync_s26hs(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags, uint32_t text_offset);
int f3s_flexspi_erase_s26hs(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags, uint32_t offset);


/** @} */

#endif /* _F3S_FLEXSPI_S26HS_H */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_s26hs.h $ $Rev: 909225 $")
#endif
