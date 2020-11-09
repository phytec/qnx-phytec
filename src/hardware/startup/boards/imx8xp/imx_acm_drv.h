/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * Copyright 2018, QNX Software Systems.
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
#ifndef IMX_ACM_DRV_H_
#define IMX_ACM_DRV_H_

#include <stdint.h>
#include <hw/nxp/imx8/sci/sci_types.h>
#include <hw/nxp/imx8/sci/svc/pm/api.h>
#include <aarch64/imx8xp/imx_acm.h>
#include "imx_startup.h"

void imx_acm_set_aud_mux(unsigned index, imx_acm_aud_clk_mux_t val);
void imx_acm_set_mclk_mux(unsigned index, imx_acm_mclk_mux_t val);
void imx_acm_set_esai_mux(unsigned index, imx_acm_esai_mux_t val);
void imx_acm_set_gpt_mux(unsigned index, imx_acm_gpt_mux_t val);
void imx_acm_set_gpt_event(unsigned index, uint32_t val);
void imx_acm_set_sai_mux(unsigned index, imx_acm_sai_mux_t val);
void imx_acm_set_spdif_mux(unsigned index, imx_acm_spdif_mux_t val);
void imx_acm_set_mqs_mux(unsigned index, imx_acm_mqs_mux_t val);

uint32_t imx_acm_get_aud_mux(unsigned index);
uint32_t imx_acm_get_mclk_mux(unsigned index);
uint32_t imx_acm_get_esai_mux(unsigned index);
uint32_t imx_acm_get_gpt_mux(unsigned index);
uint32_t imx_acm_get_gpt_event(unsigned index);
uint32_t imx_acm_get_sai_mux(unsigned index);
uint32_t imx_acm_get_spdif_mux(unsigned index);
uint32_t imx_acm_get_mqs_mux(unsigned index);

#endif /* __IMX_ACM_DRV_H */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/startup/boards/imx8xp/imx_acm_drv.h $ $Rev: 858078 $")
#endif
