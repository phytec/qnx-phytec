/*
 * Copyright (c) 2019, QNX Software Systems.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef QXP_CPU_H
#define QXP_CPU_H

#include <aarch64/mx8xp.h>
#include "imx_startup.h"

/* HW info */
void imx_init_hwinfo(imx_startup_data_t * startup_data);

/* Ethernet */
void imx_init_enet_mac_addr(sc_ipc_t ipc);
int imx_init_enet1_pads(imx_startup_data_t * startup_data);

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
#ifdef __ASM__
__SRCVERSION "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/qxp-cpu/qxp_cpu.h $ $Rev: 891625 $"
#else
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/qxp-cpu/qxp_cpu.h $ $Rev: 891625 $")
#endif
#endif
