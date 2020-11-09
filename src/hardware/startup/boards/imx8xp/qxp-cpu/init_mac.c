/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
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

#include "startup.h"
#include <hw/nxp/imx8/sci/sci.h>
#include <aarch64/mx8x.h>

#define IMX_ENET_MAC_INIT_DEBUG 0U

/* OTP fuses */
#define IMX_SC_OCOTP_MAC0 708
#define IMX_SC_OCOTP_MAC1 709
#define IMX_SC_OCOTP_MAC2 710
#define IMX_SC_OCOTP_MAC3 711

/* Offsets of the ENETs MAC address registers */
#define IMX_ENET_PALR 0xE4
#define IMX_ENET_PAUR 0xE8

/**
 * Function reads Ethernet MAC addresses from OCOTP fuses. Obtained addresses are then
 * written into ENETs peripherals.
 *
 * @param ipc       IPC handle.
 */
void imx_init_enet_mac_addr(sc_ipc_t ipc)
{
    sc_err_t      sc_status;
    unsigned int  mac_low, mac_high;
    unsigned char enet0_mac_addr[6];
    unsigned char enet1_mac_addr[6];
    unsigned int  fuse_MAC0 = 0U;
    unsigned int  fuse_MAC1 = 0U;
    unsigned int  fuse_MAC2 = 0U;
    unsigned int  fuse_MAC3 = 0U;
    uint64_t      cntvct;

    /* Read OTP fuses using SCFW API */
    sc_status = sc_misc_otp_fuse_read(ipc, IMX_SC_OCOTP_MAC0, &fuse_MAC0);
    if (sc_status != SC_ERR_NONE) {
        kprintf("%s: Can't load MAC0 otp fuse. SC status: %s\n", __FUNCTION__, sc_status2str(sc_status));
    }

    sc_status = sc_misc_otp_fuse_read(ipc, IMX_SC_OCOTP_MAC1, &fuse_MAC1);
    if (sc_status != SC_ERR_NONE) {
        kprintf("%s: Can't load MAC1 otp fuse. SC status: %s\n", __FUNCTION__, sc_status2str(sc_status));
    }

    sc_status = sc_misc_otp_fuse_read(ipc, IMX_SC_OCOTP_MAC2, &fuse_MAC2);
    if (sc_status != SC_ERR_NONE) {
        kprintf("%s: Can't load MAC2 otp fuse. SC status: %s\n", __FUNCTION__, sc_status2str(sc_status));
    }
    sc_status = sc_misc_otp_fuse_read(ipc, IMX_SC_OCOTP_MAC3, &fuse_MAC3);
    if (sc_status != SC_ERR_NONE) {
        kprintf("%s: Can't load MAC3 otp fuse. SC status: %s\n", __FUNCTION__, sc_status2str(sc_status));
    }

#if IMX_ENET_MAC_INIT_DEBUG
    kprintf("Fuse MAC0: 0x%x\n", fuse_MAC0);
    kprintf("Fuse MAC1: 0x%x\n", fuse_MAC1);
    kprintf("Fuse MAC2: 0x%x\n", fuse_MAC2);
    kprintf("Fuse MAC3: 0x%x\n", fuse_MAC3);
#endif

    /*
     * Workaround for boards which do have not MAC address burned in fuses. For such boards will be used random MAC.
     * Value of the random MAC is derived from actual counter value of the system ARMv8 timer.
     */
    if (!fuse_MAC0) {
        kprintf("MAC address for ENET0 is not programmed in Fuses. Random MAC will be used.\n", fuse_MAC0);
        __asm__ __volatile__("mrs    %0, CNTVCT_EL0" : "=r"(cntvct));
        fuse_MAC0 = 0x009F0400 | ((cntvct & 0xFF0000) << 8U);
        fuse_MAC1 = 0x00AA0004 | (cntvct & 0xFFFF);
    }

    if (!fuse_MAC2) {
        kprintf("MAC address for ENET1 is not programmed in Fuses. Random MAC will be used.\n", fuse_MAC0);
        __asm__ __volatile__("mrs    %0, CNTVCT_EL0" : "=r"(cntvct));
        fuse_MAC2 = 0x009F0400 | ((cntvct & 0xFF0000) << 8U);
        fuse_MAC3 = 0x00AA0004 | (cntvct & 0xFFFF);
    }

    /*
     * ENET0 MAC Address settings
     */
    enet0_mac_addr[0] = fuse_MAC0 & 0xFF;
    enet0_mac_addr[1] = (fuse_MAC0 >> 8) & 0xFF;
    enet0_mac_addr[2] = (fuse_MAC0 >> 16) & 0xFF;
    enet0_mac_addr[3] = (fuse_MAC0 >> 24) & 0xFF;

    enet0_mac_addr[4] = fuse_MAC1 & 0xFF;
    enet0_mac_addr[5] = (fuse_MAC1 >> 8) & 0xFF;

    /* Set MAC address into ENET0 registers  */
    mac_low = ((enet0_mac_addr[0] << 24) + (enet0_mac_addr[1] << 16) + (enet0_mac_addr[2] << 8) + enet0_mac_addr[3]);
    out32(IMX_ENET0_BASE + IMX_ENET_PALR, mac_low);

    mac_high = ((enet0_mac_addr[4] << 24) + (enet0_mac_addr[5] << 16));
    out32(IMX_ENET0_BASE + IMX_ENET_PAUR, mac_high);

#if IMX_ENET_MAC_INIT_DEBUG
    kprintf("ENET0 MAC: %b:%b:%b:%b:%b:%b\n", enet0_mac_addr[0], enet0_mac_addr[1], enet0_mac_addr[2],
                                              enet0_mac_addr[3], enet0_mac_addr[4], enet0_mac_addr[5]);
#endif
    /*
     * ENET1 MAC Address settings
     */
    enet1_mac_addr[0] = fuse_MAC2 & 0xFF;
    enet1_mac_addr[1] = (fuse_MAC2 >> 8) & 0xFF;
    enet1_mac_addr[2] = (fuse_MAC2 >> 16) & 0xFF;
    enet1_mac_addr[3] = (fuse_MAC2 >> 24) & 0xFF;

    enet1_mac_addr[4] = fuse_MAC3 & 0xFF;
    enet1_mac_addr[5] = (fuse_MAC3 >> 8) & 0xFF;

    /* Set MAC address into related ENET1 registers  */
    mac_low = ((enet1_mac_addr[0] << 24) + (enet1_mac_addr[1] << 16) + (enet1_mac_addr[2] << 8) + enet1_mac_addr[3]);
    out32(IMX_ENET1_BASE + IMX_ENET_PALR, mac_low);

    mac_high = ((enet1_mac_addr[4] << 24) + (enet1_mac_addr[5] << 16));
    out32(IMX_ENET1_BASE + IMX_ENET_PAUR, mac_high);

#if IMX_ENET_MAC_INIT_DEBUG
    kprintf("ENET1 MAC: %b:%b:%b:%b:%b:%b\n", enet1_mac_addr[0], enet1_mac_addr[1], enet1_mac_addr[2],
                                              enet1_mac_addr[3], enet1_mac_addr[4], enet1_mac_addr[5]);
#endif
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/startup/boards/imx8xp/qxp-cpu/init_mac.c $ $Rev: 869223 $")
#endif
