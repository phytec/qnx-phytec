/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017,2019 NXP
 * Copyright 2018 QNX Software Systems.
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


#include "callout.ah"
#include "board.h"

/*
 * -----------------------------------------------------------------------
 * Routine to patch callout code
 *
 * On entry:
 *  x0 - physical address of syspage
 *  x1 - virtual  address of syspage
 *  x2 - offset from start of syspage to start of the callout routine
 *  x3 - offset from start of syspage to read/write data used by callout
 * -----------------------------------------------------------------------
 */

patch_reboot:
    sub     sp, sp, #16
    stp     x19, x30, [sp]

    add     x19, x0, x2             /* x19 = address of callout routine */

    /*
     * Map reset control registers
     */
    mov     x0, #0x100
    adr     x1, Lpaddr
    bl      callout_io_map_indirect

    /*
     * Patch the callout routine
     */
    CALLOUT_PATCH   x19, w4, w5

    ldp     x19, x30, [sp]
    add     sp, sp, #16
    ret

    .align 3
Lpaddr: .quad IMX_MU0_BASE          /* Address of SC_IPC_AP_CH0 */


CALLOUT_START(reboot_imx, 0, patch_reboot)
    /* Get the MU base address (patched) */
    mov        x5, #0xabcd
    movk       x5, #0xabcd, lsl #16
    movk       x5, #0xabcd, lsl #32
    movk       x5, #0xabcd, lsl #48

#if defined(IMX_ARM_TRUSTED_FIRMWARE_ENABLED)
    adr     x0, psci_id_addr
    ldr     x0, [x0]
    adr     x1, psci_type_addr
    ldr     x1, [x1]
    smc     #0
0:
    b      0b
psci_id_addr:   .quad 0x00000000C4000012  /* IMX_PSCI_SYSTEM_RESET2_AARCH64 command */
psci_type_addr: .quad 0x0000000080000002  /* IMX_PSCI_RESET2_SYSTEM_BOARD_RESET type */

#else
    /* Call sc_pm_reset(ipc, SC_PM_RESET_TYPE_BOARD) function */
sc_pm_reboot_function:
1:
    ldr     w0, [x5, #0x20]         /* SR */
    tst     w0, #0xF00000           /* Is not empty some Tx buffer register so wait... */
    beq     1b

    /* Send "func, svc, size, version" to MU */
    mov     w0, #(13 << 24)         /* func */
    orr     w0, w0, #(0x02 << 16)   /* svc */
    orr     w0, w0, #(0x02 << 8)    /* size */
    orr     w0, w0, #(0x01 << 0)    /* version */
    str     w0, [x5, #0x00]         /* TR[0] */

2:
    ldr     w0, [x5, #0x20]         /* SR */
    tst     w0, #0x400000
    beq     2b

    /* Send "reset type" to MU */
    mov     w0, #0x02               /* SC_PM_RESET_TYPE_COLD = 0, SC_PM_RESET_TYPE_WARM = 1, SC_PM_RESET_TYPE_BOARD = 2 */
    str     w0, [x5, #0x04]         /* TR[1] */
0:
    b      0b
#endif

CALLOUT_END(reboot_imx)

#ifdef __QNXNTO__
#ifdef __USESRCVERSION
.section .ident, "SM",%progbits,1;
.asciz "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/callout_reboot.S $ $Rev: 904597 $";
.previous
#endif
#endif
