/*
* $QNXLicenseC:
* Copyright 2017-2018 NXP
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

#include "fsl_mu.h"

/******************************************************************************
 * Code
 *****************************************************************************/
void MU_SendMsg(MU_Type *base, uint32_t regIndex, uint32_t msg)
{
    assert(regIndex < MU_TR_COUNT);

    /* Wait TX register to be empty. */
    while (!(base->SR & (kMU_Tx0EmptyFlag >> regIndex))) {
    }

    base->TR[regIndex] = msg;
}

uint32_t MU_ReceiveMsg(MU_Type *base, uint32_t regIndex)
{
    assert(regIndex < MU_TR_COUNT);

    /* Wait RX register to be full. */
    while (!(base->SR & (kMU_Rx0FullFlag >> regIndex))) {
    }

    return base->RR[regIndex];
}

void MU_SetFlags(MU_Type *base, uint32_t flags)
{
    /* Wait for update finished. */
    while (base->SR & MU_SR_FUP_MASK) {
    }

    MU_SetFlagsNonBlocking(base, flags);
}

status_t MU_TriggerInterrupts(MU_Type *base, uint32_t mask)
{
    uint32_t reg = base->CR;

    /* Previous interrupt has been accepted. */
    if (!(reg & mask)) {
        /* All interrupts have been accepted, trigger now. */
        reg = (reg & (uint32_t)~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | mask;
        base->CR = reg;
        return kStatus_Success;
    } else {
        return kStatus_Fail;
    }
}

void MU_BootCoreB(MU_Type *base, mu_core_boot_mode_t mode)
{
    uint32_t reg = base->CR;

    reg = (reg & (uint32_t)~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | MU_CR_BRSTH_MASK | MU_CR_BBOOT_MASK)) | MU_CR_BBOOT(mode);

    base->CR = reg;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/fsl_mu.c $ $Rev: 869668 $")
#endif
