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

/**
 * @file       src/hardware/devc/sermx8/main.c
 * @addtogroup devc
 * @{
 */

#include <login.h>
#include <sys/procmgr.h>
#include "externs.h"
#include "smmu.h"

static uint8_t smmu_port_count = 0;

/* Called from io-char's cleanup thread */
int dev_cleanup(TTYDEV *ttydev)
{
    DEV_UART *dev = (DEV_UART *)ttydev;

    slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Cleaning up device %s", __FUNCTION__, ttydev->name);

    if (dev->usedma) {
#if defined(USE_DMA)
        dev->sdmafuncs.xfer_abort(dev->rx_dma.dma_chn);
        dev->sdmafuncs.xfer_abort(dev->tx_dma.dma_chn);
        dev->sdmafuncs.channel_release(dev->rx_dma.dma_chn);
        dev->sdmafuncs.channel_release(dev->tx_dma.dma_chn);
        dev->sdmafuncs.fini();
        my_detach_pulse(&dev->rx_dma.pulse);
        my_detach_pulse(&dev->tx_dma.pulse);
        if (dev->usesmmu) {
            smmu_port_count--;
            smmu_obj_destroy(dev->smmu_obj);
            if ( smmu_port_count == 0 ) {
                smmu_fini();
            }
        }
#endif
    }

    return EOK;
}

int
main(int argc, char *argv[])
{
    ttyctrl.max_devs = 6;
    ttyctrl.flags |= CREATE_CLEANUP_THREAD;
    ttc(TTC_INIT_PROC, &ttyctrl, 24);

    if (options(argc, argv, &smmu_port_count) == 0) {
        fprintf(stderr, "%s: No serial ports found\n", argv[0]);
        exit(0);
    }

    /* drop root */
    if (UserParm != NULL) {
        if(procmgr_ability( 0,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_SESSION,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_KEYDATA,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_RSRCDBMGR,
                            PROCMGR_AOP_DENY  | PROCMGR_ADN_NONROOT | PROCMGR_AOP_LOCK    | PROCMGR_AID_EOL)
                             != EOK){
            perror("Unable to gain procmgr abilities for nonroot operation\n");
            exit(1);
        }
        if(set_ids_from_arg(UserParm) != EOK){
            fprintf(stderr, "%s: Unable to drop to user %s", argv[0], UserParm);
            perror("\n");
            exit(1);
        }
        free(UserParm);
    }

    ttc(TTC_INIT_START, &ttyctrl, 0);

    return 0;
}


/** @} */ /* end of devc */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/devc/sermx8/main.c $ $Rev: 886857 $")
#endif
