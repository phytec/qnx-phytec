/*
 * $QNXLicenseC:
 * Copyright 2016 QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/resmgr.h>
#include <sys/neutrino.h>
#include <hw/inout.h>
#include <sys/slog.h>
#include <sys/slogcodes.h>
#include <errno.h>
#include <login.h>
#include <sys/procmgr.h>
#include <drvr/hwinfo.h>
#include <sys/imx8_sci_mgr.h>
#include <fcntl.h>
#include <pthread.h>
#include <aarch64/mx8x_smc_call.h>

/**
 * @file        src/hardware/support/wdtkick-sc-imx8/main.c
 * @addtogroup  wdtkick
 * @{
 */


/* Global variable representing file descriptor needed for communication with SC firmware */
int sci_fd;

/* For drop-root */
char *user_parm;

static char *wdog_hwi_opts[] = {
    "smc_call",       /* WDOG SMC call support */
    NULL
};

/**
 * Main function of the Watchdog refresh utility.
 *
 * @param argc  Count of the command line arguments.
 * @param argv  Array of pointers to command line arguments.
 *
 * @return  Execution status.
 * @retval  EXIT_SUCCESS    If everything is OK.
 * @retval  EXIT_FAILURE    In case of any error.
 */
int main(int argc, char *argv[])
{
    int             opt;
    unsigned        time     = 5000;    /* Default time 5000ms for WDOG timer kick */
    unsigned        priority = 10;      /* Default priority of the WDOG driver */
    /* Variables used for communication with SCU driver */
    sci_mgr_err_t   status;
    int             cnt;
    int             ret;
    char            *optstr, *freeptr, *c, *value;
    imx_smc_status_t smc_status;
    bool            smc_call_en = false;
    hwiattr_timer_t wdog_attr;
    unsigned        hwi_off;

    /* Getting the WDOG Base addresss from the Hwinfo Section if available */
    hwi_off = hwi_find_device("wdog", 0);
    if (hwi_off != HWI_NULL_OFF) {
        ret = hwiattr_get_timer(hwi_off, &wdog_attr);
        if (ret == EOK) {
            if (wdog_attr.common.optstr != NULL) {
                freeptr = optstr = strdup(wdog_attr.common.optstr);
                while ((optstr != NULL) && (*optstr != '\0')) {
                    c = optstr;
                    if ((opt = getsubopt(&optstr, wdog_hwi_opts, &value)) == -1) {
                        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "Unknown option %s", c);
                        continue;
                    }
                    switch (opt) {
                        case 0: /* smc_call */
                            if (strcmp(value, "yes") == 0) {
                                smc_call_en = true;
                            }
                        continue;
                    }
                }
                free(freeptr);
            }
        }
    }

    /* Process dash options */
    user_parm = NULL;
    while ((opt = getopt(argc, argv, "p:t:U:")) != -1) {
        switch (opt) {
            case 'p': /* Priority of the WDOG driver */
                priority = strtoul(optarg, NULL, 0);
                break;
            case 't': /* Time for WDOG timer kick */
                time = strtoul(optarg, NULL, 0);
                break;
            case 'U': /* drop-root */
                user_parm = strdup(optarg);
                break;
        }
    }
    slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO,
          "wdtkick info: smc_call_en=%d time=%dms, priority=%d", smc_call_en, time, priority);

    if (smc_call_en) {
        /* Enable IO capability */
        if (ThreadCtl(_NTO_TCTL_IO_PRIV, NULL) == -1) {
            slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "wdtkick error:  ThreadCtl.");
            return EXIT_FAILURE;
        }
    } else {
        /* Enable IO capability */
        if (ThreadCtl(_NTO_TCTL_IO, NULL) == -1) {
            slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "wdtkick error:  ThreadCtl.");
            return EXIT_FAILURE;
        }
    }

    /* Run in the background */
    if (procmgr_daemon(EXIT_SUCCESS, PROCMGR_DAEMON_NOCLOSE | PROCMGR_DAEMON_NODEVNULL) == -1) {
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "%s:  procmgr_daemon.", argv[0]);
        return EXIT_FAILURE;
    }

    /* Set priority of the WDOG refresh thread */
    if (pthread_setschedprio(pthread_self(), priority)) {
        slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "wdtkick error:  pthread_setschedprio.");
        return EXIT_FAILURE;
    }

    /* Drop root */
    if (user_parm != NULL) {
        if(procmgr_ability( 0,
                            PROCMGR_AOP_DENY | PROCMGR_ADN_NONROOT | PROCMGR_AOP_LOCK | PROCMGR_AID_EOL)
                             != EOK){
            slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
                "%s: Unable to gain procmgr abilities for nonroot operation.", argv[0]);
            return EXIT_FAILURE;
        }
        if(set_ids_from_arg(user_parm) != EOK){
            slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
                "%s: Unable to drop to user %s: %s\n", argv[0], user_parm, strerror(errno));
            return EXIT_FAILURE;
        }
        free(user_parm);
    }

    while (1) {
        if (smc_call_en) {
            /* Refresh WDOG timer */
            cnt = 0;
            do {
                smc_status = imx_sec_firmware_psci(IMX_FSL_SIP_SRTC, IMX_FSL_SIP_SRTC_PING_WDOG, 0x00, 0x00, 0x00);
            } while ((smc_status != IMX_PSCI_SUCCESS) && (cnt++ < 10));
            if (cnt >= 10) {
                perror("wdtkick error: Failed to refresh WDOG timer");
                return EXIT_FAILURE;
            }
        } else {
            /* Open SC driver */
            sci_fd = open("/dev/sc", O_RDWR);
            if (sci_fd >= 0) {
                /* Refresh WDOG timer */
                cnt = 0;
                do {
                    ret = devctl(sci_fd, IMX_DCMD_SC_TIMER_PING_WDOG, NULL, 0, (int *) &status);
                } while ((ret == EAGAIN) && (cnt++ < 10) && (delay(10) == 0));
                if (cnt >= 10) {
                    close(sci_fd);
                    perror("wdtkick error: Failed to refresh WDOG timer");
                    return EXIT_FAILURE;
                }
                /* Close SC driver */
                close(sci_fd);
            } else {
                perror("wdtkick error: Failed to open sc device");
                return EXIT_FAILURE;
            }
        }

        /* Suspend thread for given length of time */
        delay(time);
    }

    return EXIT_SUCCESS;
}

/** @} */ /* End of wdtkick-sc-imx */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/support/wdtkick-sc/imx8/main.c $ $Rev: 887005 $")
#endif
