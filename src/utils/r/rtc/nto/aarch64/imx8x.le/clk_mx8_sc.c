/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
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


#include "rtc.h"
#include <sys/imx8_sci_mgr.h>
#include <aarch64/mx8x_smc_call.h>
#include <drvr/hwinfo.h>                /* For hwi support routines in libdrvr */


/*
 * Clock setup for i.MX8 platform that use system controller.
 */
#define MX8_SC_DEVNAME		"/dev/sc"
#define MX8_MAX_NAME_LENGTH	128

static char *rtc_hwi_opts[] = {
    "smc_call",       /* RTC SMC call support */
    NULL
};

static char	sc_name[MX8_MAX_NAME_LENGTH];
static bool	smc_call_en = false;

int
RTCFUNC(init,mx8sc)(struct chip_loc *chip, char *argv[]) {
	char		*optstr, *freeptr, *c, *value;
	unsigned	hwi_off;
	unsigned	tag_idx;
	unsigned	unit;
	hwi_tag		*tag;
	int			opt;

	if (argv && argv[0] && argv[0][0]) {
		strlcpy(sc_name, argv[0], MX8_MAX_NAME_LENGTH);
	}
	else {
		strlcpy(sc_name, MX8_SC_DEVNAME, MX8_MAX_NAME_LENGTH);
	}

	/* Getting the SMC call status from the Hwinfo Section if available */
	unit = 0;
	while((hwi_off = hwi_find_device("smc_call", unit)) != HWI_NULL_OFF) {
		tag_idx = 0; //We don't need the device name in this case
		if((tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_optstr, &tag_idx)) != NULL) {
			freeptr = optstr = strdup(__hwi_find_string(tag->optstr.string));
			while ((optstr != NULL) && (*optstr != '\0')) {
				c = optstr;
				if ((opt = getsubopt(&optstr, rtc_hwi_opts, &value)) == -1) {
					fprintf(stderr, "%s - unknown option: %s\n", __FUNCTION__, c);
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
		unit++;
	}

	return (0);
}

int
RTCFUNC(get,mx8sc)(struct tm *tm, int cent_reg) {
	int							sci_fd;
	int							cnt;
	int							ret;
	imx_dcmd_sc_timer_time_t	rtc;

	/* Open SC driver */
	sci_fd = open(sc_name, O_RDWR);
	if (sci_fd < 0) {
		perror("RTC: Failed to open SC device @ /dev/sc");
		return (0);
	}

	/* get the RTC */
	cnt = 0;
	do {
		ret = devctl(sci_fd, IMX_DCMD_SC_TIMER_GET_RTC_TIME, &rtc, sizeof(rtc), NULL);
	} while ((ret == EAGAIN) && (cnt++ < 10) && (delay(10) == 0));

	if (cnt >= 10) {
		perror("RTC: Failed to get RTC time");
	}

	/* Close SC driver */
	close(sci_fd);

	tm->tm_sec  = rtc.sec;
	tm->tm_min  = rtc.min;
	tm->tm_hour = rtc.hour;
	tm->tm_mday = rtc.day;
	tm->tm_mon  = rtc.mon - 1;
	tm->tm_year = rtc.year - 1900;

	return(0);
}

int
RTCFUNC(set,mx8sc)(struct tm *tm, int cent_reg) {
	int							sci_fd;
	int							cnt;
	int							ret;
	imx_dcmd_sc_timer_time_t	rtc;

	rtc.sec  = tm->tm_sec;
	rtc.min  = tm->tm_min;
	rtc.hour = tm->tm_hour;
	rtc.day  = tm->tm_mday;
	rtc.mon  = tm->tm_mon + 1;
	rtc.year = tm->tm_year + 1900;

	if(smc_call_en) {
		uint64_t			year_mon, day_hour, min_sec;
		imx_smc_status_t	smc_status;
		year_mon =  ((rtc.year << IMX_FSL_SIP_SRTC_YEAR_SHIFT) & IMX_FSL_SIP_SRTC_YEAR_MASK) |
					((rtc.mon  << IMX_FSL_SIP_SRTC_MON_SHIFT)  & IMX_FSL_SIP_SRTC_MON_MASK);
		day_hour =  ((rtc.day  << IMX_FSL_SIP_SRTC_DAY_SHIFT)  & IMX_FSL_SIP_SRTC_DAY_MASK)  |
					((rtc.hour << IMX_FSL_SIP_SRTC_HOUR_SHIFT) & IMX_FSL_SIP_SRTC_HOUR_MASK);
		min_sec  =  ((rtc.min  << IMX_FSL_SIP_SRTC_MIN_SHIFT)  & IMX_FSL_SIP_SRTC_MIN_MASK)  |
					((rtc.sec  << IMX_FSL_SIP_SRTC_SEC_SHIFT)  & IMX_FSL_SIP_SRTC_SEC_MASK);

		/* get proper I/O privledges for smc call */
		if (ThreadCtl(_NTO_TCTL_IO_PRIV, NULL) == -1) {
			perror("Can not obtain I/O privledges");
			return (0);
		}

		/* set RTC time by smc call */
		smc_status = imx_sec_firmware_psci(IMX_FSL_SIP_SRTC, IMX_FSL_SIP_SRTC_SET_TIME, year_mon, day_hour, min_sec);
		if (smc_status != IMX_PSCI_SUCCESS) {
			perror("RTC: Failed to set RTC time");
			return (0);
		}
	}
	else {
		/* Open SC driver */
		sci_fd = open(sc_name, O_RDWR);
		if (sci_fd < 0) {
			perror("RTC: Failed to open SC device @ /dev/sc");
			return (0);
		}

		/* set the RTC */
		cnt = 0;
		do {
			ret = devctl(sci_fd, IMX_DCMD_SC_TIMER_SET_RTC_TIME, &rtc, sizeof(rtc), NULL);
		} while ((ret == EAGAIN) && (cnt++ < 10) && (delay(10) == 0));

		if (cnt >= 10) {
			perror("RTC: Failed to set RTC time");
		}

		/* Close SC driver */
		close(sci_fd);
	}

	return (0);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/utils/r/rtc/nto/aarch64/imx8x.le/clk_mx8_sc.c $ $Rev: 862449 $")
#endif
