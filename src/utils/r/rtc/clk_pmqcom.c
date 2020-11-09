/*
 * $QNXLicenseC:
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

#include "rtc.h"
#include <sys/types.h>
#include <unistd.h>
#include <devctl.h>

#define DEV_NAME "/dev/pmic"

#define PM_RTC_DCMD 70

#define _DCMD_PMIC	_DCMD_MISC

/* RTC time structure used for setting and retrieving current time
 * and also for setting the alarm time */
typedef struct
{
    uint32_t  sec;
} pm_rtc_time_type;

/***************DCMD CONFIGURATION FOR PMIC GPIO DRIVER****************/
typedef struct
{
    unsigned char pmic_chip;
    pm_rtc_time_type set_time;
    pm_rtc_time_type alarm_expire_time;
    unsigned char alarm_enable;
} pm_rtc_master_in;

#define dcmd_pm_rtc_set_time            __DIOT (_DCMD_PMIC, PM_RTC_DCMD + 1, pm_rtc_master_in)
#define dcmd_pm_rtc_get_time            __DIOT (_DCMD_PMIC, PM_RTC_DCMD + 2, pm_rtc_master_in)
#define dcmd_pm_rtc_alarm_set           __DIOT (_DCMD_PMIC, PM_RTC_DCMD + 3, pm_rtc_master_in)
#define dcmd_pm_rtc_alarm_enable        __DIOT (_DCMD_PMIC, PM_RTC_DCMD + 4, pm_rtc_master_in)

/* Maximum number of PMIC DEVICES */
#define PM_MAX_NUM_PMICS 7

static int chip_idx = 0;

/*
 * Clock setup for the RTC on the QUALCOMM power management chips ( AARCH64le ).
 */

int
RTCFUNC(init,pmqcom)(struct chip_loc *chip, char *argv[]) {

	pm_rtc_master_in rtc_cmd = {0};
	pm_rtc_time_type rtc_time = {0};
	int retval = 0;
	int i = 0;
	int fd = -1;
	int ret = EOK;
	iov_t send[1];
	iov_t receive[2];

	do {
		/* We don't need to access the chip directly.
		 * So just set access type to NONE. */
		chip->access_type = NONE;

		fd = open(DEV_NAME, O_RDWR);
		if (fd == -1) {
			fprintf(stderr,"rtc: could not get descriptor of PMIC device, err=%d (%s)\n",
																	errno, strerror(errno));
			ret = -1;
			break;
		}

		SETIOV(send, &rtc_cmd, sizeof(pm_rtc_master_in));
		SETIOV(receive+0, &rtc_cmd, sizeof(pm_rtc_master_in));
		SETIOV(receive+1, &rtc_time, sizeof(pm_rtc_time_type));

		/* Try to find an available chip among existing ones. */
		for (i = 0; i < PM_MAX_NUM_PMICS; ++i) {
			rtc_cmd.pmic_chip = i;
			if (devctlv(fd, dcmd_pm_rtc_get_time, 1, 2, send, receive, &retval) == EOK) {
				/* The required chip has been found. */
				chip_idx = i;
				break;
			}
		}
		if (i == PM_MAX_NUM_PMICS) {
			fprintf(stderr,"rtc: could not find an appropriate PMIC chip\n");
			ret = -1;
			break;
		}
	} while(0);

	if (fd != -1) {
		close(fd);
	}

	return ret;
}

int
RTCFUNC(get,pmqcom)(struct tm *tm, int cent_reg) {

	pm_rtc_master_in rtc_cmd = {0};
	pm_rtc_time_type rtc_time  = {0};
	int rc = 0;
	int retval = 0;
	int ret = EOK;
	struct tm* tm_ret;
	int fd = -1;
	iov_t send[1];
	iov_t receive[2];

	do {
		fd = open(DEV_NAME, O_RDWR);
		if (fd == -1) {
			fprintf(stderr,"rtc: could not get descriptor of PMIC device, err=%d (%s)\n",
																	errno, strerror(errno));
			ret = -1;
			break;
		}

		SETIOV(send, &rtc_cmd, sizeof(pm_rtc_master_in));
		SETIOV(receive+0, &rtc_cmd, sizeof(pm_rtc_master_in));
		SETIOV(receive+1, &rtc_time, sizeof(pm_rtc_time_type));

		rtc_cmd.pmic_chip = chip_idx;
		rc = devctlv(fd, dcmd_pm_rtc_get_time, 1, 2, send, receive, &retval);
		if (rc != EOK) {
			fprintf(stderr,"rtc: could not get descriptor of PMIC device, err=%d (%s)\n",
																			rc, strerror(rc));
			ret = -1;
			break;
		}

		tm_ret = gmtime((time_t *)&rtc_time.sec);
		tm->tm_gmtoff = tm_ret->tm_gmtoff;
		tm->tm_hour = tm_ret->tm_hour;
		tm->tm_isdst = tm_ret->tm_isdst;
		tm->tm_mday = tm_ret->tm_mday;
		tm->tm_min = tm_ret->tm_min;
		tm->tm_mon = tm_ret->tm_mon;
		tm->tm_sec = tm_ret->tm_sec;
		tm->tm_wday = tm_ret->tm_wday;
		tm->tm_yday = tm_ret->tm_yday;
		tm->tm_year = tm_ret->tm_year;
		tm->tm_zone = tm_ret->tm_zone;

		#ifdef VERBOSE_SUPPORTED
		if (verbose) {
			printf("rtc get: returned time is %d/%d/%d %d:%d:%d\n",
					tm_ret->tm_year+1900,tm_ret->tm_mon+1,tm_ret->tm_mday,tm_ret->tm_hour,tm_ret->tm_min,tm_ret->tm_sec);
		}
		#endif

	} while(0);

	if (fd != -1) {
		close(fd);
	}

    return ret;
}

int
RTCFUNC(set,pmqcom)(struct tm *tm, int cent_reg) {

	pm_rtc_master_in rtc_msg;
	int retval = 0;
	int rc = 0;
	time_t set_time = 0;
	int fd = -1;
	int ret = EOK;

	do {
		fd = open(DEV_NAME, O_RDWR);
		if (fd == -1) {
			fprintf(stderr,"rtc: could not get descriptor of PMIC device, err=%d (%s)\n",
																	errno, strerror(errno));
			ret = -1;
			break;
		}

		set_time = mktime(tm);
		if (set_time < 0) {
			fprintf(stderr, "rtc: unable to convert the time specified\n");
			ret = -1;
			break;
		}

		rtc_msg.pmic_chip = chip_idx;
		rtc_msg.set_time.sec = set_time;
		if((rc = devctl(fd, dcmd_pm_rtc_set_time, &rtc_msg,sizeof(rtc_msg),&retval))!=EOK){
			fprintf(stderr, "rtc: unable to set the time, err=%d (%s)\n", rc, strerror(rc));
			ret = -1;
			break;
		}

		#ifdef VERBOSE_SUPPORTED
		if (verbose) {
			printf("rtc set: rtc time is set to %d/%d/%d %d:%d:%d\n",
					tm->tm_year+1900,tm->tm_mon+1,tm->tm_mday,tm->tm_hour,tm->tm_min,tm->tm_sec);
		}
		#endif
	} while(0);

	if (fd != -1) {
		close(fd);
	}

    return ret;
}

#ifdef RTCALARM
int
RTCFUNC(set_alarm,pmqcom)(uint32_t time) {

	pm_rtc_master_in rtc_msg;
	int retval = 0;
	int rc = 0;
	int fd = -1;
	int ret = EOK;

	do {
		fd = open(DEV_NAME, O_RDWR);
		if (fd == -1) {
			fprintf(stderr,"rtc: could not get descriptor of PMIC device, err=%d (%s)\n",
																	errno, strerror(errno));
			ret = -1;
			break;
		}

		rtc_msg.pmic_chip = chip_idx;
		rtc_msg.alarm_expire_time.sec = time;
		rtc_msg.alarm_enable = 0;

		/* Disable alarm if it has been enabled. */
		if((rc = devctl(fd,dcmd_pm_rtc_alarm_enable, &rtc_msg,sizeof(rtc_msg),&retval)) != EOK){
			fprintf(stderr, "rtc: unable to disable alarm, err=%d (%s)\n", rc, strerror(rc));
			ret = -1;
			break;
		}

		/* Set a time for alarm. */
		if((rc = devctl(fd,dcmd_pm_rtc_alarm_set, &rtc_msg,sizeof(rtc_msg),&retval)) != EOK){
			fprintf(stderr, "rtc: unable to set alarm, err=%d (%s)\n", rc, strerror(rc));
			ret = -1;
			break;
		}

		/* Don't enable alarm here as it will be enabled by another command. */

	} while(0);

	if (fd != -1) {
		close(fd);
	}

    return ret;
}

int
RTCFUNC(enable_alarm,pmqcom)(rtc_alarm_cmd cmd) {

	pm_rtc_master_in rtc_msg;
	int rc = 0;
	int retval = 0;
	int fd = -1;
	int ret = EOK;

	do {
		fd = open(DEV_NAME, O_RDWR);
		if (fd == -1) {
			fprintf(stderr,"rtc: could not get descriptor of PMIC device, err=%d (%s)\n",
																	errno, strerror(errno));
			ret = -1;
			break;
		}

		rtc_msg.pmic_chip = chip_idx;
		if (cmd == RTC_ALARM_ON) {
			rtc_msg.alarm_enable = 1;
		} else {
			rtc_msg.alarm_enable = 0;
		}

		/* Enable alarm. */
		if((rc = devctl(fd,dcmd_pm_rtc_alarm_enable, &rtc_msg,sizeof(rtc_msg),&retval)) != EOK){
			fprintf(stderr, "rtc: unable to enable alarm, err=%d (%s)\n", rc, strerror(rc));
			ret = -1;
			break;
		}
	} while(0);

	if (fd != -1) {
		close(fd);
	}

    return ret;
}
#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/utils/r/rtc/clk_pmqcom.c $ $Rev: 886297 $")
#endif
