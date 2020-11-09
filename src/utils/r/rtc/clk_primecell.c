/*
 * $QNXLicenseC: 
 * Copyright 2007, 2008, QNX Software Systems.  
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
#include <time.h>

/* PL030 registers */
#include <arm/primecell.h>
#define PL030_ID	0x41030

/* PL031 registers */
#define PL031_RTCDR	0x000
#define PL031_RTCMR	0x004
#define PL031_RTCLR	0x008
#define PL031_RTCCR	0x00c
#define PL031_RTCIMSC	0x010
#define PL031_RTCRIS	0x014
#define PL031_RTCMIS	0x018
#define PL031_RTCICR	0x01c
#define PL031_RTCPID0	0xfe0
#define PL031_RTCPID1	0xfe4
#define PL031_RTCPID2	0xfe8
#define PL031_RTCPID3	0xfec
#define PL031_RTCCID0	0xff0
#define PL031_RTCCID1	0xff4
#define PL031_RTCCID2	0xff8
#define PL031_RTCCID3	0xffc

#define PL031_ID	0x41031

#define RTC_SIZE	0x1000

static unsigned rd_reg = 0;
static unsigned wr_reg = 0;

int
RTCFUNC(init,primecell)(struct chip_loc *chip, char *argv[])
{
	if (chip->phys == NIL_PADDR) {
		fprintf(stderr,"rtc: -b baseaddr must be specified for primecell clock type\n");
		return(-1);
	}
	if (chip->access_type == NONE) {
		chip->access_type = MEMMAPPED;
	}
	return RTC_SIZE;
}

void
primecell_detect(void)
{
	unsigned id;

	/* ID registers are in a common place */
	id = chip_read(PL031_RTCPID0, 8) & 0xff;
	id = id | ((chip_read(PL031_RTCPID1, 8) & 0xff) << 8);
	id = id | ((chip_read(PL031_RTCPID2, 8) & 0x0f) << 16);

#ifdef	VERBOSE_SUPPORTED
	if (verbose) {
		printf("rtc chipid: 0x%x\n", id);
	}
#endif

	switch (id) {
	case 0x41030:
#ifdef	VERBOSE_SUPPORTED
		if (verbose) {
			printf("rtc PrimeCell PL030 detected\n");
		}
#endif
		rd_reg = PRIMECELL_RTC_DR;
		wr_reg = PRIMECELL_RTC_LR;
		break;

	case 0x41031:
#ifdef	VERBOSE_SUPPORTED
		if (verbose) {
			printf("rtc PrimeCell PL031 detected\n");
		}
#endif
		rd_reg = PL031_RTCDR;
		wr_reg = PL031_RTCLR;
		break;

	default:
#ifdef	VERBOSE_SUPPORTED
		if (verbose) {
			printf("rtc Unknown PrimeCell RTC detected, defaulting to pl030\n");
		}
#endif
		rd_reg = PRIMECELL_RTC_DR;
		wr_reg = PRIMECELL_RTC_LR;
		break;
	}
}

int
RTCFUNC(get,primecell)(struct tm *tm, int cent_reg)
{
	time_t		t;

	primecell_detect();

	/*
	 * read RTC counter value
	 */
	t = chip_read(rd_reg, 32);

#ifdef	VERBOSE_SUPPORTED
	if (verbose) {
		printf("rtc read: %ld\n", (long)t);
	}
#endif
	
	gmtime_r(&t,tm);	
	
	return 0;
}

int
RTCFUNC(set,primecell)(struct tm *tm, int cent_reg)
{
	time_t		t;
	
	t = mktime(tm);

	/*
	 *	mktime assumes local time.  We will subtract timezone
	 */
	t -= timezone;

	primecell_detect();
#ifdef	VERBOSE_SUPPORTED
	if (verbose) {
		printf("rtc write: %ld\n", (long)t);
	}
#endif

	chip_write(wr_reg, t, 32);

	return 0;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/utils/r/rtc/clk_primecell.c $ $Rev: 829471 $")
#endif
