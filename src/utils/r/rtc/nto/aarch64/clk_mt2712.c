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

#define MT2712_RTC_BASE 0x10011000
#define MT2712_RTC_SIZE (0x80)

#define MT2712_RTC_BBPU    (0x00)
#define     MT2712_RTC_BBPU_KEY_MASK    (0xff<<8)
#define     MT2712_RTC_BBPU_KEY_BBPU    (0x43<<8)
#define     MT2712_RTC_BBPU_CBUSY       (1<<6)
#define     MT2712_RTC_BBPU_RELOAD      (1<<5)
#define     MT2712_RTC_BBPU_CLRPKY      (1<<4)
#define MT2712_RTC_IRQ_STA (0x04)
#define MT2712_RTC_IRQ_EN  (0x08)
#define MT2712_RTC_CII_EN  (0x0c)
#define MT2712_RTC_AL_MASK (0x10)
#define MT2712_RTC_TC_SEC  (0x14) // 00-59
#define MT2712_RTC_TC_MIN  (0x18) // 00-59
#define MT2712_RTC_TC_HOU  (0x1c) // 00-23
#define MT2712_RTC_TC_DOM  (0x20) // 01-31 (Note: Feb 01-29 if (year%4)==0)
#define MT2712_RTC_TC_DOW  (0x24) // 01-07
#define MT2712_RTC_TC_MTH  (0x28) // 01-12
#define MT2712_RTC_TC_YEA  (0x2c) // 000-127 (2000-2127)
#define	    MT2712_RTC_YEA_BIAS  68  // this matches Linux year offset, giving supported range 1968-2095
#define MT2712_RTC_AL_SEC  (0x30)
#define MT2712_RTC_AL_MIN  (0x34)
#define MT2712_RTC_AL_HOU  (0x38)
#define MT2712_RTC_AL_DOM  (0x3c)
#define MT2712_RTC_AL_DOW  (0x40)
#define MT2712_RTC_AL_MTH  (0x44)
#define MT2712_RTC_AL_YEA  (0x48)
#define MT2712_RTC_POWERKEY1   0x4c
#define MT2712_RTC_POWERKEY2   0x50
#define MT2712_RTC_DIFF    (0x58)
#define MT2712_RTC_CON0    (0x5c)
#define MT2712_RTC_CON1    (0x60)
#define MT2712_RTC_PDN1    (0x64)
#define MT2712_RTC_PDN2    (0x68)
#define MT2712_RTC_SPAR1   (0x6c)
#define MT2712_RTC_PROT    (0x70)
#define     MT2712_RTC_PROT_UNLOCK1 0x586a
#define     MT2712_RTC_PROT_UNLOCK2 0x9136
#define MT2712_RTC_WRTGR   (0x78)
#define     MT2712_RTC_WRTGR_WRTGR  (1<<0)


/*
 * Clock setup for the RTC on the Mediatek MT2712 (ARMv8 Cortex-A35/A72 cores).
 */


int
RTCFUNC(init,mt2712)(struct chip_loc *chip, char *argv[]) {

    if (chip->phys == NIL_PADDR) {
        chip->phys = MT2712_RTC_BASE;
    }

    if (chip->access_type == NONE) {
        chip->access_type = MEMMAPPED;
    }

    return MT2712_RTC_SIZE;
}

int
RTCFUNC(get,mt2712)(struct tm *tm, int cent_reg) {

    do {
        tm->tm_sec  = chip_read(MT2712_RTC_TC_SEC, 32);
        tm->tm_min  = chip_read(MT2712_RTC_TC_MIN, 32);
        tm->tm_hour = chip_read(MT2712_RTC_TC_HOU, 32);           // 00-23 (24hr time)
        tm->tm_mday = chip_read(MT2712_RTC_TC_DOM, 32);           // 01-31
        tm->tm_mon  = (int)chip_read(MT2712_RTC_TC_MTH, 32) - 1;  // 01-12 -> 00-11
        tm->tm_year = chip_read(MT2712_RTC_TC_YEA, 32) + MT2712_RTC_YEA_BIAS; // 000-127 -> 1968-2095
    } while(chip_read(MT2712_RTC_TC_SEC, 32) < tm->tm_sec); // get stable value

    /*
     * Validate rtc hw time registers returned
     */
    if(tm->tm_mon < 0 || tm->tm_mon > 11) {
        printf("%s: invalid rtc mon value %d, ignoring rtc\n", __FUNCTION__, tm->tm_mon+1);
        return -1;
    }
    if(tm->tm_mday < 1 || tm->tm_mday > 31) {
        printf("%s: invalid rtc mday value %d, ignoring rtc\n", __FUNCTION__, tm->tm_mday);
        return -1;
    }
    if(tm->tm_hour < 0 || tm->tm_hour > 23) {
        printf("%s: invalid rtc hour value %d, ignoring rtc\n", __FUNCTION__, tm->tm_hour);
        return -1;
    }
    if(tm->tm_min < 0 || tm->tm_min > 59) {
        printf("%s: invalid rtc min value %d, ignoring rtc\n", __FUNCTION__, tm->tm_min);
        return -1;
    }
    if(tm->tm_sec < 0 || tm->tm_sec > 59) {
        printf("%s: invalid rtc sec value %d, ignoring rtc\n", __FUNCTION__, tm->tm_sec);
        return -1;
    }

    return(0);
}

int
RTCFUNC(set,mt2712)(struct tm *tm, int cent_reg) {

#if 0
    // unlock the RTC registers
    chip_write(MT2712_RTC_PROT, MT2712_RTC_PROT_UNLOCK1, 32);
    chip_write(MT2712_RTC_WRTGR, MT2712_RTC_WRTGR_WRTGR, 32);
    while (chip_read(MT2712_RTC_BBPU, 32) & MT2712_RTC_BBPU_CBUSY) ;

    chip_write(MT2712_RTC_PROT, MT2712_RTC_PROT_UNLOCK2, 32);
    chip_write(MT2712_RTC_WRTGR, MT2712_RTC_WRTGR_WRTGR, 32);
    while (chip_read(MT2712_RTC_BBPU, 32) & MT2712_RTC_BBPU_CBUSY) ;
#endif

    chip_write(MT2712_RTC_TC_SEC, tm->tm_sec,      32);
    chip_write(MT2712_RTC_TC_MIN, tm->tm_min,      32);
    chip_write(MT2712_RTC_TC_HOU, tm->tm_hour,     32);     // 00-23 (24hr time)
    chip_write(MT2712_RTC_TC_DOW, tm->tm_wday + 1, 32);     // 1-7
    chip_write(MT2712_RTC_TC_DOM, tm->tm_mday,     32);     // 01-31 (day of month)
    chip_write(MT2712_RTC_TC_MTH, tm->tm_mon + 1,  32);     // 01-12
    chip_write(MT2712_RTC_TC_YEA, tm->tm_year - MT2712_RTC_YEA_BIAS, 32); // 000-127 (1968-2095)

    // Write register changes to RTC domain and wait for completion
    chip_write(MT2712_RTC_WRTGR, MT2712_RTC_WRTGR_WRTGR, 32);
    while (chip_read(MT2712_RTC_BBPU, 32) & MT2712_RTC_BBPU_CBUSY) ;

    return(0);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/utils/r/rtc/nto/aarch64/clk_mt2712.c $ $Rev: 886297 $")
#endif

