/*
 * $QNXLicenseC:
 * Copyright 2020, PHYTEC America
 * Copyright 2008, QNX Software Systems.
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
#include <fcntl.h>
#include <hw/i2c.h>

/*
 * RV3028 RTC with I2C Bus Access
 */
#define RV3028_SEC              0       /* 00-59 */
#define RV3028_MIN              1       /* 00-59 */
#define RV3028_HOUR             2       /* 0-1/00-23 */
#define RV3028_DAY              3       /* 01-07 */
#define RV3028_DATE             4       /* 01-31 */
#define RV3028_MONTH            5       /* 01-12 */
#define RV3028_YEAR             6       /* 00-99 */
#define RV3028_BACKUP           0x37

#define RV3028_SEC_MASK         0x7F    /* Seconds bits mask */
#define RV3028_MIN_MASK         0x7F    /* Minutes bits mask */
#define RV3028_HOUR_MASK        0x3F    /* Hour bits mask */
#define RV3028_HOUR_MASK        0x3F    /* Hour bits mask */
#define RV3028_DAY_MASK         0x07    /* Day of week bits mask */
#define RV3028_DATE_MASK        0x3F    /* Day of month bits mask */
#define RV3028_MONTH_MASK       0x1F    /* Month bits mask */

#define RV3028_I2C_ADDRESS      (0xA4 >> 1)
#define RV3028_I2C_DEVNAME      "/dev/i2c1"


static int fd = -1;

static int
rv3028_i2c_read(unsigned char reg, unsigned char val[], unsigned char num)
{
    iov_t           siov[2], riov[2];
    i2c_sendrecv_t  hdr;

    hdr.slave.addr = RV3028_I2C_ADDRESS;
    hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    hdr.send_len = 1;
    hdr.recv_len = num;
    hdr.stop = 1;

    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&siov[1], &reg, sizeof(reg));

    SETIOV(&riov[0], &hdr, sizeof(hdr));
    SETIOV(&riov[1], val, num);

    return devctlv(fd, DCMD_I2C_SENDRECV, 2, 2, siov, riov, NULL);
}

static int
rv3028_i2c_write(unsigned char reg, unsigned char val[], unsigned char num)
{
    iov_t           siov[3];
    i2c_send_t      hdr;

    hdr.slave.addr = RV3028_I2C_ADDRESS;
    hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    hdr.len = num + 1;
    hdr.stop = 1;

    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&siov[1], &reg, sizeof(reg));
    SETIOV(&siov[2], val, num);

    return devctlv(fd, DCMD_I2C_SEND, 3, 0, siov, NULL, NULL);
}

int
RTCFUNC(init,rv3028)(struct chip_loc *chip, char *argv[])
{
    uint8_t backup;

    fd = open((argv && argv[0] && argv[0][0])?
            argv[0]: RV3028_I2C_DEVNAME, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Unable to open I2C device\n");
        return -1;
    }

    /*
     * Put the deivce into direct switching mode so that it uses Vbackup
     * whenver Vdd drops below Vbackup.
     */
    rv3028_i2c_read(RV3028_BACKUP, &backup, 1);
    backup &= 0xf3;
    backup |= 0x04;
    rv3028_i2c_write(RV3028_BACKUP, &backup, 1);

    return 0;
}


int
RTCFUNC(get,rv3028)(struct tm *tm, int cent_reg)
{
    unsigned char   date[7];

    rv3028_i2c_read(RV3028_SEC, date, 7);

    tm->tm_sec  = BCD2BIN(date[RV3028_SEC] & RV3028_SEC_MASK);
    tm->tm_min  = BCD2BIN(date[RV3028_MIN] & RV3028_MIN_MASK);
    tm->tm_hour = BCD2BIN(date[RV3028_HOUR] & RV3028_HOUR_MASK);
    tm->tm_mday = BCD2BIN(date[RV3028_DATE] & RV3028_DATE_MASK);
    tm->tm_mon  = BCD2BIN(date[RV3028_MONTH] & RV3028_MONTH_MASK) - 1;
    tm->tm_year = BCD2BIN(date[RV3028_YEAR]);
    tm->tm_wday = BCD2BIN(date[RV3028_DAY] & RV3028_DAY_MASK);
    // Year in chip is 2000-based, year in structure is 1900-based
    tm->tm_year += 100;

    return(0);
}


int
RTCFUNC(set,rv3028)(struct tm *tm, int cent_reg)
{
    unsigned char   date[7];

    date[RV3028_SEC]   = BIN2BCD(tm->tm_sec);
    date[RV3028_MIN]   = BIN2BCD(tm->tm_min);
    date[RV3028_HOUR]  = BIN2BCD(tm->tm_hour);
    date[RV3028_DAY]   = BIN2BCD(tm->tm_wday);
    date[RV3028_DATE]  = BIN2BCD(tm->tm_mday);
    date[RV3028_MONTH] = BIN2BCD(tm->tm_mon + 1);
    date[RV3028_YEAR]  = BIN2BCD(tm->tm_year - 100);
    rv3028_i2c_write(RV3028_SEC, date, 7);

    return(0);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/utils/r/rtc/clk_rv3028.c $ $Rev: 680331 $")
#endif
