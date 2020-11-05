/*
 * $QNXLicenseC:
 * Copyright 2019, QNX Software Systems.
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
 * TI TPS65910 PMIC/RTC
 */
#define TPS65910_SECONDS_REG 0x00
#define TPS65910_MINUTES_REG 0x01
#define TPS65910_HOURS_REG   0x02
#define TPS65910_DAYS_REG    0x03
#define TPS65910_MONTHS_REG  0x04
#define TPS65910_YEARS_REG   0x05
#define TPS65910_WEEKDAY_REG 0x06
#define TPS65910_CTRL_REG    0x10

#define TPS65910_I2C_ADDRESS  0x2d
#define TPS65910_I2C_DEVNAME  "/dev/i2c0"

static int fd = -1;
static unsigned char ctrl[1] = {0}; /* Will store the ctrl register in here, and
                                       set/clear any bits we need to. Will recycle
                                       this for the GET_TIME feature using static
                                       shadowed registers on the RTC. */

static int
tps65910_i2c_read(unsigned char reg, unsigned char val[], unsigned char num)
{
    iov_t           siov[2], riov[2];
    i2c_sendrecv_t  hdr;

    hdr.slave.addr = TPS65910_I2C_ADDRESS;
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
tps65910_i2c_write(unsigned char reg, unsigned char val[], unsigned char num)
{
    iov_t           siov[3];
    i2c_send_t      hdr;

    hdr.slave.addr = TPS65910_I2C_ADDRESS;
    hdr.slave.fmt = I2C_ADDRFMT_7BIT;
    hdr.len = num + 1;
    hdr.stop = 1;

    SETIOV(&siov[0], &hdr, sizeof(hdr));
    SETIOV(&siov[1], &reg, sizeof(reg));
    SETIOV(&siov[2], val, num);

    return devctlv(fd, DCMD_I2C_SEND, 3, 0, siov, NULL, NULL);
}

int
RTCFUNC(init,tps65910)(struct chip_loc *chip, char *argv[])
{
    fd = open((argv && argv[0] && argv[0][0])?
            argv[0]: TPS65910_I2C_DEVNAME, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "Unable to open I2C device\n");
        return -1;
    } else {
        tps65910_i2c_read(TPS65910_CTRL_REG, ctrl, 1);
        ctrl[0] |= (1 << 0); /* Set STOP_RTC to set it running if it wasn't for some reason. */
        ctrl[0] |= (1 << 7); /* Set RTC_V_OPT RTC to store RTC reg vals in shadow registers.
                                Need to set the GET_TIME bit each time to updatet hose regs. */
        ctrl[0] &= ~(1 << 3); /* Clear MODE_12_24, set RTC into 24 hour mode */
        ctrl[0] |= (1 << 6); /* Set GET_TIME to update shadow registers */
    }
    return 0;
}

int
RTCFUNC(get,tps65910)(struct tm *tm, int cent_reg)
{
    unsigned char   date[7];

    tps65910_i2c_write(TPS65910_CTRL_REG, ctrl, 1); /* Make sure GET_TIME was set, and that we're in 24 hr mode */
    tps65910_i2c_read(TPS65910_SECONDS_REG, date, 7);

    tm->tm_sec  = BCD2BIN(date[TPS65910_SECONDS_REG]);
    tm->tm_min  = BCD2BIN(date[TPS65910_MINUTES_REG]);
    tm->tm_hour = BCD2BIN(date[TPS65910_HOURS_REG]);
    tm->tm_mday = BCD2BIN(date[TPS65910_DAYS_REG]);
    tm->tm_mon  = BCD2BIN(date[TPS65910_MONTHS_REG]) - 1;
    tm->tm_year = BCD2BIN(date[TPS65910_YEARS_REG]);
    tm->tm_wday = BCD2BIN(date[TPS65910_WEEKDAY_REG]) - 1;

    if(tm->tm_year < 70)
        tm->tm_year += 100;

    return(0);
}

int
RTCFUNC(set,tps65910)(struct tm *tm, int cent_reg)
{
    unsigned char   date[7];

    date[TPS65910_SECONDS_REG] = BIN2BCD(tm->tm_sec);
    date[TPS65910_MINUTES_REG] = BIN2BCD(tm->tm_min);
    date[TPS65910_HOURS_REG] = BIN2BCD(tm->tm_hour);
    date[TPS65910_DAYS_REG] = BIN2BCD(tm->tm_mday);
    date[TPS65910_MONTHS_REG] = BIN2BCD(tm->tm_mon + 1);
    date[TPS65910_YEARS_REG] = BIN2BCD(tm->tm_year % 100);
    date[TPS65910_WEEKDAY_REG] = BIN2BCD(tm->tm_wday + 1);

    tps65910_i2c_write(TPS65910_CTRL_REG, ctrl, 1); /* Make sure RTC is running, and in 24 hr mode */
    tps65910_i2c_write(TPS65910_SECONDS_REG, date, 7);

    return(0);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/utils/r/rtc/clk_tps65910.c $ $Rev: 891264 $")
#endif
