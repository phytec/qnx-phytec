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
#include <time.h>

#define ULTRAZED_RTC_BASE       0xFFA60000
#define SET_TIME_WRITE          0x00000000
#define SET_TIME_READ           0x00000004
#define CALIB_WRITE             0x00000008
#define CALIB_READ              0x0000000C
#define CURRENT_TIME            0x00000010
#define CURRENT_TICK            0x00000014
#define ALARM                   0x00000018
#define RTC_INT_STATUS          0x00000020
#define RTC_INT_MASK            0x00000024
#define RTC_INT_EN              0x00000028
#define RTC_INT_DIS             0x0000002C
#define ADDR_ERROR              0x00000030
#define ADDR_ERROR_INT_MASK     0x00000034
#define ADDR_ERROR_INT_EN       0x00000038
#define ADDR_ERROR_INT_DIS      0x0000003C
#define CONTROL                 0x00000040
#define SAFETY_CHK              0x00000050

#define ULTRAZED_RTC_SIZE       (0x54)

#define CONTROL_BATTERY_EN      (0x1<<31)
#define CONTROL_OSC_CNTRL_EN    (0x1<<24)
#define READ_SIZE               32


/* NOTE: using the calibration value mentioned in the reference manual,
 * for oscillator with different frequency or different static inaccuracy, program
 * this register. Forumla for the calculating new value is given in the
 * reference manual.
 */
#define CALIBRATION_VAL         0x00198231

#define INTERRUPT_CLEAR_MASK    0xFFFF
#define INT_STS_SECS_MASK       0x1

/*
 * Clock setup for the RTC on the AVNET ultrazed SOM.
 */

int
RTCFUNC( init, ultrazed)
(struct chip_loc *chip, char *argv[]) {

  if (chip->phys == NIL_PADDR) {
    chip->phys = ULTRAZED_RTC_BASE;
  }

  if (chip->access_type == NONE) {
    chip->access_type = MEMMAPPED;
  }

  return ULTRAZED_RTC_SIZE;
}

int
RTCFUNC( get, ultrazed)
(struct tm *tm, int cent_reg) {

  time_t t;

  struct tm* address;

  /* Set the oscillator crystal and battery switch enable in control register.
   * The control register must be programmed every time the MPSoc is powered on
   * Otherwise, the value returned by reading the control register can be different
   * from the actual control settings stored in the BPD (latched register).
   */
  unsigned control;
  control = chip_read(CONTROL, READ_SIZE);
  chip_write(CONTROL, control | CONTROL_BATTERY_EN |CONTROL_OSC_CNTRL_EN, READ_SIZE);

  /*
   * read RTC counter value
   * if second interrupt is there after update, it means we
   * can read value from the CURRENT_TIME register, otherwise
   * we have to readback the set value.
   */

  if ((chip_read(RTC_INT_STATUS, READ_SIZE) & INT_STS_SECS_MASK)==0) {

    t = chip_read(SET_TIME_READ, READ_SIZE);
  }

  else
  {
    do {

      t = chip_read(CURRENT_TIME, READ_SIZE);

    }while(chip_read(CURRENT_TIME, READ_SIZE) != t); // get stable value

  }
#ifdef  VERBOSE_SUPPORTED
  if (verbose) {
    printf("rtc read: %ld\n", (long)t);
  }
#endif

  address = gmtime_r(&t, tm);

  if (!address) {

    fprintf(stderr, "RTC: read back failed, set TIME first\n");
  }

  return(0);
}

int
RTCFUNC( set, ultrazed)
(struct tm *tm, int cent_reg) {

  unsigned control;
  time_t t;

  /* Set the calibration value, if it is not set already */
  if (chip_read(CALIB_READ, READ_SIZE) == 0){

    chip_write(CALIB_WRITE, CALIBRATION_VAL, READ_SIZE);
  }

  /* Set the oscillator crystal and battery switch enable in control register. */
  control = chip_read(CONTROL, READ_SIZE);
  chip_write(CONTROL, control | CONTROL_BATTERY_EN |CONTROL_OSC_CNTRL_EN, READ_SIZE);

  /* clear the interrupt status */
  chip_write(RTC_INT_STATUS, INTERRUPT_CLEAR_MASK, READ_SIZE);

  t = mktime(tm);

  /*
   * mktime assumes local time.  We will subtract timezone
   */
  t -= timezone;

#ifdef  VERBOSE_SUPPORTED
  if (verbose) {
    printf("rtc write: %ld\n", (long)t);
  }
#endif

  /* we clear the tick counter and forces the next second to signaled exactly in one second, so +1 */
  chip_write(SET_TIME_WRITE, t+1, READ_SIZE);

  return 0;

}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/utils/r/rtc/nto/aarch64/clk_ultrazed.c $ $Rev: 886297 $")
#endif

