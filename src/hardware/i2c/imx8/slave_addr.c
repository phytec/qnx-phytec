/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
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

#include "proto.h"


/**
 * Sets I2C slave address for send() and receive() commands.
 *
 * @param hdl  Pointer to I2C driver device structure.
 * @param addr I2C device slave address.
 * @param fmt  I2C device slave address length I2C_ADDRFMT_10BIT or I2C_ADDRFMT_7BIT.
 *
 * @return Execution status.
 * @retval -1 Fail with errno set.
 * @retval  0 Success.
 */
int imx_set_slave_addr(void *hdl, unsigned int addr, i2c_addrfmt_t fmt)
{
    imx_dev_t *dev = hdl;
    if ((fmt != I2C_ADDRFMT_7BIT) && (fmt != I2C_ADDRFMT_10BIT)) {
        errno = EINVAL;
        return -1;
    }
    dev->slave_addr = addr;
    dev->slave_addr_fmt = fmt;
    return 0;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/i2c/imx8/slave_addr.c $ $Rev: 869802 $")
#endif
