/*
 * $QNXLicenseC:
 * Copyright 2014, QNX Software Systems.
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

#ifndef IMX6X_DEVNP_IOCTL_H
#define IMX6X_DEVNP_IOCTL_H

#include <stdint.h>


#define IMX6X_IOCTL SIOCGDRVSPEC

#define GET_BRCM_SQI	0x1000
#define READ_BRCM_MII	0x1001
#define WRITE_BRCM_MII	0x1002
#define ENABLE_BRCM_PHY_LOWPOWER		0x1003
#define DISABLE_BRCM_PHY_LOWPOWER		0x1004
#define GENERIC_MII_OP		0x1005
#define	READ_PHY_REG	0x400
#define	WRITE_PHY_REG	0x401

/*mii op*/
#define SLEEP	0
#define READ	1
#define WRITE	2

typedef struct {
    uint8_t	sqi;		/* sqi  */
} mx6q_get_sqi_t;

typedef struct {
	uint8_t		address;		/* mii register address  */
	uint16_t	data;			/* for read, data is the value read from register, for write, data is the value write to the register*/
} mx6q_mii_request_t;

typedef struct {
	uint8_t		op;				/* mii op,  0 = sleep; 1 = read, 2 = write */
	uint8_t		addr;			/* mii MDIO phy address  */
	uint8_t		reg;			/* mii register address  */
	uint16_t	data;			/* for read, data is the value read from register, for write, data is the value write to the register, for sleep, it's the ms to sleep*/
	int			err;			/* result of this access/sleep. 0 means success. otherwise failed*/
} mx6q_mii_op_t;

typedef struct {
	uint32_t	size;			/* total size of mii_op follow*/
	mx6q_mii_op_t mii_op;		/* the first mii_op*/
} mx6q_mii_ops_req_t;

typedef	struct {
	uint8_t		phy_id;
	uint8_t		phy_reg;
	uint16_t	phy_data;
} phy_access_t;

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devnp/mx6x/public/hw/imx6x_devnp_ioctl.h $ $Rev: 845671 $")
#endif
