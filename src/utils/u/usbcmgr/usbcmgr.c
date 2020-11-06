/*
 * $QNXLicenseC:
 * Copyright 2020 PHYTEC America
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
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include <hw/i2c.h>

/*
 * Writes to registers: <reg> <len> <data>
 * Reads from registers return: <len> <data>
 */

#define I2C_ADDR	0x3f
#define I2C_DEVNAME	"/dev/i2c1"

int fd;

static int
i2c_write(uint8_t reg, uint8_t *data, uint8_t len)
{
	struct {
		i2c_send_t hdr;
		uint8_t reg;
		uint8_t len;
		uint8_t data[80];
	} msg;

	memset(&msg, 0, sizeof(msg));
	msg.hdr.slave.addr = I2C_ADDR;
	msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
	msg.hdr.len = len + 2;
	msg.hdr.stop = 1;
	msg.reg = reg;
	msg.len = len;
	memcpy(msg.data, data, len);

	return devctl(fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL);
}

static int
i2c_read(uint8_t reg, uint8_t *data, uint8_t len)
{
	struct {
		i2c_sendrecv_t hdr;
		uint8_t data[80];
	} msg;
	int ret;

	memset(&msg, 0, sizeof(msg));
	msg.hdr.slave.addr = I2C_ADDR;
	msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
	msg.hdr.send_len = 1;
	msg.hdr.recv_len = len + 1;	/* data plus one length byte */
	msg.hdr.stop = 1;
	msg.data[0] = reg;

	ret = devctl(fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL);
	if (ret != EOK) {
		fprintf(stderr, "Failed to read register %x (%d)\n", reg, ret);
	} else {
		/* check first byte against expected read lenght? */
		memcpy(data, &msg.data[1], len);
	}
	return ret;
}

void
usage(char *name)
{
    printf("%s: [status | boot]", name);
    printf("  With no option, the program will initialize the USB-C device\n");
    printf("  status: display the fields in the status register\n");
    printf("  boot: display the fields in the boot flags/OTP register\n");
}

int
initialize(void)
{

	int ret = -1;
	/*
	 * 0x29 Control Configuration Register
	 * Maintain normal USB PD behavior
	 * Externally powered
	 * Do no process or initiate swap to sink
	 * Process and initiate swap to source
	 * not SRCIntrusiveMode or SNKIntrusiveMode
	 * ProcessVconnSwap but not InitiateVconnSwap
	 * ProcessSwapToUFP but not InitiateSwapToUFP
	 * ProcessSwapToDFP and InitiateSwapToDFP
	 * Do not automatically send Sink Capabilities request
	 * AutomaticIDRequest
	 * Do not operate in Alternate Mode intrusive mode
	 * Do not force USB Gen1 operation
	 * I2C timeout = 200ms
	 */
	uint8_t ccr[] = { 0xc4, 0xd4, 0x0, 0x01, 0x7 };

	if (i2c_write(0x29, ccr, sizeof(ccr)) != EOK)
		goto done;

	ret = 0;
done:
	return ret;
}

int
status(void)
{
	uint8_t buf[5];
	int ret;

	ret = i2c_read(0x1A, buf, 5);
	if (ret != EOK) {
		fprintf(stderr, "Unable to read status register (%d)\n", ret);
	}

	/* Byte 0 */
	printf("%splug present\n", buf[0] & 0x1 ? "" : "no ");
	printf("connection state: ");
	switch((buf[0] >> 1) & 0x7)
	{
	case 0:
		printf("no connection\n");
		break;
	case 1:
		printf("port is disabled\n");
		break;
	case 2:
		printf("audio connection (Ra/Ra)\n");
		break;
	case 3:
		printf("debug connection (Rd/Rd)\n");
		break;
	case 4:
		printf("no connection (Ra but no Rd)\n");
		break;
	case 5:
		printf("reserved\n");
		break;
	case 6:
		printf("connection present (Rd but no Ra)\n");
		break;
	case 7:
		printf("connection present (Rd and Ra)\n");
		break;
	}

	if (buf[0] & 0x1)
		printf("plug orientation: %s\n",
			buf[0] & 0x10 ? "upside-up" : "upside-down");

	printf("power role: %s\n", buf[0] & 0x20 ? "source" : "sink");
	printf("data role: %s\n", buf[0] & 0x40 ? "DFP" : "UFP");
	printf("VCONN power %senabled\n", buf[0] & 0x80 ? "" : "not ");

	/* Byte 1 */
	printf("PP_5V0 switch ");
	switch(buf[1] & 0x3)
	{
	case 0:
		printf("disabled\n");
		break;
	case 1:
		printf("currently disabled due to fault\n");
		break;
	case 2:
		printf("enabled (system output)\n");
		break;
	case 3:
		printf("invalid setting\n");
		break;
	}

	printf("PP_HV switch ");
	switch((buf[1] >> 2) & 0x3)
	{
	case 0:
		printf("disabled\n");
		break;
	case 1:
		printf("currently disabled due to fault\n");
		break;
	case 2:
		printf("enabled (system output)\n");
		break;
	case 3:
		printf("enabled (system input)\n");
		break;
	}

	printf("PP_EXT switch ");
	switch((buf[1] >> 4) & 0x3)
	{
	case 0:
		printf("disabled\n");
		break;
	case 1:
		printf("currently disabled due to fault\n");
		break;
	case 2:
		printf("enabled (system output)\n");
		break;
	case 3:
		printf("enabled (system input)\n");
		break;
	}

	printf("PP_CABLE switch ");
	switch((buf[1] >> 6) & 0x3)
	{
	case 0:
		printf("disabled\n");
		break;
	case 1:
		printf("currently disabled due to fault\n");
		break;
	case 2:
		printf("enabled (system output)\n");
		break;
	case 3:
		printf("invalid setting\n");
		break;
	}

	/* Byte 2 */
	printf("%sovercurrent condition exists\n", buf[2] & 0x1 ? "" : "no ");

	printf("power source: ");
	switch((buf[2] >> 2) & 0x3)
	{
	case 0:
		printf("unknown\n");
		break;
	case 1:
		printf("VIN_3P3\n");
		break;
	case 2:
		printf("VBUS; dead battery\n");
		break;
	case 3:
		printf("VBUS; battery not dead\n");
		break;
	}

	printf("VBUS status: ");
	switch((buf[2] >> 4) & 0x3)
	{
	case 0:
		printf("at vSafe0V (less than 0.8V)\n");
		break;
	case 1:
		printf("at vSafe5V (4.75V to 5.5V)\n");
		break;
	case 2:
		printf("at PD-negotiated power level and within expected "
			"limits\n");
		break;
	case 3:
		printf("not within determined limits\n");
		break;
	}

	printf("USB host present: ");
	switch((buf[2] >> 6) & 0x3)
	{
	case 0:
		printf("VBUS not provided by far-end device\n");
		break;
	case 1:
		printf("VBUS provided by far-end device without USB "
			"communication\n");
		break;
	case 2:
		printf("VBUS provided by far-end device that is not a PD "
			"device\n");
		break;
	case 3:
		printf("VBUS provided by far-end device capable of USB "
			"communication\n");
		break;
	}

	/* Byte 3 */
	switch(buf[3] & 0x3)
	{
	case 0:
		printf("not in a legacy mode\n");
		break;
	case 1:
		printf("acting as a legacy sink\n");
		break;
	case 2:
		printf("acting as a legacy source\n");
		break;
	}

	if (buf[3] & 0x4)
		printf("GotoMin has been received as sink/sent as source\n");

	printf("BIST %sin progress\n", buf[3] & 0x8 ? "" : "not ");

	printf("%shigh voltage warning\n", buf[3] & 0x10 ? "" : "no ");
	printf("%slow voltage warning\n", buf[3] & 0x20 ? "" : "no ");

	/* Byte 4 */
	return ret;
}

int
bootflags(void)
{
	uint8_t buf[12];
	int ret;

	ret = i2c_read(0x2D, buf, 12);
	if (ret != EOK) {
		fprintf(stderr, "Unable to read boot flags register (%d)\n",
			ret);
	}

	/* Byte 0 */
	printf("SRAM has %sbeen loaded\n", buf[0] & 0x1 ? "" : "not ");
	printf("SPI flash %spresent\n", buf[0] & 0x8 ? "" : "not ");
	printf("SPI region 0 %sattempted\n", buf[0] & 0x10 ? "" : "not ");
	printf("SPI region 1 %sattempted\n", buf[0] & 0x20 ? "" : "not ");
	printf("SPI region 0 %sinvalid\n", buf[0] & 0x40 ? "" : "not ");
	printf("SPI region 1 %sinvalid\n", buf[0] & 0x80 ? "" : "not ");
	printf("PP_EXT sink path %senabled during dead-battery mode\n",
			buf[0] & 0x2 ? "" : "not ");
	printf("%s in dead-battery mode\n",
			buf[0] & 0x4 ? "booted" : "did not boot");

	/* Byte 1 */
	printf("%serror reading SPI region 0\n", buf[1] & 0x1 ? "" : "no ");
	printf("%serror reading SPI region 1\n", buf[1] & 0x2 ? "" : "no ");
	printf("%sCRC error from SPI region 0\n", buf[1] & 0x10 ? "" : "no ");
	printf("%sCRC error from SPI region 1\n", buf[1] & 0x20 ? "" : "no ");

	return EOK;
}

int
main(int argc, char *argv[])
{
	int rc = EINVAL;

	fd = open(I2C_DEVNAME, O_RDWR);
	if (fd < 0) {
		fprintf(stderr, "Failed to open %s\n", I2C_DEVNAME);
		return -1;
	}

	if (argc == 1) {
		rc = initialize();
	} else if (argc == 2) {
		if (!strcmp(argv[1], "status")) {
			rc = status();
		} else if (!strcmp(argv[1], "boot")) {
			rc = bootflags();
		} else {
			usage(argv[0]);
		}
	} else {
		usage(argv[0]);
	}

	return rc;
}
