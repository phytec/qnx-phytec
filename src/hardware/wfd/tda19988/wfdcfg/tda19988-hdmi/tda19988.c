/*
 * $QNXLicenseC:
 * Copyright 2013, QNX Software Systems.
 * Copyright 2020, PHYTEC America
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

#include <wfdqnx/wfdcfg.h>
#include <wfdqnx/wfdcfg_imx8.h>
#include <stddef.h>
#include <assert.h>
#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <pthread.h>
#include <sys/slog.h>
#include <sys/slogcodes.h>
#include <hw/i2c.h>
#include "tda19988.h"

// #define TDADEBUG		/* Dump registers when the mode is set */
// #define TEST_PATTERN		/* Display a test pattern generated within */
				/* the TDA19988 instead of using the LVDS  */
				/* data. */

/* One device and two ports, corresponding to two LVDS ports / TDA19988 chips */
#define MAX_PORT_NUM		2

#define MAX_I2C_PORT_NAME	20
#define CONFIG_FILE_NAME	"/etc/system/config/tda19988.conf"

#define EDID_LENGTH	128
#define MAX_I2C_XFER	EDID_LENGTH

#define MAX_PIX_CLOCK	68000	/* SN75LVDS82 limited to 68MHz */

/* Used to keep track of which ports have a TDA19988 connected to them */
int enabled_port[MAX_PORT_NUM] = { 0 };

int i2c_bus[MAX_PORT_NUM] = { 0 };

/* Used to keep track of VIP_CNTR_x settings for video bit positions */
uint8_t vip_cntrl_0;
uint8_t vip_cntrl_1;
uint8_t vip_cntrl_2;


int wfdcfg_port_init(struct wfdcfg_port *port);
int wfdcfg_port_uninit(struct wfdcfg_port *port);
int wfdcfg_port_set_mode(struct wfdcfg_port *port,
			struct wfdcfg_timing *timing);

struct wfdcfg_device {
	const struct wfdcfg_keyval *ext_list;
};

struct wfdcfg_port {
	int id;
	int i2c_fd;
	int last_hpd;
	const struct wfdcfg_keyval *ext_list;

	pthread_t pollthread;
	pthread_mutex_t page_mutex;;
	uint8_t current_page;
};

struct wfdcfg_mode_list {
	const struct mode *first_mode;
};

// Internal structure to keep a mode and its associated extension(s).
struct mode {
	struct wfdcfg_timing timing;
	struct wfdcfg_keyval *ext;
	struct mode *nextmode;
};

#define TIMINGLINE(khz, h1, h2, h3, h4, v1, v2, v3, v4, f) \
	{ .pixel_clock_kHz = khz, \
	.hpixels = (h1), .hfp = (h2) - (h1), .hsw = (h3) - (h2), \
	.hbp = (h4) - (h3), \
	.vlines  = (v1), .vfp = (v2) - (v1), .vsw = (v3) - (v2), \
	.vbp = (v4) - (v3), \
	.flags = (f), } \

/*
 * There are 17 established (bit field) timings and up to 4 detailed timing
 * descriptors in an EDID structure.  Plus one as a null terminator.
 */
#define MAX_MODES	22

/*
 * This array holds the modes that we pass to the display driver.  We start
 * off with a simple list and then change it when we read the EDID information
 * from the attached monitor.
 */
struct mode modes[MAX_MODES][MAX_PORT_NUM];


/***************************************************************************
 * The first part of this file deals with the interface to the TDA19988.
 * Below are the routines that implement the wfdcfg interface to the
 * display driver.
 */

/*
 * CEC register write/read functions.  CEC registers are not paged, have an
 * 8-bit address and are 8 bits wide.
 */
static int
cec_write(struct wfdcfg_port *port, uint8_t addr, uint8_t val)
{
	struct {
		i2c_send_t hdr;
		uint8_t reg;
		uint8_t val;
	} msg;

	memset(&msg, 0, sizeof(msg));
	msg.hdr.slave.addr = CEC_I2C_ADDR;
	msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
	msg.hdr.len = sizeof(addr) + sizeof(val);
	msg.hdr.stop = 1;
	msg.reg = addr;
	msg.val = val;

	return devctl(port->i2c_fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL);
}

static uint8_t
cec_read(struct wfdcfg_port *port, uint8_t addr)
{
	struct {
		i2c_sendrecv_t hdr;
		uint8_t data;
	} msg;
	uint8_t val;
	int ret;

	memset(&msg, 0, sizeof(msg));

	msg.hdr.slave.addr = CEC_I2C_ADDR;
	msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
	msg.hdr.send_len = sizeof(addr);
	msg.hdr.recv_len = sizeof(val);
	msg.hdr.stop = 0;
	msg.data = addr;

	ret = devctl(port->i2c_fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL);
	if (ret != EOK) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda19988 cec_read: failed to read addr %d\n", addr);
		val = 0;
	} else {
		val = msg.data;
	}

	return val;
}

/*
 * HDMI registers appear on a number of pages.  Given a register that we
 * are about to access, set the page accordingly.
 */
static int
set_page(struct wfdcfg_port *port, uint16_t reg)
{
	struct {
		i2c_send_t hdr;
		uint8_t select;
		uint8_t page;
	} msg;
	int ret = EOK;

	if (REG2PAGE(reg) != port->current_page) {
		memset(&msg, 0, sizeof(msg));
		msg.hdr.slave.addr = HDMI_I2C_ADDR;
		msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
		msg.hdr.len = sizeof(msg.select) + sizeof(msg.page);
		msg.hdr.stop = 1;
		msg.select = REG_CURPAGE;
		msg.page = REG2PAGE(reg);

		ret = devctl(port->i2c_fd, DCMD_I2C_SEND, &msg, sizeof(msg),
				NULL);
		if (ret != EOK) {
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
				"tda19988: failed to set to page %d (%d)\n",
				REG2PAGE(reg), ret);
		} else {
			port->current_page = REG2PAGE(reg);
		}
	}
	return ret;
}

static int
reg_read_range(struct wfdcfg_port *port, uint16_t reg, uint8_t *buf, int cnt)
{
	struct {
		i2c_sendrecv_t hdr;
		uint8_t data[MAX_I2C_XFER];
	} msg;
	int ret;

	if (cnt > MAX_I2C_XFER)
		return EINVAL;

	pthread_mutex_lock(&(port->page_mutex));
	ret = set_page(port, reg);
	if (ret != EOK)
		goto out;

	memset(&msg, 0, sizeof(msg));
	msg.hdr.slave.addr = HDMI_I2C_ADDR;
	msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
	msg.hdr.send_len = 1;
	msg.hdr.recv_len = cnt;
	msg.hdr.stop = 1;
	msg.data[0] = REG2ADDR(reg);

	ret = devctl(port->i2c_fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL);
	if (ret != EOK) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda19988 reg_read_range: failed (err %d)\n", ret);
	} else {
		memcpy(buf, msg.data, cnt);
	}

out:
	pthread_mutex_unlock(&(port->page_mutex));
	return ret;
}

static int
reg_write_range(struct wfdcfg_port *port, uint16_t reg, uint8_t *buf, int cnt)
{
	struct {
		i2c_send_t hdr;
		uint8_t data[MAX_I2C_XFER+1];
	} msg;
	int ret;

	if (cnt > MAX_I2C_XFER)
		return EINVAL;

	memset(&msg, 0, sizeof(msg));
	msg.hdr.slave.addr = HDMI_I2C_ADDR;
	msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
	msg.hdr.len = cnt + 1;
	msg.hdr.stop = 1;

	msg.data[0] = REG2ADDR(reg);
	memcpy(&msg.data[1], buf, cnt);

	pthread_mutex_lock(&(port->page_mutex));
	ret = set_page(port, reg);
	if (ret != EOK)
		goto out;

	ret = devctl(port->i2c_fd, DCMD_I2C_SEND, &msg, sizeof(msg), NULL);
	if (ret != EOK) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda19988 reg_write_range: failed (err %d)\n", ret);
	}

out:
	pthread_mutex_unlock(&(port->page_mutex));
	return ret;
}

static int
reg_read(struct wfdcfg_port *port, uint16_t reg)
{
	uint8_t val = 0;

	reg_read_range(port, reg, &val, sizeof(val));
	return val;
}

static int
reg_write(struct wfdcfg_port *port, uint16_t reg, uint8_t val)
{
	return reg_write_range(port, reg, &val, sizeof(val));
}

static int reg_write16(struct wfdcfg_port *port, uint16_t reg, uint16_t val)
{
	uint8_t buf[2];

	buf[0] = val >> 8;
	buf[1] = val & 0xFF;

	return reg_write_range(port, reg, buf, sizeof(buf));
}

static void
reg_set(struct wfdcfg_port *port, uint16_t reg, uint8_t val)
{
	uint8_t oldval;

	oldval = reg_read(port, reg);
	reg_write(port, reg, oldval | val);
}

static void
reg_clear(struct wfdcfg_port *port, uint16_t reg, uint8_t val)
{
	uint8_t oldval;

	oldval = reg_read(port, reg);
	reg_write(port, reg, oldval & ~val);
}

#ifdef TDADEBUG
#define dumpone(port, r, name) \
	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1, \
	"tdadump: %s (%x/%x) = %02x\n", name, REG2PAGE(r), REG2ADDR(r), \
		reg_read(port, r));

static void
tda19988_dump(struct wfdcfg_port *port)
{
	dumpone(port, REG_VERSION_LSB, "REG_VERSION_LSB");
	dumpone(port, REG_MAIN_CNTRL0, "REG_MAIN_CNTRL0");
	dumpone(port, REG_VERSION_MSB, "REG_VERSION_MSB");
	dumpone(port, REG_DDC_DISABLE, "REG_DDC_DISABLE");
	dumpone(port, REG_CCLK_ON,     "REG_CCLK_ON    ");
	dumpone(port, REG_I2C_MASTER,  "REG_I2C_MASTER ");
	dumpone(port, REG_FEAT_POWERDOWN, "REG_FEAT_POWERDOWN ");
	dumpone(port, REG_INT_FLAGS_0, "REG_INT_FLAGS_0");
	dumpone(port, REG_INT_FLAGS_1, "REG_INT_FLAGS_1");
	dumpone(port, REG_INT_FLAGS_2, "REG_INT_FLAGS_2");
	dumpone(port, REG_ENA_ACLK, "REG_ENA_ACLK");
	dumpone(port, REG_ENA_VP_0, "REG_ENA_VP_0");
	dumpone(port, REG_ENA_VP_1, "REG_ENA_VP_1");
	dumpone(port, REG_ENA_VP_2, "REG_ENA_VP_2");
	dumpone(port, REG_ENA_AP, "REG_ENA_AP");
	dumpone(port, REG_VIP_CNTRL_0, "REG_VIP_CNTRL_0");
	dumpone(port, REG_VIP_CNTRL_1, "REG_VIP_CNTRL_1");
	dumpone(port, REG_VIP_CNTRL_2, "REG_VIP_CNTRL_2");
	dumpone(port, REG_VIP_CNTRL_3, "REG_VIP_CNTRL_3");
	dumpone(port, REG_VIP_CNTRL_4, "REG_VIP_CNTRL_4");
	dumpone(port, REG_VIP_CNTRL_5, "REG_VIP_CNTRL_5");
	dumpone(port, REG_MUX_AP, "REG_MUX_AP");
	dumpone(port, REG_MUX_VP_VIP_OUT, "REG_MUX_VP_VIP_OUT");
	dumpone(port, REG_MAT_CONTRL, "REG_MAT_CONTRL");
	dumpone(port, REG_VIDFORMAT, "REG_VIDFORMAT");
	dumpone(port, REG_REFPIX_MSB, "REG_REFPIX_MSB");
	dumpone(port, REG_REFPIX_LSB, "REG_REFPIX_LSB");
	dumpone(port, REG_REFLINE_MSB, "REG_REFLINE_MSB");
	dumpone(port, REG_REFLINE_LSB, "REG_REFLINE_LSB");
	dumpone(port, REG_NPIX_MSB, "REG_NPIX_MSB");
	dumpone(port, REG_NPIX_LSB, "REG_NPIX_LSB");
	dumpone(port, REG_NLINE_MSB, "REG_NLINE_MSB");
	dumpone(port, REG_NLINE_LSB, "REG_NLINE_LSB");
	dumpone(port, REG_VS_LINE_STRT_1_MSB, "REG_VS_LINE_STRT_1_MSB");
	dumpone(port, REG_VS_LINE_STRT_1_LSB, "REG_VS_LINE_STRT_1_LSB");
	dumpone(port, REG_VS_PIX_STRT_1_MSB, "REG_VS_PIX_STRT_1_MSB");
	dumpone(port, REG_VS_PIX_STRT_1_LSB, "REG_VS_PIX_STRT_1_LSB");
	dumpone(port, REG_VS_LINE_END_1_MSB, "REG_VS_LINE_END_1_MSB");
	dumpone(port, REG_VS_LINE_END_1_LSB, "REG_VS_LINE_END_1_LSB");
	dumpone(port, REG_VS_PIX_END_1_MSB, "REG_VS_PIX_END_1_MSB");
	dumpone(port, REG_VS_PIX_END_1_LSB, "REG_VS_PIX_END_1_LSB");
	dumpone(port, REG_VS_LINE_STRT_2_MSB, "REG_VS_LINE_STRT_2_MSB");
	dumpone(port, REG_VS_LINE_STRT_2_LSB, "REG_VS_LINE_STRT_2_LSB");
	dumpone(port, REG_VS_PIX_STRT_2_MSB, "REG_VS_PIX_STRT_2_MSB");
	dumpone(port, REG_VS_PIX_STRT_2_LSB, "REG_VS_PIX_STRT_2_LSB");
	dumpone(port, REG_VS_LINE_END_2_MSB, "REG_VS_LINE_END_2_MSB");
	dumpone(port, REG_VS_LINE_END_2_LSB, "REG_VS_LINE_END_2_LSB");
	dumpone(port, REG_VS_PIX_END_2_MSB, "REG_VS_PIX_END_2_MSB");
	dumpone(port, REG_VS_PIX_END_2_LSB, "REG_VS_PIX_END_2_LSB");
	dumpone(port, REG_HS_PIX_START_MSB, "REG_HS_PIX_START_MSB");
	dumpone(port, REG_HS_PIX_START_LSB, "REG_HS_PIX_START_LSB");
	dumpone(port, REG_HS_PIX_STOP_MSB, "REG_HS_PIX_STOP_MSB");
	dumpone(port, REG_HS_PIX_STOP_LSB, "REG_HS_PIX_STOP_LSB");
	dumpone(port, REG_VWIN_START_1_MSB, "REG_VWIN_START_1_MSB");
	dumpone(port, REG_VWIN_START_1_LSB, "REG_VWIN_START_1_LSB");
	dumpone(port, REG_VWIN_END_1_MSB, "REG_VWIN_END_1_MSB");
	dumpone(port, REG_VWIN_END_1_LSB, "REG_VWIN_END_1_LSB");
	dumpone(port, REG_VWIN_START_2_MSB, "REG_VWIN_START_2_MSB");
	dumpone(port, REG_VWIN_START_2_LSB, "REG_VWIN_START_2_LSB");
	dumpone(port, REG_VWIN_END_2_MSB, "REG_VWIN_END_2_MSB");
	dumpone(port, REG_VWIN_END_2_LSB, "REG_VWIN_END_2_LSB");
	dumpone(port, REG_DE_START_MSB, "REG_DE_START_MSB");
	dumpone(port, REG_DE_START_LSB, "REG_DE_START_LSB");
	dumpone(port, REG_DE_STOP_MSB, "REG_DE_STOP_MSB");
	dumpone(port, REG_DE_STOP_LSB, "REG_DE_STOP_LSB");
	dumpone(port, REG_TBG_CNTRL_0, "REG_TBG_CNTRL_0");
	dumpone(port, REG_TBG_CNTRL_1, "REG_TBG_CNTRL_1");
	dumpone(port, REG_ENABLE_SPACE, "REG_ENABLE_SPACE");
	dumpone(port, REG_HVF_CNTRL_0, "REG_HVF_CNTRL_0");
	dumpone(port, REG_HVF_CNTRL_1, "REG_HVF_CNTRL_1");
	dumpone(port, REG_RPT_CNTRL, "REG_RPT_CNTRL");
	dumpone(port, REG_I2S_FORMAT, "REG_I2S_FORMAT");
	dumpone(port, REG_AIP_CLKSEL, "REG_AIP_CLKSEL");
	dumpone(port, REG_PLL_SERIAL_1, "REG_PLL_SERIAL_1");
	dumpone(port, REG_PLL_SERIAL_2, "REG_PLL_SERIAL_2");
	dumpone(port, REG_PLL_SERIAL_3, "REG_PLL_SERIAL_3");
	dumpone(port, REG_SERIALIZER, "REG_SERIALIZER");
	dumpone(port, REG_BUFFER_OUT, "REG_BUFFER_OUT");
	dumpone(port, REG_PLL_SCG1, "REG_PLL_SCG1");
	dumpone(port, REG_PLL_SCG2, "REG_PLL_SCG2");
	dumpone(port, REG_PLL_SCGN1, "REG_PLL_SCGN1");
	dumpone(port, REG_PLL_SCGN2, "REG_PLL_SCGN2");
	dumpone(port, REG_PLL_SCGR1, "REG_PLL_SCGR1");
	dumpone(port, REG_PLL_SCGR2, "REG_PLL_SCGR2");
	dumpone(port, REG_AUDIO_DIV, "REG_AUDIO_DIV");
	dumpone(port, REG_SEL_CLK, "REG_SEL_CLK");
	dumpone(port, REG_ANA_GENERAL, "REG_ANA_GENERAL");
	dumpone(port, REG_EDID_CTRL, "REG_EDID_CTRL");
	dumpone(port, REG_DDC_ADDR, "REG_DDC_ADDR");
	dumpone(port, REG_DDC_OFFS, "REG_DDC_OFFS");
	dumpone(port, REG_DDC_SEGM_ADDR, "REG_DDC_SEGM_ADDR");
	dumpone(port, REG_DDC_SEGM, "REG_DDC_SEGM");
	dumpone(port, REG_IF1_HB0, "REG_IF1_HB0");
	dumpone(port, REG_IF2_HB0, "REG_IF2_HB0");
	dumpone(port, REG_IF3_HB0, "REG_IF3_HB0");
	dumpone(port, REG_IF4_HB0, "REG_IF4_HB0");
	dumpone(port, REG_IF5_HB0, "REG_IF5_HB0");
	dumpone(port, REG_TX3, "REG_TX3");
	dumpone(port, REG_TX4, "REG_TX4");
	dumpone(port, REG_TX33, "REG_TX33");
}
#endif	/* defined TDADEBUG */

static void
tda19988_reset(struct wfdcfg_port *port)
{
	/* reset audio and i2c master: */
	reg_write(port, REG_SOFTRESET, SOFTRESET_AUDIO | SOFTRESET_I2C_MASTER);
	delay(50);
	reg_write(port, REG_SOFTRESET, 0);
	delay(50);

	/* reset transmitter: */
	reg_set(port, REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);
	reg_clear(port, REG_MAIN_CNTRL0, MAIN_CNTRL0_SR);

	/* PLL registers common configuration */
	reg_write(port, REG_PLL_SERIAL_1, 0x00);
	reg_write(port, REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(1));
	reg_write(port, REG_PLL_SERIAL_3, 0x00);
	reg_write(port, REG_SERIALIZER, 0x00);
	reg_write(port, REG_BUFFER_OUT, 0x00);
	reg_write(port, REG_PLL_SCG1, 0x00);
	reg_write(port, REG_AUDIO_DIV, AUDIO_DIV_SERCLK_8);
	reg_write(port, REG_SEL_CLK, SEL_CLK_SEL_CLK1 | SEL_CLK_ENA_SC_CLK);
	reg_write(port, REG_PLL_SCGN1, 0xfa);
	reg_write(port, REG_PLL_SCGN2, 0x00);
	reg_write(port, REG_PLL_SCGR1, 0x5b);
	reg_write(port, REG_PLL_SCGR2, 0x00);
	reg_write(port, REG_PLL_SCG2, 0x10);

	/* Write the default value MUX register */
	reg_write(port, REG_MUX_VP_VIP_OUT, 0x24);
}


/*
 * Request EDID data.  This routine is run when we see the cable plugged in.
 * Afterwards, the poll routine will monitor for when the requested data is
 * available and handle it.
 */
static void
request_edid(struct wfdcfg_port *port)
{
	uint8_t block = 0;
	uint8_t offset, segptr;

	offset = (block & 1) ? 128 : 0;
	segptr = block / 2;

	reg_clear(port, REG_TX4, TX4_PD_RAM);
	reg_write(port, REG_DDC_ADDR, 0xa0);
	reg_write(port, REG_DDC_OFFS, offset);
	reg_write(port, REG_DDC_SEGM_ADDR, 0x60);
	reg_write(port, REG_DDC_SEGM, segptr);

	/* enable reading EDID: */
	reg_write(port, REG_EDID_CTRL, 0x1);

	/* flag must be cleared by software */
	reg_write(port, REG_EDID_CTRL, 0x0);
}

/*
 * The EDID standard gives you the size and frequency for established (common),
 * but not the other timing parameters.  These timings are taken from
 * hw/xfree86/modes/xf86EdidModes.c
 */
struct wfdcfg_timing established_timings[] = {
	/* Byte 35 bit fields */
	/* 800x600 @ 60 Hz */
	TIMINGLINE(40000, 800, 840, 968, 1056, 600, 601, 605, 628,
		   WFDCFG_INVERT_HSYNC | WFDCFG_INVERT_VSYNC),
	/* 800x600 @ 56 Hz */
	TIMINGLINE(36000, 800, 824, 896, 1024, 600, 601, 603, 625,
		   WFDCFG_INVERT_HSYNC | WFDCFG_INVERT_VSYNC),
	/* 640x480 @ 75 Hz */
	TIMINGLINE(31500, 640, 656, 720, 840, 480, 481, 484, 500, 0),
	/* 640x480 @ 72 Hz */
	TIMINGLINE(31500, 640, 664, 704, 832, 480, 489, 492, 520, 0),
	/* 640x480 @ 67 Hz */
	TIMINGLINE(30240, 640, 704, 768, 864, 480, 483, 486, 525, 0),
	/* 640x480 @ 60 Hz */
	TIMINGLINE(25175, 640, 656, 752, 800, 480, 490, 492, 525, 0),
	/* 720x400 @ 88 Hz */
	TIMINGLINE(35500, 720, 738, 846, 900, 400, 421, 423, 449, 0),
	/* 720x400 @ 70 Hz */
	TIMINGLINE(28320, 720, 738, 846, 900, 400, 412, 414, 449,
			 WFDCFG_INVERT_VSYNC),

	/* Byte 36 bit fields */
	/* 1280x1024 @ 75 Hz */
	TIMINGLINE(135000, 1280, 1296, 1440, 1688, 1024, 1025, 1028, 1066,
		   WFDCFG_INVERT_HSYNC | WFDCFG_INVERT_VSYNC),
	/* 1024x768 @ 75 Hz */
	TIMINGLINE(78750, 1024, 1040, 1136, 1312, 768, 769, 772, 800, 0),
	/* 1024x768 @ 70 Hz */
	TIMINGLINE(75000, 1024, 1048, 1184, 1328, 768, 771, 777, 806, 0),
	/* 1024x768 @ 60 Hz */
	TIMINGLINE(65000, 1024, 1048, 1184, 1344, 768, 771, 777, 806, 0),
	/* 1024x768 @ 43 Hz interlaced */
	TIMINGLINE(44900, 1024, 1032, 1208, 1264, 768, 768, 776, 817,
		   WFDCFG_INVERT_HSYNC | WFDCFG_INVERT_VSYNC |
		   WFDCFG_INTERLACE),
	/* 832x624 @ 75 Hz */
	TIMINGLINE(57284, 832, 864, 928, 1152, 624, 625, 628, 667,
		   WFDCFG_INVERT_HSYNC | WFDCFG_INVERT_VSYNC),
	/* 800x600 @ 75 Hz */
	TIMINGLINE(49500, 800, 816, 896, 1056, 600, 601, 604, 625,
		   WFDCFG_INVERT_HSYNC | WFDCFG_INVERT_VSYNC),
	/* 800x600 @ 72 Hz */
	TIMINGLINE(50000, 800, 856, 976, 1040, 600, 637, 643, 666,
		   WFDCFG_INVERT_HSYNC | WFDCFG_INVERT_VSYNC),

	/* Byte 37 bit fields (bit 7) */
	/* 1152x870 @ 75 Hz (actually use x864) */
	TIMINGLINE(108000, 1152, 1216, 1344, 1600, 864, 865, 868, 900, 0),
};

/*
 * The status indicates that the requested EDID data is available.  Read and
 * analyze it.
 */
void
read_edid(struct wfdcfg_port *port, unsigned int block)
{
	uint8_t buf[EDID_LENGTH];
	uint8_t *descr;
	uint8_t checksum;
	uint32_t *ptr32;
	unsigned int hbl, hfp, hsw;
	unsigned int vbl, vfp, vsw;
	uint32_t mask, established;
	struct wfdcfg_timing *timing;
	int i, ret, modenum = 0;

	ret = reg_read_range(port, REG_EDID_DATA_0, buf, EDID_LENGTH);
	reg_set(port, REG_TX4, TX4_PD_RAM);
	reg_write(port, REG_AIP_CNTRL_0, AIP_CNTRL_0_RST_FIFO);

	if (ret != EOK) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda failed to read EDID block %d: %d\n", block, ret);
		return;
	}

	/* verify the header */
	ptr32 = (uint32_t *)buf;
	if ((*ptr32 != 0xffffff00) || (*(ptr32+1) != 0x00ffffff)) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda read invalid EDID\n");
		return;
	}

	/* verify the EDID version (0x01 for EDID 1.3 or 1.4) */
	if (buf[18] != 0x01) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda read unsupported EDID version\n");
		return;
	}

	checksum = 0;
	for (i=0; i<EDID_LENGTH; i++)
		checksum += buf[i];
	if (checksum != 0) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda EDID with incorrect checksum\n");
		return;
	}

	if (!(buf[20] & 0x80)) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda Attached display is not digital\n");
		return;
	}

	/* Start with the established timing bitmap */
	timing = &(modes[modenum][port->id].timing);
	established = (buf[37] << 16) | (buf[36] << 8) | buf[35];

	/* First, special case due to non-contiguous bits */
	if ((established & 0x800) &&
		(established_timings[16].pixel_clock_kHz < MAX_PIX_CLOCK))
	{
		*timing = established_timings[16];
		modenum++;
	}

	/*
	 * Next, loop through remaining bits.  We do this in two passes so
	 * that we put the ones with the higher resolution first.  The
	 * graphics driver uses the first one in the list unless the
	 * graphics.conf file provides a specific resolution.
	 */
	mask = (0x1 << 8);
	for (i=8; i<16; i++) {
		if (established_timings[i].pixel_clock_kHz > MAX_PIX_CLOCK)
			continue;
		if (mask & established) {
			timing = &(modes[modenum][port->id].timing);
			*timing = established_timings[i];
			modenum++;
		}
		mask = mask << 1;
	}

	mask = 0x1;
	for (i=0; i<7; i++) {
		if (established_timings[i].pixel_clock_kHz > MAX_PIX_CLOCK)
			continue;
		if (mask & established) {
			timing = &(modes[modenum][port->id].timing);
			*timing = established_timings[i];
			modenum++;
		}
		mask = mask << 1;
	}

	/* There are 4 descriptors per EDID, each 18 bytes long */
	for (i=0; i<4; i++) {
		uint32_t clock;

		descr = &buf[54 + i*18];
		/*
		 * We are only interested in detailed timing descriptors.
		 * All of the others start with 0s in the first two bytes.
		 */
		if ((descr[0] == 0) && (descr[1] == 0))
			continue;

		clock = (descr[0] + (descr[1] << 8)) * 10;
		if (clock > MAX_PIX_CLOCK) {
			uint32_t hpixels, vlines;

			hpixels = (descr[2] + ((descr[4] & 0xf0) << 4));
			vlines = (descr[5] + ((descr[7] & 0xf0) << 4));

			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
				"skipping descriptor %d with clock %d, "
				"%dx%d\n", i, clock, hpixels, vlines);
			continue;
		}

		timing = &(modes[modenum][port->id].timing);
		timing->pixel_clock_kHz = clock;
		timing->hpixels = (descr[2] + ((descr[4] & 0xf0) << 4));
		timing->vlines = (descr[5] + ((descr[7] & 0xf0) << 4));

		hbl = (descr[3] + ((descr[4] & 0x0f) << 8));
		hfp = (descr[8] + ((descr[11] & 0xc0) << 2));
		hsw = (descr[9] + ((descr[11] & 0x30) << 4));
		vbl = (descr[6] + ((descr[7] & 0x0f) << 8));
		vfp = ((descr[10] >> 4) + ((descr[11] & 0x0c) << 2));
		vsw = ((descr[10] & 0x0f) + ((descr[11] & 0x03) << 4));

		timing->hsw = hsw;
		timing->vsw = vsw;
		timing->hfp = hfp;
		timing->vfp = vfp;
		timing->hbp = hbl - hfp - hsw;
		timing->vbp = vbl - vfp - vsw;

		timing->flags = 0;
		if ((descr[17] & 0x18) == 0x10) {
			if (descr[17] & 0x04)
				timing->flags |= WFDCFG_INVERT_VSYNC;
		} else if ((descr[17] & 0x18) == 0x18) {
			if (descr[17] & 0x02)
				timing->flags |= WFDCFG_INVERT_HSYNC;
		}
		if (descr[17] & 0x80)
			timing->flags |= WFDCFG_INTERLACE;

		modenum++;
	}
	// mark end of list
	timing = &(modes[modenum][port->id].timing);
	timing->pixel_clock_kHz = 0;

	return;
}

/*
 * Enable the video ports
 */
static void
tda19988_enable(struct wfdcfg_port *port)
{
	/* enable video ports */
	reg_write(port, REG_ENA_VP_0, 0xff);
	reg_write(port, REG_ENA_VP_1, 0xff);
	reg_write(port, REG_ENA_VP_2, 0xff);

	/* set muxing */
	reg_write(port, REG_VIP_CNTRL_0, vip_cntrl_0);
	reg_write(port, REG_VIP_CNTRL_1, vip_cntrl_1);
	reg_write(port, REG_VIP_CNTRL_2, vip_cntrl_2);
}

/*
 * Disable the video ports
 */
static void
tda19988_disable(struct wfdcfg_port *port)
{
	/* disable video ports */
	reg_write(port, REG_ENA_VP_0, 0x00);
	reg_write(port, REG_ENA_VP_1, 0x00);
	reg_write(port, REG_ENA_VP_2, 0x00);
}


#if 0
The QNX graphics driver uses the wfdcfg DLLs to determine the supported
display timings as an alternative to having them hard-coded in graphics.conf.
Unfortunately, the driver only queries the DLLs when the graphics stack
("screen") is started.  That means there is no point in us checking for
a monitor being removed and then reconnected.  We would know about the
new timings that are supported, but the graphics driver will not ask for
them.  As a result, this poll is not used.  It is left here in case
the QNX grpahics driver should ever change.

Also note that this is a poll instead of an interrupt routine.  The
initial PEB-LVDS-01 board had no interrupt support.

/*
 * Periodically check hotplug detect to see if the display has just been
 * plugged in or unplugged.  Also check if previously requested EDID data
 * is available for reading.
 */
void *
do_poll(void *param)
{
	struct wfdcfg_port *port = (struct wfdcfg_port *)param;
	uint8_t level, flag2;

	while (1) {
		level = cec_read(port, REG_CEC_RXSHPDLEV) & CEC_RXSHPDLEV_HPD;
		level = level ? 1 : 0;

		if (!(port->last_hpd & level)) {
			/*
			 * just plugged in
			 * The chip has a problem when tryig to read the EDID
			 * close to a HPD assertion.  It needs 100ms to avoid
			 * timing out while trying to read EDID data.
			 */
			usleep(100000);
			request_edid(port);
		} else if (port->last_hpd & !level) {
			tda19988_disable(port);
		}
		port->last_hpd = level;

		flag2 = reg_read(port, REG_INT_FLAGS_2);

		if (flag2 & INT_FLAGS_2_EDID_BLK_RD) {
			read_edid(port, 0);
		}

		delay(2000);
	}
}
#endif


/***************************************************************************
 * The following code implements the wfdcfg interface to the display driver.
 * See http://www.qnx.com/developers/docs/7.0.0/#com.qnx.doc.screen.wfdcfg/topic/manual/rwfdcfg_overview.html
 */


/* Helper functions */

static const struct mode*
cast_timing_to_mode(const struct wfdcfg_timing *timing)
{
	char *p = (char*)timing;
	if (p) {
		p -= offsetof(struct mode, timing);
	}
	return (const struct mode*)p;
}

static const struct wfdcfg_keyval*
get_ext_from_list(const struct wfdcfg_keyval *ext_list, const char *key)
{
	while (ext_list) {
		if (!ext_list->key) {
			ext_list = NULL;
			break;
		} else if (strcmp(ext_list->key, key) == 0) {
			return ext_list;
		}
		++ext_list;
	}
	return NULL;
}


static const struct wfdcfg_keyval device_ext[] = {
	{ NULL }  // marks end of list
};

/*
 * The basic wfdcfg library functions pass information to the display driver
 * about the supported display modes.  In order to support programming of
 * the TDA19988 when the display is included in graphics.conf and to support
 * programming it when a mode is selected, we require these port extension
 * functions.
 */
static const struct wfdcfg_keyval port_ext[] = {
	{ WFDCFG_EXT_FN_PORT_INIT1, 0, wfdcfg_port_init },
	{ WFDCFG_EXT_FN_PORT_UNINIT1, 0, wfdcfg_port_uninit },
	{ WFDCFG_EXT_FN_PORT_SET_MODE2, 0, wfdcfg_port_set_mode },
	{ NULL }  // marks end of list
};


/*
 * This is called as a result of having a "begin wfd device N" section in
 * graphics.conf which includes this library in the wfd-dlls list.  Generally,
 * it is therefore called just once.  Later one or more ports will be created
 * and initialized, corresponding to one or more displays.
 */
int
wfdcfg_device_create(struct wfdcfg_device **device, int deviceid,
		const struct wfdcfg_keyval *opts)
{
	int err = EOK;
	struct wfdcfg_device *tmp_dev = NULL;
	FILE *fp;
	char line[100], param[80], setting[80];
	int value;
	int port = 0;
	(void)opts;

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_device_create: deviceid %d\n", deviceid);

	switch(deviceid) {
		case 1:
			tmp_dev = malloc(sizeof(*tmp_dev));
			if (!tmp_dev) {
				err = ENOMEM;
				goto end;
			}

			tmp_dev->ext_list = device_ext;
			break;

		default:
			/* Invalid device id*/
			err = ENOENT;
			goto end;
	}

	fp = fopen(CONFIG_FILE_NAME, "r");
	if (fp == NULL) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda unable to open config file %s\n",
			CONFIG_FILE_NAME);
		err = EIO;
		goto end;
	}

	while (fgets(line, 100, fp) != NULL) {
		if ((line[0] == '#') || (line[0] == '\n') || (line[0] == '\r'))
			continue;
		if (sscanf(line, "%s %s\n", param, setting) != 2) {
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
				"tda invalid line in config file: %s\n",
				line);
			continue;
		}

		if (!strcmp(param, "port")) {
			value = atoi(setting);
			if ((value >=  0) && (value < MAX_PORT_NUM)) {
				port = value;
			}
		} else if (!strcmp(param, "enabled")) {
			value = atoi(setting);
			enabled_port[port] = value ? 1 : 0;
		} else if (!strcmp(param, "i2cbus")) {
			i2c_bus[port] = atoi(setting);
		} else if (!strcmp(param, "VipCtrl0")) {
			vip_cntrl_0 = strtol(setting, NULL, 0);
		} else if (!strcmp(param, "VipCtrl1")) {
			vip_cntrl_1 = strtol(setting, NULL, 0);
		} else if (!strcmp(param, "VipCtrl2")) {
			vip_cntrl_2 = strtol(setting, NULL, 0);
		} else {
			slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
				"tda invalid configuration parameter %s\n",
				param);
		}
	}
	fclose(fp);


end:
	if (err) {
		free(tmp_dev);
	} else {
		*device = tmp_dev;
	}
	return err;
}

const struct wfdcfg_keyval*
wfdcfg_device_get_extension(const struct wfdcfg_device *device,
		const char *key)
{
	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_device_get_extension %s\n", key);

	return get_ext_from_list(device->ext_list, key);
}

void
wfdcfg_device_destroy(struct wfdcfg_device *device)
{
	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_device_destroy\n");

	free(device);
}

/*
 * A port is created (and later initialized) for each display in graphics.conf
 */
int
wfdcfg_port_create(struct wfdcfg_port **port,
		const struct wfdcfg_device *device, int portid,
		const struct wfdcfg_keyval *opts)
{
	int err = EOK;
	struct wfdcfg_port *tmp_port = NULL;
	(void)opts;

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_port_create: portid %d\n", portid);

	assert(device);

	switch(portid) {
		default:
			tmp_port = malloc(sizeof(*tmp_port));
			if (!tmp_port) {
				err = ENOMEM;
				goto end;
			}
			tmp_port->id = portid - 1;
			tmp_port->ext_list = port_ext;
			break;
		case 0:  // invalid
			err = ENOENT;
			goto end;
	}

end:
	if (err) {
		free(tmp_port);
	} else {
		*port = tmp_port;
	}
	return err;
}

const struct wfdcfg_keyval*
wfdcfg_port_get_extension(const struct wfdcfg_port *port, const char *key)
{
	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_port_get_extension %s\n", key);

	return get_ext_from_list(port->ext_list, key);
}

void
wfdcfg_port_destroy(struct wfdcfg_port *port)
{
	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_port_destroy\n");

	free(port);
}

/*
 * There is a one-to-one mapping between TDA19988 chips and ports.  This
 * routine is thefore called once per chip and is used to initialize it.
 */
int
wfdcfg_port_init(struct wfdcfg_port *port)
{
	int err = EOK;
	char i2c_port[MAX_I2C_PORT_NAME];
	int i;

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_port_init\n");

	/*
	 * Create a linked list for wfdcfg_mode_list_get_next to run through.
	 * This is needed since modes is a two-dimensional array and we don't
	 * want to rely on the order that the elements are stored im memory.
	 */
	for (i=0; i<MAX_MODES-1; i++) {
		modes[i][port->id].nextmode = &(modes[i+1][port->id]);
	}
	modes[MAX_MODES-1][port->id].nextmode = NULL;

	if (enabled_port[port->id] == 0) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
			"tda wfdcfg_port_init: port %d disabled in "
			"configuration file\n", port->id);

		/*
		 * If this library is included with the wfd DLLs in
		 * graphics.conf, it is used with every port.  That's a
		 * problem if there's a mixture of LCD and HDMI.  So we
		 * define here a single timing that corresponds to LCD-18.
		 */
		modes[0][port->id].timing.pixel_clock_kHz = 33000;
		modes[0][port->id].timing.hpixels = 800;
		modes[0][port->id].timing.vlines = 480;
		modes[0][port->id].timing.hsw = 128;
		modes[0][port->id].timing.vsw = 2;
		modes[0][port->id].timing.hfp = 40;
		modes[0][port->id].timing.vfp = 10;
		modes[0][port->id].timing.hbp = 216;
		modes[0][port->id].timing.vbp = 35;
		modes[0][port->id].timing.flags = 0;
		modes[1][port->id].timing.pixel_clock_kHz = 0;	/* end marker */

		goto done;
	} else {
		/*
		 * In case we can't read the EDID information to determine
		 * what the monitor supports, start with a default/safe
		 * setting.  Otherwise the graphics driver will fail when
		 * we give it an empty list of supported modes.
		 * If/when we do read EDID, we'll replace this.
		 */
		struct wfdcfg_timing *timing = &(modes[0][port->id].timing);

		*timing = established_timings[0];
		modes[1][port->id].timing.pixel_clock_kHz = 0;	/* end marker */
	}

	snprintf(i2c_port, MAX_I2C_PORT_NAME - 1, "/dev/i2c%d",
		i2c_bus[port->id]);
	port->i2c_fd = open(i2c_port, O_RDWR);
	if (port->i2c_fd < 0) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda failed to open I2C device %s\n", i2c_port);
		return port->i2c_fd;
	}

	port->current_page = 0xff;

	err = pthread_mutex_init(&(port->page_mutex), NULL);
	if (err != EOK) {
		slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR,
			"tda failed initialize mutex\n");
		return err;
	}

	/* green bits on VPB pins, aka d_vp[15..8] */
	vip_cntrl_0 = VIP_CNTRL_0_SWAP_A(2) | VIP_CNTRL_0_SWAP_B(3);
	/* blue bits on VPC pins, aka d_vp[23..16] */
	vip_cntrl_1 = VIP_CNTRL_1_SWAP_C(0) | VIP_CNTRL_1_SWAP_D(1);
	/* red bits on VPA pins, aka d_vp[7..0] */
	vip_cntrl_2 = VIP_CNTRL_2_SWAP_E(4) | VIP_CNTRL_2_SWAP_F(5);

	/* wake up the device: */
	cec_write(port, REG_CEC_ENAMODS, CEC_ENAMODS_EN_HDMI);

	tda19988_reset(port);

	/* enable DDC: */
	reg_write(port, REG_DDC_DISABLE, 0x00);

	/* set clock on DDC channel: */
	reg_write(port, REG_TX3, 39);

	cec_write(port, REG_CEC_FRO_IM_CLK_CTRL,
			CEC_FRO_IM_CLK_CTRL_GHOST_DIS |
			CEC_FRO_IM_CLK_CTRL_IMCLK_SEL);
	cec_write(port, REG_CEC_RXSHPDINTENA, 0);
	cec_write(port, REG_CEC_RXSHPDINTENA, CEC_RXSHPDINT_HPD);

	/*
	 * Clear and enable EDID read irq.  Interrupts are not actually used,
	 * but the setting of this bit in the interrupt status register will
	 * tel us that the read is complete.
	 */
	reg_read(port, REG_INT_FLAGS_2);
	reg_set(port, REG_INT_FLAGS_2, INT_FLAGS_2_EDID_BLK_RD);

#if 0
	See the note above do_poll().  This may be useful some day.

	err = pthread_create(&(port->pollthread), NULL, do_poll, port);
	if (err == EOK) {
		pthread_setname_np(port->pollthread, "tda19988 poll");
		port->pollthread = 0;
	}
#else
	/*
	 * Wait up to 2 seconds for HPD and then read the EDID info.
	 */
	for (i=0; i<100; i++) {
		if (cec_read(port, REG_CEC_RXSHPDLEV) & CEC_RXSHPDLEV_HPD) {
			usleep(100000);
			request_edid(port);
			break;
		} else {
			usleep(10000);
		}
	}
	for (i=0; i<100; i++) {
		if (reg_read(port, REG_INT_FLAGS_2) & INT_FLAGS_2_EDID_BLK_RD) {
			read_edid(port, 0);
			break;
		} else {
			usleep(10000);
		}
	}
#endif

done:
	return err;
}

int
wfdcfg_port_uninit(struct wfdcfg_port *port)
{
	int err = EOK;

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_port_uninit\n");

	if (enabled_port[port->id])
		reg_clear(port, REG_INT_FLAGS_2, INT_FLAGS_2_EDID_BLK_RD);
	if (port->pollthread != 0)
		pthread_cancel(port->pollthread);
	tda19988_disable(port);

	return err;
}

/*
 * This function is of type wfdcfg_ext_fn_port_set_mode2_t and is called to
 * program a selected timing mode.
 */
int
wfdcfg_port_set_mode(struct wfdcfg_port *port, struct wfdcfg_timing *timing)
{
	unsigned long tmds_clock;
	uint16_t ref_pix, ref_line, n_pix, n_line;
	uint16_t hs_pix_s, hs_pix_e;
	uint16_t vs1_pix_s, vs1_pix_e, vs1_line_s, vs1_line_e;
	uint16_t vs2_pix_s, vs2_pix_e, vs2_line_s, vs2_line_e;
	uint16_t vwin1_line_s, vwin1_line_e;
	uint16_t vwin2_line_s, vwin2_line_e;
	uint16_t de_pix_s, de_pix_e;
	uint8_t reg, div, rep;
	int err = EOK;

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_port_set_mode port %d %dx%d\n",
		port->id, timing->hpixels, timing->vlines);

	if (enabled_port[port->id] == 0)
		goto done;

	/*
	 * Internally TDA998x is using ITU-R BT.656 style sync but
	 * we get VESA style sync. TDA998x is using a reference pixel
	 * relative to ITU to sync to the input frame and for output
	 * sync generation. Currently, we are using reference detection
	 * from HS/VS, i.e. REFPIX/REFLINE denote frame start sync point
	 * which is position of rising VS with coincident rising HS.
	 *
	 * Now there is some issues to take care of:
	 * - HDMI data islands require sync-before-active
	 * - TDA998x register values must be > 0 to be enabled
	 * - REFLINE needs an additional offset of +1
	 * - REFPIX needs an addtional offset of +1 for UYUV and +3 for RGB
	 *
	 * So we add +1 to all horizontal and vertical register values,
	 * plus an additional +3 for REFPIX as we are using RGB input only.
	 */
	n_pix  = timing->hfp + timing->hpixels + timing->hsw + timing->hbp;
	n_line = timing->vfp + timing->vlines +  timing->vsw + timing->vbp;

	hs_pix_e = timing->hfp + timing->hsw;
	hs_pix_s = timing->hfp;
	de_pix_e = n_pix;
	de_pix_s = timing->hfp + timing->hsw + timing->hbp;
	ref_pix  = 3 + hs_pix_s;

	if ((timing->flags & WFDCFG_INTERLACE) == 0) {
		ref_line     = 1 + timing->vfp;
		vwin1_line_s = timing->vfp + timing->vsw + timing->vbp - 1;
		vwin1_line_e = vwin1_line_s + timing->vlines;
		vs1_pix_s    = vs1_pix_e = hs_pix_s;
		vs1_line_s   = timing->vfp;
		vs1_line_e   = vs1_line_s + timing->vsw;
		vwin2_line_s = vwin2_line_e = 0;
		vs2_pix_s    = vs2_pix_e  = 0;
		vs2_line_s   = vs2_line_e = 0;
	} else {
		ref_line     = 1 + timing->vfp/2;
		vwin1_line_s = (timing->vfp + timing->vsw + timing->vbp)/2;
		vwin1_line_e = vwin1_line_s + timing->vlines/2;
		vs1_pix_s    = vs1_pix_e = hs_pix_s;
		vs1_line_s   = timing->vfp/2;
		vs1_line_e   = vs1_line_s + timing->vsw/2;
		vwin2_line_s = vwin1_line_s + n_line/2;
		vwin2_line_e = vwin2_line_s + timing->vlines/2;
		vs2_pix_s    = vs2_pix_e = hs_pix_s + n_pix/2;
		vs2_line_s   = vs1_line_s + n_pix/2 ;
		vs2_line_e   = vs2_line_s + timing->vsw/2;
	}

	tmds_clock = timing->pixel_clock_kHz;

	/*
	 * The divisor is a power-of-2.  The TDA9983B datasheet gives
	 * this as a range of Msample/s, which is 10x the TMDS clock:
	 *   0 - 800 to 1500 Msamples/s
	 *   1 - 400 to 800 Msamples/s
	 *   2 - 200 to 400 Msamples/s
	 *   3 - as 2 above
	 */
	for (div = 0; div < 3; div++)
		if (80000 >> div <= tmds_clock)
			break;

	/* set HDMI HDCP mode off: */
	reg_write(port, REG_TBG_CNTRL_1, TBG_CNTRL_1_DWIN_DIS);
	reg_clear(port, REG_TX33, TX33_HDMI);
	reg_write(port, REG_ENC_CNTRL, ENC_CNTRL_CTL_CODE(0));

	/* no pre-filter or interpolator: */
#ifndef TEST_PATTERN
	reg_write(port, REG_HVF_CNTRL_0, HVF_CNTRL_0_PREFIL(0) |
			HVF_CNTRL_0_INTPOL(0));
#else
	/*
	 * generate a test pattern inside the TDA19988 instead of using the
	 * LVDS data.  Handy for debugging.
	 */
	reg_write(port, REG_HVF_CNTRL_0, HVF_CNTRL_0_PREFIL(0) |
			HVF_CNTRL_0_INTPOL(0) |
			HVF_CNTRL_0_SM | HVF_CNTRL_0_RWB);
#endif
	reg_set(port, REG_FEAT_POWERDOWN, FEAT_POWERDOWN_PREFILT);

	/* Specific to PEB-LVDS-01: No audio support */
	reg_set(port, REG_FEAT_POWERDOWN, FEAT_POWERDOWN_SPDIF);
	reg_write(port, REG_ENA_ACLK, 0);
	reg_write(port, REG_ENA_AP, 0);
	reg_write(port, REG_VIP_CNTRL_5, VIP_CNTRL_5_SP_CNT(0));
	reg_write(port, REG_VIP_CNTRL_4, VIP_CNTRL_4_BLANKIT(0) |
			VIP_CNTRL_4_BLC(0));

	reg_clear(port, REG_PLL_SERIAL_1, PLL_SERIAL_1_SRL_MAN_IZ);
	reg_clear(port, REG_PLL_SERIAL_3, PLL_SERIAL_3_SRL_CCIR |
			PLL_SERIAL_3_SRL_DE);
	reg_write(port, REG_SERIALIZER, 0);
	reg_write(port, REG_HVF_CNTRL_1, HVF_CNTRL_1_VQR(0));

	rep = 0;
	reg_write(port, REG_RPT_CNTRL, 0);
	reg_write(port, REG_SEL_CLK, SEL_CLK_SEL_VRF_CLK(0) |
			SEL_CLK_SEL_CLK1 | SEL_CLK_ENA_SC_CLK);

	reg_write(port, REG_PLL_SERIAL_2, PLL_SERIAL_2_SRL_NOSC(div) |
			PLL_SERIAL_2_SRL_PR(rep));

	/* set color matrix bypass flag: */
	reg_write(port, REG_MAT_CONTRL, MAT_CONTRL_MAT_BP |
			MAT_CONTRL_MAT_SC(0));
			//MAT_CONTRL_MAT_SC(1));
	reg_set(port, REG_FEAT_POWERDOWN, FEAT_POWERDOWN_CSC);

	/* set BIAS tmds value: */
	reg_write(port, REG_ANA_GENERAL, 0x09);

	/* Sync on rising HSYNC */
	reg = VIP_CNTRL_3_SYNC_HS;

	/*
	 * TDA19988 requires high-active sync at input stage,
	 * so invert low-active sync provided by master encoder here
	 */
	if (!(timing->flags & WFDCFG_INVERT_HSYNC))
		reg |= VIP_CNTRL_3_H_TGL;
	if (!(timing->flags & WFDCFG_INVERT_VSYNC))
		reg |= VIP_CNTRL_3_V_TGL;
	if (timing->flags & WFDCFG_INVERT_DATA_EN)
		reg |= VIP_CNTRL_3_X_TGL;
	reg_write(port, REG_VIP_CNTRL_3, reg);

	reg_write(port, REG_VIDFORMAT, 0x00);
	reg_write16(port, REG_REFPIX_MSB, ref_pix);
	reg_write16(port, REG_REFLINE_MSB, ref_line);
	reg_write16(port, REG_NPIX_MSB, n_pix);
	reg_write16(port, REG_NLINE_MSB, n_line);
	reg_write16(port, REG_VS_LINE_STRT_1_MSB, vs1_line_s);
	reg_write16(port, REG_VS_PIX_STRT_1_MSB, vs1_pix_s);
	reg_write16(port, REG_VS_LINE_END_1_MSB, vs1_line_e);
	reg_write16(port, REG_VS_PIX_END_1_MSB, vs1_pix_e);
	reg_write16(port, REG_VS_LINE_STRT_2_MSB, vs2_line_s);
	reg_write16(port, REG_VS_PIX_STRT_2_MSB, vs2_pix_s);
	reg_write16(port, REG_VS_LINE_END_2_MSB, vs2_line_e);
	reg_write16(port, REG_VS_PIX_END_2_MSB, vs2_pix_e);
	reg_write16(port, REG_HS_PIX_START_MSB, hs_pix_s);
	reg_write16(port, REG_HS_PIX_STOP_MSB, hs_pix_e);
	reg_write16(port, REG_VWIN_START_1_MSB, vwin1_line_s);
	reg_write16(port, REG_VWIN_END_1_MSB, vwin1_line_e);
	reg_write16(port, REG_VWIN_START_2_MSB, vwin2_line_s);
	reg_write16(port, REG_VWIN_END_2_MSB, vwin2_line_e);
	reg_write16(port, REG_DE_START_MSB, de_pix_s);
	reg_write16(port, REG_DE_STOP_MSB, de_pix_e);

	/* let incoming pixels fill the active space (if any) */
	reg_write(port, REG_ENABLE_SPACE, 0x00);

	/*
	 * Always generate sync polarity relative to input sync and
	 * revert input stage toggled sync at output stage
	 */
	reg = TBG_CNTRL_1_DWIN_DIS | TBG_CNTRL_1_TGL_EN;
	if (!(timing->flags & WFDCFG_INVERT_HSYNC))
		reg |= TBG_CNTRL_1_H_TGL;
	if (!(timing->flags & WFDCFG_INVERT_VSYNC))
		reg |= TBG_CNTRL_1_V_TGL;
	reg_write(port, REG_TBG_CNTRL_1, reg);

	/* must be last register set: */
	reg_write(port, REG_TBG_CNTRL_0, 0);

	tda19988_enable(port);

#ifdef TDADEBUG
	tda19988_dump(port);
#endif

done:
	return err;
}

int
wfdcfg_mode_list_create(struct wfdcfg_mode_list **list,
		const struct wfdcfg_port* port, const struct wfdcfg_keyval *opts)
{
	int err = 0;
	struct wfdcfg_mode_list *tmp_mode_list = NULL;

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_mode_list_create for port id %d\n", port->id);

	(void)opts;

	assert(port); (void)port;

	tmp_mode_list = malloc(sizeof *tmp_mode_list);
	if (!tmp_mode_list) {
		err = ENOMEM;
		goto out;
	}
	tmp_mode_list->first_mode = &modes[0][port->id];

out:
	if (err) {
		free(tmp_mode_list);
	} else {
		*list = tmp_mode_list;
	}
	return err;
}

const struct wfdcfg_keyval*
wfdcfg_mode_list_get_extension(const struct wfdcfg_mode_list *list,
		const char *key)
{
	(void)list;

	assert(key);

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_mode_list_get_extension: key %s\n", key);

	return NULL;
}

void
wfdcfg_mode_list_destroy(struct wfdcfg_mode_list *list)
{
	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_mode_list_destroy\n");

	free(list);
}

const struct wfdcfg_timing*
wfdcfg_mode_list_get_next(const struct wfdcfg_mode_list *list,
		const struct wfdcfg_timing *prev_timing)
{
	assert(list);

	const struct mode *m = list->first_mode;

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_mode_list_get_next\n");

	if (prev_timing) {
		m = cast_timing_to_mode(prev_timing);
		m = m->nextmode;
	}

	if (m->timing.pixel_clock_kHz == 0) {
		// end of list (this is not an error)
		m = NULL;
	}

	return (m == NULL) ? NULL : &m->timing;
}

const struct wfdcfg_keyval*
wfdcfg_mode_get_extension(const struct wfdcfg_timing *timing,
		const char *key)
{
	const struct wfdcfg_keyval *ext = cast_timing_to_mode(timing)->ext;

	slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_DEBUG1,
		"tda wfdcfg_mode_get_extension\n");

	return get_ext_from_list(ext, key);
}
