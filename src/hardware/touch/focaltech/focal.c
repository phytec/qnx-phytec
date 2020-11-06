/*
 * Copyright 2020 Phytec
 *
 * Based on the Linux driver edt-ft5x06.c:
 * Copyright (C) 2012 Simon Budig, <simon.budig@kernelconcepts.de>
 * Daniel Wagener <daniel.wagener@kernelconcepts.de> (M09 firmware support)
 * Lothar Waßmann <LW@KARO-electronics.de> (DT support)
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * For information on writing a QNX touch driver, including the required
 * functions/callbacks, see:
 * https://www.qnx.com/developers/docs/7.0.0/index.html#com.qnx.doc.inputevents/topic/manual/cmtouch_writing_driver.html
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <hw/i2c.h>
#include <input/mtouch_driver.h>
#include <input/parseopts.h>
#include <input/mtouch_log.h>

#define WORK_REGISTER_THRESHOLD		0x00
#define WORK_REGISTER_REPORT_RATE	0x08
#define WORK_REGISTER_GAIN		0x30
#define WORK_REGISTER_OFFSET		0x31
#define WORK_REGISTER_NUM_X		0x33
#define WORK_REGISTER_NUM_Y		0x34

#define M09_REGISTER_THRESHOLD		0x80
#define M09_REGISTER_GAIN		0x92
#define M09_REGISTER_OFFSET		0x93
#define M09_REGISTER_NUM_X		0x94
#define M09_REGISTER_NUM_Y		0x95

#define NO_REGISTER			0xff

#define TOUCH_EVENT_DOWN		0x00
#define TOUCH_EVENT_UP			0x01
#define TOUCH_EVENT_ON			0x02
#define TOUCH_EVENT_RESERVED		0x03

#define EDT_NAME_LEN			23

#define MAX_I2C_DATA_LEN	40
#define MAX_I2C_PORT_NAME	20
#define DEFAULT_I2C_NUM		7		/* MIPI DSI0 I2C0 */
#define DEFAULT_I2C_ADDR	0x38
#define DEFAULT_I2C_SPEED	100000
#define DEFAULT_IRQ		(576 + 19)	/* GPIO1_IO19 */

#define THREAD_PRIORITY		21

#define EV_INTR			1		/* code value used in pulse */

enum focal_ver {
	EDT_M06,
	EDT_M09,
};

struct focal_reg_addr {
	int reg_threshold;
	int reg_report_rate;
	int reg_gain;
	int reg_offset;
};

struct focal_data {
	pthread_t thread;
	int chid;	/* channel ID from ChannelCreate() */
	int coid;	/* connect ID from ConnectAttach() */
	int iid;	/* interrupt ID from InterruptAttachEvent() */

	struct mtouch_device *touchdev;	/* from mtouch_driver_attach() */

	int i2c_fd;
	char i2c_num;
	uint32_t i2c_addr;
	unsigned int i2c_speed;

	int irq;

	int threshold;
	int gain;
	int offset;
	int report_rate;
	int max_support_points;

	char name[EDT_NAME_LEN];

	struct focal_reg_addr reg_addr;
	enum focal_ver version;
};

/*
 * Perform an I2C transfer
 *
 * Returns
 *     0 if successful; otherwise non-zero if an error occurred.
 */
int focal_readwrite(struct focal_data *td,
		uint16_t wr_len, uint8_t *wr_buf,
		uint16_t rd_len, uint8_t *rd_buf)
{
	struct send_recv {
		i2c_sendrecv_t hdr;
		uint8_t buf[MAX_I2C_DATA_LEN];
	} msg;

	if ((wr_len > MAX_I2C_DATA_LEN) || (rd_len > MAX_I2C_DATA_LEN))
		return -1;

	memset(&msg, 0, sizeof(msg));
	msg.hdr.send_len = wr_len;
	msg.hdr.recv_len = rd_len;
	msg.hdr.slave.addr = td->i2c_addr;
	msg.hdr.slave.fmt = I2C_ADDRFMT_7BIT;
	msg.hdr.stop = 0;
	memcpy(msg.buf, wr_buf, wr_len);

	if (devctl(td->i2c_fd, DCMD_I2C_SENDRECV, &msg, sizeof(msg), NULL)) {
		mtouch_error("ft5x06", "I2C transfer error: %s",
				strerror(errno));
		return -1;
	}

	if (rd_len > 0) {
		memcpy(rd_buf, msg.buf, rd_len);
	}

	return 0;
}

static int focal_register_write(struct focal_data *td, uint8_t addr,
				uint8_t value)
{
	uint8_t wrbuf[4];

	switch (td->version) {
	case EDT_M06:
		wrbuf[0] = 0xfc;
		wrbuf[1] = addr & 0x3f;
		wrbuf[2] = value;
		wrbuf[3] = wrbuf[0] ^ wrbuf[1] ^ wrbuf[2];
		return focal_readwrite(td, 4, wrbuf, 0, NULL);
	case EDT_M09:
		wrbuf[0] = addr;
		wrbuf[1] = value;

		return focal_readwrite(td, 2, wrbuf, 0, NULL);

	default:
		return -EINVAL;
	}
}

#if 0
// Can be handy for debugging
static int focal_register_read(struct focal_data *td, uint8_t addr)
{
	uint8_t wrbuf[2], rdbuf[2];
	int error;

	switch (td->version) {
	case EDT_M06:
		wrbuf[0] = 0xfc;
		wrbuf[1] = addr & 0x3f;
		wrbuf[1] |= 0x40;

		error = focal_readwrite(td, 2, wrbuf, 2, rdbuf);
		if (error)
			return error;

		if ((wrbuf[0] ^ wrbuf[1] ^ rdbuf[0]) != rdbuf[1]) {
			mtouch_error("ft5x06", "I2C CRC error: ",
					"0x%02x expected, got 0x%02x\n",
					wrbuf[0] ^ wrbuf[1] ^ rdbuf[0],
					rdbuf[1]);
			return -1;
		}
		break;

	case EDT_M09:
		wrbuf[0] = addr;
		error = focal_readwrite(td, 1, wrbuf, 1, rdbuf);
		if (error)
			return error;
		break;

	default:
		return -1;
	}

	return rdbuf[0];
}
#endif

int focal_identify(struct focal_data *td, char *fw_version)
{
	uint8_t rdbuf[EDT_NAME_LEN];
	char *p;
	int err;
	char *model_name = td->name;

	memset(rdbuf, 0, sizeof(rdbuf));

	err = focal_readwrite(td, 1, (uint8_t *)"\xBB", EDT_NAME_LEN-1, rdbuf);
	if (err)
		return err;

        /* if we find something consistent, stay with that assumption
	 * at least M09 won't send 3 bytes here
	 */
	if (!strncasecmp((char *)(rdbuf + 1), "EP0", 3)) {
		td->version = EDT_M06;

		/* remove last '$' end marker */
		rdbuf[EDT_NAME_LEN - 1] = '\0';
		if (rdbuf[EDT_NAME_LEN - 2] == '$')
			rdbuf[EDT_NAME_LEN - 2] = '\0';

		/* look for Model/Version separator */
		p = strchr((char *)rdbuf, '*');
		if (p)
			*p++ = '\0';
		strlcpy(model_name, (char *)(rdbuf + 1), EDT_NAME_LEN);
		strlcpy(fw_version, p ? p : "", EDT_NAME_LEN);
	} else {
		td->version = EDT_M09;

		err = focal_readwrite(td, 1, (uint8_t *)"\xA6", 2, rdbuf);
		if (err)
			return err;

		strlcpy(fw_version, (char *)rdbuf, 2);

		err = focal_readwrite(td, 1, (uint8_t *)"\xA8", 1, rdbuf);
		if (err)
			return err;

		snprintf(model_name, EDT_NAME_LEN, "EP0%i%i0M09",
				rdbuf[0] >> 4, rdbuf[0] & 0x0F);
	}

	return 0;
}

void focal_set_regs(struct focal_data *td)
{
	struct focal_reg_addr *reg_addr = &td->reg_addr;

	switch (td->version) {
	case EDT_M06:
		reg_addr->reg_threshold = WORK_REGISTER_THRESHOLD;
		reg_addr->reg_report_rate = WORK_REGISTER_REPORT_RATE;
		reg_addr->reg_gain = WORK_REGISTER_GAIN;
		reg_addr->reg_offset = WORK_REGISTER_OFFSET;
		break;

	case EDT_M09:
		reg_addr->reg_threshold = M09_REGISTER_THRESHOLD;
		reg_addr->reg_report_rate = NO_REGISTER;
		reg_addr->reg_gain = M09_REGISTER_GAIN;
		reg_addr->reg_offset = M09_REGISTER_OFFSET;
		break;
	}
}

/*
 * Write the registers with options specified for the driver.  This needs to
 * be done after the chip has been identified so that we know the address
 * of the various control registers.
 */
void focal_set_options(struct focal_data *td)
{
	struct focal_reg_addr *reg_addr = &td->reg_addr;

	if (td->threshold)
		focal_register_write(td, reg_addr->reg_threshold,
					td->threshold);
	if (td->gain)
		focal_register_write(td, reg_addr->reg_gain, td->gain);
	if (td->offset)
		focal_register_write(td, reg_addr->reg_offset, td->offset);
}


/*
 * These callback functions are called for each of the touchpoints as a
 * result of focal_thread() calling mtouch_driver_process_packet() when an
 * interrupt occurs. The maximum number of touchpoints is max_touchpoints,
 * as specified in mtouch_driver_params_t (mtouch_params.h).
 */

/*
 * Retrieve the contact ID for the specified digit from packet.
 *
 * Returns
 *     0 if successful; otherwise non-zero if an error occurred.
 */
int focal_get_contact_id(void *packet, uint8_t digit_idx, uint32_t *contact_id,
			void *arg)
{
	/* both are zero-based */
	*contact_id = digit_idx;

	return 0;
}

/*
 * Retrieve the touch status for the specified digit from packet.
 * Sets *valid to 1 for down, 0 for up.
 *
 * Returns
 *     0 if successful; otherwise non-zero if an error occurred.
 */
int focal_is_contact_down(void *packet, uint8_t digit_idx, int *valid,
				void *arg)
{
	struct focal_data *td = (struct focal_data *)arg;
	uint8_t *rdbuf = (uint8_t *)packet;
	uint8_t id;
	int i, offset = 0, tplen = 0;

	*valid = 0;

	switch (td->version) {
	case EDT_M06:
		offset = 5; /* where the actual touch data starts */
		tplen = 4;  /* data comes in so called frames */
		break;
	case EDT_M09:
		offset = 3;
		tplen = 6;
		break;
	};

	for (i = 0; i < td->max_support_points; i++) {
		uint8_t *buf = &rdbuf[i * tplen + offset];
		uint8_t type;

		type = buf[0] >> 6;

		/* ignore reserved events */
		if (type == TOUCH_EVENT_RESERVED)
			continue;

		/* M06 sometimes sends bogus coords in TOUCH_DOWN */
		if (td->version == EDT_M06 && type == TOUCH_EVENT_DOWN)
			continue;

		id = (buf[2] >> 4) & 0x0f;
		if (id == digit_idx) {
			*valid = type != TOUCH_EVENT_UP;
			break;
		}
	}

	return 0;
}

/*
 * Retrieve the coordinates for the specified digit from packet.
 *
 * Returns
 *     0 if successful; otherwise non-zero if an error occurred.
 */
int focal_get_coords(void *packet, uint8_t digit_idx, int32_t *x, int32_t *y,
			void *arg)
{
	struct focal_data *td = (struct focal_data *)arg;
	uint8_t *rdbuf = (uint8_t *)packet;
	uint8_t id;
	int i, offset = 0, tplen = 0;

	switch (td->version) {
	case EDT_M06:
		offset = 5; /* where the actual touch data starts */
		tplen = 4;  /* data comes in so called frames */
		break;
	case EDT_M09:
		offset = 3;
		tplen = 6;
		break;
	};

	for (i = 0; i < td->max_support_points; i++) {
		uint8_t *buf = &rdbuf[i * tplen + offset];
		uint8_t type;

		type = buf[0] >> 6;

		/* ignore reserved events */
		if (type == TOUCH_EVENT_RESERVED)
			continue;

		/* M06 sometimes sends bogus coords in TOUCH_DOWN */
		if (td->version == EDT_M06 && type == TOUCH_EVENT_DOWN)
			continue;

		id = (buf[2] >> 4) & 0x0f;
		if (id == digit_idx) {
			*x = ((buf[0] << 8) | buf[1]) & 0x0fff;
			*y = ((buf[2] << 8) | buf[3]) & 0x0fff;
		}
	}

	return 0;
}

/*
 * Retrieve the number of touchpoints currently in contact with the screen.
 *
 * Returns
 *     0 if successful; otherwise non-zero if an error occurred.
 */
int focal_get_down_count(void *packet, uint32_t *down_count, void *arg)
{
	struct focal_data *td = (struct focal_data *)arg;
	uint8_t *rdbuf = (uint8_t *)packet;
	int count = 0;
	int i, offset = 0, tplen = 0;

	switch (td->version) {
	case EDT_M06:
		offset = 5; /* where the actual touch data starts */
		tplen = 4;  /* data comes in so called frames */
		break;
	case EDT_M09:
		offset = 3;
		tplen = 6;
		break;
	};

	for (i = 0; i < td->max_support_points; i++) {
		uint8_t *buf = &rdbuf[i * tplen + offset];
		uint8_t type;

		type = buf[0] >> 6;

		/* ignore reserved events */
		if (type == TOUCH_EVENT_RESERVED)
			continue;

		/* M06 sometimes sends bogus coords in TOUCH_DOWN */
		if (td->version == EDT_M06 && type == TOUCH_EVENT_DOWN)
			continue;

		if (type != TOUCH_EVENT_UP)
			count++;
	}
	*down_count = count;

	return 0;
}

mtouch_driver_funcs_t focal_funcs = {
	.get_contact_id = focal_get_contact_id,
	.is_contact_down = focal_is_contact_down,
	.get_coords = focal_get_coords,
	.get_down_count = focal_get_down_count,
	.get_touch_width = NULL,
	.get_touch_height = NULL,
	.get_touch_orientation = NULL,
	.get_touch_pressure = NULL,
	.get_seq_id = NULL,
	.set_event_rate = NULL,
	.get_contact_type = NULL,
	.get_select = NULL
};

mtouch_driver_params_t focal_params = {
	.capabilities = MTOUCH_CAPABILITIES_CONTACT_ID |
	                MTOUCH_CAPABILITIES_COORDS |
	                MTOUCH_CAPABILITIES_CONTACT_COUNT,
	.flags = 0,
	.max_touchpoints = 5,
	/*
	 * The width and height in touch units (not pixels) needs to be
	 * specified.  That can be done here with:
	 * .width = 1800,
	 * .height = 1024
	 * Those are the maximum values observed with the touch on LCD-018.
	 * However, instead of specifying them here, do so in the mtouch.conf
	 * file with "options = width=1800,height=1024" so that they can
	 * be adjusted without changing this driver.
	 */
};

/*
 * Check the CRC of the data from the chip.
 * Returns 1 if valid, 0 if not
 */
static uint8_t edt_ft5x06_ts_check_crc(struct focal_data *td, uint8_t *buf,
					int buflen)
{
	int i;
	uint8_t crc = 0;

	for (i = 0; i < buflen - 1; i++)
		crc ^= buf[i];

	if (crc != buf[buflen-1]) {
		mtouch_error("ft5x06", "crc error: 0x%02x expected, "
				"got 0x%02x\n", crc, buf[buflen-1]);
		return 0;
	}

	return 1;
}

void *focal_thread(void *param)
{
	struct focal_data *td;
	uint8_t cmd = 0;
	uint8_t rdbuf[63];
	int offset = 0, tplen = 0, datalen, crclen = 0;
	unsigned int flags = 0;	/* Docs say "internal use only" */
	struct _pulse pulse;
	int err;

	td = (struct focal_data *)param;

	while (1) {
		InterruptUnmask(td->irq, td->iid);

		/*
		 * We receive a pulse everytime there's an interrupt.  Wait
		 * for it.
		 */
		err = MsgReceivePulse(td->chid, &pulse, sizeof(pulse), NULL);
		if (err == -1) {
			continue;
		}

		switch (td->version) {
		case EDT_M06:
			cmd = 0xf9; /* tell the controller to send touch data */
			offset = 5; /* where the actual touch data starts */
			tplen = 4;  /* data comes in so called frames */
			crclen = 1;
			break;
		case EDT_M09:
			cmd = 0x0;
			offset = 3;
			tplen = 6;
			crclen = 0;
			break;
		};
		memset(rdbuf, 0, sizeof(rdbuf));
		datalen = tplen * td->max_support_points + offset + crclen;

		err = focal_readwrite(td, sizeof(cmd), &cmd, datalen, rdbuf);
		if (err) {
			mtouch_error("ft5x06", "Unable to fetch data, "
				"error: %d", err);
			continue;
		}

		if (td->version == EDT_M06) {
			if ((rdbuf[0] != 0xaa) || (rdbuf[1] != 0xaa) ||
			    (rdbuf[2] != datalen)) {
				mtouch_error("ft5x06", "Unexpected header: "
					"%02x%02x%02x!",
					rdbuf[0], rdbuf[1], rdbuf[2]);
				continue;
			}
			if (!edt_ft5x06_ts_check_crc(td, rdbuf, datalen))
				continue;
		}

		/*
		 * call mtouch_driver_process_packet()
		 * The packet passed to it will be given to the callback
		 * routines.
		 */
		 mtouch_driver_process_packet(td->touchdev, (void *)rdbuf,
						td, flags);
	}

	return NULL;
}

/*
 * Parse values from options of @c mtouch section of graphics.conf
 *
 * Returns
 *     0 if successful; otherwise non-zero if an error occurred.
 */
int focal_options(const char* option, const char* value, void* arg)
{
	struct focal_data *td = (struct focal_data *)arg;
	int num;

	if (!strcmp(option, "I2Cnum")) {
		td->i2c_num = atoi(value);
	} else if (!strcmp(option, "I2Caddr")) {
		td->i2c_addr = atoi(value);
	} else if (!strcmp(option, "I2Cspeed")) {
		td->i2c_speed = atoi(value);
	} else if (!strcmp(option, "irq")) {
		td->irq = atoi(value);
	} else if (!strcmp(option, "touchpoints")) {
		/* This can vary with the specific touch chip */
		focal_params.max_touchpoints = atoi(value);
		td->max_support_points = atoi(value);
	} else if (!strcmp(option, "width")) {
		focal_params.width = atoi(value);
	} else if (!strcmp(option, "height")) {
		focal_params.height = atoi(value);
	} else if (!strcmp(option, "threshold")) {
		num = atoi(value);
		if ((num > 0) && (num < 256)) {
			td->threshold = num;
		}
	} else if (!strcmp(option, "gain")) {
		num = atoi(value);
		if ((num > 0) && (num < 256)) {
			td->gain = num;
		}
	} else if (!strcmp(option, "offset")) {
		num = atoi(value);
		if ((num > 0) && (num < 256)) {
			td->offset = num;
		}
	}
	return 0;
}

/*
 * Screen uses dlsym() to look for two entries: mtouch_driver_init() and
 * mtouch_driver_fini().
 */
void *mtouch_driver_init(const char* options)
{
	pthread_attr_t			pattr;
	sched_param_t			param;
	struct focal_data		*td;
	char				i2c_port[MAX_I2C_PORT_NAME];
	struct sigevent			event;
	int				err;
	char				fw_version[EDT_NAME_LEN];

	td = (struct focal_data *)calloc(1, sizeof(struct focal_data));
	if (td == NULL){
		mtouch_error("ft5x06", "Failed to allocate memory for "
				"device structure");
		return NULL;
	}
	memset(td, 0, sizeof(struct focal_data));

	/* Set defaults that may be overridden by the options parsed below */
	td->i2c_num = DEFAULT_I2C_NUM;
	td->i2c_addr = DEFAULT_I2C_ADDR;
	td->i2c_speed = DEFAULT_I2C_SPEED;
	td->irq = DEFAULT_IRQ;
	td->max_support_points = 5;

	/* Parse the mtouch options from graphics.conf */
	input_parseopts(options, focal_options, td);

	snprintf(i2c_port, MAX_I2C_PORT_NAME - 1, "/dev/i2c%d", td->i2c_num);
	td->i2c_fd = open(i2c_port, O_RDWR);
	if (td->i2c_fd < 0) {
		mtouch_error("ft5x06", "I2C open failed: %s", strerror(errno));
		goto failI2COpen;
	}

	err = devctl(td->i2c_fd, DCMD_I2C_SET_BUS_SPEED, &td->i2c_speed,
			sizeof(td->i2c_speed), NULL);
	if (err) {
		mtouch_error("ft5x06", "Failed to set I2C bus speed: %s",
				strerror(errno));
		goto failSetSpeed;
	}

	err = focal_identify(td, fw_version);
	if (err) {
		mtouch_error("ft5x06", "Failed to read identity: %s",
				strerror(errno));
		goto failSetSpeed;
	}

	focal_set_regs(td);
	focal_set_options(td);

	mtouch_log(_SLOG_INFO, "ft5x06", "Model \"%s\", Rev. \"%s\""
			"%dx%d sensors\n", td->name, fw_version);

	/* Connect to device */
	td->chid = ChannelCreate(0);
	if (td->chid == -1) {
		mtouch_error("ft5x06", "ChannelCreate failed: %s",
				strerror(errno));
		goto failChannelCreate;
	}

	td->coid = ConnectAttach(0, 0, td->chid, _NTO_SIDE_CHANNEL, 0);
	if (td->coid == -1) {
		mtouch_error("ft5x06", "%s: ConnectAttach: %s",
				__FUNCTION__, strerror(errno));
		goto failConnectAttach;
	}

	/*
	 * Pulses are short messages that contain an 8 bit code and a 32 bit
	 * value.  Ask for a pulse event to be created and sent whenever
	 * our interrupt occurs.  The thread that we create below will wait
	 * for those pulses to do the interrupt processing.  EV_INTR is the
	 * code and NULL is the value.
	 */
	SIGEV_PULSE_INIT(&event, td->coid, THREAD_PRIORITY, EV_INTR, NULL);
	td->iid = InterruptAttachEvent(td->irq, &event,
						_NTO_INTR_FLAGS_TRK_MSK);
	if (td->iid == -1) {
		mtouch_error("ft5x06", "Failed to attach to interrupt %d: %s",
				td->irq, strerror(errno));
		goto failAttachInterrupt;
	}

	/* Connect to the Input Events library */
	td->touchdev = mtouch_driver_attach(&focal_params, &focal_funcs);
	if (td->touchdev == NULL) {
		goto failAttachLibInputEvents;
	}

	pthread_attr_init(&pattr);
	/* Thread priority for communicating directly with the hardware */
	param.sched_priority = 21;
	pthread_attr_setschedparam(&pattr, &param);
	pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);

	/*
	 * Create a thread for interrupt processing.
	 */
	int ret = pthread_create(&td->thread, &pattr, focal_thread, td);
	if (EOK != ret) {
		mtouch_error("ft5x06", "Failed to create the interrupt "
				"thread (%s - %i)", strerror(errno), ret);
		goto failCreateThread;
	}
	pthread_setname_np(td->thread, "ft5x06");

	return td;

failCreateThread:
	mtouch_driver_detach(td->touchdev);
	td->touchdev = NULL;

failAttachLibInputEvents:
	InterruptDetach(td->iid);
	td->iid = -1;

failAttachInterrupt:
	ConnectDetach(td->coid);
	td->coid = -1;

failConnectAttach:
	ChannelDestroy(td->chid);
	td->chid = -1;

failChannelCreate:
failSetSpeed:
	close(td->i2c_fd);

failI2COpen:
	free(td);

	return NULL;
}

void mtouch_driver_fini(void *dev)
{
	struct focal_data *td = dev;

	pthread_cancel(td->thread);
	pthread_join(td->thread, NULL);

	if (td->touchdev) {
		mtouch_driver_detach(td->touchdev);
		td->touchdev = NULL;
	}

	free(td);
}
