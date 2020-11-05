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

/*
 *  rpmb.h   Non-portable low-level devctl definitions
 *
*/

#ifndef __RPMB_H_INCLUDED
#define __RPMB_H_INCLUDED

#ifndef _DEVCTL_H_INCLUDED
 #include <devctl.h>
#endif

#ifndef __CAM_DEVICE_H_INCLUDED
 #include <sys/cam_device.h>
#endif

#ifndef __DCMD_CAM_H_INCLUDED
 #include <sys/dcmd_cam.h>
#endif

#include <_pack64.h>

#define RPMB_MAC_SIZE           32
#define RPMB_KEY_SIZE           RPMB_MAC_SIZE
#define RPMB_NONCE_SIZE         16
#define RPMB_DATA_SIZE          256
#define RPMB_STUFF_SIZE         196
#define RPMB_FRAME_SIZE         sizeof(rpmb_frame_t)

#define RPMB_ERR_OK				0x0000
#define RPMB_ERR_GENERAL		0x0001
#define RPMB_ERR_AUTH			0x0002
#define RPMB_ERR_COUNTER		0x0003
#define RPMB_ERR_ADDRESS		0x0004
#define RPMB_ERR_WRITE			0x0005
#define RPMB_ERR_READ			0x0006
#define RPMB_ERR_NO_KEY			0x0007
#define RPMB_ERR_COUNTER_EXPIRED	0x0080
#define RPMB_ERR_MASK			0x0007

/* JEDEC RPMB frame structure (eMMC, UFS) */
typedef struct __attribute__((__packed__)) rpmb_frame_jedec_s {
	_Uint8t		stuff[RPMB_STUFF_SIZE];
	_Uint8t		key_mac[RPMB_KEY_SIZE];
	_Uint8t		data[RPMB_DATA_SIZE];
	_Uint8t		nonce[RPMB_NONCE_SIZE];
	_Uint32t	write_counter;	/* big endian */
	_Uint16t	addr;			/* big endian */
	_Uint16t	block_count;	/* big endian */
	_Uint16t	result;			/* big endian */
	_Uint16t	req_resp;		/* big endian */
} rpmb_frame_jedec_t;

/* backwards compat */
typedef rpmb_frame_jedec_t RPMB_FRAME;

_Static_assert(sizeof(rpmb_frame_jedec_t) == 512, "invalid rpmb_frame_jedec_t size");

#define RPMB_STUFF_NVME_SIZE		223

/* NVMe RPMB frame structure */
typedef struct __attribute__((__packed__)) rpmb_frame_nvme_s {
	union {
		struct {
			_Uint8t		stuff[RPMB_STUFF_NVME_SIZE-RPMB_MAC_SIZE];
			_Uint8t		key_mac[RPMB_MAC_SIZE];
		} hmac;
	};
	_Uint8t		target;			/* target partition */
	_Uint8t		nonce[RPMB_NONCE_SIZE];
	_Uint32t	write_counter;	/* little endian */
	_Uint32t	addr;			/* little endian */
	_Uint32t	block_count;	/* little endian */
	_Uint16t	result;			/* little endian */
	_Uint16t	req_resp;		/* little endian */
	_Uint8t		data[RPMB_DATA_SIZE];
} rpmb_frame_nvme_t;

_Static_assert(sizeof(rpmb_frame_nvme_t) == 512, "invalid rpmb_frame_nvme_t size");

typedef union rpmb_frame_u {
	rpmb_frame_jedec_t	jedec;
	rpmb_frame_nvme_t	nvme;
} rpmb_frame_t;

/*
 * RPMB request/reponse types
 * Response values are identical to request values shifted by a byte (0xNN00)
 */
#define	RPMB_PROGRAM_KEY		0x1		/* Program Authentication Key */
#define	RPMB_GET_WRITE_COUNTER	0x2		/* Read Write Counter */
#define	RPMB_WRITE_DATA			0x3		/* Write Data */
#define	RPMB_READ_DATA			0x4		/* Read Data */
#define	RPMB_READ_RESULT		0x5		/* Read Result Request (Handled automatically by QNX RPMB drivers) - Has no response counterpart */
#define	RPMB_WRITE_SWPCB		0x6		/* Secure Write Protect Configuration Block write request */
#define	RPMB_READ_SWPCB			0x7		/* Secure Write Protect Configuration Block read request */

/* Convert between response and request values */
#define RPMB_RESP2REQ(CMD) ((CMD) >> 8)
#define RPMB_REQ2RESP(CMD) ((CMD) << 8)

typedef struct rpmb_info_s {
#define RPMB_DTYPE_UNKNOWN		0
#define RPMB_DTYPE_EMMC			1
#define RPMB_DTYPE_UFS			2
#define RPMB_DTYPE_NVME			3
#define RPMB_DTYPE_SIM			4
	_Uint32t		dtype;
#define RPMB_FLAG_WP			0x01
	_Uint32t		flags;
	_Uint32t		maxio;				/* Max frames that can read/written */
	_Uint64t		start_lba;			/* Starting lba */
	_Uint64t		num_lba;			/* Num lba */
	_Uint32t		parts;				/* Available partitions 1-4 (NVMe) */
#define RPMB_AUTH_ALG_HMAC		0x01
	_Uint32t		algo;				/* Authentication algorithm */
	_Uint32t		rsvd1[62];
} rpmb_info_t;

/* backwards compat */
typedef rpmb_info_t RPMB_INFO;

typedef struct rpmb_cmdhdr_s {
	_Uint32t		flags;
#define	RPMB_FLG_READ		0x00
#define	RPMB_FLG_WRITE		0x01
#define	RPMB_FLG_RL_WRITE	0x02		/* Reliable Write */
	_Uint32t		nframes;			/* Number of frames to read/write */

#define RPMB_SP_NVME		0xea
#define RPMB_SP_SCSA		0xeb
#define RPMB_SP_UFS			0xec
#define RPMB_SP_SDCARD		0xed
#define RPMB_SP_ATA			0xef
	_Uint8t			sp;					/* Security Protocol */
	_Uint8t			partno;				/* Partition number (UFS, NVMe) */
	_Uint16t		sps;				/* Security Protocol Specific */
	_Uint8t			rsvd2[20];
/*	_Uint8t			frames[0];		Variable number of frames */
} rpmb_cmdhdr_t;

/* backwards compat */
typedef rpmb_cmdhdr_t RPMB_CMDHDR;

#define DCMD_RPMB_CMD			__DIOTF(_DCMD_CAM, _SIM_RPMB + 0, rpmb_cmdhdr_t)
#define DCMD_RPMB_INFO			__DIOF(_DCMD_CAM, _SIM_RPMB + 1, rpmb_info_t)

#include <_packpop.h>

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/devb/cam/public/sys/rpmb.h $ $Rev: 906897 $")
#endif

