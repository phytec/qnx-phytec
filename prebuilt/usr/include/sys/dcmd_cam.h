/*
 * $QNXLicenseC: 
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

/*
 *  dcmd_cam.h   Non-portable low-level devctl definitions
 *
*/

#ifndef __DCMD_CAM_H_INCLUDED
#define __DCMD_CAM_H_INCLUDED

#ifndef _DEVCTL_H_INCLUDED
 #include <devctl.h>
#endif

#ifndef _SMMU_H_INCLUDED
 #include <smmu.h>
#endif

#ifndef _CDROM_H_INCLUDED
 #include <sys/cdrom.h>
#endif

#ifndef __CAM_DEVICE_H_INCLUDED
 #include <sys/cam_device.h>
#endif

#ifndef _CPT_H_INCLUDED
 #include <sys/cpt.h>
#endif

#include <_pack64.h>


typedef struct cam_devinfo CAM_DEVINFO;

typedef struct media_removable {
	int						prevent;
} MEDIA_REMOVABLE;

typedef struct _data_set_mgnt_range {
    _Uint64t				lba;
    _Uint32t				nlba;
    _Uint32t				rsvd;
} DATA_SET_MGNT_RANGE;

typedef struct _data_set_mgnt {
#define DSM_OPT_TRIM        1
#define DSM_OPT_DISCARD     2
    _Uint32t                opt;
    _Uint32t                nranges;
    _Uint32t                rsvd[4];
/*	DATA_SET_MGNT_RANGE     ranges[ ];  variable length ranges */
} DATA_SET_MGNT;

#define DEVICE_TEMPERATURE_UNKNOWN        0x7fffffff
#define DEVICE_TEMPERATURE_UNSUPPORTED    0xffffffff
#define DEVICE_TSTATE_UNKNOWN	          0x00000000
#define DEVICE_TSTATE_NORMAL	          0x00000001
#define DEVICE_TSTATE_OVERTEMP            0x00000002
#define DEVICE_TSTATE_UNDERTEMP           0x00000004
typedef struct _cam_device_temperature {
    _Uint32t    state;                    /* UNKNOWN, NORMAL, OVERTEMP, UNDERTEMP  */
    _Int32t     current_temperature;      /* (degree C) * 10, UNKNOWN, UNSUPPORTED */
    _Int32t     lowest_temperature;       /* (degree C) * 10, UNKNOWN, UNSUPPORTED */
    _Int32t     highest_temperature;      /* (degree C) * 10, UNKNOWN, UNSUPPORTED */
    _Int32t     minimum_temperature;      /* (degree C) * 10, UNKNOWN, UNSUPPORTED */
    _Int32t     maximum_temperature;      /* (degree C) * 10, UNKNOWN, UNSUPPORTED */
    _Uint32t    rsvd[2];
} CAM_DEVICE_TEMPERATURE;

#define CAM_MODULE_CAM		0x01          /* target cam */
#define CAM_MODULE_PDRV		0x02          /* target peripheral driver */
#define CAM_MODULE_SIM		0x03          /* target sim */
#define CAM_MODULE_BLK		0x04          /* target ioblk */
#define CAM_MODULE_FSYS		0x05          /* target filesystem */
typedef struct _cam_verbosity {
	_Uint32t		flags;
	_Uint32t		modules;
	_Uint32t		verbosity;
	_Uint32t		rsvd[5];
} CAM_VERBOSITY;

typedef struct _cam_timeout {
#define CAM_TIMEOUT_OP_GET	0x00
#define CAM_TIMEOUT_OP_SET	0x01
	_Uint32t		op;
	_Uint32t		g1_timeout;
	_Uint32t		g2_timeout;
	_Uint32t		g3_timeout;
	_Uint32t		rw_timeout;
	_Uint32t		rsvd[5];
} CAM_TIMEOUT;

typedef struct _cam_smmu_map {
	_Uint32t	flags;
	_Uint32t	nentries;
	_Uint32t	rsvd[8];
	__FLEXARY(struct smmu_map_entry, entries);
} CAM_SMMU_MAP;

/*
 * COMMON devctl commands (0-99)
 */
#define DCMD_CAM_DEVINFO             __DIOF( _DCMD_CAM,    0, struct cam_devinfo )
#define DCMD_CAM_EJECT_MEDIA         __DION( _DCMD_CAM,    1 )
#define DCMD_CAM_LOAD_MEDIA          __DION( _DCMD_CAM,    2 )
#define DCMD_CAM_RESERVE_UNIT        __DION( _DCMD_CAM,    3 )
#define DCMD_CAM_RELEASE_UNIT        __DION( _DCMD_CAM,    4 )
#define DCMD_CAM_MEDIA_REMOVABLE     __DIOT( _DCMD_CAM,    5, struct media_removable )
#define DCMD_CAM_SCAN_BUS            __DION( _DCMD_CAM,    6 )
#define DCMD_CAM_PASS_THRU           __DIOTF( _DCMD_CAM,   7, struct cam_pass_thru )
#define DCMD_CAM_VERIFY_MEDIA        __DION( _DCMD_CAM,    8 )
#define DCMD_CAM_DEVICE_TEMPERATURE  __DIOF( _DCMD_CAM,    9, struct _cam_device_temperature )
#define DCMD_CAM_VERBOSITY           __DIOT( _DCMD_CAM,    10, struct _cam_verbosity )
#define DCMD_CAM_DEV_SERIAL_NUMBER   __DIOF( _DCMD_CAM,    11, char[256] )
#define DCMD_CAM_TIMEOUT             __DIOTF( _DCMD_CAM,   12, struct _cam_timeout )
#define DCMD_CAM_SMMU_MAP            __DIOT( _DCMD_CAM,    13, struct _cam_smmu_map )
#define DCMD_CAM_ICE_TRANS_INFO_SET  __DIOT( _DCMD_CAM,    14, struct _ice_trans_info_params )

/* ICE (Inline Crypto Engine) payload */
typedef struct _ice_trans_info_params {
	_Uint64t	start_lba;
	_Uint64t	end_lba;
	_Uint32t	arg1;
	_Uint32t	arg2;
	_Uint32t	arg3;
	_Uint32t	arg4;
	_Uint32t	arg5;
	_Uint32t	arg6;
} ICE_TRANS_INFO;

typedef struct _io_vu_data {
	_Int32t		type;				/* vendor defined type, DCMD */
	_Uint32t	nbytes;				/* number of vendor data bytes */
	_Uint32t	rsvd[2];			/* reserved */
	void		*data;				/* pointer to vendor data */
} IO_VU_DATA;

/*
 * DISK devctl commands (25-49)
 */

#define DCMD_CAM_REASSIGN			__DIOT( _DCMD_CAM,    25 )
#define DCMD_CAM_DATA_SET_MGNT      __DIOT( _DCMD_CAM,   26, struct _data_set_mgnt )

/*
 * CDROM devctl commands (50-74)
 */
#define DCMD_CAM_CDROMPAUSE         __DION(_DCMD_CAM,  50)
#define DCMD_CAM_CDROMRESUME        __DION(_DCMD_CAM,  51)
#define DCMD_CAM_CDROMPLAYMSF       __DIOT(_DCMD_CAM,  52, struct _cdrom_playmsf)
#define DCMD_CAM_CDROMPLAYTRKIND    __DIOT(_DCMD_CAM,  53, struct _cdrom_playti)
#define DCMD_CAM_CDROMREADTOC       __DIOTF(_DCMD_CAM, 54, struct _cdrom_read_toc)
#define DCMD_CAM_CDROMLASTSESSION   __DIOTF(_DCMD_CAM,  55, struct _cdrom_read_toc)
#define DCMD_CAM_CDROMSTOP          __DION(_DCMD_CAM,  56)
#define DCMD_CAM_CDROMSTART         __DION(_DCMD_CAM,  57)
#define DCMD_CAM_CDROMREADVOL       __DIOF(_DCMD_CAM,  58, struct _cdrom_volume)
#define DCMD_CAM_CDROMSETVOL        __DIOT(_DCMD_CAM,  59, struct _cdrom_volume)
#define DCMD_CAM_CDROMSUBCHNL       __DIOTF(_DCMD_CAM, 60, union _cdrom_subch_data)
#define DCMD_CAM_CDROMREAD          __DIOTF(_DCMD_CAM, 61, struct _cdrom_raw_read)
#define DCMD_CAM_CDROM_SET_PARAM    __DIOT(_DCMD_CAM,  62, struct _cdrom_param)
#define DCMD_CAM_CDROM_EXCHANGE     __DIOT(_DCMD_CAM,  63, struct _cdrom_exchange)
#define DCMD_CAM_CDROM_MECHANISM_STATUS __DIOTF(_DCMD_CAM, 64, struct _cdrom_mechanism_status)
#define DCMD_CAM_CDROM_TEXT         __DIOF(_DCMD_CAM,  65, struct _cdrom_cd_text)
#define DCMD_CAM_CDROM_SCAN         __DIOT(_DCMD_CAM,  66, struct _cdrom_scan)

/*
 * TAPE devctl commands (75-99)
 */

#define DCMD_CAM_TAPE_REWIND       __DION(_DCMD_CAM, 75)
#define DCMD_CAM_TAPE_RETENSION    __DION(_DCMD_CAM, 76)
#define DCMD_CAM_TAPE_SPACE        __DIOT(_DCMD_CAM, 77, struct _tape_space)
#define DCMD_CAM_TAPE_WRITEFM      __DIOT(_DCMD_CAM, 78, struct _tape_writefm)
#define DCMD_CAM_TAPE_LOCATE       __DIOT(_DCMD_CAM, 79, struct _tape_locate)
#define DCMD_CAM_TAPE_POSITION     __DIOF(_DCMD_CAM, 80, struct _tape_position)

/*
 * OPTICAL devctl commands (100-124) 
 */

/*
 * SCANNER devctl commands (125-149) 
 */

/*
 * COMM devctl commands (150-174) 
 */

/*
 * WORM devctl commands (175-199) 
 */

/*
 * CHANGER devctl commands (200-224) 
 */

/*
 * SIM devctls - each SIM gets 100
 */
#define _CAM_SIM						2000
#define _SIM_EIDE						_CAM_SIM + ( 0 * 100)
#define _SIM_RAM						_CAM_SIM + ( 1 * 100)
#define _SIM_UMASS						_CAM_SIM + ( 2 * 100)
#define _SIM_SERCD						_CAM_SIM + ( 3 * 100)
#define _SIM_MMCSD						_CAM_SIM + ( 4 * 100)
#define _SIM_SDMMC						_CAM_SIM + ( 5 * 100)
#define _SIM_UFS						_CAM_SIM + ( 6 * 100)
#define _SIM_NVME						_CAM_SIM + ( 7 * 100)
#define _SIM_RPMB						_CAM_SIM + ( 8 * 100)

#include <_packpop.h>

#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/devb/cam/public/sys/dcmd_cam.h $ $Rev: 915926 $")
#endif
