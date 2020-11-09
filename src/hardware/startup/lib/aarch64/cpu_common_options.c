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

#include "startup.h"

static char *erra_opts[] = {
		"meltdown",
		"~meltdown",
		"spectrev2",
		"~spectrev2"
};

int
cpu_handle_common_option(int opt)
{
	char *cp, *value;

	switch(opt) {
	case 'X':
		{
			nxfer_slots = atoi(optarg);
			//make sure it's power of 2.
			nxfer_slots = (1u << (sizeof(int) * 8 - __builtin_clz(nxfer_slots) - 1));
			if(nxfer_slots < 2 || nxfer_slots > 256) {
				crash("Unsupported number of slots=%d, valid numbers are [2 .. 256]\n", nxfer_slots);
			}
		}
		return 1;
	case 'E': ; // erratas
		cp = optarg;
		while(*cp != '\0') {
			switch(getsubopt(&cp, erra_opts, &value)) {
			case 0:
				meltdown_active = 1;
				break;
			case 1:
				meltdown_active = 0;
				break;
			case 2:
				spectre_v2_active = 1;
				break;
			case 3:
				spectre_v2_active = 0;
				break;
			default:
				break;
			}
		}
		return 1;
	default:
		return 0;
	}
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/startup/lib/aarch64/cpu_common_options.c $ $Rev: 886103 $")
#endif
