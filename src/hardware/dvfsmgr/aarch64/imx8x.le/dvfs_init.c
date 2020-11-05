/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
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


#include "dvfs.h"

int dvfs_pre_init(void * handle)
{
	dvfs_t  *dev = (dvfs_t *)handle;

	if (init_cpu_info(dev) != EOK) {
		DVFS_ERROR("Could not initialize cpu info");
		return EFAULT;
	}

	return EOK;
}

int dvfs_init(void *handle)
{
	int			ret = EOK;
	dvfs_t		*dev = (dvfs_t *)handle;
	cpustate_t	*cpustate = NULL;

	/* Create Threads to do the CPU accounting */
	cpustate = dev->cpustate;
	if (dvfs_create_thread_cpu_act(cpustate) != EOK) {
		DVFS_ERROR("Failed to create thread for CPU load accounting");
		goto Error;
	}

Exit:
	return ret;

Error:
	ret = EFAULT;
	dvfs_fini(dev);
	goto Exit;
}

int dvfs_therm_init(__attribute__((unused)) void *handle)
{
	return EOK;
}

void dvfs_fini(void *handle)
{
	dvfs_t		*dev = (dvfs_t *)handle;
	cpustate_t	*cpustate = NULL;

	if (dev==NULL) {
		return ;
	}

	cpu_info_fini(dev);

	cpustate = dev->cpustate;
	pthread_cancel(cpustate->tid);
	DVFS_INFO(dev, 5, "Destroy the CPU load accounting thread (tid%d)", cpustate->tid);
}

int get_pwr_levels_count(__attribute__((unused)) void *handle)
{
	return get_cpu_pwr_levels_count();
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/dvfsmgr/aarch64/imx8x.le/dvfs_init.c $ $Rev: 894097 $")
#endif
