/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems.
 * Copyright 2017 NXP
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

static int _spi_register_interface(void *data)
{
	spi_dev_t		*dev = data;
	SPIDEV			*drvhdl;
	resmgr_attr_t	rattr;
	char			devname[PATH_MAX + 1];

	if ((drvhdl = dev->funcs->init(dev, dev->opts)) == NULL) {
		free(dev->opts);
		return (!EOK);
	}

	dev->drvhdl = drvhdl;

	/* set up i/o handler functions */
	memset(&rattr, 0, sizeof(rattr));
	rattr.nparts_max   = SPI_RESMGR_NPARTS_MIN;
	rattr.msg_max_size = SPI_RESMGR_MSGSIZE_MIN;

	iofunc_attr_init(&drvhdl->attr, S_IFCHR | 0666, NULL, NULL);
	drvhdl->attr.mount = &_spi_mount;

	/* register device name */
	snprintf(devname, PATH_MAX, "/dev/spi%d", dev->devnum);
	if (-1 == (dev->id = resmgr_attach(dev->dpp, &rattr, devname, _FTYPE_ANY, 0,
					&_spi_connect_funcs, &_spi_io_funcs, (void *)drvhdl))) {
		perror("resmgr_attach() failed");
		goto failed1;
	}

	resmgr_devino(dev->id, &drvhdl->attr.mount->dev, &drvhdl->attr.inode);

	if ((dev->ctp = dispatch_context_alloc(dev->dpp)) != NULL)
		return (EOK);

	perror("dispatch_context_alloc() failed");

	resmgr_detach(dev->dpp, dev->id, _RESMGR_DETACH_ALL);
failed1:
	dev->funcs->fini(drvhdl);

	return (!EOK);
}

static void* _spi_driver_thread(void *data)
{
	spi_dev_t	*dev = data;

	if (_spi_register_interface(data) != EOK)
		return NULL;

	while (1) {
		if ((dev->ctp = dispatch_block(dev->ctp)) != NULL)
			dispatch_handler(dev->ctp);
		else
			break;
	}

	return NULL;
}

int _spi_create_instance(spi_dev_t *dev)
{
	pthread_attr_t		pattr;
	struct sched_param	param;
	pthread_t			tid;
	int					policy;

	if (NULL == (dev->dpp = dispatch_create())) {
		perror("dispatch_create() failed");
		goto failed0;
	}

	pthread_attr_init(&pattr);
	pthread_attr_setschedpolicy(&pattr, SCHED_RR);
	pthread_getschedparam(pthread_self(), &policy, &param);
	param.sched_priority = 21;
	pthread_attr_setschedparam(&pattr, &param);
	pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);

	/* Create thread for this interface */
	if (pthread_create(&tid, &pattr, _spi_driver_thread, dev) != EOK) {
		perror("pthread_create() failed");
		goto failed1;
	}
	pthread_setname_np(tid, "spi_slave_thread");

	return (EOK);

failed1:
	dispatch_destroy(dev->dpp);
failed0:

	return (-1);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/spi/slave/_spi_create_instance.c $ $Rev: 855721 $")
#endif
