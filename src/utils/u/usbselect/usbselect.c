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
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <sys/mman.h>
#include <hw/inout.h>
#include <aarch64/mx8x.h>
#include <aarch64/imx8_common/imx_gpio.h>

#define GPIO_BASE	IMX_GPIO0_BASE
#define GPIO_NUM	30		/* IO30 */

void
usage(char *name)
{
	fprintf(stderr, "Usage: %s [typea | otg]\n", name);
	fprintf(stderr, "  where typea selects the Type-A host port for use\n");
	fprintf(stderr, "        otg selects the micro-B OTG port for use\n");
}

int
select(char *name, int typea)
{
	uint32_t dr, dir;
	uintptr_t vbase;

	vbase = mmap_device_io(0x100, GPIO_BASE);
	if (vbase == MAP_DEVICE_FAILED) {
		fprintf(stderr, "%s: failed to map I/O\n", name);
		return 1;
	}

	dr = in32(vbase + IMX_GPIO_DR);
	if (typea)
		dr &= ~(1 << GPIO_NUM);
	else
		dr |= (1 << GPIO_NUM);

	out32(vbase + IMX_GPIO_DR, dr);

	/* make the GPIO an output */
	dir = in32(vbase + IMX_GPIO_GDIR);
	out32(vbase + IMX_GPIO_GDIR, dir | (1 << GPIO_NUM));

	munmap_device_io(vbase, 0x100);

	return 0;
}

int
main(int argc, char *argv[]) {

	if (argc != 2) {
		usage(argv[0]);
		exit(1);
	}

	if (!strcmp(argv[1], "typea")) {
		select(argv[0], 1);
	} else if (!strcmp(argv[1], "otg")) {
		select(argv[0], 0);
	} else {
		usage(argv[0]);
		exit(1);
	}

	return 0;
}
