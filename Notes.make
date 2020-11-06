To build this BSP:

In the SDP directory, run:
	source qnxsdp-env.sh
The location of this directory is ~/qnxNNN for the default QNX Software
Center installation.  For example, ~/qnx710 for SDP 7.1.


In the BSP directory, run:
	make
	make -C images
Due to QNX Makefile limitations, you may need to remove images/ifs*.bin
before the second make lest IFS not be recreated (or run "make clean").

The resulting binaries are:
	IPL: images/ipl-imx8qxp-cpu_c0.imx

	IFS: images/ifs-imx8qxp-cpu-graphics.bin or
	     images/ifs-imx8qxp-cpu.bin

Create a partition on an SD card.  Format it with a DOS filesystem with the
label "boot".  Place the IFS file on that filesystem, renamed QNX-IFS.

Place IPL on the SD card using:
	sudo dd if=images/ipl-imx8qxp-cpu_c0.imx of=/dev/sdN bs=1k seek=32
where sdN corresponds to the device for the SD card.
