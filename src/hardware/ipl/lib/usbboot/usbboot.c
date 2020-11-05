/*
 * $QNXLicenseC:
 * Copyright 2016 QNX Software Systems.
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

#include <usbboot.h>
#include <usb.h>

#define USBBOOT_CONFIGURATION					1
#define USBBOOT_MPS_HIGH_SPEED				(0x0200)
#define USBBOOT_MPS_FULL_SPEED					(0x0040)
#define USBBOOT_MPS_TX_ENDPOINT				(0x0040)

#define DEVICE_VENDOR_ID						0x1234
#define DEVICE_PRODUCT_ID						0xfff1
#define DEVICE_BCD								0x0100

#define CFG_DESC_LEN	(USB_DESC_SIZE_CONFIG + USB_DESC_SIZE_INTERFACE + \
		USB_DESC_SIZE_ENDPOINT + USB_DESC_SIZE_ENDPOINT)

#define EP_BUFFER_SIZE							4096
#define DOWNLOAD_CHUNK_SIZE					32*1024

typedef struct _sparse {
	uint32_t		signature; /* 0xed26ff3a */
	uint16_t		major;
	uint16_t		minor;
	uint16_t		sparse_header_sz;
	uint16_t		run_header_sz;
	uint32_t		blk_sz;
	uint32_t		num_blks;	/* number of blocks in output image */
	uint32_t		num_runs;	/* number of runs in input image */
	uint32_t		crc32;
} sparse_t;

#define SPARSE_SIGNATURE	0xed26ff3a
#define SPARSE_MAJOR		1

#define RUN_TYPE_RAW			0xcac1
#define RUN_TYPE_FILL			0xcac2
#define RUN_TYPE_SKIP			0xcac3

typedef struct _run{
	uint16_t		run_type;
	uint16_t		reserved0;
	uint32_t		num_blks;	/* in blocks in output image */
	uint32_t		total_bytes;		/* in bytes of input image including header and data */
} run_t;

typedef struct _cmd {
	char *cmd;
	void (*cb)(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb);
}cmd_t;

static void usb_boot_cmd_handler(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb);
static void cmd_oem(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb);
static void cmd_erase(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb);
static void cmd_flash(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb);
static void cmd_boot(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb);
static void cmd_download(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb);
static void cmd_reboot(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb);
static void cmd_getvar(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb);

static usb_string_t default_strings[] = {
	{ USB_STR_PRODUCT_ID,		"Product" },
	{ USB_STR_SERIAL_ID,			"1234567890" },
	{ USB_STR_CONFIG_ID,			"Fastboot" },
	{ USB_STR_INTERFACE_ID,		"Fastboot" },
	{ USB_STR_MANUFACTURER_ID,	"Manufacturer" },
	{  }
};

static usbd_endpoint_descriptor_t fs_ep_in = {
	.bLength            = USB_DESC_SIZE_ENDPOINT,
	.bDescriptorType    = USB_DESC_ENDPOINT,
	.bEndpointAddress   = USB_DIRECTION_HOST | 1, /* IN */
	.bmAttributes       = USB_ATTRIB_BULK,
	.wMaxPacketSize     = USBBOOT_MPS_TX_ENDPOINT,
	.bInterval          = 0x00,
};

static usbd_endpoint_descriptor_t fs_ep_out = {
	.bLength		= USB_DESC_SIZE_ENDPOINT,
	.bDescriptorType	= USB_DESC_ENDPOINT,
	.bEndpointAddress	= USB_DIRECTION_DEVICE | 2, /* OUT */
	.bmAttributes		= USB_ATTRIB_BULK,
	.wMaxPacketSize	= USBBOOT_MPS_FULL_SPEED,
	.bInterval		= 0x00,
};

static usbd_endpoint_descriptor_t hs_ep_out = {
	.bLength		= USB_DESC_SIZE_ENDPOINT,
	.bDescriptorType	= USB_DESC_ENDPOINT,
	.bEndpointAddress	= USB_DIRECTION_DEVICE | 2, /* OUT */
	.bmAttributes		= USB_ATTRIB_BULK,
	.wMaxPacketSize	= USBBOOT_MPS_HIGH_SPEED,
	.bInterval		= 0x00,
};

static usbd_configuration_descriptor_t config_desc = {
	.bLength		= USB_DESC_SIZE_CONFIG,
	.bDescriptorType	= USB_DESC_CONFIGURATION,
	.wTotalLength		= CFG_DESC_LEN,
	.bNumInterfaces	= 1,
	.bConfigurationValue = USBBOOT_CONFIGURATION,
	.iConfiguration		= USB_STR_CONFIG_ID,
	.bmAttributes		= 0xc0,
	.bMaxPower		= 0x32,
};

static usbd_interface_descriptor_t interface_desc = {
	.bLength		= USB_DESC_SIZE_INTERFACE,
	.bDescriptorType	= USB_DESC_INTERFACE,
	.bInterfaceNumber	= 0x00,
	.bAlternateSetting	= 0x00,
	.bNumEndpoints	= 0x02,
	.bInterfaceClass	= 0xff,
	.bInterfaceSubClass	= 0x42,
	.bInterfaceProtocol	= 0x03,
	.iInterface		= USB_STR_INTERFACE_ID,
};

static cmd_t cmds[] = {
	{"getvar:", cmd_getvar },
	{"reboot", cmd_reboot},
	{"download:", cmd_download},
	{"boot", cmd_boot	},
	{"flash", cmd_flash},
	{"erase",cmd_erase},
	{"oem", cmd_oem},
};

usbboot_context_t	*ctx;
usb_dc_t		*udc;
usb_endpoint_t	*ep_in;
usb_transfer_t	*urb_in;
usb_endpoint_t	*ep_out;
usb_transfer_t	*urb_out;
uint8_t		*urb_out_buf;
paddr64_t	urb_out_buf_phys;

static size_t total_transfer_size;
static size_t current_transfer_bytes;

void strultostr ( char *cp, unsigned long n) {
	static char af[16] = "0123456789ABCDEF";
	int i;

	for(i = 0; i < 8; i++){
		cp [8-i-1] = af [n & 0xf];
		n >>= 4;
	}
	cp [8] = 0;
}

int strcmp_first(const char *s1, const char *s2)
{
	return( strncmp(s1, s2, strlen(s1)) );
}

static void usb_boot_urb_in_compl(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	if (urb->status){
		LOG1H(udc, 0, "bas urb status: ", urb->status);
	}
}

static int usb_boot_cmd_reply(usb_dc_t *udc, const char *buffer)
{
	int ret;
	int len = strlen(buffer);

	if(urb_in->complete_cbf == NULL){
		urb_in->complete_cbf = usb_boot_urb_in_compl;
	}

	if((uint8_t *)buffer != urb_in->buffer ){
		memcpy( (void *)urb_in->buffer, (void *)buffer, len );
	}

	urb_in->buffer_len = len;

	if ( (ret = usb_ep_transfer(udc, ep_in, urb_in, PIPE_FLAGS_TOKEN_IN)) != EOK ) {
		LOG1H(udc, 0, "fb_write_str: Error on queue =", ret);
	}

	return( ret );
}

const char *usb_boot_get_usb_string(unsigned int id)
{
	usb_string_t *us = NULL;

	if(ctx->dev_strings){
		for(us = ctx->dev_strings; us && us->str; us++){
			if(us->id == id){
				return( us->str );
			}
		}
	}

	for(us = default_strings; us && us->str; us++){
		if(us->id == id){
			return( us->str );
		}
	}

	return( NULL );
}

static void cmd_oem(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	char *cmdbuf = (char *)urb->buffer;
	char *replybuf = (char *)urb_in->buffer;

	if(ctx->oem){
		strcpy(replybuf, "OKAY");
		ctx->oem(cmdbuf+4, replybuf);
	}
	else{
		strcpy(replybuf, "FAIL command not implemented");
	}

	usb_boot_cmd_reply(udc, replybuf);
}

static void cmd_erase(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	char *cmdbuf = (char *)urb->buffer;
	char *replybuf = (char *)urb_in->buffer;

	if(ctx->erase){
		strcpy(replybuf, "OKAY");
		ctx->erase(cmdbuf+6, replybuf);
	}
	else{
		strcpy(replybuf, "FAIL command not implemented");
	}

	usb_boot_cmd_reply(udc, replybuf);
}

static void cmd_flash(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	char *cmdbuf = (char *)urb->buffer;
	char *replybuf = (char *)urb_in->buffer;

	if(ctx->flash){
		strcpy(replybuf, "OKAY");
		ctx->flash(cmdbuf+6, replybuf);
	}
	else{
		strcpy(replybuf, "FAIL command not implemented");
	}

	usb_boot_cmd_reply(udc, replybuf);
}

static void cmd_do_boot(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	//clean up the usb resources
	usb_boot_shutdown();

	// boot from the download address
	ctx->boot(ctx->download_buffer, current_transfer_bytes);

	//in case boot some how failed, reboot;
	if( ctx->reboot ){
		ctx->reboot();
	}
}

static void cmd_boot(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	if(ctx->boot){
		urb_in->complete_cbf = cmd_do_boot;
		usb_boot_cmd_reply(udc, "OKAY");
	}
	else{
		usb_boot_cmd_reply(udc, "FAIL command not implemented");
	}
}

static void cmd_do_download(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	char *replybuf = (char *)urb_in->buffer;

	if (urb->status != 0) {
		LOG1H(udc, 0, "Bad status: %d", urb->status);
		return;
	}

	current_transfer_bytes += urb->actual_len;

	LOG1H(udc, 4, "downloaded bytes ", current_transfer_bytes);

	if (current_transfer_bytes < total_transfer_size) {
		urb->buffer = ctx->download_buffer + current_transfer_bytes;
		urb->buffer_paddr = ctx->download_buffer_phys + current_transfer_bytes;
		urb->buffer_len = min(DOWNLOAD_CHUNK_SIZE, total_transfer_size-current_transfer_bytes);
	}
	else{
		total_transfer_size = 0;
		urb->complete_cbf = usb_boot_cmd_handler;
		urb->buffer_len = EP_BUFFER_SIZE;
		urb->buffer = urb_out_buf;
		urb->buffer_paddr = urb_out_buf_phys;

		strcpy(replybuf, "OKAY");

		usb_boot_cmd_reply(udc, replybuf);

		LOG1H(udc, 3, "download finished bytes ", current_transfer_bytes);
	}

	usb_ep_transfer(udc, ep, urb, PIPE_FLAGS_TOKEN_OUT);
}

static void cmd_download(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	char *cmdbuf = (char *)urb->buffer;
	char *replybuf = (char *)urb_in->buffer;
	char str_num[16];

	strsep(&cmdbuf, ":");

	total_transfer_size = strhextoul(cmdbuf);
	current_transfer_bytes = 0;

	LOG1H(udc, 3, "Starting download bytes:", total_transfer_size);

	if (total_transfer_size > ctx->download_buffer_size || !total_transfer_size) {
		total_transfer_size = 0;
		strcpy(replybuf, "FAIL invalid data size");
	}
	else {
		urb->complete_cbf = cmd_do_download;
		urb->buffer_len = min(DOWNLOAD_CHUNK_SIZE, total_transfer_size);
		urb->buffer = ctx->download_buffer;
		urb->buffer_paddr = ctx->download_buffer_phys;

		if(ctx->dcache_op){
			ctx->dcache_op((unsigned long)ctx->download_buffer, total_transfer_size, CACHE_OP_INVAL);
		}
		else if(ctx->dcache_op_all){
			ctx->dcache_op_all(CACHE_OP_INVAL);
		}

		strultostr(str_num, total_transfer_size);
		strcpy(replybuf, "DATA");
		strncat(replybuf, str_num, 64);
	}

	usb_boot_cmd_reply(udc, replybuf);
}

static void cmd_do_reboot(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	//clean up the usb resources
	usb_boot_shutdown();

	ctx->reboot();
}

static void cmd_reboot(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	if( ctx->reboot ){
		urb_in->complete_cbf = cmd_do_reboot;
		usb_boot_cmd_reply(udc, "OKAY");
	}
	else{
		usb_boot_cmd_reply(udc, "FAIL command not implemented");
	}
}

static void cmd_getvar(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	char *cmdbuf = (char *)urb->buffer;
	char *replybuf = (char *)urb_in->buffer;
	char str_num[16];
	int id = -1;

	strcpy(replybuf, "OKAY");
	strsep(&cmdbuf, ":");

	if (!cmdbuf) {
		usb_boot_cmd_reply(udc, "FAIL missing variable");
		return;
	}

	if (!strcmp_first("version", cmdbuf)) {
		strcpy(str_num, USBBOOT_VERSION);
		strncat(replybuf, str_num, 64);
	}
	else if (!strcmp_first("downloadsize", cmdbuf)
		||!strcmp_first("max-download-size", cmdbuf)) {
		strultostr(str_num, ctx->download_buffer_size);
		strncat(replybuf, str_num, 64);
	}
	else if (!strcmp_first("product", cmdbuf)) {
		id = USB_STR_PRODUCT_ID;
	}
	else if (!strcmp_first("serialno", cmdbuf)) {
		id = USB_STR_SERIAL_ID;
	}
	else if( ctx->getvar ) { // if board support specific variables,
		ctx->getvar(cmdbuf, replybuf, urb_in->buffer_len);
	}
	else{
		strcpy(replybuf, "FAIL variable not implemented");
	}

	if ( id != -1 ) {
		const char *str = usb_boot_get_usb_string(id);
		if( str ){
			strncat(replybuf, str, 64);
		}
		else{
			strcpy(replybuf, "FAIL value not set");
		}
	}

	usb_boot_cmd_reply(udc, replybuf);
}

static void usb_boot_cmd_handler(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	int	i;
	char *cmdbuf = (char *)urb->buffer;
	void (*func_cmd)(usb_dc_t* udc, usb_endpoint_t *ep, usb_transfer_t *urb) = NULL;

	if (urb->actual_len < urb->buffer_len){
		//null terminate
		cmdbuf[urb->actual_len] = 0;
	}
	else{
		usb_boot_cmd_reply(udc, "FAIL buffer overflow");
	}

	LOGS(udc, 1, cmdbuf);

	for (i = 0; i < (sizeof(cmds) / sizeof((cmds)[0])); i++) {
		if (!strcmp_first(cmds[i].cmd, cmdbuf)) {
			func_cmd = cmds[i].cb;
			break;
		}
	}

	if (!func_cmd){
		usb_boot_cmd_reply(udc, "FAIL unknown command");
	}
	else{
		func_cmd(udc, ep, urb);
	}

	if (urb->status == 0) {
		*cmdbuf = '\0';
		usb_ep_transfer(udc, ep, urb, PIPE_FLAGS_TOKEN_OUT);
	}
}

int usb_boot_string_descriptor(int id, uint8_t *buf)
{
	const char	*s;
	uint8_t		c;
	uint16_t		uchar;
	uint16_t		*cp;
	int			len, slen;

	if (id == 0) {
		uint16_t langid = ( ctx->langid ) ? ctx->langid : 0x0409;
		buf[0] = 4;
		buf[1] = USB_DESC_STRING;
		buf[2] = (uint8_t) langid;
		buf[3] = (uint8_t) (langid >> 8);
		return 4;
	}

	if ( (s = usb_boot_get_usb_string(id)) == NULL){
		return( -1 );
	}

	slen = min((size_t) 126, strlen(s));
	len = 0;
	cp = (uint16_t *)&buf[2];
	while (slen != 0 && (c = (uint8_t) *s++) != 0) {
		if ((c & 0x80)) {
			// we only deal with simple ones.
			LOG1H(udc, 0, "invalid utf8 ", c);
			return( -1 );
		}
		else{
			uchar = c;
		}
		UNALIGNED_PUT16(cp, uchar);
		cp++;
		len++;
		slen--;
	}

	len = len * 2 + 2; //include header 2 bytes
	buf[0] = len;
	buf[1] = USB_DESC_STRING;

	return( len );
}

static void usb_boot_ctrl_status_compl(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	if (urb->status){
		LOG1H(udc, 0, "ep0 status ", urb->status);
	}
}

static void usb_boot_ctrl_data_compl(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	if (urb->status){
		LOG1H(udc, 0, "ep0 status ", urb->status);
	}
	else{
		urb->buffer_len = 0;
		urb->complete_cbf = usb_boot_ctrl_status_compl;
		//reverse the direction
		urb->flags = ( urb->flags & PIPE_FLAGS_TOKEN_OUT)? PIPE_FLAGS_TOKEN_IN : PIPE_FLAGS_TOKEN_OUT;
		usb_ep_transfer(udc, ep, urb, PIPE_FLAGS_TOKEN_STATUS | urb->flags);
	}
}

static int usb_boot_get_descriptor(usb_dc_t *udc, usb_setup_packet_t *setup)
{
	int		ret = EOK;
	int		remains, total, len;
	uint16_t	val = setup->wValue >> 8;
	uint8_t	*buf;

	LOG1H(udc, 5, "usb_boot_get_descriptor: wValue=", setup->wValue);

	udc->ep0_urb->complete_cbf = usb_boot_ctrl_data_compl;

	switch (val) {
		case USB_DESC_DEVICE:
			{
				usbd_device_descriptor_t *desc = (usbd_device_descriptor_t *)udc->ep0_urb->buffer;
				desc->bLength = USB_DESC_SIZE_DEVICE;
				desc->bDescriptorType = USB_DESC_DEVICE;
				desc->bcdUSB = ENDIAN_LE16(0x200);
				desc->bDeviceClass = 0;
				desc->bDeviceSubClass = 0;
				desc->bDeviceProtocol = 0;
				desc->bMaxPacketSize0 = 0x40;
				desc->idVendor = ENDIAN_LE16(ctx->dev_vid);
				desc->idProduct= ENDIAN_LE16(ctx->dev_did);
				desc->bcdDevice = ENDIAN_LE16(DEVICE_BCD);
				desc->iManufacturer = USB_STR_MANUFACTURER_ID;
				desc->iProduct = USB_STR_PRODUCT_ID;
				desc->iSerialNumber = USB_STR_SERIAL_ID;
				desc->bNumConfigurations = 1;
				udc->ep0_urb->buffer_len = min(setup->wLength, USB_DESC_SIZE_DEVICE);
				ret = usb_ep_transfer(udc, udc->ep0, udc->ep0_urb, PIPE_FLAGS_TOKEN_IN);
			}
			break;

		case USB_DESC_CONFIGURATION:
			buf = udc->ep0_urb->buffer;
			remains= min(setup->wLength, 512);
			total = 0;

			if( remains <=0 ) goto done;

			len = min(remains, USB_DESC_SIZE_CONFIG);
			memcpy(buf + total, &config_desc, len);
			total += len;
			remains -= len;
			if( remains <=0 ) goto done;

			len = min(remains, USB_DESC_SIZE_INTERFACE);
			memcpy(buf + total, &interface_desc, len);
			total += len;
			remains -= len;
			if( remains <=0 ) goto done;

			len = min(remains, USB_DESC_SIZE_ENDPOINT);
			memcpy(buf + total, &fs_ep_in, len);
			total += len;
			remains -= len;
			if( remains <=0 ) goto done;

			len = min(remains, USB_DESC_SIZE_ENDPOINT);
			if (udc->speed == USB_DEVICE_HIGH_SPEED)
				memcpy(buf + total, &hs_ep_out, len);
			else
				memcpy(buf + total, &fs_ep_out, len);
			total += len;

done:
			udc->ep0_urb->buffer_len = total;
			ret = usb_ep_transfer(udc, udc->ep0, udc->ep0_urb, PIPE_FLAGS_TOKEN_IN);
			break;

		case USB_DESC_STRING:
			buf = udc->ep0_urb->buffer;
			if ( (ret = usb_boot_string_descriptor( setup->wValue & 0xff, buf)) < 0 ){
				break;
			}

			udc->ep0_urb->buffer_len = ret;
			ret = usb_ep_transfer(udc, udc->ep0, udc->ep0_urb, PIPE_FLAGS_TOKEN_IN);
			break;

		default:
			ret = -EINVAL;
	}

	return( ret );
}

static int usb_boot_get_configuration(usb_dc_t *udc, usb_setup_packet_t *setup)
{
	if (setup->wLength == 0){
		return( -1 );
	}

	udc->ep0_urb->buffer[0] = udc->cfg;
	udc->ep0_urb->buffer_len = 1;
	udc->ep0_urb->complete_cbf = usb_boot_ctrl_data_compl;

	return( usb_ep_transfer(udc, udc->ep0, udc->ep0_urb, PIPE_FLAGS_TOKEN_IN) );
}

static int usb_boot_endpoint_disable(usb_dc_t *udc)
{
	LOGS(udc, 5, "usb_boot_endpoint_disable: ");

	if( ep_in ){
		usb_ep_disable(udc, ep_in);
	}

	if( ep_out ){
		usb_ep_disable(udc, ep_out);
	}

	return( EOK );
}

static int usb_boot_endpoint_enable(usb_dc_t *udc)
{
	int ret;

	LOGS(udc, 5, "usb_boot_endpoint_enable: ");

	if( (ret = usb_ep_enable(udc, &fs_ep_in, &ep_in )) ){
		LOGS(udc, 0, "failed to enable in ep");
		goto error;
	}

	urb_in->buffer_len = EP_BUFFER_SIZE;


	if (udc->speed == USB_DEVICE_HIGH_SPEED)
		ret = usb_ep_enable(udc, &hs_ep_out, &ep_out );
	else
		ret = usb_ep_enable(udc, &fs_ep_out, &ep_out );

	if (ret) {
		LOGS(udc, 0, "failed to enable out ep");
		goto error;
	}

	urb_out->complete_cbf = usb_boot_cmd_handler;
	urb_out->buffer_len = EP_BUFFER_SIZE;

	if( (ret = usb_ep_transfer(udc, ep_out, urb_out, PIPE_FLAGS_TOKEN_OUT)) ){
		goto error;
	}

	return( EOK );

error:
	usb_boot_endpoint_disable(udc);

	return( -1 );
}

static int usb_boot_setup_process(usb_dc_t *udc, usb_setup_packet_t *setup)
{
	if((setup->bRequestType & USB_TYPE_MASK) != USB_TYPE_STANDARD){
		return( -1 );
	}

	if((setup->bRequestType & USB_DIRECTION_HOST) == 0){
		if( (setup->bRequestType & USB_RECIP_MASK) == USB_RECIPIENT_DEVICE
			&& setup->bRequest == USB_SET_CONFIGURATION){
			if(setup->wValue == USBBOOT_CONFIGURATION){
				udc->cfg = USBBOOT_CONFIGURATION;
				usb_boot_endpoint_enable(udc);
				return( EOK );
			}
			else if(setup->wValue == 0){
				udc->cfg = 0;
				usb_boot_endpoint_disable(udc);
				return( EOK );
			}
		}

		if( (setup->bRequestType & USB_RECIP_MASK) == USB_RECIPIENT_INTERFACE
			&& setup->bRequest == USB_SET_INTERFACE){
			return( usb_boot_endpoint_enable(udc));
		}
	}
	else if((setup->bRequestType & USB_RECIP_MASK) == USB_RECIPIENT_DEVICE){
		if(setup->bRequest == USB_GET_DESCRIPTOR){
			return( usb_boot_get_descriptor(udc, setup) );
		}
		if(setup->bRequest == USB_GET_CONFIGURATION){
			return( usb_boot_get_configuration(udc, setup) );
		}
	}

	return( -1 );
}

static int usb_boot_reset(usb_dc_t *udc)
{
	usb_boot_endpoint_disable(udc);

	return( EOK );
}

static int usb_boot_connect(usb_dc_t *udc, uint32_t insertion)
{
	if( insertion ){
	}
	else{
		usb_boot_endpoint_disable(udc);
	}

	return( EOK );
}

int usb_boot_init(usbboot_context_t *tctx)
{
	int ret;

	ctx = tctx;

	udc = (usb_dc_t *)tctx->malloc_buffer;

	udc->base = tctx->base;
	udc->verbose =tctx->verbose;

	//setting up the board specific callbacks.
	udc->init = tctx->dc_init;
	udc->bs_init = tctx->bs_init;
	udc->bs_fini = tctx->bs_fini;
	udc->bs_phy_reset = tctx->bs_phy_reset;
	udc->bs_delay_ms = tctx->bs_delay_ms;
	udc->bs_delay_ns = tctx->bs_delay_ns;
	udc->bs_bus_sync = tctx->bs_bus_sync;

	udc->malloc_buffer = tctx->malloc_buffer + sizeof(usb_dc_t);
	udc->malloc_buffer_phys = tctx->malloc_buffer_phys + sizeof(usb_dc_t);
	udc->malloc_size = tctx->malloc_size - sizeof(usb_dc_t);

	udc->num_endpoints = 3; //need endpoint 0, 1, 2

	ep_in = ep_out = NULL;

	if(ctx->dev_vid == 0 || ctx->dev_did == 0){
		ctx->dev_vid = DEVICE_VENDOR_ID;
		ctx->dev_did = DEVICE_PRODUCT_ID;
	}

	if ( (ret = usb_init_udc(udc)) != EOK) {
		LOGS(udc, 0, "usb_init_udc failed");
		return( -1 );
	}

	//setup the device level callbacks
	udc->setup = usb_boot_setup_process;
	udc->connect = usb_boot_connect;
	udc->reset = usb_boot_reset;

	if((ret = usb_calloc(udc, (void **)&urb_out, (paddr64_t *)NULL, 0, sizeof(usb_transfer_t)))!= EOK){
		goto err;
	}
	if((ret = usb_malloc(udc, (void **)&urb_out->buffer, &urb_out->buffer_paddr, 0, EP_BUFFER_SIZE))!= EOK){
		goto err;
	}

	if((ret = usb_calloc(udc, (void **)&urb_in, (paddr64_t *)NULL, 0, sizeof(usb_transfer_t)))!= EOK){
		goto err;
	}
	if((ret = usb_malloc(udc, (void **)&urb_in->buffer, &urb_in->buffer_paddr, 0, EP_BUFFER_SIZE))!= EOK){
		goto err;
	}

	urb_out_buf = urb_out->buffer;
	urb_out_buf_phys = urb_out->buffer_paddr;

	return( 0 );

err:
	usb_boot_shutdown();
	return( -1 );
}

int usb_boot_process(void)
{
	usb_handle_interrupts(udc);

	return( EOK );
}

void usb_boot_shutdown(void)
{
	usb_boot_endpoint_disable(udc);

	usb_exit_udc(udc);
}

int usb_boot_unsparse(void *hdl, unsigned long addr, unsigned blkno, unsigned isize,
	int (*blk_func)(void *, void *, unsigned, unsigned), void *fillBuf, int fillBufSz)
{
	uint32_t	i, j, tlen, sblks;
	uint64_t flen, wsize;
	sparse_t *h = (sparse_t *)addr;
	unsigned saddr = addr;

	if( (h->signature != SPARSE_SIGNATURE)
		|| (h->major != SPARSE_MAJOR)
		|| (h->sparse_header_sz != sizeof(sparse_t))
		|| (h->run_header_sz != sizeof(run_t))) {
		ser_putstr("incompatible sparse header\n");
		return( -1 );
	}

	if( fillBufSz & (512-1) ){
		ser_putstr("Fill buf sz has to be 512 aligned\n");
		return( -1 );
	}

	if( h->blk_sz & (512-1) ){
		ser_putstr("h->blk_sz has to be 512 aligned\n");
		return( -1 );
	}

#if 0
	ser_putstr("sparse, blksz=");
	ser_putdec(h->blk_sz);
	ser_putstr(" num runs=");
	ser_putdec(h->num_runs);
	ser_putstr("\n");
#endif

	addr += h->sparse_header_sz;
	sblks = blkno;

	for (i=0; i < h->num_runs; i++) {
		run_t *run = (run_t *) addr;

		addr += sizeof(run_t);

		switch (run->run_type) {

		case RUN_TYPE_RAW:
			{
				tlen = run->num_blks * h->blk_sz;
				if (run->total_bytes != (tlen + sizeof(run_t))) {
					ser_putstr("bad raw run size\n");
					return( -1 );
				}

				if( (blk_func(hdl, (void *)addr, blkno, tlen>>9)) != 0){
					ser_putstr("sparse blk write failed 0\n");
					return( -1 );
				}

				addr += tlen;
				blkno += tlen>>9;
			}
			break;

		case RUN_TYPE_FILL:
			{
				uint32_t f = *((uint32_t *)addr);
				uint32_t *c = (uint32_t *) fillBuf;

				flen = run->num_blks * h->blk_sz;
				wsize = (fillBufSz > flen) ? flen : fillBufSz;

				for(j = 0; j < wsize/sizeof(uint32_t); j++, c++){
					*c= f;
				}

				if(ctx->dcache_op){
					ctx->dcache_op((unsigned long)fillBuf, wsize, CACHE_OP_FLUSH);
				}
				else if(ctx->dcache_op_all){
					ctx->dcache_op_all(CACHE_OP_FLUSH);
				}

				while( flen )
				{
					if( (blk_func(hdl, (void *)fillBuf, blkno, wsize>>9)) != 0){
						ser_putstr("sparse blk write failed\n");
						return( -1 );
					}

					blkno += wsize >> 9;
					flen -= wsize;

					if( wsize > flen ){
						wsize = flen;
					}
				}

				addr += sizeof(uint32_t);
			}
			break;

		case RUN_TYPE_SKIP:
			{
				if (run->total_bytes != sizeof(run_t)) {
					ser_putstr("bad skip run size\n");
					return( -1 );
				}

				blkno += (run->num_blks * h->blk_sz) >> 9;
			}
			break;

		default:
			ser_putstr("unknown run type: "); ser_puthex(run->run_type);ser_putstr("\n");
		}

		if( addr >= (saddr +isize) ){
			break;
		}
	}

	// return how many blocks are incremented, including skipped ones.
	return (blkno - sblks);
}

unsigned usb_boot_get_downloaded_bytes(void)
{
	return( (unsigned)current_transfer_bytes );
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/lib/usbboot/usbboot.c $ $Rev: 913227 $")
#endif
