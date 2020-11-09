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

#include <usb.h>

static usbd_endpoint_descriptor_t edesc0 = {
	.bLength			= USB_DESC_SIZE_ENDPOINT,
	.bDescriptorType    = USB_DESC_ENDPOINT,
	.bEndpointAddress   = 0,
	.bmAttributes       = USB_ATTRIB_CONTROL,
	.wMaxPacketSize     = 64,
	.bInterval          = 0x00,
};

static usb_endpoint_t *usb_get_ep(usb_dc_t *udc, uint16_t index)
{
	int i;
	int epnum = index & 0xF;
	usb_endpoint_t *ep;
	for(i = 0; i< udc->num_endpoints; i++){
		ep = &udc->endps[i];
		if( (ep->edesc->bEndpointAddress & 0xF) == epnum){
			return( ep );
		}
	}
	return( NULL );
}

static void usb_status_cmpl(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	//nothing need to be done for now.
}

static void usb_data_cmpl(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb)
{
	//nothing need to be done for now.
	if( urb->status ) return;

	urb->buffer_len = 0;
	urb->complete_cbf = usb_status_cmpl;
	//reverse the direction
	urb->flags = ( urb->flags & PIPE_FLAGS_TOKEN_OUT)? PIPE_FLAGS_TOKEN_IN : PIPE_FLAGS_TOKEN_OUT;
	usb_ep_transfer(udc, ep, urb, PIPE_FLAGS_TOKEN_STATUS | urb->flags);
}

static int usb_get_status(usb_dc_t *udc, usb_setup_packet_t *setup)
{
	uint16_t			status = 0;
	uint16_t			*buf;
	usb_endpoint_t	*tep = NULL;
	usb_transfer_t		*urb;

	if((setup->bRequestType & USB_RECIP_MASK)==USB_RECIPIENT_ENDPOINT){
		if ( (tep = usb_get_ep(udc, setup->wIndex)) == NULL){
			return( -1 );
		}

		if (tep->flags & USB_EP_FLAGS_STALLED)
			status = 1 << USB_FEATURE_EPT_HALT;
	}
	else{
		return( -1 );
	};

	urb = udc->ep0_urb;
	buf = (uint16_t *)urb->buffer;
	*buf = ENDIAN_LE16(status);
	urb->buffer_len = sizeof(uint16_t);
	urb->complete_cbf = usb_data_cmpl;

	return( usb_ep_transfer(udc, udc->ep0, urb, PIPE_FLAGS_TOKEN_IN));
}

static int usb_feature(usb_dc_t *udc, usb_setup_packet_t *setup, int set)
{
	usb_endpoint_t *tep = NULL;
	int			ret = -1;

	if((setup->bRequestType & USB_RECIP_MASK)==USB_RECIPIENT_ENDPOINT){
		if(setup->wValue==USB_FEATURE_EPT_HALT){
			if( (tep = usb_get_ep(udc, setup->wIndex)) == NULL){
				return( -1 );
			}

			if(set){
				if((ret = udc->set_endpoint_state(udc, tep, USB_ENDPOINT_STATE_STALLED))==EOK){
					tep->flags |= USB_EP_FLAGS_STALLED;
				}
			}
			else{
				if((ret = udc->clear_endpoint_state(udc, tep, USB_ENDPOINT_STATE_STALLED))==EOK){
					tep->flags &= ~USB_EP_FLAGS_STALLED;
				}
			}
		}
	}

	if( ret == EOK ){
		udc->ep0_urb->buffer_len = 0;
		udc->ep0_urb->complete_cbf = usb_status_cmpl;

		return(usb_ep_transfer(udc, udc->ep0, udc->ep0_urb, PIPE_FLAGS_TOKEN_STATUS|PIPE_FLAGS_TOKEN_IN));
	}

	return( ret );
}

static int usb_set_address(usb_dc_t *udc, usb_setup_packet_t *setup)
{
	uint8_t	addr;
	int		ret = EOK;

	addr = setup->wValue;

	if (udc->state == USB_DEVICE_STATE_CONFIGED) {
		LOGS(udc, 0, "trying to set address when configured");
		return -EINVAL;
	}

	ret = udc->address_device(udc, addr);

	if (addr)
		udc->state = USB_DEVICE_STATE_ADDRESSED;
	else
		udc->state = USB_DEVICE_STATE_DEFAULT;

	return ret;
}

static int usb_set_configuration(usb_dc_t *udc, usb_setup_packet_t *setup)
{
	uint8_t cfg;
	int ret;

	cfg =setup->wValue;

	switch (udc->state) {
	case USB_DEVICE_STATE_ADDRESSED:
		if( (ret = udc->config_device(udc, cfg)) == EOK){
			ret = udc->setup(udc, setup);
		}
		break;

	case USB_DEVICE_STATE_CONFIGED:
		ret = udc->setup(udc, setup);
		if (!cfg)
			udc->state = USB_DEVICE_STATE_ADDRESSED;
		break;
	case USB_DEVICE_STATE_DEFAULT:
	default:
		ret = -EINVAL;
	}

	if( ret == EOK ){
		udc->ep0_urb->buffer_len = 0;
		udc->ep0_urb->complete_cbf = usb_status_cmpl;
		return(usb_ep_transfer(udc, udc->ep0, udc->ep0_urb, PIPE_FLAGS_TOKEN_STATUS|PIPE_FLAGS_TOKEN_IN));
	}

	return( ret );
}

static int usb_handle_standard_request(usb_dc_t *udc, usb_setup_packet_t *setup)
{
	int ret;

	switch (setup->bRequest) {
	case USB_GET_STATUS:
		ret = usb_get_status(udc, setup);
		break;
	case USB_CLEAR_FEATURE:
		ret = usb_feature(udc, setup, 0);
		break;
	case USB_SET_FEATURE:
		ret = usb_feature(udc, setup, 1);
		break;
	case USB_SET_ADDRESS:
		ret = usb_set_address(udc, setup);
		break;
	case USB_SET_CONFIGURATION:
		ret = usb_set_configuration(udc, setup);
		break;
	default:
		ret = udc->setup(udc, setup);
		break;
	};

	return ret;
}

int usb_setup_packet_process(usb_dc_t *udc, uint8_t *buffer)
{
	usb_setup_packet_t	setup_packet;

	setup_packet.bRequestType	= buffer[0];
	setup_packet.bRequest		= buffer[1];
	setup_packet.wValue		= ENDIAN_LE16( *(uint16_t *) (&buffer[2]) );
	setup_packet.wIndex		= ENDIAN_LE16( *(uint16_t *) (&buffer[4]) );
	setup_packet.wLength		= ENDIAN_LE16( *(uint16_t *) (&buffer[6]) );

	switch( setup_packet.bRequestType & USB_TYPE_MASK ) {
		case USB_TYPE_STANDARD :
			if( usb_handle_standard_request( udc, &setup_packet ) == EOK ){
				break;
			}
		case USB_TYPE_CLASS :
		case USB_TYPE_VENDOR :
			if ( udc->setup(udc, &setup_packet) == EOK ) {
				break;
			}

		// fall through
		default :
			udc->set_endpoint_state( udc, &udc->endps[0], USB_ENDPOINT_STATE_STALLED );
			return( ENOTSUP );
			break;
	}

	return( EOK );
}

int usb_handle_interrupts(usb_dc_t *udc)
{
	int err;
	uint32_t emask;

	while( 1 ){
		emask = 0;
		err = udc->interrupt(udc, &emask);

		switch( err ){
			case USB_INTR_BUS_STATE:
				break;
			case USB_INTR_EP_COMPL:
				break;
			//nothing happening
			case USB_INTR_NONE:
			default:
				break;
		}
	}

	return( EOK );
}

int usb_ep_transfer(usb_dc_t *udc, usb_endpoint_t *ep, usb_transfer_t *urb, uint32_t flags)
{
	urb->actual_len = 0;
	urb->flags = flags;
	urb->status = 0;
	return(udc->endpoint_transfer(udc, ep, urb, flags));
}

int usb_ep_enable(usb_dc_t *udc, usbd_endpoint_descriptor_t *edesc, usb_endpoint_t **ep)
{
	usb_endpoint_t *tep;
	int ret = -1;

	int epnum = edesc->bEndpointAddress & 0xF;

	if( epnum >= udc->num_endpoints ){
		return( -1 );
	}

	if((tep = &udc->endps[epnum])){
		ret = 0;
		if(!(tep->flags & USB_EP_FLAGS_ENABLED)){
			tep->edesc = edesc;
			if( (ret =udc->endpoint_enable(udc, tep)) == EOK ){
				tep->flags |= USB_EP_FLAGS_ENABLED;
				if(ep){
					*ep = tep;
				}
			}
		}
	}

	return( ret );
}

int usb_ep_disable(usb_dc_t *udc, usb_endpoint_t *ep)
{
	int ret = EOK;

	if( ep->flags & USB_EP_FLAGS_ENABLED){
		//abort transfers
		if( (ret = udc->endpoint_abort(udc, ep)) == EOK ){
		}
		if( (ret = udc->endpoint_disable(udc, ep)) == EOK ){
			ep->flags &= ~USB_EP_FLAGS_ENABLED;
		}
	}

	return( ret );
}

static int
usb_device_state_change(usb_dc_t *udc, uint32_t state)
{

	udc->state = state;

	LOG( udc, 3, "%s(%d): state= %x", __func__, __LINE__, state);

	switch( state ){
		case USB_DEVICE_STATE_INSERTED:
			udc->connect(udc, 1);
			break;

		case USB_DEVICE_STATE_RESET:
			udc->reset(udc);
			break;
		case USB_DEVICE_STATE_REMOVED:
			udc->state = USB_DEVICE_STATE_DEFAULT;
			udc->connect(udc, 0);
			break;
		default:
			break;
	}

	return( EOK );
}

static int
usb_set_device_speed(usb_dc_t *udc, uint32_t speed )
{
	udc->speed = speed;

	return( EOK );
}

int usb_init_udc(usb_dc_t *udc)
{
	int err;

	udc->bCurrent = udc->malloc_buffer;
	udc->bEnd = udc->malloc_buffer + udc->malloc_size;

	if((err = usb_calloc(udc, (void **)&udc->endps, NULL, 0, udc->num_endpoints * sizeof(usb_endpoint_t))) != EOK){
		LOGS( udc, 0, "alloc endps failed");
		goto error;
	}

	if((err = usb_calloc(udc, (void **)&udc->ep0_urb, NULL, 0, sizeof(usb_transfer_t)))!= EOK){
		LOGS( udc, 0, "alloc ep0 urb failed");
		goto error1;
	}

	//512 bytes of ep0 buffer should be big enough
	if((err = usb_malloc(udc, (void **)&udc->ep0_urb->buffer, &udc->ep0_urb->buffer_paddr, 0, 512))!= EOK){
		LOGS( udc, 0, "alloc ep0 buffer failed");
		goto error1;
	}

	udc->usb_device_state_change = usb_device_state_change;
	udc->usb_set_device_speed= usb_set_device_speed;

	if((err = udc->init(udc)) != EOK){
		LOGS( udc, 0, "udc init controller failed");
		goto error2;
	}

	// enable ep0.
	if((err = usb_ep_enable(udc, &edesc0, &udc->ep0)) != EOK){
		LOGS( udc, 0, "enable ep0 failed");
		goto error2;
	}

	udc->state = USB_DEVICE_STATE_DEFAULT;

	if((err = udc->set_bus_state(udc, USB_BUS_STATE_CONNECTED)) != EOK){
		LOGS( udc, 0, "set_bus_state failed");
		goto error2;
	}

	return( EOK );
error2:
	usb_free(udc, udc->ep0_urb);
error1:
	usb_free(udc, udc->endps);
error:
	return( err );
}

int usb_exit_udc(usb_dc_t *udc)
{
	if( udc->fini ){
		return( udc->fini(udc));
	}

	return( -1 );
}

static int _usb_alloc(usb_dc_t *udc, void **buf, paddr64_t *buf_phys, uint32_t flags, uint32_t size, int set)
{
	uint32_t	*c;

	*buf = NULL;

	c = (uint32_t *)udc->malloc_buffer;

	// minimum allocation of 4 bytes;
	size = (size + sizeof(uint32_t)) & ~(sizeof(uint32_t) -1 );

	// find any free slot that has equal size.
	while( (uint8_t *)c < udc->bCurrent ){
		if( *c == size ){
			*c |= 1;
			if( buf_phys ){
				*buf_phys = udc->malloc_buffer_phys + ((uint8_t *)(c+1) - udc->malloc_buffer);
			}

			c++;

			*buf = (uint8_t *)c;

			//memset the memory
			while(size && set){
				*c = 0;
				c++;
				size -=sizeof(uint32_t);
			}
			return( EOK );
		}
		c = (uint32_t *)((uint8_t *)c + (*c & ~1) + sizeof(uint32_t));
	}

	if( udc->bEnd - udc->bCurrent > size + sizeof(uint32_t)){
		c = (uint32_t *)udc->bCurrent;
		*c = size | 1;
		*buf = udc->bCurrent + sizeof(uint32_t);
		if( buf_phys ){
			*buf_phys = udc->malloc_buffer_phys + ((uint8_t *)(c+1) - udc->malloc_buffer);
		}
		udc->bCurrent += size + sizeof(uint32_t);
		c= *buf;
		//memset the memory
		while(size && set){
			*c = 0;
			c++;
			size -=sizeof(uint32_t);
		}
		return( EOK );
	}

	return -1;
}

int usb_malloc(usb_dc_t *udc, void **buf, paddr64_t *buf_phys, uint32_t flags, uint32_t size)
{
	return(_usb_alloc(udc, buf, buf_phys, flags, size, 0));
}

int usb_calloc(usb_dc_t *udc, void **buf, paddr64_t *buf_phys, uint32_t flags, uint32_t size)
{
	return(_usb_alloc(udc, buf, buf_phys, flags, size, 1));
}

int usb_free(usb_dc_t *udc, void *buf)
{
	uint32_t *c;

	c = (uint32_t *)udc->malloc_buffer;

	while( (uint8_t *)c < udc->bCurrent ){
		if( (uint8_t *)c + sizeof(uint32_t) == (uint8_t *)buf ){
			*c &= ~1;
			return( EOK );
		}
		c = (uint32_t *)((uint8_t *)c + (*c & ~1) + sizeof(uint32_t));
	}

	return -1;
}

 void usb_delay(usb_dc_t *udc, uint32_t d)
{
	if(udc->bs_delay_ms){
		udc->bs_delay_ms(d);
	}
}

 void usb_delay_ns(usb_dc_t *udc, uint32_t d)
{
	if(udc->bs_delay_ns){
		udc->bs_delay_ns(d);
	}
}

void usb_bus_sync(usb_dc_t *udc, uint32_t flags)
{
	if(udc->bs_bus_sync){
		udc->bs_bus_sync(udc, flags);
	}
}

#ifndef USE_IPL
int
usb_slogf( usb_dc_t * udc, int level, const char *fmt, ...)
{
	int rc;
	va_list arglist;

	if ( udc && ( level > udc->verbose ) )
		return( 0 );

	va_start( arglist, fmt );
	rc = vslogf( 12, level, fmt, arglist );
	va_end( arglist );
	return( rc );
}
#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/ipl/lib/usbboot/usb.c $ $Rev: 808052 $")
#endif
