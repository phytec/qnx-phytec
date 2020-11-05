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


#include "chipidea.h"

int
chip_idea_set_device_address(  usb_dc_t *udc, uint8_t address )
{
	/*
	 * server sets the address too soon when we short circuit the
	 * status phase. So we do it in complete_irq...
	 */
	udc->usb_device_state_change( udc, USB_DEVICE_STATE_ADDRESSED);

	return( EOK );
}

int
chip_idea_select_configuration( usb_dc_t *udc, uint8_t config )
{
	udc->usb_device_state_change( udc, USB_DEVICE_STATE_CONFIGED);

	return( EOK );
}

int
chip_idea_abort_transfer( usb_dc_t *udc, chip_idea_endpoint_t *ep )
{
	chip_ideadc				*dcctrl = udc->dc_data;
	volatile chip_idea_dtd_t	*dtd;
	int					i;
	uint32_t				timeout;
	uint32_t				flush_retry = 5;

	if ( !ep || !dcctrl ) {
		return( EOK );
	}

	if ( ep->active_urb ) {
		do  {
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTFLUSH, ep->prime_bit );

			timeout = 1000000;
			while ( in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTFLUSH ) && --timeout ) {

			}

			if ( !timeout ) {
				LOG1H(udc, 7, "Timeout waiting for endpoint flush on ep ", ep->ep_num );
			}
		} while ( ( in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSTAT ) & ep->prime_bit) && ( --flush_retry ) );

		if ( !flush_retry ) {
				LOG1H(udc, 7, "Error flushing endpoint ep ", ep->ep_num );
		}

		// stop the endpoint
		ep->qh->size_ioc_int_sts &= ~(EP_QUEUE_HEAD_STATUS_ACTIVE | EP_QUEUE_HEAD_STATUS_HALT);

		usb_delay( udc, 5 );

		// disable all active transfers
		ep->qh->next_dtd_ptr = DTD_NEXT_TERMINATE;

		dtd = ep->dtd_head;

		for( i = 0; i < CHIP_IDEA_MAX_LINKED_DTD; i++, dtd++ ) {
			dtd->next_td_ptr = DTD_NEXT_TERMINATE;
			dtd->size_ioc_sts = 0;
		}

		ep->active_urb = NULL;
	}

	return( EOK );
}

void
chip_idea_send_empty_packet( chip_ideadc *dcctrl )
{
	chip_idea_endpoint_t	*ep;
	chip_idea_dtd_t			*dtd;

	ep = &dcctrl->ep[16];
	dtd = ep->dtd_head;

	/* build a dTD  */
	dtd->next_td_ptr = DTD_NEXT_TERMINATE;
	dtd->size_ioc_sts = DTD_IOC | DTD_STATUS_ACTIVE;

	/* write dQH next pointer and dQH terminate bit to 0 */
	ep->qh->next_dtd_ptr = ep->dtd_addr;

	/* clear active &halt bit in dQH */
	ep->qh->size_ioc_int_sts &= ~(EP_QUEUE_HEAD_STATUS_ACTIVE | EP_QUEUE_HEAD_STATUS_HALT);

	/* Memory barrier between writing TDs and hitting the prime bit */
	if( dcctrl->udc->bs_bus_sync){
		 dcctrl->udc->bs_bus_sync( dcctrl->udc, 0);
	}

	/* prime endpoint */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME ) |
		ep->prime_bit );

	return;
}

static void
chip_idea_ep_qh_setup( usb_dc_t *udc, chip_idea_endpoint_t *ep, uint32_t ep_type, uint32_t max_pkt_len )
{
	chip_ideadc			*dcctrl = udc->dc_data;
	chip_idea_qh_t		*qh = NULL;

	/* set the Endpoint Capabilites Reg of dQH */
	switch ( ep_type ) {
		case USB_ENDPOINT_XFER_CONTROL:
			/* Interrupt On Setup (IOS). for control ep  */
			qh = &dcctrl->ep_qh[ (ep->ep_num * 2) + ep->dir ];
			qh->max_pkt_length = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS) | EP_QUEUE_HEAD_IOS;
			break;

		case USB_ENDPOINT_XFER_ISOC:
			ep->qh->max_pkt_length = ((max_pkt_len & USB_ENDPOINT_MPS_MSK) << EP_QUEUE_HEAD_MAX_PKT_LEN_POS) | EP_QUEUE_HEAD_ZLT_SEL
                               | ( (((max_pkt_len & USB_ENDPOINT_HB_MULT_MSK)>>USB_ENDPOINT_HB_MULT_POS)+ 1)<<30) ;
			break;

		case USB_ENDPOINT_XFER_BULK:
		case USB_ENDPOINT_XFER_INT:
			/* Always disable zlp transmission ... */
			ep->qh->max_pkt_length = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS) | EP_QUEUE_HEAD_ZLT_SEL;
			break;

		default:
			LOG1H(udc, 5, "error invalid ep type %d", ep_type );
			ep->qh->max_pkt_length = 0;
			return;
	}

	return;
}

int
chip_idea_set_endpoint_state( usb_dc_t *udc, usb_endpoint_t *usb_ep, uint32_t ep_state )
{
	chip_ideadc			*dcctrl = udc->dc_data;
	chip_idea_endpoint_t	*ep;
	uint32_t				tmp;

	if ( !( ep = (chip_idea_endpoint_t *) usb_ep->user ) ) {
		return( ENOENT );
	}

	switch ( ep_state ) {
		case USB_ENDPOINT_STATE_READY :
			break;

		case USB_ENDPOINT_STATE_ENABLE :
			break;

		case USB_ENDPOINT_STATE_DISABLED :
			break;

		case USB_ENDPOINT_STATE_NAK :
			break;

		case USB_ENDPOINT_STATE_STALLED :
			if ( ep->ep_num == 0 ) {
				/* Stall both rx & tx when stalling ep 0 ... */
				tmp = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL0 );
				out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL0, tmp | ( EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL ) );
			}
			else {
				tmp = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ) );
				if( ep->dir == CHIP_IDEA_EP_DIR_IN ) {
					out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), tmp | EPCTRL_TX_EP_STALL );
				}
				else {
					out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), tmp | EPCTRL_RX_EP_STALL );
				}
			}
			break;

		default :
			break;
	}

	return( EOK );
}

int
chip_idea_clear_endpoint_state( usb_dc_t *udc, usb_endpoint_t *usb_ep, uint32_t ep_state )
{
	chip_ideadc			*dcctrl = udc->dc_data;
	chip_idea_endpoint_t	*ep;
	uint32_t				tmp;

	if ( ! ( ep = (chip_idea_endpoint_t *)usb_ep->user ) ) {
		return( ENOENT );
	}

	switch ( ep_state ) {
		case USB_ENDPOINT_STATE_READY :
			break;

		case USB_ENDPOINT_STATE_ENABLE :
			break;

		case USB_ENDPOINT_STATE_DISABLED :
			break;

		case USB_ENDPOINT_STATE_NAK :
			break;

		case USB_ENDPOINT_STATE_STALLED :
			if ( ep->ep_num == 0 ) {
				out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL0, ~(EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL) );
			}
			else {
				tmp = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ) );
				if( ep->dir == CHIP_IDEA_EP_DIR_IN ) {
					out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), (tmp & ~EPCTRL_TX_EP_STALL) | EPCTRL_TX_DATA_TOGGLE_RST );
				}
				else {
					out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), (tmp & ~EPCTRL_RX_EP_STALL) | EPCTRL_RX_DATA_TOGGLE_RST );
				}
			}
			break;

		default :
			break;
	}

	return( EOK );
}

int
chip_idea_set_bus_state( usb_dc_t *udc, uint32_t device_state )
{
	chip_ideadc	*dcctrl = udc->dc_data;
	uint32_t		val;

	switch ( device_state ) {
		case USB_BUS_STATE_DISCONNECTED :
			CHIP_IDEA_BOARD_SPECIFIC_LINK_DOWN
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & ~USB_CMD_RUN_STOP );
			//udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_REMOVED);
			break;

		case USB_BUS_STATE_CONNECTED :
			CHIP_IDEA_BOARD_SPECIFIC_LINK_UP
			val = (in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & ~USB_CMD_ITC_MASK) |
				 USB_CMD_RUN_STOP | dcctrl->int_thresh;
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, val );
			break;

		case USB_BUS_STATE_RESUME :
		default :
			return( ENOTSUP );
			break;
	}

	return( EOK );
}

int
chip_idea_endpoint_enable( usb_dc_t *udc, usb_endpoint_t *usb_ep )
{
	chip_ideadc			*dcctrl = udc->dc_data;
	chip_idea_endpoint_t	*ep;
	int					ep_num, direction;
	uint32_t			epctrl;

	ep_num = usb_ep->edesc->bEndpointAddress & 0x7f;

	LOG1H(udc, 2, "Enable Endpoint address ", ep_num );

	if ( ep_num > (dcctrl->num_ep - 1) ) {
		return( ENOTSUP );
	}

	if ( ep_num == 0 ) {
		/* Endpoint 0 is split in half ... i.e. server only has one handle,
		 * but we have two */
		dcctrl->ep[0].ep = dcctrl->ep[16].ep	= usb_ep;
		dcctrl->ep[0].ep_num = dcctrl->ep[16].ep_num	= 0;

		dcctrl->ep[0].dir		= CHIP_IDEA_EP_DIR_OUT;
		dcctrl->ep[16].dir		= CHIP_IDEA_EP_DIR_IN;
		dcctrl->ep[0].prime_bit	= 1;
		dcctrl->ep[16].prime_bit	= (1<<16);

		usb_ep->user = &dcctrl->ep[0];

		return( EOK );
	}

	direction = ((usb_ep->edesc->bEndpointAddress & 0x80) ? CHIP_IDEA_EP_DIR_IN : CHIP_IDEA_EP_DIR_OUT);

	ep = &dcctrl->ep[ep_num + (direction ? 16 : 0)];

	ep->ep		= usb_ep;
	ep->ep_num	= ep_num;
	ep->dir	= direction;
	ep->prime_bit	= (direction == CHIP_IDEA_EP_DIR_IN ) ? (1 << (ep->ep_num + 16)) : (1 << ep->ep_num);

	ep->mps		= usb_ep->edesc->wMaxPacketSize;
	ep->type	= usb_ep->edesc->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
	epctrl = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ) );

	if( ep->dir == CHIP_IDEA_EP_DIR_IN ) {
		/* Opposite direction EP type must be changed from default if it is unused. (30.8.1.5.21) */
		if ( !( epctrl & EPCTRL_RX_EP_TYPE_MASK ) ) {
			epctrl |= (ep->type << EPCTRL_RX_EP_TYPE_SHIFT);
		}
		/* don't clobber Rx bits */
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), (epctrl & ~EPCTRL_TX_BITS_MASK) |
			 EPCTRL_TX_ENABLE | EPCTRL_TX_DATA_TOGGLE_RST | (ep->type) << EPCTRL_TX_EP_TYPE_SHIFT );
	}
	else {
		/* Opposite direction EP type must be changed from default if it is unused. (30.8.1.5.21) */
		if ( !( epctrl & EPCTRL_TX_EP_TYPE_MASK ) ) {
			epctrl |= (ep->type << EPCTRL_TX_EP_TYPE_SHIFT );
		}
		/* don't clobber Tx bits */
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL( ep->ep_num ), (epctrl & ~EPCTRL_RX_BITS_MASK) |
			EPCTRL_RX_ENABLE | EPCTRL_RX_DATA_TOGGLE_RST | (ep->type) << EPCTRL_RX_EP_TYPE_SHIFT );
	}

	usb_ep->user = ep;

	chip_idea_ep_qh_setup( udc, ep, ep->type, ep->mps );

	return( EOK );
}


int
chip_idea_endpoint_disable( usb_dc_t *udc, usb_endpoint_t *usb_ep )
{
	chip_idea_endpoint_t *ep =(chip_idea_endpoint_t *)usb_ep->user;

	if ( !ep ) {
		return( EOK );
	}

	ep->active_urb = 0;
	usb_ep->user = 0;

	return( EOK );
}

int
chip_idea_transfer_abort( usb_dc_t *udc, usb_endpoint_t *usb_ep )
{
	chip_ideadc			*dcctrl = udc->dc_data;
	chip_idea_endpoint_t	*ep;

	if ( (ep = (chip_idea_endpoint_t *)usb_ep->user ) == NULL ) {
		return( ENOENT );
	}

	if ( ep->ep_num == 0 ) {
		// abort both in and out for control endpoint
		chip_idea_abort_transfer( udc, &dcctrl->ep[ 0 ] );
		chip_idea_abort_transfer( udc, &dcctrl->ep[ 16 ] );
	}
	else {
		chip_idea_abort_transfer( udc, ep );
	}

	return( EOK );
}


int
chip_idea_transfer( usb_dc_t *udc, usb_endpoint_t *usb_ep, usb_transfer_t *urb, _uint32 flags )
{
	chip_ideadc					*dcctrl = udc->dc_data;
	chip_idea_endpoint_t			*ep;
	volatile chip_idea_dtd_t			*dtd;
	uint32_t						dtd_length, mphys_buffer, next_dtd_addr, offset;
	int							i, length;

	if ( (ep = (chip_idea_endpoint_t *) usb_ep->user) == 0 ) {
		return( ENODEV );
	}

	if ( ep->ep_num == 0 ) {
		/* select correct half of ep0 ... */
		ep = &dcctrl->ep[(flags & PIPE_FLAGS_TOKEN_IN) ? 16 : 0];

		// check if we need to send a zero length packet
		if ( ( urb->buffer_len == 0 ) && (flags & PIPE_FLAGS_TOKEN_IN) ) {
			ep->active_urb = urb;
			ep->flags = flags;
			chip_idea_send_empty_packet( dcctrl );
			return( EOK );
		}
		// fall through for normal transfer
	}

	ep->active_urb = urb;
	ep->flags = flags;
	length = urb->buffer_len - urb->actual_len;
	mphys_buffer = ((uint32_t) (urb->buffer_paddr + urb->actual_len) & 0xfffff000);
	offset = ((uint32_t) (urb->buffer_paddr + urb->actual_len) & 0xfff);

	LOG1H(udc, 5, "Transfer ep ", ep->ep_num);
	LOG1H(udc, 5, "Length ", length);
	LOG1H(udc, 5, "Flags ", flags );

	// there is a problem for token out (receive) that the controller can not properly terminate
	// the transfer in case of short packet (no interrupt fired or controller automatically advance
	// to the next DTD), so we only allow one DTD.
	// ideally, we should only restrict transfer that have URB_SHORT_XFER_OK flag set,
	// we'll rework that once the stack can pass down that flag.
	if(/*(urb->flags & PIPE_FLAGS_SHORT_XFER_OK) &&*/ (flags & PIPE_FLAGS_TOKEN_OUT)){
		length = min(length, EP_MAX_LENGTH_TRANSFER);
	}else{
		length = min(length, CHIP_IDEA_MAX_TRANSFER_SIZE);
	}
	ep->transfer_len = length;

	dtd = ep->dtd_head;
	next_dtd_addr = ep->dtd_addr;

	for ( i = 0 ; i < CHIP_IDEA_MAX_LINKED_DTD ; i++, dtd++ ) {
		dtd_length = min( EP_MAX_LENGTH_TRANSFER, length );
		dtd->buff_ptr0	= mphys_buffer + offset;

		dtd->buff_ptr1	= mphys_buffer + 0x1000;
		dtd->buff_ptr2	= mphys_buffer + 0x2000;
		dtd->buff_ptr3	= mphys_buffer + 0x3000;
		dtd->buff_ptr4	= mphys_buffer + 0x4000;

		// shouldn't need to interupt on every TD
		dtd->size_ioc_sts	= (dtd_length << DTD_LENGTH_BIT_POS) /* | DTD_IOC*/ | DTD_STATUS_ACTIVE;
		if ( (dtd_length - length) <= 0 ) {
			dtd->next_td_ptr	= DTD_NEXT_TERMINATE;
			dtd->size_ioc_sts	|= DTD_IOC;
			break;
		}

		dtd->next_td_ptr = (next_dtd_addr += sizeof( chip_idea_dtd_t ));

		mphys_buffer += dtd_length;
		length -= dtd_length;
	}

	ep->dtd_count = i + 1;

	/* Active the dtd chain */
	ep->qh->next_dtd_ptr = ep->dtd_addr;
	ep->qh->size_ioc_int_sts &= ~(EP_QUEUE_HEAD_STATUS_ACTIVE | EP_QUEUE_HEAD_STATUS_HALT);

	/* Make sure the descriptors have been written to main memory before hitting the prime bit */
	/* to ensure the controller has a coherent view of the descriptors */
	if( udc->bs_bus_sync){
		udc->bs_bus_sync(udc, 0);
	}

	/* only now ask the controller to process descriptors */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME )
		| ep->prime_bit );

	return( EOK );
}

static void
chip_idea_ep0_setup_irq(usb_dc_t *udc )
{
	chip_ideadc			*dcctrl = udc->dc_data;
	chip_idea_qh_t		*qh;
	uint8_t				buf[8];
	uint32_t				timeout;

	/* Clear bit in ENDPTSETUPSTAT */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA ) | 1 );

	qh = &dcctrl->ep_qh[0];

	/* while a hazard exists when setup package arrives */
	do {
		/* Set Setup Tripwire */
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) | USB_CMD_SUTW );

#if defined(__LITTLEENDIAN__)
		/* Copy the dQH.SetupBuffer into local buffer urb */
		memcpy( buf, qh->setup_buffer, sizeof( usb_setup_packet_t ) );
#else
		/* setup buffer needs to swapped */
		{
			int i;
			/* Copy the dQH.SetupBuffer into local buffer urb*/
			for ( i=0 ; i < 4 ; i++ ) {
			        buf[i] = qh-> setup_buffer[3-i];
			}
			for ( i = 0; i < 4 ; i++ ) {
			        buf[4+i]= qh-> setup_buffer[7-i];
			}
		}
#endif

	} while( !(in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & USB_CMD_SUTW) );

	/* Clear Setup Tripwire */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD ) & ~USB_CMD_SUTW );

	/*
	 * Due to missing code in server, we handle the set address request (0x5) ourselves,
	 * and ignore the set_address call from server.
	 */

	if ( (buf[0] == 0) && (buf[1] == 0x5) ) {
		LOG1H(udc, 3,"Assign address ", buf[2] );
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_DEVICEADDR,  buf[2] << USB_DEVICE_ADDRESS_BIT_POS | USB_DEVICE_ADDRESS_USBADRA );
		chip_idea_send_empty_packet( dcctrl );
	}

	timeout = 10000000UL;
	while ( (in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA ) & 1 ) && --timeout ) {
		continue;
	}

	/* send setup packet up to server */
	usb_setup_packet_process(udc, buf);

	return;
}

static uint32_t
dtd_check_errors( chip_ideadc *dcctrl, volatile chip_idea_dtd_t *curr_td, uint32_t error )
{
	if ( error & DTD_STATUS_DATA_BUFF_ERR ) {
		LOGS(dcctrl->udc, 1,"dtd buffer error" );
		return( USBD_STATUS_CMP_ERR | USBD_STATUS_DATA_OVERRUN );
	}
	else if ( error & DTD_STATUS_TRANSACTION_ERR ) {
		LOGS(dcctrl->udc, 1,"transaction error" );
		return( USBD_STATUS_CMP_ERR | USBD_STATUS_BITSTUFFING );
	}
	else {
		LOG1H(dcctrl->udc, 1,"dtd error ", error );
		return( USBD_STATUS_CMP_ERR | USBD_STATUS_ABORTED );
	}
}

static int
chip_idea_dtd_complete_irq( usb_dc_t *udc, uint32_t ep_complete_mask )
{
	chip_ideadc					*dcctrl = udc->dc_data;
	chip_idea_endpoint_t			*ep;
	int								epnum = 0;
	volatile chip_idea_dtd_t		*dtd;
	int							i;
	uint32_t						remainder = 0;
	uint32_t						transfer_length, actual_len;
	uint32_t						status = EOK;
	usb_transfer_t					*urb = 0;
	uint32_t						error;
	int							complete;

	for ( ; ep_complete_mask ; ep_complete_mask >>= 1, epnum++ ) {
		if ( !(ep_complete_mask & 1) ) {
			continue;
		}

		ep = &dcctrl->ep[ epnum ];
		status = EOK;

		if ( !ep->active_urb ) {
			LOG( udc, 5, "chip_idea_dtd_complete_irq : no active URB for %s ep %d", ep->dir ? "in" : "out", ep->ep_num );
			continue;
		}
		dtd = ep->dtd_head;
		transfer_length = ep->transfer_len;
		actual_len = 0;
		complete = 0;

		for ( i = 0 ; i < ep->dtd_count ; i++, dtd++ ) {
			if ( dtd->size_ioc_sts & DTD_STATUS_ACTIVE ) {
				// There is a known issue where the endpoint completion interrupt gets processed
				// before updates to the TD become visible.  To handle this case, the TD is
				// polled until the active bit becomes low.

				// Wait and check again for TD to become inactive
				usb_delay_ns(udc, 10 * 1000);
				if ( dtd->size_ioc_sts & DTD_STATUS_ACTIVE ) {
					LOGS(udc, 0, "ERROR TD still active.  Giving up and dropping transfer.");
					complete = 0;
					break;
				}

				LOGS(udc, 0, "Recovered" );
			}

			if ( ( error = (dtd->size_ioc_sts & DTD_ERROR_MASK) ) ) {
				if ( (status = dtd_check_errors( dcctrl, dtd, error )) != EOK ) {
					complete = 1;
					break;
				}
			}

			remainder = (dtd->size_ioc_sts & 0x7FFF0000) >> 16;
			actual_len += min( transfer_length, EP_MAX_LENGTH_TRANSFER ) - remainder;
			transfer_length -= min( transfer_length, EP_MAX_LENGTH_TRANSFER ) - remainder;

			if ( remainder ) {
				/* short packet - complete the transfer */
				status |= USBD_STATUS_DATA_UNDERRUN;
				complete = 1;
				break;
			}

			if ( (dtd->next_td_ptr & DTD_NEXT_TERMINATE) ) {
				break;
			}
		}

		if ( (urb = ep->active_urb) == NULL ) {
			continue;
		}

		urb->actual_len += actual_len;
		ep->active_urb = 0;

		//if short pkt, error, or full buffer_len is done, finish it
		// otherwise, enqueue another transfer
		if ( complete || actual_len == 0 || urb->actual_len>=urb->buffer_len){
			/* clear the dtd queue */
			dtd = ep->dtd_head;

			for( i = 0 ; i < CHIP_IDEA_MAX_LINKED_DTD ; i++, dtd++ ) {
				dtd->next_td_ptr = DTD_NEXT_TERMINATE;
				dtd->size_ioc_sts = 0;
			}
			if( urb->complete_cbf){
				urb->complete_cbf(udc, ep->ep, urb);
			}
		}else{
			// enque another transfer,
			// we have recursive mutex, don't need to unlock it
			chip_idea_transfer( dcctrl->udc, ep->ep, urb, ep->flags );
		}
	}

	return( EOK );
}

static void
chip_idea_portchange_irq( usb_dc_t *udc )
{
	chip_ideadc	*dcctrl = udc->dc_data;
	uint32_t		speed;
	uint32_t		portstatus;

	portstatus = in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX );

	if (dcctrl->reset_state)
	{
		/* Bus resetting is finished */
		if ( !(portstatus & PORTSCX_PORT_RESET) ) {
			/* Get the speed */
			speed = CHIPIDEA_GET_SPEED();
			switch ( speed ) {
				case PORTSCX_PORT_SPEED_HIGH:
					LOGS(udc, 3,"High speed device");
					dcctrl->udc->usb_set_device_speed( dcctrl->udc, USB_DEVICE_HIGH_SPEED);
					break;
				case PORTSCX_PORT_SPEED_FULL:
					LOGS(udc, 3,"Full speed device" );
					dcctrl->udc->usb_set_device_speed( dcctrl->udc, USB_DEVICE_FULL_SPEED);
					break;
				case PORTSCX_PORT_SPEED_LOW:
					LOGS(udc, 3,"Low speed device"  );
					dcctrl->udc->usb_set_device_speed( dcctrl->udc, USB_DEVICE_LOW_SPEED);
					break;
				default:
					break;
			}

			dcctrl->udc->usb_device_state_change( dcctrl->udc, USB_DEVICE_STATE_RESET);
			dcctrl->reset_state = 0;
		}
	} else {
		/* Portchange occurs after reset on resume.
		 * If the controller was not in reset, the portchange is due to a resume. */
		LOGS(udc, 2, "USB device resume.... " );
		//udc->usbdc_self->usbdc_device_state_change( udc, IOUSB_DEVICE_STATE_RESUMED );
	}
}

/*
 *  Interrupt handler for USB reset received
 */

static void
chip_idea_reset_irq( usb_dc_t *udc )
{
	chip_ideadc	*dcctrl = udc->dc_data;
	uint32_t		timeout;
	uint32_t		portstatus;

	portstatus = in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX );

	/* Clear the device address */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_DEVICEADDR,
					in32( dcctrl->IoBase + CHIP_IDEA_UOG_DEVICEADDR ) & ( ~USB_DEVICE_ADDRESS_MASK) );

	/* Clear all the setup token semaphores */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA ) );

	/* Clear all the endpoint complete status bits */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCOMPLETE, in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCOMPLETE ) );

	timeout = 10000000UL;

	/* Wait until all endptprime bits cleared */
	while ( (in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTPRIME )) && --timeout ) {
		continue;
	}

	if ( timeout == 0 ) {
		LOGS(udc, 5,"endptprime timeout" );
	}

	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTFLUSH, 0xFFFFFFFF );

	/* enable ep0 */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCTRL0, EPCTRL_TX_ENABLE | EPCTRL_RX_ENABLE );

	/* setup endpoint0 dQH */
	chip_idea_ep_qh_setup( udc, &dcctrl->ep[0], USB_ENDPOINT_XFER_CONTROL, USB_MAX_CTRL_PAYLOAD );
	chip_idea_ep_qh_setup( udc, &dcctrl->ep[16], USB_ENDPOINT_XFER_CONTROL, USB_MAX_CTRL_PAYLOAD );

	if ( portstatus & PORTSCX_PORT_RESET )  {
		LOGS(udc, 5, "Bus reset..." );
	}
	else {
		LOGS(udc, 5, "Controller reset..." );
	}

	dcctrl->reset_state = 1;

	return;
}

int
chip_idea_interrupt( usb_dc_t *udc, uint32_t *emask )
{
	chip_ideadc	*dcctrl = udc->dc_data;
	uint32_t		otg_sc;
	uint32_t		irq_src;
	uint32_t		ec_mask;
	uint32_t		portstatus;

	CHIP_IDEA_EXTRA_PRE_INTERRUPT_CALLOUT

	irq_src = in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBSTS );// & CHIP_IDEA_DEFAULT_IRQ_EN_MASK;

	// read OTG Status/Control
	otg_sc = in32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC );

	// clear OTG status bits
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC, otg_sc );

	/* Clear notification bits */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBSTS, irq_src );

	/* USB interrupt */
	if ( irq_src & (USB_STS_INT | USB_STS_ERR ) ) {
		/* completion of dTD */
		if ( (ec_mask = in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCOMPLETE ) ) ) {
			/* Clear the bits in the register */
			out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTCOMPLETE, ec_mask );

			chip_idea_dtd_complete_irq( udc, ec_mask );
		}

		/* Setup packet, we only support ep0 as control ep */
		if( in32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPTSETUPSATA ) & 1 ) {
			/* receive setup packet */
			chip_idea_ep0_setup_irq( udc );
		}
	}

	/* Reset Received */
	if ( irq_src & USB_STS_RESET_RECEIVED ) {
		chip_idea_reset_irq( udc );
	}

	/* Port Change */
	if ( irq_src & USB_STS_PORT_CHANGE ) {
		chip_idea_portchange_irq( udc );
	}

	/* Sleep Enable (Suspend) */
	if ( irq_src & USB_STS_SUSPEND ) {
		portstatus = in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX );
		if( (portstatus & PORTSCX_PORT_SUSPEND) && !( otg_sc & OTG_AVV) ) {
			/* power management from needs proper diconnect notification - controller not reporting disconnected state */
			/* this may need to change to report SUSPEND */
			LOGS(udc , 2, "USB device disconnect");
			udc->usb_device_state_change( udc, USB_DEVICE_STATE_REMOVED);
		}
	}

	/* USB cable disconnected: B Session End Interrupt */
	if ( ( otg_sc & OTG_BSEIS ) && ( otg_sc & OTG_BSE ) ) {
		LOGS(udc, 2, "USB device disconnect" );
	}

	/* USB cable connected: B Session Valid Interrupt */
	if ( ( otg_sc & OTG_BSVIS ) && ( otg_sc & OTG_BSV ) ) {
		LOGS(udc, 2, "USB device connect" );
		udc->usb_device_state_change( udc, USB_DEVICE_STATE_INSERTED);
	}

	/* error */
	if (irq_src & USB_STS_SYS_ERR) {
		LOG1H(udc, 0, "Error interrupt", irq_src );
	}

	CHIP_IDEA_EXTRA_INTERRUPT_CALLOUT

	return( EOK );
}


/*
Initialize the hardware
*/
int
chip_idea_chip_init( chip_ideadc *dcctrl )
{
	uint32_t			tmp, timeout;
	int				error;
	paddr64_t		qh_phys, qtd_phys;
	chip_idea_dtd_t	*dtd;
	int				i;

	/*1. stop and reset the USB controller */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD) & ~ USB_CMD_RUN_STOP );
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD, in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD) |  USB_CMD_CTRL_RESET );

	/* wait for reset to complete */
	timeout = 10000000	;
	while ( (in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBCMD) & USB_CMD_CTRL_RESET) && --timeout ) {
	}

	if ( timeout == 0 ) {
		LOGS(dcctrl->udc, 0, "chip_idea_chip_init: TIMEOUT" );
		error = ETIMEDOUT;
		goto fail;
	}

	/* 2. Set Controller Mode in the USBMODE register to device mode */
	if ( dcctrl->udc->bs_init ){
		error = dcctrl->udc->bs_init( &dcctrl->udc->bs_data, 2 );
		if ( error ) {
			LOG1H( dcctrl->udc, 0, "chipidea_custom_init2() failed err =", error);
			goto fail;
		}
	}

	// force to full speed if specified at command line
#if 0
	if ( !(dcctrl->udc->hw_ctrl.capabilities & DC_CAP_HIGH_SPEED ) ) {
		out32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX, (in32( dcctrl->IoBase + CHIP_IDEA_UOG_PORTSCX ) | PORTSCX_PORT_FORCE_FULL_SPEED) );
	}
#endif

	if ( dcctrl->udc->bs_init ){
		error = dcctrl->udc->bs_init( &dcctrl->udc->bs_data, 3 );
		if ( error ) {
			LOG1H( dcctrl->udc, 0, "chipidea_custom_init2() failed err =", error);
			goto fail;
		}
	}

	/* 3.Clear the USB status register */
	tmp = in32( dcctrl->IoBase + CHIP_IDEA_UOG_USBSTS );
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBSTS, tmp );

	/*4. allocate and Initialize device queue heads in system memory */

	/* TODO Fix memory allocations - we allocate way too much ... */

	/* initialized QHs, take care the 2K align */
	dcctrl->qh_mem_sz = (dcctrl->num_ep * 2) * sizeof( chip_idea_qh_t ) + CHIP_IDEA_QH_ALIGN;

	usb_malloc(dcctrl->udc, (void **)&dcctrl->qh_mem, &qh_phys, 0, dcctrl->qh_mem_sz);
	if ( dcctrl->qh_mem == NULL ) {
		LOGS(dcctrl->udc, 0, "failed to alloc qh_mem" );
		error = ENOMEM;
		goto fail;
	}

	memset( dcctrl->qh_mem, 0, dcctrl->qh_mem_sz );

	dcctrl->ep_qh = (chip_idea_qh_t *) ( ((unsigned long) dcctrl->qh_mem + (CHIP_IDEA_QH_ALIGN - 1) ) & USB_EP_LIST_ADDRESS_MASK);
	qh_phys = ((_uint32)qh_phys + (CHIP_IDEA_QH_ALIGN - 1)) & USB_EP_LIST_ADDRESS_MASK;

	/*5. Configure ENDPOINTLISTADDR pointer */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_ENDPOINTLISTADDR, qh_phys );

	/*6 allocate and Initialize dTDs in system memory and link to dQHs */

	/* alloc 8-Dword dTD block of memory aligned to 32-byte boundaries, alloc 4 dTDs(64K) for each endpoint */
	dcctrl->dtd_mem_sz = (dcctrl->num_ep * 2) * (sizeof( chip_idea_dtd_t ) * CHIP_IDEA_MAX_DTD_PER_ENDPOINT) + 1 + 32;

	usb_malloc(dcctrl->udc, (void **)&dcctrl->dtd_mem, &qtd_phys, 0, dcctrl->dtd_mem_sz);

	if ( dcctrl->dtd_mem == NULL ) {
		LOGS(dcctrl->udc, 0, "failed to alloc dtd_mem" );
		error = ENOMEM;
		goto fail2;
	}

	memset( dcctrl->dtd_mem, 0, dcctrl->dtd_mem_sz );
	dtd = (chip_idea_dtd_t *) (((unsigned long) dcctrl->dtd_mem + 31) & 0xFFFFFFE0);

	/* Initialize endpoint structures */
	/* init EP dTD and link 4 dTDs to each dQH */
	tmp = ((_uint32) qtd_phys + 31) & 0xFFFFFFE0;
	for( i = 0 ; i < dcctrl->num_ep ; i++ ) {

		// init RX
		dcctrl->ep[i].qh		= &dcctrl->ep_qh[(i * 2)];
		dcctrl->ep[i].dtd_head	= dtd;
		dcctrl->ep[i].dtd_addr	= tmp;
		dtd += CHIP_IDEA_MAX_DTD_PER_ENDPOINT;
		tmp += CHIP_IDEA_MAX_DTD_PER_ENDPOINT * sizeof(chip_idea_dtd_t);

		dcctrl->ep[i+16].qh			= &dcctrl->ep_qh[(i*2)+1];
		dcctrl->ep[i+16].dtd_head	= dtd;
		dcctrl->ep[i+16].dtd_addr	= tmp;
		dtd += CHIP_IDEA_MAX_DTD_PER_ENDPOINT;
		tmp += CHIP_IDEA_MAX_DTD_PER_ENDPOINT * sizeof(chip_idea_dtd_t);
	}

	return( EOK );

fail2:
	usb_free( dcctrl->udc, dcctrl->qh_mem );
fail:
	return error;
}

void
chip_idea_chip_fini( chip_ideadc *dcctrl ) {

	if(dcctrl->udc->bs_fini){
		dcctrl->udc->bs_fini( dcctrl->udc->bs_data, 2 );
		dcctrl->udc->bs_fini( dcctrl->udc->bs_data, 1 );
		dcctrl->udc->bs_fini( dcctrl->udc->bs_data, 0 );
	}

	if ( dcctrl->qh_mem != NULL ) {
		usb_free(dcctrl->udc, dcctrl->qh_mem );
	}

	if ( dcctrl->dtd_mem != NULL ) {
		usb_free(dcctrl->udc,  dcctrl->dtd_mem );
	}
}

uint32_t
chip_idea_start( usb_dc_t *udc )
{
	chip_ideadc		*dcctrl = udc->dc_data;

	/* Set interrupt enables */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBINTR, CHIP_IDEA_DEFAULT_IRQ_EN_MASK );
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC, (CHIP_IDEA_DEFAULT_OTG_IRQ_EN_MASK | in32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC ) ) );

	/* Set Run/Stop bit to Run Mode */
	if ( !(dcctrl->flags & CHIP_IDEA_FLAGS_INITIAL_LINK_DISCONNECTED) ) {
		chip_idea_set_bus_state( udc, USB_BUS_STATE_CONNECTED );
	}
	else {
		chip_idea_set_bus_state( udc, USB_BUS_STATE_DISCONNECTED );
	}

	return( EOK );
}

int
chip_idea_init( void *hdl )
{
	chip_ideadc				*dcctrl;
	uint32_t				dcparams;
	int						error;
	usb_dc_t *udc = (usb_dc_t *)hdl;

	udc->fini = chip_idea_shutdown;

	udc->address_device = chip_idea_set_device_address;
	udc->config_device = chip_idea_select_configuration;
	udc->set_bus_state = chip_idea_set_bus_state;
	udc->endpoint_enable = chip_idea_endpoint_enable;
	udc->endpoint_disable = chip_idea_endpoint_disable;
	udc->set_endpoint_state = chip_idea_set_endpoint_state;
	udc->clear_endpoint_state = chip_idea_clear_endpoint_state;
	udc->endpoint_transfer = chip_idea_transfer;
	udc->endpoint_abort = chip_idea_transfer_abort;
	udc->interrupt = chip_idea_interrupt;

	/* allocate memory for our chip handle  */
	usb_calloc(udc, (void **)&dcctrl, NULL, 0, sizeof( *dcctrl ));
	if ( (udc->dc_data = dcctrl ) == NULL ) {
		error = ENOMEM;
		LOGS( udc, 0, "failed to alloc dcctrl structure");
		goto fail;
	}

	if ( udc->bs_init ){
		error = udc->bs_init( &udc->bs_data, 0 );
		if ( error ) {
			LOG1H( udc, 0, "chipidea_custom_init() failed err =", error);
			goto fail1;
		}
	}

	//get the back pointer
	dcctrl->udc					= udc;
	dcctrl->flags					= CHIP_IDEA_FLAGS_INITIAL_LINK_DISCONNECTED;
	dcctrl->verbosity				= udc->verbose;

	memset( dcctrl->phy_tuning, 0, sizeof( dcctrl->phy_tuning ) );
	dcctrl->serial_string = NULL;
	dcctrl->IoBase = (unsigned long)udc->base;

	dcparams = in32( dcctrl->IoBase + CHIP_IDEA_UOG_DCCPARAMS );

	if ( !( dcparams & CHIP_IDEA_DEVICE_CAPABLE ) ) {
		error = ENOTSUP;
		LOGS( udc, 0, "device mode not supported " );
		goto fail2;
	}

	// get number of supported endpoint
	dcctrl->num_ep = dcparams & CHIP_IDEA_MASK_DEN;

	if ( dcctrl->num_ep < udc->num_endpoints ){
		LOGS(udc, 0, "Controller does not have enough endpoints" );
		error = ENOTSUP;
		goto fail2;
	}

	if ( udc->bs_init ){
		error = udc->bs_init( &udc->bs_data, 1 );
		if ( error ) {
			LOG1H( udc, 0, "chipidea_custom_init1() failed err =", error);
			goto fail2;
		}
	}

	if ( ( error = chip_idea_chip_init( dcctrl ) ) != EOK ) {
		LOG1H(udc, 0, "chip_idea_chip_init() failed. error = ", error );
		goto fail2;
	}

	chip_idea_start(udc);

	return( EOK );

fail2:
	if(udc->bs_fini){
		udc->bs_fini( udc->bs_data, 0 );
	}
fail1:
	usb_free( udc, udc->dc_data );
fail:
	return error;
}

uint32_t
chip_idea_stop( usb_dc_t *udc )
{
	chip_ideadc	*dcctrl = udc->dc_data;

	/* clear interrupt enables */
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_USBINTR, 0 );
	out32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC, (~(CHIP_IDEA_OTG_IRQ_EN_MASK) & in32( dcctrl->IoBase + CHIP_IDEA_UOG_OTGSC )) );
	/* disable all INTR */

	/* disconnect from bus and stop the controller */
	chip_idea_set_bus_state( udc, USB_BUS_STATE_DISCONNECTED );

	return( EOK );
}

int
chip_idea_shutdown( usb_dc_t *udc )
{
	chip_ideadc	*dcctrl = udc->dc_data;

	chip_idea_stop( udc );

	chip_idea_chip_fini( dcctrl );

	// Add board specific shutdown
	usb_free(udc, udc->dc_data );

	return( EOK );
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/lib/usbboot/udc/chipidea.c $ $Rev: 823349 $")
#endif
