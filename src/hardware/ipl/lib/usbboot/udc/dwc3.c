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

#include "dwc3.h"

#define POW2_ROUNDUP

/////////////
// MACROS  //
/////////////
#define FIFOMEM_BLK_SIZE			8

#define BLKNUM_EP0_START			0
// 512 bytes + 2 * worst_case_buswidth in bytes
// so ( 512 + ( 2 * 16 ) ) / 8 =  (544 / 8 )  =  68
// where worst_case_buswidth in bytes = 128 bits / 8 = 16
#define EP0_FIFO_SZ					68

#define BLKNUM_POOL_START			( FIFOMEM_BLK_SIZE + EP0_FIFO_SZ )


/////////////////
// LOCAL TYPES //
/////////////////
struct _blk_descr {
	unsigned int                    start;
	unsigned int                    end;
	LIST_ENTRY(_blk_descr)          link;
};

struct _fifomem_ctrl {
	dctrl_t                       *dc;
	int                           MDWIDTH_bytes;
	LIST_HEAD(, _blk_descr )      free_list;
	LIST_HEAD(, _blk_descr )      used_ist;
};

static int
dwcotg_control_transfer( dctrl_t *dc, usb_endpoint_t *usb_ep, usb_transfer_t *urb, _uint32 flags );

////////////////////////////////////////////////////////////////////////////////
//                        PRIVATE FUNCTIONS                                   //
////////////////////////////////////////////////////////////////////////////////

unsigned log2_roundup( unsigned n ) {
	unsigned cnt = 0;
	unsigned n2 =  n;

	while( n != 1 ) {
		n = n >> 1;
		cnt++;
	}

	// test orignial number to see if it is a power of 2... if not then round up
	if ( n2 & ( n2 - 1) ) {
		cnt++;
	}

	return cnt;
}


////////////////////////////////////////////////////////////////////////////////
//                         PUBLIC FUNCTIONS                                   //
////////////////////////////////////////////////////////////////////////////////

void * fifomem_init( dctrl_t  *dc ) {
	struct _fifomem_ctrl	*ctrl = NULL;
	struct _blk_descr		*freeblock=NULL;
	unsigned				fifosz_in_blks;

	usb_calloc( dc->udc, (void **)&ctrl,  NULL, 0, sizeof( struct _fifomem_ctrl ) );
	if ( ctrl == NULL ) {
		LOGS(dc->udc, 0, "fifomem_init calloc failed0");
		goto fail;
	}

	fifosz_in_blks = HW_Read32( dc->IoBase, GHWPARAMS7 ) & RAM1_DEPTH_MSK;
	ctrl->MDWIDTH_bytes = ( HW_Read32( dc->IoBase, GHWPARAMS0 ) & MDWIDTH_MSK ) >> MDWIDTH_POS;
	ctrl->dc = dc;


	LIST_INIT( &ctrl->free_list );
	LIST_INIT( &ctrl->used_ist );

	// create free list, reserving blocks  for setup and ep0 fifos
	usb_calloc( dc->udc, (void **)&freeblock,  NULL, 0, sizeof( struct _blk_descr ) );
	if ( freeblock == NULL ) {
		LOGS(dc->udc, 0, "fifomem_init calloc2 failed" );
		goto fail2;
	}

	freeblock->start = BLKNUM_POOL_START;
	freeblock->end = fifosz_in_blks - 1;
	LIST_INSERT_HEAD( &ctrl->free_list, freeblock, link );

	return ctrl;

fail2:
	usb_free(ctrl->dc->udc, ctrl );
fail:
	return NULL;
}

void fifomem_fini( void * hdl ) {
	struct _fifomem_ctrl	*ctrl = hdl;
	usb_free(ctrl->dc->udc, ctrl);
}


// returns a blknum which represents the start of the fifo of specified size
int fifomem_alloc( void * hdl, int n_pkts, int mps, int * n_blks_allocated ) {
	struct _fifomem_ctrl	*ctrl = hdl;
	unsigned int			nblk,start;
	struct _blk_descr		*p, *usedblock;
	int						size;

	// the size equation is based on SuperSpeed USB 3.0 Controller Databook
	// chapter 6.5.1 which shows that the fifo size is dependent on the buswidth.
	// Experimentally, I have shown that the controller will not work without accouting
	// for MDWIDTH
	size = n_pkts * ( mps + ctrl->MDWIDTH_bytes ) + ctrl->MDWIDTH_bytes;
	if ( size <= FIFOMEM_BLK_SIZE ) {
		size = FIFOMEM_BLK_SIZE;
	} else  {
		size = 1 <<  log2_roundup( size );
	}

	nblk = size / FIFOMEM_BLK_SIZE;
	*n_blks_allocated = nblk;

	// walk free list looking to allocate nblk
	for ( p = LIST_FIRST(&ctrl->free_list );
		  p != NULL;
		  p = LIST_NEXT( p, link )                   ) {

		if ( ( p->end - p->start + 1 ) >= nblk ) {
			// great, this free list entry has enough blocks
			// remove from free pool
			start = p->start;
			p->start += nblk;

			if ( p->start > p->end ) {
				// this descriptor has no more blocks to give... remove
				LIST_REMOVE( p, link );
				usb_free(ctrl->dc->udc, p);
			}

			// add to used pool
			usb_calloc( ctrl->dc->udc, (void **)&usedblock,  NULL, 0, sizeof( struct _blk_descr ) );
			if ( usedblock == NULL ) {
				// out of descriptors!!!
				LOGS(ctrl->dc->udc, 0, "calloc failed0" );
				return -1;
			}
			usedblock->start = start;
			usedblock->end = start + nblk - 1;
			LIST_INSERT_HEAD( &ctrl->used_ist, usedblock, link );

			return start;
		}
	}

	// couldn't find a free block...
	return -1;
}

// free the blknum allocated with fifomem_alloc
void fifomem_free( void * hdl, int blknum ) {
	struct _fifomem_ctrl	*ctrl = hdl;
	struct _blk_descr		*p,*prev,*next,*freeblock;
	int						start,end;
	int						found = 0;

	// removed  blk from used list
	for ( p = LIST_FIRST(&ctrl->used_ist );
		  p != NULL;
		  p = LIST_NEXT( p, link )                   ) {

		  if ( p->start == blknum ) {
			  // remove from used list
			  start = p->start;
			  end = p->end;
			  LIST_REMOVE( p, link );
			  usb_free(ctrl->dc->udc, p);
			  found = 1;
			  break;
		  }
	}

	// add newly freed blocks to free list
	if ( found ) {
		for ( p = LIST_FIRST(&ctrl->free_list ),prev=NULL;
			  p != NULL;
			  prev=p, p = LIST_NEXT( p, link )                   ) {

			  if ( end == ( p->start - 1 ) ) {
				  // add newly freed blocks to the front-end of this free block
				  p->start = start;
				  //check to see if we need to coalesce with previous free block
				  if ( prev && ( prev->end == ( p->start - 1 ) ) ) {
					  p->start = prev->start;
					  LIST_REMOVE( prev, link );
					  usb_free(ctrl->dc->udc, prev);
				  }
				  break;

			  } else if ( start == ( p->end + 1 ) ) {
				  p->end = end;
				  //check to see if we need to coalesce with next free block
				  if ( (next = LIST_NEXT(p, link)) && ( next->start == ( p->end + 1 ) ) ) {
					  p->end = next->end;
					  LIST_REMOVE( next, link );
					  usb_free(ctrl->dc->udc, next);
				  }
				  break;

			  } else if ( end < ( p->start - 1 ) ) {
				  // add a new free block before p
				  usb_calloc(ctrl->dc->udc, (void **)&freeblock,  NULL, 0, sizeof( struct _blk_descr ) );
				  if ( freeblock == NULL ) {
					LOGS(ctrl->dc->udc, 0, "calloc failed1" );
					  return;
				  }
				  freeblock->start = start;
				  freeblock->end = end;
				  LIST_INSERT_BEFORE( p, freeblock, link );
				  break;
			  } else if ( LIST_NEXT( p, link ) == NULL ) {
				  // add a new free block before p
				 usb_calloc(ctrl->dc->udc, (void **)&freeblock,  NULL, 0, sizeof( struct _blk_descr ) );
				  if ( freeblock == NULL ) {
					LOGS(ctrl->dc->udc, 0, "calloc failed2" );
					  return;
				  }
				  freeblock->start = start;
				  freeblock->end = end;
				  LIST_INSERT_AFTER( p, freeblock, link );
				  break;
			  }
		}
	} else {
			LOGS(ctrl->dc->udc, 0, "block not found!" );
	}

	return;
}

int fifomem_alloc_ep0( void * hdl, int * n_blks_allocated ) {
	// return dedicated fifo
	*n_blks_allocated = EP0_FIFO_SZ;
	return BLKNUM_EP0_START;
}

void fifomem_free_ep0( void * hdl, int blknum ) {
	//nothing to do fifo is dedicated
}


//////////////////////////////
// Endpoint Commands        //
//////////////////////////////

void epcmd_build_DEPSTARTCFG( cmd_t * cmd, int xfer_rsc_idx ) {
	cmd->cmd = EPCMDTYP_DEPSTARTCFG | EPCMDACT | ( xfer_rsc_idx << EPCMDPARAM_XFER_RSC_IDX_POS );
	cmd->p0 = 0;
	cmd->p1 = 0;
	cmd->p2 = 0;
}

// dir == 0 : OUT, dir !=0 : IN
void epcmd_build_DEPCFG( cmd_t * cmd, ep_ctx_t * ep, uint8_t dir, uint8_t ignore_seqn ) {
	uint32_t epdir;
	uint32_t evt_msk;
	uint32_t bInterval_m1 = 0;

	switch ( ep->type ) {
		case USB_ATTRIB_CONTROL:
			epdir = ( dir ) ? EPCMD_PRM_EPDIR_IN : EPCMD_PRM_EPDIR_OUT;

			// only enable this event for control enpoints... it is required
			// to correctly implement the control transfer state machine
			evt_msk = EPCMD_PRM_XferNRdyEn | EPCMD_PRM_XferCmplEn;
			break;
		case USB_ATTRIB_ISOCHRONOUS:
			epdir = ( ep->dir ) ? EPCMD_PRM_EPDIR_IN : EPCMD_PRM_EPDIR_OUT;
			evt_msk = EPCMD_PRM_XferInProgEn | EPCMD_PRM_XferNRdyEn;

			// as per section 9.3.3 of the dwc3 databook
			if ( ep->dc->speed == CONNECTSPD_FULL1 || ep->dc->speed == CONNECTSPD_FULL2 ) {
				bInterval_m1 = 0;
			} else {
				bInterval_m1 = ( ep->usb_ep->edesc->bInterval - 1 );
			}
			break;
		case USB_ATTRIB_BULK:
		case USB_ATTRIB_INTERRUPT:
		default:
			epdir = ( ep->dir ) ? EPCMD_PRM_EPDIR_IN : EPCMD_PRM_EPDIR_OUT;
			//evt_msk = EPCMD_PRM_XferInProgEn;
			evt_msk = EPCMD_PRM_XferCmplEn;
			break;
	}

	cmd->cmd = EPCMDTYP_DEPCFG | EPCMDACT;

	cmd->p0 =	( ep->type << EPCMD_PRM_EPType_POS )			|
				( ep->mps << EPCMD_PRM_MPS_POS )				|
				( ep->fifonum << EPCMD_PRM_FIFONUM_POS )		|
				( ep->burstsz << EPCMD_PRM_BURST_SZ_POS )		|
				(( ignore_seqn ) ? EPCMD_PRM_SEQN_IGNORE : 0 );

	cmd->p1 =   evt_msk | epdir                                                        |
	            ( ep->num  << EPCMD_PRM_EPNUM_POS )                                    |
	            ( bInterval_m1 << EPCMD_PRM_bInterval_m1_POS );
	cmd->p2 = 0;

}

void epcmd_build_DEPSTRTXFER( cmd_t * cmd, uint64_t buf_phys, int len ) {
	cmd->cmd = EPCMDTYP_DEPSTRTXFER | EPCMDACT ;
	cmd->p0 = buf_phys >> 32;
	cmd->p1 = buf_phys  & 0xffffffff;
	cmd->p2 = 0;
}

void epcmd_build_DEPUPDXFER( cmd_t * cmd, uint32_t xfer_rsc_idx ) {
	cmd->cmd = ( xfer_rsc_idx << EVT_EPPARAM_XFER_RSC_IDX_POS )| EPCMDTYP_DEPUPDXFER;
	cmd->p0 = 0;
	cmd->p1 = 0;
	cmd->p2 = 0;
}

void epcmd_build_DEPENDXFER( cmd_t * cmd, uint32_t xfer_rsc_idx ) {
	cmd->cmd = ( xfer_rsc_idx << EVT_EPPARAM_XFER_RSC_IDX_POS )| EPCMDTYP_DEPENDXFER | EPCMDACT;
	cmd->p0 = 0;
	cmd->p1 = 0;
	cmd->p2 = 0;
}


void epcmd_build_DEPXFERCFG( cmd_t * cmd, int n_xfer_resources ) {
	cmd->cmd = EPCMDTYP_DEPXFERCFG | EPCMDACT;
	cmd->p0 = n_xfer_resources;
	cmd->p1 = 0;
	cmd->p2 = 0;
}

void epcmd_build_DEPSETSTALL( cmd_t * cmd ) {
	cmd->cmd = EPCMDTYP_DEPSETSTALL | EPCMDACT;
	cmd->p0 = 0;
	cmd->p1 = 0;
	cmd->p2 = 0;
}

void epcmd_build_DEPCSTALL( cmd_t * cmd ) {
	cmd->cmd = EPCMDTYP_DEPCSTALL | EPCMDACT;
	cmd->p0 = 0;
	cmd->p1 = 0;
	cmd->p2 = 0;
}


int epcmd_send( dctrl_t * dc,	ep_ctx_t *ep, uint32_t epidx, cmd_t * cmd )  {
	int err;
	int timeout = 10;

	LOG1H(dc->udc, 6, "epcmd_send: ep->num = ", ep->num);
	LOG1H(dc->udc, 6, "epidx=", epidx );

	HW_Write32( dc->IoBase, DEPCMDPAR0(epidx), cmd->p0 );
	HW_Write32( dc->IoBase, DEPCMDPAR1(epidx), cmd->p1 );
	HW_Write32( dc->IoBase, DEPCMDPAR2(epidx), cmd->p2 );
	HW_Write32( dc->IoBase, DEPCMD(epidx), cmd->cmd  );

	// wait for command to be accepted by the controller... this should never 
	while ( --timeout && ( HW_Read32( dc->IoBase, DEPCMD(epidx) ) & EPCMDACT ) ) {
		usb_delay_ns(dc->udc, 1000);
	}

	if ( timeout == 0 ) {
		LOG1H(dc->udc, 0, "epcmd_send timeout epidx = %d", epidx );
		err = ETIME;
		goto error;
	}

	// only valid for start xfer
	cmd->xfer_rsc_idx = ( HW_Read32( dc->IoBase, DEPCMD(epidx) ) & EVT_EPPARAM_XFER_RSC_IDX_MSK ) >> EVT_EPPARAM_XFER_RSC_IDX_POS;

	return EOK;

error:
	return err;
}


//////////////////////////////
// Device Generic Commands  //
//////////////////////////////
void dgcmd_build_FIFO_ALL_FLUSH( cmd_t * cmd ) {
	cmd->cmd = CMDTYP_ALLFIFO_FLUSH | CMDACT;
	cmd->p0 = 0;
}

// !!! Assumption !!! : dgcmd_send() is NOT thread safe, so it is assumed that
// the caller will synchronize commands... If we need to make this thread safe, I suggest
// adding a command mutex per endpoint
int dgcmd_send( dctrl_t * dc,	cmd_t * cmd  )  {
	int err;
	int timeout = 100;

	HW_Write32( dc->IoBase, DGCMDPAR, cmd->p0 );
	HW_Write32( dc->IoBase, DGCMD, cmd->cmd  );


	// wait for command to be accepted by the controller
	while ( --timeout && ( HW_Read32( dc->IoBase, DGCMD ) & CMDACT ) ) {
		usb_delay_ns(dc->udc, 1000);
	}

	if ( timeout == 0 ) {
		LOGS(dc->udc, 0, "dgcmd_send timeout " );
		err = ETIME;
		goto error;
	}

	return EOK;

error:
	return err;
}

int  trb_alloc( dctrl_t * dc, ep_ctx_t * ep ) {
	int             err;

	err = usb_malloc(dc->udc, (void **)&ep->trb, (paddr64_t *)&ep->trb_phys, 0, sizeof(trb_t) + 16);
	ep->trb_mem = ep->trb;

	//make 16 bytes aligned
	ep->trb = (trb_t *)(((uintptr_t)ep->trb + 16) & ~(16-1));
	ep->trb_phys = (ep->trb_phys + 16) & ~(16-1);

	if ( ep->trb == NULL ) {
		LOGS( dc->udc, 0, "failed to calloc trb struct");
	}
	else{
		LOG1H( dc->udc, 6, "allocated trb at paddr =", ep->trb_phys );
	}

	return err;
}

int  trb_free( dctrl_t * dc, ep_ctx_t * ep )
{
	usb_free(dc->udc, ep->trb_mem);
	ep->trb = ep->trb_mem = NULL;

	return( EOK );
}

/* Control Packet: Setup Phase */
void trb_build_ctrl_setup( trb_t * trb, uint64_t setup_packet_paddr ) {
	trb->bufptr_low = setup_packet_paddr & 0xffffffff;
	trb->bufptr_high = setup_packet_paddr >> 32;
	trb->bufsz_sts = SETUP_PACKET_SIZE;
	trb->control = TRB_CTL_HWO | TRB_CTL_LST | TRBCTL_CONTROL_SETUP | TRB_CTL_IOC;
}

/* Control Packet: Data Phase */
void trb_build_ctrl_data( trb_t * trb, uint64_t buf_paddr64, uint32_t size ) {
	trb->bufptr_low = buf_paddr64 & 0xffffffff;
	trb->bufptr_high = buf_paddr64 >> 32;
	trb->bufsz_sts = size;
	trb->control = TRB_CTL_HWO | TRB_CTL_LST | TRBCTL_CONTROL_DATA | TRB_CTL_IOC;
}

/* Control Packet: Status Phase ( without data phase ) */
void trb_build_ctrl_status2( trb_t * trb ) {
	trb->bufptr_low = 0;
	trb->bufptr_high = 0;
	trb->bufsz_sts = 0;
	trb->control = TRB_CTL_HWO | TRB_CTL_LST | TRBCTL_CONTROL_STATUS2 | TRB_CTL_IOC;
}

/* Control Packet: Status Phase ( with data phase ) */
void trb_build_ctrl_status3( trb_t * trb ) {
	trb->bufsz_sts = 0;
	trb->bufptr_low = 0;
	trb->bufptr_high = 0;
	trb->control = TRB_CTL_HWO | TRB_CTL_LST | TRBCTL_CONTROL_STATUS3 | TRB_CTL_IOC;
}

/* This function assumes the normal trbs are used as part of a circular ring of trbs. So, this
 * function also manages the ring enqueue index
 */
void trb_build_normal( trb_t * trb, uint64_t buf_phys, int len ) {
	// populate trb
	trb->bufptr_low = buf_phys & 0xffffffff;
	trb->bufptr_high = buf_phys >> 32;
	trb->bufsz_sts = len;
	trb->control = TRB_CTL_HWO | TRB_CTL_LST | TRBCTL_NORMAL | TRB_CTL_IOC;
//		trb_arr[enq_idx].control = TRB_CTL_HWO | TRBCTL_NORMAL | TRB_CTL_IOC | TRB_CTL_CSP;

}

static void complete_urb( dctrl_t * dc, ep_ctx_t *ep, uint32_t urb_status ) {
	usb_transfer_t *urb = ep->urb;
	if ( urb ) {
		// we have sent/received everyting ... complete the urb
		urb->actual_len = ep->bytes_xfered;
		urb->status = urb_status;
		LOG1H(dc->udc, 6, "URB COMPLETE epnum ",ep->num);
		LOG1H(dc->udc, 6, "Length ", urb->actual_len );
		if( urb->complete_cbf ){
			urb->complete_cbf( dc->udc, ep->usb_ep, urb);
		}
	}
}

static void complete_control_urb( dctrl_t * dc, ep_ctx_t *ep, uint32_t urb_status ) {
	usb_transfer_t *urb = ep->urb;

	if ( urb ) {
		// we have sent/received everyting ... complete the urb
		urb->actual_len = ep->bytes_xfered;
		urb->status = urb_status;
		ep->urb = 0;
		LOG1H(dc->udc, 5, "URB COMPLETE epnum ",ep->num);
		LOG1H(dc->udc, 5, "Length ", urb->actual_len );
		if( urb->complete_cbf ){
			urb->complete_cbf( dc->udc, ep->usb_ep, urb);
		}
	}
}

static void event_process_USBRst( dctrl_t * dc ) {
	int          i, err;
	ep_ctx_t   * ep;

	/* designware datasheet table 9-2
	 * "Initialization on USB Reset" section
	 */

	/* Step1: TODO
	 * DEPCMD0: If a control transfer is still in progress, complete it and get the
	 * core into the “Setup a Control-Setup TRB / Start Transfer” state
	 * (see “Control Transfer Programming Model” on page 512)
	 */

	/* Step2:
	 * DEPCMDn: Issue a DEPENDXFER command for any active transfers
	 * (except for the default control endpoint 0)
	 *
	 * Note: transfer_abort() will be called by the stack to do this... don't do anything here
	 */

	/* Step3:
	 * DEPCMDn: Issue a DEPCSTALL (ClearStall) command for any endpoint that was
	 * put into STALL mode prior to the USB Reset
	 */

	// start with i = 2 because indexes 0 and 1 are reserved for the default
	// control pipe EP0.
	for(i = 2; i < MAX_N_ENDPOINTS; i++ ) {
		ep = &dc->ep_arr[i];

		if ( ep->flags & EPFLAG_STALLED ) {
			cmd_t			epcmd;

			ep->flags &= ~EPFLAG_STALLED;
			epcmd_build_DEPCSTALL( &epcmd );
			err = epcmd_send( dc, ep, ep->idx, &epcmd );
			if ( err != EOK ) {
				LOG1H(dc->udc, 0, "failed to send DEPCSTALL command to epidx = ", ep->idx );
			}
		}
	}

	/* Step4:
	 * DCFG: Set DevAddr to ‘0’
	 */
	HW_Write32And( dc->IoBase, DCFG, ~DEVADDR_MSK );
}

static void event_process_ConnectDone( dctrl_t * dc ) {
	ep_ctx_t          * ep = &dc->ep_arr[0];
	cmd_t               epcmd;
	int                 err;

	dc->flags |= DC_FLAG_CONNECTED;
	dc->udc->usb_device_state_change( dc->udc, USB_DEVICE_STATE_INSERTED);

	LOGS(dc->udc, 5, "IOUSB_DEVICE_STATE_INSERTED" );

	/* Step1:
	 * DSTS: Read this register to obtain the connection speed
	 */
	dc->speed = ( HW_Read32( dc->IoBase, DSTS ) & CONNECTSPD_MSK ) >> CONNECTSPD_POS;

	 switch ( dc->speed ) {
		case CONNECTSPD_SUPER:
			LOGS(dc->udc, 2, "SUPER SPEED DETECTED" );
			ep->mps = 512;
			HW_Write32Or( dc->IoBase, GUSB2PHYCFG, SUSPHY );
			dc->udc->usb_set_device_speed( dc->udc, USB_DEVICE_SUPER_SPEED);
			break;

		case CONNECTSPD_HIGH:
			LOGS(dc->udc, 2, "HIGH SPEED DETECTED" );
			ep->mps = 64;
			dc->udc->usb_set_device_speed( dc->udc, USB_DEVICE_HIGH_SPEED);
			break;

		case CONNECTSPD_LOW:
			LOGS(dc->udc, 2, "LOW SPEED DETECTED" );
			 dc->udc->speed = USB_DEVICE_LOW_SPEED;
			dc->udc->usb_set_device_speed( dc->udc, USB_DEVICE_LOW_SPEED);
			break;

		case CONNECTSPD_FULL1:
		case CONNECTSPD_FULL2:
		default:
			LOGS(dc->udc, 2, "FULL SPEED DETECTED" );
			ep->mps = 64;
			dc->udc->usb_set_device_speed( dc->udc, USB_DEVICE_FULL_SPEED);
			break;
	 }

	// report the reset *after* the speed has been reported
	dc->udc->usb_device_state_change( dc->udc, USB_DEVICE_STATE_RESET );

	/* Step2:
	 * GCTL: Program the RAMClkSel field to select the correct clock for the RAM
	 * clock domain. This field is reset to 0 after USB reset, so it must be
	 * reprogrammed each time on Connect Done.
	 */

	 // Do Nothing as written in the OMAP 5 refmanual

	/* Step3:
	 * DEPCMD0/DEPCMD1: Issue a DEPCFG command (with ‘Ignore Sequence Number’
	 * field set to 1) for physical endpoints 0 & 1 using the same endpoint
	 * characteristics from Power-On Reset, but set MaxPacketSize to
	 * 512 (SuperSpeed), 64 (High-Speed), 8/16/32/64 (Full-Speed), or 8 (Low-Speed).
	 */
	 // re-adjust the mps based on speed
	epcmd_build_DEPCFG( &epcmd, ep, 0, 1 );
	err = epcmd_send( dc, ep, 0, &epcmd );
	if ( err != EOK ) {
		LOGS(dc->udc, 0, "failed to send DEPCFG command to epidx = 0  " );
	}

	epcmd_build_DEPCFG( &epcmd, ep, 1, 1 );
	err = epcmd_send( dc, ep, 1, &epcmd );
	if ( err != EOK ) {
		LOGS(dc->udc, 0, "failed to send DEPCFG command to epidx = 1 " );
	}

	// wait for DEPCFG commands to complete... TODO make sure we are actually done commands
	usb_delay(dc->udc, 1);


	/* Step4: TODO
	 * GUSB2CFG/GUSB3PIPECTL: Depending on the connected speed, write to the
	 * other PHY’s control register to suspend it
	 */

	/* Step5:   This is optional, so we don't do it for now...
	 * GTXFIFOSIZn: (optional) Based on the new MaxPacketSize of IN endpoint 0,
	 * software may choose to re-allocate the TX FIFO sizes by writing to these registers.
	 */

	/*
	 * delay first setup packet detected after insertion to give the extract
	 * code a chance to run and cleanup
	 */
	ep->setup_packet_delay = 0;
}

static void event_process_DisconnEvt( dctrl_t * dc ) {
	cmd_t			 gcmd;

	LOGS(dc->udc, 2, "IOUSB_DEVICE_STATE_REMOVED" );

	// flush all fifos on disconnect
	dgcmd_build_FIFO_ALL_FLUSH(&gcmd);
	dgcmd_send( dc, &gcmd );

	dc->flags &= ~DC_FLAG_CONNECTED;
	dc->udc->usb_device_state_change( dc->udc, USB_DEVICE_STATE_REMOVED);
}

static void event_process_XferComplete( dctrl_t * dc, evt_t * decoded_event ) {
	ep_ctx_t	*ep = &dc->ep_arr[decoded_event->epidx];
	trb_t  *trb = ep->trb;

	if ( ( ep->flags & EPFLAG_XFER_ACTIVE ) == 0 ) {
		// transfer was likely aborted
		return;
	}

	if ( trb->control & TRB_CTL_HWO ) {
		LOG1H(dc->udc, 0, "TRB_CTL_HWO set for trb. epidx = ", decoded_event->epidx );
		return;
	}

	ep->flags &= ~EPFLAG_XFER_ACTIVE;

	ep->bytes_xfered += ep->req_xfer_len - ( trb->bufsz_sts & TRB_BUFSZ_MSK );

	//if ( complete ) {
		complete_urb( dc, ep, 0 );
	//}
	//more transfer needed

}

static void event_process_XferComplete_ep0( dctrl_t * dc, evt_t * decoded_event ) {
	ep_ctx_t	*ep = &dc->ep_arr[0];
	uint32_t    count;

	if ( ep->flags & EPFLAG_XFER_ACTIVE ) {
		ep->flags &= ~(EPFLAG_XFER_ACTIVE | EPFLAG_XFER_NOT_READY);

		switch ( ep->control_phase ) {
			case CONTROL_PHASE_SETUP:

				count = SETUP_PACKET_SIZE - ( ep->trb->bufsz_sts & TRB_BUFSZ_MSK );

				if ( count == SETUP_PACKET_SIZE ) {
					if ( !ep->setup_packet_delay ) {
						LOGS(dc->udc, 6, "setup packet received, but delay processing" );
						ep->setup_packet_delay = 1;
					} else {
#if 0
						LOG(dc->udc, 6, "%s: rxpkt = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x"
							, __func__ , ep->setup_packet[0], ep->setup_packet[1], ep->setup_packet[2], ep->setup_packet[3]
							, ep->setup_packet[4], ep->setup_packet[5], ep->setup_packet[6], ep->setup_packet[7] );
#endif
						usb_setup_packet_process(dc->udc, ep->setup_packet );
					}

				} else {
					LOG1H(dc->udc, 0, "Expected an 8-byte setup packet, but received ", count );
					dwcotg_setup_packet_get( dc );
				}

				break;

			case CONTROL_PHASE_DATA:
				LOGS(dc->udc, 6, "Completed Data Phase: CONTROL_PHASE_DATA... " );
				count = ep->req_xfer_len -  ( ep->trb->bufsz_sts & TRB_BUFSZ_MSK );
				ep->bytes_xfered = count;
				complete_control_urb( dc, ep, 0 );
				break;

			case CONTROL_PHASE_STATUS:
				LOGS(dc->udc, 6, "Completed Status Phase: CONTROL_PHASE_STATUS... waiting for next Setup Packet " );
				complete_control_urb( dc, ep, 0 );

				ep->control_phase = CONTROL_PHASE_SETUP;
				dwcotg_setup_packet_get( dc );

				break;
			}
	} else {
		LOG1H(dc->udc, 0, "XferComplete Event.. but not active transfer on ep->num = ", ep->num );
	}
}

static void event_process_EPCmdCmplt( dctrl_t * dc, evt_t * decoded_event ) {
}

static void event_process_XferNotReady_ep0( dctrl_t * dc, evt_t * decoded_event ) {
	ep_ctx_t	*ep = &dc->ep_arr[0];

	ep->flags |= EPFLAG_XFER_NOT_READY;
	if( ep->control_phase == CONTROL_PHASE_SETUP && ep->setup_packet_delay ){
		ep->setup_packet_delay = 0;
#if 0
		LOG(dc->udc, 6, "%s: rxpkt = 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x"
			, __func__ , ep->setup_packet[0], ep->setup_packet[1], ep->setup_packet[2], ep->setup_packet[3]
			, ep->setup_packet[4], ep->setup_packet[5], ep->setup_packet[6], ep->setup_packet[7] );
#endif
		usb_setup_packet_process(dc->udc, ep->setup_packet );
	}
	else{
		if ( !(ep->flags & EPFLAG_XFER_ACTIVE) ) {
			dwcotg_control_transfer(dc, ep->usb_ep, ep->urb, ep->xfer_flags);
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//                               PUBLIC                                       //
////////////////////////////////////////////////////////////////////////////////

void event_process( dctrl_t * dc, evt_t * decoded_event ) {

	switch( decoded_event->evt_type ) {

		case USBRst:
			event_process_USBRst( dc );
			break;

		case ConnectDone:
			event_process_ConnectDone( dc );
			break;

		case DisconnEvt:
			event_process_DisconnEvt( dc );
			break;

		case XferComplete:
			if ( decoded_event->epidx < 2 ) {
				event_process_XferComplete_ep0( dc, decoded_event );
			}
			else{
				event_process_XferComplete( dc, decoded_event );
			}
			break;

		case EPCmdCmplt:
			event_process_EPCmdCmplt( dc, decoded_event );
			break;

		case XferNotReady:
			if ( decoded_event->epidx < 2 ) {
				event_process_XferNotReady_ep0( dc, decoded_event );
			}
			break;

		default:
			break;
	}

}

// 'event_decode' doesn't do a full decode of the event, but just enough to get
// the event type.  Further decode can be achieved via
// defined marcros in the dwcotg header.
void event_decode( dctrl_t * dc, uint32_t raw_event, evt_t * decoded_event ) {

	decoded_event->raw_event = raw_event;
	if ( ( raw_event & EVT_TYPE_MSK ) == EVT_TYPE_EP ) {
		// Endpoint Event

		decoded_event->evt_class = EVT_CLASS_EP;
		decoded_event->epidx = ( raw_event & EVT_EP_NUM_MSK ) >> EVT_EP_NUM_POS;


		switch( raw_event & EVT_EP_TYPE_MSK ) {
			case EVT_EP_XferComplete:
				decoded_event->evt_type = XferComplete;
				break;
			case EVT_EP_XferNotReady:
				decoded_event->evt_type = XferNotReady;
				break;
			case EVT_EP_EPCmdCmplt:
				decoded_event->evt_type = EPCmdCmplt;
				break;
			default:
				decoded_event->evt_type = UnknownEvent;
				break;
		}

//		LOG(dc->udc, 6, "%s() evt_type %d epnum=%d, epidx = %d, raw=%x",__func__, decoded_event->evt_type, EPNUM(decoded_event->epidx), decoded_event->epidx, raw_event );

	} else {

		switch ( raw_event & EVT_TYPE2_MASK ) {
			case EVT_TYPE2_DEVICE:
				decoded_event->evt_class = EVT_CLASS_DEVICE;

				switch ( raw_event & EVT_DEV_MSK ) {
					case EVT_DEV_DisconnEvt:
						decoded_event->evt_type = DisconnEvt;
						break;
					case EVT_DEV_USBRst:
						decoded_event->evt_type = USBRst;
						break;
					case EVT_DEV_ConnectDone:
						decoded_event->evt_type = ConnectDone;
						break;
					case EVT_DEV_ULStChng:
						decoded_event->evt_type = ULStChng;
						break;
					case EVT_DEV_CmdCmplt:
						decoded_event->evt_type = CmdCmplt;
						break;
					default:
						decoded_event->evt_type = UnknownEvent;
						break;
				}

//				LOG(dc->udc, 6, "%s() evt_type %d, raw=%x",__func__, decoded_event->evt_type, raw_event );
				break;

			default:
				decoded_event->evt_class = EVT_UNKNOWN;
//				LOG(dc->udc, 0, "%s() Unrecognized Event raw = 0x%x",__func__,raw_event );
				break;
		}

	}
}

int dwcotg_xfer_start( dctrl_t * dc, ep_ctx_t * ep, uint32_t epidx ) {
	cmd_t       epcmd;
	int            rc = EOK;

	/* The current implementation only supports 1 active transfer per endpoint at a time.
	 * Given this limitation, this check for no active xfers makes sense.  If we want to add multiple
	 * xfers per endpoint, then the transfer functions  need to re-implemented
	 * to use a trb hooking model or something...... maybe dwcotg_xfer_start() to get things
	 * started, then xfer_hook(trb) or something
	 */

	if ( ( ep->flags & EPFLAG_ENABLED ) && !( ep->flags & EPFLAG_XFER_ACTIVE ) ) {
		ep->flags |= EPFLAG_XFER_ACTIVE;

		LOG(dc->udc, 6, "%s: epidx = %d control = 0x%x bufsz_sts = 0x%x paddr_hi = 0x%x paddr_lo = 0x%x ", __func__, epidx, trb->control, trb->bufsz_sts, trb->bufptr_high, trb->bufptr_low );

		//  make sure memory is updated before telling the controller to process the trb
		usb_bus_sync(dc->udc, 0);

		epcmd_build_DEPSTRTXFER( &epcmd, ep->trb_phys, 0 );
		rc = epcmd_send( dc, ep, epidx, &epcmd );
		if ( rc != EOK ) {
			LOG1H(dc->udc, 0, "failed to send epcmd_build_DEPSTRTXFER command to epidx = ", epidx );
		}
		ep->xfer_rsc_idx = epcmd.xfer_rsc_idx;

	} else {
		rc = -1;
		if ( !( ep->flags & EPFLAG_ENABLED ) ) {
			LOG1H(dc->udc, 0, "endpoint isn't enabled idx ", epidx );
		} else if ( ep->flags & EPFLAG_XFER_ACTIVE ) {
			LOG1H(dc->udc, 0, "endpoint already has an active transfer idx ", epidx );
		}
	}

	return rc;
}

int dwcotg_setup_packet_get( dctrl_t * dc ) {
	ep_ctx_t * ep = &dc->ep_arr[0];
	int        rc;

	trb_build_ctrl_setup( ep->trb, ep->setup_packet_paddr64 );
	rc = dwcotg_xfer_start( dc, ep, 0 );
	if ( rc != EOK ) {
		LOG1H(dc->udc, 0, "failed to hook setup_packet TRB on epidx -  ", ep->idx );
	}

	return rc;
}


int dwcotg_xfer_abort( dctrl_t * dc, ep_ctx_t *ep, int epidx ) {
	cmd_t			epcmd;
	int             rc;

	LOG1H(dc->udc, 7, "aborting epidx =", epidx );

	epcmd_build_DEPENDXFER( &epcmd, ep->xfer_rsc_idx );
	rc = epcmd_send( dc, ep, epidx, &epcmd );
	if ( rc != EOK ) {
		LOG1H(dc->udc, 0, "failed to send DEPENDXFER command to epidx = ", epidx );
	}

	return rc;
}


int dwcotg_interrupt( usb_dc_t *udc, uint32_t *emask )
{
	dctrl_t				*dc = udc->dc_data;
	uint32_t            evntcount_bytes, count;
	uint32_t	        raw_event;
	evt_t               decoded_event;
	volatile uint32_t   *eventq_mem = dc->eventq_mem;
	int                 timeout;

	while ( ( count = evntcount_bytes = ( HW_Read32( dc->IoBase, GEVNTCOUNT ) & EVNTCOUNT_MSK  ) ) ) {

		if ( count % DWCOTG_EVT_SZ ) {
			LOG1H(dc->udc, 0, "evntcount_bytes is not modulo ", count );
		}

		while ( count ) {

			timeout = 10;
			while ( !( raw_event = eventq_mem[ dc->eventq_cur_elem ] ) && timeout ) {
				LOGS(dc->udc, 0, "raw_event has not been updated by the hardware!!! timeout" );
				usb_delay_ns(udc, 100);
				timeout--;
			}

			LOG(dc->udc, 6, "%s: count = %d raw = 0x%x", __func__, count, raw_event );

			event_decode( dc, raw_event, &decoded_event );
			event_process( dc, &decoded_event );

			// zero out the event so we know when the hardware has actually updated the memory next time
			eventq_mem[ dc->eventq_cur_elem ] = 0;

			/*
			 * TODO: add handling for "Vendor Device Test LMP Received" event
			 * which is the only event which isn't 4 bytes.  For now, only
			 * support 4-byte events because the LMP Received event isn't enabled
			 */

			dc->eventq_cur_elem = ( dc->eventq_cur_elem + 1 ) % DWCOTG_N_EVT;
			count -= DWCOTG_EVT_SZ;
		}

		// Tell the hardware how many events were processed
		HW_Write32( dc->IoBase, GEVNTCOUNT, evntcount_bytes );
	}

	return EOK;
}

int
dwcotg_set_device_address(  usb_dc_t *udc, uint8_t address )
{
	dctrl_t	*dc = udc->dc_data;
	ep_ctx_t	*ep = &dc->ep_arr[0];
	int         err;

	LOG1H(dc->udc, 5, "set address = ", address);

	/* designware datasheet table 9-4: Initialization on SetAddress Request */

	/* Step 1:
	 * DCFG: Program the DCFG register with the device address received as part
	 * of the SetAddress request when SETUP packet is decoded.
	 */
	HW_Write32Or( dc->IoBase, DCFG,  address << DEVADDR_POS  );
	usb_delay(dc->udc, 1);


	/* Step 2:
	 *  DEPCMD1 : After receiving the XferNotReady(Status) event, acknowledge
	 * the status stage by issuing a DEPSTRTXFER command pointing to a Status TRB.
	 * This step must be done after the DCFG register is programmed with the new
	 * device address
	 *
	 */

	ep->control_phase = CONTROL_PHASE_STATUS;
	trb_build_ctrl_status2( ep->trb );

	err = dwcotg_xfer_start( dc, ep, 1 );
	if ( err != EOK ) {
		LOGS(dc->udc, 0, "failed to hook status2 trb on epidx 1 to complete status-IN phase  " );
	}

	udc->usb_device_state_change( udc, USB_DEVICE_STATE_ADDRESSED);

	return EOK;
}


int
dwcotg_select_configuration( usb_dc_t *udc, uint8_t config )
{
	dctrl_t			*dc = udc->dc_data;
	ep_ctx_t        *ep = &dc->ep_arr[0];
	cmd_t			epcmd;
	int             err;


	LOG1H(dc->udc, 2, "set config = ", config);

	/* Issue a DEPCFG command ( with ' Ignore Sequence Numer field set to 1 ) for
	 * physical endpoint 1 using current endpoint characteristics to re-initialize the TX FIFO allocation
	 */

	epcmd_build_DEPCFG( &epcmd, ep, 1, 1 );
	err = epcmd_send( dc, ep, 1, &epcmd );
	if ( err != EOK ) {
		LOG(dc->udc, 0, "failed to send DEPCFG command to epidx = 1 ", __func__ );
	}

	/* Issue a DEPSTARTCFG command  with XferRscIdx set to 2 to re-initialize the transfer resource allocation */
	epcmd_build_DEPSTARTCFG( &epcmd,2 );
	err = epcmd_send( dc, ep, 0, &epcmd );
	if ( err != EOK ) {
		LOG(dc->udc, 0, "%s: failed to send DEPSTARTCFG command to epidx = 0 ", __func__ );
	}

	udc->usb_device_state_change( udc, USB_DEVICE_STATE_CONFIGED);

	return EOK;
}


int
dwcotg_set_endpoint_state( usb_dc_t *udc, usb_endpoint_t *usb_ep, uint32_t ep_state )
{
	dctrl_t			*dc = udc->dc_data;
	ep_ctx_t		*ep = usb_ep->user;
	cmd_t		epcmd;
	int             err;

	LOG1H(dc->udc, 5, "dwcotg_set_endpoint_state = ", ep_state );

	switch ( ep_state ) {
		case USB_ENDPOINT_STATE_READY :
			LOG(dc->udc, 5, "%s: IOUSB_ENDPOINT_STATE_READY epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;

		case USB_ENDPOINT_STATE_STALLED :
			LOG(dc->udc, 5, "          STALL epnum = 0x%x",  usb_ep->edesc->bEndpointAddress);

			epcmd_build_DEPSETSTALL( &epcmd );
			err = epcmd_send( dc, ep, ep->idx, &epcmd );
			if ( err != EOK ) {
				LOG(dc->udc, 0, "%s: failed to send DEPSETSTALL command to epidx = %d ", __func__, ep->idx );
			}

			ep->flags |= EPFLAG_STALLED;

			if ( ep->num == 0 ) {
				// Get ready for next setup packet.
				ep->control_phase = CONTROL_PHASE_SETUP;
				dwcotg_setup_packet_get( dc );
			}

			break;

		case USB_ENDPOINT_STATE_RESET :
			LOG(dc->udc, 5, "%s: IOUSB_ENDPOINT_STATE_RESET epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;
		case USB_ENDPOINT_STATE_ENABLE :
			LOG(dc->udc, 5, "%s: IOUSB_ENDPOINT_STATE_ENABLE epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;

		case USB_ENDPOINT_STATE_DISABLED :
			LOG(dc->udc, 5, "%s: IOUSB_ENDPOINT_STATE_DISABLED epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;

		case USB_ENDPOINT_STATE_NAK :
			LOG(dc->udc, 5, "%s: IOUSB_ENDPOINT_STATE_NAK epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;

		default :
			break;
	}

	return EOK;
}

int
dwcotg_clear_endpoint_state( usb_dc_t *udc, usb_endpoint_t *usb_ep, uint32_t ep_state )
{
	dctrl_t			*dc = udc->dc_data;
	ep_ctx_t		*ep = usb_ep->user;
	cmd_t		epcmd;
	int             err;

	switch ( ep_state ) {
		case USB_ENDPOINT_STATE_READY :
			LOG(dc->udc, 5, "%s: CLEAR IOUSB_ENDPOINT_STATE_READY  epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;

		case USB_ENDPOINT_STATE_STALLED :
			LOG(dc->udc, 5, "          Clear STALL  epnum = 0x%x",  usb_ep->edesc->bEndpointAddress);

			epcmd_build_DEPCSTALL( &epcmd );
			err = epcmd_send( dc, ep, ep->idx, &epcmd );
			if ( err != EOK ) {
				LOG(dc->udc, 0, "%s: failed to send DEPCSTALL command to epidx = %d ", __func__, ep->idx );
			}
			ep->flags &= ~EPFLAG_STALLED;

			break;

		case USB_ENDPOINT_STATE_RESET :
			LOG(dc->udc, 5, "%s: CLEAR IOUSB_ENDPOINT_STATE_RESET  epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;

		case USB_ENDPOINT_STATE_ENABLE :
			LOG(dc->udc, 5, "%s: CLEAR IOUSB_ENDPOINT_STATE_ENABLE  epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;

		case USB_ENDPOINT_STATE_DISABLED :
			LOG(dc->udc, 5, "%s: CLEAR IOUSB_ENDPOINT_STATE_DISABLED  epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;

		case USB_ENDPOINT_STATE_NAK :
			LOG(dc->udc, 5, "%s: CLEAR IOUSB_ENDPOINT_STATE_NAK  epnum = 0x%x", __func__, usb_ep->edesc->bEndpointAddress);
			break;

		default :
			break;
	}

	return EOK;
}


int dwcotg_disconnect( dctrl_t  * dc ) {
	int         timeout = 10; // ms
	int         rc = EOK;
	int         i;
	uint32_t    reg;
	ep_ctx_t    *ep;

	LOGS(dc->udc, 5, "Aborting Transfer and stopping the controller" );

	//unidrectional non-control endpoints assumed
	for( i=2; i < dc->n_ep; i++ ) {
		ep = &dc->ep_arr[i];
		if ( ep->flags & EPFLAG_XFER_ACTIVE ) {
			// stop the controller  from processing transfers
			dwcotg_xfer_abort( dc, ep, ep->idx );
			ep->flags &= ~EPFLAG_XFER_ACTIVE;
		}
	}

	usb_delay(dc->udc, 1 );

	// do soft disconnect
	reg = HW_Read32( dc->IoBase, DCTL );
	reg &= ~( RUNSTOP );
	HW_Write32( dc->IoBase, DCTL, reg );

	while( --timeout && !( HW_Read32( dc->IoBase, DSTS ) & DEVCTRLHLT ) ) {
		usb_delay(dc->udc, 1 );
	}

	if ( timeout == 0 ) {
		LOGS(dc->udc, 0, "Controller didnt't halt  ( DSTS & DEVCTRLHLT ) == true ");
		rc = ETIME;
	}

	return rc;
}

/* call to enable connection to host or signal resume to host */
int
dwcotg_set_bus_state( usb_dc_t *udc, uint32_t device_state )
{
	dctrl_t  *dc = udc->dc_data;
	uint32_t reg;

	switch ( device_state ) {
	case USB_BUS_STATE_DISCONNECTED :
		dwcotg_disconnect( dc );
		break;

	case USB_BUS_STATE_CONNECTED :
		// do soft connect
		// start the controller
		reg = HW_Read32( dc->IoBase, DCTL );
		reg &= ~( TRGTULST_MSK );
		reg |= TRGTULST_RX_DET | RUNSTOP;
		HW_Write32Or( dc->IoBase, DCTL, reg );

	case USB_BUS_STATE_RESUME :
		break;
	}

	return EOK;
}


/* This function is meant to configure ep0 and should only be called at
 * driver init time
 */
int ep0_cfg( dctrl_t * dc ) {
	ep_ctx_t			*ep = &dc->ep_arr[0];
	cmd_t                   epcmd;
	int                     err;


	ep->mps = 512;
	ep->num = 0;
	ep->dir = 0;
	ep->idx = 0;
	ep->type = USB_ATTRIB_CONTROL;
	ep->fifonum = 0;
	ep->burstsz = 0;
	ep->urb = 0;
	ep->xfer_flags = 0;
	ep->req_xfer_len = 0;

	ep->flags = EPFLAG_ENABLED;

	// allocate transfer descriptors for this endpoint
	trb_alloc( dc, ep );
	if ( ep->trb == NULL ) {
		LOGS(dc->udc, 0, "failed to allocated trb" );
		err = ENOMEM;
		goto error;
	}

	// allocate setup packet
	usb_malloc( dc->udc, (void **)&ep->setup_packet, &ep->setup_packet_paddr64, 0, SETUP_PACKET_SIZE);
	if ( ep->setup_packet == NULL ) {
		LOGS(dc->udc, 0, "failed to allocate setup packet buffer");
		err = ENOMEM;
		goto error2;
	}

	LOG1H(dc->udc, 5, "Setup Packet Buffer Paddr = ", ep->setup_packet_paddr64 );


	/* Issue a DEPSTARTCFG command with DEPCMD0.XferRscIdx set to 0 and CmdIOC set to 0 to
	 * initialize the transfer resource allocation. Poll CmdAct for completion
	 * Only do this on first call
	 */
	epcmd_build_DEPSTARTCFG( &epcmd,0 );
	err = epcmd_send( dc, ep, 0, &epcmd );
	if ( err != EOK ) {
		LOG(dc->udc, 0, "%s: failed to send DEPSTARTCFG command to epidx = 0 ", __func__ );
	}

	/*
	 * Configure EP0.... both epidx0 ( OUT ) and epidx1(IN) need to be configured
	 * Needs to be called every time to update the endpoint MPS
	 */

	epcmd_build_DEPCFG( &epcmd, ep, 0, 0 );
	err = epcmd_send( dc, ep, 0, &epcmd );
	if ( err != EOK ) {
		LOG(dc->udc, 0, "%s: failed to send DEPCFG command to epidx = 0  ", __func__ );
	}

	epcmd_build_DEPCFG( &epcmd, ep, 1, 0 );
	err = epcmd_send( dc, ep, 1, &epcmd );
	if ( err != EOK ) {
		LOG(dc->udc, 0, "%s: failed to send DEPCFG command to epidx = 1 ", __func__ );
	}

	epcmd_build_DEPXFERCFG( &epcmd, 1 );
	err = epcmd_send( dc, ep, 0, &epcmd );
	if ( err != EOK ) {
		LOG(dc->udc, 0, "%s: failed to send DEPXFERCFG command to epidx = 0  ", __func__ );
	}

	epcmd_build_DEPXFERCFG( &epcmd, 1 );
	err = epcmd_send( dc, ep, 1, &epcmd );
	if ( err != EOK ) {
		LOG(dc->udc, 0, "%s: failed to send DEPXFERCFG command to epidx = 1 ", __func__ );
	}

	// enable endpoint 0 ( epidx0 and epidx1 )
	HW_Write32Or( dc->IoBase, DALEPENA, 3 );


	// Get Ready for Next Setup Packet... possibly after re-insertion
	LOGS(dc->udc, 5, "Get Ready for First Setup Packet");
	ep->control_phase = CONTROL_PHASE_SETUP;
	dwcotg_setup_packet_get( dc );

	return EOK;

error2:
	trb_free( dc, ep );
error:
	return err;
}

void ep0_decfg( dctrl_t * dc ) {
	ep_ctx_t		*ep = &dc->ep_arr[0];

	// disable endpoint 0... i.e. both epidx0 and epidx1
	HW_Write32And( dc->IoBase, DALEPENA, ~3 );

	usb_free(dc->udc, ep->setup_packet );

	trb_free( dc, ep );

	ep->flags &= ~EPFLAG_ENABLED;
}

// used by both bulk and interrupt endpoints
int
dwcotg_endpoint_enable( usb_dc_t *udc, usb_endpoint_t *usb_ep )
{
	dctrl_t		*dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t	*ep = usb_ep->user;
	uint32_t	epnum = usb_ep->edesc->bEndpointAddress & USB_ENDPOINT_MASK;
	uint32_t	epdir = usb_ep->edesc->bEndpointAddress & USB_ENDPOINT_IN;
	cmd_t     epcmd;
	int		err;
	int		first_call = 0;
	int          addr,sz;

	LOG1H(dc->udc, 5, "enable ep = ", usb_ep->edesc->bEndpointAddress);

	if( epnum == 0 ){
		/* ep0 has already been configured in ep0_cfg()... all we have to do is link it to the
		 * generic parent struct. This code will have to change to support non ep0 control enpdpoints
		 */
		ep = usb_ep->user = &dc->ep_arr[0];
		ep->usb_ep = usb_ep;	// backref

		return EOK;
	}

	if ( !usb_ep->user ) {
		first_call = 1;
		ep = usb_ep->user = &dc->ep_arr[EPIDX( epnum, epdir )];
		ep->usb_ep = usb_ep;	// backref
		ep->dc = dc;
		ep->mps = usb_ep->edesc->wMaxPacketSize;
		ep->num = epnum;
		ep->dir = epdir;
		ep->idx = EPIDX( epnum, epdir );
		ep->type = usb_ep->edesc->bmAttributes & 3;
		ep->flags = EPFLAG_ENABLED;

		/* IN Endpoints use separate TX fifos corresponding to the endpoint number
		 OUT endpoints use FIFO 0 */
		ep->fifonum = ( epdir ) ? epnum : 0;

		/* hardcode super speed bulk endpoints burst size to 15...
		 * companion descriptor must match
		 */

		if ( ep->type == USB_ATTRIB_BULK ) {
			ep->burstsz = 0xf;
		} else {
			ep->burstsz = 0x0;
		}

		trb_alloc( dc, ep );
		if ( ep->trb == NULL ) {
			LOG(dc->udc, 0, "%s: failed to allocated trb cluster... epnum = 0x%x epdir = 0x%x", __func__, epnum, epdir  );
			err = ENOMEM;
			goto error;
		}

		if ( ep->dir ) {
			// alloc txfifo
			addr = fifomem_alloc( dc->fifomem_hdl, 2, ep->mps, &sz );
			if ( addr < 0 ) {
				LOG1H(dc->udc, 0, "fifomem_alloc() failed for epnum =", epnum  );
				err = ENOMEM;
				goto error2;
			}
			HW_Write32( dc->IoBase, GTXFIFOSIZ( ep->fifonum ), ( addr << TXFSTADDR_POS ) | sz  );
		} // else { // rxfifos are shared}

		epcmd_build_DEPCFG( &epcmd, ep, epdir, 0 );
		err = epcmd_send( dc, ep, ep->idx, &epcmd );
		if ( err != EOK ) {
			LOG(dc->udc, 0, "%s: failed to send DEPCFG command to epidx = %d  ", __func__, ep->idx );
		}

		epcmd_build_DEPXFERCFG( &epcmd, 1 );
		err = epcmd_send( dc, ep, ep->idx, &epcmd );
		if ( err != EOK ) {
			LOG(dc->udc, 0, "%s: failed to send DEPXFERCFG command to epidx = %d  ", __func__, ep->idx );
		}
			// enable the endpoint
		HW_Write32Or( dc->IoBase, DALEPENA, EPMSK(ep->idx) );


	}

	return EOK;
error2:
	if ( first_call ) {
		trb_free( dc, ep );
	}
error:
	return err;
}


int
dwcotg_endpoint_disable( usb_dc_t *udc, usb_endpoint_t *usb_ep )
{
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t	*ep = usb_ep->user;
	cmd_t		epcmd;

	if ( ep ) {
		LOG1H(dc->udc, 5, "ep disable epnum = ", ep->num  );

		if( ep->num == 0){
			//nothing need to be done
			return( EOK );
		}

		if ( ep->flags & EPFLAG_STALLED ) {
			int err;

			LOG(dc->udc, 0, "%s:          Clear STALL  epnum = 0x%x",__func__,   usb_ep->edesc->bEndpointAddress);

			epcmd_build_DEPCSTALL( &epcmd );
			err = epcmd_send( dc, ep, ep->idx, &epcmd );
			if ( err != EOK ) {
				LOG(dc->udc, 0, "%s: failed to send DEPCSTALL command to epidx = %d ", __func__, ep->idx );
			}
			ep->flags &= ~EPFLAG_STALLED;
		}

		HW_Write32And( dc->IoBase, DALEPENA, ~EPMSK(ep->idx) );
		trb_free( dc, ep );

		if ( ep->dir ) {
			fifomem_free( dc->fifomem_hdl, HW_Read32( dc->IoBase, GTXFIFOSIZ( ep->fifonum )) >> TXFSTADDR_POS );
		}

		ep->flags &= ~EPFLAG_ENABLED;
		usb_ep->user = 0;
	}

	return ( EOK );
}

static int
dwcotg_control_transfer( dctrl_t *dc, usb_endpoint_t *usb_ep, usb_transfer_t *urb, _uint32 flags )
{
	ep_ctx_t        *ep = usb_ep->user;
	uint32_t         idx = ( flags & PIPE_FLAGS_TOKEN_IN ) ? 1 : 0;
	int               err, length;

	// wait for the controller to report XFER_NOT_READY
	ep->urb = urb;
	ep->xfer_flags = flags;
	if( ( ep->flags & EPFLAG_XFER_NOT_READY ) == 0 ){
		LOG(dc->udc, 5, "%s(%d): ", __func__, __LINE__);
		return( EOK);
	}

	length = urb->buffer_len - urb->actual_len;

	LOG1H(dc->udc, 5, "Control transfer len = ", length);
	LOG1H(dc->udc, 5, "Flags =", flags );

	//FIXME: assume mps is power of 2, which is ok for now.
	if ( ( flags & PIPE_FLAGS_TOKEN_OUT ) && !( flags & PIPE_FLAGS_TOKEN_STATUS ) && ( length & (ep->mps -1) ) ) {
		// massage the length to be module mps
		LOG1H(dc->udc, 6, "changed length ", length );
		length = (length + ep->mps) & ~( ep->mps - 1 );
	}

	ep->req_xfer_len = length;
	ep->bytes_xfered = 0;

	if ( flags & PIPE_FLAGS_TOKEN_STATUS ) {
		// Status Phase
		// controller distinguishes between 2-phase and 3-phase transfers...
		if ( ep->control_phase == CONTROL_PHASE_DATA ) {
			trb_build_ctrl_status3( ep->trb );

		} else {
			trb_build_ctrl_status2( ep->trb );
		}
		ep->control_phase = CONTROL_PHASE_STATUS;

	} else {
		trb_build_ctrl_data( ep->trb, (uint32_t) urb->buffer_paddr + urb->actual_len, length );
		ep->control_phase = CONTROL_PHASE_DATA;
	}

	err = dwcotg_xfer_start( dc, ep, idx );
	if ( err ) {
		LOG1H(dc->udc, 0, "failed to hook trb to complete data/status phase on epidx  ", idx );
	}

	return EOK;
}


int
dwcotg_transfer_abort( usb_dc_t *udc, usb_endpoint_t *usb_ep )
{
	dctrl_t		*dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t	*ep = usb_ep->user;

	if ( ep ) {
		LOG1H(dc->udc, 6, "transfer abort:  ep->num = ", ep->num );
		if(ep->num == 0){
			ep->urb = 0;
			// Get Ready for Next Setup Packet... possibly after re-insertion
			LOGS(dc->udc, 5, "Get Ready for First Setup Packet1");
			ep->control_phase = CONTROL_PHASE_SETUP;
			dwcotg_setup_packet_get( dc );
		}
		else if ( ep->flags & EPFLAG_XFER_ACTIVE ) {
			// stop the controller  from processing transfers
			dwcotg_xfer_abort( dc, ep, ep->idx );
		}

		ep->flags &= ~EPFLAG_XFER_ACTIVE;
	}

	return EOK;
}

int
dwcotg_transfer( usb_dc_t *udc, usb_endpoint_t *usb_ep, usb_transfer_t *urb, _uint32 flags )
{
	dctrl_t		*dc = ( dctrl_t * ) udc->dc_data;
	ep_ctx_t	*ep = usb_ep->user;
	int             err, length;

	if( ep->num == 0){
		return(dwcotg_control_transfer(dc, usb_ep, urb, flags));
	}

	ep->urb = urb;
	ep->xfer_flags = flags;
	length = urb->buffer_len - urb->actual_len;

	LOG1H(dc->udc, 6, "Transfer ep = ", ep->num);
	LOG1H(dc->udc, 6, "Length = ", length);
	LOG1H(dc->udc, 6, "Flags = ", flags );

	//FIXME: assume mps is power of 2, which is ok for now.
	if ( ( flags & PIPE_FLAGS_TOKEN_OUT ) && ( length & (ep->mps -1) ) ) {
		// massage the length to be module mps
		// Todo: do we want to log this?  This is not necessarily an error depending on the protocol.
		length = (length + ep->mps) & ~( ep->mps - 1 );
	}

	ep->req_xfer_len = length;
	ep->bytes_xfered = 0;

	trb_build_normal( ep->trb,  urb->buffer_paddr + urb->actual_len, length);

	err = dwcotg_xfer_start( dc, ep, ep->idx );
	if ( err ) {
		LOG(dc->udc, 0, "%s: dwcotg_xfer_start() failed (err - %d) epidx = %d", __func__, err, ep->idx  );
		goto fail;
	}

	return EOK;

fail:
	return err;
}

static void
default_config_set( usb_dc_t *udc )
{
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;

	dc->flags     = 0;
	dc->ntrb_per_ep = 1;
	udc->capability = DC_CAP_SUPER_SPEED | DC_CAP_FULL_SPEED | DC_CAP_HIGH_SPEED;
}


static int eventq_init(  dctrl_t * dc ) {
	int			err;
	paddr64_t	paddr64;

	/* Init the Event Queue
	 * Depending on the number of interrupts allocated, program the Event Buffer
	 * Address and Size registers to point to the Event Buffer locations in
	 * system memory, the sizes of the buffers, andunmask the interrupt. Note: USB
	 * operation will stop if Event Buffer memory is insufficient, because the
	 * core will stop receiving/transmitting packets
	 */
	dc->eventq_cur_elem = 0;
	usb_calloc(dc->udc, (void **)&dc->eventq_mem, &paddr64, 0, DWCOTG_EVTQ_MEM_SIZE);
	if ( dc->eventq_mem == NULL ) {
		LOGS( dc->udc, 0, "failed when allocating event q");
		err = ENOMEM;
		goto error;
	}

	/* setup the hardware to use newly allocated event memory */
	HW_Write32( dc->IoBase, GEVNTADRLO, paddr64 & 0xffffffff );
	HW_Write32( dc->IoBase, GEVNTADRHI, paddr64 >> 32 );
	HW_Write32( dc->IoBase, GEVNTSIZ, DWCOTG_EVTQ_MEM_SIZE );
	HW_Write32( dc->IoBase, GEVNTCOUNT, 0 );

	return EOK;

error:
	return err;

}

static void eventq_destroy(  dctrl_t * dc ) {
	usb_free(dc->udc, dc->eventq_mem );
}

static int reset_controller( dctrl_t * dc )
{
	int         rc = EOK;
	int         timeout = 1000;	// in ms

	HW_Write32Or( dc->IoBase, DCTL, CSFTRST  );

	while( --timeout && HW_Read32( dc->IoBase, DCTL ) & CSFTRST ) {
		usb_delay(dc->udc, 1 );
	}

	if ( timeout == 0 ) {
		LOGS(dc->udc, 0, "Core Soft Reset Failed " );
		rc = ETIME;
	}

	return rc;
}



/*
Note: The PHYs can be reset only if you are using the pipe3_reset_n and usb2phy_reset signals
from the core.

The GUSB2PHYCFG, GUSB3PIPECTL, and GCTL registers control the USB2.0 PHY reset, USB3.0 PHY
reset, and core's internal reset. Software resets must be generated in the following sequence:
    - Set GUSB2PHYCFG[31], GUSB3PIPECTL[31, and GCTL[11]. This resets the PHYs and keeps the
core in reset state.
    - Reset GUSB2PHYCFG[31] and GUSB3PIPECTL[31] after they meet PHY reset duration. This
removes the reset to the PHYs.
    -Wait for the PHY clock to stabilize and reset GCTL[11] bit. This ensures that the reset to all the
internal blocks are asserted when all the clocks are stable.
*/


static int phy_reset( dctrl_t * dc ) {

	HW_Write32Or( dc->IoBase, GUSB2PHYCFG, PHYSOFTRST  );
	HW_Write32Or( dc->IoBase, GUSB3PIPECTL, PHYSOFTRST  );
	HW_Write32Or( dc->IoBase, GCTL, CORESOFTRESET  );

	usb_delay(dc->udc, 10 );

	if( dc->udc->bs_phy_reset ){
		dc->udc->bs_phy_reset(dc->udc->bs_data, 0);
	}

	HW_Write32And( dc->IoBase, GUSB2PHYCFG, ~PHYSOFTRST  );
	HW_Write32And( dc->IoBase, GUSB3PIPECTL, ~PHYSOFTRST  );

	usb_delay(dc->udc, 10 );

	HW_Write32And( dc->IoBase, GCTL, ~CORESOFTRESET  );

	return EOK;
}

static int
chip_config ( dctrl_t * dc )
{
	int        rc;
	int        addr,sz;
	uint32_t   reg;

	rc = phy_reset( dc );
	if ( rc != EOK ) {
		goto error;
	}

	// force controller in device mode
	reg = HW_Read32( dc->IoBase, GCTL );
	reg &= ~PRTCAPDIR_MSK;
	reg |= PRTCAPDIR_DEVICE;
	HW_Write32( dc->IoBase, GCTL, reg );

	// Make sure the controller is stopped before doing reset
	HW_Write32And( dc->IoBase, DCTL, ~RUNSTOP );

	/* All of these initializations are from the SuperSpeed USB 3.0
	 * DesignWare Controller Databook, Chapter9 Table 9-1
	 */

	/* Step 1:
	 * DCTL: Set the CSftRst field to ‘1’ and wait for a read to return ‘0’.
	 * This will reset the device core
	 */
	rc = reset_controller( dc );
	if ( rc != EOK ) {
		goto error;
	}

	// set ep0 tx fifo size
	addr = fifomem_alloc_ep0( dc->fifomem_hdl, &sz );
	HW_Write32( dc->IoBase, GTXFIFOSIZ( 0 ), ( addr << TXFSTADDR_POS ) | sz  );

	/* Step 2: Nothing to do yet
	 * GSBUSCFG0/1: Leave the default values if the correct power-on
	 * values were selected during coreConsultant configuration.
	 */

	/* Step 3: Nothing to do yet
	 * GTXtHRCFG/ GRXTHRCFG:  This is required only if you are planning to enable thresholding. Leave
	 * the default values if the correct power-on values were selected during
	 * coreConsultant configuration.
	 */


	/* Step 4: Nothing to do yet
	 * GSNPSID: The software must read the Synopsys ID register to find the core
	 * version and configure the driver for any version-specific features.
	 */

	/* Step 5: Nothing to do yet
	 * GUID: Optionally, the software can program the User ID GUID register, if this
	 * register is selected for implementation in coreConsultant
	 */

	/* Step6: Nothing to do yet
	 * GUSB2PHYCFG: Program the following PHY configuration fields:
	 * USBTrdTim, FSIntf, PHYIf, TOUTCal, or leave the default values if the
	 * correct power-on values were selected during coreConsultant configuration
	 */

	/* Step 7: Nothing to do yet
	 * GUSB3PIPECTL : Program the following PHY configuration fields:
	 * DatWidth, PrtOpDir, or leave the default values if the correct power-on
	 * values were selected during coreConsultant configuration.
	 */

	/* Step 8:
	 * GTXFIFOSIZn : Write these registers to allocate prefetch buffers for each
	 * Tx endpoint. Unless the packet sizes of the endpoints are
	 * application-specific, it is recommended to use the default value. For details,
	 * see “Device TxFIFO Data Allocation” on page 331.
	 */
	/* Step 9:
	 * GRXFIFOSIZ0 : Write this register to allocate the receive buffer for all
	 * endpoints. Unless the packet sizes of the endpoints are application-specific,
	 * it is recommended to use the default value. For details, see
	 * “Device RxFIFO Data Allocation” on page 331.
	 */

	/* Step 10: */
	rc = eventq_init( dc );
	if ( rc != EOK ) {
		goto error;
	}

	/* Step 11:
	 * GCTL: Program this register to override scaledown, loopback, RAM clock select,
	 * and clock gating parameters
	 */

	/* Step 12:
	 * DCFG: Program device speed and periodic frame interval
	 */

	/* Step 13:
	 * DEVTEN: At a minimum, enable USB Reset, Connection Done, and USB/Link
	 * State Change events
	 */

	HW_Write32( dc->IoBase, DEVTEN,
		DISCONNEVTEN | USBRSTEN | CONNECTDONEEN | ULSTCNGEN | CMDCMPLTEN );

	/* Defer The the control endpoint initialization until the stack
	 * enables EP0
	 */

	 rc = ep0_cfg(dc);
	 if ( rc != EOK ) {
		LOGS(dc->udc, 0, "ep0_cfg() failed" );
		goto error2;
	 }

	return EOK;

error2:
	eventq_destroy( dc );
error:
	return rc;
}

int
dwcotg_init( void *hdl )
{
	dctrl_t *dc;
	usb_dc_t *udc = (usb_dc_t *)hdl;
	int rc = EOK;
	int phase = 0;

	udc->fini = dwcotg_shutdown;

	udc->address_device = dwcotg_set_device_address;
	udc->config_device = dwcotg_select_configuration;
	udc->set_bus_state = dwcotg_set_bus_state;
	udc->endpoint_enable = dwcotg_endpoint_enable;
	udc->endpoint_disable = dwcotg_endpoint_disable;
	udc->set_endpoint_state = dwcotg_set_endpoint_state;
	udc->clear_endpoint_state = dwcotg_clear_endpoint_state;
	udc->endpoint_transfer = dwcotg_transfer;
	udc->endpoint_abort = dwcotg_transfer_abort;
	udc->interrupt = dwcotg_interrupt;

	/* allocate device ctx */
	if( (usb_calloc(udc, (void **)&dc, NULL, 0,  sizeof ( dctrl_t )))){
		rc = ENOMEM;
		goto error;
	}

	udc->dc_data = dc;
	dc->udc = udc;
	dc->IoBase = udc->base;

	 /* set default driver configurations */
	default_config_set( udc );

	if ( udc->bs_init ){
		rc = udc->bs_init( &udc->bs_data, phase++ );
		if ( rc ) {
			LOG1H( udc, 0, "dwcotg_custom_init1() failed err = ", rc);
			goto error1;
		}
	}

	// cache how many endpoints are supported by the controller
	dc->n_ep =  ( HW_Read32( dc->IoBase, GHWPARAMS3 ) &  DWC_USB3_NUM_EPS_MSK ) >> DWC_USB3_NUM_EPS_POS;

	if ( dc->n_ep < udc->num_endpoints ){
		LOGS(udc, 0, "controller does not have enough endpoints" );
		rc = ENOTSUP;
		goto error1;
	}
	dc->n_ep = udc->num_endpoints;

	dc->fifomem_hdl = fifomem_init( dc );
	if ( dc->fifomem_hdl == NULL ) {
		LOGS( udc, 0, "failed to create fifomem allocator" );
		goto error1;
	}

	// setup usb controller
	rc = chip_config( dc );
	if ( rc ) {
		LOGS(udc, 0, "dwc otg chip config failed");
		goto error2;
	}

	if ( udc->bs_init ){
		rc = udc->bs_init( &udc->bs_data, phase++ );
		if ( rc ) {
			LOG1H( udc, 0, "dwcotg_custom_init1() failed err = %d", rc);
			goto error2;
		}
	}

	LOGS( udc, 5, "dwcotg init OK");

	udc->speed = USB_DEVICE_FULL_SPEED;

	return EOK;
error2:
	fifomem_fini(dc);
error1:
	usb_free(udc, dc);
error:
	return rc;
}

int
dwcotg_shutdown( usb_dc_t *udc )
{
	dctrl_t * dc = ( dctrl_t * ) udc->dc_data;
	int phase = 0;

	// force disconnect
	HW_Write32And( dc->IoBase, DCTL, ~RUNSTOP );

	// free resources
	ep0_decfg( dc );

	fifomem_fini( dc->fifomem_hdl );

	if ( udc->bs_fini ){
		udc->bs_fini( udc->bs_data, phase++ );
		udc->bs_fini( udc->bs_data, phase++ );
	}
	eventq_destroy( dc );

	usb_free(udc, dc);

	return EOK;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/lib/usbboot/udc/dwc3.c $ $Rev: 808052 $")
#endif
