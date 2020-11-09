/*
 * $QNXLicenseC: 
 * Copyright 2007, QNX Software Systems.  
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


/* pulse.c */
int my_attach_pulse ( void **x , struct sigevent *event , void (*handler )(HW_CONTEXT_T *hw_context ,struct sigevent *event ), HW_CONTEXT_T *hw_context );
int my_detach_pulse ( void **x );

/* mxesai_dll.c */
uint32_t num_active_playback_interfaces(HW_CONTEXT_T * mx);
uint32_t num_active_capture_interfaces(HW_CONTEXT_T * mx);

#ifdef VARIANT_mx8
/* ipc.h */
int errata_check (void);
/* errata.c */
int gpt_init ( HW_CONTEXT_T *mx, dma_driver_info_t *dma_info );
void gpt_destroy ( HW_CONTEXT_T *mx );
void gpt_stop ( HW_CONTEXT_T *mx, int channel );
void gpt_start ( HW_CONTEXT_T *mx, int channel );
void gpt_release ( HW_CONTEXT_T *mx, int channel );
int gpt_acquire( HW_CONTEXT_T *mx, int channel );
#endif


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/deva/ctrl/mxesai/proto.h $ $Rev: 886095 $")
#endif
