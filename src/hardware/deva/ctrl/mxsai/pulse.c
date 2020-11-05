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

#include "mxsai.h"

/**
 * i.MX SAI audio driver source file.
 *
 */

#define PULSE_CODE_INTERRUPT    (_PULSE_CODE_MINAVAIL+1)
#define PULSE_CODE_EXIT         (_PULSE_CODE_MINAVAIL+2)

/** Pulse structure. */
struct my_pulse_struct {
    pthread_t tid;
    int rtn;
    int chid;
    int coid;
    struct sigevent *event;
    void (*handler)(HW_CONTEXT_T * hw_context, struct sigevent * event);
    HW_CONTEXT_T *hw_context;
};

/**
 * Pulse thread for pulse message receive.
 *
 * @param args Pointer to pulse struct.
 *
 */
static void *my_pulse_thread(void *args)
{
    struct my_pulse_struct *mps = args;
    struct _pulse pulse;

    if ((mps->chid = ChannelCreate(_NTO_CHF_DISCONNECT | _NTO_CHF_UNBLOCK)) == -1
        || (mps->coid = ConnectAttach(0, 0, mps->chid, _NTO_SIDE_CHANNEL, 0)) == -1) {
        mps->rtn = -1;
        return (NULL);
    }
    SIGEV_PULSE_INIT(mps->event, mps->coid, global_options.intr_thread_prio, PULSE_CODE_INTERRUPT, 0);
    mps->rtn = 1; /* We're ok signal parent thread */

    for (;;) {
        if (MsgReceivePulse(mps->chid, &pulse, sizeof(pulse), NULL) != -1) {
            switch (pulse.code) {
                case PULSE_CODE_INTERRUPT:
                    mps->handler(mps->hw_context, mps->event);
                    break;
                case PULSE_CODE_EXIT:
                    ado_error_fmt("MX_SAI: Got Exit pulse...");
                    pthread_exit(NULL);
                default:
                    break;
            }
        } else {
            ado_error_fmt("MX_SAI: MsgReceivePulse() error %s", strerror(errno));
        }
    }

    return (NULL);
}

/**
 * Attach a new pulse handler.
 *
 * @param x          Handle to pulse structure.
 * @param event      Pointer to event structure.
 * @param handler    Pointer to handler to be attached.
 * @param hw_context Audio hardware context structure.
 *
 * @return Execution status.
 */
int my_attach_pulse(void **x, struct sigevent *event,
                    void (*handler)(HW_CONTEXT_T * hw_context, struct sigevent * event), HW_CONTEXT_T * hw_context)
{
    struct my_pulse_struct *mps;
    int policy, err;
    pthread_attr_t attr;
    struct sched_param param;

    if ((mps = (struct my_pulse_struct *) calloc(1, sizeof(struct my_pulse_struct))) == NULL) {
        return (-1);
    }

    mps->event = event;
    mps->handler = handler;
    mps->hw_context = hw_context;

    pthread_attr_init(&attr);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = global_options.intr_thread_prio;
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setschedpolicy(&attr, SCHED_RR);

    if ((err = pthread_create(&mps->tid, &attr, my_pulse_thread, mps)) != EOK) {
        ado_error_fmt("MX_SAI: Unable to attach to pulse (%s)", strerror(err));
        return -err;
    }

    /* Wait for the intr thread to fail or succeed */
    while (mps->rtn == 0) {
        delay(1);
    }
    if (mps->rtn == -1) {
        errno = EINVAL;
        return (-errno);
    }

    *x = mps;
    return (0);
}

/**
 * Detach a pulse handler.
 *
 * @param x Handle to pulse structure.
 *
 * @return Execution status.
 */
int my_detach_pulse(void **x)
{
    struct my_pulse_struct *mps = *x;

    ado_error_fmt("MX_SAI: Deliver Exit pulse...");
    MsgSendPulse(mps->coid, -1, PULSE_CODE_EXIT, 0);
    pthread_join(mps->tid, NULL);
    ConnectDetach(mps->coid);
    ChannelDestroy(mps->chid);
    return (0);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/deva/ctrl/mxsai/pulse.c $ $Rev: 859533 $")
#endif

