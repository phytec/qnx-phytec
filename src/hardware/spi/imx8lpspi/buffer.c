/*
 * $QNXLicenseC:
 * Copyright 2010, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
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

/**
 * @file       imx8lpspi/buffer.c
 * @addtogroup spi
 * @{
 */

#include <atomic.h>
#include <stdlib.h>
#include <string.h>

#include "buffer.h"

/**
 * Buffer create.
 *
 * @param size Size of the buffer.
 *
 * @return pointer to buffer or NULL if fail.
 */
buffer_t * buff_create (uint32_t size)
{
    buffer_t *cbptr;

    if ((cbptr = _smalloc(sizeof(*cbptr)))) {
        cbptr->bufsize = size ? size : MAX_BUFF_BYTES;
        cbptr->cnt = 0;
        if (!(cbptr->head = cbptr->tail = cbptr->buff = malloc (cbptr->bufsize))) {
            free (cbptr);
            cbptr = NULL;
        }
    }

    return (cbptr);
}

/**
 * Buffer flush.
 *
 * @param bptr Buffer handle.
 *
 * @return EOK always.
 */
int buff_flush(buffer_t * bptr)
{
    bptr->tail = bptr->head = bptr->buff;
    bptr->cnt = 0;

    return (EOK);
}


/**
 * Buffer put char.
 *
 * @param bptr Buffer handle.
 * @param c    Char to put.
 *
 * @return Number items in buffer or -1 if buffer full.
 */
int buff_putc(buffer_t * bptr, char c)
{
    if (bptr->cnt < bptr->bufsize) {
        *bptr->head++ = c;
        atomic_add(&bptr->cnt, 1);
        if (bptr->head >= &bptr->buff[bptr->bufsize]) {
            bptr->head = bptr->buff;
        }
    } else {
        return -1;
    }

    return (bptr->cnt);
}

/**
 * Buffer get char.
 *
 * @param bptr Buffer handle.
 * @param c    Char to get.
 *
 * @return Return 0 if everything is fine, -1 if no data in buffer.
 */
int buff_getc(buffer_t * bptr, char *c)
{

    if (bptr->cnt == 0) {
        return (-1);
    }

    *c = *bptr->tail;
    atomic_sub(&bptr->cnt, 1);

    if (++bptr->tail >= &bptr->buff[bptr->bufsize]) {
        bptr->tail = bptr->buff;
    }

    return 0;
}

/**
 * Return number of items in buffer.
 *
 * @param bptr Buffer handle.
 *
 * @return Number of items in buffer.
 */
int buff_waiting(buffer_t * bptr)
{
    return (bptr->cnt);
}

/**
 * Return free space in buffer.
 *
 * @param bptr Buffer handle.
 *
 * @return free space in buffer.
 */
int buff_remaining(buffer_t * bptr)
{
    return (bptr->bufsize - bptr->cnt);
}

/**
 * Put chars into buffer.
 *
 * @param bptr Buffer handle.
 * @param buff Pointer to a user buffer.
 * @param size Size of data to put into buffer.
 *
 * @return Number items in buffer or -1 if buffer full.
 */
int buff_put(buffer_t * bptr, char *buff, unsigned int size)
{
    unsigned int i;

    for(i = 0; i < size; i++) {
        if (buff_putc(bptr, buff[i]) < 0) {
            return -1;
        }
    }

    return buff_waiting(bptr);
}

/**
 * Get chars from buffer.
 *
 * @param bptr Buffer handle.
 * @param buff Pointer to a user buffer.
 * @param size Size of data to get from buffer.
 *
 * @return Return 0 if everything is fine, -1 if no data in buffer.
 */
int buff_get(buffer_t * bptr, char *buff, unsigned int size)
{
    unsigned int i;
    char c;

    for(i = 0; i < size; i++) {
        if (buff_getc(bptr, &c) == 0) {
            buff[i] = c;
        } else {
            return -1;
        }
    }

    return 0;
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/spi/imx8lpspi/buffer.c $ $Rev: 893541 $")
#endif
