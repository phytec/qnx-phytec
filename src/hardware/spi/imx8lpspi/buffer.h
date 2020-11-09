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

#ifndef BUFFER_H_
#define BUFFER_H_

#include <pthread.h>
#include <stdint.h>

/**
 * @file       imx8lpspi/buffer.h
 * @addtogroup spi
 * @{
 */

/** Circular buffer size */
#define MAX_BUFF_BYTES  2048

/* SPI slave buffer */
typedef struct _buffer {
    char            *head;              /**< Buffer head index */
    char            *tail;              /**< Buffer tail index */
    char            *buff;              /**< Pointer to the buffer itself */
    unsigned int    cnt;                /**< Number of items in buffer */
    int             bufsize;            /**< Overall buffer size */
} buffer_t;

/* Buffer API */
buffer_t *buff_create (uint32_t size);
int buff_flush(buffer_t * bptr);
int buff_putc(buffer_t * bptr, char c);
int buff_getc(buffer_t * bptr, char *c);
int buff_waiting(buffer_t * bptr);
int buff_remaining(buffer_t * bptr);
int buff_put(buffer_t * bptr, char *buff, unsigned int size);
int buff_get(buffer_t * bptr, char *buff, unsigned int size);

/** @} */

#endif /* BUFFER_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/spi/imx8lpspi/buffer.h $ $Rev: 863314 $")
#endif
