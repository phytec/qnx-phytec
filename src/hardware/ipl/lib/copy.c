/*
 * $QNXLicenseC:
 * Copyright 2007, 2008, 2017 QNX Software Systems.
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



/*
 * This is used to define our ipl memory copy mechanism
*/

#include "ipl.h"
#include <string.h>

/*
 * Fast memcpy for large bulk transfers, presently only implemented for AArch64
 */
extern void memcpy_bulk(void *dst, const void *src, size_t size);

#define ALIGNED32(__x)     (((__x) & 0x3) == 0)
#define ALIGNED64(__x)     (((__x) & 0x7) == 0)

int copy_memory(unsigned long dest, unsigned long src, unsigned long sz) {

/*    Easiest way is to copy over 1 byte at a time ...
    while (sz > 0) {
        *(char*)dest = *(char*)src;
        dest++;
        src++;
        sz--;
    }
*/
    short      remainder;

#if defined(__aarch64__)
    // for larger payloads that are aligned use the faster memcpy
    if (ALIGNED64(dest) && ALIGNED64(src) && (sz >= 64)) {
        memcpy_bulk((void *)dest, (const void *)src, sz);
        return(0);
    }

    // src and dest are 64-bits aligned?
    if (ALIGNED64(dest) && ALIGNED64(src)) {
        remainder = sz & 0x7;
        sz = sz >> 3;
        while (sz > 0) {
            *(_Uint64t *)dest = *(_Uint64t *)src;
            dest += 8;
            src += 8;
            sz--;
        }
    }
    else
#endif
    // src and dest are 32-bits aligned?
    if (ALIGNED32(dest) && ALIGNED32(src)) {
        remainder = sz & 0x3;
        sz = sz >> 2;
        while (sz > 0) {
            *(unsigned*)dest = *(unsigned*)src;
            dest += 4;
            src += 4;
            sz--;
        }
    } else {
        remainder = sz;
    }

    while (remainder > 0) {
        *(char*)dest = *(char*)src;
        dest++;
        src++;
        remainder--;
    }

    return(0);
}

/*
 All copying goes through here.  If paging/windowing is
 required then it will go through the utility functions
 above, otherwise copy_memory will just be called
*/
void copy (unsigned long dst, unsigned long src, unsigned long size) {
    copy_memory (dst, src, size);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/lib/copy.c $ $Rev: 847108 $")
#endif
