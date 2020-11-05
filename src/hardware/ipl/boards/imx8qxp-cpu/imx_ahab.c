/*
 * $QNXLicenseC:
 * Copyright 2018-2019 NXP
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

#include "ipl.h"
#include "board.h"
#include "imx_ipl.h"
#include <hw/nxp/imx8/sci/sci.h>
#include "imx_ipl.h"

#define IV_MAX_LEN              32
#define HASH_MAX_LEN            64
#define IMX_SECO_PT             2U

struct container_hdr {
    uint8_t version;
    uint8_t length_lo;
    uint8_t length_hi;
    uint8_t tag;
    uint32_t flags;
    uint16_t sw_version;
    uint8_t fuse_version;
    uint8_t num_images;
    uint16_t sig_blk_offset;
    uint16_t reserved;
} __attribute__((packed));

struct boot_img_t{
    uint32_t offset;
    uint32_t size;
    uint64_t dst;
    uint64_t entry;
    uint32_t hab_flags;
    uint32_t meta;
    uint8_t hash[HASH_MAX_LEN];
    uint8_t iv[IV_MAX_LEN];
}__attribute__((packed));

#if (IMX_CACHE_EN == 1)
    void aarch64_cache_flush(void);
#endif

#if (IMX_HAB_AUTHENTICATE_CONFIG == 1)
/**
 * Authenticate QNX-IFS image stored in DDR memory.
 *
 * @param img_addr QNX image DDR memory address.
 *
 * @return         Authentication of QNX-IFS image status.
 * @retval          0   QNX-IFS image was correctly authenticated.
 * @retval         -1   QNX-IFS image was not correctly authenticated.
 */
int imx_ahab_authenticate_image(paddr_t img_addr)
{
    struct container_hdr *phdr = (struct container_hdr *)img_addr;
    sc_err_t sc_status;
    sc_ipc_t ipc;
    sc_rm_mr_t mr;
    uint64_t img_dest;
    uint32_t img_size;
    int ret = 0;

    if ((phdr->tag != 0x87) && (phdr->version != 0x00)) {
        ser_putstr("Wrong container header\n");
        return -1;
    }

    if (phdr->num_images == 0x00) {
        ser_putstr("Wrong container, no image found\n");
        return -1;
    }
    copy((paddr_t)IMX_SECO_SECURE_RAM_BASE, img_addr, ((unsigned long)phdr->length_hi << 8) | phdr->length_lo);

    sc_status = sc_ipc_open(&ipc, IMX_SCU_IPC_MU);
    if (sc_status == SC_ERR_NONE) {
        sc_status = sc_seco_authenticate(ipc, SC_SECO_AUTH_CONTAINER, IMX_SECO_LOCAL_SECURE_RAM_BASE);
        if (sc_status == SC_ERR_NONE) {
            /* Copy image to the destination address */
            struct boot_img_t *img = (struct boot_img_t *)(img_addr + sizeof(struct container_hdr));
            img_dest = img->dst;
            img_size = img->size;
            copy((paddr_t)img->dst, (paddr_t)(img->offset + img_addr), img->size);

#if (IMX_CACHE_EN == 1)
            /* Flush cache */
            aarch64_cache_flush();
#endif
            /* Find the memreg and set permission for seco pt */
            sc_status = sc_rm_find_memreg(ipc, &mr, img_dest, IMX_ALIGN(img_dest + img_size, 64UL));
            if (sc_status == SC_ERR_NONE) {
                sc_status = sc_rm_set_memreg_permissions(ipc, mr, IMX_SECO_PT, SC_RM_PERM_FULL);
                if (sc_status == SC_ERR_NONE) {
                    sc_status = sc_seco_authenticate(ipc, SC_SECO_VERIFY_IMAGE, 0x01);
                    if (sc_status != SC_ERR_NONE) {
                        ser_putstr("Authenticate QNX-IFS image failed!\n");
                        ret = -1;
                    }
                    sc_status = sc_rm_set_memreg_permissions(ipc, mr, IMX_SECO_PT, SC_RM_PERM_NONE);
                    if (sc_status != SC_ERR_NONE) {
                        ser_putstr("Remove permission failed for QNX-IFS image!\n");
                        ret = -1;
                    }
                } else {
                    ser_putstr("Set permission failed for QNX-IFS image!\n");
                    ret = -1;
                }
            } else {
                ser_putstr("Can't find memreg for QNX-IFS image load address!\n");
                ret = -1;
            }
        } else {
            ser_putstr("Authenticate container header failed!\n");
            ret = -1;
        }

        (void)sc_seco_authenticate(ipc, SC_SECO_REL_CONTAINER, 0);

        /* Close IPC channel */
        sc_ipc_close(ipc);
    } else {
        ret = -1;
    }

    return ret;
}
#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/ipl/boards/imx8qxp-cpu/imx_ahab.c $ $Rev: 904597 $")
#endif
