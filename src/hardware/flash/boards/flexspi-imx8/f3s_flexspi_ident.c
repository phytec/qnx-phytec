/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017, 2019 NXP
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

#include <sys/mman.h>
#include <sys/slogcodes.h>

#include "imx_fc_flexspi.h"
#include "f3s_flexspi.h"
#include "flexspi_cmds.h"

/**
 * @file       flexspi-imx8/f3s_flexspi_ident.c
 * @addtogroup ffs3
 * @{
 */

static const f3s_dbase_t supported_devices[] = {
    {
        sizeof(f3s_dbase_t),   /* Size of complete structure with geometries */
        0,                     /* Status of structure */
        0x2C,                  /* Jedec high byte - manufacturer ID */
        0x5B,                  /* Jedec low byte - device ID */
        "Micron MT35 OCTAL NOR Flash",
        0,                     /* Flags for capabilities */
        0,                     /* Interleave for chips on bus */
        0,                     /* Width of chip */
        120000U,               /* Typical write time for cell (ns) */
        200000000U,            /* Typical erase time for unit (ns) */
        0,                     /* Read mode voltage needed */
        0,                     /* Program mode voltage needed */
        0,                     /* Number of erase cycles */
        0,                     /* Poll count timeout */
        0,                     /* Depth of erase queue per chip */
        0,                     /* Number of write buffers per chip */
        0,                     /* Size of write buffers */
        1,                     /* Number of geometries in vector */
        {{512, 17}},           /* Number of erase units for geometry; power 2 size of a unit (128kB sector size) */
        0,                     /* Chip size */
        0                      /* Proc tech */
    },
    {
        sizeof(f3s_dbase_t),   /* Size of complete structure with geometries */
        0,                     /* Status of structure */
        0x20,                  /* Jedec high byte - manufacturer ID */
        0xBB,                  /* Jedec low byte - device ID - 1.8V = 0xBB */
        "Micron MT25QU256 QSPI NOR Flash",
        0,                     /* Flags for capabilities */
        0,                     /* Interleave for chips on bus */
        0,                     /* Width of chip */
        120000U,               /* Typical write time for cell (ns) */
        200000000U,            /* Typical erase time for unit (ns) */
        0,                     /* Read mode voltage needed */
        0,                     /* Program mode voltage needed */
        0,                     /* Number of erase cycles */
        0,                     /* Poll count timeout */
        0,                     /* Depth of erase queue per chip */
        0,                     /* Number of write buffers per chip */
        0,                     /* Size of write buffers */
        1,                     /* Number of geometries in vector */
        {{512, 16}},           /* Number of erase units for geometry; power 2 size of a unit (64kB sector size) */
        0,                     /* Chip size */
        0                      /* Proc tech */
    },
    {
        sizeof(f3s_dbase_t),   /* Size of complete structure with geometries */
        0,                     /* Status of structure */
        0x20,                  /* Jedec high byte - manufacturer ID */
        0xBB,                  /* Jedec low byte - device ID - 1.8V = 0xBB */
        "Micron MT25QU512 QSPI NOR Flash",
        0,                     /* Flags for capabilities */
        0,                     /* Interleave for chips on bus */
        0,                     /* Width of chip */
        120000U,               /* Typical write time for cell (ns) */
        200000000U,            /* Typical erase time for unit (ns) */
        0,                     /* Read mode voltage needed */
        0,                     /* Program mode voltage needed */
        0,                     /* Number of erase cycles */
        0,                     /* Poll count timeout */
        0,                     /* Depth of erase queue per chip */
        0,                     /* Number of write buffers per chip */
        0,                     /* Size of write buffers */
        1,                     /* Number of geometries in vector */
        {{1024, 16}},          /* Number of erase units for geometry; power 2 size of a unit (64kB sector size) */
        0,                     /* Chip size */
        0                      /* Proc tech */
    },
    {0, 0xFFFF, 0, 0, NULL, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, {{0, 0}}, 0, 0}
};

/**
 * This is the identification callout for FLEXSPI serial NOR flash driver.
 *
 * @param dbase  Flash Services Database.
 * @param access Access Super Structure.
 * @param flags  Flags.
 * @param offset Memory offset.
 *
 * @retval EOK    Device detection was successful.
 * @retval ENOENT Device detection fail.
 */
int f3s_flexspi_ident(f3s_dbase_t *dbase, f3s_access_t *access, uint32_t flags,
                        __attribute__((unused)) uint32_t offset)
{
    static const f3s_dbase_t *probe = NULL;
    int                      mid;  /* Manufacturer id */
    int                      did;  /* Device id */
    uint32_t                 size; /* Chip total size in bytes */
    uint8_t                  i;
    int                      k;
    unsigned                 unit_size;
    imx_fspi_t               *dev = (imx_fspi_t *) access->socket.socket_handle;

    /* Check listing flag */
    if (flags & F3S_LIST_ALL) {
        /* Check if first pass */
        if (!probe) {
            probe = &supported_devices[0];
        }
        if (!probe->struct_size) {
            return (ENOENT);
        }
        *dbase = *probe++;  /* Update database entry */
        return (EOK);
    }

    /* Flash device spec calls for a 30 us delay after
     * release from power down. For simplicity, we just sleep 1 ms.
     */
    delay(1);

    if (pre_ident_cfg(dev)) {
        return ENOENT;
    }
    read_ident(dev, &mid, &did, &size);

    if (size == 0) { /* The device ident does return the size */
        size = dev->size * dev->die;
    }

    for (i = 0; i < ARRAY_SIZE(supported_devices); i++) {
        const f3s_dbase_t *sd = &supported_devices[i];

        if ((mid == sd->jedec_hi) && (did == sd->jedec_lo)) {
            for (k = 0; k < sd->geo_num; k++) {
                const struct f3s_geo_s *geo_vect = &sd->geo_vect[k];
                unit_size = 1 << geo_vect->unit_pow2;
                /* since size is a command line option it must be checked here*/
                if( (size == geo_vect->unit_num * unit_size)
                    ||
                    (size == dev->size ) ) {
                    *dbase = *sd;  /* Update database entry */
                    /* ensure the number of erase units is set right*/
                    dbase->geo_vect[0].unit_num = size / unit_size;
                    dbase->chip_size = size;
                    access->socket.unit_size = unit_size;
                    if (post_ident_cfg(dev)) {
                        return ENOENT;
                    }
                    slogf(_SLOGC_FS_FFS, _SLOG_INFO, "(devf  t%d::%s:%d) serial NOR detected: mid=0x%X did=0x%X size=%d",
                          pthread_self(), __func__, __LINE__, mid, did, size);
                    return (EOK);
                }
            }
        }
    }

    return (ENOENT);
}

/** @} */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/flash/boards/flexspi-imx8/f3s_flexspi_ident.c $ $Rev: 893539 $")
#endif
