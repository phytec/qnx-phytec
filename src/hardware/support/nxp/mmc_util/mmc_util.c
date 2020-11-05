/*
 * $QNXLicenseC:
 * Copyright 2018, QNX Software Systems.
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

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/resmgr.h>
#include <sys/neutrino.h>
#include <sys/procmgr.h>
#include <hw/inout.h>
#include <fcntl.h>
#include <string.h>
#include <glob.h>
#include <sys/dcmd_cam.h>
#include <hw/dcmd_sim_sdmmc.h>
#include <inttypes.h>

#include "mmc_util.h"

#define MMC_EXT_CSD_SIZE            512
#define MMC_GEN_CMD_READ_SIZE       512


static int get_device_info(int fd, SDMMC_DEVICE_INFO *info)
{
    int     result;

    result = devctl(fd, DCMD_SDMMC_DEVICE_INFO, info, sizeof(SDMMC_DEVICE_INFO), NULL);
    if (result != EOK) {
        fprintf(stderr, "DCMD_SDMMC_DEVICE_INFO failed %d (%s)\n", result, strerror(errno));
        return result;
    }

    return (EOK);
}

static int get_ext_csd(int fd, uint8_t *ecsd)
{
    SDMMC_CARD_REGISTER *cmd;
    uint8_t buf[sizeof(SDMMC_CARD_REGISTER) + MMC_EXT_CSD_SIZE];
    int     ret = EOK;

    cmd = (SDMMC_CARD_REGISTER *)buf;

    /* read EXT CSD information */
    cmd->action = SDMMC_CR_ACTION_READ;
    cmd->type   = SDMMC_REG_TYPE_EXT_CSD;
    ret = devctl(fd, DCMD_SDMMC_CARD_REGISTER, buf, sizeof(buf), NULL);
    if (ret != EOK) {
        fprintf(stderr, "DCMD_SDMMC_CARD_REGISTER : SDMMC_REG_TYPE_EXT_CSD failed (%s)\n", strerror(ret));
        return ret;
    }

    memcpy(ecsd, &buf[sizeof(SDMMC_CARD_REGISTER)], MMC_EXT_CSD_SIZE);

    return (ret);
}

static void dump_buffer(uint8_t *buf, int size)
{
    int i;

    for(i=0; i<size; i++){
        if((i % 16) == 0){
            printf("\n %03X (%04d):", i, i);
        }
        printf(" %02X", buf[i]);
    }
    printf("\n");
}

static int display_device_info(int fd)
{
    SDMMC_DEVICE_INFO   info;

    if (get_device_info(fd, &info) != EOK) return (EIO);

    fprintf(stderr, "\n----- device info -----\n");
    fprintf(stderr, "device type  = %s\n", info.dtype == 1 ? "MMC" : (info.dtype == 2 ? "SD" : "?"));
    fprintf(stderr, "mid          = %x\n", info.mid);
    fprintf(stderr, "pnm          = %s\n", info.pnm);
    fprintf(stderr, "prv          = %d.%d\n", info.prv >> 4, info.prv & 0x0F);
    fprintf(stderr, "psn          = %08x\n", info.psn);
    fprintf(stderr, "mdt          = %d.%d\n", info.year, info.month);
    fprintf(stderr, "spec_vers    = %d\n", info.spec_vers);
    fprintf(stderr, "spec_rev     = %d\n", info.spec_rev);
    fprintf(stderr, "device caps  = %lx\n", info.caps);
//    fprintf(stderr, "timing       = %s(%d)\n", sdmmc_timing(info.timing), info.timing);
    fprintf(stderr, "DTR          = %d\n", info.dtr);
    fprintf(stderr, "bus_width    = %d\n", info.bus_width);
    fprintf(stderr, "sectors      = %d\n", info.sectors);
    fprintf(stderr, "start_sector = %d\n", info.start_sector);
    fprintf(stderr, "sector_size  = %d\n", info.sector_size);
    fprintf(stderr, "erase_size   = %d\n", info.erase_size);
    fprintf(stderr, "-----------------------\n");

    return (EOK);
}

static int samsung_health_report(int fd)
{
    uint8_t     *ecsd = alloca(MMC_EXT_CSD_SIZE);

    int ret;
    if ((ret = get_ext_csd(fd, ecsd)) != EOK) {
        fprintf(stderr, "Get ECSD failed\n");
        return (-1);
    }

    // byte 268, 269
    // device life time estimation
    if (ecsd[269] != ecsd[268]) {
        fprintf(stderr, "EXT_CSD byte 268[%x] and 268[%x] should be the same!\n", ecsd[269], ecsd[268]);
        return (EIO);
    }
    fprintf(stderr, "Device Life Time Estimation:\n");
    if (ecsd[268] == 0x0B) {
        fprintf(stderr, "  Maximum estimated device life time exceeded!\n");
    } else if (ecsd[268] > 1 && ecsd[268] < 0x0B) {
        fprintf(stderr, "  %d - %d percent device life time used.\n", (ecsd[268] - 2) * 10, (ecsd[268] - 1) * 10);
    } else {
        fprintf(stderr, "  Invalid life time byte value [%x] returned, valid value: 0x02~0x0A\n", ecsd[268]);
    }

    // byte 267
    // Pre-EOL information
    fprintf(stderr, "Pre-EOL Information:\n");
    switch (ecsd[267]) {
        case 1:
            fprintf(stderr, "  Normal\n");
            break;
        case 2:
            fprintf(stderr, "  80 percent of reserved blocks consumed\n");
            break;
        case 3:
            fprintf(stderr, "  90 percent of reserved blocks consumed\n");
            break;
        default:
            fprintf(stderr, "  undefined Pre-EOL value %x, valid value: 0x01~0x03\n", ecsd[267]);
            break;
    }

    // byte 86
    // bad block ratio information
    fprintf(stderr, "Bad Block Ratio Information:\n");
    if (ecsd[86] > 0 && ecsd[86] <= 10) {
        fprintf(stderr, "  %d - %d bad block ratio.\n", (ecsd[86] - 1) * 10, ecsd[86] * 10);
    } else {
        fprintf(stderr, "  Invalid bad block ration byte value [%d] returned, valid value: 0x01~0x0A\n", ecsd[86]);
    }

    uint32_t    avg;
    // erase count information
    // byte [85:84]
    // Maximum Erase Count in MLC blocks
    fprintf(stderr, "Erase Count Information:\n");
    fprintf(stderr, "  Max MLC erase cycles %d\n", (ecsd[85] << 8) | ecsd[84]);
    // byte [83:80]
    // Average Erase Count in MLC blocks
    avg = ((uint32_t)ecsd[83] << 24) | ((uint32_t)ecsd[82] << 16) | ((uint32_t)ecsd[81] << 8) | ecsd[80];
    fprintf(stderr, "  AVG MLC erase cycles %d.%d\n", avg / 100, avg % 100);
    // byte [79:78]
    // Minimum Erase Count in MLC blocks
    fprintf(stderr, "  Min MLC erase cycles %d\n", (ecsd[79] << 8) | ecsd[78]);
    // byte [77:76]
    // Maximum Erase Count in SLC blocks
    fprintf(stderr, "  Max SLC erase cycles %d\n", (ecsd[77] << 8) | ecsd[76]);
    // byte [75:72]
    // Average Erase Count in SLC blocks
    avg = ((uint32_t)ecsd[75] << 24) | ((uint32_t)ecsd[74] << 16) | ((uint32_t)ecsd[73] << 8) | ecsd[72];
    fprintf(stderr, "  AVG SLC erase cycles %d.%d\n", avg / 100, avg % 100);
    // byte [71:70]
    // Minimum Erase Count in SLC blocks
    fprintf(stderr, "  Min SLC erase cycles %d\n", (ecsd[71] << 8) | ecsd[70]);

    return (EOK);
}

#define MMC_MICRON_BAD_BLOCK_COUNT  0x08
#define MMC_MICRON_ERASE_COUNT      0x10
#define MMC_MICRON_ERASE_COUNT_SLC  0x11
#define MMC_MICRON_ERASE_COUNT_MLC  0x12
static int micron_health_report(int fd)
{
    uint8_t         *buf = alloca(sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE);
    uint8_t         *bufOriginal = buf;
    int             result;
    MICRON_HEALTH   health;

    memset(buf, 0, sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE);
    SDMMC_GEN_CMD   *cmd = (SDMMC_GEN_CMD *)buf;

    cmd->arg = SDMMC_GENCMD_READ | (MMC_MICRON_BAD_BLOCK_COUNT << 1);
    cmd->flags = SDMMC_GENCMD_READ;
    cmd->blklen = MMC_GEN_CMD_READ_SIZE;
    result = devctl(fd, DCMD_SDMMC_GEN_CMD, buf, sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE, NULL);
    if (result != EOK) {
        fprintf(stderr, "DCMD_SDMMC_GENC_DM failed %d (%s)\n", result, strerror(errno));
        return result;
    }

    buf += sizeof(SDMMC_GEN_CMD);
    health.init_bad_blocks = (buf[0] << 8) | buf[1];
    health.runtime_bad_blocks = (buf[2] << 8) | buf[3];
    health.spare_blocks = (buf[4] << 8) | buf[5];

    /* MMC_MICRON_ERASE_COUNT */
    buf = bufOriginal;
    memset(buf, 0, sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE);
    cmd = (SDMMC_GEN_CMD *)buf;

    cmd->arg = SDMMC_GENCMD_READ | (MMC_MICRON_ERASE_COUNT << 1);
    cmd->flags = SDMMC_GENCMD_READ;
    cmd->blklen = MMC_GEN_CMD_READ_SIZE;
    result = devctl(fd, DCMD_SDMMC_GEN_CMD, buf, sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE, NULL);
    if (result != EOK) {
        fprintf(stderr, "DCMD_SDMMC_GENC_DM failed %d (%s)\n", result, strerror(errno));
        return result;
    }

    buf += sizeof(SDMMC_GEN_CMD);
    health.min_ec = (buf[0] << 8) | buf[1];
    health.max_ec = (buf[2] << 8) | buf[3];
    health.avg_ec = (buf[4] << 8) | buf[5];

    /* MMC_MICRON_ERASE_COUNT_SLC */
    buf = bufOriginal;
    memset(buf, 0, sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE);
    cmd = (SDMMC_GEN_CMD *)buf;

    cmd->arg = SDMMC_GENCMD_READ | (MMC_MICRON_ERASE_COUNT_SLC << 1);
    cmd->flags = SDMMC_GENCMD_READ;
    cmd->blklen = MMC_GEN_CMD_READ_SIZE;
    result = devctl(fd, DCMD_SDMMC_GEN_CMD, buf, sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE, NULL);
    if (result != EOK) {
        fprintf(stderr, "DCMD_SDMMC_GENC_DM failed %d (%s)\n", result, strerror(errno));
        return result;
    }

    buf += sizeof(SDMMC_GEN_CMD);
    health.slc_min_ec = (buf[0] << 8) | buf[1];
    health.slc_max_ec = (buf[2] << 8) | buf[3];
    health.slc_avg_ec = (buf[4] << 8) | buf[5];

    /* MMC_MICRON_ERASE_COUNT_MLC */
    buf = bufOriginal;
    memset(buf, 0, sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE);
    cmd = (SDMMC_GEN_CMD *)buf;

    cmd->arg = SDMMC_GENCMD_READ | (MMC_MICRON_ERASE_COUNT_MLC << 1);
    cmd->flags = SDMMC_GENCMD_READ;
    cmd->blklen = MMC_GEN_CMD_READ_SIZE;
    result = devctl(fd, DCMD_SDMMC_GEN_CMD, buf, sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE, NULL);
    if (result != EOK) {
        fprintf(stderr, "DCMD_SDMMC_GENC_DM failed %d (%s)\n", result, strerror(errno));
        return result;
    }

    buf += sizeof(SDMMC_GEN_CMD);

    health.mlc_min_ec = (buf[0] << 8) | buf[1];
    health.mlc_max_ec = (buf[2] << 8) | buf[3];
    health.mlc_avg_ec = (buf[4] << 8) | buf[5];

    fprintf(stderr, "init bad blocks %d\n", health.init_bad_blocks);
    fprintf(stderr, "runtime bad blocks %d\n", health.runtime_bad_blocks);
    fprintf(stderr, "spareblocks = %d\n", health.spare_blocks);
    fprintf(stderr, "min_ec = %d\n", health.min_ec);
    fprintf(stderr, "max_ec = %d\n", health.max_ec);
    fprintf(stderr, "avg_ec = %d\n", health.avg_ec);
    fprintf(stderr, "slc_min_ec = %d\n", health.slc_min_ec);
    fprintf(stderr, "slc_max_ec = %d\n", health.slc_max_ec);
    fprintf(stderr, "slc_avg_ec = %d\n", health.slc_avg_ec);
    fprintf(stderr, "mlc_min_ec = %d\n", health.mlc_min_ec);
    fprintf(stderr, "mlc_max_ec = %d\n", health.mlc_max_ec);
    fprintf(stderr, "mlc_avg_ec = %d\n", health.mlc_avg_ec);

    return EOK;
}

// Toshiba related commands
#define MMC_TOSHIBA_BAD_BLOCK_COUNT 0x00
#define MMC_TOSHIBA_ERASE_COUNT     0x07
#define MMC_TOSHIBA_PATROL_STATUS   0x16
#define MMC_TOSHIBA_EXEC_STAT_IDX   0x03
#define MMC_TOSHIBA_GEN_CMD_OK      0xA0
#define MMC_TOSHIBA_GEN_CMD_NG      0xE0

static int toshiba_get_cmd56(int fd, int command, uint32_t password, uint8_t *buf)
{
    uint8_t     *buf2 = buf + sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE;
    uint8_t     *cbuf = buf + sizeof(SDMMC_GEN_CMD);
    uint8_t     *cbuf2 = buf2 + sizeof(SDMMC_GEN_CMD);
    int         result;

    SDMMC_GEN_CMD   *cmd = (SDMMC_GEN_CMD *)buf;
    SDMMC_GEN_CMD   *cmd2 = (SDMMC_GEN_CMD *)buf2;

    cmd->flags = SDMMC_GENCMD_WRITE | SDMMC_GENCMD_NEXT;
    cmd->arg = SDMMC_GENCMD_WRITE;

    cbuf[0] = command;
    cbuf[4] = (password >> 24) & 0xFF;
    cbuf[5] = (password >> 16) & 0xFF;
    cbuf[6] = (password >> 8)  & 0xFF;
    cbuf[7] = (password >> 0)  & 0xFF;
    cmd->blklen = MMC_GEN_CMD_READ_SIZE;

    cmd2->flags = SDMMC_GENCMD_READ;
    cmd2->arg = SDMMC_GENCMD_READ;
    memset(cbuf2, 0, MMC_GEN_CMD_READ_SIZE);
    cmd2->blklen = MMC_GEN_CMD_READ_SIZE;

    result = devctl(fd, DCMD_SDMMC_GEN_CMD, buf, (sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE) * 2, NULL);
    if (result != EOK) {
        fprintf(stderr, "DCMD_SDMMC_GENC_DM failed %d (%s)\n", result, strerror(errno));
        return result;
    }

    if(cbuf2[MMC_TOSHIBA_EXEC_STAT_IDX] != MMC_TOSHIBA_GEN_CMD_OK){
        return -1;
    }
    return 0;
}

static int toshiba_health_report(int fd)
{
    uint8_t     *buf = alloca((sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE) * 2);

    printf("MMC_TOSHIBA_ERASE_COUNT:");
    if(toshiba_get_cmd56(fd, MMC_TOSHIBA_ERASE_COUNT, 0x26E901EB, buf) == 0){
        dump_buffer(buf + sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE + sizeof(SDMMC_GEN_CMD), MMC_GEN_CMD_READ_SIZE);
    }else{
        printf(" Failed\n");
    }
    printf("MMC_TOSHIBA_PATROL_STATUS:");
    if(toshiba_get_cmd56(fd, MMC_TOSHIBA_PATROL_STATUS, 0x45091238, buf) == 0){
        dump_buffer(buf + sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE + sizeof(SDMMC_GEN_CMD), MMC_GEN_CMD_READ_SIZE);
    }else{
        printf(" Failed\n");
    }
    printf("MMC_TOSHIBA_BAD_BLOCK_COUNT:");
    if(toshiba_get_cmd56(fd, MMC_TOSHIBA_BAD_BLOCK_COUNT, 0x00294823, buf) == 0){
        dump_buffer(buf + sizeof(SDMMC_GEN_CMD) + MMC_GEN_CMD_READ_SIZE + sizeof(SDMMC_GEN_CMD), MMC_GEN_CMD_READ_SIZE);
    }else{
        printf(" Failed\n");
    }

    // Now look at ExtCSD, it has few intresting pieces
    uint8_t     *ecsd = alloca(MMC_EXT_CSD_SIZE);

    int ret;
    printf("EXT_CSD:");
    if ((ret = get_ext_csd(fd, ecsd)) == EOK) {
        dump_buffer(ecsd, MMC_EXT_CSD_SIZE);
    }else{
        printf(" Failed\n");
    }

    // TODO! interpret data
    // ....................................................

    return EOK;
}

static int sandisk_health_report(int fd)
{
    uint8_t     *buf = alloca((sizeof(SDMMC_MAN_CMD) * 2) + MMC_GEN_CMD_READ_SIZE);
    int         result;

    memset(buf, 0, (sizeof(SDMMC_MAN_CMD) * 2) + MMC_GEN_CMD_READ_SIZE);

    SDMMC_MAN_CMD   *cmd = (SDMMC_MAN_CMD *)buf;
    SDMMC_MAN_CMD   *cmd2 = (SDMMC_MAN_CMD *)(buf + sizeof(SDMMC_MAN_CMD));

    // Sandisk support is still to be verified
    cmd->flags = SDMMC_MANCMD_AC | SDMMC_MANCMD_R1b | SDMMC_MANCMD_CMD3 | SDMMC_MANCMD_NEXT;
    cmd->arg = 0x96C9D71C;
    cmd->blkcnt = 0;
    cmd->blklen = 0;

    cmd2->flags = SDMMC_MANCMD_ADTC | SDMMC_MANCMD_R1b | SDMMC_MANCMD_READ | SDMMC_MANCMD_CMD4;
    cmd2->arg = 0;
    cmd2->blkcnt = 1;
    cmd2->blklen = MMC_GEN_CMD_READ_SIZE;

    result = devctl(fd, DCMD_SDMMC_MAN_CMD, buf, (sizeof(SDMMC_MAN_CMD) * 2) + MMC_GEN_CMD_READ_SIZE, NULL);
    if (result != EOK) {
        fprintf(stderr, "DCMD_SDMMC_MAN_CMD failed %d (%s)\n", result, strerror(errno));
        return result;
    }
    dump_buffer(&buf[sizeof(SDMMC_MAN_CMD) * 2], MMC_GEN_CMD_READ_SIZE);

    // TODO! interpret data
    // ....................................................
    // Data returned from the above command looks all right. If you want to interpret it, please follow SanDisk datasheet.

    return EOK;
}

static int display_ecsd(int fd)
{
    uint8_t    *ecsd = alloca(MMC_EXT_CSD_SIZE);
    int         ret;
    int         idx;

    if ((ret = get_ext_csd(fd, ecsd)) != EOK) {
        fprintf(stderr, "Get ECSD failed\n");
        return (-1);
    }

    for(idx = 0; idx < MMC_EXT_CSD_SIZE; idx++) {
        if(strcmp(raw_ecsd[idx], "") != 0) {
            fprintf(stderr, "%*d - %s: 0x%X\n", 3, idx, raw_ecsd[idx], ecsd[idx]);
        }
    }

    return EOK;
}

static int display_health_report(int fd)
{
    SDMMC_DEVICE_INFO   info;

    if (get_device_info(fd, &info) != EOK) return (EIO);

    switch (info.mid) {
        case MID_MMC_SAMSUNG:
            printf("MID_MMC_SAMSUNG\n");
            samsung_health_report(fd);
            break;
        case MID_MMC_MICRON:
            printf("MID_MMC_MICRON\n");
            micron_health_report(fd);
            break;
        case MID_MMC_TOSHIBA:
            printf("MID_MMC_TOSHIBA\n");
            toshiba_health_report(fd);
            break;
        case MID_MMC_SANDISK_2:
            printf("MID_MMC_SANDISK_2\n");
            sandisk_health_report(fd);
            break;
        default:
            fprintf(stderr, "Manufacture %x health report is not supported!\n", info.mid);
            break;
    }

    return (EOK);
}

#define     CMD_HEALTH_REPORT      1
#define     CMD_DEVICE_INFO        2
#define     CMD_READ_ECSD          3
/*
 */
int main(int argc, char **argv)
{
    int     fd, i;
    int     status = EOK;
    int     cmd = -1;
    char    *dname = "/dev/emmc0";

    while ((i = getopt(argc, argv, "d:f:Fhvr")) != -1) {
        switch (i) {
            case 'd':
                dname = optarg;
                break;
            case 'h':
                cmd = CMD_HEALTH_REPORT;
                break;
            case 'v':
                cmd = CMD_DEVICE_INFO;
                break;
            case 'r':
                cmd = CMD_READ_ECSD;
                break;
           default:
                break;
        }
    }

    if (cmd == -1) {
        fprintf(stderr, "Invalid option, please refer to use file for supported options\n");
        exit(-EINVAL);
    }

    fd = open(dname, O_RDONLY);
    if (fd == -1) {
        fprintf(stderr, "opening %s failed: %s(%d)\n", dname, strerror(errno), errno);
        exit(-1);
    }

    switch (cmd) {
        case CMD_HEALTH_REPORT:
            status = display_health_report(fd);
            break;
        case CMD_DEVICE_INFO:
            status = display_device_info(fd);
            break;
        case CMD_READ_ECSD:
            status = display_ecsd(fd);
        default:
            break;
    }

    close(fd);

    return (status);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/support/nxp/mmc_util/mmc_util.c $ $Rev: 902160 $")
#endif
