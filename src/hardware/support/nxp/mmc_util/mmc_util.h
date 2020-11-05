/*
 * $QNXLicenseC:
 * Copyright 2019, QNX Software Systems.
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

#ifndef _EMMC_UTIL_H_INCLUDED
#define _EMMC_UTIL_H_INCLUDED

#define MMC_EXT_CSD_SIZE                    512

enum ecsd_idx {
    ECSD_CMDQ_MODE_EN                       = 15,
    ECSD_SECURE_REMOVAL_TYPE                = 16,
    ECSD_PRODUCT_STATE_AWARENESS_ENABLEMENT = 17,
    ECSD_MAX_PRE_LOADING_DATA_SIZE0         = 18,
    ECSD_MAX_PRE_LOADING_DATA_SIZE1         = 19,
    ECSD_MAX_PRE_LOADING_DATA_SIZE2         = 20,
    ECSD_MAX_PRE_LOADING_DATA_SIZE3         = 21,
    ECSD_PRE_LOADING_DATA_SIZE0             = 22,
    ECSD_PRE_LOADING_DATA_SIZE1             = 23,
    ECSD_PRE_LOADING_DATA_SIZE2             = 24,
    ECSD_PRE_LOADING_DATA_SIZE3             = 25,
    ECSD_FFU_STATUS                         = 26,
    ECSD_MODE_OPERATION_CODES               = 29,
    ECSD_MODE_CONFIG                        = 30,
    ECSD_BARRIER_CTRL                       = 31,
    ECSD_FLUSH_CACHE                        = 32,
    ECSD_CACHE_CTRL                         = 33,
    ECSD_POWER_OFF_NOTIFICATION             = 34,
    ECSD_EXT_PARTITIONS_ATTRIBUTE0          = 52,
    ECSD_EXT_PARTITIONS_ATTRIBUTE1          = 53,
    ECSD_EXCEPTION_EVENTS_STATUS0           = 54,
    ECSD_EXCEPTION_EVENTS_STATUS1           = 55,
    ECSD_EXCEPTION_EVENTS_CTRL0             = 56,
    ECSD_EXCEPTION_EVENTS_CTRL1             = 57,
    ECSD_DYNCAP_NEEDED                      = 58,
    ECSD_CLASS_6_CTRL                       = 59,
    ECSD_INI_TIMEOUT_EMU                    = 60,
    ECSD_DATA_SECTOR_SIZE                   = 61,
    ECSD_USE_NATIVE_SECTOR                  = 62,
    ECSD_NATIVE_SECTOR_SIZE                 = 63,
    ECSD_PROGRAM_CID_CSD_DDR_SUPPORT        = 130,
    ECSD_PERIODIC_WAKEUP                    = 131,
    ECSD_TCASE_SUPPORT                      = 132,
    ECSD_PRODUCTION_STATE_AWARENESS         = 133,
    ECSD_SEC_BAD_BLK_MGMNT                  = 134,
    ECSD_ENH_START_ADDR0                    = 136,
    ECSD_ENH_START_ADDR1                    = 137,
    ECSD_ENH_START_ADDR2                    = 138,
    ECSD_ENH_START_ADDR3                    = 139,
    ECSD_ENH_SIZE_MULT0                     = 140,
    ECSD_ENH_SIZE_MULT1                     = 141,
    ECSD_ENH_SIZE_MULT2                     = 142,
    ECSD_GP_SIZE_MULT0                      = 143,
    ECSD_GP_SIZE_MULT1                      = 144,
    ECSD_GP_SIZE_MULT2                      = 145,
    ECSD_GP_SIZE_MULT3                      = 146,
    ECSD_GP_SIZE_MULT4                      = 147,
    ECSD_GP_SIZE_MULT5                      = 148,
    ECSD_GP_SIZE_MULT6                      = 149,
    ECSD_GP_SIZE_MULT7                      = 150,
    ECSD_GP_SIZE_MULT8                      = 151,
    ECSD_GP_SIZE_MULT9                      = 152,
    ECSD_GP_SIZE_MULT10                     = 153,
    ECSD_GP_SIZE_MULT11                     = 154,
    ECSD_PARTITION_SETTING                  = 155,
    ECSD_PARTITIONS_ATTR                    = 156,
    ECSD_MAX_ENH_SIZE_MULT0                 = 157,
    ECSD_MAX_ENH_SIZE_MULT1                 = 158,
    ECSD_MAX_ENH_SIZE_MULT2                 = 159,
    ECSD_PARTITIONING_SUP                   = 160,
    ECSD_HPI_MGMT                           = 161,
    ECSD_RST_N_FUNCTION                     = 162,
    ECSD_BKOPS_EN                           = 163,
    ECSD_BKOPS_START                        = 164,
    ECSD_SANITIZE_START                     = 165,
    ECSD_WR_REL_PARAM                       = 166,
    ECSD_WR_REL_SET                         = 167,
    ECSD_RPMB_SIZE_MULT                     = 168,
    ECSD_FW_CONFIG                          = 169,
    ECSD_USER_WP                            = 171,
    ECSD_BOOT_WP                            = 173,
    ECSD_BOOT_WP_STATUS                     = 174,
    ECSD_ERASE_GRP_DEF                      = 175,
    ECSD_BOOT_BUS_CONDITIONS                = 177,
    ECSD_BOOT_CONFIG_PROT                   = 178,
    ECSD_PART_CONFIG                        = 179,
    ECSD_ERASED_MEM_CONT                    = 181,
    ECSD_BUS_WIDTH                          = 183,
    ECSD_STROBE_SUPPORT                     = 184,
    ECSD_HS_TIMING                          = 185,
    ECSD_POWER_CLASS                        = 187,
    ECSD_CMD_SET_REV                        = 189,
    ECSD_CMD_SET                            = 191,
    ECSD_REV                                = 192,
    ECSD_CSD_STRUCTURE                      = 194,
    ECSD_CARD_TYPE                          = 196,
    ECSD_DRIVER_STRENGTH                    = 197,
    ECSD_OUT_OF_INTERRUPT_TIME              = 198,
    ECSD_PARTITION_SWITCH_TIME              = 199,
    ECSD_SEC_CNT0                           = 212,
    ECSD_SEC_CNT1                           = 213,
    ECSD_SEC_CNT2                           = 214,
    ECSD_SEC_CNT3                           = 215,
    ECSD_SLEEP_NOTIFICATION_TIME            = 216,
    ECSD_S_A_TIMEOUT                        = 217,
    ECSD_HC_WP_GRP_SIZE                     = 221,
    ECSD_ERASE_MULT                         = 223,
    ECSD_ERASE_GRP_SIZE                     = 224,
    ECSD_ACC_SIZE                           = 225,
    ECSD_BOOT_SIZE_MULT                     = 226,
    ECSD_SEC_TRIM_MULT                      = 229,
    ECSD_SEC_ERASE_MULT                     = 230,
    ECSD_SEC_FEATURE_SUPPORT                = 231,
    ECSD_TRIM_MULT                          = 232,
    ECSD_BKOPS_STATUS                       = 246,
    ECSD_POWER_OFF_LONG_TIME                = 247,
    ECSD_GENERIC_CMD6_TIME                  = 248,
    ECSD_CACHE_SIZE0                        = 249,
    ECSD_CACHE_SIZE1                        = 250,
    ECSD_CACHE_SIZE2                        = 251,
    ECSD_CACHE_SIZE3                        = 252,
    ECSD_PWR_CL_DDR_200_360                 = 253,
    ECSD_FIRMWARE_VERSION0                  = 254,
    ECSD_FIRMWARE_VERSION1                  = 255,
    ECSD_FIRMWARE_VERSION2                  = 256,
    ECSD_FIRMWARE_VERSION3                  = 257,
    ECSD_FIRMWARE_VERSION4                  = 258,
    ECSD_FIRMWARE_VERSION5                  = 259,
    ECSD_FIRMWARE_VERSION6                  = 260,
    ECSD_FIRMWARE_VERSION7                  = 261,
    ECSD_DEVICE_VERSION0                    = 262,
    ECSD_DEVICE_VERSION1                    = 263,
    ECSD_OPTIMAL_TRIM_UNIT_SIZE             = 264,
    ECSD_OPTIMAL_WRITE_SIZE                 = 265,
    ECSD_OPTIMAL_READ_SIZE                  = 266,
    ECSD_PRE_EOL_INFO                       = 267,
    ECSD_DEVICE_LIFE_TIME_EST_TYP_A         = 268,
    ECSD_DEVICE_LIFE_TIME_EST_TYP_B         = 269,
    ECSD_CMDQ_DEPTH                         = 307,
    ECSD_CMDQ_SUPPORT                       = 308,
    ECSD_BARRIER_SUPPORT                    = 486,
    ECSD_FFU_ARG0                           = 487,
    ECSD_FFU_ARG1                           = 488,
    ECSD_FFU_ARG2                           = 489,
    ECSD_FFU_ARG3                           = 490,
    ECSD_OPERATION_CODE_TIMEOUT             = 491,
    ECSD_TAG_UNIT_SIZE                      = 498,
    ECSD_DATA_TAG_SUPPORT                   = 499,
    ECSD_MAX_PACKED_WRITES                  = 500,
    ECSD_MAX_PACKED_READS                   = 501,
    ECSD_BKOPS_SUPPORTED                    = 502,
    ECSD_HPI_FEATURES                       = 503,
    ECSD_S_CMD_SET                          = 504,
    ECSD_EXT_SECURITY_ERR                   = 505,
    ECSD_MAX_IDX                            = MMC_EXT_CSD_SIZE
};

/* ECSD_FLUSH_CACHE */
#define ECSD_FLUSH_TRIGGER          0x01

/* ECSD_CACHE_CTRL */
#define ECSD_CACHE_CTRL_EN          0x01
#define ECSD_CACHE_CTRL_DIS         0x00

/* ECSD_POWER_OFF_NOTIFICATION */
#define ECSD_NO_POWER_NOTIFICATION  0x00
#define ECSD_POWERED_ON             0x01
#define ECSD_POWER_OFF_SHORT        0x02
#define ECSD_POWER_OFF_LONG         0x03

/* ECSD_USE_NATIVE_SECTOR */
#define ECSD_USE_NATIVE_SECTOR_EN   0x01

/* ECSD_NATIVE_SECTOR_SIZE */
#define ECSD_NATIVE_SECTOR_512      0x00
#define ECSD_NATIVE_SECTOR_4K       0x01
#define ECSD_NATIVE_SECTOR_MSK      0x01

/* ECSD_PARTITION_SETTING */
#define ECSD_PS_CMP                 0x01

/* ECSD_PARTITIONS_ATTR */
#define ECSD_PA_ENH_4               0x10
#define ECSD_PA_ENH_3               0x08
#define ECSD_PA_ENH_2               0x04
#define ECSD_PA_ENH_1               0x02
#define ECSD_PA_ENH_USR             0x01

/* ECSD_PARTITIONING_SUP */
#define ECSD_PS_EXT_ATTR_EN         x04
#define ECSD_PS_ENH_ATTR_EN         0x02
#define ECSD_PS_PART_EN             0x01/

/* ECSD_BKOPS_EN */
#define ECSD_BKOPS_ENABLE           1

/* ECSD_BKOPS_START */
#define ECSD_BKOPS_INITIATE

/* ECSD_SANITIZE_START */
#define ECSD_SANITIZE_INITIATE      1

/* ECSD_WR_REL_PARAM */
#define ECSD_WRP_EN_RPMB_REL_WR     0x10
#define ECSD_WRP_EN_REL_WR          0x04
#define ECSD_WRP_WR_DATA_REL        0x01

/* ECSD_WR_REL_SET */
#define ECSD_WRS_4                  0x10
#define ECSD_WRS_3                  0x08
#define ECSD_WRS_2                  0x04
#define ECSD_WRS_1                  0x02
#define ECSD_WRS_USR                0x01

/* ECSD_USER_WP */
#define ECSD_USER_WP_US_PWR_WP_EN   (1<<0)
#define ECSD_USER_WP_US_PERM_WP_EN  (1<<2)
#define ECSD_USER_WP_US_PWR_WP_DIS  (1<<3)
#define ECSD_USER_WP_US_PERM_WP_DIS (1<<4)
#define ECSD_USER_WP_CD_PERM_WP_DIS (1<<6)
#define ECSD_USER_WP_PERM_PSWD_DIS  (1<<7)

/* ECSD_ERASE_GRP_DEF */
#define ECSD_EGD_EN                 0x01

/* ECSD_PART_CONFIG */
#define ECSD_PC_ACCESS_MSK          0x7Q

/* ECSD_BUS_WIDTH */
#define ECSD_BUS_WIDTH_8            2    // Card is in 8 bit mode
#define ECSD_BUS_WIDTH_4            1    // Card is in 4 bit mode
#define ECSD_BUS_WIDTH_1Q           0    // Card is in 1 bit mode
#define ECSD_BUS_WIDTH_DDR          4    // Add to width for DDR
#define ECSD_BUS_WIDTH_ES           0x80 // (Enhanced strobe) Strobe is provided during Data Out, CRC response and CMD Response

/* ECSD_STROBE_SUPPORT */
#define ECSD_STROBE_SUPPORT_DISABLED    0   // HS400 (5.0)
#define ECSD_STROBE_SUPPORT_ENABLED     1   // HS400ES (5.1)

/* ECSD_HS_TIMING */
#define ECSD_HS_TIMING_DRV_TYPE_SHFT    4
#define ECSD_HS_TIMING_HS400            0x3
#define ECSD_HS_TIMING_HS200            0x2
#define ECSD_HS_TIMING_HS               0x1
#define ECSD_HS_TIMING_LS               0x0

/* ECSD_CARD_TYPE */
#define ECSD_CARD_TYPE_HS400_1_2V   (1<<7)  // Card can run at DDR 200MHz 1.2V
#define ECSD_CARD_TYPE_HS400_1_8V   (1<<6)  // Card can run at DDR 200MHz 1.8V

#define ECSD_CARD_TYPE_HS200_1_2V   (1<<5)  // Card can run at SDR 200MHz 1.2V
#define ECSD_CARD_TYPE_HS200_1_8V   (1<<4)      // Card can run at SDR 200MHz 1.8V

#define ECSD_CARD_TYPE_DDR_1_2V     (1<<3)  // Card can run at 52MHz 1.2V
#define ECSD_CARD_TYPE_DDR_1_8V     (1<<2)  // Card can run at 52MHz 1.8V - 3.0V

#define ECSD_CARD_TYPE_52           (1<<1)  // Card can run at 52MHz
#define ECSD_CARD_TYPE_26           (1<<0)  // Card can run at 26MHz

#define ECSD_CARD_TYPE_DDR           ECSD_CARD_TYPE_DDR_1_8V | ECSD_CARD_TYPE_DDR_1_2V )
#define ECSD_CARD_TYPE_HS400        ( ECSD_CARD_TYPE_HS400_1_8V | ECSD_CARD_TYPE_HS400_1_2V )
#define ECSD_CARD_TYPE_HS200        ( ECSD_CARD_TYPE_HS200_1_8V | ECSD_CARD_TYPE_HS200_1_2V )
#define ECSD_CARD_TYPE_52MHZ        ( ECSD_CARD_TYPE_DDR_1_8V | ECSD_CARD_TYPE_DDR_1_2V | ECSD_CARD_TYPE_52 )
#define ECSD_CARD_TYPE_MSK          0xff

/* ECSD_REV */
#define ECSD_REV_V5_1               8
#define ECSD_REV_V5                 7
#define ECSD_REV_V4_5               6
#define ECSD_REV_V4_41              5
#define ECSD_REV_V4_4               4
#define ECSD_REV_V4_3               3
#define ECSD_REV_V4_2               2
#define ECSD_REV_V4_1               1
#define ECSD_REV_V4                 0

/* ECSD_SEC_CNT0 */
#define ECSD_SEC_CNT_2GB            0x400000

/* ECSD_SEC_FEATURE_SUPPORT */
#define ECSD_SEC_SANITIZE           0x40 // SANITIZE support
#define ECSD_SEC_GB_CL_EN           0x10 // TRIM support
#define ECSD_SEC_BD_BLK_EN          0x04 // Secure purge Bad blk mgnt support
#define ECSD_SEC_ER_EN              0x01 // Secure purge support

/* ECSD_BKOPS_STATUS */
#define ECSD_BS_OPERATIONS_NONE         0
#define ECSD_BS_OPERATIONS_NON_CRITICAL 1
#define ECSD_BS_OPERATIONS_IMPACTED     2
#define ECSD_BS_OPERATIONS_CRITICAL     3

/* ECSD_BKOPS_SUPPORTED */
#define ECSD_BKOPS_SUP                  1

/* ECSD_HPI_FEATURES */
#define EXT_HPI_FEATURES_SUP_CMD12  0x02
#define EXT_HPI_FEATURES_SUPPORTED  0x01

const char *raw_ecsd[MMC_EXT_CSD_SIZE] = {
    "",                                     //0
    "",                                     //1
    "",                                     //2
    "",                                     //3
    "",                                     //4
    "",                                     //5
    "",                                     //6
    "",                                     //7
    "",                                     //8
    "",                                     //9
    "",                                     //10
    "",                                     //11
    "",                                     //12
    "",                                     //13
    "",                                     //14
    "CMDQ_MODE_EN",                         //15
    "SECURE_REMOVAL_TYPE",                  //16
    "PRODUCT_STATE_AWARENESS_ENABLEMENT",   //17
    "MAX_PRE_LOADING_DATA_SIZE0",           //18
    "MAX_PRE_LOADING_DATA_SIZE1",           //19
    "MAX_PRE_LOADING_DATA_SIZE2",           //20
    "MAX_PRE_LOADING_DATA_SIZE3",           //21
    "PRE_LOADING_DATA_SIZE0",               //22
    "PRE_LOADING_DATA_SIZE1",               //23
    "PRE_LOADING_DATA_SIZE2",               //24
    "PRE_LOADING_DATA_SIZE3",               //25
    "FFU_STATUS",                           //26
    "",                                     //27
    "",                                     //28
    "MODE_OPERATION_CODES",                 //29
    "MODE_CONFIG",                          //30
    "BARRIER_CTRL",                         //31
    "FLUSH_CACHE",                          //32
    "CACHE_CTRL",                           //33
    "POWER_OFF_NOTIFICATION",               //34
    "",                                     //35
    "",                                     //36
    "",                                     //37
    "",                                     //38
    "",                                     //39
    "",                                     //40
    "",                                     //41
    "",                                     //42
    "",                                     //43
    "",                                     //44
    "",                                     //45
    "",                                     //46
    "",                                     //47
    "",                                     //48
    "",                                     //49
    "",                                     //50
    "",                                     //51
    "EXT_PARTITIONS_ATTRIBUTE0",            //52
    "EXT_PARTITIONS_ATTRIBUTE1",            //53
    "EXCEPTION_EVENTS_STATUS0",             //54
    "EXCEPTION_EVENTS_STATUS1",             //55
    "EXCEPTION_EVENTS_CTRL0",               //56
    "EXCEPTION_EVENTS_CTRL1",               //57
    "DYNCAP_NEEDED",                        //58
    "CLASS_6_CTRL",                         //59
    "INI_TIMEOUT_EMU",                      //60
    "DATA_SECTOR_SIZE",                     //61
    "USE_NATIVE_SECTOR",                    //62
    "NATIVE_SECTOR_SIZE",                   //63
    "",                                     //64
    "",                                     //65
    "",                                     //66
    "",                                     //67
    "",                                     //68
    "",                                     //69
    "",                                     //70
    "",                                     //71
    "",                                     //72
    "",                                     //73
    "",                                     //74
    "",                                     //75
    "",                                     //76
    "",                                     //77
    "",                                     //78
    "",                                     //79
    "",                                     //80
    "",                                     //81
    "",                                     //82
    "",                                     //83
    "",                                     //84
    "",                                     //85
    "",                                     //86
    "",                                     //87
    "",                                     //88
    "",                                     //89
    "",                                     //90
    "",                                     //91
    "",                                     //92
    "",                                     //93
    "",                                     //94
    "",                                     //95
    "",                                     //96
    "",                                     //97
    "",                                     //98
    "",                                     //99
    "",                                     //100
    "",                                     //101
    "",                                     //102
    "",                                     //103
    "",                                     //104
    "",                                     //105
    "",                                     //106
    "",                                     //107
    "",                                     //108
    "",                                     //109
    "",                                     //110
    "",                                     //111
    "",                                     //112
    "",                                     //113
    "",                                     //114
    "",                                     //115
    "",                                     //116
    "",                                     //117
    "",                                     //118
    "",                                     //119
    "",                                     //120
    "",                                     //121
    "",                                     //122
    "",                                     //123
    "",                                     //124
    "",                                     //125
    "",                                     //126
    "",                                     //127
    "",                                     //128
    "",                                     //129
    "PROGRAM_CID_CSD_DDR_SUPPORT",          //130
    "PERIODIC_WAKEUP",                      //131
    "TCASE_SUPPORT",                        //132
    "PRODUCTION_STATE_AWARENESS",           //133
    "SEC_BAD_BLK_MGMNT",                    //134
    "",                                     //135
    "ENH_START_ADDR0",                      //136
    "ENH_START_ADDR1",                      //137
    "ENH_START_ADDR2",                      //138
    "ENH_START_ADDR3",                      //139
    "ENH_SIZE_MULT0",                       //140
    "ENH_SIZE_MULT1",                       //141
    "ENH_SIZE_MULT2",                       //142
    "GP_SIZE_MULT0",                        //143
    "GP_SIZE_MULT1",                        //144
    "GP_SIZE_MULT2",                        //145
    "GP_SIZE_MULT3",                        //146
    "GP_SIZE_MULT4",                        //147
    "GP_SIZE_MULT5",                        //148
    "GP_SIZE_MULT6",                        //149
    "GP_SIZE_MULT7",                        //150
    "GP_SIZE_MULT8",                        //151
    "GP_SIZE_MULT9",                        //152
    "GP_SIZE_MULT10",                       //153
    "GP_SIZE_MULT11",                       //154
    "PARTITION_SETTING",                    //155
    "PARTITIONS_ATTR",                      //156
    "MAX_ENH_SIZE_MULT0",                   //157
    "MAX_ENH_SIZE_MULT1",                   //158
    "MAX_ENH_SIZE_MULT2",                   //159
    "PARTITIONING_SUP",                     //160
    "HPI_MGMT",                             //161
    "RST_N_FUNCTION",                       //162
    "BKOPS_EN",                             //163
    "BKOPS_START",                          //164
    "SANITIZE_START",                       //165
    "WR_REL_PARAM",                         //166
    "WR_REL_SET",                           //167
    "RPMB_SIZE_MULT",                       //168
    "FW_CONFIG",                            //169
    "",                                     //170
    "USER_WP",                              //171
    "",                                     //172
    "BOOT_WP",                              //173
    "BOOT_WP_STATUS",                       //174
    "ERASE_GRP_DEF",                        //175
    "",                                     //176
    "BOOT_BUS_CONDITIONS",                  //177
    "BOOT_CONFIG_PROT",                     //178
    "PART_CONFIG",                          //179
    "",                                     //180
    "ERASED_MEM_CONT",                      //181
    "",                                     //182
    "BUS_WIDTH",                            //183
    "STROBE_SUPPORT",                       //184
    "HS_TIMING",                            //185
    "",                                     //186
    "POWER_CLASS",                          //187
    "",                                     //188
    "CMD_SET_REV",                          //189
    "",                                     //190
    "CMD_SET",                              //191
    "REV",                                  //192
    "",                                     //193
    "CSD_STRUCTURE",                        //194
    "",                                     //195
    "CARD_TYPE",                            //196
    "DRIVER_STRENGTH",                      //197
    "OUT_OF_INTERRUPT_TIME",                //198
    "PARTITION_SWITCH_TIME",                //199
    "",                                     //200
    "",                                     //201
    "",                                     //202
    "",                                     //203
    "",                                     //204
    "",                                     //205
    "",                                     //206
    "",                                     //207
    "",                                     //208
    "",                                     //209
    "",                                     //210
    "",                                     //211
    "SEC_CNT0",                             //212
    "SEC_CNT1",                             //213
    "SEC_CNT2",                             //214
    "SEC_CNT3",                             //215
    "SLEEP_NOTIFICATION_TIME",              //216
    "S_A_TIMEOUT",                          //217
    "",                                     //218
    "",                                     //219
    "",                                     //220
    "HC_WP_GRP_SIZE",                       //221
    "",                                     //222
    "ERASE_MULT",                           //223
    "ERASE_GRP_SIZE",                       //224
    "ACC_SIZE",                             //225
    "BOOT_SIZE_MULT",                       //226
    "",                                     //227
    "",                                     //228
    "SEC_TRIM_MULT",                        //229
    "SEC_ERASE_MULT",                       //230
    "SEC_FEATURE_SUPPORT",                  //231
    "TRIM_MULT",                            //232
    "",                                     //233
    "",                                     //234
    "",                                     //235
    "",                                     //236
    "",                                     //237
    "",                                     //238
    "",                                     //239
    "",                                     //240
    "",                                     //241
    "",                                     //242
    "",                                     //243
    "",                                     //244
    "",                                     //245
    "BKOPS_STATUS",                         //246
    "POWER_OFF_LONG_TIME",                  //247
    "GENERIC_CMD6_TIME",                    //248
    "CACHE_SIZE0",                          //249
    "CACHE_SIZE1",                          //250
    "CACHE_SIZE2",                          //251
    "CACHE_SIZE3",                          //252
    "PWR_CL_DDR_200_360",                   //253
    "FIRMWARE_VERSION0",                    //254
    "FIRMWARE_VERSION1",                    //255
    "FIRMWARE_VERSION2",                    //256
    "FIRMWARE_VERSION3",                    //257
    "FIRMWARE_VERSION4",                    //258
    "FIRMWARE_VERSION5",                    //259
    "FIRMWARE_VERSION6",                    //260
    "FIRMWARE_VERSION7",                    //261
    "DEVICE_VERSION0",                      //262
    "DEVICE_VERSION1",                      //263
    "OPTIMAL_TRIM_UNIT_SIZE",               //264
    "OPTIMAL_WRITE_SIZE",                   //265
    "OPTIMAL_READ_SIZE",                    //266
    "PRE_EOL_INFO",                         //267
    "DEVICE_LIFE_TIME_EST_TYP_A",           //268
    "DEVICE_LIFE_TIME_EST_TYP_B",           //269
    "",                                     //270
    "",                                     //271
    "",                                     //272
    "",                                     //273
    "",                                     //274
    "",                                     //275
    "",                                     //276
    "",                                     //277
    "",                                     //278
    "",                                     //279
    "",                                     //280
    "",                                     //281
    "",                                     //282
    "",                                     //283
    "",                                     //284
    "",                                     //285
    "",                                     //286
    "",                                     //287
    "",                                     //288
    "",                                     //289
    "",                                     //290
    "",                                     //291
    "",                                     //292
    "",                                     //293
    "",                                     //294
    "",                                     //295
    "",                                     //296
    "",                                     //297
    "",                                     //298
    "",                                     //299
    "",                                     //300
    "",                                     //301
    "",                                     //302
    "",                                     //303
    "",                                     //304
    "",                                     //305
    "",                                     //306
    "CMDQ_DEPTH",                           //307
    "CMDQ_SUPPORT",                         //308
    "",                                     //309
    "",                                     //310
    "",                                     //311
    "",                                     //312
    "",                                     //313
    "",                                     //314
    "",                                     //315
    "",                                     //316
    "",                                     //317
    "",                                     //318
    "",                                     //319
    "",                                     //320
    "",                                     //321
    "",                                     //322
    "",                                     //323
    "",                                     //324
    "",                                     //325
    "",                                     //326
    "",                                     //327
    "",                                     //328
    "",                                     //329
    "",                                     //330
    "",                                     //331
    "",                                     //332
    "",                                     //333
    "",                                     //334
    "",                                     //335
    "",                                     //336
    "",                                     //337
    "",                                     //338
    "",                                     //339
    "",                                     //340
    "",                                     //341
    "",                                     //342
    "",                                     //343
    "",                                     //344
    "",                                     //345
    "",                                     //346
    "",                                     //347
    "",                                     //348
    "",                                     //349
    "",                                     //350
    "",                                     //351
    "",                                     //352
    "",                                     //353
    "",                                     //354
    "",                                     //355
    "",                                     //356
    "",                                     //357
    "",                                     //358
    "",                                     //359
    "",                                     //360
    "",                                     //361
    "",                                     //362
    "",                                     //363
    "",                                     //364
    "",                                     //365
    "",                                     //366
    "",                                     //367
    "",                                     //368
    "",                                     //369
    "",                                     //370
    "",                                     //371
    "",                                     //372
    "",                                     //373
    "",                                     //374
    "",                                     //375
    "",                                     //376
    "",                                     //377
    "",                                     //378
    "",                                     //379
    "",                                     //380
    "",                                     //381
    "",                                     //382
    "",                                     //383
    "",                                     //384
    "",                                     //385
    "",                                     //386
    "",                                     //387
    "",                                     //388
    "",                                     //389
    "",                                     //390
    "",                                     //391
    "",                                     //392
    "",                                     //393
    "",                                     //394
    "",                                     //395
    "",                                     //396
    "",                                     //397
    "",                                     //398
    "",                                     //399
    "",                                     //400
    "",                                     //401
    "",                                     //402
    "",                                     //403
    "",                                     //404
    "",                                     //405
    "",                                     //406
    "",                                     //407
    "",                                     //408
    "",                                     //409
    "",                                     //410
    "",                                     //411
    "",                                     //412
    "",                                     //413
    "",                                     //414
    "",                                     //415
    "",                                     //416
    "",                                     //417
    "",                                     //418
    "",                                     //419
    "",                                     //420
    "",                                     //421
    "",                                     //422
    "",                                     //423
    "",                                     //424
    "",                                     //425
    "",                                     //426
    "",                                     //427
    "",                                     //428
    "",                                     //429
    "",                                     //430
    "",                                     //431
    "",                                     //432
    "",                                     //433
    "",                                     //434
    "",                                     //435
    "",                                     //436
    "",                                     //437
    "",                                     //438
    "",                                     //439
    "",                                     //440
    "",                                     //441
    "",                                     //442
    "",                                     //443
    "",                                     //444
    "",                                     //445
    "",                                     //446
    "",                                     //447
    "",                                     //448
    "",                                     //449
    "",                                     //450
    "",                                     //451
    "",                                     //452
    "",                                     //453
    "",                                     //454
    "",                                     //455
    "",                                     //456
    "",                                     //457
    "",                                     //458
    "",                                     //459
    "",                                     //460
    "",                                     //461
    "",                                     //462
    "",                                     //463
    "",                                     //464
    "",                                     //465
    "",                                     //466
    "",                                     //467
    "",                                     //468
    "",                                     //469
    "",                                     //470
    "",                                     //471
    "",                                     //472
    "",                                     //473
    "",                                     //474
    "",                                     //475
    "",                                     //476
    "",                                     //477
    "",                                     //478
    "",                                     //479
    "",                                     //480
    "",                                     //481
    "",                                     //483
    "",                                     //482
    "",                                     //484
    "",                                     //485
    "BARRIER_SUPPORT",                      //486
    "FFU_ARG0",                             //487
    "FFU_ARG1",                             //488
    "FFU_ARG2",                             //489
    "FFU_ARG3",                             //490
    "OPERATION_CODE_TIMEOUT",               //491
    "",                                     //492
    "",                                     //493
    "",                                     //494
    "",                                     //495
    "",                                     //496
    "",                                     //497
    "TAG_UNIT_SIZE",                        //498
    "DATA_TAG_SUPPORT",                     //499
    "MAX_PACKED_WRITES",                    //500
    "MAX_PACKED_READS",                     //501
    "BKOPS_SUPPORTED",                      //502
    "HPI_FEATURES",                         //503
    "S_CMD_SET",                            //504
    "EXT_SECURITY_ERR",                     //505
    "",                                     //506
    "",                                     //507
    "",                                     //508
    "",                                     //509
    "",                                     //510
    "",                                     //511
};


#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/support/nxp/mmc_util/mmc_util.h $ $Rev: 890707 $")
#endif
