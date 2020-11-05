/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2020 NXP
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

#ifndef IMX_SCI_MGR_H_
#define IMX_SCI_MGR_H_

#include <devctl.h>
#include <hw/nxp/imx8/sci/svc/rm/api.h>
#include <hw/nxp/imx8/sci/svc/pm/api.h>
#include <hw/nxp/imx8/sci/svc/irq/api.h>
#include <hw/nxp/imx8/sci/svc/misc/api.h>
#include <hw/nxp/imx8/sci/svc/pad/api.h>
#include <hw/nxp/imx8/sci/svc/seco/api.h>
#include <hw/nxp/imx8/sci/svc/timer/api.h>
#include <sys/siginfo.h>

/**
 * i.MX8 SC driver
 * @file       sc_imx8/public/sys/imx8_sci_mgr.h
 * @addtogroup sc
 * @{
 */
/** SC driver type mapping */
/*@{*/
/*! This type is used to indicate error response for most functions. */
typedef int         sci_mgr_err_t;
/*! This type is used to indicate a resource. */
typedef int         sci_mgr_rsrc_t;
/*! This type is used to store a boolean */
typedef bool        sci_mgr_bool_t;
/*! This type is used to indicate a control. */
typedef int         sci_mgr_ctrl_t;
/*! This type is used to indicate a pad. Valid values are SoC specific. */
typedef uint16_t    sci_mgr_pad_t;
/*! This type is used to store a system (full-size) address. */
typedef uint64_t    sci_mgr_faddr_t;
/*!
 * This type is used to declare a resource domain ID used by the
 * isolation HW.
 */
typedef uint8_t     sci_mgr_rm_did_t;
/*@}*/

/*!
 * @name Interrupt Types
 */
/*@{*/
/*! This type is used to declare an interrupt group. */
typedef uint8_t sci_mgr_irq_group_t;

/*! This type is used to declare a bit mask of temp interrupts. */
typedef uint8_t sci_mgr_irq_temp_t;

/*! This type is used to declare a bit mask of watchdog interrupts. */
typedef uint8_t sci_mgr_irq_wdog_t;

/*! This type is used to declare a bit mask of RTC interrupts. */
typedef uint8_t sci_mgr_irq_rtc_t;

/*! This type is used to declare a bit mask of wakeup interrupts. */
typedef uint8_t sci_mgr_irq_wake_t;
/*@}*/

/*!
 * @name Miscellaneous Types
 */
/*@{*/
/* This type is used to store a DMA channel priority group. */
typedef uint8_t sci_mgr_misc_dma_group_t;

/*! This type is used report boot status. */
typedef uint8_t sci_mgr_misc_boot_status_t;

/*! This type is used to issue SECO authenticate commands. */
typedef uint8_t sci_mgr_misc_seco_auth_cmd_t;

/*! This type is used report boot status. */
typedef uint8_t sci_mgr_misc_temp_t;
/*@}*/

/*!
 * @name PAD Types
 */
/*@{*/
/*!
 * This type is used to declare a pad config. It determines how the
 * output data is driven, pull-up is controlled, and input signal is
 * connected. Normal and OD are typical and only connect the input
 * when the output is not driven.  The IN options are less common and
 * force an input connection even when driving the output.
 */
typedef uint8_t sci_mgr_pad_config_t;

/*!
 * This type is used to declare a pad low-power isolation config.
 * ISO_LATE is the most common setting. ISO_EARLY is only used when
 * an output pad is directly determined by another input pad. The
 * other two are only used when SW wants to directly contol isolation.
 */
typedef uint8_t sci_mgr_pad_iso_t;

/*!
 * This type is used to declare a drive strength. Note it is specific
 * to 28FDSOI. Also note that valid values depend on the pad type.
 */
typedef uint8_t sci_mgr_pad_28fdsoi_dse_t;

/*!
 * This type is used to declare a pull select. Note it is specific
 * to 28FDSOI.
 */
typedef uint8_t sci_mgr_pad_28fdsoi_ps_t;

/*!
 * This type is used to declare a pull-up select. Note it is specific
 * to 28FDSOI HSIC pads.
 */
typedef uint8_t sci_mgr_pad_28fdsoi_pus_t;

/*! This type is used to declare a wakeup mode of a pad. */
typedef uint8_t sci_mgr_pad_wakeup_t;
/*@}*/

/*!
 * @name Power Management Types
 */
/*@{*/
/*!
 * This type is used to declare a power mode. Note resources only use
 * SC_PM_PW_MODE_OFF and SC_PM_PW_MODE_ON. The other modes are used only
 * as system power modes.
 */
typedef uint8_t sci_mgr_pm_power_mode_t;

/*! This type is used to declare a clock. */
typedef uint8_t sci_mgr_pm_clk_t;

/*! This type is used to declare a clock mode. */
typedef uint8_t sci_mgr_pm_clk_mode_t;

/*! This type is used to declare the clock parent. */
typedef uint8_t sci_mgr_pm_clk_parent_t;

/*! This type is used to declare clock rates. */
typedef uint32_t sci_mgr_pm_clock_rate_t;

/*! This type is used to declare a desired reset type. */
typedef uint8_t sci_mgr_pm_reset_type_t;

/*! This type is used to declare a desired reset type. */
typedef uint8_t sci_mgr_pm_reset_cause;

/*! This type is used to declare a reason for a reset. */
typedef uint8_t sci_mgr_pm_reset_reason_t;

/*! This type is used to specify a system-level interface to be power managed. */
typedef uint8_t sci_mgr_pm_sys_if_t;

/*! This type is used to specify a wake source for CPU resources. */
typedef uint8_t sci_mgr_pm_wake_src_t;
/*@}*/

/*! @name Resource Management Types */
/*@{*/
/*! This type is used to declare a resource partition. */
typedef uint8_t sci_mgr_rm_pt_t;

/*! This type is used to declare a memory region. */
typedef uint8_t sci_mgr_rm_mr_t;

/*!
 * This type is used to declare a resource domain ID used by the
 * isolation HW.
 */
typedef uint8_t sci_mgr_rm_did_t;

/*! This type is used to declare an SMMU StreamID. */
typedef uint16_t sci_mgr_rm_sid_t;

/*! This type is a used to declare master transaction attributes. */
typedef uint8_t sci_mgr_rm_spa_t;

/*!
 * This type is used to declare a resource/memory region access permission.
 * Refer to the XRDC2 Block Guide for more information.
 */
typedef uint8_t sci_mgr_rm_perm_t;
/*@}*/

/*!
 * @name Timer Types
 */
/*@{*/
/*! This type is used to configure the watchdog action. */
typedef uint8_t sci_mgr_timer_wdog_action_t;

/*! This type is used to declare a watchdog time value in milliseconds. */
typedef uint32_t sci_mgr_timer_wdog_time_t;
/*@}*/

/*!
 * @name SECO Types
 */
/*@{*/
typedef uint32_t sci_mgr_seco_rng_stat_t;
/*@}*/

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define IMX_DCMD_PM_OFFSET                           10
#define IMX_DCMD_IRQ_OFFSET                          (IMX_DCMD_PM_OFFSET + 32)
#define IMX_DCMD_MISC_OFFSET                         (IMX_DCMD_IRQ_OFFSET + 32)
#define IMX_DCMD_PAD_OFFSET                          (IMX_DCMD_MISC_OFFSET + 32)
#define IMX_DCMD_RM_OFFSET                           (IMX_DCMD_PAD_OFFSET + 32)
#define IMX_DCMD_TIMER_OFFSET                        (IMX_DCMD_RM_OFFSET + 32)
#define IMX_DCMD_EVENT_OFFSET                        (IMX_DCMD_TIMER_OFFSET + 32)
#define IMX_DCMD_SECO_OFFSET                         (IMX_DCMD_EVENT_OFFSET + 32)

#define IMX_DCMD_PM(index)                           (IMX_DCMD_PM_OFFSET + index)
#define IMX_DCMD_IRQ(index)                          (IMX_DCMD_IRQ_OFFSET + index)
#define IMX_DCMD_MISC(index)                         (IMX_DCMD_MISC_OFFSET + index)
#define IMX_DCMD_PAD(index)                          (IMX_DCMD_PAD_OFFSET + index)
#define IMX_DCMD_RM(index)                           (IMX_DCMD_RM_OFFSET + index)
#define IMX_DCMD_TIMER(index)                        (IMX_DCMD_TIMER_OFFSET + index)
#define IMX_DCMD_EVENT(index)                        (IMX_DCMD_EVENT_OFFSET + index)
#define IMX_DCMD_SECO(index)                         (IMX_DCMD_SECO_OFFSET + index)

/** SCI manager API version: 0.3 */
#define IMX_SCI_MGR_VERSION                          0x0003
#define IMX_SCI_MGR_GET_MAJOR_VERSION                (IMX_SCI_MGR_VERSION >> 8)
#define IMX_SCI_MGR_GET_MINOR_VERSION                (IMX_SCI_MGR_VERSION & 0xFF)

/** @brief Definitions for i.MX SCI devctl commands */

/** Opens SCI channel */
#define IMX_DCMD_SC_OPEN                                 __DIOT(_DCMD_MISC, 1, uint32_t)
/** Closes SCI channel */
#define IMX_DCMD_SC_CLOSE                                __DIOT(_DCMD_MISC, 2, uint32_t)
/* ------------------------------------------------- PM API ------------------------------------------------ */
/** Calls sc_pm_set_sys_power_mode() function */
#define IMX_DCMD_SC_PM_SET_SYS_POWER_MODE                __DIOT(_DCMD_MISC,  IMX_DCMD_PM(0),   imx_dcmd_sc_pm_sys_t)
/** Calls sc_pm_set_partition_power_mode() function */
#define IMX_DCMD_SC_PM_SET_PARTITION_POWER_MODE          __DIOT(_DCMD_MISC,  IMX_DCMD_PM(1),   imx_dcmd_sc_pm_sys_t)
/** Calls sc_pm_get_sys_power_mode() function */
#define IMX_DCMD_SC_PM_GET_SYS_POWER_MODE                __DIOTF(_DCMD_MISC, IMX_DCMD_PM(2),   imx_dcmd_sc_pm_sys_t)
/** Calls sc_pm_set_resource_power_mode() function */
#define IMX_DCMD_SC_PM_SET_RESOURCE_POWER_MODE           __DIOT(_DCMD_MISC,  IMX_DCMD_PM(3),   imx_dcmd_sc_pm_res_t)
/** Calls sc_pm_get_resource_power_mode() function */
#define IMX_DCMD_SC_PM_GET_RESOURCE_POWER_MODE           __DIOTF(_DCMD_MISC, IMX_DCMD_PM(4),   imx_dcmd_sc_pm_res_t)
/** Calls sc_pm_req_low_power_mode() function */
#define IMX_DCMD_SC_PM_REQ_LOW_POWER_MODE                __DIOT(_DCMD_MISC,  IMX_DCMD_PM(5),   imx_dcmd_sc_pm_res_t)
/** Calls sc_pm_set_cpu_resume_addr() function */
#define IMX_DCMD_SC_PM_SET_CPU_RESUME_ADDR               __DIOT(_DCMD_MISC,  IMX_DCMD_PM(6),   imx_dcmd_sc_pm_cpu_resume_addr_t)
/** Calls sc_pm_req_sys_if_power_mode() function */
#define IMX_DCMD_SC_PM_REQ_SYS_IF_POWER_MODE             __DIOT(_DCMD_MISC,  IMX_DCMD_PM(7),   imx_dcmd_sc_pm_req_sys_if_power_mode_t)
/** Calls sc_pm_set_clock_rate() function */
#define IMX_DCMD_SC_PM_SET_CLOCK_RATE                    __DIOT(_DCMD_MISC,  IMX_DCMD_PM(8),   imx_dcmd_sc_pm_clock_rate_t)
/** Calls sc_pm_get_clock_rate() function */
#define IMX_DCMD_SC_PM_GET_CLOCK_RATE                    __DIOTF(_DCMD_MISC, IMX_DCMD_PM(9),   imx_dcmd_sc_pm_clock_rate_t)
/** Calls sc_pm_clock_enable() function */
#define IMX_DCMD_SC_PM_CLOCK_ENABLE                      __DIOT(_DCMD_MISC,  IMX_DCMD_PM(10),   imx_dcmd_sc_pm_clock_en_t)
/** Calls sc_pm_set_clock_parent() function */
#define IMX_DCMD_SC_PM_SET_CLOCK_PARENT                  __DIOT(_DCMD_MISC,  IMX_DCMD_PM(11),   imx_dcmd_sc_pm_clk_parent_t)
/** Calls sc_pm_get_clock_parent() function */
#define IMX_DCMD_SC_PM_GET_CLOCK_PARENT                  __DIOTF(_DCMD_MISC,  IMX_DCMD_PM(12),   imx_dcmd_sc_pm_clk_parent_t)
/** Calls sc_pm_boot() function */
#define IMX_DCMD_SC_PM_BOOT                              __DIOT(_DCMD_MISC,  IMX_DCMD_PM(13),   imx_dcmd_sc_pm_boot_t)
/** Calls sc_pm_reboot() function */
#define IMX_DCMD_SC_PM_REBOOT                            __DIOT(_DCMD_MISC,  IMX_DCMD_PM(14),   sci_mgr_pm_reset_type_t)
/** Calls sc_pm_reboot_partition() function */
#define IMX_DCMD_SC_PM_REBOOT_PARTITION                  __DIOT(_DCMD_MISC,  IMX_DCMD_PM(15),   imx_dcmd_sc_pm_reboot_partition_t)
/** Calls sc_pm_reset() function */
#define IMX_DCMD_SC_PM_RESET                             __DIOT(_DCMD_MISC,  IMX_DCMD_PM(16),  sci_mgr_pm_reset_type_t)
/** Calls sc_pm_reset_reason() function */
#define IMX_DCMD_SC_PM_RESET_REASON                      __DIOTF(_DCMD_MISC, IMX_DCMD_PM(17),  sci_mgr_pm_reset_reason_t)
/** Calls sc_pm_cpu_start() function */
#define IMX_DCMD_SC_PM_CPU_START                         __DIOT(_DCMD_MISC,  IMX_DCMD_PM(18),  imx_dcmd_sc_pm_cpu_t)
/** Calls sc_pm_req_sys_if_power_mode() function */
#define IMX_DCMD_SC_PM_REQ_CPU_LOW_POWER_MODE            __DIOT(_DCMD_MISC,  IMX_DCMD_PM(19),   imx_dcmd_sc_pm_req_cpu_lpm_t)
/** Calls sc_pm_set_cpu_resume() function */
#define IMX_DCMD_SC_PM_SET_CPU_RESUME                    __DIOT(_DCMD_MISC,  IMX_DCMD_PM(20),   imx_dcmd_sc_pm_cpu_resume_t)
/** Calls sc_pm_set_resource_power_mode_all() function */
#define IMX_DCMD_SC_PM_SET_RESOURCE_POWER_MODE_ALL       __DIOT(_DCMD_MISC,  IMX_DCMD_PM(21),   imx_dcmd_sc_pm_res_all_t)
/** Calls sc_pm_cpu_reset() function */
#define IMX_DCMD_SC_PM_CPU_RESET                         __DIOT(_DCMD_MISC,  IMX_DCMD_PM(22),   imx_dcmd_sc_pm_cpu_reset_t)
/** Calls sc_pm_get_reset_part() function */
#define IMX_DCMD_SC_PM_GET_RESET_PART                    __DIOTF(_DCMD_MISC, IMX_DCMD_PM(23),  sci_mgr_rm_pt_t)
/** Calls sc_pm_set_boot_parm() function */
#define IMX_DCMD_SC_PM_SET_BOOT_PARAM                    __DIOT(_DCMD_MISC,  IMX_DCMD_PM(24),  imx_dcmd_sc_pm_set_boot_param_t)
/** Calls sc_pm_reboot_continue() function */
#define IMX_DCMD_SC_PM_REBOOT_CONTINUE                   __DIOT(_DCMD_MISC,  IMX_DCMD_PM(25),  sci_mgr_rm_pt_t)
/** Calls sc_pm_is_partition_started() function */
#define IMX_DCMD_SC_PM_IS_PARTITION_STARTED              __DIOT(_DCMD_MISC,  IMX_DCMD_PM(26),  sci_mgr_rm_pt_t)
/* ------------------------------------------------- IRQ API ----------------------------------------------- */
/** Calls sc_irq_enable() function */
#define IMX_DCMD_SC_IRQ_ENABLE                           __DIOT(_DCMD_MISC,  IMX_DCMD_IRQ(0),  imx_dcmd_sc_irq_en_t)
/** Calls sc_irq_status() function */
#define IMX_DCMD_SC_IRQ_STATUS                           __DIOTF(_DCMD_MISC,  IMX_DCMD_IRQ(1),  imx_dcmd_sc_irq_stat_t)
/* ------------------------------------------------ MISC API ----------------------------------------------- */
/** Calls sc_misc_set_control() function */
#define IMX_DCMD_SC_MISC_SET_CONTROL                     __DIOT(_DCMD_MISC,  IMX_DCMD_MISC(0), imx_dcmd_sc_misc_control_t)
/** Calls sc_misc_get_control() function */
#define IMX_DCMD_SC_MISC_GET_CONTROL                     __DIOTF(_DCMD_MISC, IMX_DCMD_MISC(1), imx_dcmd_sc_misc_control_t)
/** Calls sc_misc_set_ari() function */
#define IMX_DCMD_SC_MISC_SET_ARI                         __DIOT(_DCMD_MISC,  IMX_DCMD_MISC(2), imx_dcmd_sc_misc_ari_t)
/** Calls sc_misc_set_max_dma_group() function */
#define IMX_DCMD_SC_MISC_SET_MAX_DMA_GROUP               __DIOT(_DCMD_MISC,  IMX_DCMD_MISC(3), imx_dcmd_sc_misc_max_dma_group_t)
/** Calls sc_misc_set_dma_group() function */
#define IMX_DCMD_SC_MISC_SET_DMA_GROUP                   __DIOT(_DCMD_MISC,  IMX_DCMD_MISC(4), imx_dcmd_sc_misc_dma_group_t)
/** Calls sc_misc_boot_status() function */
#define IMX_DCMD_SC_MISC_BOOT_STATUS                     __DIOT(_DCMD_MISC,  IMX_DCMD_MISC(7), sci_mgr_misc_boot_status_t)
/** Calls sc_misc_boot_done() function */
#define IMX_DCMD_SC_MISC_BOOT_DONE                       __DIOT(_DCMD_MISC,  IMX_DCMD_MISC(8), sci_mgr_rsrc_t)
/** Calls sc_misc_otp_fuse_read() function */
#define IMX_DCMD_SC_MISC_OTP_FUSE_READ                   __DIOTF(_DCMD_MISC,  IMX_DCMD_MISC(9), imx_dcmd_sc_misc_otp_fuse_t)
/** Calls sc_misc_otp_fuse_write() function */
#define IMX_DCMD_SC_MISC_OTP_FUSE_WRITE                  __DIOT(_DCMD_MISC,  IMX_DCMD_MISC(10), imx_dcmd_sc_misc_otp_fuse_t)
#if 1
    /** Calls sc_misc_waveform_capture() function */
    #define IMX_DCMD_SC_MISC_WAVEFORM_CAPTURE            __DIOT(_DCMD_MISC, IMX_DCMD_MISC(11), sci_mgr_bool_t)
#endif
/** Calls sc_misc_set_temp() function */
#define IMX_DCMD_SC_MISC_SET_TEMP                        __DIOT(_DCMD_MISC,  IMX_DCMD_MISC(12), imx_dcmd_sc_misc_temp_t)
/** Calls sc_misc_get_temp() function */
#define IMX_DCMD_SC_MISC_GET_TEMP                        __DIOTF(_DCMD_MISC,  IMX_DCMD_MISC(13), imx_dcmd_sc_misc_temp_t)
/** Calls sc_misc_build_info() function */
#define IMX_DCMD_SC_MISC_BUILD_INFO                      __DIOF(_DCMD_MISC,  IMX_DCMD_MISC(14), imx_dcmd_sc_misc_build_info_t)
/** Calls sc_misc_get_boot_dev() function */
#define IMX_DCMD_SC_MISC_GET_BOOT_DEV                    __DIOTF(_DCMD_MISC, IMX_DCMD_MISC(15), sci_mgr_bool_t)
/** Calls sc_misc_get_button_status() function */
#define IMX_DCMD_SC_MISC_GET_BUTTON_STATUS               __DIOTF(_DCMD_MISC, IMX_DCMD_MISC(16), sci_mgr_bool_t)
/** Calls sc_misc_build_info() function */
#define IMX_DCMD_SC_MISC_UNIQUE_ID                       __DIOF(_DCMD_MISC,  IMX_DCMD_MISC(24), imx_dcmd_sc_misc_unique_id_t)
/** Calls sc_misc_api_ver() function */
#define IMX_DCMD_SC_MISC_API_VER                         __DIOF(_DCMD_MISC,  IMX_DCMD_MISC(26), imx_dcmd_sc_misc_api_ver_t)
/** Calls sc_misc_board_ioctl() function */
#define IMX_DCMD_SC_MISC_BOARD_IOCTL                     __DIOTF(_DCMD_MISC,  IMX_DCMD_MISC(27), imx_dcmd_sc_misc_board_ioctl_t)
/* ------------------------------------------------- PAD API ----------------------------------------------- */
/** Calls sc_pad_set_mux() function */
#define IMX_DCMD_SC_PAD_SET_MUX                          __DIOT(_DCMD_MISC,  IMX_DCMD_PAD(0),  imx_dcmd_sc_pad_mux_t)
/** Calls sc_pad_set_gp() function */
#define IMX_DCMD_SC_PAD_SET_GP                           __DIOT(_DCMD_MISC,  IMX_DCMD_PAD(1),  imx_dcmd_sc_pad_gp_t)
/** Calls sc_pad_set_gp_28fdsoi() function */
#define IMX_DCMD_SC_PAD_SET_GP_28FDSOI                   __DIOT(_DCMD_MISC,  IMX_DCMD_PAD(2),  imx_dcmd_sc_pad_gp_fdsoi_t)
/** Calls sc_pad_set_wakeup() function */
#define IMX_DCMD_SC_PAD_SET_WAKEUP                       __DIOT(_DCMD_MISC,  IMX_DCMD_PAD(3),  imx_dcmd_sc_pad_wakeup_t)
/** Calls sc_pad_set_all() function */
#define IMX_DCMD_SC_PAD_SET_ALL                          __DIOT(_DCMD_MISC,  IMX_DCMD_PAD(4),  imx_dcmd_sc_pad_all_t)
/** Calls sc_pad_get_mux() function */
#define IMX_DCMD_SC_PAD_GET_MUX                          __DIOTF(_DCMD_MISC, IMX_DCMD_PAD(5),  imx_dcmd_sc_pad_mux_t)
/** Calls sc_pad_get_gp() function */
#define IMX_DCMD_SC_PAD_GET_GP                           __DIOTF(_DCMD_MISC, IMX_DCMD_PAD(6),  imx_dcmd_sc_pad_gp_t)
/** Calls sc_pad_get_gp_28fdsoi() function */
#define IMX_DCMD_SC_PAD_GET_GP_28FDSOI                   __DIOTF(_DCMD_MISC, IMX_DCMD_PAD(7), imx_dcmd_sc_pad_gp_fdsoi_t)
/** Calls sc_pad_get_wakeup() function */
#define IMX_DCMD_SC_PAD_GET_WAKEUP                       __DIOTF(_DCMD_MISC, IMX_DCMD_PAD(8),  imx_dcmd_sc_pad_wakeup_t)
/** Calls sc_pad_get_all() function */
#define IMX_DCMD_SC_PAD_GET_ALL                          __DIOTF(_DCMD_MISC, IMX_DCMD_PAD(9),  imx_dcmd_sc_pad_all_t)
/** Calls sc_pad_set() function */
#define IMX_DCMD_SC_PAD_SET                              __DIOT(_DCMD_MISC,  IMX_DCMD_PAD(10), imx_dcmd_sc_pad_val_t)
/** Calls sc_pad_get() function */
#define IMX_DCMD_SC_PAD_GET                              __DIOTF(_DCMD_MISC, IMX_DCMD_PAD(11), imx_dcmd_sc_pad_val_t)
/** Calls sc_pad_set_gp_28fdsoi_hsic() function */
#define IMX_DCMD_SC_PAD_SET_GP_28FDSOI_HSIC              __DIOT(_DCMD_MISC,  IMX_DCMD_PAD(12),  imx_dcmd_sc_pad_gp_28fdsoi_hsic_t)
/** Calls sc_pad_get_gp_28fdsoi_hsic() function */
#define IMX_DCMD_SC_PAD_GET_GP_28FDSOI_HSIC              __DIOTF(_DCMD_MISC, IMX_DCMD_PAD(13),  imx_dcmd_sc_pad_gp_28fdsoi_hsic_t)
/** Calls sc_pad_set_gp_28fdsoi_comp() function */
#define IMX_DCMD_SC_PAD_SET_GP_28FDSOI_COMP              __DIOT(_DCMD_MISC,  IMX_DCMD_PAD(14), imx_dcmd_sc_pad_gp_fdsoi_comp_t)
/** Calls sc_pad_get_gp_28fdsoi_comp() function */
#define IMX_DCMD_SC_PAD_GET_GP_28FDSOI_COMP              __DIOTF(_DCMD_MISC, IMX_DCMD_PAD(15), imx_dcmd_sc_pad_gp_fdsoi_comp_t)
/* -------------------------------------------------- RM API ----------------------------------------------- */
/** Calls sc_rm_partition_alloc() function */
#define IMX_DCMD_SC_RM_PARTITION_ALLOC                   __DIOTF(_DCMD_MISC,  IMX_DCMD_RM(0),   imx_dcmd_sc_rm_partition_alloc_t)
/** Calls sc_rm_set_confidential() function */
#define IMX_DCMD_SC_RM_SET_CONFIDENTIAL                  __DIOT(_DCMD_MISC,  IMX_DCMD_RM(1),   imx_dcmd_sc_rm_set_confidential_t)
/** Calls sc_rm_partition_free() function */
#define IMX_DCMD_SC_RM_PARTITION_FREE                    __DIOT(_DCMD_MISC,  IMX_DCMD_RM(2),   sci_mgr_rm_pt_t)
/** Calls sc_rm_get_did() function */
#define IMX_DCMD_SC_RM_GET_DID                           __DIOTF(_DCMD_MISC, IMX_DCMD_RM(3),   sci_mgr_rm_did_t)
/** Calls sc_rm_partition_static() function */
#define IMX_DCMD_SC_RM_PARTITION_STATIC                  __DIOT(_DCMD_MISC,  IMX_DCMD_RM(4),   imx_dcmd_sc_rm_partition_static_t)
/** Calls sc_rm_partition_lock() function */
#define IMX_DCMD_SC_RM_PARTITION_LOCK                    __DIOT(_DCMD_MISC,  IMX_DCMD_RM(5),   sci_mgr_rm_pt_t)
/** Calls sc_rm_get_partition() function */
#define IMX_DCMD_SC_RM_GET_PARTITION                     __DIOTF(_DCMD_MISC, IMX_DCMD_RM(6),   sci_mgr_rm_pt_t)
/** Calls sc_rm_set_parent() function */
#define IMX_DCMD_SC_RM_SET_PARENT                        __DIOT(_DCMD_MISC,  IMX_DCMD_RM(7),   imx_dcmd_sc_rm_parent_t)
/** Calls sc_rm_move_all() function */
#define IMX_DCMD_SC_RM_MOVE_ALL                          __DIOT(_DCMD_MISC,  IMX_DCMD_RM(8),   imx_dcmd_sc_rm_all_t)
/** Calls sc_rm_assign_resource() function */
#define IMX_DCMD_SC_RM_ASSIGN_RESOURCE                   __DIOT(_DCMD_MISC,  IMX_DCMD_RM(9),   imx_dcmd_sc_rm_resource_t)
/** Calls sc_rm_set_resource_movable() function */
#define IMX_DCMD_SC_RM_SET_RESOURCE_MOVABLE              __DIOT(_DCMD_MISC,  IMX_DCMD_RM(10),   imx_dcmd_sc_rm_resource_mv_t)
/** Calls sc_rm_set_subsys_rsrc_movable() function */
#define IMX_DCMD_SC_RM_SET_SUBSYS_RSRC_MOVABLE           __DIOT(_DCMD_MISC,  IMX_DCMD_RM(11),   imx_dcmd_sc_rm_subsys_rsrc_mv_t)
/** Calls sc_rm_set_master_attributes() function */
#define IMX_DCMD_SC_RM_SET_MASTER_ATTRIBUTES             __DIOT(_DCMD_MISC,  IMX_DCMD_RM(12),  imx_dcmd_sc_rm_resource_attr_t)
/** Calls sc_rm_set_master_sid() function */
#define IMX_DCMD_SC_RM_SET_MASTER_SID                    __DIOT(_DCMD_MISC,  IMX_DCMD_RM(13),  imx_dcmd_sc_rm_master_sid_t)
/** Calls sc_rm_set_peripheral_permissions() function */
#define IMX_DCMD_SC_RM_SET_PERIPHERAL_PERMISSIONS        __DIOT(_DCMD_MISC,  IMX_DCMD_RM(14),  imx_dcmd_sc_rm_periph_perm_t)
/** Calls sc_rm_is_resource_owned() */
#define IMX_DCMD_SC_RM_IS_RESOURCE_OWNED                 __DIOTF(_DCMD_MISC, IMX_DCMD_RM(15),  sci_mgr_rsrc_t)
/** Calls sc_rm_is_resource_master() */
#define IMX_DCMD_SC_RM_IS_RESOURCE_MASTER                __DIOTF(_DCMD_MISC, IMX_DCMD_RM(16),  sci_mgr_rsrc_t)
/** Calls sc_rm_is_resource_peripheral */
#define IMX_DCMD_SC_RM_IS_RESOURCE_PERIPHERAL            __DIOTF(_DCMD_MISC, IMX_DCMD_RM(17),  sci_mgr_rsrc_t)
/** Calls sc_rm_get_resource_info() function */
#define IMX_DCMD_SC_RM_GET_RESOURCE_INFO                 __DIOTF(_DCMD_MISC, IMX_DCMD_RM(18),  imx_dcmd_sc_rm_master_sid_t)
/** Calls sc_rm_memreg_alloc() function */
#define IMX_DCMD_SC_RM_MEMREG_ALLOC                      __DIOTF(_DCMD_MISC, IMX_DCMD_RM(19),  imx_dcmd_sc_rm_memreg_alloc_t)
/** Calls sc_rm_memreg_split() function */
#define IMX_DCMD_SC_RM_MEMREG_SPLIT                      __DIOTF(_DCMD_MISC, IMX_DCMD_RM(20),  imx_dcmd_sc_rm_memreg_split_t)
/** Calls sc_rm_memreg_free() function */
#define IMX_DCMD_SC_RM_MEMREG_FREE                       __DIOT(_DCMD_MISC,  IMX_DCMD_RM(21),  sci_mgr_rm_mr_t)
/** Calls sc_rm_find_memreg() function */
#define IMX_DCMD_SC_RM_FIND_MEMREG                       __DIOTF(_DCMD_MISC, IMX_DCMD_RM(22),  imx_dcmd_sc_rm_find_memreg_t)
/** Calls sc_rm_assign_memreg() function */
#define IMX_DCMD_SC_RM_ASSIGN_MEMREG                     __DIOT(_DCMD_MISC,  IMX_DCMD_RM(23),  imx_dcmd_sc_rm_memreg_t)
/** Calls sc_rm_set_memreg_permissions() function */
#define IMX_DCMD_SC_RM_SET_MEMREG_PERMISSIONS            __DIOT(_DCMD_MISC,  IMX_DCMD_RM(24),  imx_dcmd_sc_rm_memreg_perm_t)
/** Calls sc_rm_is_memreg_owned() function */
#define IMX_DCMD_SC_RM_IS_MEMREG_OWNED                   __DIOT(_DCMD_MISC,  IMX_DCMD_RM(25),  sci_mgr_rm_mr_t)
/** Calls sc_rm_get_memreg_info() function */
#define IMX_DCMD_SC_RM_GET_MEMREG_INFO                   __DIOTF(_DCMD_MISC, IMX_DCMD_RM(26),  imx_dcmd_sc_rm_memreg_alloc_t)
/** Calls sc_rm_assign_pad() function */
#define IMX_DCMD_SC_RM_ASSIGN_PAD                        __DIOT(_DCMD_MISC,  IMX_DCMD_RM(27),  imx_dcmd_sc_rm_pad_t)
/** Calls sc_rm_set_pin_movable() function */
#define IMX_DCMD_SC_RM_SET_PIN_MOVABLE                   __DIOT(_DCMD_MISC,  IMX_DCMD_RM(28),  imx_dcmd_sc_rm_pin_movable_t)
/** Calls sc_rm_is_pad_owned() function */
#define IMX_DCMD_SC_RM_IS_PAD_OWNED                      __DIOT(_DCMD_MISC,  IMX_DCMD_RM(29),  sci_mgr_pad_t)
/** Calls sc_rm_dump() function */
#define IMX_DCMD_SC_RM_DUMP                              __DIOT(_DCMD_MISC,  IMX_DCMD_RM(30),  uint32_t)
/** Calls sc_rm_memreg_frag() function */
#define IMX_DCMD_SC_RM_MEMREG_FRAG                       __DIOT(_DCMD_MISC,  IMX_DCMD_RM(31),  imx_dcmd_sc_rm_memreg_frag_t)
/* ----------------------------------------------- TIMER API ----------------------------------------------- */
/** Calls sc_timer_set_wdog_timeout() function */
#define IMX_DCMD_SC_TIMER_SET_WDOG_TIMEOUT               __DIOT(_DCMD_MISC,  IMX_DCMD_TIMER(0), sci_mgr_timer_wdog_time_t)
/** Calls sc_timer_set_wdog_pre_timeout() function */
#define IMX_DCMD_SC_TIMER_SET_WDOG_PRE_TIMEOUT           __DIOT(_DCMD_MISC,  IMX_DCMD_TIMER(1), sci_mgr_timer_wdog_time_t)
/** Calls sc_timer_start_wdog() function */
#define IMX_DCMD_SC_TIMER_START_WDOG                     __DIOT(_DCMD_MISC,  IMX_DCMD_TIMER(2), sci_mgr_bool_t)
/** Calls sc_timer_stop_wdog() function */
#define IMX_DCMD_SC_TIMER_STOP_WDOG                      __DIOT(_DCMD_MISC,  IMX_DCMD_TIMER(3), uint32_t)
/** Calls sc_timer_ping_wdog() function */
#define IMX_DCMD_SC_TIMER_PING_WDOG                      __DIOT(_DCMD_MISC,  IMX_DCMD_TIMER(4), uint32_t)
/** Calls sc_timer_get_wdog_status() function */
#define IMX_DCMD_SC_TIMER_GET_WDOG_STATUS                __DIOTF(_DCMD_MISC, IMX_DCMD_TIMER(5), imx_dcmd_sc_timer_status_t)
/** Calls sc_timer_pt_get_wdog_status() function */
#define IMX_DCMD_SC_TIMER_PT_GET_WDOG_STATUS             __DIOTF(_DCMD_MISC, IMX_DCMD_TIMER(6), imx_dcmd_sc_timer_pt_status_t)
/** Calls sc_timer_set_wdog_action() function */
#define IMX_DCMD_SC_TIMER_SET_WDOG_ACTION                __DIOT(_DCMD_MISC,  IMX_DCMD_TIMER(7), imx_dcmd_sc_timer_wdog_action_t)
/** Calls sc_timer_set_rtc_time() function */
#define IMX_DCMD_SC_TIMER_SET_RTC_TIME                   __DIOT(_DCMD_MISC,  IMX_DCMD_TIMER(8), imx_dcmd_sc_timer_time_t)
/** Calls sc_timer_get_rtc_time() function */
#define IMX_DCMD_SC_TIMER_GET_RTC_TIME                   __DIOTF(_DCMD_MISC, IMX_DCMD_TIMER(9), imx_dcmd_sc_timer_time_t)
/** Calls sc_timer_get_rtc_sec1970() function */
#define IMX_DCMD_SC_TIMER_GET_RTC_SEC1970                __DIOTF(_DCMD_MISC, IMX_DCMD_TIMER(10), uint32_t)
/** Calls sc_timer_set_rtc_alarm() function */
#define IMX_DCMD_SC_TIMER_SET_RTC_ALARM                  __DIOT(_DCMD_MISC,  IMX_DCMD_TIMER(11), imx_dcmd_sc_timer_time_t)
/** Calls sc_timer_set_rtc_calb() function */
#define IMX_DCMD_SC_TIMER_SET_RTC_CALB                   __DIOT(_DCMD_MISC, IMX_DCMD_TIMER(12), uint8_t)
/** Calls sc_timer_set_rtc_periodic_alarm() function */
#define IMX_DCMD_SC_TIMER_SET_RTC_PERIODIC_ALARM         __DIOT(_DCMD_MISC, IMX_DCMD_TIMER(13), uint32_t)
/** Calls sc_timer_set_rtc_periodic_alarm() function */
#define IMX_DCMD_SC_TIMER_CANCEL_RTC_ALARM               __DION(_DCMD_MISC, IMX_DCMD_TIMER(14))
/** Calls sc_timer_set_sysctr_alarm() function */
#define IMX_DCMD_SC_TIMER_SET_SYSCTR_ALARM               __DIOT(_DCMD_MISC, IMX_DCMD_TIMER(15), uint64_t)
/** Calls sc_timer_set_sysctr_periodic_alarm() function */
#define IMX_DCMD_SC_TIMER_SET_SYSCTR_PERIODIC_ALARM      __DIOT(_DCMD_MISC, IMX_DCMD_TIMER(16), uint64_t)
/** Calls sc_timer_cancel_sysctr_alarm() function */
#define IMX_DCMD_SC_TIMER_CANCEL_SYSCTR_ALARM            __DION(_DCMD_MISC, IMX_DCMD_TIMER(17))

/* --------------------------------------------- SC driver event API -------------------------------------------- */
/**
 *
 * @param handle Handle to registered EVENT.
 * @param event  Structure that describes an event.
 *
 * @return Status of registration.
 */
#define IMX_DCMD_SC_DRV_EVENT_REGISTER                   __DIOTF(_DCMD_MISC,  IMX_DCMD_EVENT(0), imx_dcmd_sc_event_t)

/**
 * Function for SCFW EVENT unregistration from SCFW.
 *
 * @param handle Handle to unregistered EVENT.
 *
 * @return Status of unregister operation.
 */
#define IMX_DCMD_SC_DRV_EVENT_UNREGISTER                 __DIOTF(_DCMD_MISC,  IMX_DCMD_EVENT(1), imx_dcmd_sc_event_t)

/**
 * Function returns status of registered SCFW EVENTs.
 *
 * @param handle Handle to registered EVENT.
 * @param group  Groups the interrupts are in.
 * @param status Status of interrupts.
 *
 * @return Status of read IRQ status operation.
 */
#define IMX_DCMD_SC_DRV_EVENT_STATUS                     __DIOTF(_DCMD_MISC,  IMX_DCMD_EVENT(2), imx_dcmd_sc_event_status_t)

/**
 * Function returns status of registered SCFW EVENTs.
 *
 * @param handle Handle to registered EVENT.
 * @param group  Groups the interrupts are in.
 * @param mask   Mask of interrupts to affect.
 * @param enable State to change interrupts to.
 *
 * @return Status of read IRQ status operation.
 */
#define IMX_DCMD_SC_DRV_EVENT_ENABLE                     __DIOTF(_DCMD_MISC,  IMX_DCMD_EVENT(3), imx_dcmd_sc_event_enable_t)

/* ----------------------------------------------- SECO API ----------------------------------------------- */
/** Calls sc_seco_image_load() function */
#define IMX_DCMD_SC_SECO_IMAGE_LOAD                      __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(0), imx_dcmd_sc_seco_image_load_t)
/** Calls sc_seco_authenticate() function */
#define IMX_DCMD_SC_SECO_AUTHENTICATE                    __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(1), imx_dcmd_sc_seco_authenticate_t)
/** Calls sc_seco_enh_authenticate() function */
#define IMX_DCMD_SC_SECO_ENH_AUTHENTICATE                __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(2), imx_dcmd_sc_seco_enh_authenticate_t)
/** Calls sc_seco_forward_lifecycle() function */
#define IMX_DCMD_SC_SECO_FORWARD_LIFECYCLE               __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(3), uint32_t)
/** Calls sc_seco_return_lifecycle() function */
#define IMX_DCMD_SC_SECO_RETURN_LIFECYCLE                __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(4), sci_mgr_faddr_t)
/** Calls sc_seco_commit() function */
#define IMX_DCMD_SC_SECO_COMMIT                          __DIOF(_DCMD_MISC,  IMX_DCMD_SECO(5), uint32_t)
/** Calls sc_seco_attest_mode() function */
#define IMX_DCMD_SC_SECO_ATTEST_MODE                     __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(6), uint32_t)
/** Calls sc_seco_attest() function */
#define IMX_DCMD_SC_SECO_ATTEST                          __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(7), uint64_t)
/** Calls sc_seco_get_attest_pkey() function */
#define IMX_DCMD_SC_SECO_GET_ATTEST_PKEY                 __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(8), sci_mgr_faddr_t)
/** Calls sc_seco_get_attest_sign() function */
#define IMX_DCMD_SC_SECO_GET_ATTEST_SIGN                 __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(9), sci_mgr_faddr_t)
/** Calls sc_seco_attest_verify() function */
#define IMX_DCMD_SC_SECO_ATTEST_VERIFY                   __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(10), sci_mgr_faddr_t)
/** Calls sc_seco_gen_key_blob() function */
#define IMX_DCMD_SC_SECO_GEN_KEY_BLOB                    __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(11), imx_dcmd_sc_seco_gen_key_blob_t)
/** Calls sc_seco_load_key() function */
#define IMX_DCMD_SC_SECO_LOAD_KEY                        __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(12), imx_dcmd_sc_seco_load_key_t)
/** Calls sc_seco_get_mp_key() function */
#define IMX_DCMD_SC_SECO_GET_MP_KEY                      __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(13), imx_dcmd_sc_seco_get_mp_key_t)
/** Calls sc_seco_update_mpmr() function */
#define IMX_DCMD_SC_SECO_UPDATE_MPMR                     __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(14), imx_dcmd_sc_seco_update_mpmr_t)
/** Calls sc_seco_get_mp_sign() function */
#define IMX_DCMD_SC_GET_MP_SIGN                          __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(15), imx_dcmd_sc_seco_get_mp_sign_t)
/** Calls sc_seco_build_info() function */
#define IMX_DCMD_SC_SECO_BUILD_INFO                      __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(16), imx_dcmd_sc_seco_build_info_t)
/** Calls sc_seco_chip_info() function */
#define IMX_DCMD_SC_SECO_CHIP_INFO                       __DIOF(_DCMD_MISC,  IMX_DCMD_SECO(17), imx_dcmd_sc_seco_chip_info_t)
/** Calls sc_seco_enable_debug() function */
#define IMX_DCMD_SC_SECO_ENABLE_DEBUG                    __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(18), sci_mgr_faddr_t)
/** Calls sc_seco_get_event() function */
#define IMX_DCMD_SC_SECO_GET_EVENT                       __DIOTF(_DCMD_MISC,  IMX_DCMD_SECO(19), imx_dcmd_sc_seco_get_event_t)
/** Calls sc_seco_fuse_write() function */
#define IMX_DCMD_SC_SECO_FUSE_WRITE                      __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(20), sci_mgr_faddr_t)
/** Calls sc_seco_patch() function */
#define IMX_DCMD_SC_SECO_PATCH                           __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(21), sci_mgr_faddr_t)
/** Calls sc_seco_start_rng() function */
#define IMX_DCMD_SC_SECO_START_RNG                       __DIOF(_DCMD_MISC,  IMX_DCMD_SECO(22), sc_seco_rng_stat_t)
/** Calls sc_seco_sab_msg() function */
#define IMX_DCMD_SC_SECO_SAB_MSG                         __DIOT(_DCMD_MISC,  IMX_DCMD_SECO(23), sci_mgr_faddr_t)
/** Calls sc_seco_secvio_enable() function */
#define IMX_DCMD_SC_SECO_SECVIO_ENABLE                   __DION(_DCMD_MISC,  IMX_DCMD_SECO(24))
/** Calls sc_seco_secvio_config() function */
#define IMX_DCMD_SC_SECO_SECVIO_CONFIG                   __DIOTF(_DCMD_MISC,  IMX_DCMD_SECO(25), imx_dcmd_sc_seco_secvio_config_t)
/** Calls sc_seco_secvio_dgo_config() function */
#define IMX_DCMD_SC_SECO_SECVIO_DGO_CONFIG               __DIOTF(_DCMD_MISC,  IMX_DCMD_SECO(26), imx_dcmd_sc_seco_secvio_dgo_config_t)

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_SET_SYS_POWER_MODE
 *              - IMX_DCMD_SC_PM_GET_SYS_POWER_MODE
 */
typedef struct _imx_dcmd_sc_pm_sys {
    sci_mgr_rm_pt_t pt;              /**< Handle of partition */
    sci_mgr_pm_power_mode_t mode;    /**< Power mode */
} imx_dcmd_sc_pm_sys_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_SET_RESOURCE_POWER_MODE
 *              - IMX_DCMD_SC_PM_GET_RESOURCE_POWER_MODE
 */
typedef struct _imx_dcmd_sc_pm_res {
    sci_mgr_rsrc_t resource;         /**< ID of the resource */
    sci_mgr_pm_power_mode_t mode;    /**< Power mode */
} imx_dcmd_sc_pm_res_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_SET_RESOURCE_POWER_MODE
 */
typedef struct _imx_dcmd_sc_pm_req_cpu_lpm {
    sci_mgr_rsrc_t resource;        /**< ID of the resource */
    sci_mgr_pm_power_mode_t mode;   /**< Power mode */
    sci_mgr_pm_wake_src_t wake_src; /**< */
} imx_dcmd_sc_pm_req_cpu_lpm_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_CPU_RESUME_ADDR
 */
typedef struct _imx_dcmd_sc_pm_cpu_resume_addr {
    sci_mgr_rsrc_t resource;        /**< ID of the CPU resource */
    sci_mgr_faddr_t address;        /**< 64-bit resume address */
} imx_dcmd_sc_pm_cpu_resume_addr_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_CPU_RESUME
 */
typedef struct _imx_dcmd_sc_pm_cpu_resume {
    sci_mgr_rsrc_t resource;        /**< ID of the CPU resource */
    sci_mgr_bool_t isPrimary;       /**< Set true if primary wake CPU */
    sci_mgr_faddr_t address;        /**< 64-bit resume address */
} imx_dcmd_sc_pm_cpu_resume_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_REQ_SYS_IF_POWER_MDE
 */
typedef struct _imx_dcmd_sc_pm_req_sys_if_power_mode {
    sci_mgr_rsrc_t resource;        /**< ID of the resource */
    sci_mgr_pm_sys_if_t sys_if;     /**< System-level interface to be configured */
    sci_mgr_pm_power_mode_t hpm;    /**< High-power mode for the system interface */
    sci_mgr_pm_power_mode_t lpm;    /**< Low-power mode for the system interface */
} imx_dcmd_sc_pm_req_sys_if_power_mode_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_SET_CLOCK_RATE
 *              - IMX_DCMD_SC_PM_GET_CLOCK_RATE
 */
typedef struct _imx_dcmd_sc_pm_clock_rate {
    sci_mgr_rsrc_t resource;        /**< ID of the resource */
    sci_mgr_pm_clk_t clk;           /**< Clock/PLL */
    sci_mgr_pm_clock_rate_t rate;   /**< Pointer to a rate */
} imx_dcmd_sc_pm_clock_rate_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_CLOCK_ENABLE
 */
typedef struct _imx_dcmd_sc_pm_clock_en {
    sci_mgr_rsrc_t resource;        /**< ID of the resource */
    sci_mgr_pm_clk_t clk;           /**< Clock/PLL */
    sci_mgr_bool_t enable;          /**< Enable if true; otherwise disabled */
    sci_mgr_bool_t autog;           /**< HW auto clock gating */
} imx_dcmd_sc_pm_clock_en_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_SET_CLOCK_PARENT
 *              - IMX_DCMD_SC_PM_GET_CLOCK_PARENT
 */
typedef struct _imx_dcmd_sc_pm_clk_parent_t {
    sci_mgr_rsrc_t resource;        /**< ID of the resource */
    sci_mgr_pm_clk_t clk;           /**< Clock/PLL */
    sci_mgr_pm_clk_parent_t parent; /**< Clock parent */
} imx_dcmd_sc_pm_clk_parent_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_BOOT
 */
typedef struct _imx_dcmd_sc_pm_boot {
    sci_mgr_rm_pt_t pt;             /**< Handle of partition. */
    sci_mgr_rsrc_t boot_cpu;        /**< ID of the CPU resource to start */
    sci_mgr_faddr_t boot_addr;      /**< 64-bit boot address */
    sci_mgr_rsrc_t boot_mu;         /**< ID of the MU that must be powered */
    sci_mgr_rsrc_t dev;             /**< ID of the boot device that must be powered */
} imx_dcmd_sc_pm_boot_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_CPU_START
 */
typedef struct _imx_dcmd_sc_pm_cpu {
    sci_mgr_rsrc_t resource;        /**< ID of the resource */
    sci_mgr_bool_t enable;          /**< Start if true; otherwise stop */
    sci_mgr_faddr_t boot_addr;      /**< 64-bit boot address */
} imx_dcmd_sc_pm_cpu_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_CPU_RESET
 */
typedef struct _imx_dcmd_sc_pm_cpu_reset {
    sci_mgr_rsrc_t resource;        /**< ID of the CPU resource */
    sci_mgr_faddr_t address;        /**< 64-bit resume address */
} imx_dcmd_sc_pm_cpu_reset_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_SET_BOOT_PARAM
 */
typedef struct _imx_dcmd_sc_pm_set_boot_param {
    sci_mgr_rsrc_t resource_cpu;   /**< ID of the CPU resource to start */
    sci_mgr_faddr_t boot_addr;     /**< 64-bit boot address */
    sci_mgr_rsrc_t resource_mu;    /**< ID of the MU that must be powered (0=none) */
    sci_mgr_rsrc_t resource_dev;   /**< ID of the boot device that must be powered (0=none) */
} imx_dcmd_sc_pm_set_boot_param_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_REBOOT_PARTITION
 */
typedef struct _imx_dcmd_sc_pm_reboot_partition {
    sci_mgr_rm_pt_t pt;             /**< Handle of partition to reboot */
    sci_mgr_pm_reset_type_t type;   /**< Reset type */
} imx_dcmd_sc_pm_reboot_partition_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PM_SET_RESOURCE_POWER_MODE_ALL
 */
typedef struct _imx_dcmd_sc_pm_res_all {
    sci_mgr_rm_pt_t pt;             /**< Handle of child partition */
    sci_mgr_pm_power_mode_t mode;   /**< Power mode to apply */
    sci_mgr_rsrc_t exclude;         /**< Resource to exclude */
} imx_dcmd_sc_pm_res_all_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_IRQ_ENABLE
 */
typedef struct _imx_dcmd_sc_irq_en {
    sci_mgr_rsrc_t resource;        /**< ID of the resource */
    sci_mgr_irq_group_t group;      /**< Groups the interrupts are in */
    uint32_t mask;                  /**< Mask of interrupts to affect */
    sci_mgr_bool_t enable;          /**< State to change interrupts to */
} imx_dcmd_sc_irq_en_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_IRQ_STATUS
 */
typedef struct _imx_dcmd_sc_irq_stat {
    sci_mgr_rsrc_t resource;        /**< ID of the resource */
    sci_mgr_irq_group_t group;      /**< Groups the interrupts are in */
    uint32_t status;                /**< Status of interrupts */
} imx_dcmd_sc_irq_stat_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_SET_CONTROL
 *              - IMX_DCMD_SC_MISC_GET_CONTROL
 */
typedef struct _imx_dcmd_sc_misc_control {
    sci_mgr_rsrc_t resource;        /**< ID of the resource */
    sci_mgr_ctrl_t ctrl;            /**< Control to change */
    uint32_t val;                   /**< Value to apply to the control */
} imx_dcmd_sc_misc_control_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_SET_ARI
 */
typedef struct _imx_dcmd_sc_misc_ari {
    sci_mgr_rsrc_t resource;         /**< ID of the resource */
    sci_mgr_rsrc_t master;           /**< PCIe/SATA master to match */
    uint16_t ari;                    /**< ARI to match */
    sci_mgr_bool_t enable;           /**< Enable match or not */
} imx_dcmd_sc_misc_ari_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_SET_MAX_DMA_GROUP
 */
typedef struct _imx_dcmd_sc_misc_max_dma_group {
    sci_mgr_rm_pt_t pt;              /**< Handle of partition to assign \a max */
    sci_mgr_misc_dma_group_t max;    /**< Max priority group (0-31) */
} imx_dcmd_sc_misc_max_dma_group_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_SET_DMA_GROUP
 */
typedef struct _imx_dcmd_sc_misc_dma_group {
    sci_mgr_rsrc_t resource;         /**< ID of the resource */
    sci_mgr_misc_dma_group_t group;  /**< Priority group (0-31) */
} imx_dcmd_sc_misc_dma_group_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_OTP_FUSE_READ
 *              - IMX_DCMD_SC_MISC_OTP_FUSE_WRITE
 */
typedef struct _imx_dcmd_sc_misc_otp_fuse {
    uint32_t word;                   /**< Fuse word index */
    uint32_t val;                    /**< Fuse value */
} imx_dcmd_sc_misc_otp_fuse_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_SET_TEMP
 *              - IMX_DCMD_SC_MISC_GET_TEMP
 */
typedef struct _imx_dcmd_sc_misc_temp_t {

    sci_mgr_rsrc_t resource;         /**< Resource with sensor */
    sci_mgr_misc_temp_t temp;        /**< Alarm to set */
    int16_t celsius;                 /**< Whole part of temp to set */
    int8_t tenths;                   /**< Fractional part of temp to set */
} imx_dcmd_sc_misc_temp_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_BUILD_INFO
 */
typedef struct _imx_dcmd_sc_misc_build_info_t {
    uint32_t build;                  /**< Build number */
    uint32_t commit;                 /**< Commit ID (git SHA-1) */
} imx_dcmd_sc_misc_build_info_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_CHIP_INFO
 */
typedef struct _imx_dcmd_sc_misc_api_ver {
    uint16_t cl_maj;                 /**< Major part of client version */
    uint16_t cl_min;                 /**< Minor part of client version */
    uint16_t sv_maj;                 /**< Major part of SCFW version */
    uint16_t sv_min;                 /**< Minor part of SCFW version */
} imx_dcmd_sc_misc_api_ver_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_BOARD_IOCTL
 */
typedef struct _imx_dcmd_sc_misc_board_ioctl {
    uint32_t parm1;                 /**< Parameter 1 */
    uint32_t parm2;                 /**< Parameter 2 */
    uint32_t parm3;                 /**< Parameter 3 */
} imx_dcmd_sc_misc_board_ioctl_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_MISC_UNIQUE_ID
 */
typedef struct _imx_dcmd_sc_misc_unique_id {
    uint32_t id_l;                   /**< Pointer to return lower 32-bit of ID [31:0] */
    uint32_t id_h;                   /**< Pointer to return upper 32-bits of ID [63:32] */
} imx_dcmd_sc_misc_unique_id_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PAD_SET_MUX
 *              - IMX_DCMD_SC_PAD_GET_MUX
 */
typedef struct _imx_dcmd_sc_pad_mux {
    sci_mgr_pad_t pad;               /**< Pad to configure */
    uint8_t mux;                     /**< Mux setting */
    sci_mgr_pad_config_t config;     /**< Pad config */
    sci_mgr_pad_iso_t iso;           /**< Low-power isolation mode */
} imx_dcmd_sc_pad_mux_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PAD_SET_GP
 *              - IMX_DCMD_SC_PAD_GET_GP
 */
typedef struct _imx_dcmd_sc_pad_gp {
    sci_mgr_pad_t pad;               /**< Pad to configure */
    uint32_t ctrl;                   /**< Control value to set */
} imx_dcmd_sc_pad_gp_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PAD_SET_GP_28FDSOI_HSIC
 *              - IMX_DCMD_SC_PAD_GET_GP_28FDSOI_HSIC
 */
typedef struct _imx_dcmd_sc_pad_gp_28fdsoi_hsic {
    sci_mgr_pad_t pad;               /**< Pad to configure */
    sci_mgr_pad_28fdsoi_dse_t dse;   /**< Drive strength */
    sci_mgr_bool_t hys;              /**< Hysteresis */
    sci_mgr_pad_28fdsoi_pus_t pus;   /**< Pull-up select */
    sci_mgr_bool_t pke;              /**< Pull keeper enable */
    sci_mgr_bool_t pue;              /**< Pull-up enable */
} imx_dcmd_sc_pad_gp_28fdsoi_hsic_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PAD_SET_WAKEUP
 *              - IMX_DCMD_SC_PAD_GET_WAKEUP
 */
typedef struct _imx_dcmd_sc_pad_wakeup {
    sci_mgr_pad_t pad;               /**< Pad to configure */
    sci_mgr_pad_wakeup_t wakeup;     /**< Wakeup to set */
} imx_dcmd_sc_pad_wakeup_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PAD_SET_PAD_ALL
 *              - IMX_DCMD_SC_PAD_GET_PAD_ALL
 */
typedef struct _imx_dcmd_sc_pad_all {
    sci_mgr_pad_t pad;               /**< Pad to configure */
    uint8_t mux;                     /**< Mux setting */
    sci_mgr_pad_config_t config;     /**< Pad config */
    sci_mgr_pad_iso_t iso;           /**< Low-power isolation mode */
    uint32_t ctrl;                   /**< Control value to set */
    sci_mgr_pad_wakeup_t wakeup;     /**< Wakeup to set */
} imx_dcmd_sc_pad_all_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PAD_SET
 */
typedef struct _imx_dcmd_sc_pad_val {
    sci_mgr_pad_t pad;               /**< Pad to configure */
    uint32_t val;                    /**< Value to set */
} imx_dcmd_sc_pad_val_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PAD_SET_GP_28FDSOI
 *              - IMX_DCMD_SC_PAD_GET_GP_28FDSOI
 */
typedef struct _imx_dcmd_sc_pad_gp_fdsoi {
    sci_mgr_pad_t pad;               /**< Pad to configure */
    sci_mgr_pad_28fdsoi_dse_t dse;   /**< Drive strength */
    sci_mgr_pad_28fdsoi_ps_t ps;     /**< Pull select */
} imx_dcmd_sc_pad_gp_fdsoi_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_PAD_SET_GP_28FDSOI_COMP
 *              - IMX_DCMD_SC_PAD_GET_GP_28FDSOI_COMP
 */
typedef struct _imx_dcmd_sc_pad_gp_fdsoi_comp {
    sci_mgr_pad_t pad;               /**< Pad to configure */
    uint8_t compen;                  /**< Compensation/freeze mode */
    sci_mgr_bool_t fastfrz;          /**< Fast freeze */
    uint8_t rasrcp;                  /**< Compensation code for PMOS */
    uint8_t rasrcn;                  /**< Compensation code for NMOS */
    sci_mgr_bool_t nasrc_sel;        /**< NASRC read select */
    sci_mgr_bool_t psw_ovr;          /**< Pointer to return the 2.5v override */
    /** These are applicable only for IMX_DCMD_SC_PAD_GET_GP_28FDSOI_COMP */
    sci_mgr_bool_t compok;           /**< Compensation status */
    uint8_t nasrc;                   /**< NASRCP/NASRCN */
} imx_dcmd_sc_pad_gp_fdsoi_comp_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_PARTITION_ALLOC
 */
typedef struct _imx_dcmd_sc_rm_partition_alloc {
    sci_mgr_rm_pt_t pt;              /**< Handle for partition; used for subsequent function */
    sci_mgr_bool_t secure;           /**< Boolean indicating if this partition should be secure; only
                                          valid if caller is secure */
    sci_mgr_bool_t isolated;         /**< Boolean indicating if this partition should be HW isolated
                                          via XRDC; set true if new DID is desired */
    sci_mgr_bool_t restricted;       /**< Boolean indicating if this partition should be restricted; set
                                          true if masters in this partition cannot create new partitions */
    sci_mgr_bool_t grant;            /**< Boolean indicating if this partition should always grant
                                          access and control to the parent */
    sci_mgr_bool_t coherent;         /**< Boolean indicating if this partition is coherent;
                                          set true if only this partition will contain both AP clusters
                                          and they will be coherent via the CCI */
} imx_dcmd_sc_rm_partition_alloc_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_SET_CONFIDENTIAL
 */
typedef struct _imx_dcmd_sc_rm_set_confidential {
    sci_mgr_rm_pt_t pt;              /**< Handle of partition that is granting */
    sci_mgr_bool_t retro;            /**< Retro active */
} imx_dcmd_sc_rm_set_confidential_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_PARTITION_STATIC
 */
typedef struct _imx_dcmd_sc_rm_partition_static {
    sci_mgr_rm_pt_t pt;              /**< Handle for partition; used for subsequent function */
    sci_mgr_rm_did_t did;            /**< Static DID to assign */
} imx_dcmd_sc_rm_partition_static_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_SET_PARENT
 */
typedef struct _imx_dcmd_sc_rm_parent {
    sci_mgr_rm_pt_t pt;              /**< Handle of partition for which parent is to be changed */
    sci_mgr_rm_pt_t parent;          /**< Handle of partition to set as parent */
} imx_dcmd_sc_rm_parent_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_MOVE_ALL
 */
typedef struct _imx_dcmd_sc_rm_all {
    sci_mgr_rm_pt_t pt_src;          /**< Handle of partition from which resources should be moved from */
    sci_mgr_rm_pt_t pt_dst;          /**< Handler of partition to which resources should be moved to */
    sci_mgr_bool_t move_rsrc;        /**< Boolean to indicate if resources should be moved */
    sci_mgr_bool_t move_pins;        /**< Boolean to indicate if pins should be moved */
} imx_dcmd_sc_rm_all_t ;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_ASSIGN_RESOURCE
 */
typedef struct _imx_dcmd_sc_rm_resource {
    sci_mgr_rm_pt_t pt;              /**< Handle of partition for which resource should be assigned */
    sci_mgr_rsrc_t resource;         /**< Resource to assign */
} imx_dcmd_sc_rm_resource_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_SET_RESOURCE_MOVABLE
 */
typedef struct _imx_dcmd_sc_rm_resource_mv {
    sci_mgr_rsrc_t resource_fst;     /**< First resource for which flag should be set */
    sci_mgr_rsrc_t resource_lst;     /**< Last resource for which flag should be set */
    sci_mgr_bool_t movable;          /**< Movable flag */
} imx_dcmd_sc_rm_resource_mv_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_SET_SUBSYS_RSRC_MOVABLE
 */
typedef struct _imx_dcmd_sc_rm_subsys_rsrc_mv {
    sci_mgr_rsrc_t resource;         /**< Resource to use to identify subsystem */
    sci_mgr_bool_t movable;          /**< Movable flag */
} imx_dcmd_sc_rm_subsys_rsrc_mv_t;


/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_SET_MASTER_ATTRIBUTES
 */
typedef struct _imx_dcmd_sc_rm_resource_attr {
    sci_mgr_rsrc_t resource;         /**< Resource for which attributes should apply */
    sci_mgr_rm_spa_t sa;             /**< Security attribute */
    sci_mgr_rm_spa_t pa;             /**< Privilege attribute */
    sci_mgr_bool_t smmu_bypass;      /**< SMMU bypass mode */
} imx_dcmd_sc_rm_resource_attr_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_SET_MASTER_SID
 *              - IMX_DCMD_SC_RM_GET_RESOURCE_INFO
 */
typedef struct _imx_dcmd_sc_rm_master_sid {
    sci_mgr_rsrc_t resource;         /**< Master resource */
    sci_mgr_rm_sid_t sid;            /**< StreamID */
} imx_dcmd_sc_rm_master_sid_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_SET_PERIPHERAL_PERMISSIONS
 */
typedef struct _imx_dcmd_sc_rm_periph_perm {
    sci_mgr_rsrc_t resource;         /**< Master resource */
    sci_mgr_rm_pt_t pt;              /**< Handle of partition \a perm should by applied for */
    sci_mgr_rm_perm_t perm;          /**< Permissions to apply to \a resource for \a pt */
} imx_dcmd_sc_rm_periph_perm_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_MEMREG_ALLOC
 */
typedef struct _imx_dcmd_sc_rm_memreg_alloc {
    sci_mgr_rm_mr_t mr;              /**< Return handle for region; used for
                                          subsequent function calls associated with this region */
    sci_mgr_faddr_t start;           /**< Start address of region (physical) */
    sci_mgr_faddr_t end;             /**< End address of region (physical) */
} imx_dcmd_sc_rm_memreg_alloc_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_MEMREG_SPLIT
 */
typedef struct _imx_dcmd_sc_rm_memreg_split {
    sci_mgr_rm_mr_t mr;              /**< Handle of memory region to split */
    sci_mgr_rm_mr_t mr_ret;          /**< Return handle for region; used for subsequent function calls associated with this region */
    sci_mgr_faddr_t addr_start;      /**< Start address of region (physical) */
    sci_mgr_faddr_t addr_end;        /**< End address of region (physical) */
} imx_dcmd_sc_rm_memreg_split_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_FIND_MEMREG
 */
typedef struct _imx_dcmd_sc_rm_find_memreg {
    sci_mgr_rm_mr_t
    mr;                  /**< Return handle for region; used for subsequent function calls associated with this region */
    sci_mgr_faddr_t addr_start;      /**< Start address of region (physical) */
    sci_mgr_faddr_t addr_end;        /**< End address of region (physical) */
} imx_dcmd_sc_rm_find_memreg_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_ASSIGN_MEMREG
 */
typedef struct _imx_dcmd_sc_rm_memreg {
    sci_mgr_rm_pt_t pt;              /**< Handle of partition to which memory region should be assigned */
    sci_mgr_rm_mr_t mr;              /**< Handle of memory region to assign */
} imx_dcmd_sc_rm_memreg_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_SET_MEMREG_PERMISSIONS
 */
typedef struct _imx_dcmd_sc_rm_memreg_perm {
    sci_mgr_rm_mr_t mr;              /**< Handle of memory region for which permissions should apply */
    sci_mgr_rm_pt_t pt;              /**< Handle of partition \a perm should by applied for */
    sci_mgr_rm_perm_t perm;          /**< Permissions to apply to \a mr for \a pt */
} imx_dcmd_sc_rm_memreg_perm_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_ASSIGN_PAD
 */
typedef struct _imx_dcmd_sc_rm_pad {
    sci_mgr_rm_pt_t pt;              /**< Handle of partition to which pad should be assigned */
    sci_mgr_pad_t pad;               /**< Pad to assign */
} imx_dcmd_sc_rm_pad_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_RM_MEMREG_FRAG
 */
typedef struct _imx_dcmd_sc_rm_memreg_frag {
    sci_mgr_rm_mr_t  mr;             /**< Return handle */
    sci_mgr_faddr_t start_address;   /**< 64-bit start address */
    sci_mgr_faddr_t end_address;     /**< 64-bit end address */
} imx_dcmd_sc_rm_memreg_frag_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_TIMER_GET_WDOG_STATUS
 */
typedef struct _imx_dcmd_sc_timer_status {
    sci_mgr_timer_wdog_time_t timeout; /**< Pointer to return the timeout */
    sci_mgr_timer_wdog_time_t max_timeout; /**< Pointer to return the max timeout */
    sci_mgr_timer_wdog_time_t remaining_time; /**< Pointer to return the time remaining until trigger */
} imx_dcmd_sc_timer_status_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_TIMER_PT_GET_WDOG_STATUS
 */
typedef struct _imx_dcmd_sc_timer_pt_status {
    sci_mgr_rm_pt_t pt;              /**< Partition to query */
    sci_mgr_bool_t enb;              /**< Pointer to return enable status */
    sci_mgr_timer_wdog_time_t timeout; /**< Pointer to return the timeout */
    sci_mgr_timer_wdog_time_t remaining_time; /**< Pointer to return the time remaining until trigger */
} imx_dcmd_sc_timer_pt_status_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_TIMER_SET_WDOG_ACTION
 */
typedef struct _imx_dcmd_sc_timer_wdog_action_t {
    sci_mgr_rm_pt_t pt;              /**< Partition to affect */
    sci_mgr_timer_wdog_action_t action; /**< Action to be taken when a watchdog expires */
} imx_dcmd_sc_timer_wdog_action_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_TIMER_SET_RTC_TIME
 */
typedef struct _imx_dcmd_sc_timer_time {
    uint16_t year;                   /**< Pointer to return year (min 1970) */
    uint8_t mon;                     /**< Pointer to return month (1-12) */
    uint8_t day;                     /**< Pointer to return day of the month (1-31) */
    uint8_t hour;                    /**< Pointer to return hour (0-23) */
    uint8_t min;                     /**< Pointer to return minute (0-59) */
    uint8_t sec;                     /**< Pointer to return second (0-59) */
} imx_dcmd_sc_timer_time_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_DRV_EVENT_REGISTER
 *              - IMX_DCMD_SC_DRV_EVENT_UNREGISTER
 */
typedef struct _imx_dcmd_sc_event {
    void* handle;                    /**< Handle to registered EVENT */
    struct sigevent event;           /**< Structure that describes an event */
} imx_dcmd_sc_event_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_DRV_EVENT_STATUS
 */
typedef struct _imx_dcmd_sc_event_status {
    void* handle;                    /**< Handle to registered EVENT */
    sci_mgr_irq_group_t group;       /**< Groups the interrupts are in */
    uint32_t status;                 /**< Status of interrupts */
} imx_dcmd_sc_event_status_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_DRV_EVENT_ENABLE
 */
typedef struct _imx_dcmd_sc_event_enable {
    void* handle;                    /**< Handle to registered EVENT */
    sci_mgr_irq_group_t group;       /**< Groups the interrupts are in */
    uint32_t mask;                   /**< Mask of interrupts to affect */
    bool enable;                     /**< State to change interrupts to */
} imx_dcmd_sc_event_enable_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_IMAGE_LOAD
 */
typedef struct _imx_dcmd_sc_seco_image_load {
    sci_mgr_faddr_t addr_src;      /**< Address of image source */
    sci_mgr_faddr_t addr_dst;      /**< Address of image destination */
    uint32_t        len;           /**< Length of image to load */
    sci_mgr_bool_t  fw;            /**< SC_TRUE = firmware load */
} imx_dcmd_sc_seco_image_load_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_AUTHENTICATE
 */
typedef struct _imx_dcmd_sc_seco_authenticate {
    sc_seco_auth_cmd_t cmd;        /**< Authenticate command */
    sci_mgr_faddr_t    addr;       /**< Address of/or metadata */
} imx_dcmd_sc_seco_authenticate_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_ENH_AUTHENTICATE
 */
typedef struct _imx_dcmd_sc_seco_enh_authenticate {
    sc_seco_auth_cmd_t cmd;        /**< Authenticate command */
    sci_mgr_faddr_t    addr;       /**< Address of/or metadata */
    uint32_t           mask1;      /**< Metadata */
    uint32_t           mask2;      /**< Metadata */
} imx_dcmd_sc_seco_enh_authenticate_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_GEN_KEY_BLOB
 */
typedef struct _imx_dcmd_sc_seco_gen_key_blob {
    uint32_t        id;            /**< Key identifier */
    sci_mgr_faddr_t load_addr;     /**< Load address */
    sci_mgr_faddr_t export_addr;   /**< Export address */
    uint16_t        max_size;      /**< Max export size */
} imx_dcmd_sc_seco_gen_key_blob_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_LOAD_KEY
 */
typedef struct _imx_dcmd_sc_seco_load_key {
    uint32_t        id;            /**< Key identifier */
    sci_mgr_faddr_t addr;          /**< Key address */
} imx_dcmd_sc_seco_load_key_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_GET_MP_KEY
 */
typedef struct _imx_dcmd_sc_seco_get_mp_key {
    sci_mgr_faddr_t dst_addr;      /**< Destination address */
    uint16_t        dst_size;      /**< Destination size */
} imx_dcmd_sc_seco_get_mp_key_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_UPDATE_MPMR
 */
typedef struct _imx_dcmd_sc_seco_update_mpmr {
    sci_mgr_faddr_t addr;          /**< Data address */
    uint8_t         size;          /**< Size */
    uint8_t         lock;          /**< Lock_reg */
} imx_dcmd_sc_seco_update_mpmr_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_GET_MP_SIGN
 */
typedef struct _imx_dcmd_sc_seco_get_mp_sign {
    sci_mgr_faddr_t msg_addr;      /**< Message address */
    uint16_t        msg_size;      /**< Message size */
    sci_mgr_faddr_t dst_addr;      /**< Destination address */
    uint16_t        dst_size;      /**< Destination size */
} imx_dcmd_sc_seco_get_mp_sign_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_BUILD_INFO
 */
typedef struct _imx_dcmd_sc_seco_build_info {
    uint32_t version;              /**< Build number */
    uint32_t commit;               /**< Commit ID (git SHA-1) */
} imx_dcmd_sc_seco_build_info_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_CHIP_INFO
 */
typedef struct _imx_dcmd_sc_seco_chip_info {
    uint16_t lc;                   /**< Lifecycle */
    uint16_t monotonic;            /**< Monotonic counter */
    uint32_t uid_l;                /**< UID (lower 32 bits) */
    uint32_t uid_h;                /**< UID (upper 32 bits) */
} imx_dcmd_sc_seco_chip_info_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_GET_EVENT
 */
typedef struct _imx_dcmd_sc_seco_get_event {
    uint8_t  idx;                  /**< Index of event to return */
    uint32_t event;                /**< Event */
} imx_dcmd_sc_seco_get_event_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_SECVIO_CONFIG
 */
typedef struct _imx_dcmd_sc_seco_secvio_config {
    uint8_t  id;                   /**< Register ID */
    uint8_t  access;               /**< 0=read, 1=write */
    uint32_t data0;                /**< Data to read or write */
    uint32_t data1;                /**< Data to read or write */
    uint32_t data2;                /**< Data to read or write */
    uint32_t data3;                /**< Data to read or write */
    uint32_t data4;                /**< Data to read or write */
    uint8_t  size;                 /**< Number of valid data words */
} imx_dcmd_sc_seco_secvio_config_t;

/** Structure for passing data with DCMD commands:
 *              - IMX_DCMD_SC_SECO_SECVIO_DGO_CONFIG
 */
typedef struct _imx_dcmd_sc_seco_secvio_dgo_config {
    uint8_t  id;                   /**< Register ID */
    uint8_t  access;               /**< 0=read, 1=write */
    uint32_t data;                 /**< Pointer to data to read or write */
} imx_dcmd_sc_seco_secvio_dgo_config_t;

/** @} */ /* end of sc */

#endif /* IMX_SCI_MGR_H_ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/support/sc-imx8/public/sys/imx8_sci_mgr.h $ $Rev: 913737 $")
#endif
