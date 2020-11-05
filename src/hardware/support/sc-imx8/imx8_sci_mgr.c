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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <devctl.h>
#include <login.h>
#include <sys/iofunc.h>
#include <sys/dispatch.h>
#include <sys/neutrino.h>
#include <sys/procmgr.h>
#include <sys/resmgr.h>
#include <sys/slog.h>
#include <sys/slogcodes.h>
#include <sys/asound.h>
#include <sys/mman.h>
#include <sys/imx8_sci_mgr.h>
#include <hw/inout.h>
#include <aarch64/imx8_common/imx_mu.h>
#include <sys/hwinfo.h>
#include <drvr/hwinfo.h>

/**
 * i.MX SC driver
 * @file       sc/imx8_sci_mgr.c
 * @addtogroup sc
 * @{
 */

/** Macros used for debug and error information */
#define IMX_SCI_SLOG_OPCODE     _SLOG_SETCODE(_SLOGC_NPAPM,0)
#define IMX_SCI_SLOG_ERROR      _SLOG_ERROR
#define IMX_SCI_SLOG_DEBUG      _SLOG_DEBUG1
#define IMX_SCI_ERROR(...)      slogf(IMX_SCI_SLOG_OPCODE,IMX_SCI_SLOG_ERROR,"sc-imx: "__VA_ARGS__)
#define IMX_SCI_DEBUG(...)      slogf(IMX_SCI_SLOG_OPCODE,IMX_SCI_SLOG_DEBUG,"sc-imx: "__VA_ARGS__)

/** Pulse priority */
#define IMX_SC_PRIORITY         21
/** MU interrupt */
#define IMX_SC_ISR_EVENT        23

/** SCI HWI opts structure */
static char *sci_hwi_opts[] = {
    "smc_call",                             /**< SMC call support */
    NULL
};

/** SCI IRQ event structure */
typedef struct imx_event_item {
    uint32_t                irq_status;
    uint32_t                irq_enable;
} imx_event_item_t;

/** SCI IRQ event structure */
typedef struct imx_irq_event {
    struct imx_irq_event    *next;
    struct sigevent         event;
    int                     recvid;
    RESMGR_OCB_T            *ocb;
    imx_event_item_t        event_item[SC_IRQ_NUM_GROUP];
} imx_irq_event_t;

/** SCI common data structure */
typedef struct imx_sci_data {
    int                     optv;           /**< Verbose mode state */
    pthread_mutex_t         hw_lock;        /**< Mutex used for lock access to peripheral registers */
    uint32_t                base;           /**< SCI channel base address */
    sc_ipc_id_t             id;             /**< SCI channel virtual address */
    int                     intr;           /**< Interrupt event number from reference manual */
    int                     iid;            /**< Interrupt event ID returned by InterruptAttachEvent() */
    int                     chid;           /**< Main SC driver channel id */
    int                     coid;           /**< Interrupt connection */
    pthread_t               intr_tid;       /**< MU interrupt thread ID */
    struct sigevent         intrevent;      /**< sigevent structure which is delivered when interrupt occurs */
    imx_irq_event_t         *irq_event;     /**< IRQ event structure */
    sc_rsrc_t               grp_res;        /**< Resource group. */
    int                     malloc_cnt;     /**< Current count of the called malloc() function (for debug) */
    bool                    smc_call;       /**< QNX RTOS runs with ATF */
} imx_sci_data_t;

/** Global pointer to SCI data structure */
imx_sci_data_t *sci;

#define SCI_PHY_MEM_SIZE        65536

/* These prototypes are needed since we are using their names */
int imx_sci_init(void);
void options(int argc, char **argv);
void exit_handler(int dummy);

/* A resource manager mainly consists of callbacks for POSIX
 * functions a client could call. We have
 * callbacks for the open(), close and devctl() calls. */
int io_open(resmgr_context_t *ctp, io_open_t *msg, RESMGR_HANDLE_T *handle, void *extra);
int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb);
int io_close_ocb(resmgr_context_t *ctp, void *reserved, RESMGR_OCB_T *ocb);

/*
 * Our connect and I/O functions - we supply two tables
 * which will be filled with pointers to callback functions
 * for each POSIX function. The connect functions are all
 * functions that take a path, e.g. open(), while the I/O
 * functions are those functions that are used with a file
 * descriptor (fd), e.g. read().
 */
resmgr_connect_funcs_t connect_funcs;
resmgr_io_funcs_t io_funcs;

/*
 * Our dispatch, resource manager, and iofunc variables
 * are declared here. These are some small administrative things
 * for our resource manager.
 */
dispatch_t *dpp;
resmgr_attr_t rattr;
dispatch_context_t *ctp;
iofunc_attr_t ioattr;

char *progname = "imx-sci";

/*
 * For drop-root
 */
char *UserParm;

/**
 * Test if item with the "hadle" is in the event list.
 *
 * @param sci    Pointer to the SCI data structure.
 * @param handle Handle to item in the list.
 *
 * @return Test status, returns true if item is available in the event list.
 */
static bool is_item_in_list(imx_sci_data_t *sci, void * handle)
{
    imx_irq_event_t *cur_irq_event;
    bool result = false;

    /* Check if item is in list */
    cur_irq_event = sci->irq_event;
    while (cur_irq_event != NULL) {
        if (cur_irq_event == (imx_irq_event_t *)handle) {
            result = true;
            break;
        }
        cur_irq_event = cur_irq_event->next;
    }
    return result;
}

/**
 * Main function.
 *
 * @param argc Arguments count
 * @param argv Command line arguments.
 *
 * @return Execution status.
 */
int main(int argc, char **argv)
{
    int             pathID, hwi_status;
    unsigned        hwi_off;
    hwiattr_t       sc_attr;
    char            *optstr, *freeptr, *c, *value;
    int             opt;

    /* Allocates SCI struct */
    sci = (imx_sci_data_t *) calloc(1, sizeof(imx_sci_data_t));
    if (!sci) {
        IMX_SCI_ERROR("Couldn't allocate SCI data structure");
        exit(1);
    }
    if (ThreadCtl(_NTO_TCTL_IO_PRIV, 0) != 0) {
        IMX_SCI_ERROR("ThreadCtl failed");
        exit(1);
    }
    /* Initialize IPC channel */
    sci->base = 0x00U;
    /* Initialize IPC IRQ */
    sci-> intr = 0x00U;
    /* Use default MU resource for all groups */
    sci->grp_res = SC_R_MU_0A;
    /* Initialize smc_call status */
    sci->smc_call = false;
    /* Check for command line options (-vb) */
    options(argc, argv);
    if (sci->optv) {
        IMX_SCI_DEBUG("starting...");
    }
    signal(SIGINT, exit_handler);
    signal(SIGKILL, exit_handler);
    signal(SIGTERM, exit_handler);

    sci->irq_event = NULL;
    hwi_off = hwi_find_device("sc", 0);
    if (hwi_off != HWI_NULL_OFF) {
        if (sci->base == 0x00U) {
            hwi_tag *tag_location = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, 0);
            /* Peripheral base address - update if not already set*/
            if (tag_location != NULL) {
                sci->base = tag_location->location.base;
            }
        }
        if (sci->intr == 0x00U) {
            /* IRQ vector number - update if not already set*/
            hwi_tag *tag_irq = hwi_tag_find(hwi_off, HWI_TAG_NAME_irq, 0);
            if (tag_irq != NULL) {
                sci->intr = tag_irq->irq.vector;
            }
        }
        /* Get and parse "optstr" HWI option */
        hwi_status = hwiattr_get_timer(hwi_off, (hwiattr_timer_t *)&sc_attr);
        if (hwi_status == EOK) {
            if (sc_attr.common.optstr != NULL) {
                freeptr = optstr = strdup(sc_attr.common.optstr);
                while ((optstr != NULL) && (*optstr != '\0')) {
                    c = optstr;
                    if ((opt = getsubopt(&optstr, sci_hwi_opts, &value)) == -1) {
                        IMX_SCI_ERROR("Unknown option %s", c);
                        continue;
                    }
                    switch (opt) {
                        case 0: /* smc_call */
                            if (strcmp(value, "yes") == 0) {
                                sci->smc_call = true;
                                sci->grp_res = SC_R_MU_1A;
                            }
                            continue;
                    }
                }
                free(freeptr);
            }
        }
    }
    if (sci->optv) {
        IMX_SCI_DEBUG("MU BASE: 0x%x", sci->base);
        IMX_SCI_DEBUG("IRQ: %d", sci->intr);
        IMX_SCI_DEBUG("SMC call: %s", (sci->smc_call) ? "yes" : "no");
    }
    imx_sci_init();

    /* Allocate and initialize a dispatch structure for use
     * by our main loop. This is for the resource manager
     * framework to use. It will receive messages for us,
     * analyze the message type integer and call the matching
     * handler callback function (i.e. io_open, io_read, etc.) */
    dpp = dispatch_create();

    if (dpp == NULL) {
        IMX_SCI_ERROR("couldn't dispatch_create: %s", strerror(errno));
        munmap_device_memory ((void *)sci->id, SCI_PHY_MEM_SIZE);
        exit(1);
    }
    /* Set up the resource manager attributes structure. We'll
     * use this as a way of passing information to
     * resmgr_attach(). The attributes are used to specify
     * the maximum message length to be received at once,
     * and the number of message fragments (iov's) that
     * are possible for the reply.
     * For now, we'll just use defaults by setting the
     * attribute structure to zeroes. */
    memset(&rattr, 0, sizeof(rattr));

    /* Now, let's initialize the tables of connect functions and
     * I/O functions to their defaults (system fallback
     * routines) and then override the defaults with the
     * functions that we are providing. */
    iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs, _RESMGR_IO_NFUNCS,
                     &io_funcs);

    /* Now we override the default function pointers with
     * some of our own coded functions: */
    connect_funcs.open = io_open;
    io_funcs.devctl = io_devctl;
    io_funcs.close_ocb = io_close_ocb;

    /* Initialize the device attributes for the particular
     * device name we are going to register. It consists of
     * permissions, type of device, owner and group ID */
    iofunc_attr_init(&ioattr, S_IFCHR | 0666, NULL, NULL);

    /* Next we call resmgr_attach() to register our device name
     * with the process manager, and also to let it know about
     * our connect and I/O functions. */
    pathID = resmgr_attach(dpp, &rattr, "/dev/sc", _FTYPE_ANY, 0,
                           &connect_funcs, &io_funcs, &ioattr);
    if (pathID == -1) {
        IMX_SCI_ERROR("couldn't attach pathname: %s", strerror(errno));
        munmap_device_memory((void *)sci->id, SCI_PHY_MEM_SIZE);
        exit(1);
    }
    /* Now we allocate some memory for the dispatch context
     * structure, which will later be used when we receive
     * messages. */
    ctp = dispatch_context_alloc(dpp);

    /*
     * Drop-root if we need to.
     */
    if (UserParm != NULL) {
        if(procmgr_ability( 0,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_SESSION,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_KEYDATA,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_RSRCDBMGR,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_IO,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_INTERRUPT,
                            PROCMGR_AOP_DENY  | PROCMGR_ADN_NONROOT | PROCMGR_AOP_LOCK      | PROCMGR_AID_EOL)
                             != EOK){
            IMX_SCI_ERROR("Unable to gain procmgr abilities for nonroot operation\n");
            exit(1);
        }
        if(set_ids_from_arg(UserParm) != EOK){
            IMX_SCI_ERROR("%s: Unable to drop to user %s: %s", argv[0], UserParm, strerror(errno));
            exit(1);
        }
        free(UserParm);
    }

    /* Done! We can now go into our "receive loop" and wait
     * for messages. The dispatch_block() function is calling
     * MsgReceive() under the covers, and receives for us.
     * The dispatch_handler() function analyzes the message
     * for us and calls the appropriate callback function. */
    while (1) {
        if ((ctp = dispatch_block(ctp)) == NULL) {
            IMX_SCI_ERROR("dispatch_block failed: %s", strerror(errno));
            continue;
        }
        /* Call the correct callback function for the message
         * received. This is a single-threaded resource manager,
         * so the next request will be handled only when this
         * call returns. Consult our documentation if you want
         * to create a multi-threaded resource manager. */
        dispatch_handler(ctp);
    }
}

/**
 * Common io_open callback.
 *
 * @param ctp    Dispatch context
 * @param msg    io_open message
 * @param handle Structure identifying the target device.
 * @param extra  Reserved parameter
 *
 * @return EOK when success.
 */
int io_open(resmgr_context_t *ctp, io_open_t *msg, RESMGR_HANDLE_T *handle, void *extra)
{
    if (sci->optv) {
        IMX_SCI_DEBUG("in io_open");
    }
    return (iofunc_open_default(ctp, msg, handle, extra));
}
/**
 * io_close_ocb callback.
 *
 * @param ctp      Dispatch context
 * @param reserved Reserved parameter
 * @param ocb      OCB structure
 *
 * @return EOK when success.
 */
int io_close_ocb(resmgr_context_t *ctp, void *reserved, RESMGR_OCB_T *ocb)
{
    imx_irq_event_t *cur_irq_event, *prev_irq_event = NULL;
    uint32_t irq_enable[SC_IRQ_NUM_GROUP];
    unsigned int idx;

    if (sci->optv) {
        IMX_SCI_DEBUG("in io_close");
    }
    (void)pthread_mutex_lock(&sci->hw_lock);
    /* Remove all items in list with the closed 'ocb' */
    cur_irq_event = sci->irq_event;
    while (cur_irq_event != NULL) {
        if (cur_irq_event->ocb == ocb) {
            if (prev_irq_event == NULL) {
                sci->irq_event = cur_irq_event->next;
            } else {
                prev_irq_event->next = cur_irq_event->next;
            }
            free((void *)cur_irq_event);
            sci->malloc_cnt--;
            /* Disable IRQ in SCFW */
            memset((void *)irq_enable, 0x00, (sizeof(uint32_t) * SC_IRQ_NUM_GROUP));
            cur_irq_event = sci->irq_event;
            while (cur_irq_event != NULL) {
                for (idx = 0; idx < SC_IRQ_NUM_GROUP; idx++) {
                    irq_enable[idx] |= cur_irq_event->event_item[idx].irq_enable;
                }
                cur_irq_event = cur_irq_event->next;
            }
            for (idx = 0; idx < SC_IRQ_NUM_GROUP; idx++) {
                if (irq_enable[idx] == 0) {
                    (void)sc_irq_enable(sci->id, sci->grp_res, idx, irq_enable[idx], false);
                }
            }
        } else {
            prev_irq_event = cur_irq_event;
        }
        if (cur_irq_event != NULL) {
            cur_irq_event = cur_irq_event->next;
        }
    }
    (void)pthread_mutex_unlock(&sci->hw_lock);

    if (sci->optv) {
        IMX_SCI_DEBUG("--> malloc count: %d", sci->malloc_cnt);
    }
    return iofunc_close_ocb_default(ctp, reserved, ocb);
}

/**
 * io_devctl common callback.
 *
 * This function is common io_devctl callback.
 *
 * @param ctp Dispatch context
 * @param msg io_open message
 * @param ocb OCB structure
 *
 * @return EOK when success.
 */
int io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb)
{
    int sts;
    unsigned int idx;
    volatile void *data;
    int nbytes;
    sc_err_t status = SC_ERR_NONE;
    imx_irq_event_t *cur_irq_event, *new_irq_event, *prev_irq_event;
    uint32_t irq_enable[SC_IRQ_NUM_GROUP];
    union _input_data {
        /* PM API */
        imx_dcmd_sc_pm_sys_t pm_sys;
        imx_dcmd_sc_pm_res_t pm_res;
        imx_dcmd_sc_pm_req_cpu_lpm_t pm_req_cpu_lpm;
        imx_dcmd_sc_pm_cpu_resume_addr_t pm_resume_addr;
        imx_dcmd_sc_pm_cpu_resume_t pm_cpu_resume;
        imx_dcmd_sc_pm_req_sys_if_power_mode_t pm_req_sys_if_pm;
        imx_dcmd_sc_pm_clock_rate_t pm_clock_rate;
        imx_dcmd_sc_pm_clock_en_t pm_clock_en;
        imx_dcmd_sc_pm_clk_parent_t pm_clock_parrent;
        imx_dcmd_sc_pm_boot_t pm_boot;
        imx_dcmd_sc_pm_reboot_partition_t reboot_partition;
        sc_pm_reset_type_t pm_reset_type;
        sc_pm_reset_reason_t pm_reset_reason;
        imx_dcmd_sc_pm_cpu_t pm_cpu;
        imx_dcmd_sc_pm_cpu_reset_t pm_cpu_reset;
        imx_dcmd_sc_pm_res_all_t pm_res_all;
        imx_dcmd_sc_pm_set_boot_param_t pm_set_boot_param;
        /* IRQ API */
        imx_dcmd_sc_irq_en_t irq_en;
        imx_dcmd_sc_irq_stat_t irq_stat;
        /* MISC API */
        imx_dcmd_sc_misc_control_t misc_ctrl;
        imx_dcmd_sc_misc_ari_t misc_ari;
        imx_dcmd_sc_misc_max_dma_group_t max_dma_group;
        imx_dcmd_sc_misc_dma_group_t set_dma_group;
        imx_dcmd_sc_misc_dma_group_t dma_group;
        imx_dcmd_sc_misc_otp_fuse_t otp_fuse;
        imx_dcmd_sc_misc_temp_t temp;
        imx_dcmd_sc_misc_build_info_t build_info;
        imx_dcmd_sc_misc_unique_id_t unique_id;
        imx_dcmd_sc_misc_api_ver_t api_ver;
        imx_dcmd_sc_misc_board_ioctl_t board_ioctl;
        sc_misc_boot_status_t boot_stat;
        sci_mgr_rsrc_t cpu_done_booting;
        sci_mgr_rsrc_t dev;
        sci_mgr_bool_t status;
        sc_faddr_t faddr;
        uint32_t lifecycle;
        /* PAD API */
        imx_dcmd_sc_pad_mux_t pad_mux;
        imx_dcmd_sc_pad_gp_t gp;
        imx_dcmd_sc_pad_gp_28fdsoi_hsic_t fdsoi_hsic;
        imx_dcmd_sc_pad_wakeup_t wakeup;
        imx_dcmd_sc_pad_all_t all;
        imx_dcmd_sc_pad_val_t pad_val;
        imx_dcmd_sc_pad_gp_fdsoi_t fdsoi;
        imx_dcmd_sc_pad_gp_fdsoi_comp_t fdsoi_comp;
        /* RM API */
        imx_dcmd_sc_rm_partition_alloc_t rm_alloc;
        imx_dcmd_sc_rm_set_confidential_t rm_confidential;
        sci_mgr_rm_pt_t rm_pt;
        sc_rm_did_t   did;
        imx_dcmd_sc_rm_partition_static_t rm_static;
        imx_dcmd_sc_rm_parent_t rm_parent;
        imx_dcmd_sc_rm_all_t rm_all;
        imx_dcmd_sc_rm_resource_t rm_resource;
        imx_dcmd_sc_rm_resource_mv_t rm_mv;
        imx_dcmd_sc_rm_subsys_rsrc_mv_t rm_subsys_rsrc_mv;
        imx_dcmd_sc_rm_resource_attr_t rm_attr;
        imx_dcmd_sc_rm_master_sid_t rm_sid;
        imx_dcmd_sc_rm_periph_perm_t rm_periph_perm;
        sci_mgr_rsrc_t resource;
        sc_rm_mr_t rm_mr;
        imx_dcmd_sc_rm_memreg_alloc_t rm_memreg_alloc;
        imx_dcmd_sc_rm_memreg_split_t rm_memreg_split;
        imx_dcmd_sc_rm_find_memreg_t rm_find_memreg;
        imx_dcmd_sc_rm_memreg_t rm_memreg;
        imx_dcmd_sc_rm_memreg_perm_t rm_memreg_perm;
        imx_dcmd_sc_rm_pad_t rm_pad;
        imx_dcmd_sc_rm_memreg_frag_t rm_memreg_frag;
        sc_pad_t rm_sc_pad;
        /* TIMER API */
        sc_timer_wdog_time_t wdog_time;
        sc_bool_t wdog_lock;
        imx_dcmd_sc_timer_status_t timer_status;
        imx_dcmd_sc_timer_pt_status_t timer_pt_status;
        imx_dcmd_sc_timer_wdog_action_t wdog_action;
        imx_dcmd_sc_timer_time_t timer_time;
        uint32_t sec;
        sci_mgr_bool_t enable;
        uint8_t count;
        imx_dcmd_sc_event_t sc_event;
        imx_dcmd_sc_event_status_t sc_event_status;
        imx_dcmd_sc_event_enable_t sc_event_enable;
        uint64_t ticks;
        /* SECO API */
        imx_dcmd_sc_seco_image_load_t seco_image_load;
        imx_dcmd_sc_seco_authenticate_t seco_authenticate;
        imx_dcmd_sc_seco_enh_authenticate_t seco_enh_authenticate;
        uint32_t seco_change;
        uint32_t seco_commit;
        uint32_t seco_mode;
        uint64_t seco_nonce;
        imx_dcmd_sc_seco_gen_key_blob_t seco_gen_key_blob;
        imx_dcmd_sc_seco_load_key_t seco_load_key;
        imx_dcmd_sc_seco_get_mp_key_t seco_get_mp_key;
        imx_dcmd_sc_seco_update_mpmr_t seco_update_mpmr;
        imx_dcmd_sc_seco_get_mp_sign_t seco_get_mp_sign;
        imx_dcmd_sc_seco_build_info_t seco_build_info;
        imx_dcmd_sc_seco_chip_info_t seco_chip_info;
        imx_dcmd_sc_seco_get_event_t seco_get_event;
        sci_mgr_seco_rng_stat_t seco_rng_stat;
        imx_dcmd_sc_seco_secvio_config_t seco_secvio_config;
        imx_dcmd_sc_seco_secvio_dgo_config_t seco_secvio_dgo_config;
    }* input_data;

    if ((sts = iofunc_devctl_default(ctp, msg, ocb)) != _RESMGR_DEFAULT) {
        return (sts);
    }

    /* 1) assign a pointer to the data area of the message */
    data = _DEVCTL_DATA(msg->i);
    input_data = (union _input_data *) data;
    /* 2) preset the number of bytes that we'll return to zero */
    nbytes = 0;
    /* check for all commands; we'll just show the ones we're
     * interested in here
     */
    if (sci->optv) {
        IMX_SCI_DEBUG("DCMD %i", msg->i.dcmd & 0xff);
    }

    (void)pthread_mutex_lock(&sci->hw_lock);

    switch (msg->i.dcmd) {
        /* Calls sc_open() function */
        case IMX_DCMD_SC_OPEN:
            IMX_SCI_DEBUG("IMX_DCMD_SC_OPEN is deprecated, sc_ipc_open() function is called during start of SC driver.");
            break;
        /* Calls sc_close() function */
        case IMX_DCMD_SC_CLOSE:
            IMX_SCI_DEBUG("IMX_DCMD_SC_OPEN is deprecated, sc_ipc_close() function is called during start of SC driver.");
            break;
        /* ------------------------------------------------- PM API ------------------------------------------------ */
        /* Calls sc_pm_set_sys_power_mode() function */
        case IMX_DCMD_SC_PM_SET_SYS_POWER_MODE:
            status = sc_pm_set_sys_power_mode(sci->id, input_data->pm_sys.mode);
            break;
        /* Calls sc_pm_set_partition_power_mode() function */
        case IMX_DCMD_SC_PM_SET_PARTITION_POWER_MODE:
            status = sc_pm_set_partition_power_mode(sci->id, input_data->pm_sys.pt, input_data->pm_sys.mode);
            break;
        /* Calls sc_pm_get_sys_power_mode() function */
        case IMX_DCMD_SC_PM_GET_SYS_POWER_MODE:
            status = sc_pm_get_sys_power_mode(sci->id, input_data->pm_sys.pt,
                                              &input_data->pm_sys.mode);
            nbytes = sizeof(imx_dcmd_sc_pm_sys_t);
            break;
        /* Calls sc_pm_set_resource_power_mode() function */
        case IMX_DCMD_SC_PM_SET_RESOURCE_POWER_MODE:
            status = sc_pm_set_resource_power_mode(sci->id,
                                                   (sc_rsrc_t)input_data->pm_res.resource, input_data->pm_res.mode);
            break;
        /* Calls sc_pm_get_resource_power_mode() function */
        case IMX_DCMD_SC_PM_GET_RESOURCE_POWER_MODE:
            status = sc_pm_get_resource_power_mode(sci->id,
                                                   (sc_rsrc_t)input_data->pm_res.resource, &input_data->pm_res.mode);
            nbytes = sizeof(imx_dcmd_sc_pm_res_t);
            break;
        /* Calls sc_pm_req_low_power_mode() function */
        case IMX_DCMD_SC_PM_REQ_LOW_POWER_MODE:
            status = sc_pm_req_low_power_mode(sci->id,
                                              (sc_rsrc_t)input_data->pm_res.resource, input_data->pm_res.mode);
            break;
        /* Calls sc_pm_req_cpu_low_power_mode() function */
        case IMX_DCMD_SC_PM_REQ_CPU_LOW_POWER_MODE:
            status = sc_pm_req_cpu_low_power_mode(sci->id,
                                                  (sc_rsrc_t)input_data->pm_req_cpu_lpm.resource,
                                                  input_data->pm_req_cpu_lpm.mode,
                                                  input_data->pm_req_cpu_lpm.wake_src);
            break;
        /* Calls sc_pm_set_cpu_resume_addr() function */
        case IMX_DCMD_SC_PM_SET_CPU_RESUME_ADDR:
            status = sc_pm_set_cpu_resume_addr(sci->id,
                                               (sc_rsrc_t)input_data->pm_resume_addr.resource,
                                               input_data->pm_resume_addr.address);
            break;
        /* Calls sc_pm_set_cpu_resume() function */
        case IMX_DCMD_SC_PM_SET_CPU_RESUME:
            status = sc_pm_set_cpu_resume(sci->id,
                                          (sc_rsrc_t)input_data->pm_cpu_resume.resource,
                                          (sc_bool_t)input_data->pm_cpu_resume.isPrimary,
                                          input_data->pm_cpu_resume.address);
            break;
        /* Calls sc_pm_req_sys_if_power_mode() function */
        case IMX_DCMD_SC_PM_REQ_SYS_IF_POWER_MODE:
            status = sc_pm_req_sys_if_power_mode(sci->id,
                                                 (sc_rsrc_t)input_data->pm_req_sys_if_pm.resource,
                                                 input_data->pm_req_sys_if_pm.sys_if,
                                                 input_data->pm_req_sys_if_pm.hpm,
                                                 input_data->pm_req_sys_if_pm.lpm);
            break;
        /* Calls sc_pm_set_clock_rate() function */
        case IMX_DCMD_SC_PM_SET_CLOCK_RATE:
            status = sc_pm_set_clock_rate(sci->id,
                                          (sc_rsrc_t)input_data->pm_clock_rate.resource,
                                          input_data->pm_clock_rate.clk,
                                          &input_data->pm_clock_rate.rate);
            nbytes = sizeof(imx_dcmd_sc_pm_clock_rate_t);
            break;
        /* Calls sc_pm_get_clock_rate() function */
        case IMX_DCMD_SC_PM_GET_CLOCK_RATE:
            status = sc_pm_get_clock_rate(sci->id,
                                          (sc_rsrc_t)input_data->pm_clock_rate.resource,
                                          input_data->pm_clock_rate.clk, &input_data->pm_clock_rate.rate);
            nbytes = sizeof(imx_dcmd_sc_pm_clock_rate_t);
            break;
        /* Calls sc_pm_clock_enable() function */
        case IMX_DCMD_SC_PM_CLOCK_ENABLE:
            status = sc_pm_clock_enable(sci->id,
                                        (sc_rsrc_t)input_data->pm_clock_en.resource,
                                        input_data->pm_clock_en.clk,
                                        (sc_bool_t)input_data->pm_clock_en.enable,
                                        (sc_bool_t)input_data->pm_clock_en.autog);
            break;
        /* Calls sc_pm_set_clock_parent() function */
        case IMX_DCMD_SC_PM_SET_CLOCK_PARENT :
            status = sc_pm_set_clock_parent(sci->id,
                                            (sc_rsrc_t)input_data->pm_clock_parrent.resource,
                                            input_data->pm_clock_parrent.clk,
                                            input_data->pm_clock_parrent.parent);
            break;
        /* Calls sc_pm_get_clock_parent() function */
        case IMX_DCMD_SC_PM_GET_CLOCK_PARENT :
            status = sc_pm_get_clock_parent(sci->id,
                                            (sc_rsrc_t)input_data->pm_clock_parrent.resource,
                                            input_data->pm_clock_parrent.clk,
                                            &input_data->pm_clock_parrent.parent);
            nbytes = sizeof(imx_dcmd_sc_pm_clk_parent_t);
            break;
        /* Calls sc_pm_boot() function */
        case IMX_DCMD_SC_PM_BOOT:
            status = sc_pm_boot(sci->id, input_data->pm_boot.pt,
                                (sc_rsrc_t)input_data->pm_boot.boot_cpu,
                                input_data->pm_boot.boot_addr,
                                (sc_rsrc_t)input_data->pm_boot.boot_mu,
                                (sc_rsrc_t)input_data->pm_boot.dev);
            break;
        /* Calls sc_pm_cpu_start() function */
        case IMX_DCMD_SC_PM_CPU_START:
            status = sc_pm_cpu_start(sci->id,
                                     (sc_rsrc_t)input_data->pm_cpu.resource,
                                     (sc_bool_t)input_data->pm_cpu.enable,
                                     input_data->pm_cpu.boot_addr);
            break;
        /* Calls sc_pm_reboot() function */
        case IMX_DCMD_SC_PM_REBOOT:
            sc_pm_reboot(sci->id, input_data->pm_reset_type);
            break;
        /* Calls sc_pm_reboot_partition() function */
        case IMX_DCMD_SC_PM_REBOOT_PARTITION:
            status = sc_pm_reboot_partition(sci->id, input_data->reboot_partition.pt, input_data->reboot_partition.type);
            break;
        /* Calls sc_pm_reset() function */
        case IMX_DCMD_SC_PM_RESET:
            status = sc_pm_reset(sci->id, input_data->pm_reset_type);
            break;
        /* Calls sc_pm_reset_reason() function */
        case IMX_DCMD_SC_PM_RESET_REASON:
            status = sc_pm_reset_reason(sci->id, &input_data->pm_reset_reason);
            nbytes = sizeof(sc_pm_reset_reason_t);
            break;
        /* Calls sc_pm_set_resource_power_mode_all() function */
        case IMX_DCMD_SC_PM_SET_RESOURCE_POWER_MODE_ALL:
            status = sc_pm_set_resource_power_mode_all(sci->id, input_data->pm_res_all.pt,
                                                       input_data->pm_res_all.mode, input_data->pm_res_all.exclude);
            break;
        /* Calls sc_pm_cpu_reset() function */
        case IMX_DCMD_SC_PM_CPU_RESET:
            sc_pm_cpu_reset(sci->id,
                           (sc_rsrc_t)input_data->pm_cpu_reset.resource,
                           input_data->pm_cpu_reset.address);
            break;
        /* Calls sc_pm_get_reset_part() function */
        case IMX_DCMD_SC_PM_GET_RESET_PART:
            status = sc_pm_get_reset_part(sci->id, (sc_rm_pt_t *)&input_data->rm_pt);
            nbytes = sizeof(sc_rm_pt_t);
            break;
        /* Calls sc_pm_set_boot_parm() function */
        case IMX_DCMD_SC_PM_SET_BOOT_PARAM:
            status = sc_pm_set_boot_parm(sci->id, input_data->pm_set_boot_param.resource_cpu,
                                         input_data->pm_set_boot_param.boot_addr,
                                         input_data->pm_set_boot_param.resource_mu,
                                         input_data->pm_set_boot_param.resource_dev);
            break;
        /* Calls sc_pm_reboot_continue() function */
        case IMX_DCMD_SC_PM_REBOOT_CONTINUE:
            status = sc_pm_reboot_continue(sci->id, input_data->rm_pt);
            break;
        /* Calls sc_pm_is_partition_started() function */
        case IMX_DCMD_SC_PM_IS_PARTITION_STARTED:
            status = (sc_err_t)sc_pm_is_partition_started(sci->id, input_data->rm_pt);
            break;
        /* ------------------------------------------------- IRQ API ----------------------------------------------- */
        /* Calls sc_irq_enable() function */
        case IMX_DCMD_SC_IRQ_ENABLE:
            status = sc_irq_enable(sci->id,
                                   (sc_rsrc_t)input_data->irq_en.resource,
                                   input_data->irq_en.group, input_data->irq_en.mask,
                                   (sc_bool_t)input_data->irq_en.enable);
            break;
        /* Calls sc_irq_status() function */
        case IMX_DCMD_SC_IRQ_STATUS:
            status = sc_irq_status(sci->id, (sc_rsrc_t)input_data->irq_stat.resource,
                                   input_data->irq_stat.group, &input_data->irq_stat.status);
            nbytes = sizeof(imx_dcmd_sc_irq_stat_t);
            break;
        /* ------------------------------------------------ MISC API ----------------------------------------------- */
        /* Calls sc_misc_set_control() function */
        case IMX_DCMD_SC_MISC_SET_CONTROL:
            status = sc_misc_set_control(sci->id, (sc_rsrc_t)input_data->misc_ctrl.resource,
                                         (sc_ctrl_t)input_data->misc_ctrl.ctrl,
                                         input_data->misc_ctrl.val);
            break;
        /* Calls sc_misc_get_control() function */
        case IMX_DCMD_SC_MISC_GET_CONTROL:
            status = sc_misc_get_control(sci->id, (sc_rsrc_t)input_data->misc_ctrl.resource,
                                         (sc_ctrl_t)input_data->misc_ctrl.ctrl,
                                         &input_data->misc_ctrl.val);
            nbytes = sizeof(imx_dcmd_sc_misc_control_t);
            break;
        /* Calls sc_misc_set_ari() function */
        case IMX_DCMD_SC_MISC_SET_ARI:
            status = sc_misc_set_ari(sci->id, (sc_rsrc_t)input_data->misc_ari.resource,
                                     (sc_rsrc_t)input_data->misc_ari.master, input_data->misc_ari.ari,
                                     (sc_bool_t)input_data->misc_ari.enable);
            break;
        /* Calls sc_misc_set_max_dma_group() function */
        case IMX_DCMD_SC_MISC_SET_MAX_DMA_GROUP:
            status = sc_misc_set_max_dma_group(sci->id, input_data->max_dma_group.pt, input_data->max_dma_group.max);
            break;
        /* Calls sc_misc_set_dma_group() function */
        case IMX_DCMD_SC_MISC_SET_DMA_GROUP:
            status = sc_misc_set_dma_group(sci->id, (sc_rsrc_t)input_data->set_dma_group.resource,
                                           (sc_misc_dma_group_t)input_data->set_dma_group.group);
            break;
        /* Calls sc_misc_boot_status() function */
        case IMX_DCMD_SC_MISC_BOOT_STATUS:
            sc_misc_boot_status(sci->id, input_data->boot_stat);
            break;
        /* Calls sc_misc_boot_done() function */
        case IMX_DCMD_SC_MISC_BOOT_DONE:
            status = sc_misc_boot_done(sci->id, (sc_rsrc_t)input_data->cpu_done_booting);
            break;
        /* Calls sc_misc_otp_fuse_read() function */
        case IMX_DCMD_SC_MISC_OTP_FUSE_READ:
            status = sc_misc_otp_fuse_read(sci->id, input_data->otp_fuse.word, &input_data->otp_fuse.val);
            nbytes = sizeof(imx_dcmd_sc_misc_otp_fuse_t);
            break;
        /* Calls sc_misc_otp_fuse_write() function */
        case IMX_DCMD_SC_MISC_OTP_FUSE_WRITE:
            status = sc_misc_otp_fuse_write(sci->id, input_data->otp_fuse.word, input_data->otp_fuse.val);
            break;
        /* Calls sc_misc_set_temp() function */
        case IMX_DCMD_SC_MISC_SET_TEMP:
            status = sc_misc_set_temp(sci->id, (sc_rsrc_t)input_data->temp.resource, input_data->temp.temp,
                                      input_data->temp.celsius, input_data->temp.tenths);
            break;
        /* Calls sc_misc_get_temp() function */
        case IMX_DCMD_SC_MISC_GET_TEMP:
            status = sc_misc_get_temp(sci->id, (sc_rsrc_t)input_data->temp.resource, input_data->temp.temp,
                                      &input_data->temp.celsius, &input_data->temp.tenths);
            nbytes = sizeof(imx_dcmd_sc_misc_temp_t);
            break;
        /* Calls sc_misc_build_info() function */
        case IMX_DCMD_SC_MISC_BUILD_INFO:
            sc_misc_build_info(sci->id, &input_data->build_info.build, &input_data->build_info.commit);
            nbytes = sizeof(imx_dcmd_sc_misc_build_info_t);
            break;
        /* Calls sc_misc_get_boot_dev() function */
        case IMX_DCMD_SC_MISC_GET_BOOT_DEV:
            sc_misc_get_boot_dev(sci->id, (sc_rsrc_t *)&input_data->dev);
            nbytes = sizeof(sc_rsrc_t);
            break;
        /* Calls sc_misc_get_button_status() function */
        case IMX_DCMD_SC_MISC_GET_BUTTON_STATUS:
            sc_misc_get_button_status(sci->id, (sc_bool_t *)&input_data->status);
            nbytes = sizeof(sc_bool_t);
            break;
        /* Calls sc_misc_unique_id() function */
        case IMX_DCMD_SC_MISC_UNIQUE_ID:
            sc_misc_unique_id(sci->id, &input_data->unique_id.id_l, &input_data->unique_id.id_h);
            nbytes = sizeof(imx_dcmd_sc_misc_unique_id_t);
            break;
        /* Calls sc_misc_api_ver() function */
        case IMX_DCMD_SC_MISC_API_VER:
            sc_misc_api_ver(sci->id, &input_data->api_ver.cl_maj, &input_data->api_ver.cl_min, &input_data->api_ver.sv_maj, &input_data->api_ver.sv_min);
            nbytes = sizeof(imx_dcmd_sc_misc_api_ver_t);
            break;
        /* Calls sc_misc_board_ioctl() function */
        case IMX_DCMD_SC_MISC_BOARD_IOCTL:
            status = sc_misc_board_ioctl(sci->id, &input_data->board_ioctl.parm1, &input_data->board_ioctl.parm2, &input_data->board_ioctl.parm3);
            nbytes = sizeof(imx_dcmd_sc_misc_board_ioctl_t);
            break;
        /* ------------------------------------------------- PAD API ----------------------------------------------- */
        /* Calls sc_pad_set_mux() function */
        case IMX_DCMD_SC_PAD_SET_MUX:
            status = sc_pad_set_mux(sci->id, input_data->pad_mux.pad,
                                    input_data->pad_mux.mux, input_data->pad_mux.config,
                                    input_data->pad_mux.iso);
            break;
        /* Calls sc_pad_set_gp() function */
        case IMX_DCMD_SC_PAD_SET_GP:
            status = sc_pad_set_gp(sci->id, input_data->gp.pad,
                                   input_data->gp.ctrl);
            break;
        /* Calls sc_pad_set_gp_28fdsoi() function */
        case IMX_DCMD_SC_PAD_SET_GP_28FDSOI:
            status = sc_pad_set_gp_28fdsoi(sci->id, input_data->fdsoi.pad, input_data->fdsoi.dse, input_data->fdsoi.ps);
            break;
        /* Calls sc_pad_set_wakeup() function */
        case IMX_DCMD_SC_PAD_SET_WAKEUP:
            status = sc_pad_set_wakeup(sci->id, input_data->wakeup.pad,
                                       input_data->wakeup.wakeup);
            break;
        /* Calls sc_pad_set_all() function */
        case IMX_DCMD_SC_PAD_SET_ALL:
            status = sc_pad_set_all(sci->id, input_data->all.pad,
                                    input_data->all.mux, input_data->all.config,
                                    input_data->all.iso, input_data->all.ctrl,
                                    input_data->all.wakeup);
            break;
        /* Calls sc_pad_get_mux() function */
        case IMX_DCMD_SC_PAD_GET_MUX:
            status = sc_pad_get_mux(sci->id, input_data->pad_mux.pad,
                                    &input_data->pad_mux.mux, &input_data->pad_mux.config,
                                    &input_data->pad_mux.iso);
            nbytes = sizeof(imx_dcmd_sc_pad_mux_t);
            break;
        /* Calls sc_pad_get_gp() function */
        case IMX_DCMD_SC_PAD_GET_GP:
            status = sc_pad_get_gp(sci->id, input_data->gp.pad,
                                   &input_data->gp.ctrl);
            nbytes = sizeof(imx_dcmd_sc_pad_gp_t);
            break;
        /* Calls sc_pad_get_gp_28fdsoi_hsic() function */
        case IMX_DCMD_SC_PAD_GET_GP_28FDSOI_HSIC:
            status = sc_pad_get_gp_28fdsoi_hsic(sci->id, input_data->fdsoi_hsic.pad,
                                                &input_data->fdsoi_hsic.dse, (sc_bool_t *)&input_data->fdsoi_hsic.hys,
                                                &input_data->fdsoi_hsic.pus, (sc_bool_t *)&input_data->fdsoi_hsic.pke,
                                                (sc_bool_t *)&input_data->fdsoi_hsic.pue);
            nbytes = sizeof(imx_dcmd_sc_pad_gp_28fdsoi_hsic_t);
            break;
        /* Calls sc_pad_get_wakeup() function */
        case IMX_DCMD_SC_PAD_GET_WAKEUP:
            status = sc_pad_get_wakeup(sci->id, input_data->wakeup.pad,
                                       &input_data->wakeup.wakeup);
            nbytes = sizeof(imx_dcmd_sc_pad_wakeup_t);
            break;
        /* Calls sc_pad_get_all() function */
        case IMX_DCMD_SC_PAD_GET_ALL:
            status = sc_pad_get_all(sci->id, input_data->all.pad,
                                    &input_data->all.mux, &input_data->all.config,
                                    &input_data->all.iso, &input_data->all.ctrl,
                                    &input_data->all.wakeup);
            nbytes = sizeof(imx_dcmd_sc_pad_all_t);
            break;
        /* Calls sc_pad_set() function */
        case IMX_DCMD_SC_PAD_SET:
            status = sc_pad_set(sci->id, input_data->pad_val.pad, input_data->pad_val.val);
            break;
        /* Calls sc_pad_get() function */
        case IMX_DCMD_SC_PAD_GET:
            status = sc_pad_get(sci->id, input_data->pad_val.pad, &input_data->pad_val.val);
            nbytes = sizeof(imx_dcmd_sc_pad_val_t);
            break;
        /* Calls sc_pad_set_gp_28fdsoi_hsic() function */
        case IMX_DCMD_SC_PAD_SET_GP_28FDSOI_HSIC:
            status = sc_pad_set_gp_28fdsoi_hsic(sci->id, input_data->fdsoi_hsic.pad,
                                                input_data->fdsoi_hsic.dse, (sc_bool_t)input_data->fdsoi_hsic.hys,
                                                input_data->fdsoi_hsic.pus, (sc_bool_t)input_data->fdsoi_hsic.pke,
                                                (sc_bool_t)input_data->fdsoi_hsic.pue);
            break;
        /* Calls sc_pad_get_gp_28fdsoi() function */
        case IMX_DCMD_SC_PAD_GET_GP_28FDSOI:
            status = sc_pad_get_gp_28fdsoi(sci->id, input_data->fdsoi.pad, &input_data->fdsoi.dse, &input_data->fdsoi.ps);
            nbytes = sizeof(imx_dcmd_sc_pad_gp_fdsoi_t);
            break;
        /* Calls sc_pad_set_gp_28fdsoi_comp() function */
        case IMX_DCMD_SC_PAD_SET_GP_28FDSOI_COMP:
            status = sc_pad_set_gp_28fdsoi_comp(sci->id, input_data->fdsoi_comp.pad, input_data->fdsoi_comp.compen,
                                                (sc_bool_t)input_data->fdsoi_comp.fastfrz, input_data->fdsoi_comp.rasrcp,
                                                input_data->fdsoi_comp.rasrcn,
                                                (sc_bool_t)input_data->fdsoi_comp.nasrc_sel,
                                                (sc_bool_t)input_data->fdsoi_comp.psw_ovr);
            break;
        /* Calls sc_pad_get_gp_28fdsoi_comp() function */
        case IMX_DCMD_SC_PAD_GET_GP_28FDSOI_COMP:
            status = sc_pad_get_gp_28fdsoi_comp(sci->id, input_data->fdsoi_comp.pad, &input_data->fdsoi_comp.compen,
                                                (sc_bool_t *)&input_data->fdsoi_comp.fastfrz, &input_data->fdsoi_comp.rasrcp,
                                                &input_data->fdsoi_comp.rasrcn, (sc_bool_t *)&input_data->fdsoi_comp.nasrc_sel,
                                                (sc_bool_t *)&input_data->fdsoi_comp.compok, &input_data->fdsoi_comp.nasrc,
                                                (sc_bool_t *)&input_data->fdsoi_comp.psw_ovr);
            nbytes = sizeof(imx_dcmd_sc_pad_gp_fdsoi_comp_t);
            break;
        /* ------------------------------------------------- RM API ------------------------------------------------ */
        /* Calls sc_rm_partition_alloc() function */
        case IMX_DCMD_SC_RM_PARTITION_ALLOC:
            status = sc_rm_partition_alloc(sci->id, &input_data->rm_alloc.pt,
                                           (sc_bool_t)input_data->rm_alloc.secure,
                                           (sc_bool_t)input_data->rm_alloc.isolated,
                                           (sc_bool_t)input_data->rm_alloc.restricted,
                                           (sc_bool_t)input_data->rm_alloc.grant,
                                           (sc_bool_t)input_data->rm_alloc.coherent);
            nbytes = sizeof(imx_dcmd_sc_rm_partition_alloc_t);
            break;
        /* Calls sc_rm_set_confidential() function */
        case IMX_DCMD_SC_RM_SET_CONFIDENTIAL:
            status = sc_rm_set_confidential(sci->id, input_data->rm_confidential.pt, (sc_bool_t)input_data->rm_confidential.retro);
            break;
        /* Calls sc_rm_partition_free() function */
        case IMX_DCMD_SC_RM_PARTITION_FREE:
            status = sc_rm_partition_free(sci->id, input_data->rm_pt);
            break;
        /* Calls sc_rm_get_did() function */
        case IMX_DCMD_SC_RM_GET_DID:
            input_data->did = sc_rm_get_did(sci->id);
            nbytes = sizeof(input_data->did);
            break;
        /* Calls sc_rm_partition_static() function */
        case IMX_DCMD_SC_RM_PARTITION_STATIC:
            status = sc_rm_partition_static(sci->id, input_data->rm_static.pt,
                                            input_data->rm_static.did);
            break;
        /* Calls sc_rm_partition_lock() function */
        case IMX_DCMD_SC_RM_PARTITION_LOCK:
            status = sc_rm_partition_lock(sci->id, input_data->rm_pt);
            break;
        /* Calls sc_rm_get_partition() function */
        case IMX_DCMD_SC_RM_GET_PARTITION:
            status = sc_rm_get_partition(sci->id, &input_data->rm_pt);
            nbytes = sizeof(sc_rm_pt_t);
            break;
        /* Calls sc_rm_set_parent() function */
        case IMX_DCMD_SC_RM_SET_PARENT:
            status = sc_rm_set_parent(sci->id, input_data->rm_parent.pt,
                                      input_data->rm_parent.parent);
            break;
        /* Calls sc_rm_move_all() function */
        case IMX_DCMD_SC_RM_MOVE_ALL:
            status = sc_rm_move_all(sci->id, input_data->rm_all.pt_src,
                                    input_data->rm_all.pt_dst, input_data->rm_all.move_rsrc,
                                    input_data->rm_all.move_pins);
            break;
        /* Calls sc_rm_assign_resource() function */
        case IMX_DCMD_SC_RM_ASSIGN_RESOURCE:
            status = sc_rm_assign_resource(sci->id, input_data->rm_resource.pt,
                                           (sc_rsrc_t)input_data->rm_resource.resource);
            break;
        /* Calls sc_rm_set_resource_movable() function */
        case IMX_DCMD_SC_RM_SET_RESOURCE_MOVABLE :
            status = sc_rm_set_resource_movable(sci->id, (sc_rsrc_t)input_data->rm_mv.resource_fst,
                                                (sc_rsrc_t)input_data->rm_mv.resource_lst,
                                                (sc_bool_t)input_data->rm_mv.movable);
            break;

        /* Calls sc_rm_set_subsys_rsrc_movable() function */
        case IMX_DCMD_SC_RM_SET_SUBSYS_RSRC_MOVABLE :
            status = sc_rm_set_subsys_rsrc_movable(sci->id, (sc_rsrc_t)input_data->rm_subsys_rsrc_mv.resource,
                                                   (sc_bool_t)input_data->rm_subsys_rsrc_mv.movable);
            break;
        /* Calls sc_rm_set_master_attributes() function */
        case IMX_DCMD_SC_RM_SET_MASTER_ATTRIBUTES:
            status = sc_rm_set_master_attributes(sci->id,
                                                 (sc_rsrc_t)input_data->rm_attr.resource, input_data->rm_attr.sa,
                                                 input_data->rm_attr.pa,
                                                 (sc_bool_t)input_data->rm_attr.smmu_bypass);
            break;
        /* Calls sc_rm_set_master_sid() function */
        case IMX_DCMD_SC_RM_SET_MASTER_SID:
            status = sc_rm_set_master_sid(sci->id, (sc_rsrc_t)input_data->rm_sid.resource,
                                          input_data->rm_sid.sid);
            break;
        /* Calls sc_rm_set_peripheral_permissions() function */
        case IMX_DCMD_SC_RM_SET_PERIPHERAL_PERMISSIONS:
            status = sc_rm_set_peripheral_permissions(sci->id,
                                                      (sc_rsrc_t)input_data->rm_periph_perm.resource,
                                                      input_data->rm_periph_perm.pt, input_data->rm_periph_perm.perm);
            break;
        /* Calls sc_rm_is_resource_owned() function */
        case IMX_DCMD_SC_RM_IS_RESOURCE_OWNED:
            status = (sc_err_t)sc_rm_is_resource_owned(sci->id, (sc_rsrc_t)input_data->resource);
            break;
        /* Calls sc_rm_is_resource_master() function */
        case IMX_DCMD_SC_RM_IS_RESOURCE_MASTER:
            status = (sc_err_t)sc_rm_is_resource_master(sci->id, (sc_rsrc_t)input_data->resource);
            break;
        /* Calls sc_rm_is_resource_peripheral() function */
        case IMX_DCMD_SC_RM_IS_RESOURCE_PERIPHERAL:
            status = (sc_err_t)sc_rm_is_resource_peripheral(sci->id, (sc_rsrc_t)input_data->resource);
            break;
        /* Calls sc_rm_get_resource_info() function */
        case IMX_DCMD_SC_RM_GET_RESOURCE_INFO:
            status = sc_rm_get_resource_info(sci->id, (sc_rsrc_t)input_data->rm_sid.resource,
                                             &input_data->rm_sid.sid);
            nbytes = sizeof(imx_dcmd_sc_rm_master_sid_t);
            break;
        /* Calls sc_rm_memreg_alloc() function */
        case IMX_DCMD_SC_RM_MEMREG_ALLOC:
            status = sc_rm_memreg_alloc(sci->id, &input_data->rm_memreg_alloc.mr,
                                        input_data->rm_memreg_alloc.start,
                                        input_data->rm_memreg_alloc.end);
            nbytes = sizeof(imx_dcmd_sc_rm_memreg_alloc_t);
            break;
        /* Calls sc_rm_memreg_split() function */
        case IMX_DCMD_SC_RM_MEMREG_SPLIT:
            status = sc_rm_memreg_split(sci->id, input_data->rm_memreg_split.mr, &input_data->rm_memreg_split.mr_ret,
                                        input_data->rm_memreg_split.addr_start, input_data->rm_memreg_split.addr_end);
            nbytes = sizeof(imx_dcmd_sc_rm_memreg_split_t);
            break;
        /* Calls sc_rm_memreg_free() function */
        case IMX_DCMD_SC_RM_MEMREG_FREE:
            status = sc_rm_memreg_free(sci->id, input_data->rm_mr);
            break;
        /* Calls sc_rm_find_memreg() function */
        case IMX_DCMD_SC_RM_FIND_MEMREG:
            status = sc_rm_find_memreg(sci->id, &input_data->rm_find_memreg.mr,
                                       input_data->rm_find_memreg.addr_start,
                                       input_data->rm_find_memreg.addr_end);
            nbytes = sizeof(imx_dcmd_sc_rm_find_memreg_t);
            break;
        /* Calls sc_rm_assign_memreg() function */
        case IMX_DCMD_SC_RM_ASSIGN_MEMREG:
            status = sc_rm_assign_memreg(sci->id, input_data->rm_memreg.pt,
                                         input_data->rm_memreg.mr);
            break;
        /* Calls sc_rm_set_memreg_permissions() function */
        case IMX_DCMD_SC_RM_SET_MEMREG_PERMISSIONS:
            status = sc_rm_set_memreg_permissions(sci->id,
                                                  input_data->rm_memreg_perm.mr, input_data->rm_memreg_perm.pt,
                                                  input_data->rm_memreg_perm.perm);
            break;
        /* Calls sc_rm_is_memreg_owned() function */
        case IMX_DCMD_SC_RM_IS_MEMREG_OWNED:
            status = (sc_err_t)sc_rm_is_memreg_owned(sci->id, input_data->rm_mr);
            break;
        /* Calls sc_rm_get_memreg_info() function */
        case IMX_DCMD_SC_RM_GET_MEMREG_INFO:
            status = sc_rm_get_memreg_info(sci->id, input_data->rm_memreg_alloc.mr,
                                           &input_data->rm_memreg_alloc.start,
                                           &input_data->rm_memreg_alloc.end);
            nbytes = sizeof(imx_dcmd_sc_rm_memreg_alloc_t);
            break;
        /* Calls sc_rm_assign_pad() function */
        case IMX_DCMD_SC_RM_ASSIGN_PAD:
            status = sc_rm_assign_pad(sci->id, input_data->rm_pad.pt,
                                      input_data->rm_pad.pad);
            break;
        /* Calls sc_rm_is_pad_owned() function */
        case IMX_DCMD_SC_RM_IS_PAD_OWNED:
            status = (sc_err_t)sc_rm_is_pad_owned(sci->id, input_data->rm_sc_pad);
            break;
        /* Calls sc_rm_dump() function */
        case IMX_DCMD_SC_RM_DUMP:
            (void) sc_rm_dump(sci->id);
            break;
        /* Calls sc_rm_memreg_frag() function */
        case IMX_DCMD_SC_RM_MEMREG_FRAG:
            status = (sc_err_t)sc_rm_memreg_frag(sci->id,
                                                 &input_data->rm_memreg_frag.mr,
                                                 input_data->rm_memreg_frag.start_address,
                                                 input_data->rm_memreg_frag.end_address);
            break;
        /* Calls sc_timer_set_wdog_timeout() function */
        case IMX_DCMD_SC_TIMER_SET_WDOG_TIMEOUT:
            status = sc_timer_set_wdog_timeout(sci->id, input_data->wdog_time);
            break;
        /* Calls sc_timer_set_wdog_pre_timeout() function */
        case IMX_DCMD_SC_TIMER_SET_WDOG_PRE_TIMEOUT:
            status = sc_timer_set_wdog_pre_timeout(sci->id, input_data->wdog_time);
            break;
        /* Calls sc_timer_start_wdog() function */
        case IMX_DCMD_SC_TIMER_START_WDOG:
            status = sc_timer_start_wdog(sci->id, (sc_bool_t)input_data->wdog_lock);
            break;
        /* Calls sc_timer_stop_wdog() function */
        case IMX_DCMD_SC_TIMER_STOP_WDOG:
            status = sc_timer_stop_wdog(sci->id);
            break;
        /* Calls sc_timer_ping_wdog() function */
        case IMX_DCMD_SC_TIMER_PING_WDOG:
            status = sc_timer_ping_wdog(sci->id);
            break;
        /* Calls sc_timer_get_wdog_status() function */
        case IMX_DCMD_SC_TIMER_GET_WDOG_STATUS:
            status = sc_timer_get_wdog_status(sci->id,
                                              &input_data->timer_status.timeout,
                                              &input_data->timer_status.max_timeout,
                                              &input_data->timer_status.remaining_time);
            nbytes = sizeof(imx_dcmd_sc_timer_status_t);
            break;
        /* Calls sc_timer_pt_get_wdog_status() function */
        case IMX_DCMD_SC_TIMER_PT_GET_WDOG_STATUS:
            status = sc_timer_pt_get_wdog_status(sci->id,
                                                 input_data->timer_pt_status.pt,
                                                 (sc_bool_t *)&input_data->timer_pt_status.enb,
                                                 &input_data->timer_pt_status.timeout,
                                                 &input_data->timer_pt_status.remaining_time);
            nbytes = sizeof(imx_dcmd_sc_timer_pt_status_t);
            break;
        /* Calls sc_timer_set_wdog_action() function */
        case IMX_DCMD_SC_TIMER_SET_WDOG_ACTION:
            status = sc_timer_set_wdog_action(sci->id, input_data->wdog_action.pt, input_data->wdog_action.action);
            break;
        /* Calls sc_timer_set_rtc_time() function */
        case IMX_DCMD_SC_TIMER_SET_RTC_TIME:
            status = sc_timer_set_rtc_time(sci->id, input_data->timer_time.year,
                                           input_data->timer_time.mon, input_data->timer_time.day,
                                           input_data->timer_time.hour, input_data->timer_time.min,
                                           input_data->timer_time.sec);
            break;
        /* Calls sc_timer_get_rtc_time() function */
        case IMX_DCMD_SC_TIMER_GET_RTC_TIME:
            status = sc_timer_get_rtc_time(sci->id, &input_data->timer_time.year,
                                           &input_data->timer_time.mon, &input_data->timer_time.day,
                                           &input_data->timer_time.hour, &input_data->timer_time.min,
                                           &input_data->timer_time.sec);
            nbytes = sizeof(imx_dcmd_sc_timer_time_t);
            break;
        /* Calls sc_timer_get_rtc_sec1970() function */
        case IMX_DCMD_SC_TIMER_GET_RTC_SEC1970:
            status = sc_timer_get_rtc_sec1970(sci->id, &input_data->sec);
            nbytes = sizeof(uint32_t);
            break;
        /* Calls sc_timer_set_rtc_alarm() function */
        case IMX_DCMD_SC_TIMER_SET_RTC_ALARM:
            status = sc_timer_set_rtc_alarm(sci->id, input_data->timer_time.year,
                                            input_data->timer_time.mon, input_data->timer_time.day,
                                            input_data->timer_time.hour, input_data->timer_time.min,
                                            input_data->timer_time.sec);
            break;
        /* Calls sc_timer_set_rtc_calb() function */
        case IMX_DCMD_SC_TIMER_SET_RTC_CALB:
            status = sc_timer_set_rtc_calb(sci->id, input_data->count);
            break;
        /* Calls sc_timer_set_rtc_periodic_alarm() function */
        case IMX_DCMD_SC_TIMER_SET_RTC_PERIODIC_ALARM:
            status = sc_timer_set_rtc_periodic_alarm(sci->id, input_data->sec);
            break;
        /* Calls sc_timer_cancel_rtc_alarm() function */
        case IMX_DCMD_SC_TIMER_CANCEL_RTC_ALARM:
            status = sc_timer_cancel_rtc_alarm(sci->id);
            break;
        case IMX_DCMD_SC_MISC_WAVEFORM_CAPTURE:
            status = sc_misc_waveform_capture(sci->id, (sc_bool_t)input_data->enable);
            break;
        /* Calls sc_timer_set_sysctr_alarm() function */
        case IMX_DCMD_SC_TIMER_SET_SYSCTR_ALARM:
            status = sc_timer_set_sysctr_alarm(sci->id, input_data->ticks);
            break;
        /* Calls sc_timer_set_sysctr_periodic_alarm() function */
        case IMX_DCMD_SC_TIMER_SET_SYSCTR_PERIODIC_ALARM:
            status = sc_timer_set_sysctr_periodic_alarm(sci->id, input_data->ticks);
            break;
        /* Calls sc_timer_cancel_sysctr_alarm() function */
        case IMX_DCMD_SC_TIMER_CANCEL_SYSCTR_ALARM:
            status = sc_timer_cancel_sysctr_alarm(sci->id);
            break;
        /* ----------------------------------------------- SC driver API ----------------------------------------------- */
        case IMX_DCMD_SC_DRV_EVENT_REGISTER:
            /* Add new item to the list */
            if ((new_irq_event = (imx_irq_event_t *)calloc(1, sizeof(imx_irq_event_t))) != NULL) {
                sci->malloc_cnt++;
                if (sci->irq_event != NULL) {
                    new_irq_event->next = sci->irq_event;
                } else {
                    new_irq_event->next = NULL;
                }
                sci->irq_event = new_irq_event;
                new_irq_event->event = input_data->sc_event.event;
                new_irq_event->recvid = ctp->rcvid;
                new_irq_event->ocb = ocb;
                input_data->sc_event.handle = (void *)new_irq_event;
            } else {
                input_data->sc_event.handle = NULL;
                status = SC_ERR_UNAVAILABLE;
            }
            nbytes = sizeof(imx_irq_event_t);
            break;
        case IMX_DCMD_SC_DRV_EVENT_UNREGISTER:
            status = SC_ERR_PARM;
            if (is_item_in_list(sci, input_data->sc_event.handle)) {
                cur_irq_event = sci->irq_event;
                prev_irq_event = NULL;
                while (cur_irq_event != NULL) {
                    if (cur_irq_event == input_data->sc_event.handle) {
                        if (prev_irq_event == NULL) {
                            sci->irq_event = cur_irq_event->next;
                        } else {
                            prev_irq_event->next = cur_irq_event->next;
                        }
                        free((void *)cur_irq_event);
                        sci->malloc_cnt--;

                        /* Disable IRQ in SCFW */
                        memset((void *)irq_enable, 0x00, (sizeof(uint32_t) * SC_IRQ_NUM_GROUP));
                        cur_irq_event = sci->irq_event;
                        while (cur_irq_event != NULL) {
                            for (idx = 0; idx < SC_IRQ_NUM_GROUP; idx++) {
                                irq_enable[idx] |= cur_irq_event->event_item[idx].irq_enable;
                            }
                            cur_irq_event = cur_irq_event->next;
                        }
                        status = SC_ERR_NONE;
                        for (idx = 0; idx < SC_IRQ_NUM_GROUP; idx++) {
                            if (irq_enable[idx] == 0) {
                                status |= sc_irq_enable(sci->id, sci->grp_res, idx, irq_enable[idx], false);
                            }
                        }
                        break;
                    }
                    prev_irq_event = cur_irq_event;
                    cur_irq_event = cur_irq_event->next;
                }
            }
            break;
        case IMX_DCMD_SC_DRV_EVENT_STATUS:
            if ((input_data->sc_event_status.handle != NULL) && is_item_in_list(sci, input_data->sc_event.handle) &&
                (input_data->sc_event_status.group < SC_IRQ_NUM_GROUP)) {
                input_data->sc_event_status.status =
                    ((imx_irq_event_t *)input_data->sc_event_status.handle)->event_item[input_data->sc_event_status.group].irq_status;
                /* Clear event status */
                ((imx_irq_event_t *)input_data->sc_event_status.handle)->event_item[input_data->sc_event_status.group].irq_status = 0;
            } else {
                status = SC_ERR_PARM;
            }
            nbytes = sizeof(imx_irq_event_t);
            break;
        case IMX_DCMD_SC_DRV_EVENT_ENABLE:
            /* Check inputs parameters */
            if ((input_data->sc_event_enable.handle != NULL) && is_item_in_list(sci, input_data->sc_event.handle) &&
                (input_data->sc_event_enable.group < SC_IRQ_NUM_GROUP)) {
                /* Set IRQ enable/disable state */
                if (input_data->sc_event_enable.enable) {
                    ((imx_irq_event_t *)input_data->sc_event_enable.handle)->event_item[input_data->sc_event_enable.group].irq_enable |=
                        input_data->sc_event_enable.mask;
                } else {
                    ((imx_irq_event_t *)input_data->sc_event_enable.handle)->event_item[input_data->sc_event_enable.group].irq_enable &=
                        ~(input_data->sc_event_enable.mask);
                }
                /* Enable/Disable IRQ in SCFW */
                memset((void *)irq_enable, 0x00, (sizeof(uint32_t) * SC_IRQ_NUM_GROUP));
                cur_irq_event = sci->irq_event;
                while (cur_irq_event != NULL) {
                    for (idx = 0; idx < SC_IRQ_NUM_GROUP; idx++) {
                        irq_enable[idx] |= cur_irq_event->event_item[idx].irq_enable;
                    }
                    cur_irq_event = cur_irq_event->next;
                }
                for (idx = 0; idx < SC_IRQ_NUM_GROUP; idx++) {
                    if (irq_enable[idx] != 0) {
                        status |= sc_irq_enable(sci->id, sci->grp_res, idx, irq_enable[idx], true);
                    } else {
                        status |= sc_irq_enable(sci->id, sci->grp_res, idx, irq_enable[idx], false);
                    }
                }
            } else {
                status = SC_ERR_PARM;
            }
            break;
        /* ------------------------------------------------- SECO API ------------------------------------------------ */
        /* Calls sc_seco_image_load() function */
        case IMX_DCMD_SC_SECO_IMAGE_LOAD:
             status = sc_seco_image_load(sci->id, input_data->seco_image_load.addr_src,
                                                  input_data->seco_image_load.addr_dst,
                                                  input_data->seco_image_load.len,
                                                  input_data->seco_image_load.fw);
            break;
        /* Calls sc_seco_authenticate() function */
        case IMX_DCMD_SC_SECO_AUTHENTICATE:
            status = sc_seco_authenticate(sci->id, input_data->seco_authenticate.cmd,
                                                   input_data->seco_authenticate.addr);
            break;
        /* Calls sc_seco_enh_authenticate() function */
        case IMX_DCMD_SC_SECO_ENH_AUTHENTICATE:
            status = sc_seco_enh_authenticate(sci->id, input_data->seco_enh_authenticate.cmd,
                                                   input_data->seco_enh_authenticate.addr,
                                                   input_data->seco_enh_authenticate.mask1,
                                                   input_data->seco_enh_authenticate.mask2);
            break;
        /* Calls sc_seco_forward_lifecycle() function */
        case IMX_DCMD_SC_SECO_FORWARD_LIFECYCLE:
            status = sc_seco_forward_lifecycle(sci->id, input_data->seco_change);
            break;
        /* Calls sc_seco_return_lifecycle() function */
        case IMX_DCMD_SC_SECO_RETURN_LIFECYCLE:
            status = sc_seco_return_lifecycle(sci->id, input_data->faddr);
            break;
        /* Calls sc_seco_commit() function */
        case IMX_DCMD_SC_SECO_COMMIT:
            status = sc_seco_commit(sci->id, &input_data->seco_commit);
            nbytes = sizeof(uint32_t);
            break;
        /* Calls sc_seco_attest_mode() function */
        case IMX_DCMD_SC_SECO_ATTEST_MODE:
            status = sc_seco_attest_mode(sci->id, input_data->seco_mode);
            break;
        /* Calls sc_seco_attest() function */
        case IMX_DCMD_SC_SECO_ATTEST:
            status = sc_seco_attest(sci->id, input_data->seco_nonce);
            break;
        /* Calls sc_seco_get_attest_pkey() function */
        case IMX_DCMD_SC_SECO_GET_ATTEST_PKEY:
            status = sc_seco_get_attest_pkey(sci->id, input_data->faddr);
            break;
        /* Calls sc_seco_get_attest_sign() function */
        case IMX_DCMD_SC_SECO_GET_ATTEST_SIGN:
            status = sc_seco_get_attest_sign(sci->id, input_data->faddr);
            break;
        /* Calls sc_seco_attest_verify() function */
        case IMX_DCMD_SC_SECO_ATTEST_VERIFY:
            status = sc_seco_attest_verify(sci->id, input_data->faddr);
            break;
        /* Calls sc_seco_gen_key_blob() function */
        case IMX_DCMD_SC_SECO_GEN_KEY_BLOB:
            status = sc_seco_gen_key_blob(sci->id, input_data->seco_gen_key_blob.id,
                                                    input_data->seco_gen_key_blob.load_addr,
                                                    input_data->seco_gen_key_blob.export_addr,
                                                    input_data->seco_gen_key_blob.max_size);
            break;
        /* Calls sc_seco_load_key() function */
        case IMX_DCMD_SC_SECO_LOAD_KEY:
            status = sc_seco_load_key(sci->id, input_data->seco_load_key.id,
                                               input_data->seco_load_key.addr);
            break;
        /* Calls sc_seco_get_mp_key() function */
        case IMX_DCMD_SC_SECO_GET_MP_KEY:
            status = sc_seco_get_mp_key(sci->id, input_data->seco_get_mp_key.dst_addr,
                                                 input_data->seco_get_mp_key.dst_size);
            break;
        /* Calls sc_seco_update_mpmr() function */
        case IMX_DCMD_SC_SECO_UPDATE_MPMR:
            status = sc_seco_update_mpmr(sci->id, input_data->seco_update_mpmr.addr,
                                                  input_data->seco_update_mpmr.size,
                                                  input_data->seco_update_mpmr.lock);
            break;
        /* Calls sc_seco_get_mp_sign() function */
        case IMX_DCMD_SC_GET_MP_SIGN:
            status = sc_seco_get_mp_sign(sci->id, input_data->seco_get_mp_sign.msg_addr,
                                                  input_data->seco_get_mp_sign.msg_size,
                                                  input_data->seco_get_mp_sign.dst_addr,
                                                  input_data->seco_get_mp_sign.dst_size);
            break;
        /* Calls sc_seco_build_info() function */
        case IMX_DCMD_SC_SECO_BUILD_INFO:
            sc_seco_build_info(sci->id, &input_data->seco_build_info.version,
                                        &input_data->seco_build_info.commit);
            nbytes = sizeof(imx_dcmd_sc_seco_build_info_t);
            break;
        /* Calls sc_seco_chip_info() function */
        case IMX_DCMD_SC_SECO_CHIP_INFO:
            status = sc_seco_chip_info(sci->id, &input_data->seco_chip_info.lc,
                                                &input_data->seco_chip_info.monotonic,
                                                &input_data->seco_chip_info.uid_l,
                                                &input_data->seco_chip_info.uid_h);
            nbytes = sizeof(imx_dcmd_sc_seco_chip_info_t);
            break;
        /* Calls sc_seco_enable_debug() function */
        case IMX_DCMD_SC_SECO_ENABLE_DEBUG:
            status = sc_seco_enable_debug(sci->id, input_data->faddr);
            break;
        /* Calls sc_seco_get_event() function */
        case IMX_DCMD_SC_SECO_GET_EVENT:
            status = sc_seco_get_event(sci->id, input_data->seco_get_event.idx,
                                                &input_data->seco_get_event.event);
            nbytes = sizeof(imx_dcmd_sc_seco_get_event_t);
            break;
        /* Calls sc_seco_fuse_write() function */
        case IMX_DCMD_SC_SECO_FUSE_WRITE:
            status = sc_seco_fuse_write(sci->id, input_data->faddr);
            break;
        /* Calls sc_seco_patch() function */
        case IMX_DCMD_SC_SECO_PATCH:
            status = sc_seco_patch(sci->id, input_data->faddr);
            break;
        /* Calls sc_seco_start_rng() function */
        case IMX_DCMD_SC_SECO_START_RNG:
            status = sc_seco_start_rng(sci->id, &input_data->seco_rng_stat);
            nbytes = sizeof(sci_mgr_seco_rng_stat_t);
            break;
        /* Calls sc_seco_sab_msg() function */
        case IMX_DCMD_SC_SECO_SAB_MSG:
            status = sc_seco_sab_msg(sci->id, input_data->faddr);
            break;
        /* Calls sc_seco_secvio_enable() function */
        case IMX_DCMD_SC_SECO_SECVIO_ENABLE:
            status = sc_seco_secvio_enable(sci->id);
            break;
        /* Calls sc_seco_secvio_config() function */
        case IMX_DCMD_SC_SECO_SECVIO_CONFIG:
            status = sc_seco_secvio_config(sci->id, input_data->seco_secvio_config.id,
                                                    input_data->seco_secvio_config.access,
                                                    &input_data->seco_secvio_config.data0,
                                                    &input_data->seco_secvio_config.data1,
                                                    &input_data->seco_secvio_config.data2,
                                                    &input_data->seco_secvio_config.data3,
                                                    &input_data->seco_secvio_config.data4,
                                                    input_data->seco_secvio_config.size);
            nbytes = sizeof(imx_dcmd_sc_seco_secvio_config_t);
            break;
        /* Calls sc_seco_secvio_dgo_config() function */
        case IMX_DCMD_SC_SECO_SECVIO_DGO_CONFIG:
            status = sc_seco_secvio_dgo_config(sci->id, input_data->seco_secvio_dgo_config.id,
                                                        input_data->seco_secvio_dgo_config.access,
                                                        &input_data->seco_secvio_dgo_config.data);
            nbytes = sizeof(imx_dcmd_sc_seco_secvio_dgo_config_t);
            break;
        default:
            IMX_SCI_ERROR("unknown DCMD command %u", msg->i.dcmd);
            break;
    }
    (void)pthread_mutex_unlock(&sci->hw_lock);

    if (status != SC_ERR_NONE) {
        IMX_SCI_ERROR(" DCMD command 0x%x returns error %u", msg->i.dcmd, status);
    }

    /* 5) return data (if any) to the client */
    memset(&msg->o, 0, sizeof(msg->o));

    msg->o.ret_val = (int)status;
    msg->o.nbytes = nbytes;

    return (_RESMGR_PTR(ctp, &msg->o, sizeof(msg->o) + nbytes));
}

/**
 * Handles command-line options.
 *
 *  This routine handles the command-line options.
 *      -v      verbose operation
 *      -b      SCI channel base address
 *      -U      set uid:gid
 *
 * @param argc Number of arguments passed to main()
 * @param argv Pointer to the vector of arguments passed to main()
 */
void options(int argc, char **argv)
{
    int opt;
    sci->optv = 0;
    UserParm = NULL;

    while (optind < argc) {
        while ((opt = getopt(argc, argv, "p:i:U:v")) != -1) {
            switch (opt) {
                case 'p':
                    sci->base = strtoul(optarg, &optarg, 0);
                    break;
                case 'i':
                    sci->intr = strtol(optarg, &optarg, 0);
                    break;
                case 'v':
                    sci->optv = 1;
                    break;
                case 'U':
                    UserParm = (UserParm == NULL) ? strdup(optarg) : UserParm;
                    break;
            }
        }
    }
}

/**
 * Interrupt from Message Unit used by SCFW.
 *
 * @param area Memory area.
 * @param id
 *
 * @return struct sigevent*
 * @retval sigevent
 */
static const struct sigevent * mu_handler(__attribute__((unused)) void *area, __attribute__((unused)) int id)
{
    uint32_t reg;

    /* Clear General Interrupt flags */
    reg = in32(sci->id + IMX_MU_ASR);
    reg &= (IMX_MU_ASR_GIP_MASK);
    out32(sci->id + IMX_MU_ASR, reg);

    return &sci->intrevent;
}

void exit_handler(__attribute__((unused)) int dummy)
{
    uint32_t reg;

    InterruptDetach(sci->iid);

    (void)pthread_mutex_lock(&sci->hw_lock);
    (void)pthread_cancel(sci->intr_tid);

    /* Deinit MU */
    /* Disable all interrupts */
    reg = in32(sci->id + IMX_MU_ACR);
    reg &= ~(IMX_MU_ACR_GIE_MASK | IMX_MU_ACR_RIE_MASK | IMX_MU_ACR_TIE_MASK |
             IMX_MU_ACR_GIR_MASK | IMX_MU_ACR_BRDIE_MASK);
    out32(sci->id + IMX_MU_ACR, reg);

    /* Clear status flags */
    out32(sci->id + IMX_MU_ASR, (IMX_MU_ASR_GIP_MASK | IMX_MU_ASR_RF_MASK |
                                 IMX_MU_ASR_TE_MASK | IMX_MU_ASR_BRDIP_MASK));

    /* Disable General Interrupt */
    reg = in32(sci->id + IMX_MU_ACR);
    reg &= ~(IMX_MU_ACR_GIE_MASK);
    out32(sci->id + IMX_MU_ACR, reg);
    (void)pthread_mutex_unlock(&sci->hw_lock);

    (void)pthread_mutex_destroy(&sci->hw_lock);

    exit(EXIT_FAILURE);
}

/**
 * SC driver ISR thread.
 *
 * @param data
 */
static void* imx_sc_drv_thread(__attribute__((unused)) void *data)
{
    struct _pulse pulse;
    imx_irq_event_t *cur_irq_event;
    sc_irq_group_t irq_group;
    uint32_t irq_status[SC_IRQ_NUM_GROUP];
    uint32_t irq_status_ored;

    if ((sci->chid = ChannelCreate(_NTO_CHF_DISCONNECT | _NTO_CHF_UNBLOCK)) == -1) {
        IMX_SCI_ERROR("ChannelCreate() error (%s)", strerror(errno));
        return NULL;
    }
    if ((sci->coid = ConnectAttach(0, 0, sci->chid, _NTO_SIDE_CHANNEL, 0)) == -1) {
        IMX_SCI_ERROR("ConnectAttach() error (%s)", strerror(errno));
        return NULL;
    }
    SIGEV_PULSE_INIT(&sci->intrevent, sci->coid, IMX_SC_PRIORITY, IMX_SC_ISR_EVENT, NULL);

    /* coverity[no_escape] - Suppress coverity INFINITE_LOOP error */
    for (;;) {
        if (MsgReceivePulse(sci->chid, &pulse, sizeof(pulse), NULL) != -1) {
            switch (pulse.code) {
                case IMX_SC_ISR_EVENT:
                    (void)pthread_mutex_lock(&sci->hw_lock);
                    /* Get IRQ status from all groups */
                    for (irq_group = 0; irq_group < SC_IRQ_NUM_GROUP; irq_group++) {
                        if (sc_irq_status(sci->id, sci->grp_res, irq_group, &irq_status[irq_group]) != SC_ERR_NONE) {
                            IMX_SCI_ERROR("SC - Get IRQ status failed!");
                        }
                    }
                    /* Fill status and send event for all items in the list */
                    cur_irq_event = sci->irq_event;
                    while (cur_irq_event != NULL) {
                        irq_status_ored = 0;
                        for (irq_group = 0; irq_group < SC_IRQ_NUM_GROUP; irq_group++) {
                            cur_irq_event->event_item[irq_group].irq_status |= (irq_status[irq_group] &
                                                                                cur_irq_event->event_item[irq_group].irq_enable);
                            irq_status_ored |= cur_irq_event->event_item[irq_group].irq_status;
                        }
                        if (irq_status_ored != 0) {
                            MsgDeliverEvent(cur_irq_event->recvid, &cur_irq_event->event);
                        }
                        cur_irq_event = cur_irq_event->next;
                    }
                    (void)pthread_mutex_unlock(&sci->hw_lock);
                    break;
                default:
                    break;
            }
        } else {
            IMX_SCI_ERROR("MsgReceivePulse() error %s", strerror(errno));
        }
    }
    return (NULL);
}

/**
 * Initializes SC interface.
 *
 * @return Status of initialization.
 */
int imx_sci_init(void)
{
    uint32_t            reg;
    pthread_attr_t      pattr;
    struct sched_param  param;
    int                 policy;

    /* Maps SCI physical memory into a process's address space */
    sci->id = (sc_ipc_id_t) mmap_device_memory(0, SCI_PHY_MEM_SIZE,
                                               PROT_READ | PROT_WRITE | PROT_NOCACHE, 0, sci->base);
    if (sci->id == (sc_ipc_id_t)MAP_FAILED) {
        IMX_SCI_ERROR("Unable to mmap SC IPC Channel (%s)", strerror(errno));
        return EIO;
    }
    (void)pthread_mutex_init(&sci->hw_lock, NULL);
    /* Initialize Message Unit */
    /* Disable all interrupts */
    reg = in32(sci->id + IMX_MU_ACR);
    reg &= ~(IMX_MU_ACR_GIE_MASK | IMX_MU_ACR_RIE_MASK | IMX_MU_ACR_TIE_MASK |
             IMX_MU_ACR_GIR_MASK | IMX_MU_ACR_BRDIE_MASK);
    out32(sci->id + IMX_MU_ACR, reg);

    /* Clear status flags */
    out32(sci->id + IMX_MU_ASR, (IMX_MU_ASR_GIP_MASK | IMX_MU_ASR_RF_MASK |
                                 IMX_MU_ASR_TE_MASK | IMX_MU_ASR_BRDIP_MASK));

    sci->iid = InterruptAttach(sci->intr, mu_handler, NULL, 0, _NTO_INTR_FLAGS_TRK_MSK);
    if (sci->iid == -1) {
        IMX_SCI_ERROR("SC - InterruptAttachEvent() ERROR");
        return EIO;
    }

    pthread_attr_init(&pattr);
    pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);
    pthread_getschedparam(pthread_self(), &policy, &param);
    param.sched_priority = 21;
    pthread_attr_setschedparam(&pattr, &param);
    pthread_attr_setschedpolicy(&pattr, SCHED_RR);

    /* Create thread for this interface */
    /* Coverity issue fixed - Incompatible parameter */
    /* Coverity issue fixed - Forward NULL */
    if (pthread_create(&sci->intr_tid, &pattr, imx_sc_drv_thread, sci) != EOK) {
        InterruptDetach(sci->iid);
        perror("pthread_create() failed \n");
        return EIO;
    }
    pthread_setname_np(sci->intr_tid, "sc_isr_thread");

    /* Enable General Interrupt */
    reg = in32(sci->id + IMX_MU_ACR);
    reg |= (IMX_MU_ACR_GIE_MASK);
    out32(sci->id + IMX_MU_ACR, reg);

    return EOK;
}

/** @} */ /* end of sc */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/support/sc-imx8/imx8_sci_mgr.c $ $Rev: 913737 $")
#endif
