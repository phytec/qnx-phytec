/*
 * $QNXLicenseC:
 * Copyright 2010, 2011 QNX Software Systems.
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



#include "mx6q.h"
#include "bs.h"
#include <net/ifdrvcom.h>
#include <device_qnx.h>


static void mx6q_stop(struct ifnet *ifp, int disable);
static void mx6q_destroy(mx6q_dev_t *mx6q, int how);
static void mx6q_reset(mx6q_dev_t *mx6q);
static int mx6q_init(struct ifnet *ifp);

static int mx6q_attach(struct device *, struct device *, void *);
static int mx6q_detach(struct device *, int);

static void mx6q_shutdown(void *);

static void set_phys_addr (mx6q_dev_t *mx6q);
static void get_phys_addr (mx6q_dev_t *mx6q, uchar_t *addr);

struct mx6q_arg {
    void            *dll_hdl;
    char            *options;
    unsigned        idx;
    nic_config_t    cfg;
};

CFATTACH_DECL(mx6q,
    sizeof(mx6q_dev_t),
    NULL,
    mx6q_attach,
    mx6q_detach,
    NULL);

static  char *mpc_opts [] = {
    "receive",  // 0
    "transmit", // 1
    "freq",     // 2
    "rmii",     // 3
    "mii",      // 4
    "brmast",   // 5
    "rx_frame", // 6, only used on SoloX
    "rx_delay", // 7, only used on SoloX
    "tx_frame", // 8, only used on SoloX
    "tx_delay", // 9, only used on SoloX
    "mode",     // 10
    "gpt_base", // 11
    "gpt_irq",  // 12
    "iout",     // 13
    "slew",     // 14
    "clause45", // 15
    "phy",      // 16
    NULL
};

void dump_mbuf (struct mbuf *m, uint32_t length)
{
#define MX6Q_LINE_BYTES 16
    char        dump_line[(MX6Q_LINE_BYTES * 3) + 1];
    char       *p;
    uint8_t    *data;
    uint32_t    i;

    p = dump_line;
    data = mtod(m, uint8_t*);

    for (i = 0; i < length; i++) {
        snprintf(p, MX6Q_LINE_BYTES, "%02x ", *data++);

        if ((i % MX6Q_LINE_BYTES) == (MX6Q_LINE_BYTES - 1)) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_DEBUG1, "%s", dump_line);
            p = dump_line;
        } else {
            p += 3;
        }
    }
    if (i % MX6Q_LINE_BYTES) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_DEBUG1, "%s", dump_line);
        p = dump_line;
    }
}

//
// called from mx6q_detect()
//
static int
mx6q_parse_options (mx6q_dev_t *mx6q, char *options, nic_config_t *cfg, struct mx6q_arg *mx6q_arg)
{
    char    *value, *restore, *c;
    int     opt;
    int     rc = 0;
    int     err = EOK;

    /* Getting parameters from the Hwinfo Section if available */
    unsigned hwi_off = hwi_find_device("fec", mx6q_arg->idx);
    if(hwi_off != HWI_NULL_OFF) {
       unsigned  i = 0;
       hwi_tag  *tag;

       /* Find base address */
       tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, &i);
        if(tag) {
            mx6q->iobase = tag->location.base;
       }
       /* Find IRQ */
                cfg->irq[0] = hwitag_find_ivec(hwi_off, NULL);
       /* Find MDIO base address */
       tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, &i);
       if(tag) {
           mx6q->phy_base = tag->location.base;
        } else {
            if(mx6q->phy_base == 0) {
                unsigned mdio_hwi_off = hwi_find_device("fec,mdio", 0);
                if (mdio_hwi_off != HWI_NULL_OFF) {
                    unsigned  mdio_idx = 0;
                    hwi_tag  *mdio_tag;
                    mdio_tag = hwi_tag_find(mdio_hwi_off, HWI_TAG_NAME_location, &mdio_idx);
                    if(mdio_tag) {
                        mx6q->phy_base = mdio_tag->location.base;
                    }
                }
            }
        }
       /* Find phy address */
       mx6q->cfg.phy_addr = hwitag_find_phyaddr(hwi_off, NULL);
   }

    /* GPT base address */
    mx6q->tbase = GPT_BASE;
    mx6q->tirq = GPT_INT;

    unsigned hwi_off_gpt = hwi_find_device("fec0_gpt", mx6q_arg->idx);
    uintptr_t tbase;
    int tirq;
    if (hwi_off_gpt != HWI_NULL_OFF) {
        hwi_tag *tag_base = hwi_tag_find(hwi_off_gpt, HWI_TAG_NAME_location, 0);
        hwi_tag *tag_irq = hwi_tag_find(hwi_off_gpt, HWI_TAG_NAME_irq, 0);

        if (tag_base && tag_irq) {
            /* GPT base address */
            tbase = tag_base->location.base;
            if (cfg->verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "GP Timer base address = 0x%08X", (uint32_t) tbase);
            }
            /* GPT IRQ */
            tirq = hwitag_find_ivec(hwi_off_gpt, NULL);
            if (cfg->verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "GP Timer Interrupt = %d", tirq);
            }

            if(tbase && tirq) {
                mx6q->tbase = tbase;
                mx6q->tirq = tirq;
            } else {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): Invalid GPT information - Setting GPT Base and GPT IRQ to default", __FUNCTION__);
            }
        } else {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): Missing GPT information - Setting GPT Base and GPT IRQ to default", __FUNCTION__);
        }
    }

    restore = NULL;
    while (options && *options != '\0') {
        c = options;
            restore = strchr(options, ',');
            opt = getsubopt (&options, mpc_opts, &value);

        switch (opt) {
        case  0:
            if (mx6q) {
                mx6q->num_rx_descriptors = strtoul (value, 0, 0);
            }
            break;

        case  1:
           if (mx6q) {
               mx6q->num_tx_descriptors = strtoul (value, 0, 0);
           }
           break;

        case 2:
            if (mx6q) {
                if (value) {
                    mx6q->clock_freq = strtoul(value, 0, 0);
                    if(mx6q->clock_freq) {
                    mx6q->clock_freq_set = 1;
                    }
                }
            }
            break;

        case 3:
            if (mx6q) {
                mx6q->rmii = 1;
            }
            break;

        case 4:
            if (mx6q) {
                mx6q->mii = 1;
            }
            break;

        case 5:
            if (mx6q && value) {
                mx6q->brmast = strtoul(value, NULL, 0);
            }
            break;
#if defined MX6XSLX | defined MX8XP
        case 6:
            if (mx6q && value) {
                mx6q->rx_frame = strtol(value, NULL, 0);
            }
            break;

        case 7:
            if (mx6q && value) {
                mx6q->rx_delay = strtol(value, NULL, 0);
            }
            break;

        case 8:
            if (mx6q && value) {
                mx6q->tx_frame = strtol(value, NULL, 0);
            }
            break;

        case 9:
            if (mx6q && value) {
                mx6q->tx_delay = strtol(value, NULL, 0);
            }
            break;
#endif
        case 10:
            if (mx6q && value) {
                if (strcmp(value, "avb") == 0) {
                    mx6q->mode = MX6_FUNC_MODE_AVB;
                    mx6q->num_rx_queues = MAX_NUM_QUEUES;
                    mx6q->num_tx_queues = MAX_NUM_QUEUES;
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): AVB mode", __FUNCTION__);
                } else if (strcmp(value, "none") == 0) {
                    mx6q->mode = MX6_FUNC_MODE_DEFAULT;
                    mx6q->num_rx_queues = DEFAULT_NUM_RX_QUEUES;
                    mx6q->num_tx_queues = DEFAULT_NUM_TX_QUEUES;
                }else {
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): Invalid mode: %s", __FUNCTION__, value);
                    err = EINVAL;
                    rc = -1;
                }
            }
            break;
        case 11:
            if (mx6q && value) {
                mx6q->tbase = strtol(value, NULL, 0);
            }
            break;
        case 12:
            if (mx6q && value) {
                mx6q->tirq = strtol(value, NULL, 0);
            }
            break;
        case 13:
            if (mx6q && value) {
                mx6q->iout = strtol(value, NULL, 0);
            }
            break;
        case 14:
            if (mx6q && value) {
                mx6q->slew = strtol(value, NULL, 0);
            }
            break;
        case 15:
            mx6q->cl45 = 1;
            break;
        case 16:
            /* the command line config has supplied an explicit PHY_Address */
            /*   - this will override HWI_TAGS and discovered PHY addresses */
            /*   - if the PHY does not exist it will fail to be detected    */
            /*       For example: a daughter card PHY may NOT be present    */

            if (mx6q->cfg.verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "PHY Addr after hwitag checked: 0x%x",
                    mx6q->cfg.phy_addr);
            }

            if (mx6q && value) {
                /* process the string to get the PHY address */
                mx6q->cfg.phy_addr = strtol(value, NULL, 0);

                /* Mark this as an address from command line config */
                mx6q->cfg.phy_addr |= CONFIG_HAS_DEFINED_PHY_ADDR;
            }

            if (mx6q->cfg.verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "PHY Addr from command line: 0x%x",
                    mx6q->cfg.phy_addr);
            }
            break;
        default:
            if (nic_parse_options (cfg, value) != EOK) {
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): unknown option %s", __FUNCTION__, c);
                    err = EINVAL;
                    rc = -1;
            }
            break;
        }

        if (restore != NULL)
            *restore = ',';
    }

    if (mx6q->cfg.verbose) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "PHY Addr (or Device Index): 0x%x IRQ_0: %d  IRQ_1: %d",
            mx6q->cfg.phy_addr,
            cfg->irq[0],
            cfg->irq[1]);
    }
    errno = err;
    return (rc);
}

void *mx6q_rx_thread (void *arg)
{
    mx6q_dev_t        *mx6q = arg;
    int                rc, queue;
    struct _pulse      pulse;
    volatile uint32_t *base = mx6q->reg;

    while (1) {
        rc = MsgReceivePulse(mx6q->chid, &pulse, sizeof(pulse), NULL);
        if (rc == EOK) {
            switch (pulse.code) {
            case MX6Q_RX_PULSE:
            queue = pulse.value.sival_int;

            if(mx6q->num_rx_queues >= 2) {
                if (queue == 1) {
                    *(base + MX6Q_IEVENT) = IEVENT_RXF1;
                    if (mx6q_receive(mx6q, WTP, 1)) {
                        InterruptLock(&mx6q->spinlock);
                        *(base + MX6Q_IMASK) |= IMASK_RXF1EN;
                        InterruptUnlock(&mx6q->spinlock);
                    }
                }
            }
            if(mx6q->num_rx_queues >= 3) {
                if (queue == 2) {
                    *(base + MX6Q_IEVENT) = IEVENT_RXF2;
                    if (mx6q_receive(mx6q, WTP, 2)) {
                        InterruptLock(&mx6q->spinlock);
                        *(base + MX6Q_IMASK) |= IMASK_RXF2EN;
                        InterruptUnlock(&mx6q->spinlock);
                    }
                }
            }

            if (queue == 0) {
                *(base + MX6Q_IEVENT) = IEVENT_RFINT;
                if (mx6q_receive(mx6q, WTP, 0)) {
                    InterruptLock(&mx6q->spinlock);
                    *(base + MX6Q_IMASK) |= IMASK_RFIEN;
                    InterruptUnlock(&mx6q->spinlock);
                }
            }
            break;
            case MX6Q_QUIESCE_PULSE:
                quiesce_block(pulse.value.sival_int);
                break;
            default:
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "mx6 Rx: Unknown pulse %d received", pulse.code);
                break;
            }
        } else {
          slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "mx6 Rx: MsgReceivePulse: %s", strerror(rc));
        }
    }
    return NULL;
}

void mx6q_rx_thread_quiesce (void *arg, int die)
{
    mx6q_dev_t          *mx6q = arg;

    MsgSendPulse(mx6q->coid, SIGEV_PULSE_PRIO_INHERIT,
         MX6Q_QUIESCE_PULSE, die);
    return;
}

static int mx6q_rx_thread_init (void *arg)
{
    mx6q_dev_t          *mx6q = arg;
    struct nw_work_thread   *wtp = WTP;

    pthread_setname_np(gettid(), MX6Q_RX_THREAD_NAME);

    wtp->quiesce_callout = mx6q_rx_thread_quiesce;
    wtp->quiesce_arg = mx6q;
    return EOK;
}

//
// convert virtual to physical address
//
static paddr_t
vtophys (void *addr)
{
    off64_t  offset;

    if (mem_offset64 (addr, NOFD, 1, &offset, 0) == -1) {
        return (-1);
    }
    return (offset);
}


//
// called from mx6q_detect() to allocate resources
//

static int
mx6q_attach(struct device *parent, struct device *self, void *aux)
{
    mx6q_dev_t         *mx6q;
    nic_config_t       *cfg;
    int                 rc;
    uint32_t            queue, offset, i;
    struct ifnet       *ifp;
    size_t              size;
    char               *options;
    struct _iopkt_self *iopkt;
    struct nw_stk_ctl  *sctlp;
    struct mx6q_arg    *mx6q_arg;
    mpc_bd_t           *bd;
    struct mbuf        *m;
    volatile uint32_t  *base;
    extern int          pkt_typed_mem_fd; /* use io-pkt typed memory */
    int                 mmap_flags;

#if defined MX6XSLX | defined MX8XP
    int                 tx_slog = 0;
    int                 rx_slog = 0;
    uint32_t           rxic_val = 0;
    uint32_t           txic_val = 0;
#endif

    if ((self == NULL) || (aux == NULL)) {
            return EINVAL;
    }

    mx6q = (mx6q_dev_t *)self;
    cfg = &mx6q->cfg;
    iopkt = iopkt_selfp;
    sctlp = &stk_ctl;
    mx6q_arg = aux;

    options = mx6q_arg->options;

    mx6q->dev.dv_dll_hdl = mx6q_arg->dll_hdl;
    mx6q->iopkt = iopkt;
    mx6q->iid_rx = -1;
    mx6q->iid_tx = -1;
    mx6q->iid_err = -1;
    mx6q->iid_1588 = -1;
    mx6q->iout = -1;
    mx6q->slew = -1;

    /* iMX8 platforms require typed memory for descriptors
     *    since ethernet DMA has a 32-bit boundary
     * The iMX8 platforms can allocate memory above 4 Gig
     */
    mx6q->fd = pkt_typed_mem_fd;
#ifdef MX8XP
    if (mx6q->fd == NOFD) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "Attach Error: pkt_typed_mem_fd is NOFD");
        return ENOMEM;
    }
#endif
    mx6q->mii_mutex = NIC_MUTEX_UNLOCKED_VALUE;

    ifp = &mx6q->ecom.ec_if;
    ifp->if_softc = mx6q;

    if ((mx6q->sdhook = shutdownhook_establish(mx6q_shutdown, mx6q)) == NULL) {
        return ENOMEM;
    }

    /* set some defaults for the command line options */
    cfg->flags = NIC_FLAG_MULTICAST;
    cfg->media_rate = cfg->duplex = -1;
    cfg->priority = IRUPT_PRIO_DEFAULT;
    cfg->iftype = IFT_ETHER;
    cfg->lan = -1;
    cfg->phy_addr = -1;
    strlcpy((char *)cfg->uptype, "en", sizeof(cfg->uptype));
    //cfg->verbose = 1;  // XXX debug - set verbose=0 for normal output

    mx6q->num_tx_descriptors = DEFAULT_NUM_TX_DESCRIPTORS;
    mx6q->num_rx_descriptors = DEFAULT_NUM_RX_DESCRIPTORS;

    mx6q->num_rx_queues = DEFAULT_NUM_RX_QUEUES;
    mx6q->num_tx_queues = DEFAULT_NUM_TX_QUEUES;

#if defined MX6XSLX | defined MX8XP
    mx6q->rx_frame = RX_FRAME_DEFAULT;
    mx6q->rx_delay = RX_DELAY_DEFAULT;
    mx6q->tx_frame = TX_FRAME_DEFAULT;
    mx6q->tx_delay = TX_DELAY_DEFAULT;
#endif

    mx6q->probe_phy = 0;

    // set defaults - only used by nicinfo to display mtu
    cfg->mtu = cfg->mru = ETHERMTU;

    cfg->device_index = mx6q_arg->idx;

    /* the Lan # is possibly determined by config - lan=x
     *     - used only for text in string "fec__"
     *     - if lan Index is not configured use the device index
     */
    if (cfg->lan == -1) {
        cfg->lan = mx6q_arg->idx;
    }

    if ((rc = mx6q_parse_options(mx6q, options, cfg, mx6q_arg))) {
        if(errno != ENODEV) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): mx6q_parse_options() failed: %d", __FUNCTION__, rc);
        }
        mx6q_destroy(mx6q, 1);
        return rc;
    }

    /* No param given, use syspage */
    if(cfg->num_mem_windows == 0) {
        cfg->mem_window_base[0] = mx6q->iobase;
        cfg->mem_window_size[0] = MX6Q_MAP_SIZE;
    }
    cfg->mem_window_base[1] = mx6q->tbase;
    cfg->mem_window_size[1] = MX6Q_MAP_SIZE;
    cfg->num_mem_windows = 2;

    if (cfg->verbose) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Device Index: %d Lan Index: %d",
            cfg->device_index, cfg->lan);

        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Register Base:0x%"PRIx64"  GP Timer Base:0x%"PRIx64" ",
            cfg->mem_window_base[0], cfg->mem_window_base[1]);
    }

#if defined MX8XP
    /*  SMMUMAN
     *         System Memory Management Unit - MANager
     *         HW support on iMX8QM - not on the iMX8QXP
     *
     *     register the mmio device
     */
     rc = smmu_register_mmio_device(mx6q->iobase, MX6Q_MAP_SIZE);
     if (rc) {
         slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
               "%s:%d - iMX8 SMMUMAN register mmio retcode: %d",
               __FUNCTION__, __LINE__, rc);
     }
#endif

#ifndef S32V
    /* 1588 interrupt is one after the ENET interrupt */
    cfg->irq[1] = cfg->irq[0] + 1;
    cfg->num_irqs = 2;
#else
    /* 1588 interrupt is first, then Tx, Rx, Error */
    cfg->irq[1] = cfg->irq[0] + 1;
    cfg->irq[2] = cfg->irq[1] + 1;
    cfg->irq[3] = cfg->irq[2] + 1;
    cfg->num_irqs = 4;
#endif

    cfg->media = NIC_MEDIA_802_3;
    cfg->mac_length = ETHER_ADDR_LEN;

    mx6q->set_flow = MX6_FLOW_AUTO;

    // did user specify either of speed or duplex on the cmd line?
    if ((cfg->media_rate != -1) || (cfg->duplex != -1)) {

        if (cfg->media_rate == -1) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): must also specify speed when duplex is specified", __FUNCTION__);
            mx6q_destroy(mx6q, 1);
            return EINVAL;
        }
        if (cfg->duplex == -1) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): must also specify duplex when speed is specified", __FUNCTION__);
            mx6q_destroy(mx6q, 1);
            return EINVAL;
        }

        // we get here, we know both media_rate and duplex are set

        mx6q->set_flow = MX6_FLOW_NONE;
        switch(cfg->media_rate) {
        case 0:
        case 10*1000:
        case 100*1000:
            break;

        case 1000*1000:
            if (mx6q->rmii || mx6q->mii) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
		    "%s(): RMII doesn't support gigabit, only 10/100 Mb/s",
		    __FUNCTION__);
                return EINVAL;
	    }
            break;

        default:
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): invalid speed: %d", __FUNCTION__,
		cfg->media_rate/1000);
            mx6q_destroy(mx6q, 1);
            return EINVAL;
            break;
        }
    }

    cfg->connector = NIC_CONNECTOR_MII;

    strlcpy((char *)cfg->device_description, MX6Q_DEVICE_DESCRIPTION, sizeof(cfg->device_description));

    mx6q->num_rx_descriptors &= ~3;
    if (mx6q->num_rx_descriptors < MIN_NUM_RX_DESCRIPTORS) {
        mx6q->num_rx_descriptors = MIN_NUM_RX_DESCRIPTORS;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_WARNING, "%s: Requested Rx Descriptors too low. Setting to : %d",
              __FUNCTION__, MIN_NUM_RX_DESCRIPTORS);
    }
    if (mx6q->num_rx_descriptors > MAX_NUM_RX_DESCRIPTORS) {
        mx6q->num_rx_descriptors = MAX_NUM_RX_DESCRIPTORS;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_WARNING, "%s: Requested Rx Descriptors too high. Setting to : %d",
              __FUNCTION__, MAX_NUM_RX_DESCRIPTORS);
    }

    mx6q->num_tx_descriptors &= ~3;
    if (mx6q->num_tx_descriptors < MIN_NUM_TX_DESCRIPTORS) {
        mx6q->num_tx_descriptors = MIN_NUM_TX_DESCRIPTORS;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_WARNING, "%s: Requested Tx Descriptors too low. Setting to : %d",
              __FUNCTION__, MIN_NUM_TX_DESCRIPTORS);
    }
    if (mx6q->num_tx_descriptors > MAX_NUM_TX_DESCRIPTORS) {
        mx6q->num_tx_descriptors = MAX_NUM_TX_DESCRIPTORS;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_WARNING, "%s: Requested Tx Descriptors too high. Setting to : %d",
              __FUNCTION__, MAX_NUM_TX_DESCRIPTORS);
    }

    cfg->revision = NIC_CONFIG_REVISION;

    mx6q->cachectl.fd = NOFD;

    if (cache_init(0, &mx6q->cachectl, NULL) == -1) {
        rc = errno;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "mx6q_detect: cache_init: %d", rc);
        mx6q_destroy(mx6q, 1);
        return rc;
    }

    // map nic registers into virtual memory
    if ((mx6q->reg = mmap_device_memory (NULL, cfg->mem_window_size[0],
                         PROT_READ | PROT_WRITE |
                         PROT_NOCACHE,
                         MAP_SHARED,
                         cfg->mem_window_base[0])) == MAP_FAILED) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): mmap regs failed: %d", __FUNCTION__, rc);
        mx6q_destroy(mx6q, 2);
        return rc;
    }
    base = mx6q->reg;

    /* map MDIO registers into virtual memory, if needed */
    /*   ENET2 (with deviceIndex = 1 uses the ENET1 MDIO */
    if((mx6q->phy_base != 0) && (mx6q->phy_base != cfg->mem_window_base[0])) {
        if (mx6q->cfg.verbose > 4) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,
                "mmap ENET2 (fec%d) MDIO to ENET1 HW Address:0x%" PRIxPTR,
                cfg->lan,
                mx6q->phy_base);
        }

        if ((mx6q->phy_reg = mmap_device_memory (NULL, MX6Q_MAP_SIZE,
                             PROT_READ | PROT_WRITE |
                             PROT_NOCACHE,
                             MAP_SHARED,
                             mx6q->phy_base)) == MAP_FAILED) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): mmap regs failed: %d", __FUNCTION__, rc);
            mx6q_destroy(mx6q, 3);
            return rc;
        }
    } else {
        mx6q->phy_reg = mx6q->reg;
    }

    /* default MAC address to current ENET hardware setting (comes from boot loader on first boot) */
    get_phys_addr(mx6q, cfg->permanent_address);
    if (memcmp (cfg->current_address, "\0\0\0\0\0\0", 6) == 0)  {
        memcpy(cfg->current_address, cfg->permanent_address,
               ETHER_ADDR_LEN);
    }

    if (cfg->verbose) {
        nic_dump_config(cfg);
    }

    if (mx6q->fd == NOFD) {
        /* get contiguous memory from the non-Typed Memory
         *    - Using MAP_PHYS with MAP_ANON, mmap() allocates physically contiguous memory
         *    - If you use MAP_PHYS the File Descriptor must be NOFD
         */
        mmap_flags = MAP_SHARED | MAP_ANON | MAP_PHYS;
    } else {
        /* use the File Descriptor obtained from io-pkt to get contiguous Typed Memory */
        mmap_flags = MAP_SHARED;
    }

    /* Map in uncached memory for rx descr ring */
    if ((mx6q->tx_bd = mmap (NULL, sizeof (mpc_bd_t) * mx6q->num_tx_descriptors * mx6q->num_tx_queues,
        PROT_READ | PROT_WRITE | PROT_NOCACHE , mmap_flags, mx6q->fd, 0)) == MAP_FAILED) {
        rc = errno;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): mmap txd failed: %d", __FUNCTION__, rc);
        mx6q_destroy(mx6q, 3);
        return rc;
    }

    // alloc mbuf pointer array, corresponding to tx descr ring
    size = sizeof(struct mbuf *) * mx6q->num_tx_descriptors * mx6q->num_tx_queues;
    mx6q->tx_pkts = malloc(size, M_DEVBUF, M_NOWAIT);
    if (mx6q->tx_pkts == NULL) {
        rc = ENOBUFS;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): malloc tx_pkts failed", __FUNCTION__);
        mx6q_destroy(mx6q, 4);
        return rc;
    }
    memset(mx6q->tx_pkts, 0x00, size);

    // init tx descr ring
    for (queue = 0; queue < mx6q->num_tx_queues; queue++) {
        offset =  mx6q->num_tx_descriptors * queue;
        for (i = 0; i < mx6q->num_tx_descriptors; i++) {
            bd = &mx6q->tx_bd[offset + i];
            /* Clear ownership of the descriptor */
            bd->status = 0;
            bd->estatus = (queue << 20) | TXBD_ESTATUS_INT;
            if (i == (mx6q->num_tx_descriptors - 1)) {
                bd->status |= TXBD_W;
            }
        }
        mx6q->tx_pidx[queue] = mx6q->tx_cidx[queue] = mx6q->tx_descr_inuse[queue] = 0;
    }

#if defined MX8XP
    /* smmuman needs the physical address of the tx descriptors */
    mx6q->tx_descriptor_area_phys = vtophys(mx6q->tx_bd);

    rc = smmu_map_driver_memory(mx6q->tx_descriptor_area_phys,
              sizeof (mpc_bd_t) * mx6q->num_tx_descriptors * mx6q->num_tx_queues);
    if (rc != EOK) {
        rc = errno;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
            "Unable to smmu_map Tx descriptors, smmu failed with err = %s", strerror (errno));
            mx6q->tx_pkts = NULL;
        mx6q_destroy(mx6q, 5);
        mx6q->tx_pkts = NULL;
        mx6q->rx_pkts = NULL;
        return rc;
    }
#endif

    /* Map in uncached memory for rx descr ring */
    if ((mx6q->rx_bd = mmap (NULL, sizeof (mpc_bd_t) * mx6q->num_rx_descriptors * mx6q->num_rx_queues,
        PROT_READ | PROT_WRITE | PROT_NOCACHE , mmap_flags, mx6q->fd, 0)) == MAP_FAILED) {
        rc = errno;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): mmap rxd failed: %d", __FUNCTION__, rc);
        mx6q_destroy(mx6q, 5);
        return rc;
    }

    // alloc mbuf pointer array, corresponding to rx descr ring
    size = sizeof(struct mbuf *) * mx6q->num_rx_descriptors * mx6q->num_rx_queues;
    mx6q->rx_pkts = malloc(size, M_DEVBUF, M_NOWAIT);
    if (mx6q->rx_pkts == NULL) {
        rc = ENOBUFS;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): malloc rx_pkts failed", __FUNCTION__);
        mx6q_destroy(mx6q, 6);
        return rc;
    }
    memset(mx6q->rx_pkts, 0x00, size);

    if (mx6q->mode == MX6_FUNC_MODE_AVB) {
        mx6q->chid = ChannelCreate(0);
        mx6q->coid = ConnectAttach(ND_LOCAL_NODE, 0, mx6q->chid,
                       _NTO_SIDE_CHANNEL, 0);
    }

#if defined MX8XP
    /* smmuman Rx Descriptors */
    mx6q->rx_descriptor_area_phys = vtophys(mx6q->rx_bd);
    rc = smmu_map_driver_memory(mx6q->rx_descriptor_area_phys,
              sizeof (mpc_bd_t) * mx6q->num_rx_descriptors * mx6q->num_rx_queues);
    if (rc != EOK) {
        rc = errno;
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
            "Unable to smmu_map Rx descriptors, smmu failed with err = %s", strerror (errno));
            mx6q->rx_pkts = NULL;
        mx6q_destroy(mx6q, 7);
        mx6q->tx_pkts = NULL;
        mx6q->rx_pkts = NULL;
        return rc;
    }
#endif

    // init rx descr ring
    for (queue = 0; queue < mx6q->num_rx_queues; queue++) {
        offset =  mx6q->num_rx_descriptors * queue;
        for (i = 0; i < mx6q->num_rx_descriptors; i++) {
            bd = &mx6q->rx_bd[offset + i];
            bd->status = RXBD_E;
            if (i == (mx6q->num_rx_descriptors - 1)) {
                bd->status |= RXBD_W;
            }
            bd->estatus = RXBD_ESTATUS_INT;

            m = m_getcl(M_NOWAIT, MT_DATA, M_PKTHDR);
            if (m == NULL) {
                mx6q_destroy(mx6q, 7);
                return ENOMEM;
            }
            mx6q->rx_pkts[offset + i] = m;
            bd->buffer = pool_phys(m->m_data, m->m_ext.ext_page);
            CACHE_INVAL(&mx6q->cachectl, m->m_data, bd->buffer,
                    m->m_ext.ext_size);
        }
        mx6q->rx_cidx[queue] = 0;
        if (mx6q->mode == MX6_FUNC_MODE_AVB) {
            /*
             * Create a pulse for each class of traffic. Start at the
             * default rx_prio but increment by 2 for each class to
             * allow a process to run at (class - 1) receiving and still
             * have priority over the lower class traffic without
             * impacting the dequeueing of packets from the limited Rx
             * descriptors.
             */
            SIGEV_PULSE_INIT(&mx6q->isr_event[queue], mx6q->coid,
                 sctlp->rx_prio + (2 * queue),
                 MX6Q_RX_PULSE, queue);
        }
    }

    if (mx6q->mode == MX6_FUNC_MODE_AVB) {
        mx6q->inter.func = mx6q_process_interrupt;
        mx6q->inter.enable = mx6q_enable_interrupt;
    } else {
        mx6q->inter.func = mx6q_process_interrupt_rx;
        mx6q->inter.enable = mx6q_enable_rx;
    }

    mx6q->inter.arg = mx6q;

    if ((rc = interrupt_entry_init(&mx6q->inter, 0, NULL,
        cfg->priority)) != EOK) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
              "%s(): interrupt_entry_init(rx) failed: %d", __FUNCTION__, rc);
        mx6q_destroy(mx6q, 7);
        return rc;
    }

    memset(&mx6q->spinlock, 0, sizeof(mx6q->spinlock));

    if (mx6q->mode == MX6_FUNC_MODE_AVB) {
        /* pseudo interrupt for Rx queue */
        mx6q->inter_queue.func = mx6q_process_queue;
        mx6q->inter_queue.enable = mx6q_enable_queue;
        mx6q->inter_queue.arg = mx6q;

        if ((rc = interrupt_entry_init(&mx6q->inter_queue, 0, NULL,
            cfg->priority)) != EOK) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): interrupt_entry_init(rx) failed: %d", __FUNCTION__, rc);
            mx6q_destroy(mx6q, 8);
            return rc;
        }

        if ((rc = pthread_mutex_init(&mx6q->rx_mutex, NULL)) != EOK) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): rx_mutex init failed: %d",
                __FUNCTION__, rc);
            mx6q_destroy(mx6q, 9);
            return rc;
        }

        IFQ_SET_MAXLEN(&mx6q->rx_queue, IFQ_MAXLEN);
        nw_pthread_create(&mx6q->tid, NULL,
              mx6q_rx_thread, mx6q, 0,
              mx6q_rx_thread_init, mx6q);
    }

    /* Reset the chip */
    mx6q_reset(mx6q);

    switch(mx6q->num_tx_queues) {
        case(3):
            *(base + MX6Q_X_DES_START2) = vtophys((void *)&mx6q->tx_bd[mx6q->num_tx_descriptors * 2]);
        case(2):
            *(base + MX6Q_X_DES_START1) = vtophys((void *)&mx6q->tx_bd[mx6q->num_tx_descriptors]);
        case(1):
            *(base + MX6Q_X_DES_START) = vtophys((void *)&mx6q->tx_bd[0]);
            break;
        default:
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s():Invalid number of Tx Queues", __FUNCTION__);
            return EINVAL;
    }
    switch(mx6q->num_rx_queues) {
        case(3):
            *(base + MX6Q_R_DES_START2) = vtophys((void *)&mx6q->rx_bd[mx6q->num_rx_descriptors * 2]);
        case(2):
           *(base + MX6Q_R_DES_START1) = vtophys((void *)&mx6q->rx_bd[mx6q->num_rx_descriptors]);
        case(1):
            *(base + MX6Q_R_DES_START) = vtophys((void *)&mx6q->rx_bd[0]);
            break;
        default:
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s():Invalid number of Rx Queues", __FUNCTION__);
            return EINVAL;
    }

    // Set transmit FIFO to store and forward
    *(base + MX6Q_X_WMRK) = X_WMRK_STR_FWD;

    /*
     * Set maximum receive buffer size.
     * Freescale have confirmed the spec is wrong, only bits 6-10 should
     * be used.
     */
    switch(mx6q->num_rx_queues) {
        case(3):
            *(base + MX6Q_R_BUFF_SIZE2) = min(MCLBYTES, MX6Q_MAX_RBUFF_SIZE) &
                        MX6Q_MAX_RBUFF_SIZE;
        case(2):
            *(base + MX6Q_R_BUFF_SIZE1) = min(MCLBYTES, MX6Q_MAX_RBUFF_SIZE) &
                        MX6Q_MAX_RBUFF_SIZE;
        case(1):
            *(base + MX6Q_R_BUFF_SIZE) = min(MCLBYTES, MX6Q_MAX_RBUFF_SIZE) &
                    MX6Q_MAX_RBUFF_SIZE;
            break;
        default:
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s():Invalid number of Rx Queues", __FUNCTION__);
            return EINVAL;
    }

#if defined MX6XSLX || defined MX7D || defined MX8XP
    /*
     * DMA classes need to be enabled before enabling the chip.
     * There should be no traffic to them without a set bandwidth
     * ioctl call first.
     */
    *(base + MX6Q_DMACFG1) = DMACFG_DMA_CLASSEN;
    *(base + MX6Q_DMACFG2) = DMACFG_DMA_CLASSEN;
#endif
    // Set Rx FIFO thresholds for Pause generation
    *(base + MX6Q_R_SECTION_FULL_ADDR)  = RX_SECTION_FULL_THRESHOLD;
    *(base + MX6Q_R_SECTION_EMPTY_ADDR) = RX_SECTION_EMPTY_THRESHOLD;
    *(base + MX6Q_R_ALMOST_EMPTY_ADDR)  = RX_ALMOST_EMPTY_THRESHOLD;
    *(base + MX6Q_R_ALMOST_FULL_ADDR)   = RX_ALMOST_FULL_THRESHOLD;

    // Enable Pause
    *(base + MX6Q_R_CNTRL) |= RCNTRL_FCE;
    *(base + MX6Q_OP_PAUSE) = 0xFFF0;

    *(base + MX6Q_IADDR1) = 0x0;
    *(base + MX6Q_IADDR2) = 0x0;
    *(base + MX6Q_GADDR1) = 0x0;
    *(base + MX6Q_GADDR2) = 0x0;

    // Enable and clear MIB Registers
    *(base + MX6Q_MIB_CONTROL) &= ~MIB_DISABLE;
    *(base + MX6Q_MIB_CONTROL) |= MIB_CLEAR;
    *(base + MX6Q_MIB_CONTROL) &= ~MIB_CLEAR;
    mx6q_clear_stats(mx6q);

    // Set media interface type
    if (mx6q->rmii) {
        *(base + MX6Q_R_CNTRL) |= RCNTRL_RMII_MODE;
    } else if (mx6q->mii) {
        *(base + MX6Q_R_CNTRL) &= ~(RCNTRL_RGMII_ENA | RCNTRL_RMII_MODE);
    } else {  //ENET defaults to RGMII mode
        *(base + MX6Q_R_CNTRL) |= RCNTRL_RGMII_ENA;
    }

    // As per reference manual this bit should always be 1 - MII/RMII mode
    *(base + MX6Q_R_CNTRL) |= RCNTRL_MII_MODE;

    /*
     * Internal MAC clock
     * See reference manual for field mapping
     */
    *(base + MX6Q_MII_SPEED) = MII_HOLDTIME_SHIFT | MII_SPEED_SHIFT;

    // Disable internal loopback
    *(base + MX6Q_R_CNTRL) &= ~RCNTRL_LOOP;

    // Full duplex by default
    *(base + MX6Q_X_CNTRL) |= XCNTRL_FDEN;

    // Clear interrupt status
    *(base + MX6Q_IEVENT) = 0xffffffff;

#ifndef S32V
    // Attach to hardware interrupts
    if (mx6q->iid_rx == -1) {
        if (mx6q->mode == MX6_FUNC_MODE_AVB) {
            rc = InterruptAttach_r(mx6q->cfg.irq[0], mx6q_isr, mx6q,
                sizeof(*mx6q), _NTO_INTR_FLAGS_TRK_MSK);
        } else {
            rc = InterruptAttach_r(mx6q->cfg.irq[0], mx6q_isr_rx, mx6q,
                sizeof(*mx6q), _NTO_INTR_FLAGS_TRK_MSK);
        }
        if (rc < 0) {
            rc = -rc;
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): InterruptAttach_r(rx) failed: %d", __FUNCTION__, rc);
            mx6q_destroy(mx6q, 11);
            return rc;
        }
        mx6q->iid_rx = rc;
    }

    if (mx6q->iid_1588 == -1) {
        if ((rc = InterruptAttach_r(mx6q->cfg.irq[1], mx6q_1588_isr,
            mx6q, sizeof(*mx6q), _NTO_INTR_FLAGS_TRK_MSK)) < 0) {
            rc = -rc;
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): InterruptAttach_r(1588) failed: %d", __FUNCTION__, rc);
            mx6q_destroy(mx6q, 12);
            return rc;
        }
        mx6q->iid_1588 = rc;
    }

    /* Map in the timer registers */
    if ((mx6q->treg = mmap_device_memory (NULL, MX6Q_MAP_SIZE,
                          PROT_READ | PROT_WRITE |
                          PROT_NOCACHE,
                          MAP_SHARED,
                          mx6q->tbase)) == MAP_FAILED) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): mmap timer regs failed: %d",
            __FUNCTION__, rc);
        mx6q_destroy(mx6q, 12);
        return rc;
    }
#else
    // Attach to hardware interrupts
    if (mx6q->iid_rx == -1) {
        if (mx6q->mode == MX6_FUNC_MODE_AVB) {
            rc = InterruptAttach_r(mx6q->cfg.irq[1], mx6q_isr, mx6q,
                sizeof(*mx6q), _NTO_INTR_FLAGS_TRK_MSK);
        } else {
            rc = InterruptAttach_r(mx6q->cfg.irq[1], mx6q_isr_rx, mx6q,
                sizeof(*mx6q), _NTO_INTR_FLAGS_TRK_MSK);
        }
        if (rc < 0) {
            rc = -rc;
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): InterruptAttach_r(rx) failed: %d", __FUNCTION__, rc);
            mx6q_destroy(mx6q, 12);
            return rc;
        }
        mx6q->iid_rx = rc;
    }

    if (mx6q->iid_tx == -1) {
        if (mx6q->mode == MX6_FUNC_MODE_AVB) {
            rc = InterruptAttach_r(mx6q->cfg.irq[2], mx6q_isr, mx6q,
                sizeof(*mx6q), _NTO_INTR_FLAGS_TRK_MSK);
        } else {
            rc = InterruptAttach_r(mx6q->cfg.irq[2], mx6q_isr_rx, mx6q,
                sizeof(*mx6q), _NTO_INTR_FLAGS_TRK_MSK);
        }
        if (rc < 0) {
            rc = -rc;
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): InterruptAttach_r(tx) failed: %d", __FUNCTION__, rc);
            mx6q_destroy(mx6q, 12);
            return rc;
        }
        mx6q->iid_tx = rc;
    }

    if (mx6q->iid_err == -1) {
        if (mx6q->mode == MX6_FUNC_MODE_AVB) {
            rc = InterruptAttach_r(mx6q->cfg.irq[3], mx6q_isr, mx6q,
                sizeof(*mx6q), _NTO_INTR_FLAGS_TRK_MSK);
        } else {
            rc = InterruptAttach_r(mx6q->cfg.irq[3], mx6q_isr_rx, mx6q,
                sizeof(*mx6q), _NTO_INTR_FLAGS_TRK_MSK);
        }
        if (rc < 0) {
            rc = -rc;
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): InterruptAttach_r(err) failed: %d", __FUNCTION__, rc);
            mx6q_destroy(mx6q, 12);
            return rc;
        }
        mx6q->iid_err = rc;
    }

    /* The 1588 interrupt doesn't line up exactly with the 32bit versions */
#endif

    /* Enable the interrupts */
    InterruptLock(&mx6q->spinlock);

    *(base + MX6Q_IMASK) = IMASK_TS_TIMER;
    switch(mx6q->num_rx_queues) {
        case(3):
            *(base + MX6Q_IMASK) |= IMASK_RXF2EN;
        case(2):
            *(base + MX6Q_IMASK) |= IMASK_RXF1EN;
        case(1):
            *(base + MX6Q_IMASK) |= IMASK_RFIEN;
            break;
        default:
	    InterruptUnlock(&mx6q->spinlock);
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s():Invalid number of Rx Queues", __FUNCTION__);
            return EINVAL;
    }
    switch(mx6q->num_tx_queues) {
        case(3):
            *(base + MX6Q_IMASK) |= IMASK_TXF2EN;
        case(2):
            *(base + MX6Q_IMASK) |= IMASK_TXF1EN;
        case(1):
            *(base + MX6Q_IMASK) |= IMASK_TFIEN;
            break;
        default:
	    InterruptUnlock(&mx6q->spinlock);
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s():Invalid number of Tx Queues", __FUNCTION__);
            return EINVAL;
    }

#if defined MX6XSLX | defined MX8XP
    /* Setup Interrupt coalescence for TX interrupts */
    if (mx6q->tx_frame || mx6q->tx_delay) {
        if (!mx6q->tx_frame || !mx6q->tx_delay) {
		tx_slog  = 1;
        } else {
            txic_val = TXIC_ICEN | TXIC_ICFT(mx6q->tx_frame) | TXIC_ICTT(mx6q->tx_delay);
        }
    }

    *(base + MX6SLX_TXIC0) = txic_val;
    *(base + MX6SLX_TXIC1) = txic_val;
    *(base + MX6SLX_TXIC2) = txic_val;

    /* Setup Interrupt coalescence for RX interrupts */
    if (mx6q->rx_frame || mx6q->rx_delay) {
        if (!mx6q->rx_frame || !mx6q->rx_delay) {
            rx_slog  = 1;
        } else {
            rxic_val = RXIC_ICEN | RXIC_ICFT(mx6q->rx_frame) | RXIC_ICTT(mx6q->rx_delay);
        }
    }

    /* Only enable on Queue 0. Other queues have AVB traffic. */
    *(base + MX6SLX_RXIC0) = rxic_val;
#endif
    InterruptUnlock(&mx6q->spinlock);
#if defined MX6XSLX | defined MX8XP
    if(rx_slog){
	    slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): must set BOTH rx_delay %d and rx_frame %d",
                __FUNCTION__, mx6q->rx_delay, mx6q->rx_delay);
    }
    if(tx_slog){
	    slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): must set BOTH tx_delay %d and tx_frame %d",
                __FUNCTION__, mx6q->tx_delay, mx6q->tx_delay);
    }
#endif

#ifndef SWITCHMODE

    if (mx6q->iout == -1) {
        mx6q->iout = 2;     // Default iout setting
    }
    if (mx6q->slew == -1) {
        mx6q->slew = 2;     // Default slew setting
    }
    if (mx6q->iout < 0 || mx6q->iout > 3) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Invalid iout value %d - using default", mx6q->iout);
        mx6q->iout = 2;
    }
    if (mx6q->slew < 0 || mx6q->slew > 3) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Invalid slew value %d - using default", mx6q->slew);
        mx6q->slew = 2;
    }
    // Do one time PHY initialization
    if (mx6q_init_phy(mx6q)) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "fec%d: PHY init failed", cfg->lan);

        /* release all the allocated resources */
        mx6q_destroy(mx6q, 13);

        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "fec%d: PHY Interface Destroyed: how=13", cfg->lan);
        }
        return ENODEV;
    }

    if (!mx6_is_br_phy(mx6q)) {
        // hook up so media devctls work
        bsd_mii_initmedia(mx6q);
    }
#endif

    // Configure 1588 timer
    mx6q_ptp_start(mx6q);
    if (cfg->verbose) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Configured PTP 1588 timer: fec%d", cfg->lan);
    }

    // interface setup - entry points, etc
    strlcpy(ifp->if_xname, mx6q->dev.dv_xname, sizeof(ifp->if_xname));
    ifp->if_flags = IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST;
    ifp->if_ioctl = mx6q_ioctl;
    ifp->if_start = mx6q_start;
    ifp->if_init = mx6q_init;
    ifp->if_stop = mx6q_stop;
    IFQ_SET_READY(&ifp->if_snd);

    if_attach(ifp);
    ether_ifattach(ifp, mx6q->cfg.current_address);

    mx6q->ecom.ec_capabilities |= ETHERCAP_VLAN_MTU;

#if defined MX6XSLX || defined MX7D || defined MX8XP
    mx6q->ecom.ec_capabilities |= ETHERCAP_JUMBO_MTU;
#endif

    if (mx6q->mode == MX6_FUNC_MODE_AVB) {
        /* Intercept if_output for pulling off AVB packets */
        mx6q->stack_output = mx6q->ecom.ec_if.if_output;
        mx6q->ecom.ec_if.if_output = mx6q_output;
    }

    if (cfg->verbose > 3) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "successfully attached fec%d\n", cfg->lan);
    }

    return EOK;
}

static void
mx6q_destroy(mx6q_dev_t *mx6q, int how)
{
    struct ifnet        *ifp;
    int         i;
    struct mbuf     *m;
#if defined MX8XP
    nic_config_t        *cfg = &mx6q->cfg;
    int                  rc = 0;
#endif

    ifp = &mx6q->ecom.ec_if;

    /* FALLTHROUGH all of these */
    switch (how) {

    //
    // called from mx6q_destroy()
    //
    case -1:
        /*
         * Don't init() while we're dying.  Yes it can happen:
         * ether_ifdetach() calls bridge_ifdetach() which
         * tries to take us out of promiscuous mode with an
         * init().
         */
        mx6q->dying = 1;

        // shut down callbacks
        callout_stop(&mx6q->mii_callout);
        callout_stop(&mx6q->sqi_callout);

        mx6q_reset(mx6q);

        ether_ifdetach(ifp);
        if_detach(ifp);

#ifndef SWITCHMODE
        bsd_mii_finimedia(mx6q);
#endif

    //
    // called from mx6q_attach()
    //
    case 13:
#ifndef SWITCHMODE
        mx6q_fini_phy(mx6q);
#endif
#ifndef S32V
        munmap_device_memory(mx6q->treg, MX6Q_MAP_SIZE);
#endif
    case 12:
        if (mx6q->iid_1588 != -1) {
            InterruptDetach(mx6q->iid_1588);
            mx6q->iid_1588 = -1;
        }
        if (mx6q->iid_rx != -1) {
            InterruptDetach(mx6q->iid_rx);
            mx6q->iid_rx = -1;
        }
        if (mx6q->iid_tx != -1) {
            InterruptDetach(mx6q->iid_tx);
            mx6q->iid_tx = -1;
        }
        if (mx6q->iid_err != -1) {
            InterruptDetach(mx6q->iid_err);
            mx6q->iid_err = -1;
        }
    case 11:
        if (mx6q->mode == MX6_FUNC_MODE_AVB) {
            nw_pthread_reap(mx6q->tid);
        }
    case 10:
        if (mx6q->mode == MX6_FUNC_MODE_AVB) {
            pthread_mutex_destroy(&mx6q->rx_mutex);
        }
    case 9:
        if (mx6q->mode == MX6_FUNC_MODE_AVB) {
            interrupt_entry_remove(&mx6q->inter_queue, NULL);
        }
    case 8:
        interrupt_entry_remove(&mx6q->inter, NULL);
    case 7:
        if (mx6q->mode == MX6_FUNC_MODE_AVB) {
            IF_PURGE(&mx6q->rx_queue);
            pthread_mutex_destroy(&mx6q->rx_mutex);
            ConnectDetach(mx6q->coid);
            ChannelDestroy(mx6q->chid);
        }

#if defined MX8XP
        /*  SMMUMAN
         *         System Memory Management Unit - MANager
         *         HW support on iMX8QM - not on the iMX8QXP
         */
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,
            "%s:%d - smmu_unmap Rx Desc: SMMUMAN : IF:%d",
            __FUNCTION__, __LINE__,cfg->lan);

        /* register the mmio device */
        rc = smmu_unmap_driver_memory(mx6q->rx_descriptor_area_phys,
                                      MX6Q_MAP_SIZE);
        if (rc) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
                "%s:%d - SMMUMAN retcode: %d",
                __FUNCTION__, __LINE__, rc);
        }
#endif

        for (i = 0; i < mx6q->num_rx_descriptors * mx6q->num_rx_queues;
             i++) {
            if ((m = mx6q->rx_pkts[i])) {
                m_freem(m);
            }
        }
        free(mx6q->rx_pkts, M_DEVBUF);

    case 6:
        munmap(mx6q->rx_bd, sizeof(mpc_bd_t) *
               mx6q->num_rx_descriptors * mx6q->num_rx_queues);

    case 5:
#if defined MX8XP
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,
            "%s:%d - smmu_unmap Tx Desc: SMMUMAN : IF:%d",
            __FUNCTION__, __LINE__,cfg->lan);

        /* register the mmio device */
        rc = smmu_unmap_driver_memory(mx6q->tx_descriptor_area_phys,
                                      MX6Q_MAP_SIZE);
        if (rc) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
                "%s:%d - SMMUMAN retcode: %d",
                __FUNCTION__, __LINE__, rc);
        }
#endif

        for (i = 0; i < mx6q->num_tx_descriptors * mx6q->num_tx_queues;
             i++) {
            if ((m = mx6q->tx_pkts[i])) {
                m_freem(m);
            }
        }
        free(mx6q->tx_pkts, M_DEVBUF);

    case 4:
        munmap(mx6q->tx_bd, sizeof(mpc_bd_t) *
               mx6q->num_tx_descriptors * mx6q->num_tx_queues);

    case 3:
        munmap_device_memory(mx6q->reg, MX6Q_MAP_SIZE);

        if(mx6q->phy_reg && (mx6q->phy_reg != mx6q->reg)) {
            munmap_device_memory(mx6q->phy_reg, MX6Q_MAP_SIZE);
        }

    case 2:
        cache_fini(&mx6q->cachectl);

    case 1:
#if defined MX8XP
         slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,
               "%s:%d - unregister SMMUMAN device: IF:%d base:0x%lx size: %d",
               __FUNCTION__, __LINE__, cfg->lan,
               mx6q->iobase, MX6Q_MAP_SIZE);

         /* UNregister the mmio device */
         rc = smmu_unregister_mmio_device(mx6q->iobase, MX6Q_MAP_SIZE);
         if (rc) {
             slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
                   "%s:%d - iMX8 SMMUMAN retcode: %d",
                   __FUNCTION__, __LINE__, rc);
         }
#endif

        shutdownhook_disestablish(mx6q->sdhook);
        break;
    }
}

static void
mx6q_shutdown(void *arg)
{
    mx6q_dev_t  *mx6q = arg;

    mx6q_reset(mx6q);
}

static int
mx6q_detach(struct device *dev, int flags)
{
    mx6q_dev_t  *mx6q;

    mx6q = (mx6q_dev_t *)dev;

    nic_mutex_lock(&mx6q->mii_mutex);

#ifndef SWITCHMODE
    MDI_PowerdownPhy(mx6q->mdi, mx6q->cfg.phy_addr);
#endif
    mx6q_destroy(mx6q, -1);
    nic_mutex_unlock(&mx6q->mii_mutex);

    return EOK;
}

static void
mx6q_reset (mx6q_dev_t *mx6q)
{
    volatile uint32_t   *base = mx6q->reg;

    /* reset */
    *(base + MX6Q_ECNTRL) = ECNTRL_RESET;

    if (*(base + MX6Q_ECNTRL) & ECNTRL_RESET) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): Reset bit didn't clear", __FUNCTION__);
    }

    /* We use little endian enhanced descriptors */
    *(base + MX6Q_ECNTRL) = ECNTRL_DBSWP | ECNTRL_ENA_1588;

    // re program MAC address to PADDR registers
    set_phys_addr(mx6q);
}

static void
set_phys_addr (mx6q_dev_t *mx6q)
{
    // Program MAC address provided from command line argument
    uint32_t    *base = mx6q->reg;

    *(base + MX6Q_PADDR1) = (mx6q->cfg.current_address [0] << 24 |
        mx6q->cfg.current_address [1] << 16 | mx6q->cfg.current_address [2] << 8 |
        mx6q->cfg.current_address [3]);
    *(base + MX6Q_PADDR2) = (mx6q->cfg.current_address [4] << 24 |
        mx6q->cfg.current_address [5] << 16 | 0x8808);

}

/*
 *  Read MAC address from ENET hardware.
 *  Set by bootloader on reset (likely from Fuse registers
 *  but can be overridden).
 */
static void
get_phys_addr (mx6q_dev_t *mx6q, uchar_t *addr)
{
    uint32_t    *base = mx6q->reg;
    uint32_t     mx6q_paddr;

    mx6q_paddr = *(base + MX6Q_PADDR1);
    addr [0] = (mx6q_paddr >> 24) & ~0x01; /* clear multicast bit for sanity's sake */
    addr [1] = mx6q_paddr >> 16;
    addr [2] = mx6q_paddr >> 8;
    addr [3] = mx6q_paddr;
    mx6q_paddr = *(base + MX6Q_PADDR2);
    addr [4] = mx6q_paddr >> 24;
    addr [5] = mx6q_paddr >> 16;
}

void DumpPhy(mx6q_dev_t *mx6q)
{
    uint32_t      i;
    nic_config_t *cfg     = &mx6q->cfg;
    int           phy_idx = cfg->phy_addr;

    if (mx6q->cl45) return;

//    TCHAR PhyReg[]={
//        {TEXT("Control")},
//        {TEXT("Status")},
//        {TEXT("Phy1")},
//        {TEXT("Phy2")},
//        {TEXT("Auto-Nego")},
//        {TEXT("Link partner")},
//        {TEXT("Auto-Nego_ex")},
//        {TEXT("Next page")},
//        {TEXT("Link Partner next")},
//        {TEXT("1000 BT ctl")},
//        {TEXT("100 BT stat")},
//        {TEXT("RSVD")},
//        {TEXT("RSVD")},
//        {TEXT("MMD control")},
//        {TEXT("MMD data")},
//        {TEXT("ext status")},
//        {TEXT("func ctl")},
//        {TEXT("phy status")},
//        {TEXT("intr en")},
//        {TEXT("intr stat")},
//        {TEXT("smart speed")},
//    };
    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,"--------DumpPhy----------\r\n");
    nic_mutex_lock(&mx6q->mii_mutex);
    for(i=0;i<0x20;i+=4)
    {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,"phy[%d]:0x%x phy[%d]:0x%x phy[%d]:0x%x phy[%d]:0x%x",
            i,mx6q_mii_read(mx6q, phy_idx,i),i+1,mx6q_mii_read(mx6q, phy_idx,i+1),i+2,mx6q_mii_read(mx6q, phy_idx,i+2),i+3,mx6q_mii_read(mx6q, phy_idx,i+3));
    }
    nic_mutex_unlock(&mx6q->mii_mutex);
    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,"--------DumpPhy END----------\r\n");
}

static int
mx6q_init (struct ifnet *ifp)
{
    mx6q_dev_t          *mx6q = ifp->if_softc;
    nic_config_t        *cfg = &mx6q->cfg;
    volatile uint32_t   *base = mx6q->reg;
    int                  mtu;
    uint32_t             rcntrl;

    if (cfg->verbose > 3) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): starting: idx %d\n",
            __FUNCTION__, cfg->device_index);
    }

    if (mx6q->dying) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): dying", __FUNCTION__);
        return EOK;
    }

    // Sort out MAC address
    memcpy(cfg->current_address, LLADDR(ifp->if_sadl), ifp->if_addrlen);
    if (memcmp (cfg->current_address, "\0\0\0\0\0\0", 6) == 0) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s():You must specify a MAC address", __FUNCTION__);
        return EINVAL;
    }

    // write to MX6Q_PADDR1 and MX6Q_PADDR2 from cfg->current_address
    set_phys_addr(mx6q);

    // Promiscuous if required
    if (ifp->if_flags & IFF_PROMISC) {
        *(base + MX6Q_R_CNTRL) |= RCNTRL_PROM;
        cfg->flags |= NIC_FLAG_PROMISCUOUS;
    } else {
        *(base + MX6Q_R_CNTRL) &= ~RCNTRL_PROM;
        cfg->flags &= ~NIC_FLAG_PROMISCUOUS;
    }

    // Get mtu from stack for nicinfo
    cfg->mtu = ifp->if_mtu;
    cfg->mru = ifp->if_mtu;

    // Program it in to the MAC
    mtu = (ifp->if_mtu + ETHER_HDR_LEN + ETHER_CRC_LEN);
    if (mx6q->ecom.ec_capenable & ETHERCAP_VLAN_MTU) {
        mtu += ETHER_VLAN_ENCAP_LEN;
    }
    if (mtu != cfg->mtu) {
        rcntrl = *(base + MX6Q_R_CNTRL);
        rcntrl &= 0xc000ffff;
        rcntrl |= (mtu << 16);
        *(base + MX6Q_R_CNTRL) = rcntrl;
        *(base + MX6Q_TRUNC_FL_ADDR) = mtu;
    }

    if ((ifp->if_flags & IFF_RUNNING) == 0) {
        NW_SIGLOCK(&ifp->if_snd_ex, mx6q->iopkt);
        InterruptLock(&mx6q->spinlock);
        ifp->if_flags_tx |= IFF_RUNNING;
        ifp->if_flags |= IFF_RUNNING;
        InterruptUnlock(&mx6q->spinlock);
        NW_SIGUNLOCK(&ifp->if_snd_ex, mx6q->iopkt);

#ifndef SWITCHMODE
        if (!mx6_is_br_phy(mx6q)) {
            bsd_mii_mediachange(ifp);
        } else {
            mx6q_MDI_MonitorPhy(mx6q);
        }
#endif

        // PHY is now ready, turn on MAC
        *(base + MX6Q_ECNTRL) |= ECNTRL_ETHER_EN;
        *(base + MX6Q_ECNTRL) &= ~ECNTRL_SLEEP;

#ifdef SWITCHMODE
        mx6q_speeduplex(mx6q);
        mx6q->cfg.flags &= ~NIC_FLAG_LINK_DOWN;
        if_link_state_change(ifp, LINK_STATE_UP);
#endif

        // Instruct MAC to process receive frames
        switch(mx6q->num_rx_queues) {
            case(3):
                *(base + MX6Q_R_DES_ACTIVE2) = R_DES_ACTIVE;
            case(2):
                *(base + MX6Q_R_DES_ACTIVE1) = R_DES_ACTIVE;
            case(1):
                *(base + MX6Q_R_DES_ACTIVE) = R_DES_ACTIVE;
                break;
            default:
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s():Invalid number of Rx Buffers", __FUNCTION__);
                return EINVAL;
        }
    }

    if (cfg->verbose > 3) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): ending: idx %d\n",
          __FUNCTION__, cfg->device_index);
    }

#ifndef SWITCHMODE
    //DumpPhy(mx6q);
#endif

    return EOK;
}

void
mx6q_speeduplex (mx6q_dev_t *mx6q)
{
    nic_config_t        *cfg = &mx6q->cfg;
    volatile uint32_t   *base = mx6q->reg;

    if (cfg->duplex) {
        *(base + MX6Q_X_CNTRL) |= XCNTRL_FDEN;
        *(base + MX6Q_R_CNTRL) &= ~RCNTRL_DRT;
    } else {
        *(base + MX6Q_X_CNTRL) &= ~XCNTRL_FDEN;
        *(base + MX6Q_R_CNTRL) |= RCNTRL_DRT;
    }

    switch (cfg->media_rate) {
/* On i.MX6UL and i.MX6ULL board, bit 5 of reg ECNTRL is reserved. */
#ifndef MX6UL
    case 1000 * 1000L:
        *(base + MX6Q_ECNTRL) |= ECNTRL_ETH_SPEED;
        break;
#endif
    case 100 * 1000L:
#ifndef MX6UL
        *(base + MX6Q_ECNTRL) &= ~ECNTRL_ETH_SPEED;
#endif
        *(base + MX6Q_R_CNTRL) &= ~RCNTRL_RMII_10T;
        break;
    case 10 * 1000L:
        *(base + MX6Q_R_CNTRL) |= RCNTRL_RMII_10T;
#ifndef MX6UL
        *(base + MX6Q_ECNTRL) &= ~ECNTRL_ETH_SPEED;
#endif
        break;
    default:
        /* Speed 0, PHY powered down, MAC doesn't matter */
        break;
    }
}

static void
mx6q_stop(struct ifnet *ifp, int disable)
{
    mx6q_dev_t		*mx6q = ifp->if_softc;
    uint32_t		rx_active, tx_active, i, queue, offset;
    volatile uint32_t	rx, tx;
    mpc_bd_t		*bd;
    struct mbuf		*m;
    volatile uint32_t	*base = mx6q->reg;
    int             rxq_slog = 0;
    int             txq_slog = 0;

    // shut down mii probing
    callout_stop(&mx6q->mii_callout);
    callout_stop(&mx6q->sqi_callout);

#ifndef SWITCHMODE
    MDI_DisableMonitor(mx6q->mdi);
#endif
    mx6q->cfg.flags |= NIC_FLAG_LINK_DOWN;
    if_link_state_change(ifp, LINK_STATE_DOWN);

    /* Take the locks */
    NW_SIGLOCK(&ifp->if_snd_ex, mx6q->iopkt);
    InterruptLock(&mx6q->spinlock);

    /* Mark the interface as down */
    ifp->if_flags &= ~(IFF_RUNNING);
    ifp->if_flags_tx &= ~(IFF_OACTIVE | IFF_RUNNING);

    // stop the MAC by putting it to sleep
    *(base + MX6Q_ECNTRL) |= ECNTRL_SLEEP;

    /* If the DMA was running wait for it to gracefully stop */
    rx_active = tx_active = 0;
    switch(mx6q->num_rx_queues) {
        case(3):
            rx_active |= *(base + MX6Q_R_DES_ACTIVE2);
        case(2):
            rx_active |= *(base + MX6Q_R_DES_ACTIVE1);
        case(1):
            rx_active |= *(base + MX6Q_R_DES_ACTIVE);
            break;
        default:
            rxq_slog = 1;
    }
    switch(mx6q->num_tx_queues) {
        case(3):
            tx_active |= *(base + MX6Q_X_DES_ACTIVE2);
        case(2):
            tx_active |= *(base + MX6Q_X_DES_ACTIVE1);
        case(1):
            tx_active |= *(base + MX6Q_X_DES_ACTIVE);
            break;
        default:
            txq_slog = 1;
    }

    InterruptUnlock(&mx6q->spinlock);

    if(rxq_slog){
	    slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s():Invalid number of Rx Queues", __FUNCTION__);
    }
    if(txq_slog){
	    slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s():Invalid number of Tx Queues", __FUNCTION__);
    }
    for (i = 0; i < STOP_TIMEOUT; i++) {
        rx = *(base + MX6Q_R_CNTRL);
        tx = *(base + MX6Q_X_CNTRL);
        if (((rx_active == 0) || (rx & RCNTRL_GRS)) &&
            ((tx_active == 0) || (tx & XCNTRL_GTS))) {
            break;
        }
    }
    if (i >= STOP_TIMEOUT) {
	slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): Failed to gracefully stop", __FUNCTION__);
    }

    for (queue = 0; queue < mx6q->num_tx_queues; queue++) {
        offset = mx6q->num_tx_descriptors * queue;

        /* Reap any Tx descriptors */
        mx6q_transmit_complete(mx6q, queue);

        /* Clear any pending Tx buffers */
        for (i = 0; i < mx6q->num_tx_descriptors; i++) {
            if ((m = mx6q->tx_pkts[offset + i])) {
                m_freem(m);
                mx6q->tx_pkts[offset + i] = NULL;
                bd = &mx6q->tx_bd[offset + i];
                /* Clear the TXBD_R */
                bd->status &= TXBD_W;
            }
        }
        /*
         * Make sure we start queueing descriptors
         * from where the hardware stopped
         */
        mx6q->tx_pidx[queue] = mx6q->tx_cidx[queue];
        mx6q->tx_descr_inuse[queue] = 0;
    }

    /* Tx is clean, unlock ready for next time */
    NW_SIGUNLOCK(&ifp->if_snd_ex, mx6q->iopkt);

#ifndef SWITCHMODE
    if (!mx6_is_br_phy(mx6q)) {
	MDI_PowerdownPhy(mx6q->mdi, mx6q->cfg.phy_addr);
    }
#endif
}

//
// called from mx6q_entry() in mx6q.c
//
static char *detect_opt[] = {
  "deviceindex",
  NULL
};

int
mx6q_detect(void *dll_hdl, struct _iopkt_self *iopkt, char *options)
{
    struct mx6q_arg	mx6q_arg;
    struct device	*dev;
    char		*dev_opt, *dev_opt_orig, *value;
    int			i, dev_idx, opt, single, err;
    struct              ifnet          *ifp;
    struct              drvcom_config  *dcon;
    unsigned            hwi_off;
    int                 dev_num = 0;
    int                 max_dev_num = 0;
    int                 num_interfaces_added = 0;
    bool                enet1_is_configured = FALSE;
    bool                enet2_is_configured = FALSE;
    char                device_description_string[10];


    /* Check if it is already mounted by doing a "nicinfo" on each interface */
    dcon = (malloc)(sizeof(*dcon));
    if (dcon == NULL) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
              "%s:%d - malloc failed", __FUNCTION__, __LINE__);
        return ENOMEM;
    }

    /* set up the debug print counter */
    i = 0;

    /* loop to check if the Device is already configured */
    /* the first 16 entries are empty by design          */
    IFNET_FOREACH(ifp) {
        dcon->dcom_cmd.ifdc_cmd = DRVCOM_CONFIG;
        dcon->dcom_cmd.ifdc_len = sizeof(dcon->dcom_config);
        err = ifp->if_ioctl(ifp, SIOCGDRVCOM, (caddr_t)dcon);

        if ((mx6q_arg.cfg.verbose > 4) && (i > 15)) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s  loop %d err %d",
                ifp->if_xname, i, err);
            i++;
        }

        /* Check if the driver for ENET_1 is already attached */
        if ((err == EOK) && (dcon->dcom_config.num_mem_windows > 0)   &&
            (dcon->dcom_config.mem_window_base[0] == MX6Q_ENET1_BASE))  {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "ENET_1 Driver already loaded for %s",
                ifp->if_xname);
            enet1_is_configured = TRUE;
        }

        /* Check if the driver for ENET_2 is already attached */
        if ((err == EOK) && (dcon->dcom_config.num_mem_windows > 0)   &&
            (dcon->dcom_config.mem_window_base[0] == MX6Q_ENET2_BASE))  {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "ENET_2 Driver already loaded for %s",
                ifp->if_xname);
            enet2_is_configured = TRUE;
        }

        /* return if both the ENET interfaces are already configured */
        if(enet1_is_configured && enet2_is_configured) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "Config Failed: Both Enet Interfaces already exist");

            (free)(dcon);
            return EBUSY;
        }
    }
    (free)(dcon);

    /* set up the auxillary information for the device_attach() call */
    mx6q_arg.dll_hdl = dll_hdl;
    mx6q_arg.options = options;

    /* read hwinfo to get the number of Ethernet interfaces on the processor */
    for (i = 0; ; i++) {
        hwi_off = hwi_find_device("fec", i);
         if (hwi_off == HWI_NULL_OFF) {
             break;
         }
    }
    dev_num = i;
    max_dev_num = dev_num;

    /* Parse deviceindex without destroying the options */
    dev_idx = -1;
    dev_opt = NULL;
    if (options != NULL) {
	dev_opt = strdup(options);
    }
    dev_opt_orig = dev_opt;
    while (dev_opt && *dev_opt != '\0') {
	opt = getsubopt(&dev_opt, detect_opt, &value);

	if (opt == 0) {
	    if (value == NULL) {
		slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s: Ignoring deviceindex without value",
		    __FUNCTION__);
	    } else {
                /* the device Index is specified - configure just the specified index */
	        dev_idx = strtoul (value, 0, 0);
                dev_num = 1;
	    }
	}
    }
    if (dev_opt_orig != NULL) {
	(free)(dev_opt_orig);
    }

    /* Attach all mx6x ethernet controllers - based on number of devices */
    /*    Only 1 device specified if it is provided by the device_number */
    for (i = 0; i < dev_num; i++) {
        /* idx == -1 if no deviceindex passed with configuration */
        if (dev_idx == -1) {
            mx6q_arg.idx = i;
        } else {
            mx6q_arg.idx = dev_idx;
        }

        /* check if ENET_1 /Index_0 is already configured */
        if (enet1_is_configured && (mx6q_arg.idx == 0)) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "Adding Dev Index: %d It is already configured",
                mx6q_arg.idx);

            /* get the logs in the correct order */
            nic_delay(1);
            return EBUSY;
        }

        /* check if ENET_2 /Index_1 is already configured */
        if (enet2_is_configured && (mx6q_arg.idx == 1)) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "Adding Dev Index: %d It is already configured",
                mx6q_arg.idx);

            /* get the logs in the correct order */
            nic_delay(1);
            return EBUSY;
        }

        /* check if device Index is in range for this device */
        /* Index is the (deviceNumber - 1)                   */
        if ( mx6q_arg.idx > (max_dev_num -1)) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,
                "HWI tags have %d ENET/fec device(s)", max_dev_num);

            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR,
                "Failed to add Device Index %d: It is not a valid value",
                mx6q_arg.idx);

            return EINVAL;
        }

        dev = NULL; /* No parent */
        if (dev_attach("fec", options, &mx6q_ca, &mx6q_arg, &single,
		       &dev, NULL) != 0) {
            /* when looping all devices may not have a PHY */
            if (mx6q_arg.cfg.verbose > 3) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "device_attach(%d) FAILed\n", i);
            }
        } else {
            /* at least 1 interface has been added to io-pkt */
            num_interfaces_added++;
            if (mx6q_arg.cfg.verbose > 3) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "attached index:%d of %d device(s): Config idx:%d",
                     mx6q_arg.idx, dev_num, dev_idx);
            }
        }
    }

    /* Do this only if all interfaces failed to be added */
    /*   daughter cards with a PHY may not be attached   */
    if (num_interfaces_added == 0) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "could not attach any devices");
        return ENODEV;
    }

    /* at least 1 of the interfaces was added successfully */
    strcpy(device_description_string, MX6Q_DEVICE_DESCRIPTION);
    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s: attached %u devices",
        device_description_string,
        num_interfaces_added);

    return 0;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devnp/mx6x/detect.c $ $Rev: 910691 $")
#endif
