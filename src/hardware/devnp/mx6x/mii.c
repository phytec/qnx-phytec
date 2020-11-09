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

#define MII_OP_ADDR  0
#define MII_OP_WRITE 1
#define MII_OP_READ  2
#define START_C45    0
#define START_C22    1
#define MII_TA       2

/*
 * On the i.MX6 SoloX there are two interfaces but the PHYs are
 * typically both hung off the MDIO on the first. To overcome this
 * we use mx6q->phy_reg here in the MII accesses, on fec0 it will
 * be set to the normal register space, however on fec1 it will
 * be set across to fec0. See mx6q_parse_options() for where it gets
 * picked up at "Find MDIO base address" and mx6q_attach() for where it
 * gets mapped at "map MDIO registers into virtual memory, if needed".
 *
 * One problem with this is that fec1 cannot guarantee that the MDC setup
 * on fec0 is always good, e.g. if fec0 isn't discovered or if fec0 gets
 * destroyed which triggers a reset of the fec0 block. We must always ensure
 * the MDC setup is correct on each read/write access.
 */

static uint16_t
mx6q_mii_read_raw (void *handle, uint8_t cl, uint8_t phy_add, uint8_t reg_add)
{
    mx6q_dev_t         *mx6q = (mx6q_dev_t *) handle;
    volatile uint32_t  *base = mx6q->phy_reg;
    int                 timeout = MPC_TIMEOUT;
    uint32_t            val;
    uint16_t            retval;

    /*
     * Internal MAC clock
     * See reference manual for field mapping
     */
    *(base + MX6Q_MII_SPEED) = MII_HOLDTIME_SHIFT | MII_SPEED_SHIFT;

    *(base + MX6Q_IEVENT) = IEVENT_MII;
    val = ((cl << 30) | (MII_OP_READ << 28) | (phy_add << 23) | (reg_add << 18) | (MII_TA << 16));
    *(base + MX6Q_MII_DATA) = val;

    while (timeout--) {
        if (*(base + MX6Q_IEVENT) & IEVENT_MII) {
            *(base + MX6Q_IEVENT) = IEVENT_MII;
            break;
        }
        nanospin_ns (10000);
    }

    retval = ((timeout <= 0) ? 0 : (*(base + MX6Q_MII_DATA) & 0xffff));

    return retval;
}

static void
mx6q_mii_write_raw (void *handle, uint8_t cl, uint8_t op, uint8_t phy_add, uint8_t reg_add, uint16_t data)
{
    mx6q_dev_t         *mx6q = (mx6q_dev_t *) handle;
    volatile uint32_t  *base = mx6q->phy_reg;
    int                 timeout = MPC_TIMEOUT;
    uint32_t            phy_data;

    /*
     * Internal MAC clock
     * See reference manual for field mapping
     */
    *(base + MX6Q_MII_SPEED) = MII_HOLDTIME_SHIFT | MII_SPEED_SHIFT;

    *(base + MX6Q_IEVENT) = IEVENT_MII;
    phy_data = ((cl << 30) | (op << 28) | (phy_add << 23) | (reg_add << 18) | (MII_TA << 16) | data);
    *(base + MX6Q_MII_DATA) = phy_data;
    while (timeout--) {
        if (*(base + MX6Q_IEVENT) & IEVENT_MII) {
            *(base + MX6Q_IEVENT) = IEVENT_MII;
            break;
        }
        nanospin_ns (10000);
    }
}

uint16_t
mx6q_mii_read_cl45 (void *handle, uint8_t phy_add, uint16_t reg_add, uint16_t dev_add)
{
    /* Write the address. */
    mx6q_mii_write_raw (handle, START_C45, MII_OP_ADDR, phy_add, dev_add, reg_add);

    return mx6q_mii_read_raw (handle, START_C45, phy_add, dev_add);
}

void
mx6q_mii_write_cl45 (void *handle, uint8_t phy_add, uint16_t reg_add, uint16_t dev_add, uint16_t data)
{
    /* Write the address. */
    mx6q_mii_write_raw (handle, START_C45, MII_OP_ADDR, phy_add, dev_add, reg_add);

    /* Write the data. */
    mx6q_mii_write_raw (handle, START_C45, MII_OP_WRITE, phy_add, dev_add, data);
}

uint16_t
mx6q_mii_read (void *handle, uint8_t phy_add, uint8_t reg_add)
{
    mx6q_dev_t  *mx6q = (mx6q_dev_t *) handle;

    if (mx6q->cl45) {
        return mx6q_mii_read_cl45 (handle, phy_add, reg_add, 1);    // Only called from MDI_FindPhy
    }

    return mx6q_mii_read_raw (handle, START_C22, phy_add, reg_add);
}

void
mx6q_mii_write (void *handle, uint8_t phy_add, uint8_t reg_add, uint16_t data)
{
    mx6q_dev_t  *mx6q = (mx6q_dev_t *) handle;

    if (mx6q->cl45) {
        return mx6q_mii_write_cl45 (handle, phy_add, reg_add, 1, data); // Only called from MDI_FindPhy
    }
    mx6q_mii_write_raw (handle, START_C22, MII_OP_WRITE, phy_add, reg_add, data);
}

//
// drvr lib callback when PHY link state changes
//
static void
mx6q_mii_callback(void *handle, uchar_t phy, uchar_t newstate)
{
    mx6q_dev_t      *mx6q = handle;
    nic_config_t        *cfg  = &mx6q->cfg;
    char            *s;
    int         i;
    int         mode;
    struct ifnet        *ifp = &mx6q->ecom.ec_if;
    volatile uint32_t   *base = mx6q->reg;
    int         phy_idx = cfg->phy_addr;
    uint16_t        advert, lpadvert;

    switch(newstate) {
    case MDI_LINK_UP:
        if ((i = MDI_GetActiveMedia(mx6q->mdi, cfg->phy_addr, &mode)) != MDI_LINK_UP) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): MDI_GetActiveMedia() failed: %x", __FUNCTION__, i);
            mode = 0;  // force default case below - all MDI_ macros are non-zero
        }

        switch(mode) {
        case MDI_10bTFD:
            s = "10 BaseT Full Duplex";
            cfg->duplex = 1;
            cfg->media_rate = 10 * 1000L;
/* On i.MX6UL and i.MX6ULL board, bit 5 of reg ECNTRL is reserved. */
#ifndef MX6UL
            *(base + MX6Q_ECNTRL) &= ~ECNTRL_ETH_SPEED;
#endif
            break;
        case MDI_10bT:
            s = "10 BaseT Half Duplex";
            cfg->duplex = 0;
            cfg->media_rate = 10 * 1000L;
#ifndef MX6UL
            *(base + MX6Q_ECNTRL) &= ~ECNTRL_ETH_SPEED;
#endif
            break;
        case MDI_100bTFD:
            s = "100 BaseT Full Duplex";
            cfg->duplex = 1;
            cfg->media_rate = 100 * 1000L;
#ifndef MX6UL
            *(base + MX6Q_ECNTRL) &= ~ECNTRL_ETH_SPEED;
#endif
            break;
        case MDI_100bT:
            s = "100 BaseT Half Duplex";
            cfg->duplex = 0;
            cfg->media_rate = 100 * 1000L;
#ifndef MX6UL
            *(base + MX6Q_ECNTRL) &= ~ECNTRL_ETH_SPEED;
#endif
            break;
        case MDI_100bT4:
            s = "100 BaseT4";
            cfg->duplex = 0;
            cfg->media_rate = 100 * 1000L;
#ifndef MX6UL
            *(base + MX6Q_ECNTRL) &= ~ECNTRL_ETH_SPEED;
#endif
            break;
#ifndef MX6UL
        case MDI_1000bT:
            s = "1000 BaseT Half Duplex";
            cfg->duplex = 0;
            cfg->media_rate = 1000 * 1000L;
            *(base + MX6Q_ECNTRL) |= ECNTRL_ETH_SPEED;
            break;
        case MDI_1000bTFD:
            s = "1000 BaseT Full Duplex";
            cfg->duplex = 1;
            cfg->media_rate = 1000 * 1000L;
            *(base + MX6Q_ECNTRL) |= ECNTRL_ETH_SPEED;
            break;
#endif
        default:
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): unknown link mode 0x%X", __FUNCTION__, mode);
            s = "Unknown";
            cfg->duplex = 0;
            cfg->media_rate = 0L;
            return;
        }

	// immediately set new speed and duplex in nic config registers
	mx6q_speeduplex(mx6q);

	switch (mx6q->set_flow) {
	case MX6_FLOW_AUTO:
	    /* Flow control was autoneg'd, set what we got in the MAC */
        nic_mutex_lock(&mx6q->mii_mutex);
        advert = mx6q_mii_read(mx6q, phy_idx, MDI_ANAR);
        lpadvert = mx6q_mii_read(mx6q, phy_idx, MDI_ANLPAR);
        nic_mutex_unlock(&mx6q->mii_mutex);

	    if (advert & MDI_FLOW) {
		if (lpadvert & MDI_FLOW) {
		    /* Enable Tx and Rx of Pause */
		    *(base + MX6Q_R_SECTION_EMPTY_ADDR) = 0x82;
		    *(base + MX6Q_R_CNTRL) |= RCNTRL_FCE;
		    mx6q->flow_status = MX6_FLOW_BOTH;
		} else if ((advert & MDI_FLOW_ASYM) &&
			   (lpadvert & MDI_FLOW_ASYM)) {
		    /* Enable Rx of Pause */
		    *(base + MX6Q_R_SECTION_EMPTY_ADDR) = 0;
		    *(base + MX6Q_R_CNTRL) |= RCNTRL_FCE;
		    mx6q->flow_status = MX6_FLOW_RX;
		} else {
		    /* Disable all pause */
		    *(base + MX6Q_R_SECTION_EMPTY_ADDR) = 0;
		    *(base + MX6Q_R_CNTRL) &= ~RCNTRL_FCE;
		    mx6q->flow_status = MX6_FLOW_NONE;
		}
	    } else if ((advert & MDI_FLOW_ASYM) &&
		       (lpadvert & MDI_FLOW) &&
		       (lpadvert & MDI_FLOW_ASYM)) {
		/* Enable Tx of Pause */
		*(base + MX6Q_R_SECTION_EMPTY_ADDR) = 0x82;
		*(base + MX6Q_R_CNTRL) &= ~RCNTRL_FCE;
		mx6q->flow_status = MX6_FLOW_TX;
	    } else {
		/* Disable all pause */
		*(base + MX6Q_R_SECTION_EMPTY_ADDR) = 0;
		*(base + MX6Q_R_CNTRL) &= ~RCNTRL_FCE;
		mx6q->flow_status = MX6_FLOW_NONE;
	    }
	    break;

	default:
	    /* Forced */
	    mx6q->flow_status = mx6q->set_flow;
	    break;
	}

	cfg->flags &= ~NIC_FLAG_LINK_DOWN;
	if (cfg->verbose) {
	    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): link up lan %d idx %d (%s)",
		__FUNCTION__, cfg->lan, cfg->device_index, s);
	}
	if_link_state_change(ifp, LINK_STATE_UP);
	break;

    case MDI_LINK_DOWN:
        cfg->media_rate = cfg->duplex = -1;
        cfg->flags |= NIC_FLAG_LINK_DOWN;

        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,
                "%s(): Link down lan %d idx %d, calling MDI_AutoNegotiate()",
                __FUNCTION__, cfg->lan, cfg->device_index);
        }
        MDI_AutoNegotiate(mx6q->mdi, cfg->phy_addr, NoWait);
        if_link_state_change(ifp, LINK_STATE_DOWN);
        break;

    default:
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): idx %d: Unknown link state 0x%X",
        __FUNCTION__, cfg->device_index, newstate);
        break;
    }
}

/*****************************************************************************/
/* Check for link up/down condition of NXP PHY.                              */
/*****************************************************************************/

static void mx6_nxp_phy_state (mx6q_dev_t *mx6q)
{
    uint16_t        phy_data, snr;
    nic_config_t    *cfg;
    struct ifnet    *ifp;

    cfg = &mx6q->cfg;
    ifp = &mx6q->ecom.ec_if;

    nic_mutex_lock(&mx6q->mii_mutex);
    phy_data = mx6q_mii_read(mx6q, cfg->phy_addr, INT_STATUS_REG);
    if (mx6q->brmast) {
        if (phy_data & TRAINING_FAILED) {
            phy_data = mx6q_mii_read(mx6q, cfg->phy_addr, EXT_CONTROL_REG);
            mx6q_mii_write(mx6q, cfg->phy_addr, EXT_CONTROL_REG,
               (phy_data | TRAINING_RESTART));
            nic_mutex_unlock(&mx6q->mii_mutex);
            return;
        }
    }
    nic_mutex_unlock(&mx6q->mii_mutex);

    if (phy_data & PHY_INT_ERR) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): PHY INT ERR 0x%x", __FUNCTION__, phy_data);
        if (phy_data & PHY_INIT_FAIL) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - PHY Init failure");
        }
        if (phy_data & LINK_STATUS_FAIL) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Link status failure");
        }
        if (phy_data & SYM_ERR) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Symbol error");
        }
        if (phy_data & CONTROL_ERR) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - SMI control error");
        }
        if (phy_data & UV_ERR) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Under voltage error");
        }
        if (phy_data & TEMP_ERR) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Temperature error");
        }
    }

    nic_mutex_lock(&mx6q->mii_mutex);
    phy_data = mx6q_mii_read(mx6q, cfg->phy_addr, COMM_STATUS_REG);
    nic_mutex_unlock(&mx6q->mii_mutex);

    snr = (phy_data & SNR_MASK) >> SNR_SHIFT;
    if (snr != mx6q->sqi && (phy_data & LINK_UP)) {
        mx6q->sqi = (uint8_t) snr;
        if (!snr) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): SNR Worse than class A", __FUNCTION__);
    } else {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): SNR class %c", __FUNCTION__, snr + 0x40);
    }
    }
    if (phy_data & COMM_STATUS_ERR) {
    slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): COMM STATUS ERR 0x%x", __FUNCTION__, phy_data);
        if (phy_data & SSD_ERR) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - SSD error");
    }
        if (phy_data & ESD_ERR) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - ESD error");
    }
        if (phy_data & RX_ERR) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Receive error");
    }
        if (phy_data & TX_ERR) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Transmit error");
    }
    }

    if ((cfg->flags & NIC_FLAG_LINK_DOWN) && (phy_data & LINK_UP)) {
        /* Link was down and is now up */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): Link up", __FUNCTION__);
        }
        cfg->flags &= ~NIC_FLAG_LINK_DOWN;
        if_link_state_change (ifp, LINK_STATE_UP);
        nic_mutex_lock(&mx6q->mii_mutex);
        phy_data = mx6q_mii_read(mx6q, cfg->phy_addr, BASIC_CONTROL);
        nic_mutex_unlock(&mx6q->mii_mutex);
        cfg->media_rate = (phy_data & BC_SPEED_SEL) ? 100000L : 10000L;
        cfg->duplex = (phy_data & BC_DUPLEX) ? 1 : 0;

    } else if (((cfg->flags & NIC_FLAG_LINK_DOWN) == 0) &&
           ((phy_data & LINK_UP) == 0)) {
        /* Link was up and is now down */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): Link down", __func__);
        }
        mx6q->sqi = 0;
        cfg->flags |= NIC_FLAG_LINK_DOWN;
        if_link_state_change (ifp, LINK_STATE_DOWN);
    }

    nic_mutex_lock(&mx6q->mii_mutex);
    phy_data = mx6q_mii_read(mx6q, cfg->phy_addr, EXTERN_STATUS_REG);
    nic_mutex_unlock(&mx6q->mii_mutex);

    if (phy_data) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s: External status 0x%x", __FUNCTION__, phy_data);
        if (phy_data & UV_VDDA_3V3) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Undervoltage 3V3");
        }
        if (phy_data & UV_VDDD_1V8) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Undervoltage VDDD 1V8");
        }
        if (phy_data & UV_VDDA_1V8) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Undervoltage VDDA 1V8");
        }
        if (phy_data & UV_VDDIO) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Undervoltage VDDIO");
        }
        if (phy_data & TEMP_HIGH) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Temperature high");
        }
        if (phy_data & TEMP_WARN) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Temperature warning");
        }
        if (phy_data & SHORT_DETECT) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Short circuit detected");
        }
        if (phy_data & OPEN_DETECT) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Open circuit detected");
        }
        if (phy_data & POL_DETECT) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Polarity inversion detected");
        }
        if (phy_data & INTL_DETECT) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, " - Interleave detect");
        }
    }
}

/**
 * Detection of TI DP83TC811S BroadR-Reach state.
 *
 * @param   mx6q   Pointer to device data structure.
 */
static void mx6q_ti_phy_state(mx6q_dev_t *mx6q)
{
    uint16_t        val;
    nic_config_t    *cfg;
    struct ifnet    *ifp;

    if (mx6q->cfg.verbose) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s()...", __FUNCTION__);
    }

    cfg = &mx6q->cfg;
    ifp = &mx6q->ecom.ec_if;
    /* Read BMSR Register 0x0001 – Basic Mode Status Register */
    val = mx6q_mii_read(mx6q,  cfg->phy_addr, 0x1);
    /* BMSR_LINK_STATUS */
    if ((cfg->flags & NIC_FLAG_LINK_DOWN) && (val & BMSR_LINK_STATUS)) {
        /* Link was down and is now up */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): link up", __FUNCTION__);
        }
        cfg->flags &= ~NIC_FLAG_LINK_DOWN;
        if_link_state_change(ifp, LINK_STATE_UP);
        cfg->media_rate = 100 * 1000L;
        cfg->duplex = 1;
    } else if (((cfg->flags & NIC_FLAG_LINK_DOWN) == 0) && ((val & BMSR_LINK_STATUS) == 0)) {
        /* Link was up and is now down */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): link down", __FUNCTION__);
        }
        cfg->flags |= NIC_FLAG_LINK_DOWN;
        if_link_state_change(ifp, LINK_STATE_DOWN);
    }
}

/**
 * Detection of Marvell 88Q211x BroadR-Reach state.
 *
 * @param   mx6q   Pointer to device data structure.
 */
static void mx6q_mv_phy_state(mx6q_dev_t *mx6q)
{
    int             phy_idx = mx6q->cfg.phy_addr;
    int             linkup = 0;
    uint16_t        sts, ana;
    nic_config_t    *cfg;
    struct ifnet    *ifp;

    if (mx6q->cfg.verbose) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s()...", __FUNCTION__);
    }

    cfg = &mx6q->cfg;
    ifp = &mx6q->ecom.ec_if;

    nic_mutex_lock(&mx6q->mii_mutex);
    sts = mx6q_mii_read_cl45 (mx6q, phy_idx, GIGE_T1_STATUS_REG, 3);
    ana = mx6q_mii_read_cl45 (mx6q, phy_idx, AUTONEG_STATUS_REG, 7);
    nic_mutex_unlock(&mx6q->mii_mutex);

    linkup = (sts & GIGE_T1_STATUS_LINKUP)
            && ((ana & AUTONEG_STATUS_RX) == AUTONEG_STATUS_RX);
    if ((cfg->flags & NIC_FLAG_LINK_DOWN) && linkup) {
        /* Link was down and is now up */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): link up", __FUNCTION__);
        }
        cfg->flags &= ~NIC_FLAG_LINK_DOWN;
        if_link_state_change(ifp, LINK_STATE_UP);
        cfg->media_rate = 1000 * 1000L;
        cfg->duplex = 1;
        mx6q_speeduplex(mx6q);
    } else if (((cfg->flags & NIC_FLAG_LINK_DOWN) == 0) && (linkup == 0)) {
        /* Link was up and is now down */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): link down", __FUNCTION__);
        }
        cfg->flags |= NIC_FLAG_LINK_DOWN;
        if_link_state_change(ifp, LINK_STATE_DOWN);
    }
}

//
// Check for link up/down on BroadR-Reach PHY
//

static void mx6_br_phy_state(mx6q_dev_t *mx6q)
{
    uint16_t    val;
    nic_config_t    *cfg = &mx6q->cfg;
    struct ifnet    *ifp = &mx6q->ecom.ec_if;

    if (mx6q->mdi->PhyData[cfg->phy_addr]->VendorOUI == NXP) {
        mx6_nxp_phy_state (mx6q);
        return;
    }

    if (mx6q->mdi->PhyData[cfg->phy_addr]->VendorOUI == TEXAS_INSTRUMENTS) {
        mx6q_ti_phy_state(mx6q);
        return;
    }

    if (mx6q->mdi->PhyData[cfg->phy_addr]->VendorOUI == MARVELLX) {
        mx6q_mv_phy_state (mx6q);
        return;
    }

    /* Link state latches low so double read to clear */
    nic_mutex_lock(&mx6q->mii_mutex);
    val = mx6q_mii_read(mx6q,  cfg->phy_addr, 1);
    val = mx6q_mii_read(mx6q,  cfg->phy_addr, 1);
    nic_mutex_unlock(&mx6q->mii_mutex);

    if ((cfg->flags & NIC_FLAG_LINK_DOWN) &&
        (val & 4)) {
        /* Link was down and is now up */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): link up", __FUNCTION__);
        }
        cfg->flags &= ~NIC_FLAG_LINK_DOWN;
        if_link_state_change(ifp, LINK_STATE_UP);
        cfg->media_rate = 100 * 1000L;
        cfg->duplex = 1;
        // if link up again then arm callout to check sqi
        if (cfg->flags & ~NIC_FLAG_LINK_DOWN)
            callout_msec(&mx6q->sqi_callout, MX6Q_SQI_SAMPLING_INTERVAL * 1000, mx6q_BRCM_SQI_Monitor, mx6q);
    } else if (((cfg->flags & NIC_FLAG_LINK_DOWN) == 0) &&
           ((val & 4) == 0)) {
        /* Link was up and is now down */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): link down", __FUNCTION__);
        }
        cfg->flags |= NIC_FLAG_LINK_DOWN;
        if_link_state_change(ifp, LINK_STATE_DOWN);
    }
}

//
// Check to see whether this is a BroadR-Reach PHY
//

int mx6_is_br_phy (mx6q_dev_t *mx6q) {
    uint32_t	PhyAddr;
    int		is_br = 0;

    PhyAddr = mx6q->cfg.phy_addr;

    switch (mx6q->mdi->PhyData[PhyAddr]->VendorOUI) {
    case BROADCOM3:
        switch(mx6q->mdi->PhyData[PhyAddr]->Model) {
        	case BCM89811:
                is_br = 1;
                break;
        }
        break;
    case BROADCOM2:
        switch (mx6q->mdi->PhyData[PhyAddr]->Model) {
            case BCM89810:
                is_br = 1;
                break;
        }
        break;
    case NXP:
        switch (mx6q->mdi->PhyData[PhyAddr]->Model) {
            case TJA1100:
            case TJA1100_1:
            case TJA1101:
                is_br = 1;
                break;
        }
        break;
    case TEXAS_INSTRUMENTS:
        switch (mx6q->mdi->PhyData[PhyAddr]->Model) {
            case DP83TC811:
                is_br = 1;
                break;
        }
        break;
    case MARVELLX:
        switch (mx6q->mdi->PhyData[PhyAddr]->Model) {
            case MV88Q2110:
                is_br = 1;
                break;
        }
        break;
    }
    return is_br;
}

void mx6q_do_mdi_monitor(void* arg)
{
    mx6q_dev_t      *mx6q   = arg;

    if (!mx6_is_br_phy(mx6q)) {
        MDI_MonitorPhy(mx6q->mdi);
    } else {
        mx6_br_phy_state(mx6q);
    }
    kthread_exit(0);

}

//
// periodically called by stack to probe phy state
// and to clean out tx descriptor ring
//
void
mx6q_MDI_MonitorPhy (void *arg)
{
    mx6q_dev_t      *mx6q   = arg;
    nic_config_t        *cfg        = &mx6q->cfg;
    struct ifnet        *ifp        = &mx6q->ecom.ec_if;

    //
    // we will probe the PHY if:
    //   the user has forced it from the cmd line, or
    //   we have not rxd any packets since the last time we ran, or
    //   the link is considered down
    //
    if (mx6q->probe_phy       ||
        !mx6q->rxd_pkts       ||
        cfg->media_rate <= 0  ||
        cfg->flags & NIC_FLAG_LINK_DOWN) {
        if (cfg->verbose > 6) {
            /* use sloginfo so it does not overwrite ERR Logs */
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,
                "%s(): calling MDI_MonitorPhy(): rx_pkts: %d Flags: 0x%x",
                __FUNCTION__, mx6q->rxd_pkts, cfg->flags);
        }
        kthread_create1(mx6q_do_mdi_monitor, arg, NULL, NULL);

    } else {
        if (cfg->verbose > 6) {
             /* use sloginfo so it does not overwrite ERR Logs */
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO,
                "%s(): NOT calling MDI_MonitorPhy()",  __FUNCTION__);
        }
    }
    mx6q->rxd_pkts = 0;  // reset for next time we are called

    //
    // Clean out the tx descriptor ring if it has not
    // been done by the start routine in the last 2 seconds
    //
    if (!mx6q->tx_reaped) {
        NW_SIGLOCK(&ifp->if_snd_ex, mx6q->iopkt);

        mx6q_transmit_complete(mx6q, 0);

        NW_SIGUNLOCK(&ifp->if_snd_ex, mx6q->iopkt);
    }
    mx6q->tx_reaped = 0;  // reset for next time we are called

    // restart timer to call us again in 2 seconds
    callout_msec(&mx6q->mii_callout, 2 * 1000, mx6q_MDI_MonitorPhy, mx6q);
}

//
// Debug: find all PHY and dump each PHY ID
//
//
void debug_dump_all_phy (mx6q_dev_t *mx6q)
{
    int        phy_idx;
    uint16_t   phy_oui_1;
    uint16_t   phy_oui_2;

    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): all PHY that can be detected", __FUNCTION__);

    // Get PHY address
    for (phy_idx = 0; phy_idx < 32; phy_idx++) {
        if (MDI_FindPhy(mx6q->mdi, phy_idx) == MDI_SUCCESS) {
            // get the PHY ID
            phy_oui_1 = mx6q_mii_read(mx6q, phy_idx, MDI_PHYID_1);
            phy_oui_2 = mx6q_mii_read(mx6q, phy_idx, MDI_PHYID_2);

            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): MDIO Address: %d ID_REG_1:0x%x  ID_REG_2:0x%x",
                __FUNCTION__, phy_idx, phy_oui_1, phy_oui_2);
        }
    }

    return;
}

//
// Scan through all 32 possible PHY addresses looking for a PHY
//
//
static int mx6_scan_phy (mx6q_dev_t *mx6q)
{
    int     phy_idx;

    if (mx6q->cfg.verbose) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "scan for PHY: Device Index %d", mx6q->cfg.device_index);

        debug_dump_all_phy (mx6q);
    }

    /* Get PHY address  - start with PHY address == 1
     *   Check PHY address == 0 (Broadcast Address) last
     */
    for (phy_idx = 1; phy_idx < 32; phy_idx++) {
        if (MDI_FindPhy(mx6q->mdi, phy_idx) == MDI_SUCCESS) {
            if (mx6q->cfg.verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "PHY found at address %d", phy_idx);
            }
            return phy_idx;
        }
    }

   /* check the IEEE Phy Broadcast Address == 0 last
     *    Only check for ENET_1 / index=0
     *    Block detection for secondary PHY
     *      - If required
     *        explicitly define the PHY address in startup hwitag
     */
    if (mx6q->cfg.device_index == 0) {
        if (MDI_FindPhy(mx6q->mdi, 0) == MDI_SUCCESS) {
            if (mx6q->cfg.verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Index 0 PHY found at Broadcast address 0");
            }
            /* the PHY was found at addr== 0 */
            return 0;
        }
    }

    if (mx6q->cfg.verbose) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Unable to detect PHY for device Index: %d",
            mx6q->cfg.device_index);
    }

    return -1;
}

static int mx6_get_phy_addr (mx6q_dev_t *mx6q)
{
    int     phy_idx;
    int     status;

    status = MDI_Register_Extended (mx6q, mx6q_mii_write, mx6q_mii_read,
                mx6q_mii_callback,
                (mdi_t **)&mx6q->mdi, NULL, 0, 0);

    if (status != MDI_SUCCESS) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "MDI_Register_Extended() failed: %d", status);
        return -1;
    }

    if (((phy_idx = mx6_scan_phy(mx6q)) != -1) || mx6q->cl45 != 0) {
        return phy_idx;
    }

    if (mx6q->cfg.verbose) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Clause 22 scan failed, scan PHY with Clause 45");
    }

    mx6q->cl45 = 1;

    return mx6_scan_phy(mx6q);
}

/* this function disables Wirespeed mode (register 0x18, shadow 7, bit 4).
 */
static int mx6_paves3_phy_init(mx6q_dev_t *mx6q)
{
    nic_config_t    *cfg    = &mx6q->cfg;
    int     phy_idx = cfg->phy_addr;
    unsigned short val;

    nic_mutex_lock(&mx6q->mii_mutex);
    mx6q_mii_write(mx6q, phy_idx, 0x18, 0x7007);
    val = mx6q_mii_read(mx6q, phy_idx, 0x18);
    val &= 0xffef;
    val |= 0x8000;
    mx6q_mii_write(mx6q, phy_idx, 0x18, val);

    /* register 0x10, bit 14 - mdi autoneg disable */
    val = mx6q_mii_read(mx6q, phy_idx, 0x10);
    val |= 0x4000;
    mx6q_mii_write(mx6q, phy_idx, 0x10, val);
    nic_mutex_unlock(&mx6q->mii_mutex);
    return 0;
}

static int mx6_paves3_twelve_inch_phy_init(mx6q_dev_t *mx6q)
{
    nic_config_t    *cfg    = &mx6q->cfg;
    int     phy_idx = cfg->phy_addr;
    unsigned short val;

    nic_mutex_lock(&mx6q->mii_mutex);
    /* set page to 0 */
    mx6q_mii_write(mx6q, phy_idx, 0x16, 0);

    /* force MDI mode, disable polarity reversal */
    val = mx6q_mii_read(mx6q, phy_idx, 0x10);
    val &= 0xff9f;
    val |= 0x2;
    mx6q_mii_write(mx6q, phy_idx, 0x10, val);

    /* Software reset */
    val = mx6q_mii_read(mx6q, phy_idx, 0x0);
    val |= 0x8000;
    mx6q_mii_write(mx6q, phy_idx, 0x0, val);
    nic_mutex_unlock(&mx6q->mii_mutex);

    return 0;
}

static void mx6_broadreach_bcm89810_phy_init(mx6q_dev_t *mx6q)
{
    int phy_idx = mx6q->cfg.phy_addr;

    /*
     * The following came from Broadcom as 89810A2_script_v2_2.vbs
     *
     * Broadcom refuse to document what exactly is going on.
     * They insist these register writes are correct despite the
     * way sometimes the same register is written back-to-back with
     * contradictory values and others are written with default values.
     * There is also much writing to undocumented registers and reserved
     * fields in documented registers.
     */
    nic_mutex_lock(&mx6q->mii_mutex);
    mx6q_mii_write(mx6q, phy_idx, 0, 0x8000); //reset

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F93);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x107F);
    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F90);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x0001);
    mx6q_mii_write(mx6q, phy_idx, 0x00, 0x3000);
    mx6q_mii_write(mx6q, phy_idx, 0x00, 0x0200);

    mx6q_mii_write(mx6q, phy_idx, 0x18, 0x0C00);

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F90);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x0000);
    mx6q_mii_write(mx6q, phy_idx, 0x00, 0x0100);

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0001);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x0027);

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x000E);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x9B52);

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x000F);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0xA04D);

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F90);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x0001);

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F92);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x9225);

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x000A);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x0323);

    /* Shut off unused clocks */
    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0FFD);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x1C3F);

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0FFE);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x1C3F);

    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F99);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x7180);
    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F9A);
    mx6q_mii_write(mx6q, phy_idx, 0x15, 0x34C0);

    if (mx6q->mii) {
        /* MII-Lite config */
        mx6q_mii_write(mx6q, phy_idx, 0x18, 0xF167);
        mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F0E);
        mx6q_mii_write(mx6q, phy_idx, 0x15, 0x0800);
        mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F9F);
        mx6q_mii_write(mx6q, phy_idx, 0x15, 0x0000);
    } else {/* RGMII config */
        mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F0E);
        mx6q_mii_write(mx6q, phy_idx, 0x15, 0x0000);
        mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0F9F);
        mx6q_mii_write(mx6q, phy_idx, 0x15, 0x0000);
        mx6q_mii_write(mx6q, phy_idx, 0x18, 0xF1E7);
    }

    /* Disable LDS, 100Mb/s, one pair */
    if (!mx6q->brmast) {
        /* Set phy to slave mode */
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "devnp-mx6x: Setting BroadrReach phy to slave");
        mx6q_mii_write(mx6q, phy_idx, 0, 0x0200);
    } else {
        /* Set phy to master mode */
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "devnp-mx6x: Setting BroadrReach phy to master");
        mx6q_mii_write(mx6q, phy_idx, 0, 0x0208);
    }
    nic_mutex_unlock(&mx6q->mii_mutex);
}

static void mx6_broadreach_bcm89811_phy_init(mx6q_dev_t *mx6q)
{
    int phy_idx = mx6q->cfg.phy_addr;
    uint16_t     slew;

    /*
     * The following came from Broadcom as 89811_script_v0_93.vbs
     *
     * Broadcom refuse to document what exactly is going on.
     * They insist these register writes are correct despite the
     * way sometimes the same register is written back-to-back with
     * contradictory values and others are written with default values.
     * There is also much writing to undocumented registers and reserved
     * fields in documented registers.
     */

    nic_mutex_lock(&mx6q->mii_mutex);
	/* begin EMI optimization */
    mx6q_mii_write(mx6q, phy_idx, 0, 0x8000); //reset
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0028);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x0C00);

    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0312);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x030B);

    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x030A);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x34C0);

    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0166);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x0020);

    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x012D);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x9B52);
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x012E);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0xA04D);

    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0123);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x00C0);

    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0154);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x81C4);


    //IOUT = &H2&    ' 10=4.0mA
    //SLEW = &H2&    ' 10=3xdefault_slew
    //MII_PAD_SETTING = SLEW + 4*IOUT
    //v = &H0000& Or MII_PAD_SETTING * 2048
    //App.WrMii PORT, &H001E&, &H0811&   '
    //App.WrMii PORT, &H001F&, v   '
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0811);
    slew = (mx6q->slew + (4 * mx6q->iout)) * 2048;
    mx6q_mii_write(mx6q, phy_idx, 0x1F, slew);

    //v = &H0064&
    //App.WrMii PORT, &H001E&, &H01D3&   '
    //App.WrMii PORT, &H001F&, v   '
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x01D3);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x0064);

    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x01C1);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0xA5F7);

    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0028);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x0400);

    /* End EMI optimization */

    /*begin LED setup portion*/
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x001D);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x3411);

    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0820);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x0401);
    /*end of LED setup*/

    /*MII config*/
    if (mx6q->rmii) {
        mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x002F);
        mx6q_mii_write(mx6q, phy_idx, 0x1F, 0xF167);
        mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0045);
        mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x0700);
    } else if (mx6q->mii) {
        mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x002F);
        mx6q_mii_write(mx6q, phy_idx, 0x1F, 0xF167);
        mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0045);
        mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x0000);
    } else {  /* ENET defaults to RGMII mode */
        mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x0045);
        mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x0000);
        mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x002F);
        mx6q_mii_write(mx6q, phy_idx, 0x1F, 0xF1E7);
    }

    /* Disable LDS, 100Mb/s, one pair */
    if (!mx6q->brmast) {
        /* Set phy to slave mode */
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): Setting BroadrReach phy to slave", __FUNCTION__);
        mx6q_mii_write(mx6q, phy_idx, 0, 0x0200);
    } else {
        /* Set phy to master mode */
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): Setting BroadrReach phy to master", __FUNCTION__);
        mx6q_mii_write(mx6q, phy_idx, 0, 0x0208);
    }
    nic_mutex_unlock(&mx6q->mii_mutex);
}

/**
 * Initialization of TI DP83TC811 BroadR-Reach PHY.
 *
 * @param   mx6q   Pointer to device data structure.
 * @return  Execution status.
 */
static int mx6q_ti_dp83tc811_phy_init(mx6q_dev_t *mx6q)
{
    int phy_addr;
    int timeout;
    uint16_t phy_data;

    if (mx6q->cfg.verbose > 3) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s()...", __FUNCTION__);
    }

    phy_addr = mx6q->cfg.phy_addr;
    /* Call PHY reset in BMCR Register 0x0000 – Basic Mode Control Register */
    mx6q_mii_write(mx6q, phy_addr, 0x0, BMCR_RESET);
    timeout = 100;
    while (timeout) {
        if (!(phy_data = mx6q_mii_read(mx6q, phy_addr, 0x0) & BMCR_RESET)) {
            break;
        }
        timeout--;
        nanospin_ns(1000);
    }

    if (!timeout) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): PHY did not come out of reset", __FUNCTION__);
        return -1;
    }

    /* Read BMSR Register 0x0001 – Basic Mode Status Register */
    phy_data = mx6q_mii_read(mx6q, phy_addr, 0x1);
    if ((phy_data & BMSR_LINK_STATUS) != BMSR_LINK_STATUS) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): PHY link not established", __FUNCTION__);
        return -1;
    }
    return 0;

}

//
// Initialization routine for the NXP TJA1100 PHY
//

static int mx6_broadreach_nxp_phy_init (mx6q_dev_t *mx6q)
{
    int         phy_addr;
    int         timeout;
    uint16_t    phy_data;

    phy_addr = mx6q->cfg.phy_addr;
    nic_mutex_lock(&mx6q->mii_mutex);

    phy_data = mx6q_mii_read(mx6q, phy_addr, INT_STATUS_REG);
    if (phy_data & PHY_INIT_FAIL) {
        phy_data = BC_RESET;
        mx6q_mii_write(mx6q, phy_addr, BASIC_CONTROL, phy_data);
        timeout = 100;
        while (timeout) {
            if (!(phy_data = mx6q_mii_read(mx6q, phy_addr, BASIC_CONTROL) & BC_RESET)) {
                break;
            }
            timeout--;
            nanospin_ns(1000);
        }
        if (!timeout) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): PHY did not come out of reset", __FUNCTION__);
            nic_mutex_unlock(&mx6q->mii_mutex);
            return -1;
        }
        phy_data = mx6q_mii_read(mx6q, phy_addr, INT_STATUS_REG);
    }
    if (phy_data & PHY_INIT_FAIL) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): PHY initialization failed", __FUNCTION__);
        nic_mutex_unlock(&mx6q->mii_mutex);
        return -1;
    }

    phy_data = mx6q_mii_read(mx6q, phy_addr, EXT_CONTROL_REG);
    phy_data |= CONFIG_ENABLE;
    mx6q_mii_write(mx6q, phy_addr, EXT_CONTROL_REG, phy_data);
    nic_delay(2);

    phy_data = mx6q_mii_read(mx6q, phy_addr, CONFIG_REG_1);

    /* brmast = 2 means take the default hardware setting */
    if (mx6q->brmast == 2) {
        if (phy_data & MASTER_SLAVE ) {
            mx6q->brmast = 1; /* hardware setting is Master. */
            if (mx6q->cfg.verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): use default PHY master mode", __FUNCTION__);
            }
        } else {
            mx6q->brmast = 0; /* hardware setting is slave. */
            if (mx6q->cfg.verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): use default PHY slave mode", __FUNCTION__);
            }
        }
    }

    phy_data &= ~(MASTER_SLAVE | AUTO_OP | (REV_MII_MODE << MII_MODE_SHIFT));
    if (mx6q->brmast) {
        phy_data |= MASTER_SLAVE;
        if (mx6q->cfg.verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): PHY set to master mode", __FUNCTION__);
        }
    } else {
        if (mx6q->cfg.verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): PHY set to slave mode", __FUNCTION__);
        }
    }
    if (mx6q->rmii) {
        phy_data |= (RMII_MODE_25 << MII_MODE_SHIFT);
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s: Setting 25Mhz RMII mode", __func__);
    }
    mx6q_mii_write(mx6q, phy_addr, CONFIG_REG_1, phy_data);

    phy_data = mx6q_mii_read(mx6q, phy_addr, CONFIG_REG_2);
    phy_data |= JUMBO_ENABLE;
    mx6q_mii_write(mx6q, phy_addr, CONFIG_REG_2, phy_data);

    phy_data = ((NORMAL_MODE << POWER_MODE_SHIFT) | CONFIG_INH | WAKE_REQUEST);
    mx6q_mii_write(mx6q, phy_addr, EXT_CONTROL_REG, phy_data);
    nic_delay(10);
    phy_data &= ~WAKE_REQUEST;
    phy_data |= LINK_CONTROL;
    mx6q_mii_write(mx6q, phy_addr, EXT_CONTROL_REG, phy_data);
    nic_delay(5);
    nic_mutex_unlock(&mx6q->mii_mutex);
    return 0;
}

//
// Initialization routine for the NXP TJA1101 PHY
//

static int mx6_broadreach_nxp_tja1101_phy_init (mx6q_dev_t *mx6q)
{
    int         phy_addr;
    int         timeout;
    uint16_t    phy_data;

    phy_addr = mx6q->cfg.phy_addr;
    nic_mutex_lock(&mx6q->mii_mutex);

    phy_data = mx6q_mii_read(mx6q, phy_addr, INT_STATUS_REG);
    if (phy_data & PHY_INIT_FAIL) {
        phy_data = BC_RESET;
        mx6q_mii_write(mx6q, phy_addr, BASIC_CONTROL, phy_data);
        timeout = 100;
        while (timeout) {
            if (!(phy_data = mx6q_mii_read(mx6q, phy_addr, BASIC_CONTROL) & BC_RESET)) {
                break;
            }
            timeout--;
            nanospin_ns(1000);
        }
        if (!timeout) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): PHY did not come out of reset", __FUNCTION__);
            nic_mutex_unlock(&mx6q->mii_mutex);
            return -1;
        }
        phy_data = mx6q_mii_read(mx6q, phy_addr, INT_STATUS_REG);
    }
    if (phy_data & PHY_INIT_FAIL) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): PHY initialization failed", __FUNCTION__);
        nic_mutex_unlock(&mx6q->mii_mutex);
        return -1;
    }

    phy_data = mx6q_mii_read(mx6q, phy_addr, EXT_CONTROL_REG);
    phy_data |= CONFIG_ENABLE;
    mx6q_mii_write(mx6q, phy_addr, EXT_CONTROL_REG, phy_data);
    nic_delay(2);

    phy_data = mx6q_mii_read(mx6q, phy_addr, CONFIG_REG_1);

    /* brmast = 2 means take the default hardware setting */
    if (mx6q->brmast == 2) {
        if (phy_data & MASTER_SLAVE ) {
            mx6q->brmast = 1; /* hardware setting is Master. */
            if (mx6q->cfg.verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): use default PHY master mode", __FUNCTION__);
            }
        } else {
            mx6q->brmast = 0; /* hardware setting is slave. */
            if (mx6q->cfg.verbose) {
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): use default PHY slave mode", __FUNCTION__);
            }
        }
    }

    /* AUTO_OP has been moved to REG0x1B for TJA1101 */
    phy_data &= ~(MASTER_SLAVE | (REV_MII_MODE << MII_MODE_SHIFT));
    if (mx6q->brmast) {
        phy_data |= MASTER_SLAVE;
        if (mx6q->cfg.verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): PHY set to master mode", __FUNCTION__);
        }
    } else {
        if (mx6q->cfg.verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): PHY set to slave mode", __FUNCTION__);
        }
    }
    if (mx6q->rmii) {
        phy_data |= (RMII_MODE_25 << MII_MODE_SHIFT);
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s: Setting 25Mhz RMII mode", __func__);
    }
    mx6q_mii_write(mx6q, phy_addr, CONFIG_REG_1, phy_data);

	// Config AUTO_OP and CONFIG_INH due to the change between TJA1100 and TJA1101
    phy_data = mx6q_mii_read(mx6q, phy_addr, COMMON_CONFIG_REG);
    phy_data &= ~(AUTO_OP_TJA1101);
    phy_data |= CONFIG_INH_TJA1101;
    mx6q_mii_write(mx6q, phy_addr, COMMON_CONFIG_REG, phy_data);


    phy_data = mx6q_mii_read(mx6q, phy_addr, CONFIG_REG_2);
    phy_data |= JUMBO_ENABLE;
    mx6q_mii_write(mx6q, phy_addr, CONFIG_REG_2, phy_data);

    // CONFIG_INH has been moved to REG 0x1B for TJA1101
	phy_data = ((NORMAL_MODE << POWER_MODE_SHIFT) | WAKE_REQUEST);
    mx6q_mii_write(mx6q, phy_addr, EXT_CONTROL_REG, phy_data);

    nic_delay(10);
    phy_data &= ~WAKE_REQUEST;
    phy_data |= LINK_CONTROL;
    mx6q_mii_write(mx6q, phy_addr, EXT_CONTROL_REG, phy_data);
    nic_delay(5);
    nic_mutex_unlock(&mx6q->mii_mutex);
    return 0;
}

static void mx6q_broadreach_mv88q211x_phy_init(mx6q_dev_t *mx6q)
{
    int         phy_idx = mx6q->cfg.phy_addr;
    uint16_t    regval;

    nic_mutex_lock(&mx6q->mii_mutex);

    // IO Control register, deviec 31, register 0x8001
    // Set Tx/Rx delay
    // bit 15, RGMII transmit clock internally delayed
    // bit 14, RGMII receive clock transition when data stable
    regval = mx6q_mii_read_cl45 (mx6q, phy_idx, 0x8001, 31);
    regval |= 0xC000;
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x8001, 31, regval);

    // RGMII Output Impedance register, device 31, register 0x8000
    regval = mx6q_mii_read_cl45 (mx6q, phy_idx, 0x8000, 31);
    regval = (regval & ~(0x0F << 8)) | (0x0c << 8); // PMOS
    regval = (regval & ~(0x0F << 4)) | (0x0c << 4); // NMOS
    regval |= 1 << 12;  // Force PMOS/NMOS
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x8000, 31, regval);

    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFFE4, 3, 0x06B6);

    // BASE-T1 Auto-negotiation register, device 7, register 0x200
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x0200, 7, 0x0000);

    // BASE-T1 PMA/PMD Control register
    regval = mx6q_mii_read_cl45 (mx6q, phy_idx, 0x0834, 1);
    regval = (regval & ~(0x00F | (1 << 14))) | 1;
    if (mx6q->brmast) regval |= 1 << 14;    // Master
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x0834, 1, regval);
    nic_delay(5);

    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFFDE, 3, 0x402F);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x8032, 7, 0x0064);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x8031, 7, 0x0A01);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x8031, 7, 0x0C01);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFE0F, 3, 0x0000);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x800C, 3, 0x0008);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFE2A, 3, 0x3C3D);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x801D, 3, 0x0800); // LPSD Register 2, bit 11, Disable LPSD remote wakeup
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC00, 3, 0x01C0);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC17, 3, 0x0425);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC94, 3, 0x5470);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC95, 3, 0x0055);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC19, 3, 0x08D8);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC1A, 3, 0x0110);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC1B, 3, 0x0A10);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC3A, 3, 0x2725);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC61, 3, 0x2627);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC3B, 3, 0x1612);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC62, 3, 0x1C12);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC9D, 3, 0x6367);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC9E, 3, 0x8060);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC00, 3, 0x01C8);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x8000, 3, 0x0000); // Reset and Control register
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x8016, 3, 0x0011); // Function Control register

    /* softReset */
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC03, 3, 0x03E0);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC04, 3, 0x0000);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC03, 3, 0x03E1);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC04, 3, 0x0000);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC03, 3, 0x0420);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC04, 3, 0x0000);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC03, 3, 0x0421);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC04, 3, 0x0000);
    // PCS Control register, bit 15, Software reset
    regval = mx6q_mii_read_cl45 (mx6q, phy_idx, 0x0900, 3);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x0900, 3, regval | 0x8000);
    nic_delay(5);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC03, 3, 0x03E1);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC04, 3, 0x0099);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC03, 3, 0x03E1);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC04, 3, 0x0009);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC03, 3, 0x0420);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC04, 3, 0x00CC);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC03, 3, 0x0421);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFC04, 3, 0x000C);
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0xFFE4, 3, 0x000C);

    /* Soft reset RGMII interface */
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x8000, 3, 0xCC00); // Reset and Control register
    nic_delay(5);
    /* Reset bit doesn't self-clear, so clear it */
    mx6q_mii_write_cl45 (mx6q, phy_idx, 0x8000, 3, 0x0000); // Reset and Control register

    nic_mutex_unlock(&mx6q->mii_mutex);
}

//
// Initialization routine for all BroadR-Reach PHYs
//

void mx6_phy_init (mx6q_dev_t *mx6q)
{
    nic_config_t    *cfg;

    cfg = &mx6q->cfg;

    switch (mx6q->mdi->PhyData[cfg->phy_addr]->VendorOUI) {
        case BROADCOM2:
            switch(mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
                case BCM89810:
                    if (cfg->verbose > 3) {
                        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected BCM89810");
                    }
                    mx6_broadreach_bcm89810_phy_init(mx6q);
                    break;
                default:
                    break;
            }
        break;
    case BROADCOM3:
        switch(mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
            case BCM89811:
                if (cfg->verbose > 3) {
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected BCM89811");
                }
                mx6_broadreach_bcm89811_phy_init(mx6q);
                break;
            default:
                break;
        }
    break;
    case NXP:
        switch(mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
            case TJA1100:
            case TJA1100_1:
                if (cfg->verbose > 3)
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected NXP TJA1100 PHY");
                mx6_broadreach_nxp_phy_init (mx6q);
                break;
            case TJA1101:
                if (cfg->verbose > 3)
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected NXP TJA1101 PHY");
                mx6_broadreach_nxp_tja1101_phy_init (mx6q);
                break;
            default:
                break;
        }
    break;
    case TEXAS_INSTRUMENTS:
        switch (mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
            case DP83TC811:
                if (cfg->verbose > 3)
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected DP83TC811 PHY");
                mx6q_ti_dp83tc811_phy_init(mx6q);
            break;
        default:
            break;
        }
        break;
    case MARVELLX:
        switch (mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
            case MV88Q2110:
                if (cfg->verbose > 3)
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected Marvell 88Q2110 PHY");
                mx6q_broadreach_mv88q211x_phy_init(mx6q);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
}

static void mmd_write_reg(mx6q_dev_t *mx6q, int device, int reg, int val)
{
    nic_config_t *cfg = &mx6q->cfg;
    int     phy_idx = cfg->phy_addr;

    mx6q_mii_write(mx6q, phy_idx, 0x0d, device);
    mx6q_mii_write(mx6q, phy_idx, 0x0e, reg);
    mx6q_mii_write(mx6q, phy_idx, 0x0d, (1 << 14) | device);
    mx6q_mii_write(mx6q, phy_idx, 0x0e, val);
}

static int mx6_ksz9031_phy_init(mx6q_dev_t *mx6q)
{
    nic_config_t    *cfg    = &mx6q->cfg;
    int             phy_idx = cfg->phy_addr;

    nic_mutex_lock(&mx6q->mii_mutex);
    // master mode (when possible), 1000 Base-T capable
    mx6q_mii_write(mx6q, phy_idx, 0x9, 0x0f00);

    /*
     * min rx data delay, max rx/tx clock delay,
     * min rx/tx control delay
     */
    mmd_write_reg(mx6q, 2, 4, 0);
    mmd_write_reg(mx6q, 2, 5, 0);
    mmd_write_reg(mx6q, 2, 8, 0x003ff);
    nic_mutex_unlock(&mx6q->mii_mutex);

    return 0;
}

/**
 * Specific initialization of Microchip KSZ8081 PHY.
 *
 * @param   mx6q    Pointer to a structure containing data of the ENET device.
 */
void mx6q_ksz8081_phy_init(mx6q_dev_t *mx6q)
{
    nic_config_t *cfg     = &mx6q->cfg;
    int           phy_idx = cfg->phy_addr;

    nic_mutex_lock(&mx6q->mii_mutex);
    /*
     * Write PHY Control 2 register to enable RMII 50MHz mode and select
     * LED0: Link status, LED1: Activity.
     */
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x8190);
    nic_mutex_unlock(&mx6q->mii_mutex);
}

static int mx6_sabrelite_phy_init(mx6q_dev_t *mx6q)
{
    nic_config_t    *cfg    = &mx6q->cfg;
    int     phy_idx = cfg->phy_addr;

    nic_mutex_lock(&mx6q->mii_mutex);
    // 1000 Base-T full duplex capable, single port device
    mx6q_mii_write(mx6q, phy_idx, 0x9, 0x0600);

    // min rx data delay
    mx6q_mii_write(mx6q, phy_idx, 0xb, 0x8105);
    mx6q_mii_write(mx6q, phy_idx, 0xc, 0x0000);

    // max rx/tx clock delay, min rx/tx control delay
    mx6q_mii_write(mx6q, phy_idx, 0xb, 0x8104);
    mx6q_mii_write(mx6q, phy_idx, 0xc, 0xf0f0);
    mx6q_mii_write(mx6q, phy_idx, 0xb, 0x104);
    nic_mutex_unlock(&mx6q->mii_mutex);

    return 0;
}

static int mx6_sabreauto_rework(mx6q_dev_t *mx6q)
{
    nic_config_t        *cfg        = &mx6q->cfg;
    int                 phy_idx     = cfg->phy_addr;
    unsigned short val;

    nic_mutex_lock(&mx6q->mii_mutex);
    /* To enable AR8031 ouput a 125MHz clk from CLK_25M */
    mx6q_mii_write(mx6q, phy_idx, 0xd, 0x7);
    mx6q_mii_write(mx6q, phy_idx, 0xe, 0x8016);
    mx6q_mii_write(mx6q, phy_idx, 0xd, 0x4007);
    val = mx6q_mii_read(mx6q, phy_idx, 0xe);

    val &= 0xffe3;
    val |= 0x18;
    mx6q_mii_write(mx6q, phy_idx, 0xe, val);

    /* introduce tx clock delay */
    mx6q_mii_write(mx6q, phy_idx, 0x1d, 0x5);
    val = mx6q_mii_read(mx6q, phy_idx, 0x1e);
    val |= 0x0100;
    mx6q_mii_write(mx6q, phy_idx, 0x1e, val);

    /*
     * Disable SmartEEE
     * The Tx delay can mean late pause and bad timestamps.
     */
    mx6q_mii_write(mx6q, phy_idx, 0xd, 0x3);
    mx6q_mii_write(mx6q, phy_idx, 0xe, 0x805d);
    mx6q_mii_write(mx6q, phy_idx, 0xd, 0x4003);
    val = mx6q_mii_read(mx6q, phy_idx, 0xe);
    val &= ~(1 << 8);
    mx6q_mii_write(mx6q, phy_idx, 0xe, val);

    /* As above for EEE (802.3az) */
    mx6q_mii_write(mx6q, phy_idx, 0xd, 0x7);
    mx6q_mii_write(mx6q, phy_idx, 0xe, 0x3c);
    mx6q_mii_write(mx6q, phy_idx, 0xd, 0x4007);
    mx6q_mii_write(mx6q, phy_idx, 0xd,0);
    nic_mutex_unlock(&mx6q->mii_mutex);

    return 0;
}

static void bcm54220_phy_init(mx6q_dev_t *mx6q)
{
    nic_config_t *cfg     = &mx6q->cfg;
    int           phy_idx = cfg->phy_addr;

    /* Write register 0x21 to select Link up on copper side */
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x21);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x7CA8); //0x7EA8

    /* Write register 0x2F to enable RXC skew */
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x2F);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, 0x71B7);
}


int
mx6q_init_phy(mx6q_dev_t *mx6q)
{
    int			status;
    nic_config_t	*cfg;
    struct ifnet	*ifp;

    cfg = &mx6q->cfg;
    ifp = &mx6q->ecom.ec_if;

    /*
     *  Register and poll for PHY - also has debug prints
     */
    if ((status = mx6_get_phy_addr(mx6q)) == -1) {
        /* MDI Register Extended failed */
        return -1;
    }

    if ((cfg->phy_addr & HWITAG_PHY_ADDR_METADATA_MASK)
                     == HWITAG_IS_AN_ON_BOARD_PHY_ADDR) {
        /* the hwitag explicitly defines the expected Phy Address */
        /* the code does not need to use the discovered address   */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "using phyAddr from the hwitag: 0x%x",
                cfg->phy_addr);
        }

        /* remove the extra information from the Phy Address */
        cfg->phy_addr &= HWITAG_PHY_ADDR_MASK;
    } else if ((cfg->phy_addr & CONFIG_PHY_ADDR_METADATA_MASK)
                                 == CONFIG_HAS_DEFINED_PHY_ADDR) {
        /* the config explicitly defined the expected Phy Address */
        /* the code does not need to use the discovered address   */
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "using phyAddr from the command line: 0x%x",
                cfg->phy_addr);
        }

        /* remove the extra information from the Phy Address */
        cfg->phy_addr &= HWITAG_PHY_ADDR_MASK;
    } else {
        /* use the discovered Phy Address */
        cfg->phy_addr = status;

        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "using discovered phyAddr: %d for lan: %d",
                cfg->phy_addr,
                cfg->lan);
        }
    }

    // register callbacks with MII managment library
    callout_init(&mx6q->mii_callout);

    // Register callbacks with SQI calculation
    callout_init(&mx6q->sqi_callout);

    status = MDI_InitPhy(mx6q->mdi, cfg->phy_addr);
    if (status != MDI_SUCCESS) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "Failed to init PHY: %d status: %d",
            cfg->phy_addr,
            status);
        return -1;
    }

    if (mx6_is_br_phy(mx6q)) {
        mx6_phy_init(mx6q);
    } else {
        status = MDI_ResetPhy(mx6q->mdi, cfg->phy_addr, WaitBusy);
        if (status != MDI_SUCCESS) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "Failed to reset PHY: %d status: %d",
                cfg->phy_addr,
                status);
            return -1;
        }

        switch (mx6q->mdi->PhyData[cfg->phy_addr]->VendorOUI) {
        case ATHEROS:
            switch (mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
            case AR8031:
                if (cfg->verbose > 3)
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected Atheros AR8031 PHY");
                mx6_sabreauto_rework(mx6q);
                break;
            default:
                break;
            }
            break;

        case KENDIN:
            switch (mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
            case KSZ9021:
                if (cfg->verbose > 3)
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected Micrel KSZ9021 PHY");
                mx6_sabrelite_phy_init(mx6q);
                break;
            case KSZ9031:
                if (cfg->verbose > 3)
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected Micrel KSZ9031 PHY");
                mx6_ksz9031_phy_init (mx6q);
                break;
            case KSZ8081:
                if (cfg->verbose > 3)
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected Micrel KSZ8081 PHY");
                mx6q_ksz8081_phy_init(mx6q);
                break;
            default:
                break;
            }
            break;

        case BROADCOM2:
            switch (mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
            case BCM54616:
                if (cfg->verbose > 3) {
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected Broadcom BCM54616 PHY");
                }
                mx6_paves3_phy_init(mx6q);
                break;
            case BCM89810:
                if (cfg->verbose > 3) {
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected Broadcom BCM89810 PHY");
                }
                break;
            default:
                break;
            }
            break;

        case BROADCOM4:
            switch (mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
            case BCM54220:
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected Broadcom BCM54220 PHY");
                bcm54220_phy_init(mx6q);
                break;
            default:
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_WARNING, "%s(): Unknown PHY detected\n", __FUNCTION__);
                break;
            }
            break;

        case MARVELL:
            switch (mx6q->mdi->PhyData[cfg->phy_addr]->Model) {
            case ALASKA:
                if (cfg->verbose > 3)
                    slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "Detected Marvell PHY");
                mx6_paves3_twelve_inch_phy_init(mx6q);
                break;
            default:
                break;
            }
            break;

        default:
            break;
        }
        MDI_PowerdownPhy(mx6q->mdi, cfg->phy_addr);
    }

    cfg->flags |= NIC_FLAG_LINK_DOWN;
    if_link_state_change(ifp, LINK_STATE_DOWN);

    return 0;
}

void
mx6q_fini_phy(mx6q_dev_t *mx6q)
{
    MDI_DeRegister(&mx6q->mdi);
    mx6q->mdi = NULL;
}

void mx6q_do_brcm_sqi_monitor(void *arg)
{
    mx6q_dev_t      *mx6q   = arg;
    mx6_mii_sqi(mx6q);
    kthread_exit(0);
}

//
// periodically called by stack to probe sqi
//
void
mx6q_BRCM_SQI_Monitor (void *arg)
{
    mx6q_dev_t      *mx6q   = arg;
    nic_config_t        *cfg    = &mx6q->cfg;

    if (cfg->flags & NIC_FLAG_LINK_DOWN) {
        if (cfg->verbose) {
            slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): link down, clear sqi sample buffer:  \n", __FUNCTION__);
        }
        mx6_clear_sample();
    }
    else {
        kthread_create1( mx6q_do_brcm_sqi_monitor, arg, NULL, NULL);
        // restart timer to call us again in MX6Q_SQI_SAMPLING_INTERVAL seconds
        callout_msec(&mx6q->sqi_callout, MX6Q_SQI_SAMPLING_INTERVAL * 1000, mx6q_BRCM_SQI_Monitor, mx6q);
    }
}

/*read the SQI register*/
static void mx6_mii_sqi_bcm89810(mx6q_dev_t *mx6q)
{
    /*
    follow the 2 steps to read SQI register.
    1. Set up register access:
        Write 0C00h to register 18h
        Write 0002h to register 17h
    2. Read register 15h

    Refer to the datasheet, April 20, 2012 - 89810-DS03-R - Table 10, page 50
    */
    unsigned short val = 0;
    nic_config_t    *cfg = &mx6q->cfg;
    int     phy_idx = cfg->phy_addr;

    nic_mutex_lock(&mx6q->mii_mutex);
    mx6q_mii_write(mx6q, phy_idx, 0x18, 0x0C00);
    mx6q_mii_write(mx6q, phy_idx, 0x17, 0x0002);
    val = mx6q_mii_read(mx6q, phy_idx, 0x15);
    nic_mutex_unlock(&mx6q->mii_mutex);
    if (cfg->verbose > 5) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): SQI read val:  %d\n", __FUNCTION__, val);
    }

    if (val > 0)
        mx6q->sqi = mx6_calculate_sqi(val);

    if (cfg->verbose > 5) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): sqi = :  %d\n",  __FUNCTION__, mx6q->sqi);
    }
}

/*read the SQI register*/
static void mx6_mii_sqi_bcm89811(mx6q_dev_t *mx6q)
{
    /*
    follow the 2 steps to read SQI register.
    1. Set up register access:
        read-modify-write to set bit 11 = 1 in RDB register 0x028
    2. Read RDB register 0x108

    Refer to the datasheet, April 20, 2012 - 89810-DS03-R - Table 10, page 50
    */
    unsigned short val = 0;
    nic_config_t    *cfg = &mx6q->cfg;
    int     phy_idx = cfg->phy_addr;

    nic_mutex_lock(&mx6q->mii_mutex);
    //read RDB register 0x28
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x28);
    val = mx6q_mii_read(mx6q, phy_idx, 0x1F);
    //write RDB register with bit 11=1
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x28);
    mx6q_mii_write(mx6q, phy_idx, 0x1F, val|0x800);

    //read RDB register 0x108
    mx6q_mii_write(mx6q, phy_idx, 0x1E, 0x108);
    val = mx6q_mii_read(mx6q, phy_idx, 0x1F);
    nic_mutex_unlock(&mx6q->mii_mutex);

    if (cfg->verbose > 5) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): SQI read val:  %d\n", __FUNCTION__, val);
    }

    if (val > 0)
        mx6q->sqi = mx6_calculate_sqi(val);

    if (cfg->verbose > 5) {
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): sqi = :  %d\n",  __FUNCTION__, mx6q->sqi);
    }
}

/*read the SQI register*/
void mx6_mii_sqi(mx6q_dev_t *mx6q)
{
    nic_config_t       *cfg = &mx6q->cfg;

    if ((mx6q->mdi->PhyData[cfg->phy_addr]->VendorOUI == BROADCOM2) &&
        (mx6q->mdi->PhyData[cfg->phy_addr]->Model == BCM89810)) {
            mx6_mii_sqi_bcm89810(mx6q);
    }
    else if ((mx6q->mdi->PhyData[cfg->phy_addr]->VendorOUI == BROADCOM3) &&
        (mx6q->mdi->PhyData[cfg->phy_addr]->Model == BCM89811)){
            mx6_mii_sqi_bcm89811(mx6q);
    }
    else
        slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): unknown PHY, SQI not availiable.  \n", __FUNCTION__);
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devnp/mx6x/mii.c $ $Rev: 910691 $")
#endif
