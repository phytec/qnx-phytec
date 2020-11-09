/*
 * $QNXLicenseC: 
 * Copyright 2010, QNX Software Systems.  
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

#include <net/ifdrvcom.h>
#include <sys/sockio.h>
#include <hw/imx6x_devnp_ioctl.h>
#include <netdrvr/avb.h>

static void
mx6q_get_stats(mx6q_dev_t *mx6q, void *data)
{
	nic_stats_t				*stats = data;

	// read nic hardware registers into mx6q data struct stats 
	mx6q_update_stats(mx6q);

	// copy it to the user buffer
	memcpy(stats, &mx6q->stats, sizeof(mx6q->stats));
}

int
mx6q_ioctl(struct ifnet *ifp, unsigned long cmd, caddr_t data)
{
	mx6q_dev_t			*mx6q = ifp->if_softc;
	int				error = 0;
	struct ifdrv_com		*ifdc;
#ifndef SWITCHMODE
	struct ifreq			*ifr;
#endif
	struct ifdrv			*ifd;
	struct drvcom_config		*dcfgp;
	struct drvcom_stats		*dstp;


	switch (cmd) {
	case SIOCGDRVCOM:
		ifdc = (struct ifdrv_com *)data;
		switch (ifdc->ifdc_cmd) {
		case DRVCOM_CONFIG:
			dcfgp = (struct drvcom_config *)ifdc;

			if (ifdc->ifdc_len != sizeof(nic_config_t)) {
				error = EINVAL;
				break;
			}
			memcpy(&dcfgp->dcom_config, &mx6q->cfg, sizeof(mx6q->cfg));
			break;

		case DRVCOM_STATS:
			dstp = (struct drvcom_stats *)ifdc;

			if (ifdc->ifdc_len != sizeof(nic_stats_t)) {
				error = EINVAL;
				break;
			}
			mx6q_get_stats(mx6q, &dstp->dcom_stats);
			break;

		default:
			error = EOPNOTSUPP;
			break;

		}
		break;


	case SIOCSIFMEDIA:
		if (mx6_is_br_phy(mx6q)) {
			error = ENOTTY;
			break;
		}
		/* Fall through for normal PHY */
	case SIOCGIFMEDIA:
#ifndef SWITCHMODE
		ifr = (struct ifreq *)data;
		struct ifmediareq *ifmr = (struct ifmediareq *) ifr;

		if (mx6_is_br_phy(mx6q)) {
			switch (mx6q->cfg.media_rate) {
				case    100 * 1000:
					ifmr->ifm_current = IFM_ETHER | IFM_FDX | IFM_100_T1;
					break;
				case    1000 * 1000:
					ifmr->ifm_current = IFM_ETHER | IFM_FDX | IFM_1000_T1;
					break;
				default:
					ifmr->ifm_current = IFM_ETHER | IFM_NONE;
					break;
			}
			if (mx6q->brmast) {
				/* BroadR-Reach is assuming a master role (far-end phy is slave) */
				ifmr->ifm_current |= IFM_ETH_MASTER;
			}
			ifmr->ifm_active = ifmr->ifm_current;
			ifmr->ifm_count = 0;
			ifmr->ifm_status = IFM_AVALID;
			if ((mx6q->cfg.flags & NIC_FLAG_LINK_DOWN) == 0) {
				ifmr->ifm_status |= IFM_ACTIVE;
			}
		} else {
			error = ifmedia_ioctl(ifp, ifr,
						&mx6q->bsd_mii.mii_media, cmd);
		}
#else
		error = EOPNOTSUPP;
#endif
		break;


	case SIOCSDRVSPEC:
	case SIOCGDRVSPEC:
		ifd = (struct ifdrv *)data;

		switch (ifd->ifd_cmd) {
#if defined MX6XSLX || defined MX7D || defined MX8XP
		case AVB_SET_BW:
			error = mx6q_set_tx_bw(mx6q, ifd);
			break;
#endif
		case PRECISE_TIMER_DELAY:
			error = mx6q_timer_delay(mx6q, ifd);
			break;

		case GET_BRCM_SQI:
			error = mx6q_sqi_ioctl(mx6q, ifd);
			break;

		case READ_BRCM_MII:
			error = mx6q_mii_read_ioctl(mx6q, ifd);
			break;

		case WRITE_BRCM_MII:
			error = mx6q_mii_write_ioctl(mx6q, ifd);
			break;

		case ENABLE_BRCM_PHY_LOWPOWER:
			error = mx6q_br_lowpower_enable_ioctl(mx6q, ifd);
			break;

		case DISABLE_BRCM_PHY_LOWPOWER:
			error = mx6q_br_lowpower_disable_ioctl(mx6q, ifd);
			break;

		case GENERIC_MII_OP:
				error = mx6q_mii_op_ioctl(mx6q, ifd);
				break;
		case READ_PHY_REG:
		case WRITE_PHY_REG:
				error = mx6q_phy_funcs(mx6q, ifd);
				break;

		default:
			error = mx6q_ptp_ioctl(mx6q, ifd);
			break;
		}
		break;

	default:
		error = ether_ioctl(ifp, cmd, data);
		if (error == ENETRESET) {
			//
			// Multicast list has changed; set the
			// hardware filter accordingly.
			//
			if ((ifp->if_flags_tx & IFF_RUNNING) == 0) {
				//
				// interface is currently down: mx6q_init() 
				// will call mx6q_set_multicast() so
				// nothing to do
				//
			} else {
				//
				// interface is up, recalc and hit gaddrs
				//
				mx6q_set_multicast(mx6q);
			}
			error = 0;
		}
		break;
	}

	return error;
}

int mx6q_sqi_ioctl(mx6q_dev_t *mx6q, struct ifdrv *ifd)
{
		uint8_t		sqi = mx6q->sqi;

    if (!mx6_is_br_phy(mx6q)) {
		return ENODEV;
	}
	if (ifd->ifd_len != sizeof(sqi)) {
		return EINVAL;
	}
	if (ISSTACK) {
		return (copyout(&sqi, (((uint8_t *)ifd) + sizeof(*ifd)),
					sizeof(sqi)));
	} else {
		memcpy((((uint8_t *)ifd) + sizeof(*ifd)), &sqi, sizeof(sqi));
		return EOK;
	}

		return ENOTTY;
}

int mx6q_mii_read_ioctl(mx6q_dev_t *mx6q, struct ifdrv *ifd)
{
	mx6q_mii_request_t mii_read;
	nic_config_t	*cfg = &mx6q->cfg;
	int				phy_idx = cfg->phy_addr;

	if (mx6q->cl45) {
		return ENOTSUP;
	}
	if (!mx6_is_br_phy(mx6q)) {
		return ENODEV;
	}
	if (ifd->ifd_len != sizeof(mii_read)) {
		return EINVAL;
	}

	if (ISSTACK) {
		if (copyin((((uint8_t *)ifd) + sizeof(*ifd)),
			&mii_read, sizeof(mii_read))) {
				return EINVAL;
		}
	} else {
		memcpy(&mii_read, (((uint8_t *)ifd) + sizeof(*ifd)), sizeof(mii_read));
	}

    nic_mutex_lock(&mx6q->mii_mutex);
	mii_read.data = mx6q_mii_read(mx6q, phy_idx, mii_read.address);
    nic_mutex_unlock(&mx6q->mii_mutex);

	if (cfg->verbose > 5) {
		slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): mx6q_mii_read_ioctl read val:  %d\n", __FUNCTION__, mii_read.data);
	}

	if (ISSTACK) {
		return (copyout(&mii_read, (((uint8_t *)ifd) + sizeof(*ifd)),
					sizeof(mii_read)));
	} else {
		memcpy((((uint8_t *)ifd) + sizeof(*ifd)), &mii_read, sizeof(mii_read));
		return EOK;
	}

	return ENOTTY;
}

int mx6q_mii_write_ioctl(mx6q_dev_t *mx6q, struct ifdrv *ifd)
{
	mx6q_mii_request_t mii_write;
	nic_config_t	*cfg = &mx6q->cfg;
	int				phy_idx = cfg->phy_addr;

	if (mx6q->cl45) {
		return ENOTSUP;
	}
	if (!mx6_is_br_phy(mx6q)) {
		return ENODEV;
	}
	if (ifd->ifd_len != sizeof(mii_write)) {
		return EINVAL;
	}

	if (ISSTACK) {
		if (copyin((((uint8_t *)ifd) + sizeof(*ifd)),
			&mii_write, sizeof(mii_write))) {
				return EINVAL;
		}
	} else {
		memcpy(&mii_write, (((uint8_t *)ifd) + sizeof(*ifd)), sizeof(mii_write));
	}

    nic_mutex_lock(&mx6q->mii_mutex);
	mx6q_mii_write(mx6q, phy_idx, mii_write.address, mii_write.data);
    nic_mutex_unlock(&mx6q->mii_mutex);

	return EOK;
}

int mx6q_br_lowpower_enable_ioctl(mx6q_dev_t *mx6q, struct ifdrv *ifd)
{
	nic_config_t	*cfg = &mx6q->cfg;
	int				phy_idx = cfg->phy_addr;
	uint16_t		val = 0;

	if (mx6q->cl45) {
		return ENOTSUP;
	}
	if (!mx6_is_br_phy(mx6q)) {
		return ENODEV;
	}

    nic_mutex_lock(&mx6q->mii_mutex);
	val = mx6q_mii_read(mx6q, phy_idx, 0x0);
	val |= 0x800; /*set bit 11 to enable power down*/
	mx6q_mii_write(mx6q, phy_idx, 0x0, val);
    nic_mutex_unlock(&mx6q->mii_mutex);

	return EOK;
}

int mx6q_br_lowpower_disable_ioctl(mx6q_dev_t *mx6q, struct ifdrv *ifd)
{
	nic_config_t	*cfg = &mx6q->cfg;
	int				phy_idx = cfg->phy_addr;
	uint16_t		val = 0;

	if (mx6q->cl45) {
		return ENOTSUP;
	}
	if (!mx6_is_br_phy(mx6q)) {
		return ENODEV;
	}

    nic_mutex_lock(&mx6q->mii_mutex);
	val = mx6q_mii_read(mx6q, phy_idx, 0x0);
	val &= 0xF7FF; /*clear bit 11 to disable power down*/
	mx6q_mii_write(mx6q, phy_idx, 0x0, val);
    nic_mutex_unlock(&mx6q->mii_mutex);

	return EOK;
}

int mx6q_mii_op_ioctl(mx6q_dev_t *mx6q, struct ifdrv *ifd)
{
	mx6q_mii_op_t   mii_op;
	uint32_t        length,mii_op_size = 0;
	nic_config_t	*cfg = &mx6q->cfg;
	int             err = EOK;
	int             cl45;

	if (ifd->ifd_len <= sizeof(mii_op)) {
		return EINVAL;
	}

	/* get the size of mii_op*/
	if (ISSTACK) {
		if (copyin((((uint8_t *)ifd) + sizeof(*ifd)),
			&mii_op_size, sizeof(uint32_t))) {
				return EINVAL;
		}
	} else {
		memcpy(&mii_op_size, (((uint8_t *)ifd) + sizeof(*ifd)), sizeof(uint32_t));
	}

	// each clause 45 access need four Clause 22 access
	length = mx6q->cl45 ? sizeof(mii_op) * 4 : sizeof(mii_op);

    if (mii_op_size % length != 0) {
		return EINVAL;
    }
    if (mii_op_size == 0) {
		return EINVAL;
    }

    /*lock the mutex for atomic */
    length = 0;

    nic_mutex_lock(&mx6q->mii_mutex);
    // Always use clause 22 access
    cl45 = mx6q->cl45, mx6q->cl45 = 0;

    while (length < mii_op_size) {

        /* get the mii_op data*/
		if (ISSTACK) {
			if (copyin((((uint8_t *)ifd) + sizeof(*ifd)) + sizeof(mii_op_size) + length,
					&mii_op, sizeof(mii_op))) {
				err = EINVAL;
				break;
			}
		} else {
			memcpy(&mii_op, (((uint8_t *)ifd) + sizeof(*ifd)) + sizeof(mii_op_size)+ length, sizeof(mii_op));
		}

        switch(mii_op.op) {
            case SLEEP:
                if (cfg->verbose > 5) {
					slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): op SLEEP:  %d\n", __FUNCTION__, mii_op.data);
				}
				nic_delay(mii_op.data);
                mii_op.err = 0;
                break;
            case READ:
                mii_op.data = mx6q_mii_read(mx6q, mii_op.addr, mii_op.reg);
                mii_op.err  = 0;
				if (cfg->verbose > 5) {
					slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): op READ:%x,%x,%x\n", __FUNCTION__, mii_op.addr, mii_op.reg, mii_op.data);
				}
                break;
            case WRITE:
                mx6q_mii_write(mx6q, mii_op.addr, mii_op.reg, mii_op.data);
                mii_op.err = 0;
				if (cfg->verbose > 5) {
					slog2f(NULL,_SLOGC_NETWORK, _SLOG_INFO, "%s(): op WRITE:  %x,%x,%x\n", __FUNCTION__, mii_op.addr, mii_op.reg, mii_op.data);
				}
                break;
            default:
                slog2f(NULL,_SLOGC_NETWORK, _SLOG_ERROR, "%s(): unkownn mii_op  %d\n", __FUNCTION__, mii_op.op);
                mii_op.err = -1;
                err = EINVAL;
        }

		if (err != EOK) break;

		if (ISSTACK) {
            err = copyout(&mii_op, (((uint8_t *)ifd) + sizeof(*ifd)) + sizeof(mii_op_size) + length, sizeof(mii_op));
            if (err != EOK) {
                break;
            }
		} else {
			memcpy((((uint8_t *)ifd) + sizeof(*ifd)) + sizeof(mii_op_size) + length, &mii_op, sizeof(mii_op));
		}

        length += sizeof(mii_op);
    }

    mx6q->cl45 = cl45;
    nic_mutex_unlock(&mx6q->mii_mutex);

	return err;
}


int mx6q_phy_funcs(mx6q_dev_t *mx6q, struct ifdrv *ifd)

{
    phy_access_t phy;

    if (strcmp (ifd->ifd_name, "fec0")) {
        return (EINVAL);
    }
	if (mx6q->cl45) {
		return ENOTSUP;
	}
    if (ifd->ifd_data == NULL) {
        return (EINVAL);
    }
    if (ifd->ifd_len != sizeof (phy_access_t)) {
        return (EINVAL);
    }

    if (copyin ((((uint8_t *)ifd) + sizeof(*ifd)), &phy, sizeof(phy))) {
        return (EINVAL);
    }

    if (phy.phy_id >= 32) {
        return (EINVAL);
    }

    nic_mutex_lock(&mx6q->mii_mutex);

    if (ifd->ifd_cmd == READ_PHY_REG)
        phy.phy_data = mx6q_mii_read (mx6q, phy.phy_id, phy.phy_reg);
    else
        mx6q_mii_write (mx6q, phy.phy_id, phy.phy_reg, phy.phy_data);

    copyout (&phy, (((uint8_t *)ifd) + sizeof(*ifd)), sizeof(phy));

    nic_mutex_unlock(&mx6q->mii_mutex);
    return (EOK);
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devnp/mx6x/devctl.c $ $Rev: 910691 $")
#endif
