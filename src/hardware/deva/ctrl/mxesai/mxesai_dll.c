/*
 * $QNXLicenseC:
 * Copyright 2010, 2011, 2013 QNX Software Systems.
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

/*
 *
 *    mxesai_dll.c
 *      The primary interface into the mxesai DLL.
 */
#include "mxesai.h"
#include "variant.h"

/* Wait time count by usecond */
#define MAX_WAITTIME_US    1000

/**
 * This function returns the number of active capture channels
 */
uint32_t
num_active_capture_interfaces(HW_CONTEXT_T * mx)
{
	uint32_t num; /* number of capture interfaces */
	uint32_t idx = 0;

	num = 0;
	for(idx=0; idx < mx->num_rx_aif; idx++)
	{
		if(mx->aif[idx].cap_strm.active == 1)
			num++;
	}
	return num;
}

/**
 * This function returns the number of active playback channels
 */
uint32_t
num_active_playback_interfaces(HW_CONTEXT_T * mx)
{
	uint32_t num; /* number of playback interfaces */
	uint32_t idx = 0;

	num = 0;
	for(idx=0; idx < mx->num_tx_aif; idx++)
	{
		if(mx->aif[idx].play_strm.active == 1)
			num++;
	}
	return num;
}

static int32_t
mxesai_capabilities (HW_CONTEXT_T * mx, ado_pcm_t *pcm, snd_pcm_channel_info_t * info)
{
	uint32_t  i;
	int	  chn_avail = 1;

	if (info->channel == SND_PCM_CHANNEL_PLAYBACK)
	{
		info->fragment_align = mx->sample_size * mx->nslots * mx->num_sdo_pins;
		for (i = 0; i < mx->num_tx_aif; i++)
		{
			if (pcm == mx->aif[i].pcm && mx->aif[i].play_strm.pcm_subchn != NULL)
				chn_avail = 0;
		}

		if (chn_avail && (mx->clk_mode == ESAI_CLK_MASTER) && (mx->sample_rate_min != mx->sample_rate_max))
		{
			ado_mutex_lock(&mx->hw_lock);
			/* Playback and Capture are Rate locked, so adjust rate capabilities
			 * if the other side has an active stream.
			 */
			if (num_active_capture_interfaces(mx) != 0)
			{
				info->min_rate = info->max_rate = mx->sample_rate;
				info->rates = ado_pcm_rate2flag(mx->sample_rate);
			}
			ado_mutex_unlock(&mx->hw_lock);
		}
	}
	else if (info->channel == SND_PCM_CHANNEL_CAPTURE)
	{
		info->fragment_align = mx->sample_size * mx->nslots * mx->num_sdi_pins;
		for (i = 0; i < mx->num_rx_aif; i++)
		{
			if ( pcm == mx->aif[i].pcm && mx->aif[i].cap_strm.pcm_subchn != NULL)
				chn_avail = 0;
		}

		if (chn_avail && (mx->clk_mode == ESAI_CLK_MASTER) && (mx->sample_rate_min != mx->sample_rate_max))
		{
			ado_mutex_lock(&mx->hw_lock);
			/* Playback and Capture are Rate locked, so adjust rate capabilities
			 * if the other side has an active stream.
			 */
			if (num_active_playback_interfaces(mx) != 0)
			{
				info->min_rate = info->max_rate = mx->sample_rate;
				info->rates = ado_pcm_rate2flag(mx->sample_rate);
			}

			ado_mutex_unlock(&mx->hw_lock);
		}

	}

	if (chn_avail == 0)
	{
		info->formats = 0;
		info->rates = 0;
		info->min_rate = 0;
		info->max_rate = 0;
		info->min_voices = 0;
		info->max_voices = 0;
		info->min_fragment_size = 0;
		info->max_fragment_size = 0;
	}

	return (0);
}

#if VARIANT_mx8
/**
 * Get chip type and revision codes.
 *
 * @param chip_info Pointer to chip_info_t structure.
 *
 * @return  error status
 */
static int imx_get_chip_revision(chip_info_t *chip_info)
{
    unsigned hwi_off;

    chip_info->rev = 0;
    chip_info->type = 0;

    /* Look for board and silicon version */
    hwi_off = hwi_find_device("board", 0);
    if (hwi_off != HWI_NULL_OFF) {
        hwi_tag *tag_hwversion = hwi_tag_find(hwi_off, HWI_TAG_NAME_hwversion, 0);
        if (tag_hwversion != NULL) {
            chip_info->rev = tag_hwversion->hwversion.version;
            chip_info->type = tag_hwversion->hwversion.hclass;
            return (EOK);
        }
    }
    return (-1);
}
#endif

static uint8_t mxesai_port_count (void)
{
#if VARIANT_mx8
    chip_info_t chip_info;

    if (EOK != imx_get_chip_revision(&chip_info)) {
        ado_error_fmt("Unable to read i.MX8 SoC revision");
    }
    else
    {
        if (chip_info.type == IMX_CHIP_TYPE_QUAD_X_PLUS) {
            return QXP_ESAI_COUNT;
        }
    }
#endif
    return ESAI_COUNT;
}

static int calculate_fifo_watermark(uint8_t pin_cnt, ado_pcm_config_t *config)
{
	int watermark = FIFO_WATERMARK * 8;
#ifdef IMX_EDMA
	/* NOTE: The fragment size (DMA transfer size) must be an even multiple of the FIFO watermark
	 *       and the watermark must be evenly divisible by the number of active pins.
	 */
#ifndef MIN_WATERMARK
    #define MIN_WATERMARK 16
#endif
#ifndef MAX_WATERMARK
    #define MAX_WATERMARK 64
#endif
	for (watermark = MAX_WATERMARK; watermark >= MIN_WATERMARK; watermark--)
	{
		if (watermark % pin_cnt == 0)
		{
			if (config->mode.block.frag_size % watermark == 0 && !((config->mode.block.frag_size / watermark) & 0x1))
				break;  /* We found a suitable watermark */
		}
	}
	if (watermark < MIN_WATERMARK)
	{
		watermark = MAX_WATERMARK / pin_cnt * pin_cnt;
		ado_error_fmt("MX_ESAI: WARNING - Suitable watermark could not be found, defaulting to %d bytes", watermark);
	}

	/* Fragment size has to be an even multiple of the FIFO watermark
	 * Note: Just check the LSB to see if frag_size is an even multiple of the FIFO watermark or not.
	 */
	if ((config->mode.block.frag_size % watermark != 0) || ((config->mode.block.frag_size / watermark) & 0x1))
	{
		int frame_size = ado_pcm_format_byte_width(config->format.format) * config->format.voices * pin_cnt;
		int alignment = ado_lcm(watermark * 2, frame_size);    /* Multiple by 2 to make it an even multiple */

		ado_error_fmt("MX_ESAI: WARNING - Aligning fragment (%d) to be an even multiple of the FIFO watermark (%d) and the frame size (%d) - Alignment = %d",
					  config->mode.block.frag_size, watermark, frame_size, alignment);

		if (config->mode.block.frag_size <= alignment) {
			config->mode.block.frag_size = alignment;
		} else {
			config->mode.block.frag_size = config->mode.block.frag_size / alignment * alignment;
		}

		/* Update total DMA buffer size based on new fragment size */
		config->dmabuf.size = config->mode.block.frags_total * config->mode.block.frag_size;
	}
	ado_debug(DB_LVL_DRIVER, "FIFO watermark = %d, pin count = %d, Fragment size = %d", watermark, pin_cnt, config->mode.block.frag_size);
#endif
	return (watermark);
}

/**
 * This function stops TX transmission.
 *
 * @param mx ESAI hardware context structure.
 *
 * @return void
 */
static void
imx_esai_stop_tx(HW_CONTEXT_T *mx)
{
	int i = 0;

	/* Abort DMA transfer */
	if (mx->dmafuncs.xfer_abort (mx->aif[0].play_strm.dma_chn) == -1)
	{
		int rtn = errno;
		ado_error_fmt("Audio DMA Stop failed (%s)", strerror(rtn));
	}

	/* If any of the transmitters are enabled then wait for FIFO to drain */
	if (mx->esai->tcr & TCR_TEMASK)
	{
		/* Wait for FIFO to drain
		 * Wait up to 16ms for FIFO (256bytes) to drain
		 * 16 = 256 / (8khz * 2bytes) * 1000
		 */
		while (((mx->esai->tfsr&0xff) > 0) && (i<16))
		{
			i++;
			delay(1);
		}
		/* Wait for the transmit shift register to empty (underrun) */
		i = 1000;
		while(!(mx->esai->saisr & (SAISR_TDE | SAISR_TUE)) && i-- > 0)
			nanospin_ns(100);
		if (i <= 0)
			ado_error_fmt("TXSR failed to empty, saisr = 0x%x, TX FIFO level = %d words", mx->esai->saisr, mx->esai->tfsr & 0xff);
		else
			ado_debug(DB_LVL_DRIVER, "saisr = 0x%x, TX FIFO level = %d words", mx->esai->saisr, mx->esai->tfsr & 0xff);
	}
	/* Disable FIFO and transmitters
	 * NOTE: Must disable FIFO before disabling transmitters
	 */
	mx->esai->tfcr &= ~TFCR_TFE;
	mx->esai->tfcr &= ~TFCR_TEMASK;
	mx->esai->tcr &= ~TCR_TEMASK;

	/* Disable all TX slots
	 * Note: Always program tsmb first, followed by tsma (refer to ESAI Initialization chapter in iMX8 reference manual)
	 */
	mx->esai->tsmb = 0;
	mx->esai->tsma = 0;

	/* Put TX FIFO into reset */
	mx->esai->tfcr |= TFCR_TFR;

}

/**
 * This function starts TX transmission.
 *
 * @param mx ESAI hardware context structure.
 *
 * @return Execution status.
 */
static int32_t imx_esai_start_tx(HW_CONTEXT_T *mx)
{
	int i;
	int rtn = EOK;
	int16_t pin;

	/* Take TX FIFO out of reset */
	mx->esai->tfcr &= ~(TFCR_TFR);

	/* Enable requested transmitters to use FIFO (must be done before FIFO itself is enabled) */
	for (i = 0; i < MAX_NUM_SDO_PINS; i++)
	{
		pin = 0x1 << i;
		if (mx->sdo_pins & pin)
		{
			mx->esai->tfcr |= TFCR_TE(i);
		}
	}

	if (mx->sample_size == 2)
	{
		/* Set FIFO watermar, MSB of data is bit 23; Init transmit register on FIFO enable*/
		mx->esai->tfcr |= TFCR_TFWM(mx->tx_fifo_watermark) | TFCR_TWA(2) | TFCR_TIEN;
	}
	else
	{
		/* Set FIFO watermar, MSB of data is bit 31; Init transmit register on FIFO enable*/
		mx->esai->tfcr |= TFCR_TFWM(mx->tx_fifo_watermark) | TFCR_TWA(0) | TFCR_TIEN;
	}

	/* Start DMA for filling TX FIFO */
	if (mx->dmafuncs.xfer_start(mx->aif[0].play_strm.dma_chn) == -1)
	{
		rtn = errno;
		ado_error_fmt("Audio DMA Start failed (%s)", strerror(errno));
		/* Disable FIFO access for transmitters */
		mx->esai->tfcr &= ~TFCR_TEMASK;
		/* Put TX FIFO into reset */
		mx->esai->tfcr |= TFCR_TFR;
		return rtn;
	}

	/* Enable TX FIFO
	 * Note: Enabling the FIFO will trigger the DMA request
	 */
	mx->esai->tfcr |= TFCR_TFE;

	/* Give DMA time to fill the FIFO */
	while (((mx->esai->tfsr & 0xff) < 16) && (i < MAX_WAITTIME_US))
	{
		i++;
		nanospin_ns(1000);
	}
	/* Report an error if DMA didn't fill the FIFO on time, but still keep running */
	if (i >= MAX_WAITTIME_US)
	{
		rtn = ETIME;
		ado_error_fmt("Audio TX FIFO failed to fill (%d words)", mx->esai->tfsr & 0xff);
		imx_esai_stop_tx(mx);
		return rtn;
	}
	else
	{
		ado_debug(DB_LVL_DRIVER, "saisr = 0x%x, TX FIFO level = %d words", mx->esai->saisr, mx->esai->tfsr & 0xff);
	}

	/* Enable TX (also configures the SD pin to function as an output) */
	volatile uint32_t tmp = 0;
	for (i = 0; i < MAX_NUM_SDO_PINS; i++)
	{
		pin = 0x1 << i;
		if (mx->sdo_pins & pin)
		{
			tmp |= TCR_TE(i);
		}
	}
	mx->esai->tcr |= tmp; /* Enable all transmitters in a single operation */
	/* Enable TX slots
	 * Note: Always program tsmb first, followed by tsma (refer to ESAI Initialization chapter in iMX8 reference manual)
	 */
	mx->esai->tsmb = (mx->tx_slot_mask >> 16);
	mx->esai->tsma = (mx->tx_slot_mask & 0xffff);

	/* Un-mute codec after serializer is enabled to minimize pop */
#ifdef CODEC_PLAYBACK_UNMUTE
	CODEC_PLAYBACK_UNMUTE;
#endif
	return (rtn);
}

static int
mxesai_set_clock_rate ( HW_CONTEXT_T *mx, int rate)
{
	uint32_t bit_clk_div, tccr, rccr;

	tccr = rccr = 0;
	mx->sample_rate = rate;

	if (mx->fsync_pol == 0)
	{
		/* Active low Frame sync */
		tccr |= TCCR_TFSP;
		rccr |= RCCR_RFSP;
	}

	if (mx->xclk_pol == 0)
		tccr |= TCCR_TCKP;	/* TX on falling edge */

	if (mx->rclk_pol == 0)
		rccr |= RCCR_RCKP;	/* RX on falling edge */

	/* Slots/words per frame (program 1 less then desired value as per docs) */
	tccr |= TCCR_TDC((mx->nslots - 1));
	rccr |= RCCR_RDC((mx->nslots - 1));

	/* Calculate MCLK divisor.
	 * NOTE: The current implementation supports synchronous mode only -
	 *       (SAICR[SYN]=1) where the receiver uses the tx clock control register
	 *       settings. However, we still program the RCCR in the event that we
	 *       need to support asynchronous clocking in the future.
	 */
#define FIXED_DIV2 2
	bit_clk_div = mx->main_clk_freq / (FIXED_DIV2 * mx->sample_rate * mx->nslots * mx->slot_size);

	if (bit_clk_div <= 256)
	{
		// Bypass Div8 (i.e. TPSR/RPSP=1 which results in divide by 1), use TPM/RPM divider, don't use TFP/RFP divider
		tccr |= TCCR_TPSR | TCCR_TPM((bit_clk_div-1)) | TCCR_TFP(0);
		rccr |= RCCR_RPSP | RCCR_RPM((bit_clk_div-1)) | RCCR_RFP(0);
	}
	else
	{
		if (bit_clk_div <= (256*8))
		{
			// Don't bypass Div8 (TPSR=0), use TPM/RPM divider, don't use TFP/RFP divider
			tccr |= TCCR_TPM(((bit_clk_div/8)-1)) | TCCR_TFP(0);
			rccr |= RCCR_RPM(((bit_clk_div/8)-1)) | RCCR_RFP(0);
		}
		else
		{
			// Don't bypass Div8 (TPSR=0), use TPM/RPM divider, use TFP/RFP divider
			tccr |= TCCR_TPM(256) | TCCR_TFP(((bit_clk_div/(8*256))-1));
			rccr |= RCCR_RPM(256) | RCCR_RFP(((bit_clk_div/(8*256))-1));
		}
	}

	switch (mx->main_clk_src)
	{
		/*
		 * 'XTAL' refers to an SOC generated clock. The clock to supply, and divisors are
		 * are selected by the CCM Serial Clock Multiplexor Register, and Clock Divider Register.
		 */
		case MAIN_CLK_SRC_XTAL:

			/* word length frame sync; slot length = 16, word length = 16; network mode */
			mx->esai->tcr |= TCR_TSWS(0x02) | TCR_TMOD (1);

			if (mx->clk_mode == ESAI_CLK_MASTER)
			{
				/*
				 * THCKD=1: High frequency clock is an output
				 * TCKD=1:  SCKT (bit clock) output
				 * TFSD=1:  FST (frame sync) output
				 */
				tccr |= TCCR_THCKD | TCCR_TCKD | TCCR_TFSD;

				/*
				 * RHCKD=1: High Frequency clock direction output (not needed...)
				 * RCKD=1:  Receiver bit clock is an output
				 * RFSD=1:  Receiver frame sync is an output
				 */
				rccr |= RCCR_RHCKD | RCCR_RCKD | RCCR_RFSD;
			}

			/* HCKT clock will be derived from XTAL.  The on-chip esai clock source is
			 * specified by the CCM, the CCM refers to this clock as 'esai_clk_root'.
			 */
			mx->esai->ecr |= ECR_ETI | ECR_ERI;

			/* Prevent XTAL clock override */
			mx->esai->ecr &= ~(ECR_ETO | ECR_ERO);

			break;

		/* 'FSYS' refers to the SOC generated ESAI system clock, which is typically 133MHz */
		case MAIN_CLK_SRC_FSYS:
			ado_error_fmt("Clock source must be either XTAL or OSC");
			return -1;

		/* 'OSC' refers to an external Oscillator */
		case MAIN_CLK_SRC_OSC:
			if (mx->clk_mode == ESAI_CLK_MASTER)
			{
				/*
				 * THCKD = 0: High frequency clock is an input
				 * TCKD = 1: SCKT is an output (use internal bit and frame sync generators)
				 * TFSD = 1: FST (frame sync) is an output
				 *
				 * NOTE: When the internal ARM Core is the clock source you cannot
				 *       set TPSR (divide by 8 bypass), and TPM[7-0] = 0x0 and TFP[3-0] = 0x0
				 *       as this will cause clock synchronization problems...in this case
				 *       the HCKT is the clock source so we should be okay.
				 */
				tccr |= TCCR_TCKD | TCCR_TFSD;

				/*
				 * RHCKD = 0: High frequency clock is an input
				 * RCKD = 1: SCKT is an output (use internal bit and frame sync generators)
				 * RFSD = 1: FST (frame sync) is an output
				 */
				rccr |= RCCR_RCKD | RCCR_RFSD;
			}

			break;

		default:
			ado_error_fmt("main_clk_src value not supported");
			return -1;

	}

	ado_debug(DB_LVL_DRIVER, "tccr=0x%x, rccr=0x%x", tccr, rccr);

	// only program tccr and rccr registers once after all bits have been set, otherwise
	// bit clock and frame clock will not be set correctly.
	mx->esai->tccr = tccr;
	mx->esai->rccr = rccr;
	usleep(125);

	return (EOK);
}

static int32_t
mxesai_playback_aquire (HW_CONTEXT_T * mx, PCM_SUBCHN_CONTEXT_T ** strm,
	ado_pcm_config_t * config, ado_pcm_subchn_t * subchn, uint32_t * why_failed)
{
	uint32_t i;
	mxesai_aif_t *aif = NULL;
#if IMX_EDMA
	dma_addr_t addr;
#endif

	ado_mutex_lock (&mx->hw_lock);
	for ((*strm) = NULL, i = 0; i < mx->num_tx_aif; i++)
	{
		if (ado_pcm_subchn_is_channel(subchn, mx->aif[i].pcm, ADO_PCM_CHANNEL_PLAYBACK))
		{
			aif = &mx->aif[i];
			*strm = &aif->play_strm;
			break;
		}
	}

	if ((*strm) == NULL || (*strm)->pcm_subchn || aif == NULL)
	{
		*why_failed = SND_PCM_PARAMS_NO_CHANNEL;
		ado_mutex_unlock (&mx->hw_lock);
		return (EAGAIN);
	}

	(*strm)->active = 0;

	/* If on-the-fly sample rate switching is supported check if a rate switch is needed */
	if ((mx->clk_mode & ESAI_CLK_MASTER) && mx->sample_rate_min != mx->sample_rate_max)
	{
		if (config->format.rate != mx->sample_rate)
		{
			/* Playback and capture are rate locked, so wait until any active captures streams complete before
			 * changing the sample rate.
			 */
			if (num_active_capture_interfaces(mx) > 0)
			{
				ado_mutex_unlock(&mx->hw_lock);
				return (EBUSY);
			}
			mxesai_set_clock_rate(mx, config->format.rate);
		}
	}

	config->dmabuf.flags = ADO_BUF_CACHE;
	/* If num_tx_aif == 1, then we DMA right from the pcm buffer so make it DMA safe */
	if (mx->num_tx_aif == 1)
	{
		config->dmabuf.flags |= ADO_SHM_DMA_SAFE;
		mx->tx_fifo_watermark = calculate_fifo_watermark(mx->num_sdo_pins, config);
	}
	if (ado_pcm_buf_alloc(config, config->dmabuf.size, config->dmabuf.flags) == NULL)
	{
		ado_mutex_unlock(&mx->hw_lock);
		return (errno);
	}

	if (mx->num_tx_aif == 1)
	{
		dma_transfer_t tinfo;
		int frag_idx;

		memset (&tinfo, 0, sizeof (tinfo));
		tinfo.src_addrs = ado_calloc (config->mode.block.frags_total, sizeof (dma_addr_t));
		if(NULL == tinfo.src_addrs)
		{
			ado_error_fmt("Insufficient memory");
			ado_pcm_buf_free(config);
			ado_mutex_unlock (&mx->hw_lock);
			return -1;
		}
#if IMX_EDMA
		/* eDMA driver specific initialization */
		/* Allocate dst_addr for TX reg */
		tinfo.dst_addrs = &addr;
		tinfo.dst_fragments = 1;
		/* Initialize dst_addr to physical address of TX register or ASRC IN register */
		tinfo.dst_addrs->paddr = mx->esaibase + IMX_ESAI_ETDR_OFFSET;
		tinfo.dst_addrs->len = mx->sample_size;
		/* Register address cannot be incremented */
		tinfo.dst_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;
		tinfo.mode_flags = DMA_MODE_FLAG_REPEAT;
		tinfo.xfer_bytes = mx->tx_fifo_watermark;
#else
		tinfo.xfer_bytes = config->dmabuf.size;
#endif
		for (frag_idx = 0; frag_idx < config->mode.block.frags_total; frag_idx++)
		{
			tinfo.src_addrs[frag_idx].paddr =
				config->dmabuf.phys_addr + (frag_idx * config->mode.block.frag_size);
			tinfo.src_addrs[frag_idx].len = config->mode.block.frag_size;
		}
		tinfo.src_fragments	 = config->mode.block.frags_total;
		tinfo.xfer_unit_size = (mx->sample_size == 2) ? 16 : 32;		/* 16/32-bit samples */
		mx->dmafuncs.setup_xfer ((*strm)->dma_chn, &tinfo);
		ado_free (tinfo.src_addrs);
	}

	(*strm)->pcm_subchn = subchn;
	(*strm)->pcm_config = config;
	ado_mutex_unlock (&mx->hw_lock);
	return (EOK);
}


static int32_t
mxesai_playback_release (HW_CONTEXT_T * mx, PCM_SUBCHN_CONTEXT_T * strm,
	ado_pcm_config_t * config)
{
	ado_mutex_lock (&mx->hw_lock);
	ado_pcm_buf_free(config);
	strm->pcm_subchn = NULL;
	strm->pcm_config = NULL;
	ado_mutex_unlock (&mx->hw_lock);
	return (0);
}

static int32_t
mxesai_capture_aquire (HW_CONTEXT_T * mx, PCM_SUBCHN_CONTEXT_T ** strm,
	ado_pcm_config_t * config, ado_pcm_subchn_t * subchn, uint32_t * why_failed)
{
	uint32_t i;
	mxesai_aif_t *aif = NULL;
#if IMX_EDMA
	dma_addr_t addr;
#endif

	ado_mutex_lock (&mx->hw_lock);
	for ((*strm) = NULL, i = 0; i < mx->num_rx_aif; i++)
	{
		if (ado_pcm_subchn_is_channel(subchn, mx->aif[i].pcm, ADO_PCM_CHANNEL_CAPTURE))
		{
			aif = &mx->aif[i];
			*strm = &aif->cap_strm;
			break;
		}
	}

	if ((*strm) == NULL || (*strm)->pcm_subchn || aif == NULL)
	{
		*why_failed = SND_PCM_PARAMS_NO_CHANNEL;
		ado_mutex_unlock(&mx->hw_lock);
		return EAGAIN;
	}

	(*strm)->active = 0;

	/* If multiple rates supported check for rate switch */
	if ((mx->clk_mode & ESAI_CLK_MASTER) && mx->sample_rate_min != mx->sample_rate_max)
	{
		if (config->format.rate != mx->sample_rate)
		{
			/* Playback and capture are rate locked, so wait until any active playback streams complete before
			 * changing the sample rate.
			 */
			if (num_active_playback_interfaces(mx) > 0)
			{
				ado_mutex_unlock(&mx->hw_lock);
				return (EBUSY);
			}
			mxesai_set_clock_rate(mx, config->format.rate);
		}
	}


	config->dmabuf.flags = ADO_BUF_CACHE;
	/* If num_rx_aif == 1, then we DMA right into the pcm buffer so make it DMA safe */
	if (mx->num_rx_aif == 1)
	{
		config->dmabuf.flags |= ADO_SHM_DMA_SAFE;
		mx->rx_fifo_watermark = calculate_fifo_watermark(mx->num_sdi_pins, config);
	}
	if (ado_pcm_buf_alloc(config, config->dmabuf.size, config->dmabuf.flags) == NULL)
	{
		ado_mutex_unlock(&mx->hw_lock);
		return (errno);
	}
	/* Only setup the DMA tranfer if there is only 1 capture interface/device.
	 * If multiple capture devices then a ping-pong DMA buffer will be allocated and initialized in ctrl_init().
	 */
	if (mx->num_rx_aif == 1)
	{
		dma_transfer_t tinfo;
		int frag_idx;

		memset (&tinfo, 0, sizeof (tinfo));
		tinfo.dst_addrs = ado_calloc (config->mode.block.frags_total, sizeof (dma_addr_t));
		if(NULL == tinfo.dst_addrs)
		{
			ado_error_fmt("Insufficient memory");
			ado_pcm_buf_free(config);
			ado_mutex_unlock (&mx->hw_lock);
			return -1;
		}
		for (frag_idx = 0; frag_idx < config->mode.block.frags_total; frag_idx++)
		{
			tinfo.dst_addrs[frag_idx].paddr =
				config->dmabuf.phys_addr + (frag_idx * config->mode.block.frag_size);
			tinfo.dst_addrs[frag_idx].len = config->mode.block.frag_size;
		}

#if IMX_EDMA
		tinfo.src_addrs = &addr;
		tinfo.src_fragments = 1;
		/* Initialize dst_addr to physical address of RX register */
		tinfo.src_addrs->paddr = mx->esaibase + IMX_ESAI_ERDR_OFFSET;
		tinfo.src_addrs->len = mx->sample_size;
		/* Register address cannot be incremented */
		tinfo.src_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;
		tinfo.mode_flags = DMA_MODE_FLAG_REPEAT;
		tinfo.xfer_bytes = mx->rx_fifo_watermark;
#else
		tinfo.xfer_bytes = config->dmabuf.size;
#endif
		tinfo.dst_fragments = config->mode.block.frags_total;
		tinfo.xfer_unit_size = (mx->sample_size == 2) ? 16 : 32;	/* 16/32-bit samples */
		mx->dmafuncs.setup_xfer(mx->aif[0].cap_strm.dma_chn, &tinfo);
		ado_free (tinfo.dst_addrs);
	}
	(*strm)->pcm_config = config;
	(*strm)->pcm_subchn = subchn;
	ado_mutex_unlock (&mx->hw_lock);
	return (EOK);
}

static int32_t
mxesai_capture_release (HW_CONTEXT_T * mx, PCM_SUBCHN_CONTEXT_T * strm,
	ado_pcm_config_t * config)
{
	ado_mutex_lock (&mx->hw_lock);
	ado_pcm_buf_free(config);
	strm->pcm_subchn = NULL;
	strm->pcm_config = NULL;
	ado_mutex_unlock (&mx->hw_lock);
	return (0);
}

static int32_t
mxesai_prepare (HW_CONTEXT_T * mx, PCM_SUBCHN_CONTEXT_T * strm, ado_pcm_config_t * config)
{
	strm->pcm_offset = 0;
	return (0);
}

/* Since this function is forced inline, when called with a const literal sampleSize (i.e. sizeof(int16_t))
 * the compiler can optimize away any branches based on the sampleSize.
 */
static inline int32_t __attribute__ ((__always_inline__))
mxesai_reconstitute_internal (HW_CONTEXT_T *mx, PCM_SUBCHN_CONTEXT_T *pc, void *dmaptr, size_t size, const size_t sampleSize, const int channel)
{
	int cnt;
	const uint32_t pins = (channel == SND_PCM_CHANNEL_PLAYBACK) ? mx->num_sdo_pins : mx->num_sdi_pins;
	const int frame_size = mx->nslots * pins;              /* Frame size in slots/samples */
	const int frame_size_bytes = frame_size * sampleSize;  /* Frame size in bytes         */
	const int samples = size / sampleSize;
	int src_idx = 0, dst_idx;
	uint32_t vidx, pidx = 0;
	int16_t * const ptr16 = dmaptr;
	int32_t * const ptr32 = dmaptr;
	int16_t * const reconstitute_buffer16 = (channel == SND_PCM_CHANNEL_PLAYBACK) ? mx->sdo_reconstitute_buffer : mx->sdi_reconstitute_buffer;
	int32_t * const reconstitute_buffer32 = (channel == SND_PCM_CHANNEL_PLAYBACK) ? mx->sdo_reconstitute_buffer : mx->sdi_reconstitute_buffer;

	/* When using multiple data lines, each sample is shifted in/out from/to the next active line.
	 * For playback:
	 *     We must interleave the data over the pins so that left and right sample
	 *     pairs will be on the same output rather than split over multiple outputs.
	 * For capture:
	 *     Left and right samples were received on the same pin, but was interleaved with
	 *     data from the other pins as they are shifted into the FIFO. We must de-interleave
	 *     the data so that the samples from each pin are contiguous in memory.
	 * Note: When using multiple pins, each pin is dedicated to one interface, and all
	 *       interfaces must be configured for the same number of voices.
	 */
	for (cnt = 0; cnt < samples; cnt+=frame_size)
	{
		/* Reconstitute the frame */
		for (dst_idx = vidx = pidx = 0; dst_idx < frame_size; dst_idx++)
		{
			if (channel == ADO_PCM_CHANNEL_PLAYBACK)
			{
				/* For playback we interleave the pin data by dividing the frame
				 * into subframes of nslots samples, and then pulling the same voice/channel
				 * offset from each subframe and placing them contiguously into the
				 * reconstitute_buffer. Then start over pulling next voice/channel
				 * offset from each subframe. Continue this cycle until we have completed
				 * the full frame.
				 */
				src_idx = (pidx * mx->nslots) + vidx;
				if (++pidx >= pins)
				{
					pidx = 0; /* Reset pin/AIF (subframe) index back to zero */
					vidx++;   /* Increment to the next voice/slot index */
				}
			}
			else	/* ADO_PCM_CHANNEL_CAPTURE */
			{
				/* For capture we de-interleave by pulling all of the voices for
				 * the same pin placing them contiguously in the reconstitute_buffer,
				 * then start over pulling all the voices for the next pin.
				 * Continue this cycle until we have completed the ful frame.
				 */
				src_idx = (vidx * pins) + pidx;
				if (++vidx >= mx->nslots)
				{
					vidx = 0; /* Reset voice/slot index back to 0 */
					pidx++;   /* Increment to the next pin/AIF index */
				}
			}
			if (sampleSize == sizeof(int16_t))
				reconstitute_buffer16[dst_idx] = ptr16[cnt + src_idx];
			else
				reconstitute_buffer32[dst_idx] = ptr32[cnt + src_idx];
		}
		if (sampleSize == sizeof(int16_t)) {
			memcpy(&ptr16[cnt], reconstitute_buffer16, frame_size_bytes);
		} else {
			memcpy(&ptr32[cnt], reconstitute_buffer32, frame_size_bytes);
		}
	}
	return (0);
}

static int32_t
mxesai_playback_reconstitute (HW_CONTEXT_T *mx, PCM_SUBCHN_CONTEXT_T *pc, int8_t *dmaptr, size_t size)
{
	if (mx->sample_size == sizeof(int16_t))
		return (mxesai_reconstitute_internal (mx, pc, (void*)dmaptr, size, sizeof(int16_t), SND_PCM_CHANNEL_PLAYBACK));
	else
		return (mxesai_reconstitute_internal (mx, pc, (void*)dmaptr, size, sizeof(int32_t), SND_PCM_CHANNEL_PLAYBACK));
}

static int32_t
mxesai_capture_reconstitute (HW_CONTEXT_T *mx, PCM_SUBCHN_CONTEXT_T *pc, int8_t *dmaptr, size_t size)
{
	if (mx->sample_size == sizeof(int16_t))
		return (mxesai_reconstitute_internal (mx, pc, (void*)dmaptr, size, sizeof(int16_t), SND_PCM_CHANNEL_CAPTURE));
	else
		return (mxesai_reconstitute_internal (mx, pc, (void*)dmaptr, size, sizeof(int32_t), SND_PCM_CHANNEL_CAPTURE));

}


/* Since this function is forced inline, when called with a const literal sampleSize (i.e. sizeof(int16_t))
 * the compiler can optimize away any branches based on the sampleSize.
 */
static inline void __attribute__ ((__always_inline__))
mxesai_dmaplayback_combine_internal(HW_CONTEXT_T *mx, void *dstDMAAddr, const size_t size, bool calldmainterrupt, const size_t sampleSize)
{

	uint32_t cnt = 0, slot_cnt;
	uint32_t voice = 0;
	int32_t sample_offset = 0;
	int16_t *buf16;
	int32_t *buf32;
	int16_t *dmaptr16 = dstDMAAddr;
	int32_t *dmaptr32 = dstDMAAddr;

	if((NULL == dstDMAAddr) || (0 == size))
	{
		ado_error_fmt("invalid data (0x%x) or size (%d)", dstDMAAddr, size);
		return;
	}

	while ((sample_offset * mx->sample_size) < size)
	{
		for (cnt = 0, slot_cnt = 0; cnt < mx->num_tx_aif; cnt++ )
		{
			if (mx->aif[cnt].play_strm.active)
			{
				if ((mx->aif[cnt].play_strm.pcm_offset * sampleSize ) >= mx->aif[cnt].play_strm.pcm_config->dmabuf.size)
					mx->aif[cnt].play_strm.pcm_offset = 0;
				if (sampleSize == sizeof(int16_t)) {
					buf16 = (int16_t*) mx->aif[cnt].play_strm.pcm_config->dmabuf.addr + mx->aif[cnt].play_strm.pcm_offset;
				} else {
					buf32 = (int32_t*) mx->aif[cnt].play_strm.pcm_config->dmabuf.addr + mx->aif[cnt].play_strm.pcm_offset;
				}

				for (voice = 0; voice < mx->aif[cnt].play_strm.voices; voice++) {
					if (sampleSize == sizeof(int16_t)) {
						dmaptr16[sample_offset + voice] = buf16[voice];
					} else {
						dmaptr32[sample_offset + voice] = buf32[voice];
					}
				}
				/* Bump the pcm buffer offset */
				mx->aif[cnt].play_strm.pcm_offset += mx->aif[cnt].play_strm.voices;
				if (((mx->aif[cnt].play_strm.pcm_offset * sampleSize) %
					 ado_pcm_dma_int_size(mx->aif[cnt].play_strm.pcm_config)) == 0)
				{
					// Signal to io-audio (DMA transfer was completed)
					if (calldmainterrupt)
						dma_interrupt(mx->aif[cnt].play_strm.pcm_subchn);
				}
			}
			else
			{
				/* Silence fill the inactive TDM slots */
				for (voice = 0; voice <	 mx->aif[cnt].play_strm.voices; voice++) {
					if (sampleSize == sizeof(int16_t)) {
						dmaptr16[sample_offset + voice] = 0x0;
					} else {
						dmaptr32[sample_offset + voice] = 0x0;
					}
				}
			}
			slot_cnt += mx->aif[cnt].play_strm.voices;
			sample_offset += mx->aif[cnt].play_strm.voices;
		}
		/* Pad the remaining slots/words if the combined number of voices/channels accross
		 * all of our playback interfaces is less then configured nslots * num_sdo_pins
		 */
		for (;slot_cnt < mx->nslots * mx->num_sdo_pins; slot_cnt++)
		{
			if (sampleSize == sizeof(int16_t)) {
				dmaptr16[sample_offset++] = 0x0;
			} else {
				dmaptr32[sample_offset++] = 0x0;
			}
		}
	}
	/* If multiple pins are used call reconstitute to interleave the data over the pins */
	if (mx->num_sdo_pins > 1)
		mxesai_playback_reconstitute (mx, NULL, dstDMAAddr, size);

	/* Flush cache */
	msync(dstDMAAddr, size, MS_SYNC);

	if (mx->log_enabled) {
		write(mx->play_log_fd, dstDMAAddr, size);
	}
}

/*
 * This function is used when more than 1 playback pcm device is created. It is called from the
 * tx interrupt handler (or pulse) and trigger function to combine multiple pcm streams into 1
 * hardware TDM stream.
 */
static void
mxesai_dmaplayback_combine (HW_CONTEXT_T *mx, void *dstDMAAddr, const size_t size, bool calldmainterrupt)
{
	if (mx->sample_size == sizeof(int16_t)) {
		mxesai_dmaplayback_combine_internal(mx, dstDMAAddr, size, calldmainterrupt, sizeof(int16_t));
	} else {
		mxesai_dmaplayback_combine_internal(mx, dstDMAAddr, size, calldmainterrupt, sizeof(int32_t));
	}
}

static int open_log (HW_CONTEXT_T * mx, int chn_type)
{
	time_t t = time(NULL);
	struct tm *current_time = gmtime ( &t );
	char filename_buf[_POSIX_PATH_MAX];

	if(NULL == current_time)
		return -1;

	/* RAW PCM file, no wave header */
	if (chn_type == SND_PCM_CHANNEL_PLAYBACK)
	{
		snprintf(filename_buf, _POSIX_PATH_MAX, "/dev/shmem/playback_raw_%04d%02d%02d-%02d%02d%02dUTC.wav",
			1900 + current_time->tm_year, current_time->tm_mon + 1, current_time->tm_mday,
				 current_time->tm_hour, current_time->tm_min, current_time->tm_sec);

		if ((mx->play_log_fd = open ( filename_buf, O_WRONLY | O_CREAT, 0400)) == -1)
		{
			ado_error_fmt("MX_ESAI: Failed to create PCM log file (%s) - %s", filename_buf, strerror(errno));
			return (-errno);
		}
	}
	else
	{
		snprintf(filename_buf, _POSIX_PATH_MAX, "/dev/shmem/capture_raw_%04d%02d%02d-%02d%02d%02dUTC.wav",
			1900 + current_time->tm_year, current_time->tm_mon + 1, current_time->tm_mday,
				 current_time->tm_hour, current_time->tm_min, current_time->tm_sec);

		if ((mx->cap_log_fd = open ( filename_buf, O_WRONLY | O_CREAT, 0400)) == -1)
		{
			ado_error_fmt("MX_ESAI: Failed to create PCM log file (%s) - %s", filename_buf, strerror(errno));
			return (-errno);
		}
	}
	return EOK;
}

static int close_log ( HW_CONTEXT_T *mx, int chn_type)
{
	if (chn_type == SND_PCM_CHANNEL_PLAYBACK)
	{
		fsync( mx->play_log_fd );
		close(mx->play_log_fd);
	}
	else
	{
		fsync( mx->cap_log_fd );
		close(mx->cap_log_fd);
	}

	return EOK;
}

static int32_t
mxesai_playback_trigger (HW_CONTEXT_T * mx, PCM_SUBCHN_CONTEXT_T * strm, uint32_t cmd)
{
	int rtn = EOK;
	ado_mutex_lock (&mx->hw_lock);

	if (cmd == ADO_PCM_TRIGGER_GO)
	{
		if (num_active_playback_interfaces(mx) == 0)
		{
			if (mx->log_enabled)
				open_log(mx, SND_PCM_CHANNEL_PLAYBACK);
			strm->active = 1;
			mx->playback_frag_index = 0;
			if (mx->num_tx_aif > 1)
			{
				/* We use a ping-ping buffer for DMA, so divide buffer size by 2 to get the frag size */
				const size_t size = mx->playback_dmabuf.size / 2;

				/* Combine PCM data from the various PCM playback devices into the DMA buffer,
				 * Fill the entire DMA buffer (both fragments), we will backfill the DMA buffer
				 * on the interrupt completion interrupt/event
				 *
				 * Note: We cannot call dma_interrupt more than once from the trigger as this would
				 *       put subchns configured for minimum fragments into immediate underrun.
				 *       Defer the completion of the second fragment until the first real interrupt.
				 */
				mxesai_dmaplayback_combine(mx, (void*)(&mx->playback_dmabuf.addr[0]), size, false);
				mxesai_dmaplayback_combine(mx, (void*)(&mx->playback_dmabuf.addr[size]), size, true);
			}
			rtn = imx_esai_start_tx(mx);
			if(rtn != EOK)
			{
				ado_error_fmt("Couldn't start tx:%s", strerror(rtn));
				strm->active = 0;
			}
		}
		else
			strm->active = 1;
	}
	else
	{
		if (num_active_playback_interfaces(mx) == 1)
		{
#ifdef CODEC_PLAYBACK_MUTE
			CODEC_PLAYBACK_MUTE;
#endif
			/* Abort DMA transfer */
			if (mx->dmafuncs.xfer_abort (mx->aif[0].play_strm.dma_chn) == -1)
			{
				rtn = errno;
				ado_error_fmt("Audio DMA Stop failed (%s)", strerror(rtn));
			}
			imx_esai_stop_tx(mx);

			if (mx->log_enabled)
				close_log(mx, SND_PCM_CHANNEL_PLAYBACK);
		}
		strm->active = 0;
	}

	ado_mutex_unlock (&mx->hw_lock);
	return (rtn);
}

static int32_t
mxesai_capture_trigger (HW_CONTEXT_T * mx, PCM_SUBCHN_CONTEXT_T * strm, uint32_t cmd)
{
	int rtn = EOK, i, pin;
	volatile uint32_t tmp = 0;

	ado_mutex_lock (&mx->hw_lock);

	if (cmd == ADO_PCM_TRIGGER_GO)
	{
		if (num_active_capture_interfaces(mx) == 0)
		{
			if (mx->log_enabled)
				open_log(mx, SND_PCM_CHANNEL_CAPTURE);

			/* Take RX FIFO out of reset, configure watermark and word alignment */
			if (mx->sample_size == 2)
				mx->esai->rfcr = RFCR_RFWM(mx->rx_fifo_watermark) | RFCR_RWA(2);
			else
				mx->esai->rfcr = RFCR_RFWM(mx->rx_fifo_watermark) | RFCR_RWA(0);

			/* Enable requested receivers to use FIFO (must be done before FIFO itself is enabled) */
			for (i = 0; i < MAX_NUM_SDI_PINS; i++)
			{
				pin = 0x1 << i;
				if (mx->sdi_pins & pin)
				{
					tmp |= RFCR_RE(i);
				}
			}
			mx->esai->rfcr |= tmp; /* Enable all FIFOs in a single synchronous operation */
			mx->capture_frag_index = 0;
			if (mx->dmafuncs.xfer_start(mx->aif[0].cap_strm.dma_chn) == -1)
			{
				rtn = errno;
				ado_error_fmt("Audio DMA Start failed (%s)", strerror(rtn));
				/* Disable FIFO access for receivers */
				mx->esai->rfcr &= ~RFCR_REMASK;
				/* Put RX FIFO into reset (does not affect control bits) */
				mx->esai->rfcr |= RFCR_RFR;
				ado_mutex_unlock (&mx->hw_lock);
				return rtn;
			}
			/* Enable RX FIFO */
			mx->esai->rfcr |= RFCR_RFE;
			/* Enable RX (also configures the SD pin to function as an input) */
			tmp = 0;
			for (i = 0; i < MAX_NUM_SDI_PINS; i++)
			{
				pin = 0x1 << i;
				if (mx->sdi_pins & pin)
				{
					tmp |=  RCR_RE(i);
				}
			}
			mx->esai->rcr |= tmp; /* Enable all receivers in a single synchronous operation */
			/* Enable RX slots
			 * Note: Always program rsmb first, followed by rsma (refer to ESAI Initialization chapter in iMX8 reference manual)
			 */
			mx->esai->rsmb = (mx->rx_slot_mask >> 16);
			mx->esai->rsma = (mx->rx_slot_mask & 0xffff);

			ado_debug (DB_LVL_DRIVER, "RFCR=0x%x, RCR=0x%x", mx->esai->rfcr, mx->esai->rcr);

#ifdef CODEC_CAPTURE_UNMUTE
			CODEC_CAPTURE_UNMUTE;
#endif
		}
		strm->active = 1;
	}
	else
	{
		if (num_active_capture_interfaces(mx) == 1)
		{
#ifdef CODEC_CAPTURE_MUTE
			CODEC_CAPTURE_MUTE;
#endif
			/* Disable RX pins */
			mx->esai->rcr &= ~RCR_REMASK;

			/* Disable RX slots
			 * Note: Always program rsmb first, followed by rsma
			 */
			mx->esai->rsmb = 0;
			mx->esai->rsma = 0;

			/* On iMX8 the xfer_abort() simply disables the hardware DMA request to prevent
			 * further transfers. If there is an already active transfer (eDMA minor_loop)
			 * it will continue until completion. To ensure the DMA is actually stopped
			 * when the xfer_abort() returns, first wait for the FIFO to reach its watermark
			 * level to ensure there is no active minor_loop when we issue the xfer_abort().
			 */
			i = 0;
			while((mx->esai->esr & ESR_RFF) && (i++ < MAX_WAITTIME_US))
			{
				nanospin_ns(1000);
			}
			/* Log an error if DMA didn't drain the FIFO on time, but still keep running */
			if (i >= MAX_WAITTIME_US)
			{
				ado_error_fmt("Audio RX FIFO failed to drain (%d bytes)", mx->esai->rfsr & 0xff);
			}

			if (mx->dmafuncs.xfer_abort(mx->aif[0].cap_strm.dma_chn) == -1)
			{
				rtn = errno;
				ado_error_fmt("Audio DMA Stop failed (%s)", strerror(rtn));
			}
			if (mx->log_enabled)
				close_log(mx, SND_PCM_CHANNEL_CAPTURE);

			/* Disable RX FIFO */
			mx->esai->rfcr &= ~RFCR_RFE;
			/* Put RX FIFO into reset (does not affect control bits) */
			mx->esai->rfcr |= RFCR_RFR;
		}
		strm->active = 0;
	}

	ado_mutex_unlock (&mx->hw_lock);

	return (rtn);
}

/*
 * No position function as we are unable to get the transfer count of the
 * current DMA operation from the DMA microcode because bytes_left() function
 * in edma library doesn't support DMA_MODE_FLAG_REPEAT mode. The resolution
 * of the positional information returned to the client will be limited to the
 * fragment size.
 *
 * If we get new DMA microcode that supports this and the dma library's
 * bytes_left() function is updated to use this info to return the actual
 * bytes left, uncomment the below function and the function pointer
 * assignments in ctrl_init().
 */

#if 0
uint32_t mxesai_position(HW_CONTEXT_T * mx, PCM_SUBCHN_CONTEXT_T * pc, ado_pcm_config_t * config)
{
    uint32_t pos;

    ado_mutex_lock(&mx->hw_lock);

    if (pc == mx->play_strm.pcm_subchn)
    {
        pos =
            ado_pcm_dma_int_size(config) -
            mx->dmafuncs.bytes_left(mx->play_strm.dma_chn);
    } else {
        pos =
            ado_pcm_dma_int_size(config) -
            mx->dmafuncs.bytes_left(mx->cap_strm.dma_chn);
    }

    ado_mutex_unlock(&mx->hw_lock);

    return (pos);
}
#endif

/*
 * This function is used when more than 1 capture pcm devices created. It is called from
 * rx interrupt handler (or pulse) to split the hardware stream into multiple pcm streams
 *
 * Since this function is forced inline, when called with a const literal sampleSize (i.e. sizeof(int16_t))
 * the compiler can optimize away any branches based on the sampleSize.
 */
static inline void __attribute__ ((__always_inline__))
mxesai_dmacapture_split(HW_CONTEXT_T *mx, void *srcDMAAddr, const size_t size, const size_t sampleSize)
{
	uint32_t cnt = 0;
	uint32_t voice = 0;
	int32_t sample_offset = 0;
	int frame_size = mx->nslots * mx->num_sdi_pins;
	int16_t *buf16;
	int32_t *buf32;
	int16_t *dmaptr16 = srcDMAAddr;
	int32_t *dmaptr32 = srcDMAAddr;
	int src_idx;

	if((NULL == srcDMAAddr) || (0 == size))
	{
		ado_error_fmt("Invalid data (0x%lx) or size (%d)", (paddr_t)srcDMAAddr, size);
		return;
	}

	/* Invalidate cache */
	msync(srcDMAAddr, size, MS_INVALIDATE);

	if (mx->log_enabled)
		write(mx->cap_log_fd, srcDMAAddr, size);

	while ((sample_offset * sampleSize) < size)
	{
		for (cnt = 0; cnt < mx->num_rx_aif; cnt++ )
		{
			if (mx->aif[cnt].cap_strm.active)
			{
				if ((mx->aif[cnt].cap_strm.pcm_offset * sampleSize ) >= mx->aif[cnt].cap_strm.pcm_config->dmabuf.size) {
					mx->aif[cnt].cap_strm.pcm_offset = 0;
				}

				if (sampleSize == sizeof(int16_t)) {
					buf16 = (int16_t*) mx->aif[cnt].cap_strm.pcm_config->dmabuf.addr + mx->aif[cnt].cap_strm.pcm_offset;
				} else {
					buf32 = (int32_t*) mx->aif[cnt].cap_strm.pcm_config->dmabuf.addr + mx->aif[cnt].cap_strm.pcm_offset;
				}

				/* Reconstitute the frame */
				for (voice = 0; voice < mx->aif[cnt].cap_strm.voices; voice++)
				{
					src_idx = (cnt + (voice * mx->num_sdi_pins));
					if (cnt > 0)
						src_idx += (mx->aif[cnt-1].cap_strm.voices % mx->num_sdi_pins);

					if (sampleSize == sizeof(int16_t))
						buf16[voice] = dmaptr16[sample_offset + src_idx];
					else
						buf32[voice] = dmaptr32[sample_offset + src_idx];
				}

				/* Bump the pcm buffer offset */
				mx->aif[cnt].cap_strm.pcm_offset += mx->aif[cnt].cap_strm.voices;
				if (((mx->aif[cnt].cap_strm.pcm_offset * sampleSize) %
					 ado_pcm_dma_int_size(mx->aif[cnt].cap_strm.pcm_config)) == 0)
				{
					/* Signal to io-audio (DMA transfer was completed) */
					dma_interrupt(mx->aif[cnt].cap_strm.pcm_subchn);
				}
			}
		}
		sample_offset += frame_size;
	}
}

static void
mxesai_play_pulse_hdlr (HW_CONTEXT_T * mx, struct sigevent *event)
{
	int status = EOK;

	if ( (status = mx->dmafuncs.xfer_complete(mx->aif[0].play_strm.dma_chn)) != EOK)
	{
		uint32_t cnt;

		ado_error_fmt("DMA failure - 0x%x", status);
		for (cnt = 0; cnt < mx->num_tx_aif; cnt++ )
		{
			if (mx->aif[cnt].play_strm.pcm_subchn)
			{
				ado_pcm_error(mx->aif[cnt].play_strm.pcm_subchn, ADO_PCM_STATUS_ERROR);
			}
		}
	}
	else
	{
		if (mx->num_tx_aif == 1)
		{
			if (mx->aif[0].play_strm.pcm_subchn)
			{
				if (mx->log_enabled)
				{
					uint8_t *dmaptr;
					ado_pcm_config_t *config = mx->aif[0].play_strm.pcm_config;

					if (mx->playback_frag_index >= config->mode.block.frags_total)
						mx->playback_frag_index = 0;

					dmaptr = (uint8_t*)(&config->dmabuf.addr[mx->playback_frag_index * ado_pcm_dma_int_size(config)]);
					write(mx->play_log_fd, dmaptr, ado_pcm_dma_int_size(config));
					mx->playback_frag_index++;
				}
				dma_interrupt (mx->aif[0].play_strm.pcm_subchn);
			}
		}
		else
		{
			/* We use a ping-ping buffer for DMA, so divide buffer size by 2 to get the frag size */
			const size_t size = mx->playback_dmabuf.size / 2;

			if (mx->playback_frag_index >= 2)
				mx->playback_frag_index = 0;

			mxesai_dmaplayback_combine(mx, (void*)(&mx->playback_dmabuf.addr[mx->playback_frag_index * size]), size, true);
			mx->playback_frag_index++;
		}
	}
}

static void
mxesai_cap_pulse_hdlr (HW_CONTEXT_T * mx, struct sigevent *event)
{
	int status = EOK;

	if ( (status = mx->dmafuncs.xfer_complete(mx->aif[0].cap_strm.dma_chn)) != EOK)
	{
		uint32_t cnt;

		ado_error_fmt("DMA failure - 0x%x", status);
		for (cnt = 0; cnt < mx->num_rx_aif; cnt++ )
		{
			if (mx->aif[cnt].cap_strm.pcm_subchn)
			{
				ado_pcm_error(mx->aif[cnt].cap_strm.pcm_subchn, ADO_PCM_STATUS_ERROR);
			}
		}
	}
	else
	{
		if ( mx->num_rx_aif == 1)
		{
			if (mx->aif[0].cap_strm.pcm_subchn)
			{
				if (mx->log_enabled)
				{
					uint8_t *dmaptr;
					ado_pcm_config_t *config = mx->aif[0].cap_strm.pcm_config;

					if (mx->capture_frag_index >= config->mode.block.frags_total)
						mx->capture_frag_index = 0;

					dmaptr = (uint8_t*)(&config->dmabuf.addr[mx->capture_frag_index * ado_pcm_dma_int_size(config)]);
					write(mx->cap_log_fd, dmaptr, ado_pcm_dma_int_size(config));
					mx->capture_frag_index++;
				}
				dma_interrupt (mx->aif[0].cap_strm.pcm_subchn);
			}
		}
		else
		{
			/* We use a ping-ping buffer from DMA, so divide buffer size by 2 to get the frag size */
			const size_t size = mx->capture_dmabuf.size / 2;

			if (mx->capture_frag_index >= 2)
				mx->capture_frag_index = 0;

			if (mx->sample_size == sizeof(int16_t)) {
				mxesai_dmacapture_split(mx, (void*)
									(&mx->capture_dmabuf.addr[mx->capture_frag_index * size]), size, sizeof(int16_t));
			} else {
				mxesai_dmacapture_split(mx, (void *)
									(&mx->capture_dmabuf.addr[mx->capture_frag_index * size]), size, sizeof(int32_t));
			}
			mx->capture_frag_index++;
		}
	}
}

static int
mxesai_init (HW_CONTEXT_T * mx)
{
	/* person_reset_val contains a bitfield which specifies the direction
	 * of the ESAI pins and whether they are connected or not */
	uint32_t personal_reset_val = 0xfff;	/* All port pins set to ESAI function */
	uint32_t slot_bits;
	uint32_t voices, cnt;

	ado_mutex_lock(&mx->hw_lock);
	/* Enable ESAI */
	mx->esai->ecr = ECR_ESAIEN;

	/* Reset ESAI */
	mx->esai->ecr = ECR_ESAIEN | ECR_ERST;
	delay (1);

	/* Clear Reset */
	mx->esai->ecr = ECR_ESAIEN;

	/* Disconnect ESAI port */
	mx->esai->pcrc = 0;
	mx->esai->prrc = 0;

	mx->esai->tccr = mx->esai->rccr = 0;

	if (mx->async_clks == 0)
	{
		/* Synchronous mode (RX shares clocks with TX) */
		mx->esai->saicr = SAICR_SYNC;
		/* Synchronous mode, disconnect receive clock pins as tx clocks shared with rx function block */
		personal_reset_val &= ~(PORT0_SCKR | PORT1_FSR | PORT2_HCKR);
	}
	else
	{
		/* Asynchronous mode (TX and RX clocks are independent) */
		mx->esai->saicr &= ~(SAICR_SYNC);
	}

	if (mx->sample_size == 2)
	{
		/* Left align to bit 15 (i.e. 16bits) */
		mx->esai->saicr |= SAICR_ALC;
	}
	else
	{
		/* Left align to bit 23 (i.e. 24bits) */
		mx->esai->saicr &= ~SAICR_ALC;
	}

	if (mx->bit_delay)
	{
		/* 1-bit data delay */
		mx->esai->tcr |= TCR_TFSR;
		mx->esai->rcr |= RCR_RFSR;
	}

	if (mx->fs_active_width == FS_BIT)
	{
		mx->esai->tcr |= TCR_TFSL;
		mx->esai->rcr |= RCR_RFSL;
	}

	switch (mx->slot_size)
	{
		case 16:
			slot_bits = 0x2;		/* Slot length 16, word length = 16 */
			break;
		case 32:
		default:
			if (mx->sample_size == 2)
				slot_bits = 0x12;   /* Slot length 32, word length = 16 */
			else
				slot_bits = 0x1f;	/* Slot length 32, word length = 24 */
			break;
	}

	/*
	 * word length frame sync; 16 or 24 bit word length; slot size 16 or 32 bits,
	 * network mode, Transmit zero padding
	 */
	mx->esai->tcr |= TCR_TSWS(slot_bits) | TCR_TMOD (1) | TCR_PADC;
	mx->esai->rcr |= RCR_RSWS(slot_bits) | RCR_RMOD (1);

	/* RX Slot mask */
	for (cnt = 0, voices = 0; cnt < mx->num_rx_aif; cnt++)
		voices += mx->aif[cnt].cap_strm.voices;
	for (cnt = 0; cnt < voices; cnt++)
		mx->rx_slot_mask |= (1 << cnt);
	/* Disable all RX slots (will be enabled in trigger go)
	 * Note: Always program rsmb first, followed by rsma (refer to ESAI Initialization chapter in iMX8 reference manual)
	 */
	mx->esai->rsmb = 0;
	mx->esai->rsma = 0;

	/* TX Slot mask */
	for (cnt = 0, voices = 0; cnt < mx->num_tx_aif; cnt++)
		voices += mx->aif[cnt].play_strm.voices;
	for (cnt = 0; cnt < voices; cnt++)
		mx->tx_slot_mask |= (1 << cnt);
	/* Disable all TX slots (will be enabled in trigger go)
	 * Note: Always program tsmb first, followed by tsma (refer to ESAI Initialization chapter in iMX8 reference manual)
	 */
	mx->esai->tsmb = 0;
	mx->esai->tsma = 0;

#if defined(VARIANT_mx6q_ddr3)
	char mixer_params[100];
	/*
	 * Inform mixer that the SOC is the master. Note that the i.MX6 Q DDR3 evaluation board
	 * requires that the ESAI module be the clock master in order for the codec to work correctly.
	 */
	sprintf(mixer_params, ":mode=master");
	strlcat(mx->mixeropts, mixer_params, sizeof(mx->mixeropts));

	/* Inform mixer that the codec is using Left Justified format */
	sprintf(mixer_params, ":format=1");
	strlcat(mx->mixeropts, mixer_params, sizeof(mx->mixeropts));
#endif

	mxesai_set_clock_rate(mx,  mx->sample_rate_max);
	ado_debug(DB_LVL_DRIVER, "mclk = %d, sample rate = %d, nslots = %d, slot size = %d, word size = %s",
			  mx->main_clk_freq, mx->sample_rate, mx->nslots, mx->slot_size, slot_bits == 0x1f ? "24" : "16");

	/* Set which ports connect to ESAI*/
	mx->esai->prrc = personal_reset_val;
	mx->esai->pcrc = personal_reset_val;

	/* Put TX FIFO into reset
	 * NOTE: Because the RX runs off the TX clock (Synchronous clocks) we cannot put
	 * the transmitter into personal reset
	 */
	mx->esai->tfcr = TFCR_TFR;

	/* Put the RX FIFO into reset (does not affect control bits) */
	mx->esai->rfcr = RFCR_RFR;
	ado_mutex_unlock(&mx->hw_lock);

	return 0;
}

ado_dll_version_t ctrl_version;
void
ctrl_version (int *major, int *minor, char *date)
{
	*major = ADO_MAJOR_VERSION;
	*minor = 1;
	date = __DATE__;
	(void)(date); /* Unused */
}

/*
esaibase = [#]; base address of esai controller
tevt    = [#]; esai TX DMA event number
tchn    = [#]; esai TX DMA channel type
revt    = [#]; esai RX DMA event number
rchn    = [#]; esai RX DMA channel type
rate    = [#]; sample rate of audio
mixer = [[mixer option1]:[mixer options2][:[other options]]:[info]]
   mixer=info to dump the details of mixer options
*/

static char *
mxesai_opts[] = {
#define OPT_ESAIBASE      0
    "esaibase",
#define OPT_TEVT          1
    "tevt",
#define OPT_TCHN          2
    "tchn",
#define OPT_REVT          3
    "revt",
#define OPT_RCHN          4
    "rchn",
#define OPT_RATE          5
    "rate",
#define OPT_MAIN_CLK_SRC  6
    "main_clk_src",
#define OPT_MAIN_CLK_FREQ 7
    "main_clk_freq",
#define OPT_MIXER         8
    "mixer",
#define OPT_RX_VOICES     9
    "rx_voices",
#define OPT_I2C_BUS       10
    "i2c_bus",
#define OPT_I2C_ADDR      11
    "i2c_addr",
#define OPT_SDO_PINS      12
    "sdo_pin_map",
#define OPT_SDI_PINS      13
    "sdi_pin_map",
#define OPT_CLK_MODE      14
    "clk_mode",
#define OPT_NSLOTS        15
    "nslots",
#define OPT_DEBUG         16
    "debug",
#define OPT_RX_FRAG_SIZE  17
    "rx_frag_size",
#define OPT_TX_FRAG_SIZE  18
    "tx_frag_size",
#define OPT_PROTOCOL      19
    "protocol",
#define OPT_BIT_DELAY     20
    "bit_delay",
#define OPT_FSYNC_POL     21
    "fsync_pol",
#define OPT_SLOT_SIZE     22
    "slot_size",
#define OPT_TX_VOICES     23
    "tx_voices",
#define OPT_XCLK_POL      24
    "xclk_pol",
#define OPT_RCLK_POL      25
    "rclk_pol",
#define OPT_TX_CHMAP      26    /* Deprecated - Use tx_chmap */
    "sdo_chmap",
#define OPT_RX_CHMAP      27    /* Deprecated - Use rx_chmap */
    "sdi_chmap",
#define OPT_FS_ACTIVE_WIDTH  28
    "fs_active_width",
#define OPT_ASYNC_CLKS    29
    "async_clks",
#define OPT_SAMPLE_SIZE   30
    "sample_size",
#define OPT_TX_PCM_CHMAP  31
    "tx_chmap",
#define OPT_RX_PCM_CHMAP  32
    "rx_chmap",
    NULL
};

#if !(IMX_EDMA)
static void
build_dma_string (char * dmastring, unsigned dma_string_size, uint32_t fifopaddr, int dmaevent, int watermark)
{
	char str[50];

	strlcpy (dmastring, "eventnum=", dma_string_size);
	strlcat (dmastring, itoa (dmaevent, str, 10), dma_string_size);
	strlcat (dmastring, ",watermark=", dma_string_size);
	strlcat (dmastring, itoa (watermark, str, 10), dma_string_size);
	strlcat (dmastring, ",fifopaddr=0x", dma_string_size);
	strlcat (dmastring, ultoa (fifopaddr, str, 16), dma_string_size);
	strlcat (dmastring, ",regen,contloop", dma_string_size);
}
#endif
/* This function configures the various protocol specific flags
 */
static void configure_default_protocol_flags(HW_CONTEXT_T * mx)
{
	switch (mx->protocol)
	{
		case PROTOCOL_PCM:
			mx->bit_delay = 0;		/* 0-bit clock delay	   */
			mx->fsync_pol = 1;		/* Active high frame sync  */
			mx->xclk_pol = 1;		/* TX on BCLK rising edge  */
			mx->rclk_pol = 1;		/* TX on BCLK rising edge  */
			break;
		case PROTOCOL_I2S:
			mx->bit_delay = 1;		/* 1-bit clock delay	   */
			mx->fsync_pol = 0;		/* Active low frame sync   */
			mx->xclk_pol = 0;		/* TX on BCLK falling edge */
			mx->rclk_pol = 1;		/* RX on BCLK rising edge  */
			break;
		default:
			break;
	}
}

/* Convert string list of pins into a bit map */
static int16_t parse_pin_list(const char* pin_list, int16_t* pin_bit_map)
{
	uint32_t idx, cnt;
	char *next;
	int pin_number;

	/* Count number of pins in the pin_list */
	for (idx = 0, cnt = 1; idx < strlen(pin_list); (pin_list[idx] == ':') ? cnt++ : 0, idx++);

	/* Convert first pin into a bit map */
	pin_number = strtoul(pin_list, 0, 0);
	*pin_bit_map = 0x1 << pin_number;

	idx = 1;	/* Skip first pin since we handled it above */
	/* Convert the remaining list of pins (if any) into a bit map */
	while (idx < cnt && (next = strchr(pin_list, ':')) != NULL)
	{
		pin_number = strtoul(next + 1, 0, 0);
		*pin_bit_map |= 0x1 <<	pin_number;
		pin_list = next+1;
	}

	return cnt;
}

static int query_hwi_device(HW_CONTEXT_T *mx, unsigned unit)
{
	/* Getting the ESAI Base addresss from the Hwinfo Section if available */
	unsigned hwi_off = hwi_find_device("esai", unit);
#if IMX_EDMA
	uint32_t zero = 0;
#endif

	if (hwi_off != HWI_NULL_OFF)
	{
		hwi_tag *tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_location, 0);
		if (tag)
		{
			mx->esaibase = tag->location.base;
		}
#if IMX_EDMA
		tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_dma, &zero);
		if (tag)
		{
			mx->rchn = tag->dma.chnl;
		} else {
			ado_error_fmt("MX_ESAI: DMA requests not found in hwinfo table");
			return -1;
		}
		tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_dma, &zero);
		if (tag)
		{
			mx->tchn = tag->dma.chnl;
		} else {
			ado_error_fmt("MX_ESAI: DMA requests not found in hwinfo table");
			return -1;
		}
#endif
		return 0;
	}
	return -1;
}

static const char *inputsplitter_opts[] = {
#define INPUT_SPLITTER_MS         0
		"input_splitter_ms",
#define INPUT_SPLITTER_SAMPLES    1
		"input_splitter_samples",
#define INPUT_SPLITTER_ENABLE     2
		"input_splitter_enable",
		NULL
};

#define N_OPTS ((sizeof(inputsplitter_opts)/sizeof(inputsplitter_opts[0])) - 1U)

/*
 * Below function search the dictionary to find the matching input_splitter_ms and input_splitter_samples config
 */
static int mxesai_parse_inputsplitter_config(HW_CONTEXT_T * mx, uint32_t *inputsplitter_ms, uint32_t *inputsplitter_samples)
{
	const ado_dict_t *dict = NULL;
	const char* optValues[N_OPTS] = {0};
	bool enable_input_splitter = false;

	*inputsplitter_ms = 0;
	*inputsplitter_samples = 0;

	dict = ado_get_card_dict( mx->card );
	if (dict)
	{
		ado_config_load_key_values(dict, inputsplitter_opts, optValues, N_OPTS, 0, 0);

		if (optValues[INPUT_SPLITTER_ENABLE])
		{
			enable_input_splitter = (strtoul(optValues[INPUT_SPLITTER_ENABLE], NULL, 0) != 0) ? true : false;
			if (enable_input_splitter)
			{
				if (optValues[INPUT_SPLITTER_MS])
					*inputsplitter_ms = strtoul (optValues[INPUT_SPLITTER_MS], NULL, 0);

				if (optValues[INPUT_SPLITTER_SAMPLES])
					*inputsplitter_samples = strtoul (optValues[INPUT_SPLITTER_SAMPLES], NULL, 0);
			}
		}
		ado_debug(DB_LVL_DRIVER,"inputsplitter_ms %d, inputsplitter_samples %d", *inputsplitter_ms, *inputsplitter_samples);
	}
	return 0;
}

static const char *swmixer_opts[] = {
#define SW_MIXER_MS         0
		"sw_mixer_ms",
#define SW_MIXER_SAMPLES    1
		"sw_mixer_samples",
#define DISABLE_SW_MIXER    2
		"disable_sw_mixer",
		NULL
};

#define N_SWMIXER_OPTS ((sizeof(swmixer_opts)/sizeof(swmixer_opts[0])) - 1U)

/*
 * Below function search the dictionary to find the matching sw_mixer_ms and sw_mixer_samples config
 */
static int mxesai_parse_swmixer_config(HW_CONTEXT_T * mx, uint32_t *swmixer_ms, uint32_t *swmixer_samples)
{
	const ado_dict_t *dict = NULL;
	const char* optValues[N_SWMIXER_OPTS] = {0};
	bool disable_sw_mixer = false;

	*swmixer_ms = 0;
	*swmixer_samples = 0;

	dict = ado_get_card_dict( mx->card );
	if (dict)
	{
		ado_config_load_key_values(dict, swmixer_opts, optValues, N_SWMIXER_OPTS, 0, 0);

		if (optValues[DISABLE_SW_MIXER])
		{
			disable_sw_mixer = (strtoul(optValues[DISABLE_SW_MIXER], NULL, 0) != 0) ? false : true;
		}
		else
		{
			disable_sw_mixer = global_options.disable_sw_mixer;
		}

		if (!disable_sw_mixer)
		{
			if (optValues[SW_MIXER_MS]) {
				*swmixer_ms = strtoul (optValues[SW_MIXER_MS], NULL, 0);
			}
			else if (global_options.sw_mixer_ms > 0)
			{
				*swmixer_ms = global_options.sw_mixer_ms;
			}

			if (optValues[SW_MIXER_SAMPLES]) {
				*swmixer_samples = strtoul (optValues[SW_MIXER_SAMPLES], NULL, 0);
			}
			else if (global_options.sw_mixer_samples > 0)
			{
				*swmixer_samples = global_options.sw_mixer_samples;
			}
		}
		ado_debug(DB_LVL_DRIVER,"swmixer_ms %d, swmixer_samples %d", *swmixer_ms, *swmixer_samples);
	}
	return 0;
}

#define LOCAL_STR_LEN 60
static int mxesai_parse_commandline(HW_CONTEXT_T * mx, char *args)
{
	int      opt = 0;
	char	 *value;
	char	 *tx_voices = NULL;
	char	 *rx_voices = NULL;
	char	 *sdo_pin_map = NULL;
	char	 *sdi_pin_map = NULL;
	char	 local_string[LOCAL_STR_LEN];
	uint32_t idx = 0;
	int      tx_aif_cnt = 0, rx_aif_cnt = 0;
	snd_pcm_chmap_t *tx_chmap[MAX_NUM_AIFS] = { NULL };
	snd_pcm_chmap_t *rx_chmap[MAX_NUM_AIFS] = { NULL };

	strlcpy(local_string, "", sizeof(local_string));

#if defined(ESAI_BASE_ADDR)
	mx->esaibase = ESAI_BASE_ADDR;
#endif
	strlcpy (mx->mixeropts, "", sizeof(mx->mixeropts));
#ifdef ESAI_TX_DMA_EVENT
	mx->tevt = ESAI_TX_DMA_EVENT;
#endif
#ifdef ESAI_TX_DMA_CTYPE
	mx->tchn = ESAI_TX_DMA_CTYPE;
#endif
#ifdef ESAI_RX_DMA_EVENT
	mx->revt = ESAI_RX_DMA_EVENT;
#endif
#ifdef ESAI_RX_DMA_CTYPE
	mx->rchn = ESAI_RX_DMA_CTYPE;
#endif
	mx->sample_rate_min = SAMPLE_RATE_MIN;
	mx->sample_rate_max = SAMPLE_RATE_MAX;
	mx->main_clk_src = ESAI_MAIN_CLK_SRC;
#if defined(ESAI_CLK_MODE)
	mx->clk_mode = ESAI_CLK_MODE;
#else
	mx->clk_mode = ESAI_CLK_SLAVE;
#endif

#if defined(ESAI_NUM_SLOTS)
	mx->nslots = ESAI_NUM_SLOTS;
#else
	mx->nslots = 2;
#endif

#if defined(ESAI_SLOT_SIZE)
	mx->slot_size = ESAI_SLOT_SIZE;
#else
	mx->slot_size = 32;
#endif
#if defined SAMPLE_SIZE
	mx->sample_size = SAMPLE_SIZE;
#else
	mx->sample_size = 2;
#endif
#if defined(ESAI_MAIN_CLK_FREQ)
	 mx->main_clk_freq = ESAI_MAIN_CLK_FREQ;
#endif
#if defined(I2C_BUS_NUMBER)
	mx->i2c_bus = I2C_BUS_NUMBER;
#else
	mx->i2c_bus = -1;
#endif
#if defined(I2C_SLAVE_ADDR)
	mx->i2c_addr = I2C_SLAVE_ADDR;
#else
	mx->i2c_addr = -1;
#endif
#if defined SDO_PIN_MAP
	sdo_pin_map = SDO_PIN_MAP;
#else
	sdo_pin_map = "0";
#endif
#if defined SDI_PIN_MAP
	sdi_pin_map = SDI_PIN_MAP;
#else
	sdi_pin_map = "0";
#endif

#if defined ESAI_PROTOCOL
	mx->protocol = ESAI_PROTOCOL;
#else
	mx->protocol = PROTOCOL_I2S;
#endif
	configure_default_protocol_flags(mx);
	mx->log_enabled = 0;
	mx->rx_frag_size = mx->tx_frag_size = 4 * 1024;

#if defined ESAI_FS_ACTIVE_WIDTH
	mx->fs_active_width = ESAI_FS_ACTIVE_WIDTH;
#else
	mx->fs_active_width = FS_WORD;
#endif

#if defined SDO_PINS
	#warning "SDO_PINS is deprecated and should be replaced with SDO_PIN_MAP"
#endif

#if defined SDI_PINS
	#warning "SDI_PINS is deprecated and should be replaced with SDI_PIN_MAP"
#endif

	while (*args != '\0')
	{
		switch ((opt = getsubopt (&args, mxesai_opts, &value)))
		{
			case OPT_ESAIBASE:
				mx->esaibase = strtoul (value, NULL, 0);
				break;
			case OPT_TEVT:
				mx->tevt = strtol (value, NULL, 0);
				break;
			case OPT_TCHN:
				mx->tchn = strtol (value, NULL, 0);
				break;
			case OPT_REVT:
				mx->revt = strtol (value, NULL, 0);
				break;
			case OPT_RCHN:
				mx->rchn = strtol (value, NULL, 0);
				break;
			case OPT_RATE:
				{
					char *value2;
					mx->sample_rate_min = mx->sample_rate_max = strtoul(value, 0, 0);
					if (ado_pcm_rate2flag(mx->sample_rate_min) == 0)
					{
						ado_error_fmt("Invalid sample rate - %d", mx->sample_rate_min);
						goto cleanup_fail;
					}
					if ((value2 = strchr(value, ':')) != NULL)
					{
						mx->sample_rate_max = strtoul(value2 + 1, 0, 0);
						if (ado_pcm_rate2flag(mx->sample_rate_max) == 0)
						{
							ado_error_fmt("Invalid sample rate - %d", mx->sample_rate_max);
							goto cleanup_fail;
						}
					}
				}
				break;
			case OPT_MAIN_CLK_SRC:
				mx->main_clk_src = strtol (value, NULL, 0);
				break;
			case OPT_MAIN_CLK_FREQ:
				mx->main_clk_freq = strtol (value, NULL, 0);
				break;
			case OPT_MIXER:
				if (strlen (value) > MAX_MIXEROPT)
				{
					ado_error_fmt("Board specific options pass maximum len %d",
							MAX_MIXEROPT);
					goto cleanup_fail;
				}
				strlcat (mx->mixeropts, value, MAX_MIXEROPT);
				break;
			case OPT_RX_VOICES:
				if (value != NULL)
					rx_voices = ado_strdup(value);
				break;
			case OPT_TX_VOICES:
				if (value != NULL)
					tx_voices = ado_strdup(value);
				break;
			case OPT_I2C_BUS:
				mx->i2c_bus = strtol (value, NULL, 0);
				break;
			case OPT_I2C_ADDR:
				mx->i2c_addr = strtol (value, NULL, 0);
				break;
			case OPT_CLK_MODE:
				if (value && *value != '\0')
				{
					if (strcmp(value, "master") == 0)
					{
						mx->clk_mode = ESAI_CLK_MASTER;
						ado_debug (DB_LVL_DRIVER, "Audio clock mode = Master");
					}
					else if (strcmp(value, "slave") == 0)
					{
						mx->clk_mode = ESAI_CLK_SLAVE;
						ado_debug (DB_LVL_DRIVER, "Audio clock mode = Slave");
					}
				}
				break;
			case OPT_NSLOTS:
				mx->nslots = strtol (value, NULL, 0);
				if(mx->nslots > MAX_NSLOTS)
					mx->nslots = MAX_NSLOTS;
				break;
			case OPT_DEBUG:
				mx->log_enabled = 1;
				break;
			case OPT_RX_FRAG_SIZE:
				if (value && *value != '\0')
					mx->rx_frag_size = strtol (value, NULL, 0);
				break;
			case OPT_TX_FRAG_SIZE:
				if (value && *value != '\0')
					mx->tx_frag_size = strtol (value, NULL, 0);
				break;
			case OPT_PROTOCOL:
				if (value && *value != '\0')
				{
					if (strcmp(value, "i2s") == 0)
						mx->protocol = PROTOCOL_I2S;
					else
						mx->protocol = PROTOCOL_PCM;

					configure_default_protocol_flags(mx);
				}
				break;
			case OPT_BIT_DELAY:
				if (value && *value != '\0')
				{
					mx->bit_delay = atoi (value);
					if (mx->bit_delay > 1 || mx->bit_delay < 0)
					{
						ado_error_fmt("Invalid bit_delay value (0 or 1)");
						goto cleanup_fail;
					}
				}
				break;
			case OPT_FSYNC_POL:
				if (value && *value != '\0')
				{
					mx->fsync_pol = atoi (value);
					if (mx->fsync_pol > 1 || mx->fsync_pol < 0)
					{
						ado_error_fmt("Invalid fsync_pol value");
						goto cleanup_fail;
					}
				}
				break;
			case OPT_SLOT_SIZE:
				if (value && *value != '\0')
				{
					mx->slot_size = atoi (value);
					if (mx->slot_size != 16 && mx->slot_size != 32)
					{
						ado_error_fmt("Invalid slot_size value (16 or 32)");
						goto cleanup_fail;
					}
				}
				break;
			case OPT_SAMPLE_SIZE:
				if (value && *value != '\0')
				{
					mx->sample_size = atoi (value);
					switch (mx->sample_size)
					{
						case 2:
						case 16:
							mx->sample_size = 2;
							break;
						case 4:
						case 32:
							mx->sample_size = 4;
							 break;
						default:
							ado_error_fmt("Invalid sample_size value (16 or 32)");
							goto cleanup_fail;
					}
				}
				break;
			case OPT_XCLK_POL:
				if (value && *value != '\0')
				{
					mx->xclk_pol = atoi (value);
					if (mx->xclk_pol > 1 || mx->xclk_pol < 0)
					{
						ado_error_fmt("Invalid xclk_pol value");
						goto cleanup_fail;
					}
				}
				break;
			case OPT_RCLK_POL:
				if (value && *value != '\0')
				{
					mx->rclk_pol = atoi (value);
					if (mx->rclk_pol > 1 || mx->rclk_pol < 0)
					{
						ado_error_fmt("Invalid rclk_pol value");
						goto cleanup_fail;
					}
				}
				break;
			case OPT_TX_CHMAP:
				ado_error_fmt("Deprecated, use tx_chmap");
				goto cleanup_fail;
				break;
			case OPT_RX_CHMAP:
				ado_error_fmt("Deprecated, use rx_chmap");
				goto cleanup_fail;
				break;
			case OPT_SDO_PINS:
				sdo_pin_map = strdup (value);
				break;
			case OPT_SDI_PINS:
				sdi_pin_map = strdup (value);
				break;
			case OPT_FS_ACTIVE_WIDTH:
				if (value && *value != '\0')
				{
					if (strcmp(value, "word") == 0)
						mx->fs_active_width = FS_WORD;
					else if (strcmp(value, "bit") == 0)
						mx->fs_active_width = FS_BIT;
				}
				break;
			case OPT_ASYNC_CLKS:
				mx->async_clks = 1;
				break;

			case OPT_TX_PCM_CHMAP:
				if (value != NULL && tx_aif_cnt < MAX_NUM_AIFS)
				{
					tx_chmap[tx_aif_cnt] = ado_pcm_parse_chmap(value);
					if (tx_chmap[tx_aif_cnt] == NULL)
					{
						goto cleanup_fail;
					}
					tx_aif_cnt++;
				}
				else
				{
					ado_error_fmt("Invalid option %s=%s", mxesai_opts[opt], value ? value : "");
					goto cleanup_fail;
				}
				break;
			case OPT_RX_PCM_CHMAP:
				if (value != NULL && rx_aif_cnt < MAX_NUM_AIFS)
				{
					rx_chmap[rx_aif_cnt] = ado_pcm_parse_chmap(value);
					if (rx_chmap[rx_aif_cnt] == NULL)
					{
						goto cleanup_fail;
					}
					rx_aif_cnt++;
				}
				else
				{
					ado_error_fmt("Invalid option %s=%s", mxesai_opts[opt], value ? value : "");
					goto cleanup_fail;
				}
				break;
			default:
				ado_error_fmt("Unsupported option '%s'", value);
				break;
		}
	}
	if (mx->esaibase < mxesai_port_count())
	{
		/* Obtain base address and DMA requests from hwinfo table */
		if (query_hwi_device(mx, mx->esaibase) == -1)
		{
			goto cleanup_fail;
		}
	}

	if (mx->slot_size < (mx->sample_size * _BITS_BYTE))
	{
		ado_error_fmt("Slot size (%d) must be >= sample size (%d)", mx->slot_size, (mx->sample_size * _BITS_BYTE));
		goto cleanup_fail;
	}

	{
		char *value2;
		vuint32_t rx_cnt, tx_cnt;

		/* Determine the number of pins in use and convert to a bit-map */
		mx->num_sdo_pins = parse_pin_list(sdo_pin_map, &mx->sdo_pins);
		if (mx->num_sdo_pins > MAX_NUM_SDO_PINS)
		{
			ado_error_fmt("Cannot allocate more than %d SDO pins", MAX_NUM_SDO_PINS);
			goto cleanup_fail;
		}

		mx->num_sdi_pins = parse_pin_list(sdi_pin_map, &mx->sdi_pins);
		if (mx->num_sdi_pins > MAX_NUM_SDI_PINS)
		{
			ado_error_fmt("Cannot allocate more than %d SDI pins", MAX_NUM_SDI_PINS);
			goto cleanup_fail;
		}

		/* Count number of TX interfaces */
		value = (tx_voices == NULL) ? AIF_TX_VOICES : tx_voices;
		for (idx = 0, tx_cnt = 1; idx < strlen(value); (value[idx] == ':') ? tx_cnt++ : 0, idx++);
		/* Count number of RX interfaces */
		value2 = (rx_voices == NULL) ? AIF_RX_VOICES : rx_voices;
		for (idx = 0, rx_cnt = 1; idx < strlen(value2); (value2[idx] == ':') ? rx_cnt++ : 0, idx++);
		/* Allocate Audio interfaces */
		mx->num_tx_aif = strtoul(value, NULL, 0) == 0 ? 0 : tx_cnt;
		mx->num_rx_aif = strtoul(value2, NULL, 0) == 0 ? 0 : rx_cnt;

		if (mx->num_tx_aif > MAX_NUM_AIFS)
		{
			ado_error_fmt("Number of TX interfaces (%d) must be less then %d", mx->num_tx_aif, MAX_NUM_AIFS);
			goto cleanup_fail;
		}

		if (mx->num_rx_aif > MAX_NUM_AIFS)
		{
			ado_error_fmt("Number of RX interfaces (%d) must be less then %d", mx->num_rx_aif, MAX_NUM_AIFS);
			goto cleanup_fail;
		}

		/* Ensure one pin per interface when using multiple pins and interfaces */
		if ((mx->num_sdo_pins > 1) && (mx->num_tx_aif > 1) && (mx->num_sdo_pins != mx->num_tx_aif))
		{
			ado_error_fmt("When multiple sdo pins and pseudo playback interfaces are used, the number "
			              "of pins (%d) must match the number of interfaces (%d)", mx->num_sdo_pins, mx->num_tx_aif);
			goto cleanup_fail;
		}
		/* Ensure one pin per interface when using multiple pins and interfaces */
		if ((mx->num_sdi_pins > 1) && (mx->num_rx_aif > 1) && (mx->num_sdi_pins != mx->num_rx_aif))
		{
			ado_error_fmt("When multiple sdi pins and pseudo capture interfaces are used, the number "
			              "of pins (%d) must match the number of interfaces (%d)", mx->num_sdi_pins, mx->num_rx_aif);
			goto cleanup_fail;
		}

		if ((mx->aif = (mxesai_aif_t *) ado_calloc (max(mx->num_tx_aif, mx->num_rx_aif), sizeof (mxesai_aif_t))) == NULL)
		{
			ado_error_fmt("Unable to allocate memory for mxesai (%s)", strerror (errno));
			goto cleanup_fail;
		}

		value = (tx_voices == NULL) ? AIF_TX_VOICES : tx_voices;

		mx->aif[0].play_strm.voices = strtoul(value, 0, 0);
		if (tx_chmap[0] && (tx_chmap[0]->channels != mx->aif[0].play_strm.voices))
		{
			ado_error_fmt("Tx[%d] channels %d does not match tx_chmap channels %d",
						0, mx->aif[0].play_strm.voices, tx_chmap[0]->channels);
			goto cleanup_fail;

		}
		else if ((mx->num_sdo_pins > 1) && (mx->num_tx_aif > 1) && (mx->aif[0].play_strm.voices > mx->nslots))
		{
			ado_error_fmt("When multiple sdo pins are used with multiple playabck pseudo interfaces, "
			              "playback voices (%d) must be <= nslots (%d)", mx->aif[0].play_strm.voices, mx->nslots);
			goto cleanup_fail;
		}
		else if (tx_chmap[0])
		{
			mx->aif[0].play_strm.chmap = tx_chmap[0];
		}

		idx = 1;	/* Skip first character since we handled it above */
		while (idx < tx_cnt && (value2 = strchr(value, ':')) != NULL)
		{
			mx->aif[idx].play_strm.voices = strtoul(value2 + 1, 0, 0);
			if (tx_chmap[idx] && (tx_chmap[idx]->channels != mx->aif[idx].play_strm.voices))
			{
				ado_error_fmt("Tx[%d] channels %d does not match tx_chmap channels %d",
				              idx, mx->aif[idx].play_strm.voices, tx_chmap[idx]->channels);
				goto cleanup_fail;

			}
			else if ((mx->num_sdo_pins > 1) && (mx->aif[idx].play_strm.voices != mx->aif[idx-1].play_strm.voices))
			{
				ado_error_fmt("Number of voices must be the same for all playback pseudo interfaces when using multiple pins");
				goto cleanup_fail;
			}
			else if (tx_chmap[idx])
			{
				mx->aif[idx].play_strm.chmap = tx_chmap[idx];
			}
			idx++;
			value = value2+1;
		}

		value = (rx_voices == NULL) ? AIF_RX_VOICES : rx_voices;

		mx->aif[0].cap_strm.voices = strtoul(value, 0, 0);
		if (rx_chmap[0] && (rx_chmap[0]->channels != mx->aif[0].cap_strm.voices))
		{
			ado_error_fmt("Rx[%d] channels %d does not match rx_chmap channels %d",
			              0, mx->aif[0].cap_strm.voices, rx_chmap[0]->channels);
			goto cleanup_fail;
		}
		else if ((mx->num_sdi_pins > 1) && (mx->num_rx_aif > 1) && (mx->aif[0].cap_strm.voices > mx->nslots))
		{
			ado_error_fmt("When multiple sdi pins are used with multiple capture pseudo interfaces, "
			              "capture voices be <= nslots (%d/%d)", mx->aif[0].cap_strm.voices, mx->nslots);
			goto cleanup_fail;
		}
		else if (rx_chmap[0])
		{
			mx->aif[0].cap_strm.chmap = rx_chmap[0];
		}

		idx = 1;	/* Skip first character since we handled it above */
		while (idx < rx_cnt && (value2 = strchr(value, ':')) != NULL)
		{
			mx->aif[idx].cap_strm.voices = strtoul(value2 + 1, 0, 0);
			if (rx_chmap[idx] && (rx_chmap[idx]->channels != mx->aif[idx].cap_strm.voices))
			{
				ado_error_fmt("Rx[%d] channels %d does not match rx_chmap channels %d",
				              idx, mx->aif[idx].cap_strm.voices, rx_chmap[idx]->channels);
				goto cleanup_fail;

			}
			else if ((mx->num_sdi_pins > 1) && (mx->aif[idx].cap_strm.voices != mx->aif[idx-1].cap_strm.voices))
			{
				ado_error_fmt("Number of voices must be the same for all capture pseudo interfaces when using multiple pins");
				goto cleanup_fail;
			}
			else if (rx_chmap[idx])
			{
				mx->aif[idx].cap_strm.chmap = rx_chmap[idx];
			}
			idx++;
			value = value2+1;
		}
		/* Free strdup'd strings */
		if (tx_voices != NULL) {
			ado_free(tx_voices);
			tx_voices = NULL;
		}
		if (rx_voices != NULL) {
			ado_free(rx_voices);
			rx_voices = NULL;
		}
	}

	if (mx->i2c_bus != -1)
	{
		if (mx->mixeropts[0] != '\0')
			sprintf(local_string, ":dev=%d", mx->i2c_bus);
		else
			sprintf(local_string, "dev=%d", mx->i2c_bus);
		if (strlcat (mx->mixeropts, local_string, sizeof(mx->mixeropts)) >= sizeof(mx->mixeropts)) {
			ado_error_fmt("Out of space when building the mixeropts string");
			goto cleanup_fail;
		}
	}

	if (mx->i2c_addr != -1)
	{
		if (mx->mixeropts[0] != '\0')
			sprintf(local_string, ":addr=0x%x", mx->i2c_addr);
		else
			sprintf(local_string, "addr=0x%x", mx->i2c_addr);
		if (strlcat (mx->mixeropts, local_string, sizeof(mx->mixeropts)) >= sizeof(mx->mixeropts)) {
			ado_error_fmt("Out of space when building the mixeropts string");
			goto cleanup_fail;
		}
	}

	if (mx->mixeropts[0] != '\0')
		sprintf(local_string, ":rx_voices=%d", mx->aif[0].cap_strm.voices);
	else
		sprintf(local_string, "rx_voices=%d", mx->aif[0].cap_strm.voices);
	if (strlcat (mx->mixeropts, local_string, sizeof(mx->mixeropts)) >= sizeof(mx->mixeropts)) {
		ado_error_fmt("Out of space when building the mixeropts string");
		goto cleanup_fail;
	}

	return EOK;

cleanup_fail:
	for (idx = 0; idx < MAX_NUM_AIFS; idx++)
	{
		if (tx_chmap[idx])
			ado_free(tx_chmap[idx]);
		if (rx_chmap[idx])
			ado_free(rx_chmap[idx]);
	}

	if (tx_voices != NULL)
		ado_free(tx_voices);
	if (rx_voices != NULL)
		ado_free(rx_voices);

	ado_free(mx->aif);
	return (-1);
}

/* defaults if not set in variant.h */
#ifndef MXESAI_FRAME_RATE_LIST1
	#define MXESAI_FRAME_RATE_LIST1	{ SND_PCM_RATE_8000, SND_PCM_RATE_16000, SND_PCM_RATE_32000, SND_PCM_RATE_48000 }
#endif

#ifndef MXESAI_FRAME_RATE_LIST2
	#define MXESAI_FRAME_RATE_LIST2	{ SND_PCM_RATE_11025, SND_PCM_RATE_22050, SND_PCM_RATE_44100, SND_PCM_RATE_88200 }
#endif

ado_ctrl_dll_init_t ctrl_init;
int
ctrl_init (HW_CONTEXT_T ** hw_context, ado_card_t * card, char *args)
{
	uint32_t cnt = 0;
	mxesai_t * mx;
	int i;
	uint32_t rate;
#if !IMX_EDMA
	off_t physaddr;
#endif
	char	 str[100] = { 0 };
	dma_driver_info_t dma_info;
	uint32_t ratelist1[] = MXESAI_FRAME_RATE_LIST1;
	uint32_t ratelist2[] = MXESAI_FRAME_RATE_LIST2;
	ado_debug (DB_LVL_DRIVER, "CTRL_DLL_INIT: MXESAI");
	if ((mx = (mxesai_t *) ado_calloc (1, sizeof (mxesai_t))) == NULL)
	{
		ado_error_fmt("MX ESAI: Unable to allocate memory for mxesai (%s)",
			strerror (errno));
		return -1;
	}
	*hw_context = mx;
	mx->card = card;

	if (mxesai_parse_commandline(mx, args) != EOK)
	{
		ado_free (mx);
		return -1;
	}

	ado_mutex_init (&mx->hw_lock);

	if (!mx->esaibase)
	{
		ado_error_fmt("MX ESAI: Base address not specified");
		ctrl_destroy(mx);
		return -1;
	}

	ado_card_set_shortname (card, "MXESAI");
	ado_card_set_longname(card, "NXP i.MX ESAI", mx->esaibase);

	if (get_dmafuncs(&mx->dmafuncs, sizeof(dma_functions_t)) == -1)
	{
		ado_error_fmt("MX ESAI: Failed to get DMA lib functions");
		ctrl_destroy(mx);
		return -1;
	}
#ifdef IMX_EDMA
	if (mx->dmafuncs.init(NULL) == -1)
#else
	if (mx->dmafuncs.init("attach_process=1") == -1)
#endif
	{
		ado_error_fmt("MX ESAI: DMA init failed");
		ctrl_destroy(mx);
		return -1;
	}
	mx->dmafuncs.driver_info(&dma_info);
	/* Attach DMA channel for Tx */
	my_attach_pulse (&mx->aif[0].play_strm.pulse, &mx->aif[0].play_strm.dma_event, mxesai_play_pulse_hdlr, mx);
#ifdef IMX_EDMA
	strcpy(str, "tcd_reload=1");
	mx->aif[0].play_strm.dma_chn = mx->dmafuncs.channel_attach_smmu(str, &mx->aif[0].play_strm.dma_event, &mx->tchn, dma_info.max_priority,
	                                                                DMA_ATTACH_EVENT_PER_SEGMENT | DMA_ATTACH_PROCESS, ado_card_smmu(card));
#else
	/*
	 * DMA channel setup for Playback
	 * 1) watermark = must match the TX FIFO watermark in ESAI
	 * 2) eventnum = ESAI TX0 DMA event
	 * 3) fifopaddr = Physical address of ESAI TX0 FIFO
	 * 4) regen,contloop = DMA in repeat/loop mode so we only need to configure
	 *    the DMA transfer on channel acquire and not on every interrupt.
	 */
	physaddr = (mx->esaibase + offsetof(struct esai, etdr));
	build_dma_string (str, sizeof(str), physaddr, mx->tevt, FIFO_WATERMARK);

	ado_debug(DB_LVL_DRIVER, "MX ESAI: Playback dma priority = %d", dma_info.max_priority);
	mx->aif[0].play_strm.dma_chn =
		  mx->dmafuncs.channel_attach(str, &mx->aif[0].play_strm.dma_event, &mx->tchn,
									  dma_info.max_priority, DMA_ATTACH_EVENT_PER_SEGMENT);
#endif
	if (mx->aif[0].play_strm.dma_chn == NULL)
	{
		ado_error_fmt("MX ESAI: DMA playback channel attach failed");
		ctrl_destroy(mx);
		return -1;
	}

	/* Attach DMA channel for Rx */
	my_attach_pulse (&mx->aif[0].cap_strm.pulse, &mx->aif[0].cap_strm.dma_event, mxesai_cap_pulse_hdlr, mx);
#if IMX_EDMA
	strcpy(str, "tcd_reload=1");
	mx->aif[0].cap_strm.dma_chn = mx->dmafuncs.channel_attach_smmu(str, &mx->aif[0].cap_strm.dma_event, &mx->rchn, dma_info.max_priority,
	                                                               DMA_ATTACH_EVENT_PER_SEGMENT | DMA_ATTACH_PROCESS, ado_card_smmu(card));
#else
	/*
	 * DMA channel setup for Capture
	 * 1) watermark = must match the RX FIFO watermark in ESAI
	 * 2) eventnum = ESAI RX0 DMA event
	 * 3) fifopaddr = Physical address of ESAI RX0 FIFO
	 * 4) regen,contloop = DMA in repeat/loop mode so we only need to configure
	 *    the DMA transfer on channel acquire and not on every interrupt.
	 */
	physaddr = (mx->esaibase + offsetof(struct esai, erdr));
	build_dma_string (str, sizeof(str), physaddr, mx->revt, FIFO_WATERMARK);

	ado_debug(DB_LVL_DRIVER, "MX ESAI: Capture dma priority = %d", dma_info.max_priority);
	mx->aif[0].cap_strm.dma_chn =
		  mx->dmafuncs.channel_attach(str, &mx->aif[0].cap_strm.dma_event, &mx->rchn,
									  dma_info.max_priority, DMA_ATTACH_EVENT_PER_SEGMENT);
#endif
	if (mx->aif[0].cap_strm.dma_chn == NULL)
	{
		ado_error_fmt("MX ESAI: DMA capture channel attach failed");
		ctrl_destroy(mx);
		return -1;
	}

	/* MMAP peripheral registers
	 * Note: We map the registers after the DMA attachments for SMMU support
	 */
	mx->esai = ado_device_mmap (mx->esaibase, sizeof (esai_t));
	if (mx->esai == MAP_FAILED)
	{
		ado_error_fmt("MX ESAI: Unable to mmap ESAI (%s)", strerror (errno));
		ctrl_destroy(mx);
		return -1;
	}

	if (mxesai_init (mx) == -1)
	{
		ado_error_fmt("MX ESAI: Unable to initialize ESAI registers");
		ctrl_destroy(mx);
		return -1;
	}

	/* If multiple rx audio interfaces then allocate a ping-pong DMA buffer for capture.
	 * We will DMA into this buffer and copy the TDM slot data out of this buffer into the
	 * appropriate client buffers that are allocated in the aquire function.
	 */
	if(mx->num_rx_aif > 1)
	{
		dma_transfer_t tinfo;
		int frag_idx = 0;
		int dma_xfer_size = 0;
		ado_pcm_config_t config;
		uint32_t inputsplitter_ms = 0, inputsplitter_samples = 0;
		uint32_t swmixer_ms = 0, swmixer_samples = 0;
#if IMX_EDMA
		dma_addr_t addr;
#endif

		/* Align the ping and pong buffers to the audio frame size */
		int alignment = mx->sample_size * mx->nslots * mx->num_sdi_pins;
		mxesai_parse_inputsplitter_config(mx, &inputsplitter_ms, &inputsplitter_samples);
		mxesai_parse_swmixer_config(mx, &swmixer_ms, &swmixer_samples);
		if (inputsplitter_ms > 0)
		{
			dma_xfer_size = inputsplitter_ms * alignment * mx->sample_rate_max / 1000;
			ado_debug(DB_LVL_DRIVER, "rx: inputsplitter_ms %d, aligned frag_size %d, alignment %d, sample_rate_max %d", inputsplitter_ms, dma_xfer_size, alignment, mx->sample_rate_max);
		}
		else if (inputsplitter_samples > 0)
		{
			dma_xfer_size = inputsplitter_samples * alignment;
			ado_debug(DB_LVL_DRIVER, "rx: inputsplitter_samples %d, aligned frag_size %d, alignment %d", inputsplitter_samples, dma_xfer_size, alignment);
		}
		else if (swmixer_ms > 0)
		{
			dma_xfer_size = swmixer_ms * alignment * mx->sample_rate_max / 1000;
			ado_debug(DB_LVL_DRIVER, "rx: sw_mixer_ms %d, aligned frag_size %d, alignment %d, sample_rate_max %d", swmixer_ms, dma_xfer_size, alignment, mx->sample_rate_max);
		}
		else if (swmixer_samples > 0)
		{
			dma_xfer_size = swmixer_samples * alignment;
			ado_debug(DB_LVL_DRIVER, "rx: sw_mixer_samples %d, aligned frag_size %d, alignment %d", swmixer_samples, dma_xfer_size, alignment);
		}
		else
		{
			dma_xfer_size = mx->rx_frag_size / alignment * alignment;
			ado_debug(DB_LVL_DRIVER, "rx: command line frag_size %d aligned to %d, alignment %d", mx->rx_frag_size, dma_xfer_size, alignment);
		}
		mx->rx_frag_size = dma_xfer_size;
		/* Two fragments for a ping-pong buffer */
		config.mode.block.frags_total = 2;

		config.mode.block.frag_size = dma_xfer_size;
		config.format.voices = mx->nslots;
		config.format.format = mx->sample_size == 2 ? SND_PCM_SFMT_S16_LE : SND_PCM_SFMT_S32_LE;
		mx->rx_fifo_watermark = calculate_fifo_watermark(mx->num_sdi_pins, &config);
		/* Update dmabuf.size based on possible changes done by calculate_fifo_watermark() */
		mx->capture_dmabuf.size = config.mode.block.frag_size * config.mode.block.frags_total;

		mx->capture_dmabuf.phys_addr = 0; /* Ensure phys address is 0 so we map anonymous memory */
		mx->capture_dmabuf.flags = ADO_BUF_CACHE;
		if ((mx->capture_dmabuf.addr = ado_mmap_phys (mx->card, mx->capture_dmabuf.size,
		                                              mx->capture_dmabuf.flags, &mx->capture_dmabuf.phys_addr)) == MAP_FAILED)
		{
			ado_error_fmt("MX ESAI: Unable to allocate DMA buffer");
			ctrl_destroy(mx);
			return -1;
		}
		/* Setup the DMA transfer */
		memset(&tinfo, 0, sizeof(tinfo));

		/* Allocate descriptor List */
		tinfo.dst_addrs = ado_calloc( config.mode.block.frags_total, sizeof(dma_addr_t));
		if(NULL == tinfo.dst_addrs)
		{
			ado_error_fmt("MX ESAI: insufficient memory");
			ctrl_destroy(mx);
			return -1;
		}
#if IMX_EDMA
		tinfo.src_addrs = &addr;
		tinfo.src_fragments = 1;
		/* Initialize src_addrs to physical address of RX register or ASRC IN register */
		tinfo.src_addrs->paddr = mx->esaibase + IMX_ESAI_ERDR_OFFSET;
		tinfo.src_addrs->len = mx->sample_size;
		/* Register address cannot be incremented */
		tinfo.src_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;
		tinfo.mode_flags = DMA_MODE_FLAG_REPEAT;
		tinfo.xfer_bytes = mx->rx_fifo_watermark;
#else
		tinfo.xfer_bytes = mx->capture_dmabuf.size;
#endif
		for (frag_idx = 0; frag_idx < config.mode.block.frags_total; frag_idx++)
		{
			tinfo.dst_addrs[frag_idx].paddr = mx->capture_dmabuf.phys_addr + (frag_idx * config.mode.block.frag_size);
			tinfo.dst_addrs[frag_idx].len = config.mode.block.frag_size;
		}
		tinfo.dst_fragments = config.mode.block.frags_total;
		tinfo.xfer_unit_size = mx->sample_size * _BITS_BYTE;
		mx->dmafuncs.setup_xfer(mx->aif[0].cap_strm.dma_chn, &tinfo);
		ado_free(tinfo.dst_addrs);
	}

	/* If multiple tx audio interfaces then allocate a ping-pong DMA buffer for playback.
	 * We will DMA from this buffer to the hardware and copy the TDM slot data to the
	 * appropriate client buffers that are allocated in the aquire function.
	 */
	if(mx->num_tx_aif > 1)
	{
		dma_transfer_t tinfo;
		int frag_idx = 0;
		int dma_xfer_size = 0;
		ado_pcm_config_t config;
		uint32_t swmixer_ms = 0, swmixer_samples = 0;
#if IMX_EDMA
		dma_addr_t addr;
#endif
		/* Align the ping and pong buffers to the audio frame size */
		int alignment = mx->sample_size * mx->nslots * mx->num_sdo_pins;
		mxesai_parse_swmixer_config(mx, &swmixer_ms, &swmixer_samples);
		if (swmixer_ms > 0)
		{
			dma_xfer_size = swmixer_ms * alignment * mx->sample_rate_max / 1000;
			ado_debug(DB_LVL_DRIVER, "tx: sw_mixer_ms %d, aligned frag_size %d, alignment %d, sample_rate_max %d", swmixer_ms, dma_xfer_size, alignment, mx->sample_rate_max);
		}
		else if (swmixer_samples > 0)
		{
			dma_xfer_size = swmixer_samples * alignment;
			ado_debug(DB_LVL_DRIVER, "tx: sw_mixer_samples %d, aligned frag_size %d, alignment %d", swmixer_samples, dma_xfer_size, alignment);
		}
		else
		{
			dma_xfer_size = mx->tx_frag_size / alignment * alignment;
			ado_debug(DB_LVL_DRIVER, "tx: command line frag_size %d aligned to %d, alignment %d", mx->tx_frag_size, dma_xfer_size, alignment);
		}
		mx->tx_frag_size = dma_xfer_size;
		/* Two fragments for a ping-pong buffer */
		config.mode.block.frags_total = 2;

		config.mode.block.frag_size = dma_xfer_size;
		config.format.voices = mx->nslots;
		config.format.format = mx->sample_size == 2 ? SND_PCM_SFMT_S16_LE : SND_PCM_SFMT_S32_LE;
		mx->tx_fifo_watermark = calculate_fifo_watermark(mx->num_sdo_pins, &config);
		/* Update dmabuf.size based on possible changes done by calculate_fifo_watermark() */
		mx->playback_dmabuf.size = config.mode.block.frag_size * config.mode.block.frags_total;

		mx->playback_dmabuf.phys_addr = 0; /* Ensure phys address is 0 so we map anonymous memory */
		mx->playback_dmabuf.flags = ADO_BUF_CACHE;
		if ((mx->playback_dmabuf.addr = ado_mmap_phys (mx->card, mx->playback_dmabuf.size,
		                                               mx->playback_dmabuf.flags, &mx->playback_dmabuf.phys_addr)) == MAP_FAILED)

		{
			ado_error_fmt("MX ESAI: Unable to allocate DMA buffer");
			ctrl_destroy(mx);
			return -1;
		}
		/* Setup the DMA transfer */
		memset(&tinfo, 0, sizeof(tinfo));

		/* Allocate descriptor List */
		tinfo.src_addrs = ado_calloc( config.mode.block.frags_total, sizeof(dma_addr_t));
		if(NULL == tinfo.src_addrs)
		{
			ado_error_fmt("MX ESAI: insufficient memory");
			ctrl_destroy(mx);
			return -1;
		}
		for (frag_idx = 0; frag_idx < config.mode.block.frags_total; frag_idx++)
		{
			tinfo.src_addrs[frag_idx].paddr = mx->playback_dmabuf.phys_addr + (frag_idx * config.mode.block.frag_size);
			tinfo.src_addrs[frag_idx].len = config.mode.block.frag_size;
		}
#if IMX_EDMA
		/* eDMA driver specific initialization */
		/* Allocate dst_addr for TX reg */
		tinfo.dst_addrs = &addr;
		tinfo.dst_fragments = 1;
		/* Initialize dst_addr to physical address of TX register or ASRC IN register */
		tinfo.dst_addrs->paddr = mx->esaibase + IMX_ESAI_ETDR_OFFSET;
		tinfo.dst_addrs->len = mx->sample_size;
		/* Register address cannot be incremented */
		tinfo.dst_flags = DMA_ADDR_FLAG_NO_INCREMENT | DMA_ADDR_FLAG_DEVICE;
		/* Continuously repeat this DMA transfer */
		tinfo.mode_flags = DMA_MODE_FLAG_REPEAT;
		tinfo.xfer_bytes = mx->tx_fifo_watermark;
#else
		tinfo.xfer_bytes = mx->playback_dmabuf.size;
#endif
		tinfo.src_fragments = config.mode.block.frags_total;
		tinfo.xfer_unit_size = mx->sample_size * _BITS_BYTE;
		mx->dmafuncs.setup_xfer(mx->aif[0].play_strm.dma_chn, &tinfo);
		ado_free(tinfo.src_addrs);
	}

	// Define the various playback capabilities then copy to capture capabilities
	mx->aif[0].play_strm.pcm_caps.chn_flags =
		SND_PCM_CHNINFO_BLOCK | SND_PCM_CHNINFO_STREAM |
		SND_PCM_CHNINFO_INTERLEAVE | SND_PCM_CHNINFO_BLOCK_TRANSFER |
		SND_PCM_CHNINFO_MMAP | SND_PCM_CHNINFO_MMAP_VALID;

	mx->aif[0].play_strm.pcm_caps.formats = mx->sample_size == 2 ? SND_PCM_FMT_S16_LE : SND_PCM_FMT_S32_LE;

	if (mx->clk_mode == ESAI_CLK_MASTER)
	{
		uint32_t *ratelist;
		int cnt = 0;
		if ((mx->main_clk_freq % 48000) == 0)
		{
			ratelist = ratelist1;
			cnt = sizeof(ratelist1)/sizeof(ratelist1[0]);
		}
		else
		{
			ratelist = ratelist2;
			cnt = sizeof(ratelist2)/sizeof(ratelist2[0]);
		}

		for (i = 0; i < cnt; i++)
		{
			rate = ado_pcm_flag2rate(ratelist[i]);
			if (rate >= mx->sample_rate_min && rate <= mx->sample_rate_max)
			{
				mx->aif[0].play_strm.pcm_caps.rates |= ratelist[i];
			}
		}
	}
	else
	{
		mx->aif[0].play_strm.pcm_caps.rates = ado_pcm_rate2flag(mx->sample_rate_max);
	}

	mx->aif[0].play_strm.pcm_caps.min_rate = mx->sample_rate_min;
	mx->aif[0].play_strm.pcm_caps.max_rate = mx->sample_rate_max;
	mx->aif[0].play_strm.pcm_caps.min_voices = mx->aif[0].play_strm.voices;
	mx->aif[0].play_strm.pcm_caps.max_voices = mx->aif[0].play_strm.voices;
	mx->aif[0].play_strm.pcm_caps.min_fragsize = 64;
#ifdef IMX_ESAI_MAX_FRAGSIZE
	mx->aif[0].play_strm.pcm_caps.max_fragsize = IMX_ESAI_MAX_FRAGSIZE;
#else
	mx->aif[0].play_strm.pcm_caps.max_fragsize = 32 * 1024;
#endif
#ifdef IMX_EDMA
	/* Limit the number of fragments to the maximum allowed DMA descriptors */
	mx->aif[0].play_strm.pcm_caps.max_frags = MAX_DESCRIPTORS;
#endif
	if (mx->num_tx_aif > 1)
	{
		const int nvoices = mx->nslots * mx->num_sdo_pins;
		/*
		 * mx->tx_frag_size has been aligned to the audio frame size, mx->sample_size * mx->nslots * mx->num_sdo_pins
		 */
		mx->aif[0].play_strm.pcm_caps.min_fragsize = (mx->tx_frag_size / nvoices) * mx->aif[0].play_strm.voices;
		/* Align max_fragsize to the min_fragsize */
		mx->aif[0].play_strm.pcm_caps.max_fragsize =
				max( mx->aif[0].play_strm.pcm_caps.min_fragsize,
					 mx->aif[0].play_strm.pcm_caps.max_fragsize / mx->aif[0].play_strm.pcm_caps.min_fragsize
					 * mx->aif[0].play_strm.pcm_caps.min_fragsize);
		/* Ping-pong buffer used when the number of interfaces > 1 */
		mx->aif[0].play_strm.pcm_caps.max_frags = 2;
	}
	for (cnt = 1; cnt < mx->num_tx_aif; cnt++)
	{
		const int nvoices = mx->nslots * mx->num_sdo_pins;
		memcpy (&mx->aif[cnt].play_strm.pcm_caps, &mx->aif[0].play_strm.pcm_caps,
			sizeof (mx->aif[cnt].play_strm.pcm_caps));
#ifdef IMX_ESAI_MAX_FRAGSIZE
		mx->aif[cnt].play_strm.pcm_caps.max_fragsize = IMX_ESAI_MAX_FRAGSIZE;
#else
		mx->aif[cnt].play_strm.pcm_caps.max_fragsize = 32 * 1024;
#endif
		mx->aif[cnt].play_strm.pcm_caps.min_voices = mx->aif[cnt].play_strm.voices;
		mx->aif[cnt].play_strm.pcm_caps.max_voices = mx->aif[cnt].play_strm.voices;
		/*
		 * mx->tx_frag_size has been aligned to the audio frame size, mx->sample_size * mx->nslots * mx->num_sdo_pins
		 */
		mx->aif[cnt].play_strm.pcm_caps.min_fragsize = (mx->tx_frag_size / nvoices) * mx->aif[cnt].play_strm.voices;
		mx->aif[cnt].play_strm.pcm_caps.max_fragsize =
				max( mx->aif[cnt].play_strm.pcm_caps.min_fragsize,
					 mx->aif[cnt].play_strm.pcm_caps.max_fragsize / mx->aif[cnt].play_strm.pcm_caps.min_fragsize
					 * mx->aif[cnt].play_strm.pcm_caps.min_fragsize);
	}

	/*
	 * Initialize the all of the cap stream capabilities with the playback stream capabilities
	 * including aif[0].cap_strm. Then update the few members that have command line overrides.
	 */
	for (cnt = 0; cnt < mx->num_rx_aif; cnt++)
	{
		memcpy(&mx->aif[cnt].cap_strm.pcm_caps, &mx->aif[0].play_strm.pcm_caps,
			sizeof (mx->aif[cnt].cap_strm.pcm_caps));
		mx->aif[cnt].cap_strm.pcm_caps.min_fragsize = 64;
#ifdef IMX_ESAI_MAX_FRAGSIZE
		mx->aif[cnt].cap_strm.pcm_caps.max_fragsize = IMX_ESAI_MAX_FRAGSIZE;
#else
		mx->aif[cnt].cap_strm.pcm_caps.max_fragsize = 32 * 1024;
#endif
		mx->aif[cnt].cap_strm.pcm_caps.min_voices = mx->aif[cnt].cap_strm.voices;
		mx->aif[cnt].cap_strm.pcm_caps.max_voices = mx->aif[cnt].cap_strm.voices;
		if (mx->num_rx_aif > 1)
		{
			const int nvoices = mx->nslots * mx->num_sdi_pins;
			/*
			 * mx->rx_frag_size has been aligned to the audio frame size, mx->sample_size * mx->nslots * mx->num_sdi_pins
			 */
			mx->aif[cnt].cap_strm.pcm_caps.min_fragsize = (mx->rx_frag_size / nvoices) * mx->aif[cnt].cap_strm.voices;
			/* Align max_fragsize to the min_fragsize */
			mx->aif[cnt].cap_strm.pcm_caps.max_fragsize =
					max( mx->aif[cnt].cap_strm.pcm_caps.min_fragsize,
						 mx->aif[cnt].cap_strm.pcm_caps.max_fragsize / mx->aif[cnt].cap_strm.pcm_caps.min_fragsize
						 * mx->aif[cnt].cap_strm.pcm_caps.min_fragsize);
			/* Ping-pong buffer used when the number of interfaces > 1 */
			mx->aif[cnt].cap_strm.pcm_caps.max_frags = 2;
		}
	}

	mx->aif[0].play_strm.pcm_funcs.capabilities = mxesai_capabilities;
	mx->aif[0].play_strm.pcm_funcs.aquire = mxesai_playback_aquire;
	mx->aif[0].play_strm.pcm_funcs.release = mxesai_playback_release;
	mx->aif[0].play_strm.pcm_funcs.prepare = mxesai_prepare;
	mx->aif[0].play_strm.pcm_funcs.trigger = mxesai_playback_trigger;
	/* If multiple output pins are used and there is only 1 tx_aif we need to use the reconstitute
	 * callout to shuffle channel order.
	 * Note: If more then 1 tx_aif is used then the mxesai_dmaplayback_combine()
	 * will call the reconstitute function directely.
	 */
	if (mx->num_sdo_pins > 1 && mx->num_tx_aif == 1)
		mx->aif[0].play_strm.pcm_funcs.reconstitute = mxesai_playback_reconstitute;
	if (mx->num_sdo_pins > 1)
	{
		/* Allocate a playback reconstitue buffer (16bit samples) */
		if ((mx->sdo_reconstitute_buffer = ado_calloc((mx->num_sdo_pins * mx->nslots), mx->sample_size)) == NULL)
		{
			ado_error_fmt("MX ESAI: Unable to allocate playback reconstitute buffer (%s)", strerror (errno));
			ctrl_destroy(mx);
			return -1;
		}
	}

#if 0
	mx->aif[0].play_strm.pcm_funcs.position = mxesai_position;
#endif

	mx->aif[0].cap_strm.pcm_funcs.capabilities = mxesai_capabilities;
	mx->aif[0].cap_strm.pcm_funcs.aquire = mxesai_capture_aquire;
	mx->aif[0].cap_strm.pcm_funcs.release = mxesai_capture_release;
	mx->aif[0].cap_strm.pcm_funcs.prepare = mxesai_prepare;
	mx->aif[0].cap_strm.pcm_funcs.trigger = mxesai_capture_trigger;
	if (mx->num_sdi_pins > 1 && mx->num_rx_aif == 1)
		mx->aif[0].cap_strm.pcm_funcs.reconstitute = mxesai_capture_reconstitute;
	if (mx->num_sdi_pins > 1)
	{
		/* Allocate a capture reconstitue buffer (16bit samples) */
		if ((mx->sdi_reconstitute_buffer = ado_calloc((mx->num_sdi_pins * mx->nslots), mx->sample_size)) == NULL)
		{
			ado_error_fmt("MX ESAI: Unable to allocate capture reconstitute buffer (%s)", strerror (errno));
			ctrl_destroy(mx);
			return -1;
		}
	}

#if 0
	mx->aif[0].cap_strm.pcm_funcs.position = mxesai_position;
#endif

	/* Fill in the channel maps before creating the codec mixer */
	for (cnt = 0; cnt < max(mx->num_tx_aif, mx->num_rx_aif); cnt++)
	{
		if (cnt < mx->num_tx_aif && mx->aif[cnt].play_strm.chmap) {
			mx->aif[cnt].play_strm.pcm_caps.chmap = mx->aif[cnt].play_strm.chmap;
		} else {
			mx->aif[cnt].play_strm.pcm_caps.chmap = ado_pcm_get_default_chmap ( mx->aif[cnt].play_strm.voices );
		}

		if (cnt < mx->num_rx_aif && mx->aif[cnt].cap_strm.chmap) {
			mx->aif[cnt].cap_strm.pcm_caps.chmap = mx->aif[cnt].cap_strm.chmap;
		} else {
			mx->aif[cnt].cap_strm.pcm_caps.chmap = ado_pcm_get_default_chmap ( mx->aif[cnt].cap_strm.voices );
		}
	}
	/* Create the mixer */
	if (codec_mixer (card, mx, NULL))
	{
		ado_error_fmt("MX ESAI: Unable to create codec mixer");
		ctrl_destroy(mx);
		return -1;
	}
#ifdef CODEC_PLAYBACK_MUTE
	/* Initialize codec to be muted, will be unmuted from the trigger callback */
	CODEC_PLAYBACK_MUTE;
#endif

	for (cnt = 0; cnt < max(mx->num_tx_aif, mx->num_rx_aif); cnt++)
	{
		char pcm_name[_POSIX_NAME_MAX];
		char pcm_name2[_POSIX_NAME_MAX];
		snprintf(pcm_name, _POSIX_NAME_MAX, "mxesai PCM %d", cnt);
		snprintf(pcm_name2, _POSIX_NAME_MAX, "mxesai-%d", cnt);

		if (ado_pcm_create (card, pcm_name, 0, pcm_name2,
				cnt < mx->num_tx_aif ? 1: 0,
				cnt < mx->num_tx_aif ? &mx->aif[cnt].play_strm.pcm_caps : NULL,
				cnt < mx->num_tx_aif ? &mx->aif[0].play_strm.pcm_funcs : NULL ,
				cnt < mx->num_rx_aif ? 1: 0,
				cnt < mx->num_rx_aif ? &mx->aif[cnt].cap_strm.pcm_caps : NULL,
				cnt < mx->num_rx_aif ? &mx->aif[0].cap_strm.pcm_funcs : NULL,
				mx->mixer,
				&mx->aif[cnt].pcm))
		{
			ado_error_fmt("MX ESAI: Unable to create pcm devices (%s)", strerror (errno));
			ctrl_destroy(mx);
			return -1;
		}
		ado_debug(DB_LVL_DRIVER, "MX ESAI: PCM%d -> TX voices = %d, RX voices = %d, max_frag = %d", cnt,
				  mx->aif[cnt].play_strm.voices, mx->aif[cnt].cap_strm.voices,  mx->aif[cnt].play_strm.pcm_caps.max_fragsize);
		if (cnt < mx->num_tx_aif) {
			codec_set_default_group(mx, mx->aif[cnt].pcm, ADO_PCM_CHANNEL_PLAYBACK, cnt);
		}
		if (cnt < mx->num_rx_aif) {
			codec_set_default_group(mx, mx->aif[cnt].pcm, ADO_PCM_CHANNEL_CAPTURE, cnt);
		}
	}
	return 0;
}

ado_ctrl_dll_destroy_t ctrl_destroy;
int
ctrl_destroy (HW_CONTEXT_T * mx)
{
	uint32_t cnt = 0;

	ado_debug (DB_LVL_DRIVER, "CTRL_DLL_DESTROY: MXESAI");

	if (mx->playback_dmabuf.addr != NULL)
	{
		/* Free global playback transfer buffer */
		ado_munmap_phys (mx->card, mx->playback_dmabuf.addr, mx->playback_dmabuf.size);
	}
	if (mx->capture_dmabuf.addr != NULL)
	{
		/* Free global capture transfer buffer */
		ado_munmap_phys (mx->card, mx->capture_dmabuf.addr, mx->capture_dmabuf.size);
	}
	if (mx->aif[0].cap_strm.dma_chn != NULL)
	{
		my_detach_pulse(&mx->aif[0].cap_strm.pulse);
		mx->dmafuncs.channel_release(mx->aif[0].cap_strm.dma_chn);
	}
	if (mx->aif[0].play_strm.dma_chn)
	{
		my_detach_pulse(&mx->aif[0].play_strm.pulse);
		mx->dmafuncs.channel_release(mx->aif[0].play_strm.dma_chn);
	}

	mx->dmafuncs.fini();

	ado_mutex_destroy (&mx->hw_lock);

	if (mx->esai)
	{
		/* Disable ESAI */
		mx->esai->ecr = 0x0;
		ado_device_munmap(mx->esai, sizeof (esai_t));
	}

	if (mx->sdo_reconstitute_buffer != NULL)
		ado_free(mx->sdo_reconstitute_buffer);
	if (mx->sdi_reconstitute_buffer != NULL)
		ado_free(mx->sdi_reconstitute_buffer);

	for (cnt = 0; cnt < mx->num_tx_aif; cnt++)
	{
		if (mx->aif[cnt].play_strm.chmap)
			ado_free(mx->aif[cnt].play_strm.chmap);
	}
	for (cnt = 0; cnt < mx->num_rx_aif; cnt++)
	{
		if (mx->aif[cnt].cap_strm.chmap)
			ado_free(mx->aif[cnt].cap_strm.chmap);
	}

	ado_free(mx->aif);
	ado_free (mx);

	return 0;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/deva/ctrl/mxesai/mxesai_dll.c $ $Rev: 903313 $")
#endif
