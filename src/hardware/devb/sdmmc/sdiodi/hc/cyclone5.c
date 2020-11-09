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

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <hw/inout.h>
#include <sys/mman.h>
#include <internal.h>
#include <inttypes.h>
#include <stdio.h>

#ifdef SDMMC_VENDOR_ALT_SOCFPGA

#define DEBUG 0

#include <cyclone5.h>
#include <sys/syspage.h>
#include <hw/sysinfo.h>

static int socfpga_data_completed(sdio_hc_t *hc);
static int socfpga_pio_xfer(sdio_hc_t *hc, sdio_cmd_t *cmd);
static int socfpga_xfer_setup( sdio_hc_t *hc, sdio_cmd_t *cmd);
static int socfpga_powerdown(sdio_hc_t *hc);
void dump_registers(sdio_hc_t *hc);
static int socfpga_cmd(sdio_hc_t *hc, sdio_cmd_t *cmd);
static int socfpga_dma_continue(sdio_hc_t *hc, sdio_cmd_t *cmd );


/*****************************************************************************************
 * Function:  socfpga_alloc_desc_ring
 * Description: Allocate DMA descriptor ring
 *
 * sdio_hc_t *hc,
 *
 * return:  EOK or
 *          MMC_FAILURE
 * */
static inline int
socfpga_alloc_desc_ring(sdio_hc_t *hc)
{
	socfpga_ext_t   *socfpga;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	socfpga = (socfpga_ext_t *)hc->cs_hdl;

	// alloc descr ring
	if ( (socfpga->dma_buf_desc = sdio_alloc(sizeof(mmc_dma_desc_t) * socfpga->dma_num_descs) ) == NULL)
	{
		sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,"%s(): mmap failed: %d", __func__, ENOMEM);
		return ENOMEM;
	}

	socfpga->dma_buf_desc_phys = sdio_vtop((void *) socfpga->dma_buf_desc);

	return EOK;
}


/*****************************************************************************************
 * Function:  socfpga_init_desc_ring
 * Description:
 *
 * sdio_hc_t *hc

 * return:  EOK or
 *          MMC_FAILURE
 * */

static inline int
socfpga_init_desc_ring(sdio_hc_t *hc)
{
	int             i;
	mmc_dma_desc_t *bd;
	socfpga_ext_t  *socfpga;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	socfpga = (socfpga_ext_t *)hc->cs_hdl;

	// init descr ring
	for (i = 0; i < socfpga->dma_num_descs; i++) {
		bd = socfpga->dma_buf_desc + i;
		memset((void *)bd, 0, sizeof(*bd));
		bd->des0.own_bit = 0;
		bd->des0.second_address_chained = 1;
		bd->des1.buff1_size = 0;
		bd->des2 = 0;
		/* Chained mode usage */
		if (i == (socfpga->dma_num_descs - 1)) {
			/* The last element has to point to the head. */
			bd->des3 = (uint32_t) socfpga->dma_buf_desc_phys;
		} else {
			/* Point to the next element in the ring */
			bd->des3 = (uint32_t) sdio_vtop((void *) &socfpga->dma_buf_desc[i + 1]);
		}
	}

	return EOK;
}

/*****************************************************************************************
 * Function:  socfpga_reset_desc_ring
 * Description: Reset description ring
 *
 * sdio_hc_t *hc,

 * return: EOK
 *
 * */

static inline int
socfpga_clr_desc_ring(sdio_hc_t *hc)
{
	int             i;
	mmc_dma_desc_t *bd;
	socfpga_ext_t  *socfpga;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	socfpga = (socfpga_ext_t *)hc->cs_hdl;

	// initialize descr ring
	for (i = 0; i < socfpga->dma_num_descs; i++) {
		bd = socfpga->dma_buf_desc + i;
		bd->des0.own_bit = 0;
		bd->des0.dis_int_on_completion = 0;
		bd->des0.last_segment = 0;
		bd->des0.first_segment = 0;
		bd->des1.buff1_size = 0;
		bd->des2 = 0;
	}

	return EOK;
}
/*****************************************************************************************
 * Function:  socfpga_intr_event
 * Description: Interrupt handler routine
 *
 * sdio_hc_t *hc
 *
 * return:
 * */

static int socfpga_intr_event(sdio_hc_t *hc)
{

	socfpga_ext_t  *socfpga;
	uint32_t        sts, idsts;
	uint32_t        err, iderr;
	int             intr = 0;
	uintptr_t       base;
	sdio_cmd_t     *cmd = hc->wspc.cmd;
	int             cs = CS_CMD_INPROG;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;


	sts   = in32(base + ALT_SDMMC_MINTSTS_OFST);        // read Masked Interrupt Status Register
	idsts = in32(base + ALT_SDMMC_IDSTS_OFST);          // read Masked DMA Interrupt Status Register

	err   = ALT_SDMMC_MINTSTS_RESP_SET_MSK        |
	        ALT_SDMMC_MINTSTS_RESPCRCERR_SET_MSK  |
	        ALT_SDMMC_MINTSTS_DATACRCERR_SET_MSK  |
	        ALT_SDMMC_MINTSTS_RESPTO_SET_MSK      |
	        ALT_SDMMC_MINTSTS_FIFOOVUNERR_SET_MSK |
	        ALT_SDMMC_MINTSTS_STRERR_SET_MSK      |
	        ALT_SDMMC_MINTSTS_EBE_SET_MSK         |
	        ALT_SDMMC_MINTSTS_DSHTO_SET_MSK;

	iderr = ALT_SDMMC_IDSTS_FBE_SET_MSK |
	        ALT_SDMMC_IDSTS_DU_SET_MSK  |
	        ALT_SDMMC_IDSTS_CES_SET_MSK |
	        ALT_SDMMC_IDSTS_AIS_SET_MSK |
	        ALT_SDMMC_IDSTS_EB_SET_MSK;


	// clear interrupts
	out32(base + ALT_SDMMC_IDSTS_OFST, idsts);
	out32(base + ALT_SDMMC_RINTSTS_OFST, sts);

	if( ( cmd = hc->wspc.cmd ) == NULL ) {
		return( EOK );
	}

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "%s: sts 0x%x, idsts 0x%x, cmd->flags 0x%x", __FUNCTION__, sts, idsts, cmd->flags);
#if DEBUG	// this made it work, with verbose 0 changes
	printf("%s: sts %x, idsts %x, cmd->opcode %d, cmd->flags 0x%x\n", __FUNCTION__, sts, idsts, cmd->opcode, cmd->flags);
#endif

	if ((sts & err) || (idsts & iderr)) {
		// Any errors ?
		sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s: sts interrupt error %x, idsts 0x%x", __FUNCTION__, sts, idsts);

		if (sts & ALT_SDMMC_MINTSTS_RESPTO_SET_MSK)
			cs = CS_DATA_TO_ERR;
		if (sts & ALT_SDMMC_MINTSTS_EBE_SET_MSK)
			cs = CS_DATA_END_ERR;
		if (sts & ALT_SDMMC_MINTSTS_RESP_SET_MSK)
			cs = CS_CMD_CMP_ERR;
		if (sts & ALT_SDMMC_MINTSTS_RESPCRCERR_SET_MSK)
			cs = CS_CMD_CRC_ERR;
		if (sts & ALT_SDMMC_MINTSTS_DATACRCERR_SET_MSK)
			cs = CS_DATA_CRC_ERR;
		if (sts & ALT_SDMMC_MINTSTS_RESPTO_SET_MSK)
			cs = CS_CMD_TO_ERR;
		if (sts & ALT_SDMMC_MINTSTS_FIFOOVUNERR_SET_MSK)
			cs = CS_DATA_TO_ERR;
		if (sts & ALT_SDMMC_MINTSTS_STRERR_SET_MSK)
			cs = CS_DATA_TO_ERR;

		if (cs == CS_CMD_INPROG)
			cs = CS_CMD_CMP_ERR;
	}

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3,"%s: mintsts 0x%x", __FUNCTION__, sts);

	if (sts & ALT_SDMMC_MINTSTS_HLWERR_SET_MSK)
	{
		// Hardware locked write error (HLE)
		// attempt to write to hardware registers while start_cmd is 1
		intr = MMC_INTR_NONE;
		cs   = CS_CMD_CMP_ERR;

#if DEBUG
		printf("     Socfpga interrupt error ALT_SDMMC_MINTSTS_HLWERR_SET_MSK\n");
#endif

		sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,"Socfpga interrupt error ALT_SDMMC_MINTSTS_HLWERR_SET_MSK");

	}

	if (sts & ALT_SDMMC_MINTSTS_CD_SET_MSK)
	{
		// Card insertion detected
		//out32(base + ALT_SDMMC_RINTSTS_OFST, ALT_SDMMC_MINTSTS_CD_SET_MSK);
		intr |= MMC_INTR_CARDINS;
		/* Card insertion and card removal events */
		sdio_hc_event( hc, HC_EV_CD );

#if DEBUG
		printf("     Socfpga interrupt  ALT_SDMMC_MINTSTS_CD_SET_MSK\n");
#endif

		sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,"Socfpga interrupt  ALT_SDMMC_MINTSTS_CD_SET_MSK");
	}

	if (sts & ALT_SDMMC_MINTSTS_CMD_DONE_SET_MSK)
	{
		// Command Done
		intr |= MMC_INTR_COMMAND;
		if (!(cmd->flags & SCF_CTYPE_ADTC))
			cs = CS_CMD_CMP;

		if (cmd)
		{
			if (cmd->flags & SCF_RSP_136)    //cmd->flags for resp_type
			{
				cmd->rsp[0] = in32(base + ALT_SDMMC_RESP3_OFST);
				cmd->rsp[1] = in32(base + ALT_SDMMC_RESP2_OFST);
				cmd->rsp[2] = in32(base + ALT_SDMMC_RESP1_OFST);
				cmd->rsp[3] = in32(base + ALT_SDMMC_RESP0_OFST);
			}
			else
			if (cmd->flags & SCF_RSP_PRESENT)
			{
				cmd->rsp[0] = in32(base + ALT_SDMMC_RESP0_OFST);
			}
		}

			// Busy check?
		if (cmd && (cmd->flags & SCF_RSP_BUSY))
		{
			int	 i=500000000;
			while (--i > 0)
			{
				if (!ALT_SDMMC_STAT_DATA_BUSY_GET(in32( base + ALT_SDMMC_STAT_OFST ) ))
					break;
			}
#if 0
			if (i == 0)
			{
			    sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "SOCFPGA busy check time out, status = %x", in32(base + ALT_SDMMC_STAT_OFST));
#if DEBUG
			    printf("SOCFPGA busy check time out, status = %x", in32(base + ALT_SDMMC_STAT_OFST));
#endif
			}
#endif
		}
	}

	if (sts & ALT_SDMMC_MINTSTS_ACD_SET_MSK)      // Auto command done
	{
		cs = CS_CMD_CMP;
	}

	if(hc->caps & HC_CAP_DMA)
	{
		if (idsts & ALT_SDMMC_IDSTS_RI_SET_MSK)
		{
			cs = CS_CMD_CMP;
		}

		if (idsts & ALT_SDMMC_IDSTS_TI_SET_MSK)
		{
			cs = CS_CMD_CMP;
		}
	}
	else	//PIO
	{
		if (sts & ALT_SDMMC_MINTSTS_DT_SET_MSK)
		{
			intr |= MMC_INTR_DATA;                // Data complete
			cs = CS_CMD_CMP;
		}

		if (sts & ALT_SDMMC_MINTSTS_RXFIFODR_SET_MSK)
		{
			intr |= MMC_INTR_RBRDY;							// Receive FIFO Data Request
		}

		if (sts & ALT_SDMMC_MINTSTS_DTTXFIFODR_SET_MSK)
		{
			intr |= MMC_INTR_WBRDY;							// Transmit FIFO Data Request
		}
		// if data is ready then call socfpga_pio_xfer()
		if (intr  & ( MMC_INTR_DATA | MMC_INTR_RBRDY | MMC_INTR_WBRDY)) {
			socfpga_pio_xfer(hc, cmd);
			cs = CS_CMD_CMP;
		}
	}
	if( cs != CS_CMD_INPROG ) {
		sdio_cmd_cmplt( hc, cmd, cs );
	}

	return( EOK );

}
/*****************************************************************************************/
static int socfpga_event( sdio_hc_t *hc, sdio_event_t *ev )
{
	int     status;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	switch( ev->code ) {
		case HC_EV_INTR:
			status = socfpga_intr_event( hc );
			InterruptUnmask( hc->cfg.irq[0], hc->hc_iid );
			break;

		default:

			break;
	}

	return( status );
}

/*****************************************************************************************
 * Function: socfpga_pio_setup
 * Description: Setup controller for pio operations.
 *              Prepare socfpga->dcmd, socfpga->dmask and len values according to pending operation
 *
 * sdio_hc_t *hc,
 *
 * return: EOK or error
 *
 * */

static int socfpga_pio_setup(sdio_hc_t *hc, sdio_cmd_t *cmd)
{
	socfpga_ext_t  *socfpga;
	uintptr_t       base;
	int             timeout = 20000000;

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;
	int len = 0;
	int dir = cmd->flags;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	len = cmd->blksz * cmd->blks;

	// reset FIFO
	out32(base + ALT_SDMMC_CTL_OFST, ALT_SDMMC_CTL_FIFO_RST_SET_MSK | ALT_SDMMC_CTL_DMA_RST_SET_MSK);

	/* hw clears the bit when it's done */
	while (in32(base + ALT_SDMMC_CTL_OFST) & (ALT_SDMMC_CTL_CTLLER_RST_SET_MSK | ALT_SDMMC_CTL_FIFO_RST_SET_MSK	| ALT_SDMMC_CTL_DMA_RST_SET_MSK))
	{
		if (timeout == 0)
		{
#if DEBUG
			printf("Reset host timeout error\n");
#endif
			sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d: Reset host timeout error", __FUNCTION__, __LINE__);
			return (ETIMEDOUT);
		}
		timeout--;
	}

	socfpga->dcmd = ALT_SDMMC_CMD_DATA_EXPECTED_SET_MSK;     // Data transfer expected (read/write)

	out32(base + ALT_SDMMC_CARDTHRCTL_OFST, ALT_SDMMC_CARDTHRCTL_CARDRDTHREN_SET_MSK | ALT_SDMMC_CARDTHRCTL_CARDRDTHRESHOLD_SET(0x80));

	out32(base + ALT_SDMMC_BLKSIZ_OFST, ALT_SDMMC_BLKSIZ_BLOCK_SIZE_SET(cmd->blksz));
	out32(base + ALT_SDMMC_BYTCNT_OFST, ALT_SDMMC_BYTCNT_BYTE_COUNT_SET(len));

	if(len > 0 && cmd->blks > 1)
	{
		socfpga->dcmd |= ALT_SDMMC_CMD_SEND_AUTO_STOP_SET_MSK;          // Send stop command at end of data transfer
	}

	if (dir & SCF_DIR_OUT)
	{
		socfpga->dcmd  |= ALT_SDMMC_CMD_RD_WR_SET_MSK;
		socfpga->dmask |= ALT_SDMMC_INTMSK_HLE_SET_MSK;                 // Hardware locked write error (HLE)
		if ( !( hc->caps & HC_CAP_DMA ))
		{
			socfpga->dmask |= ALT_SDMMC_INTMSK_TXDR_SET_MSK |           // Transmit FIFO data request (TXDR)
			                  ALT_SDMMC_INTMSK_DTO_SET_MSK;             // Data transfer over

		}
	}
	else
	{
		if ( !( hc->caps & HC_CAP_DMA ))
		{
			socfpga->dmask |= ALT_SDMMC_INTMSK_RXDR_SET_MSK |           // Receive FIFO data request (RXDR)
			                  ALT_SDMMC_INTMSK_SBE_SET_MSK  |           // Start-bit error (SBE)
			                  ALT_SDMMC_INTMSK_DTO_SET_MSK;             // Data transfer over
		}
	}

	// disable cmd_done interrupt
	socfpga->dmask &= ALT_SDMMC_MINTSTS_CMD_DONE_CLR_MSK;

	return (EOK);
}


/*****************************************************************************************
 * Function: socfpga_pio_xfer
 * Description: Read/ Write data from/to FIFO
 *
 * sdio_hc_t *hc
 *
 * return: EOK
 * */
static int socfpga_pio_xfer(sdio_hc_t *hc, sdio_cmd_t *cmd)
{
	socfpga_ext_t  *socfpga;
	uintptr_t       base;
	uint32_t       *pbuf;
	uint32_t        items_in_fifo,dw_cnt;
	int             len;
	uint8_t        *addr;

	socfpga    = (socfpga_ext_t *)hc->cs_hdl;
	base       = socfpga->mmc_base;
	int blksz  = cmd->blksz;
	int blks   = cmd->blks;
	int xfer_len = 0;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "%s:%d", __FUNCTION__, __LINE__);

	while(blks--)
	{
		blksz = cmd->blksz;
		while( blksz )
		{
			if( sdio_sg_nxt( hc, &addr, &len, blksz ) )
			{
				sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "%s:%d: transfer complete", __FUNCTION__, __LINE__);
#if DEBUG
				printf("%s:%d: transfer complete", __FUNCTION__, __LINE__);
#endif
				break;
			}
			else
			{
				pbuf =  (uint32_t *)addr;
			}
			blksz	-= len;
			len		/= 4;

			while( len )
			{
				if (cmd->flags & SCF_DIR_IN)
				{
					items_in_fifo = ALT_SDMMC_STAT_FIFO_COUNT_GET(in32(base + ALT_SDMMC_STAT_OFST));
					if (items_in_fifo == 0)
					{
						// no data in FIFO yet
						sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3,"   No data in FIFO to read  : STATUS %08X\n",in32(base + ALT_SDMMC_STAT_OFST));
					}

					xfer_len = min( len, items_in_fifo );
					len -= xfer_len;
#if DEBUG
					printf("	Items_in_fifo %d, Reading %d\n", items_in_fifo * 4, xfer_len * 4);
#endif

					dw_cnt = xfer_len;					// number of "full" FIFO items to read

					// reading "full" dwords
					while (dw_cnt > 0)
					{
						*pbuf++ = in32(base + ALT_SDMMC_DATA_OFST);
						dw_cnt--;
					}
				}
				else // SCF_DIR_OUT
				{
					// get amount of free section in FIFO
					items_in_fifo = (MMCHS_FIFO_SIZE >> 2) - ALT_SDMMC_STAT_FIFO_COUNT_GET(in32(base + ALT_SDMMC_STAT_OFST));
					if (items_in_fifo == 0)
					{
						sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3,"	Available items_in_fifo %d\n", items_in_fifo);
#if DEBUG
						printf("	Available items_in_fifo %d\n", items_in_fifo);
#endif
					}

					xfer_len = min( len, items_in_fifo );
					len -= xfer_len;
					if (items_in_fifo != 0)
					{
#if DEBUG
						printf("	Available items_in_fifo %d, tx %d bytes\n", items_in_fifo * 4, xfer_len * 4);
#endif
						dw_cnt = xfer_len;					// number of "full" FIFO items to write
						// writing data
						for (; dw_cnt > 0; dw_cnt--)
							out32(base + ALT_SDMMC_DATA_OFST, *pbuf++);
					}
				}
				pbuf += xfer_len;
	        } // while (len)
	    }     // while (blksz)
	}         // while (blks)
	return (EOK);
}
/*****************************************************************************************
 * Function: socfpga_dma_setup
 * Description: Initialize DMA transfer operation
 *
 * sdio_hc_t *hc
 * sdio_cmd_t *cmd
 *
 *
 * return: EOK or error
 * */

static int socfpga_dma_setup(sdio_hc_t *hc, sdio_cmd_t *cmd)
{

	socfpga_ext_t  *socfpga;
	uintptr_t       base;
	mmc_dma_desc_t *bd;
	uint32_t        timeout = 20000000;
	uint32_t        dummy;
	uint32_t        dw_cnt;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;
	int dir = cmd->flags;
	int len =  cmd->blks ? cmd->blks * cmd->blksz : cmd->blksz;

	while ( (dw_cnt = ALT_SDMMC_STAT_FIFO_COUNT_GET(in32(base + ALT_SDMMC_STAT_OFST))) != 0)
	{
		// empty fifo
		while (dw_cnt > 0)
		{
			dummy = in32(base + ALT_SDMMC_DATA_OFST);
		    dw_cnt--;
		}
	}

	socfpga_clr_desc_ring(hc);

	//
	// Initialize DMA channel
	//
	out32(base + ALT_SDMMC_BMOD_OFST, in32(base + ALT_SDMMC_BMOD_OFST) | ALT_SDMMC_BMOD_SWR_SET_MSK);

	while (in32(base + ALT_SDMMC_CTL_OFST) & ALT_SDMMC_BMOD_SWR_SET_MSK)
	{
		if (timeout == 0)
		{
#if DEBUG
			printf("Reset host timeout error\n");
#endif
			sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d: Reset host timeout error", __FUNCTION__, __LINE__);
			return (-1);
		}
		timeout--;
	}

	if (len > 0) {
		bd = socfpga->dma_buf_desc;
		bd->des0.first_segment = 1;
		socfpga->dma_cur_pos = 0;

		out32(base + ALT_SDMMC_DBADDR_OFST, socfpga->dma_buf_desc_phys);

		//Set watermark
		if (dir & SCF_DIR_IN)
		{
			out32(base + ALT_SDMMC_FIFOTH_OFST,	ALT_SDMMC_FIFOTH_RX_WMARK_SET(MMCHS_DMA_SEGMENT_SIZE));
			// read it back
			dummy = in32(base + ALT_SDMMC_FIFOTH_OFST);
			if (!(dummy & ALT_SDMMC_FIFOTH_RX_WMARK_SET(MMCHS_DMA_SEGMENT_SIZE)))
				sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d: Error ALT_SDMMC_FIFOTH_OFST not set", __FUNCTION__, __LINE__);

		}
		else
		{
			out32(base + ALT_SDMMC_FIFOTH_OFST,	ALT_SDMMC_FIFOTH_TX_WMARK_SET(MMCHS_DMA_SEGMENT_SIZE)
					   | ALT_SDMMC_FIFOTH_DW_DMA_MULT_TRANSACTION_SIZE_SET(ALT_SDMMC_FIFOTH_DW_DMA_MULT_TRANSACTION_SIZE_E_RXMSIZEK1));
		}
		out32(base + ALT_SDMMC_BMOD_OFST, (in32(base + ALT_SDMMC_BMOD_OFST) & ALT_SDMMC_BMOD_PBL_CLR_MSK)
				   | ALT_SDMMC_BMOD_PBL_SET(/*ALT_SDMMC_BMOD_PBL_E_TRANS64*/ALT_SDMMC_BMOD_PBL_E_TRANS16));

		out32(base + ALT_SDMMC_IDINTEN_OFST, ALT_SDMMC_IDINTEN_NI_SET_MSK | ALT_SDMMC_IDINTEN_TI_SET_MSK | ALT_SDMMC_IDINTEN_RI_SET_MSK
				   | ALT_SDMMC_IDINTEN_FBE_SET_MSK | ALT_SDMMC_IDINTEN_DU_SET_MSK  | ALT_SDMMC_IDINTEN_CES_SET_MSK  | ALT_SDMMC_IDINTEN_AI_SET_MSK);
		// read it back
		dummy = in32(base + ALT_SDMMC_IDINTEN_OFST);
		if (!(dummy & (ALT_SDMMC_IDINTEN_NI_SET_MSK | ALT_SDMMC_IDINTEN_TI_SET_MSK | ALT_SDMMC_IDINTEN_RI_SET_MSK
				   | ALT_SDMMC_IDINTEN_FBE_SET_MSK | ALT_SDMMC_IDINTEN_DU_SET_MSK  | ALT_SDMMC_IDINTEN_CES_SET_MSK  | ALT_SDMMC_IDINTEN_AI_SET_MSK)))
			sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d: Error ALT_SDMMC_IDINTEN_OFST not set 0x%x", __FUNCTION__, __LINE__, dummy);

		out32(base + ALT_SDMMC_CTL_OFST, in32(base + ALT_SDMMC_CTL_OFST) | ALT_SDMMC_CTL_USE_INTERNAL_DMAC_SET_MSK);
		// read it back
		dummy = in32(base + ALT_SDMMC_CTL_OFST);
		if (!(dummy & ALT_SDMMC_CTL_USE_INTERNAL_DMAC_SET_MSK))
			sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d: Error DMA not set", __FUNCTION__, __LINE__);


		out32(base + ALT_SDMMC_BMOD_OFST, in32(base + ALT_SDMMC_BMOD_OFST) | ALT_SDMMC_BMOD_DE_SET_MSK);
		// read it back
		dummy = in32(base + ALT_SDMMC_BMOD_OFST);
		if (!(dummy & ALT_SDMMC_BMOD_DE_SET_MSK))
			sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d: Error ALT_SDMMC_BMOD_OFST not set", __FUNCTION__, __LINE__);
	}

	return (EOK);

}


/*****************************************************************************************
 * Function: socfpga_dma_continue
 * Description: Initialize DMA transfer descriptors
 *
 * sdio_hc_t *hc,
 * sdio_cmd_t *cmd
 *
 * return: EOK
 * */

static int socfpga_dma_continue(sdio_hc_t *hc, sdio_cmd_t *cmd )
{

	socfpga_ext_t   *socfpga;
	mmc_dma_desc_t  *bd;

	sdio_sge_t      *sgp;
	int              sgc;
	int              sg_count;
	//int              len  =  cmd->blks ? cmd->blks * cmd->blksz : cmd->blksz;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	bd = socfpga->dma_buf_desc + socfpga->dma_cur_pos;

	mmc_dma_desc_t *prev_bd = bd;

	sgc = cmd->sgc;
	sgp = cmd->sgl;

	if (sgc > hc->cfg.sg_max)
	{
		return( ENOTSUP );
	}
	sg_count = sgp->sg_count;

	if( !( cmd->flags & SCF_DATA_PHYS ) ) {
		sdio_vtop_sg( sgp, socfpga->sgl, sgc, cmd->mhdl );
		sgp = socfpga->sgl;
	}

	paddr_t tmp_paddr = sgp->sg_address;

	if (sg_count > 0) {
		while (sg_count > 0 && bd->des0.own_bit == 0) {
			int set_len = sg_count > MMCHS_DMA_SEGMENT_SIZE ? MMCHS_DMA_SEGMENT_SIZE : sg_count;
			bd->des0.own_bit = 1;
			bd->des0.dis_int_on_completion = 1;
			bd->des0.last_segment = 0;
			bd->des1.buff1_size = set_len;
			bd->des2 = tmp_paddr;
#if DEBUG
			printf("socfpga_dma_setup_add: des_addr %08X des2_paddr %08X des1_len %08X\n", (uint32_t) sdio_vtop((void *)bd), (uint32_t) tmp_paddr, set_len);
#endif
			tmp_paddr += set_len;
			sg_count -= set_len;
			if (++socfpga->dma_cur_pos >= MMCHS_DMA_DESC_NUM)
			{
				return( ENOTSUP );
			}
			prev_bd = bd;
			bd = socfpga->dma_buf_desc + socfpga->dma_cur_pos;
		}

		if(sg_count <= 0)
			prev_bd->des0.last_segment = 1;
		prev_bd->des0.dis_int_on_completion = 0;
	}

	return (EOK);

}
/*****************************************************************************************
 * Function: socfpga_dma_clear
 * Description:
 *
 * sdio_hc_t *hc
 *
 * return:	EOK
 * */


static int socfpga_dma_clear(sdio_hc_t *hc)
{

	socfpga_ext_t  *socfpga;
	uintptr_t       base;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;

	out32(base + ALT_SDMMC_IDINTEN_OFST, 0);
	out32(base + ALT_SDMMC_BMOD_OFST, in32(base + ALT_SDMMC_BMOD_OFST) & ALT_SDMMC_BMOD_DE_CLR_MSK & ALT_SDMMC_BMOD_FB_CLR_MSK);
	out32(base + ALT_SDMMC_CTL_OFST, in32(base + ALT_SDMMC_CTL_OFST) & ALT_SDMMC_CTL_USE_INTERNAL_DMAC_CLR_MSK);
	socfpga_data_completed(hc);


	return (EOK);
}

/*****************************************************************************************
 * Function: socfpga_xfer_setup
 * Description:
 *
 * sdio_hc_t *hc
 * sdio_cmd_t *cmd
 *
 * return:	EOK
 * */

static int socfpga_xfer_setup( sdio_hc_t *hc, sdio_cmd_t *cmd)
{
	uintptr_t        base;
	socfpga_ext_t   *socfpga;

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;

	// clear dma registers
	if ( hc->caps & HC_CAP_DMA )
	{
	  socfpga_dma_clear(hc);
	}
	else
	{
		out32(base + ALT_SDMMC_BMOD_OFST, in32(base + ALT_SDMMC_BMOD_OFST) & ALT_SDMMC_BMOD_DE_CLR_MSK);
	}

	// setup intmask bits
	socfpga->dmask =  ALT_SDMMC_INTMSK_CD_SET_MSK   |           // Card Detected Mask
	                  ALT_SDMMC_INTMSK_RE_SET_MSK   |           // Response error
	                  ALT_SDMMC_MINTSTS_CMD_DONE_SET_MSK |      // Command Done Mask
	                  //ALT_SDMMC_INTMSK_DTO_SET_MSK  |         // Data transfer over
	                  ALT_SDMMC_INTMSK_RCRC_SET_MSK |           // Response CRC error
	                  ALT_SDMMC_INTMSK_DCRC_SET_MSK |           // Data CRC error
	                  ALT_SDMMC_INTMSK_RTO_SET_MSK  |           // Response timeout
	                  ALT_SDMMC_INTMSK_HTO_SET_MSK  |           // Data starvation by host timeout
	                  ALT_SDMMC_INTMSK_FRUN_SET_MSK |           // FIFO underrun/overrun error
	                  ALT_SDMMC_INTMSK_EBE_SET_MSK  |           // End-bit error
	                  ALT_SDMMC_INTMSK_DRT_SET_MSK;             // Data read timeout Mask

	if( ( cmd->flags & SCF_DATA_MSK ) ) {
		sdio_sg_start( hc, cmd->sgl, cmd->sgc );
		socfpga_pio_setup(hc, cmd);

		if( cmd->sgc && ( hc->caps & HC_CAP_DMA ) )
		{
			socfpga_dma_setup(hc, cmd);
			socfpga_dma_continue(hc, cmd);
		}
	}

	return( EOK );
}

/*****************************************************************************************
 * Function: socfpga_cmd
 * Description: Driver is about to sending a command to the card.
 *              Preparing command and argument registers. Starting a command.
 *
 * sdio_hc_t *hc
 * mmc_cmd_t *cmd  - pointer to structure which holds opcode, argument, flags and response type
 *
 * return: EOK
 *
 * */

static int socfpga_cmd(sdio_hc_t *hc, sdio_cmd_t *cmd)
{

	socfpga_ext_t  *socfpga;
	uintptr_t       base;
	uint32_t        command;
	uint32_t        imask;
	int             timeout = 20000000;


	if(hc == NULL || cmd == NULL)
		return (EINVAL);


	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "%s:%d, cmd->opcode %d, cmd->flags %x, cmd->rsp %x, cmd->arg %x, cmd->blks %d, cmd->blksz %d",
				__FUNCTION__, __LINE__, cmd->opcode, cmd->flags, (unsigned int)cmd->rsp, cmd->arg, cmd->blks, cmd->blksz);
#if DEBUG
	printf("%s:%d, cmd->opcode %d, cmd->flags %x, cmd->rsp %x, cmd->arg %x, cmd->blks %d, cmd->blksz %d\n",
				__FUNCTION__, __LINE__, cmd->opcode, cmd->flags, cmd->rsp, cmd->arg, cmd->blks, cmd->blksz);
#endif

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;

	socfpga_xfer_setup( hc, cmd);

	// specify bits which are common for all operations
	command = ALT_SDMMC_CMD_CARD_NUMBER_SET(0)        |         // specify card number
	          ALT_SDMMC_CMD_CMD_INDEX_SET(cmd->opcode)|         // command index - opcode
	          ALT_SDMMC_CMD_USE_HOLD_REG_SET_MSK      |         // CMD and DATA sent to card through the HOLD Register
	          ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_SET_MSK;      // wait a completion for previous command

	switch (cmd->opcode)
	{
	    case MMC_GO_IDLE_STATE:                                 // MMC_RSP_NONE      MMC_CMD_INIT | MMC_CMD_INTR
	        command |= ALT_SDMMC_CMD_SEND_INITIALIZATION_SET_MSK;

	        break;
	    case MMC_STOP_TRANSMISSION:
	        command |= ALT_SDMMC_CMD_STOP_ABT_CMD_SET_MSK;          // Stop or abort command intended to stop current data transfer in progress.
	        command &= ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_CLR_MSK; // don't wait a completion of previous command (its related to data transfer in progress)
	        break;

	    case MMC_PROGRAM_CSD:
	    case MMC_SEND_WRITE_PROT:
	    case MMC_LOCK_UNLOCK:
	    case MMC_SEND_STATUS:
	    case MMC_SEND_NUM_WR_BLOCKS:

	        command &= ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_CLR_MSK; // don't wait a completion of previous command (its related to data transfer in progress)
	        break;
	}


	if (cmd->flags & SCF_CTYPE_ADTC)
	{
		// data transfer will be initiated by current commands
		command |= socfpga->dcmd;           // update "command" with next bits:   ALT_SDMMC_CMD_RD_WR_SET_MSK    ALT_SDMMC_CMD_SEND_AUTO_STOP_SET_MSK
		                                    //                                    ALT_SDMMC_CMD_DATA_EXPECTED_SET_MSK
	}

	if( ( cmd->flags & SCF_RSP_PRESENT ) ) {
		command |= ALT_SDMMC_CMD_RESPONSE_EXPECT_SET_MSK;  //crashes the board
		if( ( cmd->flags & SCF_RSP_136 ) ) {
			command |= ALT_SDMMC_CMD_RESPONSE_LEN_SET_MSK;
		}

		if( ( cmd->flags & SCF_RSP_CRC ) ) {		// CRC check
			command |= ALT_SDMMC_CMD_CHECK_RESPONSE_CRC_SET_MSK;
		}
	}

	if (command & ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_SET_MSK)
	{
		while (in32(base + ALT_SDMMC_STAT_OFST) & ALT_SDMMC_STAT_DATA_BUSY_SET_MSK){
			if (timeout == 0)
			{
#if DEBUG
				printf("Card data busy\n");
#endif
				sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d: Card data busy", __FUNCTION__, __LINE__);
				return (ETIMEDOUT);
			}
			timeout--;
		}
	}

	// Interrupts enable
	out32(base + ALT_SDMMC_CTL_OFST, in32(base + ALT_SDMMC_CTL_OFST) | ALT_SDMMC_CTL_INT_EN_SET(ALT_SDMMC_CTL_INT_EN_E_END));

	// Clear all possible interrupts
	out32(base + ALT_SDMMC_RINTSTS_OFST, 0xFFFFFFFF);

	imask = socfpga->dmask;			// use interrupts specified in socfpga_xfer_setup routine
	// write to INTMASK register
	out32(base + ALT_SDMMC_INTMSK_OFST,imask);


	// Setup the Argument Register and send CMD
	out32(base + ALT_SDMMC_CMDARG_OFST, ALT_SDMMC_CMDARG_CMD_ARG_SET(cmd->arg));

	// prepare command register values
	out32(base + ALT_SDMMC_CMD_OFST, command);
#if DEBUG
	printf("command %d, imask 0x%x\n", cmd->opcode, imask);
	//dump_registers(hc);
#endif

	// and start process.
	out32(base + ALT_SDMMC_CMD_OFST, command | ALT_SDMMC_CMD_START_CMD_SET_MSK);

	return (EOK);
}


/*****************************************************************************************
 * Function: socfpga_data_completed
 * Description: Disable transmit and receive fifo data interrupts
 *
 *
 * return: EOK
 * */

static int socfpga_data_completed(sdio_hc_t *hc)
{
	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);
	if(hc == NULL)
		return (EINVAL);

	socfpga_ext_t *socfpga = (socfpga_ext_t *)hc->cs_hdl;
	uintptr_t base  = socfpga->mmc_base;

	out32(base + ALT_SDMMC_INTMSK_OFST, in32(base + ALT_SDMMC_INTMSK_OFST) & ALT_SDMMC_INTMSK_TXDR_CLR_MSK & ALT_SDMMC_INTMSK_RXDR_CLR_MSK );
	return (EOK);
}



/*****************************************************************************************
 * Function: socfpga_cfg_bus
 * Description: Setup card bus width
 *
 * sdio_hc_t *hc,
 * int width  - card width value: 8 bit, 4 bit or 1 bit.
  *
 * return: EOK
 * */

static int socfpga_cfg_bus(sdio_hc_t *hc, int width)
{
	socfpga_ext_t  *socfpga;
	uintptr_t       base;
	uint32_t        ctype;

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d: width %d", __FUNCTION__, __LINE__, width);

	ctype = in32(base + ALT_SDMMC_CTYPE_OFST);
	if(width==8)
	{
		out32(base + ALT_SDMMC_CTYPE_OFST, ctype | ALT_SDMMC_CTYPE_CARD_WIDTH1_SET_MSK );
	}
	else
	{
		ctype &= ALT_SDMMC_CTYPE_CARD_WIDTH1_CLR_MSK;
		if (width == 4)
			ctype |= ALT_SDMMC_CTYPE_CARD_WIDTH2_SET_MSK;
		else
			ctype &= ALT_SDMMC_CTYPE_CARD_WIDTH2_CLR_MSK;
		out32(base + ALT_SDMMC_CTYPE_OFST, ctype);
	}
	hc->bus_width = width;

	return (EOK);
}

/*****************************************************************************************
 * Function: socfpga_send_clock_update_cmd
 * Description: Setup SD card clock value
 *
 * sdio_hc_t *hc,
 *
 * return: N/A
 *
 * */

static void socfpga_send_clock_update_cmd(sdio_hc_t *hc)
{
	socfpga_ext_t  *socfpga;
	uintptr_t       base;

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	out32(base + ALT_SDMMC_CMD_OFST, in32(base + ALT_SDMMC_CMD_OFST) |
	             ALT_SDMMC_CMD_UPDATE_CLK_REGS_ONLY_SET_MSK          |   // Do not send commands, just update clock register value into card clock domain
	             ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_SET_MSK         |   // Wait for previous data transfer completion before sending command
	             ALT_SDMMC_CMD_START_CMD_SET_MSK);                       // and start "command"

	// Wait for the clock to be stable
	// wait for command completion
	while (ALT_SDMMC_CMD_START_CMD_GET(in32(base + ALT_SDMMC_CMD_OFST)));
	return;
}


/*****************************************************************************************1267

 * Function: socfpga_clk
 * Description: Setup SD card clock value
 *
 * sdio_hc_t *hc,
 * int *clock,
 *
 * return: EOK or error
 *
 * */

static int socfpga_clk(sdio_hc_t *hc, int clock)
{
	socfpga_ext_t *socfpga;
	uintptr_t base;
	int clkd;
	int loop = 1000;

	socfpga = (socfpga_ext_t *) hc->cs_hdl;
	base = socfpga->mmc_base;

	// calculate clk_divider0 value
	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1,
			"%s:%d, socfpga->mmc_clock %d, clock %d", __FUNCTION__, __LINE__,
			socfpga->mmc_clock, clock);

	clkd = socfpga->mmc_clock / (clock);
	if (socfpga->mmc_clock != (clock) * clkd)
		clkd++;
	clock = socfpga->mmc_clock / clkd;			// update clock value
	hc->clk = clock;
	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1,
			"%s:%d, socfpga->mmc_clock %d, clock %d, div %d", __FUNCTION__,
			__LINE__, socfpga->mmc_clock, clock, clkd);

	while ((ALT_SDMMC_STAT_DATA_BUSY_GET(in32(base + ALT_SDMMC_STAT_OFST)) != 0)
			&& loop) {
		loop--;
	}
	if (loop == 0) {
		sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0,
				"%s:%d error card data busy", __FUNCTION__, __LINE__);
		return (MMC_FAILURE);

	}

	// disable clock
	out32(base + ALT_SDMMC_CLKENA_OFST,
			in32(base + ALT_SDMMC_CLKENA_OFST) & ALT_SDMMC_CLKENA_CCLK_EN_CLR_MSK);
	// switch clock source to Clock divider 0 for card 0
	out32(base + ALT_SDMMC_CLKSRC_OFST,
			in32(base + ALT_SDMMC_CLKSRC_OFST) & ALT_SDMMC_CLKSRC_CLK_SRC_CLR_MSK);

	socfpga_send_clock_update_cmd(hc);

	// setup clk_divider0 of the Clock Divider Register
	// divide by four
	out32(base + ALT_SDMMC_CLKDIV_OFST, (in32(base + ALT_SDMMC_CLKDIV_OFST)
					& ALT_SDMMC_CLKDIV_CLK_DIVR0_CLR_MSK)
					| ALT_SDMMC_CLKDIV_CLK_DIVR0_SET(clkd >> 1));

	socfpga_send_clock_update_cmd(hc);

	out32(base + ALT_SDMMC_CLKENA_OFST,
			in32(base + ALT_SDMMC_CLKENA_OFST) | ALT_SDMMC_CLKENA_CCLK_EN_SET_MSK);

	socfpga_send_clock_update_cmd(hc);

	return (EOK);
}

/*****************************************************************************************
 * Function: socfpga_pwr
 * Description: Reset host controller and card
 * The clock should be enabled and set to minimum (<400KHz)
 *
 * sdio_hc_t *hc
 * int vdd
 *
 * return: EOK
 * */


static int socfpga_pwr(sdio_hc_t *hc, int vdd)
{

	socfpga_ext_t      *socfpga;
	uintptr_t           base;

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;

	if (vdd){
	    sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d: vdd %d", __FUNCTION__, __LINE__, vdd);

	    // Enable power
	    out32(base + ALT_SDMMC_PWREN_OFST, 0x0);
	    out32(base + ALT_SDMMC_FIFOTH_OFST, ALT_SDMMC_FIFOTH_DW_DMA_MULT_TRANSACTION_SIZE_SET(ALT_SDMMC_FIFOTH_DW_DMA_MULT_TRANSACTION_SIZE_E_RXMSIZEK1) |
	                                        ALT_SDMMC_FIFOTH_RX_WMARK_SET((MMCHS_FIFO_SIZE >> 3)-1 ) |
	                                        ALT_SDMMC_FIFOTH_TX_WMARK_SET((MMCHS_FIFO_SIZE >> 3)));

	    // Configure timeouts
	    out32(base + ALT_SDMMC_TMOUT_OFST, ALT_SDMMC_TMOUT_RESPONSE_TMO_SET(0xFF) | ALT_SDMMC_TMOUT_DATA_TMO_SET(0xFFFFFF) );


	    out32(base + ALT_SDMMC_DEBNCE_OFST, 0x0FFFFF);

	    // Clear any interrupts
	    out32(base + ALT_SDMMC_RINTSTS_OFST, 0xFFFFFFFF);

	    // Disable All interrupts
	    out32(base + ALT_SDMMC_INTMSK_OFST, 0x0);

	    out32(base + ALT_SDMMC_CTL_OFST, ALT_SDMMC_CTL_INT_EN_SET(ALT_SDMMC_CTL_INT_EN_E_END));

	    // Reset cart type
	    // setup card type to 1bit
	    out32(base + ALT_SDMMC_CTYPE_OFST, ALT_SDMMC_CTYPE_CARD_WIDTH1_SET(ALT_SDMMC_CTYPE_CARD_WIDTH1_E_NON8BIT) |
	                                       ALT_SDMMC_CTYPE_CARD_WIDTH2_SET(ALT_SDMMC_CTYPE_CARD_WIDTH2_E_MOD1BIT));

	    out32(base + ALT_SDMMC_CLKENA_OFST, in32(base + ALT_SDMMC_CLKENA_OFST) | ALT_SDMMC_CLKENA_CCLK_EN_SET_MSK);

	    out32(base + ALT_SDMMC_CMD_OFST, ALT_SDMMC_CMD_UPDATE_CLK_REGS_ONLY_SET_MSK |
	                                     ALT_SDMMC_CMD_WAIT_PRVDATA_COMPLETE_SET_MSK |
	                                     ALT_SDMMC_CMD_START_CMD_SET_MSK);

	    // Enable power
	    out32(base + ALT_SDMMC_PWREN_OFST, 0xFFFF);

	    // Enumerated Card Stack

	}	// if (vdd)
	else
	{
		socfpga_powerdown(hc);
	}
	hc->vdd = vdd;

	return (EOK);
}


/*****************************************************************************************
 * Function: socfpga_cd
 * Description: Return card status
 *
 * sdio_hc_t *hc
 *
 * return: SDIO_TRUE
 * */

static int socfpga_cd(sdio_hc_t *hc)
{
	socfpga_ext_t  *socfpga;
	uintptr_t       base;
	uint32_t        status = SDIO_TRUE;

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;

	if (in32(base + ALT_SDMMC_CDETECT_OFST))
	{
		//status = SDIO_FALSE;
		//sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d: No SD card inserted", __FUNCTION__, __LINE__);
	}
	return (status);
}


/*****************************************************************************************
 * Function: socfpga_powerdown
 * Description: Disconnect power from SD card and resets a controller, FIFO and DMA
 *
 * sdio_hc_t *hc
 *
 * return: EOK or error
 * */

static int socfpga_powerdown(sdio_hc_t *hc)
{
#if DEBUG
	printf("socfpga_powerdown: \n");
#endif

	socfpga_ext_t   *socfpga;
	uintptr_t        base;
	unsigned int     timeout = 100;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;

	// Disable power
	out32(base + ALT_SDMMC_PWREN_OFST, ALT_SDMMC_PWREN_POWER_EN_SET(ALT_SDMMC_PWREN_POWER_EN_E_OFF));
	out32(base + ALT_SDMMC_CTL_OFST, in32(base + ALT_SDMMC_CTL_OFST) & ALT_SDMMC_CTL_USE_INTERNAL_DMAC_CLR_MSK);

	// Disable All interrupts
	out32(base + ALT_SDMMC_INTMSK_OFST, 0);

	// Clear any interrupts
	out32(base + ALT_SDMMC_RINTSTS_OFST, 0xFFFFFFFF);

	if(hc->caps & HC_CAP_DMA)
	{
		out32(base + ALT_SDMMC_BMOD_OFST, in32(base + ALT_SDMMC_BMOD_OFST) & ALT_SDMMC_BMOD_DE_CLR_MSK);
	}

	// Reset hardware
	out32(base + ALT_SDMMC_CTL_OFST, ALT_SDMMC_CTL_CTLLER_RST_SET_MSK            // reset controller
	           | ALT_SDMMC_CTL_FIFO_RST_SET_MSK                                  // and FIFO pointers
	           | ALT_SDMMC_CTL_DMA_RST_SET_MSK);                                 // and DMA

	/* hw clears the bit when it's done */
	while (in32(base + ALT_SDMMC_CTL_OFST) & (ALT_SDMMC_CTL_CTLLER_RST_SET_MSK | ALT_SDMMC_CTL_FIFO_RST_SET_MSK	| ALT_SDMMC_CTL_DMA_RST_SET_MSK))
	{
		if (timeout == 0)
		{
#if DEBUG
			printf("Reset host timeout error\n");
#endif
			sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s:%d error reset timeout", __FUNCTION__, __LINE__);
			return (MMC_FAILURE);
		}
		timeout--;
		delay(1);
	}

	return (EOK);
}

/*****************************************************************************************
 * Function: socfpga_dinit
 * Description: Perform shutdown of the socfpga sdmmc controller
 *
 * sdio_hc_t *hc
 *
 * return: EOK
 * */

static int socfpga_dinit(sdio_hc_t *hc)
{

	socfpga_ext_t     *socfpga;

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	socfpga_powerdown(hc);
	if( hc->hc_iid != -1 ) {
		InterruptDetach( hc->hc_iid );
	}

	if(hc->caps & HC_CAP_DMA)
	{
		sdio_free((void *)socfpga->dma_buf_desc, sizeof(mmc_dma_desc_t) * socfpga->dma_num_descs);
	}
	munmap_device_memory((void *)socfpga->mmc_base, SOCFPGA_MMCHS_SIZE);

	free(socfpga);
	return (EOK);
}

/*****************************************************************************************
 * Function: socfpga_get_syspage_clk
 * Description: Get "mmc" clock value from the system page "hwi" area
 *
 *
 * return: Clock value
 *         0 - error
 * */

static unsigned socfpga_get_syspage_clk(void)
{
	unsigned     item, offset, clock = 0;
	char        *name;
	hwi_tag     *tag;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, 0, 3, "%s:%d", __FUNCTION__, __LINE__);
	item = hwi_find_item(HWI_NULL_OFF, HWI_ITEM_DEVCLASS_DISK, "mmc", NULL);
	if (item == HWI_NULL_OFF)
		return 0;

	offset = item;

	while ((offset = hwi_next_tag(offset, 1)) != HWI_NULL_OFF) {
		tag = hwi_off2tag(offset);
		name = __hwi_find_string(((hwi_tag *)tag)->prefix.name);

		if (strcmp(name, HWI_TAG_NAME_inputclk) == 0)
			clock = ((struct hwi_inputclk *)tag)->clk / ((struct hwi_inputclk *)tag)->div;
	}
	sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, 0, 0, "%s:%d: clock %d", __FUNCTION__, __LINE__, clock);

	return clock;
}

static int socfpga_bus_mode( sdio_hc_t *hc, int bus_mode )
{
	hc->bus_mode = bus_mode;
	return( EOK );
}

static int socfpga_signal_voltage( sdio_hc_t *hc, int signal_voltage )
{
	hc->signal_voltage = signal_voltage;

	return( EOK );
}
/*****************************************************************************************
*/

static int socfpga_timing( sdio_hc_t *hc, int timing )
{

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "%s:%d: timing %d", __FUNCTION__, __LINE__, timing);
	hc->timing = timing;

	return( EOK );
}
/*****************************************************************************************/
static sdio_hc_entry_t altera_hc_entry ={ 16,
			   socfpga_dinit, NULL,
			   socfpga_cmd, NULL,
			   socfpga_event, socfpga_cd, socfpga_pwr,
			   socfpga_clk, socfpga_bus_mode,
			   socfpga_cfg_bus, socfpga_timing,
			   socfpga_signal_voltage, NULL,
			   NULL, NULL,
			   NULL
};

/*****************************************************************************************
 * Function: socfpga_init
 * Description: Creation of socfpga context instance. Mapping sdmmc controller registers in driver's user memory
 *
 * sdio_hc_t *hc
 *
 * return:  EOK or
 *          error
 * */


int	socfpga_init(sdio_hc_t *hc )
{
	sdio_hc_cfg_t       *cfg;
	socfpga_ext_t       *socfpga;
	struct sigevent      event;

	cfg = (sdio_hc_cfg_t *)&hc->cfg;
	hc->hc_iid  = -1;

	sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 3, "%s:%d", __FUNCTION__, __LINE__);

	if ((socfpga = calloc(1, sizeof(socfpga_ext_t))) == NULL)
		return (ENOMEM);

	memset(socfpga, 0, sizeof(socfpga_ext_t));

	memcpy( &hc->entry, &altera_hc_entry, sizeof( sdio_hc_entry_t ) );

	socfpga->mmc_pbase = cfg->base_addr[0] ;	// get physical address of the sdmmc controller

	if ((socfpga->mmc_base = (uintptr_t)mmap_device_memory( NULL, cfg->base_addr_size[0],
				PROT_READ | PROT_WRITE | PROT_NOCACHE, MAP_SHARED,
				cfg->base_addr[0] ) ) == (uintptr_t)MAP_FAILED )
	{
		sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "%s() Socfpga SDMMC: mmap_device_io failed", __FUNCTION__);
		goto fail1;
	}

	hc->cs_hdl = socfpga;
	if (cfg->clk)
		hc->clk_max = cfg->clk;
	else
		hc->clk_max = ALTERA_CLOCK_DEFAULT;

	// Set Capability bits
	hc->caps |= HC_CAP_HS      |        /* Host support high speed */
	            HC_CAP_DMA     |
	            HC_CAP_SV_3_0V |        /* Host support 3.0V */
	            HC_CAP_SV_3_3V |        /* Host support 3.3V */
	            //MMC_HCCAP_BW1 |       /* Host support 1 bit bus (mandatory) */
	            HC_CAP_BW4     |        /* Host support 4 bit bus (mandatory) */
	            HC_CAP_ACMD12;          /* Host support auto-stop command(ACMD12) */

	hc->ocr	   = OCR_VDD_32_33 | OCR_VDD_33_34;
	hc->caps  |= HC_CAP_SV_3_3V;

	if ((socfpga->mmc_clock = socfpga_get_syspage_clk()) == 0)
		socfpga->mmc_clock = 100000000;

	hc->caps  &= cfg->caps;             /* reconcile command line options */

	if(hc->caps & HC_CAP_DMA)
	{
		socfpga->dma_num_descs = MMCHS_DMA_DESC_NUM;
		socfpga->dma_cur_pos = 0;

		if (socfpga_alloc_desc_ring(hc) != EOK)
			goto fail0;
		socfpga_init_desc_ring(hc);
		hc->cfg.sg_max	= 1;
	}

	SIGEV_PULSE_INIT( &event, hc->hc_coid, SDIO_PRIORITY, HC_EV_INTR, NULL );
	if( ( hc->hc_iid = InterruptAttachEvent( cfg->irq[0], &event, _NTO_INTR_FLAGS_TRK_MSK ) ) == -1 ) {
		sdio_slogf( _SLOGC_SDIODI, _SLOG_INFO, hc->cfg.verbosity, 1, "%s: InterrruptAttachEvent (irq 0x%x) - %s", __FUNCTION__, cfg->irq[0], strerror( errno ) );
		socfpga_dinit( hc );
		return( errno );
	}
	else
	{
		sdio_slogf( _SLOGC_SDIODI, _SLOG_ERROR, hc->cfg.verbosity, 0, "%s: InterrruptAttachEvent (irq %d) ", __FUNCTION__, cfg->irq[0]);
	}

	socfpga_pwr(hc,0);
	return (EOK);

fail0:
	munmap_device_memory((void *)socfpga->mmc_base, SOCFPGA_MMCHS_SIZE);
fail1:
	free(socfpga);

	return (EIO);
}
/*****************************************************************************************/
#if DEBUG

void dump_registers(sdio_hc_t *hc)
{
	socfpga_ext_t   *socfpga;
	uintptr_t        base;

	int i = ALT_SDMMC_CTL_OFST;//0;
	int j = ALT_SDMMC_BACK_END_POWER_R_OFST;//ALT_SDMMC_STAT_OFST; //

	socfpga = (socfpga_ext_t *)hc->cs_hdl;
	base  = socfpga->mmc_base;

	for(; i <= j; i += 4)
	{
		printf("Reg 0x%x: 0x%x\n", i, in32(base + i));
	}
	return;
}
#endif
#endif




#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/devb/sdmmc/sdiodi/hc/cyclone5.c $ $Rev: 895829 $")
#endif
