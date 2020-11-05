/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017-2018 NXP
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


#include "externs.h"

/**
 * @file       src/hardware/devc/sermx8/tto.c
 * @addtogroup devc
 * @{
 */

/*
 * Disable DMA after 40 consecutive DMA TX transfer timeouts.
 * 40 * 50ms delay = 2000ms.
 */
#ifdef USE_DMA
    #define MAX_TIMEOUT (40)
#endif



/**
 * This function takes data from io-char's output buffer and gives it to the hardware.
 * It also deals with stty commands, by calling ser_stty(), and provides line control and line status information.
 *
 * @param ttydev Pointer to the driver's TTYDEV structure.
 * @param action One of:
 *   TTO_STTY - An stty command was received. It's called by io-char when the stty command is performed on the device. This action calls ser_stty(); the argument is ignored.
 *   TTO_CTRL - set the characteristics of the port i.e. control RS-232 modem lines.
 *       arg1 _SERCTL_BRK_CHG - called by io-char when the application requests a break such as tcsendbreak() be sent
 *       arg1 _SERCTL_DTR_CHG - changes the DTR line
 *       arg1 _SERCTL_RTS_CHG - changes the RTS line; io-char calls this to assert hardware flow control when the input buffer is filling up (based on the high-water level)
 *   TTO_LINESTATUS - a request for line status. Returns the status of the Modem Status and Modem Control registers when the user performs a devctl() with DCMD_CHR_LINESTATUS; the argument is ignored.
 *   TTO_DATA - used if tto() is called directly from the interrupt handler to transmit data or when io-char's write handler calls down to initiate a transfer.
 *   TTO_EVENT - used to call into the tto() at thread time to transmit data. The interrupt handler can return this event rather than calling tto() directly.
 * @param arg1 A data value which has different meanings for different actions. It's used to pass flags that modify the action.
 *
 * @return Execution status
 */
int tto(TTYDEV *ttydev, int action, int arg1)
{
    TTYBUF           *bup = &ttydev->obuf;
    DEV_UART         *dev = (DEV_UART *)ttydev;
    uintptr_t        base = dev->base;
    unsigned char    c;
    unsigned         cr1;
#ifdef USE_DMA
    static uint32_t  byte_cnt = 0;
    dma_transfer_t   tinfo;
    dma_addr_t       src_dma_addr, dst_dma_addr;
    uint32_t         stat_reg;
#endif
    switch (action) {
        case TTO_STTY:
            ser_stty(dev);
            return 0;

        case TTO_CTRL:
            if (arg1 & _SERCTL_BRK_CHG) {
                cr1 = in32(base + IMX_LPUART_CTRL);

                if (arg1 &_SERCTL_BRK)
                    cr1 |= IMX_LPUART_CTRL_SBK_MASK;
                else
                    cr1 &= ~IMX_LPUART_CTRL_SBK_MASK;

                out32(base + IMX_LPUART_CTRL, cr1);
            }

            /*
             * RTS Control
             */
            if (arg1 & _SERCTL_RTS_CHG) {
                if (!dev->usedma) {
                    /*
                     * Enable/disable RX interrupts to assert/clear input HW flow control
                     */
                    if (arg1 & _SERCTL_RTS) {
                        out32(base + IMX_LPUART_CTRL, in32(base + IMX_LPUART_CTRL) | (IMX_LPUART_CTRL_RIE_MASK));
                    }
                    else {
                        out32(base + IMX_LPUART_CTRL, in32(base + IMX_LPUART_CTRL) & ~(IMX_LPUART_CTRL_RIE_MASK));
                    }
                }
#if defined(USE_DMA)
                else if (dev->usedma) {
                    if (arg1 & _SERCTL_RTS) {
                        /* io-char buffer has more space. Resume receive. */
                        atomic_clr(&dev->tty.flags, IHW_PAGED);

                        /* Read status register */
                        stat_reg = in32(base + IMX_LPUART_STAT);
                        if ((stat_reg & DEVC_LPUART_ERROR_MASK) != 0 ) {
                            /* DMA error or LPUART error detected */
                            atomic_set(&dev->tty.flags, OBAND_DATA);
                            /* Check LPUART errors */
                            if ((stat_reg & IMX_LPUART_STAT_FE_MASK) && (stat_reg & IMX_LPUART_STAT_RDRF_MASK)) {
                                /* Break detected */
                                dev->rx_dma.key |= TTI_BREAK;
                                slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Break detected", __FUNCTION__);
                            }
                            if (stat_reg & IMX_LPUART_STAT_FE_MASK) {
                                /* Framing error detected */
                                dev->rx_dma.key |= TTI_FRAME;
                                slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Framing error detected", __FUNCTION__);
                            }
                            if (stat_reg & IMX_LPUART_STAT_PF_MASK) {
                                /* Parity error detected */
                                dev->rx_dma.key |= TTI_PARITY;
                                slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Parity error detected", __FUNCTION__);
                            }
                            if (stat_reg & IMX_LPUART_STAT_OR_MASK) {
                                /* Overrun error detected */
                                dev->rx_dma.key |= TTI_OVERRUN;
                                /* If RX overrun has been detected even if RTS/CTS is enabled, it means that RTS line was ignored by transmitting device. In this case LPUART Rx FIFO will be flushed. */
                                out32(base + IMX_LPUART_FIFO, in32(base + IMX_LPUART_FIFO) | IMX_LPUART_FIFO_RXFLUSH_MASK); /* Flush Rx FIFO */
                                slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_ERROR, "%s: Overrun error detected", __FUNCTION__);
                            }
                            /* Clear error status masks in the status register */
                            out32(base + IMX_LPUART_STAT, (stat_reg | DEVC_LPUART_ERROR_MASK));
                        }

                        if (dev->rx_dma.buffer0) {
                            dev->rx_dma.status |= tti2(&dev->tty, (unsigned char *)dev->rx_dma.buf, dev->rx_dma.bytes_read, dev->rx_dma.key);
                            /* Prepare RX DMA info structure using RX DMA buffer 1 */
                            lpuart_prep_rx_dma_transfer(dev, tinfo, src_dma_addr, dst_dma_addr, dev->rx_dma.phys_addr + dev->rx_dma.xfer_size + 1);

                            dev->rx_dma.buffer0 ^= 1;
                            /* Set up DMA RX transfer */
                            dev->sdmafuncs.setup_xfer(dev->rx_dma.dma_chn, &tinfo);
                            /* Start DMA RX transfer using RX DMA buffer 1 */
                            dev->sdmafuncs.xfer_start(dev->rx_dma.dma_chn);
                        } else {
                            dev->rx_dma.status |= tti2(&dev->tty, (unsigned char *)(dev->rx_dma.buf + dev->rx_dma.xfer_size + 1), dev->rx_dma.bytes_read, dev->rx_dma.key);
                            /* Prepare RX DMA info structure using RX DMA buffer 0 */
                            lpuart_prep_rx_dma_transfer(dev, tinfo, src_dma_addr, dst_dma_addr, dev->rx_dma.phys_addr);

                            dev->rx_dma.buffer0 ^= 1;
                            /* Set up DMA RX transfer */
                            dev->sdmafuncs.setup_xfer(dev->rx_dma.dma_chn, &tinfo);
                            /* Start DMA RX transfer using RX DMA buffer 0 */
                            dev->sdmafuncs.xfer_start(dev->rx_dma.dma_chn);
                        }

                        if (dev->rx_dma.status) {
                            iochar_send_event (&dev->tty);
                        }
                    }
                    /* else if ibuf is almost full do nothing since RX DMA handler will not
                     * trigger a new transfer if there is no room in the ibuf.
                     */
                }
#endif
            }
            return 0;

        case TTO_LINESTATUS:
              return (in32(base + IMX_LPUART_STAT) & 0xFFFF);

        case TTO_DATA:
        case TTO_EVENT:
            break;

        default:
            return 0;
    }


    if (dev->usedma) {
#if defined(USE_DMA)
        pthread_mutex_lock(&dev->mutex);
        if (dev->tx_xfer_active) {
            /* TX DMA transfer is active so wait for the transfer to complete before starting a new transfer */
            pthread_mutex_unlock(&dev->mutex);
            return 0;
        }
        else {
            while(bup->cnt > 0 && byte_cnt < dev->tx_dma.xfer_size) {
                if ((dev->tty.flags & (OHW_PAGED | OSW_PAGED)) && !(dev->tty.xflags & OSW_PAGED_OVERRIDE)) {
                    break;
                }

                if ((dev->tty.c_oflag & OPOST) || (dev->tty.xflags & OSW_PAGED_OVERRIDE)) {
                    /*
                     * Get the next character to print from the output buffer
                     */
                    dev_lock(&dev->tty);
                    c = tto_getchar(&dev->tty);
                    dev_unlock(&dev->tty);
                    dev->tx_dma.buf[byte_cnt++] = c;

                    /*
                     * Clear the OSW_PAGED_OVERRIDE flag as we only want
                     * one character to be transmitted in this case.
                     */
                    if (dev->tty.xflags & OSW_PAGED_OVERRIDE) {
                        atomic_clr (&dev->tty.xflags, OSW_PAGED_OVERRIDE);
                    }

                } else {
                    int buf_n, first_pass;
                    dev_lock(&dev->tty);
                    buf_n = min(bup->cnt, dev->tx_dma.xfer_size - byte_cnt);
                    first_pass = &bup->buff[bup->size] - bup->tail;
                    if (buf_n <= first_pass) {
                        memcpy(dev->tx_dma.buf + byte_cnt, bup->tail, buf_n);
                        bup->tail += buf_n;
                        if (bup->tail == &bup->buff[bup->size]) {
                            bup->tail = bup->buff;
                        }
                    } else {
                        memcpy(dev->tx_dma.buf + byte_cnt, bup->tail, first_pass);
                        memcpy(dev->tx_dma.buf + byte_cnt + first_pass, bup->buff, buf_n - first_pass);
                        bup->tail = bup->buff + (buf_n - first_pass);
                    }
                    bup->cnt -= buf_n;
                    byte_cnt += buf_n;
                    dev_unlock (&dev->tty);
                }
            }
            dev_lock(&dev->tty);
            if (byte_cnt && !(dev->tty.flags & (OHW_PAGED | OSW_PAGED))) {
                /* Prepare TX DMA info structure */
                lpuart_prep_tx_dma_transfer(dev, tinfo, src_dma_addr, dst_dma_addr, byte_cnt);
                dev->tx_xfer_active = TRUE;
                byte_cnt = 0;
                dev_unlock(&dev->tty);

                /* Set up DMA TX transfer */
                dev->sdmafuncs.setup_xfer(dev->tx_dma.dma_chn, &tinfo);

                /* Start DMA TX transfer */
                dev->sdmafuncs.xfer_start(dev->tx_dma.dma_chn);
            } else {
                dev_unlock(&dev->tty);
            }
        }
    pthread_mutex_unlock(&dev->mutex);
#endif
    }
    if (!dev->usedma) {
        while ((bup->cnt > 0) && ((in32(base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TDRE_MASK) != 0)) {
            /*
             * If the OSW_PAGED_OVERRIDE flag is set then allow
             * transmit of character even if output is suspended via
             * the OSW_PAGED flag. This flag implies that the next
             * character in the obuf is a software flow control
             * character (STOP/START).
             * Note: tx_inject sets it up so that the control
             *       character is at the start (tail) of the buffer.
             */
            if ((dev->tty.flags & (OHW_PAGED|OSW_PAGED)) && !(dev->tty.xflags & OSW_PAGED_OVERRIDE)) {
                break;
            }

            /*
             * Get the next character to print from the output buffer
             */
            dev_lock(&dev->tty);
            c=tto_getchar(&dev->tty);
            dev_unlock(&dev->tty);

            dev->tty.un.s.tx_tmr = 3;        /* Timeout 3 */
            out32(base + IMX_LPUART_DATA, (uint32_t)c);

            /* Clear the OSW_PAGED_OVERRIDE flag as we only want
             * one character to be transmitted in this case.
             */
            if (dev->tty.xflags & OSW_PAGED_OVERRIDE) {
                atomic_clr(&dev->tty.xflags, OSW_PAGED_OVERRIDE);
                break;
            }
        }
        if (!(dev->tty.flags & (OHW_PAGED|OSW_PAGED)) && (bup->cnt)) {
            cr1 = in32(base + IMX_LPUART_CTRL);
            out32(base + IMX_LPUART_CTRL, cr1 | IMX_LPUART_CTRL_TIE_MASK);
        }
    }

    return (tto_checkclients(&dev->tty));
}

/*FUNCTION**********************************************************************
 *
 * Function Name : LPUART_HAL_SetBaudRate
 * Description   : Configures the LPUART baud rate.
 * In some LPUART instances the user must disable the transmitter/receiver
 * before calling this function.
 * Generally, this may be applied to all LPUARTs to ensure safe operation.
 *
 *END**************************************************************************/
/**
 * Calculates the baudrate and initializes BAU rate register according to the desired baudrate.
 *
 * @param dev             Pointer to a device structure.
 * @param sourceClockInHz LPUART input clock source in HZ.
 * @param desiredBaudRate Desired baud rate.
 */
static void lpuart_setbaudrate(DEV_UART *dev, uint32_t sourceClockInHz, uint32_t desiredBaudRate)
{
    uintptr_t base = dev->base;
    uint16_t sbr, sbrTemp, i;
    uint32_t osr, tempDiff, calculatedBaud, baudDiff;
    uint32_t baudReg;

    /* This lpuart instantiation uses a slightly different baud rate calculation
     * The idea is to use the best OSR (over-sampling rate) possible
     * Note, osr is typically hard-set to 16 in other lpuart instantiations
     * First calculate the baud rate using the minimum OSR possible (4) */
    osr = 4;
    sbr = (sourceClockInHz / (desiredBaudRate * osr));
    calculatedBaud = (sourceClockInHz / (osr * sbr));

    if (calculatedBaud > desiredBaudRate) {
        baudDiff = calculatedBaud - desiredBaudRate;
    } else {
        baudDiff = desiredBaudRate - calculatedBaud;
    }

    /* Loop to find the best osr value possible, one that generates minimum baudDiff
     * iterate through the rest of the supported values of osr */
    for (i = 5; i <= 32; i++) {
        /* Calculate the temporary sbr value   */
        sbrTemp = (sourceClockInHz / (desiredBaudRate * i));
        /* Calculate the baud rate based on the temporary osr and sbr values */
        calculatedBaud = (sourceClockInHz / (i * sbrTemp));

        if (calculatedBaud > desiredBaudRate) {
            tempDiff = calculatedBaud - desiredBaudRate;
        } else {
            tempDiff = desiredBaudRate - calculatedBaud;
        }

        if (tempDiff <= baudDiff) {
            baudDiff = tempDiff;
            osr = i;        /* Update and store the best osr value calculated */
            sbr = sbrTemp;  /* Update store the best sbr value calculated */
        }
    }

    /* Check to see if actual baud rate is within 3% of desired baud rate
     * based on the best calculate osr value */
    if (baudDiff < ((desiredBaudRate / 100) * 3)) {
        /* Acceptable baud rate, check if osr is between 4x and 7x oversampling.
         * If so, then "BOTHEDGE" sampling must be turned on */
        baudReg = in32(base + IMX_LPUART_BAUD);
        if ((osr > 3) && (osr < 8)) {
            baudReg |= IMX_LPUART_BAUD_BOTHEDGE_MASK;
        }

        /* Program the osr value (bit value is one less than actual value) to the temporary baudReg variable */
        baudReg &= ~IMX_LPUART_BAUD_OSR_MASK;
        baudReg |= ((osr - 1) << IMX_LPUART_BAUD_OSR_SHIFT);

        /* Write the sbr value to the temporary baudReg variable */
        baudReg &= ~IMX_LPUART_BAUD_SBR_MASK;
        baudReg |= ((sbr) << IMX_LPUART_BAUD_SBR_SHIFT);

        /* Write the baudReg variable into the IMX_LPUART_BAUD register */
        out32(dev->base + IMX_LPUART_BAUD, baudReg);
    } else {
        /* Unacceptable baud rate difference of more than 3% */
        if (dev->tty.verbose >= 5) {
            slogf(_SLOG_SETCODE(_SLOGC_CHAR, 0), _SLOG_INFO, "serial: Unacceptable baud rate difference of more than 3%% for device %s", dev->tty.name);
        }
    }
}


/**
 * Configures registers that can be changed dynamically at runtime (baud, parity, stop bits, etc.).
 *
 * @param dev Pointer to a device structure.
 */
void ser_stty(DEV_UART *dev)
{
    uintptr_t base = dev->base;
    uint32_t  ctrlReg, baudReg, statReg;
    uint32_t  timeout;


#if defined(USE_DMA)
    if (dev->usedma) {
        /* Wait for DMA transmission complete flag */
        timeout = 100000;
        while (dev->tx_xfer_active && timeout--) {}
    }
#endif
    /* Wait for LPUART to finish transmission and reception */
    timeout = 100000;
    while (!(in32(dev->base + IMX_LPUART_FIFO) & IMX_LPUART_FIFO_TXEMPT_MASK) &&
           !(in32(dev->base + IMX_LPUART_FIFO) & IMX_LPUART_FIFO_RXEMPT_MASK) &&
           !(in32(dev->base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TC_MASK) &&
           !(in32(dev->base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TDRE_MASK) && timeout--) {}

    /* Disable LPUART receiver & transmitter */
     ctrlReg = in32(dev->base + IMX_LPUART_CTRL) & (~(IMX_LPUART_CTRL_RE_MASK | IMX_LPUART_CTRL_TE_MASK));
     out32(dev->base + IMX_LPUART_CTRL, ctrlReg);

    /* Clear status flags */
    statReg = in32(dev->base + IMX_LPUART_STAT) |
             (IMX_LPUART_STAT_LBKDIF_MASK |
              IMX_LPUART_STAT_RXEDGIF_MASK |
              IMX_LPUART_STAT_IDLE_MASK |
              IMX_LPUART_STAT_OR_MASK |
              IMX_LPUART_STAT_NF_MASK |
              IMX_LPUART_STAT_FE_MASK |
              IMX_LPUART_STAT_PF_MASK |
              IMX_LPUART_STAT_MA1F_MASK |
              IMX_LPUART_STAT_MA2F_MASK);
    out32(dev->base + IMX_LPUART_STAT, statReg);

     /*
     * Check hardware flow control setting
     * NOTE: On this hardware CTS is the output and RTS is the input.
     * Therefore the CTS output is responsible for input flow control, and the
     * RTS input is responsible for output flow control.
     */
    dev->modir = in32(base + IMX_LPUART_MODIR);
    /* Check if we need to enable or disable CTS */
    if ((dev->tty.c_cflag & IHFLOW) && !(dev->modir & IMX_LPUART_MODIR_TXCTSE_MASK)) {
        /*
         * If input flow control is enabled CTS (i.e. the UART
         * toggles the CTS output based on the RX/input buffer level)
         */
        dev->modir |= IMX_LPUART_MODIR_TXCTSE_MASK;
        out32(base + IMX_LPUART_MODIR, dev->modir);
    } else if (!(dev->tty.c_cflag & IHFLOW) && (dev->modir & IMX_LPUART_MODIR_TXCTSE_MASK)) {
        /* If input flow control is disabled then disable CTS */
        dev->modir &= ~(IMX_LPUART_MODIR_TXCTSE_MASK);
        out32(base + IMX_LPUART_MODIR, dev->modir);
    }

    /* Check if the transmitter should transmit based or the RTS input, or ignore the RTS input */
    if ((dev->tty.c_cflag & OHFLOW) && !(dev->modir & IMX_LPUART_MODIR_RXRTSE_MASK)) {
        /* If output flow control is enable dthen disable RTS */
        dev->modir |= IMX_LPUART_MODIR_RXRTSE_MASK;     /* Enable RTS */
        out32(base + IMX_LPUART_MODIR, dev->modir);
    } else if (!(dev->tty.c_cflag & OHFLOW) && (dev->modir & IMX_LPUART_MODIR_RXRTSE_MASK)) {
        dev->modir &= ~(IMX_LPUART_MODIR_RXRTSE_MASK);  /* Disable RTS */
        out32(base + IMX_LPUART_MODIR, dev->modir);
    }

    /* Calculate and set the baudrate */
    lpuart_setbaudrate(dev, dev->clk, dev->tty.baud);

    dev->ctrl = ctrlReg = in32(base + IMX_LPUART_CTRL);
    /* Set 8-bits LPUART mode */
    ctrlReg &= ~IMX_LPUART_CTRL_M_MASK;
    switch (dev->tty.c_cflag & CSIZE) {
        case CS8:
            /* Set 8-bits data width */
            ctrlReg &= ~IMX_LPUART_CTRL_M7_MASK;
            break;
        case CS7:
            /* Set 7-bits data width */
            ctrlReg |= IMX_LPUART_CTRL_M7_MASK;
            break;
    }

    /* Stop bits */
    baudReg = in32(base + IMX_LPUART_BAUD) & ~IMX_LPUART_BAUD_M10_MASK;
    if (dev->tty.c_cflag & CSTOPB) {
        /* 2 stop bits */
        baudReg |= IMX_LPUART_BAUD_SBNS_MASK;
    } else {
        /* 1 stop bit */
        baudReg &= ~IMX_LPUART_BAUD_SBNS_MASK;
    }
    out32(base + IMX_LPUART_BAUD, baudReg);

    /* Parity */
    ctrlReg &= ~(IMX_LPUART_CTRL_PE_MASK | IMX_LPUART_CTRL_PT_MASK);
    if (dev->tty.c_cflag & PARENB) {
        /* Enable parity */
        ctrlReg |= IMX_LPUART_CTRL_PE_MASK;
        if (dev->tty.c_cflag & PARODD) {
            /* Set odd parity */
            ctrlReg |= IMX_LPUART_CTRL_PT_MASK;
        }
        /* Set number of data bits */
        switch (dev->tty.c_cflag & CSIZE) {
            case CS8:
                /* Set 8-bits data width + parity bit (9-bits in total)*/
                ctrlReg &= ~IMX_LPUART_CTRL_M7_MASK;
                ctrlReg |= IMX_LPUART_CTRL_M_MASK;
                break;
            case CS7:
                /* Set 7-bits data width + parity bit (8-bits in total)*/
                ctrlReg &= ~IMX_LPUART_CTRL_M7_MASK;
                ctrlReg &= ~IMX_LPUART_CTRL_M_MASK;
                break;
        }
    }

    /* Set LPUART control register */
    out32(base + IMX_LPUART_CTRL, ctrlReg);

    if (dev->usedma) {
        /* Enable RX DMA and TX DMA */
        baudReg = in32(dev->base + IMX_LPUART_BAUD);
        baudReg |= (IMX_LPUART_BAUD_RDMAE_MASK | IMX_LPUART_BAUD_TDMAE_MASK | IMX_LPUART_BAUD_RIDMAE_MASK);
        out32(dev->base + IMX_LPUART_BAUD, baudReg);
    } else {
        /* Enable Receiver Ready Interrupt */
        ctrlReg = in32(base + IMX_LPUART_CTRL);
        out32(base + IMX_LPUART_CTRL, (ctrlReg | IMX_LPUART_CTRL_RIE_MASK));
    }

    /* Enable Transmitter and Receiver */
    dev->ctrl = ctrlReg = in32(base + IMX_LPUART_CTRL);
    out32(base + IMX_LPUART_CTRL, ctrlReg | (IMX_LPUART_CTRL_RE_MASK | IMX_LPUART_CTRL_TE_MASK));
}


/**
 * Check whether a device has drained.
 *
 * @param ttydev Pointer to a tty device structure.
 * @param count
 *
 * @return Execution status
 * @retval 1 Device has drained.
 * @retval 0 Device has not drained.
 */
int drain_check (TTYDEV * ttydev, uintptr_t * count)
{
    TTYBUF *bup = &ttydev->obuf;
    DEV_UART *dev = (DEV_UART *) ttydev;
    uint32_t drain_size = 0;

    if (dev->usedma) {

#if defined(USE_DMA)
        /* if the device has DRAINED, return 1 */
        if ((bup->cnt == 0) &&
            !dev->tx_xfer_active &&
            (in32(dev->base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TC_MASK) &&
            (in32(dev->base + IMX_LPUART_FIFO) & IMX_LPUART_FIFO_TXEMPT_MASK)) {
            return 1;
        }
        /* Set drain_size */
        drain_size = DMA_XFER_SIZE + FIFO_SIZE;
#endif
    }
    else {
        /* If the device has DRAINED, return 1 */
        if ((bup->cnt == 0) &&
            (in32(dev->base + IMX_LPUART_STAT) & IMX_LPUART_STAT_TC_MASK) &&
            (in32(dev->base + IMX_LPUART_FIFO) & IMX_LPUART_FIFO_TXEMPT_MASK)) {
            return 1;
        }
        /* Set drain_size */
        drain_size = FIFO_SIZE;
    }


    /* If the device has not DRAINED, set a timer based on 50ms counts
     * wait for the time it takes for one character to be transmitted
     * out the shift register.  We do this dynamically since the
     * baud rate can change.
     */
    if (count != NULL) {
        *count = (ttydev->baud == 0) ? 0 : ((IO_CHAR_DEFAULT_BITSIZE * drain_size) / ttydev->baud) + 1;
    }
    return 0;
}


/** @} */ /* end of devc */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/devc/sermx8/tto.c $ $Rev: 905321 $")
#endif
