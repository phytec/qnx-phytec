/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
 * Copyright 2016, Freescale Semiconductor, Inc.
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

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <errno.h>
#include <malloc.h>
#include <string.h>
#include <devctl.h>
#include <atomic.h>
#include <unistd.h>
#include <gulliver.h>
#include <sys/mman.h>
#include <hw/inout.h>

#include "canmx7x.h"
#include "proto.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

// Macro to get the register bit associated with the mailbox index
#define MAILBOX(mbxid)           (1 << mbxid)

// Set default CAN message ID's in a range that is valid for both standard and extended ID's
#define CANMID_DEFAULT           0x00100000
// Define Interrupts to Enable
#define FLEXCAN_GIM_INTR         (IMX7_FLEXCAN_MCR_WAKEMSK | IMX7_FLEXCAN_CTRL1_BOFFMSK | \
                                  IMX7_FLEXCAN_CTRL1_ERRMASK | IMX7_FLEXCAN_CTRL1_TWRNMSK | \
                                  IMX7_FLEXCAN_CTRL1_RWRNMSK)

#define CAN_PRESDIV_50K_XTAL     0x22

//#define DEBUG_DRVR

/* Function prototypes */
void can_tx(CANDEV_FLEXCAN *cdev, canmsg_t *txmsg);
void can_debug(CANDEV_FLEXCAN *dev);
void can_print_mailbox(CANDEV_FLEXCAN_INFO *devinfo);
void can_print_reg(CANDEV_FLEXCAN_INFO *devinfo);

/**
 * Function to modify a register.
 *
 * @param port Register address.
 * @param mask Clear mask.
 * @param data Data to be written into a register.
 */
void set_port32(uintptr_t port, uint32_t mask, uint32_t data)
{
    out32(port, (in32(port) & ~mask) | (data & mask));
}

/**
 * CAN interrupt handler.
 *
 * @param area Pointer to a CAN device info.
 * @param id   Mailbox ID.
 *
 * @return Pointer to an event structure.
 */
static const struct sigevent *can_intr(void *area, __attribute__((unused)) int id)
{
    struct sigevent         *event = NULL;
    CANDEV_FLEXCAN_INFO     *devinfo = area;
    CANDEV_FLEXCAN          *devlist = devinfo->devlist;
    uint32_t                estat, irqsrc1, irqsrc2, ctrl, canmcf_ide = 0;
    uint32_t                iflag_rx = 0, iflag_tx = 0, oflag_rx = 0;
    uint32_t                tx_irq_reg = 0, oflag_tx = 0;
    canmsg_t                *txmsg = NULL;
    canmsg_t                *rxmsg = NULL;
    uint32_t                mbxid = 0; /* mailbox index */
    uint32_t                *val32;
    unsigned int                     i;
    static int              erroractive = 1;

    devinfo->stats.total_interrupts++;

    // Check for System and Error Interrupts - log the error and clear
    // the interrupt source.
    if ((estat = in32(devinfo->base + IMX7_FLEXCAN_ESR1))) {
        /* One of the following can actually occur:
         -> BusOff.
         -> ErrInt - in this case indicates that at least one of the Error Bits (bits 15-10) is set.
         -> FCS - Fault Confinement State tells us
         00 - all ok, error active mode
         01 - error passive mode
         1x - Bus off
         -> RX warning level reached
         -> Tx warning level reached
         -> WakeUp Interrupt

         Most bits in CANESR are read only, except TWRN_INT, RWRN_INT, BOFF_INT, WAK_INT and
         ERR_INT, which are interrupt flags that can be cleared by writing 1 to them.

         If we have an error interrupt, reset all error conditions, we have saved them in estat
         for processing later on, so that we can move this error handling at the end of the ISR
         to have better RX response times.

         BusOff disables the Interrupt generation itself by resetting
         the mask in CANCTRL.

         We do reset all possible interrupts and look what we got later.
         */
        // Tx Warning Level Interrupt Flag
        if (estat & IMX7_FLEXCAN_ESR1_TWRNINT) {
            devinfo->stats.error_warning_state_count++;
            // Store error to be retrieved by devctl
            atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_TWRNINT);
            // Clear interrupt source
            out32(devinfo->base + IMX7_FLEXCAN_ESR1, IMX7_FLEXCAN_ESR1_TWRNINT);
        }
        // Rx Warning Level Interrupt Flag
        if (estat & IMX7_FLEXCAN_ESR1_RWRNINT) {
            devinfo->stats.error_warning_state_count++;
            // Store error to be retrieved by devctl
            atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_RWRNINT);
            // Clear interrupt source
            out32(devinfo->base + IMX7_FLEXCAN_ESR1, IMX7_FLEXCAN_ESR1_RWRNINT);
        }
        // Error Interrupt Flag
        if (estat & IMX7_FLEXCAN_ESR1_ERRINT) {
            // Store error to be retrieved by devctl
            atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_ERRINT);
            if (estat & IMX7_FLEXCAN_ESR1_BITERR_RECESSIVE_DOMINANT) {
                // Store error to be retrieved by devctl
                atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_BITERR_RECESSIVE_DOMINANT);
                devinfo->stats.recess_bit_dom_errors++;
                devinfo->stats.total_frame_errors++;
            }
            if (estat & IMX7_FLEXCAN_ESR1_BITERR_DOMINANT_RECESSIVE) {
                // Store error to be retrieved by devctl
                atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_BITERR_DOMINANT_RECESSIVE);
                devinfo->stats.dom_bit_recess_errors++;
                devinfo->stats.total_frame_errors++;
            }
            if (estat & IMX7_FLEXCAN_ESR1_ACKERR) {
                // Store error to be retrieved by devctl
                atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_ACKERR);
                devinfo->stats.missing_ack++;
                devinfo->stats.total_frame_errors++;
            }
            if (estat & IMX7_FLEXCAN_ESR1_CRCERR) {
                // Store error to be retrieved by devctl
                atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_CRCERR);
                devinfo->stats.crc_errors++;
                devinfo->stats.total_frame_errors++;
            }
            if (estat & IMX7_FLEXCAN_ESR1_FORMERR) {
                // Store error to be retrieved by devctl
                atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_FORMERR);
                devinfo->stats.form_errors++;
                devinfo->stats.total_frame_errors++;
            }
            if (estat & IMX7_FLEXCAN_ESR1_STUFFERR) {
                // Store error to be retrieved by devctl
                atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_STUFFERR);
                devinfo->stats.stuff_errors++;
                devinfo->stats.total_frame_errors++;
            }
            // Clear interrupt source
            out32(devinfo->base + IMX7_FLEXCAN_ESR1, IMX7_FLEXCAN_ESR1_ERRINT);
        }
        // Bus-Off Interrupt Flag
        if (estat & IMX7_FLEXCAN_ESR1_BOFFINT) {
            // Store error to be retrieved by devctl
            atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_BOFFINT);
            devinfo->stats.bus_off_state_count++;
            devinfo->stats.total_frame_errors++;

            // Clear interrupt source
            out32(devinfo->base + IMX7_FLEXCAN_ESR1, IMX7_FLEXCAN_ESR1_BOFFINT);
        }
        // Wakeup-Up Interrupt Flag
        if (estat & IMX7_FLEXCAN_ESR1_WAKEINT) {
            // Store error to be retrieved by devctl
            atomic_set(&devinfo->canestat, IMX7_FLEXCAN_ESR1_WAKEINT);
            devinfo->stats.wake_up_count++;
            // Clear interrupt source
            out32(devinfo->base + IMX7_FLEXCAN_ESR1, IMX7_FLEXCAN_ESR1_WAKEINT);
        }
        /* FCS - the Fault Confinement State
         *   if Bit4 (IMX7_FLEXCAN_ESR1_FCS_ERROR_PASSIVE) is set :  Error Passive
         *   else Error Active.
         */
        /* Going back to Error Active can happen without an error Int ? */
        if ((estat & IMX7_FLEXCAN_ESR1_FCS_MASK)== 0){
            if (!erroractive) {
                /* Going error active */
            }
            erroractive = 1;
        }

        if ((estat & IMX7_FLEXCAN_ESR1_FCS_MASK)== IMX7_FLEXCAN_ESR1_FCS_ERROR_PASSIVE) {
            if (erroractive) {
                /* Going error passive */
                devinfo->stats.error_passive_state_count++;
            }
            erroractive = 0;
        }

        // Clear Out All CANESR Interrupts
        out32(devinfo->base + IMX7_FLEXCAN_ESR1, in32(devinfo->base + IMX7_FLEXCAN_ESR1) | 0xffffffff);
    }

    /*
     * Loop as long as the CAN controller shows interrupts.
     * Check for message object interrupts.
     */
    // Check for Mailbox Interrupts
    while (1) {
        irqsrc1 = in32(devinfo->base + IMX7_FLEXCAN_IFLAG1);
        irqsrc2 = in32(devinfo->base + IMX7_FLEXCAN_IFLAG2);
        if ((irqsrc1 == 0) && (irqsrc2 == 0)) {
            break;
        }

        if (devinfo->mode == CANDEV_MODE_RAW_FRAME) {
            // Get the id of the mailbox that generated the interrupt
            iflag_rx = irqsrc1 & RAW_MODE_RX_IRQ;
            iflag_tx = irqsrc1 & RAW_MODE_TX_IRQ;
            oflag_tx = iflag_tx;
            iflag_tx = iflag_tx >> devinfo->numrx;
            tx_irq_reg = IMX7_FLEXCAN_IFLAG1;
        } else {
            // Get the id of the mailbox that generated the interrupt
            iflag_rx = irqsrc1;
            iflag_tx = irqsrc2;
            oflag_tx = iflag_tx;
            tx_irq_reg = IMX7_FLEXCAN_IFLAG2;
        }
        // Determine if it is a transmit or receive interrupt
        if (iflag_rx > 0) {
            oflag_rx = iflag_rx;
            for (i = 0; (i < devinfo->numrx) && (iflag_rx > 0); i++) {
                if (iflag_rx & 1) {
                    mbxid = i;
                    break;
                } else
                    iflag_rx = iflag_rx >> 1;
            }

            // Get the next free receive message or overwrite the oldest
            rxmsg = canmsg_dequeue_element(devlist[mbxid].cdev.free_queue);
            // if no queue elements are free, re-use the oldest rx message
            if (!rxmsg) {
                rxmsg = canmsg_dequeue_element(devlist[mbxid].cdev.msg_queue);
                devinfo->stats.sw_receive_q_full++;
                if (!rxmsg) {
                    /* Break from the while loop, both queues are full */
                    break;
                }
            }
            /*
             * Reading the canmcf control status word of the message mbx
             * buffer triggers a lock for that buffer.  It stays locked until
             * the free running timer is read, preventing corruption issues
             * due to a new message filling the mailbox.  A shadow register
             * takes care of preserving new messages.
             */

            /*
             * Make sure that the mailbox is not busy - CPU access is not permitted while busy.
             */
            while (((ctrl = devinfo->canmsg[mbxid].canmcf) & (REC_CODE_BUSY << 24)) != 0) {}
            /* Retrieve the message length from the DLC field */
            rxmsg->cmsg.len = (ctrl & IMX7_FLEXCAN_MSG_BUF_DLC_MASK) >> IMX7_FLEXCAN_MSG_BUF_DLC_SHIFT;

            /* Save the message ID */
            rxmsg->cmsg.mid = devinfo->canmsg[mbxid].canmid;
            /* Save message timestamp */
            rxmsg->cmsg.ext.timestamp = ctrl & 0x0000FFFF;
            canmcf_ide = ctrl & IMX7_FLEXCAN_MB_CNT_IDE;

            rxmsg->cmsg.ext.is_extended_mid = canmcf_ide > 0;

            /*
             *  Access the data as a uint32_t array for endian conversion
             *  and copy data from receive mailbox to receive message
             */
            val32 = (uint32_t *)rxmsg->cmsg.dat;
            val32[0] = devinfo->canmsg[mbxid].canmdl;
            val32[1] = devinfo->canmsg[mbxid].canmdh;

            if (devinfo->iflags & INFO_FLAGS_ENDIAN_SWAP) {
                // Convert from Big Endian to Little Endian since data is received MSB
                ENDIAN_SWAP32(&val32[0]);
                ENDIAN_SWAP32(&val32[1]);
            }

            /* Set mailbox to empty */
            devinfo->canmsg[mbxid].canmcf = ((devinfo->canmsg[mbxid].canmcf & ~MSG_BUF_CODE_MASK)
                    | (REC_CODE_EMPTY << MSG_BUF_CODE_SHIFT) | canmcf_ide);

            /* Unlock the message buffer by reading the CAN free running timer */
            devinfo->timer = in32(devinfo->base + IMX7_FLEXCAN_TIMER);

            /* Clear receive mailbox interrupts by writing a 1 to the interrupt source */
            out32(devinfo->base + IMX7_FLEXCAN_IFLAG1, oflag_rx);

            // Add populated element to the receive queue
            canmsg_queue_element(devlist[mbxid].cdev.msg_queue, rxmsg);
            devinfo->stats.received_frames++;

            /* Check for one or more blocked clients and return event to
             event handler */
            if (devlist[mbxid].cdev.wait_client_queue->cnt) {
                event = &devlist[mbxid].cdev.event;
            }

            if (event) {
                return event;
            }
        }

        if (iflag_tx > 0) {
            out32(devinfo->base + tx_irq_reg, oflag_tx);
            for (i = devinfo->numrx; (i < (devinfo->numrx + devinfo->numtx)) && (iflag_tx > 0); i++) {
                if (iflag_tx & 1) {
                    mbxid = i;
                    break;
                } else {
                    iflag_tx = iflag_tx >> 1;
                }
            }
            // Transmit mailbox processing
            devinfo->stats.transmitted_frames++;

            // Determine if there is another message to transmit
            txmsg = canmsg_dequeue_element(devlist[mbxid].cdev.msg_queue);
            if (txmsg) {
                // Transmit next message
                can_tx(&devlist[mbxid], txmsg);
            } else {
                // No more transmit messages, end transmission
                atomic_clr(&devlist[mbxid].dflags, CANDEV_TX_ENABLED);
            }

            if (devlist[mbxid].cdev.wait_client_queue->cnt) {
                event = &devlist[mbxid].cdev.event;
            }
            if (event) {
                return event;
            }
        }
        // Clear Mailbox Interrupt Flag
        out32(devinfo->base + IMX7_FLEXCAN_IFLAG1, irqsrc1);
        out32(devinfo->base + IMX7_FLEXCAN_IFLAG2, irqsrc2);
    }

    return event;
}

/**
 * LIBCAN driver transmit function.
 *
 * @param cdev Pointer to a CAN device.
 *
 * @return none
 */
void can_drvr_transmit(CANDEV *cdev)
{
    CANDEV_FLEXCAN *dev = (CANDEV_FLEXCAN *)cdev;
    canmsg_t *txmsg;

    // Make sure transmit isn't already in progress and there is valid data
    if (!(dev->dflags & CANDEV_TX_ENABLED) && (txmsg = canmsg_dequeue_element(cdev->msg_queue))) {
        // Indicate transmit is in progress
        atomic_set(&dev->dflags, CANDEV_TX_ENABLED);
        // Start transmission
        can_tx(dev, txmsg);
        // tx message is copied - return element to free queue
        canmsg_queue_element(cdev->free_queue, txmsg);
    }
}

/**
 * LIBCAN driver devctl function.
 *
 * @param cdev Pointer to a CAN device.
 * @param dcmd CAN command.
 * @param data Pointer to a CAN command data.

 * @retval EOK In case of success.
 * @retval EINVAL Invalid argument.
 * @retval EXIT_FAILURE FlexCAN Freeze Mode failed.
 * @retval ENOTSUP Command not supported.
 */
int can_drvr_devctl(CANDEV *cdev, int dcmd, DCMD_DATA *data)
{
    CANDEV_FLEXCAN *dev = (CANDEV_FLEXCAN *)cdev;
    CANDEV_FLEXCAN_INFO *devinfo = dev->devinfo;
    int mbxid = dev->mbxid;
    int timeout = 20000, i;
    uint32_t cantest = 0, offset = 0;
    uint32_t *canmid = &dev->devinfo->canmsg[mbxid].canmid;
    uint32_t *canmcf = &dev->devinfo->canmsg[mbxid].canmcf;

    switch (dcmd) {
        case CAN_DEVCTL_SET_MID:
            if (devinfo->mode == CANDEV_MODE_RAW_FRAME) {
                return (EINVAL);
            }
            // Disable Object
            if (mbxid < (int)devinfo->numrx) {
                set_port32(devinfo->base + IMX7_FLEXCAN_IMASK1, MAILBOX(mbxid), 0);
            } else {
                set_port32(devinfo->base + IMX7_FLEXCAN_IMASK2, MAILBOX(mbxid), 0);
            }
            // Set new message ID
            *canmid = (*canmid & ~FLEXCAN_MID_MASK_EXT) | data->mid;
            // Enable Object
            if (mbxid < (int)devinfo->numrx) {
                set_port32(devinfo->base + IMX7_FLEXCAN_IMASK1, MAILBOX(mbxid), MAILBOX(mbxid));
            } else {
                set_port32(devinfo->base + IMX7_FLEXCAN_IMASK2, MAILBOX(mbxid), MAILBOX(mbxid));
            }
            break;

        case CAN_DEVCTL_GET_MID:
            // Read device's current message ID (removing non-message ID bits)
            data->mid = *canmid & FLEXCAN_MID_MASK_EXT;
            break;

        case CAN_DEVCTL_SET_MFILTER:
            // Make sure this is a receive mailbox, and we're not in raw mode.
            if ((!(mbxid < (int)devinfo->numrx)) || (devinfo->mode == CANDEV_MODE_RAW_FRAME)) {
                return (EINVAL);
            }

            /*
             * Rx Individual Mask Registers are used as acceptance masks for ID filtering
             * in Rx message buffers. They can only be accessed by the ARM while the
             * module is in freeze mode. Outside of freeze mode, write accesses are
             * blocked and read accesses return all zeros.
             */
            // Enable Freeze Mode capability
            set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_FRZ, IMX7_FLEXCAN_MCR_FRZ);
            // Enter Freeze Mode
            set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_HALT, IMX7_FLEXCAN_MCR_HALT);
            // Wait for indication that FlexCAN in Freeze Mode
            while ((in32(devinfo->base + IMX7_FLEXCAN_MCR) & IMX7_FLEXCAN_MCR_FRZACK) != IMX7_FLEXCAN_MCR_FRZACK) {
                if (timeout-- == 0) {
                    perror("FlexCAN Freeze Mode failed: FRZ_ACK timeout!\n");
                    exit(EXIT_FAILURE);
                }
            }

            // Disable mailbox events
            set_port32(devinfo->base + IMX7_FLEXCAN_IMASK1, MAILBOX(mbxid), 0);

            /*
             * Enable local acceptance mask if the filter value is non-zero
             * Set receive filter by programming local acceptance mask (CANLAM).
             * Every bit set in LAM mask means that the corresponding bit in the CANMID is checked
             */
            offset = mbxid * 0x4;
            out32((devinfo->canlam + offset), (data->mfilter & IMX7_FLEXCAN_LAM_MASK));
//            fprintf(stderr, "CANIMR%d = 0x%02x\n", mbxid, in32(devinfo->canlam + offset));
            // Re-enable mailbox events
            set_port32(devinfo->base + IMX7_FLEXCAN_IMASK1, MAILBOX(mbxid), MAILBOX(mbxid));

            // Take FlexCAN out of freeze mode
            set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_HALT, 0);

            for (i = 0; i < IMX7_FLEXCAN_SET_MODE_RETRIES; i++) {
                cantest = in32(devinfo->base + IMX7_FLEXCAN_MCR);
                if (!(cantest & (IMX7_FLEXCAN_MCR_NOTRDY | IMX7_FLEXCAN_MCR_FRZACK))) {
                    break;
                }
            }
            break;

        case CAN_DEVCTL_GET_MFILTER:

            // Make sure this is a receive mailbox
            if (!(mbxid < (int)devinfo->numrx)) {
                return (EINVAL);
            }
            /*
             * The FLEXCAN must be in Freeze mode for the filter registers to be accessed.
             * Halt FlexCAN and wait for freeze acknowledge (pending TXs and RXs done
             */

            // Enable Freeze Mode capability
            set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_FRZ, IMX7_FLEXCAN_MCR_FRZ);
            // Enter Freeze Mode
            set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_HALT, IMX7_FLEXCAN_MCR_HALT);

            // Wait for indication that FlexCAN is in Freeze Mode
            while ((in32(devinfo->base + IMX7_FLEXCAN_MCR) & IMX7_FLEXCAN_MCR_FRZACK) != IMX7_FLEXCAN_MCR_FRZACK) {
                if (timeout-- == 0) {
                    perror("FlexCAN Freeze Mode failed: FRZ_ACK timeout!\n");
                    exit(EXIT_FAILURE);
                }
            }

            offset = mbxid * 0x4;

            // Read device's current message ID (removing non-message ID bits)
            data->mfilter = in32(devinfo->canlam + offset);

            // Take FlexCAN out of freeze mode and continue
            set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_HALT, 0);

            for (i = 0; i < IMX7_FLEXCAN_SET_MODE_RETRIES; i++) {
                cantest = in32(devinfo->base + IMX7_FLEXCAN_MCR);
                if (!(cantest & (IMX7_FLEXCAN_MCR_NOTRDY | IMX7_FLEXCAN_MCR_FRZACK))) {
                    break;
                }
            }
            break;

        case CAN_DEVCTL_SET_PRIO:
            // Make sure this is a transmit mailbox and priority is valid,
            // and we're not in raw mode
            if ((mbxid < (int)devinfo->numrx) || (data->prio > FLEXCAN_MCF_TPL_MAXVAL)
                 || (devinfo->mode == CANDEV_MODE_RAW_FRAME)) {
                return (EINVAL);
            }

            // Disable Object
            if (mbxid < (int)devinfo->numrx) {
                set_port32(devinfo->base + IMX7_FLEXCAN_IMASK1, MAILBOX(mbxid), 0);
            } else {
                set_port32(devinfo->base + IMX7_FLEXCAN_IMASK2, MAILBOX(mbxid), 0);
            }
            // Set new priority
            *canmcf = (*canmcf & ~FLEXCAN_MCF_TPL_MASK) | (data->prio << FLEXCAN_MCF_TPL_SHIFT);
            // Enable Object
            if (mbxid < (int)devinfo->numrx) {
                set_port32(devinfo->base + IMX7_FLEXCAN_IMASK1, MAILBOX(mbxid), MAILBOX(mbxid));
            } else {
                set_port32(devinfo->base + IMX7_FLEXCAN_IMASK2, MAILBOX(mbxid), MAILBOX(mbxid));
            }
            break;

        case CAN_DEVCTL_GET_PRIO:
            // Make sure this is a transmit mailbox
            if (mbxid < (int)devinfo->numrx) {
                return (EINVAL);
            }

            // Read device's TX prio level in CANMCF
            data->prio = (*canmcf & FLEXCAN_MCF_TPL_MASK) >> FLEXCAN_MCF_TPL_SHIFT;
            break;

        case CAN_DEVCTL_SET_TIMESTAMP:
            // Check if we should set the Local Network Time or clear the MSB bit.
            // We use the max timestamp value since no-one will likely set this value.
            if (!(data->timestamp == 0xFFFF)) {
                // Set the current Local Network Time
                out32(devinfo->base + IMX7_FLEXCAN_TIMER, data->timestamp);
            }
            break;

        case CAN_DEVCTL_GET_TIMESTAMP:
            // Read the current Local Network Time
            data->timestamp = in32(devinfo->base + IMX7_FLEXCAN_TIMER);
            break;

        case CAN_DEVCTL_READ_CANMSG_EXT:
            // This is handled by the interrupt handler
            break;

        case CAN_DEVCTL_RX_FRAME_RAW_NOBLOCK:
            /* No break */
        case CAN_DEVCTL_RX_FRAME_RAW_BLOCK:
            /* No break */
        case CAN_DEVCTL_TX_FRAME_RAW:
            /*  These devctls are handled by the interrupt handler -
             * check for raw mode
             */
            if (devinfo->mode != CANDEV_MODE_RAW_FRAME) {
                return (EINVAL);
            }
            break;
        case CAN_DEVCTL_ERROR:
            // Read current state of CAN Error and Status register
            data->error.drvr1 = in32(devinfo->base + IMX7_FLEXCAN_ESR1);
            // Read and clear CANES devctl info
            data->error.drvr2 = atomic_clr_value(&devinfo->canestat, 0xffffffff);
            // Read current state of CAN Receive Error Counter Register
            data->error.drvr3 = (in32(devinfo->base + IMX7_FLEXCAN_ECR) & 0xFF00);
            // Read current state of CAN Transmit Error Counter Register
            data->error.drvr4 = (in32(devinfo->base + IMX7_FLEXCAN_ECR) & 0x00FF);
            break;

        case CAN_DEVCTL_DEBUG_INFO:
            // Print debug information
            can_debug(dev);
            break;
        case CAN_DEVCTL_GET_STATS:
            // Copy statistics counters to client structure
            memcpy(&data->stats, &devinfo->stats, sizeof(CAN_DEVCTL_STATS));
            break;

        case CAN_DEVCTL_GET_INFO:
            // Copy statistics counters to client structure
            strcpy(data->info.description, devinfo->initinfo.description);
            data->info.msgq_size = devinfo->initinfo.msgq_size;
            data->info.waitq_size = devinfo->initinfo.waitq_size;
            data->info.mode = devinfo->mode;
            data->info.bit_rate = devinfo->initinfo.bitrate;
            data->info.bit_rate_prescaler = devinfo->initinfo.br_brp + 1;
            data->info.sync_jump_width = devinfo->initinfo.br_rjw + 1;
            data->info.time_segment_1 = devinfo->initinfo.br_pseg1 + 1;
            data->info.time_segment_2 = devinfo->initinfo.br_pseg2 + 1;
            data->info.num_tx_mboxes = devinfo->numtx;
            data->info.num_rx_mboxes = devinfo->numrx;
            data->info.loopback_internal = (devinfo->iflags & INIT_FLAGS_LOOPBACK) ? 1 : 0;
            data->info.autobus_on = (devinfo->iflags & INIT_FLAGS_AUTOBUS) ? 1 : 0;
            data->info.silent = 0;
            break;
        default:
            // Driver does not support this DEVCTL
            return (ENOTSUP);
    }

    return (EOK);
}

/**
 * Initialize CAN device registers
 *
 * @param devinfo      Pointer to a device info structure.
 * @param devinit      Pointer to a device init structure.
 * @param mdriver_intr Mini_driver active status flag.
 */
void can_init_intr(CANDEV_FLEXCAN_INFO *devinfo, CANDEV_FLEXCAN_INIT *devinit,
        int mdriver_intr)
{
    // Attach interrupt handler for system interrupts
    devinfo->iidsys = InterruptAttach(devinit->irqsys, can_intr, devinfo, 0,
                                      _NTO_INTR_FLAGS_TRK_MSK | _NTO_INTR_FLAGS_END);
    if (devinfo->iidsys == -1) {
        perror("InterruptAttach irqsys");
        exit(EXIT_FAILURE);
    }

    /* Interrupts on Rx, Tx, any Status change and data overrun */
    if (devinfo->mode == CANDEV_MODE_RAW_FRAME) {
        // Enable raw mailbox interrupts (MB0 to MB1)
        out32(devinfo->base + IMX7_FLEXCAN_IMASK1, (RAW_MODE_TX_IRQ | RAW_MODE_RX_IRQ));
    } else {
        // Enable all mailbox interrupts (MB0 to MB63)
        out32(devinfo->base + IMX7_FLEXCAN_IMASK1, 0xFFFFFFFF);
        out32(devinfo->base + IMX7_FLEXCAN_IMASK2, 0xFFFFFFFF);
    }

    // Add mini-driver's bufferred CAN messages if mini-driver is active.
    if ((devinit->flags & INIT_FLAGS_MDRIVER_INIT) && (mdriver_intr != -1)) {
        mdriver_init_data(devinfo, devinit);
    }

    // Enable all mailbox interrupts (MB0 to MB63)
    out32(devinfo->base + IMX7_FLEXCAN_IMASK1, 0xFFFFFFFF);
    out32(devinfo->base + IMX7_FLEXCAN_IMASK2, 0xFFFFFFFF);

    // Enable all system/error interrupts to be generated on interrupt line
    // Note - must do this AFTER calling InterruptAttach since mini-driver clears interrupts
    // on the MDRIVER_INTR_ATTACH state.
    // Wake Up Interrupt is enabled
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_WAKEMSK, IMX7_FLEXCAN_MCR_WAKEMSK);
    // Bus Off interrupt enabled
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_BOFFMSK, IMX7_FLEXCAN_CTRL1_BOFFMSK);
    // Error interrupt enabled
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_ERRMASK, IMX7_FLEXCAN_CTRL1_ERRMASK);
    // Tx Warning Interrupt enabled
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_TWRNMSK, IMX7_FLEXCAN_CTRL1_TWRNMSK);
    // Rx Warning Interrupt enabled
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_RWRNMSK, IMX7_FLEXCAN_CTRL1_RWRNMSK);
}

/**
 *  Initialize CAN device registers.
 *
 * @param devinfo Pointer to a device info structure.
 * @param devinit Pointer to a device init structure.
 */
void can_init_hw(CANDEV_FLEXCAN_INFO *devinfo, CANDEV_FLEXCAN_INIT *devinit)
{
    int timeout = 20000, counter = 0;
    unsigned int i;
    uint32_t canmcf_ide = 0, cantest = 0;
    uint32_t canlam = 0;

    /* Go to INIT mode:
     * Any configuration change/initialization requires that the FlexCAN be
     * frozen by either asserting the HALT bit in the
     * module configuration register or by reset.
     */

    // Reset Device
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_SOFTRST, IMX7_FLEXCAN_MCR_SOFTRST);
    /* Since soft reset is synchronous and has to follow a request/acknowledge procedure
     * across clock domains, it may take some time to fully propagate its effect.
     */
    while ((in32(devinfo->base + IMX7_FLEXCAN_MCR) & IMX7_FLEXCAN_MCR_SOFTRST) != 0){
        if (timeout-- == 0) {
            perror("Enter Init Mode: SOFT_RST timeout!\n");
            exit(EXIT_FAILURE);
        }
    }
    // Give reset time
    nanospin_ns(1000);

    /* Go to Disable Mode:
     * The clock source (CLK_SRC bit) should be selected while the module is in Disable Mode.
     * After the clock source is selected and the module is enabled (MDIS bit negated),
     * FlexCAN automatically goes to Freeze Mode.
     */
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_MDIS, IMX7_FLEXCAN_MCR_MDIS);
    // Wait for indication that FlexCAN is in Disable Mode
    while ((in32(devinfo->base + IMX7_FLEXCAN_MCR) & IMX7_FLEXCAN_MCR_LPM_ACK) != IMX7_FLEXCAN_MCR_LPM_ACK) {
        if (timeout-- == 0) {
            perror("Low Power Mode: LPM_ACK timeout!\n");
            exit(EXIT_FAILURE);
        }
    }
    // For some SOC's such as the i.mx7x chips we need to explicitly enable the FlexCAN at this point since Freeze Mode is NOT
    // always automatically enabled.
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_MDIS, 0);
    timeout = 20000;
    while ((in32(devinfo->base + IMX7_FLEXCAN_MCR) & IMX7_FLEXCAN_MCR_FRZACK) != IMX7_FLEXCAN_MCR_FRZACK) {
        if (timeout-- == 0) {
            perror("Unable to enter Freeze Mode\n");
            exit(EXIT_FAILURE);
        }
    }
    /* Determine the bit timing parameters: PROPSEG, PSEG1, PSEG2, RJW.
     * Determine the bit rate by programming the PRESDIV field.
     * The prescaler divide register (PRESDIV) allows the user to select
     * the ratio used to derive the S-Clock from the system clock.
     * The time quanta clock operates at the S-clock frequency.
     */
    out32(devinfo->base + IMX7_FLEXCAN_CTRL1,
          (devinit->br_presdiv << IMX7_FLEXCAN_CTRL1_PRESDIV_SHIFT)
          | (devinit->br_propseg << IMX7_FLEXCAN_CTRL1_PROPSEG_SHIFT)
          | (devinit->br_rjw << IMX7_FLEXCAN_CTRL1_RJW_SHIFT)
          | (devinit->br_pseg1 << IMX7_FLEXCAN_CTRL1_PSEG1_SHIFT)
          | (devinit->br_pseg2 << IMX7_FLEXCAN_CTRL1_PSEG2_SHIFT));

    // Select clock source to be IP bus clock by default
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_CLKSRC, IMX7_FLEXCAN_CTRL1_CLKSRC);
    // Select External reference if flag set
    if (devinit->flags & INIT_FLAGS_CLKSRC) {
        set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_CLKSRC, 0);
        set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_PRESDIV_MASK, CAN_PRESDIV_50K_XTAL);
    }

    /*
     * For any configuration change/initialization it is required that FlexCAN be put into Freeze Mode.
     * The following is a generic initialization sequence applicable to the FlexCAN module:
     */

    /* 1. Initialize the Module Configuration Register */
    // Affected registers are in Supervisor memory space
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_SUPV, IMX7_FLEXCAN_MCR_SUPV);
    // Enable the individual filtering per MB and reception queue features by setting the IRMQ bit
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_IRMQ, IMX7_FLEXCAN_MCR_IRMQ);
    // Enable the warning interrupts by setting the WRN_EN bit
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_WRN_EN, IMX7_FLEXCAN_MCR_WRN_EN);
    // If required, disable frame self reception by setting the SRX_DIS bit
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_SRX_DIS, IMX7_FLEXCAN_MCR_SRX_DIS);
    // Enable the abort mechanism by setting the AEN bit
    //set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_AEN, IMX7_FLEXCAN_MCR_AEN);
    // Enable the local priority feature by setting the LPRIO_EN bit
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_LPRIO_EN, IMX7_FLEXCAN_MCR_LPRIO_EN);
    // Format A One full ID (standard or extended) per filter element
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_IDAM_FormatA, IMX7_FLEXCAN_MCR_IDAM_FormatA);
    // Maximum MBs in use
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_MAXMB_MASK, IMX7_FLEXCAN_MCR_MAXMB_MAXVAL);

    /* 2. Initialize the Control Register */
    // Disable Bus-Off Interrupt
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_BOFFMSK, 0);
    // Disable Error Interrupt
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_ERRMASK, 0);
    // Tx Warning Interrupt disabled
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_TWRNMSK, 0);
    // Rx Warning Interrupt disabled
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_RWRNMSK, 0);

    // Rx mailbox IDE and RTR are compared to filter
    if (devinfo->mode == CANDEV_MODE_RAW_FRAME) {
        set_port32(devinfo->base +IMX7_FLEXCAN_CTRL2, IMX7_FLEXCAN_CTRL2_EACEN, IMX7_FLEXCAN_CTRL2_EACEN);
        canlam = 0;
    } else {
        canlam = 0xffffffff;
    }

    // Single Sample Mode should be set by default
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_SAMP, 0);
    // Check if Triple Sample Mode should be set
    if ((devinit->flags & INIT_FLAGS_BITRATE_SAM) && (devinit->br_presdiv >= IMX7_FLEXCAN_CTRL1_PRESDIV_SAM_MIN)) {
        set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_SAMP, IMX7_FLEXCAN_CTRL1_SAMP);
    }
    // Disable self-test/loop-back by default
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_LPB, 0);
    // Enable self-test/loop-back for testing
    if (devinit->flags & INIT_FLAGS_LOOPBACK) {
        set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_SRX_DIS, 0);
        set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_LPB, IMX7_FLEXCAN_CTRL1_LPB);
    }
    // Disable Timer Sync feature by default
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_TSYNC, 0);
    // Enable Timer Sync feature
    if (devinit->flags & INIT_FLAGS_TSYN) {
        set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_TSYNC, IMX7_FLEXCAN_CTRL1_TSYNC);
    }

    /* Listen-Only Mode:
     * In listen-only mode, the CAN module is able to receive messages
     * without giving an acknowledgment.
     * Since the module does not influence the CAN bus in this mode
     * the host device is capable of functioning like a monitor
     * or for automatic bit-rate detection.
     */
    // De-activate Listen Only Mode by default
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_LOM, 0);
    // Select Bus monitor mode if flag set
    if (devinit->flags & INIT_FLAGS_LOM) {
        set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_LOM, IMX7_FLEXCAN_CTRL1_LOM);
    }
    // Enable Automatic recovering from Bus Off state
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_BOFFREC, 0);
    // Disable Auto bus on
    if (devinit->flags & INIT_FLAGS_AUTOBUS) {
        set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_BOFFREC, IMX7_FLEXCAN_CTRL1_BOFFREC);
    }

    /* Determine the internal arbitration mode (LBUF bit) */
    /* The LBUF bit defines the transmit-first scheme.
     * 0 = Message buffer with lowest ID is transmitted first.
     * 1 = Lowest numbered buffer is transmitted first.
     */
    // Buffer with highest priority is transmitted first
    set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_LBUF, 0);
    // Lowest number buffer is transmitted first
    if (devinit->flags & INIT_FLAGS_LBUF) {
        set_port32(devinfo->base + IMX7_FLEXCAN_CTRL1, IMX7_FLEXCAN_CTRL1_LBUF, IMX7_FLEXCAN_CTRL1_LBUF);
    }

    // Set initial value for Local Network Time
    if (devinit->flags & INIT_FLAGS_TIMESTAMP) {
        out32(devinfo->base + IMX7_FLEXCAN_TIMER, devinit->timestamp);
    }

    // Clear interrupts, error counters and set mask registers

    // Global registers instead of individual regsiters (Negate IRMQ)
    if ((in32(devinfo->base + IMX7_FLEXCAN_MCR) & IMX7_FLEXCAN_MCR_IRMQ)== 0) {
        out32(devinfo->base + IMX7_FLEXCAN_RXGMASK, 0xffffffff);
        out32(devinfo->base + IMX7_FLEXCAN_RX14MASK, 0xffffffff);
        out32(devinfo->base + IMX7_FLEXCAN_RX15MASK, 0xffffffff);
    }
    out32(devinfo->base + IMX7_FLEXCAN_ECR, 0);
    out32(devinfo->base + IMX7_FLEXCAN_ESR1, in32(devinfo->base + IMX7_FLEXCAN_ESR1) | 0xffffffff);
    // Disable Buffer Interrupt
    out32(devinfo->base + IMX7_FLEXCAN_IMASK2, 0);
    out32(devinfo->base + IMX7_FLEXCAN_IMASK1, 0);
    // Clear Interrupt Flag by writing it to 1
    out32(devinfo->base + IMX7_FLEXCAN_IFLAG2, 0xffffffff);
    out32(devinfo->base + IMX7_FLEXCAN_IFLAG1, 0xffffffff);

    /* Initialize CAN mailboxes in device memory */

    // Check if the 29 bit extended identifier should be enabled
    if (devinit->flags & INIT_FLAGS_EXTENDED_MID) {
        canmcf_ide |= FLEXCAN_MCF_IDE;
    }

    // Configure Receive Mailboxes
    counter = 0;
    for (i = 0; i < devinfo->numrx; i++) {
        // Disable mailbox to configure message object
        // The control/status word of all message buffers are written
        // as an inactive receive message buffer.
        devinfo->canmsg[i].canmcf = ((REC_CODE_NOT_ACTIVE & 0x0F) << 24);
        // Initialize default receive message ID
        if (canmcf_ide) {    // Extended frame
            devinfo->canmsg[i].canmid = (devinit->midrx + CANMID_DEFAULT * counter++)
                    & FLEXCAN_MID_MASK_EXT;
        } else {             // Standard frame
            devinfo->canmsg[i].canmid = (devinit->midrx + CANMID_DEFAULT * counter++)
                    & FLEXCAN_MID_MASK_STD;
        }
        devinfo->canmsg[i].canmdl = 0x0;
        devinfo->canmsg[i].canmdh = 0x0;
        // Put MB into rx queue
        devinfo->canmsg[i].canmcf = ((REC_CODE_EMPTY & 0x0F) << 24) | canmcf_ide;
    }

    // Configure Transmit Mailboxes
    counter = 0;
    for (i = devinfo->numrx; i < (devinfo->numrx + devinfo->numtx); i++) {
        // Disable mailbox to configure message object
        // Transmission inactive, Set message data size
        devinfo->canmsg[i].canmcf = ((TRANS_CODE_NOT_READY & 0x0F) << 24) | (8 << 16) | canmcf_ide;
        // Initialize default transmit message ID
        if (canmcf_ide) {    // Extended frame
            devinfo->canmsg[i].canmid = (devinit->midtx + CANMID_DEFAULT * counter++)
                                        & FLEXCAN_MID_MASK_EXT;
        } else {    // Standard frame
            devinfo->canmsg[i].canmid = (devinit->midtx + CANMID_DEFAULT * counter++)
                                        & FLEXCAN_MID_MASK_STD;
        }
        devinfo->canmsg[i].canmdl = 0x0;
        devinfo->canmsg[i].canmdh = 0x0;
    }

    // Enable the FlexCAN module
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_MDIS, 0);
    // Wait for indication that FlexCAN not in any of the low power modes
    timeout = 20000;
    while ((in32(devinfo->base + IMX7_FLEXCAN_MCR) & IMX7_FLEXCAN_MCR_LPM_ACK) != 0) {
        if (timeout-- == 0) {
            perror("Not able to come out of Low Power Mode: LPM_ACK timeout!\n");
            exit(EXIT_FAILURE);
        }
    }

    /* Rx Individual Mask Registers (RXIMR0-RXIMR63):
     * 1: The corresponding bit in the filter is checked against the one received
     * 0: the corresponding bit in the filter is don't care
     */
    i = 0;
    while (i < (IMX7_FLEXCAN_MAILBOX_SIZE * devinfo->numrx)) {
        out32(devinfo->canlam + i, canlam);
        i += IMX7_FLEXCAN_MAILBOX_SIZE;
    }

    // Synchronize with can bus
    set_port32(devinfo->base + IMX7_FLEXCAN_MCR, IMX7_FLEXCAN_MCR_HALT, 0);

    for (i = 0; i < IMX7_FLEXCAN_SET_MODE_RETRIES; i++) {
        cantest = in32(devinfo->base + IMX7_FLEXCAN_MCR);
        if (!(cantest & (IMX7_FLEXCAN_MCR_NOTRDY | IMX7_FLEXCAN_MCR_FRZACK))) {
            break;
        }
    }

#ifdef DEBUG_DRVR
    cantest = in32(devinfo->base + IMX7_FLEXCAN_MCR);
    if (!(cantest & IMX7_FLEXCAN_MCR_FRZACK)) {
        fprintf(stderr, "Out from Freezemode\n");
    }
#endif
}

/**
 * Transmit a CAN message from the specified mailbox.
 *
 * @param dev   Pointer to a CAN device structure.
 * @param txmsg Pointer to a CAN message structure.
 */
void can_tx(CANDEV_FLEXCAN *dev, canmsg_t *txmsg)
{
    CANDEV_FLEXCAN_INFO *devinfo = dev->devinfo;
    int mbxid = dev->mbxid;
    // Access the data as a uint32_t array
    uint32_t *val32 = (uint32_t *)txmsg->cmsg.dat;
    uint32_t can_mcf;

    /*
     * IDLE - Idle Status. The IDLE bit indicates, when there is activity
     * on the CAN bus
     * 1 - The CAN bus is Idle
     *
     * TX/RX - Transmission/receive status.
     * Indicates when the FlexCAN module is transmitting or receiving a message.
     * TX/RX has no meaning, when IDLE = 1
     * 0 - FlexCAN is receiving when IDLE = 0
     * 1 - FlexCAN is transmitting when IDLE = 0
     */
    // Wait till bus available
    while ((in32(devinfo->base + IMX7_FLEXCAN_ESR1) & IMX7_FLEXCAN_ESR1_IDLE)== 0) {}

    /*
     * Set mailbox to inactive while filling it.
     * Also set the msg length in the mcf to the actual message length
     */
    can_mcf = devinfo->canmsg[mbxid].canmcf;
    can_mcf &= ~(MSG_BUF_CODE_MASK | IMX7_FLEXCAN_MSG_BUF_DLC_MASK);
    can_mcf |= TRANS_CODE_NOT_READY << MSG_BUF_CODE_SHIFT;
    can_mcf |= txmsg->cmsg.len << IMX7_FLEXCAN_MSG_BUF_DLC_SHIFT;

    /*
     * if we are in raw mode, set the MCF_IDE bit if txmsg is an extended message
     * Otherwise just use the bit set when the mailbox was configured i
     */
    if (devinfo->mode == CANDEV_MODE_RAW_FRAME) {
        can_mcf &= ~FLEXCAN_MCF_IDE;
        can_mcf |= (txmsg->cmsg.ext.is_extended_mid) ? FLEXCAN_MCF_IDE : 0;
    }

    devinfo->canmsg[mbxid].canmcf = can_mcf;
    // Copy message data into transmit mailbox
    devinfo->canmsg[mbxid].canmdl = val32[0];
    devinfo->canmsg[mbxid].canmdh = val32[1];

    if (devinfo->iflags & INFO_FLAGS_ENDIAN_SWAP) {
        // Convert from Little Endian to Big Endian since data is transmitted MSB
        ENDIAN_SWAP32(&devinfo->canmsg[mbxid].canmdl);
        ENDIAN_SWAP32(&devinfo->canmsg[mbxid].canmdh);
    }

    /*
     * Set the MID if we are in raw mode.
     * Note that in normal mode, cmsg.mid is set to zero, so we have
     * to wrap this in a conditional
     */
    if (devinfo->mode == CANDEV_MODE_RAW_FRAME) {
        devinfo->canmsg[mbxid].canmid = txmsg->cmsg.mid;
    }

    // Transmission active
    devinfo->canmsg[mbxid].canmcf |= ((TRANS_CODE_TRANSMIT_ONCE & 0x0F) << 24);
}

/**
 * Print debug information.
 *
 * @param dev Pointer to a device structure.
 */
void can_debug(CANDEV_FLEXCAN *dev)
{
    fprintf(stderr, "\nCAN REG\n");
    can_print_reg(dev->devinfo);
    fprintf(stderr, "\nMailboxes\n");
    can_print_mailbox(dev->devinfo);
}

/**
 * Print CAN device registers.
 *
 * @param devinfo Pointer to a device info structure.
 */
void can_print_reg(CANDEV_FLEXCAN_INFO *devinfo)
{
    fprintf(stderr, "\n******************************************************\n");
    fprintf(stderr, "CANMCR = 0x%02x\t", in32(devinfo->base + IMX7_FLEXCAN_MCR));
    fprintf(stderr, "  CANCTRL = 0x%02x\n", in32(devinfo->base + IMX7_FLEXCAN_CTRL1));
    fprintf(stderr, "CANTIMER = 0x%02x\t", in32(devinfo->base + IMX7_FLEXCAN_TIMER));
    fprintf(stderr, "  CANRXGMASK = 0x%02x\n", in32(devinfo->base + IMX7_FLEXCAN_RXGMASK));
    fprintf(stderr, "CANRX14MASK = 0x%02x", in32(devinfo->base + IMX7_FLEXCAN_RX14MASK));
    fprintf(stderr, "  CANRX15MASK = 0x%02x\n", in32(devinfo->base + IMX7_FLEXCAN_RX15MASK));
    fprintf(stderr, "CANECR = 0x%02x\t\t", in32(devinfo->base + IMX7_FLEXCAN_ECR));
    fprintf(stderr, "  CANESR = 0x%02x\n", in32(devinfo->base + IMX7_FLEXCAN_ESR1));
    fprintf(stderr, "CANIMASK2 = 0x%02x\t", in32(devinfo->base + IMX7_FLEXCAN_IMASK2));
    fprintf(stderr, "  CANIMASK1 = 0x%02x\n", in32(devinfo->base + IMX7_FLEXCAN_IMASK1));
    fprintf(stderr, "CANIFLAG2  = 0x%02x\t", in32(devinfo->base + IMX7_FLEXCAN_IFLAG2));
    fprintf(stderr, "  CANIFLAG1 = 0x%02x\n", in32(devinfo->base + IMX7_FLEXCAN_IFLAG1));
    fprintf(stderr, "******************************************************\n");
}

/**
 * Print CAN device mailbox memory.
 *
 * @param devinfo Device info structure.
 */
void can_print_mailbox(CANDEV_FLEXCAN_INFO *devinfo)
{
    unsigned int i = 0;

    fprintf(stderr, "RX Mailboxes\n");
    fprintf(stderr, "MB\tMID\t\tMCF\t\tMDH\t\tMDL\n");
    fprintf(stderr, "======================================================\n");
    for (i = 0; i < devinfo->numrx; i++) {
        fprintf(stderr, "RX%d\t0x%8X\t0x%8X\t0x%8X\t0x%8X\t\n", i, devinfo->canmsg[i].canmid,
                devinfo->canmsg[i].canmcf, devinfo->canmsg[i].canmdh, devinfo->canmsg[i].canmdl);
    }
    fprintf(stderr, "\nTX Mailboxes\n");
    fprintf(stderr, "MB\tMID\t\tMCF\t\tMDH\t\tMDL\n");
    fprintf(stderr, "======================================================\n");
    for (i = devinfo->numrx; i < (devinfo->numrx + devinfo->numtx); i++) {
        fprintf(stderr, "TX%d\t0x%8X\t0x%8X\t0x%8X\t0x%8X\t\n", i, devinfo->canmsg[i].canmid,
                devinfo->canmsg[i].canmcf, devinfo->canmsg[i].canmdh, devinfo->canmsg[i].canmdl);
    }
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/can/mx7x/canmx7x.c $ $Rev: 893475 $")
#endif
