/*
* $QNXLicenseC:
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

#ifndef _FSL_MU_H_
#define _FSL_MU_H_

#include "fsl_common.h"

/*!
 * @addtogroup mu
 * @{
 */

/*! @file */

/******************************************************************************
 * Definitions
 *****************************************************************************/

/*! @name Driver version */
/*@{*/
/*! @brief MU driver version 2.0.0. */
#define FSL_MU_DRIVER_VERSION (MAKE_VERSION(2, 0, 0))
/*@}*/

/*!
 * @brief MU status flags.
 */
enum _mu_status_flags {
    kMU_Tx0EmptyFlag = (1U << (MU_SR_TEn_SHIFT + 3U)), /*!< TX0 empty. */
    kMU_Tx1EmptyFlag = (1U << (MU_SR_TEn_SHIFT + 2U)), /*!< TX1 empty. */
    kMU_Tx2EmptyFlag = (1U << (MU_SR_TEn_SHIFT + 1U)), /*!< TX2 empty. */
    kMU_Tx3EmptyFlag = (1U << (MU_SR_TEn_SHIFT + 0U)), /*!< TX3 empty. */

    kMU_Rx0FullFlag = (1U << (MU_SR_RFn_SHIFT + 3U)), /*!< RX0 full.  */
    kMU_Rx1FullFlag = (1U << (MU_SR_RFn_SHIFT + 2U)), /*!< RX1 full.  */
    kMU_Rx2FullFlag = (1U << (MU_SR_RFn_SHIFT + 1U)), /*!< RX2 full.  */
    kMU_Rx3FullFlag = (1U << (MU_SR_RFn_SHIFT + 0U)), /*!< RX3 full.  */

    kMU_GenInt0Flag = (1U << (MU_SR_GIPn_SHIFT + 3U)), /*!< General purpose interrupt 0 pending. */
    kMU_GenInt1Flag = (1U << (MU_SR_GIPn_SHIFT + 2U)), /*!< General purpose interrupt 0 pending. */
    kMU_GenInt2Flag = (1U << (MU_SR_GIPn_SHIFT + 1U)), /*!< General purpose interrupt 0 pending. */
    kMU_GenInt3Flag = (1U << (MU_SR_GIPn_SHIFT + 0U)), /*!< General purpose interrupt 0 pending. */

    kMU_EventPendingFlag = MU_SR_EP_MASK,  /*!< MU event pending.               */
    kMU_FlagsUpdatingFlag = MU_SR_FUP_MASK /*!< MU flags update is on-going.    */
};

/*!
 * @brief MU interrupt source to enable.
 */
enum _mu_interrupt_enable {
    kMU_Tx0EmptyInterruptEnable = (1U << (MU_CR_TIEn_SHIFT + 3U)), /*!< TX0 empty. */
    kMU_Tx1EmptyInterruptEnable = (1U << (MU_CR_TIEn_SHIFT + 2U)), /*!< TX1 empty. */
    kMU_Tx2EmptyInterruptEnable = (1U << (MU_CR_TIEn_SHIFT + 1U)), /*!< TX2 empty. */
    kMU_Tx3EmptyInterruptEnable = (1U << (MU_CR_TIEn_SHIFT + 0U)), /*!< TX3 empty. */

    kMU_Rx0FullInterruptEnable = (1U << (MU_CR_RIEn_SHIFT + 3U)), /*!< RX0 full.  */
    kMU_Rx1FullInterruptEnable = (1U << (MU_CR_RIEn_SHIFT + 2U)), /*!< RX1 full.  */
    kMU_Rx2FullInterruptEnable = (1U << (MU_CR_RIEn_SHIFT + 1U)), /*!< RX2 full.  */
    kMU_Rx3FullInterruptEnable = (1U << (MU_CR_RIEn_SHIFT + 0U)), /*!< RX3 full.  */

    kMU_GenInt0InterruptEnable = (1U << (MU_CR_GIEn_SHIFT + 3U)), /*!< General purpose interrupt 0. */
    kMU_GenInt1InterruptEnable = (1U << (MU_CR_GIEn_SHIFT + 2U)), /*!< General purpose interrupt 1. */
    kMU_GenInt2InterruptEnable = (1U << (MU_CR_GIEn_SHIFT + 1U)), /*!< General purpose interrupt 2. */
    kMU_GenInt3InterruptEnable = (1U << (MU_CR_GIEn_SHIFT + 0U))  /*!< General purpose interrupt 3. */
};

/*!
 * @brief MU interrupt that could be triggered to the other core.
 */
enum _mu_interrupt_trigger {
    kMU_NmiInterruptTrigger = MU_CR_NMI_MASK,                      /*!< NMI interrupt.               */
    kMU_GenInt0InterruptTrigger = (1U << (MU_CR_GIRn_SHIFT + 3U)), /*!< General purpose interrupt 0. */
    kMU_GenInt1InterruptTrigger = (1U << (MU_CR_GIRn_SHIFT + 2U)), /*!< General purpose interrupt 1. */
    kMU_GenInt2InterruptTrigger = (1U << (MU_CR_GIRn_SHIFT + 1U)), /*!< General purpose interrupt 2. */
    kMU_GenInt3InterruptTrigger = (1U << (MU_CR_GIRn_SHIFT + 0U))  /*!< General purpose interrupt 3. */
};

/*!
 * @brief Core B boot mode.
 */
typedef enum _mu_core_boot_mode {
    kMU_CoreBootFromAddr0 = 0x00U, /*!< Boot from 0x00.      */
    kMU_CoreBootFromDmem = 0x01U,  /*!< Boot from DMEM base. */
    kMU_CoreBootFromImem = 0x02U   /*!< Boot from IMEM base. */
} mu_core_boot_mode_t;

/*!
 * @brief Power mode definition.
 */
typedef enum _mu_power_mode {
    kMU_PowerModeRun = 0x00U,  /*!< Run mode.           */
    kMU_PowerModeWait = 0x01U, /*!< WAIT mode.          */
    kMU_PowerModeStop = 0x02U, /*!< STOP/VLPS mode.     */
    kMU_PowerModeDsm = 0x03U   /*!< DSM: LLS/VLLS mode. */
} mu_power_mode_t;

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @name MU initialization.
 * @{
 */
/*!
 * @brief Initializes the MU module.
 *
 * This function enables the MU clock only.
 *
 * @param base MU peripheral base address.
 */
static inline void MU_Init(MU_Type *base)
{
#if defined(MU0_A)
    if (MU0_A == base) {
        CLOCK_EnableClock(kCLOCK_Mu0A);
    }
#endif
#if defined(MU0_B)
    if (MU0_B == base) {
        CLOCK_EnableClock(kCLOCK_Mu0B);
    }
#endif
}

/*!
 * @brief De-initializes the MU module.
 *
 * This function disables the MU clock only.
 *
 * @param base MU peripheral base address.
 */
static inline void MU_Deinit(MU_Type *base)
{
#if defined(MU0_A)
    if (MU0_A == base) {
        CLOCK_DisableClock(kCLOCK_Mu0A);
    }
#endif
#if defined(MU0_B)
    if (MU0_B == base) {
        CLOCK_DisableClock(kCLOCK_Mu0B);
    }
#endif
}

/* @} */

/*!
 * @name MU Message
 * @{
 */

/*!
 * @brief Writes a message to the TX register.
 *
 * This function writes a message to the specific TX register. It does not check
 * whether the TX register is empty or not. The upper layer should make sure the TX
 * register is empty before calling this function. This function can be used
 * in ISR for better performance.
 *
 * @code
 * while (!(kMU_Tx0EmptyFlag & MU_GetStatusFlags(base))) { } // Wait for TX0 register empty.
 * MU_SendMsgNonBlocking(base, 0U, MSG_VAL); // Write message to the TX0 register.
 * @endcode
 *
 * @param base MU peripheral base address.
 * @param regIndex  TX register index.
 * @param msg      Message to send.
 */
static inline void MU_SendMsgNonBlocking(MU_Type *base, uint32_t regIndex, uint32_t msg)
{
    assert(regIndex < MU_TR_COUNT);

    base->TR[regIndex] = msg;
}

/*!
 * @brief Blocks to send a message.
 *
 * This function waits until the TX register is empty and sends the message.
 *
 * @param base MU peripheral base address.
 * @param regIndex  TX register index.
 * @param msg      Message to send.
 */
void MU_SendMsg(MU_Type *base, uint32_t regIndex, uint32_t msg);

/*!
 * @brief Reads a message from the RX register.
 *
 * This function reads a message from the specific RX register. It does not check
 * whether the RX register is full or not. The upper layer should make sure the RX
 * register is full before calling this function. This function can be used
 * in ISR for better performance.
 *
 * @code
 * uint32_t msg;
 * while (!(kMU_Rx0FullFlag & MU_GetStatusFlags(base)))
 * {
 * } // Wait for the RX0 register full.
 *
 * msg = MU_ReceiveMsgNonBlocking(base, 0U);  // Read message from RX0 register.
 * @endcode
 *
 * @param base MU peripheral base address.
 * @param regIndex  TX register index.
 * @return The received message.
 */
static inline uint32_t MU_ReceiveMsgNonBlocking(MU_Type *base, uint32_t regIndex)
{
    assert(regIndex < MU_TR_COUNT);

    return base->RR[regIndex];
}

/*!
 * @brief Blocks to receive a message.
 *
 * This function waits until the RX register is full and receives the message.
 *
 * @param base MU peripheral base address.
 * @param regIndex  RX register index.
 * @return The received message.
 */
uint32_t MU_ReceiveMsg(MU_Type *base, uint32_t regIndex);

/* @} */

/*!
 * @name MU Flags
 * @{
 */

/*!
 * @brief Sets the 3-bit MU flags reflect on the other MU side.
 *
 * This function sets the 3-bit MU flags directly. Every time the 3-bit MU flags are changed,
 * the status flag \c kMU_FlagsUpdatingFlag asserts indicating the 3-bit MU flags are
 * updating to the other side. After the 3-bit MU flags are updated, the status flag
 * \c kMU_FlagsUpdatingFlag is cleared by hardware. During the flags updating period,
 * the flags cannot be changed. The upper layer should make sure the status flag
 * \c kMU_FlagsUpdatingFlag is cleared before calling this function.
 *
 * @code
 * while (kMU_FlagsUpdatingFlag & MU_GetStatusFlags(base))
 * {
 * } // Wait for previous MU flags updating.
 *
 * MU_SetFlagsNonBlocking(base, 0U); // Set the mU flags.
 * @endcode
 *
 * @param base MU peripheral base address.
 * @param flags The 3-bit MU flags to set.
 */
static inline void MU_SetFlagsNonBlocking(MU_Type *base, uint32_t flags)
{
    uint32_t reg = base->CR;
    reg = (reg & (uint32_t)~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | MU_CR_Fn_MASK)) | MU_CR_Fn(flags);
    base->CR = reg;
}

/*!
 * @brief Blocks setting the 3-bit MU flags reflect on the other MU side.
 *
 * This function blocks setting the 3-bit MU flags. Every time the 3-bit MU flags are changed,
 * the status flag \c kMU_FlagsUpdatingFlag asserts indicating the 3-bit MU flags are
 * updating to the other side. After the 3-bit MU flags are updated, the status flag
 * \c kMU_FlagsUpdatingFlag is cleared by hardware. During the flags updating period,
 * the flags cannot be changed. This function waits for the MU status flag
 * \c kMU_FlagsUpdatingFlag cleared and sets the 3-bit MU flags.
 *
 * @param base MU peripheral base address.
 * @param flags The 3-bit MU flags to set.
 */
void MU_SetFlags(MU_Type *base, uint32_t flags);

/*!
 * @brief Gets the current value of the 3-bit MU flags set by the other side.
 *
 * This functions gets the current 3-bit MU flags on the current side.
 *
 * @param base MU peripheral base address.
 * @return flags   Current value of the 3-bit flags.
 */
static inline uint32_t MU_GetFlags(MU_Type *base)
{
    return (base->SR & MU_SR_Fn_MASK) >> MU_SR_Fn_SHIFT;
}

/* @} */

/*!
 * @name Status and Interrupt.
 * @{
 */

/*!
 * @brief Gets the MU status flags.
 *
 * This function returns the bit mask of the MU status flags. See _mu_status_flags.
 *
 * @code
 * uint32_t flags;
 * flags = MU_GetStatusFlags(base); // Get all status flags.
 * if (kMU_Tx0EmptyFlag & flags)
 * {
 *     // The TX0 register is empty. Message can be sent.
 *     MU_SendMsgNonBlocking(base, 0U, MSG0_VAL);
 * }
 * if (kMU_Tx1EmptyFlag & flags)
 * {
 *     // The TX1 register is empty. Message can be sent.
 *     MU_SendMsgNonBlocking(base, 1U, MSG1_VAL);
 * }
 * @endcode
 *
 * @param base MU peripheral base address.
 * @return      Bit mask of the MU status flags, see _mu_status_flags.
 */
static inline uint32_t MU_GetStatusFlags(MU_Type *base)
{
    return (base->SR & (MU_SR_TEn_MASK | MU_SR_RFn_MASK | MU_SR_GIPn_MASK | MU_SR_EP_MASK | MU_SR_FUP_MASK));
}

/*!
 * @brief Clears the specific MU status flags.
 *
 * This function clears the specific MU status flags. The flags to clear should
 * be passed in as bit mask. See _mu_status_flags.
 *
 * @code
 * //Clear general interrupt 0 and general interrupt 1 pending flags.
 * MU_ClearStatusFlags(base, kMU_GenInt0Flag | kMU_GenInt1Flag);
 * @endcode
 *
 * @param base MU peripheral base address.
 * @param mask  Bit mask of the MU status flags. See _mu_status_flags.
 *
 * @note Only clear the general purpose interrupt flags using this function.
 * The other flags, such as kMU_Rx0FullFlag, are cleared by hardware.
 */
static inline void MU_ClearStatusFlags(MU_Type *base, uint32_t mask)
{
    base->SR = (mask & MU_SR_GIPn_MASK);
}

/*!
 * @brief Enables the specific MU interrupts.
 *
 * This function enables the specific MU interrupts. The interrupts to enable
 * should be passed in as bit mask. See _mu_interrupt_enable.
 *
 * @code
 * // Enable general interrupt 0 and TX0 empty interrupt.
 * MU_EnableInterrupts(base, kMU_GenInt0InterruptEnable | kMU_Tx0EmptyInterruptEnable);
 * @endcode
 *
 * @param base MU peripheral base address.
 * @param mask  Bit mask of the MU interrupts. See _mu_interrupt_enable.
 */
static inline void MU_EnableInterrupts(MU_Type *base, uint32_t mask)
{
    uint32_t reg = base->CR;
    reg = (reg & (uint32_t)~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | mask;
    base->CR = reg;
}

/*!
 * @brief Disables the specific MU interrupts.
 *
 * This function disables the specific MU interrupts. The interrupts to disable
 * should be passed in as bit mask. See _mu_interrupt_enable.
 *
 * @code
 * // Disable general interrupt 0 and TX0 empty interrupt.
 * MU_DisableInterrupts(base, kMU_GenInt0InterruptEnable | kMU_Tx0EmptyInterruptEnable);
 * @endcode
 *
 * @param base MU peripheral base address.
 * @param mask  Bit mask of the MU interrupts. See _mu_interrupt_enable.
 */
static inline void MU_DisableInterrupts(MU_Type *base, uint32_t mask)
{
    uint32_t reg = base->CR;
    reg &= ~((MU_CR_GIRn_MASK | MU_CR_NMI_MASK) | mask);
    base->CR = reg;
}

/*!
 * @brief Triggers interrupts to the other core.
 *
 * This function triggers the specific interrupts to the other core. The interrupts
 * to trigger are passed in as bit mask. See \ref _mu_interrupt_trigger.
 * The MU should not trigger an interrupt to the other core when the previous interrupt
 * has not been processed by the other core. This function checks whether the
 * previous interrupts have been processed. If not, it returns an error.
 *
 * @code
 * if (kStatus_Success != MU_TriggerInterrupts(base, kMU_GenInt0InterruptTrigger | kMU_GenInt2InterruptTrigger))
 * {
 *     // Previous general purpose interrupt 0 or general purpose interrupt 2
 *     // has not been processed by the other core.
 * }
 * @endcode
 *
 * @param base MU peripheral base address.
 * @param mask Bit mask of the interrupts to trigger. See _mu_interrupt_trigger.
 * @retval kStatus_Success    Interrupts have been triggered successfully.
 * @retval kStatus_Fail       Previous interrupts have not been accepted.
 */
status_t MU_TriggerInterrupts(MU_Type *base, uint32_t mask);

/*!
 * @brief Clear non-maskable interrupt (NMI) sent by the other core.
 *
 * This functions clears non-maskable interrupt (NMI) sent by the other core.
 *
 * @param base MU peripheral base address.
 */
static inline void MU_ClearNmi(MU_Type *base)
{
    base->SR = MU_SR_NMIC_MASK;
}

/* @} */

/*!
 * @name MU misc functions
 * @{
 */

/*!
 * @brief Boots the core B.
 *
 * This function boots the core B with a boot configuration.
 *
 * @param base MU peripheral base address.
 * @param mode Core B boot mode.
 * @note Only MU side A can use this function.
 */
void MU_BootCoreB(MU_Type *base, mu_core_boot_mode_t mode);

/*!
 * @brief Holds the core B reset.
 *
 * This function causes the core B to be held in reset following any reset event.
 *
 * @param base MU peripheral base address.
 */
static inline void MU_HoldCoreBReset(MU_Type *base)
{
    uint32_t reg = base->CR;
    reg = (reg & (uint32_t)~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | MU_CR_BRSTH_MASK;
    base->CR = reg;
}

/*!
 * @brief Resets the MU for both A side and B side.
 *
 * This function resets the MU for both A side and B side. Before reset, it is
 * recommended to interrupt processor B, because this function may affect the
 * ongoing processor B programs.
 *
 * @param base MU peripheral base address.
 * @note Only MU side A could use this function.
 */
static inline void MU_ResetBothSides(MU_Type *base)
{
    uint32_t reg = base->CR;
    reg = (reg & (uint32_t)~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK)) | MU_CR_MUR_MASK;
    base->CR = reg;
}

/*!
 * @brief Enables or disables the clock on the other core.
 *
 * This function enables or disables the platform clock on the other core when
 * that core enters a stop mode. If disabled, the platform clock for the other
 * core is disabled when it enters stop mode. If enabled, the platform clock
 * keeps running on the other core in stop mode, until this core also enters
 * stop mode.
 *
 * @param base MU peripheral base address.
 * @param enable   Enable or disable the clock on the other core.
 */
static inline void MU_SetClockOnOtherCoreEnable(MU_Type *base, bool enable)
{
    uint32_t reg = base->CR;

    reg &= (uint32_t)~(MU_CR_GIRn_MASK | MU_CR_NMI_MASK);

    if (enable) {
        reg |= MU_CR_CLKE_MASK;
    } else {
        reg &= (uint32_t)~(MU_CR_CLKE_MASK);
    }

    base->CR = reg;
}

/*!
 * @brief Gets the power mode of the other core.
 *
 * This function gets the power mode of the other core.
 *
 * @param base MU peripheral base address.
 * @return Power mode of the other core.
 */
static inline mu_power_mode_t MU_GetOtherCorePowerMode(MU_Type *base)
{
    return (mu_power_mode_t)((base->SR & MU_SR_PM_MASK) >> MU_SR_PM_SHIFT);
}

/* @} */

#if defined(__cplusplus)
}
#endif /*_cplusplus*/
/*@}*/

#endif /* _FSL_MU_H_*/

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/fsl_mu.h $ $Rev: 869668 $")
#endif
