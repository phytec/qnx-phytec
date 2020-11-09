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

#ifndef __HW_MU_REGISTERS_H__
#define __HW_MU_REGISTERS_H__

#include <stdint.h>

/* ----------------------------------------------------------------------------
   -- MU Peripheral Access Layer
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MU_Peripheral_Access_Layer MU Peripheral Access Layer
 * @{
 */

/** MU - Register Layout Typedef */
typedef struct {
    volatile uint32_t TR[4];                             /**< Transmit Register, array offset: 0x00, array step: 0x4 */
    volatile uint32_t RR[4];                             /**< Receive Register, array offset: 0x10, array step: 0x4 */
    volatile uint32_t SR;                                /**< Status Register, offset: 0x20 */
    volatile uint32_t CR;                                /**< Control Register, offset: 0x24 */
} MU_Type;

/* ----------------------------------------------------------------------------
   -- MU Register Masks
   ---------------------------------------------------------------------------- */

/*!
 * @addtogroup MU_Register_Masks MU Register Masks
 * @{
 */

/*! @name TR - Transmit Register */
#define MU_TR_DATA_MASK                          (0xFFFFFFFFU)
#define MU_TR_DATA_SHIFT                         (0U)
#define MU_TR_DATA(x)                            (((uint32_t)(((uint32_t)(x)) << MU_TR_DATA_SHIFT)) & MU_TR_DATA_MASK)

/* The count of MU_TR */
#define MU_TR_COUNT                              (4U)

/*! @name RR - Receive Register */
#define MU_RR_DATA_MASK                          (0xFFFFFFFFU)
#define MU_RR_DATA_SHIFT                         (0U)
#define MU_RR_DATA(x)                            (((uint32_t)(((uint32_t)(x)) << MU_RR_DATA_SHIFT)) & MU_RR_DATA_MASK)

/* The count of MU_RR */
#define MU_RR_COUNT                              (4U)

/*! @name SR - Status Register */
#define MU_SR_Fn_MASK                            (0x7U)
#define MU_SR_Fn_SHIFT                           (0U)
#define MU_SR_Fn(x)                              (((uint32_t)(((uint32_t)(x)) << MU_SR_Fn_SHIFT)) & MU_SR_Fn_MASK)
#define MU_SR_NMIC_MASK                          (0x8U)
#define MU_SR_NMIC_SHIFT                         (3U)
#define MU_SR_NMIC(x)                            (((uint32_t)(((uint32_t)(x)) << MU_SR_NMIC_SHIFT)) & MU_SR_NMIC_MASK)
#define MU_SR_EP_MASK                            (0x10U)
#define MU_SR_EP_SHIFT                           (4U)
#define MU_SR_EP(x)                              (((uint32_t)(((uint32_t)(x)) << MU_SR_EP_SHIFT)) & MU_SR_EP_MASK)
#define MU_SR_PM_MASK                            (0x60U)
#define MU_SR_PM_SHIFT                           (5U)
#define MU_SR_PM(x)                              (((uint32_t)(((uint32_t)(x)) << MU_SR_PM_SHIFT)) & MU_SR_PM_MASK)
#define MU_SR_FUP_MASK                           (0x100U)
#define MU_SR_FUP_SHIFT                          (8U)
#define MU_SR_FUP(x)                             (((uint32_t)(((uint32_t)(x)) << MU_SR_FUP_SHIFT)) & MU_SR_FUP_MASK)
#define MU_SR_TEn_MASK                           (0xF00000U)
#define MU_SR_TEn_SHIFT                          (20U)
#define MU_SR_TEn(x)                             (((uint32_t)(((uint32_t)(x)) << MU_SR_TEn_SHIFT)) & MU_SR_TEn_MASK)
#define MU_SR_RFn_MASK                           (0xF000000U)
#define MU_SR_RFn_SHIFT                          (24U)
#define MU_SR_RFn(x)                             (((uint32_t)(((uint32_t)(x)) << MU_SR_RFn_SHIFT)) & MU_SR_RFn_MASK)
#define MU_SR_GIPn_MASK                          (0xF0000000U)
#define MU_SR_GIPn_SHIFT                         (28U)
#define MU_SR_GIPn(x)                            (((uint32_t)(((uint32_t)(x)) << MU_SR_GIPn_SHIFT)) & MU_SR_GIPn_MASK)

/*! @name CR - Control Register */
#define MU_CR_Fn_MASK                            (0x7UL)
#define MU_CR_Fn_SHIFT                           (0U)
#define MU_CR_Fn(x)                              (((uint32_t)(((uint32_t)(x)) << MU_CR_Fn_SHIFT)) & MU_CR_Fn_MASK)
#define MU_CR_NMI_MASK                           (0x8UL)
#define MU_CR_NMI_SHIFT                          (3U)
#define MU_CR_NMI(x)                             (((uint32_t)(((uint32_t)(x)) << MU_CR_NMI_SHIFT)) & MU_CR_NMI_MASK)
#define MU_CR_MUR_MASK                           (0x20U)
#define MU_CR_MUR_SHIFT                          (5U)
#define MU_CR_MUR(x)                             (((uint32_t)(((uint32_t)(x)) << MU_CR_MUR_SHIFT)) & MU_CR_MUR_MASK)
#define MU_CR_BRSTH_MASK                         (0x80UL)
#define MU_CR_BRSTH_SHIFT                        (7U)
#define MU_CR_BRSTH(x)                           (((uint32_t)(((uint32_t)(x)) << MU_CR_BRSTH_SHIFT)) & MU_CR_BRSTH_MASK)
#define MU_CR_CLKE_MASK                          (0x100UL)
#define MU_CR_CLKE_SHIFT                         (8U)
#define MU_CR_CLKE(x)                            (((uint32_t)(((uint32_t)(x)) << MU_CR_CLKE_SHIFT)) & MU_CR_CLKE_MASK)
#define MU_CR_BBOOT_MASK                         (0x600UL)
#define MU_CR_BBOOT_SHIFT                        (9U)
#define MU_CR_BBOOT(x)                           (((uint32_t)(((uint32_t)(x)) << MU_CR_BBOOT_SHIFT)) & MU_CR_BBOOT_MASK)
#define MU_CR_GIRn_MASK                          (0xF0000UL)
#define MU_CR_GIRn_SHIFT                         (16U)
#define MU_CR_GIRn(x)                            (((uint32_t)(((uint32_t)(x)) << MU_CR_GIRn_SHIFT)) & MU_CR_GIRn_MASK)
#define MU_CR_TIEn_MASK                          (0xF00000U)
#define MU_CR_TIEn_SHIFT                         (20U)
#define MU_CR_TIEn(x)                            (((uint32_t)(((uint32_t)(x)) << MU_CR_TIEn_SHIFT)) & MU_CR_TIEn_MASK)
#define MU_CR_RIEn_MASK                          (0xF000000U)
#define MU_CR_RIEn_SHIFT                         (24U)
#define MU_CR_RIEn(x)                            (((uint32_t)(((uint32_t)(x)) << MU_CR_RIEn_SHIFT)) & MU_CR_RIEn_MASK)
#define MU_CR_GIEn_MASK                          (0xF0000000U)
#define MU_CR_GIEn_SHIFT                         (28U)
#define MU_CR_GIEn(x)                            (((uint32_t)(((uint32_t)(x)) << MU_CR_GIEn_SHIFT)) & MU_CR_GIEn_MASK)


/*!
 * @}
 */ /* End of group MU_Register_Masks */


/* MU - Peripheral instance base addresses */

/** This define is used to access MU registers */
#define DSC_MU_BASE_ADDR(X, Y)  ((MU_Type*) (((uint32_t) DSC_BASE_ADDR(X)) \
    + 0xC000 + (0x80 * Y)))

/** Peripheral MU0 base pointer */
#define MU0                                     ((MU_Type *)MU0_BASE)
/** Peripheral MU1 base pointer */
#define MU1                                     ((MU_Type *)MU1_BASE)
/** Peripheral MU2 base pointer */
#define MU2                                     ((MU_Type *)MU2_BASE)
/** Peripheral MU3 base pointer */
#define MU3                                     ((MU_Type *)MU3_BASE)
/** Peripheral MU4 base pointer */
#define MU4                                     ((MU_Type *)MU4_BASE)
/** Peripheral MU5 base pointer */
#define MU5                                     ((MU_Type *)MU5_BASE)
/** Peripheral MU6 base pointer */
#define MU6                                     ((MU_Type *)MU6_BASE)
/** Peripheral MU7 base pointer */
#define MU7                                     ((MU_Type *)MU7_BASE)
/** Peripheral MU8 base pointer */
#define MU8                                     ((MU_Type *)MU8_BASE)
/** Peripheral MU9 base pointer */
#define MU9                                     ((MU_Type *)MU9_BASE)
/** Peripheral MU10 base pointer */
#define MU10                                    ((MU_Type *)MU10_BASE)

/** Array initializer of MU peripheral base addresses */
#define MU_BASE_ADDRS                            { MU0_BASE, MU1_BASE,  \
                                                   MU2_BASE, MU3_BASE,  \
                                                   MU4_BASE, MU5_BASE,  \
                                                   MU6_BASE, MU7_BASE,  \
                                                   MU8_BASE, MU9_BASE,  \
                                                   MU10_BASE }
/** Array initializer of MU peripheral base pointers */
#define MU_BASE_PTRS                             { MU0, MU1, MU2    \
                                                   MU3, MU4, MU5    \
                                                   MU6, MU7, MU8    \
                                                   MU9, MU10 }
/** Interrupt vectors for the MU peripheral type */
#define MU_IRQS                                  { NotAvail_IRQn, NotAvail_IRQn, \
                                                   NotAvail_IRQn, NotAvail_IRQn, \
                                                   NotAvail_IRQn, NotAvail_IRQn, \
                                                   NotAvail_IRQn, NotAvail_IRQn, \
                                                   NotAvail_IRQn, NotAvail_IRQn, \
                                                   NotAvail_IRQn }

/*!
 * @}
 */ /* end of group MU_Peripheral_Access_Layer */

#endif /* __HW_MU_REGISTERS_H__ */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/lib/hw_vendor/nxp/imx8/sci/aarch64/MX8_mu.h $ $Rev: 875993 $")
#endif
