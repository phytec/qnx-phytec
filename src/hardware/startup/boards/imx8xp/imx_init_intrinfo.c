/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017 NXP
 * Copyright 2018, QNX Software Systems.
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

#include "startup.h"
#include <aarch64/mx8xp.h>
#include <aarch64/imx8_common/imx_pcie.h>
#include <aarch64/imx8_common/imx_intmux.h>

/**
 * i.MX startup source file.
 *
 * @file       imx_init_intrinfo.c
 * @addtogroup startup
 * @{
 *
 * <h2 class="groupheader"> Interrupt vector mapping</h2>
 * <div class="memitem">
 * <div class="memproto">
 * Interrupt vector:<br/>
 * </div>
 *  <div class="memdoc">
 *   <UL>
 *    <li> 32 - 543: ARM General Interrupt Controller (same IRQs as defined in corresponding reference manual)
 *    <li>544 - 575: GPIO 0 interrupts, device GPIO0[0 - 31]
 *    <li>576 - 607: GPIO 1 interrupts, device GPIO1[0 - 31]
 *    <li>608 - 639: GPIO 2 interrupts, device GPIO2[0 - 31]
 *    <li>640 - 671: GPIO 3 interrupts, device GPIO3[0 - 31]
 *    <li>672 - 703: GPIO 4 interrupts, device GPIO4[0 - 31]
 *    <li>704 - 735: GPIO 5 interrupts, device GPIO5[0 - 31]
 *    <li>4096 - 4351: PCIeB MSI interrupts, device MSI[0 - 255]
 *   </UL>
 *  </div>
 * </div>
 */

extern struct callout_rtn interrupt_id_imx_msi;
extern struct callout_rtn interrupt_eoi_imx_msi;
extern struct callout_rtn interrupt_mask_imx_msi;
extern struct callout_rtn interrupt_unmask_imx_msi;

extern struct callout_rtn interrupt_id_imx_gpio;
extern struct callout_rtn interrupt_eoi_imx_gpio;
extern struct callout_rtn interrupt_mask_imx_gpio;
extern struct callout_rtn interrupt_unmask_imx_gpio;

extern struct callout_rtn interrupt_id_imx_irqsteer;
extern struct callout_rtn interrupt_eoi_imx_irqsteer;
extern struct callout_rtn interrupt_mask_imx_irqsteer;
extern struct callout_rtn interrupt_unmask_imx_irqsteer;

extern struct callout_rtn interrupt_id_imx_intmux;
extern struct callout_rtn interrupt_eoi_imx_intmux;
extern struct callout_rtn interrupt_mask_imx_intmux;
extern struct callout_rtn interrupt_unmask_imx_intmux;

static paddr_t imx_pcieb_msi_base = (IMX_PCIEB_BASE + IMX_PCIE_PL_MSICIn_ENB);

static paddr_t imx_gpio0_base = IMX_GPIO0_BASE;
static paddr_t imx_gpio1_base = IMX_GPIO1_BASE;
static paddr_t imx_gpio2_base = IMX_GPIO2_BASE;
static paddr_t imx_gpio3_base = IMX_GPIO3_BASE;
static paddr_t imx_gpio4_base = IMX_GPIO4_BASE;
static paddr_t imx_gpio5_base = IMX_GPIO5_BASE;

static paddr_t imx_dc0_mipi0_base = IMX_DC0_MIPI0_BASE;
static paddr_t imx_dc0_mipi1_base = IMX_DC0_MIPI1_BASE;
static paddr_t imx_mipi_csi0_base = IMX_MIPI_CSI0_BASE;
static paddr_t imx_pi_base = IMX_PI_BASE;

static paddr_t imx_m40_intmux_chn0_base = IMX_M40_INTMUX_BASE + IMX_INTMUX_CHAN(0);
static paddr_t imx_m40_intmux_chn1_base = IMX_M40_INTMUX_BASE + IMX_INTMUX_CHAN(1);
static paddr_t imx_m40_intmux_chn2_base = IMX_M40_INTMUX_BASE + IMX_INTMUX_CHAN(2);
static paddr_t imx_m40_intmux_chn3_base = IMX_M40_INTMUX_BASE + IMX_INTMUX_CHAN(3);
static paddr_t imx_m40_intmux_chn4_base = IMX_M40_INTMUX_BASE + IMX_INTMUX_CHAN(4);
static paddr_t imx_m40_intmux_chn5_base = IMX_M40_INTMUX_BASE + IMX_INTMUX_CHAN(5);
static paddr_t imx_m40_intmux_chn6_base = IMX_M40_INTMUX_BASE + IMX_INTMUX_CHAN(6);
static paddr_t imx_m40_intmux_chn7_base = IMX_M40_INTMUX_BASE + IMX_INTMUX_CHAN(7);

static const struct startup_intrinfo pci_steer_intr[] = {

    /**< PCIeB MSI/MSI-X interrupts (256) starting at 0x1000) */
    {   .vector_base     = 0x1000,              /* Vector base (MSI/MSI-X 0x1000 - 0x10FF) */
        .num_vectors     = 256,                 /* 256 MSI's */
        .cascade_vector  = IMX_PCIEB_MSI,       /* MSI vector number */
        .cpu_intr_base   = 0,                   /* CPU vector base (MSI/MSI-X from 0 - 255) */
        .cpu_intr_stride = 0,
        .flags           = INTR_FLAG_MSI,
        .id              = { INTR_GENFLAG_NOGLITCH, 0, &interrupt_id_imx_msi },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK, 0, &interrupt_eoi_imx_msi },
        .mask            = &interrupt_mask_imx_msi,
        .unmask          = &interrupt_unmask_imx_msi,
        .config          = 0,
        .patch_data      = &imx_pcieb_msi_base,
    },
    /**< GPIO 0 interrupt vectors 544 - 575 (device: GPIO0[0 - 31]) */
    {    .vector_base     = 544,                 /* Vector base */
         .num_vectors     = 32,                  /* Number of vectors */
         .cascade_vector  = IMX_GPIO0_IRQ,       /* Cascade vector */
         .cpu_intr_base   = 0,                   /* CPU vector base */
         .cpu_intr_stride = 0,                   /* CPU vector stride */
         .flags           = 0,                   /* Flags */
         .id              = { 0, 0, &interrupt_id_imx_gpio },
         .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_gpio },
         .mask            = &interrupt_mask_imx_gpio, /* Mask callout */
         .unmask          = &interrupt_unmask_imx_gpio, /* Unmask callout */
         .config          = 0,                   /* Config callout */
         .patch_data      = &imx_gpio0_base,
    },
    /**< GPIO 1 interrupt vectors 576 - 607 (device: GPIO1[0 - 31]) */
    {   .vector_base     = 576,                  /* Vector base */
         .num_vectors     = 32,                  /* Number of vectors */
         .cascade_vector  = IMX_GPIO1_IRQ,       /* Cascade vector */
         .cpu_intr_base   = 0,                   /* CPU vector base */
         .cpu_intr_stride = 0,                   /* CPU vector stride */
         .flags           = 0,                   /* Flags */
         .id              = { 0, 0, &interrupt_id_imx_gpio },
         .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_gpio },
         .mask            = &interrupt_mask_imx_gpio, /* Mask callout */
         .unmask          = &interrupt_unmask_imx_gpio, /* Unmask callout */
         .config          = 0,                   /* Config callout */
         .patch_data      = &imx_gpio1_base,
    },
    /**< GPIO 2 interrupt vectors 608 - 639 (device: GPIO2[0 - 31]) */
    {   .vector_base     = 608,                  /* Vector base */
         .num_vectors     = 32,                  /* Number of vectors */
         .cascade_vector  = IMX_GPIO2_IRQ,       /* Cascade vector */
         .cpu_intr_base   = 0,                   /* CPU vector base */
         .cpu_intr_stride = 0,                   /* CPU vector stride */
         .flags           = 0,                   /* Flags */
         .id              = { 0, 0, &interrupt_id_imx_gpio },
         .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_gpio },
         .mask            = &interrupt_mask_imx_gpio, /* Mask callout */
         .unmask          = &interrupt_unmask_imx_gpio, /* Unmask callout */
         .config          = 0,                   /* Config callout */
         .patch_data      = &imx_gpio2_base,
    },
    /**< GPIO 3 interrupt vectors 640 - 671 (device: GPIO3[0 - 31]) */
    {   .vector_base     = 640,                  /* Vector base */
         .num_vectors     = 32,                  /* Number of vectors */
         .cascade_vector  = IMX_GPIO3_IRQ,       /* Cascade vector */
         .cpu_intr_base   = 0,                   /* CPU vector base */
         .cpu_intr_stride = 0,                   /* CPU vector stride */
         .flags           = 0,                   /* Flags */
         .id              = { 0, 0, &interrupt_id_imx_gpio },
         .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_gpio },
         .mask            = &interrupt_mask_imx_gpio, /* Mask callout */
         .unmask          = &interrupt_unmask_imx_gpio, /* Unmask callout */
         .config          = 0,                   /* Config callout */
         .patch_data      = &imx_gpio3_base,
    },
    /**< GPIO 4 interrupt vectors 672 - 703 (device: GPIO4[0 - 31]) */
    {   .vector_base     = 672,                  /* Vector base */
         .num_vectors     = 32,                  /* Number of vectors */
         .cascade_vector  = IMX_GPIO4_IRQ,       /* Cascade vector */
         .cpu_intr_base   = 0,                   /* CPU vector base */
         .cpu_intr_stride = 0,                   /* CPU vector stride */
         .flags           = 0,                   /* Flags */
         .id              = { 0, 0, &interrupt_id_imx_gpio },
         .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_gpio },
         .mask            = &interrupt_mask_imx_gpio, /* Mask callout */
         .unmask          = &interrupt_unmask_imx_gpio, /* Unmask callout */
         .config          = 0,                   /* Config callout */
         .patch_data      = &imx_gpio4_base,
    },
    /**< GPIO 5 interrupt vectors 704 - 735 (device: GPIO5[0 - 31]) */
    {   .vector_base     = 704,                  /* Vector base */
         .num_vectors     = 32,                  /* Number of vectors */
         .cascade_vector  = IMX_GPIO5_IRQ,       /* Cascade vector */
         .cpu_intr_base   = 0,                   /* CPU vector base */
         .cpu_intr_stride = 0,                   /* CPU vector stride */
         .flags           = 0,                   /* Flags */
         .id              = { 0, 0, &interrupt_id_imx_gpio },
         .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_gpio },
         .mask            = &interrupt_mask_imx_gpio, /* Mask callout */
         .unmask          = &interrupt_unmask_imx_gpio, /* Unmask callout */
         .config          = 0,                   /* Config callout */
         .patch_data      = &imx_gpio5_base,
    },
    /**< STEER interrupt vectors */
    {   .vector_base     = IMX_DC0_MIPI0_STEER_IRQ0,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_DC0_MIPI0_IRQ,                      /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_irqsteer },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_irqsteer },
        .mask            = &interrupt_mask_imx_irqsteer,           /* Mask callout */
        .unmask          = &interrupt_unmask_imx_irqsteer,         /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_dc0_mipi0_base,
    },
    /**< STEER interrupt vectors */
    {   .vector_base     = IMX_DC0_MIPI1_STEER_IRQ0,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_DC0_MIPI1_IRQ,                      /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_irqsteer },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_irqsteer },
        .mask            = &interrupt_mask_imx_irqsteer,           /* Mask callout */
        .unmask          = &interrupt_unmask_imx_irqsteer,         /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_dc0_mipi1_base,
    },
    /**< STEER interrupt vectors */
    {   .vector_base     = IMX_MIPI_CSI0_STEER_IRQ0,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_MIPI_CSI0_IRQ,                      /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_irqsteer },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_irqsteer },
        .mask            = &interrupt_mask_imx_irqsteer,           /* Mask callout */
        .unmask          = &interrupt_unmask_imx_irqsteer,         /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_mipi_csi0_base,
    },
    /**< STEER interrupt vectors */
    {   .vector_base     = IMX_PI_STEER_IRQ0,                      /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_PI_IRQ,                             /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_irqsteer },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_irqsteer },
        .mask            = &interrupt_mask_imx_irqsteer,           /* Mask callout */
        .unmask          = &interrupt_unmask_imx_irqsteer,         /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_pi_base,
    },
    /**< M40 INTMUX CHN0 interrupt vectors */
    {   .vector_base     = IMX_M40_INTMUX_BASE_IRQ0,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_M40_INTMUX_IRQ0,                    /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_intmux },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_intmux },
        .mask            = &interrupt_mask_imx_intmux,             /* Mask callout */
        .unmask          = &interrupt_unmask_imx_intmux,           /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_m40_intmux_chn0_base,
    },
    /**< M40 INTMUX CHN1 interrupt vectors */
    {   .vector_base     = IMX_M40_INTMUX_BASE_IRQ1,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_M40_INTMUX_IRQ1,                    /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_intmux },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_intmux },
        .mask            = &interrupt_mask_imx_intmux,             /* Mask callout */
        .unmask          = &interrupt_unmask_imx_intmux,           /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_m40_intmux_chn1_base,
    },
    /**< M40 INTMUX CHN2 interrupt vectors */
    {   .vector_base     = IMX_M40_INTMUX_BASE_IRQ2,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_M40_INTMUX_IRQ2,                    /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_intmux },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_intmux },
        .mask            = &interrupt_mask_imx_intmux,             /* Mask callout */
        .unmask          = &interrupt_unmask_imx_intmux,           /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_m40_intmux_chn2_base,
    },
    /**< M40 INTMUX CHN3 interrupt vectors */
    {   .vector_base     = IMX_M40_INTMUX_BASE_IRQ3,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_M40_INTMUX_IRQ3,                    /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_intmux },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_intmux },
        .mask            = &interrupt_mask_imx_intmux,             /* Mask callout */
        .unmask          = &interrupt_unmask_imx_intmux,           /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_m40_intmux_chn3_base,
    },
    /**< M40 INTMUX CHN4 interrupt vectors */
    {   .vector_base     = IMX_M40_INTMUX_BASE_IRQ4,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_M40_INTMUX_IRQ4,                    /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_intmux },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_intmux },
        .mask            = &interrupt_mask_imx_intmux,             /* Mask callout */
        .unmask          = &interrupt_unmask_imx_intmux,           /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_m40_intmux_chn4_base,
    },
    /**< M40 INTMUX CHN5 interrupt vectors */
    {   .vector_base     = IMX_M40_INTMUX_BASE_IRQ5,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_M40_INTMUX_IRQ5,                    /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_intmux },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_intmux },
        .mask            = &interrupt_mask_imx_intmux,             /* Mask callout */
        .unmask          = &interrupt_unmask_imx_intmux,           /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_m40_intmux_chn5_base,
    },
    /**< M40 INTMUX CHN6 interrupt vectors */
    {   .vector_base     = IMX_M40_INTMUX_BASE_IRQ6,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_M40_INTMUX_IRQ6,                    /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_intmux },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_intmux },
        .mask            = &interrupt_mask_imx_intmux,             /* Mask callout */
        .unmask          = &interrupt_unmask_imx_intmux,           /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_m40_intmux_chn6_base,
    },
    /**< M40 INTMUX CHN1 interrupt vectors */
    {   .vector_base     = IMX_M40_INTMUX_BASE_IRQ7,               /* Vector base */
        .num_vectors     = 32,                                     /* Number of vectors */
        .cascade_vector  = IMX_M40_INTMUX_IRQ7,                    /* Cascade vector */
        .cpu_intr_base   = 0,                                      /* CPU vector base */
        .cpu_intr_stride = 0,                                      /* CPU vector stride */
        .flags           = 0,                                      /* Flags */
        .id              = { 0, 0, &interrupt_id_imx_intmux },
        .eoi             = { INTR_GENFLAG_LOAD_INTRMASK,   0, &interrupt_eoi_imx_intmux },
        .mask            = &interrupt_mask_imx_intmux,             /* Mask callout */
        .unmask          = &interrupt_unmask_imx_intmux,           /* Unmask callout */
        .config          = 0,                                      /* Config callout */
        .patch_data      = &imx_m40_intmux_chn7_base,
    },
};

/**
 * Initialize Generic Interrupt Controller (GIC).
 */
void init_intrinfo(void)
{
    /* Initialize GIC  */
    gic_v3_init(IMX_GIC_GICD_BASE, IMX_GIC_GICR_BASE, IMX_GIC_GICC_BASE, 0, 0);
    gicv_asinfo(IMX_GIC_GICC_BASE+0x20000, IMX_GIC_GICC_BASE+0x10000, NULL_PADDR, 0, 0);
    /* Add the interrupt callouts */
    add_interrupt_array(pci_steer_intr, sizeof(pci_steer_intr));
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_init_intrinfo.c $ $Rev: 891625 $")
#endif
