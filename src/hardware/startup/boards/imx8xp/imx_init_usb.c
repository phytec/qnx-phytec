/*
 * $QNXLicenseC:
 * Copyright 2016, Freescale Semiconductor, Inc.
 * Copyright 2017, NXP
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

/*
 * This i.MX8 series SOCs contain two USB controllers:
 * The first two OTG1 and OTG2 controllers can function in Host or Device mode. Both OTG modules use an on chip UTMI PHY
 */

#include "startup.h"
#include <aarch64/mx8xp.h>
#include <aarch64/imx8_common/imx_usb.h>
#include <aarch64/imx8_common/imx_usb2phy.h>
#include <aarch64/imx8_common/imx_usb3phy.h>
#include <aarch64/imx8_common/imx_usb3.h>
#include <aarch64/imx8_common/imx_usb3_non_core.h>

#include "imx_startup.h"

#define shift2left(x) ((x) * 4)

/**
 * This function initializes USB PHY
 *
 * @param phy_addr USB_PHY base address.
 */
static void init_usb2_phy(uint32_t phy_addr)
{
    /* Soft-reset USB PHY */
    out32(phy_addr + IMX_USB2PHYx_CTRL_SET, IMX_USB2PHYx_CTRL_SFTRST_MASK);
    imx_usleep(10);

    /* Remove clock gate and soft reset */
    out32(phy_addr + IMX_USB2PHYx_CTRL_CLR, IMX_USB2PHYx_CTRL_SFTRST_MASK | IMX_USB2PHYx_CTRL_CLKGATE_MASK);
    imx_usleep(10);

    /* Power up PHY */
    out32(phy_addr + IMX_USB2PHYx_PWD, 0);

    /* Enable FS/LS device */
    out32(phy_addr + IMX_USB2PHYx_CTRL_SET, (IMX_USB2PHYx_CTRL_ENAUTOSET_USBCLKS_MASK |
                                             IMX_USB2PHYx_CTRL_ENAUTOCLR_PHY_PWD_MASK |
                                             IMX_USB2PHYx_CTRL_ENAUTOCLR_CLKGATE_MASK |
                                             IMX_USB2PHYx_CTRL_ENAUTO_PWRON_PLL_MASK  |
                                             IMX_USB2PHYx_CTRL_ENUTMILEVEL3_MASK      |
                                             IMX_USB2PHYx_CTRL_ENUTMILEVEL2_MASK));
}

/**
 * This function initializes USB PLL
 *
 * @param phy_addr USB_PHY base address.
 */
static void init_usb2_pll(uint32_t phy_addr)
{
    unsigned int cnt = 100;

    out32(phy_addr + IMX_USB2PHYx_PLL_SIC_SET, IMX_USB2PHYx_PLL_REG_ENABLE_MASK);   /* Enable regulator */
    imx_usleep(25);
    out32(phy_addr + IMX_USB2PHYx_PLL_SIC_SET, IMX_USB2PHYx_PLL_POWER_MASK);        /* Enable POWER */
    imx_usleep(25);
    out32(phy_addr + IMX_USB2PHYx_PLL_SIC_CLR, IMX_USB2PHYx_PLL_BYPASS_MASK);       /* Disable Bypass */
    imx_usleep(1);
    out32(phy_addr + IMX_USB2PHYx_PLL_SIC_SET, IMX_USB2PHYx_PLL_ENABLE_MASK);       /* Enable PLL */


    while (!(in32(phy_addr + IMX_USB2PHYx_PLL_SIC) & IMX_USB2PHYx_PLL_LOCK_MASK) && (cnt > 0)){
        cnt--;
        imx_usleep(10);
    }
    if(cnt == 0){
        crash("USB2.0 Init: Timeout waiting on PLL lock");
    }

    out32(phy_addr + IMX_USB2PHYx_PLL_SIC_SET, IMX_USB2PHYx_PLL_EN_USB_CLKS_MASK);  /* Enable USB Clocks */
}

/**
 * This function initializes USB3.0 PHY
 *
 * @param phy_addr USB_PHY base address.
 */

static void init_usb3_phy(__attribute__((unused)) uint32_t phy_addr)
{
    /* This initialization is based on section "Bring-up
     * initialization sequence" from USB3.0 SSPHY User Guide
     */
    out16((IMX_USB3_PHY_BASE + 0xC800 * 4), 0x0830);
    out16((IMX_USB3_PHY_BASE + 0x01E0 * 4), 0x0010);
    out16((IMX_USB3_PHY_BASE + 0x0084 * 4), 0x00F0);
    out16((IMX_USB3_PHY_BASE + 0x0085 * 4), 0x0018);
    out16((IMX_USB3_PHY_BASE + 0x0094 * 4), 0x00D0);
    out16((IMX_USB3_PHY_BASE + 0x0095 * 4), 0x4AAA);
    out16((IMX_USB3_PHY_BASE + 0x0096 * 4), 0x0034);
    out16((IMX_USB3_PHY_BASE + 0x0098 * 4), 0x01EE);
    out16((IMX_USB3_PHY_BASE + 0x0099 * 4), 0x7F03);
    out16((IMX_USB3_PHY_BASE + 0x0097 * 4), 0x0020);
    out16((IMX_USB3_PHY_BASE + 0x01C2 * 4), 0x0000);
    out16((IMX_USB3_PHY_BASE + 0x01C0 * 4), 0x0000);
    out16((IMX_USB3_PHY_BASE + 0x01C1 * 4), 0x0000);
    out16((IMX_USB3_PHY_BASE + 0x01C5 * 4), 0x0007);
    out16((IMX_USB3_PHY_BASE + 0x01C6 * 4), 0x0027);
    out16((IMX_USB3_PHY_BASE + 0x01C7 * 4), 0x0008);
    out16((IMX_USB3_PHY_BASE + 0x01C4 * 4), 0x0022);/* Uncommented bit*/
    out16((IMX_USB3_PHY_BASE + 0x0061 * 4), 0x000A);
    out16((IMX_USB3_PHY_BASE + 0x40EA * 4), 0x0139);
    out16((IMX_USB3_PHY_BASE + 0x4001 * 4), 0xBEFC);
    out16((IMX_USB3_PHY_BASE + 0x4100 * 4), 0x7799);
    out16((IMX_USB3_PHY_BASE + 0x4101 * 4), 0x7798);
    out16((IMX_USB3_PHY_BASE + 0x4102 * 4), 0x5098);
    out16((IMX_USB3_PHY_BASE + 0x4103 * 4), 0x5098);
    out16((IMX_USB3_PHY_BASE + 0x4106 * 4), 0x2090);
    out16((IMX_USB3_PHY_BASE + 0x4107 * 4), 0x2090);
    out16((IMX_USB3_PHY_BASE + 0x8000 * 4), 0xA6FD);
    out16((IMX_USB3_PHY_BASE + 0x8001 * 4), 0xA6FD);
    out16((IMX_USB3_PHY_BASE + 0x8002 * 4), 0xA410);
    out16((IMX_USB3_PHY_BASE + 0x8003 * 4), 0x2410);
    out16((IMX_USB3_PHY_BASE + 0x8006 * 4), 0x23FF);
    out16((IMX_USB3_PHY_BASE + 0x8007 * 4), 0x2010);
    out16((IMX_USB3_PHY_BASE + 0x4058 * 4), 0x0020);
    out16((IMX_USB3_PHY_BASE + 0x41E7 * 4), 0x00FF);
    out16((IMX_USB3_PHY_BASE + 0x80E3 * 4), 0x0002);
    out16((IMX_USB3_PHY_BASE + 0x8090 * 4), 0x0013);
    out16((IMX_USB3_PHY_BASE + 0x8058 * 4), 0x0000);
    out16((IMX_USB3_PHY_BASE + 0x81DC * 4), 0x1004);
    out16((IMX_USB3_PHY_BASE + 0x81DF * 4), 0x4041);
    out16((IMX_USB3_PHY_BASE + 0x81F5 * 4), 0x0480);
    out16((IMX_USB3_PHY_BASE + 0x81D3 * 4), 0x8006);
    out16((IMX_USB3_PHY_BASE + 0x81C7 * 4), 0x003F);
    out16((IMX_USB3_PHY_BASE + 0x81C2 * 4), 0x543F);
    out16((IMX_USB3_PHY_BASE + 0x81C1 * 4), 0x543F);
    out16((IMX_USB3_PHY_BASE + 0x81C9 * 4), 0x0000);
    out16((IMX_USB3_PHY_BASE + 0x81F8 * 4), 0x8000);
    out16((IMX_USB3_PHY_BASE + 0x81F9 * 4), 0x0003);
    out16((IMX_USB3_PHY_BASE + 0x81DD * 4), 0x2408);
    out16((IMX_USB3_PHY_BASE + 0x81D5 * 4), 0x05CA);
    out16((IMX_USB3_PHY_BASE + 0x81E1 * 4), 0x0258);
    out16((IMX_USB3_PHY_BASE + 0x81BF * 4), 0x1FFF);
    out16((IMX_USB3_PHY_BASE + 0x4002 * 4), 0x02C6);
    out16((IMX_USB3_PHY_BASE + 0x4004 * 4), 0x0002);
    out16((IMX_USB3_PHY_BASE + 0x4003 * 4), 0x02C6);
    out16((IMX_USB3_PHY_BASE + 0x4005 * 4), 0x0010);
    out16((IMX_USB3_PHY_BASE + 0x4006 * 4), 0x0010);
    out16((IMX_USB3_PHY_BASE + 0x4007 * 4), 0x0010);
    out16((IMX_USB3_PHY_BASE + 0x4008 * 4), 0x0010);
    out16((IMX_USB3_PHY_BASE + 0x4009 * 4), 0x0010);
    out16((IMX_USB3_PHY_BASE + 0x400A * 4), 0x0002);
    out16((IMX_USB3_PHY_BASE + 0x400B * 4), 0x0002);
    out16((IMX_USB3_PHY_BASE + 0x400C * 4), 0x0002);
    out16((IMX_USB3_PHY_BASE + 0x400D * 4), 0x0002);
    out16((IMX_USB3_PHY_BASE + 0x400E * 4), 0x0002);
    out16((IMX_USB3_PHY_BASE + 0x400F * 4), 0x0002);
    out16((IMX_USB3_PHY_BASE + 0x4122 * 4), 0x0FFF);
    out16((IMX_USB3_PHY_BASE + 0x4123 * 4), 0x01E0);
    out16((IMX_USB3_PHY_BASE + 0x40F2 * 4), 0x0090);
    out16((IMX_USB3_PHY_BASE + 0x4102 * 4), 0x509B);
    out16((IMX_USB3_PHY_BASE + 0x41F5 * 4), 0x0003);
}

/**
 * This function configures pin muxing, enables VBUS and calls controller initialization method for OTG1 controller.
 */
void imx_usb2_otg1_host_init(void)
{
    /*
     * USB2.0 OTG1 INIT NON-CORE REGISTERS
     */

    /* Set the USB 5V control switch power polarity high-active */
    out32(IMX_USB2OTG1_BASE + IMX_USBNC_OTG_CTRL1,
          (in32(IMX_USB2OTG1_BASE + IMX_USBNC_OTG_CTRL1) | IMX_USBNC_OTG_CTRL1_PWR_POL_MASK));

    /* Set the USB 5V control switch over current polarity low-active*/
    out32(IMX_USB2OTG1_BASE + IMX_USBNC_OTG_CTRL1,
          (in32(IMX_USB2OTG1_BASE + IMX_USBNC_OTG_CTRL1) | IMX_USBNC_OTG_CTRL1_OVER_CUR_POL_MASK));

#if 0
    /* disable external charge detect to improve signal quality */
    out32(IMX_USB2PHY1_BASE + IMX_USB_ANALOG_CHRG_DETECT, (IMX_USB_ANALOG_CHRG_DETECT_EN_B_MASK |
                                                              IMX_USB_ANALOG_CHRG_DETECT_CHK_CHRG_B_MASK));
#endif

    /*
     * USB2.0 OTG1 INIT PLL Clock
     */
    init_usb2_pll(IMX_USB2PHY1_BASE);

    /*
     * USB2.0 OTG1 INIT CORE REGISTERS
     */
    /* Stop OTG controller core */
    out32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD, in32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD) & ~IMX_USB_USBCMD_RS_MASK);
    while (in32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD) & IMX_USB_USBCMD_RS_MASK);

    /* Reset OTG controller core */
    out32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD, in32(IMX_USB2OTG1_BASE) | IMX_USB_USBCMD_RST_MASK);
    while (in32(IMX_USB2OTG1_BASE + IMX_USB_USBCMD) & IMX_USB_USBCMD_RST_MASK);

    /*
     * USB2.0 OTG1 INIT PHY
     */
    init_usb2_phy(IMX_USB2PHY1_BASE);
}

/**
 * This function configures controller core and calls controller initialization method for OTG2 controller.
 */
void imx_usb3_otg2_host_init(void)
{
    uint32_t reg_value = 0;
    uint32_t cnt = 0;

    /*
     * Wait until phy clocks are valid
     */
    do {
        reg_value = in32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_SSPHY_STATUS_OFFSET);
        imx_usleep(10);

    } while ((reg_value & IMX_USB3_NON_CORE_SSPHY_STATUS_ALLCLKS_VLD_MASK) !=
            IMX_USB3_NON_CORE_SSPHY_STATUS_ALLCLKS_VLD_MASK);

    /*
     * USB2.0 OTG1 INIT NON-CORE REGISTERS
     */

    /* Set XHCI mode and disable overcurrent */
    reg_value = in32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET);    /* Read */
    reg_value &= (uint32_t)~IMX_USB3_NON_CORE_CORE_CTRL11_MODE_STRAP_MASK;          /* Clear mode bits */
    reg_value |= (uint32_t)(IMX_USB3_NON_CORE_CORE_CTRL11_MODE_STRAP_MASK & 0b010); /* Set XHCI mode */
    reg_value |= (uint32_t)IMX_USB3_NON_CORE_CORE_CTRL11_OVERCURRENT_DISABLE_MASK;  /* Set OverCurrent disabled */
    out32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET, reg_value);    /* Write */

    /* Clear PHY APB reset */
    reg_value = in32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET);    /* Read */
    reg_value &= (uint32_t)~IMX_USB3_NON_CORE_CORE_CTRL11_PHYAPB_SW_RESET_MASK;     /* PHY APB SW Reset disable */
    out32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET, reg_value);    /* Write */

    /*
     * USB3.0 OTG2 INIT PHY
     */
    init_usb3_phy(IMX_USB3_PHY_BASE);

    /*
     * USB3.0 OTG2 INIT NON-CORE INTERRUPT REGISTERS
     */
    reg_value = in32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_INT_OFFSET);            /* Read */
    reg_value |= 0x0000001;                                                         /* Enable XHCI interrupt */
    out32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_INT_OFFSET, reg_value);            /* Write */

    /*
     * USB3.0 OTG2 - CLEAR ALL RESETS
     */
    reg_value = in32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET);    /* Read */
    reg_value &= ~(IMX_USB3_NON_CORE_CORE_CTRL11_PWR_SW_RESET_MASK |
                   IMX_USB3_NON_CORE_CORE_CTRL11_APB_SW_RESET_MASK |
                   IMX_USB3_NON_CORE_CORE_CTRL11_AXI_SW_RESET_MASK |
                   0x10000000                                      |
                   IMX_USB3_NON_CORE_CORE_CTRL11_PHY_SW_RESET_MASK |
                   IMX_USB3_NON_CORE_CORE_CTRL11_PHYAPB_SW_RESET_MASK);
    out32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_CTRL11_OFFSET, reg_value);    /* Write */

    /* Wait for XHCI_POWER_ON_READY */
    cnt = 10000;
    do {
        reg_value = in32(IMX_USB3_CTRL_BASE + IMX_USB3_NON_CORE_CORE_STATUS_OFFSET);        /* Load NON_CORE_STATUS register */
        imx_usleep(100);
        cnt--;
    } while (!(reg_value & IMX_USB3_NON_CORE_CORE_STATUS_XHCI_POWER_ON_READY_MASK) && (cnt > 0));
    if (cnt == 0) {
        crash("USB3 Init: Timeout waiting on XHCI_POWER_ON_READY");
    }

    /* Wait for CNR */
    cnt = 10000;
    do {
        reg_value = in32(IMX_USB3_CORE_BASE + IMX_USB3_USBSS_USBSTS_OFFSET);
        imx_usleep(100);
        cnt--;
    } while ((reg_value & IMX_USB3_USBSS_USBSTS_CNR_MASK) && (cnt > 0));
    if (cnt == 0) {
        crash("USB3 Init: Timeout waiting on Controller Not Ready");
    }
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/boards/imx8xp/imx_init_usb.c $ $Rev: 891625 $")
#endif
