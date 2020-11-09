/*
 * Header for the Texas Instruments DP83867 PHY
 *
 * Copyright (C) 2015 Texas Instruments Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#ifndef	DP83867_H
#define DP83867_H

#define DP83867_DEVADDR		0x1f

#define MII_DP83867_PHYCTRL	0x10
#define MII_DP83867_PHYSTS	0x11
#define MII_DP83867_MICR	0x12
#define MII_DP83867_ISR		0x13
#define MII_DP83867_LEDCR2	0x19
#define DP83867_CTRL		0x1f
#define DP83867_CFG3		0x1e

/* Extended Registers */
#define DP83867_CFG4            0x0031
#define DP83867_RGMIICTL	0x0032
#define DP83867_STRAP_STS1	0x006E
#define DP83867_RGMIIDCTL	0x0086
#define DP83867_IO_MUX_CFG	0x0170

#define DP83867_SW_RESET	(1 << 15)
#define DP83867_SW_RESTART	(1 << 14)

/* PHYCTRL bits */
#define MII_DP83867_PHYCTRL_FORCE_LINK_GOOD	(1 << 10)

#define DP83867_PHYCR_FIFO_DEPTH_3_B_NIB	0x00
#define DP83867_PHYCR_FIFO_DEPTH_4_B_NIB	0x01
#define DP83867_PHYCR_FIFO_DEPTH_6_B_NIB	0x02
#define DP83867_PHYCR_FIFO_DEPTH_8_B_NIB	0x03

/* MICR Interrupt bits */
#define MII_DP83867_MICR_AN_ERR_INT_EN		(1 << 15)
#define MII_DP83867_MICR_SPEED_CHNG_INT_EN	(1 << 14)
#define MII_DP83867_MICR_DUP_MODE_CHNG_INT_EN	(1 << 13)
#define MII_DP83867_MICR_PAGE_RXD_INT_EN	(1 << 12)
#define MII_DP83867_MICR_AUTONEG_COMP_INT_EN	(1 << 11)
#define MII_DP83867_MICR_LINK_STS_CHNG_INT_EN	(1 << 10)
#define MII_DP83867_MICR_FALSE_CARRIER_INT_EN	(1 << 8)
#define MII_DP83867_MICR_SLEEP_MODE_CHNG_INT_EN	(1 << 4)
#define MII_DP83867_MICR_WOL_INT_EN		(1 << 3)
#define MII_DP83867_MICR_XGMII_ERR_INT_EN	(1 << 2)
#define MII_DP83867_MICR_POL_CHNG_INT_EN	(1 << 1)
#define MII_DP83867_MICR_JABBER_INT_EN		(1 << 0)

/* LEDCR2 bits */
#define MII_DP83867_LEDCR2_LED_2_POLARITY	(1 << 10)
#define MII_DP83867_LEDCR2_LED_1_POLARITY	(1 << 6)
#define MII_DP83867_LEDCR2_LED_0_POLARITY	(1 << 2)

/* RGMIICTL bits */
#define DP83867_RGMII_EN			(7 << 1)
#define DP83867_RGMII_TX_CLK_DELAY_EN		(1 << 1)
#define DP83867_RGMII_RX_CLK_DELAY_EN		(1 << 0)

/* STRAP_STS1 bits */
#define DP83867_STRAP_STS1_RESERVED		(1 << 11)

/* PHY CTRL bits */
#define DP83867_PHYCR_FIFO_DEPTH_SHIFT		14
#define DP83867_PHYCR_FIFO_DEPTH_MASK		(3 << 14)
#define DP83867_PHYCR_RESERVED_MASK		(1 << 11)

/* RGMIIDCTL bits */
#define DP83867_RGMII_TX_CLK_DELAY_SHIFT	4

#define	DP83867_RGMIIDCTL_250_PS		0x0
#define	DP83867_RGMIIDCTL_500_PS		0x1
#define	DP83867_RGMIIDCTL_750_PS		0x2
#define	DP83867_RGMIIDCTL_1_NS			0x3
#define	DP83867_RGMIIDCTL_1_25_NS		0x4
#define	DP83867_RGMIIDCTL_1_50_NS		0x5
#define	DP83867_RGMIIDCTL_1_75_NS		0x6
#define	DP83867_RGMIIDCTL_2_00_NS		0x7
#define	DP83867_RGMIIDCTL_2_25_NS		0x8
#define	DP83867_RGMIIDCTL_2_50_NS		0x9
#define	DP83867_RGMIIDCTL_2_75_NS		0xa
#define	DP83867_RGMIIDCTL_3_00_NS		0xb
#define	DP83867_RGMIIDCTL_3_25_NS		0xc
#define	DP83867_RGMIIDCTL_3_50_NS		0xd
#define	DP83867_RGMIIDCTL_3_75_NS		0xe
#define	DP83867_RGMIIDCTL_4_00_NS		0xf

/* IO_MUX_CFG bits */
#define DP83867_IO_MUX_CFG_IO_IMPEDANCE_CTRL	0x1f

#define DP83867_IO_MUX_CFG_IO_IMPEDANCE_MAX	0x0
#define DP83867_IO_MUX_CFG_IO_IMPEDANCE_MIN	0x1f
#define DP83867_IO_MUX_CFG_CHA_TCLK		8
#define DP83867_IO_MUX_CFG_CLK_O_SEL_MASK	(0x1f << 8)
#define DP83867_IO_MUX_CFG_CLK_O_SEL_SHIFT	8

/* CFG4 bits */
#define DP83867_CFG4_PORT_MIRROR_EN              (1 << 0)
#define DP83867_CFG4_INT_TEST_MODE_1             (1 << 7)





#endif	/* defined DP83867_H */
