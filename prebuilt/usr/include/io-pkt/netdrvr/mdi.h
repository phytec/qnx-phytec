/*
 * $QNXLicenseC:
 * Copyright 2014,2019 QNX Software Systems.
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

#ifndef	_NETDRVR_MDI_H_INCLUDED
#define	_NETDRVR_MDI_H_INCLUDED

#include <inttypes.h>
#include <signal.h>
#include <time.h>

/*
 * Our pulse.code value from the mdi->CallBackTimer in NTO. This timer is
 * used to monitor our PHY interfaces for changes every 3 seconds.
 */
#define	MDI_TIMER	0x0002

/*
 * A typical MDI Protocol Sequence looks like
 *	<PREAMBLE><ST><OP><PHYAD><REGAD><TA><DATA><IDLE>
 */
#define	MDI_ST		0x01
#define	MDI_READ_OP	0x02
#define	MDI_WRITE_OP	0x01
#define	MDI_PHYAD_MASK	0x1F
#define	MDI_REGAD_MASK	0x1F
#define	MDI_WRITE_TA	0x02

/* Media */
#define	MDI_10bT	(1<<0)
#define	MDI_10bTFD	(1<<1)
#define	MDI_100bT	(1<<2)
#define	MDI_100bTFD	(1<<3)
#define	MDI_100bT4	(1<<4)
#define	MDI_100bT2	(1<<5)
#define	MDI_100bT2FD	(1<<6)
#define MDI_UNKNOWN	(1<<7)
#define	MDI_FLOW	(1<<10)
#define	MDI_FLOW_ASYM	(1<<11)
#define	MDI_1000bT	(1<<12)
#define	MDI_1000bTFD	(1<<13)
#define	MDI_1000bX	(1<<14)
#define	MDI_1000bXFD	(1<<15)

/* Basic Mode Control Register */
#define MDI_BMCR			0x00
	#define BMCR_RESET		(1<<15)
	#define BMCR_LOOPBACK		(1<<14)
	#define BMCR_SPEED_100		(1<<13)
	#define BMCR_AN_ENABLE		(1<<12)
	#define BMCR_SLEEP		(1<<11)
	#define BMCR_ISOLATE		(1<<10)
	#define BMCR_RESTART_AN		(1<<9)
	#define BMCR_FULL_DUPLEX	(1<<8)
	#define BMCR_COLLISION_TEST	(1<<7)
	#define BMCR_SPEED_1000		(1<<6)

/* Basic Mode Status Register */
#define MDI_BMSR			0x01
	#define	BMSR_100bT4		(1<<15)
	#define	BMSR_100bXFD		(1<<14)
	#define	BMSR_100bX		(1<<13)
	#define	BMSR_10FD		(1<<12)
	#define	BMSR_10			(1<<11)
	#define	BMSR_100bT2FD		(1<<10)
	#define	BMSR_100bT2		(1<<9)
	#define BMSR_EXT_STATUS		(1<<8)
	#define BMSR_AN_COMPLETE	(1<<5)
	#define BMSR_REMOTE_FAULT	(1<<4)
	#define BMSR_AN_ABILITY		(1<<3)
	#define BMSR_LINK_STATUS	(1<<2)
	#define BMSR_JABBER_DETECT	(1<<1)
	#define BMSR_EXTENDED_CAP	(1<<0)

/* PHY Identifier  Registers */
#define	MDI_PHYID_1			0x02
#define	MDI_PHYID_2			0x03
	#define	PHYID_VENDOR(x)		((uint32_t)(((x) >> 10) & 0x3FFFFF))
	#define	PHYID_MODEL(x)		((uint8_t)(((x) >> 4) & 0x3F))
	#define	PHYID_REV(x)		((uint8_t)((x) & 0xF))

/* Auto-Negotiation Advertisement Register */
#define MDI_ANAR			0x04
	#define ANAR_NEXT_PAGE		(1<<15)
	#define ANAR_ACKNOWLEDGE	(1<<14)
	#define ANAR_REMOTE_FAULT	(1<<13)
	#define	ANAR_FLOW_ASYMMETRIC	(1<<11)
	#define	ANAR_FLOW_CONTROL	(1<<10)
	#define	ANAR_ADVERT_MSK		(0x1fe0)

	#define ANAR_100bT4		(1 << 9)
	#define ANAR_100bTFD		(1 << 8)
	#define ANAR_100bT		(1 << 7)
	#define ANAR_10bTFD		(1 << 6)
	#define ANAR_10bT		(1 << 5)

/* Auto-Negotiation Link Partner Ability Register */
#define MDI_ANLPAR			0x05
	#define ANLPAR_NEXT_PAGE	(1<<15)
	#define ANLPAR_ACKNOWLEDGE	(1<<14)
	#define ANLPAR_MSG_PAGE		(1<<13)
	#define	ANLPAR_ACKNOWLEDGE2	(1<<12)
	#define	ANLPAR_TOGGLE		(1<<11)
	#define	ANAR_ADVERT_MSK		(0x1fe0)

/* Auto-Negotiation Expansion Register */
#define	MDI_ANAE			0x06
	#define	ANAE_PAR_DETECT_FAULT	(1<<4)
	#define	ANAE_LP_NP_ABLE		(1<<3)
	#define	ANAE_NP_ABLE		(1<<2)
	#define	ANAE_PAGE_RX		(1<<1)
	#define	ANAE_LP_AN_ABLE		(1<<0)

/* Auto-Negotiation Next Page Transmit Register */
#define	MDI_ANPT			0x07
	#define	ANPT_NXT_PAGE		(1<<15)
	#define	ANPT_MSG_PAGE		(1<<13)
	#define	ANPT_ACK2		(1<<12)
	#define	ANPT_TOGGLE		(1<<11)

/* Auto-Negotiation Link Partner Next Page Register */
#define	MDI_ALPNP			0x08

/* MASTER-SLAVE Control Register (100Base-T2 and 1000Base-T only) */
#define	MDI_MSCR			0x09
	#define	MSCR_MANUAL_CONFIG	(1<<12)
	#define	MSCR_CONFIG_VALUE	(1<<11)
	#define	MSCR_PORT_TYPE		(1<<10)
	#define	MSCR_ADV_1000bTFD	(1<<9)
	#define	MSCR_ADV_1000bT		(1<<8)

/* MASTER-SLAVE Status Register (100Base-T2 and 1000Base-T only) */
#define	MDI_MSSR			0x0a
	#define	MSSR_CONFIG_FAULT	(1<<15)
	#define	MSSR_CONFIG_RESULT	(1<<14)
	#define	MSSR_LOCAL_RX_STATUS	(1<<13)
	#define	MSSR_REMOTE_RX_STATUS	(1<<12)
	#define	MSSR_LP_1000bTFD	(1<<11)
	#define	MSSR_LP_1000bT		(1<<10)

/* Extended Mode Status Register (GMII only) */
#define	MDI_EMSR			0x0f
	#define	EMSR_1000bXFD		(1<<15)
	#define	EMSR_1000bX		(1<<14)
	#define	EMSR_1000bTFD		(1<<13)
	#define	EMSR_1000bT		(1<<12)

/* Clause 45 extended register definitions */

/* MDIO Manageable Devices (MMDs). */
#define MDI_MMD_PMA_PMD		1	/* Physical Medium Attachment */
					/* Physical Medium Dependent */
#define MDI_MMD_WIS		2	/* WAN Interface Sublayer */
#define MDI_MMD_PCS		3	/* Physical Coding Sublayer */
#define MDI_MMD_PHY_XS		4	/* PHY Extender Sublayer */
#define MDI_MMD_DTE_XS		5	/* DTE Extender Sublayer */
#define MDI_MMD_TC		6	/* Transmission Convergence */
#define MDI_MMD_AN		7	/* Auto-Negotiation */
#define MDI_MMD_C22_EXT		29	/* Clause 22 extension */
#define MDI_MMD_VEND_1		30	/* Vendor specific 1 */
#define MDI_MMD_VEND_2		31	/* Vendor specific 2 */

/* PMA/PMD Control 1 */
#define	PMA_PMD_CTRL_1			0x00
	#define	CTRL_RESET			(1<<15)
	#define	CTRL_SPEED_100			(1<<13)
	#define	CTRL_LOW_POWER			(1<<11)
	#define	CTRL_SPEED_1000			(1<<6)
	#define	CTRL_10PASS_TS			(1<<2)
	#define	CTRL_LOOPBACK			(1<<0)

/* PMA/PMD Status 1 */
#define	PMA_PMD_STAT_1			0x01
	#define	STAT_FAULT			(1<<7)
	#define	STAT_RX_LINK_STAT		(1<<2)
	#define	STAT_LOW_POWER_ABIL		(1<<1)

/* PMA/PMD device identifier */
#define	PMA_PMD_PHYID_1			0x02
#define	PMA_PMD_PHYID_2			0x03

/* PMA/PMD speed ability */
#define	PMA_PMD_SPD_ABIL		0x04
	#define	SPD_10M_CAP			(1<<6)
	#define	SPD_100M_CAP			(1<<5)
	#define	SPD_1000M_CAP			(1<<4)
	#define	SPD_10PASS_TS_CAP		(1<<2)
	#define	SPD_2BASE_TL_CAP		(1<<1)
	#define	SPD_10G_CAP			(1<<0)

/* PMA/PMD devices in package */
#define	PMA_PMD_DIP_1			0x05
	#define	AUTO_NEG_PRES			(1<<7)
	#define	TC_PRES				(1<<6)
	#define	DTE_XS_PRES			(1<<5)
	#define	PHY_XS_PRES			(1<<4)
	#define	PCS_PRES			(1<<3)
	#define	WIS_PRES			(1<<2)
	#define	PMA_PMD_PRES			(1<<1)
	#define	CLAUSE_22_PRES			(1<<0)

#define	PMA_PMD_DIP_2			0x06
	#define	VEND_SPEC_2			(1<<15)
	#define	VEND_SPEC_1			(1<<14)
	#define	CLAUSE_22_EXT_PRES		(1<<13)

/* PMA/PMD control 2 */
#define	PMA_PMD_CTRL_2			0x07
	#define	CTRL_2_10BASE_T			0x0f
	#define	CTRL_2_100BASE_TX		0x0e
	#define	CTRL_2_1000BASE_KX		0x0d
	#define	CTRL_2_1000BASE_T		0x0c
	#define	CTRL_2_10GBASE_KR		0x0b
	#define	CTRL_2_10GBASE_KX4		0x0a
	#define	CTRL_2_10GBASE_T		0x09
	#define	CTRL_2_10GBASE_LRM		0x08
	#define	CTRL_2_10GBASE_SR		0x07
	#define	CTRL_2_10GBASE_LR		0x06
	#define	CTRL_2_10GBASE_ER		0x05
	#define	CTRL_2_10GBASE_LX4		0x04
	#define	CTRL_2_10GBASE_SW		0x03
	#define	CTRL_2_10GBASE_LW		0x02
	#define	CTRL_2_10GBASE_EW		0x01
	#define	CTRL_2_10GBASE_CX4		0x00

/* 10G PMA/PMD status 2 */
#define	PMA_PMD_10G_STAT_2		0x08
	#define	STAT_2_DEV_PRES			(2<<14)
	#define	STAT_2_DEV_NOT_PRES		(3<<14)
	#define	STAT_2_TX_FAULT_ABIL		(1<<13)
	#define	STAT_2_RX_FAULT_ABIL		(1<<12)
	#define	STAT_2_TX_FAULT			(1<<11)
	#define	STAT_2_RX_FAULT			(1<<10)
	#define	STAT_2_EXT_ABIL			(1<<9)
	#define	STAT_2_TX_DIS_ABIL		(1<<8)
	#define	STAT_2_10GBASE_SR_ABIL		(1<<7)
	#define	STAT_2_10GBASE_LR_ABIL		(1<<6)
	#define	STAT_2_10GBASE_ER_ABIL		(1<<5)
	#define	STAT_2_10GBASE_LX4_ABIL		(1<<4)
	#define	STAT_2_10GBASE_SW_ABIL		(1<<3)
	#define	STAT_2_10GBASE_LW_ABIL		(1<<2)
	#define	STAT_2_10GBASE_EW_ABIL		(1<<1)
	#define	STAT_2_PMA_LOOPBACK_ABIL	(1<<0)

/* 10G PMA/PMD transmit disable */
#define	PMA_PMD_10G_XMIT_DIS		0x09
	#define	PMD_TX_DIS_3			(1<<4)
	#define	PMD_TX_DIS_2			(1<<3)
	#define	PMD_TX_DIS_1			(1<<2)
	#define	PMD_TX_DIS_0			(1<<1)
	#define	GLOB_PMD_TX_DIS			(1<<0)

/* 10G PMD receive signal detect */
#define	PMD_10G_RX_SIG_DET		0x0a
	#define	PMD_RX_SIG_DET_3		(1<<4)
	#define	PMD_RX_SIG_DET_2		(1<<3)
	#define	PMD_RX_SIG_DET_1		(1<<2)
	#define	PMD_RX_SIG_DET_0		(1<<1)
	#define	GLOB_PMD_RX_DET			(1<<0)

/* 10G PMA/PMD extended ability register */
#define	PMA_PMD_10G_EXT_ABIL		0x0b
	#define	EXT_10BASE_T_ABIL		(1<<8)
	#define	EXT_100BASE_TX_ABIL		(1<<7)
	#define	EXT_1000BASE_KX_ABIL		(1<<6)
	#define	EXT_1000BASE_T_ABIL		(1<<5)
	#define	EXT_10GBASE_KR_ABIL		(1<<4)
	#define	EXT_10GBASE_KX4_ABIL		(1<<3)
	#define	EXT_10GBASE_T_ABIL		(1<<2)
	#define	EXT_10GBASE_LRM_ABIL		(1<<1)
	#define	EXT_10GBASE_CX4_ABIL		(1<<0)

/* accessing Clause 45 MMD registers with */
/* Clause 22 MDIO Register Access Method */
/* MMD Access Control register 13 */

#define	MMD_ACCESS_CTRL			0x0d
	#define MMD_CTRL_ADDRESS		(0<<14)
	#define MMD_CTRL_DATA			(1<<14)
	#define MMD_CTRL_DATA_INC_RW		(2<<14)
	#define MMD_CTRL_DATA_INC_W		(3<<14)

typedef union
{
	uint16_t u16;
	struct
	{
		uint16_t function : 2;
		uint16_t reserved_5_13 : 9;
		uint16_t devad : 5;
	} s;
} xmdi_control_t;

/* Clause 45 Address/Data register 14 */

#define	MMD_ADDR_DATA			0x0e

/* PMA/PMD package identifier */
#define	PMA_PMD_PKG_ID_1		0x0e
#define	PMA_PMD_PKG_ID_2		0x0f

/* Clause 45 register access methods */

/* Write sequence */
/*
   Write register 0x0d
   Bits [15:14] = Register function 0 = Address mode
   Bits [13:5] = Reserved - Write 0x0
   Bits [4:0] = Clause 45 DEVAD
   Write register 0x0e = Clause 45 Address
   Write register 0x0d
   Bits [15:14] = Register function 1 = Data mode
   Bits [13:5] = Reserved - Write 0x0
   Bits [4:0] = Clause 45 DEVAD
   Write register 0x0e = Data value
*/

/* Read sequence */
/*
   Write register 0x0d
   Bits [15:14] = Register function 0 = Address mode
   Bits [13:5] = Reserved - Write 0x0
   Bits [4:0] = Clause 45 DEVAD
   Write register 0x0e = Clause 45 Address
   Write register 0x0d
   Bits [15:14] = Register function 1 = Data mode
   Bits [13:5] = Reserved - Write 0x0
   Bits [4:0] = Clause 45 DEVAD
   Read register 0x0e = Data value
*/


#define	MDI_VENDOR_BASE			(0x10)

/* OUI IDs for PHY Manufacturers. */
#define	NATIONAL_SEMICONDUCTOR		0x00080017
	#define	DP_83840		0x00
		#define	DP_83840A	0x01
	#define	DP_83843		0x01

	#define	NS83840A_PAR		(MDI_VENDOR_BASE + 0x09)
	#define	NS83840A_DUPLEX_STAT	(1<<7)
	#define	NS83840A_SPEED_10	(1<<6)

	#define	NS83843_PHYSTS		(MDI_VENDOR_BASE)
	#define	NS83843_DUPLEX		(1<<2)
	#define	NS83843_SPEED_10	(1<<1)


#define	LEVEL_ONE			0x001E0400
	#define	LXT_9746		0x00

	#define	L9746_CHIP_STATUS	(MDI_VENDOR_BASE + 0x04)
	#define	L9746_LINK_STATUS	(1<<13)
	#define	L9746_DUPLEX_STAT	(1<<12)
	#define	L9746_SPEED_100		(1<<11)

#define	QUALITY_SEMICONDUCTOR		0x00006051
	#define	QS6612_HUGH		0x00
	#define	QS6612			0x01

	#define	QS6612_INT_MASK		(MDI_VENDOR_BASE + 0x0E)
	#define	QS6612_PHY_CONTROL	(MDI_VENDOR_BASE + 0x0F)
	#define	QS6612_GET_MODE(x)	(uint8_t)((x & (1<<4|1<<3|1<<2)) >> 2)
	#define	QS6612_AUTONEG		0x00
	#define	QS6612_10bT		0x01
	#define	QS6612_100bT		0x02
	#define	QS6612_100bT4		0x04
	#define	QS6612_10bTFD		0x05
	#define	QS6612_100bTFD		0x06
	#define	QS6612_ISOLATE		0x07

#define	ICS				0x0000057D
	#define	ICS1889			0x01
	#define	ICS1890			0x02
		#define	INTERNAL	0x00
		#define	ALPHA_1890	0x01
		#define	GEN_REL		0x02
		#define	J_RELEASE	0x03
	#define	ICS1890_QPOLL		(MDI_VENDOR_BASE + 0x01)
	#define	ICS1890_SPEED_100	(1<<15)
	#define	ICS1890_DUPLEX		(1<<14)

#define	INTEL				0x0000AA00
	#define	I82555			0x15
		#define	I82555_REV	0x00
	#define	I82555_SCTRL		(MDI_VENDOR_BASE + 0x00)
	#define	I82555_SPEED_100	(1<<1)
	#define	I82555_DUPLEX		(1<<0)

#define	DAVICOM				0x0000606E
	#define	DM9101			0x00
		#define	DM9101_REV	0x00
		#define	DM9101_AREV	0x01
	#define	DM_DSCR			(MDI_VENDOR_BASE + 0x00)
	#define	DM_DSCSR		(MDI_VENDOR_BASE + 0x01)
		#define	DSCSR_100FDX	(1<<15)
		#define	DSCSR_100TX	(1<<14)
		#define	DSCSR_10FDX	(1<<13)
		#define	DSCSR_10TX	(1<<12)
	#define	DM_10BTCSR		(MDI_VENDOR_BASE + 0x02)

#define	MYSON				0x0000C0B4
	#define	MTD972			0x00

#define	BROADCOM			0x00001018
    #define BM5202			0x21
    #define BM5202_SPEED_MASK   	(1 << 10 | 1 << 9 | 1 << 8 )
    #define BM5202_10BT			(0 << 10 | 0 << 9 | 1 << 8 )
    #define BM5202_10BTFDX		(0 << 10 | 1 << 9 | 0 << 8 )
    #define BM5202_100BTX		(0 << 10 | 1 << 9 | 1 << 8 )
    #define BM5202_100BT4		(1 << 10 | 0 << 9 | 0 << 8 )
    #define BM5202_100BTXFDX		(1 << 10 | 0 << 9 | 1 << 8 )

#define	BROADCOM2			0x0000d897
	#define BCM89810		0xc
	#define BCM54616		0x11

#define	BROADCOM3			0x2b8094
	#define BCM89811		0x2

#define BROADCOM4			0x180361
	#define BCM54220		0x18

#define MARVELL				0x5043
	#define ALASKA			0x1d

#define MARVELLX			0x0AC2
	#define MV88Q2110		0x18
		#define	GIGE_T1_STATUS_REG	0x0901
			#define	GIGE_T1_STATUS_LINKUP	(1 << 2)
		#define	AUTONEG_STATUS_REG	0x8001
			#define	AUTONEG_STATUS_LOCAL_RX	(1 << 13)
			#define AUTONEG_STATUS_REMOTE_RX	(1 << 12)
			#define AUTONEG_STATUS_RX	(AUTONEG_STATUS_LOCAL_RX | AUTONEG_STATUS_REMOTE_RX)


#define	LSILOGIC			0x5be
	#define	LSI80225		0x8
	#define	LSI_STATOUT		(MDI_VENDOR_BASE + 0x02)
		#define	LSI_100MB	(1<<7)
		#define	LSI_FD		(1<<6)

#define	TDK	0x0000c039
	#define TDK78Q2120		0x14
	/* TDK 78Q2110 has 3 specifice registers */
	#define TDK_MR16		0x10			/* Vendor Specific */
	#define TDK_MR17		0x11			/* Interrupt Control/Status */
	#define TDK_MR18		0x12			/* Diagnostic */
	#define TDK_MR18_DPLX		(1 << 11)		/* set = full duplex */
	#define TDK_MR18_RATE		(1 << 10)		/* set = 100 Base    */
	#define TDK_MR18_RXLOCK		(1 << 8)		/* set we have a locked signal */

#define KENDIN				0x885
	#define KSZ9021			0x21
	#define KSZ9031			0x22
	#define KSZ8051			0x15
	#define KSZ8081			0x16

#define ATHEROS				0x1374
	#define AR8031			0x7

#define	NXP					0x6037
	#define	TJA1100			0x6
	#define	TJA1100_1		0x4
	#define TJA1101			0x10

#define TEXAS_INSTRUMENTS	0x80028
	#define DP83TC811		0x25
	#define DP83867			0x23

#define MICROSEMI			0x1C1
	#define VSC8531			0x17



/*
 * Determines if the MASTER-SLAVE registers (9 and 10) are valid, based
 * on the media supported by the PHY.
 */
#define MDI_MS_VALID(__media) \
    ((__media) & (MDI_100bT2 | MDI_100bT2FD | \
    MDI_1000bT | MDI_1000bTFD | MDI_1000bX | MDI_1000bXFD))

/**********************************************************
 ** Structures.
 **********************************************************/
struct mdi;
typedef uint16_t	(*MDIReadGen)(void *, uint8_t, int);
typedef void		(*MDIWriteGen)(void *, uint8_t, int, uint16_t);
typedef uint16_t	(*MDIReadFunc)(void *, uint8_t, uint8_t);
typedef void		(*MDIWriteFunc)(void *, uint8_t, uint8_t, uint16_t);
typedef void		(*MDICallBack)(void *, uint8_t, uint8_t);
typedef uint16_t	(*MDIReadFuncCl45)(void *, uint8_t, uint16_t, uint16_t);
typedef void		(*MDIWriteFuncCl45)(void *, uint8_t, uint16_t, uint16_t, uint16_t);

#define			MDI_CALLBACKTIME	0x3L	/* 3 Seconds */

typedef	int	(*PHYGetMedia)(struct mdi *mdi, int PhyAddr, int *Media);
typedef void	(*PHYDumpVendor)(struct mdi *mdi, int PhyAddr);
typedef void	(*PHYResetComplete)(struct mdi *mdi, int PhyAddr);
typedef	void	(*PHYSpaceHolder)(struct mdi *mdi, int PhyAddr);
typedef enum	{ WaitBusy = 0, NoWait, IrqNoWait } MDIWaitType;
typedef enum	{ MDI_WaitBusy = 0, MDI_NoWait, MDI_IrqNoWait } MDI_WaitType;


typedef struct {
	PHYGetMedia	GetMedia;
	PHYDumpVendor	DumpVendor;
	PHYResetComplete ResetComplete;
	PHYSpaceHolder	Holder1;
	PHYSpaceHolder	Holder2;

	char		*Desc;
	uint32_t	VendorOUI;
	uint8_t		Model;
	uint8_t		Rev;

	uint8_t		Cnt;
	uint8_t		Padding1;
	uint16_t	CurAdvert;
	uint8_t		Padding[2];

	uint16_t	Control;
	uint16_t	StatusReg;

	uint16_t	SetSpeed;
	uint16_t	SetAdvert;
	uint8_t		SetDuplex;
	uint8_t		CurrState;
	uint8_t		Padding2[2];
	int			MediaCapable;
} PhyData_t;

typedef struct mdi {
	PhyData_t	*PhyData[32];
	void	   	*handle;		/* Driver-private handle */

	MDIReadGen	Read;
	MDIWriteGen	Write;
	MDIReadFunc	ReadLeg;
	MDIWriteFunc	WriteLeg;
	MDIReadFuncCl45	ReadCl45;
	MDIWriteFuncCl45	WriteCl45;
	MDICallBack	CallBack;
	timer_t		CallBackTimer;
	struct itimerspec	CBTimer;
	uint8_t		DisableMonitor;
	uint8_t		LDownTest;
	uint8_t		Padding[2];
	uint16_t	Page;
} mdi_t;

/**********************************************************
 ** Return Codes.
 **********************************************************/

/* MDI Return Codes */
#define	MDI_FAILURE		0
#define	MDI_SUCCESS		1

#define	MDI_LINK_UP		2
#define	MDI_LINK_DOWN		3
#define	MDI_LINK_UNKNOWN	4
#define	MDI_RESET_PHY		5
#define	MDI_AUTONEG		6

#define	MDI_BADPARAM		9
#define	MDI_NOMEM		10
#define	MDI_UNSUPPORTED		11
#define	MDI_INVALID_MEDIA	12
#define	MDI_PROXY_FAILURE	13
#define	MDI_TIMER_FAILURE	14

/**********************************************************
 ** Public Interface.
 **********************************************************/
int MDI_Register(void *handle,
    MDIWriteFunc write, MDIReadFunc read, MDICallBack callback,
    mdi_t **mdi, struct sigevent *event);
int MDI_Register_Extended (void *handle, MDIWriteFunc write, MDIReadFunc read,
     MDICallBack callback, mdi_t **mdi, struct sigevent *event,
     int priority, int callback_interval);
int MDI_RegisterCl45(void *handle,
    MDIWriteFuncCl45 write, MDIReadFuncCl45 read, MDICallBack callback,
    mdi_t **mdi, struct sigevent *event);
int MDI_Register_ExtendedCl45 (void *handle, MDIWriteFuncCl45 write, MDIReadFuncCl45 read,
     MDICallBack callback, mdi_t **mdi, struct sigevent *event,
     int priority, int callback_interval);
void MDI_DeRegister(mdi_t **mdi);
int MDI_FindPhy(mdi_t *mdi, int PhyAddr);
int MDI_InitPhy(mdi_t *mdi, int PhyAddr);
int MDI_GetLinkStatus(mdi_t *mdi, int PhyAddr);
int MDI_GetActiveMedia(mdi_t *mdi, int PhyAddr, int *Media);
int MDI_SyncPhy(mdi_t *mdi, int PhyAddr);
int MDI_IsolatePhy(mdi_t *mdi, int PhyAddr);
int MDI_DeIsolatePhy(mdi_t *mdi, int PhyAddr);
int MDI_PowerdownPhy(mdi_t *mdi, int PhyAddr);
int MDI_PowerupPhy(mdi_t *mdi, int PhyAddr);
int MDI_ResetPhy(mdi_t *mdi, int PhyAddr, MDIWaitType Wait);
int MDI_SetSpeedDuplex(mdi_t *mdi, int PhyAddr, int Speed, int Duplex);
int MDI_SetAdvert(mdi_t *mdi, int PhyAddr, int MediaCode);
int MDI_GetAdvert(mdi_t *mdi, int PhyAddr, int *Advert);
int MDI_GetMediaCapable(mdi_t *mdi, int PhyAddr, int *Advert);
int MDI_GetPartnerAdvert(mdi_t *mdi, int PhyAddr, int *Advert);
int MDI_AutoNegotiate (mdi_t *mdi, int PhyAddr, int timeout);
int MDI_GetCallBackpid(mdi_t *mdi, pid_t *pid);
int MDI_DisableMonitor(mdi_t *mdi);
int MDI_EnableMonitor(mdi_t *mdi, int LDownTest);
void MDI_MonitorPhy(mdi_t *mdi);
int MDI_DumpRegisters(mdi_t *mdi, int PhyAddr);
int MDI_Autonegotiated_Active_Media(mdi_t *mdi, int PhyAddr, int *Media);

/******************************************************************************
 ** Specific PHY Support Included.
 *****************************************************************************/

/* Broadcom BM5202 */
int GetMedia_BM5202(mdi_t *mdi, int PhyAddr, int *Media);
void DumpVendor_BM5202(mdi_t *mdi, int PhyAddr);
void ResetComplete_BM5202(mdi_t *mdi, int PhyAddr);

/* National SemiConductor DP83840 */
int GetMedia_DP83840A(mdi_t *mdi, int PhyAddr, int *Media);
void DumpVendor_DP83840A(mdi_t *mdi, int PhyAddr);
void ResetComplete_DP83840A(mdi_t *mdi, int PhyAddr);

/* National SemiConductor DP83843 */
int GetMedia_DP83843(mdi_t *mdi, int PhyAddr, int *Media);
void DumpVendor_DP83843(mdi_t *mdi, int PhyAddr);

/* Level One LXT9746 */
int GetMedia_LXT9746(mdi_t *mdi, int PhyAddr, int *Media);
void DumpVendor_LXT9746(mdi_t *mdi, int PhyAddr);

/* Quality SemiConductor QS6612 */
int GetMedia_QS6612(mdi_t *mdi, int PhyAddr, int *Media);
void DumpVendor_QS6612(mdi_t *mdi, int PhyAddr);

/* Integrated Circuit Systems ICS1890 */
int GetMedia_ICS1890(mdi_t *mdi, int PhyAddr, int *Media);
void DumpVendor_ICS1890(mdi_t *mdi, int PhyAddr);
void ResetComplete_ICS1890(mdi_t *mdi, int PhyAddr);

/* INTEL Corp. I82555 */
int GetMedia_I82555(mdi_t *mdi, int PhyAddr, int *Media);
void DumpVendor_I82555(mdi_t *mdi, int PhyAddr);
void ResetComplete_I82555(mdi_t *mdi, int PhyAddr);

/* Davicom Inc. DM9101 */
int GetMedia_DM9101(mdi_t *mdi, int PhyAddr, int *Media);
void DumpVendor_DM9101(mdi_t *mdi, int PhyAddr);
void ResetComplete_DM9101(mdi_t *mdi, int PhyAddr);

/* LSI Logic. 80225 */
int GetMedia_LSI80225(mdi_t *mdi, int PhyAddr, int *Media);

/* TDK 78Q2120 */
void DumpVendor_TDK78Q2120(mdi_t *mdi, int PhyAddr);

#endif /* _NETDRVR_MDI_H_INCLUDED */

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/lib/io-pkt/sys/lib/libnetdrvr/public/netdrvr/mdi.h $ $Rev: 888088 $")
#endif
