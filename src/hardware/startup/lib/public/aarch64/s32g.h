/*
 * $QNXLicenseC:
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
 * NXP S32G SOCs:
 * S32G - 2x2 (2 cores per cluster) Cortex-A53 based SOC
 */

#ifndef	__AARCH64_S32G_H_INCLUDED
#define	__AARCH64_S32G_H_INCLUDED


/*
 * System Memory Map
 */

/*
 * LINFlexD (used for LIN and UART/serial communication)
 */
#define S32G_LINFLEXD0_BASE					0x401C8000
#define S32G_LINFLEXD0_IRQ					114
#define S32G_LINFLEXD1_BASE					0x401CC000
#define S32G_LINFLEXD1_IRQ					115

#define S32G_LINFLEXD0_SIZE					0x3000

/* Clock Generation Module */
#define S32G_MC_CGM0_BASE					0x40030000
#define S32G_MC_CGM1_BASE					0x40034000

#define S32G_MC_CGM0_PCS_SDUR					0x703
#define S32G_MC_CGM0_PCS_DIVC1					0x704
#define S32G_MC_CGM0_PCS_DIVE1					0x708
#define S32G_MC_CGM0_PCS_DIVS1					0x70C
#define S32G_MC_CGM0_PCS_DIVC2					0x710
#define S32G_MC_CGM0_PCS_DIVE2					0x714
#define S32G_MC_CGM0_PCS_DIVS2					0x718
#define S32G_MC_CGM0_DIV_UPD_TYPE				0x7D4
#define S32G_MC_CGM0_DIV_UPD_TRIG				0x7D8
#define S32G_MC_CGM0_DIV_UPD_STAT				0x7DC
#define S32G_MC_CGM0_SC_SS					0x7E4
#define S32G_MC_CGM0_SC_DC0					0x7E8
#define S32G_MC_CGM0_SC_DC1					0x7EC
#define S32G_MC_CGM0_SC_DC2					0x7F0
#define S32G_MC_CGM0_ACn_SC(n)					(0x800+(n*0x20))	// select current clock source
#define S32G_MC_CGM0_ACn_SS(n)					(0x804+(n*0x20))	// read current clock source
#define S32G_MC_CGM0_ACn_DC0(n)					(0x808+(n*0x20))	// divider reg 0
	#define S32G_MC_CGM0_ACn_DE				(0x1 << 31)	// Divider Enable
	#define S32G_MC_CGM0_AC0_DIV_OFFSET			16
	#define S32G_MC_CGM0_AC0_DIV_MASK			(0x3 << 16)
	#define S32G_MC_CGM0_AC1_DIV_OFFSET			16
	#define S32G_MC_CGM0_AC1_DIV_MASK			(0x3 << 16)
	#define S32G_MC_CGM0_AC2_DIV_OFFSET			16
	#define S32G_MC_CGM0_AC2_DIV_MASK			(0x3 << 16)
	#define S32G_MC_CGM0_AC3_DIV_OFFSET			16
	#define S32G_MC_CGM0_AC3_DIV_MASK			(0x3 << 16)
	#define S32G_MC_CGM0_AC4_DIV_OFFSET			16
	#define S32G_MC_CGM0_AC4_DIV_MASK			(0xf << 16)
	#define S32G_MC_CGM0_AC5_DIV_OFFSET			16
	#define S32G_MC_CGM0_AC5_DIV_MASK			(0x3 << 16)
	#define S32G_MC_CGM0_AC5_DIV1_OFFSET			16
	#define S32G_MC_CGM0_AC5_DIV1_MASK			(0x3 << 16)
	#define S32G_MC_CGM0_AC6_DIV_OFFSET			16
	#define S32G_MC_CGM0_AC6_DIV_MASK			(0x3 << 16)
	#define S32G_MC_CGM0_AC7_DIV_OFFSET			16
	// No AC7 DIV0?
	#define S32G_MC_CGM0_AC7_DIV1_MASK			(0x1f << 16)
	#define S32G_MC_CGM0_AC8_DIV_OFFSET			16
	#define S32G_MC_CGM0_AC8_DIV_MASK			(0x3 << 16)
	#define S32G_MC_CGM0_AC8_DIV1_OFFSET			16
	#define S32G_MC_CGM0_AC8_DIV1_MASK			(0xf << 16)
	// ...

#define S32G_MC_CGM0_ACn_DC1(n)					(0x80C+(n*0x20))	// divider reg 1 (not available for all aux clocks!)

/* Mode Entry Module - controls chip mode and mode transition sequences */
#define S32G_MC_ME_BASE						0x40088000
#define S32G_MC_ME_SIZE						0x1000
	#define S32G_MC_ME_CTL					0x0
	#define S32G_MC_ME_MODE					0x4
	#define S32G_MC_ME_UPD					0x8
	#define S32G_MC_ME_STAT					0xC
	#define S32G_MC_ME_COREID				0x10


/*
 * GMAC Controller
 */
#define S32G_ENET_BASE						0x4033C000
#define S32G_ENET_IRQ						89

#define S32G_ENET_SIZE						0x2000

/*
 * SWT
 */

#define S32G_SWT0_BASE 0x40100000
#define S32G_SWT1_BASE 0x40104000
#define S32G_SWT2_BASE 0x40108000
#define S32G_SWT3_BASE 0x4010C000
#define S32G_SWT4_BASE 0x40200000
#define S32G_SWT5_BASE 0x40204000
#define S32G_SWT6_BASE 0x40208000

#define S32G_SWT_SIZE	0x1000

#define SWT_CR		0x00	/* SWT Control Register */
    #define SWT_CR_FIXED_SS	(0 << 9)	/* -> Fixed Service Sequence */
    #define SWT_CR_KEYED_SS	(1 << 9)	/* -> Keyed Service Sequence */
    #define SWT_CR_RIA		(1 << 8)	/* -> Reset on Invalid Access */
    #define SWT_CR_WND		(1 << 7)	/* -> Window Mode */
    #define SWT_CR_ITR		(1 << 6)	/* -> Interrupt then reset */
    #define SWT_CR_HLK		(1 << 5)	/* -> Hard Lock */
    #define SWT_CR_SLK		(1 << 4)	/* -> Soft Lock */
    #define SWT_CR_STP		(1 << 2)	/* -> Stop Mode Control */
    #define SWT_CR_FRZ		(1 << 1)	/* -> Debug Mode Control */
    #define SWT_CR_WEN		(1 << 0)	/* -> Watchdog Enable */
#define SWT_IR		0x04	/* SWT Interrupt Register */
#define SWT_TO		0x08	/* SWT Timeout Register */
#define SWT_SR		0x10	/* SWT Service Register */
    #define WDT_SEQ1		0xA602	        /* -> service sequence 1 */
    #define WDT_SEQ2		0xB480	        /* -> service sequence 2 */
#define SWT_CO		0x14	/* SWT Counter Output Register */
#define SWT_SK		0x18	/* SWT Service Key Register */
#define SWT_RRR		0x1C	/* SWT Reset Request Register */
    #define SWT_RRR_RRF		(1 << 0)	/* -> Reset Request Flag */

/*
 * SIUL2
 */
#define S32G_SIUL2_BASE		0x4009C000
#define S32G_SIUL2_1_BASE	0x44010000

#define SIUL2_MIDR1		0x04
#define SIUL2_MIDR2		0x08
#define SIUL2_DISR0		0x10
#define SIUL2_DIRER0		0x18
#define SIUL2_DIRSR0		0x20
#define SIUL2_IREER0		0x28
#define SIUL2_IFEER0		0x30
#define SIUL2_IFER0		0x38
#define SIUL2_IFMCRn(i)		(0x40 + 4 * (i))
#define SIUL2_IFCPR		0xC0

/* SIUL2_MSCR specifications as stated in Reference Manual:
 * 0 - 359 Output Multiplexed Signal Configuration Registers
 * 512- 1023 Input Multiplexed Signal Configuration Registers */
#define SIUL2_MSCRn(i)		(0x240 + 4 * (i))
#define SIUL2_IMCRn(i)		(0xA40 +  4 * (i))
#define SIUL2_GPDOn(i)		(0x1300 + 4 * (i))
#define SIUL2_GPDIn(i)		(0x1500 + 4 * (i))
#define SIUL2_PGPDOn(i)		(0x1700 +  2 * (i))
#define SIUL2_PGPDIn(i)		(0x1740 + 2 * (i))
#define SIUL2_MPGPDOn(i)	(0x1780 + 4 * (i))

/* GPIO */
/* 163 GPIOs in output mode, we assume the GPIO number is in range */
#define SIUL2_GPDO_for_GPIO(i)		(((i) & (~0x3))>>2)
#define SIUL2_GPDO_PDO_off_for_GPIO(i)	(((i) & (0x3))
#define SIUL2_PDOn(i)			(SIUL2_GPDOn(SIUL2_GPDO_for_GPIO(i) + \
						SIUL2_GPDO_PDO_off_for_GPIO(i))
#define SIUL2_GPIO_VALUE0		(0x00)
#define SIUL2_GPIO_VALUE1		(0x01)

/* SIUL2_MSCR masks */
#define SIUL2_MSCR_DDR_DO_TRIM(v)	((v) & 0xC0000000)
#define SIUL2_MSCR_DDR_DO_TRIM_MIN	(0 << 30)
#define SIUL2_MSCR_DDR_DO_TRIM_50PS	(1 << 30)
#define SIUL2_MSCR_DDR_DO_TRIM_100PS	(2 << 30)
#define SIUL2_MSCR_DDR_DO_TRIM_150PS	(3 << 30)

#define SIUL2_MSCR_DDR_INPUT(v)		((v) & 0x20000000)
#define SIUL2_MSCR_DDR_INPUT_CMOS	(0 << 29)
#define SIUL2_MSCR_DDR_INPUT_DIFF_DDR	(1 << 29)

#define SIUL2_MSCR_DDR_SEL(v)		((v) & 0x18000000)
#define SIUL2_MSCR_DDR_SEL_DDR3		(0 << 27)
#define SIUL2_MSCR_DDR_SEL_LPDDR2	(2 << 27)

#define SIUL2_MSCR_DDR_ODT(v)		((v) & 0x07000000)
#define SIUL2_MSCR_DDR_ODT_120ohm	(1 << 24)
#define SIUL2_MSCR_DDR_ODT_60ohm	(2 << 24)
#define SIUL2_MSCR_DDR_ODT_40ohm	(3 << 24)
#define SIUL2_MSCR_DDR_ODT_30ohm	(4 << 24)
#define SIUL2_MSCR_DDR_ODT_24ohm	(5 << 24)
#define SIUL2_MSCR_DDR_ODT_20ohm	(6 << 24)
#define SIUL2_MSCR_DDR_ODT_17ohm	(7 << 24)

#define SIUL2_MSCR_DCYCLE_TRIM(v)	((v) & 0x00C00000)
#define SIUL2_MSCR_DCYCLE_TRIM_NONE	(0 << 22)
#define SIUL2_MSCR_DCYCLE_TRIM_LEFT	(1 << 22)
#define SIUL2_MSCR_DCYCLE_TRIM_RIGHT	(2 << 22)

#define SIUL2_MSCR_OBE(v)		((v) & 0x00200000)
#define SIUL2_MSCR_OBE_EN		(1 << 21)

#define SIUL2_MSCR_ODE(v)		((v) & 0x00100000)
#define SIUL2_MSCR_ODE_EN		(1 << 20)

#define SIUL2_MSCR_IBE(v)		((v) & 0x00010000)
#define SIUL2_MSCR_IBE_EN		(1 << 19)

#define SIUL2_MSCR_HYS(v)		((v) & 0x00400000)
#define SIUL2_MSCR_HYS_EN		(1 << 18)

#define SIUL2_MSCR_INV(v)		((v) & 0x00020000)
#define SIUL2_MSCR_INV_EN		(1 << 17)

#define SIUL2_MSCR_PKE(v)		((v) & 0x00010000)
#define SIUL2_MSCR_PKE_EN		(1 << 16)

#define SIUL2_MSCR_SRE(v)		((v) & 0x0000C000)
#define SIUL2_MSCR_SRE_SPEED_LOW_50	(0 << 14)
#define SIUL2_MSCR_SRE_SPEED_LOW_100	(1 << 14)
#define SIUL2_MSCR_SRE_SPEED_HIGH_100	(2 << 14)
#define SIUL2_MSCR_SRE_SPEED_HIGH_200	(3 << 14)


#define SIUL2_MSCR_PUE(v)		((v) & 0x00002000)
#define SIUL2_MSCR_PUE_EN		(1 << 13)

#define SIUL2_MSCR_PUS(v)		((v) & 0x00001800)
#define SIUL2_MSCR_PUS_100K_DOWN	(0 << 11)
#define SIUL2_MSCR_PUS_50K_UP		(1 << 11)
#define SIUL2_MSCR_PUS_100K_UP		(2 << 11)
#define SIUL2_MSCR_PUS_33K_UP		(3 << 11)

#define SIUL2_MSCR_DSE(v)		((v) & 0x00000700)
#define SIUL2_MSCR_DSE_240ohm		(1 << 8)
#define SIUL2_MSCR_DSE_120ohm		(2 << 8)
#define SIUL2_MSCR_DSE_80ohm		(3 << 8)
#define SIUL2_MSCR_DSE_60ohm		(4 << 8)
#define SIUL2_MSCR_DSE_48ohm		(5 << 8)
#define SIUL2_MSCR_DSE_40ohm		(6 << 8)
#define SIUL2_MSCR_DSE_34ohm		(7 << 8)

#define SIUL2_MSCR_CRPOINT_TRIM(v)	((v) & 0x000000C0)
#define SIUL2_MSCR_CRPOINT_TRIM_1	(1 << 6)

#define SIUL2_MSCR_SMC(v)		((v) & 0x00000020)
#define SIUL2_MSCR_MUX_MODE(v)		((v) & 0x0000000f)
#define SIUL2_MSCR_MUX_MODE_ALT0	(0x0)
#define SIUL2_MSCR_MUX_MODE_ALT1	(0x1)
#define SIUL2_MSCR_MUX_MODE_ALT2	(0x2)
#define SIUL2_MSCR_MUX_MODE_ALT3	(0x3)

/*
 * EDMA3
 */

#define S32G_EDMA0_BASE			0x40144000
#define S32G_EDMA0TX0_IRQ		40	/*EDMA tx channel 0~15 interrupt */
#define S32G_EDMA0TX1_IRQ		41	/*EDMA tx channel 16~31 interrupt */
#define S32G_EDMA0ERR_IRQ		42

#define S32G_EDMA1_BASE			0x40244000
#define S32G_EDMA1TX0_IRQ		43	/*EDMA tx channel 0~15 interrupt */
#define S32G_EDMA1TX1_IRQ		44	/*EDMA tx channel 16~31 interrupt */
#define S32G_EDMA1ERR_IRQ		45

#define S32G_EDMA_NUM_CH		32

#define S32G_DMAMUX0_BASE		0x4012C000
#define S32G_DMAMUX1_BASE		0x40130000
#define S32G_DMAMUX2_BASE		0x4022C000
#define S32G_DMAMUX3_BASE		0x40230000
#define S32G_DMAMUX_NUM_SLOTS		32


/*
 * CAN
 */
#define S32G_CAN0_PORT			0x401B8000
#define S32G_CAN0_MEM			0x40055080
#define S32G_CAN0_IRQ			74

#define S32G_CAN1_PORT			0x401BC000
#define S32G_CAN1_MEM			0x40055080
#define S32G_CAN1_IRQ			74

#define S32G_CAN2_PORT			0x401AC000
#define S32G_CAN2_MEM			0x400BE080
#define S32G_CAN2_IRQ			76

#define S32G_CAN3_PORT			0x401B0000
#define S32G_CAN3_MEM			0x40055080
#define S32G_CAN3_IRQ			74

#define S32G_CAN_SIZE			0x1000

/*
 * DSPI
 */
#define S32G_SPI0_BASE			0x401D4000
#define S32G_SPI0_IRQ			117
#define S32G_SPI1_BASE			0x401D8000
#define S32G_SPI1_IRQ			118
#define S32G_SPI2_BASE			0x401DC000
#define S32G_SPI2_IRQ			119
#define S32G_SPI3_BASE			0x402C8000
#define S32G_SPI3_IRQ			120
#define S32G_SPI4_BASE			0x402CC000
#define S32G_SPI4_IRQ			121
#define S32G_SPI5_BASE			0x402D0000
#define S32G_SPI5_IRQ			122

#define S32G_SPI_SIZE			0x140

/*
 * I2C Controllers
 */
#define S32G_I2C0_BASE			0x401E4000
#define S32G_I2C0_IRQ			124
#define S32G_I2C1_BASE			0x402E8000
#define S32G_I2C1_IRQ			125
#define S32G_I2C2_BASE			0x401EC000
#define S32G_I2C2_IRQ			126
#define S32G_I2C3_BASE			0x402D8000
#define S32G_I2C3_IRQ			127
#define S32G_I2C4_BASE			0x402DC000
#define S32G_I2C4_IRQ			128

#define S32G_I2C_SIZE			0x1000


#endif	/* __AARCH64_S32G_H_INCLUDED */


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
#ifdef __ASM__
__SRCVERSION "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/lib/public/aarch64/s32g.h $ $Rev: 870901 $"
#else
__SRCVERSION( "$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/startup/lib/public/aarch64/s32g.h $ $Rev: 870901 $" )
#endif
#endif
