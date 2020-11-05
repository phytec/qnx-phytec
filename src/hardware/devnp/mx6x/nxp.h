/*
 * $QNXLicenseC:
 * Copyright 2016, QNX Software Systems.
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

#ifndef	NXP_H
#define NXP_H

/* NXP TJA1100 Broadreach extended register set */

#define BASIC_CONTROL					0
	#define BC_RESET						(1 << 15)
	#define BC_RESET_MASK					(0x00008000U)
	#define BC_LOOPB						(1 << 14)
	#define BC_LOOPBACK_MASK				(0x00004000U)
	#define BC_SPEED_SEL					(1 << 13)
	#define BC_SPEED_SELECT_MASK_0			(0x00002000U)
	#define BC_AUTONEG_ENABLE_SHIFT			12
	#define BC_AUTONEG_ENABLE_MASK			(0x00001000U)
	#define BC_POWER_DOWN_SHIFT				11
	#define BC_POWER_DOWN_MASK				(0x00000800U)
	#define BC_ISOLATE_SHIFT				10
	#define BC_ISOLATE_MASK					(0x00000400U)
	#define BC_RE_AUTONEG_SHIFT				9
	#define BC_RE_AUTONEG_MASK				(0x00000200U)
	#define BC_DUPLEX						(1 << 8)
	#define BC_DUPLEX_MODE_MASK				(0x00000100U)
	#define BC_SPEED_SELECT_SHIFT_1			6
	#define BC_SPEED_SELECT_MASK_1			(0x00000040U)
	#define BC_SPEED_SELECT_SHIFT_OVLP		1
	#define BC_COLLISION_TEST_SHIFT			7
	#define BC_COLLISION_TEST_MASK			(0x00000080U)
	#define BC_UNIDIRECTIONAL_ENABLE_SHIFT	5
	#define BC_UNIDIRECTIONAL_ENABLE_MASK	(0x00000020U)

#define BASIC_STATUS					1

#define EXT_STATUS_REG					15

#define EXT_CONTROL_REG					17
	#define LINK_CONTROL				(1 << 15)
	#define LINK_CONTROL_SHIFT			(15)
	#define LINK_CONTROL_MASK			(0x00008000U)
	#define POWER_MODE_SHIFT			11
	#define POWER_MODE_MASK				(0x00007800U)
	#define NORMAL_MODE				0x3
	#define STANDBY_MODE				0xc
	#define SLEEP_REQUEST				0xb
	#define SLAVE_JITTER_TEST			(1 << 10)
	#define SLAVE_JITTER_TEST_MASK		(0x00000400U)
	#define TRAINING_RESTART			(1 << 9)
	#define TRAINING_RESTART_MASK		(0x00000200U)
	#define TEST_MODE_SHIFT				6
	#define TEST_MODE_MASK				(0x000001C0U)
	#define CABLE_TEST					(1 << 5)
	#define CABLE_TEST_SHIFT			(5)
	#define CABLE_TEST_MASK					(0x00000020U)
	#define LOOPBACK_MODE_SHIFT			3
	#define LOOPBACK_MODE_MASK			(0x00000018U)
	#define INTERNAL_LOOPBACK			0x1
	#define EXTERNAL_LOOPBACK			0x2
	#define REMOTE_LOOPBACK				0x3
	#define CONFIG_ENABLE				(1 << 2)
	#define CONFIG_EN_MASK				(0x00000004U)
	#define CONFIG_INH					(1 << 1)
	#define CONFIG_INH_MASK				(0x00000002U)
	#define WAKE_REQUEST				(1 << 0)
	#define WAKE_REQUEST_MASK			(0x00000001U)

#define CONFIG_REG_1									18
	#define MASTER_SLAVE								(1 << 15)
	#define MASTER_SLAVE_SHIFT							(15)
	#define MASTER_SLAVE_MASK							(0x00008000U)
	#define AUTO_OP										(1 << 14)
	#define AUTO_OP_SHIFT								(14)
	#define AUTO_OP_MASK								(0x00004000U)
	#define LINK_LENGTH_SHIFT							12
	#define LINK_LENGTH_MASK							(0x00003000U)
	#define TX_AMPLITUDE_SHIFT							10
	#define TX_AMPLITUDE_MASK							(0x00000C00U)
	#define AMP_500MV									0x0
	#define AMP_750MV									0x1
	#define AMP_1000MV									0x2
	#define AMP_1250MV									0x3
	#define MII_MODE_SHIFT								8
	#define MII_MODE_MASK								(0x00000300U)
	#define MII_MODE									0x0
	#define RMII_MODE_50								0x1
	#define RMII_MODE_25								0x2
	#define REV_MII_MODE								0x3
	#define MII_DRIVER									(1 << 7)
	#define MII_DRIVER_SHIFT							(7)
	#define MII_DRIVER_MASK								(0x00000080U)
	#define SLEEP_CONFIRM								(1 << 6)
	#define SLEEP_CONFIRM_SHIFT							(6)
	#define SLEEP_CONFIRM_MASK							(0x00000040U)
	#define LED_MODE_SHIFT								4
	#define LED_MODE_MASK								(0x00000030U)
	#define LED_ENABLE									(1 << 3)
	#define LED_ENABLE_SHIFT							(3)
	#define LED_ENABLE_MASK								(0x00000008U)
	#define CONFIG_WAKE									(1 << 2)
	#define CONFIG_WAKE_SHIFT							(2)
	#define CONFIG_WAKE_MASK							(0x00000004U)
	#define AUTO_PWD									(1 << 1)
	#define AUTO_PWD_SHIFT								(1)
	#define AUTO_PWD_MASK								(0x00000002U)
	#define LPS_ACTIVE									(1 << 0)
	#define LPS_ACTIVE_MASK								(0x00000001U)

#define CONFIG_REG_2										19
	#define PHY_ADDR_SHIFT									11
	#define SNR_AVERAGING_SHIFT								9
	#define SNR_AVERAGING_MASK								(0x00000600U)
	#define SNR_WLIMIT_SHIFT								6
	#define SNR_WLIMIT_MASK									(0x000001C0U)
	#define SNR_FAIL_LIMIT_SHIFT							3
	#define SNR_FAILLIMIT_MASK								(0x00000038U)
	#define JUMBO_ENABLE									(1 << 2)
	#define JUMBO_ENABLE_SHIFT								(2)
	#define JUMBO_ENABLE_MASK								(0x00000004U)
	#define SLEEP_REQ_TO_P4									0
	#define SLEEP_REQ_TO_1									1
	#define SLEEP_REQ_TO_4									2
	#define SLEEP_REQ_TO_16									3
	#define SLEEP_REQUEST_TO_MASK	(0x00000003U)

#define SYMBOL_ERR_CNT_REG				20

#define INT_STATUS_REG					21
	#define POW_ON					(1 << 15)
	#define WAKEUP					(1 << 14)
	#define WAKEUP_MASK				(0x00004000U)
	#define WUR_RECEIVED				(1 << 13)
	#define WUR_RECEIVED_MASK			(0x00002000U)
	#define LPS_RECEIVED				(1 << 12)
	#define LPS_RECEIVED_MASK			(0x00001000U)
	#define PHY_INIT_FAIL				(1 << 11)
	#define PHY_INIT_FAIL_MASK			(0x00000800U)
	#define LINK_STATUS_FAIL			(1 << 10)
	#define LINK_STATUS_FAIL_MASK  (0x00000400U)
	#define LINK_STATUS_UP				(1 << 9)
	#define LINK_STATUS_UP_MASK			(0x00000200U)
	#define SYM_ERR					(1 << 8)
	#define SYM_ERR_MASK			(0x00000100U)
	#define TRAINING_FAILED				(1 << 7)
	#define TRAINING_FAILED_MASK		(0x00000080U)
	#define SNR_WARNING				(1 << 6)
	#define SNR_WARNING_MASK		(0x00000040U)
	#define CONTROL_ERR				(1 << 5)
	#define CONTROL_ERROR_MASK		(0x00000020U)
	#define TXEN_CLAMPED				(1 << 4)
	#define TXEN_CLAMPED_MASK			(0x00000010U)
	#define UV_ERR					(1 << 3)
	#define UV_ERR_MASK				(0x00000008U)
	#define UV_RECOVERY				(1 << 2)
	#define UV_RECOVERY_MASK		(0x00000004U)
	#define TEMP_ERR				(1 << 1)
	#define TEMP_ERROR_MASK			(0x00000002U)
	#define SLEEP_ABORT				(1 << 0)
	#define SLEEP_ABORT_MASK		(0x00000001U)

#define PHY_INT_ERR	(PHY_INIT_FAIL | LINK_STATUS_FAIL | SYM_ERR | CONTROL_ERR | UV_ERR | TEMP_ERR)

#define INT_ENABLE_REG					22
	#define SLEEP_ABORT_MASK			(0x00000001U)
	#define TEMP_ERROR_SHIFT			(1)
	#define TEMP_ERROR_MASK				(0x00000002U)
	#define UV_RECOVERY_SHIFT			(2)
	#define UV_RECOVERY_MASK			(0x00000004U)
	#define UV_ERR_SHIFT				(3)
	#define UV_ERR_MASK					(0x00000008U)
	#define TXEN_CLAMPED_SHIFT			(4)
	#define TXEN_CLAMPED_MASK			(0x00000010U)
	#define CONTROL_ERROR_SHIFT			(5)
	#define CONTROL_ERROR_MASK			(0x00000020U)
	#define SNR_WARNING_SHIFT			(6)
	#define SNR_WARNING_MASK			(0x00000040U)
	#define TRAINING_FAILED_SHIFT		(7)
	#define TRAINING_FAILED_MASK		(0x00000080U)
	#define SYM_ERR_SHIFT				(8)
	#define SYM_ERR_MASK				(0x00000100U)
	#define LINK_STATUS_UP_SHIFT		(9)
	#define LINK_STATUS_UP_MASK			(0x00000200U)
	#define LINK_STATUS_FAIL_SHIFT		(10)
	#define LINK_STATUS_FAIL_MASK		(0x00000400U)
	#define PHY_INIT_FAIL_SHIFT			(11)
	#define PHY_INIT_FAIL_MASK			(0x00000800U)
	#define LPS_RECEIVED_SHIFT			(12)
	#define LPS_RECEIVED_MASK			(0x00001000U)
	#define WUR_RECEIVED_SHIFT			(13)
	#define WUR_RECEIVED_MASK			(0x00002000U)
	#define WAKEUP_SHIFT				(14)
	#define WAKEUP_MASK					(0x00004000U)
	#define PWON_SHIFT					(15)

#define COMM_STATUS_REG					23
	#define LINK_UP						(1 << 15)
	#define LINK_UP_SHIFT				(15)
	#define LINK_UP_MASK				(0x00008000U)
	#define TX_MODE_SHIFT				13
	#define TX_MODE_MASK				(0x00006000U)
	#define TX_DIS						0x0
	#define TX_SEND_N					0x1
	#define TX_SEND_I					0x2
	#define TX_SEND_Z					0x3
	#define LOC_RX_STATUS				(1 << 12)
	#define LOC_RCVR_STATUS_SHIFT		(12)
	#define LOC_RCVR_STATUS_MASK		(0x00001000U)
	#define REM_RX_STATUS				(1 << 11)
	#define REM_RCVR_STATUS_SHIFT		(11)
	#define REM_RCVR_STATUS_MASK		(0x00000800U)
	#define SCR_LOCKED					(1 << 10)
	#define SCR_LOCKED_SHIFT			(10)
	#define SCR_LOCKED_MASK				(0x00000400U)
	#define SSD_ERR						(1 << 9)
	#define SSD_ERROR_SHIFT				(9)
	#define SSD_ERROR_MASK				(0x00000200U)
	#define ESD_ERR						(1 << 8)
	#define ESD_ERROR_SHIFT				(8)
	#define ESD_ERROR_MASK				(0x00000100U)
	#define SNR_SHIFT					5
	#define SNR_MASK					(0x07 << SNR_SHIFT)
	#define RX_ERR						(1 << 4)
	#define RECEIVE_ERROR_SHIFT			(4)
	#define RECEIVE_ERROR_MASK			(0x00000010U)
	#define TX_ERR						(1 << 3)
	#define TRANSMIT_ERROR_SHIFT		(3)
	#define TRANSMIT_ERROR_MASK			(0x00000008U)
	#define PHY_STATE_IDLE				0x0
	#define PHY_STATE_INIT				0x1
	#define PHY_STATE_CONFIG			0x2
	#define PHY_STATE_OFFLINE			0x3
	#define PHY_STATE_ACTIVE			0x4
	#define PHY_STATE_ISOLATE			0x5
	#define PHY_CABLE_TEST				0x6
	#define PHY_TEST_MODE				0x7
#define COMM_STATUS_ERR	(SSD_ERR | ESD_ERR | RX_ERR | TX_ERR)

#define GEN_STATUS_REG					24
	#define INT_STATUS					(1 << 15)
	#define INT_STATUS_SHIFT			(15)
	#define PLL_LOCKED					(1 << 14)
	#define PLL_LOCKED_SHIFT			(14)
	#define PLL_LOCKED_MASK				(0x00004000U)
	#define LOCAL_WU					(1 << 13)
	#define LOCAL_WU_SHIFT				(13)
	#define LOCAL_WU_MASK				(0x00002000U)
	#define REMOTE_WU					(1 << 12)
	#define REMOTE_WU_SHIFT				(12)
	#define REMOTE_WU_MASK				(0x00001000U)
	#define DATA_DET_WU					(1 << 11)
	#define DATA_DET_WU_SHIFT			(11)
	#define DATA_DET_WU_MASK			(0x00000800U)
	#define EN_STATUS					(1 << 10)
	#define EN_STATUS_SHIFT				(10)
	#define EN_STATUS_MASK				(0x00000400U)
	#define RESET_STATUS				(1 << 9)
	#define RESET_STATUS_SHIFT			(9)
	#define RESET_STATUS_MASK			(0x00000200U)
	#define LINK_FAIL_CNT_SHIFT			3
	#define LINKFAIL_CNT_MASK			(0x000000F8U)

#define EXTERN_STATUS_REG				25
	#define UV_VDDA_3V3					(1 << 14)
	#define UV_VDDA_3V3_SHIFT			(14)
	#define UV_VDDA_3V3_MASK			(0x00004000U)
	#define UV_VDDD_1V8					(1 << 13)
	#define UV_VDDD_1V8_SHIFT			(13)
	#define UV_VDDD_1V8_MASK			(0x00002000U)
	#define UV_VDDA_1V8					(1 << 12)
	#define UV_VDDA_1V8_SHIFT			(12)
	#define UV_VDDA_1V8_MASK			(0x00001000U)
	#define UV_VDDIO					(1 << 11)
	#define UV_VDDIO_SHIFT				(11)
	#define UV_VDDIO_MASK				(0x00000800U)
	#define TEMP_HIGH					(1 << 10)
	#define TEMP_HIGH_SHIFT				(10)
	#define TEMP_HIGH_MASK				(0x00000400U)
	#define TEMP_WARN					(1 << 9)
	#define TEMP_WARN_SHIFT				(9)
	#define TEMP_WARN_MASK				(0x00000200U)
	#define SHORT_DETECT				(1 << 8)
	#define SHORT_DETECT_SHIFT			(8)
	#define SHORT_DETECT_MASK			(0x00000100U)
	#define OPEN_DETECT					(1 << 7)
	#define OPEN_DETECT_SHIFT			(7)
	#define OPEN_DETECT_MASK			(0x00000080U)
	#define POL_DETECT					(1 << 6)
	#define POLARITY_DETECT_SHIFT		(6)
	#define POLARITY_DETECT_MASK		(0x00000040U)
	#define INTL_DETECT					(1 << 5)
	#define INTERLEAVE_DETECT_SHIFT		(5)
	#define INTERLEAVE_DETECT_MASK		(0x00000020U)

#define LINK_FAIL_CNT_REG							26
#define LINK_FAIL_COUNTER_LOC_RCVR_COUNTER_SHIFT	(8)


// TJA1101 Registers
#define COMMON_CONFIG_REG				27
	#define AUTO_OP_TJA1101				(1 << 15)
	#define AUTO_OP_SHIFT_TJA1101			(15)
	#define AUTO_OP_MASK_TJA1101			(0x00008000U)
	#define CONFIG_INH_TJA1101			(1 << 5)
	#define CONFIG_INH_MASK_TJA1101			(0x00000020U)


#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/devnp/mx6x/nxp.h $ $Rev: 874784 $")
#endif
