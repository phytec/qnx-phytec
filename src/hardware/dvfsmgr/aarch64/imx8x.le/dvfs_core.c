/*
 * $QNXLicenseC:
 * Copyright 2017, QNX Software Systems. All Rights Reserved.
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


#include "dvfs.h"
#include <sys/debug.h>
#include <sys/procfs.h>
#include <sys/imx8_sci_mgr.h>
#include <aarch64/mx8x_smc_call.h>
#include <drvr/hwinfo.h>


/*
 * Structure containing cpu related information
 */
typedef struct
{
	struct qtime_entry	*qtp;								// used to get system time
	uint32_t			chip_type;							// iMX8 chip type
	int					num_cpu;							// total number of active cores
	dvfs_pwrtbl_t		*cpu_pwr_lvl_tbl;					// pointer to cpu power table structure
	int					num_pwr_lvls;						// total number of power level
	uint32_t			cfglevel;							// current power configuratuon level
	char				cpu_clstr1_name[CHIP_STRING_SIZE];	// CPU cluster1 name string
	uint32_t			cpu_clstr1_src;						// CPU source for cluster1 CPU (A53 or A35)
	uint32_t			cpu_clstr1_freq;					// CPU frequency for cluster1 CPU
	char				cpu_clstr2_name[CHIP_STRING_SIZE];	// CPU cluster2 name string
	uint32_t			cpu_clstr2_src;						// CPU source for cluster2 CPU (A72)
	uint32_t			cpu_clstr2_freq;					// CPU frequency for cluster2 CPU
	uint32_t			soc_temp;							// SOC temprature
	uint32_t			cpuload;							// average CPU load
	uint32_t			load[MX8X_CPU_NUMBER_MAX];			// CPU load for all cores
	uint64_t			sutime[3][MX8X_CPU_NUMBER_MAX];		// CPU idle time for all cores
	int					sc_fd;								// file descriptor for system controller device
	char				sc_path[PATH_MAX];					// system controller path
	int					cl_fd;								// file descriptor for CPU load
	char				cl_path[PATH_MAX];					// CPU load process path
	int					smc_call_en;						// SMC call for ARM Trusted Firmware enabled
} dvfs_cpu_info;

/*
 * Local static variable Initialization
 */
static dvfs_cpu_info	cpu_info = {0};

#define MX8QM_PWR_TABLE_SIZE		4
#define MX8QXP_PWR_TABLE_SIZE		2

static	dvfs_pwrtbl_t	cpu_pwr_levels_iMX8QM_2400MHz[MX8QM_PWR_TABLE_SIZE] = {
	{
		.sys_pll_freq			= 1200000000,		// PLL cluster1 frequency, Unsupported
		.cpu_freq				= 1200000000,		// CPU cluster1 frequency
		.cpu_voltage			= 1100000,			// CPU cluster1 voltage, Unsupported
		.max_allowed_freq		= 0,		        // CPU cluster1 Max frequency
		.cpu_freq_cluster2		= 1596000000,		// CPU cluster2 frequency
		.efuse_val				= 0,				// Unsupported
		.opp_lvl				= PWR_OPP_HIGH,
	},
	{
		.sys_pll_freq			= 1104000000,
		.cpu_freq				= 1104000000,
		.cpu_voltage			= 1100000,
		.max_allowed_freq		= 0,
		.cpu_freq_cluster2		= 1296000000,
		.efuse_val				= 0,
		.opp_lvl				= PWR_OPP_HIGH,
	},
	{
		.sys_pll_freq			= 900000000,
		.cpu_freq				= 900000000,
		.cpu_voltage			= 1000000,
		.max_allowed_freq		= 0,
		.cpu_freq_cluster2		= 1056000000,
		.efuse_val				= 0,
		.opp_lvl				= PWR_OPP_OD,
	},
	{
		.sys_pll_freq			= 600000000,
		.cpu_freq				= 600000000,
		.cpu_voltage			= 900000,
		.max_allowed_freq		= 0,
		.cpu_freq_cluster2		= 600000000,
		.efuse_val				= 0,
		.opp_lvl				= PWR_OPP_NOM,
	}
};

static dvfs_pwrtbl_t cpu_pwr_levels_iMX8QXP_1200MHZ[MX8QXP_PWR_TABLE_SIZE] = {
	{
		.sys_pll_freq			= 1200000000,
		.cpu_freq				= 1200000000,
		.cpu_voltage			= MAX_SAFE_VOLT,
		.max_allowed_freq		= 0,
		.cpu_freq_cluster2		= 1200000000,
		.efuse_val				= 0,
		.opp_lvl				= PWR_OPP_HIGH,
	},
	{
		.sys_pll_freq			= 1000000000,
		.cpu_freq				= 1000000000,
		.cpu_voltage			= MAX_SAFE_VOLT,
		.max_allowed_freq		= 0,
		.cpu_freq_cluster2		= 1000000000,
		.efuse_val				= 0,
		.opp_lvl				= PWR_OPP_HIGH,
	}
};


/************************************************************
 * API Function
 ************************************************************/

/*
 * cpu_parse_options: SoC-specific option
 * - Description: This function handles platform specific options. If no
 * option is given, this function is still called with "" in order to initialize
 * cpu_info structure.
 * - Returns:
 *  - EFAULT if "use" option is used or unknown option is passed, EOK otherwise
 */
int cpu_parse_options(void *handle, char *options)
{
	dvfs_t			*dev = (dvfs_t *) handle;
	char			*value, *optstr, *freeptr, *c, *smc_value;
	unsigned		hwi_off;
	unsigned		tag_idx;
	unsigned		unit;
	hwi_tag			*tag;
	int				opt;
	int				ret = EOK;
	static	char	*opts[] = {
#define OPT_USE			0
						"use",
#define OPT_SC_DEV		1
						"sc_path",
#define OPT_CL_DEV		2
						"cl_path",
#define OPT_ATF_MODE_EN 3
						"smc_call",
						NULL
					};

	strlcpy(cpu_info.sc_path, MX8X_SC_DECVICE_DEFUALT, sizeof(cpu_info.sc_path));
	strlcpy(cpu_info.cl_path, MX8X_CL_DECVICE_DEFUALT, sizeof(cpu_info.cl_path));
	cpu_info.smc_call_en = 0;		// ARM trusted firmware disabled

	/* Getting the SMC call status from the Hwinfo Section if available */
	unit = 0;
	while((hwi_off = hwi_find_device("smc_call", unit)) != HWI_NULL_OFF) {
		tag_idx = 0;	//We don't need the device name in this case
		if((tag = hwi_tag_find(hwi_off, HWI_TAG_NAME_optstr, &tag_idx)) != NULL) {
			freeptr = optstr = strdup(__hwi_find_string(tag->optstr.string));
			while ((optstr != NULL) && (*optstr != '\0')) {
				c = optstr;
				if ((opt = getsubopt(&optstr, opts, &smc_value)) == -1) {
					fprintf(stderr, "%s - unknown option: %s\n", __FUNCTION__, c);
					continue;
				}

				switch (opt) {
					case 3:		/* smc_call */
						if (strcmp(smc_value, "yes") == 0) {
							cpu_info.smc_call_en = 1;
						}
						DVFS_INFO(dev, 1, "ARM trusted firmware enabled");
					continue;
				}
			}
			free(freeptr);
		}
		unit++;
	}

	while(*options) {
		switch(getsubopt(&options, opts, &value)) {
			case OPT_USE:
				fprintf(stderr, "\niMX8 Specific Options\n");
				fprintf(stderr, "-Dsc_path=xxxx,cl_path=xxxx\n");
				fprintf(stderr, "the default sc_path is %s\n", MX8X_SC_DECVICE_DEFUALT);
				fprintf(stderr, "the default cl_path is %s\n", MX8X_CL_DECVICE_DEFUALT);
				return EFAULT;
				break;
			case OPT_SC_DEV:
				strlcpy(cpu_info.sc_path, value, sizeof(cpu_info.sc_path));
				DVFS_INFO(dev, 1, "System controller device is %s", cpu_info.sc_path);
				break;
			case OPT_CL_DEV:
				strlcpy(cpu_info.cl_path, value, sizeof(cpu_info.cl_path));
				DVFS_INFO(dev, 1, "CPU load accounting process path is %s", cpu_info.cl_path);
				break;
			default:
				ret = EFAULT;
				break;
		}
	}

	return ret;
}

/*
 * Sample the CPU idle time
 *   index = MX8X_IDEL_START
 *   index = MX8X_IDEL_END
 */
static int mx8x_idle_time(dvfs_t *dev, int index)
{
	debug_thread_t	data;
	int				i;

	/* index=0, getting CPU idle time of start
	 * index=1, getting CPU idle time of end
	 */
	if ((index == MX8X_IDEL_START) || (index == MX8X_IDEL_END)) {
		memset(&data, 0, sizeof(debug_thread_t));
		for (i = 0, data.tid = 1; i < dev->num_cpu; ++i, ++data.tid) {
			cpu_info.sutime[index][i] = (devctl(cpu_info.cl_fd, DCMD_PROC_TIDSTATUS, &data, sizeof(debug_thread_t), NULL) == EOK) ? data.sutime : 0;
		}

		/* Calculate the idle time between start and end period */
		if (index == MX8X_IDEL_END) {
			for (i = 0; i < dev->num_cpu; ++i) {
				cpu_info.sutime[MX8X_IDEL_DIFF][i] = cpu_info.sutime[MX8X_IDEL_END][i] - cpu_info.sutime[MX8X_IDEL_START][i];
			}
		}

		return 0;
	}

	DVFS_ERROR("Unsupported index value %d (only support index 0 and 1)", index);
	return -1;
}

/*
 * Calculate the CPU load
 */
static int mx8x_cpu_load(dvfs_t *dev)
{
	int			i, load;
	uint64_t	idling;
	uint64_t	before_t, after_t, elapsed;
	cpustate_t	*cpustate = NULL;

	/* Start point */
	before_t = cpu_info.qtp->nsec;
	mx8x_idle_time(dev, MX8X_IDEL_START);

	delay(dev->accounting_interval);

	/* End point */
	after_t = cpu_info.qtp->nsec;
	mx8x_idle_time(dev, MX8X_IDEL_END);

	/* Copy load for each CPU core */
	cpustate = dev->cpustate;
	elapsed  = after_t - before_t;
	for (i = 0; i < dev->num_cpu; ++i) {
		/* calculate CPU load for each core */
		if (elapsed > cpu_info.sutime[MX8X_IDEL_DIFF][i]) {
			cpu_info.load[i] = 100 - 100 * cpu_info.sutime[MX8X_IDEL_DIFF][i] / elapsed;
		} else {
			cpu_info.load[i] = 0;
		}
		dev->pre_x_load[i] = cpustate->preload = cpustate->cpuload;
		dev->cpu_x_load[i] = cpustate->cpuload = cpu_info.load[i];
		cpustate++;
		DVFS_INFO(dev, 5, "CPU%d load is %d", i, dev->cpu_x_load[i]);
	}

	/* calculate the average CPU load */
	idling   = 0;
	for (i = 0; i < dev->num_cpu; ++i) {
		idling += cpu_info.sutime[MX8X_IDEL_DIFF][i];
	}
	load = (int)((100 * idling) / (elapsed * dev->num_cpu));
	load = 100 - min(100, max(0, load));
	cpu_info.cpuload = load;
	DVFS_INFO(dev, 4, "The average CPU load is %d", load);

	return (load);
}

/*
 * mx8x_get_chip_info: Get chip information
 * - Description: This function get the chip type and revision information.
 * - Returns:
 *  - status of temperature-related initialization. EOK on success, EFAULT on failure
 */
static uint32_t mx8x_get_chip_info(dvfs_t *dev)
{
	int							cnt;
	int							ret;
	sc_err_t					status;
	uint32_t					chip_type, chip_rev;
	imx_dcmd_sc_misc_control_t	misc_ctrl;
	char						chip_type_str[CHIP_STRING_SIZE];	// iMX8 chip type string
	char						chip_rev_str[CHIP_STRING_SIZE];		// iMX8 chip revision string

	/* get the chip infomation */
	misc_ctrl.resource	= SC_R_SYSTEM;
	misc_ctrl.ctrl		= SC_C_ID;
	misc_ctrl.val		= IMX_CHIP_TYPE_UNKNOWN;
	cnt					= 0;
	do {
		ret = devctl(cpu_info.sc_fd, IMX_DCMD_SC_MISC_GET_CONTROL, &misc_ctrl, sizeof(misc_ctrl), (int *) &status);
	} while ((ret == EAGAIN) && (cnt++ < MX8X_SC_RETRY) && (delay(MX8X_SC_DELAY) == 0));

	if (cnt >= MX8X_SC_RETRY) {
		DVFS_ERROR("Failed to get misc control information");
	}

	if (SC_ERR_NONE != status) {
		DVFS_ERROR("Failed to get misc control information (nerror is %d)", status);
	}

	chip_type = misc_ctrl.val & 0x1F;

	switch (chip_type) {
		case IMX_CHIP_TYPE_QUAD_MAX:
			strlcpy(chip_type_str, "QuadMax", sizeof(chip_type_str));
			break;
		case IMX_CHIP_TYPE_QUAD_X_PLUS:
			strlcpy(chip_type_str, "QuadXPlus", sizeof(chip_type_str));
			break;
		case IMX_CHIP_TYPE_DUAL_X_PLUS:
			strlcpy(chip_type_str, "DualXPlus", sizeof(chip_type_str));
			break;
		default:
			strlcpy(chip_type_str, "Unknown Variant", sizeof(chip_type_str));
			break;
	}

	chip_rev = (misc_ctrl.val >> 5) & 0x0F;
	switch (chip_rev) {
		case IMX_CHIP_REV_A:
			strlcpy(chip_rev_str, "A", sizeof(chip_rev_str));
			break;
		case IMX_CHIP_REV_B:
			strlcpy(chip_rev_str, "B", sizeof(chip_rev_str));
			break;
		default:
			strlcpy(chip_rev_str, "Unknown Revision", sizeof(chip_rev_str));
			break;
	}

	DVFS_INFO(dev, 1, "iMX8 %s rev_%s", chip_type_str, chip_rev_str);
	return chip_type;
}

static uint32_t mx8x_get_freq(dvfs_t *dev, char *corename, sc_rsrc_t resource)
{
	int							cnt;
	int							ret;
	sc_err_t					status;
	imx_dcmd_sc_pm_clock_rate_t	freq;

	if (resource >= SC_R_LAST) {
		return 0;
	}

	/* get the frequency */
	freq.resource	= resource;
	freq.clk		= SC_PM_CLK_CPU;
	freq.rate		= 0;
	cnt				= 0;
	do {
		ret = devctl(cpu_info.sc_fd, IMX_DCMD_SC_PM_GET_CLOCK_RATE, &freq, sizeof(freq), (int *) &status);
	} while ((ret == EAGAIN) && (cnt++ < MX8X_SC_RETRY) && (delay(MX8X_SC_DELAY) == 0));

	if (cnt >= MX8X_SC_RETRY) {
		DVFS_ERROR("Failed to get %s frequency", corename);
	}

	if (SC_ERR_NONE != status) {
		DVFS_ERROR("Failed to get %s frequency (nerror is %d)", corename, status);
	}

	DVFS_INFO(dev, 5, "Get %s frequency is %u", corename, freq.rate);

	return freq.rate;
}

static int mx8x_set_freq(dvfs_t *dev, char *corename, sc_rsrc_t resource, sc_pm_clock_rate_t rate)
{
	int							cnt;
	int							ret;
	sc_err_t					status;
	imx_dcmd_sc_pm_clock_rate_t	freq;
	imx_smc_status_t			smc_status;

	if (resource >= SC_R_LAST) {
		DVFS_ERROR("Unsupported resource %d", resource);
		return (-1);
	}

	if (cpu_info.smc_call_en == 1) {
		int cluster_id = (cpu_info.cpu_clstr2_src == resource) ? 1 : 0;
		smc_status = imx_sec_firmware_psci(IMX_FSL_SIP_CPUFREQ, IMX_FSL_SIP_SET_CPUFREQ, cluster_id, rate, 0);
		if (IMX_PSCI_SUCCESS != smc_status) {
			DVFS_ERROR("Failed to Set %s frequency to %u (nerror is %d)", corename, rate, status);
			return (-1);
		} else {
			DVFS_INFO(dev, 5, "Set %s frequency to %u", corename, rate);
		}
		return smc_status;
	}

	/* set the PLL frequency */
	freq.resource	= resource;
	freq.clk		= SC_PM_CLK_CPU;
	freq.rate		= rate;
	cnt				= 0;
	do {
		ret = devctl(cpu_info.sc_fd, IMX_DCMD_SC_PM_SET_CLOCK_RATE, &freq, sizeof(freq), (int *) &status);
	} while ((ret == EAGAIN) && (cnt++ < MX8X_SC_RETRY) && (delay(MX8X_SC_DELAY) == 0));

	if (cnt >= MX8X_SC_RETRY) {
		DVFS_ERROR("Failed to Set %s frequency to %u", corename, rate);
	}

	if (SC_ERR_NONE != status) {
		DVFS_ERROR("Failed to Set %s frequency to %u (nerror is %d)", corename, rate, status);
	}

	DVFS_INFO(dev, 5, "Set %s frequency to %u", corename, rate);

	return (status);
}

static int mx8x_get_temperature(__attribute__((unused)) dvfs_t *dev, char *corename, sc_rsrc_t resource)
{
	int						cnt;
	int						ret;
	sc_err_t				status;
	imx_dcmd_sc_misc_temp_t	temp;

	if (resource >= SC_R_LAST) {
		DVFS_ERROR("Unsupported resource %d", resource);
		return (MX8X_TEMP_ERROR);
	}

	/* get the temperature */
	temp.resource	= resource;
	temp.temp		= SC_MISC_TEMP;
	cnt				= 0;
	do {
		ret = devctl(cpu_info.sc_fd, IMX_DCMD_SC_MISC_GET_TEMP, &temp, sizeof(temp), (int *) &status);
	} while ((ret == EAGAIN) && (cnt++ < MX8X_SC_RETRY) && (delay(MX8X_SC_DELAY) == 0));

	if (cnt >= MX8X_SC_RETRY) {
		DVFS_ERROR("Failed to get %s CPU temperature", corename);
		temp.celsius = MX8X_TEMP_ERROR;
	}

	if (SC_ERR_NONE != status) {
		DVFS_ERROR("Failed to get %s CPU temperature (nerror is %d)", corename, status);
		temp.celsius = MX8X_TEMP_ERROR;
	}

	return temp.celsius;
}

uint32_t get_pwr_lvl_freq(__attribute__((unused)) void *handle, uint8_t pwr_level)
{
	if(pwr_level >= cpu_info.num_pwr_lvls || pwr_level < 0) {
		return -1;
	}
	return cpu_info.cpu_pwr_lvl_tbl[pwr_level].cpu_freq;
}

uint32_t get_cpu_freq(void *handle)
{
	dvfs_t	*dev = (dvfs_t *) handle;

	cpu_info.cpu_clstr1_freq = mx8x_get_freq(dev, cpu_info.cpu_clstr1_name, cpu_info.cpu_clstr1_src);
	cpu_info.cpu_clstr2_freq = mx8x_get_freq(dev, cpu_info.cpu_clstr2_name, cpu_info.cpu_clstr2_src);

	return cpu_info.cpu_clstr1_freq;
}

int get_cpu_temperature(void *handle)
{
	dvfs_t	*dev = (dvfs_t *) handle;
    int		cnt;
    int		temp, high_temp;

	/*
	 * Note: There’s an errata on the temp sensor on A0 silicon,
	 * the temprature may read low sometimes.
	 * Need to filter out the occasional low readings.
	 */
	high_temp = MX8X_TEMP_ERROR;

	/* get soc temperature */
	for (cnt = 0; cnt < MX8X_TEMP_READ_CNT; cnt++) {
		temp = mx8x_get_temperature(dev, "SOC", SC_R_SYSTEM);
		if (temp > high_temp)	high_temp = temp;
		delay(MX8X_SC_DELAY);
	}

	cpu_info.soc_temp = high_temp;
	DVFS_INFO(dev, 5, "iMX8 Soc temperature is %d", cpu_info.soc_temp);

	return cpu_info.soc_temp;
}

int set_cpu_freq(void *handle, uint32_t freq)
{
	dvfs_t			*dev = (dvfs_t *) handle;

	if (cpu_info.cpu_clstr1_freq != freq) {
		if (mx8x_set_freq(dev, cpu_info.cpu_clstr1_name, cpu_info.cpu_clstr1_src, freq)) {
			DVFS_ERROR("Failed to set CPU freqency");
			return EFAULT;
		}
		cpu_info.cpu_clstr1_freq = mx8x_get_freq(dev, cpu_info.cpu_clstr1_name, cpu_info.cpu_clstr1_src);
		if (cpu_info.cpu_clstr1_freq != freq) {
			DVFS_ERROR("Failed to set %s CPU freqency: set to %u, get %u",
				cpu_info.cpu_clstr1_name, freq, cpu_info.cpu_clstr1_freq);
			return EFAULT;
		}
	}

	/* Use the power table to set the culster2 CPU frequency */
	if ((cpu_info.cpu_clstr2_src == SC_R_A72) && (cpu_info.cpu_clstr2_freq != cpu_info.cpu_pwr_lvl_tbl[cpu_info.cfglevel].cpu_freq_cluster2)) {
		if (mx8x_set_freq(dev, cpu_info.cpu_clstr2_name, cpu_info.cpu_clstr2_src, cpu_info.cpu_pwr_lvl_tbl[cpu_info.cfglevel].cpu_freq_cluster2)) {
			DVFS_ERROR("Failed to set CPU freqency");
			return EFAULT;
		}
		cpu_info.cpu_clstr2_freq = mx8x_get_freq(dev, cpu_info.cpu_clstr2_name, cpu_info.cpu_clstr2_src);
		if (cpu_info.cpu_clstr2_freq != cpu_info.cpu_pwr_lvl_tbl[cpu_info.cfglevel].cpu_freq_cluster2) {
			DVFS_ERROR("Failed to set %s CPU freqency: set to %u, get %u",
				cpu_info.cpu_clstr2_name, cpu_info.cpu_pwr_lvl_tbl[cpu_info.cfglevel].cpu_freq_cluster2, cpu_info.cpu_clstr2_freq);
			return EFAULT;
		}
	}

	return EOK;
}

uint32_t get_pwr_lvl_volt(__attribute__((unused)) void *handle, uint8_t pwr_level)
{
	if (pwr_level >= cpu_info.num_pwr_lvls || pwr_level < 0) {
		return -1;
	}

	return cpu_info.cpu_pwr_lvl_tbl[pwr_level].cpu_voltage;
}

void pre_pwr_lvl_config(void *handle, int lvl)
{
	dvfs_t	*dev = (dvfs_t *) handle;

	cpu_info.cfglevel = lvl;
	DVFS_INFO(dev, 5, "Config as the power level %d", lvl);
}

void post_pwr_lvl_config(void *handle, __attribute__((unused)) int lvl)
{
	dvfs_t	*dev = (dvfs_t *) handle;

	DVFS_INFO(dev, 7, "Unsupport feature, No action");
}

void update_boundaries(void *handle)
{
	dvfs_t	*dev = (dvfs_t *) handle;

	DVFS_INFO(dev, 7, "Unsupport feature, No action");
}

uint32_t get_cpu_voltage(void *handle)
{
	/* MX8X System controller set CPU voltage automaticlly base on the CPU frequency,
	 * Configuration table may use the same voltage value for all levels
	 * Always return the voltage level 0 value in the power table
	 */

	return get_pwr_lvl_volt(handle, 0);
}

int set_cpu_voltage(void *handle, __attribute__((unused)) uint32_t voltage)
{
	dvfs_t	*dev = (dvfs_t *) handle;

	/* MX8X System controller set CPU voltage automaticlly base on the CPU frequency,
	 * No action required to set the cpu voltage
	 */
	DVFS_INFO(dev, 7, "Unsupport feature, No action");
	return EOK;
}

/************************************************************
 * Private Functions
 ************************************************************/

/*
 * get_cpu_pwr_levels_count: Returns the maximum number of power levels
 * - Description: This function returns the maximum number of power levels allowed
 * for a given operating mode (determined during initialization)
 * - Returns:
 *  - The number of power levels for the current operating mode
 */
int get_cpu_pwr_levels_count()
{
	return cpu_info.num_pwr_lvls;
}

/*
 * cpu_info_fini: CPU-related de-initialization
 * - Description: This function is charge of performing various de-initialization
 * and/or memory cleanups.
 */
void cpu_info_fini(__attribute__((unused)) dvfs_t *dev)
{
	if (cpu_info.sc_fd > 0) {
		close(cpu_info.sc_fd);
		cpu_info.sc_fd = 0;
	}

	if (cpu_info.cl_fd > 0) {
		close(cpu_info.cl_fd);
		cpu_info.cl_fd = 0;
	}

	return;
}

/*
 * init_cpu_info: CPU initialization
 * - Description: This function initializes numerous CPU-related structures and/or
 * memory mapping/configurations.
 * - Arguments:
 *  - dev: A pointer to global dvfs structure
 * - Returns:
 *  - Status of CPU initialization. EOK on success, EFAULT on failure
 */
int init_cpu_info(dvfs_t *dev)
{
	/* Open System controller device */
	if ((cpu_info.sc_fd = open(cpu_info.sc_path, O_RDWR)) < 0) {
		DVFS_ERROR("Could not open %s", cpu_info.sc_path);
		return EFAULT;
	}

	/* Open CPU Load process */
	if ((cpu_info.cl_fd = open(cpu_info.cl_path, O_RDWR)) < 0) {
		DVFS_ERROR("Could not open %s", cpu_info.cl_path);
		return EFAULT;
	}

	cpu_info.qtp		= SYSPAGE_ENTRY(qtime);
	cpu_info.num_cpu	= dev->num_cpu;
	cpu_info.chip_type	= mx8x_get_chip_info(dev);

	/* Initial the CPU source for SCFW */
	if (cpu_info.chip_type == IMX_CHIP_TYPE_QUAD_MAX) {
		cpu_info.cpu_clstr1_src = SC_R_A53;
		strlcpy(cpu_info.cpu_clstr1_name, "A53", sizeof(cpu_info.cpu_clstr1_name));
		cpu_info.cpu_clstr2_src = SC_R_A72;
		strlcpy(cpu_info.cpu_clstr2_name, "A72", sizeof(cpu_info.cpu_clstr2_name));
	}
	else if (cpu_info.chip_type == IMX_CHIP_TYPE_QUAD_X_PLUS) {
		cpu_info.cpu_clstr1_src = SC_R_A35;
		strlcpy(cpu_info.cpu_clstr1_name, "A35", sizeof(cpu_info.cpu_clstr1_name));
		cpu_info.cpu_clstr2_src = SC_R_LAST;		// unsupport second CPU cluster
		strlcpy(cpu_info.cpu_clstr2_name, "UNKNOWN", sizeof(cpu_info.cpu_clstr2_name));
	}
	else if (cpu_info.chip_type == IMX_CHIP_TYPE_DUAL_X_PLUS) {
		cpu_info.cpu_clstr1_src = SC_R_A35;
		strlcpy(cpu_info.cpu_clstr1_name, "A35", sizeof(cpu_info.cpu_clstr1_name));
		cpu_info.cpu_clstr2_src = SC_R_LAST;		// unsupport second CPU cluster
		strlcpy(cpu_info.cpu_clstr2_name, "UNKNOWN", sizeof(cpu_info.cpu_clstr2_name));
	}
	else {
		DVFS_ERROR("Unknow hardware");
		return EFAULT;
	}

	if (dev->dvfs_pwrtbl == NULL) {
		if (cpu_info.chip_type == IMX_CHIP_TYPE_QUAD_X_PLUS) {
			cpu_info.cpu_pwr_lvl_tbl    = cpu_pwr_levels_iMX8QXP_1200MHZ;
			cpu_info.num_pwr_lvls       = MX8QXP_PWR_TABLE_SIZE;
		}
		else if (cpu_info.chip_type == IMX_CHIP_TYPE_QUAD_MAX) {
			cpu_info.cpu_pwr_lvl_tbl    = cpu_pwr_levels_iMX8QM_2400MHz;
			cpu_info.num_pwr_lvls       = MX8QM_PWR_TABLE_SIZE;
		}
		else {
			DVFS_ERROR("Unknow hardware");
			return EFAULT;
		}

		DVFS_INFO(dev, 1, "Default power table:");
		for (int i = 0; i < cpu_info.num_pwr_lvls; i++) {
			DVFS_INFO(dev, 1, "level%d cluster1 CPU Freq: %u", i, cpu_info.cpu_pwr_lvl_tbl[i].cpu_freq);
			DVFS_INFO(dev, 1, "level%d cluster2 CPU Freq: %u", i, cpu_info.cpu_pwr_lvl_tbl[i].cpu_freq_cluster2);
		}
	} else {
		/* use user-defined pwr table */
		cpu_info.cpu_pwr_lvl_tbl = dev->dvfs_pwrtbl;
		cpu_info.num_pwr_lvls    = dev->dvfs_pwrtbl_num_lvls;
		DVFS_INFO(dev, 1, "Working with user-provided Power table, number of levels: %d", cpu_info.num_pwr_lvls);
	}

	return EOK;
}

/*
 * dvfs_thread_cpu_act: CPU Accounting Thread
 * - Description: This function (Thread) performs CPU load calculation. There is one instance
 * of this thread for each CPU in the system.
 * - Arguments:
 *  - cpustate: A pointer to the global cpustate (per cpu)
 * - Returns:
 *  - -1 in case of error.
 */
static int dvfs_thread_cpu_act(cpustate_t *cpustate)
{
	char			thread_name[_NTO_THREAD_NAME_MAX];
	dvfs_t			*dev = cpustate->dev;
	dvfs_table_t	*dvfs_table = NULL;
	int				cpuload;

	if(dev == NULL) {
		return -1;
	}

	/* Attempt to name the thread */
	snprintf(thread_name, sizeof(thread_name), __FUNCTION__);
	pthread_setname_np(0, thread_name);

	dvfs_table = dev->dvfs_table;

	while(1) {
		/* calculate CPU load */
		cpuload = mx8x_cpu_load(dev);

		DVFS_MUTEX_LOCK
		/* determine if we need to panic */
		if (cpuload >= DVFS_PANIC_THRESH) {
			/* send CPU increase pulse */
			DVFS_INFO(dev, 5, "Exceed level %d CPU panic level(%d): cpuload=%d",
				dev->cfglevel, DVFS_PANIC_THRESH, cpuload);
			struct _cpumsg cpumsg;
			cpumsg.msg_code = DVFS_CTRLTH_PULSE_PANIC;
			cpumsg.cpu0_load = dev->cpu_x_load[0];
			cpumsg.cpu1_load = dev->cpu_x_load[1];
			MsgSend(dev->coid, &cpumsg, sizeof(cpumsg) + 1, NULL, 0);
		} else if (cpuload > dvfs_table[dev->cfglevel].up_thr) {
			/* send CPU increase pulse */
			DVFS_INFO(dev, 5, "Exceed level %d CPU high threshold(%d): cpuload=%d",
				dev->cfglevel, dvfs_table[dev->cfglevel].up_thr, cpuload);
			struct _cpumsg cpumsg;
			cpumsg.msg_code = DVFS_CTRLTH_PULSE_CPUINC;
			cpumsg.cpu0_load = dev->cpu_x_load[0];
			cpumsg.cpu1_load = dev->cpu_x_load[1];
			MsgSend(dev->coid, &cpumsg, sizeof(cpumsg) + 1, NULL, 0);
		} else if (cpuload < dvfs_table[dev->cfglevel].down_thr) {
			/* send CPU decrease pulse */
			DVFS_INFO(dev, 5, "Exceed level %d CPU how threshold(%d): cpuload=%d",
				dev->cfglevel, dvfs_table[dev->cfglevel].down_thr, cpuload);
			struct _cpumsg cpumsg;
			cpumsg.msg_code = DVFS_CTRLTH_PULSE_CPUDEC;
			cpumsg.cpu0_load = dev->cpu_x_load[0];
			cpumsg.cpu1_load = dev->cpu_x_load[1];
			MsgSend(dev->coid, &cpumsg, sizeof(cpumsg) + 1, NULL, 0);
		}
		DVFS_MUTEX_UNLOCK
	}
}

/*
 * dvfs_create_thread_cpu_act: CPU Accounting Thread Creator
 * - Description: This function is in charge of creating CPU Accounting threads
 * for each CPU.
 * - Arguments:
 *  - cpustate: A pointer to the global cpustate (per cpu)
 * - Returns:
 *  - Status of thread creation. EOK on success, -1 on failure
 */
int dvfs_create_thread_cpu_act(cpustate_t *cpustate)
{
	dvfs_t				*dev = cpustate->dev;
	pthread_attr_t		pattr;
	struct sched_param	param;

	if(dev == NULL) {
		return -1;
	}

	pthread_attr_init(&pattr);
	pthread_attr_setschedpolicy(&pattr, SCHED_RR);
	param.sched_priority = dev->prio;
	pthread_attr_setschedparam(&pattr, &param);
	pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate( &pattr, PTHREAD_CREATE_DETACHED );

	if (pthread_create( &cpustate->tid, &pattr, (void *) dvfs_thread_cpu_act, cpustate)) {
		DVFS_ERROR("Unable to CPU accounting thread for cpu %d", cpustate->cpuid);
		return -1;
	}

	DVFS_INFO(dev, 1, "Created the CPU load accounting thread (tid%d)", cpustate->tid);
	return EOK;
}

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.1.0/trunk/hardware/dvfsmgr/aarch64/imx8x.le/dvfs_core.c $ $Rev: 894097 $")
#endif
