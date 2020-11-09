/*
 * $QNXLicenseC:
 * Copyright 2009,2015 QNX Software Systems.
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

#define _SLOGC_DVFS				90
#define CFG_FILE_COMMENT_SYMBOL '#'
#define MAX_DIGITS_IN_COLUMN 	3
#define MAX_DIGITS_IN_PWRTBL	12

#ifndef DVFS_EMAC_VAL
#define DVFS_EMAC_VAL			0x08
#endif
#ifndef MAX_LVL_UP_COUNTER
#define MAX_LVL_UP_COUNTER		10
#endif
#ifndef MAX_LVL_DOWN_COUNTER
#define MAX_LVL_DOWN_COUNTER	128
#endif
#ifndef MIN_LVL_UP_COUNTER
#define MIN_LVL_UP_COUNTER		20
#endif
#ifndef MIN_LVL_DOWN_COUNTER
#define MIN_LVL_DOWN_COUNTER	10
#endif
#ifndef MIDDLE_LVL_UP_COUNTER
#define MIDDLE_LVL_UP_COUNTER	100
#endif
#ifndef MIDDLE_LVL_DOWN_COUNTER
#define MIDDLE_LVL_DOWN_COUNTER	200
#endif
#ifndef MAX_THRESHOLD_VALUE
#define MAX_THRESHOLD_VALUE		100
#endif

#define DEFAULT_DVFS_STATE_TABLE "/etc/system/config/dvfs.conf"
#define DEFAULT_DVFS_POWER_TABLE "/etc/system/config/dvfs_pwrtbl.conf"

/**
 * dvfs_parse_options: Parse command-line options
 * - Description: This function parses various command-line options from main function. In
 * addition, it initializes some of dvfs global strucure.
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 * - argc, argv: command-line options
 * - Returns:
 * - EOK on success, EFAULT on failure (such as unknown command)
 */
int dvfs_parse_options(dvfs_t *dev,int argc, char **argv)
{
	int ret = EOK;
	int i;

	dev->verbose			= 0;
	dev->prio				= THREAD_PRIO_CPUACT;
	dev->accounting_interval = 1000;
	dev->num_cpu			= _syspage_ptr->num_cpu;
	dev->cfglevel			= 0;
	dev->wfi_workaround		= 0;
	dev->dvfs_mode			= DVFS_MODE_AUTO;
	dev->dvfs_app.notify_coid = 0;
	dev->dvfs_app.pid		= 0;
	dev->enable_speed_grade	= 1;
#if 0
	dev->cpudec_maxreq		= CPUDEC_MAXIMUM_REQUESTS;
	dev->cpudec_numreq		= 0;
	dev->cpuinc_maxreq		= CPUDEC_MAXIMUM_REQUESTS;
	dev->cpuinc_numreq		= 0;
#endif
	strncpy(dev->cfg_file_path, DEFAULT_DVFS_STATE_TABLE, PATH_MAX);
	strncpy(dev->cfg_pwr_table_path, DEFAULT_DVFS_POWER_TABLE, PATH_MAX);

	UserParm = NULL;

	cpu_parse_options(dev, "");
	while ((i = getopt(argc, argv, "vwp:i:c:d:u:U:sD:o:")) != -1) {
		switch ( i ) {
			case 'v':
				dev->verbose++;
				break;
			case 'p':
				dev->prio = atoi(optarg);
				break;
			case 'i':
				dev->accounting_interval = atoi(optarg);
				break;
			case 'c':
				strncpy(dev->cfg_file_path, optarg, PATH_MAX);
				break;
			case 'o':
				strncpy(dev->cfg_pwr_table_path, optarg, PATH_MAX);
				break;
			case 'w':
				dev->wfi_workaround = 1;
				break;
			case 's':
				dev->enable_speed_grade = 0;
				break;
			case 'D':
				ret = cpu_parse_options(dev, optarg);
				break;
#if 0
			case 'd':
				dev->cpudec_maxreq = atoi(optarg);
				break;
			case 'u':
				dev->cpuinc_maxreq = atoi(optarg);
				break;
#endif
			case 'U':
				UserParm = strdup(optarg);
				break;
			default:
				DVFS_ERROR("unknown parameter %c", i);
				ret = EFAULT;
				break;
		}
	}

	return ret;
}

/**
 * dvfslog: DVFS Log function
 * - Description: This function is used to log various information, warnings and errors
 * - Arguments:
 * - severity: Log severity (info, warning, error, ...)
 * - func: The name of function issuing the log
 * - line: The line number where the log was called
 * - verbose_level: verbosity level
 * - fmt: Message
 */
void dvfslog(int severity, const char *func, int line, int verbose_level,
			const char *fmt, ...)
{
	va_list arglist;
	char buf[512];

	// get the user message
	va_start(arglist, fmt);
	vsnprintf(buf, sizeof(buf), fmt, arglist);
	va_end(arglist);

	slogf(_SLOGC_DVFS, severity, "DVFS:%s(%d): %s", func, line, buf);
}

/**
 * get_gcd: Get Greatest Common Divider
 * - Description: This function returns the greatest common divider between
 * the two input values.
 * - Arguments:
 * - a: input number
 * - b: input number
 * - Returns:
 * - Greatest common divider between a and b
 */
unsigned get_gcd(unsigned a, unsigned b)
{
	unsigned c;

	while (1) {
		c = a % b;
		if (c == 0)
			return b;
		a = b;
		b = c;
	}
}

/**
 * dvfs_disable_range: Disable DVFS Table Power Levels
 * - Description: This function is used to disable the range of the
 * configuration levels from the start(rs) to the maximum level.
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 * - rs: Starting range
 * - owner: The requester (application, thermal thread, etc)
 * - Returns:
 * - EOK on success, EINVAL on failure
 */
int dvfs_disable_range(dvfs_t *dev, int rs, int owner)
{
	dvfs_table_t *dvfs_table = NULL;

	if ((dev == NULL) || (rs >= dev->tbcfg_length)) {
		return EINVAL;
	}

	dvfs_table = dev->dvfs_table;

	for (;;) {
		if (owner == DVFS_SCALE_THERMAL_MON) {
			dvfs_table[rs].locked_by_temp++;
		} else {
			dvfs_table[rs].disabled++;
		}
		rs = (owner == DVFS_SCALE_THERMAL_MON)
			? (rs - 1)
			: (rs + 1);
		if ((rs < 0 ) || (rs >= dev->tbcfg_length)) {
			break;
		}
	}
	return EOK;
}


/**
 * dvfs_enable_range: Enable DVFS Table Power Levels
 * - Description: This function is used to enable the range of the
 * configuration levels from the start(rs) to the maximum level.
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 * - rs: Starting range
 * - owner: The requester (application, thermal thread, etc)
 * - Returns:
 * - EOK on success, EINVAL on failure
 */
int dvfs_enable_range(dvfs_t *dev, int rs, int owner)
{
	dvfs_table_t *dvfs_table = NULL;

	if ((dev == NULL) || (rs >= dev->tbcfg_length)) {
		return EINVAL;
	}

	dvfs_table = dev->dvfs_table;

	for (;;) {
		if (owner == DVFS_SCALE_THERMAL_MON) {
			if (dvfs_table[rs].locked_by_temp) {
				dvfs_table[rs].locked_by_temp--;
			}
		} else {
			if (dvfs_table[rs].disabled) {
				dvfs_table[rs].disabled--;
			}
		}
		rs = (owner == DVFS_SCALE_THERMAL_MON)
			? (rs - 1)
			: (rs + 1);
		if ((rs < 0 ) || (rs >= dev->tbcfg_length)) {
			break;
		}
	}
	return EOK;
}

/**
 * dvfs_check_power_levels: Check content of dvfs config file
 * - Description: This function checks whether the dvfs config file
 * contains correct data
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 * - dvfs_table: A pointer to the dvfs table
 * - len: Length of dvfs table
 * - Returns:
 * - EOK on success, EFAULT on failure
 */
int dvfs_check_power_levels(dvfs_t *dev, dvfs_table_t t[], int len)
{
	int ret = EFAULT;
	dvfs_table_t temp_lvl;
	int i, j;
	if (len < 2) {
		DVFS_ERROR("At least two power levels should be specified in the config file");
		return ret;
	}
	//sort power levels
	for (i = 1; i < len; ++i) {
		for (j = i; j > 0; --j) {
			if (t[j].pwr_lvl == t[j-1].pwr_lvl) {
				DVFS_ERROR("Two rows in the config file have the same power level %d\n", t[j].pwr_lvl);
				return ret;
			}
			if (t[j].pwr_lvl < t[j - 1].pwr_lvl) {
				// swap power levels
				temp_lvl.pwr_lvl = t[j - 1].pwr_lvl;
				temp_lvl.up_thr = t[j - 1].up_thr;
				temp_lvl.down_thr = t[j - 1].down_thr;
				temp_lvl.panic_thr = t[j - 1].panic_thr;
				temp_lvl.up_cnt = t[j - 1].up_cnt;
				temp_lvl.down_cnt = t[j - 1].down_cnt;
				temp_lvl.emac = t[j - 1].emac;
				temp_lvl.disabled = t[j - 1].disabled;
				temp_lvl.up_temp_thr = t[j - 1].up_temp_thr;
				temp_lvl.down_temp_thr = t[j - 1].down_temp_thr;

				t[j - 1].pwr_lvl = t[j].pwr_lvl;
				t[j - 1].up_thr = t[j].up_thr;
				t[j - 1].down_thr = t[j].down_thr;
				t[j - 1].panic_thr = t[j].panic_thr;
				t[j - 1].up_cnt = t[j].up_cnt;
				t[j - 1].down_cnt = t[j].down_cnt;
				t[j - 1].emac = t[j].emac;
				t[j - 1].disabled = t[j].disabled;
				t[j - 1].up_temp_thr = t[j].up_temp_thr;
				t[j - 1].down_temp_thr = t[j].down_temp_thr;

				t[j].pwr_lvl = temp_lvl.pwr_lvl;
				t[j].up_thr = temp_lvl.up_thr;
				t[j].down_thr = temp_lvl.down_thr;
				t[j].panic_thr = temp_lvl.panic_thr;
				t[j].up_cnt = temp_lvl.up_cnt;
				t[j].down_cnt = temp_lvl.down_cnt;
				t[j].emac = temp_lvl.emac;
				t[j].disabled = temp_lvl.disabled;
				t[j].up_temp_thr = temp_lvl.up_temp_thr;
				t[j].down_temp_thr = temp_lvl.down_temp_thr;
			}
		}
	}
	ret = EOK;
	return ret;
}

/**
 * get_next_level_from_file: Populate dvfs table entry from config file (one entry)
 * - Description: This function parses config file and tries to fill up the description
 * for one power level. If error occurs during parsing it writes error status into
 * 'err' parameter. If all is OK then 'lvl_desc' parameter will contain description
 * of one power level.
 * - Arguments:
 * - dev: A pointer to global dvfs structure
 * - fd: File descriptor of dvfs config file
 * - lvl_desc: One dvfs table entry
 * - err: Error (if encountered)
 * - Returns:
 * - EOK on success, -1 if end of file is reached
 */
int get_next_level_from_file(dvfs_t *dev, int fd, dvfs_table_t *lvl_desc, int *err)
{
	typedef enum {
		COLUMN_POWER_LEVEL = 0,
		COLUMN_UP_THRESHOLD,
		COLUMN_DOWN_THRESHOLD,
		COLUMN_UP_TEMPERATURE_THRESHOLD,
		COLUMN_DOWN_TEMPERATURE_THRESHOLD,
		COLUMN_MAX_COLUMN
	} cfg_column_types;

	int ret = EOK;
	char buf[1];
	char value[MAX_DIGITS_IN_COLUMN];
	uint8_t val_ind = 0;
	uint8_t val_started = 0;
	uint8_t pwr_level_started = 0;
	uint8_t new_line = 1;
	uint8_t comment_line = 0;
	int pwr_lvl = -1;
	int up_thresh_perc = 0;
	int down_thresh_perc = 0;
	int up_temp_thr = 0;
	int down_temp_thr = 0;
	int val = 0;
	cfg_column_types current_column = COLUMN_POWER_LEVEL;

	memset(value, 0, MAX_DIGITS_IN_COLUMN * sizeof(char));
	*err = 0;
	while ( (ret = read(fd, buf, 1)) != -1 && ret != 0) {
		if (comment_line == 1) {
			if (buf[0] == '\n') {
				//if we reached the end of line then need to set the new line indication
				new_line = 1;
				comment_line = 0;;
			}
			continue;
		}
		if (new_line == 1) {
			if (buf[0] == ' ' || buf[0] == '\t' || buf[0] == '\n' || buf[0] == '\r' || buf[0] == EOF) {
				//skip all space symbols at the beginning of the line
				continue;
			} else if (buf[0] == CFG_FILE_COMMENT_SYMBOL) {
				new_line = 0;
				comment_line = 1;
				continue;
			} else {
				new_line = 0;
			}
		}
		if (val_started == 0) {
			if (buf[0] == ' ' || buf[0] == '\t' || buf[0] == '\n' || buf[0] == '\r' || buf[0] == EOF) {
				//skip space characters between values
				continue;
			} else if (buf[0] != '0' && buf[0] != '1' && buf[0] != '2' && buf[0] != '3'
					&& buf[0] != '4' && buf[0] != '5' &&
					buf[0] != '6' && buf[0] != '7' && buf[0] != '8' && buf[0] != '9') {
				DVFS_ERROR("Error parsing config file. Only positive digital values are allowed");
				*err = -1;
				break;
			}
			if (val_ind == MAX_DIGITS_IN_COLUMN) {
				DVFS_ERROR("Error parsing config file. Numbers maximum length is %d", MAX_DIGITS_IN_COLUMN);
				*err = -1;
				break;
			}
			val_started = 1;
			pwr_level_started = 1;
			value[val_ind++] = buf[0];
		} else {
			//val_started == 1
			if (buf[0] == ' ' || buf[0] == '\t' || buf[0] == '\n' || buf[0] == '\r' || buf[0] == EOF) {
				if ((buf[0] == '\n' || buf[0] == '\r' || buf[0] == EOF) &&
					current_column != COLUMN_DOWN_TEMPERATURE_THRESHOLD) {
					//unexpected end of line, not all parameters are specified
					DVFS_ERROR("Not all parameters are specified in the config file");
					*err = -1;
					break;
				}
				//convert characters value into int
				val = atoi(value);
				//save parameter value
				switch (current_column) {
					case COLUMN_POWER_LEVEL:
						pwr_lvl = val;
						if (pwr_lvl >= get_pwr_levels_count(dev)) {
							DVFS_ERROR("Error in the config file. Maximum level should not exceed %d", get_pwr_levels_count(dev));
							*err = -1;
						} else if (pwr_lvl < 0) {
							DVFS_ERROR("Error in the config file. Minimum level should not be less than 0");
							*err = -1;
						}
						break;
					case COLUMN_UP_THRESHOLD:
						up_thresh_perc = val;
						break;
					case COLUMN_DOWN_THRESHOLD:
						down_thresh_perc = val;
						break;
					case COLUMN_UP_TEMPERATURE_THRESHOLD:
						up_temp_thr = val;
						break;
					case COLUMN_DOWN_TEMPERATURE_THRESHOLD:
						down_temp_thr = val;
						break;
					default:
						DVFS_ERROR("Unexpected error during parsing of config file");
						*err = -1;
						break;
				}
				if (0 != *err) {
					//some unexpected error
					break;
				}
				val_started = 0;
				memset(value, 0, MAX_DIGITS_IN_COLUMN * sizeof(char));
				val_ind = 0;
				val = 0;
				current_column++;
				if (current_column == COLUMN_MAX_COLUMN) {
					break;
				}
			} else {
				if (buf[0] != '0' && buf[0] != '1' && buf[0] != '2' && buf[0] != '3'
					&& buf[0] != '4' && buf[0] != '5' &&
					buf[0] != '6' && buf[0] != '7' && buf[0] != '8' && buf[0] != '9') {
					DVFS_ERROR("Error parsing config file. Only digital values are allowed");
					*err = -1;
					break;
				}
				if (val_ind == MAX_DIGITS_IN_COLUMN) {
					DVFS_ERROR("Error parsing config file. Numbers maximum length is %d", MAX_DIGITS_IN_COLUMN);
					*err = -1;
					break;
				}
				value[val_ind++] = buf[0];
			}
		}
	}
	if (*err == 0) {
		if (1 == val_started && COLUMN_DOWN_TEMPERATURE_THRESHOLD == current_column) {
			//for the case when the config file ends without empty line at the end
			down_temp_thr = atoi(value);
			current_column++;
		}
		if (pwr_level_started == 1 && current_column == COLUMN_MAX_COLUMN) {
			ret = EOK;
			DVFS_INFO(dev, 1, "Parsed level: %d, %d, %d, %d, %d",
					pwr_lvl, up_thresh_perc, down_thresh_perc, up_temp_thr, down_temp_thr);
			//fill level description
			lvl_desc->pwr_lvl = pwr_lvl;
			lvl_desc->up_thr = (MAX_THRESHOLD_VALUE * up_thresh_perc) / 100;
			lvl_desc->down_thr = (MAX_THRESHOLD_VALUE * down_thresh_perc) / 100;
			lvl_desc->panic_thr = MAX_THRESHOLD_VALUE;
			lvl_desc->up_cnt = 0; // will be set later
			lvl_desc->down_cnt = 0; // will be set later
			lvl_desc->emac = DVFS_EMAC_VAL;
			lvl_desc->up_temp_thr = up_temp_thr;
			lvl_desc->down_temp_thr = down_temp_thr;
			lvl_desc->disabled = 0;
			lvl_desc->locked_by_temp = 0;
			lvl_desc->rapps = NULL;
		} else if (pwr_level_started == 1 && current_column != COLUMN_MAX_COLUMN) {
			ret = EOK;
			*err = -1;
			DVFS_ERROR("Not all parameters are specified in the config file");
		} else {
			ret = -1; //end of file is reached
		}
	} else {
		ret = EOK;
	}
	return ret;
}

/*
 * dvfs_set_up_down_threshold_counters()
 *
 * Description: This function initializes up and down threshold counters for power levels
 *
 * Returns: None
 *
 */
static void dvfs_set_up_down_threshold_counters(dvfs_table_t t[], int len)
{
	int i =0;

	// initialize counters for the first level
	t[0].up_cnt = MAX_LVL_UP_COUNTER;
	t[0].down_cnt = MAX_LVL_DOWN_COUNTER;

	// initialize counters for middle levels
	for (i = 1; i < len - 1; ++i)
	{
		t[i].up_cnt = MIDDLE_LVL_UP_COUNTER;
		t[i].down_cnt = MIDDLE_LVL_DOWN_COUNTER;
	}

	// initialize counters for the last level
	t[len -1].up_cnt = MIN_LVL_UP_COUNTER;
	t[len -1].down_cnt = MIN_LVL_DOWN_COUNTER;
}

/**
 * dvfsc_tblock_table_init: Initialize DVFS Config Table
 * - Description: This function initializes DVFS Config Table
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 * - Returns:
 * - EOK on success, EFAULT on failure
 */
int dvfs_dvfs_table_init(dvfs_t *dev)
{
	int				ret = EFAULT;
	int				i;
	int				cfg_fd;
	dvfs_table_t	*t_init = NULL;
	dvfs_table_t	lvl_desc = {0};
	int				err;
	int				*tlen;
	dvfs_table_t	**tbcfg;

	tlen = &dev->tbcfg_length;
	tbcfg = &dev->dvfs_table;

	if ((t_init = malloc(sizeof(dvfs_table_t) * get_pwr_levels_count(dev))) == NULL) {
		return ret;
	}

	if ( (cfg_fd = open(dev->cfg_file_path, O_RDONLY )) != -1 ) {
		if ((tbcfg != NULL) && (tlen != NULL)) {
			*tlen = 0;
			//parse config file and fill the load tracking table
			while (get_next_level_from_file(dev, cfg_fd, &lvl_desc, &err) != -1) {
				if (err != 0) {
					break; //parse error occurred, stop further processing
				}
				t_init[*tlen].pwr_lvl = lvl_desc.pwr_lvl;
				t_init[*tlen].up_thr = lvl_desc.up_thr;
				t_init[*tlen].down_thr = lvl_desc.down_thr;
				t_init[*tlen].panic_thr = lvl_desc.panic_thr;
				t_init[*tlen].up_cnt = lvl_desc.up_cnt;
				t_init[*tlen].down_cnt = lvl_desc.down_cnt;
				t_init[*tlen].emac = lvl_desc.emac;
				t_init[*tlen].up_temp_thr = lvl_desc.up_temp_thr;
				t_init[*tlen].down_temp_thr = lvl_desc.down_temp_thr;
				t_init[*tlen].disabled = lvl_desc.disabled;
				t_init[*tlen].locked_by_temp = lvl_desc.locked_by_temp;
				t_init[*tlen].rapps = lvl_desc.rapps;
				*tlen = *tlen + 1;
			}
			if (0 == err) {
				if (dvfs_check_power_levels(dev, t_init, *tlen) == EOK) {
					dvfs_set_up_down_threshold_counters(t_init, *tlen);
					if ((*tbcfg = malloc(*tlen * sizeof(dvfs_table_t))) != NULL) {
						for (i = 0; i < *tlen; i++) {
							(*tbcfg)[i].pwr_lvl	= t_init[i].pwr_lvl;
							(*tbcfg)[i].up_thr	= t_init[i].up_thr;
							(*tbcfg)[i].down_thr = t_init[i].down_thr;
							(*tbcfg)[i].panic_thr = t_init[i].panic_thr;
							(*tbcfg)[i].up_cnt	= t_init[i].up_cnt;
							(*tbcfg)[i].down_cnt = t_init[i].down_cnt;
							(*tbcfg)[i].emac	= t_init[i].emac;
							(*tbcfg)[i].up_temp_thr = t_init[i].up_temp_thr;
							(*tbcfg)[i].down_temp_thr = t_init[i].down_temp_thr;
							(*tbcfg)[i].disabled	= t_init[i].disabled;
							(*tbcfg)[i].locked_by_temp = t_init[i].locked_by_temp;
							(*tbcfg)[i].rapps = t_init[i].rapps;
							//print the level
							DVFS_INFO(dev, 4, "Level: %d, Up thre: %d, Up cnt: %d, Down thre: %d, Down cnt: %d, emac = %d",
								(*tbcfg)[i].pwr_lvl, (*tbcfg)[i].up_thr, (*tbcfg)[i].up_cnt, (*tbcfg)[i].down_thr, (*tbcfg)[i].down_cnt, (*tbcfg)[i].emac);

						}
						ret = EOK;
					}
				}
			}
			else {
				//error occurred during parsing
				*tlen = 0;
			}
		}
		close(cfg_fd);
	}
	else {
		DVFS_ERROR("Unable to open config file %s\n", dev->cfg_file_path);
	}
	if (t_init != NULL) {
		free(t_init);
	}
	return ret;
}

/**
 *
 */
 int dvfs_get_next_pwrtbl_row(dvfs_t *dev, int fd, dvfs_pwrtbl_t *lvl_desc, int *err)
 {
	typedef enum {
		COLUMN_OPP = 0,
		COLUMN_PLL_FREQ,
		COLUMN_FREQ,
		COLUMN_CPU_VOLT,
#if defined (iMX6_SOC)
		COLUMN_SOC_VOLT,
#endif
#if defined (iMX8_SOC)
		COLUMN_FREQ_CLUSTER2,
#endif
		COLUMN_EFUSEREG,
		COLUMN_MAX_COLUMN
	} cfg_column_types;

	int ret = EOK;
	char buf[1];
	char value[MAX_DIGITS_IN_PWRTBL];
	uint8_t val_ind = 0;
	uint8_t val_started = 0;
	uint8_t pwr_level_started = 0;
	uint8_t new_line = 1;
	uint8_t comment_line = 0;
	int val = 0;
	int tmp_opp = PWR_OPP_UNKNOWN;
	uint32_t tmp_freq = 0;
	uint32_t tmp_cpu_volt = 0;
#if defined (iMX6_SOC)
	uint32_t tmp_soc_volt = 0;
#endif
#if defined (iMX8_SOC)
	uint32_t tmp_freq_cluster2 = 0;
#endif
	uint32_t tmp_pll_freq = 0;
	uint32_t tmp_efuse_val = 0;
	int bsecond_digit = 0;

	cfg_column_types current_column = COLUMN_OPP;
	memset(value, 0, MAX_DIGITS_IN_COLUMN * sizeof(char));
	memset(lvl_desc, 0, sizeof(dvfs_pwrtbl_t));
	*err = 0;

	while ( (ret = read(fd, buf, 1)) != -1 && ret != 0) {
		if (comment_line == 1) {
			if (buf[0] == '\n') {
				//if we reached the end of line then need to set the new line indication
				new_line = 1;
				comment_line = 0;;
			}
			continue;
		}
		if (new_line == 1) {
			if (buf[0] == ' ' || buf[0] == '\t' || buf[0] == '\n' || buf[0] == '\r' || buf[0] == EOF) {
				//skip all space symbols at the beginning of the line
				continue;
			} else if (buf[0] == CFG_FILE_COMMENT_SYMBOL) {
				new_line = 0;
				comment_line = 1;
				continue;
			} else {
				new_line = 0;
			}
		}
		if (val_started == 0) {
			if (buf[0] == ' ' || buf[0] == '\t' || buf[0] == '\n' || buf[0] == '\r' || buf[0] == EOF) {
				//skip space characters between values
				continue;
			} else if (buf[0] != '0' && buf[0] != '1' && buf[0] != '2' && buf[0] != '3'
					&& buf[0] != '4' && buf[0] != '5' &&
					buf[0] != '6' && buf[0] != '7' && buf[0] != '8' && buf[0] != '9') {
				DVFS_ERROR("Error parsing config file. Only positive digital values are allowed");
				*err = -1;
				break;
			}
			if (val_ind == MAX_DIGITS_IN_PWRTBL) {
				DVFS_ERROR("Error parsing config file. Numbers maximum length is %d", MAX_DIGITS_IN_PWRTBL);
				*err = -1;
				break;
			}
			val_started = 1;
			pwr_level_started = 1;
			value[val_ind++] = buf[0];
			if (current_column == COLUMN_EFUSEREG) {
				bsecond_digit = 1;
			}
		} else {
			//val_started == 1
			if (buf[0] == ' ' || buf[0] == '\t' || buf[0] == '\n' || buf[0] == '\r' || buf[0] == EOF) {
				if ((buf[0] == '\n' || buf[0] == '\r' || buf[0] == EOF) &&
					current_column != COLUMN_EFUSEREG) {
					//unexpected end of line, not all parameters are specified
					DVFS_ERROR("Not all parameters are specified in the config file");
					*err = -1;
					break;
				}
				//convert characters value into int
				if (current_column == COLUMN_EFUSEREG) {
					val = strtol(value, NULL, 16);
				} else {
					val = atoi(value);
				}
				//save parameter value
				switch (current_column) {
					case COLUMN_OPP:
						tmp_opp = val;
						if (tmp_opp < PWR_OPP_LOW || tmp_opp > PWR_OPP_HIGH) {
							DVFS_ERROR("Error in the config file. OPP Value must be betweek %d and %d ", PWR_OPP_LOW, PWR_OPP_HIGH);
							*err = -1;
						}
						break;
					case COLUMN_PLL_FREQ:
						tmp_pll_freq = val;
						break;
					case COLUMN_FREQ:
						tmp_freq = val;
						break;
					case COLUMN_CPU_VOLT:
						tmp_cpu_volt = val;
						break;
#if defined (iMX6_SOC)
					case COLUMN_SOC_VOLT:
						tmp_soc_volt = val;
						break;
#endif
#if defined (iMX8_SOC)
					case COLUMN_FREQ_CLUSTER2:
						tmp_freq_cluster2 = val;
						break;
#endif
					case COLUMN_EFUSEREG:
						tmp_efuse_val = val;
						break;
					default:
						DVFS_ERROR("Unexpected error during parsing of config file");
						*err = -1;
						break;
				}
				if (0 != *err) {
					//some unexpected error
					break;
				}
				val_started = 0;
				memset(value, 0, MAX_DIGITS_IN_PWRTBL * sizeof(char));
				val_ind = 0;
				val = 0;
				current_column++;
				if (current_column == COLUMN_MAX_COLUMN) {
					break;
				}
			} else {
				if (current_column == COLUMN_EFUSEREG) {
					if (bsecond_digit) {
						if (buf[0] != '0' && buf[0] != '1' && buf[0] != '2' && buf[0] != '3'
							&& buf[0] != '4' && buf[0] != '5' &&
							buf[0] != '6' && buf[0] != '7' && buf[0] != '8' && buf[0] != '9' && buf[0] != 'x' && buf[0] != 'X') {
								DVFS_ERROR("Error parsing config file. Only digital values are allowed");
								*err = -1;
								break;
						}
						bsecond_digit = 0;
					} else {
						if (buf[0] != '0' && buf[0] != '1' && buf[0] != '2' && buf[0] != '3'
							&& buf[0] != '4' && buf[0] != '5' &&
							buf[0] != '6' && buf[0] != '7' && buf[0] != '8' && buf[0] != '9' &&
							buf[0] != 'a' && buf[0] != 'A' && buf[0] != 'b' && buf[0] != 'b' &&
							buf[0] != 'c' && buf[0] != 'C' && buf[0] != 'd' && buf[0] != 'D' &&
							buf[0] != 'e' && buf[0] != 'E' && buf[0] != 'f' && buf[0] != 'F') {
								DVFS_ERROR("Error parsing config file. Only digital values are allowed");
								*err = -1;
								break;
						}
					}
				} else if (buf[0] != '0' && buf[0] != '1' && buf[0] != '2' && buf[0] != '3'
					&& buf[0] != '4' && buf[0] != '5' &&
					buf[0] != '6' && buf[0] != '7' && buf[0] != '8' && buf[0] != '9') {
					DVFS_ERROR("Error parsing config file. Only digital values are allowed");
					*err = -1;
					break;
				}
				if (val_ind == MAX_DIGITS_IN_PWRTBL) {
					DVFS_ERROR("Error parsing config file. Numbers maximum length is %d", MAX_DIGITS_IN_PWRTBL);
					*err = -1;
					break;
				}
				value[val_ind++] = buf[0];
			}
		}
	} // while loop
	if (*err == 0) {
		if (1 == val_started && COLUMN_EFUSEREG == current_column) {
			// for the case when the config file ends without empty line and the end
			tmp_efuse_val = strtol(value, NULL, 16);
			current_column++;
		}
		if (pwr_level_started == 1 && current_column == COLUMN_MAX_COLUMN) {
			ret = EOK;

			// fill in table
			lvl_desc->opp_lvl = tmp_opp;
			lvl_desc->sys_pll_freq = tmp_pll_freq;
			lvl_desc->cpu_freq = tmp_freq;
			lvl_desc->cpu_voltage = tmp_cpu_volt;
			lvl_desc->efuse_val = tmp_efuse_val;
#if defined (iMX6_SOC)
			lvl_desc->soc_voltage = tmp_soc_volt;
			DVFS_INFO(dev, 1, "Parsed level: %d, %d, %d, %d, %d, 0x%x",
				tmp_opp, tmp_pll_freq, tmp_freq, tmp_cpu_volt, tmp_soc_volt, tmp_efuse_val);
#elif defined (iMX8_SOC)
			lvl_desc->cpu_freq_cluster2 = tmp_freq_cluster2;
			DVFS_INFO(dev, 1, "Parsed level: %d, %u, %u, %u, %u, 0x%x",
				lvl_desc->opp_lvl, lvl_desc->sys_pll_freq, lvl_desc->cpu_freq, lvl_desc->cpu_voltage,
				lvl_desc->cpu_freq_cluster2, lvl_desc->efuse_val);
#else
			DVFS_INFO(dev, 1, "Parsed level: %d, %d, %d, %d, 0x%x",
				tmp_opp, tmp_pll_freq, tmp_freq, tmp_cpu_volt, tmp_efuse_val);
#endif
		} else if (pwr_level_started == 1 && current_column != COLUMN_MAX_COLUMN) {
			ret = EOK;
			*err = -1;
			DVFS_ERROR("Not all parameters are specified in the config file");
		} else {
			ret = -1;
		}
	} else {
		ret = EOK;
	}
	return ret;
 }

int dvfs_populate_pwrtbl(dvfs_t *dev) {
	int ret = ENOENT;
	int fd;
	dvfs_pwrtbl_t **ptr_pwrtbl;
	dvfs_pwrtbl_t tmp_pwr_tbl[MAX_PWR_LVLS];

	int pwrtbl_idx = 0;
	int idx;
	int err;

	ptr_pwrtbl = &dev->dvfs_pwrtbl;
	if ((fd = open(dev->cfg_pwr_table_path, O_RDONLY)) != -1) {
		// read in table
		while (dvfs_get_next_pwrtbl_row(dev, fd, &tmp_pwr_tbl[pwrtbl_idx++], &err) == EOK) {
			if (err != 0) {
				break;
			}
		}
		DVFS_INFO(dev, 1, "pwrtbl_idx= %d", pwrtbl_idx);

		// store size if valid table was read
		if (err == 0) {
			//Verify table
			for (idx = 0; idx < pwrtbl_idx-1; idx++) {
				if (idx < (pwrtbl_idx - 2) ) {
					if (tmp_pwr_tbl[idx+1].opp_lvl > tmp_pwr_tbl[idx].opp_lvl) {
						DVFS_ERROR("OPP of next level may be less than or equal to current level");
						return EINVAL;
					} else if (tmp_pwr_tbl[idx+1].cpu_freq >= tmp_pwr_tbl[idx].cpu_freq) {
						DVFS_ERROR("Frequency of next level MUST be less than current level");
						return EINVAL;
					} else if (tmp_pwr_tbl[idx+1].cpu_voltage > tmp_pwr_tbl[idx].cpu_voltage) {
						DVFS_ERROR("Voltage of next level may be less than or equal to current level");
						return EINVAL;
					}
#if defined (iMX6_SOC)
					else if (tmp_pwr_tbl[idx+1].soc_voltage > tmp_pwr_tbl[idx].soc_voltage) {
						DVFS_ERROR("Voltage of next level may be less than or equal to current level");
						return EINVAL;
					}
#endif
#if defined (iMX8_SOC)
					else if (tmp_pwr_tbl[idx+1].cpu_freq_cluster2 >= tmp_pwr_tbl[idx].cpu_freq_cluster2) {
						DVFS_ERROR("Frequency of next level MUST be less than current level");
						return EINVAL;
					}
#endif
				}
				if (tmp_pwr_tbl[idx].sys_pll_freq < 1000000) {
					DVFS_ERROR("PLL Frequency too low! Valid PLL Frequency above 1MHz");
					return EINVAL;
				} else if (tmp_pwr_tbl[idx].cpu_freq < 1000000) {
					DVFS_ERROR("CPU Frequency too low! Valid CPU Frequency above 1MHz");
					return EINVAL;
				}
#if !defined(ZYNQ_SOC)
				else if (tmp_pwr_tbl[idx].cpu_freq > tmp_pwr_tbl[idx].sys_pll_freq) {
					DVFS_ERROR("CPU Frequency must be less than or equal to PLL Frequency");
					return EINVAL;
				}
				else if (tmp_pwr_tbl[idx].sys_pll_freq % tmp_pwr_tbl[idx].cpu_freq) {
					DVFS_ERROR("CPU Frequency must be divisible by PLL Frequency");
					return EINVAL;
				}
#endif
				else if (tmp_pwr_tbl[idx].cpu_voltage <= 0 || tmp_pwr_tbl[idx].cpu_voltage >2000000) {
					DVFS_ERROR("Voltage value too high! Valid voltage between (0 2V]");
					return EINVAL;
#if defined (iMX6_SOC)
				} else if (tmp_pwr_tbl[idx].soc_voltage <= 0 || tmp_pwr_tbl[idx].soc_voltage >2000000) {
					DVFS_ERROR("Voltage value too high! Valid voltage between (0 2V]");
					return EINVAL;
#endif
#if defined (iMX8_SOC)
				} else if (tmp_pwr_tbl[idx].cpu_freq_cluster2 < 1000000) {
					DVFS_ERROR("CPU cluster2 Frequency too low! Valid CPU Frequency above 1MHz");
					return EINVAL;
#endif
				} else if (tmp_pwr_tbl[idx].sys_pll_freq > tmp_pwr_tbl[0].sys_pll_freq) {
					DVFS_ERROR("Based on provided table, PLL Frequency may no exceed %d Hz", tmp_pwr_tbl[0].sys_pll_freq);
					return EINVAL;
				}
#if !defined(ZYNQ_SOC)
				else if (tmp_pwr_tbl[idx].cpu_freq > tmp_pwr_tbl[0].sys_pll_freq) {
					DVFS_ERROR("Based on provided table, Frequency may no exceed %d Hz", tmp_pwr_tbl[0].sys_pll_freq);
					return EINVAL;
				}
#endif
			}

			// table is valid. Copy to dvfs structure
			*ptr_pwrtbl = malloc(pwrtbl_idx * sizeof(dvfs_pwrtbl_t));
			dev->dvfs_pwrtbl_num_lvls = pwrtbl_idx-1;
			if (*ptr_pwrtbl == NULL) {
				DVFS_ERROR("Unable to create DVFS Power Table structure");
				return ENOMEM;
			}

			for (idx = 0; idx < dev->dvfs_pwrtbl_num_lvls ; idx++) {
				(*ptr_pwrtbl)[idx].opp_lvl = tmp_pwr_tbl[idx].opp_lvl;
				(*ptr_pwrtbl)[idx].sys_pll_freq = tmp_pwr_tbl[idx].sys_pll_freq;
				(*ptr_pwrtbl)[idx].cpu_freq = tmp_pwr_tbl[idx].cpu_freq;
				(*ptr_pwrtbl)[idx].cpu_voltage = tmp_pwr_tbl[idx].cpu_voltage;
				(*ptr_pwrtbl)[idx].efuse_val = tmp_pwr_tbl[idx].efuse_val;
				(*ptr_pwrtbl)[idx].max_allowed_freq = (*ptr_pwrtbl)[0].sys_pll_freq;
#if defined (iMX6_SOC)
				(*ptr_pwrtbl)[idx].soc_voltage = tmp_pwr_tbl[idx].soc_voltage;
				DVFS_INFO(dev, 4, "OPP = %d, PLL Freq = %d, Freq = %d, CPU Volt = %d, SOC Volt %d, EFUSE REG = 0x%x, Max Allow Freq = %d",
					(*ptr_pwrtbl)[idx].opp_lvl, (*ptr_pwrtbl)[idx].sys_pll_freq,
					(*ptr_pwrtbl)[idx].cpu_freq, (*ptr_pwrtbl)[idx].cpu_voltage, (*ptr_pwrtbl)[idx].soc_voltage,
					(*ptr_pwrtbl)[idx].efuse_val, (*ptr_pwrtbl)[idx].max_allowed_freq);
#elif defined (iMX8_SOC)
				(*ptr_pwrtbl)[idx].cpu_freq_cluster2 = tmp_pwr_tbl[idx].cpu_freq_cluster2;
				DVFS_INFO(dev, 4, "OPP = %d, PLL Freq = %u, CPU Freq = %u, CPU Volt = %u, Max Allow CPU Freq = %u, CPU Freq2 = %u, EFUSE REG = 0x%x",
					(*ptr_pwrtbl)[idx].opp_lvl, (*ptr_pwrtbl)[idx].sys_pll_freq,
					(*ptr_pwrtbl)[idx].cpu_freq, (*ptr_pwrtbl)[idx].cpu_voltage,
					(*ptr_pwrtbl)[idx].max_allowed_freq, (*ptr_pwrtbl)[idx].cpu_freq_cluster2,
					(*ptr_pwrtbl)[idx].efuse_val);
#else
				DVFS_INFO(dev, 4, "OPP = %d, PLL Freq = %d, Freq = %d, Volt = %d, EFUSE REG = 0x%x, Max Allow Freq = %d",
					(*ptr_pwrtbl)[idx].opp_lvl, (*ptr_pwrtbl)[idx].sys_pll_freq,
					(*ptr_pwrtbl)[idx].cpu_freq, (*ptr_pwrtbl)[idx].cpu_voltage,
					(*ptr_pwrtbl)[idx].efuse_val, (*ptr_pwrtbl)[idx].max_allowed_freq);
#endif
			}
			ret = EOK;
		} else {
			DVFS_ERROR("Invalid Power Table Config File. Please see user guide");
		}
	} else {
		DVFS_ERROR("No Power Table Config File found!!");
	}

	return ret;
}


#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/dvfsmgr/dvfs_utils.c $ $Rev: 887556 $")
#endif
