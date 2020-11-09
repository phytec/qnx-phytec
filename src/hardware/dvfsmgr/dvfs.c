/*
 * $QNXLicenseC:
 * Copyright 2012,2015 QNX Software Systems.
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
#include <login.h>
#include <sys/procmgr.h>

#define DVFS_DEV_PATH					"/dev/dvfs"
#define DVFSC_STATUS_STR_LENGTH			2048			// Status string length
#define TEMP_MEASURE_SLEEP_INTERVAL		1000			// Temperature measurement intervals (default = 1sec)
#define BG_THREAD_SLEEP_INTERVAL		5000			// Background task intervals (default = 5sec)

/**
 * Resource Manager Related variables
 */
resmgr_connect_funcs_t connect_funcs;
resmgr_io_funcs_t	io_funcs;
iofunc_attr_t		attr;

/**
 * Global pointer to the dvfs structure
 */
dvfs_t *gdvfs = NULL; // pointer to dvfs structure

/**
 * internal external function definition
 */
extern int set_cpu_power_level(dvfs_t *dev, uint8_t pwr_level);
extern int dvfs_dvfs_table_init(dvfs_t *handle);
extern int dvfs_enable_range(dvfs_t *dev, int rs, int owner);
extern int dvfs_disable_range(dvfs_t *dev, int rs, int owner);
extern int dvfs_parse_options(dvfs_t *dev,int argc, char **argv);
extern int dvfs_populate_pwrtbl(dvfs_t *dev);

/************************************************************************
 * THREADS
 ***********************************************************************/

#ifdef	__ARM__
/**
 * dvfs_wfi_thread: Idle Thread (at priority #1)
 * - Description: This thread runs at lowest possible priority (#1) and simply waits for interrupts.
 * This thread is mandatory for platforms that compute CPU load using software (i.e. omap5).This
 * thread is currently mandatory for hardware-computed CPU load in SMP mode (i.e. mx6). SW_CPULOAD is
 * used to determine whether this thread performs any CPU-load related activities or not. This definition
 * must be defined in variant.h.
 * - Arguments:
 * - cpustate: A pointer to cpustate structure
 * - Returns:
 * - -1 for error.
 */
int dvfs_wfi_thread(cpustate_t *cpustate)
{
	char thread_name[_NTO_THREAD_NAME_MAX];
	int cpu = cpustate->cpuid;
	unsigned runmask = (1 << cpu);

	/* Attempt to name the thread. */
	snprintf(thread_name, sizeof(thread_name), "dvfs_wfi_thread_%d\n", cpu);
	pthread_setname_np(0, thread_name);
	/*specify the runmask*/
	if (ThreadCtl(_NTO_TCTL_RUNMASK_GET_AND_SET, &runmask) == -1) {
		slogf(_SLOGC_BLOCK_IOBLK, _SLOG_ERROR,"Unable to Specify runmask for CPU %d", cpu);
		return -1;
	}

	for (;;) {
		__asm__ __volatile__(
				"cpsid i\n"
				"dsb\n"
				"wfi\n"
				"cpsie i\n" );
	}
	return EOK;
}
#endif

/**
 * dvfs_ctrl: DVFS Control Thread
 * - Description: This thread is in charge or receiving pulses/messages from various
 * sources (interrupts, threads) and performing dynamic voltage/frequency scaling if necessary.
 * For detailed explanation on the functionality of this thread, please refer to documentation.
 * - Arguments:
 * - dev: A pointer to global dvfs structure
 */
int dvfs_ctrl(dvfs_t *dev)
{
	dvfs_table_t *dvfs_table = NULL;
	int		rcvid;
	int		cur_index;
	char	thread_name[_NTO_THREAD_NAME_MAX];
	int		i;
	int		temp;
	union {
		struct	_pulse pulse;
		struct	_cpumsg cpumsg;
	} msg;

	dvfs_table = dev->dvfs_table;

	/* Attempt to name the thread. */
	snprintf(thread_name, sizeof(thread_name), "dvfs_ctrl_thread\n");
	pthread_setname_np(0, thread_name);

	while (1) {
		if ((rcvid = MsgReceive(dev->chid, &msg, sizeof(msg), NULL)) == -1) {
			continue;
		}
		if (rcvid == 0) { // it's a pulse;
			switch (msg.pulse.code) {
				case DVFS_CTRLTH_PULSE_THERMAL:
					temp = msg.pulse.value.sival_int;
					cur_index = dev->cfglevel;
					// Only react if in auto or semi-auto mode. Notify
					if (dev->dvfs_mode == DVFS_MODE_MAN) {
						break;
					}

					// update config table based on current temperature reading
					for (i = 0; i < dev->tbcfg_length; i++) {
						if ((temp > dvfs_table[i].up_temp_thr) &&
							((!dvfs_table[i].locked_by_temp) || (dvfs_table[i].locked_by_temp && dev->cfglevel == i))	&&
							(i < (dev->tbcfg_length - 1))) {

							/* Maximum temperature of the power level has been exceeded */
							DVFS_INFO(dev, 1, "Maximum temperature (%d) of %d level has been exceeded", temp, i);

							// disable level
							dvfs_disable_range(dev, i, DVFS_SCALE_THERMAL_MON);

							// update current level to next lower state
							if (i == cur_index) {
								cur_index++;
							}
						} else if ((dvfs_table[i].locked_by_temp) &&
								(temp <= dvfs_table[i].down_temp_thr)) {

							DVFS_INFO(dev, 1, "level %d is re-enabled (temperature %d)", i, temp);
							dvfs_enable_range(dev, i, DVFS_SCALE_THERMAL_MON);
							for (;;) {
								if ((cur_index == 0) || (!dvfs_table[cur_index].disabled)) {
									break;
								}
								cur_index--;
							}
						}
					}

					// change state if neccessary
					if (cur_index != dev->cfglevel) {
						if (set_cpu_power_level(dev, dvfs_table[cur_index].pwr_lvl) != EOK) {
							DVFS_WARNING(dev, 1, "Could not set power level (%d)", cur_index);
						} else {
							int j = 0;
							related_apps_t *p = NULL;
							// Notify client if we need to run at a power level disabled by the user
							if ((cur_index > dev->cfglevel) && dvfs_table[cur_index].disabled) {
								for (j = dev->cfglevel; j < cur_index; j++) {
									p = dvfs_table[j].rapps;
									while (p != NULL) {
										MsgSendPulse(p->notify_coid, -1, DVFS_NOTIFY_REJECTED, cur_index);
										p = p->next_app;
									}
								}
							} else {
								//Notify client that previously disabled level has been re-enabled
								for (j = (dev->cfglevel - 1); j >= cur_index; j--) {
									p = dvfs_table[j].rapps;
									while (p != NULL) {
										MsgSendPulse(p->notify_coid, -1, DVFS_NOTIFY_RESTORED, cur_index);
										p = p->next_app;
									}
								}
							}
							dev->cfglevel = cur_index;
						}
					}
					update_boundaries(dev);
					break;
				case DVFS_CTRLTH_PULSE_TERMINATE:
					DVFS_INFO(dev, 4, "Control Thread Terminated");
					pthread_exit(NULL);
					break;
				default:
					break;
			}
		} else {
			switch (msg.cpumsg.msg_code) {
				case DVFS_CTRLTH_PULSE_CPUINC:

					// only react if in auto mode
					if (dev->dvfs_mode != DVFS_MODE_AUTO) {
						break;
					}
					cur_index = dev->cfglevel;
					if (cur_index > 0) {
						// update only if the next level is not locked
						if (!dvfs_table[cur_index - 1].locked_by_temp && !dvfs_table[cur_index - 1].disabled) {
							// call appropriate function
							if (set_cpu_power_level(dev, dvfs_table[--cur_index].pwr_lvl) != EOK) {
								DVFS_WARNING(dev, 1, "Could not set power level");
							} else {
								dev->cfglevel = cur_index;
							}
						}
					}
					break;
				case DVFS_CTRLTH_PULSE_CPUDEC:

					// only react if in auto mode
					if (dev->dvfs_mode != DVFS_MODE_AUTO) {
						break;
					}
					cur_index = dev->cfglevel;
					if (cur_index < dev->tbcfg_length - 1) {
						if (!dvfs_table[cur_index+1].locked_by_temp && !dvfs_table[cur_index+1].disabled) {
							// call appropriate function
							cur_index += 1;
							if (set_cpu_power_level(dev, dvfs_table[cur_index].pwr_lvl) != EOK) {
								DVFS_WARNING(dev, 1, "Could not set power level");
							} else {
								dev->cfglevel = cur_index;
							}
						}
					}
					break;
				case DVFS_CTRLTH_PULSE_SETLVL:
				{
					int req_lvl;
					// only react in auto mode
					if (dev->dvfs_mode != DVFS_MODE_AUTO) {
						break;
					}
					cur_index = dev->cfglevel;
					req_lvl = msg.cpumsg.lvl;
					if (req_lvl == cur_index) {
						// nothing to be done
						break;
					}
					if (req_lvl >= get_pwr_levels_count(dev) || req_lvl < 0) {
						//invalid power level
						DVFS_ERROR("Unable to set CPU Power Level to %d, allowed levels are from 0 to %d",
										req_lvl, get_pwr_levels_count(dev) - 1);
						break;
					}
					if (!dvfs_table[req_lvl].locked_by_temp && !dvfs_table[req_lvl].disabled) {
						DVFS_INFO(dev, 1, "Changing Power Level (%d-->%d)(%dHz-->%dHz)(%duV->%duV)", cur_index, req_lvl,
							get_pwr_lvl_freq(dev, cur_index), get_pwr_lvl_freq(dev, req_lvl), get_pwr_lvl_volt(dev, cur_index), get_pwr_lvl_volt(dev, req_lvl));
						// call appropriate function
						if (set_cpu_power_level(dev, dvfs_table[req_lvl].pwr_lvl) != EOK) {
							DVFS_WARNING(dev, 1, "Could not set power level");
						} else {
							dev->cfglevel = req_lvl;
						}
					}
				}
					break;
				case DVFS_CTRLTH_PULSE_PANIC:
					// only react if in auto mode
					if (dev->dvfs_mode != DVFS_MODE_AUTO) {
						break;
					}
					cur_index = 0;

					if (dev->cfglevel > 0) {
						// find the highest ALLOWED power level
						for (cur_index=0; cur_index < dev->tbcfg_length; cur_index++) {
							// update only if the next level is not locked
							if (cur_index < dev->cfglevel) {
								if (!dvfs_table[cur_index].locked_by_temp && !dvfs_table[cur_index].disabled) {
									// call appropriate function
									if (set_cpu_power_level(dev, dvfs_table[cur_index].pwr_lvl) != EOK) {
										DVFS_WARNING(dev, 1, "Could not set power level");
									} else {
										dev->cfglevel = cur_index;
									}
									break;
								}
							}
						}
					}
					break;
			}
			MsgReply(rcvid, EOK, NULL, 0);
		}
	}
	return EOK;
}

/**
 * dvfs_therm_thread: Temperature Monitoring Thread
 * - Description: This thread is in charge of monitoring core temperature and notifying
 * the Control Thread periodically.
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 */
void dvfs_therm_thread(dvfs_t *dev)
{
	int temp;
	char thread_name[_NTO_THREAD_NAME_MAX];

	/* Attempt to name the thread. */
	snprintf(thread_name, sizeof(thread_name), "dvfs_thermal_thread\n");
	pthread_setname_np(0, thread_name);

	while (1) {
		temp = get_cpu_temperature(dev);
		MsgSendPulse(dev->coid, -1, DVFS_CTRLTH_PULSE_THERMAL, temp);
		dev->ctemp = temp;
		delay(TEMP_MEASURE_SLEEP_INTERVAL);
	}
	return;
}

/**
 * dvfs_bg_thread: Background Thread
 * - Description: This thread is in charge of performing background or
 * cleanup tasks
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 */
void dvfs_bg_thread(dvfs_t *dev)
{
	int fd;
	char process_name[100];
	char thread_name[_NTO_THREAD_NAME_MAX];
	int i;

	/* Attempt to name the thread. */
	snprintf(thread_name, sizeof(thread_name), "dvfs_background_thread\n");
	pthread_setname_np(0, thread_name);

	while (1) {
		delay(BG_THREAD_SLEEP_INTERVAL);
		// Check to see the registerd PID exists (if in manual or semi mode)
		DVFS_MODE_MUTEX_LOCK
		if (dev->dvfs_mode != DVFS_MODE_AUTO) {
			if (dev->dvfs_app.pid > 0) {
				sprintf(process_name, "/proc/%d", dev->dvfs_app.pid);
				if ((fd = open(process_name, O_RDONLY)) == -1) {
					DVFS_ERROR("Process ( %s ) does not exist! Switching to AUTO Mode", process_name);
					dev->dvfs_mode = DVFS_MODE_AUTO;
					if (dev->dvfs_app.notify_coid > 0) {
						ConnectDetach(dev->dvfs_app.notify_coid);
						dev->dvfs_app.notify_coid = 0;
					}
					dev->dvfs_app.pid = 0;
				} else {
					close(fd);
				}
			}
		}
		DVFS_MODE_MUTEX_UNLOCK

		// Make sure the apps that requested a minimum level of operation still exist
		DVFS_RAPP_MUTEX_LOCK
		for (i = 0; i < dev->tbcfg_length; i++) {
			related_apps_t *next = dev->dvfs_table[i].rapps;
			related_apps_t *prev = NULL;
			while (next != NULL) {
				sprintf(process_name, "/proc/%d", next->pid);
				if ((fd = open(process_name, O_RDONLY)) == -1) {
					DVFS_ERROR("Process ( %s ) does not exist! Removing it", process_name);
					if (prev == NULL) {
						dev->dvfs_table[i].rapps = next->next_app;
					} else {
						prev->next_app = next->next_app;
					}
					dvfs_enable_range(dev, (i + 1), DVFS_SCALE_APPLICATION);
					free(next);
					next = NULL;
				} else {
					close(fd);
				}
				if (next == NULL) {
					if (prev == NULL) {
						next = dev->dvfs_table[i].rapps;
					} else {
						next = prev->next_app;;
					}
				} else {
					prev = next;
					next = next->next_app;
				}
			}
		}
		DVFS_RAPP_MUTEX_UNLOCK
	}

}
/************************************************************************
 * END THREADS
 ***********************************************************************/

/**
 * remove_rapps: Remove an application from the application request list
 * - Description:
 * - This function removes an application that had previoius requested a
 * a minimum level of performance from the dvfs manager
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 * - Returns:
 * - Status. EOK on success, EFAULT on failure
 */
int remove_rapps(dvfs_t *dev)
{
	int i;
	int ret = EOK;
	related_apps_t *p = NULL;

	for (i = 0; i < dev->tbcfg_length; i++) {
		while (dev->dvfs_table[i].rapps != NULL) {
			p = dev->dvfs_table[i].rapps;
			dev->dvfs_table[i].rapps
					= (related_apps_t *) dev->dvfs_table[i].rapps->next_app;
			if (p->notify_coid > 0) {
				MsgSendPulse(p->notify_coid, -1, DVFS_NOTIFY_TERMINATED, 0);
				if (ConnectDetach(p->notify_coid)) {
					DVFS_ERROR("Unable to detach connection");
					ret = EFAULT;
				}
			}
			free(p);
			dvfs_enable_range(dev, 0, DVFS_SCALE_APPLICATION);
		}
	}

	return ret;
}

/**
 * set_cpu_power_level: Transition to a given power level.
 * - Description:
 * - This function sets the SoC Frequency and Voltage to the
 * specified power level.
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 * - pwr_level: The desired new power level.
 * - Returns:
 * - Status of power level transition. EOK on success, EFAULT on failure
 */
int set_cpu_power_level(dvfs_t *dev, uint8_t pwr_level)
{
	uint32_t curr_cpu_freq;
	uint32_t cpu_freq;
	uint32_t curr_cpu_volt;
	uint32_t cpu_volt;
	uint8_t freq_changed = 0;
	uint8_t volt_changed = 0;
	int	ret = EOK;

	if (pwr_level >= get_pwr_levels_count(dev) || pwr_level < 0) {
		DVFS_ERROR("Unable to set CPU Power Level to %d, allowed levels are from 0 to %d",
				pwr_level, get_pwr_levels_count(dev) - 1);
		ret = EFAULT;
		goto Error;
	}

	curr_cpu_freq = get_cpu_freq(dev);
	cpu_freq = get_pwr_lvl_freq(dev, pwr_level);

	curr_cpu_volt = get_cpu_voltage(dev);
	if (curr_cpu_volt == INVALID_VOLTAGE || curr_cpu_volt > MAX_SAFE_VOLT) {
		goto End;
	}

	cpu_volt = get_pwr_lvl_volt(dev, pwr_level);

	if (cpu_freq == curr_cpu_freq) {
		goto End;
	}

	if (dev->cfglevel != -1) {
		DVFS_INFO(dev, 1, "%s Power Level (threshold = %d)(%d-->%d)(%dHz-->%dHz)(%duV-->%duV)",
								(dev->cfglevel >= pwr_level)? "Increasing":"Decreasing",
								(dev->cfglevel >= pwr_level)? dev->dvfs_table[dev->cfglevel].up_thr : dev->dvfs_table[dev->cfglevel].down_thr,
								dev->cfglevel, pwr_level,
								get_pwr_lvl_freq(dev, dev->cfglevel), get_pwr_lvl_freq(dev, pwr_level),
								get_pwr_lvl_volt(dev, dev->cfglevel), get_pwr_lvl_volt(dev, pwr_level));
	} else {
		DVFS_INFO(dev, 1, "Transition to Max Power Level (-->%d)(%dHz-->%dHz)(%duV-->%duV)",
								pwr_level,
								get_cpu_freq(dev), get_pwr_lvl_freq(dev, pwr_level),
								get_cpu_voltage(dev), get_pwr_lvl_volt(dev, pwr_level));
	}
	// perform any pre-action that might be needed by hardware
	pre_pwr_lvl_config(dev, pwr_level);

	if (cpu_freq > curr_cpu_freq) {
		//increase voltage first
		if (cpu_volt != curr_cpu_volt) {
			if (set_cpu_voltage(dev, cpu_volt) != EOK) {
				ret = EFAULT;
				goto Error;
			}
			usleep(1000);
			volt_changed = 1;
		}
	}
	if (set_cpu_freq(dev, cpu_freq) != EOK) {
		ret = EFAULT;
		goto Error;
	}

	//change CPU frequency
	freq_changed = 1;

	if (cpu_freq < curr_cpu_freq) {
		// set cpu voltage
		if (cpu_volt != curr_cpu_volt) {
			if (set_cpu_voltage(dev, cpu_volt) != EOK) {
				ret = EFAULT;
				goto Error;
			}
			usleep(1000);
		}
	}

End:
	if (ret == EOK) {
		// perform any post-action that might be needed by hardware
		post_pwr_lvl_config(dev, pwr_level);
	}
	return ret;

Error:
	if (volt_changed > 0) {
		//restore voltage value
		set_cpu_voltage(dev, curr_cpu_volt);
	}
	if (freq_changed > 0) {
		// restore previous frequency value
		set_cpu_freq(dev, curr_cpu_freq);
	}
	goto End;
}

/**
 * dvfs_init_cpustate: Creates cpustate structure for each core
 * - Description: This function creates a cpustate structure for every
 * active core
 * - Arguments:
 * - dev: A pointer to global dvfs structure
 * - Returns:
 * - Status of cpustate creation. EOK on success, ENOMEM on failure
 */

int dvfs_init_cpustate(dvfs_t *dev)
{
	cpustate_t *cpustate = NULL;
	int		i;

	if ((dev->cpustate = calloc(1, sizeof(cpustate_t) * (dev->num_cpu))) == NULL) {
		DVFS_ERROR("Could not allocate memory for CPU load array");
		return ENOMEM;
	}

	cpustate = dev->cpustate;
	for (i = 0; i<dev->num_cpu; i++) {
		cpustate->dev = dev;
		cpustate->cpuid = i;
		cpustate->desired_index = 0;
		cpustate->num_cycles_idle = 0;
		cpustate++;
	}
	return EOK;
}


/**
 * dvfs_create_background_thread: Create background thread
 * - Description: This function is in charge of creating background
 * thread.
 * - Arguments:
 * - dev: A pointer to global dvfs structure
 * - Returns:
 * - Status of thread creation. EOK on success, EFAULT on failure
 */
int dvfs_create_background_thread(dvfs_t *dev)
{
	pthread_attr_t		pattr;
	struct sched_param	param;

	pthread_attr_init(&pattr);
	pthread_attr_setschedpolicy(&pattr, SCHED_RR);
	param.sched_priority = THREAD_PRIO_BG;
	pthread_attr_setschedparam(&pattr, &param);
	pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);

	if (pthread_create(&dev->bg_tid, &pattr, (void *) dvfs_bg_thread, dev)) {
		DVFS_ERROR("Unable to create background thread");
		return EFAULT;
	}
	return EOK;

}

#ifdef	__ARM__
/**
 * dvfs_create_wfi_threads: Create IDLE threads
 * - Description: This function creates IDLE threads (1 per CPU).
 * - Arguments:
 * - dev: A pointer to global dvfs structure
 * - Returns:
 * - Status of thread creation. EOK on success, EFAULT on failure
 */
int dvfs_create_wfi_threads(dvfs_t *dev)
{
	cpustate_t				*cpustate = NULL;
	pthread_attr_t			pattr;
	struct sched_param		param;
	int						i;

	pthread_attr_init(&pattr);
	pthread_attr_setschedpolicy(&pattr, SCHED_RR);
	param.sched_priority = THREAD_PRIO_IDLE;
	pthread_attr_setschedparam(&pattr, &param);
	pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);
	pthread_attr_setdetachstate( &pattr, PTHREAD_CREATE_DETACHED );

	cpustate = dev->cpustate;
	for (i = 0; i<dev->num_cpu; i++) {
		if ( pthread_create( &dev->wfi_tid, &pattr, (void *) dvfs_wfi_thread, cpustate )) {
			DVFS_ERROR("Unable to create thread for cpu %d", cpustate->cpuid);
			return EFAULT;
		}
		cpustate++;
	}
	return EOK;
}
#endif

/**
 * dvfs_create_ctrl_thread: Create Control Thread
 * - Description: This function creates the Control thread.
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 * - Returns:
 * - Status of thread creation. EOK on success, EFAULT on failure
 */
int dvfs_create_ctrl_thread(dvfs_t *dev)
{
	pthread_attr_t		pattr;
	struct sched_param	param;

	if ((dev->chid = ChannelCreate(_NTO_CHF_DISCONNECT | _NTO_CHF_UNBLOCK)) == -1
		|| (dev->coid = ConnectAttach( 0, 0, dev->chid, _NTO_SIDE_CHANNEL, 0 )) == -1 ) {
		DVFS_ERROR("ChannelCreate failed");
		return EFAULT;
	}
	pthread_attr_init(&pattr);
	pthread_attr_setschedpolicy(&pattr, SCHED_RR);
	param.sched_priority = THREAD_PRIO_CTRL; // default - will get bumped by higher priority thread calls
	pthread_attr_setschedparam(&pattr, &param);
	pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);

	if (pthread_create(&dev->tid, &pattr, (void *) dvfs_ctrl, dev)) {
		DVFS_ERROR("Unable to create control thread");
		return EFAULT;
	}
	return EOK;
}

/**
 * dvfs_create_therm_thread: Create Temperature Monitoring Thread
 * - Description: This function creates the temperature monitoring thread.
 * - Arguments:
 * - dev: A pointer to the global dvfs structure
 * - Returns:
 * - Status of thread creation. EOK on success, EFAULT on failture
 */
int dvfs_create_therm_thread(dvfs_t *dev)
{
	pthread_attr_t		pattr;
	struct sched_param	param;

	pthread_attr_init(&pattr);
	pthread_attr_setschedpolicy(&pattr, SCHED_RR);
	param.sched_priority = dev->prio - 1;
	pthread_attr_setschedparam(&pattr, &param);
	pthread_attr_setinheritsched(&pattr, PTHREAD_EXPLICIT_SCHED);

	if (pthread_create(&dev->therm_tid, &pattr, (void *) dvfs_therm_thread, dev)) {
		DVFS_ERROR("Unable to create thermal thread");
		return EFAULT;
	}
	return EOK;
}

/**
 * dvfs_io_read: DVFS resource manager read function
 * - Description: This function replies with all relevant information with regards
 * to the DVFS status. The returned message is in String format.
 * - Arguments:
 * - ctp: A pointer to the resmgr_context_t structure that the resource manager library uses to pass context information between functions
 * - msg: A pointer to the io_read_t structure that contains the message that the resource manager received.
 * - ocb: A pointer to the iofunc_ocb_t structure for the Open Control Block that was created when the client opened the resource.
 */
int dvfs_io_read(resmgr_context_t *ctp, io_read_t *msg, RESMGR_OCB_T *ocb)
{
	int		nreply;
	int		status;
	static int nbytes = 0;
	static int offset;
	static char stat_str[DVFSC_STATUS_STR_LENGTH];
#ifdef SW_CPULOAD
	cpustate_t *cpustate = NULL;
#endif
	int		i;
	char modeofop[32];

	/* Verify a client's read access to a resource */
	if ((status = iofunc_read_verify(ctp, msg, ocb, NULL)) != EOK) {
		return status;
	}

	if (nbytes == 0) {

		if (msg->i.nbytes < DVFSC_STATUS_STR_LENGTH) {
			DVFS_WARNING(gdvfs, 1, "Read buffer too small (%d). Read buffer should be at least %d bytes", msg->i.nbytes, DVFSC_STATUS_STR_LENGTH);
			return (ERANGE);
		}


		switch (gdvfs->dvfs_mode) {
			case DVFS_MODE_SEMI:
				strcpy(modeofop, "Semi-Auto");
				break;
			case DVFS_MODE_MAN:
				strcpy(modeofop, "Manual");
				break;
			case DVFS_MODE_AUTO:
			default:
				strcpy(modeofop, "Auto");
				break;
		}

		snprintf(stat_str, DVFSC_STATUS_STR_LENGTH,
				"-----------------------------------------------------------------------------------------\n");
		snprintf(stat_str + strlen(stat_str),
		(DVFSC_STATUS_STR_LENGTH - strlen(stat_str)),
				"\t\t\t\tCore Temp = %d	\n", gdvfs->ctemp);

#ifdef SW_CPULOAD
		cpustate = gdvfs->cpustate;
		for (i = 0; i<gdvfs->num_cpu; i++) {
			snprintf(stat_str + strlen(stat_str),
			(DVFSC_STATUS_STR_LENGTH - strlen(stat_str)),
					"\t\t\t\tCPU<%d> Load = %d	\n", cpustate->cpuid, cpustate->cpuload);
			cpustate++;
		}
#endif
		snprintf(stat_str + strlen(stat_str),
		(DVFSC_STATUS_STR_LENGTH - strlen(stat_str)),
				"\t\t\t\tMode = %s (%d) \n", modeofop, gdvfs->dvfs_mode);

		snprintf(stat_str + strlen(stat_str),
		(DVFSC_STATUS_STR_LENGTH - strlen(stat_str)),
				"-----------------------------------------------------------------------------------------\n");
		snprintf(stat_str + strlen(stat_str),
		(DVFSC_STATUS_STR_LENGTH - strlen(stat_str)),
				" # \t|\tApp\t|\tThermal\t|\tFreq (Hz)\t|\tVolt (uV)\t|\n");
		snprintf(stat_str + strlen(stat_str),
		(DVFSC_STATUS_STR_LENGTH - strlen(stat_str)),
				"-----------------------------------------------------------------------------------------\n");
		for (i = 0; i < gdvfs->tbcfg_length; i++) {
			snprintf(stat_str + strlen(stat_str),
					(DVFSC_STATUS_STR_LENGTH - strlen(stat_str)),
					" %c%d \t|\t%s\t|\t%s\t|\t%d\t|\t%d\t\t|\n",(i == gdvfs->cfglevel)?'*':' ' ,i,
			(gdvfs->dvfs_table[i].disabled)?"x":"-",
			(gdvfs->dvfs_table[i].locked_by_temp)?"x":"-",
			get_pwr_lvl_freq(gdvfs, i), get_pwr_lvl_volt(gdvfs, i));
		}
		snprintf(stat_str + strlen(stat_str),
				(DVFSC_STATUS_STR_LENGTH - strlen(stat_str)),
				"-----------------------------------------------------------------------------------------\n");

		nbytes = strlen(stat_str);
		nreply = (nbytes <= msg->i.nbytes) ? nbytes : msg->i.nbytes;
		offset = nreply;
		MsgReply(ctp->rcvid, nreply, stat_str, nreply);
	} else if (offset < nbytes) {
		/*
		* This function has been invoked at the second time etc., We check
		* how many bytes are left and send the message to client
		*/
		nreply = min((nbytes - offset), msg->i.nbytes);
		MsgReply(ctp->rcvid, nreply, stat_str + offset, nreply);
		offset += nreply;
	} else {
		nbytes = 0;
		offset = 0;
		MsgReply(ctp->rcvid, EOK, NULL, 0);
	}
	return _RESMGR_NOREPLY;
}


/**
 * dvfsc_io_devctl: DVFS resource manager device control function
 * - Description: This function uses to handle an _IO_DEVCTL message
 * - Arguments:
 * - ctp: A pointer to the resmgr_context_t structure that the resource manager library uses to pass context information between functions
 * - msg: A pointer to the io_read_t structure that contains the message that the resource manager received.
 * - ocb: A pointer to the iofunc_ocb_t structure for the Open Control Block that was created when the client opened the resource.
 * - Returns:
 * - EOK on success, EINVAL on failure
 *
 */
int dvfs_io_devctl(resmgr_context_t *ctp, io_devctl_t *msg, RESMGR_OCB_T *ocb)
{
	int			ret = _RESMGR_ERRNO(-EINVAL);
	dvfs_data_t	cfgl;
	dvfs_t			*dev = gdvfs;

	do {
		/* Make default actions for _IO_DEVCTL message */
		if (iofunc_devctl_default(ctp, msg, ocb) != _RESMGR_DEFAULT) {
			break;
		}

		if (_DEVCTL_DATA(msg->i) == NULL) {
			DVFS_ERROR("Missing configuration level");
			break;
		}
		if (DVFS_DEVCTL_GETSTATUS != msg->i.dcmd) {
			memcpy(&cfgl, (dvfs_data_t *)(_DEVCTL_DATA(msg->i)), sizeof(dvfs_data_t));
			if (msg->i.dcmd != DVFS_DEVCTL_SET_MODE) {
				if ((!((cfgl.reql >= 0) && (cfgl.reql < dev->tbcfg_length))) &&
						(msg->i.dcmd != DVFS_DEVCTL_RUN_MAX && msg->i.dcmd != DVFS_DEVCTL_RUN_MIN)) {
					DVFS_ERROR("Unknown configuration level (%d)", cfgl.reql);
					break;
				}
			}
		}
		switch (msg->i.dcmd) {
			case DVFS_DEVCTL_SET_MIN:
			{
				// only allow in auto mode
				if (dev->dvfs_mode != DVFS_MODE_AUTO) {
					DVFS_ERROR("Unable to set. Must be in Auto Mode");
					break;
				}

				related_apps_t **pp = NULL;
				DVFS_INFO(dev, 1, "Set minimum level=%d pid=%d", cfgl.reql, cfgl.capp_pid);

				if (dev->dvfs_table[cfgl.reql].locked_by_temp) {
					DVFS_ERROR("Unable to set, level (%d) was locked by thermal monitor", cfgl.reql);
					break;
				}
				DVFS_MUTEX_LOCK
				dvfs_disable_range(dev, (cfgl.reql + 1), DVFS_SCALE_APPLICATION);
				if (dev->cfglevel > cfgl.reql) {
					if (set_cpu_power_level(dev, cfgl.reql)) {
						DVFS_MUTEX_UNLOCK
						DVFS_ERROR("Error has occurred during of transition between levels");
						break;
					} else {
						dev->cfglevel = cfgl.reql;
					}
				}
				DVFS_MUTEX_UNLOCK

				DVFS_RAPP_MUTEX_LOCK
				if (dev->dvfs_table[cfgl.reql].rapps == NULL) {
					pp = &dev->dvfs_table[cfgl.reql].rapps;
				} else {
					related_apps_t *p = dev->dvfs_table[cfgl.reql].rapps;
					while (p->next_app != NULL) {
						p = p->next_app;
					}
					pp = (related_apps_t **)&p->next_app;
				}
				if ((pp != NULL) && ((*pp = malloc(sizeof(related_apps_t))) != NULL)) {
					(*pp)->next_app = NULL;
					(*pp)->pid	= cfgl.capp_pid;
					(*pp)->notify_coid = ConnectAttach(0, (*pp)->pid,
														cfgl.capp_chid,
														_NTO_SIDE_CHANNEL, 0);
					ret = _RESMGR_ERRNO(EOK);
				} else {
					DVFS_ERROR("Unable to allocate application entry");
				}
				DVFS_RAPP_MUTEX_UNLOCK
			}
				update_boundaries(dev);
				break;
			case DVFS_DEVCTL_UNSET_MIN:
			{
					related_apps_t *next = dev->dvfs_table[cfgl.reql].rapps;
					related_apps_t *prev = NULL;

					DVFS_INFO(dev, 1, "Unset minimum level=%d pid=%d", cfgl.reql, cfgl.capp_pid);
					DVFS_RAPP_MUTEX_LOCK
					while (next != NULL) {
						if (next->pid == cfgl.capp_pid) {
							if (prev == NULL) {
								dev->dvfs_table[cfgl.reql].rapps = next->next_app;
								free(next);
							} else {
								prev->next_app = next->next_app;
								free(next);
							}
							dvfs_enable_range(dev, (cfgl.reql + 1), DVFS_SCALE_APPLICATION);
							break;
						}
						prev = next;
						next = next->next_app;
					}
					DVFS_RAPP_MUTEX_UNLOCK
					update_boundaries(dev);
					ret = _RESMGR_ERRNO(EOK);
			}
				break;
			case DVFS_DEVCTL_SET_MODE:
			{
				// check to see if another app has already made such request
				DVFS_MODE_MUTEX_LOCK
				if (dev->dvfs_app.pid > 0) {
					// check to see if the request is from the same app or another app (only one app allowed)
					if (dev->dvfs_app.pid == cfgl.capp_pid) {
						// make sure the requested mode is supported
						if (cfgl.reql > DVFS_MODE_MAX) {
							DVFS_ERROR("Unrecognized mode requested (%d)", cfgl.reql);
							DVFS_MODE_MUTEX_UNLOCK
							break;
						}
						dev->dvfs_mode = cfgl.reql;
						if (dev->dvfs_mode == DVFS_MODE_AUTO) {
							// if auto mode requested, reset dvfs_app for other apps to use
							if (dev->dvfs_app.notify_coid > 0) {
								ConnectDetach(dev->dvfs_app.notify_coid);
								dev->dvfs_app.notify_coid = 0;
							}
							dev->dvfs_app.pid = 0;
						}
					} else {
						DVFS_ERROR("Another program (pid %d) has already set the mode", dev->dvfs_app.pid);
						DVFS_MODE_MUTEX_UNLOCK
						break;
					}
				} else {
					// first time request or new request from an app
					if (cfgl.reql > DVFS_MODE_MAX) {
						DVFS_ERROR("Unrecognized mode requested (%d)", cfgl.reql);
						DVFS_MODE_MUTEX_UNLOCK
						break;
					}
					// store information
					if (cfgl.reql != DVFS_MODE_AUTO) {
						// store notifiers and pid
						dev->dvfs_app.notify_coid = ConnectAttach(0, cfgl.capp_pid, cfgl.capp_chid, _NTO_SIDE_CHANNEL, 0);
						dev->dvfs_app.pid = cfgl.capp_pid;

						// notify all application that their previous request is not valid
						remove_rapps(gdvfs);


					}
					dev->dvfs_mode = cfgl.reql;
				}
				DVFS_MODE_MUTEX_UNLOCK
				ret = _RESMGR_ERRNO(EOK);
			}
			break;
			case DVFS_DEVCTL_SET_PWR_LVL:
			{
				dvfs_table_t *dvfs_table = dev->dvfs_table;

				/* only the app that successfully set mode to "manual" or "semi-auto"
				* is allowed to change levels.
				*/
				if (dev->dvfs_mode == DVFS_MODE_AUTO || cfgl.capp_pid != dev->dvfs_app.pid) {
					DVFS_ERROR("Auto Mode detected OR request is not from pid %d", dev->dvfs_app.pid);
					break;
				}

				// make sure the requested power level is valid
				if (cfgl.reql > dev->tbcfg_length) {
					DVFS_ERROR("Invalid power level requested (%d)", cfgl.reql);
					break;
				}

				// in manual mode, no one can change power levels, so we don't need any MUTEX
				if (dev->dvfs_mode == DVFS_MODE_MAN) {
					if (set_cpu_power_level(dev, cfgl.reql)) {
						DVFS_ERROR("Error has occurred during of transition between levels");
						break;
					} else {
						dev->cfglevel = cfgl.reql;
					}
				} else if (dev->dvfs_mode == DVFS_MODE_SEMI) {
					//change levels only if it's not temperature locked
					if (dev->cfglevel != cfgl.reql) {
						if (!dvfs_table[cfgl.reql].locked_by_temp) {
							DVFS_MUTEX_LOCK
							if (set_cpu_power_level(dev, cfgl.reql)) {
								DVFS_MUTEX_UNLOCK
								DVFS_ERROR("Error has occurred during of transition between levels");
								break;
							} else {
								dev->cfglevel = cfgl.reql;
							}
							DVFS_MUTEX_UNLOCK
						}
					}
				}
				ret = _RESMGR_ERRNO(EOK);
			}
				break;
			case DVFS_DEVCTL_RUN_MAX:
			{
				dvfs_table_t *dvfs_table = dev->dvfs_table;
				int cur_index;

				/* only the app that successfully set mode to "manual" or "semi-auto"
				* is allowed to change levels.
				*/
				if (dev->dvfs_mode == DVFS_MODE_AUTO || cfgl.capp_pid != dev->dvfs_app.pid) {
					DVFS_ERROR("Auto Mode detected OR request is not from pid %d", dev->dvfs_app.pid);
					break;
				}

				// in manual mode, no one can change power levels, so we don't need DVFS_MUTEX
				if (dev->dvfs_mode == DVFS_MODE_MAN) {
					if (set_cpu_power_level(dev, 0)) {
						DVFS_ERROR("Error has occurred during of transition between levels");
						break;
					} else {
						dev->cfglevel = 0;
					}
				} else if (dev->dvfs_mode == DVFS_MODE_SEMI) {
					//find the highest level that is not temperature locked
					if (dev->cfglevel > 0) {
						// find the highest ALLOWED power level
						for (cur_index=0; cur_index < dev->tbcfg_length; cur_index++) {
							// update only if the next level is not locked
							if (cur_index < dev->cfglevel) {
								if (!dvfs_table[cur_index].locked_by_temp) {
									DVFS_MUTEX_LOCK
									// call appropriate function
									if (set_cpu_power_level(dev, dvfs_table[cur_index].pwr_lvl) != EOK) {
										DVFS_MUTEX_UNLOCK
										DVFS_ERROR("Error has occurred during of transition between levels");
										break;
									} else {
										dev->cfglevel = cur_index;
									}
									DVFS_MUTEX_UNLOCK
									break;
								}
							}
						}
					}
				}
				ret = _RESMGR_ERRNO(EOK);
			}
				break;
			case DVFS_DEVCTL_RUN_MIN:
			{
				/* only the app that successfully set mode to "manual" or "semi-auto"
				* is allowed to change levels.
				*/
				if (dev->dvfs_mode == DVFS_MODE_AUTO || cfgl.capp_pid != dev->dvfs_app.pid) {
					DVFS_ERROR("Auto Mode detected OR request is not from pid %d", dev->dvfs_app.pid);
					break;
				}

				if (set_cpu_power_level(dev, dev->tbcfg_length - 1)) {
					DVFS_ERROR("Error has occurred during of transition between levels");
					break;
				} else {
					dev->cfglevel = dev->tbcfg_length - 1;
				}
				ret = _RESMGR_ERRNO(EOK);
			}
				break;
			case DVFS_DEVCTL_GETSTATUS:
			{
				dvfs_status_t	stat;
				cpustate_t		*cpustate;
				int			nbytes;
				int			i;

				cpustate = dev->cpustate;
				for (i = 0; i<dev->num_cpu; i++) {
					stat.cpuload[i] = cpustate->cpuload;
					cpustate++;
				}
				stat.temperature = dev->ctemp;
				stat.pwrlevl = dev->cfglevel;
				stat.mode = dev->dvfs_mode;
				stat.total_pwrlvl = dev->tbcfg_length;
				stat.freq = get_pwr_lvl_freq(dev, dev->cfglevel);
				stat.volt = get_pwr_lvl_volt(dev, dev->cfglevel);

				nbytes = sizeof(dvfs_status_t);
				memcpy((dvfs_status_t *)(_DEVCTL_DATA(msg->o)), &stat , nbytes);
				msg->o.ret_val = 0;
				msg->o.nbytes = nbytes;
				return _RESMGR_PTR(ctp, msg, sizeof(msg->o) + nbytes);
			}
			break;
			default:
				DVFS_ERROR("Unknown command");
				break;
		}
	} while (0);
	return ret;
}

/**
 * dvfs_shutdown: DVFS Shutdown function
 * - Description: This function performs DVFS shutdown initiated either by error
 * or user
 * - Arguments:
 * - signo: Type of signal received
 */
void dvfs_shutdown(int signo)
{
	switch (signo) {
		case SIGHUP:
		case SIGINT:
		case SIGQUIT:
		case SIGTERM:

			if (gdvfs == NULL) {
				// Shutdown is already completed. Nothing left to do
				break;
			}

			// terminate control thread
			if (gdvfs->tid > 0) {
				DVFS_INFO(gdvfs, 1, "DVFS Shutdown Initiated...");
				MsgSendPulse(gdvfs->coid, gdvfs->prio, DVFS_CTRLTH_PULSE_TERMINATE, 0);

				pthread_join(gdvfs->tid, NULL);
				gdvfs->tid = 0;

				// Set to lowest power level for safety
				DVFS_ERROR("Setting to lowest possible power level (lvl %d)(%dHz)(%duV)", gdvfs->tbcfg_length - 1,
					get_pwr_lvl_freq(gdvfs, gdvfs->tbcfg_length - 1), get_pwr_lvl_volt(gdvfs, gdvfs->tbcfg_length - 1));
				set_cpu_power_level(gdvfs, gdvfs->tbcfg_length - 1);
			}

			// terminate thermal thread
			if (gdvfs->therm_tid > 0) {
				DVFS_INFO(gdvfs, 4, "Thermal Thread Terminated");
				pthread_cancel(gdvfs->therm_tid);
				gdvfs->therm_tid = 0;
			}

			// terminate background thread
			if (gdvfs->bg_tid > 0) {
				DVFS_INFO(gdvfs, 4, "Background Thread Terminated");
				pthread_cancel(gdvfs->bg_tid);
				gdvfs->bg_tid = 0;
			}

			// terminate wfi thread
			if (gdvfs->wfi_tid > 0) {
				DVFS_INFO(gdvfs, 4, "WFI Threads Terminated");
				pthread_cancel(gdvfs->wfi_tid);
				gdvfs->wfi_tid = 0;
			}

			if (gdvfs->coid > 0) {
				ConnectDetach(gdvfs->coid);
			}
			if (gdvfs->chid > 0) {
				ChannelDestroy(gdvfs->chid);
			}

			if (gdvfs->dvfs_pwrtbl != NULL) {
				free(gdvfs->dvfs_pwrtbl);
				gdvfs->dvfs_pwrtbl_num_lvls = 0;
			}

			if (gdvfs->dvfs_table != NULL) {
				int i;
				related_apps_t *p = NULL;
				for (i = 0; i < gdvfs->tbcfg_length; i++) {
					while (gdvfs->dvfs_table[i].rapps != NULL) {
						p = gdvfs->dvfs_table[i].rapps;
						gdvfs->dvfs_table[i].rapps = (related_apps_t *) gdvfs->dvfs_table[i].rapps->next_app;
						if (p->notify_coid > 0) {
							MsgSendPulse(p->notify_coid, -1, DVFS_NOTIFY_TERMINATED, 0);
							if (ConnectDetach(p->notify_coid)) {
								DVFS_ERROR("Unable to detach connection");
							}
						}
						free(p);
					}
				}
				free(gdvfs->dvfs_table);
				gdvfs->dvfs_table = NULL;
				gdvfs->tbcfg_length = 0;
			}

			if (gdvfs->cpustate != NULL) {
				if (gdvfs->cpustate->tid > 0) {
					dvfs_fini(gdvfs);
					DVFS_INFO(gdvfs, 4, "CPU Account Threads Terminated");
					gdvfs->cpustate->tid = 0;
				}
			}

			if (gdvfs->cpustate != NULL) {
				free(gdvfs->cpustate);
			}

			pthread_mutex_destroy(&gdvfs->dvfs_mutex);
			pthread_mutex_destroy(&gdvfs->dvfs_mode_mutex);
			pthread_mutex_destroy(&gdvfs->dvfs_rapp_mutex);

			if (gdvfs != NULL) {
				DVFS_INFO(gdvfs, 1, "Shutdown Completed Successfully");
				free(gdvfs);
				gdvfs = NULL;
			}
			break;
		default:
			break;
	}
	return;
}

int main(int argc, char **argv)
{
	dvfs_t			*dev = NULL;
	resmgr_attr_t	resmgr_attr;
	dispatch_t		*dpp;
	dispatch_context_t *ctp;
	pthread_mutexattr_t mattr;
	int				ret;
	int				id;
	int				fd;

	/*
	* Request I/O privileges
	*/
	if (ThreadCtl(_NTO_TCTL_IO_PRIV, 0) == -1) {
		DVFS_ERROR("Unable to obtain I/O privileges");
		ret = EXIT_FAILURE;
		goto End;
	}

	if ((fd = open(DVFS_DEV_PATH, O_RDWR)) != -1) {
		DVFS_ERROR("Only 1 instance of DVFS is allowed to run");
		return -1;
	}

	// Allocate dvfs_t structure
	if ((dev = (dvfs_t *) calloc( 1, sizeof( dvfs_t ))) == NULL ) {
		ret = EXIT_FAILURE;
		goto End;
	}

	gdvfs = dev;

	dev->num_cpu = _syspage_ptr->num_cpu;

	// parse command-line options
	if (dvfs_parse_options(dev,argc,argv) != EOK) {
		DVFS_ERROR("Invalid options - See use file");
		ret = EXIT_FAILURE;
		goto End;
	}

	// initialize mutex
	pthread_mutexattr_init(&mattr);
	pthread_mutexattr_setrecursive(&mattr, PTHREAD_RECURSIVE_ENABLE);

	if ( pthread_mutex_init( &dev->dvfs_mutex, &mattr ) != EOK ) {
		DVFS_ERROR(" Unable to initialize mutex");
		ret = EXIT_FAILURE;
		goto End;
	}

	if ( pthread_mutex_init( &dev->dvfs_mode_mutex, &mattr ) != EOK ) {
		DVFS_ERROR(" Unable to initialize mode mutex");
		ret = EXIT_FAILURE;
		goto End;
	}

	if ( pthread_mutex_init( &dev->dvfs_rapp_mutex, &mattr ) != EOK ) {
		DVFS_ERROR(" Unable to initialize rapp mutex");
		ret = EXIT_FAILURE;
		goto End;
	}

	// Attempt to populate user-provided config table
	dev->dvfs_pwrtbl = NULL;
	dev->dvfs_pwrtbl_num_lvls = 0;
	if (dvfs_populate_pwrtbl(dev) != EOK) {
		DVFS_ERROR("Could not populate power table");
		// Don't exit. A table may not be needed
	}

	// perform any early cpu initialization (if neccessary)
	if (dvfs_pre_init(dev) != EOK) {
		DVFS_ERROR("CPU pre-initialization failed");
		ret = EXIT_FAILURE;
		goto End;
	}

	// initialize/process configuration table
	if (dvfs_dvfs_table_init(dev) != EOK) {
		DVFS_ERROR("Could not initialize DVFS table");
		ret = EXIT_FAILURE;
		goto End;
	}

	// Allocate Per CPU state structure
	if (dvfs_init_cpustate(dev) != EOK) {
		DVFS_ERROR("Could not initialize per CPU state structure");
		ret = EXIT_FAILURE;
		goto End;
	}

#ifdef	__ARM__
	// create wfi threads
	if (dev->wfi_workaround) {
		if (dvfs_create_wfi_threads(dev) != EOK) {
			ret = EXIT_FAILURE;
			goto End;
		}
	}
#endif

	// create background thread
	if (dvfs_create_background_thread(dev) != EOK) {
		ret = EXIT_FAILURE;
		goto End;
	}

	/* Run at maximum power level initially */
	dev->cfglevel = -1;
	DVFS_INFO(dev, 1, "Initial CPU Frequency : %dHz", get_cpu_freq(dev));
	if (set_cpu_power_level(dev, dev->dvfs_table[0].pwr_lvl) != EOK) {
		DVFS_ERROR("Could not set to maximum power level on startup");
		ret = EXIT_FAILURE;
		goto End;
	}
	DVFS_INFO(dev, 1, "Starting CPU Frequency : %dHz", get_cpu_freq(dev));
	dev->cfglevel = 0;

	// create control thread
	if (dvfs_create_ctrl_thread(dev) != EOK) {
		ret = EXIT_FAILURE;
		goto End;
	}

	/* SoC-specific initialization
	* may also include thread creation for cpu load calculation (in
	* case of sw cpu load) or interrupts (in case of hw cpu
	* load)
	*/
	if (dvfs_init(dev) != EOK) {
		ret = EXIT_FAILURE;
		goto End;
	}

	/**
	* Initialize Thermal modules
	*/
	if (dvfs_therm_init(dev) != EOK) {
		ret = EXIT_FAILURE;
		goto End;
	}

	// create thermal thread
	if (dvfs_create_therm_thread(dev) != EOK) {
		ret = EXIT_FAILURE;
		goto End;
	}

	dev->act.sa_handler = dvfs_shutdown;
	sigaction(SIGHUP, &dev->act, &dev->oact);
	sigaction(SIGINT, &dev->act, &dev->oact);
	sigaction(SIGQUIT, &dev->act, &dev->oact);
	sigaction(SIGTERM, &dev->act, &dev->oact);


	if ((dpp = dispatch_create()) == NULL) {
		DVFS_ERROR("%s: Unable to allocate dispatch handle.", argv[0]);
		ret = EXIT_FAILURE;
		goto End;
	}

	memset(&resmgr_attr, 0, sizeof(resmgr_attr_t));
	resmgr_attr.nparts_max	= 1;
	resmgr_attr.msg_max_size = 2048;

	iofunc_func_init(_RESMGR_CONNECT_NFUNCS, &connect_funcs,
					_RESMGR_IO_NFUNCS, &io_funcs);
	iofunc_attr_init(&attr, S_IFNAM | 0666, NULL, NULL);

	io_funcs.read	= dvfs_io_read;
	io_funcs.devctl = dvfs_io_devctl;

	if ((id = resmgr_attach(dpp, &resmgr_attr, DVFS_DEV_PATH,
							_FTYPE_ANY, 0, &connect_funcs,
							&io_funcs, &attr)) == -1) {
		DVFS_ERROR("%s: Unable to attach name.", argv[0]);
		ret = EXIT_FAILURE;
		goto End;
	}

	ctp = dispatch_context_alloc(dpp);
	procmgr_daemon(EXIT_SUCCESS, PROCMGR_DAEMON_NOCHDIR | PROCMGR_DAEMON_NOCLOSE | PROCMGR_DAEMON_NODEVNULL);

	/* Drop Root */
	if (UserParm != NULL) {
        if(procmgr_ability( 0,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_KEYDATA,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_IO,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_MEM_PHYS,
                            PROCMGR_AOP_ALLOW | PROCMGR_ADN_NONROOT | PROCMGR_AID_INTERRUPT,
                            PROCMGR_AOP_DENY  | PROCMGR_ADN_NONROOT | PROCMGR_AOP_LOCK      | PROCMGR_AID_EOL)
                             != EOK){
            DVFS_ERROR("%s: Unable to gain procmgr abilities for nonroot operation.", argv[0]);
            ret = EXIT_FAILURE;
            goto End;
        }
        if(set_ids_from_arg(UserParm) != EOK){
            DVFS_ERROR("%s: Unable to drop to user %s: %s", argv[0], UserParm, strerror(errno));
            ret = EXIT_FAILURE;
            goto End;
        }
        free(UserParm);
    }

	while (1) {
		if ((ctp = dispatch_block(ctp)) == NULL) {
			goto End;
		}
		dispatch_handler(ctp);
	}

End:
	// perform any shutdown tasks
	dvfs_shutdown(SIGTERM);
	return ret;
}



#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/dvfsmgr/dvfs.c $ $Rev: 887556 $")
#endif
