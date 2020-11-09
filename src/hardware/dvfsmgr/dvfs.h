/*
 * $QNXLicenseC:
 * Copyright 2012, QNX Software Systems.
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

#ifndef DVFS_H
#define DVFS_H

#include <stdio.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <errno.h>
#include <unistd.h>
#include <devctl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/mman.h>
#include <sys/iofunc.h>
#include <pthread.h>
#include <sys/dispatch.h>
#include <sys/sysmgr.h>
#include <sys/sysmsg.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/syspage.h>
#include <fcntl.h>
#include <sys/slogcodes.h>
#include <sys/slog.h>
#include <time.h>
#include <stdarg.h>
#include <hw/inout.h>
#include <sys/trace.h>
#include <sys/neutrino.h>
#include <sys/procmgr.h>
#include <hw/dvfs_api.h>
#include "dvfs_priv_api.h"
#include "variant.h"

/* Maximum number of supported cores */
#define MAX_NUM_CPU 32

/* Definition used to identify the source of power level enable/disable */
#define DVFS_SCALE_APPLICATION 0
#define DVFS_SCALE_THERMAL_MON 1

/* PULSES/MSGS send to CTRL THREAD */
#define DVFS_CTRLTH_PULSE_INTERRUPT        0x10           // DVFS Interrupt pulse
#define DVFS_CTRLTH_PULSE_TERMINATE        0x11           // Control thread pulse: Termination pulse
#define DVFS_CTRLTH_PULSE_THERMAL          0x12           // Control thread pulse: New measured CPU temperature
#define DVFS_CTRLTH_PULSE_CPUINC           0x13           // Control thread pulse: Increase power Level
#define DVFS_CTRLTH_PULSE_CPUDEC           0x14           // Control thread pulse: Decrease power level
#define DVFS_CTRLTH_PULSE_PANIC            0x15           // Control thread pulse: Jump to maximum power level
#define DVFS_CTRLTH_PULSE_RELAX            0x16           // Control thread pulse: Jump to lowest power level (not currently supported)
#define DVFS_CTRLTH_PULSE_SETLVL           0x17           // Control thread pulse: Set to specific power level
#define DVFS_CTRLTH_PULSE_NONE             0x99           // No Pulse
/* PANIC Threshold */
#define DVFS_PANIC_THRESH 95

/**
 * Thread Default Priority
 */
#define THREAD_PRIO_DEFAULT 10
#define THREAD_PRIO_CPUACT  51
#define THREAD_PRIO_IDLE    1
#define THREAD_PRIO_CTRL    (THREAD_PRIO_DEFAULT)
#define THREAD_PRIO_BG      (THREAD_PRIO_DEFAULT)

#define MAX_PWR_LVLS        20 // Maximum possible power levels

typedef struct __dvfs         dvfs_t;
typedef struct __cpustate     cpustate_t;
typedef struct __dvfs_pwrtbl  dvfs_pwrtbl_t;

/**
 * Message structure used by CPU load-calculating threads
 */
struct _cpumsg {
    int msg_code;
    int cpu0_load;
    int cpu1_load;
    int lvl;
};

/**
 * This structure is a linked-list of applications that have requested
 * a minimum level of performance for each power level entry
 */
typedef struct related_apps{
    pid_t               pid;
    int                 notify_coid;
    struct related_apps *next_app;
} related_apps_t;


/**
 * This structure contains the DVFS power table. Not all entries are mandatory.
 * Also, some entries might be specific to certain hardware and may be ignored by
 * others.
 */
typedef struct _dvfs_table{
    uint8_t        pwr_lvl;        // Power level corresponding to the configuration level
    int            up_thr;         // Upper threshold value of the configuration level
    int            down_thr;       // Down threshold value of the configuration level
    int            panic_thr;      // Panic threshold value of the configuration level
    int            up_temp_thr;    // Max temperature for this configuration level
    int            down_temp_thr;  // Temperature from which the level is allowed to operate
    int            up_cnt;         // Up counter threshold value
    int            down_cnt;       // Down counter threshold value
    int            emac;           // EMAC value
    int            disabled;       // This configuration level is disabled if the value more than zero
    int            locked_by_temp; // This level has been locked by thermal monitor
    related_apps_t *rapps;
} dvfs_table_t;


/**
 * Per cpu structure used to manage various cpu-related states/values.
 */
struct __cpustate{
    int             cpuid;           // cpu number
    int             cpuload;         // current CPU load
    int             preload;         // previously calculated CPU load
    int             desired_index;   // desired state index
    pthread_t       tid;             // thread ID of the CPU accounting thread
    dvfs_t          *dev;            // pointer to global DVFS structure
    uint64_t        num_cycles_idle; // number of idle cycles of corresponding "wfi" thread
    int             percentage_idle; // number of idle cycles computed in percentage
};

/**
 * This structure is the main DVFS structure. It includes both common
 * and SoC-specific variables
 */
struct __dvfs {
    cpustate_t          *cpustate;                       // pointer to glo
    int                 accounting_interval;             // CPU load calculation interval (in ms)
    int                 prio;                            // priority of the CPU accounting thread
    int                 num_cpu;                         // total number of active cores
    int                 verbose;                         // verbosity level
    dvfs_table_t        *dvfs_table;                     // pointer to the dvfs power level table
    int                 chid;                            // main thread channel id
    int                 coid;                            // main thread connection id
    pthread_t           tid;                             // main thread id
    pthread_t           wfi_tid;                         // wfi thread id
    pthread_t           therm_tid;                       // thermal monitoring thread id
    pthread_t           bg_tid;                          // background thread id
    pthread_mutex_t     dvfs_mutex;                      // dvfs mutex
    pthread_mutex_t     dvfs_mode_mutex;                 // dvfs mode change mutex
    pthread_mutex_t     dvfs_rapp_mutex;                 // dvfs mode change mutex
    pthread_barrier_t   cpu_barrier;                     // dvfs thread barrier
    char                cfg_file_path[PATH_MAX];         // dvfs configuration file path
    char                cfg_pwr_table_path[PATH_MAX];    // dvfs power table file path
    int                 tbcfg_length;                    // length of the configuration table
    int                 cfglevel;                        // current power configuration level
    int                 cpu_x_load[MAX_NUM_CPU];         // a working copy of cpu loads
    int                 pre_x_load[MAX_NUM_CPU];         // a working copy of previous cpu loads
    int                 wfi_workaround;                  // option to enable wfi workaround
    int                 iid;                             // interrupt request identifier
    int                 ctemp;                           // current core temperature
    struct sigevent     int_ev;                          // Interrupt event
    struct sigaction    act;                             // action structure for termination signals
    struct sigaction    oact;                            // old action for termination signals
    uint8_t             dvfs_mode;                       // current dvfs operating mode [auto/semi/manua]
    related_apps_t      dvfs_app;                        // information regarding the application in control of manual/semi mode
    int                 enable_speed_grade;              // enable the speed grading
    dvfs_pwrtbl_t       *dvfs_pwrtbl;                    // Table containing the user-defined power table
    int                 dvfs_pwrtbl_num_lvls;            // Number of actual power levels
#if 0
    uint8_t             cpudec_numreq;                   // number of cpu decrease request received from hardware
    uint8_t             cpudec_maxreq;                   // maximum number of cpu decrease requests to be received by hardware
    uint8_t             cpuinc_numreq;                   // number of cpu increase request received from hardware
    uint8_t             cpuinc_maxreq;                   // maximum number of cpu increase requests to be received by hardware
#endif
} ;

struct __dvfs_pwrtbl {
#define PWR_OPP_LOW     0
#define PWR_OPP_NOM     1
#define PWR_OPP_OD      2
#define PWR_OPP_HIGH    3
#define PWR_OPP_UNKNOWN 99
    uint32_t sys_pll_freq;
    uint32_t cpu_freq;
    uint32_t cpu_voltage;
#if defined (iMX6_SOC)
    uint32_t soc_voltage;
#endif
#if defined (iMX8_SOC)
    uint32_t cpu_freq_cluster2;								// used for second CPU cluster
#endif
    uint32_t max_allowed_freq;
    uint32_t efuse_val;
    int      opp_lvl;
};
#define CPUDEC_MAXIMUM_REQUESTS 1
#define CPUINC_MAXIMUM_REQUESTS 1

#define INVALID_VOLTAGE         0
/* Utils */
int cpu_parse_options(void *handle, char *options);
void dvfslog(int severity, const char *func, int line, int verbose_level, const char *fmt, ...);
unsigned get_gcd(unsigned a, unsigned b);

#define DVFS_MUTEX_LOCK     if( pthread_mutex_lock( &dev->dvfs_mutex ) ) { \
        fprintf( stderr, "mutex lock %s %d\n", __FILE__, __LINE__ );     }
#define DVFS_MUTEX_UNLOCK     if( pthread_mutex_unlock( &dev->dvfs_mutex ) ) { \
        fprintf( stderr, "mutex lock %s %d\n", __FILE__, __LINE__ ); }

#define DVFS_MODE_MUTEX_LOCK     if( pthread_mutex_lock( &dev->dvfs_mode_mutex ) ) { \
        fprintf( stderr, "mode mutex lock %s %d\n", __FILE__, __LINE__ );     }
#define DVFS_MODE_MUTEX_UNLOCK     if( pthread_mutex_unlock( &dev->dvfs_mode_mutex ) ) { \
        fprintf( stderr, "mode mutex lock %s %d\n", __FILE__, __LINE__ ); }

#define DVFS_RAPP_MUTEX_LOCK     if( pthread_mutex_lock( &dev->dvfs_rapp_mutex ) ) { \
        fprintf( stderr, "rapp mutex lock %s %d\n", __FILE__, __LINE__ );     }
#define DVFS_RAPP_MUTEX_UNLOCK     if( pthread_mutex_unlock( &dev->dvfs_rapp_mutex ) ) { \
        fprintf( stderr, "rapp mutex lock %s %d\n", __FILE__, __LINE__ ); }


#define DVFS_BARRIER_WAIT pthread_barrier_wait(&dev->cpu_barrier);


#define DVFS_INFO(handle, verbose_level, format, args... ) \
            do { if(handle->verbose >= verbose_level) dvfslog( _SLOG_INFO, __PRETTY_FUNCTION__, __LINE__, verbose_level, format, ##args ); } while(0)
#define DVFS_WARNING(handle, verbose_level, format, args... ) \
            do { if(handle->verbose >= verbose_level) dvfslog( _SLOG_WARNING, __PRETTY_FUNCTION__, __LINE__, verbose_level, format, ##args ); } while(0)
#define DVFS_ERROR( format, args... ) \
            do { dvfslog( _SLOG_ERROR, __PRETTY_FUNCTION__, __LINE__, 0, format, ##args ); } while(0)

#define DIV_ROUND(n,d)      (((n) + ((d)/2)) / (d))

/* Drop-root */
char *UserParm;
#endif

#if defined(__QNXNTO__) && defined(__USESRCVERSION)
#include <sys/srcversion.h>
__SRCVERSION("$URL: http://svn.ott.qnx.com/product/branches/7.0.0/trunk/hardware/dvfsmgr/dvfs.h $ $Rev: 887556 $")
#endif
