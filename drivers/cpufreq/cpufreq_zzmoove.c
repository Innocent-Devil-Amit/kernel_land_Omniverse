/*
 *  drivers/cpufreq/cpufreq_zzmoove.c
 *
 *  Copyright (C)  2001 Russell King
 *            (C)  2003 Venkatesh Pallipadi <venkatesh.pallipadi@intel.com>
 *                      Jun Nakajima <jun.nakajima@intel.com>
 *            (C)  2009 Alexander Clouter <alex@digriz.org.uk>
 *            (C)  2012 Michael Weingaertner <mialwe@googlemail.com>
 *                      Zane Zaminsky <cyxman@yahoo.com>
 *                      Jean-Pierre Rasquin <yank555.lu@gmail.com>
 *                      ffolkes <ffolkes@ffolkes.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * -------------------------------------------------------------------------------------------------------------------------------------------------------
 * -  Description:																	 -
 * -------------------------------------------------------------------------------------------------------------------------------------------------------
 *
 * 'ZZMoove' governor is based on the modified 'conservative' (original author Alexander Clouter <alex@digriz.org.uk>) 'smoove' governor from Michael
 * Weingaertner <mialwe@googlemail.com> (source: https://github.com/mialwe/mngb/) ported/modified/optimzed for I9300 since November 2012 and further
 * improved for exynos and snapdragon platform (but also working on other platforms like OMAP) by ZaneZam,Yank555 and ffolkes in 2013/14/15
 * CPU Hotplug modifications partially taken from ktoonservative governor from ktoonsez KT747-JB kernel (https://github.com/ktoonsez/KT747-JB)
 *
 * -------------------------------------------------------------------------------------------------------------------------------------------------------
 * -																			 -
 * -------------------------------------------------------------------------------------------------------------------------------------------------------
 */

// ZZ: disable kernel power management
// #define DISABLE_POWER_MANAGEMENT

// AP: use msm8974 lcd status notifier
// #define USE_LCD_NOTIFIER

#include <linux/cpu.h>
#ifdef USE_LCD_NOTIFIER
#include <linux/lcd_notify.h>
#endif
#include <linux/cpufreq.h>
#if defined(CONFIG_HAS_EARLYSUSPEND) && !defined (DISABLE_POWER_MANAGEMENT)
#include <linux/earlysuspend.h>
#endif
#ifdef CONFIG_EXYNOS4_EXPORT_TEMP
#include <linux/exynos4_export_temp.h>		// ZZ: Exynos4 temperatue reading support
#endif
#include <linux/hrtimer.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#if defined(CONFIG_POWERSUSPEND) && !defined (DISABLE_POWER_MANAGEMENT)
#include <linux/powersuspend.h>
#endif
#include <linux/sched.h>
#include <linux/tick.h>
#include <linux/version.h>

// #define ENABLE_SNAP_THERMAL_SUPPORT		// ZZ: Snapdragon temperature tripping support

#if defined(CONFIG_THERMAL_TSENS8974) || defined(CONFIG_THERMAL_TSENS8960) && defined(ENABLE_SNAP_THERMAL_SUPPORT) // ZZ: Snapdragon temperature sensor
#include <linux/msm_tsens.h>
#endif

// #define ENABLE_INPUTBOOSTER			// ZZ: enable/disable inputbooster support
// #define ENABLE_WORK_RESTARTLOOP		// ZZ: enable/disable restart loop for touchboost

#ifdef ENABLE_INPUTBOOSTER
#include <linux/slab.h>
#include <linux/input.h>
#endif

// Yank: enable/disable sysfs interface to display current zzmoove version
#define ZZMOOVE_VERSION "1.0 beta7"

// ZZ: support for 2,4 or 8 cores (this will enable/disable hotplug threshold tuneables and limit hotplug max limit tuneable)
#define MAX_CORES					(4)

// ZZ: enable/disable hotplug support
#define ENABLE_HOTPLUGGING

// ZZ: enable support for native hotplugging on snapdragon platform
// #define SNAP_NATIVE_HOTPLUGGING

// ZZ: enable for sources with backported cpufreq implementation of 3.10 kernel
// #define CPU_IDLE_TIME_IN_CPUFREQ

// ZZ: include profiles header file and set name for 'custom' profile (informational for a changed profile value)
#include "cpufreq_zzmoove_profiles.h"
#define DEF_PROFILE_NUMBER				(0)	// ZZ: default profile number (profile = 0 = 'none' = tuneable mode)
static char custom_profile[20] = "custom";			// ZZ: name to show in sysfs if any profile value has changed

// ff: allows tuneables to be tweaked without reverting to "custom" profile
#define DEF_PROFILE_STICKY_MODE				(1)

// Yank: enable/disable debugging code
// #define ZZMOOVE_DEBUG

/*
 * The polling frequency of this governor depends on the capability of
 * the processor. Default polling frequency is 1000 times the transition
 * latency of the processor. The governor will work on any processor with
 * transition latency <= 10mS, using appropriate samplingrate. For CPUs
 * with transition latency > 10mS (mostly drivers with CPUFREQ_ETERNAL)
 * this governor will not work. All times here are in uS.
 */
#define TRANSITION_LATENCY_LIMIT	    (10 * 1000 * 1000)	// ZZ: default transition latency limit
#define LATENCY_MULTIPLIER				(1000)	// ZZ: default latency multiplier
#define MIN_LATENCY_MULTIPLIER				(100)	// ZZ: default min latency multiplier
#define MIN_SAMPLING_RATE_RATIO				(2)	// ZZ: default min sampling rate ratio

// ZZ: general tuneable defaults
#define DEF_FREQUENCY_UP_THRESHOLD			(70)	// ZZ: default regular scaling up threshold
#ifdef ENABLE_HOTPLUGGING
#define DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG		(68)	// ZZ: default hotplug up threshold for all cpus (cpu0 stays allways on)
#define DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG_FREQ		(0)	// Yank: default hotplug up threshold frequency for all cpus (0 = disabled)
#endif /* ENABLE_HOTPLUGGING */
#define DEF_SMOOTH_UP					(75)	// ZZ: default cpu load trigger for 'boosting' scaling frequency
#define DEF_FREQUENCY_DOWN_THRESHOLD			(52)	// ZZ: default regular scaling down threshold
#ifdef ENABLE_HOTPLUGGING
#define DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG		(55)	// ZZ: default hotplug down threshold for all cpus (cpu0 stays allways on)
#define DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG_FREQ	(0)	// Yank: default hotplug down threshold frequency for all cpus (0 = disabled)
#endif /* ENABLE_HOTPLUGGING */
#define DEF_IGNORE_NICE					(0)	// ZZ: default ignore nice load
#define DEF_AUTO_ADJUST_FREQ				(0)	// ZZ: default auto adjust frequency thresholds

// ZZ: hotplug-switch, -block, -idle, -limit and scaling-block, -fastdown, -responiveness, -proportional tuneable defaults
#ifdef ENABLE_HOTPLUGGING
#define DEF_DISABLE_HOTPLUG				(0)	// ZZ: default hotplug switch
#define DEF_HOTPLUG_BLOCK_UP_CYCLES			(0)	// ZZ: default hotplug up block cycles
#define DEF_HOTPLUG_BLOCK_DOWN_CYCLES			(0)	// ZZ: default hotplug down block cycles
#define DEF_BLOCK_UP_MULTIPLIER_HOTPLUG1		(0)	// ff: default hotplug up block multiplier for core 2
#define DEF_BLOCK_UP_MULTIPLIER_HOTPLUG2		(0)	// ff: default hotplug up block multiplier for core 3
#define DEF_BLOCK_UP_MULTIPLIER_HOTPLUG3		(0)	// ff: default hotplug up block multiplier for core 4
#define DEF_BLOCK_DOWN_MULTIPLIER_HOTPLUG1		(0)	// ff: default hotplug down block multiplier for core 2
#define DEF_BLOCK_DOWN_MULTIPLIER_HOTPLUG2		(0)	// ff: default hotplug down block multiplier for core 3
#define DEF_BLOCK_DOWN_MULTIPLIER_HOTPLUG3		(0)	// ff: default hotplug down block multiplier for core 4
#define DEF_HOTPLUG_STAGGER_UP				(0)	// ff: only bring one core up at a time when hotplug_online_work() called
#define DEF_HOTPLUG_STAGGER_DOWN			(0)	// ff: only bring one core down at a time when hotplug_offline_work() called
#define DEF_HOTPLUG_IDLE_THRESHOLD			(0)	// ZZ: default hotplug idle threshold
#define DEF_HOTPLUG_IDLE_FREQ				(0)	// ZZ: default hotplug idle freq
#define DEF_HOTPLUG_ENGAGE_FREQ				(0)	// ZZ: default hotplug engage freq. the frequency below which we run on only one core (0 = disabled) (ffolkes)
#define DEF_HOTPLUG_MAX_LIMIT				(0)	// ff: default for hotplug_max_limit. the number of cores we allow to be online (0 = disabled)
#define DEF_HOTPLUG_MIN_LIMIT				(0)	// ff: default for hotplug_min_limit. the number of cores we require to be online (0 = disabled)
#define DEF_HOTPLUG_LOCK				(0)	// ff: default for hotplug_lock. the number of cores we require to be online (0 = disabled)
#endif /* ENABLE_HOTPLUGGING */
#define DEF_SCALING_BLOCK_THRESHOLD			(0)	// ZZ: default scaling block threshold
#define DEF_SCALING_BLOCK_CYCLES			(0)	// ZZ: default scaling block cycles
#define DEF_SCALING_BLOCK_FREQ				(0)	// ZZ: default scaling block freq
#define DEF_SCALING_UP_BLOCK_CYCLES			(0)	// ff: default scaling-up block cycles
#define DEF_SCALING_UP_BLOCK_FREQ			(0)	// ff: default scaling-up block frequency threshold
#ifdef CONFIG_EXYNOS4_EXPORT_TEMP
#define DEF_TMU_READ_DELAY				(1000)	// ZZ: delay for cpu temperature reading in ms (tmu driver polling intervall is 10 sec)
#define DEF_SCALING_BLOCK_TEMP				(0)	// ZZ: default cpu temperature threshold in �C
#endif /* CONFIG_EXYNOS4_EXPORT_TEMP */
#ifdef ENABLE_SNAP_THERMAL_SUPPORT				// ff: snapdragon temperature tripping defaults
#define DEF_SCALING_TRIP_TEMP				(60)	// ff: default trip cpu temp
#define DEF_TMU_CHECK_DELAY				(2500)	// ZZ: default delay for snapdragon thermal tripping
#define DEF_TMU_CHECK_DELAY_SLEEP			(10000)	// ZZ: default delay for snapdragon thermal tripping at sleep
#endif /* ENABLE_SNAP_THERMAL_SUPPORT */
#define DEF_SCALING_BLOCK_FORCE_DOWN			(2)	// ZZ: default scaling block force down
#define DEF_SCALING_FASTDOWN_FREQ			(0)	// ZZ: default scaling fastdown freq. the frequency beyond which we apply a different up_threshold (ffolkes)
#define DEF_SCALING_FASTDOWN_UP_THRESHOLD		(95)	// ZZ: default scaling fastdown up threshold. the up threshold when scaling fastdown freq has been exceeded (ffolkes)
#define DEF_SCALING_FASTDOWN_DOWN_THRESHOLD		(90)	// ZZ: default scaling fastdown up threshold. the down threshold when scaling fastdown freq has been exceeded (ffolkes)
#define DEF_SCALING_RESPONSIVENESS_FREQ			(0)	// ZZ: default frequency below which we use a lower up threshold (ffolkes)
#define DEF_SCALING_RESPONSIVENESS_UP_THRESHOLD		(30)	// ZZ: default up threshold we use when below scaling responsiveness freq (ffolkes)
#define DEF_SCALING_PROPORTIONAL			(0)	// ZZ: default proportional scaling

// ZZ: sampling rate idle and sampling down momentum tuneable defaults
#define DEF_SAMPLING_RATE_IDLE_THRESHOLD		(0)	// ZZ: default sampling rate idle threshold
#define DEF_SAMPLING_RATE_IDLE				(180000)// ZZ: default sampling rate idle (must not be 0!)
#define DEF_SAMPLING_RATE_IDLE_DELAY			(0)	// ZZ: default sampling rate idle delay
#define DEF_SAMPLING_DOWN_FACTOR			(1)	// ZZ: default sampling down factor (stratosk default = 4) here disabled by default
#define MAX_SAMPLING_DOWN_FACTOR			(100000)// ZZ: changed from 10 to 100000 for sampling down momentum implementation
#define DEF_SAMPLING_DOWN_MOMENTUM			(0)	// ZZ: default sampling down momentum, here disabled by default
#define DEF_SAMPLING_DOWN_MAX_MOMENTUM			(0)	// ZZ: default sampling down max momentum stratosk default=16, here disabled by default
#define DEF_SAMPLING_DOWN_MOMENTUM_SENSITIVITY		(50)	// ZZ: default sampling down momentum sensitivity
#define MAX_SAMPLING_DOWN_MOMENTUM_SENSITIVITY		(1000)	// ZZ: default maximum for sampling down momentum sensitivity

#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
// ZZ: tuneable defaults for early suspend
#define MAX_SAMPLING_RATE_SLEEP_MULTIPLIER		(8)	// ZZ: default maximum for sampling rate sleep multiplier
#define DEF_SAMPLING_RATE_SLEEP_MULTIPLIER		(2)	// ZZ: default sampling rate sleep multiplier
#define DEF_UP_THRESHOLD_SLEEP				(90)	// ZZ: default up threshold sleep
#define DEF_DOWN_THRESHOLD_SLEEP			(44)	// ZZ: default down threshold sleep
#define DEF_SMOOTH_UP_SLEEP				(75)	// ZZ: default smooth up sleep
#define DEF_EARLY_DEMAND_SLEEP				(1)	// ZZ: default early demand sleep
#define DEF_GRAD_UP_THRESHOLD_SLEEP			(30)	// ZZ: default grad up sleep
#define DEF_FAST_SCALING_SLEEP_UP			(0)	// Yank: default fast scaling sleep for upscaling
#define DEF_FAST_SCALING_SLEEP_DOWN			(0)	// Yank: default fast scaling sleep for downscaling
#define DEF_FREQ_LIMIT_SLEEP				(0)	// ZZ: default freq limit sleep
#ifdef ENABLE_HOTPLUGGING
#define DEF_DISABLE_HOTPLUG_SLEEP			(0)	// ZZ: default hotplug sleep switch
#endif /* ENABLE_HOTPLUGGING */
#endif

/*
 * ZZ: Hotplug Sleep: 0 do not touch hotplug settings at suspend, so all cores will stay online
 * the value is equivalent to the amount of cores which should be online at suspend
 */
#ifdef ENABLE_HOTPLUGGING
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
#define DEF_HOTPLUG_SLEEP				(0)	// ZZ: default hotplug sleep
#endif
#endif /* ENABLE_HOTPLUGGING */

// ZZ: tuneable defaults for Early Demand
#define DEF_GRAD_UP_THRESHOLD				(25)	// ZZ: default grad up threshold
#define DEF_EARLY_DEMAND				(0)	// ZZ: default early demand, default off

/*
 * ZZ: Frequency Limit: 0 do not limit frequency and use the full range up to policy->max limit
 * values policy->min to policy->max in khz
 */
#define DEF_FREQ_LIMIT					(0)	// ZZ: default freq limit

/*
 * ZZ: Fast Scaling: 0 do not activate fast scaling function
 * values 1-4 to enable fast scaling and 5 for auto fast scaling (insane scaling)
 */
#define DEF_FAST_SCALING_UP				(0)	// Yank: default fast scaling for upscaling
#define DEF_FAST_SCALING_DOWN				(0)	// Yank: default fast scaling for downscaling
#define DEF_AFS_THRESHOLD1				(25)	// ZZ: default auto fast scaling step one
#define DEF_AFS_THRESHOLD2				(50)	// ZZ: default auto fast scaling step two
#define DEF_AFS_THRESHOLD3				(75)	// ZZ: default auto fast scaling step three
#define DEF_AFS_THRESHOLD4				(90)	// ZZ: default auto fast scaling step four

// ff: Input Booster defaults
#ifdef ENABLE_INPUTBOOSTER
#define DEF_INPUTBOOST_CYCLES				(0)	// ff: default number of cycles to boost up/down thresholds
#define DEF_INPUTBOOST_UP_THRESHOLD			(80)	// ff: default up threshold for inputbooster
#define DEF_INPUTBOOST_PUNCH_CYCLES			(20)	// ff: default number of cycles to meet or exceed punch freq
#define DEF_INPUTBOOST_PUNCH_FREQ			(1000000)// ff: default frequency to keep cur_freq at or above
#define DEF_INPUTBOOST_PUNCH_ON_FINGERDOWN		(0)	// ff: default for constant punching (like a touchbooster)
#define DEF_INPUTBOOST_PUNCH_ON_FINGERMOVE		(0)	// ff: default for constant punching (like a touchbooster)
#define DEF_INPUTBOOST_PUNCH_ON_EPENMOVE		(0)	// ff: default for constant punching (like a touchbooster)
#define DEF_INPUTBOOST_ON_TSP				(1)	// ff: default to boost on touchscreen input events
#define DEF_INPUTBOOST_ON_TSP_HOVER			(1)	// ff: default to boost on touchscreen hovering input events
#define DEF_INPUTBOOST_ON_GPIO				(1)	// ff: default to boost on gpio (button) input events
#define DEF_INPUTBOOST_ON_TK				(1)	// ff: default to boost on touchkey input events
#define DEF_INPUTBOOST_ON_EPEN				(1)	// ff: default to boost on e-pen input events
#define DEF_INPUTBOOST_TYPINGBOOSTER_UP_THRESHOLD	(40)	// ff: default up threshold for typing booster
#define DEF_INPUTBOOST_TYPINGBOOSTER_CORES		(3)	// ff: default cores for typing booster
#endif /* ENABLE_INPUTBOOSTER */

// ff: Music Detection defaults
#define DEF_MUSIC_MAX_FREQ				(0)	// ff: default maximum freq to maintain while music is on
#define DEF_MUSIC_MIN_FREQ				(0)	// ff: default minimum freq to maintain while music is on
#ifdef ENABLE_HOTPLUGGING
#define DEF_MUSIC_MIN_CORES				(2)	// ZZ: default minimum cores online while music is on
#endif

// ZZ: Sampling Down Momentum variables
static unsigned int min_sampling_rate;				// ZZ: minimal possible sampling rate
static unsigned int orig_sampling_down_factor;			// ZZ: for saving previously set sampling down factor
static unsigned int zz_sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR;		// ff: actual use variable, so dbs_tuner_ins version stays constant
static unsigned int orig_sampling_down_max_mom;			// ZZ: for saving previously set smapling down max momentum
static unsigned int zz_sampling_down_max_mom;			// ff: actual use variable, so dbs_tuner_ins version stays constant

// ZZ: search limit for frequencies in scaling table, variables for scaling modes and state flag for suspend detection
static struct cpufreq_frequency_table *system_freq_table;	// ZZ: static system frequency table
static int scaling_mode_up;					// ZZ: fast scaling up mode holding up value during runtime
static int scaling_mode_down;					// ZZ: fast scaling down mode holding down value during runtime
static bool freq_table_desc = false;				// ZZ: true for descending order, false for ascending order
static int freq_init_count = 0;					// ZZ: flag for executing freq table order and limit optimization code at gov start
static unsigned int max_scaling_freq_soft = 0;			// ZZ: init value for 'soft' scaling limit, 0 = full range
static unsigned int max_scaling_freq_hard = 0;			// ZZ: init value for 'hard' scaling limit, 0 = full range
static unsigned int min_scaling_freq_soft = 0;			// ZZ: init value for 'soft' scaling limit, 0 = full range
static unsigned int min_scaling_freq_hard = 0;			// ZZ: init value for 'hard' scaling limit, 0 = full range
static unsigned int system_table_end = CPUFREQ_TABLE_END;	// ZZ: system freq table end for order detection, table size calculation and freq validations
static unsigned int limit_table_end = CPUFREQ_TABLE_END;	// ZZ: initial (full range) search end limit for frequency table in descending ordered table
static unsigned int limit_table_start = 0;			// ZZ: search start limit for frequency table in ascending order
static unsigned int freq_table_size = 0;			// Yank: upper index limit of frequency table
static unsigned int min_scaling_freq = 0;			// Yank: lowest frequency index in global frequency table
static bool suspend_flag = false;				// ZZ: flag for suspend status, true = in early suspend

// ZZ: hotplug-, scaling-block, scaling fastdown vars and sampling rate idle counters. flags for scaling, setting profile, cpu temp reading and hotplugging
#ifdef ENABLE_HOTPLUGGING
static int possible_cpus = 0;					// ZZ: for holding the maximal amount of cores for hotplugging
static unsigned int hplg_down_block_cycles = 0;			// ZZ: delay cycles counter for hotplug down blocking
static unsigned int hplg_up_block_cycles = 0;			// ZZ: delay cycles counter for hotplug up blocking
static unsigned int num_online_cpus_last = 0;			// ff: how many cores were online last cycle
static unsigned int zz_hotplug_block_up_cycles = 0;
static unsigned int zz_hotplug_block_down_cycles = 0;
#endif /* ENABLE_HOTPLUGGING */
static unsigned int scaling_block_cycles_count = 0;		// ZZ: scaling block cycles counter
static unsigned int sampling_rate_step_up_delay = 0;		// ZZ: sampling rate idle up delay cycles
static unsigned int sampling_rate_step_down_delay = 0;		// ZZ: sampling rate idle down delay cycles
static unsigned int scaling_up_threshold = 0;			// ZZ: scaling up threshold for fastdown/responsiveness functionality
static unsigned int scaling_down_threshold = 0;			// ZZ: scaling down threshold for fastdown functionality
#ifdef ENABLE_HOTPLUGGING
static bool hotplug_idle_flag = false;				// ZZ: flag for hotplug idle mode
static int __refdata enable_cores = 0;				// ZZ: mode for enabling offline cores for various functions in the governor
static int __refdata disable_cores = 0;				// ZZ: mode for disabling online cores for various functions in the governor
static bool hotplug_up_in_progress;				// ZZ: flag for hotplug up function call - block if hotplugging is in progress
static bool hotplug_down_in_progress;				// ZZ: flag for hotplug down function call - block if hotplugging is in progress
static bool boost_hotplug = false;				// ZZ: early demand boost hotplug flag
#endif /* ENABLE_HOTPLUGGING */
static bool boost_freq = false;					// ZZ: early demand boost freq flag
static bool force_down_scaling = false;				// ZZ: force down scaling flag
static bool cancel_up_scaling = false;				// ZZ: cancel up scaling flag
static bool set_profile_active = false;				// ZZ: flag to avoid changing of any tuneables during profile apply
#ifdef CONFIG_EXYNOS4_EXPORT_TEMP
#ifdef ENABLE_HOTPLUGGING
static bool hotplug_up_temp_block;				// ZZ: flag for blocking up hotplug work during temp freq blocking
#endif /* ENABLE_HOTPLUGGING */
static bool cancel_temp_reading = false;			// ZZ: flag for starting temp reading work
static bool temp_reading_started = false;			// ZZ: flag for canceling temp reading work

// ZZ: Exynos CPU temp reading work
static void tmu_read_temperature(struct work_struct * tmu_read_work);	// ZZ: prepare temp reading work
static DECLARE_DELAYED_WORK(tmu_read_work, tmu_read_temperature);	// ZZ: declare delayed work for temp reading
static unsigned int cpu_temp;						// ZZ: static var for holding current cpu temp
#endif /* CONFIG_EXYNOS4_EXPORT_TEMP */

// ZZ: current load & frequency for hotplugging work and scaling. max/min frequency for proportional scaling and auto freq threshold adjustment
static unsigned int cur_load = 0;				// ZZ: current load for hotplugging functions
static unsigned int cur_freq = 0;				// ZZ: current frequency for hotplugging functions
static unsigned int pol_max = 0;				// ZZ: current max freq for proportional scaling and auto adjustment of freq thresholds
static unsigned int old_pol_max = 0;				// ZZ: previous max freq for auto adjustment of freq thresholds
static unsigned int pol_min = 0;				// ZZ: current min freq for auto adjustment of freq thresholds
static unsigned int pol_step = 0;				// ZZ: policy change step for auto adjustment of freq thresholds

// ZZ: temp variables and flags to hold offset values for auto adjustment of freq thresholds
#ifdef ENABLE_HOTPLUGGING
static unsigned int temp_hotplug_engage_freq = 0;
static bool temp_hotplug_engage_freq_flag = false;
static unsigned int temp_hotplug_idle_freq = 0;
static bool temp_hotplug_idle_freq_flag = false;
#endif /* ENABLE_HOTPLUGGING */
static unsigned int temp_scaling_block_freq = 0;
static bool temp_scaling_block_freq_flag = false;
static unsigned int temp_scaling_fastdown_freq = 0;
static bool temp_scaling_fastdown_freq_flag = false;
static unsigned int temp_scaling_responsiveness_freq = 0;
static bool temp_scaling_responsiveness_freq_flag = false;
#ifdef ENABLE_HOTPLUGGING
static unsigned int temp_up_threshold_hotplug_freq1 = 0;
static bool temp_up_threshold_hotplug_freq1_flag = false;
static unsigned int temp_down_threshold_hotplug_freq1 = 0;
static bool temp_down_threshold_hotplug_freq1_flag = false;
#if (MAX_CORES == 4 || MAX_CORES == 8)
static unsigned int temp_up_threshold_hotplug_freq2 = 0;
static bool temp_up_threshold_hotplug_freq2_flag = false;
static unsigned int temp_up_threshold_hotplug_freq3 = 0;
static bool temp_up_threshold_hotplug_freq3_flag = false;
static unsigned int temp_down_threshold_hotplug_freq2 = 0;
static bool temp_down_threshold_hotplug_freq2_flag = false;
static unsigned int temp_down_threshold_hotplug_freq3 = 0;
static bool temp_down_threshold_hotplug_freq3_flag = false;
#endif
#if (MAX_CORES == 8)
static unsigned int temp_up_threshold_hotplug_freq4 = 0;
static bool temp_up_threshold_hotplug_freq4_flag = false;
static unsigned int temp_up_threshold_hotplug_freq5 = 0;
static bool temp_up_threshold_hotplug_freq5_flag = false;
static unsigned int temp_up_threshold_hotplug_freq6 = 0;
static bool temp_up_threshold_hotplug_freq6_flag = false;
static unsigned int temp_up_threshold_hotplug_freq7 = 0;
static bool temp_up_threshold_hotplug_freq7_flag = false;
static unsigned int temp_down_threshold_hotplug_freq4 = 0;
static bool temp_down_threshold_hotplug_freq4_flag = false;
static unsigned int temp_down_threshold_hotplug_freq5 = 0;
static bool temp_down_threshold_hotplug_freq5_flag = false;
static unsigned int temp_down_threshold_hotplug_freq6 = 0;
static bool temp_down_threshold_hotplug_freq6_flag = false;
static unsigned int temp_down_threshold_hotplug_freq7 = 0;
static bool temp_down_threshold_hotplug_freq7_flag = false;
#endif
#ifdef ENABLE_INPUTBOOSTER
static unsigned int temp_inputboost_punch_freq = 0;
static bool temp_inputboost_punch_freq_flag = false;
#endif

// ZZ: hotplug load thresholds array
static int hotplug_thresholds[2][8] = {
    { 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 },
    { 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 }
    };

// ZZ: hotplug frequencies thresholds array
static int hotplug_thresholds_freq[2][8] = {
    { 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 },
    { 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 }
    };

// ZZ: hotplug frequencies out of range array
static int hotplug_freq_threshold_out_of_range[2][8] = {
    { 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 },
    { 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 }
    };
#endif /* ENABLE_HOTPLUGGING */

static unsigned int temp_music_min_freq = 0;
static bool temp_music_min_freq_flag = false;
static unsigned int temp_music_max_freq = 0;
static bool temp_music_max_freq_flag = false;

// ZZ: core on which we are running
static unsigned int on_cpu = 0;

// ZZ: Early Suspend variables
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
static unsigned int sampling_rate_awake;			// ZZ: for saving sampling rate awake value
static unsigned int up_threshold_awake;				// ZZ: for saving up threshold awake value
static unsigned int down_threshold_awake;			// ZZ: for saving down threshold awake value
static unsigned int smooth_up_awake;				// ZZ: for saving smooth up awake value
static unsigned int freq_limit_awake;				// ZZ: for saving frequency limit awake value
static unsigned int fast_scaling_up_awake;			// Yank: for saving fast scaling awake value for upscaling
static unsigned int fast_scaling_down_awake;			// Yank: for saving fast scaling awake value for downscaling
#ifdef ENABLE_HOTPLUGGING
static unsigned int disable_hotplug_awake;			// ZZ: for saving hotplug switch
static unsigned int hotplug1_awake;				// ZZ: for saving hotplug1 threshold awake value
#if (MAX_CORES == 4 || MAX_CORES == 8)
static unsigned int hotplug2_awake;				// ZZ: for saving hotplug2 threshold awake value
static unsigned int hotplug3_awake;				// ZZ: for saving hotplug3 threshold awake value
#endif
#if (MAX_CORES == 8)
static unsigned int hotplug4_awake;				// ZZ: for saving hotplug4 threshold awake value
static unsigned int hotplug5_awake;				// ZZ: for saving hotplug5 threshold awake value
static unsigned int hotplug6_awake;				// ZZ: for saving hotplug6 threshold awake value
static unsigned int hotplug7_awake;				// ZZ: for saving hotplug7 threshold awake value
#endif
#endif /* ENABLE_HOTPLUGGING */
static unsigned int sampling_rate_asleep;			// ZZ: for setting sampling rate value at early suspend
static unsigned int up_threshold_asleep;			// ZZ: for setting up threshold value at early suspend
static unsigned int down_threshold_asleep;			// ZZ: for setting down threshold value at early suspend
static unsigned int smooth_up_asleep;				// ZZ: for setting smooth scaling value at early suspend
static unsigned int freq_limit_asleep;				// ZZ: for setting frequency limit value at early suspend
static unsigned int fast_scaling_up_asleep;			// Yank: for setting fast scaling value at early suspend for upscaling
static unsigned int fast_scaling_down_asleep;			// Yank: for setting fast scaling value at early suspend for downscaling
#ifdef ENABLE_HOTPLUGGING
static unsigned int disable_hotplug_asleep;			// ZZ: for setting hotplug on/off at early suspend
#endif /* ENABLE_HOTPLUGGING */
#endif

#ifdef USE_LCD_NOTIFIER
static struct notifier_block zzmoove_lcd_notif;
#endif

#ifdef ENABLE_INPUTBOOSTER
// ff: Input Booster variables
static unsigned int boost_on_tsp = DEF_INPUTBOOST_ON_TSP;	// ff: hardcoded since it'd be silly not to use it
static unsigned int boost_on_tsp_hover = DEF_INPUTBOOST_ON_TSP_HOVER;
static unsigned int boost_on_gpio = DEF_INPUTBOOST_ON_GPIO;	// ff: hardcoded since it'd be silly not to use it
static unsigned int boost_on_tk = DEF_INPUTBOOST_ON_TK;		// ff: hardcoded since it'd be silly not to use it
static unsigned int boost_on_epen = DEF_INPUTBOOST_ON_EPEN;	// ff: hardcoded since it'd be silly not to use it
static unsigned int inputboost_last_type = 0;
static unsigned int inputboost_last_code = 0;
static unsigned int inputboost_last_value = 0;
static int inputboost_report_btn_touch = -1;
static int inputboost_report_btn_toolfinger = -1;
static int inputboost_report_mt_trackingid = 0;
static bool flg_inputboost_report_mt_touchmajor = false;
static bool flg_inputboost_report_abs_xy = false;
int flg_ctr_cpuboost = 0;
static int flg_ctr_inputboost = 0;
static int flg_ctr_inputboost_punch = 0;
static int flg_ctr_inputbooster_typingbooster = 0;
static int ctr_inputboost_typingbooster_taps = 0;
static struct timeval time_typingbooster_lasttapped;
#ifdef ZZMOOVE_DEBUG
static struct timeval time_touchbooster_lastrun;
static unsigned int time_since_touchbooster_lastrun = 0;
#endif
#endif /* ENABLE_INPUTBOOSTER */

// ff: other variables
static int scaling_up_block_cycles_count = 0;
static int music_max_freq_step = 0;

#ifdef ENABLE_WORK_RESTARTLOOP
struct work_struct work_restartloop;
static bool work_restartloop_in_progress = false;		// ZZ: flag to avoid loop restart to frequently
#endif

#ifdef ENABLE_SNAP_THERMAL_SUPPORT
static void tmu_check_work(struct work_struct * work_tmu_check);
static DECLARE_DELAYED_WORK(work_tmu_check, tmu_check_work);
static int tmu_temp_cpu = 0;
static int tmu_temp_cpu_last = 0;
static int flg_ctr_tmu_overheating = 0;
static int tmu_throttle_steps = 0;
static int ctr_tmu_neutral = 0;
static int ctr_tmu_falling = 0;
#endif

#ifdef ENABLE_HOTPLUGGING
struct work_struct hotplug_offline_work;			// ZZ: hotplugging down work
struct work_struct hotplug_online_work;				// ZZ: hotplugging up work
#endif /* ENABLE_HOTPLUGGING */

static void do_dbs_timer(struct work_struct *work);

struct cpu_dbs_info_s {
	u64 time_in_idle;					// ZZ: for exit time handling
	u64 idle_exit_time;					// ZZ: for exit time handling
	u64 prev_cpu_idle;
	u64 prev_cpu_wall;
	u64 prev_cpu_nice;
	struct cpufreq_policy *cur_policy;
	struct delayed_work work;
	unsigned int down_skip;					// ZZ: Sampling Down reactivated
	unsigned int requested_freq;
	unsigned int rate_mult;					// ZZ: Sampling Down Momentum - sampling rate multiplier
	unsigned int momentum_adder;				// ZZ: Sampling Down Momentum - adder
	int cpu;
	unsigned int enable:1;
	unsigned int prev_load;					// ZZ: Early Demand - for previous load

	/*
	 * percpu mutex that serializes governor limit change with
	 * do_dbs_timer invocation. We do not want do_dbs_timer to run
	 * when user is changing the governor or limits.
	 */
	struct mutex timer_mutex;
};

static bool dbs_info_enabled = false;
static DEFINE_PER_CPU(struct cpu_dbs_info_s, cs_cpu_dbs_info);
static unsigned int dbs_enable;					// number of CPUs using this policy
static DEFINE_MUTEX(dbs_mutex);					// dbs_mutex protects dbs_enable in governor start/stop.
static struct workqueue_struct *dbs_wq;
#ifdef ENABLE_WORK_RESTARTLOOP
static struct workqueue_struct *dbs_aux_wq;
#endif
static struct dbs_tuners {
	char profile[20];					// ZZ: profile tuneable
	unsigned int profile_number;				// ZZ: profile number tuneable
	unsigned int profile_sticky_mode;			// ff: sticky profile mode
	unsigned int auto_adjust_freq_thresholds;		// ZZ: auto adjust freq thresholds tuneable
	unsigned int sampling_rate;				// ZZ: normal sampling rate tuneable
	unsigned int sampling_rate_current;			// ZZ: currently active sampling rate tuneable
	unsigned int sampling_rate_idle;			// ZZ: sampling rate at idle tuneable
	unsigned int sampling_rate_idle_threshold;		// ZZ: sampling rate switching threshold tuneable
	unsigned int sampling_rate_idle_delay;			// ZZ: sampling rate switching delay tuneable
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	unsigned int sampling_rate_sleep_multiplier;		// ZZ: sampling rate sleep multiplier tuneable for early suspend
#endif
	unsigned int sampling_down_factor;			// ZZ: sampling down factor tuneable (reactivated)
	unsigned int sampling_down_momentum;			// ZZ: sampling down momentum tuneable
	unsigned int sampling_down_max_mom;			// ZZ: sampling down momentum max tuneable
	unsigned int sampling_down_mom_sens;			// ZZ: sampling down momentum sensitivity tuneable
	unsigned int up_threshold;				// ZZ: scaling up threshold tuneable
#ifdef ENABLE_HOTPLUGGING
	unsigned int up_threshold_hotplug1;			// ZZ: up threshold hotplug tuneable for core1
	unsigned int up_threshold_hotplug_freq1;		// Yank: up threshold hotplug freq tuneable for core1
	unsigned int block_up_multiplier_hotplug1;
#if (MAX_CORES == 4 || MAX_CORES == 8)
	unsigned int up_threshold_hotplug2;			// ZZ: up threshold hotplug tuneable for core2
	unsigned int up_threshold_hotplug_freq2;		// Yank: up threshold hotplug freq tuneable for core2
	unsigned int block_up_multiplier_hotplug2;
	unsigned int up_threshold_hotplug3;			// ZZ: up threshold hotplug tuneable for core3
	unsigned int up_threshold_hotplug_freq3;		// Yank: up threshold hotplug freq tuneable for core3
	unsigned int block_up_multiplier_hotplug3;
#endif
#if (MAX_CORES == 8)
	unsigned int up_threshold_hotplug4;			// ZZ: up threshold hotplug tuneable for core4
	unsigned int up_threshold_hotplug_freq4;		// Yank: up threshold hotplug freq tuneable for core4
	unsigned int up_threshold_hotplug5;			// ZZ: up threshold hotplug tuneable for core5
	unsigned int up_threshold_hotplug_freq5;		// Yank: up threshold hotplug freq tuneable for core5
	unsigned int up_threshold_hotplug6;			// ZZ: up threshold hotplug tuneable for core6
	unsigned int up_threshold_hotplug_freq6;		// Yank: up threshold hotplug freq tuneable  for core6
	unsigned int up_threshold_hotplug7;			// ZZ: up threshold hotplug tuneable for core7
	unsigned int up_threshold_hotplug_freq7;		// Yank: up threshold hotplug freq tuneable for core7
#endif
#endif /* ENABLE_HOTPLUGGING */
	unsigned int up_threshold_sleep;			// ZZ: up threshold sleep tuneable for early suspend
	unsigned int down_threshold;				// ZZ: down threshold tuneable
#ifdef ENABLE_HOTPLUGGING
	unsigned int down_threshold_hotplug1;			// ZZ: down threshold hotplug tuneable for core1
	unsigned int down_threshold_hotplug_freq1;		// Yank: down threshold hotplug freq tuneable for core1
	unsigned int block_down_multiplier_hotplug1;
#if (MAX_CORES == 4 || MAX_CORES == 8)
	unsigned int down_threshold_hotplug2;			// ZZ: down threshold hotplug tuneable for core2
	unsigned int down_threshold_hotplug_freq2;		// Yank: down threshold hotplug freq tuneable for core2
	unsigned int block_down_multiplier_hotplug2;
	unsigned int down_threshold_hotplug3;			// ZZ: down threshold hotplug tuneable for core3
	unsigned int down_threshold_hotplug_freq3;		// Yank: down threshold hotplug freq tuneable for core3
	unsigned int block_down_multiplier_hotplug3;
#endif
#if (MAX_CORES == 8)
	unsigned int down_threshold_hotplug4;			// ZZ: down threshold hotplug tuneable for core4
	unsigned int down_threshold_hotplug_freq4;		// Yank: down threshold hotplug freq tuneable for core4
	unsigned int down_threshold_hotplug5;			// ZZ: down threshold hotplug tuneable for core5
	unsigned int down_threshold_hotplug_freq5;		// Yank: down threshold hotplug_freq tuneable for core5
	unsigned int down_threshold_hotplug6;			// ZZ: down threshold hotplug tuneable for core6
	unsigned int down_threshold_hotplug_freq6;		// Yank: down threshold hotplug freq tuneable for core6
	unsigned int down_threshold_hotplug7;			// ZZ: down threshold hotplug tuneable for core7
	unsigned int down_threshold_hotplug_freq7;		// Yank: down threshold hotplug freq tuneable for core7
#endif
#endif /* ENABLE_HOTPLUGGING */
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	unsigned int down_threshold_sleep;			// ZZ: down threshold sleep tuneable for early suspend
#endif
	unsigned int ignore_nice;				// ZZ: ignore nice load tuneable
	unsigned int smooth_up;					// ZZ: smooth up tuneable
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	unsigned int smooth_up_sleep;				// ZZ: smooth up sleep tuneable for early suspend
#ifdef ENABLE_HOTPLUGGING
	unsigned int hotplug_sleep;				// ZZ: hotplug sleep tuneable for early suspend
#endif /* ENABLE_HOTPLUGGING */
#endif
	unsigned int freq_limit;				// ZZ: freq limit tuneable
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	unsigned int freq_limit_sleep;				// ZZ: freq limit sleep tuneable for early suspend
#endif
	unsigned int fast_scaling_up;				// Yank: fast scaling tuneable for upscaling
	unsigned int fast_scaling_down;				// Yank: fast scaling tuneable for downscaling
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	unsigned int fast_scaling_sleep_up;			// Yank: fast scaling sleep tuneable for early suspend for upscaling
	unsigned int fast_scaling_sleep_down;			// Yank: fast scaling sleep tuneable for early suspend for downscaling
#endif
	unsigned int afs_threshold1;				// ZZ: auto fast scaling step one threshold
	unsigned int afs_threshold2;				// ZZ: auto fast scaling step two threshold
	unsigned int afs_threshold3;				// ZZ: auto fast scaling step three threshold
	unsigned int afs_threshold4;				// ZZ: auto fast scaling step four threshold
	unsigned int grad_up_threshold;				// ZZ: early demand grad up threshold tuneable
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	unsigned int grad_up_threshold_sleep;			// ZZ: early demand grad up threshold tuneable for early suspend
#endif
	unsigned int early_demand;				// ZZ: early demand master switch tuneable
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	unsigned int early_demand_sleep;			// ZZ: early demand master switch tuneable for early suspend
#endif
#ifdef ENABLE_HOTPLUGGING
	unsigned int disable_hotplug;				// ZZ: hotplug switch tuneable
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	unsigned int disable_hotplug_sleep;			// ZZ: hotplug switch for sleep tuneable for early suspend
#endif
	unsigned int hotplug_block_up_cycles;			// ZZ: hotplug up block cycles tuneable
	unsigned int hotplug_block_down_cycles;			// ZZ: hotplug down block cycles tuneable
	unsigned int hotplug_stagger_up;			// ff: hotplug stagger up tuneable
	unsigned int hotplug_stagger_down;			// ff: hotplug stagger down tuneable
	unsigned int hotplug_idle_threshold;			// ZZ: hotplug idle threshold tuneable
	unsigned int hotplug_idle_freq;				// ZZ: hotplug idle freq tuneable
	unsigned int hotplug_engage_freq;			// ZZ: frequency below which we run on only one core (ffolkes)
	unsigned int hotplug_max_limit;				// ff: the number of cores we allow to be online
	unsigned int hotplug_min_limit;				// ff: the number of cores we require to be online
	unsigned int hotplug_min_limit_saved;			// ff: the number of cores we require to be online
	unsigned int hotplug_min_limit_touchbooster;		// ff: the number of cores we require to be online
	unsigned int hotplug_lock;				// ff: the number of cores we allow to be online
#endif /* ENABLE_HOTPLUGGING */
	unsigned int scaling_block_threshold;			// ZZ: scaling block threshold tuneable
	unsigned int scaling_block_cycles;			// ZZ: scaling block cycles tuneable
	unsigned int scaling_up_block_cycles;			// ff: scaling-up block cycles tuneable
	unsigned int scaling_up_block_freq;			// ff: scaling-up block freq threshold tuneable
#ifdef CONFIG_EXYNOS4_EXPORT_TEMP
	unsigned int scaling_block_temp;			// ZZ: scaling block temp tuneable
#endif
#ifdef ENABLE_SNAP_THERMAL_SUPPORT
	unsigned int scaling_trip_temp;				// ff: snapdragon temperature tripping
#endif
	unsigned int scaling_block_freq;			// ZZ: scaling block freq tuneable
	unsigned int scaling_block_force_down;			// ZZ: scaling block force down tuneable
	unsigned int scaling_fastdown_freq;			// ZZ: frequency beyond which we apply a different up threshold (ffolkes)
	unsigned int scaling_fastdown_up_threshold;		// ZZ: up threshold when scaling fastdown freq exceeded (ffolkes)
	unsigned int scaling_fastdown_down_threshold;		// ZZ: down threshold when scaling fastdown freq exceeded (ffolkes)
	unsigned int scaling_responsiveness_freq;		// ZZ: frequency below which we use a lower up threshold (ffolkes)
	unsigned int scaling_responsiveness_up_threshold;	// ZZ: up threshold we use when below scaling responsiveness freq (ffolkes)
	unsigned int scaling_proportional;			// ZZ: proportional to load scaling
#ifdef ENABLE_INPUTBOOSTER
	// ff: input booster
	unsigned int inputboost_cycles;				// ff: default number of cycles to boost up/down thresholds
	unsigned int inputboost_up_threshold;			// ff: default up threshold for inputbooster
	unsigned int inputboost_punch_cycles;			// ff: default number of cycles to meet or exceed punch freq
	unsigned int inputboost_punch_freq;			// ff: default frequency to keep cur_freq at or above
	unsigned int inputboost_punch_on_fingerdown;
	unsigned int inputboost_punch_on_fingermove;
	unsigned int inputboost_punch_on_epenmove;
	unsigned int inputboost_typingbooster_up_threshold;
	unsigned int inputboost_typingbooster_cores;
#endif /* ENABLE_INPUTBOOSTER */

	// ff: Music Detection
	unsigned int music_max_freq;				// ff: music max freq
	unsigned int music_min_freq;				// ff: music min freq
#ifdef ENABLE_HOTPLUGGING
	unsigned int music_min_cores;				// ff: music min freq
#endif
	unsigned int music_state;				// ff: music state

// ZZ: set tuneable default values
} dbs_tuners_ins = {
	.profile = "none",
	.profile_number = DEF_PROFILE_NUMBER,
	.profile_sticky_mode = DEF_PROFILE_STICKY_MODE,
	.auto_adjust_freq_thresholds = DEF_AUTO_ADJUST_FREQ,
	.sampling_rate_idle = DEF_SAMPLING_RATE_IDLE,
	.sampling_rate_idle_threshold = DEF_SAMPLING_RATE_IDLE_THRESHOLD,
	.sampling_rate_idle_delay = DEF_SAMPLING_RATE_IDLE_DELAY,
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	.sampling_rate_sleep_multiplier = DEF_SAMPLING_RATE_SLEEP_MULTIPLIER,
#endif
	.sampling_down_factor = DEF_SAMPLING_DOWN_FACTOR,
	.sampling_down_momentum = DEF_SAMPLING_DOWN_MOMENTUM,
	.sampling_down_max_mom = DEF_SAMPLING_DOWN_MAX_MOMENTUM,
	.sampling_down_mom_sens = DEF_SAMPLING_DOWN_MOMENTUM_SENSITIVITY,
	.up_threshold = DEF_FREQUENCY_UP_THRESHOLD,
#ifdef ENABLE_HOTPLUGGING
	.up_threshold_hotplug1 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG,
	.up_threshold_hotplug_freq1 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG_FREQ,
	.block_up_multiplier_hotplug1 = DEF_BLOCK_UP_MULTIPLIER_HOTPLUG1,
#if (MAX_CORES == 4 || MAX_CORES == 8)
	.up_threshold_hotplug2 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG,
	.up_threshold_hotplug_freq2 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG_FREQ,
	.block_up_multiplier_hotplug2 = DEF_BLOCK_UP_MULTIPLIER_HOTPLUG2,
	.up_threshold_hotplug3 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG,
	.up_threshold_hotplug_freq3 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG_FREQ,
	.block_up_multiplier_hotplug3 = DEF_BLOCK_UP_MULTIPLIER_HOTPLUG3,
#endif
#if (MAX_CORES == 8)
	.up_threshold_hotplug4 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG,
	.up_threshold_hotplug_freq4 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG_FREQ,
	.up_threshold_hotplug5 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG,
	.up_threshold_hotplug_freq5 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG_FREQ,
	.up_threshold_hotplug6 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG,
	.up_threshold_hotplug_freq6 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG_FREQ,
	.up_threshold_hotplug7 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG,
	.up_threshold_hotplug_freq7 = DEF_FREQUENCY_UP_THRESHOLD_HOTPLUG_FREQ,
#endif
#endif /* ENABLE_HOTPLUGGING */
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	.up_threshold_sleep = DEF_UP_THRESHOLD_SLEEP,
#endif
	.down_threshold = DEF_FREQUENCY_DOWN_THRESHOLD,
#ifdef ENABLE_HOTPLUGGING
	.down_threshold_hotplug1 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG,
	.down_threshold_hotplug_freq1 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG_FREQ,
	.block_down_multiplier_hotplug1 = DEF_BLOCK_DOWN_MULTIPLIER_HOTPLUG1,
#if (MAX_CORES == 4 || MAX_CORES == 8)
	.down_threshold_hotplug2 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG,
	.down_threshold_hotplug_freq2 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG_FREQ,
	.block_down_multiplier_hotplug2 = DEF_BLOCK_DOWN_MULTIPLIER_HOTPLUG2,
	.down_threshold_hotplug3 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG,
	.down_threshold_hotplug_freq3 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG_FREQ,
	.block_down_multiplier_hotplug3 = DEF_BLOCK_DOWN_MULTIPLIER_HOTPLUG3,
#endif
#if (MAX_CORES == 8)
	.down_threshold_hotplug4 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG,
	.down_threshold_hotplug_freq4 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG_FREQ,
	.down_threshold_hotplug5 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG,
	.down_threshold_hotplug_freq5 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG_FREQ,
	.down_threshold_hotplug6 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG,
	.down_threshold_hotplug_freq6 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG_FREQ,
	.down_threshold_hotplug7 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG,
	.down_threshold_hotplug_freq7 = DEF_FREQUENCY_DOWN_THRESHOLD_HOTPLUG_FREQ,
#endif
#endif /* ENABLE_HOTPLUGGING */
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	.down_threshold_sleep = DEF_DOWN_THRESHOLD_SLEEP,
#endif
	.ignore_nice = DEF_IGNORE_NICE,
	.smooth_up = DEF_SMOOTH_UP,
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	.smooth_up_sleep = DEF_SMOOTH_UP_SLEEP,
#ifdef ENABLE_HOTPLUGGING
	.hotplug_sleep = DEF_HOTPLUG_SLEEP,
#endif /* ENABLE_HOTPLUGGING */
#endif
	.freq_limit = DEF_FREQ_LIMIT,
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	.freq_limit_sleep = DEF_FREQ_LIMIT_SLEEP,
#endif
	.fast_scaling_up = DEF_FAST_SCALING_UP,
	.fast_scaling_down = DEF_FAST_SCALING_DOWN,
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	.fast_scaling_sleep_up = DEF_FAST_SCALING_SLEEP_UP,
	.fast_scaling_sleep_down = DEF_FAST_SCALING_SLEEP_DOWN,
#endif
	.afs_threshold1 = DEF_AFS_THRESHOLD1,
	.afs_threshold2 = DEF_AFS_THRESHOLD2,
	.afs_threshold3 = DEF_AFS_THRESHOLD3,
	.afs_threshold4 = DEF_AFS_THRESHOLD4,
	.grad_up_threshold = DEF_GRAD_UP_THRESHOLD,
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	.grad_up_threshold_sleep = DEF_GRAD_UP_THRESHOLD_SLEEP,
#endif
	.early_demand = DEF_EARLY_DEMAND,
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	.early_demand_sleep = DEF_EARLY_DEMAND_SLEEP,
#endif
#ifdef ENABLE_HOTPLUGGING
	.disable_hotplug = DEF_DISABLE_HOTPLUG,
#if (defined(CONFIG_HAS_EARLYSUSPEND) || defined(CONFIG_POWERSUSPEND) && !defined(DISABLE_POWER_MANAGEMENT)) || defined(USE_LCD_NOTIFIER)
	.disable_hotplug_sleep = DEF_DISABLE_HOTPLUG_SLEEP,
#endif
	.hotplug_block_up_cycles = DEF_HOTPLUG_BLOCK_UP_CYCLES,
	.hotplug_block_down_cycles = DEF_HOTPLUG_BLOCK_DOWN_CYCLES,
	.hotplug_stagger_up = DEF_HOTPLUG_STAGGER_UP,
	.hotplug_stagger_down = DEF_HOTPLUG_STAGGER_DOWN,
	.hotplug_idle_threshold = DEF_HOTPLUG_IDLE_THRESHOLD,
	.hotplug_idle_freq = DEF_HOTPLUG_IDLE_FREQ,
	.hotplug_engage_freq = DEF_HOTPLUG_ENGAGE_FREQ,
	.hotplug_max_limit = DEF_HOTPLUG_MAX_LIMIT,
	.hotplug_min_limit = DEF_HOTPLUG_MIN_LIMIT,
	.hotplug_min_limit_saved = DEF_HOTPLUG_MIN_LIMIT,
	.hotplug_min_limit_touchbooster = 0,
	.hotplug_lock = DEF_HOTPLUG_LOCK,
#endif /* ENABLE_HOTPLUGGING */
	.scaling_block_threshold = DEF_SCALING_BLOCK_THRESHOLD,
	.scaling_block_cycles = DEF_SCALING_BLOCK_CYCLES,
	.scaling_up_block_cycles = DEF_SCALING_UP_BLOCK_CYCLES,
	.scaling_up_block_freq = DEF_SCALING_UP_BLOCK_FREQ,
#ifdef CONFIG_EXYNOS4_EXPORT_TEMP
	.scaling_block_temp = DEF_SCALING_BLOCK_TEMP,
#endif
#ifdef ENABLE_SNAP_THERMAL_SUPPORT
	.scaling_trip_temp = DEF_SCALING_TRIP_TEMP,
#endif
	.scaling_block_freq = DEF_SCALING_BLOCK_FREQ,
	.scaling_block_force_down = DEF_SCALING_BLOCK_FORCE_DOWN,
	.scaling_fastdown_freq = DEF_SCALING_FASTDOWN_FREQ,
	.scaling_fastdown_up_threshold = DEF_SCALING_FASTDOWN_UP_THRESHOLD,
	.scaling_fastdown_down_threshold = DEF_SCALING_FASTDOWN_DOWN_THRESHOLD,
	.scaling_responsiveness_freq = DEF_SCALING_RESPONSIVENESS_FREQ,
	.scaling_responsiveness_up_threshold = DEF_SCALING_RESPONSIVENESS_UP_THRESHOLD,
	.scaling_proportional = DEF_SCALING_PROPORTIONAL,
#ifdef ENABLE_INPUTBOOSTER
	// ff: Input Booster
	.inputboost_cycles = DEF_INPUTBOOST_CYCLES,
	.inputboost_up_threshold = DEF_INPUTBOOST_UP_THRESHOLD,
	.inputboost_punch_cycles = DEF_INPUTBOOST_PUNCH_CYCLES,
	.inputboost_punch_freq = DEF_INPUTBOOST_PUNCH_FREQ,
	.inputboost_punch_on_fingerdown = DEF_INPUTBOOST_PUNCH_ON_FINGERDOWN,
	.inputboost_punch_on_fingermove = DEF_INPUTBOOST_PUNCH_ON_FINGERMOVE,
	.inputboost_punch_on_epenmove = DEF_INPUTBOOST_PUNCH_ON_EPENMOVE,
	.inputboost_typingbooster_up_threshold = DEF_INPUTBOOST_TYPINGBOOSTER_UP_THRESHOLD,
	.inputboost_typingbooster_cores = DEF_INPUTBOOST_TYPINGBOOSTER_CORES,
#endif /* ENABLE_INPUTBOOSTER */
	.music_max_freq = DEF_MUSIC_MAX_FREQ,
	.music_min_freq = DEF_MUSIC_MIN_FREQ,
#ifdef ENABLE_HOTPLUGGING
	.music_min_cores = DEF_MUSIC_MIN_CORES,
#endif /* ENABLE_HOTPLUGGING */
	.music_state = 0,
};

#ifdef ENABLE_INPUTBOOSTER
// ff: Input Booster
static void interactive_input_event(struct input_handle *handle, unsigned int type, unsigned int code, int value)
{
	unsigned int time_since_typingbooster_lasttapped = 0;
	unsigned int flg_do_punch_id = 0;
	struct timeval time_now;
	bool flg_inputboost_mt_touchmajor = false;
	bool flg_inputboost_abs_xy = false;
	bool flg_force_punch = false;
	int inputboost_btn_toolfinger = -1;
	int inputboost_btn_touch = -1;
	int inputboost_mt_trackingid = 0;
	int tmp_flg_ctr_inputboost_punch = 0;

	// ff: don't do any boosting when overheating
#ifdef ENABLE_SNAP_THERMAL_SUPPORT
	if (tmu_throttle_steps > 0)
		return;
#endif
	/*
	 * ff: ignore events if inputboost isn't enabled (we shouldn't ever be here in that case)
	 *     or screen-off (but allow power press)
	 */
	if (!dbs_tuners_ins.inputboost_cycles
		|| (suspend_flag && inputboost_last_code != 116))
		return;

	if (type == EV_SYN && code == SYN_REPORT) {
#ifdef ZZMOOVE_DEBUG
		pr_info("[zzmoove] syn input event. device: %s, saw_touchmajor: %d, saw_xy: %d, toolfinger: %d, touch: %d, trackingid: %d\n",
				handle->dev->name, flg_inputboost_report_mt_touchmajor, flg_inputboost_report_abs_xy, inputboost_report_btn_toolfinger, inputboost_report_btn_touch, inputboost_report_mt_trackingid);
#endif
		if (strstr(handle->dev->name, "touchscreen") || strstr(handle->dev->name, "synaptics")) {

			// ff: don't boost if not enabled, or while sleeping
			if (!boost_on_tsp || suspend_flag)
				return;

			// ff: save the event's data flags
			flg_inputboost_mt_touchmajor = flg_inputboost_report_mt_touchmajor;
			flg_inputboost_abs_xy = flg_inputboost_report_abs_xy;
			inputboost_btn_toolfinger = inputboost_report_btn_toolfinger;
			inputboost_btn_touch = inputboost_report_btn_touch;
			inputboost_mt_trackingid = inputboost_report_mt_trackingid;

			// ff: reset the event's data flags
			flg_inputboost_report_mt_touchmajor = false;
			flg_inputboost_report_abs_xy = false;
			inputboost_report_btn_toolfinger = -1;
			inputboost_report_btn_touch = -1;
			inputboost_report_mt_trackingid = 0;
#ifdef ZZMOOVE_DEBUG
			pr_info("[zzmoove] syn input event. device: %s, saw_touchmajor: %d, saw_xy: %d, toolfinger: %d, touch: %d, trackingid: %d\n",
					handle->dev->name, flg_inputboost_mt_touchmajor, flg_inputboost_abs_xy, inputboost_btn_toolfinger, inputboost_btn_touch, inputboost_mt_trackingid);
#endif
			// ff: try to determine what kind of event we just saw
			if (!flg_inputboost_mt_touchmajor
				&& (flg_inputboost_abs_xy || inputboost_mt_trackingid < 0)
				&& inputboost_btn_touch < 0) {
				/*
				 * ff: assume hovering since:
				 *     no width was reported, and btn_touch isn't set, and (xy coords were included or trackingid is -1 meaning hover-up)
				 */

				 // ff: don't boost if not enabled
				if (!boost_on_tsp_hover)
					return;
#ifdef ZZMOOVE_DEBUG
				// ff: hover is hardcoded to only punch on first hover, otherwise it'd be punching constantly
				if (inputboost_mt_trackingid > 0) {

					pr_info("[zzmoove] touch - first hover btn_touch: %d\n", inputboost_btn_touch);
					/*// ff: unused, but kept for future use
					flg_do_punch_id = 4;
					flg_force_punch = false;*/
				} else if (inputboost_mt_trackingid < 0) {

					pr_info("[zzmoove] touch - end hover btn_touch: %d\n", inputboost_btn_touch);
					/*// ff: don't boost if not enabled, or while sleeping
					flg_ctr_inputboost_punch = 0;
					flg_ctr_inputboost = 0;*/
				} else {
					pr_info("[zzmoove] touch - update hover btn_touch: %d\n", inputboost_btn_touch);
				}
#endif
			} else if (inputboost_mt_trackingid > 0) {
				// ff: new finger detected event
#ifdef ZZMOOVE_DEBUG
				pr_info("[zzmoove] touch - first touch\n");
#endif
				flg_do_punch_id = 2;

				// ff: should we boost on every finger down event?
				if (dbs_tuners_ins.inputboost_punch_on_fingerdown)
					flg_force_punch = true;

				// ff: typing booster. detects rapid taps, and if found, boosts core count and up_threshold
				if (dbs_tuners_ins.inputboost_typingbooster_up_threshold) {

					// ff: save current time
					do_gettimeofday(&time_now);

					// ff: get time difference
					time_since_typingbooster_lasttapped = (time_now.tv_sec - time_typingbooster_lasttapped.tv_sec) * MSEC_PER_SEC +
										(time_now.tv_usec - time_typingbooster_lasttapped.tv_usec) / USEC_PER_MSEC;

					// ff: was that typing or just a doubletap?
					if (time_since_typingbooster_lasttapped < 250) {

						// ff: tap is probably typing
						ctr_inputboost_typingbooster_taps++;
#ifdef ZZMOOVE_DEBUG
						pr_info("[zzmoove] inputboost - typing booster - valid tap: %d\n", ctr_inputboost_typingbooster_taps);
#endif
					} else {
						// ff: tap too quick, probably a doubletap, ignore
						ctr_inputboost_typingbooster_taps = 0;
#ifdef ZZMOOVE_DEBUG
						pr_info("[zzmoove] inputboost - typing booster - invalid tap: %d (age: %d)\n", ctr_inputboost_typingbooster_taps, time_since_typingbooster_lasttapped);
#endif
					}

					if ((flg_ctr_inputbooster_typingbooster < 1 && ctr_inputboost_typingbooster_taps > 1)			// ff: if booster wasn't on, require 3 taps
						|| (flg_ctr_inputbooster_typingbooster > 0 && ctr_inputboost_typingbooster_taps > 0)) {		// ff: otherwise, refill with only 2
#ifdef ZZMOOVE_DEBUG
						// ff: probably typing, so start the typing booster
						if (flg_ctr_inputbooster_typingbooster < 1)
						    pr_info("[zzmoove] inputboost - typing booster on!\n");
#endif
						// ff: set typing booster up_threshold counter
						flg_ctr_inputbooster_typingbooster = 15;

						// ff: request a punch
						flg_do_punch_id = 12;

						/*
						 * ff: forcing this will effectively turn this into a touchbooster,
						 *     as it will keep applying the punch freq until the typing (taps) stops
						 */
						flg_force_punch = true;

#ifdef ZZMOOVE_DEBUG
					} else {
						pr_info("[zzmoove] inputboost - typing booster - tapctr: %d, flgctr: %d\n", ctr_inputboost_typingbooster_taps, flg_ctr_inputbooster_typingbooster);
#endif
					}
					// ff: and finally, set the time so we can compare to it on the next tap
					do_gettimeofday(&time_typingbooster_lasttapped);
				}

#ifdef ZZMOOVE_DEBUG
			} else if (inputboost_mt_trackingid < 0) {
				// ff: finger-lifted event. do nothing
				pr_info("[zzmoove] touch - end touch\n");
#endif
			} else if (flg_inputboost_mt_touchmajor) {
				// ff: width was reported, assume regular tap
#ifdef ZZMOOVE_DEBUG
				pr_info("[zzmoove] touch - update touch\n");
#endif
				// ff: should we treat this like a touchbooster and always punch on movement?
				if (dbs_tuners_ins.inputboost_punch_on_fingermove) {
					flg_do_punch_id = 3;
					flg_force_punch = true;
				}

			} else {
				// ff: unknown event. do nothing
#ifdef ZZMOOVE_DEBUG
				pr_info("[zzmoove] touch - unknown\n");
#endif
				return;
			}

		} else if (strstr(handle->dev->name, "gpio")) {

			// ff: don't boost if not enabled, or while sleeping
			if (!boost_on_gpio || suspend_flag)
				return;

			// ff: check for home button
			if (inputboost_last_code == 172) {
				if (suspend_flag) {
					// ff: home press while suspended shouldn't boost as hard
					flg_ctr_cpuboost = 2;
#ifdef ZZMOOVE_DEBUG
					pr_info("[zzmoove] inputboost - gpio punched freq immediately!\n");
#endif
				} else {
					// ff: home press while screen on should boost
					flg_ctr_cpuboost = 10;
#ifdef ENABLE_WORK_RESTARTLOOP
					queue_work_on(0, dbs_aux_wq, &work_restartloop);
#ifdef ZZMOOVE_DEBUG
					pr_info("[zzmoove] inputboost - gpio punched freq immediately!\n");
#endif
					// ff: don't punch, since we just did manually
#endif
				}
			} else {
				/*
				 * ff: other press (aka vol up on note4)
				 *     treat it as a normal button press
				 */
				flg_do_punch_id = 7;
				flg_force_punch = true;
			}

		} else if (strstr(handle->dev->name, "touchkey")) {

			// ff: don't boost if not enabled, or while sleeping
			if (!boost_on_tk || suspend_flag)
				return;

			// ff: check for recents button
			if (inputboost_last_code == 254) {
				/*
				 * ff: recents press. do more than punch,
				 *     and set the max-boost and max-core counters, too
				 */
				flg_ctr_cpuboost = 20;
			} else {
				// ff: anything else (ie. back press)
				flg_ctr_cpuboost = 3;
			}

			// ff: always manually punch for touchkeys
#ifdef ENABLE_WORK_RESTARTLOOP
#ifdef ZZMOOVE_DEBUG
			pr_info("[zzmoove] inputboost - tk punched freq immediately!\n");
#endif
			queue_work_on(0, dbs_aux_wq, &work_restartloop);
#endif
			// ff: don't punch, since we just did manually

		} else if (strstr(handle->dev->name, "e-pen")) {
			// ff: s-pen is hovering or writing

			// ff: don't boost if not enabled, or while sleeping
			if (!boost_on_epen || suspend_flag)
				return;

			// ff: request a punch
			flg_do_punch_id = 11;

			// ff: should we treat this like a touchbooster and always punch on movement?
			if (dbs_tuners_ins.inputboost_punch_on_epenmove)
				flg_force_punch = true;

		} else if (strstr(handle->dev->name, "qpnp_pon")) {
			// ff: on the note4/opo, this is the power key and volume down

			/*
			 * ff: only boost if power is press while screen-off
			 *     let it still apply a normal boost on screen-on to speed up going into suspend
			 */
			if (inputboost_last_code == 116 && suspend_flag) {
				// disabled since we're still boosting in the pon driver
				/*flg_ctr_cpuboost = 25;
				flg_ctr_inputboost = 100;
				queue_work_on(0, dbs_aux_wq, &work_restartloop);
				pr_info("[zzmoove] inputboost - gpio/powerkey punched freq immediately and skipped the rest\n");*/

				// ff: not only don't punch, but don't even start the booster, since we just did both with zzmoove_boost() externally
				return;
			} else {
				// ff: even though it's coming from a different device, treat this if it was a gpio event anyway
				flg_do_punch_id = 7;
				flg_force_punch = true;
			}
		}

		if (flg_do_punch_id						// ff: punch is requested
			&& dbs_tuners_ins.inputboost_punch_cycles		// ff: punch is enabled
			&& dbs_tuners_ins.inputboost_punch_freq			// ff: punch is enabled
			&& (flg_ctr_inputboost < 1 || flg_force_punch)) {	// ff: this is the first event since the inputbooster ran out, or it is forced
										//     a punch length and frequency is set, so boost!
			// ff: but only do so if it hasn't been punched yet, or if it hasn't been punched by a touch yet
			// ff: save the punch counter state so we can avoid redundantly flushing the punch
			tmp_flg_ctr_inputboost_punch = flg_ctr_inputboost_punch;

			// ff: refill the punch counter. remember, flg_ctr_inputboost_punch is decremented before it is used, so add 1
			flg_ctr_inputboost_punch = dbs_tuners_ins.inputboost_punch_cycles + 1;

			// ff: don't immediately apply the punch if we're already boosted or punched
#ifdef ENABLE_WORK_RESTARTLOOP
			if (flg_ctr_cpuboost < 5 && tmp_flg_ctr_inputboost_punch < 1) {
				queue_work_on(0, dbs_aux_wq, &work_restartloop);
#endif
#ifdef ZZMOOVE_DEBUG
				do_gettimeofday(&time_touchbooster_lastrun);
				pr_info("[zzmoove] inputboost - punched freq immediately for: %d\n", flg_do_punch_id);
#endif
#ifdef ENABLE_WORK_RESTARTLOOP
			}
#endif
#ifdef ZZMOOVE_DEBUG
			pr_info("[zzmoove] inputboost - punch set to %d mhz for %d cycles (punched by: %d, forced: %d)\n",
					dbs_tuners_ins.inputboost_punch_freq, dbs_tuners_ins.inputboost_punch_cycles, flg_do_punch_id, flg_force_punch);
#endif
		}

		// ff: refill the inputboost counter to apply the up_threshold
		flg_ctr_inputboost = dbs_tuners_ins.inputboost_cycles;

	} else {
#ifdef ZZMOOVE_DEBUG
		pr_info("[zzmoove] ev input event. name: %s, type: %d, code: %d, value: %d\n", handle->dev->name, type, code, value);
#endif
		// ff: we need to keep track of the data sent in this report
		if (strstr(handle->dev->name, "touchscreen") || strstr(handle->dev->name, "synaptics")) {
			if (code == BTN_TOOL_FINGER) {
				// ff: 0 = nothing at all, 1 - touch OR hover starting
				inputboost_report_btn_toolfinger = value;

			} else if (code == BTN_TOUCH) {
				// ff: 0 = up/hovering, 1 - touch starting
				inputboost_report_btn_touch = value;

			} else if (code == ABS_MT_TRACKING_ID) {
				// ff: -1 = finger-up, >1 = finger-down
				inputboost_report_mt_trackingid = value;
#ifdef ABS_MT_SUMSIZE
			} else if (code == ABS_MT_TOUCH_MAJOR || code == ABS_MT_SUMSIZE) {
#else
			} else if (code == ABS_MT_TOUCH_MAJOR) {
#endif
				// ff: this is a touch report
				flg_inputboost_report_mt_touchmajor = true;

			} else if (code == ABS_MT_POSITION_X || code == ABS_MT_POSITION_Y) {
				// ff: this is a hover report, maybe
				flg_inputboost_report_abs_xy = true;
			}
		} else {
			// ff: a simple saving of the last event is sufficent for non-tsp events
			inputboost_last_type = type;
			inputboost_last_code = code;
			inputboost_last_value = value;
		}
	}
}

static int input_dev_filter(const char *input_dev_name)
{
	if (strstr(input_dev_name, "sec_touchscreen") ||
		strstr(input_dev_name, "sec_e-pen") ||
		strstr(input_dev_name, "gpio-keys") ||
		strstr(input_dev_name, "sec_touchkey") ||
		strstr(input_dev_name, "s2s_pwrkey") ||						// ZZ: opo power key?
		strstr(input_dev_name, "msm8974-taiko-mtp-snd-card Button Jack") ||		// ZZ: opo sound button?
		strstr(input_dev_name, "msm8974-taiko-mtp-snd-card Headset Jack") ||		// ZZ: opo headset jack
		strstr(input_dev_name, "synaptics-rmi-ts") ||					// ZZ: opo touchscreen
		strstr(input_dev_name, "qpnp_pon")						// ff: note4/opo power and volume-down key
		//strstr(input_dev_name, "es705")						// ff: note4 always-on audio monitoring, but no input events are sent, so it's pointless
		) {
#ifdef ZZMOOVE_DEBUG
		pr_info("[zzmoove] inputboost - monitoring input device: %s\n", input_dev_name);
#endif
		return 0;

	} else {
#ifdef ZZMOOVE_DEBUG
		pr_info("[zzmoove] inputboost - ignored input device: %s\n", input_dev_name);
#endif
		return 1;
	}
}

static int interactive_input_connect(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id)
{
	struct input_handle *handle;
	int error;

	if (input_dev_filter(dev->name))
		return -ENODEV;

	handle = kzalloc(sizeof(struct input_handle), GFP_KERNEL);
