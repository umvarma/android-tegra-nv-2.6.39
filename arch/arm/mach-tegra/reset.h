/*
 * arch/arm/mach-tegra/reset.h
 *
 * CPU reset dispatcher.
 *
 * Copyright (c) 2011, NVIDIA Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __MACH_TEGRA_RESET_H
#define __MACH_TEGRA_RESET_H

#define TEGRA_RESET_MASK_PRESENT	0
#define TEGRA_RESET_MASK_LP1		1
#define TEGRA_RESET_MASK_LP2		2
#define TEGRA_RESET_STARTUP_SECONDARY	3
#define TEGRA_RESET_STARTUP_LP2		4
#define TEGRA_RESET_STARTUP_LP1		5
#define TEGRA_RESET_DATA_SIZE		6

#ifndef __ASSEMBLY__

#include <linux/cpumask.h>

extern unsigned long __tegra_cpu_reset_handler_data[TEGRA_RESET_DATA_SIZE];

void __tegra_cpu_reset_handler_start(void);
void tegra_secondary_startup(void);

#ifdef CONFIG_PM_SLEEP
#define tegra_cpu_lp1_mask ((unsigned long *)(IO_ADDRESS(TEGRA_RESET_HANDLER_BASE + \
		((u32)&__tegra_cpu_reset_handler_data[TEGRA_RESET_MASK_LP1] - \
		 (u32)__tegra_cpu_reset_handler_start))))

#define tegra_cpu_reset_handler_ptr ((u32 *)(IO_ADDRESS(TEGRA_RESET_HANDLER_BASE + \
		((u32)__tegra_cpu_reset_handler_data - \
		 (u32)__tegra_cpu_reset_handler_start))))

#define tegra_cpu_lp2_mask ((cpumask_t *)(IO_ADDRESS(TEGRA_RESET_HANDLER_BASE + \
		((u32)&__tegra_cpu_reset_handler_data[TEGRA_RESET_MASK_LP2] - \
		 (u32)__tegra_cpu_reset_handler_start))))
#endif

void tegra_cpu_reset_handler_enable(void);
void __init tegra_cpu_reset_handler_init(void);

#ifdef CONFIG_PM_SLEEP
void tegra_cpu_reset_handler_save(void);
void tegra_cpu_reset_handler_restore(void);
#endif
#endif
#endif
