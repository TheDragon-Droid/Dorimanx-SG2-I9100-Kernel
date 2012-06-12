/*
 * Copyright (C) STMicroelectronics 2009
 * Copyright (C) ST-Ericsson SA 2010
 *
 * License Terms: GNU General Public License v2
 *	Based on ARM realview platform
 *
 * Author: Sundar Iyer <sundar.iyer@stericsson.com>
 *
 */
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/smp.h>

#include <asm/smp_plat.h>

<<<<<<< HEAD
#include "setup.h"
=======
#include <mach/setup.h>
>>>>>>> 28e8e29... ARM: consolidate pen_release instead of having per platform definitions

/*
 * platform-specific code to shutdown a CPU
 *
 * Called with IRQs disabled
 */
void __ref ux500_cpu_die(unsigned int cpu)
{
	/* directly enter low power state, skipping secure registers */
	for (;;) {
		__asm__ __volatile__("dsb\n\t" "wfi\n\t"
				: : : "memory");
		if (pen_release == cpu_logical_map(cpu)) {
			/*
			 * OK, proper wakeup, we're done
			 */
			break;
		}
	}
}
