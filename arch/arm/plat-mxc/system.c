/*
 * Copyright (C) 1999 ARM Limited
 * Copyright (C) 2000 Deep Blue Solutions Ltd
 * Copyright 2006-2012 Freescale Semiconductor, Inc.
 * Copyright 2008 Juergen Beisert, kernel@pengutronix.de
 * Copyright 2009 Ilya Yanok, Emcraft Systems Ltd, yanok@emcraft.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <mach/hardware.h>
#include <mach/common.h>
#include <mach/system.h>
#include <asm/proc-fns.h>
#include <asm/system.h>
#ifdef CONFIG_SMP
#include <linux/smp.h>
#endif
#include <asm/mach-types.h>

static void __iomem *wdog_base;
extern u32 enable_ldo_mode;


static void arch_reset_special_mode(char mode, const char *cmd)
{
	if (strcmp(cmd, "download") == 0)
		do_switch_mfgmode();
	else if (strcmp(cmd, "recovery") == 0)
		do_switch_recovery();
	else if (strcmp(cmd, "fastboot") == 0)
		do_switch_fastboot();
}

/*
 * Reset the system. It is called by machine_restart().
 */
void arch_reset(char mode, const char *cmd)
{
	unsigned int wcr_enable;

	arch_reset_special_mode(mode, cmd);

#ifdef CONFIG_ARCH_MX6
	/* To change the bootcfg by software for curie board:
	   1. load required bootcfg to SRC_GPR9 (0x020d8040)
	   2. set bit 28 of SRC_GPR10 (0x020d8044)
	   3. then reset the system

	   to return to normal boot mode, clear SRC_GPR10[28]
	 */
	// eMMC 1-bit mode
	{
	u32 bmsr1 = __raw_readl(IO_ADDRESS(SRC_BASE_ADDR + 0x4));
	u32 gpr9 = __raw_readl(IO_ADDRESS(SRC_BASE_ADDR + SRC_GPR9));
	u32 gpr10 = __raw_readl(IO_ADDRESS(SRC_BASE_ADDR + SRC_GPR10));

	if(bmsr1 == 0x4000d860) {
		// original mode is eMMC 8-bit DDR boot
		__raw_writel(0x40001860, IO_ADDRESS(SRC_BASE_ADDR + SRC_GPR9));
		__raw_writel(gpr10 | 0x10000000, IO_ADDRESS(SRC_BASE_ADDR + SRC_GPR10));
	} else {
		// original mode is SD boot, unchanged
	}
	};

	/* wait for reset to assert... */
	if (enable_ldo_mode == LDO_MODE_BYPASSED) {
		/*On Sabresd board use WDOG2 to reset external PMIC, so here do
		* more WDOG2 reset.*/
		wcr_enable = 0x14;
		__raw_writew(wcr_enable, IO_ADDRESS(MX6Q_WDOG2_BASE_ADDR));
		__raw_writew(wcr_enable, IO_ADDRESS(MX6Q_WDOG2_BASE_ADDR));
	} else
		wcr_enable = (1 << 2);
	__raw_writew(wcr_enable, wdog_base);
	/* errata TKT039676, SRS bit may be missed when
	SRC sample it, need to write the wdog controller
	twice to avoid it */
	__raw_writew(wcr_enable, wdog_base);

	/* wait for reset to assert... */
	mdelay(500);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	return;
#endif

#ifdef CONFIG_MACH_MX51_EFIKAMX
	if (machine_is_mx51_efikamx()) {
		mx51_efikamx_reset();
		return;
	}
#endif

	if (cpu_is_mx1()) {
		wcr_enable = (1 << 0);
	} else {
		struct clk *clk;

		clk = clk_get_sys("imx2-wdt.0", NULL);
		if (!IS_ERR(clk))
			clk_enable(clk);
		wcr_enable = (1 << 2);
	}

	/* Assert SRS signal */
	__raw_writew(wcr_enable, wdog_base);

	/* wait for reset to assert... */
	mdelay(500);

	printk(KERN_ERR "Watchdog reset failed to assert reset\n");

	/* delay to allow the serial port to show the message */
	mdelay(50);

	/* we'll take a jump through zero as a poor second */
	cpu_reset(0);
}

void mxc_arch_reset_init(void __iomem *base)
{
	wdog_base = base;
}
