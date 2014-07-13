/*
 * Copyright (C) 2014 Shanghai Zhixing Information Technology Co.Ltd.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/types.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/nodemask.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/fsl_devices.h>
#include <linux/spi/spi.h>
#include <linux/spi/flash.h>
#include <linux/i2c.h>
#include <linux/ata.h>
#include <linux/mtd/mtd.h>
#include <linux/mtd/map.h>
#include <linux/mtd/partitions.h>
#include <linux/regulator/consumer.h>
#include <linux/pmic_external.h>
#include <linux/pmic_status.h>
#include <linux/ipu.h>
#include <linux/mxcfb.h>
#include <linux/pwm_backlight.h>
#include <linux/fec.h>
#include <linux/memblock.h>
#include <linux/gpio.h>
#include <linux/etherdevice.h>
#include <linux/power/sabresd_battery.h>
#include <linux/regulator/anatop-regulator.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/mfd/mxc-hdmi-core.h>

#include <mach/common.h>
#include <mach/hardware.h>
#include <mach/mxc_dvfs.h>
#include <mach/memory.h>
#include <mach/iomux-mx6q.h>
#include <mach/imx-uart.h>
#include <mach/viv_gpu.h>
#include <mach/ahci_sata.h>
#include <mach/ipu-v3.h>
#include <mach/mxc_hdmi.h>
#include <mach/mxc_asrc.h>
#include <mach/mipi_dsi.h>

#include <asm/irq.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>

#include "usb.h"
#include "devices-imx6q.h"
#include "crm_regs.h"
#include "cpu_op-mx6.h"
#include "board-mx6q_curie.h"

/* CPU Regulator Global */
extern char *gp_reg_id;
extern char *soc_reg_id;
extern char *pu_reg_id;

/* Debug Uart */
static inline void mx6q_curie_init_uart(void)
{
	imx6q_add_imx_uart(0, NULL);
}

/* USB Host & OTG */
static void __init mx6q_curie_init_usb(void)
{
	imx_otg_base = MX6_IO_ADDRESS(MX6Q_USB_OTG_BASE_ADDR);

	/* For MX6Q:
	 * GPR1 bit13 meaning:
	 * Bit13:       0 - selects ENET_RX_ER as the pin of USB_OTG_ID
	 *              1 - selects GPIO_1 as the pin of USB_OTG_ID
	 *
	 * Curie selects ENET_RX_ER
	 */
	mxc_iomux_set_gpr_register(1, 13, 1, 0);

	/* USB OTG power is always on */
	/* USB Host power is managed by the on-board USB hub */
	/* So we don't need to setup the vbus callback */
	/* vbus callback prototype: static void (bool) */
	//mx6_set_otghost_vbus_func(imx6q_curie_usbotg_vbus);
	//mx6_set_host1_vbus_func(imx6q_curie_host1_vbus);
}

/* Ethernet FEC */
#define CURIE_RGMII_RST        IMX_GPIO_NR(1, 25)

static int mx6q_curie_fec_phy_init(struct phy_device *phydev)
{
	// RTL8211E: disable Green Ethernet
	phy_write(phydev, 31, 0x0003);
	phy_write(phydev, 25, 0x3246);
	phy_write(phydev, 16, 0xa87c);
	phy_write(phydev, 31, 0x0000);
	return 0;
}

static struct fec_platform_data fec_data __initdata = {
	.init = mx6q_curie_fec_phy_init,
	.phy = PHY_INTERFACE_MODE_RGMII,
	.gpio_irq = -1,
};

/* I2C */
static struct imxi2c_platform_data mx6q_curie_i2c_data = {
	.bitrate = 100000,
};

static struct i2c_board_info mx6q_curie_i2c1_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("mxc_hdmi_i2c", 0x50),
	},
};

/* DVFS */
static struct mxc_dvfs_platform_data mx6q_curie_dvfscore_data = {
	.reg_id = "VDDCORE",
	.soc_id	= "VDDSOC",
	.clk1_id = "cpu_clk",
	.clk2_id = "gpc_dvfs_clk",
	.gpc_cntr_offset = MXC_GPC_CNTR_OFFSET,
	.ccm_cdcr_offset = MXC_CCM_CDCR_OFFSET,
	.ccm_cacrr_offset = MXC_CCM_CACRR_OFFSET,
	.ccm_cdhipr_offset = MXC_CCM_CDHIPR_OFFSET,
	.prediv_mask = 0x1F800,
	.prediv_offset = 11,
	.prediv_val = 3,
	.div3ck_mask = 0xE0000000,
	.div3ck_offset = 29,
	.div3ck_val = 2,
	.emac_val = 0x08,
	.upthr_val = 25,
	.dnthr_val = 9,
	.pncthr_val = 33,
	.upcnt_val = 10,
	.dncnt_val = 10,
	.delay_time = 80,
};

/* PMIC */
#define CURIE_PMIC_INT        IMX_GPIO_NR(7, 13)
extern int mx6q_curie_init_pfuze100(u32 int_gpio);
static void __init mx6q_curie_init_pmic(void)
{
	int ret = gpio_request(CURIE_PMIC_INT, "pFUZE-int");
	if (ret) {
		printk(KERN_ERR"request pFUZE-int error!!\n");
		return;
	}
	gpio_direction_input(CURIE_PMIC_INT);
	mx6q_curie_init_pfuze100(CURIE_PMIC_INT);
}

/* PM */
static void mx6q_curie_suspend_enter(void)
{
	/* suspend preparation */
}

static void mx6q_curie_suspend_exit(void)
{
	/* resume restore */
}
static const struct pm_platform_data mx6q_curie_pm_data __initconst = {
	.name = "imx_pm",
	.suspend_enter = mx6q_curie_suspend_enter,
	.suspend_exit = mx6q_curie_suspend_exit,
};
static void mx6q_curie_snvs_poweroff(void)
{
	/* do nothing */
}

/* Board Functions */
static void __init fixup_mxc_board(struct machine_desc *desc, struct tag *tags,
				   char **cmdline, struct meminfo *mi)
{
}

/*!
 * Board specific initialization.
 */
static void __init mx6_curie_board_init(void)
{
	if (cpu_is_mx6q()) {
		mxc_iomux_v3_setup_multiple_pads(mx6q_curie_pads,
			ARRAY_SIZE(mx6q_curie_pads));
	}

	/* Debug UART */
	mx6q_curie_init_uart();
	/* USB Host & OTG */
	mx6q_curie_init_usb();
	/* Ethernet: FEC */
	gpio_request(CURIE_RGMII_RST, "rgmii_reset");
	gpio_direction_output(CURIE_RGMII_RST, 0);
	mdelay(15);
	gpio_direction_output(CURIE_RGMII_RST, 1);
	mdelay(200);
	imx6_init_fec(fec_data);
	/* RTC */
	imx6q_add_imx_snvs_rtc();
	/* I2C */
	imx6q_add_imx_i2c(1, &mx6q_curie_i2c_data);
	imx6q_add_imx_i2c(2, &mx6q_curie_i2c_data);
	i2c_register_board_info(1, mx6q_curie_i2c1_board_info,
			ARRAY_SIZE(mx6q_curie_i2c1_board_info));
	/* CPU Regulator Global */
	gp_reg_id = mx6q_curie_dvfscore_data.reg_id;
	soc_reg_id = mx6q_curie_dvfscore_data.soc_id;
	/* PMIC */
	mx6q_curie_init_pmic();
	/* DVFS */
	imx6q_add_dvfs_core(&mx6q_curie_dvfscore_data);
	/* Bus Freq */
	imx6q_add_busfreq();
	/* PM */
	imx6q_add_pm_imx(0, &mx6q_curie_pm_data);
	pm_power_off = mx6q_curie_snvs_poweroff;
}

extern void __iomem *twd_base;
static void __init mx6_curie_timer_init(void)
{
	struct clk *uart_clk;
#ifdef CONFIG_LOCAL_TIMERS
	twd_base = ioremap(LOCAL_TWD_ADDR, SZ_256);
	BUG_ON(!twd_base);
#endif
	mx6_clocks_init(32768, 24000000, 0, 0);

	uart_clk = clk_get_sys("imx-uart.0", NULL);
	early_console_setup(UART1_BASE_ADDR, uart_clk);
}

static struct sys_timer mx6_curie_timer = {
	.init   = mx6_curie_timer_init,
};

static void __init mx6q_curie_reserve(void)
{
}

/*
 * initialize __mach_desc_MX6Q_CURIE data structure.
 */
MACHINE_START(MX6Q_SABRESD, "i.MX 6Quad/Dual Curie Board")
	.boot_params = MX6_PHYS_OFFSET + 0x100,
	.fixup = fixup_mxc_board,
	.map_io = mx6_map_io,
	.init_irq = mx6_init_irq,
	.init_machine = mx6_curie_board_init,
	.timer = &mx6_curie_timer,
	.reserve = mx6q_curie_reserve,
MACHINE_END
