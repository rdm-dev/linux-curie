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

/* Thermal */
static const struct anatop_thermal_platform_data
		mx6q_curie_anatop_thermal_data __initconst = {
		.name = "anatop_thermal",
};

/* VMMC Regulator */
static struct regulator_consumer_supply mx6q_curie_vmmc_consumers[] = {
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.1"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.2"),
	REGULATOR_SUPPLY("vmmc", "sdhci-esdhc-imx.3"),
};

static struct regulator_init_data mx6q_curie_vmmc_init = {
	.num_consumer_supplies = ARRAY_SIZE(mx6q_curie_vmmc_consumers),
	.consumer_supplies = mx6q_curie_vmmc_consumers,
};

static struct fixed_voltage_config mx6q_curie_vmmc_reg_config = {
	.supply_name		= "vmmc",
	.microvolts		= 3300000,
	.gpio			= -1,
	.init_data		= &mx6q_curie_vmmc_init,
};

static struct platform_device mx6q_curie_vmmc_reg_devices = {
	.name	= "reg-fixed-voltage",
	.id	= 3,
	.dev	= {
		.platform_data = &mx6q_curie_vmmc_reg_config,
	},
};

/* SDHC2 for Wi-Fi Module */
static const struct esdhc_platform_data mx6q_curie_sd2_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

/* SDHC3 for SD Card Slot */
#define CURIE_SD3_CD        IMX_GPIO_NR(6, 8)
#define CURIE_SD3_WP        IMX_GPIO_NR(6, 15)
static const struct esdhc_platform_data mx6q_curie_sd3_data __initconst = {
	.cd_gpio = CURIE_SD3_CD,
	.wp_gpio = CURIE_SD3_WP,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_CONTROLLER,
};

/* SDHC4 for eMMC */
static const struct esdhc_platform_data mx6q_curie_sd4_data __initconst = {
	.always_present = 1,
	.keep_power_at_suspend = 1,
	.support_8bit = 1,
	.delay_line = 0,
	.cd_type = ESDHC_CD_PERMANENT,
};

/* SATA */
static struct clk *sata_clk;
/* HW Initialization, if return 0, initialization is successful. */
static int mx6q_curie_sata_init(struct device *dev, void __iomem *mmio)
{
	u32 tmpdata;
	int ret = 0, i;
	struct clk *clk;

	sata_clk = clk_get(dev, "imx_sata_clk");
	if (IS_ERR(sata_clk)) {
		dev_err(dev, "no sata clock.\n");
		return PTR_ERR(sata_clk);
	}
	ret = clk_enable(sata_clk);
	if (ret) {
		dev_err(dev, "can't enable sata clock.\n");
		goto put_sata_clk;
	}

	/* Set PHY Paremeters, two steps to configure the GPR13,
	 * one write for rest of parameters, mask of first write is 0x07FFFFFD,
	 * and the other one write for setting the mpll_clk_off_b
	 *.rx_eq_val_0(iomuxc_gpr13[26:24]),
	 *.los_lvl(iomuxc_gpr13[23:19]),
	 *.rx_dpll_mode_0(iomuxc_gpr13[18:16]),
	 *.sata_speed(iomuxc_gpr13[15]),
	 *.mpll_ss_en(iomuxc_gpr13[14]),
	 *.tx_atten_0(iomuxc_gpr13[13:11]),
	 *.tx_boost_0(iomuxc_gpr13[10:7]),
	 *.tx_lvl(iomuxc_gpr13[6:2]),
	 *.mpll_ck_off(iomuxc_gpr13[1]),
	 *.tx_edgerate_0(iomuxc_gpr13[0]),
	 */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x07FFFFFF) | 0x0593E4C4), IOMUXC_GPR13);

	/* enable SATA_PHY PLL */
	tmpdata = readl(IOMUXC_GPR13);
	writel(((tmpdata & ~0x2) | 0x2), IOMUXC_GPR13);

	usleep_range(100, 200);
	sata_phy_cr_addr(SATA_PHY_CR_CLOCK_RESET, mmio);
	sata_phy_cr_write(SATA_PHY_CR_RESET_EN, mmio);
	usleep_range(100, 200);
	/* waiting for the rx_pll is stable */
	for (i = 0; i <= 5; i++) {
		sata_phy_cr_addr(SATA_PHY_CR_LANE0_OUT_STAT, mmio);
		sata_phy_cr_read(&ret, mmio);
		if (ret & SATA_PHY_CR_LANE0_RX_STABLE) {
			pr_info("sata phy rx_pll is stable!\n");
			break;
		} else if (i == 5)
			pr_info("wating for sata rx_pll lock time out\n");
		usleep_range(1000, 2000);
	}

	/* Get the AHB clock rate, and configure the TIMER1MS reg later */
	clk = clk_get(NULL, "ahb");
	if (IS_ERR(clk)) {
		dev_err(dev, "no ahb clock.\n");
		ret = PTR_ERR(clk);
		goto release_sata_clk;
	}
	tmpdata = clk_get_rate(clk) / 1000;
	clk_put(clk);

#ifdef CONFIG_SATA_AHCI_PLATFORM
	ret = sata_init(mmio, tmpdata);
	if (ret == 0)
		return ret;
#else
	usleep_range(1000, 2000);
	/* AHCI PHY enter into PDDQ mode if the AHCI module is not enabled */
	tmpdata = readl(mmio + PORT_PHY_CTL);
	writel(tmpdata | PORT_PHY_CTL_PDDQ_LOC, mmio + PORT_PHY_CTL);
	pr_info("No AHCI save PWR: PDDQ %s\n", ((readl(mmio + PORT_PHY_CTL)
					>> 20) & 1) ? "enabled" : "disabled");
#endif

release_sata_clk:
	/* disable SATA_PHY PLL */
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);
put_sata_clk:
	clk_put(sata_clk);

	return ret;
}

#ifdef CONFIG_SATA_AHCI_PLATFORM
static void mx6q_curie_sata_exit(struct device *dev)
{
	clk_disable(sata_clk);
	clk_put(sata_clk);
}

static int imx_ahci_suspend(struct device *dev)
{
	writel((readl(IOMUXC_GPR13) & ~0x2), IOMUXC_GPR13);
	clk_disable(sata_clk);

	return 0;
}

static int imx_ahci_resume(struct device *dev)
{
	int ret;

	ret = clk_enable(sata_clk);
	if (ret)
		dev_err(dev, "can't enable sata clock.\n");

	writel(((readl(IOMUXC_GPR13) & ~0x2) | 0x2), IOMUXC_GPR13);

	return 0;
}

static struct ahci_platform_data mx6q_curie_sata_data = {
	.init = mx6q_curie_sata_init,
	.exit = mx6q_curie_sata_exit,
	.suspend = imx_ahci_suspend,
	.resume = imx_ahci_resume,
};
#endif

/* Wi-Fi */
#define CURIE_WIFI_CS        IMX_GPIO_NR(2, 4)
static void __init mx6q_curie_init_wifi(void)
{
	gpio_request(CURIE_WIFI_CS, "wifi-cs");
	gpio_direction_output(CURIE_WIFI_CS, 1);
	mdelay(10);
	gpio_direction_output(CURIE_WIFI_CS, 0);
}

/* LED */
#if defined(CONFIG_LEDS_TRIGGER) || defined(CONFIG_LEDS_GPIO)

#define CURIE_LED_BOOT       IMX_GPIO_NR(7, 12)
#define CURIE_LED_WIFI       IMX_GPIO_NR(1, 6)
#define CURIE_LED_ERROR      IMX_GPIO_NR(1, 1)
#define CURIE_LED_USER1      IMX_GPIO_NR(1, 2)
#define CURIE_LED_USER2      IMX_GPIO_NR(1, 9)

#define GPIO_LED(gpio_led, name_led, act_low, state_suspend, trigger)	\
{									\
	.gpio			= gpio_led,				\
	.name			= name_led,				\
	.active_low		= act_low,				\
	.retain_state_suspended = state_suspend,			\
	.default_state		= 0,					\
	.default_trigger	= trigger,		\
}

// for Curie, the leds are active low
static struct gpio_led mx6q_curie_gpio_leds[] = {
	GPIO_LED(CURIE_LED_BOOT, "boot", 1, 1, "heartbeat"),
	GPIO_LED(CURIE_LED_WIFI, "wifi", 1, 1, "wifi"),
	GPIO_LED(CURIE_LED_ERROR, "error", 1, 1, "error"),
	GPIO_LED(CURIE_LED_USER1, "user1", 1, 1, "user1"),
	GPIO_LED(CURIE_LED_USER2, "user2", 1, 1, "user2"),
};

static struct gpio_led_platform_data mx6q_curie_gpio_leds_data = {
	.leds		= mx6q_curie_gpio_leds,
	.num_leds	= ARRAY_SIZE(mx6q_curie_gpio_leds),
};

static struct platform_device mx6q_curie_gpio_led_device = {
	.name		= "leds-gpio",
	.id		= -1,
	.num_resources  = 0,
	.dev		= {
		.platform_data = &mx6q_curie_gpio_leds_data,
	}
};

static void __init mx6q_curie_init_leds(void)
{
	platform_device_register(&mx6q_curie_gpio_led_device);
}
#else
static void __init mx6q_curie_init_leds(void) {}
#endif

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
	/* Thermal */
	imx6q_add_anatop_thermal_imx(1, &mx6q_curie_anatop_thermal_data);
	/* VMMC Regulator */
	platform_device_register(&mx6q_curie_vmmc_reg_devices);
	/* SDHC
	   Move sd4 to first because sd4 connect to emmc.
	   Mfgtools want emmc is mmcblk0 and other sd card is mmcblk1.
	 */
	imx6q_add_sdhci_usdhc_imx(3, &mx6q_curie_sd4_data);
	imx6q_add_sdhci_usdhc_imx(1, &mx6q_curie_sd2_data);
	imx6q_add_sdhci_usdhc_imx(2, &mx6q_curie_sd3_data);
	/* SATA, mx6q only */
	if (cpu_is_mx6q()) {
#ifdef CONFIG_SATA_AHCI_PLATFORM
		imx6q_add_ahci(0, &mx6q_curie_sata_data);
#else
		mx6q_curie_sata_init(NULL,
			(void __iomem *)ioremap(MX6Q_SATA_BASE_ADDR, SZ_4K));
#endif
	}
	/* OCOTP */
	imx6q_add_otp();
	imx6q_add_viim();
	/* Wi-Fi */
	mx6q_curie_init_wifi();
	/* LED */
	mx6q_curie_init_leds();
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
