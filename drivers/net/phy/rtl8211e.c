/*
 * drivers/net/phy/rtl8211e.c
 *
 * Driver for Realtek PHY RTL8211E
 *
 * Author: Cui Gao <gao.cui@windsolve.com>
 *
 * Copyright (c) 2014 Shanghai Zhixing Information Technology Co.Ltd
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/phy.h>

#define RTL8211E_PHYSR		0x11
#define RTL8211E_PHYSR_DUPLEX	0x2000
#define RTL8211E_PHYSR_SPEED	0xc000
#define RTL8211E_INER		0x12
#define RTL8211E_INER_INIT	0x0400 /* 0x0400: Link Changed */
#define RTL8211E_INSR		0x13

MODULE_DESCRIPTION("PHY RTL8211E driver");
MODULE_AUTHOR("Cui Gao");
MODULE_LICENSE("GPL");

static int rtl8211e_ack_interrupt(struct phy_device *phydev)
{
	int err;

	err = phy_read(phydev, RTL8211E_INSR);

	return (err < 0) ? err : 0;
}

static int rtl8211e_config_intr(struct phy_device *phydev)
{
	int err;

	if (phydev->interrupts == PHY_INTERRUPT_ENABLED)
		err = phy_write(phydev, RTL8211E_INER,
				RTL8211E_INER_INIT);
	else
		err = phy_write(phydev, RTL8211E_INER, 0);

	return err;
}

/* RTL8211B */
static struct phy_driver rtl8211e_driver = {
	.phy_id		= 0x001cc915,
	.name		= "RTL8211E Gigabit Ethernet",
	.phy_id_mask	= 0x001fffff,
	.features	= PHY_GBIT_FEATURES,
	.flags		= PHY_HAS_INTERRUPT,
	.config_aneg	= &genphy_config_aneg,
	.read_status	= &genphy_read_status,
	.ack_interrupt	= &rtl8211e_ack_interrupt,
	.config_intr	= &rtl8211e_config_intr,
	.driver		= { .owner = THIS_MODULE,},
};

static int __init rtl8211e_init(void)
{
	int ret;

	ret = phy_driver_register(&rtl8211e_driver);

	return ret;
}

static void __exit rtl8211e_exit(void)
{
	phy_driver_unregister(&rtl8211e_driver);
}

module_init(rtl8211e_init);
module_exit(rtl8211e_exit);

static struct mdio_device_id __maybe_unused rtl8211e_tbl[] = {
	{ 0x001cc915, 0x001fffff },
	{ }
};

MODULE_DEVICE_TABLE(mdio, rtl8211e_tbl);
