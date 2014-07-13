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

#ifndef _BOARD_MX6Q_CURIE_H
#define _BOARD_MX6Q_CURIE_H
#include <mach/iomux-mx6q.h>

static iomux_v3_cfg_t mx6q_curie_pads[] = {
	/* UART1 for debug */
	MX6Q_PAD_CSI0_DAT10__UART1_TXD,
	MX6Q_PAD_CSI0_DAT11__UART1_RXD,

	/* USBOTG ID pin */
	MX6Q_PAD_ENET_RX_ER__ANATOP_USBOTG_ID,

	/* ENET */
	MX6Q_PAD_ENET_MDIO__ENET_MDIO,
	MX6Q_PAD_ENET_MDC__ENET_MDC,
	MX6Q_PAD_RGMII_TXC__ENET_RGMII_TXC,
	MX6Q_PAD_RGMII_TD0__ENET_RGMII_TD0,
	MX6Q_PAD_RGMII_TD1__ENET_RGMII_TD1,
	MX6Q_PAD_RGMII_TD2__ENET_RGMII_TD2,
	MX6Q_PAD_RGMII_TD3__ENET_RGMII_TD3,
	MX6Q_PAD_RGMII_TX_CTL__ENET_RGMII_TX_CTL,
	MX6Q_PAD_ENET_REF_CLK__ENET_TX_CLK,
	MX6Q_PAD_RGMII_RXC__ENET_RGMII_RXC,
	MX6Q_PAD_RGMII_RD0__ENET_RGMII_RD0,
	MX6Q_PAD_RGMII_RD1__ENET_RGMII_RD1,
	MX6Q_PAD_RGMII_RD2__ENET_RGMII_RD2,
	MX6Q_PAD_RGMII_RD3__ENET_RGMII_RD3,
	MX6Q_PAD_RGMII_RX_CTL__ENET_RGMII_RX_CTL,
	/* RGMII_nRST */
	MX6Q_PAD_ENET_CRS_DV__GPIO_1_25,
	/* RGMII Interrupt */
	MX6Q_PAD_ENET_RXD1__GPIO_1_26,

	/* I2C2 for HDMI */
	MX6Q_PAD_EIM_D16__I2C2_SDA,
	MX6Q_PAD_EIM_EB2__I2C2_SCL,

	/* I2C3 for PMIC */
	MX6Q_PAD_EIM_D18__I2C3_SDA,
	MX6Q_PAD_EIM_D17__I2C3_SCL,

};

#endif

