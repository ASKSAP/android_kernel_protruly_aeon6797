/* ============================================================================	*/
/*                                                                            	*/
/*   bluewhale_charger.h                                                      	*/
/*   (c) 2001 Author                                                          	*/
/*                                                                            	*/
/*   Description                                                              	*/
/* This program is free software; you can redistribute it and/or modify it    	*/
/* under the terms of the GNU General Public License version 2 as published   	*/
/* by the Free Software Foundation.						*/
/*										*/
/* This program is distributed in the hope that it will be useful, but        	*/
/* WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY 	*/
/* or FITNESS FOR A PARTICULAR PURPOSE.						*/
/* See the GNU General Public License for more details.				*/
/*										*/
/* You should have received a copy of the GNU General Public License along	*/
/* with this program.  If not, see <http://www.gnu.org/licenses/>.		*/
/* ============================================================================	*/

#ifndef	__BLUEWHALE_CHARGER_H
#define	__BLUEWHALE_CHARGER_H __FILE__


/**
 *	 Attention Please!
 * DO NOT Change the following configuration 
 * unless you understand the source code thoroughly.
 */



/*
 * BSP-Related SETTING
 */

#define CHARGER_I2C_NUM 0
#define CHARGER_RTHM	0


/*
 * CHARGER HW DEFAULT SETTING
 */
#define CHGR_DEF_CC		1000	
#define CHGR_DEF_CV		4200
#define CHGR_DEF_CV_T34		4200
#define CHGR_DEF_CV_T45		4200
#define CHGR_DEF_ILIMIT		500

#define CHGR_DEF_EOC		80
#define CHGR_DEF_WK_VOL		3000
#define CHGR_DEF_WK_CUR		400
#define CHGR_DEF_VSYS		3600
#define CHGR_DEF_RECHG_VOL	100
#define CHGR_DEF_WK_TIMER	0
#define CHGR_DEF_CC_TIMER	0



/*
 * O2 CHARGER SOFTWARE CONFIGURATION
 */
#define O2_CONFIG_DYNAMIC_CV_SUPPORT 	1
#define O2_CONFIG_MTK_PEP_SUPPORT	1
#define O2_CONFIG_MTK_PLATFORM_SUPPORT	1
#define O2_CONFIG_OZ1C105G_SUPPORT	0

/*
 * functions exported
 */
int32_t bluewhale_get_charging_status_extern(void);
int32_t bluewhale_dump_register_extern(void);

int32_t bluewhale_set_vbus_current_extern(int32_t ilmt_ma);
int32_t bluewhale_get_vbus_current_extern(void);
int32_t bluewhale_set_charger_current_extern(int32_t chg_ma);
int32_t bluewhale_get_charger_current_extern(void);
int32_t bluewhale_set_chg_volt_extern(int32_t chgvolt_mv);
int32_t bluewhale_get_chargingfull_status_extern(void);
int32_t bluewhale_cable_out_config_extern(void);
int32_t bluewhale_power_off_config_extern(void);

int32_t bluewhale_init_extern(void);
int32_t bluewhale_enable_otg_extern(int32_t enable);
int32_t bluewhale_chip_suspend_extern(int32_t enable);

int32_t bluewhale_set_eoc_current_extern(int32_t eoc_ma);


#if O2_CONFIG_MTK_PEP_SUPPORT
int32_t bluewhale_PEP_increase_extern(int32_t increase);
int32_t bluewhale_PEP_reset_extern(void) ;
#endif




#endif //__BLUEWHALE_CHARGER_H
