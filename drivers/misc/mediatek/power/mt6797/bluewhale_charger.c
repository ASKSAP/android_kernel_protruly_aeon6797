/*****************************************************************************
* Copyright(c) O2Micro, 2013. All rights reserved.
*	
* O2Micro OZ8806 battery gauge driver
* File: bluewhale_charger.c

* Author: Eason.yuan
* $Source: /data/code/CVS
* $Revision:  $
*
* This program is free software and can be edistributed and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*	
* This Source Code Reference Design for O2MICRO OZ8806 access (\u201cReference Design\u201d) 
* is sole for the use of PRODUCT INTEGRATION REFERENCE ONLY, and contains confidential 
* and privileged information of O2Micro International Limited. O2Micro shall have no 
* liability to any PARTY FOR THE RELIABILITY, SERVICEABILITY FOR THE RESULT OF PRODUCT 
* INTEGRATION, or results from: (i) any modification or attempted modification of the 
* Reference Design by any party, or (ii) the combination, operation or use of the 
* Reference Design with non-O2Micro Reference Design.
*****************************************************************************/


/**
 * Section: header files
 */
#include <linux/kernel.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/jiffies.h>

#include "bluewhale_charger.h"

#if O2_CONFIG_MTK_PLATFORM_SUPPORT
#include <mach/mt_charging.h>
#include <mt-plat/charging.h>
#include <mt-plat/battery_meter.h>
#include <mt-plat/battery_common.h>
#include <mt-plat/battery_meter_hal.h> 
#include <mach/mt_battery_meter.h>
#endif


#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif


#if O2_CONFIG_OZ1C105G_SUPPORT
extern int32_t  afe_read_cell_volt(int32_t *voltage);
extern int32_t  afe_read_current(int32_t *dat);
#endif




/**
 * Section: global typedef && param definition && reg definition
 */

/* Voltage Registers (R/W) */
#define		REG_CHARGER_VOLTAGE		0x00
#define		REG_T34_CHG_VOLTAGE		0x01
#define		REG_T45_CHG_VOLTAGE		0x02
  #define	  CHG_VOLT_STEP			  25	//step 25mV

#define		REG_WAKEUP_VOLTAGE		0x03
  #define	  WK_VOLT_STEP			  100	//step 100mV

#define		REG_RECHARGE_HYSTERESIS		0x04
  #define	  RECHG_VOLT_STEP		  50	//step 50mV

#define		REG_MIN_VSYS_VOLTAGE		0x05
  #define	  VSYS_VOLT_STEP		  200	//step 200mV


/* Current Registers (R/W) */
#define		REG_CHARGE_CURRENT		0x10
  #define	  CHG_CURRT_STEP		  100	//step 100mA

#define		REG_WAKEUP_CURRENT		0x11
  #define	  WK_CURRT_STEP			  10	//step 10mA

#define		REG_END_CHARGE_CURRENT		0x12
  #define	  EOC_CURRT_STEP		  10	//step 10mA

#define		REG_VBUS_LIMIT_CURRENT		0x13
  #define	  VBUS_ILMT_STEP		  100	//step 100mA


/* Protection Register (R/W) */
#define		REG_SAFETY_TIMER		0x20
  #define	  WAKEUP_TIMER_MASK		  0x0F
  #define	  WK_TIMER_15MIN		  0x01	//15min wakeup timer
  #define	  WK_TIMER_30MIN		  0x02	//30min wakeup timer
  #define	  WK_TIMER_45MIN		  0x03	//45min wakeup timer
  #define	  WK_TIMER_60MIN		  0x04	//60min wakeup timer
  #define	  WK_TIMER_75MIN		  0x05	//60min wakeup timer
  #define 	  WK_TIMER_90MIN		  0x06	//60min wakeup timer
  #define	  WK_TIMER_105MIN		  0x07	//60min wakeup timer
  #define 	  CC_TIMER_MASK			  0xF0
  #define	  CC_TIMER_120MIN		  0x10	//120min CC charge timer
  #define	  CC_TIMER_180MIN		  0x20	//180min CC charge timer
  #define	  CC_TIMER_240MIN		  0x30	//240min CC charge timer
  #define	  CC_TIMER_300MIN		  0x40	//300min CC charge timer
  #define	  CC_TIMER_390MIN		  0x50	//390min CC charge timer
  #define	  CC_TIMER_480MIN		  0x60	//480min CC charge timer
  #define	  CC_TIMER_570MIN		  0x70	//570min CC charge timer


/* Charger Control Register (R/W) */
#define		REG_CHARGER_CONTROL		0x30
  #define	  RTHM_SELECT			  0x01	//0:100K, 1:10K

/* Status Registers (R) */
#define		REG_VBUS_STATUS			0x40
  #define	  VSYS_OVP_FLAG			  0x01	//VSYS OVP event flag
  #define	  VBUS_UVP_FLAG			  0x10	//VBUS UVP event flag
  #define	  VBUS_OK_FLAG			  0x20	//VBUS OK flag
  #define	  VBUS_OVP_FLAG			  0x40	//VBUS OVP event flag
  #define	  VDC_PR_FLAG                     0x80 	// 1 when VDC < VDC threshold for system priority

#define		REG_CHARGER_STATUS		0x41
  #define	  CHARGER_INIT			  0x01	//Before init flag
  #define	  IN_WAKEUP_STATE		  0x02	//In Wakeup State
  #define	  IN_CC_STATE			  0x04	//In CC Charge State
  #define	  IN_CV_STATE			  0x08	//In CV Charge State
  #define	  CHARGE_FULL_STATE		  0x10	//Charge Full State
  #define	  WK_TIMER_FLAG			  0x20	//WK CHG Timer Overflow
  #define	  CC_TIMER_FLAG			  0x40	//CC CHG Timer Overflow 

#define		REG_THM_STATUS			0x42
  #define	  THM_T1_STATE			  0x01	//T1 > THM state 
  #define	  THM_T12_STATE			  0x02	//THM in T12 state 
  #define	  THM_T23_STATE			  0x04	//THM in T23 state 
  #define	  THM_T34_STATE			  0x08	//THM in T34 state 
  #define	  THM_T45_STATE			  0x10	//THM in T45 state 
  #define	  THM_T5_STATE			  0x20	//THM > T5 state 
  #define	  INT_OTP_FLAG			  0x40	//Internal OTP event

#define BW_ABS(a,b) ( ((a) > (b)) ? ((a) - (b)) : ((b) - (a)) )

typedef enum {
	THM_DISABLE = 0,
        THM_UNDER_T1,
        THM_RANGE_T12,
        THM_RANGE_T23,
        THM_RANGE_T34,
        THM_RANGE_T45,
        THM_OVER_T5,
	THM_ITOT
} bluewhale_thermal ;

struct bluewhale_charger_info {
	struct device		*dev;
	struct i2c_client	*client;

        bool			vbus_ovp;
        bool			vbus_ok;
        bool			vbus_uvp;
        bool			vsys_ovp;
        bool			vdc_pr;
	bool			cc_timeout;
	bool			wak_timeout;
	bool			chg_full;
	bool			in_cc_state;
	bool			in_cv_state;
	bool			in_wakeup_state;
	bool			initial_state;
	bluewhale_thermal	thermal_status;

	int32_t       		chg_volt;
	int32_t        		t34_cv;
	int32_t        		t45_cv;
	int32_t        		wakeup_volt;
	int32_t        		eoc_current;
	int32_t        		vbus_current;
	int32_t        		rechg_hystersis;
	int32_t        		charger_current;
	int32_t        		wakeup_current;
	int32_t        		safety_cc_timer;
	int32_t        		safety_wk_timer;	
		
};

#define	RETRY_CNT	5

#define DBG_LEVEL KERN_ERR
#define bluewhale_dbg(fmt, args...) printk(DBG_LEVEL "[bluewhale]:" pr_fmt(fmt) "\n", ## args)
#define ilimit_dbg(fmt, args...) printk(DBG_LEVEL "[ilimit_work]:" pr_fmt(fmt) "\n", ## args)








/**
 *	Section: global variables
 */
static struct bluewhale_charger_info *charger_info = NULL;
static DEFINE_MUTEX(bluewhale_mutex);


#if O2_CONFIG_MTK_PLATFORM_SUPPORT
kal_bool chargin_hw_init_done = false;
#endif










/**
 *	Section: function prototypes
 */








/**
 *	Section: function definitions
 */


/*****************************************************************************
 * Description:
 *		bluewhale_read_byte 
 * Parameters:
 *		charger:	charger data
 *		index:	register index to be read
 *		*dat:	buffer to store data read back
 * Return:
 *      negative errno else a data byte received from the device.
 *****************************************************************************/
static int32_t bluewhale_read_byte(struct bluewhale_charger_info *charger, uint8_t index, uint8_t *dat)
{
	int32_t ret;
	uint8_t i;
	struct i2c_client *client = charger->client;

	for(i = 0; i < RETRY_CNT; i++) {
		ret = i2c_smbus_read_byte_data(client, index);

		if(ret >= 0) 
			break;
		else
			dev_err(&client->dev, "%s: err %d, %d times\n", __func__, ret, i);
	}

	if(i >= RETRY_CNT) {
		return ret;
	}
 
	*dat = (uint8_t)ret;

	return ret;
}


/*****************************************************************************
 * Description:
 *		bluewhale_write_byte 
 * Parameters:
 *		charger:	charger data
 *		index:	register index to be write
 *		dat:		write data
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_write_byte(struct bluewhale_charger_info *charger, uint8_t index, uint8_t dat)
{
	int32_t ret;
	uint8_t i;
	struct i2c_client *client = charger->client;
	
	for(i = 0; i < RETRY_CNT; i++) {
		ret = i2c_smbus_write_byte_data(client, index, dat);
		if(ret >= 0) 
			break;
		else
			dev_err(&client->dev, "%s: err %d, %d times\n", __func__, ret, i);
	}

	if(i >= RETRY_CNT) {
		return ret;
	}

	return ret;
}

static int32_t bluewhale_update_bits(struct bluewhale_charger_info *charger, uint8_t reg, uint8_t mask, uint8_t data)
{
	int32_t ret;
	uint8_t tmp;

	ret = bluewhale_read_byte(charger, reg, &tmp);
	if (ret < 0)
		return ret;

	if ( (tmp & mask) != data) {
		tmp &= ~mask;
		tmp |= data & mask;
		return bluewhale_write_byte(charger, reg, tmp);
	} else {
		return 0;
	}
}



/*****************************************************************************
 * Description:
 *		bluewhale_set_chg_volt 
 * Parameters:
 *		charger:	charger data
 *		chgvolt_mv:	charge voltage to be written
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t adji_get_bat_volt(void) ;

static int32_t bluewhale_set_chg_volt(struct bluewhale_charger_info *charger, int32_t chgvolt_mv)
{
	int32_t ret = 0;
	u8 chg_volt = 0;

#if O2_CONFIG_DYNAMIC_CV_SUPPORT

#define O2_DYNAMIC_CV_THRESHOLD (3900)

	int32_t vbat = 0;

	vbat = adji_get_bat_volt() ;

	if(vbat < O2_DYNAMIC_CV_THRESHOLD) {
		if(chgvolt_mv > 4000)
			chgvolt_mv = 4000 ;

		printk("[DCV 1] chgvolt_mv: %d\n", chgvolt_mv);
	
	} else {
		int32_t aaa = 0;
		int32_t bbb = 0;
		int32_t newCV = 0;
		
		aaa = vbat / CHG_VOLT_STEP ;
		bbb = vbat % CHG_VOLT_STEP ;
		
		newCV = aaa * CHG_VOLT_STEP + 100;
		if(bbb > 0) {
			newCV += CHG_VOLT_STEP ;
		}
		
		if(chgvolt_mv > newCV)
			chgvolt_mv = newCV ;

		printk("[DCV 2] chgvolt_mv: %d\n", chgvolt_mv);
	}

#endif //O2_CONFIG_DYNAMIC_CV_SUPPORT

	if (chgvolt_mv < 4000)
		chgvolt_mv = 4000;		//limit to 4.0V
	else if (chgvolt_mv > 4600)
		chgvolt_mv = 4600;		//limit to 4.6V

	chg_volt = (chgvolt_mv - 4000) / CHG_VOLT_STEP;	//step is 25mV

	printk("[DCV 3] chg_volt: %d\n", chg_volt);
	
	ret = bluewhale_update_bits(charger, REG_CHARGER_VOLTAGE, 0x1f, chg_volt);

	return ret;
}

static int32_t bluewhale_set_chg_volt_intern(struct bluewhale_charger_info *charger, int32_t chgvolt_mv)
{
	int32_t ret = 0;
	u8 chg_volt = 0;

	if (chgvolt_mv < 4000)
		chgvolt_mv = 4000;		//limit to 4.0V
	else if (chgvolt_mv > 4600)
		chgvolt_mv = 4600;		//limit to 4.6V

	chg_volt = (chgvolt_mv - 4000) / CHG_VOLT_STEP;	//step is 25mV

	ret = bluewhale_update_bits(charger, REG_CHARGER_VOLTAGE, 0x1f, chg_volt);

	return ret;
}


static int32_t bluewhale_get_chg_volt(struct bluewhale_charger_info *charger)
{
	uint8_t data;
	int32_t ret = 0;

	ret = bluewhale_read_byte(charger, REG_CHARGER_VOLTAGE, &data);
	if (ret < 0) {
		return ret;
	}
	return (data * CHG_VOLT_STEP);	//step is 25mA
}

/*****************************************************************************
 * Description:
 *		bluewhale_set_t34_cv
 * Parameters:
 *		charger:	charger data
 *		chgvolt_mv:	charge voltage to be written at t34
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_t34_cv(struct bluewhale_charger_info *charger, int32_t chgvolt_mv)
{
	int32_t ret = 0;
	u8 chg_volt = 0; 

	if (chgvolt_mv < 4000)
		chgvolt_mv = 4000;		//limit to 4.0V
	else if (chgvolt_mv > 4600)
		chgvolt_mv = 4600;		//limit to 4.6V

	chg_volt = (chgvolt_mv - 4000) / CHG_VOLT_STEP;	//step is 25mV

	ret = bluewhale_update_bits(charger, REG_T34_CHG_VOLTAGE, 0x1f, chg_volt);

	return ret;
}

/*****************************************************************************
 * Description:
 *		bluewhale_set_t45_cv 
 * Parameters:
 *		charger:	charger data
 *		chgvolt_mv:	charge voltage to be written at t45
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_t45_cv(struct bluewhale_charger_info *charger, int32_t chgvolt_mv)
{
	int32_t ret = 0;
	u8 chg_volt = 0; 

	if (chgvolt_mv < 4000)
		chgvolt_mv = 4000;		//limit to 4.0V
	else if (chgvolt_mv > 4600)
		chgvolt_mv = 4600;		//limit to 4.6V

	chg_volt = (chgvolt_mv - 4000) / CHG_VOLT_STEP;	//step is 25mV

	ret = bluewhale_update_bits(charger, REG_T45_CHG_VOLTAGE, 0x1f, chg_volt);

	return ret;
}

/*****************************************************************************
 * Description:
 *		bluewhale_set_wakeup_volt 
 * Parameters:
 *		charger:	charger data
 *		wakeup_mv:	set wake up voltage
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_wakeup_volt(struct bluewhale_charger_info *charger, int32_t wakeup_mv)
{
	int32_t ret = 0;
	u8 wak_volt = 0; 

	if (wakeup_mv < 1500)
		wakeup_mv = 1500;		//limit to 1.5V
	else if (wakeup_mv > 3000)
		wakeup_mv = 3000;		//limit to 3.0V

	wak_volt = wakeup_mv / WK_VOLT_STEP;	//step is 100mV

	ret = bluewhale_update_bits(charger, REG_WAKEUP_VOLTAGE, 0x1f, wak_volt);

	return ret;
}

/*****************************************************************************
 * Description:
 *		bluewhale_set_rechg_hystersis
 * Parameters:
 *		charger:	charger data
 *		hyst_mv:	set Recharge hysteresis Register
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_rechg_hystersis(struct bluewhale_charger_info *charger, int32_t hyst_mv)
{
	int32_t ret = 0;
	u8 rechg = 0; 
		
	if (hyst_mv > 200) {
		hyst_mv = 200;			//limit to 200mV
	}
	//Notice: with 00h, recharge function is disabled

	rechg = hyst_mv / RECHG_VOLT_STEP;	//step is 50mV
	
	ret = bluewhale_update_bits(charger, REG_RECHARGE_HYSTERESIS, 0x07, rechg);
	return ret;
}

/*****************************************************************************
 * Description:
 *		bluewhale_set_min_vsys 
 * Parameters:
 *		charger:	charger data
 *		min_vsys_mv:	min sys voltage to be written
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_min_vsys(struct bluewhale_charger_info *charger, int32_t min_vsys_mv)
{
	int32_t ret = 0;
	u8 vsys_val = 0; 

	if (min_vsys_mv < 1800)
		min_vsys_mv = 1800;		//limit to 1.8V
	else if (min_vsys_mv > 3600)
		min_vsys_mv = 3600;		//limit to 3.6V

	vsys_val = min_vsys_mv / VSYS_VOLT_STEP;	//step is 200mV

	ret = bluewhale_update_bits(charger, REG_MIN_VSYS_VOLTAGE, 0x1f, vsys_val);

	return ret;
}


/*****************************************************************************
 * Description:
 *		bluewhale_set_charger_current
 * Parameters:
 *		charger:	charger data
 *		chg_ma:	set charger current
 *		Only 600mA, 800mA, 1000mA, ... , 3800mA, 4000mA can be accurate
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_charger_current(struct bluewhale_charger_info *charger, int32_t chg_ma)
{
	int32_t ret = 0;
	u8 chg_curr = 0; 

	if (chg_ma < 600)
		chg_ma = 0 ;

	if (chg_ma > 4000)
		chg_ma = 4000;		//limit to 4A	

	if (chg_ma == 2000)
		chg_ma = 2200 ;

	chg_curr = chg_ma / (CHG_CURRT_STEP*2) ;	//step is 100mA
	chg_curr *= 2 ;

	ret = bluewhale_update_bits(charger, REG_CHARGE_CURRENT, 0x3f, chg_curr);
	//notice: chg_curr value less than 06h, charger will be disabled.
	//charger can power system in this case.

	return ret;
}

static int32_t bluewhale_get_charger_current(struct bluewhale_charger_info *charger)
{
	uint8_t data;
	int32_t ret = 0;

	ret = bluewhale_read_byte(charger, REG_CHARGE_CURRENT, &data);
	if (ret < 0)
		return ret;
	return (data * CHG_CURRT_STEP);	//step is 100mA
}


/*****************************************************************************
 * Description:
 *		bluewhale_set_eoc_current 
 * Parameters:
 *		charger:	charger data
 *		eoc_ma:	set end of charge current
 *		Only 0mA, 20mA, 40mA, ... , 320mA can be accurate
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_eoc_current(struct bluewhale_charger_info *charger, int32_t eoc_ma)
{
	int32_t ret = 0;
	u8 eoc_curr = 0; 

	//Notice: with 00h, end of charge function function is disabled
	if (eoc_ma <= 0)
		eoc_ma = 0;		//min value is 0mA
	else if (eoc_ma > 320)
		eoc_ma = 320;		//limit to 320mA

	eoc_curr = eoc_ma / (EOC_CURRT_STEP*2);	//step is 10mA
	eoc_curr *= 2 ;

	ret = bluewhale_update_bits(charger, REG_END_CHARGE_CURRENT, 0x3f, eoc_curr);
	return ret;
}

static int32_t bluewhale_get_eoc_current(struct bluewhale_charger_info *charger)
{
	uint8_t data;
	int32_t ret = 0;

	ret = bluewhale_read_byte(charger, REG_END_CHARGE_CURRENT, &data);
	if (ret < 0)
		return ret;
	return (data * EOC_CURRT_STEP);	//step is 10mA
}

/*****************************************************************************
 * Description:
 *		bluewhale_set_wakeup_current 
 * Parameters:
 *		charger:	charger data
 *		wak_ma:	set wakeup current
 *		Only 100mA, 120mA, ... , 400mA can be accurate
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_wakeup_current(struct bluewhale_charger_info *charger, int32_t wak_ma)
{
	int32_t ret = 0;
	u8 wak_curr = 0; 
	if (wak_ma < 100)
		wak_ma = 100;		//limit to 100mA
	if (wak_ma > 400)
		wak_ma = 400;		//limit to 400mA
	wak_curr = wak_ma / (WK_CURRT_STEP*2);	//step is 10mA
	wak_curr *= 2 ;

	ret = bluewhale_update_bits(charger, REG_WAKEUP_CURRENT, 0x3f, wak_curr);
	return ret;
}

/*****************************************************************************
 * Description:
 *		bluewhale_set_vbus_current
 * Parameters:
 *		charger:	charger data
 *		ilmt_ma:	set input current limit
 *		Only 100mA, 500mA, 700mA, 900mA, 1000mA, 1200mA, 1400mA, 1500mA, 1700mA, 1900mA, 2000mA can be accurate
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_vbus_current(struct bluewhale_charger_info *charger, int32_t ilmt_ma)
{
	int32_t ret = 0;
	u8 input_curr = 0;

	if(ilmt_ma < 300)
		input_curr = 0x01;		//100mA
	else if(ilmt_ma >= 300 && ilmt_ma < 600)
		input_curr = 0x05;		//500mA
	else if(ilmt_ma >= 600 && ilmt_ma < 800)
		input_curr = 0x0c;	//700mA
	else if(ilmt_ma >= 800 && ilmt_ma < 950)
		input_curr = 0x09;	//900mA
	else if(ilmt_ma >= 950 && ilmt_ma < 1100)
		input_curr = 0x10;	//1000mA
	else if(ilmt_ma >= 1100 && ilmt_ma < 1300)
		input_curr = 0x12;	//1200mA
	else if(ilmt_ma >= 1300 && ilmt_ma < 1450)
		input_curr = 0x0e;	//1400mA
	else if(ilmt_ma >= 1450 && ilmt_ma < 1600)
		input_curr = 0x0f;	//1500mA
	else if(ilmt_ma >= 1600 && ilmt_ma < 1800)
		input_curr = 0x11;	//1700mA
	else if(ilmt_ma >= 1800 && ilmt_ma < 1950)
		input_curr = 0x13;	//1900mA
	else if(ilmt_ma >= 1950)
		input_curr = 0x14;	//2000mA

	ret = bluewhale_update_bits(charger, REG_VBUS_LIMIT_CURRENT, 0x1f, input_curr);
	return ret;
}

/*****************************************************************************
 * Description:
 *		bluewhale_get_vbus_current
 * Parameters:
 *		charger:	charger data
 * Return:
 *		Vbus input current in mA
 *****************************************************************************/
static int32_t bluewhale_get_vbus_current(struct bluewhale_charger_info *charger)
{
	int32_t ret = 0;
	u8 data = 0;

	ret = bluewhale_read_byte(charger, REG_VBUS_LIMIT_CURRENT, &data);
	if (ret < 0)
		return ret;

	switch(data)
	{
		case 0x00: return 0;
		case 0x01: ;
		case 0x02: ;
		case 0x03: return 100;
		case 0x04: ;
		case 0x05: ;
		case 0x06: ;
		case 0x07: return 500;
		case 0x08: ;
		case 0x09: ;
		case 0x0a: ;
		case 0x0b: return 900;
		case 0x0c: ;
		case 0x0d: return 700;
		case 0x0e: return 1400;
		case 0x0f: return 1500;
		case 0x10: return 1000;
		case 0x11: return 1700;
		case 0x12: return 1200;
		case 0x13: return 1900;
		case 0x14: return 2000;
		default:
				   return -1;
	}
}

/*****************************************************************************
 * Description:
 *		bluewhale_set_safety_cc_timer
 * Parameters:
 *		charger:	charger data
 *		tmin:	set safety cc charge time
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_safety_cc_timer(struct bluewhale_charger_info *charger, int32_t tmin)
{
	int32_t ret = 0;
	u8 tval = 0; 

	//Notice: with 0xh, saftety timer function is disabled
	if (tmin == 0) {	//disable
		tval = 0;
	}
	else if (tmin == 120) {	//120min
		tval = CC_TIMER_120MIN;
	}
	else if (tmin == 180) {	//180min
		tval = CC_TIMER_180MIN;
	}
	else if (tmin == 240) {	//240min
		tval = CC_TIMER_240MIN;
	}
	else if (tmin == 300) {	//300min
		tval = CC_TIMER_300MIN;
	}
	else if (tmin == 390) {	//300min
		tval = CC_TIMER_390MIN;
	}
	else if (tmin == 480) {	//300min
		tval = CC_TIMER_480MIN;
	}
	else if (tmin == 570) {	//300min
		tval = CC_TIMER_570MIN;
	}
	else {				//invalid values
		printk("%s: invalide value, set default value 180 minutes\n", __func__);
		tval = CC_TIMER_180MIN;	//default value
	}

	ret = bluewhale_update_bits(charger, REG_SAFETY_TIMER, CC_TIMER_MASK, tval);

	return ret;
}

/*****************************************************************************
 * Description:
 *		bluewhale_set_safety_wk_timer 
 * Parameters:
 *		charger:	charger data
 *		tmin:	set safety wakeup time
 * Return:
 *      negative errno, zero on success.
 *****************************************************************************/
static int32_t bluewhale_set_safety_wk_timer(struct bluewhale_charger_info *charger, int32_t tmin)
{
	int32_t ret = 0;
	u8 tval = 0; 

	//Notice: with x0h, saftety timer function is disabled
	if (tmin == 0) {	//disable
		tval = 0;
	}
	else if (tmin == 15) {	//15min
		tval = WK_TIMER_15MIN;
	}
	else if (tmin == 30) {	//30min
		tval = WK_TIMER_30MIN;
	}
	else if (tmin == 45) {	//45min
		tval = WK_TIMER_45MIN;
	}
	else if (tmin == 60) {	//60min
		tval = WK_TIMER_60MIN;
	}
	else if (tmin == 75) {	//60min
		tval = WK_TIMER_75MIN;
	}
	else if (tmin == 90) {	//60min
		tval = WK_TIMER_90MIN;
	}
	else if (tmin == 105) {	//60min
		tval = WK_TIMER_105MIN;
	}
	else {				//invalid values
		printk("%s: invalide value, set default value 30 minutes\n", __func__);
		tval = WK_TIMER_30MIN;	//default value
	}

	ret = bluewhale_update_bits(charger, REG_SAFETY_TIMER, WAKEUP_TIMER_MASK, tval);
	return ret;
}










static int32_t bluewhale_get_setting_data(struct bluewhale_charger_info *charger)
{
	int32_t ret =0 ;

	if (!charger)
		return -EINVAL;

	ret = bluewhale_get_chg_volt(charger) ;
	if(ret < 0)
		return ret;
	else
		charger->chg_volt = ret ;

	ret = bluewhale_get_charger_current(charger) ;
	if(ret < 0)
		return ret;
	else
		charger->charger_current = ret ;

	ret = bluewhale_get_vbus_current(charger) ;
	if(ret < 0)
		return ret;
	else
		charger->vbus_current = ret ;

	ret = bluewhale_get_eoc_current(charger) ;
	if(ret < 0)
		return ret;
	else
		charger->eoc_current = ret ;

	return 0 ;
}

static int32_t bluewhale_check_charger_status(struct bluewhale_charger_info *charger)
{
	u8 data_status;
	u8 data_thm = 0;
	u8 i = 0;
	int32_t ret;
	
	/* Check charger status */
	ret = bluewhale_read_byte(charger, REG_CHARGER_STATUS, &data_status);
	if (ret < 0) {
		printk(KERN_ERR "%s: fail to get Charger status\n", __func__);
		return ret; 
	}

	charger->initial_state = (data_status & CHARGER_INIT) ? 1 : 0;
	charger->in_wakeup_state = (data_status & IN_WAKEUP_STATE) ? 1 : 0;
	charger->in_cc_state = (data_status & IN_CC_STATE) ? 1 : 0;
	charger->in_cv_state = (data_status & IN_CV_STATE) ? 1 : 0;
	charger->chg_full = (data_status & CHARGE_FULL_STATE) ? 1 : 0;
	charger->cc_timeout = (data_status & CC_TIMER_FLAG) ? 1 : 0;
	charger->wak_timeout = (data_status & WK_TIMER_FLAG) ? 1 : 0;
 
	/* Check thermal status*/
	ret = bluewhale_read_byte(charger, REG_THM_STATUS, &data_thm);
	if (ret < 0) {
		printk(KERN_ERR "%s: fail to get Thermal status\n", __func__);
		return ret; 
	}
	if (!data_thm)
		charger->thermal_status = THM_DISABLE;
	else {
		for (i = 0; i < 7; i ++) {
			if (data_thm & (1 << i))
				charger->thermal_status = i + 1;
		}
	}

	return ret;
}

static int32_t bluewhale_check_vbus_status(struct bluewhale_charger_info *charger)
{
	int32_t ret = 0;
	u8 data_status = 0;

	/* Check VBUS and VSYS */
	ret = bluewhale_read_byte(charger, REG_VBUS_STATUS, &data_status);

	if(ret < 0) {
		printk(KERN_ERR "%s: fail to get Vbus status\n", __func__);
	}

	charger->vdc_pr = (data_status & VDC_PR_FLAG) ? 1 : 0;
	charger->vsys_ovp = (data_status & VSYS_OVP_FLAG) ? 1 : 0;
	charger->vbus_ovp = (data_status & VBUS_OVP_FLAG) ? 1 : 0; //it's reserved for oz1c115
	charger->vbus_uvp = (data_status & VBUS_UVP_FLAG) ? 1 : 0;
	charger->vbus_ok = (data_status & VBUS_OK_FLAG) ? 1 : 0;

	if (!charger->vbus_ok)
		printk(KERN_ERR "%s: invalid adapter or no adapter\n", __func__);

	if (charger->vdc_pr)
		bluewhale_dbg("vdc < vdc threshold of system priority");

	return ret > 0 ? charger->vbus_ok : ret;
}

static void print_all_charger_info(struct bluewhale_charger_info *charger)
{
	printk("## bluewhale charger information ##\n"
		"chg_volt: %d, vbus_current: %d, charger_current: %d, eoc_current: %d\n"
		"vdc_pr: %d, vbus_ovp: %d, vbus_ok: %d, vbus_uvp: %d, vsys_ovp: %d,\n"
		"cc_timeout: %d, wak_timeout: %d, chg_full: %d, in_cc_state: %d, in_cv_state: %d\n"
		"initial_state: %d, in_wakeup_state: %d, thermal_status: %d\n",
		charger->chg_volt, charger->vbus_current,
		charger->charger_current,charger->eoc_current,
		charger->vdc_pr, charger->vbus_ovp, charger->vbus_ok,
		charger->vbus_uvp, charger->vsys_ovp,
		charger->cc_timeout, charger->wak_timeout, charger->chg_full,
		charger->in_cc_state, charger->in_cv_state,
		charger->initial_state,charger->in_wakeup_state,
		charger->thermal_status);
}

static void print_all_register(struct bluewhale_charger_info *charger)
{
	u8 i = 0;
	u8 data = 0;

    	printk(KERN_NOTICE "[bluewhale charger]:\n");

    	for (i=REG_CHARGER_VOLTAGE; i<=REG_MIN_VSYS_VOLTAGE; i++) {
		bluewhale_read_byte(charger, i, &data);
		printk(KERN_NOTICE "[0x%02x]=0x%02x ", i, data);        
	}

    	for (i=REG_CHARGE_CURRENT; i<=REG_VBUS_LIMIT_CURRENT; i++) {
		bluewhale_read_byte(charger, i, &data);
		printk(KERN_NOTICE "[0x%02x]=0x%02x ", i, data);        
	}

	bluewhale_read_byte(charger, REG_SAFETY_TIMER, &data);
	printk(KERN_NOTICE "[0x%02x]=0x%02x ", REG_SAFETY_TIMER, data);        

	bluewhale_read_byte(charger, REG_CHARGER_CONTROL, &data);
	printk(KERN_NOTICE "[0x%02x]=0x%02x ", REG_CHARGER_CONTROL, data);        

    	for (i=REG_VBUS_STATUS; i<=REG_THM_STATUS; i++) {
		bluewhale_read_byte(charger, i, &data);
		printk(KERN_NOTICE "[0x%02x]=0x%02x ", i, data);        
    	}
    
	printk(KERN_NOTICE "\n");
}

static int32_t bluewhale_get_charging_status(struct bluewhale_charger_info *charger)
{
	int32_t ret = 0;

	if (!charger)
		return -EINVAL;

	ret = bluewhale_check_vbus_status(charger);
	if(ret < 0)
		goto end;

	ret = bluewhale_check_charger_status(charger);
	if(ret < 0)
		goto end;

	ret = bluewhale_get_setting_data(charger);
	if(ret < 0)
		goto end;

	print_all_register(charger);
	print_all_charger_info(charger);

end:
	return ret;
}








int32_t bluewhale_set_chg_volt_extern(int32_t chgvolt_mv)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);

	if (charger_info) {
		ret = bluewhale_set_chg_volt(charger_info, chgvolt_mv);
		ret = bluewhale_set_t34_cv(charger_info, chgvolt_mv);
		ret = bluewhale_set_t45_cv(charger_info, chgvolt_mv);
	} else {
		ret = -EINVAL;
	}

	mutex_unlock(&bluewhale_mutex);
	return ret;
}


int32_t bluewhale_set_eoc_current_extern(int32_t eoc_ma)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);
	if (charger_info)
		ret = bluewhale_set_eoc_current(charger_info, eoc_ma);
	else
		ret = -EINVAL;
	mutex_unlock(&bluewhale_mutex);
	return ret;
}






int32_t bluewhale_get_vbus_current_extern(void)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);
	if (charger_info)
		ret = bluewhale_get_vbus_current(charger_info);
	else
		ret = -EINVAL;
	mutex_unlock(&bluewhale_mutex);
	return ret;
}



int32_t bluewhale_get_charger_current_extern(void)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);
	if (charger_info)
		ret = bluewhale_get_charger_current(charger_info);
	else
		ret = -EINVAL;
	mutex_unlock(&bluewhale_mutex);
	return ret;
}


int32_t bluewhale_set_charger_current_extern(int32_t chg_ma)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);
	if (charger_info)
		ret = bluewhale_set_charger_current(charger_info, chg_ma);
	else
		ret = -EINVAL;
	mutex_unlock(&bluewhale_mutex);
	return ret;
}


int32_t bluewhale_get_charging_status_extern(void)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);
	if (charger_info)
		ret = bluewhale_get_charging_status(charger_info);
	else
		ret = -EINVAL;
	mutex_unlock(&bluewhale_mutex);
	
	if (ret < 0)
		return -1;
	
	return ret;
}

int32_t bluewhale_dump_register_extern(void)
{
	int32_t ret=0 ;
	
	mutex_lock(&bluewhale_mutex);

	if(charger_info)
		print_all_register(charger_info);
	else
		ret = -EINVAL ;
 
	mutex_unlock(&bluewhale_mutex);

	return ret;
	
}

static int32_t bluewhale_get_chargingfull_status(struct bluewhale_charger_info *charger)
{
	u8 data_status ;
	int32_t ret ;
 
	/* Check charger status */
	ret = bluewhale_read_byte(charger, REG_CHARGER_STATUS, &data_status);
	if (ret < 0) {
		printk(KERN_ERR "%s: fail to get Charger status\n", __func__);
		return ret; 
	}

	ret = (data_status & CHARGE_FULL_STATE) ? 1 : 0 ;

	return ret ;
}

int32_t bluewhale_get_chargingfull_status_extern(void)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);

	if (charger_info)
		ret = bluewhale_get_chargingfull_status(charger_info) ;
	else
		ret =-EINVAL;

	mutex_unlock(&bluewhale_mutex);

	return ret;
}



#if O2_CONFIG_MTK_PEP_SUPPORT

#define VBUS_LOW 100
#define VBUS_HIGH 700


static void logic_low(struct bluewhale_charger_info *charger)
{
	bluewhale_set_vbus_current(charger, VBUS_LOW);
}

static void logic_high(struct bluewhale_charger_info *charger)
{
	bluewhale_set_vbus_current(charger, VBUS_HIGH);
}

static void o2_pep_increase(struct bluewhale_charger_info *charger)
{
	logic_low(charger);
	msleep(150);

	logic_high(charger);
	msleep(100);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(100);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(300);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(300);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(300);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(500);
	
	logic_low(charger);
	msleep(100);

	bluewhale_set_vbus_current(charger, VBUS_HIGH);
}

static void o2_pep_decrease(struct bluewhale_charger_info *charger)
{
	logic_low(charger);
	msleep(150);

	logic_high(charger);
	msleep(300);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(300);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(300);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(100);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(100);
	logic_low(charger);
	msleep(100);
	logic_high(charger);
	msleep(500);

	logic_low(charger);
	msleep(50);

	bluewhale_set_vbus_current(charger, VBUS_HIGH);
}

static void o2_pep_reset(struct bluewhale_charger_info *charger)
{
	logic_low(charger);
	msleep(250);
	logic_high(charger);

}

int32_t bluewhale_PEP_increase_extern(int32_t increase)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);

	if(charger_info) {
		if(increase) {
			o2_pep_increase(charger_info);
		} else {
			o2_pep_decrease(charger_info);
		}
	} else {
		ret = -EINVAL ;
	}

	mutex_unlock(&bluewhale_mutex);
	return ret ;
}

int32_t bluewhale_PEP_reset_extern(void)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);

	if(charger_info) {
		o2_pep_reset(charger_info) ;
	} else {
		ret = -EINVAL;
	}

	mutex_unlock(&bluewhale_mutex);
	return ret ;
}

#endif //O2_CONFIG_MTK_PEP_SUPPORT





static int32_t bluewhale_enable_otg(struct bluewhale_charger_info *charger, int32_t enable)
{
	uint8_t data;
	uint8_t mask = 0;

    	data = (enable == 0) ? 0 : 1;

	if (data) //enable otg, disable suspend
		mask = 0x06;
	else //disable otg
		mask = 0x04;

	return bluewhale_update_bits(charger, REG_CHARGER_CONTROL, mask, data << 2);
}


int32_t bluewhale_enable_otg_extern(int32_t enable)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);
	if (charger_info)
		ret = bluewhale_enable_otg(charger_info, enable);
	else
		ret = -EINVAL;
	mutex_unlock(&bluewhale_mutex);
	return ret;
}


int32_t bluewhale_init(struct bluewhale_charger_info *charger)
{
  	printk(KERN_NOTICE "O2Micro bluewhale hardware init\n");

	if (!charger) return -EINVAL;
	
	// write rthm  100k/10k
	bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x1, CHARGER_RTHM);

	//set chager PWM TON time
	bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x18, 0x0 << 3);
	
	//disable SOP
	bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x80, 0x0 << 7);

	/* Min VSYS:3.6V */
    	bluewhale_set_min_vsys(charger, CHGR_DEF_VSYS);	
	bluewhale_set_eoc_current(charger, CHGR_DEF_EOC);

	bluewhale_set_wakeup_volt(charger, CHGR_DEF_WK_VOL) ;
	bluewhale_set_wakeup_current(charger, CHGR_DEF_WK_CUR) ;
	
	bluewhale_set_safety_wk_timer(charger, CHGR_DEF_WK_TIMER) ;
	bluewhale_set_safety_cc_timer(charger, CHGR_DEF_CC_TIMER);

	bluewhale_set_rechg_hystersis(charger, CHGR_DEF_RECHG_VOL);

	return 0;
}

static int32_t bluewhale_init_default(struct bluewhale_charger_info *charger)
{
  	printk(KERN_NOTICE "O2Micro bluewhale hardware default init\n");

	if (!charger) return -EINVAL;
	
	// write rthm  100k/10k
	bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x1, CHARGER_RTHM);

	//set chager PWM TON time
	bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x18, 0x00 << 3);
	
	//disable SOP
	bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x80, 0x0 << 7);

	//disable OTG
	bluewhale_enable_otg(charger, 0);
	
	/* Min VSYS:3.6V */
    	bluewhale_set_min_vsys(charger, CHGR_DEF_VSYS);	
	
	//CV voltage
	bluewhale_set_chg_volt_intern(charger, CHGR_DEF_CV);
	bluewhale_set_t34_cv(charger, CHGR_DEF_CV_T34);
	bluewhale_set_t45_cv(charger, CHGR_DEF_CV_T45);

	// useless,we will make this later
	//set ilmit
	bluewhale_set_vbus_current(charger, CHGR_DEF_ILIMIT);
	
	//set charging current, CC
	bluewhale_set_charger_current(charger, CHGR_DEF_CC);

	bluewhale_set_eoc_current(charger, CHGR_DEF_EOC);

	bluewhale_set_wakeup_volt(charger, CHGR_DEF_WK_VOL) ;
	bluewhale_set_wakeup_current(charger, CHGR_DEF_WK_CUR) ;
	
	bluewhale_set_safety_wk_timer(charger, CHGR_DEF_WK_TIMER) ;
	bluewhale_set_safety_cc_timer(charger, CHGR_DEF_CC_TIMER);

	bluewhale_set_rechg_hystersis(charger, CHGR_DEF_RECHG_VOL);

	return 0;
}

int32_t bluewhale_init_extern(void)
{
	int ret = 0;
	mutex_lock(&bluewhale_mutex);
	if (charger_info)
		ret = bluewhale_init(charger_info);
	else
		ret = -EINVAL;
	mutex_unlock(&bluewhale_mutex);
	return ret;
}


/*
 * oz1c115c suspend enable/disable
 */
static int32_t bluewhale_chip_suspend(struct bluewhale_charger_info *charger, int32_t enable)
{	
	int32_t ret = 0 ;
	if(enable)
		ret = bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x02, 1 << 1);
	else 
		ret = bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x02, 0 << 1);

	return ret ;
}

int32_t bluewhale_chip_suspend_extern(int32_t enable)
{
	int32_t ret = 0;

	mutex_lock(&bluewhale_mutex);
	
	if (charger_info)
		ret = bluewhale_chip_suspend(charger_info, enable) ;
	else
		ret = -EINVAL;
	
	mutex_unlock(&bluewhale_mutex);
	
	return ret;
}





/**
 * Section: SOFTWARE DPM ALGORITHM
 */

#define O2_CONFIG_SOFTWARE_DPM_SUPPORT 	1

#if O2_CONFIG_SOFTWARE_DPM_SUPPORT

// SDPM internal optimizing mechanism
#define O2_CONFIG_LINEAR_COMP_SUPPORT		1
#define O2_CONFIG_ILIMIT_BACKOFF_TWOSTEP	1
#define O2_CONFIG_ILIMIT_NTAG_SUPPORT		1
#define O2_CONFIG_ILIMIT_YTAG_SUPPORT		1


// SDPM default parameter setting
#define ADJI_SDPM_ILIMIT_THRESHOLD 	(700)
#define ADJI_SDPM_ILIMIT_HIGH 		(1700)

#define ADJI_ILELS_NUM 			(10)
#define ADJI_CHRENV_RTNUM 		(2)

#define ADJI_SOP_CHECK_TIMES_MAX	(6)
#define ADJI_SOP_CHECK_WAIT_TIME	(10)

#define ADJI_JUDG_VBUSVOLT_LOWTH 	(4400)
#define ADJI_JUDG_SOPDISFUNC_IBATTH 	(-300)
#define ADJI_JUDG_IENOUGH_IBATTH 	(50)
#define ADJI_JUDG_AD_OSC_IBATTH		(400)
#define ADJI_JUDG_AD_OSC_VBUSTH		(300)


#if O2_CONFIG_OZ1C105G_SUPPORT
#define ADJI_AVG_INFO_ARRAY_SIZE	(1)
#define ADJI_AVG_INFO_WAIT_TIME		(1000)
#define ADJI_AVG_INFO_CHECK_INTERVAL	(50)
#define ADJI_AVG_INFO_FIRST_DELAY	(300)
#else
#define ADJI_AVG_INFO_ARRAY_SIZE	(10)
#define ADJI_AVG_INFO_WAIT_TIME		(400)
#define ADJI_AVG_INFO_CHECK_INTERVAL	(20)
#define ADJI_AVG_INFO_FIRST_DELAY	(100)
#endif


#define ADJI_SYSENV_CHANGE_IBATTH	(200)
#define ADJI_SYSENV_CHANGE_VBUSTH	(100)

#define ADJI_SYSENV_BETTER_IBATTH_IL	(200)
#define ADJI_SYSENV_BETTER_IBATTH_IM	(300)
#define ADJI_SYSENV_BETTER_IBATTH_IH	(500)

#define BAT_RES 0

enum {
	ADJI_RET_NOISE_OR_APOWERLOW,
	ADJI_RET_VBUSLOW,
	ADJI_RET_IENOUGH,
	ADJI_RET_SOPDISFUNC,
	ADJI_RET_HIGHEST
} adji_ret ;

enum {
	ADJI_CHECK_ILIMIT_RET_OK,
	ADJI_CHECK_ILIMIT_RET_SOP,
	ADJI_CHECK_ILIMIT_RET_LOWVOL,
	ADJI_CHECK_ILIMIT_RET_LOWCUR,
	ADJI_CHECK_ILIMIT_RET_EVB,
	ADJI_CHECK_ILIMIT_RET_EVW,
	ADJI_CHECK_ILIMIT_RET_HIGHVOL,
	ADJI_CHECK_ILIMIT_RET_ADDISFUNC
} adji_check_ilimit_ret ;

struct charger_sysenv {
	int32_t vbus_volt ;
	int32_t	vbat ;
	int32_t ibat ;
} ;

static struct charger_sysenv adji_chrenv_momt  ;
static struct charger_sysenv adji_chrenv_adjmomt  ;
static struct charger_sysenv adji_chrenv_rt[ADJI_CHRENV_RTNUM] ;
static uint8_t adji_chrenv_rtpos = 0 ;
static uint8_t adji_chrenv_rtpos_rewind = 0 ;

static int32_t adji_ilel_array[ADJI_ILELS_NUM] = {500,700,900,1000,1200,1400,1500,1700,1900,2000} ;
static int32_t adji_bat_curr[ADJI_ILELS_NUM] = {0,0,0,0,0,0,0,0,0,0} ;

#if O2_CONFIG_ILIMIT_NTAG_SUPPORT
#define ADJI_NTAG_CNT_NUM	(5)
static int32_t adji_ilel_ntag[ADJI_ILELS_NUM] = {0,0,0,0,0,0,0,0,0,0} ;
static int32_t adji_ilel_ntag_judge_cnt[ADJI_ILELS_NUM] = {0,0,0,0,0,0,0,0,0,0} ;

#endif

#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
#define ADJI_YTAG_CNT_NUM	(5)
#define ADJI_YTAG_IBATTH	(200)
#define ADJI_YTAG_VBATTH	(80)
#define ADJI_YTAG_VBUSTH	(ADJI_JUDG_VBUSVOLT_LOWTH)
static int32_t adji_ilel_ytag[ADJI_ILELS_NUM] = {0,0,0,0,0,0,0,0,0,0} ;
static uint8_t adji_ytag_cnt = 0 ;
#endif

static uint8_t adji_backoff_step_cnt = 0 ;

static uint8_t adji_started = 0 ;



// SDPM Debugging parameter
static int32_t o2_debug_sdpm_ilimit_threshold = ADJI_SDPM_ILIMIT_THRESHOLD  ;
static int32_t o2_debug_sdpm_ilimit_high = ADJI_SDPM_ILIMIT_HIGH ;
static int32_t o2_debug_sdpm_sop_check_times = ADJI_SOP_CHECK_TIMES_MAX ; 
static int32_t o2_debug_sdpm_judg_ienough_ibatth = ADJI_JUDG_IENOUGH_IBATTH ; 
static int32_t o2_debug_sdpm_judg_sopdisfunc_ibatth = ADJI_JUDG_SOPDISFUNC_IBATTH ; 
static int32_t o2_debug_sdpm_judg_adosc_ibatth = ADJI_JUDG_AD_OSC_IBATTH ; 
static int32_t o2_debug_sdpm_judg_adosc_vbusth = ADJI_JUDG_AD_OSC_VBUSTH ; 
static int32_t o2_debug_sdpm_sop_check_wait_time = ADJI_SOP_CHECK_WAIT_TIME ; 
static int32_t o2_debug_sdpm_sysenv_change_ibatth = ADJI_SYSENV_CHANGE_IBATTH ; 
static int32_t o2_debug_sdpm_sysenv_change_vbusth = ADJI_SYSENV_CHANGE_VBUSTH ; 
static int32_t o2_debug_sdpm_sysenv_better_ibatth_il = ADJI_SYSENV_BETTER_IBATTH_IL ; 
static int32_t o2_debug_sdpm_sysenv_better_ibatth_im = ADJI_SYSENV_BETTER_IBATTH_IM ; 
static int32_t o2_debug_sdpm_sysenv_better_ibatth_ih = ADJI_SYSENV_BETTER_IBATTH_IH ; 
static int32_t o2_debug_sdpm_avg_info_check_interval = ADJI_AVG_INFO_CHECK_INTERVAL ; 
static int32_t o2_debug_sdpm_avg_info_first_delay = ADJI_AVG_INFO_FIRST_DELAY ; 
static int32_t o2_debug_sdpm_avg_info_wait_time = ADJI_AVG_INFO_WAIT_TIME ; 


#if O2_CONFIG_LINEAR_COMP_SUPPORT
static int32_t o2_linear_comp_support = 0 ;
#endif

#if O2_CONFIG_ILIMIT_BACKOFF_TWOSTEP
static int32_t o2_backoff_twostep_support = 1;
#endif

#if O2_CONFIG_ILIMIT_NTAG_SUPPORT
static int32_t o2_ntag_support = 1 ;
static int32_t o2_ntag_debug_cnt_num = ADJI_NTAG_CNT_NUM ;
#endif

#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
static int32_t o2_ytag_support = 1 ;
static int32_t o2_ytag_debug_cnt_num = ADJI_YTAG_CNT_NUM ;
static int32_t o2_ytag_debug_ibatth = ADJI_YTAG_IBATTH ;
static int32_t o2_ytag_debug_vbatth = ADJI_YTAG_VBATTH ;
#endif







/*
 * Basic information needed for software dpm.
 * User should realize the following three functions for their specific environment.
 */

/*** platform-specific-api ***/

static int32_t adji_get_bat_volt(void)
{

#if O2_CONFIG_OZ1C105G_SUPPORT
	int32_t vbat = 0 ;
	int32_t ret = 0 ;

	ret = afe_read_cell_volt(&vbat);

	return vbat ;
#else


#if O2_CONFIG_MTK_PLATFORM_SUPPORT
	return battery_meter_get_battery_voltage(1) ;
#else
	return -1 ;

#endif	//O2_CONFIG_MTK_PLATFORM_SUPPORT



#endif	//O2_CONFIG_OZ1C105G_SUPPORT

}

static int32_t adji_get_bat_curr(void)
{
#if O2_CONFIG_OZ1C105G_SUPPORT
	int32_t cur = 0 ;
	int32_t ret = 0 ;
	ret = afe_read_current(&cur);	

	return cur ;
#else

#if O2_CONFIG_MTK_PLATFORM_SUPPORT
        int32_t cur = 0;
        int32_t val = 0;
        int32_t ret = 0;

        ret = bm_ctrl_cmd(BATTERY_METER_CMD_GET_HW_FG_CURRENT, &cur);
        cur = cur/ 10;

        ret = bm_ctrl_cmd(BATTERY_METER_CMD_GET_HW_FG_CURRENT_SIGN, &val);
        if (val)
                return cur;
        else
                return 0 - cur;
#else
	return -1 ;

#endif	//O2_CONFIG_MTK_PLATFORM_SUPPORT

#endif	//O2_CONFIG_OZ1C105G_SUPPORT
}

static int32_t adji_get_vbus_volt(void)
{

#if O2_CONFIG_MTK_PLATFORM_SUPPORT
	return battery_meter_get_charger_voltage() ;
#else
	return -1;
#endif

}

/*** platform-specific-api ***/

static void adji_bubble_sorting(int32_t *vstart, int32_t size)
{
	int32_t i;
	int32_t j;
	int32_t tmp;

	for(i=0; i<(size-1); i++) {
		for(j=i; j<(size-1); j++) {
			if(vstart[j] > vstart[j+1]) {
				tmp = vstart[j+1] ;
				vstart[j+1] = vstart[j];
				vstart[j] = tmp ;
			}
		}
	}
}

static int32_t adji_cal_avg_info(int32_t *vstart, int32_t size)
{
	if(size == 1)
		return vstart[0] ;

	if(size == 2)
		return ( (vstart[0] + vstart[1])/2 ) ;

	if(size >= 3) {
		int32_t i ;
		int32_t tmp=0 ;
	
		for(i=1; i<(size -1);  i++)
			tmp += vstart[i] ;

		return ( tmp/(size-2) ) ;
	}

	return 0 ;
}

static int32_t adji_get_avg_info(int32_t * ibat, int32_t * vbat, int32_t * vbus, int32_t mode)
{
	int32_t ibat_array[ADJI_AVG_INFO_ARRAY_SIZE] ;
	int32_t vbat_array[ADJI_AVG_INFO_ARRAY_SIZE] ;
	int32_t vbus_array[ADJI_AVG_INFO_ARRAY_SIZE] ;

	int32_t ibat_tmp ;
	int32_t vbat_tmp ;
	int32_t vbus_tmp ;
	int32_t i ;
	int32_t j ;

	unsigned long stamp_start ; 
	unsigned long stamp_current ;	
	long stamp_diff ;
	long msec_diff ;

	stamp_start = jiffies ;

	if(mode == 0) {
		ibat_tmp = adji_get_bat_curr() ;
		vbat_tmp = adji_get_bat_volt() ;
		vbus_tmp = adji_get_vbus_volt() ;
	
		//in mode 0, we delay some time
		//to wait for info to get stable	
		msleep(o2_debug_sdpm_avg_info_first_delay) ;
	}

	i = 0 ;

	do {
		msleep(o2_debug_sdpm_avg_info_check_interval) ;

		ibat_array[i] = adji_get_bat_curr() ;
		vbat_array[i] = adji_get_bat_volt() ;
		vbus_array[i] = adji_get_vbus_volt() ;
			
		if(i == 0) {

			if(mode == 0) {
				//make sure we get the new value
				if( (ibat_array[i] != ibat_tmp) && (vbus_array[i] != vbus_tmp) ) {
					i++ ;
				}	

			} else {
				i++ ;
			}

		} else {
			i++ ;
		}

		stamp_current = jiffies ;
		stamp_diff = (long)stamp_current - (long)stamp_start ;
		msec_diff = stamp_diff * 1000 /HZ ;
	} while ( (i<ADJI_AVG_INFO_ARRAY_SIZE) && (msec_diff <= o2_debug_sdpm_avg_info_wait_time) ) ;

	if (i == 0)
		i = 1;

	adji_bubble_sorting(&(ibat_array[0]), i);	
	adji_bubble_sorting(&(vbat_array[0]), i);	
	adji_bubble_sorting(&(vbus_array[0]), i);	

	*ibat = adji_cal_avg_info(&(ibat_array[0]), i);
	*vbat = adji_cal_avg_info(&(vbat_array[0]), i);
	*vbus = adji_cal_avg_info(&(vbus_array[0]), i);
	
	//print all vol/curr infomation
	ilimit_dbg("[%s] all ibat,vbat,vbus (total num: %d) : ", __FUNCTION__, i);
	for(j=0; j<i; j++) {
		printk("%d	%d	%d\n", ibat_array[j], vbat_array[j], vbus_array[j] );
	}
	
	ilimit_dbg("[%s] average ibat:%d,vbat:%d,vbus:%d ", __FUNCTION__, *ibat, *vbat, *vbus);

	return 0 ;
}



#if O2_CONFIG_ILIMIT_NTAG_SUPPORT

static void adji_ntag_clear_all(void)
{
	int32_t i ;

	for(i=0; i<ADJI_ILELS_NUM; i++) {
		adji_ilel_ntag[i] = 0 ;		
		adji_ilel_ntag_judge_cnt[i] = 0 ;
	}
}

static int32_t adji_ntag_get_max_ilimit(void)
{
	int32_t i ;

	//from low to high
	for(i=0; i<ADJI_ILELS_NUM; i++) {
		if(adji_ilel_ntag[i] != 0)
			break;
	}

	if(i>0)	i-- ;

	return adji_ilel_array[i] ;
}

static void adji_ntag_set_ilimit(int32_t ilimit)
{
	int32_t i ;
	
	for(i=0; i<ADJI_ILELS_NUM; i++) {
		if(ilimit <= adji_ilel_array[i])
			break;
	}

	if(i < ADJI_ILELS_NUM)
		adji_ilel_ntag[i] = 1;

}

#endif	//O2_CONFIG_ILIMIT_NTAG_SUPPORT


#if O2_CONFIG_ILIMIT_YTAG_SUPPORT

static void adji_ytag_clear_all(void)
{
	int32_t i ;

	for(i=0; i<ADJI_ILELS_NUM; i++) {
		adji_ilel_ytag[i] = 0 ;		
	}

	adji_ytag_cnt = 0 ;
}

static void adji_ytag_clear_upon(int32_t ilimit)
{
	int32_t i ;
	
	for(i=(ADJI_ILELS_NUM-1); i<0; i--) {
		if(ilimit < adji_ilel_array[i])
			adji_ilel_ytag[i] = 0 ;
	}

}

static int32_t adji_ytag_get_min_ilimit(void)
{
	int32_t i ;

	//from high to low
	for(i=(ADJI_ILELS_NUM-1); i<0; i--) {
		if(adji_ilel_ytag[i] != 0)
			break;
	}

	if(i<0) 
		i = 0 ;

	return adji_ilel_array[i] ;
}

static void adji_ytag_set_ilimit(int32_t ilimit)
{
	int32_t i ;
	
	for(i=0; i<ADJI_ILELS_NUM; i++) {
		if(ilimit <= adji_ilel_array[i])
			break;
	}

	if(i < ADJI_ILELS_NUM)
		adji_ilel_ytag[i] = 1;

}


#endif	//O2_CONFIG_ILIMIT_YTAG_SUPPORT




/* Note:
 * This function is very important for SDMP
 * When combined with DYNAMIC_CV, you need to 
 * be very careful since CV-setting is very close
 * to the real battery voltage.
 */
static int32_t adji_check_ilimit(int32_t ilimit, int32_t cval, int32_t vbus_volt, int32_t vbat, int32_t ibat)
{
	int32_t tmp ;
	int32_t ret ;

	/*
	 * Absolute ilimit-adjusting condition
	 * Need to adjust ilimit from 500 to 2000.
	 */
	
	//ilimit-500 is found
	if(ilimit == 500) {
		ret =  ADJI_CHECK_ILIMIT_RET_SOP ;
		ilimit_dbg("[%s] ilimit-level (%d) check aci_i500_fail.", __FUNCTION__, ilimit);
		goto aci_i500_fail ;
	}






	/*
	 * Absolute ilimit-readjusting condition.
	 * These conditions indicate some unstable or unsafe states.
	 * We decide to readjust ilimit if these conditions happen for conservative considering.
	 * Note: means that we should lower ilimit level (down-adjusting).
	 */

	//vbus voltage too low.
	if(vbus_volt < ADJI_JUDG_VBUSVOLT_LOWTH) {
		ret =  ADJI_CHECK_ILIMIT_RET_LOWVOL ;
		ilimit_dbg("[%s] ilimit-level (%d) check aci_absolute_fail: vbus_volt is %d.", 
				__FUNCTION__, ilimit, vbus_volt);
		goto aci_absolute_fail ; 
	}


	//possibly, adaptor's oscilating happens when ilimit is higher than 700
	//adaptor's oscilating condition is very strict
	//this determining condition is trying to catch the behavior of adaptor's disfunction
	if(ilimit > 700) {
		if( ibat < (adji_chrenv_momt.ibat - o2_debug_sdpm_judg_adosc_ibatth) && 
			vbus_volt > (adji_chrenv_momt.vbus_volt + o2_debug_sdpm_judg_adosc_vbusth) ) {

#if O2_CONFIG_ILIMIT_NTAG_SUPPORT
if(o2_ntag_support) 
{

			adji_ntag_set_ilimit(ilimit) ;
			ilimit_dbg("[%s] [NTAG] ilimit-level (%d) oscilating, ntag this level.", __FUNCTION__, ilimit);

}
#endif
			
			ret =  ADJI_CHECK_ILIMIT_RET_ADDISFUNC;
			ilimit_dbg("[%s] ilimit-level (%d) check aci_absolute_fail: oscilating.", 
					__FUNCTION__, ilimit);
			goto aci_absolute_fail ; 

		}
	}


	//battery current too small.
	if(vbat < cval - 80) {
		if(ilimit >= 1400) {

			if(ibat < 200) {
				ret = ADJI_CHECK_ILIMIT_RET_LOWCUR ;
				ilimit_dbg("[%s] ilimit-level (%d) check aci_absolute_fail: ibat is %d.", 
						__FUNCTION__, ilimit, ibat);
				goto aci_absolute_fail ; 
			}

		} else if (ilimit >= 900) {

			if(ibat < 50) {
				ret = ADJI_CHECK_ILIMIT_RET_LOWCUR ;
				ilimit_dbg("[%s] ilimit-level (%d) check aci_absolute_fail: ibat is %d.", 
						__FUNCTION__, ilimit, ibat);
				goto aci_absolute_fail ; 

			}

		}
	}



#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
if(o2_ytag_support)
{

	/*
	 * To determine some specific ilimit level is 
	 * available for this specific charging-behavior,
	 * We need a very strict juding condition (to make sure that
	 * once the level is y-tagged, it is guaranteed to be available)
	 * or the result will be disastrous.
	 * Note: This is a dangerous mechanism!
	 */

	if( ilimit < adji_ytag_get_min_ilimit() ) {
		if( (adji_ret == ADJI_RET_VBUSLOW) ||
			(adji_ret == ADJI_RET_SOPDISFUNC) ||
				(adji_ret == ADJI_RET_NOISE_OR_APOWERLOW) ) {
			adji_ytag_clear_upon(ilimit);
			ilimit_dbg("[%s] [YTAG] ilimit-level (%d), clear above levels.", __FUNCTION__, ilimit);

		}
			
	}
	
	if( (vbat < (cval - o2_ytag_debug_vbatth)) && (vbus_volt > ADJI_YTAG_VBUSTH) ) {
	
		//assume: ilimit lower 100, system comsumes 600, 
		//when ilimit is higher, make it harder to achieve ytag-setting
		//this makes sense.
		if(ibat > (ilimit - 100 - 600 + o2_ytag_debug_ibatth) ) {

			if(adji_ytag_cnt < o2_ytag_debug_cnt_num) {
				adji_ytag_cnt++ ;
				ilimit_dbg("[%s] [YTAG] ilimit-level (%d) keep stable, count: %d.", __FUNCTION__, ilimit, adji_ytag_cnt);
			}

			if(adji_ytag_cnt == o2_ytag_debug_cnt_num) {
				adji_ytag_set_ilimit(ilimit) ;
				ilimit_dbg("[%s] [YTAG] ilimit-level (%d) is available, reach count: %d.", __FUNCTION__, ilimit, adji_ytag_cnt);
			}

		} else {
			adji_ytag_cnt = 0 ;
		} 	

	} else {
		adji_ytag_cnt = 0 ;
	}

}
#endif




	/* 
	 * When ilimit has been high enough, DO NOT DO the optimization 
	 * to aviod unnecessary readjusting. Because DPM-optimizing may 
	 * be too risky and damage the algorithm's stability-level, we do 
	 * not want that trouble if the ilimit is relatively high.
	 * Just Let It Go!
	 */

	if(ilimit >= o2_debug_sdpm_ilimit_high) {
		ret =  ADJI_CHECK_ILIMIT_RET_OK;
		goto aci_succeed ; 
	}






	
	/*
	 * DPM-optimizing ilimit-readjusting condition.
	 * These conditions indicate some states which 
	 * ilimit could be raised to optimize charging efficiecy.
	 * Note: Means that we could raise ilimit level (up-adjusting).
	 * Note: These optimizing-operation may be risky and very useful at the mean time.
	 */

	//the big condition which up-readjusting is allowed
	if ( vbus_volt > (ADJI_JUDG_VBUSVOLT_LOWTH + 100) ) {
		
		switch(adji_ret) {
			case ADJI_RET_NOISE_OR_APOWERLOW:

				//system's noise or real SOP
				/*if (adji_backoff_step_cnt == 1) {
					
					if (adji_chrenv_adjmomt.vbus_volt > 4650) {
						if(vbus_volt > 4550) {
							ret = ADJI_CHECK_ILIMIT_RET_HIGHVOL ;
							goto aci_relative_fail ; 
						}
					}

				} else if (adji_backoff_step_cnt == 2) {

					if (adji_chrenv_adjmomt.vbus_volt > 4600) {
						if(vbus_volt > 4650) {
							ret =  ADJI_CHECK_ILIMIT_RET_HIGHVOL ;
							goto aci_relative_fail ; 
						}

					}
				}

				break ;*/

				tmp = adji_chrenv_momt.vbus_volt ;

				if(BW_ABS(vbus_volt,tmp) < 100) {

					//when system consume less &&  more AD-power is need
					//note: the charger's system environment is better

					if(ilimit < 1200)	//700-900-1000
						tmp = o2_debug_sdpm_sysenv_better_ibatth_il ;

					else if(ilimit < 1700) 	//1200-1400-1500
						tmp = o2_debug_sdpm_sysenv_better_ibatth_im ;

					else if(ilimit < 2000) 	//1700-1900-2000
						tmp = o2_debug_sdpm_sysenv_better_ibatth_ih ;

					if (ibat > (adji_chrenv_momt.ibat + tmp) ) {
						ret = ADJI_CHECK_ILIMIT_RET_EVB;
						ilimit_dbg("[%s] ilimit-level (%d) check aci_relative_fail: ibat is %d %d %d.", 
								__FUNCTION__, ilimit, ibat, adji_chrenv_momt.ibat, tmp);
						goto aci_relative_fail ; 
					}

				}

				break ;

			case ADJI_RET_IENOUGH:
				tmp = adji_chrenv_momt.vbus_volt ;

				if(BW_ABS(vbus_volt,tmp) < o2_debug_sdpm_sysenv_change_vbusth) {

					//when system consume less &&  more AD-power is need
					//note: the charger's system environment is better
					if (ibat > (adji_chrenv_momt.ibat + o2_debug_sdpm_sysenv_change_ibatth) ) {
						ret = ADJI_CHECK_ILIMIT_RET_EVB;
						ilimit_dbg("[%s] ilimit-level (%d) check aci_relative_fail: ibat is %d %d.", 
								__FUNCTION__, ilimit, ibat, adji_chrenv_momt.ibat);
						goto aci_relative_fail ; 
					}

					//when system consume more && more AD-power is need
					//note: the charger's system environment is worse
					if (ibat < (adji_chrenv_momt.ibat - o2_debug_sdpm_sysenv_change_ibatth) ) {
						ret = ADJI_CHECK_ILIMIT_RET_EVW;
						ilimit_dbg("[%s] ilimit-level (%d) check aci_relative_fail: ibat is %d %d.", 
								__FUNCTION__, ilimit, ibat, adji_chrenv_momt.ibat);
						goto aci_relative_fail ; 
					}

				}

				break ;

			case ADJI_RET_VBUSLOW:

				//I don't know why I add this. Just think maybe it will help.
				//Because when this happens, it definitely means something wrong.
				if(vbus_volt > (adji_chrenv_momt.vbus_volt + 300)) {
					ret = ADJI_CHECK_ILIMIT_RET_HIGHVOL ;
					ilimit_dbg("[%s] ilimit-level (%d) check aci_relative_fail: vbus_volt is %d %d.", 
							__FUNCTION__, ilimit, vbus_volt, adji_chrenv_momt.vbus_volt);
					goto aci_relative_fail ; 
				}
				break ;

			case ADJI_RET_SOPDISFUNC:
			case ADJI_RET_HIGHEST:
				//do not try in these cases
				//do not do any thing
				break;

			default:
				break;
		}

	}

	ret = ADJI_CHECK_ILIMIT_RET_OK;

aci_succeed:
	return ret ;

aci_i500_fail:
	return ret ;

aci_absolute_fail:

#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
if(o2_ytag_support)
{

	if( ilimit < adji_ytag_get_min_ilimit() ) {
		if( (adji_ret == ADJI_RET_VBUSLOW) ||
			(adji_ret == ADJI_RET_SOPDISFUNC) ||
				(adji_ret == ADJI_RET_NOISE_OR_APOWERLOW) ) {
			adji_ytag_clear_upon(ilimit);
			ilimit_dbg("[%s] [YTAG] ilimit-level (%d), clear above levels.", __FUNCTION__, ilimit);
		}
	}

}
#endif
	return ret ;

aci_relative_fail:
	return ret ;


		
}


static void adji_chrenv_reset_all(void)
{
	uint8_t i ;

	adji_chrenv_momt.vbus_volt = 0 ;
	adji_chrenv_momt.vbat = 0 ;
	adji_chrenv_momt.ibat = 0 ;

	adji_chrenv_adjmomt.vbus_volt = 0 ;
	adji_chrenv_adjmomt.vbat = 0 ;
	adji_chrenv_adjmomt.ibat = 0 ;

	for(i=0; i< ADJI_CHRENV_RTNUM; i++) {
		adji_chrenv_rt[i].vbus_volt = 0;
		adji_chrenv_rt[i].vbat = 0;
		adji_chrenv_rt[i].ibat = 0;
	}

	adji_chrenv_rtpos = 0;
	adji_chrenv_rtpos_rewind = 0;
		
}

static void adji_chrenv_update_momt(int32_t vbus, int32_t vbat, int32_t ibat)
{
	adji_chrenv_momt.vbus_volt = vbus ;
	adji_chrenv_momt.vbat = vbat ;
	adji_chrenv_momt.ibat = ibat ;
}

static void adji_chrenv_update_adjmomt(int32_t vbus, int32_t vbat, int32_t ibat)
{
	adji_chrenv_adjmomt.vbus_volt = vbus ;
	adji_chrenv_adjmomt.vbat = vbat ;
	adji_chrenv_adjmomt.ibat = ibat ;
}

#if 0
//reserved for future improvement

//not used yet
static void adji_chrenv_update(int32_t vbus, int32_t vbat, int32_t ibat)
{
	adji_chrenv_rt[adji_chrenv_rtpos].vbus_volt = vbus ;
	adji_chrenv_rt[adji_chrenv_rtpos].vbat = vbat ;
	adji_chrenv_rt[adji_chrenv_rtpos].ibat = ibat ;

	adji_chrenv_rtpos ++ ;
 
	if(adji_chrenv_rtpos == ADJI_CHRENV_RTNUM) {
		adji_chrenv_rtpos = 0 ;
		adji_chrenv_rtpos_rewind = 1 ;
	}
}


//not used yet
static int32_t adji_chrenv_get_avg_ibat(void)
{
	int32_t ret=0 ;
	int32_t i ;	

	if(adji_chrenv_rtpos_rewind) {
		for(i=0; i<ADJI_CHRENV_RTNUM; i++)
			ret += adji_chrenv_rt[i].ibat ;	

		ret /= ADJI_CHRENV_RTNUM ;
	} else {
		for(i=0; i<adji_chrenv_rtpos; i++)
			ret += adji_chrenv_rt[i].ibat ;

		if(adji_chrenv_rtpos != 0)
			ret /= adji_chrenv_rtpos ;
	}

	return ret ;
}

//not used yet
static int32_t adji_chrenv_get_avg_vbat(void)
{
	return 0 ;
}

//not used yet
static int32_t adji_chrenv_get_avg_vbusvolt(void)
{
	return 0 ;
}

#endif






/*
 * adji_try_raise_ilimit -- try to raise ilimit to @ilimit
 * @ptr to charger struct
 * @ilimit: the targeted ilimit value
 * @retval: -1 if failed, 0 if succeed
 */
static int32_t adji_try_raise_ilimit(struct bluewhale_charger_info *charger, int32_t ilimit)
{
	int32_t ret = 0;
	int32_t vbus_current = 0;

	ret = bluewhale_get_vbus_current(charger) ;
	if(ret < 0) {
		ret = -1;
		goto end;
	}

	vbus_current = ret ;
	if(vbus_current == ilimit) {
		ret = 0 ;
		goto end ;
	}



#if (O2_CONFIG_LINEAR_COMP_SUPPORT)
if(o2_linear_comp_support) 
{

	//set new ilimit before enable SOP
	ret =  bluewhale_set_vbus_current(charger, ilimit) ;
	ilimit_dbg("[%s] set vbus_current: %d", __FUNCTION__, ilimit);
	if(ret < 0) {
		printk(KERN_ERR "[%s]: fail to set vbus current, err %d\n", __FUNCTION__,ret);
		ret = -1;
		goto end;
	}

	//wait some time, then start SOP and check
	//this operation is for waiting linear-compenation-ad to be stable
	switch(ilimit) {
		case 700:
		case 900:
			msleep(20) ;
			break;
		case 1000:
		case 1200:
		case 1400:
		case 1500:
		case 1700:
		case 1900:
		case 2000:
			msleep(100) ;
			break;
		default:
			break;
	}


}
#endif


	
	//enable SOP
	ret = bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x80, 0x1 << 7);
	if(ret < 0) {
		ret = -1;
		goto end;
	}
	
	msleep(1) ;

#if (O2_CONFIG_LINEAR_COMP_SUPPORT)
if(o2_linear_comp_support == 0) 
#endif
{
	//set new ilimit
	ret =  bluewhale_set_vbus_current(charger, ilimit) ;
	ilimit_dbg("[%s] set vbus_current: %d", __FUNCTION__, ilimit);
	if(ret < 0) {
		printk(KERN_ERR "[%s]: fail to set vbus current, err %d\n", __FUNCTION__,ret);
		ret = -1;
		goto end;
	}


}

	//wait 10ms, then check SOP
	msleep(o2_debug_sdpm_sop_check_wait_time) ;

	//disable SOP
	ret = bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x80, 0x0 << 7);
	if(ret < 0) {
		ret = -1;
		goto end ;
	}

	//check new ilimit
	ret = bluewhale_get_vbus_current(charger);
	if(ret < 0) {
		printk(KERN_ERR "[%s]: fail to get vbus current, err %d\n", __FUNCTION__,ret);
		ret = -1;
		goto end;
	}

	vbus_current = ret ;
	ilimit_dbg("[%s] get vbus_current: %d", __FUNCTION__, vbus_current);

	if(vbus_current == ilimit) 
		return 0;
	else
		return -1;

end:
	//disable SOP, and then return
	bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x80, 0x0 << 7);
	return ret ;

}

/*
 * adji_adjust_ilimit_up -- raise ilimit up between @ilimit_min and @ilimit_max
 * @ptr to charger struct
 * @ilimit_min: the minimal value of ilimit for adjusting
 * @ilimit_max: the maximal value of ilimit for adjusting
 * @retval: -1 if adji_dpm is stopped, the successfully adjusted ilimit value otherwise
 */
static int32_t adji_adjust_ilimit_up(struct bluewhale_charger_info *charger, int32_t ilimit_min, int32_t ilimit_max)
{
	int32_t ibat = 0 ;
	int32_t vbat = 0 ;
	int32_t vbus_volt = 0 ;
	int32_t bat_cur_diff = 0 ;
	int32_t ilimit_diff = 0 ;
	int32_t temp_cc = 0;
	int32_t ret ;
	int8_t fail = 0 ;
	int8_t i = 0 ;
	int8_t j = 0 ;

#if O2_CONFIG_ILIMIT_BACKOFF_TWOSTEP
	int8_t backoff_2nd = 0 ;
#endif


#if O2_CONFIG_ILIMIT_NTAG_SUPPORT
	int32_t ilimit_ntag = 0 ;
#endif

#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
	int32_t ilimit_ytag = 0 ;
#endif


	if(ilimit_min <= 0)
		ilimit_min = 500 ;

	if(ilimit_max <= 0)
		ilimit_max = 2000 ;


#if O2_CONFIG_ILIMIT_NTAG_SUPPORT
if(o2_ntag_support) 
{

	ilimit_ntag = adji_ntag_get_max_ilimit();

	if(ilimit_max > ilimit_ntag)
		ilimit_max = ilimit_ntag ;

}
#endif

#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
if(o2_ytag_support)
{

	adji_ytag_cnt = 0 ;
	
	ilimit_ytag = adji_ytag_get_min_ilimit();

}
#endif

	//force cc to 2200 for ilimit-adjusting
	ret = bluewhale_get_charger_current(charger) ;
	if(ret < 0) {
		ilimit_dbg("[%s] get charger current error: %d", __FUNCTION__, ret);
		return -1 ;
	} else {
		temp_cc = ret ;
		if(temp_cc != 2200)
			bluewhale_set_charger_current(charger, 2200) ;
	}


	//
	for(i=0; i < (ADJI_ILELS_NUM); i++) {

		//skip the ilimit-level needless to adjust
		if( ilimit_min > adji_ilel_array[i] )
			continue;

		//stop at the ilimit-level whose value larger than ilimit_max
		if( ilimit_max < adji_ilel_array[i] )
			break;

		ilimit_dbg("[%s] try ilimit level %d(%d)", __FUNCTION__, adji_ilel_array[i], i);

		//adjust ilimit & check SOP
		//this step won't take much time
#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
if(o2_ytag_support)
{

		if(adji_ilel_array[i] <= ilimit_ytag) {
			j = 0;
			ret = bluewhale_set_vbus_current(charger, adji_ilel_array[i]);
		} else {
			for(j=0; j<o2_debug_sdpm_sop_check_times; j++) {
				ret = adji_try_raise_ilimit(charger, adji_ilel_array[i]) ;
			
				if(ret == 0)
					break;
			}
		}

}
else
#endif
{
		for(j=0; j<o2_debug_sdpm_sop_check_times; j++) {
			ret = adji_try_raise_ilimit(charger, adji_ilel_array[i]) ;
			
			if(ret == 0)
				break;
		}

}

		//SOP at ilimit-level i
		if(j==o2_debug_sdpm_sop_check_times) {
			ilimit_dbg("[%s] try ilimit level %d(%d) SOP-Checking Fail, try times %d", 
					__FUNCTION__, adji_ilel_array[i], i, j);

			adji_ret = ADJI_RET_NOISE_OR_APOWERLOW ;
			fail =1 ;
			break;
		}

		ilimit_dbg("[%s] try ilimit level %d(%d) SOP-Checking OK, try times %d", 
				__FUNCTION__, adji_ilel_array[i], i, (j+1) );

		//get average ibat/vbat/vbus, and then check
		adji_get_avg_info(&ibat, &vbat, &vbus_volt, 0) ;
			
		if(vbus_volt < ADJI_JUDG_VBUSVOLT_LOWTH) {
			ilimit_dbg("[%s] try ilimit level %d(%d) Vbus-Checking Fail",
					__FUNCTION__, adji_ilel_array[i], i);

			adji_ret = ADJI_RET_VBUSLOW ;
			fail = 1 ;
			break;
		}
		ilimit_dbg("[%s] try ilimit level %d(%d) Vbus-Checking OK",
				__FUNCTION__, adji_ilel_array[i], i);
		
		adji_bat_curr[i] = ibat ;

		if(i>0) {
			bat_cur_diff = adji_bat_curr[i] - adji_bat_curr[i-1] ;
			ilimit_diff = adji_ilel_array[i] - adji_ilel_array[i-1] ;

			ilimit_dbg("[%s] adji_bat_curr[%d](%d), adji_bat_curr[%d](%d)",
					__FUNCTION__, i, adji_bat_curr[i], i-1, adji_bat_curr[i-1]);

			ilimit_dbg("[%s] bat_cur_diff(%d), ilimit_diff(%d)",
					__FUNCTION__, bat_cur_diff, ilimit_diff);

			if(bat_cur_diff < o2_debug_sdpm_judg_sopdisfunc_ibatth) {
				ilimit_dbg("[%s] try ilimit level %d(%d) IBAT-ABS-Checking Fail: %d",
						__FUNCTION__, adji_ilel_array[i], i, bat_cur_diff);

				adji_ret = ADJI_RET_SOPDISFUNC ;
				fail = 1 ;
				break ;
			}			
			ilimit_dbg("[%s] try ilimit level %d(%d) IBAT-ABS-Checking OK: %d",
					__FUNCTION__, adji_ilel_array[i], i, bat_cur_diff);

			if(i > 1) {
				bat_cur_diff = adji_bat_curr[i] - adji_bat_curr[i-2] ;

				//if the current does not increase obviously	
				if(bat_cur_diff < o2_debug_sdpm_judg_ienough_ibatth) {
					ilimit_dbg("[%s] try ilimit level %d(%d) IBAT-ENG-Checking Fail: %d",
							__FUNCTION__, adji_ilel_array[i], i, bat_cur_diff);
				
					adji_ret = ADJI_RET_IENOUGH ;
					fail =1 ;
					break ;
				}

				ilimit_dbg("[%s] try ilimit level %d(%d) IBAT-ENG-Checking OK: %d",
						__FUNCTION__, adji_ilel_array[i], i, bat_cur_diff);
	
			}

		} else {
			ilimit_dbg("[%s] bat_crr[%d](%d)", __FUNCTION__, i, adji_bat_curr[i]);
		}
	}

	if(fail == 1) {

		adji_backoff_step_cnt = 0 ;

#if O2_CONFIG_ILIMIT_NTAG_SUPPORT
if(o2_ntag_support) 
{

		if(i>1) {
			if( (adji_ret == ADJI_RET_NOISE_OR_APOWERLOW) ||
				(adji_ret == ADJI_RET_VBUSLOW) || (adji_ret == ADJI_RET_SOPDISFUNC) ) {

				if(adji_ilel_ntag_judge_cnt[i] < o2_ntag_debug_cnt_num) {
					adji_ilel_ntag_judge_cnt[i] += 1 ;
					ilimit_dbg("[%s] [NTAG] ilimit-level (%d) fails, ntag count num: %d.",
							__FUNCTION__, adji_ilel_array[i], adji_ilel_ntag_judge_cnt[i]);
				}

				
				if(adji_ilel_ntag_judge_cnt[i] == o2_ntag_debug_cnt_num) {

#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
if(o2_ytag_support)
{

					ilimit_ytag = adji_ytag_get_min_ilimit() ;
					
					if(adji_ilel_array[i] > ilimit_ytag) {
						adji_ntag_set_ilimit(adji_ilel_array[i]);

						ilimit_dbg("[%s] [NTAG] ilimit-level (%d) reach max count (%d), not YTAG, ntag it.",
								__FUNCTION__, adji_ilel_array[i], adji_ilel_ntag_judge_cnt[i]);
					} else {

						ilimit_dbg("[%s] [NTAG] ilimit-level (%d) reach max count (%d), but YTAG, ignore it.",
								__FUNCTION__, adji_ilel_array[i], adji_ilel_ntag_judge_cnt[i]);

					}

}
else
#endif 
{
					adji_ntag_set_ilimit(adji_ilel_array[i]);
					ilimit_dbg("[%s] [NTAG] ilimit-level (%d) reach max count (%d), ntag it.",
							__FUNCTION__, adji_ilel_array[i], adji_ilel_ntag_judge_cnt[i]);

}
				}
			}
		}

}
#endif

#if (O2_CONFIG_ILIMIT_BACKOFF_TWOSTEP)
if(o2_backoff_twostep_support) 
{

		//wait some time
		msleep(50) ;

}
#endif

		ilimit_dbg("[%s] Fail at %d(%d)", __FUNCTION__, adji_ilel_array[i], i);
	
		//make sure we try raise ilimit to 700 at the least	
		if (i>1) {
			i--;
			adji_backoff_step_cnt = 1 ;
		}
		
		ret = adji_try_raise_ilimit(charger, adji_ilel_array[i]) ;


#if (O2_CONFIG_ILIMIT_BACKOFF_TWOSTEP)
if(o2_backoff_twostep_support) 
{


		if (ret == 0) {

			//these three results is too dangerous
			//need to keep lowering ilimit if necessary
			if ( (adji_ret == ADJI_RET_NOISE_OR_APOWERLOW) || (adji_ret == ADJI_RET_VBUSLOW) 
				|| (adji_ret == ADJI_RET_SOPDISFUNC) ) {

				adji_get_avg_info(&ibat, &vbat, &vbus_volt, 0) ;
			
				//if vbus-volt is still low 
				//keep decreasing ilimit level
				if (vbus_volt < (ADJI_JUDG_VBUSVOLT_LOWTH + 50) ) {
					backoff_2nd = 1 ;
				} else {
					backoff_2nd = 2 ;
				}

			}
			
		}


		if ( (i>1) && ( (ret != 0) || (backoff_2nd == 1) ) ) {

			//update adji_ret
			if(ret != 0) 
				adji_ret = ADJI_RET_NOISE_OR_APOWERLOW ;

			if(backoff_2nd == 1)
				adji_ret = ADJI_RET_VBUSLOW ;

			//wait some time
			msleep(50) ;

			//make sure we try raise ilimit to 700 at the least	
			i-- ;
			adji_backoff_step_cnt = 2 ;

			ilimit_dbg("[%s]: first step back fail, try backoff second step: %d(%d)", 
					__FUNCTION__, adji_ilel_array[i], i);

			ret = adji_try_raise_ilimit(charger, adji_ilel_array[i]) ;
	
		}

		//
		if(backoff_2nd != 2)
			adji_get_avg_info(&ibat, &vbat, &vbus_volt, 0) ;


}
else
{
		adji_get_avg_info(&ibat, &vbat, &vbus_volt, 0) ;
}
#else
		adji_get_avg_info(&ibat, &vbat, &vbus_volt, 0) ;
#endif

		adji_chrenv_update_momt(vbus_volt, vbat, ibat) ;
		adji_chrenv_update_adjmomt(vbus_volt, vbat, ibat) ;
		
		if(ret == 0) {
			ilimit_dbg("[%s]: succeed at %d", __FUNCTION__, adji_ilel_array[i] ) ;

			if(temp_cc != 2200)
				bluewhale_set_charger_current(charger, temp_cc) ;

			return adji_ilel_array[i] ;
		} else {
			ilimit_dbg("[%s]: succeed at %d", __FUNCTION__, adji_ilel_array[0] ) ;

			if(temp_cc != 2200)
				bluewhale_set_charger_current(charger, temp_cc) ;

			return adji_ilel_array[0] ;
		}
	}

	
	adji_backoff_step_cnt = 0 ;
	adji_ret = ADJI_RET_HIGHEST ;
	i-- ;
	
	ilimit_dbg("[%s] succeed at ADJI_RET_HIGHEST", __FUNCTION__);

	adji_chrenv_update_momt(vbus_volt, vbat, ibat) ;
	adji_chrenv_update_adjmomt(vbus_volt, vbat, ibat) ;
	if(temp_cc != 2200)
		bluewhale_set_charger_current(charger, temp_cc) ;
	return adji_ilel_array[i] ;
}


static int32_t  adji_adjust_ilimit_wrapper(struct bluewhale_charger_info *charger, int32_t ilimit)
{
	int32_t vbus_volt = 0 ;
	int32_t bat_volt = 0 ;
	int32_t bat_curr = 0 ;

	int32_t tempcv = 0 ;
	int32_t vbus_ilimit_lel = 0 ;
	int32_t check_ret = 0 ;
	int32_t ret = 0 ;

	//get all vol/curr information	
	//bat_curr = adji_get_bat_curr() ;
	//bat_volt = adji_get_bat_volt() ;
	//vbus_volt = adji_get_vbus_volt() ;
	adji_get_avg_info(&bat_curr, &bat_volt, &vbus_volt, 1) ;
	
	ilimit_dbg("[%s]: vbus_volt: %d, bat_volt: %d, bat_curr: %d", __FUNCTION__, vbus_volt, bat_volt, bat_curr);

	//get ilimit-level and CV Setting, and then check ilimit
	ret = bluewhale_get_vbus_current(charger) ;
	if(ret < 0) {
		return ret ;
	} else {
		vbus_ilimit_lel = ret ;
	}
	ilimit_dbg("[%s]: vbus_ilimit_lel: %d", __FUNCTION__, vbus_ilimit_lel);
		
	ret = bluewhale_get_chg_volt(charger);
	if(ret < 0) {
		return ret;
	} else {
		tempcv =  ret ;
	}

	check_ret = adji_check_ilimit(vbus_ilimit_lel, tempcv, vbus_volt, bat_volt, bat_curr) ;

	ilimit_dbg("[%s] adji_check_ilimit ret: %d", __FUNCTION__, check_ret);
	if(check_ret != ADJI_CHECK_ILIMIT_RET_OK) {

		ret = adji_adjust_ilimit_up(charger, 0, ilimit) ; 
		
		/** not distinguish these specific conditions for now
		//reserved for future inprovement

		if( check_ret == ADJI_CHECK_ILIMIT_RET_SOP || check_ret == ADJI_CHECK_ILIMIT_RET_LOWVOL || check_ret == ADJI_CHECK_ILIMIT_RET_LOWCUR ) {
			ret = adji_adjust_ilimit_up(charger, 0, ilimit) ; 
		} else if( check_ret == ADJI_CHECK_ILIMIT_RET_EVB || check_ret == ADJI_CHECK_ILIMIT_RET_EVW ) {
			ret = adji_adjust_ilimit_up(charger, vbus_ilimit_lel, ilimit) ; 
		}
		*/

	} else {
		adji_chrenv_update_momt(vbus_volt, bat_volt, bat_curr) ;
	}

	return ret ;
}


static void bluewhale_dis_sdpm(void)
{
	adji_started = 0 ;
	adji_chrenv_reset_all();
	printk("bluewhale_dis_sdpm\n");
#if O2_CONFIG_ILIMIT_NTAG_SUPPORT
if(o2_ntag_support)
{
    printk("adji_ntag_clear_all\n");
	adji_ntag_clear_all();

}
#endif

#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
if(o2_ytag_support)
{

	adji_ytag_clear_all();

}
#endif

}

 

#endif	//O2_CONFIG_SOFTWARE_DPM_SUPPORT


int32_t bluewhale_set_vbus_current_extern(int32_t ilmt_ma)
{
	int32_t ret = 0;

#if O2_CONFIG_SOFTWARE_DPM_SUPPORT
	static int32_t adji_ilimit_pre = 0 ;
#endif

	mutex_lock(&bluewhale_mutex);

	if (charger_info) {
#if O2_CONFIG_SOFTWARE_DPM_SUPPORT
		if(ilmt_ma <= o2_debug_sdpm_ilimit_threshold) {
			//set vbus current directly
			ret = bluewhale_set_vbus_current(charger_info, ilmt_ma);
			bluewhale_dis_sdpm();
		} else {

			if(adji_started == 0) {
				ilimit_dbg("[%s] adji just started fot the first time.", __FUNCTION__);
				ret = adji_adjust_ilimit_up(charger_info, 0, ilmt_ma) ; 
				adji_ilimit_pre = ilmt_ma ;
				adji_started = 1 ;
			} else {
				if(adji_ilimit_pre != ilmt_ma) {
					ilimit_dbg("[%s] adji stared, ilimit changed: old %d, new %d.",
							 __FUNCTION__, adji_ilimit_pre, ilmt_ma);
					ret = adji_adjust_ilimit_up(charger_info, 0, ilmt_ma) ; 
					adji_ilimit_pre = ilmt_ma ;
				} else {
					ilimit_dbg("[%s] adji stared, ilimit not changed: %d.",
							 __FUNCTION__, ilmt_ma);
					ret = adji_adjust_ilimit_wrapper(charger_info, ilmt_ma) ;
				}	
			}

		}
#else
		ret = bluewhale_set_vbus_current(charger_info, ilmt_ma);
#endif
	} else {
		ret = -EINVAL;
	}

	mutex_unlock(&bluewhale_mutex);
	return ret;
}


int32_t bluewhale_power_off_config_extern(void)
{
	int32_t ret = 0 ;

	//mutex_lock(&bluewhale_mutex);

	if(charger_info) {

		//set ilimit to default 500
		ret = bluewhale_set_vbus_current(charger_info, 500);

#if O2_CONFIG_SOFTWARE_DPM_SUPPORT
		bluewhale_dis_sdpm() ;
#endif

	} else {
		ret = -EINVAL ;
	}

	//mutex_unlock(&bluewhale_mutex);

	return ret ;
}


int32_t bluewhale_cable_out_config_extern(void)
{
	int32_t ret = 0 ;

	mutex_lock(&bluewhale_mutex);
    printk("bluewhale_cable_out_config_extern\n");
	if(charger_info) {

		//set ilimit to default 500
		ret = bluewhale_set_vbus_current(charger_info, 500);

#if O2_CONFIG_SOFTWARE_DPM_SUPPORT
		bluewhale_dis_sdpm() ;
#endif

	} else {
		ret = -EINVAL ;
	}

	mutex_unlock(&bluewhale_mutex);

	return ret ;
}






/*
 * Section: bluewhale attribues
 */



//SDPM Debugging Attributes
#if O2_CONFIG_SOFTWARE_DPM_SUPPORT



static ssize_t bluewhale_sdpm_debug_ilimit_th_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_ilimit_threshold = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_ilimit_th_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_ilimit_threshold:%d\n", o2_debug_sdpm_ilimit_threshold);
}



static ssize_t bluewhale_sdpm_debug_ilimit_high_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_ilimit_high = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_ilimit_high_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_ilimit_high:%d\n", o2_debug_sdpm_ilimit_high);
}



static ssize_t bluewhale_sdpm_debug_sop_check_times_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_sop_check_times = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_sop_check_times_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_sop_check_times:%d\n", o2_debug_sdpm_sop_check_times);
}




static ssize_t bluewhale_sdpm_debug_judg_ienough_ibatth_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_judg_ienough_ibatth = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_judg_ienough_ibatth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_judg_ienough_ibatth:%d\n", o2_debug_sdpm_judg_ienough_ibatth);
}





static ssize_t bluewhale_sdpm_debug_judg_sopdisfunc_ibatth_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_judg_sopdisfunc_ibatth = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_judg_sopdisfunc_ibatth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_judg_sopdisfunc_ibatth:%d\n", o2_debug_sdpm_judg_sopdisfunc_ibatth);
}




static ssize_t bluewhale_sdpm_debug_judg_adosc_ibatth_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_judg_adosc_ibatth = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_judg_adosc_ibatth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_judg_adosc_ibatth:%d\n", o2_debug_sdpm_judg_adosc_ibatth);
}



static ssize_t bluewhale_sdpm_debug_judg_adosc_vbusth_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_judg_adosc_vbusth = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_judg_adosc_vbusth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_judg_adosc_vbusth:%d\n", o2_debug_sdpm_judg_adosc_vbusth);
}




static ssize_t bluewhale_sdpm_debug_sop_check_wait_time_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_sop_check_wait_time = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_sop_check_wait_time_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_sop_check_wait_time:%d\n", o2_debug_sdpm_sop_check_wait_time);
}




static ssize_t bluewhale_sdpm_debug_sysenv_change_ibatth_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_sysenv_change_ibatth = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_sysenv_change_ibatth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_sysenv_change_ibatth:%d\n", o2_debug_sdpm_sysenv_change_ibatth);
}




static ssize_t bluewhale_sdpm_debug_sysenv_change_vbusth_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_debug_sdpm_sysenv_change_vbusth = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_sysenv_change_vbusth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_sysenv_change_vbusth:%d\n", o2_debug_sdpm_sysenv_change_vbusth);
}





static ssize_t bluewhale_sdpm_debug_sysenv_better_ibatth_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t vall, valm, valh;

        if (sscanf(buf, "%d %d %d", &vall, &valm, &valh) == 3) { 
		o2_debug_sdpm_sysenv_better_ibatth_il = vall ;
		o2_debug_sdpm_sysenv_better_ibatth_im = valm ;
		o2_debug_sdpm_sysenv_better_ibatth_ih = valh ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_sysenv_better_ibatth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_sysenv_better: %d(il) %d(im) %d(ih)\n", 
		o2_debug_sdpm_sysenv_better_ibatth_il, 
		o2_debug_sdpm_sysenv_better_ibatth_im, 
		o2_debug_sdpm_sysenv_better_ibatth_ih);
}





static ssize_t bluewhale_sdpm_debug_avg_info_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val1, val2, val3;

        if (sscanf(buf, "%d %d %d", &val1, &val2, &val3) == 3) { 
		o2_debug_sdpm_avg_info_check_interval = val1 ; 
		o2_debug_sdpm_avg_info_first_delay = val2 ; 
		o2_debug_sdpm_avg_info_wait_time = val3 ; 
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_debug_avg_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_debug_sdpm_avg_info: %d(check interval) %d(first delay) %d(wait time)\n", 
		o2_debug_sdpm_avg_info_check_interval,
		o2_debug_sdpm_avg_info_first_delay,
		o2_debug_sdpm_avg_info_wait_time);
}











#if O2_CONFIG_LINEAR_COMP_SUPPORT

static ssize_t bluewhale_sdpm_conf_linear_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_linear_comp_support = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_conf_linear_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "linear:%d\n", o2_linear_comp_support);
}

#endif

#if O2_CONFIG_ILIMIT_BACKOFF_TWOSTEP

static ssize_t bluewhale_sdpm_conf_backoff_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_backoff_twostep_support = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_conf_backoff_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "backoff:%d\n", o2_backoff_twostep_support);
}

#endif

#if O2_CONFIG_ILIMIT_NTAG_SUPPORT

static ssize_t bluewhale_sdpm_ntag_debug_cnt_num_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_ntag_debug_cnt_num = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_ntag_debug_cnt_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_ntag_debug_cnt_num:%d\n", o2_ntag_debug_cnt_num);
}


static ssize_t bluewhale_sdpm_conf_ntag_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_ntag_support = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_conf_ntag_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ntag:%d\n", o2_ntag_support);
}
#endif


#if O2_CONFIG_ILIMIT_YTAG_SUPPORT

static ssize_t bluewhale_sdpm_ytag_debug_cnt_num_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_ytag_debug_cnt_num = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_ytag_debug_cnt_num_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_ytag_debug_cnt_num:%d\n", o2_ytag_debug_cnt_num);
}

static ssize_t bluewhale_sdpm_ytag_debug_ibatth_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_ytag_debug_ibatth = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_ytag_debug_ibatth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_ytag_debug_ibatth:%d\n", o2_ytag_debug_ibatth);
}


static ssize_t bluewhale_sdpm_ytag_debug_vbatth_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_ytag_debug_vbatth = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_ytag_debug_vbatth_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "o2_ytag_debug_vbatth:%d\n", o2_ytag_debug_vbatth);
}


static ssize_t bluewhale_sdpm_conf_ytag_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;

        if (sscanf(buf, "%d", &val) == 1) { 
		o2_ytag_support = val ;
		return count;
	} else {
		return -EINVAL ;
	}

}

ssize_t bluewhale_sdpm_conf_ytag_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "ytag:%d\n", o2_ytag_support);
}

#endif

#endif //O2_CONFIG_SOFTWARE_DPM_SUPPORT










static int32_t otg_enable = 0;
static ssize_t bluewhale_otg_store(struct device *dev, struct device_attribute *attr, 
               const char *buf, size_t count) 
{
	int32_t val;
	struct bluewhale_charger_info *charger;

	if (kstrtoint(buf, 10, &val))
		return -EINVAL;

	if (charger_info->dev == dev)
	    charger = dev_get_drvdata(dev);
	else //this device is the provate device of power supply 
	    charger = dev_get_drvdata(dev->parent);

	if (val == 1 || val == 0) {
		bluewhale_enable_otg(charger, val);
		bluewhale_dbg("%s: %s OTG", __func__, val?"enable":"disable");
		otg_enable = val;

		return count;
	} else {
		bluewhale_dbg("%s: wrong parameters", __func__);
	}

	return -EINVAL;
}

ssize_t bluewhale_otg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", otg_enable);
}

static int32_t reg_enable = 0;

#define BLUEWHALE_PARAMS_ATTR(x)  \
static ssize_t bluewhale_write_##x(struct device *dev, struct device_attribute *attr, \
		const char *buf, size_t count) \
{ \
	int data; \
	struct bluewhale_charger_info *charger;\
	if (charger_info->dev == dev)\
	    charger = dev_get_drvdata(dev);\
	else\
	    charger = dev_get_drvdata(dev->parent);\
	if( kstrtoint(buf, 10, &data) == 0) { \
		if(reg_enable ==1)\
		    bluewhale_set_##x(charger, data);\
		return count; \
	}else { \
		pr_err("bluewhale reg %s set failed\n",#x); \
		return 0; \
	} \
	return 0;\
}\
static ssize_t bluewhale_read_##x(struct device *dev, struct device_attribute *attr, \
               char *buf) \
{ \
	struct bluewhale_charger_info *charger;\
	if (charger_info->dev == dev)\
	    charger = dev_get_drvdata(dev);\
	else\
	    charger = dev_get_drvdata(dev->parent);\
	charger->x = bluewhale_get_##x(charger);\
    return sprintf(buf,"%d\n", charger->x);\
}

BLUEWHALE_PARAMS_ATTR(vbus_current) 	//input current limit
BLUEWHALE_PARAMS_ATTR(charger_current) 	//charge current
BLUEWHALE_PARAMS_ATTR(chg_volt) 	//charge voltage

static ssize_t bluewhale_registers_show(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int32_t result = 0;
	u8 i = 0;
	u8 data = 0;
	struct bluewhale_charger_info *charger;

	if (charger_info->dev == dev)
	    charger = dev_get_drvdata(dev);
	else //this device is the provate device of power supply 
	    charger = dev_get_drvdata(dev->parent);

    	for (i=REG_CHARGER_VOLTAGE; i<=REG_MIN_VSYS_VOLTAGE; i++) {
		bluewhale_read_byte(charger, i, &data);
		result += sprintf(buf + result, "[0x%.2x] = 0x%.2x\n", i, data);
    	}

    	for (i=REG_CHARGE_CURRENT; i<=REG_VBUS_LIMIT_CURRENT; i++) {
		bluewhale_read_byte(charger, i, &data);
		result += sprintf(buf + result, "[0x%.2x] = 0x%.2x\n", i, data);
	}

	bluewhale_read_byte(charger, REG_SAFETY_TIMER, &data);
	result += sprintf(buf + result, "[0x%.2x] = 0x%.2x\n", REG_SAFETY_TIMER, data);

	bluewhale_read_byte(charger, REG_CHARGER_CONTROL, &data);
	result += sprintf(buf + result, "[0x%.2x] = 0x%.2x\n", REG_CHARGER_CONTROL, data);

    	for (i=REG_VBUS_STATUS; i<=REG_THM_STATUS; i++) {
		bluewhale_read_byte(charger, i, &data);
		result += sprintf(buf + result, "[0x%.2x] = 0x%.2x\n", i, data);
    	}
	
	return result;
}

static DEVICE_ATTR(registers, S_IRUGO, bluewhale_registers_show, NULL);
static DEVICE_ATTR(vbus_current, S_IWUSR|S_IRUGO, bluewhale_read_vbus_current, bluewhale_write_vbus_current);
static DEVICE_ATTR(charger_current, S_IWUSR|S_IRUGO, bluewhale_read_charger_current, bluewhale_write_charger_current);
static DEVICE_ATTR(chg_volt, S_IWUSR|S_IRUGO, bluewhale_read_chg_volt, bluewhale_write_chg_volt);
static DEVICE_ATTR(enable_otg, S_IRUGO | S_IWUSR, bluewhale_otg_show, bluewhale_otg_store);

#if O2_CONFIG_SOFTWARE_DPM_SUPPORT

#if O2_CONFIG_LINEAR_COMP_SUPPORT
static DEVICE_ATTR(config_linear, S_IRUGO | S_IWUSR, bluewhale_sdpm_conf_linear_show, bluewhale_sdpm_conf_linear_store);
#endif

#if O2_CONFIG_ILIMIT_BACKOFF_TWOSTEP
static DEVICE_ATTR(config_backoff, S_IRUGO | S_IWUSR, bluewhale_sdpm_conf_backoff_show, bluewhale_sdpm_conf_backoff_store);
#endif

#if O2_CONFIG_ILIMIT_NTAG_SUPPORT
static DEVICE_ATTR(config_ntag, S_IRUGO | S_IWUSR, bluewhale_sdpm_conf_ntag_show, bluewhale_sdpm_conf_ntag_store);
static DEVICE_ATTR(ntag_debug_cntnum, S_IRUGO | S_IWUSR, bluewhale_sdpm_ntag_debug_cnt_num_show, bluewhale_sdpm_ntag_debug_cnt_num_store);
#endif

#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
static DEVICE_ATTR(config_ytag, S_IRUGO | S_IWUSR, bluewhale_sdpm_conf_ytag_show, bluewhale_sdpm_conf_ytag_store);
static DEVICE_ATTR(ytag_debug_cntnum, S_IRUGO | S_IWUSR, bluewhale_sdpm_ytag_debug_cnt_num_show, bluewhale_sdpm_ytag_debug_cnt_num_store);
static DEVICE_ATTR(ytag_debug_ibatth, S_IRUGO | S_IWUSR, bluewhale_sdpm_ytag_debug_ibatth_show, bluewhale_sdpm_ytag_debug_ibatth_store);
static DEVICE_ATTR(ytag_debug_vbatth, S_IRUGO | S_IWUSR, bluewhale_sdpm_ytag_debug_vbatth_show, bluewhale_sdpm_ytag_debug_vbatth_store);
#endif


static DEVICE_ATTR(debug_ilimit_th, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_ilimit_th_show, bluewhale_sdpm_debug_ilimit_th_store);
static DEVICE_ATTR(debug_ilimit_high, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_ilimit_high_show, bluewhale_sdpm_debug_ilimit_high_store);
static DEVICE_ATTR(debug_sop_check_times, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_sop_check_times_show, bluewhale_sdpm_debug_sop_check_times_store);
static DEVICE_ATTR(debug_judg_ienough_ibatth, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_judg_ienough_ibatth_show, bluewhale_sdpm_debug_judg_ienough_ibatth_store);
static DEVICE_ATTR(debug_judg_sopdisfunc_ibatth, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_judg_sopdisfunc_ibatth_show, bluewhale_sdpm_debug_judg_sopdisfunc_ibatth_store);
static DEVICE_ATTR(debug_judg_adosc_ibatth, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_judg_adosc_ibatth_show, bluewhale_sdpm_debug_judg_adosc_ibatth_store);
static DEVICE_ATTR(debug_judg_adosc_vbusth, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_judg_adosc_vbusth_show, bluewhale_sdpm_debug_judg_adosc_vbusth_store);
static DEVICE_ATTR(debug_sop_check_wait_time, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_sop_check_wait_time_show, bluewhale_sdpm_debug_sop_check_wait_time_store);
static DEVICE_ATTR(debug_sysenv_change_ibatth, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_sysenv_change_ibatth_show, bluewhale_sdpm_debug_sysenv_change_ibatth_store);
static DEVICE_ATTR(debug_sysenv_change_vbusth, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_sysenv_change_vbusth_show, bluewhale_sdpm_debug_sysenv_change_vbusth_store);
static DEVICE_ATTR(debug_sysenv_better_ibatth, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_sysenv_better_ibatth_show, bluewhale_sdpm_debug_sysenv_better_ibatth_store);
static DEVICE_ATTR(debug_avg_info, S_IRUGO | S_IWUSR, bluewhale_sdpm_debug_avg_info_show, bluewhale_sdpm_debug_avg_info_store);


#endif	//O2_CONFIG_SOFTWARE_DPM_SUPPORT


static struct attribute *bluewhale_attributes[] = {
	&dev_attr_registers.attr,
	&dev_attr_vbus_current.attr, //input current limit
	&dev_attr_charger_current.attr, //charge current
	&dev_attr_chg_volt.attr, //charge voltage
	//&dev_attr_vbus_volt.attr,     //vbus voltage
	&dev_attr_enable_otg.attr, //enable/disable OTG(boost)
	
#if O2_CONFIG_SOFTWARE_DPM_SUPPORT

	//for debugging sdpm
#if O2_CONFIG_LINEAR_COMP_SUPPORT
	&dev_attr_config_linear.attr,
#endif

#if O2_CONFIG_ILIMIT_BACKOFF_TWOSTEP
	&dev_attr_config_backoff.attr,
#endif

#if O2_CONFIG_ILIMIT_NTAG_SUPPORT
	&dev_attr_config_ntag.attr,
	&dev_attr_ntag_debug_cntnum.attr,
#endif

#if O2_CONFIG_ILIMIT_YTAG_SUPPORT
	&dev_attr_config_ytag.attr,
	&dev_attr_ytag_debug_cntnum.attr,
	&dev_attr_ytag_debug_ibatth.attr,
	&dev_attr_ytag_debug_vbatth.attr,
#endif

	&dev_attr_debug_ilimit_th.attr,
	&dev_attr_debug_ilimit_high.attr,
	&dev_attr_debug_sop_check_times.attr,
	&dev_attr_debug_judg_ienough_ibatth.attr,
	&dev_attr_debug_judg_sopdisfunc_ibatth.attr,
	&dev_attr_debug_judg_adosc_ibatth.attr,
	&dev_attr_debug_judg_adosc_vbusth.attr,
	&dev_attr_debug_sop_check_wait_time.attr,
	&dev_attr_debug_sysenv_change_ibatth.attr,
	&dev_attr_debug_sysenv_change_vbusth.attr,
	&dev_attr_debug_sysenv_better_ibatth.attr,
	&dev_attr_debug_avg_info.attr,


#endif	//O2_CONFIG_SOFTWARE_DPM_SUPPORT
	NULL,
};

static struct attribute_group bluewhale_attribute_group = {
	.attrs = bluewhale_attributes,
};
static int bluewhale_create_sys(struct device *dev)
{
	int err;

	printk(KERN_NOTICE "bluewhale_create_sysfs\n");	
	
	if(NULL == dev){
		printk(KERN_ERR "%s: creat bluewhale sysfs fail: NULL dev\n", __func__);
		return -EINVAL;
	}

	err = sysfs_create_group(&(dev->kobj), &bluewhale_attribute_group);

	if (err) {
		printk(KERN_ERR "%s: creat bluewhale sysfs group fail\n", __func__);
		return -EIO;
	}

	printk(KERN_NOTICE "creat bluewhale sysfs group ok\n");	
	return err;
}










/*****************************************************************************
 * Description:
 *		bluewhale_charger_probe
 * Parameters:
 *		
 * Return:
 *      negative errno if any error
 *****************************************************************************/
static int  bluewhale_charger_probe(struct i2c_client *client,
									const struct i2c_device_id *id)
{
	struct bluewhale_charger_info *charger;
	int ret = 0;

	printk(KERN_NOTICE "O2Micro bluewhale Driver Loading 0\n");

	charger = devm_kzalloc(&client->dev, sizeof(*charger), GFP_KERNEL);
	if (!charger) {
		dev_err(&client->dev, "Can't alloc charger struct\n");
		return -ENOMEM;
	}
	
	charger->client = client;	
	charger->dev = &client->dev;

	i2c_set_clientdata(client, charger);

	//for public use
	charger_info = charger;

	printk(KERN_NOTICE "O2Micro bluewhale Driver Loading 1\n");

	//sys/class/i2c-dev/i2c-2/device/2-0010/
	ret = bluewhale_create_sys(&(client->dev));
	
	if(ret){
		printk(KERN_ERR "Err failed to creat charger attributes\n");
		goto sys_failed;
	}

	//hardware initialization	
	bluewhale_init_default(charger);
	
#if O2_CONFIG_MTK_PLATFORM_SUPPORT
	chargin_hw_init_done = true;
#endif

	printk(KERN_NOTICE "O2Micro bluewhale Driver Loading 2\n");
	return 0;

sys_failed:

	return ret;
}


/*****************************************************************************
 * Description:
 *		bluewhale_charger_remove
 * Parameters:
 *		
 * Return:
 *      negative errno if any error
 *****************************************************************************/
static int  bluewhale_charger_remove(struct i2c_client *client)
{
	struct bluewhale_charger_info *charger = i2c_get_clientdata(client);

	bluewhale_set_charger_current(charger, CHGR_DEF_CC);
	bluewhale_set_vbus_current(charger, CHGR_DEF_ILIMIT);

	printk(KERN_CRIT"%s: bluewhale charger driver removed\n", __func__);
	return 0;
}
/*****************************************************************************
 * Description:
 *		bluewhale_charger_shutdown
 * Parameters:
 *****************************************************************************/
static void  bluewhale_charger_shutdown(struct i2c_client *client)
{
	struct bluewhale_charger_info *charger = i2c_get_clientdata(client);

	printk(KERN_CRIT"%s: bluewhale charger driver shutdown\n", __func__);

	bluewhale_set_charger_current(charger, CHGR_DEF_CC);
	bluewhale_set_vbus_current(charger, CHGR_DEF_ILIMIT);
	
	bluewhale_update_bits(charger, REG_CHARGER_CONTROL, 0x80, 0x0 << 7);
}


/*****************************************************************************
 * Description:
 *		bluewhale_charger_suspend
 * Parameters:
 *		
 * Return:
 *      negative errno if any error
 *****************************************************************************/
static int bluewhale_charger_suspend(struct device *dev)
{
	return 0;
}


/*****************************************************************************
 * Description:
 *		bluewhale_charger_resume
 * Parameters:
 *		
 * Return:
 *      negative errno if any error
 *****************************************************************************/
static int bluewhale_charger_resume(struct device *dev)
{
	return 0;
}

static const struct i2c_device_id bluewhale_i2c_ids[] = {
		{"bluewhale-charger", 0},
		{}
}; 

MODULE_DEVICE_TABLE(i2c, bluewhale_i2c_ids);

static const struct dev_pm_ops pm_ops = {
        .suspend        = bluewhale_charger_suspend,
        .resume		= bluewhale_charger_resume,
};

#ifdef CONFIG_OF
static const struct of_device_id oz1c105c_of_match[] = {
	{.compatible = "mediatek,sw_charger"},
	{},
};
MODULE_DEVICE_TABLE(of, oz1c105c_of_match);
#else
static struct i2c_board_info __initdata i2c_bluewhale={ I2C_BOARD_INFO("bluewhale-charger", 0x10)}; // 7-bit address of 0x20
#endif

static struct i2c_driver bluewhale_charger_driver = {
	.driver	= {
		.name 	= "bluewhale-charger",
		.pm	= &pm_ops,
	#ifdef CONFIG_OF
		.of_match_table = oz1c105c_of_match,
	#endif
	},
	.probe		= bluewhale_charger_probe,
	.shutdown	= bluewhale_charger_shutdown,
	.remove     	= bluewhale_charger_remove,
	.id_table	= bluewhale_i2c_ids,
};

static int __init bluewhale_charger_init(void)
{
#ifndef CONFIG_OF
	struct i2c_adapter *i2c_adap;
	static struct i2c_client *charger_client; //Notice: this is static pointer
#endif
	int ret = 0;

	pr_err("%s\n", __func__);

#ifndef CONFIG_OF
	i2c_adap = i2c_get_adapter(CHARGER_I2C_NUM);
	if (i2c_adap)
	{
		charger_client = i2c_new_device(i2c_adap, &i2c_bluewhale);
		i2c_put_adapter(i2c_adap);
	}
	else {
		pr_err("failed to get adapter %d.\n", CHARGER_I2C_NUM);
		pr_err("statically declare I2C devices\n");

		ret = i2c_register_board_info(CHARGER_I2C_NUM, &i2c_bluewhale, 1);

		if(ret != 0)
			pr_err("failed to register i2c_board_info.\n");
	}
#endif

	ret = i2c_add_driver(&bluewhale_charger_driver);

	if(ret != 0)
        pr_err("failed to register bluewhale_charger i2c driver.\n");
	else
        pr_err("Success to register bluewhale_charger i2c driver.\n");

	return ret;
}

static void __exit bluewhale_charger_exit(void)
{
	printk(KERN_CRIT"bluewhale driver exit\n");
	
	i2c_del_driver(&bluewhale_charger_driver);
}

module_init(bluewhale_charger_init);
module_exit(bluewhale_charger_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("O2Micro");
MODULE_DESCRIPTION("O2Micro BlueWhale Charger Driver");
