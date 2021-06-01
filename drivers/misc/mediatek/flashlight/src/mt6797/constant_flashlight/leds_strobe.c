/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/cdev.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/time.h>
#include "kd_flashlight.h"
#include <asm/io.h>
#include <asm/uaccess.h>
#include "kd_flashlight_type.h"
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/i2c.h>
#include <linux/leds.h>

/*#include <cust_gpio_usage.h>*/
#include <mt_gpio.h>
#include <gpio_const.h>


#define TAG_NAME "[leds_strobe.c]"
#define PK_DBG_NONE(fmt, arg...)    do {} while (0)
#define PK_DBG_FUNC(fmt, arg...)    printk(TAG_NAME "%s: " fmt, __func__ , ##arg)

//#define DEBUG_LEDS_STROBE
#ifdef DEBUG_LEDS_STROBE
#define PK_DBG PK_DBG_FUNC
#else
#define PK_DBG PK_DBG_NONE
#endif

/******************************************************************************
 * local variables
******************************************************************************/
static DEFINE_SPINLOCK(g_strobeSMPLock);	/* cotta-- SMP proection */

static u32 strobe_Res = 0;
static u32 strobe_Timeus = 0;
static BOOL g_strobe_On = 0;

static int g_timeOutTimeMs=0;

static DEFINE_MUTEX(g_strobeSem);

static struct work_struct workTimeOut;
static void work_timeOutFunc(struct work_struct *data);


#define STROBE_DEVICE_ID 0xC6


static int g_ataEnable=0;


struct mutex g_strobeLock;


struct LM3644_platform_data
{
	u8 torch_pin_enable;	/* 1:  TX1/TORCH pin isa hardware TORCH enable */
	u8 pam_sync_pin_enable;	/* 1:  TX2 Mode The ENVM/TX2 is a PAM Sync. on input */
	u8 thermal_comp_mode_enable;	/* 1: LEDI/NTC pin in Thermal Comparator Mode */
	u8 strobe_pin_disable;	/* 1 : STROBE Input disabled */
	u8 vout_mode_enable;	/* 1 : Voltage Out Mode enable */
};

struct LM3644_chip_data
{
	struct i2c_client *client;

	/* struct led_classdev cdev_flash; */
	/* struct led_classdev cdev_torch; */
	/* struct led_classdev cdev_indicator; */

	struct LM3644_platform_data *pdata;
	struct mutex lock;

	u8 last_flag;
	u8 no_pdata;
};

static int LM3644_chip_init(struct LM3644_chip_data *chip)
{


	return 0;
}
/*****************************************************************************
Functions
*****************************************************************************/
//extern int iWriteRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u16 i2cId);
//extern int iReadRegI2C(u8 *a_pSendData , u16 a_sizeSendData, u8 * a_pRecvData, u16 a_sizeRecvData, u16 i2cId);

#define LM3644_NAME "leds-LM3644"
static struct i2c_client *LM3644_i2c_client = NULL;
//static struct i2c_board_info __initdata i2c_LM3644={I2C_BOARD_INFO(LM3644_NAME, STROBE_DEVICE_ID>>1)};

static int readReg(u8 reg)
{
	int val=0;

	mutex_lock(&g_strobeLock);
	val =  i2c_smbus_read_byte_data(LM3644_i2c_client, reg);
	mutex_unlock(&g_strobeLock);
	return val;
}

static int writeReg(u8 reg, u8 data)
{
	int ret=0;

	mutex_lock(&g_strobeLock);
	ret =  i2c_smbus_write_byte_data(LM3644_i2c_client, reg, data);
	mutex_unlock(&g_strobeLock);

	if (ret < 0)
		PK_DBG("failed writting at 0x%02x\n", reg);
	return ret;
}

static int LM3644_probe(struct i2c_client *client,
                        const struct i2c_device_id *id)
{
	struct LM3644_chip_data *chip;
	struct LM3644_platform_data *pdata = client->dev.platform_data;

	int err = -1;

	PK_DBG("LM3644_probe start--->.\n");

	client->addr = STROBE_DEVICE_ID >> 1; //sanford.lin

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		err = -ENODEV;
		printk(KERN_ERR  "LM3644 i2c functionality check fail.\n");
		return err;
	}

	chip = kzalloc(sizeof(struct LM3644_chip_data), GFP_KERNEL);
	chip->client = client;

	mutex_init(&g_strobeLock);

	i2c_set_clientdata(client, chip);

	if (pdata == NULL)  	/* values are set to Zero. */
	{
		printk("LM3644 Platform data does not exist\n");
		pdata = kzalloc(sizeof(struct LM3644_platform_data), GFP_KERNEL);
		chip->pdata = pdata;
		chip->no_pdata = 1;
	}

	chip->pdata = pdata;
	if (LM3644_chip_init(chip) < 0)
		goto err_chip_init;

	LM3644_i2c_client = client;

	PK_DBG("LM3644 Initializing is done \n");

	return 0;
err_chip_init:
	i2c_set_clientdata(client, NULL);
	kfree(chip);
	PK_DBG("LM3644 probe is failed\n");
	return -ENODEV;
}

static int LM3644_remove(struct i2c_client *client)
{
	struct LM3644_chip_data *chip = i2c_get_clientdata(client);

	if (chip->no_pdata)
		kfree(chip->pdata);
	kfree(chip);
	return 0;
}

static const struct i2c_device_id LM3644_id[] =
{
	{LM3644_NAME, 0},
	{}
};

#ifdef CONFIG_OF
static const struct of_device_id strobe_of_match[] =
{
	{.compatible = "mediatek,strobe_main"},
	{},
};
#endif

static struct i2c_driver LM3644_i2c_driver =
{
	.driver = {
		.name  = LM3644_NAME,
#ifdef CONFIG_OF
		.of_match_table = strobe_of_match,
#endif
	},
	.probe	= LM3644_probe,
	  .remove   = LM3644_remove,
	   .id_table = LM3644_id,
    };

static int __init LM3644_init(void)
{
	printk("LM3644_init\n");

	//i2c_register_board_info(I2C_STROBE_MAIN_CHANNEL, &i2c_LM3644, 1);

	return i2c_add_driver(&LM3644_i2c_driver);
}

static void __exit LM3644_exit(void)
{
	i2c_del_driver(&LM3644_i2c_driver);
}

module_init(LM3644_init);
module_exit(LM3644_exit);

MODULE_DESCRIPTION("Flash driver for LM3644");
MODULE_AUTHOR("pw <pengwei@mediatek.com>");
MODULE_LICENSE("GPL v2");

enum
{
	e_DutyNum = 26,
};

static int isMovieMode[e_DutyNum] = {1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static int torchDuty[e_DutyNum] = {35,71,106,127,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
//50,100,150,179ma

//static int flashDuty[e_DutyNum] = {3,8,12,14,16,20,25,29,33,37,42,46,50,55,59,63,67,72,76,80,84,93,101,110,118,127};
//45,105,150,175,200,250,300,350,400,450,500,550,600,650,700,750,800,850,900,950,1000,1100,1200,1300,1400,1500mA

static int flashDuty[] = {3,8,12,14,16,18,20,22,24,26,29,32,36,39,43,45,48,52,55,59,63,67,72,76,80,84};
//45,105,150,175,200,225,250,270,290,325,355,385,435,470,515,535,575,615,655,700,750,800,855,900,950,1000mA

int m_duty1=0;
int m_duty2=0;
int LED1CloseFlag = 0;
int LED2CloseFlag = 0;

int flashEnable_LM3644_2(void)
{
	int err;
	int enable_value = 0;
	int flag=0;

	PK_DBG(" flashDisable_LM3644_2 S line=%d\n",__LINE__);

	enable_value = readReg(0x01);
	printk(" LED1&2_enable_value S =0x%x\n",enable_value);

	flag = readReg(0x0B);
	printk(" flashEnable_LM3644_2 flag =0x%x\n",flag);




	if((LED1CloseFlag == 1) && (LED2CloseFlag == 1))
	{
		err = writeReg(0x01, (enable_value & 0xF0));
		return 0;
	}

	if(LED1CloseFlag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
			err = writeReg(0x01, (enable_value & 0xF0) | 0x0A);
		else
			err = writeReg(0x01, (enable_value & 0xF0) | 0x0E);
	}
	else if(LED2CloseFlag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
			err = writeReg(0x01, (enable_value & 0xF0) | 0x09);
		else
			err = writeReg(0x01, (enable_value & 0xF0) | 0x0D);
	}
	else
	{
		if((isMovieMode[m_duty1] == 1) && (isMovieMode[m_duty2] == 1))
			err = writeReg(0x01, (enable_value & 0xF0) | 0x0B);
		else
			err = writeReg(0x01, (enable_value & 0xF0) | 0x0F);
	}

	if(g_ataEnable==1)
	{
		err = writeReg(0x01, (enable_value & 0xF0) | 0x0B);
		return 0;
	}
	else if(g_ataEnable==2)
	{
		err = writeReg(0x01, (enable_value & 0xF0) | 0x00);
		return 0;
	}

	enable_value = readReg(0x01);
	printk(" LED1&2_enable_value E =0x%x\n",enable_value);

	PK_DBG(" flashDisable_LM3644_2 E line=%d\n",__LINE__);
	return err;
}

void flashDisable_LM3644_2(void)
{
	flashEnable_LM3644_2();
}

int setDuty_LM3644_2(int duty)
{
	int err =0;
//	int led1_duty = 0, duty_Value;
	int temp;
	
	if(duty < 0)
		duty = 0;
	else if(duty >= e_DutyNum)
		duty = e_DutyNum - 1;
	m_duty2 = duty;
	printk("%s,LED1CloseFlag=%d,LED2CloseFlag=%d,m_duty1=%d,m_duty2=%d\n",__func__,LED1CloseFlag,LED2CloseFlag,m_duty1,m_duty2);

	if((LED1CloseFlag == 1) && (LED2CloseFlag == 1))
	{

	}
	else if(LED1CloseFlag == 1)
	{
		if(isMovieMode[m_duty2] == 1)
		{
			temp = readReg(0x06);
			printk(" LED2_torch_register S =0x%x\n",temp);
			err =  writeReg(0x06, torchDuty[m_duty2]);
			temp = readReg(0x06);
			printk(" LED2_torch_register E =0x%x\n",temp);
		}
		else
		{
			temp = readReg(0x04);
			printk(" LED2_flash_register S =0x%x\n",temp);
			err =  writeReg(0x04, flashDuty[m_duty2]);
			temp = readReg(0x04);
			printk(" LED2_flash_register E =0x%x\n",temp);
		}
	}
	else if(LED2CloseFlag == 1)
	{
		if(isMovieMode[m_duty1] == 1)
		{
			temp = readReg(0x05);
			printk(" LED1_torch_register S =0x%x\n",temp);
			err =  writeReg(0x05, torchDuty[m_duty1]);
			temp = readReg(0x05);
			printk(" LED1_torch_register E =0x%x\n",temp);

		}
		else
		{
			temp = readReg(0x03);
			printk(" LED1_flash_register S =0x%x\n",temp);
			err =  writeReg(0x03, flashDuty[m_duty1]);
			temp = readReg(0x03);
			printk(" LED1_flash_register E =0x%x\n",temp);

		}
	}
	else
	{
		if((isMovieMode[m_duty1] == 1)&&((isMovieMode[m_duty2] == 1)))
		{
			temp = readReg(0x05);
			temp = readReg(0x06);
			printk(" LED1&2_torch_register S =0x%x\n",temp);
			err =  writeReg(0x05, torchDuty[m_duty1]);
			err =  writeReg(0x06, torchDuty[m_duty2]);
			temp = readReg(0x05);
			temp = readReg(0x06);
			printk(" LED1&2_torch_register E =0x%x\n",temp);
		}
		else
		{
			temp = readReg(0x03);
			temp = readReg(0x04);
			printk(" LED1&2_flash_register S =0x%x\n",temp);
			err =  writeReg(0x03, flashDuty[m_duty1]);
			err =  writeReg(0x04, flashDuty[m_duty2]);
			temp = readReg(0x03);
			temp = readReg(0x04);
			printk(" LED1&2_flash_register E =0x%x\n",temp);
		}
	}

	if (g_ataEnable == 1)
	{
		m_duty1 = 0;
		writeReg(0x06, 0);
		m_duty2 = 0;
		writeReg(0x05, 0);
	}

	return err;
}

int flashEnable_LM3644_1(void)
{
	return 0;
}

int flashDisable_LM3644_1(void)
{
	return 0;
}

int setDuty_LM3644_1(int duty)
{
	int err=0;
//	int led2_duty = 0, duty_Value= 0;
	PK_DBG(" setDuty_LM3644_1 S line=%d\n",__LINE__);
	//mt_set_gpio_out(FLASH_GPIO_EN,GPIO_OUT_ONE);
	if(duty<0)
		duty=0;
	else if(duty>=e_DutyNum)
		duty=e_DutyNum-1;
	m_duty1=duty;

	PK_DBG(" setDuty_LM3644_1 E line=%d\n",__LINE__);
	return err;
}

int init_LM3644(void)
{
	int err;

	PK_DBG(" init_LM3644 S line=%d\n",__LINE__);
	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_HIGH);
	flashlight_gpio_set(FLASHLIGHT_PIN_TORCH, STATE_HIGH);
	err = writeReg(0x01, 0x00);
	err = writeReg(0x08, 0x1F);
	//err =  writeReg(0x11, 0);
	//err =  writeReg(0x12, 0);
	//err =  writeReg(0xC0, 0x52);

	PK_DBG(" init_LM3644 E line=%d\n",__LINE__);
	return err;
}

int FL_Enable(void)
{
	flashEnable_LM3644_1();
	PK_DBG(" FL_Enable line=%d\n", __LINE__);

	return 0;
}

int FL_Disable(void)
{
	flashDisable_LM3644_1();
	PK_DBG(" FL_Disable line=%d\n", __LINE__);
	return 0;
}

int FL_dim_duty(kal_uint32 duty)
{
	setDuty_LM3644_1(duty);

	PK_DBG(" FL_dim_duty line=%d\n", __LINE__);
	return 0;
}

int FL_Init(void)
{
	init_LM3644();

	INIT_WORK(&workTimeOut, work_timeOutFunc);
	PK_DBG(" FL_Init line=%d\n", __LINE__);
	return 0;
}

int FL_Uninit(void)
{
	//FL_Disable();
	flashlight_gpio_set(FLASHLIGHT_PIN_HWEN, STATE_LOW);
	flashlight_gpio_set(FLASHLIGHT_PIN_TORCH, STATE_LOW);
	PK_DBG(" FL_UnInit line=%d\n",__LINE__);
	return 0;
}

/*****************************************************************************
User interface
*****************************************************************************/
static void work_timeOutFunc(struct work_struct *data)
{
	FL_Disable();
	PK_DBG("ledTimeOut_callback\n");
}

enum hrtimer_restart ledTimeOutCallback(struct hrtimer *timer)
{
	schedule_work(&workTimeOut);
	return HRTIMER_NORESTART;
}

static struct hrtimer g_timeOutTimer;
void timerInit(void)
{
	INIT_WORK(&workTimeOut, work_timeOutFunc);
	g_timeOutTimeMs=1000; //1s
	hrtimer_init(&g_timeOutTimer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	g_timeOutTimer.function = ledTimeOutCallback;

}

static int constant_flashlight_ioctl(unsigned int cmd, unsigned long arg)
{
	int i4RetValue = 0;
	int ior_shift;
	int iow_shift;
	int iowr_shift;
	ior_shift = cmd - (_IOR(FLASHLIGHT_MAGIC, 0, int));
	iow_shift = cmd - (_IOW(FLASHLIGHT_MAGIC, 0, int));
	iowr_shift = cmd - (_IOWR(FLASHLIGHT_MAGIC, 0, int));
//	PK_DBG("LM3644 constant_flashlight_ioctl() line=%d ior_shift=%d, iow_shift=%d iowr_shift=%d arg=%d\n",__LINE__, ior_shift, iow_shift, iowr_shift, arg);
	switch(cmd)
	{

	case FLASH_IOC_SET_TIME_OUT_TIME_MS:
//			PK_DBG("FLASH_IOC_SET_TIME_OUT_TIME_MS: %d\n",arg);

		g_timeOutTimeMs = arg;
		break;


	case FLASH_IOC_SET_DUTY:
//   		PK_DBG("FLASHLIGHT_DUTY: %d\n",arg);
		m_duty1 = arg;
		//FL_dim_duty(arg);
		break;


	case FLASH_IOC_SET_STEP:
//   		PK_DBG("FLASH_IOC_SET_STEP: %d\n",arg);

		break;

	case FLASH_IOC_SET_ONOFF:
//   		PK_DBG("FLASHLIGHT_ONOFF: %d\n",arg);
		if(arg==1)
		{
			if(g_timeOutTimeMs!=0)
			{
				ktime_t ktime;
				ktime = ktime_set( 0, g_timeOutTimeMs*1000000 );
				hrtimer_start(&g_timeOutTimer, ktime, HRTIMER_MODE_REL);
			}
			LED1CloseFlag = 0;
			//LED2CloseFlag = 0;
			//FL_Enable();
		}
		else
		{
			LED1CloseFlag = 1;
			//LED2CloseFlag = 1;
			//FL_Disable();
			hrtimer_cancel(&g_timeOutTimer);
		}
		break;

	default:
		PK_DBG(" No such command\n");
		i4RetValue = -EPERM;
		break;
	}
	return i4RetValue;
}

static int constant_flashlight_open(void *pArg)
{
	int i4RetValue = 0;
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	if (0 == strobe_Res)
	{
		FL_Init();
		timerInit();
		/* LED On Status */
		g_strobe_On = TRUE;
	}
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);
	spin_lock_irq(&g_strobeSMPLock);

	if(strobe_Res)
	{
		printk(" busy!\n");
		i4RetValue = -EBUSY;
	}
	else
	{
		strobe_Res += 1;
	}

	spin_unlock_irq(&g_strobeSMPLock);
	PK_DBG("constant_flashlight_open line=%d\n", __LINE__);

	return i4RetValue;
}

static int constant_flashlight_release(void *pArg)
{
	PK_DBG(" constant_flashlight_release\n");

	if (strobe_Res)
	{
		spin_lock_irq(&g_strobeSMPLock);

		strobe_Res = 0;
		strobe_Timeus = 0;

		/* LED On Status */
		g_strobe_On = FALSE;

		spin_unlock_irq(&g_strobeSMPLock);

		FL_Uninit();
	}

	PK_DBG(" Done\n");

	return 0;
}

FLASHLIGHT_FUNCTION_STRUCT	constantFlashlightFunc=
{
	constant_flashlight_open,
	constant_flashlight_release,
	constant_flashlight_ioctl
};

MUINT32 constantFlashlightInit(PFLASHLIGHT_FUNCTION_STRUCT *pfFunc)
{
	if (pfFunc != NULL)
	{
		*pfFunc = &constantFlashlightFunc;
	}
	return 0;
}

/* LED flash control for high current capture mode*/
ssize_t strobe_VDIrq(void)
{
	return 0;
}
EXPORT_SYMBOL(strobe_VDIrq);

/*************aeon add for factory mode flashlight test*********/
int Flashlight_Switch=0;//aeon add for factory mode  flashlight test
static int flag = 1;

void Flashlight_ON(void)
{
	g_ataEnable = 1;
//	FL_dim_duty(0);
	if(0 == strobe_Res)
	{
		FL_Init();
		Flashlight_Switch=0;
	}
	if(flag==1)
	{
		FL_Enable();
		Flashlight_Switch=1;
	}

	init_LM3644();

	flashEnable_LM3644_2();

}

void Flashlight_OFF(void)
{
	g_ataEnable = 2;

	FL_Uninit();
	flashDisable_LM3644_2();
}

EXPORT_SYMBOL(Flashlight_ON);
EXPORT_SYMBOL(Flashlight_OFF);
EXPORT_SYMBOL(Flashlight_Switch);
/**************************end**********************/
