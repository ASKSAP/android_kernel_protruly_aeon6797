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

#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <asm/atomic.h>

#include "kd_camera_typedef.h"
#include "kd_imgsensor.h"
#include "kd_imgsensor_define.h"
#include "kd_camera_feature.h"
#include "kd_camera_hw.h"
/******************************************************************************
 * Debug configuration
******************************************************************************/
#define PFX "[kd_camera_hw]"

//#define DEBUG_CAMERA_HW_K
#ifdef DEBUG_CAMERA_HW_K
#define PK_DBG(fmt, arg...)			pr_debug(PFX fmt, ##arg)
#define PK_ERR(fmt, arg...)         pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...) 		pr_debug(PFX fmt, ##arg)
#else
#define PK_DBG(fmt, arg...)
#define PK_ERR(fmt, arg...)			pr_err(fmt, ##arg)
#define PK_INFO(fmt, arg...)		pr_debug(PFX fmt, ##arg)
#endif


#define IDX_PS_MODE		1
#define IDX_PS_ON		2
#define IDX_PS_OFF		3


#define IDX_PS_CMRST	0
#define IDX_PS_CMPDN	4


#define IDX_AEON_CAMERA_OFFSET 	4
#define IDX_CAMERA_MAX_NUM 		5

extern void ISP_MCLK1_EN(BOOL En);
extern void ISP_MCLK2_EN(BOOL En);
extern void ISP_MCLK3_EN(BOOL En);

// ext. buck fan53526 use gpio 110 which may be used by camera ldo control.
static u32 extbuck_fan53526_exist = 0;

u32 pinSetIdx = 0;		/* default main sensor */
u32 pinSet[IDX_CAMERA_MAX_NUM][8] =
{
	/* for main sensor */
	{
		CAMERA_CMRST_PIN,
		CAMERA_CMRST_PIN_M_GPIO,	/* mode */
		GPIO_OUT_ONE,		/* ON state */
		GPIO_OUT_ZERO,		/* OFF state */
		CAMERA_CMPDN_PIN,
		CAMERA_CMPDN_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
	},
	/* for sub sensor */
	{
		CAMERA_CMRST1_PIN,
		CAMERA_CMRST1_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
		CAMERA_CMPDN1_PIN,
		CAMERA_CMPDN1_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
	},
	/* for Main2 sensor */
	{
		CAMERA_CMRST2_PIN,
		CAMERA_CMRST2_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
		CAMERA_CMPDN2_PIN,
		CAMERA_CMPDN2_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
	},

	/* for main aeon sensor */
	{
		CAMERA_CMRST_PIN,
		CAMERA_CMRST_PIN_M_GPIO,	/* mode */
		GPIO_OUT_ONE,		/* ON state */
		GPIO_OUT_ZERO,		/* OFF state */
		CAMERA_CMPDN_PIN,
		CAMERA_CMPDN_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
	},
	/* for sub aeon sensor */
	{
		CAMERA_CMRST2_PIN,
		CAMERA_CMRST2_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
		CAMERA_CMPDN2_PIN,
		CAMERA_CMPDN2_PIN_M_GPIO,
		GPIO_OUT_ONE,
		GPIO_OUT_ZERO,
	},
};

#ifndef CONFIG_MTK_LEGACY
#define CUST_AVDD AVDD - AVDD
#define CUST_DVDD DVDD - AVDD
#define CUST_DOVDD DOVDD - AVDD
#define CUST_AFVDD AFVDD - AVDD
#define CUST_SUB_AVDD SUB_AVDD - AVDD
#define CUST_SUB_DVDD SUB_DVDD - AVDD
#define CUST_SUB_DOVDD SUB_DOVDD - AVDD
#define CUST_MAIN2_AVDD MAIN2_AVDD - AVDD
#define CUST_MAIN2_DVDD MAIN2_DVDD - AVDD
#define CUST_MAIN2_DOVDD MAIN2_DVDD - AVDD
#endif

PowerCust PowerCustList =
{
	{
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for AVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for DVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for DOVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_Low},	/* for AFVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for SUB_AVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for SUB_DVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for SUB_DOVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for MAIN2_AVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for MAIN2_DVDD; */
		{GPIO_UNSUPPORTED, GPIO_MODE_GPIO, Vol_High},	/* for MAIN2_DOVDD; */
		/* {GPIO_SUPPORTED, GPIO_MODE_GPIO, Vol_Low}, */
	}
};

PowerUp PowerOnList =
{
	{
		/************PQL Camera*******************/
		{
			SENSOR_DRVNAME_OV8865_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{PDN, Vol_Low, 5},
				{RST, Vol_Low, 5},
				{DOVDD, Vol_1800, 5},
				{AVDD, Vol_2800, 5},
				{DVDD, Vol_1200, 5},
				{AFVDD, Vol_2800, 5},
				{PDN, Vol_High, 5},
				{RST, Vol_High, 5}
			},
		},

		{
			SENSOR_DRVNAME_OV8866_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{PDN, Vol_Low, 5},
				{RST, Vol_Low, 5},
				{DOVDD, Vol_1800, 5},
				{AVDD, Vol_2800, 5},
				{DVDD, Vol_1200, 5},
				{AFVDD, Vol_2800, 5},
				{PDN, Vol_High, 5},
				{RST, Vol_High, 5}
			},
		},

		{
			SENSOR_DRVNAME_IMX258_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 1},
				{PDN, Vol_Low, 5},
				{RST, Vol_Low, 5},
				{DOVDD, Vol_1800, 2},
				{AVDD, Vol_2800, 2},
				{DVDD, Vol_1200, 2},
				{AFVDD, Vol_2800, 2},
				{PDN, Vol_High, 5},
				{RST, Vol_High, 5}
			},
		},
 		{
			SENSOR_DRVNAME_IMX259_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 1},
				{PDN, Vol_Low, 5},
				{RST, Vol_Low, 5},
				{DOVDD, Vol_1800, 2},
				{AVDD, Vol_2800, 2},
				{DVDD, Vol_1200, 2},
				{AFVDD, Vol_2800, 2},
				{PDN, Vol_High, 5},
				{RST, Vol_High, 5}
			},
		}, 
		{
			SENSOR_DRVNAME_OV8856_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD, Vol_1200, 10},
				{AFVDD, Vol_2800, 12},
				{RST, Vol_Low, 10},
				{RST, Vol_High, 15},
				{PDN, Vol_Low, 10},
				{PDN, Vol_High, 10},
			},
		},

		{
			SENSOR_DRVNAME_HI843B_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD, Vol_1200, 10},
				{AFVDD, Vol_2800, 12},
				{RST, Vol_Low, 10},
				{RST, Vol_High, 15},
				{PDN, Vol_Low, 10},
				{PDN, Vol_High, 10},
			},
		},

/* 		{
			SENSOR_DRVNAME_S5K3P3SX_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{AFVDD, Vol_2800, 5},
				{PDN, Vol_Low, 4},
				{PDN, Vol_High, 0},
				{RST, Vol_Low, 1},
				{RST, Vol_High, 0},
			},
		}, */
		{
			SENSOR_DRVNAME_S5K5E2YA_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{AFVDD, Vol_2800, 5},
				{PDN, Vol_Low, 4},
				{PDN, Vol_High, 0},
				{RST, Vol_Low, 1},
				{RST, Vol_High, 0},
			},
		},
		{
			SENSOR_DRVNAME_S5K2P8_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{AFVDD, Vol_2800, 5},
				{PDN, Vol_Low, 4},
				{PDN, Vol_High, 0},
				{RST, Vol_Low, 1},
				{RST, Vol_High, 0},
			},
		},
		{
			SENSOR_DRVNAME_S5K2X8_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{PDN, Vol_Low, 0},
				{RST, Vol_Low, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{AFVDD, Vol_2800, 1},
				{PDN, Vol_High, 0},
				{RST, Vol_High, 0}
			},
		},
		{
			SENSOR_DRVNAME_OV13858_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{PDN, Vol_Low, 10},
				{RST, Vol_Low, 10},
				{DVDD, Vol_1200, 10},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},				
				{AFVDD, Vol_2800, 10},
				{PDN, Vol_High, 10},
				{RST, Vol_High, 10}
			},
		},
		{
			SENSOR_DRVNAME_IMX299_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 1},
				{PDN, Vol_Low, 5},
				{RST, Vol_Low, 5},
				{DOVDD, Vol_1800, 2},
				{AVDD, Vol_2800, 2},
				{DVDD, Vol_1200, 2},
				{AFVDD, Vol_2800, 2},
				{PDN, Vol_High, 5},
				{RST, Vol_High, 5}
			},
		},
		{
			SENSOR_DRVNAME_IMX298_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 10},
				{PDN, Vol_Low, 10},
				{RST, Vol_Low, 10},
				{DOVDD, Vol_1800, 10},
				{AVDD, Vol_2800, 10},
				{DVDD, Vol_1200, 10},
				{AFVDD, Vol_2800, 10},
				{PDN, Vol_High, 10},
				{RST, Vol_High, 10}
			},
		},
		{
			SENSOR_DRVNAME_IMX230_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{AVDD, Vol_2800, 10},
				{DOVDD, Vol_1800, 10},
				{DVDD, Vol_1200, 10},
				{AFVDD, Vol_2800, 5},
				{PDN, Vol_Low, 0},
				{PDN, Vol_High, 0},
				{RST, Vol_Low, 0},
				{RST, Vol_High, 0}
			},
		},

		{
			SENSOR_DRVNAME_S5K3P8SP_MIPI_RAW,
			{
				{SensorMCLK, Vol_High, 0},
				{PDN, Vol_Low, 0},
				{RST, Vol_Low, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{AFVDD, Vol_2800, 1},
				{PDN, Vol_High, 0},
				{RST, Vol_High, 0}
			},
		},

		{
			SENSOR_DRVNAME_S5K3L8_MIPI_RAW_SUNRISE,
			{
				{SensorMCLK, Vol_High, 0},
				{PDN, Vol_Low, 0},
				{RST, Vol_Low, 0},
				{DOVDD, Vol_1800, 0},
				{AVDD, Vol_2800, 0},
				{DVDD, Vol_1200, 0},
				{AFVDD, Vol_2800, 1},
				{PDN, Vol_High, 0},
				{RST, Vol_High, 0}
			},
		},
		/* add new sensor before this line */
		{NULL,},
	}
};

#define VOL_2800 2800000
#define VOL_1800 1800000
#define VOL_1500 1500000
#define VOL_1200 1200000
#define VOL_1220 1220000
#define VOL_1000 1000000

/* GPIO Pin control*/
struct platform_device *cam_plt_dev = NULL;
struct pinctrl *camctrl = NULL;
struct pinctrl_state *cam0_pnd_h = NULL;/* main cam */
struct pinctrl_state *cam0_pnd_l = NULL;
struct pinctrl_state *cam0_rst_h = NULL;
struct pinctrl_state *cam0_rst_l = NULL;
struct pinctrl_state *cam1_pnd_h = NULL;/* sub cam */
struct pinctrl_state *cam1_pnd_l = NULL;
struct pinctrl_state *cam1_rst_h = NULL;
struct pinctrl_state *cam1_rst_l = NULL;
struct pinctrl_state *cam2_pnd_h = NULL;/* main2 cam */
struct pinctrl_state *cam2_pnd_l = NULL;
struct pinctrl_state *cam2_rst_h = NULL;
struct pinctrl_state *cam2_rst_l = NULL;
struct pinctrl_state *cam_ldo_normal_main_vcama_h = NULL;/* for AVDD */
struct pinctrl_state *cam_ldo_normal_main_vcama_l = NULL;
struct pinctrl_state *cam_ldo_fish_main_vcamd_h = NULL;/* for DVDD */
struct pinctrl_state *cam_ldo_fish_main_vcamd_l = NULL;
struct pinctrl_state *cam_ldo_normal_fish_vcama_h = NULL;/* for AVDD */
struct pinctrl_state *cam_ldo_normal_fish_vcama_l = NULL;
struct pinctrl_state *cam_ldo_fish_sub_vcamd_h = NULL;/* for DVDD */
struct pinctrl_state *cam_ldo_fish_sub_vcamd_l = NULL;
struct pinctrl_state *cam_ldo_vcamio_h = NULL;/* for DOVDD */
struct pinctrl_state *cam_ldo_vcamio_l = NULL;
struct pinctrl_state *cam_ldo_vcamaf_h = NULL;/* for AFVDD */
struct pinctrl_state *cam_ldo_vcamaf_l = NULL;
struct pinctrl_state *cam_ldo_sub_vcamd_h = NULL;/* for SUB_DVDD */
struct pinctrl_state *cam_ldo_sub_vcamd_l = NULL;
struct pinctrl_state *cam_ldo_main2_vcamd_h = NULL;/* for MAIN2_DVDD */
struct pinctrl_state *cam_ldo_main2_vcamd_l = NULL;
struct pinctrl_state *cam_mipi_switch_en_h = NULL;/* for mipi switch */
struct pinctrl_state *cam_mipi_switch_en_l = NULL;
struct pinctrl_state *cam_mipi_switch_sel_h = NULL;/* for mipi switch */
struct pinctrl_state *cam_mipi_switch_sel_l = NULL;
struct pinctrl_state *cam3_pnd_h = NULL;/* main cam */
struct pinctrl_state *cam3_pnd_l = NULL;
struct pinctrl_state *cam3_rst_h = NULL;
struct pinctrl_state *cam3_rst_l = NULL;

int mtkcam_gpio_init(struct platform_device *pdev)
{
	int ret = 0;

	camctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(camctrl))
	{
		dev_err(&pdev->dev, "Cannot find camera pinctrl!");
		ret = PTR_ERR(camctrl);
	}
	/*Cam0 Power/Rst Ping initialization */
	cam0_pnd_h = pinctrl_lookup_state(camctrl, "cam0_pnd1");
	if (IS_ERR(cam0_pnd_h))
	{
		ret = PTR_ERR(cam0_pnd_h);
		PK_ERR("%s : pinctrl err, cam0_pnd_h\n", __func__);
	}

	cam0_pnd_l = pinctrl_lookup_state(camctrl, "cam0_pnd0");
	if (IS_ERR(cam0_pnd_l))
	{
		ret = PTR_ERR(cam0_pnd_l);
		PK_ERR("%s : pinctrl err, cam0_pnd_l\n", __func__);
	}

	cam0_rst_h = pinctrl_lookup_state(camctrl, "cam0_rst1");
	if (IS_ERR(cam0_rst_h))
	{
		ret = PTR_ERR(cam0_rst_h);
		PK_ERR("%s : pinctrl err, cam0_rst_h\n", __func__);
	}

	cam0_rst_l = pinctrl_lookup_state(camctrl, "cam0_rst0");
	if (IS_ERR(cam0_rst_l))
	{
		ret = PTR_ERR(cam0_rst_l);
		PK_ERR("%s : pinctrl err, cam0_rst_l\n", __func__);
	}

	/*Cam1 Power/Rst Ping initialization */
	cam1_pnd_h = pinctrl_lookup_state(camctrl, "cam1_pnd1");
	if (IS_ERR(cam1_pnd_h))
	{
		ret = PTR_ERR(cam1_pnd_h);
		PK_ERR("%s : pinctrl err, cam1_pnd_h\n", __func__);
	}

	cam1_pnd_l = pinctrl_lookup_state(camctrl, "cam1_pnd0");
	if (IS_ERR(cam1_pnd_l))
	{
		ret = PTR_ERR(cam1_pnd_l);
		PK_ERR("%s : pinctrl err, cam1_pnd_l\n", __func__);
	}

	cam1_rst_h = pinctrl_lookup_state(camctrl, "cam1_rst1");
	if (IS_ERR(cam1_rst_h))
	{
		ret = PTR_ERR(cam1_rst_h);
		PK_ERR("%s : pinctrl err, cam1_rst_h\n", __func__);
	}

	cam1_rst_l = pinctrl_lookup_state(camctrl, "cam1_rst0");
	if (IS_ERR(cam1_rst_l))
	{
		ret = PTR_ERR(cam1_rst_l);
		PK_ERR("%s : pinctrl err, cam1_rst_l\n", __func__);
	}

	cam2_pnd_h = pinctrl_lookup_state(camctrl, "cam2_pnd1");
	if (IS_ERR(cam2_pnd_h))
	{
		ret = PTR_ERR(cam2_pnd_h);
		PK_ERR("%s : pinctrl err, cam2_pnd_h\n", __func__);
	}

	cam2_pnd_l = pinctrl_lookup_state(camctrl, "cam2_pnd0");
	if (IS_ERR(cam2_pnd_l))
	{
		ret = PTR_ERR(cam2_pnd_l);
		PK_ERR("%s : pinctrl err, cam2_pnd_l\n", __func__);
	}

	cam2_rst_h = pinctrl_lookup_state(camctrl, "cam2_rst1");
	if (IS_ERR(cam2_rst_h))
	{
		ret = PTR_ERR(cam2_rst_h);
		PK_ERR("%s : pinctrl err, cam2_rst_h\n", __func__);
	}

	cam2_rst_l = pinctrl_lookup_state(camctrl, "cam2_rst0");
	if (IS_ERR(cam2_rst_l))
	{
		ret = PTR_ERR(cam2_rst_l);
		PK_ERR("%s : pinctrl err, cam2_rst_l\n", __func__);
	}

	/*externel LDO enable */
	/*GPIO 135 for MAIN CAM AVDD*/
	cam_ldo_normal_main_vcama_h = pinctrl_lookup_state(camctrl, "cam_ldo_vcama_1");
	if (IS_ERR(cam_ldo_normal_main_vcama_h))
	{
		ret = PTR_ERR(cam_ldo_normal_main_vcama_h);
		PK_ERR("%s : pinctrl err, cam_ldo_normal_main_vcama_h\n", __func__);
	}

	cam_ldo_normal_main_vcama_l = pinctrl_lookup_state(camctrl, "cam_ldo_vcama_0");
	if (IS_ERR(cam_ldo_normal_main_vcama_l))
	{
		ret = PTR_ERR(cam_ldo_normal_main_vcama_l);
		PK_ERR("%s : pinctrl err, cam_ldo_normal_main_vcama_l\n", __func__);
	}

	/*externel LDO enable */
	/*GPIO 137 for MAIN CAM DVDD*/
	cam_ldo_fish_main_vcamd_h = pinctrl_lookup_state(camctrl, "cam_ldo_vcamd_1");
	if (IS_ERR(cam_ldo_fish_main_vcamd_h))
	{
		ret = PTR_ERR(cam_ldo_fish_main_vcamd_h);
		PK_ERR("%s : pinctrl err, cam_ldo_fish_main_vcamd_h\n", __func__);
	}

	cam_ldo_fish_main_vcamd_l = pinctrl_lookup_state(camctrl, "cam_ldo_vcamd_0");
	if (IS_ERR(cam_ldo_fish_main_vcamd_l))
	{
		ret = PTR_ERR(cam_ldo_fish_main_vcamd_l);
		PK_ERR("%s : pinctrl err, cam_ldo_fish_main_vcamd_l\n", __func__);
	}

	/*externel LDO enable */
	/*GPIO 140 for MAIN CAM AVDD*/
	cam_ldo_normal_fish_vcama_h = pinctrl_lookup_state(camctrl, "cam2_ldo_vcama_1");
	if (IS_ERR(cam_ldo_normal_fish_vcama_h))
	{
		ret = PTR_ERR(cam_ldo_normal_fish_vcama_h);
		PK_ERR("%s : pinctrl err, cam_ldo_normal_fish_vcama_h\n", __func__);
	}

	cam_ldo_normal_fish_vcama_l = pinctrl_lookup_state(camctrl, "cam2_ldo_vcama_0");
	if (IS_ERR(cam_ldo_normal_fish_vcama_l))
	{
		ret = PTR_ERR(cam_ldo_normal_fish_vcama_l);
		PK_ERR("%s : pinctrl err, cam_ldo_normal_fish_vcama_l\n", __func__);
	}

	/*externel LDO enable */
	/*GPIO 136 for MAIN CAM DVDD*/
	cam_ldo_fish_sub_vcamd_h = pinctrl_lookup_state(camctrl, "cam2_ldo_vcamd_1");
	if (IS_ERR(cam_ldo_fish_sub_vcamd_h))
	{
		ret = PTR_ERR(cam_ldo_fish_sub_vcamd_h);
		PK_ERR("%s : pinctrl err, cam_ldo_fish_sub_vcamd_h\n", __func__);
	}

	cam_ldo_fish_sub_vcamd_l = pinctrl_lookup_state(camctrl, "cam2_ldo_vcamd_0");
	if (IS_ERR(cam_ldo_fish_sub_vcamd_l))
	{
		ret = PTR_ERR(cam_ldo_fish_sub_vcamd_l);
		PK_ERR("%s : pinctrl err, cam_ldo_fish_sub_vcamd_l\n", __func__);
	}

	/*mipi switch enable */
	cam_mipi_switch_en_h = pinctrl_lookup_state(camctrl, "cam_mipi_switch_en_1");
	if (IS_ERR(cam_mipi_switch_en_h))
	{
		ret = PTR_ERR(cam_mipi_switch_en_h);
		PK_ERR("%s : pinctrl err, cam_mipi_switch_en_1\n", __func__);
	}

	cam_mipi_switch_en_l = pinctrl_lookup_state(camctrl, "cam_mipi_switch_en_0");
	if (IS_ERR(cam_mipi_switch_en_l))
	{
		ret = PTR_ERR(cam_mipi_switch_en_l);
		PK_ERR("%s : pinctrl err, cam_mipi_switch_en_0\n", __func__);
	}

	/*mipi switch sel */
	cam_mipi_switch_sel_h = pinctrl_lookup_state(camctrl, "cam_mipi_switch_sel_1");
	if (IS_ERR(cam_mipi_switch_sel_h))
	{
		ret = PTR_ERR(cam_mipi_switch_sel_h);
		PK_ERR("%s : pinctrl err, cam_mipi_switch_sel_1\n", __func__);
	}

	cam_mipi_switch_sel_l = pinctrl_lookup_state(camctrl, "cam_mipi_switch_sel_0");
	if (IS_ERR(cam_mipi_switch_sel_l))
	{
		ret = PTR_ERR(cam_mipi_switch_sel_l);
		PK_ERR("%s : pinctrl err, cam_mipi_switch_sel_0\n", __func__);
	}

	cam3_pnd_h = pinctrl_lookup_state(camctrl, "cam3_pnd1");
	if (IS_ERR(cam3_pnd_h))
	{
		ret = PTR_ERR(cam3_pnd_h);
		PK_ERR("%s : pinctrl err, cam3_pnd_h\n", __func__);
	}

	cam3_pnd_l = pinctrl_lookup_state(camctrl, "cam3_pnd0");
	if (IS_ERR(cam3_pnd_l))
	{
		ret = PTR_ERR(cam3_pnd_l);
		PK_ERR("%s : pinctrl err, cam3_pnd_l\n", __func__);
	}

	cam3_rst_h = pinctrl_lookup_state(camctrl, "cam3_rst1");
	if (IS_ERR(cam3_rst_h))
	{
		ret = PTR_ERR(cam0_rst_h);
		PK_ERR("%s : pinctrl err, cam3_rst_h\n", __func__);
	}

	cam3_rst_l = pinctrl_lookup_state(camctrl, "cam3_rst0");
	if (IS_ERR(cam3_rst_l))
	{
		ret = PTR_ERR(cam3_rst_l);
		PK_ERR("%s : pinctrl err, cam3_rst_l\n", __func__);
	}

	return ret;
}

int mtkcam_gpio_set(int PinIdx, int PwrType, int Val)
{
	int ret = 0;
	static signed int mAVDD0_usercounter = 0;
	static signed int mAVDD1_usercounter = 0;
	static signed int mDVDD0_usercounter = 0;
	static signed int mDVDD1_usercounter = 0;

	if (IS_ERR(camctrl))
	{
		return -1;
	}

	switch (PwrType)
	{
	case RST:
		if (PinIdx == 3)
		{
			if (Val == 0 && !IS_ERR(cam0_rst_l))
				pinctrl_select_state(camctrl, cam0_rst_l);
			else if (Val == 1 && !IS_ERR(cam0_rst_h))
				pinctrl_select_state(camctrl, cam0_rst_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		}
		else if (PinIdx == 1)
		{
			if (Val == 0 && !IS_ERR(cam2_rst_l))
				pinctrl_select_state(camctrl, cam2_rst_l);
			else if (Val == 1 && !IS_ERR(cam2_rst_h))
				pinctrl_select_state(camctrl, cam2_rst_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		}
		else if (PinIdx == 0)
		{
			if (Val == 0 && !IS_ERR(cam3_rst_l))
				pinctrl_select_state(camctrl, cam3_rst_l);
			else if (Val == 1 && !IS_ERR(cam3_rst_h))
				pinctrl_select_state(camctrl, cam3_rst_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		}
		else if (PinIdx == 4)
		{
			if (Val == 0 && !IS_ERR(cam1_rst_l))
				pinctrl_select_state(camctrl, cam1_rst_l);
			else if (Val == 1 && !IS_ERR(cam1_rst_h))
				pinctrl_select_state(camctrl, cam1_rst_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		}
		else
		{
			if (Val == 0 && !IS_ERR(cam2_rst_l))
				pinctrl_select_state(camctrl, cam2_rst_l);
			else if (Val == 1 && !IS_ERR(cam2_rst_h))
				pinctrl_select_state(camctrl, cam2_rst_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, RST\n", __func__,PinIdx ,Val);
		}
		break;
	case PDN:
		if (PinIdx == 3)
		{
			if (Val == 0 && !IS_ERR(cam0_pnd_l))
				pinctrl_select_state(camctrl, cam0_pnd_l);
			else if (Val == 1 && !IS_ERR(cam0_pnd_h))
				pinctrl_select_state(camctrl, cam0_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		}
		else if (PinIdx == 1)
		{
			if (Val == 0 && !IS_ERR(cam2_pnd_l))
				pinctrl_select_state(camctrl, cam2_pnd_l);
			else if (Val == 1 && !IS_ERR(cam2_pnd_h))
				pinctrl_select_state(camctrl, cam2_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		}
		else if (PinIdx == 0)
		{
			if (Val == 0 && !IS_ERR(cam3_pnd_l))
				pinctrl_select_state(camctrl, cam3_pnd_l);
			else if (Val == 1 && !IS_ERR(cam3_pnd_h))
				pinctrl_select_state(camctrl, cam3_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		}
		else if (PinIdx == 4)
		{
			if (Val == 0 && !IS_ERR(cam1_pnd_l))
				pinctrl_select_state(camctrl, cam1_pnd_l);
			else if (Val == 1 && !IS_ERR(cam1_pnd_h))
				pinctrl_select_state(camctrl, cam1_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		}
		else
		{
			if (Val == 0 && !IS_ERR(cam2_pnd_l))
				pinctrl_select_state(camctrl, cam2_pnd_l);
			else if (Val == 1 && !IS_ERR(cam2_pnd_h))
				pinctrl_select_state(camctrl, cam2_pnd_h);
			else
				PK_ERR("%s : pinctrl err, PinIdx %d, Val %d, PDN\n", __func__,PinIdx ,Val);
		}
		break;
		
	case AVDD:
	case MAIN2_AVDD:
		/*Main & Main2 use same cotrol GPIO */
		//cam_ldo_fish_main_vcamd_h  FISH MAIN
		//cam_ldo_fish_sub_vcamd_h)  FISH SUB
		//cam_ldo_normal_main_vcama_h NORMAL MAIN
		//cam_ldo_normal_fish_vcama_h)  FISH MAIN
		if (PinIdx == 3){		
			PK_DBG("mAVDD0_usercounter(%d)\n",mAVDD0_usercounter);
			if (Val == 0 && !IS_ERR(cam_ldo_normal_main_vcama_l))
			{
				mAVDD0_usercounter --;
				if(mAVDD0_usercounter <= 0)
				{
					if(mAVDD0_usercounter < 0)
						PK_ERR("Please check AVDD pin control\n");

					mAVDD0_usercounter = 0;
					pinctrl_select_state(camctrl, cam_ldo_normal_main_vcama_l);
				}

			}
			else if (Val == 1 && !IS_ERR(cam_ldo_normal_main_vcama_h))
			{
				mAVDD0_usercounter ++;
				pinctrl_select_state(camctrl, cam_ldo_normal_main_vcama_h);
			}
		}
		else if( (PinIdx == 0) || (PinIdx == 1)){
			if (Val == 0 && !IS_ERR(cam_ldo_normal_fish_vcama_l))
			{
				mAVDD1_usercounter --;
				if(mAVDD1_usercounter <= 0)
				{
					mAVDD1_usercounter = 0;
					pinctrl_select_state(camctrl, cam_ldo_normal_fish_vcama_l);
				}

			}
			else if (Val == 1 && !IS_ERR(cam_ldo_normal_fish_vcama_h))
			{
				mAVDD1_usercounter ++;
				pinctrl_select_state(camctrl, cam_ldo_normal_fish_vcama_h);
			}
		}
		break;
		
	case DVDD:
	case MAIN2_DVDD:
		PK_DBG("mDVDD0_usercounter(%d)\n",mDVDD0_usercounter);
		//cam_ldo_fish_main_vcamd_h  FISH MAIN
		//cam_ldo_fish_sub_vcamd_h)  FISH SUB
		if (PinIdx == 0){
			if (Val == 0 && !IS_ERR(cam_ldo_fish_main_vcamd_l))
			{
				mDVDD0_usercounter --;
				if(mDVDD0_usercounter <= 0)
				{
					mDVDD0_usercounter = 0;
					pinctrl_select_state(camctrl, cam_ldo_fish_main_vcamd_l);
				}
			}
			else if (Val == 1 && !IS_ERR(cam_ldo_fish_main_vcamd_h))
			{
				mDVDD0_usercounter ++;
				pinctrl_select_state(camctrl, cam_ldo_fish_main_vcamd_h);
			}
		}
		else if (PinIdx == 1){
			if (Val == 0 && !IS_ERR(cam_ldo_fish_sub_vcamd_l))
			{
				mDVDD1_usercounter --;
				if(mDVDD1_usercounter <= 0)
				{
					mDVDD1_usercounter = 0;
					pinctrl_select_state(camctrl, cam_ldo_fish_sub_vcamd_l);
				}
			}
			else if (Val == 1 && !IS_ERR(cam_ldo_fish_sub_vcamd_h))
			{
				mDVDD1_usercounter ++;
				pinctrl_select_state(camctrl, cam_ldo_fish_sub_vcamd_h);
			}
		}
		break;
		
	case DOVDD:
	case AFVDD:
	case SUB_AVDD:
		if (PinIdx == 1){
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_ldo_normal_fish_vcama_l);
			else
				pinctrl_select_state(camctrl, cam_ldo_normal_fish_vcama_h);
		}
		break;
                
	case SUB_DVDD:
		if (PinIdx == 0){
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_ldo_fish_main_vcamd_l);
			else
				pinctrl_select_state(camctrl, cam_ldo_fish_main_vcamd_h);
		}
		else if (PinIdx == 1){
			if (Val == 0)
				pinctrl_select_state(camctrl, cam_ldo_fish_sub_vcamd_l);
			else
				pinctrl_select_state(camctrl, cam_ldo_fish_sub_vcamd_h);
		}
		break;
	case MAIN_LDO_AVDD:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo_normal_main_vcama_l);
		else
			pinctrl_select_state(camctrl, cam_ldo_normal_main_vcama_h);
		break;
	case MAIN_LDO_DVDD:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo_fish_main_vcamd_l);
		else
			pinctrl_select_state(camctrl, cam_ldo_fish_main_vcamd_h);
		break;
	case MAIN2_LDO_AVDD:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo_normal_fish_vcama_l);
		else
			pinctrl_select_state(camctrl, cam_ldo_normal_fish_vcama_h);
		break;
	case MAIN2_LDO_DVDD:
		if (Val == 0)
			pinctrl_select_state(camctrl, cam_ldo_fish_sub_vcamd_l);
		else
			pinctrl_select_state(camctrl, cam_ldo_fish_sub_vcamd_h);
		break;
	default:
		PK_ERR("PwrType(%d) is invalid !!\n", PwrType);
		break;
	};

	PK_DBG("PinIdx(%d) PwrType(%d) val(%d)\n", PinIdx, PwrType, Val);

	return ret;
}

BOOL hwpoweron(PowerInformation pwInfo, char *mode_name)
{
	if (pwInfo.PowerType == AVDD)
	{
		if (pinSetIdx == 2)
		{
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, MAIN2_AVDD, PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Voltage))
				{
					PK_INFO("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}
		else if((pinSetIdx == 0) || (pinSetIdx == 1) ||(pinSetIdx == 3) )
		{
			if (mtkcam_gpio_set(pinSetIdx, AVDD, PowerCustList.PowerCustInfo[CUST_AVDD].Voltage))
			{
				PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
			}
		}
		else if(pinSetIdx == 4)
		{
			if (PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, SUB_AVDD, PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}
		else
		{
			if (PowerCustList.PowerCustInfo[CUST_AVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, AVDD, PowerCustList.PowerCustInfo[CUST_AVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}
	}
	else if (pwInfo.PowerType == DVDD)
	{
		if (pinSetIdx == 2)
		{
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (pwInfo.Voltage == Vol_1200)
				{
					pwInfo.Voltage = Vol_1210;
				}
				if (TRUE != _hwPowerOn(DVDD, pwInfo.Voltage))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, MAIN2_DVDD, PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_MAIN2_DVDD] set gpio failed!!\n");
				}
			}
		}
		else if ( (pinSetIdx == 0)||(pinSetIdx == 1))
		{
			if (mtkcam_gpio_set(pinSetIdx, DVDD, PowerCustList.PowerCustInfo[CUST_DVDD].Voltage))
			{
				PK_ERR("[CAMERA CUST_DVDD] set gpio failed!!\n");
			}
		}
		else if (pinSetIdx == 4)
		{
			if (PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerOn(DVDD, pwInfo.Voltage))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, SUB_DVDD, PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_SUB_DVDD] set gpio failed!!\n");
				}
			}
		}
		else if (pinSetIdx == 3)
		{
			if (PowerCustList.PowerCustInfo[CUST_DVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (pwInfo.Voltage == Vol_1200)
				{
					pwInfo.Voltage = Vol_1210;
					//PK_INFO("[CAMERA SENSOR] Sub camera VCAM_D power 1.2V to 1.21V\n");
				}
				if (TRUE != _hwPowerOn(DVDD, pwInfo.Voltage))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, DVDD, PowerCustList.PowerCustInfo[CUST_DVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_DVDD] set gpio failed!!\n");
				}
			}
		}
		else
		{
			if (PowerCustList.PowerCustInfo[CUST_DVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (pwInfo.Voltage == Vol_1200)
				{
					pwInfo.Voltage = Vol_1210;
					//PK_INFO("[CAMERA SENSOR] Sub camera VCAM_D power 1.2V to 1.21V\n");
				}
				if (TRUE != _hwPowerOn(DVDD, pwInfo.Voltage))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, DVDD, PowerCustList.PowerCustInfo[CUST_DVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_DVDD] set gpio failed!!\n");
				}
			}
		}
	}
	else if (pwInfo.PowerType == DOVDD)
	{
		if (PowerCustList.PowerCustInfo[CUST_DOVDD].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage))
			{
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		}
		else
		{
			if (mtkcam_gpio_set(pinSetIdx, DOVDD, PowerCustList.PowerCustInfo[CUST_DOVDD].Voltage))
			{
				PK_ERR("[CAMERA CUST_DOVDD] set gpio failed!!\n");
			}
		}
	}
	else if (pwInfo.PowerType == AFVDD)
	{
		if (PowerCustList.PowerCustInfo[CUST_AFVDD].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if (TRUE != _hwPowerOn(pwInfo.PowerType, pwInfo.Voltage))
			{
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		}
		else
		{
			if (mtkcam_gpio_set(pinSetIdx, AFVDD, PowerCustList.PowerCustInfo[CUST_AFVDD].Voltage))
			{
				PK_ERR("[CAMERA CUST_DOVDD] set gpio failed!!\n");
			}
		}
	}
	else if (pwInfo.PowerType == PDN)
	{
		if (pwInfo.Voltage == Vol_High)
		{
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]))
			{
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
		else
		{
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
			{

				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	}
	else if (pwInfo.PowerType == RST)
	{
		if (pwInfo.Voltage == Vol_High)
		{
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]))
			{
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
		else
		{
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
			{

				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	}
	else if (pwInfo.PowerType == SensorMCLK)
	{
		if (pinSetIdx == 0)
		{
			ISP_MCLK1_EN(TRUE);
		}
		else if (pinSetIdx == 1)
		{
			ISP_MCLK3_EN(TRUE);
		}
		else if (pinSetIdx == 3)
		{
			ISP_MCLK1_EN(TRUE);
		}
		else if (pinSetIdx == 4)
		{
			ISP_MCLK2_EN(TRUE);
		}
		else
		{
			ISP_MCLK3_EN(TRUE);
		}
	}
	else
	{
	}

	if (pwInfo.Delay > 0)
		mdelay(pwInfo.Delay);
	return TRUE;
}

BOOL hwpowerdown(PowerInformation pwInfo, char *mode_name)
{
	if (pwInfo.PowerType == AVDD)
	{
		if (pinSetIdx == 2)
		{
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerDown(pwInfo.PowerType))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, MAIN2_AVDD, 1-PowerCustList.PowerCustInfo[CUST_MAIN2_AVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}
		else if( (pinSetIdx == 0) || (pinSetIdx == 1) || (pinSetIdx == 3))
		{
			if (mtkcam_gpio_set(pinSetIdx, AVDD, 1-PowerCustList.PowerCustInfo[CUST_AVDD].Voltage))
			{
				PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
			}
		}
		else if(pinSetIdx == 4)
		{
			if (PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerDown(pwInfo.PowerType))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, SUB_AVDD, 1-PowerCustList.PowerCustInfo[CUST_SUB_AVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}
		else
		{
			if (PowerCustList.PowerCustInfo[CUST_AVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerDown(pwInfo.PowerType))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, AVDD, 1-PowerCustList.PowerCustInfo[CUST_AVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");
				}
			}
		}
	}
	else if (pwInfo.PowerType == DVDD)
	{
		if (pinSetIdx == 2)
		{
			if (PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerDown(DVDD))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, MAIN2_DVDD, 1-PowerCustList.PowerCustInfo[CUST_MAIN2_DVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_MAIN2_DVDD] set gpio failed!!\n");
				}
			}
		}
		else if ( (pinSetIdx == 0) || (pinSetIdx == 1))
		{
			if (mtkcam_gpio_set(pinSetIdx, DVDD, 1-PowerCustList.PowerCustInfo[CUST_DVDD].Voltage))
			{
				PK_ERR("[CAMERA CUST_SUB_DVDD] set gpio failed!!\n");
			}
		}
		else if (pinSetIdx == 4)
		{
			if (PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerDown(DVDD))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, SUB_DVDD, 1-PowerCustList.PowerCustInfo[CUST_SUB_DVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_SUB_DVDD] set gpio failed!!\n");
				}
			}
		}
		else if (pinSetIdx == 3)
		{
			if (PowerCustList.PowerCustInfo[CUST_DVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerDown(DVDD))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, DVDD, 1-PowerCustList.PowerCustInfo[CUST_DVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_DVDD] set gpio failed!!\n");
				}
			}
		}
		else
		{
			if (PowerCustList.PowerCustInfo[CUST_DVDD].Gpio_Pin == GPIO_UNSUPPORTED)
			{
				if (TRUE != _hwPowerDown(DVDD))
				{
					PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
					return FALSE;
				}
			}
			else
			{
				if (mtkcam_gpio_set(pinSetIdx, DVDD, 1-PowerCustList.PowerCustInfo[CUST_DVDD].Voltage))
				{
					PK_ERR("[CAMERA CUST_DVDD] set gpio failed!!\n");
				}
			}
		}
	}
	else if (pwInfo.PowerType == DOVDD)
	{
		if (PowerCustList.PowerCustInfo[CUST_DOVDD].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if (TRUE != _hwPowerDown(pwInfo.PowerType))
			{
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		}
		else
		{
			if (mtkcam_gpio_set(pinSetIdx, DOVDD, 1-PowerCustList.PowerCustInfo[CUST_DOVDD].Voltage))
			{
				PK_ERR("[CAMERA CUST_AVDD] set gpio failed!!\n");/* 1-voltage for reverse*/
			}
		}
	}
	else if (pwInfo.PowerType == AFVDD)
	{
		if (PowerCustList.PowerCustInfo[CUST_AFVDD].Gpio_Pin == GPIO_UNSUPPORTED)
		{
			if (TRUE != _hwPowerDown(pwInfo.PowerType))
			{
				PK_ERR("[CAMERA SENSOR] Fail to enable digital power\n");
				return FALSE;
			}
		}
		else
		{
			if (mtkcam_gpio_set(pinSetIdx, AFVDD, 1-PowerCustList.PowerCustInfo[CUST_AFVDD].Voltage))
			{
				PK_ERR("[CAMERA CUST_DOVDD] set gpio failed!!\n");/* 1-voltage for reverse*/
			}
		}
	}
	else if (pwInfo.PowerType == PDN)
	{
		if (pwInfo.Voltage == Vol_High)
		{
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_ON]))
			{
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
		else
		{
			if (mtkcam_gpio_set(pinSetIdx, PDN, pinSet[pinSetIdx][IDX_PS_CMPDN + IDX_PS_OFF]))
			{

				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	}
	else if (pwInfo.PowerType == RST)
	{
		if (pwInfo.Voltage == Vol_High)
		{
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_ON]))
			{
				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
		else
		{
			if (mtkcam_gpio_set(pinSetIdx, RST, pinSet[pinSetIdx][IDX_PS_CMRST + IDX_PS_OFF]))
			{

				PK_ERR("[CAMERA SENSOR] set gpio failed!!\n");
			}
		}
	}
	else if (pwInfo.PowerType == SensorMCLK)
	{
		if (pinSetIdx == 0)
		{
			ISP_MCLK1_EN(FALSE);
		}
		else if (pinSetIdx == 1)
		{
			ISP_MCLK3_EN(FALSE);
		}
		else if (pinSetIdx == 3)
		{
			ISP_MCLK1_EN(FALSE);
		}
		else if (pinSetIdx == 4)
		{
			ISP_MCLK2_EN(FALSE);
		}
		else
		{
			ISP_MCLK3_EN(FALSE);
		}
	}
	else
	{
	}
	return TRUE;
}

//#define CUSTOM_DUMMY_BQL_SENSOR
#define CUSTOM_SENSOR_GROUP_SWAP

static int sensorPowerStatus[4]={0,0,0,0};

int kdSensorPowerStatus(int SensorIdx)
{
	return sensorPowerStatus[SensorIdx];
}

int kdCISModulePowerOn(CAMERA_DUAL_CAMERA_SENSOR_ENUM SensorIdx, char *currSensorName, BOOL On,
                       char *mode_name)
{
	int pwListIdx, pwIdx;
	BOOL sensorInPowerList = KAL_FALSE;
	
	if (DUAL_CAMERA_MAIN_SENSOR == SensorIdx){
#ifdef CUSTOM_SENSOR_GROUP_SWAP
#ifdef CUSTOM_DUMMY_BQL_SENSOR
		pinSetIdx = 0;
#else
		pinSetIdx = 3;
#endif
#else
		pinSetIdx = 0;
 #endif
	}
	else if (DUAL_CAMERA_SUB_SENSOR == SensorIdx)
	{
#ifdef CUSTOM_SENSOR_GROUP_SWAP
#ifdef CUSTOM_DUMMY_BQL_SENSOR
		pinSetIdx = 1;
#else
		pinSetIdx = 4;
#endif
#else
		pinSetIdx = 1;
 #endif
	}
	else if (DUAL_CAMERA_MAIN_3_SENSOR == SensorIdx)
	{
#ifdef CUSTOM_SENSOR_GROUP_SWAP
		pinSetIdx = 0;
#else
#ifdef CUSTOM_DUMMY_BQL_SENSOR
		pinSetIdx = 0;
#else
		pinSetIdx = 3;
#endif
#endif
	}
	else if (DUAL_CAMERA_SUB_3_SENSOR == SensorIdx)
	{
#ifdef CUSTOM_SENSOR_GROUP_SWAP
		pinSetIdx = 1;
#else
#ifdef CUSTOM_DUMMY_BQL_SENSOR
		pinSetIdx = 1;
#else
		pinSetIdx = 4;
#endif
#endif
	}
	else if (DUAL_CAMERA_MAIN_2_SENSOR == SensorIdx)
	{
		pinSetIdx = 2;
	}

	/* power ON */
	if (On)
	{
		sensorPowerStatus[pinSetIdx] = 1;
		PK_INFO("PowerOn:SensorName=%s, pinSetIdx=%d, sensorIdx:%d\n", currSensorName, pinSetIdx, SensorIdx);
		if( (pinSetIdx==1) ||(pinSetIdx==0))
		{
			pinctrl_select_state(camctrl, cam_mipi_switch_sel_h);
			mdelay(1);

			for (pwListIdx = 0; pwListIdx < IDX_AEON_CAMERA_OFFSET; pwListIdx++)
			{
				if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
				        && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
				{
					//PK_INFO("sensorIdx:%d\n", SensorIdx);
					sensorInPowerList = KAL_TRUE;
					for (pwIdx = 0; pwIdx < 10; pwIdx++)
					{
						PK_DBG("PowerType:%d\n", PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);
						if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType != VDD_None)
						{
							if (hwpoweron(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx], mode_name) == FALSE)
								goto _kdCISModulePowerOn_exit_;
						}
						else
						{
							PK_DBG("pwIdx=%d\n", pwIdx);
							break;
						}
					}
					break;
				}
				else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
				{
					break;
				}
				else
				{
				}
			}
		}
		else
		{
			pinctrl_select_state(camctrl, cam_mipi_switch_sel_l);
			mdelay(1);

			for (pwListIdx = IDX_AEON_CAMERA_OFFSET; pwListIdx < 16; pwListIdx++)
			{
				if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
				        && (0 == strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
				{
					sensorInPowerList = KAL_TRUE;
					for (pwIdx = 0; pwIdx < 10; pwIdx++)
					{
						PK_DBG("PowerType:%d\n", PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);
						if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType != VDD_None)
						{
							if (hwpoweron(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx], mode_name) == FALSE)
								goto _kdCISModulePowerOn_exit_;
						}
						else
						{
							PK_DBG("pwIdx=%d\n", pwIdx);
							break;
						}
					}
					break;
				}
				else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
				{
					break;
				}
				else
				{
				}
			}
		}
	}
	else
	{	
		/* power OFF */
		sensorPowerStatus[pinSetIdx] = 0;
		PK_INFO("PowerOFF:pinSetIdx=%d, sensorIdx:%d\n", pinSetIdx, SensorIdx);
		if( (pinSetIdx==1) ||(pinSetIdx==0))
		{
			for (pwListIdx = 0; pwListIdx < IDX_AEON_CAMERA_OFFSET; pwListIdx++)
			{
				if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
				        && (0 ==strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
				{
					sensorInPowerList = KAL_TRUE;
					for (pwIdx = 9; pwIdx >= 0; pwIdx--)
					{
						PK_DBG("PowerType:%d\n", PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);
						if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
						        PowerType != VDD_None)
						{
							if (hwpowerdown(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx], mode_name) == FALSE)
								goto _kdCISModulePowerOn_exit_;
							if (pwIdx > 0)
							{
								if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx - 1].Delay > 0)
									mdelay(PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx - 1].Delay);
							}
						}
						else
						{
							PK_DBG("pwIdx=%d\n", pwIdx);
						}
					}
				}
				else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
				{
					break;
				}
				else
				{
				}
			}
		}
		else
		{
			for (pwListIdx = IDX_AEON_CAMERA_OFFSET; pwListIdx < 16; pwListIdx++)
			{
				if (currSensorName && (PowerOnList.PowerSeq[pwListIdx].SensorName != NULL)
				        && (0 ==strcmp(PowerOnList.PowerSeq[pwListIdx].SensorName,currSensorName)))
				{
					sensorInPowerList = KAL_TRUE;
					for (pwIdx = 9; pwIdx >= 0; pwIdx--)
					{
						PK_DBG("PowerType:%d\n", PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].PowerType);
						if (PowerOnList.PowerSeq[pwListIdx].PowerInfo[pwIdx].
						        PowerType != VDD_None)
						{
							if (hwpowerdown
							        (PowerOnList.PowerSeq[pwListIdx].
							         PowerInfo[pwIdx], mode_name) == FALSE)
								goto _kdCISModulePowerOn_exit_;
							if (pwIdx > 0)
							{
								if (PowerOnList.PowerSeq[pwListIdx].
								        PowerInfo[pwIdx - 1].Delay > 0)
									mdelay(PowerOnList.
									       PowerSeq[pwListIdx].
									       PowerInfo[pwIdx - 1].Delay);
							}
						}
						else
						{
							PK_DBG("pwIdx=%d\n", pwIdx);
						}
					}
				}
				else if (PowerOnList.PowerSeq[pwListIdx].SensorName == NULL)
				{
					break;
				}
				else
				{
				}
			}
		}
	}

	return 0;

_kdCISModulePowerOn_exit_:
	return -EIO;
}

static int __init dt_get_extbuck_info(unsigned long node, const char *uname, int depth, void *data)
{
	struct devinfo_extbuck_tag
	{
		u32 size;
		u32 tag;
		u32 extbuck_fan53526_exist;
	} *tags;
	unsigned int size = 0;

	if (depth != 1 || (strcmp(uname, "chosen") != 0 && strcmp(uname, "chosen@0") != 0))
		return 0;

	tags = (struct devinfo_extbuck_tag *) of_get_flat_dt_prop(node, "atag,extbuck_fan53526", &size);

	if (tags)
	{
		extbuck_fan53526_exist = tags->extbuck_fan53526_exist;
		PK_INFO("[Camera] fan53526_exist = %d\n",
		        extbuck_fan53526_exist);
	}
	return 0;
}

static int __init kd_camera_hw_init(void)
{
	of_scan_flat_dt(dt_get_extbuck_info, NULL);

	return 0;
}

arch_initcall(kd_camera_hw_init);
EXPORT_SYMBOL(kdCISModulePowerOn);

/* !-- */
/*  */
