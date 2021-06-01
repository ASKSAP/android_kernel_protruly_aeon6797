/*
 * File:         byd_fps_bf66xx.h
 *
 * Created:	     2014-11-07
 * Depend on:    byd_fps_bf66xx.c
 * Description:  BYD Fingerprint IC driver for Android
 *
 * Copyright (C) 2014 BYD Company Limited
 *
 * Licensed under the GPL-2 or later.
 *
 * History:
 *
 * Contact: qi.ximing@byd.com;
 *
 */
#ifndef __FPS_BF66XX_H__
#define __FPS_BF66XX_H__

#define PLATFORM_MTK             // MTK kernel 3.18 platform
//#define PLATFORM_MTK_KERNEL3_10  // MTK kernel 3.10 platform
//#define PLATFORM_QUALCOMM        // Qualcomm msm8909 platform
#define USE_ANDROID_M_ARCH

/*******************************************************************************
* 第一部分：平台相关设置 - 通讯设置
*     通讯配置是对中断端口及 SPI的初始化配置， SPI配置通常在Kernel代码的配置文件
*  dts中（通常在 arch/arm/boot中），或在主板特定的初始化代码文件中（通常为
*  arch/arm/mach-xxxx/board-xxxx.c，在spi_board_info结构中定义），中断配置通常放
* 在驱动平台代码byd_fps_platform.c的端口配置函数 byd_fps_platform_init中。
*     将 SPI设备挂载到平台的 SPI总线，需要告知平台从设备驱动名BYD_FPS_DEV_NAME，
* 所用SPI总线编号及从设备 SPI通讯配置，驱动需要被告知平台分配给本设备的中断号
* BYD_FPS_IRQ_NUM，或者中断端口号。
*******************************************************************************/

/* 1. 配置中断 */

/* 中断触发类型，需与IC给定的一致，如IRQF_TRIGGER_FALLING, IRQF_TRIGGER_LOW,
IRQF_TRIGGER_RISING, IRQF_TRIGGER_HIGH */

#define EINT_TRIGGER_TYPE  IRQF_TRIGGER_FALLING 

/* 中断请求号IRQ，定义为一常量。可选 */
//#define BYD_FPS_IRQ_NUM  // not define it

/* 中断端口号，可选
    中断端口号用在函数byd_fps_platform_init() 中进行中断端口配置。此外，如果无法
直接给出上述的中断请求号，必须给出中断端口号*/
#ifndef BYD_FPS_IRQ_NUM
//#define BYD_FPS_EINT_PORT  // defined from .dts file
#endif

/* 2. 复位端口号, 可选 */
//#define BYD_FPS_RESET_PORT	GPIO_FP_RST_PIN

/* 3. SPI速度（实现特定于平台，目前TINY4412, QUALCOMM有效） */
#define BYD_FPS_SPI_SPEED (1 * 1000 * 1000) // 1.8, 0.56250
#define SPI_OS_SPEED  500  //扫描失调值的spi 速度配置 建议500k 96次积分
#define SPI_FP_SPEED  2400 //扫描指纹图像时的失调值配置 ：16次积分：2400k ；32次积分：1300k
#define SPI_FPS11_FPS12_SPEED 8000//FPS11,FPS12在32次积分的情况下SPI传输速度

/* 5. mtk平台主机SPI每次读出有数量限制，某些平台需4BYTE对齐 */
//#define SPI_TRANS_4BYTE 

/*******************************************************************************
* 第二部分：芯片设置
*******************************************************************************/

/* 1. 芯片型号选择，只能单选（不能同时多选） */
//#define CONFIG_FPS01
//#define CONFIG_FPS02
//#define CONFIG_FPS03 0x00//芯片型号: 0x01, 0x02, ...; 0x00 不作型号识别
//#define CONFIG_FPS13   0x4040//0x5f60
#define CONFIG_FPS12  0x5040 
#define CONFIG_FPS11  0x4840
#define CONFIG_FPS22  0x7040

/* 2. 版本信息，如下格式：
      IC编码-IC封装码-驱动版本号-客户编码-项目编码-参数版本编码 */

	#define BYD_FPS_DRIVER_DESCRIPTION  "FPS12&FPS11&FPS22-BF66xx-DRV-CLIENT0000-PROJ00-PARA1.0"


/* 3. 芯片参数 */
#if 0
#define BYD_FPS_FAE_DETECT22 {\
		700,		/*fpd_th_on 按下域值   */ \
		311,		/*fpd_th_off 抬起域值  */ \
		50*1000,	/*FG_REST_TI2 手指慢检测时间间隔(单位us), 步进为0.25ms,最小250us,最大512ms  */ \
		6*1000,		/*FG_REST_TI1 手指快检测时间间隔(单位us), 步进为0.25ms,最小250us,最大512ms  */ \
		10*1000,	/*NO_FINGER_NUM 无手指超时,(单位ms).最小xx,最大20S,建议10S */ \
	}
#endif	
//otp写没写变化量看相关log
#define BYD_FPS_FAE_DETECT22 {\
		30,		/*此系数30%表示取变化量的30%用于调按下阈值，变化量默认是6000，则按下阈值为0.3*6000 */ \
		20,		/*此系数20%表示取变化量的20%用于调抬起阈值,变化量默认是6000，则抬起阈值为0.2*6000*/ \
		50*1000,	/*FG_REST_TI2 手指慢检测时间间隔(单位us),步进为8ms,最小8ms,最大256ms */ \
		6*1000,		/*FG_REST_TI1 手指快检测时间间隔(单位us),步进为2ms,最小2ms,最大32ms */   \
		10*1000,	/*NO_FINGER_NUM 无手指超时,(单位ms).最小xx,最大20S,建议10S */ \
		25,     /*此系数25%表示取变化量的25%用于调失调值阈值，变化量默认是6000，则失调值阈值为0.25*6000*/ \
}
#define BYD_FPS_FAE_IMAGE22 {\
		0x01,/*0x01 tx_clk_sel,  tx 发射频率1.23M*/\
		0x07,/*dig_inte_sel  16 次指纹扫描积分次数*/\
		0x05,/*读取指纹数据读完每行delay的时间，单位us*/\
}
//#ifdef CONFIG_FPS12
#define BYD_FPS_32COATING
//#define BYD_FPS_64ceramic100
//#define BYD_FPS_96ceramic100
//#define BYD_FPS_128glass17

#define BYD_FPS_FAE_DETECT12 {\
		700,		/*fpd_th_on 按下域值   */ \
		311,		/*fpd_th_off 抬起域值  */ \
		50*1000,	/*FG_REST_TI2 手指慢检测时间间隔(单位us), 步进为0.25ms,最小250us,最大YY  */ \
		6*1000,		/*FG_REST_TI1 手指快检测时间间隔(单位us), 步进为0.25ms,最小250us,最大512ms  */ \
		10*1000,	/*NO_FINGER_NUM 无手指超时,(单位ms),最小xx,最大32S,建议10S */ \
	}
#ifdef  BYD_FPS_32COATING
#define BYD_FPS_FAE_IMAGE12 {\
		0x03,/*0x03 tx_clk_sel tx, 发射频率380K*/\
		0x0f,/*dig_inte_sel  32 次指纹扫描积分次数*/\
		0x39,/*read_row_cfg0 area0出指纹扫描中断位置*/\
		0x39,/*read_row_cfg1 area1出指纹扫描中断位置*/\
}
#define BYD_FPS_FAE_FIR12 {                                           \
		0x2d42, 0x2252, 0x2e18, 0x3b64, 0x4a1c, 0x59da, 0x6a52, 0x7b12, \
		0x8bae, 0x9ba2, 0xaa72, 0xb7a2, 0xc2c4, 0xcb7a, 0xd174, 0xd480, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
	}
#endif//END BYD_FPS_32COATING
#ifdef  BYD_FPS_64ceramic100
#define BYD_FPS_FAE_IMAGE12 {\
		0x03,/*tx_clk_sel tx发射频率380K*/\
		0x1f,/*dig_inte_sel  64 次指纹扫描积分次数*/\
		0x4f,/*read_row_cfg0 area0出指纹扫描中断位置*/\
		0x4f,/*read_row_cfg1 area1出指纹扫描中断位置*/\
}
#define BYD_FPS_FAE_FIR12 {                                           \
		0x1c56, 0x0fd0, 0x1408, 0x18ce, 0x1e28, 0x2414, 0x2a98, 0x31b2, \
		0x3958, 0x4180, 0x4a28, 0x5344, 0x5cb6, 0x6686, 0x708e, 0x7ac4, \
		0x850e, 0x8f58, 0x9988, 0xa384, 0xad34, 0xb682, 0xbf52, 0xc78e, \
		0xcf22, 0xd5f2, 0xdbf2, 0xe10c, 0xe532, 0xe85a, 0xea78, 0xeb88, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
	}
#endif//END BYD_FPS_64ceramic100
#ifdef  BYD_FPS_96ceramic100
#define BYD_FPS_FAE_IMAGE12 {\
		0x03,/*tx_clk_sel tx发射频率380K*/\
		0x2f,/*dig_inte_sel  96 次指纹扫描积分次数*/\
		0x39,/*read_row_cfg0 area0出指纹扫描中断位置*/\
		0x39,/*read_row_cfg1 area1出指纹扫描中断位置*/\
}
#define BYD_FPS_FAE_FIR12 {                                           \
		0x192c, 0x096c, 0x0b22, 0x0d02, 0x0f0e, 0x1142, 0x13a4, 0x1632, \
		0x18ec, 0x1bd0, 0x1ee2, 0x221e, 0x2582, 0x290e, 0x2cc0, 0x3096, \
		0x348e, 0x38a6, 0x3cd8, 0x4128, 0x4582, 0x49fc, 0x4e7e, 0x5304, \
		0x5792, 0x5c24, 0x60b0, 0x6532, 0x69a4, 0x6e06, 0x7250, 0x7680, \
		0x7a8c, 0x7e72, 0x822e, 0x85ba, 0x8914, 0x8c34, 0x8f1c, 0x91c0, \
		0x9428, 0x9644, 0x981a, 0x99a8, 0x9ae6, 0x9bd6, 0x9c78, 0x9cca, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
	}
#endif//BYD_FPS_96ceramic100
#ifdef  BYD_FPS_128glass175
#define BYD_FPS_FAE_IMAGE12 {\
		0x00,/*tx_clk_sel tx发射频率380K 03*/\
		0x3f,/*dig_inte_sel  128 次指纹扫描积分次数*/\
		0x5f,/*read_row_cfg0 area0出指纹扫描中断位置*/\
		0x5f,/*read_row_cfg1 area1出指纹扫描中断位置*/\
}
#define BYD_FPS_FAE_FIR12 {                                           \
		0x2f66, 0x0d5a, 0x0f30, 0x1128, 0x1344, 0x1584, 0x17ea, 0x1a74, \
		0x1d22, 0x1ff6, 0x22ee, 0x260a, 0x294c, 0x2cb4, 0x303e, 0x33ec, \
		0x37bc, 0x3bae, 0x3fc2, 0x43f4, 0x4842, 0x4cae, 0x5138, 0x55da, \
		0x5a8e, 0x5f60, 0x643e, 0x692e, 0x6e2e, 0x7338, 0x784a, 0x7d66, \
		0x8284, 0x87a2, 0x8cbe, 0x91d8, 0x96e8, 0x9bee, 0xa0e4, 0xa5ca, \
		0xaa9c, 0xaf56, 0xb3f4, 0xb874, 0xbcd4, 0xc112, 0xc528, 0xc916, \
		0xccda, 0xd06e, 0xd3d0, 0xd70a, 0xd9f2, 0xdcbe, 0xdf4a, 0xe18e, \
		0xe398, 0xe56c, 0xe702, 0xe856, 0xe964, 0xea2c, 0xeab0, 0xeaf2, \
	}
#endif//BYD_FPS_128glass175
//#endif//end CONFIG_FPS12

//#ifdef CONFIG_FPS13
#define BYD_FPS_FAE_DETETC13 {\
		0x21,0xff,/*fpd_th_h 0x18 抬起域值   */ \
		0x23,0xf0,/*fpd_th_l 0x10 抬起域值*/ \
		0xbf,0x18,/*touch_scan_ti 间隔时间*/ \
	}

#define BYD_FPS_FAE_IMAGE13 {\
		0x11,0x01,/*tx_clk_sel    发射频率 */           \
		0x17,0x0f,/*dig_inte_sel  指纹扫描积分次数*/            \
		0x2f,0x0c,/*scan_data_cut 截位选择*/            \
		0x31,0x00,/*scan_data_rd  截8位*/\
	}
//#endif

//#ifdef CONFIG_FPS03
#define BYD_FPS_FAE_DETETC03 {\
		0x17,0x03,/*dig_init */ \
		0x21,0x1f,/*th_h     */ \
		0x23,0x00,/*th_l     */ \
		0x25,0x18,/*sub_set  */ \
		0xbf,0x18,/*scan_time*/ \
	}
#define BYD_FPS_FAE_IMAGE03 {\
		0x11,0x00,/*tx_clk */             \
		0x17,0x0f,/*//0x3f,dig_init*/            \
		0x2f,0x0c,/*//0x0f,scan_data_cut[30:15]*/\
		0xb3,0x00,/*int_cf*/              \
		0xc1,0x08,/*CP_sel*/              \
	}
//#endif


//#ifdef CONFIG_FPS11
#define BYD_FPS_32COATING
//#define BYD_FPS_64ceramic100
//#define BYD_FPS_96ceramic100
//#define BYD_FPS_128glass175

#define BYD_FPS_FAE_DETECT11 {\
		700,		/*fpd_th_on 按下域值//4383 4095   */ \
		311,		/*fpd_th_off 抬起域值  3247 3427*/ \
		50*1000,	/*FG_REST_TI2 手指慢检测时间间隔(单位us), 步进为0.25ms,最小250us,最大YY  */ \
		6*1000,		/*FG_REST_TI1 手指快检测时间间隔(单位us), 步进为0.25ms,最小250us,最大512ms  */ \
		10*1000,	/*NO_FINGER_NUM 无手指超时,(单位ms),最小xx,最大32S,建议10S */ \
	}
#ifdef  BYD_FPS_32COATING
#define BYD_FPS_FAE_IMAGE11 {\
		0x04,/*tx_clk_sel 0x00,tx发射频率380K*/\
		0x0f,/*dig_inte_sel  32 次指纹扫描积分次数*/\
		0x61,/*read_row_cfg0 area0出指纹扫描中断位置0x61*/\
		0x61,/*read_row_cfg1 area1出指纹扫描中断位置0x61*/\
}
#define BYD_FPS_FAE_FIR11 {                                           \
		0x2d42, 0x2252, 0x2e18, 0x3b64, 0x4a1c, 0x59da, 0x6a52, 0x7b12, \
		0x8bae, 0x9ba2, 0xaa72, 0xb7a2, 0xc2c4, 0xcb7a, 0xd174, 0xd480, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
	}
#endif//END BYD_FPS_32COATING
#ifdef  BYD_FPS_64ceramic100
#define BYD_FPS_FAE_IMAGE11 {\
		0x04,/*tx_clk_sel tx发射频率380K*/\
		0x1f,/*dig_inte_sel  64 次指纹扫描积分次数*/\
		0x71,/*read_row_cfg0 area0出指纹扫描中断位置*/\
		0x71,/*read_row_cfg1 area1出指纹扫描中断位置*/\
}
#define BYD_FPS_FAE_FIR11 {                                           \
		0x1c56, 0x0fd0, 0x1408, 0x18ce, 0x1e28, 0x2414, 0x2a98, 0x31b2, \
		0x3958, 0x4180, 0x4a28, 0x5344, 0x5cb6, 0x6686, 0x708e, 0x7ac4, \
		0x850e, 0x8f58, 0x9988, 0xa384, 0xad34, 0xb682, 0xbf52, 0xc78e, \
		0xcf22, 0xd5f2, 0xdbf2, 0xe10c, 0xe532, 0xe85a, 0xea78, 0xeb88, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
	}	
#endif//END BYD_FPS_64ceramic100
#ifdef  BYD_FPS_96ceramic100
#define BYD_FPS_FAE_IMAGE11 {\
		0x04,/*tx_clk_sel tx发射频率380K*/\
		0x2f,/*dig_inte_sel  96 次指纹扫描积分次数*/\
		0x76,/*read_row_cfg0 area0出指纹扫描中断位置*/\
		0x76,/*read_row_cfg1 area1出指纹扫描中断位置*/\
}
#define BYD_FPS_FAE_FIR11 {                                           \
		0x192c, 0x096c, 0x0b22, 0x0d02, 0x0f0e, 0x1142, 0x13a4, 0x1632, \
		0x18ec, 0x1bd0, 0x1ee2, 0x221e, 0x2582, 0x290e, 0x2cc0, 0x3096, \
		0x348e, 0x38a6, 0x3cd8, 0x4128, 0x4582, 0x49fc, 0x4e7e, 0x5304, \
		0x5792, 0x5c24, 0x60b0, 0x6532, 0x69a4, 0x6e06, 0x7250, 0x7680, \
		0x7a8c, 0x7e72, 0x822e, 0x85ba, 0x8914, 0x8c34, 0x8f1c, 0x91c0, \
		0x9428, 0x9644, 0x981a, 0x99a8, 0x9ae6, 0x9bd6, 0x9c78, 0x9cca, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
		0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, 0xffff, \
	}	
#endif//BYD_FPS_96ceramic100
#ifdef  BYD_FPS_128glass175
#define BYD_FPS_FAE_IMAGE11 {\
		0x04,/*tx_clk_sel tx发射频率380K 03*/\
		0x3f,/*dig_inte_sel  128 次指纹扫描积分次数*/\
		0x7f,/*read_row_cfg0 area0出指纹扫描中断位置*/\
		0x7f,/*read_row_cfg1 area1出指纹扫描中断位置*/\ 
}
#define BYD_FPS_FAE_FIR11 {                                           \
		0x2f66, 0x0d5a, 0x0f30, 0x1128, 0x1344, 0x1584, 0x17ea, 0x1a74, \
		0x1d22, 0x1ff6, 0x22ee, 0x260a, 0x294c, 0x2cb4, 0x303e, 0x33ec, \
		0x37bc, 0x3bae, 0x3fc2, 0x43f4, 0x4842, 0x4cae, 0x5138, 0x55da, \
		0x5a8e, 0x5f60, 0x643e, 0x692e, 0x6e2e, 0x7338, 0x784a, 0x7d66, \
		0x8284, 0x87a2, 0x8cbe, 0x91d8, 0x96e8, 0x9bee, 0xa0e4, 0xa5ca, \
		0xaa9c, 0xaf56, 0xb3f4, 0xb874, 0xbcd4, 0xc112, 0xc528, 0xc916, \
		0xccda, 0xd06e, 0xd3d0, 0xd70a, 0xd9f2, 0xdcbe, 0xdf4a, 0xe18e, \
		0xe398, 0xe56c, 0xe702, 0xe856, 0xe964, 0xea2c, 0xeab0, 0xeaf2, \
	}	
#endif//BYD_FPS_128glass175
//#endif//end CONFIG_FPS11

#define BYD_FPS_FAE_FINGER_DOWN_DIFF	300 // 2000 // 按下抬起差值

/*******************************************************************************
* 第三部分：算法参数设置
*******************************************************************************/

#if (defined BYD_FPS_ALG_IN_KERNEL) || (defined BYD_FPS_ALG_IN_TZ)
#ifdef USE_ANDROID_M_ARCH
#define CONFIG_TEMPLATE_FILE_PATH  "/data/data/com.android.settings/files/template/"
#else
#define CONFIG_TEMPLATE_FILE_PATH  "/data/data/com.fingerprints.fps/files/template/"
#endif
#define CONFIG_ALG_NUM_PARA		//如果需要配置算法模板数量参数，打开该宏
#define CONFIG_ALG_AREA_PARA	//如果需要配置算法有效面积参数，打开该宏


// config maximum number of fingers and the number of templates per finger
#ifdef  CONFIG_ALG_NUM_PARA
#define CONFIG_ALG_MAX_NUM_FINGERS			  		5
#define CONFIG_ALG_NUM_TEMPLATES_PER_FENGER_BIG  		8//FPS11&FPS12
#define CONFIG_ALG_NUM_TEMPLATES_PER_FENGER_SMALL  		12
#define CONFIG_ALG_MAX_NUM_TEMPLATES_12          		30		//一个手指最多模板数量
#define CONFIG_ALG_MAX_NUM_TEMPLATES_11          		40		//一个手指最多模板数量
#define CONFIG_ALG_MAX_NUM_TEMPLATES_22         		99//20
#define CONFIG_ALG_SAME_FJUDGE                		1		//是否允许统一手指录入同一ID，0：允许；1：不允许
#endif

// config sensor area
#ifdef  CONFIG_ALG_AREA_PARA
#define CONFIG_ALG_SENSOR_VALID_ENROLL_AREA			50
#define CONFIG_ALG_SENSOR_INVALID_ENROLL_BLOCK		50
#define CONFIG_ALG_SENSOR_VALID_MATCH_AREA			60
#define CONFIG_ALG_SENSOR_INVALID_MATCH_BLOCK		60
#define CONFIG_ALG_SENSOR_INVALID_VARIANCE			6
#define CONFIG_ALG_SENSOR_MOVEXY_BIG					20//FPS12
#define CONFIG_ALG_SENSOR_MOVEXY_SMALL					25//FPS11&FPS22
#endif

#endif // BYD_FPS_ALG_IN_KERNEL or BYD_FPS_ALG_IN_TZ


/*******************************************************************************
* 第四部分：公共声明 - 研发内部使用
*******************************************************************************/

/* 设备ID及驱动名，必须与平台的设备配置文件所定义的SPI设备名称一致 */
#define BYD_FPS_NAME "byd_fps"
#define BYD_FPS_FASYNC

/* 端口配置方式：
    如果端口配置在平台端（设备配置文件或端口配置函数）中，请打开该定义，否则表示
调用驱动的端口配置函数 */
//#define BYD_FPS_PLATFORM_INIT

/* 以EARLYSUSPEND、FB、PM_SLEEP、LINUX SUSPEND/RESUME为优先顺序，请undef所采用休
   眠方式之前的所有定义*/
#undef CONFIG_HAS_EARLYSUSPEND
#undef CONFIG_FB
#undef CONFIG_PM_SLEEP

/* SPI在probe中初始化失败问题，仅在个别客户平台出现 */
#ifdef PLATFORM_MTK_KERNEL3_10
#define MTK_SPI_INIT_BUG_FIX
#endif

#define FPS_THREAD_MODE_IDLE             0
#define FPS_THREAD_MODE_ALG_PROC         1
#define FPS_THREAD_MODE_EXIT             6

#include <linux/semaphore.h>

/**
 * struct byd_fps_thread_task - thread task data
 * @mode:	current thread mode
 * @should_stop:	thread is supposed to be stopped
 * @sem_idle:
 * @wait_job:
 * @thread:
 *
 */
struct byd_fps_thread_task {
	int mode;
	int should_stop;
	struct semaphore sem_idle;
	wait_queue_head_t wait_job;
	struct task_struct *thread;
};


#endif
