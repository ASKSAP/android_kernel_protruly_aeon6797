/*
 * File:         byd_fps11_libbf663x.h
 *
 * Created:	     2017-02-24
 * Description:  BYD fingerprint IC driver library, chip working process oriented
 *
 * Copyright (C) 2017 BYD Company Limited
 *
 */

#ifndef __FPS22_LIBBF66XX_H__
#define __FPS22_LIBBF66XX_H__

#include "byd_fps_libbf663x.h"

//定义寄存器信息
//register of fps22
//Analogous Scan time sequence configure. ADDR: 0x01~0x05
#define REG_SENSOR_MODE22                 0x01
#define REG_TX_CLK_SEL22					0x02
#define REG_TX_CLK_H_TIME22              0x03
#define REG_SEN_RST_TIME22                0x04
#define REG_FPD_INIT_NUM22                0x05

#define VAL_SENSOR_MODE22					0x06
#define VAL_TX_CLK_SEL22					0x01
#define VAL_TX_CLK_H_TIME22               0x0A
#define VAL_SEN_RST_TIME22                0x05
#define VAL_FPD_INIT_NUM22                0x00

//Fingerprint scan configure. ADDR: 0x06~0x09
#define REG_DIG_INTE_SEL22				0x06
#define REG_DUMMY_CFG_H22					0x07
#define REG_DUMMY_CFG_L22					0x08
#define REG_FPD_DATA22					0x09

#define VAL_DIG_INTE_SEL22				0x0F
#define VAL_DUMMY_CFG_H22					0x00
#define VAL_DUMMY_CFG_L22					0x00
//#define VAL_FPD_DATA					0x00

//Finger detect configure. ADDR: 0x0A~0x17
#define REG_FPD_TH_ON_H22					0x0A
#define REG_FPD_TH_ON_L22					0x0B
#define REG_FPD_TH_OFF_H22				0x0C
#define REG_FPD_TH_OFF_L22				0x0D
#define REG_FPD_TH_ON_H2_22				0x0E
#define REG_FPD_TH_ON_L2_22				0x0F
#define REG_FPD_TH_OFF_H2_22				0x10
#define REG_FPD_TH_OFF_L2_22				0x11
#define REG_INT_FINGER_CFG22				0x12
#define REG_FPD_STATE22					0x13
#define REG_FINGER_STATE22				0x14
#define REG_SUB_VALUE_SEL22				0x15
#define REG_SUB_VALUE_H22					0x16
#define REG_SUB_VALUE_L22					0x17

#define VAL_FPD_TH_ON_H22					0xFE
#define VAL_FPD_TH_ON_L22					0x00
#define VAL_FPD_TH_OFF_H22				0xFF
#define VAL_FPD_TH_OFF_L22				0x00
#define VAL_FPD_TH_ON_H2_22				0xFE
#define VAL_FPD_TH_ON_L2_22				0x00
#define VAL_FPD_TH_OFF_H2_22				0xFF
#define VAL_FPD_TH_OFF_L2_22				0x00
#define VAL_INT_FINGER_CFG22				0x03//三块触发中断
//#define VAL_FPD_STATE					0x00
#define VAL_FINGER_STATE22				0x00
#define VAL_SUB_VALUE_SEL22				0x00
//#define VAL_SUB_VALUE_H					0x00
//#define VAL_SUB_VALUE_L					0x00

//FPD test. ADDR: 0x18~0x19
#define REG_FPD_TEST_MODE22				0x18
#define REG_FPD_TEST_CFG22				0x19

#define VAL_FPD_TEST_MODE22				0x00
#define VAL_FPD_TEST_CFG22				0x00

//Mode configure. ADDR: 0x30~0x36
//#define REG_CHIP_MODE					0x30
#define REG_FG_REST_TI1_22					0x31
#define REG_FG_REST_TI2_22					0x32
#define REG_NO_FINGER_NUM22				0x33
#define REG_OS_TH_H22					0x34
#define REG_OS_TH_L22						0x35
#define REG_FG_INIT_TIME22				0x36

#define VAL_CHIP_MODE22					0x01
#define VAL_FG_REST_TI1_22					0x02//6ms
#define VAL_FG_REST_TI2_22					0x05//48ms
#define VAL_NO_FINGER_NUM22				0x41//5S
#define VAL_OS_TH_H22						0xfe
#define VAL_OS_TH_L22						0xd0
#define VAL_FG_INIT_TIME22				0x01


//System configure. ADDR: 0x37~0x3E
#define REG_RST_STATE22					0x37
#define REG_SOFT_RST22					0x38
#define REG_INT_STATE22					0x39
#define REG_INT_CFG22						0x3A
#define REG_IO_STATE_SEL22				0x3B
#define REG_PD_ANA_A22					0x3C
#define REG_PD_ANA_B22					0x3D
#define REG_SUB_SEL22						0x3E

//#define VAL_RST_STATE					0x01
//#define VAL_SOFT_RST					0xAA
//#define VAL_INT_STATE					0x00
#define VAL_INT_CFG22						0x01//建议高电平有效，中断上升沿触发
#define VAL_IO_STATE_SEL22				0x00
#define VAL_PD_ANA_A22_2V					0x20//0x20
#define VAL_PD_ANA_B22_2V					0x70//0x70//0x00 Vtx == 2.0,需配置为0x70；Vtx == 1.7,需配置为0x00
#define VAL_PD_ANA_A22_1P7V					0x20//0x20
#define VAL_PD_ANA_B22_1P7V	                0x30
#define VAL_SUB_SEL22						0x00//3个子区域

//Analogous parameters configure. ADDR: 0x3F~0x40
#define REG_ADC_SH_CFG22					0x3F
#define REG_SEL_CP_VDDS22				0x40

#define VAL_ADC_SH_CFG22					0x99//0x95/
#define VAL_SEL_CP_VDDS22					0x33

//Abnormal status checking. ADDR: 0x41~0x43
#define REG_CHECK_ERROR_EN22				0x41
#define REG_CHECK_ERROR_TIME22			0x42
#define REG_CHECK_ERROR_STATE22			0x43

#define VAL_CHECK_ERROR_EN22				0xC3//关闭异常
#define VAL_CHECK_ERROR_TIME22			0x07
//#define VAL_CHECK_ERROR_STATE			0x00

//Configure bytes from OTP. ADDR: 0x44~0x46
#define REG_FINGER_OS_DATA_SEL22			0x44
#define REG_FINGER_OS_DATA_H22			0x45
#define REG_FINGER_OS_DATA_L22			0x46

#define VAL_FINGER_OS_DATA_SEL22			0x00
#define VAL_FINGER_OS_DATA_H22			0x03
#define VAL_FINGER_OS_DATA_L22			0xFF

//Testing. ADDR: 0x47
#define REG_TIMER_TEST_EN22				0x47

#define VAL_TIMER_TEST_EN22				0x00

//预留寄存器.  ADDR: 0x48
#define REG_DUMMY_SFR_122					0x48

#define VAL_DUMMY_SFR_122					0x00

#define REG_ADJ_SFR_ADDR22				0x68
#define REG_ADJ_SFR_DATA22				0x6B

#if 0
//OTP operation. ADDR: 0x63~0x66
#define REG_SFR_OTP_CMD					0x63
#define REG_SFR_OTP_DATA				0x64
#define REG_OTP_CMD						0x65
#define REG_OTP_DATA					0x66

#define VAL_SFR_OTP_CMD					0x00
#define VAL_SFR_OTP_DATA				0x00
#define VAL_OTP_CMD						0x00
#define VAL_OTP_DATA					0x00

//Adjust & Testing. ADDR: 0x67~0x6B
#define REG_ADJ_MODE					0x67
#define REG_ADJ_SFR_ADDR				0x68
#define REG_ADJ_INFO_1					0x69
#define REG_ADJ_INFO_2					0x6A
#define REG_ADJ_SFR_DATA				0x6B

#define VAL_ADJ_MODE					0x00
#define VAL_ADJ_SFR_ADDR				0x00
#define VAL_ADJ_INFO_1					0xFF
#define VAL_ADJ_INFO_2					0xFF
#define VAL_ADJ_SFR_DATA				0x00
#endif

#if 0
//BYD_FPS_REG_ADDR Array.
 unsigned char byd_fps_reg_addr[5+4+14+11+4+2+3+3+4] = {
	//Analogous Scan time sequence configure. ADDR: 0x01~0x05, 5
	REG_SENSOR_MODE,			//ADDR: 0x01
	REG_TX_CLK_SEL,				//ADDR: 0x02
	REG_TX_CLK_H_TIME,			//ADDR: 0x03
	REG_SEN_RST_TIME,			//ADDR: 0x04
	REG_FPD_INIT_NUM,			//ADDR: 0x05
	
	//Fingerprint scan configure. ADDR: 0x06~0x09, 4
	REG_DIG_INTE_SEL,			//ADDR: 0x06
	REG_DUMMY_CFG_H,            //ADDR: 0x07
	REG_DUMMY_CFG_L,            //ADDR: 0x08
	REG_FPD_DATA22,               //ADDR: 0x09
	
	//Finger detect configure. ADDR: 0x0A~0x17, 14
	REG_FPD_TH_ON_H,			//ADDR: 0x0A
	REG_FPD_TH_ON_L,            //ADDR: 0x0B
	REG_FPD_TH_OFF_H,           //ADDR: 0x0C
	REG_FPD_TH_OFF_L,           //ADDR: 0x0D
	REG_FPD_TH_ON_H2,   		//ADDR: 0x0E
	REG_FPD_TH_ON_L2,           //ADDR: 0x0F
	REG_FPD_TH_OFF_H2,          //ADDR: 0x10
	REG_FPD_TH_OFF_L2,          //ADDR: 0x11
	REG_INT_FINGER_CFG, 		//ADDR: 0x12
	REG_FPD_STATE,              //ADDR: 0x13
	REG_FINGER_STATE,           //ADDR: 0x14
	REG_SUB_VALUE_SEL,          //ADDR: 0x15
	REG_SUB_VALUE_H,    		//ADDR: 0x16
	REG_SUB_VALUE_L,            //ADDR: 0x17

	//Mode configure. ADDR: 0x30~0x36, 7
	REG_CHIP_MODE,				//ADDR: 0x30
	REG_FG_REST_TI1,			//ADDR: 0x31
	REG_FG_REST_TI2,			//ADDR: 0x32
	REG_NO_FINGER_NUM,			//ADDR: 0x33
	REG_OS_TH_H,				//ADDR: 0x34
	REG_OS_TH_L,				//ADDR: 0x35
	REG_FG_INIT_TIME,			//ADDR: 0x36
	
	//System configure. ADDR: 0x37~0x3E, 8
	REG_RST_STATE,				//ADDR: 0x37
	REG_SOFT_RST,				//ADDR: 0x38
	REG_INT_STATE,				//ADDR: 0x39
	REG_INT_CFG,				//ADDR: 0x3A
	REG_IO_STATE_SEL,			//ADDR: 0x3B
	REG_PD_ANA_A,				//ADDR: 0x3C
	REG_PD_ANA_B,				//ADDR: 0x3D
	REG_SUB_SEL,				//ADDR: 0x3E
	
	//Analogous parameters configure. ADDR: 0x3F~0x40, 2
	REG_ADC_SH_CFG,				//ADDR: 0x3F
	REG_SEL_CP_VDDS,			//ADDR: 0x40
	
	//Abnormal status checking. ADDR: 0x41~0x43, 3
	REG_CHECK_ERROR_EN,			//ADDR: 0x41
	REG_CHECK_ERROR_TIME,		//ADDR: 0x42
	REG_CHECK_ERROR_STATE,		//ADDR: 0x43
	
	//Configure bytes from OTP. ADDR: 0x44~0x46, 3
	REG_FINGER_OS_DATA_SEL,		//ADDR: 0x44
	REG_FINGER_OS_DATA_H,		//ADDR: 0x45
	REG_FINGER_OS_DATA_L,		//ADDR: 0x46
	
	//OTP operation. ADDR: 0x63~0x66, 4
	REG_SFR_OTP_CMD,			//ADDR: 0x63
	REG_SFR_OTP_DATA,			//ADDR: 0x64
	REG_OTP_CMD,				//ADDR: 0x65
	REG_OTP_DATA,				//ADDR: 0x66
};
#endif

//BYD_FPS_REG_ADDR_VAL Array.
#define BYD_CFG_LEN_ANA1 	5
#define BYD_CFG_LEN_FP 		3
#define BYD_CFG_LEN_FNG 	10
#define BYD_CFG_LEN_MODE 	6
#define BYD_CFG_LEN_SYS 	5
#define BYD_CFG_LEN_ANA2 	2
#define BYD_CFG_LEN_ABN 	2

#define BYD_FPS_BUF_CFG_LEN	(BYD_CFG_LEN_ANA1+BYD_CFG_LEN_FP+BYD_CFG_LEN_FNG+BYD_CFG_LEN_MODE+BYD_CFG_LEN_SYS+BYD_CFG_LEN_ANA2+BYD_CFG_LEN_ABN)
#if 0
 unsigned char byd_fps_reg_addr_val[(BYD_CFG_LEN_ANA1 + BYD_CFG_LEN_FP + BYD_CFG_LEN_FNG 
		+ BYD_CFG_LEN_MODE + BYD_CFG_LEN_SYS + BYD_CFG_LEN_ANA2 + BYD_CFG_LEN_ABN)*2] = {
	//Analogous Scan time sequence configure. ADDR: 0x01~0x05.
	//BYD_CFG_LEN_ANA1 == 5
	REG_SENSOR_MODE,		VAL_SENSOR_MODE,		//ADDR: 0x01
	REG_TX_CLK_SEL,			VAL_TX_CLK_SEL,         //ADDR: 0x02
	REG_TX_CLK_H_TIME,		VAL_TX_CLK_H_TIME,      //ADDR: 0x03
	REG_SEN_RST_TIME,		VAL_SEN_RST_TIME,       //ADDR: 0x04
	REG_FPD_INIT_NUM,		VAL_FPD_INIT_NUM,       //ADDR: 0x05
	
	//Fingerprint scan configure. ADDR: 0x06~0x09.
	//BYD_CFG_LEN_FP  == 3
	REG_DIG_INTE_SEL,		VAL_DIG_INTE_SEL,		//ADDR: 0x06
	REG_DUMMY_CFG_H,        VAL_DUMMY_CFG_H,        //ADDR: 0x07
	REG_DUMMY_CFG_L,        VAL_DUMMY_CFG_L,        //ADDR: 0x08
	
	//Finger detect configure. ADDR: 0x0A~0x17.
	//BYD_CFG_LEN_FNG == 10
	REG_FPD_TH_ON_H,		VAL_FPD_TH_ON_H,		//ADDR: 0x0A
	REG_FPD_TH_ON_L,        VAL_FPD_TH_ON_L,        //ADDR: 0x0B
	REG_FPD_TH_OFF_H,       VAL_FPD_TH_OFF_H,       //ADDR: 0x0C
	REG_FPD_TH_OFF_L,       VAL_FPD_TH_OFF_L,       //ADDR: 0x0D
	REG_FPD_TH_ON_H2,       VAL_FPD_TH_ON_H2,       //ADDR: 0x0E
	REG_FPD_TH_ON_L2,       VAL_FPD_TH_ON_L2,       //ADDR: 0x0F
	REG_FPD_TH_OFF_H2,      VAL_FPD_TH_OFF_H2,      //ADDR: 0x10
	REG_FPD_TH_OFF_L2,      VAL_FPD_TH_OFF_L2,      //ADDR: 0x11
	REG_INT_FINGER_CFG,     VAL_INT_FINGER_CFG,     //ADDR: 0x12
	//REG_FPD_STATE,          VAL_FPD_STATE,          //ADDR: 0x13
	REG_FINGER_STATE,       VAL_FINGER_STATE,       //ADDR: 0x14
	
	//Mode configure. ADDR: 0x31~0x36.
	//BYD_CFG_LEN_MODE == 6
	REG_FG_REST_TI1,        VAL_FG_REST_TI1,        //ADDR: 0x31
	REG_FG_REST_TI2,        VAL_FG_REST_TI2,        //ADDR: 0x32
	REG_NO_FINGER_NUM,      VAL_NO_FINGER_NUM,      //ADDR: 0x33
	REG_OS_TH_H,            VAL_OS_TH_H,            //ADDR: 0x34
	REG_OS_TH_L,            VAL_OS_TH_L,            //ADDR: 0x35
	REG_FG_INIT_TIME,       VAL_FG_INIT_TIME,       //ADDR: 0x36
	
	//System configure. ADDR: 0x38~0x3E.
	//BYD_CFG_LEN_SYS == 5
	//REG_SOFT_RST,           VAL_SOFT_RST,           //ADDR: 0x38
	//REG_INT_STATE,          VAL_INT_STATE,          //ADDR: 0x39
	REG_INT_CFG,            VAL_INT_CFG,            //ADDR: 0x3A
	REG_IO_STATE_SEL,		VAL_IO_STATE_SEL,		//ADDR: 0x3B
	REG_PD_ANA_A,           VAL_PD_ANA_A,           //ADDR: 0x3C
	REG_PD_ANA_B,           VAL_PD_ANA_B,           //ADDR: 0x3D
	REG_SUB_SEL,            VAL_SUB_SEL,            //ADDR: 0x3E
	
	//Analogous parameters configure. ADDR: 0x3F~0x40.
	//BYD_CFG_LEN_ANA2 == 2
	REG_ADC_SH_CFG,			VAL_ADC_SH_CFG,			//ADDR: 0x3F
	REG_SEL_CP_VDDS,        VAL_SEL_CP_VDDS,        //ADDR: 0x40
	
	//Abnormal status checking. ADDR: 0x41~0x43.
	//BYD_CFG_LEN_ABN == 2
	REG_CHECK_ERROR_EN,		VAL_CHECK_ERROR_EN,		//ADDR: 0x41
	REG_CHECK_ERROR_TIME,   VAL_CHECK_ERROR_TIME,   //ADDR: 0x42
	
};

#endif

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Adjust SFR.
#define REG_ADJ_MUX_EN22					0x51
#define REG_ADJ_BG_DIG22					0x52
#define REG_ADJ_OSC22					0x53
#define REG_ADJ_BG_FPD22					0x54
#define REG_ADJ_BG_S_I22					0x55
#define REG_ADJ_BG_S22					0x56
#define REG_ADJ_BG_FPD_I22				0x57
#define REG_ADJ_VTX_SEL22				0x58
#define REG_ADJ_VCAL_SEL22				0x59
#define REG_ADJ_TIMER22					0x5A

//Configure word.
#define REG_SPI_MODE					0x5B

//Testing registers of "ADJ MODE"
#define REG_ANA_TEST22					0x5C
#define REG_TEST_CLK_CFG22				0x5D
#define REG_DPT_ADC_DATA_H22				0x5E
#define REG_DPT_ADC_DATA_L22				0x5F
#define REG_TEST_ROW_START22				0x60
#define REG_TEST_ROW_END22				0x61
////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------------------
//
//#define CHIP_MODE_IDLE					0x00	//IDLE mode.
#define CHIP_MODE_FG_DET22				0x33	//Low power finger detect.
#define CHIP_MODE_FP_SCAN22				0x55	//FP scan. Fng_det + fp_scan .
#define CHIP_MODE_OS_SCAN22				0xAA	//OS value scan. Fng_det + os_scan.
#define CHIP_MODE_SLEEP22					0xCC	//Sleep mode.

#define IRQ_DATA_READY			0x01
#define IRQ_DATA_ERROR			0x02
#define IRQ_INT_FINGER			0x04
#define IRQ_INT_TIMER			0x08
#define IRQ_INT_CHECK_ERROR		0x10
#define IRQ_NO_FNG_TIMEOUT22		0x20

#ifdef BYD_FPS_BUF_CFG

#define SPI_WR			1
#define SPI_RD			0

//Pointer of array for "cfg buf".
// unsigned char * chip_cfg_func_fpd22 = byd_fps_reg_addr_val;

#endif

#define OTP_MODULE_ID22             		0x80
#define OTP_MODULE_ID_END22             	0x87
#define OTP_MODULE_BYTE22           		8

#define OTP_BAD_POINT_CHIP_START22		0x88
#define OTP_BAD_POINT_CHIP22				0x89
#define OTP_BAD_POINT_CHIP_END22			0xC4
#define OTP_BAD_POINT_CHIP_LEN22			60

#define OTP_BAD_POINT_POTTING_START22		0xC5
#define OTP_BAD_POINT_POTTING22			0xC6
#define OTP_BAD_POINT_POTTING_END22		0x101
#define OTP_BAD_POINT_POTTING_LEN22		60

#define OTP_BAD_POINT_MODULE_START22		0x102
#define OTP_BAD_POINT_MODULE22			0x103
#define OTP_BAD_POINT_MODULE_END22		0x13E
#define OTP_BAD_POINT_MODULE_LEN22		60

#define OTP_MODULE_INFO_START22			0x13F
#define OTP_MODULE_INFO_END22				0x17E
#define OTP_MODULE_INFO_LEN				64

#define OTP_MARK						0x17F

//
unsigned char byd_fps_init_os_full[96*72]={};
unsigned char byd_fps_init_os_cut[96*56]={};
#endif