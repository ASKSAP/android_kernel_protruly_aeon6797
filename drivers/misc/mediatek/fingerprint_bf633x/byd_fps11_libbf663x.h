/*
 * File:         byd_fps11_libbf663x.h
 *
 * Created:	     2017-02-24
 * Description:  BYD fingerprint IC driver library, chip working process oriented
 *
 * Copyright (C) 2017 BYD Company Limited
 *
 */

#ifndef __FPS11_LIBBF66XX_H__
#define __FPS11_LIBBF66XX_H__

#include "byd_fps_libbf663x.h"

//定义寄存器信息
//register of fps11
//analogous circuit: scan time sequence configure.
/*#define REG_SENSOR_MODE                 0x01
#define REG_TX_CLK_SEL					0x02
#define REG_TX_CLK_H_TIME               0x03
#define REG_SEN_RST_TIME                0x04
#define REG_INIT_TX_SEL                 0x05
#define REG_FP_INIT_NUM                 0x06
#define REG_FP_INIT_ROW                 0x07
#define REG_FP_INIT_COL                 0x08
#define REG_FG_INIT_NUM                 0x09
#define REG_FG_INIT_ROW                 0x0A
//digital circuit: fingerprint scan.
#define REG_FIR_CMD		                0x0B
#define REG_FIR_DATA	                0x0C
#define REG_DIG_INTE_SEL                0x0D
#define REG_FRAME_ADD_NUM	            0x0E
#define REG_SCAN_DATA_CUT_SEL           0x0F
#define REG_FRAME_ADD_CUT_SEL           0x10
#define REG_READ_ROW_CFG0               0x11
#define REG_READ_ROW_CFG1               0x12
#define REG_FPD_DATA		            0x13
#define REG_ERROR_LINE_NUM              0x14
#define REG_SRAM_EMPTY	                0x15
//digital circuit: finger touch on/off check (scan).
#define REG_FG_ROW_START_A              0x16
#define REG_FG_ROW_START_B		        0x17
#define REG_FG_ROW_START_C	            0x1
*/
//#define REG_FG_ROW_NUM_AB               0x19
#define REG_FG_ROW_NUM                  0x1A    //名字变化
/*#define REG_FINGER_INTE_SEL             0x1B
#define REG_SUB_CUT_SEL                 0x1C
#define REG_FPD_TH_ON_H                 0x1D
#define REG_FPD_TH_ON_L                 0x1E
#define REG_FPD_TH_OFF_H                0x1F
#define REG_FPD_TH_OFF_L                0x20
#define REG_INT_MODE_SET                0x21
#define REG_SUB_SET_L                   0x22
//#define REG_SUB_SET_H                   0x23
#define REG_INT_FINGER_NUM              0x24
#define REG_FPD_STATE_L 				0x25
//#define REG_FPD_STATE_H		            0x26
#define REG_FINGER_STATE                0x27
#define REG_SUB_VALUE_SEL               0x28
#define REG_SUB_VALUE_H                 0x29
#define REG_SUB_VALUE_L                 0x2A
//FPD test
#define REG_FPD_TEST_MODE               0x2B
#define REG_ADC_DP_DATA_H               0x2C
#define REG_ADC_DP_DATA_L               0x2D
#define REG_TEST_FIR_CTRL               0x2E
//work mode configure.
#define REG_CHIP_MODE                   0x30
#define REG_SCAN_CMD                    0x31
#define REG_AREA0_ROW_START             0x32
#define REG_AREA0_ROW_END	            0x33
#define REG_AREA0_COL_START             0x34
#define REG_AREA0_COL_END	            0x35
#define REG_AREA1_ROW_START             0x36
#define REG_AREA1_ROW_END	            0x37
#define REG_AREA1_COL_START             0x38
#define REG_AREA1_COL_END	            0x39
#define REG_FP_WAIT_TI_H	            0x3A
#define REG_FP_WAIT_TI_L	            0x3B
#define REG_OS_WAIT_TI_H	            0x3C
#define REG_OS_WAIT_TI_L	            0x3D
#define REG_FG_REST_TI1_EN	            0x3E
#define REG_FG_REST_TI1_H	            0x3F
#define REG_FG_REST_TI1_L 		        0x40
#define REG_FG_REST_TI2_H	            0x41
#define REG_FG_REST_TI2_L 		        0x42
#define REG_FP_CYCLE_REST_TI	        0x43
#define REG_FP_SINGLE_REST_TI	        0x44
#define REG_NO_FINGER_NUM_H		        0x45
#define REG_NO_FINGER_NUM_L		        0x46
#define REG_OS_TH_H			            0x47
#define REG_OS_TH_L			            0x48
#define REG_FG_INIT_TIME	            0x49
#define REG_CFG_FLAG		            0x4A
#define REG_FP_INT_FINGER_EN            0x4B
//System set.
#define REG_SOFT_RST		            0x4C
#define REG_INT_STATE		            0x4D
#define REG_FUNCTION_SEL	            0x4E
#define REG_INT_CFG			            0x4F
#define REG_IO_STATE_SEL	            0x50
#define REG_PD_ANA_A		            0x51
#define REG_PD_ANA_B		            0x52
//test.
#define REG_TEST_FP_SCAN                0x53
#define REG_TIMER_TEST_EN               0x54
//Analogous circuit parameters.
#define REG_ADC_I_SEL                   0x55
#define REG_SEN_BUF_I                   0x56
#define REG_IN_PHASE	                0x57
#define REG_SH_I_SEL                    0x58
#define REG_TX_CFG	                    0x59
//SFR Adjust.
#define REG_VTX_SEL_CTRL                0x5A
#define REG_VCAL_SEL_CTRL               0x5B
//cfg byte & finger os.
#define REG_FINGER_OS_DATA_SEL          0x5C
#define REG_FINGER_OS_DATA_H            0x5D
#define REG_FINGER_OS_DATA_L            0x5E
*/
#define REG_DUMMY_SFR_L                 0x5F  //sys_sfr_a0的dummy寄存器。内部调试用，不要对外公开。
//fpd_sfr                                     //digital circuit: finger touch on/off check (scan).
#define REG_FG_ROW_START_D              0x60  //手指扫描D区域起始行
#define REG_FG_ROW_START_E              0x61  //手指区域E区域起始行
#define REG_SUB_AREA_SEL                0x62
//OTP OP.
/*#define REG_SFR_OTP_CMD                 0x63
#define REG_SFR_OTP_DATA                0x64
#define REG_OTP_CMD                     0x65
#define REG_OTP_DATA	                0x66*/
//Adjust & test
//#define REG_ADJ_MODE                    0x67 
#define REG_ADJ_SFR_ADDR                0x68 //
//#define REG_ADJ_INFO_1                  0x69
//#define REG_ADJ_INFO_2                  0x6A
#define REG_ADJ_SFR_DATA                0x6B //

//System set                                  //新增
#define REG_KEY_EN                      0x6C
#define REG_CPU_TIMER_CFG               0x6D
#define REG_INT_STATE_KEY               0x6E
#define REG_RST_STATE                   0x6F
#define REG_CPU_PARA_ADDR               0x70
#define REG_CPU_PARA_DATA               0x71
#define REG_PD_INT_FINGER               0x72

//csd scan                                    //新增
#define REG_KEY_STATE                   0x7B
#define REG_CSD_SFR_ADDR                0x7C
#define REG_CSD_SFR_DATA                0x7D
#define REG_MKEY_LEVEL                  0x7E

//-------------------------------------------
//
/*#define CHIP_MODE_IDLE		0
#define CHIP_MODE_FG_DET	1
#define CHIP_MODE_SLEEP		2

#define CHIP_SCAN_CMD_FG_DET    	0
#define CHIP_SCAN_CMD_FP_SCAN    	0x11*/


#ifdef BYD_FPS_CFG_SEPERATE

#ifdef BYD_FPS_COVER_PLATE
    #define VAL_SENSOR_MODE			1  //4倍放大系数
#else
	#define VAL_SENSOR_MODE			3  //2倍放大系数
#endif

unsigned char VAL_TX_CLK_SEL11 = 	4;    //380khz 根据VAL_SENSOR_MODE值来定TXT配置
#define VAL_TX_CLK_H_TIME11		0x0c  //1000ns
#define VAL_SEN_RST_TIME		0x11
#define VAL_INIT_TX_SEL			1
#define VAL_FP_INIT_NUM			0xcf
#define VAL_FP_INIT_ROW			0
#define VAL_FP_INIT_COL			0
#define VAL_FG_INIT_NUM			0xcf
#define VAL_FG_INIT_ROW			16//24	//same with REG_FG_ROW_START_A(0x16)

//unsigned char  VAL_DIG_INTE_SEL	= 0x0f;//byd_fps_fae_image[1];//INTE_NUM=(VAL+1)*2

#ifdef BYD_FPS_COVER_PLATE
    #define VAL_SCAN_DATA_CUT_SEL			0x14  //指纹扫描数据存储位数截取选择
#else
	#define VAL_SCAN_DATA_CUT_SEL			0x12  //64位为0x13//指纹扫描数据存储位数截取选择 //0x11-17, 0x12-18, 0x13-19
#endif
//unsigned char  VAL_READ_ROW_CFG0 = 0x61;  //区域0的数据中断输出的起始行配置:97行
//unsigned char  VAL_READ_ROW_CFG1 = 0x61;   //区域1的数据中断输出的起始行配置:97行


#define VAL_FG_ROW_START_A_GEST				0x01//24 0x10
#define VAL_FG_ROW_START_B_GEST				0x2c//44
#define VAL_FG_ROW_START_C_GEST				0x56//80  0x48

#define VAL_FG_ROW_START_A				0x10//16
#define VAL_FG_ROW_START_B11				0x26//38
#define VAL_FG_ROW_START_C11				0x3c//60
#define VAL_FG_ROW_START_D				0x52//82
#define VAL_FG_ROW_START_E				0x68//104

#define VAL_REG_SUB_AREA_SEL            0x1F//5个区域全部使能

#define VAL_FG_ROW_NUM   				0x07

#define VAL_FINGER_INTE_SEL				0
#define VAL_SUB_CUT_SEL					4

//unsigned char VAL_FPD_TH_ON_H = 0xee;//byd_fps_fae_detect[0];
//unsigned char VAL_FPD_TH_ON_L = 0xe0;//byd_fps_fae_detect[1];

//unsigned char VAL_FPD_TH_OFF_H = 0xf3;//byd_fps_fae_detect[2];
//unsigned char VAL_FPD_TH_OFF_L = 0x50;//byd_fps_fae_detect[3];

#define VAL_INT_MODE_SET11				0x07 //触摸子区域的个数大于等于设定触摸个数后，才发出中断!目前是1个子区域
#define VAL_INT_MODE_SET_FP11           	0x0F
#define VAL_SUB_SET_L11					0x1F 
#define VAL_INT_FINGER_NUM				0x00

#define VAL_CHIP_MODE					0
#define VAL_SCAN_CMD					0

#define VAL_AREA0_ROW_START				0
#define VAL_AREA0_ROW_END11				0x7f
#define VAL_AREA0_COL_START				0
#define VAL_AREA0_COL_END11				0x41

#define VAL_AREA1_ROW_START				0
#define VAL_AREA1_ROW_END11				0x7f
#define VAL_AREA1_COL_START				0
#define VAL_AREA1_COL_END11				0x41

#define VAL_FP_WAIT_TI_H				0
#define VAL_FP_WAIT_TI_L				0x1E//0
#define VAL_OS_WAIT_TI_H				0
#define VAL_OS_WAIT_TI_L				0
#define VAL_FG_REST_TI1_EN				0
//unsigned char VAL_FG_REST_TI1_H	= 0x00;//byd_fps_fae_detect[4];
//unsigned char VAL_FG_REST_TI1_L	= 0x07;//byd_fps_fae_detect[5];
//unsigned char VAL_FG_REST_TI2_H	= 0x00;//byd_fps_fae_detect[6];
//unsigned char VAL_FG_REST_TI2_L	= 0xc7;//byd_fps_fae_detect[7];
#define VAL_FP_CYCLE_REST_TI			0x07
#define VAL_FG_SINGLE_REST_TI			0x07
//unsigned char VAL_NO_FINGER_NUM_H = 0x0f;//byd_fps_fae_detect[8];
//unsigned char VAL_NO_FINGER_NUM_L =	0xa0;//byd_fps_fae_detect[9];
#define VAL_OS_TH_H						0xF1
#define VAL_OS_TH_L						0xFF
#define VAL_FG_INIT_TIME				0
#define VAL_FP_INT_FINGER_EN			0

#define VAL_FUNCTION_SEL				0
#define VAL_INT_CFG						0
#define VAL_IO_STATE_SEL				0x00	//0x02
#define VAL_PD_ANA_A					0x00	//0xff
#define VAL_PD_ANA_B					0x00	//0x0e

#define VAL_ADC_I_SEL					2
#define VAL_SEN_BUF_I					1
#define VAL_IN_PHASE					3
#define VAL_SH_I_SEL					2
#define VAL_TX_CFG						0x19

#endif

#define OTP_MODULE_ID11             0x90
#define OTP_MODULE_BYTE11           8

#define OTP_BAD_POINT_CHIP_START11  0x98
#define OTP_BAD_POINT_CHIP11    0x99
#define OTP_BAD_POINT_CHIP_END11   0x198
 
#define OTP_BAD_POINT_POTTING_START11  0x199
#define OTP_BAD_POINT_POTTING11   0x19A
#define OTP_BAD_POINT_POTTING_END11  0x21D
 
#define OTP_BAD_POINT_MODULE_START11  0x21E
#define OTP_BAD_POINT_MODULE11   0x21F
#define OTP_BAD_POINT_MODULE_END11  0x2A0
 
#define OTP_BAD_COLUMN_4711    0x2A1
#define OTP_BAD_COLUMN_4811    0x2A2
 
#define OTP_OS_DATA_START11   0x2A3
#define OTP_OS_DATA_END11      0x3422
 
#define OTP_COLUMN_ADJ_START11   0x3423
#define OTP_COLUMN_ADJ_END11     0x34A2
 
#define OTP_MODULE_INFO_START11   0x34A3
#define OTP_MODULE_INFO_END11     0x34E2

#define OTP_MARK11  0x34E3

#endif