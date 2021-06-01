/*
 * File:         byd_fps_libbf663x.h
 *
 * Created:	     2015-05-05
 * Description:  BYD fingerprint IC driver library, chip working process oriented
 *
 * Copyright (C) 2015 BYD Company Limited
 *
 */

#ifndef __FPS_LIBBF66XX_H__
#define __FPS_LIBBF66XX_H__

#include "byd_fps_bf66xx.h"

#define BYD_FPS_SPI_NO_DELAY									/*Read FP image data by SPI, Set the method:a.delay;b.slow down spi frq;c. add dummy read.*/
#define	BYD_FPS_SPI_DUMMY_ONCE	(0)
#define	BYD_FPS_SPI_DUMMY_NUM	BYD_FPS_SPI_DUMMY_ONCE*8					//nember of dummy data needed when SPI too fast.fps11&fps12 set to 0,fps22 need to be setted
#define	BYD_FPS_DATA_BYTE		3/2					//size of image data with 1 sensor.
#define	BYD_FPS_CHECK_SUB_AREA_SET	1				//set 1, to look up "sub_b4_value".
#define	BYD_FPS_SUB_AREA_SCAN_THRD_SET	0x3eb8		//config the threadhold value of sub_area_scan.

#define  OS_AREA_BLOCK12            9// 9,FPS11 have five block area but FPS12 have nine block area 
#define  OS_AREA_BLOCK11            5 // 5,FPS11 have five block area but FPS12 have nine block area 
#define  OS_AREA_BLOCK22            3// 3,FPS22 have three block area 

#define CONFIG_ALG_SAFE_PARA	//如果需要配置算法安全等级参数，打开该宏
/* config safe level*/
#ifdef  CONFIG_ALG_SAFE_PARA
#define CONFIG_ALG_SAFE_LEVEL						2
#define CONFIG_ALG_SAFE_LEVEL_PAYMENT				1
#endif

/*--------------------sensor dpi---------------*/
//bf5531
#define	BYD_FPS_SENSOR_WIDTH_5531	64			//size of 1 line sensors.
#define	BYD_FPS_SENSOR_HEIGHT_5531	72		    //size of lines of sensors.
#define	BYD_FPS_IMG_DATA_WIDTH_5531	96			//size of 1 line image data.
#define	BYD_FPS_IMG_DATA_HEIGHT_5531	72		    //size of lines of sensors.
//64*1.5=96 bytes, 96*72=6912 bytes.
//bf5541
//FPS 56行*64列的芯片方案。
#define	BYD_FPS_SENSOR_WIDTH_5541	64			//size of 1 line sensors.
#define	BYD_FPS_SENSOR_HEIGHT_5541	56		    //size of lines of sensors.
#define	BYD_FPS_IMG_DATA_WIDTH_5541	96			//size of 1 line image data.
#define	BYD_FPS_IMG_DATA_HEIGHT_5541	56	

//bf6677
//fps 96*96
#define	BYD_FPS_SENSOR_WIDTH12	96					//size of 1 line sensors.
#define	BYD_FPS_SENSOR_HEIGHT12	96				//size of lines of sensors.
#define	BYD_FPS_IMG_DATA_WIDTH12	144					//size of 1 line image data.
#define	BYD_FPS_IMG_DATA_HEIGHT12	96				//size of lines of sensors.


//bf6678
//fps 66*128
#define	BYD_FPS_SENSOR_WIDTH11	  66				//size of 1 line sensors.
#define	BYD_FPS_SENSOR_HEIGHT11	  128		    //size of lines of sensors.
#define	BYD_FPS_IMG_DATA_WIDTH11	99				//size of 1 line image data.
#define	BYD_FPS_IMG_DATA_HEIGHT11	128		    //size of lines of sensors.

/*---------------------end sensor dpi----------------------*/

//=============FPS13/FPS12芯片增加的宏控制==========
#define BYD_FPS_DECODE_IMAGE    //解密原始图像

//#define BYD_FPS_COVER_PLATE    //陶瓷盖板

/*
CONFIG_ALG_SENSOR_CUTMETHOD:截位方法，陶瓷盖板1~4
1：减最小值除1
2：减最小值除2
3：减最小值除3
4：减最小值除4
COUTING  0，5~8
5：截11位
6：截10位
7：截9位
8：截8位
0：动态截位
*/

#ifdef BYD_FPS_COVER_PLATE
    #define CONFIG_ALG_SENSOR_CUTMETHOD                 3    //陶瓷盖板默认3
#else
	#define CONFIG_ALG_SENSOR_CUTMETHOD                 0    //coating默认为0
#endif 

//1.芯片初始化的SPI传输方式：各寄存器独立写入 or 全部放在在buf中，一次写入。二选一。
#define BYD_FPS_CFG_SEPERATE
//#define BYD_FPS_BUF_CFG

/*2.mtk平台spi主机每次读书有数量限制，需要打开该宏，高通平台不需要*/
#if defined PLATFORM_MTK || defined PLATFORM_MTK_KERNEL3_10 
	//#ifdef CONFIG_FPS12
		#define BYD_RD_LINE_NUM12		6
	//#else
	    #define BYD_RD_LINE_NUM11		8
		#define BYD_RD_LINE_NUM         10//FPS22
    //#endif
#endif


/*3.选择创建设备文件节点的方式*/
#define BYD_FPS_SYSFS
	#ifdef BYD_FPS_SYSFS
		#define BYD_FPS_MISC_NAME  "byd_fps_misc" //"byd_fps"     //"byd_fps_misc"
	#endif 

/*4 选择是否要上报按键*/
//#define BYD_FPS_TIMER_KEY//按键以及手势上报的timer控制功能，如果手势功能关掉，该宏也要关闭
//按键和FOCUS手势的总开关
//#define BYD_FPS_GEST 

#ifdef BYD_FPS_GEST

#define BYD_FPS_REPORT_KEY//驱动自己控制按键的上报

#ifdef BYD_FPS_REPORT_KEY
#define KEY_FINGERPRINT   0x2f0//0x2f0  KEY_BACK
#endif

//#define BYD_FPS_FOCUS//focus手势开关，控制上下左右，双击，长按

//由上层控制不上报按键手势和focus手势,只要BYD_FPS_REPORT_KEY和BYD_FPS_FOCUS有一个是开的，该宏就要打开
#define BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED
	#ifdef  BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED
		#define BYD_FPS_GEST_OFF       0
		#define BYD_FPS_ONLY_KEY_ON    1
		#define BYD_FPS_ONLY_FOCUS_ON  2
		#define BYD_FPS_GEST_ON        3
	#endif
#endif

/*6.电源控制*/
//#define BYD_FPS_POWER_ON//ONLY FOR SENSOR POWER ON
//#define BYD_FPS_POWER_CTRL
#ifdef BYD_FPS_POWER_CTRL
	#define POWER_ON_TIME 40
	#define POWER_DOWN_TIME 20
#ifdef PLATFORM_MTK_KERNEL3_10//3.10内核需要配置以下信息
	#define 	BYD_POWER_PIN		(GPIO87 | 0x80000000)
	#define 	BYD_POWER_PIN_M_GPIO   GPIO_MODE_00
	#define 	BYD_POWER_PIN_M_DAIPCMOUT   GPIO_MODE_01
#endif
#endif
/*7. 抬起发中断*/
#define BYD_FPS_UP_DOWN_ASYNC
//============================================


/**
 * struct byd_fps_data - fingerprint chip related data
 * @spi:	spi device
 * @irq:	IRQ number for interrupt
 * @...:	other member...
 *
 * This struct is used by byd_fps_bf66xx.c, byd_fps_platform.c and
 * by byd_fps_libbf663x.c
 */
struct byd_fps_data {
	struct spi_device *spi;

	struct __wait_queue_head *waiting_interrupt_return; // wait_queue_head_t
	int interrupt_done;

	//struct byd_fps_thread_task thread_task;

	unsigned char *data_buffer;

	unsigned char *data_buf_1;//录入，比对选帧数据缓存
	unsigned char *data_buf_2;

	unsigned char *data_buf_os_otp;

	//#ifdef BYD_FPS_DECODE_IMAGE
	unsigned char *data_buf_decode;//保存解密后的数据
	//#endif
	#ifdef BYD_FPS_FOCUS
	unsigned char *left_right_state;//左右
	unsigned char *up_down_state;//上下
	#endif
	//unsigned int avail_data;
	int capture_done;
};

/* 1. 黑屏唤醒 */

#define BYD_FPS_INPUT_WAKEUP
/* 2. 手势 */
#define BYD_FPS_GESTURE

/* 3. 启用监视器 */
	/*方式1,延时+查询寄存器*/
//#define BYD_FPS_FG_DETECT_DELAY      //delay 3ms会只扫描一次，要根据指纹采集时间的长短，时间长，采用该方式比较合理；
	/*方式2,定时器+查询寄存器*/
//#define BYD_FPS_TIMER
//#define BYD_TIMER_FNG_UP
//#define BYD_MONITOR
//#define BYD_TIMER_FNG_DOWN
	/*方式3,计数中断个数*/
//#define BYD_FINGER_INTR_CNT//如果采集时间短，采用该方式合理

//#define  BYD_FPS_DELAY_SET_TIME
//#ifdef BYD_FPS_DELAY_SET_TIME
//#define BYD_FPS_DELAY_SET_VALUE  5
//#endif


/* library functions */
#define BYD_FPS_FAE_DETECT_LEN  6
int byd_fps_chip_sleep(struct spi_device *spi);
int byd_fps_chip_idle(struct spi_device *spi);

int byd_fps_chip_detect(struct spi_device *spi);
int byd_fps_chip_fg_detect(struct spi_device *spi);

int byd_fps_chip_state(struct spi_device *spi);

int byd_fps_chip_scan_cmd_state(struct spi_device *spi);
int byd_fps_chip_intr_state(struct spi_device *spi);

int byd_fps_chip_intr_flag_clr(struct spi_device *spi, unsigned char clear);
int byd_fps_chip_check_fng_state(struct spi_device *spi);

int byd_fps_config_sensor_poweron(struct spi_device *spi);

int byd_fps_finger_detect_cfg(struct spi_device *spi, const unsigned char *tx);
void byd_fps_finger_detect_pre_cfg(unsigned char up_down_flag,unsigned char quick_slow_flag);
void byd_fps_finger_pd_cfg(struct spi_device *spi, int val);

int byd_fps_set_finger_state(struct spi_device *spi, unsigned char on_flag);
unsigned char byd_fps_get_finger_state(struct spi_device *spi);
int byd_fps_get_finger_detect_value(struct spi_device *spi);
void byd_fps_check_finger_state(struct byd_fps_data *byd_fps);

int byd_fps_check_reg(struct spi_device *spi);

//int byd_fps_chip_read(struct spi_device *spi);
int byd_fps_chip_read(struct spi_device *spi, unsigned int recycle);

unsigned int read_throld_on(struct spi_device *spi);
unsigned int read_throld_off(struct spi_device *spi);
void byd_fps_read_finger_detect_value(struct spi_device *spi,char *buf);

/***  Module    :  byd_os  ***/
#define OS_IN_OTP_LEGACY	0xff // OS value from OTP, 2 columns compensation
#define OS_IN_OTP			0xaa // OS value from OTP, 2 columns compensation
//#define OS_BY_SCAN		0x55 // Pixel Compensation data in OTP
//#define OS_OTP_ERR		0x00 // OS acquired by scan, because OTP OS burnning error,
int byd_os_init(struct byd_fps_data *byd_fps);
void byd_os_exit(struct byd_fps_data *byd_fps);
void byd_os_start_scan(void);
int byd_os_update(struct byd_fps_data *byd_fps);
/***  Module end:  byd_os  ***/

void byd_fps_cutimage_grey(struct byd_fps_data *byd_fps);

int byd_fps_auto_wait_finger_up_down_timeout(struct byd_fps_data *byd_fps, unsigned char finger_request, int timeout);

int byd_fps_fp_get_image_data(struct byd_fps_data *byd_fps, unsigned char image_height, unsigned char image_width);
//int byd_fps_task_capture(struct byd_fps_data *byd_fps, int finger_wait);
int byd_fps_task_capture(struct byd_fps_data *byd_fps, unsigned char scan_cmd);
int byd_sys_dev_ioctl(const char *buf);
int byd_fps_alg_version_init(unsigned short chip_key,unsigned char chip_type);
int byd_fps_multi_enrol(unsigned char byd_fps_show[12]);
int byd_fps_multi_match(unsigned char byd_fps_show[12]);
int byd_fps_alg_back_proc(void);
#ifdef BYD_FPS_SYSFS
int byd_fps_show_image(char *buf, struct byd_fps_data *byd_fps);
#else
int byd_fps_show_image(struct byd_fps_data *byd_fps);
#endif

unsigned int byd_fps_read_version(struct spi_device *spi);

int byd_fps_decode_cfg(struct spi_device *spi, unsigned char dec_data);
int byd_fps_gest_cfg(struct spi_device *spi);
int byd_fps_gest_cfg_back(struct spi_device *spi);
int byd_fps_get_fpd_state(struct spi_device *spi,unsigned char gest_int_num);
void byd_fps_timer_int_clear(void);
void byd_fps_start_gest_cacu(void);
int byd_fps_gest_judge(struct byd_fps_data *byd_fps,unsigned char gest_int_num);

int byd_fps_power_restart(struct spi_device *spi,unsigned char fng_flag);

int byd_fps_int_test_cfg_back(struct spi_device*spi);

int byd_os_scan(struct byd_fps_data *byd_fps);
int byd_fps_dig_inte_set(struct byd_fps_data *byd_fps, unsigned char en_flag);

#define OTP_FPD_TH   0x1F0
#define OTP_FPD_TH_LTENGTH  6
int byd_read_otp_fpd_var(void);
int byd_fps_chip_error_state22(struct spi_device *spi);
#endif