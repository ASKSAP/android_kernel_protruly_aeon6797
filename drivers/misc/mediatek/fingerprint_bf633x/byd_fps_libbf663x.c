/*
 * File:         byd_fps_libbf663x.c
 *
 * Created:	     2015-05-05
 * Description:  BYD fingerprint IC driver library - Linux kernel independent
 *
 * Copyright (C) 2015 BYD Company Limited
 *
 */

#include "byd_algorithm.h" // must before any kernel head include
#include "byd_fps_bf66xx.h"
#include "byd_fps_libbf663x.h"

//#ifdef CONFIG_FPS12
	#include "byd_fps12_libbf663x.h"
//#else
    #include "byd_fps11_libbf663x.h"
//#endif 
#include "byd_fps22_libbf663x.h"

#ifdef BYD_FPS_ALG_IN_TZ
#include "fingerprint_qsee.h"
#endif

 
/* ============================== local constant =========================== */
/* spi buf length*/
#define BYD_FPS_SPI_1K
#define BYD_FULL_IMAGE
#define BYD_RD_2LINE
//#define BYD_FPS_RST//解决fps03b，雪花点问题 



// -------------- ioctl command -------------- //
#define BYD_FPS_KK_NOTHING               0
#define BYD_FPS_SET_CHIP_RESET           1
#define BYD_FPS_SET_CHIP_SLEEP           2
#define BYD_FPS_INTR_STATE               3
#define BYD_FPS_SET_KEY_FUNC             4
#define BYD_FPS_SET_SINGLE_CAPTURE       5
#define BYD_FPS_SET_MULTI_CAPTURE        6
#define BYD_FPS_SET_CHIP_DOWNLOAD_PARAM  7
#define BYD_FPS_SET_WAIT_FINGER_UP       8
#define BYD_FPS_SET_ABORT_WAIT           9
#define BYD_FPS_SET_FINGERPRINT_ENROL    10
#define BYD_FPS_SET_FINGERPRINT_MATCH    11
#define BYD_FPS_SET_WAIT_FINGER_DOWN     12
#define BYD_FPS_SET_DELET_TEMPLET        13
#define BYD_FPS_IOCTL_IS_FINGER_EXIST    14   //_IO(BYD_FPS_IOCTL_MAGIC_NO, 14)
#define BYD_FPS_IOCTL_IS_DATABASE_EXIST  15   //_IO(BYD_FPS_IOCTL_MAGIC_NO, 15)
#define BYD_FPS_SET_SAFE_LEVEL			 16
#define BYD_FPS_SET_FP_WAKEUP			 17
#define BYD_FPS_DELAY_SET  				 18


#define BYD_FPS_DEFAULT_FPD_THRE 6000
/* ======================== public global constant ========================= */

extern const unsigned int BYD_FPS_CONFIG_VERSION11;
extern const unsigned int BYD_FPS_CONFIG_VERSION12;
extern const unsigned int BYD_FPS_CONFIG_VERSION22;
extern const unsigned char BYD_FPS_MAX_FRAME;
extern const unsigned char BYD_FPS_IMAGE_WIDTH11;
extern const unsigned char BYD_FPS_IMAGE_HEIGHT11;
extern const unsigned int BYD_FPS_IMAGE_SIZE11;
extern const unsigned int BYD_FPS_IMAGE_BUFFER_SIZE11;

extern const unsigned char BYD_FPS_IMAGE_WIDTH12;
extern const unsigned char BYD_FPS_IMAGE_HEIGHT12;
extern const unsigned int BYD_FPS_IMAGE_SIZE12;
extern const unsigned int BYD_FPS_IMAGE_BUFFER_SIZE12;

extern unsigned char BYD_FPS_IMAGE_WIDTH;
extern unsigned char BYD_FPS_IMAGE_HEIGHT;
extern unsigned char BYD_FPS_DATA_WIDTH;
extern unsigned char BYD_FPS_DATA_HEIGHT;
extern unsigned int BYD_FPS_IMAGE_SIZE;
extern unsigned int BYD_FPS_IMAGE_BUFFER_SIZE;

extern const unsigned int BYD_FPS_DEFAULT_IRQ_TIMEOUT;
extern const unsigned char FINGER_DOWN;
extern const unsigned char FINGER_UP;
extern const unsigned char BYD_FPS_INT_MODE;
extern const unsigned int BYD_FPS_SPI_DUMMY;
extern const unsigned char BYD_FPS_16BIT_IMAGE;
extern const unsigned char BYD_FPS_CHECK_SUB_AREA;
extern const unsigned int BYD_FPS_SUB_AREA_SCAN_THRD;
extern const unsigned short BYD_FPS_BUF_SIZE;
#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
extern unsigned int byd_fps_keydown_flag;//按键的使能标志
unsigned int byd_fps_keyopen_flag = BYD_FPS_ONLY_KEY_ON;
#endif
unsigned char byd_fps_chip_flag = 0x00;

/* ======================== public global variables ======================== */
#if (defined BYD_FPS_ALG_IN_KERNEL) || (defined BYD_FPS_ALG_IN_TZ)
	extern unsigned char byd_fps_alg_ret[6];
  #ifdef BYD_FPS_ALG_IN_TZ
	extern unsigned char qsee_run;
  #endif
#endif

unsigned char finger_present;

unsigned char byd_fps_finger_up_async_flag = 0;

extern unsigned long set_timeout_finger_down;
extern unsigned long set_timeout_finger_up;
extern unsigned long set_timeout_sub_scan;
extern unsigned long set_timeout_image_scan;
extern unsigned short *byd_fps_image_data;
extern unsigned char byd_app_get_image;
extern unsigned char work_cmd;

extern unsigned char byd_last_cmd;

extern unsigned char trans_num;
extern unsigned int trans_rest;
extern unsigned char read_first;

int byd_rst_cfg = 0;

unsigned char byd_fps_sec_scan_flag;
unsigned char byd_fps_fail_reason;
unsigned char byd_alg[6] = {0,0,0,0,0,0};

extern unsigned char byd_show_flag ;
extern unsigned char byd_enrol_match_flag;

unsigned char byd_fps_ageing_test_flag = 0;
unsigned char ReceiVeCmdType;

unsigned char byd_fps_sub_value12[18] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; //读子区域数据
unsigned char byd_fps_sub_value11[10] = {0,0,0,0,0,0,0,0,0,0}; //读子区域数据
unsigned char byd_fps_sub_value22[6] = {0,0,0,0,0,0}; //读子区域数据

extern struct byd_fps_data *this_byd_fps;

#ifdef BYD_FPS_FG_DETECT_DELAY
extern unsigned char byd_fps_fg_det_state;
#endif

#ifdef BYD_FINGER_INTR_CNT
extern unsigned char finger_intr_flag;
extern unsigned char finger_intr_num;
#endif
#ifdef BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED
extern unsigned char byd_fps_key_func_flag;
extern unsigned char byd_fps_key_mesg;
void byd_fps_report_key_up(void);
#endif
extern unsigned char finger_intr_ctrl;
unsigned short otp_fpd_th = 0;//用来存储从OTP读出的指纹变化量的最小值
extern unsigned short byd_fps_fae_detect_para22[BYD_FPS_FAE_DETECT_LEN];
extern unsigned short byd_fps_fae_detect_para[5];
extern unsigned char byd_fps_fae_detect[10];
extern unsigned char byd_fps_fae_detect22[12];//add os th
extern unsigned char byd_fps_fae_image[4];
extern unsigned short byd_fps_fae_fir[64];


/* ======================== public function declare ======================== */
/* kernel dependent functions
 * THESE FUNCTIONS NEED TO BE IMPLEMENTED OUTSIDE */
 /*** interface defined by byd_fps_spi.c ***/
#define SPI_WR			1
#define SPI_RD			0
int byd_fps_spi_xfer(struct spi_device *spi, unsigned int count, unsigned char* tx, unsigned char* rx);
int byd_fps_spi_block_write(struct spi_device *spi, u8 first_reg, unsigned int count, u8 *buf);
int byd_fps_spi_read(struct spi_device *spi, u8 reg);
int byd_fps_spi_write(struct spi_device *spi, u8 reg, u8 val);

int byd_fps_suspend(struct device *dev);
int byd_fps_resume(struct device *dev);

long byd_fps_wait_event_interruptible_timeout(long timeout); // int condition, ...
long byd_fps_wait_chip_interrupt_timeout(long timeout, unsigned char * intr_state);

int byd_fps_check_thread_should_stop(void);
void byd_fps_stop_thread(unsigned char work_cmd);
void byd_fps_start_template_merging(void);
void byd_fps_msleep(unsigned int msecs);
void byd_fps_udelay(unsigned int usecs);
void byd_fps_mdelay(unsigned int msecs);
int byd_fps_spi_speed(struct spi_device *spi, int frq_khz);
int byd_fps_read_image(int count);

void byd_power_control(unsigned char down_time,unsigned char up_time);
static int byd_fps_otp_read(short int otp_addr, int addr_len, unsigned char * pbuf,unsigned char change_flag);
#ifdef CONFIG_FPS11
static int byd_fps_read_chip(void);
#endif 

/* ======================== private global variables  ====================== */
#if (defined BYD_FPS_ALG_IN_KERNEL) || (defined BYD_FPS_ALG_IN_TZ)
	unsigned int byd_fps_match_id;
#endif
unsigned short byd_fp_id;
unsigned short byd_chip_info = 0;
static unsigned short byd_chip_11 = 0;
static unsigned short byd_chip_12 = 0;
static unsigned short byd_chip_22 = 0xffff;


static unsigned short th_up = 0xffff;
extern unsigned char byd_fps_finger_up_flag;
//#define SPI_BUF_SIZE_LIMIT		1024
//#define BYD_RD_LINE_NUM			6//move to byd_fps_libbf663x.h
#if defined(BYD_RD_LINE_NUM11) || defined (BYD_RD_LINE_NUM12)
static unsigned int byd_rd_num;
static unsigned int byd_rd_rest;
#endif


//extern unsigned char g_need_scan_flag;
#ifdef BYD_FPS_UP_DOWN_ASYNC
extern unsigned char g_need_finger_down_scan_flag;
extern unsigned char g_need_finger_up_scan_flag;
#endif
/***  Module    :  byd_os  ***/
#ifdef BYD_FPS_COVER_PLATE
	 #define OS_BLOCK_DIFF              300   //每块灰度值与基线灰度值差允许范围（无手指阈值） 4倍放大系数 
	 #define OS_UNIFORMITY_DIFF         80    //九块灰度一致性误差最大允许值
#else
	 #define OS_BLOCK_DIFF              150   //每块灰度值与基线灰度值差允许范围（无手指阈值） 2倍放大系数 
	 #define OS_UNIFORMITY_DIFF         40    //九块灰度一致性误差最大允许值 
	 #define OS_UNIFORMITY_DIFF22		100
#endif



#define OS_MATCH_FAILED_MAX		10	// OS scan update rate, by counting the number of match failed
#define OS_MATCH_FAILED_MAX22	30	// OS scan update rate, by counting the number of match failed
#define OS_SCAN_FAILED_MAX		4	// the number of continuously OS scan failed, unexpected error, give warnning log

#define OS_UPDATE_FAILED_MAX				3  //失调值连续更新失败次数(前提：满足三块子区域检测)最大值，之后强制更新 可调
#define  OS_PGD_ACT                         200//平均单点灰度差值正向阈值，正差值越大表明当前采的失调图像灰度越小   可调
#define  OS_PGD_NEG							(-15)//平均单点灰度差值负向阈值，负差值越大表明当前采的失调图像灰度越大  



unsigned char Os_Otp_or_Scan  = 0;	// 0x55/0xaa/0xff
unsigned char Os_Scan_Expired = 1;	// false 0, true otherwise, must be initialized to true 
static unsigned char Match_Failed_Counter = 0;
unsigned char OS_First_Catch = 0;//首次采失调的标志，首次只需满足三块区域检测  1：代表首次   0：为非首次
//static unsigned short int byd_fps_column_ratio[96]; // deprecated
/***  Module end:  byd_os  ***/

/* ======================== private function declare ======================= */
//static int byd_fps_adj(struct spi_device *spi);
//static int byd_fps_cfg_fpd(struct spi_device *spi);
//static int byd_fps_cfg_func(struct spi_device *spi);
//static int byd_fps_finger_scan_fae(struct spi_device *spi, const unsigned char *tx);
static int byd_fps_chip_reset(struct spi_device *spi);

//static int byd_fps_finger_image_capture(struct byd_fps_data *byd_fps, int timeout);
//static int byd_fps_clear_dummy(char * img_buf, int dummy, unsigned char width, unsigned char height);

void byd_fps_mutex_lock(unsigned char lock_flag,unsigned char type_flag);

#ifdef BYD_FPS_TIMER
#ifdef BYD_TIMER_FNG_UP
int byd_timer_startup(void);
extern void byd_fps_set_timer_up(unsigned char flag);
extern int byd_fps_upif(void);
#endif

#ifdef BYD_TIMER_FNG_DOWN
void byd_fps_downif(void);
#endif
#endif

#ifdef BYD_FPS_GESTURE
void byd_fps_start_detect_gest(void);
void byd_fps_gesture_off(void);
int byd_fps_get_gest_state(void);
#endif

#ifdef BYD_FPS_INPUT_WAKEUP
void byd_fps_set_wakelock(char wakelock);
char byd_fps_get_wakelock(void);
void byd_fps_set_suspend_flag(char suspend_flag);
char byd_fps_get_suspend_flag(void);
void byd_fps_set_susp_match_cnt(char susp_match_cnt);
char byd_fps_get_susp_match_cnt(void);
void byd_fps_enable_irq_wake(struct spi_device *spi);
int byd_fps_qual_suspend(void);
#endif
#ifndef BYD_FPS_SYSFS
int byd_fps_copy_to_user(char *buf,char *buff,size_t count);
#endif
/* ======================== local function definition ====================== */

//chip register cfg.
/* *******************************************************************************
 * Function    :  byd_fps_set_fae_para
 * Description :  set fae para to the big buffer.
 * In          :  void
 * Return      :  void
   ******************************************************************************/
#ifdef BYD_FPS_BUF_CFG
int byd_fps_buf_tranfer(struct spi_device *spi)
{
	int ret;
	if(byd_fps_chip_flag == 0x11)
		ret = byd_fps_spi_xfer(spi,396,chip_cfg_func_fpd11,NULL);
	
	
	if(byd_fps_chip_flag == 0x12){
		#ifndef SPI_TRANS_4BYTE
		FP_DBG("%s:transfer start.chip_cfg_func_fpd12[0]=0x%x,chip_cfg_func_fpd12[27]=0x%x,chip_cfg_func_fpd12[28]=0x%x\n",__func__,chip_cfg_func_fpd12[0],chip_cfg_func_fpd12[27],chip_cfg_func_fpd12[28]);
		ret = byd_fps_spi_xfer(spi,394,chip_cfg_func_fpd12,NULL);
		if(ret < 0){
			pr_err("byd chip_cfg_func_fpd spi_sync failed.\n");
			return -1;
		}
		#else
		ret = byd_fps_spi_xfer(spi,396,chip_cfg_func_fpd12,NULL);
		if(ret < 0){
			pr_err("byd chip_cfg_func_fpd spi_sync failed.\n");
			return -1;
		}
		#endif
	}		
	FP_DBG("%s:transfer end\n",__func__);
	return 0;
}
void byd_fps_set_fae_para(struct spi_device *spi)
{
	#if 1
	int i,j;
	//配置FAE手指
	if(byd_fps_chip_flag == 0x11){
		for(i=311,j=0;j<10;j++){
			chip_cfg_func_fpd11[i] = byd_fps_fae_detect[j];
			i+=2;
		}
	}else if(byd_fps_chip_flag == 0x12){
		for(i=307,j=0;j<10;j++){
			chip_cfg_func_fpd12[i] = byd_fps_fae_detect[j];
			i+=2;
		}	
	}
	//配置指纹tx
	for(i=3,j=0;j<2;j++){
		chip_cfg_func_fpd[i] = byd_fps_fae_image[j];
		i+=2;
	}
	//根据积分次数选择截位
	if(byd_fps_fae_image[1]== 0x0f)
	{
		chip_cfg_func_fpd[285] =  0x12;	
	}else if((byd_fps_fae_image[1]== 0x1f)||(byd_fps_fae_image[1]== 0x2f))
	{
		chip_cfg_func_fpd[285] =  0x13;
	}
	else if(byd_fps_fae_image[1]== 0x3f)
	{
		chip_cfg_func_fpd[285] =  0x14;
	}
	for(i=289,j=2;j<4;j++){
		chip_cfg_func_fpd[i] = byd_fps_fae_image[j];
		i+=2;
	}
	
	for(i=27,j=0;j<64;j++){
		chip_cfg_func_fpd[i] = (unsigned char)byd_fps_fae_fir[j]&0x00ff;
		chip_cfg_func_fpd[i+2] = (unsigned char)((byd_fps_fae_fir[j]&0xff00) >> 8);
		i+=4;
	}
	#endif
	byd_fps_buf_tranfer(spi);
}
#endif
#ifdef CONFIG_FPS22
static int byd_fps_cfg_ana_seq22(struct spi_device *spi)
{
	int ret;

	//VAL_TX_CLK_SEL = byd_fps_fae_image[0];
	
	ret = byd_fps_spi_write(spi, REG_SENSOR_MODE22, VAL_SENSOR_MODE22);
	if(ret != 0){
		pr_err("byd REG_SENSOR_MODE22 spi_sync failed.\n");
		return -1;
	}
	
	ret = byd_fps_spi_write(spi, REG_TX_CLK_SEL22, byd_fps_fae_image[0]);
	if(ret != 0){
		pr_err("byd REG_TX_CLK_SEL spi_sync failed.\n");
		return -1;
	}
	
	ret = byd_fps_spi_write(spi, REG_TX_CLK_H_TIME22, VAL_TX_CLK_H_TIME22);
	if(ret != 0){
		pr_err("byd REG_TX_CLK_H_TIME spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_SEN_RST_TIME22, VAL_SEN_RST_TIME22);
	if(ret != 0){
		pr_err("byd REG_SEN_RST_TIME spi_sync failed.\n");
		return -1;
	}

	ret = byd_fps_spi_write(spi, REG_FPD_INIT_NUM22, VAL_FPD_INIT_NUM22);
	if(ret != 0){
		pr_err("byd REG_FPD_INIT_NUM spi_sync failed.\n");
		return -1;
	}
	
	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_fp_scan
 * Description :  set configure value to FPD of chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_fp_scan22(struct spi_device *spi)
{
	int ret;
	ret = byd_fps_spi_write(spi, REG_DIG_INTE_SEL22, byd_fps_fae_image[1]);
	if(ret != 0){
		pr_err("byd REG_DIG_INTE_SEL22 spi_sync failed.\n");
		return -1;
	}
	
	
	//Number of dummy data when read FP image when fingerprint scan.
	ret = byd_fps_spi_write(spi, REG_DUMMY_CFG_H22, VAL_DUMMY_CFG_H22);
	if(ret != 0){
		pr_err("byd REG_DIG_INTE_SEL spi_sync failed.\n");
		return -1;
	}
	
	ret = byd_fps_spi_write(spi, REG_DUMMY_CFG_L22, VAL_DUMMY_CFG_L22);
	if(ret != 0){
		pr_err("byd REG_DIG_INTE_SEL spi_sync failed.\n");
		return -1;
	}

	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_fng_scan
 * Description :  set configure value to FPD of chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_fng_scan22(struct spi_device *spi)
{
	int ret;
	//finger state scan: area set.
	//VAL_FPD_TH_ON_H = byd_fps_fae_detect[0];
	//VAL_FPD_TH_ON_L = byd_fps_fae_detect[1];
	//VAL_FPD_TH_OFF_H = byd_fps_fae_detect[2];
	//VAL_FPD_TH_OFF_L = byd_fps_fae_detect[3];

	//finger checking threshold
	ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_H22, byd_fps_fae_detect22[0]);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_ON_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_L22, byd_fps_fae_detect22[1]);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_H22, byd_fps_fae_detect22[2]);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_OFF_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_L22, byd_fps_fae_detect22[3]);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_OFF_L spi_sync failed.\n");
		return -1;
	}
	
	//finger checking threshold copy.
	ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_H2_22, byd_fps_fae_detect22[0]);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_ON_H2 spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_L2_22, byd_fps_fae_detect22[1]);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_ON_L2 spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_H2_22, byd_fps_fae_detect22[2]);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_OFF_H2 spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_L2_22, byd_fps_fae_detect22[3]);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_OFF_L2 spi_sync failed.\n");
		return -1;
	}
	
	//INT finger cfg.
	ret = byd_fps_spi_write(spi, REG_INT_FINGER_CFG22, VAL_INT_FINGER_CFG22);
	if(ret != 0){
		pr_err("byd REG_INT_MODE_SET spi_sync failed.\n");
		return -1;
	}

	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_mode
 * Description :  set configure value to FPD of chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_mode22(struct spi_device *spi)
{
	int ret;
	//VAL_FG_REST_TI1_H	= byd_fps_fae_detect[4];
	//VAL_FG_REST_TI1_L	= byd_fps_fae_detect[5];
	//VAL_FG_REST_TI2_H	= byd_fps_fae_detect[6];
	//VAL_FG_REST_TI2_L	= byd_fps_fae_detect[7];
	//VAL_NO_FINGER_NUM_H = byd_fps_fae_detect[8];
	//VAL_NO_FINGER_NUM_L = byd_fps_fae_detect[9];
	/*
	//Chip mode.
	ret = byd_fps_spi_write(spi, REG_CHIP_MODE, VAL_CHIP_MODE);
	if(ret != 0){
		pr_err("byd REG_CHIP_MODE spi_sync failed.\n");
		return -1;
	}
	*/

	//FP timers set.
	//T3
	/*ret = byd_fps_spi_write(spi, REG_FG_REST_TI1_H, byd_fps_fae_detect[4]);
	if(ret != 0){
		pr_err("byd REG_FG_REST_TI1_H spi_sync failed.\n");
		return -1;
	}*/
	ret = byd_fps_spi_write(spi, REG_FG_REST_TI1_22, byd_fps_fae_detect22[5]);
	if(ret != 0){
		pr_err("byd REG_FG_REST_TI1 spi_sync failed.\n");
		return -1;
	}
	
	//T4
	/*ret = byd_fps_spi_write(spi, REG_FG_REST_TI2_H, byd_fps_fae_detect[6]);
	if(ret != 0){
		pr_err("byd REG_FG_REST_TI2_H spi_sync failed.\n");
		return -1;
	}*/
	ret = byd_fps_spi_write(spi, REG_FG_REST_TI2_22, byd_fps_fae_detect22[7]);
	if(ret != 0){
		pr_err("byd REG_FG_REST_TI2 spi_sync failed.\n");
		return -1;
	}
	
	//no Finger time-out.
	/*ret = byd_fps_spi_write(spi, REG_NO_FINGER_NUM, byd_fps_fae_detect[9]);
	if(ret != 0){
		pr_err("byd REG_NO_FINGER_NUM spi_sync failed.\n");
		return -1;
	}*/
	
	ret = byd_fps_spi_write(spi, REG_NO_FINGER_NUM22, VAL_NO_FINGER_NUM22);
	if(ret != 0){
		pr_err("byd REG_NO_FINGER_NUM spi_sync failed.\n");
		return -1;
	}

	//os_th, threshold of OS.
	#if 0
	ret = byd_fps_spi_write(spi, REG_OS_TH_H22, VAL_OS_TH_H22);
	if(ret != 0){
		pr_err("byd REG_OS_TH_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_OS_TH_L22, VAL_OS_TH_L22);
	if(ret != 0){
		pr_err("byd REG_OS_TH_L spi_sync failed.\n");
		return -1;
	}
	#endif
	ret = byd_fps_spi_write(spi, REG_OS_TH_H22, byd_fps_fae_detect22[10]);
	if(ret != 0){
		pr_err("byd REG_OS_TH_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_OS_TH_L22, byd_fps_fae_detect22[11]);
	if(ret != 0){
		pr_err("byd REG_OS_TH_L spi_sync failed.\n");
		return -1;
	}
	
	//initiate time, when finger scanning.
	ret = byd_fps_spi_write(spi, REG_FG_INIT_TIME22, VAL_FG_INIT_TIME22);
	if(ret != 0){
		pr_err("byd REG_FG_INIT_TIME spi_sync failed.\n");
		return -1;
	}
	
	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_system
 * Description :  set configure value to FPD of chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_system22(struct spi_device *spi)
{
	int ret;
	//

	//Interrupt cfg.
	ret = byd_fps_spi_write(spi, REG_INT_CFG22, VAL_INT_CFG22);
	if(ret != 0){
		pr_err("byd REG_INT_CFG spi_sync failed.\n");
		return -1;
	}
	
	//IO state select.
	ret = byd_fps_spi_write(spi, REG_IO_STATE_SEL22, VAL_IO_STATE_SEL22);
	if(ret != 0){
		pr_err("byd REG_IO_STATE_SEL spi_sync failed.\n");
		return -1;
	}
	
	//analogous A power-down set.
	ret = byd_fps_spi_write(spi, REG_PD_ANA_A22, VAL_PD_ANA_A22_2V);
	if(ret != 0){
		pr_err("byd REG_PD_ANA_A spi_sync failed.\n");
		return -1;
	}
	//analogous B power-down set.
	ret = byd_fps_spi_write(spi, REG_PD_ANA_B22, VAL_PD_ANA_B22_2V);
	if(ret != 0){
		pr_err("byd REG_PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	
	//Finger detect area set. 0: one sub-block; 1: three sub-blocks.
	ret = byd_fps_spi_write(spi, REG_SUB_SEL22, VAL_SUB_SEL22);
	if(ret != 0){
		pr_err("byd REG_SUB_SEL spi_sync failed.\n");
		return -1;
	}
	
	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_ana_para
 * Description :  set configure value to chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_ana_para22(struct spi_device *spi)
{
	int ret;
	//ADC SH CFG
	ret = byd_fps_spi_write(spi, REG_ADC_SH_CFG22, VAL_ADC_SH_CFG22);
	if(ret != 0){
		pr_err("byd REG_ADC_SH_CFG spi_sync failed.\n");
		return -1;
	}
	
	//CP VDDS select.
	ret = byd_fps_spi_write(spi, REG_SEL_CP_VDDS22, VAL_SEL_CP_VDDS22);
	if(ret != 0){
		pr_err("byd REG_SEL_CP_VDDS spi_sync failed.\n");
		return -1;
	}

	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_abnorm_status
 * Description :  set configure value to chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_abnorm_status22(struct spi_device *spi)
{
	int ret;
	ret = byd_fps_spi_write(spi, REG_CHECK_ERROR_EN22, VAL_CHECK_ERROR_EN22);
	if(ret != 0){
		pr_err("byd REG_CHECK_ERROR_EN spi_sync failed.\n");
		return -1;
	}
	
	ret = byd_fps_spi_write(spi, REG_CHECK_ERROR_TIME22, VAL_CHECK_ERROR_TIME22);
	if(ret != 0){
		pr_err("byd REG_SEL_CP_VDDS spi_sync failed.\n");
		return -1;
	}
		
	return 0;
}

//读otp的模拟配置参数:12个
#define OTP_ANA_START  0x60
#define OTP_ANA_LENGTH  12
static int byd_read_otp_ana22(unsigned char *ana_seq_buf)//
{
	int ret,i;
	ret = byd_fps_otp_read(OTP_ANA_START,OTP_ANA_LENGTH,ana_seq_buf,0);
	DBG_TIME("%s:byd OTP ana_seq_buf byte:",__func__);
	for(i=0;i<OTP_ANA_LENGTH;i++) {
		DBG_TIME("byd[%d]=0x%x,",i,ana_seq_buf[i]);
		/*if((i%8) == 0) {
			DBG_TIME("\n");
		}*/
	}
	DBG_TIME("\n");
	
	return ret;
}
static int byd_fps_cfg_from_otp_ana22(void)
{
	struct spi_device * spi = this_byd_fps->spi;
	unsigned char ana_seq_buf[12];
	int ret,otp_write_flag=12,i;
	
	byd_fps_chip_idle(spi);
	byd_read_otp_ana22(ana_seq_buf);
	for (i=0;i<OTP_ANA_LENGTH;i++) {
		if(ana_seq_buf[i] == 0xff)
				otp_write_flag--;		
	}
	DBG_TIME("%s:otp_write_flag= %d\n",__func__,otp_write_flag);
	if (otp_write_flag == 0) {//如果otp没写如下模拟参数，就按之前配的为准
		//可以通过查看Vtx电压的配置知道Vtx电压，进而配置寄存器，Vtx==1.7V，REG_ADJ_SFR_DATA ！=0x09;Vtx==2V，REG_ADJ_SFR_DATA ==0x09;
		ret = byd_fps_spi_write(spi,REG_ADJ_SFR_ADDR22,REG_ADJ_VTX_SEL22);
		if(ret != 0){
			pr_err("byd REG_ADJ_SFR_ADDR22 spi_sync failed.\n");
			return -1;
	    }
		ret = byd_fps_spi_read(spi,REG_ADJ_SFR_DATA22);
		byd_fps_mdelay(2);
		DBG_TIME("byd REG_ADJ_VTX_SEL22=0x%x\n",ret);
		if (ret !=0x09) {//0x09表示vtx用的电压是2.0v，则之相反
			ret = byd_fps_spi_write(spi,REG_PD_ANA_B22,VAL_PD_ANA_A22_1P7V);//VAL_PD_ANA_B 配成1.7v
			DBG_TIME("byd REG_PD_ANA_B22=0x%x\n",ret);
			if(ret != 0){
				pr_err("byd REG_PD_ANA_B spi_sync failed.\n");
				return -1;
			}
			ret = byd_fps_spi_write(spi,REG_PD_ANA_A22,VAL_PD_ANA_B22_1P7V);//
			DBG_TIME("byd REG_PD_ANA_A22=0x%x\n",ret);
			if(ret != 0){
				pr_err("byd REG_PD_ANA_A22 spi_sync failed.\n");
				return -1;
			}
		}
		return 0;
	}else if(otp_write_flag == 12){
		
    ret = byd_fps_spi_write(spi, REG_SENSOR_MODE22,ana_seq_buf[0] );//VAL_SENSOR_MODE,ana_seq_buf[0]
	if(ret != 0){
		pr_err("byd REG_SENSOR_MODE spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_TX_CLK_SEL22,ana_seq_buf[1] );//byd_fps_fae_image[0]
	if(ret != 0){
		pr_err("byd REG_TX_CLK_SEL spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_TX_CLK_H_TIME22, ana_seq_buf[2]);//VAL_TX_CLK_H_TIME
	if(ret != 0){
		pr_err("byd REG_TX_CLK_H_TIME spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_SEN_RST_TIME22, ana_seq_buf[3]);//VAL_SEN_RST_TIME
	if(ret != 0){
		pr_err("byd REG_SEN_RST_TIME spi_sync failed.\n");
		return -1;
	}

	ret = byd_fps_spi_write(spi, REG_FPD_INIT_NUM22,ana_seq_buf[4] );//VAL_FPD_INIT_NUM
	if(ret != 0){
		pr_err("byd REG_FPD_INIT_NUM spi_sync failed.\n");
		return -1;
	}
    ret = byd_fps_spi_write(spi,REG_FPD_TEST_MODE22,ana_seq_buf[5]);//VAL_FPD_TEST_MODE
	if(ret != 0){
		pr_err("byd REG_FPD_TEST_MODE spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi,REG_FPD_TEST_CFG22,ana_seq_buf[6]);//VAL_FPD_TEST_CFG
		if(ret != 0){
		pr_err("byd REG_FPD_TEST_CFG spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi,REG_FG_INIT_TIME22,ana_seq_buf[7]);//VAL_FG_INIT_TIME
		if(ret != 0){
		pr_err("byd REG_FG_INIT_TIME spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi,REG_ADC_SH_CFG22,ana_seq_buf[8]);//VAL_ADC_SH_CFG
		if(ret != 0){
		pr_err("byd REG_ADC_SH_CFG spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi,REG_SEL_CP_VDDS22,ana_seq_buf[9]);//VAL_SEL_CP_VDDS
		if(ret != 0){
		pr_err("byd REG_SEL_CP_VDDS spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi,REG_PD_ANA_A22,ana_seq_buf[10]);//VAL_PD_ANA_A
		if(ret != 0){
		pr_err("byd REG_PD_ANA_A spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi,REG_PD_ANA_B22,ana_seq_buf[11]);//VAL_PD_ANA_B
		if(ret != 0){
		pr_err("byd REG_PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	printk("%s:open check error\n",__func__);
	#if 1
	ret = byd_fps_spi_write(spi,REG_CHECK_ERROR_EN22,0x77);//打开芯片纠错功能
	if(ret != 0){
		pr_err("byd REG_CHECK_ERROR_EN spi_sync failed.\n");
		return -1;
	}
	#endif
	}
	return 0;
}

int byd_fps_cfg_reg_module22(void)
{
	struct spi_device *spi = this_byd_fps->spi;
	
	if(spi) {
		byd_fps_chip_idle(spi);
	
		byd_fps_cfg_ana_seq22(spi);
		byd_fps_cfg_fp_scan22(spi);
		byd_fps_cfg_fng_scan22(spi);
		byd_fps_cfg_mode22(spi);
		byd_fps_cfg_system22(spi);
		byd_fps_cfg_ana_para22(spi);
		byd_fps_cfg_abnorm_status22(spi);
		
		byd_fps_cfg_from_otp_ana22();
	}
	
	
	return 0;
}
#endif
//chip register cfg.
/* *******************************************************************************
 * Function    :  byd_fps_cfg_ana_seq
 * Description :  set configure value to FPD of chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
  ******************************************************************************/

#ifdef BYD_FPS_CFG_SEPERATE

static int byd_fps_cfg_ana_seq(struct spi_device *spi)
{
	int ret;

	VAL_TX_CLK_SEL11 = byd_fps_fae_image[0];
	VAL_TX_CLK_SEL12 = byd_fps_fae_image[0];
	ret = byd_fps_spi_write(spi, REG_SENSOR_MODE, VAL_SENSOR_MODE);
	if(ret != 0){
		pr_err("byd REG_SENSOR_MODE spi_sync failed.\n");
		return -1;
	}
	if(byd_fps_chip_flag == 0x11){
		ret = byd_fps_spi_write(spi, REG_TX_CLK_SEL, VAL_TX_CLK_SEL11);
		if(ret != 0){
			pr_err("byd REG_TX_CLK_SEL spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_TX_CLK_H_TIME, VAL_TX_CLK_H_TIME11);
		if(ret != 0){
			pr_err("byd REG_TX_CLK_H_TIME spi_sync failed.\n");
			return -1;
		}
	}
	if(byd_fps_chip_flag == 0x12){
		ret = byd_fps_spi_write(spi, REG_TX_CLK_SEL, VAL_TX_CLK_SEL12);
		if(ret != 0){
			pr_err("byd REG_TX_CLK_SEL spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_TX_CLK_H_TIME, VAL_TX_CLK_H_TIME12);
		if(ret != 0){
			pr_err("byd REG_TX_CLK_H_TIME spi_sync failed.\n");
			return -1;
		}
	}
	ret = byd_fps_spi_write(spi, REG_SEN_RST_TIME, VAL_SEN_RST_TIME);
	if(ret != 0){
		pr_err("byd REG_SEN_RST_TIME spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_INIT_TX_SEL, VAL_INIT_TX_SEL);
	if(ret != 0){
		pr_err("byd REG_INIT_TX_SEL spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FP_INIT_NUM, VAL_FP_INIT_NUM);
	if(ret != 0){
		pr_err("byd REG_FP_INIT_NUM spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FP_INIT_ROW, VAL_FP_INIT_ROW);
	if(ret != 0){
		pr_err("byd REG_FP_INIT_ROW spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FP_INIT_COL, VAL_FP_INIT_COL);
	if(ret != 0){
		pr_err("byd REG_FP_INIT_COL spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FG_INIT_NUM, VAL_FG_INIT_NUM);
	if(ret != 0){
		pr_err("byd REG_FG_INIT_NUM spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FG_INIT_ROW, VAL_FG_INIT_ROW);
	if(ret != 0){
		pr_err("byd REG_FG_INIT_ROW spi_sync failed.\n");
		return -1;
	}
	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_fp_scan
 * Description :  set configure value to FPD of chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_fp_scan(struct spi_device *spi)
{
	int ret;
	unsigned char i;
	VAL_DIG_INTE_SEL = byd_fps_fae_image[1];
	VAL_READ_ROW_CFG0 = byd_fps_fae_image[2];
	VAL_READ_ROW_CFG1 = byd_fps_fae_image[3];
	//FIR cfg.
	ret = byd_fps_spi_write(spi, REG_FPD_TEST_MODE, 0x00);
	if(ret != 0){
		pr_err("byd REG_FPD_TEST_MODE spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FIR_CMD, 0x63);
	if(ret != 0){
		pr_err("byd REG_FIR_CMD spi_sync failed.\n");
		return -1;
	}
	for(i=0;i<(VAL_DIG_INTE_SEL+1);i++) {
		//FIR_DATA_L
		ret = byd_fps_spi_write(spi, REG_FIR_DATA, (unsigned char)(byd_fps_fae_fir[i]&0x00FF));
		if(ret != 0){
			pr_err("byd REG_FIR_DATA_L spi_sync failed.\n");
			return -1;
		}
		//FIR_DATA_H
		ret = byd_fps_spi_write(spi, REG_FIR_DATA, (unsigned char)((byd_fps_fae_fir[i]>>8)&0x00FF));
		if(ret != 0){
			pr_err("byd REG_FIR_DATA_H spi_sync failed.\n");
			return -1;
		}
	}
	
	//#ifdef DEBUG
	#if 0
	//Read FIR
	ret = byd_fps_spi_write(spi, REG_FPD_TEST_MODE, 0x00);
	if(ret != 0){
		pr_err("byd REG_FPD_TEST_MODE spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FIR_CMD, 0x66);
	if(ret != 0){
		pr_err("byd REG_FIR_CMD spi_sync failed.\n");
		return -1;
	}
	for(i=0;i<(VAL_DIG_INTE_SEL+1);i++) {
		ret = byd_fps_spi_read(spi, REG_FIR_DATA);
		if(ret < 0){
			pr_err("byd REG_FIR_DATA_L spi_sync failed.\n");
			return -1;
		}
		DBG_TIME("byd FIR[%d]_L=0x%x\n", i, ret);
		ret = byd_fps_spi_read(spi, REG_FIR_DATA);
		if(ret < 0){
			pr_err("byd REG_FIR_DATA_H spi_sync failed.\n");
			return -1;
		}
		DBG_TIME("byd FIR[%d]_H=0x%x\n", i, ret);
	}
	#endif
	
	//digital integrate num of fingerprint scan.
	ret = byd_fps_spi_write(spi, REG_DIG_INTE_SEL, VAL_DIG_INTE_SEL);
	if(ret != 0){
		pr_err("byd REG_DIG_INTE_SEL spi_sync failed.\n");
		return -1;
	}
	#define VAL_FRAME_ADD_NUM	0x00
	//frame added num of fingerprint scan.
	ret = byd_fps_spi_write(spi, REG_FRAME_ADD_NUM, VAL_FRAME_ADD_NUM);
	if(ret != 0){
		pr_err("byd REG_FRAME_ADD_NUM spi_sync failed.\n");
		return -1;
	}
	
	//scan data cut .
	ret = byd_fps_spi_write(spi, REG_SCAN_DATA_CUT_SEL, VAL_SCAN_DATA_CUT_SEL);
	if(ret != 0){
		pr_err("byd REG_SCAN_DATA_CUT_SEL spi_sync failed.\n");
		return -1;
	}
	#define VAL_FRAME_ADD_CUT_SEL	0x00
	//scan frame added data cut .
	ret = byd_fps_spi_write(spi, REG_FRAME_ADD_CUT_SEL, VAL_FRAME_ADD_CUT_SEL);
	if(ret != 0){
		pr_err("byd REG_FRAME_ADD_CUT_SEL spi_sync failed.\n");
		return -1;
	}
	//unsigned char  VAL_READ_ROW_CFG0 = byd_fps_fae_image[2];//0x5f
	//read row cfg .
	ret = byd_fps_spi_write(spi, REG_READ_ROW_CFG0, VAL_READ_ROW_CFG0);
	if(ret != 0){
		pr_err("byd REG_READ_ROW_CFG0 spi_sync failed.\n");
		return -1;
	}
	//unsigned char VAL_READ_ROW_CFG1 = byd_fps_fae_image[3];//0x5f
	ret = byd_fps_spi_write(spi, REG_READ_ROW_CFG1, VAL_READ_ROW_CFG1);
	if(ret != 0){
		pr_err("byd REG_READ_ROW_CFG1 spi_sync failed.\n");
		return -1;
	}
	
	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_fg_scan
 * Description :  set configure value to FPD of chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_fg_scan(struct spi_device *spi)
{
	int ret;
	//finger state scan: area set.
	VAL_FPD_TH_ON_H = byd_fps_fae_detect[0];
	VAL_FPD_TH_ON_L = byd_fps_fae_detect[1];
	VAL_FPD_TH_OFF_H = byd_fps_fae_detect[2];
	VAL_FPD_TH_OFF_L = byd_fps_fae_detect[3];

	ret = byd_fps_spi_write(spi, REG_FG_ROW_START_A, VAL_FG_ROW_START_A);
	if(ret != 0){
		pr_err("byd REG_FG_ROW_START_A spi_sync failed.\n");
		return -1;
	}
	
	//#ifdef CONFIG_FPS12
	if(byd_fps_chip_flag == 0x12){
		ret = byd_fps_spi_write(spi, REG_FG_ROW_START_B, VAL_FG_ROW_START_B12);
		if(ret != 0){
			pr_err("byd REG_FG_ROW_START_B spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_FG_ROW_START_C, VAL_FG_ROW_START_C12);
		if(ret != 0){
			pr_err("byd REG_FG_ROW_START_C spi_sync failed.\n");
			return -1;
		}
	
		ret = byd_fps_spi_write(spi, REG_FG_ROW_NUM_AB, VAL_FG_ROW_NUM_AB);
		if(ret != 0){
			pr_err("byd REG_FG_ROW_NUM_AB spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_FG_ROW_NUM_C, VAL_FG_ROW_NUM_C);
		if(ret != 0){
			pr_err("byd REG_FG_ROW_NUM_C spi_sync failed.\n");
			return -1;
		}
	}
	//#else
	if(byd_fps_chip_flag == 0x11){
		ret = byd_fps_spi_write(spi, REG_FG_ROW_START_B, VAL_FG_ROW_START_B11);
		if(ret != 0){
			pr_err("byd REG_FG_ROW_START_B spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_FG_ROW_START_C, VAL_FG_ROW_START_C11);
		if(ret != 0){
			pr_err("byd REG_FG_ROW_START_C spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_FG_ROW_START_D, VAL_FG_ROW_START_D);
		if(ret != 0){
			pr_err("byd REG_FG_ROW_START_D spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_FG_ROW_START_E, VAL_FG_ROW_START_E);
		if(ret != 0){
			pr_err("byd REG_FG_ROW_START_E spi_sync failed.\n");
			return -1;
		}

		//enable row(A,B,C,D,E)
		ret = byd_fps_spi_write(spi, REG_SUB_AREA_SEL, VAL_REG_SUB_AREA_SEL);
		if(ret != 0){
			pr_err("byd REG_SUB_AREA_SEL spi_sync failed.\n");
			return -1;
		}
		
		//num of row (A,B,C,D,E)
		ret = byd_fps_spi_write(spi, REG_FG_ROW_NUM, VAL_FG_ROW_NUM);
		if(ret != 0){
			pr_err("byd REG_FG_ROW_NUM spi_sync failed.\n");
			return -1;
		}
	}
	//#endif
	
	//digital integrate num of Finger state check.
	ret = byd_fps_spi_write(spi, REG_FINGER_INTE_SEL, VAL_FINGER_INTE_SEL);
	if(ret != 0){
		pr_err("byd REG_FINGER_INTE_SEL spi_sync failed.\n");
		return -1;
	}
	//Sub area data cut .
	ret = byd_fps_spi_write(spi, REG_SUB_CUT_SEL, VAL_SUB_CUT_SEL);
	if(ret != 0){
		pr_err("byd REG_SUB_CUT_SEL spi_sync failed.\n");
		return -1;
	}
	//finger checking threshold
	ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_H, VAL_FPD_TH_ON_H);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_ON_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_L, VAL_FPD_TH_ON_L);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_H, VAL_FPD_TH_OFF_H);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_OFF_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_L, VAL_FPD_TH_OFF_L);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_OFF_L spi_sync failed.\n");
		return -1;
	}
	if(byd_fps_chip_flag == 0x11){
		//INT mode set.
		ret = byd_fps_spi_write(spi, REG_INT_MODE_SET, VAL_INT_MODE_SET11);
		if(ret != 0){
			pr_err("byd REG_INT_MODE_SET spi_sync failed.\n");
			return -1;
		}
		//Sub area position select.
		ret = byd_fps_spi_write(spi, REG_SUB_SET_L, VAL_SUB_SET_L11);
		if(ret != 0){
			pr_err("byd REG_SUB_SET_L spi_sync failed.\n");
			return -1;
		}
	}
	if(byd_fps_chip_flag == 0x12){
		//INT mode set.
		ret = byd_fps_spi_write(spi, REG_INT_MODE_SET, VAL_INT_MODE_SET12);
		if(ret != 0){
			pr_err("byd REG_INT_MODE_SET spi_sync failed.\n");
			return -1;
		}
		//Sub area position select.
		ret = byd_fps_spi_write(spi, REG_SUB_SET_L, VAL_SUB_SET_L12);
		if(ret != 0){
			pr_err("byd REG_SUB_SET_L spi_sync failed.\n");
			return -1;
		}
	}
	//#ifdef CONFIG_FPS12
	if(byd_fps_chip_flag == 0x12){
		ret = byd_fps_spi_write(spi, REG_SUB_SET_H, VAL_SUB_SET_H);
		if(ret != 0){
			pr_err("byd REG_SUB_SET_H spi_sync failed.\n");
			return -1;
		}
	}
	//#endif 
	//INT FINGER NUM
	ret = byd_fps_spi_write(spi, REG_INT_FINGER_NUM, VAL_INT_FINGER_NUM);
	if(ret != 0){
		pr_err("byd REG_INT_FINGER_NUM spi_sync failed.\n");
		return -1;
	}
	return 0;
}

#if 0
/* *******************************************************************************
 * Function    :  byd_fps_fpd_test
 * Description :  set configure value to chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
#define VAL_FPD_TEST_MODE				0
#define VAL_TEST_FIR_CTRL				0

static int byd_fps_fpd_test(struct spi_device *spi)
{
	int ret;
	//Test mode.
	ret = byd_fps_spi_write(spi, REG_FPD_TEST_MODE, VAL_FPD_TEST_MODE);
	if(ret != 0){
		pr_err("byd REG_SENSOR_MODE spi_sync failed.\n");
		return -1;
	}
	/*
	//Read value of ADC.
	ret = byd_fps_spi_read(spi, REG_ADC_DP_DATA_H);
	if(ret < 0){
		pr_err("byd REG_ADC_DP_DATA_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_read(spi, REG_ADC_DP_DATA_L);
	if(ret < 0){
		pr_err("byd REG_ADC_DP_DATA_L spi_sync failed.\n");
		return -1;
	}
	*/
	//Test FIR CTRL.
	ret = byd_fps_spi_write(spi, REG_TEST_FIR_CTRL, VAL_TEST_FIR_CTRL);
	if(ret != 0){
		pr_err("byd REG_TEST_FIR_CTRL spi_sync failed.\n");
		return -1;
	}
	return 0;
}
#endif

/* *******************************************************************************
 * Function    :  byd_fps_cfg_mode
 * Description :  set configure value to FPD of chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_mode(struct spi_device *spi)
{
	int ret;
	VAL_FG_REST_TI1_H	= byd_fps_fae_detect[4];
	VAL_FG_REST_TI1_L	= byd_fps_fae_detect[5];
	VAL_FG_REST_TI2_H	= byd_fps_fae_detect[6];
	VAL_FG_REST_TI2_L	= byd_fps_fae_detect[7];
	VAL_NO_FINGER_NUM_H = byd_fps_fae_detect[8];
	VAL_NO_FINGER_NUM_L = byd_fps_fae_detect[9];
	//Chip mode.
	ret = byd_fps_spi_write(spi, REG_CHIP_MODE, VAL_CHIP_MODE);
	if(ret != 0){
		pr_err("byd REG_CHIP_MODE spi_sync failed.\n");
		return -1;
	}

	//Scan_cmd.
	ret = byd_fps_spi_write(spi, REG_SCAN_CMD, VAL_SCAN_CMD);
	if(ret != 0){
		pr_err("byd REG_SCAN_CMD spi_sync failed.\n");
		return -1;
	}
	//FPD sensor area(0,1) set.
	//area 0
	ret = byd_fps_spi_write(spi, REG_AREA0_ROW_START, VAL_AREA0_ROW_START);
	if(ret != 0){
		pr_err("byd REG_AREA0_ROW_START spi_sync failed.\n");
		return -1;
	}
	
	ret = byd_fps_spi_write(spi, REG_AREA0_COL_START, VAL_AREA0_COL_START);
	if(ret != 0){
		pr_err("byd REG_AREA0_COL_START spi_sync failed.\n");
		return -1;
	}
	if(byd_fps_chip_flag == 0x11){
		ret = byd_fps_spi_write(spi, REG_AREA0_ROW_END, VAL_AREA0_ROW_END11);
		if(ret != 0){
			pr_err("byd REG_AREA0_ROW_END spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_AREA0_COL_END, VAL_AREA0_COL_END11);
		if(ret != 0){
			pr_err("byd REG_AREA0_COL_END spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_AREA1_ROW_END, VAL_AREA1_ROW_END11);
		if(ret != 0){
			pr_err("byd REG_AREA0_ROW_END spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_AREA1_COL_END, VAL_AREA1_COL_END11);
		if(ret != 0){
			pr_err("byd REG_AREA0_COL_END spi_sync failed.\n");
			return -1;
		}
	}
	if(byd_fps_chip_flag == 0x12){
		ret = byd_fps_spi_write(spi, REG_AREA0_ROW_END, VAL_AREA0_ROW_END12);
		if(ret != 0){
			pr_err("byd REG_AREA0_ROW_END spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_AREA0_COL_END, VAL_AREA0_COL_END12);
		if(ret != 0){
			pr_err("byd REG_AREA0_COL_END spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_AREA1_ROW_END, VAL_AREA1_ROW_END12);
		if(ret != 0){
			pr_err("byd REG_AREA0_ROW_END spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_AREA1_COL_END, VAL_AREA1_COL_END12);
		if(ret != 0){
			pr_err("byd REG_AREA0_COL_END spi_sync failed.\n");
			return -1;
		}
	}
	//area 1
	ret = byd_fps_spi_write(spi, REG_AREA1_ROW_START, VAL_AREA1_ROW_START);
	if(ret != 0){
		pr_err("byd REG_AREA0_ROW_START spi_sync failed.\n");
		return -1;
	}
	
	ret = byd_fps_spi_write(spi, REG_AREA1_COL_START, VAL_AREA1_COL_START);
	if(ret != 0){
		pr_err("byd REG_AREA0_COL_START spi_sync failed.\n");
		return -1;
	}
	
	
	//FP timers set.
	//T1
	ret = byd_fps_spi_write(spi, REG_FP_WAIT_TI_H, VAL_FP_WAIT_TI_H);
	if(ret != 0){
		pr_err("byd REG_FP_WAIT_TI_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FP_WAIT_TI_L, VAL_FP_WAIT_TI_L);
	if(ret != 0){
		pr_err("byd REG_FP_WAIT_TI_L spi_sync failed.\n");
		return -1;
	}
	//T2
	ret = byd_fps_spi_write(spi, REG_OS_WAIT_TI_H, VAL_OS_WAIT_TI_H);
	if(ret != 0){
		pr_err("byd REG_OS_WAIT_TI_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_OS_WAIT_TI_L, VAL_OS_WAIT_TI_L);
	if(ret != 0){
		pr_err("byd REG_OS_WAIT_TI_L spi_sync failed.\n");
		return -1;
	}
	//T4 Enable when finger state check.
	ret = byd_fps_spi_write(spi, REG_FG_REST_TI1_EN, VAL_FG_REST_TI1_EN);
	if(ret != 0){
		pr_err("byd REG_FG_REST_TI1_EN spi_sync failed.\n");
		return -1;
	}
	//T3
	ret = byd_fps_spi_write(spi, REG_FG_REST_TI1_H, VAL_FG_REST_TI1_H);
	if(ret != 0){
		pr_err("byd REG_FG_REST_TI1_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FG_REST_TI1_L, VAL_FG_REST_TI1_L);
	if(ret != 0){
		pr_err("byd REG_FG_REST_TI1_L spi_sync failed.\n");
		return -1;
	}
	//T4
	ret = byd_fps_spi_write(spi, REG_FG_REST_TI2_H, VAL_FG_REST_TI2_H);
	if(ret != 0){
		pr_err("byd REG_FG_REST_TI2_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FG_REST_TI2_L, VAL_FG_REST_TI2_L);
	if(ret != 0){
		pr_err("byd REG_FG_REST_TI2_L spi_sync failed.\n");
		return -1;
	}
	//T5
	ret = byd_fps_spi_write(spi, REG_FP_CYCLE_REST_TI, VAL_FP_CYCLE_REST_TI);
	if(ret != 0){
		pr_err("byd REG_FP_CYCLE_REST_TI spi_sync failed.\n");
		return -1;
	}
	//T6
	ret = byd_fps_spi_write(spi, REG_FP_SINGLE_REST_TI, VAL_FG_SINGLE_REST_TI);
	if(ret != 0){
		pr_err("byd REG_FP_SINGLE_REST_TI spi_sync failed.\n");
		return -1;
	}
	//no Finger time-out.
	ret = byd_fps_spi_write(spi, REG_NO_FINGER_NUM_H, VAL_NO_FINGER_NUM_H);
	if(ret != 0){
		pr_err("byd REG_NO_FINGER_NUM_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_NO_FINGER_NUM_L, VAL_NO_FINGER_NUM_L);
	if(ret != 0){
		pr_err("byd REG_NO_FINGER_NUM_L spi_sync failed.\n");
		return -1;
	}
	//os_th, threshold of OS.
	ret = byd_fps_spi_write(spi, REG_OS_TH_H, VAL_OS_TH_H);
	if(ret != 0){
		pr_err("byd REG_OS_TH_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_OS_TH_L, VAL_OS_TH_L);
	if(ret != 0){
		pr_err("byd REG_OS_TH_L spi_sync failed.\n");
		return -1;
	}
	//initiate time, when finger scanning.
	ret = byd_fps_spi_write(spi, REG_FG_INIT_TIME, VAL_FG_INIT_TIME);
	if(ret != 0){
		pr_err("byd REG_FG_INIT_TIME spi_sync failed.\n");
		return -1;
	}
	//fg interrupt enable, when finger scanning & FP data scan.
	ret = byd_fps_spi_write(spi, REG_FP_INT_FINGER_EN, VAL_FP_INT_FINGER_EN);
	if(ret != 0){
		pr_err("byd REG_FP_INT_FINGER_EN spi_sync failed.\n");
		return -1;
	}
	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_system
 * Description :  set configure value to FPD of chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_system(struct spi_device *spi)
{
	int ret;
	unsigned char val_temp;
	//soft_rst, int_state,etc
	//cryptograph set. 0--加密, 1--不加密.
	#ifdef BYD_FPS_DECODE_IMAGE
	val_temp = 0x00;//VAL_FUNCTION_SEL;
	#else
	val_temp = 0x01;
	#endif
	ret = byd_fps_spi_write(spi, REG_FUNCTION_SEL, val_temp);
	if(ret != 0){
		pr_err("byd REG_FUNCTION_SEL spi_sync failed.\n");
		return -1;
	}
	//Interrupt cfg.
	ret = byd_fps_spi_write(spi, REG_INT_CFG, VAL_INT_CFG);
	if(ret != 0){
		pr_err("byd REG_INT_CFG spi_sync failed.\n");
		return -1;
	}
	//IO state select.
	ret = byd_fps_spi_write(spi, REG_IO_STATE_SEL, VAL_IO_STATE_SEL);
	if(ret != 0){
		pr_err("byd REG_IO_STATE_SEL spi_sync failed.\n");
		return -1;
	}
	//analogous a power-down set.
	ret = byd_fps_spi_write(spi, REG_PD_ANA_A, VAL_PD_ANA_A);
	if(ret != 0){
		pr_err("byd REG_PD_ANA_A spi_sync failed.\n");
		return -1;
	}
	//analogous b power-down set.
	ret = byd_fps_spi_write(spi, REG_PD_ANA_B, VAL_PD_ANA_B);
	if(ret != 0){
		pr_err("byd REG_PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_cfg_ana_para
 * Description :  set configure value to chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_cfg_ana_para(struct spi_device *spi)
{
	int ret;
	//adc偏置电流选择
	ret = byd_fps_spi_write(spi, REG_ADC_I_SEL, VAL_ADC_I_SEL);
	if(ret != 0){
		pr_err("byd REG_ADC_I_SEL spi_sync failed.\n");
		return -1;
	}
	//sensor buf current.
	ret = byd_fps_spi_write(spi, REG_SEN_BUF_I, VAL_SEN_BUF_I);
	if(ret != 0){
		pr_err("byd REG_SEN_BUF_I spi_sync failed.\n");
		return -1;
	}
	//in phase.
	ret = byd_fps_spi_write(spi, REG_IN_PHASE, VAL_IN_PHASE);
	if(ret != 0){
		pr_err("byd REG_IN_PHASE spi_sync failed.\n");
		return -1;
	}
	//sh_i_sel.
	ret = byd_fps_spi_write(spi, REG_SH_I_SEL, VAL_SH_I_SEL);
	if(ret != 0){
		pr_err("byd REG_SH_I_SEL spi_sync failed.\n");
		return -1;
	}
	//TX .
	ret = byd_fps_spi_write(spi, REG_TX_CFG, VAL_TX_CFG);
	if(ret != 0){
		pr_err("byd REG_TX_CFG spi_sync failed.\n");
		return -1;
	}
	return 0;
}
#endif

/* *******************************************************************************
 * Function    :  byd_fps_rd_fg_os_cfg
 * Description :  set configure value to chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_rd_fg_os_cfg(struct spi_device *spi, unsigned char fg_sub)
{
	unsigned char  v_cfg;
	int ret;
	//Write type of CFG,
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_spi_write(spi, REG_FINGER_OS_DATA_SEL, fg_sub);
		if(ret != 0){
			pr_err("byd REG_FINGER_OS_DATA_SEL spi_sync failed.\n");
			return -1;
		}
		//Read CFG.
		ret = byd_fps_spi_read(spi, REG_FINGER_OS_DATA_H);
		if(ret < 0){
			pr_err("byd REG_FINGER_OS_DATA_H spi_sync failed.\n");
			return -1;
		}
		v_cfg =(unsigned char)ret;
		DBG_TIME("%s:REG_FINGER_OS_DATA_H:0x%x,fg_sub:%d\n",__func__,v_cfg,fg_sub);
		ret = byd_fps_spi_read(spi, REG_FINGER_OS_DATA_L);
		if(ret < 0){
			pr_err("byd REG_FINGER_OS_DATA_L spi_sync failed.\n");
			return -1;
		}
		DBG_TIME("%s:REG_FINGER_OS_DATA_L:0x%x\n",__func__,(unsigned char)ret);
		if(byd_fps_chip_flag == 0x11){
			if (fg_sub < OS_AREA_BLOCK11)
				return ((v_cfg << 8) | ret) * 4;
			else
				return (v_cfg << 8) | ret;
		}else if(byd_fps_chip_flag == 0x12){
			if (fg_sub < OS_AREA_BLOCK12)
				return ((v_cfg << 8) | ret) * 4;
			else
				return (v_cfg << 8) | ret;
		
		}
	}else if(byd_fps_chip_flag == 0x22){
		ret = byd_fps_spi_write(spi, REG_FINGER_OS_DATA_SEL22, fg_sub);
		if(ret != 0){
			pr_err("byd REG_FINGER_OS_DATA_SEL spi_sync failed.\n");
			return -1;
		}
		//Read CFG.
		ret = byd_fps_spi_read(spi, REG_FINGER_OS_DATA_H22);
		if(ret < 0){
			pr_err("byd REG_FINGER_OS_DATA_H spi_sync failed.\n");
			return -1;
		}
		v_cfg =(unsigned char)ret;
		DBG_TIME("%s:REG_FINGER_OS_DATA_H:0x%x,fg_sub:%d\n",__func__,v_cfg,fg_sub);
		ret = byd_fps_spi_read(spi, REG_FINGER_OS_DATA_L22);
		if(ret < 0){
			pr_err("byd REG_FINGER_OS_DATA_L spi_sync failed.\n");
			return -1;
		}
		DBG_TIME("%s:REG_FINGER_OS_DATA_L:0x%x\n",__func__,(unsigned char)ret);

		if (fg_sub < OS_AREA_BLOCK22)
			return ((v_cfg << 8) | ret) * 4;
		else
			return (v_cfg << 8) | ret;
		
	}
	return 0;
}


#if 0
/* *******************************************************************************
 * Function    :  byd_fps_rd_cfg_flag
 * Description :  set configure value to chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_rd_cfg_flag(struct spi_device *spi)
{
	int ret;
	
	//Read CFG.
	ret = byd_fps_spi_read(spi, REG_CFG_FLAG);
	if(ret < 0){
		pr_err("byd REG_CFG_FLAG spi_sync failed.\n");
		return -1;
	}
	
	return ret;
}
#endif

#ifdef CONFIG_FPS11
#define VAL_CSD_ENABLE                      0x8c
#define VAL_CSD_DISABLE                     0x9A
#define VAL_CPU_TIMER_CFG					1
#define VAL_PD_INT_FINGER                   0

#if 0
/* *******************************************************************************
 * Function    :  byd_fps_csd_cfg
 * Description :  set configure value to chip, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
   ******************************************************************************/
static int byd_fps_csd_cfg(struct spi_device *spi)
{
	int ret;
	//csd is enable or disable
	ret = byd_fps_spi_write(spi, REG_KEY_EN, VAL_CSD_DISABLE);
	if(ret != 0){
		pr_err("byd REG_ADC_I_SEL spi_sync failed.\n");
		return -1;
	}
	//cpu_timer_cfg
	ret = byd_fps_spi_write(spi, REG_CPU_TIMER_CFG, VAL_CPU_TIMER_CFG);
	if(ret != 0){
		pr_err("byd REG_SEN_BUF_I spi_sync failed.\n");
		return -1;
	}
	//pd_int_finger
	ret = byd_fps_spi_write(spi, REG_PD_INT_FINGER, VAL_PD_INT_FINGER);
	if(ret != 0){
		pr_err("byd REG_IN_PHASE spi_sync failed.\n");
		return -1;
	}
}
#endif 
#endif 

//读otp的手指子区域变化量
//#define OTP_FPD_TH   0x1F0
//#define OTP_FPD_TH_LTENGTH  6
int byd_read_otp_fpd_var(void)
{
	int ret,i;
	unsigned short fpd_th_buf[3], otp_fpd_th = 0xffff;
	
	ret = byd_fps_otp_read(OTP_FPD_TH,OTP_FPD_TH_LTENGTH,(unsigned char*)fpd_th_buf,0);
	DBG_TIME("%s:byd OTP fpd_th_buf byte:",__func__);
	for(i=0;i<OTP_FPD_TH_LTENGTH/2;i++) {
		DBG_TIME("%s:byd[%d]=0x%x,", __func__,i,fpd_th_buf[i]);
		if (fpd_th_buf[i] < otp_fpd_th)
			otp_fpd_th = fpd_th_buf[i];
		DBG_TIME("%s:byd otp_fpd_th =%d\n ",__func__,otp_fpd_th);	
	}
	DBG_TIME("\n");
	
	return otp_fpd_th;
}

/* *******************************************************************************
 * fae para
   ******************************************************************************/
//固定参数
const unsigned int fg_sh_init_num = 32;
const unsigned int fg_row_num=24;
const unsigned int fg_init_time = 400;	//400us

//输入的变量
//unsigned int integral_num = VAL_FINGER_INTE_SEL;
////float frequency_tx = ;	//select list, tx_period = 1000/frequency_tx;
////unsigned int fg_reset_ti1 = ;	//0xfff0, fg_rest_time = (fg_reset_ti1+1)*250 us.
//unsigned int no_finger_time = byd_fps_fae_detect_para[4];	//10*1000 ms.

unsigned int byd_fps_tx_period(void)
{
	//float tx_period;
	unsigned int tx_period, tx_period_tmp;
	unsigned char base_val;
	unsigned char adc_init_en=1;
	unsigned char tx_sel=0x03;	//tx_clk 
	tx_sel = byd_fps_fae_image[0];
	
	adc_init_en = VAL_SENSOR_MODE & 0x01;

	base_val = 60+adc_init_en*12;
	
	if(tx_sel < 5) {
		tx_period_tmp = (base_val + tx_sel*4)*3125;	//Extended 100*1000 times.
	} else {
		tx_period_tmp = (base_val+8 + tx_sel*2)*3125;	//Extended 100*1000 times.
	}
	tx_period = tx_period_tmp/100;	//Extended 1000 times.
	return tx_period;
}
unsigned int byd_fps_tx_period22(void)
{
	//float tx_period;
	unsigned int tx_period, tx_period_tmp;
	unsigned char base_val;
	unsigned char adc_init_en=1;
	unsigned char adc_inte_num=1;
	unsigned char tx_sel=0x03;	//tx_clk 
	tx_sel = byd_fps_fae_image[0];
	
	adc_init_en = VAL_SENSOR_MODE & 0x01;
	adc_inte_num = (VAL_SENSOR_MODE & 0x04)/0x04;
	base_val = (8+3*(adc_init_en+1))*4/(1+adc_inte_num);
	
	FP_DBG("byd base_val=%d,adc_init_en=%d,adc_inte_num=%d\n",base_val,adc_init_en,adc_inte_num);
	if(tx_sel < 5) {
		tx_period_tmp = (base_val + tx_sel*4)*3125;	//Extended 100*1000 times.
	} else {
		tx_period_tmp = (base_val+8 + tx_sel*2)*3125;	//Extended 100*1000 times.
	}
	tx_period = tx_period_tmp/10;	//Extended 10,000 times.
	FP_DBG("byd tx_period=%d,tx_period_tmp=%d\n",tx_period,tx_period_tmp);
	return tx_period;
}
unsigned long byd_fps_long_div(unsigned long div1, unsigned long div2)
{
	if(div2 < 0) {
		return 0;
	}
	if((div1 < div2)||(div1 == 0)) {
		return 0;
	} else {
		unsigned long count=0;
		while(div1>div2) {
			div1 = div1 - div2;
			count++;
		}
		return count;
	}
	return 0;
}

unsigned short byd_fps_no_fng_num(unsigned int no_finger_time)
{
	unsigned int tx_period, scan_1fg_time, time_all_1fg, time_all_1fg_tmp;
	unsigned int fg_rest_time;
	unsigned long no_finger_num, no_finger_time_temp;
	unsigned int integral_num = (VAL_FINGER_INTE_SEL+1)*2;

	FP_DBG("%s:integral_num=%d\n", __func__, integral_num);
	FP_DBG("%s:fg_sh_init_num=%d, fg_row_num=%d, fg_init_time=%d us,no_finger_time=%d ms\n", __func__, fg_sh_init_num, fg_row_num, fg_init_time, no_finger_time);
	
	fg_rest_time = byd_fps_fae_detect_para[3];	//calculate with us.
	FP_DBG("%s:fg_rest_time=%d us\n", __func__, fg_rest_time);
	
	//1. tx_period = 1000/frequency_tx;
	tx_period = byd_fps_tx_period();
	FP_DBG("%s:tx_period=%d ns\n", __func__, tx_period);
	
	//2. scan_1fg_time = (1 + fg_sh_init_num * tx_period + ( (tx_period*integral_num + 0.5)*fg_row_num + tx_period ));
	scan_1fg_time = (1*1000 + fg_sh_init_num * tx_period + ( (tx_period*integral_num + 1000/2)*fg_row_num + tx_period ) );
	FP_DBG("%s:scan_1fg_time=%d ns\n", __func__, scan_1fg_time);
	
	//3. fg_rest_time = (fg_reset_ti1+1)*250;
	
	//4. time_all_1fg = fg_init_time + scan_1fg_time + fg_rest_time;
	time_all_1fg_tmp = fg_init_time*1000 + scan_1fg_time + fg_rest_time*1000;
	time_all_1fg = time_all_1fg_tmp/1000;
	FP_DBG("%s:time_all_1fg_tmp=%d ns\n", __func__, time_all_1fg_tmp);
	FP_DBG("%s:time_all_1fg=%d us\n", __func__, time_all_1fg);
	
	//5.1 no_finger_num = (no_finger_time*1000)/time_all_1fg;
	no_finger_time_temp = (no_finger_time*1000);
	FP_DBG("%s:no_finger_time_temp(no_finger_time)=%d us\n", __func__, no_finger_time_temp);
	
	//5.2 no_finger_num = (unsigned long)((no_finger_time*1000)*100*1000)/(unsigned long)time_all_1fg;
	no_finger_num = no_finger_time_temp/time_all_1fg;
	//no_finger_num = byd_fps_long_div(no_finger_num_temp, time_all_1fg);

	FP_DBG("%s:no_finger_num=%d\n", __func__, no_finger_num);
	
	return no_finger_num;
}
unsigned short byd_fps_no_fng_num22(unsigned int no_finger_time)
{
	unsigned int tx_period, scan_1fg_time, time_all_1fg, time_all_1fg_tmp;
	unsigned int fg_rest_time;
	unsigned long no_finger_num, no_finger_time_temp;
	unsigned int integral_num = 2;	//(VAL_FINGER_INTE_SEL+1)*2;

	FP_DBG("%s:integral_num=%d\n", __func__, integral_num);
	FP_DBG("%s:fg_sh_init_num=%d, fg_row_num=%d, fg_init_time=%d us,no_finger_time=%d ms\n", __func__, fg_sh_init_num, fg_row_num, fg_init_time, no_finger_time);
	
	fg_rest_time = byd_fps_fae_detect_para22[3];	//calculate with us.
	FP_DBG("%s:fg_rest_time=%d us\n", __func__, fg_rest_time);
	//1
	//tx_period = 1000/frequency_tx;
	tx_period = byd_fps_tx_period22();
	FP_DBG("%s:tx_period=%d ns\n", __func__, tx_period);
	//2
	//scan_1fg_time = (1 + fg_sh_init_num * tx_period + ( (tx_period*integral_num + 0.5)*fg_row_num + tx_period ));
	scan_1fg_time = (1*10000 + fg_sh_init_num * tx_period + ( (tx_period*integral_num + 10000/2)*fg_row_num + tx_period ) );
	FP_DBG("%s:scan_1fg_time=%d ns\n", __func__, scan_1fg_time);
	//3
	//fg_rest_time = (fg_reset_ti1+1)*250;
	//4
	//time_all_1fg = fg_init_time + scan_1fg_time + fg_rest_time;
	time_all_1fg_tmp = fg_init_time*10000 + scan_1fg_time + fg_rest_time*10000;
	//5
	//no_finger_num = (no_finger_time*1000)/time_all_1fg;
	time_all_1fg = time_all_1fg_tmp/10000;
	FP_DBG("%s:time_all_1fg_tmp=%d ns\n", __func__, time_all_1fg_tmp);
	FP_DBG("%s:time_all_1fg=%d us\n", __func__, time_all_1fg);
	
	//5.1 no_finger_num = (no_finger_time*1000)/time_all_1fg;
	
	no_finger_time_temp = (no_finger_time*1000);
	FP_DBG("%s:no_finger_time_temp(no_finger_time)=%d us\n", __func__, (int)no_finger_time_temp);
	
	
	//no_finger_num = (unsigned long)((no_finger_time*1000)*100*1000)/(unsigned long)time_all_1fg;
	no_finger_num = (no_finger_time_temp/time_all_1fg/32)+1;
	//no_finger_num = byd_fps_long_div(no_finger_num_temp, time_all_1fg);

	FP_DBG("%s:no_finger_num=%d\n", __func__, (int)no_finger_num);
	return no_finger_num;
}
/* *******************************************************************************
 * Function    :  byd_fps_init_fae_para
 * Description :  set fae para to the big buffer.
 * In          :  void
 * Return      :  0 success
   ******************************************************************************/
int byd_fps_init_fae_para(void)
{
	unsigned short temp;

	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		//fpd_th_on 按下域值
		temp = (65535-byd_fps_fae_detect_para[0]);
		FP_DBG("%s:byd_fps_fae_detect_para[0]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para[0],temp,temp);
		byd_fps_fae_detect[0] = (unsigned char)(temp>>8);
		byd_fps_fae_detect[1] = (unsigned char)temp;
		FP_DBG("%s:fpd_th_on=0x%x,fae_detect[0,1]=0x%x%x\n", __func__, temp, byd_fps_fae_detect[0], byd_fps_fae_detect[1]);
	
		//fpd_th_off 抬起域值
		temp = (65535-byd_fps_fae_detect_para[1]);
		FP_DBG("%s:byd_fps_fae_detect_para[1]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para[1],temp,temp);
		byd_fps_fae_detect[2] = (unsigned char)(temp>>8);
		byd_fps_fae_detect[3] = (unsigned char)temp;
		FP_DBG("%s:fpd_th_off=0x%x,fae_detect[2,3]=0x%x%x\n", __func__, temp, byd_fps_fae_detect[2], byd_fps_fae_detect[3]);
	
	
	
		//NO_FINGER_NUM 无手指超时,  建议10S.
	
		//FG_REST_TI2 手指慢检测时间间隔(ms), 步进为0.25ms, t=(FG_REST_TI2 + 1)*250us
		temp = (byd_fps_fae_detect_para[2]/250 - 1);
		FP_DBG("%s:byd_fps_fae_detect_para[2]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para[2],temp,temp);
		byd_fps_fae_detect[6] = (unsigned char)(temp>>8);
		byd_fps_fae_detect[7] = (unsigned char)temp;
		DBG_TIME("%s:FG_REST_TI2=0x%x,fae_detect[6,7]=0x%x%x\n", __func__, temp, byd_fps_fae_detect[6], byd_fps_fae_detect[7]);
	
		//FG_REST_TI1 手指快检测时间间隔(ms), 步进为0.25ms, t=(FG_REST_TI1 + 1)*250us
		temp = (byd_fps_fae_detect_para[3]/250 - 1);
		FP_DBG("%s:byd_fps_fae_detect_para[3]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para[3],temp,temp);
		byd_fps_fae_detect[4] = (unsigned char)(temp>>8);
		byd_fps_fae_detect[5] = (unsigned char)temp;
		DBG_TIME("%s:FG_REST_TI1=0x%x,fae_detect[4,5]=0x%x%x\n", __func__, temp, byd_fps_fae_detect[4], byd_fps_fae_detect[5]);
		
		temp = byd_fps_no_fng_num(byd_fps_fae_detect_para[4]);
		
		FP_DBG("%s:byd_fps_fae_detect_para[4]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para[4],temp,temp);
		byd_fps_fae_detect[8] = (unsigned char)(temp>>8);
		byd_fps_fae_detect[9] = (unsigned char)temp;
		FP_DBG("%s:NO_FINGER_NUM=0x%x,fae_detect[8,9]=0x%x%x\n", __func__, temp, byd_fps_fae_detect[8], byd_fps_fae_detect[9]);
	}else if(byd_fps_chip_flag == 0x22){
		byd_fps_chip_idle(this_byd_fps->spi);
		otp_fpd_th = byd_read_otp_fpd_var();//从otp获取三块子区域变化量的最小值,变化量=失调值-胶块值（芯片那边给的是指纹的变化量：12位，所以要算阈值就得放大16倍）
		DBG_TIME("byd otp_fpd_th=%d\n",otp_fpd_th);
		
		if (otp_fpd_th == 0xffff) {//如果为0xffff，说明没写在otp，按老方法配阈值
			otp_fpd_th = BYD_FPS_DEFAULT_FPD_THRE/16;//默认值按照6000计算;6000/16
			//fpd_th_on 按下域值
			temp = (65535-byd_fps_fae_detect_para22[0]*otp_fpd_th*16/100);//将配置的按下阈值
			DBG_TIME("%s:byd_fps_fae_detect_para22[0]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para22[0],temp,temp);
			byd_fps_fae_detect22[0] = (unsigned char)(temp>>8);
			byd_fps_fae_detect22[1] = (unsigned char)temp;
			DBG_TIME("%s:fpd_th_on=0x%x,fae_detect[0,1]=0x%x%x\n", __func__, temp, byd_fps_fae_detect22[0], byd_fps_fae_detect22[1]);
			//fpd_th_off 抬起域值
			temp = (65535-byd_fps_fae_detect_para22[1]*otp_fpd_th*16/100);//将配置的抬起阈值
			FP_DBG("%s:byd_fps_fae_detect_para22[1]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para22[1],temp,temp);
			byd_fps_fae_detect22[2] = (unsigned char)(temp>>8);
			byd_fps_fae_detect22[3] = (unsigned char)temp;
			DBG_TIME("%s:fpd_th_off=0x%x,fae_detect[2,3]=0x%x%x\n", __func__, temp, byd_fps_fae_detect22[2], byd_fps_fae_detect22[3]);
			//OS_TH_H 失调值阈值
			byd_fps_fae_detect22[10] = VAL_OS_TH_H;//没写otp之前，失调阈值按照byd_fps22_libbf663x.h配的
			byd_fps_fae_detect22[11] = VAL_OS_TH_L;
			DBG_TIME("%s:fpd_th_off=0x%x,fae_detect[2,3]=0x%x%x\n", __func__, temp, byd_fps_fae_detect22[10], byd_fps_fae_detect22[11]);
		} else {
			//fpd_th_on 按下域值
			temp = (65535-(byd_fps_fae_detect_para22[0]*otp_fpd_th*16)/100);
			DBG_TIME("%s:byd_fps_fae_detect_para22[5]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para22[5],temp,temp);
			byd_fps_fae_detect22[0] = (unsigned char)(temp>>8);
			byd_fps_fae_detect22[1] = (unsigned char)temp;
			DBG_TIME("%s:fpd_th_on=0x%x,fae_detect[0,1]=0x%x%x\n", __func__, temp, byd_fps_fae_detect22[0], byd_fps_fae_detect22[1]);
			//fpd_th_off 抬起域值
			temp = (65535-(byd_fps_fae_detect_para22[1]*otp_fpd_th*16)/100);
			DBG_TIME("%s:byd_fps_fae_detect_para[6]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para22[6],temp,temp);
			byd_fps_fae_detect22[2] = (unsigned char)(temp>>8);
			byd_fps_fae_detect22[3] = (unsigned char)temp;
			DBG_TIME("%s:fpd_th_off22=0x%x,fae_detect22[2,3]=0x%x%x\n", __func__, temp, byd_fps_fae_detect22[2], byd_fps_fae_detect22[3]);
			//OS_TH_H 失调值阈值
			temp = (65535-(byd_fps_fae_detect_para22[5]*otp_fpd_th*16)/100);
			DBG_TIME("%s:byd_fps_fae_detect_para22[6]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para22[7],temp,temp);
			byd_fps_fae_detect22[10] = (unsigned char)(temp>>8);
			byd_fps_fae_detect22[11] = (unsigned char)temp;
			DBG_TIME("%s:fpd_os_th_off22=0x%x,fae_detect22[2,3]=0x%x%x\n", __func__, temp, byd_fps_fae_detect22[10], byd_fps_fae_detect22[11]);
		}
		//FG_REST_TI2 手指慢检测时间间隔(ms), 步进为8ms, t=(FG_REST_TI2 + 1)*8000us
		temp = (byd_fps_fae_detect_para22[2]/8000 - 1);
		DBG_TIME("%s:byd_fps_fae_detect_para22[2]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para22[2],temp,temp);
		byd_fps_fae_detect22[6] = (unsigned char)(temp>>8);
		byd_fps_fae_detect22[7] = (unsigned char)temp;
		DBG_TIME("%s:FG_REST_TI2=0x%x,fae_detect[6,7]=0x%x%x\n", __func__, temp, byd_fps_fae_detect22[6], byd_fps_fae_detect22[7]);
	
		//FG_REST_TI1 手指快检测时间间隔(ms), 步进为2ms, t=(FG_REST_TI1 + 1)*2000us
		temp = (byd_fps_fae_detect_para22[3]/2000 - 1);
		DBG_TIME("%s:byd_fps_fae_detect_para22[3]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para22[3],temp,temp);
		byd_fps_fae_detect22[4] = (unsigned char)(temp>>8);
		byd_fps_fae_detect22[5] = (unsigned char)temp;
		DBG_TIME("%s:FG_REST_TI1=0x%x,fae_detect[4,5]=0x%x%x\n", __func__, temp, byd_fps_fae_detect22[4], byd_fps_fae_detect22[5]);
		
		temp = byd_fps_no_fng_num22(byd_fps_fae_detect_para22[4]);
		
		DBG_TIME("%s:byd_fps_fae_detect_para22[4]=%d,temp=%d(0x%x)\n", __func__, byd_fps_fae_detect_para22[4],temp,temp);
		byd_fps_fae_detect22[8] = (unsigned char)(temp>>8);
		byd_fps_fae_detect22[9] = (unsigned char)temp;
		DBG_TIME("%s:NO_FINGER_NUM22=0x%x,fae_detect[8,9]=0x%x%x\n", __func__, temp, byd_fps_fae_detect22[8], byd_fps_fae_detect22[9]);
	}
	return 0;
}

/*******************************************************************************
 * Function    :  byd_fps_chip_state
 * Description :  get chip mode, by SPI command.
 * In          :  *spi
 * Return      :  0 -- chip mode, -1 -- failed
 *                0: chip idle; 1: chip sub scan; 2: chip finger detect;
 *                3: chip sleep; 4: chip fingerprint scan.
 *******************************************************************************/
int byd_fps_chip_state(struct spi_device *spi)
{
	int ret;
	ret = byd_fps_spi_read(spi, REG_CHIP_MODE);
	if(ret < 0){
		pr_err("byd REG_ CHIP_MODE spi_sync failed.\n");
		return -1;
	}
	FP_DBG("%s: Read REG_ CHIP_MODE =%d \n", __func__, ret);
	return ret;
}

/*******************************************************************************
 * Function    :  byd_fps_chip_scan_cmd_state
 * Description :  get chip mode, by SPI command.
 * In          :  *spi
 * Return      :  0 -- chip mode, -1 -- failed
 *                0: chip idle; 1: chip sub scan; 2: chip finger detect;
 *                3: chip sleep; 4: chip fingerprint scan.
 *******************************************************************************/
int byd_fps_chip_scan_cmd_state(struct spi_device *spi)
{
	int ret;
	ret = byd_fps_spi_read(spi, REG_SCAN_CMD);
	if(ret < 0){
		pr_err("byd REG_ CHIP_MODE spi_sync failed.\n");
		return -1;
	}
	FP_DBG("%s: Read REG_ CHIP_MODE =%d \n", __func__, ret);
	return ret;
}


/*******************************************************************************
 * Function    :  byd_fps_chip_reset
 * Description :  reset chip by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
 *******************************************************************************/
static int byd_fps_chip_reset(struct spi_device *spi)
{
	#if 1
	int i;
	int ret;
	ret = byd_fps_spi_write(spi, REG_SOFT_RST, 0x55);
	if(ret != 0){
		pr_err("byd REG_SOFT_RST spi_sync failed.\n");
		return -1;
	}
	byd_fps_mdelay(3);
	for(i=5;i>0;i--) {
		ret = byd_fps_spi_read(spi, REG_SOFT_RST);
		if(ret < 0){
			pr_err("byd REG_SOFT_RST spi_sync failed.\n");
			return -1;
		}
		if((unsigned char)ret == 0xAA) {
			break;
		}
		byd_fps_mdelay(2);
	}
	DBG_TIME("%s: 2nd, REG _SOFT_RST=%x\n", __func__, ret);
	#endif
	return 0;
}

/*******************************************************************************
 * Function    :  byd_fps_chip_sleep
 * Description :  set chip to sleep mode by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
 *******************************************************************************/
int byd_fps_chip_sleep(struct spi_device *spi)
{
	int ret;
	ret = byd_fps_spi_write(spi, REG_CHIP_MODE, CHIP_MODE_SLEEP);
	if(ret != 0){
		pr_err("byd REG_ CHIP_MODE spi_sync failed.\n");
		return -1;
	}

	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_chip_idle
 * Description :  set chip to idle mode, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
 *******************************************************************************/
int byd_fps_chip_idle(struct spi_device *spi)
{
	int ret;

	ret = byd_fps_spi_write(spi, REG_CHIP_MODE, CHIP_MODE_IDLE);
	if(ret != 0){
		pr_err("byd REG_ CHIP_MODE spi_sync failed.\n");
		return -1;
	}
	byd_fps_mdelay(1);	// 0.5ms;
	
	return 0;
}


/* *******************************************************************************
 * Function    :  byd_fps_chip_fg_detect
 * Description :  set chip to finger detect mode, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
 *******************************************************************************/
int byd_fps_chip_fg_detect(struct spi_device *spi)
{
	int ret;
		
	FP_DBG("%s: write REG_ CHIP_MODE=0x01 \n",__func__);
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_spi_write(spi, REG_CHIP_MODE, CHIP_MODE_FG_DET);
		if(ret != 0){
			pr_err("byd REG_CHIP_MODE spi_sync failed.\n");
			return -1;
		}
	}else if(byd_fps_chip_flag == 0x22){
		ret = byd_fps_spi_write(spi, REG_CHIP_MODE, CHIP_MODE_FG_DET22);
		if(ret != 0){
			pr_err("byd REG_CHIP_MODE spi_sync failed.\n");
			return -1;
		}
	}
	FP_DBG("%s: write REG_CHIP_MODE End \n",__func__);
	return 0;
}

int byd_fps_chip_detect(struct spi_device *spi)
{
	byd_fps_chip_fg_detect(spi);
	return 0;
}

/*******************************************************************************
 * Function    :  byd_fps_chip_fp_scan
 * Description :  set chip to fingerprint scan mode, by SPI command.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
 *******************************************************************************/
//scan area & times.
#define SCAN_AREA0_SNGL		0x10
#define SCAN_AREA0_CYCL		0x20
#define SCAN_AREA1_SNGL		0x30
#define SCAN_AREA1_CYCL		0x50
#define SCAN_OS_DATA		0x60
//Scan order.
#define SCAN_POSIT_ORDER	0x01//正序
#define SCAN_INFIX_ORDER	0x02//中序
#define SCAN_INVERT_ORDER	0x03//倒序
//scan_cmd = SCAN_AREA0_SNGL|SCAN_POSIT_ORDER

#define SCAN_OS_DATA22		0x61
#define SCAN_FP_DATA22		0x11
int byd_fps_chip_fp_scan(struct spi_device *spi, unsigned char scan_cmd)
{
	int ret;
	ret = byd_fps_chip_idle(spi);
	
	byd_fps_chip_intr_flag_clr(spi, 0);
	byd_fps_set_finger_state(spi, 0);
	
	DBG_TIME("%s: byd_fps_chip_flag=0x%x,scan_cmd=0x%x \n",__func__, byd_fps_chip_flag, scan_cmd);
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_spi_write(spi, REG_SCAN_CMD, scan_cmd);
		if(ret != 0){
			pr_err("byd REG_CHIP_MODE spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_CHIP_MODE, CHIP_MODE_FG_DET);
		if(ret != 0){
			pr_err("byd REG_CHIP_MODE spi_sync failed.\n");
			return -1;
		}
	}else if(byd_fps_chip_flag == 0x22){
		if(scan_cmd == SCAN_OS_DATA22){
			ret = byd_fps_spi_write(spi, REG_CHIP_MODE, CHIP_MODE_OS_SCAN22);
			if(ret != 0){
				pr_err("byd REG_CHIP_MODE spi_sync failed.\n");
				return -1;
			}
		}else if(scan_cmd == SCAN_FP_DATA22){
			ret = byd_fps_spi_write(spi, REG_CHIP_MODE, CHIP_MODE_FP_SCAN22);
			if(ret != 0){
				pr_err("byd REG_CHIP_MODE spi_sync failed.\n");
				return -1;
			}
		}
	}
	FP_DBG("%s: write REG_CHIP_MODE End \n",__func__);
	return 0;
}

/*******************************************************************************
 * Function    :  byd_fps_chip_intr_state
 * Description :  get chip mode, by SPI command.
 * In          :  *spi
 * Return      :  0 -- chip mode, -1 -- failed
 *                0: chip idle; 1: chip sub scan; 2: chip finger detect;
 *                3: chip sleep; 4: chip fingerprint scan.
 *******************************************************************************/
int byd_fps_chip_intr_state(struct spi_device *spi)
{
	int ret = 0;
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_spi_read(spi, REG_INT_STATE);
		if(ret < 0){
			pr_err("byd REG_ INT_STATE spi_sync failed.\n");
			return -1;
		}
		FP_DBG("%s: Read REG_ INT_STATE =%d \n", __func__, ret);
	}else if(byd_fps_chip_flag == 0x22){
		ret = byd_fps_spi_read(spi, REG_INT_STATE22);
		if(ret < 0){
			pr_err("byd REG_ INT_STATE spi_sync failed.\n");
			return -1;
		}
		FP_DBG("%s: Read REG_ INT_STATE22 =%d \n", __func__, ret);
		
	}
	return ret;
}

/*******************************************************************************
 * Function    :  byd_fps_chip_intr_flag_clr
 * Description :  clear interrupt flag of chip, by SPI command.
 * In          :  *spi, clear
 * Return      :  0 -- success, -1 -- failed
 *******************************************************************************/
int byd_fps_chip_intr_flag_clr(struct spi_device *spi, unsigned char clear)
{
	int ret;
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_spi_write(spi, REG_INT_CFG, 0x80|VAL_INT_CFG);
		if(ret != 0){
			pr_err("byd REG_ INT_STATE spi_sync failed.\n");
			return -1;
		}
	
		ret = byd_fps_spi_write(spi, REG_INT_STATE, clear&0x0f);
		if(ret != 0){
			pr_err("byd REG_ INT_STATE spi_sync failed.\n");
			return -1;
		}
		FP_DBG("%s: Write REG_ INT_STATE =%d \n", __func__, ret);
	}else if(byd_fps_chip_flag == 0x22){
	
		ret = byd_fps_spi_write(spi, REG_INT_STATE22, clear&0x1f);
		if(ret != 0){
			pr_err("byd REG_ INT_STATE22 spi_sync failed.\n");
			return -1;
		}
		FP_DBG("%s: Write REG_ INT_STATE22 =%d \n", __func__, ret);
		
		
	}
	return 0;
}

/* *******************************************************************************
 * Function    :  byd_fps_chip_pd_cfg
 * Description :  set pd_ana_a & pd_ana_b.
 * In          :  *spi
 * Return      :  0 -- success, -1 -- failed
 *******************************************************************************/
int byd_fps_chip_pd_cfg(struct spi_device *spi, unsigned char pd_flag)
{
	int ret;

	if(pd_flag == 0) {
		ret = byd_fps_spi_write(spi, REG_PD_ANA_A, 0x3F);
		if(ret != 0){
			pr_err("byd REG_ PD_ANA_A spi_sync failed.\n");
			return -1;
		}
		ret = byd_fps_spi_write(spi, REG_PD_ANA_B, 0x00);
		if(ret != 0){
			pr_err("byd REG_ PD_ANA_B spi_sync failed.\n");
			return -1;
		}
	} else {
		if(pd_flag == 1) {
			ret = byd_fps_spi_write(spi, REG_PD_ANA_A, VAL_PD_ANA_A);
			if(ret != 0){
				pr_err("byd REG_ PD_ANA_A spi_sync failed.\n");
				return -1;
			}
			ret = byd_fps_spi_write(spi, REG_PD_ANA_B, VAL_PD_ANA_B);
			if(ret != 0){
				pr_err("byd REG_ PD_ANA_B spi_sync failed.\n");
				return -1;
			}
		}
	}
	return 0;
}
// *******************************************************************************
// * Function    :  byd_fps_decode_cfg
// * Description :  config jiami mode, by SPI command.
// * In          :  *spi
// * Return      :  0 -- success, -1 -- failed
// *******************************************************************************
int byd_fps_decode_cfg(struct spi_device *spi, unsigned char dec_data)
{
	int ret;
	ret = byd_fps_spi_write(spi, REG_FUNCTION_SEL, dec_data);
	if(ret != 0){
		pr_err("byd REG_ PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	return 0;
}

int byd_fps_dig_inte_set(struct byd_fps_data *byd_fps, unsigned char en_flag)
{
	int ret;
	if(en_flag == 1) {
		FP_DBG("%s:Set SPI frq 500khz\n", __func__);
		byd_fps_spi_speed(byd_fps->spi, SPI_OS_SPEED);		//SPI clock SET 500 SPI_OS_SPEED
		FP_DBG("%s:Set DIG_INTE 96\n", __func__);
		ret = byd_fps_spi_write(byd_fps->spi, REG_DIG_INTE_SEL22, 0x2F);	//0x2F==47, DIG_INTE (47+1)*2=96次积分.
		if(ret != 0){
			pr_err("byd REG_DIG_INTE_SEL spi_sync failed.\n");
			return -1;
		}
	}
	if(en_flag == 0) {
		FP_DBG("%s:Set SPI frq 1000khz\n", __func__);
		byd_fps_spi_speed(byd_fps->spi,SPI_FP_SPEED);		//SPI clock SET 2400  SPI_FP_SPEED
		FP_DBG("%s:Set DIG_INTE 64\n", __func__);
		ret = byd_fps_spi_write(byd_fps->spi, REG_DIG_INTE_SEL22, byd_fps_fae_image[1]);	//0x1F==31, DIG_INTE (31+1)*2=64次积分.
		if(ret != 0){
			pr_err("byd REG_DIG_INTE_SEL spi_sync failed.\n");
			return -1;
		}
	}
	return 0;
}
// *******************************************************************************
// * Function    :  byd_fps_finger_subarea_detect_cfg
// * Description :  config detect subarea, by SPI command.
// * In          :  *spi
// * Return      :  0 -- success, -1 -- failed
// *******************************************************************************
static int byd_fps_finger_subarea_detect_cfg( struct spi_device *spi, unsigned char flag )
{
	int ret =0;
	
	if (flag == 1) {
		/*在指纹扫描模式时将手指触发个数改为4，保证至少采到半个手指以上的图像2017.02.24 cgh*/
		if(byd_fps_chip_flag == 0x12)
			ret = byd_fps_spi_write(spi, REG_INT_MODE_SET, VAL_INT_MODE_SET_FP12);
		if(byd_fps_chip_flag == 0x11)
			ret = byd_fps_spi_write(spi, REG_INT_MODE_SET, VAL_INT_MODE_SET_FP11);
	} else if ( flag== 0) {
		/*在采集完图像之后重新将手指触发个数调为2，以满足按键等的灵敏度需求*/
		if(byd_fps_chip_flag == 0x11)
			ret = byd_fps_spi_write(spi, REG_INT_MODE_SET, VAL_INT_MODE_SET11);
		if(byd_fps_chip_flag == 0x12)
			ret = byd_fps_spi_write(spi, REG_INT_MODE_SET, VAL_INT_MODE_SET12);
	}
	if(ret != 0){
		pr_err("byd REG_INT_MODE_SET spi_sync failed.\n");
		return -1;
	}
	
	return 0;
}

///////////////////////////////////////////////////////////////////
#ifdef BYD_FPS_GEST
int byd_fps_gest_cfg(struct spi_device *spi)
{
	int ret;
	byd_fps_mutex_lock(1,1);
	ret = byd_fps_spi_write(spi, REG_FG_ROW_START_A, VAL_FG_ROW_START_A_GEST);
	if(ret != 0){
		pr_err("byd REG_ PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FG_ROW_START_B, VAL_FG_ROW_START_B_GEST);
	if(ret != 0){
		pr_err("byd REG_ PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FG_ROW_START_C, VAL_FG_ROW_START_C_GEST);
	if(ret != 0){
		pr_err("byd REG_ PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	byd_fps_mutex_lock(2,1);
	return 0;
}
int byd_fps_gest_cfg_back(struct spi_device *spi)
{
	int ret;
	byd_fps_mutex_lock(1,1);
	ret = byd_fps_spi_write(spi, REG_FG_ROW_START_A, VAL_FG_ROW_START_A);
	if(ret != 0){
		pr_err("byd REG_ PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FG_ROW_START_B, VAL_FG_ROW_START_B12);
	if(ret != 0){
		pr_err("byd REG_ PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FG_ROW_START_C, VAL_FG_ROW_START_C12);
	if(ret != 0){
		pr_err("byd REG_ PD_ANA_B spi_sync failed.\n");
		return -1;
	}
	byd_fps_mutex_lock(2,1);
	return 0;
}
#endif
#ifdef BYD_FPS_FOCUS
//mmx????
unsigned char byd_fps_x_calc(unsigned char a1_state,unsigned char a2_state,unsigned char a3_state,unsigned char b1_state,unsigned char b2_state,unsigned char b3_state,unsigned char c1_state,unsigned char c2_state,unsigned char c3_state)
{
	unsigned char x,state_sum;
	state_sum = c1_state+c2_state+c3_state+b1_state+b2_state+b3_state+a1_state+a2_state+a3_state;
	FP_DBG("%s:state_sum :%d\n",__func__,state_sum);
	if(state_sum != 0){
		x = ((c1_state+c2_state+c3_state)*3+(b1_state+b2_state+b3_state)*2+(a1_state+a2_state+a3_state)*1)*10/(state_sum);
		FP_DBG("%s:x zuobiao :%d\n",__func__,x);
	}else{
		x = 0;
	}
	return x;
}
//mmx上下方向
unsigned char byd_fps_y_calc(unsigned char a1_state,unsigned char a2_state,unsigned char a3_state,unsigned char b1_state,unsigned char b2_state,unsigned char b3_state,unsigned char c1_state,unsigned char c2_state,unsigned char c3_state)
{
	unsigned char y,state_sum;
	state_sum = c1_state+c2_state+c3_state+b1_state+b2_state+b3_state+a1_state+a2_state+a3_state;
	FP_DBG("%s:state_sum :%d\n",__func__,state_sum);
	if(state_sum != 0){
		y = ((a3_state+b3_state+c3_state)*3+(a2_state+b2_state+c2_state)*2+(a1_state+b1_state+c1_state)*1)*10/(state_sum);
		FP_DBG("%s:y zuobiao :%d\n",__func__,y);
	}else{
		y = 0;
	}
	return y;
}

int byd_fps_get_fpd_state(struct spi_device *spi,unsigned char gest_int_num)//,unsigned char *a_state,unsigned char *b_state,unsigned char *c_state,)//unsigned char *fpd_sate)
{
	int ret;
	unsigned char fpd_state_l,fpd_state_h;
	unsigned char a1_state,a2_state,a3_state;
	unsigned char b1_state,b2_state,b3_state;
	unsigned char c1_state,c2_state,c3_state;
	ret = byd_fps_spi_read(spi,REG_FPD_STATE_L);
	if(ret < 0){
		pr_err("byd read REG_FG_REST_TI1_EN spi_sync failed.\n");
		return -1;
	}
	fpd_state_l = (unsigned char)ret;
	FP_DBG("%s:fpd_state_l = 0x%x\n",__func__,(unsigned char)ret);
	ret = byd_fps_spi_read(spi,REG_FPD_STATE_H);
	if(ret < 0){
		pr_err("byd read REG_FG_REST_TI1_EN spi_sync failed.\n");
		return -1;
	}
	fpd_state_h = (unsigned char)ret;
	FP_DBG("%s:fpd_state_h = 0x%x\n",__func__,(unsigned char)ret);
	a1_state = fpd_state_l & 0x01;
	a2_state = (fpd_state_l & 0x02)/2;
	a3_state = (fpd_state_l & 0x04)/4;
	b1_state = (fpd_state_l & 0x08)/8;
	b2_state = (fpd_state_l & 0x10)/16;
	b3_state = (fpd_state_l & 0x20)/32;
	c1_state = fpd_state_h & 0x01;
	c2_state = (fpd_state_h & 0x02)/2;
	c3_state = (fpd_state_h & 0x04)/4;
	FP_DBG("%s:a1_state= %d,a2_state= %d,a3_state= %d,b1_state= %d,b2_state= %d,b3_state= %d,c1_state= %d,c2_state= %d,c3_state= %d,gest_int_num=%d\n",__func__,a1_state,a2_state,a3_state,b1_state,b2_state,b3_state,c1_state,c2_state,c3_state,gest_int_num);
	this_byd_fps->left_right_state[gest_int_num] = byd_fps_x_calc(a1_state,a2_state,a3_state,b1_state,b2_state,b3_state,c1_state,c2_state,c3_state);
	this_byd_fps->up_down_state[gest_int_num] = byd_fps_y_calc(a1_state,a2_state,a3_state,b1_state,b2_state,b3_state,c1_state,c2_state,c3_state);
	return 0;
}
#endif

// *******************************************************************************
// * Function    :  byd_fps_check_reg
// * Description :  read register's value of chip, by SPI command.
// * In          :  *spi
// * Return      :  0 -- success, -1 -- failed
// *******************************************************************************
//static int byd_fps_check_reg(struct spi_device *spi)
int byd_fps_check_reg(struct spi_device *spi)
{
	int i;
	int ret;
	/*#ifdef CONFIG_FPS12
		for(i=1; i<=0x77; i++) {
	#else
		for(i=1; i<=0x7E; i++) {
	#endif
		if(i != REG_FPD_DATA) {
			ret = byd_fps_spi_read(spi, i);
			DBG_TIME("byd [0x%x]=0x%x\n", i, ret);
		} else {
			DBG_TIME("byd [0x%x]=0x00\n", i);
		}
	}*/
	if(byd_fps_chip_flag == 0x12){
		for(i=1; i<=0x77; i++) {
			if(i != REG_FPD_DATA) {
				ret = byd_fps_spi_read(spi, i);
				DBG_TIME("byd [0x%x]=0x%x\n", i, ret);
			} else {
				DBG_TIME("byd [0x%x]=0x00\n", i);
			}
		}	
	}
	if(byd_fps_chip_flag == 0x11){
		for(i=1; i<=0x7E; i++) {
			if(i != REG_FPD_DATA) {
				ret = byd_fps_spi_read(spi, i);
				DBG_TIME("byd [0x%x]=0x%x\n", i, ret);
			} else {
				DBG_TIME("byd [0x%x]=0x00\n", i);
			}
		}
		
	}
	//回读FIR系数
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
	ret = byd_fps_spi_write(spi, REG_FPD_TEST_MODE, 0x00);
	if(ret != 0){
		pr_err("byd REG_FPD_TEST_MODE spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FIR_CMD, 0x66);
	if(ret != 0){
		pr_err("byd REG_FIR_CMD spi_sync failed.\n");
		return -1;
	}
	for(i=0;i<64;i++) {
		ret = byd_fps_spi_read(spi, REG_FIR_DATA);
		if(ret < 0){
			pr_err("byd REG_FIR_DATA_L spi_sync failed.\n");
			return -1;
		}
		DBG_TIME("byd FIR[%d]_L=0x%x\n", i, ret);
		ret = byd_fps_spi_read(spi, REG_FIR_DATA);
		if(ret < 0){
			pr_err("byd REG_FIR_DATA_H spi_sync failed.\n");
			return -1;
		}
		DBG_TIME("byd FIR[%d]_H=0x%x\n", i, ret);
	}
	}
	if(byd_fps_chip_flag == 0x22){
		
		for(i=1; i<=0x6B; i++) 
		{
			if(i != REG_FPD_DATA22) {
				ret = byd_fps_spi_read(spi, i);
				DBG_TIME("byd[0x%x]=0x%x, ", i, ret);
			} else {
				DBG_TIME("byd[0x%x]=0x00, ", i);
			}
			if((i%8) ==0) {
				DBG_TIME("\n");
			}
		}
	}
	return 0;
}

/*******************************************************************************
 * Function    :  byd_fps_read_version
 * Description :  get chip version, by SPI command.
 * In          :  *spi
 * Return      :  0 -- number of chip version, -1 -- failed
 *******************************************************************************/
unsigned int byd_fps_read_version(struct spi_device *spi)
{
	int ret = 0;
	unsigned char adj_info1 = 0;
	unsigned char adj_info2 = 0;
	unsigned int adj_info = 0;

	ret = byd_fps_spi_read(spi, REG_ADJ_INFO_1);
	if (ret < 0){
		pr_err("byd REG_ ADJ_INFO1 spi_sync failed.\n");
		return -1;
	}
	adj_info1 = (unsigned char)ret;
	printk("%s:REG_ADJ_INFO1 = 0x%x\n",__func__,adj_info1);

	ret = byd_fps_spi_read(spi, REG_ADJ_INFO_2);
	if (ret < 0){
		pr_err("byd REG_ ADJ_INFO2 spi_sync failed.\n");
		return -1;
	}
	adj_info2 = (unsigned char)ret;
	printk("%s:REG_ADJ_INFO2 = 0x%x\n",__func__,adj_info2);

	adj_info =  adj_info2 << 8 | adj_info1;
	DBG_TIME("%s:ADJ_INFO = 0x%x\n",__func__,adj_info);
	DBG_TIME("%s:ADJ_INFO2 = 0x%x\n",__func__,adj_info&0xf8c0);
	return adj_info;
}
void byd_fps_read_chip_key_12(struct spi_device *spi)
{
	byd_chip_12 = byd_fps_rd_fg_os_cfg(spi,9);
	DBG_TIME("byd_chip_12 :0x%x\n",byd_chip_12);
}

void byd_fps_read_chip_key_11(struct spi_device *spi)
{
	byd_chip_11 = byd_fps_read_chip(); 
	DBG_TIME("byd_chip_12 :0x%x\n",byd_chip_11);
}
/*******************************************************************************
 * Function    :  byd_fps_chip_check_fng_state
 * Description :  check finger state.
 * In          :  *spi
 * Return      :  0/1 -- finger state, -1 -- failed,
 *******************************************************************************/
 int byd_fps_chip_check_fng_state(struct spi_device *spi)
{
	int ret;
	char byd_fng_stat;
	byd_fng_stat = -10;
	ret = 0;
	ret = byd_fps_spi_read(spi, REG_INT_STATE);
	if(ret < 0){
		pr_err("byd REG_INT_STATE spi_sync failed.\n");
		return -1;
	}
	#ifdef BYD_FPS_TEST_TIME_LOG
	DBG_TIME("%s: byd finger up int_timer=0x%x!\n", __func__, ret);
	#endif

	if(ret&IRQ_INT_TIMER) {
		
		ret = 0;
		ret = byd_fps_spi_read(spi, REG_FINGER_STATE);
		if(ret < 0){
			pr_err("byd REG_FINGER_STATE spi_sync failed.\n");
			return -1;
		}
		byd_fng_stat = (char)ret;
		#ifdef BYD_FPS_TEST_TIME_LOG
		DBG_TIME("%s: byd finger up in delay 3ms!,finger_state=%d\n", __func__, ret);
		#endif

		//清除 int_timer
		ret = byd_fps_spi_write(spi, REG_INT_STATE, 0x00);
		if(ret != 0){
			pr_err("byd REG_ INT_STATE spi_sync failed.\n");
			return -1;
		}
	}
	return byd_fng_stat;
}

/*******************************************************************************
 * Function    :  byd_fps_get_finger_state
 * Description :  check finger state.
 * In          :  *spi
 * Return      :  0/1 -- finger state, -1 -- failed,
 *******************************************************************************/
int byd_fps_set_finger_state(struct spi_device *spi, unsigned char on_flag)
{
	int ret;
	ret = byd_fps_spi_write(spi, REG_FINGER_STATE, on_flag);
	if(ret != 0){
		pr_err("byd REG_ INT_STATE spi_sync failed.\n");
		return -1;
	}
	return ret;
}
/*******************************************************************************
 * Function    :  byd_fps_otp_read
 * Description :  read otp information.
 * In          :  *spi
 * Return      :  0 -- secceed, -1 -- failed,
 *******************************************************************************/
 //OTP COMMAND
#define SFR_OTP_RD_START          0x85
#define SFR_OTP_RD_END            0x8A
  
#define SFR_OTP_POINT_ANY         0xEA
#define SFR_OTP_POINT_INC         0x77
#ifdef BYD_FPS_CFG_SEPERATE
static int byd_fps_otp_read(short int otp_addr, int addr_len, unsigned char * pbuf,unsigned char change_flag)
{
	int ret,i;
	unsigned char otp_addr_l,otp_addr_h,temp = 0;
 
	otp_addr_l = otp_addr&0xFF;
	otp_addr_h = (otp_addr&0xFF00)>>8;
	//1. read start command.
	ret = byd_fps_spi_write(this_byd_fps->spi, REG_SFR_OTP_CMD, SFR_OTP_RD_START);
	if(ret != 0){
		pr_err("byd REG_SFR_OTP_CMD spi_sync failed.ret=%d\n", ret);
		return -1;
	}
	//2. write address to sfr.
	ret = byd_fps_spi_write(this_byd_fps->spi, REG_SFR_OTP_CMD, SFR_OTP_POINT_ANY);
	if(ret != 0){
		pr_err("byd REG_SFR_OTP_CMD spi_sync failed.ret=%d\n", ret);
		return -1;
	}
	ret = byd_fps_spi_write(this_byd_fps->spi, REG_SFR_OTP_DATA, otp_addr_l);
	if(ret != 0){
		pr_err("byd REG_SFR_OTP_DATA spi_sync failed.ret=%d\n", ret);
		return -1;
	}
	FP_DBG("%s:WRITE otp_addr_l:0x%x\n",__func__,otp_addr_l);
	ret = byd_fps_spi_write(this_byd_fps->spi, REG_SFR_OTP_DATA, otp_addr_h);
	if(ret != 0){
		pr_err("byd REG_SFR_OTP_DATA spi_sync failed.ret=%d\n", ret);
		return -1;
	}
	FP_DBG("%s:WRITE otp_addr_h:0x%x\n",__func__,otp_addr_h);
	//3. read data from otp.
	//read data, low byte.
	ret = byd_fps_spi_read(this_byd_fps->spi, REG_SFR_OTP_DATA);
	if(ret < 0){
		pr_err("byd REG_SFR_OTP_DATA spi_sync failed.ret=%d\n", ret);
		return -1;
	}
	pbuf[0] = (unsigned char)ret;
	FP_DBG("%s:pbuf[0]:0x%x\n",__func__,pbuf[0]);
	
	//#ifdef CONFIG_FPS12
	if(byd_fps_chip_flag == 0x12){
		//read data, high byte.
		ret = byd_fps_spi_read(this_byd_fps->spi, REG_SFR_OTP_DATA);
		if(ret < 0){
			pr_err("byd REG_SFR_OTP_DATA spi_sync failed.ret=%d\n", ret);
			return -1;
		}
		pbuf[1] = ret;
		FP_DBG("%s:pbuf[1]:0x%x\n",__func__,pbuf[1]);
		//4. read the rest of data from otp.
		for(i=0; i<addr_len-1; i++) {
		 //address add one.
			ret = byd_fps_spi_write(this_byd_fps->spi, REG_SFR_OTP_CMD, SFR_OTP_POINT_INC);
			if(ret != 0){
				pr_err("byd REG_SFR_OTP_CMD spi_sync failed.ret=%d\n", ret);
				return -1;
			}
			//read data, low byte. Little-endian mode
			ret = byd_fps_spi_read(this_byd_fps->spi, REG_SFR_OTP_DATA);
			if(ret < 0){
				pr_err("byd REG_SFR_OTP_DATA spi_sync failed.ret=%d\n", ret);
				return -1;
			}
			pbuf[2+i*2] = ret;
			//DBG_TIME("%s:pbuf[2+i*2]:0x%x\n",__func__,pbuf[2+i*2]);
			ret = byd_fps_spi_read(this_byd_fps->spi, REG_SFR_OTP_DATA);
			if(ret < 0){
				pr_err("byd REG_SFR_OTP_DATA spi_sync failed.ret=%d\n", ret);
				return -1;
			}
			pbuf[3+i*2] = ret;
			//DBG_TIME("%s:pbuf[3+i*2]:0x%x\n",__func__,pbuf[3+i*2]);
		}
	}
	//#else
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x22)){
		//4. read the rest of data from otp.
		for(i=0; i < (addr_len-1); i++) {
		 //address add one.
			ret = byd_fps_spi_write(this_byd_fps->spi, REG_SFR_OTP_CMD, SFR_OTP_POINT_INC);
			if(ret != 0){
				pr_err("byd REG_SFR_OTP_CMD spi_sync failed.ret=%d\n", ret);
				return -1;
			}
			//read data, low byte. Little-endian mode
			ret = byd_fps_spi_read(this_byd_fps->spi, REG_SFR_OTP_DATA);
			if(ret < 0){
				pr_err("byd REG_SFR_OTP_DATA spi_sync failed.ret=%d\n", ret);
				return -1;
			}
			pbuf[i+1] = ret;
			FP_DBG("%s:pbuf[%d]:0x%x\n",__func__,i+1,pbuf[i+1]);
		}
		
	}
	//for little endian
	if((addr_len > 1)&&(change_flag == 1)){
		for(i=0;i<addr_len;i=i+2){
			temp = pbuf[i];
			pbuf[i] = pbuf[i+1];
			pbuf[i+1] = temp;
				
		}
	}

	//5. read end.
	ret = byd_fps_spi_write(this_byd_fps->spi, REG_SFR_OTP_CMD, SFR_OTP_RD_END);
	FP_DBG("%s:OUT\n",__func__);
	return ret;
}
#endif

#ifdef CONFIG_FPS11
#define OTP_CHIPKEY_H  0x22
#define OTP_CHIPKEY_L  0x23
static int byd_fps_read_chip(void)
{
    unsigned char chipkey_h,chipkey_l;
	int chip_key_h,chip_key_l;
	int chipkey;
	
	chip_key_h = byd_fps_otp_read(OTP_CHIPKEY_H, 1, &chipkey_h,0);
	if (chip_key_h == -1) {
		pr_err("%s failed, chip_key_h = %d\n", __func__, chip_key_h);
		return chip_key_h;
	}
	
	chip_key_l = byd_fps_otp_read(OTP_CHIPKEY_L, 1, &chipkey_l,0);
	if (chip_key_l == -1) {
		pr_err("%s failed, chip_key_l = %d\n", __func__, chip_key_l);
		return chip_key_l;
	}
	
	chipkey = (chipkey_h << 8) | chipkey_l;
	
	printk("%s:chipkey=0x%x\n", __func__, chipkey);
	
	return chipkey;
	
}
#endif 
/*******************************************************************************
 * Function    :  byd_fps_get_finger_state
 * Description :  check finger state.
 * In          :  *spi
 * Return      :  0/1 -- finger state, -1 -- failed,
 *******************************************************************************/
unsigned char byd_fps_get_finger_state(struct spi_device *spi)
{
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		return (unsigned char)byd_fps_spi_read(spi, REG_FINGER_STATE);
	}
	if(byd_fps_chip_flag == 0x22){
		return (unsigned char)byd_fps_spi_read(spi, REG_FINGER_STATE22);
	}
	return 0;
}


/*******************************************************************************
 * Function    :  byd_fps_get_finger_threshold
 * Description :  check finger threshold.
 * In          :  *spi
 * Return      :  0/1 -- finger threshold, -1 -- failed,
 *******************************************************************************/
 int byd_fps_get_finger_threshold(struct spi_device *spi)
{
	int ret;
	unsigned int value_on=0,value_off=0;
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_spi_read(spi, REG_FPD_TH_ON_H);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_ON_H spi_sync failed.%d\n", ret);
			return ret;
		}
		
		DBG_TIME("%s:REG_FPD_TH_ON_H=0x%x\n", __func__, ret);
		value_on = (unsigned char)ret;
	
		ret = byd_fps_spi_read(spi, REG_FPD_TH_ON_L);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_ON_L spi_sync failed.%d\n", ret);
			return ret;
		}

		DBG_TIME("%s:REG_FPD_TH_ON_L=0x%x\n", __func__, ret);
		value_on = (value_on<<8)|(unsigned char)ret;
		DBG_TIME("%s:REG_FPD_TH_ON=0x%x\n", __func__, value_on);
	
		ret = byd_fps_spi_read(spi, REG_FPD_TH_OFF_H);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_ON_L spi_sync failed.%d\n", ret);
			return ret;
		}
		DBG_TIME("%s:REG_FPD_TH_OFF_H=0x%x\n", __func__, ret);
		value_off = (unsigned char)ret;
	
		ret = byd_fps_spi_read(spi, REG_FPD_TH_OFF_L);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_OFF_L spi_sync failed.%d\n", ret);
			return ret;
		}
		DBG_TIME("%s:REG_FPD_TH_OFF_L=0x%x\n", __func__, ret);
		value_off = (value_off<<8)|(unsigned char)ret;
		DBG_TIME("%s:REG_FPD_TH_OFF=0x%x\n", __func__, value_off);
	}else if(byd_fps_chip_flag == 0x22){
		ret = byd_fps_spi_read(spi, REG_FPD_TH_ON_H22);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_ON_H spi_sync failed.%d\n", ret);
			return ret;
		}
		
		DBG_TIME("%s:REG_FPD_TH_ON_H22=0x%x\n", __func__, ret);
		value_on = (unsigned char)ret;
	
		ret = byd_fps_spi_read(spi, REG_FPD_TH_ON_L22);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_ON_L spi_sync failed.%d\n", ret);
			return ret;
		}

		DBG_TIME("%s:REG_FPD_TH_ON_L22=0x%x\n", __func__, ret);
		value_on = (value_on<<8)|(unsigned char)ret;
		DBG_TIME("%s:REG_FPD_TH_ON=0x%x\n", __func__, value_on);
	
		/*REG_FPD_TH_ON_H2, REG_FPD_TH_ON_L2*/
		ret = byd_fps_spi_read(spi, REG_FPD_TH_ON_H2_22);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_ON_H2_22 spi_sync failed.%d\n", ret);
			return ret;
		}
		
		DBG_TIME("%s:REG_FPD_TH_ON_H2_22=0x%x\n", __func__, ret);
		value_on = (unsigned char)ret;
	
		ret = byd_fps_spi_read(spi, REG_FPD_TH_ON_L2_22);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_ON_L2 spi_sync failed.%d\n", ret);
			return ret;
		}

		DBG_TIME("%s:REG_FPD_TH_ON_L2_22=0x%x\n", __func__, ret);
		value_on = (value_on<<8)|(unsigned char)ret;
		DBG_TIME("%s:REG_FPD_TH_ON2=0x%x\n", __func__, value_on);
	
	/*REG_FPD_TH_OFF_H, REG_FPD_TH_OFF_L*/
		ret = byd_fps_spi_read(spi, REG_FPD_TH_OFF_H22);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_ON_L spi_sync failed.%d\n", ret);
			return ret;
		}
		DBG_TIME("%s:REG_FPD_TH_OFF_H22=0x%x\n", __func__, ret);
		value_off = (unsigned char)ret;
	
		ret = byd_fps_spi_read(spi, REG_FPD_TH_OFF_L22);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_OFF_L spi_sync failed.%d\n", ret);
			return ret;
		}
		DBG_TIME("%s:REG_FPD_TH_OFF_L22=0x%x\n", __func__, ret);
		value_off = (value_off<<8)|(unsigned char)ret;
		DBG_TIME("%s:REG_FPD_TH_OFF=0x%x\n", __func__, value_off);
	
		/*REG_FPD_TH_OFF_H2, REG_FPD_TH_OFF_L2*/
		ret = byd_fps_spi_read(spi, REG_FPD_TH_OFF_H2_22);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_ON_H2 spi_sync failed.%d\n", ret);
			return ret;
		}
		DBG_TIME("%s:REG_FPD_TH_OFF_H2_22=0x%x\n", __func__, ret);
		value_off = (unsigned char)ret;
	
		ret = byd_fps_spi_read(spi, REG_FPD_TH_OFF_L2_22);
		if (ret < 0) {
			pr_err("byd REG_FPD_TH_OFF_L2 spi_sync failed.%d\n", ret);
			return ret;
		}
		DBG_TIME("%s:REG_FPD_TH_OFF_L2_22=0x%x\n", __func__, ret);
		value_off = (value_off<<8)|(unsigned char)ret;
		DBG_TIME("%s:REG_FPD_TH_OFF2=0x%x\n", __func__, value_off);
		
	}
	return (value_on << 16 | value_off);
}

/*******************************************************************************
 * Function    :  byd_fps_get_finger_detect_value
 * Description :  check finger value.
 * In          :  *spi
 * Return      :  0/1 -- finger value, -1 -- failed,
 *******************************************************************************/
 int byd_fps_get_finger_detect_value(struct spi_device *spi)
{
	int ret;
	unsigned char value_h = 0,value_l = 0;
	unsigned char i;
	unsigned char j=0;
	//byd_fps_check_reg(spi);
	if(byd_fps_chip_flag == 0x11){
		value_h = 0;
		value_l = 0;
		for(i=0,j=0;i<OS_AREA_BLOCK11;i++) {
			//ret = byd_fps_rd_fg_os_cfg(spi,i);
			//DBG_TIME("%s:os:0x%x\n",__func__,ret);
			ret = byd_fps_spi_write(spi, REG_SUB_VALUE_SEL, i);
			if(ret != 0){
				pr_err("byd REG_SUB_VALUE_SEL spi_sync failed.\n");
				return -1;
			}
		
			ret = byd_fps_spi_read(spi, REG_SUB_VALUE_H);
			if (ret < 0)
				return ret;
			DBG_TIME("%s:SUB_H[%d]=0x%x\n", __func__, i, ret);
			value_h = (unsigned char)ret;
		
			byd_fps_sub_value11[j] = value_h;
		
			ret = byd_fps_spi_read(spi, REG_SUB_VALUE_L);
			if (ret < 0)
				return ret;
			DBG_TIME("%s:SUB_L[%d]=0x%x\n", __func__, i, ret);
			value_l = (unsigned char)ret;
		
			byd_fps_sub_value11[j+1] = value_l;
		
			j+=2;
		}
	}
	if(byd_fps_chip_flag == 0x12){
		value_h = 0;
		value_l = 0;
		printk("%s:fps12 get finger scan data\n",__func__);
		for(i=0,j=0;i<OS_AREA_BLOCK12;i++) {
			//ret = byd_fps_rd_fg_os_cfg(spi,i);
			//DBG_TIME("%s:os:0x%x\n",__func__,ret);
			ret = byd_fps_spi_write(spi, REG_SUB_VALUE_SEL, i);
			if(ret != 0){
				pr_err("byd REG_SUB_VALUE_SEL spi_sync failed.\n");
				return -1;
			}
		
			ret = byd_fps_spi_read(spi, REG_SUB_VALUE_H);
			if (ret < 0)
				return ret;
			DBG_TIME("%s:SUB_H_12[%d]=0x%x\n", __func__, i, ret);
			value_h = (unsigned char)ret;
		
			byd_fps_sub_value12[j] = value_h;
		
			ret = byd_fps_spi_read(spi, REG_SUB_VALUE_L);
			if (ret < 0)
				return ret;
			DBG_TIME("%s:SUB_L_12[%d]=0x%x\n", __func__, i, ret);
			value_l = (unsigned char)ret;
		
			byd_fps_sub_value12[j+1] = value_l;
		
			j+=2;
		}
	}
	
	if(byd_fps_chip_flag == 0x22){
		value_h = 0;
		value_l = 0;
		printk("%s:fps12 get finger scan data\n",__func__);
		for(i=0,j=0;i<OS_AREA_BLOCK22;i++) {
			//ret = byd_fps_rd_fg_os_cfg(spi,i);
			//DBG_TIME("%s:os:0x%x\n",__func__,ret);
			ret = byd_fps_spi_write(spi, REG_SUB_VALUE_SEL22, i);
			if(ret != 0){
				pr_err("byd REG_SUB_VALUE_SEL spi_sync failed.\n");
				return -1;
			}
		
			ret = byd_fps_spi_read(spi, REG_SUB_VALUE_H22);
			if (ret < 0)
				return ret;
			DBG_TIME("%s:SUB_H_12[%d]=0x%x\n", __func__, i, ret);
			value_h = (unsigned char)ret;
		
			byd_fps_sub_value22[j] = value_h;
		
			ret = byd_fps_spi_read(spi, REG_SUB_VALUE_L22);
			if (ret < 0)
				return ret;
			DBG_TIME("%s:SUB_L_12[%d]=0x%x\n", __func__, i, ret);
			value_l = (unsigned char)ret;
		
			byd_fps_sub_value22[j+1] = value_l;
		
			j+=2;
		}
	}

	return (value_h << 8 | value_l);
}

/*********************************************************************************
 * byd_fps_fg_detect_mode_cfg() - 配置检测手指按下/抬起
 * @up_down_flag:	check if finger is off/on sensor.
 * @quick_slow_flag:
 *
 **********************************************************************************/
#define BYD_FPS_FG_CHECK_UP				0
#define BYD_FPS_FG_CHECK_DOWN			1
#define BYD_FPS_FG_CHECK_SLOW			0
#define BYD_FPS_FG_CHECK_FAST			1
#if 0
int byd_fps_fg_detect_mode_cfg(unsigned char up_down_flag)
{
	int ret;
	unsigned char tx[4]={(REG_FINGER_STATE<<1)|SPI_WR, 0, (REG_FINGER_STATE<<1)|SPI_RD,0xFF}, rx[4]={0xFF, 0xFF, 0xFF, 0xFF};
	if(up_down_flag == 0 || up_down_flag == 1) {
		ret = byd_fps_spi_read(this_byd_fps->spi, REG_FINGER_STATE);
		
		finger_present = (unsigned char)ret
		
		ret = byd_fps_spi_xfer(this_byd_fps->spi, 4, tx, rx);
		ret = byd_fps_spi_write(this_byd_fps->spi, REG_FINGER_STATE, (1-up_down_flag));
	} else {
		pr_err("%s: up_down_flag not 0 or 1", __func__);
		return -1;
	}
}
#endif
#if 0
// *******************************************************************************
// * Function    :  byd_fps_chip_check_fng_os_data
// * Description :  check finger detect os_cfg value.
// * In          :  *spi
// * Return      :  0/1 -- finger state, -1 -- failed,
// *******************************************************************************
int byd_fps_chip_check_fng_os_data(struct spi_device *spi)
{
	int i;
	int ret;
	unsigned short os_cfg_h, os_cfg_l;
	for(i=0; i<=9; i++) {
		ret = byd_fps_spi_write(spi, REG_FINGER_OS_DATA_SEL, i);
		if(ret != 0){
			pr_err("byd REG_FINGER_OS_DATA_SEL spi_sync failed.\n");
			return -1;
		}
		os_cfg_h = byd_fps_spi_read(spi, REG_FINGER_OS_DATA_H);
		if(ret < 0){
			pr_err("byd REG_FINGER_OS_DATA_H spi_sync failed.\n");
			return -1;
		}
		os_cfg_l = byd_fps_spi_read(spi, REG_FINGER_OS_DATA_L);
		if(ret < 0){
			pr_err("byd REG_FINGER_OS_DATA_L spi_sync failed.\n");
			return -1;
		}
		DBG_TIME("byd fng_os_data[%d]=0x%x\n", i, (os_cfg_h<<8)|os_cfg_l);
	}
	return (os_cfg_h << 8 ) | os_cfg_l;
}
#endif
void byd_fps_fae_para_12(void)
{
	int i=0;
	unsigned short byd_fps_fae_detect_para_cp[5] = BYD_FPS_FAE_DETECT12;
	//unsigned char byd_fps_fae_detect[10] = {0};//BYD_FPS_FAE_DETECT;
	unsigned char byd_fps_fae_image_cp[4] = BYD_FPS_FAE_IMAGE12;
	unsigned short byd_fps_fae_fir_cp[64] = BYD_FPS_FAE_FIR12;
	for(i=0;i<5;i++){
		byd_fps_fae_detect_para[i] = byd_fps_fae_detect_para_cp[i];
	}
	for(i=0;i<4;i++){
		byd_fps_fae_image[i] = byd_fps_fae_image_cp[i];
	}
	for(i=0;i<64;i++){
		byd_fps_fae_fir[i] = byd_fps_fae_fir_cp[i];
	}
}
void byd_fps_fae_para_11(void)
{
	int i=0;
	unsigned short byd_fps_fae_detect_para_cp[5] = BYD_FPS_FAE_DETECT11;
	//unsigned char byd_fps_fae_detect[10] = {0};//BYD_FPS_FAE_DETECT;
	unsigned char byd_fps_fae_image_cp[4] = BYD_FPS_FAE_IMAGE11;
	unsigned short byd_fps_fae_fir_cp[64] = BYD_FPS_FAE_FIR11;
	for(i=0;i<5;i++){
		byd_fps_fae_detect_para[i] = byd_fps_fae_detect_para_cp[i];
	}
	for(i=0;i<4;i++){
		byd_fps_fae_image[i] = byd_fps_fae_image_cp[i];
	}
	for(i=0;i<64;i++){
		byd_fps_fae_fir[i] = byd_fps_fae_fir_cp[i];
	}
}

void byd_fps_fae_para_22(void)
{
	int i=0;
	unsigned short byd_fps_fae_detect_para_cp[BYD_FPS_FAE_DETECT_LEN] = BYD_FPS_FAE_DETECT22;
	//unsigned char byd_fps_fae_detect[10] = {0};//BYD_FPS_FAE_DETECT;
	unsigned char byd_fps_fae_image_cp[3] = BYD_FPS_FAE_IMAGE22;
	for(i=0;i<BYD_FPS_FAE_DETECT_LEN;i++){
		byd_fps_fae_detect_para22[i] = byd_fps_fae_detect_para_cp[i];
	}
	for(i=0;i<3;i++){
		byd_fps_fae_image[i] = byd_fps_fae_image_cp[i];
	}
}

int byd_fps_chip_error_state22(struct spi_device *spi)
{
	int ret;
	ret = byd_fps_spi_read(spi, REG_CHECK_ERROR_STATE22);
	if(ret < 0){
		pr_err("byd REG_CHECK_ERROR_STATE22 spi_sync failed.\n");
		return -1;
	}
	DBG_TIME("%s: Read REG_CHECK_ERROR_STATE22 =%d \n", __func__, ret);
	return ret;
}


void byd_fps_chip_para_init(unsigned char chip_flag)
{
	if(chip_flag == 0x11){
		
		byd_fps_fae_para_11();
	}else if(chip_flag == 0x12){

		byd_fps_fae_para_12();
		
	}else if(chip_flag == 0x22){
		byd_fps_fae_para_22();
		
	}
	
}
int byd_fps_sensor_image_para_init(unsigned char img_w, unsigned char img_h)
{
	BYD_FPS_IMAGE_WIDTH = img_w;
	BYD_FPS_IMAGE_HEIGHT = img_h;
	BYD_FPS_DATA_WIDTH = img_w*3/2;
	BYD_FPS_DATA_HEIGHT = img_h;

	BYD_FPS_IMAGE_SIZE = (img_w)*(img_h)*BYD_FPS_DATA_BYTE;
	BYD_FPS_IMAGE_BUFFER_SIZE = (((img_w)*2+(BYD_FPS_SPI_DUMMY_NUM))*(img_h)+1);
	
	return 0;
}

void byd_fps_cfg_all_reg(struct spi_device*spi)
{
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		printk("%s:byd fps11&12 cfg register start\n",__func__);
		byd_fps_chip_idle(spi);
		#ifdef BYD_FPS_BUF_CFG
		byd_fps_set_fae_para(spi);
		#endif
		#ifdef BYD_FPS_CFG_SEPERATE
		byd_fps_cfg_ana_seq(spi);
		byd_fps_cfg_fp_scan(spi);
		byd_fps_cfg_fg_scan(spi);
		byd_fps_cfg_mode(spi);
		byd_fps_cfg_system(spi);
		byd_fps_cfg_ana_para(spi);
		#endif
	}
	if(byd_fps_chip_flag == 0x22){
		printk("%s:byd fps22 cfg register start\n",__func__);
		byd_fps_cfg_reg_module22();
	}
}
// *******************************************************************************
// * Function    :  byd_fps_config_sensor_poweron
// * Description :  configure chip when power on reset or reset by software.
// * In          :  *spi
// * Return      :  0 -- success, -1 -- failed
// *******************************************************************************
unsigned short byd_fps_chip_type = 0x0000;
int byd_fps_config_sensor_poweron(struct spi_device *spi)
{	
	int ret;
	ret = 0;
	//设置版本号，获取算法版本号
	FP_DBG("%s:go to version\n",__func__);
	byd_fps_mdelay(20);	//wait if chip not finish 'POR'.

	byd_fps_chip_idle(spi);
	
	byd_fps_mdelay(1);
	
	//test
	ret = byd_fps_chip_state(spi);
	DBG_TIME("%s:1 chip mode=%d\n", __func__, ret);
	
	byd_chip_info = byd_fps_read_version(spi);
	//FP_DBG("%s: chip version=0x%x\n",__func__, byd_chip_info);
	
	/*if(BYD_FPS_CONFIG_VERSION != 0)*/{
		//if((byd_chip_info&0xC0C7)!= BYD_FPS_CONFIG_VERSION)
		if((byd_chip_info&0xf8c0)== BYD_FPS_CONFIG_VERSION11){
			byd_fps_chip_flag = 0x11;
			byd_fps_read_chip_key_11(spi);
			byd_fps_alg_version_init(byd_chip_11,11);
			byd_fps_sensor_image_para_init(BYD_FPS_SENSOR_WIDTH11, BYD_FPS_SENSOR_HEIGHT11);
			byd_fps_spi_speed(spi,SPI_FPS11_FPS12_SPEED);
		}else if((byd_chip_info&0xf8c0)== BYD_FPS_CONFIG_VERSION12){
			byd_fps_chip_flag = 0x12;
			byd_fps_read_chip_key_12(spi);
			byd_fps_alg_version_init(byd_chip_12,12);
			byd_fps_sensor_image_para_init(BYD_FPS_SENSOR_WIDTH12, BYD_FPS_SENSOR_HEIGHT12);
			byd_fps_spi_speed(spi,SPI_FPS11_FPS12_SPEED);
		}else if((byd_chip_info&0xf8c3) == (BYD_FPS_CONFIG_VERSION22|0x0001)) {	//0x7741, 0x7041
			byd_fps_alg_version_init(byd_chip_22,31);
			byd_fps_sensor_image_para_init(BYD_FPS_SENSOR_WIDTH_5531, BYD_FPS_SENSOR_HEIGHT_5531);
			byd_fps_chip_flag = 0x22;
			byd_fps_chip_type = 0x5531;
			byd_fps_spi_speed(spi,SPI_FP_SPEED);
		}else if((byd_chip_info&0xf8c3) == (BYD_FPS_CONFIG_VERSION22|0x0002)) {	//0x7742, 0x7042
			byd_fps_alg_version_init(byd_chip_22,32);
			byd_fps_sensor_image_para_init(BYD_FPS_SENSOR_WIDTH_5531, BYD_FPS_SENSOR_HEIGHT_5531);
			byd_fps_spi_speed(spi,SPI_FP_SPEED);
			byd_fps_chip_flag = 0x22;
			byd_fps_chip_type = 0x5532;
		}else if((byd_chip_info&0xf8c3) == BYD_FPS_CONFIG_VERSION22) {	//0x7040
			byd_fps_alg_version_init(byd_chip_22,41);
			byd_fps_sensor_image_para_init(BYD_FPS_SENSOR_WIDTH_5541, BYD_FPS_SENSOR_HEIGHT_5541);
			byd_fps_spi_speed(spi,SPI_FP_SPEED);
			byd_fps_chip_flag = 0x22;
			byd_fps_chip_type = 0x5541;
		}else{
			pr_err("%s: Fatal Error: chip not bf66xx !!!\n", __func__);
			return -10;
		}
		printk("%s:byd init fae para step 1 start\n",__func__);
		byd_fps_chip_para_init(byd_fps_chip_flag);
		printk("%s:byd init fae para step 1 end\n",__func__);
	}
	//byd_chip = byd_fps_rd_fg_os_cfg(spi,9);
	printk("%s:byd init fae para step 2 start\n",__func__);
	byd_fps_init_fae_para();	//Init fae param.
	printk("%s:byd init fae para step 2 end\n",__func__);
	
	printk("%s:byd cfg register start\n",__func__);
	byd_fps_cfg_all_reg(spi);
	printk("%s:byd cfg register end\n",__func__);
	byd_fps_mdelay(1);
	byd_fps_chip_fg_detect(spi);


	return 0;
}

int byd_fps_finger_detect_cfg(struct spi_device *spi, const unsigned char *tx)
{
	return 0;
}

void byd_fps_finger_detect_pre(unsigned char up_down_flag,unsigned char quick_slow_flag)
{
	
}

void byd_fps_finger_detect_pre_cfg(unsigned char up_down_flag,unsigned char quick_slow_flag)
{
	int ret;
	if(quick_slow_flag == 0) {
		
	}
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_spi_write(this_byd_fps->spi, REG_FINGER_STATE, (1-up_down_flag));
	}else if(byd_fps_chip_flag == 0x22){
		ret = byd_fps_spi_write(this_byd_fps->spi, REG_FINGER_STATE22, (1-up_down_flag));
		
	}
	byd_fps_chip_detect(this_byd_fps->spi);
}


#if !defined(DEBUG) && defined(BYD_FPS_FINGER_DETECT_VALUE)
#include <linux/jiffies.h>
#include <linux/delay.h>

unsigned long start_time = 0;
#endif

#define FINGER_SAFE_MARGIN	50
#define FINGER_THRESHOLD_MIN	60000

/*********************************************************************************
 * byd_fps_check_fg_state_timeout() - 等待手指按下/抬起
 * @byd_fps:	fingerprint data.
 *				refer to struct byd_fps for more information.
 * @finger_request:
 * @timeout:	maximum waiting time.
 *				function would return when timeout exceed given value.
 *
 *  This function will block while waiting for finger touching sensor or leaving
 * sensor.
 *
 * The maxmum timout can be passed in during function call, return value are
 *
 *  0   -- 手指状态符合
 *
 * -1   -- SPI通信错误
 *
 * -2   -- 被APP层发送命令取消操作
 *
 * -110 -- 检测超时(-ETIMEDOUT)
 *
 * -512 --等待事件的过程被信号中断(-ERESTARTSYS)
 *
 * Return: status for finger detection above
 **********************************************************************************/
//int byd_fps_check_fg_state_timeout(struct byd_fps_data *byd_fps, unsigned char finger_request, int timeout)
int byd_fps_auto_wait_finger_up_down_timeout(struct byd_fps_data *byd_fps, unsigned char finger_request, int timeout)
{
	int ret = 0;
	int rest_time = 0;
	//unsigned char value_h,value_l;
	//int value;
	unsigned char finger_state;
	unsigned char int_state;
	struct spi_device *spi = byd_fps->spi;
	//unsigned char tx[4]={(REG_FINGER_STATE<<1)|SPI_WR, 0, (REG_FINGER_STATE<<1)|SPI_RD,0xFF}, rx[4]={0xFF, 0xFF, 0xFF, 0xFF};
	//byd_fps_check_reg(byd_fps->spi);
	//byd_fps_read_version(byd_fps->spi);
	if(finger_request != 0 && finger_request != 1) {
		pr_err("%s: up_down_flag not 0 or 1", __func__);
		return -3;
	} else {
		
		//ret = byd_fps_spi_read(this_byd_fps->spi, REG_FINGER_STATE);
		finger_present = byd_fps_get_finger_state(this_byd_fps->spi);
		if(finger_request == finger_present) {
			#ifdef BYD_FPS_TEST_TIME_LOG
			DBG_TIME("\n%s: finger_request=0x%x,finger_present=0x%x,timeout=%d\n", __func__, finger_request, finger_present, timeout);
			#endif

			if(finger_request == 0 && Os_Scan_Expired == 1
				&& Os_Otp_or_Scan != OS_IN_OTP && Os_Otp_or_Scan != OS_IN_OTP_LEGACY)
					byd_os_start_scan();

			return 0;
		} else {
			goto fg_check;
		}
		
	}

fg_check:
	rest_time = timeout;	// set rest_time with variables (timeout).
	byd_fps->interrupt_done = 0;
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_spi_write(spi, REG_FG_REST_TI1_EN, 1);//apk界面使用快定时
		if(ret != 0){
			pr_err("byd REG_FG_REST_TI1_EN spi_sync failed.\n");
			return -1;
		}
	}
	byd_fps_chip_detect(spi);	//detect fng
	
	#if !defined(DEBUG) && defined(BYD_FPS_FINGER_DETECT_VALUE)
	/* 通过观察手指按下到出手指中断时间，确定手指探测灵敏度BYD_FPS_FAE_FINGER_DOWN_DIFF的设置 */
	if(BYD_FPS_CHECK_SUB_AREA && finger_request == 1 && start_time == 0 ) {
		unsigned short value = 0;

		do {
			mdelay(5);
			value = byd_fps_get_finger_detect_value(spi);
			if (value < FINGER_THRESHOLD_MIN) {
				pr_err("byd_fps_get_finger_detect_value() failed.\n");
				break;
			}
			//DBG_TIME("CURRENT:0x%x(%d) \n", value, value);
		} while (value > th_up);

		start_time = jiffies;
		DBG_TIME("START - th_up:0x%x(%d), current:0x%x(%d) \n", th_up, th_up, value, value);
	} // BYD_FPS_CHECK_SUB_AREA
	rest_time = 10;
	#endif
	
	while((rest_time > 0)&&(finger_present != finger_request)) {	//time == 0, 转到读状态寄存器查询.
		if (byd_fps_check_thread_should_stop()) return -EINTR;


		ret = byd_fps_wait_event_interruptible_timeout(rest_time);

		#ifdef BYD_FPS_TEST_TIME_LOG
		DBG_TIME("%s:WAIT END, rest_time=%d\n", __func__, ret);
		#endif
		if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
			if(byd_fps->interrupt_done == 1) {
				byd_fps->interrupt_done = 0;
				ret = byd_fps_spi_write(spi, REG_INT_CFG, VAL_INT_CFG&0x7F);	//清除中断信号,电平恢复.
				if(ret != 0){
					pr_err("byd REG_ INT_STATE spi_sync failed.\n");
					return -1;
				}
			}
		}

		if(ret >= 0){
			rest_time = ret;
		} else {
			pr_err("%s:ret=%d\n", __func__, ret);
			return ret;
		}
		if (byd_fps_check_thread_should_stop()) 
			return -EINTR;

		//通过观察SUB_D4的值，决定中断触发阈值： REG_FPD_TH_H, REG_FPD_TH_L
		if(BYD_FPS_CHECK_SUB_AREA){
			unsigned short value = 0;
			value = byd_fps_get_finger_detect_value(spi);
			if (value < 0)
				pr_err("byd_fps_get_finger_detect_value() failed.\n");
			DBG_TIME("%s :SUB_B4_VALUE = 0x%x (%d)\n",__func__, value, value);
			
			value = byd_fps_get_finger_threshold(spi);
			if (value < 0)
				pr_err("byd_fps_get_finger_threshold() failed.\n");
			DBG_TIME("%s :FPD_THRESHOLD = 0x%x (%d)\n",__func__, value, value);
		}
		//check int_flag
		int_state = (unsigned char)byd_fps_chip_intr_state(spi);
		FP_DBG("%s: INT_state = 0x%x\n" ,__func__,int_state);
	
		//清零中断状态
		byd_fps_chip_intr_flag_clr(spi,0);

		if(int_state & IRQ_INT_FINGER) {
			finger_state = byd_fps_get_finger_state(spi);
			FP_DBG("%s: finger_state = 0x%x\n" ,__func__,finger_state);
			finger_present = finger_state;
		}

        if (byd_fps_check_thread_should_stop()) return -EINTR;
	}	//while

	if(rest_time) {
		#if !defined(DEBUG) && defined(BYD_FPS_FINGER_DETECT_VALUE)
		unsigned short value = 0;
		if (BYD_FPS_CHECK_SUB_AREA && finger_request == 1 /*&& finger_intr_num == 1*/) {
			mdelay(5);
			value = byd_fps_get_finger_detect_value(spi);
			if (value < 0)
				pr_err("byd_fps_get_finger_detect_value() failed.\n");

			DBG_TIME("STOP  - th_up:0x%x(%d), current:0x%x(%d), time(%ld-%ld):%d \n",
					th_up, th_up, value, value,
					jiffies, start_time, jiffies_to_msecs(jiffies - start_time));
			start_time = 0;
		}
		#endif
		DBG_TIME("%s: finger_state is OK\n" ,__func__);

		if(finger_request == 0 && Os_Scan_Expired == 1
			&& Os_Otp_or_Scan != OS_IN_OTP && Os_Otp_or_Scan != OS_IN_OTP_LEGACY)
				byd_os_start_scan();

		return 0;
	} else {
		FP_DBG("%s: finger_state NOT OK, before TimeOut\n" ,__func__);
		finger_state = byd_fps_get_finger_state(spi);;
		finger_present = finger_state;
		DBG_TIME("%s: in -ETIMEDOUT finger_state is %d\n" ,__func__, finger_state);
		if(finger_present == finger_request ) {

		if(finger_request == 0 && Os_Scan_Expired == 1
			&& Os_Otp_or_Scan != OS_IN_OTP && Os_Otp_or_Scan != OS_IN_OTP_LEGACY)
				byd_os_start_scan();

			return 0;
		}else{
			return -ETIMEDOUT;
		}
	}
	
	/* Finger up threshold update */
	if (finger_request) { // update only limited in condition of down request timeout
		ret = byd_fps_get_finger_detect_value(spi) - FINGER_SAFE_MARGIN;
		if (ret > th_up && ret < 65535) {
			th_up = ret;
			//byd_fps_set_finger_detect_value(spi, th_down);
			DBG_TIME("%s: *** finger up threshold UPDATED*** 0x%x(%d) \n", __func__, th_up, th_up);
		}
	}
}

/*******************************************************************************
 * Function    :  byd_fps_fp_get_image_data
 * Description :  set registers  before scan fingerprint. Can't modify by FAE.
 * In          :  *byd_fps, image_height, sub_full_flag, timeout.
 * Return      :  0 -- success, -1 -- failed
 *******************************************************************************/

int byd_fps_fp_get_image_data(struct byd_fps_data *byd_fps, unsigned char image_height, unsigned char image_width)
{	
	//unsigned char rx_temp;
	unsigned int img_cnt = 0,icnt=0,i;
	struct spi_device *spi;
	FP_DBG("\n\n%s: Start...\n", __func__);
	#ifdef BYD_RD_LINE_NUM
	//DBG_TIME("%s: BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM=%d\n", __func__, BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM);
	#endif
	i = 0;
	spi = byd_fps->spi;
	if(byd_fps_chip_flag == 0x12){
		memset(byd_fps->data_buffer, 0xff, BYD_FPS_IMAGE_BUFFER_SIZE12);
	

		#ifdef BYD_RD_LINE_NUM12//每次读两行
		#ifdef BYD_FPS_TEST_TIME_LOG
		DBG_TIME("%s: read fpd_data12 START\n", __func__);
		#endif
		byd_rd_num = BYD_FPS_IMAGE_HEIGHT12/BYD_RD_LINE_NUM12;
		byd_rd_rest = BYD_FPS_IMAGE_HEIGHT12%BYD_RD_LINE_NUM12;
		if(byd_rd_rest > 0) {
			byd_rd_num = byd_rd_num + 1;
		}
		DBG_TIME("%s: byd_rd_num12=%d,byd_rd_rest12=%d\n", __func__, byd_rd_num, byd_rd_rest);
		DBG_TIME("%s: BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM=%d\n", __func__, BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12);
		for(i=0;i<byd_rd_num;i++) { //read 2 lines/time
			unsigned char rx_temp;
			//#ifdef CONFIG_FPS12
		    unsigned int byd_rd_length=288;
		//#else
			//unsigned int byd_rd_length11=198;
		//#endif
		rx_temp = byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12]; //96*(3/2)*2=640, BYD_FPS_DATA_BYTE=2;
		byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12] = REG_FPD_DATA<<1|SPI_RD;
		{
			if(i<byd_rd_num-1) {
				byd_rd_length = BYD_FPS_IMAGE_WIDTH12*BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12+1;
			} else {
				if(byd_rd_rest>0) {
					byd_rd_length = BYD_FPS_IMAGE_WIDTH12*BYD_FPS_DATA_BYTE*byd_rd_rest+1;
				}else{
					byd_rd_length = BYD_FPS_IMAGE_WIDTH12*BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12+1;
				}
			
			}
			byd_fps_spi_xfer(spi, byd_rd_length,
						&byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12],
						&byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12]);
		}
		if(i > 0){
			byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12] = rx_temp;
		}
		//if(i)
		DBG_TIME("%s: read fpd_data: i=%d,[%d]%x,%x,%x,%x,%x\n", __func__, i, i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12, byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12],
		byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12+1],byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12+2],
		byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12*BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12+3],byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH12 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM12+4]);
	}
	
	#ifdef BYD_FPS_TEST_TIME_LOG
	DBG_TIME("%s: read fpd_data12 END\n", __func__);
	#endif
	
	#else	// BYD_RD_LINE_NUM, BYD_RD_2LINE
	DBG_TIME("%s:transfer start\n",__func__);
	byd_fps->data_buffer[0] = REG_FPD_DATA<<1|SPI_RD;
	byd_fps_spi_xfer(spi, BYD_FPS_IMAGE_BUFFER_SIZE12, byd_fps->data_buffer, byd_fps->data_buffer);
	
	DBG_TIME("%s: READ_IMAGE_DATA is End-2, byd_fps->data_buffer[0]=0x%x,[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,\
			[5]=0x%x,[6]=0x%x,[7]=0x%x,[8]=0x%x,[9]=0x%x\n", __func__, byd_fps->data_buffer[0], byd_fps->data_buffer[1],\
			byd_fps->data_buffer[2], byd_fps->data_buffer[3], byd_fps->data_buffer[4], byd_fps->data_buffer[5],\
			byd_fps->data_buffer[6], byd_fps->data_buffer[7], byd_fps->data_buffer[8], byd_fps->data_buffer[9]);
	DBG_TIME("%s: 1 full frame transfer all in 1, end \n",__func__);
	#endif	//END BYD_RD_LINE_NUM, BYD_RD_2LINE
	
	//deal with : data bytes.
	icnt = BYD_FPS_IMAGE_SIZE12;
	img_cnt=BYD_FPS_IMAGE_BUFFER_SIZE12;
	DBG_TIME("%s:BYD_FPS_IMAGE_SIZE12=%d,BYD_FPS_IMAGE_BUFFER_SIZE12=%d\n", __func__, BYD_FPS_IMAGE_SIZE12, BYD_FPS_IMAGE_BUFFER_SIZE12);

	return BYD_FPS_IMAGE_SIZE12;
	}
	if(byd_fps_chip_flag == 0x11){
		memset(byd_fps->data_buffer, 0xff, BYD_FPS_IMAGE_BUFFER_SIZE11);
	

		#ifdef BYD_RD_LINE_NUM11//每次读两行
		#ifdef BYD_FPS_TEST_TIME_LOG
		DBG_TIME("%s: read fpd_data START\n", __func__);
		#endif
		byd_rd_num = BYD_FPS_IMAGE_HEIGHT11/BYD_RD_LINE_NUM11;
		byd_rd_rest = BYD_FPS_IMAGE_HEIGHT11%BYD_RD_LINE_NUM11;
		if(byd_rd_rest > 0) {
			byd_rd_num = byd_rd_num + 1;
		}
		DBG_TIME("%s: byd_rd_num=%d,byd_rd_rest=%d\n", __func__, byd_rd_num, byd_rd_rest);
		FP_DBG("%s: BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM=%d\n", __func__, BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11);
		for(i=0;i<byd_rd_num;i++) { //read 2 lines/time
			unsigned char rx_temp;
			//#ifdef CONFIG_FPS12
		    unsigned int byd_rd_length=198;
		rx_temp = byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11]; //96*(3/2)*2=640, BYD_FPS_DATA_BYTE=2;
		byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11] = REG_FPD_DATA<<1|SPI_RD;
		{
			if(i<byd_rd_num-1) {
				byd_rd_length = BYD_FPS_IMAGE_WIDTH11*BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11+1;
			} else {
				if(byd_rd_rest>0) {
					byd_rd_length = BYD_FPS_IMAGE_WIDTH11*BYD_FPS_DATA_BYTE*byd_rd_rest+1;
				}else{
					byd_rd_length = BYD_FPS_IMAGE_WIDTH11*BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11+1;
				}
			
			}
			byd_fps_spi_xfer(spi, byd_rd_length,
						&byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11],
						&byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11]);
		}
		if(i > 0){
			byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11] = rx_temp;
		}
		//if(i)
		FP_DBG("%s: read fpd_data: i=%d,[%d]%x,%x,%x,%x,%x\n", __func__, i, i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11, byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11],
		byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11+1],byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11+2],
		byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11*BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11+3],byd_fps->data_buffer[i * BYD_FPS_IMAGE_WIDTH11 *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM11+4]);
	}
	
	#ifdef BYD_FPS_TEST_TIME_LOG
	DBG_TIME("%s: read fpd_data END\n", __func__);
	#endif
	
	#else	// BYD_RD_LINE_NUM, BYD_RD_2LINE
	DBG_TIME("%s:transfer start\n",__func__);
	byd_fps->data_buffer[0] = REG_FPD_DATA<<1|SPI_RD;
	byd_fps_spi_xfer(spi, BYD_FPS_IMAGE_BUFFER_SIZE11, byd_fps->data_buffer, byd_fps->data_buffer);
	
	DBG_TIME("%s: READ_IMAGE_DATA is End-2, byd_fps->data_buffer[0]=0x%x,[1]=0x%x,[2]=0x%x,[3]=0x%x,[4]=0x%x,\
			[5]=0x%x,[6]=0x%x,[7]=0x%x,[8]=0x%x,[9]=0x%x\n", __func__, byd_fps->data_buffer[0], byd_fps->data_buffer[1],\
			byd_fps->data_buffer[2], byd_fps->data_buffer[3], byd_fps->data_buffer[4], byd_fps->data_buffer[5],\
			byd_fps->data_buffer[6], byd_fps->data_buffer[7], byd_fps->data_buffer[8], byd_fps->data_buffer[9]);
	DBG_TIME("%s: 1 full frame transfer all in 1, end \n",__func__);
	#endif	//END BYD_RD_LINE_NUM, BYD_RD_2LINE
	
	//deal with : data bytes.
	icnt = BYD_FPS_IMAGE_SIZE11;
	img_cnt=BYD_FPS_IMAGE_BUFFER_SIZE11;
	DBG_TIME("%s:BYD_FPS_IMAGE_SIZE12=%d,BYD_FPS_IMAGE_BUFFER_SIZE12=%d\n", __func__, BYD_FPS_IMAGE_SIZE11, BYD_FPS_IMAGE_BUFFER_SIZE11);

	return BYD_FPS_IMAGE_SIZE11;
	}
	return 0;
}
/*******************************************************************************
 * Function    :  byd_fps_fp_get_image_data
 * Description :  set registers  before scan fingerprint. Can't modify by FAE.
 * In          :  *byd_fps, image_height, sub_full_flag, timeout.
 * Return      :  0 -- success, -1 -- failed
 *******************************************************************************/
#define BYD_READ_ONCE_LEN	(BYD_FPS_DATA_WIDTH/8)	//64*1.5/8=96/8=12
#define BYD_READ_ONCE_DELAY	70
int byd_rd_delay = 50;//byd_fps_fae_image[2];	//50us
#if 0
int byd_fps_fp_get_image_data22(struct byd_fps_data *byd_fps, unsigned char image_height, unsigned char image_width)
{	
	int ret=0;
	int byd_rd_length;
	unsigned char rx_temp;
	unsigned int lcnt=0,tcnt=0,icnt=0,i;
	unsigned char i_line,j;
	unsigned char finger_state=0;
	struct spi_device *spi;
	
	FP_DBG("\n\n%s: Start...\n", __func__);
	#ifdef BYD_RD_LINE_NUM
	DBG_TIME("%s: BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM=%d\n", __func__, BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM);
	#endif
	i = 0;
	j = 0;
	finger_state=0;
	i_line = 0;
	lcnt = 0;
	tcnt = 0;
	icnt = 0;
	byd_rd_delay = byd_fps_fae_image[2];
	
	spi = byd_fps->spi;
	/*Initial the "byd_fps->data_buffer" with "0xFF". */
	memset(byd_fps->data_buffer, 0xff, BYD_FPS_IMAGE_BUFFER_SIZE);
	
	#ifdef BYD_RD_LINE_NUM		//每次读N行, N<=BYD_FPS_IMAGE_HEIGHT.
		DBG_TIME("%s: read fpd_data START,BYD_FPS_IMAGE_HEIGHT=%d,BYD_RD_LINE_NUM=%d\n", __func__,BYD_FPS_IMAGE_HEIGHT,BYD_RD_LINE_NUM);
		
		byd_rd_num = BYD_FPS_IMAGE_HEIGHT/BYD_RD_LINE_NUM;
		byd_rd_rest = BYD_FPS_IMAGE_HEIGHT%BYD_RD_LINE_NUM;
		if(byd_rd_rest > 0) {
			byd_rd_num = byd_rd_num + 1;
		}
		printk("%s: BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM=%d,byd_rd_num=%d,byd_rd_rest=%d\n", __func__, BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM,byd_rd_num,byd_rd_rest);
		for(i=0;i<byd_rd_num;i++) { //read N lines once, 'byd_rd_num' cycles total.
			rx_temp = byd_fps->data_buffer[i * BYD_FPS_DATA_WIDTH * BYD_RD_LINE_NUM]; //96*(3/2)*2=640, BYD_FPS_DATA_BYTE=2;
			byd_fps->data_buffer[i * BYD_FPS_DATA_WIDTH * BYD_RD_LINE_NUM] = REG_FPD_DATA22<<1|SPI_RD;
			
			if (byd_rd_rest > 0) {
				if (i < byd_rd_num - 1) {
					byd_rd_length = (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM+1;
				} else {
					byd_rd_length = (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * byd_rd_rest+1;
				}
			} else if (byd_rd_rest == 0) {
				byd_rd_length = (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM+1;
			}
			FP_DBG("%s:i=%d(%d),byd_rd_length=%d\n", __func__, i, byd_rd_num, byd_rd_length);
			
		/*#if (BYD_FPS_SPI_DUMMY_ONCE == 0)*/
		#if 0
			DBG_TIME("byd_rd_delay=%d us\n", byd_rd_delay);
			DBG_TIME("%s: %d dummy read fpd_data (multi)\n", __func__, BYD_FPS_SPI_DUMMY_ONCE);	
			for(i_line=0; i_line<BYD_RD_LINE_NUM; i_line++) {
				printk("%s: line = %d\n", __func__, i_line);
				for(j=0; j<8; j++) {
					unsigned char fpd_data_temp=0;
					fpd_data_temp = byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN];
					byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line + j * BYD_READ_ONCE_LEN] = REG_FPD_DATA22<<1|SPI_RD;
					//FP_DBG("byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line + j * BYD_READ_ONCE_LEN](%d)=0x%x\n", BYD_FPS_DATA_WIDTH * i_line + j * BYD_READ_ONCE_LEN, byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line + j * BYD_READ_ONCE_LEN]);
					ret = byd_fps_spi_xfer(spi, BYD_READ_ONCE_LEN+1,
							&byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line+j * BYD_READ_ONCE_LEN],
							&byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line+j * BYD_READ_ONCE_LEN]);
					byd_fps->data_buffer[i * BYD_FPS_DATA_WIDTH * i_line + j*BYD_READ_ONCE_LEN] = fpd_data_temp;
					/*Test image data*/
					{
						unsigned char i=0;
						int index=0;
						printk("byd DBuf: ret(byd_fps_spi_xfer)=%d", ret);
						for(i=0;i<BYD_READ_ONCE_LEN+1;i++) {
							index = BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN + i;
							printk("[%04d]=0x%x,", index, byd_fps->data_buffer[index]);
						}
						printk("\n");
					
					}
					/*
					printk("%s:byd_fps->data_buffer[%d],%d,%d,%d,%d,%d=0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n",__func__,BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN, BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+1, BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+2, BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+3,BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+4, BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+5,
					byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN],byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+1],byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+2],byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+3],byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+4],byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line+ j * BYD_READ_ONCE_LEN+5]
					);*/
					//byd_fps_udelay(byd_rd_delay);
					//byd_fps_udelay(200);
					byd_fps_mdelay(2);
				}
			}
			
			if (i > 0) {
				byd_fps->data_buffer[i * BYD_FPS_DATA_WIDTH * BYD_RD_LINE_NUM] = rx_temp;
			}
			DBG_TIME("%s: no dummy read fpd_data END\n", __func__);
		#else
			
			ret = byd_fps_spi_xfer(spi, byd_rd_length,
				&byd_fps->data_buffer[i * (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM],
				&byd_fps->data_buffer[i * (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM]);
			
			DBG_TIME("byd DBuf(multi:%d(%d line)): ret(byd_fps_spi_xfer)=%d\n", i, BYD_RD_LINE_NUM, ret);
			
			if (i > 0) {
				byd_fps->data_buffer[i * (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM] = rx_temp;
			}
			printk("%s: %d dummy read fpd_data END\n", __func__, BYD_FPS_SPI_DUMMY_NUM/8);
			#ifdef DEBUG
			{
				unsigned int icnt=0;
				int index=0;
				//printk("byd LINE 0:\n");
				for(icnt=1;icnt<byd_rd_length;icnt++) {
					index = i * (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM + icnt;
					printk("byd[%04d]=0x%02x, ", index, byd_fps->data_buffer[index]);
					if((icnt%12) == 0) {
						printk("\n");
					}
					if((icnt%96)==0) {
						printk("byd LINE %d:\n", i*BYD_RD_LINE_NUM+icnt/96);
					}
				}
			}
			#endif
			#endif
		}
		
		#if (BYD_FPS_SPI_DUMMY_ONCE>0)
		//Remove dummy data.
		#if 0
		for(lcnt=0; lcnt<BYD_FPS_DATA_HEIGHT; lcnt++) {
			for(tcnt=0;tcnt<8;tcnt++) {
				for(icnt=1; icnt<=BYD_READ_ONCE_LEN ; icnt++) {
					byd_fps->data_buffer[lcnt*tcnt * BYD_READ_ONCE_LEN + icnt] = byd_fps_tmp->data_buffer[lcnt*tcnt * (BYD_READ_ONCE_LEN + BYD_FPS_SPI_DUMMY_ONCE) +icnt];
				}
			}
		}
		#else
		for(lcnt=0; lcnt<BYD_FPS_DATA_HEIGHT; lcnt++) {
			for(tcnt=0;tcnt<8;tcnt++) {
				for(icnt=1; icnt<=BYD_READ_ONCE_LEN ; icnt++) {
					byd_fps->data_buffer[lcnt*tcnt * BYD_READ_ONCE_LEN + icnt] = byd_fps->data_buffer[(lcnt*tcnt +1)* (BYD_READ_ONCE_LEN + BYD_FPS_SPI_DUMMY_ONCE) +icnt];
				}
			}
		}
		#endif
		
		#endif
	
	#endif	//END BYD_RD_LINE_NUM

	DBG_TIME("%s: read fpd_data END\n", __func__);

	//deal with : data bytes.

	//icnt = BYD_FPS_IMAGE_SIZE;			//64*1.5*72=6912.
	//img_cnt=BYD_FPS_IMAGE_BUFFER_SIZE;	//64*2*72+1=9612+1=9613.
	DBG_TIME("%s:BYD_FPS_IMAGE_SIZE=%d,BYD_FPS_IMAGE_BUFFER_SIZE=%d\n", __func__, BYD_FPS_IMAGE_SIZE, BYD_FPS_IMAGE_BUFFER_SIZE);

	return BYD_FPS_IMAGE_SIZE;
}
#endif
int byd_fps_fp_get_image_data22(struct byd_fps_data *byd_fps, unsigned char image_height, unsigned char image_width)
{	
	int ret=0;
	unsigned int lcnt = 0;
	unsigned int tcnt = 0;
	unsigned int icnt = 0;
	unsigned char i_line,j,i_line_cnt,i,i_line_index;
	unsigned char finger_state=0;
	struct spi_device *spi;
	#ifdef BYD_FPS_SPI_NO_DELAY
	unsigned char rx_temp;
	int byd_rd_length;
	#else
	unsigned char *byd_img_buf=NULL;
	byd_img_buf = (unsigned char *)byd_malloc(BYD_FPS_IMAGE_BUFFER_SIZE);
	memset(byd_img_buf, 0x00, BYD_FPS_IMAGE_BUFFER_SIZE);
	#endif
	
	FP_DBG("\n\n%s: Start...\n", __func__);
	#ifdef BYD_RD_LINE_NUM
	DBG_TIME("%s: BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM=%d\n", __func__, BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM);
	#endif
	j = 0;
	finger_state=0;
	i_line = 0;
	i_line_cnt = 0;
	i_line_index = 0;
	lcnt = 0;
	tcnt = 0;
	icnt = 0;
	byd_rd_delay = byd_fps_fae_image[2];
	
	spi = byd_fps->spi;
	/*Initial the "byd_fps->data_buffer" with "0xFF". */
	memset(byd_fps->data_buffer, 0xff, BYD_FPS_IMAGE_BUFFER_SIZE);
	
	#ifdef BYD_RD_LINE_NUM		//每次读N行, N<=BYD_FPS_IMAGE_HEIGHT.
		DBG_TIME("%s: read fpd_data START,BYD_FPS_IMAGE_HEIGHT=%d,BYD_RD_LINE_NUM=%d\n", __func__,BYD_FPS_IMAGE_HEIGHT,BYD_RD_LINE_NUM);
		
		byd_rd_num = BYD_FPS_IMAGE_HEIGHT/BYD_RD_LINE_NUM;
		byd_rd_rest = BYD_FPS_IMAGE_HEIGHT%BYD_RD_LINE_NUM;
		if(byd_rd_rest > 0) {
			byd_rd_num = byd_rd_num + 1;
		}
		DBG_TIME("%s: BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM=%d,byd_rd_num=%d,byd_rd_rest=%d\n", __func__, BYD_FPS_IMAGE_WIDTH *BYD_FPS_DATA_BYTE*BYD_RD_LINE_NUM,byd_rd_num,byd_rd_rest);
		for(i=0;i<byd_rd_num;i++) { //read N lines once, 'byd_rd_num' cycles total.
		#ifdef BYD_FPS_SPI_NO_DELAY
			rx_temp = byd_fps->data_buffer[i * BYD_FPS_DATA_WIDTH * BYD_RD_LINE_NUM]; //96*(3/2)*2=640, BYD_FPS_DATA_BYTE=2;
			byd_fps->data_buffer[i * BYD_FPS_DATA_WIDTH * BYD_RD_LINE_NUM] = REG_FPD_DATA22<<1|SPI_RD;
			
			if (byd_rd_rest > 0) {
				if (i < byd_rd_num - 1) {
					byd_rd_length = (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM+1;
				} else {
					byd_rd_length = (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * byd_rd_rest+1;
				}
			} else if (byd_rd_rest == 0) {
				byd_rd_length = (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM+1;
			}
			FP_DBG("%s:i=%d(%d),byd_rd_length=%d\n", __func__, i, byd_rd_num, byd_rd_length);
			
			DBG_TIME("%s: %d dummy read fpd_data (multi)\n", __func__, BYD_FPS_SPI_DUMMY_ONCE);	
			ret = byd_fps_spi_xfer(spi, byd_rd_length,
				&byd_fps->data_buffer[i * (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM],
				&byd_fps->data_buffer[i * (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM]);
			printk("byd DBuf(multi:%d(%d line)): ret(byd_fps_spi_xfer)=%d\n", i, BYD_RD_LINE_NUM, ret);
			if (i > 0) {
				byd_fps->data_buffer[i * (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM] = rx_temp;
			}
			DBG_TIME("%s: %d dummy read fpd_data END\n", __func__, BYD_FPS_SPI_DUMMY_NUM/8);
			
			/*Test image data*/
		/*	{
				unsigned int icnt=0;
				int index=0;
				for(icnt=1;icnt<byd_rd_length;icnt++) {
					index = i * (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM + icnt;
					FP_DBG("byd[%04d]=0x%02x, ", index, byd_fps->data_buffer[index]);
					if((icnt%12) == 0) {
						FP_DBG("\n");
					}
					if((icnt%96)==0) {
						FP_DBG("byd LINE %d Finished!\n", i*BYD_RD_LINE_NUM+icnt/96);
					}
				}
			}*/
		#else
			DBG_TIME("byd_rd_delay=%d us\n", byd_rd_delay);
			DBG_TIME("byd_rd_num=%d,(i=%d)\n", byd_rd_num, i);
			if(i<byd_rd_num-1) {
				i_line_cnt = BYD_RD_LINE_NUM;
			} else {
				if(byd_rd_rest>0) {
					i_line_cnt = byd_rd_rest;
				} else {
					i_line_cnt = BYD_RD_LINE_NUM;
				}
			}
			for(i_line=0; i_line<i_line_cnt; i_line++) {
				i_line_index = i_line+i*BYD_RD_LINE_NUM;
				printk("%s: i_line_cnt=%d\n", __func__, i_line_cnt);
				printk("%s: i_line_index=%d\n", __func__, i_line_index);
				printk("%s: line=%d\n", __func__, i_line);
				for(j=0; j<8; j++) {
					unsigned char fpd_data_temp=0,fpd_data_temp_rx=0;
					fpd_data_temp = byd_fps->data_buffer[BYD_FPS_DATA_WIDTH * i_line_index+ j * BYD_READ_ONCE_LEN];
					fpd_data_temp_rx = byd_img_buf[BYD_FPS_DATA_WIDTH * i_line_index+ j * BYD_READ_ONCE_LEN];
					byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line_index + j * BYD_READ_ONCE_LEN] = REG_FPD_DATA22<<1|SPI_RD;
					FP_DBG("byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line_index + j * BYD_READ_ONCE_LEN](%d)=0x%x\n", BYD_FPS_DATA_WIDTH * i_line_index + j * BYD_READ_ONCE_LEN, byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line_index + j * BYD_READ_ONCE_LEN]);
					ret = byd_fps_spi_xfer(spi, BYD_READ_ONCE_LEN+1,
							&byd_fps->data_buffer[ BYD_FPS_DATA_WIDTH * i_line_index+j * BYD_READ_ONCE_LEN],
							&byd_img_buf[ BYD_FPS_DATA_WIDTH * i_line_index+j * BYD_READ_ONCE_LEN]);
					byd_fps->data_buffer[BYD_FPS_DATA_WIDTH * i_line_index+ j * BYD_READ_ONCE_LEN] = fpd_data_temp;
					byd_img_buf[BYD_FPS_DATA_WIDTH * i_line_index+ j * BYD_READ_ONCE_LEN] = fpd_data_temp_rx;
					#ifdef DEBUG
					{
						unsigned char i=0;
						int index=0;
						printk("\nbyd DBuf: ret(byd_fps_spi_xfer)=%d ", ret);
						for(i=0;i<BYD_READ_ONCE_LEN+1;i++) {
							index = BYD_FPS_DATA_WIDTH * i_line_index+ j * BYD_READ_ONCE_LEN + i;
							//printk("[%04d]=0x%02x,", index, byd_fps->data_buffer[index]);
							printk("[%04d]=0x%02x,", index, byd_img_buf[index]);
						}
						printk("\n");
						
					}
					#endif
					byd_fps_udelay(byd_rd_delay);
					//byd_fps_mdelay(2);
				}
			}
			
			if (i > 0) {
				//byd_fps->data_buffer[i * BYD_FPS_DATA_WIDTH * BYD_RD_LINE_NUM] = rx_temp;
			}

			
			DBG_TIME("%s: no dummy read fpd_data END\n", __func__);
			
			//byd_fps_udelay(byd_rd_delay);
			/*{
				unsigned int icnt=0;
				int index=0;
				//printk("byd LINE 0:\n");
				for(icnt=1;icnt<byd_rd_length;icnt++) {
					index = i * (BYD_FPS_DATA_WIDTH + BYD_FPS_SPI_DUMMY_NUM) * BYD_RD_LINE_NUM + icnt;
					printk("byd[%04d]=0x%02x, ", index, byd_fps->data_buffer[index]);
					if((icnt%12) == 0) {
						printk("\n");
					}
					if((icnt%96)==0) {
						printk("byd LINE %d:\n", i*BYD_RD_LINE_NUM+icnt/96);
					}
				}
			}*/
			#endif
		}
		
		#ifndef BYD_FPS_SPI_NO_DELAY
		memcpy(byd_fps->data_buffer,byd_img_buf, BYD_FPS_IMAGE_BUFFER_SIZE);
	  	byd_free(byd_img_buf);
	  	
		#endif
	#endif	//END BYD_RD_LINE_NUM


	DBG_TIME("%s: read fpd_data END\n", __func__);
	DBG_TIME("%s:BYD_FPS_IMAGE_SIZE=%d,BYD_FPS_IMAGE_BUFFER_SIZE=%d\n", __func__, BYD_FPS_IMAGE_SIZE, BYD_FPS_IMAGE_BUFFER_SIZE);

	return BYD_FPS_IMAGE_SIZE;
}
// *******************************************************************************
// * Function    :  byd_fps_task_capture
// * Description :  Processing task of capture.
// * In          :  *byd_fps, finger_wait
// * Return      :  0 -- success, -1 -- failed
// * 返回值：
// * 0			正常执行：检测到手指、完成分块检测、扫描得到1幅图像数据.
// * -1			spi通信错误.
// * -4			手指等待过程被打断(取消).(-EINTR)
// * -3			分块检测出错.
// * -2			SPI read DATAERROR.
// * -110		未能检测到中断事件而超时.(-ETIMEDOUT)
// * -512		等待事件的过程被信号中断.(-ERESTARTSYS)
// *******************************************************************************
//int byd_fps_task_capture(struct byd_fps_data *byd_fps, unsigned char proc, unsigned char scan_cmd)
int byd_fps_task_capture(struct byd_fps_data *byd_fps, unsigned char scan_cmd)
{
	int ret=0;
	int spi_ret=0;
	//unsigned int wait_time;
	unsigned char intr_state;
	//unsigned char fng_state;
	
	DBG_TIME("\n%s: capture: 0x%x! \n",__func__, scan_cmd);
	//byd_fps_check_reg(byd_fps->spi);
	//Initiate work mode.

	if(scan_cmd == 0x0E) {
		ret = byd_fps_chip_fp_scan(byd_fps->spi, SCAN_OS_DATA|SCAN_POSIT_ORDER);	
	} else {
		ret = byd_fps_chip_fp_scan(byd_fps->spi, SCAN_AREA0_SNGL|SCAN_POSIT_ORDER);
	}
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_finger_subarea_detect_cfg(byd_fps->spi,1);//设置手指块数触发为4
	}
	
	//finger_print_image_scan
	ret = byd_fps_wait_event_interruptible_timeout(set_timeout_image_scan);

	if(BYD_FPS_CHECK_SUB_AREA) {
		byd_fps_get_finger_detect_value(byd_fps->spi);
		byd_fps_get_finger_threshold(byd_fps->spi);
	}
	
	intr_state = byd_fps_chip_intr_state(byd_fps->spi);
	DBG_TIME("%s: rest time=%d,intr_state=0x%x\n", __func__, ret, intr_state);
	//if((intr_state &0x41)==0x41) 
	if((intr_state & IRQ_DATA_READY) == IRQ_DATA_READY)
	{
		byd_fps_chip_intr_flag_clr(byd_fps->spi, 0);
		//ret = byd_fps_finger_image_capture(byd_fps->spi, set_timeout_image_scan);
		/*if(byd_fps_chip_flag == 0x12)
			ret = byd_fps_fp_get_image_data(byd_fps, BYD_FPS_IMAGE_HEIGHT12, BYD_FPS_IMAGE_WIDTH12);
		if(byd_fps_chip_flag == 0x11)
			ret = byd_fps_fp_get_image_data(byd_fps, BYD_FPS_IMAGE_HEIGHT11, BYD_FPS_IMAGE_WIDTH11);*/
		
		if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
			ret = byd_fps_fp_get_image_data(byd_fps, BYD_FPS_IMAGE_HEIGHT, BYD_FPS_IMAGE_WIDTH);
		}
		if(byd_fps_chip_flag == 0x22){
			//byd_fps_check_reg(byd_fps->spi);
			ret = byd_fps_fp_get_image_data22(byd_fps,BYD_FPS_IMAGE_HEIGHT,BYD_FPS_IMAGE_HEIGHT);
		}
	}else{//乱出中断，但中断状态不满足的情况
		ret = -10;
		memset(byd_fps->data_buffer, 0xff, BYD_FPS_IMAGE_BUFFER_SIZE);
	}
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		spi_ret = byd_fps_finger_subarea_detect_cfg(byd_fps->spi,0);//将手指块数触发设为2
	}

	return ret;
}

#ifdef BYD_FPS_SYSFS

// *******************************************************************************
// * Function    :  byd_fps_show_image
// * Description :  read data from driver by app in user space.
// * In          :  dev, attr, buf
// * Return      :  -int(有无采集到图像0：没有采集到图像)
// *******************************************************************************
unsigned char byd_fps_fpd_file_1[]="/sdcard/byd_fps/byd_fps_data_1.bin";
unsigned char byd_fps_fpd_file_2[]="/sdcard/byd_fps/byd_fps_data_2.bin";
unsigned char byd_fps_fpd_file_3[]="/sdcard/byd_fps/byd_fps_data_3.bin";
int byd_fps_show_image(char *buf,struct byd_fps_data *byd_fps)
{
 	int ret = 0;
 	int i=0;
	DBG_TIME("%s:byd_app_get_image=%d,read_first=%d,trans_num=%d\n", __func__, byd_app_get_image, read_first, trans_num);
 	if(byd_app_get_image == 1) {	//正常读取数据
		/*if(byd_fps_chip_flag == 0x12)*/{
			if((read_first>0) && (read_first<trans_num+1)){ // keep transfer until read_first==0
				if (read_first == trans_num) { // first time transfer
				//byd_fps_finger_subarea_detect_cfg(byd_fps, 1);
				byd_fps_mutex_lock(1,1);
				ret = byd_fps_read_image(0x04);	//1011, read full image into data_buffer
				//ret = byd_fps_read_image(0x0c);	//0100,
				byd_fps_mutex_lock(2,1);
				//byd_fps_finger_subarea_detect_cfg(byd_fps, 0);
				DBG_TIME("%s:read image 1st with picture ret=%d:\n", __func__,ret);
				if(ret == 0){
					buf[0] = 0;	//capture no image
					return ret;
				}

			#ifdef BYD_FPS_DECODE_IMAGE
				
				#ifdef BYD_FPS_ALG_IN_KERNEL
					byd_fps_adjustimage(&byd_fps->data_buffer[1],(unsigned short *)&byd_fps->data_buf_decode[1]);
					
					for(i=0;i<(BYD_FPS_IMAGE_BUFFER_SIZE-1)/2;i++) {
						//unsigned short *
						((unsigned short *)&byd_fps->data_buf_decode[1])[i] = ((unsigned short *)&byd_fps->data_buf_decode[1])[i]<<4;
					}
				#endif
				#ifdef BYD_FPS_ALG_IN_TZ
					fp_qsee_adjustimage((unsigned short *)&byd_fps->data_buf_decode[1]);
				#endif
				
			#else
				{
					unsigned short int * p_buf=NULL;
					unsigned short int * p_os_buf=NULL;
					unsigned int i=0;

					byd_fps_encoding(&byd_fps->data_buffer[1], (unsigned short *)&byd_fps->data_buf_2[1]);
					byd_fps_encoding(&byd_fps->data_buf_os_otp[1], (unsigned short *)&byd_fps->data_buf_1[1]);
					
					DBG_TIME("%s:pre process data--add-move bit--\n", __func__);

					p_buf = (unsigned short int *)&byd_fps->data_buf_2[1];
					p_os_buf = (unsigned short int *)&byd_fps->data_buf_1[1];
					DBG_TIME("%s:pre process data--add-move bit--,[0]0x%x,[1]%x,[2]%x,[3]%x,[4]%x\n", __func__, p_buf[0], p_buf[1], p_buf[2], p_buf[3], p_buf[4]);
					for(i=0;i<(BYD_FPS_IMAGE_BUFFER_SIZE-1)/2;i++) {
						unsigned short temp;
						temp = 4095-p_os_buf[i]+p_buf[i];
						if(temp>4095)
							temp = 4095;
						else 
						{
							temp = 4095-p_os_buf[i]+p_buf[i];
						}
						p_os_buf[i] = (temp&0x0FFF)<<4;
					}
				}
			#endif

			} // END OF read_first == trans_num
			DBG_TIME("byd byd_fps_bf66xx_spi_qsee_show() read_first=%d\n", read_first);
			if(ret == BYD_FPS_IMAGE_SIZE){
				buf[0] = 1;	//capture image
			}
			#ifdef BYD_FPS_DECODE_IMAGE
			DBG_TIME("%s: read_first=%d,trans_num=%d,trans_rest=%d,buf[]=0x%x,%x,%x,%x,%x\n", __func__, read_first, trans_num, trans_rest, 
				byd_fps->data_buf_decode[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+0],byd_fps->data_buf_decode[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+1],
				byd_fps->data_buf_decode[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+2],byd_fps->data_buf_decode[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+3],
				byd_fps->data_buf_decode[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+4]);
			if (read_first == 1 && trans_rest > 0) { // the last transfer
				memcpy(&buf[1], &byd_fps->data_buf_decode[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE], trans_rest);	//test
				ret = trans_rest + 1;
			} else { // keep transfer for every data chunck
				DBG_TIME("byd memcpy data_buf to buf\n");
				memcpy(&buf[1], &byd_fps->data_buf_decode[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE], BYD_FPS_BUF_SIZE);	//test
				ret = BYD_FPS_BUF_SIZE + 1;
			}
			#else
			FP_DBG("%s: read_first=%d,trans_num=%d,trans_rest=%d,buf[]=0x%x,%x,%x,%x,%x\n", __func__, read_first, trans_num, trans_rest, 
				byd_fps->data_buf_1[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+0],byd_fps->data_buf_1[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+1],
				byd_fps->data_buf_1[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+2],byd_fps->data_buf_1[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+3],
				byd_fps->data_buf_1[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE+4]);
			if (read_first == 1 && trans_rest > 0) {
				memcpy(&buf[1], &byd_fps->data_buf_1[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE], trans_rest);	//test
				ret = trans_rest + 1;
			} else {
				FP_DBG("byd memcpy data_buf to buf\n");
				memcpy(&buf[1], &byd_fps->data_buf_1[1+(trans_num - read_first)*BYD_FPS_BUF_SIZE], BYD_FPS_BUF_SIZE);	//test
				ret = BYD_FPS_BUF_SIZE+1;
			}
			#endif
			read_first--;
			} // END OF read_first>0 && read_first<trans_num+1
			if(read_first == 0){
				DBG_TIME("byd read all times:%d\n", trans_num);
				read_first = trans_num;
				byd_last_cmd = 5;
				if(byd_enrol_match_flag == 0) {
					byd_last_cmd = 0;
				}
				byd_show_flag = 2;	//上层认为第一次一定能读到图像数据，第二次读算法结果。
			}
		}
		return ret;
	} else {	//APP不读取数据 byd_app_get_image != 1
		
		// byd_fps_finger_subarea_detect_cfg(byd_fps, 1);
		byd_fps_mutex_lock(1,1);
		
		if(byd_app_get_image == 2) {
			ret = byd_fps_read_image(0x0c);//0100,
			FP_DBG("%s:suspend_flag=%d\n", __func__, byd_fps_get_suspend_flag());
		} else {
			ret = byd_fps_read_image(0x04);
		}
			
		byd_fps_mutex_lock(2,1);
		//byd_fps_finger_subarea_detect_cfg(byd_fps, 0);
		DBG_TIME("%s:read image 2nd ret=%d:\n", __func__,ret);
		if(ret == 0) {
			buf[0] = 0;	//capture no image
			//byd_last_cmd = 0;
			byd_show_flag = 2;
			return ret;
		}
		if(ret == BYD_FPS_IMAGE_SIZE) {
			buf[0] = 1;	//capture image
			DBG_TIME("byd capt all image,no disp\n");
		}
		
		ret = 1;
		byd_last_cmd = 5;
		byd_show_flag = 2;
	}
	DBG_TIME("%s:END,ret = %d\n",__func__,ret);
	return ret;
}

#else

// *******************************************************************************
// * Function    :  byd_fps_show_image
// * Description :  read data from driver by app in user space.
// * In          :  dev, attr, buf
// * Return      :    1 captured image, but need not send up
//                   >1 image_size + 1, should send up
//                   <0 error code, -110 captured no image
// *******************************************************************************
int byd_fps_show_image(struct byd_fps_data *byd_fps)
{
 	int ret = 0;
 	int i=0;
	DBG_TIME("%s: entered, byd_app_get_image=%d\n", __func__, byd_app_get_image);
 	if(byd_app_get_image == 1) {	//正常读取数据
		//byd_fps_finger_subarea_detect_cfg(byd_fps, 1);
		byd_fps_mutex_lock(1,1);
		ret = byd_fps_read_image(0x04);	//1011, read full image into data_buffer
		//ret = byd_fps_read_image(0x0c);	//0100,
		byd_fps_mutex_lock(2,1);
		//byd_fps_finger_subarea_detect_cfg(byd_fps, 0);
		DBG_TIME("%s:read image 1st with picture ret=%d:\n", __func__,ret);

		#ifdef BYD_FPS_DECODE_IMAGE
				
			if (ret == 0) {
				byd_fps->data_buf_decode[0] = 0;	//capture no image
				return -110;
			}
			#ifdef BYD_FPS_ALG_IN_KERNEL
					DBG_TIME("%s:start byd_fps_adjustimage\n",__func__);
					byd_fps_adjustimage(&byd_fps->data_buffer[1], (unsigned short *)&byd_fps->data_buf_decode[1]);
					
					for (i = 0; i < (BYD_FPS_IMAGE_BUFFER_SIZE - 1) / 2; i++)
						((unsigned short *)&byd_fps->data_buf_decode[1])[i] = ((unsigned short *)&byd_fps->data_buf_decode[1])[i] << 4;
			#endif
			byd_fps->data_buf_decode[0] = 1;	//captured image
				
		#else
			if (ret == 0) {
				byd_fps->data_buf_1[0] = 0;	//capture no image
				return -110;
			}
			{
				unsigned short int * p_buf = NULL;
				unsigned short int * p_os_buf = NULL;
				unsigned int i=0;

				byd_fps_encoding(&byd_fps->data_buffer[1], (unsigned short *)&byd_fps->data_buf_2[1]);
				byd_fps_encoding(&byd_fps->data_buf_os_otp[1], (unsigned short *)&byd_fps->data_buf_1[1]);

				DBG_TIME("%s:pre process data--add-move bit--\n", __func__);

				p_buf = (unsigned short int *)&byd_fps->data_buf_2[1];
				p_os_buf = (unsigned short int *)&byd_fps->data_buf_1[1];
				DBG_TIME("%s:pre process data--add-move bit--,[0]0x%x,[1]%x,[2]%x,[3]%x,[4]%x\n", __func__, p_buf[0], p_buf[1], p_buf[2], p_buf[3], p_buf[4]);
				for (i = 0; i < (BYD_FPS_IMAGE_BUFFER_SIZE - 1) / 2; i++) {
					unsigned short temp;
					temp = 4095 - p_os_buf[i] + p_buf[i];
					if (temp > 4095)
						temp = 4095;
					p_os_buf[i] = (temp & 0x0FFF) << 4;
				}
				byd_fps->data_buf_1[0] = 1;
			}
		#endif
		/*if(byd_fps_chip_flag == 0x12)
			ret = BYD_FPS_IMAGE_BUFFER_SIZE12;
		if(byd_fps_chip_flag == 0x11)
			ret = BYD_FPS_IMAGE_BUFFER_SIZE11;*/
		ret = BYD_FPS_IMAGE_BUFFER_SIZE;
		
		byd_last_cmd = 5;
		if (byd_enrol_match_flag == 0)
			byd_last_cmd = 0;
		byd_show_flag = 2;	//上层认为第一次一定能读到图像数据，第二次读算法结果。
		return ret;
		
	} else {	//APP不读取数据 byd_app_get_image != 1
		
		// byd_fps_finger_subarea_detect_cfg(byd_fps, 1);
		byd_fps_mutex_lock(1,1);
		
		if (byd_app_get_image == 2) {
			ret = byd_fps_read_image(0x0c);//0100,
			FP_DBG("%s:suspend_flag=%d\n", __func__, byd_fps_get_suspend_flag());
		} else {
			ret = byd_fps_read_image(0x04);
		}
			
		byd_fps_mutex_lock(2,1);
		//byd_fps_finger_subarea_detect_cfg(byd_fps, 0);
		DBG_TIME("%s:read image 2nd ret=%d:\n", __func__,ret);
		if(ret == 0) {
			byd_fps->data_buffer[0] = 0;	//capture no image
			//byd_last_cmd = 0;
			byd_show_flag = 2;
			return -110;
		}
		/*if(byd_fps_chip_flag == 0x11){
			if(ret == BYD_FPS_IMAGE_SIZE11) {
				byd_fps->data_buffer[0] = 1;	//captured image
				DBG_TIME("byd capt all image,no disp\n");
			}
		}
		if(byd_fps_chip_flag == 0x12){
			if(ret == BYD_FPS_IMAGE_SIZE12) {
				byd_fps->data_buffer[0] = 1;	//captured image
				DBG_TIME("byd capt all image,no disp\n");
			}
		}*/
		if(ret == BYD_FPS_IMAGE_SIZE){
			byd_fps->data_buffer[0] = 1;	//captured image
			DBG_TIME("byd capt all image,no disp\n");
		}
		ret = 1; // image data need not to be transfered
		byd_last_cmd = 5;
		byd_show_flag = 2;
	}
	
	DBG_TIME("%s:END,ret = %d\n",__func__,ret);
	return ret;
}

#endif

//extern void byd_fps_wakeup_susp_worker(void);
extern void byd_fps_set_sensor_finger_detected(void);
extern int g_need_send_signal;
unsigned char byd_fps_int_test = 0;


void byd_fps_set_sensor_finger_detected(void) {
	int ret;
	byd_fps_mutex_lock(1,1);
	g_need_send_signal = 1;
	byd_fps_chip_idle(this_byd_fps->spi);		//sean test
////////////////////////////added by xzy .for esd
	byd_fps_mdelay(1);
	DBG_TIME("%s:cfg start 1\n",__func__);
	byd_fps_cfg_all_reg(this_byd_fps->spi);
	DBG_TIME("%s:cfg end\n",__func__);
	byd_fps_mdelay(1);
////////////////////////////////////////
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = byd_fps_spi_write(this_byd_fps->spi, REG_FG_REST_TI1_EN, 0);//休眠唤醒使用可变定时
		ret = byd_fps_spi_write(this_byd_fps->spi, REG_FINGER_STATE, 0);//清除finger_state寄存器
	}else if(byd_fps_chip_flag == 0x22){
		
		ret = byd_fps_spi_write(this_byd_fps->spi, REG_FINGER_STATE22, 0);//清除finger_state寄存器
	}
	//byd_fps_enable_irq_wake(this_byd_fps->spi);
	byd_fps_chip_detect(this_byd_fps->spi);
	
	byd_fps_mutex_lock(2,1);
}

// *******************************************************************************
// * Function    :  byd_fps_power_restart
// * Description :  control the power to IC,
// * In          :  spi
// * Return      :  0 -- success,
// *
// *******************************************************************************
#ifdef BYD_FPS_POWER_CTRL
int byd_fps_power_restart(struct spi_device *spi,unsigned char fng_flag)
{
	byd_power_control(20,40);//重新上电
	//上电重新配置
	byd_fps_mutex_lock(1,1);
	byd_fps_chip_idle(spi);
	byd_fps_mdelay(1);
	DBG_TIME("%s:cfg start 1\n",__func__);
	byd_fps_cfg_all_reg(spi);
	DBG_TIME("%s:cfg end\n",__func__);
	byd_fps_mdelay(1);
	if(fng_flag == 1)//检测抬起
	{
		byd_fps_finger_detect_pre_cfg(0,1);
	}
	else if(fng_flag == 2){//检测按下
		byd_fps_finger_detect_pre_cfg(1,1);
		
	}
	else{
		byd_fps_chip_detect(spi);
	}
	byd_fps_mutex_lock(2,1);
	return 0;
	
}
#endif
// *******************************************************************************
// * Function    :  byd_sys_dev_ioctl
// * Description :  resolve the command from APK
// * In          :  *buf
// * Return      :  0 -- success, 
// *******************************************************************************
extern unsigned short finger_throld;
int byd_fps_int_test_cfg_back(struct spi_device*spi)
{
	int ret;
	if(byd_fps_chip_flag == 0x12){
		ret = byd_fps_spi_write(spi, REG_INT_MODE_SET, VAL_INT_MODE_SET12);
		if(ret != 0){
			pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
			return -1;
		}
	}
	if(byd_fps_chip_flag == 0x11){
		ret = byd_fps_spi_write(spi, REG_INT_MODE_SET, VAL_INT_MODE_SET11);
		if(ret != 0){
			pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
			return -1;
		}
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_H, finger_throld>>8);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_ON_H spi_sync failed.\n");
		return -1;
	}
	ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_L, finger_throld);
	if(ret != 0){
		pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
		return -1;
	}
	return ret;
}

int byd_sys_dev_ioctl(const char *buf)
{
	int ret = 0;
	char cmd;
	int error=0;
	char arg_temp[5] = {0, 0, 0,0,0};
	struct byd_fps_data *byd_fps = this_byd_fps;
	struct spi_device *spi = byd_fps->spi;
    unsigned short value_tx_sel = 0 ;
	unsigned short value_slow_time = 0;
	unsigned short value_temp = 0;

	cmd = buf[0];
	arg_temp[0] = buf[1];
	arg_temp[1] = buf[2];
	arg_temp[2] = buf[3];
	arg_temp[3] = buf[4];
	arg_temp[4] = buf[5];
	
	FP_DBG("%s: IN ioctl start, cmd=%d, arg_temp[0]=%d, arg_temp[1]=%d\n", __func__, cmd, arg_temp[0], arg_temp[1]);
	work_cmd = 0;
	error = 0;

	byd_last_cmd = cmd; //record the last command.
	switch (cmd) {
		case BYD_FPS_KK_NOTHING: // 0
		{
		#ifdef BYD_FPS_GESTURE
			unsigned char icnt=50;
		
			//byd_fps_gesture_on_flag = 0;
			byd_fps_gesture_off();
		
			while((byd_fps_get_gest_state() != 0 )&& (icnt>0)){
				byd_fps_msleep(2);
				icnt--;
				DBG_TIME("%s,1 gest_sta=%d\n", __func__, byd_fps_get_gest_state());
			}
			if(byd_fps_get_gest_state() != 0) {
				error = -255;
				DBG_TIME("%s,2 gest_sta=%d\n", __func__, byd_fps_get_gest_state());
			} else 
		#endif
		#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 1;
		#endif
			{
				byd_chip_info=0x00;
				printk("%s:========byd_chip_info-1 = 0x%x=======\n",__func__,byd_chip_info&0xf8c0);
				byd_fps_mutex_lock(1,1);
				byd_chip_info = byd_fps_read_version(spi);
				byd_fps_mutex_lock(2,1);
				printk("%s:========byd_chip_info = 0x%x=======\n",__func__,byd_chip_info&0xf8c0);
				if(byd_fps_chip_flag == 0x12){
					if((byd_chip_info&0xf8c0)!= CONFIG_FPS12){
						pr_err("%s: Fatal Error: chip not bf66xx !!!,spi error\n", __func__);
						printk("%s------------ byd power down & up-----------\n",__func__);
						#ifdef BYD_FPS_POWER_CTRL
						byd_fps_power_restart(this_byd_fps->spi,0);
						#endif
						
					}
				}
				
				if(byd_fps_chip_flag == 0x11){
					if((byd_chip_info&0xf8c0)!= CONFIG_FPS11){
						pr_err("%s: Fatal Error: chip not bf66xx !!!,spi error\n", __func__);
						printk("%s------------ byd power down & up-----------\n",__func__);
						#ifdef BYD_FPS_POWER_CTRL
						byd_fps_power_restart(this_byd_fps->spi,0);
						#endif
						
					}
				}
				if(byd_fps_chip_flag == 0x22){
					if((byd_chip_info&0xf8c0)!= CONFIG_FPS22){
						pr_err("%s: Fatal Error: chip not bf66xx !!!,spi error\n", __func__);
						printk("%s------------ byd power down & up-----------\n",__func__);
						#ifdef BYD_FPS_POWER_CTRL
						byd_fps_power_restart(this_byd_fps->spi,0);
						#endif
						
					}
				}
				byd_fps_set_sensor_finger_detected();
				#ifdef BYD_FPS_UP_DOWN_ASYNC
				g_need_finger_down_scan_flag = 1;
				#endif
			}

		}
		break;

		case BYD_FPS_SET_CHIP_RESET:  // 1
#ifdef BYD_FPS_GESTURE
			//byd_fps_gesture_on_flag = 0;
			byd_fps_gesture_off();
#endif
		#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 0;
		#endif
		    byd_fps_mutex_lock(1,1);
			byd_fps_chip_fg_detect(spi);
		    byd_fps_mutex_lock(2,1);
		break;

		case BYD_FPS_SET_CHIP_SLEEP:  // 2
#ifdef BYD_FPS_GESTURE
			//byd_fps_gesture_on_flag = 0;
			byd_fps_gesture_off();
#endif
			byd_fps_mutex_lock(1,1);
			byd_fps_chip_sleep(spi);
			byd_fps_mutex_lock(2,1);
			byd_last_cmd = 0;
		break;

		case BYD_FPS_INTR_STATE:  // 3
#ifdef BYD_FPS_GESTURE
			//byd_fps_gesture_on_flag = 0;
			//byd_fps_gesture_off();
#endif
			//byd_fps_mutex_lock(1,1);
			//byd_fps_chip_idle(spi);
			//byd_fps_mutex_lock(2,1);
			
			byd_last_cmd = 3;
		break;
#ifdef BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED
	
		case BYD_FPS_SET_KEY_FUNC:  // 4
			/*switch(arg_temp[0]){
				case BYD_FPS_GEST_OFF:
					byd_fps_keyopen_flag = 0;
				break;
				case BYD_FPS_ONLY_KEY_ON:
					byd_fps_keyopen_flag = 1;
				break;
				case BYD_FPS_ONLY_FOCUS_ON:
				break;
				case BYD_FPS_GEST_ON:
				break;
					
			}*/
			byd_fps_keyopen_flag = arg_temp[0];
			//}
			byd_last_cmd = 0;
		break;
#endif
		case BYD_FPS_SET_SINGLE_CAPTURE:  // 5
			set_timeout_finger_down = 10 * arg_temp[0]; //arg_temp[0];  //arg_temp[0]/4;  //
			FP_DBG("Sean_Debug -%s: the set_timeout_finger_down is %lu now! \n", __func__, set_timeout_finger_down);
			#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 1;
			#endif
			byd_app_get_image = arg_temp[1];//上层是否要读取图像数据。0:不读取; 1:要读取; 2:不读取+不检测手指按下.
			byd_fps_finger_up_flag = 0;
			DBG_TIME("%s:byd_app_get_image=%d,wakelock=%d,suspend_flag=%d\n",__func__,byd_app_get_image,byd_fps_get_wakelock(),byd_fps_get_suspend_flag());
			
			switch(arg_temp[2]) {	//0：只采集图像,1：录入；2：比对; 3:高安全级别比对
				case 0:
					byd_enrol_match_flag = 0;
				break;
				
				case 1:
					byd_enrol_match_flag = 1;
				break;
				
				case 2:
					#ifdef  CONFIG_ALG_SAFE_PARA
					//CONFIG_ALG_SAFE_LEVEL 2
					#ifdef BYD_FPS_ALG_IN_KERNEL
					byd_fps_mutex_lock(1,2);
					DBG_TIME("%s:safe level 2\n", __func__);
					byd_fps_safe_level_get_n_set(CONFIG_ALG_SAFE_LEVEL);
					DBG_TIME("%s:safe level 2 end\n", __func__);
					byd_fps_mutex_lock(2,2);
					#endif
					#ifdef BYD_FPS_ALG_IN_TZ
					fp_qsee_safe_level(CONFIG_ALG_SAFE_LEVEL);
					#endif
					#endif
					
					byd_enrol_match_flag = 2;
				break;
				
				case 3:
					#ifdef  CONFIG_ALG_SAFE_PARA
					//CONFIG_ALG_SAFE_LEVEL_PAYMENT 1
					#ifdef BYD_FPS_ALG_IN_KERNEL
					byd_fps_mutex_lock(1,2);
					DBG_TIME("%s:safe level 1 start\n", __func__);
					byd_fps_safe_level_get_n_set(CONFIG_ALG_SAFE_LEVEL_PAYMENT);
					DBG_TIME("%s:safe level 1 END\n", __func__);
					byd_fps_mutex_lock(2,2);
					#endif
					#ifdef BYD_FPS_ALG_IN_TZ
					fp_qsee_safe_level(CONFIG_ALG_SAFE_LEVEL_PAYMENT);
					#endif
					#endif
					
					byd_enrol_match_flag = 2;
				break;
				
				default:
				break;
			}
			byd_fp_id = arg_temp[3];
			byd_fp_id = (byd_fp_id<<8)|arg_temp[4];
			byd_show_flag = 1;
            
			#ifdef BYD_FPS_SYSFS
				read_first = trans_num;
			#endif 

#ifdef BYD_FPS_GESTURE
			byd_fps_gesture_off();
#endif
		break;

#ifdef BYD_FPS_GESTURE
		case BYD_FPS_SET_MULTI_CAPTURE: //6
			
			//byd_fps_start_detect_gest();
				
			byd_last_cmd = 0;
			error = 0;
		break;
		case BYD_FPS_SET_CHIP_DOWNLOAD_PARAM: //7
			//byd_fps_gesture_off();
			byd_last_cmd = 0;
			error = 0;
		break;
#endif

		case BYD_FPS_SET_WAIT_FINGER_UP:  // 8
			set_timeout_finger_up = 2 * arg_temp[0];
			#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 1;
			#endif
			FP_DBG("%s: ioctl-8 byd_fps_wait_finger_up_timeout set_timeout_finger_up: %ld!\n", __func__, set_timeout_finger_up);
			if(byd_fps_finger_up_flag == 1)
			{
				byd_fps_finger_up_flag = 0;
				error = 0;
				break;
			}
			if(finger_intr_ctrl == 2) {
				#ifdef BYD_FPS_FG_DETECT_DELAY
				if(byd_fps_fg_det_state == 1) {
					#ifdef BYD_FPS_TEST_TIME_LOG
					DBG_TIME("%s: byd finger up in 3ms!\n", __func__);
					#endif
					byd_fps_fg_det_state = 0;
					error = 0;
					break;
				}
				#endif
				
				#ifdef BYD_FPS_TIMER
				#ifdef BYD_TIMER_FNG_UP
				error = byd_fps_upif();
				if(error == 1){
					error = 0;
					break;
				}
				#endif
				#endif
				
				#ifdef BYD_FINGER_INTR_CNT

				if(finger_intr_num == 1) {
					finger_intr_num = 0;
					FP_DBG("%s: ---------break--------\n", __func__);
					error = 0;
					break;
				}
				#endif
				
				error = -1;
				finger_intr_ctrl = 1;	//没检测到抬起,转入下面进行等待抬起
				DBG_TIME("%s: finger_intr_ctrl=%d, goto autowait\n", __func__, finger_intr_ctrl);
			}
			if((finger_intr_ctrl != 2)||(byd_fps_finger_up_flag!=1)) {
				DBG_TIME("%s: finger_intr_ctrl=%d\n", __func__, finger_intr_ctrl);
				work_cmd = 1;
				{
					byd_fps_mutex_lock(1,1);
                    
					error = byd_fps_auto_wait_finger_up_down_timeout(byd_fps, 0, set_timeout_finger_up);
					byd_fps_mutex_lock(2,1);
					FP_DBG("%s: byd_fps_wait_finger_up_timeout return %d!\n", __func__, error);
					
				}
				work_cmd = 0;
			}
			#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 0;
			#endif
			byd_last_cmd = 0;
			FP_DBG("%s: byd_fps_wait_finger_up_timeout return- %d!\n", __func__, error);

		break;

		case BYD_FPS_SET_ABORT_WAIT: // 9
			FP_DBG("%s: Sean_Debug - the cmd is %d now, work_cmd=%d! \n", __func__, cmd, work_cmd);

#ifdef	BYD_FPS_ALG_IN_TZ
#ifdef	STOP_EVERY_TIME
			if(qsee_run == 1){
				ret = fp_qsee_stop();
				qsee_run = 0;
				FP_DBG("%s: byd call fp_ qsee_stop()\n", __func__);
			}
#endif
#endif

			byd_fps_stop_thread(work_cmd);

		#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 0;
		#endif
			g_need_send_signal = 0;
			byd_fps_mutex_lock(1,1);
			byd_fps_chip_detect(spi);
			byd_fps_mutex_lock(2,1);
			FP_DBG("%s: Sean_Debug - the IDLE_MODE cmd is return! cmd = 0x%x, !\n", __func__, cmd);  //sean 20141125 15:21
			
			error = 0; //
			work_cmd = 0; //work _cmd 0 -- ACTIVATE_IDLE_MODE ;

			FP_DBG("%s: Abort - mutex_unlock\n", __func__);
			byd_last_cmd = 0;
		break;

		case BYD_FPS_SET_WAIT_FINGER_DOWN: // 12
			DBG_TIME("%s:byd_fps_alg_ret[0]=%d\n",__func__,byd_fps_alg_ret[0]);
			if(byd_fps_alg_ret[0]!=1){
				byd_fps_mutex_lock(1,1);
				
				byd_fps_cfg_all_reg(spi);
				DBG_TIME("%s:cfg end\n",__func__);
				byd_fps_mdelay(1);
				
				byd_fps_chip_detect(spi);
				byd_fps_mutex_lock(2,1);
			}
		#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 1;
		#endif
			DBG_TIME("%s:change,suspend_flag=%d\n", __func__, byd_fps_get_suspend_flag());
			if(byd_fps_get_suspend_flag() == 0) {
				DBG_TIME("%s:change,suspend_flag=%d\n", __func__, byd_fps_get_suspend_flag());
				byd_fps_set_suspend_flag(8);	//change  to 2 (in suspend)
				DBG_TIME("%s:change 7->8,suspend_flag=%d\n", __func__, byd_fps_get_suspend_flag());
				byd_fps_finger_up_async_flag = 1;
				byd_fps_mutex_lock(1,1);
				byd_fps_finger_detect_pre_cfg(0,1);//抬起+快速
				byd_fps_mutex_lock(2,1);
				//byd_fps_enable_irq_wake(this_byd_fps->spi);
				//byd_fps_finger_up_async_flag = 1;
				#ifdef BYD_FPS_UP_DOWN_ASYNC
				g_need_finger_up_scan_flag = 1;
				#endif
			} else {
				error = -255;

			}
			byd_last_cmd = 0;
			
		break;

		case BYD_FPS_SET_DELET_TEMPLET: // 13
			byd_fp_id = (arg_temp[0]<<8)|arg_temp[1];
			ret = 0;
			DBG_TIME("%s: byd Sean_Debug - fp_qsee_delete() now! byd_fp_id=%d\n", __func__, byd_fp_id);
#ifdef 	BYD_FPS_ALG_IN_TZ
			if(qsee_run == 0){
				if(!fp_qsee_start()){
					return -101;
				}else{
					FP_DBG("%s: byd call fp_ qsee_start()\n", __func__);
					qsee_run = 1;
				}
			}
			error = fp_qsee_delete(byd_fp_id);
#ifdef STOP_EVERY_TIME
			if(qsee_run == 1){
				ret = fp_qsee_stop();
				FP_DBG("%s: byd call fp_ qsee_stop()\n", __func__);
				qsee_run = 0;
			}
#endif
#endif

#ifdef 	BYD_FPS_ALG_IN_KERNEL
			error = 0;
			FP_DBG("%s: byd call ALG_IN_KERNEL()--fpsDelTem\n", __func__);
			byd_fps_mutex_lock(1,2);
			error = byd_fps_remove_template(byd_fp_id);
			byd_fps_mutex_lock(2,2);
#endif

			byd_last_cmd = 0;
		break;

		case BYD_FPS_IOCTL_IS_FINGER_EXIST: //14
			byd_fp_id = (arg_temp[0]<<8)|arg_temp[1];
			g_need_send_signal = 0;
		#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 1;
		#endif
#ifdef 	BYD_FPS_ALG_IN_TZ
			ret = 0;
			if(qsee_run == 0){
				if(!fp_qsee_start()){
					return -101;
				}else{
					FP_DBG("%s: byd call fp_ qsee_start()\n", __func__);
					qsee_run = 1;
				}
			}
			error = fp_qsee_check_FPID(byd_fp_id);
#ifdef STOP_EVERY_TIME
			if(qsee_run == 1){
				ret = fp_qsee_stop();
				FP_DBG("%s: byd call fp_ qsee_stop()\n", __func__);
				qsee_run = 0;
			}
#endif
#endif

#ifdef 	BYD_FPS_ALG_IN_KERNEL
			error = 0;
			FP_DBG("%s: byd call ALG_IN_KERNEL()--IsFingerExist\n", __func__);
			byd_fps_mutex_lock(1,2);
			error = has_fingerprint_enrolled(byd_fp_id);
			byd_fps_mutex_lock(2,2);
#endif
		#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 0;
		#endif
			byd_last_cmd = 0;
		break;

		case BYD_FPS_IOCTL_IS_DATABASE_EXIST: //15
			error = 15;
		break;

		case BYD_FPS_SET_SAFE_LEVEL: //16
			#ifdef BYD_FPS_ALG_IN_KERNEL
			byd_fps_mutex_lock(1,2);
			byd_fps_safe_level_get_n_set(arg_temp[0]);
			byd_fps_mutex_lock(2,2);
			#endif
			#ifdef BYD_FPS_ALG_IN_TZ
			fp_qsee_safe_level(arg_temp[0]);
			#endif
			byd_last_cmd = 16;
			error = 0;
		break;
		
		case 19:          // wasted code is here.
		#if defined(BYD_FPS_REPORT_KEY)||defined(BYD_FPS_FOCUS)
			byd_fps_keydown_flag = 1;//进入apk测试界面，关闭按键功能，退出测试界面发送09命令打开
		#endif
			switch(arg_temp[0]) {
				case 1:
				byd_fps_mutex_lock(1,1);
				if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
					ret = byd_fps_spi_write(spi, 0x5F, 0xA5);
					if(ret != 0){
						pr_err("byd REG_ CHIP_MODE spi_sync failed.ret=%d\n", ret);
						return ret;
					}
					ret = byd_fps_spi_read(spi, 0x5F);
					if(ret != 0xA5){
						pr_err("byd REG_ CHIP_MODE spi_sync failed.ret=%d\n", ret);
						return ret;
					}
				}else if(byd_fps_chip_flag == 0x22){
					ret = byd_fps_spi_write(spi, 0x48, 0xA5);
					if(ret != 0){
						pr_err("byd REG_ CHIP_MODE spi_sync failed.ret=%d\n", ret);
						return ret;
					}
					ret = byd_fps_spi_read(spi, 0x48);
					if(ret != 0xA5){
						pr_err("byd REG_ CHIP_MODE spi_sync failed.ret=%d\n", ret);
						return ret;
					}
					
				}
				printk("%s:cmd19,read(5f)=0x%x\n", __func__, ret);
				byd_fps_chip_reset(spi);
				printk("%s:cmd19,reset end\n", __func__);
				
				byd_fps_cfg_all_reg(spi);
				
				byd_fps_mdelay(1);
				
				byd_fps_chip_fg_detect(spi);				
				
				byd_fps_mutex_lock(2,1);
				break;
				
				case 2:    //FG_REST_TI2 手指慢检测时间间隔(ms), 步进为0.25ms, t=(FG_REST_TI2 + 1)*250us
					

					value_temp = ((arg_temp[1]<<8) | arg_temp[2]); //上层传下的是250us对应的数字 250us--1,500us--2...
	
					//FG_REST_TI2 手指慢检测时间间隔(ms), 步进为0.25ms, t=(FG_REST_TI2 + 1)*250us
					value_slow_time = (value_temp - 1);
					printk("%s:cmd19,value_slow_time=0x%x,0x%x,0x%x\n", __func__,value_slow_time,arg_temp[1],arg_temp[2]);
					
					byd_fps_fae_detect[6] = (unsigned char)(value_slow_time>>8);
					byd_fps_fae_detect[7] = (unsigned char)value_slow_time;
					printk("%s:FG_REST_TI2=0x%x,fae_detect[6,7]=0x%x,0x%x\n", __func__, value_temp, byd_fps_fae_detect[6],byd_fps_fae_detect[7]);
					byd_fps_mutex_lock(1,1);
					if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
						ret = byd_fps_spi_write(spi, REG_FG_REST_TI2_H, byd_fps_fae_detect[6]);
						if(ret != 0){
							pr_err("byd REG_FG_REST_TI2_H spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_FG_REST_TI2_L, byd_fps_fae_detect[7]);
						if(ret != 0){
							pr_err("byd REG_FG_REST_TI2_L spi_sync failed.\n");
							return -1;
						}
					}
					
					byd_fps_mutex_lock(2,1);
				break;
				
				case 3:   //发射频率
					
					value_tx_sel = arg_temp[1];
					value_tx_sel = (value_tx_sel<<8)|arg_temp[2];
				    
					byd_fps_fae_image[0] = value_tx_sel;
					
					printk("%s:cmd19,value_tx_sel=0x%x,byd_fps_fae_image[0]=0x%x\n", __func__, value_tx_sel,byd_fps_fae_image[0]);
					
					if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
						byd_fps_mutex_lock(1,1);
						ret = byd_fps_spi_write(spi, REG_TX_CLK_SEL, value_tx_sel);
						if(ret != 0){
							pr_err("byd REG_TX_CLK_SEL spi_sync failed.\n");
							return -1;
						}
						//write row VAL_READ_ROW_CFG0 .
						ret = byd_fps_spi_write(spi, REG_READ_ROW_CFG0, arg_temp[3]);
						if(ret != 0){
							pr_err("byd REG_READ_ROW_CFG0 spi_sync failed.\n");
							return -1;
						}
					
						//write row VAL_READ_ROW_CFG1
						ret = byd_fps_spi_write(spi, REG_READ_ROW_CFG1, arg_temp[4]);
						if(ret != 0){
							pr_err("byd REG_READ_ROW_CFG1 spi_sync failed.\n");
							return -1;
						}
						byd_fps_mutex_lock(2,1);
						
						//NO_FINGER_NUM 无手指超时,  建议10S.
						value_temp = byd_fps_no_fng_num(byd_fps_fae_detect_para[4]);
						printk("%s:byd_fps_fae_detect_para[4]=%d,value_temp=%d(%x)\n", __func__, byd_fps_fae_detect_para[4],value_temp,value_temp);
						byd_fps_fae_detect[8] = (unsigned char)(value_temp>>8);
						byd_fps_fae_detect[9] = (unsigned char)value_temp;
						printk("%s:cmd19,byd_fps_fae_detect[8]=0x%x,byd_fps_fae_detect[9]=0x%x\n", __func__, byd_fps_fae_detect[8],byd_fps_fae_detect[9]);
						//no Finger time-out.
						byd_fps_mutex_lock(1,1);
						ret = byd_fps_spi_write(spi, REG_NO_FINGER_NUM_H, byd_fps_fae_detect[8]);
						if(ret != 0){
							pr_err("byd REG_NO_FINGER_NUM_H spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_NO_FINGER_NUM_L, byd_fps_fae_detect[9]);
						if(ret != 0){
							pr_err("byd REG_NO_FINGER_NUM_L spi_sync failed.\n");
							return -1;
						}		
						byd_fps_mutex_lock(2,1);
					}
					if(byd_fps_chip_flag == 0x22){
						ret = byd_fps_spi_write(spi, REG_TX_CLK_SEL22, value_tx_sel);
						if(ret != 0){
							pr_err("byd REG_TX_CLK_SEL spi_sync failed.\n");
							return -1;
						}
						//NO_FINGER_NUM 无手指超时,  建议10S.
						value_temp = byd_fps_no_fng_num22(byd_fps_fae_detect_para22[4]);
						printk("%s:byd_fps_fae_detect_para22[4]=%d,value_temp=%d(%x)\n", __func__, byd_fps_fae_detect_para22[4],value_temp,value_temp);
						byd_fps_fae_detect22[8] = (unsigned char)(value_temp>>8);
						byd_fps_fae_detect22[9] = (unsigned char)value_temp;
						printk("%s:cmd19,byd_fps_fae_detect22[8]=0x%x,byd_fps_fae_detect22[9]=0x%x\n", __func__, byd_fps_fae_detect[228],byd_fps_fae_detect22[9]);
						//no Finger time-out.
						byd_fps_mutex_lock(1,1);
						ret = byd_fps_spi_write(spi, REG_NO_FINGER_NUM22, byd_fps_fae_detect22[9]);
						if(ret != 0){
							pr_err("byd REG_NO_FINGER_NUM_H22 spi_sync failed.\n");
							return -1;
						}		
						byd_fps_mutex_lock(2,1);
						
					}
					
					
				break;
				
				case 4:  //读阈值
					 
				break;
				
				case 5:  //写阈值 按下 抬起
				    
					
					//fpd_th_on 按下域值
					
					byd_fps_mutex_lock(1,1);
					if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
						
						value_temp = (arg_temp[1]<<8) | arg_temp[2];
					
						value_temp = (65535-value_temp);
						printk("%s:value_temp=%d\n", __func__, value_temp);
						byd_fps_fae_detect[0] = (unsigned char)(value_temp>>8);
						byd_fps_fae_detect[1] = (unsigned char)value_temp;
						printk("%s:cmd19,byd_fps_fae_detect[0]=0x%x,byd_fps_fae_detect[1]=0x%x\n", __func__, byd_fps_fae_detect[0],byd_fps_fae_detect[1]);
						ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_H, byd_fps_fae_detect[0]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_ON_H spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_L, byd_fps_fae_detect[1]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
							return -1;
						}
					}else if(byd_fps_chip_flag == 0x22){
						
							//fpd_th_on 按下域值
						value_temp = arg_temp[2]*otp_fpd_th*16/100;//根据按下系数算按下阈值(arg_temp[1]<<8) | arg_temp[2];
						byd_fps_fae_detect_para22[0] = arg_temp[2];
					
						value_temp = (65535 - value_temp);
						printk("%s:value_temp=%d\n", __func__, value_temp);
						byd_fps_fae_detect22[0] = (unsigned char)(value_temp>>8);
						byd_fps_fae_detect22[1] = (unsigned char)value_temp;
						printk("%s:cmd19,byd_fps_fae_detect22[0]=0x%x,byd_fps_fae_detect[1]=0x%x\n", __func__, byd_fps_fae_detect22[0],byd_fps_fae_detect22[1]);
					
						ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_H22, byd_fps_fae_detect22[0]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_ON_H spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_L22, byd_fps_fae_detect22[1]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_H2_22, byd_fps_fae_detect22[0]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_ON_H spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_L2_22, byd_fps_fae_detect22[1]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
							return -1;
						}
						
					}
					byd_fps_mutex_lock(2,1);
					
					byd_fps_mutex_lock(1,1);
					if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
						
						//fpd_th_off 抬起域值
						value_temp = (arg_temp[3]<<8) | arg_temp[4];
					
						value_temp = (65535-value_temp);
						printk("%s:value_temp=%d\n", __func__, value_temp);
						byd_fps_fae_detect[2] = (unsigned char)(value_temp>>8);
						byd_fps_fae_detect[3] = (unsigned char)value_temp;
						printk("%s:cmd19,byd_fps_fae_detect[2]=0x%x,byd_fps_fae_detect[3]=0x%x\n", __func__, byd_fps_fae_detect[2],byd_fps_fae_detect[3]);
						ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_H, byd_fps_fae_detect[2]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_OFF_H spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_L, byd_fps_fae_detect[3]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_OFF_L spi_sync failed.\n");
							return -1;
						}
					}else if(byd_fps_chip_flag == 0x22){
						
						value_temp = arg_temp[4]*otp_fpd_th*16/100;//根据抬起系数算抬起阈值(arg_temp[3]<<8) | arg_temp[4];
						byd_fps_fae_detect_para22[1] = arg_temp[4];
					
						value_temp = (65535-value_temp);
						printk("%s:value_temp=%d\n", __func__, value_temp);
						byd_fps_fae_detect22[2] = (unsigned char)(value_temp>>8);
						byd_fps_fae_detect22[3] = (unsigned char)value_temp;
						printk("%s:cmd19,byd_fps_fae_detect22[2]=0x%x,byd_fps_fae_detect22[3]=0x%x\n", __func__, byd_fps_fae_detect22[2],byd_fps_fae_detect22[3]);
						ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_H22, byd_fps_fae_detect22[2]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_OFF_H spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_L22, byd_fps_fae_detect22[3]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_OFF_L spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_H2_22, byd_fps_fae_detect22[2]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_OFF_H spi_sync failed.\n");
							return -1;
						}
						ret = byd_fps_spi_write(spi, REG_FPD_TH_OFF_L2_22, byd_fps_fae_detect22[3]);
						if(ret != 0){
							pr_err("byd REG_FPD_TH_OFF_L spi_sync failed.\n");
							return -1;
						}
						
					}
					byd_fps_mutex_lock(2,1);

					
					
				break;
				
				case 8:
				if(arg_temp[1]==1){//spi测试
					//byd_fps_test_flag = 1;
					
					
				}else if(arg_temp[1]==2){//int测试
					//byd_fps_test_flag = 2;
					byd_fps_mutex_lock(1,1);
					finger_throld = read_throld_on(spi);
					ret = byd_fps_spi_write(spi, REG_INT_MODE_SET, 0X03);
					if(ret != 0){
						pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
						return -1;
					}
					ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_H, 0xff);
					if(ret != 0){
						pr_err("byd REG_FPD_TH_ON_H spi_sync failed.\n");
						return -1;
					}
					ret = byd_fps_spi_write(spi, REG_FPD_TH_ON_L, 0xff);
					if(ret != 0){
						pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
						return -1;
					}
					byd_fps_mutex_lock(2,1);
					byd_fps_int_test = 1;
				
				}else if(arg_temp[1]==3){//坏点测试
					//byd_fps_test_flag = 3;
					
				}
				byd_fps_ageing_test_flag = arg_temp[1];
				break;
			}
			
			byd_last_cmd = 19;
			ReceiVeCmdType = arg_temp[0];
		
		break;

		default:
			DBG_TIME("%s: Sean_Debug - the cmd is 0x%x now! \n", __func__, cmd);
		break;
	}

	FP_DBG("%s: End,return=%d\n\n", __func__, error);
	return error;
}

// *******************************************************************************
// * Function    :  byd_fps_trans_data_tz
// * Description :  send version num to tz
// * In          :  void
// * Return      :  0 -- success,
// *******************************************************************************
#ifdef BYD_FPS_ALG_IN_TZ
int	byd_fps_trans_data_tz(unsigned char * data, unsigned int data_len, unsigned int page_index,\
						unsigned int page_total, unsigned int flag)
{
	return(fp_qsee_transfer(data, data_len, byd_chip_info, page_total, flag));
}
#endif


//#ifdef FPS12
static void byd_fps_saveadjust_image(unsigned char *data_image)
//#else
//void byd_fps_saveadjust_image(unsigned short *data_image)
//#endif
{
#ifdef BYD_FPS_ALG_IN_TZ
	int ret;

	if(qsee_run == 0){
		if(!fp_qsee_start()){
			return;
		}else{
			qsee_run = 1;
			FP_DBG("%s: byd call fp_ qsee_start()\n", __func__);
		}
	}
	ret = fp_qsee_saveadjust();	
	#ifdef STOP_EVERY_TIME
		if(qsee_run == 1){
			ret = fp_qsee_stop();
			qsee_run = 0;
			FP_DBG("%s: byd call fp_ qsee_stop()\n", __func__);
		}
	#endif
#endif
#ifdef BYD_FPS_ALG_IN_KERNEL
	byd_fps_saveadjust(data_image);
#endif
}



// *******************************************************************************
// * Function    :  byd_fps_alg_version_init
// * Description :  initialize the memory&files of ALGORITHM
// * In          :  void
// * Return      :  0 -- success,
// * -1 -- request memory fail, -2 -- open files fail, -3--read/write file fail
// *******************************************************************************
#ifdef BYD_FPS_ALG_IN_KERNEL
int byd_fps_alg_version_init(unsigned short chip_key,unsigned char chip_type)
{
	int ret;
	ret = 0;
	//FP_DBG("%s:byd_fps_set_version, byd_chip_info = 0x%x\n",__func__, byd_chip_info);
	ret = byd_fps_set_version(chip_key, byd_fps_alg_ret,chip_type);
	//ret = byd_fps_set_version(0x525, byd_fps_alg_ret,chip_type);
	//ret = byd_fps_set_version(0x00, byd_fps_alg_ret);
	//ret: -1, 申请内存失败
	//ret: -2, 加载文件失败
	//ret: -3, 文件读写失败
	if(ret < 0){
		byd_fps_alg_ret[0] = 255;
		byd_fps_alg_ret[1] = 250-ret;
		FP_DBG("%s: alg failed,ret = %d\n",__func__, ret);
		return -1;
	}
	FP_DBG("%s:byd_fps_set_version succeed,ret = %d\n",__func__,ret);

	return 0;
}
#endif

 // *******************************************************************************
// * Function    :  byd_fps_qual_judge
// * Description :  
// * In          :  void
// * Return      :  int
// *******************************************************************************/
int byd_fps_qual_judge( unsigned char enrol_match_flag )
{
	int ret=0;
	
	u8 scan_num = 0;
	u8 area_thre = 0;
	u8 fail_num = 0;
	u8 mid_num =0;
	u8 byd_fps_choose_flag = 0;
	int i;
	unsigned char byd_fps_qual[4];
	struct byd_fps_data *byd_fps = this_byd_fps;
	DBG_TIME("%s:IN,enrol_match_flag=%d\n",__func__,enrol_match_flag);
	if(enrol_match_flag == 1){//enrol
		//DBG_TIME("%s:IN,enrol\n",__func__);
		scan_num = 1;
		area_thre = 0;
		
	}else if(enrol_match_flag == 2){//match
		//DBG_TIME("%s:IN,match\n",__func__);
		scan_num = 1;
		area_thre = 0;
	}
	byd_fps_sec_scan_flag = 0;//0:不需要进行第二次扫描 ，1：需要进行第二次扫描
	for(i=0;i<scan_num;i++){
		byd_fps_qual[0] = 254;//算法需要初始化为254，解密的依据
		byd_fps_qual[1] = 0;
		byd_fps_qual[2] = 0;//面积
		byd_fps_qual[3] = 0;//脊谷差
		byd_fps_mutex_lock(1,2);
		if(enrol_match_flag == 1){
		#ifdef BYD_FPS_ALG_IN_KERNEL
			//#ifdef FPS12
			byd_fps_enrolqualityjudge((unsigned char *)byd_fps_image_data,byd_fps_qual);
			//#else
			//byd_fps_enrolqualityjudge(byd_fps_image_data,byd_fps_qual);
			//#endif
		#endif
		#ifdef BYD_FPS_ALG_IN_TZ
			fp_qsee_qual_enrol(byd_fps_qual);
		#endif
		}
		else if(enrol_match_flag == 2){
			#ifdef BYD_FPS_TEST_TIME_LOG
				DBG_TIME("%s:matchqualityjudge START\n",__func__);
			#endif
			#ifdef BYD_FPS_ALG_IN_KERNEL
				//#ifdef FPS12
				byd_fps_matchqualityjudge((unsigned char *)byd_fps_image_data,byd_fps_qual);
				//#else
				//byd_fps_matchqualityjudge(byd_fps_image_data,byd_fps_qual);
				//#endif
			#endif
			#ifdef BYD_FPS_ALG_IN_TZ
				fp_qsee_qual_match(byd_fps_qual);
			#endif
			#ifdef BYD_FPS_TEST_TIME_LOG
				DBG_TIME("%s:matchqualityjudge END \n",__func__);
			#endif
		}
		byd_fps_mutex_lock(2,2);
		DBG_TIME("%s:byd_fps_qual[0]= %d,[1]= %d,[2]=%d,[3]=%d\n",__func__,byd_fps_qual[0],byd_fps_qual[1],byd_fps_qual[2],byd_fps_qual[3]);
		if(byd_fps_qual[0] == 1){
			byd_alg[i] = byd_fps_qual[2];
			byd_alg[i+3] = byd_fps_qual[3];
			if((byd_fps_qual[2] >= area_thre)&&(byd_fps_qual[3] >= 10)){
				byd_fps_sec_scan_flag = 0;
				FP_DBG("%s:HIGH image,goto enrol or match\n",__func__);
				break;
			}else{
				byd_fps_sec_scan_flag = 1;
				//DBG_TIME("%s:MID image ,IMAGE BAK,second scan\n",__func__);
				if(mid_num ==0){
					DBG_TIME("%s:save the first data buf\n",__func__);
					memcpy(byd_fps->data_buf_1,byd_fps->data_buffer,BYD_FPS_IMAGE_BUFFER_SIZE12 * BYD_FPS_MAX_FRAME);
					mid_num++;
				}else if(mid_num==1){
					DBG_TIME("%s:save the second data buf\n",__func__);
					//memcpy(byd_fps->data_buf_2,byd_fps->data_buffer,BYD_FPS_IMAGE_BUFFER_SIZE * BYD_FPS_MAX_FRAME);
					mid_num++;
				}
				if(i != (scan_num -1)){
					int error=0;
					byd_fps_mutex_lock(1,1);
					ret = byd_fps_read_image(0x0C);
					byd_fps_mutex_lock(2,1);
					#ifdef BYD_FPS_INPUT_WAKEUP
					
					error = byd_fps_qual_suspend();
					if(error == 1){
						return -1;//表示立刻返回，有了优先级更高的操作
					}
					#endif
				}
			}
		}else{
			byd_alg[i] = byd_fps_qual[2];
			byd_alg[i+3] = byd_fps_qual[3];
			
			if(i != (scan_num -1)){
				int error=0;
				//DBG_TIME("%s:LOW qual , scan\n",__func__);
				byd_fps_mutex_lock(1,1);
				ret = byd_fps_read_image(0x0C);
				byd_fps_mutex_lock(2,1);
				#ifdef BYD_FPS_INPUT_WAKEUP
				
				error = byd_fps_qual_suspend();
				if(error == 1){
					return -1;
				}
				#endif
			}
			fail_num++;
		}
	}
	if(fail_num == scan_num){//如果每次都达不到底线，则直接返回给上层
		FP_DBG("%s:no best,fail\n",__func__);
		byd_fps_fail_reason = byd_fps_qual[1];
		byd_fps_choose_flag = 1;
		return 1;
	}
	if(byd_fps_sec_scan_flag == 0){//说明最后一帧已经达到标准，直接进行下一步录入或者比对
		FP_DBG("%s:end,the last is the best\n",__func__);
		//byd_alg[5] = 1;
		return 0;
	}
	
	else if(byd_fps_sec_scan_flag == 1){//进行相应的计算，得到最优的一帧
		if(mid_num==1){//第一帧采集MID，第二帧采集FAIL，快速抬起
			byd_fps_image_data = (unsigned short*)&byd_fps->data_buf_1[1];
		}else{
			if((byd_alg[3]>10)&&(byd_alg[4]>10)){//脊骨差都>10.选面积大的
				if(byd_alg[0]>byd_alg[1]){
					byd_fps_image_data = (unsigned short*)&byd_fps->data_buf_1[1];
					byd_alg[5] = 1;
				}else{
					byd_alg[5] = 2;
				}
			}else{
				if((byd_alg[3]*2+byd_alg[0])>(byd_alg[4]*2+byd_alg[1])){//脊骨分别>10,<10.选面积+脊骨差大的
					byd_fps_image_data = (unsigned short*)&byd_fps->data_buf_1[1];
					byd_alg[5] = 1;
				}else{
					byd_alg[5] = 2;//
				}
			}	
		}
		return 0;
	}
	#ifdef BYD_FPS_ALG_IN_TZ
	if(byd_fps_choose_flag == 1){
		ret = fp_qsee_transfer((unsigned char *)byd_fps_image_data, BYD_FPS_IMAGE_SIZE, 0, 0x1234, 0x00010000);//#define TZ_ENROL_SENC  0x00010000
		DBG_TIME("%s: call 2nd fp_qsee_transfer() end\n", __func__);
	}
	#endif
	return 0;
}

// *******************************************************************************
// * Function    :  byd_fps_multi_enrol
// * Description :  call ALGORITHM, merge & write template.
// * In          :  void
// * Return      :  0 -- success,
// * -1 -- request memory fail, -2 -- open files fail, -3--read/write file fail
// * note: byd_enrol_match_flag ==1 (enrol)
// *******************************************************************************
int byd_fps_multi_enrol(unsigned char byd_fps_show[12])
{
	int ret;
	unsigned char qual_ret;
	unsigned char i;

	for(i=0;i<6;i++){
		byd_alg[i] = 0; 	
	}
	for(i=0;i<12;i++){
		byd_fps_show[i] = 0; 	
	}
	#ifdef BYD_FPS_TEST_TIME_LOG
	DBG_TIME("%s:start qual judge && enrol\n",__func__);
	#endif
	qual_ret = byd_fps_qual_judge(1);//质量判断
	if(qual_ret == 0){
		for(i=0;i<6;i++){
			byd_fps_alg_ret[i] = 0;
		}
		byd_fps_mutex_lock(1,2);
#ifdef BYD_FPS_ALG_IN_KERNEL
		//#ifdef FPS12
		ret = byd_fps_enroll((unsigned char *)byd_fps_image_data, byd_fp_id, byd_fps_alg_ret);
		//#else
		//ret = byd_fps_enroll(byd_fps_image_data, byd_fp_id, byd_fps_alg_ret);
		//#endif
#endif
#ifdef BYD_FPS_ALG_IN_TZ
		if(qsee_run == 0){
			if(!fp_qsee_start()){
				return -101;
			}else{
				qsee_run = 1;
				FP_DBG("%s: byd call fp_ qsee_start()\n", __func__);
			}
		}
		ret = fp_qsee_save(byd_fp_id);
		#ifdef STOP_EVERY_TIME
		if(qsee_run == 1){
			ret = fp_qsee_stop();
			qsee_run = 0;
			FP_DBG("%s: byd call fp_ qsee_stop()\n", __func__);
		}
		#endif	
#endif
		
		#ifdef BYD_FPS_TEST_TIME_LOG
		DBG_TIME("byd _qsee_show2()-enrol fingprint ret=%d.result[0]=%d,[1]=%d,[2]=%d,[3]=%d,[4]=%d,[5]=%d\n", ret, byd_fps_alg_ret[0],byd_fps_alg_ret[1],byd_fps_alg_ret[2],byd_fps_alg_ret[3],byd_fps_alg_ret[4],byd_fps_alg_ret[5]);
		#endif
		byd_fps_mutex_lock(2,2);
		if(ret < 0){
			byd_fps_alg_ret[0] = 255;
			byd_fps_alg_ret[1] = 250-ret;
			FP_DBG("%s:byd_fps_enroll malloc space failed,ret = %d\n",__func__,ret);
		}
		#ifdef BYD_FPS_POWER_CTRL
		if(byd_fps_alg_ret[0] == 255){
			//if(byd_fps_alg_ret[1] == 106)//图像异常
			{
				byd_fps_power_restart(this_byd_fps->spi,0);
			}
		}
		#endif
	} else {
		if(qual_ret == 1) {
			byd_fps_alg_ret[0] = 255;
			byd_fps_alg_ret[1] = byd_fps_fail_reason;
		}
	}
	
	for(i=0;i<6;i++){
		byd_fps_show[i] = byd_fps_alg_ret[i];
		byd_fps_show[i+6] = byd_alg[i];
	}
	//memmove(buf, byd_fps_show, 12);
	byd_last_cmd = 0;
	ret = 12;
	return ret;
}

// *******************************************************************************
// * Function    :  byd_fps_multi_match
// * Description :  call ALGORITHM, merge & write template.
// * In          :  void
// * Return      :  0 -- success,
// * -1 -- request memory fail, -2 -- open files fail, -3--read/write file fail
// * note: byd_enrol_match_flag ==2 (match)
// *******************************************************************************
int byd_fps_multi_match(unsigned char byd_fps_show[12])
{
	int ret;
	//int image_ret = 0;
	unsigned char qual_ret;
	//unsigned char byd_fps_enrol_qual[4];
	unsigned char i;
	unsigned char user_id;
	//unsigned char byd_fps_show[12];
	for(i=0;i<6;i++) {
		byd_alg[i] = 0;
	}
	for(i=0;i<12;i++) {
		byd_fps_show[i] = 0;
	}
	qual_ret = byd_fps_qual_judge(2);//质量判断
	if(qual_ret == 0) {
		#ifdef BYD_FPS_TEST_TIME_LOG
		DBG_TIME("%s: fpsMatch start\n", __func__);
		#endif
		for(i=0;i<6;i++){
			byd_fps_alg_ret[i] = 0;
		}
	#ifdef BYD_FPS_ALG_IN_KERNEL
		byd_fps_mutex_lock(1,2);
		//#ifdef	FPS12
		DBG_TIME("%s: fpsMatch start 12,byd_fps_image_data=%p\n", __func__, byd_fps_image_data);
		ret = byd_fps_match((unsigned char *)byd_fps_image_data, 0, byd_fps_alg_ret);
		DBG_TIME("%s: fpsMatch end 12\n", __func__);
		//#else
		//ret = byd_fps_match(byd_fps_image_data, 0, byd_fps_alg_ret);
		//#endif
		byd_fps_mutex_lock(2,2);
	
		DBG_TIME("byd _qsee_show()-match fingprint.result[0]=%d,[1]=%d,[2]=%d,[3]=%d,[4]=%d,[5]=%d\n", byd_fps_alg_ret[0],byd_fps_alg_ret[1],byd_fps_alg_ret[2],byd_fps_alg_ret[3],byd_fps_alg_ret[4],byd_fps_alg_ret[5]);
		//ret: -1, 申请内存失败
		//ret: -2, 加载文件失败
		//ret: -3, 文件读写失败
		if(ret < 0){
			byd_fps_alg_ret[0] = 255;
			byd_fps_alg_ret[1] = 250-ret;
			FP_DBG("%s:byd_fps_match() malloc space failed,ret = %d\n",__func__,ret);
		}
		#ifdef BYD_FPS_TEST_TIME_LOG
		DBG_TIME("%s: fpsMatch end\n", __func__);
		#endif
	#endif
	#ifdef BYD_FPS_ALG_IN_TZ
	if(qsee_run == 0){
		if(!fp_qsee_start()){
			return -101;
		}else{
			qsee_run = 1;
			FP_DBG("%s: byd call fp_ qsee_start()\n", __func__);
			}
	}
		FP_DBG("%s: byd Sean_Debug - fp_qsee_match() now! \n", __func__);
		ret = fp_qsee_match();
		//ret: -1, 申请内存失败
		//ret: -2, 加载文件失败
		//ret: -3, 文件读写失败
		if(ret < 0){
			byd_fps_alg_ret[0] = 255;
			byd_fps_alg_ret[1] = 250-ret;
			FP_DBG("%s:byd_fps fp_qsee_match() malloc space failed,ret = %d\n",__func__,ret);
		}
		FP_DBG("%s: fpsMatch end\n", __func__);
		FP_DBG("byd _qsee_show()-match fingprint.result[0]=%d,[1]=%d,[2]=%d,[3]=%d,[4]=%d,[5]=%d\n",\
				byd_fps_alg_ret[0],byd_fps_alg_ret[1],byd_fps_alg_ret[2],byd_fps_alg_ret[3],byd_fps_alg_ret[4],byd_fps_alg_ret[5]);
		#ifdef STOP_EVERY_TIME
			if(qsee_run == 1){
			ret = fp_qsee_stop();
			qsee_run = 0;
			FP_DBG("%s: byd call fp_ qsee_stop()\n", __func__);
			}
		#endif
		
	#endif
		//比对成功后启动内核线程，进行后拼接及模板回写
		
		if(byd_fps_alg_ret[0] == 1){

			//Match_Failed_Counter = 0;  //匹配成功 清零 即连续失败次数更新

			byd_fps_match_id = 0;
			byd_fps_match_id = (byd_fps_alg_ret[2]<<24)|(byd_fps_alg_ret[3]<<16)|(byd_fps_alg_ret[4]<<8)|(byd_fps_alg_ret[5]);
			user_id = byd_fps_match_id/100;
			#ifdef BYD_FPS_TEST_TIME_LOG
			DBG_TIME("%s: byd_fps_match_id = %d,user_id = %d , byd_fps_alg_ret[0] = %d\n", __func__, byd_fps_match_id,user_id, byd_fps_alg_ret[0]);
			#endif
			byd_fps_start_template_merging();
			DBG_TIME("%s: byd Sean_Debug - start thread_func end, go next! \n", __func__);
		} else {
			Match_Failed_Counter++;
			if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
				if (Match_Failed_Counter > OS_MATCH_FAILED_MAX)
					Os_Scan_Expired = 1;
			}else if(byd_fps_chip_flag == 0x22){
				if (Match_Failed_Counter > OS_MATCH_FAILED_MAX22)
					Os_Scan_Expired = 1;
			}
			//比对失败
			#ifdef BYD_FPS_POWER_CTRL
			if(byd_fps_alg_ret[0] == 255){
				//if(byd_fps_alg_ret[1] == 106)//图像异常
				{
					byd_fps_power_restart(this_byd_fps->spi,0);
				}
			}
			#endif
		}
	} else {
		if(qual_ret == 1) {
			byd_fps_alg_ret[0] = 255;
			byd_fps_alg_ret[1] = byd_fps_fail_reason;
		}
	}
	
	for(i=0;i<6;i++){
		FP_DBG("%s:byd_alg[%d] = %d\n",__func__,i,byd_alg[i]);//前三个是面积，后三个是脊谷差	
	}
	for(i=0;i<6;i++){
		byd_fps_show[i] = byd_fps_alg_ret[i];
		byd_fps_show[i+6] = byd_alg[i];
	}
	//memmove(buf, byd_fps_show, 12);
	//byd_last_cmd = 0;
	ret = 12;
	return ret;
}

// *******************************************************************************
// * Function    :  byd_fps_alg_back_proc
// * Description :  call ALGORITHM, merge & write template.
// * In          :  void
// * Return      :  0 -- success,
// * -1 -- request memory fail, -2 -- open files fail, -3--read/write file fail
// *******************************************************************************
int byd_fps_alg_back_proc(void)
{
#ifdef BYD_FPS_ALG_IN_KERNEL
	int ret = 0;
	unsigned char byd_fps_merge_ret[6];

	ret = byd_fps_merge_template(byd_fps_match_id,byd_fps_merge_ret);//比对成功后拼接
	//ret: -1, 申请内存失败
	//ret: -2, 加载文件失败
	//ret: -3, 文件读写失败
	if(ret < 0){
		byd_fps_merge_ret[0] = 255;
		byd_fps_merge_ret[1] = 250-ret;
		FP_DBG("%s: byd_fps_merge_template alg failed,ret = %d\n",__func__,ret);
		return ret;
	}

	FP_DBG("%s:byd_fps_merge_template result = %d\n",__func__, byd_fps_merge_ret[0]);
	/*if(byd_fps_merge_ret[0] == 1)*/ {	//判断拼接是否成功，成功则回写模板
		ret = 0;
		ret = byd_fps_write_template();//内存信息写回模板文件
		//ret: -1, 申请内存失败
		//ret: -2, 加载文件失败
		//ret: -3, 文件读写失败
		if(ret < 0){
			byd_fps_merge_ret[0] = 255;
			byd_fps_merge_ret[1] = 250-ret;
			FP_DBG("%s: byd_fps_write_template alg failed,ret = %d\n",__func__,ret);
			return ret;
		}
	}
#endif

#ifdef BYD_FPS_ALG_IN_TZ
	fp_qsee_alg_back_proc(byd_fps_match_id);
#endif

	DBG_TIME("%s: merge Write finish  ,OUT!!!\n ", __func__);
	return 0;
}

unsigned int read_throld_on(struct spi_device *spi)
{
	unsigned int ret;
	unsigned int throld_h = 0;
	unsigned int throld_l = 0;
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		throld_h = byd_fps_spi_read(spi, REG_FPD_TH_ON_H);
		if(throld_h < 0){
			pr_err("byd REG_FPD_TH_ON_H spi_sync failed.\n");
			return -1;
		}
	
		throld_l = byd_fps_spi_read(spi, REG_FPD_TH_ON_L);
		if(throld_l < 0){
			pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
			return -1;
		}
	}else if(byd_fps_chip_flag == 0x22){
		throld_h = byd_fps_spi_read(spi, REG_FPD_TH_ON_H22);
		if(throld_h < 0){
			pr_err("byd REG_FPD_TH_ON_H spi_sync failed.\n");
			return -1;
		}
	
		throld_l = byd_fps_spi_read(spi, REG_FPD_TH_ON_L22);
		if(throld_l < 0){
			pr_err("byd REG_FPD_TH_ON_L spi_sync failed.\n");
			return -1;
		}
		
	}
	ret = (throld_h<<8)|throld_l;
	
	return ret;
}

unsigned int read_throld_off(struct spi_device *spi)
{
	unsigned int ret;
	unsigned int throld_h = 0;
	unsigned int throld_l = 0;
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		throld_h = byd_fps_spi_read(spi, REG_FPD_TH_OFF_H);
		if(throld_h < 0){
			pr_err("byd REG_FPD_TH_OFF_H spi_sync failed.\n");
			return -1;
		}
	
		throld_l = byd_fps_spi_read(spi, REG_FPD_TH_OFF_L);
		if(throld_l < 0){
			pr_err("byd REG_FPD_TH_OFF_L spi_sync failed.\n");
			return -1;
		}
	}else if(byd_fps_chip_flag == 0x22){
		throld_h = byd_fps_spi_read(spi, REG_FPD_TH_OFF_H22);
		if(throld_h < 0){
			pr_err("byd REG_FPD_TH_OFF_H spi_sync failed.\n");
			return -1;
		}
	
		throld_l = byd_fps_spi_read(spi, REG_FPD_TH_OFF_L22);
		if(throld_l < 0){
			pr_err("byd REG_FPD_TH_OFF_L spi_sync failed.\n");
			return -1;
		}
	}
	ret = (throld_h<<8)|throld_l;
	
	return ret;
}

void byd_fps_read_finger_detect_value(struct spi_device *spi,char *buf)
{
	//int ret;
	int value = 0;
	unsigned char i;
	unsigned char j = 0;
	unsigned int byd_fps_temp_value11[OS_AREA_BLOCK11]={0}; 
	unsigned int byd_fps_temp_value12[OS_AREA_BLOCK12]={0};
	unsigned int byd_fps_temp_value22[OS_AREA_BLOCK22]={0};
	
	if(byd_fps_chip_flag == 0x11){
		for(i=0; i<(OS_AREA_BLOCK11*2); i++){
			byd_fps_sub_value11[i] = 0;
		}
	}
	if(byd_fps_chip_flag == 0x12){
		for(i=0; i<(OS_AREA_BLOCK12*2); i++){
			byd_fps_sub_value12[i] = 0;
		}
	}
	if(byd_fps_chip_flag == 0x22){
		for(i=0; i<(OS_AREA_BLOCK22*2); i++){
			byd_fps_sub_value22[i] = 0;
		}
	}
	printk("%s:cmd19,byd_fps_chip_detect start\n", __func__);
	//byd_fps_chip_fg_detect(spi); //芯片模式设置
	
   // byd_fps_mdelay(10);	

	value = byd_fps_get_finger_detect_value(spi); //获取各区域阈值
	if(byd_fps_chip_flag == 0x11){
		for(i=0,j=0; i<OS_AREA_BLOCK11; i++){
			value = ((byd_fps_sub_value11[j]<<8)|byd_fps_sub_value11[j+1]);
			byd_fps_temp_value11[i] = 65535 - value;
			printk("%s:cmd19,byd_fps_temp_value11[%d] is=%d\n", __func__,i,byd_fps_temp_value11[i]);
			j+=2;
		}

		for(i=0,j=0; i<(OS_AREA_BLOCK11*2); i+=2){
			buf[i] = byd_fps_temp_value11[j]>>8;
			//printk("%s:cmd19,buf is=%d\n", __func__,buf[i]);
			buf[i+1] = byd_fps_temp_value11[j] & 0x00ff;
			j++;
		//printk("%s:cmd19,buf is=%d\n", __func__,buf[i+1]);
		}
	}
	if(byd_fps_chip_flag == 0x12){
		for(i=0,j=0; i<OS_AREA_BLOCK12; i++){
			value = ((byd_fps_sub_value12[j]<<8)|byd_fps_sub_value12[j+1]);
			byd_fps_temp_value12[i] = 65535 - value;
			printk("%s:cmd19,byd_fps_temp_value12[%d] is=%d\n", __func__,i,byd_fps_temp_value12[i]);
			j+=2;
		}

		for(i=0,j=0; i<(OS_AREA_BLOCK12*2); i+=2){
			buf[i] = byd_fps_temp_value12[j]>>8;
			//printk("%s:cmd19,buf is=%d\n", __func__,buf[i]);
			buf[i+1] = byd_fps_temp_value12[j] & 0x00ff;
			j++;
		//printk("%s:cmd19,buf is=%d\n", __func__,buf[i+1]);
		}
	}
	if(byd_fps_chip_flag == 0x22){
		for(i=0,j=0; i<OS_AREA_BLOCK22; i++){
			value = ((byd_fps_sub_value22[j]<<8)|byd_fps_sub_value22[j+1]);
			byd_fps_temp_value22[i] = 65535 - value;
			printk("%s:cmd19,byd_fps_temp_value12[%d] is=%d\n", __func__,i,byd_fps_temp_value12[i]);
			j+=2;
		}

		for(i=0,j=0; i<(OS_AREA_BLOCK22*2); i+=2){
			buf[i] = byd_fps_temp_value22[j]>>8;
			//printk("%s:cmd19,buf is=%d\n", __func__,buf[i]);
			buf[i+1] = byd_fps_temp_value22[j] & 0x00ff;
			j++;
		//printk("%s:cmd19,buf is=%d\n", __func__,buf[i+1]);
		}
	}
}

// *******************************************************************************
// * Module      :  byd_os
// * Description :  OS adjustment and pixel compensation for fingerprint image
// *******************************************************************************

/**
 * OS_Of_Finger_Detect - store OS data of 9 blocks of finger detect
 *
 * this is a class variable.
 */
static int OS_Of_Finger_Detect11[OS_AREA_BLOCK11];
static int OS_Of_Finger_Detect12[OS_AREA_BLOCK12];
static int OS_Of_Finger_Detect22[OS_AREA_BLOCK22];

/*
 * os_read_otp_data() - read OS or pixel compensation data from otp memory
 * @os_data:	Point to a buffer to store data.
 *		the size of buffer must be the size of the fingerprint image.
 *
 * OTP_OS_DATA_START and BYD_FPS_IMAGE_SIZE must be pre-defined.
 *
 * Return: status, 0 - success, -1 - failure
 */
static int os_read_otp_data(unsigned char *os_data)
{
	int ret = 0;
	//unsigned char *os_data=NULL;
	//os_data = byd_malloc(6912*2);
	//ret = byd_fps_otp_read(OTP_MODULE_ID,OTP_MODULE_BYTE/2,os_data);
	//DBG_TIME("%s:data[0]:0x%x,data[1]:0x%x,data[2]:0x%x,data[3]:0x%x,\n",__func__,os_data[0],os_data[1],os_data[2],os_data[3]);
	//#ifdef CONFIG_FPS12
	if(byd_fps_chip_flag == 0x12)
		ret = byd_fps_otp_read(OTP_OS_DATA_START12, BYD_FPS_IMAGE_SIZE12/2, os_data,0);
	//#else
	if(byd_fps_chip_flag == 0x11)
		ret = byd_fps_otp_read(OTP_OS_DATA_START11, BYD_FPS_IMAGE_SIZE11, os_data,0);
	//#endif
	
	FP_DBG("%s:data:0x%x,data[0]:0x%x,data[1]:0x%x,data[2]:0x%x,data[3]:0x%x,\n",__func__,os_data[0],os_data[1],os_data[2],os_data[3],os_data[4]);

	return ret;
}

/*
 * os_read_otp_bad_point() - read bad point data from otp memory
 * @bad_points:	Point to a buffer to store data.
 *		the size of buffer must be big enough for fingerprint image.
 *
 * OTP_BAD_POINT_* must be pre-defined.
 *
 * Return: status, 0 - success, -1 - failure
 */
//#ifdef CONFIG_FPS12
static int os_read_otp_bad_point(unsigned short int *bad_points)
{
	int ret = 0;
	unsigned short int bp_num=0;
	unsigned short int bp_chip_num, bp_potting_num, bp_module_num;
	unsigned short int bad_point_num=0;
	unsigned char bad_point_chip_num, bad_point_potting_num, bad_point_module_num;
	
	printk("%s:byd_fps_chip_flag = 0x%x\n",__func__,byd_fps_chip_flag);
	//unsigned short int *bad_points;
	if(byd_fps_chip_flag == 0x12){
		
		ret = byd_fps_otp_read(OTP_BAD_POINT_CHIP_START12, 1, (unsigned char *)&bp_chip_num,0);  //read number of bad point of "chip bad point"
		if(bp_chip_num == 0xffff){
			DBG_TIME("%s:bp_chip_num not write\n",__func__);
			//return -1;
		}else {
			bp_num += bp_chip_num;
		}

		ret = byd_fps_otp_read(OTP_BAD_POINT_POTTING_START12, 1, (unsigned char *)&bp_potting_num,0); //read number of bad point of "potting bad point"
		if(bp_potting_num == 0xffff){
			DBG_TIME("%s:bp_potting_num not write\n",__func__);
			//return -1;
		}else {
			bp_num += bp_potting_num;
		}
		ret = byd_fps_otp_read(OTP_BAD_POINT_MODULE_START12, 1, (unsigned char *)&bp_module_num,0);  //read number of bad point of "module bad point"
		if(bp_module_num == 0xffff){
			DBG_TIME("%s:bp_module_num not write\n",__func__);
			//return -1;
		}else {
			bp_num += bp_module_num;
		}
		//bp_num = bp_chip_num + bp_potting_num + bp_module_num;
		DBG_TIME("%s:bp_chip_num:%d,bp_potting_num:%d,bp_module_num:%d,bp_num:%d\n",__func__,bp_chip_num,bp_potting_num,bp_module_num,bp_num);
		//bad_points = byd_malloc((bp_num+1)*2);

		bad_points[0] = bp_num;
		//record bad point information in 2 bytes.
		if((bp_chip_num != 0)&&(bp_chip_num != 0xffff)){
			DBG_TIME("%s:read chip huaidian zuobiao\n",__func__);
			ret = byd_fps_otp_read(OTP_BAD_POINT_CHIP12, bp_chip_num, (unsigned char *)&(bad_points[1]),1);
			DBG_TIME("%s:bad_points[0]:0x%x,bad_points[1]:0x%x,bad_points[2]:0x%x,bad_points[3]:0x%x,bad_points[4]:0x%x\n",__func__,bad_points[0],bad_points[1],bad_points[2],bad_points[3],bad_points[4]);
		}
		if((bp_potting_num != 0)&&(bp_potting_num != 0xffff)){
			DBG_TIME("%s:read potting huaidian zuobiao\n",__func__);
			ret = byd_fps_otp_read(OTP_BAD_POINT_POTTING12, bp_potting_num, (unsigned char *)&(bad_points[1+bp_chip_num]),1);
		}
		if((bp_module_num != 0)&&(bp_module_num != 0xffff)){
			DBG_TIME("%s:read module huaidian zuobiao\n",__func__);
			ret = byd_fps_otp_read(OTP_BAD_POINT_MODULE12, bp_module_num, (unsigned char *)&(bad_points[1+bp_chip_num+bp_potting_num]),1);
		}
	}
	if(byd_fps_chip_flag == 0x11){
		
		DBG_TIME("%s:byd_fps11 bad point read start\n",__func__);
		ret = byd_fps_otp_read(OTP_BAD_POINT_CHIP_START11, 1, &bad_point_chip_num,0);  //read number of bad point of "chip bad point"
		if(bad_point_chip_num == 0xff) {
			printk("%s:bad_point_chip_num not write\n",__func__);	
		}else {
			bad_point_num += bad_point_chip_num;
		}
		ret = byd_fps_otp_read(OTP_BAD_POINT_POTTING_START11, 1, &bad_point_potting_num,0); //read number of bad point of "potting bad point"
		if(bad_point_potting_num == 0xff){
			DBG_TIME("%s:bad_point_potting_num not write\n",__func__);
		}else {
			bad_point_num += bad_point_potting_num;
		}
	
		ret = byd_fps_otp_read(OTP_BAD_POINT_MODULE_START11, 1, &bad_point_module_num,0);  //read number of bad point of "module bad point"
		if(bad_point_module_num == 0xff){
			DBG_TIME("%s:bad_point_module_num not write\n",__func__);	
		}else {
			bad_point_num += bad_point_module_num;
		}
		FP_DBG("%s:bad_point_chip_num:%d,bad_point_potting_num:%d,bad_point_module_num:%d,bad_point_num:%d\n",__func__,bad_point_chip_num,bad_point_potting_num,bad_point_module_num,bad_point_num);

		bad_points[0] = bad_point_num;
		//record bad point information in 2 bytes.
		if((bad_point_chip_num != 0)&&(bad_point_chip_num != 0xff)){
			printk("%s:read chip huaidian zuobiao\n",__func__);
			ret = byd_fps_otp_read(OTP_BAD_POINT_CHIP11, bad_point_chip_num*2, (unsigned char *)&(bad_points[1]),1);
			FP_DBG("%s:bad_points[0]:0x%x,bad_points[1]:0x%x,bad_points[2]:0x%x,bad_points[3]:0x%x,bad_points[4]:0x%x\n",__func__,bad_points[0],bad_points[1],bad_points[2],bad_points[3],bad_points[4]);
		}
	
		if((bad_point_potting_num != 0)&&(bad_point_potting_num != 0xff)){
			FP_DBG("%s:read potting huaidian zuobiao\n",__func__);
			ret = byd_fps_otp_read(OTP_BAD_POINT_POTTING11, bad_point_potting_num*2, (unsigned char *)&(bad_points[1+bad_point_chip_num]),1);
		}
	
		if((bad_point_module_num != 0)&&(bad_point_module_num != 0xff)){
			FP_DBG("%s:read module huaidian zuobiao\n",__func__);
			ret = byd_fps_otp_read(OTP_BAD_POINT_MODULE11, bad_point_module_num*2, (unsigned char *)&(bad_points[1+bad_point_chip_num+bad_point_potting_num]),1);
		}	
	}
	if(byd_fps_chip_flag == 0x22){
		
		DBG_TIME("%s:byd_fps22 bad point read start\n",__func__);
		ret = byd_fps_otp_read(OTP_BAD_POINT_CHIP_START22, 1, &bad_point_chip_num,0);  //read number of bad point of "chip bad point"
		if(bad_point_chip_num == 0xff) {
			printk("%s:bad_point_chip_num not write\n",__func__);	
		}else {
			bad_point_num += bad_point_chip_num;
		}
		ret = byd_fps_otp_read(OTP_BAD_POINT_POTTING_START22, 1, &bad_point_potting_num,0); //read number of bad point of "potting bad point"
		if(bad_point_potting_num == 0xff){
			DBG_TIME("%s:bad_point_potting_num not write\n",__func__);
		}else {
			bad_point_num += bad_point_potting_num;
		}
	
		ret = byd_fps_otp_read(OTP_BAD_POINT_MODULE_START22, 1, &bad_point_module_num,0);  //read number of bad point of "module bad point"
		if(bad_point_module_num == 0xff){
			DBG_TIME("%s:bad_point_module_num not write\n",__func__);	
		}else {
			bad_point_num += bad_point_module_num;
		}
		FP_DBG("%s:bad_point_chip_num:%d,bad_point_potting_num:%d,bad_point_module_num:%d,bad_point_num:%d\n",__func__,bad_point_chip_num,bad_point_potting_num,bad_point_module_num,bad_point_num);

		bad_points[0] = bad_point_num;
		//record bad point information in 2 bytes.
		if((bad_point_chip_num != 0)&&(bad_point_chip_num != 0xff)){
			printk("%s:read chip huaidian zuobiao\n",__func__);
			ret = byd_fps_otp_read(OTP_BAD_POINT_CHIP22, bad_point_chip_num*2, (unsigned char *)&(bad_points[1]),1);
			FP_DBG("%s:bad_points[0]:0x%x,bad_points[1]:0x%x,bad_points[2]:0x%x,bad_points[3]:0x%x,bad_points[4]:0x%x\n",__func__,bad_points[0],bad_points[1],bad_points[2],bad_points[3],bad_points[4]);
		}
	
		if((bad_point_potting_num != 0)&&(bad_point_potting_num != 0xff)){
			FP_DBG("%s:read potting huaidian zuobiao\n",__func__);
			ret = byd_fps_otp_read(OTP_BAD_POINT_POTTING22, bad_point_potting_num*2, (unsigned char *)&(bad_points[1+bad_point_chip_num]),1);
		}
	
		if((bad_point_module_num != 0)&&(bad_point_module_num != 0xff)){
			FP_DBG("%s:read module huaidian zuobiao\n",__func__);
			ret = byd_fps_otp_read(OTP_BAD_POINT_MODULE22, bad_point_module_num*2, (unsigned char *)&(bad_points[1+bad_point_chip_num+bad_point_potting_num]),1);
		}	
	}
	return ret;
}

/*
 * os_is_otp_or_scan() - read the OS flag from OTP
 *
 * the OS flag is used to determin OTP data is for OS or pixel compensation
 *
 * Return: status for success or failure.
 */
static int os_is_otp_or_scan(void)
{
  	int ret;
	unsigned char mark;
	if(byd_fps_chip_flag == 0x11){
	ret = byd_fps_otp_read(OTP_MARK11, 1, &mark,0);
		if (ret == -1) {
			pr_err("%s failed, ret = %d\n", __func__, ret);
			return ret;
		}
	}
	if(byd_fps_chip_flag == 0x12){
		ret = byd_fps_otp_read(OTP_MARK12, 1, &mark,0);
		if (ret == -1) {
			pr_err("%s failed, ret = %d\n", __func__, ret);
			return ret;
		}
	}
	ret = mark;

	return ret;
}

#ifdef CONFIG_FPS11
// *******************************************************************************
// * Function    :  byd_fps_get_osdata_from_block
// * Description :  get osdata from five block
// * In          :  byd_fps
// *******************************************************************************
int byd_fps_get_osdata_from_block(unsigned short int * image_data,unsigned int area_block)
{
	int image_block_data = 0;
    unsigned int i = 0;
	unsigned int j = 0;
	unsigned int sum_block_value = 0;
	unsigned int block_count = 0 ;
    char area_column[16]={10,13,16,19,22,25,28,31,34,37,40,43,46,49,52,55};       //子区域所在的列号
	
	for( i = area_block; i < (area_block+8); i++) {
		for( j = 0; j < 16; j++) {
			sum_block_value += image_data[i*BYD_FPS_IMAGE_WIDTH11 + area_column[j]];  //A区域
			block_count++;
		}
	}
	
	image_block_data = sum_block_value/block_count;
	
	return image_block_data;
}
#endif

// *******************************************************************************
// * Function    :  byd_fps_get_osdata_from_block
// * Description :  get osdata from 3 blocks.
// * In          :  byd_fps
// *******************************************************************************
int byd_fps_get_osdata_from_block_72X64(unsigned short int * image_data,unsigned int area_block_row)
{
	int image_block_data = 0;
    unsigned int i = 0;
	unsigned int j = 0;
	unsigned int sum_block_value = 0;
	unsigned int block_count = 0 ;
    char area_column[8]={3, 11, 19, 27, 35, 43, 51, 59};       //子区域所在的起始列号
	FP_DBG("%s:area_block_row=%d\n", __func__, area_block_row);
	
	for( i = area_block_row; i < (area_block_row+8); i++) {
		for( j = 0; j < 8; j++) {
			sum_block_value += image_data[(i*BYD_FPS_IMAGE_WIDTH + area_column[j])];  //A区域
			block_count++;
		}
	}
	
	image_block_data = sum_block_value/block_count;
	FP_DBG("%s:area_block_row=%d,block_count=%d\n", __func__, area_block_row, block_count);
	return image_block_data;
}

int byd_fps_get_osdata_from_block_56X64(unsigned short int * image_data,unsigned int area_block_row)
{
	int image_block_data = 0;
    unsigned int i = 0;
	unsigned int j = 0;
	unsigned int sum_block_value = 0;
	unsigned int block_count = 0 ;
    char area_column[8]={3, 11, 19, 27, 35, 43, 51, 59};       //子区域所在的起始列号
	FP_DBG("%s:area_block_row=%d\n", __func__, area_block_row);
	for( i = area_block_row; i < (area_block_row+4); i++) {
		for( j = 0; j < 8; j++) {
			sum_block_value += image_data[(i*BYD_FPS_IMAGE_WIDTH + area_column[j])];  //A区域和C区只有4行.
			block_count++;
		}
	}
	
	image_block_data = sum_block_value/block_count;
	FP_DBG("%s:area_block_row=%d,block_count=%d\n", __func__, area_block_row, block_count);
	return image_block_data;
}
/*
逐点比较：
*/
static int os_data_compare(unsigned short int * image_data,unsigned short int * os_image_data )
{
	int i,temp,suma=0,sumb=0;
	for (i = 0;i <BYD_FPS_IMAGE_WIDTH*BYD_FPS_IMAGE_HEIGHT;i++)
	{

		temp = image_data[i] - os_image_data[i];
		/*if ((image_data[i]>=2500) || (image_data[i]==0) )//|| (temp >500) || (temp <-500) )
		{
			printk("byd **********************************************:%x,%x\n",image_data[i],temp);
			return -10;		}*/
		if (temp >=0)//符合更新的数据差
		{
			suma = suma +temp;	
		}
		else  //不符合更新的数据差
		{
			sumb = sumb +temp;		
		}
	}
	
	
	printk("byd suma=%d\n",(suma*100)/(BYD_FPS_IMAGE_WIDTH*BYD_FPS_IMAGE_HEIGHT));
	printk("byd sumb=%d\n",(sumb*100)/(BYD_FPS_IMAGE_WIDTH*BYD_FPS_IMAGE_HEIGHT));
	
	return (suma + sumb)*100/(BYD_FPS_IMAGE_WIDTH*BYD_FPS_IMAGE_HEIGHT);//4行
	
}
/*
 * os_fng_updown_compare() - judge sensor is touched or not
 * @d_imagedata:	Data of finger image
 *
 * judge finger is up or down pass through nine block of os data diff
 *
 * Return:	0 -- up, 1 -- down
 */
 
 static const unsigned char byd_VAL_FG_ROW_START_72X64[3] = {4, 32, 60};
static const unsigned char byd_VAL_FG_ROW_START_56X64[3] = {0, 32, 52};
static int byd_os_fng_updown_compare22(unsigned char *d_imagedata)
{
	unsigned int i=0;
	unsigned short int * p_buf = NULL;
	unsigned short int * os_buf;
	unsigned int byd_fps_number = 0 ;
	unsigned int area_block = 0;
    char uniformity_flag = 0 ;
	int image_block_values[3]={0,0,0};
	int image_diff[3]={0,0,0};
	int buffer_max = 0;
	int buffer_min = 0;
	int up_down_flag = 0;
	static int count_fail = 0; //防止单向失调更新的错误
	int ret =0;
	int data_flag = 0;
	
	os_buf = (unsigned short int *)byd_malloc((BYD_FPS_IMAGE_WIDTH*BYD_FPS_IMAGE_HEIGHT)*sizeof(unsigned short int));
	if (!os_buf ) {
		pr_err(" os_buf failed to get free memory\n");
		return -1;
	}	
	p_buf =  (unsigned short int *)byd_malloc((BYD_FPS_IMAGE_WIDTH*BYD_FPS_IMAGE_HEIGHT)*sizeof(unsigned short int));
	if (!p_buf) {
		pr_err(" p_buf failed to get free memory\n");
		return -1;
	}
	DBG_TIME("%s, byd_malloc(p_buf) OK\n", __func__);
	
	memset(p_buf,0,(BYD_FPS_IMAGE_WIDTH*BYD_FPS_IMAGE_HEIGHT)*sizeof(unsigned short int));
 	//拆分并且重新排列顺序.
	byd_fps_encoding22(d_imagedata, p_buf);//p_buf代表示前采回的失调值做了拆分和排序的数据；
	byd_fps_encoding22(&this_byd_fps->data_buf_os_otp[1], os_buf);//os_buf表示上一帧失调值做了拆分和排序的数据；
	

 	for(i = 0; i < OS_AREA_BLOCK22; i++) {
		printk("byd %s:i=%d,OS_Of_Finger_Detect22 is :%d\n",__func__,i,OS_Of_Finger_Detect22[i]);
	}
	
    for(i = 0; i < OS_AREA_BLOCK22; i++) {
		if( i == 0) {
			//area_block = VAL_FG_ROW_START_A;
			if(BYD_FPS_IMAGE_HEIGHT != 56) {
				area_block = byd_VAL_FG_ROW_START_72X64[i];
			} else {
				area_block = byd_VAL_FG_ROW_START_56X64[i];
			}
		} else if (i == 1) {
			//area_block = VAL_FG_ROW_START_B;
			if(BYD_FPS_IMAGE_HEIGHT != 56) {
				area_block = byd_VAL_FG_ROW_START_72X64[i];
			} else {
				area_block = byd_VAL_FG_ROW_START_56X64[i];
			}
		} else if (i == 2) {
			//area_block = VAL_FG_ROW_START_C;
			if(BYD_FPS_IMAGE_HEIGHT != 56) {
				area_block = byd_VAL_FG_ROW_START_72X64[i];
			} else {
				area_block = byd_VAL_FG_ROW_START_56X64[i];
			}
		}
		if(((i==0)||(i==2))&&(BYD_FPS_IMAGE_HEIGHT == 56)) {
			image_block_values[i] = byd_fps_get_osdata_from_block_56X64(&p_buf[0],area_block);
		} else {
			image_block_values[i] = byd_fps_get_osdata_from_block_72X64(&p_buf[0],area_block);
		}
		
		DBG_TIME("%s:i=%d,image_block_values is :%d\n",__func__,i,image_block_values[i]);
	}
	DBG_TIME("%s:byd_fps_block_diff is :%d\n",__func__,OS_BLOCK_DIFF);
	DBG_TIME("%s:byd_fps_uniformity_values is :%d\n",__func__,OS_UNIFORMITY_DIFF22);
	
	for(i = 0; i < OS_AREA_BLOCK22; i++) {
		if(((OS_Of_Finger_Detect22[i] - image_block_values[i]) < OS_BLOCK_DIFF) && ((OS_Of_Finger_Detect22[i] - image_block_values[i]) > (-OS_BLOCK_DIFF))) {
			byd_fps_number ++ ;
		}
		image_diff[i] = OS_Of_Finger_Detect22[i] - image_block_values[i];
		DBG_TIME("%s:i=%d,image_diff is :%d\n",__func__,i,image_diff[i]);
	}

	buffer_max = image_diff[0];
	buffer_min = image_diff[0];
	
	for(i = 0; i < OS_AREA_BLOCK22; i++) { 
		if(image_diff[i] > buffer_max)
			buffer_max = image_diff[i];
		if(image_diff[i] < buffer_min)
			buffer_min = image_diff[i];
	}

	if((buffer_max - buffer_min) < OS_UNIFORMITY_DIFF22) {//各区域的一致性
		uniformity_flag = 1;
	}
	printk("byd uniformity_flag=%d\n",uniformity_flag);
	if((byd_fps_number == OS_AREA_BLOCK22) && (uniformity_flag == 1)) {
		//up_down_flag = FINGER_UP;//手指抬起
		ret = os_data_compare(p_buf,os_buf);//再次判断采集到的失调值数据是否满足条件：与上次的失调值做比较
		if ( (ret <= OS_PGD_ACT && ret >=0) || count_fail >= OS_UPDATE_FAILED_MAX || OS_First_Catch==1) {
			OS_First_Catch = 0;
			data_flag = 1;
			count_fail = 0;	
			printk("byd update success!!!!count_fail=%d\n",count_fail);
		} else if (ret < 0 && ret > OS_PGD_NEG ) {
			data_flag = 1;
			count_fail = 0;
			printk("byd update success!!!!count_fail=%d\n",count_fail);
		} else  {
			data_flag =0;
			count_fail ++;
			printk("byd update failed!!!!count_fail=%d\n",count_fail);
		}
		printk("byd data_flag=%d\n",data_flag);	
		if (data_flag==1)
			up_down_flag = FINGER_UP;//手指抬起
		else
			up_down_flag = FINGER_DOWN;//手指按下
		
		
	} else {
		up_down_flag = FINGER_DOWN;//手指按下
	}

	DBG_TIME("%s:up_down_flag is :%d\n",__func__,up_down_flag);

	if(p_buf != NULL) {
		byd_free(p_buf);
		p_buf = NULL;
	}

    return up_down_flag;
}
static int os_fng_updown_compare(unsigned char *d_imagedata)
{
	unsigned int i=0;
	int up_down_flag = 0;
	if(byd_fps_chip_flag == 0x12){
		unsigned int j=0;
		unsigned int image_temp = 0;
		unsigned int image_value = 0;
		unsigned int image_tempvalue = 0;
		unsigned int tempscript = 0;
		unsigned short int * p_buf = NULL;
		unsigned int sum_block_value1 = 0;
		unsigned int sum_block_value2 = 0;
		unsigned int sum_block_value3 = 0;
		unsigned int byd_fps_count = 0 ;
		unsigned int byd_fps_number = 0 ;
		char uniformity_flag = 0 ;
		int image_block_values[9]={0,0,0,0,0,0,0,0,0};
		int image_diff[9]={0,0,0,0,0,0,0,0,0};
		char column_A[8]={30,26,22,18,14,10,6,2};       //???A1?B1?C1?????
		char column_B[8]={46,42,38,34,81,85,89,93};     //???A2?B2?C2?????
		char column_C[8]={49,53,57,61,65,69,73,77};     //???A3?B3?C3?????
		int buffer_max = 0;
		int buffer_min = 0;
		

		p_buf =  (unsigned short int *)byd_malloc((BYD_FPS_IMAGE_WIDTH12*BYD_FPS_IMAGE_HEIGHT12)*sizeof(unsigned short int));
		memset(p_buf,0,(BYD_FPS_IMAGE_WIDTH12*BYD_FPS_IMAGE_HEIGHT12)*sizeof(unsigned short int));

		for( i = 0;i < BYD_FPS_IMAGE_SIZE12 ;i+=3) {
			image_temp = d_imagedata[i];
			image_value = d_imagedata[i+1];
			image_tempvalue = d_imagedata[i+2];

			p_buf[tempscript] = (image_temp<<4) | (image_value>>4);
			p_buf[tempscript+1] = ((image_value&0x0f)<<8) | image_tempvalue;

			tempscript = tempscript + 2 ;
		}

		tempscript = VAL_FG_ROW_START_A;

		for( i = tempscript; i < (tempscript+8); i++) {
			for( j = 0; j < 8; j++) {
				sum_block_value1 += p_buf[i*BYD_FPS_IMAGE_WIDTH12 + column_A[j]];  //A1区域
				sum_block_value2 += p_buf[i*BYD_FPS_IMAGE_WIDTH12 + column_B[j]];  //A2区域
				sum_block_value3 += p_buf[i*BYD_FPS_IMAGE_WIDTH12 + column_C[j]];  //A3区域
				byd_fps_count++;
			}
		}

		image_block_values[0] = sum_block_value1/byd_fps_count;
		image_block_values[1] = sum_block_value2/byd_fps_count;
		image_block_values[2] = sum_block_value3/byd_fps_count;

		tempscript = VAL_FG_ROW_START_B12;
		sum_block_value1 = 0;
		sum_block_value2 = 0;
		sum_block_value3 = 0;
		byd_fps_count = 0;

		for( i = tempscript; i < (tempscript+8); i++) {
			for( j = 0; j < 8; j++) {
				sum_block_value1 += p_buf[i*BYD_FPS_IMAGE_WIDTH12 + column_A[j]];  //B1区域
				sum_block_value2 += p_buf[i*BYD_FPS_IMAGE_WIDTH12 + column_B[j]];  //B2区域
				sum_block_value3 += p_buf[i*BYD_FPS_IMAGE_WIDTH12 + column_C[j]];  //B3区域
				byd_fps_count++;
			}
		}

		image_block_values[3] = sum_block_value1/byd_fps_count;
		image_block_values[4] = sum_block_value2/byd_fps_count;
		image_block_values[5] = sum_block_value3/byd_fps_count;

		tempscript = VAL_FG_ROW_START_C12;
		sum_block_value1 = 0;
		sum_block_value2 = 0;
		sum_block_value3 = 0;
		byd_fps_count = 0;

		for( i = tempscript; i < (tempscript+8); i++) {
			for( j = 0; j < 8; j++) {
				sum_block_value1 += p_buf[i*BYD_FPS_IMAGE_WIDTH12 + column_A[j]];  //C1区域
				sum_block_value2 += p_buf[i*BYD_FPS_IMAGE_WIDTH12 + column_B[j]];  //C2区域
				sum_block_value3 += p_buf[i*BYD_FPS_IMAGE_WIDTH12 + column_C[j]];  //C3区域
				byd_fps_count++;
			}
		}

		image_block_values[6] = sum_block_value1/byd_fps_count;
		image_block_values[7] = sum_block_value2/byd_fps_count;
		image_block_values[8] = sum_block_value3/byd_fps_count;

		for(i = 0; i < 9; i++) {
			DBG_TIME("%s:i=%d,OS_Of_Finger_Detect12 is :%d\n",__func__,i,OS_Of_Finger_Detect12[i]);
		}

		for(i = 0; i < 9; i++) {
			DBG_TIME("%s:i=%d,image_block_values is :%d\n",__func__,i,image_block_values[i]);
		}

		DBG_TIME("%s:byd_fps_block_diff is :%d\n",__func__,OS_BLOCK_DIFF);
		DBG_TIME("%s:byd_fps_uniformity_values is :%d\n",__func__,OS_UNIFORMITY_DIFF);

		for(i = 0; i < 9; i++) {
			if(((OS_Of_Finger_Detect12[i] - image_block_values[i]) < OS_BLOCK_DIFF) && ((OS_Of_Finger_Detect12[i] - image_block_values[i]) > (-OS_BLOCK_DIFF))) {
				byd_fps_number ++ ;
			}
			image_diff[i] = OS_Of_Finger_Detect12[i] - image_block_values[i];
			DBG_TIME("%s:i=%d,image_diff is :%d\n",__func__,i,image_diff[i]);
		}

		buffer_max = image_diff[0];
		buffer_min = image_diff[0];

		for(i = 0; i < 9; i++) {
			if(image_diff[i] > buffer_max)
				buffer_max = image_diff[i];
			if(image_diff[i] < buffer_min)
				buffer_min = image_diff[i];
		}

		if((buffer_max - buffer_min) < OS_UNIFORMITY_DIFF) {//9个区域的一致性
			uniformity_flag = 1;
		}

		if((byd_fps_number == 9) && (uniformity_flag == 1)) {
			up_down_flag = FINGER_UP;//????
		} else {
			up_down_flag = FINGER_DOWN;//手指按下
		}

		DBG_TIME("%s:up_down_flag is :%d\n",__func__,up_down_flag);

		if(p_buf != NULL) {
			byd_free(p_buf);
			p_buf = NULL;
		}
	}
	if(byd_fps_chip_flag == 0x11){
		unsigned int image_temp = 0;
		unsigned int image_value = 0;
		unsigned int image_tempvalue = 0;
		unsigned int tempscript = 0;
		unsigned short int * p_buf = NULL;
		unsigned int byd_fps_number = 0 ;
		unsigned int area_block = 0;
		char uniformity_flag = 0 ;
		int image_block_values[5]={0,0,0,0,0};
		int image_diff[5]={0,0,0,0,0};
		int buffer_max = 0;
		int buffer_min = 0;
		//int up_down_flag = 0;

		p_buf =  (unsigned short int *)byd_malloc((BYD_FPS_IMAGE_WIDTH11*BYD_FPS_IMAGE_HEIGHT11)*sizeof(unsigned short int));
		if (!p_buf) {
			pr_err(" p_buf failed to get free memory\n");
			return -1;
		}
		DBG_TIME("%s, byd_malloc(p_buf) OK\n", __func__);
	
		memset(p_buf,0,(BYD_FPS_IMAGE_WIDTH11*BYD_FPS_IMAGE_HEIGHT11)*sizeof(unsigned short int));

		for( i = 0;i < BYD_FPS_IMAGE_SIZE11 ;i+=3) {
			image_temp = d_imagedata[i];
			image_value = d_imagedata[i+1];
			image_tempvalue = d_imagedata[i+2];

			p_buf[tempscript] = (image_temp<<4) | (image_value>>4);
			p_buf[tempscript+1] = ((image_value&0x0f)<<8) | image_tempvalue;

			tempscript = tempscript + 2 ;
		}
 
		for(i = 0; i < OS_AREA_BLOCK11; i++) {
			printk("%s:i=%d,OS_Of_Finger_Detect11 is :%d\n",__func__,i,OS_Of_Finger_Detect11[i]);
		}
	
		for(i = 0; i < OS_AREA_BLOCK11; i++) {
			if( i == 0) {
				area_block = VAL_FG_ROW_START_A;
			} else if (i == 1) {
				area_block = VAL_FG_ROW_START_B11;
			} else if (i == 2) {
				area_block = VAL_FG_ROW_START_C11;
			} else if (i == 3) {
				area_block = VAL_FG_ROW_START_D;
			} else if (i == 4) {
				area_block = VAL_FG_ROW_START_E;
			}
			image_block_values[i] = byd_fps_get_osdata_from_block(&p_buf[0],area_block);
		
			DBG_TIME("%s:i=%d,image_block_values is :%d\n",__func__,i,image_block_values[i]);
		}

	
		DBG_TIME("%s:byd_fps_block_diff is :%d\n",__func__,OS_BLOCK_DIFF);
		DBG_TIME("%s:byd_fps_uniformity_values is :%d\n",__func__,OS_UNIFORMITY_DIFF);
	
		for(i = 0; i < OS_AREA_BLOCK11; i++) {
			if(((OS_Of_Finger_Detect11[i] - image_block_values[i]) < OS_BLOCK_DIFF) && ((OS_Of_Finger_Detect11[i] - image_block_values[i]) > (-OS_BLOCK_DIFF))) {
				byd_fps_number ++ ;
			}
			image_diff[i] = OS_Of_Finger_Detect11[i] - image_block_values[i];
			DBG_TIME("%s:i=%d,image_diff is :%d\n",__func__,i,image_diff[i]);
		}

		buffer_max = image_diff[0];
		buffer_min = image_diff[0];
	
		for(i = 0; i < OS_AREA_BLOCK11; i++) { 
			if(image_diff[i] > buffer_max)
				buffer_max = image_diff[i];
			if(image_diff[i] < buffer_min)
				buffer_min = image_diff[i];
		}

		if((buffer_max - buffer_min) < OS_UNIFORMITY_DIFF) {//5个区域的一致性
			uniformity_flag = 1;
		}

		if((byd_fps_number == OS_AREA_BLOCK11) && (uniformity_flag == 1)) {
			up_down_flag = FINGER_UP;//手指抬起
		} else {
			up_down_flag = FINGER_DOWN;//手指按下
		}

		DBG_TIME("%s:up_down_flag is :%d\n",__func__,up_down_flag);

		if(p_buf != NULL) {
			byd_free(p_buf);
			p_buf = NULL;
		}
		
	}
    return up_down_flag;
}
//#else

/* -----------------------------------------------------------------------------
 * byd_os_update() - acquiring OS image and update to fingerprint algorithm
 * @byd_fps:	fps object is passed in for acquiring OS image
 *
 * image acquisition occurs only if no finger is presented
 * after successful acquiring, Os_Scan_Expired & Match_Failed_Counter is cleared
 *
 * Return:	0 -- success, -1 -- failure
 * -----------------------------------------------------------------------------
 */
int byd_os_update(struct byd_fps_data *byd_fps)
{
	static unsigned char os_scan_failed_counter = 0;
	int up_down_flag = 0;
	unsigned char fng_state = 0;

	DBG_TIME("%s:start scan os\n",__func__);
	byd_fps_mdelay(5); // wait until finger absent totally
	byd_fps_mutex_lock(1,1);
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		byd_fps_decode_cfg(byd_fps->spi,1);//只有在扫描失调值时配置为不加密
	}
	byd_fps_read_image(0x0E);
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		byd_fps_decode_cfg(byd_fps->spi,0);
	}
	byd_fps_mutex_lock(2,1);

	//再次判断是否抬起
	byd_fps_mutex_lock(1,1);
	fng_state = byd_fps_get_finger_state(byd_fps->spi);
	byd_fps_mutex_lock(2,1);

	if(fng_state == 0) {
		if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
			up_down_flag = os_fng_updown_compare(&byd_fps->data_buffer[1]);
		}else if(byd_fps_chip_flag == 0x22){
			up_down_flag = byd_os_fng_updown_compare22(&byd_fps->data_buffer[1]);	
		}
		if(up_down_flag == FINGER_UP) { //手指抬起
			os_scan_failed_counter = 0;
			Os_Scan_Expired = 0;
			Match_Failed_Counter = 0;
			memcpy(byd_fps->data_buf_os_otp, byd_fps->data_buffer, BYD_FPS_IMAGE_BUFFER_SIZE * BYD_FPS_MAX_FRAME);
			byd_fps_mutex_lock(1,2);
			byd_fps_saveadjust_image(&byd_fps->data_buf_os_otp[1]);
			byd_fps_mutex_lock(2,2);
			return 0; // OS image acquisition success
		} else if (up_down_flag == FINGER_DOWN) { //手指按下
			DBG_TIME("%s:*****************os update failed**************\n",__func__);
		}
	}
	printk("%s: *** finger detected, data discarded! *** \n", __func__);
	//Os_Scan_Expired = 1;
	os_scan_failed_counter ++;
	if(os_scan_failed_counter > OS_SCAN_FAILED_MAX) {
		// TO DO ...
		DBG_TIME("%s:os set failed\n",__func__);
		DBG_TIME("%s:you set throld is too low\n",__func__);
	}
	DBG_TIME("%s:finger on chip before scan\n",__func__);
	return -1; // OS image acquisition failure
}
/* -----------------------------------------------------------------------------
 * byd_os_scan() - acquiring OS image and update to fingerprint algorithm to check num of bad point
 * @byd_fps:	fps object is passed in for acquiring OS image
 *
 * 
 *
 * Return:	0 -- success, -1 -- failure
 * -----------------------------------------------------------------------------
 */
unsigned int bpnum = 0;
int byd_os_scan(struct byd_fps_data *byd_fps)
{
	//static unsigned char os_scan_failed_counter = 0;
	int up_down_flag = 0;
	unsigned char fng_state = 0;

	DBG_TIME("%s:start scan os\n",__func__);
	byd_fps_mdelay(5); // wait until finger absent totally
	byd_fps_mutex_lock(1,1);
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		byd_fps_decode_cfg(byd_fps->spi,1);//只有在扫描失调值时配置为不加密
	}
	byd_fps_read_image(0x0E);
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		byd_fps_decode_cfg(byd_fps->spi,0);
	}
	byd_fps_mutex_lock(2,1);

	//再次判断是否抬起
	byd_fps_mutex_lock(1,1);
	fng_state = byd_fps_get_finger_state(byd_fps->spi);
	byd_fps_mutex_lock(2,1);

	if(fng_state == 0) {
		if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
			up_down_flag = os_fng_updown_compare(&byd_fps->data_buffer[1]);
		}else if(byd_fps_chip_flag == 0x22){
			up_down_flag = byd_os_fng_updown_compare22(&byd_fps->data_buffer[1]);	
		}

		if(up_down_flag == FINGER_UP) { //手指抬起
			//os_scan_failed_counter = 0;
			//Os_Scan_Expired = 0;
			//Match_Failed_Counter = 0;
			memcpy(byd_fps->data_buf_os_otp, byd_fps->data_buffer, BYD_FPS_IMAGE_BUFFER_SIZE * BYD_FPS_MAX_FRAME);
			byd_fps_mutex_lock(1,2);
			//byd_fps_saveadjust_image(&byd_fps->data_buf_os_otp[1]);
			byd_badpoint_judge(&byd_fps->data_buf_os_otp[1],(unsigned int *)(&bpnum));
			byd_fps_mutex_lock(2,2);
			
		} 
	}
	return 0; // OS image acquisition success
	
}

/*
 * os_get_compens_image() - expand column compensation data to image
 * @data_image:	image buffer
 *
 * Return:	0 -- success, -1 -- failure
 */
 //#ifdef CONFIG_FPS12
static int os_get_compens_image(unsigned char *data_image)
{
	int ret = 0;
	unsigned int i = 0;
	unsigned int j = 0;
	unsigned int image_temp = 0;
	unsigned int image_value = 0;
	unsigned int tempscript = 0;
	unsigned int * p_buf = NULL;
	if(byd_fps_chip_flag == 0x12){
		unsigned char column47_info[2];
		unsigned char column48_info[2];

		ret = byd_fps_otp_read(OTP_BAD_COLUMN_4712, 1, &column47_info[0],0);  //read ratio of "bad column 47"
		if (ret == -1)
			return ret;
		ret = byd_fps_otp_read(OTP_BAD_COLUMN_4812, 1, &column48_info[0],0);  //read ratio of "bad column 48"

		if(column47_info[1] == 255)
			column47_info[1] = 128;
	
		if(column48_info[1] == 255)
			column48_info[1] = 128;
	
		p_buf =  (unsigned int *)byd_malloc((BYD_FPS_IMAGE_WIDTH12*BYD_FPS_IMAGE_HEIGHT12)*sizeof(unsigned int));
		//memset(p_buf,1024,(BYD_FPS_IMAGE_WIDTH*BYD_FPS_IMAGE_HEIGHT)*sizeof(unsigned int));

		for(i = 0; i < BYD_FPS_IMAGE_HEIGHT12; i++) {
			for( j = 0; j< BYD_FPS_IMAGE_WIDTH12; j++) {
				if (j == 47)
					p_buf[i*BYD_FPS_IMAGE_WIDTH12+j] = column47_info[1] + 896 ; //47列修调系数 芯片高低位烧反 column47_info[0]
				else if (j == 48)
					p_buf[i*BYD_FPS_IMAGE_WIDTH12+j] = column48_info[1] + 896 ; //48列修调系数
				else
					p_buf[i*BYD_FPS_IMAGE_WIDTH12+j] = 1024;
			}
		}

		for(i = 0;i < (BYD_FPS_IMAGE_WIDTH12*BYD_FPS_IMAGE_HEIGHT12); i+=2)	{
			image_temp = p_buf[i];
			image_value = p_buf[i+1];

			data_image[tempscript] = image_temp >> 4 ;
			data_image[tempscript+1] = ((image_temp & 0xf) << 4) | (image_value >> 8);
			data_image[tempscript+2] = image_value & 0xff;

			tempscript = tempscript + 3 ;
		}

		if(p_buf != NULL)
		{
			byd_free(p_buf);
			p_buf = NULL;
		}
	}
	if(byd_fps_chip_flag == 0x11){
		p_buf =  (unsigned int *)byd_malloc((BYD_FPS_IMAGE_WIDTH11*BYD_FPS_IMAGE_HEIGHT11)*sizeof(unsigned int));
		//memset(p_buf,1024,(BYD_FPS_IMAGE_WIDTH*BYD_FPS_IMAGE_HEIGHT)*sizeof(unsigned int));
		if (!p_buf) {
			pr_err(" p_buf failed to get free memory\n");
			return -1;
		}
		DBG_TIME("%s, byd_malloc(p_buf) OK\n", __func__);
	
		for(i = 0; i < BYD_FPS_IMAGE_HEIGHT11; i++) {
			for( j = 0; j< BYD_FPS_IMAGE_WIDTH11; j++) {
				p_buf[i*BYD_FPS_IMAGE_WIDTH11+j] = 1024;
			}
		}

		for(i = 0;i < (BYD_FPS_IMAGE_WIDTH11*BYD_FPS_IMAGE_HEIGHT11); i+=2)	{
			image_temp = p_buf[i];
			image_value = p_buf[i+1];

			data_image[tempscript] = image_temp >> 4 ;
			data_image[tempscript+1] = ((image_temp & 0xf) << 4) | (image_value >> 8);
			data_image[tempscript+2] = image_value & 0xff;

			tempscript = tempscript + 3 ;
		}

		if(p_buf != NULL)
		{
			byd_free(p_buf);
			p_buf = NULL;
		}	
	}
	return ret;
}
//读otp的修正（补偿）系数 
#define OTP_ADJUSTIMAGE_START   0x1D0
#define OTP_ADJUSTIMAGE_LTENGTH   32
static int byd_read_otp_saveadjustround(unsigned short *imageadjudge_buf)
{
	int ret,i;
	ret = byd_fps_otp_read(OTP_ADJUSTIMAGE_START,OTP_ADJUSTIMAGE_LTENGTH,(unsigned char*)imageadjudge_buf,0);
	DBG_TIME("byd OTP imageadjudge_buf byte:");
	for(i=0;i<OTP_ADJUSTIMAGE_LTENGTH/2;i++) {
		DBG_TIME("byd[%d]=0x%x,", i,imageadjudge_buf[i]);
		if(((i+1)%8) == 0) {
			DBG_TIME("\n");
		}
	}
	DBG_TIME("\n");
	return ret;
}

/* -----------------------------------------------------------------------------
 * byd_os_init() - OS adjustment and pixel compensation during machine startup
 * @byd_fps:	fps object is passed in for acquiring image
 *
 * OS image should be in data_buf_os_otp after the function call
 *
 * Return:	0 -- success, -1 -- failure
 * -----------------------------------------------------------------------------
 */
int byd_os_init(struct byd_fps_data *byd_fps)
{
	int ret = 0;
    unsigned int i = 0;
	
	unsigned short int bad_point[519];	
	unsigned short imageadjudge_buf22[16];
	for(i=0;i<519;i++){
		bad_point[i] = 0;
	}
	
	byd_fps->data_buf_os_otp = (u8 *)byd_malloc(BYD_FPS_IMAGE_BUFFER_SIZE * BYD_FPS_MAX_FRAME);
	if (!byd_fps->data_buf_os_otp) {
		pr_err("failed to get free memory\n");
		return -1;
	}
	DBG_TIME("%s, byd_malloc(buf_os) OK\n", __func__);
	byd_fps_chip_idle(byd_fps->spi);

	DBG_TIME("%s:otp read start \n",__func__);
	os_read_otp_bad_point(bad_point);
	
	/*read 修正（补偿）系数*/
	if(byd_fps_chip_flag == 0x22){
		byd_read_otp_saveadjustround(imageadjudge_buf22);
	}
	if(byd_fps_chip_flag == 0x12){
		for( i = 0; i < OS_AREA_BLOCK12; i++)
			OS_Of_Finger_Detect12[i] = byd_fps_rd_fg_os_cfg(byd_fps->spi, i);
	}
	if(byd_fps_chip_flag == 0x11){
		for( i = 0; i < OS_AREA_BLOCK11; i++)
			OS_Of_Finger_Detect11[i] = byd_fps_rd_fg_os_cfg(byd_fps->spi, i);
	}
	if(byd_fps_chip_flag == 0x22){
		for( i = 0; i < OS_AREA_BLOCK22; i++)
			OS_Of_Finger_Detect22[i] = byd_fps_rd_fg_os_cfg(byd_fps->spi, i);
		
	}
	
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		ret = os_read_otp_data(&byd_fps->data_buf_os_otp[1]);//OTP读补偿系数或失调值
		if (ret == -1)
			return ret;
	}
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		Os_Otp_or_Scan = os_is_otp_or_scan();
		if (Os_Otp_or_Scan < 0)
			return Os_Otp_or_Scan;
		DBG_TIME("%s:otp read end \n",__func__);

		//0x55表示存放补偿系数，0xaa表示存放失调值，0xff表示旧版存放失调值
		if (Os_Otp_or_Scan == OS_IN_OTP || Os_Otp_or_Scan == OS_IN_OTP_LEGACY) {
			byd_fps_saveadjust_image(&byd_fps->data_buf_os_otp[1]);
			//读otp读取必须在idle模式下
			os_get_compens_image(&byd_fps->data_buffer[1]);//otp读取补偿系数并转换成图像
			byd_fps_chip_detect(byd_fps->spi); // restore fg detect mode after rd OTP
			byd_fps_saveadjust2(&byd_fps->data_buffer[1], bad_point[0], &bad_point[1]);
		} else { // if (Os_Otp_or_Scan == OS_BY_SCAN || Os_Otp_or_Scan == OS_OTP_ERR)
			byd_fps_saveadjust2(&byd_fps->data_buf_os_otp[1], bad_point[0], &bad_point[1]);  //传补偿系数
			byd_fps_chip_detect(byd_fps->spi);
			memset(byd_fps->data_buf_os_otp, 0xFF, BYD_FPS_IMAGE_BUFFER_SIZE);
			byd_fps_saveadjust_image(&byd_fps->data_buf_os_otp[1]);
			// acquire OS data during machine startup (only when finger is not present
			byd_os_update(byd_fps); //采集并传输失调值
		}
	}else if(byd_fps_chip_flag == 0x22){
		byd_fps_saveadjustround(imageadjudge_buf22);//传修正（补偿）系数
		byd_fps_saveadjust2(&byd_fps->data_buf_os_otp[1], bad_point[0], &bad_point[1]);  //传补偿(修正)系数和坏点信息
		//byd_fps_saveadjust2(&byd_fps->data_buf_os_otp[1], 0, &bad_point[1]);  //传补偿(修正)系数和坏点信息
	
		byd_fps_chip_detect(byd_fps->spi);
	
		// Preparing default OS data in case of finger present during maching startup.
		memset(byd_fps->data_buf_os_otp, 0xFF, BYD_FPS_IMAGE_BUFFER_SIZE);
		DBG_TIME("%s:byd_fps_save_adjust_image() \n",__func__);
		byd_fps_saveadjust_image(&byd_fps->data_buf_os_otp[1]);	//失调值数据
	
		DBG_TIME("%s:byd_os_update \n",__func__);
		// Acquires OS data during machine start-up (only when finger is not present
		OS_First_Catch = 1;
		byd_os_update(byd_fps); //采集并传输失调值
		if (Os_Scan_Expired == 0)
			OS_First_Catch = 0;
		
	}
	return ret;
}

/* -----------------------------------------------------------------------------
 * byd_os_exit() - destructor of byd_os module
 * @byd_fps:	fps object
 *
 * Return:	none
 * -----------------------------------------------------------------------------
 */
void byd_os_exit(struct byd_fps_data *byd_fps)
{
	if(byd_fps->data_buf_os_otp)
		byd_free(byd_fps->data_buf_os_otp);
}


// *******************************************************************************
// * end of Module :  byd_os
// *******************************************************************************
