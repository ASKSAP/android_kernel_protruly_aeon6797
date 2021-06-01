/*
 * File:         byd_fps_wakeup.c
 *
 * Created:	     2015-11-22
 * Description:  BYD Fingerprint IC driver for Android
 *
 * Copyright (C) 2015 BYD Company Limited
 *
 * Licensed under the GPL-2 or later.
 *
 * History:
 *
 * Contact: qi.ximing@byd.com;
 *
 */

#include <linux/spi/spi.h>
#include <linux/delay.h>      // msleep()
#include <linux/input.h>
#include <linux/interrupt.h>  // IRQ_HANDLED
#include <linux/kthread.h>
// byd_algorithm.h must be included after linux inclution and before byd_fps_*.h
#include "byd_algorithm.h"  // must after kernel head include
#include "byd_fps_bf66xx.h" //#include <linux/input/byd_fps_bf66xx.h>
#include "byd_fps_libbf663x.h"

#if defined(CONFIG_HAS_EARLYSUSPEND)
#include <linux/earlysuspend.h>
#endif


/* ======================== public global constant ========================= */
extern const unsigned int BYD_FPS_IMAGE_SIZE;
//extern const unsigned char BYD_FPS_16BIT_IMAGE;

extern unsigned char byd_fps_fg_det_state;
extern u8 work_cmd;
extern struct byd_fps_thread_task *thread_task;
extern struct mutex byd_fps_mutex_chip;
extern struct mutex byd_fps_mutex_alg;

#if (defined BYD_FPS_ALG_IN_KERNEL) || (defined BYD_FPS_ALG_IN_TZ)
extern unsigned char byd_fps_alg_ret[6];
extern unsigned int byd_fps_match_id;
#endif

#ifdef BYD_FPS_ALG_IN_KERNEL
extern unsigned char byd_fps_add_txt;//是否加载模板到内存
extern unsigned short *byd_fps_image_data;
#endif

extern unsigned char byd_last_cmd;

#ifdef BYD_FINGER_INTR_CNT
extern u8 finger_intr_flag;
extern u8 finger_intr_num;
#endif

extern unsigned char finger_intr_ctrl;
extern const unsigned char FINGER_DOWN;
extern const unsigned char FINGER_UP;
extern const unsigned char byd_fps_fae_detect[6];
extern unsigned char byd_fps_fae_detect_temp[6];
extern unsigned char finger_present;
extern unsigned long set_timeout_finger_down;
extern unsigned long set_timeout_finger_up;
extern unsigned char byd_fps_instr_stop_flag;	//中断停止标志.
extern unsigned char byd_fps_int_test;//工厂测试标志？？
extern unsigned char byd_fps_sub_value22[6];

extern struct byd_fps_data  *this_byd_fps;
struct input_dev *byd_input_dev;
unsigned char byd_fps_gesture_suspend = 0;
unsigned char byd_fps_susflag = 0;
//#ifdef BYD_FPS_REPORT_KEY
#if 1// (defined(BYD_FPS_REPORT_KEY))||(defined(BYD_FPS_FOCUS))||(defined(BYD_FPS_REPORT_KEY_ONLY))
extern unsigned int byd_fps_keydown_flag;
extern unsigned int byd_fps_keyopen_flag;
#endif


extern unsigned int byd_fps_keydown_flag;
#ifdef BYD_FPS_TIMER_KEY
unsigned char byd_fps_get_timer_flag(void);
void byd_fps_downif(void);
#endif
unsigned char byd_fps_finger_up_flag = 0;
#ifdef BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED
unsigned char byd_fps_key_func_flag = 0;
#endif
#if (defined(BYD_FPS_REPORT_KEY))||(defined(BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED))
unsigned char byd_fps_key_mesg = 0;
#endif
#ifdef BYD_FPS_FASYNC
 void byd_fps_send_fasync(void);
#endif
#ifdef BYD_FPS_FOCUS
void byd_fps_timer_up_stop(void);
extern unsigned char gest_int_number;
#endif

//#if (defined(BYD_FPS_REPORT_KEY))&&(defined(BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED))
#ifdef BYD_FPS_GEST
unsigned char byd_fps_gest_key_flag = 0;
#endif
extern unsigned char byd_fps_finger_up_async_flag;
/***  Module    :  byd_os  ***/
extern unsigned char Os_Scan_Expired;
extern unsigned char Os_Otp_or_Scan;
extern unsigned char byd_fps_chip_flag;
/***  Module end:  byd_os  ***/
/**
 * struct byd_fps_wakeup_data_t - system wakeup data
 * @early_suspend:	use Android's early suspend
 * @fb_notif:	use FB if configured
 *
 */
 
struct byd_fps_wakeup_data_t
{
  #if defined(CONFIG_HAS_EARLYSUSPEND)
	struct early_suspend early_suspend;
  #elif defined(CONFIG_FB)
	struct notifier_block fb_notif;
  #endif
  	struct workqueue_struct *byd_fps_workqueue_key;
	struct work_struct byd_fps_work_key;
	
	//struct __wait_queue_head *waiting_finger_state; // wait_queue_head_t
	//int finger_state_done;
	
	struct workqueue_struct *byd_fps_workqueue_susp;
	struct work_struct byd_fps_work_susp;
	
	
	
	//struct input_dev *byd_input_dev;
	unsigned char byd_fps_wakeup_lock;//指纹芯片能否休眠唤醒的开关
	unsigned char byd_fps_suspend_flag;//休眠标志
	unsigned char byd_fps_susp_match_cnt;
	unsigned char byd_fps_suspend_qual;
};


struct byd_fps_wakeup_data_t *byd_fps_wakeup_data = NULL;

#ifdef MTK_SPI_INIT_BUG_FIX // for client specific bug - SPI init failure
	int byd_fps_init_port(void);
#endif

#ifdef BYD_FPS_GESTURE
void byd_fps_gesture_off(void);
#endif

#ifdef BYD_FPS_TIMER
#ifdef BYD_TIMER_FNG_UP
int byd_fps_upif(void);
#endif

#ifdef BYD_TIMER_FNG_DOWN
void byd_fps_scan_set(unsigned char flag);
#endif
#endif//end BYD_FPS_TIMER

//#define BYD_FPS_WAKE_REPORT_KEY

//#define BYD_FPS_TASK_RPT
#ifdef BYD_FPS_TASK_RPT
static void byd_fps_input_pwrkey_report(unsigned long data);
static DECLARE_TASKLET(byd_fps_input_pwrkey_tasklet, byd_fps_input_pwrkey_report, 0);
#endif

#define BYD_FPS_WORKQUEUE_NAME_KEY  "byd_fps_workqueue_name_key"
#define BYD_FPS_WORKQUEUE_NAME_SUSP  "byd_fps_workqueue_name_susp"
void byd_fps_rd_fpd(struct byd_fps_data *byd_fps);
long byd_fps_wait_event_interruptible_timeout(long timeout);
void byd_fps_stop_thread(u8 work_cmd);
int byd_fps_read_image(int count);
void byd_fps_start_template_merging(void);
static int byd_fps_register_input_device(struct device *dev, struct input_dev **input_dev);
void byd_fps_wakeup_exit(struct device *dev);
//void byd_fps_finger_detect_pre_cfg(unsigned char up_down_flag,unsigned char quick_slow_flag);
int byd_fps_qual_judge( unsigned char enrol_match_flag );

int byd_fps_spi_write(struct spi_device *spi, u8 reg, u8 val);
extern void byd_fps_mutex_lock(unsigned char lock_flag,unsigned char type_flag);
void byd_fps_msleep(unsigned int msecs);

void byd_fps_start_timer_key(void);
void byd_fps_enable_irq_wake(struct spi_device *spi)
{
#ifdef BYD_FPS_IRQ_NUM
	enable_irq_wake(BYD_FPS_IRQ_NUM);
#endif
#ifndef BYD_FPS_IRQ_NUM
	printk("%s:enable irq wake,irq num =%d\n",__func__,spi->irq);
	enable_irq_wake(spi->irq);
#endif
}

void byd_fps_disable_irq_wake(struct spi_device *spi)
{
#ifdef BYD_FPS_IRQ_NUM
	disable_irq_wake(BYD_FPS_IRQ_NUM);
#endif
#ifndef BYD_FPS_IRQ_NUM
	disable_irq_wake(spi->irq);
#endif
}
#if (defined(BYD_FPS_REPORT_KEY))&&(defined(BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED))
void byd_fps_set_gest_key_flag(unsigned char flag)
{
	byd_fps_gest_key_flag = flag;
	
}
#endif
// *******************************************************************************
// * Function    :  byd_fps_set_wakelock
// * Description :  set byd_fps_wakeup_data->byd_fps_wakeup_lock.Screen unlock mode.
// * In          :  wakelock
// * Return      :  void
// *******************************************************************************
void byd_fps_set_wakelock(char wakelock)
{
	if(byd_fps_wakeup_data == NULL) {
		return ;//return -1;
	}
	byd_fps_wakeup_data->byd_fps_wakeup_lock = wakelock;
	
	
}

char byd_fps_get_wakelock(void)
{
	if(byd_fps_wakeup_data == NULL) {
		return -1;
	}
	return byd_fps_wakeup_data->byd_fps_wakeup_lock;
}

void byd_fps_set_suspend_flag(char suspend_flag)
{
	if(byd_fps_wakeup_data == NULL) {
		return ;//return -1;
	}
	byd_fps_wakeup_data->byd_fps_suspend_flag = suspend_flag;
}

char byd_fps_get_suspend_flag(void)
{
	if(byd_fps_wakeup_data == NULL) {
		return -1;
	}
	return byd_fps_wakeup_data->byd_fps_suspend_flag;
}

void byd_fps_set_susp_match_cnt(char susp_match_cnt)
{
	byd_fps_wakeup_data->byd_fps_susp_match_cnt = susp_match_cnt;
}

char byd_fps_get_susp_match_cnt(void)
{
	return byd_fps_wakeup_data->byd_fps_susp_match_cnt;
}

//------------------------------input device register-------------------------
static int byd_fps_register_input_device(struct device *dev, struct input_dev **input_dev_p)
{
	int ret = 0;
	struct input_dev *input_dev;
	
	input_dev = input_allocate_device();
	if (!input_dev) {
		pr_err("%s: input allocate device failed\n", __FUNCTION__);
		return -ENOMEM;
	}
	printk("%s: Sean Debug -- input_allocate_device() OK!\n",__func__);
	input_dev->name = "byd_fps_vkey";
	input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = dev;
	input_dev->id.vendor = 0x0001;
	input_dev->id.product = 0x0001;
	input_dev->id.version = 0x0010;

	set_bit(EV_SYN, input_dev->evbit); // can be omitted?
	set_bit(EV_KEY, input_dev->evbit); // for button only?
	//set_bit(EV_ABS, input_dev->evbit); // must
//#if (defined(BYD_FPS_REPORT_KEY))||(defined(BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED))
#ifdef BYD_FPS_REPORT_KEY_ONLY
	set_bit(KEY_FINGERPRINT_ONLY,  input_dev->keybit);
	input_set_capability(input_dev, EV_KEY, KEY_FINGERPRINT_ONLY);
#endif
#ifdef BYD_FPS_REPORT_KEY
	set_bit(KEY_FINGERPRINT,  input_dev->keybit);
	input_set_capability(input_dev, EV_KEY, KEY_FINGERPRINT);
#endif
#if (defined(BYD_FPS_GESTURE)||defined(BYD_FPS_FOCUS))
	set_bit(KEY_ENTER,  input_dev->keybit);
	input_set_capability(input_dev, EV_KEY, KEY_ENTER);
	/////////////////////////////////////////2017.02.22 added by cgh//////////
	set_bit(KEY_BACK,  input_dev->keybit);
	input_set_capability(input_dev, EV_KEY, KEY_BACK);
	//////////////////////////////////////////////////
	set_bit(KEY_UP,  input_dev->keybit);	
	input_set_capability(input_dev, EV_KEY, KEY_UP);
	set_bit(KEY_LEFT,  input_dev->keybit);	
	input_set_capability(input_dev, EV_KEY, KEY_LEFT);
	set_bit(KEY_DOWN,  input_dev->keybit);	
	input_set_capability(input_dev, EV_KEY, KEY_DOWN);
	set_bit(KEY_RIGHT,  input_dev->keybit);	
	input_set_capability(input_dev, EV_KEY, KEY_RIGHT);
#endif
	ret = input_register_device(input_dev);
	if (ret < 0) {
		pr_err("%s: input device regist failed\n", __func__);
		//input_unregister_device(byd_input_dev);
		input_free_device(input_dev); // kfree(input_dev);
		input_dev = NULL;
	}
	
	*input_dev_p = input_dev;
	
	return ret;
}


void byd_fps_wakeup_susp_worker(void)
{
	if(!work_pending(&byd_fps_wakeup_data->byd_fps_work_susp)){
		queue_work(byd_fps_wakeup_data->byd_fps_workqueue_susp,&byd_fps_wakeup_data->byd_fps_work_susp);
	}
}
//#if (defined(BYD_FPS_REPORT_KEY))||(defined(BYD_FPS_REPORT_KEY_FOCUS_CONTROLLED))
#ifdef BYD_FPS_REPORT_KEY
void byd_fps_report_key_up(void)
{
	input_report_key(byd_input_dev, KEY_FINGERPRINT, 0);
	input_sync(byd_input_dev);
	#ifdef BYD_FPS_TEST_TIME_LOG
		printk("%s:report KEY_FINGERPRINT 0 for \n", __func__);
	#endif
}
#endif
int g_need_send_signal = 0;		
// *******************************************************************************
// * Function    :  byd_fps_intr_work
// * Description :  put work to workqueue when in interrupt service routing.
// * In          :  void
// * Return      :  void
// *******************************************************************************
static unsigned char byd_err_state=0;
int byd_fps_intr_work(void)
{
	printk("%s: byd  IN,byd_fps_finger_up_async_flag=%d\n",__func__,byd_fps_finger_up_async_flag);
	if( (byd_err_state == 0) &&(byd_fps_instr_stop_flag != 1)&&(byd_fps_int_test!=1)){
		if(g_need_send_signal == 1)
		{
			#ifdef BYD_FPS_TIMER_KEY
			byd_fps_downif();//开启timer定时
			#endif
			#ifdef BYD_FPS_FASYNC
			//异步通知上层处理解锁事物.
			byd_fps_send_fasync();
			g_need_send_signal = 0;
			return 0;
			#endif
		}
	
		//Check fng "up", then wait fng "down". 
		//Calling "byd_fps_set_suspend_flag(8)" in byd_sys_dev_ioctl() cmd=12.
		if(byd_fps_get_suspend_flag() == 8) {
			
			#ifdef BYD_FPS_UP_DOWN_ASYNC
				if(byd_fps_finger_up_async_flag == 1){
				#ifdef BYD_FPS_FASYNC
					//异步通知上层处理解锁事物.
					DBG_TIME("%s:finger up async\n",__func__);
					byd_fps_send_fasync();
				#endif
					byd_fps_finger_up_async_flag = 0;
					
				}
			#endif
			DBG_TIME("%s: byd fng up now,Suspend! \n", __func__);
			printk("%s: byd_fps_os_worker is start! Os_Scan_Expired is = %d \n", __func__, Os_Scan_Expired);
			if ((Os_Otp_or_Scan != OS_IN_OTP) && (Os_Otp_or_Scan != OS_IN_OTP_LEGACY) && (Os_Scan_Expired == 1)) {
				/*wake up thread "thread_task->wait_job" to scan OS data.*/
				byd_os_start_scan();
			}
			printk("%s: byd_fps_os_worker is end! \n", __func__);
	
			DBG_TIME("%s: byd check fng down,Suspend! \n", __func__);
			
			FP_DBG("%s:-----------byd interrupt workqueue_fng_down------\n", __func__);
			/*Put work "byd_fps_work_susp", whose working function is "byd_fps_worker_susp()", into work queue "byd_fps_workqueue_susp".*/
			if(!work_pending(&byd_fps_wakeup_data->byd_fps_work_susp)){
				queue_work(byd_fps_wakeup_data->byd_fps_workqueue_susp,&byd_fps_wakeup_data->byd_fps_work_susp);
			}
			FP_DBG("%s:-----------byd interrupt workqueue_fng_down END------\n", __func__);
			return 0;
		}
	} else if(byd_err_state != 0){
		byd_err_state = 0;
		return -1;
	}else {
		return 0;
	}
	
	#if defined(BYD_FPS_REPORT_KEY)&&defined(BYD_FPS_FOCUS)
	byd_fps_set_gest_key_flag(0);
	#endif
	
	/*Put work "byd_fps_work_key", whose working function is "byd_fps_worker_key()", into the work queue "byd_fps_workqueue_key".*/
	#ifdef BYD_FPS_GEST
	if(!work_pending(&byd_fps_wakeup_data->byd_fps_work_key)){
		queue_work(byd_fps_wakeup_data->byd_fps_workqueue_key,&byd_fps_wakeup_data->byd_fps_work_key);
	}
	#endif
	return 0;
}

extern void byd_fps_mutex_lock(unsigned char lock_flag,unsigned char type_flag);
#ifdef BYD_FPS_UP_DOWN_ASYNC
extern unsigned char g_need_finger_down_scan_flag;
#endif
static void byd_fps_worker_susp(struct work_struct *work)
{
	printk("%s: IN\n", __func__);
	if(byd_fps_wakeup_data->byd_fps_suspend_flag == 8) {
		byd_fps_wakeup_data->byd_fps_suspend_flag = 0;
		printk("%s: check fng down\n", __func__);
		byd_fps_msleep(60);
		byd_fps_mutex_lock(1,1);
		byd_fps_chip_idle(this_byd_fps->spi);//added by xzy
		//byd_fps_enable_irq_wake(this_byd_fps->spi);	//deep sleep , wake up
		byd_fps_finger_detect_pre_cfg(1,1);//按下+快速, wait fng down
		byd_fps_mutex_lock(2,1);
		g_need_send_signal = 1;
		#ifdef BYD_FPS_UP_DOWN_ASYNC
		g_need_finger_down_scan_flag = 1;
		#endif
		printk("%s: waiting fng down...\n", __func__);
	}
}
unsigned char byd_fps_get_fingeup_flag(void)
{
	return byd_fps_finger_up_flag;
}
static void byd_fps_worker_key(struct work_struct *work)
{
	unsigned char fng_state=0, intr_state=0;
	/*if(byd_last_cmd != 5)*/{
		byd_fps_mutex_lock(1,1);
		intr_state = (unsigned char)byd_fps_chip_intr_state(this_byd_fps->spi);
		fng_state = byd_fps_get_finger_state(this_byd_fps->spi);
		byd_fps_mutex_lock(2,1);
		#ifdef BYD_FPS_TEST_TIME_LOG
		printk("%s:read intr state=0x%x\n", __func__, intr_state);
		#endif
		#ifdef BYD_FPS_TEST_TIME_LOG
		printk("%s:read fng state=0x%x\n", __func__, fng_state);
		#endif
	}
	//byd_fps_wakeup_data->finger_state_done = 1;
	//wake_up_interruptible(byd_fps_wakeup_data->waiting_finger_state);
	if(((intr_state&0x04) ==0x4)&&(fng_state == 1))
	{
		byd_fps_finger_up_flag = 0;
	}
	if(((intr_state&0x04) ==0x4)&&(fng_state == 0))
	{
		byd_fps_finger_up_flag = 1;
	}
	
	if(byd_fps_chip_flag == 0x22){
		/*Added by sean, FPS22 some error checked by chip, triggering interrupt*/
		if((intr_state&0x10) ==0x10) {
			//unsigned char byd_err_state=0;
			byd_fps_mutex_lock(1,1);
			byd_err_state = byd_fps_chip_error_state22(this_byd_fps->spi);
			//Clear "REG_INT_STATE", also Clear "REG_CHECK_ERROR_STATE".
			byd_fps_chip_intr_flag_clr(this_byd_fps->spi,0);
			byd_fps_mutex_lock(2,1);

			#ifdef BYD_FPS_POWER_CTRL
			byd_fps_power_restart(this_byd_fps->spi,0);	//fng state? wait what?
			#endif
			intr_state = 0;
			fng_state = 0;
			return;
		}	
	
		/*added byd cgh to judge  SUB value is ok if not reset chip */
		if ( byd_fps_sub_value22[0] < 0xa0 && byd_fps_sub_value22[2] <0xa0 && byd_fps_sub_value22[4] <0xa0 ) {//normal value is bigger than 65535-6000=e88f
			#ifdef BYD_FPS_POWER_CTRL
			byd_fps_power_restart(this_byd_fps->spi,0);	//
			#endif
		
			return;
		}
	}
	if((byd_last_cmd != 5) &&(byd_last_cmd != 1)&&(byd_fps_instr_stop_flag != 1)&&(byd_fps_int_test!=1)){
		//intr_state = byd_fps_get_intr_state(this_byd_fps->spi);
		#ifdef BYD_FPS_TEST_TIME_LOG
		printk("%s:read intr state=0x%x\n", __func__, intr_state);
		#endif
	#ifdef BYD_FPS_TIMER_KEY
	//printk("%s:timer_flag = %d,byd_fps_keydown_flag =%d,byd_fps_keyopen_flag =%d\n",__func__, byd_fps_get_timer_flag(),byd_fps_keydown_flag,byd_fps_keyopen_flag);
	if(byd_fps_get_timer_flag() != 1)//定时时间到，或者初始化
	#endif
	{
	#ifdef BYD_FPS_POWER_CTRL
	if(((byd_fps_keyopen_flag == BYD_FPS_ONLY_FOCUS_ON)||(byd_fps_keyopen_flag == BYD_FPS_GEST_ON)) && ((intr_state&0x04) != 0x4)&&(byd_fps_keydown_flag == 0)){
		byd_fps_power_restart(this_byd_fps->spi,0);
	}
	#endif
	#ifdef BYD_FPS_FOCUS
	//滑动手势改变区域块的配置
	if((byd_fps_keydown_flag == 0)&&(byd_fps_keyopen_flag == BYD_FPS_GEST_ON)){
		
		byd_fps_gest_cfg(this_byd_fps->spi);
	}
	if((byd_fps_keydown_flag == 1)||(byd_fps_keyopen_flag == BYD_FPS_GEST_OFF)){
		
		byd_fps_gest_cfg_back(this_byd_fps->spi);
	}
	#endif
	#ifdef BYD_FPS_FOCUS
	if(((intr_state&0x04) ==0x4)&&(byd_fps_keydown_flag == 0)&&((byd_fps_keyopen_flag == BYD_FPS_ONLY_FOCUS_ON)||(byd_fps_keyopen_flag == BYD_FPS_GEST_ON))){
		if(byd_fps_finger_up_flag == 0){
			gest_int_number = 0;
			byd_fps_start_gest_cacu();//开启timer定时,读取fpd_state,
		}else if(byd_fps_finger_up_flag == 1){
			byd_fps_timer_up_stop();
			if((gest_int_number-1)>=5){
				printk("%s:finger up,time enough,caculate gesture\n",__func__);
				//byd_fps_gest_key_flag = 1;
				byd_fps_gest_judge(this_byd_fps,gest_int_number-1);
			}
			gest_int_number = 0;
		}
	}
	#endif
	#ifdef BYD_FPS_GEST
	if((byd_fps_keyopen_flag != BYD_FPS_GEST_OFF)&&(byd_fps_finger_up_flag == 1)&&(byd_fps_keydown_flag == 0)&&(byd_fps_gest_key_flag ==0)){
		
		//byd_fps_start_timer_key();
	}
	#endif
	#ifdef BYD_FPS_REPORT_KEY
		/*if((byd_fps_keydown_flag == 0)&&((byd_fps_keyopen_flag == BYD_FPS_ONLY_KEY_ON)||(byd_fps_keyopen_flag == BYD_FPS_GEST_ON))&&((intr_state&0x04) ==0x4))*/
		if((byd_fps_keydown_flag == 0)&&((intr_state&0x04) ==0x4))
		{
			
			if(byd_input_dev != NULL) {
				if((fng_state == 1)&&(byd_fps_gest_key_flag == 0)) {
					input_report_key(byd_input_dev, KEY_FINGERPRINT, 1);
					input_sync(byd_input_dev);
					byd_fps_key_mesg = 1;
				#ifdef BYD_FPS_TEST_TIME_LOG
					printk("%s:report KEY_FINGERPRINT 1\n", __func__);
				#endif
					
				}else{
					if((fng_state == 0)&&(byd_fps_key_mesg == 1)&&(byd_fps_gest_key_flag == 0)) {
						input_report_key(byd_input_dev, KEY_FINGERPRINT, 0);
						input_sync(byd_input_dev);
						byd_fps_key_mesg = 2;
					#ifdef BYD_FPS_TEST_TIME_LOG
					printk("%s:report KEY_FINGERPRINT 0\n", __func__);
					#endif
					}
				}
				#ifdef BYD_FPS_POWER_CTRL
				byd_fps_power_restart(this_byd_fps->spi,byd_fps_key_mesg);
				#endif
			}
			
		}
	#endif
	}
	}
}


int byd_fps_qual_suspend(void)
{
	if (byd_fps_wakeup_data->byd_fps_suspend_qual == 1) {//休眠时进行指令判断的标志
		if (byd_fps_wakeup_data->byd_fps_suspend_flag == 0) {
			return 1;//表示立刻返回，有了优先级更高的操作
		}
	}
	return 0;
}
#if defined(CONFIG_HAS_EARLYSUSPEND)
#define BYD_SUSPEND_LEVEL 1
static void byd_fps_early_suspend(struct early_suspend *handler)
{
	printk("%s, byd no 5\n", __func__);
	byd_fps_keydown_flag = 1;
}

// *******************************************************************************
// * Function    :  byd_fps_late_resume
// * Description :  disalbe system wakeup interrupt and fps entry the sleep mode
// * In          :  handler - dummy variable
// * Return      :  none
// *******************************************************************************
static void byd_fps_late_resume(struct early_suspend *handler)
{
	printk("%s, byd no 5\n", __func__);
	byd_fps_keydown_flag = 0;
	//byd_fps_resume(&this_byd_fps->spi->dev);
}
#endif
// *******************************************************************************
// * Function    :  byd_fps_wakeup_init
// * Description :  malloc/init variable, init work, create work queue, register early_suspend.
// * In          :  dev  :	pointer to device
// * Return      :  0 -- finished,
// *******************************************************************************
int byd_fps_wakeup_init(struct device *dev)
{
	int ret = -1;
	
	byd_fps_wakeup_data = devm_kzalloc(dev, sizeof(*byd_fps_wakeup_data), GFP_KERNEL);
	if (!byd_fps_wakeup_data)	{
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	byd_fps_wakeup_data->byd_fps_wakeup_lock = 2;	//0--默认关闭指纹唤醒系统的功能。
	byd_fps_wakeup_data->byd_fps_suspend_flag = 0;
	byd_fps_wakeup_data->byd_fps_susp_match_cnt = 0;
	byd_fps_wakeup_data->byd_fps_suspend_qual = 0;
	
	INIT_WORK(&byd_fps_wakeup_data->byd_fps_work_susp, byd_fps_worker_susp);
	byd_fps_wakeup_data->byd_fps_workqueue_susp = create_workqueue(BYD_FPS_WORKQUEUE_NAME_SUSP);//创建新的工作队列
	
	
	#if defined(CONFIG_HAS_EARLYSUSPEND) //&& 0
	//EARLY_SUSPEND_LEVEL_BLANK_SCREEN (50), EARLY_SUSPEND_LEVEL_STOP_DRAWING (100), EARLY_SUSPEND_LEVEL_DISABLE_FB (150)
	byd_fps_wakeup_data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + BYD_SUSPEND_LEVEL;
	byd_fps_wakeup_data->early_suspend.suspend = byd_fps_early_suspend;
	byd_fps_wakeup_data->early_suspend.resume = byd_fps_late_resume;
	register_early_suspend(&byd_fps_wakeup_data->early_suspend);
	#endif
	//byd_fps_wakeup_data->finger_state_done = 0;
	//byd_fps_wakeup_data->waiting_finger_state = kzalloc(sizeof(*byd_fps_wakeup_data->waiting_finger_state), GFP_KERNEL);
	//init_waitqueue_head(byd_fps_wakeup_data->waiting_finger_state);
	INIT_WORK(&byd_fps_wakeup_data->byd_fps_work_key, byd_fps_worker_key);
	byd_fps_wakeup_data->byd_fps_workqueue_key = create_workqueue(BYD_FPS_WORKQUEUE_NAME_KEY);//创建新的工作队列
	byd_fps_enable_irq_wake(this_byd_fps->spi);
	
 	printk("%s: register_input_device\n", __func__);
	ret = byd_fps_register_input_device(dev, &byd_input_dev);
	if (ret < 0) {
		byd_fps_wakeup_exit(dev);
		return ret;
	}
	printk("%s: Sean Debug -- byd_fps_register_input_device() OK!\n",__func__);

	return 0;
}



// *******************************************************************************
// * Function    :  byd_fps_wakeup_exit
// * Description :  unregister,in byd_fps_remove()
// * In          :  dev - (supposed to be spi device)
// * Return      :  void
// *******************************************************************************
void byd_fps_wakeup_exit(struct device *dev)
{

	if (byd_input_dev) {
		input_unregister_device(byd_input_dev);
		input_free_device(byd_input_dev); //kfree(byd_input_dev);
	}
	destroy_workqueue(byd_fps_wakeup_data->byd_fps_workqueue_susp);
	destroy_workqueue(byd_fps_wakeup_data->byd_fps_workqueue_key);
	//kfree(byd_fps_wakeup_data->waiting_finger_state);
	devm_kfree(dev, byd_fps_wakeup_data);
	
}
