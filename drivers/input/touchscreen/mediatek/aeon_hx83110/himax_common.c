/* Himax Android Driver Sample Code for common functions
*
* Copyright (C) 2017 Himax Corporation.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

#include "himax_common.h"
#include "himax_ic.h"

#define SUPPORT_FINGER_DATA_CHECKSUM 0x0F
#define TS_WAKE_LOCK_TIMEOUT		(2 * HZ)
#define FRAME_COUNT 5

#ifdef HX_RST_PIN_FUNC
extern void himax_ic_reset(uint8_t loadconfig,uint8_t int_off);
#endif

#if defined(HX_AUTO_UPDATE_FW)
	unsigned char i_CTPM_FW[]=
	{
		#include "20170511_HX83110-A_HUASHI_CFG0101.i"
	};
#endif

extern void HX_report_ESD_event(void);

//static int		tpd_keys_local[HX_KEY_MAX_COUNT] = HX_KEY_ARRAY; // for Virtual key array

struct himax_ts_data *private_ts;
struct himax_ic_data *ic_data;
struct himax_report_data *hx_touch_data;

static int		HX_TOUCH_INFO_POINT_CNT   = 0;

unsigned long	FW_VER_MAJ_FLASH_ADDR;
unsigned long 	FW_VER_MIN_FLASH_ADDR;
unsigned long 	CFG_VER_MAJ_FLASH_ADDR;
unsigned long 	CFG_VER_MIN_FLASH_ADDR;

unsigned long 	FW_VER_MAJ_FLASH_LENG;
unsigned long 	FW_VER_MIN_FLASH_LENG;
unsigned long 	CFG_VER_MAJ_FLASH_LENG;
unsigned long 	CFG_VER_MIN_FLASH_LENG;

unsigned long 	FW_CFG_VER_FLASH_ADDR;

#ifdef HX_AUTO_UPDATE_FW
	int g_i_FW_VER = 0;
	int g_i_CFG_VER = 0;
#endif

unsigned char	IC_TYPE = 11;
unsigned char	IC_CHECKSUM = 0;

#ifdef HX_ESD_RECOVERY
	u8 HX_ESD_RESET_ACTIVATE = 0;
#endif
	u8 	HX_HW_RESET_ACTIVATE = 0;

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
extern int himax_touch_proc_init(void);
extern void himax_touch_proc_deinit(void);
//PROC-START
#ifdef  HX_TP_PROC_FLASH_DUMP
extern void	himax_ts_flash_func(void);
extern void setFlashBuffer(void);
extern bool getFlashDumpGoing(void);
extern uint8_t getSysOperation(void);
extern void setSysOperation(uint8_t operation);
#endif

#if defined(HX_PLATFOME_DEFINE_KEY)
extern void	himax_platform_key(void);
#endif

#ifdef HX_TP_PROC_DIAG
	extern void	himax_ts_diag_func(void);

	extern int16_t *getMutualBuffer(void);
	extern int16_t *getMutualNewBuffer(void);
	extern int16_t *getMutualOldBuffer(void);
	extern int16_t *getSelfBuffer(void);
	extern uint8_t getXChannel(void);
	extern uint8_t getYChannel(void);
	extern uint8_t getDiagCommand(void);
	extern void setXChannel(uint8_t x);
	extern void setYChannel(uint8_t y);
	extern void setMutualBuffer(void);
	extern void setMutualNewBuffer(void);
	extern void setMutualOldBuffer(void);
	extern uint8_t	coordinate_dump_enable;
	extern struct file	*coordinate_fn;
	extern uint8_t diag_coor[128];
	extern int himax_set_diag_cmd(struct himax_ic_data *ic_data,struct himax_report_data *hx_touch_data);
#ifdef HX_TP_PROC_2T2R
	extern int16_t *getMutualBuffer_2(void);
	extern uint8_t getXChannel_2(void);
	extern uint8_t getYChannel_2(void);
	extern void setXChannel_2(uint8_t x);
	extern void setYChannel_2(uint8_t y);
	extern void setMutualBuffer_2(void);
#endif
#endif
//PROC-END
#endif

extern int himax_parse_dt(struct himax_ts_data *ts,
				struct himax_i2c_platform_data *pdata);
extern bool himax_calculateChecksum(struct i2c_client *client, bool change_iref);

static uint8_t 	vk_press = 0x00;
static uint8_t 	AA_press = 0x00;
static uint8_t 	EN_NoiseFilter = 0x00;
static uint8_t	Last_EN_NoiseFilter = 0x00;
static int	hx_point_num	= 0;																	// for himax_ts_work_func use
static int	p_point_num	= 0xFFFF;
static int	tpd_key	   	= 0x00;
static int	tpd_key_old	= 0x00;
static int	probe_fail_flag	= 0;
#ifdef HX_USB_DETECT_GLOBAL
bool USB_detect_flag = 0;
#endif

#if defined(CONFIG_FB)
int fb_notifier_callback(struct notifier_block *self,
				 unsigned long event, void *data);
#elif defined(CONFIG_HAS_EARLYSUSPEND)
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

#if defined(HX_LOADIN_CONFIG)
	bool config_load 	= false;
	struct himax_config *config_selected = NULL;
#endif

#if defined(HX_PALM_REPORT)
	int himax_palm_detect(uint8_t *buf);
#endif

#ifdef HX_GESTURE_TRACK
static int gest_pt_cnt;
static int gest_pt_x[GEST_PT_MAX_NUM];
static int gest_pt_y[GEST_PT_MAX_NUM];
static int gest_start_x=0,gest_start_y=0,gest_end_x=0,gest_end_y=0;
static int gest_width=0,gest_height=0,gest_mid_x=0,gest_mid_y=0;
static int gn_gesture_coor[16];
#endif

#ifdef HX_CHIP_STATUS_MONITOR
struct chip_monitor_data *g_chip_monitor_data;
#endif

#if defined(HX_TP_PROC_SELF_TEST)
extern int g_self_test_entered;
#endif

int himax_report_data_init(void);
extern int himax_get_touch_data_size(void);
//void himax_HW_reset(uint8_t loadconfig,uint8_t int_off);
extern void himax_log_touch_data(uint8_t *buf,struct himax_report_data *hx_touch_data);
extern void himax_log_touch_event(int x,int y,int w,int loop_i,uint8_t EN_NoiseFilter,int touched);
extern void himax_log_touch_event_detail(struct himax_ts_data *ts,int x,int y,int w,int loop_i,uint8_t EN_NoiseFilter,int touched,uint16_t old_finger);

#if defined(HX_ESD_RECOVERY)
extern void himax_esd_ic_reset(void);
extern int himax_ic_esd_recovery(int hx_esd_event,int hx_zero_event,int length);
#endif

extern int himax_dev_set(struct himax_ts_data *ts);

int himax_input_register(struct himax_ts_data *ts)
{
	int ret = 0;

	ret = himax_dev_set(ts);
	if(ret < 0)
		goto input_device_fail;
	
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);

#if defined(HX_PLATFOME_DEFINE_KEY)
	himax_platform_key();
#else
	set_bit(KEY_BACK, ts->input_dev->keybit);
	set_bit(KEY_HOME, ts->input_dev->keybit);
	set_bit(KEY_MENU, ts->input_dev->keybit);
	set_bit(KEY_SEARCH, ts->input_dev->keybit);
#endif
#if defined(HX_SMART_WAKEUP) || defined(HX_PALM_REPORT)
	set_bit(KEY_POWER, ts->input_dev->keybit);
#endif
#if defined(HX_SMART_WAKEUP)
	set_bit(KEY_CUST_01, ts->input_dev->keybit);
	set_bit(KEY_CUST_02, ts->input_dev->keybit);
	set_bit(KEY_CUST_03, ts->input_dev->keybit);
	set_bit(KEY_CUST_04, ts->input_dev->keybit);
	set_bit(KEY_CUST_05, ts->input_dev->keybit);
	set_bit(KEY_CUST_06, ts->input_dev->keybit);
	set_bit(KEY_CUST_07, ts->input_dev->keybit);
	set_bit(KEY_CUST_08, ts->input_dev->keybit);
	set_bit(KEY_CUST_09, ts->input_dev->keybit);
	set_bit(KEY_CUST_10, ts->input_dev->keybit);
	set_bit(KEY_CUST_11, ts->input_dev->keybit);
	set_bit(KEY_CUST_12, ts->input_dev->keybit);
	set_bit(KEY_CUST_13, ts->input_dev->keybit);
	set_bit(KEY_CUST_14, ts->input_dev->keybit);
	set_bit(KEY_CUST_15, ts->input_dev->keybit);
#endif
	set_bit(BTN_TOUCH, ts->input_dev->keybit);

	//set_bit(KEY_APP_SWITCH, ts->input_dev->keybit);

	set_bit(INPUT_PROP_DIRECT, ts->input_dev->propbit);

	if (ts->protocol_type == PROTOCOL_TYPE_A) {
		//ts->input_dev->mtsize = ts->nFinger_support;
		input_set_abs_params(ts->input_dev, ABS_MT_TRACKING_ID,
		0, 3, 0, 0);
	} else {/* PROTOCOL_TYPE_B */
		set_bit(MT_TOOL_FINGER, ts->input_dev->keybit);
		input_mt_init_slots(ts->input_dev, ts->nFinger_support,0);
	}

	I("input_set_abs_params: mix_x %d, max_x %d, min_y %d, max_y %d\n",
		ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_y_min, ts->pdata->abs_y_max);

	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X,ts->pdata->abs_x_min, ts->pdata->abs_x_max, ts->pdata->abs_x_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y,ts->pdata->abs_y_min, ts->pdata->abs_y_max, ts->pdata->abs_y_fuzz, 0);
	input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
	//input_set_abs_params(ts->input_dev, ABS_MT_PRESSURE,ts->pdata->abs_pressure_min, ts->pdata->abs_pressure_max, ts->pdata->abs_pressure_fuzz, 0);
//	input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR,ts->pdata->abs_width_min, ts->pdata->abs_width_max, ts->pdata->abs_pressure_fuzz, 0);

//	input_set_abs_params(ts->input_dev, ABS_MT_AMPLITUDE, 0, ((ts->pdata->abs_pressure_max << 16) | ts->pdata->abs_width_max), 0, 0);
//	input_set_abs_params(ts->input_dev, ABS_MT_POSITION, 0, (BIT(31) | (ts->pdata->abs_x_max << 16) | ts->pdata->abs_y_max), 0, 0);


	return NO_ERR;
	
input_device_fail:
	I("%s, input device register fail!\n",__func__);
	return ret;
}

static void calcDataSize(uint8_t finger_num)
{
	struct himax_ts_data *ts_data = private_ts;
	ts_data->coord_data_size = 4 * finger_num;
	ts_data->area_data_size = ((finger_num / 4) + (finger_num % 4 ? 1 : 0)) * 4;
	ts_data->coordInfoSize = ts_data->coord_data_size + ts_data->area_data_size + 4;
	ts_data->raw_data_frame_size = 128 - ts_data->coord_data_size - ts_data->area_data_size - 4 - 4 - 1;
	if(ts_data->raw_data_frame_size == 0)
		{
			E("%s: could NOT calculate! \n", __func__);
			return;
		}
	ts_data->raw_data_nframes  = ((uint32_t)ts_data->x_channel * ts_data->y_channel +
									ts_data->x_channel + ts_data->y_channel) / ts_data->raw_data_frame_size +
									(((uint32_t)ts_data->x_channel * ts_data->y_channel +
									ts_data->x_channel + ts_data->y_channel) % ts_data->raw_data_frame_size)? 1 : 0;
	I("%s: coord_data_size: %d, area_data_size:%d, raw_data_frame_size:%d, raw_data_nframes:%d", __func__, ts_data->coord_data_size, ts_data->area_data_size, ts_data->raw_data_frame_size, ts_data->raw_data_nframes);
}

void calculate_point_number(void)
{
	HX_TOUCH_INFO_POINT_CNT = ic_data->HX_MAX_PT * 4 ;

	if ( (ic_data->HX_MAX_PT % 4) == 0)
		HX_TOUCH_INFO_POINT_CNT += (ic_data->HX_MAX_PT / 4) * 4 ;
	else
		HX_TOUCH_INFO_POINT_CNT += ((ic_data->HX_MAX_PT / 4) +1) * 4 ;
}

#ifdef HX_AUTO_UPDATE_FW
static int i_update_FW(void)
{
	int upgrade_times = 0;
	unsigned char* ImageBuffer = i_CTPM_FW;
	int fullFileLength = sizeof(i_CTPM_FW);
	uint8_t ret = 0, result = 0;

	I("%s: i_fullFileLength = %d\n", __func__,fullFileLength);

			himax_int_enable(private_ts->client->irq,0);
update_retry:
			if(fullFileLength == FW_SIZE_32k){
				ret = fts_ctpm_fw_upgrade_with_sys_fs_32k(private_ts->client,ImageBuffer,fullFileLength,false);
			}else if(fullFileLength == FW_SIZE_60k){
				ret = fts_ctpm_fw_upgrade_with_sys_fs_60k(private_ts->client,ImageBuffer,fullFileLength,false);
			}else if (fullFileLength == FW_SIZE_64k){
				ret = fts_ctpm_fw_upgrade_with_sys_fs_64k(private_ts->client,ImageBuffer,fullFileLength,false);
			}else if (fullFileLength == FW_SIZE_124k){
				ret = fts_ctpm_fw_upgrade_with_sys_fs_124k(private_ts->client,ImageBuffer,fullFileLength,false);
			}else if (fullFileLength == FW_SIZE_128k){
				ret = fts_ctpm_fw_upgrade_with_sys_fs_128k(private_ts->client,ImageBuffer,fullFileLength,false);
			}
			if(ret == 0){
				upgrade_times++;
				E("%s: TP upgrade error, upgrade_times = %d\n", __func__, upgrade_times);
				if(upgrade_times < 3)
					goto update_retry;
				else
					result = -1;//upgrade fail
			}
			else{
				ic_data->vendor_fw_ver = g_i_FW_VER;
				ic_data->vendor_config_ver = g_i_CFG_VER;
				result = 1;//upgrade success
				I("%s: TP upgrade OK\n", __func__);
			}
#ifdef HX_RST_PIN_FUNC
			himax_ic_reset(true,false);
#endif
			himax_int_enable(private_ts->client->irq,1);
			return result;
}
#endif

#if defined(HX_LOADIN_CONFIG)
static int himax_parse_config(struct himax_ts_data *ts, struct himax_config *pdata)
{
	struct himax_config *cfg_table;
	struct device_node *node, *pp = NULL;
	struct property *prop;
	uint8_t cnt = 0, i = 0;
	u32 data = 0;
	uint32_t coords[4] = {0};
	int len = 0;
	char str[6]={0};

	node = ts->client->dev.of_node;
	if (node == NULL) {
		E(" %s, can't find device_node", __func__);
		return -ENODEV;
	}

	while ((pp = of_get_next_child(node, pp)))
		cnt++;

	if (!cnt)
	{
		E(" %s,cnt ", __func__);
		return -ENODEV;
	}

	cfg_table = kzalloc(cnt * (sizeof *cfg_table), GFP_KERNEL);
	if (!cfg_table)
		return -ENOMEM;

	pp = NULL;
	while ((pp = of_get_next_child(node, pp))) {
		if (of_property_read_u32(pp, "default_cfg", &data) == 0)
			cfg_table[i].default_cfg = data;

		if (of_property_read_u32(pp, "sensor_id", &data) == 0)
			cfg_table[i].sensor_id = (data);

		if (of_property_read_u32(pp, "fw_ver_main", &data) == 0)
			cfg_table[i].fw_ver_main = data;

		if (of_property_read_u32(pp, "fw_ver_minor", &data) == 0)
			cfg_table[i].fw_ver_minor = data;

		if (of_property_read_u32_array(pp, "himax,tw-coords", coords, 4) == 0) {
			cfg_table[i].tw_x_min = coords[0], cfg_table[i].tw_x_max = coords[1];	//x
			cfg_table[i].tw_y_min = coords[2], cfg_table[i].tw_y_max = coords[3];	//y
		}

		if (of_property_read_u32_array(pp, "himax,pl-coords", coords, 4) == 0) {
			cfg_table[i].pl_x_min = coords[0], cfg_table[i].pl_x_max = coords[1];	//x
			cfg_table[i].pl_y_min = coords[2], cfg_table[i].pl_y_max = coords[3];	//y
		}

		prop = of_find_property(pp, "c1", &len);
		if ((!prop)||(!len)) {
			strcpy(str,"c1");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c1, prop->value, len);
		prop = of_find_property(pp, "c2", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c2");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c2, prop->value, len);
		prop = of_find_property(pp, "c3", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c3");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c3, prop->value, len);
		prop = of_find_property(pp, "c4", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c4");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c4, prop->value, len);
		prop = of_find_property(pp, "c5", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c5");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c5, prop->value, len);
		prop = of_find_property(pp, "c6", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c6");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c6, prop->value, len);
		prop = of_find_property(pp, "c7", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c7");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c7, prop->value, len);
		prop = of_find_property(pp, "c8", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c8");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c8, prop->value, len);
		prop = of_find_property(pp, "c9", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c9");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c9, prop->value, len);
		prop = of_find_property(pp, "c10", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c10");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c10, prop->value, len);
		prop = of_find_property(pp, "c11", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c11");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c11, prop->value, len);
		prop = of_find_property(pp, "c12", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c12");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c12, prop->value, len);
		prop = of_find_property(pp, "c13", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c13");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c13, prop->value, len);
		prop = of_find_property(pp, "c14", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c14");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c14, prop->value, len);
		prop = of_find_property(pp, "c15", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c15");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c15, prop->value, len);
		prop = of_find_property(pp, "c16", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c16");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c16, prop->value, len);
		prop = of_find_property(pp, "c17", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c17");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c17, prop->value, len);
		prop = of_find_property(pp, "c18", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c18");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c18, prop->value, len);
		prop = of_find_property(pp, "c19", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c19");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c19, prop->value, len);
		prop = of_find_property(pp, "c20", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c20");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c20, prop->value, len);
		prop = of_find_property(pp, "c21", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c21");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c21, prop->value, len);
		prop = of_find_property(pp, "c22", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c22");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c22, prop->value, len);
		prop = of_find_property(pp, "c23", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c23");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c23, prop->value, len);
		prop = of_find_property(pp, "c24", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c24");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c24, prop->value, len);
		prop = of_find_property(pp, "c25", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c25");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c25, prop->value, len);
		prop = of_find_property(pp, "c26", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c26");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c26, prop->value, len);
		prop = of_find_property(pp, "c27", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c27");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c27, prop->value, len);
		prop = of_find_property(pp, "c28", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c28");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c28, prop->value, len);
		prop = of_find_property(pp, "c29", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c29");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c29, prop->value, len);
		prop = of_find_property(pp, "c30", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c30");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c30, prop->value, len);
		prop = of_find_property(pp, "c31", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c31");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c31, prop->value, len);
		prop = of_find_property(pp, "c32", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c32");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c32, prop->value, len);
		prop = of_find_property(pp, "c33", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c33");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c33, prop->value, len);
		prop = of_find_property(pp, "c34", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c34");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c34, prop->value, len);
		prop = of_find_property(pp, "c35", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c35");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c35, prop->value, len);
		prop = of_find_property(pp, "c36", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c36");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c36, prop->value, len);
		prop = of_find_property(pp, "c37", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c37");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c37, prop->value, len);
		prop = of_find_property(pp, "c38", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c38");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c38, prop->value, len);
		prop = of_find_property(pp, "c39", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c39");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c39, prop->value, len);
		prop = of_find_property(pp, "c40", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c40");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c40, prop->value, len);
#if 1
		I(" config version=[%02x]", cfg_table[i].c40[1]);
#endif
		prop = of_find_property(pp, "c41", &len);
		if ((!prop)||(!len)) {
			strcpy(str, "c41");
			goto of_find_property_error;
			}
		memcpy(cfg_table[i].c41, prop->value, len);

#if 1
		I(" DT#%d-def_cfg:%d,id:%05x, FW:%x.%x, len:%d,", i,
			cfg_table[i].default_cfg, cfg_table[i].sensor_id,
			cfg_table[i].fw_ver_main, cfg_table[i].fw_ver_minor, cfg_table[i].length);
		//I(" pl=[x_m%02d y_M%04d]\n", cfg_table[i].pl_x_min, cfg_table[i].pl_y_max);
#endif
		i++;
	of_find_property_error:
	if (!prop) {
		D(" %s:Looking up %s property in node %s failed",
			__func__, str, pp->full_name);
		return -ENODEV;
	} else if (!len) {
		D(" %s:Invalid length of configuration data in %s\n",
			__func__, str);
		return -EINVAL;
		}
	}

	i = 0;	//confirm which config we should load
	while ((ic_data->vendor_fw_ver)<
		(cfg_table[i].fw_ver_main << 8 | cfg_table[i].fw_ver_minor)) {
		i++;
	}
	if(cfg_table[i].default_cfg!=0)
		goto startloadconf;
	while (cfg_table[i].sensor_id > 0 && (cfg_table[i].sensor_id !=  ic_data->vendor_sensor_id)) {
		I(" id:%#x!=%#x, (i++)",cfg_table[i].sensor_id, ic_data->vendor_sensor_id);
		i++;
	}
	startloadconf:
	if (i <= cnt) {
		I(" DT-%s cfg idx(%d) in cnt(%d)", __func__, i, cnt);
		pdata->fw_ver_main  = cfg_table[i].fw_ver_main;
		pdata->fw_ver_minor = cfg_table[i].fw_ver_minor;
		pdata->sensor_id      	= cfg_table[i].sensor_id;

		memcpy(pdata->c1, cfg_table[i].c1,sizeof(pdata->c1));
		memcpy(pdata->c2, cfg_table[i].c2,sizeof(pdata->c2));
		memcpy(pdata->c3, cfg_table[i].c3,sizeof(pdata->c3));
		memcpy(pdata->c4, cfg_table[i].c4,sizeof(pdata->c4));
		memcpy(pdata->c5, cfg_table[i].c5,sizeof(pdata->c5));
		memcpy(pdata->c6, cfg_table[i].c6,sizeof(pdata->c6));
		memcpy(pdata->c7, cfg_table[i].c7,sizeof(pdata->c7));
		memcpy(pdata->c8, cfg_table[i].c8,sizeof(pdata->c8));
		memcpy(pdata->c9, cfg_table[i].c9,sizeof(pdata->c9));
		memcpy(pdata->c10, cfg_table[i].c10,sizeof(pdata->c10));
		memcpy(pdata->c11, cfg_table[i].c11,sizeof(pdata->c11));
		memcpy(pdata->c12, cfg_table[i].c12,sizeof(pdata->c12));
		memcpy(pdata->c13, cfg_table[i].c13,sizeof(pdata->c13));
		memcpy(pdata->c14, cfg_table[i].c14,sizeof(pdata->c14));
		memcpy(pdata->c15, cfg_table[i].c15,sizeof(pdata->c15));
		memcpy(pdata->c16, cfg_table[i].c16,sizeof(pdata->c16));
		memcpy(pdata->c17, cfg_table[i].c17,sizeof(pdata->c17));
		memcpy(pdata->c18, cfg_table[i].c18,sizeof(pdata->c18));
		memcpy(pdata->c19, cfg_table[i].c19,sizeof(pdata->c19));
		memcpy(pdata->c20, cfg_table[i].c20,sizeof(pdata->c20));
		memcpy(pdata->c21, cfg_table[i].c21,sizeof(pdata->c21));
		memcpy(pdata->c22, cfg_table[i].c22,sizeof(pdata->c22));
		memcpy(pdata->c23, cfg_table[i].c23,sizeof(pdata->c23));
		memcpy(pdata->c24, cfg_table[i].c24,sizeof(pdata->c24));
		memcpy(pdata->c25, cfg_table[i].c25,sizeof(pdata->c25));
		memcpy(pdata->c26, cfg_table[i].c26,sizeof(pdata->c26));
		memcpy(pdata->c27, cfg_table[i].c27,sizeof(pdata->c27));
		memcpy(pdata->c28, cfg_table[i].c28,sizeof(pdata->c28));
		memcpy(pdata->c29, cfg_table[i].c29,sizeof(pdata->c29));
		memcpy(pdata->c30, cfg_table[i].c30,sizeof(pdata->c30));
		memcpy(pdata->c31, cfg_table[i].c31,sizeof(pdata->c31));
		memcpy(pdata->c32, cfg_table[i].c32,sizeof(pdata->c32));
		memcpy(pdata->c33, cfg_table[i].c33,sizeof(pdata->c33));
		memcpy(pdata->c34, cfg_table[i].c34,sizeof(pdata->c34));
		memcpy(pdata->c35, cfg_table[i].c35,sizeof(pdata->c35));
		memcpy(pdata->c36, cfg_table[i].c36,sizeof(pdata->c36));
		memcpy(pdata->c37, cfg_table[i].c37,sizeof(pdata->c37));
		memcpy(pdata->c38, cfg_table[i].c38,sizeof(pdata->c38));
		memcpy(pdata->c39, cfg_table[i].c39,sizeof(pdata->c39));
		memcpy(pdata->c40, cfg_table[i].c40,sizeof(pdata->c40));
		memcpy(pdata->c41, cfg_table[i].c41,sizeof(pdata->c41));

		ts->tw_x_min = cfg_table[i].tw_x_min, ts->tw_x_max = cfg_table[i].tw_x_max;	//x
		ts->tw_y_min = cfg_table[i].tw_y_min, ts->tw_y_max = cfg_table[i].tw_y_max;	//y

		ts->pl_x_min = cfg_table[i].pl_x_min, ts->pl_x_max = cfg_table[i].pl_x_max;	//x
		ts->pl_y_min = cfg_table[i].pl_y_min, ts->pl_y_max = cfg_table[i].pl_y_max;	//y

		I(" DT#%d-def_cfg:%d,id:%05x, FW:%x.%x, len:%d,", i,
			cfg_table[i].default_cfg, cfg_table[i].sensor_id,
			cfg_table[i].fw_ver_main, cfg_table[i].fw_ver_minor, cfg_table[i].length);
		I(" DT-%s:tw-coords = %d, %d, %d, %d\n", __func__, ts->tw_x_min,
				ts->tw_x_max, ts->tw_y_min, ts->tw_y_max);
		I(" DT-%s:pl-coords = %d, %d, %d, %d\n", __func__, ts->pl_x_min,
				ts->pl_x_max, ts->pl_y_min, ts->pl_y_max);
		I(" config version=[%02x]", pdata->c40[1]);
	} else {
		E(" DT-%s cfg idx(%d) > cnt(%d)", __func__, i, cnt);
		return -EINVAL;
	}
	return 0;
}

int himax_load_config_from_system(struct i2c_client *client, struct himax_i2c_platform_data *pdata, bool config_load, struct himax_config *config_selected)
{
#if defined(HX_LOADIN_CONFIG)
	int rc= 0;
#ifndef CONFIG_OF
	int i = 0;
#endif
#endif

#if !defined(CONFIG_OF)
		pdata = client->dev.platform_data;
			if (!pdata) {
				E("%s: Necessary parameters pdata are null!\n", __func__);
				return -1;
			}
#endif

#ifdef CONFIG_OF
		//I("%s, config_selected, %X\n", __func__ ,(uint32_t)config_selected);
		if(config_load == false)
			{
				rc = himax_parse_config(private_ts, config_selected);
				if (rc < 0) {
					E(" DT:cfg table parser FAIL. ret=%d\n", rc);
					goto HimaxErr;
				} else if (rc == 0){
					if ((private_ts->tw_x_max)&&(private_ts->tw_y_max))
						{
							pdata->abs_x_min = private_ts->tw_x_min;
							pdata->abs_x_max = private_ts->tw_x_max;
							pdata->abs_y_min = private_ts->tw_y_min;
							pdata->abs_y_max = private_ts->tw_y_max;
							I(" DT-%s:config-panel-coords = %d, %d, %d, %d\n", __func__, pdata->abs_x_min,
							pdata->abs_x_max, pdata->abs_y_min, pdata->abs_y_max);
						}
					if ((private_ts->pl_x_max)&&(private_ts->pl_y_max))
						{
							pdata->screenWidth = private_ts->pl_x_max;
							pdata->screenHeight= private_ts->pl_y_max;
							I(" DT-%s:config-display-coords = (%d, %d)", __func__, pdata->screenWidth,
							pdata->screenHeight);
						}
					config_load = true;
					I(" DT parser Done\n");
					}
			}
#else
		I("pdata->hx_config_size=%x.\n",(pdata->hx_config_size+1));
		I("config_type_size=%x.\n",sizeof(struct himax_config));

		if (pdata->hx_config)
		{
			for (i = 0; i < pdata->hx_config_size/sizeof(struct himax_config); ++i) {
			I("(pdata->hx_config)[%x].fw_ver_main=%x.\n",i,(pdata->hx_config)[i].fw_ver_main);
			I("(pdata->hx_config)[%x].fw_ver_minor=%x.\n",i,(pdata->hx_config)[i].fw_ver_minor);
			I("(pdata->hx_config)[%x].sensor_id=%x.\n",i,(pdata->hx_config)[i].sensor_id);

				if ((private_ts->vendor_fw_ver_H << 8 | private_ts->vendor_fw_ver_L)<
					((pdata->hx_config)[i].fw_ver_main << 8 | (pdata->hx_config)[i].fw_ver_minor)) {
					continue;
				}else{
					if ((private_ts->vendor_sensor_id == (pdata->hx_config)[i].sensor_id)) {
						config_selected = &((pdata->hx_config)[i]);
						I("hx_config selected, %X\n", (uint32_t)config_selected);
						config_load = true;
						break;
					}
					else if ((pdata->hx_config)[i].default_cfg) {
						I("default_cfg detected.\n");
						config_selected = &((pdata->hx_config)[i]);
						I("hx_config selected, %X\n", (uint32_t)config_selected);
						config_load = true;
						break;
					}
				}
			}
		}
		else
		{
			E("[HimaxError] %s pdata->hx_config is not exist \n",__func__);
			goto HimaxErr;
		}
#endif

	return config_load;
HimaxErr:
	return -1;

}
#endif

int himax_loadSensorConfig(struct i2c_client *client, struct himax_i2c_platform_data *pdata)
{

	if (!client) {
		E("%s: Necessary parameters client are null!\n", __func__);
		return -1;
	}
#if defined(HX_LOADIN_CONFIG)

	if(config_load == false)
		{
			config_selected = kzalloc(sizeof(*config_selected), GFP_KERNEL);
			if (config_selected == NULL) {
				E("%s: alloc config_selected fail!\n", __func__);
				return -1;
			}
		}
	config_load = himax_load_config_from_system(client,pdata,config_load,config_selected);
	if(config_load == true)
		himax_write_config_into_chip(client,config_selected);

	kfree(config_selected);
	config_selected = NULL;
#endif

	I("%s: initialization complete\n", __func__);
	
	return NO_ERR;
}

#if defined(HX_SMART_WAKEUP)||defined(HX_HIGH_SENSE)||defined(HX_USB_DETECT_GLOBAL)
static void himax_resend_cmd_func(bool suspended)
{
	struct himax_ts_data *ts;

	ts = private_ts;

#ifdef HX_SMART_WAKEUP
	himax_set_SMWP_enable(ts->client,ts->SMWP_enable,suspended);
#endif
#ifdef HX_HIGH_SENSE
	himax_set_HSEN_enable(ts->client,ts->HSEN_enable,suspended);
#endif
#ifdef HX_USB_DETECT_GLOBAL
	himax_cable_detect_func(true);
#endif
}
#endif

#ifdef HX_ESD_RECOVERY
void himax_esd_hw_reset(void)
{
	I("START_Himax TP: ESD - Reset\n");	
#if defined(HX_TP_PROC_SELF_TEST)
	if (g_self_test_entered == 1)
	{
		I("In self test ,not  TP: ESD - Reset\n");
		return;
	}
	#endif
	HX_ESD_RESET_ACTIVATE = 1;
#if defined(HX_CHIP_STATUS_MONITOR)
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT=0;
#endif
	himax_esd_ic_reset();

	I("END_Himax TP: ESD - Reset\n");
}
#endif

#ifdef HX_CHIP_STATUS_MONITOR
static void himax_chip_monitor_function(struct work_struct *work) //for ESD solution
{
	int ret=0;

	I(" %s: POLLING_COUNT=%x, STATUS=%x\n", __func__,g_chip_monitor_data->HX_CHIP_POLLING_COUNT,ret);
	if(g_chip_monitor_data->HX_CHIP_POLLING_COUNT >= (g_chip_monitor_data->HX_POLLING_TIMES-1))//POLLING TIME
	{
		g_chip_monitor_data->HX_ON_HAND_SHAKING=1;
		ret = himax_hand_shaking(private_ts->client); //0:Running, 1:Stop, 2:I2C Fail
		g_chip_monitor_data->HX_ON_HAND_SHAKING=0;
		if(ret == 2)
		{
			I(" %s: I2C Fail \n", __func__);
			himax_esd_hw_reset();
		}
		else if(ret == 1)
		{
			I(" %s: MCU Stop \n", __func__);
			himax_esd_hw_reset();
		}
		g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;//clear polling counter
	}
	else
		g_chip_monitor_data->HX_CHIP_POLLING_COUNT++;
	
	
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 1;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, g_chip_monitor_data->HX_POLLING_TIMER*HZ);

	return;
}
#endif

#ifdef HX_SMART_WAKEUP
#ifdef HX_GESTURE_TRACK
static void gest_pt_log_coordinate(int rx, int tx)
{
	//driver report x y with range 0 - 255 , we scale it up to x/y pixel
	gest_pt_x[gest_pt_cnt] = rx*(ic_data->HX_X_RES)/255;
	gest_pt_y[gest_pt_cnt] = tx*(ic_data->HX_Y_RES)/255;
}
#endif
static int himax_parse_wake_event(struct himax_ts_data *ts)
{
	uint8_t buf[64];
#ifdef HX_GESTURE_TRACK
	int tmp_max_x=0x00,tmp_min_x=0xFFFF,tmp_max_y=0x00,tmp_min_y=0xFFFF;
	int gest_len;
#endif
	int i=0, check_FC = 0, gesture_flag = 0;
	
	memcpy(buf,hx_touch_data->hx_event_buf,hx_touch_data->event_size);
	
	for(i=0;i<GEST_PTLG_ID_LEN;i++)
	{
		if (check_FC==0)
		{
			if((buf[0]!=0x00)&&((buf[0]<=0x0F)||(buf[0]==0x80)))
			{
				check_FC = 1;
				gesture_flag = buf[i];
			}
			else
			{
				check_FC = 0;
				I("ID START at %x , value = %x skip the event\n", i, buf[i]);
				break;
			}
		}
		else
		{
			if(buf[i]!=gesture_flag)
			{
				check_FC = 0;
				I("ID NOT the same %x != %x So STOP parse event\n", buf[i], gesture_flag);
				break;
			}
		}

		I("0x%2.2X ", buf[i]);
		if (i % 8 == 7)
				I("\n");
	}
	I("Himax gesture_flag= %x\n",gesture_flag );
	I("Himax check_FC is %d\n", check_FC);

	if (check_FC == 0)
		return 0;
	if(buf[GEST_PTLG_ID_LEN] != GEST_PTLG_HDR_ID1 ||
			buf[GEST_PTLG_ID_LEN+1] != GEST_PTLG_HDR_ID2)
		return 0;

#ifdef HX_GESTURE_TRACK
	if(buf[GEST_PTLG_ID_LEN] == GEST_PTLG_HDR_ID1 &&
			buf[GEST_PTLG_ID_LEN+1] == GEST_PTLG_HDR_ID2)
	{
		gest_len = buf[GEST_PTLG_ID_LEN+2];

		I("gest_len = %d ",gest_len);

		i = 0;
		gest_pt_cnt = 0;
		I("gest doornidate start \n %s",__func__);
		while(i<(gest_len+1)/2)
		{
			gest_pt_log_coordinate(buf[GEST_PTLG_ID_LEN+4+i*2],buf[GEST_PTLG_ID_LEN+4+i*2+1]);
			i++;

			I("gest_pt_x[%d]=%d \n",gest_pt_cnt,gest_pt_x[gest_pt_cnt]);
			I("gest_pt_y[%d]=%d \n",gest_pt_cnt,gest_pt_y[gest_pt_cnt]);

			gest_pt_cnt +=1;
		}
		if(gest_pt_cnt)
			{
				for(i=0; i<gest_pt_cnt; i++)
					{
						if(tmp_max_x<gest_pt_x[i])
							tmp_max_x=gest_pt_x[i];
						if(tmp_min_x>gest_pt_x[i])
							tmp_min_x=gest_pt_x[i];
						if(tmp_max_y<gest_pt_y[i])
							tmp_max_y=gest_pt_y[i];
						if(tmp_min_y>gest_pt_y[i])
							tmp_min_y=gest_pt_y[i];
					}
				I("gest_point x_min= %d, x_max= %d, y_min= %d, y_max= %d\n",tmp_min_x,tmp_max_x,tmp_min_y,tmp_max_y);
				gest_start_x=gest_pt_x[0];
				gn_gesture_coor[0] = gest_start_x;
				gest_start_y=gest_pt_y[0];
				gn_gesture_coor[1] = gest_start_y;
				gest_end_x=gest_pt_x[gest_pt_cnt-1];
				gn_gesture_coor[2] = gest_end_x;
				gest_end_y=gest_pt_y[gest_pt_cnt-1];
				gn_gesture_coor[3] = gest_end_y;
				gest_width = tmp_max_x - tmp_min_x;
				gn_gesture_coor[4] = gest_width;
				gest_height = tmp_max_y - tmp_min_y;
				gn_gesture_coor[5] = gest_height;
				gest_mid_x = (tmp_max_x + tmp_min_x)/2;
				gn_gesture_coor[6] = gest_mid_x;
				gest_mid_y = (tmp_max_y + tmp_min_y)/2;
				gn_gesture_coor[7] = gest_mid_y;
				gn_gesture_coor[8] = gest_mid_x;//gest_up_x
				gn_gesture_coor[9] = gest_mid_y-gest_height/2;//gest_up_y
				gn_gesture_coor[10] = gest_mid_x;//gest_down_x
				gn_gesture_coor[11] = gest_mid_y+gest_height/2;	//gest_down_y
				gn_gesture_coor[12] = gest_mid_x-gest_width/2;	//gest_left_x
				gn_gesture_coor[13] = gest_mid_y;	//gest_left_y
				gn_gesture_coor[14] = gest_mid_x+gest_width/2;	//gest_right_x
				gn_gesture_coor[15] = gest_mid_y; //gest_right_y

			}

	}
#endif
	if(gesture_flag != 0x80)
	{
		if(!ts->gesture_cust_en[gesture_flag])
			{
				I("%s NOT report customer key \n ",__func__);
				return 0;//NOT report customer key
			}
	}
	else
	{
		if(!ts->gesture_cust_en[0])
			{
				I("%s NOT report report double click \n",__func__);
				return 0;//NOT report power key
			}
	}

	if(gesture_flag == 0x80)
		return EV_GESTURE_PWR;
	else
		return gesture_flag;
}

void himax_wake_check_func(void)
{
	int ret_event = 0, KEY_EVENT = 0;

	ret_event = himax_parse_wake_event(private_ts);
	switch (ret_event) {
		case EV_GESTURE_PWR:
			KEY_EVENT = KEY_POWER;
		break;
		case EV_GESTURE_01:
			KEY_EVENT = KEY_CUST_01;
		break;
		case EV_GESTURE_02:
			KEY_EVENT = KEY_CUST_02;
		break;
		case EV_GESTURE_03:
			KEY_EVENT = KEY_CUST_03;
		break;
		case EV_GESTURE_04:
			KEY_EVENT = KEY_CUST_04;
		break;
		case EV_GESTURE_05:
			KEY_EVENT = KEY_CUST_05;
		break;
		case EV_GESTURE_06:
			KEY_EVENT = KEY_CUST_06;
		break;
		case EV_GESTURE_07:
			KEY_EVENT = KEY_CUST_07;
		break;
		case EV_GESTURE_08:
			KEY_EVENT = KEY_CUST_08;
		break;
		case EV_GESTURE_09:
			KEY_EVENT = KEY_CUST_09;
		break;
		case EV_GESTURE_10:
			KEY_EVENT = KEY_CUST_10;
		break;
		case EV_GESTURE_11:
			KEY_EVENT = KEY_CUST_11;
		break;
		case EV_GESTURE_12:
			KEY_EVENT = KEY_CUST_12;
		break;
		case EV_GESTURE_13:
			KEY_EVENT = KEY_CUST_13;
		break;
		case EV_GESTURE_14:
			KEY_EVENT = KEY_CUST_14;
		break;
		case EV_GESTURE_15:
			KEY_EVENT = KEY_CUST_15;
		break;
	}
	if(ret_event)
		{
			I(" %s SMART WAKEUP KEY event %x press\n",__func__,KEY_EVENT);
			input_report_key(private_ts->input_dev, KEY_EVENT, 1);
			input_sync(private_ts->input_dev);
			//msleep(100);
			I(" %s SMART WAKEUP KEY event %x release\n",__func__,KEY_EVENT);
			input_report_key(private_ts->input_dev, KEY_EVENT, 0);
			input_sync(private_ts->input_dev);
			FAKE_POWER_KEY_SEND=true;
#ifdef HX_GESTURE_TRACK
			I("gest_start_x= %d, gest_start_y= %d, gest_end_x= %d, gest_end_y= %d\n",gest_start_x,gest_start_y,
			gest_end_x,gest_end_y);
			I("gest_width= %d, gest_height= %d, gest_mid_x= %d, gest_mid_y= %d\n",gest_width,gest_height,
			gest_mid_x,gest_mid_y);
			I("gest_up_x= %d, gest_up_y= %d, gest_down_x= %d, gest_down_y= %d\n",gn_gesture_coor[8],gn_gesture_coor[9],
			gn_gesture_coor[10],gn_gesture_coor[11]);
			I("gest_left_x= %d, gest_left_y= %d, gest_right_x= %d, gest_right_y= %d\n",gn_gesture_coor[12],gn_gesture_coor[13],
			gn_gesture_coor[14],gn_gesture_coor[15]);
#endif
		}
}

#endif
static void himax_ts_button_func(int tp_key_index,struct himax_ts_data *ts)
{
	uint16_t x_position = 0, y_position = 0;
if ( tp_key_index != 0x00)
	{
		I("virtual key index =%x\n",tp_key_index);
		if ( tp_key_index == 0x01) {
			vk_press = 1;
			I("back key pressed\n");
				if (ts->pdata->virtual_key)
				{
					if (ts->button[0].index) {
						x_position = (ts->button[0].x_range_min + ts->button[0].x_range_max) / 2;
						y_position = (ts->button[0].y_range_min + ts->button[0].y_range_max) / 2;
					}
					if (ts->protocol_type == PROTOCOL_TYPE_A) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
						input_mt_sync(ts->input_dev);
					} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
						1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
					}
				}
				else
					input_report_key(ts->input_dev, KEY_BACK, 1);
		}
		else if ( tp_key_index == 0x02) {
			vk_press = 1;
			I("home key pressed\n");
				if (ts->pdata->virtual_key)
				{
					if (ts->button[1].index) {
						x_position = (ts->button[1].x_range_min + ts->button[1].x_range_max) / 2;
						y_position = (ts->button[1].y_range_min + ts->button[1].y_range_max) / 2;
					}
						if (ts->protocol_type == PROTOCOL_TYPE_A) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
						input_mt_sync(ts->input_dev);
					} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
						1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
					}
				}
				else
					input_report_key(ts->input_dev, KEY_HOME, 1);
		}
		else if ( tp_key_index == 0x04) {
			vk_press = 1;
			I("APP_switch key pressed\n");
				if (ts->pdata->virtual_key)
				{
					if (ts->button[2].index) {
						x_position = (ts->button[2].x_range_min + ts->button[2].x_range_max) / 2;
						y_position = (ts->button[2].y_range_min + ts->button[2].y_range_max) / 2;
					}
						if (ts->protocol_type == PROTOCOL_TYPE_A) {
						input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, 0);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
						input_mt_sync(ts->input_dev);
					} else if (ts->protocol_type == PROTOCOL_TYPE_B) {
						input_mt_slot(ts->input_dev, 0);
						input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER,
						1);
						input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR,
							100);
						input_report_abs(ts->input_dev, ABS_MT_PRESSURE,
							100);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_X,
							x_position);
						input_report_abs(ts->input_dev, ABS_MT_POSITION_Y,
							y_position);
					}
				}
				else
				{	
					//input_report_key(ts->input_dev, KEY_APP_SWITCH, 1);	
				}
		}
		input_sync(ts->input_dev);
	}
else/*tp_key_index =0x00*/
	{
		I("virtual key released\n");
		vk_press = 0;
		if (ts->protocol_type == PROTOCOL_TYPE_A) {
			input_mt_sync(ts->input_dev);
		}
		else if (ts->protocol_type == PROTOCOL_TYPE_B) {
			input_mt_slot(ts->input_dev, 0);
			input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
		}
		input_report_key(ts->input_dev, KEY_BACK, 0);
		input_report_key(ts->input_dev, KEY_HOME, 0);
		//input_report_key(ts->input_dev, KEY_APP_SWITCH, 0);
	input_sync(ts->input_dev);
	}
}

int himax_report_data_init(void)
{
	if(hx_touch_data->hx_coord_buf!=NULL)
	{
		kfree(hx_touch_data->hx_coord_buf);
	}
#ifdef HX_TP_PROC_DIAG
	if(hx_touch_data->hx_rawdata_buf!=NULL)
	{
		kfree(hx_touch_data->hx_rawdata_buf);
	}
#endif

#if defined(HX_SMART_WAKEUP)
	hx_touch_data->event_size = 56;
	if(hx_touch_data->hx_event_buf!=NULL)
	{
		kfree(hx_touch_data->hx_event_buf);
	}
#endif

	hx_touch_data->touch_all_size = himax_get_touch_data_size();
	hx_touch_data->raw_cnt_max = ic_data->HX_MAX_PT/4;
	hx_touch_data->raw_cnt_rmd = ic_data->HX_MAX_PT%4;

	if (hx_touch_data->raw_cnt_rmd != 0x00) //more than 4 fingers
	{
		hx_touch_data->rawdata_size = cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT+hx_touch_data->raw_cnt_max+2)*4;
	}
	else //less than 4 fingers
	{
		hx_touch_data->rawdata_size = cal_data_len(hx_touch_data->raw_cnt_rmd, ic_data->HX_MAX_PT, hx_touch_data->raw_cnt_max);
		hx_touch_data->touch_info_size = (ic_data->HX_MAX_PT+hx_touch_data->raw_cnt_max+1)*4;
	}
	
	I("%s: ic_data->HX_MAX_PT:%d,hx_raw_cnt_max:%d,hx_raw_cnt_rmd:%d,g_hx_rawdata_size:%d,hx_touch_data->touch_info_size:%d\n",__func__,ic_data->HX_MAX_PT,hx_touch_data->raw_cnt_max,hx_touch_data->raw_cnt_rmd,hx_touch_data->rawdata_size,hx_touch_data->touch_info_size);
	
	hx_touch_data->hx_coord_buf = kzalloc(sizeof(uint8_t)*(hx_touch_data->touch_info_size),GFP_KERNEL);
	if(hx_touch_data->hx_coord_buf == NULL)
		goto mem_alloc_fail;
#ifdef HX_TP_PROC_DIAG
	hx_touch_data->hx_rawdata_buf = kzalloc(sizeof(uint8_t)*(hx_touch_data->touch_all_size - hx_touch_data->touch_info_size),GFP_KERNEL);
	if(hx_touch_data->hx_rawdata_buf == NULL)
		goto mem_alloc_fail;
#endif

#if defined(HX_SMART_WAKEUP)
	hx_touch_data->hx_event_buf = kzalloc(sizeof(uint8_t)*(hx_touch_data->event_size),GFP_KERNEL);
	if(hx_touch_data->hx_event_buf == NULL)
		goto mem_alloc_fail;
#endif

	return NO_ERR;

mem_alloc_fail:
	kfree(hx_touch_data->hx_coord_buf);
#if defined(HX_TP_PROC_DIAG)
	kfree(hx_touch_data->hx_rawdata_buf);
#endif
#if defined(HX_SMART_WAKEUP)
	kfree(hx_touch_data->hx_event_buf);
#endif
	
	I("%s: Memory allocate fail!\n",__func__);
	return MEM_ALLOC_FAIL;
	
}

void himax_report_key(struct himax_ts_data *ts)
{
	if(hx_point_num!=0)
	{
		//Touch KEY
		if ((tpd_key_old != 0x00)&&(tpd_key == 0x00)) 
		{
			//temp_x[0] = 0xFFFF;
			//temp_y[0] = 0xFFFF;
			//temp_x[1] = 0xFFFF;
			//temp_y[1] = 0xFFFF;
			himax_ts_button_func(tpd_key,ts);
			hx_touch_data->finger_on = 0;
		}
		input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
		input_sync(ts->input_dev);
	}
	else
	{
		if (tpd_key != 0x00) 
		{
			himax_ts_button_func(tpd_key,ts);
			hx_touch_data->finger_on = 1;
		}
		else if ((tpd_key_old != 0x00)&&(tpd_key == 0x00)) 
		{
			himax_ts_button_func(tpd_key,ts);
			hx_touch_data->finger_on = 0;
		}
		input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
		input_sync(ts->input_dev);
	}
	tpd_key_old = tpd_key;
	Last_EN_NoiseFilter = EN_NoiseFilter;
}

void himax_report_points(struct himax_ts_data *ts)
{
	int x = 0;
	int y = 0;
	int w = 0;
	int base = 0;
	int32_t	loop_i = 0;
	uint16_t old_finger = 0;
	
#if defined(HX_TP_PROC_DIAG)
	struct timeval t;
	struct tm broken;
#endif
        printk("himax_report_points\n");	
	//I("%s:Entering\n",__func__);

	/* finger on/press */
	if (hx_point_num != 0 )
	{	
		old_finger = ts->pre_finger_mask;
		ts->pre_finger_mask = 0;
		hx_touch_data->finger_num = hx_touch_data->hx_coord_buf[ts->coordInfoSize - 4] & 0x0F;
		hx_touch_data->finger_on = 1;
		AA_press = 1;
		for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++)
		{
			base = loop_i * 4;
			x = hx_touch_data->hx_coord_buf[base] << 8 | hx_touch_data->hx_coord_buf[base + 1];
			y = (hx_touch_data->hx_coord_buf[base + 2] << 8 | hx_touch_data->hx_coord_buf[base + 3]);
			w = hx_touch_data->hx_coord_buf[(ts->nFinger_support * 4) + loop_i];
			//x = ic_data->HX_X_RES - x;
			//y = ic_data->HX_Y_RES - y;
			if(x >= 0 && x <= ts->pdata->abs_x_max && y >= 0 && y <= ts->pdata->abs_y_max)
			{
				hx_touch_data->finger_num--;
				if ((ts->debug_log_level & BIT(3)) > 0)
				{
					himax_log_touch_event_detail(ts,x,y,w,loop_i,EN_NoiseFilter,HX_FINGER_ON,old_finger);
				}
				if (ts->protocol_type == PROTOCOL_TYPE_B)
				{
					input_mt_slot(ts->input_dev, loop_i);
				}

				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, w);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, w);
				input_report_abs(ts->input_dev, ABS_MT_PRESSURE, w);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
						
				if (ts->protocol_type == PROTOCOL_TYPE_A)
				{
					input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, loop_i);
					input_mt_sync(ts->input_dev);
				}
				/* ts->protocol_type == PROTOCOL_TYPE_B */
				else
				{
					ts->last_slot = loop_i;
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 1);
				}

				if (!ts->first_pressed)
				{
					ts->first_pressed = 1;
					printk("himax %d, %d\n", x, y);
				}

				ts->pre_finger_data[loop_i][0] = x;
				ts->pre_finger_data[loop_i][1] = y;
				if (ts->debug_log_level & BIT(1))
					himax_log_touch_event(x, y, w,loop_i,EN_NoiseFilter,HX_FINGER_ON);

				ts->pre_finger_mask = ts->pre_finger_mask + (1 << loop_i);
			}
			/* report coordinates */
			else
			{
				if (ts->protocol_type == PROTOCOL_TYPE_B)
				{
					input_mt_slot(ts->input_dev, loop_i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
				}

				if (loop_i == 0 && ts->first_pressed == 1)
				{
					ts->first_pressed = 2;
					I("E1@%d, %d\n",
					ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
				}
				if ((ts->debug_log_level & BIT(3)) > 0)
				{
					himax_log_touch_event_detail(ts,x,y,w,loop_i,Last_EN_NoiseFilter,HX_FINGER_LEAVE,old_finger);
				}
			}
		}

		input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
		input_sync(ts->input_dev);
	}
	
	/* finger leave/release */
	else
	{
#if defined(HX_PALM_REPORT)
		if(himax_palm_detect(hx_touch_data->hx_coord_buf) == NO_ERR)
		{
			I(" %s HX_PALM_REPORT KEY power event press\n",__func__);
			input_report_key(ts->input_dev, KEY_POWER, 1);
			input_sync(ts->input_dev);
			msleep(100);
			I(" %s HX_PALM_REPORT KEY power event release\n",__func__);
			input_report_key(ts->input_dev, KEY_POWER, 0);
			input_sync(ts->input_dev);
			return;
		}
#endif
		hx_touch_data->finger_on = 0;
		AA_press = 0;
		if (ts->protocol_type == PROTOCOL_TYPE_A)
		input_mt_sync(ts->input_dev);

		for (loop_i = 0; loop_i < ts->nFinger_support; loop_i++)
		{
			if (((ts->pre_finger_mask >> loop_i) & 1) == 1)
			{
				if (ts->protocol_type == PROTOCOL_TYPE_B)
				{
					input_mt_slot(ts->input_dev, loop_i);
					input_mt_report_slot_state(ts->input_dev, MT_TOOL_FINGER, 0);
				}
			}
			if(ts->pre_finger_mask > 0 && (ts->debug_log_level & BIT(3)) > 0)
			{
				if (((ts->pre_finger_mask >> loop_i) & 1) == 1)
				{
					if (ts->useScreenRes)
					{
						I("status:%X, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n", 0, loop_i+1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
						ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
					}
					else
					{
						I("status:%X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",0, loop_i+1, ts->pre_finger_data[loop_i][0],ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
					}
				}
			}
		}
		if (ts->pre_finger_mask > 0)
		{
			/*for (loop_i = 0; loop_i < ts->nFinger_support && (ts->debug_log_level & BIT(3)) > 0; loop_i++)
			{
				if (((ts->pre_finger_mask >> loop_i) & 1) == 1)
				{
					if (ts->useScreenRes)
					{
						I("status:%X, Screen:F:%02d Up, X:%d, Y:%d, N:%d\n", 0, loop_i+1, ts->pre_finger_data[loop_i][0] * ts->widthFactor >> SHIFTBITS,
						ts->pre_finger_data[loop_i][1] * ts->heightFactor >> SHIFTBITS, Last_EN_NoiseFilter);
					}
					else
					{
						I("status:%X, Raw:F:%02d Up, X:%d, Y:%d, N:%d\n",0, loop_i+1, ts->pre_finger_data[loop_i][0],ts->pre_finger_data[loop_i][1], Last_EN_NoiseFilter);
					}
				}
			}*/
			ts->pre_finger_mask = 0;
		}

		if (ts->first_pressed == 1)
		{
			ts->first_pressed = 2;
			I("E1@%d, %d\n",ts->pre_finger_data[0][0] , ts->pre_finger_data[0][1]);
		}

		if (ts->debug_log_level & BIT(1))
			himax_log_touch_event(x, y, w,loop_i,EN_NoiseFilter,HX_FINGER_LEAVE);

#ifdef HX_TP_PROC_DIAG
		//coordinate dump start
		if (coordinate_dump_enable == 1)
		{
			do_gettimeofday(&t);
			time_to_tm(t.tv_sec, 0, &broken);

			sprintf(&hx_touch_data->coordinate_char[0], "%2d:%2d:%2d:%lu,", broken.tm_hour, broken.tm_min, broken.tm_sec, t.tv_usec/1000);
			sprintf(&hx_touch_data->coordinate_char[15], "Touch up!");
			coordinate_fn->f_op->write(coordinate_fn,&hx_touch_data->coordinate_char[0],15 + (ic_data->HX_MAX_PT+5)*2*sizeof(char)*5 + 2,&coordinate_fn->f_pos);
		}
		//coordinate dump end
#endif
		input_report_key(ts->input_dev, BTN_TOUCH, hx_touch_data->finger_on);
		input_sync(ts->input_dev);
	}
	Last_EN_NoiseFilter = EN_NoiseFilter;
	
	//I("%s:End\n",__func__);
}

int himax_touch_get(struct himax_ts_data *ts,uint8_t *buf,int ts_status)
{
	int ret = 0;
	
	switch(ts_status)
	{
		/*normal*/
		case 1: 
#ifdef HX_TP_PROC_DIAG
			hx_touch_data->diag_cmd = getDiagCommand();

			if((hx_touch_data->diag_cmd) 
				|| (HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
				|| (HX_ESD_RESET_ACTIVATE)
#endif
			)
			{
				ret = himax_read_event_stack(ts->client, buf, 128);
			}
			else
			{
				ret = himax_read_event_stack(ts->client, buf, hx_touch_data->touch_info_size);
			}

			if (!ret)
#else
			if(!himax_read_event_stack(ts->client, buf, hx_touch_data->touch_info_size))
#endif		
			{
				E("%s: can't read data from chip!\n", __func__);
				goto err_workqueue_out;
			}
		break;
#if defined(HX_SMART_WAKEUP)
		/*SMWP*/
		case 2:
			himax_burst_enable(ts->client, 0);
			if(!himax_read_event_stack(ts->client,buf,hx_touch_data->event_size))
			{
				E("%s: can't read data from chip!\n", __func__);
				goto err_workqueue_out;
			}
		break;
#endif
		default:
		break;
	}
	return NO_ERR;

err_workqueue_out:
	return I2C_FAIL;
	
}

int himax_checksum_cal(struct himax_ts_data *ts,uint8_t *buf,int ts_status)
{
#if defined(HX_ESD_RECOVERY)
	int hx_esd_event = 0;
	int hx_zero_event = 0;
	int shaking_ret = 0;
#endif
	uint16_t check_sum_cal = 0;
	int32_t	loop_i = 0;
	int length = 0;
	
	/* Normal */
	if(ts_status == HX_REPORT_COORD)
		length = hx_touch_data->touch_info_size;
#if defined(HX_SMART_WAKEUP)
	/* SMWP */
	else if(ts_status == HX_REPORT_SMWP_EVENT)
		length = (GEST_PTLG_ID_LEN+GEST_PTLG_HDR_LEN);
#endif
	else
	{
		I("%s, Neither Normal Nor SMWP error!\n",__func__);
	}
	//I("Now status=%d,length=%d\n",ts_status,length);
	for (loop_i = 0; loop_i < length; loop_i++)
	{
		check_sum_cal+=buf[loop_i];
		
		/*if (ts->debug_log_level & BIT(0)) 
		{
			I("P %d = 0x%2.2X ", loop_i, hx_touch_data->hx_coord_buf[loop_i]);
			if (loop_i % 8 == 7)
				I("\n");
		}*/

#ifdef HX_ESD_RECOVERY
		if(ts_status == HX_REPORT_COORD)
		{
			/* case 1 ESD recovery flow */
			if (buf[loop_i] == 0xED)
			{
				hx_esd_event++;
			}
			/* case 2 ESD recovery flow-Disable */
			else if(buf[loop_i] == 0x00)
				hx_zero_event++;
			else
			{
				hx_zero_event = 0;
				hx_esd_event = 0;
			}
		}
#endif		
	}

	if(ts_status == HX_REPORT_COORD)
	{
#ifdef HX_ESD_RECOVERY
		if ((hx_esd_event == length || hx_zero_event == length)
			&& (HX_HW_RESET_ACTIVATE == 0)
			&& (HX_ESD_RESET_ACTIVATE == 0)
#if defined(HX_TP_PROC_DIAG)
			&& (hx_touch_data->diag_cmd == 0)
#endif
#if defined(HX_TP_PROC_SELF_TEST)
			&& (g_self_test_entered == 0)
#endif
			)
		{
			shaking_ret = himax_ic_esd_recovery(hx_esd_event,hx_zero_event,length);
			if(shaking_ret == CHECKSUM_FAIL)
			{
				himax_esd_hw_reset();
				goto checksum_fail;
			}
			else if(shaking_ret == ERR_WORK_OUT)
				goto err_workqueue_out;
			else
			{
				//I("I2C running. Nothing to be done!\n");
				goto workqueue_out;
			}
		}
		else if (HX_ESD_RESET_ACTIVATE)
		{

#ifdef HX_RESEND_CMD
			queue_delayed_work(ts->himax_resend_cmd_wq, &ts->resend_cmd_work, msecs_to_jiffies(50));
#endif
			/* drop 1st interrupts after chip reset */
			HX_ESD_RESET_ACTIVATE = 0;
			I("[HX_ESD_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
			goto checksum_fail;
		}
		
		else if (HX_HW_RESET_ACTIVATE)
#else
		if (HX_HW_RESET_ACTIVATE)
#endif
		{

#ifdef HX_RESEND_CMD
			queue_delayed_work(ts->himax_resend_cmd_wq, &ts->resend_cmd_work, msecs_to_jiffies(50));
#endif
			/* drop 1st interrupts after chip reset */
			HX_HW_RESET_ACTIVATE = 0;
			I("[HX_HW_RESET_ACTIVATE]:%s: Back from reset, ready to serve.\n", __func__);
				goto ready_to_serve;
		}
	}

	if ((check_sum_cal % 0x100 != 0) )
	{
		I("[HIMAX TP MSG] checksum fail : check_sum_cal: 0x%02X\n", check_sum_cal);
		goto checksum_fail;
	}
	
	/* I("%s:End\n",__func__); */
	return NO_ERR;

ready_to_serve:
	return READY_TO_SERVE;
checksum_fail:
	return CHECKSUM_FAIL;
#ifdef HX_ESD_RECOVERY
err_workqueue_out:
	return ERR_WORK_OUT;
workqueue_out:
	return WORK_OUT;
#endif
}

int himax_ts_work_status(struct himax_ts_data *ts)
{
	/* 1: normal, 2:SMWP */
	int result = HX_REPORT_COORD; 
	uint8_t diag_cmd;

#ifdef HX_TP_PROC_DIAG
	diag_cmd = getDiagCommand();
#endif

#ifdef HX_SMART_WAKEUP
	if (atomic_read(&ts->suspend_mode)&&(!FAKE_POWER_KEY_SEND)&&(ts->SMWP_enable)&&(!diag_cmd)) {
		result = HX_REPORT_SMWP_EVENT;
	}
#endif
	/* I("Now Status is %d\n",result); */
	return result;
}

void himax_assign_touch_data(uint8_t *buf,int ts_status)
{
	if(ts_status == HX_REPORT_COORD)
		memcpy(hx_touch_data->hx_coord_buf,&buf[0],hx_touch_data->touch_info_size);
#if defined(HX_SMART_WAKEUP)
	else
		memcpy(hx_touch_data->hx_event_buf,buf,hx_touch_data->event_size);
#endif

#ifdef HX_TP_PROC_DIAG
	if((hx_touch_data->diag_cmd)
		|| (HX_HW_RESET_ACTIVATE)
#ifdef HX_ESD_RECOVERY
		|| (HX_ESD_RESET_ACTIVATE)
#endif
		)
	{
		memcpy(hx_touch_data->hx_rawdata_buf,&buf[hx_touch_data->touch_info_size],hx_touch_data->touch_all_size - hx_touch_data->touch_info_size);
	}
#endif
	
}

void himax_coord_report(struct himax_ts_data *ts)
{

#if defined(HX_TP_PROC_DIAG)
		//touch monitor raw data fetch
		if(himax_set_diag_cmd(ic_data,hx_touch_data))	
			I("%s: coordinate dump fail and bypass with checksum err\n",__func__);
#endif
		EN_NoiseFilter = (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT+2]>>3);
		//I("EN_NoiseFilter=%d\n",EN_NoiseFilter);
		EN_NoiseFilter = EN_NoiseFilter & 0x01;
		//I("EN_NoiseFilter2=%d\n",EN_NoiseFilter);

#if defined(HX_EN_SEL_BUTTON) || defined(HX_EN_MUT_BUTTON)
		tpd_key = (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT+2]>>4);
		/* All (VK+AA)leave */
		if (tpd_key == 0x0F)
		{
			tpd_key = 0x00;
		}
		//I("[DEBUG] tpd_key:  %x\r\n", tpd_key);
#else
		tpd_key = 0x00;
#endif

		p_point_num = hx_point_num;

		if (hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
			hx_point_num = 0;
		else
			hx_point_num= hx_touch_data->hx_coord_buf[HX_TOUCH_INFO_POINT_CNT] & 0x0f;

		/* Touch Point information */
		if(!tpd_key && !tpd_key_old)
			himax_report_points(ts);
		else
			himax_report_key(ts);
		
		/* I("%s:END\n",__func__); */
		
}

void himax_ts_work(struct himax_ts_data *ts)
{
	uint8_t hw_reset_check[2];
	uint8_t buf[128];
	uint16_t check_sum_cal = 0;
	//int loop_i = 0;
	int ts_status = 0;
#ifdef HX_CHIP_STATUS_MONITOR
		int j=0;
#endif

#if defined(HX_USB_DETECT_GLOBAL)
	himax_cable_detect_func(false);
#endif

#if defined(HX_CHIP_STATUS_MONITOR)
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT=0;
	if(g_chip_monitor_data->HX_ON_HAND_SHAKING)//chip on hand shaking,wait hand shaking
	{
		for(j=0; j<100; j++)
		{
			if(g_chip_monitor_data->HX_ON_HAND_SHAKING==0)//chip on hand shaking end
			{
				I("%s:HX_ON_HAND_SHAKING OK check %d times\n",__func__,j);
				break;
			}
			else
				msleep(1);
		}
		if(j==100)
		{
			E("%s:HX_ON_HAND_SHAKING timeout reject interrupt\n",__func__);
			return;
		}
	}
#endif

	ts_status = himax_ts_work_status(ts);
	if(ts_status > HX_REPORT_SMWP_EVENT || ts_status < HX_REPORT_COORD)
		goto neither_normal_nor_smwp;

	memset(buf, 0x00, sizeof(buf));
	memset(hw_reset_check, 0x00, sizeof(hw_reset_check));
	
	//I("New Method for ts_work\n");
	
	if(himax_touch_get(ts,buf,ts_status))
		goto err_workqueue_out;

	if (ts->debug_log_level & BIT(0)) 
	{

		himax_log_touch_data(buf,hx_touch_data);

	}
	
	check_sum_cal = himax_checksum_cal(ts,buf,ts_status);
	if (check_sum_cal == CHECKSUM_FAIL)
		goto checksum_fail;
	else if (check_sum_cal == READY_TO_SERVE)
		goto ready_to_serve;
	else if (check_sum_cal == ERR_WORK_OUT)
		goto err_workqueue_out;
	else if (check_sum_cal == WORK_OUT)
		goto workqueue_out;
	/* checksum calculate pass and assign data to global touch data*/
	else
		himax_assign_touch_data(buf,ts_status);
	
	if(ts_status == HX_REPORT_COORD)
		himax_coord_report(ts);
#if defined(HX_SMART_WAKEUP)
	else
	{
		wake_lock_timeout(&ts->ts_SMWP_wake_lock, TS_WAKE_LOCK_TIMEOUT);
		msleep(200);
		himax_wake_check_func();
	}
#endif

checksum_fail:
workqueue_out:
ready_to_serve:
neither_normal_nor_smwp:
	return;

err_workqueue_out:
	I("%s: Now reset the Touch chip.\n", __func__);

#ifdef HX_RST_PIN_FUNC
	himax_ic_reset(true,true);
#endif

	goto workqueue_out;
}
enum hrtimer_restart himax_ts_timer_func(struct hrtimer *timer)
{
	struct himax_ts_data *ts;

	ts = container_of(timer, struct himax_ts_data, timer);
	queue_work(ts->himax_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

#if defined( HX_USB_DETECT_CALLBACK)
static void himax_cable_tp_status_handler_func(int connect_status)
{
	struct himax_ts_data *ts;
	I("Touch: cable change to %d\n", connect_status);
	ts = private_ts;
	if (ts->cable_config) {
		if (!atomic_read(&ts->suspend_mode)) {
			if ((!!connect_status) != ts->usb_connected) {
				if (!!connect_status) {
					ts->cable_config[1] = 0x01;
					ts->usb_connected = 0x01;
				} else {
					ts->cable_config[1] = 0x00;
					ts->usb_connected = 0x00;
				}

				i2c_himax_master_write(ts->client, ts->cable_config,
					sizeof(ts->cable_config), HIMAX_I2C_RETRY_TIMES);

				I("%s: Cable status change: 0x%2.2X\n", __func__, ts->cable_config[1]);
			} else
				I("%s: Cable status is the same as previous one, ignore.\n", __func__);
		} else {
			if (connect_status)
				ts->usb_connected = 0x01;
			else
				ts->usb_connected = 0x00;
			I("%s: Cable status remembered: 0x%2.2X\n", __func__, ts->usb_connected);
		}
	}
}

static struct t_cable_status_notifier himax_cable_status_handler = {
	.name = "usb_tp_connected",
	.func = himax_cable_tp_status_handler_func,
};

#endif

#ifdef HX_AUTO_UPDATE_FW
static void himax_update_register(struct work_struct *work)
{
	I(" %s in", __func__);
#if defined(HX_CHIP_STATUS_MONITOR)
	I("Cancel Chip monitor during auto-updating!\n");
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 0;
	cancel_delayed_work_sync(&private_ts->himax_chip_monitor);
#endif
	if(i_update_FW() == false)
			I("NOT Have new FW=NOT UPDATE=\n");
	else
			I("Have new FW=UPDATE=\n");
#ifdef HX_CHIP_STATUS_MONITOR
	I("Auto-updating over, now chip monitor working!\n");
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 1;
	queue_delayed_work(private_ts->himax_chip_monitor_wq, &private_ts->himax_chip_monitor, g_chip_monitor_data->HX_POLLING_TIMER*HZ);
#endif
}
#endif

#if defined(HX_USB_DETECT_GLOBAL)
void himax_cable_detect_func(bool force_renew)
{
	struct himax_ts_data *ts;
	u32 connect_status = 0;

	connect_status = USB_detect_flag;//upmu_is_chr_det();
	ts = private_ts;
	//I("Touch: cable status=%d, cable_config=%p, usb_connected=%d \n", connect_status,ts->cable_config, ts->usb_connected);
	if (ts->cable_config) {
		if (((!!connect_status) != ts->usb_connected) || force_renew) {

			if (!!connect_status) {
				ts->cable_config[1] = 0x01;
				ts->usb_connected = 0x01;
			} else {
				ts->cable_config[1] = 0x00;
				ts->usb_connected = 0x00;
			}

			himax_usb_detect_set(ts->client,ts->cable_config);

			I("%s: Cable status change: 0x%2.2X\n", __func__, ts->usb_connected);
			}
		//else
		//	I("%s: Cable status is the same as previous one, ignore.\n", __func__);
	}
}
#endif

#ifdef CONFIG_FB
static void himax_fb_register(struct work_struct *work)
{
	int ret = 0;
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
							work_att.work);
	I(" %s in\n", __func__);

	ts->fb_notif.notifier_call = fb_notifier_callback;
	ret = fb_register_client(&ts->fb_notif);
	if (ret)
		E(" Unable to register fb_notifier: %d\n", ret);
}
#endif

#ifdef HX_RESEND_CMD
static void himax_resend_cmd_work(struct work_struct *work)
{
	struct himax_ts_data *ts = container_of(work, struct himax_ts_data,
							resend_cmd_work.work);

	I(" %s in\n", __func__);

	himax_resend_cmd_func(ts->suspended);
}
#endif

#ifdef  HX_TP_PROC_FLASH_DUMP
static void himax_ts_flash_work_func(struct work_struct *work)
{
	himax_ts_flash_func();
}
#endif

#ifdef  HX_TP_PROC_DIAG
static void himax_ts_diag_work_func(struct work_struct *work)
{
	himax_ts_diag_func();
}
#endif

int himax_chip_common_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret = 0, err = 0;
	bool auto_update_flag = false;
	struct himax_ts_data *ts;
	struct himax_i2c_platform_data *pdata;
       printk("himax_chip_common_probe\n");
	//Check I2C functionality
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	ts = kzalloc(sizeof(struct himax_ts_data), GFP_KERNEL);
	if (ts == NULL) {
		E("%s: allocate himax_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	i2c_set_clientdata(client, ts);
	ts->client = client;
	ts->dev = &client->dev;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (pdata == NULL) { /*Allocate Platform data space*/
		err = -ENOMEM;
			goto err_dt_platform_data_fail;
	}

	ic_data = kzalloc(sizeof(*ic_data), GFP_KERNEL);
	if (ic_data == NULL) { /*Allocate IC data space*/
		err = -ENOMEM;
			goto err_dt_ic_data_fail;
	}

	/* allocate report data */
	hx_touch_data = kzalloc(sizeof(struct himax_report_data),GFP_KERNEL);
	if(hx_touch_data == NULL)
	{
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	if (himax_parse_dt(ts, pdata) < 0)
	{
		printk("himax pdata is NULL for DT\n");
		goto err_alloc_dt_pdata_failed;
	}

#ifdef HX_RST_PIN_FUNC
	ts->rst_gpio = pdata->gpio_reset;
#endif

	himax_gpio_power_config(ts->client, pdata);

#ifndef CONFIG_OF
	if (pdata->power) {
		ret = pdata->power(1);
		if (ret < 0) {
			E("%s: power on failed\n", __func__);
			goto err_power_failed;
		}
	}
#endif
	private_ts = ts;

	if (himax_ic_package_check(ts->client) == false) {
		printk("Himax chip doesn NOT EXIST");
		goto err_ic_package_failed;
	}

	if (pdata->virtual_key)
		ts->button = pdata->virtual_key;
#ifdef  HX_TP_PROC_FLASH_DUMP
		ts->flash_wq = create_singlethread_workqueue("himax_flash_wq");
		if (!ts->flash_wq)
		{
			E("%s: create flash workqueue failed\n", __func__);
			err = -ENOMEM;
			goto err_create_wq_failed;
		}
	
		INIT_WORK(&ts->flash_work, himax_ts_flash_work_func);
	
		setSysOperation(0);
		setFlashBuffer();
#endif

#ifdef  HX_TP_PROC_DIAG
	  ts->himax_diag_wq = create_singlethread_workqueue("himax_diag");
		if (!ts->himax_diag_wq)
		{
			E("%s: create diag workqueue failed\n", __func__);
			err = -ENOMEM;
			goto err_create_wq_failed;
		}
		INIT_DELAYED_WORK(&ts->himax_diag_delay_wrok, himax_ts_diag_work_func);
#endif


	himax_read_FW_ver(client);

	auto_update_flag = !himax_calculateChecksum(client, false);
#ifdef HX_AUTO_UPDATE_FW
	auto_update_flag |= (( ic_data->vendor_fw_ver < g_i_FW_VER ) || ( ic_data->vendor_config_ver < g_i_CFG_VER ));
	if (auto_update_flag)
	{
				ts->himax_update_wq = create_singlethread_workqueue("HMX_update_reuqest");
		if (!ts->himax_update_wq) {
			printk(" allocate syn_update_wq failed\n");
			err = -ENOMEM;
			goto err_update_wq_failed;
		}
		INIT_DELAYED_WORK(&ts->work_update, himax_update_register);
		queue_delayed_work(ts->himax_update_wq, &ts->work_update, msecs_to_jiffies(2000));
	}
#endif

	//Himax Power On and Load Config
	if (himax_loadSensorConfig(client, pdata)) {
		E("%s: Load Sesnsor configuration failed, unload driver.\n", __func__);
		goto err_detect_failed;
	}	
	himax_power_on_init(client);

	calculate_point_number();
#ifdef HX_TP_PROC_DIAG
	setXChannel(ic_data->HX_RX_NUM); // X channel
	setYChannel(ic_data->HX_TX_NUM); // Y channel

	setMutualBuffer();
	setMutualNewBuffer();
	setMutualOldBuffer();
	if (getMutualBuffer() == NULL) {
		E("%s: mutual buffer allocate fail failed\n", __func__);
		return -1;
	}
#ifdef HX_TP_PROC_2T2R
	if(Is_2T2R){
		setXChannel_2(ic_data->HX_RX_NUM_2); // X channel
		setYChannel_2(ic_data->HX_TX_NUM_2); // Y channel

		setMutualBuffer_2();

		if (getMutualBuffer_2() == NULL) {
			E("%s: mutual buffer 2 allocate fail failed\n", __func__);
			return -1;
		}
	}
#endif	
#endif
#ifdef CONFIG_OF
	ts->power = pdata->power;
#endif
	ts->pdata = pdata;

	ts->x_channel = ic_data->HX_RX_NUM;
	ts->y_channel = ic_data->HX_TX_NUM;
	ts->nFinger_support = ic_data->HX_MAX_PT;
	//calculate the i2c data size
	calcDataSize(ts->nFinger_support);
	I("%s: calcDataSize complete\n", __func__);
#ifdef CONFIG_OF
	ts->pdata->abs_pressure_min        = 0;
	ts->pdata->abs_pressure_max        = 200;
	ts->pdata->abs_width_min           = 0;
	ts->pdata->abs_width_max           = 200;
	pdata->cable_config[0]             = 0xF0;
	pdata->cable_config[1]             = 0x00;
#endif
	ts->suspended                      = false;
#if defined( HX_USB_DETECT_CALLBACK)||defined(HX_USB_DETECT_GLOBAL)
	ts->usb_connected = 0x00;
	ts->cable_config = pdata->cable_config;
#endif
	ts->protocol_type = pdata->protocol_type;
	I("%s: Use Protocol Type %c\n", __func__,
	ts->protocol_type == PROTOCOL_TYPE_A ? 'A' : 'B');

	ret = himax_input_register(ts);
	if (ret) {
		E("%s: Unable to register %s input device\n",
			__func__, ts->input_dev->name);
		goto err_input_register_device_failed;
	}
#ifdef CONFIG_FB
		ts->himax_att_wq = create_singlethread_workqueue("HMX_ATT_reuqest");
		if (!ts->himax_att_wq) {
			printk(" allocate syn_att_wq failed\n");
			err = -ENOMEM;
			goto err_get_intr_bit_failed;
		}
		INIT_DELAYED_WORK(&ts->work_att, himax_fb_register);
		queue_delayed_work(ts->himax_att_wq, &ts->work_att, msecs_to_jiffies(15000));
#endif

#if defined(HX_CHIP_STATUS_MONITOR)//for ESD solution
	I("Enter HX_CHIP_STATUS_MONITOR! \n");
	
	g_chip_monitor_data = kzalloc(sizeof(struct chip_monitor_data),GFP_KERNEL);
	if(g_chip_monitor_data == NULL)
	{
		err = -ENOMEM;
		goto err_alloc_monitor_data;
	}
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;
	g_chip_monitor_data->HX_POLLING_TIMER	= 5;//unit:sec
	g_chip_monitor_data->HX_POLLING_TIMES	= 2;//ex:5(timer)x2(times)=10sec(polling time)
	g_chip_monitor_data->HX_ON_HAND_SHAKING = 0;//
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 1;

	ts->himax_chip_monitor_wq = create_singlethread_workqueue("himax_chip_monitor_wq");
	if (!ts->himax_chip_monitor_wq)
	{
		E(" %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto err_create_chip_monitor_wq_failed;
	}
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 1;

	INIT_DELAYED_WORK(&ts->himax_chip_monitor, himax_chip_monitor_function);

	queue_delayed_work(ts->himax_chip_monitor_wq, &ts->himax_chip_monitor, g_chip_monitor_data->HX_POLLING_TIMER*HZ);

#endif

#ifdef HX_SMART_WAKEUP
	ts->SMWP_enable=0;
	wake_lock_init(&ts->ts_SMWP_wake_lock, WAKE_LOCK_SUSPEND, HIMAX_common_NAME);
#endif
#ifdef HX_HIGH_SENSE
	ts->HSEN_enable=0;
#endif

#if defined(HX_SMART_WAKEUP)||defined(HX_HIGH_SENSE)||defined(HX_USB_DETECT_GLOBAL)
#ifdef HX_RESEND_CMD
	ts->himax_resend_cmd_wq = create_singlethread_workqueue("HMX_RESEND_CMD_WORK");
	if (!ts->himax_resend_cmd_wq) {
		printk(" allocate himax_resend_cmd_wq failed\n");
		err = -ENOMEM;
		goto err_resend_cmd_wq_failed;
	}
	INIT_DELAYED_WORK(&ts->resend_cmd_work, himax_resend_cmd_work);
#endif
#endif

	//touch data init
	err=himax_report_data_init();
	if(err)
		goto err_report_data_init_failed;

#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
	himax_touch_proc_init();
#endif

#if defined( HX_USB_DETECT_CALLBACK)
	if (ts->cable_config)		
		cable_detect_register_notifier(&himax_cable_status_handler);
#endif

	err = himax_ts_register_interrupt(ts->client);
	if (err)
		goto err_register_interrupt_failed;

#ifdef HX_AUTO_UPDATE_FW
	if (auto_update_flag)
	{
		himax_int_enable(client->irq,0);
	}
#endif
	tpd_load_status = 1;
return 0;

err_register_interrupt_failed:
err_report_data_init_failed:

#ifdef HX_RESEND_CMD
err_resend_cmd_wq_failed:
	wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
#ifdef  HX_CHIP_STATUS_MONITOR
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 0;
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
	destroy_workqueue(ts->himax_chip_monitor_wq);
err_create_chip_monitor_wq_failed:
err_alloc_monitor_data:
	kfree(g_chip_monitor_data);
#endif
#ifdef CONFIG_FB
err_get_intr_bit_failed:
#endif
err_input_register_device_failed:
	input_free_device(ts->input_dev);
err_detect_failed:
#ifdef HX_AUTO_UPDATE_FW
err_update_wq_failed:
#endif
#ifdef  HX_TP_PROC_FLASH_DUMP
err_create_wq_failed:
#endif
err_ic_package_failed:

err_alloc_dt_pdata_failed:
#ifndef CONFIG_OF
err_power_failed:
err_get_platform_data_fail:
#endif

	kfree(ic_data);

err_dt_ic_data_fail:
	kfree(pdata);

err_dt_platform_data_fail:
	kfree(ts);

err_alloc_data_failed:
	kfree(hx_touch_data);

err_check_functionality_failed:
	probe_fail_flag = 1;
	tpd_load_status = 0;
	return err;

}

int himax_chip_common_remove(struct i2c_client *client)
{
	struct himax_ts_data *ts = i2c_get_clientdata(client);
#if defined(CONFIG_TOUCHSCREEN_HIMAX_DEBUG)
	himax_touch_proc_deinit();
#endif
#ifdef  HX_CHIP_STATUS_MONITOR
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 0;
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
	destroy_workqueue(ts->himax_chip_monitor_wq);
	kfree(g_chip_monitor_data);
#endif
#ifdef CONFIG_FB
	if (fb_unregister_client(&ts->fb_notif))
		dev_err(&client->dev, "Error occurred while unregistering fb_notifier.\n");
#endif

	if (!ts->use_irq)
		hrtimer_cancel(&ts->timer);

	destroy_workqueue(ts->himax_wq);

	if (ts->protocol_type == PROTOCOL_TYPE_B)
		input_mt_destroy_slots(ts->input_dev);

	input_unregister_device(ts->input_dev);

#ifdef HX_SMART_WAKEUP
		wake_lock_destroy(&ts->ts_SMWP_wake_lock);
#endif
	kfree(ts);

	return 0;

}

int himax_chip_common_suspend(struct himax_ts_data *ts)
{
	int ret;
#ifdef HX_CHIP_STATUS_MONITOR
	int t = 0;
#endif

	if(ts->suspended)
	{
		printk("himax_chip_common_suspend Already suspended. Skipped. \n");
		return 0;
	}
	else
	{
		ts->suspended = true;
		I("%s: enter \n", __func__);
	}

#ifdef HX_TP_PROC_FLASH_DUMP
	if (getFlashDumpGoing())
	{
		I("[himax] %s: Flash dump is going, reject suspend\n",__func__);
		return 0;
	}
#endif

#ifdef HX_CHIP_STATUS_MONITOR
	if(g_chip_monitor_data->HX_ON_HAND_SHAKING)//chip on hand shaking,wait hand shaking
	{
		for(t=0; t<100; t++)
		{
			if(g_chip_monitor_data->HX_ON_HAND_SHAKING==0)//chip on hand shaking end
				{
					I("%s:HX_ON_HAND_SHAKING OK check %d times\n",__func__,t);
					break;
				}
			else
				msleep(1);
		}
		if(t==100)
		{
			E("%s:HX_ON_HAND_SHAKING timeout reject suspend\n",__func__);
			return 0;
		}
	}
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 0;
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;
	cancel_delayed_work_sync(&ts->himax_chip_monitor);
#endif
	
#if defined(HX_SMART_WAKEUP)||defined(HX_HIGH_SENSE)||defined(HX_USB_DETECT_GLOBAL)
#ifndef HX_RESEND_CMD
		himax_resend_cmd_func(ts->suspended);
#endif
#endif

#ifdef HX_SMART_WAKEUP
	if(ts->SMWP_enable)
	{
		atomic_set(&ts->suspend_mode, 1);
		ts->pre_finger_mask = 0;
		FAKE_POWER_KEY_SEND=false;
		I("[himax] %s: SMART_WAKEUP enable, reject suspend\n",__func__);
		return 0;
	}
#endif

	himax_int_enable(ts->client->irq,0);

	himax_suspend_ic_action(ts->client);

	if (!ts->use_irq) {
		ret = cancel_work_sync(&ts->work);
		if (ret)
			himax_int_enable(ts->client->irq,1);
	}

	//ts->first_pressed = 0;
	atomic_set(&ts->suspend_mode, 1);
	ts->pre_finger_mask = 0;
	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(0);
	printk("himax_chip_common_suspend END \n");
	return 0;
}

int himax_chip_common_resume(struct himax_ts_data *ts)
{

#ifdef HX_CHIP_STATUS_MONITOR
	int t=0;
#endif

	printk("himax_chip_common_resume enter \n");
	if(ts->suspended == false)
	{
		I("%s: It had entered resume,skip this step \n", __func__);
		return 0;
	}
	else
		ts->suspended = false;
	
	atomic_set(&ts->suspend_mode, 0);
	
	if (ts->pdata->powerOff3V3 && ts->pdata->power)
		ts->pdata->power(1);

#ifdef HX_CHIP_STATUS_MONITOR
	if(g_chip_monitor_data->HX_ON_HAND_SHAKING)//chip on hand shaking,wait hand shaking
	{
		for(t=0; t<100; t++)
			{
				if(g_chip_monitor_data->HX_ON_HAND_SHAKING == 0)//chip on hand shaking end
					{
						I("%s:HX_ON_HAND_SHAKING OK check %d times\n",__func__,t);
						break;
					}
				else
					msleep(1);
			}
		if(t==100)
			{
				E("%s:HX_ON_HAND_SHAKING timeout reject resume\n",__func__);
				return 0;
			}
	}
#endif

#if defined(HX_SMART_WAKEUP)||defined(HX_HIGH_SENSE)||defined(HX_USB_DETECT_GLOBAL)
#if defined(HX_RESEND_CMD)
		queue_delayed_work(ts->himax_resend_cmd_wq, &ts->resend_cmd_work, msecs_to_jiffies(1000));
#else
		himax_resend_cmd_func(ts->suspended);
#endif
#endif

	himax_resume_ic_action(ts->client);

	himax_int_enable(ts->client->irq,1);

#ifdef HX_CHIP_STATUS_MONITOR
	g_chip_monitor_data->HX_CHIP_POLLING_COUNT = 0;
	g_chip_monitor_data->HX_CHIP_MONITOR_EN = 1;
	queue_delayed_work(ts->himax_chip_monitor_wq, &ts->himax_chip_monitor, g_chip_monitor_data->HX_POLLING_TIMER*HZ); //for ESD solution
#endif
	printk("himax_chip_common_resume END \n");
	return 0;
}

