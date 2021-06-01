#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif

#include "lcm_drv.h"

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
/*#include <mach/mt_pm_ldo.h>*/
#ifdef CONFIG_MTK_LEGACY
#include <mach/mt_gpio.h>
#endif
#endif
#ifdef CONFIG_MTK_LEGACY
#include <cust_gpio_usage.h>
#endif
#ifndef CONFIG_FPGA_EARLY_PORTING
#if defined(CONFIG_MTK_LEGACY)
#include <cust_i2c.h>
#endif
#endif

#ifdef BUILD_LK
#define LCM_LOGI(string, args...)  dprintf(0, "[LK/"LOG_TAG"]"string, ##args)
#define LCM_LOGD(string, args...)  dprintf(1, "[LK/"LOG_TAG"]"string, ##args)
#else
#define LCM_LOGI(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#define LCM_LOGD(fmt, args...)  pr_debug("[KERNEL/"LOG_TAG"]"fmt, ##args)
#endif

#define LCM_ID_ILI7807D (0x7807)
extern int aeon_gpio_set(const char *name);
static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))
#define MDELAY(n)       (lcm_util.mdelay(n))
#define UDELAY(n)       (lcm_util.udelay(n))

#define dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update) \
	lcm_util.dsi_set_cmdq_V22(cmdq, cmd, count, ppara, force_update)
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
        lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) \
        lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) \
        lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) \
        lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) \
        lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#define set_gpio_lcd_enp(cmd) \
		lcm_util.set_gpio_lcd_enp_bias(cmd)
#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>


#ifndef CONFIG_FPGA_EARLY_PORTING

#define LP_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL    /* for I2C channel 0 */
#define I2C_ID_NAME "lp3101"
#define LP_ADDR 0x3E

#if defined(CONFIG_MTK_LEGACY)
static struct i2c_board_info lp3101_board_info __initdata = {I2C_BOARD_INFO(I2C_ID_NAME, LP_ADDR)};
#endif
#if !defined(CONFIG_MTK_LEGACY)
static const struct of_device_id lcm_of_match[] = {
		{.compatible = "mediatek,I2C_LCD_BIAS"},
		{},
};
#endif

static struct i2c_client *lp3101_i2c_client;

/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int lp3101_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int lp3101_remove(struct i2c_client *client);
extern int aeon_gpio_set(const char *name);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/
struct lp3101_dev	{	
	struct i2c_client *client;

};

static const struct i2c_device_id lp3101_id[] = {
	{I2C_ID_NAME, 0},
	{}
};

/* #if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)) */
/* static struct i2c_client_address_data addr_data = { .forces = forces,}; */
/* #endif */
static struct i2c_driver lp3101_iic_driver = {
	.id_table	= lp3101_id,
	.probe		= lp3101_probe,
	.remove		= lp3101_remove,
	/* .detect               = mt6605_detect, */
	.driver = {
		.owner = THIS_MODULE,
		.name	= "lp3101",
#if !defined(CONFIG_MTK_LEGACY)
			.of_match_table = lcm_of_match,
#endif
	},
};

/*****************************************************************************
 * Function
 *****************************************************************************/
static int lp3101_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	LCM_LOGI("lp3101_iic_probe\n");
	LCM_LOGI("LP: info==>name=%s addr=0x%x\n",client->name,client->addr);
	lp3101_i2c_client  = client;		
	return 0;
}

static int lp3101_remove(struct i2c_client *client)
{
	LCM_LOGI("lp3101_remove\n");
	lp3101_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

static int lp3101_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = lp3101_i2c_client;
	char write_data[2] = { 0 };
	if (client == NULL) {
		LCM_LOGI("i2c_client = NULL, skip lp3101_write_bytes\n");
		return 0;
	}
	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
	if (ret < 0)
		LCM_LOGI("lp3101 write data fail !!\n");
	return ret;
}

static int __init lp3101_iic_init(void)
{
	if(strstr(saved_command_line, "aeon_ft8761_fhd_dsi_vdo_x568") == NULL)
		return 0;

	LCM_LOGI( "lp3101_iic_init\n");
#if defined(CONFIG_MTK_LEGACY)
	i2c_register_board_info(LP_I2C_BUSNUM, &lp3101_board_info, 1);
#endif
	LCM_LOGI( "lp3101_iic_init2\n");
	i2c_add_driver(&lp3101_iic_driver);
	LCM_LOGI( "lp3101_iic_init success\n");	
	return 0;
}

static void __exit lp3101_iic_exit(void)
{
  LCM_LOGI( "lp3101_iic_exit\n");
  i2c_del_driver(&lp3101_iic_driver);  
}

module_init(lp3101_iic_init);
module_exit(lp3101_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK lp3101 I2C Driver");
MODULE_LICENSE("GPL");
#endif
#endif

/* static unsigned char lcd_id_pins_value = 0xFF; */
//static const unsigned char LCD_MODULE_ID = 0x01;
#define LCM_DSI_CMD_MODE									0
#define FRAME_WIDTH                                     (1080)
#define FRAME_HEIGHT                                    (1920)

#ifndef CONFIG_FPGA_EARLY_PORTING
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#endif

#define REGFLAG_DELAY       0xFFFC
#define REGFLAG_UDELAY  0xFFFB
#define REGFLAG_END_OF_TABLE    0xFFFD
#define REGFLAG_RESET_LOW   0xFFFE
#define REGFLAG_RESET_HIGH  0xFFFF

//static LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

struct LCM_setting_table {
	unsigned int cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
	{0x28,0,{}},
	{REGFLAG_DELAY, 50, {}},
	{0x10,0,{}},
	{REGFLAG_DELAY, 120, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_resume_setting[] = {
	{0x11,0,{}},
	{REGFLAG_DELAY, 150, {}},
	{0x29,0,{}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
#if 1
static struct LCM_setting_table init_setting[] = {
	 
	//		{0x01,0,{}},
	//		{REGFLAG_DELAY, 130, {}},

		//-------------  Display Setting  -------------------------
		//CMD2 ENABLE
		{0xff,3,{0x87,0x16,0x01}},
		{0x00,1,{0x80}},
		{0xff,2,{0x87,0x16}},
																																												   
		//TCON RTN Setting
		{0x00,1,{0x80}}, //C080                                                                                                                                         
		{0xC0,15,{0x00,0x7E,0x00,0x10,0x07,0x00,0x7E,0x10,0x10,0x00,0x7E,0x00,0x10,0x10,0x00}}, //

		{0x00,1,{0xA0}}, //C0A0                                                                                                                                         
		{0xC0,7,{0x0B,0x01,0x05,0x01,0x01,0x1B,0x07}},

		{0x00,1,{0xD0}}, //C0D0
		{0xC0,7,{0x0B,0x01,0x05,0x01,0x01,0x1B,0x07}},


		//A587 TP term SEL&XSEL setting 
		{0x00,1,{0x87}},                                                                                                                                        
		{0xA5,4,{0x00,0x07,0x77,0x77}},

		//LTPS INITIAL CODE
		// C280H 	LTPS VST Setting1  
		{0x00,1,{0x80}},                                                                                                                                        
		{0xC2,8,{0x82,0x00,0x14,0x0A,0x81,0x00,0x14,0x0A}},

		// C2B0H 	LTPS CKV Setting1
		{0x00,1,{0xB0}}, //C2B0                                                                                                                                         
		{0xC2,15,{0x00,0x01,0x00,0x14,0x00,0x01,0x02,0x00,0x14,0x00,0x82,0x00,0x00,0x14,0x00}},

		// C2C0H 	LTPS CKV Setting2
		{0x00,1,{0xC0}}, //C2C0                                                                                                                                         
		{0xC2,5,{0x81,0x00,0x00,0x14,0x00}},

		// C2DAH 	LTPS CKV Setting3
		{0x00,1,{0xDA}}, //C2DA                                                                                                                                         
		{0xC2,2,{0x33,0x33}},

		// C2DAH 	LTPS VEND Setting1
		{0x00,1,{0xE0}}, //C2E0                                                                                                                                         
		{0xC2,4,{0x05,0x00,0x00,0x03}},

		// C3AA --?
		{0x00,1,{0xAA}}, //C3AA                                                                                                                                         
		{0xC3,2,{0x9C,0x9C}},

		// C3D0H 	- LTPS GOFF Setting1
		{0x00,1,{0xD0}}, //C3D0                                                                                                                                         
		{0xC3,15,{0x00,0x02,0x02,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x00,0x00,0x6B,0x00,0x09}},

		//TP INITIAL CODE
		{0x00,1,{0x80}}, //CE80
		{0xCE,6,{0x25,0x00,0xA0,0x00,0x78,0xFF}},//ce80=25

		{0x00,1,{0x90}}, //CE90                                                                                                                                         
		{0xCE,8,{0x00,0x49,0x0E,0x53,0x00,0x49,0x00,0x6E}},

		{0x00,1,{0xB0}}, //CEB0                                                                                                                                         
		{0xCE,6,{0x00,0x00,0x60,0x79,0x00,0x60}},




		//PANELIF INITIAL CODE
		// CC80H 	-- Panel if initial code
		{0x00,1,{0x80}}, //CC80                                                                                                                                         
		{0xCC,12,{0x01,0x02,0x03,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E}},

		// CC90H - Panel IF D2U
		{0x00,1,{0x90}}, //CC90
		{0xCC,12,{0x01,0x03,0x02,0x09,0x08,0x07,0x06,0x0E,0x0D,0x0C,0x0B,0x0A}},


		// CCA0H - Panel IF 1
		{0x00,1,{0xA0}}, //CCA0
		{0xCC,15,{0x1A,0x1B,0x1C,0x1D,0x1E,0x1F,0x18,0x19,0x20,0x21,0x10,0x22,0x22,0x22,0x22}},

		//{0x00,0xB0}},
		//{0xCC,0x22,0x22,0x22,0x22}},

		// CB80H - Panel IF U2D
		{0x00,1,{0x80}}, //CB80
		{0xCB,8,{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},

		// CB90H - PANELIF ENMODE LOW
		{0x00,1,{0x90}}, //CB90
		{0xCB,15,{0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}},


		{0x00,1,{0xA0}},
		{0xCB,15,{0xF0,0xF0,0xF0,0x50,0x50,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00}},

		{0x00,1,{0xB0}},
		{0xCB,2,{0x00,0x00}},

		// CBC0H - PANELIF ENMODE HIGH1
		{0x00,1,{0xC0}}, //CBC0
		{0xCB,15,{0x00,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0x05,0xF5,0xF5,0xF5}},

		{0x00,1,{0xD0}},
		{0xCB,15,{0x05,0x05,0x05,0x55,0x55,0x05,0x05,0xF0,0x0F,0x30,0x05,0x05,0x05,0x05,0x05}},

		{0x00,1,{0xE0}},
		{0xCB,2,{0x05,0x05}},

		// CBF0H - PANELIF ENMODE LOW VOLTAGE DETECTION
		{0x00,1,{0xF0}}, //CBF0
		{0xCB,8,{0x00,0x00,0xFF,0x3F,0x50,0x30,0x03,0x00}},

		// CD80H - PANELIF CGOUT RIGHT1
		{0x00,1,{0x80}}, //CD80
		{0xCD,15,{0x05,0x07,0x24,0x24,0x24,0x24,0x01,0x03,0x17,0x24,0x12,0x11,0x10,0x0F,0x0E}},

		{0x00,1,{0x90}}, //CD90
		{0xCD,3,{0x0D,0x24,0x24}},

		// CDA0H PANELIF CGOUT LEFT1
		{0x00,1,{0xA0}}, //CDA0
		{0xCD,15,{0x04,0x06,0x24,0x24,0x24,0x13,0x14,0x02,0x24,0x24,0x12,0x11,0x10,0x0F,0x0E}},

		{0x00,1,{0xB0}}, //CDB0
		{0xCD,3,{0x0D,0x24,0x24}},

		// B3A0H  - ZigZag G-swap Soft panel setting
		{0x00,1,{0xA0}}, //B3A0
		{0xB3,7,{0x03,0x04,0x38,0x07,0x80,0x11,0x48}}, 

		// MIPI OSC Setting---? 
		{0x00,1,{0x86}}, //B086
		{0xB0,1,{0x0B}},

		// Power setting
		// C580H  - Pump & regulator
		{0x00,1,{0x80}},  //C580
		{0xC5,10,{0x00,0xC1,0xDD,0xC4,0x14,0x1E,0x00,0x55,0x50,0x03}}, // sx=3v

		// C590H  - power control setting
		{0x00,1,{0x90}},  //C590
		{0xC5,10,{0x44,0x24,0x14,0xC0,0x88,0x00,0x29,0x37,0x55,0x50}}, // VGH=10.1v  VGL=-10.5v

		// ---??
		{0x00,1,{0x90}}, 
		{0xCF,4,{0xFF,0x00,0xFE,0x00}}, 

		// D800H - GVDDP GVDDN
		{0x00,1,{0x00}},  //GVDD/NGVDD=+/-5.25
		{0xD8,2,{0x30,0x30}}, 

		// D900H - VCOM
		{0x00,1,{0x00}}, 
		{0xD9,5,{0x80,0x93,0x93,0x93,0x93}}, 

		{0x00,1,{0x00}},                                                                                                //                           
		{0xE1,24,{0x05,0x0f,0x25,0x3a,0x45,0x52,0x65,0x73,0x7a,0x83,0x8b,0x93,0x65,0x5e,0x5c,0x56,0x49,0x3f,0x32,0x28,0x21,0x15,0x0c,0x0B}},

		// E200 - Gamma1 - 2.2 
		{0x00,1,{0x00}},                                                                                                //
		{0xE2,24,{0x05,0x0f,0x25,0x3a,0x45,0x52,0x65,0x73,0x7a,0x83,0x8b,0x93,0x65,0x5e,0x5c,0x56,0x49,0x3f,0x32,0x28,0x21,0x15,0x0c,0x0B}},

		//Power Reducing
		{0x00,1,{0xC1}},  //VDD18%LVDSVDD
		{0xC5,1,{0x44}},             

		{0x00,1,{0x80}},  //SD_SAP 
		{0xC4,1,{0x41}}, 

		{0x00,1,{0x94}},  //IBIAS(GP,AP)
		{0xC5,1,{0x48}}, 

		{0x00,1,{0x95}},  //VGH/VGL Pump x2
		{0xC5,1,{0x00}}, 


		{0x00,1,{0xC0}}, 
	{0xC0,2,{0X01,0X10}}, 


		{0x00,1,{0x88}}, //B3A0
	{0xC3,2,{0x33,0x33}}, 


		{0x00,1,{0x98}}, //B3A0
	{0xC3,2,{0x33,0x33}},

		// SSD/SSDB output in V-blanking
		{0x00,1,{0x84}}, 
		{0xA5,1,{0x80}}, 

		{0x00,1,{0x84}}, 
	{0xCF,2,{0x10,0xA0}}, 


		//LCD_BUSY_200us
		{0x00,1,{0xD1}},
		{0xCF,4,{0x02,0x04,0x02,0xC9}},
		{0x00,1,{0xD7}},
		{0xCF,4,{0x02,0x04,0x02,0xC9}},

		{0x00,1,{0xD0}},
		{0xF6,3,{0x69,0x13,0x0C}},		
		

		{0x00,1,{0x94}},
		{0xCF,4,{0x00,0x00,0x10,0x20}},
		{0x00,1,{0xA4}},
		{0xCF,4,{0x00,0x07,0x01,0x80}},		

		{0x00,1,{0xD0}},
		{0xCF,1,{0x08}},
		
		//CMD2 DISABLE
		{0x00,1,{0x00}},
		{0xff,3,{0x00,0x00,0x00}},
		{0x00,1,{0x80}},
		{0xff,2,{0x00,0x00}},
		
		{0x11,0,{}},
	{REGFLAG_DELAY, 150, {}},
	
		{0x29,0,{}},
	{REGFLAG_DELAY, 10, {}},

	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

#endif


static void push_table(void *cmdq, struct LCM_setting_table *table,
	unsigned int count, unsigned char force_update)
{
	unsigned int i;
	unsigned cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;

		switch (cmd) {

			case REGFLAG_DELAY:
				if (table[i].count <= 10)
					MDELAY(table[i].count);
				else
					MDELAY(table[i].count);
				break;

			case REGFLAG_UDELAY:
				UDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
			dsi_set_cmdq_V22(cmdq, cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}


static void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}

static void lcm_get_params(LCM_PARAMS *params)
{
	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;

#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
	params->dsi.switch_mode = SYNC_PULSE_VDO_MODE;
#else
	params->dsi.mode = BURST_VDO_MODE;//BURST_VDO_MODE;
#endif
	LCM_LOGI("lcm_get_params lcm_dsi_mode %d\n", lcm_dsi_mode);
	params->dsi.switch_mode_enable = 0;

	/* DSI */
	/* Command mode setting */
	params->dsi.LANE_NUM = LCM_FOUR_LANE;
	/* The following defined the fomat for data coming from LCD engine. */
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	params->dsi.vertical_sync_active = 2;
	params->dsi.vertical_backporch = 16;
	params->dsi.vertical_frontporch = 16;
	params->dsi.vertical_active_line				= FRAME_HEIGHT;

	params->dsi.horizontal_sync_active = 4;
	params->dsi.horizontal_backporch = 32;
	params->dsi.horizontal_frontporch = 32;
	params->dsi.horizontal_active_pixel			= FRAME_WIDTH;
	params->dsi.ssc_disable = 1;
#ifndef CONFIG_FPGA_EARLY_PORTING
#if (LCM_DSI_CMD_MODE)
	params->dsi.PLL_CLOCK = 423;	/* this value must be in MTK suggested table */
#else
	params->dsi.PLL_CLOCK = 437;	/* this value must be in MTK suggested table */
#endif
#else
	params->dsi.pll_div1 = 0;
	params->dsi.pll_div2 = 0;
	params->dsi.fbk_div = 0x1;
#endif
	params->dsi.clk_lp_per_line_enable = 1;
	params->dsi.esd_check_enable = 1;
	params->dsi.customization_esd_check_enable = 1;
	params->dsi.lcm_esd_check_table[0].cmd = 0xAC;
	params->dsi.lcm_esd_check_table[0].count = 1;
	params->dsi.lcm_esd_check_table[0].para_list[0] = 0x00;

	
}

#ifdef BUILD_LK
#ifndef CONFIG_FPGA_EARLY_PORTING
#define lp3101_SLAVE_ADDR_WRITE  0x7C  
static struct mt_i2c_t lp3101_i2c;

static int lp3101_write_byte(kal_uint8 addr, kal_uint8 value)
{
	kal_uint32 ret_code = I2C_OK;
	kal_uint8 write_data[2];
	kal_uint16 len;

	write_data[0] = addr;
	write_data[1] = value;

    lp3101_i2c.id = 1; /* I2C2; */
	/* Since i2c will left shift 1 bit, we need to set FAN5405 I2C address to >>1 */
    lp3101_i2c.addr = (lp3101_SLAVE_ADDR_WRITE >> 1);
    lp3101_i2c.mode = ST_MODE;
    lp3101_i2c.speed = 100;
	len = 2;

    ret_code = i2c_write(&lp3101_i2c, write_data, len);
	/* printf("%s: i2c_write: ret_code: %d\n", __func__, ret_code); */

	return ret_code;
}
#else
/* extern int mt8193_i2c_write(u16 addr, u32 data); */
/* extern int mt8193_i2c_read(u16 addr, u32 *data); */

/* #define TPS65132_write_byte(add, data)  mt8193_i2c_write(add, data) */
/* #define TPS65132_read_byte(add)  mt8193_i2c_read(add) */

#endif
#endif

static void lcm_init_power(void)
{
/*#ifndef MACH_FPGA
#ifdef BUILD_LK
	mt6325_upmu_set_rg_vgp1_en(1);
#else
	LCM_LOGI("%s, begin\n", __func__);
	hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");
	LCM_LOGI("%s, end\n", __func__);
#endif
#endif*/
}

static void lcm_suspend_power(void)
{
/*#ifndef MACH_FPGA
#ifdef BUILD_LK
	mt6325_upmu_set_rg_vgp1_en(0);
#else
	LCM_LOGI("%s, begin\n", __func__);
	hwPowerDown(MT6325_POWER_LDO_VGP1, "LCM_DRV");
	LCM_LOGI("%s, end\n", __func__);
#endif
#endif*/
}

static void lcm_resume_power(void)
{
/*#ifndef MACH_FPGA
#ifdef BUILD_LK
	mt6325_upmu_set_rg_vgp1_en(1);
#else
	LCM_LOGI("%s, begin\n", __func__);
	hwPowerOn(MT6325_POWER_LDO_VGP1, VOL_DEFAULT, "LCM_DRV");
	LCM_LOGI("%s, end\n", __func__);
#endif
#endif*/
}

static void lcm_init(void)
{
//	unsigned int data_array[16];
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
#ifndef CONFIG_FPGA_EARLY_PORTING
	int ret = 0;
#endif
	cmd = 0x00;
	data = 0x12;
	SET_RESET_PIN(0);
	aeon_gpio_set("aeon_lcd_bias1");

#ifndef CONFIG_FPGA_EARLY_PORTING

	MDELAY(20);

	ret=lp3101_write_bytes(cmd,data);


	if (ret < 0)
		LCM_LOGI("FT8716----lp3101---cmd=%0x-- i2c write error-----\n",cmd);
	else
		LCM_LOGI("FT8716----lp3101---cmd=%0x-- i2c write success-----\n",cmd);
	cmd = 0x01;
	data = 0x12;

	ret=lp3101_write_bytes(cmd,data);


	if (ret < 0)
		LCM_LOGI("FT8716----lp3101----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("FT8716----lp3101----cmd=%0x--i2c write success----\n", cmd);

#endif
	SET_RESET_PIN(1);
	MDELAY(50);
	SET_RESET_PIN(0);
	MDELAY(50);

	SET_RESET_PIN(1);
	MDELAY(100);
//lcm_init_registers();
	push_table(0,init_setting, sizeof(init_setting) / sizeof(struct LCM_setting_table), 1);
}

static void lcm_suspend(void)
{
	printk(" FT8716 lcm_suspend enter!\n");
	push_table(0, lcm_suspend_setting, sizeof(lcm_suspend_setting) / sizeof(struct LCM_setting_table), 1);
	MDELAY(10);
	
	//SET_RESET_PIN(1);
	//MDELAY(50);
	//SET_RESET_PIN(0);
	//MDELAY(50);
	//set_gpio_lcd_enp(0);
        aeon_gpio_set("aeon_lcd_bias0");
	/* SET_RESET_PIN(0); */
}

static void lcm_resume(void)
{
		//lcm_init();
	aeon_gpio_set("aeon_lcd_bias1");
	MDELAY(10);
	push_table(0, lcm_resume_setting, sizeof(lcm_resume_setting) / sizeof(struct LCM_setting_table), 1);
}


static unsigned int lcm_compare_id(void)
{
	#if 0
	unsigned char buffer[5];
	unsigned int array[16];   
	unsigned char cmd = 0x0;
	unsigned char data = 0xFF;
	int ret = 0;
	cmd = 0x00;
	data = 0x12;

	SET_RESET_PIN(0);
	MDELAY(50);
#ifndef MACH_FPGA
	mt_set_gpio_mode(GPIO_65132_EN, GPIO_MODE_00);
	mt_set_gpio_dir(GPIO_65132_EN, GPIO_DIR_OUT);
	mt_set_gpio_out(GPIO_65132_EN, GPIO_OUT_ONE);
	MDELAY(20);
#ifdef BUILD_LK
	ret = lp3101_write_byte(cmd, data);
#else
	ret = lp3101_write_bytes(cmd,data);
#endif

	if (ret < 0)
		LCM_LOGI("FT8716----FT8716----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("FT8716----FT8716----cmd=%0x--i2c write success----\n", cmd);
	MDELAY(20);

	cmd = 0x01;
	data = 0x12;

#ifdef BUILD_LK
	ret = lp3101_write_byte(cmd,data); 		
#else
	ret = lp3101_write_bytes(cmd,data);
#endif

	if (ret < 0)
		LCM_LOGI("FT8716----FT8716----cmd=%0x--i2c write error----\n", cmd);
	else
		LCM_LOGI("FT8716----FT8716----cmd=%0x--i2c write success----\n", cmd);

#endif
	SET_RESET_PIN(1);	
	MDELAY(50);
	SET_RESET_PIN(0);
	MDELAY(50);

	SET_RESET_PIN(1);
	MDELAY(100);	
	array[0] = 0x00053700;  /* read id return two byte,version and id */
	dsi_set_cmdq(array, 1, 1);

	read_reg_v2(0xA1, buffer, 5);
	MDELAY(20);
	LCM_LOGI("FT8716 buffer[0] = 0x%x, buffer[1]=0x%x, buffer[2]=0x%x\n", buffer[0], buffer[1], buffer[2]);
	if((buffer[0] == 0x01) && (buffer[0] == 0x8B))
	{
		return 1;
	}	
	else
	{
		return 0;
	}
	#endif
	return 0;
}


/* return TRUE: need recovery */
/* return FALSE: No need recovery */




LCM_DRIVER aeon_ft8761_fhd_dsi_vdo_x568_lcm_drv = 
{
    .name			= "ft8716_lead",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params     = lcm_get_params,
	.init           = lcm_init,
	.compare_id = lcm_compare_id,
	.init_power = lcm_init_power,
	.resume_power = lcm_resume_power,
	.suspend_power = lcm_suspend_power,
	.resume         = lcm_resume,
	.suspend        = lcm_suspend,
};
