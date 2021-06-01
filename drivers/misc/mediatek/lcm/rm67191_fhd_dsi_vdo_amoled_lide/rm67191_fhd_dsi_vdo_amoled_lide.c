#ifndef BUILD_LK
#include <linux/string.h>
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
//#include <mt-plat/mt_pm_ldo.h>
#include <mt-plat/mt_gpio.h>
#endif
#ifdef BUILD_LK
#define LCD_DEBUG(fmt)  dprintf(CRITICAL, fmt)
#else
#define LCD_DEBUG(fmt)  pr_debug(fmt)
#endif

// ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------

#define FRAME_WIDTH  (1080)
#define FRAME_HEIGHT (1920)



// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------

static LCM_UTIL_FUNCS lcm_util;
//static int  VOL_2V9=20;
//static int currentBrightness;

#define SET_RESET_PIN(v)    (lcm_util.set_reset_pin((v)))

#define UDELAY(n) (lcm_util.udelay(n))
#define MDELAY(n) (lcm_util.mdelay(n))

#define REGFLAG_DELAY             								0xFC
#define REGFLAG_UDELAY             								0xFB

#define REGFLAG_END_OF_TABLE      							    	0xFD  

// ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------

#define dsi_set_cmd_by_cmdq_dual(handle, cmd, count, ppara, force_update) \
		lcm_util.dsi_set_cmdq_V23(handle, cmd, (unsigned char)(count), \
					  (unsigned char *)(ppara), (unsigned char)(force_update))
#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) \
    lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update)	lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd)					lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums)		lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd)					lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size)		lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)
#define dsi_swap_port(swap)				lcm_util.dsi_swap_port(swap)
#define dsi_lcm_set_gpio_out(pin, out)						(lcm_util.set_gpio_out(pin, out))
#define dsi_lcm_set_gpio_mode(pin, mode)					(lcm_util.set_gpio_mode(pin, mode))
#define dsi_lcm_set_gpio_dir(pin, dir)						(lcm_util.set_gpio_dir(pin, dir))
#define dsi_lcm_set_gpio_pull_enable(pin, en)				(lcm_util.set_gpio_pull_enable(pin, en))



#define   LCM_DSI_CMD_MODE							(1)

struct LCM_setting_table {
    unsigned char cmd;
    unsigned char count;
    unsigned char para_list[64];
};

static struct LCM_setting_table lcm_initialization_setting[] = {
#if 0
	{0xFE, 1,  {0x0E}},
	{0x4B, 1,  {0x00}},	
	{0x4C, 1,  {0x0F}},	
	{0x4D, 1,  {0x20}},	
	{0x4E, 1,  {0x40}},	
	{0x4F,1,{0x60}},                                                           
	{0x50,1,{0xA0}},                                                           
	{0x51,1,{0xC0}},                                                                                                                      
	{0x52,1,{0xE0}},                                                                                                                      																																		 
	{0x53,1,{0xFF}}, 
	{REGFLAG_DELAY, 20, {}},	                                                          
	{0xFE,1,{0x0D}},                                                           
	{0x42,1,{0x00}},                                                                                                                      
	{0x18,1,{0x08}},  
	{0x08,1,{0x41}},                                                           
	{0x46,1,{0x02}},                                                           
	{0x72,1,{0x09}},                                                                                                                      
	{0x1E,1,{0x04}},  
	{0x1E,1,{0x00}},    	
	{REGFLAG_DELAY, 20, {}},
	{0xFE,1,{0x0A}},                                                           
	{0x24,1,{0x17}},                                                           
	{0x04,1,{0x07}},                                                                                                                      
	{0x1A,1,{0x0C}},                                                                                                                      																																		 
	{0x0F,1,{0x44}}, 
	{REGFLAG_DELAY, 20, {}},	                                                          
	{0xFE,1,{0x0B}},                                                           
	{0x28,1,{0x40}},                                                                                                                      
	{0x29,1,{0x4F}}, 
	{REGFLAG_DELAY, 20, {}},	 
	{0xFE,1,{0x04}},                                                           
	{0x00,1,{0x0C}},                                                           
	{0x05,1,{0x10}},                                                                                                                      
	{0x06,1,{0x08}}, 	
	{0x08,1,{0x10}},                                                           
	{0x09,1,{0x08}},                                                           
	{0x0A,1,{0xE6}},                                                                                                                      
	{0x0B,1,{0x8C}},                                                                                                                      																																		 
	{0x1A,1,{0x12}},                                                           
	{0x1E,1,{0xE0}},///E8                                                           
	{0x29,1,{0x93}},  	
	{0x2A,1,{0x93}},                                                           
	{0x2F,1,{0x02}},                                                           
	{0x31,1,{0x02}},                                                                                                                      
	{0x33,1,{0x05}},                                                                                                                      																																		 
	{0x37,1,{0x2D}},                                                           
	{0x38,1,{0x2D}},                                                           
	{0x3A,1,{0x1E}},                                                                                                                      
	{0x3B,1,{0x1E}},  	
	{0x3D,1,{0x27}},                                                           
	{0x3F,1,{0x80}},                                                           
	{0x40,1,{0x40}},                                                                                                                      
	{0x41,1,{0xE0}},                                                                                                                      																																		 
	{0x4F,1,{0x2F}},                                                           
	{0x50,1,{0x1E}},  //14 elvss -3.5v
	{REGFLAG_DELAY, 20, {}},		                                                         
	{0xFE,1,{0x06}},                                                                                                                      
	{0x00,1,{0xEC}},  
	{0x05,1,{0x06}},                                                           
	{0x07,1,{0xA2}},                                                           
	{0x08,1,{0xEC}},                                                                                                                      
	{0x0D,1,{0x04}},  
	{0x0F,1,{0xA2}},                                                           
	{0x32,1,{0xEC}},                                                           
	{0x37,1,{0x07}},                                                                                                                      
	{0x39,1,{0x83}},  
	{0x3A,1,{0xEC}},                                                           
	{0x41,1,{0x06}},                                                           
	{0x43,1,{0x83}}, 		
	{0x44,1,{0xEC}},                                                           
	{0x49,1,{0x06}},                                                           
	{0x4B,1,{0xA2}},                                                                                                                      
	{0x4C,1,{0xEC}},                                                                                                                      																																		 
	{0x51,1,{0x04}},                                                           
	{0x53,1,{0xA2}},                                                           
	{0x75,1,{0xEC}},                                                                                                                      
	{0x7A,1,{0x07}},  
	{0x7C,1,{0x83}},
	
	{0x7D,1,{0xEC}},	
	                                                           
	{0x82,1,{0x06}},                                                           
	{0x84,1,{0x83}},                                                                                                                      
	{0x85,1,{0xEC}},  
	{0x86,1,{0x0F}},                                                           
	{0x87,1,{0xFF}},                                                           
	{0x88,1,{0x00}},                                                                                                                      
	{0x8A,1,{0x02}},  
	{0x8C,1,{0xA2}},                                                           
	{0x8D,1,{0xEE}},                                                           
	{0x8E,1,{0x01}},                                                                                                                      
	{0x8F,1,{0xE8}},  
	{0x90,1,{0x0A}},                                                           
	{0x92,1,{0x06}},                                                           
	{0x93,1,{0xA0}},                                                                                                                      
	{0x94,1,{0xA8}},  
	{0x95,1,{0xEC}},                                                           
	{0x96,1,{0x0F}},                                                           
	{0x97,1,{0xFF}},                                                                                                                      
	{0x98,1,{0x00}},  
	{0x9A,1,{0x02}},                                                           
	{0x9C,1,{0xA2}},                                                           
	{0xA6,1,{0x21}},                                                                                                                      
	{0xA7,1,{0x05}},
	{0xA9,1,{0x06}},                                                           
	{0xAC,1,{0x04}}, 	
	{0xB1,1,{0x12}},                                                           
	{0xB2,1,{0x17}},                                                           
	{0xB3,1,{0x17}},                                                                                                                      
	{0xB4,1,{0x17}},                                                                                                                      																																		 
	{0xB5,1,{0x17}},                                                           
	{0xB6,1,{0x11}},                                                           
	{0xB7,1,{0x08}},                                                                                                                      
	{0xB8,1,{0x09}},  
	{0xB9,1,{0x06}},                                                           
	{0xBA,1,{0x07}},                                                           
	{0xBB,1,{0x17}}, 
	{0xBC,1,{0x17}},                                                           
	{0xBD,1,{0x17}},                                                           
	{0xBF,1,{0x17}},                                                                                                                      
	{0xC0,1,{0x17}},                                                                                                                      																																		 
	{0xC1,1,{0x17}},                                                           
	{0xC2,1,{0x17}},                                                           
	{0xC3,1,{0x17}},                                                                                                                      
	{0xC4,1,{0x0F}},  
	{0xC5,1,{0x0E}},                                                           
	{0xC6,1,{0x00}},                                                           
	{0xC7,1,{0x01}},                                                                                                                      
	{0xC8,1,{0x10}},  
	{REGFLAG_DELAY, 20, {}},	
	{0xFE,1,{0x00}},                                                           
	{0xC2,1,{0x08}},   //03                                                        
	{0x35,1,{0x00}},                                                                                                                      
	{0x44,2,{0x33,0x80}},  
	{0x11,1,{0x00}},  
	{REGFLAG_DELAY, 200, {}},	
	{0x29, 1,  {0x00}},
	{REGFLAG_DELAY, 120, {}},	

#else 
	{0xFE, 1,  {0x0D}},
	{0x20, 1,  {0x00}},	
	{0x21, 1,  {0x00}},	
	{0xFE, 1,  {0x0E}},
	{0x4B, 1,  {0x00}},	
	{0x4C, 1,  {0x0F}},	
	{0x4D, 1,  {0x20}},	
	{0x4E, 1,  {0x40}},	
	{0x4F,1,{0x60}},                                                           
	{0x50,1,{0xA0}},                                                           
	{0x51,1,{0xC0}},                                                                                                                      
	{0x52,1,{0xE0}},                                                                                                                      																																		 
	{0x53,1,{0xFF}}, 
	{0xFE,1,{0x0D}},
	{0x18,1,{0x08}},
	{0x42,1,{0x01}},
	{0x08,1,{0x41}},
	{0x46,1,{0x02}},
	{0xFE,1,{0x0A}},
	{0x24,1,{0x17}},
	{0x04,1,{0x07}},
	{0x1A,1,{0x0C}},
	{0x0F,1,{0x44}},
	{0xFE,1,{0x04}},
	{0x00,1,{0x08}},
	{0x05,1,{0x14}},
	{0x06,1,{0x08}},
	{0x01,1,{0x14}},
	{0x08,1,{0x08}},
	{0x09,1,{0x14}},
	{0x10,1,{0x08}},
	{0x0A,1,{0xE6}},
	{0x0B,1,{0x8C}},
	{0x1A,1,{0x12}},
	{0x29,1,{0x93}},
	{0x2A,1,{0x93}},
	{0x2F,1,{0x02}},
	{0x31,1,{0x02}},
	{0x33,1,{0x05}},
	{0x37,1,{0x2D}},
	{0x38,1,{0x2D}},
	{0x3A,1,{0x1E}},
	{0x3B,1,{0x1E}},
	{0x3D,1,{0x27}},
	{0x3F,1,{0x80}},
	{0x40,1,{0x40}},
	{0x41,1,{0xE0}},
	{0x4F,1,{0x1E}},
	{0x50,1,{0x2F}},
	{0xFE,1,{0x06}},
	{0x07,1,{0xA2}},
	{0x0F,1,{0xA2}},
	{0x39,1,{0x83}},
	{0x43,1,{0x83}},
	{0x4B,1,{0xA2}},
	{0x53,1,{0xA2}},
	{0x7C,1,{0x83}},
	{0x84,1,{0x83}},
	{0x85,1,{0xEC}},
	{0x86,1,{0x0F}},
	{0x87,1,{0xFF}},
	{0x88,1,{0x00}},
	{0x8C,1,{0xA2}},
	{0xFE,1,{0x06}},
	{0x95,1,{0xEC}},
	{0x96,1,{0x0F}},
	{0x97,1,{0xFF}},
	{0x98,1,{0x00}},
	{0x9C,1,{0xA2}},
	{0xFE,1,{0x06}},
	{0xB1,1,{0x12}},
	{0xB2,1,{0x17}},
	{0xB3,1,{0x17}},
	{0xB4,1,{0x17}},
	{0xB5,1,{0x17}},
	{0xB6,1,{0x13}},
	{0xB7,1,{0x08}},
	{0xB8,1,{0x09}},
	{0xB9,1,{0x06}},
	{0xBA,1,{0x07}},
	{0xBB,1,{0x17}},
	{0xBC,1,{0x17}},
	{0xBD,1,{0x17}},
	{0xBE,1,{0x17}},
	{0xBF,1,{0x17}},
	{0xC0,1,{0x17}},
	{0xC1,1,{0x17}},
	{0xC2,1,{0x17}},
	{0xC3,1,{0x17}},
	{0xC4,1,{0x0F}},
	{0xC5,1,{0x0E}},
	{0xC6,1,{0x00}},
	{0xC7,1,{0x01}},
	{0xC8,1,{0x10}},
///////////////esd ADD///////////////////
	{0xFE, 1, {0x06}},
	{0x95, 1, {0xEC}},
	{0x8D, 1, {0xEE}},
	{0x44, 1, {0xEC}},
	{0x4C, 1, {0xEC}},
	{0x32, 1, {0xEC}},
	{0x3A, 1, {0xEC}},// 13
	{0x7D, 1, {0xEC}},
	{0x75, 1, {0xEC}},// 
	{0x00, 1, {0xEC}},
	{0x08, 1, {0xEC}},
	{0x85, 1, {0xEC}},
	{0xA6, 1, {0x21}},
	{0xA7, 1, {0x05}},
	{0xA9, 1, {0x06}},
	{0x82, 1, {0x06}},
	{0x41, 1, {0x06}},
	{0x7A, 1, {0x07}},
	{0x37, 1, {0x07}},
	{0x05, 1, {0x06}},
	{0x49, 1, {0x06}},
	{0x0D, 1, {0x04}},
	{0x51, 1, {0x04}},
///////////////esd ADD///////////////////	
	{0xFE,1,{0x06}},
	{0x44,1,{0xCC}},
	{0x4C,1,{0xCC}},
	{0x32,1,{0xCC}},
	{0x3A,1,{0xCC}},
	{0x7D,1,{0xCC}},
	{0x75,1,{0xCC}},
	{0x00,1,{0xCC}},
	{0x08,1,{0xCC}},
	{0x9D,1,{0xEA}},
	{0x9E,1,{0x01}},
	{0x9F,1,{0xE8}},
	{0xA0,1,{0x06}},
	{0xA4,1,{0xA0}},
	{0xA3,1,{0x06}},
	{0xA5,1,{0x60}},
	{0x9A,1,{0x02}},
	{0x49,1,{0x05}},
	{0x51,1,{0x03}},
	{0x37,1,{0x05}},
	{0x41,1,{0x04}},
	{0x8A,1,{0x02}},
	{0x05,1,{0x05}},
	{0x0D,1,{0x03}},
	{0x7A,1,{0x03}},
	{0x82,1,{0x02}},
	{0xFE,1,{0x00}},
	{0x51,1,{0x0F}},
	{0xC2,1,{0x08}},
	{0x35,1,{0x00}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0x29,1,{0x00}},
	{REGFLAG_DELAY, 50, {}},
	{0xFE, 1, {0x0F}},	
	{REGFLAG_DELAY, 100, {}},

#endif	
	
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};
static struct LCM_setting_table lcm_poweron_setting[] = {

	{0xFE, 1,  {0x07}},
	{0xA9, 1,  {0xFA}},
	//{REGFLAG_UDELAY, 16667, {}},
	{REGFLAG_DELAY, 17, {}},	
	{0xFE, 1,  {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}	
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
{0x51, 1, {0xFF}},
{REGFLAG_END_OF_TABLE, 0x00, {}}
};
/*
static struct LCM_setting_table lcm_poweroff_setting[] = {

	{0xFE, 1,  {0x07}},
	{0xA9, 1,  {0x6A}},
	//{REGFLAG_UDELAY, 16667, {}},
	{REGFLAG_DELAY, 17, {}},
	{0xFE, 1,  {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}	
};*/
static struct LCM_setting_table lcm_sleepout[] = {

	{0x11, 1,  {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}	
};
static struct LCM_setting_table lcm_sleepin[] = {

	{0x10, 1,  {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}	
};
static struct LCM_setting_table lcm_displayon[] = {

	{0x29, 1,  {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}	
};
static struct LCM_setting_table lcm_displayoff[] = {

	{0x28, 1,  {0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}	
};

static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
    unsigned int i;

    for(i = 0; i < count; i++)
    {
        unsigned cmd;
        cmd = table[i].cmd;

        switch (cmd) {

            	case REGFLAG_DELAY :
              		if(table[i].count <= 10)
                    		MDELAY(table[i].count);
                	else
                    		MDELAY(table[i].count);
                	break;
				
		case REGFLAG_UDELAY :
			UDELAY(table[i].count);
			break;

            	case REGFLAG_END_OF_TABLE :
                	break;

            	default:
                	dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
        }
    }
}

// ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------

static void lcm_set_util_funcs(const LCM_UTIL_FUNCS * util)
{
	memcpy(&lcm_util, util, sizeof(LCM_UTIL_FUNCS));
}


static void lcm_get_params(LCM_PARAMS * params)
{

	memset(params, 0, sizeof(LCM_PARAMS));

	params->type = LCM_TYPE_DSI;

	params->width = FRAME_WIDTH;
	params->height = FRAME_HEIGHT;
	params->physical_width = 68;
	params->physical_height = 121;
#if (LCM_DSI_CMD_MODE)
	params->dsi.mode = CMD_MODE;
#else
	params->dsi.mode = BURST_VDO_MODE;//BURST_VDO_MODE;
#endif
	params->dsi.switch_mode_enable = 0;
	params->dsi.noncont_clock = true;
	params->dsi.cont_clock = 0;
	params->dsi.clk_lp_per_line_enable = 1;
	// 1 Three lane or Four lane
	params->dsi.LANE_NUM				= LCM_FOUR_LANE;
	
	//The following defined the fomat for data coming from LCD engine.
	params->dsi.data_format.color_order = LCM_COLOR_ORDER_RGB;
	params->dsi.data_format.trans_seq = LCM_DSI_TRANS_SEQ_MSB_FIRST;
	params->dsi.data_format.padding = LCM_DSI_PADDING_ON_LSB;
	params->dsi.data_format.format = LCM_DSI_FORMAT_RGB888;

	/* Highly depends on LCD driver capability. */
	params->dsi.packet_size = 256;
	params->dsi.ssc_disable = 1;
	//params->dsi.ssc_range = 3;
	/* video mode timing */

	params->dsi.PS = LCM_PACKED_PS_24BIT_RGB888;

	// Bit rate calculation
	//1 Every lane speed
	params->dsi.PLL_CLOCK = 428;//?????????????200~220
	
params->dsi.esd_check_enable =1;
params->dsi.customization_esd_check_enable =0;
params->dsi.lcm_esd_check_table[0].cmd = 0x0a;
params->dsi.lcm_esd_check_table[0].count =1;
params->dsi.lcm_esd_check_table[0].para_list[0] = 0x9c;	
//params->dsi.lcm_esd_check_table[1].cmd = 0x03;
//params->dsi.lcm_esd_check_table[1].count =1;
//params->dsi.lcm_esd_check_table[1].para_list[0] = 0x72;
//params->dsi.lcm_esd_check_table[2].cmd = 0x06;
//params->dsi.lcm_esd_check_table[2].count =1;
//params->dsi.lcm_esd_check_table[2].para_list[0] = 0x09;

}
extern int aeon_gpio_set(const char *name);
static void lcm_init(void)
{	


#ifdef BUILD_LK
	//mt6331_upmu_set_rg_vgp1_en(1);
	//pmic_set_register_value(PMIC_RG_VGP1_VOSEL,5);
	//pmic_set_register_value(PMIC_RG_VGP1_EN,1);
#else
	//hwPowerOn(MT6331_POWER_LDO_VGP1, VOL_3000, "LCM_DRV");
#endif
	char buf;
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);	
	MDELAY(10);	
	SET_RESET_PIN(1);
	MDELAY(20);	

	
	push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);	
	MDELAY(20);

	push_table(lcm_backlight_level_setting, sizeof(lcm_backlight_level_setting) / sizeof(struct LCM_setting_table), 1);	
	MDELAY(20);
	
	push_table(lcm_sleepout, sizeof(lcm_sleepout) / sizeof(struct LCM_setting_table), 1);	
	MDELAY(50);// time must equal to 50

	push_table(lcm_displayon, sizeof(lcm_displayon) / sizeof(struct LCM_setting_table), 1);	
       MDELAY(10);
		
	
	MDELAY(50);		//20
	//set VENG to -2.9V
	
	push_table(lcm_poweron_setting, sizeof(lcm_poweron_setting) / sizeof(struct LCM_setting_table), 1);	
	read_reg_v2(0x0a, &buf, 2);
	#if defined(BUILD_LK)
	printf("*******lcm= lk lide rm67191***************  buf=%x\n",buf);
	#else
	printk("*******lcm= kernel lide rm67191***************  buf=%x\n",buf);
	#endif
}

static void lcm_suspend(void)
{

	push_table(lcm_displayoff, sizeof(lcm_displayoff) / sizeof(struct LCM_setting_table), 1);	
	MDELAY(120);	
	

	//push_table(lcm_poweroff_setting, sizeof(lcm_poweroff_setting) / sizeof(struct LCM_setting_table), 1);	
	//MDELAY(10);	
	
	push_table(lcm_sleepin, sizeof(lcm_sleepin) / sizeof(struct LCM_setting_table), 1);	
	MDELAY(20);	


}


static void lcm_resume(void)
{
#if 0
	int i;
	unsigned char tempBrightness;

	for(i=0;;i++)
	{	
		if(lcm_initialization_setting[i].cmd == 0x51)//brightness ctrl
		{	    
			tempBrightness = lcm_initialization_setting[i].para_list[0];
			lcm_initialization_setting[i].para_list[0] = currentBrightness;
			break;
		}
	}    
	//lcm_init();
	for(i=0;;i++)
	{
		if(lcm_initialization_setting[i].cmd == 0x51)//brightness ctrl
		{	    
			lcm_initialization_setting[i].para_list[0] = tempBrightness;
			break;
		}    
	}
#endif
/*
	//aeon_gpio_set("aeon_lcm_ldo1");//GPIO139 high
	//MDELAY(10);
	aeon_gpio_set("aeon_lcm_ldo0");//GPIO139 low
	MDELAY(10);
	aeon_gpio_set("aeon_lcm_ldo1");//GPIO139 high
	MDELAY(20);
	//lcm_init();
	
	SET_RESET_PIN(1);
	MDELAY(10);
	SET_RESET_PIN(0);	
	MDELAY(10);	
	SET_RESET_PIN(1);
	MDELAY(120);	
*/		
	//char buf;	
	//MDELAY(120);
	push_table(lcm_displayon, sizeof(lcm_displayon) / sizeof(struct LCM_setting_table), 1);	
    MDELAY(10);

	//push_table(lcm_initialization_setting, sizeof(lcm_initialization_setting) / sizeof(struct LCM_setting_table), 1);	
	//MDELAY(20);	

	push_table(lcm_sleepout, sizeof(lcm_sleepout) / sizeof(struct LCM_setting_table), 1);	
	MDELAY(50);// time must equal to 50


	   	
	//push_table(lcm_poweron_setting, sizeof(lcm_poweron_setting) / sizeof(struct LCM_setting_table), 1);	
/*
	read_reg_v2(0x0a, &buf, 2);
	#if defined(BUILD_LK)
	printf("*******lcm= lk lide rm67191***************  buf=%x\n",buf);
	#else
	printk("*******lcm= kernel lide rm67191***************  buf=%x\n",buf);
	#endif*/
}

#if (LCM_DSI_CMD_MODE)
static void lcm_update(unsigned int x, unsigned int y,
		       unsigned int width, unsigned int height)
{
	unsigned int x0 = x;
	unsigned int y0 = y;
	unsigned int x1 = x0 + width - 1;
	unsigned int y1 = y0 + height - 1;

	unsigned char x0_MSB = ((x0 >> 8) & 0xFF);
	unsigned char x0_LSB = (x0 & 0xFF);
	unsigned char x1_MSB = ((x1 >> 8) & 0xFF);
	unsigned char x1_LSB = (x1 & 0xFF);
	unsigned char y0_MSB = ((y0 >> 8) & 0xFF);
	unsigned char y0_LSB = (y0 & 0xFF);
	unsigned char y1_MSB = ((y1 >> 8) & 0xFF);
	unsigned char y1_LSB = (y1 & 0xFF);

	unsigned int data_array[16];

	data_array[0] = 0x00053902;
	data_array[1] =
	    (x1_MSB << 24) | (x0_LSB << 16) | (x0_MSB << 8) | 0x2a;
	data_array[2] = (x1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x00053902;
	data_array[1] =
	    (y1_MSB << 24) | (y0_LSB << 16) | (y0_MSB << 8) | 0x2b;
	data_array[2] = (y1_LSB);
	dsi_set_cmdq(data_array, 3, 1);

	data_array[0] = 0x002c3909;
	dsi_set_cmdq(data_array, 1, 0);

}
#endif

#if 0
static struct LCM_setting_table id_read[] = {
/*	{0x01,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
	{0xFF,5,{0x77,0x01,0x00,0x00,0x11}},
	{0xD1,1,{0x11}},
	{0x11,1,{0x00}},
	{REGFLAG_DELAY, 120, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}
*/
	{0xFE,1,{0x0D}},
	{0x1E,1,{0x40}},
	{0x1E,1,{0x00}},
	{REGFLAG_DELAY, 20, {}},
    {REGFLAG_END_OF_TABLE, 0x00, {}}

};
#endif


static unsigned int lcm_compare_id(void)
{
//#ifdef BUILD_LK

		char id1,id2,id3;
    //mt_set_gpio_mode(GPIO_LCD_ID_PIN, GPIO_LCM_ID_M_GPIO);
    //mt_set_gpio_mode(GPIO_LCD_ID_PIN, GPIO_MODE_00);
	//mt_set_gpio_dir(GPIO_LCD_ID_PIN, GPIO_DIR_IN);		
	//mt_set_gpio_pull_enable(GPIO_LCD_ID_PIN,GPIO_PULL_DISABLE);
	//mt_set_gpio_pull_select(GPIO_LCD_ID_PIN,GPIO_PULL_DOWN);
    //MDELAY(1);
	
        //Do reset here
        SET_RESET_PIN(1);
	    MDELAY(20);
	    SET_RESET_PIN(0);
        MDELAY(50);
	    SET_RESET_PIN(1);
	    MDELAY(120); 

	//push_table(id_read, sizeof(id_read) / sizeof(struct LCM_setting_table), 1);
	//MDELAY(20);
		read_reg_v2(0xe1, &id1, 2);
		read_reg_v2(0xe2, &id2, 2);
		read_reg_v2(0xe3, &id3, 2);


	#if defined(BUILD_LK)
	printf("*******lk lide rm67191***************  id1=%x,id2=%x,id3=%x\n", id1,id2,id3);
	#else
	printk("*******kernel lide rm67191***************  id1=%x,id2=%x,id3=%x\n", id1,id2,id3);
	#endif
	//id1=67,id2=19,id3=b
	return (id1 == 0x67) ? 1: 0;
	//return 1;

}

static void lcm_setbacklight_cmdq(void* handle,unsigned int level)
{
	unsigned int cmd = 81;
	unsigned int count =1;
 	unsigned int currentBrightness = level;
 	dsi_set_cmd_by_cmdq_dual(handle, cmd, count, &currentBrightness, 1);
#ifdef BUILD_LK
 	dprintf(0,"%s,lk rm67191 backlight: level = %d\n", __func__, level);
#else
 	printk("%s, kernel rm67191 backlight: level = %d\n", __func__, level);
#endif
 // Refresh value of backlight level.
} 

LCM_DRIVER rm67191_fhd_dsi_vdo_amoled_lide_lcm_drv = {
	.name = "rm67191_fhd_dsi_vdo_amoled_lide",
	.set_util_funcs = lcm_set_util_funcs,
	.get_params = lcm_get_params,
	.init = lcm_init,
	.suspend = lcm_suspend,
	.resume = lcm_resume,
	.compare_id    = lcm_compare_id,
#if (LCM_DSI_CMD_MODE)
	.update = lcm_update,
#endif
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
};
