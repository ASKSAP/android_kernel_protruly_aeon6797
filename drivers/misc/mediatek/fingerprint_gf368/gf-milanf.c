#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/spi/spi.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/ioctl.h>
#include <linux/input.h>   
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/compat.h>
#include <linux/poll.h>
#include <linux/sort.h>
#include <linux/types.h>
#include <linux/version.h>
#include <linux/spi/spidev.h>
#include <linux/spi/spi.h>
#include <linux/usb.h>
#include <linux/usb/ulpi.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/power_supply.h>
#include <linux/gpio.h>
//#include <mach/mt_gpio.h>
//#include <mach/mt_spi.h>
//#include <mach/eint.h>
//#include <cust_eint.h>
#include <asm/uaccess.h>
#include <linux/ktime.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/of_irq.h>
#include <linux/completion.h>
#include <mach/mt_clkmgr.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/gpio.h>
#include <asm/io.h>

#include <linux/platform_data/spi-mt65xx.h>
#include <linux/spi/spi.h>
#include <linux/regulator/consumer.h>
#include <mt-plat/mt_boot_common.h>
#include <linux/platform_device.h>

#include <linux/notifier.h> //add for fb
#include <linux/fb.h>       //add for fb
#include "gf-common.h"
#include "gf-milanf.h"
#include "gf-regs.h"

#if CONFIG_HAS_EARLYSUSPEND
static int suspend_flag = 0;
#endif
/* The main reason to have this class is to make mdev/udev create the
 * /dev/spidevB.C character device nodes exposing our userspace API.
 * It also simplifies memory management.
 */
static struct class *gf_spi_class;
/*************************data stream***********************
 *	FRAME NO  | RING_H  | RING_L  |  DATA STREAM | CHECKSUM |
 *     1B      |   1B    |  1B     |    2048B     |  2B      |
 ************************************************************/
static unsigned bufsiz = 14260;
static unsigned char g_frame_buf[14260]={0};
static DECLARE_BITMAP(minors, N_SPI_MINORS);
static LIST_HEAD(device_list);
static DEFINE_MUTEX(device_list_lock);
struct gf_dev_t gf;
static unsigned char g_sendorID[16] = {0};
static unsigned short g_fdt_delta = 0;
static unsigned short g_tcode_img = 0;
static int g_irq_enabled = 1;

//static int g_irq_enable = 1;
module_param(bufsiz, uint, S_IRUGO);
MODULE_PARM_DESC(bufsiz, "data bytes in biggest supported SPI message");
int gf_write_configs(struct gf_dev_t *gf_dev,struct gf_configs config[],int len);
unsigned int fingerprint_irq = 0;
//static struct pinctrl *pinctrl1;
//static struct pinctrl_state *pins_default;
//static struct pinctrl_state *eint_as_int, /* *eint_pulldown, *eint_pulldisable, */ *rst_output0, *rst_output1;
static struct of_device_id fingerprint_of_match[] = {
	{ .compatible = "mediatek,mt6797-finger", },
	{},
};
MODULE_DEVICE_TABLE(of,fingerprint_of_match);

int goodix_chip_detected = 0;

#if 0 // defined(CONFIG_OF)
struct platform_device *fingerprint_device = NULL;
#endif

struct gf_key_map key_map[] =
{   
      //{  "POWER",  KEY_POWER,     68 },	/*KEY*/
      //{  "HOME" ,   KEY_HOME,   	68 },	/*KEY*/
      //{  "MENU" ,   KEY_MENU,   	68 },	/*KEY*/
      {  "BACK" ,   KEY_BACK,   	68 },	/*KEY*/
      {  "UP"   ,      KEY_UP,     	103 },	/*NAV_LEFT*/
      {  "DOWN" ,  KEY_DOWN,   	108 },	/*NAV_RIGHT*/
      {  "LEFT" ,     KEY_LEFT,   	105 },	/*NAV_UP*/
      {  "RIGHT",   KEY_RIGHT,  	106 },	/*NAV_DOWN*/
      //{  "CLICK",  KEY_F19,    		68 },/*NAV_CLICK*/
};

/************* Configs Definition **********/
struct gf_configs normal_cfg[]=        
{
	{GF_FDT_AREA_NUM,0x0000},
	{GF_MODE_CTRL0,0x4010},/*enable osr and wake up timer*/
	{GF_MODE_CTRL2,0x0080},
	{GF_PIXEL_CTRL3,0x2808},
	{GF_PIXEL_CTRL4,0x0148},
	{GF_CMP_NUM_CTRL,0x0000},
	{GF_FDT_DELTA,0x0C7F},
	{GF_FDT_THR0,0x817F},
	{GF_FDT_THR1,0x8C7F},
	{GF_FDT_THR2,0x977F},
	{GF_FDT_THR3,0xB07F},
	{GF_FDT_THR4,0x867F},
	{GF_FDT_THR5,0x8C7F},
	{GF_FDT_THR6,0xA07F},
	{GF_FDT_THR7,0xB37F},
	{GF_FDT_THR8,0x847F},
	{GF_FDT_THR9,0x887F},
	{GF_FDT_THR10,0xA07F},
	{GF_FDT_THR11,0xB87F},
	{GF_ENCRYPT_EN,0x0000}, /*default do not encrypt*/
};

struct gf_configs fdt_down_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0482}, /*enable fdt INT*/
	{GF_MODE_CTRL0,0x4010},
	{GF_MODE_CTRL1,0x2001}, 
	{GF_MODE_CTRL2,0x0014}, 
	{GF_PIXEL_CTRL6,0x0100},
	{GF_FDT_DELTA,0x157F},
	{GF_FDT_AREA_NUM,0x0007},
	{GF_FDT,0x0401},        /*fdt enabel, detect down*/

};

struct gf_configs nav_fdt_down_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0482}, /*enable fdt INT*/
	{GF_MODE_CTRL0,0x4010},
	{GF_MODE_CTRL1,0x2001}, 
	{GF_MODE_CTRL2,0x0014}, 
	{GF_PIXEL_CTRL6,0x0100},
	{GF_FDT_DELTA,0x157F},
	{GF_FDT_AREA_NUM,0x0000},
	{GF_FDT,0x0001},        /*fdt enabel, detect down*/
};

struct gf_configs fdt_up_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0482}, /*enable fdt INT*/
	{GF_MODE_CTRL0,0x4010},
	{GF_MODE_CTRL1,0x2001}, 
	{GF_MODE_CTRL2,0x0014}, 
	{GF_PIXEL_CTRL6,0x0100}, 
	{GF_FDT_DELTA,0x137F},
	{GF_FDT,0x0003},        /*fdt enabel, detect up*/
};

struct gf_configs ff_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0402}, /*enable fdt INT*/
	{GF_MODE_CTRL0,0x4010},	
	{GF_MODE_CTRL1,0x2001}, 
	{GF_MODE_CTRL2,0x0032}, 
	{GF_PIXEL_CTRL6,0x0100}, 
	{GF_FDT_DELTA,0x157F},
	{GF_FDT,0x0001},        /*fdt enabel, detect down*/	
};

struct gf_configs nav_cfg[]=
{
	{GF_IRQ_CTRL0, 0x0408}, /*enable data_int INT*/
	{GF_MODE_CTRL1,0x0810}, /*manual mode*/
	{GF_PIXEL_CTRL6,0x0100},
	{GF_FDT,0x0001},        /*fdt enabel, detect down*/
};

struct gf_configs nav_img_cfg[]=
{
	{GF_PIXEL_CTRL6,0x0080},       /*set rate 128*/
	{GF_IRQ_CTRL0,0x0408},         /*enable data_int INT*/
	{GF_PIXEL_CTRL1,0x0008},       /*set one_frame mode*/
	{GF_PIXEL_CTRL0,0x0501},	
};

struct gf_configs img_cfg[]=
{
	{GF_PIXEL_CTRL6,0x0100},       
	{GF_IRQ_CTRL0,0x0408},         /*enable data_int INT*/
	{GF_PIXEL_CTRL1,0x0008},       /*set one_frame mode*/
	{GF_PIXEL_CTRL0,0x0501},	
};

struct gf_configs nav_img_manual_cfg[]=
{
	{GF_PIXEL_CTRL6,0x0080},
	{GF_IRQ_CTRL0,0x0408},
	{GF_PIXEL_CTRL1,0x0004}, /*Manual Mode Enable*/
	{GF_PIXEL_CTRL0,0x0501},
	{GF_PIXEL_CTRL2,0x0100}, /*Shadow Mode Select*/ 
};

static void gf_enable_irq(struct gf_dev_t* gf_dev)
{
	if(g_irq_enabled) {
		pr_info("%s gx368 irq has been enabled.\n",__func__);
	} else {
		pr_info("%s called.\n",__func__);
		enable_irq(fingerprint_irq);
		g_irq_enabled = 1;
	}
}

static void gf_disable_irq(struct gf_dev_t* gf_dev)
{
	if(g_irq_enabled) {
		g_irq_enabled = 0;
		pr_info("%s called.\n",__func__);
		disable_irq(fingerprint_irq);
	} else {
		pr_info("%sgx368 irq has been disabled.\n",__func__);
	}
}

#if 1 //add for parse otp 
void readOTPByte(struct gf_dev_t* gf_dev,unsigned short usBank,
					unsigned short usAddr,unsigned char* pValue)
{
    unsigned short data = 0;
    gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0020);
    gf_spi_write_word(gf_dev,GF_OTP_ADDR2,usBank);
    gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0021);
    gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0020);

    gf_spi_write_word(gf_dev,GF_OTP_ADDR2,((usAddr<<8) & (0xFFFF)));
    gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0022);
    gf_spi_write_word(gf_dev,GF_OTP_ADDR1,0x0020);
    gf_spi_read_word(gf_dev,GF_OTP_ADDR3,&data);

    *pValue = data & 0xFF;
}

void gf_parse_otp(struct gf_dev_t* gf_dev,unsigned char* pSensorID)
{
    unsigned short i = 0;
    unsigned char ucOTP[32] = {0};
    unsigned char Tcode,Diff;
    unsigned short Tcode_use,Diff_use,Diff_256;
	
    for(i=0;i<32;i++)
    {
        readOTPByte(gf_dev,(i>>2),(i & 0x03),ucOTP+i);
		printk("gx368 OTP_BYTE[%d] = 0x%x \n",i,ucOTP[i]);
    }
			
        memcpy(pSensorID,ucOTP,16);
	Tcode = (ucOTP[22] & 0xF0) >> 4;
	Diff = ucOTP[22] & 0x0F;
	printk("gx368 Tcode = 0x%x \n",Tcode);
	printk("gx368 Diff = 0x%x \n",Diff);
	if((Tcode != 0) && (Diff != 0) && ((ucOTP[22] & ucOTP[23]) == 0)) 
	{
		Tcode_use = (Tcode + 1)*16;
		Diff_use = (Diff + 2)*100;

		Diff_256 = (Diff_use * 256) / Tcode_use;
		pr_info("%s Tcode:0x%x(%d),Diff:0x%x(%d)\n",__func__,Tcode,Tcode,Diff,Diff);
		pr_info("%s Tcode_use:0x%x(%d),Diff_use:0x%x(%d),Diff_256:0x%x(%d)\n",
			        __func__,Tcode_use,Tcode_use,Diff_use,Diff_use,Diff_256,Diff_256);
		g_fdt_delta = (((Diff_256/3)>>4)<<8)|0x7F;
		g_tcode_img = Tcode_use;		
	}
	pr_info("%s g_tcode_img:0x%x,g_fdt_delta:0x%x\n",__func__,g_tcode_img,g_fdt_delta);
} 

#endif //add for parse otp

static int gf_reg_key_kernel(struct gf_dev_t *gf_dev)
{
    int i;
    set_bit(EV_KEY, gf_dev->input->evbit); //tell the kernel is key event
    for(i = 0; i< ARRAY_SIZE(key_map); i++) {
        set_bit(key_map[i].val, gf_dev->input->keybit);
    }

    gf_dev->input->name = GF_INPUT_NAME;
    if (input_register_device(gf_dev->input)){
        pr_warn("Failed to register GF as input device.\n");
		return -1;
    }
	return 0;
}

inline static void gf_irq_cfg(struct gf_dev_t *gf_dev)
{
	/*Config IRQ pin, referring to platform.*/
	spi_dts_gpio_select_state(DTS_GPIO_STATE_AS_EINT); 
}

/********************************************************************
*CPU output low level in RST pin to reset GF. This is the MUST action for GF.
*Take care of this function. IO Pin driver strength / glitch and so on.
********************************************************************/
inline static void gf_hw_reset(struct gf_dev_t *gf_dev, int ms)
{	
	gf_debug(DEFAULT_DEBUG, "gf_hw_reset\n");

	spi_dts_gpio_select_state(DTS_GPIO_STATE_FINGER_RST0); 
	msleep(3);
	spi_dts_gpio_select_state(DTS_GPIO_STATE_FINGER_RST1);  

	if (ms)
	    msleep(ms);
	//gf_write_configs(gf_dev, normal_cfg, sizeof(normal_cfg)/sizeof(struct gf_configs));
}
/*********************************************************
**Power control
**function: hwPowerOn   hwPowerDown
*********************************************************/
int gf_power_on(struct gf_dev_t *gf_dev, bool onoff)
{
//	struct regulator *reg = NULL;
//	int ret = 0;
	gf_debug(DEFAULT_DEBUG,"%s onoff = %d", __func__, onoff);
	if(onoff) {
		if (0 == gf_dev->poweron){
			/*INT GPIO Pull-down before power on*/

	               spi_dts_gpio_select_state(DTS_GPIO_STATE_EINT_IN_LOW); 


			/*Reset GPIO Output-low before poweron*/

			spi_dts_gpio_select_state(DTS_GPIO_STATE_FINGER_RST0); 

			msleep(5);


			gf_dev->poweron = 1;
			
			msleep(20);

			/*INT GPIO set floating after poweron and controlled by GF*/

	                spi_dts_gpio_select_state(DTS_GPIO_STATE_AS_EINT); 

			msleep(5);
			
			/*Reset GPIO Output-high, GF works*/

			spi_dts_gpio_select_state(DTS_GPIO_STATE_FINGER_RST1);  

			msleep(60);
		}
	} else {
		if (1 == gf_dev->poweron){

	                spi_dts_gpio_select_state(DTS_GPIO_STATE_AS_EINT); 

			spi_dts_gpio_select_state(DTS_GPIO_STATE_FINGER_RST0);


			msleep(10);

			gf_dev->poweron = 0;
			msleep(50);
		}
	}
	return 0;
}

/* -------------------------------------------------------------------- */
/* devfs                                                                */
/* -------------------------------------------------------------------- */
static ssize_t gf_debug_show(struct device *dev, 
	struct device_attribute *attr, char *buf)
{
	return (sprintf(buf, "%d\n", g_debug_level));
}
static ssize_t gf_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int g_debug = 0;
	sscanf(buf, "%d", &g_debug);
	//gf_debug_level(g_debug);
	//return strnlen(buf, count); 
	return count;
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, gf_debug_show, gf_debug_store);

static struct attribute *gf_debug_attrs[] = {
    &dev_attr_debug.attr,
    NULL
};

static const struct attribute_group gf_debug_attr_group = {
    .attrs = gf_debug_attrs,
    .name = "gf_debug"
};

int gf_write_configs(struct gf_dev_t *gf_dev,struct gf_configs config[],int len)
{
	int cnt;
	int length = len;
	int ret = 0;
	for(cnt=0;cnt<length;cnt++)
	{
		gf_debug(DEFAULT_DEBUG,"addr = 0x%x, value = 0x%x\n",config[cnt].addr,config[cnt].value);
		ret = gf_spi_write_word(gf_dev,config[cnt].addr,config[cnt].value);
		if(ret < 0){
			gf_error("%s failed. ",__func__);
			return ret;
		}
	}
	return 0;
}

/*-------------------------------------------------------------------------*/
/* Read-only message with current device setup */
static ssize_t gf_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
 	struct gf_dev_t *gf_dev = filp->private_data;
	int status = 0;
	int len = 0;

	FUNC_ENTRY();
	if(buf == NULL || count > bufsiz)
	{
		printk("gx368 input parameters invalid.\n");
		return -EMSGSIZE;
	}
       gf_spi_set_mode(gf_dev->spi, SPEED_6MHZ, 1);//wenhongjun add 20150519
	len = gf_spi_read_data(gf_dev,0xAAAA,count,g_frame_buf);
 	gf_spi_set_mode(gf_dev->spi, SPEED_1MHZ, 0);//wenhongjun add 20150519	
	//pr_info("%s read length = %d \n",__func__,len);
	status = copy_to_user(buf, g_frame_buf, count);
	if(status != 0) {
		gf_error("%s copy_to_user failed. status = %d ",__func__,status);
		return -EFAULT;
	}
	FUNC_EXIT();
       return 0;
}

/* Write-only message with current device setup */
static ssize_t gf_write(struct file *filp, const char __user *buf,
	size_t count, loff_t *f_pos)
{
	return 0;
}

static long gf_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct gf_dev_t *gf_dev = (struct gf_dev_t *)filp->private_data;
	struct gf_ioc_transfer *ioc = NULL;
	struct gf_key gf_key={0};
	u8* tmpbuf = NULL; 
	int ret = 0;
	int retval = 0;
	int err = 0;
	unsigned char command = 0;
	unsigned char config_type = 0;
	int i;
	#if 0
	struct gf_configs* p_cfg = NULL;
	unsigned char cfg_len = 0;
	struct gf_mode_config* p_mode_cfg = NULL;
	#endif
   
	printk("[-----gx368/gf3208 gf_ioctl------]cmd = %d\n",cmd);
	if (_IOC_TYPE(cmd) != GF_IOC_MAGIC)
	        return -ENOTTY;

    /* Check access direction once here; don't repeat below.
     * IOC_DIR is from the user perspective, while access_ok is
     * from the kernel perspective; so they look reversed.
     */
    if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE,
			(void __user *)arg, _IOC_SIZE(cmd));
    if (err == 0 && _IOC_DIR(cmd) & _IOC_WRITE)
		err = !access_ok(VERIFY_READ,
			(void __user *)arg, _IOC_SIZE(cmd));
    if (err)
		return -EFAULT;
	
    FUNC_ENTRY();
    switch(cmd) {
	case GF_IOC_RW:
	    ioc = kzalloc(sizeof(*ioc), GFP_KERNEL);
		if (ioc == NULL) {
			gf_error("kzalloc ioc failed.");
			retval = -ENOMEM;
			break;
		}
	    /*copy command data from user to kernel.*/
	    if(copy_from_user(ioc, (struct gf_ioc_transfer*)arg, sizeof(*ioc))){
			gf_error("Failed to copy command from user to kernel.");
			retval = -EFAULT;
			break;
	    }
		tmpbuf = kzalloc(ioc->len, GFP_KERNEL);
		if (tmpbuf == NULL) {
			gf_error("kzalloc tmpbuf failed.");
			retval = -ENOMEM;
			break;			
		}
	    if((ioc->len > bufsiz)||(ioc->len == 0)) {
			gf_error("The request length[%d] is longer than supported maximum buffer length[%d].", 
				ioc->len, bufsiz);
			retval = -EMSGSIZE;
			break;
	    }

	    if(ioc->cmd == GF_R) {
			/*if want to read data from hardware.*/
			//gf_debug(DEFAULT_DEBUG,"gf_ioctl Read data from 0x%x, len = 0x%x buf = 0x%p\n", (int)ioc->addr, (int)ioc->len, (void __user*)((unsigned long)ioc->buf));
			mutex_lock(&gf_dev->frame_lock);
			gf_spi_read_data(gf_dev, ioc->addr, ioc->len, tmpbuf);
			mutex_unlock(&gf_dev->frame_lock);

			mutex_lock(&gf_dev->buf_lock);
		#if PROCESSOR_64_BIT
			ret = copy_to_user((void __user*)((unsigned long)ioc->buf), tmpbuf, ioc->len);
		#else	
			ret = copy_to_user(ioc->buf, gf_dev->buffer + GF_RDATA_OFFSET, ioc->len);
		#endif
			mutex_unlock(&gf_dev->buf_lock);

			if(ret) {
			    gf_error("Failed to copy data from kernel to user.");
			    retval = -EFAULT;
			    break;
			}
	    } else if (ioc->cmd == GF_W) {
			/*if want to read data from hardware.*/
			gf_debug(DEFAULT_DEBUG,"gf_ioctl Write data to 0x%x, len = 0x%x", ioc->addr, ioc->len);
		#if PROCESSOR_64_BIT			
			ret = copy_from_user(tmpbuf, (void __user*)((unsigned long) ioc->buf), ioc->len);
		#else			
			ret = copy_from_user(gf_dev->buffer + GF_WDATA_OFFSET, ioc->buf, ioc->len);
		#endif
			if(ret){
			    gf_error("Failed to copy data from user to kernel.");
			    retval = -EFAULT;
			    break;
			}
			
			mutex_lock(&gf_dev->frame_lock);
			gf_spi_write_data(gf_dev, ioc->addr, ioc->len, tmpbuf);
			mutex_unlock(&gf_dev->frame_lock);
	    }
		else {
			gf_error("Error command for gf_ioctl.");
			retval = -EFAULT;
	    }
	    break;
	case GF_IOC_CMD:
		retval = __get_user(command ,(u32 __user*)arg);
		//pr_info("%s GF_IOC_CMD command is %x \n",__func__,command);
		mutex_lock(&gf_dev->buf_lock);
		gf_spi_send_cmd(gf_dev,&command,1);
		mdelay(2);
		mutex_unlock(&gf_dev->buf_lock);
	    break;
    #if 1
	case GF_IOC_CONFIG:
		retval = __get_user(config_type, (u32 __user*)arg);
		if(config_type == CONFIG_FDT_DOWN){
			gf_write_configs(gf_dev,fdt_down_cfg,sizeof(fdt_down_cfg)/sizeof(struct gf_configs));
			break;
		}
		else if(config_type == CONFIG_FDT_UP){
			gf_write_configs(gf_dev,fdt_up_cfg,sizeof(fdt_up_cfg)/sizeof(struct gf_configs));
			break;
		}
		else if(config_type == CONFIG_FF){
			gf_write_configs(gf_dev,ff_cfg,sizeof(ff_cfg)/sizeof(struct gf_configs));
			break;
		}
		else if(config_type == CONFIG_NAV){
			gf_write_configs(gf_dev,nav_cfg,sizeof(nav_cfg)/sizeof(struct gf_configs));
			break;
		}
		else if(config_type == CONFIG_IMG){
			gf_write_configs(gf_dev,img_cfg,sizeof(img_cfg)/sizeof(struct gf_configs));
			break;
		}else if(config_type == CONFIG_NAV_IMG){
			gf_debug(DEFAULT_DEBUG, "%s CONFIG_NAV_IMG.",__func__);
			gf_write_configs(gf_dev,nav_img_cfg,sizeof(nav_img_cfg)/sizeof(struct gf_configs));
			break;
		}else if(config_type == CONFIG_NAV_IMG_MAN) {
			gf_write_configs(gf_dev,nav_img_manual_cfg,sizeof(nav_img_manual_cfg)/sizeof(struct gf_configs));
			break;
		}else if(config_type == CONFIG_NAV_FDT_DOWN) {
			gf_write_configs(gf_dev,nav_fdt_down_cfg,sizeof(nav_fdt_down_cfg)/sizeof(struct gf_configs));			
			break;
		}
		else{
			gf_debug(DEFAULT_DEBUG, "%s unknow config_type is %d ",__func__,config_type);
			break;
		}
    #else
	case GF_IOC_CONFIG:
        p_mode_cfg = kzalloc(sizeof(*p_mode_cfg), GFP_KERNEL);
        if(p_mode_cfg == NULL) {
            gf_error("kzalloc p_mode_cfg failed. \n");
            retval = -ENOMEM;
            break;
        }

        if(copy_from_user(p_mode_cfg, (struct gf_mode_config*)arg, sizeof(*p_mode_cfg))){
            gf_error("Failed to copy command from user to kernel.\n");
            retval = -EFAULT;
            break;
        }
        
        cfg_len = p_mode_cfg->cfg_len*sizeof(struct gf_configs);
        if (cfg_len > 0) {
            p_cfg = (struct gf_mode_config*)kzalloc(cfg_len, GFP_KERNEL);
            if(p_mode_cfg == NULL) {
                gf_error("kzalloc p_cfg failed. \n");
                retval = -ENOMEM;
                break;
            }
        } else {
            gf_error("err cfg_len = %d\n", cfg_len);
            retval = -EFAULT;
            break;
        }

        if(copy_from_user(p_cfg, p_mode_cfg->p_cfg, cfg_len)){
            gf_error("Failed to copy command from user to kernel.\n");
            retval = -EFAULT;
            break;
        }
        gf_write_configs(gf_dev,p_cfg,p_mode_cfg->cfg_len);
        break;
   #endif
   
	case GF_IOC_RESET:
	    gf_hw_reset(gf_dev,10);
	    gf_debug(DEFAULT_DEBUG, "%s GF_IOC_RESET ",__func__);
	    break;
	case GF_IOC_ENABLE_IRQ:
		gf_debug(DEFAULT_DEBUG, "%s ++++++++++++ GF_IOC_ENABLE_IRQ ",__func__);
		gf_enable_irq(gf_dev);
		break;
	case GF_IOC_DISABLE_IRQ:
		gf_debug(DEFAULT_DEBUG, "%s ------------ GF_IOC_DISABLE_IRQ ",__func__);
		gf_disable_irq(gf_dev);
		break;
	case GF_IOC_SENDKEY:
		gf_debug(DEFAULT_DEBUG, "%s GF_IOC_SENDKEY.",__func__);
		if(copy_from_user(&gf_key,(struct gf_key*)arg, sizeof(struct gf_key))) {
			gf_debug(DEFAULT_DEBUG, "%s GF_IOC_SENDKEY failed to copy data from user.",__func__);
			retval = -EFAULT;
			break;
		}
		gf_debug(DEFAULT_DEBUG, "gf_key.key = %d\n",gf_key.key);
		for(i = 0; i< ARRAY_SIZE(key_map); i++) {
            if(key_map[i].dev_value == gf_key.key){
                input_report_key(gf_dev->input, gf_key.key, gf_key.value);
                input_sync(gf_dev->input);
                break;
            }
        }

        if(i == ARRAY_SIZE(key_map)) {
            gf_error("key %d not support yet ", gf_key.key);
            retval = -EFAULT;
        }
		break;
	default:
	    gf_error("%s gf doesn't support this command.",__func__);
		gf_error("%s CMD = 0x%x,_IOC_DIR:0x%x,_IOC_TYPE:0x%x,IOC_NR:0x%x,IOC_SIZE:0x%x",
						__func__,cmd,_IOC_DIR(cmd),_IOC_TYPE(cmd),_IOC_NR(cmd),_IOC_SIZE(cmd));
	    retval = -EFAULT;
	    break;
    }
    FUNC_EXIT();
	if(tmpbuf != NULL){
		kfree(tmpbuf);
		tmpbuf = NULL;
	}
	if(ioc != NULL) {
		kfree(ioc);
		ioc = NULL;
	}
    return retval;
}

static unsigned int gf_poll(struct file *filp, struct poll_table_struct *wait)
{
    return 0;
}

#if CONFIG_HAS_EARLYSUSPEND
static void gf_early_suspend(struct early_suspend *h)
{    
	struct gf_dev_t *gf_dev = container_of(h, struct gf_dev_t, early_fp);
	gf_debug(DEFAULT_DEBUG,"gf  suspend.");
	suspend_flag = 1;
}


static void gf_late_resume(struct early_suspend *h)
{
	struct gf_dev_t *gf_dev = container_of(h, struct gf_dev_t, early_fp);	
	gf_debug(DEFAULT_DEBUG,"gf  resume");

	suspend_flag = 0;
}
#endif

/*******************************************
**Interrupter
**
*******************************************/
static irqreturn_t fingerprint_eint_interrupt_handler(unsigned irq, struct irq_desc *desc)
{
	struct gf_dev_t *gf_dev = &gf;
	
#if GF_FASYNC
	if(gf_dev->async) {
		gf_debug(DEFAULT_DEBUG,"async ");
		kill_fasync(&gf_dev->async, SIGIO, POLL_IN);
	}
#endif
	return IRQ_HANDLED;

}

#if GF_FASYNC
static int gf_fasync(int fd, struct file *filp, int mode)
{
    struct gf_dev_t *gf_dev = filp->private_data;
    int ret;

    FUNC_ENTRY();
    ret = fasync_helper(fd, filp, mode, &gf_dev->async);
    FUNC_EXIT();
    return ret;
}
#endif

static int gf_open(struct inode *inode, struct file *filp)
{
    struct gf_dev_t *gf_dev;
    int status = -ENXIO;
	int cnt=0;
	unsigned short reg;
     
    FUNC_ENTRY();
    //mt_spi_set_mode();
    printk("gx368_open\n");
    mutex_lock(&device_list_lock);

    list_for_each_entry(gf_dev, &device_list, device_entry) {
		if(gf_dev->devt == inode->i_rdev) {
		    gf_debug(DEFAULT_DEBUG, "Found");
		    status = 0;
		    break;
		}
    }

    if(status == 0){
		mutex_lock(&gf_dev->buf_lock);
		if( gf_dev->buffer == NULL) {
		    gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
		    if(gf_dev->buffer == NULL) {
				gf_error("open/ENOMEM");
				status = -ENOMEM;
		    }
		}
		mutex_unlock(&gf_dev->buf_lock);

		if(status == 0) {
		    gf_dev->users++;
		    filp->private_data = gf_dev;
			#if 1
			/* 1. disable irq */
			gf_disable_irq(gf_dev);

			/* 2. set chip to init state */
			gf_hw_reset(gf_dev,0); 
			while(cnt < 10){
                gf_spi_read_word(gf_dev,GF_IRQ_CTRL3,&reg);
                pr_info("%s IRQ status : 0x%x ,cnt = %d",__func__,reg,cnt);
                if(reg == 0x0100) {
                    gf_spi_write_word(gf_dev,GF_IRQ_CTRL2,0x0100);
                    break;
                }
                cnt++;
            }
			
		/* 3. parse otp */
		gf_parse_otp(gf_dev,g_sendorID);

        	if(g_tcode_img!=0 && g_fdt_delta!=0)
        	{
        		fdt_down_cfg[5].value = g_fdt_delta;
        		ff_cfg[5].value = g_fdt_delta;
        		nav_fdt_down_cfg[5].value = g_fdt_delta;
        		fdt_up_cfg[5].value = (((g_fdt_delta>>8)-2)<<8)|0x7F;
        		img_cfg[0].value = g_tcode_img;
        		pr_info("%s use img_tcode:0x%x fdt_delta:0x%x fdt_up_delta:0x%x from OTP\n",
        			      __func__,g_tcode_img,g_fdt_delta,((((g_fdt_delta>>8)-2)<<8)|0x7F));
        	}
		#endif

			/* 4. open device node and open irq*/
		    nonseekable_open(inode, filp);
		    printk("gx368 Succeed to open device. irq = %d\n", gf_dev->spi->irq);
		    //mt_eint_unmask(gf_dev->spi->irq);
		    gf_enable_irq(gf_dev);		
		}
	} else {
		gf_error("No device for minor %d", iminor(inode));
	}
	mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}

static int gf_release(struct inode *inode, struct file *filp)
{
    struct gf_dev_t *gf_dev = filp->private_data;
    int    status = 0;
    FUNC_ENTRY();
    mutex_lock(&device_list_lock);

    filp->private_data = NULL;

    /*last close??*/
    gf_dev->users --;
    if(!gf_dev->users) {
		gf_debug(DEFAULT_DEBUG, "disable_irq. irq = %d", gf_dev->spi->irq);
		//mt_eint_mask(gf_dev->spi->irq);
		gf_disable_irq(gf_dev);
    }
    mutex_unlock(&device_list_lock);
    FUNC_EXIT();
    return status;
}
static long gf_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    return gf_ioctl(filp, cmd, (unsigned long)(arg));
}

static const struct file_operations gf_fops = {
    .owner =	THIS_MODULE,
    /* REVISIT switch to aio primitives, so that userspace
     * gets more complete API coverage.  It'll simplify things
     * too, except for the locking.
     */
    .write =	gf_write,
    .read =		gf_read,
    .unlocked_ioctl = gf_ioctl,
	.compat_ioctl	= gf_compat_ioctl,
    .open =		gf_open,
    .release =	gf_release,
    .poll   = gf_poll,
#if GF_FASYNC
    .fasync = gf_fasync,
#endif
};

/*-------------------------------------------------------------------------*/



static int fingerprint_get_gpio_info(struct gf_dev_t *pdev)
{
	long ret = 0;
	//struct pinctrl *pctrl;
	struct pinctrl_state *pState = 0;
	int i;
	struct device_node *node;
	/* retrieve */
	printk("gx368 spi_dts_gpio_init\n");
	
	//pctrl = devm_pinctrl_get(&pdev->dev);
	node = of_find_compatible_node(NULL, NULL,"mediatek,mt6797-finger");
	#if 1
	if(node){
		printk("gx368 spi_dts_gpio_init get node success\n");
		#if 1
		for(i=0;i<DTS_GPIO_STATE_MAX;i++){
			pState = pinctrl_lookup_state(pdev->pctrl, this_state_name[i]);
			if (IS_ERR(pState)) {
				printk("gx368 spi_dts_gpio_init gpio_name=%s failed\n",  this_state_name[i]);
				ret = PTR_ERR(pState);
				return ret;
			}
		}
		#endif
		printk("gx368 spi_dts_gpio_init success xielei\n");
		this_pctrl = pdev->pctrl;
	}else{
		printk("gx368 spi_dts_gpio_init get node failed\n");
       }
	#endif

	return ret;
}


#if 0 //def CONFIG_OF
static int fingerprint_dts_probe(struct platform_device *dev)
{
	fingerprint_device = dev;
	printk("gx368/gf3208 nasri...%s %d\n",__func__,__LINE__);

	//fingerprint_get_gpio_info(dev);
	return 0;
}
static const struct dev_pm_ops fingerprint_pm_ops = {
	.suspend = NULL,
	.resume = NULL,
};
static struct platform_driver fingerprint_dts_driver = {
	.probe = fingerprint_dts_probe,
	.remove = NULL,
	.shutdown = NULL,
	.driver = {
		   .name = "fingerprint_gf3208",
			.pm = &fingerprint_pm_ops,
		    .owner = THIS_MODULE,
		    //.bus	= &spi_bus_type,
		   .of_match_table = fingerprint_of_match,
		   },
};
#endif


/*-------------------------------------------------------------------------*/
static int  gf_probe(struct spi_device *spi)
{
	struct gf_dev_t *gf_dev = &gf;
	int status;
	int ret;
	unsigned long minor;
	int tt = 0;
	struct device_node *node = NULL;
	u32 ints[2] = {0};//xielei@20151219
	FUNC_ENTRY();
	printk("gx368/gf3208 driver version is %s.\n",GF_DRIVER_VERSION);
	//fingerprint_get_gpio_info(&spi->dev);

	/* Initialize the driver data */
	gf_dev->spi = spi;
	spi_set_drvdata(spi, gf_dev);
		
	/* Allocate driver data */
	gf_dev->buffer = kzalloc(bufsiz + GF_RDATA_OFFSET, GFP_KERNEL);
	if(gf_dev->buffer == NULL) {
		status = -ENOMEM;
		goto err_alloc_buffer;
	}

	spin_lock_init(&gf_dev->spi_lock);
	mutex_init(&gf_dev->buf_lock);
	mutex_init(&gf_dev->frame_lock);
	INIT_LIST_HEAD(&gf_dev->device_entry);

	#if 1
	spi->dev.of_node=of_find_compatible_node(NULL, NULL, "mediatek,mt6797-finger");
	gf_dev->pctrl = devm_pinctrl_get(&spi->dev);
    	if (IS_ERR(gf_dev->pctrl)) {
    		ret = PTR_ERR(gf_dev->pctrl);
    		printk("gx368 Cannot find fp pctrl!\n");
    		return ret;
    	}
	#endif
	fingerprint_get_gpio_info(gf_dev);
	
	/*power on*/
	gf_power_on(gf_dev, 1);

	/*setup gf configurations.*/
	gf_debug(DEFAULT_DEBUG, "Setting gf device configuration.");
	
	/*SPI parameters.*/	
	gf_spi_setup(gf_dev, 8*1000*1000);
	gf_spi_set_mode(gf_dev->spi, SPEED_1MHZ, 0);

	//nasri
	//gf_spi_pins_config();

	gf_spi_write_word(gf_dev,0x0124,0x0100); 
	gf_spi_write_word(gf_dev,0x0204,0x0000);

#if 1
	/*SPI test to read chip ID 002202A0*/
	//cut times
	//while(1){
		for(tt=0;tt<3;tt++) 
                {
			unsigned short chip_id_1 = 0;
			unsigned short chip_id_2 = 0;

			gf_spi_read_word(gf_dev,0x0000,&chip_id_1);
			gf_spi_read_word(gf_dev,0x0002,&chip_id_2);

			printk("gx368/gf3208 %s chip id is 0x%04x 0x%04x \n",__func__,chip_id_2,chip_id_1);
			if(chip_id_2 == 0x0022 && chip_id_1 == 0x02A0)
                        {
				goodix_chip_detected = 1;
			}
			else 
                        {
                        	spi_set_drvdata(spi, NULL);
                        	status = -ENOMEM;
				return status;
			}
		}
//	}
#endif
	
	/* If we can allocate a minor number, hook up this device.
	 * Reusing minors is fine so long as udev or mdev is working.
	 */

    /* Claim our 256 reserved device numbers.  Then register a class
     * that will key udev/mdev to add/remove /dev nodes.  Last, register
     * the driver which manages those device numbers.
     */
    BUILD_BUG_ON(N_SPI_MINORS > 256);
    status = register_chrdev(SPIDEV_MAJOR, CHRD_DRIVER_NAME, &gf_fops);
    if (status < 0){
		gf_error("Failed to register char device!");
		goto err_register_char;
    }
    gf_spi_class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(gf_spi_class)) {
		gf_error("Failed to create class.");
		FUNC_EXIT();
		status = PTR_ERR(gf_spi_class);
		goto err_creat_class;
    }

	mutex_lock(&device_list_lock);
	minor = find_first_zero_bit(minors, N_SPI_MINORS);
	if (minor < N_SPI_MINORS) {
		struct device *dev;
		status = sysfs_create_group(&spi->dev.kobj,&gf_debug_attr_group);
		if(status){
		    gf_error("Failed to create sysfs file.");
		    goto err_creat_group;
		}
		gf_dev->devt = MKDEV(SPIDEV_MAJOR, minor);
		dev = device_create(gf_spi_class, &spi->dev, gf_dev->devt,
			  gf_dev, DEV_NAME);
		status = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	} else {
		gf_error( "no minor number available!");
		status = -ENODEV;
	}
	if (status == 0) {
		set_bit(minor, minors);
		list_add(&gf_dev->device_entry, &device_list);
	}
	mutex_unlock(&device_list_lock);
	
	/*register device within input system.*/
	gf_dev->input = input_allocate_device();
	if(gf_dev->input == NULL) {
		gf_error("Failed to allocate input device.");
		status = -ENOMEM;
		goto err_alloc_input;
	}
	
	status = gf_reg_key_kernel(gf_dev);
	if (status){
		gf_error("Failed to register input device.");
		goto err_free_input;
	}

	/*irq config and interrupt init*/
	gf_irq_cfg(gf_dev);
	//gf_dev->spi->irq = GF_IRQ_NUM;

	node = of_find_matching_node(node, fingerprint_of_match);

	if (node) {
		//of_property_read_u32_array(node, "debounce", ints, ARRAY_SIZE(ints));
		//gpio_set_debounce(ints[0], ints[1]);

		of_property_read_u32_array(node,"debounce",ints,ARRAY_SIZE(ints));
		printk("gx368 ints[0]=%d,ints[1]=%d\n",ints[0],ints[1]);
		printk("gx368 ints[0]=%d,ints[1]=%d\n",ints[0],ints[1]);
		gpio_set_debounce(ints[0], ints[1]);

		fingerprint_irq = irq_of_parse_and_map(node, 0);
		printk("gx368_gf3208 fingerprint_irq=%d\n   ",fingerprint_irq);
		gf_dev->spi->irq = irq_of_parse_and_map(node,0);
		printk("gx368_gf3208 gf_dev->spi->irq =%d\n   ",gf_dev->spi->irq );

		ret = request_irq(gf_dev->spi->irq, (irq_handler_t) fingerprint_eint_interrupt_handler, IRQF_TRIGGER_RISING,"finger-eint", NULL);
		printk("gx368_gf3208 fingerprint_irq request_irq ret=%d\n   ",ret);
		//ret=request_threaded_irq(fingerprint_irq, NULL, fingerprint_eint_interrupt_handler, IRQF_TRIGGER_HIGH  | IRQF_ONESHOT, "gx368", NULL);	
		//printk("gx368/gf3208 gf_dev->spi->irq =%x\n   ",gf_dev->spi->irq);		
	}

	//mt_eint_registration(gf_dev->spi->irq, EINTF_TRIGGER_RISING, gf_irq, 1);
   	//mt_eint_mask(gf_dev->spi->irq); //mask interrupt
   	
	gf_disable_irq(gf_dev);


#if CONFIG_HAS_EARLYSUSPEND		
	//gf_dev->early_fp.level		= EARLY_SUSPEND_LEVEL_DISABLE_FB - 1,
	gf_dev->early_fp.suspend	= gf_early_suspend,		
	gf_dev->early_fp.resume		= gf_late_resume,    	
	register_early_suspend(&gf_dev->early_fp);
#endif

	gf_debug(DEFAULT_DEBUG,"GF installed.");

	return status;
err_free_input:
	if (gf_dev->input != NULL)
		input_free_device(gf_dev->input);
err_alloc_input:
err_creat_group:
	class_destroy(gf_spi_class);
err_creat_class:	
	unregister_chrdev(SPIDEV_MAJOR, SPI_DEV_NAME);
err_register_char:
//err_check_9p:
	if (gf_dev->buffer !=NULL)
		kfree(gf_dev->buffer);
err_alloc_buffer:	
	FUNC_EXIT();
	return status;
}

static int  gf_remove(struct spi_device *spi)
{
    struct gf_dev_t *gf_dev = spi_get_drvdata(spi);
    FUNC_ENTRY();

    /* make sure ops on existing fds can abort cleanly */
//    if(gf_dev->spi->irq) {
//		free_irq(gf_dev->spi->irq, gf_dev);
//    }

    gf_dev->spi = NULL;
    spi_set_drvdata(spi, NULL);

    /* prevent new opens */
    mutex_lock(&device_list_lock);
    sysfs_remove_group(&spi->dev.kobj, &gf_debug_attr_group);
    list_del(&gf_dev->device_entry);
    device_destroy(gf_spi_class, gf_dev->devt);
    clear_bit(MINOR(gf_dev->devt), minors);
    if (gf_dev->users == 0) {
		if(gf_dev->input != NULL)
		    input_unregister_device(gf_dev->input);

		if(gf_dev->buffer != NULL)
		    kfree(gf_dev->buffer);
    }
    mutex_unlock(&device_list_lock);
    class_destroy(gf_spi_class);
    unregister_chrdev(SPIDEV_MAJOR, SPI_DEV_NAME);
    FUNC_EXIT();
    return 0;
}
static int gf_suspend_test(struct device *dev)
{
   // g_debug_level |= SUSPEND_DEBUG;
   	printk("gx368/gf3208 gf: %s\n", __func__);
    return 0;
}

static int gf_resume_test(struct device *dev)
{
    //g_debug &= ~SUSPEND_DEBUG;
    printk("gx368/gf3208 gf: %s\n", __func__);
    return 0;
}
static const struct dev_pm_ops gf_pm = {
    .suspend = gf_suspend_test,
    .resume = gf_resume_test
};

static struct spi_driver gf_spi_driver = {
    .driver = {
	.name =		SPI_DEV_NAME,
	.owner =	THIS_MODULE,
	//.bus	= &spi_bus_type,
	.pm = &gf_pm,
    },
    .probe =	gf_probe,
    .remove =	gf_remove,

    //.suspend = gf_suspend_test,
    //.resume = gf_resume_test,

    /* NOTE:  suspend/resume methods are not necessary here.
     * We don't do anything except pass the requests to/from
     * the underlying controller.  The refrigerator handles
     * most issues; the controller driver handles the rest.
     */
};
/*-------------------------------------------------------------------------*/

static struct spi_board_info spi_board_devs[] __initdata = {
	[0] = {
	    .modalias=SPI_DEV_NAME,
		.bus_num = 1,
		.chip_select=0,
		.mode = SPI_MODE_0,
		.controller_data = &spi_conf_mt65xx,
	},
};

static int __init gf_init(void)
{
	int status = 0;

	printk("gx368 SPI driver.");
	//status = platform_driver_register(&fingerprint_dts_driver);
//	printk("gx368/gf3208 platform_driver_register status=%d\n",status);
	spi_register_board_info(spi_board_devs,ARRAY_SIZE(spi_board_devs));
	status = spi_register_driver(&gf_spi_driver);
	printk("gx368/gf3208 spi_register_driver status=%d\n",status);

	if (status < 0) {
		gf_error("Failed to register SPI driver.");
	}

	return status;
}
module_init(gf_init);

static void __exit gf_exit(void)
{
    spi_unregister_driver(&gf_spi_driver);
}
module_exit(gf_exit);

MODULE_DESCRIPTION("User mode SPI device interface");
MODULE_LICENSE("GPL");

