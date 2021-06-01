/***************************************************************************
 * Filename:
 * ---------
 *  mlx90615.c
 *
 * Project:
 * --------
 *
 * Description:
 * ------------
 *
 * Author:
 * -------
 *  LiangChi Huang, ext 25609, LiangChi.Huang@mediatek.com, 2012-08-09
 *
 *****************************************************************************/

#define pr_fmt(fmt) "["KBUILD_MODNAME"]" fmt
/*****************************************************************************
 * Include
 *****************************************************************************/
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#endif


#ifndef CONFIG_MTK_FPGA
#include <mt_clkbuf_ctl.h>	/*  for clock buffer */
#endif

/* #include <cust_eint.h> */
/* #include <cust_i2c.h>  */

#if defined(CONFIG_MTK_LEGACY)
#include <mach/mt_gpio.h>
#include <cust_gpio_usage.h>
#endif

/*****************************************************************************
 * Define
 *****************************************************************************/

#define I2C_ID_NAME "MLX90615"

#define MAX_BUFFER_SIZE	    100
#define NFC_CLIENT_TIMING   100	/* I2C speed */

#define ACK	            0
#define	NACK            1

//MLX90614 constants
#define SA				0x00	// Slave address
#define DEFAULT_SA		0x5A	// Default Slave address
#define RAM_Access		0x00	// RAM access command
#define EEPROM_Access	0x20	// EEPROM access command
#define RAM_Tobj1		0x07	// To1 address in the eeprom

//ioctl
#define TEMP_SEN_IO  'T'
#define TEMP_SEN_WAKE_UP_CMD   _IO(TEMP_SEN_IO, 0x01)
#define TEMP_SEN_SLEEP_CMD 	   _IO(TEMP_SEN_IO, 0x02)


#if !defined(CONFIG_MTK_LEGACY)
struct platform_device *temp_plt_dev = NULL;
struct pinctrl *temp_pctrl = NULL;
struct pinctrl_state *wake_up_pin_default = NULL;
struct pinctrl_state *wake_up_pin_l = NULL;
struct pinctrl_state *power_on_pin_l = NULL;
struct pinctrl_state *power_on_pin_h = NULL;
#endif

/*****************************************************************************
 * Global Variable
 *****************************************************************************/
struct mlx90615_dev _mlx90615_dev;


/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int mt_temp_probe(struct platform_device *pdev);
static int mt_temp_remove(struct platform_device *pdev);

static int mlx90615_probe(struct i2c_client *client,const struct i2c_device_id *id);
static int mlx90615_remove(struct i2c_client *client);
static int mlx90615_dev_open(struct inode *inode, struct file *filp);
static long mlx90615_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,unsigned long arg);
static ssize_t mlx90615_dev_read(struct file *filp, char __user *buf,size_t count, loff_t *offset);
static ssize_t mlx90615_dev_write(struct file *filp, const char __user *buf,size_t count, loff_t *offset);
static int mlx90615_dev_enable(struct i2c_client *client,bool on);


/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct mlx90615_dev 
{
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	struct i2c_client *client;
	struct miscdevice mlx90615_device;
};

static const struct i2c_device_id mlx90615_id[] = 
{
	{I2C_ID_NAME, 0},
	{}
};


#ifdef CONFIG_OF
static const struct of_device_id temp_switch_of_match[] = {
	{.compatible = "mediatek,temp"},
	{},
};
#endif


static struct i2c_driver mlx90615_dev_driver = 
{
	.id_table = mlx90615_id,
	.probe =    mlx90615_probe,
	.remove =   mlx90615_remove,
	/* .detect               = mlx90615_detect, */
	.driver = 
	   {
       .name =  I2C_ID_NAME,
       .owner = THIS_MODULE,
#ifdef CONFIG_OF
       .of_match_table = temp_switch_of_match,
#endif
       },
};


static const struct file_operations mlx90615_dev_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.read =  mlx90615_dev_read,
	.write = mlx90615_dev_write,
	.open =  mlx90615_dev_open,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mlx90615_dev_unlocked_ioctl,
#endif
	.unlocked_ioctl = mlx90615_dev_unlocked_ioctl,
};


static const struct of_device_id temp_dev_of_match[] = {
	{.compatible = "mediatek,temp-sen",},
	{},
};


static struct platform_driver mtk_temp_platform_driver = {
	.probe = mt_temp_probe,
	.remove = mt_temp_remove,
	.driver = {
		   .name = I2C_ID_NAME,
		   .owner = THIS_MODULE,
#ifdef CONFIG_OF
		   .of_match_table = temp_dev_of_match,
#endif
		   },
};


unsigned char PEC_Check(unsigned char pec[])
{
	unsigned char 	crc[6];
	unsigned char	BitPosition=47;
	unsigned char	shift;
	unsigned char	i;
	unsigned char	j;
	unsigned char	temp;

	do{
		crc[5]=0;				/* Load CRC value 0x000000000107 */
		crc[4]=0;
		crc[3]=0;
		crc[2]=0;
		crc[1]=0x01;
		crc[0]=0x07;
		BitPosition=47;			/* Set maximum bit position at 47 */
		shift=0;
				
		//Find first 1 in the transmited message
		i=5;					/* Set highest index */
		j=0;
		while((pec[i]&(0x80>>j))==0 && i>0){
			BitPosition--;
			if(j<7){
				j++;
			}
			else{
				j=0x00;
				i--;
			}
		}/*End of while */
		
		shift=BitPosition-8;	/*Get shift value for crc value*/
		
		
		//Shift crc value 
		while(shift){
			for(i=5; i<0xFF; i--){
				if((crc[i-1]&0x80) && (i>0)){
					temp=1;
				}
				else{
					temp=0;
				}
				crc[i]<<=1;
				crc[i]+=temp;
			}/*End of for*/
			shift--;
		}/*End of while*/
		
		
		//Exclusive OR between pec and crc		
		for(i=0; i<=5; i++){
			pec[i] ^=crc[i];
		}/*End of for*/
	}while(BitPosition>8);/*End of do-while*/
	
	return pec[0];
}/*End of PEC_calculation*/


#if !defined(CONFIG_MTK_LEGACY)
static int mt_temp_pinctrl_select(struct pinctrl *p, struct pinctrl_state *s)
{
	int ret = 0;

	if (p != NULL && s != NULL) {
		ret = pinctrl_select_state(p, s);
	} else {
		pr_debug("%s : pinctrl_select err\n", __func__);
		ret = -1;
	}
	return ret;
}

static int mt_temp_pinctrl_init(struct platform_device *pdev)
{
	int ret = 0;

	temp_pctrl = devm_pinctrl_get(&pdev->dev);
	if (IS_ERR(temp_pctrl)) {
		dev_err(&pdev->dev, "Cannot find pinctrl!");
		ret = PTR_ERR(temp_pctrl);
		goto end;
	}

	wake_up_pin_default = pinctrl_lookup_state(temp_pctrl, "default");
	if (IS_ERR(wake_up_pin_default)) {
		ret = PTR_ERR(wake_up_pin_default);
		pr_debug("%s : pinctrl err, wake_up_pin_default\n", __func__);
	}

	wake_up_pin_l = pinctrl_lookup_state(temp_pctrl, "wake_up_low");
	if (IS_ERR(wake_up_pin_l)) {
		ret = PTR_ERR(wake_up_pin_l);
		pr_debug("%s : pinctrl err, wake_up_pin_l\n", __func__);
	}

	power_on_pin_l = pinctrl_lookup_state(temp_pctrl, "power_on_low");
	if (IS_ERR(power_on_pin_l)) {
		ret = PTR_ERR(power_on_pin_l);
		pr_debug("%s : pinctrl err, power_on_pin_l\n", __func__);
	}

	power_on_pin_h = pinctrl_lookup_state(temp_pctrl, "power_on_high");
	if (IS_ERR(power_on_pin_h)) {
		ret = PTR_ERR(power_on_pin_h);
		pr_debug("%s : pinctrl err, power_on_pin_h\n", __func__);
	}	
	
	ret = mt_temp_pinctrl_select(temp_pctrl, wake_up_pin_default);
end:

	return ret;
}
#endif

static int mlx90615_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret;

	printk("mlx90615_dev_probe\n");

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -ENODEV;

	
	_mlx90615_dev.client = client;
	
	
	/* init mutex and queues */
	//init_waitqueue_head(&_mlx90615_dev.read_wq);
	//mutex_init(&_mlx90615_dev.read_mutex);

	i2c_set_clientdata(client, &_mlx90615_dev);
	
    mlx90615_dev_enable(_mlx90615_dev.client,false);//enter sleep
	
	_mlx90615_dev.mlx90615_device.minor = MISC_DYNAMIC_MINOR;
	_mlx90615_dev.mlx90615_device.name = "temp";
	_mlx90615_dev.mlx90615_device.fops = &mlx90615_dev_fops;

	ret = misc_register(&_mlx90615_dev.mlx90615_device);
	
	if (ret) 
	{
		pr_debug("%s : misc_register failed\n", __FILE__);
		return ret;
	}

	return 0;
}

static int mlx90615_remove(struct i2c_client *client)
{
	pr_debug("mlx90615_remove\n");
	
	misc_deregister(&_mlx90615_dev.mlx90615_device);
	//mutex_destroy(&_mlx90615_dev.read_mutex);
	/* kfree(mlx90615_dev); */
	return 0;
}

static int mt_temp_probe(struct platform_device *pdev)
{
	int ret = 0;

#if !defined(CONFIG_MTK_LEGACY)

	temp_plt_dev = pdev;

	pr_debug("%s : &temp_plt_dev=%p\n", __func__, temp_plt_dev);

	ret = mt_temp_pinctrl_init(pdev);

#endif

	return 0;
}

static int mt_temp_remove(struct platform_device *pdev)
{
	pr_debug("%s : &pdev=%p\n", __func__, pdev);
	return 0;
}



static ssize_t mlx90615_dev_read(struct file *filp, char __user *buf,size_t count, loff_t *offset)
{
	struct mlx90615_dev *mlx90615_dev = filp->private_data;
	
	int     ret = 0;
	int     read_retry = 5;
	char    read_buf[32],read_buf_Ta[32];

    printk("mlx90615_dev_read !\n");
	
	memset(read_buf,0,sizeof(read_buf));

	/* Read data */
	while (read_retry) 
	{
	    ret=i2c_smbus_read_i2c_block_data(mlx90615_dev->client,0x27,3,read_buf);
		if (ret < 0)
		{
			pr_debug("%s: i2c_master_recv failed: %d, read_retry: %d\n",__func__, ret, read_retry);
		    read_retry--;
			continue;
		}
		break;
	}
	read_retry = 5;
	while (read_retry) 
	{
	    ret=i2c_smbus_read_i2c_block_data(mlx90615_dev->client,0x26,3,read_buf_Ta);
		if (ret < 0)
		{
			pr_debug("%s: i2c_master_recv failed: %d, read_retry: %d\n",__func__, ret, read_retry);
		    read_retry--;
			continue;
		}
		break;
	}
	
	if (ret < 0)
	{
		pr_err("%s: i2c_master_recv failed: %d, read_retry: %d\n",__func__, ret, read_retry);
		return ret;
	}
	
	printk("mlx90615_dev_read T0 temp= %x %x %x %x  \n",read_buf[0],read_buf[1],read_buf[2],read_buf[3]);
    printk("mlx90615_dev_read Ta temp= %x %x %x %x  \n",read_buf_Ta[0],read_buf_Ta[1],read_buf_Ta[2],read_buf_Ta[3]);
	read_buf[2] = read_buf_Ta[0];
	read_buf[3] = read_buf_Ta[1];
	printk("mlx90615_dev_read T0-TA temp= %x %x %x %x  \n",read_buf[0],read_buf[1],read_buf[2],read_buf[3]);
	
	if (copy_to_user(buf, read_buf, 4)) 
	{
		pr_debug("%s : failed to copy to user space\n", __func__);
		return -EFAULT;
	}

	return 0;

}


static ssize_t mlx90615_dev_write(struct file *filp, const char __user *buf,size_t count, loff_t *offset)
{
	struct mlx90615_dev *mlx90615_dev;
	//char wr_buf[20];
	//s32  ret = 0;
	
	mlx90615_dev = filp->private_data;

	return 0;
	/*
	if(count>20)
	   count=20;
	
      if (copy_from_user(wr_buf,buf, count)) 
	{
		pr_debug("%s : failed to copy from user space.\n", __func__);
		return -EFAULT;
	}

	ret =i2c_smbus_write_i2c_block_data(mlx90615_dev->client,wr_buf[0],count-1,&wr_buf[1]);

	if (ret<0) 
	{
        pr_debug("%s : i2c_master_send returned %d\n", __func__, ret);
	    ret = -EIO;
	    return ret;
	}

	return ret;
	*/
}



static int mlx90615_dev_open(struct inode *inode, struct file *filp)
{

	struct mlx90615_dev *mlx90615_dev = container_of(filp->private_data, struct mlx90615_dev, mlx90615_device);

	filp->private_data = mlx90615_dev;

	return 0;
}

static int mlx90615_dev_enable(struct i2c_client *client,bool on)
{
  int32_t ret=0;
  if(on)
  {
	 ret=mt_temp_pinctrl_select(temp_pctrl, power_on_pin_h);
	 msleep(3);
    // ret=mt_temp_pinctrl_select(temp_pctrl, wake_up_pin_l);
	// msleep(10);
	 ret=mt_temp_pinctrl_select(temp_pctrl, wake_up_pin_default);
	 msleep(3);
  }
  else  //sleep
  {
     ret =i2c_smbus_write_byte(client,0xc6);
     msleep(3);
	 ret=mt_temp_pinctrl_select(temp_pctrl, power_on_pin_l);	
	 
     if (ret<0) 
     {
        pr_debug("%s : mlx90615_dev_enable returned %d\n", __func__, ret);
	    ret = -EIO;
	    return ret;
     }
  }
  return ret;
}

static long mlx90615_dev_unlocked_ioctl(struct file *filp, unsigned int cmd,unsigned long arg)
{
     struct mlx90615_dev *mlx90615_dev = filp->private_data;
	 int err = 0;
	 
	 printk( "mlx90615_dev_unlocked_ioctl,cmd,%d,%lu\n",cmd,arg);

	 switch(cmd)
	 {
     case TEMP_SEN_WAKE_UP_CMD:
	 	  printk( "mlx90615_dev_enable wake up\n");
	 	  err=mlx90615_dev_enable(mlx90615_dev->client,true);
	 	  break;
	 case TEMP_SEN_SLEEP_CMD:
	 	  printk( "mlx90615_dev_enable sleep\n");
	 	  err=mlx90615_dev_enable(mlx90615_dev->client,false);
	 	  break;
	 };

	 return err;
}


/*
 * module load/unload record keeping
*/
static int __init mlx90615_dev_init(void)
{
	int ret = 0;

	printk("mlx90615_dev_init\n");

	ret=platform_driver_register(&mtk_temp_platform_driver);
	if (ret) {
		printk("[mlx90615_dev_init] mtk_temp_platform_driver fail ~");
		return ret;
	}

	i2c_add_driver(&mlx90615_dev_driver);

	printk("mlx90615_dev_init success\n");
	return 0;
}


static void __exit mlx90615_dev_exit(void)
{
	printk("mlx90615_dev_exit\n");

	i2c_del_driver(&mlx90615_dev_driver);
}


module_init(mlx90615_dev_init);
module_exit(mlx90615_dev_exit);

MODULE_AUTHOR("LiangChi Huang");
MODULE_DESCRIPTION("MTK TEMP driver");
MODULE_LICENSE("GPL");
