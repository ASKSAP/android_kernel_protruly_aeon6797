/*
 * File:         byd_fps_spi.c
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

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>  // gpio_set_value()
#include <linux/err.h>   // IS_ERR(), PTR_ERR(); for tiny4412
#include <linux/of.h>
#include <linux/of_irq.h>

#include "byd_algorithm.h"
#include "byd_fps_bf66xx.h" //#include <linux/input/byd_fps_bf66xx.h>
#include "byd_fps_libbf663x.h"
extern struct byd_fps_data *this_byd_fps;
int byd_power_set(struct spi_device *spi,int cmd,unsigned char set_time);
extern unsigned char byd_fps_chip_flag;
// -------------------------------------------------------------------- //
//#include "../fp_drv/fp_drv.h"
//#if FP_CONBAT
//static int byd_init_flag = 0;
//#endif
#include <linux/types.h>

#define BYD_USE_SPI_4GB

#ifdef BYD_USE_SPI_4GB
#include <linux/of_reserved_mem.h>

extern dma_addr_t SpiDmaBufTx_pa;
extern dma_addr_t SpiDmaBufRx_pa;
extern char *spi_tx_local_buf;
extern char *spi_rx_local_buf;
extern unsigned long long local_buf_size;
//extern int reserve_memory_spi_fn(struct reserved_mem *rmem);

		/* map physical addr to virtual addr */
extern int spi_local_buff_map(void);

#endif


/*** interface defined by byd_fps_bf663x.c, called by byd_fps_spi.c ***/
struct byd_fps_data *byd_fps_probe(struct spi_device *spi);
void byd_fps_remove(struct byd_fps_data *bydfps);
int byd_fps_platform_init(struct spi_device *spi);
int byd_fps_platform_exit(struct device *dev);
#if defined(BYD_FPS_INPUT_WAKEUP) && defined(CONFIG_PM_SLEEP)
extern const struct dev_pm_ops byd_fps_pm_ops;
#elif !defined(CONFIG_PM_SLEEP) && !defined(CONFIG_FB) && defined(CONFIG_HAS_EARLYSUSPEND)
int byd_fps_suspend(struct spi_device *spi, pm_message_t mesg); // Linux suspend/resume
int byd_fps_resume(struct spi_device *spi);
#endif

// -------------------------------------------------------------------- //

#ifdef CONFIG_FPS03
#define MAX_SPI_FREQ_HZ	18*1000*1000
#elif defined(CONFIG_FPS13)||defined(CONFIG_FPS12)||defined(CONFIG_FPS11)
#define MAX_SPI_FREQ_HZ	24*1000*1000
#else
#define MAX_SPI_FREQ_HZ	12*1000*1000
#endif

#define SPI_WR			1
#define SPI_RD			0

// -------------------------------------------------------------------- //

#ifdef PLATFORM_MTK_KERNEL3_10
		#include <cust_eint.h>
		extern void byd_fps_bf66xx_interrupt(void);
#else
		extern irqreturn_t byd_fps_bf66xx_interrupt(int irq, void *dev_id);
#endif
#if defined PLATFORM_MTK || defined PLATFORM_MTK_KERNEL3_10

#include <linux/irqchip/mt-eic.h>

#ifdef PLATFORM_MTK_KERNEL3_10
    #include <mach/mt_spi.h>	//"mt_spi.h" //struct mt_chip_conf
#else
	#include "mt_spi.h"
#endif 

struct mt_chip_conf spi_conf = {
	.setuptime = 10,
	.holdtime = 10,
	.high_time = 50,
	.low_time = 50,

	.cs_idletime = 10,
	.ulthgh_thrsh = 0,
	.cpol = 0,
	.cpha = 0,
	.rx_mlsb = SPI_MSB,
	.tx_mlsb = SPI_MSB,
	.tx_endian = 0,
	.rx_endian = 0,
	.com_mod = DMA_TRANSFER,
	.pause = 0,
	.finish_intr = 1,
	.deassert = 0,
	.ulthigh = 0,
	.tckdly = 0,
};

#endif
int byd_fps_spi_speed(struct spi_device *spi, int frq_khz)
{
	unsigned int half_time;
	half_time = 48*1000/(frq_khz);
	printk("%s:calc half_time=%d\n", __func__, half_time);
	half_time = half_time+half_time/5;
	printk("%s:half_time adjust=%d\n", __func__, half_time);
	printk("%s:Final half_time=%d\n", __func__, half_time);
	spi_conf.setuptime = half_time;
	spi_conf.holdtime = half_time;
	spi_conf.high_time = half_time;
	spi_conf.low_time = half_time;

	spi_conf.cs_idletime = half_time;
	if(spi_setup(spi) < 0){
		pr_warn("bf66xx:Failed to set spi.\n");
	}
	return 0;
}

// *******************************************************************************
// * Function    :  byd_fps_spi_xfer
// * Description :  spi transfer.
// * In          :  *spi, count, *tx_buf, *rx_buf.
// * Return      :  0--succeed, -1-- fail.
// *******************************************************************************
int byd_fps_spi_xfer(struct spi_device *spi,
			unsigned int count, unsigned char *tx_buf, unsigned char *rx_buf)
{
	int ret;
	struct spi_message m;


	static struct spi_transfer t = {
		.cs_change = 0,
		.delay_usecs = 0,
		.speed_hz = BYD_FPS_SPI_SPEED,
		.tx_dma = 0,
		.rx_dma = 0,
		.bits_per_word = 8,
	};
	t.tx_buf = tx_buf;
	t.rx_buf = rx_buf;
	t.len = count;
	#ifdef BYD_USE_SPI_4GB
	if(count >32){
		memcpy_toio(spi_tx_local_buf,tx_buf,count);
		memcpy_toio(spi_rx_local_buf,rx_buf,count);
		t.tx_buf = spi_tx_local_buf;
		t.rx_buf = spi_rx_local_buf;
		t.tx_dma = SpiDmaBufTx_pa;
		t.rx_dma = SpiDmaBufRx_pa;
		//FP_DBG("%s:byd use spi 4GB\n",__func__);
	}
	#endif
	#ifdef BYD_USE_SPI_4GB
	spi_local_buff_map();
	#endif
	if (count <= 0)
		return -1;
	//set SPI mode
	#if defined PLATFORM_MTK || defined PLATFORM_MTK_KERNEL3_10
	if (count > 32)
		spi_conf.com_mod = DMA_TRANSFER;
	else
		spi_conf.com_mod = FIFO_TRANSFER;
    #endif

	spi_message_init(&m);
	spi_message_add_tail(&t, &m);

	ret = spi_sync(spi, &m);
	if (ret) {
		pr_err("byd spi_sync failed.\n");
	}
	#ifdef BYD_USE_SPI_4GB
	if(count > 32)
	{
		//printk("%s:spi_rx_local_buf[0]:0x%x,spi_rx_local_buf[1]:0x%x,\n",__func__,spi_rx_local_buf[0],spi_rx_local_buf[1]);
		memcpy_fromio(rx_buf,spi_rx_local_buf,count);
		//printk("%s:rx_buf[0]:0x%x,rx_buf[1]:0x%x\n",__func__,rx_buf[0],rx_buf[1]);
	}
		
	#endif

	return ret;
}

// ---------------------- spi interface ------------------------------- //

/**
 * byd_fps_spi_block_write()
 * @spi: spi device
 * @first_reg: first register to be written to
 * @count: total number of bytes to be written to, plus 1
 * @buf: data to be written to
 *       note: the first byte should be reserved
 *
 * Return: status
 */
int byd_fps_spi_block_write(struct spi_device *spi,
				 u8 first_reg, unsigned int count, u8 *buf)
{
	u8 dummy[count];
	buf[0] = first_reg << 1 | SPI_WR;

	return byd_fps_spi_xfer(spi, count, buf, dummy);
}

/**
 * byd_fps_spi_read()
 * @spi: spi device
 * @reg: the register to be read
 *
 * Return: value of the register or error if less 0
 */
int byd_fps_spi_read(struct spi_device *spi, u8 reg)
{
	int ret;
	#ifdef SPI_TRANS_4BYTE//defined in byd_fps_bf66xx.h
	u8 tx_buf[4], rx_buf[4] = {0, 0,0,0};
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		tx_buf[0] = 0XBE;
	}else if(byd_fps_chip_flag == 0x22){
		tx_buf[0] = 0X90;
	}
	tx_buf[1] = 0X00;
	tx_buf[2] = reg << 1 | SPI_RD;
	tx_buf[3] = 0x00;
	ret = byd_fps_spi_xfer(spi, 4, tx_buf, rx_buf)? : rx_buf[3];
	#else
	u8 tx_buf[2], rx_buf[2] = {0, 0};
	tx_buf[0] = reg << 1 | SPI_RD;
	tx_buf[1] = 0x00;
	ret = byd_fps_spi_xfer(spi, 2, tx_buf, rx_buf)? : rx_buf[1];
	#endif

	return ret;
}

// *******************************************************************************
// * Function    :  byd_fps_spi_write
// * Description :  send value to register of fps chip.
// * In          :  @spi: spi device;
// *             :  @reg: the register to be written to;
// *             :  @val: the value to be written to the register;
// * Return      :  status
// *******************************************************************************
int byd_fps_spi_write(struct spi_device *spi, u8 reg, u8 val)
{
	#ifdef SPI_TRANS_4BYTE
	u8 tx_buf[4], dummy[4];
	if((byd_fps_chip_flag == 0x11)||(byd_fps_chip_flag == 0x12)){
		tx_buf[0] = 0XBE;
	}else if(byd_fps_chip_flag == 0x22){
		tx_buf[0] = 0X90;
	}
	tx_buf[1] = 0X00;
	tx_buf[2] = reg << 1 | SPI_WR;
	tx_buf[3] = val;
	return byd_fps_spi_xfer(spi, 4, tx_buf, dummy);

	#else
	u8 tx_buf[2], dummy[2];
	tx_buf[0] = reg << 1 | SPI_WR;
	tx_buf[1] = val;
	return byd_fps_spi_xfer(spi, 2, tx_buf, dummy);
	#endif
}

// -------------------------------------------------------------------- //
// probe()
// -------------------------------------------------------------------- //

// *******************************************************************************
// * Function    :  byd_fps_spi_probe
// * Description :  spi configure in "probe".
// * In          :  @spi: spi device;
// * Return      :  status
// *******************************************************************************
static int byd_fps_spi_probe(struct spi_device *spi)
{
	struct byd_fps_data  *bydfps = NULL;
	int err;

	printk("%s: name:%s, bus_num:%d, num_slave:%d, chip_select:%d, irq:%d \n",
			__func__, spi->modalias, spi->master->bus_num, spi->master->num_chipselect,
			 spi->chip_select, spi->irq);

#if defined PLATFORM_MTK || defined PLATFORM_MTK_KERNEL3_10
	spi->mode = SPI_MODE_0; //CPOL=CPHA=0
	spi->controller_data = (void *) &spi_conf;	//setup spi master's mode & frequency
 	spi->max_speed_hz = MAX_SPI_FREQ_HZ;
#ifdef PLATFORM_MTK
	spi->dev.of_node=of_find_compatible_node(NULL, NULL, "mediatek,mt6797-finger");
#if 0
		spidev->reg = regulator_get(&spi->dev, "vfp");
		ret = regulator_set_voltage(spidev->reg, 2800000, 2800000);	/*set 2.8v*/
		if (ret) {
			dev_err("regulator_set_voltage(%d) failed!\n", ret);
			return -1;
		}
#endif
#endif
#endif
	/* don't exceed max specified SPI CLK frequency */
	if (spi->max_speed_hz > MAX_SPI_FREQ_HZ) {
		dev_err(&spi->dev, "SPI CLK %d Hz?\n", spi->max_speed_hz);
		return -EINVAL;
	}

	/* --- port setup (gpio, spi, irq, reset) ------------------------------- */
	err = byd_fps_platform_init(spi);
	if(err != 0) {
		dev_err(&spi->dev, "byd_fps_platform_init failed.\n");
		return err;
	}
	
	bydfps = byd_fps_probe(spi);
	if (IS_ERR(bydfps))
		return PTR_ERR(bydfps);

	spi->bits_per_word = 8;
	//ret = mt_spi_setup(spi);
	err = spi_setup(spi); // must be after byd_fps_init_port()
	if (err) {
		dev_dbg(&spi->dev, "spi master doesn't support 8 bits/word\n");
		return err;
	}

	spi_set_drvdata(spi, bydfps); /*** save byd_fps_data ***/

// ---------------------- interrupt init ------------------------------ //

#ifdef PLATFORM_MTK_KERNEL3_10
	mt_eint_set_hw_debounce(spi->irq, 0);
	mt_eint_registration(spi->irq, EINT_TRIGGER_TYPE/* EINTF_TRIGGER_RISING CUST_EINTF_TRIGGER_RISING, CUST_EINTF_TRIGGER_FALLING*/, byd_fps_bf66xx_interrupt, 1);
	//mt_eint_mask(spi->irq);
	//mt_eint_unmask(spi->irq);
#else
	err = request_threaded_irq(spi->irq, NULL, byd_fps_bf66xx_interrupt,
			 EINT_TRIGGER_TYPE | IRQF_ONESHOT, spi->dev.driver->name, bydfps);
	if (err < 0) {
		pr_err("%s: Fatal Error: request_irq() failed: %d\n", __func__, err);
		return err;
	}
	
	enable_irq(spi->irq);
#endif
	dev_dbg(&spi->dev, "%s: irq request OK, irq = %d \n", __func__, spi->irq);
	dev_dbg(&spi->dev, "%s: HZ=%d \n", __func__, HZ);
//#if FP_CONBAT
	//byd_init_flag = 1;
//#endif
//#ifdef CONFIG_BSP_TW_HWINFO
//	{
		//extern char twdevinfo[];
		//strcat(twdevinfo, "FINGER: ");
		//strcat(twdevinfo, "BYD finger 2017-03-17");
		//strcat(twdevinfo,"(*)");
		//strcat(twdevinfo,"\n");
	//}
//#endif	 
	return 0;
}

// -------------------------------------------------------------------- //
// remove()
// -------------------------------------------------------------------- //

// *******************************************************************************
// * Function    :  byd_fps_spi_remove
// * Description :  remove resource, clear spi  when driver exit.
// * In          :  @spi: spi device;
// * Return      :  status
// *******************************************************************************
static int byd_fps_spi_remove(struct spi_device *spi)
{
	struct byd_fps_data *bydfps = spi_get_drvdata(spi);

	byd_fps_remove(bydfps);
	if(spi->irq >= 0)
		free_irq(spi->irq, (void*)0);

	byd_fps_platform_exit(&spi->dev);
	spi_set_drvdata(spi, NULL);

	return 0;
}

#ifdef PLATFORM_MTK_KERNEL3_10
struct spi_device_id byd_spi_id_table = {BYD_FPS_NAME, 0};
#else
static struct of_device_id fp_bf66xx_match_table[] = {
	{ .compatible = "mediatek,mt6797-finger",},
	{ },
};
#endif
//static const struct spi_device_id byd_id = {"madev", 1};

//MODULE_DEVICE_TABLE(spi, byd_id);

static struct spi_driver byd_fps_driver = {
	.driver = {
		.name = BYD_FPS_NAME,
		.owner = THIS_MODULE,
	  #ifndef PLATFORM_MTK_KERNEL3_10
		.of_match_table = fp_bf66xx_match_table,
	  #endif
	  #if defined(BYD_FPS_INPUT_WAKEUP) && defined(CONFIG_PM_SLEEP) \
	      && !defined(CONFIG_FB) && !defined(CONFIG_HAS_EARLYSUSPEND)
		//.pm = &byd_fps_pm_ops,
	  #endif
		.bus = &spi_bus_type,
	},
	//.id_table = &byd_id,
	.probe = byd_fps_spi_probe,
	.remove = byd_fps_spi_remove, //.remove = __devexit_p(byd_fps_spi_remove),
  #ifdef PLATFORM_MTK_KERNEL3_10
	.id_table = &byd_spi_id_table,
  #endif
	#if defined(BYD_FPS_INPUT_WAKEUP) && !defined(CONFIG_PM_SLEEP) \
    && !defined(CONFIG_FB) && defined(CONFIG_HAS_EARLYSUSPEND)
	//.suspend = byd_fps_suspend,
	//.resume = byd_fps_resume,
	#endif
};
#if 0
//#if FP_CONBAT
static int byd_local_init(void)
{
	printk("***************************************************\n");
	printk("*           Init BYD Fingerprint Driver  \n");
	printk("*               %s \n", BYD_FPS_NAME);
//	printk("* %s \n", BYD_FPS_DRIVER_DESCRIPTION);
	printk("***************************************************\n");
#if defined PLATFORM_MTK || defined PLATFORM_MTK_KERNEL3_10
	//ret = spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
#endif

	if (spi_register_driver(&byd_fps_driver)) {
		printk("*%s: spi_register_driver failured!\n",__func__);
		return -EINVAL;
	}
	
	return 0;
}

static int byd_local_uninit(void)
{
	printk("%s\n",__func__);
	spi_unregister_driver(&byd_fps_driver);
	return 0;
}

static int byd_init_ok(void)
{
	return byd_init_flag;
}

static struct fp_driver_t byd_device_driver = {
	.device_name = BYD_FPS_NAME, 
	.local_init = byd_local_init,
	.local_uninit = byd_local_uninit,
	.init_ok = byd_init_ok,
 };

static int byd_fps_init(void)
{
	fp_driver_load(&byd_device_driver);

	return 0;
}

static void byd_fps_exit(void)
{
	fp_driver_remove(&byd_device_driver);
}
#else
// *******************************************************************************
// * Function    :  byd_fps_init
// * Description :  register spi  driver when module loaded.
// * In          :  void.
// * Return      :  0--succeed, -EINVAL--fail to register spi driver.
// *******************************************************************************
static int __init byd_fps_init(void)
{
#if defined PLATFORM_MTK || defined PLATFORM_MTK_KERNEL3_10
	int ret;
	static struct spi_board_info spi_board_devs[] __initdata = {
		[0] = {
			.modalias = BYD_FPS_NAME,
			.max_speed_hz = MAX_SPI_FREQ_HZ,
			.bus_num = 1,
			.chip_select = 1,
			.mode = SPI_MODE_0,
			.controller_data = &spi_conf,
		},
	};
#endif

	printk("***************************************************\n");
	printk("*           Init BYD Fingerprint Driver  \n");
	printk("*               %s \n", BYD_FPS_NAME);
//	printk("* %s \n", BYD_FPS_DRIVER_DESCRIPTION);
	printk("***************************************************\n");
#if defined PLATFORM_MTK || defined PLATFORM_MTK_KERNEL3_10
	ret = spi_register_board_info(spi_board_devs, ARRAY_SIZE(spi_board_devs));
	printk("%s, spi_register_board_info ret=%d\n", __func__, ret);
#endif

	if (spi_register_driver(&byd_fps_driver)) {
		printk("*%s: spi_register_driver failured!\n",__func__);
		return -EINVAL;
	}

	return 0;
}

// *******************************************************************************
// * Function    :  byd_fps_exit
// * Description :  unregister spi  driver when module unloaded.
// * In          :  void.
// * Return      :  void.
// *******************************************************************************
static void __exit byd_fps_exit(void)
{
	printk("%s\n",__func__);
	#ifdef BYD_FPS_POWER_ON
	byd_power_set(this_byd_fps->spi,0,0);
	#endif
	spi_unregister_driver(&byd_fps_driver);
}
#endif
module_init(byd_fps_init);
module_exit(byd_fps_exit);

MODULE_AUTHOR("Simon Chee <qi.ximing@byd.com>");
MODULE_DESCRIPTION("BYD fingerprint chip SPI bus driver");
MODULE_LICENSE("GPL");
