/*
 * File:         byd_fps_platform.c
 *
 * Created:	     2015-05-05
 * Description:  BYD fingerprint driver - MTK platform dependent part
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
#include <linux/gpio.h>
//#include <linux/delay.h> // msleep()

#include "byd_algorithm.h"
#include "byd_fps_bf66xx.h" //#include <linux/input/byd_fps_bf66xx.h>
#include "byd_fps_libbf663x.h"
/******************************************************************************/
#ifdef PLATFORM_MTK // Kernel 3.18 version
/******************************************************************************/

#include <mach/gpio_const.h>
#include "mt_spi.h"
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>

extern struct byd_fps_data *this_byd_fps;
void byd_fps_mdelay(unsigned int msecs);
#ifdef BYD_FPS_POWER_ON
int byd_power_set(struct spi_device *spi,int cmd,unsigned char set_time)
{
	int ret = 0;
	struct pinctrl *pinctrl1;
	struct pinctrl_state *fingerprint_power_off;
	struct pinctrl_state *fingerprint_power_on;
	
	pinctrl1 = devm_pinctrl_get(&spi->dev);
	if (IS_ERR(pinctrl1)) {
    	ret = PTR_ERR(pinctrl1);
    	dev_err(&spi->dev, "Cannot find fp pinctrl1!\n");
    	return ret;
	}
	printk("%s:byd_fps spidev_dts_init start \n",__func__);
	
	switch (cmd) {
		case 0:
			fingerprint_power_off = pinctrl_lookup_state(pinctrl1, "power_low");
			if (IS_ERR(fingerprint_power_off)) {
				ret = PTR_ERR(fingerprint_power_off);
				dev_err(&spi->dev, "Cannot find pinctrl1 power_low!\n");
				return ret;
			}
			pinctrl_select_state(pinctrl1,fingerprint_power_off);
			byd_fps_mdelay(set_time);
			break;
		case 1:
			fingerprint_power_on = pinctrl_lookup_state(pinctrl1, "power_high");
			if (IS_ERR(fingerprint_power_on)) {
				ret = PTR_ERR(fingerprint_power_on);
				dev_err(&spi->dev, "Cannot find pinctrl1 power_high!\n");
				return ret;
			}
			pinctrl_select_state(pinctrl1,fingerprint_power_on);
			byd_fps_mdelay(set_time);
			break;
	}
	
	return 0;
}
#endif
#ifdef BYD_FPS_POWER_CTRL
 
void byd_power_control(unsigned char down_time,unsigned char up_time)
{
	int ret = 0;

	ret = byd_power_set(this_byd_fps->spi,0,down_time); //掉电
	if (ret) {
			dev_err(&this_byd_fps->spi->dev, "Could not set power low.\n");
		}
		
	ret = byd_power_set(this_byd_fps->spi,1,up_time);   //上电
	if (ret) {
			dev_err(&this_byd_fps->spi->dev, "Could not set power high.\n");
		}
}
#endif

/**
 * byd_fps_platform_init() - platform specific configuration
 * @spi:	spi device
 *
 *  This function is usually used for interrupt pin or SPI pin configration
 * during initialization of FPS driver.
 * Return: status
 */
int byd_fps_platform_init(struct spi_device *spi)
{
	int ret = 0;
	struct pinctrl_state *eint_as_int;
	struct pinctrl *pinctrl1;
	
	struct device_node *node;
	u32 ints[2] = {0, 0};
	pinctrl1 = devm_pinctrl_get(&spi->dev);
	if (IS_ERR(pinctrl1)) {
    	ret = PTR_ERR(pinctrl1);
    	dev_err(&spi->dev, "Cannot find fp pinctrl1!\n");
    	return ret;
	}
	printk("%s:byd_fps spidev_dts_init start \n",__func__);
	node = of_find_compatible_node(NULL, NULL, "mediatek,mt6797-finger");
	
	if (node) {
        printk("%s:byd_fps_fp_pinctrl\n",__func__);
        	eint_as_int = pinctrl_lookup_state(pinctrl1, "eint_as_int");
        if (IS_ERR(eint_as_int)) {
        	ret = PTR_ERR(eint_as_int);
        	dev_err(&spi->dev, "Cannot find fp pinctrl bf_eint_as_int!\n");
        	return ret;
        }
		printk("%s: byd_fps_irq_start\n",__func__);
		of_property_read_u32_array( node, "debounce", ints, ARRAY_SIZE(ints));
		gpio_request(ints[0], "fingerprint-irq");
		gpio_set_debounce(ints[0], ints[1]);
		printk("%s:ints[0] = %d, ints[1] = %d!!\n", __func__,ints[0], ints[1]);
		spi->irq = irq_of_parse_and_map(node, 0);
		printk("%s:spi->irq = %d\n", __func__, spi->irq);
		pinctrl_select_state(pinctrl1,eint_as_int);
		if (spi->irq<0)
		{
			printk("%s:irq_of_parse_and_map fail!!\n",__func__);
			return -EINVAL;
		}
	} else {
		printk("byd_fps null dev node\n");
		return -EINVAL;
	}
	return ret;
}

/**
 * byd_fps_platform_exit() - clean up for platform specific configuration
 * @dev:	pointer to device
 *
 * Return: status
 */
int byd_fps_platform_exit(struct device *dev)
{
	return 0;
}

#endif // def PLATFORM_MTK

/******************************************************************************/
#ifdef PLATFORM_MTK_KERNEL3_10
/******************************************************************************/

#include <mach/gpio_const.h>
#include <mach/mt_gpio.h>
#include <mach/mt_pm_ldo.h>
#include <mach/mt_typedefs.h>
#include <mach/mt_boot.h>
#include "cust_gpio_usage.h"

#ifdef MTK_SPI_INIT_BUG_FIX // for client specific bug - SPI init failure
static unsigned char byd_mtk_spi_init = 0; // for mtk specific bug - spi init failure
#endif
int byd_fps_init_port(void);


/**
 * byd_fps_platform_exit() - clean up for platform specific configuration
 * @dev:	pointer to device
 *
 * Return: status
 */
int byd_fps_platform_exit(struct device *dev)
{
#ifdef BYD_FPS_EINT_PORT
#endif
	return 0;
}
#ifdef BYD_FPS_POWER_CTRL
void byd_fps_mdelay(unsigned int msecs);
void byd_power_enable(unsigned char up_time)
{		/*rst pin referring to samsung KIT.*/
	printk("%s: start\n", __func__);
	mt_set_gpio_mode(BYD_POWER_PIN, BYD_POWER_PIN_M_GPIO);
	mt_set_gpio_dir(BYD_POWER_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(BYD_POWER_PIN, GPIO_OUT_ONE);
    //msleep(60);
	byd_fps_mdelay(up_time);
}

void byd_power_disable(unsigned char down_time)
{		/*rst pin referring to samsung KIT.*/
	printk("%s: start\n", __func__);
	mt_set_gpio_mode(BYD_POWER_PIN, BYD_POWER_PIN_M_GPIO);
	mt_set_gpio_dir(BYD_POWER_PIN, GPIO_DIR_OUT);
	mt_set_gpio_out(BYD_POWER_PIN, GPIO_OUT_ZERO);
    //msleep(60);
	byd_fps_mdelay(down_time);
}

void byd_power_control(unsigned char down_time,unsigned char up_time)
{
	byd_power_disable(down_time);
	
	byd_power_enable(up_time);

}
#endif
static void byd_spi_gpioinit(void)
{
	printk("%s: start\n", __func__);
	
	mt_set_gpio_mode(BYD_FPS_CSS_PORT, BYD_FPS_CSS_PORT_MODE);
	mt_set_gpio_pull_enable(BYD_FPS_CSS_PORT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(BYD_FPS_CSS_PORT, GPIO_PULL_UP);

	mt_set_gpio_mode(BYD_FPS_SCK_PORT, BYD_FPS_SCK_PORT_MODE);
	mt_set_gpio_pull_enable(BYD_FPS_SCK_PORT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(BYD_FPS_SCK_PORT, GPIO_PULL_DOWN);

	mt_set_gpio_mode(BYD_FPS_MISO_PORT, BYD_FPS_MISO_PORT_MODE);
	mt_set_gpio_pull_enable(BYD_FPS_MISO_PORT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(BYD_FPS_MISO_PORT, GPIO_PULL_DOWN);

	mt_set_gpio_mode(BYD_FPS_MOSI_PORT, BYD_FPS_MOSI_PORT_MODE);
	mt_set_gpio_pull_enable(BYD_FPS_MOSI_PORT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(BYD_FPS_MOSI_PORT, GPIO_PULL_DOWN);

	printk("%s: end\n", __func__);
}

/**
 * byd_fps_platform_init() - platform specific configuration
 * @spi:	spi device
 *
 *  This function is usually used for interrupt pin or SPI pin configration
 * during initialization of FPS driver.
 * Return: status
 */
int byd_fps_platform_init(struct spi_device *spi)
{
	int ret = 0;

	byd_fps_init_port(); // spi port only
#ifdef MTK_SPI_INIT_BUG_FIX // for client specific bug - SPI init failure
	byd_mtk_spi_init = 0; // re-clean flag for re-init SPI port
#endif
#ifdef BYD_FPS_RESET_PORT
	mt_set_gpio_mode(BYD_FPS_RESET_PORT, GPIO_FP_RST_PIN_M_GPIO);
	mt_set_gpio_dir(BYD_FPS_RESET_PORT, GPIO_DIR_OUT);
	mt_set_gpio_pull_enable(BYD_FPS_RESET_PORT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(BYD_FPS_RESET_PORT, GPIO_PULL_UP);
	mt_set_gpio_out(BYD_FPS_RESET_PORT, GPIO_OUT_ONE);
	FP_DBG("%s: Init in PLATFORM, rst_gpio is 0x%x .", __func__, BYD_FPS_RESET_PORT);
#endif
#ifdef BYD_FPS_POWER_CTRL	
	byd_power_enable(0);
#endif
#ifdef BYD_FPS_EINT_PORT
	mt_set_gpio_mode(BYD_FPS_EINT_PORT, BYD_FPS_EINT_PORT_MODE);
	mt_set_gpio_dir(BYD_FPS_EINT_PORT, GPIO_DIR_IN);
	mt_set_gpio_pull_enable(BYD_FPS_EINT_PORT, GPIO_PULL_ENABLE);
	mt_set_gpio_pull_select(BYD_FPS_EINT_PORT, GPIO_PULL_UP);
	FP_DBG("%s: Init in PLATFORM, irq_gpio is 0x%x .", __func__, BYD_FPS_EINT_PORT);
#endif

#ifdef BYD_FPS_IRQ_NUM
	spi->irq = BYD_FPS_IRQ_NUM;
	FP_DBG("%s: BYD_FPS_IRQ_NUM is defined as %d\n", __func__, BYD_FPS_IRQ_NUM);
#elif defined(BYD_FPS_EINT_PORT)
	if (spi->irq < 0) {
		int irq;
		irq = gpio_to_irq(BYD_FPS_EINT_PORT); //byd_fps_platform_data->irq_gpio;
		if (irq <= 0)
			pr_err("%s: gpio_to_irq() failed! Port:%d\n", __func__, BYD_FPS_EINT_PORT);
		spi->irq = irq;
		FP_DBG("%s: gpio_to_irq(%d) is %d!\n", __func__, BYD_FPS_EINT_PORT, irq);
	}
#endif

	if (spi->irq <= 0) {
		pr_err("%s: Fatal Error: invalid IRQ number:%d \n", __func__, spi->irq);
		byd_fps_platform_exit(&spi->dev);
		return -1;
	}
	return ret;
}


/*
 * byd_fps_init_port() - interrupt pin (and sometimes SPI pin) configration
 *
 * Return      :  status
 */
int byd_fps_init_port(void)
{
#ifdef MTK_SPI_INIT_BUG_FIX // for client specific bug - SPI init failure
  if (byd_mtk_spi_init == 0) {
#endif
	FP_DBG("%s: initialize SPI port\n", __func__);
	byd_spi_gpioinit();
#if 0
	mt_set_gpio_mode(BYD_FPS_CSS_PORT, BYD_FPS_CSS_PORT_MODE);
	mt_set_gpio_mode(BYD_FPS_SCK_PORT, BYD_FPS_SCK_PORT_MODE);
	#ifdef DEBUG
	{
		int ret = 0;
		ret = mt_get_gpio_dir(BYD_FPS_MISO_PORT);
		printk("Sean_Debug -%s: DIR is 0x%x now! \n", __func__, ret);
		ret = mt_get_gpio_pull_enable(BYD_FPS_MISO_PORT);
		printk("Sean_Debug -%s: PULL enable is 0x%x now! \n", __func__, ret);
		ret = mt_get_gpio_pull_select(BYD_FPS_MISO_PORT);
		printk("Sean_Debug -%s: PULL select is 0x%x now! \n", __func__, ret);
		//mt_set_gpio_dir(GPIO_SPI_MISO_PIN, GPIO_DIR_IN);
		//mt_set_gpio_pull_enable(GPIO_SPI_MISO_PIN, GPIO_PULL_ENABLE);
		//mt_set_gpio_pull_select(GPIO_SPI_MISO_PIN, GPIO_PULL_UP);
	}
	#endif
	mt_set_gpio_mode(BYD_FPS_MISO_PORT, BYD_FPS_MISO_PORT_MODE);
	mt_set_gpio_mode(BYD_FPS_MOSI_PORT, BYD_FPS_MOSI_PORT_MODE);
	
#endif
#ifdef MTK_SPI_INIT_BUG_FIX // for client specific bug - SPI init failure
	#if 0
	{
	    int err = 0;
		spi->bits_per_word = 8;
		//ret = mt_spi_setup(spi);
		err = spi_setup(spi);
		if (err) {
			dev_dbg(&spi->dev, "spi master doesn't support 8 bits/word\n");
			return err;
		}
	}
	#endif
		byd_mtk_spi_init = 1;
  } // end of if byd_mtk_spi_init == 0
#endif

	return 0;
}

#endif // def PLATFORM_MTK_KERNEL3_10

/******************************************************************************/
#ifdef PLATFORM_QUALCOMM // msm8909/msm8937
/******************************************************************************/

#include <linux/of_gpio.h>

/*
 * struct byd_fps_platform_data_t - platform specific data
 * @reset_gpio:	gpio number for reset
 * @irq_gpio:	gpio number for interrupt
 *
 * Platform specific member include Qualcomm's data
 *
 */
struct byd_fps_platform_data_t
{
	unsigned int irq_gpio;
	const char * name;
	unsigned int max_freq;
};

static struct byd_fps_platform_data_t *byd_fps_platform_data = NULL;

static int byd_fps_parse_dt(struct device *dev, struct byd_fps_platform_data_t *fp_data);
static int byd_fps_init_port(struct device *dev, int on);


/**
 * byd_fps_platform_exit() - clean up for platform specific configuration
 * @dev:	pointer to device
 *
 * Return: status
 */
int byd_fps_platform_exit(struct device *dev)
{
#ifdef BYD_FPS_EINT_PORT
	if(gpio_is_valid(BYD_FPS_EINT_PORT))
		gpio_free(BYD_FPS_EINT_PORT);
#else
	if (byd_fps_platform_data) {
		if(gpio_is_valid(byd_fps_platform_data->irq_gpio))
			gpio_free(byd_fps_platform_data->irq_gpio);

		devm_kfree(dev, byd_fps_platform_data);
	}
#endif
	return 0;
}

/**
 * byd_fps_platform_init() - platform specific configuration
 * @spi:	spi device
 *
 *  This function is usually used for interrupt pin or SPI pin configration
 * during initialization of FPS driver.
 * Return: status
 */
int byd_fps_platform_init(struct spi_device *spi)
{
	int ret = 0;

#ifdef BYD_FPS_IRQ_NUM
	spi->irq = BYD_FPS_IRQ_NUM;
	FP_DBG("%s: BYD_FPS_IRQ_NUM is defined as %d\n", __func__, BYD_FPS_IRQ_NUM);
#else
	byd_fps_platform_data = devm_kzalloc(&spi->dev, sizeof(*byd_fps_platform_data), GFP_KERNEL);
	if (!byd_fps_platform_data)	{
		dev_err(&spi->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	if (spi->irq < 0) {
		int irq;

	  #ifdef BYD_FPS_EINT_PORT
		byd_fps_platform_data->irq_gpio = BYD_FPS_EINT_PORT;
	  #else
		ret = byd_fps_parse_dt(&spi->dev, byd_fps_platform_data);
		if (ret)
			goto err;

		FP_DBG("%s: irq_gpio:%d \n", __func__, byd_fps_platform_data->irq_gpio);
	  #endif
		ret = byd_fps_init_port(&spi->dev, true);
		if (ret)
			goto err;

		irq = gpio_to_irq(byd_fps_platform_data->irq_gpio);
		if (irq < 0) {
			pr_err("%s: gpio_to_irq() failed:%d, Port:%d\n", __func__, irq, byd_fps_platform_data->irq_gpio);
			gpio_free(byd_fps_platform_data->irq_gpio);
			ret = irq;
			goto err;
		}

		spi->irq = irq;
		FP_DBG("%s: gpio_to_irq(%d):%d!\n", __func__, byd_fps_platform_data->irq_gpio, irq);
		//msleep(500);
	}
	return ret;
err:
	devm_kfree(&spi->dev, byd_fps_platform_data);
	return ret;
#endif
	return ret;
}

/*
 * byd_fps_parse_dt() - get platform meta data from DTS configuration.
 * @dev:	spi device
 * @fp_data:	platform data
 *
 * refer to: arch/arm/boot/dts/qcom/msm8937-qrd.dtsi
 *
 * Return      :  status
 */
static int byd_fps_parse_dt(struct device *dev, struct byd_fps_platform_data_t *fp_data)
{
	struct device_node *np = dev->of_node;
	u32 gpio_flag;
	/*int rc;

	fp_data->name = "fp_bf6600";
	rc = of_property_read_string(np, "fingerprint,name", &fp_data->name);
	if (rc && (rc != -EINVAL)) {
		dev_err(dev, "%s: Unable to read name\n", __func__);
		return rc;
	}
	FP_DBG("%s: name:%s \n", __func__, fp_data->name);

	// get spi max speed
	rc = of_property_read_u32(np, "fingerprint,spi-max-frequency", &fp_data->max_freq);
	FP_DBG("%s: frequency:%d \n", __func__, fp_data->max_freq);
	*/

	/* get irq gpio*/
	fp_data->irq_gpio = of_get_named_gpio_flags(np, "fingerprint,irq-gpio",
												0,
												&gpio_flag);
	if (fp_data->irq_gpio < 0) {
		dev_err(dev, "%s: Unable to read irq_gpio\n", __func__);
		return fp_data->irq_gpio;
	}
	FP_DBG("%s: irq_gpio:%d \n", __func__, fp_data->irq_gpio);

	return 0;
}

/*
 * byd_fps_init_port() - interrupt pin (and sometimes SPI pin) configration
 * @dev:	spi device
 * @on:	gpio request or release
 *
 * Return      :  status
 */
static int byd_fps_init_port(struct device *dev, int on)
{
	int ret = 0;
	struct byd_fps_platform_data_t * data = byd_fps_platform_data;

	FP_DBG("%s(%d): entered \n", __func__, on);

	if (on)	{
		if (!gpio_is_valid(data->irq_gpio))
			return -1;

		FP_DBG("%s: irq_gpio:%d \n", __func__ , data->irq_gpio);
		ret = gpio_request(data->irq_gpio, "bf66xx_irq_gpio");
		if (ret) {
			dev_err(dev, "%s: irq port, gpio_request() failed:%d \n", __func__, ret);
			return ret;
		}

		ret = gpio_direction_input(data->irq_gpio);
		if (ret) {
			dev_err(dev, "%s: irq port, gpio_direction_input() failed:%d \n", __func__, ret);
			gpio_free(data->irq_gpio);
			return ret;
		}
	} else {
		if (gpio_is_valid(data->irq_gpio))
			gpio_free(data->irq_gpio);
	}

	return 0;
}

#endif // def PLATFORM_QUALCOMM
