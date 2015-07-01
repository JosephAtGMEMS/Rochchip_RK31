/*
 * Copyright (C) 2013 ROCKCHIP, Inc.
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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/skbuff.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/ion.h>
#include <linux/cpufreq.h>
#include <linux/clk.h>
#include <linux/rk_fb.h>
#include <linux/regulator/machine.h>
#include <linux/rfkill-rk.h>
#include <linux/sensor-dev.h>
#include <linux/mfd/tps65910.h>
#include <linux/regulator/act8846.h>
#include <linux/regulator/act8931.h>
#include <linux/regulator/rk29-pwm-regulator.h>

#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/dvfs.h>
#include <mach/board.h>
#include <mach/hardware.h>
#include <mach/io.h>
#include <mach/gpio.h>
#include <mach/iomux.h>

#include <plat/board.h>

#include <plat/efuse.h>

#ifdef CONFIG_MACH_RK3026_86V_WIFI_ESP8089
#include <linux/esp_ext_gpio.h>
#endif
#if defined(CONFIG_SPIM_RK29)
#include "../../../drivers/spi/rk29_spim.h"
#endif

#ifdef CONFIG_SND_SOC_RK3026
#include "../../../sound/soc/codecs/rk3026_codec.h"
#endif

#if defined(CONFIG_RK_HDMI)
        #include "../../../drivers/video/rockchip/hdmi/rk_hdmi.h"
#endif

#if defined(CONFIG_GPS_RK)
#include "../../../drivers/misc/gps/rk_gps/rk_gps.h"
#endif
#include "board-rk3026-86v-camera.c"
/***********************************************************
*	board config
************************************************************/
//system power on
#ifndef CONFIG_PMIC_AXP192
#define POWER_ON_PIN		RK30_PIN1_PA2   //PWR_HOLD
#endif

//touchscreen
#ifdef CONFIG_MACH_RK3026_PMU_ACT9831
#define TOUCH_RST_PIN		INVALID_GPIO
#define TOUCH_RST_VALUE		GPIO_HIGH
#define TOUCH_INT_PIN		RK30_PIN1_PA1
#elif defined(CONFIG_PMIC_AXP192)
#define TOUCH_RST_PIN		INVALID_GPIO
#define TOUCH_RST_VALUE		GPIO_HIGH
#define TOUCH_INT_PIN		RK30_PIN2_PB2
#else
#define TOUCH_RST_PIN		RK30_PIN2_PB2
#define TOUCH_RST_VALUE		GPIO_HIGH
//#define TOUCH_PWR_PIN		RK2928_PIN2_PB3
//#define TOUCH_PWR_VALUE		GPIO_LOW
#define TOUCH_INT_PIN		RK30_PIN1_PB0
#endif
//backlight
#define LCD_DISP_ON_PIN
#define BL_PWM			0  // (0 ~ 2)
#define PWM_EFFECT_VALUE  	0
//#define PWM_MUX_NAME      	GPIO0D2_PWM_0_NAME
//#define PWM_MUX_MODE     	GPIO0D_PWM_0
//#define PWM_MUX_MODE_GPIO	GPIO0D_GPIO0D2
//#define PWM_GPIO 	 	RK2928_PIN0_PD2


#define BL_EN_PIN		RK2928_PIN3_PC1
#define BL_EN_VALUE		GPIO_HIGH
#define BL_EN_MUX_NAME  	GPIO3C1_OTG_DRVVBUS_NAME
#define BL_EN_MUX_MODE   	GPIO3C_GPIO3C1


//fb
#if defined(CONFIG_MACH_RK3026_YK_k71)
#define LCD_EN_PIN		RK2928_PIN0_PA1
#else
#define LCD_EN_PIN		RK2928_PIN1_PB3
#endif
#ifdef	CONFIG_MACH_RK3026_86V_NOPMU
#define LCD_EN_VALUE		GPIO_HIGH
#else
#define LCD_EN_VALUE		GPIO_LOW
#endif
#define LCD_CS_PIN		INVALID_GPIO
#define LCD_CS_VALUE		GPIO_HIGH

#ifdef CONFIG_MACH_RK3026_86V_SCREEN_785_ANTI_SHAKE
#define LCD_VGL_VGH_PIN RK30_PIN2_PC2
#define LCD_STANDBY_PIN RK30_PIN2_PC3
#define LCD_RST_PIN	RK30_PIN2_PB7
#endif
//gsensor
#ifdef CONFIG_MACH_RK3026_PMU_ACT9831
#define GS_INT_PIN		RK2928_PIN2_PB2
#else
#define GS_INT_PIN		RK2928_PIN1_PB2
#endif
#ifdef CONFIG_MACH_RK3026_PMU_ACT9831
#define CHARING_CURRENT_500MA 0
#define CHARING_CURRENT_1000MA 1
#define PC_MODE 0
#define ADAPT_MODE 1
#define   DC_CUR_SET_PIN   RK30_PIN3_PB3
static int ac_current = CHARING_CURRENT_1000MA;
#endif

//sdmmc
//Reference to board-rk3026-tb-sdmmc-config.c

//keyboard
//#define RK31XX_MAINBOARD_V1      //if mainboard is RK31XX_MAINBOARD_V1.0
#define PLAY_ON_PIN		RK30_PIN1_PA4	//wakeup key		

//pwm vcom
#define REG_PWM_VCOM			1  // (0 ~ 2)
//pwm regulator
#ifdef CONFIG_PWM_LOGIC_WITH_ARM
#define REG_PWM_LOGIC			PWM_NULL // (0 ~ 2)
#define REG_PWM_ARM			1 // (0 ~ 2)
#else
#define REG_PWM_LOGIC			1 // (0 ~ 2)
#define REG_PWM_ARM			0 // (0 ~ 2)
#endif

//pmic
#define PMU_INT_PIN		RK30_PIN1_PB1
#ifdef CONFIG_MACH_RK3026_PMU_ACT9831
#define PMU_SLEEP_PIN		INVALID_GPIO //RK30_PIN1_PA1
#else
#define PMU_SLEEP_PIN		RK30_PIN1_PA1
#endif

//ion reserve memory
#define ION_RESERVE_SIZE        (80 * SZ_1M)
#ifdef CONFIG_MACH_RK3026_86V_WIFI_ESP8089
static struct esp_ext_gpio_ops *ext_gpio_ops = NULL;
void register_ext_gpio_ops(struct esp_ext_gpio_ops *ops)
{
	if (ops)
		ext_gpio_ops = ops;
}
EXPORT_SYMBOL(register_ext_gpio_ops);
void unregister_ext_gpio_ops(void)
{
	ext_gpio_ops = NULL;
}
EXPORT_SYMBOL(unregister_ext_gpio_ops);
int rk3026_esp_gpio_request(int gpio_no)
{
	if (ext_gpio_ops && ext_gpio_ops->gpio_request)
		return ext_gpio_ops->gpio_request(gpio_no);
	return -EINVAL;
}
EXPORT_SYMBOL(rk3026_esp_gpio_request);
int rk3026_esp_gpio_release(int gpio_no)
{
	if (ext_gpio_ops && ext_gpio_ops->gpio_release)
		return ext_gpio_ops->gpio_release(gpio_no);
	return -EINVAL;
}
EXPORT_SYMBOL(rk3026_esp_gpio_release);
int rk3026_esp_gpio_set_mode(int gpio_no, int mode, void *data)
{
	if (ext_gpio_ops && ext_gpio_ops->gpio_set_mode)
		return ext_gpio_ops->gpio_set_mode(gpio_no, mode, data);
	return -EINVAL;
}
EXPORT_SYMBOL(rk3026_esp_gpio_set_mode);
int rk3026_esp_gpio_get_mode(int gpio_no)
{
	if (ext_gpio_ops && ext_gpio_ops->gpio_get_mode)
		return ext_gpio_ops->gpio_get_mode(gpio_no);
	return -EINVAL;
}
EXPORT_SYMBOL(rk3026_esp_gpio_get_mode);
int rk3026_esp_gpio_set_state(int gpio_no, int state)
{
	if (ext_gpio_ops && ext_gpio_ops->gpio_set_state)
		return ext_gpio_ops->gpio_set_state(gpio_no, state);
	return -EINVAL;
}
EXPORT_SYMBOL(rk3026_esp_gpio_set_state);
int rk3026_esp_gpio_get_state(int gpio_no)
{
	if (ext_gpio_ops && ext_gpio_ops->gpio_get_state)
		return ext_gpio_ops->gpio_get_state(gpio_no);
	return -EINVAL;
}
EXPORT_SYMBOL(rk3026_esp_gpio_get_state);
int rk3026_esp_irq_ack(int gpio_no)
{
	if (ext_gpio_ops && ext_gpio_ops->irq_ack)
		return ext_gpio_ops->irq_ack(gpio_no);
	return -EINVAL;
}
EXPORT_SYMBOL(rk3026_esp_irq_ack);
#endif
static int pwm_mode[] = {PWM0, PWM1, PWM2};
static inline int rk_gpio_request(int gpio, int direction, int value, const char *label)
{
	int ret = 0;
	unsigned long flags = 0;

	if(!gpio_is_valid(gpio))
		return 0;

	if(direction == GPIOF_DIR_IN)
		flags = GPIOF_IN;
	else if(value == GPIO_LOW)
		flags = GPIOF_OUT_INIT_LOW;
	else
		flags = GPIOF_OUT_INIT_HIGH;

	ret = gpio_request_one(gpio, flags, label);
	if(ret < 0)
		pr_err("Failed to request '%s'\n", label);

	return ret;
}

static struct spi_board_info board_spi_devices[] = {
};

/***********************************************************
*	touchscreen
************************************************************/
#if defined(CONFIG_TOUCHSCREEN_GSLX680)
//#define TOUCH_RESET_PIN RK30_PIN0_PC1
//#define TOUCH_EN_PIN NULL
//#define TOUCH_INT_PIN RK30_PIN0_PB4
	
	int gslx680_init_platform_hw(void)
	{
	#if (TOUCH_RST_PIN != INVALID_GPIO)
       if(gpio_request(TOUCH_RST_PIN,NULL) != 0){
			gpio_free(TOUCH_RST_PIN);
			printk("gslx680_init_platform_hw gpio_request error\n");
			return -EIO;
		}
	 #endif      
		if(gpio_request(TOUCH_INT_PIN,NULL) != 0){
			gpio_free(TOUCH_INT_PIN);
			printk("gslx680_init_platform_hw  gpio_request error\n");
			return -EIO;
		}
	#if (TOUCH_RST_PIN != INVALID_GPIO)	
		gpio_direction_output(TOUCH_RST_PIN, TOUCH_RST_VALUE);
		mdelay(10);
		gpio_set_value(TOUCH_RST_PIN,!TOUCH_RST_VALUE);
		mdelay(10);
		gpio_set_value(TOUCH_RST_PIN,TOUCH_RST_VALUE);
		msleep(300);
	#endif	
		return 0;
	
	}
	
	struct ts_hw_data     gslx680_info = {
		#if(TOUCH_RST_PIN != INVALID_GPIO)
		.reset_gpio = TOUCH_RST_PIN,
		#else
		//.reset_gpio = TOUCH_RST_PIN,
		#endif
		.touch_en_gpio = TOUCH_INT_PIN,
		.init_platform_hw = gslx680_init_platform_hw,
	};
#endif

#if defined(CONFIG_TOUCHSCREEN_ZET62XX)
#define TOUCH_RESET_PIN 	RK30_PIN2_PB2
#define TOUCH_INT_PIN 	        RK30_PIN1_PB0
int zet6221_init_platform_hw(void)
{
    if(gpio_request(TOUCH_RESET_PIN,NULL) != 0){
      gpio_free(TOUCH_RESET_PIN);
      printk("zet6221_init_platform_hw gpio_request error\n");
      return -EIO;
    }

    if(gpio_request(TOUCH_INT_PIN,NULL) != 0){
      gpio_free(TOUCH_INT_PIN);
      printk("zet6221_init_platform_hw gpio_request error\n");
      return -EIO;
    }
    //gpio_pull_updown(TOUCH_INT_PIN, 1);
    gpio_direction_output(TOUCH_RESET_PIN, 0);
    msleep(500);
    gpio_set_value(TOUCH_RESET_PIN,GPIO_LOW);
    msleep(500);
    gpio_set_value(TOUCH_RESET_PIN,GPIO_HIGH);
	mdelay(100);

    return 0;
}

struct ts_hw_data     zet6221_info = {
	.reset_gpio = TOUCH_RESET_PIN,
	.touch_en_gpio = TOUCH_INT_PIN,
  	.init_platform_hw= zet6221_init_platform_hw,
};
#endif

#if defined(CONFIG_TOUCHSCREEN_GT8XX)
static int goodix_init_platform_hw(void)
{
	int ret  = 0;

	ret = rk_gpio_request(TOUCH_PWR_PIN, GPIOF_DIR_OUT, TOUCH_PWR_VALUE, "touch_pwr");
	if(ret < 0)
		return ret; 
	msleep(100);

	ret = rk_gpio_request(TOUCH_RST_PIN, GPIOF_DIR_OUT, TOUCH_RST_VALUE, "touch_rst");
	if(ret < 0)
		return ret; 
	msleep(100);

	return 0;
}

struct goodix_platform_data goodix_info = {
	.model = 8105,
	.irq_pin = TOUCH_INT_PIN,
	.rest_pin = TOUCH_RST_PIN,
	.init_platform_hw = goodix_init_platform_hw,
};
#endif

/***********************************************************
*	rk30  backlight
************************************************************/
#ifdef CONFIG_BACKLIGHT_RK29_BL
static int rk29_backlight_io_init(void)
{
	int ret = 0;

	iomux_set(pwm_mode[BL_PWM]);
	//ret = rk_gpio_request(RK30_PIN0_PD2, GPIOF_DIR_OUT, GPIO_LOW, "PWM");
	msleep(50);
#ifdef  LCD_DISP_ON_PIN
	ret = rk_gpio_request(BL_EN_PIN, GPIOF_DIR_OUT, BL_EN_VALUE, "bl_en");
	if(ret < 0)
		return ret;
#endif
#ifdef CONFIG_MACH_RK3026_86V_CAMERA_FLASH
	gpio_direction_output(RK30_PIN0_PD3, 0);
	gpio_set_value(RK30_PIN0_PD3, 0);
#endif
#ifdef CONFIG_MACH_RK3026_86V_CAMERA_FLASH_USE_I2C0
	gpio_direction_output(RK30_PIN0_PA0, 0);
	gpio_set_value(RK30_PIN0_PA0, 0);
	gpio_direction_output(RK30_PIN0_PA1, 0);
	gpio_set_value(RK30_PIN0_PA1, 0);
#endif
	return 0;
}

static int rk29_backlight_io_deinit(void)
{
	int pwm_gpio;
#ifdef  LCD_DISP_ON_PIN
	gpio_free(BL_EN_PIN);
#endif
	pwm_gpio = iomux_mode_to_gpio(pwm_mode[BL_PWM]);
	return rk_gpio_request(BL_EN_PIN, GPIOF_DIR_OUT, GPIO_LOW, "BL_PWM");
//	printk("####liufeng####:%s[%d]\n",__func__,__LINE__);
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret, pwm_gpio = iomux_mode_to_gpio(pwm_mode[BL_PWM]);

	ret = rk_gpio_request(pwm_gpio, GPIOF_DIR_OUT, GPIO_LOW, "BL_PWM");
	if(ret < 0)
		return ret;
#ifdef  LCD_DISP_ON_PIN
	gpio_direction_output(BL_EN_PIN, !BL_EN_VALUE);
#endif
#if !defined(CONFIG_MACH_RK3026_86V_NOPMU) && !defined(CONFIG_MACH_RK3026_86V_CAMERA_FLASH)
	msleep(30);
	pwm_gpio = iomux_mode_to_gpio(pwm_mode[1]);
	ret = rk_gpio_request(pwm_gpio, GPIOF_DIR_OUT, GPIO_LOW, "VCOM_PWM");
#endif
	printk("####liufeng####:%s[%d]\n",__func__,__LINE__);
	return ret;
}

static int rk29_backlight_pwm_resume(void)
{
	int pwm_gpio = iomux_mode_to_gpio(pwm_mode[BL_PWM]);
#if !defined(CONFIG_MACH_RK3026_86V_NOPMU) && !defined(CONFIG_MACH_RK3026_86V_CAMERA_FLASH)
	int pwm_gpio_vcom = iomux_mode_to_gpio(pwm_mode[1]);
#endif
	gpio_free(pwm_gpio);
#if !defined(CONFIG_MACH_RK3026_86V_NOPMU) && !defined(CONFIG_MACH_RK3026_86V_CAMERA_FLASH)
	gpio_free(pwm_gpio_vcom);
#endif
	printk("####liufeng####:%s[%d],\n",__func__,__LINE__);
#if !defined(CONFIG_MACH_RK3026_86V_NOPMU) && !defined(CONFIG_MACH_RK3026_86V_CAMERA_FLASH)
	iomux_set(pwm_mode[1]);
	msleep(500);//30);
#endif
	iomux_set(pwm_mode[BL_PWM]);
#ifdef  LCD_DISP_ON_PIN
	msleep(30);
	gpio_direction_output(BL_EN_PIN, BL_EN_VALUE);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif
	return 0;
}

static struct rk29_bl_info rk29_bl_info = {
	.pwm_id = BL_PWM,
	.min_brightness=80,
	.max_brightness=255,
	.brightness_mode = BRIGHTNESS_MODE_CONIC,
	.bl_ref = PWM_EFFECT_VALUE,
	.io_init = rk29_backlight_io_init,
	.io_deinit = rk29_backlight_io_deinit,
	.pwm_suspend = rk29_backlight_pwm_suspend,
	.pwm_resume = rk29_backlight_pwm_resume,
	.pre_div = 10000,
#ifdef CONFIG_MACH_RK3026_86V_SCREEN_785_ANTI_SHAKE
	.delay_ms = 150,
#endif
};

static struct platform_device rk29_device_backlight = {
	.name	= "rk29_backlight",
	.id 	= -1,
	.dev	= {
		.platform_data  = &rk29_bl_info,
	}
};
#endif

/***********************************************************
*	fb
************************************************************/
#ifdef CONFIG_MACH_RK3026_86V_SCREEN_785_ANTI_SHAKE
void gll_close_screen(void)
{
	printk("[gll] rk30_pm_power_off\n");
	rk29_backlight_pwm_suspend();
	mdelay(500);
	gpio_set_value(LCD_EN_PIN, !LCD_EN_VALUE);
}
#endif
#ifdef CONFIG_FB_ROCKCHIP
static int rk_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
	int ret = 0;
#ifdef CONFIG_MACH_RK3026_PMU_ACT9831
	if(INVALID_GPIO!= DC_CUR_SET_PIN)
	{
	    if (gpio_request(DC_CUR_SET_PIN, NULL) != 0){
	            gpio_free(DC_CUR_SET_PIN);
	            printk(KERN_ERR "DC_CUR_SET_PIN gpio_request error\n");
	    }
	gpio_direction_output(DC_CUR_SET_PIN, GPIO_HIGH);
    gpio_set_value(DC_CUR_SET_PIN, GPIO_HIGH);
	}
    ac_current = CHARING_CURRENT_1000MA;
#endif
#ifdef CONFIG_MACH_RK3026_86V_WIFI_ESP8089
	{
	    if (gpio_request(RK30_PIN1_PA0, NULL) != 0){
	            gpio_free(RK30_PIN1_PA0);
	            printk(KERN_ERR "DC_CUR_SET_PIN gpio_request error\n");
	    }
	gpio_direction_output(RK30_PIN1_PA0, GPIO_LOW);
	}
#endif
	if(INVALID_GPIO!= LCD_CS_PIN)
	{
	ret = rk_gpio_request(LCD_CS_PIN, GPIOF_DIR_OUT, LCD_CS_VALUE, "lcd_cs");
	if(ret < 0)
		return ret;
	}

//	printk("####liufeng####:%s[%d]\n",__func__,__LINE__);
#ifdef CONFIG_MACH_RK3026_86V_SCREEN_785_ANTI_SHAKE
        if (gpio_request(LCD_EN_PIN, NULL) != 0){
                gpio_free(LCD_EN_PIN);
                printk(KERN_ERR "LCD EN PIN gpio_request error\n");
                return -EIO;
	    }

	if (gpio_request(LCD_RST_PIN, NULL) != 0){
                gpio_free(LCD_RST_PIN);
                printk("LCD RST PIN gpio_request error\n");
                return -EIO;
        }
        if (gpio_request(LCD_STANDBY_PIN, NULL) != 0){
                gpio_free(LCD_STANDBY_PIN);
                printk(KERN_ERR "lcd standby gpio_request error\n");
                return -EIO;
        }
	if (gpio_request(LCD_VGL_VGH_PIN, NULL) != 0){
                gpio_free(LCD_VGL_VGH_PIN);
                printk("lcd VGL VGH gpio_request error\n");
                return -EIO;
        }
	gpio_direction_output(LCD_EN_PIN,LCD_EN_VALUE);
	gpio_direction_output(LCD_RST_PIN,GPIO_LOW);
	gpio_direction_output(LCD_STANDBY_PIN,GPIO_LOW);
	gpio_direction_output(LCD_VGL_VGH_PIN,GPIO_LOW);
	mdelay(10);
	gpio_set_value(LCD_RST_PIN,GPIO_HIGH);
	mdelay(8);
	gpio_set_value(LCD_STANDBY_PIN, 1);
	mdelay(25);
        gpio_set_value(LCD_VGL_VGH_PIN, 1);
	return 0;
#else
	return rk_gpio_request(LCD_EN_PIN, GPIOF_DIR_OUT, LCD_EN_VALUE, "lcd_en");
#endif
}

static int rk_fb_io_disable(void)
{
#ifdef CONFIG_MACH_RK3026_86V_SCREEN_785_ANTI_SHAKE
	gll_close_screen();
#endif
#ifndef CONFIG_MACH_RK3026_86V_SCREEN_785_ANTI_SHAKE
	gpio_set_value(LCD_CS_PIN, !LCD_CS_VALUE);
	gpio_set_value(LCD_EN_PIN, !LCD_EN_VALUE);
#endif

	return 0;
}

static int rk_fb_io_enable(void)
{
#ifndef CONFIG_MACH_RK3026_86V_SCREEN_785_ANTI_SHAKE
	gpio_set_value(LCD_CS_PIN, LCD_CS_VALUE);
	gpio_set_value(LCD_EN_PIN, LCD_EN_VALUE);
#endif
	return 0;
}

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
struct rk29fb_info lcdc0_screen_info = {
#if defined(CONFIG_RK_HDMI) && defined(CONFIG_HDMI_SOURCE_LCDC0) && defined(CONFIG_DUAL_LCDC_DUAL_DISP_IN_KERNEL)
	.prop	   = EXTEND,	//extend display device
	.io_init    = NULL,
	.io_disable = NULL,
	.io_enable = NULL,
	.set_screen_info = hdmi_init_lcdc,
#else
	.prop	   = PRMRY,		//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
#endif
};
#endif

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
struct rk29fb_info lcdc1_screen_info = {
#if defined(CONFIG_RK_HDMI) && defined(CONFIG_HDMI_SOURCE_LCDC1) && defined(CONFIG_DUAL_LCDC_DUAL_DISP_IN_KERNEL)
	.prop	   = EXTEND,	//extend display device
	.io_init    = NULL,
	.io_disable = NULL,
	.io_enable = NULL,
	.set_screen_info = hdmi_init_lcdc,
#else
	.prop	   = PRMRY,		//primary display device
	.io_init   = rk_fb_io_init,
	.io_disable = rk_fb_io_disable,
	.io_enable = rk_fb_io_enable,
	.set_screen_info = set_lcd_info,
#endif
};
#endif

static struct resource resource_fb[] = {
	[0] = {
		.name  = "fb0 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "ipp buf",  //for rotate
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[2] = {
		.name  = "fb2 buf",
		.start = 0,
		.end   = 0,//RK30_FB0_MEM_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
};

static struct platform_device device_fb = {
	.name		= "rk-fb",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(resource_fb),
	.resource	= resource_fb,
};
#endif

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
static struct resource resource_lcdc0[] = {
	[0] = {
		.name  = "lcdc0 reg",
		.start = RK3026_LCDC0_PHYS,
		.end   = RK3026_LCDC0_PHYS + RK3026_LCDC0_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	
	[1] = {
		.name  = "lcdc0 irq",
		.start = IRQ_LCDC,
		.end   = IRQ_LCDC,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc0 = {
	.name		  = "rk30-lcdc",
	.id		  = 0,
	.num_resources	  = ARRAY_SIZE(resource_lcdc0),
	.resource	  = resource_lcdc0,
	.dev 		= {
		.platform_data = &lcdc0_screen_info,
	},
};
#endif

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
static struct resource resource_lcdc1[] = {
	[0] = {
		.name  = "lcdc1 reg",
		.start = RK3026_LCDC1_PHYS,
		.end   = RK3026_LCDC1_PHYS + RK3026_LCDC1_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name  = "lcdc1 irq",
		.start = IRQ_LCDC1,
		.end   = IRQ_LCDC1,
		.flags = IORESOURCE_IRQ,
	},
};

static struct platform_device device_lcdc1 = {
	.name		  = "rk30-lcdc",
	.id		  = 1,
	.num_resources	  = ARRAY_SIZE(resource_lcdc1),
	.resource	  = resource_lcdc1,
	.dev 		= {
		.platform_data = &lcdc1_screen_info,
	},
};
#endif

static int rk_platform_add_display_devices(void)
{
	struct platform_device *fb = NULL;  //fb
	struct platform_device *lcdc0 = NULL; //lcdc0
	struct platform_device *lcdc1 = NULL; //lcdc1
	struct platform_device *bl = NULL; //backlight
#ifdef CONFIG_FB_ROCKCHIP
	fb = &device_fb;
#endif

#if defined(CONFIG_LCDC0_RK3066B) || defined(CONFIG_LCDC0_RK3188)
	lcdc0 = &device_lcdc0,
#endif

#if defined(CONFIG_LCDC1_RK3066B) || defined(CONFIG_LCDC1_RK3188)
	lcdc1 = &device_lcdc1,
#endif

#ifdef CONFIG_BACKLIGHT_RK29_BL
	bl = &rk29_device_backlight,
#endif
	__rk_platform_add_display_devices(fb,lcdc0,lcdc1,bl);

	return 0;
}


/***********************************************************
*	gsensor
************************************************************/
// mma 8452 gsensor
#if defined (CONFIG_GS_MMA8452)
#define MMA8452_INT_PIN GS_INT_PIN
static int mma8452_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data mma8452_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma8452_init_platform_hw,
        .orientation = {-1, 0, 0, 0, -1, 0, 0, 0, 1},
};
#endif

/*MC3XXX gsensor*/
#if defined (CONFIG_GS_MC3XXX)
#define MC3XXX_INT_PIN   GS_INT_PIN

static int MC3XXX_init_platform_hw(void)
{
	/*
	if(gpio_request(MC3XXX_INT_PIN,NULL) != 0){
		gpio_free(MC3XXX_INT_PIN);
		printk("MC3XXX_init_platform_hw gpio_request error\n");
		return -EIO;
	}
	gpio_pull_updown(MC3XXX_INT_PIN, 1);
	*/
	return 0;
}
static struct sensor_platform_data MC3XXX_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 0,
	.poll_delay_ms = 30,
        .init_platform_hw = MC3XXX_init_platform_hw,
        #if defined(CONFIG_MACH_RK3026_YX706) || defined(CONFIG_MACH_RK3026_YKSC3068)
        .orientation = {0, -1, 0, 1, 0, 0, 0, 0, 1},
	#elif defined(CONFIG_MACH_RK3026_86V_SCREEN_785_ANTI_SHAKE)
			#ifdef CONFIG_TOUCHSCREEN_GSL3670_785_768x1024_807
			        .orientation = {0, 1, 0, -1, 0, 0, 0, 0, 1},
			#elif defined(CONFIG_LOGO_LINUX_768x1024_VERTICAL_CLUT224)
				.orientation = {-1, 0, 0, 0, -1, 0, 0, 0, 1},
			#else
			        .orientation = {0, -1, 0, 1, 0, 0, 0, 0, 1},
			#endif
        #else
        .orientation = {1, 0, 0, 0, 1, 0, 0, 0, 1},
		#endif
};

#endif

/*GMA302gsensor*/
#if defined (CONFIG_GS_GMA302)
#define GMA302_INT_PIN   GS_INT_PIN
static int GMA302_init_platform_hw(void){
       return 0;
}
static struct sensor_platform_data GMA302_info = {
       .type = SENSOR_TYPE_ACCEL,
       .irq_enable = 0,
       .poll_delay_ms = 30,
        .init_platform_hw = GMA302_init_platform_hw,
        .orientation = {0, 1, 0, -1, 0, 0, 0, 0, 1},
};
#endif
/*GMA303gsensor*/
#if defined (CONFIG_GS_GMA303)
#define GMA303_INT_PIN   GS_INT_PIN
static int GMA303_init_platform_hw(void){
       return 0;
}
static struct sensor_platform_data GMA303_info = {
       .type = SENSOR_TYPE_ACCEL,
       .irq_enable = 0,
       .poll_delay_ms = 30,
        .init_platform_hw = GMA303_init_platform_hw,
        .orientation = {0, 1, 0, -1, 0, 0, 0, 0, 1},
};
#endif


#if (defined(CONFIG_GS_STK8312) || defined(CONFIG_GS_STK8313))
    #define STK831X_INT_PIN   GS_INT_PIN
    static int stk831x_init_platform_hw(void){
       return 0;
    }
    static struct sensor_platform_data stk831x_info = {
      .type = SENSOR_TYPE_ACCEL,
      .irq_enable = 0,
      .poll_delay_ms = 30,
      .init_platform_hw = stk831x_init_platform_hw,
      #if defined(CONFIG_MACH_RK3026_YKSC3068)
	.orientation = {0, 1, 0, 1, 0, 0, 0, 0, -1},
	#elif defined(CONFIG_MACH_RK3026_LVDS_SUNSAM_SCREEN)
	.orientation = {-1, 0, 0, 0, 1, 0, 0, 0, -1},
	#elif defined(CONFIG_MACH_RK3026_86V_SCREEN_785_ANTI_SHAKE)
	#if defined(CONFIG_LOGO_LINUX_768x1024_VERTICAL_CLUT224)
		.orientation = {1, 0, 0, 0, -1, 0, 0, 0, -1},
	#else
		.orientation = {0, 1, 0, 1, 0, 0, 0, 0, -1},
	#endif
      #else
	.orientation = {-1, 0, 0, 0, 1, 0, 0, 0, -1},
      #endif
    }; 
 #endif
#if defined (CONFIG_GS_MIR3DA)
static int mir3da_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data mir3da_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 0,
	.poll_delay_ms = 30,
        .init_platform_hw = mir3da_init_platform_hw,
        .orientation = {1, 0, 0, 0, 1, 0, 0, 0, 1},
};
#endif

// lsm303d gsensor
#if defined (CONFIG_GS_LSM303D)
#define LSM303D_INT_PIN GS_INT_PIN
static int lms303d_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data lms303d_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = lms303d_init_platform_hw,
        .orientation = {-1, 0, 0, 0, -1, 0, 0, 0, 1},
};
#endif


/*MMA7660 gsensor*/
#if defined (CONFIG_GS_MMA7660)
#define MMA7660_INT_PIN   GS_INT_PIN

static int mma7660_init_platform_hw(void)
{
	//rk30_mux_api_set(GPIO1B2_SPI_RXD_UART1_SIN_NAME, GPIO1B_GPIO1B2);

	return 0;
}

static struct sensor_platform_data mma7660_info = {
	.type = SENSOR_TYPE_ACCEL,
	.irq_enable = 1,
	.poll_delay_ms = 30,
        .init_platform_hw = mma7660_init_platform_hw,
	.orientation = {0, 1, 0, 1, 0, 0, 0, 0, -1},
};
#endif

#if defined (CONFIG_GS_D10)
#define D10_INT_PIN   GS_INT_PIN
static int d10_init_platform_hw(void)
{
       return 0;
}
static struct sensor_platform_data d10_info = {
        .type = SENSOR_TYPE_ACCEL,
        .irq_enable = 0,
        .poll_delay_ms = 30,
        .init_platform_hw = d10_init_platform_hw,
        .orientation = { 0, -1, 0, 1, 0, 0, 0, 0, 1},
};
#endif

#if defined (CONFIG_GS_MXC6225)
#define MXC6225_INT_PIN   GS_INT_PIN

static int mxc6225_init_platform_hw(void)
{
        return 0;
}

static struct sensor_platform_data mxc6225_info = {
        .type = SENSOR_TYPE_ACCEL,
        .irq_enable = 0,
        .poll_delay_ms = 30,
        .init_platform_hw = mxc6225_init_platform_hw,
		#if defined(CONFIG_MACH_RK3026_Q8_GSENSOR)
        .orientation = { -1, 0, 0, 0, -1, 0, 0, 0, 0},
                #elif defined(CONFIG_MACH_RK3026_YX706)
                .orientation = {0, 1, 0, -1, 0, 0, 0, 0, 0},
                #elif defined(CONFIG_MACH_RK3026_YX781B)
                .orientation = {-1, 0, 0, 0, -1, 0, 0, 0, 0},
                #elif defined(CONFIG_MACH_RK3026_YX781)
                .orientation = {1, 0, 0, 0, 1, 0, 0, 0, 0},
                #elif defined(CONFIG_MACH_RK3026_YC708S)
                .orientation = {0, -1, 0, 1, 0, 0, 0, 0, 0},
                #elif defined(CONFIG_LOGO_LINUX_768x1024_VERTICAL_CLUT224)
					.orientation = { 0, -1, 0, 1, 0, 0, 0, 0, 1},

		#else
			
		.orientation = { 1, 0, 0, 0, 1, 0, 0, 0, 1},
		#endif
};
#endif
#if defined (CONFIG_GS_LIS3DH)
#define LIS3DH_INT_PIN   GS_INT_PIN
	
	static int lis3dh_init_platform_hw(void)
	{
	
		return 0;
	}
	
	static struct sensor_platform_data lis3dh_info = {
		.type = SENSOR_TYPE_ACCEL,
		.irq_enable = 1,
		.poll_delay_ms = 30,
		.init_platform_hw = lis3dh_init_platform_hw,
		.orientation = {1, 0, 0, 0, 1, 0, 0, 0, 1},
	};
#endif

#if defined (CONFIG_COMPASS_AK8975)
static struct sensor_platform_data akm8975_info =
{
	.type = SENSOR_TYPE_COMPASS,
	.irq_enable = 1,
	.poll_delay_ms = 30,
	.m_layout = 
	{
		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},

		{
			{1, 0, 0},
			{0, 1, 0},
			{0, 0, 1},
		},
	}
};

#endif

#if defined (CONFIG_COMPASS_AK8963)
static struct sensor_platform_data akm8963_info =
{
       .type = SENSOR_TYPE_COMPASS,
       .irq_enable = 1,
       .poll_delay_ms = 30,
       .m_layout = 
       {
               {
                       {0, 1, 0},
                       {1, 0, 0},
                       {0, 0, -1},
               },

               {
                       {1, 0, 0},
                       {0, 1, 0},
                       {0, 0, 1},
               },

               {
                       {0, -1, 0},
                       {-1, 0, 0},
                       {0, 0, -1},
               },

               {
                       {1, 0, 0},
                       {0, 1, 0},
                       {0, 0, 1},
               },
       }
};

#endif


#if defined(CONFIG_GYRO_L3G4200D)

#include <linux/l3g4200d.h>
#define L3G4200D_INT_PIN  RK30_PIN0_PB4

static int l3g4200d_init_platform_hw(void)
{
	return 0;
}

static struct sensor_platform_data l3g4200d_info = {
	.type = SENSOR_TYPE_GYROSCOPE,
	.irq_enable = 1,
	.poll_delay_ms = 30,
	.orientation = {0, 1, 0, -1, 0, 0, 0, 0, 1},
	.init_platform_hw = l3g4200d_init_platform_hw,
	.x_min = 40,//x_min,y_min,z_min = (0-100) according to hardware
	.y_min = 40,
	.z_min = 20,
};

#endif

#ifdef CONFIG_LS_CM3217
static struct sensor_platform_data cm3217_info = {
	.type = SENSOR_TYPE_LIGHT,
	.irq_enable = 0,
	.poll_delay_ms = 500,
};

#endif

/***********************************************************
*	keyboard
************************************************************/
#include <plat/key.h>

static struct rk29_keys_button key_button[] = {
        {
                .desc   = "play",
                .code   = KEY_POWER,
                .gpio   = PLAY_ON_PIN,
                .active_low = PRESS_LEV_LOW,
                .wakeup = 1,
        },
/* disable adc keyboard,
 * because rk280a adc reference voltage is 3.3V, but
 * rk30xx mainbord key's supply voltage is 2.5V and
 * rk31xx mainbord key's supply voltage is 1.8V.
 */
 #if defined(CONFIG_MACH_RK3026_YX706) || defined(CONFIG_MACH_RK3026_YC708S)
	{
		.desc	= "vol+",
		.code	= KEY_VOLUMEUP,
		.gpio = INVALID_GPIO,
		.adc_value	= 512,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "vol-",
		.code	= KEY_VOLUMEDOWN,
		.gpio = INVALID_GPIO,
		.adc_value	= 1,
		.active_low = PRESS_LEV_LOW,
	},
#else
	{
		.desc	= "vol+",
		.code	= KEY_VOLUMEUP,
		.gpio = INVALID_GPIO,
		.adc_value	= 1,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "vol-",
		.code	= KEY_VOLUMEDOWN,
		.gpio = INVALID_GPIO,
		.adc_value	= 512,
		.active_low = PRESS_LEV_LOW,
	},
#endif
        #if defined(CONFIG_MACH_RK3026_YKSC3068)	
	{
		.desc   = "esc",
		.code   = KEY_BACK,
		.adc_value      = 742,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
#endif
	/*{
        .desc   = "menu",
        .code   = EV_MENU,
        .adc_value      = 133,
        .gpio = INVALID_GPIO,
        .active_low = PRESS_LEV_LOW,
    },
    {
         .code   = KEY_HOME,
         .adc_value      = 550,
         .gpio = INVALID_GPIO,
         .active_low = PRESS_LEV_LOW,
    },
    {
         .desc   = "esc",
         .code   = KEY_BACK,
         .adc_value      = 333,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},
	{
		.desc	= "camera",
		.code	= KEY_CAMERA,
		.adc_value	= 742,
		.gpio = INVALID_GPIO,
		.active_low = PRESS_LEV_LOW,
	},*/

};

struct rk29_keys_platform_data rk29_keys_pdata = {
	.buttons	= key_button,
	.nbuttons	= ARRAY_SIZE(key_button),
	.chn	= 3,  //chn: 0-7, if do not use ADC,set 'chn' -1
};
#ifdef CONFIG_PMIC_AXP192
	static int __boot_from_source = -1 ; // 1: power on ; 2 : dc
	static int store_boot_source(){
	    int cnt ;
	    for ( cnt  = 0 ;cnt  < rk29_keys_pdata.nbuttons; cnt ++) {
	            if( strcmp( rk29_keys_pdata.buttons[cnt].desc, "play") == 0 ){
	                    if ( (gpio_get_value( rk29_keys_pdata.buttons[cnt].gpio ) ^ 
	                         rk29_keys_pdata.buttons[cnt].active_low ) ){
	                            __boot_from_source = 1 ;
	                    }
	            }
	    }
	    if( __boot_from_source < 0 ){
	            __boot_from_source = 2 ;
	    }
	    printk("%s >>>> __boot_from_source is %d \n",__func__,__boot_from_source );
	    return __boot_from_source ;
	}
	int get_boot_source(){
	    return __boot_from_source ;
	}
#endif
/***********************************************************
*	usb wifi
************************************************************/
#if defined(CONFIG_RTL8192CU) || defined(CONFIG_RTL8188EU) || defined(CONFIG_RTL8192DU) || defined(CONFIG_RTL8723AU)

static void rkusb_wifi_power(int on) {
	int ret=0;
	struct regulator *ldo = NULL;
	printk("hjc:%s[%d],on=%d\n",__func__,__LINE__,on);

#if  defined(CONFIG_PWM_CONTROL_LOGIC) || defined(CONFIG_PWM_CONTROL_ARM) || defined(CONFIG_MACH_RK3026_PMU_ACT9831)

#else

#if defined(CONFIG_MFD_TPS65910)	
	if (pmic_is_tps65910())
		#if defined(CONFIG_MACH_YK_86V_V2_0)//liufeng@20130909
		ldo = regulator_get(NULL, "vdac");	//vdac
		#else
		ldo = regulator_get(NULL, "vmmc");  //vccio_wl
		#endif
#endif
#if defined(CONFIG_REGULATOR_ACT8931)
	if(pmic_is_act8931())
		ldo = regulator_get(NULL, "act_ldo4");  //vccio_wl
#endif	
	
#if defined(CONFIG_PMIC_AXP192)
	if(pmic_is_axp192())
		ldo = regulator_get(NULL, "ldo3");  //vccio_wl
#endif

	if (ldo == NULL)
	{
		printk("\n\n\n%s get ldo error,please mod this\n\n\n");
		
	}
	else
	{

		if(!on) {
		//regulator_set_voltage(ldo, 3000000, 3000000);
		regulator_set_voltage(ldo, 2850000, 2850000);
		regulator_set_voltage(ldo, 2850000, 2850000);
		while(!regulator_is_enabled(ldo)) 
		ret = regulator_enable(ldo);
		if(ret != 0){
			printk("faild to enable vmmc\n");
		}
		printk("%s: vccio_wl enable\n", __func__);
	} else {
		printk("%s: vccio_wl disable\n", __func__);
		while(regulator_is_enabled(ldo)) 
		ret = regulator_disable(ldo);
		if(ret != 0){
			printk("faild to disable vmmc\n");
		}
		printk("%s: vccio_wl disable\n", __func__);
	}	
	regulator_put(ldo);
	udelay(100);
	}
#endif
}

#endif



/***********************************************************
*	sdmmc
************************************************************/
#ifdef CONFIG_SDMMC_RK29
#include "board-rk3026-86v-sdmmc-config.c"
#include "../plat-rk/rk-sdmmc-ops.c"
#include "../plat-rk/rk-sdmmc-wifi.c"
#endif //endif ---#ifdef CONFIG_SDMMC_RK29

#ifdef CONFIG_SDMMC0_RK29
#define CONFIG_SDMMC0_USE_DMA
static int rk29_sdmmc0_cfg_gpio(void)
{
	rk29_sdmmc_set_iomux(0, 0xFFFF);
	#if defined(CONFIG_SDMMC0_RK29_SDCARD_DET_FROM_GPIO)
        iomux_set_gpio_mode(iomux_gpio_to_mode(RK29SDK_SD_CARD_DETECT_N));
    	#else
        iomux_set(MMC0_DETN);
    	#endif	

	#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	gpio_request(SDMMC0_WRITE_PROTECT_PIN, "sdmmc-wp");
	gpio_direction_input(SDMMC0_WRITE_PROTECT_PIN);
	#endif
	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34 | MMC_VDD_34_35 | MMC_VDD_35_36),
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,

	.set_iomux = rk29_sdmmc_set_iomux,
#ifdef USE_SDMMC_DATA4_DATA7	
	.emmc_is_selected = NULL,
#endif
	.dma_name = "sd_mmc",
#ifdef CONFIG_SDMMC0_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC) && defined(CONFIG_USE_SDMMC0_FOR_WIFI_DEVELOP_BOARD)
	.status = rk29sdk_wifi_mmc0_status,
	.register_status_notify = rk29sdk_wifi_mmc0_status_register,
#endif

#if defined(RK29SDK_SD_CARD_PWR_EN) || (INVALID_GPIO != RK29SDK_SD_CARD_PWR_EN)
	.power_en = RK29SDK_SD_CARD_PWR_EN,
	.power_en_level = RK29SDK_SD_CARD_PWR_EN_LEVEL,
#else
	.power_en = INVALID_GPIO,
	.power_en_level = GPIO_LOW,
#endif    
	.enable_sd_wakeup = 0,

#if defined(CONFIG_SDMMC0_RK29_WRITE_PROTECT)
	.write_prt = SDMMC0_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC0_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

	.det_pin_info = {    
    		#if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        	.io             = RK29SDK_SD_CARD_DETECT_N, //INVALID_GPIO,
        	.enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
    		#else
        	.io             = INVALID_GPIO,
        	.enable         = GPIO_LOW,
    		#endif    
    	}, 

};
#endif // CONFIG_SDMMC0_RK29

#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	gpio_request(SDMMC1_WRITE_PROTECT_PIN, "sdio-wp");
	gpio_direction_input(SDMMC1_WRITE_PROTECT_PIN);
#endif
	return 0;
}

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail =
	    (MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 |
	     MMC_VDD_33_34),

#if !defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_SDIO_IRQ |
		      MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#else
	.host_caps =
	    (MMC_CAP_4_BIT_DATA | MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED),
#endif

	.io_init = rk29_sdmmc1_cfg_gpio,

	.set_iomux = rk29_sdmmc_set_iomux,
#ifdef USE_SDMMC_DATA4_DATA7	
		.emmc_is_selected = NULL,
#endif
	.dma_name = "sdio",
#ifdef CONFIG_SDMMC1_USE_DMA
	.use_dma = 1,
#else
	.use_dma = 0,
#endif

#if defined(CONFIG_WIFI_CONTROL_FUNC) || defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
	.status = rk29sdk_wifi_status,
	.register_status_notify = rk29sdk_wifi_status_register,
#endif

#if defined(CONFIG_SDMMC1_RK29_WRITE_PROTECT)
	.write_prt = SDMMC1_WRITE_PROTECT_PIN,
	.write_prt_enalbe_level = SDMMC1_WRITE_PROTECT_ENABLE_VALUE;
#else
	.write_prt = INVALID_GPIO,
#endif

    #if defined(CONFIG_RK29_SDIO_IRQ_FROM_GPIO)
	.sdio_INT_gpio = RK29SDK_WIFI_SDIO_CARD_INT,
    #endif

	.det_pin_info = {    
	#if defined(CONFIG_USE_SDMMC1_FOR_WIFI_DEVELOP_BOARD)
		#if defined(RK29SDK_SD_CARD_DETECT_N) || (INVALID_GPIO != RK29SDK_SD_CARD_DETECT_N)  
        	.io             = RK29SDK_SD_CARD_DETECT_N,
     		#else
         	.io             = INVALID_GPIO,
		#endif   

        	.enable         = RK29SDK_SD_CARD_INSERT_LEVEL,
	#else
        	.io             = INVALID_GPIO,
        	.enable         = GPIO_LOW,
	#endif
	},
	.enable_sd_wakeup = 0,
};
#endif //endif--#ifdef CONFIG_SDMMC1_RK29

#ifdef CONFIG_SDMMC2_RK29
static int rk29_sdmmc2_cfg_gpio(void)
{
    ;
}

struct rk29_sdmmc_platform_data default_sdmmc2_data = {
	.host_ocr_avail =
	    (MMC_VDD_165_195|MMC_VDD_25_26 | MMC_VDD_26_27 | MMC_VDD_27_28 | MMC_VDD_28_29 |
	     MMC_VDD_29_30 | MMC_VDD_30_31 | MMC_VDD_31_32 | MMC_VDD_32_33 | MMC_VDD_33_34),

	.host_caps = (MMC_CAP_4_BIT_DATA | MMC_CAP_8_BIT_DATA| MMC_CAP_NONREMOVABLE  |
	        MMC_CAP_MMC_HIGHSPEED | MMC_CAP_SD_HIGHSPEED |
		    MMC_CAP_1_8V_DDR | MMC_CAP_1_2V_DDR |MMC_CAP_UHS_SDR12 |MMC_CAP_UHS_SDR25 |MMC_CAP_UHS_SDR50|
		    /*MMC_CAP_UHS_SDR104|MMC_CAP_UHS_DDR50|*/
		    MMC_CAP_BUS_WIDTH_TEST | MMC_CAP_ERASE | MMC_CAP_CMD23),


	.io_init = rk29_sdmmc2_cfg_gpio,
	.set_iomux = rk29_sdmmc_set_iomux,
	.emmc_is_selected = sdmmc_is_selected_emmc,

	//.power_en = INVALID_GPIO,
   // .power_en_level = GPIO_LOW,

	.dma_name = "emmc",
	.use_dma = 1,

};
#endif//endif--#ifdef CONFIG_SDMMC2_RK29


#ifdef CONFIG_BATTERY_RK30_ADC_FAC

#ifdef CONFIG_MACH_RK3026_PMU_ACT9831
#define DC_DET_PIN  INVALID_GPIO //RK30_PIN3_PB3
#else
#if  defined(CONFIG_PWM_CONTROL_LOGIC) && defined(CONFIG_PWM_CONTROL_ARM)
#define DC_DET_PIN RK30_PIN1_PB1
#else
#define DC_DET_PIN RK30_PIN1_PA5
#endif
#endif

#define CHARGE_OK_PIN   INVALID_GPIO//RK30_PIN0_PC6


#ifdef CONFIG_MFD_TPS65910
static int tps65910_charge_ok;
static irqreturn_t tps65910_gpio0_r_irq(int irq, void *irq_data)//上升沿中断函数
{
	//printk("-----------------chg_ok_det######### %s\n",__func__);
	tps65910_charge_ok = 1 ;
	return IRQ_HANDLED;
}

static irqreturn_t tps65910_gpio0_f_irq(int irq, void *irq_data)//下降沿中断函数
{
	//printk("-----------------chg_no_ok######### %s\n",__func__);
	tps65910_charge_ok = 0 ;
	return IRQ_HANDLED;
}

static int charging_ok_int = -1 ;
extern struct tps65910 *g_tps65910;   
int rk30_battery_adc_charging_ok( ){
    //printk(">>>>>>>>>>return tps65910_charge_ok = %d \n",tps65910_charge_ok);
#if defined(CONFIG_MFD_TRS65910)/***************shine************************/     
    /********************check register to understand if the battery is full or now********************/
	//printk("%s:g_pmic_type=%d\n",__func__,g_pmic_type);
	if (pmic_is_tps65910()){
		int  reg;
		//printk("=======liufeng 20131207==%s==\n",__FUNCTION__);
		#if 1
		reg = tps65910_reg_read(g_tps65910, 0x60);
		//printk("*********************************reg = %x***********\n",reg);
		reg &= 0x02;
		//printk("*********************************revise reg = %x***********\n",reg);
		if(reg == 0x02)
		{
		tps65910_charge_ok = 1 ;
		}else{
		tps65910_charge_ok = 0 ;
		}
		#else
		tps65910_charge_ok = 0 ;
		#endif 
	}
#else		  
    if (pmic_is_tps65910() && (charging_ok_int == -1) ){
	charging_ok_int = 0 ;
	//上升沿
	//ret = request_irq(IRQ_BOARD_BASE+TPS65910_IRQ_GPIO_R, tps65910_gpio0_r_irq, IRQF_TRIGGER_RISING, "chg_ok", NULL);

	int 
	    ret = request_threaded_irq( IRQ_BOARD_BASE +TPS65910_IRQ_GPIO_R,
		    NULL, tps65910_gpio0_r_irq, IRQF_TRIGGER_RISING,
		    "chg_ok", NULL);
	if (ret) {
	    printk("failed to request_irq IRQ_BOARD_BASE +TPS65910_IRQ_GPIO_R , error = %d \n",ret);
	}else{
	    printk("request_irq IRQ_BOARD_BASE +TPS65910_IRQ_GPIO_R success \n");
	}

	//下降沿
	//ret = request_irq(IRQ_BOARD_BASE +TPS65910_IRQ_GPIO_F, tps65910_gpio0_f_irq, IRQF_TRIGGER_FALLING, "chg_no_ok", NULL);
	ret = request_threaded_irq( IRQ_BOARD_BASE +TPS65910_IRQ_GPIO_F,
		NULL, tps65910_gpio0_f_irq, IRQF_TRIGGER_FALLING,
		"chg_no_ok", NULL);
	if (ret) {
	    printk("failed to request_irq IRQ_BOARD_BASE +TPS65910_IRQ_GPIO_F , error = %d \n",ret);
	}else{
	    printk("request_irq IRQ_BOARD_BASE +TPS65910_IRQ_GPIO_F success \n");
	}


    }
#endif
    if( gpio_get_value(DC_DET_PIN) == GPIO_LOW){         
		//printk("======liufeng 20131207  tps65910_charge_ok:%d\n", tps65910_charge_ok);
	if( tps65910_charge_ok ){
	    return 1 ;
	}
    }

    return 0 ;
}
#endif

int rk30_adc_battery_io_init(void){
	//dc charge detect pin
	int ret=0;
	if (DC_DET_PIN != INVALID_GPIO){
	    	ret = gpio_request(DC_DET_PIN, NULL);
	    	if (ret) {
	    		printk("failed to request dc_det gpio\n");
	    	}
	
	    	gpio_pull_updown(DC_DET_PIN, PullDisable);//important
	    	ret = gpio_direction_input(DC_DET_PIN);
	    	if (ret) {
	    		printk("failed to set gpio dc_det input\n");
	    	}
	}
	
	//charge ok detect
	if (CHARGE_OK_PIN != INVALID_GPIO){
 		ret = gpio_request(CHARGE_OK_PIN, NULL);
	    	if (ret) {
	    		printk("failed to request charge_ok gpio\n");
	    	}
	
	    	gpio_pull_updown(CHARGE_OK_PIN, GPIOPullUp);//important
	    	ret = gpio_direction_input(CHARGE_OK_PIN);
	    	if (ret) {
	    		printk("failed to set gpio charge_ok input\n");
	    	}
	}
 		ret = gpio_request(CHARGE_OK_PIN, NULL);
	    	if (ret) {
	    		printk("failed to request charge_ok gpio\n");
	    	}
	
	    	gpio_pull_updown(CHARGE_OK_PIN, GPIOPullUp);//important
	    //	ret = gpio_direction_input(RK30_PIN3_PD6);
	    //	if (ret) {
	    //		printk("failed to set gpio charge_ok input\n");
	    //	}
	

}

#if defined(CONFIG_REGULATOR_ACT8931)
extern  int act8931_charge_det;
extern  int act8931_charge_ok;

int rk30_battery_adc_is_dc_charging( ){
        return  act8931_charge_det  ;  
}
int rk30_battery_adc_charging_ok( ){
       return act8931_charge_ok ;
}
static int set_usb_charging_current(int mode)
{
        if ((ac_current == CHARING_CURRENT_1000MA) && (mode == PC_MODE) ) {
                printk("charging: set charging current 500ma\n");
                gpio_set_value(DC_CUR_SET_PIN, GPIO_LOW);
                ac_current = CHARING_CURRENT_500MA;
        } else if (mode == ADAPT_MODE) {
                if ((ac_current == CHARING_CURRENT_500MA)) {
                        printk("charging: set charging current 1000ma\n");
                        gpio_set_value(DC_CUR_SET_PIN, GPIO_HIGH);
                        ac_current = CHARING_CURRENT_1000MA;
                }
        }
	return 0;
}
#endif

static struct rk30_adc_battery_platform_data rk30_adc_battery_platdata = {
        .dc_det_pin      = DC_DET_PIN,
        .batt_low_pin    = INVALID_GPIO, 
        .charge_set_pin  = INVALID_GPIO,
#if defined(CONFIG_MACH_RK3026_86V_NOPMU)
        .charge_ok_pin   = RK30_PIN0_PA0,//RK30_PIN1_PA0,
#else        
        .charge_ok_pin   = INVALID_GPIO,//RK30_PIN1_PA0,
#endif
	 .usb_det_pin = INVALID_GPIO,
        .dc_det_level    = GPIO_LOW,
        .charge_ok_level = GPIO_HIGH,

	#if defined(CONFIG_REGULATOR_ACT8931)
	.is_dc_charging  = rk30_battery_adc_is_dc_charging,
	.charging_ok	 = rk30_battery_adc_charging_ok ,
	.control_usb_charging= set_usb_charging_current,
	#endif

	.reference_voltage = 3200, // the rK2928 is 3300;RK3066 and rk29 are 2500;rk3066B is 1800;
       .pull_up_res = 200,     //divider resistance ,  pull-up resistor
       .pull_down_res = 200, //divider resistance , pull-down resistor

	  .io_init = rk30_adc_battery_io_init,
	  #ifdef CONFIG_MFD_TPS65910
	  .charging_ok     = rk30_battery_adc_charging_ok ,
	  #endif
	.is_reboot_charging = 1,
        .save_capacity   = 1 ,
        .low_voltage_protection = 3600,    
};

static struct platform_device rk30_device_adc_battery = {
        .name   = "rk30-battery",
        .id     = -1,
        .dev = {
                .platform_data = &rk30_adc_battery_platdata,
        },
};
#endif


/***********************************************************
*	rfkill
************************************************************/
#ifdef CONFIG_RFKILL_RK
// bluetooth rfkill device, its driver in net/rfkill/rfkill-rk.c
static struct rfkill_rk_platform_data rfkill_rk_platdata = {
	.type               = RFKILL_TYPE_BLUETOOTH,

	.poweron_gpio       = { // BT_REG_ON
		.io             = INVALID_GPIO, //RK30_PIN3_PC7,
		.enable         = GPIO_HIGH,
		.iomux          = {
			.name       = "bt_poweron",
			//.fgpio      = GPIO3_C7,
		},
	},

	.reset_gpio         = { // BT_RST
	#if defined(CONFIG_RTL8723AU) 
		.io             = INVALID_GPIO,  // set io to INVALID_GPIO for disable it
	#else
		.io             = RK30_PIN1_PB3, // set io to INVALID_GPIO for disable it
	#endif
		.enable         = GPIO_LOW,
		.iomux          = {
			.name       = "bt_reset",
			.fgpio      = GPIO1_B3,
		},
	}, 

	.wake_gpio          = { // BT_WAKE, use to control bt's sleep and wakeup
	#if defined(CONFIG_RTL8723AU) 
		.io             = INVALID_GPIO,  // set io to INVALID_GPIO for disable it
	#else
		.io             = RK30_PIN1_PB2, // set io to INVALID_GPIO for disable it
	#endif
		.enable         = GPIO_HIGH,
		.iomux          = {
			.name       = "bt_wake",
			.fgpio      = GPIO1_B2,
		},
	},

	.wake_host_irq      = { // BT_HOST_WAKE, for bt wakeup host when it is in deep sleep
		.gpio           = {
	#if defined(CONFIG_RTL8723AU) 
		.io             = INVALID_GPIO,  // set io to INVALID_GPIO for disable it
	#else
			.io         = RK30_PIN0_PA4, // set io to INVALID_GPIO for disable it
	#endif
			.enable     = GPIO_LOW,      // set GPIO_LOW for falling, set 0 for rising
			.iomux      = {
				.name   = "bt_wake_host",
				//.fgpio  = GPIO0_A4,  
			},
		},
	},

	.rts_gpio           = { // UART_RTS, enable or disable BT's data coming
	#if defined(CONFIG_RTL8723AU) 
		.io             = INVALID_GPIO,  // set io to INVALID_GPIO for disable it
	#else
		.io             = RK30_PIN1_PA3, // set io to INVALID_GPIO for disable it
	#endif
		.enable         = GPIO_LOW,
		.iomux          = {
			.name       = "bt_rts",
			.fgpio      = GPIO1_A3,
			.fmux       = UART0_RTSN,
		},
	}
};

static struct platform_device device_rfkill_rk = {
    .name   = "rfkill_rk",
    .id     = -1,
    .dev    = {
        .platform_data = &rfkill_rk_platdata,
    },
};
#endif

/***********************************************************
*	ion
************************************************************/
#ifdef CONFIG_ION
static struct ion_platform_data rk30_ion_pdata = {
	.nr = 1,
	.heaps = {
		{
			.type = ION_HEAP_TYPE_CARVEOUT,
			.id = ION_NOR_HEAP_ID,
			.name = "norheap",
			.size = ION_RESERVE_SIZE,
		}
	},
};

static struct platform_device device_ion = {
	.name = "ion-rockchip",
	.id = 0,
	.dev = {
		.platform_data = &rk30_ion_pdata,
	},
};
#endif

/***********************************************************
*	vcom pwm regulator
************************************************************/
#ifdef CONFIG_INCAR_PNLVCOM_ADJUST
#define REG_PWM_VCOM			1  // (0 ~ 2)
static int pwm_voltage_map_VCOM[] = {
	800000,  825000,  850000,  875000,  900000,  925000 ,
	950000,  975000,  1000000, 1025000, 1050000, 1075000, 
	1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 
	1250000, 1275000, 1300000, 1325000, 1350000, 1375000
};

static struct regulator_consumer_supply pwm_dcdc1_consumers_VCOM[] = {
	{
		.supply = "vcc_vcom",
	}
};

struct regulator_init_data pwm_regulator_init_dcdc_VCOM[1] = {
	{
		.constraints = {
			.name = "PWM_VCOM",
			.min_uV = 600000,
			.max_uV = 1800000,      //0.6-1.8V
			.apply_uV = true,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(pwm_dcdc1_consumers_VCOM),
		.consumer_supplies = pwm_dcdc1_consumers_VCOM,
	},
};

static struct pwm_platform_data pwm_regulator_info_VCOM[1] = {
	{
		.pwm_id = REG_PWM_VCOM,
		.pwm_voltage = 1200000,
		.suspend_voltage = 1050000,
		.min_uV = 950000,
		.max_uV = 1400000,
		.coefficient = 504,     //50.4%
		.pwm_voltage_map = pwm_voltage_map_VCOM,
		.init_data      = &pwm_regulator_init_dcdc_VCOM[0],
	},
};
struct platform_device pwm_regulator_device_VCOM[1] = {
	{
		.name = "pwm-voltage-regulator-vcom",
		.id = 0,
		.dev            = {
			.platform_data = &pwm_regulator_info_VCOM[0],
		}
	},
};

static void pwm_regulator_init_VCOM(void)
{
	pwm_regulator_info_VCOM[0].pwm_gpio = iomux_mode_to_gpio(pwm_mode[REG_PWM_VCOM]);
	pwm_regulator_info_VCOM[0].pwm_iomux_pwm = pwm_mode[REG_PWM_VCOM];
	pwm_regulator_info_VCOM[0].pwm_iomux_gpio = iomux_switch_gpio_mode(pwm_mode[REG_PWM_VCOM]);
}
#endif

/***********************************************************
*	pwm regulator
************************************************************/
//#if defined(CONFIG_RK30_PWM_REGULATOR) && !defined(CONFIG_INCAR_PNLVCOM_ADJUST)
#if defined(CONFIG_RK30_PWM_REGULATOR) 
static int pwm_voltage_map[] = {
	800000,  825000,  850000,  875000,  900000,  925000 ,
	950000,  975000,  1000000, 1025000, 1050000, 1075000, 
	1100000, 1125000, 1150000, 1175000, 1200000, 1225000, 
	1250000, 1275000, 1300000, 1325000, 1350000, 1375000,
	1400000
};



static struct regulator_consumer_supply pwm_dcdc1_consumers[] = {
#ifdef CONFIG_PWM_LOGIC_WITH_ARM
	{
		.supply = "vdd_cpu",
	}
#else
	{
		.supply = "vdd_core",
	}
#endif
};

static struct regulator_consumer_supply pwm_dcdc2_consumers[] = {
#ifdef CONFIG_PWM_LOGIC_WITH_ARM
	{
		.supply = "vdd_core",
	}
#else
	{
		.supply = "vdd_cpu",
	}
#endif
};

struct regulator_init_data pwm_regulator_init_dcdc[2] = {
	{
		.constraints = {
			.name = "PWM_DCDC1",
			.min_uV = 600000,
			.max_uV = 1800000,      //0.6-1.8V
			.apply_uV = true,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(pwm_dcdc1_consumers),
		.consumer_supplies = pwm_dcdc1_consumers,
	},
	{
		.constraints = {
			.name = "PWM_DCDC2",
			.min_uV = 600000,
			.max_uV = 1800000,      //0.6-1.8V
			.apply_uV = true,
			.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
		},
		.num_consumer_supplies = ARRAY_SIZE(pwm_dcdc2_consumers),
		.consumer_supplies = pwm_dcdc2_consumers,
	},
};

static struct pwm_platform_data pwm_regulator_info[2] = {
	{
#ifdef CONFIG_PWM_LOGIC_WITH_ARM
		.pwm_id = REG_PWM_ARM,
#else
		.pwm_id = REG_PWM_LOGIC,
#endif
		.pwm_voltage = 1200000,
		.suspend_voltage = 1050000,
#ifdef CONFIG_PWM_LOGIC_WITH_ARM
		.min_uV = 900000,
		.max_uV = 1400000,
#else
		.min_uV = 800000,
		.max_uV = 1375000,
#endif
		.coefficient = 480,     //50.4%
		.pwm_voltage_map = pwm_voltage_map,
		.init_data      = &pwm_regulator_init_dcdc[0],
	},
	{
#ifdef CONFIG_PWM_LOGIC_WITH_ARM
		.pwm_id = REG_PWM_LOGIC,
#else
		.pwm_id = REG_PWM_ARM,
#endif
		.pwm_voltage = 1200000,
		.suspend_voltage = 1050000,
		.min_uV = 900000,
		.max_uV = 1400000,
		.coefficient = 480,     //50.4%
		.pwm_voltage_map = pwm_voltage_map,
		.init_data      = &pwm_regulator_init_dcdc[1],
	},
};
struct platform_device pwm_regulator_device[2] = {
	{
		.name = "pwm-voltage-regulator",
		.id = 0,
		.dev            = {
			.platform_data = &pwm_regulator_info[0],
		}
	},
	{
		.name = "pwm-voltage-regulator",
		.id = 1,
		.dev            = {
			.platform_data = &pwm_regulator_info[1],
		}
	},
};

static void pwm_regulator_init(void)
{
#ifdef CONFIG_PWM_LOGIC_WITH_ARM
	pwm_regulator_info[0].pwm_gpio = iomux_mode_to_gpio(pwm_mode[REG_PWM_ARM]);;
	pwm_regulator_info[0].pwm_iomux_pwm = pwm_mode[REG_PWM_ARM];
	pwm_regulator_info[0].pwm_iomux_gpio = iomux_switch_gpio_mode(pwm_mode[REG_PWM_ARM]);
#else
	pwm_regulator_info[0].pwm_gpio = iomux_mode_to_gpio(pwm_mode[REG_PWM_LOGIC]);;
	pwm_regulator_info[0].pwm_iomux_pwm = pwm_mode[REG_PWM_LOGIC];
	pwm_regulator_info[0].pwm_iomux_gpio = iomux_switch_gpio_mode(pwm_mode[REG_PWM_LOGIC]);
#endif
#ifdef CONFIG_PWM_CONTROL_ARM
	pwm_regulator_info[1].pwm_gpio = iomux_mode_to_gpio(pwm_mode[REG_PWM_ARM]);
	pwm_regulator_info[1].pwm_iomux_pwm = pwm_mode[REG_PWM_ARM];
	pwm_regulator_info[1].pwm_iomux_gpio = iomux_switch_gpio_mode(pwm_mode[REG_PWM_ARM]);
#endif
}
#endif

int __sramdata pwm_iomux_logic, pwm_do_logic, pwm_dir_logic, pwm_en_logic;
int __sramdata pwm_iomux_arm, pwm_do_arm, pwm_dir_arm, pwm_en_arm;
#define grf_readl(offset)       readl_relaxed(RK30_GRF_BASE + offset)
#define grf_writel(v, offset)   do { writel_relaxed(v, RK30_GRF_BASE + offset); dsb(); } while (0)

#define gpio_readl(offset)       readl_relaxed(RK2928_GPIO0_BASE + offset)
#define gpio_writel(v, offset)   do { writel_relaxed(v, RK2928_GPIO0_BASE + offset); dsb(); } while (0)

#define GPIO_DIR 0x04
#define GPIO_D0  0x00

#define GPIO0_D2_OFFSET		20
void __sramfunc rk30_pwm_logic_suspend_voltage(void)
{
	/* pwm0: GPIO0_D2, pwm1: GPIO0_D3, pwm2: GPIO0_D4 */
	sram_udelay(10000);

#ifdef CONFIG_PWM_CONTROL_LOGIC
#ifdef CONFIG_PWM_LOGIC_WITH_ARM	
	int off = GPIO0_D2_OFFSET + 2*REG_PWM_ARM;
#else
	int off = GPIO0_D2_OFFSET + 2*REG_PWM_LOGIC;
#endif
	pwm_iomux_logic = grf_readl(GRF_GPIO0D_IOMUX);
	pwm_dir_logic = gpio_readl(GPIO_DIR);
	pwm_do_logic = gpio_readl(GPIO_D0);

	grf_writel(1<<off, GRF_GPIO0D_IOMUX);
	gpio_writel(pwm_dir_logic | 0x08000000, GPIO_DIR);
	gpio_writel(pwm_do_logic | 0x08000000, GPIO_D0); 	
#endif

#ifdef CONFIG_PWM_CONTROL_ARM
        off = GPIO0_D2_OFFSET + 2*REG_PWM_ARM;

	pwm_iomux_arm = grf_readl(GRF_GPIO0D_IOMUX);
	pwm_dir_arm = gpio_readl(GPIO_DIR);
	pwm_do_arm = gpio_readl(GPIO_D0);

	grf_writel(1<<off, GRF_GPIO0D_IOMUX);
	gpio_writel(pwm_dir_arm | 0x04000000, GPIO_DIR);
	gpio_writel(pwm_do_arm | 0x04000000, GPIO_D0);	
#endif
}

void __sramfunc rk30_pwm_logic_resume_voltage(void)
{
	/* pwm0: GPIO0_D2, pwm1: GPIO0_D3, pwm2: GPIO0_D4 */

#ifdef CONFIG_PWM_CONTROL_LOGIC
	int off = GPIO0_D2_OFFSET + 2*REG_PWM_LOGIC;
	grf_writel((1<<off)|pwm_iomux_logic, GRF_GPIO0D_IOMUX);
	gpio_writel(pwm_dir_logic, GPIO_DIR);
	gpio_writel(pwm_do_logic, GPIO_D0);
	sram_udelay(10000);
	
#endif
#ifdef CONFIG_PWM_CONTROL_ARM
	off = GPIO0_D2_OFFSET + 2*REG_PWM_ARM;

	grf_writel((1<<off)|pwm_iomux_arm, GRF_GPIO0D_IOMUX);
	gpio_writel(pwm_dir_arm, GPIO_DIR);
	gpio_writel(pwm_do_arm, GPIO_D0);
	sram_udelay(10000);
#endif
}

extern void pwm_suspend_voltage(void);
extern void pwm_resume_voltage(void);
extern void pwm_suspend_voltage_vcom(void);
extern void pwm_resume_voltage_vcom(void);

void  rk30_pwm_suspend_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_suspend_voltage();
#endif
}

void  rk30_pwm_resume_voltage_set(void)
{
#ifdef CONFIG_RK30_PWM_REGULATOR
	pwm_resume_voltage();
#endif
}

/***********************************************************
*	pmic
************************************************************/
int __sramdata g_pmic_type =  0;

#ifdef CONFIG_MFD_TPS65910
#define TPS65910_HOST_IRQ 	PMU_INT_PIN
#define PMU_POWER_SLEEP		PMU_SLEEP_PIN
static struct pmu_info  tps65910_dcdc_info[] = {
	{
		.name = "vdd_cpu",
		.min_uv = 1200000,
		.max_uv = 1200000,
	},
	{
		.name = "vdd_core",
		.min_uv = 1200000,
		.max_uv = 1200000,
	},
	{
		.name = "vio",
		.min_uv = 3300000,
		.max_uv = 3300000,
	},
};

static struct pmu_info tps65910_ldo_info[] = {

	{
		.name          = "vpll",   //vcc25
		.min_uv          = 2500000,
		.max_uv         = 2500000,
	},

	{
		.name          = "vdig1",    //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},

	{
		.name          = "vdig2",   //vdd11
		.min_uv          = 1100000,
		.max_uv         = 1100000,
	},
	{
		.name          = "vaux1",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "vaux2",   //vcca33
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vaux33",   //vcc_tp
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
	{
		.name          = "vmmc",   //vcca30
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
#if defined(CONFIG_MACH_YK_86V_V2_0)//liufeng@20130909
	{
		.name          = "vdac",   //
		.min_uv          = 2850000,
		.max_uv         = 2850000,
	},
#else
	{
		.name		   = "vdac",   //
		.min_uv 		 = 1800000,
		.max_uv 		= 1800000,
	},
#endif
};
#include "../mach-rk30/board-pmu-tps65910.c"
#endif

#ifdef CONFIG_PMIC_AXP192
#define AXP192_DEVICES_ADDR		(0x68 >> 1)
#define AXP192_HOST_IRQ			PMU_INT_PIN
static struct pmu_info axp192_dcdc_info[] = {
	{
		.name 	= "dcdc1",		// vcc_ddr
		.min_uv = 1500000,
		.max_uv = 1500000,
	},
	{
		.name 	= "vdd_cpu",	// vdd_cpu
		.min_uv = 1000000,
		.max_uv = 1000000,
	},
	{
		.name 	= "dcdc3",		// vcc_io
		.min_uv = 3300000,//3000000,
		.max_uv = 3300000,//3000000,
	},
	{
		.name 	= "ldoio0",		// vcc28_cif
		.min_uv = 2800000,
		.max_uv = 2800000,
	},
};
static struct pmu_info axp192_ldo_info[] = {
	{
		.name   = "ldo1",   // vcc_rtc
		.min_uv = 3300000,
		.max_uv = 3300000,
	},
	{
		.name   = "ldo2",	// vcca_33 
		.min_uv = 3300000,
		.max_uv = 3300000,
	},
	{
		.name   = "ldo3",   // vcc_wl
		.min_uv = 3300000,
		.max_uv = 3300000,
	},
};
#include "../../../../drivers/power/xpower-axp/axp-board.c"
#endif /* CONFIG_PMIC_AXP192 */
void __sramfunc board_pmu_suspend(void)
{
	#if defined (CONFIG_MFD_TPS65910)
	if(pmic_is_tps65910())
		board_pmu_tps65910_suspend();
	#endif
}

void __sramfunc board_pmu_resume(void)
{
	#if defined (CONFIG_MFD_TPS65910)
	if(pmic_is_tps65910())
		board_pmu_tps65910_resume();
	#endif
}

static int gpio_118,
     gpio_11c,
     gpio_120,
     gpio_124,
     gpio_128,
     gpio_12c,
     gpio_130,
     gpio_134,
     lcd_gpio;

void board_gpio_suspend(void)
{

	
	gpio_118 = grf_readl(0x0118);
	gpio_11c = grf_readl(0x011c);
	gpio_120 = grf_readl(0x0120);
	gpio_124 = grf_readl(0x0124);
	gpio_128 = grf_readl(0x0128);
	gpio_12c = grf_readl(0x012c);
	gpio_130 = grf_readl(0x0130);
	gpio_134 = grf_readl(0x0134);
//	lcd_gpio = grf_readl(0x0cc);
	grf_writel(0xffffffff, 0x0118);
	grf_writel(0xffffffff, 0x011c);
	grf_writel(0xfffffdff, 0x0120);
	grf_writel(0xffffffff, 0x0124);
	grf_writel(0xff7fff7f, 0x0128);
	grf_writel(0xffffffff, 0x012c);
	grf_writel(0xfefffeff, 0x0130);
	grf_writel(0xffffffff, 0x0134);
	
	sram_printch('9');

}
 void board_gpio_resume(void) 
 {
	grf_writel(0xffff0000|gpio_118, 0x0118);
	grf_writel(0xffff0000|gpio_11c, 0x011c);
	grf_writel(0xffff0000|gpio_120, 0x0120);
	grf_writel(0xffff0000|gpio_124, 0x0124);
	grf_writel(0xffff0000|gpio_128, 0x0128);
	grf_writel(0xffff0000|gpio_12c, 0x012c);	
	grf_writel(0xffff0000|gpio_130, 0x0130);
	grf_writel(0xffff0000|gpio_134, 0x0134);
 }

#ifdef CONFIG_SND_SOC_RK3026
struct rk3026_codec_pdata rk3026_codec_pdata_info={
    .spk_ctl_gpio = INVALID_GPIO,
#ifdef CONFIG_MACH_RK3026_PMU_ACT9831
#if defined(CONFIG_MACH_RK3026_86V_WIFI_ESP8089) 
    .hp_ctl_gpio = EXT_GPIO_MTMS,
#else
    .hp_ctl_gpio = RK2928_PIN1_PB0,
#endif
#else
    .hp_ctl_gpio = RK2928_PIN1_PA0,
#endif
};

static struct resource resources_acodec[] = {
	{
		.start 	= RK2928_ACODEC_PHYS,
		.end 	= RK2928_ACODEC_PHYS + RK2928_ACODEC_SIZE - 1,
		.flags 	= IORESOURCE_MEM,
	},
};

static struct platform_device rk3026_codec = {
	.name	= "rk3026-codec",
	.id		= -1,
	.resource = resources_acodec,
    	.dev = {
        	.platform_data = &rk3026_codec_pdata_info,
    }
};
#endif
#if defined(CONFIG_GPS_RK)
#define GPS_OSCEN_PIN 	INVALID_GPIO//RK2928_PIN1_PB0
#define GPS_RXEN_PIN 	INVALID_GPIO//RK2928_PIN1_PB0

static int rk_gps_io_init(void)
{
	printk("%s \n", __FUNCTION__);
	
	gpio_request(GPS_OSCEN_PIN, NULL);
	gpio_direction_output(GPS_OSCEN_PIN, GPIO_LOW);	
	
	iomux_set(GPS_CLK);//GPS_CLK
	iomux_set(GPS_MAG);//GPS_MAG
	iomux_set(GPS_SIGN);//GPS_SIGN
#if 0
	gpio_request(RK30_PIN1_PA6, NULL);
	gpio_direction_output(RK30_PIN1_PA6, GPIO_LOW);

	gpio_request(RK30_PIN1_PA5, NULL);
	gpio_direction_output(RK30_PIN1_PA5, GPIO_LOW);	

	gpio_request(RK30_PIN1_PA7, NULL);
	gpio_direction_output(RK30_PIN1_PA7, GPIO_LOW);		
#endif	
	return 0;
}
static int rk_gps_power_up(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

static int rk_gps_power_down(void)
{
	printk("%s \n", __FUNCTION__);

	return 0;
}

static int rk_gps_reset_set(int level)
{
	return 0;
}
static int rk_enable_hclk_gps(void)
{
	struct clk *gps_aclk = NULL;
	gps_aclk = clk_get(NULL, "aclk_gps");
	if(gps_aclk) {
		clk_enable(gps_aclk);
		clk_put(gps_aclk);
		printk("%s \n", __FUNCTION__);
	}
	else
		printk("get gps aclk fail\n");
	return 0;
}
static int rk_disable_hclk_gps(void)
{
	struct clk *gps_aclk = NULL;
	gps_aclk = clk_get(NULL, "aclk_gps");
	if(gps_aclk) {
		//TO wait long enough until GPS ISR is finished.
		msleep(5);
		clk_disable(gps_aclk);
		clk_put(gps_aclk);
		printk("%s \n", __FUNCTION__);
	}	
	else
		printk("get gps aclk fail\n");
	return 0;
}
static struct rk_gps_data rk_gps_info = {
	.io_init = rk_gps_io_init,
	.power_up = rk_gps_power_up,
	.power_down = rk_gps_power_down,
	.reset = rk_gps_reset_set,
	.enable_hclk_gps = rk_enable_hclk_gps,
	.disable_hclk_gps = rk_disable_hclk_gps,
	.GpsSign = RK2928_PIN1_PA5,
	.GpsMag = RK2928_PIN1_PA4,        //GPIO index
	.GpsClk = RK2928_PIN1_PA2,        //GPIO index
	.GpsVCCEn = GPS_OSCEN_PIN,     //GPIO index
#if 0	
	.GpsSpi_CSO = RK30_PIN1_PA4,    //GPIO index
	.GpsSpiClk = RK30_PIN1_PA5,     //GPIO index
	.GpsSpiMOSI = RK30_PIN1_PA7,	  //GPIO index
#endif	
	.GpsIrq = IRQ_GPS,
	.GpsSpiEn = 0,
	.GpsAdcCh = 2,
	.u32GpsPhyAddr = RK2928_GPS_PHYS,
	.u32GpsPhySize = RK2928_GPS_SIZE,
};

static struct platform_device rk_device_gps = {
	.name = "gps_hv5820b",
	.id = -1,
	.dev		= {
	.platform_data = &rk_gps_info,
		}
	};
#endif
#if defined (CONFIG_TOUCHSCREEN_ZET62XX)
/*
important notice: zet62xx in charge mode 
 must define the function get_system_charge_status
*/
#define TS_RST_GPIO   RK2928_PIN3_PC1
#define TS_INT_GPIO   INVALID_GPIO //RK2928_PIN1_PB0
extern int dwc_vbus_status(void);
static int zet62xx_init_platform_hw(void)
{
	if(TS_RST_GPIO!=INVALID_GPIO){
	  if(gpio_request(TS_RST_GPIO,"ts_rst_gpio")!= 0){
			gpio_free(TS_RST_GPIO);
			printk("zet62xx_init_platform_hw rst_gpio gpio_request error\n");
			return -EIO;
		}   
		gpio_direction_output(TS_RST_GPIO, GPIO_HIGH);
	}
	return 0;
}

/*detect system charge status.
 *return:   1:charging 0:not charging
*/
static int zet62xx_get_charge_status(void)
{
	int val0=0,val1=0,val2=0;
	#ifdef CONFIG_BATTERY_RK30_ADC_FAC
	if(rk30_adc_battery_platdata.is_dc_charging){
		val0=rk30_adc_battery_platdata.is_dc_charging();
	}
	if(rk30_adc_battery_platdata.dc_det_pin!=INVALID_GPIO){
		val1=(gpio_get_value(rk30_adc_battery_platdata.dc_det_pin)==rk30_adc_battery_platdata.dc_det_level)?1:0;
	}
	#ifdef CONFIG_BATTERY_RK30_USB_CHARGE
	{
		val2=dwc_vbus_status()!=0?1:0;
	}
	#endif
  #endif
	return val0|val1|val2;
}
	struct zet62xx_platform_data  zet62xx_info = {
		.reset_gpio = TS_RST_GPIO,
		.irq_gpio   = TS_INT_GPIO,
		.init_platform_hw = zet62xx_init_platform_hw,
	#ifdef CONFIG_BATTERY_RK30_ADC_FAC
		.get_system_charge_status=zet62xx_get_charge_status,
	#endif
	};
#endif

#if defined(CONFIG_TOUCHSCREEN_AW5209)
/*important notice :use the aw5x0x,when system  suspend,vcc_tp can not be closed.if vcc_tp is closed,
suspend current will be too large.
RK3026 use pmu tps65910,
board-pmu-tps65910.c
must modify 
#if defined (CONFIG_ARCH_RK3026)
	val |= 0x0b;
#else	
	val |= 0x0b;
#endif
avoid vcc_tp is closed when system suspend.
*/
#define TOUCH_RESET_PIN RK30_PIN3_PC3
#define TOUCH_INT_PIN   RK30_PIN3_PC7
extern int dwc_vbus_status(void);
static int aw5x0x_init_platform_hw(void)
{
	if(TOUCH_RESET_PIN!=INVALID_GPIO){
	  if(gpio_request(TOUCH_RESET_PIN,NULL)!= 0){
			gpio_free(TOUCH_RESET_PIN);
			printk("aw5x0x_init_platform_hw rst_gpio gpio_request error\n");
			return -EIO;
		}     
		gpio_direction_output(TOUCH_RESET_PIN,GPIO_HIGH);
		msleep(10);
		gpio_set_value(TOUCH_RESET_PIN,GPIO_LOW);
		msleep(100);
		gpio_set_value(TOUCH_RESET_PIN,GPIO_HIGH);
		msleep(10);
	}
	return 0;
}

/*detect system charge status.
 *return:   1:charging 0:not charging
*/
static int aw5x0x_get_charge_status(void)
{
	int val0=0,val1=0,val2=0;
	#ifdef CONFIG_BATTERY_RK30_ADC_FAC
	if(rk30_adc_battery_platdata.is_dc_charging){
		val0=rk30_adc_battery_platdata.is_dc_charging();
	}
	if(rk30_adc_battery_platdata.dc_det_pin!=INVALID_GPIO){
		val1=(gpio_get_value(rk30_adc_battery_platdata.dc_det_pin)==rk30_adc_battery_platdata.dc_det_level)?1:0;
	}
	#ifdef CONFIG_BATTERY_RK30_USB_CHARGE
	{
		val2=dwc_vbus_status()!=0?1:0;
	}
	#endif
  #endif
	return val0|val1|val2;
}

struct aw5x0x_platform_data  aw5x0x_info = {
		.reset_gpio = TOUCH_RESET_PIN,
		.irq_gpio = TOUCH_INT_PIN,
		.init_platform_hw = aw5x0x_init_platform_hw,
	  #ifdef CONFIG_BATTERY_RK30_ADC_FAC	
		.get_system_charge_status=aw5x0x_get_charge_status,
		#endif
};
#endif
/***********************************************************
*	i2c
************************************************************/

#ifdef CONFIG_REGULATOR_ACT8931
#define ACT8931_HOST_IRQ		RK30_PIN2_PB1 //RK30_PIN2_PB1//depend on your hardware


#define ACT8931_CHGSEL_PIN    INVALID_GPIO //RK30_PIN0_PD0 //depend on your hardware


static struct pmu_info  act8931_dcdc_info[] = {
	{
		.name          = "act_dcdc1",   //vdd_io
		.min_uv          = 3200000,
		.max_uv         = 3200000,
	},
	{
		.name          = "act_dcdc2",    //ddr
#ifdef CONFIG_MACH_RK3026_FOR_KINGSTON_DDR
		.min_uv          = 1500000,
		.max_uv         = 1500000,
#else
		.min_uv          = 1500000,
		.max_uv         = 1500000,
#endif
	},
	{
		.name          = "vdd_cpu",   //vdd_arm
		.min_uv          = 1200000,
		.max_uv         = 1200000,
	},
};
static  struct pmu_info  act8931_ldo_info[] = {
	{
		.name          = "act_ldo1",   //vcc28_cif
		.min_uv          = 2800000,
		.max_uv         = 2800000,
	},
	{
		.name          = "act_ldo2",    //vcc18_cif
		.min_uv          = 1800000,
		.max_uv         = 1800000,
	},
	{
		.name          = "act_ldo3",    //vcca30
		.min_uv          = 3200000,
		.max_uv         = 3200000,
	},
	{
		.name          = "act_ldo4",    //vcc_wl
		.min_uv          = 3300000,
		.max_uv         = 3300000,
	},
};
#include "../mach-rk2928/board-rk2928-sdk-act8931.c"
#endif


#ifdef CONFIG_I2C0_RK30
static struct i2c_board_info __initdata i2c0_info[] = {
#if defined (CONFIG_MFD_TPS65910)
	{
		.type           = "tps65910",
		.addr           = TPS65910_I2C_ID0,
		.flags          = 0,
		.irq            = TPS65910_HOST_IRQ,
		.platform_data = &tps65910_data,
	},
#endif

#if defined (CONFIG_REGULATOR_ACT8931)
	{
		.type    		= "act8931",
		.addr           = 0x5b, 
		.flags			= 0,
		.irq            = ACT8931_HOST_IRQ,
		.platform_data=&act8931_data,
	},
#endif
#ifdef CONFIG_MACH_RK3026_PMU_ACT9831
#if !defined(CONFIG_MACH_RK3026_86V_NOPMU)
#if defined (CONFIG_RTC_HYM8563)
    {    
        .type           = "rtc_hym8563",
        .addr           = 0x51,
        .flags          = 0, 
#ifdef CONFIG_MACH_RK3026_86V_WIFI_ESP8089
        .irq            = INVALID_GPIO, //RK30_PIN1_PA5,
#else
        .irq            = RK30_PIN1_PA5,
#endif
    },   
#endif
#endif
#endif
};
#endif

#ifdef CONFIG_I2C1_RK30
static struct i2c_board_info __initdata i2c1_info[] = {
#ifndef CONFIG_MACH_RK3026_PMU_ACT9831_CHG_GSENSOR_TO_I2C2
#if defined (CONFIG_GS_MXC6225)
        {
                .type           = "gs_mxc6225",
                .addr           = 0x15,
                .flags          = 0,
                .irq            = MXC6225_INT_PIN,
                .platform_data  = &mxc6225_info,
        },
#endif
#if defined (CONFIG_GS_MMA7660)
	{
		.type	        = "gs_mma7660",//gs_mma7660
		.addr	        = 0x4c,
		.flags	        = 0,
		.irq	        = MMA7660_INT_PIN,
		.platform_data = &mma7660_info,
	},
#endif
#if defined (CONFIG_GS_MC3XXX)
	{
		.type	        = "gs_mc3XXX",
		.addr	        = 0x4c,
		.flags	        = 0,
		.irq	        = MC3XXX_INT_PIN,
		.platform_data = &MC3XXX_info,
	},
#endif
#if (defined(CONFIG_GS_STK8312))
    {
      .type           = "gs_stk8312",
      .addr           = 0x3D,
      .flags          = 0,
      .irq            = STK831X_INT_PIN,
      .platform_data  = &stk831x_info,
    },
    #endif
#if defined(CONFIG_GS_STK8313)
    {
      .type           = "gs_stk8313",
      .addr           = 0x22,
      .flags          = 0,
      .irq            = STK831X_INT_PIN,
      .platform_data  = &stk831x_info,
    },
    #endif 
#if defined (CONFIG_GS_MIR3DA)
	{
		.type	        = "gs_mir3da",
		.addr	        = 0x27,
		.flags	        = 0,
		.irq	        = GS_INT_PIN,
		.platform_data = &mir3da_info,
	},
#endif
#if defined (CONFIG_GS_GMA303)
       {
               .type           = "gma303",
               .addr           = 0x18,
               .flags          = 0,
               .irq            = GMA303_INT_PIN,
               .platform_data = &GMA303_info,
       },
#endif
#if defined (CONFIG_GS_GMA302)
       {
               .type           = "gma302",
               .addr           = 0x18,
               .flags          = 0,
               .irq            = 0,//GMA302_INT_PIN,
               .platform_data = &GMA302_info,
       },
#endif
#endif

#ifndef CONFIG_MACH_RK3026_PMU_ACT9831
#if defined (CONFIG_RTC_HYM8563)
    {    
        .type           = "rtc_hym8563",
        .addr           = 0x51,
        .flags          = 0, 
        .irq            = RK30_PIN1_PB1,
    },   
#endif
#endif
#if defined(CONFIG_MACH_RK3026_86V_NOPMU)
#if defined (CONFIG_GS_D10)
{
	.type           = "gs_dmard10",
	.addr           = 0x18,
	.flags          = 0,
	.irq            = D10_INT_PIN,
	.platform_data  = &d10_info,
},
#endif
#endif
#if defined (CONFIG_PMIC_AXP192)
	{
		.type = "axp_mfd",
		.addr           = AXP192_DEVICES_ADDR,
		.flags  = 0,		
		.irq            = AXP192_HOST_IRQ,
		.platform_data = &axp_pdata,
	},
#endif
#if defined (CONFIG_GS_MMA8452)
	{
		.type	        = "gs_mma8452",
		.addr	        = 0x1d,
		.flags	        = 0,
		.irq	        = MMA8452_INT_PIN,
		.platform_data = &mma8452_info,
	},
#endif
#if defined (CONFIG_GS_LIS3DH)
	{
		.type	        = "gs_lis3dh",
		.addr	        = 0x19,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
		.flags	        = 0,
		.irq	        = LIS3DH_INT_PIN,
		.platform_data = &lis3dh_info,
	},
#endif
#if defined (CONFIG_GS_LSM303D)
        {
            .type           = "gs_lsm303d",
            .addr           = 0x1d,   //0x19(SA0-->VCC), 0x18(SA0-->GND)
            .flags          = 0,
            .irq            = LSM303D_INT_PIN,         
	    .platform_data  = &lms303d_info,           
        },
#endif
#if defined (CONFIG_COMPASS_AK8975)
	{
		.type          = "ak8975",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8975_info,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8975_info,
	},
#endif
#if defined (CONFIG_COMPASS_AK8963)
	{
		.type          = "ak8963",
		.addr          = 0x0d,
		.flags         = 0,
		.irq           = RK30_PIN3_PD7,	
		.platform_data = &akm8963_info,
	},
#endif
#if defined (CONFIG_GYRO_L3G4200D)
	{
		.type          = "l3g4200d_gryo",
		.addr          = 0x69,
		.flags         = 0,
		.irq           = L3G4200D_INT_PIN,
		.platform_data = &l3g4200d_info,
	},
#endif

#if defined (CONFIG_TOUCHSCREEN_AW5209)
    {
        .type           = "aw5306_ts",
        .addr           = 0x38,
        .flags          = 0,
        .platform_data  = &aw5x0x_info,
    },
#endif

#ifdef CONFIG_MACH_RK3026_86V_CAMERA_FLASH_USE_I2C0
#if defined (CONFIG_REGULATOR_ACT8931)
	{
		.type    		= "act8931",
		.addr           = 0x5b, 
		.flags			= 0,
		.irq            = ACT8931_HOST_IRQ,
		.platform_data=&act8931_data,
	},
#endif
#if defined (CONFIG_RTC_HYM8563)
    {    
        .type           = "rtc_hym8563",
        .addr           = 0x51,
        .flags          = 0, 
#ifdef CONFIG_MACH_RK3026_86V_WIFI_ESP8089
        .irq            = INVALID_GPIO, //RK30_PIN1_PA5,
#else
        .irq            = RK30_PIN1_PA5,
#endif
    },   
#endif
#endif

};
#endif

#ifdef CONFIG_I2C2_RK30
static struct i2c_board_info __initdata i2c2_info[] = {
#if defined (CONFIG_TOUCHSCREEN_GT8XX)
	{
		.type          = "Goodix-TS",
		.addr          = 0x55,
		.flags         = 0,
		.irq           = TOUCH_INT_PIN,
		.platform_data = &goodix_info,
	},
#endif
#if defined (CONFIG_TOUCHSCREEN_GSLX680)
    {
        .type           = "gslX680",
        .addr           = 0x40,
        .flags          = 0,
        .platform_data =&gslx680_info,
    },
#endif
#if defined(CONFIG_TOUCHSCREEN_ZET62XX)
        {
                .type          = "zet6221-ts",
                .addr          = 0x76,
                .flags         = 0,
                .irq           = TS_INT_GPIO,
                .platform_data = &zet62xx_info,
        },
#endif
#ifdef CONFIG_MACH_RK3026_PMU_ACT9831_CHG_GSENSOR_TO_I2C2
#if defined (CONFIG_GS_MXC6225)
        {
                .type           = "gs_mxc6225",
                .addr           = 0x15,
                .flags          = 0,
                .irq            = MXC6225_INT_PIN,
                .platform_data  = &mxc6225_info,
        },
#endif
#if defined (CONFIG_GS_MMA7660)
	{
		.type	        = "gs_mma7660",//gs_mma7660
		.addr	        = 0x4c,
		.flags	        = 0,
		.irq	        = MMA7660_INT_PIN,
		.platform_data = &mma7660_info,
	},
#endif
#if defined (CONFIG_GS_MC3XXX)
	{
		.type	        = "gs_mc3XXX",
		.addr	        = 0x4c,
		.flags	        = 0,
		.irq	        = MC3XXX_INT_PIN,
		.platform_data = &MC3XXX_info,
	},
#endif
#if (defined(CONFIG_GS_STK8312))
    {
      .type           = "gs_stk8312",
      .addr           = 0x3D,
      .flags          = 0,
      .irq            = STK831X_INT_PIN,
      .platform_data  = &stk831x_info,
    },
    #endif
#if defined(CONFIG_GS_STK8313)
    {
      .type           = "gs_stk8313",
      .addr           = 0x22,
      .flags          = 0,
      .irq            = STK831X_INT_PIN,
      .platform_data  = &stk831x_info,
    },
#endif
#if defined (CONFIG_GS_GMA302)
       {
               .type           = "gma302",
               .addr           = 0x18,
               .flags          = 0,
               .irq            = GMA302_INT_PIN,
               .platform_data = &GMA302_info,
       },
#endif
#if defined (CONFIG_GS_GMA303)
       {
               .type           = "gma303",
               .addr           = 0x18,
               .flags          = 0,
               .irq            = GMA303_INT_PIN,
               .platform_data = &GMA303_info,
       },
#endif
#if defined (CONFIG_GS_D10)
{
	.type           = "gs_dmard10",
	.addr           = 0x18,
	.flags          = 0,
	.irq            = D10_INT_PIN,
	.platform_data  = &d10_info,
},
#endif
#if defined (CONFIG_GS_MIR3DA)
	{
		.type	        = "gs_mir3da",
		.addr	        = 0x27,
		.flags	        = 0,
		.irq	        = GS_INT_PIN,
		.platform_data = &mir3da_info,
	},
#endif
#endif
};
#endif

#ifdef CONFIG_I2C3_RK30
static struct i2c_board_info __initdata i2c3_info[] = {
};
#endif

#ifdef CONFIG_I2C_GPIO_RK30
#define I2C_SDA_PIN     INVALID_GPIO// RK2928_PIN2_PD6   //set sda_pin here
#define I2C_SCL_PIN     INVALID_GPIO//RK2928_PIN2_PD7   //set scl_pin here
static int rk30_i2c_io_init(void)
{
        return 0;
}

struct i2c_gpio_platform_data default_i2c_gpio_data = {
       .sda_pin = I2C_SDA_PIN,
       .scl_pin = I2C_SCL_PIN,
       .udelay = 5, // clk = 500/udelay = 100Khz
       .timeout = 100,//msecs_to_jiffies(100),
       .bus_num    = 5,
       .io_init = rk30_i2c_io_init,
};

static struct i2c_board_info __initdata i2c_gpio_info[] = {
};
#endif

static void __init rk30_i2c_register_board_info(void)
{
#ifdef CONFIG_I2C0_RK30
	i2c_register_board_info(0, i2c0_info, ARRAY_SIZE(i2c0_info));
#endif
#ifdef CONFIG_I2C1_RK30
	i2c_register_board_info(1, i2c1_info, ARRAY_SIZE(i2c1_info));
#endif
#ifdef CONFIG_I2C2_RK30
	i2c_register_board_info(2, i2c2_info, ARRAY_SIZE(i2c2_info));
#endif
#ifdef CONFIG_I2C3_RK30
	i2c_register_board_info(3, i2c3_info, ARRAY_SIZE(i2c3_info));
#endif
#ifdef CONFIG_I2C_GPIO_RK30
	i2c_register_board_info(4, i2c_gpio_info, ARRAY_SIZE(i2c_gpio_info));
#endif
}

/***********************************************************
*	board init
************************************************************/
static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_ION
	&device_ion,
#endif
#ifdef CONFIG_WIFI_CONTROL_FUNC
	&rk29sdk_wifi_device,
#endif
#ifdef CONFIG_RFKILL_RK
	&device_rfkill_rk,
#endif
#ifdef CONFIG_BATTERY_RK30_ADC_FAC
 	&rk30_device_adc_battery,
#endif
#ifdef CONFIG_SND_SOC_RK3026
	&rk3026_codec,
#endif
#ifdef CONFIG_GPS_RK
	&rk_device_gps,
#endif
#ifdef CONFIG_INCAR_PNLVCOM_ADJUST	
	&pwm_regulator_device_VCOM,
#endif	

#ifdef CONFIG_PWM_CONTROL_LOGIC
	&pwm_regulator_device[0],
#endif

#if defined(CONFIG_PWM_CONTROL_ARM) || defined(CONFIG_PWM_LOGIC_WITH_ARM)
	&pwm_regulator_device[1],
#endif

};


static void rk30_pm_power_off(void)
{
#if defined(CONFIG_MFD_TPS65910)
//	printk("==liufeng 2013-10-29 ===========%s, \n", __FUNCTION__);
	printk("%s:g_pmic_type=%d\n",__func__,g_pmic_type);
	if(pmic_is_tps65910())
        #if defined(CONFIG_MFD_TRS65910)
//		printk("==liufeng 2013-10-29 ===========\n");
		if( gpio_get_value(DC_DET_PIN) == GPIO_LOW){		 
        arm_pm_restart(0, "charge");
		}
		#else
		tps65910_device_shutdown();//tps65910 shutdown
        #endif
#endif

#if  defined(CONFIG_PMIC_AXP192)
	if(pmic_is_axp192())
		axp_power_off();

#endif

#if defined(CONFIG_REGULATOR_ACT8931)
	if(pmic_is_act8931()){
		 printk("enter dcdet pmic_is_act8931===========\n");
               if(rk30_battery_adc_is_dc_charging())
               {
                       printk("enter restart===========\n");
                       arm_pm_restart(0, "charge");
               }
              act8931_device_shutdown();
        #ifdef CONFIG_MACH_RK3026_86V_CAMERA_FLASH_USE_I2C0
		      iomux_mode_to_gpio(GPIO0_A2);
		      rk_gpio_request(GPIO0_A2, GPIOF_DIR_OUT, GPIO_LOW, "I2C1_SCL");
		      iomux_mode_to_gpio(GPIO0_A3);
		      rk_gpio_request(GPIO0_A3, GPIOF_DIR_OUT, GPIO_LOW, "I2C1_SDA");
        #else
	      iomux_mode_to_gpio(GPIO0_A0);
	      rk_gpio_request(GPIO0_A0, GPIOF_DIR_OUT, GPIO_LOW, "I2C0_SCL");
	      iomux_mode_to_gpio(GPIO0_A1);
	       rk_gpio_request(GPIO0_A1, GPIOF_DIR_OUT, GPIO_LOW, "I2C0_SDA");
	   #endif
	}
#endif
#if  !defined(CONFIG_PMIC_AXP192)
	if (DC_DET_PIN != INVALID_GPIO){
		if( gpio_get_value(DC_DET_PIN) == GPIO_LOW)        
			arm_pm_restart(0, "charge");
#ifdef CONFIG_RK30_PWM_REGULATOR
	if(gpio_get_value(DC_DET_PIN) == GPIO_LOW)
	{
			 printk("enter restart===========\n");
	         arm_pm_restart(0, "charge");
	}
#endif
	}
	#ifdef	CONFIG_MACH_RK3026_86V_NOPMU
	if( gpio_get_value(DC_DET_PIN) == GPIO_LOW)        
		arm_pm_restart(0, "charge");
	#endif

	gpio_direction_output(POWER_ON_PIN, GPIO_LOW);
#endif
	while(1);
}

static void __init machine_rk30_board_init(void)
{
#ifdef CONFIG_INCAR_PNLVCOM_ADJUST
	pwm_regulator_init_VCOM();
#endif
#ifdef	CONFIG_RK30_PWM_REGULATOR
	pwm_regulator_init();
#endif
	#if defined(CONFIG_PMIC_AXP192)
		store_boot_source();
	#endif
	avs_init();
	pm_power_off = rk30_pm_power_off;
	#if !defined(CONFIG_PMIC_AXP192)
	rk_gpio_request(POWER_ON_PIN, GPIOF_DIR_OUT, GPIO_HIGH, "system power on");
	#endif
	rk30_i2c_register_board_info();
	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
	platform_add_devices(devices, ARRAY_SIZE(devices));
	rk_platform_add_display_devices();	
#if defined(CONFIG_WIFI_CONTROL_FUNC)
	rk29sdk_wifi_bt_gpio_control_init();
#elif defined(CONFIG_WIFI_COMBO_MODULE_CONTROL_FUNC)
    rk29sdk_wifi_combo_module_gpio_init();
#endif
}

static void __init rk30_reserve(void)
{
	//fb reserve
#ifdef CONFIG_FB_ROCKCHIP
	resource_fb[0].start = board_mem_reserve_add("fb0 buf", get_fb_size());
	resource_fb[0].end = resource_fb[0].start + get_fb_size()- 1;
	#if 0
	resource_fb[1].start = board_mem_reserve_add("ipp buf", RK30_FB0_MEM_SIZE);
	resource_fb[1].end = resource_fb[1].start + RK30_FB0_MEM_SIZE - 1;
	#endif

	#if defined(CONFIG_FB_ROTATE) || !defined(CONFIG_THREE_FB_BUFFER)
	resource_fb[2].start = board_mem_reserve_add("fb2 buf",get_fb_size());
	resource_fb[2].end = resource_fb[2].start + get_fb_size() - 1;
	#endif
#endif
	//ion reserve
#ifdef CONFIG_ION
	rk30_ion_pdata.heaps[0].base = board_mem_reserve_add("ion", ION_RESERVE_SIZE);
#endif
#ifdef CONFIG_VIDEO_RK29
	rk30_camera_request_reserve_mem();
#endif
#ifdef CONFIG_GPS_RK
	//it must be more than 8MB
	rk_gps_info.u32MemoryPhyAddr = board_mem_reserve_add("gps", SZ_8M);
#endif
	board_mem_reserved();
}

/***********************************************************
*	clock
************************************************************/
static struct cpufreq_frequency_table dvfs_arm_table[] = {
#if defined(CONFIG_MACH_RK3026_86V_NOPMU)
	{.frequency = 312 * 1000,       .index = 1150 * 1000},
	{.frequency = 504 * 1000,       .index = 1150 * 1000},
	{.frequency = 816 * 1000,       .index = 1200 * 1000},
	{.frequency = 912 * 1000,       .index = 1260 * 1000},
	{.frequency = 1008 * 1000,      .index = 1310 * 1000},
#else
#if defined(CONFIG_MACH_ACT8931_LOGIC_WITH_ARM)
	{.frequency = 312 * 1000,       .index = 1200 * 1000},
	{.frequency = 504 * 1000,       .index = 1200 * 1000},
#else
	{.frequency = 312 * 1000,       .index = 1000 * 1000},
	{.frequency = 504 * 1000,       .index = 1050 * 1000},
#endif
	{.frequency = 816 * 1000,       .index = 1250 * 1000},
	{.frequency = 912 * 1000,       .index = 1300 * 1000},
	{.frequency = 1008 * 1000,      .index = 1350 * 1000},
	//{.frequency = 1200 * 1000,      .index = 1200 * 1000},
	//{.frequency = 1416 * 1000,      .index = 1200 * 1000},
	//{.frequency = 1608 * 1000,      .index = 1200 * 1000},
#endif
	{.frequency = CPUFREQ_TABLE_END},
};

#if defined(CONFIG_MFD_TRS65910)
static struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 200 * 1000,       .index = 1100 * 1000},
	{.frequency = 266 * 1000,       .index = 1100 * 1000},
	{.frequency = 400 * 1000,       .index = 1100 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 1000 * 1000},
	{.frequency = 300 * 1000,      .index = 1000 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL,     .index = 1000 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};
#else
static struct cpufreq_frequency_table dvfs_gpu_table[] = {
	{.frequency = 200 * 1000,       .index = 1100 * 1000},
	{.frequency = 266 * 1000,       .index = 1100 * 1000},
	{.frequency = 400 * 1000,       .index = 1100 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
};

static struct cpufreq_frequency_table dvfs_ddr_table[] = {
#ifdef CONFIG_MACH_RK3026_FOR_ETT_DDR
	{.frequency = 336 * 1000 + DDR_FREQ_NORMAL,     .index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
#elif defined(CONFIG_MACH_RK3026_FOR_KINGSTON_DDR)
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL,     .index = 1200 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
#else
	{.frequency = 200 * 1000 + DDR_FREQ_SUSPEND,    .index = 1100 * 1000},
	{.frequency = 336 * 1000,                       .index = 1100 * 1000},
	{.frequency = 400 * 1000 + DDR_FREQ_NORMAL,     .index = 1100 * 1000},
	{.frequency = CPUFREQ_TABLE_END},
#endif
};
#endif

extern void adjust_dvfs_table(int soc_version, struct cpufreq_frequency_table *table);
void __init board_clock_init(void)
{
	rk2928_clock_data_init(periph_pll_default, codec_pll_default, RK30_CLOCKS_DEFAULT_FLAGS);
	//dvfs_set_arm_logic_volt(dvfs_cpu_logic_table, cpu_dvfs_table, dep_cpu2core_table);	
	printk(KERN_INFO "rk3026 soc version:%d\n", rk3026_version_val());
	adjust_dvfs_table(rk3026_version_val(), dvfs_arm_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "cpu"), dvfs_arm_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "gpu"), dvfs_gpu_table);
	dvfs_set_freq_volt_table(clk_get(NULL, "ddr"), dvfs_ddr_table);
}

/************************ end *****************************/
MACHINE_START(RK30, "RK30board")
	.boot_params	= PLAT_PHYS_OFFSET + 0x800,
	.fixup		= rk2928_fixup,
	.reserve	= &rk30_reserve,
	.map_io		= rk2928_map_io,
	.init_irq	= rk2928_init_irq,
	.timer		= &rk2928_timer,
	.init_machine	= machine_rk30_board_init,
MACHINE_END
