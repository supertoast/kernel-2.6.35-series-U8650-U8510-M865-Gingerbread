/* ====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * 
 *                     TRULY R61529  LCD kernel driver
 * 
 * GENERAL DESCRIPTION
 *   driver for R61529LCD on U8510
 * REFERENCES
 * 
 * EXTERNALIZED FUNCTIONS
 *   None.
 * 
 * INITIALIZATION AND SEQUENCING REQUIREMENTS
 * 
 * Copyright (c) 2010 by HUAWEI, Incorporated.  All Rights Reserved.
 * ====*====*====*====*====*====*====*====*====*====*====*====*====*====*====
 * ===========================================================================
 * 
 *                       EDIT HISTORY FOR FILE
 * 
 *  This section contains comments describing changes made to this file.
 *   Notice that changes are listed in reverse chronological order.
 * 
 * 
 * when       who      what, where, why
 * -------------------------------------------------------------------------------
 */
 
/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <linux/delay.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include "lcdc_huawei_config.h"
#include <mach/gpio.h>
#include <linux/io.h>
#include <linux/gpio.h>

#define lCD_DRIVER_NAME "lcdc_truly_r61529_hvga"

#ifdef CONFIG_HUAWEI_BACKLIGHT_USE_CABC
static uint16 lcd_backlight_level[LCD_MAX_BACKLIGHT_LEVEL + 1] = 
  {0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0, 0xFF};
#endif  
static int lcd_reset_gpio;
static struct lcd_state_type truly_r61529_hvga_state = { 0 };
static struct msm_panel_common_pdata *lcdc_truly_r61529_hvga_pdata;
static lcd_panel_type lcd_panel_hvga = LCD_NONE;
struct sequence* truly_r61529_hvga_init_table = NULL;


static const struct sequence truly_r61529_hvga_standby_exit_table[] = 
{
	{0x11, TYPE_COMMAND,120},
	{0x29, TYPE_COMMAND,0},
};

static const struct sequence truly_r61529_hvga_standby_enter_table[]= 
{	
	{0x10,TYPE_COMMAND,200},	  	
};

static void truly_r61529_hvga_disp_powerup(void)
{
	if (!truly_r61529_hvga_state.disp_powered_up && !truly_r61529_hvga_state.display_on) {
		/* Reset the hardware first */
		/* Include DAC power up implementation here */
		truly_r61529_hvga_state.disp_powered_up = TRUE;
	}
	 return;
}

static void truly_r61529_hvga_disp_on(void)
{
	if (truly_r61529_hvga_state.disp_powered_up && !truly_r61529_hvga_state.display_on) 
	{
		LCD_DEBUG("%s: disp on lcd\n", __func__);
		/* Initialize LCD */
		truly_r61529_hvga_state.display_on = TRUE;
	}
	return;
}

static void truly_r61529_hvga_reset(void)
{
	/* Reset LCD*/
	lcdc_truly_r61529_hvga_pdata->panel_config_gpio(1);
	lcd_reset_gpio = *(lcdc_truly_r61529_hvga_pdata->gpio_num + 4);
	return;  
}

static int truly_r61529_hvga_panel_on(struct platform_device *pdev)
{
	boolean para_debug_flag = FALSE;
	uint32 para_num = 0;
	/* open debug file and read the para */
	switch(lcd_panel_hvga)
	{
		case LCD_R61529_TRULY_HVGA:
			para_debug_flag = lcd_debug_malloc_get_para( "truly_r61529_hvga_init_table", 
			(void**)&truly_r61529_hvga_init_table,&para_num);
			break;
		default:
			break;
	}    
       
	if (!truly_r61529_hvga_state.disp_initialized) 
	{
		truly_r61529_hvga_reset();
		lcd_spi_init(lcdc_truly_r61529_hvga_pdata);	/* LCD needs SPI */
		truly_r61529_hvga_disp_powerup();
		truly_r61529_hvga_disp_on(); 
		truly_r61529_hvga_state.disp_initialized = TRUE;
		if( (TRUE == para_debug_flag) && (NULL != truly_r61529_hvga_init_table))
		{
			// lcd_reset();
			process_lcdc_table(truly_r61529_hvga_init_table, para_num,lcd_panel_hvga);
		}
		LCD_DEBUG("%s: truly r61529  lcd initialized\n", __func__);
	} 
	else if (!truly_r61529_hvga_state.display_on) 
	{
		switch(lcd_panel_hvga)
		{
			case LCD_R61529_TRULY_HVGA:
				if( (TRUE == para_debug_flag)&&(NULL != truly_r61529_hvga_init_table))
				{
					//lcd_reset();
					process_lcdc_table(truly_r61529_hvga_init_table, para_num,lcd_panel_hvga);
				}
				else
				{
					/* Exit Standby Mode */
					process_lcdc_table((struct sequence*)&truly_r61529_hvga_standby_exit_table, 
					ARRAY_SIZE(truly_r61529_hvga_standby_exit_table), lcd_panel_hvga);
				}
				/* we use this function to set low to the SPI CS pin 
				 * to ensure this LCD can enter sleep mode and be waken up
				 */
				truly_r61529_set_cs(lcdc_truly_r61529_hvga_pdata);		
				break;
			default:
				break;
		}
		LCD_DEBUG("%s: Exit Standby Mode\n", __func__);
		truly_r61529_hvga_state.display_on = TRUE;
	}
	/* Must malloc before,then you can call free */
	if((TRUE == para_debug_flag)&&(NULL != truly_r61529_hvga_init_table))
	{
		lcd_debug_free_para((void *)truly_r61529_hvga_init_table);
	}
	return 0;
}

static int truly_r61529_hvga_panel_off(struct platform_device *pdev)
{
	if (truly_r61529_hvga_state.disp_powered_up && truly_r61529_hvga_state.display_on) {
		switch(lcd_panel_hvga)
		{
			case LCD_R61529_TRULY_HVGA:
				/* Enter Standby Mode */
				process_lcdc_table((struct sequence*)&truly_r61529_hvga_standby_enter_table, 
				ARRAY_SIZE(truly_r61529_hvga_standby_enter_table), lcd_panel_hvga);

				/* we use this function to set low to the SPI CS pin 
				 * to ensure this LCD can enter sleep mode and be waken up
				 */
				truly_r61529_set_cs(lcdc_truly_r61529_hvga_pdata);
				break;
			default:
				break;
		}
		truly_r61529_hvga_state.display_on = FALSE;
		LCD_DEBUG("%s: Enter Standby Mode\n", __func__);
	}
	return 0;
}

static void truly_r61529_hvga_set_backlight(struct msm_fb_data_type *mfd)
{
	int bl_level = mfd->bl_level;
       
#ifdef CONFIG_HUAWEI_BACKLIGHT_USE_CABC
	if(LCD_MAX_BACKLIGHT_LEVEL < mfd->bl_level) {
		mfd->bl_level = 1;
	}

	/* BCTRL	=1	(Backlight Control Block, This bit is always used to switch brightness for display.)
	 * DD	=0	(Display Dimming)
	 * BL	=1	(Backlight Control)
	 */ 
	serigo(0x3D, 0x24);
	/*Set backlight level*/
	serigo(0x3C, lcd_backlight_level[mfd->bl_level]);

	/* Set still picture mode for content adaptive image functionality*/
	serigo(0x3E, 0x02);

	/* set the minimum brightness value of the display for CABC function*/
	serigo(0x3F, 0x00);
#else
	pwm_set_backlight(bl_level);
#endif
	return;
}

static void truly_r61529_hvga_panel_set_contrast(struct msm_fb_data_type *mfd, unsigned int contrast)
{
	return;
}

static int __devinit truly_r61529_hvga_probe(struct platform_device *pdev)
{
	if (pdev->id == 0) {
		lcdc_truly_r61529_hvga_pdata = pdev->dev.platform_data;
		return 0;
	}
	msm_fb_add_device(pdev);
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = truly_r61529_hvga_probe,
	.driver = {
		.name   = lCD_DRIVER_NAME,
	},
};

static struct msm_fb_panel_data truly_r61529_hvga_panel_data = {
	.on = truly_r61529_hvga_panel_on,
	.off = truly_r61529_hvga_panel_off,
	.set_backlight = truly_r61529_hvga_set_backlight,
	.set_contrast = truly_r61529_hvga_panel_set_contrast,
};

static struct platform_device this_device = {
	.name   = lCD_DRIVER_NAME,
	.id	= 1,
	.dev	= {
		.platform_data = &truly_r61529_hvga_panel_data,
	},
};

static int __init truly_r61529_hvga_panel_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;

	lcd_panel_hvga = lcd_panel_probe();
	if((LCD_R61529_TRULY_HVGA != lcd_panel_hvga) &&  \
	    (msm_fb_detect_client(lCD_DRIVER_NAME))  )
	{
		return 0;
	}

	LCD_DEBUG(" lcd_type=%s, lcd_panel_hvga = %d\n", lCD_DRIVER_NAME, lcd_panel_hvga);

	ret = platform_driver_register(&this_driver);
	if (ret)
		return ret;

	pinfo = &truly_r61529_hvga_panel_data.panel_info;
	pinfo->xres = 320;
	pinfo->yres = 480;
	pinfo->type = LCDC_PANEL;
	pinfo->pdest = DISPLAY_1;
	pinfo->wait_cycle = 0;
	pinfo->bpp = 18;
	pinfo->fb_num = 2;
	pinfo->bl_max = LCD_MAX_BACKLIGHT_LEVEL;
	pinfo->bl_min = LCD_MIN_BACKLIGHT_LEVEL;
	if(LCD_R61529_TRULY_HVGA== lcd_panel_hvga)
	{
		/* changge the frequency high */
		pinfo->clk_rate = 9660 * 1000;    /*for HVGA pixel clk*/
	}
	else
	{
		pinfo->clk_rate = 8192000;    /*for HVGA pixel clk*/
	}
	pinfo->lcdc.h_back_porch = 20;
	pinfo->lcdc.h_front_porch = 40;
	pinfo->lcdc.h_pulse_width = 10;
	pinfo->lcdc.v_back_porch = 8;
	pinfo->lcdc.v_front_porch = 15;
	pinfo->lcdc.v_pulse_width = 2;

	pinfo->lcdc.border_clr = 0;     /* blk */
	pinfo->lcdc.underflow_clr = 0xff;       /* blue */
	pinfo->lcdc.hsync_skew = 0;

	ret = platform_device_register(&this_device);
	if (ret)
		platform_driver_unregister(&this_driver);

	return ret;
}

module_init(truly_r61529_hvga_panel_init);


