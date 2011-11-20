/* drivers\video\msm\mddi_nt35510.c
 * NT35510 LCD driver for 7x27 platform
 *
 * Copyright (C) 2010 HUAWEI Technology Co., ltd.
 * 
 * Date: 2010/12/30
 * 
 */

#include "msm_fb.h"
#include "lcdc_huawei_config.h"
#include "mddihost.h"
		/*delate one line */
#define lCD_DRIVER_NAME "mddi_nt35510_wvga"
struct sequence* nt35510_wvga_init_table = NULL;
static lcd_panel_type lcd_panel_wvga = LCD_NONE;
		/*delate lcd  initialization sequence */

/* Decrease delay time to 100ms to solve flash the screen of suspend */
static const struct sequence nt35510_wvga_standby_exit_table[] = 
{
   /*set the delay time 120ms*/
	{0x1100,0,100}
};

static const struct sequence nt35510_wvga_standby_enter_table[]= 
{
  /*set the delay time 120ms*/
	{0x1000,0,100}
};
static int nt35510_lcd_on(struct platform_device *pdev)
{
	boolean para_debug_flag = FALSE;
    uint32 para_num = 0;
	int ret;
		/*delate one line */
    /* open debug file and read the para */    
    para_debug_flag = lcd_debug_malloc_get_para( "nt35510_wvga_init_table", 
    	(void**)&nt35510_wvga_init_table,&para_num);       
	
    if( (TRUE == para_debug_flag)&&(NULL != nt35510_wvga_init_table))
    {
		//lcd_reset();
		ret = process_lcdc_table(nt35510_wvga_init_table, para_num, lcd_panel_wvga);
    }
    else
    {
		/* Exit Standby Mode */
		ret = process_lcdc_table((struct sequence*)&nt35510_wvga_standby_exit_table, 
					ARRAY_SIZE(nt35510_wvga_standby_exit_table), lcd_panel_wvga);
    }
       
	/* Must malloc before,then you can call free */
	if((TRUE == para_debug_flag)&&(NULL != nt35510_wvga_init_table))
	{
		lcd_debug_free_para((void *)nt35510_wvga_init_table);
	}
		/*delate lcd reset function and read register function  */
	LCD_DEBUG("%s: nt35510_lcd exit sleep mode,ret=%d\n",__func__,ret);
	
	return ret;
}

static int nt35510_lcd_off(struct platform_device *pdev)
{
	int ret = 0;
	process_lcdc_table((struct sequence*)&nt35510_wvga_standby_enter_table, 
    	      		ARRAY_SIZE(nt35510_wvga_standby_enter_table), lcd_panel_wvga);
	LCD_DEBUG("%s: nt35510_lcd enter sleep mode,ret=%d\n",__func__,ret);
	return ret;
}

static void nt35510_set_backlight(struct msm_fb_data_type *mfd)
{
    int bl_level = mfd->bl_level;    

    pwm_set_backlight(bl_level);
    return;
}

static int __devinit nt35510_probe(struct platform_device *pdev)
{
	msm_fb_add_device(pdev);
	return 0;
}

static struct platform_driver this_driver = {
	.probe  = nt35510_probe,
	.driver = {
		.name   = lCD_DRIVER_NAME,
	},
};

static struct msm_fb_panel_data nt35510_panel_data = {
	.on = nt35510_lcd_on,
	.off = nt35510_lcd_off,
	.set_backlight = nt35510_set_backlight,
};

static struct platform_device this_device = {
	.name = lCD_DRIVER_NAME,
	.id	= 0,
	.dev = {
		.platform_data = &nt35510_panel_data,
	}
};

static int __init nt35510_init(void)
{
	int ret;
	struct msm_panel_info *pinfo;
	lcd_panel_wvga=lcd_panel_probe();
	if(LCD_MDDI_NT35510_ALPHA_SI_WVGA!=lcd_panel_wvga)
	{
		return 0;
	}
	
	LCD_DEBUG(" lcd_type=%s, lcd_panel_wvga = %d\n", lCD_DRIVER_NAME, lcd_panel_wvga);
	
	ret = platform_driver_register(&this_driver);
	if (!ret) 
	{
		pinfo = &nt35510_panel_data.panel_info;
		pinfo->xres = 480;
		pinfo->yres = 800;
		pinfo->type = MDDI_PANEL;
		pinfo->pdest = DISPLAY_1;
		pinfo->mddi.vdopkt = MDDI_DEFAULT_PRIM_PIX_ATTR;
		pinfo->wait_cycle = 0;
		/* Set MDDI clk 160MHz,set 16bit per pixel,
		 * adjust the start of data to sync with vsync signal
		 */
		pinfo->bpp = 16;
		pinfo->fb_num = 2;
/* Increase MDDI clk to 192M to reach 45fps */
/* Decrease MDDI clk to 160M and it is 30fps */
		pinfo->clk_rate = 160000000;
		pinfo->clk_min =  160000000;
		pinfo->clk_max =  160000000;
		pinfo->lcd.vsync_enable = TRUE;
        pinfo->lcd.refx100 = 5500;
		pinfo->lcd.v_back_porch = 0;
		pinfo->lcd.v_front_porch = 0;
		pinfo->lcd.v_pulse_width = 22;
		pinfo->lcd.hw_vsync_mode = TRUE;
		pinfo->lcd.vsync_notifier_period = 0;
		pinfo->bl_max = 255;

		ret = platform_device_register(&this_device);
		if (ret)
		{
			platform_driver_unregister(&this_driver);
			LCD_DEBUG("%s: Failed on platform_device_register(): rc=%d \n",__func__, ret);
		}
	}

	return ret;
}
module_init(nt35510_init);
