/*
 * Copyright (C) 2010 Huawei, Inc.
 * Copyright (c) 2008-2010, Huawei. All rights reserved.
 *
 */
 
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/errno.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <asm/mach-types.h>
#include "linux/hardware_self_adapt.h"

static unsigned int camera_id = 0;
static unsigned int lcd_id = 0;
static unsigned int ts_id = 0;
static unsigned int sub_board_id = 0;
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
static unsigned int charge_flag = 0;
#endif
static unsigned int lcd_y_res = 480;

#ifdef CONFIG_FRAMEBUF_SELF_ADAPT
static __u32	frame_buffer_size = 0;
static __u32	frame_buffer_start = 0;	/* physical start address */
#endif
extern compass_gs_position_type  get_compass_gs_position();

extern wifi_device_type  get_wifi_device();

static unsigned int recovery_boot_mode = 0;

#define ATAG_CAMERA_ID 0x4d534D74
int __init parse_tag_camera_id(const struct tag *tags)
{
	 struct tag *t = (struct tag *)tags;
 
	 camera_id = t->u.revision.rev;
	 printk(KERN_DEBUG "%s: camera_id = 0x%x\n", __func__, camera_id);
	 
	 return camera_id;
}
 __tagtable(ATAG_CAMERA_ID, parse_tag_camera_id);
 
#define ATAG_LCD_ID 0x4d534D73
int __init parse_tag_lcd_id(const struct tag *tags)
{
	 struct tag *t = (struct tag *)tags;
 
	 lcd_id = t->u.revision.rev;
	 printk(KERN_DEBUG "%s: lcd_id = 0x%x\n", __func__, lcd_id);
	 
	 return lcd_id;
}
 __tagtable(ATAG_LCD_ID, parse_tag_lcd_id);
 
#define ATAG_TS_ID 0x4d534D75
int __init parse_tag_ts_id(const struct tag *tags)
{
	 struct tag *t = (struct tag *)tags;
 
	 ts_id = t->u.revision.rev;
	 printk(KERN_DEBUG "%s: ts_id = 0x%x\n", __func__, ts_id);
	 
	 return ts_id;
}
__tagtable(ATAG_TS_ID, parse_tag_ts_id);
 
#define ATAG_SUB_BOARD_ID 0x4d534D76
int __init parse_tag_sub_board_id(const struct tag *tags)
{
	 struct tag *t = (struct tag *)tags;

	 sub_board_id = t->u.revision.rev;
	 printk(KERN_DEBUG "%s: sub_board_id = 0x%x\n", __func__, sub_board_id);
	 
	 return sub_board_id;
}
 __tagtable(ATAG_SUB_BOARD_ID, parse_tag_sub_board_id);
 
 
#ifdef CONFIG_USB_AUTO_INSTALL
#define ATAG_BOOT_MODE_ID   0x4d534d77
 int __init parse_tag_boot_mode_id(const struct tag *tags)
 {
	 struct tag *t = (struct tag *)tags;
 
	 recovery_boot_mode = t->u.revision.rev;
	 printk(KERN_DEBUG "%s: usb_mode_id = 0x%x\n", __func__, recovery_boot_mode);
	 return recovery_boot_mode;
 }
 __tagtable(ATAG_BOOT_MODE_ID, parse_tag_boot_mode_id);
#endif  /* #ifdef CONFIG_USB_AUTO_INSTALL */
 
/* get the charge flag from atag */
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
#define ATAG_CHARGE_FLAG  0x4d534D78
int __init parse_tag_charge_flag(const struct tag *tags)
{
    struct tag *t = (struct tag *)tags;

    charge_flag = t->u.revision.rev;
    printk(KERN_DEBUG "%s: charge_flag = 0x%x\n", __func__, charge_flag);

    return charge_flag;  
}
__tagtable(ATAG_CHARGE_FLAG, parse_tag_charge_flag);
#endif

#ifdef CONFIG_FRAMEBUF_SELF_ADAPT
/*get framebuffer address and size from atag, passed by bootloader*/
#define ATAG_FRAME_BUFFER_ID 0x4d534D79
int __init parse_tag_frame_buffer(const struct tag *tags)
{
	frame_buffer_size = tags->u.mem.size;
	frame_buffer_start = tags->u.mem.start;
	
    printk(KERN_DEBUG "%s: fb addr= 0x%x, size=0x%0x\n", __func__, frame_buffer_start, frame_buffer_size);
}
__tagtable(ATAG_FRAME_BUFFER_ID, parse_tag_frame_buffer);

#define ATAG_LCD_Y_RES_FLAG 0x4d534D7A
int __init parse_tag_lcd_y_res_flag(const struct tag *tags)
{
    struct tag *t = (struct tag *)tags;

    lcd_y_res= t->u.revision.rev;
    printk(KERN_DEBUG "%s: lcd_y_res = %d\n", __func__, lcd_y_res);

    return lcd_y_res;  
}
__tagtable(ATAG_LCD_Y_RES_FLAG, parse_tag_lcd_y_res_flag);

/*used in board-msm7x27.c*/
void get_frame_buffer_mem_region(__u32 *start_addr, __u32 *size)
{
	*start_addr = frame_buffer_start;
	*size = frame_buffer_size;
}
#endif

unsigned int get_hw_camera_id(void)
{
	return camera_id;
}

unsigned int get_hw_lcd_id(void)
{
	return lcd_id;
}

unsigned int get_hw_ts_id(void)
{
	return ts_id;
}

hw_ver_sub_type get_hw_sub_board_id(void)
{
	return (hw_ver_sub_type)(sub_board_id&HW_VER_SUB_MASK);
}

#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
unsigned int get_charge_flag(void)
{
    return charge_flag;
}
#endif

lcd_type atag_get_lcd_y_res(void)
{
   return (lcd_type)lcd_y_res;
}

/* the function interface to check boot mode in kernel */
unsigned char bootimage_is_recovery(void)
{
  return recovery_boot_mode;
}

lcd_panel_type lcd_panel_probe(void)
{
	 lcd_panel_type hw_lcd_panel = LCD_NONE;
	 unsigned int lcd_id = LCD_NONE;
	 
#ifdef CONFIG_MSM7X27_SURF_DEBUG
	 return LCD_NONE;
#endif

	 lcd_id = get_hw_lcd_id();

	 if( machine_is_msm7x27_u8510()||machine_is_msm7x27_u8510_1())
	 {
		 switch (lcd_id)
		 {
			 case 0:
				 hw_lcd_panel = LCD_R61529_TRULY_HVGA;
				 break;
			 case 1:
                 /*add an new lcd for U8510 and the s6d05a0 is not used anymore*/
				 //hw_lcd_panel = LCD_S6D05A0_INNOLUX_HVGA;
				 hw_lcd_panel = LCD_NT35410_CHIMEI_HVGA;
				 break;
            case 2:
                hw_lcd_panel = LCD_HX8357B_TIANMA_HVGA;
                break;
			 default: 
              hw_lcd_panel = LCD_NT35410_CHIMEI_HVGA;
				 break;
		 }
	 }

 	 if( machine_is_msm7x27_m650())
	 {
		 switch (lcd_id)
		 {
			 case 0:
				 hw_lcd_panel = LCD_HX8368A_TRULY_QVGA;
				 break;

			 default: 
				 hw_lcd_panel = LCD_HX8368A_TRULY_QVGA;
				 break;
		 }
	 }
	if(machine_is_msm7x27_c8800())
	{
		switch (lcd_id)
	    {
	        case 0:
				hw_lcd_panel = LCD_MDDI_NT35582_BYD_WVGA;
				break;
			case 1:
				hw_lcd_panel = LCD_MDDI_NT35582_TRULY_WVGA;
				break;				
			case 2:
				hw_lcd_panel = LCD_MDDI_NT35510_ALPHA_SI_WVGA;
				break;
	        default: 
				hw_lcd_panel = LCD_MDDI_NT35582_BYD_WVGA;
				break;
	    }
	}
	 if( machine_is_msm7x27_m865()||machine_is_msm7x27_u8650()||machine_is_msm7x27_c8650() || machine_is_msm7x27_c8651())
	 {
		 switch (lcd_id)
		 {
			 case 0: 
				 hw_lcd_panel = LCD_S6D74A0_SAMSUNG_HVGA;
				 break;
			 case 1:
				 hw_lcd_panel = LCD_ILI9481DS_TIANMA_HVGA;
				 break;
			 case 2:  
				 hw_lcd_panel = LCD_ILI9481D_INNOLUX_HVGA;
				 break;
			 default : 
				 hw_lcd_panel = LCD_S6D74A0_SAMSUNG_HVGA;
				 break; 
	 	}
	 }
	 return hw_lcd_panel;
}

/*===========================================================================


FUNCTION     lcd_align_probe

DESCRIPTION
  This function probe which LCD align type should be used

DEPENDENCIES
  
RETURN VALUE
  None

SIDE EFFECTS
  None
===========================================================================*/
lcd_align_type lcd_align_probe(void)
{
    lcd_panel_type  hw_lcd_panel = LCD_NONE;
    lcd_align_type  lcd_align    =  LCD_PANEL_ALIGN_LSB;
     
/* add C8650 branch */
/* M865 use the both LCD,so add it */
  if(machine_is_msm7x27_u8650() || machine_is_msm7x27_m865() || machine_is_msm7x27_c8650() || machine_is_msm7x27_c8651())
    {               
        hw_lcd_panel = lcd_panel_probe();
        if ((hw_lcd_panel == LCD_ILI9481DS_TIANMA_HVGA) ||(hw_lcd_panel == LCD_ILI9481D_INNOLUX_HVGA))
        {
            lcd_align = LCD_PANEL_ALIGN_MSB;
        }
        else
        {
            lcd_align = LCD_PANEL_ALIGN_LSB;
        }
    }
    return lcd_align;

}
/*return the string by compass position*/
char *get_compass_gs_position_name(void)
{
	compass_gs_position_type compass_gs_position=COMPASS_TOP_GS_TOP;
	char *position_name=NULL;

	compass_gs_position = get_compass_gs_position();

	switch(compass_gs_position)
	{
		case COMPASS_TOP_GS_TOP:
			 position_name = "COMPASS_TOP_GS_TOP";
			 break;
			 
		case COMPASS_TOP_GS_BOTTOM:
			 position_name = "COMPASS_TOP_GS_BOTTOM";
			 break;

		case COMPASS_BOTTOM_GS_TOP:
			 position_name = "COMPASS_BOTTOM_GS_TOP";
			 break;

		case COMPASS_BOTTOM_GS_BOTTOM:
			 position_name = "COMPASS_BOTTOM_GS_BOTTOM";
			 break;
			 
		case COMPASS_NONE_GS_BOTTOM:
			 position_name = "COMPASS_NONE_GS_BOTTOM";
			 break;

		default:
			 position_name = "COMPASS_TOP_GS_TOP";
			 break;
	}

	return position_name;
	
}

char *get_wifi_device_name(void)
{
	wifi_device_type wifi_device = WIFI_BROADCOM;
	char *device_name = "WIFI_NONE";
	wifi_device = get_wifi_device();

	if(wifi_device == WIFI_BROADCOM)
	{
		device_name = "WIFI_BROADCOM";
	}
	return device_name;
}


char *get_lcd_panel_name(void)
{
	 lcd_panel_type hw_lcd_panel = LCD_NONE;
	 char *pname = NULL;
	 
	 hw_lcd_panel = lcd_panel_probe();
	 
	 switch (hw_lcd_panel)
	 {
		 case LCD_S6D74A0_SAMSUNG_HVGA:
			 pname = "SAMSUNG S6D74A0";
			 break;
			 
		 case LCD_ILI9325_INNOLUX_QVGA:
			 pname = "INNOLUX ILI9325";
			 break;
		 case LCD_ILI9331B_TIANMA_QVGA:
			 pname = "TIANMA ILI9331B";
			 break;
		 case LCD_ILI9325_BYD_QVGA:
			 pname = "BYD ILI9325";
			 break;

		 case LCD_ILI9325_WINTEK_QVGA:
			 pname = "WINTEK ILI9325";
			 break;

		 case LCD_SPFD5408B_KGM_QVGA:
			 pname = "KGM SPFD5408B";
			 break;

		 case LCD_HX8368A_TRULY_QVGA:
			 pname = "TRULY HX8368A";
			 break;

		 case LCD_HX8368A_SEIKO_QVGA:
			 pname = "SEIKO HX8368A";
			 break;
		 case LCD_HX8347D_TRULY_QVGA:
			 pname = "TRULY HX8347D";
			 break;
		 case LCD_HX8347D_INNOLUX_QVGA:
			 pname = "INNOLUX HX8347D";
			 break;
		 case LCD_ILI9325C_WINTEK_QVGA:
			 pname = "WINTEK ILI9325C";
			 break;
		 case LCD_HX8357A_TRULY_HVGA:
			 pname = "TRULY HX8357A";
			 break;
		 case LCD_HX8357A_WINTEK_HVGA:
			 pname = "WIMTEK HX8357A";
			 break;
             
         case LCD_HX8357B_TIANMA_HVGA:
             pname = "TIANMA HX8357B";
             break;                        
		 case LCD_R61529_TRULY_HVGA:
			 pname = "TRULY R61529";
			 break;                        
        case LCD_S6D05A0_INNOLUX_HVGA:
            pname = "INNOLUX S6D05A0";
			 break;
		case LCD_MDDI_NT35582_BYD_WVGA:
			pname = "BYD NT35582";
			break;
		case LCD_MDDI_NT35582_TRULY_WVGA:
			pname = "TRULY NT35582";
			break;
		case LCD_MDDI_NT35510_ALPHA_SI_WVGA:
			pname =  "TRULY NT35510";
			break;
		case LCD_ILI9481DS_TIANMA_HVGA:
			pname = "TIANMA ILI9481";
			break;
		case LCD_ILI9481D_INNOLUX_HVGA:
			pname = "INNOLUX ILI9481";
			break;
        /*add an new lcd for U8510*/
        case LCD_NT35410_CHIMEI_HVGA:
            pname = "CHIMEI NT35410";
            break;
		 default:
			 pname = "UNKNOWN LCD";
			 break;
	 }

 return pname;
}

int board_surport_fingers(bool * is_surport_fingers)
{
	 int result = 0;

	 if (is_surport_fingers == NULL)
	 {
		  return -ENOMEM;
	 }
	 
	 *is_surport_fingers = false;

	 return result;
}

int board_use_tssc_touch(bool * use_touch_key)
{
	 int result = 0;

	 *use_touch_key = false;
	 return result;
}

int board_support_ofn(bool * ofn_support)
{
	 int ret = 0;

	 if(NULL == ofn_support)
	 {
		 return -EPERM;
	 }

	 *ofn_support = false;
	 return ret;
}

static bool camera_i2c_state = false;
bool camera_is_supported(void)
{
	 return camera_i2c_state;
}

void set_camera_support(bool status)
{
	 camera_i2c_state = status;
}
static bool st303_gs_state = false;

bool st303_gs_is_supported(void)
{
	 return st303_gs_state;
}

void set_st303_gs_support(bool status)
{
	 st303_gs_state = status;
}

/*
*  return: 0 ----not support RGB LED driver
* 		 1 ----support RGB LED driver
*/

/*M650 support rgb led */
bool rgb_led_is_supported(void)
{
	bool ret = false;
    /* c8800 support led */
	ret = (bool)(machine_is_msm7x27_m650()||machine_is_msm7x27_u8650()||machine_is_msm7x27_c8800()||
	            machine_is_msm7x27_m865()||machine_is_msm7x27_c8650() || machine_is_msm7x27_c8651());
	return ret;
}

/*M650 support qwerty keypad */
bool qwerty_is_supported(void)
{
	bool ret = false;
	ret = (bool)(machine_is_msm7x27_m650());
	return ret;
}

