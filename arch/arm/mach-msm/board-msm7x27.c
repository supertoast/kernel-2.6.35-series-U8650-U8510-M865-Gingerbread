/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2011, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
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
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/power_supply.h>


#include <mach/hardware.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/setup.h>
#ifdef CONFIG_CACHE_L2X0
#include <asm/hardware/cache-l2x0.h>
#endif

#include <asm/mach/mmc.h>
#include <mach/vreg.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/pmic.h>
#include <mach/msm_iomap.h>
#include <mach/msm_rpcrouter.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/rpc_pmapp.h>
#include <mach/msm_serial_hs.h>
#include <mach/memory.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/i2c.h>
#include <linux/android_pmem.h>
#include <mach/camera.h>

#include "devices.h"
#include "clock.h"
#include "msm-keypad-devices.h"
#include "linux/hardware_self_adapt.h"
#include "pm.h"
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_TM1319
#include <linux/synaptics_i2c_rmi.h>
#endif
/*add code to support JOY(U8120) */
#ifdef CONFIG_HUAWEI_FEATURE_OFN_KEY
#include "msm_ofn_key.h"
#endif

#ifdef CONFIG_HUAWEI_JOGBALL
#include "jogball_device.h"
#endif

#ifdef CONFIG_HUAWEI_WIFI_SDCC
#include <linux/wifi_tiwlan.h>
#include <linux/skbuff.h>
#endif

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
#include "gpio_hw.h"
#endif
#ifdef CONFIG_ARCH_MSM7X27
#include <linux/msm_kgsl.h>
#endif

#ifdef CONFIG_USB_ANDROID
#include <linux/usb/android_composite.h>
#endif

#ifdef CONFIG_USB_AUTO_INSTALL
#include "../../../drivers/usb/gadget/usb_switch_huawei.h"
#include "../../../arch/arm/mach-msm/proc_comm.h"
#include "smd_private.h"

#ifdef CONFIG_HUAWEI_KERNEL
#include <linux/touch_platform_config.h>
#endif
#ifdef CONFIG_HUAWEI_NFC_PN544
#include <linux/nfc/pn544.h>
#endif

#define USB_SERIAL_LEN 20
smem_huawei_vender usb_para_data;

static char buf_virtualkey[500];
static ssize_t  buf_vkey_size=0;


app_usb_para usb_para_info;
#ifdef CONFIG_HUAWEI_KERNEL
atomic_t touch_detected_yet = ATOMIC_INIT(0); 
#define MSM_7x27_TOUCH_INT       29
#define MSM_7x27_RESET_PIN 		 96
struct vreg *vreg_gp5 = NULL;
struct vreg *vreg_synt= NULL;
#endif



usb_pid_stru usb_pid_array[]={
    {PID_ONLY_CDROM,     PID_NORMAL,     PID_UDISK, PID_AUTH,     PID_GOOGLE, PID_WLAN}, /* for COMMON products */
    {PID_ONLY_CDROM_TMO, PID_NORMAL_TMO, PID_UDISK, PID_AUTH_TMO, PID_GOOGLE, PID_WLAN}, /* for TMO products */
};

usb_pid_stru *curr_usb_pid_ptr = &usb_pid_array[0];
#endif  


#ifdef CONFIG_ARCH_MSM7X25
#define MSM_PMEM_MDP_SIZE	0xb21000
#define MSM_PMEM_ADSP_SIZE	0x97b000
#define MSM_PMEM_AUDIO_SIZE	0x121000
#define MSM_FB_SIZE		0x200000
#define PMEM_KERNEL_EBI1_SIZE	0x64000
#endif
#define ATMEL_RMI_TS_IRQ       29
#define CPT_1044_TS_IRQ        29
#define CPT_1044_TS_I2C_ADR    0x72
#ifdef CONFIG_ARCH_MSM7X27

//#define MSM_PMEM_MDP_SIZE	0x1B76000
#ifdef CONFIG_HUAWEI_KERNEL
#define MSM_PMEM_MDP_WVGA_SIZE  0x1780000

#define MSM_PMEM_MDP_HVGA_SIZE  0x0C10000

#define MSM_PMEM_MDP_QVGA_SIZE  0x0910000    //last modfication is 0x0B10000
#endif
#define MSM_PMEM_MDP_SIZE	0x1310000     //last modfication is 0x1780000
#define MSM_PMEM_ADSP_SIZE	0xC00000   //last modfication is 0xB71000
#define MSM_PMEM_AUDIO_SIZE	0x5B000

#define LCD_FRAME_BUFF_END_ADDR	 0x1D500000ul
 

#define HUAWEI_CRASH_MEM_SIZE   (0) /* delete crash dump memory in 7x27*/
#define HUAWEI_SHARE_MEMORY_SIZE (1024*1024) /*1M cust partition*/

#ifdef CONFIG_MSM7X27_SURF_DEBUG
#define MSM_FB_SIZE		0x177000
#else
#define MSM_FB_SIZE		0x200000 //2M
#endif

#define PMEM_KERNEL_EBI1_SIZE	0x1C000
#endif
#define BATT_LOW        3400
#define BATT_LEVEL_MAX  100
#define BATT_LEVEL_MIN  0 
static DEFINE_MUTEX(lcdc_config);
char front_sensor_name[128];
char back_sensor_name[128];
 struct msm_camera_sensor_vreg sensor_vreg_array_c8651[] ;
compass_gs_position_type  get_compass_gs_position()
{
	compass_gs_position_type compass_gs_position=COMPASS_TOP_GS_TOP;
	
	if(machine_is_msm7x27_c8800())
	{
		compass_gs_position=COMPASS_TOP_GS_BOTTOM;
	}
	else if(machine_is_msm7x27_m650())
	{
		compass_gs_position=COMPASS_BOTTOM_GS_TOP;
	}
	else if(machine_is_msm7x27_u8650()||machine_is_msm7x27_m865())
	{
		compass_gs_position=COMPASS_BOTTOM_GS_BOTTOM;
	}
	else if(machine_is_msm7x27_c8650())
	{
		compass_gs_position=COMPASS_NONE_GS_BOTTOM;
	}	
	else
	{
		compass_gs_position=COMPASS_TOP_GS_TOP;
	}	
	return compass_gs_position;
}

wifi_device_type  get_wifi_device()
{
	wifi_device_type wifi_device = WIFI_BROADCOM;
	if(machine_is_msm7x27_u8650()||
	   machine_is_msm7x27_c8650()||
	   machine_is_msm7x27_c8800()||
	   machine_is_msm7x27_m865()||
	   machine_is_msm7x27_m650()||
	   machine_is_msm7x27_u8510_1()||
	   machine_is_msm7x27_c8651())
	{
		wifi_device = WIFI_BROADCOM;
	}
	return wifi_device;
}

static struct resource smc91x_resources[] = {
	[0] = {
		.start	= 0x9C004300,
		.end	= 0x9C0043ff,
		.flags	= IORESOURCE_MEM,
	},
	[1] = {
		.start	= MSM_GPIO_TO_INT(132),
		.end	= MSM_GPIO_TO_INT(132),
		.flags	= IORESOURCE_IRQ,
	},
};
#ifdef CONFIG_HUAWEI_BATTERY

static struct platform_device huawei_battery_device = {
	.name = "huawei_battery",
	.id		= -1,
};
#endif
#ifdef CONFIG_USB_FUNCTION
static struct usb_mass_storage_platform_data usb_mass_storage_pdata = {
	.nluns          = 0x02,
	.buf_size       = 16384,
	.vendor         = "GOOGLE",
	.product        = "Mass storage",
	.release        = 0xffff,
};

static struct platform_device mass_storage_device = {
	.name           = "usb_mass_storage",
	.id             = -1,
	.dev            = {
		.platform_data          = &usb_mass_storage_pdata,
	},
};
#endif
#ifdef CONFIG_USB_ANDROID
#ifdef CONFIG_USB_AUTO_INSTALL
static char *usb_functions_hw_normal_adb[] = {
	"modem",
	"nmea",
	"usb_mass_storage",
	"adb",
	"diag",
};

static char *usb_functions_hw_normal[] = {
	"modem",
	"nmea",
	"usb_mass_storage",
};

static char *usb_functions_hw_ms[] = {
	"usb_mass_storage",
};

static char *usb_functions_google_ms[] = {
	"usb_mass_storage",
};

static char *usb_functions_google_ms_adb[] = {
	"usb_mass_storage",
	"adb",	
};

static char *usb_functions_rndis[] = {
	"rndis", 
};

static char *usb_functions_all[] = {	
#ifdef CONFIG_USB_ANDROID_RNDIS
		"rndis",
#endif
#ifdef CONFIG_USB_F_SERIAL
	"modem",
	"nmea",
#endif
	"usb_mass_storage",
	"adb",
#ifdef CONFIG_USB_ANDROID_DIAG
	"diag",
#endif
#ifdef CONFIG_USB_ANDROID_RMNET
	"rmnet",
#endif
#ifdef CONFIG_USB_ANDROID_ACM
	"acm",
#endif
};


static struct android_usb_product_hw usb_products[] = {
    {
        .adb_product_id = PID_UDISK,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .adb_functions  = usb_functions_hw_ms,
        .product_id = PID_UDISK,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .functions  = usb_functions_hw_ms,
        .nluns = 2,
        .cdrom_index=-1,
    },
    {
        .adb_product_id = PID_ONLY_CDROM,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .adb_functions  = usb_functions_hw_ms,
        .product_id = PID_ONLY_CDROM,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .functions  = usb_functions_hw_ms,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_AUTH,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .adb_functions  = usb_functions_hw_normal_adb,
        .product_id = PID_AUTH,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .functions  = usb_functions_hw_normal_adb,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_NORMAL,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .adb_functions  = usb_functions_hw_normal_adb,
        .product_id = PID_NORMAL,
        .num_functions = ARRAY_SIZE(usb_functions_hw_normal),
        .functions  = usb_functions_hw_normal,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_ONLY_CDROM_TMO,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .adb_functions  = usb_functions_hw_ms,
        .product_id = PID_ONLY_CDROM_TMO,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_ms),
        .functions  = usb_functions_hw_ms,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_AUTH_TMO,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .adb_functions  = usb_functions_hw_normal_adb,
        .product_id = PID_AUTH_TMO,
        .num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .functions  = usb_functions_hw_normal_adb,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_NORMAL_TMO,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_hw_normal_adb),
        .adb_functions  = usb_functions_hw_normal_adb,
        .product_id = PID_NORMAL_TMO,
        .num_functions = ARRAY_SIZE(usb_functions_hw_normal),
        .functions  = usb_functions_hw_normal,
        .nluns = 1,
        .cdrom_index=0,
    },
    {
        .adb_product_id = PID_GOOGLE,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_google_ms_adb),
        .adb_functions  = usb_functions_google_ms_adb,
        .product_id = PID_GOOGLE_MS,
        .num_functions = ARRAY_SIZE(usb_functions_google_ms),
        .functions  = usb_functions_google_ms,
        .nluns = 3,
        .cdrom_index=1,
    },
    {
        .adb_product_id = PID_WLAN,
        .adb_num_functions  = ARRAY_SIZE(usb_functions_rndis),
        .adb_functions  = usb_functions_rndis,
        .product_id = PID_WLAN,
        .num_functions  = ARRAY_SIZE(usb_functions_rndis),
        .functions  = usb_functions_rndis,
        .nluns = 1,
        .cdrom_index=-1,
    },
};


static char product_name[MAX_NAME_LEN];
static char vendor_name[MAX_NAME_LEN];
static char manufacturer_name[MAX_NAME_LEN];
#define MAX_LENS 3
static struct usb_mass_storage_platform_data mass_storage_pdata = {
	.nluns		= MAX_LENS,
	.vendor 	= vendor_name,//"Qualcomm Incorporated",
	.product				= product_name,//"Mass storage",
	.release	= 0x0100,
};

static struct platform_device usb_mass_storage_device = {
	.name = "usb_mass_storage",
	.id = -1,
	.dev	= {
		.platform_data = &mass_storage_pdata,
	},
};

static struct usb_ether_platform_data rndis_pdata = {
	/* ethaddr is filled by board_serialno_setup */
	.vendorID = HUAWEI_VID,
	.vendorDescr	= manufacturer_name,
};

static struct platform_device rndis_device = {
	.name = "rndis",
	.id = -1,
	.dev	= {
		.platform_data = &rndis_pdata,
	},
};

static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id	= HUAWEI_VID,
	.product_id = 0x9026,
	.version	= 0x0100,
	.product_name 	= product_name,
	.manufacturer_name	= manufacturer_name,
	.num_products = ARRAY_SIZE(usb_products),
	.products = usb_products,
	.num_functions = ARRAY_SIZE(usb_functions_all),
	.functions = usb_functions_all,
	.serial_number = "1234567890ABCDEF",
};
#endif

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};

static int __init board_serialno_setup(char *serialno)
{
	int i;
	char *src = serialno;

	/* create a fake MAC address from our serial number.
	 * first byte is 0x02 to signify locally administered.
	 */
	rndis_pdata.ethaddr[0] = 0x02;
	for (i = 0; *src; i++) {
		/* XOR the USB serial across the remaining bytes */
		rndis_pdata.ethaddr[i % (ETH_ALEN - 1) + 1] ^= *src++;
	}

	android_usb_pdata.serial_number = serialno;
	return 1;
}
__setup("androidboot.serialno=", board_serialno_setup);
#endif

static struct platform_device smc91x_device = {
	.name		= "smc91x",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(smc91x_resources),
	.resource	= smc91x_resources,
};

#ifdef CONFIG_USB_FUNCTION
static struct usb_function_map usb_functions_map[] = {
	{"diag", 0},
	{"adb", 1},
	{"modem", 2},
	{"nmea", 3},
	{"mass_storage", 4},
	{"ethernet", 5},
	{"rmnet", 6},
};

/* dynamic composition */
static struct usb_composition usb_func_composition[] = {
	{
		.product_id         = 0x9012,
		.functions	    = 0x5, /* 0101 */
	},

	{
		.product_id         = 0x9013,
		.functions	    = 0x15, /* 10101 */
	},

	{
		.product_id         = 0x9014,
		.functions	    = 0x30, /* 110000 */
	},

	{
		.product_id         = 0x9016,
		.functions	    = 0xD, /* 01101 */
	},

	{
		.product_id         = 0x9017,
		.functions	    = 0x1D, /* 11101 */
	},

	{
		.product_id         = 0xF000,
		.functions	    = 0x10, /* 10000 */
	},

	{
		.product_id         = 0xF009,
		.functions	    = 0x20, /* 100000 */
	},

	{
		.product_id         = 0x9018,
		.functions	    = 0x1F, /* 011111 */
	},
#ifdef CONFIG_USB_FUNCTION_RMNET
	{
		.product_id         = 0x9021,
		/* DIAG + RMNET */
		.functions	    = 0x41,
	},
	{
		.product_id         = 0x9022,
		/* DIAG + ADB + RMNET */
		.functions	    = 0x43,
	},
#endif

};

static struct msm_hsusb_platform_data msm_hsusb_pdata = {
	.version	= 0x0100,
	.phy_info	= (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
	.vendor_id          = 0x5c6,
	.product_name       = "Qualcomm HSUSB Device",
	.serial_number      = "1234567890ABCDEF",
	.manufacturer_name  = "Qualcomm Incorporated",
	.compositions	= usb_func_composition,
	.num_compositions = ARRAY_SIZE(usb_func_composition),
	.function_map   = usb_functions_map,
	.num_functions	= ARRAY_SIZE(usb_functions_map),
	.config_gpio    = NULL,
};
#endif

#ifdef CONFIG_USB_EHCI_MSM
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	if (on)
		msm_hsusb_vbus_powerup();
	else
		msm_hsusb_vbus_shutdown();
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
	.phy_info       = (USB_PHY_INTEGRATED | USB_PHY_MODEL_65NM),
};

static void __init msm7x2x_init_host(void)
{
	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa())
		return;

	msm_add_host(0, &msm_usb_host_pdata);
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	if (init) {
		/*
		 * PHY 3.3V analog domain(VDDA33) is powered up by
		 * an always enabled power supply (LP5900TL-3.3).
		 * USB VREG default source is VBUS line. Turning
		 * on USB VREG has a side effect on the USB suspend
		 * current. Hence USB VREG is explicitly turned
		 * off here.
		 */
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_enable(vreg_3p3);
		vreg_disable(vreg_3p3);
		vreg_put(vreg_3p3);
	}

	return 0;
}

static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		ret = msm_pm_app_rpc_init(callback);
	} else {
		msm_pm_app_rpc_deinit(callback);
		ret = 0;
	}
	return ret;
}

static int msm_otg_rpc_phy_reset(void __iomem *regs)
{
	return msm_hsusb_phy_reset();
}

static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
#ifdef CONFIG_USB_EHCI_MSM
	.vbus_power = msm_hsusb_vbus_power,
#endif
	.ldo_init		= msm_hsusb_ldo_init,
	.pclk_required_during_lpm = 1,
	.pclk_src_name		= "ebi1_usb_clk",
};

#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata;
#endif
#endif

#define SND(desc, num) { .name = #desc, .id = num }
static struct snd_endpoint snd_endpoints_list[] = {
	SND(HANDSET, 0),
	SND(MONO_HEADSET, 2),
	SND(HEADSET, 3),
	SND(SPEAKER, 6),
	SND(TTY_HEADSET, 8),
	SND(TTY_VCO, 9),
	SND(TTY_HCO, 10),
	SND(BT, 12),
	SND(IN_S_SADC_OUT_HANDSET, 16),
	SND(IN_S_SADC_OUT_SPEAKER_PHONE, 25),
#ifdef CONFIG_HUAWEI_KERNEL
	SND(FM_HEADSET, 26),
	SND(FM_SPEAKER, 27),
	SND(BT_EC_OFF,  28),
	SND(HEADSET_AND_SPEAKER,29),
	SND(HANDSET_2NDMIC, 30),
	SND(CURRENT,    32),
#else
	SND(CURRENT, 27),
#endif
};
#undef SND

static struct msm_snd_endpoints msm_device_snd_endpoints = {
	.endpoints = snd_endpoints_list,
	.num = sizeof(snd_endpoints_list) / sizeof(struct snd_endpoint)
};

static struct platform_device msm_device_snd = {
	.name = "msm_snd",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_snd_endpoints
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#ifdef CONFIG_ARCH_MSM7X25
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP)| \
	(1<<MSM_ADSP_CODEC_MP3))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_WAV)|(1<<MSM_ADSP_CODEC_ADPCM)| \
	(1<<MSM_ADSP_CODEC_YADPCM)|(1<<MSM_ADSP_CODEC_QCELP)| \
	(1<<MSM_ADSP_CODEC_MP3))
#define DEC3_FORMAT 0
#define DEC4_FORMAT 0
#else
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)
#endif

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DMA)), 0,
	0, 0, 0,

	/* Concurrency 1 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	 /* Concurrency 2 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 3 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 4 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 5 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),

	/* Concurrency 6 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	0, 0, 0, 0,

	/* Concurrency 7 */
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC4_FORMAT),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
#ifdef CONFIG_ARCH_MSM7X25
	DEC_INFO("AUDPLAY1TASK", 14, 1, 5),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 5),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 0),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 0),  /* AudPlay4BitStreamCtrlQueue */
#else
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
#endif
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static struct android_pmem_platform_data android_pmem_kernel_ebi1_pdata = {
	.name = PMEM_KERNEL_EBI1_DATA_NAME,
	/* if no allocator_type, defaults to PMEM_ALLOCATORTYPE_BITMAP,
	 * the only valid choice at this time. The board structure is
	 * set to all zeros by the C runtime initialization and that is now
	 * the enum value of PMEM_ALLOCATORTYPE_BITMAP, now forced to 0 in
	 * include/linux/android_pmem.h.
	 */
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
	.name = "pmem_adsp",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
	.name = "pmem_audio",
	.allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

static struct platform_device android_pmem_adsp_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
	.name = "android_pmem",
	.id = 2,
	.dev = { .platform_data = &android_pmem_audio_pdata },
};

static struct platform_device android_pmem_kernel_ebi1_device = {
	.name = "android_pmem",
	.id = 4,
	.dev = { .platform_data = &android_pmem_kernel_ebi1_pdata },
};

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(87, 5, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(86, 3, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(85, 3, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(84, 4, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
	{ .gpio_cfg = TSIF_B_SYNC, .label =  "tsif_sync", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_clk = "tsif_clk",
	.tsif_pclk = "tsif_pclk",
	.tsif_ref_clk = "tsif_ref_clk",
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

#define LCDC_CONFIG_PROC          21
#define LCDC_UN_CONFIG_PROC       22
#define LCDC_API_PROG             0x30000066
#define LCDC_API_VERS             0x00010001

#define GPIO_OUT_132    132
#define GPIO_OUT_131    131
#define GPIO_OUT_103    103
#define GPIO_OUT_102    102
#ifndef CONFIG_MSM7X27_SURF_DEBUG
#define GPIO_OUT_101    101 
#else
#define GPIO_OUT_88     88
#endif

static struct msm_rpc_endpoint *lcdc_ep;

static int msm_fb_lcdc_config(int on)
{
	int rc = 0;
	struct rpc_request_hdr hdr;
	mutex_lock(&lcdc_config);
	if (on)
		pr_info("lcdc config\n");
	else
		pr_info("lcdc un-config\n");

	lcdc_ep = msm_rpc_connect_compatible(LCDC_API_PROG, LCDC_API_VERS, 0);
	if (IS_ERR(lcdc_ep)) {
		printk(KERN_ERR "%s: msm_rpc_connect failed! rc = %ld\n",
			__func__, PTR_ERR(lcdc_ep));
		mutex_unlock(&lcdc_config);
		return -EINVAL;
	}

	rc = msm_rpc_call(lcdc_ep,
				(on) ? LCDC_CONFIG_PROC : LCDC_UN_CONFIG_PROC,
				&hdr, sizeof(hdr),
				5 * HZ);
	if (rc)
		printk(KERN_ERR
			"%s: msm_rpc_call failed! rc = %d\n", __func__, rc);

	msm_rpc_close(lcdc_ep);
	mutex_unlock(&lcdc_config);
	return rc;
}

#ifdef CONFIG_MSM7X27_SURF_DEBUG
static int gpio_array_num[] = {
				GPIO_OUT_132, /* spi_clk */
				GPIO_OUT_131, /* spi_cs  */
				GPIO_OUT_103, /* spi_sdi */
				GPIO_OUT_102, /* spi_sdoi */
				GPIO_OUT_88
				};
#else
static int gpio_array_num[] = {
    GPIO_OUT_131, /* spi_clk */
    GPIO_OUT_132, /* spi_cs  */
    GPIO_OUT_103, /* spi_sdi */
    GPIO_OUT_101, /* spi_sdoi */
    GPIO_OUT_102  /* LCD reset*/
				};
#endif

#ifndef CONFIG_MSM7X27_SURF_DEBUG
static void lcdc_gordon_gpio_init(void)
{
	if (gpio_request(GPIO_OUT_131, "spi_clk"))
		pr_err("failed to request gpio spi_clk\n");
	if (gpio_request(GPIO_OUT_132, "spi_cs"))
		pr_err("failed to request gpio spi_cs\n");
	if (gpio_request(GPIO_OUT_103, "spi_sdi"))
		pr_err("failed to request gpio spi_sdi\n");
	if (gpio_request(GPIO_OUT_101, "spi_sdoi"))
		pr_err("failed to request gpio spi_sdoi\n");
	if (gpio_request(GPIO_OUT_102, "gpio_dac"))
		pr_err("failed to request gpio_dac\n");
}
#else
static void lcdc_gordon_gpio_init(void)
{
	if (gpio_request(GPIO_OUT_132, "spi_clk"))
		pr_err("failed to request gpio spi_clk\n");
	if (gpio_request(GPIO_OUT_131, "spi_cs"))
		pr_err("failed to request gpio spi_cs\n");
	if (gpio_request(GPIO_OUT_103, "spi_sdi"))
		pr_err("failed to request gpio spi_sdi\n");
	if (gpio_request(GPIO_OUT_102, "spi_sdoi"))
		pr_err("failed to request gpio spi_sdoi\n");
	if (gpio_request(GPIO_OUT_88, "gpio_dac"))
		pr_err("failed to request gpio_dac\n");
}
#endif

#ifndef CONFIG_MSM7X27_SURF_DEBUG
static uint32_t lcdc_gpio_table[] = {
	GPIO_CFG(GPIO_OUT_131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_103, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_101, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_102, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
#else
static uint32_t lcdc_gpio_table[] = {
	GPIO_CFG(GPIO_OUT_132, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_131, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_103, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_102, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	GPIO_CFG(GPIO_OUT_88,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
};
#endif

static void config_lcdc_gpio_table(uint32_t *table, int len, unsigned enable)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n],
			enable ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static void lcdc_gordon_config_gpios(int enable)
{
	config_lcdc_gpio_table(lcdc_gpio_table,
		ARRAY_SIZE(lcdc_gpio_table), enable);
}

#ifdef CONFIG_MSM7X27_SURF_DEBUG
static char *msm_fb_lcdc_vreg[] = {
	"gp5"
};

static int msm_fb_lcdc_power_save(int on)
{
	struct vreg *vreg[ARRAY_SIZE(msm_fb_lcdc_vreg)];
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(msm_fb_lcdc_vreg); i++) {
		if (on) {
			vreg[i] = vreg_get(0, msm_fb_lcdc_vreg[i]);
			rc = vreg_enable(vreg[i]);
			if (rc) {
				printk(KERN_ERR "vreg_enable: %s vreg"
						"operation failed \n",
						msm_fb_lcdc_vreg[i]);
				goto bail;
			}
		} else {
			int tmp;
			vreg[i] = vreg_get(0, msm_fb_lcdc_vreg[i]);
			tmp = vreg_disable(vreg[i]);
			if (tmp) {
				printk(KERN_ERR "vreg_disable: %s vreg "
						"operation failed \n",
						msm_fb_lcdc_vreg[i]);
				if (!rc)
					rc = tmp;
			}
			tmp = gpio_tlmm_config(GPIO_CFG(GPIO_OUT_88, 0,
						GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA), GPIO_CFG_ENABLE);
			if (tmp) {
				printk(KERN_ERR "gpio_tlmm_config failed\n");
				if (!rc)
					rc = tmp;
			}
			gpio_set_value(88, 0);
			mdelay(15);
			gpio_set_value(88, 1);
			mdelay(15);
		}
	}

	return rc;

bail:
	if (on) {
		for (; i > 0; i--)
			vreg_disable(vreg[i - 1]);
	}

	return rc;
}
#else
static struct msm_gpio lcd_panel_gpios[] = {
	{ GPIO_CFG(98, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red5" },
	{ GPIO_CFG(99, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red6" },
	{ GPIO_CFG(100, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red7" },
	{ GPIO_CFG(111, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn1" },
	{ GPIO_CFG(112, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn0" },
	{ GPIO_CFG(113, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu7" },
	{ GPIO_CFG(114, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu6" },
	{ GPIO_CFG(115, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu5" },
	{ GPIO_CFG(116, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu4" },
	{ GPIO_CFG(117, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu3" },
	{ GPIO_CFG(118, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu2" },
	{ GPIO_CFG(119, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn4" },
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn3" },
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn2" },
	{ GPIO_CFG(125, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu5" },
	{ GPIO_CFG(126, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu6" },
};
static int msm_fb_lcdc_power_save(int on)
{
	int rc;
	if (on)
    {
		rc = msm_gpios_enable(lcd_panel_gpios, ARRAY_SIZE(lcd_panel_gpios));
		if (rc < 0) 
        {
			printk(KERN_ERR "%s: gpio config failed: %d\n",
				__func__, rc);
		}
	} 
    else
    {
		msm_gpios_disable(lcd_panel_gpios, ARRAY_SIZE(lcd_panel_gpios));
    }

}

#endif
static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_gpio_config = msm_fb_lcdc_config,
	.lcdc_power_save   = msm_fb_lcdc_power_save,
};

static struct msm_panel_common_pdata lcdc_gordon_panel_data = {
	.panel_config_gpio = lcdc_gordon_config_gpios,
	.gpio_num          = gpio_array_num,
};

static struct platform_device lcdc_gordon_panel_device = {
	.name   = "lcdc_gordon_vga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_gordon_panel_data,
	}
};
static struct platform_device lcdc_ili9325_panel_device = 
{
	.name   = "lcdc_ili9325_qvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_gordon_panel_data,
	}
};
static struct platform_device lcdc_ili9331b_panel_device = 
{
    .name = "lcd_ili9331b_qvga",
    .id   = 0,
    .dev  = {
        .platform_data = &lcdc_gordon_panel_data,
    }
};

static struct platform_device lcdc_s6d74a0_panel_device = 
{
	.name   = "lcdc_s6d74a0_hvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_gordon_panel_data,
	}
};


static struct platform_device lcdc_spfd5408b_panel_device = 
{
	.name   = "lcdc_spfd08b_qvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_gordon_panel_data,
	}
};


/*add hx8357a LCD for U8300*/
static struct platform_device lcdc_hx8357a_panel_device = 
{
	.name   = "lcdc_hx8357a_qvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_gordon_panel_data,
	}
};
/*add hx8357a hvga LCD for U8500*/
static struct platform_device lcdc_hx8357a_hvga_panel_device = 
{
	.name   = "lcdc_hx8357a_hvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_gordon_panel_data,
	}
};
static struct platform_device lcdc_hx8357b_panel_device = 
{
    .name   = "lcdc_hx8357b_hvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_gordon_panel_data,
    }
};
static struct platform_device lcdc_truly_r61529_panel_device = 
{
    .name   = "lcdc_truly_r61529_hvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_gordon_panel_data,
    }
};

/* U8300 need to support the HX8368a ic driver of TRULY LCD */
static struct platform_device lcdc_hx8368a_panel_device = 
{
	.name   = "lcdc_hx8368a_qvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_gordon_panel_data,
	}
};

static struct platform_device lcdc_hx8347d_panel_device = 
{
	.name   = "lcdc_hx8347d_qvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_gordon_panel_data,
	}
};

static struct platform_device lcdc_ili9325c_panel_device = 
{
	.name   = "lcd_ili9325c_qvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_gordon_panel_data,
	}
};

/* add s6d05a0 hvga LCD for U8500 */
static struct platform_device lcdc_s6d05a0_panel_device = 
{
    .name   = "lcdc_s6d05a0_hvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_gordon_panel_data,
    }
};
/*add an new lcd for U8510*/
static struct platform_device lcdc_nt35410_panel_device = 
{
    .name   = "lcdc_nt35410_hvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_gordon_panel_data,
    }
};
static struct platform_device lcdc_ili9481_panel_device = 
{
    .name   = "lcdc_ili9481_hvga",
    .id     = 0,
    .dev    = {
        .platform_data = &lcdc_gordon_panel_data,
    }
};

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	int ret = -EPERM;

	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		if (!strcmp(name, "lcdc_gordon_vga"))
			ret = 0;
		else
			ret = -ENODEV;
	}

	return ret;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct resource huawei_share_memory_resources[] = {
	{
		.flags  = IORESOURCE_MEM,
	}
};

static struct platform_device huawei_share_memory_device = {
	.name   	= "hw_share_mem",
	.id     	= 0,
	.num_resources  = ARRAY_SIZE(huawei_share_memory_resources),
	.resource       = huawei_share_memory_resources,
};

static struct platform_device rgb_leds_device = {
	.name   = "rgb-leds",
	.id     = 0,
};
#ifdef CONFIG_BT
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
};

enum {
	BT_WAKE,
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
	BT_PCM_DOUT,
	BT_PCM_DIN,
	BT_PCM_SYNC,
	BT_PCM_CLK,
	BT_HOST_WAKE,
};

static unsigned bt_config_power_on[] = {
	GPIO_CFG(42, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* WAKE */
	GPIO_CFG(43, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* RFR */
	GPIO_CFG(44, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* CTS */
	GPIO_CFG(45, 2, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* Rx */
	GPIO_CFG(46, 3, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* Tx */
#ifndef CONFIG_HUAWEI_KERNEL
	GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),	/* PCM_CLK */
#else 
    GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_DOUT */
    GPIO_CFG(69, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_DIN */
    GPIO_CFG(70, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_SYNC */
    GPIO_CFG(71, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_CLK */
    GPIO_CFG(88, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
    GPIO_CFG(109, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* BT_REG_ON */
#endif
    GPIO_CFG(83, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* HOST_WAKE */

};
static unsigned bt_config_power_off[] = {
	GPIO_CFG(42, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* WAKE */
#ifndef CONFIG_HUAWEI_KERNEL
	GPIO_CFG(43, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* RFR */
	GPIO_CFG(44, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* CTS */
	GPIO_CFG(45, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* Rx */
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* Tx */
#else
    GPIO_CFG(43, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),  /* RFR */
    GPIO_CFG(44, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),  /* CTS */
    GPIO_CFG(45, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),  /* Rx */
    GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),  /* Tx */
    GPIO_CFG(88, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* reset */
    GPIO_CFG(109, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* BT_REG_ON */
#endif
	GPIO_CFG(68, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCM_DOUT */
	GPIO_CFG(69, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCM_DIN */
	GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCM_SYNC */
	GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* PCM_CLK */
	GPIO_CFG(83, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* HOST_WAKE */
};

static int bluetooth_power(int on)
{
#ifndef CONFIG_HUAWEI_KERNEL
  struct vreg *vreg_bt;
#endif
	int pin, rc;

	printk(KERN_DEBUG "%s\n", __func__);
#ifndef CONFIG_HUAWEI_KERNEL
	/* do not have vreg bt defined, gp6 is the same */
	/* vreg_get parameter 1 (struct device *) is ignored */
	vreg_bt = vreg_get(NULL, "gp6");

	if (IS_ERR(vreg_bt)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_bt));
		return PTR_ERR(vreg_bt);
	}
#endif

	if (on) {
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
				return -EIO;
			}
		}

#ifndef CONFIG_HUAWEI_KERNEL
		/* units of mV, steps of 50 mV */
		rc = vreg_set_level(vreg_bt, 2600);
		if (rc) {
			printk(KERN_ERR "%s: vreg set level failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
		rc = vreg_enable(vreg_bt);
		if (rc) {
			printk(KERN_ERR "%s: vreg enable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
#else   
        printk(KERN_ERR "bt power on");
        rc = gpio_direction_output(109, 1);  /*bt on :109 -->1*/
        if (rc) 
        {
            printk(KERN_ERR "%s: generation wifi power (%d)\n",
                   __func__, rc);
            return -EIO;
        }
		mdelay(1);

        rc = gpio_direction_output(88, 1);  /*bton:88 -->1*/
		if (rc) {
			printk(KERN_ERR "%s: generation BTS4020 main clock is failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
#endif
        
	} else {
#ifndef CONFIG_HUAWEI_KERNEL
		rc = vreg_disable(vreg_bt);
		if (rc) {
			printk(KERN_ERR "%s: vreg disable failed (%d)\n",
			       __func__, rc);
			return -EIO;
		}
#else
        rc = gpio_direction_output(88, 0);  /*bt off:88 -->0*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power off fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }

        rc = gpio_direction_output(109, 0);  /*bt_reg_on off on :109 -->0*/
        if (rc) 
        {
            printk(KERN_ERR "%s:  bt power off fail (%d)\n",
                   __func__, rc);
            return -EIO;
        }
#endif
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],
					      GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_off[pin], rc);
				return -EIO;
			}
		}
	}
	return 0;
}

static void __init bt_power_init(void)
{
    int pin = 0, rc = 0;

	msm_bt_power_device.dev.platform_data = &bluetooth_power;
    
	if (gpio_request(88, "BT_POWER"))		
  		  printk("Failed to request gpio 88 for BT_POWER\n");	
    
	if (gpio_request(109, "BT_REG_ON"))		
  		  printk("Failed to request gpio 109 for BT_REG_ON\n");	

	for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
		rc = gpio_tlmm_config(bt_config_power_on[pin],
				      GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
			       "%s: gpio_tlmm_config(%#x)=%d\n",
			       __func__, bt_config_power_on[pin], rc);
		}
	}

    rc = gpio_direction_output(88, 0);  /*bt off:88 -->0*/
    if (rc) 
    {
        printk(KERN_ERR "%s:  bt power off fail (%d)\n",
               __func__, rc);
    }

    rc = gpio_direction_output(109, 0);  /*bt_reg_on off on :109 -->0*/
    if (rc) 
    {
        printk(KERN_ERR "%s:  bt power off fail (%d)\n",
               __func__, rc);
    }

	for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
		rc = gpio_tlmm_config(bt_config_power_off[pin],
				      GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
			       "%s: gpio_tlmm_config(%#x)=%d\n",
			       __func__, bt_config_power_off[pin], rc);
		}
	}
}
#else
#define bt_power_init(x) do {} while (0)
#endif

#ifdef CONFIG_ARCH_MSM7X27
static struct resource kgsl_resources[] = {
	{
		.name = "kgsl_reg_memory",
		.start = 0xA0000000,
		.end = 0xA001ffff,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "kgsl_yamato_irq",
		.start = INT_GRAPHICS,
		.end = INT_GRAPHICS,
		.flags = IORESOURCE_IRQ,
	},
};

static struct kgsl_platform_data kgsl_pdata;

static struct platform_device msm_device_kgsl = {
	.name = "kgsl",
	.id = -1,
	.num_resources = ARRAY_SIZE(kgsl_resources),
	.resource = kgsl_resources,
	.dev = {
		.platform_data = &kgsl_pdata,
	},
};
#endif

static struct platform_device msm_device_pmic_leds = {
	.name   = "pmic-leds",
	.id = -1,
};

static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 83,
		.end	= 83,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 42,
		.end	= 42,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(83),
		.end	= MSM_GPIO_TO_INT(83),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};

#ifdef CONFIG_HUAWEI_KERNEL
/*
 *the fucntion touch_power used to contrl the tp's power
 */

int touch_power(int on)
{
    int rc_gp5 = 0;
    int value = 0;
    int rc_synt = 0;
    if(machine_is_msm7x27_c8651())
    {
      	if (on)
	{
          vreg_synt= vreg_get(NULL,"synt");
          if (IS_ERR(vreg_synt)) 
          {
            pr_err("%s:gp5 power init get failed\n", __func__);
	     return rc_synt;
          }
          rc_synt=vreg_set_level(vreg_synt, 2850);
          if(rc_synt)
          {
            pr_err("%s:gp5 power init faild\n",__func__);
            return rc_synt;
          }
          rc_synt=vreg_enable(vreg_synt);
         if (rc_synt) 
          {
            pr_err("%s:gp5 power init failed \n", __func__);
	   }
        
        mdelay(50);     
  
	}
	else
	{
	  if(NULL != vreg_synt)
	  {
	    rc_synt = vreg_disable(vreg_synt);
	    if (rc_synt)
           {
               pr_err("%s:gp5 power disable failed \n", __func__);
           }
          }
	}
	return rc_synt;
    }
    if(machine_is_msm7x27_u8510())
    {
        value = 2700;
    }
    else
    {
        value = 2800;
    }
	if (on)
	{
        vreg_gp5 = vreg_get(NULL,"gp5");
        if (IS_ERR(vreg_gp5)) 
        {
		    pr_err("%s:gp5 power init get failed\n", __func__);
            goto err_power_fail;
        }
        rc_gp5=vreg_set_level(vreg_gp5, value);
        if(rc_gp5)
        {
            pr_err("%s:gp5 power init faild\n",__func__);
            goto err_power_fail;
        }
        rc_gp5=vreg_enable(vreg_gp5);
        if (rc_gp5) 
        {
		    pr_err("%s:gp5 power init failed \n", __func__);
	    }
        
        mdelay(50);     
  
	}
	else
	{
		if(NULL != vreg_gp5)
		{
           rc_gp5 = vreg_disable(vreg_gp5);
           if (rc_gp5)
           {
               pr_err("%s:gp5 power disable failed \n", __func__);
           }
        }
	}
err_power_fail:
	return rc_gp5;
}

/*
 *use the touch_gpio_config_interrupt to config the gpio
 *which we used, but the gpio number can't exposure to user
 *so when the platform or the product changged please self self adapt
 */
 
int touch_gpio_config_interrupt(void)
{
	int gpio_config = 0;
    int ret = 0;
    gpio_config = GPIO_CFG(MSM_7x27_TOUCH_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
    ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
    return ret;    
}
/*
 *the fucntion set_touch_probe_flag when the probe is detected use this function can set the flag ture
 */

void set_touch_probe_flag(int detected)/*we use this to detect the probe is detected*/
{
    if(detected >= 0)
    {
    	atomic_set(&touch_detected_yet, 1);
    }
    else
    {
    	atomic_set(&touch_detected_yet, 0);
    }
    return;
}

/*
 *the fucntion read_touch_probe_flag when the probe is ready to detect first we read the flag 
 *if the flag is set ture we will break the probe else we 
 *will run the probe fucntion
 */

int read_touch_probe_flag(void)
{
    int ret = 0;
    ret = atomic_read(&touch_detected_yet);
    return ret;
}

/*this function reset touch panel */
int touch_reset(void)
{
    int ret = 0;

	gpio_request(MSM_7x27_RESET_PIN,"TOUCH_RESET");
	ret = gpio_tlmm_config(GPIO_CFG(MSM_7x27_RESET_PIN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	ret = gpio_direction_output(MSM_7x27_RESET_PIN, 1);
	mdelay(5);
	ret = gpio_direction_output(MSM_7x27_RESET_PIN, 0);
	mdelay(10);                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
	ret = gpio_direction_output(MSM_7x27_RESET_PIN, 1);
	mdelay(50);//must more than 10ms.

	return ret;
}

/*this function return reset gpio at 7x27 platform */
int get_touch_reset_pin()
{
	int ret = MSM_7x27_RESET_PIN;
	return ret;
}

/*this function get the tp  resolution*/
static int get_phone_version(struct tp_resolution_conversion *tp_resolution_type)
{
    int ret = 0;
    if(machine_is_msm7x27_c8800())
    { 
        tp_resolution_type->lcd_x = LCD_X_WVGA;
        tp_resolution_type->lcd_y = LCD_Y_WVGA;
        tp_resolution_type->jisuan = LCD_JS_WVGA;
    }
	else if(machine_is_msm7x27_m650())
	{
        tp_resolution_type->lcd_x = LCD_X_QVGA;
        tp_resolution_type->lcd_y = LCD_Y_QVGA;
        tp_resolution_type->jisuan = LCD_Y_QVGA;
	    ret = touch_reset();
		if(ret)
		{
			printk(KERN_ERR "%s: reset failed \n", __func__);
            return -1;
		}
	}
	else if(machine_is_msm7x27_u8650() || machine_is_msm7x27_c8650() || machine_is_msm7x27_m865() || machine_is_msm7x27_c8651())
	{
        tp_resolution_type->lcd_x = LCD_X_HVGA;
        tp_resolution_type->lcd_y = LCD_Y_HVGA;
        tp_resolution_type->jisuan = 531; //this number is for 865
		
		ret = touch_reset();
		if(ret)
		{
			printk(KERN_ERR "%s: reset failed \n", __func__);
            return -1;
		}
	}
    else
    {
        tp_resolution_type->lcd_x = LCD_X_HVGA;
        tp_resolution_type->lcd_y = LCD_Y_HVGA;   
        tp_resolution_type->jisuan = LCD_JS_HVGA;
    }
    return 1;
}

static struct touch_hw_platform_data touch_hw_data = {
    .touch_power = touch_power,
    .touch_gpio_config_interrupt = touch_gpio_config_interrupt,
    .set_touch_probe_flag = set_touch_probe_flag,
    .read_touch_probe_flag = read_touch_probe_flag,
    .touch_reset = touch_reset,
    .get_touch_reset_pin = get_touch_reset_pin,
    .get_phone_version = get_phone_version,
};

#ifdef CONFIG_HUAWEI_NFC_PN544
/* this function is used to reset pn544 by controlling the ven pin */
static int pn544_ven_reset(void)
{
	int ret=0;
	int gpio_config=0;
	ret = gpio_request(GPIO_NFC_VEN, "gpio 85 for NFC pn544");
	
	gpio_config = GPIO_CFG(GPIO_NFC_VEN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	
	ret = gpio_direction_output(GPIO_NFC_VEN,0);
	mdelay(10);
	gpio_set_value(GPIO_NFC_VEN, 1);
	mdelay(5);
	return 0;
}

static int pn544_interrupt_gpio_config(void)
{
	int ret=0;
	int gpio_config=0;
	gpio_config = GPIO_CFG(GPIO_NFC_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA);
	ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
	ret = gpio_request(GPIO_NFC_INT, "gpio 84 for NFC pn544");
	ret = gpio_direction_input(GPIO_NFC_INT);
	return 0;
}


static struct pn544_nfc_platform_data pn544_hw_data = 
{
	.pn544_ven_reset = pn544_ven_reset,
	.pn544_interrupt_gpio_config = pn544_interrupt_gpio_config,
};

#endif
#endif

/* add platform data and interrupt control for aps, */
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_9900
int aps9900_gpio_config_interrupt(void)
{
    int gpio_config = 0;
    int ret = 0;
    
    gpio_config = GPIO_CFG(MSM_7X30_APS9900_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA);
    ret = gpio_tlmm_config(gpio_config, GPIO_CFG_ENABLE);
    return ret; 
}

static struct aps9900_hw_platform_data aps9900_hw_data = {
    .aps9900_gpio_config_interrupt = aps9900_gpio_config_interrupt,
};
#endif
static int gsensor_support_dummyaddr(void)
{
    int ret = -1;	/*default value means actual address*/

    if( machine_is_msm7x27_u8510()||machine_is_msm7x27_u8510_1())
    {
		ret = (int)GS_ST303DLH;
    }
    return ret;
}
static int gsensor_support_dummyaddr_adi346(void)
{
    int ret = -1;	/*default value means actual address*/

    ret = (int)GS_ADI346;

    return ret;
}
			
static int gs_init_flag = 0;   /*gsensor is not initialized*/

#ifdef CONFIG_ACCELEROMETER_ST_L1S35DE
static struct gs_platform_data gs_st_platform_data = {
    .adapt_fn = gsensor_support_dummyaddr,
    .slave_addr = (0x38 >> 1),  /*i2c slave address*/
    .dev_id = NULL,        /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=NULL,
};
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_MMA8452
static struct gs_platform_data gs_mma8452_platform_data = {
    .adapt_fn = NULL,
    .slave_addr = (0x38 >> 1),  /*i2c slave address*/
    .dev_id = 0x2A,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_BOSCH_BMA250
static struct gs_platform_data gs_bma250_platform_data = {
    .adapt_fn = NULL,
    .slave_addr = (0x32 >> 1),  /*i2c slave address*/
    .dev_id = 0x03,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ST_LSM303DLH
static struct gs_platform_data st303_gs_platform_data = {
    .adapt_fn = gsensor_support_dummyaddr,
    .slave_addr = (0x32 >> 1),  /*i2c slave address*/
    .dev_id = NULL,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=NULL,
};
#endif

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ST_LIS3XH
static struct gs_platform_data gs_st_lis3xh_platform_data = {
    .adapt_fn = NULL,
    .slave_addr = (0x30 >> 1),  /*i2c slave address*/
    .dev_id = NULL,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ADI_ADXL346
static struct gs_platform_data gs_adi346_platform_data = {
    .adapt_fn = gsensor_support_dummyaddr_adi346,
    .slave_addr = (0xA6 >> 1),  /*i2c slave address*/
    .dev_id = NULL,    /*WHO AM I*/
    .init_flag = &gs_init_flag,
    .get_compass_gs_position=get_compass_gs_position,
};
#endif 
static struct i2c_board_info i2c_devices[] = {
#ifdef CONFIG_HUAWEI_CAMERA
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1GX
	{
		I2C_BOARD_INFO("s5k4e1gx", 0x6E >> 1),
	},
#endif 
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9P017
    {
        I2C_BOARD_INFO("mt9p017", 0x6D),//i2c slave address is addr[7:1], addr[0] is read/write bit.
    },
#endif
#ifdef CONFIG_MT9T013
	{
		I2C_BOARD_INFO("mt9t013_liteon", 0x6C >> 1),
	},
	{
		I2C_BOARD_INFO("mt9t013_byd", 0x6C),
	},
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
	{
		I2C_BOARD_INFO("ov3647", 0x90 >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV7690
	{
		I2C_BOARD_INFO("ov7690", 0x42 >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_OV7690
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_HIMAX0356
	{
		I2C_BOARD_INFO("himax0356", 0x68 >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_OV7690
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9D113
	{
		I2C_BOARD_INFO("mt9d113", 0x78 >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_MT9D113
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_S5K5CA
	{
		I2C_BOARD_INFO("s5k5ca", 0x5a >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_S5K5CA
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_S5K4CDGX
	{
		I2C_BOARD_INFO("s5k4cdgx", 0xac >> 1),
	},
#endif
#endif //CONFIG_HUAWEI_CAMERA
#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_TM1319
    {
            I2C_BOARD_INFO(SYNAPTICS_I2C_RMI_NAME, 0x20),                
            .irq = MSM_GPIO_TO_INT(29)  /*gpio 20 is interupt for touchscreen.*/
    },
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI_TM
    {
            I2C_BOARD_INFO("synaptics-tm", 0x24),                
            .irq = MSM_GPIO_TO_INT(29)  /*gpio 29 is interupt for touchscreen.*/
    },
#endif
#ifndef CONFIG_MELFAS_RESTORE_FIRMWARE
#ifdef CONFIG_ACCELEROMETER_ADXL345
    {
        I2C_BOARD_INFO("GS", 0xA6 >> 1),	  
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },	
#endif

#ifdef CONFIG_ACCELEROMETER_ST_L1S35DE
    {
        I2C_BOARD_INFO("gs_st", 0x70 >> 1),  // actual address 0x38, fake address (0x38 << 1)
        .platform_data = &gs_st_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },	
  
    {
        I2C_BOARD_INFO("gs_st", 0x3A >> 1),	  
       .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },	

#endif
    
#ifdef CONFIG_ACCELEROMETER_MMA7455L
    {
        I2C_BOARD_INFO("freescale", 0x38 >> 1),	  
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
#endif	

#ifdef CONFIG_SENSORS_AKM8973
    {
        I2C_BOARD_INFO("akm8973", 0x3c >> 1),//7 bit addr, no write bit
        .irq = MSM_GPIO_TO_INT(107)
    },
#endif 
#endif
#ifdef CONFIG_MOUSE_OFN_AVAGO_A320
	{
		I2C_BOARD_INFO("avago_OFN", 0x33),
        .irq = MSM_GPIO_TO_INT(37)
	},
#endif	

#ifdef CONFIG_QWERTY_KEYPAD_ADP5587
	{
		I2C_BOARD_INFO("adp5587", 0x6a >> 1),// actual address 0x68, fake address 0x6a
		.irq = MSM_GPIO_TO_INT(39)
	},
#endif 

#ifdef CONFIG_TOUCHSCREEN_MELFAS
	{
		I2C_BOARD_INFO("melfas-ts", 0x22),                
        .platform_data = &touch_hw_data,
        .irq = MSM_GPIO_TO_INT(MSM_7x27_TOUCH_INT),
        .flags = true, //this flags is the switch of the muti_touch 
	},	
#endif

#ifdef CONFIG_TOUCHSCREEN_CYPRESS
	{
		I2C_BOARD_INFO("cypress-ts", 0x23),                
		.platform_data = &touch_hw_data,
		.irq = MSM_GPIO_TO_INT(MSM_7x27_TOUCH_INT),
		.flags = true, //this flags is the switch of the muti_touch 
	},	
#endif


#ifndef CONFIG_MELFAS_RESTORE_FIRMWARE
#ifdef CONFIG_PROXIMITY_EVERLIGHT_APS_12D
  {
    I2C_BOARD_INFO("aps-12d", 0x88 >> 1),
  },
#endif


#endif
#ifdef CONFIG_HUAWEI_NFC_PN544
	{
		I2C_BOARD_INFO(PN544_DRIVER_NAME, PN544_I2C_ADDR),
		.irq = MSM_GPIO_TO_INT(GPIO_NFC_INT),
		.platform_data = &pn544_hw_data,
	},
#endif
#ifdef CONFIG_HUAWEI_FEATURE_RMI_TOUCH
#if 0
	{	
		I2C_BOARD_INFO("Synaptics_rmi", 0x24), //M650 T1 use this addr
		.platform_data = &touch_hw_data,
		.irq = MSM_GPIO_TO_INT(MSM_7x27_TOUCH_INT),
		.flags = true, //this flags is the switch of the muti_touch 
	}, 
#endif
	{   
		I2C_BOARD_INFO("Synaptics_rmi", 0x70),
        .platform_data = &touch_hw_data,
        .irq = MSM_GPIO_TO_INT(MSM_7x27_TOUCH_INT),
        .flags = true, //this flags is the switch of the muti_touch 
	},
#endif
#ifdef CONFIG_CYPRESS_1044_TS
	{   
		I2C_BOARD_INFO("cpt_1044_ts", CPT_1044_TS_I2C_ADR),
        .irq = MSM_GPIO_TO_INT(CPT_1044_TS_IRQ),
        .flags = true,
	},
#endif
#ifdef CONFIG_HUAWEI_FEATURE_AT42QT_TS
	{   
		I2C_BOARD_INFO("atmel-rmi-ts", 0x4a),
        .platform_data = &touch_hw_data,
        .irq = MSM_GPIO_TO_INT(MSM_7x27_TOUCH_INT),
        .flags = true, //this flags is the switch of the muti_touch 
	},
#endif

#ifdef CONFIG_TOUCHSCREEN_SYNAPTICS_T1021
	{   
		I2C_BOARD_INFO("Synaptics_rmi", 0x24),
        .platform_data = &touch_hw_data,
        .irq = MSM_GPIO_TO_INT(MSM_7x27_TOUCH_INT),
        .flags = true, //this flags is the switch of the muti_touch 
	},
#endif

	#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ST_LSM303DLH
	{
		I2C_BOARD_INFO("st303_gs", 0x64 >> 1),         
		.platform_data = &st303_gs_platform_data,
		//.irq = MSM_GPIO_TO_INT() 
	},
	{
		I2C_BOARD_INFO("st303_compass", 0x3e >> 1),/* actual i2c address is 0x3c    */             
		//.irq = MSM_GPIO_TO_INT() 
	},
	#endif
	#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_MMA8452
    {
        I2C_BOARD_INFO("gs_mma8452", 0x38 >> 1),
        .platform_data = &gs_mma8452_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
#endif	

#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_BOSCH_BMA250
    {
        I2C_BOARD_INFO("gs_bma250", 0x32 >> 1),
        .platform_data = &gs_bma250_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
#endif	
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_AK8975
    {
        I2C_BOARD_INFO("akm8975", 0x18 >> 1),//7 bit addr, no write bit
        .irq = MSM_GPIO_TO_INT(107)
    },
#endif 
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ST_LIS3XH
    {
        I2C_BOARD_INFO("gs_st_lis3xh", 0x30 >> 1),
	 .platform_data = &gs_st_lis3xh_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
#endif
#ifdef CONFIG_HUAWEI_FEATURE_SENSORS_ACCELEROMETER_ADI_ADXL346
    {
        I2C_BOARD_INFO("gs_adi346", 0xA8>>1),  /* actual address 0xA6, fake address 0xA8*/
	 .platform_data = &gs_adi346_platform_data,
        .irq = MSM_GPIO_TO_INT(19)    //MEMS_INT1
    },
#endif 
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_HI253
	{
		I2C_BOARD_INFO("hi253", 0x20),// on 7x25 platform ,actual address 0x20, fake address 0x40
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_HI253
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9T113
	{
		I2C_BOARD_INFO("mt9t113", 0x7A >> 1),
	},
#endif //CONFIG_HUAWEI_CAMERA_SENSOR_MT9T113
#ifdef CONFIG_HUAWEI_FEATURE_PROXIMITY_EVERLIGHT_APS_9900
	{   
		I2C_BOARD_INFO("aps-9900", 0x72 >> 1), //the actual address is 0x39
        .irq = MSM_GPIO_TO_INT(MSM_7X30_APS9900_INT),
        .platform_data = &aps9900_hw_data,
	},
#endif
};
static struct i2c_board_info i2c_aux2_devices[] = {
#ifdef CONFIG_HUAWEI_FEATURE_QWERTY_KEYPAD_ADP5587
	{
		I2C_BOARD_INFO("adp5587", 0x68 >> 1),
		.irq = MSM_GPIO_TO_INT(40)
	},
#endif 
};

#ifdef CONFIG_MSM_CAMERA
static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT0 */
	GPIO_CFG(1,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT1 */
	GPIO_CFG(2,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT0 */
	GPIO_CFG(1,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT1 */
	GPIO_CFG(2,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_16MA), /* MCLK */
#ifndef CONFIG_MSM7X27_SURF_DEBUG
    GPIO_CFG(92, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),/*CAMIF_SHDN_INS */
    GPIO_CFG(90, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA),/*CAMIF_SHDN_OUTS */
    GPIO_CFG(89, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* reset */
    GPIO_CFG(91, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* vcm */
#endif
	};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

static struct vreg *vreg_gp2;
static struct vreg *vreg_gp3;

static void msm_camera_vreg_config(int vreg_en)
{
	int rc;

	if (vreg_gp2 == NULL) {
		vreg_gp2 = vreg_get(NULL, "gp2");
		if (IS_ERR(vreg_gp2)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp2", PTR_ERR(vreg_gp2));
			return;
		}

		rc = vreg_set_level(vreg_gp2, 1800);
		if (rc) {
			printk(KERN_ERR "%s: GP2 set_level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_gp3 == NULL) {
		vreg_gp3 = vreg_get(NULL, "gp3");
		if (IS_ERR(vreg_gp3)) {
			printk(KERN_ERR "%s: vreg_get(%s) failed (%ld)\n",
				__func__, "gp3", PTR_ERR(vreg_gp3));
			return;
		}

		rc = vreg_set_level(vreg_gp3, 2850);
		if (rc) {
			printk(KERN_ERR "%s: GP3 set level failed (%d)\n",
				__func__, rc);
		}
	}

	if (vreg_en) {
		rc = vreg_enable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 enable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_enable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 enable failed (%d)\n",
				__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_gp2);
		if (rc) {
			printk(KERN_ERR "%s: GP2 disable failed (%d)\n",
				 __func__, rc);
		}

		rc = vreg_disable(vreg_gp3);
		if (rc) {
			printk(KERN_ERR "%s: GP3 disable failed (%d)\n",
				__func__, rc);
		}
	}
}

static int config_camera_on_gpios(void)
{
	int vreg_en = 1;

	if (machine_is_msm7x25_ffa() ||
	    machine_is_msm7x27_ffa())
		msm_camera_vreg_config(vreg_en);

	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
	return 0;
}

static void config_camera_off_gpios(void)
{
	int vreg_en = 0;

	if (machine_is_msm7x25_ffa() ||
	    machine_is_msm7x27_ffa())
		msm_camera_vreg_config(vreg_en);

	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

#ifdef CONFIG_HUAWEI_CAMERA
static int32_t sensor_vreg_enable
(
    struct msm_camera_sensor_vreg *sensor_vreg,
    uint8_t vreg_num
)

{
    struct vreg *vreg_handle;
    uint8_t temp_vreg_sum;
    int32_t rc;
   if(machine_is_msm7x27_c8651())
   {
        sensor_vreg = sensor_vreg_array_c8651;
   }
    if(sensor_vreg == NULL)
    {
        return 0;
    }
    
    for(temp_vreg_sum = 0; temp_vreg_sum < vreg_num;temp_vreg_sum++)
    {
        vreg_handle = vreg_get(0, sensor_vreg[temp_vreg_sum].vreg_name);
    	if (!vreg_handle) {
    		printk(KERN_ERR "vreg_handle get failed\n");
    		return -EIO;
    	}
    	rc = vreg_set_level(vreg_handle, sensor_vreg[temp_vreg_sum].mv);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle set level failed\n");
    		return -EIO;
    	}
    	rc = vreg_enable(vreg_handle);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle enable failed\n");
    		return -EIO;
    	}
	mdelay(1);

    }
    return 0;
}

static int32_t sensor_vreg_disable(
       struct msm_camera_sensor_vreg *sensor_vreg,
       uint8_t vreg_num)
{
    struct vreg *vreg_handle;
    uint8_t temp_vreg_sum;
    int32_t rc;
    
    if(sensor_vreg == NULL)
    {
        return 0;
    }

    for(temp_vreg_sum = 0; temp_vreg_sum < vreg_num;temp_vreg_sum++)
    {
    
		/*power on camera can reduce i2c error. */
        /* c8800 should shutdown 3 power source */
        /* C8800 also power on camera for camera go to standby mode */
        if( sensor_vreg[temp_vreg_sum].always_on)
        {
            continue;
        }
		
        vreg_handle = vreg_get(0, sensor_vreg[temp_vreg_sum].vreg_name);
    	if (!vreg_handle) {
    		printk(KERN_ERR "vreg_handle get failed\n");
    		return -EIO;
    	}
    	rc = vreg_disable(vreg_handle);
    	if (rc) {
    		printk(KERN_ERR "vreg_handle disable failed\n");
    		return -EIO;
    	}
    }
    return 0;
}
 struct msm_camera_sensor_vreg sensor_vreg_array[] = {
    {
		.vreg_name   = "gp1",
		.mv	  = 2600,
		.always_on = 1,
	},    
    {
		.vreg_name   = "gp2",
		.mv	  = 1800,
		.always_on = 1,
	},
    {
		.vreg_name   = "gp6",
		.mv	  = 2800,
		.always_on = 1,
	}, 
    {
		.vreg_name   = "boost",
		.mv	  = 5000,
		.always_on = 0,
	},     
};
 struct msm_camera_sensor_vreg sensor_vreg_array_c8651[] = {
    {
		.vreg_name   = "gp1",
		.mv	  = 1800,
		.always_on = 1,
	},    
    {
		.vreg_name   = "gp5",
		.mv	  = 1800,
		.always_on = 1,
	},
    {
		.vreg_name   = "gp6",
		.mv	  = 2850,
		.always_on = 1,
	}, 
    {
		.vreg_name   = "boost",
		.mv	  = 5000,
		.always_on = 0,
	},     
};
static bool board_support_flash(void)
{
	//add the product of c8650/u8650/m865/m650
    if(machine_is_msm7x27_u8510()||machine_is_msm7x27_u8510_1()||machine_is_msm7x27_u8650()||
        machine_is_msm7x27_c8650()||machine_is_msm7x27_m865()||machine_is_msm7x27_m650())
    {
       return false;
    }

    return true;
}
#endif //CONFIG_HUAWEI_CAMERA
static struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.mdcphy = MSM_MDC_PHYS,
	.ioext.mdcsz  = MSM_MDC_SIZE,
	.ioext.appphy = MSM_CLK_CTL_PHYS,
	.ioext.appsz  = MSM_CLK_CTL_SIZE,
#ifdef CONFIG_HUAWEI_CAMERA 	
	.get_board_support_flash = board_support_flash,
#endif
};

int pmic_set_flash_led_current(enum pmic8058_leds id, unsigned mA)
{
	int rc;
	rc = pmic_flash_led_set_current(mA);
	return rc;
}

static struct msm_camera_sensor_flash_src msm_flash_src = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PMIC,
	._fsrc.pmic_src.num_of_src = 1,
	._fsrc.pmic_src.low_current  = 30,
	._fsrc.pmic_src.high_current = 100,
	._fsrc.pmic_src.led_src_1 = 0,
	._fsrc.pmic_src.led_src_2 = 0,
	._fsrc.pmic_src.pmic_set_current = pmic_set_flash_led_current,
};

#ifdef CONFIG_MT9D112
static struct msm_camera_sensor_flash_data flash_mt9d112 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9d112_data = {
	.sensor_name    = "mt9d112",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_mt9d112
};

static struct platform_device msm_camera_sensor_mt9d112 = {
	.name      = "msm_camera_mt9d112",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9d112_data,
	},
};
#endif

#ifdef CONFIG_S5K3E2FX
static struct msm_camera_sensor_flash_data flash_s5k3e2fx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_s5k3e2fx
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_flash_data flash_mt9p012 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name    = "mt9p012",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 88,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_mt9p012
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name      = "msm_camera_mt9p012",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

#ifdef CONFIG_MT9P012_KM
static struct msm_camera_sensor_flash_data flash_mt9p012_km = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_km_data = {
	.sensor_name    = "mt9p012_km",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 88,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_mt9p012_km
};

static struct platform_device msm_camera_sensor_mt9p012_km = {
	.name      = "msm_camera_mt9p012_km",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_km_data,
	},
};
#endif

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_flash_data flash_mt9t013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 89,
	.sensor_pwd     = 85,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_mt9t013
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name      = "msm_camera_mt9t013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif

#ifdef CONFIG_VB6801
static struct msm_camera_sensor_flash_data flash_vb6801 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_vb6801_data = {
	.sensor_name    = "vb6801",
	.sensor_reset   = 89,
	.sensor_pwd     = 88,
	.vcm_pwd        = 0,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_vb6801
};

static struct platform_device msm_camera_sensor_vb6801 = {
	.name      = "msm_camera_vb6801",
	.dev       = {
		.platform_data = &msm_camera_sensor_vb6801_data,
	},
};
#endif
#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_flash_data flash_mt9t013_byd= {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_byd_data = {
	.sensor_name	= (char *)back_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 90,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
	.flash_data     = &flash_mt9t013_byd,
        .flash_type		= MSM_CAMERA_FLASH_LED,
        .sensor_module_id  = 30,
        .sensor_module_value  = 1,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
    //    .master_init_control_slave = sensor_master_init_control_slave, 
};

static struct platform_device msm_camera_sensor_mt9t013_byd = {
	.name	   = "msm_camera_byd3m",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9t013_byd_data,
	},
};
#endif

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_flash_data flash_mt9t013_liteon= {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_liteon_data = {
	.sensor_name	= (char *)back_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 90,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
	.flash_data     = &flash_mt9t013_liteon,
        .flash_type		= MSM_CAMERA_FLASH_LED,
        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
    //    .master_init_control_slave = sensor_master_init_control_slave,
};

static struct platform_device msm_camera_sensor_mt9t013_liteon = {
	.name	   = "msm_camera_liteon3m",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9t013_liteon_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1GX
static struct msm_camera_sensor_flash_data flash_s5k4e1gx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k4e1gx_data = {
	.sensor_name    = "23060049-SAM",
	.sensor_reset   = 89,
	.sensor_pwd     = 90,
	.vcm_pwd        = 91,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_s5k4e1gx,
    .flash_type		= MSM_CAMERA_FLASH_LED,
       
    .sensor_module_id  = 30,
    .sensor_module_value  = 1,
	.sensor_vreg  = sensor_vreg_array,
	.vreg_num     = ARRAY_SIZE(sensor_vreg_array),
	.vreg_enable_func = sensor_vreg_enable,//msm_camera_vreg_config_on,
	.vreg_disable_func = sensor_vreg_disable,//msm_camera_vreg_config_off,
	.slave_sensor = 0,

//	.vcm_enable     = 0,

//	.resource       = msm_camera_resources,

//	.num_resources  = ARRAY_SIZE(msm_camera_resources)
};

static struct platform_device msm_camera_sensor_s5k4e1gx = {
	.name = "msm_camera_s5k4e1gx",
	.dev = {
		.platform_data = &msm_camera_sensor_s5k4e1gx_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9P017
static struct msm_camera_sensor_flash_data flash_mt9p017 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p017_data = {
	.sensor_name    = "mt9p017",
	.sensor_reset   = 89,
	.sensor_pwd     = 90,
	.vcm_pwd        = 91,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_mt9p017,

    .sensor_module_id  = 30,
    .sensor_module_value  = 0,
    .sensor_vreg  = sensor_vreg_array,
    .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
    .vreg_enable_func = sensor_vreg_enable,
    .vreg_disable_func = sensor_vreg_disable,
    .slave_sensor = 0, 		
};

static struct platform_device msm_camera_sensor_mt9p017 = {
	.name      = "msm_camera_mt9p017",
	.id        = -1,
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p017_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9D113
static struct msm_camera_sensor_info msm_camera_sensor_mt9d113_data = {
	.sensor_name	= (char *)back_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 90,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
        .flash_type		= MSM_CAMERA_FLASH_LED,

        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
    //    .master_init_control_slave = sensor_master_init_control_slave,

};

static struct platform_device msm_camera_sensor_mt9d113 = {
	.name	   = "msm_camera_mt9d113",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9d113_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9T113
static struct msm_camera_sensor_info msm_camera_sensor_mt9t113_data = {
	.sensor_name	= (char *)back_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 90,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
        .flash_type		= MSM_CAMERA_FLASH_LED,

        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
    //    .master_init_control_slave = sensor_master_init_control_slave,

};

static struct platform_device msm_camera_sensor_mt9t113 = {
	.name	   = "msm_camera_mt9t113",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_mt9t113_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_S5K5CA
static struct msm_camera_sensor_flash_data flash_s5k5ca= {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_s5k5ca_data = {
	.sensor_name	= (char *)back_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 90,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
        .flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_data     = &flash_s5k5ca,
        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
    //    .master_init_control_slave = sensor_master_init_control_slave,

};
static struct platform_device msm_camera_sensor_s5k5ca = {
	.name	   = "msm_camera_s5k5ca",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_s5k5ca_data,
	},
};
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_S5K4CDGX
static struct msm_camera_sensor_flash_data flash_s5k4cdgx= {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_s5k4cdgx_data = {
	.sensor_name	= (char *)back_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 90,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
        .flash_type		= MSM_CAMERA_FLASH_LED,
	.flash_data     = &flash_s5k4cdgx,
        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
    //    .master_init_control_slave = sensor_master_init_control_slave,

};
static struct platform_device msm_camera_sensor_s5k4cdgx = {
	.name	   = "msm_camera_s5k4cdgx",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_s5k4cdgx_data,
	},
};
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
static struct msm_camera_sensor_flash_data flash_ov3647= {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src
};
static struct msm_camera_sensor_info msm_camera_sensor_ov3647_data = {
	.sensor_name	= (char *)back_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 90,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
	.flash_data     = &flash_ov3647,	
        .flash_type		= MSM_CAMERA_FLASH_LED,

        .sensor_module_id  = 30,
        .sensor_module_value  = 0,
        .sensor_vreg  = sensor_vreg_array,
        .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
        .vreg_enable_func = sensor_vreg_enable,
        .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 0,
    //    .master_init_control_slave = sensor_master_init_control_slave,

};

static struct platform_device msm_camera_sensor_ov3647 = {
	.name	   = "msm_camera_ov3647",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_ov3647_data,
	},
};
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_HI253
static struct msm_camera_sensor_info msm_camera_sensor_hi253_data = {
	.sensor_name	= (char *)back_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 90,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_LED,

    .sensor_module_id  = 30,
    .sensor_module_value  = 0,
    .sensor_vreg  = sensor_vreg_array,
    .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
    .vreg_enable_func = sensor_vreg_enable,
    .vreg_disable_func = sensor_vreg_disable,
    .slave_sensor = 0,
    //    .master_init_control_slave = sensor_master_init_control_slave,
};

static struct platform_device msm_camera_sensor_hi253 = {
	.name	   = "msm_camera_hi253",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_hi253_data,
	},
};
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV7690	
static struct msm_camera_sensor_info msm_camera_sensor_ov7690_data = {
	.sensor_name	= (char *)front_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 92,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_LED,

    .sensor_module_id  = 30,
    .sensor_module_value  = 0,
    .sensor_vreg  = sensor_vreg_array,
    .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
    .vreg_enable_func = sensor_vreg_enable,
    .vreg_disable_func = sensor_vreg_disable,
        .slave_sensor = 1,
    //    .master_init_control_slave = sensor_master_init_control_slave,
};

static struct platform_device msm_camera_sensor_ov7690 = {
	.name	   = "msm_camera_ov7690",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_ov7690_data,
	},
};
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_HIMAX0356	
static struct msm_camera_sensor_info msm_camera_sensor_himax0356_data = {
	.sensor_name	= (char *)front_sensor_name,
	.sensor_reset	= 89,
	.sensor_pwd	= 92,
	.vcm_pwd	= 91,
	.pdata		= &msm_camera_device_data,
    .flash_type		= MSM_CAMERA_FLASH_LED,

    .sensor_module_id  = 30,
    .sensor_module_value  = 0,
    .sensor_vreg  = sensor_vreg_array,
    .vreg_num     = ARRAY_SIZE(sensor_vreg_array),
    .vreg_enable_func = sensor_vreg_enable,
    .vreg_disable_func = sensor_vreg_disable,
    .slave_sensor = 1,
    //    .master_init_control_slave = sensor_master_init_control_slave,
};

static struct platform_device msm_camera_sensor_himax0356 = {
	.name	   = "msm_camera_himax0356",
	.id        = -1,
	.dev	    = {
		.platform_data = &msm_camera_sensor_himax0356_data,
	},
};
#endif 
#endif
static u32 msm_calculate_batt_capacity(u32 current_voltage);
static struct msm_psy_batt_pdata msm_psy_batt_data = {
#ifndef CONFIG_HUAWEI_KERNEL
	.voltage_min_design 	= 2800,
#else
    .voltage_min_design     = BATT_LOW,
#endif
	.voltage_max_design	= 4300,
	.avail_chg_sources   	= AC_CHG | USB_CHG ,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
	.calculate_capacity	= &msm_calculate_batt_capacity,
};

static u32 msm_calculate_batt_capacity(u32 current_voltage)
{
	u32 low_voltage   = msm_psy_batt_data.voltage_min_design;
	u32 high_voltage  = msm_psy_batt_data.voltage_max_design;
#ifdef CONFIG_HUAWEI_KERNEL
/* to confirm the battery level will not overflow */
    if(current_voltage <= low_voltage)
    {
        return BATT_LEVEL_MIN;
    }
    else if(current_voltage >= high_voltage)
    {
        return BATT_LEVEL_MAX;
    }
    else
    {
#endif
	return (current_voltage - low_voltage) * 100
		/ (high_voltage - low_voltage);
#ifdef CONFIG_HUAWEI_KERNEL
    }
#endif
}
static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};


static struct platform_device huawei_serial_device = {
	.name = "hw_ss_driver",
	.id		= -1,
};

static struct platform_device huawei_device_detect = {
	.name = "hw-dev-detect",
	.id		= -1,
};

static struct platform_device *devices[] __initdata = {
	&msm_device_smd,
	&msm_device_dmov,
	&msm_device_nand,

#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif

#ifdef CONFIG_USB_FUNCTION
	&msm_device_hsusb_peripheral,
	&mass_storage_device,
#endif

#ifdef CONFIG_USB_ANDROID
	&usb_mass_storage_device,
	&rndis_device,
	&android_usb_device,
#endif
	&msm_device_i2c,
	&i2c_gpio_huawei,
	&smc91x_device,
/* remove default touchscreen of qualcomm */
/* use the macro to remove the ts device */
#ifndef CONFIG_HUAWEI_KERNEL
	&msm_device_tssc,
#endif
	&android_pmem_kernel_ebi1_device,
	&android_pmem_device,
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_fb_device,
#ifdef CONFIG_MSM7X27_SURF_DEBUG
	&lcdc_gordon_panel_device,
#else
    &lcdc_ili9325_panel_device,
    &lcdc_ili9331b_panel_device,
    &lcdc_s6d74a0_panel_device,
    &lcdc_spfd5408b_panel_device,
    &lcdc_hx8357a_panel_device,
    &lcdc_hx8368a_panel_device,
    &lcdc_hx8347d_panel_device,
    &lcdc_ili9325c_panel_device,
    &lcdc_hx8357a_hvga_panel_device,
    &lcdc_s6d05a0_panel_device,
	&lcdc_ili9481_panel_device,
	&lcdc_truly_r61529_panel_device,
    &lcdc_nt35410_panel_device,


#endif
	&msm_device_uart_dm1,
#ifdef CONFIG_BT
	&msm_bt_power_device,
#endif
	&msm_device_pmic_leds,
	&msm_device_snd,
	&msm_device_adspdec,
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#ifdef CONFIG_MT9P012_KM
	&msm_camera_sensor_mt9p012_km,
#endif
#ifdef CONFIG_VB6801
	&msm_camera_sensor_vb6801,
#endif
	&msm_bluesleep_device,
#ifdef CONFIG_ARCH_MSM7X27
	&msm_device_kgsl,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
	&hs_device,
	&msm_batt_device,
    &lcdc_hx8357b_panel_device,
#ifdef CONFIG_HUAWEI_BATTERY

	&huawei_battery_device,
#endif
	&huawei_serial_device,
    &huawei_device_detect,
#ifdef CONFIG_HUAWEI_RGB_KEY_LIGHT
	&rgb_leds_device,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013_byd,
#endif
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013_liteon,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV3647
	&msm_camera_sensor_ov3647,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_OV7690
	&msm_camera_sensor_ov7690,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_HIMAX0356	
	&msm_camera_sensor_himax0356,
#endif 
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9D113
   &msm_camera_sensor_mt9d113,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9T113
   &msm_camera_sensor_mt9t113,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_S5K5CA
   &msm_camera_sensor_s5k5ca,
#endif

#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_S5K4CDGX
   &msm_camera_sensor_s5k4cdgx,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_HI253	
    &msm_camera_sensor_hi253,
#endif
#ifdef CONFIG_HUAWEI_SENSOR_S5K4E1GX
	&msm_camera_sensor_s5k4e1gx,
#endif
#ifdef CONFIG_HUAWEI_CAMERA_SENSOR_MT9P017	
    &msm_camera_sensor_mt9p017,
#endif

	&huawei_share_memory_device,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.gpio = 97,
};

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", 0);
	msm_fb_register_device("lcdc", &lcdc_pdata);
}

extern struct sys_timer msm_timer;

static void __init msm7x2x_init_irq(void)
{
	msm_init_irq();
}

static struct msm_acpu_clock_platform_data msm7x2x_clock_data = {
	.acpu_switch_time_us = 50,
	.max_speed_delta_khz = 400000,
	.vdd_switch_time_us = 62,
	.max_axi_khz = 160000,
};

void msm_serial_debug_init(unsigned int base, int irq,
			   struct device *clk_device, int signal_irq);

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

static unsigned long vreg_sts, gpio_sts;
static struct vreg *vreg_mmc;
static unsigned mpp_mmc = 2;

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};

static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(51, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(53, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(54, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
	{GPIO_CFG(55, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(56, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc1_clk"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(62, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc2_clk"},
	{GPIO_CFG(63, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(64, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(65, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(66, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(67, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},
};

static struct msm_gpio sdc2_sleep_cfg_data[] = {
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc2_clk"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc2_cmd"},
	{GPIO_CFG(64, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc2_dat_3"},
	{GPIO_CFG(65, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc2_dat_2"},
	{GPIO_CFG(66, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc2_dat_1"},
	{GPIO_CFG(67, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "sdc2_dat_0"},
};
static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(88, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_clk"},
	{GPIO_CFG(89, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(90, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(91, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(92, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(93, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(19, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(20, 3, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(21, 4, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(107, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(108, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_0"},
	{GPIO_CFG(109, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_clk"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = sdc2_sleep_cfg_data,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = NULL,
	},
};

static void msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];
	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
			return;
		}
		msm_gpios_disable_free(curr->cfg_data, curr->size);
	}
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	msm_sdcc_setup_gpio(pdev->id, !!vdd);

	if (vdd == 0) {
		if (!vreg_sts)
			return 0;

		clear_bit(pdev->id, &vreg_sts);

		if (!vreg_sts) {
			if (machine_is_msm7x25_ffa() ||
					machine_is_msm7x27_ffa()) {
				rc = mpp_config_digital_out(mpp_mmc,
				     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
				     MPP_DLOGIC_OUT_CTRL_LOW));
			} else
				rc = vreg_disable(vreg_mmc);
			if (rc)
				printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
		}
		return 0;
	}

	if (!vreg_sts) {
		if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
			rc = mpp_config_digital_out(mpp_mmc,
			     MPP_CFG(MPP_DLOGIC_LVL_MSMP,
			     MPP_DLOGIC_OUT_CTRL_HIGH));
		} else {
			rc = vreg_set_level(vreg_mmc, 2850);
			if (!rc)
				rc = vreg_enable(vreg_mmc);
		}
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	set_bit(pdev->id, &vreg_sts);
	return 0;
}

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm7x27_sdcc_slot_status(struct device *dev)
{
	if((unsigned int) gpio_get_value(MSM_GPIO_SDMC_CD_N))
	{
	        return 0;
	}
	return 1;
}
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static struct mmc_platform_data msm7x2x_sdc1_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x27_sdcc_slot_status,
	.status_irq  = MSM_GPIO_TO_INT(MSM_GPIO_SDMC_CD_N),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};

static struct mmc_platform_data msm7x2x_sdc1_data_no_detect = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};

#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static struct mmc_platform_data msm7x2x_sdc2_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	 //.sdiowakeup_irq = MSM_GPIO_TO_INT(66),
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
static struct mmc_platform_data msm7x2x_sdc3_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x2x_sdc4_data = {
	.ocr_mask	= MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static void sdio_wakeup_gpiocfg_slot2(void)
{
	gpio_request(66, "sdio_wakeup");
	gpio_direction_output(66, 1);
	gpio_free(66);
}
#endif
#ifdef CONFIG_HUAWEI_WIFI_SDCC

#define PREALLOC_WLAN_NUMBER_OF_SECTIONS	4
#define PREALLOC_WLAN_NUMBER_OF_BUFFERS		160
#define PREALLOC_WLAN_SECTION_HEADER		24

#define WLAN_SECTION_SIZE_0	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_1	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 128)
#define WLAN_SECTION_SIZE_2	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 512)
#define WLAN_SECTION_SIZE_3	(PREALLOC_WLAN_NUMBER_OF_BUFFERS * 1024)

#define WLAN_SKB_BUF_NUM	16

static unsigned wlan_wakes_msm[] = {
    GPIO_CFG(17, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA) };

/* for wifi power supply */
static unsigned wifi_config_power_on[] = {
    GPIO_CFG(108, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA) };/* WL_REG_ON */
  
#if 0
static unsigned wifi_config_power_off[] = {
    GPIO_CFG(108, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA) };/* WL_REG_ON */
#endif
	   
static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

typedef struct wifi_mem_prealloc_struct {
	void *mem_ptr;
	unsigned long size;
} wifi_mem_prealloc_t;

static wifi_mem_prealloc_t wifi_mem_array[PREALLOC_WLAN_NUMBER_OF_SECTIONS] = {
	{ NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER) },
	{ NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER) }
};

static void *bcm_wifi_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_NUMBER_OF_SECTIONS)
		return wlan_static_skb;
	if ((section < 0) || (section > PREALLOC_WLAN_NUMBER_OF_SECTIONS))
		return NULL;
	if (wifi_mem_array[section].size < size)
		return NULL;
	return wifi_mem_array[section].mem_ptr;
}

static int bcm_wifi_set_power(int enable)
{
	int ret = 0;

   	if (enable)
	{
	        #if 0
	        ret = gpio_tlmm_config(wifi_config_power_on[0], GPIO_CFG_ENABLE);		
		 if (ret) {
				printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, wifi_config_power_on[0], ret);
				return -EIO;
			}
	        #endif
	        ret = gpio_direction_output(108, 1);  /*wifi on :108 -->1*/
	        if (ret) 
	        {
	            printk(KERN_ERR "%s: WL_REG_ON  failed to pull up (%d)\n",
	                   __func__, ret);
	            return -EIO;
                    }
		
		mdelay(1);
		if(machine_is_msm7x27_c8651())
		{
		  ret = gpio_direction_output(36, 1);  /*wifi reset :36 -->1*/
		  if (ret) 
		  {
		    printk(KERN_ERR "%s: WL_REG_ON  failed to pull up (%d)\n",
				__func__, ret);
		    return -EIO;
		  }
		}
		else
		{
		  // WLAN chip to reset
		  ret = mpp_config_digital_out(20,
				MPP_CFG(MPP_DLOGIC_LVL_MSMP, MPP_DLOGIC_OUT_CTRL_HIGH));  
		  if (ret) 
		  {
		    printk(KERN_ERR "%s: WL_RST_N  failed to pull up(%d)\n",
					__func__, ret);
		    return -EIO;
		  }
		}

		mdelay(150);
		printk(KERN_ERR "%s: wifi power successed to pull up\n",__func__);
		
	}else{

		ret = gpio_direction_output(108, 0);  /*wifi on :108 -->0*/
	       if (ret) 
	       {
	           printk(KERN_ERR "%s:  WL_REG_ON  failed to pull down (%d)\n",
	                   __func__, ret);
	            return -EIO;
	       }

	        #if 0
	       ret = gpio_tlmm_config(wifi_config_power_off[0], GPIO_CFG_ENABLE); 
	       if (ret) {
	            printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n",
	                   __func__, wifi_config_power_off[0], ret);
	            return -EIO;
	        }
	        #endif
		mdelay(1);
		if(machine_is_msm7x27_c8651())
		{
		  ret = gpio_direction_output(36, 0);  /*wifi reset :36 -->0*/
		  if (ret) 
		  {
		    printk(KERN_ERR "%s: WL_REG_ON  failed to pull up (%d)\n",
				__func__, ret);
		    return -EIO;
		  }
		}
		else
		{
		  // WLAN chip down 
		  ret = mpp_config_digital_out(20,
				MPP_CFG(MPP_DLOGIC_LVL_MSMP, MPP_DLOGIC_OUT_CTRL_LOW));  /* pull down */
		  if (ret) {
		    printk(KERN_ERR "%s: WL_RST_N failed to pull down(%d)\n",
					__func__, ret);
		    return -EIO;
		  }
		}
		mdelay(1);
		printk(KERN_ERR "%s: wifi power successed to pull down\n",__func__);
	}

	return ret;
}

int __init bcm_wifi_init_gpio_mem(void)
{
	int i;
	int rc=0;
	
	if (gpio_request(108, "WL_REG_ON"))		
  		  printk("Failed to request gpio 108 for WL_REG_ON\n");	

	if (gpio_tlmm_config(wifi_config_power_on[0], GPIO_CFG_ENABLE))
		printk(KERN_ERR "%s: Failed to configure GPIO[108]\n", __func__);

	if (gpio_direction_output(108, 0)) 
		printk(KERN_ERR "%s: WL_REG_ON  failed to pull up \n", __func__);

	if (gpio_request(17, "wlan_wakes_msm"))		
  		  printk("Failed to request gpio 17 for wlan_wakes_msm\n");			

	if(machine_is_msm7x27_c8651())
	{
	  if (gpio_request(36, "WL_REG_RESET"))		
  		    printk("Failed to request gpio 108 for WL_REG_ON\n");	
	}

	rc = gpio_tlmm_config(wlan_wakes_msm[0], GPIO_CFG_ENABLE);	
	if (rc)		
		printk(KERN_ERR "%s: Failed to configure GPIO[17] = %d\n",__func__, rc);

	printk("dev_alloc_skb malloc 32k buffer to avoid page allocation fail\n");
	for(i=0;( i < WLAN_SKB_BUF_NUM );i++) {
		if (i < (WLAN_SKB_BUF_NUM/2))
			wlan_static_skb[i] = dev_alloc_skb(4096); //malloc skb 4k buffer
		else
			wlan_static_skb[i] = dev_alloc_skb(32768); //malloc skb 32k buffer
	}
	for(i=0;( i < PREALLOC_WLAN_NUMBER_OF_SECTIONS );i++) {
		wifi_mem_array[i].mem_ptr = kmalloc(wifi_mem_array[i].size,
							GFP_KERNEL);
		if (wifi_mem_array[i].mem_ptr == NULL)
			return -ENOMEM;
	}
	
	printk("bcm_wifi_init_gpio_mem successfully \n");
	return 0;
}

int bcm_set_carddetect(int detect)
{
	return (int)get_hw_sub_board_id();
}
static struct wifi_platform_data bcm_wifi_control = {
	.mem_prealloc	= bcm_wifi_mem_prealloc,
	.set_power	=bcm_wifi_set_power,
	.set_carddetect = bcm_set_carddetect,
};

static struct platform_device bcm_wifi_device = {
        /* bcm4329_wlan device */
        .name           = "bcm4329_wlan",
        .id             = 1,
        .num_resources  = 0,
        .resource       = NULL,
        .dev            = {
                .platform_data = &bcm_wifi_control,
        },
};
#endif

static void __init msm7x2x_init_mmc(void)
{
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	int rc = 0;
#endif

	if (!machine_is_msm7x25_ffa() && !machine_is_msm7x27_ffa()) {
		vreg_mmc = vreg_get(NULL, "mmc");
		if (IS_ERR(vreg_mmc)) {
			printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			       __func__, PTR_ERR(vreg_mmc));
			return;
		}
	}

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
    if(machine_is_msm7x27_u8510()||machine_is_msm7x27_u8510_1()||machine_is_msm7x27_c8800()||machine_is_msm7x27_u8650())
    {
        	if (gpio_request(MSM_GPIO_SDMC_CD_N, "sdc1_status_irq"))
        		pr_err("failed to request gpio sdc1_status_irq\n");
                
        	rc = gpio_tlmm_config(GPIO_CFG(MSM_GPIO_SDMC_CD_N, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
        				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        	if (rc)
        		printk(KERN_ERR "%s: Failed to configure GPIO %d\n",
        				__func__, rc);
        	
        	rc = set_irq_wake(MSM_GPIO_TO_INT(MSM_GPIO_SDMC_CD_N), 1);
        	if (rc < 0)
        	{
        		printk(KERN_ERR "%s: Unable to set wakeup IRQ %d\n",
        				__func__, rc);	
        	}
    }
#endif
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
    if(machine_is_msm7x27_u8510()||machine_is_msm7x27_u8510_1()||machine_is_msm7x27_c8800()||machine_is_msm7x27_u8650())
    {
	    msm_add_sdcc(1, &msm7x2x_sdc1_data);
	}
	else
	{
		msm_add_sdcc(1, &msm7x2x_sdc1_data_no_detect);
	}
#endif
    
	if (machine_is_msm7x25_surf() || machine_is_msm7x27_surf() ||
		machine_is_msm7x27_ffa() || cpu_is_msm7x27()) {
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
		msm_sdcc_setup_gpio(2, 1);
		msm_add_sdcc(2, &msm7x2x_sdc2_data);
#ifdef CONFIG_HUAWEI_WIFI_SDCC
	bcm_wifi_init_gpio_mem();
	platform_device_register(&bcm_wifi_device);
#endif
#endif
	}

	if (machine_is_msm7x25_surf() || machine_is_msm7x27_surf() || cpu_is_msm7x27()) {
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
		msm_add_sdcc(3, &msm7x2x_sdc3_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
		msm_add_sdcc(4, &msm7x2x_sdc4_data);
#endif
	}
}
#else
#define msm7x2x_init_mmc() do {} while (0)
#endif


static struct msm_pm_platform_data msm7x25_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
};

static struct msm_pm_platform_data msm7x27_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency = 16000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].residency = 20000,

	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].supported = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].suspend_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency = 12000,
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].residency = 20000,

	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].supported = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].suspend_enabled
		= 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].idle_enabled = 1,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency = 2000,
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].residency = 0,
};

static void
msm_i2c_gpio_config(int iface, int config_type)
{
	int gpio_scl;
	int gpio_sda;
	if (iface) {
		gpio_scl = 95;
		gpio_sda = 96;
	} else {
		gpio_scl = 60;
		gpio_sda = 61;
	}
	if (config_type) {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 1, GPIO_CFG_INPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	} else {
		gpio_tlmm_config(GPIO_CFG(gpio_scl, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
		gpio_tlmm_config(GPIO_CFG(gpio_sda, 0, GPIO_CFG_OUTPUT,
					GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	}
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 100000,
	.rmutex  = 0,
	.pri_clk = 60,
	.pri_dat = 61,
	.aux_clk = 95,
	.aux_dat = 96,
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (gpio_request(60, "i2c_pri_clk"))
		pr_err("failed to request gpio i2c_pri_clk\n");
	if (gpio_request(61, "i2c_pri_dat"))
		pr_err("failed to request gpio i2c_pri_dat\n");

#ifndef CONFIG_HUAWEI_KERNEL
	if (gpio_request(95, "i2c_sec_clk"))
		pr_err("failed to request gpio i2c_sec_clk\n");
	if (gpio_request(96, "i2c_sec_dat"))
		pr_err("failed to request gpio i2c_sec_dat\n");
#endif

	if (cpu_is_msm7x27())
		msm_i2c_pdata.pm_lat =
		msm7x27_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;
	else
		msm_i2c_pdata.pm_lat =
		msm7x25_pm_data[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN]
		.latency;

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static void usb_mpp_init(void)
{
	unsigned rc;
	unsigned mpp_usb = 7;

	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		rc = mpp_config_digital_out(mpp_usb,
			MPP_CFG(MPP_DLOGIC_LVL_VDD,
				MPP_DLOGIC_OUT_CTRL_HIGH));
		if (rc)
			pr_err("%s: configuring mpp pin"
				"to enable 3.3V LDO failed\n", __func__);
	}
}

static void msm7x27_wlan_init(void)
{
	int rc = 0;
	/* TBD: if (machine_is_msm7x27_ffa_with_wcn1312()) */
	if (machine_is_msm7x27_ffa()) {
		rc = mpp_config_digital_out(3, MPP_CFG(MPP_DLOGIC_LVL_MSMP,
				MPP_DLOGIC_OUT_CTRL_LOW));
		if (rc)
			printk(KERN_ERR "%s: return val: %d \n",
				__func__, rc);
	}
}

static unsigned char network_cdma = 0;
static unsigned char runmode_factory = 0;
unsigned char network_is_cdma(void)
{
  return network_cdma;
}
unsigned char runmode_is_factory(void)
{
  return runmode_factory;
}

static void proc_factory_para(void)
{
    smem_huawei_vender *factory_para_ptr;

    factory_para_ptr = (smem_huawei_vender*)smem_alloc(SMEM_ID_VENDOR0, sizeof(smem_huawei_vender));
    if (!factory_para_ptr)
    {
    	USB_PR("lxy: %s: Can't find factory parameter\n", __func__);
        return;
    }

    if (NETWORK_CDMA == factory_para_ptr->network_type)
    {
      network_cdma = 1;
      printk("lxy: %s: network_cdma\n", __func__);
    }
    else
    {
      network_cdma = 0;
      printk("lxy: %s: network_umts\n", __func__);
    }
    
    if (MAGIC_NUMBER_FACTORY == factory_para_ptr->run_mode)
    {
      runmode_factory = 1;
      printk("lxy: %s: runmode_factory\n", __func__);
    }
    else
    {
      runmode_factory = 0;
      printk("lxy: %s: runmode_normal\n", __func__);
    }
    
    if (bootimage_is_recovery())
    {
      printk("lxy: %s: bootmode_recovery\n", __func__);
    }
    else
    {
      printk("lxy: %s: bootmode_system\n", __func__);
    }
} 

#ifdef CONFIG_USB_AUTO_INSTALL
u16 pid_index_to_pid(u32 pid_index)
{
		u16 usb_pid = 0xFFFF;
		
		switch(pid_index)
		{
				case CDROM_INDEX:
				case SLATE_TEST_INDEX:
						usb_pid = curr_usb_pid_ptr->cdrom_pid;
						break;
				case NORM_INDEX:
						usb_pid = curr_usb_pid_ptr->norm_pid;
						break;
				case AUTH_INDEX:
						usb_pid = curr_usb_pid_ptr->auth_pid;
						break;
				case GOOGLE_INDEX:
						usb_pid = curr_usb_pid_ptr->google_pid;
						break;
				
				case GOOGLE_WLAN_INDEX:
						usb_pid = curr_usb_pid_ptr->wlan_pid;
				case ORI_INDEX:
				default:
						usb_pid = curr_usb_pid_ptr->norm_pid;
						break;
		}

		USB_PR("lxy: %s, pid_index=%d, usb_pid=0x%x\n", __func__, pid_index, usb_pid);
		return usb_pid;
}

void set_usb_device_name(void)
{
	memset(manufacturer_name, 0, MAX_NAME_LEN);
	memset(product_name, 0, MAX_NAME_LEN);
	memset(vendor_name, 0, MAX_NAME_LEN);
	
	strcpy(manufacturer_name, "Huawei Incorporated");
	strcpy(product_name, "Android Adapter");
}

void set_usb_pid_sn(u32 pid_index)
{
		switch(pid_index)
		{
				case GOOGLE_WLAN_INDEX:
						USB_PR("lxy: set pid=0x%x, sn=NULL\n", GOOGLE_WLAN_INDEX);
						android_set_product_id(PID_WLAN);
						set_usb_sn(NULL);
						break;
				case GOOGLE_INDEX:
						USB_PR("lxy: set pid=0x%x, sn=%s\n", PID_GOOGLE_MS, usb_para_data.usb_para.usb_serial);
						android_set_product_id(PID_GOOGLE_MS);
						set_usb_sn(usb_para_data.usb_para.usb_serial);
						break;
						
				case NORM_INDEX:
						USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, USB_SN_STRING);
						android_set_product_id(curr_usb_pid_ptr->norm_pid);
						set_usb_sn(USB_SN_STRING);
						break;
				case SLATE_TEST_INDEX:
				case CDROM_INDEX:
						USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->cdrom_pid, "");
						android_set_product_id(curr_usb_pid_ptr->cdrom_pid);
						set_usb_sn(NULL);
						break;
						
				case ORI_INDEX:
						USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, "");
						android_set_product_id(curr_usb_pid_ptr->norm_pid);
						set_usb_sn(NULL);
						break;
						
				case AUTH_INDEX:
						USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->auth_pid, "");
						android_set_product_id(curr_usb_pid_ptr->auth_pid);
						set_usb_sn(NULL);
						break;
						
				default:
						USB_PR("lxy: set pid=0x%x, sn=%s\n", curr_usb_pid_ptr->norm_pid, "");
						android_set_product_id(curr_usb_pid_ptr->norm_pid);
						set_usb_sn(NULL);
						break;
		}

}


static void proc_usb_para(void)
{
		smem_huawei_vender *usb_para_ptr;
		char *vender_name="t-mobile";

		USB_PR("lxy: < %s\n", __func__);

		usb_para_info.usb_pid_index = 0;
		usb_para_info.usb_pid = PID_NORMAL;

		set_usb_device_name();
		
		usb_para_ptr = (smem_huawei_vender*)smem_alloc(SMEM_ID_VENDOR0, sizeof(smem_huawei_vender));
		if (!usb_para_ptr)
		{
			USB_PR("lxy: %s: Can't find usb parameter\n", __func__);
				return;
		}

		USB_PR("lxy: vendor:%s,country:%s\n", usb_para_ptr->vender_para.vender_name, usb_para_ptr->vender_para.
country_name);

		memcpy(&usb_para_data, usb_para_ptr, sizeof(smem_huawei_vender));
		
		if(!memcmp(usb_para_ptr->vender_para.vender_name, vender_name, strlen(vender_name)))
		{
				curr_usb_pid_ptr = &usb_pid_array[1];
				USB_PR("lxy: USB setting is TMO\n");
		}
		else
		{
				curr_usb_pid_ptr = &usb_pid_array[0];
				USB_PR("lxy: USB setting is NORMAL\n");
		}

		USB_PR("lxy: smem usb_serial=%s, usb_pid_index=%d\n", usb_para_ptr->usb_para.usb_serial, usb_para_ptr->usb_para.
usb_pid_index);

		if (0 == usb_para_data.usb_para.usb_serial[0] 
			&& GOOGLE_INDEX == usb_para_ptr->usb_para.usb_pid_index)
		{
			USB_PR("%s usb serial number is null in google mode. so switch to original mode\n", __func__);
			usb_para_ptr->usb_para.usb_pid_index = ORI_INDEX;
		}

		usb_para_info.usb_pid_index = usb_para_ptr->usb_para.usb_pid_index;
		
		usb_para_info.usb_pid = pid_index_to_pid(usb_para_ptr->usb_para.usb_pid_index);

		set_usb_pid_sn(usb_para_info.usb_pid_index);

        if ((0 == memcmp(usb_para_data.vender_para.vender_name, VENDOR_EMOBILE, strlen(VENDOR_EMOBILE)))
        && (0 == memcmp(usb_para_data.vender_para.country_name, COUNTRY_JAPAN, strlen(COUNTRY_JAPAN))))
        {
            u16 index = 0;
            for (; index < android_usb_pdata.num_products; index++)
            {
                if (PID_GOOGLE == android_usb_pdata.products[index].adb_product_id)
                {
                    android_usb_pdata.products[index].nluns = 2;/* 2 is two udisk */
                    android_usb_pdata.products[index].cdrom_index = -1;/* -1 is not support cdrom */
                    USB_PR("%s %s%s doesn't need the cdrom. nluns=%d, cdrom_index=%d\n",
                    __func__, usb_para_data.vender_para.country_name, usb_para_data.vender_para.vender_name,
                    android_usb_pdata.products[index].nluns, android_usb_pdata.products[index].cdrom_index);
                }  
            }  
        }
		
		USB_PR("lxy: curr_usb_pid_ptr: 0x%x, 0x%x, 0x%x, 0x%x, 0x%x\n", 
				curr_usb_pid_ptr->cdrom_pid, 
				curr_usb_pid_ptr->norm_pid, 
				curr_usb_pid_ptr->udisk_pid,
				curr_usb_pid_ptr->auth_pid,
				curr_usb_pid_ptr->google_pid);
		USB_PR("lxy: usb_para_info: usb_pid_index=%d, usb_pid = 0x%x>\n", 
				usb_para_info.usb_pid_index, 
				usb_para_info.usb_pid);
}
#endif	/* CONFIG_USB_AUTO_INSTALL */


static ssize_t synaptics_virtual_keys_show(struct kobject *kobj,
			       struct kobj_attribute *attr, char *buf)
{
        memcpy( buf, buf_virtualkey, buf_vkey_size );
		return buf_vkey_size; 
}

static struct kobj_attribute synaptics_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.synaptics",
		.mode = S_IRUGO,
	},
	.show = &synaptics_virtual_keys_show,
};
static struct kobj_attribute atmel_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.atmel-rmi-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &synaptics_virtual_keys_show,
};

static struct attribute *synaptics_properties_attrs[] = {
	&synaptics_virtual_keys_attr.attr,
    &atmel_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group synaptics_properties_attr_group = {
	.attrs = synaptics_properties_attrs,
};
static void __init virtualkeys_init(void)
{
    struct kobject *properties_kobj;
    int ret;

    if(machine_is_msm7x27_c8800())
    {  
        buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":67:851:130:62"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":192:851:112:62"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_HOME)   ":309:851:116:62"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":424:851:110:62"
        		   "\n");  
    }
    else if( machine_is_msm7x27_m865() || machine_is_msm7x27_u8650() ||machine_is_msm7x27_c8650() ||machine_is_msm7x27_c8651())
    {
        buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_HOME)  ":35:520:70:60"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":118:520:70:60"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)   ":205:520:70:60"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":285:520:70:60"
        		   "\n");
    }
    else
    {
        buf_vkey_size = sprintf(buf_virtualkey,
        			__stringify(EV_KEY) ":" __stringify(KEY_BACK)  ":50:520:80:20"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)   ":160:520:80:20"
        		   ":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":270:520:80:20"
        		   "\n");  
    }

   	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		ret = sysfs_create_group(properties_kobj,
					 &synaptics_properties_attr_group);
	if (!properties_kobj || ret)
		pr_err("failed to create board_properties\n");
}

static void __init msm7x2x_init(void)
{

#ifdef CONFIG_ARCH_MSM7X25
	msm_clock_init(msm_clocks_7x25, msm_num_clocks_7x25);
#elif defined(CONFIG_ARCH_MSM7X27)
	msm_clock_init(msm_clocks_7x27, msm_num_clocks_7x27);
#endif

#if defined(CONFIG_SMC91X)
	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa()) {
		smc91x_resources[0].start = 0x98000300;
		smc91x_resources[0].end = 0x980003ff;
		smc91x_resources[1].start = MSM_GPIO_TO_INT(85);
		smc91x_resources[1].end = MSM_GPIO_TO_INT(85);
		if (gpio_tlmm_config(GPIO_CFG(85, 0,
					      GPIO_CFG_INPUT,
					      GPIO_CFG_PULL_DOWN,
					      GPIO_CFG_2MA),
				     GPIO_CFG_ENABLE)) {
			printk(KERN_ERR
			       "%s: Err: Config GPIO-85 INT\n",
				__func__);
		}
	}
#endif

	if (cpu_is_msm7x27())
		msm7x2x_clock_data.max_axi_khz = 200000;

	msm_acpu_clock_init(&msm7x2x_clock_data);

#ifdef CONFIG_ARCH_MSM7X27
	/* This value has been set to 160000 for power savings. */
	/* OEMs may modify the value at their discretion for performance */
	/* The appropriate maximum replacement for 160000 is: */
	/* clk_get_max_axi_khz() */
	kgsl_pdata.high_axi_3d = 160000;

	/* 7x27 doesn't allow graphics clocks to be run asynchronously to */
	/* the AXI bus */
	kgsl_pdata.max_grp2d_freq = 0;
	kgsl_pdata.min_grp2d_freq = 0;
	kgsl_pdata.set_grp2d_async = NULL;
	kgsl_pdata.max_grp3d_freq = 0;
	kgsl_pdata.min_grp3d_freq = 0;
	kgsl_pdata.set_grp3d_async = NULL;
	kgsl_pdata.imem_clk_name = "imem_clk";
	kgsl_pdata.grp3d_clk_name = "grp_clk";
	kgsl_pdata.grp3d_pclk_name = "grp_pclk";
	kgsl_pdata.grp2d0_clk_name = NULL;
	kgsl_pdata.idle_timeout_3d = HZ/5;
	kgsl_pdata.idle_timeout_2d = 0;

#ifdef CONFIG_KGSL_PER_PROCESS_PAGE_TABLE
	kgsl_pdata.pt_va_size = SZ_32M;
	kgsl_pdata.pt_max_count = 32;
#else
	kgsl_pdata.pt_va_size = SZ_128M;
	kgsl_pdata.pt_max_count = 1;
#endif
#endif
	usb_mpp_init();

#ifdef CONFIG_USB_FUNCTION
	msm_hsusb_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;

	msm_device_hsusb_peripheral.dev.platform_data = &msm_hsusb_pdata;
#endif

	#ifdef CONFIG_USB_AUTO_INSTALL
	proc_usb_para();
	#endif  /* #ifdef CONFIG_USB_AUTO_INSTALL */

    proc_factory_para();

#ifdef CONFIG_USB_MSM_OTG_72K
	msm_device_otg.dev.platform_data = &msm_otg_pdata;
	if (machine_is_msm7x25_surf() || machine_is_msm7x25_ffa()) {
		msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_20_PERCENT;
		msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_5_PERCENT;
		msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_ENABLE;
		msm_otg_pdata.phy_reset = msm_otg_rpc_phy_reset;
	}
	if (machine_is_msm7x27_surf() || machine_is_msm7x27_ffa() || cpu_is_msm7x27()) {
		msm_otg_pdata.pemp_level =
			PRE_EMPHASIS_WITH_10_PERCENT;
		msm_otg_pdata.drv_ampl = HS_DRV_AMPLITUDE_5_PERCENT;
		msm_otg_pdata.cdr_autoreset = CDR_AUTO_RESET_DISABLE;
		msm_otg_pdata.phy_reset_sig_inverted = 1;
	}

#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
		msm7x27_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
	msm_gadget_pdata.is_phy_status_timer_on = 1;
#endif
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif

	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_MSM_CAMERA
	config_camera_off_gpios(); /* might not be necessary */
#endif
	msm_device_i2c_init();
	i2c_register_board_info(0, i2c_devices, ARRAY_SIZE(i2c_devices));

	i2c_register_board_info(2, i2c_aux2_devices, ARRAY_SIZE(i2c_aux2_devices));

#if defined(CONFIG_SURF_FFA_GPIO_KEYPAD) && defined(CONFIG_MSM7X27_SURF_DEBUG)
	if (machine_is_msm7x25_ffa() || machine_is_msm7x27_ffa())
		platform_device_register(&keypad_device_7k_ffa);
	else
		platform_device_register(&keypad_device_surf);
#else
	if(machine_is_msm7x27_u8510()||machine_is_msm7x27_u8510_1())
    {
	    platform_device_register(&keypad_device_u8510);
    }   
	else if( machine_is_msm7x27_c8800() )
    {
    	platform_device_register(&keypad_device_c8800);
    }   
    else if( machine_is_msm7x27_u8650() )
    {
      platform_device_register(&keypad_device_u8650);
    }   

	else if(machine_is_msm7x27_m865())
	{
		platform_device_register(&keypad_device_m865);  
	}
	else if(machine_is_msm7x27_c8650())
	{
		platform_device_register(&keypad_device_c8650);  
	}	
	else if(machine_is_msm7x27_c8651())
	{
		platform_device_register(&keypad_device_c8651);  
	}
#endif
	lcdc_gordon_gpio_init();
	msm_fb_add_devices();
#ifdef CONFIG_USB_EHCI_MSM
	msm7x2x_init_host();
#endif
	msm7x2x_init_mmc();
	bt_power_init();

	if (cpu_is_msm7x27())
		msm_pm_set_platform_data(msm7x27_pm_data,
					ARRAY_SIZE(msm7x27_pm_data));
	else
		msm_pm_set_platform_data(msm7x25_pm_data,
					ARRAY_SIZE(msm7x25_pm_data));
	msm7x27_wlan_init();
#ifdef CONFIG_HUAWEI_FEATURE_OFN_KEY
    init_ofn_ok_key_device();
#endif
    virtualkeys_init();

#ifdef CONFIG_HUAWEI_KERNEL
    hw_extern_sdcard_add_device();
#endif

	printk("cpu_is_msm7x27() = %d\n",cpu_is_msm7x27());
	printk("machine_arch_type = %d\n",machine_arch_type);
}

static unsigned pmem_kernel_ebi1_size = PMEM_KERNEL_EBI1_SIZE;
static int __init pmem_kernel_ebi1_size_setup(char *p)
{
	pmem_kernel_ebi1_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi1_size", pmem_kernel_ebi1_size_setup);

static unsigned pmem_mdp_size = MSM_PMEM_MDP_SIZE;
static int __init pmem_mdp_size_setup(char *p)
{
	pmem_mdp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_mdp_size", pmem_mdp_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

#ifdef CONFIG_HUAWEI_KERNEL
static int __init set_msm_pmem_mdp_size(void)
{
	lcd_type lcd_y_res = LCD_IS_HVGA;

	lcd_y_res =  atag_get_lcd_y_res();
	printk("%s: get lcd_y_res = %d. from ATAG_LCD_Y_RES_FLAG \n", __func__ , lcd_y_res);
	switch(lcd_y_res)
	{
		case LCD_IS_QVGA:
			pmem_mdp_size = MSM_PMEM_MDP_QVGA_SIZE;
			break;
		case LCD_IS_HVGA:
			pmem_mdp_size = MSM_PMEM_MDP_HVGA_SIZE;
			break;
		case LCD_IS_WVGA:
			pmem_mdp_size = MSM_PMEM_MDP_WVGA_SIZE;
			break;
		default:
			pmem_mdp_size = MSM_PMEM_MDP_WVGA_SIZE;
			break;
	}

	return 0;
}
#endif

static void __init msm_msm7x2x_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	#ifdef CONFIG_FRAMEBUF_SELF_ADAPT
	__u32 framebuf_addr = 0; 
	__u32 framebuf_size = 0;
	#endif

#ifdef CONFIG_HUAWEI_KERNEL
	set_msm_pmem_mdp_size();
#endif
	size = pmem_mdp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_pdata.start = __pa(addr);
		android_pmem_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for mdp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_adsp_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_adsp_pdata.start = __pa(addr);
		android_pmem_adsp_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for adsp "
			"pmem arena\n", size, addr, __pa(addr));
	}

	size = pmem_audio_size;
	if (size) {
		addr = alloc_bootmem(size);
		android_pmem_audio_pdata.start = __pa(addr);
		android_pmem_audio_pdata.size = size;
		pr_info("allocating %lu bytes (at %lx physical) for audio "
			"pmem arena\n", size , __pa(addr));
	}

#ifdef CONFIG_FRAMEBUF_SELF_ADAPT
	get_frame_buffer_mem_region(&framebuf_addr, &framebuf_size);
	msm_fb_resources[0].start = framebuf_addr;
	msm_fb_resources[0].end = framebuf_addr + framebuf_size -1;
	
	pr_info("allocating %lu bytes at %p for framebuffer\n", framebuf_size, framebuf_addr);

	size = HUAWEI_SHARE_MEMORY_SIZE;
	if (size) {
		huawei_share_memory_resources[0].start = LCD_FRAME_BUFF_END_ADDR + HUAWEI_CRASH_MEM_SIZE;
		huawei_share_memory_resources[0].end = huawei_share_memory_resources[0].start +  size - 1;
	}
#else
	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem(size);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
#endif

	size = pmem_kernel_ebi1_size;
	if (size) {
		addr = alloc_bootmem_aligned(size, 0x100000);
		android_pmem_kernel_ebi1_pdata.start = __pa(addr);
		android_pmem_kernel_ebi1_pdata.size = size;
		pr_info("allocating %lu bytes at %p (%lx physical) for kernel"
			" ebi1 pmem arena\n", size, addr, __pa(addr));
	}
}
static void __init msm7x2x_fixup(struct machine_desc *desc,
                                 struct tag *tags,
                                 char **cmdline,
                                 struct meminfo *mi)
{

}
static void __init msm7x2x_map_io(void)
{
	msm_map_common_io();
	msm_msm7x2x_allocate_memory_regions();

	if (socinfo_init() < 0)
		BUG();
#ifdef CONFIG_CACHE_L2X0
//	if (machine_is_msm7x27_surf() || machine_is_msm7x27_ffa() ) {
		/* 7x27 has 256KB L2 cache:
			64Kb/Way and 4-Way Associativity;
			evmon/parity/share disabled. */
		if ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) > 1)
			|| ((SOCINFO_VERSION_MAJOR(socinfo_get_version()) == 1)
			&& (SOCINFO_VERSION_MINOR(socinfo_get_version()) >= 3)))
			/* R/W latency: 4 cycles; */
			l2x0_init(MSM_L2CC_BASE, 0x0006801B, 0xfe000000);
		else
			/* R/W latency: 3 cycles; */
			l2x0_init(MSM_L2CC_BASE, 0x00068012, 0xfe000000);
//	}
#endif
}

MACHINE_START(MSM7X27_SURF, "QCT MSM7x27 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7X27_FFA, "QCT MSM7x27 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7X25_SURF, "QCT MSM7x25 SURF")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7X25_FFA, "QCT MSM7x25 FFA")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io        = MSM_DEBUG_UART_PHYS,
	.io_pg_offst    = ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.map_io		= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END


MACHINE_START(MSM7X27_U8510, "HUAWEI U8510 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io		= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup          = msm7x2x_fixup,
	.map_io 	= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7X27_M650, "HUAWEI M650 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io		= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup          = msm7x2x_fixup,
	.map_io 	= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END
MACHINE_START(MSM7X27_U8510_1, "HUAWEI U8510_1 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io		= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup          = msm7x2x_fixup,
	.map_io 	= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7X27_C8800, "HUAWEI C8800 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io		= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup          = msm7x2x_fixup,
	.map_io 	= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END

MACHINE_START(MSM7X27_U8650, "HUAWEI U8650 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io		= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup          = msm7x2x_fixup,
	.map_io 	= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END
MACHINE_START(MSM7X27_M865, "HUAWEI M865 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io		= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup          = msm7x2x_fixup,
	.map_io 	= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END
MACHINE_START(MSM7X27_C8650, "HUAWEI C8650 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io		= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup          = msm7x2x_fixup,
	.map_io 	= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END
MACHINE_START(MSM7X27_C8651, "HUAWEI C8651 BOARD")
#ifdef CONFIG_MSM_DEBUG_UART
	.phys_io		= MSM_DEBUG_UART_PHYS,
	.io_pg_offst	= ((MSM_DEBUG_UART_BASE) >> 18) & 0xfffc,
#endif
	.boot_params	= PHYS_OFFSET + 0x100,
	.fixup          = msm7x2x_fixup,
	.map_io 	= msm7x2x_map_io,
	.init_irq	= msm7x2x_init_irq,
	.init_machine	= msm7x2x_init,
	.timer		= &msm_timer,
MACHINE_END
