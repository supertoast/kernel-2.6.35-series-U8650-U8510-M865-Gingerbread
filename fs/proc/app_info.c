/*
 *  linux/fs/proc/app_info.c
 *
 *
 * Changes:
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/time.h>
#include <linux/kernel.h>
#include <linux/kernel_stat.h>
#include <linux/fs.h>
#include <linux/tty.h>
#include <linux/string.h>
#include <linux/mman.h>
#include <linux/quicklist.h>
#include <linux/proc_fs.h>
#include <linux/ioport.h>
#include <linux/mm.h>
#include <linux/mmzone.h>
#include <linux/pagemap.h>
#include <linux/interrupt.h>
#include <linux/swap.h>
#include <linux/slab.h>
#include <linux/genhd.h>
#include <linux/smp.h>
#include <linux/signal.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/seq_file.h>
#include <linux/times.h>
#include <linux/profile.h>
#include <linux/utsname.h>
#include <linux/blkdev.h>
#include <linux/hugetlb.h>
#include <linux/jiffies.h>
#include <linux/sysrq.h>
#include <linux/vmalloc.h>
#include <linux/crash_dump.h>
#include <linux/pid_namespace.h>
#include <linux/bootmem.h>
#include <asm/uaccess.h>
#include <asm/pgtable.h>
#include <asm/io.h>
#include <asm/tlb.h>
#include <asm/div64.h>
#include "internal.h"
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <linux/hardware_self_adapt.h>

#define PROC_MANUFACTURER_STR_LEN 30
#define MAX_VERSION_CHAR 40
#define BOARD_ID_LEN 20
#define LCD_NAME_LEN 20
#define HW_VERSION   20
#define HW_VERSION_SUB_VER  6
#define SUB_VER_LEN  10

static char appsboot_version[MAX_VERSION_CHAR + 1];
static char str_flash_nand_id[PROC_MANUFACTURER_STR_LEN] = {0};
static u32 camera_id;
static u32 ts_id;
#ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE
static u32 charge_flag;
extern unsigned int get_charge_flag(void);
#endif
/*del 1 line*/

#ifdef CONFIG_HUAWEI_KERNEL
extern void get_flash_id(char *,int);
#endif

/* same as in proc_misc.c */
static int
proc_calc_metrics(char *page, char **start, off_t off, int count, int *eof, int len)
{
	if (len <= off + count)
		*eof = 1;
	*start = page + off;
	len -= off;
	if (len > count)
		len = count;
	if (len < 0)
		len = 0;
	return len;
}

#define ATAG_BOOT_READ_FLASH_ID 0x4d534D72
static int __init parse_tag_boot_flash_id(const struct tag *tag)
{
    char *tag_flash_id=(char*)&tag->u;
    memset(str_flash_nand_id, 0, PROC_MANUFACTURER_STR_LEN);
    memcpy(str_flash_nand_id, tag_flash_id, PROC_MANUFACTURER_STR_LEN);
    
    printk("########proc_misc.c: tag_boot_flash_id= %s\n", tag_flash_id);

    return 0;
}
__tagtable(ATAG_BOOT_READ_FLASH_ID, parse_tag_boot_flash_id);

/*parse atag passed by appsboot, ligang 00133091, 2009-4-13, start*/
#define ATAG_BOOT_VERSION 0x4d534D71 /* ATAG BOOT VERSION */
static int __init parse_tag_boot_version(const struct tag *tag)
{
    char *tag_boot_ver=(char*)&tag->u;
    memset(appsboot_version, 0, MAX_VERSION_CHAR + 1);
    memcpy(appsboot_version, tag_boot_ver, MAX_VERSION_CHAR);
     
    //printk("nand_partitions.c: appsboot_version= %s\n\n", appsboot_version);

    return 0;
}
__tagtable(ATAG_BOOT_VERSION, parse_tag_boot_version);


#define ATAG_CAMERA_ID 0x4d534D74
static int __init parse_tag_camera_id(const struct tag *tag)
{
    char *tag_boot_ver=(char*)&tag->u;
	
    memcpy((void*)&camera_id, tag_boot_ver, sizeof(u32));
     
    return 0;
}
__tagtable(ATAG_CAMERA_ID, parse_tag_camera_id);


#define ATAG_TS_ID 0x4d534D75
static int __init parse_tag_ts_id(const struct tag *tag)
{
    char *tag_boot_ver=(char*)&tag->u;
	
    memcpy((void*)&ts_id, tag_boot_ver, sizeof(u32));
     
    return 0;
}


__tagtable(ATAG_TS_ID, parse_tag_ts_id);


/*write the position of compass into the file of "/proc/app_info" */
static int app_version_read_proc(char *page, char **start, off_t off,
				 int count, int *eof, void *data)
{
	int len;
	char *ker_ver = HUAWEI_KERNEL_VERSION;
	char *lcd_name = NULL;
	char s_board_id[BOARD_ID_LEN] = {0};
	char sub_ver[SUB_VER_LEN] = {0};
	char hw_version_id[HW_VERSION] = {0};
	char hw_version_sub_ver[HW_VERSION_SUB_VER] = {0};	
	char *compass_gs_name = NULL;
	char *wifi_name = NULL;

	switch(machine_arch_type)
	{
		case MACH_TYPE_MSM7X27_U8510:
			strcpy(s_board_id, "MSM7X27_U8510");
			break;

		case MACH_TYPE_MSM7X27_U8510_1:
			strcpy(s_board_id, "MSM7X27_U8510_1");
			break;
			
		case MACH_TYPE_MSM7X27_M650:
			strcpy(s_board_id, "MSM7X27_M650");
			break;

		case MACH_TYPE_MSM7X27_C8800:
			strcpy(s_board_id, "MSM7X27_C8800");
			break;

        case MACH_TYPE_MSM7X27_U8650:
			strcpy(s_board_id, "MSM7X27_U8650");
			break;
		
		case MACH_TYPE_MSM7X27_M865:
			strcpy(s_board_id, "MSM7X27_M865");
			break;
		case MACH_TYPE_MSM7X27_C8650:
			strcpy(s_board_id, "MSM7X27_C8650");
			break;

		default:
			strcpy(s_board_id, "ERROR");
			break;
	}

    sprintf(sub_ver, ".Ver%c", 'A'+(char)get_hw_sub_board_id());
    strcat(s_board_id, sub_ver);
  
    strcat(hw_version_id, hw_version_sub_ver);
   
	lcd_name = get_lcd_panel_name();
	
	compass_gs_name=get_compass_gs_position_name();

	wifi_name = get_wifi_device_name();
	
#ifdef CONFIG_HUAWEI_KERNEL	
    memset(str_flash_nand_id,0,PROC_MANUFACTURER_STR_LEN);
    get_flash_id(str_flash_nand_id,PROC_MANUFACTURER_STR_LEN);
#endif	
    /* write the power down charge flag to the file /proc/app_info,
     * so we can read the flag in recovery mode to decide we enter  
     * the recovery mode or power down charge movie
     */
    #ifdef CONFIG_HUAWEI_POWER_DOWN_CHARGE    
    charge_flag = get_charge_flag();
    
	len = snprintf(page, PAGE_SIZE, "APPSBOOT:\n"
	"%s\n"
	"KERNEL_VER:\n"
	"%s\n"
	 "FLASH_ID:\n"
	"%s\n"
	"board_id:\n%s\n"
	"lcd_id:\n%s\n"
	"cam_id:\n%d\n"
	"ts_id:\n%d\n"
	"hw_version:\n%s\n"
	"charge_flag:\n%d\n"
	"compass_gs_position:\n%s\n"
	"wifi_name:\n%s\n",
	appsboot_version, ker_ver, str_flash_nand_id, s_board_id, lcd_name, camera_id, ts_id, hw_version_id, charge_flag,compass_gs_name,wifi_name);
    #else
	len = snprintf(page, PAGE_SIZE, "APPSBOOT:\n"
	"%s\n"
	"KERNEL_VER:\n"
	"%s\n"
	 "FLASH_ID:\n"
	"%s\n"
	"board_id:\n%s\n"
	"lcd_id:\n%s\n"
	"cam_id:\n%d\n"
	"ts_id:\n%d\n"
	"hw_version:\n%s\n"
	"compass_gs_position:\n%s\n"
	"wifi_name:\n%s\n",
	appsboot_version, ker_ver, str_flash_nand_id, s_board_id, lcd_name, camera_id, ts_id, hw_version_id,compass_gs_name,wifi_name);
    #endif
	
	return proc_calc_metrics(page, start, off, count, eof, len);
}

void __init proc_app_info_init(void)
{
	static struct {
		char *name;
		int (*read_proc)(char*,char**,off_t,int,int*,void*);
	} *p, simple_ones[] = {
		
        {"app_info", app_version_read_proc},
		{NULL,}
	};
	for (p = simple_ones; p->name; p++)
		create_proc_read_entry(p->name, 0, NULL, p->read_proc, NULL);

}


