/*
 * Gadget Driver for Android
 *
 * Copyright (C) 2008 Google, Inc.
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights reserved.
 * Author: Mike Lockwood <lockwood@android.com>
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

/* #define DEBUG */
/* #define VERBOSE_DEBUG */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/fs.h>

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/utsname.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include <linux/usb/android_composite.h>
#include <linux/usb/ch9.h>
#include <linux/usb/composite.h>
#include <linux/usb/gadget.h>

#include "gadget_chips.h"

#include "usb_switch_huawei.h"
/*
 * Kbuild is not very cooperative with respect to linking separately
 * compiled library objects into one module.  So for now we won't use
 * separate compilation ... ensuring init/exit sections work to shrink
 * the runtime footprint, and giving us at least some parts of what
 * a "gcc --combine ... part1.c part2.c part3.c ... " build would.
 */
#include "usbstring.c"
#include "config.c"
#include "epautoconf.c"
#include "composite.c"

static struct delayed_work android_usb_switch_work;
/* destination PID when switch USB composition */
static u16 usb_switch_dest_pid = 0;
/* 0: USB sn is NULL
   1: USB sn is valid
*/
static u16 usb_sn_valid = 0;
static u8 serial_number[MAX_NAME_LEN]="";
static u16 product_id = 0x1035;

MODULE_AUTHOR("Mike Lockwood");
MODULE_DESCRIPTION("Android Composite USB Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");

static const char longname[] = "Gadget Android";

/* Default vendor and product IDs, overridden by platform data */
#define VENDOR_ID		0x18D1
#define PRODUCT_ID		0x0001

struct android_dev {
	struct usb_composite_dev *cdev;
	struct usb_configuration *config;
	int num_products;
	struct android_usb_product_hw *products;
	int num_functions;
	char **functions;

	int version;
	int adb_enable;
	struct mutex lock;
};

static struct android_dev *_android_dev;

/* string IDs are assigned dynamically */

#define STRING_MANUFACTURER_IDX		0
#define STRING_PRODUCT_IDX		1
#define STRING_SERIAL_IDX		2

/* String Table */
static struct usb_string strings_dev[] = {
	/* These dummy values should be overridden by platform data */
	[STRING_MANUFACTURER_IDX].s = "Android",
	[STRING_PRODUCT_IDX].s = "Android",
	[STRING_SERIAL_IDX].s = "0123456789ABCDEF",
	{  }			/* end of list */
};

static struct usb_gadget_strings stringtab_dev = {
	.language	= 0x0409,	/* en-us */
	.strings	= strings_dev,
};

static struct usb_gadget_strings *dev_strings[] = {
	&stringtab_dev,
	NULL,
};

static struct usb_device_descriptor device_desc = {
	.bLength              = sizeof(device_desc),
	.bDescriptorType      = USB_DT_DEVICE,
	.bcdUSB               = __constant_cpu_to_le16(0x0200),
	.bDeviceClass         = USB_CLASS_PER_INTERFACE,
	.idVendor             = __constant_cpu_to_le16(VENDOR_ID),
	.idProduct            = __constant_cpu_to_le16(PRODUCT_ID),
	.bcdDevice            = __constant_cpu_to_le16(0xffff),
	.bNumConfigurations   = 1,
};

static struct usb_otg_descriptor otg_descriptor = {
	.bLength =		sizeof otg_descriptor,
	.bDescriptorType =	USB_DT_OTG,
	.bmAttributes =		USB_OTG_SRP | USB_OTG_HNP,
	.bcdOTG               = __constant_cpu_to_le16(0x0200),
};

static const struct usb_descriptor_header *otg_desc[] = {
	(struct usb_descriptor_header *) &otg_descriptor,
	NULL,
};

static struct list_head _functions = LIST_HEAD_INIT(_functions);
static int _registered_function_count = 0;
static void matche_rndis_and_comm_function(int isrndis);
extern usb_switch_stru usb_switch_para;
extern void usb_get_state(unsigned *state_para, unsigned *usb_state_para);

void android_usb_set_connected(int connected)
{
  USB_PR("%s connected=%d\n", __func__, connected);
  if (_android_dev && _android_dev->cdev && _android_dev->cdev->gadget) {
		if (connected)
			usb_gadget_connect(_android_dev->cdev->gadget);
		else
			usb_gadget_disconnect(_android_dev->cdev->gadget);
	}
}

static struct android_usb_function *get_function(const char *name)
{
	struct android_usb_function	*f;
	list_for_each_entry(f, &_functions, list) {
    USB_PR("%s name=%s, fun_name=%s\n", __func__, name, f->name);
		if (!strcmp(name, f->name))
			return f;
	}
	return 0;
}

static void bind_functions(struct android_dev *dev)
{
	struct android_usb_function	*f;
	char **functions = dev->functions;
	int i;
	USB_PR("%s \n", __func__);
	for (i = 0; i < dev->num_functions; i++) {
		char *name = *functions++;
		f = get_function(name);
		if (f)
		{
			USB_PR("%s name=%s \n", __func__, f->name);
			f->bind_config(dev->config);
		}
		else
			USB_PR("function %s not found in bind_functions\n", name);
}

	/*
	 * set_alt(), or next config->bind(), sets up
	 * ep->driver_data as needed.
	 */
	usb_ep_autoconfig_reset(dev->cdev->gadget);
}

static int __devinit android_bind_config(struct usb_configuration *c)
{
	struct android_dev *dev = _android_dev;

	USB_PR("android_bind_config reg_count=%d fun_count=%d\n",
	 _registered_function_count, dev->num_functions);
	dev->config = c;

	/* bind our functions if they have all registered */
	if (_registered_function_count == dev->num_functions)
		bind_functions(dev);

	USB_PR("%s end\n", __func__);
	return 0;
}

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl);

static struct usb_configuration android_config_driver = {
	.label		= "android",
	.bind		= android_bind_config,
	.setup		= android_setup_config,
	.bConfigurationValue = 1,
	.bMaxPower	= 0xFA, /* 500ma */
};

static int android_setup_config(struct usb_configuration *c,
		const struct usb_ctrlrequest *ctrl)
{
	int i;
	int ret = -EOPNOTSUPP;
    /* del the USB_PR msg */
	for (i = 0; i < android_config_driver.next_interface_id; i++) {
		if (android_config_driver.interface[i]->setup) {
			ret = android_config_driver.interface[i]->setup(
				android_config_driver.interface[i], ctrl);
			if (ret >= 0)
				return ret;
		}
	}
	return ret;
}

static int product_matches_pid_to_functions(u16 pid)
{
	struct usb_function   *f;
	struct android_dev *dev = _android_dev;
	struct android_usb_product_hw *p = dev->products;
	int count = dev->num_products;
	char **functions;
	int i;

	USB_PR("%s \n", __func__);
	for (i = 0; i < count; i++, p++) {
		if (pid == p->adb_product_id || pid == p->product_id)
		{
			break;
		}
	}

	if (i >= count)
	{
		USB_PR("%s not find pid=%d\n", __func__, pid);
		return 0;
	}
	USB_PR("%s matches_pid=0x%x \n", __func__, p->product_id);

	matche_rndis_and_comm_function(0);

	if (dev->adb_enable)
	{
		functions = p->adb_functions;
		count = p->adb_num_functions;
		USB_PR("%s adb enable count=%d\n", __func__, count);
	}
	else
	{
		functions = p->functions;
		count = p->num_functions;
		USB_PR("%s adb disable count=%d\n", __func__, count);
	}

	list_for_each_entry(f, &android_config_driver.functions, list)
	{
		f->disabled = 1;
		USB_PR("%s matches_pid=0x%x function=%s num_functions=%d hidden\n", 
		  __func__, p->product_id, f->name, count);

		for (i=0; i<count; i++)
		{
			if (!strcmp(f->name, *(functions+i)))
			{
				if (!strcmp(f->name, "rndis"))
				{
				  matche_rndis_and_comm_function(1);
				}
				f->disabled = 0;
				USB_PR("%s matches_pid=0x%x function=%s enable\n", 
				  __func__, p->product_id, f->name);
				break;
			}
		}
	}

	return 1;
}

void switch_composite_function(u16 pid)
{
	struct android_dev *dev = _android_dev;
    unsigned ui_state, ui_usb_state;
	USB_PR("%s pid=%d\n", __func__, pid);
    usb_get_state(&ui_state, &ui_usb_state);

	mutex_lock(&dev->lock);
	if (!product_matches_pid_to_functions(pid))
	{
		USB_PR("%s pid=0x%x error\n", __func__, pid);
		usb_switch_para.inprogress = 0;
		mutex_unlock(&dev->lock);
		return;
	}

	device_desc.idProduct = __constant_cpu_to_le16(pid);
	if (dev->cdev)
		dev->cdev->desc.idProduct = device_desc.idProduct;

	product_id = pid;
	/* close all stored file in each lun */
    fsg_close_all_file();

	/* clear the serial number when switch to cdrom mode */
	if (pid == curr_usb_pid_ptr->cdrom_pid)
	{
	  set_usb_sn(NULL);
	}
	/* if the pid index is slate test, then set the serial number valid */
	else if ((SLATE_TEST_INDEX == usb_para_info.usb_pid_index)
	&& (pid == curr_usb_pid_ptr->norm_pid))
	{
	  if (0 == usb_para_data.usb_para.usb_serial[0])
	  {
	    /* if the serial number is null, set the default string */
	    set_usb_sn(USB_SN_STRING);
	  }
	  else
	  {
	    /* else set the usb serial number value */
	    set_usb_sn(usb_para_data.usb_para.usb_serial);
	  }
	}

/*2 means usb offline we don't need to force reenumeration otherwise usb event may lose*/
    if (2 != ui_state)
    {
      USB_PR("%s ui_state=%d usb_composite_force_reset\n", __func__, ui_state);
      /* force reenumeration */
	  usb_composite_force_reset(dev->cdev);
    }
	mutex_unlock(&dev->lock);
	usb_switch_para.inprogress = 0;
	/* open the cdrom lun file */
    fsg_store_file_again();
}

int serial_str_id=-1;//string id of sequence number in string descriptor
//return true if current product id is udisk id otherwise false
int in_usb_mode_normal_udisk(void)
{
     return product_id==curr_usb_pid_ptr->udisk_pid;
}

static int __devinit android_bind(struct usb_composite_dev *cdev)
{
	struct android_dev *dev = _android_dev;
	struct usb_gadget	*gadget = cdev->gadget;
	int			gcnum, id, ret;

	USB_PR("android_bind\n");

	/* Allocate string descriptor numbers ... note that string
	 * contents can be overridden by the composite_dev glue.
	 */
	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_MANUFACTURER_IDX].id = id;
	device_desc.iManufacturer = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_PRODUCT_IDX].id = id;
    device_desc.iProduct = id;

	id = usb_string_id(cdev);
	if (id < 0)
		return id;
	strings_dev[STRING_SERIAL_IDX].id = id;
//save this id to global variable wich will be used in enum later
	serial_str_id = id;
	if(0 == usb_sn_valid)
	{
		device_desc.iSerialNumber = 0;
	}
	else
	{
		device_desc.iSerialNumber = id;
	}

	if (gadget_is_otg(cdev->gadget))
		android_config_driver.descriptors = otg_desc;

	if (!usb_gadget_set_selfpowered(gadget))
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_SELFPOWER;

	if (gadget->ops->wakeup)
		android_config_driver.bmAttributes |= USB_CONFIG_ATT_WAKEUP;

	/* register our configuration */
	ret = usb_add_config(cdev, &android_config_driver);
	if (ret) {
		USB_PR("usb_add_config failed\n");
		return ret;
	}

	gcnum = usb_gadget_controller_number(gadget);
	if (gcnum >= 0)
		device_desc.bcdDevice = cpu_to_le16(0x0200 + gcnum);
	else {
		/* gadget zero is so simple (for now, no altsettings) that
		 * it SHOULD NOT have problems with bulk-capable hardware.
		 * so just warn about unrcognized controllers -- don't panic.
		 *
		 * things like configuration and altsetting numbering
		 * can need hardware-specific attention though.
		 */
		USB_PR("%s: controller '%s' not recognized\n",
			longname, gadget->name);
		device_desc.bcdDevice = __constant_cpu_to_le16(0x9999);
	}

	usb_gadget_set_selfpowered(gadget);
	dev->cdev = cdev;
	USB_PR("%s bind product_id=0x%x\n", __func__, product_id);
	device_desc.idProduct = __constant_cpu_to_le16(product_id);
	cdev->desc.idProduct = device_desc.idProduct;

	return 0;
}

static struct usb_composite_driver android_usb_driver = {
	.name		= "android_usb",
	.dev		= &device_desc,
	.strings	= dev_strings,
	.bind		= android_bind,
	.enable_function = android_enable_function,
};

u16 android_get_product_id(void)
{
    return product_id;
}

/* for initialization */
void android_set_product_id(u16 pid)
{
  product_id = pid;
}

/* set USB serial number
   typically called before USB switch
*/
void set_usb_sn(char *sn_ptr)
{
    int len = 0;
    int iSerialNumber = 0;
    struct android_dev *dev = _android_dev;

    if(sn_ptr == NULL)
    {
        usb_sn_valid = 0;
        /* if sn_ptr is null, set the iSerialNumber to zero for no serial */
        iSerialNumber = 0;
    }
    else
    {
        len = strlen(sn_ptr);
        if (len >= (MAX_NAME_LEN-1)) 
        {
            USB_PR("serial number string too long\n");
            return;
        }
        strncpy(serial_number, sn_ptr, len);
        serial_number[len] = '\0';
        usb_sn_valid = 1;
        /* if sn_ptr isn't null, set the iSerialNumber to serial_str_id */
        if (-1 != serial_str_id)
        {
            iSerialNumber = serial_str_id;
        }
    }

    if (dev && dev->cdev)
    {
      /* if dev and cdev isn't null, set the serial number to composite device */
      USB_PR("%s iSerialNumber=%d\n", __func__, iSerialNumber);
      dev->cdev->desc.iSerialNumber = iSerialNumber;
    }
}

/* call back function for delay work */
static void android_usb_switch_func(struct work_struct *w)
{
  USB_PR("lxy: %s\n", __func__); 
  switch_composite_function(usb_switch_dest_pid);
}

/* Called in usb_prepare() */
void android_delay_work_init(int add_flag)
{
    static int delay_work_init_flag = 0;

    if(1 == add_flag)
    {
        if(0 == delay_work_init_flag)
        {
            USB_PR("lxy: %s, initialize OK.\n", __func__);
            delay_work_init_flag = 1;
            INIT_DELAYED_WORK(&android_usb_switch_work, android_usb_switch_func);
        }
        else
        {
            USB_PR("lxy: %s, already initialized.\n", __func__);
        }
    }
    else
    {
        if(1 == delay_work_init_flag)
        {
            USB_PR("lxy: %s, cancel OK.\n", __func__);
            delay_work_init_flag = 0;
          cancel_delayed_work_sync(&android_usb_switch_work);
        }
        else
        {
            USB_PR("lxy: %s, already cancelled.\n", __func__);
        }
    }
}

/* Global interface for USB switch
   pid:             destination product id
   delay_tick:      delay time to initiate the switch, 1 tick = 10ms
*/

int usb_switch_composition(u16 pid, unsigned long delay_tick)
{  
  USB_PR("lxy: %s, pid=0x%x\n", __func__, pid);
     
  if (product_id == pid)
  {
    USB_PR("lxy: %s, pid=0x%x is same, ignore it\n", __func__, pid);
    usb_switch_para.inprogress = 0;
    return 0;
  }
  
  usb_switch_dest_pid = pid;
    
  schedule_delayed_work(&android_usb_switch_work, delay_tick);

  return 0;
}

void kernel_set_adb_enable(u8 enable)
{
    struct android_dev *dev = _android_dev;
    dev->adb_enable = enable;
}

void android_register_function(struct android_usb_function *f)
{
	struct android_dev *dev = _android_dev;

	USB_PR("android_register_function %s count=%d\n", f->name, _registered_function_count+1);
	list_add_tail(&f->list, &_functions);
	_registered_function_count++;

	/* bind our functions if they have all registered
	 * and the main driver has bound.
	 */
	if (dev->config && _registered_function_count == dev->num_functions) {
		bind_functions(dev);
		(void)product_matches_pid_to_functions(product_id);
	}
}

static void matche_rndis_and_comm_function(int isrndis)
{
	struct android_dev *dev = _android_dev;
	/* We need to specify the COMM class in the device descriptor
	* if we are using RNDIS.
	*/
	if (isrndis) {
		USB_PR("%s rndis desc\n", __func__);
		dev->cdev->desc.bDeviceClass = USB_CLASS_COMM;
	} else {
		USB_PR("%s common desc\n", __func__);
		dev->cdev->desc.bDeviceClass = USB_CLASS_PER_INTERFACE;
		dev->cdev->desc.bDeviceSubClass      = 0;
		dev->cdev->desc.bDeviceProtocol      = 0;
	}
}

static struct android_usb_product_hw *android_get_product_from_pid(int pid)
{
  struct android_dev *dev = _android_dev;
  struct android_usb_product_hw *up = dev->products;
  int index;
  USB_PR("%s \n", __func__);
  for (index = 0; index < dev->num_products; index++, up++) {
    if (pid == up->product_id || pid == up->adb_product_id)
    {
      return up;
    }
  }
  
  return 0;
}

int get_current_ms_cdrom_index(void)
{
  struct android_usb_product_hw* p = android_get_product_from_pid(product_id);
  if (0 == p)
  {
    USB_PR("%s curr cdrom_index=0 pid=%d\n", __func__, product_id);
    return 0;
  }

  USB_PR("%s curr cdrom_index=%d pid=%d\n", __func__, p->cdrom_index, product_id);
  return p->cdrom_index;
}

int get_current_ms_lun(void)
{
  struct android_usb_product_hw* p = android_get_product_from_pid(product_id);
  if (0 == p)
  {
    USB_PR("%s curr lun=1 pid=%d\n", __func__, product_id);
    return 1;
  }

  USB_PR("%s curr lun=%d pid=%d\n", __func__, p->nluns, product_id);
  return p->nluns;
}


void android_enable_function(struct usb_function *f, int enable)
{
	struct android_dev *dev = _android_dev;
	int disable = !enable;
	struct android_usb_product_hw *up;
	USB_PR("%s func(%s) enable=%d, adb stauts=%d, fun hidden status=%d\n", 
	__func__, f->name, enable, dev->adb_enable, f->disabled);

	if (!!f->disabled == disable) {
		USB_PR("%s function is already status=%d\n", __func__, disable);
		return;
	}

	if (!strcmp(f->name, "adb"))
	{
        if (dev->adb_enable == enable)
        {
            USB_PR("%s function is already status=%d\n", __func__, disable);
              return;
        }
        dev->adb_enable = enable;
	}
    else if (!strcmp(f->name, "rndis"))
    {
        if (enable)
        {
            USB_PR("%s enable rndis\n", __func__);
            product_id = curr_usb_pid_ptr->wlan_pid;
        }
        else
        {
            USB_PR("%s disable rndis\n", __func__);
            product_id = usb_para_info.usb_pid; 
        }
    }
    else
    {
        USB_PR("%s cann't enable or disable fun(%s)\n", __func__, f->name);
        	return;
    }
  
	up = android_get_product_from_pid(product_id);
	if (0 == up)
	{
		USB_PR("%s cann't find pid=%d\n", __func__, product_id);
		return;
	}

    if (!strcmp(f->name, "adb")
    && (up->adb_product_id == up->product_id) 
    && (up->adb_num_functions == up->num_functions))
	{
		USB_PR("%s current pid(%d) can't need adb\n", __func__, product_id);
		return;
	}
	//switch to different product id according adb's state
	if(dev->adb_enable)
		switch_composite_function(up->adb_product_id);	
	else	
		switch_composite_function(up->product_id);
}

static int __devinit android_probe(struct platform_device *pdev)
{
	struct android_usb_platform_data *pdata = pdev->dev.platform_data;
	struct android_dev *dev = _android_dev;
	int result;

	USB_PR("android_probe pdata: %p\n", pdata);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);

	result = pm_runtime_get(&pdev->dev);
	if (result < 0) {
		USB_PR("Runtime PM: Unable to wake up the device, rc = %d\n", result);
		return result;
	}

	if (pdata) {
	dev->products = pdata->products;
	dev->num_products = pdata->num_products;
	dev->functions = pdata->functions;
	dev->num_functions = pdata->num_functions;
	if (pdata->vendor_id)
		device_desc.idVendor =
			__constant_cpu_to_le16(pdata->vendor_id);

	device_desc.idProduct =
	__constant_cpu_to_le16(product_id);

	if (pdata->version)
		dev->version = pdata->version;

	if (pdata->product_name)
		strings_dev[STRING_PRODUCT_IDX].s = pdata->product_name;
	if (pdata->manufacturer_name)
		strings_dev[STRING_MANUFACTURER_IDX].s =
				pdata->manufacturer_name;
	if (pdata->serial_number)
		strings_dev[STRING_SERIAL_IDX].s = serial_number;
	}

	if ((product_id == PID_GOOGLE_MS) 
		|| (SLATE_TEST_INDEX == usb_para_info.usb_pid_index))
	{
		dev->adb_enable = 0;
	}
	else
	{
		dev->adb_enable = 1;
	}

	return usb_composite_register(&android_usb_driver);
}

static int andr_runtime_suspend(struct device *dev)
{
	USB_PR("pm_runtime: suspending...\n");
	return 0;
}

static int andr_runtime_resume(struct device *dev)
{
	USB_PR("pm_runtime: resuming...\n");
	return 0;
}

static struct dev_pm_ops andr_dev_pm_ops = {
	.runtime_suspend = andr_runtime_suspend,
	.runtime_resume = andr_runtime_resume,
};

static struct platform_driver android_platform_driver = {
	.driver = { .name = "android_usb", .pm = &andr_dev_pm_ops},
	.probe = android_probe,
};

static int __devinit init(void)
{
	struct android_dev *dev;

	USB_PR("android init\n");
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	mutex_init(&dev->lock);
	/* set default values, which should be overridden by platform data */
	_android_dev = dev;

	return platform_driver_register(&android_platform_driver);
}
module_init(init);

static void __exit cleanup(void)
{
	usb_composite_unregister(&android_usb_driver);
	platform_driver_unregister(&android_platform_driver);
	kfree(_android_dev);
	_android_dev = NULL;
}
module_exit(cleanup);
