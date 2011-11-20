/* drivers/input/touchscreen/synaptics_i2c_rmi_tm.c
 *
 * Copyright (C) 2009 HUAWEI.
 **
 */

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/synaptics_i2c_rmi.h>

#include <mach/gpio.h>
#include <mach/vreg.h>

#include <linux/hardware_self_adapt.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include <linux/namei.h>
#include <linux/vmalloc.h>

#include <linux/touch_platform_config.h>
#include "linux/hardware_self_adapt.h"
#include <linux/slab.h>

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif


/*
 * DEBUG SWITCH
 *
 */ 
//#define TS_DEBUG 
#undef TS_DEBUG 

#ifdef TS_DEBUG
#define TS_DEBUG_RMI(fmt, args...) printk(KERN_DEBUG fmt, ##args)
#else
#define TS_DEBUG_RMI(fmt, args...)
#endif

/*use this to contrl the debug message*/
/* debug info */
static int synaptics_debug_mask_t1021;
module_param_named(synaptics_debug, synaptics_debug_mask_t1021, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define DBG_MASK(x...) do {\
	if (synaptics_debug_mask_t1021) \
		printk(KERN_DEBUG x);\
	} while (0)

#define GPIO_TOUCH_INT   29

#define LCD_X_MAX    320
#define LCD_Y_MAX    480
/* height of virtual key */
#define LCD_VIRTUAL_KEY_H 20

enum
{
    F01_RMI_DATA00  = 0x13,
    F01_RMI_CTRL00  = 0x23,
    F01_RMI_CTRL01_00 = 0x24,
    F01_RMI_CMD00   = 0x67,
    F01_RMI_QUERY00 = 0x6E,

    F01_RMI_DATA0 = 0x13,
    F01_RMI_CTLR0 = 0x25,
    F01_RMI_QUERY0 = 0x6E,

    F34_RMI_QUERY0 = 0x65,
    F34_RMI_QUERY1 = 0x66,
    F34_RMI_QUERY2 = 0x67,
    F34_RMI_QUERY3 = 0x68,
    F34_RMI_QUERY4 = 0x69,
    F34_RMI_QUERY5 = 0x6A,
    F34_RMI_QUERY6 = 0x6B,
    F34_RMI_QUERY7 = 0x6C,
    F34_RMI_QUERY8 = 0x6D,
    
    F34_RMI_DATA0 = 0x00,
    F34_RMI_DATA1 = 0x01,
    F34_RMI_DATA2 = 0x02,
    F34_RMI_DATA3 = 0x12,
	
};

#define  Manufacturer_ID  0x01

//----------------------------------------------
//TOUCHSCREEN_EXTRA_KEY
//----------------------------------------------
//#ifdef CONFIG_HUAWEI_TOUCHSCREEN_EXTRA_KEY

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef abs
#define abs(a)  ((0 < (a)) ? (a) : -(a))
#endif

/* TS_X_MAX=2060, TX_Y_MAX=3264, offset=32 */

#define TS_X_MAX     2060
#define TS_Y_MAX     3264
#define TS_KEY_Y_MAX 248

static int lcd_x = 0;
static int lcd_y = 0;
static int virtual_key_h = 0;

#define X_START    (0)
#define X_END      (TS_X_MAX) 
#define Y_START    (TS_Y_MAX-TS_KEY_Y_MAX+50)
#define Y_END      (TS_Y_MAX)

#define EXTRA_MAX_TOUCH_KEY    4
#define TS_KEY_DEBOUNCE_TIMER_MS 60

#ifdef CONFIG_SYNAPTICS_UPDATE_TS_FIRMWARE 
static struct i2c_client *g_client = NULL;
static ssize_t update_firmware_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf);
static ssize_t update_firmware_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count);

static int ts_firmware_file(void);
static int i2c_update_firmware(struct i2c_client *client); 

static struct kobj_attribute update_firmware_attribute = {
	.attr = {.name = "update_firmware", .mode = 0666},
	.show = update_firmware_show,
	.store = update_firmware_store,
};
#endif

/* to define virt button of touch panel */
typedef struct 
{
    u16  center_x;
    u16  center_y;
    u16  x_width;
    u16  y_width;
    u32   touch_keycode;
} button_region;

struct synaptics_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *synaptics_wq;
	struct work_struct  work;
	int use_irq;
	struct hrtimer timer;	
	int (*power)(struct i2c_client* client, int on);
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
	int reported_finger_count;
	int max_x;
	int max_y;
	int last_x;
	int last_y;
	int x_offset;
	int y_offset;
	bool is_first_point;
	bool use_touch_key;
	bool move_fast;
	bool is_surport_fingers;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h);
static void synaptics_ts_late_resume(struct early_suspend *h);
#endif

static int synaptics_ts_power(struct i2c_client *client, int on);

static int synaptics_init_panel_data(struct synaptics_ts_data *ts)
{
	int ret = true;
	int i = 0;
	int value[4] = {0};

	ts->is_first_point = true;
	for(i=0; i<4; i++)
    {
//		value[i] = i2c_smbus_read_byte_data(ts->client, 0x2D+i);
		value[i] = i2c_smbus_read_byte_data(ts->client, 0x2B+i);
		DBG_MASK("%s: Cannot support multi fingers!0x2B + %d = 0x%x\n", __FUNCTION__, i, value[i]);

		if (value[i] < 0) 
        {
			ret = value[i];
			printk(KERN_ERR "i2c_smbus_read_byte_data failed\n");
			goto err_init_panel_data_failed;
		}
	}
	ts->max_x = value[0] | (value[1] << 8);
	ts->max_y = value[2] | (value[3] << 8);

	ts->x_offset = 4*(ts->max_x/LCD_X_MAX);  /*4 pix*/
	ts->y_offset = ts->x_offset;

    ts->use_touch_key = false;

	printk(KERN_INFO "%s: use_touch_key=%d, is_surport_fingers=%d,\n TS_X_MAX=%d, TX_Y_MAX=%d, offset=%d\n",
	                 __FUNCTION__, ts->use_touch_key, ts->is_surport_fingers, ts->max_x, ts->max_y, ts->x_offset);
    
err_init_panel_data_failed:
	return ret;
}

static void synaptics_ts_work_func(struct work_struct *work)
{
	int i;
	int ret;
	int bad_data = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[18];
	uint16_t x, y;
	uint8_t z,wx,wy;
	uint8_t finger;
	uint8_t gesture0;
	uint8_t gesture1;
	uint8_t device_st;
	uint8_t irq_st;

	uint16_t x2, y2, wx2,wy2,z2;
	uint8_t finger2_pressed = 0;

	struct synaptics_ts_data *ts = container_of(work, struct synaptics_ts_data, work);

	start_reg = F01_RMI_DATA00;
	msg[0].addr = ts->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	msg[1].addr = ts->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = sizeof(buf);
	msg[1].buf = buf;
	DBG_MASK("synaptics_ts_work_func\n"); 

	for (i = 0; i < ((ts->use_irq && !bad_data) ? 1 : 5); i++) { 	
		ret = i2c_transfer(ts->client->adapter, msg, 2);
		if (ret < 0) 
		{
			pr_err("%d times i2c_transfer failed\n",i);
			bad_data = 1;
			continue;
		}
		else
		{
		    bad_data = 0;
		}
		if (i == 5) 
		{
			pr_err("%d times i2c_transfer error\n", i);
			if (ts->power) {
				ret = ts->power(ts->client,1);
				if (ret < 0)
					pr_err("%s:synaptics_ts_resume power off failed\n", __FUNCTION__);
			}
			break;
		}
		
		if (ret < 0) {
			DBG_MASK(KERN_ERR "synaptics_ts_work_func: i2c_transfer failed\n");
			bad_data = 1;
		} else {
			bad_data = 0;	
			device_st = buf[0];
			irq_st = buf[1];
			x = (buf[5] & 0x0f) | ((uint16_t)buf[3] << 4); /* x aixs */ 
			y= ((buf[5] & 0xf0) >> 4) | ((uint16_t)buf[4] << 4);  /* y aixs */ 
			z = buf[7];				    /* pressure */	
			wx = buf[6] & 0x0f;			    /* x width */ 	
			wy = (buf[6] & 0x1f) >> 4;			    /* y width */ 	
			finger = buf[2] & 0x0f;                        /* numbers of fingers */  
			gesture0 = buf[13];		            /* code of gesture */ 
			gesture1 = buf[14];                         /* enhanced data of gesture */  
			DBG_MASK("device_st = 0x%x irq_st = 0x%x x = %d y = %d z = %d wx = %d wy = %d finger = %d gesture0 = 0x%x gesture1 = 0x%x\n",
					device_st, irq_st, x, y, z, wx, wy, finger,gesture0,gesture1);
			x2 = (buf[10] & 0x0f) | ((uint16_t)buf[8] << 4);
			y2 = ((buf[10] & 0xf0) >> 4) | ((uint16_t)buf[9] << 4);  /* y2 aixs */ 
			z2 = buf[12];
			wx2 = buf[11] & 0x0f;			    /* x width */ 	
			wy2 = (buf[11] & 0xf0) >> 4;			    /* y width */
			finger2_pressed = (finger&0x0C)>>2;
			DBG_MASK("finger0_state = 0x%x, x = %d y = %d z = %d wx = %d wy = %d\n",
					finger&0x03, x, y, z, wx, wy);
			DBG_MASK("finger1_state = 0x%x, x2 = %d y2 = %d z2 = %d wx2 = %d wy2 = %d\n",
					(finger&0x0C)>>2, x2, y2, z2, wx2, wy2);

            x2 = x2*320/TS_X_MAX;;
            y2 = (ts->max_y- y2)*531/TS_Y_MAX;
            
            x = x*320/TS_X_MAX;
            y = (ts->max_y- y)*531/TS_Y_MAX;

			DBG_MASK("finger0_state = 0x%x, x = %d y = %d z = %d wx = %d wy = %d\n",
					finger&0x03, x, y, z, wx, wy);
			DBG_MASK("finger1_state = 0x%x, x2 = %d y2 = %d z2 = %d wx2 = %d wy2 = %d\n",
					(finger&0x0C)>>2, x2, y2, z2, wx2, wy2);          

			if(ts->is_surport_fingers)
			{
			   /*
			 if (z) {
					input_report_abs(ts->input_dev, ABS_X, x);
					input_report_abs(ts->input_dev, ABS_Y, y);
				}
				input_report_abs(ts->input_dev, ABS_PRESSURE, z);
				input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, wy);
				input_report_key(ts->input_dev, BTN_TOUCH, finger);
			   */
				if (!finger)
					z = 0;
				input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, z);
				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, wy);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x);
				input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y);
				input_mt_sync(ts->input_dev);
				if (finger2_pressed) {
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, z2);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, wy2);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_X, x2);
					input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, y2);
					input_mt_sync(ts->input_dev);
				} else if (ts->reported_finger_count > 1) {
					input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
					input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
					input_mt_sync(ts->input_dev);
				}
				ts->reported_finger_count = finger;
				input_sync(ts->input_dev);
				
				goto ts_surport_fingers_end;
			}
            
			if (z) 
			{
				/* 
				* always report the first point  whether slip	or click
				*/ 
				if (ts->is_first_point) 
                {

					DBG_MASK("is_first_point  \n");
					ts->last_x = x;
					ts->last_y = y;
					ts->move_fast = false;
					ts->is_first_point = false;

					input_report_abs(ts->input_dev, ABS_X, x);
					input_report_abs(ts->input_dev, ABS_Y, y);

				} else 
				{
					DBG_MASK("else is_first_point  \n");
					if((abs(ts->last_x - x) > ts->x_offset) || (abs(ts->last_y - y) > ts->y_offset))
					{
						if(abs(ts->last_y - y) > (6*ts->max_y/LCD_Y_MAX))
						{
							ts->move_fast = true;
						}
						else
						{
							ts->move_fast = false;
						}
						ts->last_x = x;
						ts->last_y = y;
						
						DBG_MASK("else update_pen_and_key_state  \n");
						input_report_abs(ts->input_dev, ABS_X, x);
						input_report_abs(ts->input_dev, ABS_Y, y);

					}
				}
			}
			else
			{
				/* 
				* The next point must be first point whether slip or click after 
				* this up event
				*/ 		
				ts->is_first_point = true;
			}

			if(!ts->use_touch_key)
			{
				input_report_abs(ts->input_dev, ABS_PRESSURE, z);
				//input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, w);
				input_report_key(ts->input_dev, BTN_TOUCH, finger);
				input_sync(ts->input_dev);
			}
		}

	}
	
ts_surport_fingers_end:
	if (ts->use_irq) {
		enable_irq(ts->client->irq);
		DBG_MASK("enable irq\n");
	}
}
static enum hrtimer_restart synaptics_ts_timer_func(struct hrtimer *timer)
{
	struct synaptics_ts_data *ts = container_of(timer, struct synaptics_ts_data, timer);
	DBG_MASK("synaptics_ts_timer_func\n");
	queue_work(ts->synaptics_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 12500000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t synaptics_ts_irq_handler(int irq, void *dev_id)
{
	struct synaptics_ts_data *ts = dev_id;
	disable_irq_nosync(ts->client->irq);
//	disable_irq(ts->client->irq);
	DBG_MASK("synaptics_ts_irq_handler,disable irq\n");
	queue_work(ts->synaptics_wq, &ts->work);
	return IRQ_HANDLED;
}

static int synaptics_ts_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	struct synaptics_ts_data *ts;
	int ret = 0;
	int i;
	struct synaptics_i2c_rmi_platform_data *pdata;

    struct touch_hw_platform_data *touch_pdata;

    touch_pdata = client->dev.platform_data;

    DBG_MASK(" In synaptics_ts_probe: \n");

    if(NULL == touch_pdata)
    {
        printk("the touch_pdata is NULL please check the init code !\n");
        goto err_platform_data_init_failed;
    }

    if(touch_pdata->read_touch_probe_flag)
    {
        ret = touch_pdata->read_touch_probe_flag();
    }
    if(ret)
    {
        printk(KERN_ERR "%s: the touch driver has detected! \n", __func__);
        return -1;
    }
    else
    {
        printk(KERN_ERR "%s: it's the first touch driver! \n", __func__);
    }

    lcd_x = LCD_X_MAX;
    lcd_y = LCD_Y_MAX;
    /* height of virtual key */
    virtual_key_h = LCD_VIRTUAL_KEY_H;
    
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("synaptics_ts_probe: need I2C_FUNC_I2C\n");
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
	
	/* power on touchscreen */
	/*make power on*/
    if(touch_pdata->touch_power)
    {
        ret = touch_pdata->touch_power(1);
    }
    if(ret)
    {
        printk(KERN_ERR "%s: power on failed \n", __func__);
        goto err_power_on_failed;
    }

#if 0
    /* driver  detect its device  */  
	for(i = 0; i < 3; i++) 
    {
		ret = i2c_smbus_read_byte_data(client, 0x7D);
        DBG_MASK("synaptics_ts manufacturer id = %d\n", ret);

		if (ret == Manufacturer_ID){
			DBG_MASK("synaptics_ts manufacturer id = %d\n", ret); 
			goto succeed_find_device;
		}
	}
	if( i == 3) {
		pr_err("no synaptics-tm device\n ");	
		goto err_find_touchpanel_failed;
	}
#endif

succeed_find_device:
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) 
    {
		ret = -ENOMEM;
		goto err_alloc_data_failed;
	}

	ts->client = client;
	i2c_set_clientdata(client, ts);

	pdata = client->dev.platform_data;

    /* reset chip */
	ts->power = synaptics_ts_power;
	if (ts->power) 
    {
		ret = ts->power(ts->client, 1);
		if (ret < 0) 
        {
			pr_err("synaptics_ts_probe reset failed\n");
			goto err_power_failed;
		}
	}
    //msleep(200); /* wait for device reset; */

#ifdef CONFIG_SYNAPTICS_UPDATE_TS_FIRMWARE     
       g_client = client;  
       for (i = 0 ; i < 3; i++) {
            ret= ts_firmware_file();   
            if (!ret)
                break;
       }
#endif

	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL01_00, 0); /* disable interrupt */
	if(ret < 0)
    {
	    pr_err("%s: fail to disable interrupt!\n", __FUNCTION__);
	    goto err_power_failed;
	}

    ts->is_surport_fingers = client->flags;

	ret = synaptics_init_panel_data(ts);
	/*set tht bit means detected the tocuh yet in the next time we don't probe it*/
    if(touch_pdata->set_touch_probe_flag)
    {
        touch_pdata->set_touch_probe_flag(ret);
    }
	if (ret <= 0) {
		printk(KERN_ERR "Error synaptics_init_panel device (%d)\n", ret);
		ret = -ENODEV;
		goto err_power_failed;
	}

    #ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* set current device flag as present */
    set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
    #endif
    
	ts->synaptics_wq = create_singlethread_workqueue("synaptics_wq");
	if (!ts->synaptics_wq) 
    {
		pr_err("create synaptics_wq error\n");
		ret = -ENOMEM;
		goto err_destroy_wq;
	}
	INIT_WORK(&ts->work, synaptics_ts_work_func);
	
	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) 
    {
		ret = -ENOMEM;
		pr_err("synaptics_ts_probe: Failed to allocate input device\n");
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "Synaptics_T1021";
	
    set_bit(BTN_2, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(ABS_X, ts->input_dev->absbit);
	set_bit(ABS_Y, ts->input_dev->absbit);
    set_bit(KEY_NUMLOCK, ts->input_dev->keybit);
	input_set_abs_params(ts->input_dev, ABS_X, 0, lcd_x, 0, 0);
	/* touch pad y axis region must be 0-lcd_y */
	input_set_abs_params(ts->input_dev, ABS_Y, 0, lcd_y, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	if(ts->is_surport_fingers)
	{
		input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, lcd_x, 0, 0);
		/* touch pad y axis region must be 0-lcd_y */
		input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, lcd_y, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
		input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 15, 0, 0);
	}
	ret = input_register_device(ts->input_dev);
	if (ret) 
    {
		DBG_MASK(KERN_ERR "synaptics_ts_probe: Unable to register %s input device\n", ts->input_dev->name);
		goto err_input_register_device_failed;
	}

    if(touch_pdata->touch_gpio_config_interrupt)
    {
        ret = touch_pdata->touch_gpio_config_interrupt();
    }
    if (ret) 
	{
		ret = -EIO;
		goto err_key_input_register_device_failed;
	}

	if (client->irq) 
    {
		ret = gpio_request(irq_to_gpio(client->irq), client->name);
        DBG_MASK("the gpio is %d!\n", irq_to_gpio(client->irq));
		gpio_direction_input(irq_to_gpio(client->irq));

		DBG_MASK("Requesting IRQ...\n");

		if (request_irq(client->irq, synaptics_ts_irq_handler,
				IRQF_TRIGGER_LOW, client->name, ts) >= 0) 
	    {
			DBG_MASK("Received IRQ!\n");
            i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL01_00, 0x07); /* enable  int */
			ts->use_irq = 1;
			if (set_irq_wake(client->irq, 1) < 0)
				printk(KERN_ERR "failed to set IRQ wake\n");
		} else 
		{
			DBG_MASK("Failed to request IRQ!\n");
		}
	}

	if (!ts->use_irq) 
    {
		printk(KERN_ERR "Synaptics RMI4 device %s in polling mode\n", client->name);
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = synaptics_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = synaptics_ts_early_suspend;
	ts->early_suspend.resume = synaptics_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

	printk(KERN_INFO "synaptics_ts_probe: Start touchscreen %s in %s mode\n", ts->input_dev->name, ts->use_irq ? "interrupt" : "polling");

	return 0;

err_key_input_register_device_failed:

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_destroy_wq:
   	destroy_workqueue(ts->synaptics_wq);
    
err_power_failed:
	kfree(ts);
err_alloc_data_failed:
	
err_find_touchpanel_failed:
    ret = touch_pdata->touch_power(0);
    {
        printk(KERN_ERR "the power is off: gp5 = %d \n ", ret);   
    }

err_power_on_failed:
err_check_functionality_failed:
err_platform_data_init_failed:    
	return ret;
}

static int synaptics_ts_power(struct i2c_client *client, int on)
{
	int ret;
	if (on)
    {       
        ret = i2c_smbus_write_byte_data(client, F01_RMI_CTRL00, 0x80);/*sensor on*/	
		if (ret < 0)
        {
        	DBG_MASK(KERN_ERR "synaptics sensor can not wake up\n");
        }      
		
        ret = i2c_smbus_write_byte_data(client, F01_RMI_CMD00, 0x01);/*touchscreen reset*/

        if (ret < 0)
        DBG_MASK(KERN_ERR "synaptics chip can not reset\n");

        msleep(200); /* wait for device reset; */

	}
	else 
    {
		ret = i2c_smbus_write_byte_data(client, F01_RMI_CTRL00, 0x81); /* set touchscreen to deep sleep mode*/
		if (ret < 0)
			DBG_MASK(KERN_ERR "synaptics touch can not enter very-low power state\n");
	}
	return ret;	
}

static int synaptics_ts_remove(struct i2c_client *client)
{
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ts->early_suspend);
	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	input_unregister_device(ts->input_dev);

	if(ts->synaptics_wq)
	{
	   destroy_workqueue(ts->synaptics_wq); 
	}
	
	kfree(ts);
	return 0;
}

static int synaptics_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);
	DBG_MASK("In synaptics_ts_suspend\n");
	if (ts->use_irq)
		disable_irq(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
	ret = i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL01_00, 0); /* disable interrupt */
	if (ret < 0)
		DBG_MASK(KERN_ERR "synaptics_ts_suspend disable interrupt failed\n");
	if (ts->power) {
		ret = ts->power(client,0);
		if (ret < 0)
			DBG_MASK(KERN_ERR "synaptics_ts_resume power off failed\n");
	}
	return 0;
}

static int synaptics_ts_resume(struct i2c_client *client)
{
	int ret = 0;
	struct synaptics_ts_data *ts = i2c_get_clientdata(client);

	DBG_MASK("In synaptics_ts_resume\n");
	if (ts->power) {
		ret = ts->power(client, 1);
		if (ret < 0)
		{
			DBG_MASK("synaptics_ts_resume power on failed\n");
		}
	}
	
	if (ts->use_irq) {
		enable_irq(client->irq);
		ret =i2c_smbus_write_byte_data(ts->client, F01_RMI_CTRL01_00, 0x07); /* enable abs int */
		{
			if (ret < 0)
			{
				DBG_MASK("enable asb interrupt failed\n");		
				return ret;	
			}
		}
	}
	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void synaptics_ts_early_suspend(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void synaptics_ts_late_resume(struct early_suspend *h)
{
	struct synaptics_ts_data *ts;
	ts = container_of(h, struct synaptics_ts_data, early_suspend);
	synaptics_ts_resume(ts->client);
}
#endif


#ifdef CONFIG_SYNAPTICS_UPDATE_TS_FIRMWARE 

struct RMI4_FDT{
  unsigned char m_QueryBase;
  unsigned char m_CommandBase;
  unsigned char m_ControlBase;
  unsigned char m_DataBase;
  unsigned char m_IntSourceCount;
  unsigned char m_ID;
};

static int RMI4_read_PDT(struct i2c_client *client)
{
	// Read config data
	struct RMI4_FDT temp_buf;
	struct RMI4_FDT m_PdtF34Flash;
	struct RMI4_FDT m_PdtF01Common;
	struct i2c_msg msg[2];
	unsigned short start_addr; 

	memset(&m_PdtF34Flash,0,sizeof(struct RMI4_FDT));
	memset(&m_PdtF01Common,0,sizeof(struct RMI4_FDT));

	for(start_addr = 0xe9; start_addr > 10; start_addr -= sizeof(struct RMI4_FDT)){
		msg[0].addr = client->addr;
		msg[0].flags = 0;
		msg[0].len = 1;
		msg[0].buf = (unsigned char *)&start_addr;
		msg[1].addr = client->addr;
		msg[1].flags = I2C_M_RD;
		msg[1].len = sizeof(struct RMI4_FDT);
		msg[1].buf = (unsigned char *)&temp_buf;
		if(i2c_transfer(client->adapter, msg, 2) < 0){
			printk("%s:%d: read RIM4 PDT error!\n",__FUNCTION__,__LINE__);
			return -1;
		}

		if(temp_buf.m_ID == 0x34){
			memcpy(&m_PdtF34Flash,&temp_buf,sizeof(struct RMI4_FDT ));
		}else if(temp_buf.m_ID == 0x01){
			memcpy(&m_PdtF01Common,&temp_buf,sizeof(struct RMI4_FDT ));
		}else if (temp_buf.m_ID == 0){		//end of PDT
			break;
		}
	  }

	if((m_PdtF01Common.m_CommandBase != F01_RMI_CMD00) || (m_PdtF34Flash.m_QueryBase != F34_RMI_QUERY0)){
		printk("%s:%d: RIM4 PDT has changed!!!\n",__FUNCTION__,__LINE__);
		return -1;
	}

	return 0;

}

//to be improved .......
int RMI4_wait_attn(struct i2c_client * client,int udleay)
{
	int loop_count=0;
	int ret=0;

	do{
		mdelay(udleay);
		ret = i2c_smbus_read_byte_data(client,F34_RMI_DATA3);
		// Clear the attention assertion by reading the interrupt status register
		i2c_smbus_read_byte_data(client,F01_RMI_DATA0 + 1);
	}while(loop_count++ < 0x10 && (ret != 0x80));

	if(loop_count >= 0x10){
		DBG_MASK("RMI4 wait attn timeout:ret=0x%x\n",ret);
		return -1;
	}
	return 0;
}

int RMI4_disable_program(struct i2c_client *client)
{
	unsigned char cdata; 
	unsigned int loop_count=0;
  
	printk("RMI4 disable program...\n");
	// Issue a reset command
	i2c_smbus_write_byte_data(client,F01_RMI_CMD00,0x01);

	// Wait for ATTN to be asserted to see if device is in idle state
	RMI4_wait_attn(client,20);

	// Read F01 Status flash prog, ensure the 6th bit is '0'
	do{
		cdata = i2c_smbus_read_byte_data(client,F01_RMI_DATA0);
		udelay(2);
	} while(((cdata & 0x40) != 0) && (loop_count++ < 10));

	//Rescan the Page Description Table
	return RMI4_read_PDT(client);
}

static int RMI4_enable_program(struct i2c_client *client)
{
	unsigned short bootloader_id = 0 ;
	int ret = -1;
	printk("RMI4 enable program...\n");
	 // Read and write bootload ID
	bootloader_id = i2c_smbus_read_word_data(client,F34_RMI_QUERY0);
	i2c_smbus_write_word_data(client,F34_RMI_DATA2,bootloader_id);

	  // Issue Enable flash command
	if(i2c_smbus_write_byte_data(client, F34_RMI_DATA3, 0x0F) < 0){
		DBG_MASK("RMI enter flash mode error\n");
		return -1;
	}
	ret = RMI4_wait_attn(client,12);

	//Rescan the Page Description Table
	RMI4_read_PDT(client);
	return ret;
}

static unsigned long ExtractLongFromHeader(const unsigned char* SynaImage) 
{
  	return((unsigned long)SynaImage[0] +
         (unsigned long)SynaImage[1]*0x100 +
         (unsigned long)SynaImage[2]*0x10000 +
         (unsigned long)SynaImage[3]*0x1000000);
}

static int RMI4_check_firmware(struct i2c_client *client,const unsigned char *pgm_data)
{
	unsigned long checkSumCode;
	unsigned long m_firmwareImgSize;
	unsigned long m_configImgSize;
	unsigned short m_bootloadImgID; 
	unsigned short bootloader_id;
	const unsigned char *SynaFirmware;
	unsigned char m_firmwareImgVersion;
	unsigned short UI_block_count;
	unsigned short CONF_block_count;
	unsigned short fw_block_size;

  	SynaFirmware = pgm_data;
	checkSumCode = ExtractLongFromHeader(&(SynaFirmware[0]));
	m_bootloadImgID = (unsigned int)SynaFirmware[4] + (unsigned int)SynaFirmware[5]*0x100;
	m_firmwareImgVersion = SynaFirmware[7];
	m_firmwareImgSize    = ExtractLongFromHeader(&(SynaFirmware[8]));
	m_configImgSize      = ExtractLongFromHeader(&(SynaFirmware[12]));
 
	UI_block_count  = i2c_smbus_read_word_data(client,F34_RMI_QUERY5);
	fw_block_size = i2c_smbus_read_word_data(client,F34_RMI_QUERY3);
	CONF_block_count = i2c_smbus_read_word_data(client,F34_RMI_QUERY7);
	bootloader_id = i2c_smbus_read_word_data(client,F34_RMI_QUERY0);

	  return (m_firmwareImgVersion != 0 || bootloader_id == m_bootloadImgID) ? 0 : -1;

}


static int RMI4_write_image(struct i2c_client *client,unsigned char type_cmd,const unsigned char *pgm_data)
{
	unsigned short block_size;
	unsigned short img_blocks;
	unsigned short block_index;
	const unsigned char * p_data;
	int i;

	block_size = i2c_smbus_read_word_data(client,F34_RMI_QUERY3);
	switch(type_cmd ){
		case 0x02:
			img_blocks = i2c_smbus_read_word_data(client,F34_RMI_QUERY5);	//UI Firmware
			break;
		case 0x06:
			img_blocks = i2c_smbus_read_word_data(client,F34_RMI_QUERY7);	//Configure	
			break;
		default:
			DBG_MASK("image type error\n");
			goto error;
	}

	p_data = pgm_data;
	for(block_index = 0; block_index < img_blocks; ++block_index){
		printk("#");
		// Write Block Number
		if(i2c_smbus_write_word_data(client, F34_RMI_DATA0,block_index) < 0){
			DBG_MASK("write block number error\n");
			goto error;
		}

		for(i=0;i<block_size;i++){
			if(i2c_smbus_write_byte_data(client, F34_RMI_DATA2+i, *(p_data+i)) < 0){
				DBG_MASK("RMI4_write_image: block %d data 0x%x error\n",block_index,*p_data);
				goto error;
			}
			udelay(15);
		}
		p_data += block_size;	

		// Issue Write Firmware or configuration Block command
		if(i2c_smbus_write_word_data(client, F34_RMI_DATA3, type_cmd) < 0){
			DBG_MASK("issue write command error\n");
			goto error;
		}

		// Wait ATTN. Read Flash Command register and check error
		if(RMI4_wait_attn(client,5) != 0)
			goto error;
	}

	return 0;
error:
	return -1;
}


static int RMI4_program_configuration(struct i2c_client *client,const unsigned char *pgm_data )
{
	int ret;
	unsigned short block_size;
	unsigned short ui_blocks;

	printk("\nRMI4 program Config firmware...\n");
	block_size = i2c_smbus_read_word_data(client,F34_RMI_QUERY3);
	ui_blocks = i2c_smbus_read_word_data(client,F34_RMI_QUERY5);	//UI Firmware

	if(RMI4_write_image(client, 0x06,pgm_data+ui_blocks*block_size ) < 0){
		DBG_MASK("write configure image error\n");
		return -1;
	}
	ret = i2c_smbus_read_byte_data(client,F34_RMI_DATA3);
	return ((ret & 0xF0) == 0x80 ? 0 : ret);
}

static int RMI4_program_firmware(struct i2c_client *client,const unsigned char *pgm_data)
{
	int ret=0;
	unsigned short bootloader_id;

	printk("RMI4 program UI firmware...\n");

	//read and write back bootloader ID
	bootloader_id = i2c_smbus_read_word_data(client,F34_RMI_QUERY0);
	i2c_smbus_write_word_data(client,F34_RMI_DATA2, bootloader_id );
	//issue erase commander
	if(i2c_smbus_write_byte_data(client, F34_RMI_DATA3, 0x03) < 0){
		DBG_MASK("RMI4_program_firmware error, erase firmware error \n");
		return -1;
	}
	RMI4_wait_attn(client,300);

	//check status
	if((ret = i2c_smbus_read_byte_data(client,F34_RMI_DATA3)) != 0x80){
		return -1;
	}

	//write firmware
	if( RMI4_write_image(client,0x02,pgm_data) <0 ){
		DBG_MASK("write UI firmware error!\n");
		return -1;
	}

	ret = i2c_smbus_read_byte_data(client,F34_RMI_DATA3);
	return ((ret & 0xF0) == 0x80 ? 0 : ret);
}

static int synaptics_download(struct i2c_client *client,const unsigned char *pgm_data)
{
	int ret;

	ret = RMI4_read_PDT(client);
	if(ret != 0){
		printk("RMI page func check error\n");
		return -1;
	}

	ret = RMI4_enable_program(client);
	if( ret != 0){
		printk("%s:%d:RMI enable program error,return...\n",__FUNCTION__,__LINE__);
		goto error;
	}

	ret = RMI4_check_firmware(client,pgm_data);
	if( ret != 0){
		printk("%s:%d:RMI check firmware error,return...\n",__FUNCTION__,__LINE__);
		goto error;
	}

	ret = RMI4_program_firmware(client, pgm_data + 0x100);
	if( ret != 0){
		printk("%s:%d:RMI program firmware error,return...",__FUNCTION__,__LINE__);
		goto error;
	}

	RMI4_program_configuration(client, pgm_data +  0x100);
	return RMI4_disable_program(client);

error:
	RMI4_disable_program(client);
	printk("%s:%d:error,return ....",__FUNCTION__,__LINE__);
	return -1;

}

static int i2c_update_firmware(struct i2c_client *client) 
{
	char *buf;
	struct file	*filp;
    struct inode *inode = NULL;
	mm_segment_t oldfs;
    uint16_t	length;
	int ret = 0;
	const char filename[]="/sdcard/update/synaptics.img";

	/* open file */
    oldfs = get_fs();
    set_fs(KERNEL_DS);
    filp = filp_open(filename, O_RDONLY, S_IRUSR);
    if (IS_ERR(filp)) {
        printk("%s: file %s filp_open error\n", __FUNCTION__,filename);
        set_fs(oldfs);
        return -1;
    }

    if (!filp->f_op) {
        printk("%s: File Operation Method Error\n", __FUNCTION__);
        filp_close(filp, NULL);
        set_fs(oldfs);
        return -1;
    }

    inode = filp->f_path.dentry->d_inode;
    if (!inode) {
        printk("%s: Get inode from filp failed\n", __FUNCTION__);
        filp_close(filp, NULL);
        set_fs(oldfs);
        return -1;
    }

    /* file's size */
    length = i_size_read(inode->i_mapping->host);
	if (!( length > 0 && length < 62*1024 )){
		printk("file size error\n");
		filp_close(filp, NULL);
        set_fs(oldfs);
		return -1;
	}

	/* allocation buff size */
	buf = vmalloc(length+(length%2));		/* buf size if even */
	if (!buf) {
		printk("alloctation memory failed\n");
		filp_close(filp, NULL);
        set_fs(oldfs);
		return -1;
	}

    /* read data */
    if (filp->f_op->read(filp, buf, length, &filp->f_pos) != length) {
        printk("%s: file read error\n", __FUNCTION__);
        filp_close(filp, NULL);
        set_fs(oldfs);
		vfree(buf);
        return -1;
    }

	ret = synaptics_download(client,buf);

 	filp_close(filp, NULL);
    set_fs(oldfs);
	vfree(buf);
	return ret;
}

static int ts_firmware_file(void)
{
	int ret;
	struct kobject *kobject_ts;
	kobject_ts = kobject_create_and_add("touch_screen", NULL);
	if (!kobject_ts) {
		printk("create kobjetct error!\n");
		return -1;
	}
	ret = sysfs_create_file(kobject_ts, &update_firmware_attribute.attr);
	if (ret) {
		kobject_put(kobject_ts);
		printk("create file error\n");
		return -1;
	}
	return 0;	
}


/*
 * The "update_firmware" file where a static variable is read from and written to.
 */
static ssize_t update_firmware_show(struct kobject *kobj, struct kobj_attribute *attr,char *buf)
{
	return 1;
}


static ssize_t update_firmware_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int i;
	int ret = -1;

	printk("#################update_firmware_store######################\n");

	if ( (buf[0] == '2')&&(buf[1] == '\0') ) {

		/* driver detect its device  */
		for (i = 0; i < 3; i++) {	
			ret = i2c_smbus_read_byte_data(g_client, F01_RMI_QUERY00);
			if (ret == Manufacturer_ID){
				goto firmware_find_device;
			}

		}
		printk("Do not find synaptics device\n");	
		return -1;

firmware_find_device:

		disable_irq(g_client->irq);
		/*update firmware*/
		ret = i2c_update_firmware(g_client);
		enable_irq(g_client->irq);
 
		if( 0 != ret ){
			printk("Update firmware failed!\n");
			ret = -1;
		} else {
			printk("Update firmware success!\n");
			arm_pm_restart(0,&ret);
			ret = 1;
		}
	}
	
	return ret;
 }

#endif

static const struct i2c_device_id synaptics_ts_id[] = {
	{ "Synaptics_rmi_t1021", 0 },
	{ }
};

static struct i2c_driver synaptics_ts_driver = {
	.probe		= synaptics_ts_probe,
	.remove		= synaptics_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= synaptics_ts_suspend,
	.resume		= synaptics_ts_resume,
#endif
	.id_table	= synaptics_ts_id,
	.driver = {
		.name	= "Synaptics_rmi_t1021",
	},
};

static int __devinit synaptics_ts_init(void)
{
	return i2c_add_driver(&synaptics_ts_driver);
}

static void __exit synaptics_ts_exit(void)
{
	i2c_del_driver(&synaptics_ts_driver);
}

module_init(synaptics_ts_init);
module_exit(synaptics_ts_exit);

MODULE_DESCRIPTION("Synaptics Touchscreen Driver");
MODULE_LICENSE("GPL");

