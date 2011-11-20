/*
 * SMEM Sleep log driver.
 * Allows a user space process to get the SMEM log of sleep.
 *
 * Copyright (c) 2011 HUAWEI <hujun@huawei.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the smems of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/fs.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <linux/pci.h>
#include <linux/smp_lock.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include "smd_private.h"
#include <linux/slab.h>
#include "smem_sleep_log.h"

#define NAME			"smem_sleep_log"

MODULE_AUTHOR("hw <hw@huawei.com>");
MODULE_DESCRIPTION("SMEM sleep log Driver");
MODULE_LICENSE("GPL");

static int major;
module_param(major, int, 0);
MODULE_PARM_DESC(major, "Major device number");

//static struct cdev smem_sleep_log_cdev;
//static struct class *smem_sleep_log_class;
struct smem_log_data {
	struct cdev cdev;
	struct device *pdevice; 
	struct class *kt_class;
    struct semaphore open_sem;
    size_t buf_offset;
    size_t buf_size;    
};

struct smem_log_data *smem_log_dev;


/* reserve 32 entries even though some aren't usable */
#define SMEM_SLEEP_LOG_COUNT	32

/* IO block size */
#define SMEM_SLEEP_LOG_SIZE	256

/*buffer to save smem sleep log*/
char * psmem_buffer = NULL;
#define MAX_SMEM_SLEEP_LOG_BUF_SIZE 40000 /*5*unit32 = 20 bytes*/
#define MAX_SMEM_SLEEP_LOG_EVENT_BUF_SIZE	MAX_SMEM_SLEEP_LOG_BUF_SIZE
#define MAX_SMEM_SLEEP_VOTER_BUF_SIZE	1152  /*2 * MAX_NUM_SLEEP_CLIENTS * (SLEEPLOG_CLIENT_LIST_MAX_STR + 1)*/

#define SMEM_SLEEP_LOG_DEBUG(fmt, args...) printk(KERN_DEBUG fmt, ##args)

static ssize_t smem_sleep_log_read(struct file *file, char __user *buf,
				size_t len, loff_t *ppos)
{
    struct smem_log_data *dev = (struct smem_log_data *)file->private_data;
    size_t copy_len =0;
    if(dev->buf_offset >= dev->buf_size)
    {
        /*copy is finished */
        return 0; 
    }
    else
    {
       copy_len = ((dev->buf_size - dev->buf_offset) > len) ? len : (dev->buf_size - dev->buf_offset);
    }

    if(NULL != psmem_buffer)
    {   
        /*copy data to user space */
        if(copy_to_user((void __user *)buf, psmem_buffer + dev->buf_offset, copy_len))
        {
            SMEM_SLEEP_LOG_DEBUG(KERN_ERR "smem_sleep_log_read: Fail to copy memory to user space !\n");
            return -EFAULT;       
        }
        dev->buf_offset += copy_len;
    }
    else
    {
        return -ENOMEM;
    }

	return copy_len;
}

static int smem_sleep_log_open(struct inode *inode, struct file *file)
{
    if (down_interruptible(&smem_log_dev->open_sem))
    {
    	SMEM_SLEEP_LOG_DEBUG(KERN_ERR "smem_open: can not get open_sem!\n");
        return -ERESTARTSYS; 
    }
	file->private_data = smem_log_dev; 
    smem_log_dev->buf_size = 0; 
    /*delete 6 lines*/
	return 0;

}
static int smem_sleep_log_release(struct inode *inode, struct file *file)
{
    psmem_buffer = NULL;
	up(&smem_log_dev->open_sem);
	return 0;
}

/*not malloc memory by kmalloc any more, and uste the memory get by smem_alloc directly */
static long smem_sleep_log_ioctl(struct file *file, unsigned int cmd,
	unsigned long arg)
{
	int ret = 0;
    struct smem_log_data *dev = (struct smem_log_data *)file->private_data;
    dev->buf_offset = 0;

    switch(cmd)
    {
        /*copy Pwr envent from SMEM to memory*/
        case MSM_SMEM_SLEEP_LOG_IOCTL_GET_PWR_EVENT:
            psmem_buffer = smem_alloc(SMEM_SMEM_LOG_POWER_EVENTS,MAX_SMEM_SLEEP_LOG_EVENT_BUF_SIZE);
            dev->buf_size = MAX_SMEM_SLEEP_LOG_EVENT_BUF_SIZE;        
            if(NULL == psmem_buffer)
            {
                SMEM_SLEEP_LOG_DEBUG(KERN_ERR "smem_sleep_log_ioctl: failed get PWR event\n");
                return -ENOMEM;

            }
            break;
        /*copy sleep voters from SMEM to memory*/
        case MSM_SMEM_SLEEP_LOG_IOCTL_GET_SLEEP_VOTER:
            psmem_buffer = smem_alloc(SMEM_SLEEP_STATIC,MAX_SMEM_SLEEP_VOTER_BUF_SIZE);
            dev->buf_size = MAX_SMEM_SLEEP_VOTER_BUF_SIZE;
            if(NULL == psmem_buffer)
            {
                SMEM_SLEEP_LOG_DEBUG(KERN_ERR "smem_sleep_log_ioctl: failed get Sleep voter\n");
                return -ENOMEM;
            }

            break;
        default: 
            break;
            
    }
	return ret;
}

static const struct file_operations smem_sleep_log_fops = {
	.owner	= THIS_MODULE,
	.unlocked_ioctl = smem_sleep_log_ioctl,
	.read	= smem_sleep_log_read,
	.open	= smem_sleep_log_open,
	.release = smem_sleep_log_release
};

static int __init smem_sleep_log_init(void)
{
	dev_t	dev_id;
	u32	low, hi;
	int	retval;
    int error;
    int ret = 0;
        
	smem_log_dev = kzalloc(sizeof(struct smem_log_data), GFP_KERNEL);	
    if (!smem_log_dev)
    {
        SMEM_SLEEP_LOG_DEBUG(KERN_ERR "smem_sleep_log_init: Unable to alloc memory for device\n"); 
		return (-ENOMEM);
    }


	/*init mutex*/
	init_MUTEX(&smem_log_dev->open_sem); 

	if (major) {
		dev_id = MKDEV(major, 0);
		retval = register_chrdev_region(dev_id, SMEM_SLEEP_LOG_COUNT,
						NAME);
	} else {
		retval = alloc_chrdev_region(&dev_id, 0, SMEM_SLEEP_LOG_COUNT,
					     NAME);
		major = MAJOR(dev_id);
	}

	if (retval) {
        SMEM_SLEEP_LOG_DEBUG(KERN_ERR "smem_sleep_log cant get major\n");
        kfree(smem_log_dev);
		return -1;
	}
    
    smem_log_dev->kt_class = class_create(THIS_MODULE, NAME); 
    if (IS_ERR(smem_log_dev->kt_class)) 
    {
        SMEM_SLEEP_LOG_DEBUG(KERN_ERR "failed to class_create\n"); 
        goto can_not_create_class; 
    }

    smem_log_dev->pdevice = device_create(smem_log_dev->kt_class, NULL, dev_id, "%s", NAME); 
	if (IS_ERR(smem_log_dev->pdevice)) {
		SMEM_SLEEP_LOG_DEBUG(KERN_ERR "Can't create smem log device\n");
		goto can_not_create_class;
	}

	cdev_init(&(smem_log_dev->cdev), &smem_sleep_log_fops);
    smem_log_dev->cdev.owner = THIS_MODULE;
    
	error = cdev_add(&(smem_log_dev->cdev), dev_id, SMEM_SLEEP_LOG_COUNT);
    if (error) {
        SMEM_SLEEP_LOG_DEBUG(KERN_ERR "init_key_test_cdev: Failed  cdev_add\n");
        error = ENOENT;
        goto can_not_add_cdev;
    }
  
    return 0;
    can_not_add_cdev:
       class_unregister(smem_log_dev->kt_class); 
    can_not_create_class:
       unregister_chrdev_region(dev_id, 1);
       kfree(smem_log_dev);
       return error;
}

static void __exit smem_sleep_log_cleanup(void)
{
	dev_t dev_id = MKDEV(major, 0);
    if(smem_log_dev)
    {
        kfree(smem_log_dev);
        smem_log_dev = NULL;
    }

	cdev_del(&(smem_log_dev->cdev));
    class_unregister(smem_log_dev->kt_class);     
	unregister_chrdev_region(dev_id, SMEM_SLEEP_LOG_COUNT);
}

module_init(smem_sleep_log_init);
module_exit(smem_sleep_log_cleanup);

