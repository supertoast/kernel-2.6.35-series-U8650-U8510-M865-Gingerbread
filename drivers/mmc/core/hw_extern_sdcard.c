/*
 * Copyright (c) 2010, HUAWEI. All rights reserved.
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

#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>

#include <linux/platform_device.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/atomic.h>

static int hw_extern_sdcard_probe(struct platform_device *pdev);
static int hw_extern_sdcard_open(struct inode *inode, struct file *file);
static ssize_t hw_extern_sdcard_read(struct file *file, char __user *buf,
			  size_t count, loff_t *pos);
static int hw_extern_sdcard_release(struct inode *inode, struct file *file);


static int hw_extern_sdcardMounted_probe(struct platform_device *pdev);
static ssize_t hw_extern_sdcardMounted_read(struct file *file, char __user *buf,
			  size_t count, loff_t *pos);
static ssize_t hw_extern_sdcardMounted_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *pos);


static struct platform_driver hw_extern_sdcard_driver = {
    .probe      = hw_extern_sdcard_probe,
	.driver	= {
		.name	= "hw_extern_sdcard",
		.owner	= THIS_MODULE,
	},
};

const struct file_operations hw_extern_sdcard_fops = {
	.owner = THIS_MODULE,
    .open = hw_extern_sdcard_open,
	.read = hw_extern_sdcard_read,
	.release	= hw_extern_sdcard_release,
};

static struct miscdevice hw_extern_sdcard_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hw_extern_sdcard",
	.fops = &hw_extern_sdcard_fops,
};


static struct platform_driver hw_extern_sdcardMounted_driver = {
    .probe      = hw_extern_sdcardMounted_probe,
	.driver	= {
		.name	= "hw_extern_sdcardMounted",
		.owner	= THIS_MODULE,
	},
};

const struct file_operations hw_extern_sdcardMounted_fops = {
	.owner = THIS_MODULE,
    .open = hw_extern_sdcard_open,
	.read = hw_extern_sdcardMounted_read,
	.write = hw_extern_sdcardMounted_write,
	.release	= hw_extern_sdcard_release,
};

static struct miscdevice hw_extern_sdcardMounted_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hw_extern_sdcardMounted",
	.fops = &hw_extern_sdcardMounted_fops,
};

// 1 means extern sdcard exists, otherwise 0 means not exist.
atomic_t hw_extern_sdcard_flag;
atomic_t hw_extern_sdcardMounted_flag;
// interface function. It`s called by mmc-host moudule when extern sdcard insert.
void hw_extern_sdcard_insert(void)
{
    atomic_set(&hw_extern_sdcard_flag,1);
}

// interface function. It`s called by mmc-host moudule when extern sdcard remouve.
void hw_extern_sdcard_remove(void)
{
    atomic_set(&hw_extern_sdcard_flag,0);
}


static int hw_extern_sdcard_probe(struct platform_device *pdev)
{
    int ret;
	
	/*hw_extern_sdcard_flag flag doesn't need initialization at here, becase it is 
	   a global variable, it has been initilized to zero by default. And device prob maybe
	   called after hw_extern_sdcard_insert, this will clear the hw_extern_sdcard_flag*/
    /*atomic_set(&hw_extern_sdcard_flag,0);*/
    pr_err("%s: enter-----> \n", __func__);

    ret = misc_register(&hw_extern_sdcard_device);
    if (ret) {
	pr_err("%s: Unable to register misc device.\n", __func__);
	return ret;
    }
    
    return 0;
}

static int hw_extern_sdcardMounted_probe(struct platform_device *pdev)
{
    int ret;
    atomic_set(&hw_extern_sdcardMounted_flag,0);
    
    pr_err("%s: enter-----> \n", __func__);

    ret = misc_register(&hw_extern_sdcardMounted_device);
    if (ret) {
	pr_err("%s: Unable to register misc device.\n", __func__);
	return ret;
    }
    
    return 0;
}

// open file.
static int hw_extern_sdcard_open(struct inode *inode, struct file *file)
{
    return 0;
}

static ssize_t hw_extern_sdcard_read(struct file *file, char __user *buf,
			  size_t count, loff_t *pos)
{
    const int buffer_len = 2;
    char buffer[buffer_len];
    
    // read buffer length must biger than 2 bytes.
    if (2 > count) {
		pr_err("%s read failed! read count error.\n",__func__);
		return -1;
    }

    // read mount must be 1st because only the first byte is valid.
    if (1 < *pos) {
		pr_err("%s read failed! read pos error.\n",__func__);
		return -1;
    }

    memset(buffer,0,sizeof(buffer));
    sprintf(buffer, "%d", atomic_read(&hw_extern_sdcard_flag));

    if (copy_to_user(buf, buffer, buffer_len)) {
        return -1;
    }
    
	return buffer_len;
}

static ssize_t hw_extern_sdcardMounted_read(struct file *file, char __user *buf,
			  size_t count, loff_t *pos)
{
    const int buffer_len = 2;
    char buffer[buffer_len];
    
    // read buffer length must biger than 2 bytes.
    if (2 > count) {
		pr_err("%s read failed! read count error.\n",__func__);
		return -1;
    }

    // read mount must be 1st because only the first byte is valid.
    if (1 < *pos) {
		pr_err("%s read failed! read pos error.\n",__func__);
		return -1;
    }

    memset(buffer,0,sizeof(buffer));
    sprintf(buffer, "%d", atomic_read(&hw_extern_sdcardMounted_flag));

    if (copy_to_user(buf, buffer, buffer_len)) {
        return -1;
    }
    
	return buffer_len;
}
static ssize_t hw_extern_sdcardMounted_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *pos)
{
    char tmp[2];

    if (count > 2){ 
	count=2;
    }
	
    memset(tmp,0,sizeof(tmp));
    if(count != 0){
	if(copy_from_user(tmp, buf, count)){
	  return -1;
	}
	if(tmp[0]=='0')
	  atomic_set(&hw_extern_sdcardMounted_flag,0);
	  if(tmp[0]=='1')
	    atomic_set(&hw_extern_sdcardMounted_flag,1);
    }  
    return 1;
}

static int hw_extern_sdcard_release(struct inode *inode, struct file *file)
{
	return 0;
}

static int __init hw_extern_sdcard_init(void)
{
	int ret=platform_driver_register(&hw_extern_sdcard_driver);
	if(ret)
		return ret;

	ret=platform_driver_register(&hw_extern_sdcardMounted_driver);

	if(ret)
		platform_driver_unregister(&hw_extern_sdcard_driver);

	return ret;
}
late_initcall(hw_extern_sdcard_init);


static void __exit hw_extern_sdcard_exit(void)
{
	platform_driver_unregister(&hw_extern_sdcard_driver);
	platform_driver_unregister(&hw_extern_sdcardMounted_driver);
}
module_exit(hw_extern_sdcard_exit);

