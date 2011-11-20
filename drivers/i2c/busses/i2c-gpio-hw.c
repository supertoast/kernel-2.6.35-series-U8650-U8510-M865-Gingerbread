/*
 *           COPYRIGHT INFORMATION
 *
 *  Copyright (c) 2004 HUAWEI, Incorporated and its licensors.  All Rights
 *  Reserved.  HUAWEI Proprietary.  Export of this technology or software
 *  is regulated by the CHINA. Government. Diversion contrary to CHINA law prohibited.
 *
 *            EDIT HISTORY FOR MODULE
 *
 *  This section contains comments describing changes made to the module.
 *  Notice that changes are listed in reverse chronological order.
 *
 *  $Header: 
 *
 *  when       who     what, where, why
 */

#include <linux/i2c.h>
#include <linux/i2c-algo-bit.h>
#include <linux/i2c-gpio.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <asm/gpio.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <mach/board.h>
#include <linux/mutex.h>
#include <linux/timer.h>
#include <linux/remote_spinlock.h>
#include <linux/pm_qos_params.h>
#include <mach/gpio.h>
#include <linux/slab.h>

static unsigned int I2C_GPIO_SCL = 77;
static unsigned int I2C_GPIO_SDA = 97;
static unsigned int I2C_GPIO_DELAY_US = 4;
#define OK     0
#define ERROR  1

static struct mutex  i2c_mlock;

/* creates /sys/module/i2c_gpio_hw/parameters/debug_mask file */
enum {
	I2C_DEBUG = 1U << 0,
};

static int i2c_debug_mask = 0;

module_param(i2c_debug_mask, int, S_IRUGO | S_IWUSR | S_IWGRP);

#define I2C_DBG_PRINTK(x...)  \
	do {					  \
		if (i2c_debug_mask)	  \
			printk(KERN_ERR x);	\
	} while (0)


static void i2c_gpio_stop_trans(void)
{
	//gpio_set(adp5587_sda_out, 0);
	//gpio_set_value(I2C_GPIO_SDA,0);
	
	//gpio_dir(adp5587_sda_out, 1);
	gpio_direction_output(I2C_GPIO_SDA,0);
	
	//gpio_dir(adp5587_scl, 1);
	gpio_direction_output(I2C_GPIO_SCL,0);
	
	//udelay(adp5587_wait_time);	
	udelay(I2C_GPIO_DELAY_US);

	//gpio_set(adp5587_scl, 1);
	gpio_set_value(I2C_GPIO_SCL,1);

	//udelay(adp5587_wait_time);
	udelay(I2C_GPIO_DELAY_US);

	//gpio_set(adp5587_sda_out, 1);	
	gpio_set_value(I2C_GPIO_SDA,1);
	
	//udelay(adp5587_wait_time);
	udelay(I2C_GPIO_DELAY_US);

}


static void i2c_gpio_write_1byte(unsigned char writebyte)
{
	int i;
	
	for(i = 7; i >= 0; i--)
	{	
		//gpio_set(adp5587_sda_out, ((writebyte >> i) & 0x01));
		gpio_set_value(I2C_GPIO_SDA,((writebyte >> i) & 0x01));
		
		//udelay(adp5587_wait_time);
		udelay(I2C_GPIO_DELAY_US);

		//gpio_set(adp5587_scl, 1);
		gpio_set_value(I2C_GPIO_SCL,1);
		
		//udelay(adp5587_wait_time);		
		udelay(I2C_GPIO_DELAY_US);

		//gpio_set(adp5587_scl, 0);
		gpio_set_value(I2C_GPIO_SCL,0);
		
		//udelay(adp5587_wait_time);
		udelay(I2C_GPIO_DELAY_US);

		if(i==0)
		{
			//gpio_dir(adp5587_sda_in, 0);			
			gpio_direction_input(I2C_GPIO_SDA);
		}

	}
}


static unsigned char i2c_gpio_read_1byte(void)
{
	int i;
	unsigned char readbit, readvalue;
	readvalue = 0;
	
	for(i = 7; i >= 0; i--)
	{
		//gpio_set(adp5587_scl, 1);
		gpio_set_value(I2C_GPIO_SCL,1);
		
		//udelay(adp5587_wait_time);
		udelay(I2C_GPIO_DELAY_US);
		
		readbit = gpio_get_value(I2C_GPIO_SDA);
		
		readvalue = (readvalue<<1) | readbit;

		//gpio_set(adp5587_scl, 0);		
		gpio_set_value(I2C_GPIO_SCL,0);
		
		//udelay(adp5587_wait_time);
		udelay(I2C_GPIO_DELAY_US);
	}
	
	return(readvalue);
}

static unsigned char i2c_gpio_write(unsigned char *pdata, unsigned char numbytes,unsigned char addr)
{
	unsigned char *data = pdata;
	unsigned char controlWord, j;
	unsigned char readvalue;
	unsigned char error = OK;

	gpio_direction_output(I2C_GPIO_SDA,1);
	gpio_direction_output(I2C_GPIO_SCL,1);
	udelay(I2C_GPIO_DELAY_US);

	gpio_set_value(I2C_GPIO_SDA,0);
	udelay(I2C_GPIO_DELAY_US);
	gpio_set_value(I2C_GPIO_SCL,0);
	udelay(I2C_GPIO_DELAY_US);

	/* address and write command */
	controlWord = addr;

	/* transfer the address and operation command */
	i2c_gpio_write_1byte(controlWord);

	udelay(I2C_GPIO_DELAY_US);

	gpio_set_value(I2C_GPIO_SCL,1);
	udelay(I2C_GPIO_DELAY_US);	
	/* CHECK ACK for control word */

	readvalue = gpio_get_value(I2C_GPIO_SDA);

	gpio_set_value(I2C_GPIO_SCL,0);
	udelay(I2C_GPIO_DELAY_US);

	/* restore SDIO as output */
	gpio_direction_output(I2C_GPIO_SDA,0);
	udelay(I2C_GPIO_DELAY_US);

	if(readvalue != 0)
	{
		i2c_gpio_stop_trans();
		error = ERROR;
		I2C_DBG_PRINTK("i2c write slave addr error  \n");
		return(error);
	}
	
	/* write data */
	for(j = 0; j < numbytes; j++, data++)
	{
		i2c_gpio_write_1byte(*data);
		
		udelay(I2C_GPIO_DELAY_US);

		gpio_set_value(I2C_GPIO_SCL,1);
		udelay(I2C_GPIO_DELAY_US);
		
		/* CHECK ACK for control word */
		readvalue = gpio_get_value(I2C_GPIO_SDA);

		gpio_set_value(I2C_GPIO_SCL,0);
		udelay(I2C_GPIO_DELAY_US);
		
		/* restore SDIO as output */
		gpio_direction_output(I2C_GPIO_SDA,0);
		udelay(I2C_GPIO_DELAY_US);

		if(readvalue != 0)
		{
			i2c_gpio_stop_trans();
			error = ERROR;	
			I2C_DBG_PRINTK("i2c write data error  \n");			
			return(error);
		}

	}

	i2c_gpio_stop_trans();
	return(error);
}

static unsigned char i2c_gpio_read(unsigned char reg, unsigned char numbytes, unsigned char *readData,unsigned char addr)
{
	unsigned char controlWord, j;
	unsigned char readvalue;
	unsigned char error = OK;

	/* START */
	gpio_direction_output(I2C_GPIO_SCL,1);
	gpio_direction_output(I2C_GPIO_SDA,1);
	udelay(I2C_GPIO_DELAY_US);
	
	gpio_set_value(I2C_GPIO_SDA,0);
	udelay(I2C_GPIO_DELAY_US);
	gpio_set_value(I2C_GPIO_SCL,0);
	udelay(I2C_GPIO_DELAY_US);
	
	controlWord = addr & (~I2C_M_RD);

	/* transfer the address and operation command */
	i2c_gpio_write_1byte(controlWord);

	gpio_set_value(I2C_GPIO_SCL,1);
	udelay(I2C_GPIO_DELAY_US);
	
	/* CHECK ACK for control word */
	readvalue = gpio_get_value(I2C_GPIO_SDA);

	gpio_set_value(I2C_GPIO_SCL,0);
	udelay(I2C_GPIO_DELAY_US);

	/* restore SDIO as output */
	gpio_direction_output(I2C_GPIO_SDA,0);
	udelay(I2C_GPIO_DELAY_US);

	if(readvalue != 0)
	{
		i2c_gpio_stop_trans();
		error = ERROR;
		I2C_DBG_PRINTK("i2c read write slave addr error  \n");
		return(error);
	}	

	i2c_gpio_write_1byte(reg);

	gpio_set_value(I2C_GPIO_SCL,1);
	udelay(I2C_GPIO_DELAY_US);
	
	/* CHECK ACK for control word */
	readvalue = gpio_get_value(I2C_GPIO_SDA);

	gpio_set_value(I2C_GPIO_SCL,0);
	udelay(I2C_GPIO_DELAY_US);
	
	/* restore SDIO as output */
	gpio_direction_output(I2C_GPIO_SDA,0);
    udelay(I2C_GPIO_DELAY_US);
	
	if(readvalue != 0)
	{
		i2c_gpio_stop_trans();
		error = ERROR;
		I2C_DBG_PRINTK("i2c read read data error \n");		
		return(error);
	}

	/* START */
	gpio_set_value(I2C_GPIO_SCL,1);	
	gpio_set_value(I2C_GPIO_SDA,1);
	udelay(I2C_GPIO_DELAY_US);	

	gpio_set_value(I2C_GPIO_SDA,0);	
	udelay(I2C_GPIO_DELAY_US);	
	gpio_set_value(I2C_GPIO_SCL,0);	
	udelay(I2C_GPIO_DELAY_US);

	/* address and read command */
	controlWord = addr;

	/* transfer the address and operation command */
	i2c_gpio_write_1byte(controlWord);

	gpio_set_value(I2C_GPIO_SCL,1);	
	udelay(I2C_GPIO_DELAY_US);
	
	/* CHECK ACK for control word */
	readvalue = gpio_get_value(I2C_GPIO_SDA);
	
	gpio_set_value(I2C_GPIO_SCL,0);	
	udelay(I2C_GPIO_DELAY_US);	
	
/*add the ACK and NACK signal after read byte */	
	/* restore SDIO as output */
	gpio_direction_output(I2C_GPIO_SDA,0);
	/* delete one line*/

	/* read data */
	for(j = 0; j < numbytes; j++, readData++)
	{
		/* delete one line*/
		gpio_direction_input(I2C_GPIO_SDA);
		udelay(I2C_GPIO_DELAY_US);

		*readData = i2c_gpio_read_1byte();

		gpio_direction_output(I2C_GPIO_SDA,1);
		/* delete one line*/

		/* SEND ACK */
		if (j == (numbytes - 1))
		{
			gpio_set_value(I2C_GPIO_SDA,1);
		}
		else
		{
			gpio_set_value(I2C_GPIO_SDA,0);
		}
		
		udelay(I2C_GPIO_DELAY_US);
		gpio_set_value(I2C_GPIO_SCL,1);
		udelay(I2C_GPIO_DELAY_US);
		gpio_set_value(I2C_GPIO_SCL,0);
		udelay(I2C_GPIO_DELAY_US);
		/* delete two lines*/
		gpio_set_value(I2C_GPIO_SDA,0);
		udelay(I2C_GPIO_DELAY_US);
	}

	i2c_gpio_stop_trans();
	return *readData;
}

int i2c_gpio_xfer(struct i2c_adapter *adap, struct i2c_msg msgs[], int num)
{
	DECLARE_COMPLETION_ONSTACK(complete);
	int ret;
	int rem = num;
	uint16_t addr;
	int i;
	
	mutex_lock(&i2c_mlock);
	
	addr = msgs[0].addr << 1;

	if (rem == 1 )//message number
  	{
		/*close log ,if need some logs ,write debug_mask file */
		if (msgs[0].flags & I2C_M_RD)
		{		
			if(i2c_debug_mask & I2C_DEBUG)
			{
				I2C_DBG_PRINTK("i2c_smbus_read_byte \n");
				
				for( i=0; i<msgs[0].len; i++)
					I2C_DBG_PRINTK("data%d :0x%02x \n",i,msgs[0].buf[i]);
			}
			
			i2c_gpio_read(msgs[0].buf[0],msgs[0].len,msgs[0].buf,(addr|0x01));
			
			if(i2c_debug_mask & I2C_DEBUG)
			{
				I2C_DBG_PRINTK("read dat from slave \n");
				for( i=0; i<msgs[1].len; i++)
					I2C_DBG_PRINTK("data%d :0x%02x ",i,msgs[0].buf[i]);
			}
		
		}else {
		
			if(i2c_debug_mask & I2C_DEBUG)
			{
				I2C_DBG_PRINTK("i2c_smbus_write_byte_data addr:0x%02x \n",addr);
				for( i=0; i<msgs[0].len; i++)
					I2C_DBG_PRINTK("data%d :0x%02x \n",i,msgs[0].buf[i]);
			}
			i2c_gpio_write(&(msgs[0].buf[0]),msgs[0].len,addr);
		
		}
  	}else if (rem == 2)
	{
		if (!(msgs[0].flags & I2C_M_RD) && (msgs[1].flags & I2C_M_RD))
		{
			/*close log ,if need some logs ,write debug_mask file */
			if(i2c_debug_mask & I2C_DEBUG)
			{
				I2C_DBG_PRINTK("i2c read two msgs, i2c_transfer \n");
				
				for( i=0; i<msgs[0].len; i++)
					I2C_DBG_PRINTK("data%d :0x%02x \n",i,msgs[0].buf[i]);
			}
			i2c_gpio_read(msgs[0].buf[0],msgs[1].len,msgs[1].buf,(addr|0x01));

			if(i2c_debug_mask & I2C_DEBUG)
			{
				I2C_DBG_PRINTK("read dat from slave \n");
				for( i=0; i<msgs[1].len; i++)
					I2C_DBG_PRINTK("data%d :0x%02x ",i,msgs[1].buf[i]);
			}
		}else {
			//i2c_gpio_write(msgs->buf,msgs->len,addr);
		
		}

	}else{
		printk(KERN_ERR "Don't support this mode \n");
	}
	ret = num;

	mutex_unlock(&i2c_mlock);
	return ret;
}

u32 i2c_gpio_func(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm i2c_gpio_algo = {
	.master_xfer	= i2c_gpio_xfer,
	.functionality	= i2c_gpio_func,
};

static int __devinit i2c_gpio_probe(struct platform_device *pdev)
{
	int			 ret = 0;
	struct i2c_adapter *adapter;
	int rc;

	adapter = kzalloc(sizeof(struct i2c_adapter), GFP_KERNEL);
	if (adapter == NULL) {
		printk(KERN_ERR "i2c_gpio_probe can't allocate inteface!\n");
		rc = -ENOMEM;
		goto err_probe_exit;
	}
	snprintf(adapter->name, sizeof(adapter->name), "i2c gpio adapter");
	adapter->algo = &i2c_gpio_algo;
	adapter->nr = pdev->id;
	adapter->dev.parent = &pdev->dev;

	platform_set_drvdata(pdev, adapter);

	//i2c_set_adapdata(adapter, dev);

	rc = i2c_add_numbered_adapter(adapter);
	if (rc) {
		printk(KERN_ERR "Adapter %s registration failed\n",adapter->name);
		goto err_add_adapter_failed;
	}

	mutex_init(&i2c_mlock);
	
	rc = gpio_request(I2C_GPIO_SCL, "i2c-gpio-scl");
	rc = gpio_request(I2C_GPIO_SDA, "i2c-gpio-sda");
	printk(KERN_ERR "i2c-gpio-hw init ok \n");
	return 0;
err_add_adapter_failed:
	platform_set_drvdata(pdev, NULL);
	kfree(adapter);
err_probe_exit:
	return ret;
}

static int __devexit i2c_gpio_remove(struct platform_device *pdev)
{
	struct i2c_gpio_platform_data *pdata;
	struct i2c_adapter *adap;

	adap = platform_get_drvdata(pdev);
	pdata = pdev->dev.platform_data;

	i2c_del_adapter(adap);
	gpio_free(pdata->scl_pin);
	gpio_free(pdata->sda_pin);
	kfree(adap->algo_data);
	kfree(adap);

	return 0;
}

static struct platform_driver i2c_gpio_driver = {
	.driver		= {
		.name	= "i2c-gpio-hw",
		.owner	= THIS_MODULE,
	},
	.probe		= i2c_gpio_probe,
	.remove		= __devexit_p(i2c_gpio_remove),
};

static int __init i2c_gpio_init(void)
{
	int ret;
/* C8800 can't use the i2c bus,because GPIO97 used as MDP vsync */
	if(machine_is_msm7x27_c8800())
	{
		ret = -1;
	}
	else
	{
		ret = platform_driver_register(&i2c_gpio_driver);
	}
	if (ret)
		printk(KERN_ERR "i2c-gpio: probe failed: %d\n", ret);

	return ret;
}
static void __exit i2c_gpio_exit(void)
{
	platform_driver_unregister(&i2c_gpio_driver);
}

subsys_initcall(i2c_gpio_init);
module_exit(i2c_gpio_exit);

MODULE_DESCRIPTION("I2C GPIO Bus Driver");
MODULE_LICENSE("GPL");
