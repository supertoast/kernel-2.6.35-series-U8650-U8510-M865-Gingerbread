
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/gpio.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/freezer.h>
#include <linux/akm8973.h>
#include <linux/earlysuspend.h>
#include "linux/st303.h"
#include <mach/gpio.h>
#include <mach/vreg.h>
#include "linux/hardware_self_adapt.h"
#ifdef CONFIG_ANDROID_POWER
#include <linux/android_power.h>
#endif

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
#include <linux/hw_dev_dec.h>
#endif

#include "../../../arch/arm/mach-msm/proc_comm.h"
#define COMPASS_INIT_FAIL		(99)
#define COMPASS_INIT_SUCCESS	(0)
//#define DEBUG_WQ 
#define DEBUG 0
#define     GS_POLLING  1
//#define GS303_DEBUG_ST303_COMPASS  
//#undef GS303_DEBUG_ST303_COMPASS

#ifdef GS303_DEBUG_ST303_COMPASS
#define GS303_DEBUG(fmt, args...) printk(KERN_ERR fmt, ##args)
#else
#define GS303_DEBUG(fmt, args...)
#endif

/* Magnetometer registers */
#define CRA_REG_M   0x00  /* Configuration register A */
#define CRB_REG_M   0x01  /* Configuration register B */
#define MR_REG_M    0x02  /* Mode register */

/* Output register start address*/
#define OUT_X_M           0x03

/* read HIDED_EARTH_MAGIC_REG for match WHO AM I
   303DLH DEV ID = 0x00 
   303DLM DEV ID = 0x3c  */
#define HIDED_EARTH_MAGIC_REG  0x0F

int st303_dev_id = DEV_ID_NONE;


#ifdef DEBUG_WQ
static struct workqueue_struct *gs_wq;
#endif

static struct i2c_client *this_client;
extern struct input_dev *sensor_dev;
#ifdef DEBUG_WQ
static int accel_delay = 1000; 
#endif

struct st303_data {
	struct input_dev *input_dev;
	struct work_struct work;
	struct early_suspend early_suspend;
	struct i2c_client *client;
	struct hrtimer timer;	
	#ifdef DEBUG_WQ
	int use_irq;
	#endif
		
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void st303_early_suspend(struct early_suspend *h);
static void st303_early_resume(struct early_suspend *h);
#endif

static DECLARE_WAIT_QUEUE_HEAD(open_wq);

static atomic_t open_count;
static atomic_t open_flag;
static atomic_t reserve_open_flag;

static atomic_t m_flag;
static atomic_t mv_flag;

static short st303d_delay = 0;

static int compass_debug_mask = 0;
		
enum {
	COMPASS_DEBUG_DATA = 1U << 0,
	COMPASS_DEBUG_REG = 1U << 1,
	COMPASS_DEBUG_REPORT = 1U << 2,
	COMPASS_DEBUG_OTHER = 1U << 3,
};

#ifdef CONFIG_ANDROID_POWER
static atomic_t suspend_flag = ATOMIC_INIT(0);
#endif

#ifdef DEBUG_WQ
static inline int reg_read(struct st303_data *gs , int reg)
{
	int val;


	val = i2c_smbus_read_byte_data(gs->client, reg);
	if (val < 0)
		printk(KERN_ERR "st303_gs.c failed\n");

	return val;
}
#endif

/* following are the sysfs callback functions */

static int AKI2C_RxData(char *rxData, int length)
{
	struct i2c_msg msgs[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = 1,
		 .buf = rxData,
		 },
		{
		 .addr = this_client->addr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msgs, 2) < 0) {
		printk(KERN_ERR "AKI2C_RxData: transfer error\n");
		return -EIO;
	} else
		return 0;
}

static int AKI2C_TxData(char *txData, int length)
{
#if 0
	struct i2c_msg msg[] = {
		{
		 .addr = this_client->addr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};

	if (i2c_transfer(this_client->adapter, msg, 1) < 0) {
		printk(KERN_ERR "AKI2C_TxData: transfer error\n");
		return -EIO;
	} else
#endif
		return 0;
}

static void AKECS_Report_Value(short *rbuf)
{
	struct st303_data *data = i2c_get_clientdata(this_client);
	
	if(compass_debug_mask & COMPASS_DEBUG_REPORT)
	{
		printk("%d %d %d ", rbuf[0], rbuf[1], rbuf[2]);
		printk("%d %d %d \n", rbuf[3], rbuf[4], rbuf[5]);
	}

	/* Report orientation sensor information */
	if (atomic_read(&m_flag)) {
		input_report_abs(data->input_dev, ABS_RX, rbuf[0]);
		input_report_abs(data->input_dev, ABS_RY, rbuf[1]);
		input_report_abs(data->input_dev, ABS_RZ, rbuf[2]);
		input_report_abs(data->input_dev, ABS_RUDDER, 3);
	}
	
	/* Report magnetic sensor information */
	if (atomic_read(&mv_flag)) {
		input_report_abs(data->input_dev, ABS_HAT0X, rbuf[3]);
		input_report_abs(data->input_dev, ABS_HAT0Y, rbuf[4]);
		input_report_abs(data->input_dev, ABS_BRAKE, rbuf[5]);
	}
	
	input_sync(data->input_dev);
}

static int AKECS_GetOpenStatus(void)
{

#if DEBUG
	printk(KERN_ERR "%s    \n", __FUNCTION__);
#endif
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) != 0));

	return atomic_read(&open_flag);
}

static int AKECS_GetCloseStatus(void)
{

#if DEBUG
	printk(KERN_ERR "%s wait forever \n", __FUNCTION__);
#endif
	wait_event_interruptible(open_wq, (atomic_read(&open_flag) <= 0));

	return atomic_read(&open_flag);
}

static void AKECS_CloseDone(void)
{
#if DEBUG
	printk(KERN_ERR "%s \n", __FUNCTION__);
#endif
	atomic_set(&m_flag, 1);
	atomic_set(&mv_flag, 1);
}

static int st303_aot_open(struct inode *inode, struct file *file)
{
	int ret = -1;
#if DEBUG
	printk(KERN_ERR "%s  \n", __FUNCTION__);
#endif
	if (atomic_cmpxchg(&open_count, 0, 1) == 0) {
		if (atomic_cmpxchg(&open_flag, 0, 1) == 0) {
			atomic_set(&reserve_open_flag, 1);
			wake_up(&open_wq);
			ret = 0;
		}
	}
	return ret;
}

static int st303_aot_release(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_ERR "%s  \n", __FUNCTION__);
#endif
	atomic_set(&reserve_open_flag, 0);
	atomic_set(&open_flag, 0);
	atomic_set(&open_count, 0);
	wake_up(&open_wq);
	return 0;
}

static int
st303_aot_ioctl(struct inode *inode, struct file *file,
	      unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	short flag;
#if DEBUG
	printk(KERN_ERR "%s  \n", __FUNCTION__);
#endif

	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG:     /*set open magnetic sensor flag*/
	case ECS_IOCTL_APP_SET_AFLAG:     /*set open acceleration sensor flag*/
	case ECS_IOCTL_APP_SET_TFLAG:     /*set open temprature sensor flag*/
	case ECS_IOCTL_APP_SET_MVFLAG:   /*set open move  sensor flag*/
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		if (flag < 0 || flag > 1)
			return -EINVAL;
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		if (copy_from_user(&flag, argp, sizeof(flag)))
			return -EFAULT;
		break;
	default:
		break;
	}

	/*get device ID*/
	switch (cmd) {
	case ECS_IOCTL_APP_SET_MFLAG: 
		atomic_set(&m_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MFLAG: /*get open magnetic sensor flag*/
		flag = atomic_read(&m_flag);
		break;
	case ECS_IOCTL_APP_SET_MVFLAG:
		atomic_set(&mv_flag, flag);
		break;
	case ECS_IOCTL_APP_GET_MVFLAG: /*get open move sensor flag*/
		flag = atomic_read(&mv_flag);
		break;
	case ECS_IOCTL_APP_SET_DELAY:
		st303d_delay = flag;
		break;
	case ECS_IOCTL_APP_GET_DELAY:
		flag = st303d_delay;
		break;
	case ECS_IOCTL_APP_GET_DEVID:
		break;
	default:
		return -ENOTTY;
	}

	switch (cmd) {
	case ECS_IOCTL_APP_GET_MFLAG:
	case ECS_IOCTL_APP_GET_AFLAG:
	case ECS_IOCTL_APP_GET_TFLAG:
	case ECS_IOCTL_APP_GET_MVFLAG:
	case ECS_IOCTL_APP_GET_DELAY:
		if (copy_to_user(argp, &flag, sizeof(flag)))
			return -EFAULT;
		break;
	case ECS_IOCTL_APP_GET_DEVID:
		if((st303_dev_id == DEV_ID_303DLH) )
		{
			if (copy_to_user(argp, ST303DLH_I2C_NAME, strlen(ST303DLH_I2C_NAME)+1))
				return -EFAULT;
		}
		else if(st303_dev_id == DEV_ID_303DLM)
		{
			if (copy_to_user(argp, ST303DLM_I2C_NAME, strlen(ST303DLM_I2C_NAME)+1))
				return -EFAULT;
		}
		break;
	default:
		break;
	}

	return 0;
}

static int st303d_open(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_ERR "%s  \n", __FUNCTION__);
#endif
	return nonseekable_open(inode, file);
}

static int st303d_release(struct inode *inode, struct file *file)
{
#if DEBUG
	printk(KERN_ERR "%s\n", __FUNCTION__);
#endif
	AKECS_CloseDone();
	return 0;
}

static int
st303d_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	   unsigned long arg)
{
	void __user *argp = (void __user *)arg;

	char rwbuf[16];
	int ret = -1, status;
	short value[12], delay;	
	signed short tempx = 0,tempy = 0,tempz = 0;

	switch (cmd) {
		case ECS_IOCTL_READ:
		case ECS_IOCTL_WRITE:
			if (copy_from_user(&rwbuf, argp, sizeof(rwbuf)))
				return -EFAULT;
			break;
			
		case ECS_IOCTL_SET_YPR:
			if (copy_from_user(&value, argp, sizeof(value)))
				return -EFAULT;
			break;
		
		default:
			//printk(KERN_ERR "%s default error \n", __FUNCTION__);
			break;
	}

	switch (cmd) {

		case ECS_IOCTL_READ:
#if DEBUG
			printk(" len %02x:", rwbuf[0]);
			printk(" addr %02x:", rwbuf[1]);
#endif
			if (rwbuf[0] < 1)
				return -EINVAL;
			ret = AKI2C_RxData(&rwbuf[1], rwbuf[0]);

			if(DEV_ID_303DLM == st303_dev_id)
			{
				int temp = 0;
				/*Y raw data reg is 0x07 and 0x08*/
				/*Z raw data reg is 0x05 and 0x06*/
				temp = rwbuf[3];
				rwbuf[3] = rwbuf[5];
				rwbuf[5] = temp;
				
				temp = rwbuf[4];
				rwbuf[4] = rwbuf[6];
				rwbuf[6] = temp;
			}
			
			if(compass_debug_mask & COMPASS_DEBUG_DATA)
			{
				tempx = (signed short)(((rwbuf[1])<<8)|rwbuf[2]);
				tempy = (signed short)(((rwbuf[3])<<8)|rwbuf[4]);
				tempz = (signed short)(((rwbuf[5])<<8)|rwbuf[6]);
				
				printk("m:%d %d %d \n",tempx,tempy,tempz);
			}

			if(compass_debug_mask & COMPASS_DEBUG_REG)
			{
				int ij = 0;
				
				for(ij=0;ij<0x9;ij++)
				{
					printk("0x%2x reg: 0x%x \n",ij,i2c_smbus_read_byte_data(this_client,ij));
				}
			}

			if (ret < 0)
				return ret;
			break;
		case ECS_IOCTL_WRITE:
#if DEBUG
			printk("ECS_IOCTL_WRITE %x\n", cmd);
			printk(" len %02x:", rwbuf[0]);
			for(i=0; i<rwbuf[0]; i++){
				printk(" %02x", rwbuf[i+1]);
			}
#endif
			if (rwbuf[0] < 2)
				return -EINVAL;
			ret = AKI2C_TxData(&rwbuf[1], rwbuf[0]);
#if DEBUG
			printk(" ret = %d\n", ret);
#endif
			if (ret < 0)
				return ret;
			break;
		case ECS_IOCTL_SET_YPR:
#if DEBUG
			//printk("ECS_IOCTL_SET_YPR %x ypr=%x\n", cmd, value);
#endif
			AKECS_Report_Value(value);
			break;
		case ECS_IOCTL_GET_OPEN_STATUS:
#if DEBUG
			printk("ECS_IOCTL_GET_OPEN_STATUS %x start\n", cmd);
#endif
			status = AKECS_GetOpenStatus();
#if DEBUG
			printk("ECS_IOCTL_GET_OPEN_STATUS %x end status=%x\n", cmd, status);
#endif
			break;
		case ECS_IOCTL_GET_CLOSE_STATUS:
#if DEBUG
			printk("ECS_IOCTL_GET_CLOSE_STATUS %x start\n", cmd);
#endif
			status = AKECS_GetCloseStatus();
#if DEBUG
			printk("ECS_IOCTL_GET_CLOSE_STATUS %x end status=%x\n", cmd, status);
#endif
			break;

		case ECS_IOCTL_GET_DELAY:
			delay = st303d_delay;
#if DEBUG
			printk("ECS_IOCTL_GET_DELAY %x delay=%x\n", cmd, delay);
#endif
			break;
		default:
			return -ENOTTY;
	}

	switch (cmd) {
		case ECS_IOCTL_READ:
			if (copy_to_user(argp, &rwbuf, sizeof(rwbuf)))
				return -EFAULT;
			break;
			
		case ECS_IOCTL_GET_OPEN_STATUS:
			if (copy_to_user(argp, &status, sizeof(status)))
				return -EFAULT;
			break;
			
		case ECS_IOCTL_GET_CLOSE_STATUS:
			if (copy_to_user(argp, &status, sizeof(status)))
				return -EFAULT;
			break;
			
		case ECS_IOCTL_GET_DELAY:
			if (copy_to_user(argp, &delay, sizeof(delay)))
				return -EFAULT;
			break;

		case ECS_IOCTL_SET_YPR:
			break;
						
		default:
			break;
	}

	return 0;
}
static int st303_init_client(struct i2c_client *client)
{
	init_waitqueue_head(&open_wq);

	/* As default, report all information */
	/*modify the default values*/
	atomic_set(&m_flag, 0);
	atomic_set(&mv_flag, 0);

	return 0;
}

static struct file_operations st303d_fops = {
	.owner = THIS_MODULE,
	.open = st303d_open,
	.release = st303d_release,
	.ioctl = st303d_ioctl,
};

static struct file_operations st303_aot_fops = {
	.owner = THIS_MODULE,
	.open = st303_aot_open,
	.release = st303_aot_release,
	.ioctl = st303_aot_ioctl,
};

static struct miscdevice st303_aot_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "compass_aot",
	.fops = &st303_aot_fops,
};

static struct miscdevice st303d_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "compass_dev",
	.fops = &st303d_fops,
};

#ifdef DEBUG_WQ
static void gs_work_func(struct work_struct *work)
{
	
	//int status;	
	signed short x = 0;
	signed short y = 0;
	signed short z = 0;
	u8 udata[2]={0};

	
	struct st303_data *gs = container_of(work, struct st303_data, work);
	int	sesc = accel_delay/1000;
	int nsesc = (accel_delay%1000)*1000000;
	

{
		udata[0]= 0;
		udata[1]= 0;
		udata[0] = reg_read(gs, 0x3 ); 
		GS303_DEBUG(KERN_ERR "compress   A_x h 0x%x \n", udata[0]);
		udata[1] = reg_read(gs, 0x4 );  
		GS303_DEBUG(KERN_ERR "compress  A_x l 0x%x \n", udata[1]);
		x = ((udata[0])<<8)|udata[1];

		udata[0]= 0;
		udata[1]= 0;
		udata[0] = reg_read(gs, 0x5 );  
		GS303_DEBUG(KERN_ERR "compress   A_y h 0x%x \n", udata[0]);
		udata[1] = reg_read(gs, 0x6 );  
		GS303_DEBUG(KERN_ERR "compress   A_y l 0x%x \n", udata[1]);
		y = ((udata[0])<<8)|udata[1];

		udata[0]= 0;
		udata[1]= 0;
		udata[0] = reg_read(gs, 0x7 );  
		GS303_DEBUG(KERN_ERR "compress   A_z h 0x%x \n", udata[0]);
		udata[1] = reg_read(gs, 0x8 );  
		GS303_DEBUG(KERN_ERR "compress  A_z l 0x%x \n", udata[1]);
		z= ((udata[0])<<8)|udata[1];
		GS303_DEBUG(KERN_INFO "%s :%d   st303_compress probe  A  x :0x%x y :0x%x z :0x%x \n",__func__,__LINE__,x,y,z);

}

	
	if (gs->use_irq)
		enable_irq(gs->client->irq);
	else
		hrtimer_start(&gs->timer, ktime_set(sesc, nsesc), HRTIMER_MODE_REL);
	GS303_DEBUG(KERN_INFO "%s :%d   st303_compass probe \n",__func__,__LINE__);
}


static enum hrtimer_restart gs_timer_func(struct hrtimer *timer)
{
	struct st303_data *gs = container_of(timer, struct st303_data, timer);		
	queue_work(gs_wq, &gs->work);
	/* delete one line */
	return HRTIMER_NORESTART;
}

#ifndef   GS_POLLING 	

static irqreturn_t gs_irq_handler(int irq, void *dev_id)
{
	struct st303_data *gs = dev_id;
	disable_irq(gs->client->irq);
	queue_work(gs_wq, &gs->work);
	return IRQ_HANDLED;
}

static int gs_config_int_pin(void)
{
	int err;

	err = gpio_request(GPIO_INT2, "gpio_compass_int");
	if (err) 
	{
		printk(KERN_ERR "gpio_request failed for st gs int\n");
		return -1;
	}	

	err = gpio_configure(GPIO_INT2, GPIOF_INPUT | IRQF_TRIGGER_HIGH);
	if (err) 
	{
		gpio_free(GPIO_INT2);
		printk(KERN_ERR "gpio_config failed for gs int HIGH\n");
		return -1;
	}     

	return 0;
}

static void gs_free_int(void)
{

	gpio_free(GPIO_INT2);

}
#endif
#endif
static ssize_t show_debug_mask(struct device *dev, struct device_attribute *attr,char *buf)
{
  size_t i;
  i = scnprintf(buf, PAGE_SIZE, "%d\n",compass_debug_mask);
  return i;
}

static ssize_t store_debug_mask(struct device *dev, struct device_attribute *attr,char *buf)
{
  size_t i;
  sscanf(buf, "%d", &compass_debug_mask);  
  return i;
}
static DEVICE_ATTR(compass_debug_mask, S_IRUGO | S_IWUSR, show_debug_mask, store_debug_mask);
int st303_probe(struct i2c_client *client, const struct i2c_device_id * devid)
{
	struct st303_data *st303;
	int err=0;
	int rc;
	#ifdef DEBUG_WQ
	#ifndef   GS_POLLING
	int ret;
	#endif
	#endif
	int i,j;
	unsigned long nv_compass_state = 0;
	unsigned long  nv_value = 1;
    unsigned nv_item = 60019;
    int  rval = -1;

	
	rval = msm_proc_comm(PCOM_NV_READ, &nv_item, (unsigned*)&nv_value); 
	if(0 == rval)
	{
		printk(KERN_ERR"st303_compass: read OK! nv(%d)=%d, rval=%d\n", nv_item, (int)nv_value, rval);
	}
	else
	{
		printk(KERN_ERR"st303_compass: read failed! nv(%d)=%d, rval=%d\n", nv_item, (int)nv_value, rval);
	}
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}
	
	client->addr = 0x1E;//8bit address is 0x3c

#ifdef DEBUG_WQ
	#ifndef   GS_POLLING 	
	ret = gs_config_int_pin();
	if(ret <0)
	{
		goto exit_check_functionality_failed;
	}
	#endif
#endif
	
	st303 = kzalloc(sizeof(struct st303_data), GFP_KERNEL);
	if (!st303) {
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
#ifdef DEBUG_WQ
	INIT_WORK(&st303->work, gs_work_func);
#endif
	st303->client = client;
   	i2c_set_clientdata(client, st303);
	st303_init_client(client);
	this_client = client;

	st303->input_dev = sensor_dev;
	if ((st303->input_dev == NULL)||(st303->input_dev->id.vendor != GS_ST303DLH)) {
		err = -ENOMEM;
		printk(KERN_ERR "st303_compass probe: Failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	for(i=0; i<3; i++)
	{
		rc = i2c_smbus_read_byte_data(client,HIDED_EARTH_MAGIC_REG);  /*read HIDED Magic register in order to check who am I*/
		if(rc < 0){
			if(i == 2){
				err = -ENODEV;
				printk(KERN_ERR"st303_compass: read HIDED_EARTH_MAGIC_REG failure\n");
				goto exit_input_dev_alloc_failed;
			}
		}else{
			if(rc == 0x00){
				st303_dev_id = DEV_ID_303DLH;
				printk(KERN_INFO "st303_compass: find device 303DLH = 0x%x\n", rc);
			}
			else if(rc == 0x3c){
				st303_dev_id = DEV_ID_303DLM;
				printk(KERN_INFO "st303_compass: find device 303DLM12 = 0x%x\n", rc);
			}
			else{
				st303_dev_id = DEV_ID_303DLH;
				printk(KERN_ERR"st303_compass: Not match device ID = 0x%x, set default compass 303DLH\n", rc);
			}
			break;
		}
	}

    #ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_COMPASS);
    #endif
    
	for(i=0;i<3;i++)
	{
		j = 0;
		rc = i2c_smbus_write_byte_data(client,CRA_REG_M,0x18);//0x18 75HZ  0X14 30HZ
		if (rc < 0){
			j++;
			printk(KERN_ERR"st303_compass write CRA_REG_M failed \n");
		}
		
		rc = i2c_smbus_write_byte_data(client,CRB_REG_M,0xe0);//(0x20 1.3gauss) (0x40 1.9gauss) (0x80 4.0gauss) (0xe0 8.1gauss)
		if (rc < 0){
			j++;
			printk(KERN_ERR"st303_compass write CRB_REG_M failed \n");
		}
		
		rc = i2c_smbus_write_byte_data(client,MR_REG_M, 0x00);//normal mode
		if (rc < 0){
			j++;
			printk(KERN_ERR"st303_compass write MR_REG_M failed \n");
		}

		if (j == 0)
		{
			break;
		}
	}
	if (j != 0)
	{
		err = -ENODEV;
		goto exit_input_dev_alloc_failed;
		
	}else{
	
		if( nv_value != COMPASS_INIT_SUCCESS)
		{
			nv_compass_state = COMPASS_INIT_SUCCESS;
			rval = msm_proc_comm(PCOM_NV_WRITE, &nv_item, (unsigned*)&nv_compass_state); 
	        if(0 == rval)
	        {
	            printk(KERN_ERR"st303_compass: probe OK,write OK! nv(%d)=%d, rval=%d\n", nv_item, (int)nv_compass_state, rval);
	        }
	        else
	        {
	            printk(KERN_ERR"st303_compass: probe OK,write failed! nv(%d)=%d, rval=%d\n", nv_item, (int)nv_compass_state, rval);
	        }
		}
	}	
	
	set_bit(EV_ABS, st303->input_dev->evbit);
	set_bit(ABS_X, st303->input_dev->absbit);
	set_bit(ABS_Y, st303->input_dev->absbit);
	set_bit(ABS_Z, st303->input_dev->absbit);
	/* azimuth */
	input_set_abs_params(st303->input_dev, ABS_RX, 0, 360, 0, 0);
	/* pitch */
	input_set_abs_params(st303->input_dev, ABS_RY, -180, 180, 0, 0);
	/* roll */
	input_set_abs_params(st303->input_dev, ABS_RZ, -90, 90, 0, 0);
	/* status of magnetic sensor */
	input_set_abs_params(st303->input_dev, ABS_RUDDER, 0, 3, 0, 0);
	
	/* x-axis of raw magnetic vector */
	input_set_abs_params(st303->input_dev, ABS_HAT0X, -2048, 2047, 0, 0);
	/* y-axis of raw magnetic vector */
	input_set_abs_params(st303->input_dev, ABS_HAT0Y, -2048, 2047, 0, 0);
	/* z-axis of raw magnetic vector */
	input_set_abs_params(st303->input_dev, ABS_BRAKE, -2048, 2047, 0, 0);
	err = misc_register(&st303d_device);
	if (err) {
		printk(KERN_ERR "st303_probe: st303d_device register failed\n");
		goto exit_misc_device_register_failed1;
	}
	err = misc_register(&st303_aot_device);
	if (err) {
		printk(KERN_ERR"st303_probe: st303_aot_device register failed\n");
		goto exit_misc_device_register_failed2;
	}
#ifdef DEBUG_WQ
#ifndef   GS_POLLING 
	if (client->irq) {
		ret = request_irq(client->irq, gs_irq_handler, 0, client->name, st303);
		if (ret == 0)
			st303->use_irq = 1;
		else
			dev_err(&client->dev, "request_irq failed\n");
	}
#endif 

	if (!st303->use_irq) {
		hrtimer_init(&st303->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		st303->timer.function = gs_timer_func;
	}
#endif

	/* create /sys/devices/i2c-0/0-001f/compass_debug_mask file */
	err = device_create_file(&client->dev, &dev_attr_compass_debug_mask);
	if (err)
	{
		printk(KERN_ERR "st303_probe: create sys file compass_status fail\n");		
		goto exit_misc_device_register_failed2;
	}	

#ifdef CONFIG_HAS_EARLYSUSPEND
	st303->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	st303->early_suspend.suspend = st303_early_suspend;
	st303->early_suspend.resume = st303_early_resume;
	register_early_suspend(&st303->early_suspend);
#endif

#ifdef DEBUG_WQ
	gs_wq = create_singlethread_workqueue("gs_wq");
	if (!gs_wq)
		return -ENOMEM;
#endif

	GS303_DEBUG(KERN_ERR "%s client->addr 0x%x  \n", __FUNCTION__, client->addr);
#ifdef DEBUG_WQ
hrtimer_start(&st303->timer, ktime_set(0, 500000000), HRTIMER_MODE_REL);
#endif
	return 0;

exit_misc_device_register_failed2:
	misc_deregister(&st303d_device);
exit_misc_device_register_failed1:
//exit_input_register_device_failed:
	input_free_device(st303->input_dev);
exit_input_dev_alloc_failed:
	kfree(st303);
exit_alloc_data_failed:
#ifdef DEBUG_WQ
#ifndef   GS_POLLING 
	gs_free_int();
#endif
#endif
exit_check_functionality_failed:
	
	nv_compass_state = COMPASS_INIT_FAIL;
	if(nv_compass_state != nv_value)
	{
		rval = msm_proc_comm(PCOM_NV_WRITE, &nv_item, (unsigned*)&nv_compass_state); 
		if(0 == rval)
		{
			printk(KERN_ERR"st303_compass:probe failded write OK! nv(%d)=%d, rval=%d\n", nv_item, (int)nv_compass_state, rval);
		}
		else
		{
			printk(KERN_ERR"st303_compass: probe failded write failed! nv(%d)=%d, rval=%d\n", nv_item, (int)nv_compass_state, rval);
		}
	}	
	return err;
}

static int st303_detect(struct i2c_client *client, int kind,
			  struct i2c_board_info *info)
{
	strlcpy(info->type, "st303", I2C_NAME_SIZE);
	return 0;
}

static int st303_remove(struct i2c_client *client)
{
	struct st303_data *st303 = i2c_get_clientdata(client);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&st303->early_suspend);
#endif

	input_unregister_device(st303->input_dev);
	//i2c_detach_client(client);
	kfree(st303);
	return 0;
}


static int st303_suspend(struct i2c_client *client, pm_message_t mesg)
{ 
	int ret;

	ret = i2c_smbus_write_byte_data(client,MR_REG_M,0x03);//0x02 reg 03 sleep
	if (ret < 0)
			printk(KERN_ERR "st303_suspend power off failed\n");
	atomic_set(&reserve_open_flag, atomic_read(&open_flag));
	atomic_set(&open_flag, 0);
	wake_up(&open_wq);
	
	return 0;
}

static int st303_resume(struct i2c_client *client)
{

	int ret;

	ret = i2c_smbus_write_byte_data(client,MR_REG_M,0x0);//0x2reg 00 normal
	if (ret < 0)
			printk(KERN_ERR "st303_resume power measure failed\n");
	atomic_set(&open_flag, atomic_read(&reserve_open_flag));
	wake_up(&open_wq);

	return 0;
}


#ifdef CONFIG_HAS_EARLYSUSPEND
static void st303_early_suspend(struct early_suspend *h)
{
      struct st303_data *st303 ;
	st303 = container_of(h, struct st303_data, early_suspend);
	
	st303_suspend(this_client, PMSG_SUSPEND);
}

static void st303_early_resume(struct early_suspend *h)
{	
	struct st303_data *st303 ;
	st303 = container_of(h, struct st303_data, early_suspend);
	
	st303_resume(this_client);
}
#endif

static const struct i2c_device_id st303_id[] = {
	{ "st303_compass", 0 },
	{ }
};

static struct i2c_driver st303_driver = {
	.class = I2C_CLASS_HWMON,
	.probe = st303_probe,
	.remove = st303_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = st303_suspend,
	.resume = st303_resume,
#endif
	.id_table = st303_id,
	.driver = {
		   .owner = THIS_MODULE,
		   .name = "st303_compass",
		   },
	.detect = st303_detect,
	
};

static int __init st303_init(void)
{
	GS303_DEBUG(KERN_INFO "%s : st303_gs probe \n",__func__);
	return i2c_add_driver(&st303_driver);
}

static void __exit st303_exit(void)
{
	i2c_del_driver(&st303_driver);
}

late_initcall(st303_init);
module_exit(st303_exit);
MODULE_DESCRIPTION("st303 compass driver");
MODULE_LICENSE("GPL");

