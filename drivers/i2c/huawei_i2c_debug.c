/*
* huawei_i2c_debug.c - i2c-bus driver, char device interface.
* used to debug i2c devices.
*
* Copyright (c) 2010 huawei 
*
* This file may be distributed under the terms of the GNU GPL license.
*/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>
#include <linux/smp_lock.h>
#include <linux/jiffies.h>
#include <asm/uaccess.h>
#include <linux/cdev.h>
#include <linux/debugfs.h>
#include <linux/file.h>
#include <linux/unistd.h>
#include <linux/syscalls.h>
#include <linux/delay.h>

#define I2C_DEBUG 
#define I2C_MINORS	256
#define I2C_NAME_SIZE	20

#define I2C_CMD_REG_STR_LEN 5
#define I2C_DEBUG_REG_STR_LEN 3
#define I2C_DEBUG_REGV_STR_LEN 5
#define I2C_DEBUG_CMD_LEN 50
#define I2C_DEBUG_LOG_FILE "/data/i2c_debug_result.txt"
#define I2C_DEBUG_BEGIN_FILE "/data/i2c_debug_begin.txt"
#define I2C_DEBUG_END_FILE "/data/i2c_debug_end.txt"

static LIST_HEAD(i2c_debug_dev_list);
static DEFINE_SPINLOCK(i2c_dev_list_lock);
static LIST_HEAD(regs_data_list);
static DEFINE_SPINLOCK(regs_data_list_lock);

static struct class *i2c_debug_dev_class;
static dev_t g_dev_no;
static struct workqueue_struct *i2c_debug_wq;
struct work_struct  g_work;
struct i2c_debug_dev *g_i2c_dev;

#ifdef I2C_DEBUG
#define I2C_DEBUG_MACRO(fmt, args...) printk(KERN_DEBUG "i2c-debug:" fmt, ##args)
#else
#define I2C_DEBUG_MACRO(fmt, args...)
#endif

#define I2C_DEBUG_CLEAR_BUF_MANUAL 0x0001
#define I2C_DEBUG_SHOW_XFER_BUF 0x0010
#define I2C_DEBUG_HOOK_QUEUE_WORK_OFF 0x0100
#define I2C_DEBUG_CLEAR_BUF 0x1000

static int i2c_debug_mask;
module_param_named(i2c_debug_param, i2c_debug_mask, uint, S_IRUGO | S_IWUSR)

#define I2C_DEBUG_DRIVER_NAME "i2c-debug"

struct i2c_debug_dev {
	struct list_head list;
	struct i2c_adapter *adap;
	struct i2c_client *client;
	struct cdev cdev;
	dev_t dev_no;
	struct device* dev;
	struct work_struct  work;
	u8 hook_flag;
};

typedef enum{
	I2C_DEBUG_BEGIN,
	I2C_DEBUG_END,
	SET_SLAVE_ADDR,
	SET_SLAVE_REG,
	SET_DELAY_TIME,
	LOCK_I2C_BUS,
	I2C_READ_REG,
	I2C_DEBUG_RW_fLAG,
	I2C_DEBUG_BUFF_FLAG,
	NOT_SUPPORT,
}i2c_debug_cmd_i;

struct i2c_debug_cmd{
	i2c_debug_cmd_i cmd;
	char cmd_str[I2C_CMD_REG_STR_LEN];
};

/*the arrary index must be same as i2c_debug_cmd_i */
static struct i2c_debug_cmd debug_cmd_array[]={
	{I2C_DEBUG_BEGIN,"begn"},
	{I2C_DEBUG_END,"ends"},
	{SET_SLAVE_ADDR, "addr"},
	{SET_SLAVE_REG, "rwrg"},
	{SET_DELAY_TIME,"dely"},
	{LOCK_I2C_BUS, "lkbs"},
	{I2C_READ_REG,"rdrg"},
	{I2C_DEBUG_RW_fLAG,"rwfg"},
	{I2C_DEBUG_BUFF_FLAG,"buff"},
};

typedef enum{
	FILE_NEW,
	FILE_APPEND,
	FILE_REMOVE,
}dgfile_ops_type;

struct reg_pair{
	i2c_debug_cmd_i type;
	char reg[32];
	char value[32];
};

#define SIZE_PER_NODE 256
struct reg_data_node{
	struct list_head list;
	unsigned int index;
	struct reg_pair reg_pairs[SIZE_PER_NODE];
};
static int i2cdev_debug_detach_adapter(struct i2c_adapter *adap);
static int i2cdev_debug_attach_adapter(struct i2c_adapter *adap);
static void i2c_debug_buff_process_work_func(struct work_struct *work);
static int i2c_debug_clear_buff(void);
static int i2cdev_debug_reg_read (struct i2c_client *client, const char *s_reg, const char* s_value);


static struct i2c_driver i2cdev_debug_driver = {
	.driver = {
		.name	= "i2c_debug_driver",
	},
	.attach_adapter	= i2cdev_debug_attach_adapter,
	.detach_adapter	= i2cdev_debug_detach_adapter,
};

void i2c_debug_logfile(char *name, dgfile_ops_type ops_type, const char* str)
{
	int fd;
	int len = 0;
	
	switch(ops_type)
	{
	case FILE_NEW:
		fd = sys_open(name, O_CREAT , 0);
		if (fd < 0){
			printk("error occured while opening file %s\n", name);
			return;
		}
		sys_close(fd);
		break;
	case FILE_APPEND:
		fd = sys_open(name, O_RDWR | O_CREAT | O_APPEND, 0);
		if (fd < 0){
			printk("error occured while opening file %s\n", name);
			return;
		}

		len = strlen(str);
		sys_write(fd, str, len);
		sys_close(fd);
		break;
	case FILE_REMOVE:
		sys_unlink(name);
		break;
	default:
		break;
	}
	
}

char *i2c_debug_strncpy(char *dest, const char *src, size_t count)
{
	char *tmp = dest;

	while (count) {
		if ((*tmp = *src) != ',')
			src++;
		else{
			*tmp = 0;
			break;
		}
		tmp++;
		count--;
	}
	return dest;
}

static struct i2c_debug_dev *i2c_dev_get_by_minor(unsigned index)
{
	struct i2c_debug_dev *i2c_dev;

	spin_lock(&i2c_dev_list_lock);
	list_for_each_entry(i2c_dev, &i2c_debug_dev_list, list) {
		if (i2c_dev->adap->nr == index)
			goto found;
	}
	i2c_dev = NULL;
found:
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}


static int i2cdev_debug_open(struct inode *inode, struct file *file)
{
	unsigned int minor = iminor(inode);
	struct i2c_debug_dev *i2c_dev;
	int ret = 0;

	I2C_DEBUG_MACRO("%s entry\n", __FUNCTION__);
	lock_kernel();
	i2c_dev = i2c_dev_get_by_minor(minor);
	if (!i2c_dev) {
		ret = -ENODEV;
		goto out;
	}
	g_i2c_dev = i2c_dev;
	file->private_data = i2c_dev;

	I2C_DEBUG_MACRO("%s\n",__FUNCTION__);
out:
	unlock_kernel();
	return ret;
}

static s32 i2c_debug_smbus_xfer_emulated(struct i2c_adapter * adapter, u16 addr,
                                   unsigned short flags,
                                   char read_write, u32 command, u32 command_len, int size,
                                   union i2c_smbus_data * data)
{
	unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX+3];
	unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX+2];
	int num = read_write == I2C_SMBUS_READ?2:1;
	struct i2c_msg msg[2] = { { addr, flags, 1, msgbuf0 },
	                          { addr, flags | I2C_M_RD, 0, msgbuf1 }
							};

	int i;
	int status;

	u32 mask[] = {
		0x000000FF,
		0x0000FF00,
		0x00FF0000,
		0xFF000000,
	};

	I2C_DEBUG_MACRO("%s read_write=%d, command=0x%x, command_len=%d\n",__FUNCTION__, read_write, command, command_len);

	memset(msgbuf0, 0, I2C_SMBUS_BLOCK_MAX+3);
	memset(msgbuf1, 0, I2C_SMBUS_BLOCK_MAX+2);
	for(i = 0; i < command_len; i++)
		msgbuf0[i] = (command & mask[command_len-i-1]) >> 8*(command_len - i - 1);
	
	if (read_write == I2C_SMBUS_READ) {
		msg[0].len = command_len;
		msg[1].len = data->block[0];
	} else {
		msg[0].len = data->block[0] + command_len;
		if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 1) {
			printk("Invalid block write size %d\n",data->block[0]);
			return -EINVAL;
		}
		for (i = 0; i < data->block[0]; i++)
			msgbuf0[i+command_len] = data->block[i+1];
	}
 
	if(I2C_DEBUG_SHOW_XFER_BUF & i2c_debug_mask){
		I2C_DEBUG_MACRO("msg[0].addr=0x%x\n", msg[0].addr);
		I2C_DEBUG_MACRO("msg[0].len=0x%x\n", msg[0].len);
		for(i = 0; i < I2C_SMBUS_BLOCK_MAX+3; i++)
			I2C_DEBUG_MACRO("msgbuf0[%d]=0x%x\n", i, msgbuf0[i]);
		
		I2C_DEBUG_MACRO("msg[1].addr=0x%x\n", msg[1].addr);
		I2C_DEBUG_MACRO("msg[1].len=0x%x\n", msg[1].len);
		for(i = 0; i < I2C_SMBUS_BLOCK_MAX+2; i++)
			I2C_DEBUG_MACRO("msgbuf1[%d]=0x%x\n", i, msgbuf1[i]);
	}
	status = i2c_transfer(adapter, msg, num);
	I2C_DEBUG_MACRO("%s status=%d\n",__FUNCTION__, status);
	if (status < 0)
		return status;

	
	if (read_write == I2C_SMBUS_READ){
		for (i = 0; i < data->block[0]; i++)
			data->block[i+1] = msgbuf1[i];
	}

	return 0;
}

/*only can wirte a reg
*/
static int i2cdev_debug_reg_write (struct i2c_client *client, const char *s_reg, const char* s_value)
{
	u32 reg = 0;
	u32 reg_len = 0;
	u32 value = 0;
	__u8 b_val = 0;
	u32 value_len = 0;
	u32 command = 0;
	u32 command_len = 0;
	char *endp;
	int i = 0;
	char tmp[3] = {0};
	union i2c_smbus_data data;
	
	reg_len = strlen(s_reg);
	value_len = strlen(s_value);

	reg = simple_strtoull(s_reg, &endp, 16);
	if (s_reg == endp) {
		printk("%s: %s is not a number\n", __FUNCTION__, s_reg);
		return -EINVAL;
	}
	
	value = simple_strtoull(s_value, &endp, 16);
	if (s_value == endp) {
		printk("%s: %s is not a number\n", __FUNCTION__, s_value);
		return -EINVAL;
	}

	I2C_DEBUG_MACRO("%s:atoi reslut, reg=0x%x, reg_len=%d, value=0x%x, value_len=%d\n", 
		__FUNCTION__, reg,reg_len,value,value_len);

	if(value_len > I2C_SMBUS_BLOCK_MAX){
		I2C_DEBUG_MACRO("%s: NOT SUPPORT!\n", __FUNCTION__);
		return -1;
	}
	command = reg;
	command_len = (reg_len - 2)/2;

	value_len = (value_len - 2)/2;
	data.block[0] = value_len;

	for(i = 0; i < value_len; i++){
		memset(tmp, 0, 3);
		strncpy(tmp, (s_value+i*2 + 2), 2);
		b_val = simple_strtoull(tmp, &endp, 16);
		if (s_value == endp) {
			printk("%s: %s is not a number\n", __FUNCTION__, s_value);
			return -EINVAL;
		}
		data.block[1+i] = b_val;
		I2C_DEBUG_MACRO("block[%d]=0x%x\n", 1+i, data.block[1+i]);
	}
	
	return i2c_debug_smbus_xfer_emulated(client->adapter, client->addr, 0,
				  I2C_SMBUS_WRITE, command, command_len,
				  I2C_SMBUS_I2C_BLOCK_DATA, &data);

}

/*only can wirte a reg
*/
static int i2cdev_debug_reg_read (struct i2c_client *client, const char *s_reg, const char* s_value)
{
	u32 reg = 0;
	u32 reg_len = 0;
	u32 read_len = 0;
	u32 command = 0;
	u32 command_len = 0;
	char *endp;
	union i2c_smbus_data data;
	s32 ret = 0;
	char log_buf[100] = {0};
	char tmp[20] = {0};
	int i = 0;

	reg_len = strlen(s_reg);

	reg = simple_strtoull(s_reg, &endp, 16);
	if (s_reg == endp) {
		printk("%s: %s is not a number\n", __FUNCTION__, s_reg);
		return -EINVAL;
	}
	
	read_len = simple_strtoull(s_value, &endp, 16);
	if (s_value == endp) {
		printk("%s: %s is not a number\n", __FUNCTION__, s_value);
		return -EINVAL;
	}

	I2C_DEBUG_MACRO("%s:atoi reslut, reg=0x%x, reg_len=%d, read len=0x%x\n", 
		__FUNCTION__, reg,reg_len,read_len);

	if(read_len > I2C_SMBUS_BLOCK_MAX){
		I2C_DEBUG_MACRO("%s: NOT SUPPORT!\n", __FUNCTION__);
		return -1;
	}
	command = reg;

	//read bytes number
	data.block[0] = read_len;
	command_len = (reg_len-2)/2;
	
	ret =  i2c_debug_smbus_xfer_emulated(client->adapter, client->addr, 0,
				  I2C_SMBUS_READ, command,command_len,
				  I2C_SMBUS_I2C_BLOCK_DATA, &data);

	if(ret){
		sprintf(log_buf, "read i2c data failed: addr=0x%x, flags=0x%x, reg=0x%x, read_len =%02dByte\n", 
			client->addr ,client->flags ,command, read_len);
		i2c_debug_logfile(I2C_DEBUG_LOG_FILE, FILE_APPEND,log_buf);
		return -1;
	}
	else{
		//sprintf(log_buf, "read i2c data sucess: addr=0x%x, flags=0x%x, reg=0x%x, read_len =%02dByte\n", 
		//	client->addr ,client->flags ,command, read_len);
		//i2c_debug_logfile(I2C_DEBUG_LOG_FILE, FILE_APPEND,log_buf);

		memset(log_buf, 0 ,100);
		sprintf(log_buf, "reg:0x%x = ", command);
		for(i = 0; i < data.block[0]; i++)
		{
			sprintf(tmp,"0x%x ", data.block[i+1]);
			strcat(log_buf, tmp);
		}
		strcat(log_buf, "\n");
		I2C_DEBUG_MACRO("%s", log_buf);
		i2c_debug_logfile(I2C_DEBUG_LOG_FILE, FILE_APPEND,log_buf);
	}

	return 0;
}

static ssize_t i2cdev_debug_read (struct file *file, char __user *buf, size_t count,
                            loff_t *offset)
{
	return 0;
}

static int i2cdev_debug_release(struct inode *inode, struct file *file)
{

	I2C_DEBUG_MACRO("%s entry\n", __FUNCTION__);
	//i2c_put_adapter(client->adapter);
 
	return 0;
}


/*set i2c client address
*/
static int i2c_debug_set_addr(struct i2c_client * client, char * value)
{
	u32 addr = 0;
	char *endp;
	
	addr = simple_strtoull(value, &endp, 16);
	if (value == endp) {
		printk("%s: %s is not a number\n", __FUNCTION__, value);
		return -EINVAL;
	}
 
	I2C_DEBUG_MACRO("set i2c addr: 0x%x\n", addr);
	client->adapter->debug_addr = addr;
	client->addr = addr;
	return 0;
}

static int i2c_debug_lock_bus(struct i2c_client * client, char *data)
{
	char *endp;
	u32 i_data = simple_strtoull(data, &endp, 16);

	if (data == endp) {
		printk("%s: %s is not a number\n", __FUNCTION__, data);
		return -EINVAL;
	}

	if(i_data == 1)
		client->adapter->bus_debug_flag = 1;
	else
		client->adapter->bus_debug_flag = 0;

	return 0;
}

static int i2c_debug_clear_buff(void)
{
	struct reg_data_node *pos;
	struct list_head * l_pos;
	int j = 0;
	
	I2C_DEBUG_MACRO("%s\n",__FUNCTION__);
	
	if(list_empty(&regs_data_list))
		return 0;

	spin_lock(&regs_data_list_lock);
	while (!list_empty(&regs_data_list)){
		l_pos = regs_data_list.next;
		list_del(l_pos);
		printk("delete node %d\n", j);
		pos = container_of(l_pos, struct reg_data_node, list);
		if(pos)
			kfree(pos);
		j++;
	}
	spin_unlock(&regs_data_list_lock);
	return 0;
}

static int i2c_debug_mdelay(const char *s_ms)
{
	u32 delay = 0;
	char *endp;
	
	delay = simple_strtoull(s_ms, &endp, 16);
	if (s_ms == endp) {
		printk("%s: %s is not a number\n", __FUNCTION__, s_ms);
		return -EINVAL;
	}
 
	I2C_DEBUG_MACRO("delay time: 0x%x\n", delay);

	mdelay(delay);
	return 0;
	
}

static void i2c_debug_set_hook(struct i2c_debug_dev *pdev, const char *s_flag)
{
	u32 flg = 0;
	char *endp;
	
	flg = simple_strtoull(s_flag, &endp, 16);
	if (s_flag == endp) {
		printk("%s: %s is not a number\n", __FUNCTION__, s_flag);
		return;
	}
 
	I2C_DEBUG_MACRO("hook flag: 0x%x\n", flg);

	pdev->hook_flag = flg;
}

/*process i2c debug command
*/
static int i2c_debug_cmd_process(struct work_struct *work, i2c_debug_cmd_i cmd, 
		char * reg, char* value)
{
	int res = 0;
	struct i2c_debug_dev *pdev;
	struct i2c_client * client;
	char log_buf[100];

	memset(log_buf, 0, 100);
	pdev = container_of(work, struct i2c_debug_dev, work);
	client = pdev->client;

	I2C_DEBUG_MACRO("%s:cmd = %d, reg = %s, value=%s\n",__FUNCTION__, cmd , reg, value);

	switch(cmd)
	{
	case I2C_DEBUG_BEGIN:
		i2c_debug_logfile(I2C_DEBUG_BEGIN_FILE, FILE_NEW, "");
		break;
	case SET_SLAVE_ADDR:
		i2c_debug_set_addr(client, value);
		break;
	case SET_SLAVE_REG:
		res = i2cdev_debug_reg_write(client, reg, value);
		if(res){
			I2C_DEBUG_MACRO("%s: write i2c(0x%x) failed, reg = %s, value = %s\n", __FUNCTION__,client->addr,
				reg, value);
			
			sprintf(log_buf, "write failed, reg = %s, value = %s\n", reg, value);
		}
		else{
			I2C_DEBUG_MACRO("%s: write i2c(0x%x) success, reg = %s, value = %s\n", __FUNCTION__,client->addr,
				reg, value);
			
			sprintf(log_buf, "write success, reg = %s, value = %s\n", reg, value);
		}
		i2c_debug_logfile(I2C_DEBUG_LOG_FILE,FILE_APPEND,log_buf);
		break;
	case I2C_READ_REG:
		i2cdev_debug_reg_read(client, reg, value);
		break;
	case LOCK_I2C_BUS:
		i2c_debug_lock_bus(client, value);
		break;
	case SET_DELAY_TIME:
		i2c_debug_mdelay(value);
		break;
	case I2C_DEBUG_END:
		i2c_debug_logfile(I2C_DEBUG_END_FILE, FILE_NEW, "");
		break;
	case I2C_DEBUG_BUFF_FLAG:
		i2c_debug_set_hook(pdev, value);
		break;
	default:
		break;
	}
	return res;
}

/*cmd type in the fist four chars
*/
static i2c_debug_cmd_i i2c_debug_parse_i2c_cmd(char *cmd)
{
	int i = 0;
	int debug_cmd_array_len = sizeof(debug_cmd_array)/sizeof(struct i2c_debug_cmd);
	int res;
	
	if(cmd == NULL)
		return NOT_SUPPORT;
	
	for(i = 0; i < debug_cmd_array_len; i++)
	{
		res = strncmp(cmd, debug_cmd_array[i].cmd_str, I2C_CMD_REG_STR_LEN-1);
		if(!res)
			break;
	}

	if(i == debug_cmd_array_len){
		I2C_DEBUG_MACRO("%s: NOT_SUPPORT parse cmd: %s \n",__FUNCTION__, cmd);
		return NOT_SUPPORT;
	}

	I2C_DEBUG_MACRO("%s: parse cmd = %d\n",__FUNCTION__, debug_cmd_array[i].cmd);
	return debug_cmd_array[i].cmd;
}

/* data format: 
1.8bit reg address N bit data(N <= I2C_SMBUS_BLOCK_MAX)
regv=0x11,   0x1111

2. 16bit reg address, N bit data(N <= I2C_SMBUS_BLOCK_MAX)
regv=0x1111,0x1111

3. begn=0x0001

4.ends=0x0001

5.rwfg=0x0001

*/
static int i2c_debug_add_regs_to_buf(const char* data, i2c_debug_cmd_i type)
{
	struct reg_data_node *node;
	struct reg_data_node *node_last;
	struct list_head *end;
	struct reg_pair tmp;
	int len, total_len;
	int is_new_node = 0;
	const char *parse_data = 0;
	
	//if list is empty or the last node is full, we create a new node
	if(list_empty(&regs_data_list))
		is_new_node = 1;
	else{
		end = regs_data_list.prev;
		node_last = list_entry(end, struct reg_data_node, list);
		if(node_last->index == SIZE_PER_NODE)
			is_new_node = 1;
	}

	I2C_DEBUG_MACRO("%s: is_new_node=%d\n",__FUNCTION__,is_new_node);
	if(is_new_node)
	{
		node = (struct reg_data_node*)kzalloc(sizeof(struct reg_data_node),GFP_KERNEL);
		if(!node)
		{
			printk("%s: node alloc failed\n", __FUNCTION__);
			return -ENOMEM;
		}
		node->index = 0;
		spin_lock(&regs_data_list_lock);
		list_add_tail(&node->list, &regs_data_list);
		spin_unlock(&regs_data_list_lock);
		node_last = node;
	}
	
	memset(tmp.reg, 0, 32);
	memset(tmp.value,0,32);

	tmp.type= type;
	parse_data = data+I2C_CMD_REG_STR_LEN;
	total_len = strlen(parse_data);

	I2C_DEBUG_MACRO("%s: tmp.type=%d, parse_data=%s, total_len=%d\n",__FUNCTION__,tmp.type, parse_data,total_len);
	//in this switch,we parse the command, and save reg value to data list
	switch(tmp.type)
	{
	case SET_SLAVE_REG:
	case I2C_READ_REG:
		i2c_debug_strncpy(tmp.reg, parse_data, total_len);//skip 0x
		len = strlen(tmp.reg);
		i2c_debug_strncpy(tmp.value, parse_data+len+1, total_len-len-1);
		strcpy(node_last->reg_pairs[node_last->index].reg, tmp.reg);
		strcpy(node_last->reg_pairs[node_last->index].value, tmp.value);
		break;
	case SET_SLAVE_ADDR:
	case I2C_DEBUG_BEGIN:
	case I2C_DEBUG_END:
	case SET_DELAY_TIME:
	case LOCK_I2C_BUS:
	case I2C_DEBUG_RW_fLAG:
	case I2C_DEBUG_BUFF_FLAG:
		strcpy(node_last->reg_pairs[node_last->index].value,parse_data);
		break;
	default:
		break;
	}
	node_last->reg_pairs[node_last->index].type = tmp.type;
	I2C_DEBUG_MACRO("add new reg: type = %d, reg=%s, value=%s\n",node_last->reg_pairs[node_last->index].type,
		node_last->reg_pairs[node_last->index].reg, node_last->reg_pairs[node_last->index].value);
	node_last->index++;

	return 0;
}

void i2c_debug_parse_file(const char* data, struct i2c_debug_dev* i2c_dev)
{
	char tmp[I2C_DEBUG_CMD_LEN];
	i2c_debug_cmd_i cmd_type;
	int i = 0;
	int j = 0;
	
	memset(tmp, 0, I2C_DEBUG_CMD_LEN);
	cmd_type = NOT_SUPPORT;
	
	while(data[i] != 0)
	{
	   if(data[i] != 13)//13 10 = '\n'
		 tmp[j]=data[i];
	   else{
	   	  I2C_DEBUG_MACRO("%s: parse cmd = %s\n",__FUNCTION__,tmp);
	   	  cmd_type = i2c_debug_parse_i2c_cmd(tmp);
		  i2c_debug_add_regs_to_buf(tmp,cmd_type);
		  
		  memset(tmp, 0, I2C_DEBUG_CMD_LEN);
		  j = -1;
		  i++; //skip 10
	   }
	   j++;
	   i++;
	}
	if(tmp[0] != NULL){
		I2C_DEBUG_MACRO("%s: parse cmd = %s\n",__FUNCTION__,tmp);
		cmd_type = i2c_debug_parse_i2c_cmd(tmp);
		i2c_debug_add_regs_to_buf(tmp,cmd_type);
	}
	
	if(cmd_type == I2C_DEBUG_END){
		I2C_DEBUG_MACRO("%s: queue work\n",__FUNCTION__);
		queue_work(i2c_debug_wq, &i2c_dev->work);
	}

}
static ssize_t i2cdev_debug_cmd_receive (struct file *file, const char __user *buf, size_t count,
                             loff_t *offset)
{
	char *tmp;

	struct i2c_debug_dev* i2c_dev = (struct i2c_debug_dev*)file->private_data;
	
	if (count > 8192)
		count = 8192;

	tmp = kmalloc(count,GFP_KERNEL);
	if (tmp==NULL)
		return -ENOMEM;
	
	if (copy_from_user(tmp,buf,count)) {
		kfree(tmp);
		return -EFAULT;
	}

	
	I2C_DEBUG_MACRO("receive org data: %s\n",tmp);

	i2c_debug_parse_file(tmp, i2c_dev);

	kfree(tmp);
	return count;
}

static const struct file_operations i2cdev_debug_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= i2cdev_debug_read,
	.write		= i2cdev_debug_cmd_receive,
	.open		= i2cdev_debug_open,
	.release	= i2cdev_debug_release,
};

static struct i2c_debug_dev *i2c_debug_dev_save_adapter(struct i2c_adapter *adap)
{
	struct i2c_debug_dev *i2c_dev;

	if (adap->nr >= I2C_MINORS) {
		printk(KERN_ERR "i2c-dev: Out of device minors (%d)\n",
		       adap->nr);
		return ERR_PTR(-ENODEV);
	}

	i2c_dev = kzalloc(sizeof(struct i2c_debug_dev), GFP_KERNEL);
	if (!i2c_dev)
		return ERR_PTR(-ENOMEM);
	i2c_dev->adap = adap;

	spin_lock(&i2c_dev_list_lock);
	list_add_tail(&i2c_dev->list, &i2c_debug_dev_list);
	spin_unlock(&i2c_dev_list_lock);
	return i2c_dev;
}

static void return_i2c_debug_dev(struct i2c_debug_dev *i2c_dev)
{
	spin_lock(&i2c_dev_list_lock);
	list_del(&i2c_dev->list);
	spin_unlock(&i2c_dev_list_lock);
	kfree(i2c_dev);
}


/*when a adapter registered, we must bind a i2c dev to it, so we can
   use the adapter algo to transfer i2c data
*/
static int i2cdev_debug_attach_adapter(struct i2c_adapter *adap)
{
	struct i2c_debug_dev *i2c_dev;
	struct i2c_client *client;
	int res;
	int major;
	
	dev_t dev_no = g_dev_no;
	
	major = MAJOR(dev_no);
	
	i2c_dev = i2c_debug_dev_save_adapter(adap);
	if (IS_ERR(i2c_dev))
		return PTR_ERR(i2c_dev);

	i2c_dev->dev_no = MKDEV(major, adap->nr);

	i2c_dev->dev = device_create(i2c_debug_dev_class, &adap->dev, i2c_dev->dev_no, NULL, \
		"i2c-debug-%d", adap->nr);
	
	if (IS_ERR(i2c_dev->dev)) {
		res = PTR_ERR(i2c_dev->dev);
		goto error;
	}
	
	cdev_init(&i2c_dev->cdev, &i2cdev_debug_fops);
	res = cdev_add(&i2c_dev->cdev, i2c_dev->dev_no, 1);
	if(res)
	{
		printk("i2c-debug-dev: minor = %d, cdev_add fail\n", adap->nr);
		goto error_destroy;
	}
		
	client = kzalloc(sizeof(*client), GFP_KERNEL);
	if (!client) {
		res = -ENOMEM;
		goto error_destroy;
	}

	i2c_dev->client = client;
	snprintf(client->name, I2C_NAME_SIZE, "i2c-debug-dev %d", adap->nr);
	client->driver = &i2cdev_debug_driver;

	client->adapter = adap;
	//i2c_dev->work = &g_work;
	INIT_WORK(&i2c_dev->work, i2c_debug_buff_process_work_func);
	
	I2C_DEBUG_MACRO("i2c-dev: adapter [%s] registered as minor %d\n",
		 adap->name, adap->nr);
	
	return 0;
error_destroy:
	device_destroy(i2c_debug_dev_class, i2c_dev->dev_no);
error:
	return_i2c_debug_dev(i2c_dev);
	return res;
}

static int i2cdev_debug_detach_adapter(struct i2c_adapter *adap)
{
	struct i2c_debug_dev *i2c_dev;
	struct i2c_client *client;
	
	i2c_dev = i2c_dev_get_by_minor(adap->nr);
	if (!i2c_dev) /* attach_adapter must have failed */
		return 0;

	/*release the memory of client*/
	client = i2c_dev->client;
	if(client){
		kfree(client);
	}
	
	//device_remove_file(i2c_dev->dev, &dev_attr_name);
	return_i2c_debug_dev(i2c_dev);
	device_destroy(i2c_debug_dev_class, MKDEV(I2C_MAJOR, adap->nr));

	pr_debug("i2c-dev: adapter [%s] unregistered\n", adap->name);
	return 0;
}


//used by driver
int i2c_dbg_transfer(void)
{
	struct reg_data_node *pos;
	int i = 0;
	char *pvalue = "0x0";
	int len = 0;

	len = strlen(pvalue);

	//clear hook flag
	list_for_each_entry(pos, &regs_data_list, list){
		for(i=0; i<pos->index; i++)
			if(pos->reg_pairs[i].type == I2C_DEBUG_BUFF_FLAG){
				len = strlen(pos->reg_pairs[i].value);
				memset(pos->reg_pairs[i].value, 0 ,len);
				strcpy(pos->reg_pairs[i].value, pvalue);
				I2C_DEBUG_MACRO("%s: change hook flag to %s\n",__FUNCTION__, pos->reg_pairs[i].value);
			}
	}

	if((I2C_DEBUG_HOOK_QUEUE_WORK_OFF & i2c_debug_mask) == 0){
		I2C_DEBUG_MACRO("%s: queue work\n", __FUNCTION__);
		g_i2c_dev->hook_flag = 0;
		queue_work(i2c_debug_wq, &g_i2c_dev->work);
	}
		
	return 0;
}

static int i2c_dbg_get_buff(void *data, u64 *val)
{
	struct reg_data_node *pos;
	int i = 0;
	int j = 0;
	
	list_for_each_entry(pos, &regs_data_list, list){
		printk("===========================\n");
		printk("*    node index: %d\n",j);
		printk("===========================\n");
		for(i=0; i<pos->index; i++)
			printk("%d. type=%d(%s), reg=%s, value=%s\n",i, pos->reg_pairs[i].type, \
				debug_cmd_array[pos->reg_pairs[i].type].cmd_str ,pos->reg_pairs[i].reg,pos->reg_pairs[i].value);
		printk("===========================\n");
		j++;
	}
		
	return 0;
}

static int i2c_dbg_set_buff(void *data, u64 val)
{
	I2C_DEBUG_MACRO("%s: data=%s, val=%d\n",__FUNCTION__, (char*)data, (int)val);

	
	if((I2C_DEBUG_CLEAR_BUF & i2c_debug_mask) != 0)
	  i2c_debug_clear_buff();
	else
	  i2c_dbg_transfer();
	
	return 0;
}

static void i2c_debug_buff_process_work_func(struct work_struct *work)
{
	struct reg_data_node *pos;
	struct i2c_debug_dev *pdev;
	int i = 0;
	int j = 0;
	
	pdev = container_of(work, struct i2c_debug_dev, work);

	I2C_DEBUG_MACRO("%s: entry\n",__FUNCTION__);

	list_for_each_entry(pos, &regs_data_list, list){
		for(i=0; i<pos->index; i++){
			I2C_DEBUG_MACRO("pos->index = %d\n", pos->index);
			I2C_DEBUG_MACRO("%d. type=%d(%s), reg=%s, value=%s\n",i, pos->reg_pairs[i].type, \
				debug_cmd_array[pos->reg_pairs[i].type].cmd_str ,pos->reg_pairs[i].reg,pos->reg_pairs[i].value);

			i2c_debug_cmd_process(work, pos->reg_pairs[i].type, pos->reg_pairs[i].reg, pos->reg_pairs[i].value);
			if(pdev->hook_flag == 1){
				I2C_DEBUG_MACRO("%s: break due to hook\n", __FUNCTION__);
				i2c_debug_logfile(I2C_DEBUG_LOG_FILE,FILE_APPEND,"i2c transfer has been hooked!\n");
				i2c_debug_logfile(I2C_DEBUG_END_FILE, FILE_NEW, "");
				break;
			}
		}
		I2C_DEBUG_MACRO("j = %d\n", j);
		j++;
	}

	//auto clear buffer
	if((0 == (I2C_DEBUG_CLEAR_BUF_MANUAL & i2c_debug_mask)) && (pdev->hook_flag != 1))
		i2c_debug_clear_buff();

}
DEFINE_SIMPLE_ATTRIBUTE(i2c_dbg_fops, i2c_dbg_get_buff, i2c_dbg_set_buff, "%llu\n");

static int __init i2c_debug_dev_init(void)
{
	int res;
	dev_t dev;
	
	printk(KERN_INFO "huawei i2c debug driver\n");

	i2c_debug_dev_class = class_create(THIS_MODULE, "i2c-debug");
	if (IS_ERR(i2c_debug_dev_class)) {
		res = PTR_ERR(i2c_debug_dev_class);
		goto out;
	}

	/*allocate cdev major number,max minor is 255
	*/
	res = alloc_chrdev_region(&dev, 0, I2C_MINORS, I2C_DEBUG_DRIVER_NAME);
	if (res)
		goto out_unreg_class;

	g_dev_no = dev;

	res = i2c_add_driver(&i2cdev_debug_driver);
	if (res)
		goto out_unreg_chrdev;

	debugfs_create_file("i2c-debug", 0644, NULL, NULL, &i2c_dbg_fops);
	
	i2c_debug_wq = create_singlethread_workqueue("i2c_debug_wq");
	if (!i2c_debug_wq) {
		printk("create i2c_debug_wq error\n");
		res = -ENOMEM;
		goto out_del_i2c_driver;
	}

	INIT_WORK(&g_work, i2c_debug_buff_process_work_func);
	return 0;
out_del_i2c_driver:
	i2c_del_driver(&i2cdev_debug_driver);
out_unreg_chrdev:
	unregister_chrdev_region(dev, I2C_MINORS);

out_unreg_class:
	class_destroy(i2c_debug_dev_class);
out:
	printk(KERN_ERR "%s: Driver Initialisation failed\n", __FILE__);
	return res;
}

static void __exit i2c_debug_dev_exit(void)
{
	
	i2c_del_driver(&i2cdev_debug_driver);
	class_destroy(i2c_debug_dev_class);
	unregister_chrdev(I2C_MAJOR,"i2c");
}

MODULE_DESCRIPTION("huawei i2c debug driver");
MODULE_LICENSE("GPL");

module_init(i2c_debug_dev_init);
module_exit(i2c_debug_dev_exit);

