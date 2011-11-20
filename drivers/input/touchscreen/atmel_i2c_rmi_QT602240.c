
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/atmel_i2c_rmi.h>
#include <linux/ctype.h>
#include <linux/earlysuspend.h>
#include <mach/gpio.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <mach/vreg.h>
#include <linux/touch_platform_config.h>

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
#define TS_DEBUG_TS(fmt, args...) printk(KERN_INFO fmt, ##args)
#else
#define TS_DEBUG_TS(fmt, args...)
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

/*use this to contrl the debug message*/
static int atmel_debug_mask;
module_param_named(atmel_debug, atmel_debug_mask, int,
		S_IRUGO | S_IWUSR | S_IWGRP);

#define ATMEL_DBG_MASK(x...) do {\
	if (atmel_debug_mask) \
		printk(KERN_DEBUG x);\
	} while (0)


/*max y for the key region. total height of the touchscreen is 91.5mm,
and the height of the key region is 8.5mm, TS_Y_MAX * 8.5 /91.5 */
#undef TOUCH_12BIT
#ifdef TOUCH_12BIT
#define TS_X_MAX 4095
#define TS_Y_MAX 4095
#else
#define TS_X_MAX 1023
#define TS_Y_MAX 1023
#endif
#define TS_X_OFFSET  3*(TS_X_MAX/LCD_X_MAX)
#define TS_Y_OFFSET  TS_X_OFFSET

#define LCD_X_MAX 479
#define ATMEL_FAMILY_ID 0x80

/*! \brief Returned by get_object_address() if object is not found. */
#define OBJECT_NOT_FOUND   0u

/* Array of I2C addresses where we are trying to find the chip. */
extern uint8_t i2c_addresses[];

/*! Address where object table starts at touch IC memory map. */
#define OBJECT_TABLE_START_ADDRESS      7U

/*! Size of one object table element in touch IC memory map. */
#define OBJECT_TABLE_ELEMENT_SIZE     	6U

/*! Offset to RESET register from the beginning of command processor. */
#define RESET_OFFSET		            0u

/*! Offset to BACKUP register from the beginning of command processor. */
#define BACKUP_OFFSET		1u

/*! Offset to CALIBRATE register from the beginning of command processor. */
#define CALIBRATE_OFFSET	2u

/*! Offset to REPORTALL register from the beginning of command processor. */
#define REPORTATLL_OFFSET	3u

/*! Offset to DEBUG_CTRL register from the beginning of command processor. */
#define DEBUG_CTRL_OFFSET	5u

#define DEBUG_INC_PAGE 0x01
#define DEBUG_DEC_PAGE 0x02
#define DEBUG_DELTAS_MODE 0x10
#define DEBUG_REFER_MODE 0x11
#define DEBUG_CTE_MODE 0x31


#define GEN_MESSAGEPROCESSOR_T5 5
#define GEN_COMMANDPROCESSOR_T6 6
#define GEN_POWERCONFIG_T7 7
#define T7_INSTANCE 0
#define GEN_ACQUISITIONCONFIG_T8 8

#define TOUCH_MULTITOUCHSCREEN_T9 9

#define TOUCH_KEYARRAY_T15 15

#define PROCG_GRIPFACESUPPRESSION_T20 20
#define PROCG_NOISESUPPRESSION_T22 22

#define TOUCH_PROXIMITY_T23 23

#define PROCI_ONETOUCHGESTUREPROCESSOR_T24 24
#define SPT_SELFTEST_T25 25
#define PROCI_TWOTOUCHGESTUREPROCESSOR_T27 27


#define SPT_CTECONFIG_T28 28

#define DEBUG_DIAGNOSTIC_T37 37

#define KEY_RELEASE 0x0
#define KEY_NUHBER1 0x1
#define KEY_NUHBER2 0x2
#define KEY_NUHBER3 0x4
#define KEY_NUHBER4 0x8

struct info_id_t {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_x_size;
	u8 matrix_y_size;
	u8 num_declared_objects;
};

struct object_t {
	u8 object_type;
	u16 object_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;
};

struct info_block_t {
	struct info_id_t id;
	struct object_t *pobject_table;
	u32 CRC;
};

/*! \brief Struct holding the object type / instance info.
 * 
 * Struct holding the object type / instance info. An array of these maps
 * report id's to object type / instance (array index = report id).  Note
 * that the report ID number 0 is reserved.
 * 
 */

struct report_id_map_t {
   u8 object_type; 	/*!< Object type. */
   u8 instance;   	    /*!< Instance number. */
};

static struct info_block_t info_block;

/*! Pointer to report_id - > object type/instance map structure. */
static struct report_id_map_t *report_id_map = NULL;

/*! Maximum report id number. */
static int max_report_id = 0;

/*! Maximum message length. */
static u8 max_message_length;

static u8 T9_base_reportID = 0;

/*! Message processor address. */
u16 message_processor_address;

/*! Command processor address. */
u16 command_processor_address;

/*! Command processor address. */
u16 debug_diagnostic_address;

/*! Message buffer holding the latest message received. */
u8 *touch_msg = NULL;

static uint32_t timer_tick = 0;

static u8 atmel_timer = 0;

#define ENABLE 1

#define DISABLE 0

static u8 cal_check_flag = 0;

/* Unique ID allocation */
static struct i2c_client *g_client;

static struct workqueue_struct *atmel_wq;

static DECLARE_MUTEX(atmel_i2c_lock);

struct atmel_ts_data {
	uint16_t addr;
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct work_struct  work;
	int use_irq;
        
	struct hrtimer timer;	
	int (*power)(struct i2c_client* client, int on);
	unsigned int test;
	unsigned int lock;
	int touch_x;
	int touch_y;
	int touchamplitude;
	int sizeoftouch;
	bool is_support_multi_touch; //multi_touch function switch

    struct early_suspend early_suspend;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmel_ts_early_suspend(struct early_suspend *h);
static void atmel_ts_late_resume(struct early_suspend *h);
#endif

void cal_maybe_good(void);

static int atmel_ts_power(struct i2c_client *client, int on);

static inline void touchkey_test(struct atmel_ts_data *ts,
			unsigned long value)
{
	if (value == 0 && ts->test > 0)
	{
		ts->test = value;
		TS_DEBUG_TS("\ntouch key test mode disable!\n");
	}
	else if (value > 0 && ts->test == 0)
	{
		ts->test = value;
		TS_DEBUG_TS("\ntouch key test mode enable!\n");
	}
	else
		TS_DEBUG_TS(KERN_WARNING "%s: getting touch key test command value = %lud\n",__func__, value);
}
static ssize_t touchkey_test_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct atmel_ts_data *ts = dev_get_drvdata(dev);
	if( ts )
	      return sprintf(buf, "%d\n", ts->test);
	else
	      return 0;
}

static ssize_t touchkey_test_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct atmel_ts_data *ts = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
	      count++;

	if (count == size) {
		ret = count;
		touchkey_test(ts, state);
	}

	return ret;
}

static DEVICE_ATTR(test, S_IRUGO | S_IWUSR, touchkey_test_show, touchkey_test_store); 

static inline void touch_lock(struct atmel_ts_data *ts, unsigned long value)
{
	if (value == 0 && ts->lock > 0)
	{
		ts->test = value;
		TS_DEBUG_TS("\ntouch lock mode disable!\n");
	}
	else if (value > 0 && ts->lock == 0)
	{
		ts->test = value;
		TS_DEBUG_TS("\ntouch lock mode enable!\n");
	}
	else
		TS_DEBUG_TS(KERN_WARNING "%s: getting touch lock command value = %lud\n",__func__, value);
}

static ssize_t touch_lock_show(struct device *dev,
			struct device_attribute *attr,
			char *buf)
{
	struct  atmel_ts_data *ts = dev_get_drvdata(dev);
	if( ts )
		return sprintf(buf, "%d\n", ts->lock);
	else
		return 0;
}

static ssize_t touch_lock_store(struct device *dev,
			struct device_attribute *attr,
			const char *buf, size_t size)
{
	struct atmel_ts_data *ts = dev_get_drvdata(dev);
	ssize_t ret = -EINVAL;
	char *after;
	unsigned long state = simple_strtoul(buf, &after, 10);
	size_t count = after - buf;

	if (*after && isspace(*after))
		count++;

	if (count == size) {
		ret = count;
		touch_lock(ts, state);
	}

	return ret;
}

static DEVICE_ATTR(lock, S_IRUGO | S_IWUSR, touch_lock_show, touch_lock_store); 


/*---------------- Function Header -------------------------------------------

	FUNCTION	int write_mem_I2C( u16 Address, u8 ByteCount, u8 *pData )
	PURPOSE		Writes multiple bytes to device at 'I2cAddress'.
				Calls address_slave() to address device to write
				Writes bytes.
	INPUT		Address:    specifies where to write to in the device
				ByteCount: 	number of bytes to read
				Data:       Pointer to write-data (byte) array
	OUTPUT		0 if read completes
				others if failed
	MODIFIES	Nothing
				 
----------------------------------------------------------------------------*/
static
int write_mem_I2C( u16 Address, u8 ByteCount, u8 *pData )
{
	int ret;
	u8 data[3];
	u8 i;
	u8 *ReadDataPtr;
	if( g_client == NULL )	/* Not initialize the client */
		return -1;	
		
	ReadDataPtr = pData;
	for ( i = 0; i < ByteCount; i++)
	{
		data[0] = (Address + i) & 0xff;
		data[1] = (Address + i) >> 8;
		data[2] = *ReadDataPtr++;
		ret = i2c_master_send(g_client, data, 3);
		if(ret<0) {
			TS_DEBUG_TS(KERN_ERR "ATMEL: write reg [%04x] failed %d\n", Address, ret);
			return ret;
		}
	}
	
	return 0;
}

/*---------------- Function Header -------------------------------------------

	FUNCTION	int read_mem_I2C( u16 Address, u8 ByteCount, u8 *pData )
	PURPOSE		Reads multiple bytes from device at 'I2cAddress'.
				Calls write_mem() to set address-pointer = Address
				Calls address_slave() to address device to read
				Reads bytes.
	INPUT		Address:    specifies where to read from in the device
				ByteCount: 	number of bytes to read
				Data:       Pointer to return-data (byte) array
	OUTPUT		0 if read completes
				others if failed
	MODIFIES	Nothing
				 
----------------------------------------------------------------------------*/
static
int read_mem_I2C( u16 Address, u8 ByteCount, u8 *pData )
{ 
	int ret;
	u8 data[2];
	
	if( g_client == NULL )	/* Not initialize the client */	
		return -1;

	data[0] = Address & 0xff;
	data[1] = Address >> 8;
	ret = i2c_smbus_write_byte_data(g_client,data[0],data[1]);
	if(ret) {
		TS_DEBUG_TS(KERN_ERR "ATMEL: read reg [%04x], write address failed %d\n", Address, ret);
		return ret;
	}
	i2c_master_recv(g_client, pData, ByteCount);

	return 0; 
}

static
int write_mem( u16 Address, u8 ByteCount, u8 *pData )
{
	int ret;
	down(&atmel_i2c_lock);
	ret = write_mem_I2C(Address,ByteCount,pData);
	up(&atmel_i2c_lock);
	return ret;
}

static
int read_mem( u16 Address, u8 ByteCount, u8 *pData )
{
	int ret;
	down(&atmel_i2c_lock);
	ret = read_mem_I2C(Address,ByteCount,pData);
	up(&atmel_i2c_lock);
	return ret;
}

/*!
 * \brief Reads the id part of info block.
 * 
 * Reads the id part of the info block (7 bytes) from touch IC to 
 * info_block struct.
 *
 */
static  
int read_id_block(struct info_id_t *id)
{
    read_mem(0, 1, (u8 *) &id->family_id);
    read_mem(1, 1, (u8 *) &id->variant_id);
    read_mem(2, 1, (u8 *) &id->version);
    read_mem(3, 1, (u8 *) &id->build);
    read_mem(4, 1, (u8 *) &id->matrix_x_size);
    read_mem(5, 1, (u8 *) &id->matrix_y_size);
    read_mem(6, 1, (u8 *) &id->num_declared_objects);
                                
    return 0;   
}

/*!
 * \brief Reads the object table element part of info block.
 * 
 * Reads the object table element part of the info block from touch IC to 
 * info_block struct.
 *
 */
static  
int read_object_table_element(u32 address, struct object_t *object_table_element)
{
    read_mem(address + 0, 1, (u8 *) &object_table_element->object_type);
    read_mem(address + 1, 2, (u8 *) &object_table_element->object_address);
    read_mem(address + 3, 1, (u8 *) &object_table_element->size);
    read_mem(address + 4, 1, (u8 *) &object_table_element->instances);
    read_mem(address + 5, 1, (u8 *) &object_table_element->num_report_ids);
                                
    return 0;   
}

/*!
 * \brief CRC calculation routine.
 * 
 * @param crc the current CRC sum.
 * @param byte1 1st byte of new data to add to CRC sum.
 * @param byte2 2nd byte of new data to add to CRC sum.
 * @return crc the new CRC sum.
 * 
 */

u32 CRC_24(u32 crc, u8 byte1, u8 byte2)
{
   static const u32 crcpoly = 0x80001B;
   u32 result;
   u16 data_word;
   
   data_word = (u16) ((u16) (byte2 << 8u) | byte1);
   result = ((crc << 1u) ^ (u32) data_word);
   
   if (result & 0x1000000)
   {
      result ^= crcpoly;
   }
   
   return(result);
}

/*!
 * \brief Calculates the CRC sum for the info block & object table area,
 * and checks it matches the stored CRC.
 * 
 * Global interrupts need to be on when this function is called
 * since it reads the info block & object table area from the touch chip.
 * 
 * @param *crc_pointer Pointer to memory location where 
 *        the calculated CRC sum for the info block & object 
 *        will be stored.
 * @return 0 if calculation succeed, ERROR otherwise .
 * 
 */

static int calculate_infoblock_crc(u32 *crc_pointer)
{

	u32 crc = 0;
	u32 crc_area_size;
	u8 *mem;   
	u32 i;


	/* 7 bytes of version data, 6 * NUM_OF_OBJECTS bytes of object table. */
	crc_area_size = OBJECT_TABLE_START_ADDRESS + info_block.id.num_declared_objects * OBJECT_TABLE_ELEMENT_SIZE;

	mem = (u8 *) kzalloc(crc_area_size, GFP_KERNEL);
	if (mem == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "calculate_infoblock_crc kzalloc failed\n");
		return 1;
	}

	read_mem(0, crc_area_size, mem);

	i = 0;
	while (i < (crc_area_size - 1))
	{
		crc = CRC_24(crc, *(mem + i), *(mem + i + 1));
		i += 2;
	}

	if (crc_area_size % 2 == 1)
	{
		crc = CRC_24(crc, *(mem + i), 0);
	}
	kfree(mem);

	/* Return only 24 bit CRC. */
	*crc_pointer = (crc & 0x00FFFFFF);
	return 0;
}

/*!
 * \brief Returns the start address of the selected object.
 * 
 * Returns the start address of the selected object and instance 
 * number in the touch chip memory map.  
 *
 * @param object_type the object ID number.
 * @param instance the instance number of the object.
 * 
 * @return object address, or OBJECT_NOT_FOUND if object/instance not found.
 * 
 */

u16 get_object_address(u8 object_type, u8 instance)
{
   u8 object_table_index = 0;
   u8 address_found = 0;
   u16 address = OBJECT_NOT_FOUND;
   
   struct object_t *object_table;
   struct object_t obj;
   object_table = info_block.pobject_table;
   while ((object_table_index < info_block.id.num_declared_objects) &&
          !address_found)
   {
      obj = object_table[object_table_index];
      /* Does object type match? */
      if (obj.object_type == object_type)
      {
         address_found = 1;

         /* Are there enough instances defined in the FW? */
         if (obj.instances >= instance)
         {
            address = obj.object_address + (obj.size + 1) * instance;
         }
      }
      object_table_index++;
   }

   return(address);
}

/*!
 * \brief Reads message from the message processor.
 * 
 * This function reads a message from the message processor and calls
 * the message handler function provided by application earlier.
 *
 * @return MESSAGE_READ_OK if driver setup correctly and message can be read 
 * without problems, or MESSAGE_READ_FAILED if setup not done or incorrect, 
 * we are both polling the CHANGE line and monitoring it with interrupt that
 * or if there's a previous message read still ongoing (this can happen if
 * is triggered by falling edge).
 * 
 */

static int get_message(void)
{
	read_mem(message_processor_address, max_message_length, touch_msg);
	return 0;
}


/*!
 * \brief Returns the size of the selected object in the touch chip memory map.
 * 
 * Returns the size of the selected object in the touch chip memory map.
 *
 * @param object_type the object ID number.
 * 
 * @return object size, or OBJECT_NOT_FOUND if object not found.
 * 
 */

static u8 get_object_size(u8 object_type)
{
   u8 object_table_index = 0;
   u8 object_found = 0;
   u16 size = OBJECT_NOT_FOUND;
   
   struct object_t *object_table;
   struct object_t obj;
   object_table = info_block.pobject_table;
   while ((object_table_index < info_block.id.num_declared_objects) &&
          !object_found)
   {
      obj = object_table[object_table_index];
      /* Does object type match? */
      if (obj.object_type == object_type)
      {
         object_found = 1;
         size = obj.size + 1;
      }
      object_table_index++;
   }

   return(size);
}

/*!
 * \brief Writes power config. 
 * 
 * @param cfg power config struct.
 * 
 * @return 0 if successful.
 * 
 */

int write_power_config(int on)
{
	u16 object_address;
	u8 *tmp = NULL;
	u8 object_size;
   
	object_address = get_object_address(GEN_POWERCONFIG_T7, T7_INSTANCE);
	object_size = get_object_size(GEN_POWERCONFIG_T7);

	if (object_size < 3)
	{
		TS_DEBUG_TS(KERN_ERR "write_power_config: object_size: 0x%02x\n", object_size);
		return 1;
	}

	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "write_power_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "write_power_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);
    if(on)
    {
	    *(tmp + 0) = 50; //0xff//Idle Acquisition
	    *(tmp + 1) = 10; //0xff//Active Acquisition
	    *(tmp + 2) = 50; //0x32//Active to Idle Timeout
    }
    else
    {
        *(tmp + 0) = 0; //0xff//Idle Acquisition
        *(tmp + 1) = 0; //0xff//Active Acquisition
        *(tmp + 2) = 50; //0x32//Active to Idle Timeout            
    }
	write_mem(object_address, object_size, tmp);

	kfree(tmp);
	
	return 0;
}

/*!
 * \brief Writes power config. 
 * 
 * @param cfg power config struct.
 * 
 * @return 0 if successful.
 * 
 */

int write_acquisition_config(u8 instance,int flag)
{
	u16 object_address;
	u8 *tmp = NULL;
	u8 object_size;
   
	object_address = get_object_address(GEN_ACQUISITIONCONFIG_T8, instance);
	object_size = get_object_size(GEN_ACQUISITIONCONFIG_T8);

	if (object_size < 8)
	{
		TS_DEBUG_TS(KERN_ERR "write_acquisition_config: object_size: 0x%02x\n", object_size);
		return 1;
	}

	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "write_acquisition_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "write_acquisition_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);
    
	*(tmp + 0) = 8; //chrgtime
	*(tmp + 1) = 5; //Reserved
	*(tmp + 2) = 5; //tchdrift
	*(tmp + 3) = 10; //driftst
	*(tmp + 4) = 0; //tchautocal
	*(tmp + 5) = 0; //sync
	if(0 == flag)
    {
	
    	*(tmp + 6) = 5; //0x0a//ATCHCALST
    	*(tmp + 7) = 40; //0x0f//ATCHCALSTHR
    }
    else 
    {
        *(tmp + 6) = 0; //0x0a//ATCHCALST
    	*(tmp + 7) = 0; //0x0f//ATCHCALSTHR    
    }


	write_mem(object_address, object_size, tmp);

	kfree(tmp);
	
	return 0;
}

/*!
 * \brief read multitouchscreen config. 
 * 
 * 
 * This function will read the configuration of multitouchscreen
 * instance number.
 * 
 * @param instance the instance number of the multitouchscreen.
 * @param cfg multitouchscreen config struct.
 * @return 0 if successful.
 * 
 */

int read_multitouchscreen_config(u8 instance)
{
	u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;
	int i;

	object_size = get_object_size(TOUCH_MULTITOUCHSCREEN_T9);
	if (object_size < 30)
	{
		TS_DEBUG_TS(KERN_ERR "read_multitouchscreen_config: object_size: 0x%02x\n", object_size);
		return 1;
	}
	
	object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "read_multitouchscreen_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "read_multitouchscreen_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);

	read_mem(object_address, object_size, tmp);

	for (i = 0; i < object_size; i++)
	{
		TS_DEBUG_TS(KERN_ERR "T9[%d] = 0x%02x\n", i, *(tmp + i));		
	}

	kfree(tmp);

	return 0;
}

/*!
 * \brief Writes multitouchscreen config. 
 * 
 * 
 * This function will write the given configuration to given multitouchscreen
 * instance number.
 * 
 * @param instance the instance number of the multitouchscreen.
 * @param cfg multitouchscreen config struct.
 * @return 0 if successful.
 * 
 */

int write_multitouchscreen_config(u8 instance,int flag)
{
	u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;

	object_size = get_object_size(TOUCH_MULTITOUCHSCREEN_T9);
	if (object_size < 30)
	{
		TS_DEBUG_TS(KERN_ERR "write_multitouchscreen_config: object_size: 0x%02x\n", object_size);
		return 1;
	}
	
	object_address = get_object_address(TOUCH_MULTITOUCHSCREEN_T9, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "write_multitouchscreen_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "write_multitouchscreen_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);
	
	*(tmp + 0) = 139; //0x83//ctrl
	*(tmp + 1) = 0; //xorigin
	*(tmp + 2) = 0; //yorigin
	*(tmp + 3) = 17; //xsize
	*(tmp + 4) = 12; //ysize
	*(tmp + 5) = 0; //akscfg
	*(tmp + 6) = 17; //blen
    if(0 == flag)
    {
    	*(tmp + 7) = 50; //0x1d; //tchthr

    }
    else
    {
        *(tmp + 7) = 50; //0x1d; //tchthr

    }
	*(tmp + 8) = 2; //tchdi
	*(tmp + 9) = 1; //orientate
	*(tmp + 10) = 0; //mrgtimeout
	*(tmp + 11) = 3; //movhysti
	/* make the point report every pix */
	*(tmp + 12) = 1; //movhystn
	*(tmp + 13) = 0;//0x2e; //movfilter
	*(tmp + 14) = 2; //numtouch
	*(tmp + 15) = 10; //mrghyst
	*(tmp + 16) = 10; //mrgthr
	*(tmp + 17) = 10; //tchamphyst
	*(tmp + 18) = 0; //xres
	*(tmp + 19) = 0; //xres
	*(tmp + 20) = 0; //yres
	*(tmp + 21) = 0; //yres
	*(tmp + 22) = 35; //xloclip
	*(tmp + 23) = 33; //xhiclip
	*(tmp + 24) = 52; //yloclip
	*(tmp + 25) = 51; //yhiclip
	*(tmp + 26) = 148; //xedgectrl
	*(tmp + 27) = 10; //xedgedist
	*(tmp + 28) = 139; //yedgectrl
	*(tmp + 29) = 30; //yedgedist

	write_mem(object_address, object_size, tmp);

	kfree(tmp);

	return 0;
}

/*!
 * \brief Writes keyarray config. 
 * 
 * 
 * This function will write the given configuration to given keyarray
 * instance number.
 * 
 * @param instance the instance number of the keyarray.
 * @param cfg keyarray config struct.
 * @return 0 if successful.
 * 
 */

int write_keyarray_config(u8 instance)
{
	u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;

	object_size = get_object_size(TOUCH_KEYARRAY_T15);
	if (object_size < 11)
	{
		TS_DEBUG_TS(KERN_ERR "write_keyarray_config: object_size: 0x%02x\n", object_size);
		return 1;
	}
	
	object_address = get_object_address(TOUCH_KEYARRAY_T15, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "write_keyarray_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "write_keyarray_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);

	*(tmp + 0) = 0x00; //ctrl
	*(tmp + 1) = 0x03; //xorigin
	*(tmp + 2) = 0x0b; //yorigin
	*(tmp + 3) = 0x00; //xsize
	*(tmp + 4) = 0x00; //ysize
	*(tmp + 5) = 0; //akscfg
	*(tmp + 6) = 0x10; //blen
	*(tmp + 7) = 0x28; //tchthr
	*(tmp + 8) = 0x02; //tchdi
	*(tmp + 9) = 0; //reserved1
	*(tmp + 10) = 0; //reserved2

	write_mem(object_address, object_size, tmp);

	kfree(tmp);

	return 0;
}

/*!
 * \brief Writes gripfacesuppression config. 
 * 
 * 
 * This function will write the given configuration to given gripfacesuppression
 * instance number.
 * 
 * @param instance the instance number of the gripfacesuppression.
 * @param cfg gripfacesuppression config struct.
 * @return 0 if successful.
 * 
 */

int write_gripfacesuppression_config(u8 instance)
{
	u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;

	object_size = get_object_size(PROCG_GRIPFACESUPPRESSION_T20);
	if (object_size < 12)
	{
		TS_DEBUG_TS(KERN_ERR "write_gripfacesuppression_config: object_size: 0x%02x\n", object_size);
		return 1;
	}
	
	object_address = get_object_address(PROCG_GRIPFACESUPPRESSION_T20, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "write_gripfacesuppression_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "write_gripfacesuppression_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);

	/* turn off the fripfacesuppression */
	*(tmp + 0) = 0x00; //0x05; //ctrl
	*(tmp + 1) = 0; //xlogrip
	*(tmp + 2) = 0; //xhigrip
	*(tmp + 3) = 0; //ylogrip
	*(tmp + 4) = 0; //yhigrip
	*(tmp + 5) = 0; //maxtchs
	*(tmp + 6) = 0; //reserved
	*(tmp + 7) = 80; //szthr1
	*(tmp + 8) = 40; //szthr2
	*(tmp + 9) = 4; //shpthr1
	*(tmp + 10) = 35; //shpthr2
	*(tmp + 11) = 10; //supextto

	write_mem(object_address, object_size, tmp);

	kfree(tmp);

	return 0;
}

/*!
 * \brief Writes noisesuppression config. 
 * 
 * 
 * This function will write the given configuration to given noisesuppression
 * instance number.
 * 
 * @param instance the instance number of the noisesuppression.
 * @param cfg noisesuppression config struct.
 * @return 0 if successful.
 * 
 */

int write_noisesuppression_config(u8 instance)
{
	u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;

	object_size = get_object_size(PROCG_NOISESUPPRESSION_T22);
	if (object_size < 17)
	{
		TS_DEBUG_TS(KERN_ERR "write_noisesuppression_config: object_size: 0x%02x\n", object_size);
		return 1;
	}
	
	object_address = get_object_address(PROCG_NOISESUPPRESSION_T22, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "write_noisesuppression_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "write_noisesuppression_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);

	*(tmp + 0) = 0x05; //0x05; //ctrl
	*(tmp + 1) = 0; //reserved
	*(tmp + 2) = 0; //reserved
	*(tmp + 3) = 0x19; //GCAFUL
	*(tmp + 4) = 0x00; //GCAFUL
	*(tmp + 5) = 0xe7; //GCAFLL
	*(tmp + 6) = 0xff; //GCAFLL
	*(tmp + 7) = 4; //actvgcafvalid
	*(tmp + 8) = 20; //noisethr
	*(tmp + 9) = 0; //reserved
	*(tmp + 10) = 0; //freqhopscale
	*(tmp + 11) = 4; //freq burst0
	*(tmp + 12) = 14; //freq burst1
	*(tmp + 13) = 26; //freq burst2
	*(tmp + 14) = 255; //freq burst3
	*(tmp + 15) = 255; //freq burst4
	*(tmp + 16) = 4; //idlegcafvalid
	write_mem(object_address, object_size, tmp);

	kfree(tmp);

	return 0;
}

/*!
 * \brief read onetouchgesture config. 
 * 
 * 
 * This function will read the configuration of onetouchgesture
 * instance number.
 * 
 * @param instance the instance number of the onetouchgesture.
 * @param cfg onetouchgesture config struct.
 * @return 0 if successful.
 * 
 */

int write_Proximity_Config_Init(u8 instance)
{

    u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;
	object_size = get_object_size(TOUCH_PROXIMITY_T23);
	if (object_size < 30)
	{
		TS_DEBUG_TS(KERN_ERR "write_proximity_config: object_size: 0x%02x\n", object_size);
		return 1;
	}
	
	object_address = get_object_address(TOUCH_PROXIMITY_T23, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "write_proximity_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "write_proximity_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);

	write_mem(object_address, object_size, tmp);

	kfree(tmp);
    
	return 0;
    
}

int write_One_Touch_Gesture_Config_Init(u8 instance)
{

    u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;
	object_size = get_object_size(PROCI_ONETOUCHGESTUREPROCESSOR_T24);
	if (object_size < 19)
	{
		TS_DEBUG_TS(KERN_ERR "PROCI_ONETOUCHGESTUREPROCESSOR_T24: object_size: 0x%02x\n", object_size);
		return 1;
	}
	
	object_address = get_object_address(PROCI_ONETOUCHGESTUREPROCESSOR_T24, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "PROCI_ONETOUCHGESTUREPROCESSOR_T24: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "PROCI_ONETOUCHGESTUREPROCESSOR_T24: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);

	write_mem(object_address, object_size, tmp);

	kfree(tmp);
    
	return 0;
    
}

/*!
 * \brief read sptselftest config. 
 * 
 * 
 * This function will read the configuration of sptselftest
 * instance number.
 * 
 * @param instance the instance number of the sptselftest.
 * @param cfg sptselftest config struct.
 * @return 0 if successful.
 * 
 */

int read_sptselftest_config(u8 instance)
{
	u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;
	int i;

	object_size = get_object_size(SPT_SELFTEST_T25);
	if (object_size < 14)
	{
		TS_DEBUG_TS(KERN_ERR "read_sptselftest_config: object_size: 0x%02x\n", object_size);
		return 1;
	}
		TS_DEBUG_TS(KERN_ERR "read_sptselftest_config: 1object_size: 0x%02x\n", object_size);
	
	object_address = get_object_address(SPT_SELFTEST_T25, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "read_sptselftest_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "read_sptselftest_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);

	read_mem(object_address, object_size, tmp);

	for (i = 0; i < object_size; i++)
	{
		TS_DEBUG_TS(KERN_ERR "T25[%d] = 0x%02x\n", i, *(tmp + i));		
	}

	kfree(tmp);

	return 0;
}

/*!
 * \brief read twotouchgesture config. 
 * 
 * 
 * This function will read the configuration of twotouchgesture
 * instance number.
 * 
 * @param instance the instance number of the twotouchgesture.
 * @param cfg twotouchgesture config struct.
 * @return 0 if successful.
 * 
 */

int read_twotouchgesture_config(u8 instance)
{
	u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;
	int i;

	object_size = get_object_size(PROCI_TWOTOUCHGESTUREPROCESSOR_T27);
	if (object_size < 7)
	{
		TS_DEBUG_TS(KERN_ERR "read_twotouchgesture_config: object_size: 0x%02x\n", object_size);
		return 1;
	}
		TS_DEBUG_TS(KERN_ERR "read_twotouchgesture_config: 1object_size: 0x%02x\n", object_size);
	
	object_address = get_object_address(PROCI_TWOTOUCHGESTUREPROCESSOR_T27, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "read_twotouchgesture_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "read_twotouchgesture_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);

	read_mem(object_address, object_size, tmp);

	for (i = 0; i < object_size; i++)
	{
		TS_DEBUG_TS(KERN_ERR "T27[%d] = 0x%02x\n", i, *(tmp + i));		
	}

	kfree(tmp);

	return 0;
}

/*!
 * \brief Writes CTE config. 
 * 
 * 
 * This function will write the given configuration to given CTE
 * instance number.
 * 
 * @param instance the instance number of the CTE.
 * @param cfg CTE config struct.
 * @return 0 if successful.
 * 
 */

int write_CTE_config(u8 instance)
{
	u16 object_address = 0;
	u8 *tmp = NULL;
	u8 object_size;

	object_size = get_object_size(SPT_CTECONFIG_T28);
	if (object_size < 6)
	{
		TS_DEBUG_TS(KERN_ERR "write_CTE_config: object_size: 0x%02x\n", object_size);
		return 1;
	}
	
	object_address = get_object_address(SPT_CTECONFIG_T28, instance);
	if (object_address == 0)
	{
		TS_DEBUG_TS(KERN_ERR "write_CTE_config: object_address is zero\n");
		return 1;
	}

	tmp = (u8 *) kzalloc(object_size, GFP_KERNEL);
	if (tmp == NULL)
	{
		TS_DEBUG_TS(KERN_ERR "write_CTE_config: kzalloc for tmp failed\n");
		return 1;
	}
	memset(tmp, 0, object_size);
	*(tmp + 0) = 0; //ctrl
	*(tmp + 1) = 0; //cmd
	*(tmp + 2) = 1; //mode
	*(tmp + 3) = 16; //idlegcafdepth
	*(tmp + 4) = 32; //actvgcafdepth
	*(tmp + 5) = 10; //voltage

	write_mem(object_address, object_size, tmp);

	kfree(tmp);

	return 0;
}

/*!
 * \brief Resets the chip.
 * 
 *  This function will send a reset command to touch chip.
 *
 * @return WRITE_MEM_OK if writing the command to touch chip was successful.
 * 
 */

int reset_chip(void)
{
   u8 data = 1u;
   return(write_mem(command_processor_address + RESET_OFFSET, 1, &data));
}


/*!
 * \brief Calibrates the chip.
 * 
 * This function will send a calibrate command to touch chip.
 * 
 * @return WRITE_MEM_OK if writing the command to touch chip was successful.
 * 
 */

/* delete some lines which is not used anymore */
   
/*!
 * \brief Backups config area.
 * 
 * This function will send a command to backup the configuration to
 * non-volatile memory.
 * 
 * @return WRITE_MEM_OK if writing the command to touch chip was successful.
 * 
 */

int backup_config(void)
{
   
   /* Write 0x55 to BACKUPNV register to initiate the backup. */
   u8 data = 0x55u;
   return(write_mem(command_processor_address + BACKUP_OFFSET, 1, &data));
}

/*!
 * \brief debug config area.
 * 
 * 
 * @return WRITE_MEM_OK if writing the command to touch chip was successful.
 * 
 */

int debug_config(u8 data)
{
   return(write_mem(command_processor_address + DEBUG_CTRL_OFFSET, 1, &data));
}

/*!
 * \brief Return report id of given object/instance.
 * 
 *  This function will return a report id corresponding to given object type
 *  and instance, or 
 * 
 * @param object_type the object type identifier.
 * @param instance the instance of object.
 * 
 * @return report id, or 255 if the given object type does not have report id,
 * of if the firmware does not support given object type / instance.
 * 
 */

u8 type_to_report_id(u8 object_type, u8 instance)
{
	u8 report_id = 1;
	u8 report_id_found = 0;

	while((report_id <= max_report_id) && (report_id_found == 0))
	{
		if((report_id_map[report_id].object_type == object_type) &&
			(report_id_map[report_id].instance == instance))
		{
			report_id_found = 1;
		}
		else
		{
			report_id++;	
		}
	}
	if (report_id_found)
	{
		return(report_id);
	}
	else
	{
		return 0xff;
	}
}

/*!
 * \brief Maps report id to object type and instance.
 * 
 *  This function will return an object type id and instance corresponding
 *  to given report id.
 * 
 * @param report_id the report id.
 * @param *instance pointer to instance variable. This function will set this
 *        to instance number corresponding to given report id.
 * @return the object type id, or 255 if the given report id is not used
 * at all.
 * 
 */

u8 report_id_to_type(u8 report_id, u8 *instance)
{
	if (report_id <= max_report_id)
	{
		*instance = report_id_map[report_id].instance;
		return(report_id_map[report_id].object_type);
	}
	else
	{
		return 0xff;
	}   
}

/*! 
 * \brief Return the maximum report id in use in the touch chip.
 * 
 * @return maximum_report_id 
 * 
 */

uint8_t get_max_report_id(void)
{
	return max_report_id;
}

static void touch_input_report_key(struct atmel_ts_data *ts, unsigned int code, int value)
{
	input_report_key(ts->input_dev, code, value);
}

/* This function sends a calibrate command to the maXTouch chip.
 *  * While calibration has not been confirmed as good, this function sets
 *  * the ATCHCALST and ATCHCALSTHR to zero to allow a bad cal to always recover
 *  * Returns WRITE_MEM_OK if writing the command to touch chip was successful.
 *  */
uint8_t calibrate_chip_error(void)
{
	uint8_t data = 1u;
	int ret;
	/* resume calibration must be performed with zero settings */
    /* write the acquisition config to 0*/
	ret = write_acquisition_config(0,1);
	TS_DEBUG_TS("write_acquisition_config's ret is %d!\n",ret);
	/* Restore settings to the local structure so that when we confirm the
	 *      * cal is good we can correct them in the chip.
	 *      * This must be done before returning. */
	/* send calibration command to the chip */
    /* Write temporary acquisition config to chip. */	
	if(0 == ret)
	{
		/* Change calibration suspend settings to zero until calibration confirmed good. */
		ret = write_mem(command_processor_address + CALIBRATE_OFFSET, 1, &data);
        TS_DEBUG_TS("calibrate_chip_error the ret is %d !\n",ret);
		/* Set flag for calibration lock-up recovery if cal command was successful. */
		if(0 == ret)
		{
			/* Set flag to show we must still confirm if calibration was good or bad. */
			cal_check_flag = 1u;
		}
        else
        {
            TS_DEBUG_TS("the write_mem is failed!\n");
        }
	}
    else
    {
		/* "Acquisition config write failed!\n" */
		printk("%s the write the acquisition failed!\n",__func__);
    }
    TS_DEBUG_TS("the cal_check_flag is %u",cal_check_flag);
	msleep(20);
	return ret;
}

void check_chip_calibration(void)
{
	uint8_t data_buffer[100] = { 0 };
	uint8_t try_ctr = 0;
	uint8_t data_byte = 0xF3; /* dianostic command to get touch flags */
	uint16_t diag_address;
	uint8_t tch_ch = 0;
    uint8_t atch_ch = 0;
	uint8_t check_mask = 0;
	uint8_t i = 0;
	uint8_t j = 0;
	uint8_t x_line_limit = 0;
	/* We have had the first touchscreen or face suppression message
	 *     * after a calibration - check the sensor state and try to confirm if
	 *         * cal was good or bad */
	/* Get touch flags from the chip using the diagnostic object. */
	/* Write command to command processor to get touch flags.
	 *     * Command 0xF3 required to do this. */
	write_mem(command_processor_address + DEBUG_CTRL_OFFSET, 1, &data_byte);
	TS_DEBUG_TS("%d the write_mem is ok !\n",__LINE__);
	/* Get the address of the diagnostic object so we can get the data we need. */
	diag_address = get_object_address(DEBUG_DIAGNOSTIC_T37,0);
	TS_DEBUG_TS("%d the get_object_address is ok !\n",__LINE__);
	msleep(20);
	/* Read touch flags from the diagnostic object.
	 *     * Clear buffer so the while loop can run first time */
	memset( data_buffer, 0xFF, sizeof( data_buffer ) );
	TS_DEBUG_TS("%d the memset is ok !\n",__LINE__);
	/* wait for diagnostic object to update */
	while(!((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00)))
	{
		/* wait for data to be valid */
		if(try_ctr > 100)
		{
			/* Failed! Diagnostic Data did not update. */
			break;
		}
		/* Wait for diagnostic data to update */
		msleep(5);
		try_ctr++; /* timeout counter */
		read_mem(diag_address, 2, data_buffer);
        TS_DEBUG_TS("%d the read_mem is ok !\n",__LINE__);
	}
	/* Data is ready - read the detection flags */
	read_mem(diag_address, 82,data_buffer);
	/*   Data array for mXT224 is is 20 x 16 bits for each set of flags:
	 *     *  2 byte header
	 *     *  40 bytes for touch flags
	 *     *  40 bytes for antitouch flags*/
	/* Count up the channels/bits if we received the data properly */
	if((data_buffer[0] == 0xF3) && (data_buffer[1] == 0x00))
	{
		/* On mXT224 mode 0 = 16X lines, mode 1 = 17X, etc, up to mode 4.*/
		x_line_limit = 16 + 1;

        #if 0
        if(x_line_limit > 20)
		{
			/* Hard limit at 20 for mXT224 so we don't over-index the array */
			x_line_limit = 20;
		}
        #endif
		/* Double the limit, as the array is in bytes not words. */
		x_line_limit = x_line_limit << 1;
		/* Count the channels and print the flags to the log. */
		/* Check X lines - data is in words so increment 2 at a time */
		for(i = 0 ; i < x_line_limit; i+=2)
		{
			/* Count how many bits set for this row */
			for(j = 0 ; j < 8; j++)
			{
				/* Create a bit mask to check against */
				check_mask = 1 << j;
				/* Check detect flags */
				if(data_buffer[2+i] & check_mask)
				{
					tch_ch++;
				}
				if(data_buffer[3+i] & check_mask)
				{
					tch_ch++;
				}
				/* check anti-detect flags */
				if(data_buffer[42+i] & check_mask)
				{
					atch_ch++;
				}
				if(data_buffer[43+i] & check_mask)
				{
					atch_ch++;
				}
			}
		}

        TS_DEBUG_TS("%d the tch_ch is %d !\n",tch_ch,__LINE__);
        TS_DEBUG_TS("%d the atch_ch is %d !\n",atch_ch,__LINE__);
		/* Send page up command so we can detect when data updates next time.
		 *      * Page byte will sit at 1 until we next send F3 command. */
		data_byte = 0x01;
		write_mem(command_processor_address + DEBUG_CTRL_OFFSET, 1, &data_byte);
		TS_DEBUG_TS("the write_mem for command is ok!\n ");
		/* Process counters and decide if cal was good or if we must re-calibrate. */
		if((tch_ch) && (atch_ch == 0))
		{
			/* Calibration may be good */
			cal_maybe_good();
			TS_DEBUG_TS("the func cal_maybe_good is used!\n");
		}
		/* CAL_THR is configurable. A starting value of 10 to 20 is suggested.
		 *       * This can then be tuned for the particular design. */
		else if((tch_ch - 25) <= atch_ch && (tch_ch || atch_ch))
		{					
			/* Calibration was bad - must recalibrate and check afterwards. */
			calibrate_chip_error();
			/* Disable the timer */
			atmel_timer = DISABLE;
			timer_tick = 0;
			TS_DEBUG_TS("the timer is disabled!\n");
		}
		else
		{
			/* Calibration was not decided yet. We cannot confirm if good or bad.
			 *          * We must wait for next touch message to confirm. */
			cal_check_flag = 1u;
			/* Reset the 100 ms timer */
			atmel_timer = ENABLE;
			timer_tick = jiffies;
			TS_DEBUG_TS("the current time is %lu\n", jiffies);
		}
	}
    TS_DEBUG_TS("the cal_check_flag is %u!\n",cal_check_flag);
}
void cal_maybe_good(void)
{
    int ret;
	/* Check if the timer is enabled */
	if(atmel_timer == ENABLE)
	{
		TS_DEBUG_TS("cal_maybe_good: the current time is %lu\n", jiffies);
		if((jiffies - timer_tick) /10 > 5) /* Check if the timer timedout of 0.5seconds */
		{
			/* Cal was good - don't need to check any more */
			cal_check_flag = 0;
			/* Disable the timer */
			atmel_timer = DISABLE;
			timer_tick = 0;
			/* Write back the normal acquisition config to chip. */
			if (1 == write_acquisition_config(0,0))
			{
				/* "Acquisition config write failed!\n" */
                
				printk("\n[ERROR] line : %d\n", __LINE__);
			}
            
            ret = write_multitouchscreen_config(0,1);

            printk("the cal_maybe_good is ok! the ret is %d\n",ret);
		}
		else
		{
			cal_check_flag = 1u;
            TS_DEBUG_TS("the time is not yet!\n");
		}
	}
	else
	{
		/* Timer not enabled, so enable it */
		atmel_timer = ENABLE; // enable for 100ms timer
		timer_tick = jiffies;
		cal_check_flag = 1u;
        TS_DEBUG_TS("the cal_maybe_good is enable time!\n");
	}
}

static int atmel_ts_initchip(void)
{
	write_acquisition_config(0,0);
	write_power_config(1);
	write_multitouchscreen_config(0,0);
	write_noisesuppression_config(0);
	write_CTE_config(0);
    write_gripfacesuppression_config(0);//test 0906
    write_Proximity_Config_Init(0);
    write_One_Touch_Gesture_Config_Init(0);
	backup_config();
	reset_chip();
	msleep(50);
    TS_DEBUG_TS("the initchip is ok!\n");
	return 0;
}

static void atmel_ts_work_func(struct work_struct *work)
{
	u8 ins=0;
	u8 obj;
	u8 x_MSB=0;
	u8 y_MSB=0;
	u8 xy_LSB=0;
	u8 status=0;
	u8 component=0;
	u8 keys;
	static u32 key_pressed = 0;
	static bool first_point_pressed = FALSE;
	static bool second_point_pressed = FALSE;
    static bool last_is_2points = FALSE;//if it's 2 points pressed last time.
    static char first_point_id = 1; 
    static int point_1_x;
    static int point_1_y;
    static int point_1_amplitude;
    static int point_1_width;
    static int point_2_x;
    static int point_2_y;
    static int point_2_amplitude;
    static int point_2_width;
    static u32 key_tmp_old;

	u8 point_index = 1;
	struct atmel_ts_data *ts = container_of(work, struct atmel_ts_data, work);
	get_message();
	obj = report_id_to_type(*touch_msg, &ins);
	switch (obj)
	{
		case GEN_COMMANDPROCESSOR_T6:
			status = *(touch_msg + 1);
            /*if the calibration is done then make the pressed flag default*/
            if(status & 0x10)
            {
                first_point_pressed = FALSE;
	            second_point_pressed = FALSE;
            }
			TS_DEBUG_TS("T6 status = 0x%02x\n", status);
			break;
			
		case TOUCH_MULTITOUCHSCREEN_T9:
			point_index = *touch_msg - T9_base_reportID + 1;
          
            
			if (point_index > 2)
			{
				//only support 2 point now
				TS_DEBUG_TS("too many point\n");
				break;
			}
						
			status = *(touch_msg + 1);
			x_MSB = *(touch_msg + 2);
			y_MSB = *(touch_msg + 3);
			xy_LSB = *(touch_msg + 4);
			ts->sizeoftouch = *(touch_msg + 5);
			ts->touchamplitude = *(touch_msg + 6);
			component = *(touch_msg + 7);
            /* if the status is detected and the cal_check_flag is 1 then we should make the calibration */
            if(0 != (status & 0x80)&&(cal_check_flag))
            {
                check_chip_calibration();
            }
#ifdef TOUCH_12BIT
			ts->touch_x = (x_MSB << 4) + ((xy_LSB >> 4) & 0x0f);
			ts->touch_y = (y_MSB << 4) + (xy_LSB & 0x0f);
#else		
			ts->touch_x = (x_MSB << 2) + ((xy_LSB >> 6) & 0x03);
			ts->touch_y = (y_MSB << 2) + ((xy_LSB >> 2) & 0x03);
            
            ts->touch_x = (ts->touch_x * 480) / 1023;
            ts->touch_y = (ts->touch_y * 882) / 1023;
#endif
			TS_DEBUG_TS("version 3;point %d released : %s,x=%04d,  y=%04d\n", 
			point_index,((1 << 5) & status) ? "yes":"no", ts->touch_x, ts->touch_y);
            ATMEL_DBG_MASK("version 3;point %d released : %s,x=%04d,  y=%04d\n", 
		    point_index,((1 << 5) & status) ? "yes":"no", ts->touch_x, ts->touch_y);

            if(ts->is_support_multi_touch)
			{
                /*the 5-bit in STATUS register specifies the current point just released*/
				if((1 == point_index) && !((1 << 5) & status))
					first_point_pressed = TRUE;
				else if((1 == point_index) && ((1 << 5) & status))
					first_point_pressed = FALSE;

					if((2 == point_index) && !((1 << 5) & status))
						second_point_pressed = TRUE;
					else if((2 == point_index) && ((1 << 5) & status))
						second_point_pressed = FALSE;
				    /*when two points are pressed, multi_touch mode is triggered.*/
                    /*if pressed, need to save the current coordinates*/
                    if(!((1 << 5) & status))
                    {   
                        if(1 == point_index)
                        {
                            TS_DEBUG_TS("save point 1\t");
                            point_1_x = ts->touch_x;
                            point_1_y = ts->touch_y;
                            point_1_amplitude = ts->touchamplitude;
                            point_1_width = ts->sizeoftouch;
                        }
                        else
                        {
                            TS_DEBUG_TS("save point 2\t");
                            point_2_x = ts->touch_x;
                            point_2_y = ts->touch_y;
                            point_2_amplitude = ts->touchamplitude;
                            point_2_width = ts->sizeoftouch;
                        }
                    }
                    else
                    {
                        if(1 == point_index)
                        {
                            /*if index-1 released, index-2 point remains working*/
                            first_point_id = 2;
                        }
                        else
                        {
                            /*if index-2 released, index-1 point remains working*/
                            first_point_id =1;
                        }
                    }
                    /*if both two points are released, we need to reset first_point_id*/
                    if(!first_point_pressed && !second_point_pressed)
                    {
                        point_1_amplitude = 0;
                        point_1_width = 0;
                        point_2_amplitude = 0;
                        point_2_width = 0;
                        first_point_id =1;
                    }
                    /*to report the first point event*/
                    if(1 == first_point_id)
                    {
                        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, point_1_amplitude);
				        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, point_1_width);
				        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, point_1_x);
				        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, point_1_y);
				        input_mt_sync(ts->input_dev);
                    }
                    else
                    {
                        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, point_2_amplitude);
				        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, point_2_width);
				        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, point_2_x);
				        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, point_2_y);
				        input_mt_sync(ts->input_dev);
                    }
                    /*if there are two points pressed at present, we should report the second point event*/
                    if(first_point_pressed && second_point_pressed)
                    {
                        if(1 == first_point_id)
                        {
                            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, point_2_amplitude);
					        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, point_2_width);
					        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, point_2_x);
					        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, point_2_y);
					        input_mt_sync(ts->input_dev);
                        }
                        else
                        {
                            input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, point_1_amplitude);
					        input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, point_1_width);
					        input_report_abs(ts->input_dev, ABS_MT_POSITION_X, point_1_x);
					        input_report_abs(ts->input_dev, ABS_MT_POSITION_Y, point_1_y);
					        input_mt_sync(ts->input_dev);
                        }
                    }
                    else if(last_is_2points)//when one point released...
                    {
                        input_report_abs(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0);
        				input_report_abs(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0);
        				input_mt_sync(ts->input_dev);
                    }
                    input_sync(ts->input_dev);
                    if(first_point_pressed && second_point_pressed)
                        last_is_2points = TRUE;
                    else
                        last_is_2points = FALSE;
				}
				else
				{

					input_report_abs(ts->input_dev, ABS_X, ts->touch_x);
					input_report_abs(ts->input_dev, ABS_Y,  ts->touch_y);

					input_report_abs(ts->input_dev, ABS_PRESSURE, ts->touchamplitude);
					input_report_abs(ts->input_dev, ABS_TOOL_WIDTH, ts->sizeoftouch);
					if ((1 << 5) & status)//release bit
					{
    						input_report_key(ts->input_dev, BTN_TOUCH, 0);
					}
					else
					{
    						input_report_key(ts->input_dev, BTN_TOUCH, 1);
					}         
					input_sync(ts->input_dev);

				}
                /* move the key area code to here */
			break;
		case TOUCH_KEYARRAY_T15:
			status = *(touch_msg + 1);
			keys = *(touch_msg + 2);
			TS_DEBUG_TS("status = 0x%02x, keys = 0x%02x\n", status, keys);
			if (keys > 0)
				TS_DEBUG_TS("~");
			else
				TS_DEBUG_TS("^");

			switch (keys)
			{
				case KEY_RELEASE:
					if (key_pressed)
					{
					 	touch_input_report_key(ts, key_pressed, 0);
						input_sync(ts->input_dev);
						key_pressed = 0;
					}
					break;
				case KEY_NUHBER1:
					if (0 == key_pressed)
					{
						if (ts->test > 0) 
							key_pressed = KEY_BRL_DOT1;
						else
							key_pressed = KEY_SEARCH;							
					 	touch_input_report_key(ts, key_pressed, 1);
						input_sync(ts->input_dev);
					}
					break;
				case KEY_NUHBER2:
					if (0 == key_pressed)
					{
						if (ts->test > 0) 
							key_pressed = KEY_BRL_DOT2;
						else
							key_pressed = KEY_MENU;
					 	touch_input_report_key(ts, key_pressed, 1);
						input_sync(ts->input_dev);
					}
					break;
				case KEY_NUHBER3:
					if (0 == key_pressed)
					{
						if (ts->test > 0) 
							key_pressed = KEY_BRL_DOT3;
						else
							key_pressed = KEY_HOME;
					 	touch_input_report_key(ts, key_pressed, 1);
						input_sync(ts->input_dev);
					}
					break;
				case KEY_NUHBER4:
					if (0 == key_pressed)
					{
						if (ts->test > 0) 
							key_pressed = KEY_BRL_DOT5;
						else
							key_pressed = KEY_BACK;
					 	touch_input_report_key(ts, key_pressed, 1);
						input_sync(ts->input_dev);
					}
					break;
				default:
					break;
			}
				
				
			break;
        case PROCG_GRIPFACESUPPRESSION_T20:         
            status = *(touch_msg + 1);
            if(0 != (status & 0x01)&&(cal_check_flag))
            {
                check_chip_calibration();
            }

            break;
            
		default:
			TS_DEBUG_TS("T%d detect\n", obj);
			break;
	}
	
	if (ts->use_irq) {
		enable_irq(ts->client->irq);
	}
	return;
}
static enum hrtimer_restart atmel_ts_timer_func(struct hrtimer *timer)
{
	struct atmel_ts_data *ts = container_of(timer, struct atmel_ts_data, timer);
	//TS_DEBUG_TS(" atmel_ts_timer_func\n");
	queue_work(atmel_wq, &ts->work);
	hrtimer_start(&ts->timer, ktime_set(0, 200000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static irqreturn_t atmel_ts_irq_handler(int irq, void *dev_id)
{
	struct atmel_ts_data *ts = dev_id;
	TS_DEBUG_TS(KERN_ERR "atmel_ts_irq_handler,irq occured  disable irq\n");

/* In the kernel32 disable_irq is not safe so we use disable_irq_nosync to avoid the irq occured in the probe*/
    disable_irq_nosync(ts->client->irq);
    queue_work(atmel_wq, &ts->work);
	return IRQ_HANDLED;
}

static int atmel_ts_probe(
	struct i2c_client *client,const struct i2c_device_id *id)
{
       
	struct atmel_ts_data *ts;
	int ret = 0;
	int i;
    struct vreg *v_gp4 = NULL;
    int gpio_config;
	struct object_t *object_table = NULL;
	u32 current_address; //the address is normal erveryone can use it
	u32 crc_calculated = 0; //crc check
	u8 tmp[3] = {0};
	int current_report_id = 0;
    struct touch_hw_platform_data *touch_pdata;
  
	TS_DEBUG_TS(KERN_ERR "atmel_ts_probe in");
    ts = NULL;

    /*when the probe is come in we first detect the probe for touch is ready*/
    touch_pdata = client->dev.platform_data;

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

	g_client = client;

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
    
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) 
    {
		printk(KERN_ERR "%s: need I2C_FUNC_I2C\n", __func__);
		ret = -ENODEV;
		goto err_check_functionality_failed;
	}
    TS_DEBUG_TS("the i2c_check_functionality is ok \n");
    
	/* Read the info block data. */
	info_block.id.family_id = 0;
	ret = read_id_block(&info_block.id);
    if(ret < 0)
    {
        printk(KERN_ERR "the slave is not decteted\n");
        goto err_check_functionality_failed;
    }
	for (i = 0; i < 7; i++)
	{
		TS_DEBUG_TS(KERN_ERR "ids[%d] = 0x%02x\n", i, *(((u8 *)&info_block.id) + i));
	}
	if (ATMEL_FAMILY_ID != info_block.id.family_id)
	{
		ret = -1;
		TS_DEBUG_TS(KERN_ERR "atmel_ts_probe: failed to find ATMEL IC\n");
		goto err_probe_IC_fail;
	}
	
	/* Read object table. */
	object_table = (struct object_t *) kzalloc(info_block.id.num_declared_objects * sizeof(struct object_t), GFP_KERNEL);
	if (object_table == NULL)
	{
		ret = -ENOMEM;
		TS_DEBUG_TS(KERN_ERR "atmel_ts_probe: kzalloc for object_table failed\n");
		goto err_alloc_data_failed1;
	}
	
	info_block.pobject_table = object_table;

	/* Reading the whole object table block to memory directly doesn't work cause sizeof object_t
	isn't necessarily the same on every compiler/platform due to alignment issues. Endianness
	can also cause trouble. */
	current_address = OBJECT_TABLE_START_ADDRESS;
	max_report_id = 0;
	for (i = 0; i < info_block.id.num_declared_objects; i++)
	{
		read_object_table_element(current_address + i * OBJECT_TABLE_ELEMENT_SIZE, object_table + i);
		max_report_id += (object_table + i)->num_report_ids;
      
	      /* Find out the maximum message length. */
		if ((object_table + i)->object_type == GEN_MESSAGEPROCESSOR_T5)
		{
			max_message_length = (object_table + i)->size + 1;
			TS_DEBUG_TS(KERN_ERR "max_message_length = 0x%02x\n", max_message_length);
		}
	}
		
	read_mem(OBJECT_TABLE_START_ADDRESS + info_block.id.num_declared_objects * OBJECT_TABLE_ELEMENT_SIZE, 3, tmp);
	info_block.CRC = tmp[0] | (tmp[1] << 8) | (tmp[2] << 16);
	calculate_infoblock_crc(&crc_calculated);
	TS_DEBUG_TS(KERN_ERR "info_block->CRC = 0x%08x\n", info_block.CRC);
	TS_DEBUG_TS(KERN_ERR "crc_calculated = 0x%08x\n", crc_calculated);

	if (info_block.CRC != crc_calculated)
	{	
		TS_DEBUG_TS(KERN_ERR "atmel_ts_probe: CRC not match, need reflash ROM of touch IC\n");
		goto err_power_failed;
	}
	
	/* Allocate memory for report id map now that the number of report id's 
	* is known. */
	report_id_map = (struct report_id_map_t *)kzalloc(sizeof(struct report_id_map_t) * max_report_id, GFP_KERNEL);
	if (report_id_map == NULL)
	{
		ret = -ENOMEM;
		TS_DEBUG_TS(KERN_ERR "atmel_ts_probe: kzalloc for report_id_map failed\n");
		goto err_alloc_data_failed2;
	}

	/* Report ID 0 is reserved, so start from 1. */

	report_id_map[0].instance = 0;
	report_id_map[0].object_type = 0;
	current_report_id = 1;

	for (i = 0; i < info_block.id.num_declared_objects; i++)
	{
		if ((object_table + i)->num_report_ids != 0)
		{
			int instance;
			for (instance = 0; 
				instance <= (object_table + i)->instances; 
				instance++)
			{
				int start_report_id = current_report_id;
				for (; 
					current_report_id < 
					(start_report_id + (object_table + i)->num_report_ids);
					current_report_id++)
				{
					report_id_map[current_report_id].instance = instance;
					report_id_map[current_report_id].object_type = 
					(object_table + i)->object_type;
				}
			}
		}
	}

	/* Store message processor address, it is needed often on message reads. */
	message_processor_address = get_object_address(GEN_MESSAGEPROCESSOR_T5, 0);

	/* Store command processor address. */
	command_processor_address = get_object_address(GEN_COMMANDPROCESSOR_T6, 0);
	
	/* Store command processor address. */
	debug_diagnostic_address = get_object_address(DEBUG_DIAGNOSTIC_T37, 0);
		
	/* msg. */
	touch_msg = (u8 *) kzalloc(max_message_length, GFP_KERNEL);
	if (touch_msg == NULL)
	{
		ret = -ENOMEM;
		TS_DEBUG_TS(KERN_ERR "atmel_ts_probe: kzalloc for touch_msg failed\n");
		goto err_alloc_data_failed3;
	}
	
	T9_base_reportID = type_to_report_id(TOUCH_MULTITOUCHSCREEN_T9, 0);
    msleep(10);
	
	atmel_ts_initchip();
    
	msleep(10);

    calibrate_chip_error();
      
	atmel_wq = create_singlethread_workqueue("atmel_wq");
	if (!atmel_wq) {
		//return -ENOMEM;
        goto error_wq_creat_failed;
	}
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (ts == NULL) {
		ret = -ENOMEM;
		goto err_alloc_data_failed4;
	}

	INIT_WORK(&ts->work, atmel_ts_work_func);
	ts->is_support_multi_touch = client->flags;
    ts->client = client;
	i2c_set_clientdata(client, ts);

    /*
	ts->power = atmel_ts_power;
	if (ts->power) {
		ret = ts->power(ts->client, 1);
		if (ret < 0) {
			goto err_power_failed;
		}
		msleep(50);
	}
    */

    #ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_TOUCH_PANEL);
    #endif

    // sjm debug
    if(touch_pdata->set_touch_probe_flag)
    {
        touch_pdata->set_touch_probe_flag(0);
    }

	ts->input_dev = input_allocate_device();
	if (ts->input_dev == NULL) {
		ret = -ENOMEM;
		goto err_input_dev_alloc_failed;
	}
	ts->input_dev->name = "atmel-rmi-touchscreen";
	dev_set_drvdata(&(ts->input_dev->dev), ts);
	
	set_bit(EV_SYN, ts->input_dev->evbit);
	set_bit(EV_KEY, ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ts->input_dev->keybit);
	set_bit(EV_ABS, ts->input_dev->evbit);
	set_bit(ABS_X, ts->input_dev->absbit);
	set_bit(ABS_Y, ts->input_dev->absbit);
    set_bit(KEY_NUMLOCK, ts->input_dev->keybit);

	input_set_abs_params(ts->input_dev, ABS_X, 0, 480, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(ts->input_dev, ABS_TOOL_WIDTH, 0, 15, 0, 0);
	if(ts->is_support_multi_touch)
	{
		input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, 0, 480, 0, 0);

		input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, 0, 800, 0, 0);

		input_set_abs_params(ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
        /* changge the report limit to make the touch receive phone well */
		input_set_abs_params(ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 30, 0, 0);
	}
	ret = input_register_device(ts->input_dev);
	if (ret) {
		goto err_input_register_device_failed;
	}
	
    if(touch_pdata->touch_gpio_config_interrupt)
    {
        ret = touch_pdata->touch_gpio_config_interrupt();
    }
    if (ret) 
	{
		ret = -EIO;
		goto err_input_dev_alloc_failed;
	}

	ts->test = 0;
	if(device_create_file(&(ts->input_dev->dev), &dev_attr_test)) {
		TS_DEBUG_TS("Failed to add touch key test attrs\n");
	}

	ts->lock = 0;
	if(device_create_file(&(ts->input_dev->dev), &dev_attr_lock)) {
		TS_DEBUG_TS("Failed to add touch lock attrs\n");
	}
	
	TS_DEBUG_TS(KERN_ERR "atmel_ts_probe: client->irq = 0x%x\n", client->irq);

	if (client->irq) {
        gpio_request(irq_to_gpio(client->irq), client->name);
        printk("the gpio is %d!\n", irq_to_gpio(client->irq));
		gpio_direction_input(irq_to_gpio(client->irq));

		ret = request_irq(client->irq, atmel_ts_irq_handler, IRQF_TRIGGER_LOW, client->name, ts);
		if (ret) {
			TS_DEBUG_TS(KERN_ERR "atmel_ts_probe: request irq failed, use polling way\n");
			ts->use_irq = 0;
            TS_DEBUG_TS(KERN_ERR "atmel_ts_probe: request irq is %d,and the ret is %d\n",client->irq,ret);  
		}
		else {
			ts->use_irq = 1;
            TS_DEBUG_TS(KERN_ERR "atmel_ts_probe:request irq the ret is %d\n",ret);
		}
	}
	
	if (!ts->use_irq) {
		hrtimer_init(&ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ts->timer.function = atmel_ts_timer_func;
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		TS_DEBUG_TS(" cyj set timer\n");
	}

	//important, read msg to clear interrupt
	get_message();
    
#ifdef CONFIG_HAS_EARLYSUSPEND
	ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ts->early_suspend.suspend = atmel_ts_early_suspend;
	ts->early_suspend.resume = atmel_ts_late_resume;
	register_early_suspend(&ts->early_suspend);
#endif

    /* Clear touch screen interrupt status register before irq_handler ready, let tp release irq pin */
    if(ts->use_irq)
    {
		for( i = 5; i >0 ; i--)
		{
		    if(i2c_smbus_read_byte_data(client, 0x14) >= 0)
			break;
		}
    }

	return 0;

err_input_register_device_failed:
	input_free_device(ts->input_dev);

err_input_dev_alloc_failed:
err_power_failed:
	kfree(ts);
err_alloc_data_failed4:
error_wq_creat_failed:    
	kfree(touch_msg);
err_alloc_data_failed3:
	kfree(report_id_map);
err_alloc_data_failed2:
	kfree(info_block.pobject_table);
err_alloc_data_failed1:
err_probe_IC_fail:
err_check_functionality_failed:

    ret = touch_pdata->touch_power(0);
    {
        printk(KERN_ERR "the power is off: gp5 = %d \n ", ret);   
    }

err_platform_data_init_failed:
   
err_power_on_failed:
    printk(KERN_ERR "the atmel's power init is failed!\n");
	
	return ret;
}

static int atmel_ts_power(struct i2c_client *client, int on)
{
	return 0;
}

static int atmel_ts_remove(struct i2c_client *client)
{
	struct atmel_ts_data *ts = i2c_get_clientdata(client);

	if (ts->use_irq)
		free_irq(client->irq, ts);
	else
		hrtimer_cancel(&ts->timer);
	device_remove_file(&ts->input_dev->dev, &dev_attr_lock);
	device_remove_file(&ts->input_dev->dev, &dev_attr_test);
	input_unregister_device(ts->input_dev);
		
	kfree(ts);
	g_client = NULL;
	kfree(touch_msg);
	kfree(report_id_map);
	kfree(info_block.pobject_table);
	return 0;
}

static int atmel_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret;
	struct atmel_ts_data *ts = i2c_get_clientdata(client);
	
	if (ts->use_irq)
		disable_irq_nosync(client->irq);
	else
		hrtimer_cancel(&ts->timer);
	ret = cancel_work_sync(&ts->work);
	if (ret && ts->use_irq) /* if work was pending disable-count is now 2 */
		enable_irq(client->irq);
    write_power_config(0);
	return 0;
}

static int atmel_ts_resume(struct i2c_client *client)
{
	struct atmel_ts_data *ts = i2c_get_clientdata(client);
    write_power_config(1);
	calibrate_chip_error();
	
	if (ts->use_irq) {
		enable_irq(client->irq);
	}

	else
		hrtimer_start(&ts->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void atmel_ts_early_suspend(struct early_suspend *h)
{
	struct atmel_ts_data *ts;
	ts = container_of(h, struct atmel_ts_data, early_suspend);
	atmel_ts_suspend(ts->client, PMSG_SUSPEND);
}

static void atmel_ts_late_resume(struct early_suspend *h)
{
	struct atmel_ts_data *ts;
	ts = container_of(h, struct atmel_ts_data, early_suspend);
	atmel_ts_resume(ts->client);
}
#endif

static const struct i2c_device_id atmel_ts_id[] = {
	{ ATMEL_I2C_RMI_NAME, 0 },
	{ }
};

static struct i2c_driver atmel_ts_driver = {
	.probe		= atmel_ts_probe,
	.remove		= atmel_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= atmel_ts_suspend,
	.resume		= atmel_ts_resume,
#endif
	.id_table	= atmel_ts_id,
	.driver = {
	.name	= ATMEL_I2C_RMI_NAME,
	},
};
 
static int __devinit atmel_ts_init(void)
{
       TS_DEBUG_TS(KERN_ERR "atmel_ts init\n");
	return i2c_add_driver(&atmel_ts_driver);
}

static void __exit atmel_ts_exit(void)
{
	i2c_del_driver(&atmel_ts_driver);
	if (atmel_wq)
		destroy_workqueue(atmel_wq);
}

module_init(atmel_ts_init);
module_exit(atmel_ts_exit);

MODULE_DESCRIPTION("ATMEL Touchscreen Driver");
MODULE_LICENSE("GPL");
