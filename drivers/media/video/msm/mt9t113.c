/* Edit format */
#include <linux/delay.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <media/msm_camera.h>
#include <mach/gpio.h>
#include <mach/camera.h>
#include "mt9t113.h"

#include "linux/hardware_self_adapt.h"

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
 #include <linux/hw_dev_dec.h>
#endif

#undef CDBG
#define CDBG(fmt, args...) printk(KERN_INFO "mt9t113.c: " fmt, ## args)

#define MT9T113_DEFAULT_CLOCK_RATE 24000000

#define MT9T113_CHIP_ID 0x4680
#define MT9T113_REG_CHIP_ID 0x0000
#define MT9T113_REG_MODEL_ID 0x0604
#define MT9T113_ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#define MODEL_BYD 0x02
#define MODEL_SUNNY 0x00

/* Set flag that indicate if camera is initialization */

const static char mt9t113_supported_effect[] = "none,mono,negative,sepia,aqua";
static uint16_t mt9t113_model_id = MODEL_BYD;
static struct  mt9t113_work_t *mt9t113sensorw = NULL;
static struct  i2c_client *mt9t113_client  = NULL;
static struct mt9t113_ctrl_t *mt9t113_ctrl = NULL;
static enum mt9t113_reg_update_t last_rupdate = -1;
static enum mt9t113_setting_t last_rt = -1;
static uint8_t mt9t113_init_flag = false;

static DECLARE_WAIT_QUEUE_HEAD(mt9t113_wait_queue);
DECLARE_MUTEX(mt9t113_sem);


//================================================================================================
//	Run capture
//================================================================================================

const static struct mt9t113_i2c_reg_conf mt9t113_effect_off_reg_config[] =
{
    {0x098E, 0xE887}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0xEC87}, 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0
};
const static struct mt9t113_i2c_reg_conf mt9t113_effect_Mono_reg_config[] =
{
    {0x098E, 0xE887}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0001}, 	// MCU_DATA_0
    {0x098E, 0xEC87}, 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0001}, 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0
};
const static struct mt9t113_i2c_reg_conf mt9t113_effect_neg_reg_config[] =
{
    {0x098E, 0xE887}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0003}, 	// MCU_DATA_0
    {0x098E, 0xEC87}, 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0003}, 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0
};

const static struct mt9t113_i2c_reg_conf mt9t113_effect_sepia_reg_config[] =
{
    {0x098E, 0xE887}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0002}, 	// MCU_DATA_0
    {0x098E, 0xEC87}, 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0002}, 	// MCU_DATA_0
    {0x098E, 0xE889}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CR]
    {0x0990, 0x001E}, 	// MCU_DATA_0
    {0x098E, 0xE88A}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CB]
    {0x0990, 0x009C}, 	// MCU_DATA_0
    {0x098E, 0xEC89}, 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CR]
    {0x0990, 0x001E}, 	// MCU_DATA_0
    {0x098E, 0xEC8A}, 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CB]
    {0x0990, 0x009C}, 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0
};
const static struct mt9t113_i2c_reg_conf mt9t113_effect_aqua_reg_config[] =
{
    {0x098E, 0xE887}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0002}, 	// MCU_DATA_0
    {0x098E, 0xEC87}, 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SELECT_FX]
    {0x0990, 0x0002}, 	// MCU_DATA_0
    {0x098E, 0xE889}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CR]
    {0x0990, 0x00E2}, 	// MCU_DATA_0
    {0x098E, 0xE88A}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_SEPIA_CB]
    {0x0990, 0x0030}, 	// MCU_DATA_0
    {0x098E, 0xEC89}, 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CR]
    {0x0990, 0x00E2}, 	// MCU_DATA_0
    {0x098E, 0xEC8A}, 	// MCU_ADDRESS [PRI_B_CONFIG_SYSCTRL_SEPIA_CB]
    {0x0990, 0x0030}, 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0
};

const static struct mt9t113_i2c_reg_conf mt9t113_wb_auto_reg_config[] =
{
    {0x098E, 0x6848}, 	// MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
    {0x0990, 0x003F}, 	// MCU_DATA_0
    {0x098E, 0x6865}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
    {0x0990, 0x801F}, 	// MCU_DATA_0
    {0x098E, 0x6867}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
    {0x0990, 0x12F7}, 	// MCU_DATA_0
    {0x098E, 0x6881}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
    {0x0990, 0x000B},		//0x0002 	// MCU_DATA_0
    {0x098E, 0x6883}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
    {0x0990, 0x000B}, 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0
};

const static struct mt9t113_i2c_reg_conf mt9t113_wb_incabd_reg_config[] =
{
    {0x098E, 0x6848}, 	// MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6865}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6867}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6881}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
    {0x0990, 0x0009},		//0x0000 	// MCU_DATA_0
    {0x098E, 0x6883}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
    {0x0990, 0x0009},		//0x0000 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0

    {0x098E, 0xAC3B}, 	// MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
    {0x0990, 0x005F}, 	// MCU_DATA_0
    {0x098E, 0xAC3C}, 	// MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
    {0x0990, 0x002E}, 	// MCU_DATA_0
};

const static struct mt9t113_i2c_reg_conf mt9t113_wb_fluore_reg_config[] =
{
    {0x098E, 0x6848}, 	// MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6865}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6867}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6881}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
    {0x0990, 0x0009},		//0x0000 	// MCU_DATA_0
    {0x098E, 0x6883}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
    {0x0990, 0x0009},		//0x0000 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0

    {0x098E, 0xAC3B}, 	// MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
    {0x0990, 0x0048}, 	// MCU_DATA_0
    {0x098E, 0xAC3C}, 	// MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
    {0x0990, 0x0036}, 	// MCU_DATA_0
};
const static struct mt9t113_i2c_reg_conf mt9t113_wb_daylight_reg_config[] =
{
    {0x098E, 0x6848}, 	// MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6865}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6867}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6881}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
    {0x0990, 0x0009},		//0x0000 	// MCU_DATA_0
    {0x098E, 0x6883}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
    {0x0990, 0x0009},		//0x0000 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0

    {0x098E, 0xAC3B}, 	// MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
    {0x0990, 0x0041}, 	// MCU_DATA_0
    {0x098E, 0xAC3C}, 	// MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
    {0x0990, 0x0058}, 	// MCU_DATA_0
};

const static struct mt9t113_i2c_reg_conf mt9t113_wb_clouday_reg_config[] =
{
    {0x098E, 0x6848}, 	// MCU_ADDRESS [PRI_A_CONFIG_AWB_ALGO_RUN]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6865}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_ENTER]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6867}, 	// MCU_ADDRESS [PRI_A_CONFIG_STAT_ALGO_RUN]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x6881}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_ENTER]
    {0x0990, 0x0009},		//0x0000 	// MCU_DATA_0
    {0x098E, 0x6883}, 	// MCU_ADDRESS [PRI_A_CONFIG_SYSCTRL_ALGO_RUN]
    {0x0990, 0x0009},		//0x0000 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0006}, 	// MCU_DATA_0

    {0x098E, 0xAC3B}, 	// MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
    {0x0990, 0x003E}, 	// MCU_DATA_0
    {0x098E, 0xAC3C}, 	// MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
    {0x0990, 0x005E}, 	// MCU_DATA_0
};

const static struct mt9t113_i2c_reg_conf mt9t113_snapshot_reg_config[] =
{
    //[Capture]	2048 x 1536
    {0x098E, 0xEC09}, // MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x8400}, // MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0002}, // MCU_DATA_0
    {0x3400, 0x7A24}, // MIPI_CONTROL
};

const static struct mt9t113_i2c_reg_conf mt9t113_preview_reg_config[] =
{
    //[Preview]	1024x768
    {0x098E, 0xEC09}, // MCU_ADDRESS [PRI_B_NUM_OF_FRAMES_RUN]
    {0x0990, 0x0005}, // MCU_DATA_0
    {0x098E, 0x8400}, // MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0001}, // MCU_DATA_0
};

//=================================================================================================
//	* Name		:	4CDGX EVT1.0 Initial Setfile
//	* PLL mode	:	MCLK=24MHz / SYSCLK=30MHz / PCLK=60MHz
//	* FPS		:	Preview YUV 640X480 15~7.5fps Capture YUV 2048x1536 7.5fps
//	* Made by	:	SYS.LSI Sang-il Park
//	* Date		:	2009.11.03
//	* History
//						: 09.11.03	Initial draft (based LG-SB210 tuning value)
//						: 09.11.06	Changed GAS LUT & Alpha & AFIT for Shading compensation
//=================================================================================================

//=================================================================================================
//	ARM Go
//=================================================================================================
const static struct mt9t113_i2c_reg_conf mt9t113_init_reg_config1[] =
{
    // PLL
    {0x0018, 0x4129}, // TANDBY_CONTROL_AND_STATUS
    {0x0018, 0x4029}, // STANDBY_CONTROL_AND_STATUS
    {0x0010, 0x0118}, // PLL_DIVIDERS
    {0x0012, 0x0070}, // PLL_P_DIVIDERS
    {0x002A, 0x76A9}, // PLL_P4_P5_P6_DIVIDERS // 2011-3-18
    {0x0018, 0x402E}, //Standby:Default = 16430
};

// wait for FW initialization complete
//POLL_FIELD=STANDBY_CONTROL_AND_STATUS,STANDBY_DONE,==1,DELAY=10,TIMEOUT=100

const static struct mt9t113_i2c_reg_conf mt9t113_init_reg_config2[] =
{
    {0x0022, 0x0140}, // VDD_DIS_COUNTER
    {0x001E, 0x0701}, // PAD_SLEW_PAD_CONFIG
    {0x0112, 0x0012}, // RX_FIFO_CONTROL
    {0x3B84, 0x0062}, // I2C_MASTER_FREQUENCY_DIVIDER
    {0x098E, 0x2C03}, // MCU_ADDRESS [AWB_ALGO]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x2803}, // MCU_ADDRESS [AE_ALGO]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x4800}, //Row Start (A)
    {0x0990, 0x0010}, //      = 16
    {0x098E, 0x4802}, //Column Start (A)
    {0x0990, 0x0010}, //      = 16
    {0x098E, 0x4804}, //Row End (A)
    {0x0990, 0x062D}, //      = 1581
    {0x098E, 0x4806}, //Column End (A)
    {0x0990, 0x082D}, //      = 2093
    {0x098E, 0x4808}, //Base Frame Lines (A)
    {0x0990, 0x0359}, //      = 857
    {0x098E, 0x480A}, //Line Length (A)
    {0x0990, 0x0AB7}, //      = 2743
    {0x098E, 0x480C}, //Fine Correction (A)
    {0x0990, 0x0399}, //      = 921
    {0x098E, 0x480E}, //Row Speed (A)
    {0x0990, 0x0111}, //      = 273
    {0x098E, 0x4810}, //Read Mode (A)
    {0x0990, 0x046F}, //046C //      = 1132 //mirror  mode
    {0x098E, 0x4812}, //Fine IT Min (A)
    {0x0990, 0x0510}, //      = 1296
    {0x098E, 0x4814}, //Fine IT Max Margin (A)
    {0x0990, 0x01BA}, //      = 442
    {0x098E, 0x482D}, //Row Start (B)
    {0x0990, 0x0018}, //      = 24
    {0x098E, 0x482F}, //Column Start (B)
    {0x0990, 0x0018}, //      = 24
    {0x098E, 0x4831}, //Row End (B)
    {0x0990, 0x0627}, //      = 1575
    {0x098E, 0x4833}, //Column End (B)
    {0x0990, 0x0827}, //      = 2087
    {0x098E, 0x4835}, //Base Frame Lines (B)
    {0x0990, 0x065D}, //      = 1629
    {0x098E, 0x4837}, //Line Length (B)
    {0x0990, 0x0E8A}, //      = 3722
    {0x098E, 0x4839}, //Fine Correction (B)
    {0x0990, 0x019F}, //      = 415
    {0x098E, 0x483B}, //Row Speed (B)
    {0x0990, 0x0111}, //      = 273
    {0x098E, 0x483D}, //Read Mode (B)
    {0x0990, 0x0027}, //	0024//      = 36 //mirror mode
    {0x098E, 0x483F}, //Fine IT Min (B)
    {0x0990, 0x0266}, //      = 614
    {0x098E, 0x4841}, //Fine IT Max Margin (B)
    {0x0990, 0x010A}, //      = 266
    {0x098E, 0xB81A}, //fd_zone_height
    {0x0990, 0x0005}, //      = 5
    {0x098E, 0x481A}, //fd_period_50Hz (A)
    {0x0990, 0x00D2}, //      = 210
    {0x098E, 0x481C}, //fd_period_60Hz (A)
    {0x0990, 0x00AF}, //      = 175
    {0x098E, 0xC81E}, //fd_search_f1_50hz (A)
    {0x0990, 0x0022}, //      = 34
    {0x098E, 0xC81F}, //fd_search_f2_50hz (A)
    {0x0990, 0x0024}, //      = 36
    {0x098E, 0xC820}, //fd_search_f1_60hz (A)
    {0x0990, 0x0029}, //      = 41
    {0x098E, 0xC821}, //fd_search_f2_60hz (A)
    {0x0990, 0x002B}, //      = 43
    {0x098E, 0x4847}, //fd_period_50Hz (B)
    {0x0990, 0x009B}, //      = 155
    {0x098E, 0x4849}, //fd_period_60Hz (B)
    {0x0990, 0x0081}, //      = 129
    {0x098E, 0xC84B}, //fd_search_f1_50hz (B)
    {0x0990, 0x0018}, //      = 24
    {0x098E, 0xC84C}, //fd_search_f2_50hz (B)
    {0x0990, 0x001A}, //      = 26
    {0x098E, 0xC84D}, //fd_search_f1_60hz (B)
    {0x0990, 0x001E}, //      = 30
    {0x098E, 0xC84E}, //fd_search_f2_60hz (B)
    {0x0990, 0x0020}, //      = 32

    //{0x098E, 0x6800},	//Output Width (A)
    //{0x0990, 0x0280},	//      = 320 ==] 640
    //{0x098E, 0x6802},	//Output Height (A)
    //{0x0990, 0x01E0},	//      = 240 ==] 480

    {0x098E, 0x6800}, // MCU_ADDRESS [PRI_A_IMAGE_WIDTH]
    {0x0990, 0x0400}, // MCU_DATA_0
    {0x098E, 0x6802}, // MCU_ADDRESS [PRI_A_IMAGE_HEIGHT]
    {0x0990, 0x0300}, // MCU_DATA_0

    {0x098E, 0x6804}, //FOV Width (A)
    {0x0990, 0x0400}, //      = 1024
    {0x098E, 0x6806}, //FOV Height (A)
    {0x0990, 0x0300}, //      = 768
    {0x098E, 0xE892}, //JPEG Mode (A)
    {0x0990, 0x0000}, //      = 0
    {0x098E, 0x6C00}, //Output Width (B)
    {0x0990, 0x0800}, //      = 2048
    {0x098E, 0x6C02}, //Output Height (B)
    {0x0990, 0x0600}, //      = 1536
    {0x098E, 0x6C04}, //FOV Width (B)
    {0x0990, 0x0800}, //      = 2048
    {0x098E, 0x6C06}, //FOV Height (B)
    {0x0990, 0x0600}, //      = 1536
    {0x098E, 0xEC92}, //JPEG Mode (B)
    {0x0990, 0x0000}, //      = 0

    {0x098E, 0x6CA6}, // MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_TX_CONTROL_VAR]
    {0x0990, 0x082D}, // MCU_DATA_0
    {0x098E, 0xECA5}, // MCU_ADDRESS [PRI_B_CONFIG_JPEG_OB_SPOOF_CONTROL_VAR]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x6C94}, // MCU_ADDRESS [PRI_B_CONFIG_JPEG_CONFIG]
    {0x0990, 0x0C34}, // MCU_DATA_0
    {0x3C86, 0x00E1}, // OB_PCLK1_CONFIG
    {0x3C20, 0x0000}, // TX_SS_CONTROL
    //AE
    {0x098E, 0x6820}, // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_TARGET_FDZONE]
    {0x0990, 0x0007}, // MCU_DATA_0
    {0x098E, 0x6822}, // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_TARGET_AGAIN]
    {0x0990, 0x0064}, // MCU_DATA_0
    {0x098E, 0x6824}, // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_TARGET_DGAIN]
    {0x0990, 0x0080}, // MCU_DATA_0
    {0x098E, 0xE826}, // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_BASE_TARGET]
    {0x0990, 0x0045}, // MCU_DATA_0
    {0x098E, 0x6829}, // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MIN_VIRT_DGAIN]
    {0x0990, 0x0080}, // MCU_DATA_0
    {0x098E, 0x682B}, // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MAX_VIRT_DGAIN]
    {0x0990, 0x0080}, // MCU_DATA_0
    {0x098E, 0x682D}, // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MIN_VIRT_AGAIN]
    {0x0990, 0x0038}, // MCU_DATA_0
    {0x098E, 0x486F}, // MCU_ADDRESS [CAM1_CTL_MAX_ANALOG_GAIN]
    {0x0990, 0x0120}, // MCU_DATA_0
    {0x098E, 0x4871}, // MCU_ADDRESS [CAM1_CTL_MIN_ANALOG_GAIN]
    {0x0990, 0x0038}, // MCU_DATA_0
    {0x098E, 0x682F}, // MCU_ADDRESS [PRI_A_CONFIG_AE_TRACK_AE_MAX_VIRT_AGAIN]
    {0x0990, 0x0120}, // MCU_DATA_0
    {0x098E, 0x6815}, // MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_50HZ]
    {0x0990, 0x000D}, // MCU_DATA_0
    {0x098E, 0x6817}, // MCU_ADDRESS [PRI_A_CONFIG_FD_MAX_FDZONE_60HZ]
    {0x0990, 0x0010}, // MCU_DATA_0
    //FD_set
    {0x098E, 0xA005}, // MCU_ADDRESS [FD_FDPERIOD_SELECT]
    {0x0990, 0x0001}, // MCU_DATA_0

    {0x098E, 0x680F}, // MCU_ADDRESS [PRI_A_CONFIG_FD_ALGO_ENTER]
    {0x0990, 0x0003}, // MCU_DATA_0
    {0x098E, 0xA006}, // MCU_ADDRESS [FD_SMOOTH_COUNTER]
    {0x0990, 0x0008}, // MCU_DATA_0
    {0x098E, 0xA007}, // MCU_ADDRESS [FD_STAT_MIN]
    {0x0990, 0x0003}, // MCU_DATA_0
    {0x098E, 0xA008}, // MCU_ADDRESS [FD_STAT_MAX]
    {0x0990, 0x0005}, // MCU_DATA_0
    {0x098E, 0xA00A}, // MCU_ADDRESS [FD_MIN_AMPLITUDE]
    {0x0990, 0x0000}, // MCU_DATA_0
    //AWB
    {0x098E, 0x4873}, // MCU_ADDRESS [CAM1_AWB_CCM_L_0]
    {0x0990, 0x01E1}, // MCU_DATA_0
    {0x098E, 0x4875}, // MCU_ADDRESS [CAM1_AWB_CCM_L_1]
    {0x0990, 0xFEFA}, // MCU_DATA_0
    {0x098E, 0x4877}, // MCU_ADDRESS [CAM1_AWB_CCM_L_2]
    {0x0990, 0x0024}, // MCU_DATA_0
    {0x098E, 0x4879}, // MCU_ADDRESS [CAM1_AWB_CCM_L_3]
    {0x0990, 0xFFD0}, // MCU_DATA_0
    {0x098E, 0x487B}, // MCU_ADDRESS [CAM1_AWB_CCM_L_4]
    {0x0990, 0x0136}, // MCU_DATA_0
    {0x098E, 0x487D}, // MCU_ADDRESS [CAM1_AWB_CCM_L_5]
    {0x0990, 0xFFF7}, // MCU_DATA_0
    {0x098E, 0x487F}, // MCU_ADDRESS [CAM1_AWB_CCM_L_6]
    {0x0990, 0xFFD7}, // MCU_DATA_0
    {0x098E, 0x4881}, // MCU_ADDRESS [CAM1_AWB_CCM_L_7]
    {0x0990, 0xFF56}, // MCU_DATA_0
    {0x098E, 0x4883}, // MCU_ADDRESS [CAM1_AWB_CCM_L_8]
    {0x0990, 0x01D1}, // MCU_DATA_0
    {0x098E, 0x4885}, // MCU_ADDRESS [CAM1_AWB_CCM_L_9]
    {0x0990, 0x0022}, // MCU_DATA_0
    {0x098E, 0x4887}, // MCU_ADDRESS [CAM1_AWB_CCM_L_10]
    {0x0990, 0x005F}, // MCU_DATA_0
    {0x098E, 0x4889}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_0]
    {0x0990, 0x001A}, // MCU_DATA_0
    {0x098E, 0x488B}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_1]
    {0x0990, 0x005B}, // MCU_DATA_0
    {0x098E, 0x488D}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_2]
    {0x0990, 0xFF8A}, // MCU_DATA_0
    {0x098E, 0x488F}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_3]
    {0x0990, 0xFFF2}, // MCU_DATA_0
    {0x098E, 0x4891}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_4]
    {0x0990, 0x0022}, // MCU_DATA_0
    {0x098E, 0x4893}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_5]
    {0x0990, 0xFFEA}, // MCU_DATA_0
    {0x098E, 0x4895}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_6]
    {0x0990, 0x0021}, // MCU_DATA_0
    {0x098E, 0x4897}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_7]
    {0x0990, 0x0037}, // MCU_DATA_0
    {0x098E, 0x4899}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_8]
    {0x0990, 0xFFA7}, // MCU_DATA_0
    {0x098E, 0x489B}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_9]
    {0x0990, 0x0016}, // MCU_DATA_0
    {0x098E, 0x489D}, // MCU_ADDRESS [CAM1_AWB_CCM_RL_10]
    {0x0990, 0xFFC8}, // MCU_DATA_0
    {0x098E, 0x489F}, // MCU_ADDRESS [CAM1_AWB_LL_CCM_0]
    {0x0990, 0x0100}, // MCU_DATA_0
    {0x098E, 0x48A1}, // MCU_ADDRESS [CAM1_AWB_LL_CCM_1]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48A3}, // MCU_ADDRESS [CAM1_AWB_LL_CCM_2]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48A5}, // MCU_ADDRESS [CAM1_AWB_LL_CCM_3]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48A7}, // MCU_ADDRESS [CAM1_AWB_LL_CCM_4]
    {0x0990, 0x0100}, // MCU_DATA_0
    {0x098E, 0x48A9}, // MCU_ADDRESS [CAM1_AWB_LL_CCM_5]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48AB}, // MCU_ADDRESS [CAM1_AWB_LL_CCM_6]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48AD}, // MCU_ADDRESS [CAM1_AWB_LL_CCM_7]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48AF}, // MCU_ADDRESS [CAM1_AWB_LL_CCM_8]
    {0x0990, 0x0100}, // MCU_DATA_0
    //AWB
    {0x098E, 0x4887}, // MCU_ADDRESS
    {0x0990, 0x004E}, // MCU_DATA_0
    {0x098E, 0x489D}, // MCU_ADDRESS
    {0x0990, 0xFFE1}, // MCU_DATA_0
    {0x098E, 0x48B8}, // MCU_ADDRESS [CAM1_AWB_X_SHIFT]
    {0x0990, 0x002D}, // MCU_DATA_0
    {0x098E, 0x48BA}, // MCU_ADDRESS [CAM1_AWB_Y_SHIFT]
    {0x0990, 0x0011}, // MCU_DATA_0
    {0x098E, 0x48BC}, // MCU_ADDRESS [CAM1_AWB_RECIP_XSCALE]
    {0x0990, 0x0080}, // MCU_DATA_0
    {0x098E, 0x48BE}, // MCU_ADDRESS [CAM1_AWB_RECIP_YSCALE]
    {0x0990, 0x00AB}, // MCU_DATA_0
    {0x098E, 0x48C0}, // MCU_ADDRESS [CAM1_AWB_ROT_CENTER_X]
    {0x0990, 0x03FC}, // MCU_DATA_0
    {0x098E, 0x48C2}, // MCU_ADDRESS [CAM1_AWB_ROT_CENTER_Y]
    {0x0990, 0x03E7}, // MCU_DATA_0
    {0x098E, 0xC8C4}, // MCU_ADDRESS [CAM1_AWB_ROT_SIN]
    {0x0990, 0x0034}, // MCU_DATA_0
    {0x098E, 0xC8C5}, // MCU_ADDRESS [CAM1_AWB_ROT_COS]
    {0x0990, 0x0026}, // MCU_DATA_0
    {0x098E, 0x48C6}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_0]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48C8}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_1]
    {0x0990, 0x0001}, // MCU_DATA_0
    {0x098E, 0x48CA}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_2]
    {0x0990, 0x1100}, // MCU_DATA_0
    {0x098E, 0x48CC}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_3]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48CE}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_4]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48D0}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_5]
    {0x0990, 0x0011}, // MCU_DATA_0
    {0x098E, 0x48D2}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_6]
    {0x0990, 0x1111}, // MCU_DATA_0
    {0x098E, 0x48D4}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_7]
    {0x0990, 0x1100}, // MCU_DATA_0
    {0x098E, 0x48D6}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_8]
    {0x0990, 0x0000}, // MCU_DATA_0
    {0x098E, 0x48D8}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_9]
    {0x0990, 0x0011}, // MCU_DATA_0
    {0x098E, 0x48DA}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_10]
    {0x0990, 0x1111}, // MCU_DATA_0
    {0x098E, 0x48DC}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_11]
    {0x0990, 0x1110}, //1110// MCU_DATA_0
    {0x098E, 0x48DE}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_12]
    {0x0990, 0x0111}, // MCU_DATA_0
    {0x098E, 0x48E0}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_13]
    {0x0990, 0x1111}, // MCU_DATA_0
    {0x098E, 0x48E2}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_14]
    {0x0990, 0x2122}, // MCU_DATA_0
    {0x098E, 0x48E4}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_15]
    {0x0990, 0x2110}, // MCU_DATA_0
    {0x098E, 0x48E6}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_16]
    {0x0990, 0x0123}, // MCU_DATA_0
    {0x098E, 0x48E8}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_17]
    {0x0990, 0x3332}, // MCU_DATA_0
    {0x098E, 0x48EA}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_18]
    {0x0990, 0x2123}, // MCU_DATA_0
    {0x098E, 0x48EC}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_19]
    {0x0990, 0x3321}, // MCU_DATA_0
    {0x098E, 0x48EE}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_20]
    {0x0990, 0x0134}, // MCU_DATA_0
    {0x098E, 0x48F0}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_21]
    {0x0990, 0x4443}, // MCU_DATA_0
    {0x098E, 0x48F2}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_22]
    {0x0990, 0x2112}, // MCU_DATA_0
    {0x098E, 0x48F4}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_23]
    {0x0990, 0x3321}, // MCU_DATA_0
    {0x098E, 0x48F6}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_24]
    {0x0990, 0x0123}, // MCU_DATA_0
    {0x098E, 0x48F8}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_25]
    {0x0990, 0x3432}, // MCU_DATA_0
    {0x098E, 0x48FA}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_26]
    {0x0990, 0x1111}, // MCU_DATA_0
    {0x098E, 0x48FC}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_27]
    {0x0990, 0x2221}, // MCU_DATA_0
    {0x098E, 0x48FE}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_28]
    {0x0990, 0x0111}, // MCU_DATA_0
    {0x098E, 0x4900}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_29]
    {0x0990, 0x1111}, // MCU_DATA_0
    {0x098E, 0x4902}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_30]
    {0x0990, 0x1111}, // MCU_DATA_0
    {0x098E, 0x4904}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_31]
    {0x0990, 0x1111}, // MCU_DATA_0
    {0x098E, 0xC8B6}, // MCU_ADDRESS [CAM1_AWB_LUMA_THRESH_LOW]
    {0x0990, 0x0008}, // MCU_DATA_0
    {0x098E, 0xC8B5}, // MCU_ADDRESS [CAM1_AWB_LUMA_THRESH_HIGH]
    {0x0990, 0x00F5}, // MCU_DATA_0
    {0x098E, 0xAC37}, // MCU_ADDRESS [AWB_R_SCENE_RATIO_LOWER]
    {0x0990, 0x0036}, // MCU_DATA_0
    {0x098E, 0xAC38}, // MCU_ADDRESS [AWB_R_SCENE_RATIO_UPPER]
    {0x0990, 0x005E}, // MCU_DATA_0
    {0x098E, 0xAC39}, // MCU_ADDRESS [AWB_B_SCENE_RATIO_LOWER]
    {0x0990, 0x0026}, // MCU_DATA_0
    {0x098E, 0xAC3A}, // MCU_ADDRESS [AWB_B_SCENE_RATIO_UPPER]
    {0x0990, 0x004B}, // MCU_DATA_0
    {0x098E, 0xAC3B}, // MCU_ADDRESS [AWB_R_RATIO_PRE_AWB]
    {0x0990, 0x0055}, // MCU_DATA_0
    {0x098E, 0xAC3C}, // MCU_ADDRESS [AWB_B_RATIO_PRE_AWB]
    {0x0990, 0x0025}, // MCU_DATA_0

    {0x098E, 0x48DC}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_11]
    {0x0990, 0x1116}, //1110// MCU_DATA_0
    {0x098E, 0x48D4}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_7]
    {0x0990, 0xFFFF}, // MCU_DATA_0
    {0x098E, 0x48D8}, // MCU_ADDRESS [CAM1_AWB_WEIGHT_TABLE_9]
    {0x0990, 0xFFFF}, // MCU_DATA_0
    {0x098E, 0xE851}, // MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_L]
    {0x0990, 0x0080}, // MCU_DATA_0
    {0x098E, 0xE853}, // MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_L]
    {0x0990, 0x0095}, // MCU_DATA_0
    {0x098E, 0xE854}, // MCU_ADDRESS [PRI_A_CONFIG_AWB_K_R_R]
    {0x0990, 0x0086}, // MCU_DATA_0
    {0x098E, 0xE876}, // MCU_ADDRESS [PRI_A_CONFIG_LL_START_SATURATION]
    {0x0990, 0x00B0}, // MCU_DATA_0
    {0x098E, 0x48B8}, // MCU_ADDRESS [PRI_A_CONFIG_AWB_K_B_L]
    {0x0990, 0x002D}, // MCU_DATA_0
    {0x098E, 0x48BA}, // MCU_ADDRESS [PRI_A_CONFIG_LL_START_SATURATION]
    {0x0990, 0x0011}, // MCU_DATA_0

    //[Patch_5_2]
    {0x0982, 0x0000}, // ACCESS_CTL_STAT
    {0x098A, 0x0A80}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x3C3C},
    {0x0992, 0xCE05},
    {0x0994, 0x1F1F},
    {0x0996, 0x0204},
    {0x0998, 0x0CCC},
    {0x099A, 0x33D4},
    {0x099C, 0x30ED},
    {0x099E, 0x00FC},
    {0x098A, 0x0A90}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x0590},
    {0x0992, 0xBDA8},
    {0x0994, 0x93CE},
    {0x0996, 0x051F},
    {0x0998, 0x1F02},
    {0x099A, 0x0110},
    {0x099C, 0xCC33},
    {0x099E, 0xD830},
    {0x098A, 0x0AA0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0xED02},
    {0x0992, 0xCC05},
    {0x0994, 0xB8ED},
    {0x0996, 0x00C6},
    {0x0998, 0x06BD},
    {0x099A, 0xA8B1},
    {0x099C, 0xCE05},
    {0x099E, 0x1F1F},
    {0x098A, 0x0AB0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x0208},
    {0x0992, 0x0CCC},
    {0x0994, 0x33D6},
    {0x0996, 0x30ED},
    {0x0998, 0x00FC},
    {0x099A, 0x0592},
    {0x099C, 0xBDA8},
    {0x099E, 0x93CC},
    {0x098A, 0x0AC0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x33F4},
    {0x0992, 0x30ED},
    {0x0994, 0x02CC},
    {0x0996, 0xFFE9},
    {0x0998, 0xED00},
    {0x099A, 0xFC05},
    {0x099C, 0x94C4},
    {0x099E, 0x164F},
    {0x098A, 0x0AD0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0xBDA9},
    {0x0992, 0x0ACE},
    {0x0994, 0x051F},
    {0x0996, 0x1F02},
    {0x0998, 0x020A},
    {0x099A, 0xCC32},
    {0x099C, 0x1030},
    {0x099E, 0xED00},
    {0x098A, 0x0AE0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x4FBD},
    {0x0992, 0xA8E4},
    {0x0994, 0x3838},
    {0x0996, 0x393C},
    {0x0998, 0x3CFC},
    {0x099A, 0x0322},
    {0x099C, 0xB303},
    {0x099E, 0x2030},
    {0x098A, 0x0AF0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0xED02},
    {0x0992, 0xCE03},
    {0x0994, 0x141F},
    {0x0996, 0x0408},
    {0x0998, 0x3ECE},
    {0x099A, 0x0314},
    {0x099C, 0x1F0B},
    {0x099E, 0x0134},
    {0x098A, 0x0B00}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x30EC},
    {0x0992, 0x0227},
    {0x0994, 0x2F83},
    {0x0996, 0x0000},
    {0x0998, 0x2C18},
    {0x099A, 0xF603},
    {0x099C, 0x244F},
    {0x099E, 0xED00},
    {0x098A, 0x0B10}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0xFC03},
    {0x0992, 0x20A3},
    {0x0994, 0x00B3},
    {0x0996, 0x0322},
    {0x0998, 0x241A},
    {0x099A, 0xFC03},
    {0x099C, 0x22FD},
    {0x099E, 0x0320},
    {0x098A, 0x0B20}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x2012},
    {0x0992, 0xF603},
    {0x0994, 0x244F},
    {0x0996, 0xF303},
    {0x0998, 0x20B3},
    {0x099A, 0x0322},
    {0x099C, 0x2306},
    {0x099E, 0xFC03},
    {0x098A, 0x0B30}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x22FD},
    {0x0992, 0x0320},
    {0x0994, 0xBD7D},
    {0x0996, 0x9038},
    {0x0998, 0x3839},
    {0x099A, 0x3C3C},
    {0x099C, 0xFC07},
    {0x099E, 0x4327},
    {0x098A, 0x0B40}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x5FDE},
    {0x0992, 0x431F},
    {0x0994, 0xB410},
    {0x0996, 0x563C},
    {0x0998, 0xFC07},
    {0x099A, 0x4130},
    {0x099C, 0xED00},
    {0x099E, 0x3CCC},
    {0x098A, 0x0B50}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x0008},
    {0x0992, 0x30ED},
    {0x0994, 0x00FC},
    {0x0996, 0x0743},
    {0x0998, 0xBDAA},
    {0x099A, 0x7C38},
    {0x099C, 0x38BD},
    {0x099E, 0xE9E4},
    {0x098A, 0x0B60}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x30ED},
    {0x0992, 0x02CC},
    {0x0994, 0x0064},
    {0x0996, 0xED00},
    {0x0998, 0xCC01},
    {0x099A, 0x00BD},
    {0x099C, 0xAA7C},
    {0x099E, 0xFD03},
    {0x098A, 0x0B70}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x103C},
    {0x0992, 0xFC07},
    {0x0994, 0x4530},
    {0x0996, 0xED00},
    {0x0998, 0x3CCC},
    {0x099A, 0x0008},
    {0x099C, 0x30ED},
    {0x099E, 0x00FC},
    {0x098A, 0x0B80}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x0743},
    {0x0992, 0xBDAA},
    {0x0994, 0x7C38},
    {0x0996, 0x38BD},
    {0x0998, 0xE9E4},
    {0x099A, 0x30ED},
    {0x099C, 0x02CC},
    {0x099E, 0x0064},
    {0x098A, 0x0B90}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0xED00},
    {0x0992, 0xCC01},
    {0x0994, 0x00BD},
    {0x0996, 0xAA7C},
    {0x0998, 0xFD03},
    {0x099A, 0x1220},
    {0x099C, 0x03BD},
    {0x099E, 0x7993},
    {0x098A, 0x0BA0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x3838},
    {0x0992, 0x390F},
    {0x0994, 0xF601},
    {0x0996, 0x05C1},
    {0x0998, 0x0326},
    {0x099A, 0x14F6},
    {0x099C, 0x0106},
    {0x099E, 0xC106},
    {0x098A, 0x0BB0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x260D},
    {0x0992, 0xF630},
    {0x0994, 0x4DC4},
    {0x0996, 0xF0CA},
    {0x0998, 0x08F7},
    {0x099A, 0x304D},
    {0x099C, 0xBD0B},
    {0x099E, 0xC10E},
    {0x098A, 0x0BC0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x39F6},
    {0x0992, 0x304D},
    {0x0994, 0xC4F0},
    {0x0996, 0xCA09},
    {0x0998, 0xF730},
    {0x099A, 0x4DDE},
    {0x099C, 0xF218},
    {0x099E, 0xCE0A},
    {0x098A, 0x0BD0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x00CC},
    {0x0992, 0x001D},
    {0x0994, 0xBDB5},
    {0x0996, 0x31DE},
    {0x0998, 0xA818},
    {0x099A, 0xCE0A},
    {0x099C, 0x1ECC},
    {0x099E, 0x001D},
    {0x098A, 0x0BE0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0xBDB5},
    {0x0992, 0x31DE},
    {0x0994, 0xA618},
    {0x0996, 0xCE0A},
    {0x0998, 0x3CCC},
    {0x099A, 0x0013},
    {0x099C, 0xBDB5},
    {0x099E, 0x31CC},
    {0x098A, 0x0BF0}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x0A80},
    {0x0992, 0xFD0A},
    {0x0994, 0x0ECC},
    {0x0996, 0x0AE7},
    {0x0998, 0xFD0A},
    {0x099A, 0x30CC},
    {0x099C, 0x0B3A},
    {0x099E, 0xFD0A},
    {0x098A, 0x0C00}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x4CCC},
    {0x0992, 0x0A00},
    {0x0994, 0xDDF2},
    {0x0996, 0xCC0A},
    {0x0998, 0x1EDD},
    {0x099A, 0xA8CC},
    {0x099C, 0x0A3C},
    {0x099E, 0xDDA6},
    {0x098A, 0x0C10}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0xC601},
    {0x0992, 0xF701},
    {0x0994, 0x0CF7},
    {0x0996, 0x010D},
    {0x098A, 0x8C18}, // PHYSICAL_ADDR_ACCESS
    {0x0990, 0x0039}, // MCU_DATA_0
    {0x098E, 0x0012}, // MCU_ADDRESS [MON_ADDR]
    {0x0990, 0x0BA3}, // MCU_DATA_0
    {0x098E, 0x0003}, // MCU_ADDRESS [MON_ALGO]
    {0x0990, 0x0004}, // MCU_DATA_0
};

const static struct mt9t113_i2c_reg_conf mt9t113_init_reg_config3_sunny[] =
{
    //[Char_settings]
    {0x3ED6,  0x0F00}, // RESERVED
    {0x3EF2,  0xD965}, // RESERVED
    {0x3FD2,  0xD965}, // RESERVED
    {0x3EF8,  0x7F7F}, // RESERVED
    {0x3ED8,  0x7F1D}, // RESERVED
    {0x3172,  0x0033}, // RESERVED
    {0x3EEA,  0x0200}, // RESERVED
    {0x3EE2,  0x0050}, // RESERVED
    {0x316A,  0x8200}, // RESERVED
    {0x316C,  0x8200}, // RESERVED
    {0x3EFC,  0xA8E8}, // RESERVED
    {0x3EFE,  0x130D}, // RESERVED
    // Additional Optimized Settings
    {0x3180,  0xB3FF}, // RESERVED
    {0x30B2,  0xC000}, // RESERVED
    {0x30BC,  0x0384}, // RESERVED
    {0x30C0,  0x1220}, // RESERVED

    // Low_Power_Mode
    {0x3170, 0x000A}, //Dynamic pwr setting
    {0x3174, 0x8060}, //Dynamic pwr setting
    {0x3ECC, 0x22B0}, //Dynamic pwr setting
    {0x098E, 0x482B}, //LP Mode (A)
    {0x0990, 0x22B0}, //
    {0x098E, 0x4858}, //LP Mode (B)
    {0x0990, 0x22B0},
    {0x317A, 0x000A},
    {0x098E, 0x4822},
    {0x0990, 0x000A},
    {0x098E, 0x4824},
    {0x0990, 0x000A},
    {0x098E, 0x484F},
    {0x0990, 0x000A},
    {0x098E, 0x4851},
    {0x0990, 0x000A},

    //Lens Correction
    {0x3210,  0x01B0}, // COLOR_PIPELINE_CONTROL
    {0x3640,  0x0190}, // P_G1_P0Q0
    {0x3642,  0x048E}, // P_G1_P0Q1
    {0x3644,  0x6911}, // P_G1_P0Q2
    {0x3646,  0x262A}, // P_G1_P0Q3
    {0x3648,  0xC211}, // P_G1_P0Q4
    {0x364A,  0x0250}, // P_R_P0Q0
    {0x364C,  0xA74E}, // P_R_P0Q1
    {0x364E,  0x3C71}, // P_R_P0Q2
    {0x3650,  0x1110}, // P_R_P0Q3
    {0x3652,  0xF130}, // P_R_P0Q4
    {0x3654,  0x0250}, // P_B_P0Q0
    {0x3656,  0x5E4E}, // P_B_P0Q1
    {0x3658,  0x1B51}, // P_B_P0Q2
    {0x365A,  0x988F}, // P_B_P0Q3
    {0x365C,  0x9D30}, // P_B_P0Q4
    {0x365E,  0x01F0}, // P_G2_P0Q0
    {0x3660,  0xE00E}, // P_G2_P0Q1
    {0x3662,  0x6E51}, // P_G2_P0Q2
    {0x3664,  0x352F}, // P_G2_P0Q3
    {0x3666,  0xE0D1}, // P_G2_P0Q4
    {0x3680,  0xA32E}, // P_G1_P1Q0
    {0x3682,  0x8F2F}, // P_G1_P1Q1
    {0x3684,  0xBDAF}, // P_G1_P1Q2
    {0x3686,  0x06B0}, // P_G1_P1Q3
    {0x3688,  0x2550}, // P_G1_P1Q4
    {0x368A,  0xA6CE}, // P_R_P1Q0
    {0x368C,  0x16CE}, // P_R_P1Q1
    {0x368E,  0x702C}, // P_R_P1Q2
    {0x3690,  0xCD8E}, // P_R_P1Q3
    {0x3692,  0x8310}, // P_R_P1Q4
    {0x3694,  0x11AD}, // P_B_P1Q0
    {0x3696,  0x1B4F}, // P_B_P1Q1
    {0x3698,  0x1AD0}, // P_B_P1Q2
    {0x369A,  0x80B0}, // P_B_P1Q3
    {0x369C,  0x9512}, // P_B_P1Q4
    {0x369E,  0x214B}, // P_G2_P1Q0
    {0x36A0,  0xA62F}, // P_G2_P1Q1
    {0x36A2,  0x27B0}, // P_G2_P1Q2
    {0x36A4,  0x54D0}, // P_G2_P1Q3
    {0x36A6,  0x81D2}, // P_G2_P1Q4
    {0x36C0,  0x0F92}, // P_G1_P2Q0
    {0x36C2,  0x0391}, // P_G1_P2Q1
    {0x36C4,  0xF093}, // P_G1_P2Q2
    {0x36C6,  0x9193}, // P_G1_P2Q3
    {0x36C8,  0x0196}, // P_G1_P2Q4
    {0x36CA,  0x1B52}, // P_R_P2Q0
    {0x36CC,  0x094F}, // P_R_P2Q1
    {0x36CE,  0xCAB3}, // P_R_P2Q2
    {0x36D0,  0xA173}, // P_R_P2Q3
    {0x36D2,  0x7355}, // P_R_P2Q4
    {0x36D4,  0x6A71}, // P_B_P2Q0
    {0x36D6,  0x6530}, // P_B_P2Q1
    {0x36D8,  0xB293}, // P_B_P2Q2
    {0x36DA,  0xC272}, // P_B_P2Q3
    {0x36DC,  0x6335}, // P_B_P2Q4
    {0x36DE,  0x0AB2}, // P_G2_P2Q0
    {0x36E0,  0x154E}, // P_G2_P2Q1
    {0x36E2,  0xF453}, // P_G2_P2Q2
    {0x36E4,  0x9533}, // P_G2_P2Q3
    {0x36E6,  0x0DB6}, // P_G2_P2Q4
    {0x3700,  0x610C}, // P_G1_P3Q0
    {0x3702,  0x3110}, // P_G1_P3Q1
    {0x3704,  0xB071}, // P_G1_P3Q2
    {0x3706,  0xF391}, // P_G1_P3Q3
    {0x3708,  0x3F94}, // P_G1_P3Q4
    {0x370A,  0x0BF0}, // P_R_P3Q0
    {0x370C,  0x6FAE}, // P_R_P3Q1
    {0x370E,  0xDC72}, // P_R_P3Q2
    {0x3710,  0xCF10}, // P_R_P3Q3
    {0x3712,  0x0A94}, // P_R_P3Q4
    {0x3714,  0xADEC}, // P_B_P3Q0
    {0x3716,  0x9F70}, // P_B_P3Q1
    {0x3718,  0xB052}, // P_B_P3Q2
    {0x371A,  0x7C90}, // P_B_P3Q3
    {0x371C,  0x2954}, // P_B_P3Q4
    {0x371E,  0x576D}, // P_G2_P3Q0
    {0x3720,  0x3D90}, // P_G2_P3Q1
    {0x3722,  0xE512}, // P_G2_P3Q2
    {0x3724,  0xAC72}, // P_G2_P3Q3
    {0x3726,  0x3574}, // P_G2_P3Q4
    {0x3740,  0x9CB3}, // P_G1_P4Q0
    {0x3742,  0xAF53}, // P_G1_P4Q1
    {0x3744,  0x6EF6}, // P_G1_P4Q2
    {0x3746,  0x6D95}, // P_G1_P4Q3
    {0x3748,  0xA039}, // P_G1_P4Q4
    {0x374A,  0xA413}, // P_R_P4Q0
    {0x374C,  0xC7D2}, // P_R_P4Q1
    {0x374E,  0x6436}, // P_R_P4Q2
    {0x3750,  0x0416}, // P_R_P4Q3
    {0x3752,  0x8EB9}, // P_R_P4Q4
    {0x3754,  0x9153}, // P_B_P4Q0
    {0x3756,  0xAF93}, // P_B_P4Q1
    {0x3758,  0x6196}, // P_B_P4Q2
    {0x375A,  0x47F5}, // P_B_P4Q3
    {0x375C,  0x9179}, // P_B_P4Q4
    {0x375E,  0x9093}, // P_G2_P4Q0
    {0x3760,  0x81D2}, // P_G2_P4Q1
    {0x3762,  0x6296}, // P_G2_P4Q2
    {0x3764,  0x0756}, // P_G2_P4Q3
    {0x3766,  0x9BD9}, // P_G2_P4Q4
    {0x3782,  0x02E0}, // CENTER_ROW
    {0x3784,  0x03C0}, // CENTER_COLUMN
    {0x3210,  0x01B8}, // COLOR_PIPELINE_CONTROL

    //low_light
    {0x098E,  0x4918}, // MCU_ADDRESS [CAM1_LL_START_GAIN_METRIC]
    {0x0990,  0x0039}, // MCU_DATA_0
    {0x098E,  0x491A}, // MCU_ADDRESS [CAM1_LL_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0x6872}, // MCU_ADDRESS [PRI_A_CONFIG_LL_START_BRIGHTNESS]
    {0x0990,  0x0005}, // MCU_DATA_0
    {0x098E,  0x6874}, // MCU_ADDRESS [PRI_A_CONFIG_LL_STOP_BRIGHTNESS]
    {0x0990,  0x008C}, // MCU_DATA_0
    {0x098E,  0x4956}, // MCU_ADDRESS [CAM1_LL_DC_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x4958}, // MCU_ADDRESS [CAM1_LL_DC_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0x495A}, // MCU_ADDRESS [CAM1_LL_DC_START]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x495C}, // MCU_ADDRESS [CAM1_LL_DC_STOP]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x495E}, // MCU_ADDRESS [CAM1_LL_CDC_AGG_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x4960}, // MCU_ADDRESS [CAM1_LL_CDC_AGG_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0xC962}, // MCU_ADDRESS [CAM1_LL_CDC_AGG_START]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0xC963}, // MCU_ADDRESS [CAM1_LL_CDC_AGG_STOP]
    {0x0990,  0x0003}, // MCU_DATA_0
    {0x098E,  0x4964}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x4966}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0x4968}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_T3START]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x496A}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_T3STOP]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x496C}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_T4START]
    {0x0990,  0x0014}, // MCU_DATA_0
    {0x098E,  0x496E}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_T4STOP]
    {0x0990,  0x000C}, // MCU_DATA_0
    {0x098E,  0xC970}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_TO_START]
    {0x0990,  0x0004}, // MCU_DATA_0
    {0x098E,  0xC971}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_TO_STOP]
    {0x0990,  0x000F}, // MCU_DATA_0
    {0x098E,  0x4972}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x4974}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0x4976}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_T3START]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x4978}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_T3STOP]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x497A}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_T4START]
    {0x0990,  0x00C8}, // MCU_DATA_0
    {0x098E,  0x497C}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_T4STOP]
    {0x0990,  0x003C}, // MCU_DATA_0
    {0x098E,  0xC97E}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_TO_START]
    {0x0990,  0x0004}, // MCU_DATA_0
    {0x098E,  0xC97F}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_TO_STOP]
    {0x0990,  0x000F}, // MCU_DATA_0
    {0x098E,  0x491C}, // MCU_ADDRESS [CAM1_LL_GRB_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x491E}, // MCU_ADDRESS [CAM1_LL_GRB_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0xC920}, // MCU_ADDRESS [CAM1_LL_GRB_SLOPE_START]
    {0x0990,  0x000B}, // MCU_DATA_0
    {0x098E,  0xC921}, // MCU_ADDRESS [CAM1_LL_GRB_SLOPE_STOP]
    {0x0990,  0x002C}, // MCU_DATA_0
    {0x098E,  0xC922}, // MCU_ADDRESS [CAM1_LL_GRB_OFFSET_START]
    {0x0990,  0x0007}, // MCU_DATA_0
    {0x098E,  0xC923}, // MCU_ADDRESS [CAM1_LL_GRB_OFFSET_STOP]
    {0x0990,  0x001D}, // MCU_DATA_0
    {0x098E,  0x4926}, // MCU_ADDRESS [CAM1_LL_SFFB_START_ANALOG_GAIN]
    {0x0990,  0x0039}, // MCU_DATA_0
    {0x098E,  0x4928}, // MCU_ADDRESS [CAM1_LL_SFFB_END_ANALOG_GAIN]
    {0x0990,  0x00A0}, // MCU_DATA_0
    {0x098E,  0x492A}, // MCU_ADDRESS [CAM1_LL_SFFB_RAMP_START]
    {0x0990,  0x0082}, // MCU_DATA_0
    {0x098E,  0x492C}, // MCU_ADDRESS [CAM1_LL_SFFB_RAMP_STOP]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x492E}, // MCU_ADDRESS [CAM1_LL_SFFB_SLOPE_START]
    {0x0990,  0x0015}, // MCU_DATA_0
    {0x098E,  0x4930}, // MCU_ADDRESS [CAM1_LL_SFFB_SLOPE_STOP]
    {0x0990,  0x0015}, // MCU_DATA_0
    {0x098E,  0x4932}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH1START]
    {0x0990,  0x0002}, // MCU_DATA_0
    {0x098E,  0x4934}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH1STOP]
    {0x0990,  0x0004}, // MCU_DATA_0
    {0x098E,  0x4936}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH2START]
    {0x0990,  0x0008}, // MCU_DATA_0
    {0x098E,  0x4938}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH2STOP]
    {0x0990,  0x0009}, // MCU_DATA_0
    {0x098E,  0x493A}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH3START]
    {0x0990,  0x000C}, // MCU_DATA_0
    {0x098E,  0x493C}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH3STOP]
    {0x0990,  0x000D}, // MCU_DATA_0
    {0x098E,  0x493E}, // MCU_ADDRESS [CAM1_LL_SFFB_MAX_THRESH_START]
    {0x0990,  0x0015}, // MCU_DATA_0
    {0x098E,  0x4940}, // MCU_ADDRESS [CAM1_LL_SFFB_MAX_THRESH_STOP]
    {0x0990,  0x0013}, // MCU_DATA_0
    {0x098E,  0xC944}, // MCU_ADDRESS [CAM1_LL_SFFB_FLATNESS_START]
    {0x0990,  0x0023}, // MCU_DATA_0
    {0x098E,  0xC945}, // MCU_ADDRESS [CAM1_LL_SFFB_FLATNESS_STOP]
    {0x0990,  0x007F}, // MCU_DATA_0
    {0x098E,  0xC946}, // MCU_ADDRESS [CAM1_LL_SFFB_TRANSITION_START]
    {0x0990,  0x0007}, // MCU_DATA_0
    {0x098E,  0xC947}, // MCU_ADDRESS [CAM1_LL_SFFB_TRANSITION_STOP]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0xC948}, // MCU_ADDRESS [CAM1_LL_SFFB_SOBEL_FLAT_START]
    {0x0990,  0x0002}, // MCU_DATA_0
    {0x098E,  0xC949}, // MCU_ADDRESS [CAM1_LL_SFFB_SOBEL_FLAT_STOP]
    {0x0990,  0x0002}, // MCU_DATA_0
    {0x098E,  0xC94A}, // MCU_ADDRESS [CAM1_LL_SFFB_SOBEL_SHARP_START]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0xC94B}, // MCU_ADDRESS [CAM1_LL_SFFB_SOBEL_SHARP_STOP]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0xC906}, // MCU_ADDRESS [CAM1_LL_DM_EDGE_TH_START]
    {0x0990,  0x0006}, // MCU_DATA_0
    {0x098E,  0xC907}, // MCU_ADDRESS [CAM1_LL_DM_EDGE_TH_STOP]
    {0x0990,  0x0028}, // MCU_DATA_0
    {0x098E,  0xBC02}, // MCU_ADDRESS [LL_MODE]
    {0x0990,  0x0005}, // MCU_DATA_0
    {0x098E,  0xC908}, // MCU_ADDRESS [CAM1_LL_AP_KNEE_START]
    {0x0990,  0x0006}, // MCU_DATA_0
    {0x098E,  0xC909}, // MCU_ADDRESS [CAM1_LL_AP_KNEE_STOP]
    {0x0990,  0x0028}, // MCU_DATA_0
    {0x098E,  0xC90A}, // MCU_ADDRESS [CAM1_LL_AP_MANTISSA_START]
    {0x0990,  0x0007}, // MCU_DATA_0
    {0x326C,  0x0F0A}, // APERTURE_PARAMETERS_2D
    {0x098E,  0xC94C}, // MCU_ADDRESS [CAM1_LL_DELTA_GAIN]
    {0x0990,  0x0003}, // MCU_DATA_0
    {0x098E,  0xC94E}, // MCU_ADDRESS [CAM1_LL_DELTA_THRESHOLD_START]
    {0x0990,  0x003C}, // MCU_DATA_0
    {0x098E,  0xC94F}, // MCU_ADDRESS [CAM1_LL_DELTA_THRESHOLD_STOP]
    {0x0990,  0x0064}, // MCU_DATA_0
    {0x098E,  0xE877}, // MCU_ADDRESS [PRI_A_CONFIG_LL_END_SATURATION]
    {0x0990,  0x0050}, // MCU_DATA_0
    //gamma
    {0x098E,  0x3C42}, // MCU_ADDRESS [LL_START_GAMMA_FTB]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x3C44}, // MCU_ADDRESS [LL_STOP_GAMMA_FTB]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x4912}, // MCU_ADDRESS [CAM1_LL_START_GAMMA_BM]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x4914}, // MCU_ADDRESS [CAM1_LL_MID_GAMMA_BM]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x4916}, // MCU_ADDRESS [CAM1_LL_STOP_GAMMA_BM]
    {0x0990,  0x0037}, // MCU_DATA_0
    {0x098E,  0xBC09}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_0]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0xBC0A}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_1]
    {0x0990,  0x0011}, // MCU_DATA_0
    {0x098E,  0xBC0B}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_2]
    {0x0990,  0x0023}, // MCU_DATA_0
    {0x098E,  0xBC0C}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_3]
    {0x0990,  0x003F}, // MCU_DATA_0
    {0x098E,  0xBC0D}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_4]
    {0x0990,  0x0067}, // MCU_DATA_0
    {0x098E,  0xBC0E}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_5]
    {0x0990,  0x0085}, // MCU_DATA_0
    {0x098E,  0xBC0F}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_6]
    {0x0990,  0x009B}, // MCU_DATA_0
    {0x098E,  0xBC10}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_7]
    {0x0990,  0x00AD}, // MCU_DATA_0
    {0x098E,  0xBC11}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_8]
    {0x0990,  0x00BB}, // MCU_DATA_0
    {0x098E,  0xBC12}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_9]
    {0x0990,  0x00C7}, // MCU_DATA_0
    {0x098E,  0xBC13}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_10]
    {0x0990,  0x00D1}, // MCU_DATA_0
    {0x098E,  0xBC14}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_11]
    {0x0990,  0x00DA}, // MCU_DATA_0
    {0x098E,  0xBC15}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_12]
    {0x0990,  0x00E1}, // MCU_DATA_0
    {0x098E,  0xBC16}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_13]
    {0x0990,  0x00E8}, // MCU_DATA_0
    {0x098E,  0xBC17}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_14]
    {0x0990,  0x00EE}, // MCU_DATA_0
    {0x098E,  0xBC18}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_15]
    {0x0990,  0x00F3}, // MCU_DATA_0
    {0x098E,  0xBC19}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_16]
    {0x0990,  0x00F7}, // MCU_DATA_0
    {0x098E,  0xBC1A}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_17]
    {0x0990,  0x00FB}, // MCU_DATA_0
    {0x098E,  0xBC1B}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_18]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0xBC1C}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_0]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0xBC1D}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_1]
    {0x0990,  0x0011}, // MCU_DATA_0
    {0x098E,  0xBC1E}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_2]
    {0x0990,  0x0023}, // MCU_DATA_0
    {0x098E,  0xBC1F}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_3]
    {0x0990,  0x003F}, // MCU_DATA_0
    {0x098E,  0xBC20}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_4]
    {0x0990,  0x0067}, // MCU_DATA_0
    {0x098E,  0xBC21}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_5]
    {0x0990,  0x0085}, // MCU_DATA_0
    {0x098E,  0xBC22}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_6]
    {0x0990,  0x009B}, // MCU_DATA_0
    {0x098E,  0xBC23}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_7]
    {0x0990,  0x00AD}, // MCU_DATA_0
    {0x098E,  0xBC24}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_8]
    {0x0990,  0x00BB}, // MCU_DATA_0
    {0x098E,  0xBC25}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_9]
    {0x0990,  0x00C7}, // MCU_DATA_0
    {0x098E,  0xBC26}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_10]
    {0x0990,  0x00D1}, // MCU_DATA_0
    {0x098E,  0xBC27}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_11]
    {0x0990,  0x00DA}, // MCU_DATA_0
    {0x098E,  0xBC28}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_12]
    {0x0990,  0x00E1}, // MCU_DATA_0
    {0x098E,  0xBC29}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_13]
    {0x0990,  0x00E8}, // MCU_DATA_0
    {0x098E,  0xBC2A}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_14]
    {0x0990,  0x00EE}, // MCU_DATA_0
    {0x098E,  0xBC2B}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_15]
    {0x0990,  0x00F3}, // MCU_DATA_0
    {0x098E,  0xBC2C}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_16]
    {0x0990,  0x00F7}, // MCU_DATA_0
    {0x098E,  0xBC2D}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_17]
    {0x0990,  0x00FB}, // MCU_DATA_0
    {0x098E,  0xBC2E}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_18]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0xBC2F}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_0]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0xBC30}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_1]
    {0x0990,  0x0017}, // MCU_DATA_0
    {0x098E,  0xBC31}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_2]
    {0x0990,  0x0020}, // MCU_DATA_0
    {0x098E,  0xBC32}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_3]
    {0x0990,  0x0032}, // MCU_DATA_0
    {0x098E,  0xBC33}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_4]
    {0x0990,  0x005A}, // MCU_DATA_0
    {0x098E,  0xBC34}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_5]
    {0x0990,  0x0078}, // MCU_DATA_0
    {0x098E,  0xBC35}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_6]
    {0x0990,  0x0089}, // MCU_DATA_0
    {0x098E,  0xBC36}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_7]
    {0x0990,  0x0098}, // MCU_DATA_0
    {0x098E,  0xBC37}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_8]
    {0x0990,  0x00A6}, // MCU_DATA_0
    {0x098E,  0xBC38}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_9]
    {0x0990,  0x00B4}, // MCU_DATA_0
    {0x098E,  0xBC39}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_10]
    {0x0990,  0x00C3}, // MCU_DATA_0
    {0x098E,  0xBC3A}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_11]
    {0x0990,  0x00CB}, // MCU_DATA_0
    {0x098E,  0xBC3B}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_12]
    {0x0990,  0x00D0}, // MCU_DATA_0
    {0x098E,  0xBC3C}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_13]
    {0x0990,  0x00D4}, // MCU_DATA_0
    {0x098E,  0xBC3D}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_14]
    {0x0990,  0x00DC}, // MCU_DATA_0
    {0x098E,  0xBC3E}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_15]
    {0x0990,  0x00E4}, // MCU_DATA_0
    {0x098E,  0xBC3F}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_16]
    {0x0990,  0x00EA}, // MCU_DATA_0
    {0x098E,  0xBC40}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_17]
    {0x0990,  0x00F5}, // MCU_DATA_0
    {0x098E,  0xBC41}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_18]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0x3C42}, // MCU_ADDRESS [LL_START_GAMMA_FTB]
    {0x0990,  0x0032}, // MCU_DATA_0
    {0x098E,  0x3C44}, // MCU_ADDRESS [LL_STOP_GAMMA_FTB]
    {0x0990,  0x0000}, // MCU_DATA_0

    //A CCM
    {0x0018,  0x002A}, // STANDBY_CONTROL_AND_STATUS
    {0x098E,  0xAC02}, // MCU_ADDRESS [AWB_MODE]
    {0x0990,  0x0006}, // MCU_DATA_0
    {0x098E,  0x2800}, // MCU_ADDRESS [AE_TRACK_STATUS]
    {0x0990,  0x001C}, // MCU_DATA_0
    {0x098E,  0x8400}, // MCU_ADDRESS
    {0x0990,  0x0006}, // MCU_DATA_0
};
const static struct mt9t113_i2c_reg_conf mt9t113_init_reg_config3_byd[] =
{
    //[Char_settings]
    {0x3ED6,  0x0F00}, // RESERVED
    {0x3EF2,  0xD965}, // RESERVED
    {0x3FD2,  0xD965}, // RESERVED
    {0x3EF8,  0x7F7F}, // RESERVED
    {0x3ED8,  0x7F1D}, // RESERVED
    {0x3172,  0x0033}, // RESERVED
    {0x3EEA,  0x0200}, // RESERVED
    {0x3EE2,  0x0050}, // RESERVED
    {0x316A,  0x8200}, // RESERVED
    {0x316C,  0x8200}, // RESERVED
    {0x3EFC,  0xA8E8}, // RESERVED
    {0x3EFE,  0x130D}, // RESERVED
    // Additional Optimized Settings
    {0x3180,  0xB3FF}, // RESERVED
    {0x30B2,  0xC000}, // RESERVED
    {0x30BC,  0x0384}, // RESERVED
    {0x30C0,  0x1220}, // RESERVED

    // Low_Power_Mode
    {0x3170, 0x000A }, //Dynamic pwr setting
    {0x3174, 0x8060 }, //Dynamic pwr setting
    {0x3ECC, 0x22B0 }, //Dynamic pwr setting
    {0x098E, 0x482B }, //LP Mode (A)
    {0x0990, 0x22B0 }, //
    {0x098E, 0x4858 }, //LP Mode (B)
    {0x0990, 0x22B0 },
    {0x317A, 0x000A },
    {0x098E, 0x4822 },
    {0x0990, 0x000A },
    {0x098E, 0x4824 },
    {0x0990, 0x000A },
    {0x098E, 0x484F },
    {0x0990, 0x000A },
    {0x098E, 0x4851 },
    {0x0990, 0x000A },

    //Lens Correction
    {0x3210,  0x01B0},
    {0x3640,  0x00D0},
    {0x3642,  0x130E},
    {0x3644,  0x62B1},
    {0x3646,  0xF4CE},
    {0x3648,  0x65ED},
    {0x364A,  0x0190},
    {0x364C,  0xD28D},
    {0x364E,  0x4291},
    {0x3650,  0x178D},
    {0x3652,  0x326C},
    {0x3654,  0x01D0},
    {0x3656,  0x5EEE},
    {0x3658,  0x2AB1},
    {0x365A,  0xF30E},
    {0x365C,  0x2E2E},
    {0x365E,  0x06D0},
    {0x3660,  0xC4EE},
    {0x3662,  0x7C71},
    {0x3664,  0x908F},
    {0x3666,  0x9FEF},
    {0x3680,  0xBDAD},
    {0x3682,  0xC38E},
    {0x3684,  0xA050},
    {0x3686,  0x28AF},
    {0x3688,  0x2451},
    {0x368A,  0xD58D},
    {0x368C,  0x22EE},
    {0x368E,  0xBCAE},
    {0x3690,  0x9C0D},
    {0x3692,  0x960A},
    {0x3694,  0x3E0D},
    {0x3696,  0x01CF},
    {0x3698,  0x7E4F},
    {0x369A,  0xA02F},
    {0x369C,  0x9BD1},
    {0x369E,  0x7CEC},
    {0x36A0,  0xFEEE},
    {0x36A2,  0x0690},
    {0x36A4,  0x4CAF},
    {0x36A6,  0xB731},
    {0x36C0,  0x7E91},
    {0x36C2,  0x08F0},
    {0x36C4,  0x5D32},
    {0x36C6,  0xD011},
    {0x36C8,  0xB814},
    {0x36CA,  0x0DB2},
    {0x36CC,  0xAB4E},
    {0x36CE,  0x2F32},
    {0x36D0,  0xBF52},
    {0x36D2,  0xE553},
    {0x36D4,  0x5B91},
    {0x36D6,  0x4E50},
    {0x36D8,  0x3FB2},
    {0x36DA,  0x8AB1},
    {0x36DC,  0xD373},
    {0x36DE,  0x7FF1},
    {0x36E0,  0x9290},
    {0x36E2,  0x4012},
    {0x36E4,  0x9FD1},
    {0x36E6,  0x8074},
    {0x3700,  0x1DEE},
    {0x3702,  0x162E},
    {0x3704,  0x5C90},
    {0x3706,  0x63CE},
    {0x3708,  0x4692},
    {0x370A,  0x628F},
    {0x370C,  0x59CE},
    {0x370E,  0xA82D},
    {0x3710,  0x16AB},
    {0x3712,  0x5B12},
    {0x3714,  0x4D2D},
    {0x3716,  0x920C},
    {0x3718,  0x53EF},
    {0x371A,  0x3D51},
    {0x371C,  0x4231},
    {0x371E,  0x182F},
    {0x3720,  0x44AE},
    {0x3722,  0xDE90},
    {0x3724,  0xE150},
    {0x3726,  0x09D4},
    {0x3740,  0xDDD1},
    {0x3742,  0xA151},
    {0x3744,  0xB194},
    {0x3746,  0x50F1},
    {0x3748,  0x1056},
    {0x374A,  0xFCD1},
    {0x374C,  0x9B91},
    {0x374E,  0x8C14},
    {0x3750,  0x0AD4},
    {0x3752,  0x25F6},
    {0x3754,  0xBE71},
    {0x3756,  0x9C72},
    {0x3758,  0xCB73},
    {0x375A,  0x0BF1},
    {0x375C,  0x1395},
    {0x375E,  0xBF71},
    {0x3760,  0x03D1},
    {0x3762,  0xA514},
    {0x3764,  0x5DD2},
    {0x3766,  0x6695},
    {0x3782,  0x02E0},
    {0x3784,  0x0420},
    {0x3210,  0x01B8},

    //low_light
    {0x098E,  0x4918}, // MCU_ADDRESS [CAM1_LL_START_GAIN_METRIC]
    {0x0990,  0x0039}, // MCU_DATA_0
    {0x098E,  0x491A}, // MCU_ADDRESS [CAM1_LL_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0x6872}, // MCU_ADDRESS [PRI_A_CONFIG_LL_START_BRIGHTNESS]
    {0x0990,  0x0005}, // MCU_DATA_0
    {0x098E,  0x6874}, // MCU_ADDRESS [PRI_A_CONFIG_LL_STOP_BRIGHTNESS]
    {0x0990,  0x008C}, // MCU_DATA_0
    {0x098E,  0x4956}, // MCU_ADDRESS [CAM1_LL_DC_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x4958}, // MCU_ADDRESS [CAM1_LL_DC_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0x495A}, // MCU_ADDRESS [CAM1_LL_DC_START]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x495C}, // MCU_ADDRESS [CAM1_LL_DC_STOP]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x495E}, // MCU_ADDRESS [CAM1_LL_CDC_AGG_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x4960}, // MCU_ADDRESS [CAM1_LL_CDC_AGG_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0xC962}, // MCU_ADDRESS [CAM1_LL_CDC_AGG_START]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0xC963}, // MCU_ADDRESS [CAM1_LL_CDC_AGG_STOP]
    {0x0990,  0x0003}, // MCU_DATA_0
    {0x098E,  0x4964}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x4966}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0x4968}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_T3START]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x496A}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_T3STOP]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x496C}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_T4START]
    {0x0990,  0x0014}, // MCU_DATA_0
    {0x098E,  0x496E}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_T4STOP]
    {0x0990,  0x000C}, // MCU_DATA_0
    {0x098E,  0xC970}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_TO_START]
    {0x0990,  0x0004}, // MCU_DATA_0
    {0x098E,  0xC971}, // MCU_ADDRESS [CAM1_LL_CDC_BRIGHT_TO_STOP]
    {0x0990,  0x000F}, // MCU_DATA_0
    {0x098E,  0x4972}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x4974}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0x4976}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_T3START]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x4978}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_T3STOP]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x497A}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_T4START]
    {0x0990,  0x00C8}, // MCU_DATA_0
    {0x098E,  0x497C}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_T4STOP]
    {0x0990,  0x003C}, // MCU_DATA_0
    {0x098E,  0xC97E}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_TO_START]
    {0x0990,  0x0004}, // MCU_DATA_0
    {0x098E,  0xC97F}, // MCU_ADDRESS [CAM1_LL_CDC_DARK_TO_STOP]
    {0x0990,  0x000F}, // MCU_DATA_0
    {0x098E,  0x491C}, // MCU_ADDRESS [CAM1_LL_GRB_START_GAIN_METRIC]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x491E}, // MCU_ADDRESS [CAM1_LL_GRB_STOP_GAIN_METRIC]
    {0x0990,  0x0100}, // MCU_DATA_0
    {0x098E,  0xC920}, // MCU_ADDRESS [CAM1_LL_GRB_SLOPE_START]
    {0x0990,  0x000B}, // MCU_DATA_0
    {0x098E,  0xC921}, // MCU_ADDRESS [CAM1_LL_GRB_SLOPE_STOP]
    {0x0990,  0x002C}, // MCU_DATA_0
    {0x098E,  0xC922}, // MCU_ADDRESS [CAM1_LL_GRB_OFFSET_START]
    {0x0990,  0x0007}, // MCU_DATA_0
    {0x098E,  0xC923}, // MCU_ADDRESS [CAM1_LL_GRB_OFFSET_STOP]
    {0x0990,  0x001D}, // MCU_DATA_0
    {0x098E,  0x4926}, // MCU_ADDRESS [CAM1_LL_SFFB_START_ANALOG_GAIN]
    {0x0990,  0x0039}, // MCU_DATA_0
    {0x098E,  0x4928}, // MCU_ADDRESS [CAM1_LL_SFFB_END_ANALOG_GAIN]
    {0x0990,  0x00A0}, // MCU_DATA_0
    {0x098E,  0x492A}, // MCU_ADDRESS [CAM1_LL_SFFB_RAMP_START]
    {0x0990,  0x0082}, // MCU_DATA_0
    {0x098E,  0x492C}, // MCU_ADDRESS [CAM1_LL_SFFB_RAMP_STOP]
    {0x0990,  0x0040}, // MCU_DATA_0
    {0x098E,  0x492E}, // MCU_ADDRESS [CAM1_LL_SFFB_SLOPE_START]
    {0x0990,  0x0015}, // MCU_DATA_0
    {0x098E,  0x4930}, // MCU_ADDRESS [CAM1_LL_SFFB_SLOPE_STOP]
    {0x0990,  0x0015}, // MCU_DATA_0
    {0x098E,  0x4932}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH1START]
    {0x0990,  0x0002}, // MCU_DATA_0
    {0x098E,  0x4934}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH1STOP]
    {0x0990,  0x0004}, // MCU_DATA_0
    {0x098E,  0x4936}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH2START]
    {0x0990,  0x0008}, // MCU_DATA_0
    {0x098E,  0x4938}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH2STOP]
    {0x0990,  0x0009}, // MCU_DATA_0
    {0x098E,  0x493A}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH3START]
    {0x0990,  0x000C}, // MCU_DATA_0
    {0x098E,  0x493C}, // MCU_ADDRESS [CAM1_LL_SFFB_LOW_THRESH3STOP]
    {0x0990,  0x000D}, // MCU_DATA_0
    {0x098E,  0x493E}, // MCU_ADDRESS [CAM1_LL_SFFB_MAX_THRESH_START]
    {0x0990,  0x0015}, // MCU_DATA_0
    {0x098E,  0x4940}, // MCU_ADDRESS [CAM1_LL_SFFB_MAX_THRESH_STOP]
    {0x0990,  0x0013}, // MCU_DATA_0
    {0x098E,  0xC944}, // MCU_ADDRESS [CAM1_LL_SFFB_FLATNESS_START]
    {0x0990,  0x0023}, // MCU_DATA_0
    {0x098E,  0xC945}, // MCU_ADDRESS [CAM1_LL_SFFB_FLATNESS_STOP]
    {0x0990,  0x007F}, // MCU_DATA_0
    {0x098E,  0xC946}, // MCU_ADDRESS [CAM1_LL_SFFB_TRANSITION_START]
    {0x0990,  0x0007}, // MCU_DATA_0
    {0x098E,  0xC947}, // MCU_ADDRESS [CAM1_LL_SFFB_TRANSITION_STOP]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0xC948}, // MCU_ADDRESS [CAM1_LL_SFFB_SOBEL_FLAT_START]
    {0x0990,  0x0002}, // MCU_DATA_0
    {0x098E,  0xC949}, // MCU_ADDRESS [CAM1_LL_SFFB_SOBEL_FLAT_STOP]
    {0x0990,  0x0002}, // MCU_DATA_0
    {0x098E,  0xC94A}, // MCU_ADDRESS [CAM1_LL_SFFB_SOBEL_SHARP_START]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0xC94B}, // MCU_ADDRESS [CAM1_LL_SFFB_SOBEL_SHARP_STOP]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0xC906}, // MCU_ADDRESS [CAM1_LL_DM_EDGE_TH_START]
    {0x0990,  0x0006}, // MCU_DATA_0
    {0x098E,  0xC907}, // MCU_ADDRESS [CAM1_LL_DM_EDGE_TH_STOP]
    {0x0990,  0x0028}, // MCU_DATA_0
    {0x098E,  0xBC02}, // MCU_ADDRESS [LL_MODE]
    {0x0990,  0x0005}, // MCU_DATA_0
    {0x098E,  0xC908}, // MCU_ADDRESS [CAM1_LL_AP_KNEE_START]
    {0x0990,  0x0006}, // MCU_DATA_0
    {0x098E,  0xC909}, // MCU_ADDRESS [CAM1_LL_AP_KNEE_STOP]
    {0x0990,  0x0028}, // MCU_DATA_0
    {0x098E,  0xC90A}, // MCU_ADDRESS [CAM1_LL_AP_MANTISSA_START]
    {0x0990,  0x0007}, // MCU_DATA_0
    {0x326C,  0x0F0A}, // APERTURE_PARAMETERS_2D
    {0x098E,  0xC94C}, // MCU_ADDRESS [CAM1_LL_DELTA_GAIN]
    {0x0990,  0x0003}, // MCU_DATA_0
    {0x098E,  0xC94E}, // MCU_ADDRESS [CAM1_LL_DELTA_THRESHOLD_START]
    {0x0990,  0x003C}, // MCU_DATA_0
    {0x098E,  0xC94F}, // MCU_ADDRESS [CAM1_LL_DELTA_THRESHOLD_STOP]
    {0x0990,  0x0064}, // MCU_DATA_0
    {0x098E,  0xE877}, // MCU_ADDRESS [PRI_A_CONFIG_LL_END_SATURATION]
    {0x0990,  0x0050}, // MCU_DATA_0
    //gamma
    {0x098E,  0x3C42}, // MCU_ADDRESS [LL_START_GAMMA_FTB]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x3C44}, // MCU_ADDRESS [LL_STOP_GAMMA_FTB]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x4912}, // MCU_ADDRESS [CAM1_LL_START_GAMMA_BM]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0x4914}, // MCU_ADDRESS [CAM1_LL_MID_GAMMA_BM]
    {0x0990,  0x0001}, // MCU_DATA_0
    {0x098E,  0x4916}, // MCU_ADDRESS [CAM1_LL_STOP_GAMMA_BM]
    {0x0990,  0x0037}, // MCU_DATA_0
    {0x098E,  0xBC09}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_0]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0xBC0A}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_1]
    {0x0990,  0x0011}, // MCU_DATA_0
    {0x098E,  0xBC0B}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_2]
    {0x0990,  0x0023}, // MCU_DATA_0
    {0x098E,  0xBC0C}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_3]
    {0x0990,  0x003F}, // MCU_DATA_0
    {0x098E,  0xBC0D}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_4]
    {0x0990,  0x0067}, // MCU_DATA_0
    {0x098E,  0xBC0E}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_5]
    {0x0990,  0x0085}, // MCU_DATA_0
    {0x098E,  0xBC0F}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_6]
    {0x0990,  0x009B}, // MCU_DATA_0
    {0x098E,  0xBC10}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_7]
    {0x0990,  0x00AD}, // MCU_DATA_0
    {0x098E,  0xBC11}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_8]
    {0x0990,  0x00BB}, // MCU_DATA_0
    {0x098E,  0xBC12}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_9]
    {0x0990,  0x00C7}, // MCU_DATA_0
    {0x098E,  0xBC13}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_10]
    {0x0990,  0x00D1}, // MCU_DATA_0
    {0x098E,  0xBC14}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_11]
    {0x0990,  0x00DA}, // MCU_DATA_0
    {0x098E,  0xBC15}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_12]
    {0x0990,  0x00E1}, // MCU_DATA_0
    {0x098E,  0xBC16}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_13]
    {0x0990,  0x00E8}, // MCU_DATA_0
    {0x098E,  0xBC17}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_14]
    {0x0990,  0x00EE}, // MCU_DATA_0
    {0x098E,  0xBC18}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_15]
    {0x0990,  0x00F3}, // MCU_DATA_0
    {0x098E,  0xBC19}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_16]
    {0x0990,  0x00F7}, // MCU_DATA_0
    {0x098E,  0xBC1A}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_17]
    {0x0990,  0x00FB}, // MCU_DATA_0
    {0x098E,  0xBC1B}, // MCU_ADDRESS [LL_GAMMA_CONTRAST_CURVE_18]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0xBC1C}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_0]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0xBC1D}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_1]
    {0x0990,  0x0011}, // MCU_DATA_0
    {0x098E,  0xBC1E}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_2]
    {0x0990,  0x0023}, // MCU_DATA_0
    {0x098E,  0xBC1F}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_3]
    {0x0990,  0x003F}, // MCU_DATA_0
    {0x098E,  0xBC20}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_4]
    {0x0990,  0x0067}, // MCU_DATA_0
    {0x098E,  0xBC21}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_5]
    {0x0990,  0x0085}, // MCU_DATA_0
    {0x098E,  0xBC22}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_6]
    {0x0990,  0x009B}, // MCU_DATA_0
    {0x098E,  0xBC23}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_7]
    {0x0990,  0x00AD}, // MCU_DATA_0
    {0x098E,  0xBC24}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_8]
    {0x0990,  0x00BB}, // MCU_DATA_0
    {0x098E,  0xBC25}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_9]
    {0x0990,  0x00C7}, // MCU_DATA_0
    {0x098E,  0xBC26}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_10]
    {0x0990,  0x00D1}, // MCU_DATA_0
    {0x098E,  0xBC27}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_11]
    {0x0990,  0x00DA}, // MCU_DATA_0
    {0x098E,  0xBC28}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_12]
    {0x0990,  0x00E1}, // MCU_DATA_0
    {0x098E,  0xBC29}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_13]
    {0x0990,  0x00E8}, // MCU_DATA_0
    {0x098E,  0xBC2A}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_14]
    {0x0990,  0x00EE}, // MCU_DATA_0
    {0x098E,  0xBC2B}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_15]
    {0x0990,  0x00F3}, // MCU_DATA_0
    {0x098E,  0xBC2C}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_16]
    {0x0990,  0x00F7}, // MCU_DATA_0
    {0x098E,  0xBC2D}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_17]
    {0x0990,  0x00FB}, // MCU_DATA_0
    {0x098E,  0xBC2E}, // MCU_ADDRESS [LL_GAMMA_NEUTRAL_CURVE_18]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0xBC2F}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_0]
    {0x0990,  0x0000}, // MCU_DATA_0
    {0x098E,  0xBC30}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_1]
    {0x0990,  0x0017}, // MCU_DATA_0
    {0x098E,  0xBC31}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_2]
    {0x0990,  0x0020}, // MCU_DATA_0
    {0x098E,  0xBC32}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_3]
    {0x0990,  0x0032}, // MCU_DATA_0
    {0x098E,  0xBC33}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_4]
    {0x0990,  0x005A}, // MCU_DATA_0
    {0x098E,  0xBC34}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_5]
    {0x0990,  0x0078}, // MCU_DATA_0
    {0x098E,  0xBC35}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_6]
    {0x0990,  0x0089}, // MCU_DATA_0
    {0x098E,  0xBC36}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_7]
    {0x0990,  0x0098}, // MCU_DATA_0
    {0x098E,  0xBC37}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_8]
    {0x0990,  0x00A6}, // MCU_DATA_0
    {0x098E,  0xBC38}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_9]
    {0x0990,  0x00B4}, // MCU_DATA_0
    {0x098E,  0xBC39}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_10]
    {0x0990,  0x00C3}, // MCU_DATA_0
    {0x098E,  0xBC3A}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_11]
    {0x0990,  0x00CB}, // MCU_DATA_0
    {0x098E,  0xBC3B}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_12]
    {0x0990,  0x00D0}, // MCU_DATA_0
    {0x098E,  0xBC3C}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_13]
    {0x0990,  0x00D4}, // MCU_DATA_0
    {0x098E,  0xBC3D}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_14]
    {0x0990,  0x00DC}, // MCU_DATA_0
    {0x098E,  0xBC3E}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_15]
    {0x0990,  0x00E4}, // MCU_DATA_0
    {0x098E,  0xBC3F}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_16]
    {0x0990,  0x00EA}, // MCU_DATA_0
    {0x098E,  0xBC40}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_17]
    {0x0990,  0x00F5}, // MCU_DATA_0
    {0x098E,  0xBC41}, // MCU_ADDRESS [LL_GAMMA_NRCURVE_18]
    {0x0990,  0x00FF}, // MCU_DATA_0
    {0x098E,  0x3C42}, // MCU_ADDRESS [LL_START_GAMMA_FTB]
    {0x0990,  0x0032}, // MCU_DATA_0
    {0x098E,  0x3C44}, // MCU_ADDRESS [LL_STOP_GAMMA_FTB]
    {0x0990,  0x0000}, // MCU_DATA_0

    //A CCM
    {0x0018,  0x002A}, // STANDBY_CONTROL_AND_STATUS
    {0x098E,  0xAC02}, // MCU_ADDRESS [AWB_MODE]
    {0x0990,  0x0006}, // MCU_DATA_0
    {0x098E,  0x2800}, // MCU_ADDRESS [AE_TRACK_STATUS]
    {0x0990,  0x001C}, // MCU_DATA_0
    {0x098E,  0x8400}, // MCU_ADDRESS
    {0x0990,  0x0006}, // MCU_DATA_0
};

const static struct mt9t113_i2c_reg_conf mt9t113_antibanding_off_reg_config[] =
{
    {0x098E, 0x2003}, 	// MCU_ADDRESS [FD_ALGO]
    {0x0990, 0x0003}, 	// MCU_DATA_0
    {0x098E, 0xA005}, 	// MCU_ADDRESS [FD_FDPERIOD_SELECT]
    {0x0990, 0x0000}, 	// MCU_DATA_0
    {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
    {0x0990, 0x0005}, 	// MCU_DATA_0
};
const static struct mt9t113_i2c_reg_conf mt9t113_antibanding_50hz_reg_config[] =
{
    {0x098E, 0x2003},   // MCU_ADDRESS [FD_ALGO]
    {0x0990, 0x0002},   // MCU_DATA_0
    {0x098E, 0xA005},   // MCU_ADDRESS [FD_FDPERIOD_SELECT]
    {0x0990, 0x0001},   // MCU_DATA_0
};
const static struct mt9t113_i2c_reg_conf mt9t113_antibanding_60hz_reg_config[] =
{
    {0x098E, 0x2003},   // MCU_ADDRESS [FD_ALGO]
    {0x0990, 0x0002},   // MCU_DATA_0
    {0x098E, 0xA005},   // MCU_ADDRESS [FD_FDPERIOD_SELECT]
    {0x0990, 0x0000},   // MCU_DATA_0
};

static int mt9t113_i2c_rxdata(unsigned short saddr,
                              unsigned char *rxdata, int length)
{
    struct i2c_msg msgs[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = 2,
            .buf = rxdata,
        },
        {
            .addr  = saddr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = rxdata,
        },
    };

    if (i2c_transfer(mt9t113_client->adapter, msgs, 2) < 0)
    {
        CDBG("mt9t113_i2c_rxdata failed!\n");
        return -EIO;
    }

    return 0;
}

static int32_t mt9t113_i2c_read_w(unsigned short raddr, unsigned short *rdata)
{
    int32_t rc = 0;
    unsigned char buf[4];

    if (!rdata)
    {
        return -EIO;
    }

    memset(buf, 0, sizeof(buf));

    buf[0] = (raddr & 0xFF00) >> 8;
    buf[1] = (raddr & 0x00FF);

    rc = mt9t113_i2c_rxdata(mt9t113_client->addr, buf, 2);
    if (rc < 0)
    {
        return rc;
    }

    *rdata = buf[0] << 8 | buf[1];

    if (rc < 0)
    {
        CDBG("mt9t113_i2c_read failed!\n");
    }

    return rc;
}

static int32_t mt9t113_i2c_txdata(unsigned short saddr,
                                  unsigned char *txdata, int length)
{
    int i = 0;
    struct i2c_msg msg[] =
    {
        {
            .addr  = saddr,
            .flags = 0,
            .len = length,
            .buf = txdata,
        },
    };

    for (i = 0; i < 10; i++)
    {
        if (i2c_transfer(mt9t113_client->adapter, msg, 1) >= 0)
        {
            return 0;
        }
        else
        {
            CDBG("mt9t113_i2c_txdata faild(%d)\n", i);
        }
    }

    CDBG("mt9t113_i2c_txdata faild\n");
    return -EIO;
}

static int32_t mt9t113_i2c_write_w(unsigned short waddr, unsigned short wdata)
{
    int32_t rc = -EFAULT;
    unsigned char buf[4];

    memset(buf, 0, sizeof(buf));
    buf[0] = (waddr & 0xFF00) >> 8;
    buf[1] = (waddr & 0x00FF);
    buf[2] = (wdata & 0xFF00) >> 8;
    buf[3] = (wdata & 0x00FF);

    rc = mt9t113_i2c_txdata(mt9t113_client->addr, buf, 4);

    if (rc < 0)
    {
        CDBG("i2c_write_w failed, addr = 0x%x, val = 0x%x!\n",
             waddr, wdata);
    }

    return rc;
}

static int32_t mt9t113_i2c_write_w_table(struct mt9t113_i2c_reg_conf const *reg_conf_tbl,
                                         int                                num_of_items_in_table)
{
    int i;
    int32_t rc = -EFAULT;

    for (i = 0; i < num_of_items_in_table; i++)
    {
        rc = mt9t113_i2c_write_w(reg_conf_tbl->waddr, reg_conf_tbl->wdata);
        if (rc < 0)
        {
            break;
        }

        reg_conf_tbl++;
    }

    return rc;
}

int32_t mt9t113_set_fps(struct fps_cfg    *fps)
{
    /* input is new fps in Q8 format */
    int32_t rc = 0;

    CDBG("mt9t113_set_fps\n");
    return rc;
}

int32_t mt9t113_write_exp_gain(uint16_t gain, uint32_t line)
{
    CDBG("mt9t113_write_exp_gain\n");
    return 0;
}

int32_t mt9t113_set_pict_exp_gain(uint16_t gain, uint32_t line)
{
    int32_t rc = 0;

    CDBG("mt9t113_set_pict_exp_gain\n");

    mdelay(10);

    /* camera_timed_wait(snapshot_wait*exposure_ratio); */
    return rc;
}
static int32_t mt9t113_set_mirror_mode(void)
{
    int32_t rc = 0;

    const static struct mt9t113_i2c_reg_conf mt9t113_mirror_mode_reg_config[] =
    {
        {0x098E, 0x4810}, 	// MCU_ADDRESS [CAM1_CTX_A_READ_MODE]
        {0x0990, 0x046C}, 	// MCU_DATA_0
        {0x098E, 0x483D}, 	// MCU_ADDRESS [CAM1_CTX_B_READ_MODE]
        {0x0990, 0x0024}, 	// MCU_DATA_0
        {0x098E, 0x8400}, 	// MCU_ADDRESS [SEQ_CMD]
        {0x0990, 0x0006}, 	// MCU_DATA_0      
    };

    if (machine_is_msm7x27_m650()) 
    {
        rc = mt9t113_i2c_write_w_table(mt9t113_mirror_mode_reg_config,
                    MT9T113_ARRAY_SIZE(mt9t113_mirror_mode_reg_config));
    }

    return rc;
}

int32_t mt9t113_setting(enum mt9t113_reg_update_t rupdate,
                        enum mt9t113_setting_t    rt)
{
    int32_t rc = 0;

    if ((rupdate == last_rupdate) && (rt == last_rt))
    {
        CDBG("mt9t113_setting exit\n");
        return rc;
    }

    CDBG("mt9t113_setting in rupdate=%d,rt=%d\n", rupdate, rt);
    switch (rupdate)
    {
    case UPDATE_PERIODIC:

        if (rt == RES_PREVIEW)
        {
            /* Write preview register */
            rc = mt9t113_i2c_write_w_table(mt9t113_preview_reg_config,
                                           MT9T113_ARRAY_SIZE(mt9t113_preview_reg_config));

            mdelay(10);
            break;
        }
        else
        {
            /* Write snapshot register */

            rc = mt9t113_i2c_write_w_table(mt9t113_snapshot_reg_config,
                                           MT9T113_ARRAY_SIZE(mt9t113_snapshot_reg_config));
            mdelay(10);
        }

        break;

    case REG_INIT:
        rc = mt9t113_i2c_write_w_table(mt9t113_init_reg_config1,
                                       MT9T113_ARRAY_SIZE(mt9t113_init_reg_config1));
        if (rc < 0)
        {
            CDBG("mt9t113_init_reg_config1  error");
        }
        else
        {
            CDBG("mt9t113_init_reg_config1 succeed");
        }

        msleep(50);

        rc = mt9t113_i2c_write_w_table(mt9t113_init_reg_config2,
                                       MT9T113_ARRAY_SIZE(mt9t113_init_reg_config2));
        if (rc < 0)
        {
            CDBG("mt9t113_init_reg_config2  error");
        }
        else
        {
            CDBG("mt9t113_init_reg_config2 succeed");
        }

        msleep(150);

        /* Write init sensor register */
        if (MODEL_SUNNY == mt9t113_model_id)
        {
            rc = mt9t113_i2c_write_w_table(mt9t113_init_reg_config3_sunny,
                                           MT9T113_ARRAY_SIZE(mt9t113_init_reg_config3_sunny));
            if (rc < 0)
            {
                CDBG("mt9t113_init_reg_config3_sunny  error");
            }
            else
            {
                CDBG("mt9t113_init_reg_config3_sunny succeed");
            }
        }
        else
        {
            rc = mt9t113_i2c_write_w_table(mt9t113_init_reg_config3_byd,
                                           MT9T113_ARRAY_SIZE(mt9t113_init_reg_config3_byd));
            if (rc < 0)
            {
                CDBG("mt9t113_init_reg_config3_byd  error");
            }
            else
            {
                CDBG("mt9t113_init_reg_config3_byd succeed");
            }
        }
        rc = mt9t113_set_mirror_mode();
        if (rc < 0)
            {
                CDBG("mt9t113_set_mirror_mode  error");
            }
            else
            {
                CDBG("mt9t113_set_mirror_mode succeed");
            }
        msleep(200);

        break;

    default:

        rc = -EFAULT;

        break;
    }/*switch (rupdate) */
    if (rc == 0)
    {
        last_rupdate = rupdate;
        last_rt = rt;
    }

    return rc;
}

int32_t mt9t113_video_config(int mode, int res)
{
    int32_t rc;

    switch (res)
    {
    case QTR_SIZE:

        rc = mt9t113_setting(UPDATE_PERIODIC, RES_PREVIEW);

        if (rc < 0)
        {
            return rc;
        }

        break;

    case FULL_SIZE:

        rc = mt9t113_setting(UPDATE_PERIODIC, RES_CAPTURE);

        if (rc < 0)
        {
            return rc;
        }

        break;

    default:

        return 0;
    } /*switch */

    mt9t113_ctrl->prev_res   = res;
    mt9t113_ctrl->curr_res   = res;
    mt9t113_ctrl->sensormode = mode;

    return rc;
}

int32_t mt9t113_snapshot_config(int mode)
{
    int32_t rc = 0;

    CDBG("mt9t113_snapshot_config in\n");
    rc = mt9t113_setting(UPDATE_PERIODIC, RES_CAPTURE);
    msleep(50);
    if (rc < 0)
    {
        return rc;
    }

    mt9t113_ctrl->curr_res = mt9t113_ctrl->pict_res;
    mt9t113_ctrl->sensormode = mode;

    return rc;
}

int32_t mt9t113_power_down(void)
{
    int32_t rc = 0;

    mdelay(5);
    return rc;
}

int32_t mt9t113_move_focus(int direction, int32_t num_steps)
{
    return 0;
}

static int mt9t113_sensor_init_done(const struct msm_camera_sensor_info *data)
{
    if (false == mt9t113_init_flag)
    {
        gpio_direction_output(data->sensor_reset, 0);
        gpio_free(data->sensor_reset);
    }
    gpio_direction_output(data->sensor_pwd, 1);
    gpio_free(data->sensor_pwd);

    if (data->vreg_disable_func)
    {
        data->vreg_disable_func(data->sensor_vreg, data->vreg_num);
    }

    last_rupdate = -1;
    last_rt = -1;

    return 0;
}

static int mt9t113_probe_init_sensor(const struct msm_camera_sensor_info *data)
{
    int rc;
    unsigned short chipid;

    /*pull down power down */
    rc = gpio_request(data->sensor_pwd, "mt9t113");
    if (!rc || (rc == -EBUSY))
    {
        CDBG("rc = gpio_request data->sensor_pwd, mt9t113 ");
        gpio_direction_output(data->sensor_pwd, 1);
    }
    else
    {
        goto init_probe_fail;
    }
    if(false == mt9t113_init_flag)
    {
        rc = gpio_request(data->sensor_reset, "mt9t113");
        if (!rc)
        {
            CDBG("rc = gpio_request data->sensor_reset, mt9t113 ");
            rc = gpio_direction_output(data->sensor_reset, 0);
        }
        else
        {
            CDBG("%s %d  gpio request error\n", __FUNCTION__, __LINE__);
            goto init_probe_fail;
        }
    }
    mdelay(5);

    if (data->vreg_enable_func)
    {
        rc = data->vreg_enable_func(data->sensor_vreg, data->vreg_num);

        if (rc < 0)
        {
            goto init_probe_fail;
        }
    }

    msleep(20);

    if ((data->master_init_control_slave == NULL) || (data->master_init_control_slave(data) == 0))
    {
        CDBG("mt9t113 hardware reset!!!");

        rc = gpio_direction_output(data->sensor_pwd, 0);

        if (rc < 0)
        {
            goto init_probe_fail;
        }

        msleep(20);

        /* Set the sensor reset when camera is not init. */

        /*hardware reset*/
        if(false == mt9t113_init_flag)
        {
            rc = gpio_direction_output(data->sensor_reset, 1);

            if (rc < 0)
            {
                goto init_probe_fail;
            }
        
            msleep(20);
        }
    }

    /* Read chip ID when camera is not init. */  
    if(false == mt9t113_init_flag)
    {
        rc = mt9t113_i2c_read_w(MT9T113_REG_CHIP_ID, &chipid);
        if (rc < 0)
        {
            CDBG("mt9t113_i2c_read_w Model_ID failed!! rc=%d", rc);
            goto init_probe_fail;
        }

        CDBG("mt9t113 chipid = 0x%x\n", chipid);

        /* 4. Compare sensor ID to MT9T113 ID: */

        if (chipid != MT9T113_CHIP_ID)

        {
            CDBG("mt9t113 Model_ID  error!!");

            rc = -ENODEV;

            goto init_probe_fail;
        }

        /* Sensor init at probe,not in open function */
        rc = mt9t113_setting(REG_INIT, RES_PREVIEW);
        
        if (rc < 0)
        {
            goto init_probe_fail;
        }
        /* 3. Read sensor Model ID: */
        if (mt9t113_i2c_read_w(MT9T113_REG_MODEL_ID, &mt9t113_model_id) < 0)
        {
            CDBG("mt9t113_model_id READ ERROR = 0x%x!!\n", mt9t113_model_id);
        }
        
        mt9t113_model_id = mt9t113_model_id & 0x03;
        CDBG("%s: model is %d init sensor!", __FUNCTION__, mt9t113_model_id);

        if (MODEL_BYD == mt9t113_model_id)
        {
            strncpy((char *)data->sensor_name, "23060043SF-MT-B", strlen("23060043SF-MT-B"));
        }
        else if (MODEL_SUNNY == mt9t113_model_id)
        {
            strncpy((char *)data->sensor_name, "23060051FF-MT-S", strlen("23060051FF-MT-S"));
        }
        else
        {
            strncpy((char *)data->sensor_name, "23060043SF-MT-B", strlen("23060043SF-MT-B"));
            CDBG("sensor name is read errir!");
        }

        CDBG("sensor name is %s.", data->sensor_name);
    }
    goto init_probe_done;
init_probe_fail:
    CDBG("%s----%d: init fail\n", __FUNCTION__, __LINE__);
    mt9t113_sensor_init_done(data);
init_probe_done:
    return rc;
}

int mt9t113_sensor_open_init(const struct msm_camera_sensor_info *data)
{
    int32_t rc;

    mt9t113_ctrl = kzalloc(sizeof(struct mt9t113_ctrl_t), GFP_KERNEL);
    if (!mt9t113_ctrl)
    {
        CDBG("mt9t113_sensor_open_init failed!\n");

        rc = -ENOMEM;

        goto init_done;
    }

    mt9t113_ctrl->fps_divider = 1 * 0x00000400;
    mt9t113_ctrl->pict_fps_divider = 1 * 0x00000400;
    mt9t113_ctrl->set_test = TEST_OFF;
    mt9t113_ctrl->prev_res = QTR_SIZE;
    mt9t113_ctrl->pict_res = FULL_SIZE;

    if (data)
    {
        mt9t113_ctrl->sensordata = data;
    }/*enable mclk first */
    msm_camio_clk_rate_set(MT9T113_DEFAULT_CLOCK_RATE);
    msleep(20);

    msm_camio_camif_pad_reg_reset();
    msleep(20);

    rc = mt9t113_probe_init_sensor(data);
    if (rc < 0)
    {
        goto init_fail;
    }

    rc = mt9t113_setting(REG_INIT, RES_PREVIEW);
    if (rc < 0)
    {
        goto init_fail;
    }
    else
    {
        goto init_done;
    }

init_fail:
    kfree(mt9t113_ctrl);
    CDBG("%s----%d:init fail\n", __FUNCTION__, __LINE__);
init_done:
    CDBG("%s----%d:init done\n", __FUNCTION__, __LINE__);
    return rc;
}

int mt9t113_init_client(struct i2c_client *client)
{
    /*Initialize the MSM_CAMI2C Chip */
    init_waitqueue_head(&mt9t113_wait_queue);
    return 0;
}

int32_t mt9t113_set_sensor_mode(int mode, int res)
{
    int32_t rc = 0;

    CDBG("%s----%d\n", __FUNCTION__, __LINE__);
    switch (mode)
    {
    case SENSOR_PREVIEW_MODE:

        CDBG("SENSOR_PREVIEW_MODE,res=%d\n", res);

        rc = mt9t113_video_config(mode, res);

        break;

    case SENSOR_SNAPSHOT_MODE:
    case SENSOR_RAW_SNAPSHOT_MODE:

        CDBG("SENSOR_SNAPSHOT_MODE\n");

        rc = mt9t113_snapshot_config(mode);

        break;

    default:

        rc = -EINVAL;

        break;
    }

    return rc;
}

static long mt9t113_set_effect(int mode, int effect)
{
    struct mt9t113_i2c_reg_conf const *reg_conf_tbl = NULL;
    int num_of_items_in_table = 0;
    long rc = 0;

    CDBG("%s----%d\n", __FUNCTION__, __LINE__);
    switch (effect)
    {
    case CAMERA_EFFECT_OFF:

        reg_conf_tbl = mt9t113_effect_off_reg_config;

        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_effect_off_reg_config);

        break;

    case CAMERA_EFFECT_MONO:

        reg_conf_tbl = mt9t113_effect_Mono_reg_config;

        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_effect_Mono_reg_config);

        break;

    case CAMERA_EFFECT_NEGATIVE:

        reg_conf_tbl = mt9t113_effect_neg_reg_config;

        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_effect_neg_reg_config);

        break;

    case CAMERA_EFFECT_SEPIA:

        reg_conf_tbl = mt9t113_effect_sepia_reg_config;

        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_effect_sepia_reg_config);

        break;

    case CAMERA_EFFECT_AQUA:

        reg_conf_tbl = mt9t113_effect_aqua_reg_config;

        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_effect_aqua_reg_config);

        break;

    case CAMERA_EFFECT_WHITEBOARD:

        break;

    case CAMERA_EFFECT_BLACKBOARD:

        break;

    default:
        return 0;
    }

    rc = mt9t113_i2c_write_w_table(reg_conf_tbl, num_of_items_in_table);
    return rc;
}

static long mt9t113_set_wb(int wb)
{
    struct mt9t113_i2c_reg_conf const *reg_conf_tbl = NULL;
    int num_of_items_in_table = 0;
    long rc = 0;

    CDBG("%s----%d in,wb=%d\n", __FUNCTION__, __LINE__, wb);
    switch (wb)
    {
    case CAMERA_WB_AUTO:
        reg_conf_tbl = mt9t113_wb_auto_reg_config;
        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_wb_auto_reg_config);
        break;

    case CAMERA_WB_INCANDESCENT:
        reg_conf_tbl = mt9t113_wb_incabd_reg_config;
        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_wb_incabd_reg_config);
        break;

    case CAMERA_WB_CUSTOM:
        break;
    case CAMERA_WB_FLUORESCENT:
        reg_conf_tbl = mt9t113_wb_fluore_reg_config;
        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_wb_fluore_reg_config);
        break;

    case CAMERA_WB_DAYLIGHT:
        reg_conf_tbl = mt9t113_wb_daylight_reg_config;
        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_wb_daylight_reg_config);
        break;

    case CAMERA_WB_CLOUDY_DAYLIGHT:
        reg_conf_tbl = mt9t113_wb_clouday_reg_config;
        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_wb_clouday_reg_config);
        break;

    case CAMERA_WB_TWILIGHT:
        break;

    case CAMERA_WB_SHADE:
        break;

    default:
        return 0;
    }

    rc = mt9t113_i2c_write_w_table(reg_conf_tbl, num_of_items_in_table);

    return rc;
}

static long mt9t113_set_antibanding(int antibanding)
{
    struct mt9t113_i2c_reg_conf const *reg_conf_tbl = NULL;
    int num_of_items_in_table = 0;
    long rc = 0;

    CDBG("%s----%d\n", __FUNCTION__, __LINE__);
    switch (antibanding)
    {
    case CAMERA_ANTIBANDING_OFF:
        reg_conf_tbl = mt9t113_antibanding_off_reg_config;
        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_antibanding_off_reg_config);
        mdelay(5);
        break;

    case CAMERA_ANTIBANDING_60HZ:
        reg_conf_tbl = mt9t113_antibanding_60hz_reg_config;
        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_antibanding_60hz_reg_config);
        mdelay(5);
        break;

    case CAMERA_ANTIBANDING_50HZ:
        reg_conf_tbl = mt9t113_antibanding_50hz_reg_config;
        num_of_items_in_table = MT9T113_ARRAY_SIZE(mt9t113_antibanding_50hz_reg_config);
        mdelay(5);

        break;

    default:
        return 0;
    }

    rc = mt9t113_i2c_write_w_table(reg_conf_tbl, num_of_items_in_table);
    return rc;
}

int mt9t113_sensor_config(void __user *argp)
{
    struct sensor_cfg_data cdata;
    long rc = 0;

    CDBG("%s----%d\n", __FUNCTION__, __LINE__);

    if (copy_from_user(&cdata,
                       (void *)argp,
                       sizeof(struct sensor_cfg_data)))
    {
        return -EFAULT;
    }

    down(&mt9t113_sem);

    CDBG("mt9t113_sensor_config: cfgtype = %d\n",
         cdata.cfgtype);
    switch (cdata.cfgtype)
    {
    case CFG_GET_PICT_FPS:
        break;

    case CFG_GET_PREV_L_PF:
        break;

    case CFG_GET_PREV_P_PL:
        break;

    case CFG_GET_PICT_L_PF:
        break;

    case CFG_GET_PICT_P_PL:
        break;

    case CFG_GET_PICT_MAX_EXP_LC:
        break;

    case CFG_SET_FPS:
    case CFG_SET_PICT_FPS:

        rc = mt9t113_set_fps(&(cdata.cfg.fps));
        break;

    case CFG_SET_EXP_GAIN:

        rc = mt9t113_write_exp_gain(
            cdata.cfg.exp_gain.gain,
            cdata.cfg.exp_gain.line);
        break;

    case CFG_SET_PICT_EXP_GAIN:

        rc = mt9t113_set_pict_exp_gain(
            cdata.cfg.exp_gain.gain,
            cdata.cfg.exp_gain.line);
        break;

    case CFG_SET_MODE:
        rc = mt9t113_set_sensor_mode(cdata.mode,
                                     cdata.rs);
        break;

    case CFG_PWR_DOWN:

        rc = mt9t113_power_down();
        break;

    case CFG_MOVE_FOCUS:

        rc = mt9t113_move_focus(
            cdata.cfg.focus.dir,
            cdata.cfg.focus.steps);
        break;

    case CFG_SET_DEFAULT_FOCUS:
        break;

    case CFG_SET_EFFECT:

        rc = mt9t113_set_effect(cdata.mode,
                                cdata.cfg.effect);
        break;

    case CFG_SET_WB:

        rc = mt9t113_set_wb(cdata.cfg.effect);
        break;

    case CFG_SET_ANTIBANDING:

        rc = mt9t113_set_antibanding(cdata.cfg.effect);
        break;

    case CFG_MAX:
        if (copy_to_user((void *)(cdata.cfg.pict_max_exp_lc),
                         mt9t113_supported_effect,
                         MT9T113_ARRAY_SIZE(mt9t113_supported_effect)))
        {
            CDBG("copy mt9t113_supported_effect to user fail\n");
            rc = -EFAULT;
        }
        else
        {
            rc = 0;
        }

        break;

    default:
        rc = -EFAULT;
        break;
    }

    up(&mt9t113_sem);

    return rc;
}

int mt9t113_sensor_release(void)
{
    int rc = -EBADF;

    down(&mt9t113_sem);

    mt9t113_power_down();
    mt9t113_sensor_init_done(mt9t113_ctrl->sensordata);
    /* For go to sleep mode, follow the datasheet */
    msleep(150);
    kfree(mt9t113_ctrl);
    up(&mt9t113_sem);
    return rc;
}

static int mt9t113_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{
    int rc = 0;

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
    {
        rc = -ENOTSUPP;
        goto probe_failure;
    }

    mt9t113sensorw =
        kzalloc(sizeof(struct mt9t113_work_t), GFP_KERNEL);

    if (!mt9t113sensorw)
    {
        rc = -ENOMEM;
        goto probe_failure;
    }

    i2c_set_clientdata(client, mt9t113sensorw);
    mt9t113_init_client(client);
    mt9t113_client = client;
    msleep(50);

    CDBG("i2c probe ok\n");
    return 0;

probe_failure:
    kfree(mt9t113sensorw);
    mt9t113sensorw = NULL;
    pr_err("i2c probe failure %d\n", rc);
    return rc;
}

static const struct i2c_device_id mt9t113_i2c_id[] =
{
    { "mt9t113", 0},
    { }
};

static struct i2c_driver mt9t113_i2c_driver =
{
    .id_table = mt9t113_i2c_id,
    .probe    = mt9t113_i2c_probe,
    .remove   = __exit_p(mt9t113_i2c_remove),
    .driver   = {
        .name = "mt9t113",
    },
};

static int mt9t113_sensor_probe(const struct msm_camera_sensor_info *info,
                                struct msm_sensor_ctrl *s)
{
    /* We expect this driver to match with the i2c device registered
     * in the board file immediately. */
    int rc = i2c_add_driver(&mt9t113_i2c_driver);

    if ((rc < 0) || (mt9t113_client == NULL))
    {
        rc = -ENOTSUPP;
        goto probe_done;
    }

    /* enable mclk first */
    msm_camio_clk_rate_set(MT9T113_DEFAULT_CLOCK_RATE);
    msleep(20);

    rc = mt9t113_probe_init_sensor(info);
    if (rc < 0)
    {
        CDBG("camera sensor mt9t113 probe is failed!!!\n");
        i2c_del_driver(&mt9t113_i2c_driver);
        goto probe_done;
    }
    else
    {
        mt9t113_init_flag = true;
        CDBG("camera sensor mt9t113 probe is succeed!!!\n");
    }

#ifdef CONFIG_HUAWEI_HW_DEV_DCT
    /* detect current device successful, set the flag as present */
    set_hw_dev_flag(DEV_I2C_CAMERA_MAIN);
#endif

    s->s_init = mt9t113_sensor_open_init;
    s->s_release = mt9t113_sensor_release;
    s->s_config = mt9t113_sensor_config;
    mt9t113_sensor_init_done(info);
    msleep(150);
    set_camera_support(true);

probe_done:
    return rc;
}

static int __mt9t113_probe(struct platform_device *pdev)
{
    return msm_camera_drv_start(pdev, mt9t113_sensor_probe);
}

static struct platform_driver msm_camera_driver =
{
    .probe     = __mt9t113_probe,
    .driver    = {
        .name  = "msm_camera_mt9t113",
        .owner = THIS_MODULE,
    },
};

static int __init mt9t113_init(void)
{
    return platform_driver_register(&msm_camera_driver);
}

fs_initcall(mt9t113_init);

