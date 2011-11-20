#ifndef ANDMSM_SHARE_H
#define ANDMSM_SHARE_H
/*===========================================================================

                        INCLUDE FILES FOR MODULE

===========================================================================*/
/*===========================================================================

                        DEFINITIONS

===========================================================================*/

typedef unsigned long uint32;
typedef unsigned char byte;
typedef unsigned char boolean;

#define ANDMSM_PACKET_HEAD_LEN                  12

#define VIRTMSM_SHAREMEM_MAGIC0                 0x51525354
#define VIRTMSM_SHAREMEM_MAGIC1                 0x61626364

#define ANDMSM_SIO_MSG_BUFFER_LEN               4096

#define ANDMSM_SIO_MSG_HEAD_LEN                 8

#define ANDMSM_SIO_MSM_CONTENT_LEN              (ANDMSM_SIO_MSG_BUFFER_LEN - ANDMSM_SIO_MSG_HEAD_LEN)

#define LCD_WIDTH                               240
#define LCD_HEIGHT                              320

#define ANDMSM_CMD_MASK                         0x0000FFFF
#define ANDMSM_CMD_RESPONSE_MASK                0x80000000

struct SHARE_MEM_STRU
{
    uint32  ulMagicNum[2];
    uint32  ulFlag;
    uint32  ulBulkAddr;
    uint32  ulBulkSize;
    uint32  ulReserved[7];
    uint32  ulBlockNSize[16];
};

struct MSG_Q_STRU
{
    uint32  ulRD;
    uint32  ulWR;
    uint32  ulContent[ANDMSM_SIO_MSM_CONTENT_LEN / 4];
};

struct DISP_BUF_STRU
{
    byte    aBuf_1[LCD_HEIGHT * LCD_WIDTH * 2];
    byte    aBuf_2[LCD_HEIGHT * LCD_WIDTH * 2];
};

typedef struct
{
    uint32 total_length;    
    uint32 type;            
    uint32 port_num;        
    uint32 cmd_length;     
    uint32 data;            
}andmsm_cmd_stru;

struct MSG2_Q_STRU
{
  uint32  ulRD;
  uint32  ulWR;
  uint32  ulTotalLen;
  uint32  ulContentOffset;
};

#define MSG2_INIT_QUEUE(p, addr, len) \
  do\
  {\
    (p) = (void *)(addr);\
    (p)->ulRD = 0;\
    (p)->ulWR = 0;\
    (p)->ulTotalLen = (len) - sizeof(struct MSG2_Q_STRU);\
  }while(0)


/* define the exchanged data type between android and modem,
   corresponding to the 2nd field(type) in data structure.
*/
typedef enum
{
  VIRTMSM_MSG_SIO_WRITE = 0,
  VIRTMSM_MSG_SIO_READ,  
  VIRTMSM_MSG_SIO_CMD,
  VIRTMSM_MSG_MAX
}andmsm_cmd_type_enum;

/*  define the port_num, 
    corresponding to the 3rd field (port_num) in data structure 
*/
typedef enum
{
    ANDMSM_SIO_PORT_SRV_CHN_0 = 0,
    ANDMSM_SIO_PORT_SRV_CHN_1,
    ANDMSM_SIO_PORT_SRV_CHN_2,
    ANDMSM_SIO_PORT_SRV_CHN_3,
    ANDMSM_SIO_PORT_SRV_CHN_4,
    ANDMSM_SIO_PORT_SRV_CHN_5,
    ANDMSM_SIO_PORT_SRV_CHN_6,
    ANDMSM_SIO_PORT_SRV_CHN_7,
    ANDMSM_SIO_PORT_SRV_CHN_8,
    ANDMSM_SIO_PORT_SRV_CHN_9,
    ANDMSM_SIO_PORT_SRV_CHN_A,
    ANDMSM_SIO_PORT_SRV_CHN_B,
    ANDMSM_SIO_PORT_SRV_CHN_C,
    ANDMSM_SIO_PORT_SRV_CHN_D,
    ANDMSM_SIO_PORT_SRV_CHN_E,
    ANDMSM_SIO_PORT_SRV_CHN_F,
    ANDMSM_SIO_PORT_SRV_CHN_10,
    ANDMSM_SIO_PORT_SRV_CHN_11,
    ANDMSM_SIO_PORT_SRV_CHN_12,
    ANDMSM_SIO_PORT_SRV_CHN_13,
    ANDMSM_SIO_PORT_SRV_CHN_14,
    ANDMSM_SIO_PORT_SRV_CHN_15,
    ANDMSM_SIO_PORT_SRV_CHN_16,
    ANDMSM_SIO_PORT_SRV_CHN_17,
    ANDMSM_SIO_PORT_SRV_CHN_18,
    ANDMSM_SIO_PORT_SRV_CHN_19,
    ANDMSM_SIO_PORT_SRV_CHN_1A,
    ANDMSM_SIO_PORT_SRV_CHN_1B,
    ANDMSM_SIO_PORT_SRV_CHN_1C,
    ANDMSM_SIO_PORT_SRV_CHN_1D,
    ANDMSM_SIO_PORT_SRV_CHN_1E,
    ANDMSM_SIO_PORT_SRV_CHN_1F,
    ANDMSM_SIO_PORT_MAX
}andmsm_port_num_enum;


/*===========================================================================

                        MACRO DEFINITIONS

===========================================================================*/
#define KERNEL_SUSPEND_SLEEP_STATUS     0xFFFFEEEE
#define KERNEL_ARCH_IDLE_STATUS         0xFF123456

/*===========================================================================

                        DATA DECLARATIONS

===========================================================================*/
/*===========================================================================

                        FUNCTION DECLARATIONS

===========================================================================*/

#endif  /* #ifndef ANDMSM_SHARE_H */

