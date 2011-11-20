 /* support temp nfc test before the HAL and the Up-level code
 * is added to our project.
 * It is re-written according to a test software
 * provided by the NXP company who is the manufacturer of pn544
 * Maybe we will remove this file later.
 */ 


#ifndef _SCRIPT_ARRAYS_H
#define _SCRIPT_ARRAYS_H

#define MAX_SINGLE_CMD_SIZE     	50
#define CMD_TYPE_SEND_NORMAL      	0		//normal
#define CMD_TYPE_CONDITIONAL      	1		//conditional
#define CMD_TYPE_WAIT_EVENT       	2		//waiting event
#define CMD_TYPE_STOP_HERE        	3		//stop here
#define CMD_TYPE_SEND_EVENT       	4		//sending event


typedef struct
{
	int type;									//0: normal, 1: conditional, 2: waiting event, 3: stop here
	int action;									//0: if not matching, return
												//1:if not matching, ignore and continue anyway
												//2: if not matching, compare to next command line
												//3: if no response, send current command again 
	char scriptCmdLine[MAX_SINGLE_CMD_SIZE];	//script format
	char expectdResult[MAX_SINGLE_CMD_SIZE];	//response with content
}SINGLE_CMD_LINE;


extern SINGLE_CMD_LINE GetVersion_Array[];
extern SINGLE_CMD_LINE EnableAllEmu_Array[];
extern SINGLE_CMD_LINE ReaderMode_Array[];
extern SINGLE_CMD_LINE InitiatorMode_Array[];
extern SINGLE_CMD_LINE TargetMode_Array[];
extern SINGLE_CMD_LINE RFSWITCHON_ARRAY[];
extern SINGLE_CMD_LINE CLOCKSET_ARRAY[];
extern SINGLE_CMD_LINE CLEARSWPCFG_ARRAY[];
extern SINGLE_CMD_LINE SWPPBTF_ARRAY[];
extern SINGLE_CMD_LINE SWPPBTFENABLE_ARRAY[];
extern SINGLE_CMD_LINE SIMPLERESET_ARRAY[];
extern SINGLE_CMD_LINE IRQCONFIG_ARRAY[];
extern SINGLE_CMD_LINE TICCONFIG_ARRAY[];
extern SINGLE_CMD_LINE STANDBY_ARRAY[];

#endif

