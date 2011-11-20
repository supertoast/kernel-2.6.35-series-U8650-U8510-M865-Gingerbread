/* support temp nfc test before the HAL and the Up-level code
 * is added to our project.
 * It is re-written according to a test software
 * provided by the NXP company who is the manufacturer of pn544
 * Maybe we will remove this file later.
 */ 
#include <linux/nfc/pn544.h>
#include <linux/string.h>
#include <linux/delay.h>
#include "scriptarrays.h"
#include "commonutility.h"
#include <linux/i2c.h>
#include <linux/slab.h> 
#include <linux/irq.h> 
#include <linux/interrupt.h>
/* we add pn544_debug_control to let the 
 * driver can control the printk more easier.
 */
#define PN544_HCI_DEBUG(message, ...) \
	do { \
	if (pn544_debug_mask && pn544_debug_control) \
	printk(message, ## __VA_ARGS__); \
	} while (0)

unsigned char llcSendBuf[34];
int llcSendLen=0;
unsigned char llcRecvBuf[34];
int llcRecvLen=0;


void DumpBuffer(char *buffer, int len, int direction)
{
	char finalStr[200];
	char ascBuf[150];
	int ascLen;

	ascLen = len * 2;
	BCD_2_ASC((unsigned char *)buffer, (unsigned char *)ascBuf, &ascLen);

	if(direction == 0)
	{
		sprintf(finalStr, "PC -> IFD: %s\n\n", ascBuf);
	}
	else if(direction == 1)
	{
		sprintf(finalStr, "IFD -> PC: %s\n\n", ascBuf);
	}
	else 
	{
		sprintf(finalStr, "DumpBuffer: %s\n\n", ascBuf);
	}

	PN544_HCI_DEBUG("%s",finalStr);

}


int pn544_hcibase_send(unsigned char *pszBuf , unsigned short SendCnt , unsigned short TimeOut)
{
	int ret=0;
	PN544_HCI_DEBUG("%s:enterd,sendcnt=%d\n",__func__,SendCnt);

	ret=pn544_send_for_mmi(pszBuf,SendCnt);
	return (int)ret;
}

int pn544_hcibase_read(unsigned char *pszBuf , unsigned short RecvCnt , unsigned short TimeOut)
{

	int ret=0;
	enum pn544_irq irq=PN544_NONE;

	PN544_HCI_DEBUG("%s:entered,RecvCnt=%d\n",__func__,RecvCnt);


	if (0!=pn544_use_read_irq)
	{
		irq = pn544_irq_state(pn544_info); 
		ret=pn544_read_for_mmi(pszBuf);
		pn544_info->read_irq=PN544_NONE;
		if (ret<0)
		{
			goto out;
		}
	}
	else
	{
		mdelay(5);
		ret=pn544_read_for_mmi(pszBuf);
		if (ret<0)
		{
			goto out;
		}
	}


	PN544_HCI_DEBUG("%s:exit ret=%d\n",__func__,ret);

out:
	return ret;
}

int ChannelSend(unsigned char *contSendBuf, unsigned char contSendLen)
{
	int ret;

	PN544_HCI_DEBUG("%s:enter\n",__func__);

	memset(llcSendBuf, 0x00, sizeof(llcSendBuf));
	llcSendLen = 0;

	/* prepare buffer for llc layer */
	llcSendBuf[0] = contSendLen + 2;
	memcpy(llcSendBuf + 1, contSendBuf, contSendLen);

	phLlcNfc_H_ComputeCrc(llcSendBuf, contSendLen + 1, llcSendBuf + contSendLen + 1, llcSendBuf + contSendLen + 2);
	llcSendLen = contSendLen + 3;

	/* send llc layer command request to chip */

	ret=pn544_hcibase_send(llcSendBuf, llcSendLen, 3);

	if(ret != llcSendLen)
	{
		return -1;
	}

	/* dump log from PC to IFD */
	DumpBuffer((char *)llcSendBuf, llcSendLen, 0);

	return 0;
}

int ChannelReceive(unsigned char *contRecvBuf, unsigned char *contRecvLenPtr)
{
	unsigned char calCRC[2];
	int tmplen=0;

	memset(llcRecvBuf, 0x00, sizeof(llcRecvBuf));
	llcRecvLen = 0;

	/* receive llc layer command response from chip */
	llcRecvLen = 0;

	tmplen = pn544_hcibase_read(llcRecvBuf, ((int)llcRecvBuf[0])+1, 2);

	llcRecvLen += tmplen;

	phLlcNfc_H_ComputeCrc(llcRecvBuf, llcRecvLen - 2, calCRC, calCRC + 1);

	if(memcmp(llcRecvBuf + llcRecvLen - 2, calCRC, 2) == 0)
	{
		memcpy(contRecvBuf, llcRecvBuf + 1, llcRecvLen - 3);
		*contRecvLenPtr = llcRecvLen - 3;

		/* dump log from IFD to PC */
		DumpBuffer((char *)llcRecvBuf, llcRecvLen, 1);
		return 0;
	}
	else
	{
		PN544_HCI_DEBUG("%s:CRC error\n",__func__);
		return -3;
	}
}


int ScriptCmdExecute(unsigned char *scriptReqCmd, int scriptReqLen, unsigned char *scriptRespCmd, int *scriptRespLenPtr, unsigned char *frameNsPtr, unsigned char *frameNrPtr, int cmdType)
{
	unsigned char tmpContentSendBuf[50];
	unsigned char tmpContentSendLen;
	unsigned char tmpContentRecvBuf[50];
	unsigned char tmpContentRecvLen;

	int tmpAscLen;
	int tmpBcdLen;

	unsigned char tmpNumber = 0;

	int ret;

	int idx;
	unsigned char sFrameType;
	int orgCmdSendFlag;


	memset(tmpContentSendBuf, 0x00, sizeof(tmpContentSendBuf));
	tmpContentSendLen = 0;
	memset(tmpContentRecvBuf, 0x00, sizeof(tmpContentRecvBuf));
	tmpContentRecvLen = 0;

	if(cmdType == CMD_TYPE_SEND_NORMAL)
	{
		PN544_HCI_DEBUG("%s:--COMMDAND--\n",__func__);
	}
	else if(cmdType == CMD_TYPE_CONDITIONAL)
	{
		PN544_HCI_DEBUG("%s:---CONDITIONAL---\n",__func__);
	}
	else if(cmdType == CMD_TYPE_SEND_EVENT)
	{
		PN544_HCI_DEBUG("%s:---SEND EVENT---\n",__func__);
	}
	else if(cmdType == CMD_TYPE_WAIT_EVENT)
	{
		PN544_HCI_DEBUG("%s:---WAIT EVENT---\n",__func__);
	}
	else
	{
		PN544_HCI_DEBUG("%s:---STOP HERE---\n",__func__);
	}

	if((cmdType == CMD_TYPE_SEND_NORMAL) || (cmdType == CMD_TYPE_SEND_EVENT) 
		|| (cmdType == CMD_TYPE_CONDITIONAL))
	{
		if(scriptReqCmd[0] == 'U')
		{
			if(memcmp(scriptReqCmd + 1, "RSET", 4) == 0)
			{
				tmpContentSendBuf[0] = 0xF9;

				tmpAscLen = scriptReqLen - 5;
				tmpBcdLen = ASC_2_BCD(scriptReqCmd + 5, tmpContentSendBuf + 1, &tmpAscLen);
				tmpContentSendLen = 1 + tmpBcdLen;
			}
			else
			{
				return -1;
			}
		}
		else if(scriptReqCmd[0] == 'I')
		{
			tmpContentSendBuf[0] = 0x80|((*frameNsPtr)<<3)|(*frameNrPtr);
			tmpAscLen = scriptReqLen - 1;
			tmpBcdLen = ASC_2_BCD(scriptReqCmd + 1, tmpContentSendBuf + 1, &tmpAscLen);
			tmpContentSendLen = 1 + tmpBcdLen;

			if(*frameNsPtr < 7)
			{
				*frameNsPtr = *frameNsPtr + 1;
			}
			else
			{
				*frameNsPtr = 0;
			}
		}
		else if(scriptReqCmd[0] == 'U')
		{
			return -2;
		}
		else
		{
			return -1;
		}
	}
	orgCmdSendFlag = 1;
	for(idx = 0; idx < 3; idx ++)
	{
		if(orgCmdSendFlag == 1)
		{
			if((cmdType == CMD_TYPE_SEND_NORMAL) || (cmdType == CMD_TYPE_SEND_EVENT) 
				|| (cmdType == CMD_TYPE_CONDITIONAL))
			{
				ret = ChannelSend(tmpContentSendBuf, tmpContentSendLen);
				if(ret != 0)
				{
					return -3;
				}
			}
		}

		ret = ChannelReceive(tmpContentRecvBuf, &tmpContentRecvLen);
		if(ret != 0)
		{
			return -4;
		}

		if((tmpContentRecvBuf[0] & 0xC0) == 0x80)		//i-frame
		{
			tmpNumber = (tmpContentRecvBuf[0] & 0x38) >> 3;
			if(tmpNumber != *frameNrPtr)
			{
				orgCmdSendFlag = 1;
				continue;
			}
			else
			{
				if((tmpNumber + 1) <= 7)
				{
					*frameNrPtr = tmpNumber + 1;
				}
				else
				{
					*frameNrPtr = 0;
				}
			}
			*frameNsPtr = tmpContentRecvBuf[0] & 0x07;

			/* send s-frame with RR */
			tmpContentSendBuf[0] = 0xC0|(*frameNrPtr);
			tmpContentSendLen = 1;
			ret = ChannelSend(tmpContentSendBuf, tmpContentSendLen);
			if(ret != 0)
			{
				return -3;
			}

			break;
		}
		else if((tmpContentRecvBuf[0] & 0xE0) == 0xC0)	//s-frame
		{
			sFrameType = (tmpContentRecvBuf[0] & 0x18) >> 3;
			if(sFrameType == 0x00)			//RR
			{
				*frameNsPtr = tmpContentRecvBuf[0] & 0x07;
				if(cmdType == CMD_TYPE_SEND_EVENT)
				{
					break;
				}
				else
				{
					orgCmdSendFlag = 0;
					PN544_HCI_DEBUG("%s:sFrameType == 0x00 and cmdType != CMD_TYPE_SEND_EVENT\n",__func__);
					continue;
				}
			}
			else if(sFrameType == 0x01)		//REJ
			{
				orgCmdSendFlag = 1;
				PN544_HCI_DEBUG("%s:sFrameType == 0x01\n",__func__);
				continue;
			}
			else if(sFrameType == 0x02)		//RNR
			{
				orgCmdSendFlag = 1;
				PN544_HCI_DEBUG("%s:sFrameType == 0x02\n",__func__);
				continue;
			}
			else							//SREJ
			{
				return -5;
			}
		}
		else if((tmpContentRecvBuf[0] & 0xE0) == 0xE0)	//u-frame
		{
			break;
		}
		else
		{
			return -5;
		}
	}
	if(idx >= 3)
	{
		PN544_HCI_DEBUG("%s:frame retransmission exceeds 3.\n",__func__);
		return -6;
	}

	memcpy(scriptRespCmd, tmpContentRecvBuf, tmpContentRecvLen);
	*scriptRespLenPtr = tmpContentRecvLen;

	return 0;
}
int ScriptBatchRun(SINGLE_CMD_LINE *tmpCmdLineElem, unsigned char *lastRespCmd, 
				   int *lastRespLenPtr, unsigned char *frameNsPtr, unsigned char *frameNrPtr)
{
	unsigned char scriptReqBuf[70];
	int scriptReqLen=0;
	unsigned char scriptRespBuf[70];
	int scriptRespLen=0;

	unsigned char condBuf[70];
	int condBufLen=0;

	int ret=0;
	int idx=0;
	int retryCount=0;
	int condBranchNeededFlag=0;

	char tmpBuf[30];
	PN544_HCI_DEBUG("%s:enter\n",__func__);
	memset(scriptReqBuf, 0x00, sizeof(scriptReqBuf));
	scriptReqLen = 0;
	memset(scriptRespBuf, 0x00, sizeof(scriptRespBuf));
	scriptRespLen = 0;

	idx = 0;
	ret = 0;
	retryCount = 0;
	condBranchNeededFlag = 0;
	while(1)
	{
		if(tmpCmdLineElem[idx].type == CMD_TYPE_STOP_HERE)
		{
			break;
		}

		if(condBranchNeededFlag == 1)
		{
			if(tmpCmdLineElem[idx].type == CMD_TYPE_CONDITIONAL)
			{
				condBufLen = strlen(tmpCmdLineElem[idx].expectdResult);
				ASC_2_BCD((unsigned char *)tmpCmdLineElem[idx].expectdResult, condBuf, &condBufLen);
				if(memcmp(scriptRespBuf + 1, condBuf, scriptRespLen - 1) == 0)
				{
					strcpy((char *)scriptReqBuf, tmpCmdLineElem[idx].scriptCmdLine);
					scriptReqLen = strlen((char *)scriptReqBuf);
					ret = ScriptCmdExecute(scriptReqBuf, scriptReqLen, scriptRespBuf, 
						&scriptRespLen, frameNsPtr, frameNrPtr, tmpCmdLineElem[idx].type);
					if(ret != 0)
					{
						return -1;
					}

					condBranchNeededFlag = 0;
				}
			}

			retryCount = 0;
		}
		else
		{
			strcpy((char *)scriptReqBuf, tmpCmdLineElem[idx].scriptCmdLine);
			scriptReqLen = strlen((char *)scriptReqBuf);
			ret = ScriptCmdExecute(scriptReqBuf, scriptReqLen, scriptRespBuf, 
				&scriptRespLen, frameNsPtr, frameNrPtr, tmpCmdLineElem[idx].type);
			if(ret == 0)
			{
				condBufLen = strlen(tmpCmdLineElem[idx].expectdResult);
				ASC_2_BCD((unsigned char *)tmpCmdLineElem[idx].expectdResult, condBuf, &condBufLen);
				if((strlen(tmpCmdLineElem[idx].expectdResult) > 0) && (memcmp(scriptRespBuf + 1, condBuf, scriptRespLen - 1) != 0))
				{
					PN544_HCI_DEBUG("%s:point 1\n",__func__);
					if(tmpCmdLineElem[idx].action == 0)
					{
						/* return error directly */
						return -1;
					}
					else if(tmpCmdLineElem[idx].action == 1)
					{
						/* continue anyway */
					}
					else if(tmpCmdLineElem[idx].action == 2)
					{
						/* need condtional branch in next command */
						condBranchNeededFlag = 1;
					}
				}

				retryCount = 0;
			}
			else
			{
				if(tmpCmdLineElem[idx].action == 3)
				{
					retryCount ++;
					if(retryCount < 3)
					{
						mdelay(2000);
						continue;
					}
					else
					{
						return -2;
					}
				}
				else
				{
					if(tmpCmdLineElem[idx].type == 2)
					{
						return ret;
					}
				}
			}
		}

		idx ++;

		memset(tmpBuf, 0x00, sizeof(tmpBuf));
		sprintf(tmpBuf, "idx: %d, ret: %d\n", idx - 1, ret);
		PN544_HCI_DEBUG("%s:%s",__func__,tmpBuf);
	}

	memcpy(lastRespCmd, scriptRespBuf, scriptRespLen);
	*lastRespLenPtr = scriptRespLen;

	return 0;
}

int pn544_hcibase_open(void)
{
	return 0;
}

int pn544_hci_exec(char *buf)
{
	unsigned char lastRespBuf[80];
	int lastRespLen=0;

	unsigned char ascUidBuf[80];
	int ascUidLen = 0;

	int ret=0;
	unsigned char frameNs = 0;
	unsigned char frameNr = 0;

	SINGLE_CMD_LINE *curCmdLineElement=NULL;
	int readmodeon =0 ;

	switch(pn544_debug_mask)
	{
		case 11:
			curCmdLineElement = GetVersion_Array;
			break;
		case 12:
			curCmdLineElement = EnableAllEmu_Array;
			break;
		case 13:
			curCmdLineElement = ReaderMode_Array;
			readmodeon=1;
			break;
		case 14:
			curCmdLineElement = InitiatorMode_Array;
			break;
		case 15:
			curCmdLineElement = TargetMode_Array;
			break;
		case 16:
			curCmdLineElement = RFSWITCHON_ARRAY;
			break;
		case 17:
			curCmdLineElement = CLOCKSET_ARRAY;
			break;
		case 18:
			curCmdLineElement = CLEARSWPCFG_ARRAY;
			break;
		case 19:
			curCmdLineElement = SWPPBTF_ARRAY;
			break;
		case 20:
			curCmdLineElement = SWPPBTFENABLE_ARRAY;
			break;
		case 21:
			curCmdLineElement =SIMPLERESET_ARRAY;
			break;
		case 22:
			curCmdLineElement =IRQCONFIG_ARRAY;
			break;
		case 23:
			curCmdLineElement = TICCONFIG_ARRAY;
			break;
		/* we add the STANDBY_ARRAY to set the EEPORM
		 * to config the PN544 can enter the standby mode 
		 */

		case 24:
			curCmdLineElement = STANDBY_ARRAY;
			break;
		default:
			curCmdLineElement = ReaderMode_Array;
			readmodeon=1;
			break;
	}

	PN544_HCI_DEBUG("%s:enter\n",__func__);

	ret = ScriptBatchRun(curCmdLineElement, lastRespBuf, &lastRespLen, &frameNs, &frameNr);

	if(ret==0)
	{
		if(readmodeon==1)
		{
			memset(ascUidBuf, 0x00, sizeof(ascUidBuf));
			ascUidLen = (lastRespLen - 3) * 2;
			BCD_2_ASC(lastRespBuf + 3, ascUidBuf, &ascUidLen);
			strcpy(buf,(char *)ascUidBuf);
		}
		else
		{
			strcpy(buf,"nfc_ok");
		}	
	}
	else
	{
		strcpy(buf,"nfc_error");	
	}
	return ret;

}


