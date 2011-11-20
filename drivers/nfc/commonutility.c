 /* support temp nfc test before the HAL and the Up-level code
 * is added to our project.
 * It is re-written according to a test software
 * provided by the NXP company who is the manufacturer of pn544
 * Maybe we will remove this file later.
 */ 
#include <linux/ctype.h>
#include "commonutility.h"
#include "commontypes.h"

static void phLlcNfc_H_UpdateCrc(
								 uint8_t     crcByte, 
								 uint16_t    *pCrc
								 );

void phLlcNfc_H_ComputeCrc(
						   uint8_t     *pData, 
						   uint8_t     length,
						   uint8_t     *pCrc1, 
						   uint8_t     *pCrc2
						   )
{
	uint8_t     crc_byte = 0, 
		index = 0;
	uint16_t    crc = 0;

#ifdef CRC_A
	crc = 0x6363; /* ITU-V.41 */
#else
	crc = 0xFFFF; /* ISO/IEC 13239 (formerly ISO/IEC 3309) */
#endif /* #ifdef CRC_A */

	do 
	{
		crc_byte = pData[index];
		phLlcNfc_H_UpdateCrc(crc_byte, &crc);
		index++;
	} while (index < length);

#ifndef INVERT_CRC
	crc = ~crc; /* ISO/IEC 13239 (formerly ISO/IEC 3309) */
#endif /* #ifndef INVERT_CRC */

	*pCrc1 = (uint8_t) (crc & 0xFF);
	*pCrc2 = (uint8_t) ((crc >> 8) & 0xFF);
	return;
}

static 
void 
phLlcNfc_H_UpdateCrc(
					 uint8_t     crcByte, 
					 uint16_t    *pCrc
					 )
{
	crcByte = (crcByte ^ (uint8_t)((*pCrc) & 0x00FF));
	crcByte = (crcByte ^ (uint8_t)(crcByte << 4));
	*pCrc = (*pCrc >> 8) ^ ((uint16_t)crcByte << 8) ^
		((uint16_t)crcByte << 3) ^
		((uint16_t)crcByte >> 4);
}

unsigned char bcdChar(unsigned char achar)
{
	unsigned char result;

	if (isalnum(achar))
		result = toupper(achar);
	else 
		result = 'D';
	if (result > 0x46)
		result = 'D';
	return result;
}


int BCD_2_ASC(unsigned char *src, unsigned char *dest, int *srcLen)
{
	int i, ascLen;
	char achar, bcdLen;
	unsigned char *bcd, *asc;

	dest[*srcLen] = 0;
	bcd = src;
	asc = dest;
	ascLen = *srcLen;
	*srcLen = (*srcLen +1) >>1;

	if ( ascLen & 0x01 )
	{
		achar = *bcd++ & 0x0f;
		*asc++ = (achar > 9) ? achar + 0x41 - 10 : achar + 0x30;
		ascLen--;
	}

	bcdLen = ascLen >>1;
	for(i = 0; i < bcdLen; i++)
	{
		achar = (*bcd & 0xf0) >> 4;
		*asc++ = (achar > 9) ? achar + 0x41 - 10 : achar + 0x30;
		achar = *bcd++ & 0x0f;
		*asc++ = (achar > 9) ? achar + 0x41 - 10 : achar + 0x30;
	}

	return (int)(asc - dest);
}


int ASC_2_BCD(unsigned char *src, unsigned char *dest, int *srcLen)
{
	int bcdLen, i, ascLen;
	unsigned char *asc, *bcd, achar;


	asc = src;
	bcd = dest;
	ascLen = *srcLen;

	if ( ascLen & 0x01 )
	{
		achar = bcdChar(*asc++);
		*bcd++ = (achar <= 0x39)? achar - 0x30 : achar - 0x41 + 10;
		ascLen--;
	}

	bcdLen = ascLen >>1;

	for(i=0;i<bcdLen;i++)
	{
		achar = bcdChar(*asc++);
		*bcd = ((achar <= 0x39) ? achar - 0x30 : achar - 0x41 + 10);
		*bcd = *bcd << 4;
		achar = bcdChar(*asc++);
		*bcd += ((achar <= 0x39) ? achar - 0x30 : achar - 0x41 + 10);
		bcd++;
	}

	return (int)(bcd - dest);
}



