 /* support temp nfc test before the HAL and the Up-level code
 * is added to our project.
 * It is re-written according to a test software
 * provided by the NXP company who is the manufacturer of pn544
 * Maybe we will remove this file later.
 */ 
#ifndef _COMMON_UTILITY_H

#define _COMMON_UTILITY_H

#include "commontypes.h"

void 
phLlcNfc_H_ComputeCrc(
					  uint8_t     *pData, 
					  uint8_t     length,
					  uint8_t     *pCrc1, 
					  uint8_t     *pCrc2
					  );


unsigned char bcdChar(unsigned char achar);
int BCD_2_ASC(unsigned char *src, unsigned char *dest, int *srcLen);
int ASC_2_BCD(unsigned char *src, unsigned char *dest, int *srcLen);


#endif


