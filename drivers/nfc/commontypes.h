 /* support temp nfc test before the HAL and the Up-level code
 * is added to our project.
 * It is re-written according to a test software
 * provided by the NXP company who is the manufacturer of pn544
 * Maybe we will remove this file later.
 */ 
#ifndef PHNFCTYPES 
#define PHNFCTYPES 

#include <linux/types.h>

/**
 * name NFC Types
 *
 * File: ref phNfcTypes.h
 *
 */
#define PHNFCTYPES_FILEREVISION "$Revision: 1.13 $" /* ingroup grp_file_attributes */
#define PHNFCTYPES_FILEALIASES  "$Aliases: NFC_FRI1.1_WK926_R28_1,NFC_FRI1.1_WK928_R29_1,NFC_FRI1.1_WK930_R30_1,NFC_FRI1.1_WK934_PREP_1,NFC_FRI1.1_WK934_R31_1,NFC_FRI1.1_WK941_PREP1,NFC_FRI1.1_WK941_PREP2,NFC_FRI1.1_WK941_1,NFC_FRI1.1_WK943_R32_1,NFC_FRI1.1_WK949_PREP1,NFC_FRI1.1_WK943_R32_10,NFC_FRI1.1_WK943_R32_13,NFC_FRI1.1_WK943_R32_14,NFC_FRI1.1_WK1007_R33_1,NFC_FRI1.1_WK1007_R33_4,NFC_FRI1.1_WK1017_PREP1,NFC_FRI1.1_WK1017_R34_1,NFC_FRI1.1_WK1017_R34_2,NFC_FRI1.1_WK1023_R35_1 $"     /**< \ingroup grp_file_attributes */

#ifndef TRUE
/* ingroup grp_nfc_common */
#define TRUE			(0x01)			  
#endif

#ifndef FALSE
/* ingroup grp_nfc_common */
#define FALSE			(0x00)
#endif

/* ingroup grp_nfc_common
 * UTF8 Character String
 */
typedef uint8_t			utf8_t;	

/* ingroup grp_nfc_common
 * boolean data type 
 */
typedef uint8_t			bool_t;	

/* ingroup grp_nfc_common
 * NFC-FRI and HAL return values 
 * ref phNfcStatus.h for different status
 * values
 */
typedef uint16_t        NFCSTATUS;     
#ifndef NULL
#define NULL  ((void *)0)
#endif

/* This Macro to be used to resolve Unused and unreference
 * compiler warnings. 
 */

#define PHNFC_UNUSED_VARIABLE(x) for((x)=(x);(x)!=(x);)

#endif /* PHNFCTYPES */

