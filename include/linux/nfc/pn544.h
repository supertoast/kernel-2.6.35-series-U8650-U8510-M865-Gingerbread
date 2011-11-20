/* ====*====*====*====*====*====*====*====*====*====*====*====*====*====*====*
 * 
 *                     PN544  Near Field Communication (NFC) driver
 * 
 * GENERAL DESCRIPTION
 *   driver for PN544 NFC on Huawei Mobile

 * REFERENCES
 * 
 * EXTERNALIZED FUNCTIONS
 *   None.
 * 
 * INITIALIZATION AND SEQUENCING REQUIREMENTS
 * 
 * Copyright (c) 2011 by HUAWEI, Incorporated.  All Rights Reserved.
 * ====*====*====*====*====*====*====*====*====*====*====*====*====*====*====
 * ===========================================================================
 * 
 *                       EDIT HISTORY FOR FILE
 * 
 *  This section contains comments describing changes made to this file.
 *   Notice that changes are listed in reverse chronological order.
 * 
 * 
 * when       who      what, where, why
 * -------------------------------------------------------------------------------
 */

/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora Forum nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * Alternatively, provided that this notice is retained in full, this software
 * may be relicensed by the recipient under the terms of the GNU General Public
 * License version 2 ("GPL") and only version 2, in which case the provisions of
 * the GPL apply INSTEAD OF those given above.  If the recipient relicenses the
 * software under the GPL, then the identification text in the MODULE_LICENSE
 * macro must be changed to reflect "GPLv2" instead of "Dual BSD/GPL".  Once a
 * recipient changes the license terms to the GPL, subsequent recipients shall
 * not relicense under alternate licensing terms, including the BSD or dual
 * BSD/GPL terms.  In addition, the following license statement immediately
 * below and between the words START and END shall also then apply when this
 * software is relicensed under the GPL:
 *
 * START
 *
 * This program is free software; you can redistribute it and/or modify it under
 * the terms of the GNU General Public License version 2 and only version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 * FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 * details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * END
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef _HUAWEI_PN544_H_
#define _HUAWEI_PN544_H_

#include <linux/i2c.h>
#include <linux/miscdevice.h> 

#define HCI_MODE					0 
#define FW_MODE						1 

#define PN544_I2C_ADDR				0x28
#define GPIO_NFC_INT				84
#define GPIO_NFC_VEN				85

#define PN544_DRIVER_NAME			"pn544"
#define PN544_DRIVER_DESC 			"NFC driver for PN544 on huawei mobile"
#define PN544_MAX_I2C_TRANSFER 		0x0400 
#define PN544_MSG_MAX_SIZE     		0x21							/* at normal HCI mode */ 
#define PN544_LLC_HCI_OVERHEAD 		3								/* header + crc (to length) */ 
#define PN544_LLC_MIN_SIZE     		(1 + PN544_LLC_HCI_OVERHEAD)	/* length + */ 
#define PN544_LLC_MAX_DATA    		(PN544_MSG_MAX_SIZE - 2) 
#define PN544_LLC_MAX_HCI_SIZE 		(PN544_LLC_MAX_DATA - 2) 
#define PN544_NODATA				((PN544_I2C_ADDR << 1) + 1)
#define PN544_RESET_SEND_SIZE 		6
#define PN544_RESET_RECEIVE_SIZE 	4
#define PN544_MAX_PACK_LEN			34
#define PN544_DEBUG_ON				1
#define PN544_DEBUG_OFF				0
#define PN544_DEBUG_SET_CLOCKANDSTANDBY		17

struct pn544_llc_packet { 
	unsigned char length;					/* of rest of packet */ 
	unsigned char header; 
	unsigned char data[PN544_LLC_MAX_DATA]; /* includes crc-ccitt */ 
}; 


enum pn544_state { 
	PN544_ST_COLD, 
	PN544_ST_READY, 
}; 

enum pn544_irq { 
	PN544_NONE, 
	PN544_INT, 
}; 
struct pn544_info { 
	struct miscdevice miscdev; 
	struct i2c_client *i2c_dev; 

	enum pn544_state state; 
	wait_queue_head_t read_wait; 
	loff_t read_offset; 
	int use_read_irq;
	enum pn544_irq read_irq; 
	struct mutex read_mutex;			/* Serialize read_irq access */ 
	struct mutex mutex;					/* Serialize info struct access */ 
	struct mutex mutex_mmi;				/* Serialize info struct access */ 
	u8 *buf; 
	unsigned int buflen; 
	bool				irq_enabled;
	spinlock_t	irq_enabled_lock;       /* irq spinlock */ 
};

struct pn544_nfc_platform_data {
	int (*pn544_ven_reset)(void);
	int (*pn544_interrupt_gpio_config)(void);
};

/* the following functions are used for huawei MMI_TEST */ 

int pn544_mmi_init(void);

int pn544_hci_exec(char *);
int pn544_hci_reset(void);
int pn544_hcibase_open(void);

int pn544_send_for_mmi(char * pszBuf,unsigned short SendCnt);
int pn544_read_for_mmi(char * pszBuf);
enum pn544_irq pn544_irq_state(struct pn544_info *info) ;

extern int pn544_debug_mask;
extern int pn544_debug_control;
extern int pn544_use_read_irq;
extern struct i2c_client pn544_client;
extern struct pn544_info * pn544_info;


#endif

