/* Copyright (c), Code HUAWEI. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef LCDC_HUAWEI_CONFIG_H
#include "msm_fb.h"
#include <linux/hardware_self_adapt.h>
#include "lcd_hw_debug.h"

#define LCDC_HUAWEI_CONFIG_H

#define GP_MD_REG_ADDR_OFFSET              0x0058
#define GP_NS_REG_ADDR_OFFSET              0x005C
#define MSM_GP_MD_REG_VIRT_ADD            (MSM_CLK_CTL_BASE + GP_MD_REG_ADDR_OFFSET)
#define MSM_GP_NS_REG_VIRT_ADD            (MSM_CLK_CTL_BASE + GP_NS_REG_ADDR_OFFSET)

#define PWM_LCD_NOT_N_M_VAL                0xFE4D
#define PWM_LCD_M_VAL                      0x0001

#define GP_ROOT_ENA                        (1 << 11)
#define GP_CLK_BRANCH_ENA                  (1 << 9)
#define GP_MNCNTR_EN                       (1 << 8)
#define GP_NS_REG_SRC_SEL                  (0 << 0)
#define GP_NS_REG_PRE_DIV_SEL              (1 << 3)
#define GP_NS_REG_MNCNTR_MODE              (3 << 5)
#define GP_NS_REG_GP_N_VAL                 (PWM_LCD_NOT_N_M_VAL << 16)
#define GP_MD_REG_M_VAL                    (PWM_LCD_M_VAL << 16)
/* Move from the every LCD file ,those are common */
#define TRACE_LCD_DEBUG 1
#if TRACE_LCD_DEBUG
#define LCD_DEBUG(x...) printk(KERN_ERR "[LCD_DEBUG] " x)
#else
#define LCD_DEBUG(x...) do {} while (0)
#endif
/* LCD_MDELAY will select mdelay or msleep according value */
#define LCD_MDELAY(time_ms)   	\
	do							\
	{ 							\
		if (time_ms>10)			\
			msleep(time_ms);	\
		else					\
			mdelay(time_ms);	\
	}while(0)	
#define TYPE_COMMAND	         (1<<0)
#define TYPE_PARAMETER           (1<<1)
#define START_BYTE_COMMAND 		0x00
#define START_BYTE_PARAMETER	0x01
#define LCD_MIN_BACKLIGHT_LEVEL 0
#define LCD_MAX_BACKLIGHT_LEVEL	255
struct lcd_state_type
{
	boolean disp_initialized;
	boolean display_on;
	boolean disp_powered_up;
};
void pwm_set_backlight(int level);
int process_lcdc_table(struct sequence *table, size_t count, lcd_panel_type lcd_panel);


void lcd_spi_init(struct msm_panel_common_pdata *lcdc_pnael_data);
void truly_r61529_set_cs(struct msm_panel_common_pdata *lcdc_pnael_data);
void seriout_transfer_byte(uint8 reg, uint8 start_byte);
void seriout_cmd(uint16 reg, uint8 start_byte);
void seriout_data(uint16 data, uint8 start_byte);
void lcd_set_backlight_pwm(int level);
uint8 seri_read_byte(uint8 start_byte);
void seriout_byte_9bit(uint8 start_byte, uint8 data);
int lcd_reset(uint32 lcd_host_type);
/* We delete the function declarations here as we put 
 * the decalaration in a seperate file.
 */ 

#endif
