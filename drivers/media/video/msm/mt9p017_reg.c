/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "mt9p017.h"
#include <linux/kernel.h>

/*Micron settings from Applications for lower power consumption.*/
struct reg_struct const mt9p017_reg_pat[2] = {
	{ /* Preview */
		/* vt_pix_clk_div          REG=0x0300 */
		13,  /* 5 */

		/* vt_sys_clk_div          REG=0x0302 */
		1,
		/* pre_pll_clk_div         REG=0x0304 */
		2,
		/* pll_multiplier          REG=0x0306 */
		54,

		/* op_pix_clk_div          REG=0x0308 */
		10,  /* 10 */

		/* op_sys_clk_div          REG=0x030A */
		1,

		/* scale_m                 REG=0x0404 */
		16,

		/* row_speed               REG=0x3016 */
		0x0111,

		/* x_addr_start            REG=0x3004 */
		8,

		/* x_addr_end              REG=0x3008 */
		2597,

		/* y_addr_start            REG=0x3002 */
		8,

		/* y_addr_end              REG=0x3006 */
		1949,

		/* read_mode               REG=0x3040
		 * Preview 2x2 skipping */
		0x04C3, /* 0x046C 2x2 binning */

		/* x_output_size           REG=0x034C */
		1296,

		/* y_output_size           REG=0x034E */
		972,

		/* line_length_pck         REG=0x300C */
		3178,

		/* frame_length_lines      REG=0x300A */
		1045,

		/* coarse_integration_time REG=0x3012 */
		1044,

		/* fine_integration_time   REG=0x3014 */
		2342,

		/* fine_correction         REG=0x3010 */
		388,

		/* vcm_control             REG=0x30F0 */
		0x8000,
	},
	{ /* Snapshot */
		/* vt_pix_clk_div          REG=0x0300 */
		13,

		/* vt_sys_clk_div          REG=0x0302 */
		1,

		/* pre_pll_clk_div         REG=0x0304 */
		2,

		/* pll_multiplier          REG=0x0306
		 * 60 for 10fps snapshot */
		54,

		/* op_pix_clk_div          REG=0x0308 */
		10,

		/* op_sys_clk_div          REG=0x030A */
		1,

		/* scale_m                 REG=0x0404 */
		16,

		/* row_speed               REG=0x3016 */
		0x0111,

		/* x_addr_start            REG=0x3004 */
		0,

		/* x_addr_end              REG=0x3008 */
		2607,

		/* y_addr_start            REG=0x3002 */
		0,

		/* y_addr_end              REG=0x3006 */
		1959,

		/* read_mode               REG=0x3040 */
		0x41,

		/* x_output_size           REG=0x034C */
		2608,

		/* y_output_size           REG=0x034E */
		1960,

		/* line_length_pck         REG=0x300C */
		5756, //0x167C,

		/* frame_length_lines      REG=0x300A 10 fps snapshot */
		2037, //0x7F5,

		/* coarse_integration_time REG=0x3012 */
		2036, //0x7F4,

		/* fine_integration_time   REG=0x3014 */
		5274, //0x149A,

		/* fine_correction         REG=0x3010 */
		0xA0,

		/* vcm_control             REG=0x30F0 */
		0x8000,
	}
};


struct mt9p017_i2c_reg_conf const mt9p017_test_tbl[] = {
	{0x3044, 0x0544 & 0xFBFF},
	{0x30CA, 0x0004 | 0x0001},
	{0x30D4, 0x9020 & 0x7FFF},
	{0x31E0, 0x0003 & 0xFFFE},
	{0x3180, 0x91FF & 0x7FFF},
	{0x301A, (0x10CC | 0x8000) & 0xFFF7},
	{0x301E, 0x0000},
	{0x3780, 0x0000},
};
struct mt9p017_i2c_reg_conf const mt9p017_rolloff_tbl[] = {
	{0x3600, 0x0650},
	{0x3602, 0x564D},
	{0x3604, 0x6730},
	{0x3606, 0x49CC},
	{0x3608, 0xC790},
	{0x360A, 0x0350},
	{0x360C, 0xF7ED},
	{0x360E, 0x5970},
	{0x3610, 0x378F},
	{0x3612, 0xDCD0},
	{0x3614, 0x0290},
	{0x3616, 0x4C2D},
	{0x3618, 0x35AF},
	{0x361A, 0xA5ED},
	{0x361C, 0xC1CE},
	{0x361E, 0x0310},
	{0x3620, 0x83EE},
	{0x3622, 0x79B0},
	{0x3624, 0x0F2F},
	{0x3626, 0xEEF0},
	{0x3640, 0x86AD},
	{0x3642, 0xAE8D},
	{0x3644, 0x9D4E},
	{0x3646, 0x782B},
	{0x3648, 0x216F},
	{0x364A, 0xAC6C},
	{0x364C, 0x33CD},
	{0x364E, 0x922C},
	{0x3650, 0xA12D},
	{0x3652, 0x3DCB},
	{0x3654, 0x506C},
	{0x3656, 0x306D},
	{0x3658, 0x934B},
	{0x365A, 0xC5CD},
	{0x365C, 0x6568},
	{0x365E, 0x0CEC},
	{0x3660, 0xEE8D},
	{0x3662, 0x0A8E},
	{0x3664, 0x104E},
	{0x3666, 0xECCE},
	{0x3680, 0x0FF1},
	{0x3682, 0x1B8F},
	{0x3684, 0x92D3},
	{0x3686, 0x8910},
	{0x3688, 0x1FF4},
	{0x368A, 0x1BD1},
	{0x368C, 0x5A0D},
	{0x368E, 0x89B3},
	{0x3690, 0xFF10},
	{0x3692, 0x1994},
	{0x3694, 0x2DD0},
	{0x3696, 0x796C},
	{0x3698, 0xC912},
	{0x369A, 0x194F},
	{0x369C, 0x7633},
	{0x369E, 0x06B1},
	{0x36A0, 0x018E},
	{0x36A2, 0x8F13},
	{0x36A4, 0xF110},
	{0x36A6, 0x2014},
	{0x36C0, 0xA089},
	{0x36C2, 0x44AD},
	{0x36C4, 0x3C4B},
	{0x36C6, 0x658C},
	{0x36C8, 0xDF10},
	{0x36CA, 0x2D2E},
	{0x36CC, 0xAC8A},
	{0x36CE, 0xD450},
	{0x36D0, 0x742E},
	{0x36D2, 0x4E4F},
	{0x36D4, 0xE86D},
	{0x36D6, 0xE1AD},
	{0x36D8, 0x6CAF},
	{0x36DA, 0x2D8F},
	{0x36DC, 0x9B71},
	{0x36DE, 0x9C8D},
	{0x36E0, 0x55CE},
	{0x36E2, 0xC28F},
	{0x36E4, 0xED4F},
	{0x36E6, 0x23F0},
	{0x3700, 0xECF1},
	{0x3702, 0x9130},
	{0x3704, 0x31F4},
	{0x3706, 0xED2F},
	{0x3708, 0xB8B4},
	{0x370A, 0xE4D1},
	{0x370C, 0x220B},
	{0x370E, 0x2394},
	{0x3710, 0xEED1},
	{0x3712, 0xD4B3},
	{0x3714, 0x9431},
	{0x3716, 0x428E},
	{0x3718, 0x1894},
	{0x371A, 0xBCF2},
	{0x371C, 0x9C94},
	{0x371E, 0xD271},
	{0x3720, 0xE56B},
	{0x3722, 0x1E54},
	{0x3724, 0xEA10},
	{0x3726, 0x8EF4},
	{0x3782, 0x04A4},
	{0x3784, 0x03B4},
	{0x37C0, 0x0000},
	{0x37C2, 0x0000},
	{0x37C4, 0x0000},
	{0x37C6, 0x0000},
	{0x3780, 0x8000},
};

struct mt9p017_reg mt9p017_regs = {
	.reg_pat = &mt9p017_reg_pat[0],
	.reg_pat_size = ARRAY_SIZE(mt9p017_reg_pat),
	.ttbl = &mt9p017_test_tbl[0],
	.ttbl_size = ARRAY_SIZE(mt9p017_test_tbl),
	.rftbl = &mt9p017_rolloff_tbl[0],
	.rftbl_size = ARRAY_SIZE(mt9p017_rolloff_tbl)
};
