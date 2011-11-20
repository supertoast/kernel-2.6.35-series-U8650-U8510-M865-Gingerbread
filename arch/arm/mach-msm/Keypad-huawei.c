/*
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2008-2009, Code Aurora Forum. All rights reserved.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/gpio_event.h>

#include <asm/mach-types.h>

#undef  KEYMAP_INDEX
#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(u8510_keypad_col_gpios) + (col))

/*                                          U8510 keypad begin                                                                 */
static unsigned int u8510_keypad_col_gpios[] = { 41, 40, 39 };
static unsigned int u8510_keypad_row_gpios[] = { 35 };

static const unsigned short u8510_keypad_keymap_surf[ARRAY_SIZE(u8510_keypad_col_gpios) *
					  ARRAY_SIZE(u8510_keypad_row_gpios)] = {

	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
	[KEYMAP_INDEX(0, 2)] = KEY_HOME

};

/* u8510 keypad platform device information */
static struct gpio_event_matrix_info u8510_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= u8510_keypad_keymap_surf,
	.output_gpios	= u8510_keypad_row_gpios,
	.input_gpios	= u8510_keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(u8510_keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(u8510_keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *u8510_keypad_info[] = {
	&u8510_keypad_matrix_info.info
};

static struct gpio_event_platform_data u8510_keypad_data = {
	.name		= "surf_keypad",
	.info		= u8510_keypad_info,
	.info_count	= ARRAY_SIZE(u8510_keypad_info)
};

struct platform_device keypad_device_u8510 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &u8510_keypad_data,
	},
};

/*  C8800 keypad */
#define KEYMAP_C8800_INDEX(row, col) ((row)*ARRAY_SIZE(c8800_keypad_col_gpios) + (col))

/*                                          C8800 keypad begin                                                                 */
static unsigned int c8800_keypad_col_gpios[] = { 41, 40 };
static unsigned int c8800_keypad_row_gpios[] = { 35 };

static const unsigned short c8800_keypad_keymap_surf[ARRAY_SIZE(c8800_keypad_col_gpios) *
					  ARRAY_SIZE(c8800_keypad_row_gpios)] = {

	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN

};

/* u8510 keypad platform device information */
static struct gpio_event_matrix_info c8800_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= c8800_keypad_keymap_surf,
	.output_gpios	= c8800_keypad_row_gpios,
	.input_gpios	= c8800_keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(c8800_keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(c8800_keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *c8800_keypad_info[] = {
	&c8800_keypad_matrix_info.info
};

static struct gpio_event_platform_data c8800_keypad_data = {
	.name		= "surf_keypad",
	.info		= c8800_keypad_info,
	.info_count	= ARRAY_SIZE(c8800_keypad_info)
};

struct platform_device keypad_device_c8800 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &c8800_keypad_data,
	},
};

#define KEYMAP_U8650_INDEX(row, col) ((row)*ARRAY_SIZE(u8650_keypad_col_gpios) + (col))

/*                                          U8650 keypad begin                                                                 */
static unsigned int u8650_keypad_col_gpios[] = { 41, 40 };
static unsigned int u8650_keypad_row_gpios[] = { 35 };

static const unsigned short u8650_keypad_keymap_surf[ARRAY_SIZE(u8650_keypad_col_gpios) *
					  ARRAY_SIZE(u8650_keypad_row_gpios)] = {

    /* the volume keys reverse */
	[KEYMAP_U8650_INDEX(0, 0)] = KEY_VOLUMEDOWN,
	[KEYMAP_U8650_INDEX(0, 1)] = KEY_VOLUMEUP

};

/* u8650 keypad platform device information */
static struct gpio_event_matrix_info u8650_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= u8650_keypad_keymap_surf,
	.output_gpios	= u8650_keypad_row_gpios,
	.input_gpios	= u8650_keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(u8650_keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(u8650_keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *u8650_keypad_info[] = {
	&u8650_keypad_matrix_info.info
};

static struct gpio_event_platform_data u8650_keypad_data = {
	.name		= "surf_keypad",
	.info		= u8650_keypad_info,
	.info_count	= ARRAY_SIZE(u8650_keypad_info)
};

struct platform_device keypad_device_u8650 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &u8650_keypad_data,
	},
};

/*delete U8651 branch*/

#undef  KEYMAP_INDEX
#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(m865_keypad_col_gpios) + (col))

/*                                          M865 keypad begin                                                                 */
static unsigned int m865_keypad_col_gpios[] = { 41, 40};
static unsigned int m865_keypad_row_gpios[] = { 35 };
static const unsigned short m865_keypad_keymap_surf[ARRAY_SIZE(m865_keypad_col_gpios) *
					  ARRAY_SIZE(m865_keypad_row_gpios)] = {

	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
};
/* M865 keypad platform device information */
static struct gpio_event_matrix_info m865_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= m865_keypad_keymap_surf,
	.output_gpios	= m865_keypad_row_gpios,
	.input_gpios	= m865_keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(m865_keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(m865_keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *m865_keypad_info[] = {
	&m865_keypad_matrix_info.info
};

static struct gpio_event_platform_data m865_keypad_data = {
	.name		= "surf_keypad",
	.info		= m865_keypad_info,
	.info_count	= ARRAY_SIZE(m865_keypad_info)
};

struct platform_device keypad_device_m865 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &m865_keypad_data,
	},
};
#undef  KEYMAP_INDEX
#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(c8650_keypad_col_gpios) + (col))

/*                                          C8650 keypad begin                                                                 */
static unsigned int c8650_keypad_col_gpios[] = { 41, 40};
static unsigned int c8650_keypad_row_gpios[] = { 35 };

static const unsigned short c8650_keypad_keymap_surf[ARRAY_SIZE(c8650_keypad_col_gpios) *
					  ARRAY_SIZE(c8650_keypad_row_gpios)] = {

	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
};

/* C8650 keypad platform device information */
static struct gpio_event_matrix_info c8650_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= c8650_keypad_keymap_surf,
	.output_gpios	= c8650_keypad_row_gpios,
	.input_gpios	= c8650_keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(c8650_keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(c8650_keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *c8650_keypad_info[] = {
	&c8650_keypad_matrix_info.info
};

static struct gpio_event_platform_data c8650_keypad_data = {
	.name		= "surf_keypad",
	.info		= c8650_keypad_info,
	.info_count	= ARRAY_SIZE(c8650_keypad_info)
};

struct platform_device keypad_device_c8650 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &c8650_keypad_data,
	},
};

#undef  KEYMAP_INDEX
#define KEYMAP_INDEX(row, col) ((row)*ARRAY_SIZE(c8651_keypad_col_gpios) + (col))

/*                                          C8651 keypad begin                                                                 */
static unsigned int c8651_keypad_col_gpios[] = { 41, 40};
static unsigned int c8651_keypad_row_gpios[] = { 35 };

static const unsigned short c8651_keypad_keymap_surf[ARRAY_SIZE(c8651_keypad_col_gpios) *
					  ARRAY_SIZE(c8651_keypad_row_gpios)] = {

	[KEYMAP_INDEX(0, 0)] = KEY_VOLUMEUP,
	[KEYMAP_INDEX(0, 1)] = KEY_VOLUMEDOWN,
};

/* C8650 keypad platform device information */
static struct gpio_event_matrix_info c8651_keypad_matrix_info = {
	.info.func	= gpio_event_matrix_func,
	.keymap		= c8651_keypad_keymap_surf,
	.output_gpios	= c8651_keypad_row_gpios,
	.input_gpios	= c8651_keypad_col_gpios,
	.noutputs	= ARRAY_SIZE(c8651_keypad_row_gpios),
	.ninputs	= ARRAY_SIZE(c8651_keypad_col_gpios),
	.settle_time.tv.nsec = 40 * NSEC_PER_USEC,
	.poll_time.tv.nsec = 20 * NSEC_PER_MSEC,
	.flags		= GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE |
			  GPIOKPF_PRINT_UNMAPPED_KEYS
};

static struct gpio_event_info *c8651_keypad_info[] = {
	&c8651_keypad_matrix_info.info
};

static struct gpio_event_platform_data c8651_keypad_data = {
	.name		= "surf_keypad",
	.info		= c8651_keypad_info,
	.info_count	= ARRAY_SIZE(c8651_keypad_info)
};

struct platform_device keypad_device_c8651 = {
	.name	= GPIO_EVENT_DEV_NAME,
	.id	= -1,
	.dev	= {
		.platform_data	= &c8651_keypad_data,
	},
};

