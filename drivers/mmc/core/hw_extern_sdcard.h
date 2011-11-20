/*
 * Copyright (c) 2010, HUAWEI. All rights reserved.
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
 
#ifndef HW_EXTERN_SDCARD_H
#define HW_EXTERN_SDCARD_H

// interface function. It`s called by mmc-host moudule when extern sdcard insert.
void hw_extern_sdcard_insert(void);

// interface function. It`s called by mmc-host moudule when extern sdcard remouve.
void hw_extern_sdcard_remove(void);

#endif /* HW_EXTERN_SDCARD_H */
