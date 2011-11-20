/*
 * Copyright (C) 2010-2012 HUAWEI Incorporated.
 */
#ifndef __LINUX_SMEM_SLEEP_LOG_H
#define __LINUX_SMEM_SLEEP_LOG_H

#include <linux/types.h>
#include <asm/sizes.h>
#include <linux/ioctl.h>

#define MSM_SMEM_SLEEP_LOG_IOCTL_MAGIC 's'

#define MSM_SMEM_SLEEP_LOG_IOCTL_GET_PWR_EVENT \
	_IO(MSM_SMEM_SLEEP_LOG_IOCTL_MAGIC, 1)

#define MSM_SMEM_SLEEP_LOG_IOCTL_GET_SLEEP_VOTER \
	_IO(MSM_SMEM_SLEEP_LOG_IOCTL_MAGIC, 2)

#endif /* __LINUX_SMEM_SLEEP_LOG_H */

