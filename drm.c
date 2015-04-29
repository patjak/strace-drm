/*
 * Copyright (c) 2015 Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Authors:
 *    Patrik Jakobsson <patrik.jakobsson@linux.intel.com>
 */

#include "defs.h"

#include <sys/types.h>
#include <unistd.h>
#include <string.h>
#include <linux/limits.h>
#include <stdint.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <drm.h>

#define DRM_MAX_NAME_LEN 128

inline int drm_is_priv(const unsigned int num)
{
	return (_IOC_NR(num) >= DRM_COMMAND_BASE &&
		_IOC_NR(num) < DRM_COMMAND_END);
}

static int drm_get_driver_name(struct tcb *tcp, char *name, size_t bufsize)
{
	char path[PATH_MAX];
	char link[PATH_MAX];
	int ret;

	ret = getfdpath(tcp, tcp->u_arg[0], path, PATH_MAX - 1);
	if (!ret)
		return ret;

	snprintf(link, PATH_MAX, "/sys/class/drm/%s/device/driver",
		 basename(path));

	ret = readlink(link, path, PATH_MAX - 1);
	if (ret < 0)
		return ret;

	path[ret] = '\0';
	strncpy(name, basename(path), bufsize);

	return 0;
}

int drm_is_driver(struct tcb *tcp, const char *name)
{
	char drv[DRM_MAX_NAME_LEN];
	int ret;

	ret = drm_get_driver_name(tcp, drv, DRM_MAX_NAME_LEN);
	if (ret)
		return 0;

	return strcmp(name, drv) == 0;
}

int drm_ioctl(struct tcb *tcp, const unsigned int code, long arg)
{
	return 0;
}
