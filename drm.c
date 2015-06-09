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

#include <drm.h>
#include <linux/limits.h>

#define DRM_MAX_NAME_LEN 128

extern int drm_i915_decode_number(struct tcb *tcp, unsigned int arg);
extern int drm_i915_ioctl(struct tcb *tcp, const unsigned int code, long arg);

struct drm_ioctl_priv {
	char name[DRM_MAX_NAME_LEN];
};

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
	if (ret < 0)
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
	struct drm_ioctl_priv *priv;
	int ret;

	/*
	 * If no private data is allocated we are detecting the driver name for
	 * the first time and must resolve it.
	 */
	if (tcp->priv_data == NULL) {
		tcp->priv_data = xcalloc(1, sizeof(struct drm_ioctl_priv));
		priv = tcp->priv_data;

		ret = drm_get_driver_name(tcp, priv->name, DRM_MAX_NAME_LEN);
		if (ret)
			return 0;
	}

	priv = tcp->priv_data;

	return strncmp(name, priv->name, DRM_MAX_NAME_LEN) == 0;
}

int drm_decode_number(struct tcb *tcp, unsigned int arg)
{
	if (drm_is_priv(tcp->u_arg[1])) {
		if (verbose(tcp) && drm_is_driver(tcp, "i915"))
			return drm_i915_decode_number(tcp, arg);
	}

	return 0;
}

int drm_ioctl(struct tcb *tcp, const unsigned int code, long arg)
{
	int ret = 0;

	/* Check for device specific ioctls */
	if (drm_is_priv(tcp->u_arg[1])) {
		if (verbose(tcp) && drm_is_driver(tcp, "i915"))
			ret = drm_i915_ioctl(tcp, code, arg);
	}

	/* Free any allocated private data */
	if (exiting(tcp) && tcp->priv_data != NULL) {
		free(tcp->priv_data);
		tcp->priv_data = NULL;
	}

	return ret;
}
