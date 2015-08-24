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

#if defined(HAVE_DRM_H) || defined(HAVE_DRM_DRM_H)

#include <sys/param.h>

#ifdef HAVE_DRM_H
#include <drm.h>
#else
#include <drm/drm.h>
#endif

#define DRM_MAX_NAME_LEN 128

static inline int drm_is_priv(const unsigned int num)
{
	return (_IOC_NR(num) >= DRM_COMMAND_BASE &&
		_IOC_NR(num) < DRM_COMMAND_END);
}

static char *drm_get_driver_name(struct tcb *tcp)
{
	char path[PATH_MAX];
	char link[PATH_MAX];
	int ret;

	if (getfdpath(tcp, tcp->u_arg[0], path, PATH_MAX - 1) < 0)
		return NULL;

	if (snprintf(link, PATH_MAX, "/sys/class/drm/%s/device/driver",
	    basename(path)) >= (signed int)sizeof(link))
		return NULL;

	ret = readlink(link, path, PATH_MAX - 1);
	if (ret < 0)
		return NULL;

	path[ret] = '\0';
	return strdup(basename(path));
}

int drm_is_driver(struct tcb *tcp, const char *name)
{
	char *priv;

	/*
	 * If no private data is allocated we are detecting the driver name for
	 * the first time and must resolve it.
	 */
	if (tcp->_priv_data == NULL) {
		priv = drm_get_driver_name(tcp);

		if (priv == NULL)
			return 0;

		set_tcb_priv_data(tcp, priv, free);
	}

	return strncmp(name, get_tcb_priv_data(tcp), DRM_MAX_NAME_LEN) == 0;
}

int drm_decode_number(struct tcb *tcp, unsigned int code)
{
	return 0;
}

int drm_ioctl(struct tcb *tcp, const unsigned int code, long arg)
{
	return 0;
}

#endif /* HAVE_DRM_H || HAVE_DRM_DRM_H */
