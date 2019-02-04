/*
 * Copyright (c) 2019 Patrik Jakobsson <pjakobsson@suse.de>
 *
 * SPDX-License-Identifier: LGPL-2.1-or-later
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

#include MPERS_DEFS

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

static int drm_is_driver(struct tcb *tcp, const char *name)
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

MPERS_PRINTER_DECL(int, drm_decode_number, struct tcb *tcp, unsigned int code)
{
	return 0;
}

MPERS_PRINTER_DECL(int, drm_ioctl, struct tcb *tcp, const unsigned int code,
		   long arg)
{
	return 0;
}

#endif /* HAVE_DRM_H || HAVE_DRM_DRM_H */
