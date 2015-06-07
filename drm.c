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

#ifdef HAVE_DRM_H
#include <drm.h>
#else
#include <drm/drm.h>
#endif

#include <sys/param.h>

#define DRM_MAX_NAME_LEN 128

inline int drm_is_priv(const unsigned int num)
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

	snprintf(link, PATH_MAX, "/sys/class/drm/%s/device/driver",
		 basename(path));

	ret = readlink(link, path, PATH_MAX - 1);
	if (ret < 0)
		return NULL;

	path[ret] = '\0';
	return strdup(basename(path));
}

static void drm_free_priv(void *data)
{
	free(data);
}

int drm_is_driver(struct tcb *tcp, const char *name)
{
	char *priv;

	/*
	 * If no private data is allocated we are detecting the driver name for
	 * the first time and must resolve it.
	 */
	if (tcp->priv_data == NULL) {
		tcp->priv_data = drm_get_driver_name(tcp);
		if (tcp->priv_data == NULL)
			return 0;

		tcp->free_priv_data = drm_free_priv;
	}

	priv = tcp->priv_data;

	return strncmp(name, priv, DRM_MAX_NAME_LEN) == 0;
}

int drm_decode_number(struct tcb *tcp, unsigned int code)
{
	if (drm_is_priv(tcp->u_arg[1])) {
		if (verbose(tcp) && drm_is_driver(tcp, "i915"))
			return drm_i915_decode_number(tcp, code);
	}

	return 0;
}

static int drm_version(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_version ver;

	if (exiting(tcp)) {
		if (umove(tcp, arg, &ver))
			return RVAL_DECODED;

		tprintf(", {version_major=%d, version_minor=%d, version_patchlevel=%d, "
			"name_len=%lu, name=", ver.version_major,
			ver.version_minor, ver.version_patchlevel,
			ver.name_len);
		printstr(tcp, (long)ver.name, ver.name_len);
		tprintf(", date_len=%lu, date=", ver.date_len);
		printstr(tcp, (long)ver.date, ver.date_len);
		tprintf(", desc_len=%lu, desc=", ver.desc_len);
		printstr(tcp, (long)ver.desc, ver.desc_len);
		tprints("}");
	}

	return RVAL_DECODED | 1;
}

static int drm_get_unique(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_unique unique;

	if (exiting(tcp)) {
		if (umove(tcp, arg, &unique))
			return RVAL_DECODED;

		tprintf(", {unique_len=%lu, unique=", unique.unique_len);
		printstr(tcp, (long)unique.unique, unique.unique_len);
		tprints("}");
	}

	return RVAL_DECODED | 1;
}

static int drm_get_magic(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_auth auth;

	if (exiting(tcp)) {
		if (umove(tcp, arg, &auth))
			return RVAL_DECODED;

		tprintf(", {magic=%u}", auth.magic);
	}

	return RVAL_DECODED | 1;
}

static int drm_wait_vblank(struct tcb *tcp, const unsigned int code, long arg)
{
	union drm_wait_vblank vblank;

	if (umove(tcp, arg, &vblank))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {request={type=%u, sequence=%u, signal=%lu}",
			vblank.request.type, vblank.request.sequence,
			vblank.request.signal);
	} else if (exiting(tcp)) {
		tprintf(", reply={type=%u, sequence=%u, tval_sec=%ld, tval_usec=%ld}}",
			vblank.reply.type, vblank.reply.sequence,
			vblank.reply.tval_sec, vblank.reply.tval_usec);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_get_resources(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_card_res res;

	if (exiting(tcp)) {
		if (umove(tcp, arg, &res))
			return RVAL_DECODED;

		tprintf(", {fb_id_ptr=%p, crtc_id_ptr=%p, connector_id_ptr=%p, "
			"encoder_id_ptr=%p, count_fbs=%u, count_crtcs=%u, "
			"count_connectors=%u, count_encoders=%u, min_width=%u, "
			"max_width=%u, min_height=%u, max_height=%u}",
			(void *)res.fb_id_ptr, (void *)res.crtc_id_ptr,
			(void *)res.connector_id_ptr, (void *)res.encoder_id_ptr,
			res.count_fbs, res.count_crtcs, res.count_connectors,
			res.count_encoders, res.min_width, res.max_width,
			res.min_height, res.max_height);
	}

	return RVAL_DECODED | 1;
}

static void drm_mode_print_modeinfo(struct drm_mode_modeinfo *info)
{
	tprintf("clock=%u, hdisplay=%hu, hsync_start=%hu, hsync_end=%hu, "
		"htotal=%hu, hskew=%hu, vdisplay=%hu, vsync_start=%hu, "
		"vsync_end=%hu, vtotal=%hu, vscan=%hu, vrefresh=%u, "
		"flags=0x%x, type=%u, name=%s", info->clock, info->hdisplay,
		info->hsync_start, info->hsync_end, info->htotal, info->hskew,
		info->vdisplay, info->vsync_start, info->vsync_end,
		info->vtotal, info->vscan, info->vrefresh, info->flags,
		info->type, info->name);
}

static int drm_mode_get_crtc(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_crtc crtc;

	if (umove(tcp, arg, &crtc))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {crtc_id=%u", crtc.crtc_id);
	} else if (exiting(tcp)) {
		tprintf(", set_connectors_ptr=%p, count_connectors=%u, "
			"fb_id=%u, x=%u, y=%u, gamma_size=%u, mode_valid=%u, "
			"mode={", (void *)crtc.set_connectors_ptr,
			crtc.count_connectors, crtc.fb_id, crtc.x, crtc.y,
			crtc.gamma_size, crtc.mode_valid);

		drm_mode_print_modeinfo(&crtc.mode);
		tprintf("}}");
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_set_crtc(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_crtc crtc;

	if (entering(tcp)) {
		if (umove(tcp, arg, &crtc))
			return RVAL_DECODED;

		tprintf(", {set_connectors_ptr=%p, count_connectors=%u, "
			"crtc_id=%u, fb_id=%u, x=%u, y=%u, gamma_size=%u, "
			"mode_valid=%u, mode={", (void *)crtc.set_connectors_ptr,
			crtc.count_connectors, crtc.crtc_id, crtc.fb_id, crtc.x,
			crtc.y, crtc.gamma_size, crtc.mode_valid);

		drm_mode_print_modeinfo(&crtc.mode);
		tprintf("}}");
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_cursor(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_cursor cursor;


	if (entering(tcp)) {
		if (umove(tcp, arg, &cursor))
			return RVAL_DECODED;

		tprintf(", {flags=0x%x, crtc_id=%u, x=%d, y=%d, width=%u, "
			"height=%u, handle=%u}", cursor.flags, cursor.crtc_id,
			cursor.x, cursor.y, cursor.width, cursor.height,
			cursor.handle);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_cursor2(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_cursor2 cursor;

	if (entering(tcp)) {
		if (umove(tcp, arg, &cursor))
			return RVAL_DECODED;

		tprintf(", {flags=0x%x, crtc_id=%u, x=%d, y=%d, width=%u, "
			"height=%u, handle=%u, hot_x=%d, hot_y=%d}",
			cursor.flags, cursor.crtc_id, cursor.x, cursor.y,
			cursor.width, cursor.height, cursor.handle,
			cursor.hot_x, cursor.hot_y);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_get_gamma(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_crtc_lut lut;


	if (entering(tcp)) {
		if (umove(tcp, arg, &lut))
			return RVAL_DECODED;

		/* We don't print the entire table, just the pointers */
		tprintf(", {crtc_id=%u, gamma_size=%u, red=%p, green=%p, "
			"blue=%p}", lut.crtc_id, lut.gamma_size,
			(void *)lut.red, (void *)lut.green, (void *)lut.blue);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_set_gamma(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_crtc_lut lut;

	if (entering(tcp)) {
		if (umove(tcp, arg, &lut))
			return RVAL_DECODED;

		/* We don't print the entire table, just the rgb pointers */
		tprintf(", {crtc_id=%u, gamma_size=%u, red=%p, green=%p, "
			"blue=%p}", lut.crtc_id, lut.gamma_size,
			(void *)lut.red, (void *)lut.green, (void *)lut.blue);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_get_encoder(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_get_encoder enc;

	if (umove(tcp, arg, &enc))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {encoder_id=%u", enc.encoder_id);
	} else if (exiting(tcp)) {
		/* TODO: Print name of encoder type */
		tprintf(", encoder_type=%u, crtc_id=%u, possible_crtcs=0x%x, "
			"possible_clones=0x%x}", enc.encoder_type,
			enc.crtc_id, enc.possible_crtcs, enc.possible_clones);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_get_connector(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_get_connector con;

	if (umove(tcp, arg, &con))
		return RVAL_DECODED;

	/* We could be very verbose here but keep is simple for now */
	if (entering(tcp)) {
		tprintf(", {connector_id=%u", con.connector_id);
	} else if (exiting(tcp)) {
		tprintf(", encoders_ptr=%p, modes_ptr=%p, props_ptr=%p, "
			"prop_values_ptr=%p, count_modes=%u, count_props=%u, "
			"count_encoders=%u, encoder_id=%u, connector_type=%u, "
			"connector_type_id=%u, connection=%u, mm_width=%u, "
			"mm_height=%u, subpixel=%u}", (void *)con.encoders_ptr,
			(void *)con.modes_ptr, (void *)con.props_ptr,
			(void *)con.prop_values_ptr, con.count_modes,
			con.count_props, con.count_encoders, con.encoder_id,
			con.connector_type, con.connector_type_id,
			con.connection, con.mm_width, con.mm_height,
			con.subpixel);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_get_property(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_get_property prop;

	if (umove(tcp, arg, &prop))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {prop_id=%u", prop.prop_id);
	} else if (exiting(tcp)) {
		tprintf(", values_ptr=%p, enum_blob_ptr=%p, flags=0x%x, "
			"name=%s, count_values=%u, count_enum_blobs=%u}",
			(void *)prop.values_ptr, (void *)prop.enum_blob_ptr,
			prop.flags, prop.name, prop.count_values,
			prop.count_enum_blobs);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_set_property(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_connector_set_property prop;

	if (entering(tcp)) {
		if (umove(tcp, arg, &prop))
			return RVAL_DECODED;

		tprintf(", {value=%Lu, prop_id=%u, connector_id=%u}",
			prop.value, prop.prop_id, prop.connector_id);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_get_prop_blob(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_get_blob blob;

	if (umove(tcp, arg, &blob))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {blob_id=%u", blob.blob_id);
	} else if (exiting(tcp)) {
		tprintf(", length=%u, data=%p}", blob.length,
			(void *)blob.data);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_add_fb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_fb_cmd cmd;

	if (umove(tcp, arg, &cmd))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {width=%u, height=%u, pitch=%u, bpp=%u, depth=%u, "
			"handle=%u", cmd.width, cmd.height, cmd.pitch,
			cmd.bpp, cmd.depth, cmd.handle);
	} else if (exiting(tcp)) {
		tprintf(", fb_id=%u}", cmd.fb_id);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_get_fb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_fb_cmd cmd;

	if (umove(tcp, arg, &cmd))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {fb_id=%u", cmd.fb_id);
	} else {
		tprintf(", width=%u, height=%u, pitch=%u, bpp=%u, depth=%u, "
		"handle=%u}", cmd.width, cmd.height, cmd.pitch,
		cmd.bpp, cmd.depth, cmd.handle);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_rm_fb(struct tcb *tcp, const unsigned int code, long arg)
{
	unsigned int handle;


	if (entering(tcp)) {
		if (umove(tcp, arg, &handle))
			return RVAL_DECODED;

		tprintf(", %u", handle);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_page_flip(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_crtc_page_flip flip;

	if (entering(tcp)) {
		if (umove(tcp, arg, &flip))
			return RVAL_DECODED;

		tprintf(", {crtc_id=%u, fb_id=%u, flags=0x%x, user_data=0x%Lx}",
			flip.crtc_id, flip.fb_id, flip.flags, flip.user_data);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_dirty_fb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_fb_dirty_cmd cmd;


	if (entering(tcp)) {
		if (umove(tcp, arg, &cmd))
			return RVAL_DECODED;

		tprintf(", {fb_id=%u, flags=0x%x, color=0x%x, num_clips=%u, "
			"clips_ptr=%p}", cmd.fb_id, cmd.flags, cmd.color,
			cmd.num_clips, (void *)cmd.clips_ptr);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_create_dumb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_create_dumb dumb;

	if (umove(tcp, arg, &dumb))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {width=%u, height=%u, bpp=%u, flags=0x%x",
			dumb.width, dumb.height, dumb.bpp, dumb.flags);
	} else if (exiting(tcp)) {
		tprintf(", handle=%u, pitch=%u, size=%Lu}", dumb.handle,
			dumb.pitch, dumb.size);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_map_dumb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_map_dumb dumb;

	if (umove(tcp, arg, &dumb))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {handle=%u", dumb.handle);
	} else if (exiting(tcp)) {
		tprintf(", offset=%Lu}", dumb.offset);
	}

	return RVAL_DECODED | 1;
}

static int drm_mode_destroy_dumb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_destroy_dumb dumb;

	if (entering(tcp)) {
		if (umove(tcp, arg, &dumb))
			return RVAL_DECODED;

		tprintf(", {handle=%u}", dumb.handle);
	}

	return RVAL_DECODED | 1;
}

static int drm_gem_close(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_gem_close close;

	if (entering(tcp)) {
		if (umove(tcp, arg, &close))
			return RVAL_DECODED;

		tprintf(", {handle=%u}", close.handle);
	}

	return RVAL_DECODED | 1;
}

int drm_ioctl(struct tcb *tcp, const unsigned int code, long arg)
{
	int ret = 0;

	/* Check for device specific ioctls */
	if (drm_is_priv(tcp->u_arg[1])) {
		if (verbose(tcp) && drm_is_driver(tcp, "i915"))
			ret = drm_i915_ioctl(tcp, code, arg);
	} else {
		switch (code) {
		case DRM_IOCTL_VERSION:
			ret = drm_version(tcp, code, arg);
			break;
		case DRM_IOCTL_GET_UNIQUE:
			ret = drm_get_unique(tcp, code, arg);
			break;
		case DRM_IOCTL_GET_MAGIC:
			ret = drm_get_magic(tcp, code, arg);
			break;
		case DRM_IOCTL_WAIT_VBLANK:
			ret = drm_wait_vblank(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_GETRESOURCES:
			ret = drm_mode_get_resources(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_GETCRTC:
			ret = drm_mode_get_crtc(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_SETCRTC:
			ret = drm_mode_set_crtc(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_CURSOR:
			ret = drm_mode_cursor(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_CURSOR2:
			ret = drm_mode_cursor2(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_GETGAMMA:
			ret = drm_mode_get_gamma(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_SETGAMMA:
			ret = drm_mode_set_gamma(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_GETENCODER:
			ret = drm_mode_get_encoder(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_GETCONNECTOR:
			ret = drm_mode_get_connector(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_GETPROPERTY:
			ret = drm_mode_get_property(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_SETPROPERTY:
			ret = drm_mode_set_property(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_GETPROPBLOB:
			ret = drm_mode_get_prop_blob(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_GETFB:
			ret = drm_mode_get_fb(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_ADDFB:
			ret = drm_mode_add_fb(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_RMFB:
			ret = drm_mode_rm_fb(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_PAGE_FLIP:
			ret = drm_mode_page_flip(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_DIRTYFB:
			ret = drm_mode_dirty_fb(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_CREATE_DUMB:
			ret = drm_mode_create_dumb(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_MAP_DUMB:
			ret = drm_mode_map_dumb(tcp, code, arg);
			break;
		case DRM_IOCTL_MODE_DESTROY_DUMB:
			ret = drm_mode_destroy_dumb(tcp, code, arg);
			break;
		case DRM_IOCTL_GEM_CLOSE:
			ret = drm_gem_close(tcp, code, arg);
			break;
		}
	}

	return ret;
}

#endif /* HAVE_DRM_H || HAVE_DRM_DRM_H */
