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

static int drm_version(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_version ver;
	char *name, *date, *desc;
	int ret;

	if (entering(tcp) || umove(tcp, arg, &ver))
		return 0;

	name = calloc(ver.name_len + 1, 1);
	if (!name)
		return 0;
	ret = umovestr(tcp, (long)ver.name, ver.name_len, name);
	if (ret < 0)
		goto free_name;

	date = calloc(ver.date_len + 1, 1);
	if (!date)
		goto free_name;
	ret = umovestr(tcp, (long)ver.date, ver.date_len, date);
	if (ret < 0)
		goto free_date;

	desc = calloc(ver.desc_len + 1, 1);
	if (!desc)
		goto free_date;
	ret = umovestr(tcp, (long)ver.desc, ver.desc_len, desc);
	if (ret < 0)
		goto free_desc;

	tprintf(", {version_major=%d, version_minor=%d, version_patchlevel=%d, "
		"name_len=%lu, name=%s, date_len=%lu, date=%s, "
		"desc_len=%lu, desc=%s}",
		ver.version_major, ver.version_minor, ver.version_patchlevel,
		ver.name_len, name, ver.date_len, date, ver.desc_len,
		desc);

free_desc:
	free(desc);
free_date:
	free(date);
free_name:
	free(name);

	if (ret < 0)
		return 0;

	return 1;
}

static int drm_get_unique(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_unique unique;
	char *str;
	int ret;

	if (entering(tcp) || umove(tcp, arg, &unique))
		return 0;

	str = calloc(unique.unique_len + 1, 1);
	if (!str)
		return 0;
	ret = umovestr(tcp, (long)unique.unique, unique.unique_len, str);

	tprintf(", {unique_len=%lu, unique=%s}", unique.unique_len, str);

	free(str);

	if (ret < 0)
		return 0;

	return 1;
}

static int drm_get_magic(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_auth auth;

	if (entering(tcp) || umove(tcp, arg, &auth))
		return 0;

	tprintf(", {magic=%u}", auth.magic);

	return 1;
}

static int drm_wait_vblank(struct tcb *tcp, const unsigned int code, long arg)
{
	union drm_wait_vblank vblank;

	if (umove(tcp, arg, &vblank))
		return 0;

	if (entering(tcp)) {
		tprintf(", {request={type=%u, sequence=%u, signal=%lu}",
			vblank.request.type, vblank.request.sequence,
			vblank.request.signal);
	} else {
		tprintf(", reply={type=%u, sequence=%u, tval_sec=%ld, tval_usec=%ld}}",
			vblank.reply.type, vblank.reply.sequence,
			vblank.reply.tval_sec, vblank.reply.tval_usec);
	}

	return 1;
}

static int drm_mode_get_resources(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_card_res res;

	if (entering(tcp) || umove(tcp, arg, &res))
		return 0;

	tprintf(", {fb_id_ptr=%p, crtc_id_ptr=%p, connector_id_ptr=%p, "
		"encoder_id_ptr=%p, count_fbs=%u, count_crtcs=%u, "
		"count_connectors=%u, count_encoders=%u, min_width=%u, "
		"max_width=%u, min_height=%u, max_height=%u}",
		(void *)res.fb_id_ptr, (void *)res.crtc_id_ptr,
		(void *)res.connector_id_ptr, (void *)res.encoder_id_ptr,
		res.count_fbs, res.count_crtcs, res.count_connectors,
		res.count_encoders, res.min_width, res.max_width,
		res.min_height, res.max_height);

	return 1;
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

	if (entering(tcp) || umove(tcp, arg, &crtc))
		return 0;

	tprintf(", {set_connectors_ptr=%p, count_connectors=%u, crtc_id=%u, "
		"fb_id=%u, x=%u, y=%u, gamma_size=%u, mode_valid=%u, mode={",
		(void *)crtc.set_connectors_ptr, crtc.count_connectors,
		crtc.crtc_id, crtc.fb_id, crtc.x, crtc.y, crtc.gamma_size,
		crtc.mode_valid);

	drm_mode_print_modeinfo(&crtc.mode);
	tprintf("}}");

	return 1;
}

static int drm_mode_set_crtc(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_crtc crtc;

	if (entering(tcp) || umove(tcp, arg, &crtc))
		return 0;

	tprintf(", {set_connectors_ptr=%p, count_connectors=%u, crtc_id=%u, "
		"fb_id=%u, x=%u, y=%u, gamma_size=%u, mode_valid=%u, mode={",
		(void *)crtc.set_connectors_ptr, crtc.count_connectors,
		crtc.crtc_id, crtc.fb_id, crtc.x, crtc.y, crtc.gamma_size,
		crtc.mode_valid);

	drm_mode_print_modeinfo(&crtc.mode);
	tprintf("}}");

	return 1;
}

static int drm_mode_cursor(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_cursor cursor;

	if (entering(tcp) || umove(tcp, arg, &cursor))
		return 0;

	tprintf(", {flags=0x%x, crtc_id=%u, x=%d, y=%d, width=%u, height=%u, "
		"handle=%u}", cursor.flags, cursor.crtc_id, cursor.x, cursor.y,
		cursor.width, cursor.height, cursor.handle);

	return 1;
}

static int drm_mode_cursor2(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_cursor2 cursor;

	if (entering(tcp) || umove(tcp, arg, &cursor))
		return 0;

	tprintf(", {flags=0x%x, crtc_id=%u, x=%d, y=%d, width=%u, height=%u, "
		"handle=%u, hot_x=%d, hot_y=%d}", cursor.flags, cursor.crtc_id,
		cursor.x, cursor.y, cursor.width, cursor.height, cursor.handle,
		cursor.hot_x, cursor.hot_y);

	return 1;
}

static int drm_mode_get_gamma(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_crtc_lut lut;

	if (entering(tcp) || umove(tcp, arg, &lut))
		return 0;

	/* We don't print the entire table, just the pointers */
	tprintf(", {crtc_id=%u, gamma_size=%u, red=%p, green=%p, blue=%p}",
		lut.crtc_id, lut.gamma_size, (void *)lut.red, (void *)lut.green,
		(void *)lut.blue);

	return 1;
}


static int drm_mode_set_gamma(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_crtc_lut lut;

	if (entering(tcp) || umove(tcp, arg, &lut))
		return 0;

	/* We don't print the entire table, just the rgb pointers */
	tprintf(", {crtc_id=%u, gamma_size=%u, red=%p, green=%p, blue=%p}",
		lut.crtc_id, lut.gamma_size, (void *)lut.red, (void *)lut.green,
		(void *)lut.blue);

	return 1;
}

static int drm_mode_get_encoder(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_get_encoder enc;

	if (entering(tcp) || umove(tcp, arg, &enc))
		return 0;

	/* TODO: Print name of encoder type */
	tprintf(", {encoder_id=%u, encoder_type=%u, crtc_id=%u, "
		"possible_crtcs=0x%x, possible_clones=0x%x}", enc.encoder_id,
		enc.encoder_type, enc.crtc_id, enc.possible_crtcs,
		enc.possible_clones);

	return 1;
}

static int drm_mode_get_connector(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_get_connector con;

	if (entering(tcp) || umove(tcp, arg, &con))
		return 0;

	/* We could be very verbose here but keep is simple for now */
	tprintf(", {encoders_ptr=%p, modes_ptr=%p, props_ptr=%p, "
		"prop_values_ptr=%p, count_modes=%u, count_props=%u, "
		"count_encoders=%u, encoder_id=%u, connector_id=%u, "
		"connector_type=%u, connector_type_id=%u, connection=%u, "
		"mm_width=%u, mm_height=%u, subpixel=%u}",
		(void *)con.encoders_ptr, (void *)con.modes_ptr,
		(void *)con.props_ptr, (void *)con.prop_values_ptr,
		con.count_modes, con.count_props, con.count_encoders,
		con.encoder_id, con.connector_id, con.connector_type,
		con.connector_type_id, con.connection, con.mm_width,
		con.mm_height, con.subpixel);

	return 1;
}

static int drm_mode_attachmode(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_mode_cmd cmd;

	if (entering(tcp) || umove(tcp, arg, &cmd))
		return 0;

	tprintf(", {connector_id=%u, mode={", cmd.connector_id);
	drm_mode_print_modeinfo(&cmd.mode);
	tprintf("}");

	return 1;
}

static int drm_mode_detachmode(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_mode_cmd cmd;

	if (entering(tcp) || umove(tcp, arg, &cmd))
		return 0;

	tprintf(", {connector_id=%u, mode={", cmd.connector_id);
	drm_mode_print_modeinfo(&cmd.mode);
	tprintf("}");

	return 1;
}

static int drm_mode_get_property(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_get_property prop;

	if (entering(tcp) || umove(tcp, arg, &prop))
		return 0;

	tprintf(", {values_ptr=%p, enum_blob_ptr=%p, prop_id=%u, flags=0x%x, "
		"name=%s, count_values=%u, count_enum_blobs=%u}",
		(void *)prop.values_ptr, (void *)prop.enum_blob_ptr,
		prop.prop_id, prop.flags, prop.name, prop.count_values,
		prop.count_enum_blobs);

	return 1;
}

static int drm_mode_set_property(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_connector_set_property prop;

	if (entering(tcp) || umove(tcp, arg, &prop))
		return 0;

	tprintf(", {value=%Lu, prop_id=%u, connector_id=%u}", prop.value,
		prop.prop_id, prop.connector_id);

	return 1;
}

static int drm_mode_get_prop_blob(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_get_blob blob;

	if (entering(tcp) || umove(tcp, arg, &blob))
		return 0;

	tprintf(", {blob_id=%u, length=%u, data=%p}", blob.blob_id, blob.length,
		(void *)blob.data);

	return 1;
}

static int drm_mode_add_fb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_fb_cmd cmd;

	if (entering(tcp) || umove(tcp, arg, &cmd))
		return 0;

	tprintf(", {fb_id=%u, width=%u, height=%u, pitch=%u, bpp=%u, depth=%u, "
		"handle=%u}", cmd.fb_id, cmd.width, cmd.height, cmd.pitch,
		cmd.bpp, cmd.depth, cmd.handle);

	return 1;
}

static int drm_mode_get_fb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_fb_cmd cmd;

	if (entering(tcp) || umove(tcp, arg, &cmd))
		return 0;

	tprintf(", {fb_id=%u, width=%u, height=%u, pitch=%u, bpp=%u, depth=%u, "
		"handle=%u}", cmd.fb_id, cmd.width, cmd.height, cmd.pitch,
		cmd.bpp, cmd.depth, cmd.handle);

	return 1;
}

static int drm_mode_rm_fb(struct tcb *tcp, const unsigned int code, long arg)
{
	unsigned int handle;

	if (entering(tcp) || umove(tcp, arg, &handle))
		return 0;

	tprintf(", %u", handle);

	return 1;
}

static int drm_mode_page_flip(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_crtc_page_flip flip;

	if (entering(tcp) || umove(tcp, arg, &flip))
		return 0;

	tprintf(", {crtc_id=%u, fb_id=%u, flags=0x%x, user_data=0x%Lx}",
		flip.crtc_id, flip.fb_id, flip.flags, flip.user_data);

	return 1;
}

static int drm_mode_dirty_fb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_fb_dirty_cmd cmd;

	if (entering(tcp) || umove(tcp, arg, &cmd))
		return 0;

	tprintf(", {fb_id=%u, flags=0x%x, color=0x%x, num_clips=%u, "
		"clips_ptr=%p}", cmd.fb_id, cmd.flags, cmd.color, cmd.num_clips,
		(void *)cmd.clips_ptr);

	return 1;
}

static int drm_mode_create_dumb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_create_dumb dumb;

	if (entering(tcp) || umove(tcp, arg, &dumb))
		return 0;

	tprintf(", {height=%u, width=%u, bpp=%u, flags=0x%x, handle=%u, "
		"pitch=%u, size=%Lu}", dumb.height, dumb.width, dumb.bpp,
		dumb.flags, dumb.handle, dumb.pitch, dumb.size);

	return 1;
}

static int drm_mode_map_dumb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_map_dumb dumb;

	if (entering(tcp) || umove(tcp, arg, &dumb))
		return 0;

	tprintf(", {handle=%u, offset=%Lu}", dumb.handle, dumb.offset);

	return 1;
}

static int drm_mode_destroy_dumb(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_mode_destroy_dumb dumb;

	if (entering(tcp) || umove(tcp, arg, &dumb))
		return 0;

	tprintf(", {handle=%u}", dumb.handle);

	return 1;
}

static int drm_gem_close(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_gem_close close;

	if (entering(tcp) || umove(tcp, arg, &close))
		return 0;

	tprintf(", {handle=%u}", close.handle);

	return 1;
}

int drm_ioctl(struct tcb *tcp, const unsigned int code, long arg)
{
	/* Check for device specific ioctls */
	if (drm_is_priv(tcp->u_arg[1])) {
		if (verbose(tcp) && drm_is_driver(tcp, "i915"))
			return drm_i915_ioctl(tcp, code, arg);
	}

	switch (code) {
	case DRM_IOCTL_VERSION:
		return drm_version(tcp, code, arg);
	case DRM_IOCTL_GET_UNIQUE:
		return drm_get_unique(tcp, code, arg);
	case DRM_IOCTL_GET_MAGIC:
		return drm_get_magic(tcp, code, arg);
	case DRM_IOCTL_WAIT_VBLANK:
		return drm_wait_vblank(tcp, code, arg);
	case DRM_IOCTL_MODE_GETRESOURCES:
		return drm_mode_get_resources(tcp, code, arg);
	case DRM_IOCTL_MODE_GETCRTC:
		return drm_mode_get_crtc(tcp, code, arg);
	case DRM_IOCTL_MODE_SETCRTC:
		return drm_mode_set_crtc(tcp, code, arg);
	case DRM_IOCTL_MODE_CURSOR:
		return drm_mode_cursor(tcp, code, arg);
	case DRM_IOCTL_MODE_CURSOR2:
		return drm_mode_cursor2(tcp, code, arg);
	case DRM_IOCTL_MODE_GETGAMMA:
		return drm_mode_get_gamma(tcp, code, arg);
	case DRM_IOCTL_MODE_SETGAMMA:
		return drm_mode_set_gamma(tcp, code, arg);
	case DRM_IOCTL_MODE_GETENCODER:
		return drm_mode_get_encoder(tcp, code, arg);
	case DRM_IOCTL_MODE_GETCONNECTOR:
		return drm_mode_get_connector(tcp, code, arg);
	case DRM_IOCTL_MODE_ATTACHMODE:
		return drm_mode_attachmode(tcp, code, arg);
	case DRM_IOCTL_MODE_DETACHMODE:
		return drm_mode_detachmode(tcp, code, arg);
	case DRM_IOCTL_MODE_GETPROPERTY:
		return drm_mode_get_property(tcp, code, arg);
	case DRM_IOCTL_MODE_SETPROPERTY:
		return drm_mode_set_property(tcp, code, arg);
	case DRM_IOCTL_MODE_GETPROPBLOB:
		return drm_mode_get_prop_blob(tcp, code, arg);
	case DRM_IOCTL_MODE_GETFB:
		return drm_mode_get_fb(tcp, code, arg);
	case DRM_IOCTL_MODE_ADDFB:
		return drm_mode_add_fb(tcp, code, arg);
	case DRM_IOCTL_MODE_RMFB:
		return drm_mode_rm_fb(tcp, code, arg);
	case DRM_IOCTL_MODE_PAGE_FLIP:
		return drm_mode_page_flip(tcp, code, arg);
	case DRM_IOCTL_MODE_DIRTYFB:
		return drm_mode_dirty_fb(tcp, code, arg);
	case DRM_IOCTL_MODE_CREATE_DUMB:
		return drm_mode_create_dumb(tcp, code, arg);
	case DRM_IOCTL_MODE_MAP_DUMB:
		return drm_mode_map_dumb(tcp, code, arg);
	case DRM_IOCTL_MODE_DESTROY_DUMB:
		return drm_mode_destroy_dumb(tcp, code, arg);
	case DRM_IOCTL_GEM_CLOSE:
		return drm_gem_close(tcp, code, arg);
	}

	return 0;
}
