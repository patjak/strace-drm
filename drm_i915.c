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
#include <i915_drm.h>
#else
#include <drm/drm.h>
#include <drm/i915_drm.h>
#endif

#include "xlat/drm_i915_ioctls.h"
#include "xlat/drm_i915_getparams.h"
#include "xlat/drm_i915_setparams.h"

static int i915_getparam(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_getparam param;
	int value;

	if (umove(tcp, arg, &param))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprints(", {param=");
		printxval(drm_i915_getparams, param.param, "I915_PARAM_???");
	} else if (exiting(tcp)) {
		if (umove(tcp, (long)param.value, &value))
			return RVAL_DECODED;

		tprints(", value=");
		switch (param.param) {
		case I915_PARAM_CHIPSET_ID:
			tprintf("0x%04x", value);
			break;
		default:
			tprintf("%d", value);
		}
		tprints("}");
	}

	return RVAL_DECODED | 1;
}

static int i915_setparam(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_setparam param;

	if (entering(tcp)) {
		if (umove(tcp, arg, &param))
			return RVAL_DECODED;

		tprints(", {param=");
		printxval(drm_i915_setparams, param.param, "I915_PARAM_???");
		tprintf(", value=%d}", param.value);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_execbuffer2(struct tcb *tcp, const unsigned int code,
				long arg)
{
	struct drm_i915_gem_execbuffer2 eb;

	if (entering(tcp)) {
		if (umove(tcp, arg, &eb))
			return RVAL_DECODED;

		tprintf(", {buffers_ptr=%p, buffer_count=%u, "
			"batch_start_offset=%x, batch_len=%u, DR1=%u, DR4=%u, "
			"num_cliprects=%u, cliprects_ptr=%p, flags=0x%Lx}",
			(void *)eb.buffers_ptr, eb.buffer_count,
			eb.batch_start_offset, eb.batch_len, eb.DR1, eb.DR4,
			eb.num_cliprects, (void *)eb.cliprects_ptr, eb.flags);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_busy(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_gem_busy busy;

	if (umove(tcp, arg, &busy))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {handle=%u", busy.handle);
	} else if (exiting(tcp)) {
		tprintf(", busy=%c, ring=%u}",
			(busy.busy & 0x1) ? 'Y' : 'N', (busy.busy >> 16));
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_create(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_gem_create create;

	if (umove(tcp, arg, &create))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {size=%Lu", create.size);
	} else if (exiting(tcp)) {
		tprintf(", handle=%u}", create.handle);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_pread(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_gem_pread pr;


	if (entering(tcp)) {
		if (umove(tcp, arg, &pr))
			return RVAL_DECODED;

		tprintf(", {handle=%u, offset=%Lu, size=%Lu, data_ptr=%p}",
			pr.handle, pr.offset, pr.size, (void *)pr.data_ptr);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_pwrite(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_gem_pwrite pw;

	if (entering(tcp)) {
		if (umove(tcp, arg, &pw))
			return RVAL_DECODED;

		tprintf(", {handle=%u, offset=%Lu, size=%Lu, data_ptr=%p}",
			pw.handle, pw.offset, pw.size, (void *)pw.data_ptr);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_mmap(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_gem_mmap mmap;

	if (umove(tcp, arg, &mmap))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {handle=%u, size=%Lu", mmap.handle, mmap.size);
	} else if (exiting(tcp)) {
		tprintf(", offset=%Lu, addr_ptr=%p}",
			mmap.offset, (void *)mmap.addr_ptr);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_mmap_gtt(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_gem_mmap_gtt mmap;

	if (umove(tcp, arg, &mmap))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {handle=%u", mmap.handle);
	} else if (exiting(tcp)) {
		tprintf(", offset=%Lu}", mmap.offset);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_set_domain(struct tcb *tcp, const unsigned int code,
			       long arg)
{
	struct drm_i915_gem_set_domain dom;

	if (entering(tcp)) {
		if (umove(tcp, arg, &dom))
			return RVAL_DECODED;

		tprintf(", {handle=%u, read_domains=%x, write_domain=%x}",
			dom.handle, dom.read_domains, dom.write_domain);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_madvise(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_gem_madvise madv;

	if (umove(tcp, arg, &madv))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {handle=%u, madv=%u", madv.handle, madv.madv);
	} else if (exiting(tcp)) {
		tprintf(", retained=%u}", madv.retained);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_get_tiling(struct tcb *tcp, const unsigned int code,
			       long arg)
{
	struct drm_i915_gem_get_tiling tiling;

	if (umove(tcp, arg, &tiling))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {handle=%u", tiling.handle);
	} else if (exiting(tcp)) {
		tprintf(", tiling_mode=%u, swizzle_mode=%u}", tiling.tiling_mode,
			tiling.swizzle_mode);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_set_tiling(struct tcb *tcp, const unsigned int code,
			       long arg)
{
	struct drm_i915_gem_set_tiling tiling;

	if (umove(tcp, arg, &tiling))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {handle=%u, tiling_mode=%u, stride=%u",
			tiling.handle, tiling.tiling_mode, tiling.stride);
	} else if (exiting(tcp)) {
		tprintf(", swizzle_mode=%u}", tiling.swizzle_mode);
	}

	return RVAL_DECODED | 1;
}

static int i915_gem_userptr(struct tcb *tcp, const unsigned int code, long arg)
{
	struct drm_i915_gem_userptr uptr;

	if (umove(tcp, arg, &uptr))
		return RVAL_DECODED;

	if (entering(tcp)) {
		tprintf(", {user_ptr=%p, user_size=%Lu", (void *)uptr.user_ptr,
			uptr.user_size);
	} else if (exiting(tcp)) {
		tprintf(", flags=0x%x, handle=%u}", uptr.flags, uptr.handle);
	}

	return RVAL_DECODED | 1;
}

int drm_i915_decode_number(struct tcb *tcp, unsigned int arg)
{
	const char *str = xlookup(drm_i915_ioctls, arg);

	if (str) {
		tprintf("%s", str);
		return 1;
	}

	return 0;
}

int drm_i915_ioctl(struct tcb *tcp, const unsigned int code, long arg)
{
	switch (code) {
	case DRM_IOCTL_I915_GETPARAM:
		return i915_getparam(tcp, code, arg);
	case DRM_IOCTL_I915_SETPARAM:
		return i915_setparam(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_EXECBUFFER2:
		return i915_gem_execbuffer2(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_BUSY:
		return i915_gem_busy(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_CREATE:
		return i915_gem_create(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_PREAD:
		return i915_gem_pread(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_PWRITE:
		return i915_gem_pwrite(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_MMAP:
		return i915_gem_mmap(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_MMAP_GTT:
		return i915_gem_mmap_gtt(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_SET_DOMAIN:
		return i915_gem_set_domain(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_MADVISE:
		return i915_gem_madvise(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_GET_TILING:
		return i915_gem_get_tiling(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_SET_TILING:
		return i915_gem_set_tiling(tcp, code, arg);
	case DRM_IOCTL_I915_GEM_USERPTR:
		return i915_gem_userptr(tcp, code, arg);
	}

	return RVAL_DECODED;
}

#endif /* HAVE_DRM_H || HAVE_DRM_DRM_H */
