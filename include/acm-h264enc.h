/*
 * acm-h264enc.h - header for AV Codec Middleware H264 Encoder Driver
 *
 * Copyright (C) 2013 Atmark Techno, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */
#if !defined(ARMADILLO_ACM_H264ENC_H)
#define ARMADILLO_ACM_H264ENC_H

#if defined(__KERNEL__)
#include <media/v4l2-device.h>
#include <linux/types.h>
#else
#include <stdint.h>
#include <stdbool.h>
#endif

enum acm_h264_rate_control_mode {
	ACM_H264_RATE_CONTROL_MODE_CBR = 0,
	ACM_H264_RATE_CONTROL_MODE_CBR_NO_SKIP = 1,
	ACM_H264_RATE_CONTROL_MODE_VBR = 2,
};

#define V4L2_CID_TARGET_BIT_RATE	(V4L2_CID_PRIVATE_BASE + 0)
#define V4L2_CID_MAX_FRAME_SIZE		(V4L2_CID_PRIVATE_BASE + 1)
#define V4L2_CID_RATE_CONTROL_MODE	(V4L2_CID_PRIVATE_BASE + 2)
#define V4L2_CID_FRAME_RATE_RESOLUTION	(V4L2_CID_PRIVATE_BASE + 3)
#define V4L2_CID_FRAME_RATE_TICK	(V4L2_CID_PRIVATE_BASE + 4)
#define V4L2_CID_MAX_GOP_LENGTH		(V4L2_CID_PRIVATE_BASE + 5)
#define V4L2_CID_B_PIC_MODE		(V4L2_CID_PRIVATE_BASE + 6)

#endif /* !defined(ARMADILLO_ACM_H264ENC_H) */
