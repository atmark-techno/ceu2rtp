#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <unistd.h>
#include <gst/gst.h>
#include <gst/app/gstappsrc.h>
#include <gst/gstatomicqueue.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <linux/videodev2.h>
#include <acm-h264enc.h>

#define C2R_CAM_DEVICE "/dev/video1"
#define C2R_CAM_WIDTH  (1920)
#define C2R_CAM_HEIGHT (1080)
#define C2R_CAM_MMAP_COUNT (3)

#define C2R_ENC_DEVICE "/dev/video2"
#define C2R_ENC_WIDTH  (1920)
#define C2R_ENC_HEIGHT (1080)
#define C2R_ENC_OUTPUT_MMAP_COUNT  (3)
#define C2R_ENC_CAPTURE_MMAP_COUNT (4)

enum {
	GST_ACMH264ENC_RATE_CONTROL_MODE_UNKNOWN      = -1,
	GST_ACMH264ENC_RATE_CONTROL_MODE_CBR_SKIP     = 0,
	GST_ACMH264ENC_RATE_CONTROL_MODE_CBR_NON_SKIP = 1,
	GST_ACMH264ENC_RATE_CONTROL_MODE_VBR_NON_SKIP = 2,
};

enum {
	GST_ACMH264ENC_B_PIC_MODE_UNKNOWN = -1,
	GST_ACMH264ENC_B_PIC_MODE_0_B_PIC = 0,
	GST_ACMH264ENC_B_PIC_MODE_1_B_PIC = 1,
	GST_ACMH264ENC_B_PIC_MODE_2_B_PIC = 2,
	GST_ACMH264ENC_B_PIC_MODE_3_B_PIC = 3,
};

#define DEFAULT_BITRATE        (8000000) /* default 8Mbps */
#define DEFAULT_MAX_FRAME_SIZE (2000000)
#define DEFAULT_RATE_CONTROL_MODE     GST_ACMH264ENC_RATE_CONTROL_MODE_VBR_NON_SKIP
#define DEFAULT_FRAME_RATE_RESOLUTION (30)
#define DEFAULT_FRAME_RATE_TICK (1)
#define DEFAULT_MAX_GOP_LENGTH  (30)
#define DEFAULT_B_PIC_MODE (3)
#define DEFAULT_X_OFFSET   (0)
#define DEFAULT_Y_OFFSET   (0)
#define DEFAULT_FPS_N      (30)
#define DEFAULT_FPS_D      (1)
#define DEFAULT_IP_ADDR    "127.0.0.1"
#define DEFAULT_PORT       (5004)

#define DEFAULT_MAX_SIZE_SECOND  (1)
#define DEFAULT_MAX_BYTES ((DEFAULT_BITRATE / 8) * DEFAULT_MAX_SIZE_SECOND)

typedef struct _C2rMmapInfo {
	void   *addr;
	size_t  size;
} C2rMmapInfo;

typedef struct _C2rIndexQueue {
	GstAtomicQueue *queue;
} C2rIndexQueue;

typedef struct _C2rV4l2Buf {
	enum v4l2_buf_type type;        /* V4L2 memory type    */
	C2rMmapInfo *mmap;              /* mmap info           */
	struct v4l2_exportbuffer *eb;   /* Used in DMABUF mode */
	C2rIndexQueue *idx_queue;
} C2rV4l2Buf;

typedef struct _C2rTask {
	GstTask    *task;
	GRecMutex   mutex;
} C2rTask;

typedef struct _C2rDevice {
	int    fd;
	gchar *name;
} C2rDevice;

typedef struct _C2rCamera {
	C2rDevice   dev;
	C2rV4l2Buf *buf;
} C2rCamera;

typedef struct _C2rEncoder {
	C2rDevice   dev;
	GstMemory  *spspps_buf;
	C2rV4l2Buf *src_buf;
	C2rV4l2Buf *dst_buf;
	gint        pre_encode_num;     /* b_pic_mode + 1 */
	gint        buffer_in_out_count;
	GMutex      count_lock;         /* lock buffer_in_out_count */
} C2rEncoder;

typedef struct _C2rAppsrcSize {
	int input_width;
	int input_height;
	int output_width;
	int output_height;
} C2rAppsrcSize;

typedef struct _C2rAppsrc {
	GstElement *appsrc;
	C2rAppsrcSize size;

	C2rCamera cam;
	C2rEncoder enc;

	C2rTask cam_task;
	C2rTask enc_output_task;
	C2rTask enc_capture_task;
} C2rAppsrc;

typedef struct _C2rPipeline {
	GstElement *pipeline;
	C2rAppsrc *c2r_appsrc;
} C2rPipeline;

typedef struct _C2rOptions {
	C2rAppsrcSize size;
	gchar *camera_name;
	gchar *encoder_name;
	gchar *ip_addr;
	int    port;
} C2rOptions;

static gboolean c2r_caught_intr = FALSE;

static int c2r_xioctl (int fd, int request, void *arg)
{
	for (;;) {
		int ret = ioctl (fd, request, arg);
		if (ret < 0) {
			if (errno == EINTR)
				continue;
			return -errno;
		}
		break;
	}

	return 0;
}

static void c2r_sigint_handler (int arg)
{
	(void)arg;
	c2r_caught_intr = TRUE;
}

static void c2r_print_error_message (GstMessage * msg)
{
	GError *err = NULL;
	gchar *name, *debug = NULL;

	name = gst_object_get_path_string (msg->src);
	gst_message_parse_error (msg, &err, &debug);

	g_printerr ("ERROR: from element %s: %s\n", name, err->message);
	if (debug != NULL)
		g_printerr ("Additional debug info:\n%s\n", debug);

	g_error_free (err);
	g_free (debug);
	g_free (name);
}

static void c2r_task_setup (C2rTask *c2r_task,
		GstTaskFunction func, gpointer user_data)
{
	c2r_task->task = gst_task_new (func, user_data, NULL);
	g_rec_mutex_init (&c2r_task->mutex);
	gst_task_set_lock (c2r_task->task, &c2r_task->mutex);
}

static void c2r_index_queue_push (C2rIndexQueue *idx_queue, guint32 idx)
{
	guint32 *push_idx;

	push_idx = malloc (sizeof(guint32));
	*push_idx = idx;

	gst_atomic_queue_push (idx_queue->queue, (gpointer) push_idx);
}

/* return a usable index, G_MAXUINT32 on failure */
static guint32 c2r_index_queue_pop (C2rIndexQueue *idx_queue)
{
	guint32  ret_idx;
	guint32 *poped_idx;

	poped_idx = (guint32 *)gst_atomic_queue_pop (idx_queue->queue);

	if (poped_idx) {
		ret_idx = *poped_idx;
		free (poped_idx);
	} else {
		ret_idx = G_MAXUINT32;
	}

	return ret_idx;
}

static C2rIndexQueue *c2r_index_queue_new (guint buf_size)
{
	C2rIndexQueue *result;

	result = malloc (sizeof(C2rIndexQueue));
	if (!result) {
		return NULL;
	}

	result->queue = gst_atomic_queue_new ((guint)buf_size);
	if (!result->queue) {
		free (result);
		return NULL;
	}

	return result;
}

static void c2r_index_queue_free (C2rIndexQueue *idx_queue)
{
	gint queue_len;

	queue_len = gst_atomic_queue_length (idx_queue->queue);
	if (!queue_len) {
		int i;
		for (i = 0; i < queue_len; i++)
			c2r_index_queue_pop (idx_queue);
	}

	gst_atomic_queue_unref (idx_queue->queue);
}

static C2rV4l2Buf *c2r_v4l2_buf_new (int buf_size, enum v4l2_buf_type type)
{
	C2rV4l2Buf *result;

	result = malloc (sizeof(C2rV4l2Buf));
	if (!result) {
		return NULL;
	}

	result->mmap = malloc (sizeof(C2rMmapInfo) * buf_size);
	if (!result->mmap) {
		goto alloc_mmap_failed;
	}

	result->eb = malloc (sizeof(struct v4l2_exportbuffer) * buf_size);
	if (!result->eb) {
		goto alloc_eb_failed;
	}

	result->idx_queue = c2r_index_queue_new ((guint)buf_size);
	if (!result->idx_queue) {
		goto new_idx_queue_failed;
	}

	result->type = type;

	return result;

	/* ERROR */
new_idx_queue_failed:
	free (result->eb);

alloc_eb_failed:
	free (result->mmap);

alloc_mmap_failed:
	free (result);

	return NULL;
}

static void c2r_v4l2_buf_free (C2rV4l2Buf *buf)
{
	c2r_index_queue_free (buf->idx_queue);
	free (buf->mmap);
	free (buf->eb);
	free (buf);
}

static int c2r_device_set_format (C2rDevice dev, enum v4l2_buf_type type,
		int width, int height, unsigned int format, enum v4l2_field field)
{
	struct v4l2_format fmt;

	memset(&fmt, 0, sizeof(fmt));
	fmt.type                = type;
	fmt.fmt.pix.width       = width;
	fmt.fmt.pix.height      = height;
	fmt.fmt.pix.pixelformat = format;
	fmt.fmt.pix.field       = field;

	return c2r_xioctl (dev.fd, VIDIOC_S_FMT, &fmt);
}

static int c2r_device_set_control (C2rDevice dev, unsigned int id, int value)
{
	struct v4l2_control ctrl;
	ctrl.id = id;
	ctrl.value = value;
	return c2r_xioctl (dev.fd, VIDIOC_S_CTRL, &ctrl);
}

static int c2r_device_request_buffers (C2rDevice dev, enum v4l2_buf_type type,
		enum v4l2_memory memory, int count)
{
	struct v4l2_requestbuffers req;

	memset (&req, 0, sizeof(req));

	req.type   = type;
	req.memory = memory;
	req.count  = count;
	return c2r_xioctl (dev.fd, VIDIOC_REQBUFS, &req);
}

static int c2r_device_export_buffers (C2rDevice dev, C2rV4l2Buf *buf)
{
	int i;
	for (i = 0; i < C2R_ENC_OUTPUT_MMAP_COUNT; i++) {
		int ret;
		buf->eb[i].type  = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		buf->eb[i].index = i;
		buf->eb[i].plane = 0;
		buf->eb[i].flags = O_CLOEXEC;

		ret = ioctl(dev.fd, VIDIOC_EXPBUF, &buf->eb[i]);
		if (ret){
			g_print ("VIDIOC_EXPBUF error %d index= %d \n", ret, i);
			return -1;
		} else {
			g_print ("VIDIOC_EXPBUF success. index= %d returned fd=%d \n",
					i, buf->eb[i].fd);
		}
	}

	return 0;
}

static int c2r_device_query_buffers (C2rDevice dev, C2rV4l2Buf *c2r_buf,
		enum v4l2_buf_type type, enum v4l2_memory memory, int count)
{
	int i;

	for (i = 0; i < count; ++i) {
		struct v4l2_buffer tmp_buf;

		memset(&tmp_buf, 0, sizeof(tmp_buf));

		tmp_buf.type   = type;
		tmp_buf.memory = memory;
		tmp_buf.index  = i;

		if (c2r_xioctl (dev.fd, VIDIOC_QUERYBUF, &tmp_buf))
			return -1;

		c2r_index_queue_push (c2r_buf->idx_queue, (guint32) tmp_buf.index);

		if (memory != V4L2_MEMORY_DMABUF) {
			c2r_buf->mmap[i].addr =
				mmap(NULL, tmp_buf.length, PROT_READ | PROT_WRITE ,
						MAP_SHARED, dev.fd, tmp_buf.m.offset);
			c2r_buf->mmap[i].size = tmp_buf.length;

			if (MAP_FAILED == c2r_buf->mmap[i].addr)
				return -1;
		}
	}

	return 0;
}

static int c2r_device_enqueue_buffers (C2rDevice dev, C2rV4l2Buf *c2r_buf,
		enum v4l2_buf_type type, enum v4l2_memory memory, int count,
		struct v4l2_exportbuffer *export_buf)
{
	int i;
	struct v4l2_buffer buf;

	for (i = 0; i < count; i++) {
		memset(&buf, 0, sizeof(buf));
		buf.type   = type;
		buf.memory = memory;
		buf.index  = c2r_index_queue_pop (c2r_buf->idx_queue);
		if (export_buf) buf.m.fd = export_buf[i].fd;
		if (c2r_xioctl (dev.fd, VIDIOC_QBUF, &buf))
			return -1;
	}

	return 0;
}

static int c2r_device_open (C2rDevice *dev)
{
	struct stat st;

	if (-1 == stat(dev->name, &st)) {
		fprintf(stderr, "Cannot identify '%s': %d, %s\n",
				dev->name, errno, strerror(errno));
		return -1;
	}

	if (!S_ISCHR(st.st_mode)) {
		fprintf(stderr, "%s is no device\n", dev->name);
		return -1;
	}

	dev->fd = open(dev->name, O_RDWR, 0);

	if (-1 == dev->fd) {
		fprintf(stderr, "Cannot open '%s': %d, %s\n",
				dev->name, errno, strerror(errno));
		return -1;
	}

	return 0;
}

static void c2r_device_close (C2rDevice *dev)
{
	close(dev->fd);
}

static int c2r_camera_setup (C2rCamera *cam, C2rAppsrcSize size,
		C2rV4l2Buf *src_buf)
{
	int ret;

	ret = c2r_device_set_format (
			cam->dev, V4L2_BUF_TYPE_VIDEO_CAPTURE,
			size.input_width, size.input_height,
			V4L2_PIX_FMT_NV12, V4L2_FIELD_INTERLACED);
	if (ret) {
		perror("ioctl(VIDIOC_S_FMT)");
	}

	ret = c2r_device_request_buffers (cam->dev, V4L2_BUF_TYPE_VIDEO_CAPTURE,
			V4L2_MEMORY_DMABUF, C2R_CAM_MMAP_COUNT);
	if (ret) {
		perror("ioctl(VIDIOC_REQBUFS)");
	}

	ret = c2r_device_query_buffers (cam->dev, cam->buf,
			V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_DMABUF,
			C2R_CAM_MMAP_COUNT);
	if (ret) {
		perror("ioctl(VIDIOC_QUERYBUF)");
	}

	ret = c2r_device_enqueue_buffers (cam->dev, cam->buf,
			V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_DMABUF,
			C2R_CAM_MMAP_COUNT, src_buf->eb);
	if (ret) {
		perror("Enqueue Buffers");
	}

	return ret;
}

static int c2r_camera_stream_on (C2rCamera *cam)
{
	int ret;
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = c2r_xioctl (cam->dev.fd, VIDIOC_STREAMON, &type);
	if (ret) {
		perror("ioctl(VIDIOC_STREAMON)");
		return -1;
	}
	return 0;
}

static int c2r_camera_stream_off (C2rCamera *cam)
{
	int ret;
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	ret = c2r_xioctl (cam->dev.fd, VIDIOC_STREAMOFF, &type);
	if (ret) {
		perror("ioctl(VIDIOC_STREAMOFF)");
		return -1;
	}

	return 0;
}

static int c2r_encoder_setup (C2rEncoder *enc,
		C2rAppsrcSize size)
{
	int ret;

	enc->pre_encode_num = DEFAULT_B_PIC_MODE;
	enc->buffer_in_out_count = -(enc->pre_encode_num + 1);
	g_mutex_init (&enc->count_lock);

	/* s_fmt */
	ret = c2r_device_set_format (
			enc->dev, V4L2_BUF_TYPE_VIDEO_OUTPUT,
			size.input_width, size.input_height,
			V4L2_PIX_FMT_NV12, V4L2_FIELD_NONE);
	if (ret) return -1;
	ret = c2r_device_set_format (
			enc->dev, V4L2_BUF_TYPE_VIDEO_CAPTURE,
			size.output_width, size.output_height,
			V4L2_PIX_FMT_H264, V4L2_FIELD_NONE);
	if (ret) return -1;

	/* s_ctrl */
	/* bit_rate */
	ret = c2r_device_set_control (enc->dev,
			V4L2_CID_TARGET_BIT_RATE, DEFAULT_BITRATE);
	if (ret) return -1;
	/* max_frame_size */
	ret = c2r_device_set_control (enc->dev,
			V4L2_CID_MAX_FRAME_SIZE, DEFAULT_MAX_FRAME_SIZE);
	if (ret) return -1;
	/* rate_control_mode */
	ret = c2r_device_set_control (enc->dev,
			V4L2_CID_RATE_CONTROL_MODE, DEFAULT_RATE_CONTROL_MODE);
	if (ret) return -1;
	/* frame_rate_resolution */
	ret = c2r_device_set_control (enc->dev,
			V4L2_CID_FRAME_RATE_RESOLUTION, DEFAULT_FRAME_RATE_RESOLUTION);
	if (ret) return -1;
	/* frame_rate_tick */
	ret = c2r_device_set_control (enc->dev,
			V4L2_CID_FRAME_RATE_TICK, DEFAULT_FRAME_RATE_TICK);
	if (ret) return -1;
	/* max_GOP_length */
	ret = c2r_device_set_control (enc->dev,
			V4L2_CID_MAX_GOP_LENGTH, DEFAULT_MAX_GOP_LENGTH);
	if (ret) return -1;
	/* B_pic_modeの指定 */
	ret = c2r_device_set_control (enc->dev,
			V4L2_CID_B_PIC_MODE, DEFAULT_B_PIC_MODE);
	if (ret) return -1;

	/* reqbuf */
	ret = c2r_device_request_buffers (enc->dev, V4L2_BUF_TYPE_VIDEO_OUTPUT,
			V4L2_MEMORY_MMAP, C2R_ENC_OUTPUT_MMAP_COUNT);
	if (ret) return -1;
	ret = c2r_device_request_buffers (enc->dev, V4L2_BUF_TYPE_VIDEO_CAPTURE,
			V4L2_MEMORY_MMAP, C2R_ENC_CAPTURE_MMAP_COUNT);
	if (ret) return -1;

	/* exportbuf */
	ret = c2r_device_export_buffers (enc->dev, enc->src_buf);
	if (ret) return -1;

	/* querybuf */
	ret = c2r_device_query_buffers (enc->dev, enc->src_buf,
			V4L2_BUF_TYPE_VIDEO_OUTPUT,  V4L2_MEMORY_MMAP,
			C2R_ENC_OUTPUT_MMAP_COUNT);
	if (ret) return -1;
	ret = c2r_device_query_buffers (enc->dev, enc->dst_buf,
			V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP,
			C2R_ENC_CAPTURE_MMAP_COUNT);
	if (ret) return -1;

	/* qbuf - capture */
	ret = c2r_device_enqueue_buffers (enc->dev, enc->dst_buf,
			V4L2_BUF_TYPE_VIDEO_CAPTURE, V4L2_MEMORY_MMAP,
			C2R_ENC_CAPTURE_MMAP_COUNT, NULL);
	if (ret) return -1;

	return ret;
}

static int c2r_encoder_stream_on (C2rEncoder *enc)
{
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (c2r_xioctl (enc->dev.fd, VIDIOC_STREAMON, &type))
		return -1;

	type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	if (c2r_xioctl (enc->dev.fd, VIDIOC_STREAMON, &type))
		return -1;

	return 0;
}

static int c2r_encoder_stream_off (C2rEncoder *enc)
{
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (c2r_xioctl (enc->dev.fd, VIDIOC_STREAMOFF, &type))
		return -1;

	type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	if (c2r_xioctl (enc->dev.fd, VIDIOC_STREAMOFF, &type))
		return -1;

	return 0;
}

/* Camera -> Encoder:src */
static void c2r_appsrc_camera_loop (gpointer appsrc)
{
	int i;
	C2rAppsrc *c2r_appsrc = (C2rAppsrc *)appsrc;
	struct v4l2_buffer cam_buf;
	struct v4l2_buffer enc_buf;
	guint32 enc_index = G_MAXUINT32;

	memset(&cam_buf, 0, sizeof(cam_buf));
	memset(&enc_buf, 0, sizeof(enc_buf));

	cam_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	cam_buf.memory = V4L2_MEMORY_DMABUF;

	c2r_xioctl (c2r_appsrc->cam.dev.fd, VIDIOC_DQBUF, &cam_buf);

	/* look for the index based on fd */
	for (i = 0; i < C2R_ENC_OUTPUT_MMAP_COUNT; i++) {
		if (cam_buf.m.fd == c2r_appsrc->enc.src_buf->eb[i].fd) {
			enc_index = i;
			break;
		}
	}

	if (enc_index != G_MAXUINT32) {
		enc_buf.type      = V4L2_BUF_TYPE_VIDEO_OUTPUT;
		enc_buf.memory    = V4L2_MEMORY_MMAP;
		enc_buf.index     = enc_index;
		enc_buf.bytesused = cam_buf.bytesused;
		enc_buf.length    = cam_buf.bytesused;

		c2r_xioctl (c2r_appsrc->enc.dev.fd, VIDIOC_QBUF, &enc_buf);
		g_mutex_lock (&c2r_appsrc->enc.count_lock);
		c2r_appsrc->enc.buffer_in_out_count++;
		g_mutex_unlock (&c2r_appsrc->enc.count_lock);

		c2r_index_queue_push (
				c2r_appsrc->cam.buf->idx_queue, cam_buf.index);
	} else {
		g_print ("cam pop fail\n");
		gst_task_stop (c2r_appsrc->cam_task.task);
	}

	if (c2r_caught_intr)
		gst_task_stop (c2r_appsrc->cam_task.task);
}

static void c2r_appsrc_encoder_capture_loop (gpointer appsrc)
{
	C2rAppsrc *c2r_appsrc = (C2rAppsrc *)appsrc;
	struct v4l2_buffer buf;
	static gint pre_encode_count = -1;
	static guint64 frame_count = 0;
	GstMapInfo info;

	memset(&buf, 0, sizeof(buf));

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	c2r_xioctl (c2r_appsrc->enc.dev.fd, VIDIOC_DQBUF, &buf);

	if (pre_encode_count >= c2r_appsrc->enc.pre_encode_num) {
		GstBuffer *buffer;

		buffer = gst_buffer_new_allocate (NULL, buf.bytesused, NULL);
		gst_buffer_map (buffer, &info, GST_MAP_WRITE);

		info.size = buf.bytesused;
		memcpy (info.data, c2r_appsrc->enc.dst_buf->mmap[buf.index].addr,
				buf.bytesused);

		c2r_xioctl (c2r_appsrc->enc.dev.fd, VIDIOC_QBUF, &buf);
		g_mutex_lock (&c2r_appsrc->enc.count_lock);
		c2r_appsrc->enc.buffer_in_out_count--;
		g_mutex_unlock (&c2r_appsrc->enc.count_lock);

		gst_buffer_unmap(buffer, &info);

		/* set duration, frame_count */
		buffer->duration   = gst_util_uint64_scale_int (GST_SECOND,
				DEFAULT_FPS_D, DEFAULT_FPS_N);
		buffer->offset     = frame_count++;
		buffer->offset_end = frame_count;

		if (pre_encode_count < c2r_appsrc->enc.pre_encode_num + 1) {
			gst_buffer_prepend_memory (buffer, c2r_appsrc->enc.spspps_buf);
			++pre_encode_count;
		}

		/* gst_app_src_push_buffer()
		 *   Adds a buffer to the queue of buffers that the appsrc element will
		 *   push to its source pad.
		 *   When the block property is TRUE, this function can block until free
		 *   space becomes available in the queue.
		 */
		gst_app_src_push_buffer (GST_APP_SRC(c2r_appsrc->appsrc), buffer);
	} else {
		if (pre_encode_count < 0) {
			c2r_appsrc->enc.spspps_buf =
				gst_allocator_alloc (NULL, buf.bytesused, NULL);
			gst_memory_map (c2r_appsrc->enc.spspps_buf, &info, GST_MAP_WRITE);
			memcpy (info.data, c2r_appsrc->enc.dst_buf->mmap[buf.index].addr,
					buf.bytesused);
			gst_memory_unmap(c2r_appsrc->enc.spspps_buf, &info);
		}

		c2r_xioctl (c2r_appsrc->enc.dev.fd, VIDIOC_QBUF, &buf);
		++pre_encode_count;
	}

	g_mutex_lock (&c2r_appsrc->enc.count_lock);
	if (c2r_caught_intr && !c2r_appsrc->enc.buffer_in_out_count) {
		gst_task_stop (c2r_appsrc->enc_capture_task.task);
	}
	g_mutex_unlock (&c2r_appsrc->enc.count_lock);
}

/* Encoder:src -> Camera */
static void c2r_appsrc_encoder_output_loop (gpointer appsrc)
{
	C2rAppsrc *c2r_appsrc = (C2rAppsrc *)appsrc;
	struct v4l2_buffer cam_buf;
	struct v4l2_buffer enc_buf;
	guint32 cam_index;
	guint32 enc_index;

	memset (&cam_buf, 0, sizeof(cam_buf));
	memset (&enc_buf, 0, sizeof(enc_buf));

	enc_buf.type = V4L2_BUF_TYPE_VIDEO_OUTPUT;
	enc_buf.memory = V4L2_MEMORY_MMAP;

	enc_index = c2r_index_queue_pop (
			c2r_appsrc->enc.src_buf->idx_queue);

	c2r_xioctl (c2r_appsrc->enc.dev.fd, VIDIOC_DQBUF, &enc_buf);

	cam_index = c2r_index_queue_pop (
			c2r_appsrc->cam.buf->idx_queue);

	if (cam_index != G_MAXUINT32) {
		cam_buf.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cam_buf.memory = V4L2_MEMORY_DMABUF;
		cam_buf.index  = cam_index;
		cam_buf.m.fd   = c2r_appsrc->enc.src_buf->eb[enc_index].fd;

		c2r_xioctl (c2r_appsrc->cam.dev.fd, VIDIOC_QBUF, &cam_buf);

		c2r_index_queue_push (
				c2r_appsrc->enc.src_buf->idx_queue, enc_buf.index);
	} else {
		g_print ("enc pop fail\n");
		gst_task_stop (c2r_appsrc->enc_output_task.task);
	}

	if (c2r_caught_intr)
		gst_task_stop (c2r_appsrc->enc_output_task.task);
}

static gboolean c2r_appsrc_check_intr (C2rAppsrc * c2r_appsrc)
{
	if (!c2r_caught_intr) {
		return TRUE;
	} else {
		g_print ("Stoping tasks ...\n");

		gst_task_join (c2r_appsrc->cam_task.task);
		gst_task_join (c2r_appsrc->enc_output_task.task);
		gst_task_join (c2r_appsrc->enc_capture_task.task);

		g_print ("Pushing EOS ...\n");
		gst_app_src_end_of_stream (GST_APP_SRC(c2r_appsrc->appsrc));

		c2r_caught_intr = FALSE;

		/* remove timeout handler */
		return FALSE;
	}
}

/* return a new C2rAppsrc, NULL on failure.  */
static C2rAppsrc *c2r_appsrc_new (GstElement *pipeline, C2rOptions *opt)
{
	C2rAppsrc *result;

	result = malloc (sizeof(C2rAppsrc));
	if (!result) {
		return NULL;
	}

	result->appsrc = gst_bin_get_by_name (GST_BIN (pipeline), "appsrc0");
	if (!result->appsrc) {
		goto alloc_failed;
	}

	result->cam.buf = c2r_v4l2_buf_new (
			C2R_CAM_MMAP_COUNT, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (!result->cam.buf) {
		goto alloc_failed;
	}
	result->enc.src_buf = c2r_v4l2_buf_new (
			C2R_ENC_OUTPUT_MMAP_COUNT, V4L2_BUF_TYPE_VIDEO_OUTPUT);
	if (!result->enc.src_buf) {
		c2r_v4l2_buf_free (result->cam.buf);
		goto alloc_failed;
	}
	result->enc.dst_buf = c2r_v4l2_buf_new (
			C2R_ENC_CAPTURE_MMAP_COUNT, V4L2_BUF_TYPE_VIDEO_CAPTURE);
	if (!result->enc.src_buf) {
		c2r_v4l2_buf_free (result->enc.src_buf);
		c2r_v4l2_buf_free (result->cam.buf);
		goto alloc_failed;
	}
	result->cam.dev.name       = opt->camera_name;
	result->enc.dev.name       = opt->encoder_name;
	result->size.input_width   = opt->size.input_width;
	result->size.input_height  = opt->size.input_height;
	result->size.output_width  = opt->size.output_width;
	result->size.output_height = opt->size.output_height;

	if (c2r_device_open(&result->enc.dev)) {
		g_print ("fail c2r_device_open(enc)\n");
		goto encoder_open_failed;
	}
	if (c2r_device_open (&result->cam.dev)) {
		g_print ("fail c2r_device_open(cam)\n");
		goto camera_open_failed;
	}

	if (c2r_encoder_setup (&result->enc, result->size)) {
		g_print ("fail c2r_encoder_setup()\n");
		goto setup_failed;
	}
	if (c2r_camera_setup (&result->cam,
				result->size, result->enc.src_buf)) {
		g_print ("fail c2r_camera_setup()\n");
		goto setup_failed;
	}

	if (c2r_encoder_stream_on (&result->enc)) {
		g_print ("c2r_encoder_stream_on() failed\n");
		goto setup_failed;
	}
	if (c2r_camera_stream_on (&result->cam)) {
		g_print ("c2r_camera_stream_on() failed\n");
		goto setup_failed;
	}

	c2r_task_setup (&result->enc_capture_task,
			c2r_appsrc_encoder_capture_loop, result);
	c2r_task_setup (&result->enc_output_task,
			c2r_appsrc_encoder_output_loop, result);
	c2r_task_setup (&result->cam_task, 
			c2r_appsrc_camera_loop, result);

	return result;

	/* ERROR */
setup_failed:
	c2r_device_close (&result->cam.dev);

camera_open_failed:
	c2r_device_close (&result->enc.dev);

encoder_open_failed:
	c2r_v4l2_buf_free (result->enc.dst_buf);
	c2r_v4l2_buf_free (result->enc.src_buf);
	c2r_v4l2_buf_free (result->cam.buf);

alloc_failed:
	free (result);
	return NULL;
}

static void c2r_appsrc_free (C2rAppsrc *c2r_appsrc)
{
	c2r_camera_stream_off (&c2r_appsrc->cam);
	c2r_encoder_stream_off (&c2r_appsrc->enc);

	c2r_device_close (&c2r_appsrc->cam.dev);
	c2r_device_close (&c2r_appsrc->enc.dev);

	c2r_v4l2_buf_free (c2r_appsrc->cam.buf);
	c2r_v4l2_buf_free (c2r_appsrc->enc.src_buf);
	c2r_v4l2_buf_free (c2r_appsrc->enc.dst_buf);

	free (c2r_appsrc);
}

/* return a new C2rPipeline, NULL on failure.  */
static C2rPipeline *c2r_pipeline_new (C2rOptions *opt)
{
	gchar launch_str[500];
	C2rPipeline *result;

	result = malloc (sizeof(C2rPipeline));
	if (!result) {
		return NULL;
	}

	sprintf (launch_str,
			"appsrc is-live=true do-timestamp=true format=3 "
			"max-bytes=%d block=true "
			"! video/x-h264,stream-format=byte-stream,"
			"width=%d,height=%d,framerate=30/1 "
			"! h264parse ! rtph264pay config-interval=3 "
			"! udpsink force-ipv4=true host=%s port=%d",
			DEFAULT_MAX_BYTES,
			opt->size.output_width, opt->size.output_height,
			opt->ip_addr, opt->port);

	g_print ("Parsing pipeline: %s\n", launch_str);

	result->pipeline = gst_parse_launch (launch_str, NULL);
	if (!result->pipeline) {
		g_print ("gst_parse_launch() failed\n");
		goto setup_failed;
	}

	result->c2r_appsrc = c2r_appsrc_new (result->pipeline, opt);
	if (!result->c2r_appsrc) {
		g_print ("c2r_appsrc_new() failed\n");
		goto new_appsrc_failed;
	}

	return result;

	/* ERROR */
new_appsrc_failed:
	gst_object_unref (result->pipeline);

setup_failed:
	free (result);
	return NULL;
}

static void c2r_pipeline_free (C2rPipeline *c2r_pipeline)
{
	c2r_appsrc_free (c2r_pipeline->c2r_appsrc);

	g_print ("Setting pipeline to NULL ...\n");

	gst_element_set_state (c2r_pipeline->pipeline, GST_STATE_NULL);
	g_object_unref (c2r_pipeline->pipeline);

	free (c2r_pipeline);
}

static int c2r_pipeline_play (C2rPipeline *c2r_pipeline)
{
	GstBus *bus;
	gulong timeout_id;
	GstMessage *msg = NULL;

	g_print ("Setting pipeline to PLAYING ...\n");
	gst_element_set_state (c2r_pipeline->pipeline, GST_STATE_PLAYING);

	g_print ("Starting tasks ...\n");
	gst_task_start (c2r_pipeline->c2r_appsrc->enc_capture_task.task);
	gst_task_start (c2r_pipeline->c2r_appsrc->enc_output_task.task);
	gst_task_start (c2r_pipeline->c2r_appsrc->cam_task.task);

	timeout_id = g_timeout_add (200,
			(GSourceFunc) c2r_appsrc_check_intr, c2r_pipeline->c2r_appsrc);

	bus = gst_element_get_bus (c2r_pipeline->pipeline);
	msg = gst_bus_poll (bus, GST_MESSAGE_EOS | GST_MESSAGE_ERROR, GST_CLOCK_TIME_NONE);

	if (!msg)
		goto exit;

	if (GST_MESSAGE_TYPE (msg) == GST_MESSAGE_ERROR)
		c2r_print_error_message (msg);

exit:
	if (msg)
		gst_message_unref (msg);

	gst_object_unref (bus);
	g_source_remove (timeout_id);

	return 0;
}

static int c2r_options_check (C2rOptions opt)
{
	if (opt.size.input_height < opt.size.output_height ||
			opt.size.input_width < opt.size.output_width) {
		g_print ("The output width and height are too large. \
				They must be equal or less than input width and height.");

		return -1;
	}

	return 0;
}

int main(int argc, char *argv[])
{
	C2rOptions options;
	C2rPipeline *c2r_pipeline;
	GError *error = NULL;
	GOptionContext *context;
	GOptionEntry cmd_option_entries[] =
	{
		{ "camera",        'c', 0, G_OPTION_ARG_FILENAME, &options.camera_name,        "camera device",  NULL },
		{ "encoder",       'e', 0, G_OPTION_ARG_FILENAME, &options.encoder_name,       "encoder device", NULL },
		{ "input-width",   0,   0, G_OPTION_ARG_INT,      &options.size.input_width,   "input width",    NULL },
		{ "input-height",  0,   0, G_OPTION_ARG_INT,      &options.size.input_height,  "input height",   NULL },
		{ "output-width",  0,   0, G_OPTION_ARG_INT,      &options.size.output_width,  "output width",   NULL },
		{ "output-height", 0,   0, G_OPTION_ARG_INT,      &options.size.output_height, "output height",  NULL },
		{ "ip",            'i', 0, G_OPTION_ARG_STRING,   &options.ip_addr,            "ip addr",        NULL },
		{ "port",          'p', 0, G_OPTION_ARG_INT,      &options.port,               "port",           NULL },
		{ NULL }
	};

	signal(SIGINT, c2r_sigint_handler);

	options.camera_name        = C2R_CAM_DEVICE;
	options.encoder_name       = C2R_ENC_DEVICE;
	options.size.input_width   = C2R_CAM_WIDTH;
	options.size.input_height  = C2R_CAM_HEIGHT;
	options.size.output_width  = C2R_ENC_WIDTH;
	options.size.output_height = C2R_ENC_HEIGHT;
	options.ip_addr            = DEFAULT_IP_ADDR;
	options.port               = DEFAULT_PORT;

	context = g_option_context_new ("- v4l2src-acmh264enc");
	g_option_context_add_main_entries (context, cmd_option_entries, NULL);
	if (!g_option_context_parse (context, &argc, &argv, &error)) {
		g_print ("option parsing failed: %s\n", error->message);
		goto prepare_failed;
	}

	if (c2r_options_check (options))
		goto prepare_failed;

	gst_init (&argc, &argv);

	c2r_pipeline = c2r_pipeline_new (&options);
	if (!c2r_pipeline) {
		g_print ("c2r_pipeline_new() failed\n");
		goto prepare_failed;
	}

	if (c2r_pipeline_play (c2r_pipeline)) {
		g_print ("c2r_pipeline_play() failed\n");
	}

	c2r_pipeline_free (c2r_pipeline);

	/* ERROR */
prepare_failed:
	g_option_context_free (context);

	return 0;
}
