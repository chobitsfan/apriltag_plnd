/*
*  V4L2 video capture example
*
*  This program can be used and distributed without restrictions.
*
*      This program is provided with the V4L2 API
* see https://linuxtv.org/docs.php for more information
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <linux/videodev2.h>

#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>
#include <apriltag/tagStandard41h12.h>
#include <time.h>
#include <signal.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>

#define CAM_RES_W 1280
#define CAM_RES_H 720

#define CLEAR(x) memset(&(x), 0, sizeof(x))

#define SPEED_TEST 0

#define AMR_2
#ifdef MY_TEST_BOARD
#define MARKERS_COUNT 6
#endif
#ifdef AMR_1
#define MARKERS_COUNT 9
#endif
#ifdef AMR_2
#define MARKERS_COUNT 9
#endif

enum io_method {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

struct buffer {
    void   *start;
    size_t  length;
};

static char            *dev_name;
static enum io_method   io = IO_METHOD_MMAP;
static int              fd = -1;
struct buffer          *buffers;
static unsigned int     n_buffers;
static int              out_buf;
static int              force_format = 1;
static int              frame_count = 70;

apriltag_detector_t *td;
apriltag_family_t *tf;
apriltag_detection_info_t det_info = {.tagsize = 0.113, .fx = 978.0558315419056, .fy = 980.40099676993566, .cx = 644.32270873931213, .cy = 377.51661754419627};
matd_t* tgt_offset;

int ipc_fd;
struct sockaddr_in server;

bool gogogo = true;

void sig_handler(int signum) {
    gogogo = false;
}

static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
    int r;

    do {
	    r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

#ifdef MY_TEST_BOARD
static double tag_sz[MARKERS_COUNT] = {0.161, 0.113, 0.05, 0.05, 0.05, 0.113};
static double tgt_offset_x[MARKERS_COUNT] = {0, 0.2, 0, -0.2, 0.2, -0.2};
static double tgt_offset_y[MARKERS_COUNT] = {0.2, -0.25, -0.1, 0, 0, -0.25};
#endif
#ifdef AMR_1
static double tag_sz[MARKERS_COUNT] = {0.161, 0.161, 0.161, 0.161, 0.161, 0.113, 0.113, 0.113, 0.076};
static double tgt_offset_x[MARKERS_COUNT] = {0.41, 0, 0, 0, 0, 0, -0.48, 0, 0};
static double tgt_offset_y[MARKERS_COUNT] = {0, 0, 0, 0, 0, -0.37, 0, 0.37, 0};
#endif
#ifdef AMR_2
static double tag_sz[MARKERS_COUNT] = {0.161, 0.113, 0.113, 0.054, 0.076, 0.038, 0.054, 0.038, 0.038};
static double tgt_offset_x[MARKERS_COUNT] = {0.23, -0.17, 0.25, 0.23, -0.2, 0, -0.23, 0, 0};
static double tgt_offset_y[MARKERS_COUNT] = {0.27, 0.27, -0.24, 0, -0.24, 0, 0, 0.3, -0.25};
#endif

static void process_image(void *p, int size)
{
    //if (out_buf) fwrite(p, size, 1, stdout);

    //fflush(stderr);
    //fprintf(stderr, ".");
    //fflush(stderr);
    //printf("%d\n", size);

    double ipc_data[6];
    image_u8_t img_header = { .width=CAM_RES_W, .height=CAM_RES_H, .stride=CAM_RES_W, .buf=p };
    apriltag_pose_t pose;
#if SPEED_TEST
    struct timespec start, stop;
    clock_gettime(CLOCK_MONOTONIC, &start);
#endif
    zarray_t *detections = apriltag_detector_detect(td, &img_header);
    //clock_t end = clock();
    //printf("%f\n", (float)(end - begin) / CLOCKS_PER_SEC);
    if (zarray_size(detections) > 0) {
        apriltag_detection_t *det;
        zarray_get(detections, 0, &det);

        // Do stuff with detections here.
        if (det->id < MARKERS_COUNT) {
			det_info.det = det;
			det_info.tagsize = tag_sz[det->id];
			estimate_tag_pose(&det_info, &pose);
			tgt_offset->data[0]=tgt_offset_x[det->id];
			tgt_offset->data[1]=tgt_offset_y[det->id];
			matd_t* m1 = matd_multiply(pose.R, tgt_offset);
			matd_t* m2 = matd_add(m1, pose.t);
			ipc_data[0] = m2->data[0];
			ipc_data[1] = m2->data[1];
			ipc_data[2] = m2->data[2];
			if (det->id == 0) {
				ipc_data[3] = pose.R->data[3]; // I only need these 2 elements to compute yaw
				ipc_data[4] = pose.R->data[0];
			} else {
				ipc_data[3] = ipc_data[4] = 0;
			}
			ipc_data[5] = det->id;
			sendto(ipc_fd, ipc_data, sizeof(ipc_data), 0, (const struct sockaddr *)&server, sizeof(server));
			matd_destroy(m1);
			matd_destroy(m2);
			matd_destroy(pose.t);
			matd_destroy(pose.R);
        } else {
            printf("tag id %d found ?!\n", det->id);
        }
    }
    #if SPEED_TEST
    if (zarray_size(detections) == 0) {
        clock_gettime(CLOCK_MONOTONIC, &stop);
	    printf("%d\n", (int)((stop.tv_sec-start.tv_sec)*1000+(stop.tv_nsec-start.tv_nsec)*1e-6));
    }
    #endif
    apriltag_detections_destroy(detections);
}

static int read_frame(void)
{
    struct v4l2_buffer buf, buf2;
    unsigned int i;

    switch (io) {
    case IO_METHOD_READ:
	    if (-1 == read(fd, buffers[0].start, buffers[0].length)) {
		    switch (errno) {
		    case EAGAIN:
			    return 0;

		    case EIO:
			    /* Could ignore EIO, see spec. */

			    /* fall through */

		    default:
			    errno_exit("read");
		    }
	    }

	    process_image(buffers[0].start, buffers[0].length);
	    break;

    case IO_METHOD_MMAP:
	    CLEAR(buf);

	    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    buf.memory = V4L2_MEMORY_MMAP;

	    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
		    switch (errno) {
		    case EAGAIN:
			    return 0;

		    case EIO:
			    /* Could ignore EIO, see spec. */

			    /* fall through */

		    default:
			    errno_exit("VIDIOC_DQBUF");
		    }
	    }

            //https://lightbits.github.io/v4l2_real_time/
            //Dequeue all the buffers before we start processing, pick the latest one, and place everyone else back on the queue
	    CLEAR(buf2);
	    buf2.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    buf2.memory = V4L2_MEMORY_MMAP;
	    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf2)) {
            	if (errno == EAGAIN) {
	            assert(buf.index < n_buffers);
	    	    process_image(buffers[buf.index].start, buf.bytesused);
	            if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) errno_exit("VIDIOC_QBUF");
                    return 0;
                } else {
		    errno_exit("VIDIOC_DQBUF");
                }
            }

	    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf)) errno_exit("VIDIOC_QBUF");
	    assert(buf2.index < n_buffers);
	    process_image(buffers[buf2.index].start, buf2.bytesused);

	    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf2)) errno_exit("VIDIOC_QBUF");
	    break;

    case IO_METHOD_USERPTR:
	    CLEAR(buf);

	    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    buf.memory = V4L2_MEMORY_USERPTR;

	    if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
		    switch (errno) {
		    case EAGAIN:
			    return 0;

		    case EIO:
			    /* Could ignore EIO, see spec. */

			    /* fall through */

		    default:
			    errno_exit("VIDIOC_DQBUF");
		    }
	    }

	    for (i = 0; i < n_buffers; ++i)
		    if (buf.m.userptr == (unsigned long)buffers[i].start
			&& buf.length == buffers[i].length)
			    break;

	    assert(i < n_buffers);

	    process_image((void *)buf.m.userptr, buf.bytesused);

	    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		    errno_exit("VIDIOC_QBUF");
	    break;
    }

    return 1;
}

static void mainloop(void)
{
    while (gogogo) {
	    fd_set fds;
	    struct timeval tv;
	    int r;

	    FD_ZERO(&fds);
	    FD_SET(fd, &fds);

	    /* Timeout. */
	    tv.tv_sec = 2;
	    tv.tv_usec = 0;

	    r = select(fd + 1, &fds, NULL, NULL, &tv);

	    if (-1 == r) {
		    if (EINTR == errno)
			    continue;
		    errno_exit("select");
	    }

	    if (0 == r) {
		    fprintf(stderr, "select timeout\\n");
		    exit(EXIT_FAILURE);
	    }

	    read_frame();
	    /* EAGAIN - continue select loop. */
    }
}

static void stop_capturing(void)
{
    enum v4l2_buf_type type;

    switch (io) {
    case IO_METHOD_READ:
	    /* Nothing to do. */
	    break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
	    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
		    errno_exit("VIDIOC_STREAMOFF");
	    break;
    }
}

static void start_capturing(void)
{
    unsigned int i;
    enum v4l2_buf_type type;

    switch (io) {
    case IO_METHOD_READ:
	    /* Nothing to do. */
	    break;

    case IO_METHOD_MMAP:
	    for (i = 0; i < n_buffers; ++i) {
		    struct v4l2_buffer buf;

		    CLEAR(buf);
		    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		    buf.memory = V4L2_MEMORY_MMAP;
		    buf.index = i;

		    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
			    errno_exit("VIDIOC_QBUF");
	    }
	    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
		    errno_exit("VIDIOC_STREAMON");
	    break;

    case IO_METHOD_USERPTR:
	    for (i = 0; i < n_buffers; ++i) {
		    struct v4l2_buffer buf;

		    CLEAR(buf);
		    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		    buf.memory = V4L2_MEMORY_USERPTR;
		    buf.index = i;
		    buf.m.userptr = (unsigned long)buffers[i].start;
		    buf.length = buffers[i].length;

		    if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
			    errno_exit("VIDIOC_QBUF");
	    }
	    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
		    errno_exit("VIDIOC_STREAMON");
	    break;
    }
}

static void uninit_device(void)
{
    unsigned int i;

    switch (io) {
    case IO_METHOD_READ:
	    free(buffers[0].start);
	    break;

    case IO_METHOD_MMAP:
	    for (i = 0; i < n_buffers; ++i)
		    if (-1 == munmap(buffers[i].start, buffers[i].length))
			    errno_exit("munmap");
	    break;

    case IO_METHOD_USERPTR:
	    for (i = 0; i < n_buffers; ++i)
		    free(buffers[i].start);
	    break;
    }

    free(buffers);
}

static void init_read(unsigned int buffer_size)
{
    buffers = calloc(1, sizeof(*buffers));

    if (!buffers) {
	    fprintf(stderr, "Out of memory\\n");
	    exit(EXIT_FAILURE);
    }

    buffers[0].length = buffer_size;
    buffers[0].start = malloc(buffer_size);

    if (!buffers[0].start) {
	    fprintf(stderr, "Out of memory\\n");
	    exit(EXIT_FAILURE);
    }
}

static void init_mmap(void)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count = 3;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
	    if (EINVAL == errno) {
		    fprintf(stderr, "%s does not support "
			     "memory mappingn", dev_name);
		    exit(EXIT_FAILURE);
	    } else {
		    errno_exit("VIDIOC_REQBUFS");
	    }
    }

    if (req.count < 2) {
	    fprintf(stderr, "Insufficient buffer memory on %s\\n",
		     dev_name);
	    exit(EXIT_FAILURE);
    }

    buffers = calloc(req.count, sizeof(*buffers));

    if (!buffers) {
	    fprintf(stderr, "Out of memory\\n");
	    exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < req.count; ++n_buffers) {
	    struct v4l2_buffer buf;

	    CLEAR(buf);

	    buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    buf.memory      = V4L2_MEMORY_MMAP;
	    buf.index       = n_buffers;

	    if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
		    errno_exit("VIDIOC_QUERYBUF");

	    buffers[n_buffers].length = buf.length;
	    buffers[n_buffers].start =
		    mmap(NULL /* start anywhere */,
			  buf.length,
			  PROT_READ | PROT_WRITE /* required */,
			  MAP_SHARED /* recommended */,
			  fd, buf.m.offset);

	    if (MAP_FAILED == buffers[n_buffers].start)
		    errno_exit("mmap");
    }
}

static void init_userp(unsigned int buffer_size)
{
    struct v4l2_requestbuffers req;

    CLEAR(req);

    req.count  = 4;
    req.type   = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_USERPTR;

    if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
	    if (EINVAL == errno) {
		    fprintf(stderr, "%s does not support "
			     "user pointer i/on", dev_name);
		    exit(EXIT_FAILURE);
	    } else {
		    errno_exit("VIDIOC_REQBUFS");
	    }
    }

    buffers = calloc(4, sizeof(*buffers));

    if (!buffers) {
	    fprintf(stderr, "Out of memory\\n");
	    exit(EXIT_FAILURE);
    }

    for (n_buffers = 0; n_buffers < 4; ++n_buffers) {
	    buffers[n_buffers].length = buffer_size;
	    buffers[n_buffers].start = malloc(buffer_size);

	    if (!buffers[n_buffers].start) {
		    fprintf(stderr, "Out of memory\\n");
		    exit(EXIT_FAILURE);
	    }
    }
}

void print_v4l2_fourcc(unsigned int fourcc)
{
	char buf[8];
	buf[0] = fourcc & 0x7f;
	buf[1] = (fourcc >> 8) & 0x7f;
	buf[2] = (fourcc >> 16) & 0x7f;
	buf[3] = (fourcc >> 24) & 0x7f;
	if (fourcc & (1 << 31)) {
		buf[4] = '-';
		buf[5] = 'B';
		buf[6] = 'E';
		buf[7] = '\0';
	} else {
		buf[4] = '\0';
	}
	printf("%s\n", buf);
}

static void init_device(void)
{
    struct v4l2_capability cap;
    struct v4l2_cropcap cropcap;
    struct v4l2_crop crop;
    struct v4l2_format fmt;
    unsigned int min;

    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
	    if (EINVAL == errno) {
		    fprintf(stderr, "%s is no V4L2 device\\n",
			     dev_name);
		    exit(EXIT_FAILURE);
	    } else {
		    errno_exit("VIDIOC_QUERYCAP");
	    }
    }

    if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
	    fprintf(stderr, "%s is no video capture device\\n",
		     dev_name);
	    exit(EXIT_FAILURE);
    }

    switch (io) {
    case IO_METHOD_READ:
	    if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
		    fprintf(stderr, "%s does not support read i/o\\n",
			     dev_name);
		    exit(EXIT_FAILURE);
	    }
	    break;

    case IO_METHOD_MMAP:
    case IO_METHOD_USERPTR:
	    if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
		    fprintf(stderr, "%s does not support streaming i/o\\n",
			     dev_name);
		    exit(EXIT_FAILURE);
	    }
	    break;
    }


    /* Select video input, video standard and tune here. */


    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
	    crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	    crop.c = cropcap.defrect; /* reset to default */

	    if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
		    switch (errno) {
		    case EINVAL:
			    /* Cropping not supported. */
			    break;
		    default:
			    /* Errors ignored. */
			    break;
		    }
	    }
    } else {
	    /* Errors ignored. */
    }


    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (force_format) {
	    fmt.fmt.pix.width       = CAM_RES_W;
	    fmt.fmt.pix.height      = CAM_RES_H;
	    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUV420;
	    fmt.fmt.pix.field       = V4L2_FIELD_NONE;

	    if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
		    errno_exit("VIDIOC_S_FMT");

	    /* Note VIDIOC_S_FMT may change width and height. */
    } else {
	    /* Preserve original settings as set by v4l2-ctl for example */
	    if (-1 == xioctl(fd, VIDIOC_G_FMT, &fmt))
		    errno_exit("VIDIOC_G_FMT");
    }

    CLEAR(fmt);
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (xioctl(fd, VIDIOC_G_FMT, &fmt) == 0) {
        printf("fmt %d %d\n", fmt.fmt.pix.width, fmt.fmt.pix.height);
	print_v4l2_fourcc(fmt.fmt.pix.pixelformat);
    }

    /* Buggy driver paranoia. */
    min = fmt.fmt.pix.width * 2;
    if (fmt.fmt.pix.bytesperline < min)
	    fmt.fmt.pix.bytesperline = min;
    min = fmt.fmt.pix.bytesperline * fmt.fmt.pix.height;
    if (fmt.fmt.pix.sizeimage < min)
	    fmt.fmt.pix.sizeimage = min;

    switch (io) {
    case IO_METHOD_READ:
	    init_read(fmt.fmt.pix.sizeimage);
	    break;

    case IO_METHOD_MMAP:
	    init_mmap();
	    break;

    case IO_METHOD_USERPTR:
	    init_userp(fmt.fmt.pix.sizeimage);
	    break;
    }
}

static void close_device(void)
{
    if (-1 == close(fd))
	    errno_exit("close");

    fd = -1;
}

static void open_device(void)
{
    struct stat st;

    if (-1 == stat(dev_name, &st)) {
	    fprintf(stderr, "Cannot identify '%s': %d, %s\\n",
		     dev_name, errno, strerror(errno));
	    exit(EXIT_FAILURE);
    }

    if (!S_ISCHR(st.st_mode)) {
	    fprintf(stderr, "%s is no devicen", dev_name);
	    exit(EXIT_FAILURE);
    }

    fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);

    if (-1 == fd) {
	    fprintf(stderr, "Cannot open '%s': %d, %s\\n",
		     dev_name, errno, strerror(errno));
	    exit(EXIT_FAILURE);
    }
}

static void usage(FILE *fp, int argc, char **argv)
{
    fprintf(fp,
	     "Usage: %s [options]\\n\\n"
	     "Version 1.3\\n"
	     "Options:\\n"
	     "-d | --device name   Video device name [%s]n"
	     "-h | --help          Print this messagen"
	     "-m | --mmap          Use memory mapped buffers [default]n"
	     "-r | --read          Use read() callsn"
	     "-u | --userp         Use application allocated buffersn"
	     "-o | --output        Outputs stream to stdoutn"
	     "-f | --format        Force format to 640x480 YUYVn"
	     "-c | --count         Number of frames to grab [%i]n"
	     "",
	     argv[0], dev_name, frame_count);
}

static const char short_options[] = "d:hmruofc:";

static const struct option
long_options[] = {
    { "device", required_argument, NULL, 'd' },
    { "help",   no_argument,       NULL, 'h' },
    { "mmap",   no_argument,       NULL, 'm' },
    { "read",   no_argument,       NULL, 'r' },
    { "userp",  no_argument,       NULL, 'u' },
    { "output", no_argument,       NULL, 'o' },
    { "format", no_argument,       NULL, 'f' },
    { "count",  required_argument, NULL, 'c' },
    { 0, 0, 0, 0 }
};

int main(int argc, char **argv)
{
    signal(SIGINT, sig_handler);
    signal(SIGTERM, sig_handler);

    //system("v4l2-ctl -p 20");

    td = apriltag_detector_create();
    tf = tagStandard41h12_create();
    apriltag_detector_add_family(td, tf);
    td->quad_decimate = 3;
    td->nthreads = 4;
    tgt_offset = matd_create(3, 1);
    tgt_offset->data[0]=0;
    tgt_offset->data[1]=0;
    tgt_offset->data[2]=0;

    if ((ipc_fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        return 1;
    }
    memset(&server, 0, sizeof(server));
    /* Set up the server name */
    server.sin_family      = AF_INET;            /* Internet Domain    */
    server.sin_port        = htons(17510);  //Server Port
    server.sin_addr.s_addr = inet_addr("127.0.0.1");

    dev_name = "/dev/video0";

    for (;;) {
	    int idx;
	    int c;

	    c = getopt_long(argc, argv,
			    short_options, long_options, &idx);

	    if (-1 == c)
		    break;

	    switch (c) {
	    case 0: /* getopt_long() flag */
		    break;

	    case 'd':
		    dev_name = optarg;
		    break;

	    case 'h':
		    usage(stdout, argc, argv);
		    exit(EXIT_SUCCESS);

	    case 'm':
		    io = IO_METHOD_MMAP;
		    break;

	    case 'r':
		    io = IO_METHOD_READ;
		    break;

	    case 'u':
		    io = IO_METHOD_USERPTR;
		    break;

	    case 'o':
		    out_buf++;
		    break;

	    case 'f':
		    force_format = 1;
		    break;

	    case 'c':
		    errno = 0;
		    frame_count = strtol(optarg, NULL, 0);
		    if (errno)
			    errno_exit(optarg);
		    break;

	    default:
		    usage(stderr, argc, argv);
		    exit(EXIT_FAILURE);
	    }
    }

    open_device();
    init_device();
    start_capturing();
    mainloop();
    stop_capturing();
    uninit_device();
    close_device();
    tagStandard41h12_destroy(tf);
    apriltag_detector_destroy(td);
    matd_destroy(tgt_offset);
    close(ipc_fd);
    fprintf(stderr, "\n");
    return 0;
}
