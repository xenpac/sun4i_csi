/*
 * drivers/media/video/sun4i_csi/csi0/sun4i_drv_csi.c
 *
 * (C) Copyright 2007-2012
 * Allwinner Technology Co., Ltd. <www.allwinnertech.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA

 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 Driver file for both csi0, csi1, see define CSIPORT0 below to switch driver
 
 all functions are declared static! as well as data

Picture Sizes:
Note: CSI Fifos capture one complete Line at a time! Then DMA to memory during horizontal blank.
 The max. Pixelclocks/Bytes per Line the CSI-Module can handle is 4096 Bytes (fifo-size).
 The max. Lines per Frame the CSI-Module can handle is also 4096  (12Bits)
 Depending on the Videoformat one Pixel has a specific Bitlength (depth) of information. common values are 8, 12 or 16 bits per pixel.
 So the max. Pixel per line is reduced to: 4096*8/Bitlength); for 8bit=4096; for 12bit=2730, for 16bit=2048 (pixel per line)
 So the max.Framesize can be max: 4096*4096.
 
 Planar formats use one fifo for each channel ( Y, U, V). CSI Module seperates the CAM data into  the 3 channels!
 So in this case (YUV422 planar) the max picture size can be indeed 4096'4096 !!
 
 

 */

/*
 * Sun4i Camera Interface  driver based on lemaker sunxi driver.
 * Author: Thomas Krueger
 */
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/mutex.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <linux/delay.h>

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>

#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/moduleparam.h>

#include <plat/sys_config.h>
#include <mach/clock.h>
#include <mach/irqs.h>
#include <linux/regulator/consumer.h>

#include "../include/sun4i_csi_core.h"
#include "../include/sun4i_dev_csi.h"
#include "sun4i_csi_reg.h"

#include <linux/timer.h>

// this driver works for which port ?? !
#define CSIPORT1

//for internel driver debug
#define DBG   		1
//debug level 0~3: 3=only errors,2=+ routine trace, 1=+ details, 0= + interupt messages
#define DBG_LEV 	0

//for internel driver debug
#if(DBG==1)
#define csi_debug(l,x,arg...) if(l >= DBG_LEV) printk(KERN_CRIT"[CSI_DRIVER]"x,##arg)
#else
#define csi_debug(l,x,arg...)
#endif



#define CSI_MAJOR_VERSION 1
#define CSI_MINOR_VERSION 0
#define CSI_RELEASE 0
#define CSI_VERSION \
	KERNEL_VERSION(CSI_MAJOR_VERSION, CSI_MINOR_VERSION, CSI_RELEASE)
#define CSI_MODULE_NAME "sun4i_csi"




#define REGS_pBASE					(0x01C00000)	 	      // register base addr
#define SDRAM_REGS_pBASE    (REGS_pBASE + 0x01000)    // SDRAM Controller


#define CSI_OUT_RATE      (24*1000*1000)
#define CSI_ISP_RATE			(80*1000*1000)
#define CSI_MAX_FRAME_MEM (32*1024*1024)
//#define TWI_NO		 (1)

#define MIN_WIDTH  (32)
#define MIN_HEIGHT (32)
#define MAX_WIDTH  (4096)
#define MAX_HEIGHT (4096)

static unsigned video_nr = 0;
static unsigned first_flag = 0;
static unsigned IsCapturing=0;

// the module parameter values
static char cam_name[I2C_NAME_SIZE];  // csi camera module (cam_name) , name
static uint i2c_addr;  //its i2c address
struct csi_camera *pcamera;
static struct timer_list my_timer;
static unsigned long Error1,Error2,Error3,Error4,Error5,Error6,Error7,Error8,Error9,Error10,Error11;

/* modprobe sun4i_csi0 cam=name i2c_addr=0xaa , so you can specify which cameramodule to be used!
 driver moduels parameters: (S_IRUGO|S_IWUSR are file permissions as defines)
 below two functions to copy MODUL PARAMETERS, so can specify in modprobe command optionally
 NOTE: specifying a cam module-parameter makes the kernel to load the named modul BEFORE this driver is installed,
 so it is available at entry. Thus the associated camera sub-device driver is available.
 */
module_param_string(cam, cam_name, sizeof(cam_name), S_IRUGO|S_IWUSR); // copy modul param "cam" to variable cam_name
module_param(i2c_addr,uint, S_IRUGO|S_IWUSR); //copy modul param i2c_addr to variable i2c_addr as uint

/** return all unprocessed buffers from dmaque to user application with given state.
*/
static void return_all_buffers(struct csi_camera *pcam,  enum vb2_buffer_state state)
{
	struct cam_buffer *buf, *node;
	unsigned long flags;

	spin_lock_irqsave(&pcam->slock, flags);
	list_for_each_entry_safe(buf, node, &pcam->dmaque, list) 
	{
		vb2_buffer_done(&buf->vb, state);
		list_del(&buf->list);
	}
	spin_unlock_irqrestore(&pcam->slock, flags);
}


void checklist(void)
{
	struct cam_buffer *buf;
    struct csi_camera *pcam = pcamera;
    struct list_head *dma_q = &pcam->dmaque;




	struct list_head *i;
	list_for_each(i, dma_q) 
	{
		buf = list_entry(i, struct cam_buffer, list);
		switch (buf->vb.state)
		{	
		case VIDEOBUF_DONE:
		case VIDEOBUF_QUEUED:
		break;
		default:
//		Error10++;
		break;
		}
		Error10++;
	}
}
//+++:+++++++++++++++++++++++++++++++++++++++++++  csi device subfunctions subroutines


/**@brief IRQ handler csi module
*/
void my_timer_callback( unsigned long data )
{
    struct cam_buffer *buf;
    struct csi_camera *pcam = pcamera;
    struct list_head *dma_q = &pcam->dmaque;


    csi_debug(0,"--csi_isr:\n");
	if (!IsCapturing) return; 

// a frame was captured...
	
    spin_lock(&pcam->slock); // dont interrupt me!

//checklist();
	
	// if dma-buffer list is empty, just skip this capture. The app did not queue a buffer yet.
    if (list_empty(dma_q))
    {
        csi_debug(0,"++ret:frame filled not on queue\n"); // app is too slow!
		Error1++; //the frame filled was not on the queue, so we cant get its pointer!
        goto unlock; //fill the last buffer again
    }

	// get address of dma-buffer-structure we just filled. (always the first list member!)	
    buf = list_first_entry(dma_q,struct cam_buffer, list);
    csi_debug(0,"++ret:buf ptr=%p\n",buf);

 
     list_del(&buf->list); // remove that buffer from dma que, it has been serviced! 
   

    // keep a sequential frame count
	buf->vb.v4l2_buf.sequence = pcam->sequence++;

	vb2_buffer_done(&buf->vb, VB2_BUF_STATE_DONE); // wake up waiting task from poll. she may fetch the buffer now
	
	
    // get next buffer from the dma-queue to use for the next capture:
    if (list_empty(dma_q)) // is dma-queue empty?
    {
		Error3++; //list empty, cannot que next
        csi_debug(0,"++next:buf list is empty\n");
        goto unlock;  // filling the last buffer again !!
    }



		Error4++; // que next buf
    buf = list_first_entry(dma_q,struct cam_buffer, list); // get next buffer in dma queue, the first one again

 //   csi_set_addr(pcam,buf); // tell csi fifos about its address
	// dont set VIDEOBUF_ACTIVE flag !!! somehow messes videobuf up.
unlock:
    spin_unlock(&pcam->slock);
  mod_timer(&my_timer, jiffies + msecs_to_jiffies(100)); // restart in msecs

		Error5++; //number of ints

}







//+++:++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  vb2 ops functions
//Videobuf operations. buffer management for V4L2 drivers

/**@brief register a videobuffer queue with the videobuf2 layer

called after VIDIOC_REQBUFS or VIDIOC_CREATE_BUFS  before memory allocation.

called from vb2_core_reqbufs:
Ask the driver how many buffers and planes per buffer it requires.
Driver also sets the size and allocator context for each plane.

Here the driver shall check, if the number of buffers is correct.
It shall also set the max expected buffersize.
setup the buffersize according to picture dimensions in the camera struct.
setup the buffercount to a value we think is right
set num_planes to 1, as we only use single buffer per dma.

@param vq(pointer) = struct vb2_queue is the handle of the buffer queue
@param fmt = pointer to target format (should be NULL)
@param count(pointer) = pointer to suggested number of buffers
@param num_planes = pointer to number of planes
@param size(pointer) = pointer to expected maximum size for each buffer
@param alloc_ctxs = pointer to a plane info struct. we not use it.
@return 0=success
*/
static int vb2_drv_bufq_setup(struct vb2_queue *vq, const struct v4l2_format *fmt,
		       unsigned int *nbuffers, unsigned int *nplanes,
		       unsigned int sizes[],  void *alloc_ctxs[])
{
    struct csi_camera *pcam = vb2_get_drv_priv(vq);

    csi_debug(2,"Video vb2_drv_bufq_setup: count=%d, size=%d\n", *nbuffers, sizes[0]);

	sizes[0] = pcam->frame_size;
	*nplanes = 1;

     if (*nbuffers == 0 || *nbuffers > 6) // check limits
        *nbuffers = 6;
		
    csi_debug(2,"%s, buffer count=%d, size=%d\n", __func__,*nbuffers, sizes[0]);

	return 0;
}



/**@brief for a given buffer, set the buffer's size,  properly in the plane
Prepare the buffer for queueing to the DMA engine: check and set the payload size.

The buf_prepare() callback is invoked when user space queues the buffer 
(i.e. in response to a VIDIOC_QBUF operation);

@param  vb(pointer) = struct vb2_buffer is the buffer to be handled
@return 0=succes; 
*/
static int vb2_drv_buf_prepare(struct vb2_buffer *vb)
{
    struct csi_camera *pcam = vb2_get_drv_priv(vb->vb2_queue);

    csi_debug(1,"vb2_drv_buf_prepare\n");

	if (vb2_plane_size(vb, 0) < pcam->frame_size) 
	{
		csi_debug(3,"buffer too small (%lu < %lu)\n", vb2_plane_size(vb, 0), pcam->frame_size);
		return -EINVAL;
	}

	vb2_set_plane_payload(vb, 0, pcam->frame_size);
	return 0;
}



/**@brief queue a buffer for IO
Queue this buffer to the DMA engine.

user called VIDIOC_QBUF. 
- vb2_drv_buf_prepare
- vb2_drv_buf_queue

When a buffer is queued for I/O, it is passed to buf_queue(), which should put it onto the driver's list of available buffers 
and set its state to VIDEOBUF_QUEUED. 
Note that this function is called with the queue spinlock held!

@param  vb(pointer) = struct vb2_buffer is the buffer to be handled
*/
static void vb2_drv_buf_queue(struct vb2_buffer *vb)
{
    struct csi_camera *pcam = vb2_get_drv_priv(vb->vb2_queue);
	struct cam_buffer *buf = container_of(vb, struct cam_buffer, vb);
	unsigned long flags;
	
    csi_debug(1,"buffer_queued ptr=%p\n",vb);

	spin_lock_irqsave(&pcam->slock, flags); // add to dma queue
    list_add_tail(&buf->list, &pcam->dmaque); // add to dma queue.
	spin_unlock_irqrestore(&pcam->slock, flags);
}



/**@brief  Start streaming I/O
*/
static int vb2_drv_streamon(struct vb2_queue *vq, unsigned int count)
{
    struct csi_camera *pcam = vb2_get_drv_priv(vq);
    struct list_head *dma_q = &pcam->dmaque;
 


    csi_debug(2,"ioctl - vidioc_streamon\n");
	
	pcam->sequence = 0;
 

    if (IsCapturing)
    {
        csi_debug(3,"***ERROR:error stream has been already on\n");
		//In case of an error, return all active buffers to the QUEUED state
		return_all_buffers(pcam, VB2_BUF_STATE_QUEUED);		
        return 0;
    }



	if (list_empty(dma_q)) csi_debug(3,"dma-queue empty streamon\n");
	csi_debug(3,"starting timer\n");
	// get current buffer from the waiting buffer list and set its address for first csi dma.
  /* setup your timer to call my_timer_callback */
  setup_timer(&my_timer, my_timer_callback, 0);
  /* setup timer interval to 200 msecs */
  mod_timer(&my_timer, jiffies + msecs_to_jiffies(100)); // initial
 csi_debug(3,"OK\n");
    IsCapturing = 1;
	
    return 0;
}



/**@brief  stop streaming I/O
*/
static int vb2_drv_streamoff(struct vb2_queue *vq)
{
    struct csi_camera *pcam = vb2_get_drv_priv(vq);

    csi_debug(2,"ioctl - vidioc_streamoff\n");
	
	return_all_buffers(pcam, VB2_BUF_STATE_ERROR); //Release all active buffers from dmaque

    if (!IsCapturing)
    {
        csi_debug(3,"***ERROR:error stream has been already off\n");
        return 0;
    }

    IsCapturing = 0;

	del_timer_sync(&my_timer);

	first_flag = 0; 
	
 csi_debug(3,"streamoff\n");
    return 0;
}







/** videobuf2 layer operations

Here are the functions that videobuf2 layer calls to the driver:

 The vb2 queue ops. Note that since q->lock is set we can use the standard
 vb2_ops_wait_prepare/finish helper functions. If q->lock would be NULL,
 then this driver would have to provide these ops. Sooo...all locking already done!
 */
 static struct vb2_ops csi_videoq_ops =
{
	.queue_setup		= vb2_drv_bufq_setup,   // set num_buf, size, planes
	.buf_prepare		= vb2_drv_buf_prepare, // check and set the payload size
	.buf_queue			= vb2_drv_buf_queue,  // queue to DMA
	.start_streaming	= vb2_drv_streamon, // called when streamon
	.stop_streaming		= vb2_drv_streamoff, // called when streamoff

	// the following functions (vb2_ops_...) are all in videobuf2-v4l2.c
	//.wait_prepare		= vb2_ops_wait_prepare, // in upper layer
	//.wait_finish		= vb2_ops_wait_finish, // in upper layer
};



//+++:++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ video ioctl functions

/**@brief query capabilities
The ioctl takes a pointer to a struct v4l2_capability which is filled by the driver. 
When the driver is not compatible with this specification the ioctl returns an EINVAL error code.
*/
static int vidioc_querycap(struct file *file, void  *priv,
                           struct v4l2_capability *cap)
{
    struct csi_camera *pcam = video_drvdata(file);
    csi_debug(2,"ioctl - vidioc_querycap=sun4i_csi\n");
    strcpy(cap->driver, "sun4i_csi");
    strcpy(cap->card, "sun4i_csi");
    strlcpy(cap->bus_info, pcam->v4l2_dev.name, sizeof(cap->bus_info));

    cap->version = CSI_VERSION;
    cap->capabilities = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
                        //V4L2_CAP_READWRITE;  //support read and/or write, here its just read! Tomk
    return 0;
}

/**@brief Enumerate image formats
To enumerate image formats applications initialize the type and index field of struct v4l2_fmtdesc 
and call the VIDIOC_ENUM_FMT ioctl with a pointer to this structure. 
Drivers fill the rest of the structure or return an EINVAL error code. 
All formats are enumerable by beginning at index zero and incrementing by one until EINVAL is returned.

struct v4l2_fmtdesc {
	__u32		    index;             // Format number  given in  request!
	enum v4l2_buf_type  type;           //buffer type = V4L2_BUF_TYPE_VIDEO_CAPTURE   ?    
	__u32               flags;			// none
	__u8		    description[32];   // format name in ascii, user display
	__u32		    pixelformat;       // Format fourcc  code    
	__u32		    reserved[4];
	We need to set description[32] and pixelformat=fourcc.
};

Caller tells us the wanted index. He wants the fourcc-code and a description.
We call subdev and pass index and address of u32 code.
subdev returns the code. we translate the code into ascii description.
*/
static int vidioc_enum_fmt_vid_cap(struct file *file, void  *priv,  struct v4l2_fmtdesc *f)
{

	struct csi_camera *pcam = video_drvdata(file);

    csi_debug(2,"ioctl - vidioc_enum_fmt_vid_cap index:%u \n",f->index);
	

    if (f->index != 0)  return -EINVAL;

	f->description[0]=(char)(pcam->fourcc);
	f->description[1]=(char)(pcam->fourcc>>8);
	f->description[2]=(char)(pcam->fourcc>>16);
	f->description[3]=(char)(pcam->fourcc>>24);
	f->description[4]=0;
	
	csi_debug(2,"----------returned fourcc:%s\n",f->description);
    return 0;
}

/**@brief  get the video data format
To query the current parameters applications set the type field of a struct v4l2_format to the 
respective buffer (stream) type. 
For example video capture devices use V4L2_BUF_TYPE_VIDEO_CAPTURE. 
When the application calls the VIDIOC_G_FMT ioctl with a pointer to this structure the driver 
fills the respective member of the fmt union. 
In case of video capture devices that is the struct v4l2_pix_format pix member. 
When the requested buffer type is not supported drivers return an EINVAL error code.
struct v4l2_format {
	enum v4l2_buf_type type;  // V4L2_BUF_TYPE_VIDEO_CAPTURE for camera
	union {
		struct v4l2_pix_format		pix;     // V4L2_BUF_TYPE_VIDEO_CAPTURE, see below
		__u8	raw_data[200];    // not used
	} fmt;
};
struct v4l2_pix_format {
	__u32         		width; // pixel per line
	__u32			height;  // lines per frame
	__u32			pixelformat; // fourcc-code
	enum v4l2_field  	field; // use V4L2_FIELD_NONE for camera
	__u32            	bytesperline;	//calculated bytes per line = (pixel_per_line * Bits_per_pixel)/8
	__u32          		sizeimage; //height * width(bytesperline)
	enum v4l2_colorspace	colorspace;  // V4L2_COLORSPACE_SMPTE170M for camera
	__u32			priv;		// private data, depends on pixelformat 
};

Caller wants: widht, height, fourcc-code, bytesperline, sizeimage, colorspace.
We have this information available in the camera structure. so no need to call subdev.
*/
static int vidioc_g_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct csi_camera *pcam = video_drvdata(file);
	
	if (pcam != pcamera) csi_debug(2,"pcam mismatch\n");
	
	// copy current video format to structure-fields:
    f->fmt.pix.width        = pcam->width;
    f->fmt.pix.height       = pcam->height;
    f->fmt.pix.field        = V4L2_FIELD_NONE; // always V4L2_FIELD_NONE
    f->fmt.pix.pixelformat  = pcam->fourcc;
    f->fmt.pix.bytesperline = pcam->bytesperline; //(f->fmt.pix.width * bitsperpixel) >> 3; // depth = bitdepth = Bits per pixel
    f->fmt.pix.sizeimage    = pcam->frame_size; //f->fmt.pix.height * f->fmt.pix.bytesperline;

    csi_debug(2,"ioctl - vidioc_g_fmt_vid_cap returned fourcc:%c%c%c%c\n",(char)(pcam->fourcc),(char)(pcam->fourcc>>8),(char)(pcam->fourcc>>16),(char)(pcam->fourcc>>24) );

    return 0;
}



/**@brief try a data format. return best fit. just info!
The VIDIOC_TRY_FMT ioctl is equivalent to VIDIOC_S_FMT with one exception: 
it does not change driver state. 
It does NOT set the format in the camera!
It can also be called at any time, never returning EBUSY. 
This function is provided to negotiate parameters, to learn about hardware limitations, 
without disabling I/O or possibly time consuming hardware preparations. 
Although strongly recommended drivers are not required to implement this ioctl.

The Caller wants a best matching working format to his suggestion.
He gives us fourcc, width, height, bytesperline, sizeimage.
We return the best matching fourcc,width, height, bytesperline, sizeimage.
-
we call subdev and pass info in struct v4l2_mbus_framefmt. (it uses this)
.width = width
.height = hight
.code = fourcc
.field = bitsperpixel
On return, we calculate the missing bytesperline, sizeimage.
*/
static int vidioc_try_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct csi_camera *pcam = video_drvdata(file);
 

    csi_debug(2,"ioctl - vidioc_try_fmt_vid_cap - fourcc:%c%c%c%c\n", (char)(f->fmt.pix.pixelformat), (char)(f->fmt.pix.pixelformat>>8), (char)(f->fmt.pix.pixelformat>>16), (char)(f->fmt.pix.pixelformat>>24) );


    //copy the returned video format to structure to be retuned
    f->fmt.pix.width = pcam->width;
    f->fmt.pix.height = pcam->height;
	f->fmt.pix.pixelformat =  pcam->fourcc; //fourcc
	f->fmt.pix.bytesperline = pcam->bytesperline;  //tomk
	f->fmt.pix.sizeimage =pcam->frame_size;
    f->fmt.pix.field = V4L2_FIELD_NONE; 

    return 0;
}


/**@brief set data format. will set best fit and return its info!
To change the current format parameters applications initialize the type field and all fields 
of the respective fmt union member. 
For details see the documentation of the various devices types in Chapter 4. 
Good practice is to query the current parameters first, and to modify only those parameters 
not suitable for the application. 
When the application calls the VIDIOC_S_FMT ioctl with a pointer to a v4l2_format structure 
the driver checks and adjusts the parameters against hardware abilities. 
Drivers should not return an error code unless the input is ambiguous, 
this is a mechanism to fathom device capabilities and to approach parameters acceptable 
for both the application and driver. On success the driver may program the hardware, 
allocate resources and generally prepare for data exchange. 
Finally the VIDIOC_S_FMT ioctl returns the current format parameters as VIDIOC_G_FMT does. 
Very simple, inflexible devices may even ignore all input and always return the default parameters. 
However all V4L2 devices exchanging data with the application must implement the VIDIOC_G_FMT and 
VIDIOC_S_FMT ioctl. 
When the requested buffer type is not supported drivers return an EINVAL error code on a VIDIOC_S_FMT attempt. 
When I/O is already in progress or the resource is not available for other reasons drivers 
return the EBUSY error code.

Caller wants us to set this format: widht, height, fourcc-code, bytesperline, sizeimage, colorspace.
we call subdev and pass info in struct v4l2_mbus_framefmt. (it uses this, or trys best match as in try_format)
It returns the set parameters .
.width = width
.height = hight
.code = fourcc
.field = bitsperpixel

The subdevice stores this info in camera struct as working format.
We also check the number of planes to use and init csi according.
*/
static int vidioc_s_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct csi_camera *pcam = video_drvdata(file);
	

    csi_debug(2,"ioctl - vidioc_s_fmt_vid_cap - fourcc:%c%c%c%c\n", (char)(f->fmt.pix.pixelformat), (char)(f->fmt.pix.pixelformat>>8), (char)(f->fmt.pix.pixelformat>>16), (char)(f->fmt.pix.pixelformat>>24) );

	// first stop the capturing before we can set new format
    if (IsCapturing)
    {
        csi_debug(3,"%s device busy\n", __func__);
        return -EBUSY;
    }

 	
// update the information in the callers struct v4l2_format:
	f->fmt.pix.width = pcam->width;
	f->fmt.pix.height = pcam->height;
	f->fmt.pix.pixelformat = pcam->fourcc;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline = pcam->bytesperline;
	f->fmt.pix.sizeimage = pcam->frame_size;
	f->fmt.pix.colorspace = 0;
	// todo: setup mplane info

    return 0;
}


//+++:++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  video stream functions

/**@brief Initiate Memory Mapping or User Pointer I/O.

This ioctl is used to initiate memory mapped or user pointer I/O. 
Memory mapped buffers are located in device memory and must be allocated with this 
ioctl before they can be mapped into the application's address space. 
User buffers are allocated by applications themselves, and this ioctl is merely used to 
switch the driver into user pointer I/O mode.

To allocate device buffers applications initialize three fields of a v4l2_requestbuffers structure. 
They set the type field to the respective stream or buffer type, the count field to the desired number 
of buffers, and memory must be set to V4L2_MEMORY_MMAP. 
When the ioctl is called with a pointer to this structure the driver attempts to allocate 
the requested number of buffers and stores the actual number allocated in the count field. 
It can be smaller than the number requested, even zero, when the driver runs out of free memory. 
A larger number is possible when the driver requires more buffers to function correctly.[1] 
When memory mapping I/O is not supported the ioctl returns an EINVAL error code.
*/
static int vidioc_reqbufs(struct file *file, void *priv, struct v4l2_requestbuffers *p)
{
    struct csi_camera *pcam = video_drvdata(file);

    csi_debug(2,"ioctl - vidioc_reqbufs\n");

    return vb2_reqbufs(&pcam->vb2_vidq, p);
}


/**@brief Query the status of a buffer
This ioctl is part of the memory mapping I/O method. 
It can be used to query the status of a buffer at any time after buffers have been allocated 
with the vidioc_reqbufs ioctl.

Applications set the type field of a struct v4l2_buffer to the same buffer type as previously struct 
v4l2_format type and struct v4l2_requestbuffers type, and the index field. 
Valid index numbers range from zero to the number of buffers allocated with vidioc_reqbufs 
(struct v4l2_requestbuffers count) minus one. 
After calling vidioc_querybuf with a pointer to this structure drivers return an error code or 
fill the rest of the structure.

In the flags field the V4L2_BUF_FLAG_MAPPED, V4L2_BUF_FLAG_QUEUED and V4L2_BUF_FLAG_DONE flags 
will be valid. 
The memory field will be set to V4L2_MEMORY_MMAP, the m.offset contains the offset of 
the buffer from the start of the device memory, the length field its size. 
The driver may or may not set the remaining fields and flags, they are meaningless in this context.
*/
static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    struct csi_camera *pcam = video_drvdata(file);
    csi_debug(2,"ioctl - vidioc_querybuf\n");

    return vb2_querybuf(&pcam->vb2_vidq, p);
}


/**@brief enqueue a userbuffer with the driver
Applications call the VIDIOC_QBUF ioctl to enqueue an empty (capturing) buffer
in the driver's incoming queue. 
*/
static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{

    struct csi_camera *pcam = video_drvdata(file);
    csi_debug(1,"ioctl - vidioc_qbuf  ptr=%p\n",p);
		Error6++; //qbuf
 
	return vb2_qbuf(&pcam->vb2_vidq, p);
}


/**@brief  dequeue a filled (capturing)  userbuffer from the driver's outgoing queue
*/
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    struct csi_camera *pcam = video_drvdata(file);
    csi_debug(1,"ioctl - vidioc_dqbuf\n");
		Error7++; //dque

	return vb2_dqbuf(&pcam->vb2_vidq, p, file->f_flags & O_NONBLOCK);

}

/** Allocate buffers and any required auxiliary structs
wrapper from user space
*/
static int vidioc_create_bufs(struct file *file, void *priv, struct v4l2_create_buffers *create)
{
    struct csi_camera *pcam = video_drvdata(file);
    csi_debug(1,"ioctl - vidioc_create_bufs\n");

		return vb2_create_bufs(&pcam->vb2_vidq, create);
}

/**
wrapper
*/
static int vidioc_prepare_buf(struct file *file, void *priv, struct v4l2_buffer *b)
{
    struct csi_camera *pcam = video_drvdata(file);
	    csi_debug(1,"ioctl - vidioc_prepare_buf\n");

return vb2_prepare_buf(&pcam->vb2_vidq, b);
}

/**
wrapper

static int vidioc_expbuf(struct file *file, void *priv, struct v4l2_exportbuffer *p)
{
    struct csi_camera *pcam = video_drvdata(file);
	return vb2_expbuf(&pcam->vb2_vidq, p);
}
*/

static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
    struct csi_camera *pcam = video_drvdata(file);
	    csi_debug(1,"ioctl - vidioc_streamon\n");

	return vb2_streamon(&pcam->vb2_vidq, i);
}

static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
    struct csi_camera *pcam = video_drvdata(file);
	    csi_debug(1,"ioctl - vidioc_streamoff\n");

	return vb2_streamoff(&pcam->vb2_vidq, i);
}



/**@brief Enumerate video inputs
To query the attributes of a video input applications initialize the index field of struct v4l2_input 
and call the VIDIOC_ENUMINPUT ioctl with a pointer to this structure. Drivers fill the rest of 
the structure or return an EINVAL error code when the index is out of bounds. 
To enumerate all inputs applications shall begin at index zero, incrementing by one until 
the driver returns EINVAL.
*/
static int vidioc_enum_input(struct file *file, void *priv,
                             struct v4l2_input *inp)
{
    csi_debug(2,"ioctl - vidioc_enum_input\n");
    if (inp->index != 0)

        return -EINVAL;
 

    inp->type = V4L2_INPUT_TYPE_CAMERA;

    return 0;
}



/**@brief  Query the current video input. we only support one input
 It is good practice to select an input before querying or negotiating any other parameters.
*/
static int vidioc_g_input(struct file *file, void *priv, unsigned int *i)
{
    csi_debug(2,"ioctl - vidioc_get_input\n");
    *i = 0;
    return 0;
}


 

/**@brief select the current video input.we only support one input
*/
static int vidioc_s_input(struct file *file, void *priv, unsigned int i)
{
    csi_debug(2,"ioctl - vidioc_set_input\n");
    if (i != 0)
        return -EINVAL;
    return 0;
}



/**@brief Enumerate controls 
To query the attributes of a control applications set the id field of a struct v4l2_queryctrl and call 
the VIDIOC_QUERYCTRL ioctl with a pointer to this structure. 
The driver fills the rest of the structure or returns an EINVAL error code when the id is invalid.

It is possible to enumerate controls by calling VIDIOC_QUERYCTRL with successive id values starting 
from V4L2_CID_BASE up to and exclusive V4L2_CID_BASE_LASTP1. 
Drivers may return EINVAL if a control in this range is not supported. 
Further applications can enumerate private controls, which are not defined in this specification, 
by starting at V4L2_CID_PRIVATE_BASE and incrementing id until the driver returns EINVAL.

In both cases, when the driver sets the V4L2_CTRL_FLAG_DISABLED flag in the flags field this control 
is permanently disabled and should be ignored by the application.
*/
static int vidioc_queryctrl(struct file *file, void *priv,
                            struct v4l2_queryctrl *qc)
{
    int ret;
	ret = -EINVAL;
    csi_debug(2,"ioctl - vidioc_queryctrl:%u\n",qc->id);

    return ret;
}



/**@brief get value of a control
To get the current value of a control applications initialize the id field of a struct v4l2_control 
and call the VIDIOC_G_CTRL ioctl with a pointer to this structure. 

When the id is invalid drivers return an EINVAL error code. 
*/
static int vidioc_g_ctrl(struct file *file, void *priv,
                         struct v4l2_control *ctrl)
{
    int ret;
	ret = -EINVAL;
    csi_debug(2,"ioctl - vidioc_g_ctrl\n");

    return ret;
}



/**@brief set value of a control
To change the value of a control applications initialize the id and value fields of a 
struct v4l2_control and call the VIDIOC_S_CTRL ioctl.

When the id is invalid drivers return an EINVAL error code. 
When the value is out of bounds drivers can choose to take the closest valid value or 
return an ERANGE error code, whatever seems more appropriate. 
However, VIDIOC_S_CTRL is a write-only ioctl, it does not return the actual new value.
*/
static int vidioc_s_ctrl(struct file *file, void *priv,
                         struct v4l2_control *ctrl)
{
    int ret;
	ret = -EINVAL;
    csi_debug(2,"ioctl - vidioc_s_ctrl\n");

    return ret;
}


/**@brief Get frame rate
The current video standard determines a nominal number of frames per second. 
If less than this number of frames is to be captured, applications can request frame 
skipping or duplicating on the driver side. 
This is especially useful when using the read() or write(), which are not augmented by timestamps 
or sequence counters, and to avoid unneccessary data copying.

Further these ioctls can be used to determine the number of buffers used internally by a driver 
in read/write mode. For implications see the section discussing the read() function.

To get and set the streaming parameters applications call the VIDIOC_G_PARM and VIDIOC_S_PARM ioctl, 
respectively. They take a pointer to a struct v4l2_streamparm which contains a union holding separate 
parameters for input and output devices.
*/
static int vidioc_g_parm(struct file *file, void *priv,
                         struct v4l2_streamparm *parms)
{
    int ret;
	ret = -EINVAL;
    csi_debug(2,"ioctl - vidioc_g_parm\n");
 
    return ret;
}


/**@brief set frame rate
see above
*/
static int vidioc_s_parm(struct file *file, void *priv,
                         struct v4l2_streamparm *parms)
{
    int ret;
	ret = -EINVAL;
    csi_debug(2,"ioctl - vidioc_s_parm\n");
  
    return ret;
}



/**@brief read multible frames (count) from the videobuffer queue to the application
NOTE: general possible IO Methods:
- read
- mmap
- userpointer
Read file I/O
we only support mmap !! but...
Read needs to start and stop the engine to capture one buffer. Tomk...done by vb2!
*/
static ssize_t csi_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
    struct csi_camera *pcam = video_drvdata(file);

	 csi_debug(2,"read called\n");
		Error8++; //read

		return vb2_read(&pcam->vb2_vidq, data, count, ppos, file->f_flags & O_NONBLOCK);									

}


/**@brief wait for completion of a capture. called from user app
*/
static unsigned int csi_poll(struct file *file, struct poll_table_struct *wait)
{
    struct csi_camera *pcam = video_drvdata(file);
		Error9++; //poll
	    csi_debug(1,"ioctl - csi_poll\n");

	return vb2_poll(&pcam->vb2_vidq, file, wait);

}




//+++:+++++++++++++++++++++++++++++++++++++++++++++++++   driver service functions


/**@brief File operations, open

open the csi module
*/
static int csi_open(struct file *file)
{
    struct csi_camera *pcam = video_drvdata(file);


    csi_debug(3,"- csi_open\n");
	Error1=Error2=Error3=Error4=Error5=Error6=Error7=Error8=Error9=Error10=Error11=0;
    if (pcam->opened == 1)
    {
        csi_debug(3,"***ERROR:device open called more than once!\n");
        return -EBUSY;
    }
	// save the pointer to our camera datastructure in the kernels file structure.
	// it will be given as the priv pointer in later ioctl calls
	file->private_data = pcam; //tomk



 
    INIT_LIST_HEAD(&pcam->dmaque);

    pcam->opened = 1;
    return v4l2_fh_open(file); //v4l2-fh.c. new way for  file handle specific data, required for new drivers
}

/** file operations, close
*/

/**@brief 
*/
static int csi_close(struct file *file)
{
    struct csi_camera *pcam = video_drvdata(file);
 
    csi_debug(2,"- csi_close\n");
	csi_debug(3,"++Close++++++++Error1:%lu Error2:%lu Error3:%lu Error4:%lu Error5:%lu Error6:%lu Error7:%lu Error8:%lu Error9:%lu Error10:%lu Error11:%lu\n",Error1,Error2,Error3,Error4,Error5,Error6,Error7,Error8,Error9,Error10,Error11);

 
	vb2_queue_release(&pcam->vb2_vidq);

    pcam->opened=0;
    IsCapturing = 0;
	
	

	return v4l2_fh_release(file);
}

/** memory-mapped file I/O
The mmap() function asks to map length bytes starting at offset in the memory of the device specified by fd 
into the application address space, preferably at address start
*/

/**@brief 
*/
static int csi_mmap(struct file *file, struct vm_area_struct *vma)
{
    struct csi_camera *pcam = video_drvdata(file);
    int ret;
Error11++; //mmap
    csi_debug(2,"mmap called, vma=0x%08lx\n", (unsigned long)vma);

	ret = vb2_mmap(&pcam->vb2_vidq, vma); // map one buffer

    csi_debug(1,"vma start=0x%08lx, size=%ld, ret=%d\n",
              (unsigned long)vma->vm_start,
              (unsigned long)vma->vm_end - (unsigned long)vma->vm_start,
              ret);
		  
    return ret;
}





//+++:+++++++++++++++++++++++++++++++++++++++++++++++++++++  device structures


// ioctl interface:
// device ioctl operations
static const struct v4l2_ioctl_ops csi_ioctl_ops =
{
    .vidioc_querycap          = vidioc_querycap,       		// get capabilitys of this device
    .vidioc_enum_fmt_vid_cap  = vidioc_enum_fmt_vid_cap,	// return info of a given video format
    .vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap,		// get current active video format
    .vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap,		// try video format. try your best matching format, most used
    .vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap,		// set a given video format

    .vidioc_reqbufs           = vidioc_reqbufs,				// Initiate Memory Mapping or User Pointer I/O
    .vidioc_querybuf          = vidioc_querybuf,			// Query the status of a buffer
    .vidioc_qbuf              = vidioc_qbuf,				// enqueue a free user buffer to driverqueue
    .vidioc_dqbuf             = vidioc_dqbuf,				// dequeue a used buffer from driverqueue
	.vidioc_create_bufs		= vidioc_create_bufs,		// new in vb2, Allocate buffers and any required auxiliary structs
	.vidioc_prepare_buf		= vidioc_prepare_buf,		// new in vb2, Pass ownership of a buffer from userspace to the kernel
//	.vidioc_expbuf			= vidioc_expbuf,			// new in vb2, Export a buffer as a file descriptor
    .vidioc_streamon          = vidioc_streamon,			// start videostream, later calls vidioc_streamon
    .vidioc_streamoff         = vidioc_streamoff,		// stop videostream,  later calls vidioc_streamoff

	
    .vidioc_enum_input        = vidioc_enum_input,			// return attributes of a given video input
    .vidioc_g_input           = vidioc_g_input,				// Query the current video input number
    .vidioc_s_input           = vidioc_s_input,				// select the current video input number
    .vidioc_queryctrl         = vidioc_queryctrl,			// return the attributes of a given control
    .vidioc_g_ctrl            = vidioc_g_ctrl,				// get controlvalue of given control
    .vidioc_s_ctrl            = vidioc_s_ctrl,				// set controlvalue of given control
    .vidioc_g_parm		 	  = vidioc_g_parm,				// get current streaming parameters
    .vidioc_s_parm		  	  = vidioc_s_parm,				// set streaming parameters ie. frame skipping, number of buffers


/*
    .vidioc_g_chip_ident	= viacam_g_chip_ident,			// read chip ids (experimental)
    .vidioc_s_std		= viacam_s_std,						// set video standard like PAL SECAM
	
    .vidioc_enum_framesizes = viacam_enum_framesizes,		// Enumerate frame sizes (experimental)
    .vidioc_enum_frameintervals = viacam_enum_frameintervals, //Enumerate frame intervals
*/
};



// device file operations  fops
static const struct v4l2_file_operations csi_fops =
{
    .owner	  = THIS_MODULE,
    .unlocked_ioctl = video_ioctl2, //in upper layer V4L2 ioctl handler
	
    .open	  = csi_open, 
    .release  = csi_close,
    .read     = csi_read,
    .mmap     = csi_mmap,
    .poll	  = csi_poll,
/*	
	// the following functions (vb2_ops_...) are all in videobuf2-v4l2.c
	.open = v4l2_fh_open, //v4l2-fh.c. new way for  file handle specific data, required for new drivers
	.release = vb2_fop_release,
	.read = vb2_fop_read,
	.mmap = vb2_fop_mmap,
	.poll = vb2_fop_poll,	
*/	
};




// all device operations:
static struct video_device csi_template =
{
    .name		= "csi",
    .fops       = &csi_fops,
    .ioctl_ops 	= &csi_ioctl_ops,
    .release	= video_device_release,   //kernel function!
};







// +++:++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++   mayor functions



/*
This probe function starts the device initialization:
initializing hardware, allocating resources, and registering the device with the kernel as a v4l2 device.
platform devcice probe function!
*/

/**@brief startup processing when loading the driver
*/
static int csi_probe(struct platform_device *pdev)
{
    struct csi_camera *pcam;
	struct vb2_queue *q;
    struct video_device *pviddev;
    int ret = 0;


    csi_debug(3,"------------------- csi_probe start>>\n");

    // alloc memory for my big csi_camera structure containing all cam-info. this is we dynamicly allocate a structure instance.(to save kernel/module size)
	// it will be the state container for this instance of the device driver
    pcam = kzalloc(sizeof(struct csi_camera), GFP_KERNEL); // zeros the area!
    if (!pcam)
    {
        csi_debug(3,"***ERROR:request csi_camera mem failed!\n");
        return -ENOMEM;
    }
	csi_debug(2,"sizeof struct csi_camera:%d\n",sizeof(struct csi_camera));
	
		pcamera=pcam;
	
    pcam->id = pdev->id; // save the platform_device id = csi port 0 or 1
	csi_debug(3,"pcam->id:%u\n",pcam->id);
    pcam->pdev = pdev;  // and the address of the platform_device strucutre.

    spin_lock_init(&pcam->slock);  //init/create a locking mechanism

csi_debug(2,"v4l2_device_register start \n");

    /* v4l2 device register */
    // register v4l2_device struct. while registered device is in use.
    ret = v4l2_device_register(&pdev->dev, &pcam->v4l2_dev);
    if (ret)
    {
        csi_debug(3,"***ERROR:Error registering v4l2 device\n");
        goto err_irq;

    }
	
csi_debug(2,"v4l2_device_registered\n");

    dev_set_drvdata(&(pdev)->dev, (pcam));  // set address of our camera struct for later usage as priv pointer. in dd.c

	
		// set default videoformat: 
	pcam->fourcc = V4L2_PIX_FMT_YUYV;
    pcam->width = 640;
    pcam->height = 480;
	pcam->bitsperpixel = 16;
	pcam->bytesperline = (pcam->bitsperpixel * pcam->width)>>3;
	pcam->frame_size = pcam->bytesperline * pcam->height;



        // set camera powersupply options
        pcam->iovdd = NULL;
        pcam->avdd = NULL;
        pcam->dvdd = NULL;

    pcam->input=0;//default v4l2-input

 csi_debug(2,"vd alloc\n");

    /*video device register	*/
    ret = -ENOMEM;
    pviddev = video_device_alloc(); // allocate memory for video_device struct
    if (!pviddev)
    {
        goto err_clk;
    }

    *pviddev = csi_template;  //init with default values CSI
    pviddev->v4l2_dev = &pcam->v4l2_dev;
csi_debug(2,"vd register start\n");

    dev_set_name(&pviddev->dev, "csi-1"); // set video device name
    ret = video_register_device(pviddev, VFL_TYPE_GRABBER, video_nr); // register video device
    if (ret < 0)
    {
csi_debug(3,"video_register_failed\n");
        goto rel_vdev;
    }
	
	/* now we have kernel allocated memory for our driver structures "csi_camera" and "video_device".
	the addresses are now given to v4l2 kernel to be used in subsequent calls into this driver.
	video_set_drvdata() does that.
	We can later retrieve the pointers from the kernel with the function video_drvdata().
	This is used throughout this driver code!
	*/
	video_set_drvdata(pviddev, pcam); // register private data pointer pcam. in v4l2-dev.h.  Calls dev_set_drvdata() !!!
	
csi_debug(2,"video_register_deviceed\n");

     pcam->pviddev = pviddev; //add video device struct pointer

    csi_debug(2,"V4L2 device registered as %s\n",video_device_node_name(pviddev));

    //initialize the videobuf2 queue
	q = &pcam->vb2_vidq;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP; // | VB2_USERPTR | VB2_READ;
	q->drv_priv = pcam;
	q->buf_struct_size = sizeof(struct cam_buffer);
	q->ops = &csi_videoq_ops;
	q->mem_ops = &vb2_dma_contig_memops;
//	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
//	q->gfp_flags = GFP_DMA32;
//	q->min_buffers_needed = 1;
	


//	if (vb2_queue_init(q))  csi_debug(3,"-que init failed\n");
csi_debug(2,"vb2 que init done\n");


    /* init list video dma queue */
    INIT_LIST_HEAD(&pcam->dmaque);
    csi_debug(3,"--------------------------------probe-end successfull <<<<\n");
    return 0;

rel_vdev:
    video_device_release(pviddev);
err_clk:


    v4l2_device_unregister(&pcam->v4l2_dev);

err_irq:

    kfree(pcam);

    csi_debug(3,"***csi_probe-ERROR:failed to install\n");

    return ret;
}

// platform device release function specified in platform device struct. 
//must be there, otherwise we get a kernel complains!

/**@brief 
*/
void csi_dev_release(struct device *pdevice)
{
    csi_debug(2,"- dev_release. no function\n");

}



//platform driver remove

/**@brief 
*/
static int __devexit csi_remove(struct platform_device *pdev)
{
    struct csi_camera *pcam=(struct csi_camera *)dev_get_drvdata(&(pdev)->dev);

    csi_debug(2,"- csi_remove\n");
	
	if (pcam == NULL)
	csi_debug(3,"get_drvdata returned null!!\n");


        v4l2_info(&pcam->v4l2_dev, "unregistering %s\n", video_device_node_name(pcam->pviddev));
        video_unregister_device(pcam->pviddev);  //unregister video device. does not return anything!
       v4l2_device_unregister(&pcam->v4l2_dev); //unregister linux device. does not return anything!
		csi_debug(2,"v4l2 and video device unregistered!\n");
         kfree(pcam);
    

    return 0;
}

//platform device suspend

/**@brief called by power management, standby on (should also stop capture)
usually never called
used, if the system goes to a low power state.
stby_mode indicates what happens of the A20 cpu enters low power state.
*/
static int csi_suspend(struct platform_device *pdev, pm_message_t state)
{


    csi_debug(2,"- csi_suspend\n");

 
    return 0;
}

//platform device resume

/**@brief called by power management, standby off 
usually never called.
used, if the system goes to a low power state.
*/
static int csi_resume(struct platform_device *pdev)
{

    csi_debug(2,"- csi_resume\n");


    return 0;
}




//+++:++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/*
The connected camera uses 2 platform drivers:
- the csi0 interface
- the i2c interface
both of which are not discoverable and part of the cpu chip.

The attached camera however is discoverable by probing via th ei2c bus.

This implementation does not use the i2c interface !!!!

struct platform_driver {
	int (*probe)(struct platform_device *);
	int (*remove)(struct platform_device *);
	void (*shutdown)(struct platform_device *);
	int (*suspend)(struct platform_device *, pm_message_t state);
	int (*resume)(struct platform_device *);
	struct device_driver driver;
	const struct platform_device_id *id_table;
};
*/

// device interface:
static struct platform_driver csi_driver =
{
    .probe		= csi_probe,
    .remove		= __devexit_p(csi_remove),
    .suspend	= csi_suspend,
    .resume		= csi_resume,
    //.id_table	= csi_driver_ids,
    .driver = {  // the device driver
        .name	= "sun4i_csi",
        .owner	= THIS_MODULE,
    }
};

// the bus control registers(mem) and interrupt(irq) resources!
#ifdef CSIPORT0
static struct resource csi0_resource[] =
{
    [0] = {
        .start	= CSI0_REGS_BASE,
        .end	= CSI0_REGS_BASE + CSI0_REG_SIZE - 1,
        .flags	= IORESOURCE_MEM,
    },
    [1] = {
        .start	= SW_INTC_IRQNO_CSI0,
        .end	= SW_INTC_IRQNO_CSI0,
        .flags	= IORESOURCE_IRQ,
    },
};
#else
static struct resource csi1_resource[] =
{
    [0] = {
        .start	= CSI1_REGS_BASE,
        .end	= CSI1_REGS_BASE + CSI1_REG_SIZE - 1,
        .flags	= IORESOURCE_MEM,
    },
    [1] = {
        .start	= SW_INTC_IRQNO_CSI1,
        .end	= SW_INTC_IRQNO_CSI1,
        .flags	= IORESOURCE_IRQ,
    },
};
#endif


// the platform-device structure !!!!+++:++++++++++++++++++++++++
/*
struct platform_device {
	const char	* name;
	int		id;
	struct device	dev;
	u32		num_resources;
	struct resource	* resource;

	const struct platform_device_id	*id_entry;

	// MFD cell pointer
	struct mfd_cell *mfd_cell;

	// arch specific additions
	struct pdev_archdata	archdata;
};
*/

#ifdef CSIPORT0
static struct platform_device csi_device =
{

    .name           	= "sun4i_csi",   // this drivers name is sun4i_csi and it can handle deices of the same name.
    .id             	= 0,
    .num_resources		= ARRAY_SIZE(csi0_resource),   // how many resources did we specify
    .resource       	= csi0_resource,               // where are our resources
    .dev.release      = csi_dev_release,   // device release function. must be there, otherwise we get a kernel complain!
};
#else
static struct platform_device csi_device =
{

    .name           	= "sun4i_csi",   // this drivers name is sun4i_csi and it can handle deices of the same name.
    .id             	= 1,
    .num_resources		= ARRAY_SIZE(csi1_resource),   // how many resources did we specify
    .resource       	= csi1_resource,               // where are our resources
    .dev.release      = csi_dev_release,   // device release function. must be there, otherwise we get a kernel complain!
};
#endif






// registers csi platform-device and platform-driver. called on modul_init
static int __init csi_init(void)
{
    u32 ret;

    csi_debug(2,"Welcome to CSI driver\n");
    csi_debug(2,"- csi_init\n");

    /*
    int script_parser_fetch(char *main_name, char *sub_name, int value[], int count);
    this function reads config-settings from the script.bin file fex-file.
    */
    // driver registration with a list of functions and a list of devices this driver can handle. here, its only one device, the sun4i_csi
    ret = platform_driver_register(&csi_driver);  // register my driver with the kernel and give the kernel a list of devices (one) this driver is able to service,

    if (ret)
    {
        csi_debug(3,"***ERROR:platform driver register failed\n");
        return -1;
    }

    ret = platform_device_register(&csi_device); // register the above mentioned platform-device sun4i_csi with the kernel.
    if (ret)
    {
        csi_debug(3,"***ERROR:platform device register failed\n");
        return -1;
    }
    return 0;
}

// unregisters csi platform-device and platform-driver. called on modul_exit
static void __exit csi_exit(void)
{

    csi_debug(2,"- csi_exit\n");


	// unregister platform device csi
        platform_device_unregister(&csi_device); 
        platform_driver_unregister(&csi_driver); 
 
}
/** Overview v4l2 call interface:
init/exit = register/unregister platform driver and platform device.
probe = register v4l2-device, video-device
remove = unregister v4l2-device, video-device
*/
module_init(csi_init);  // is called first, when module is loaded, modprobe sun4i_csi0 : sequence: csi_init, csi_probe, csi_open, csi_close
module_exit(csi_exit);  // is called when module is removed ,rmmod. sequence: csi_exit, csi_remove, dev_release

MODULE_AUTHOR("Tom Krueger");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("CSI driver for sun4i");
