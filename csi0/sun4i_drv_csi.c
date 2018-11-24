/* https://github.com/xenpac/Camera-CSI-Driver-Linux-Sunxi.git
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
 Driver file for both csi0 and csi1, see define CSIPORT0 below to switch driver
 
 all functions are declared static! as well as data
 
Some Notes on the used videobuf framework here on kernel 3.4 with sunxi:
- its..bugy..with lemaker addins in videobuf-dma-contig.c
- it uses some dma-contig functions but no kernel dma.
- videobuf2...well not much better

Picture Sizes:
Note: CSI Fifos capture one complete Line at a time! Then DMA to memory during horizontal blank.
 The max. Pixelclocks/Bytes per Line the CSI-Module can handle is 4096 Bytes (fifo-size).
 The max. Lines per Frame the CSI-Module can handle is also 4096  (12Bits)
 Depending on the Videoformat one Pixel has a specific Bitlength (depth) of information. common values are 8, 12 or 16 bits per pixel.
 So the max. Pixel per line is reduced to: 4096*8/Bitlength); for 8bit=4096; for 12bit=2730, for 16bit=2048 (pixel per line)
 So the max.Framesize can be max: 4096*4096.
 
 Planar formats use one fifo for each channel ( Y, U, V). CSI Module seperates the CAM data into  the 3 channels!
 So in this case (YUV422 planar) the max picture size can be indeed 4096'4096 !!
 
 

	script_parser_fetch:  this is an allwinner specific file containig a specific configuration of all the soc registers.
	the kernel loads script.bin into memory. the function script_parser_fetch() reads specified values from this memory region to
	give info on current register settings.

	structure of this driver:
	- the driver itself registers as a v4l2 video device
	- the attached camera module is registered as a v4l2_subdev device with i2c port preallocated.
	  (uses the code in fe. oc5640.c)
	v4l2_subdev_call = calls the cameras driver (the subdev) to perform an action.
	This is not i2c, it calls a function in the fe. ov5740.c file.
	Note that the camera-module specified in script.bin will be loaded as a subdevice to this driver!
	All camera related functions will be called through the subdevice interface, fe. ov5640.

	The i2c adapter itself must be loaded in this driver and passed to the camera subdevice for io.
	
	modprobe parameters are (example): modprobe cam=oc7670 i2c_addr=0x78
	if only one parameter is specified, the parameters are ignored.
	a wrong i2c_addr (of the cam module) will result in not working device!
	
	Thomas Krueger, Oct 2017
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

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 20)
#include <linux/freezer.h>
#endif

#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-common.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf-dma-contig.h>
#include <linux/moduleparam.h>

#include <plat/sys_config.h>
#include <mach/clock.h>
#include <mach/irqs.h>
#include <linux/regulator/consumer.h>

#include "../include/sun4i_csi_core.h"
#include "../include/sun4i_dev_csi.h"
#include "sun4i_csi_reg.h"

// this driver works for which port ?? !
#define CSIPORT0

//for internel driver debug
#define DBG   		1
//debug level 0~3: 3=only errors,2=+ routine trace, 1=+ details, 0= + interupt messages
#define DBG_LEV 	3

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

//#define AJUST_DRAM_PRIORITY
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

static unsigned IsCapturing=0;

// the module parameter values
static char cam_name[I2C_NAME_SIZE];  // csi camera module (cam_name) , name
static uint i2c_addr;  //its i2c address

// debug counters!
static unsigned long Error1,Error2,Error3,Error4,Error5,Error6,Error7,Error8,Error9,Error10,Error11;

/* modprobe sun4i_csi0 cam=name i2c_addr=0xaa , so you can specify which cameramodule to be used!
 driver moduels parameters: (S_IRUGO|S_IWUSR are file permissions as defines)
 below two functions to copy MODUL PARAMETERS, so can specify in modprobe command optionally
 NOTE: specifying a cam module-parameter makes the kernel to load the named modul BEFORE this driver is installed,
 so it is available at entry. Thus the associated camera sub-device driver is available.
 */
module_param_string(cam, cam_name, sizeof(cam_name), S_IRUGO|S_IWUSR); // copy modul param "cam" to variable cam_name
module_param(i2c_addr,uint, S_IRUGO|S_IWUSR); //copy modul param i2c_addr to variable i2c_addr as uint

static struct i2c_board_info  dev_sensor = 
{
        .platform_data	= NULL,
};

//the following color formats are supported for attached cameras. along with its csi-port format config
/**
This table contains info about video formats:
name = user readable format name used in struct v4l2_fmtdesc
fourcc = 4-character pixelformat-code used in struct v4l2_fmtdesc
cam_fmt = videosize in pixel etc,  in struct v4l2_format, and in struct v4l2_mbus_framefmt used in subdevice.
input_fmt = csi bus-data format of the capture port
output_fmt = csi bus-data format going into the memory/framebuffer
depth = bytes per pixel (depend on format)
planes_cnt = ??

NOTE:
Data storage for an image can be pixel-oriented (interleaved) or planar-oriented (planar). 
For images in pixel order (interleaved), all channel values (YUV) for each pixel are clustered and stored consecutively. 
Their layout depends on the color model and downsampling scheme.

Planar means that in memory there is one array of luminance values, one array of U_color and one array of V_color.
These 3 tables are easier to manage by software.(codecs)
The three tables could however be stored in one buffer,  all Y first, then all U, then all V.
It needs to know the pixels per image however! to seperate the sections.

Non-planar storage would not really need to know about picture size as all info intermixed (Pixel interleaved).

The CSI module in YUV422 planar mode, uses fifo-0 for Y, fifo-1 for U and fifo-2 for V.!!!
There dma bufferaddress must each be set to the correct offset in the destination buffer.

FOURCC codes:
first letter is format type: P = planar; Y=packet(interleaved)  
second number tells the chroma sampling: 0=420; 1=421; 2=422; 4=444;
the last 2 digits tell the bits per pixel.

*/


//+++:+++++++++++++++++++++++++++++++++++++++++++  csi device subfunctions subroutines






/**@brief set DMA destination memory address of CSI fifo 
@param *pcam = poniter to camera struct for register base address
@param buf = the wanted fifo to set the address to
@param addr = the memory address(buffer) for the fifo to do DMA.
*/
void static inline bsp_csi_set_buffer_address(struct csi_camera *pcam,enum csi_buf_e buf, u32 addr)
{
    //bufer0a +4 = buffer0b, bufer0a +8 = buffer1a
    W(pcam->regs+CSI_REG_BUF_0_A + (buf<<2), addr);
}


/**@brief set all DMA addresses of the three csi fifos depending on conversion protocol
@param *pcam = poniter to camera struct for register base address
@param *buffer = buffer address in memory to be used for DMA
*/
static inline void csi_set_addr(struct csi_camera *pcam,struct videobuf_buffer *buffer)
{

    dma_addr_t addr_org;

   csi_debug(0,"csi_set_addr\n");


    addr_org = buffer->boff; //videobuf_to_dma_contig(buffer); // get memory address of buffer


    if(pcam->csi_mode.input_fmt==CSI_RAW)
    {
        pcam->csi_buf_addr.y  = addr_org;  // no planes, all data contiguous
        pcam->csi_buf_addr.cb = addr_org;  // not used
        pcam->csi_buf_addr.cr = addr_org;   // not used 

    }
    else if(pcam->csi_mode.input_fmt==CSI_BAYER)
    {
        //really rare here
        pcam->csi_buf_addr.cb = addr_org;//for G channel
        pcam->csi_buf_addr.y  = addr_org + pcam->width*pcam->height*1/2;//for B channel
        pcam->csi_buf_addr.cr = addr_org + pcam->width*pcam->height*3/4;//for R channel

    }
    else if(pcam->csi_mode.input_fmt==CSI_CCIR656)
    {
        //TODO:

    }
    else if(pcam->csi_mode.input_fmt==CSI_YUV422)
    {

        switch (pcam->csi_mode.output_fmt)
        {
        case CSI_PLANAR_YUV422: //bps=16
            pcam->csi_buf_addr.y  = addr_org;  //start Y at offset 0
            pcam->csi_buf_addr.cb = addr_org + pcam->width*pcam->height; //start U at offset1
            pcam->csi_buf_addr.cr = addr_org + pcam->width*pcam->height*3/2;  //start V at offset2
            break;

        case CSI_PLANAR_YUV420: //bps=12
            pcam->csi_buf_addr.y  = addr_org;
            pcam->csi_buf_addr.cb = addr_org + pcam->width*pcam->height;
            pcam->csi_buf_addr.cr = addr_org + pcam->width*pcam->height*5/4;
            break;

        case CSI_UV_CB_YUV422:
        case CSI_UV_CB_YUV420:
        case CSI_MB_YUV422:
        case CSI_MB_YUV420:
            pcam->csi_buf_addr.y  = addr_org;
            pcam->csi_buf_addr.cb = addr_org + pcam->width*pcam->height;  //U and V combined
            pcam->csi_buf_addr.cr = addr_org + pcam->width*pcam->height;  // not used
            break;

        default:
            break;
        }
    }

	// setup the destination dma bufferaddresses for each fifo
    bsp_csi_set_buffer_address(pcam, CSI_BUF_0_A, pcam->csi_buf_addr.y); // start Y
    bsp_csi_set_buffer_address(pcam, CSI_BUF_0_B, pcam->csi_buf_addr.y); // startY alternate buffer if doublebuffermode is set.
    bsp_csi_set_buffer_address(pcam, CSI_BUF_1_A, pcam->csi_buf_addr.cb); //start U
    bsp_csi_set_buffer_address(pcam, CSI_BUF_1_B, pcam->csi_buf_addr.cb); // start U alternate
    bsp_csi_set_buffer_address(pcam, CSI_BUF_2_A, pcam->csi_buf_addr.cr); //start V
    bsp_csi_set_buffer_address(pcam, CSI_BUF_2_B, pcam->csi_buf_addr.cr); //start V alternate

    //csi_debug(1,"csi_buf_addr_y=%x\n",  pcam->csi_buf_addr.y);
    //csi_debug(1,"csi_buf_addr_cb=%x\n", pcam->csi_buf_addr.cb);
    //csi_debug(1,"csi_buf_addr_cr=%x\n", pcam->csi_buf_addr.cr);

}


/**@brief get into pcam: csi_ahb_clk, csi_clk_src, csi_module_clk, csi_isp_src_clk, csi_dram_clk
*/
static int csi_clk_get(struct csi_camera *pcam)
{
    int ret;
#ifdef CSIPORT0
    pcam->csi_ahb_clk=clk_get(NULL, "ahb_csi0");
#else
    pcam->csi_ahb_clk=clk_get(NULL, "ahb_csi1");
#endif
    if (pcam->csi_ahb_clk == NULL)
    {
        csi_debug(3,"***ERROR:get csi0 ahb clk error!\n");
        return -1;
    }

    if(pcam->csi_sig_cfg.mclk==24000000 || pcam->csi_sig_cfg.mclk==12000000)
    {
        pcam->csi_clk_src=clk_get(NULL,"hosc");
        if (pcam->csi_clk_src == NULL)
        {
            csi_debug(3,"***ERROR:get csi0 hosc source clk error!\n");
            return -1;
        }
    }
    else
    {
        pcam->csi_clk_src=clk_get(NULL,"video_pll1");
        if (pcam->csi_clk_src == NULL)
        {
            csi_debug(3,"***ERROR:get csi0 video pll1 source clk error!\n");
            return -1;
        }
    }
#ifdef CSIPORT0
    pcam->csi_module_clk=clk_get(NULL,"csi0");
#else
    pcam->csi_module_clk=clk_get(NULL,"csi1");
#endif
	
    if(pcam->csi_module_clk == NULL)
    {
        csi_debug(3,"***ERROR:get csi0 module clk error!\n");
        return -1;
    }

    ret = clk_set_parent(pcam->csi_module_clk, pcam->csi_clk_src);
    if (ret == -1)
    {
        csi_debug(3,"***ERROR: csi set parent failed \n");
        return -1;
    }

    clk_put(pcam->csi_clk_src);

    ret = clk_set_rate(pcam->csi_module_clk,pcam->csi_sig_cfg.mclk);
    if (ret == -1)
    {
        csi_debug(3,"***ERROR:set csi0 module clock error\n");
        return -1;
    }

    pcam->csi_isp_src_clk=clk_get(NULL,"video_pll0");
    if (pcam->csi_isp_src_clk == NULL)
    {
        csi_debug(3,"***ERROR:get csi_isp source clk error!\n");
        return -1;
    }

    pcam->csi_isp_clk=clk_get(NULL,"csi_isp");
    if(pcam->csi_isp_clk == NULL)
    {
        csi_debug(3,"***ERROR:get csi_isp clk error!\n");
        return -1;
    }

    ret = clk_set_parent(pcam->csi_isp_clk, pcam->csi_isp_src_clk);
    if (ret == -1)
    {
        csi_debug(3,"***ERROR: csi_isp set parent failed \n");
        return -1;
    }

    clk_put(pcam->csi_isp_src_clk);

    ret = clk_set_rate(pcam->csi_isp_clk, CSI_ISP_RATE);
    if (ret == -1)
    {
        csi_debug(3,"***ERROR:set csi_isp clock error\n");
        return -1;
    }

#ifdef CSIPORT0
    pcam->csi_dram_clk = clk_get(NULL, "sdram_csi0");
#else
    pcam->csi_dram_clk = clk_get(NULL, "sdram_csi1");
#endif
 	
    if (pcam->csi_dram_clk == NULL)
    {
        csi_debug(3,"***ERROR:get csi0 dram clk error!\n");
        return -1;
    }

    return 0;
}


//clock configure

/**@brief set camera clock asto parameter mclk in pcam
*/
static int csi_clk_out_set(struct csi_camera *pcam)
{
    int ret;
    ret = clk_set_rate(pcam->csi_module_clk, pcam->csi_sig_cfg.mclk);
    if (ret == -1)
    {
        csi_debug(3,"***ERROR:set csi0 module clock error\n");
        return -1;
    }

    return 0;
}





/**@brief turn on csi module clocks
*/
static int csi_clk_enable(struct csi_camera *pcam)
{
    clk_enable(pcam->csi_ahb_clk);  //\linux-sunxi-lemaker-3.4\arch\arm\mach-sun7i\clock\clock.c
//	clk_enable(pcam->csi_module_clk);
    clk_enable(pcam->csi_isp_clk); // linux-sunxi-lemaker-3.4\arch\arm\mach-sun4i\clock\clock.c
    clk_enable(pcam->csi_dram_clk);
    clk_reset(pcam->csi_module_clk, 0);

    return 0;
}


/**@brief turn off csi module clock
*/
static int csi_clk_disable(struct csi_camera *pcam)
{
    clk_disable(pcam->csi_ahb_clk);
//	clk_disable(pcam->csi_module_clk);
    clk_disable(pcam->csi_isp_clk);
    clk_disable(pcam->csi_dram_clk);
    clk_reset(pcam->csi_module_clk, 1);

    return 0;
}


/**@brief set csi_ahb_clk, csi_module_clk, csi_dram_clk
*/
static int csi_clk_release(struct csi_camera *pcam)
{
    clk_put(pcam->csi_ahb_clk);
    pcam->csi_ahb_clk = NULL;

    clk_put(pcam->csi_module_clk);
    pcam->csi_module_clk = NULL;

    clk_put(pcam->csi_dram_clk);
    pcam->csi_dram_clk = NULL;

    return 0;
}



/**@brief clear interrupt flags of the CSI video buffer engine
*/
void static inline bsp_csi_int_clear_status(struct csi_camera *pcam,enum csi_int_e interrupt)
{
    W(pcam->regs+CSI_REG_INT_STATUS, interrupt);
}


/**@brief IRQ handler csi module
The CSI module will fill a given buffer with video data from current cameraframe.
When finished, it will generate an interrupt which lands in this routine.
The CSI module needs the address of the buffer BEFORE it gets the cameraframe!
So before the capture is enabled there must be at least one buffer queued from the application,
otherwise the data will be written to nomansland!
Data is actually written directly to the application queued buffer without the need of copying.
At startup at least one buffer must be queued!
*/
static irqreturn_t csi_isr(int irq, void *priv)  //++++ interrupt !!! 
{
    struct videobuf_buffer *buf;
    struct csi_camera *pcam = (struct csi_camera *)priv;
    struct list_head *dma_q = &pcam->dmaque;


    csi_debug(0,"--csi_isr:\n");

// a frame was captured...
	
    spin_lock(&pcam->slock); // interlocking with vb_buf_queue/videobuf

	if (!IsCapturing) goto unlock; 

	
	// if dma-buffer list is empty, just skip this capture. The app did not queue new buffer. this should not happen!
    if (list_empty(dma_q))
    {
        csi_debug(0,"++ret:frame filled not on queue\n"); // app is way too slow!
		Error1++; //que-empty: the frame filled was not on the queue, so we cant get its pointer!
        goto unlock; //fill the last buffer again..there must have been at least one!!!! else big trouble! nomansland-write/kernel mem-fault!
    }
	
	// check if there is only one buffer on the queue. if this happens, the app is too slow or apps fps is lower than we supply.
	if (dma_q->next->next == dma_q)
	{
		Error2++; //only one buffer, cannot que next
		// we stay on that buffer until a second one is supplied by the application
		goto unlock;
	}
	// now we have at least two buffers on the dma-queue!

	// get address of dma-buffer-structure we just filled. (The first list member)	
    buf = list_entry(dma_q->next,struct videobuf_buffer, queue);
    csi_debug(0,"++ret:buf ptr=%p\n",buf);

 
   
	/* check if any app is waiting for this buffer.
	if 0 is returned, there is no process waiting for that buffer. app is too slow, or apps fps is lower than we supply.
	we keep staying on this buffer....
	*/
    if (!waitqueue_active(&buf->done)) // just check if wait_queue is empty for this buffer
    {
		Error3++; // buffer is not expected by anyone. 
        csi_debug(0,"++ret:Nobody is waiting on this buffer\n"); // app is too slow. 
		//goto unlock; // give videobuf time to wait for this buffer..(proper solution!).... but fps is a bit faster not to do it!
    }


 //   buf->field_count++; // keep a sequential frame count

    buf->state = VIDEOBUF_DONE; // this tells videobuf, that new data is filled. 
    wake_up(&buf->done); // wake up waiting task from poll. she may fetch the buffer now
    list_del(&buf->queue); // remove that videobuf buffer from dma queue, it has been serviced! 
	

//queue_next_buf:	
		Error4++; // que next buf, and last successfull commited

    buf = list_entry(dma_q->next,struct videobuf_buffer, queue); // get next buffer from beginning of dma queue

	if (buf->state != VIDEOBUF_QUEUED) csi_debug(0,"++next Buf not queued-flag\n");

    csi_set_addr(pcam,buf); // tell csi fifos about its address
	buf->state = VIDEOBUF_ACTIVE; // videobuf..dont touch it!
	// dont set VIDEOBUF_ACTIVE flag !!! somehow messes videobuf up. if set, videobuf wont touch/update the buffer!
unlock:
    spin_unlock(&pcam->slock);

    bsp_csi_int_clear_status(pcam,CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE
	Error5++; //number of ints

    return IRQ_HANDLED;
}






//+++:++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  videobuf layer functions
//Videobuf operations. buffer management for V4L2 drivers

/**@brief register a videobuffer queue with the videobuf layer
user called VIDIOC_REQBUFS
- here vidioc_regbufs
- here vb_buf_setup

Here the driver shall check, if the number of buffers is correct.
It shall also set the max expected buffersize.
setup the buffersize according to picture dimensions in the camera struct.
setup the buffercount to a value we think is right

@param vq(pointer) = struct videobuf_queue is the handle of the buffer queue
@param count(pointer) = pointer to suggested number of buffers
@param size(pointer) = pointer to expected maximum size for each buffer
@return 0=success
*/
static int vb_buf_setup(struct videobuf_queue *vq, unsigned int *count, unsigned int *size)
{
    struct csi_camera *pcam = vq->priv_data;

    csi_debug(2,"Video vb_buf_setup\n");

	
	*size = pcam->frame_size;

     if (*count == 0 || *count > 6) // check limits
        *count = 6;

    csi_debug(2,"%s, buffer count=%d, size=%d\n", __func__,*count, *size);

    return 0;
}



/**@brief for a given buffer, set the buffer's size, width, height, and field  properly
user called VIDIOC_QBUF. 
- vb_buf_prepare


If the buffer's state field is VIDEOBUF_NEEDS_INIT, the driver should pass it to: videobuf_iolock .
buffer's state will change to VIDEOBUF_PREPARED then.

@param  vq(pointer) = struct videobuf_queue is the handle of this queue
@param  vb(pointer) = struct videobuf_buffer is the buffer to be handled
@param  field = odd, even or both fields of a picture. we use: V4L2_FIELD_NONE (fields are not interlaced)
@return 0=succes; EINVAL=invalid parameters; else errorcode of videobuf_iolock
*/
static int vb_buf_prepare(struct videobuf_queue *vq, struct videobuf_buffer *vb, enum v4l2_field field)
{
    struct csi_camera *pcam = vq->priv_data;
    int ret;

    csi_debug(1,"vb_buf_prepare\n");


    if (pcam->width  < MIN_WIDTH || pcam->width  > MAX_WIDTH ||
            pcam->height < MIN_HEIGHT || pcam->height > MAX_HEIGHT)
    {
	csi_debug(3,"****error1 in%s\n",__func__);
        return -EINVAL;
    }

    vb->size = pcam->frame_size;

	//if bufferpointer is 0,OR buffersize < picturesize
    if (0 != vb->baddr && vb->bsize < vb->size)
    {
	csi_debug(3,"****error2 in %s baddr:%lu  bsize:%u  size:%lu\n",__func__, vb->baddr,vb->bsize,vb->size );
        return -EINVAL;
    }

    // These properties only change when queue is idle, see s_fmt. then also need to realloc memory!
    vb->width  = pcam->width;
    vb->height = pcam->height;
	vb->bytesperline = pcam->bytesperline;
	vb->size = pcam->frame_size;
    vb->field  = V4L2_FIELD_NONE; //field; // we use: V4L2_FIELD_NONE (fields are not interlaced from the camera)

    if (VIDEOBUF_NEEDS_INIT == vb->state)
    {
        ret = videobuf_iolock(vq, vb, NULL); //  calls __videobuf_iolock in dma-contic, no alloc is done!
        if (ret) return ret;
   }

    vb->boff= videobuf_to_dma_contig(vb); // get memory address of buffer
    vb->state = VIDEOBUF_PREPARED;

    return 0;

}


/**@brief queue a buffer for IO

user called VIDIOC_QBUF. 
- vb_buf_prepare
- vb_buf_queue  (put buffer on dma queue)

When a buffer is queued for I/O, it is passed to buf_queue(), which should put it onto the driver's list of available buffers 
and set its state to VIDEOBUF_QUEUED. 
Note that this function is called with the queue spinlock held!

@param  vq(pointer) = struct videobuf_queue is the handle of this queue
@param  vb(pointer) = struct videobuf_buffer is the buffer to be handled
*/
static void vb_buf_queue(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{
    struct csi_camera *pcam = vq->priv_data;
    struct list_head *dma_q = &pcam->dmaque;
		Error10++; // number of buffers queued from application

    csi_debug(1,"buffer_queued ptr=%p\n",vb);
    vb->state = VIDEOBUF_QUEUED;
	// Note: the videobuf framework already set the spin_lock when this function is called! so no need to do it again!!!
	//if (spin_trylock(&pcam->slock)) csi_debug(3,"spinlock not set!!!\n");  // to test here!
    list_add_tail(&vb->queue, dma_q); // integrate the videobuf buffer into the dma queue.  vb->queue is a list_head struct.
}


/**@brief release a buffer. mark as unused. remove from dma queue!

called in response to stream_off. or videobuf_mmap_free

Finally, buf_release() is called when a buffer is no longer intended to be used. 
The driver should ensure that there is no I/O active on the buffer, 
possibly called from:
videobuf_dma_unmap
videobuf_dma_free
videobuf_vmalloc_free
videobuf_dma_contig_free
videobuf_mmap_free

do:
set bufferstate to VIDEOBUF_NEEDS_INIT.
actually remove buffer from dma queue

our implementation: just change state and reinit dma queue in streamoff

@param  vq(pointer) = struct videobuf_queue is the handle of this queue
@param  vb(pointer) = struct videobuf_buffer is the buffer to be handled
*/
static void vb_buf_release(struct videobuf_queue *vq, struct videobuf_buffer *vb)
{

    csi_debug(2,"vb_buf_release\n");
	
	vb->state = VIDEOBUF_NEEDS_INIT;
}






/** videobuf layer operations

NOTE, that we do not use the more modern videobuf2 layer, but the older videobuf layer!

This struct contains pointers to the videobuf layer routines
for Videobuffer management.
It is used by the videobuf layer to access related driver functions.

videobuf_queue_dma_contig_init() is used to open the queue.
videobuf_to_dma_contig(buffer) is used to get memory address of buffer

videobuf_read_stream() is used to read multible frames from the queue to the application.
videobuf_streamon
videobuf_streamoff
 videobuf_iolock
*/
static struct videobuf_queue_ops csi_video_qops =
{
    .buf_setup    = vb_buf_setup,  // set max bufsize, number of buffers
    .buf_prepare  = vb_buf_prepare, //sets picture size/buffer size, sets up buffer structure.
    .buf_queue    = vb_buf_queue,  // put buffer on capture queue. NOTE: remove buffer from capture queue is done in the interrupt!
    .buf_release  = vb_buf_release,  // remove buffer from usage
};





//+++:++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ video ioctl functions

/**@brief query capabilities
The ioctl takes a pointer to a struct v4l2_capability which is filled by the driver. 
When the driver is not compatible with this specification the ioctl returns an EINVAL error code.

*/
static int vidioc_querycap(struct file *file, void  *priv,
                           struct v4l2_capability *cap)
{
    struct csi_camera *pcam = priv;
    csi_debug(2,"ioctl - vidioc_querycap=sun4i_csi\n");
    strcpy(cap->driver, "sun4i_csi");
    strcpy(cap->card, pcam->cam_name); //mod this to include the camera name!!!tomk
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
    int ret;
	struct csi_camera *pcam = priv;

    csi_debug(2,"ioctl - vidioc_enum_fmt_vid_cap index:%u \n",f->index);
	
    ret = v4l2_subdev_call(pcam->psubdev,video,enum_mbus_fmt,f->index,&f->pixelformat);

    if (ret)
    {
		csi_debug(2,"not supported\n");
        return ret; // not supported, end of list
    }

	f->description[0]=(char)(f->pixelformat);
	f->description[1]=(char)(f->pixelformat>>8);
	f->description[2]=(char)(f->pixelformat>>16);
	f->description[3]=(char)(f->pixelformat>>24);
	f->description[4]=0;
	
	csi_debug(2,"----------returned fourcc:%s\n",f->description);
    return 0;
}

/**@brief get windowsize of given index
The subdevice will lookup its windowsize at given index and return type, width, height.
*/
static int vidioc_enum_framesizes(struct file *file, void *priv, struct v4l2_frmsizeenum *fsize)
{
    int ret;
	struct csi_camera *pcam = priv;
    csi_debug(2,"ioctl - vidioc_enum_framesizes index:%u \n",fsize->index);
	
    ret = v4l2_subdev_call(pcam->psubdev,video,enum_framesizes,fsize);

    if (ret)
    {
		csi_debug(2,"invalid size index\n");// not supported, end of list
    }
        return ret; 
}


/**@brief get FPS of given windowsize at given fps-index
The subdevice will lookup the FPS at given index and return type, nominator, denominator as interval time in seconds.
*/
static int vidioc_enum_frameintervals(struct file *file, void *priv, struct v4l2_frmivalenum *fival)
{
    int ret;
	struct csi_camera *pcam = priv;
    csi_debug(2,"ioctl - vidioc_enum_frameintervals index:%u \n",fival->index);
	
    ret = v4l2_subdev_call(pcam->psubdev,video,enum_frameintervals,fival);

    if (ret)
    {
		csi_debug(2,"invalid fps index\n");// not supported, end of list
    }
        return ret; 
}


/**@brief  get the current video data format
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
NOTE: initially the camera is not set to a specific format.
The App must issue a s_fmt ioctl to activate the format in the camera!!!!
The camera has however supplied the default format in the pcam struct to be read here.
*/
static int vidioc_g_fmt_vid_cap(struct file *file, void *priv, struct v4l2_format *f)
{
    struct csi_camera *pcam = priv;
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
    struct csi_camera *pcam = priv;
    struct v4l2_mbus_framefmt mbus_fmt;
    int ret = 0;

    csi_debug(2,"ioctl - vidioc_try_fmt_vid_cap - fourcc:%c%c%c%c\n", (char)(f->fmt.pix.pixelformat), (char)(f->fmt.pix.pixelformat>>8), (char)(f->fmt.pix.pixelformat>>16), (char)(f->fmt.pix.pixelformat>>24) );
    // limit the resolution to the max values the CSI-module can support
    if(f->fmt.pix.width > MAX_WIDTH || f->fmt.pix.height > MAX_HEIGHT)
    {
        csi_debug(3,"***ERROR:size is too large,automatically set to maximum!\n");
        f->fmt.pix.width = MAX_WIDTH;
        f->fmt.pix.height = MAX_HEIGHT;
    }

    mbus_fmt.width = f->fmt.pix.width;
    mbus_fmt.height = f->fmt.pix.height;
    mbus_fmt.code = f->fmt.pix.pixelformat;
	mbus_fmt.field = (f->fmt.pix.bytesperline<<3)/f->fmt.pix.width; //bitsperpixel!
	
	csi_debug(2,"request: width:%u hight:%u bytesperline:%u framesize:%u bitsperpixel%u",f->fmt.pix.width,f->fmt.pix.height,f->fmt.pix.bytesperline, f->fmt.pix.sizeimage,mbus_fmt.field);
	
	// call camera to check the video format
	// Let the sensor code look over and tweak the requested formatting.
    ret = v4l2_subdev_call(pcam->psubdev,video,try_mbus_fmt,&mbus_fmt);
    if (ret < 0)
    {
        csi_debug(3,"***ERROR:v4l2 sub device could not set video format\n");
        return ret;
    }

    //copy the returned video format to structure to be retuned
    f->fmt.pix.width = mbus_fmt.width;
    f->fmt.pix.height = mbus_fmt.height;
	f->fmt.pix.pixelformat =  mbus_fmt.code; //fourcc
	f->fmt.pix.bytesperline = (mbus_fmt.width*mbus_fmt.field)>>3;  //tomk
	f->fmt.pix.sizeimage = mbus_fmt.height*f->fmt.pix.bytesperline;
    f->fmt.pix.field = V4L2_FIELD_NONE; 
	csi_debug(2,"matched: width:%u hight:%u bytesperline:%u framesize:%u bitsperpixel%u",f->fmt.pix.width,f->fmt.pix.height,f->fmt.pix.bytesperline, f->fmt.pix.sizeimage,mbus_fmt.field);

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
    struct csi_camera *pcam = priv;
    struct videobuf_queue *q = &pcam->vb_vidq;
    struct v4l2_mbus_framefmt mbus_fmt;
	
    int ret,width_buf,height_buf,width_len;


    csi_debug(2,"ioctl - vidioc_s_fmt_vid_cap - fourcc:%c%c%c%c\n", (char)(f->fmt.pix.pixelformat), (char)(f->fmt.pix.pixelformat>>8), (char)(f->fmt.pix.pixelformat>>16), (char)(f->fmt.pix.pixelformat>>24) );

	// first stop the capturing before we can set new format
    if (IsCapturing)
    {
        csi_debug(3,"%s device busy\n", __func__);
        return -EBUSY;
    }

    mutex_lock(&q->vb_lock); //videobuf, dont disturb

    mbus_fmt.width = f->fmt.pix.width;
    mbus_fmt.height = f->fmt.pix.height;
    mbus_fmt.code = f->fmt.pix.pixelformat;
	mbus_fmt.field = (f->fmt.pix.bytesperline<<3)/f->fmt.pix.width; //bitsperpixel!

	csi_debug(2,"request: width:%u hight:%u bytesperline:%u framesize:%u bitsperpixel%u",f->fmt.pix.width,f->fmt.pix.height,f->fmt.pix.bytesperline, f->fmt.pix.sizeimage,mbus_fmt.field);

	// call camera to set the video format
    ret = v4l2_subdev_call(pcam->psubdev,video,s_mbus_fmt,&mbus_fmt);
    if (ret < 0)
    {
        csi_debug(3,"***ERROR:v4l2 sub device s_fmt failed!\n");
        goto out;
    }
	
	// OK, the camera has been set now, its format is in the camera struct. mbus_fmt Information is updated by the camera
	
	/* lets check, if yuv422 format was selected in the camera.
	if so,  csi module can do format translation to planar buffer usage, 
	which gives bigger framesize and ? speed.
	Thus converting to a planar yuv422 format.
	
	The camera itself does not support planar formats.
	*/


    //save the current format info

    pcam->vb_vidq.field = V4L2_FIELD_NONE; // set videobuf field, we only support progressive scan, not interlaced!

	// config csi module format converter. csi_mode was set in s_fmt in Camera driver!
    bsp_csi_configure(pcam,&pcam->csi_mode);
//	tomk

	if (pcam->csi_mode.input_fmt == CSI_RAW) 
	{
    width_buf = pcam->bytesperline;  // pixelclocks per line on the input
    height_buf = pcam->height; // line per frame
    width_len  = pcam->bytesperline; // output fifo length, the max of the three fifo if planar(width), or bytesperline
	}
	else //planar
	{
	width_buf = pcam->bytesperline;
	height_buf = pcam->height;
	width_len  = pcam->width; //max of 3 fifos=Y
	}

    //horizontal and vertical offset are constant zero. pixel_clocks_perline, number_of_lines, number_of_bytesperline
	//limit the number of bytes the csi-module will send to the buffer
    bsp_csi_set_size(pcam,width_buf,height_buf,width_len);
	
	csi_debug(2,"------------csi config: input_fmt:%u output_fmt:%u field_sel:%u, seq:%u\n",pcam->csi_mode.input_fmt,pcam->csi_mode.output_fmt,pcam->csi_mode.field_sel,pcam->csi_mode.seq);
	csi_debug(2,"------------csi picsize: width:%u height:%u fifobytes:%u\n",width_buf,height_buf,width_len);
    ret = 0;
	
// update the information in the callers struct v4l2_format:
	f->fmt.pix.width = pcam->width;
	f->fmt.pix.height = pcam->height;
	f->fmt.pix.pixelformat = pcam->fourcc;
	f->fmt.pix.field = V4L2_FIELD_NONE;
	f->fmt.pix.bytesperline = pcam->bytesperline;
	f->fmt.pix.sizeimage = pcam->frame_size;
	f->fmt.pix.colorspace = 0;
	// todo: setup mplane info
out:
    mutex_unlock(&q->vb_lock);
    return ret;
}


//+++:++++++++++++++++++++++++++++++++++++++++++++++++++++++++++  video stream functions

/**@brief Initiate Memory Mapping or User Pointer I/O. Here we actually allocate all the buffer memory!
Only called once from the application!
call flow: videobuf_reqbufs->vb_buf_setup(here)->__videobuf_mmap_setup->__videobuf_free->videobuf_alloc_vb;

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
static int vidioc_reqbufs(struct file *file, void *priv,
                          struct v4l2_requestbuffers *p)
{
    struct csi_camera *pcam = priv;

    csi_debug(2,"ioctl - vidioc_reqbufs\n");

    return videobuf_reqbufs(&pcam->vb_vidq, p);
}


/**@brief Query the status of a buffer
This ioctl is part of the memory mapping I/O method. 
It can be used to query the status of a buffer at any time after buffers have been allocated 
with the VIDIOC_REQBUFS ioctl.

Applications set the type field of a struct v4l2_buffer to the same buffer type as previously struct 
v4l2_format type and struct v4l2_requestbuffers type, and the index field. 
Valid index numbers range from zero to the number of buffers allocated with VIDIOC_REQBUFS 
(struct v4l2_requestbuffers count) minus one. 
After calling VIDIOC_QUERYBUF with a pointer to this structure drivers return an error code or 
fill the rest of the structure.

In the flags field the V4L2_BUF_FLAG_MAPPED, V4L2_BUF_FLAG_QUEUED and V4L2_BUF_FLAG_DONE flags 
will be valid. 
The memory field will be set to V4L2_MEMORY_MMAP, the m.offset contains the offset of 
the buffer from the start of the device memory, the length field its size. 
The driver may or may not set the remaining fields and flags, they are meaningless in this context.
*/
static int vidioc_querybuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    struct csi_camera *pcam = priv;
    csi_debug(2,"ioctl - vidioc_querybuf\n");

    return videobuf_querybuf(&pcam->vb_vidq, p);
}


/**@brief enqueue a userbuffer with the driver
Applications call the VIDIOC_QBUF ioctl to enqueue an empty (capturing) buffer
in the driver's incoming queue. 
*/
static int vidioc_qbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{

    struct csi_camera *pcam = priv;
    csi_debug(1,"ioctl - vidioc_qbuf  ptr=%p\n",p);
		Error6++; //qbuf, should match Error10
 
    return videobuf_qbuf(&pcam->vb_vidq, p);
}


/**@brief  dequeue a filled (capturing)  userbuffer from the driver's outgoing queue
*/
static int vidioc_dqbuf(struct file *file, void *priv, struct v4l2_buffer *p)
{
    struct csi_camera *pcam = priv;
    csi_debug(1,"ioctl - vidioc_dqbuf\n");
		Error7++; //dque

    return videobuf_dqbuf(&pcam->vb_vidq, p, file->f_flags & O_NONBLOCK);
}



/**@brief  Start streaming I/O
The VIDIOC_STREAMON ioctl starts the capture process
of streaming (memory mapping or user pointer) I/O.
*/
static int vidioc_streamon(struct file *file, void *priv, enum v4l2_buf_type i)
{
    struct csi_camera *pcam = priv;
    struct list_head *dma_q = &pcam->dmaque;
    struct videobuf_buffer *buf;

    int ret;
    csi_debug(2,"ioctl - vidioc_streamon\n");
    csi_debug(2,"turn video stream on\n");
    if (i != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
        return -EINVAL;
    }

    if (IsCapturing)
    {
        csi_debug(3,"***ERROR:error stream has been already on\n");
        return 0;
    }


    ret = videobuf_streamon(&pcam->vb_vidq); // tell videobuf layer to turn on stream
    if (ret)
    {
        return ret;
    }

	if (list_empty(dma_q)) 
	{
	csi_debug(2,"dma-queue empty on start\n");
	 return -EINVAL;
	}
    ret = v4l2_subdev_call(pcam->psubdev,core, s_power, CSI_SUBDEV_STBY_OFF);
    if (ret!=0)
    {
        csi_debug(3,"***ERROR:sensor standby off error when streamon!\n");
        return ret;
    }
	
	// get first buffer dma queue and set its address for first csi dma and interrupt.
    buf = list_entry(dma_q->next,struct videobuf_buffer, queue);
    csi_set_addr(pcam,buf);

    bsp_csi_int_clear_status(pcam,CSI_INT_FRAME_DONE);// clear csi int status
    bsp_csi_int_enable(pcam, CSI_INT_FRAME_DONE);//enable capture interrupts
    bsp_csi_capture_video_start(pcam);  //start capture

    IsCapturing = 1;
    return 0;
}



/**@brief  stop streaming I/O
The VIDIOC_STREAMOFF ioctl stops the capture process
of streaming (memory mapping or user pointer) I/O.
*/
static int vidioc_streamoff(struct file *file, void *priv, enum v4l2_buf_type i)
{
    struct csi_camera *pcam = priv;
    int ret;

    csi_debug(2,"ioctl - vidioc_streamoff\n");

    if (!IsCapturing)
    {
        csi_debug(3,"***ERROR:error stream has been already off\n");
        return 0;
    }

    IsCapturing = 0;


    bsp_csi_int_disable(pcam,CSI_INT_FRAME_DONE);//disable csi int
    bsp_csi_int_clear_status(pcam,CSI_INT_FRAME_DONE);//clear int status
    bsp_csi_capture_video_stop(pcam); // stop capture
	
        ret = v4l2_subdev_call(pcam->psubdev,core, s_power, CSI_SUBDEV_STBY_ON); //put camera to standby
        if (ret!=0)
        {
            csi_debug(3,"***ERROR:sensor CSI_SUBDEV_STBY_ON error when streamoff!\n");
        }
  
	

    ret = videobuf_streamoff(&pcam->vb_vidq);
    if (ret!=0)
    {
        csi_debug(3,"***ERROR:videobu_streamoff error!\n");
        return ret;
    }

	spin_lock_irq(&pcam->slock);
   INIT_LIST_HEAD(&pcam->dmaque); // init the dma buffer queue, so its empty for next quebufs
	spin_unlock_irq(&pcam->slock);

    return 0;
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
    struct csi_camera *pcam = priv;
    csi_debug(2,"ioctl - vidioc_enum_input\n");
    if (inp->index > pcam->dev_qty-1)
    {
        csi_debug(3,"***ERROR:input index invalid! %d\n",inp->index);
        return -EINVAL;
    }

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
    struct csi_camera *pcam = priv;
    int ret;
    csi_debug(2,"ioctl - vidioc_queryctrl:%u\n",qc->id);
    ret = v4l2_subdev_call(pcam->psubdev,core,queryctrl,qc);

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
    struct csi_camera *pcam = priv;
    int ret;
    csi_debug(2,"ioctl - vidioc_g_ctrl\n");
    ret = v4l2_subdev_call(pcam->psubdev,core,g_ctrl,ctrl);

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
    struct csi_camera *pcam = priv;
    struct v4l2_queryctrl qc;
    int ret;
    csi_debug(2,"ioctl - vidioc_s_ctrl\n");
    qc.id = ctrl->id;
    ret = vidioc_queryctrl(file, priv, &qc);
    if (ret < 0)
    {
        return ret;
    }

    if (ctrl->value < qc.minimum || ctrl->value > qc.maximum)
    {
        return -ERANGE;
    }

    ret = v4l2_subdev_call(pcam->psubdev,core,s_ctrl,ctrl);
 
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
    struct csi_camera *pcam = priv;
    int ret;
    csi_debug(2,"ioctl - vidioc_g_parm\n");
    ret = v4l2_subdev_call(pcam->psubdev,video,g_parm,parms);
 
    return ret;
}


/**@brief set frame rate
see above
*/
static int vidioc_s_parm(struct file *file, void *priv,
                         struct v4l2_streamparm *parms)
{
    struct csi_camera *pcam = priv;
    int ret;
    csi_debug(2,"ioctl - vidioc_s_parm\n");
	//arguments: v4l2_subdev_call(psubdevice, operation_structure, function, args..) 
    ret = v4l2_subdev_call(pcam->psubdev,video,s_parm,parms);
   
    return ret;
}



/**@brief read multible frames (count) from the videobuffer queue to the application
NOTE: general possible IO Methods:
- read
- mmap
- userpointer
Read file I/O
we only support mmap !!
Read needs to start and stop the engine to capture one buffer. Tomk
*/
static ssize_t csi_read(struct file *file, char __user *data, size_t count, loff_t *ppos)
{
    struct csi_camera *pcam = video_drvdata(file);

		Error8++; //read

    if(IsCapturing)
    {
        return videobuf_read_stream(&pcam->vb_vidq, data, count, ppos, 0,
                                    file->f_flags & O_NONBLOCK);
    }
    else
    {
        csi_debug(3,"***ERROR:read: csi is not generating error!\n");
        return -EINVAL;
    }
}


/**@brief wait for completion of a capture. called from user app
*/
static unsigned int csi_poll(struct file *file, struct poll_table_struct *wait)
{
    struct csi_camera *pcam = video_drvdata(file);
    struct videobuf_queue *q = &pcam->vb_vidq;
		Error9++; //poll


    if(IsCapturing)
    {
        return videobuf_poll_stream(file, q, wait);
    }
    else
    {
        csi_debug(2,"***ERROR:poll: csi is not generating error!\n");
        return -EINVAL;
    }
}




//+++:+++++++++++++++++++++++++++++++++++++++++++++++++   driver service functions


/**@brief File operations, open. someone starts using the camera!

open the csi module
*/
static int csi_open(struct file *file)
{
    struct csi_camera *pcam = video_drvdata(file);
    int ret;
 

    csi_debug(3,"- csi_open\n");
	Error1=Error2=Error3=Error4=Error5=Error6=Error7=Error8=Error9=Error10=Error11=0;
    if (pcam->opened == 1)
    {
        csi_debug(3,"***ERROR:device open called more than once!\n");
        return -EBUSY;
    }
	// save the pointer to our camera datastructure in the kernels file structure.
	// it will be given as the priv pointer in later ioctl calls
	file->private_data = pcam;

    csi_clk_enable(pcam); // open


        pcam->csi_mode.vref       = pcam->csi_sig_cfg.vref; // config cameras active polaritys
        pcam->csi_mode.href       = pcam->csi_sig_cfg.href;
        pcam->csi_mode.clock      = pcam->csi_sig_cfg.clock;
        csi_clk_out_set(pcam);  //configure the output clock to the camera

		//power on the camera to standby mode
        ret = v4l2_subdev_call(pcam->psubdev,core, s_power, CSI_SUBDEV_PWR_ON); //csi_open. power up the camera 
        if (ret!=0)
        {
            csi_debug(3,"***ERROR:sensor CSI_SUBDEV_PWR_ON error when csi open!\n");
        }


    pcam->input=0;//default v4l2-input, we only have one!

    bsp_csi_open(pcam); // start the csi module
    bsp_csi_set_offset(pcam,0,0);//set h and v offset to zero for full picture


	// init the camera and driver
    ret = v4l2_subdev_call(pcam->psubdev,core, init, 0); // just adds the i2c driver
    if (ret!=0)
    {
        csi_debug(3,"***ERROR:sensor initial error when camera init!\n");
        return ret;
    }
    else
    {
        csi_debug(2,"sensor initial success when csi open!\n");
    }

    //initialize the videobuf queue
    videobuf_queue_dma_contig_init(&pcam->vb_vidq, &csi_video_qops,
                                   NULL, &pcam->slock, V4L2_BUF_TYPE_VIDEO_CAPTURE,
                                   V4L2_FIELD_NONE,//default format, can be changed by s_fmt
                                   sizeof(struct videobuf_buffer), pcam,NULL);

    INIT_LIST_HEAD(&pcam->dmaque); // init the dma buffer queue

    pcam->opened = 1;
    return 0;
}

/** file operations, close
*/

/**@brief file operations->release
*/
static int csi_close(struct file *file)
{
    struct csi_camera *pcam = video_drvdata(file);
    int ret;

    csi_debug(2,"- csi_close\n");
	csi_debug(3,"++Close++++++++Error1:%lu Error2:%lu Error3:%lu Error4:%lu Error5:%lu Error6:%lu Error7:%lu Error8:%lu Error9:%lu Error10:%lu Error11:%lu\n",Error1,Error2,Error3,Error4,Error5,Error6,Error7,Error8,Error9,Error10,Error11);

    bsp_csi_int_disable(pcam,CSI_INT_FRAME_DONE);//CSI_INT_FRAME_DONE
    //bsp_csi_int_clear_status(pcam,CSI_INT_FRAME_DONE);

    bsp_csi_capture_video_stop(pcam);
    bsp_csi_close(pcam);

    csi_clk_disable(pcam);  //close


    videobuf_stop(&pcam->vb_vidq); // stop streaming or reading if on
	
	// possible alternate function to use: videobuf_vm_close which calls sunxi_buf_free
    ret=videobuf_mmap_free(&pcam->vb_vidq); //alloc memory free: check if buffer is not mapped,calls vb_buf_release, and free its memory.
	
    if (ret!=0)
    {
        csi_debug(3,"***ERROR:videobuf_mmap_free error!\n");
        return ret;
    }

    pcam->opened=0;
    IsCapturing = 0;
	
	//tomk
            ret = v4l2_subdev_call(pcam->psubdev,core, s_power, CSI_SUBDEV_PWR_OFF); // turn camera power off:csi_close
            if (ret!=0)
            {
                csi_debug(3,"***ERROR:failed to turn off power when close\n");
                return ret;
            }
	

    return 0;
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

    ret = videobuf_mmap_mapper(&pcam->vb_vidq, vma); // alloc memory: this function will call "sunxi_buf_alloc(" in dma-contig, so allocates the memory for buffer, and maps it

	if (ret) csi_debug(3,"CSI_mmap failed to alloc buffers!\n");
	
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
	.vidioc_enum_framesizes   = vidioc_enum_framesizes,     // return windowsize of given index
	.vidioc_enum_frameintervals = vidioc_enum_frameintervals, // return fps of wanted windowsize
    .vidioc_g_fmt_vid_cap     = vidioc_g_fmt_vid_cap,		// get current active video format
    .vidioc_try_fmt_vid_cap   = vidioc_try_fmt_vid_cap,		// try video format. try your best matching format, most used
    .vidioc_s_fmt_vid_cap     = vidioc_s_fmt_vid_cap,		// set a given video format
    .vidioc_reqbufs           = vidioc_reqbufs,				// Initiate Memory Mapping or User Pointer I/O
    .vidioc_querybuf          = vidioc_querybuf,			// Query the status of a buffer
    .vidioc_qbuf              = vidioc_qbuf,				// enqueue a free user buffer to driverqueue
    .vidioc_dqbuf             = vidioc_dqbuf,				// dequeue a used buffer from driverqueue
    .vidioc_enum_input        = vidioc_enum_input,			// return attributes of a given video input
    .vidioc_g_input           = vidioc_g_input,				// Query the current video input number
    .vidioc_s_input           = vidioc_s_input,				// select the current video input number
    .vidioc_streamon          = vidioc_streamon,			// start videostream
    .vidioc_streamoff         = vidioc_streamoff,			// stop videostream
    .vidioc_queryctrl         = vidioc_queryctrl,			// return the attributes of a given control
    .vidioc_g_ctrl            = vidioc_g_ctrl,				// get controlvalue of given control
    .vidioc_s_ctrl            = vidioc_s_ctrl,				// set controlvalue of given control
    .vidioc_g_parm		 	  = vidioc_g_parm,				// get current streaming parameters
    .vidioc_s_parm		  	  = vidioc_s_parm,				// set streaming parameters ie. frame skipping, number of buffers

};



// device file operations  fops
static const struct v4l2_file_operations csi_fops =
{
    .owner	  = THIS_MODULE,
    .open	  = csi_open,
    .release  = csi_close,
    .read     = csi_read,
    .poll	  = csi_poll,
    .ioctl    = video_ioctl2,
    .mmap     = csi_mmap,
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


// parameter: struct csi_camera = in sun4i_csi_core.h = various operating parameters of the camers

/**@brief 
*/
static int fetch_csi_config(struct csi_camera *pcam)
{
    int ret;
    csi_debug(2,"fetch csi bus config from fex\n");
    /* fetch device quantity issue. This is configured in the script.bin / fex of the A20

    see: tools/bananapi/camera/A10 Camera Sensor equipment drive development (2012-01-30)(1).pdf

    On bananaPi Board, we only have CSI0 port, but there are seperate csi Reset and Standby_en(output enable) provided ! ?

    if more than 1 camera is selected (1) in fex, then for each camera there needs to be a config
    (note: 2 cameras may share one csi port! ie. mulitplexing!)

    fex parameters:
    csi_dev_qty         = 2  // number of configured camera modules connected to this csi port. max 2!.
                           ee. The quantity of devices linked to csi interface
    csi_stby_mode       = 0  // standy power status: 0=power is on at standby 1=power is off at standby

    csi_mname           = "gc0308"   // name of camera (and the module.ko file)
    csi_twi_id          = 1  //i2C port. TWI = Two Wire Interface
    csi_twi_addr        = 0x42  //i"c address of camera
    csi_if              = 0  // interface bus-width. 0=8bit
    csi_vflip           = 0     flip in vertical direction 0:disable 1:enable
    csi_hflip           = 0     flip in horizontal direction 0:disable 1:enable
    csi_iovdd           = "axp20_pll" //axp209 powersupply controller chip for the camera module, needs to specify csi_vol_iovdd for desired voltage
                       //see schematic of board for details which ldo controller is used. or none.
    				    camera module io power , pmu power supply
    csi_avdd            = ""  // analog powersupply source  "" = default. camera module analog power , pmu power supply
    csi_dvdd            = ""  // digital powersupply source ""=default. camera module core power , pmu power supply
    csi_vol_iovdd       = 2800  //2800 mV   VOL = voltage in mV
    csi_vol_dvdd        =
    csi_vol_avdd        =
    csi_flash_pol       = 0   // the active polarity of the flash light IO. 0=low active 1=high active

    csi_mname_b         = "gt2005"
    csi_twi_id_b        = 1
    csi_twi_addr_b      = 0x78
    csi_if_b            = 0
    csi_vflip_b         = 0
    csi_hflip_b         = 0
    csi_iovdd_b         = "axp20_pll"
    csi_avdd_b          = ""
    csi_dvdd_b          = ""
    csi_vol_iovdd_b     = 2800
    csi_vol_avdd_b      =
    csi_vol_dvdd_b      =
    csi_flash_pol_b     = 0
    */


    // get number of configured cameras
#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_dev_qty", &pcam->dev_qty , sizeof(int));
#else
    ret = script_parser_fetch("csi1_para","csi_dev_qty", &pcam->dev_qty , sizeof(int));
#endif
    if (ret)
    {
        ret=1;
        goto script_err;
    }
    // only one camera must be configured
    if(pcam->dev_qty != 1)
    {
        csi_debug(3,"Error: no camera device is specified in fex file or more than 1");
        return (-1);
    }

	
	
// fetch camera modul parameters from script.bin

    // get camera name
#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_mname", (int *)&pcam->cam_name , I2C_NAME_SIZE*sizeof(char));
#else
    ret = script_parser_fetch("csi1_para","csi_mname", (int *)&pcam->cam_name , I2C_NAME_SIZE*sizeof(char));
#endif
    if (ret)
    {
        ret=2;
        goto script_err;
    }

    // get i2c port number
#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_twi_id", &pcam->twi_id , sizeof(int));
#else
    ret = script_parser_fetch("csi1_para","csi_twi_id", &pcam->twi_id , sizeof(int));
#endif
    if (ret)
    {
        ret=3;
        goto script_err;
    }
    /* fetch standby mode */
#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_stby_mode", &pcam->stby_mode , sizeof(int));
#else
    ret = script_parser_fetch("csi1_para","csi_stby_mode", &pcam->stby_mode , sizeof(int));
#endif
    if (ret)
    {
        ret=4;
        goto script_err;
    }

    // get i2c port number
#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_twi_addr", &pcam->i2c_addr , sizeof(int));
#else
    ret = script_parser_fetch("csi1_para","csi_twi_addr", &pcam->i2c_addr , sizeof(int));
#endif
    if (ret)
    {
        ret=5;
        goto script_err;
    }


    /* fetch interface issue*/
#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_if", &pcam->interface , sizeof(int));
#else
    ret = script_parser_fetch("csi1_para","csi_if", &pcam->interface , sizeof(int));
#endif
    if (ret)
    {
        ret=6;
        goto script_err;
    }

    /* fetch power issue*/

#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_iovdd", (int *)&pcam->iovdd_str , 32*sizeof(char));
#else
    ret = script_parser_fetch("csi1_para","csi_iovdd", (int *)&pcam->iovdd_str , 32*sizeof(char));
#endif
    if (ret)
    {
        ret=7;
        goto script_err;
    }

#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_avdd", (int *)&pcam->avdd_str , 32*sizeof(char));
#else
    ret = script_parser_fetch("csi1_para","csi_avdd", (int *)&pcam->avdd_str , 32*sizeof(char));
#endif
    if (ret)
    {
        ret=8;
        goto script_err;
    }

#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_dvdd", (int *)&pcam->dvdd_str , 32*sizeof(char));
#else
    ret = script_parser_fetch("csi1_para","csi_dvdd", (int *)&pcam->dvdd_str , 32*sizeof(char));
#endif
    if (ret)
    {
        ret=9;
        goto script_err;
    }

    /* fetch flip issue */
#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_vflip", &pcam->vflip , sizeof(int));
#else
    ret = script_parser_fetch("csi1_para","csi_vflip", &pcam->vflip , sizeof(int));
#endif
    if (ret)
    {
        ret=10;
        goto script_err;
    }

#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_hflip", &pcam->hflip , sizeof(int));
#else
    ret = script_parser_fetch("csi1_para","csi_hflip", &pcam->hflip , sizeof(int));
#endif
    if (ret)
    {
        ret=11;
        goto script_err;
    }

    /* fetch flash light issue */
#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_flash_pol", &pcam->flash_pol , sizeof(int));
#else
    ret = script_parser_fetch("csi1_para","csi_flash_pol", &pcam->flash_pol , sizeof(int));
#endif
    if (ret)
    {
        ret=12;
        goto script_err;
    }


    // check, if module parameters have been inputted
    if((i2c_addr != 0) && (cam_name[0]!=0))
	{
        //get module parameters for cam module name, and i2c_addr
        pcam->i2c_addr = i2c_addr; 
		strcpy(pcam->cam_name,cam_name); 
		// TODO: fetch the correct csi-port and camera parameters from the stated camera driver.
	}

// print captured camera config parameters:
    csi_debug(2,"Camera: %s\n",pcam->cam_name);
    csi_debug(2,"twi_id = %x\n",pcam->twi_id);
    csi_debug(2,"i2c_addr = %x\n",pcam->i2c_addr);
    csi_debug(2,"interface = %x\n",pcam->interface);
    csi_debug(2,"vflip = %x\n",pcam->vflip);
    csi_debug(2,"hflip = %x\n",pcam->hflip);
    csi_debug(2,"iovdd_str = %s\n",pcam->iovdd_str);
    csi_debug(2,"avdd_str = %s\n",pcam->avdd_str);
    csi_debug(2,"dvdd_str = %s\n",pcam->dvdd_str);
    csi_debug(2,"flash_pol = %x\n---fetch config end---\n",pcam->flash_pol);


    return 0;

script_err:
    csi_debug(3,"***ERROR %d:fetch  from script.bin failed\n",ret);
    return (-1);

}


/*
This probe function starts the device initialization:
initializing hardware, allocating resources, and registering the device with the kernel as a v4l2 device.
platform devcice probe function!
*/

/**@brief 
*/
static int csi_probe(struct platform_device *pdev)
{
    struct csi_camera *pcam;
    struct resource *res;
    struct video_device *pviddev;
    struct i2c_adapter *i2c_adap;
    int ret = 0;
    __u32 fourcc, index;

    csi_debug(2,"------------------- csi_probe start>>\n");

    // alloc memory for my big csi_camera structure containing all cam-info. this is we dynamicly allocate a structure instance.(to save kernel/module size)
    pcam = kzalloc(sizeof(struct csi_camera), GFP_KERNEL);
    if (!pcam)
    {
        csi_debug(3,"***ERROR:request csi_camera mem failed!\n");
        return -ENOMEM;
    }
	csi_debug(2,"sizeof struct csi_camera:%d\n",sizeof(struct csi_camera));
	
	
	
    pcam->id = pdev->id; // save the platform_device id = csi port 0 or 1
    pcam->pdev = pdev;  // and the address of the platform_device strucutre.

    spin_lock_init(&pcam->slock);  //init/create a locking mechanism

    // get  pointer to csi resource-struct (the memory addresses of csi register)
    res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
    if (!res)
    {
        csi_debug(3,"***ERROR:failed to find the registers\n");
        ret = -ENOENT;
        goto err_info;
    }

    //reserve the csi-register memory region so i can use it.
    pcam->regs_res = request_mem_region(res->start, resource_size(res), dev_name(&pdev->dev));

    if (!pcam->regs_res)
    {
        csi_debug(3,"***ERROR:failed to obtain register region\n");
        ret = -ENOENT;
        goto err_info;
    }

	/*ioremap() function is used to map the physical addres of an I/O device to the kernel virtual address.
	Kernel creates a page table i.e mapping of virtual address to the physical address requested.
	When we do iounmap() this mapping is destroyed.
	*/
    // map the csi-register addresspace to ?
    pcam->regs = ioremap(res->start, resource_size(res));
    if (!pcam->regs)
    {
        csi_debug(3,"***ERROR:failed to map registers\n");
        ret = -ENXIO;
        goto err_req_region;
    }


    /*get irq resource*/

    //get pointer to the irq resource struct
    res = platform_get_resource(pdev, IORESOURCE_IRQ, 0);
    if (!res)
    {
        csi_debug(3,"***ERROR:failed to get IRQ resource\n");
        ret = -ENXIO;
        goto err_regs_unmap;
    }

    pcam->irq = res->start;

    //install an interrupt
    ret = request_irq(pcam->irq, csi_isr, 0, pdev->name, pcam);
    if (ret)
    {
        csi_debug(3,"***ERROR:failed to install irq (%d)\n", ret);
        goto err_clk;
    }

    /*pin resource*/
#ifdef CSIPORT0
    pcam->csi_pin_hd = gpio_request_ex("csi0_para",NULL);  // allocate iopins
#else
    pcam->csi_pin_hd = gpio_request_ex("csi1_para",NULL);  // allocate iopins
#endif
    if (pcam->csi_pin_hd==-1)
    {
        csi_debug(3,"***ERROR:csi pin request error!\n");
        ret = -ENXIO;
        goto err_irq;
    }

    /* v4l2 device register */
    // register v4l2_device struct. while registered device is in use.
    ret = v4l2_device_register(&pdev->dev, &pcam->v4l2_dev);
    if (ret)
    {
        csi_debug(3,"***ERROR:Error registering v4l2 device\n");
        goto err_irq;

    }
	
csi_debug(2,"v4l2_device_registered\n");

    dev_set_drvdata(&(pdev)->dev, (pcam));  // integrate in media platform ?! do this before register device??

    // fetch camera config from script.bin (or maybe from the camera module??)tomk
    ret = fetch_csi_config(pcam);
    if (ret)
    {
        csi_debug(3,"***ERROR:Error at fetch_csi_config\n");
        goto err_irq;
    }

    // v4l2 subdev register, the camera module specified in script.bin,	like ov5640.c. subdev-module will be loaded,else error!

        i2c_adap = i2c_get_adapter(pcam->twi_id);  //in i2c-core.c

        if (i2c_adap == NULL)
        {
            csi_debug(3,"***ERROR:request i2c adapter failed\n");
            ret = -EINVAL;
            goto free_dev;
        }

		// allocate memory for v4l2_subdev device struct-pointer
        pcam->psubdev = kmalloc(sizeof(struct v4l2_subdev *),GFP_KERNEL);
        if (pcam->psubdev == NULL)
        {
            csi_debug(3,"***ERROR:unable to allocate memory for subdevice pointers\n");
            ret = -ENOMEM;
            goto free_dev;
        }

        dev_sensor.addr = (unsigned short)(pcam->i2c_addr>>1);
        strcpy(dev_sensor.type,pcam->cam_name);

        //Load an i2c sub-device: in v4l2-common.c, returns a pointer to the subdev device struct.
        // register the camera-driver as a subdevice
        pcam->psubdev = v4l2_i2c_new_subdev_board(&pcam->v4l2_dev,
                                              i2c_adap,
                                              &dev_sensor,
                                              NULL);

        if (!pcam->psubdev)
        {
            csi_debug(3,"***ERROR:Error registering v4l2 subdevice\n");
            goto free_dev;
        }
        else
        {
            csi_debug(2,"registered sub device\n");
        }

        // config default camera interface settings
        pcam->csi_sig_cfg.mclk = CSI_OUT_RATE;
        pcam->csi_sig_cfg.vref = CSI_LOW;
        pcam->csi_sig_cfg.href = CSI_LOW;
        pcam->csi_sig_cfg.clock = CSI_FALLING;
		
		// set default videoformat: 
		index=0;
    ret = v4l2_subdev_call(pcam->psubdev,video,enum_mbus_fmt,index,&fourcc);
	if (ret) csi_debug(3,"***error-could not fetch fourcc\n");
		pcam->fourcc=fourcc;
		pcam->width = 640;
		pcam->height=480;

		// query cameras  port settings
        ret = v4l2_subdev_call(pcam->psubdev,core,ioctl,CSI_SUBDEV_CMD_GET_INFO,&pcam->csi_sig_cfg); 
        if (ret < 0)
        {
            csi_debug(3,"***ERROR:Error when get camera info use default!\n");
        }

#ifdef CSIPORT0
        pcam->csi_sig_cfg.csi_port = 0; // set this drivers csi port
#else
        pcam->csi_sig_cfg.csi_port = 1; // set this drivers csi port
#endif
		// update cameras port settings
        ret = v4l2_subdev_call(pcam->psubdev,core,ioctl,CSI_SUBDEV_CMD_SET_INFO,&pcam->csi_sig_cfg); 
        if (ret < 0)
        {
            csi_debug(3,"***ERROR:Error when set camera info use default!\n");
        }

        // set camera powersupply options
        pcam->iovdd = NULL;
        pcam->avdd = NULL;
        pcam->dvdd = NULL;

        if(strcmp(pcam->iovdd_str,""))
        {
            pcam->iovdd = regulator_get(NULL, pcam->iovdd_str);
            if (pcam->iovdd == NULL)
            {
                csi_debug(3,"***ERROR:get regulator csi_iovdd error!\n");
                goto free_dev;
            }
        }

        if(strcmp(pcam->avdd_str,""))
        {
            pcam->avdd = regulator_get(NULL, pcam->avdd_str);
            if (pcam->avdd == NULL)
            {
                csi_debug(3,"***ERROR:get regulator csi_avdd error!\n");
                goto free_dev;
            }
        }

        if(strcmp(pcam->dvdd_str,""))
        {
            pcam->dvdd = regulator_get(NULL, pcam->dvdd_str);
            if (pcam->dvdd == NULL)
            {
                csi_debug(3,"***ERROR:get regulator csi_dvdd error!\n");
                goto free_dev;
            }
        }









        csi_debug(1,"pcam->psubdev = %p\n",pcam->psubdev);
        csi_debug(1,"pcam->csi_sig_cfg = %p\n",&pcam->csi_sig_cfg);
        csi_debug(1,"pcam->csi_sig_cfg.csi_port = %d\n",pcam->csi_sig_cfg.csi_port);
        csi_debug(1,"pcam->csi_sig_cfg.vref = %d\n",pcam->csi_sig_cfg.vref);
        csi_debug(1,"pcam->csi_sig_cfg.href = %d\n",pcam->csi_sig_cfg.href);
        csi_debug(1,"pcam->csi_sig_cfg.clock = %d\n",pcam->csi_sig_cfg.clock);
        csi_debug(1,"pcam->csi_sig_cfg.mclk = %d\n",pcam->csi_sig_cfg.mclk);
        csi_debug(1,"pcam->iovdd = %p\n",pcam->iovdd);
        csi_debug(1,"pcam->avdd = %p\n",pcam->avdd);
        csi_debug(1,"pcam->dvdd = %p\n",pcam->dvdd);
    


    /*clock resource*/
    if (csi_clk_get(pcam))
    {
        csi_debug(3,"***ERROR:csi clock get failed!\n");
        ret = -ENXIO;
        goto unreg_dev;
    }

    csi_debug(1,"%s(): csi-%d registered successfully\n",__func__, pcam->id);

    /*video device register	*/
    ret = -ENOMEM;
    pviddev = video_device_alloc(); // allocate memory for video_device struct
    if (!pviddev)
    {
        goto err_clk;
    }

    *pviddev = csi_template;  //init with default values CSI
    pviddev->v4l2_dev = &pcam->v4l2_dev;

    dev_set_name(&pviddev->dev, "csi-0"); // set specific CSI-Port
    ret = video_register_device(pviddev, VFL_TYPE_GRABBER, video_nr); // register video device
    if (ret < 0)
    {
        goto rel_vdev;
    }
	
	/* now we have kernel allocated memory for our driver structures "csi_camera" and "video_device".
	the addresses are now given to v4l2 kernel to be used in subsequent calls into this driver.
	video_set_drvdata() does that.
	We can later retrieve the pointers from the kernel with the function video_drvdata().
	This is used throughout this driver code!
	*/
	video_set_drvdata(pviddev, pcam); // register private data pointers
	
csi_debug(2,"video_register_deviceed\n");

     pcam->pviddev = pviddev; //add video device struct pointer

    csi_debug(2,"V4L2 device registered as %s\n",video_device_node_name(pviddev));


    /* init list video dma queue */
//    INIT_LIST_HEAD(&pcam->dmaque);
    csi_debug(2,"--------------------------------probe-end successfull <<<<\n");
    return 0;

rel_vdev:
    video_device_release(pviddev);
err_clk:
    csi_clk_release(pcam);
unreg_dev:
    v4l2_device_unregister(&pcam->v4l2_dev);
free_dev:
//	kfree(pcam);
err_irq:
    free_irq(pcam->irq, pcam);
err_regs_unmap:
    iounmap(pcam->regs);
err_req_region:
    release_resource(pcam->regs_res);
    kfree(pcam->regs_res);
err_info:
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

    csi_debug(2,"- csi_remove all driver/device resources\n");
	
	if (pcam == NULL)
	csi_debug(3,"get_drvdata returned null!!\n");


        v4l2_info(&pcam->v4l2_dev, "unregistering %s\n", video_device_node_name(pcam->pviddev));
        video_unregister_device(pcam->pviddev);  //unregister video device. does not return anything!
        csi_clk_release(pcam);
        v4l2_device_unregister(&pcam->v4l2_dev); //unregister linux device. does not return anything!
		csi_debug(2,"v4l2 and video device unregistered!\n");
        free_irq(pcam->irq, pcam);
        iounmap(pcam->regs);
        release_resource(pcam->regs_res);
        kfree(pcam->regs_res);
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
    struct csi_camera *pcam=(struct csi_camera *)dev_get_drvdata(&(pdev)->dev);


    csi_debug(2,"- csi_suspend\n");

    if (pcam->opened==1)
    {

		//tomk
            csi_debug(2,"set camera to standby!\n");
            return v4l2_subdev_call(pcam->psubdev,core, s_power, CSI_SUBDEV_STBY_ON);
		
	
    }
    return 0;
}

//platform device resume

/**@brief called by power management, standby off 
usually never called.
used, if the system goes to a low power state.
*/
static int csi_resume(struct platform_device *pdev)
{
    int ret;
    struct csi_camera *pcam=(struct csi_camera *)dev_get_drvdata(&(pdev)->dev);

    csi_debug(2,"- csi_resume\n");

    if (pcam->opened==1)
    {
        csi_clk_out_set(pcam);
		
           ret = v4l2_subdev_call(pcam->psubdev,core, s_power,CSI_SUBDEV_STBY_OFF);


    }

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
    .driver = {
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
    int csi_used;
    csi_debug(2,"Welcome to CSI driver\n");
    csi_debug(2,"- csi_init\n");

    /*
    int script_parser_fetch(char *main_name, char *sub_name, int value[], int count);
    this function reads config-settings from the script.bin file fex-file.
    */
#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_used", &csi_used , sizeof(int)); //check, if csi0 is enabled
#else
    ret = script_parser_fetch("csi1_para","csi_used", &csi_used , sizeof(int)); //check, if csi0 is enabled
#endif
    if (ret)
    {
        csi_debug(3,"***ERROR in %s Line:%d\n",__func__,__LINE__);
        return -1;
    }

    if(!csi_used)
    {
        csi_debug(3,"***ERROR:csi_used=0,csi driver is not enabled in fex!\n");
        return 0;
    }

    // driver registration with a list of functions and a list of devices this driver can handle. here, its only one device, the sun4i_csi
    ret = platform_driver_register(&csi_driver);  // register my driver with the kernel and give the kernel a list of devices (one) this driver is able to service,

    if (ret)
    {
        csi_debug(3,"***ERROR:platform driver register failed\n");
        return -1;
    }

    ret = platform_device_register(&csi_device); // register the above mentioned device sun4i_csi with the kernel.
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
    int csi_used,ret;

    csi_debug(2,"- csi_exit\n");

#ifdef CSIPORT0
    ret = script_parser_fetch("csi0_para","csi_used", &csi_used , sizeof(int));
#else
    ret = script_parser_fetch("csi1_para","csi_used", &csi_used , sizeof(int));
#endif
    if (ret)
    {
        csi_debug(3,"***ERROR:fetch csi_used from script.bin failed\n");
        return;
    }

    if(csi_used)
    {
	// unregister platform device csi
        platform_device_unregister(&csi_device); 
        platform_driver_unregister(&csi_driver); 
    }
}
/** Overview v4l2 call interface:
init/exit = register/unregister platform driver and platform device.
probe = register v4l2-device, video-device
remove = unregister v4l2-device, video-device
*/
module_init(csi_init);  // is called first, when module is loaded, modprobe  : sun4i_csi0 : sequence: csi_init, csi_probe, csi_open, csi_close
module_exit(csi_exit);  // is called when module is removed ,rmmod. sequence: csi_exit, csi_remove, dev_release

MODULE_AUTHOR("Thomas Krueger, Hofgeismar, Germany");
MODULE_LICENSE("Dual BSD/GPL");
MODULE_DESCRIPTION("V1-Allwinner A20 CSI Video Capture Driver for Sunxi-Kernel-3.4xx");
