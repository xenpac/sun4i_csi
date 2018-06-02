/*
 * drivers/media/video/sun4i_csi/include/sun4i_csi_core.h
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
 */

/*
 * Sun4i Camera core header file
 * Author:raymonxiu
*/
#ifndef _SUN4I_CSI_CORE_H_
#define _SUN4I_CSI_CORE_H_

#include <linux/types.h>
#include <media/videobuf-core.h>
#include <media/videobuf2-core.h>
#include <media/v4l2-device.h>
#include <linux/videodev2.h>
#include <linux/i2c.h>
#include <media/v4l2-mediabus.h>

//for internel driver debug
#define DBG_EN   		0
//debug level 0~3
#define DBG_LEVEL 	3

//for internel driver debug
#if(DBG_EN==1)
#define csi_dbg(l,x,arg...) if(l <= DBG_LEVEL) printk("[CSI_DEBUG]"x,##arg)
#else
#define csi_dbg(l,x,arg...)
#endif

//print when error happens
#define csi_err(x,arg...) printk(KERN_INFO"[CSI_ERR]"x,##arg)

//print unconditional, for important info
#define csi_print(x,arg...) printk(KERN_INFO"[CSI]"x,##arg)

/*
 * the csi port is able to receive the following data formats from the camera. 
 */
enum csi_input_fmt_e
{
    CSI_RAW=0,     /* raw stream  */
    CSI_BAYER,     /* byer rgb242 */
    CSI_CCIR656,   /* ccir656     */
    CSI_YUV422,    /* yuv422      */
};

/*
 * csi bus output data format
 */
enum csi_output_fmt_e
{
    /* only when input is raw */
    CSI_PASS_THROUTH = 0,                /* raw */

    /* only when input is bayer */
    CSI_PLANAR_RGB242 = 0,               /* planar rgb242 */

    /* only when input is ccir656 */
    CSI_FIELD_PLANAR_YUV422 = 0,         /* parse a field(odd or even) into planar yuv420 */
    CSI_FIELD_PLANAR_YUV420 = 1,         /* parse a field(odd or even) into planar yuv420 */
    CSI_FRAME_PLANAR_YUV420 = 2,
    CSI_FRAME_PLANAR_YUV422 = 3,
    CSI_FIELD_UV_CB_YUV422  = 4,         /* parse and reconstruct evry 2 fields(odd and even) into a frame, format is planar yuv420 */
    CSI_FIELD_UV_CB_YUV420  = 5,
    CSI_FRAME_UV_CB_YUV420  = 6,
    CSI_FRAME_UV_CB_YUV422  = 7,
    CSI_FIELD_MB_YUV422     = 8,
    CSI_FIELD_MB_YUV420     = 9,
    CSI_FRAME_MB_YUV422     = 10,
    CSI_FRAME_MB_YUV420     = 11,
    CSI_INTLC_INTLV_YUV422  = 15,

    /* only when input is yuv422 */
    CSI_PLANAR_YUV422=0,                /* parse yuv422 into planar yuv422 */
    CSI_PLANAR_YUV420=1,                /* parse yuv422 into planar yuv420 */
    CSI_UV_CB_YUV422=4,
    CSI_UV_CB_YUV420=5,
    CSI_MB_YUV422=8,
    CSI_MB_YUV420=9,
};

/*
 * input field selection, only when input is ccir656
 */
enum csi_field_sel_e
{
    CSI_ODD,    /* odd field */
    CSI_EVEN,   /* even field */
    CSI_EITHER, /* either field */
};

/*
 * input data sequence
 */
enum csi_seq_e
{
    /* only when input is yuv422 */
    CSI_YUYV=0,  //confirmed tomk! asto allwiner manual!
    CSI_YVYU,
    CSI_UYVY,
    CSI_VYUY,

    /* only when input is byer */
    CSI_RGRG=0,               /* first line sequence is RGRG... */
    CSI_GRGR,                 /* first line sequence is GRGR... */
    CSI_BGBG,                 /* first line sequence is BGBG... */
    CSI_GBGB,                 /* first line sequence is GBGB... */
};

/*
 * input reference signal polarity
 */
enum csi_ref_e
{
    CSI_LOW,    /* active low */
    CSI_HIGH,   /* active high */
};

/*
 * input data valid of the input clock edge type
 */
enum csi_clock_e
{
    CSI_FALLING,    /* active falling */
    CSI_RISING,     /* active rising */
};

/// current configuration og the csi module data input and output port
struct csi_config
{
 enum   csi_input_fmt_e  input_fmt;   //csi input format
 enum   csi_output_fmt_e output_fmt;  // csi output format
 enum   csi_field_sel_e  field_sel;   // odd/even/none video fields. only rlevant for ccir565 format!!
 enum   csi_seq_e        seq;         // input colorfield sequence . only for yuv422 input format
 enum   csi_ref_e        vref;        // input vref signal polarity 
 enum   csi_ref_e        href;        // input href signal polarity 
 enum   csi_clock_e        clock;       // input data valid of the input clock edge type 
};



/*
 * csi buffer
 */

enum csi_buf_e
{
    CSI_BUF_0_A,    /* FIFO for Y address A */
    CSI_BUF_0_B,    /* FIFO for Y address B */
    CSI_BUF_1_A,    /* FIFO for Cb address A */
    CSI_BUF_1_B,    /* FIFO for Cb address B */
    CSI_BUF_2_A,    /* FIFO for Cr address A */
    CSI_BUF_2_B,    /* FIFO for Cr address B */
};

/*
 * csi capture status
 */
struct csi_capture_status
{
    _Bool picture_in_progress;
    _Bool video_in_progress;
};


/*
 * csi double buffer
 */
enum csi_double_buf_e
{
    CSI_BUF_A,
    CSI_BUF_B,
};

/*
 * csi double buffer status
 */
struct csi_double_buf_status
{
    _Bool             enable;   /* double buffer enable */
enum csi_double_buf_e cur;     /* current frame selected output type, buffer A or B*/
enum csi_double_buf_e next;    /* next frame output type, buffer A or B */
};

/*
 * csi interrupt
 */
enum csi_int_e
{
    CSI_INT_CAPTURE_DONE     = 0X1,
    CSI_INT_FRAME_DONE       = 0X2,
    CSI_INT_BUF_0_OVERFLOW   = 0X4,
    CSI_INT_BUF_1_OVERFLOW   = 0X8,
    CSI_INT_BUF_2_OVERFLOW   = 0X10,
    CSI_INT_PROTECTION_ERROR = 0X20,
    CSI_INT_HBLANK_OVERFLOW  = 0X40,
    CSI_INT_VSYNC_TRIG       = 0X80,
};

/*
 * csi interrupt status
 */
struct csi_int_status
{
    _Bool capture_done;
    _Bool frame_done;
    _Bool buf_0_overflow;
    _Bool buf_1_overflow;
    _Bool buf_2_overflow;
    _Bool protection_error;
    _Bool hblank_overflow;
    _Bool vsync_trig;
};

/*
 * csi bus signals required by the connected camera. used in camera driver
 */
struct csi_signal_config
{
    int					mclk;		/* the mclk frequency for sensor module in HZ unit*/
    enum csi_ref_e        vref;        /* input vref signal polarity */
    enum csi_ref_e        href;        /* input href signal polarity */
    enum csi_clock_e      clock;       /* input data valid of the input clock edge type */
    int					csi_port;		/*0 for csi back , 1 for csi front*/
};

struct csi_buf_addr
{
    dma_addr_t	y;  //bufferstart for Y
    dma_addr_t	cb; //bufferstart for cb (U)
    dma_addr_t	cr; //bufferstart for cr (V)
};

struct csi_fmt
{
    u8					name[32];
enum v4l2_mbus_pixelcode cam_fmt;
    u32   				fourcc;          /* v4l2 format id */
enum    csi_input_fmt_e	input_fmt;
enum    csi_output_fmt_e 	output_fmt;
    int   				depth;
    u16	  				planes_cnt;
};

struct csi_size
{
    u32		csi_width;
    u32		csi_height;
};


// in vb2 buffer definition there is no private list_head queue to be used for our private dmaque, so we have to build a custom buffer 
struct cam_buffer 
{
	struct vb2_buffer	vb;  // the vb2 standard buffer. this item is managed by the vb2 layers queue.
	struct list_head	list;  // the private list:head to be used for dmaque.
};



// csi camera device structure contains all relevant information
struct csi_camera
{
    struct v4l2_device 	   	v4l2_dev;  // V4L2 device instance data
    struct v4l2_subdev		*psubdev; // pointer to sub-device data
    struct platform_device	*pdev;		// pointer to platform device data
    struct video_device     *pviddev;  //pointer to video device data

    int						id;  		//platform device id=csi-port 0 or 1
    spinlock_t              slock;		// the global lock to avoid queue messup.

    /* Several counters */
    unsigned 		   		millisecs;
    unsigned long           jiffies;
	
    /*working state*/
    int						opened;

    int			   			input;		// v4l2 device number of csi-connected cameras.video-input: only one, so always 0.

    // current active video format, video working set
	unsigned int 			fourcc;  // the format code
    unsigned int            width;
    unsigned int            height;
	unsigned int			bitsperpixel;
	unsigned int			bytesperline;
    unsigned long			frame_size;
	
	// vieobuffer queue from vb/vb2 layer
	union {
    struct videobuf_queue   vb_vidq;   		// for vb
		struct vb2_queue vb2_vidq;  // for vb2, contains items of type cam_buffer, linked via vb2_buffer->queued_entry
	};
		__u32			sequence; // sequence number of the frames

	struct list_head		dmaque; // driver internal dma buffer queue. contains items of type cam_buffer, linked via cam_buffer->list


    /*pin,clock,irq resource*/
    int						csi_pin_hd;  // csi GPIO pin resource for camera reset etc.
    struct clk				*csi_clk_src;  // csi clock source
    struct clk				*csi_ahb_clk; // ahb bus clock
    struct clk				*csi_module_clk; //csi module clock
    struct clk				*csi_dram_clk; //csi dram clock
    struct clk				*csi_isp_src_clk; //csi isp source clock
    struct clk				*csi_isp_clk; //csi isp clock
    int						irq;  // irq resource
    void __iomem			*regs;  // csi register base addresse
    struct resource			*regs_res;  // csi register resource

	// Camera working parameters
    int					stby_mode;  //flag, camera in standby mode or not
    struct regulator 	 *iovdd;		  /*interface voltage source of sensor module*/
    struct regulator 	 *avdd;			/*anlog voltage source of sensor module*/
    struct regulator 	 *dvdd;			/*core voltage source of sensor module*/
    int interface;  // csi_if, csi bus interface width: 0= 8bit
    int vflip;
    int hflip;
    int flash_pol;
	// new------------------------------
    char cam_name[I2C_NAME_SIZE];
    char iovdd_str[32];
    char avdd_str[32];
    char dvdd_str[32];
    int twi_id;  // i2c interface number
    uint i2c_addr;  // cameras i2c address
	// new end----------------------
    // CSI Bus parameters
    struct csi_config			csi_mode;  // csi-working parameters:. used by bsp_csi_configure
    struct csi_buf_addr		csi_buf_addr;  // csi fifo usage and DMA buffer addresses
    struct csi_signal_config csi_sig_cfg;  // csi bus signal-config of connected camera. returned by camera in get_info

    /* camera config */
    int dev_qty;
};

//protos board support in csi_reg.c
void  bsp_csi_open(struct csi_camera *pcam);
void  bsp_csi_close(struct csi_camera *pcam);
void  bsp_csi_configure(struct csi_camera *pcam,struct csi_config *mode);
void  bsp_csi_double_buffer_enable(struct csi_camera *pcam);
void  bsp_csi_double_buffer_disable(struct csi_camera *pcam);
void  bsp_csi_capture_video_start(struct csi_camera *pcam);
void  bsp_csi_capture_video_stop(struct csi_camera *pcam);
void  bsp_csi_capture_picture(struct csi_camera *pcam);
void  bsp_csi_capture_get_status(struct csi_camera *pcam,struct csi_capture_status * status);
void 	bsp_csi_set_size(struct csi_camera *pcam, u32 length_h, u32 length_v, u32 buf_length_h);
void 	bsp_csi_set_offset(struct csi_camera *pcam,u32 start_h, u32 start_v);
void  bsp_csi_int_enable(struct csi_camera *pcam,enum csi_int_e interrupt);
void  bsp_csi_int_disable(struct csi_camera *pcam,enum csi_int_e interrupt);

#endif  /* _CSI_H_ */
