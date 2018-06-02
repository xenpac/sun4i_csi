/*  using alternate window size routines:
in sensor_try_fmt_internal, zoom_image or scale_image is used!!!


 * drivers/media/video/sun4i_csi/device/ov5640.c
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

/** A V4L2 driver for ov5640 camera modules.
 Intro:
 This driver talks to the camera via the i2c bus. (to set/get camera parameters)
 It also talks to the gpio-pin_io-module for IO-lines for Reset/Standby/etc, the powermodule to set sensor_power,
 and CSI_clock_module to set the camera-clock.
 It does not touch the CSI Bus interface! (All done in the csi-module driver)

 Videoformats on the cameras 8-bit parallel Bus:
 The Video-Dataformat is specified as "enum v4l2_mbus_pixelcode".(so V4l2 Subsystem knows how to deal with the data)
 NOTE: v4l2_mbus_pixelcode is depreciated (linus torvald) and  MEDIA_BUS_FMT_ definitions in media-bus-format.h should be used.
 It tells how video data is coded and how many bits per pixel are used.
 The data then passes through the CSI module.

 The CSI Module:
 Data is captured into a fifo-buffer. Its format can be RAW(any Camera format) or YUV422(default).
 Data can be optionally converted before it leaves the CSI_module out to the memory.
 Possible output conversions are:
 PassThrough = no conversion. mostly used. Input must be set to RAW!
 When input is set to YUV422(default), the following conversions are possible:
 - planar YUV422 (default)
 - planar YUV420
 - planar YUV422 UV combined
 - planar YUV420 UV combined
 - tiled YUV422 (crobbed)
 - tiled YUV420

 Data is then transferred to the memory via CSI-internal DMA, and an Interrupt is generated at the end of a full frame dma-transfer.
 For this, the fifos need a destination memory address (buffer) to place the data of the frame. (This is not a kernel DMA!)
 Double buffering is possible which uses alternating fifos/buffers. (read while other is written to!)

 Picture Sizes:
 The max. Pixelclocks/Bytes per Line the CSI-Module can handle is 4096 Bytes.
 The max. Lines per Frame the CSI-Module can handle is also 4096  (12Bits)
 Depending on the Videoformat one Pixel has a specific Bitlength (depth) of information. common values are 8, 12 or 16 bits per pixel.
 So the max. Pixel per line is reduced to: 4096*8/Bitlength); for 8bit=4096; for 12bit=2730, for 16bit=2048 (pixel per line)
 So the max.Framesize can be max: 4096*4096.

 V4L2 Media Bus Video Frame Format: (V4L2 Video Information fields)
 * struct v4l2_mbus_framefmt - frame format on the media bus
 * @width:	frame width
 * @height:	frame height
 * @code:	data format code (from enum v4l2_mbus_pixelcode)
 * @field:	used interlacing type (from enum v4l2_field), we use: V4L2_FIELD_NONE (fields are not interlaced)
 * @colorspace:	colorspace of the data (from enum v4l2_colorspace)
 * @ycbcr_enc:	YCbCr encoding of the data (from enum v4l2_ycbcr_encoding)
 * @quantization: quantization of the data (from enum v4l2_quantization)
 * @xfer_func:  transfer function of the data (from enum v4l2_xfer_func)

 Common Frame Sizes and Framerates:
  at 24MHZ input clock, the outputted pixelclock is 48MHZ, thus pumping out 48-million Bytes per second.!!
  At two Bytes per Pixel, we can pump out 24 million pixel per second.
  Format      Framesize/Pixel   Framerate  (24-million / Framesize*2 = Frames per Second)
  =====================================
 320 * 240    76800        156
 640 * 480    307200       39
 1280 * 720   921600       13
 1920 * 1080  2073600       5
 1280 * 960   1228800       9
 2592 * 1944  5038840		2

 Framerate also depends on exposure time.

 OV5640 default format after reset:
 2590*1944,
 Raw RGB (it bypasses the internal format controler)
 The sensor-array uses Bayer pattern!

 July 7, 2017, Thomas Krueger
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/clk.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-mediabus.h>
#include <linux/io.h>
#include <plat/sys_config.h>
#include <linux/regulator/consumer.h>
#include <mach/system.h>
#include "../include/sun4i_csi_core.h"
#include "../include/sun4i_dev_csi.h"

MODULE_AUTHOR("Thomas Krueger, Hofgeismar, Germany");
MODULE_DESCRIPTION("V2-Sub_Device Driver for the OV5640 CAM Module");
MODULE_LICENSE("GPL");


//for internel driver debug
#define DBG   		1
//debug level 0~3: 3=only errors,2=+ routine trace, 1=+ details, 0= + interupt messages
#define DBG_LEV 	2

//for internel driver debug
#if(DBG==1)
#define csi_debug(l,x,arg...) if(l >= DBG_LEV) printk(KERN_CRIT"[-----OV5640]"x,##arg)
#else
#define csi_debug(l,x,arg...)
#endif


#define MCLK (24*1000*1000)
#define VREF_POL	CSI_LOW  //act. low for capture
#define HREF_POL	CSI_HIGH //act. high for capture
#define CLK_POL		CSI_RISING //latch data at rising edge of pixelclock

#define V4L2_IDENT_SENSOR 0x5640

//define the voltage level of control signal
#define CSI_STBY_ON			1
#define CSI_STBY_OFF 		0
#define CSI_RST_ON			0
#define CSI_RST_OFF			1
#define CSI_PWR_ON			1
#define CSI_PWR_OFF			0


#define REG_ADDR_STEP 2		// number of i2c address bytes per transfer
#define REG_DATA_STEP 1		// number of i2c databytes per transfer
#define REG_STEP 			(REG_ADDR_STEP+REG_DATA_STEP)		// total number of bytes to transfer per i2c cycle

// Window Sizes:
#define QSXGA_WIDTH		2592
#define QSXGA_HEIGHT	1936
#define QXGA_WIDTH 		2048
#define QXGA_HEIGHT		1536
#define P1080P_WIDTH	1920
#define P1080P_HEIGHT	1080
#define UXGA_WIDTH		1600
#define UXGA_HEIGHT		1200
#define P720_WIDTH 		1280
#define P720_HEIGHT		720
//SXGA: 1280*960
#define SXGA_WIDTH		1280
#define SXGA_HEIGHT		960
#define HD720_WIDTH 	1280
#define HD720_HEIGHT	720
//XGA: 1024*768
#define XGA_WIDTH		1024
#define XGA_HEIGHT 		768
#define SVGA_WIDTH		800
#define SVGA_HEIGHT 	600
#define VGA_WIDTH			640
#define VGA_HEIGHT		480
#define QVGA_WIDTH		320
#define QVGA_HEIGHT		240
#define CIF_WIDTH			352
#define CIF_HEIGHT		288
#define QCIF_WIDTH		176
#define	QCIF_HEIGHT		144



#define SENSOR_FRAME_RATE 30

// The ov5640 sits on i2c with ID 0x78
#define I2C_ADDR 0x78



//protos:
static int sensor_reset(struct v4l2_subdev *sd, u32 val);

// this sensors supported formats
struct sensor_format_struct; 

// This cameras Bus Signal Config
struct csi_signal_config ccm_info_con =
{
    .mclk 	= MCLK,
    .vref 	= VREF_POL,
    .href 	= HREF_POL,
    .clock	= CLK_POL,
    .csi_port	= 0,  // will be correctly set by the driver
};


/** our special camera settings struct
As we are usnig the camera struct from the csi driver, we only save active control settings here.
*/
struct sensor_info
{
    struct v4l2_subdev sd;  // subdev device handle
    struct sensor_format_struct *fmt;  // Current format
    struct csi_signal_config *csi_sig_cfg;
    // picture dimensions
    int	width; 		//in pixel
    int	height;		// in pixel
    //picture options
    int brightness;
    int	contrast;
    int saturation;
    int hue;
    int hflip;
    int vflip;
    int gain;
    int autogain;
    int exp;
    enum v4l2_exposure_auto_type autoexp;
    int autowb;
    enum v4l2_whiteblance wb;
    enum v4l2_colorfx clrfx;
    enum v4l2_flash_mode flash_mode;
// fps stuff
	int fps;
	ulong pxclk;
};

// get address of info sruct from the subdev handle
static inline struct sensor_info *to_state(struct v4l2_subdev *sd)
{
    return container_of(sd, struct sensor_info, sd);
}

// active control settings
struct sensor_control
{
    unsigned char autogain;
    unsigned char gain;
    unsigned char color;
    unsigned char pattern;
    unsigned char autoexp;
    unsigned char hflip;	//0=off; 1=on
    unsigned char vflip;	//0=off; 1=on
} control;

// i2c subdev handle for gloabl usage. (yes, thats bad!)
struct v4l2_subdev *i2cdev;

// +++:++++++++++++++++++++  Sensor Register Value Lists for Windowsize  ***************************
// search for +++:


struct regval_list
{
    unsigned char reg_num[REG_ADDR_STEP];
    unsigned char value[REG_DATA_STEP];
};

//tom: 
static struct regval_list sensor_default_regs[] =
{
//15fps YUV mode

// startup:
    {{0x31,0x03},{0x11}},//pll clock select: 0x01 = pll 0x00=pad clock? 7-2=debug mode.default=0
//{{0x30,0x08},{0x82}},// software reset(7) or powerdown(6).(5-0)=debugmode.default=2
    {{0x30,0x08},{0x42}},//powerdown. during register setting!
    {{0x31,0x03},{0x03}},//(Bit1) System input clock select. 0=extern, 1=pll; default:0
    {{0x30,0x17},{0xff}},//data outputs 0-3 =output 1=output, 0=input. default:0
    {{0x30,0x18},{0xff}},//data outputs 4-9 =output  1=output, 0=input



// PLL Settings:
//    {{0x30,0x39},{0x80}}, //bypass pll, very slow 1fps, so we need pll!
    {{0x30,0x34},{0x1a}},//(6-4)=charge pump loop filter. (3-0)=BITdivider. 1a=10bit(default), 18=8bit(a bit faster!)
    {{0x30,0x35},{0x21}},//(7-4)=SystemClockDivider. (3-0)=MIPIdivider(must stay at 1!). default=0x11
    {{0x30,0x36},{0x46}},//(7-0)=Multipiler. default=0x69.  0x46->30fps
    {{0x30,0x37},{0x13}},//(4)=Rootdivider. (3-0)=Predivider.  default=0x03.
    {{0x31,0x08},{0x01}},//(5-4)=PCLK Rootdivider=0;(3-2)=sclk2x root divider=dontcare; (1-0)=SCLK root divider; default:0x16.



/*	// define ISP input size (Subwindow to be scanned from sensor) 
    //The ISP input size is the total pixel data read from sensor pixel array.
    //Typically the larger ISP input size, the less frame rate can be reached.
    {{0x38,0x00},{0x00}},             //sensor window x-start = 0  (default 0) 
    {{0x38,0x01},{0x00}},             //
    {{0x38,0x02},{0x00}},             //sensor window y-start = 4  (default 0)
    {{0x38,0x03},{0x04}},             //
    {{0x38,0x04},{0x0a}},             //sensor window x-end  = a3f=2623    (default 2623)
    {{0x38,0x05},{0x3f}},             //
    {{0x38,0x06},{0x07}},             //sensor window y-end = 79b = 1947   (default 1951)
    {{0x38,0x07},{0x9b}},             //subwindow = 2623 * 1943

    // Final output window size. either from DVP/DownScale(Scale enable), or from ISP/Subwindowed(Scale disable). This is a MUST!!!!!
    //Der Bildausschnitt bleibt der gleiche bei veranderten werten, die Helligkeit (exposure) wird aber besser bei grösserem format, mehr zeit.
    {{0x38,0x08},{0x02}},            //output width from x-start = 280 = 640    DVPHO (default 2592)
    {{0x38,0x09},{0x80}},            //
    {{0x38,0x0a},{0x01}},            //output height from y-start = 1e0 = 480   DVPVO (default 1944)
    {{0x38,0x0b},{0xe0}},

    // Total physical Window Size of image sensor(sensor array size):
	// 0c5640 sensor array = 2624 * 1964. active pixel that can be outputted: 2592 * 1944. minxy-offset=32/8!
    {{0x38,0x0c},{0x07}},             //total width = 768 = 1896   HTS (default 2844)  Horizontal total size for 1 line in pclk
    {{0x38,0x0d},{0x68}},             //
    {{0x38,0x0e},{0x03}},             //total height = 3d8 = 984   VTS (default 1968)  Vertical total size for 1 frame
    {{0x38,0x0f},{0xd8}},             //

// define an x and y offset from x/y-start for origin of ISP-Subwindow	- this is the Output window. (no Scaler!)
// define pre-scale size:
    {{0x38,0x10},{0x00}},             //isp x-offset = 10 = 16  H offset(default 16) default min offset = valid array size
    {{0x38,0x11},{0x10}},              //
    {{0x38,0x12},{0x00}},              //isp y-offset = 6 = 6   V offset(default 4)
    {{0x38,0x13},{0x06}},              //resulting isp-input= 2591 * 1931
------------------------------------------------*/


    {{0x3c,0x00},{0x04}},
    {{0x3c,0x01},{0xb4}},
//    {{0x31,0x08},{0x01}},//
    {{0x36,0x30},{0x36}},//
    {{0x36,0x31},{0x0e}},//
    {{0x36,0x32},{0xe2}},//
    {{0x36,0x33},{0x12}},//
    {{0x36,0x21},{0xe0}},//
    {{0x37,0x04},{0xa0}},//
    {{0x37,0x03},{0x5a}},//
    {{0x37,0x15},{0x78}},//
    {{0x37,0x17},{0x01}},//
    {{0x37,0x0b},{0x60}},//
    {{0x37,0x05},{0x1a}},//
    {{0x39,0x05},{0x02}},//
    {{0x39,0x06},{0x10}},//
    {{0x39,0x01},{0x0a}},//
    {{0x37,0x31},{0x12}},//
    {{0x36,0x00},{0x08}},//
    {{0x36,0x01},{0x33}},//
    {{0x30,0x2d},{0x60}},//
    {{0x36,0x20},{0x52}},//
    {{0x37,0x1b},{0x20}},//
    {{0x47,0x1c},{0x50}},//
    {{0x3a,0x13},{0x43}},//
    {{0x3a,0x18},{0x00}},//
    {{0x3a,0x19},{0xf8}},//
    {{0x36,0x35},{0x13}},//
    {{0x36,0x36},{0x03}},//
    {{0x36,0x34},{0x40}},//
    {{0x36,0x22},{0x01}},//
    {{0x3c,0x01},{0x34}},//
    {{0x3c,0x04},{0x28}},//
    {{0x3c,0x05},{0x98}},//
    {{0x3c,0x06},{0x00}},//
    {{0x3c,0x07},{0x08}},//
    {{0x3c,0x08},{0x00}},//
    {{0x3c,0x09},{0x1c}},//
    {{0x3c,0x0a},{0x9c}},//
    {{0x3c,0x0b},{0x40}},//
//{{0x38,0x20},{0x41}},//
//{{0x38,0x21},{0x07}},//

    {{0x38,0x14},{0x31}},// muss
    {{0x38,0x15},{0x31}},// muss
    {{0x38,0x14},{0x11}},// subsample off
    {{0x38,0x15},{0x11}},// subsample off

//window size
    {{0x38,0x00},{0x00}},//
    {{0x38,0x01},{0x00}},//00
    {{0x38,0x02},{0x00}},//
    {{0x38,0x03},{0x04}},//
    {{0x38,0x04},{0x0a}},//
    {{0x38,0x05},{0x3f}},//
    {{0x38,0x06},{0x07}},//
    {{0x38,0x07},{0x9b}},//


    {{0x38,0x08},{0x02}},//
    {{0x38,0x09},{0x80}},//
    {{0x38,0x0a},{0x01}},//
    {{0x38,0x0b},{0xe0}},//
	

    {{0x38,0x0c},{0x0b}},// 1896. 0x0768
    {{0x38,0x0d},{0x1c}},//
    {{0x38,0x0e},{0x07}},// 984- 0x03d8
    {{0x38,0x0f},{0xb0}},//

    {{0x38,0x10},{0x00}},//
    {{0x38,0x11},{0x10}},//
    {{0x38,0x12},{0x00}},//
    {{0x38,0x13},{0x06}},//
//window size end


    {{0x36,0x18},{0x00}},// ??mustbe
    {{0x36,0x12},{0x29}},// ??must be
//-------------------------------------------	
	
    {{0x37,0x08},{0x64}},// ??
    {{0x37,0x09},{0x52}},// ??
    {{0x37,0x0c},{0x03}},// ??
	
    {{0x3a,0x02},{0x03}},// night mode ceiling
    {{0x3a,0x03},{0xd8}},// night mode ceiling
    {{0x3a,0x08},{0x01}},// banding steps 50/60Hz
    {{0x3a,0x09},{0x27}},// banding steps 50/60Hz
    {{0x3a,0x0a},{0x00}},// banding steps 50/60Hz
    {{0x3a,0x0b},{0xf6}},// banding steps 50/60Hz
    {{0x3a,0x0e},{0x03}},// AEC B50 max
    {{0x3a,0x0d},{0x04}},// AEC B60 max
    {{0x3a,0x14},{0x03}},// AEC Max exposure 50Hz
    {{0x3a,0x15},{0xd8}},// AEC Max exposure 50Hz
	
    {{0x40,0x01},{0x02}},// BLC control1
    {{0x40,0x04},{0x02}},// BLC control4
	
    {{0x30,0x00},{0x00}},// System Reset00:=off=enable 50/60Hz detect
    {{0x30,0x02},{0x1c}},// System Reset02: (default:0x1C)
    {{0x30,0x04},{0xff}},// Clock enable: (default:0xCF)
    {{0x30,0x06},{0xc3}},// Clock enable2: (default:0xE3)
    {{0x30,0x0e},{0x58}},// Mipi Control00:(default:0x58)
    {{0x30,0x2e},{0x00}},// ??

    {{0x30,0x2c},{0xc2}},//bit[7:6]: output drive capability
    //00: 1x   01: 2x  10: 3x  11: 4x

    {{0x43,0x00},{0x30}},// Format Control: YUV422-YUYV. (default:0xF8)
	
    {{0x50,0x1f},{0x00}},// Format Mux Control:ISP YUV422. muss ein

    {{0x47,0x13},{0x03}},// JPEG Mode:(default:0x02)
    {{0x44,0x07},{0x04}},// JPEG control7:(default:0x0C)
    {{0x44,0x0e},{0x00}},// ??

    {{0x46,0x0b},{0x35}},// debug mode
    {{0x46,0x0c},{0x20}},// FIFO Control (default:0x20)
    {{0x48,0x37},{0x22}}, // PCLK Period (default:0x10)
    {{0x38,0x24},{0x02}},//DVP PCLK divider value
	
    {{0x50,0x00},{0xa7}},//0x80=LensCorrection=0n, 0x40=?, 0x04=BlackPixelCancel; 0x02=WhitePixelCancel;0x01=ColorInterpolation (Default:0x06)
    {{0x50,0x01},{0xa3}},//0x80=Digital Effecrs;0x40=?; 0x02=Colormatrix;0x01=AutoWhiteBalance  (Default=0x01). muss sein!!
//--------oben muss sein

// AWB settings:
    {{0x51,0x80},{0xff}},
    {{0x51,0x81},{0xf2}},
    {{0x51,0x82},{0x00}},
    {{0x51,0x83},{0x14}},
    {{0x51,0x84},{0x25}},
    {{0x51,0x85},{0x24}},
    {{0x51,0x86},{0x0f}},
    {{0x51,0x87},{0x0f}},
    {{0x51,0x88},{0x0f}},
    {{0x51,0x89},{0x80}},
    {{0x51,0x8a},{0x5d}},
    {{0x51,0x8b},{0xe3}},
    {{0x51,0x8c},{0xa7}},
    {{0x51,0x8d},{0x40}},
    {{0x51,0x8e},{0x33}},
    {{0x51,0x8f},{0x5e}},
    {{0x51,0x90},{0x4e}},
	
    {{0x51,0x91},{0xf8}}, //AWB Top Limit
    {{0x51,0x92},{0x04}}, //AWB Bottom Limit
	
    {{0x51,0x93},{0x70}},
    {{0x51,0x94},{0xf0}},
    {{0x51,0x95},{0xf0}},
    {{0x51,0x96},{0x03}},
    {{0x51,0x97},{0x01}},
    {{0x51,0x98},{0x06}},
    {{0x51,0x99},{0xd0}},
    {{0x51,0x9a},{0x04}},
    {{0x51,0x9b},{0x00}},
    {{0x51,0x9c},{0x04}},
    {{0x51,0x9d},{0x87}},
    {{0x51,0x9e},{0x38}},
//---end AWB

//color Matrix:
    {{0x53,0x81},{0x1e}}, // color matrix enable
    {{0x53,0x82},{0x5b}},
    {{0x53,0x83},{0x08}},
    {{0x53,0x84},{0x0a}},
    {{0x53,0x85},{0x7e}},
    {{0x53,0x86},{0x88}},
    {{0x53,0x87},{0x7c}},
    {{0x53,0x88},{0x6c}},
    {{0x53,0x89},{0x10}},
    {{0x53,0x8a},{0x01}},
    {{0x53,0x8b},{0x98}},
    {{0x53,0x00},{0x08}},
    {{0x53,0x01},{0x30}},
    {{0x53,0x02},{0x10}},
    {{0x53,0x03},{0x00}},
    {{0x53,0x04},{0x08}},
    {{0x53,0x05},{0x30}},
    {{0x53,0x06},{0x08}},
    {{0x53,0x07},{0x16}},
    {{0x53,0x09},{0x08}},
    {{0x53,0x0a},{0x30}},
    {{0x53,0x0b},{0x04}},
    {{0x53,0x0c},{0x06}},
//Gamma Control:	
    {{0x54,0x80},{0x01}},
    {{0x54,0x81},{0x08}},
    {{0x54,0x82},{0x14}},
    {{0x54,0x83},{0x28}},
    {{0x54,0x84},{0x51}},
    {{0x54,0x85},{0x65}},
    {{0x54,0x86},{0x71}},
    {{0x54,0x87},{0x7d}},
    {{0x54,0x88},{0x87}},
    {{0x54,0x89},{0x91}},
    {{0x54,0x8a},{0x9a}},
    {{0x54,0x8b},{0xaa}},
    {{0x54,0x8c},{0xb8}},
    {{0x54,0x8d},{0xcd}},
    {{0x54,0x8e},{0xdd}},
    {{0x54,0x8f},{0xea}},
    {{0x54,0x90},{0x1d}},
	
// SDE Control:	
    {{0x55,0x80},{0x04}},
    {{0x55,0x87},{0x05}},
    {{0x55,0x88},{0x09}},
    {{0x55,0x83},{0x40}},
    {{0x55,0x84},{0x10}},
    {{0x55,0x89},{0x10}},
    {{0x55,0x8a},{0x00}},
    {{0x55,0x8b},{0xf8}},
	
//Lens Control register, correction values:	
    {{0x58,0x00},{0x3D}},
    {{0x58,0x01},{0x1F}},
    {{0x58,0x02},{0x17}},
    {{0x58,0x03},{0x16}},
    {{0x58,0x04},{0x1E}},
    {{0x58,0x05},{0x3A}},
    {{0x58,0x06},{0x14}},
    {{0x58,0x07},{0x0A}},
    {{0x58,0x08},{0x07}},
    {{0x58,0x09},{0x06}},
    {{0x58,0x0A},{0x0A}},
    {{0x58,0x0B},{0x11}},
    {{0x58,0x0C},{0x0B}},
    {{0x58,0x0D},{0x04}},
    {{0x58,0x0E},{0x00}},
    {{0x58,0x0F},{0x00}},
    {{0x58,0x10},{0x04}},
    {{0x58,0x11},{0x0A}},
    {{0x58,0x12},{0x0B}},
    {{0x58,0x13},{0x04}},
    {{0x58,0x14},{0x00}},
    {{0x58,0x15},{0x00}},
    {{0x58,0x16},{0x04}},
    {{0x58,0x17},{0x0A}},
    {{0x58,0x18},{0x14}},
    {{0x58,0x19},{0x0A}},
    {{0x58,0x1A},{0x06}},
    {{0x58,0x1B},{0x06}},
    {{0x58,0x1C},{0x09}},
    {{0x58,0x1D},{0x12}},
    {{0x58,0x1E},{0x3D}},
    {{0x58,0x1F},{0x21}},
    {{0x58,0x20},{0x18}},
    {{0x58,0x21},{0x17}},
    {{0x58,0x22},{0x1F}},
    {{0x58,0x23},{0x3B}},
    {{0x58,0x24},{0x37}},
    {{0x58,0x25},{0x36}},
    {{0x58,0x26},{0x28}},
    {{0x58,0x27},{0x25}},
    {{0x58,0x28},{0x37}},
    {{0x58,0x29},{0x35}},
    {{0x58,0x2A},{0x25}},
    {{0x58,0x2B},{0x34}},
    {{0x58,0x2C},{0x24}},
    {{0x58,0x2D},{0x26}},
    {{0x58,0x2E},{0x26}},
    {{0x58,0x2F},{0x32}},
    {{0x58,0x30},{0x50}},
    {{0x58,0x31},{0x42}},
    {{0x58,0x32},{0x16}},
    {{0x58,0x33},{0x36}},
    {{0x58,0x34},{0x35}},
    {{0x58,0x35},{0x34}},
    {{0x58,0x36},{0x34}},
    {{0x58,0x37},{0x26}},
    {{0x58,0x38},{0x26}},
    {{0x58,0x39},{0x36}},
    {{0x58,0x3A},{0x28}},
    {{0x58,0x3B},{0x36}},
    {{0x58,0x3C},{0x37}},
    {{0x58,0x3D},{0xCE}},
	
    {{0x50,0x25},{0x00}}, //????

//----ja below
	
	// aec limits, ohne etwas zu hell
    {{0x3a,0x0f},{0x30}},
    {{0x3a,0x10},{0x28}},
    {{0x3a,0x1b},{0x30}},
    {{0x3a,0x1e},{0x26}},
    {{0x3a,0x11},{0x60}},
    {{0x3a,0x1f},{0x14}},


//-----
    {{0x30,0x08},{0x02}}, //poweron at end




};



// +++:++++++++++++++++++++  Sensor Register Value Lists for color formats  ***************************


static struct regval_list sensor_fmt_yuv422_yuyv[] =
{

    {{0x43,	0x00} , {0x30}}	//YUYV
};



// +++:++++++++++++++++++++  Sensor Register Value Lists for Control settings  ***************************

// +++:++++++++++++++++++++  Pixel Formats  ***************************

/** supported video data formats of this sensor
*/
static struct sensor_format_struct
{
// video format: (as seen in the video buffer)
    u32 fourcc; // videoformat. if csi conversion, its the output format! see videodev2.h for defines!
    int bitsperpixel; // for this format. (bytesperline). if csi conversion, bytesperline equals the max plane (Y) = width!
// csi format conversion info:
    enum csi_input_fmt_e	csi_input;  // csi module format converter input
    enum csi_output_fmt_e 	csi_output; // csi module format converter output
    enum csi_seq_e			byte_order; // the byte order of the color of input, ie. YUYV, YVYU, UYVY,VYUY
//camera init data:
    struct regval_list *regs;  // the cameras videoformat. for csi conversion always yuv422. i2c init array to intialyze a format
    int	regs_size;  // number of entrys in above init table

} sensor_formats[] =
{
 
    {
        .fourcc		=  V4L2_PIX_FMT_YUYV,  //yuyv direct from camera, geht
        .bitsperpixel	= 16,
        .csi_input = CSI_RAW,
        .csi_output = CSI_PASS_THROUTH,
        .byte_order = 0,
        .regs 		= sensor_fmt_yuv422_yuyv,
        .regs_size = ARRAY_SIZE(sensor_fmt_yuv422_yuyv),
    },


};

 

// +++:++++++++++++++++++++  Sensor Window Sizes   ***************************


/** supported screen sizes of this camera
*/
 struct sensor_win_size
{
    int	width;
    int	height;
    // register list
    struct regval_list *regs;
    int regs_size;
} ;
// +++:++++++++++++++++++++  I2C IO functions ***************************


/** i2c read a register value from camera
@param sd = pointer to i2c subdevice
@param reg = pointer to address_byte_array, number of byte defined in REG_ADDR_STEP
@param value = pointer to receiving byte_data
@return 0=success; else error
*/
static int sensor_read(struct v4l2_subdev *sd, unsigned char *reg,  unsigned char *value)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    u8 data[REG_STEP];
    struct i2c_msg msg;
    int ret,i;

    for(i = 0; i < REG_ADDR_STEP; i++)
        data[i] = reg[i]; //copy address bytes

    for(i = REG_ADDR_STEP; i < REG_STEP; i++)
        data[i] = 0xff; // append 0xff as filler, if any
    /*
     * Send out the register address...
     */
    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = REG_ADDR_STEP;
    msg.buf = data;
    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret < 0)
    {
        csi_debug(3,"Error %d on register write\n", ret);
        return ret;
    }
    /*
     * ...then read the value.
     */

    msg.flags = I2C_M_RD;
    msg.len = REG_DATA_STEP;
    msg.buf = &data[REG_ADDR_STEP];

    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret >= 0)
    {
        for(i = 0; i < REG_DATA_STEP; i++)
            value[i] = data[i+REG_ADDR_STEP];
        ret = 0;
    }
    else
    {
        csi_debug(3,"Error %d on register read\n", ret);
    }
    return ret;
}

/** write a registervalue to camera
@param sd = pointer to i2c subdevice
@param reg = pointer to address_byte_array, number of byte defined in REG_ADDR_STEP
@param value = pointer to  byte_data
@return 0=success; else error

*/
static int sensor_write(struct v4l2_subdev *sd, unsigned char *reg,
                        unsigned char *value)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    struct i2c_msg msg;
    unsigned char data[REG_STEP];
    int ret,i;

    for(i = 0; i < REG_ADDR_STEP; i++)
        data[i] = reg[i];
    for(i = REG_ADDR_STEP; i < REG_STEP; i++)
        data[i] = value[i-REG_ADDR_STEP];

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = REG_STEP;
    msg.buf = data;


    ret = i2c_transfer(client->adapter, &msg, 1);
    if (ret > 0)
    {
        ret = 0;
    }
    else if (ret < 0)
    {
        csi_debug(3,"sensor_write error!\n");
    }
    return ret;
}




/** Write a list of registers
@param sd = pointer to i2c subdevice
@param reg = pointer to the register value list
@param size = number of entrys in the register value list
@return 0=success; else error
*/
static int sensor_write_array(struct v4l2_subdev *sd, struct regval_list *vals , uint size)
{
    int i,ret;
//	unsigned char rd;
    if (size == 0)
        return -EINVAL;

    for(i = 0; i < size ; i++)
    {
        if(vals->reg_num[0] == 0xff && vals->reg_num[0] == 0xff)
        {
            msleep(vals->value[0] * 256 + vals->value[1]);
        }
        else
        {
            ret = sensor_write(sd, vals->reg_num, vals->value);
            if (ret < 0)
            {
                csi_debug(3,"sensor_write_err!\n");
                return ret;
            }
        }
        vals++;
    }

    return 0;
}



// +++:++++++++++++++++++++  Sensor Controls Functions, camera-module specific  ***************************



/* pll settings:	input clock(XVCLK)=24MHZ from CSI. (camera XVCLK input range= 6 - 27 MHZ)
Block diagram:
                                                                                      >MIPIdivider->mipi clock
                                                                                      |
XVCLK->Predivider->Phase detector->Loop filter(chargepump)->VCO->System clock divider-> Rootdivider->BITdivider->PCLKdivider->PCLK
                  |                                             |
                  <--Frequency Divider(Multiplier)<-------------< 500-1000Mhz

PLL Block explain:
The phase detector sees fe. 8 mhz from input and 8mhz from feedback to be inlock.
8mhz * multiplier = pll-Fout.
if multiplier is 10, Fout = 80mhz.

PixelClock Calc: (prepare below values from the registers)
- for VCO:
Predivider = reg0x3037 & 0x0f;  (0=1;1=1;2=2;3=3;4=4;5=1.5;6=6;7=2.5;8=8;9-15=1;)
Multiplier = reg0x3036;   (PLL feedback) (finetuning)(4-127, 128-252=only even)
- for PCLK:
SystemClockDivider = reg3035>>4; (divides PLLout into the system)(0=16; else 1-15)
if (SystemClockDivider==0) SystemClockDivider = 16;
Rootdivider = (reg0x3037 & 0x10) + 1;  // so if set, divide by 2
BITdivider = reg0x3034 & 0x0f; // only 2 values allowed. (0x08=2; 0x0A=2.5)
-
PCLKdivider = (reg0x3108 =0x01) make it constant factor=0x01!. For calc,take constant Factor of: 6
MIPIdivider = (reg0x3035 & 0x0f) (not used for PCLK)

Step1:	System clock divider ( make it a constant factor in your calculation!)
SYSdivider = SystemClockDivider * Rootdivider * BITdivider * PCLKdivider

Step2: calc VCO ((PLLout) (fine tune with this!)
PLL_Output: VCO = (Inputclock/Predivider) * Multiplier;  Best Range: 500 - 1000 Mhz


Step3:	calc PCLK = Pixelclock for parallel port output data.
PCLK = VCO / SYSdivider

Step4:   calc Frames per second based on your picture size. example YUV422(16bit per pixel): 640*480*2 = 614400 Bytes
FPS = PCLK / 0,614400
or
PCLK = FPS * 0,614400

---
Example:
SYSdivider = 2 * 2 * 2.5 * 6 = 60
VCO = 24/3 * 70 = 560   (Mhz)
PCLK = VCO / SYSdivider = 9,33   (Mhz)
FPS = 9,33/0,614400 = 15.1
or (needed PCLK)
PCLK = 15 * 0,614400 = 9,21   (Mhz)

Thats it ! and it works!

// PLL Settings:
//    {{0x30,0x39},{0x80}}, //bypass pll, gives only 1 fps!
    {{0x30,0x34},{0x1a}},//(6-4)=charge pump loop filter. (3-0)=BITdivider. 1a=10bit(default), 18=8bit(a bit faster!)
    {{0x30,0x35},{0x21}},//(7-4)=SystemClockDivider. (3-0)=MIPIdivider(must stay at 1!). default=0x11
    {{0x30,0x36},{0x46}},//(7-0)=Multipiler. default=0x69.  0x46->30fps
    {{0x30,0x37},{0x13}},//(4)=Rootdivider. (3-0)=Predivider.  default=0x03.

	{{0x31,0x08},{0x16}}, //(5-4)=PCLKdivider;(3-2)=sclk2x root divider; (1-0)=SCLK root divider; default:0x16.

*/
/** get pixel clock
exit: 0= error, else pixelclock
*/
static ulong get_pclk(struct v4l2_subdev *sd)
{
    struct sensor_info *info = to_state(sd);
    unsigned long pclk,VCO,SYSdivider;
    unsigned char Predivider,Multiplier,SystemClockDivider,Rootdivider,BITdivider,PCLKdivider;
    unsigned char address[REG_ADDR_STEP];


        address[0]=0x30;
        address[1]=0x37;
    sensor_read(i2cdev, address, &Predivider);
    Predivider = Predivider & 0x0f; //Predivider
	//(0=1;1=1;2=2;3=3;4=4;5=1.5;6=6;7=2.5;8=8;9-15=1;) we take *10 to support fraction
	switch (Predivider)
	{
	case 2:
	Predivider=20;//2
	break;
	case 3:
	Predivider=30;//3
	break;
	case 4:
	Predivider=40;//4
	break;
	case 5:
	Predivider=15;//1.5
	break;
	case 6:
	Predivider=60;//6
	break;
	case 7:
	Predivider=25;//2.5
	break;
	case 8:
	Predivider=80;//8
	break;
	case 0:
	case 1:
	default:
	Predivider=10; //1
	break;
	}


        address[0]=0x30;
        address[1]=0x36;
    sensor_read(i2cdev, address, &Multiplier); //Multiplier. (4-127, 128-252=only even)
   if(Multiplier > 128)
	{
		Multiplier &= 0xFE; //only even values!
	}

        address[0]=0x30;
        address[1]=0x35;
    sensor_read(i2cdev, address, &SystemClockDivider); //SystemClockDivider
    SystemClockDivider  >>= 4;
	if (!SystemClockDivider) SystemClockDivider=16;

        address[0]=0x30;
        address[1]=0x37;
    sensor_read(i2cdev, address, &Rootdivider); //Rootdivider
	if (Rootdivider & 0x10) Rootdivider=2;
	else Rootdivider=1;

        address[0]=0x30;
        address[1]=0x34;
    sensor_read(i2cdev, address, &BITdivider); //BITdivider (0x08=2; 0x0A=2.5) we take *10 to support fraction
    BITdivider &=  0x0f;
	switch (BITdivider)
	{
	case 0x08: // 2
	BITdivider = 20;
	break;
	case 0x0a: //2.5
	BITdivider = 25;
	break;
	default:
csi_debug(2,"invalid Bitdivider!\n");	
	BITdivider = 10; // assume 1
break;
	}

        address[0]=0x31;
        address[1]=0x08;
    sensor_read(i2cdev, address, &PCLKdivider); //SCLK root divider
    PCLKdivider &= 0x03;
   PCLKdivider = 1 << PCLKdivider;


		VCO = (MCLK / Predivider) * Multiplier * 10;

SYSdivider = SystemClockDivider * Rootdivider * BITdivider * PCLKdivider;

        pclk = (VCO / SYSdivider) * 10;

    csi_debug(2,"Predivider = %d,Multiplier = %d,SystemClockDivider = %d,Rootdivider = %d,BITdivider = %d,PCLKdivider = %d\n",\
                Predivider,Multiplier,SystemClockDivider,Rootdivider,BITdivider,PCLKdivider);
csi_debug(2,"Pxclk: %lu  VCO:%lu SYSDIV: %lu\n",pclk,VCO,SYSdivider);

    info->pxclk = pclk;
    return pclk;
}


void dmp(void) //dump register list for debug
{
    unsigned char address[REG_ADDR_STEP], data,i;
        address[0]=0x38;
	

	i=0;
	while (i<16)
	{
	        address[1]=i;
    sensor_read(i2cdev, address, &data);
	csi_debug(2,"reg:0x%02x val:0x%02x\n",i,data);
	i++;
	}

}

/*
max gain =248, min gain = 16
max exposure = 980, min exposure= 2..measured 

settings info:
agc aec: exposure(3500-02), gain(350a-b),luminance(56a1),extra(350c-d),aec(3503),agc,ashigh(3a0f),aslow(3a10),achigh(3a1b),aclow(3a1e)
(exta-lines only in nightmode or manual aec!)
aec-average-lumina enable manual and size: aw,xstart,xend,ystart,yend);
aec submodes 50/60Hz banding abf: bandon(3aA00_5),bandauto(3c01_7),bandfreq(3C0C_0) 
aec nightmode: nighton(3aA00_2), frameinsert(3A05_5), nightgain(3A17_0,1), max_expos60(3A02,3A03),max_expos50(3A14,3A15),

blcon(blacllevel)(4000_0), lencon(5000_7),rawg(5000_5), bpc((5000_2), wpc(5000_1), cip(5000_0),
sde(5001_7),scaleon(5001_5), uva(5001_2), colmat(5001_1), awb(5001_0),

subsampon, 2x2on, binon, subsamvalues: xx xx
  0x3814-15, subsample: 0x11=off, 0x31=2. if both 0x31 then 2x2 
  0x3821 [0] =binning. vertical binning will automatically turn on when in vertical-subsampled formats
  
*/
// info about used settings: this a debug function to get interesting settings info of the camera.
void showinfo(void) 
{
unsigned char address[REG_ADDR_STEP], data;
unsigned int luminance, gain, aec,agc, ashigh,aslow,achigh,aclow,extra;
unsigned long exposure;
unsigned  aw, xstart,xend,ystart,yend,bandon, bandauto,bandf50,nighton,frameinsert,nightgain,max_expo60,max_expo50,minexpo;
unsigned blc,lenc,rawg,bpc,wpc,cip, scaleon, sde, uva,colmat,awb,subsampon, binon, subh,subv;

// luminance 
address[0]=0x56;
address[1]=0xa1;
sensor_read(i2cdev, address, &data);
luminance = data;
//gain
address[0]=0x35;
address[1]=0x0a;
sensor_read(i2cdev, address, &data);
gain=data&0x03;
gain <<=8;
address[1]=0x0b;
sensor_read(i2cdev, address, &data);
gain|=data;
//ashigh
address[0]=0x3a;
address[1]=0x0f;
sensor_read(i2cdev, address, &data);
ashigh=data;
//aslow
address[0]=0x3a;
address[1]=0x10;
sensor_read(i2cdev, address, &data);
aslow=data;
//achigh
address[0]=0x3a;
address[1]=0x1b;
sensor_read(i2cdev, address, &data);
achigh=data;
//aclow
address[0]=0x3a;
address[1]=0x1e;
sensor_read(i2cdev, address, &data);
aclow=data;
//exposure
address[0]=0x35;
address[1]=0x00;
sensor_read(i2cdev, address, &data);
exposure = data&0x0f;
exposure<<=8;
address[1]=0x01;
sensor_read(i2cdev, address, &data);
exposure |= data;
exposure<<=8;
address[1]=0x02;
sensor_read(i2cdev, address, &data);
exposure |= data;
exposure >>=4;
// extra dummy lines
address[0]=0x35;
address[1]=0x0c;
sensor_read(i2cdev, address, &data);
extra=data;
extra<<=8;
address[1]=0x0d;
sensor_read(i2cdev, address, &data);
extra |= data;

// enables
address[0]=0x35;
address[1]=0x03;
sensor_read(i2cdev, address, &data);
aec=agc=0;
if  (!(data&0x01)) aec=1;
if  (!(data&0x02)) agc=1;

csi_debug(3,"--->Exposure:%lu Gain:%u Lum:%u extraL:%u aec:%u agc:%u ash:%u asl:%u ach:%u acl:%u\n", exposure, gain,luminance,extra,aec,agc,ashigh,aslow,achigh,aclow);

// read avg window enable 
address[0]=0x50;
address[1]=0x1d;
sensor_read(i2cdev, address, &data);
aw=0;
//get avg window
if (data&0x10) aw=1;
address[0]=0x56;
address[1]=0x80;
sensor_read(i2cdev, address, &data);
xstart=data&0x0f;
xstart<<=8;
address[1]=0x81;
sensor_read(i2cdev, address, &data);
xstart|=data;

address[1]=0x84;
sensor_read(i2cdev, address, &data);
xend=data&0x0f;
xend<<=8;
address[1]=0x85;
sensor_read(i2cdev, address, &data);
xend|=data;

address[1]=0x82;
sensor_read(i2cdev, address, &data);
ystart=data&0x07;
ystart<<=8;
address[1]=0x83;
sensor_read(i2cdev, address, &data);
ystart|=data;

address[1]=0x86;
sensor_read(i2cdev, address, &data);
yend=data&0x07;
yend<<=8;
address[1]=0x87;
sensor_read(i2cdev, address, &data);
yend|=data;

csi_debug(3,"- avlumwinON:%u xstart:%u xend:%u ystart:%u yend:%u\n",aw,xstart,xend,ystart,yend);

address[0]=0x3a;
address[1]=0x00;
sensor_read(i2cdev, address, &data);
bandon=0;
if (data&0x20) bandon=1;
address[0]=0x3c;
address[1]=0x01;
sensor_read(i2cdev, address, &data);
bandauto=0;
if (data&0x80) bandauto=1;
address[0]=0x3c;
address[1]=0x0c;
sensor_read(i2cdev, address, &data);
bandf50=0; //60hz
if (data&0x01) bandf50=1;

csi_debug(3,"- bandon:%u bandauto:%u bandfreq50:%u\n",bandon, bandauto,bandf50);

address[0]=0x3a;
address[1]=0x00;
sensor_read(i2cdev, address, &data);
nighton=0;
if (data%0x02) nighton=1;

address[1]=0x05;
sensor_read(i2cdev, address, &data);
frameinsert=0;
if (data&0x20) frameinsert=1;

address[1]=0x17;
sensor_read(i2cdev, address, &data);
data&=0x03;
switch (data)
{
case 0:
nightgain=0;
break;
case 1:
nightgain=10;
break;
case 2:
nightgain=30;
break;
case 3:
nightgain=70;
break;

}

address[1]=0x02;
sensor_read(i2cdev, address, &data);
max_expo60 = data;
max_expo60<<=8;
address[1]=0x03;
sensor_read(i2cdev, address, &data);
max_expo60 |= data;

address[1]=0x14;
sensor_read(i2cdev, address, &data);
max_expo50 = data;
max_expo50<<=8;
address[1]=0x15;
sensor_read(i2cdev, address, &data);
max_expo50 |= data;

address[1]=0x01;
sensor_read(i2cdev, address, &data);
minexpo=data;
csi_debug(3,"- nighton:%u frameinsert:%u nightgain:%u max_expo60:%u max_expo50:%u minexpo:%u\n",nighton,frameinsert,nightgain,max_expo60,max_expo50,minexpo); 

address[0]=0x40;
address[1]=0x00;
sensor_read(i2cdev, address, &data);
blc=0;
if (data&0x01) blc=1;
address[0]=0x50;
address[1]=0x00;
sensor_read(i2cdev, address, &data);
lenc=rawg=bpc=wpc=cip=0;
if (data%0x80) lenc=1;
if (data%0x20) rawg=1;
if (data%0x04) bpc=1;
if (data%0x02) wpc=1;
if (data%0x01) cip=1;

address[0]=0x50;
address[1]=0x00;
sensor_read(i2cdev, address, &data);
sde=scaleon=uva=colmat=awb=0;
if (data&0x80) sde=1;
if (data&0x20) scaleon=1;
if (data&0x04) uva=1;
if (data&0x02) colmat=1;
if (data&0x01) awb=1;

csi_debug(3,"- blc:%u lenc:%u rawg:%u bpc:%u wpc:%u cip:%u\n", blc,lenc,rawg,bpc,wpc,cip);
csi_debug(3,"- scaleon:%u sde:%u uva:%u colmat:%u awb:%u\n", scaleon, sde, uva,colmat,awb);

address[0]=0x38;
address[1]=0x14;
sensor_read(i2cdev, address, &data);
subv=subh=subsampon=binon=0;
subh=data;
address[1]=0x15;
sensor_read(i2cdev, address, &data);
subv=data;
if ((subv!=0x11)||(subh!=0x11)) subsampon=1;
if ((subv==0x31)&&(subh==0x31)) subsampon=2; //2x2
address[0]=0x38;
address[1]=0x21;
sensor_read(i2cdev, address, &data);
if (data&0x01) binon=1; // horizontal binning on?

csi_debug(3,"- subsampon:%u (2=2x2) binnOn:%u  valh:0x%02x valv:0x%02x\n",subsampon, binon, subh,subv);
}


/* AEC/AGC: this is in the analog function block!
+++ average-based algorithm +++

AEC = automatic exposure control
AGC = automatic gain control
..work together: 0x3503...
AWB = automatic white balance. 0x5001
ABF = automatic Band filter 50/60Hz. 0x3C01 . this seems to be disclosed.
ABLC = automatic black level calibration. 0x4000..


AEC/AGC control Registers:
0x3500 = AEC PK EXPOSURE 19:16
0x3501 = AEC PK EXPOSURE 15:8
0x3502 = AEC PK EXPOSURE 7:0
0x3503 = AEC AGC Mode Control, auto manual

0x350a = AEC PK REAL GAIN 9:8
0x350b = AEC PK REAL GAIN 7:0
0x350c = AEC PK VTS 15:8
0x350d = AEC PK VTS 7:0

--AEC/AGC power down domain control registers:
0x3a00 = aec option enable incl. nightmode
0x3A01 = Minimum Exposure Output Limit
0x3A02 = 60Hz Maximum Exposure Output Limit 15:8
0x3A03 = 60Hz Maximum Exposure Output Limit 7:0
0x3A04 = na
0x3A05 = AEC System Control 2, night mode frame insert etc.
0x3A06 = AEC System Control 3, step manual...
0x3A07 = AEC Manual Step Register
0x3A08 = 50Hz Band Width 9:8
0x3A09 = 50Hz Band Width 7:0
0x3A0a = 60Hz Band Width 13:8
0x3A0b = 60Hz Band Width 7:0
0x3A0c = Decimal line high limit zone, E1 max and min
0x3A0d = 60Hz Max Bands in One Frame 5:0
0x3A0e = 50Hz Max Bands in One Frame 5:0
0x3A0f = Stable Range High Limit (Enter)
0x3A10 = Stable Range Low Limit (Enter)
0x3A11 = Step Manual Mode, Fast Zone High Limit
.
0x3A13 = Pre-gain enable
0x3A14 = 50Hz Maximum Exposure Output Limit 15:8
0x3A15 = 50Hz Maximum Exposure Output Limit 7:0
.
0x3A17 = Gain Base When in Night Mode
0x3A18 = Gain Output Top Limit 9:8
0x3A19 = Gain Output Top Limit 7:0
0x3A1a = Difference minimal
0x3A1b = Stable Range High Limit (Go Out)
0x3A1c = Exposure Values Added When Strobe is On 15:8
0x3A1d = Exposure Values Added When Strobe is On 7:0
0x3A1e = Stable Range Low Limit (Go Out)
0x3A1f = Step Manual Mode, Fast Zone Low Limit
0x3A20 = Strobe option enable
0x3A21 = Insert frame number enable

0x3A25 = Freeze counter enable

*/
void setup_aec(void)
{
}


/* preview is possibly scaled image, automatic aec agc
capture is non-automatic aec agc.
so preview gathers information on current light and stuff,
then capture is using these values to set a static picture.
precondition: timing/framesize must have been written before!

void set_preview(void)
{
unsigned char address[REG_ADDR_STEP], data;
	unsigned long sysclk;
	unsigned band_step60, max_band60, band_step50, max_band50, hts, vts;

// aec agc to auto	
	address[0]=0x35;
	address[1]=0x03;
	data = 0;
	sensor_write(i2cdev, address, &data); 
//set_banding:	
	// read sysclock, PCLK 
	sysclk = get_pclk();
	
	// read HTS 
        address[0]=0x38;
        address[1]=0x0c;
    sensor_read(i2cdev, address, &data); 
	hts=data<<8;
        address[1]=0x0d;
    sensor_read(i2cdev, address, &data); 
	hts |= data;
	
	// read VTS 
        address[1]=0x0e;
    sensor_read(i2cdev, address, &data); 
	vts=data<<8;
        address[1]=0x0f;
    sensor_read(i2cdev, address, &data); 
	vts |= data;

//60Hz Maximum Exposure Output Limit = vts	
	address[0]=0x3a;
	address[1]=0x02;
	data = vts>>8;
	sensor_write(i2cdev, address, &data); 
	address[1]=0x03;
	data = vts;
	sensor_write(i2cdev, address, &data); 
//50Hz Maximum Exposure Output Limit = vts	
	address[1]=0x14;
	data = vts>>8;
	sensor_write(i2cdev, address, &data); 
	address[1]=0x15;
	data = vts;
	sensor_write(i2cdev, address, &data); 
// calculate banding filter 
	// 60Hz 
	band_step60 = sysclk * 100 / hts * 100 / 120;
//set 60Hz step size	
	address[0]=0x3a;
	address[1]=0x0a;
	data = band_step60>>8;
	sensor_write(i2cdev, address, &data); 
	address[1]=0x0b;
	data = band_step60;
	sensor_write(i2cdev, address, &data); 
	
		band_step50 = sysclk * 100 / hts;

//set 50Hz step size	
	address[1]=0x08;
	data = band_step50>>8;
	sensor_write(i2cdev, address, &data); 
	address[1]=0x09;
	data = band_step50;
	sensor_write(i2cdev, address, &data); 

	max_band50 = ((vts - 4) / band_step50);
//set 50Hz Max Bands in One Frame, 1 byte bit 5:0
	address[1]=0x0e;
	data = max_band50;
	sensor_write(i2cdev, address, &data); 


}
*/
/* capture mode assumes we hav run the camera in preview mode (autoexposure)
and now have settled light values available to be set in non-auto mode.
ie. the picture shall be stable from now on, no more changes.

void set_capture(void)
{
unsigned char address[REG_ADDR_STEP], data;
	unsigned long preview_shutter, sysclk;
	unsigned preview_gain16, luminance, hts, vts, banding, capture_bandingfilter, capture_max_band, capture_gain16_shutter;

// aec agc to manual ie. auto off	
	address[0]=0x35;
	address[1]=0x03;
	data = 0x03;
	sensor_write(i2cdev, address, &data); 
	
// get 20bit exposure value: AEC PK EXPOSURE	
        address[1]=0x00;
    sensor_read(i2cdev, address, &data); 
	preview_shutter=data&0x0f; // bits 19:16 on lower 4 bits
	preview_shutter<<=8;
        address[1]=0x01;
    sensor_read(i2cdev, address, &data); 
	preview_shutter |= data; // bits 15:8
        address[1]=0x02;
	preview_shutter<<=8;
    sensor_read(i2cdev, address, &data); 
	preview_shutter |= data; // bits 7:0
	
// get AEC PK REAL GAIN
        address[1]=0x0a;
    sensor_read(i2cdev, address, &data); 
	preview_gain16 = data; 
	preview_gain16<<=8;
        address[1]=0x0b;
    sensor_read(i2cdev, address, &data); 
	preview_gain16 |= data; 
// get average luminance ( High 8 bits of average value)
        address[0]=0x56;
        address[1]=0xa1;
    sensor_read(i2cdev, address, &data); 
	luminance = data; 
	
// turn off nightmode	
        address[0]=0x3a;
        address[1]=0x00;
    sensor_read(i2cdev, address, &data); 
	data &= 0xFB;	// night mode off, bit[2] = 0 
	sensor_write(i2cdev, address, &data); 
// get hts vts sysclock
	// read sysclock, PCLK 
	sysclk = get_pclk();
	
	// read HTS 
        address[0]=0x38;
        address[1]=0x0c;
    sensor_read(i2cdev, address, &data); 
	hts=data<<8;
        address[1]=0x0d;
    sensor_read(i2cdev, address, &data); 
	hts |= data;
	
	// read VTS 
        address[1]=0x0e;
    sensor_read(i2cdev, address, &data); 
	vts=data<<8;
        address[1]=0x0f;
    sensor_read(i2cdev, address, &data); 
	vts |= data;
	
// get banding filter value
        address[0]=0x3c;
        address[1]=0x01;
    sensor_read(i2cdev, address, &data); // get Band mode
	if (data & 0x80) //manual
	{
        address[1]=0x00;
    sensor_read(i2cdev, address, &data); // Band value manual setting
		if (data & 0x04) banding = 50;//50 hz
		else banding = 60;
	} 
	else //auto
	{
        address[1]=0x0c;
    sensor_read(i2cdev, address, &data); // Band value manual setting
		if (data & 0x01) banding = 50;//50 hz
		else banding = 60;
	}
// calc capture_bandingfilter
	if (banding == 60) 
		capture_bandingfilter = sysclk * 100 / hts * 100 / 120; // 60 Hz 
	 else  
	 capture_bandingfilter = sysclk * 100 / hts;
// calc capture_max_band
	capture_max_band = ((vts - 4) / capture_bandingfilter);
// calculate capture shutter/gain16
	capture_gain16_shutter = preview_gain16 * preview_shutter * sysclk;
	
// correct capture_gain16_shutter as to luminance value
	if (luminance > ae_low && luminance < ae_high) // in stable range
		capture_gain16_shutter =
		    capture_gain16_shutter / sysclk * hts / capture_hts * ae_target / luminance;
	 else //unstable
		capture_gain16_shutter =
		    capture_gain16_shutter / preview_sysclk * preview_hts / capture_hts;
	
	

}

*/

void auto_exposure(void) //trys to set everything automatic
{
unsigned char address[REG_ADDR_STEP], data;
address[0]=0x35;
address[1]=0x03;
data = 0;
sensor_write(i2cdev, address, &data); // aec agc to auto
address[0]=0x3c;
address[1]=0x01;
sensor_write(i2cdev, address, &data); // automatic banding 50/60Hz 0
//AEC System Control 0
address[0]=0x3a;
address[1]=0x00;
data = 0x7c; //0x78;
sensor_write(i2cdev, address, &data); // band function enable, night mode on =0x04
//AEC System Control 2
address[0]=0x3a;
address[1]=0x05;
data = 0x64;
sensor_write(i2cdev, address, &data); //  In night mode, insert frame ,Step auto mode,4
//night mode gain
address[0]=0x3a;
address[1]=0x17;
data = 0x11; // 0x11 = max
sensor_write(i2cdev, address, &data); 

}
/** calc fps
read AEC-VTS, VTS, HTS from the registers and calc pixelclock.
Then Framerate = pclk / ((vts_extra+vts) * hts);

exit: 0=error, else fps
*/
static int calc_fps(struct v4l2_subdev *sd)
{
    struct sensor_info *info = to_state(sd);
    unsigned long  pclk;
	unsigned int vts,hts,vts_extra, outheight, outwidth, ispheight, ispwidth, preoutwidth, preoutheight, offset, fps;
    unsigned char address[REG_ADDR_STEP], data;

//---------------- get timing for fps
        address[0]=0x35;

// get AEC VTS value if set.		
        address[1]=0x0C;
    sensor_read(i2cdev, address, &data);
	vts_extra = data<<8;
        address[1]=0x0D;
    sensor_read(i2cdev, address, &data);
    vts_extra |= data;

        address[0]=0x38;
		
        address[1]=0x0C;
    sensor_read(i2cdev, address, &data); // pclk per line  HTS
    hts = data<<8;
       address[1]=0x0D;
    sensor_read(i2cdev, address, &data);
    hts |= data;
 	
       address[1]=0x0E;
    sensor_read(i2cdev, address, &data); //number of lines  VTS
    vts = data<<8;
       address[1]=0x0F;
    sensor_read(i2cdev, address, &data);
    vts |= data;
	
	
//------------------ get isp window size as offset from origin. the end-values are allways odd!
       address[1]=0x00;
    sensor_read(i2cdev, address, &data);
	offset = data<<8;
       address[1]=0x01;
    sensor_read(i2cdev, address, &data);
    offset |= data; //x-start
	
       address[1]=0x04;
    sensor_read(i2cdev, address, &data);
	ispwidth = data<<8;
       address[1]=0x05;
    sensor_read(i2cdev, address, &data);
    ispwidth |= data+1;  //x-end //(1 to n adjust!)
	ispwidth -= offset;
//-
       address[1]=0x02;
    sensor_read(i2cdev, address, &data);
	offset = data<<8;
       address[1]=0x03;
    sensor_read(i2cdev, address, &data);
    offset |= data; //y-start
	
       address[1]=0x06;
    sensor_read(i2cdev, address, &data);
	ispheight = data<<8;
       address[1]=0x07;
    sensor_read(i2cdev, address, &data);
    ispheight |= data+1; //x-end //(1 to n adjust!)
	ispheight -= offset;
	

//------------------	get output size before scaling, as double offset from isp origin

       address[1]=0x10; // x-offset
    sensor_read(i2cdev, address, &data);
	offset = data<<8;
       address[1]=0x11;
    sensor_read(i2cdev, address, &data);
    offset |= data;  //x-offset
	offset *= 2;
	preoutwidth = ispwidth - offset;
	
       address[1]=0x12; //y-offset
    sensor_read(i2cdev, address, &data);
	offset = data<<8;
       address[1]=0x13;
    sensor_read(i2cdev, address, &data);
    offset |= data;  //y-offset
	offset *= 2;
	preoutheight = ispheight - offset;

//------------------get output size
        address[1]=0x08;
    sensor_read(i2cdev, address, &data); // height of output window
	outwidth = data<<8;
       address[1]=0x09;
    sensor_read(i2cdev, address, &data);
    outwidth |= data;
	
       address[1]=0x0a;
    sensor_read(i2cdev, address, &data); //width of output window
	outheight = data<<8;
       address[1]=0x0b;
    sensor_read(i2cdev, address, &data);
    outheight |= data;

    if((hts&&(vts+vts_extra)) == 0)
        return 0;
    pclk=get_pclk(sd);
     if (!pclk) 
	 {
	 csi_debug(2,"calc pclk failed\n");
	 return 0;  
	 }

    fps = pclk / ((vts_extra+vts) * hts);
   info->fps=fps;
csi_debug(2,"IspWidth:%u IspHeight:%u PoutWidht:%u PoutHeight:%u Outwidth;%u OutHeight:%u\n",ispwidth, ispheight,preoutwidth, preoutheight, outwidth, outheight);
csi_debug(2,"------------>HTS:%u VTS:%u EXTRA:%u\n",hts,vts,vts_extra);
    csi_debug(2,"***Framerate: FPS: %u Pixelclock:%lu\n",fps, pclk);

    return fps;
}


/*
precondition:  scaling is on, subsample is off

set output window size by using down-scaling from the whole physical sensor. full view
isp input window is set to max.
if aspect-ratio is 4:3 then just downsclaing is used.
if aspect is not 4:3, x or y isp offset is set to desired aspect ratio. then its downscaled from there.
no-we just downscale without aspect correction

We must use max HTS VTS because we need to read the full sensor.
If we would crob / zoom the picture we need only read a fraction of the sensor, thus changing timing HTS VTS possible.

input: desired window size
exit:_ 0=success, else error
*/
int scale_image(unsigned int outwidth, unsigned int outheight)
{
unsigned char address[REG_ADDR_STEP], data;
unsigned int xoffset, yoffset, ispx, ispy, hts, vts;
//unsigned long lval;

// 2592 * 1944 = isp max
ispx = 2592;  // isp input is the full frame/full view non crobbed.
ispy = 1944;
xoffset = 16; // the default offset for above full frame. these def values mask out the non visilbe sensor rows/colomns
yoffset = 4;
//hts,vts depends on the isp input size!  timing max=5fps
hts = 2844;
vts = 1968;



if ( (outwidth > ispx) || (outheight > ispy) || (outwidth < 40) || (outheight < 40) ) return 1; // error


// just skip aspect ratio, we may so have distorted pictures, thats fine.
/*
lval = 0;
// see if dividable by 4. we need aspect ratio 4:3
if ((outwidth % 4 != 0)||(outheight % 3 != 0)) 
{
lval = 1;
}
else // check if aspect is 4:3
{
	lval = outwidth / 4;
	lval *= 3;
	if (lval != outheight) // not 4:3
	{
	lval=1;
	}
	else lval =0;
}

if (lval) // need to crop input image to fit aspect of output for scaling correctly
{
lval = ispy * outwidth;
lval /= outheight;
if (lval > ispx) return 2; // invalid proportion
yoffset += (ispx - lval)/2; // just change yoffset to fit output aspect ratio for isp
}
*/
// ispx correction
ispx -=1; // must be odd for first window
ispx += 32; // add standard offset 2 * 16
// ispy correction
ispy -=1; // must be odd for first window
ispy += 8; // add standard offset 2 * 4

address[0]=0x38;
// set isp window full frame/ full view of camera
//isp x-start
address[1]=0x00;
data = 0;
sensor_write(i2cdev, address, &data);
address[1]=0x01;
data = 0;
sensor_write(i2cdev, address, &data);	
//isp x-end
address[1]=0x04;
data = ispx>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x05;
data = ispx; //(0 to n-1!)
sensor_write(i2cdev, address, &data);	

//isp y-start
address[1]=0x02;
data = 0;
sensor_write(i2cdev, address, &data);
address[1]=0x03;
data = 0;
sensor_write(i2cdev, address, &data);	
//isp y-end
address[1]=0x06;
data = ispy>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x07;
data = ispy; //(0 to n-1!)
sensor_write(i2cdev, address, &data);	

// set isp offset--not changed as we do just scale without aspect correction
// the isp offset is the position of top/left of output window.
//out x-offset
address[1]=0x10;
data = xoffset>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x11;
data = xoffset;
sensor_write(i2cdev, address, &data);	
//out y-offset
address[1]=0x12;
data = yoffset>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x13;
data = yoffset;
sensor_write(i2cdev, address, &data);	

//output window size---the scaled picture
address[1]=0x08;
data = outwidth>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x09;
data = outwidth;
sensor_write(i2cdev, address, &data);	
//out y
address[1]=0x0a;
data = outheight>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x0b;
data = outheight;
sensor_write(i2cdev, address, &data);	


// timing: HTS VTS to full.  pxclk per line, number of lines
address[1]=0x0c;
data = hts>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x0d;
data = hts;
sensor_write(i2cdev, address, &data);	
//out y
address[1]=0x0e;
data = vts>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x0f;
data = vts;
sensor_write(i2cdev, address, &data);	



// subsample off	
address[1]=0x14;
data = 0x11; // 0x11=off. 0x31 = on
sensor_write(i2cdev, address, &data);
address[1]=0x15;
sensor_write(i2cdev, address, &data);	
address[1]=0x21;
data=0x00; // binning on =0x01, off=0x00
sensor_write(i2cdev, address, &data);	

// SDE on(7), scale on(5), UV average off(2), CMX on(1), AWB on(0)
address[0]=0x50;
address[1]=0x01;
data = 0xa3;
sensor_write(i2cdev, address, &data);

return 0;
}

/*
This function creates a subwindow of given size. It zooms towards the center of the original picture.
This is done by chaning the ISP input size using registers 0 to 7.

scaling is off
subsample is off

Framerate HTS * VTS / pxclk: determines exposure and shutter.
1896 * 984 (1,92)= 15 fps
2844 * 1968 (1,44)= 5 fps

entry: 

- outwidth,outheight 	size of output window. The desired window size. This Subwindow is in the ISP-Input window.
						if either value is 0, output window is set to max resolution!!!
						This window is the Output of the ISP and does not influence FPS!
All values must be even!						

exit:_ 0=success, else error
*/
int zoom_image(unsigned int ispwidth, unsigned int ispheight, unsigned int outwidth, unsigned int outheight)
{
unsigned char address[REG_ADDR_STEP], data;
unsigned int ispx, ispy, outx, outy, maxx, maxy,hts,vts;
maxx = 2592; // max ISP input size = max active sensor area
maxy = 1944;
hts = 2844; // max sensor area including dark pixels.
vts = 1968;
address[0]=0x38;

// check for even values
ispx=1;
if ( (ispx&outwidth)||(ispx&outheight) ) return 1; //error


	
	if (!outwidth || !outheight)
	{
		outwidth = maxx;
		outheight = maxy;
	}


if ( (outwidth > maxx) || (outheight > maxy) ) return 1; // error


/* calc hts and vts for fastest FPS:
*/
if ((outwidth < maxx)||(outheight < maxy)) // if output size is smaller than biggest possible
{
//set HTS:
ispx = outwidth + 252; //minimum HREF offtime is 252
while (ispx%4) ispx++; // advance hts until dividable by 4
// known working hts values: 1422, 1896, 2133, 2844
if (ispx < 1422) //experiment shows, values under 1422 do not work, because of analog sensor readout timing as limitation
ispx=1422; // min limit.
if (ispx < hts) // max limit
hts = ispx;
// else take maxvalue set above

//set VTS:
ispy = outheight + (20232/hts) + 1;  //20232 = vsync predelay(5688)+vsync postdelay(14544). see datasheet
while (ispy%4) ispy++; // advance vts until dividable by 4
if (ispy < vts) 
vts = ispy;
// else take maxvalue set above

}


ispx = (maxx - outwidth)/2; //x-start of scanning, can be 0! /2 because either side.
ispy = (maxy - outheight)/2; //y-start of scanning

// set isp input window, this influences FPS range: 0 to n-1, so odd endsizes allways -------------------------
//isp x-start
address[1]=0x00;
data = ispx>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x01;
data = ispx;
sensor_write(i2cdev, address, &data);	
//isp x-end
ispx += outwidth-1; //(0 to n-1!)
address[1]=0x04;
data = ispx>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x05;
data = ispx; 
sensor_write(i2cdev, address, &data);	

//isp y-start
address[1]=0x02;
data = ispy>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x03;
data = ispy;
sensor_write(i2cdev, address, &data);	
//isp y-end
ispy += outheight-1; //(0 to n-1!)
address[1]=0x06;
data = ispy>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x07;
data = ispy; 
sensor_write(i2cdev, address, &data);	

// set output window offsets-------------------------we do not use default values as 16 and 4!
outx = 0;
outy = 0;


//set internal isp-offset, x-offset
address[1]=0x10;
data = outx>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x11;
data = outx;
sensor_write(i2cdev, address, &data);	
// y-offset
address[1]=0x12;
data = outy>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x13;
data = outy;
sensor_write(i2cdev, address, &data);	

//output window size
address[1]=0x08;
data = outwidth>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x09;
data = outwidth;
sensor_write(i2cdev, address, &data);	
//out y
address[1]=0x0a;
data = outheight>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x0b;
data = outheight;
sensor_write(i2cdev, address, &data);	

// set HTS and VTS
address[1]=0x0c;
data = hts>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x0d;
data = hts;
sensor_write(i2cdev, address, &data);
address[1]=0x0e;
data = vts>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x0f;
data = vts;
sensor_write(i2cdev, address, &data);

// subsample off	
address[1]=0x14;
data = 0x11;
sensor_write(i2cdev, address, &data);
address[1]=0x15;
sensor_write(i2cdev, address, &data);	
// scale off
// SDE on(7), scale off(5), UV average off(2), CMX on(1), AWB on(0): 
address[0]=0x50;
address[1]=0x01;
data = 0x83;
sensor_write(i2cdev, address, &data);

//auto_exposure();
return 0;
}





/** get current value of a control

struct v4l2_control
{
    __u32            id;
    __s32            value;
 }


@param sd = pointer to i2c subdevice
@param ctrl = pointer to control structure containing the control to set
@return 0=success, -EINVAL
*/
static int sensor_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    unsigned char data;

    csi_debug(2,"get control called:%u--\n",ctrl->id);

    switch (ctrl->id)
    {
    case V4L2_CID_COLORFX: // test pattern
        data = control.pattern;
        break;

    case V4L2_CID_EXPOSURE: //exposure modes
        data = control.autoexp;
        break;

    case V4L2_CID_AUTOGAIN: // autogain on/off
        data = control.autogain;
        break;

    case V4L2_CID_VFLIP:
        data = control.vflip;
        break;

    case V4L2_CID_HFLIP:
        data = control.hflip;
        break;


    case V4L2_CID_GAIN:
        data = control.gain;
        break;

    default:
        return -EINVAL;
    }
    ctrl->value=data;
    return (0);
}

/** set value of a control

Here all the controls we support are being set.
single register settings are done here without need of register list.

@param sd = pointer to i2c subdevice
@param ctrl = pointer to control structure containing the control with value
@return 0=success, -ERANGE, -EINVAL
*/
static int sensor_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
    unsigned char address[REG_ADDR_STEP];
    unsigned char data = 0;
    int ret=0;

    csi_debug(2,"set control called:%u--%u \n",ctrl->id, ctrl->value);

    data = ctrl->value;
    switch (ctrl->id)
    {
    case V4L2_CID_COLORFX: //testpattern output, 4 modes
        data = ctrl->value;
        control.pattern = data;
        if (data)
        {
            data -= 1;
            data |= 0x80;
        }
        address[0]=0x50;
        address[1]=0x3D;
        ret=sensor_write(i2cdev, address, &data);
        break;

    case V4L2_CID_EXPOSURE: //exposure modes, tbd..
        data = ctrl->value;
        control.autoexp = data;
        break;

    case V4L2_CID_AUTOGAIN: // autogain on/off
        control.autogain = ctrl->value;
        address[0]=0x3A;
        address[1]=0x00;
        ret=sensor_read(i2cdev, address, &data);
        if (ctrl->value) data |= 0x02;
        else data &= ~0x02;
        ret+=sensor_write(i2cdev, address, &data);
        break;

    case V4L2_CID_VFLIP: // picture flip
        control.vflip = ctrl->value;
        address[0]=0x38;
        address[1]=0x20;
        ret=sensor_read(i2cdev, address, &data);
        if (ctrl->value) data |= 0x02;
        else data &= ~0x02;
        ret+=sensor_write(i2cdev, address, &data);
        break;

    case V4L2_CID_HFLIP:
        control.hflip = ctrl->value;
        address[0]=0x38;
        address[1]=0x21;
        ret=sensor_read(i2cdev, address, &data);
        if (ctrl->value) data |= 0x02;
        else data &= ~0x02;
        ret+=sensor_write(i2cdev, address, &data);
        break;


    case V4L2_CID_GAIN:  //manual gain adjust
        control.gain = ctrl->value;
        address[0]=0x55;
        address[1]=0x86;
        data=ctrl->value;
        ret=sensor_write(i2cdev, address, &data);
        break;

    default:
        return -EINVAL;
    }

    if (ret) csi_debug(3,"+++++++++error in %s\n",__func__);
    return 0;
}

/** query the avalability of a control and if supported,
fill in min, max, step and default value for these controls.
see include/linux/videodev2.h for details

EINVAL if control not supported.

For the ov5640, the following controls are supported:
Horizontal flip: 0x3821 Bit1: 1=on; 0=off; preread value
Vertical flip:   0x3820 Bit1: 1=on; 0=off; preread value

contrast,  is done by enabling digital effects:
0x5001 byte_value: 0=off (no adjustment possible) 0x01=on adjustment possible.(***digital effects, default=1)
if on:
brightness: 0x5587 byte_value: 0 - 255, 127=middle  (***Y-bright)
contrast:   0x5586 byte_value: 0 - 255, 127=middle (***Y-gain)  works!
saturation: not yet done


night_mode: supports long exposure times depend on light, changes frame rate.(***nightmode,default=0)
night_mode: 0x3A00 bit2: 1=on; 0=off preread value

auto_gain is used for enable/disable digital effects. set contrast, brightness, saturation.
auto_exposure is used to enable/disable night mode.

Testpattern:
mode 0 = test pattern off. ie. normal picture!
mode 1 = eight color bar
mode 2 = gradual change at vertical
mode 3 = gradual change at horizontal
mode 4 = gradual change at vertical and horizontal

automatic exposure options :  tbd...
- LAEC = quick lightchange response bit-6
- auto Banding(Flicker) bit 5
- less 1 band = quick change flicker bit 4
- nightmode  bit2
0x3A00:
default: 0x78

 ***autogain AGC:
 0x3503 bit 1  = manual gain enable manual
 0x350A = gain hibyte (only bits 0-1)  so its a 10 bit value.
 0x350B = gain low byte

To init the range of the control  v4l2_ctrl_query_fill is used:
v4l2_ctrl_query_fill(pointer to struct, min_val, max_val, stepsize, default)
*/
static int sensor_queryctrl(struct v4l2_subdev *sd,
                            struct v4l2_queryctrl *qc)
{
    csi_debug(2,"query-control called--\n");
    switch (qc->id)
    {
    case V4L2_CID_COLORFX: // test pattern
        v4l2_ctrl_query_fill(qc, 0, 4, 1, 0);
        qc->type = V4L2_CTRL_TYPE_INTEGER;
        return 0;

    case V4L2_CID_EXPOSURE: //exposure modes
        v4l2_ctrl_query_fill(qc, 0, 4, 1, 1);
        qc->type = V4L2_CTRL_TYPE_INTEGER;
        return 0;
    case V4L2_CID_AUTOGAIN:   // autogain on/off
        v4l2_ctrl_query_fill(qc, 0, 1, 1, 1);
        qc->type = V4L2_CTRL_TYPE_INTEGER;
        return 0;

    case V4L2_CID_VFLIP:
    case V4L2_CID_HFLIP:
        v4l2_ctrl_query_fill(qc, 0, 1, 1, 0);
        qc->type = V4L2_CTRL_TYPE_BOOLEAN;
        return 0;

    case V4L2_CID_GAIN:
        v4l2_ctrl_query_fill(qc, 0, 128, 1, 64);
        qc->flags = V4L2_CTRL_FLAG_SLIDER;
        return 0;

    default:
        break;
    }
    csi_debug(2,"--unknown control--\n");
    return -EINVAL;
}


/** get framerate

The Camera does not support a fixed frame rate.
Framerate depends on the windowsize and pixelclock and colorformat and exposure.
So...ni specific Framerate !!!
We tell, that we are using 30 fps frames per second.
*/
static int sensor_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
    struct v4l2_captureparm *cp = &parms->parm.capture;
    struct sensor_info *info = to_state(sd);

    if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    memset(cp, 0, sizeof(struct v4l2_captureparm));
    cp->capability = V4L2_CAP_TIMEPERFRAME;
    cp->timeperframe.numerator = 1;

    if (info->width > SVGA_WIDTH && info->height > SVGA_HEIGHT)
    {
        cp->timeperframe.denominator = SENSOR_FRAME_RATE/2;
    }
    else
    {
        cp->timeperframe.denominator = SENSOR_FRAME_RATE;
    }

    return 0;
}

/** set framerate, we not support
*/
static int sensor_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *parms)
{
// not support setting framerate.
    return -EINVAL;
}


// +++:++++++++++++++++++++ everything below here should be camera-modul independant ***********************



// +++:++++++++++++++++++++  v4l2_subdev_video_ops sensor_video_ops  ***************************


// this has exta pointers for the found entrys in format and size table.
static int sensor_try_fmt_internal(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt, struct sensor_format_struct **ppformat,
                                   struct sensor_win_size **ppsize)
{

    int index;
//    struct sensor_win_size *psize;

    // try to find the wanted format in our format table
    for (index = 0; index < ARRAY_SIZE(sensor_formats); index++)
        if (sensor_formats[index].fourcc == fmt->code)
            break;

    // if nothing found, we take the first format, which should be the best for the camera anyway.
    if (index >= ARRAY_SIZE(sensor_formats))
    {
        index = 0;
    }




    // set the found pointers as info for the caller
    if (ppformat != NULL)
        *ppformat = sensor_formats + index;
		
	// we do not use the first 2 parameters!!
	if (zoom_image(0,0,fmt->width,fmt->height)) return 	-EINVAL;	// full size max
//	if (scale_image(fmt->width,fmt->height)) return 	-EINVAL;	// full size max
 

    // update the information to be returned
    fmt->code = sensor_formats[index].fourcc;
    fmt->field = sensor_formats[index].bitsperpixel;  //missuse field as bpp!
    return 0;
}



/** v4l2_subdev_video_ops - sensor_enum_fmt
index = the wanted index in the format table
Caller tells us the wanted index. He wants the fourcc-code and a description.
We get the  index and address of u32 code.
We go into the format table and copy the fourcc to *code.
*/
static int sensor_enum_fmt(struct v4l2_subdev *sd, unsigned index,
                           enum v4l2_mbus_pixelcode *code)
{

    if (index >= ARRAY_SIZE(sensor_formats))
        return -EINVAL;

    *code = sensor_formats[index].fourcc;
    return 0;
}


/**
The Caller wants a best matching working format to his suggestion.
He gives us fourcc, width, height.
We return the best matching fourcc,width, height, bitsperpixel.
-
we get info in struct v4l2_mbus_framefmt.
.width = width
.height = hight
.code = fourcc
.field = bitsperpixel

We return the best matching format in those 4 fileds.
actually we misuse field for bitsperpixel! the csi driver knows.
*/
static int sensor_try_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
    return sensor_try_fmt_internal(sd, fmt, NULL, NULL);
}


/** set cameras video format and picture size
Caller wants us to set this format: widht, height, fourcc-code, bytesperline, sizeimage, colorspace.
we as subdev get  info in struct v4l2_mbus_framefmt.
.width = width
.height = hight
.code = fourcc
.field = bitsperpixel

We transmit the relevant format and windowsize registers to the camera via i2c.
We store the current format and windowsize in camera struct as working format.
colorspace not supported.
*/
static int sensor_s_fmt(struct v4l2_subdev *sd, struct v4l2_mbus_framefmt *fmt)
{
    int ret;
    struct sensor_format_struct *pformat;
    struct sensor_win_size *psize;
    struct csi_camera *pcam=(struct csi_camera *)dev_get_drvdata(sd->v4l2_dev->dev);

    csi_debug(2,"sensor_s_fmt\n");

    // we call try-fmt again here(user should have done that before), just to make sure!, also we get the pointers we need.
    ret = sensor_try_fmt_internal(sd, fmt, &pformat, &psize);
    if (ret)
        return ret; // no match found


    


    //update videoinfo in camera struct, csi driver
    pcam->fourcc = pformat->fourcc;
    pcam->width = fmt->width;
    pcam->height = fmt->height;
    pcam->bitsperpixel = pformat->bitsperpixel;
    pcam->bytesperline = (pformat->bitsperpixel * fmt->width)>>3;
    pcam->frame_size = pcam->bytesperline * fmt->height;
    pcam->csi_mode.input_fmt = pformat->csi_input;
    pcam->csi_mode.output_fmt = pformat->csi_output;
    pcam->csi_mode.seq = pformat->byte_order;

    // update the information to be returned
    fmt->code = pformat->fourcc;
    fmt->field = pformat->bitsperpixel; //missusing field as bpp!

	
    msleep(100);
	calc_fps(sd);

    return 0;
}


// +++:++++++++++++++++++++  v4l2_subdev_core_ops sensor_core_ops  ***************************
/** verify that camera module is actually the specified one.
it checks, if the given i2c address responds.
does not check the camera register id code.!

may need local function here!
*/
static int sensor_g_chip_ident(struct v4l2_subdev *sd,
                               struct v4l2_dbg_chip_ident *chip)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    csi_debug(2,"chip ident called--\n");

    return v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_SENSOR, 0);
}



/** v4l2_subdev_core_ops
*/
static int sensor_reset(struct v4l2_subdev *sd, u32 val)
{
    struct csi_camera *pcam=(struct csi_camera *)dev_get_drvdata(sd->v4l2_dev->dev);
    struct sensor_info *info = to_state(sd);
    char csi_reset_str[32];
    csi_debug(2,"sensor reset called--\n");

    if(info->csi_sig_cfg->csi_port == 0)
    {
        strcpy(csi_reset_str,"csi_reset");
    }
    else if(info->csi_sig_cfg->csi_port == 1)
    {
        strcpy(csi_reset_str,"csi_reset_b");
    }

    switch(val)
    {
    case CSI_SUBDEV_RST_OFF:
        csi_debug(1,"CSI_SUBDEV_RST_OFF\n");
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
        msleep(10);
        break;
    case CSI_SUBDEV_RST_ON:
        csi_debug(1,"CSI_SUBDEV_RST_ON\n");
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_RST_ON,csi_reset_str);
        msleep(10);
        break;
    case CSI_SUBDEV_RST_PUL:
        csi_debug(1,"CSI_SUBDEV_RST_PUL\n");
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
        msleep(10);
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_RST_ON,csi_reset_str);
        msleep(100);
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
        msleep(10);
        break;
    default:
        return -EINVAL;
    }

    return 0;
}

/** check sensor id
read the sensors id and compare to given id
*/
static int sensor_detect(struct v4l2_subdev *sd)
{
    int ret;
    struct regval_list regs;

    regs.reg_num[0] = 0x30;
    regs.reg_num[1] = 0x0A;
    ret = sensor_read(sd, regs.reg_num, regs.value);
    if (ret < 0)
    {
        csi_debug(3,"sensor_read err at sensor_detect!\n");
        return ret;
    }

    if(regs.value[0] != 0x56)
        return -ENODEV;

    return 0;
}


/** v4l2_subdev_core_ops - camera_init
called when camera is opened, after poweron.
*/
static int camera_init(struct v4l2_subdev *sd, u32 val)
{
    int ret;
    csi_debug(2,"camera init called--check ID\n");

    /*Make sure it is a target sensor*/
    ret = sensor_detect(sd);
    if (ret)
    {
        csi_debug(3,"chip found is not an target chip.\n");
        return ret;
    }
// here we init the camera with all values needed for all formats. format specific stuff is transfered on top of this.
   return sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
}




/** camera power and standby management
camtest:
gpio_read_one_pin_value(u32 p_handler, "csi_d1")  beispiel um pinio auf dem csi zu machen.
during power-on reset must be applied
standby on disables i2c interface!!
@param struct v4l2_subdev *sd = specify the i2c device to be used
@param on:
- CSI_SUBDEV_STBY_ON = reset_off, mclk_on, outputs_off, standby_on, standby_off, standby_on, mclk_off
- CSI_SUBDEV_STBY_OFF= mclk_on, reset_off, reset_on, reset_off, standby_off
- CSI_SUBDEV_PWR_ON  = mclk_off, standby_on, reset_on, power_on, .....
- CSI_SUBDEV_PWR_OFF.....review this!!!!!!!!!!!!!!
@return  -EINVAL=command not supported; 0=success
*/
static int sensor_power(struct v4l2_subdev *sd, int on)
{
    struct csi_camera *pcam=(struct csi_camera *)dev_get_drvdata(sd->v4l2_dev->dev);
    struct sensor_info *info = to_state(sd);
    char csi_stby_str[32],csi_power_str[32],csi_reset_str[32];

    csi_debug(2,"sensor power called--\n");

    if(info->csi_sig_cfg->csi_port == 0)
    {
        strcpy(csi_stby_str,"csi_stby");
        strcpy(csi_power_str,"csi_power_en");
        strcpy(csi_reset_str,"csi_reset");
    }
    else if(info->csi_sig_cfg->csi_port == 1)
    {
        strcpy(csi_stby_str,"csi_stby_b");
        strcpy(csi_power_str,"csi_power_en_b");
        strcpy(csi_reset_str,"csi_reset_b");
    }

    switch(on)
    {
    case CSI_SUBDEV_STBY_ON:
        csi_debug(1,"CSI_SUBDEV_STBY_ON\n");
//       gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_STBY_ON,csi_stby_str); // in \linux-sunxi-lemaker-3.4\arch\arm\plat-sunxi/sys_config.c
        msleep(10);
        break;
    case CSI_SUBDEV_STBY_OFF:
        csi_debug(1,"CSI_SUBDEV_STBY_OFF\n");
//       gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_STBY_OFF,csi_stby_str); // no i2c possible if standby!!. so we leave it on.
        msleep(10);
        break;
    case CSI_SUBDEV_PWR_ON:
        csi_debug(1,"CSI_SUBDEV_PWR_ON\n");
//activate output pins for reset and standby
        gpio_set_one_pin_io_status(pcam->csi_pin_hd,1,csi_stby_str);//set the gpio to output
        gpio_set_one_pin_io_status(pcam->csi_pin_hd,1,csi_reset_str);//set the gpio to output

        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_STBY_ON,csi_stby_str); //standby and reset on during powerup!
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_RST_ON,csi_reset_str);

        //turn power supply on
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_PWR_ON,csi_power_str);
        msleep(10);
        if(pcam->dvdd)
        {
            regulator_enable(pcam->dvdd);
            msleep(10);
        }
        if(pcam->avdd)
        {
            regulator_enable(pcam->avdd);
            msleep(10);
        }
        if(pcam->iovdd)
        {
            regulator_enable(pcam->iovdd);
            msleep(10);
        }
        //active mclk
        clk_enable(pcam->csi_module_clk);
        msleep(10);
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_STBY_OFF,csi_stby_str);
        msleep(10);
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_RST_OFF,csi_reset_str);
        msleep(10);
        break;

    case CSI_SUBDEV_PWR_OFF:
        csi_debug(1,"CSI_SUBDEV_PWR_OFF\n");
		showinfo();
        //power supply off
        if(pcam->iovdd)
        {
            regulator_disable(pcam->iovdd);
            msleep(10);
        }
        if(pcam->avdd)
        {
            regulator_disable(pcam->avdd);
            msleep(10);
        }
        if(pcam->dvdd)
        {
            regulator_disable(pcam->dvdd);
            msleep(10);
        }
        gpio_write_one_pin_value(pcam->csi_pin_hd,CSI_PWR_OFF,csi_power_str);
        msleep(10);

        //inactive mclk after power off
        clk_disable(pcam->csi_module_clk);
        //disable outputs for reset and standby
        gpio_set_one_pin_io_status(pcam->csi_pin_hd,0,csi_reset_str);//set the gpio to input
        gpio_set_one_pin_io_status(pcam->csi_pin_hd,0,csi_stby_str);//set the gpio to input
        break;
    default:
        return -EINVAL;
    }

    return 0;
}


/** v4l2_subdev_core_ops - sensor_ioctl
*/
static long sensor_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
    int ret=0;
    csi_debug(2,"sensor ioctl called--\n");

    switch(cmd)
    {
    case CSI_SUBDEV_CMD_GET_INFO:
    {
        struct sensor_info *info = to_state(sd);
        struct csi_signal_config *csi_sig_cfg = arg;

        csi_debug(1,"CSI_SUBDEV_CMD_GET_INFO\n");

        csi_sig_cfg->mclk 	=	info->csi_sig_cfg->mclk ;
        csi_sig_cfg->vref 	=	info->csi_sig_cfg->vref ;
        csi_sig_cfg->href 	=	info->csi_sig_cfg->href ;
        csi_sig_cfg->clock	=	info->csi_sig_cfg->clock;
        csi_sig_cfg->csi_port	=	info->csi_sig_cfg->csi_port;

        csi_debug(1,"csi_sig_cfg.mclk=%d\n ",info->csi_sig_cfg->mclk);
        csi_debug(1,"csi_sig_cfg.vref=%d\n ",info->csi_sig_cfg->vref);
        csi_debug(1,"csi_sig_cfg.href=%d\n ",info->csi_sig_cfg->href);
        csi_debug(1,"csi_sig_cfg.clock=%d\n ",info->csi_sig_cfg->clock);
        csi_debug(1,"csi_sig_cfg.csi_port=%d\n ",info->csi_sig_cfg->csi_port);
        break;
    }
    case CSI_SUBDEV_CMD_SET_INFO:
    {
        struct sensor_info *info = to_state(sd);
        struct csi_signal_config *csi_sig_cfg = arg;

        csi_debug(1,"CSI_SUBDEV_CMD_SET_INFO\n");

        info->csi_sig_cfg->mclk 	=	csi_sig_cfg->mclk 	;
        info->csi_sig_cfg->vref 	=	csi_sig_cfg->vref 	;
        info->csi_sig_cfg->href 	=	csi_sig_cfg->href 	;
        info->csi_sig_cfg->clock	=	csi_sig_cfg->clock	;
        info->csi_sig_cfg->csi_port	=	csi_sig_cfg->csi_port	;

        csi_debug(1,"csi_sig_cfg.mclk=%d\n ",info->csi_sig_cfg->mclk);
        csi_debug(1,"csi_sig_cfg.vref=%d\n ",info->csi_sig_cfg->vref);
        csi_debug(1,"csi_sig_cfg.href=%d\n ",info->csi_sig_cfg->href);
        csi_debug(1,"csi_sig_cfg.clock=%d\n ",info->csi_sig_cfg->clock);
        csi_debug(1,"csi_sig_cfg.csi_port=%d\n ",info->csi_sig_cfg->csi_port);

        break;
    }
    default:
        csi_debug(1,"--unknown ioctl--\n");
        return -EINVAL;
    }
    return ret;
}



/* --------------------------- Subdevice Driver Interface -------------------------------------------- */

/** v4l2_subdev_core_ops: (this is a lemaker special version in \include\media\v4l2-subdev.h)
subdevice core operations:
supported:
- g_chip_ident  read chip id from camera
- g_ctrl		get control value
- s_ctrl		set control value
- queryctrl		enumerate controls
- reset    generic reset command. The argument selects which subsystems to reset.
			Passing 0 will always reset the whole chip. Do not use for new drivers without
			discussing this first on the linux-media mailinglist.
			There should be no reason normally to reset a device.
- init	    initialize the sensor registers to some sort of reasonable default values.
			Do not use for new drivers and should be removed in existing drivers.
- s_power    puts subdevice in power saving mode (on == 0) or normal operation mode (on == 1).
- ioctl    called at the end of ioctl() syscall handler at the V4L2 core.
			used to provide support for private ioctls used on the driver.
*/
static const struct v4l2_subdev_core_ops sensor_core_ops =
{
    .g_chip_ident = sensor_g_chip_ident,
    .g_ctrl = sensor_g_ctrl,
    .s_ctrl = sensor_s_ctrl,
    .queryctrl = sensor_queryctrl,
    .reset = sensor_reset,
    .init = camera_init,
    .s_power = sensor_power,
    .ioctl = sensor_ioctl,
};

/** v4l2_subdev_video_ops    (this is a lemaker special version in \include\media\v4l2-subdev.h)
subdevice video operations:
- enum_mbus_fmt		enumerate  pixel formats if this camera
- try_mbus_fmt		try to set a pixel format, choose best fit
- s_mbus_fmt		set a pixel format
- s_parm			Streaming parameters are intended to optimize the video capture process as well as I/O.
					Presently applications can request a high quality capture mode with the VIDIOC_S_PARM ioctl.
- g_parm			get Streaming parameters
*/
static const struct v4l2_subdev_video_ops sensor_video_ops =
{
    .enum_mbus_fmt = sensor_enum_fmt,
    .try_mbus_fmt = sensor_try_fmt,
    .s_mbus_fmt = sensor_s_fmt,
    .s_parm = sensor_s_parm,
    .g_parm = sensor_g_parm,
};

/**
subdevice all operations:
*/
static const struct v4l2_subdev_ops sensor_ops =
{
    .core = &sensor_core_ops,
    .video = &sensor_video_ops,
};

// +++:++++++++++++++++++++  Subdev Driver functions ***************************
// this is called when the ov5640 driver is loaded . modprobe.
static int sensor_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct v4l2_subdev *sd;
    struct sensor_info *info;
//	int ret;
    csi_debug(2,"Subdev Driver Probe\n");

    info = kzalloc(sizeof(struct sensor_info), GFP_KERNEL);
    if (info == NULL)
        return -ENOMEM;

    sd = &info->sd;
    // open i2c interface and get a handle to it(sd).
    v4l2_i2c_subdev_init(sd, client, &sensor_ops);
    i2cdev = sd; //save i2c device handle .tomk
    // set default values:

    info->fmt = &sensor_formats[0];
    info->csi_sig_cfg = &ccm_info_con;

    info->brightness = 0;
    info->contrast = 0;
    info->saturation = 0;
    info->hue = 0;
    info->hflip = 0;
    info->vflip = 0;
    info->gain = 0;
    info->autogain = 1;
    info->exp = 0;
    info->autoexp = 0;
    info->autowb = 1;
    info->wb = 0;
// framerate stuff
	info->fps=0;
	info->pxclk=0;

    return 0;
}

// this is called when the camera driver ov5640 is removed. rmmod.
static int sensor_remove(struct i2c_client *client)
{
    struct v4l2_subdev *sd = i2c_get_clientdata(client);
    csi_debug(2,"Subdev Driver Remove\n");

    v4l2_device_unregister_subdev(sd);
    kfree(to_state(sd));
    return 0;
}

static const struct i2c_device_id sensor_id[] =
{
    { "ov5640", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, sensor_id);


static struct i2c_driver sensor_driver =
{
    .driver = {
        .owner = THIS_MODULE,
        .name = "ov5640",
    },
    .probe = sensor_probe,
    .remove = sensor_remove,
    .id_table = sensor_id,
};


static __init int subdev_init(void)
{
    csi_debug(2,"Subdev Driver Init\n");
    return i2c_add_driver(&sensor_driver);
}

static __exit void subdev_exit(void)
{
    csi_debug(2,"Subdev Driver Exit\n");

    i2c_del_driver(&sensor_driver);
}

module_init(subdev_init);
module_exit(subdev_exit);

