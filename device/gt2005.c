/*  autofocus test
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

#include "regs.h"

// +++:++++++++++++++++++++  Sensor Register Value Lists for color formats  ***************************


static struct regval_list sensor_fmt_yuv422_yuyv[] =
{

    {{0x43,	0x00} , {0x30}}	//YUYV
};



// +++:++++++++++++++++++++  Sensor Register Value Lists for Control settings  ***************************

static struct regval_list start_af[] =
{

    {{0x30,	0x00} , {0x10}},	//enable function blocks
    {{0x30,	0x01} , {0x08}},	//clear reset
    {{0x30,	0x04} , {0xEF}},	//enable all clocks
    {{0x30,	0x05} , {0xF7}},	//enable all clocks
    {{0x30,	0x08} , {0x02}},	//clear reset
    {{0x3f,	0x00} , {0x00}},	//run af
    {{0x30,	0x22} , {0x04}},	//contin af mode
    {{0x30,	0x23} , {0x01}},	//contin af mode
};


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
static struct sensor_win_size
{
    int	width;
    int	height;
    // register list
    struct regval_list *regs;
    int regs_size;
} sensor_win_sizes[] =
{

    {
        .width			= 2592,
        .height 		= 1944,
        .regs			= sensorBIG,
        .regs_size	= ARRAY_SIZE(sensorBIG),
    },

    {
        .width			= 640,
        .height 		= 480,
        .regs			= sensorVGA,
        .regs_size	= ARRAY_SIZE(sensorVGA),
    },
 
};


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

/** calc fps


exit: 0=error, else fps
*/
static int calc_fps(struct v4l2_subdev *sd)
{
    struct sensor_info *info = to_state(sd);
    unsigned char low, high;
    unsigned long  pclk;
	unsigned int vts,hts,vts_extra, outheight, outwidth, ispheight, ispwidth, preoutwidth, preoutheight, offset, fps;
    unsigned char address[REG_ADDR_STEP];

//---------------- get timing for fps
        address[0]=0x38;
        address[1]=0x0C;
    sensor_read(i2cdev, address, &high); // pclk per line
        address[0]=0x38;
        address[1]=0x0D;
    sensor_read(i2cdev, address, &low);
    hts = high * 256 + low;
 	
        address[0]=0x38;
        address[1]=0x0E;
    sensor_read(i2cdev, address, &high); //number of lines
        address[0]=0x38;
        address[1]=0x0F;
    sensor_read(i2cdev, address, &low);
   vts = high * 256 + low;
	
        address[0]=0x35;
        address[1]=0x0C;
    sensor_read(i2cdev, address, &high);
        address[0]=0x35;
        address[1]=0x0D;
    sensor_read(i2cdev, address, &low);
    vts_extra = high * 256 + low;
//------------------ get isp window size as offset from origin
        address[0]=0x38;
        address[1]=0x00;
    sensor_read(i2cdev, address, &high);
        address[0]=0x38;
        address[1]=0x01;
    sensor_read(i2cdev, address, &low);
    offset = high * 256 + low;  //x-start
	
        address[0]=0x38;
        address[1]=0x04;
    sensor_read(i2cdev, address, &high);
        address[0]=0x38;
        address[1]=0x05;
    sensor_read(i2cdev, address, &low);
    ispwidth = high * 256 + low;  //x-end
	ispwidth -= offset;
//-
        address[0]=0x38;
        address[1]=0x02;
    sensor_read(i2cdev, address, &high);
        address[0]=0x38;
        address[1]=0x03;
    sensor_read(i2cdev, address, &low);
    offset = high * 256 + low;  //x-start
	
        address[0]=0x38;
        address[1]=0x06;
    sensor_read(i2cdev, address, &high);
        address[0]=0x38;
        address[1]=0x07;
    sensor_read(i2cdev, address, &low);
    ispheight = high * 256 + low;  //x-end
	ispheight -= offset;
	

//------------------	get output size before scaling, as double offset from isp origin

        address[0]=0x38;
        address[1]=0x10;
    sensor_read(i2cdev, address, &high);
        address[0]=0x38;
        address[1]=0x11;
    sensor_read(i2cdev, address, &low);
    offset = high * 256 + low;  //x-offset
	offset *= 2;
	preoutwidth = ispwidth - offset;
	
        address[0]=0x38;
        address[1]=0x12;
    sensor_read(i2cdev, address, &high);
        address[0]=0x38;
        address[1]=0x13;
    sensor_read(i2cdev, address, &low);
    offset = high * 256 + low;  //y-offset
	offset *= 2;
	preoutheight = ispheight - offset;

//------------------get output size
       address[0]=0x38;
        address[1]=0x08;
    sensor_read(i2cdev, address, &high); // height of output window
        address[0]=0x38;
        address[1]=0x09;
    sensor_read(i2cdev, address, &low);
    outwidth = high * 256 + low;
	
        address[0]=0x38;
        address[1]=0x0a;
    sensor_read(i2cdev, address, &high); //width of output window
        address[0]=0x38;
        address[1]=0x0b;
    sensor_read(i2cdev, address, &low);
    outheight = high * 256 + low;

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

    if (parms->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    memset(cp, 0, sizeof(struct v4l2_captureparm));
    cp->capability = V4L2_CAP_TIMEPERFRAME;
    cp->timeperframe.numerator = 1;

        cp->timeperframe.denominator = SENSOR_FRAME_RATE;

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
    struct sensor_win_size *psize;

    // try to find the wanted format in our format table
    for (index = 0; index < ARRAY_SIZE(sensor_formats); index++)
        if (sensor_formats[index].fourcc == fmt->code)
            break;

    // if nothing found, we take the first format, which should be the best for the camera anyway.
    if (index >= ARRAY_SIZE(sensor_formats))
    {
        index = 0;
    }


    // we take the window size, that is smaller or equal to the wanted one.
    for (psize = sensor_win_sizes; psize < sensor_win_sizes + (ARRAY_SIZE(sensor_win_sizes)); psize++)
        if (fmt->width >= psize->width && fmt->height >= psize->height)
            break;

    // if no match found, take the smallest one
    if (psize >= sensor_win_sizes + (ARRAY_SIZE(sensor_win_sizes)))
        psize--;

    // set the found pointers as info for the caller
    if (ppformat != NULL)
        *ppformat = sensor_formats + index;
    if (ppsize != NULL)
        *ppsize = psize;


    // update the information to be returned
    fmt->width = psize->width;
    fmt->height = psize->height;
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
    unsigned char address[REG_ADDR_STEP], data;
	

    csi_debug(1,"sensor_s_fmt\n");

    // we call try-fmt again here(user should have done that before), just to make sure!, also we get the pointers we need.
    ret = sensor_try_fmt_internal(sd, fmt, &pformat, &psize);
    if (ret)
        return ret; // no match found

    // set camera matched windowsize . (if actually one found, yeah always!)
    if (psize->regs)
    {
        ret = sensor_write_array(sd, psize->regs , psize->regs_size);
        if (ret < 0)
            return ret;
    }

    // set the camera videoformat as to best found.
    //sensor_write_array(sd, pformat->regs , pformat->regs_size);

    //update videoinfo in camera struct, csi driver
    pcam->fourcc = pformat->fourcc;
    pcam->width = psize->width;
    pcam->height = psize->height;
    pcam->bitsperpixel = pformat->bitsperpixel;
    pcam->bytesperline = (pformat->bitsperpixel * psize->width)>>3;
    pcam->frame_size = pcam->bytesperline * psize->height;
    pcam->csi_mode.input_fmt = pformat->csi_input;
    pcam->csi_mode.output_fmt = pformat->csi_output;
    pcam->csi_mode.seq = pformat->byte_order;

    // update the information to be returned
    fmt->width = psize->width;
    fmt->height = psize->height;
    fmt->code = pformat->fourcc;
    fmt->field = pformat->bitsperpixel; //missusing field as bpp!

    sensor_write_array(sd, start_af , ARRAY_SIZE(start_af));
	

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
   ret=sensor_write_array(sd, sensor_default_regs , ARRAY_SIZE(sensor_default_regs));
   if (ret) 
        csi_debug(3,"failed write defaultregs\n");  
   ret=sensor_write_array(sd, af_software , ARRAY_SIZE(af_software));
   if (ret) 
        csi_debug(3,"failed write AF_software\n");  
		
   return 0;
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

