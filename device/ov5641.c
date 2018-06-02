/** A V4L2 driver for ov5640 camera modules.
 Intro:
 This driver talks to the camera via the i2c bus. (to set/get camera parameters)
 It also talks to the gpio-pin_io-module for IO-lines for Reset/Standby/etc, the powermodule to set sensor_power,
 and CSI_clock_module to set the camera-clock.
 It does not touch the CSI Bus interface! (All done in the csi-module driver)

 Videoformats on the cameras 8-bit parallel Bus:
 The Video-Bus-Dataformat was specified as "enum v4l2_mbus_pixelcode".
 NOTE: v4l2_mbus_pixelcode is depreciated (linus torvald) and  MEDIA_BUS_FMT_ definitions in media-bus-format.h should be used.
 For our parallel interface there is only one MediaBusFormat: V4L2_MBUS_PARALLEL (parallel interface with hsync and vsync)
 We use the Video Format as FOURCC code to be reported to the driver.
 It tells how video data is coded and how many bits per pixel are used.
 The data then passes through the CSI module.

 The CSI Module:
 Data is captured into a fifo-buffer. Its format can be RAW(any Camera format) or YUV422(default).
 Data can be optionally converted before it leaves the CSI_module out to the memory.
 Possible output conversions are:
 PassThrough = no conversion. mostly used. Input must be set to RAW!
 When input is set to YUV422(default), the following conversions are possible:
 - planar YUV422P (default)
 - planar YUV420P
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
 So the max.Framesize can be max: 4096*4096 Bytes.
 Depending on the Videoformat one Pixel has a specific Bitlength (depth) of information. common values are 8, 12 or 16 bits per pixel.
 So the max. Pixel per line is reduced to: 4096*8/Bitlength); for 8bit=4096; for 12bit=2730, for 16bit=2048 (pixel per line)

 V4L2 Media Bus Video Frame Format: (V4L2 Video Information fields)
 * struct v4l2_mbus_framefmt - frame format on the media bus
 * @width:	frame width in pixel (one line)
 * @height:	frame height in number of lines
 * @code:	data format code (from enum v4l2_mbus_pixelcode)
 * @field:	specifys interlacing type (from enum v4l2_field), we use: V4L2_FIELD_NONE (fields are not interlaced)
 * @colorspace:	colorspace of the data (from enum v4l2_colorspace)
 * @ycbcr_enc:	YCbCr encoding of the data (from enum v4l2_ycbcr_encoding)
 * @quantization: quantization of the data (from enum v4l2_quantization)
 * @xfer_func:  transfer function of the data (from enum v4l2_xfer_func)

 Common Frame Sizes and Framerates:
  at 24MHZ input clock, the outputted pixelclock is fe. 48MHZ, thus pumping out 48-million Bytes per second.!!
  At two Bytes per Pixel, we can pump out 24 million pixel per second.
  Format      Framesize/Pixel   Framerate  (24-million / Framesize*2 = Frames per Second)
  =====================================
 320 * 240    76800        156
 640 * 480    307200       39
 1280 * 720   921600       13
 1920 * 1080  2073600       5
 1280 * 960   1228800       9
 2592 * 1944  5038840		2

 But Framerate also depends on exposure time and internal logic limitations.
 
In practice, pixelclocks over 48MHZ do not really work over the CSI interface.

 OV5640 default format after reset:
 2590*1944,
Bayer RAW10(it bypasses the internal format controler)
 The sensor-array uses Bayer color pattern!
 
 Hardware Interface:
 D0-D7 = parallel dataport
 HREF = horizontal sync (one pulse per line)
 VREF = vertical/picture sync (one pulse per frame)

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
MODULE_DESCRIPTION("V4L2-Sub_Device Driver for the OV5640 CAM Module");
MODULE_LICENSE("GPL");


//for internel driver debug, prints kernel debug messages to the console or logfile.
#define DBG   		1
//debug level 0~3: 3=only errors,2=+ routine trace, 1=+ details, 0= + interupt messages
#define DBG_LEV 	2

//for internel driver debug
#if(DBG==1)
#define csi_debug(l,x,arg...) if(l >= DBG_LEV) printk(KERN_CRIT"[-----OV5640]"x,##arg)
#else
#define csi_debug(l,x,arg...)
#endif


#define MCLK (24*1000*1000)	  // camera input clock from CSI
#define VREF_POL	CSI_LOW  //act. low during frame capture
#define HREF_POL	CSI_HIGH //act. high during line capture
#define CLK_POL		CSI_RISING //latch Byte-data at rising edge of pixelclock

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

/*Sensor init values for DVP parallel port (not the Mipi port!) 
Note, some registers are undocumented and taken AS IS from example code supplied by omnivision.
*/
// subsampling from 1280x960 binning 2x2, 648x484 with dummy
static struct regval_list sensor_init_regs[] =  //default: VGA 640x480, 15fps, subsample On, Scaling On
{
    {{0x30,0x08},{0x42}},	//stop streaming during register setting
//***** HW settings	
    {{0x31,0x03},{0x03}},   //clock source from pll.must be 0x03 for unkown reason (datasheet incorrect)
    {{0x30,0x17},{0xff}}, //output pins enable:vsync,href,pclk,d9-d7
    {{0x30,0x18},{0xff}}, //output pins enable:d0-d5
    {{0x30,0x2c},{0xc2}}, //bit[7:6]: output drive capability:00: 1x   01: 2x  10: 3x  11: 4x
    {{0x30,0x0e},{0x58}}, //DVP enable, (default:0x58): 0x58 = 0101 1000 = mipi off, mipiPhy=off; mipiPhy=powerdown, DVP parallel port=0n
    {{0x46,0x0b},{0x35}}, // debug mode??DVP setting
    {{0x46,0x0c},{0x20}}, // DVP PCLK divider (default:0x20) (VFIFO control)
    {{0x48,0x37},{0x22}}, // PCLK Period (default:0x10)
    {{0x38,0x24},{0x02}}, //DVP PCLK divider value
    {{0x30,0x2e},{0x00}}, // ??
//***** Module enables/config
    {{0x30,0x00},{0x10}}, //enable function blocks (0 = enable),disbale OTP. default:0x30
    {{0x30,0x01},{0x08}}, //enable function blocks, no s2p  default:0x08
    {{0x30,0x02},{0x1c}}, // enable function blocks, ,vfifo,format,average. default:0x1C
    {{0x30,0x04},{0xef}}, //enable  clocks, no otp default:0xCF
    {{0x30,0x05},{0xF7}}, //enable clocks, no s2p. default:0xF7
    {{0x30,0x06},{0xE3}}, // enable clocks,  .default:0xE3
    {{0x30,0x07},{0xE7}}, //enable clocks, no mipi. default:0xff

    {{0x43,0x00},{0x30}}, // Format Control: YUV422-YUYV. (default:0xF8)
    {{0x50,0x1f},{0x00}}, // Format Mux Control:ISP YUV422
	
    {{0x50,0x00},{0xa7}}, //optimisation enables, lenc[7],gamma[5],bpc[2],  wpc[1],cip[0]
	// CIP to get color, gamma to get brightness).
//Scaling enable!!	
    {{0x50,0x01},{0xa7}}, //>>>optimisation enables,  SDE[7], Scaling[5],   UVav[2],CMX[1],AWB[0]
	//AWB+CMX=important,else colors wrong, UVaverage gives better color contrast.was 0xa3
	
    {{0x50,0x03},{0x08}}, //optimisation enables,  bin???[2], draw afc window[1]
    {{0x50,0x25},{0x00}}, //????
	
//***** banding filter: 50/60Hz detector control 
    {{0x3c,0x00},{0x04}}, //50/60 Hz manual Band select ( Band50 default value). default:0x00
    {{0x3c,0x01},{0x34}}, // band manual enable[7], Sum auto mode enable[5], Band counter enable[4], Counter threshold[3-0] for band change = 4
//- bandautodetect(3c01_7) = if on(0), will detect 50 or 60 hz light frequency automaticly. default:0x00

    {{0x3c,0x04},{0x28}},//5060HZ CTRL04:Threshold for low sum default:0x20
    {{0x3c,0x05},{0x98}},//5060HZ CTRL05: Threshold for high sum. default:0x70
    {{0x3c,0x06},{0x00}},// Lightmeter1 threshold[15:8] default: 0x0000
    {{0x3c,0x07},{0x08}},// Lightmeter1 threshold[7:0]
    {{0x3c,0x08},{0x00}},// Lightmeter2 threshold[15:8] default:0x012c
    {{0x3c,0x09},{0x1c}},// Lightmeter2 threshold[7:0]
    {{0x3c,0x0a},{0x9c}},// Sample number[15:8] default:0x4e1f
    {{0x3c,0x0b},{0x40}},// Sample number[7:0]
	
//***** Exposure Gain Nightmode Banding
//control:
    {{0x35,0x03},{0x00}}, // AEC AGC auto enable(0). AECauto[0], AGCauto[1] default: 0x00 = both enabled
//nightmode+banding:	
    {{0x3a,0x00},{0x78}},  //AEC Band Night: BandFunctionOn[5], NightModeOn[2] default:0x78=BandingOn
//- bandauto(3aA00_5) = auto banding, if on, aec is done in steps of "band steps". if off, aec is continuoues.

    {{0x3a,0x05},{0x30}}, //NightModeInsertFrames[6] , exposureStepAuto[5],step auto-ratio can be set in bits 0-4.default:0x30

    {{0x3a,0x17},{0x01}}, //[1:0]   Gain night threshold = gain value when in nightmode
//    {{0x3a,0x02},{0x03}},// night mode ceiling or 16bit max_exposure60hz (max exposure when banding is used)
//    {{0x3a,0x03},{0xd8}},// only this is night-ceiling!. default: 15744
	
//    {{0x3a,0x14},{0x03}},// AEC Max exposure 50Hz = 16bit max_exposure50hz(max exposure when banding is used)
//    {{0x3a,0x15},{0xd8}},// AEC Max exposure 50Hz. default: 3648

    {{0x3a,0x08},{0x01}},// 16bit 50hz band step width, is set approx to VTS automaticly, maybe need to set extra framelines as well.
    {{0x3a,0x09},{0x27}},// banding steps 50 width. default:0x0127
    {{0x3a,0x0a},{0x00}},// 16bit 60hz band step width
    {{0x3a,0x0b},{0xf6}},// banding steps 60Hz width. default:0x00f6
    {{0x3a,0x0e},{0x03}},// 50Hz Max Bands in One Frame. default:0x06
    {{0x3a,0x0d},{0x04}},// 60Hz Max Bands in One Frame. default: 0x08

	// aec automatic exposure control range: here you define what exposure/brightness you want.
    {{0x3a,0x0f},{48}},  //48.Stable Range High Limit (enter),  stable region . this is relevant for auto aec!
    {{0x3a,0x10},{40}},  //40.Stable Range Low Limit (enter),
    {{0x3a,0x1b},{48}},  //48.control Range High Limit (go out), control region . control should be bigger than stable
    {{0x3a,0x1e},{38}},  //38.control Range Low Limit (go out) if yavg within this range, no change, else will adjust as to stable range.
	
    {{0x3a,0x11},{0x60}}, // Fast Zone High Limit
    {{0x3a,0x1f},{0x14}}, // Fast Zone Low Limit
//Gain:	
    {{0x3a,0x13},{0x43}},// pre-gain enable[6], pregain value[5-0]. default:0x40 = pregain is on with gain = 1x
//    {{0x3a,0x18},{0x00}},// AEC gain ceiling[9:8] default:0x03e0 = max allowed gain 
//default max test!    {{0x3a,0x19},{0xf8}},// AEC gain ceiling[7:0]
	
//***** subsample and binning  (Scaling see above)
    {{0x38,0x14},{0x31}},// horizontal subsampling on
    {{0x38,0x15},{0x31}},// vertical subsampling on = 2x2 binning, reduces picturesize by 4
    {{0x38,0x21},{0x01}}, // horizontal binning on[0] =1  for 2x2.default:0x00, hbinninOn=0x01
	
//***** Blacklevel Control BLC  (auto enable=0x4002 = 0x45=default)
    {{0x40,0x01},{0x02}},// BLC control1: BLC starting line = 2. default:0x00
    {{0x40,0x04},{0x02}},// BLC control4: BLC line number to use

//*** JPEG stuff -- V4L2_PIX_FMT_JPEG
    {{0x44,0x07},{0x04}}, // quantization scale ie. quality, lower numbers = better qualtiy

// PLL Settings = Pixelclock:
//    {{0x30,0x39},{0x80}}, //test: bypass pll, very slow 1fps, so we need pll!
    {{0x30,0x34},{0x1a}},//(6-4)=charge pump loop filter. (3-0)=BITdivider. 1a=10bit(default), 18=8bit(a bit faster!)
    {{0x30,0x35},{0x21}},//(7-4)=SystemClockDivider. (3-0)=MIPIdivider(must stay at 1!). default=0x11
    {{0x30,0x36},{0x46}},//(7-0)=Multipiler. default=0x69.  0x46->30fps
    {{0x30,0x37},{0x13}},//(4)=Rootdivider. (3-0)=Predivider.  default=0x03.
    {{0x31,0x08},{0x01}},//(5-4)=PCLK Rootdivider=0;(3-2)=sclk2x root divider=dontcare; (1-0)=SCLK root divider; default:0x16.


//window size = 640 X 480
    {{0x38,0x00},{0x00}},//start x =0
    {{0x38,0x01},{0x00}},//
    {{0x38,0x02},{0x00}},//start y = 4
    {{0x38,0x03},{0x04}},//
    {{0x38,0x04},{0x0a}},//end x = 2623
    {{0x38,0x05},{0x3f}},//
    {{0x38,0x06},{0x07}},//end y = 1947
    {{0x38,0x07},{0x9b}},//

    {{0x38,0x08},{0x02}},//  640
    {{0x38,0x09},{0x80}},//
    {{0x38,0x0a},{0x01}},// 480
    {{0x38,0x0b},{0xe0}},//

    {{0x38,0x0c},{0x07}},// 1896. 0x0768 hts
    {{0x38,0x0d},{0x68}},//
    {{0x38,0x0e},{0x03}},// 984- 0x03d8 vts
    {{0x38,0x0f},{0xd8}},//

    {{0x38,0x10},{0x00}},//offsett 16
    {{0x38,0x11},{0x10}},//
    {{0x38,0x12},{0x00}},//
    {{0x38,0x13},{0x06}},// 6
//window size end



//*****  various omnivision custom init settings from here **************************

// undocumented registers, taken AS IS: (possibly 50/60hz detection custum settings, or AF coil related)
    {{0x36,0x30},{0x36}},// these are OPTIONAL
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

//undocumented settings related to color and pixel stuff:	
    {{0x36,0x00},{0x08}},// debug for AF coil??
    {{0x36,0x01},{0x33}},//
    {{0x30,0x2d},{0x60}},//system setting undocumented
    {{0x36,0x20},{0x52}},//
    {{0x37,0x1b},{0x20}},//
    {{0x47,0x1c},{0x50}},//
	
    {{0x36,0x35},{0x13}},// from here must be>>>>>
    {{0x36,0x36},{0x03}},// ??mustbe
    {{0x36,0x34},{0x40}},//
    {{0x36,0x22},{0x01}},//

    {{0x36,0x18},{0x00}},// ??mustbe
    {{0x36,0x12},{0x29}},// ??must be
    {{0x37,0x08},{0x64}},// ??
    {{0x37,0x09},{0x52}},// ??
    {{0x37,0x0c},{0x03}},// to here must be <<<<<

//+++  image sensor processor ISP digital functions +++
// ISP: Automatic whitebalance AWB settings:
    {{0x51,0x80},{0xff}},//  these are OPTIONAL
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
	
    {{0x51,0x93},{0x70}}, //red limit
    {{0x51,0x94},{0xf0}}, //green limit
    {{0x51,0x95},{0xf0}}, //blue limit
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

//ISP: color Matrix CMX (cancel out crosstalk and convert color space, convert  RGB to YUV )
    {{0x53,0x81},{0x1e}}, // color matrix enable.  these are OPTIONAL
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
//ISP: Gamma Control: ( compensate for the non-linear characteristics of the sensor)
    {{0x54,0x80},{0x01}}, // important
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
	
//ISP: SDE Control, special digital effects ( hue/saturation control, brightness, contrast)	
    {{0x55,0x80},{0x04}},// these are OPTIONAL
    {{0x55,0x87},{0x05}},
    {{0x55,0x88},{0x09}},
    {{0x55,0x83},{0x40}},
    {{0x55,0x84},{0x10}},
    {{0x55,0x89},{0x10}},
    {{0x55,0x8a},{0x00}},
    {{0x55,0x8b},{0xf8}},
	
//ISP: Lens Control register, lens correction values:	(these are custom values supplied by omnivision for specific customer, so take as is)
    {{0x58,0x00},{0x3D}},// these are OPTIONAL
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

//-----
    {{0x30,0x08},{0x02}}, //poweron at end. start sensor streaming
};

// full resolution (16 dummy pixel horizontal, 8 dummy lines vertical), 2608x1952 with dummy
static struct regval_list sensor_qsxga_regs[] =   //qsxga: 2592*1944, subsample Off, Scaling Off
{
    {{0x30,0x08},{0x42}},	//stop streaming during register setting

//Scaling off!!	
    {{0x50,0x01},{0x87}}, //>>>optimisation enables,  SDE[7], Scaling[5],   UVav[2],CMX[1],AWB[0]
//***** subsample and binning off (Scaling see above)
    {{0x38,0x14},{0x11}},// horizontal subsampling on
    {{0x38,0x15},{0x11}},// vertical subsampling on = 2x2 binning, reduces picturesize by 4
    {{0x38,0x21},{0x00}}, // horizontal binning on[0] =1  for 2x2.default:0x00, hbinninOn=0x01

 // PLL Settings = Pixelclock:
    {{0x30,0x34},{0x1a}},//(6-4)=charge pump loop filter. (3-0)=BITdivider. 1a=10bit(default), 18=8bit(a bit faster!)
    {{0x30,0x35},{0x21}},//(7-4)=SystemClockDivider. (3-0)=MIPIdivider(must stay at 1!). default=0x11
    {{0x30,0x36},{0x46}},//(7-0)=Multipiler. default=0x69.  0x46->30fps
    {{0x30,0x37},{0x13}},//(4)=Rootdivider. (3-0)=Predivider.  default=0x03.
    {{0x31,0x08},{0x01}},//(5-4)=PCLK Rootdivider=0;(3-2)=sclk2x root divider=dontcare; (1-0)=SCLK root divider; default:0x16.

 
//window size = 2592x1944 : 
    {{0x38,0x00},{0x00}},//start x =0
    {{0x38,0x01},{0x00}},//
    {{0x38,0x02},{0x00}},//start y = 8
    {{0x38,0x03},{0x08}},//
    {{0x38,0x04},{0x0a}},//end x = 2623
    {{0x38,0x05},{0x3f}},//
    {{0x38,0x06},{0x07}},//end y = 1963
    {{0x38,0x07},{0xab}},//

    {{0x38,0x08},{0x0a}},//  2592
    {{0x38,0x09},{0x20}},//
    {{0x38,0x0a},{0x07}},// 1944
    {{0x38,0x0b},{0x98}},//

    {{0x38,0x0c},{0x0b}},// HTS 2844
    {{0x38,0x0d},{0x1c}},//
    {{0x38,0x0e},{0x07}},// VTS  1968
    {{0x38,0x0f},{0xb0}},//

    {{0x38,0x10},{0x00}},//offsett 16
    {{0x38,0x11},{0x10}},//
    {{0x38,0x12},{0x00}},//
    {{0x38,0x13},{0x06}},// 6
//window size end


	
    {{0x30,0x08},{0x02}}, //poweron at end. start sensor streaming
	
};


// cropping from full resolution , 1936x1088 with dummy pixels
static struct regval_list sensor_1080p_regs[] =   //1080: 1920*1080 //pclk:90m 15fps, subsample Off, Scaling Off
{
    { {0x31,0x03}, {0x11}
    },
//{{0x30,0x08},{0x82}},
    {{0x30,0x08},{0x42}},
    {{0x31,0x03},{0x03}},
    {{0x30,0x17},{0xff}},
    {{0x30,0x18},{0xff}},
    {{0x30,0x34},{0x1a}},
//{{0x30,0x35},{0x11}},
    {{0x30,0x35},{0x21}},
    {{0x30,0x36},{0x69}},


    {{0x30,0x2c},{0xc2}},//bit[7:6]: output drive capability
    //00: 1x   01: 2x  10: 3x  11: 4x

    {{0x30,0x37},{0x13}},
    {{0x31,0x08},{0x01}},//pclk div = 2
    {{0x36,0x30},{0x36}},
    {{0x36,0x31},{0x0e}},
    {{0x36,0x32},{0xe2}},
    {{0x36,0x33},{0x12}},
    {{0x36,0x21},{0xe0}},
    {{0x37,0x04},{0xa0}},
    {{0x37,0x03},{0x5a}},
    {{0x37,0x15},{0x78}},
    {{0x37,0x17},{0x01}},
    {{0x37,0x0b},{0x60}},
    {{0x37,0x05},{0x1a}},
    {{0x39,0x05},{0x02}},
    {{0x39,0x06},{0x10}},
    {{0x39,0x01},{0x0a}},
    {{0x37,0x31},{0x12}},
    {{0x36,0x00},{0x08}},
    {{0x36,0x01},{0x33}},
    {{0x30,0x2d},{0x60}},
    {{0x36,0x20},{0x52}},
    {{0x37,0x1b},{0x20}},
    {{0x47,0x1c},{0x50}},
    {{0x3a,0x13},{0x43}},
    {{0x3a,0x18},{0x00}},
    {{0x3a,0x19},{0xf8}},
    {{0x36,0x35},{0x13}},
    {{0x36,0x36},{0x03}},
    {{0x36,0x34},{0x40}},
    {{0x36,0x22},{0x01}},
    {{0x3c,0x01},{0x34}},
    {{0x3c,0x04},{0x28}},
    {{0x3c,0x05},{0x98}},
    {{0x3c,0x06},{0x00}},
    {{0x3c,0x07},{0x07}},
    {{0x3c,0x08},{0x00}},
    {{0x3c,0x09},{0x1c}},
    {{0x3c,0x0a},{0x9c}},
    {{0x3c,0x0b},{0x40}},
    {{0x38,0x14},{0x11}},
    {{0x38,0x15},{0x11}},
	
    {{0x38,0x00},{0x01}}, //336
    {{0x38,0x01},{0x50}},
    {{0x38,0x02},{0x01}}, //434
    {{0x38,0x03},{0xb2}},
    {{0x38,0x04},{0x08}}, //2287 = 1952 x
    {{0x38,0x05},{0xef}},
    {{0x38,0x06},{0x05}}, //1521 = 1088 y
    {{0x38,0x07},{0xf1}},
	
    {{0x38,0x08},{0x07}}, //1920
    {{0x38,0x09},{0x80}},
    {{0x38,0x0a},{0x04}}, //1080
    {{0x38,0x0b},{0x38}},
	
    {{0x38,0x0c},{0x09}}, //2500
    {{0x38,0x0d},{0xc4}}, 
    {{0x38,0x0e},{0x04}}, //1120
    {{0x38,0x0f},{0x60}},
    {{0x38,0x10},{0x00}},
    {{0x38,0x11},{0x10}},
    {{0x38,0x12},{0x00}},
    {{0x38,0x13},{0x04}},
	
    {{0x36,0x18},{0x04}},
    {{0x36,0x12},{0x2b}},
    {{0x37,0x08},{0x64}},
    {{0x37,0x09},{0x12}},
    {{0x37,0x0c},{0x00}},
    {{0x3a,0x02},{0x04}},
    {{0x3a,0x03},{0x60}},
    {{0x3a,0x08},{0x01}},
    {{0x3a,0x09},{0x50}},
    {{0x3a,0x0a},{0x01}},
    {{0x3a,0x0b},{0x18}},
    {{0x3a,0x0e},{0x03}},
    {{0x3a,0x0d},{0x04}},
    {{0x3a,0x14},{0x04}},
    {{0x3a,0x15},{0x60}},
    {{0x40,0x01},{0x02}},
    {{0x40,0x04},{0x06}},
    {{0x30,0x00},{0x00}},
    {{0x30,0x02},{0x1c}},
    {{0x30,0x04},{0xff}},
    {{0x30,0x06},{0xc3}},
    {{0x30,0x0e},{0x58}},
    {{0x30,0x2e},{0x00}},
    {{0x43,0x00},{0x30}},
    {{0x50,0x1f},{0x00}},
    {{0x47,0x13},{0x02}},
    {{0x44,0x07},{0x04}},
    {{0x44,0x0e},{0x00}},
    {{0x46,0x0b},{0x37}},
    {{0x46,0x0c},{0x20}},
    {{0x48,0x37},{0x16}},
    {{0x38,0x24},{0x04}},
    {{0x50,0x00},{0xa7}},
    {{0x50,0x01},{0x83}},
    {{0x51,0x80},{0xff}},
    {{0x51,0x81},{0xf2}},
    {{0x51,0x82},{0x00}},
    {{0x51,0x83},{0x14}},
    {{0x51,0x84},{0x25}},
    {{0x51,0x85},{0x24}},
    {{0x51,0x86},{0x09}},
    {{0x51,0x87},{0x09}},
    {{0x51,0x88},{0x09}},
    {{0x51,0x89},{0x75}},
    {{0x51,0x8a},{0x54}},
    {{0x51,0x8b},{0xe0}},
    {{0x51,0x8c},{0xb2}},
    {{0x51,0x8d},{0x42}},
    {{0x51,0x8e},{0x3d}},
    {{0x51,0x8f},{0x56}},
    {{0x51,0x90},{0x46}},
    {{0x51,0x91},{0xf8}},
    {{0x51,0x92},{0x04}},
    {{0x51,0x93},{0x70}},
    {{0x51,0x94},{0xf0}},
    {{0x51,0x95},{0xf0}},
    {{0x51,0x96},{0x03}},
    {{0x51,0x97},{0x01}},
    {{0x51,0x98},{0x04}},
    {{0x51,0x99},{0x12}},
    {{0x51,0x9a},{0x04}},
    {{0x51,0x9b},{0x00}},
    {{0x51,0x9c},{0x06}},
    {{0x51,0x9d},{0x82}},
    {{0x51,0x9e},{0x38}},
    {{0x53,0x81},{0x1e}},
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
    {{0x55,0x80},{0x02}},
    {{0x55,0x83},{0x40}},
    {{0x55,0x84},{0x10}},
    {{0x55,0x89},{0x10}},
    {{0x55,0x8a},{0x00}},
    {{0x55,0x8b},{0xf8}},
    {{0x58,0x00},{0x23}},
    {{0x58,0x01},{0x14}},
    {{0x58,0x02},{0x0f}},
    {{0x58,0x03},{0x0f}},
    {{0x58,0x04},{0x12}},
    {{0x58,0x05},{0x26}},
    {{0x58,0x06},{0x0c}},
    {{0x58,0x07},{0x08}},
    {{0x58,0x08},{0x05}},
    {{0x58,0x09},{0x05}},
    {{0x58,0x0a},{0x08}},
    {{0x58,0x0b},{0x0d}},
    {{0x58,0x0c},{0x08}},
    {{0x58,0x0d},{0x03}},
    {{0x58,0x0e},{0x00}},
    {{0x58,0x0f},{0x00}},
    {{0x58,0x10},{0x03}},
    {{0x58,0x11},{0x09}},
    {{0x58,0x12},{0x07}},
    {{0x58,0x13},{0x03}},
    {{0x58,0x14},{0x00}},
    {{0x58,0x15},{0x01}},
    {{0x58,0x16},{0x03}},
    {{0x58,0x17},{0x08}},
    {{0x58,0x18},{0x0d}},
    {{0x58,0x19},{0x08}},
    {{0x58,0x1a},{0x05}},
    {{0x58,0x1b},{0x06}},
    {{0x58,0x1c},{0x08}},
    {{0x58,0x1d},{0x0e}},
    {{0x58,0x1e},{0x29}},
    {{0x58,0x1f},{0x17}},
    {{0x58,0x20},{0x11}},
    {{0x58,0x21},{0x11}},
    {{0x58,0x22},{0x15}},
    {{0x58,0x23},{0x28}},
    {{0x58,0x24},{0x46}},
    {{0x58,0x25},{0x26}},
    {{0x58,0x26},{0x08}},
    {{0x58,0x27},{0x26}},
    {{0x58,0x28},{0x64}},
    {{0x58,0x29},{0x26}},
    {{0x58,0x2a},{0x24}},
    {{0x58,0x2b},{0x22}},
    {{0x58,0x2c},{0x24}},
    {{0x58,0x2d},{0x24}},
    {{0x58,0x2e},{0x06}},
    {{0x58,0x2f},{0x22}},
    {{0x58,0x30},{0x40}},
    {{0x58,0x31},{0x42}},
    {{0x58,0x32},{0x24}},
    {{0x58,0x33},{0x26}},
    {{0x58,0x34},{0x24}},
    {{0x58,0x35},{0x22}},
    {{0x58,0x36},{0x22}},
    {{0x58,0x37},{0x26}},
    {{0x58,0x38},{0x44}},
    {{0x58,0x39},{0x24}},
    {{0x58,0x3a},{0x26}},
    {{0x58,0x3b},{0x28}},
    {{0x58,0x3c},{0x42}},
    {{0x58,0x3d},{0xce}},
    {{0x50,0x25},{0x00}},
    {{0x3a,0x0f},{0x30}},
    {{0x3a,0x10},{0x28}},
    {{0x3a,0x1b},{0x30}},
    {{0x3a,0x1e},{0x26}},
    {{0x3a,0x11},{0x60}},
    {{0x3a,0x1f},{0x14}},
    {{0x30,0x08},{0x02}},
    {{0x30,0x35},{0x21}},
};



// cropping 2592x1944 to 2560x1440, subsampling in vertical and horizontal binning 2x2, 1296x728 with dummy
static struct regval_list sensor_720p_regs[] =   //1280*720, subsample On, Scaling Off
{
    { {0x31,0x03}, {0x11}
    },
//	{{0x30,0x08},{0x82}},
    {{0x30,0x08},{0x42}},
    {{0x31,0x03},{0x03}},
    {{0x30,0x17},{0xff}},
    {{0x30,0x18},{0xff}},
    {{0x30,0x34},{0x1a}},
    //{{0x30,0x35},{0x11}},////
    {{0x30,0x35},{0x21}},////

    {{0x30,0x2c},{0xc2}},//bit[7:6]: output drive capability
    //00: 1x   01: 2x  10: 3x  11: 4x

    {{0x30,0x36},{0x69}},
    {{0x30,0x37},{0x13}},
    {{0x31,0x08},{0x01}},//pclk div = 1
    {{0x36,0x30},{0x36}},
    {{0x36,0x31},{0x0e}},
    {{0x36,0x32},{0xe2}},
    {{0x36,0x33},{0x12}},
    {{0x36,0x21},{0xe0}},
    {{0x37,0x04},{0xa0}},
    {{0x37,0x03},{0x5a}},
    {{0x37,0x15},{0x78}},
    {{0x37,0x17},{0x01}},
    {{0x37,0x0b},{0x60}},
    {{0x37,0x05},{0x1a}},
    {{0x39,0x05},{0x02}},
    {{0x39,0x06},{0x10}},
    {{0x39,0x01},{0x0a}},
    {{0x37,0x31},{0x12}},
    {{0x36,0x00},{0x08}},
    {{0x36,0x01},{0x33}},
    {{0x30,0x2d},{0x60}},
    {{0x36,0x20},{0x52}},
    {{0x37,0x1b},{0x20}},
    {{0x47,0x1c},{0x50}},
    {{0x3a,0x13},{0x43}},
    {{0x3a,0x18},{0x00}},
    {{0x3a,0x19},{0xf8}},
    {{0x36,0x35},{0x13}},
    {{0x36,0x36},{0x03}},
    {{0x36,0x34},{0x40}},
    {{0x36,0x22},{0x01}},
    {{0x3c,0x01},{0x34}},
    {{0x3c,0x04},{0x28}},
    {{0x3c,0x05},{0x98}},
    {{0x3c,0x06},{0x00}},
    {{0x3c,0x07},{0x07}},
    {{0x3c,0x08},{0x00}},
    {{0x3c,0x09},{0x1c}},
    {{0x3c,0x0a},{0x9c}},
    {{0x3c,0x0b},{0x40}},
	
    {{0x38,0x14},{0x31}},
    {{0x38,0x15},{0x31}},
    {{0x38,0x00},{0x00}},
    {{0x38,0x01},{0x00}},
    {{0x38,0x02},{0x00}},
    {{0x38,0x03},{0xfa}},
    {{0x38,0x04},{0x0a}},
    {{0x38,0x05},{0x3f}},
    {{0x38,0x06},{0x06}},
    {{0x38,0x07},{0xa9}},
    {{0x38,0x08},{0x05}},
    {{0x38,0x09},{0x00}},
    {{0x38,0x0a},{0x02}},
    {{0x38,0x0b},{0xd0}},
    {{0x38,0x0c},{0x07}},
    {{0x38,0x0d},{0x64}},
    {{0x38,0x0e},{0x02}},
    {{0x38,0x0f},{0xe4}},
    {{0x38,0x10},{0x00}},
    {{0x38,0x11},{0x10}},
    {{0x38,0x12},{0x00}},
    {{0x38,0x13},{0x04}},
	
    {{0x36,0x18},{0x00}},
    {{0x36,0x12},{0x29}},
    {{0x37,0x08},{0x64}},
    {{0x37,0x09},{0x52}},
    {{0x37,0x0c},{0x03}},
    {{0x3a,0x02},{0x02}},
    {{0x3a,0x03},{0xe4}},
    {{0x3a,0x08},{0x01}},
    {{0x3a,0x09},{0xbc}},
    {{0x3a,0x0a},{0x01}},
    {{0x3a,0x0b},{0x72}},
    {{0x3a,0x0e},{0x01}},
    {{0x3a,0x0d},{0x02}},
    {{0x3a,0x14},{0x02}},
    {{0x3a,0x15},{0xe4}},
    {{0x40,0x01},{0x02}},
    {{0x40,0x04},{0x02}},
	
    {{0x30,0x00},{0x00}},
    {{0x30,0x02},{0x1c}},
    {{0x30,0x04},{0xff}},
    {{0x30,0x06},{0xc3}},
    {{0x30,0x0e},{0x58}},
    {{0x30,0x2e},{0x00}},
	
    {{0x43,0x00},{0x30}},
    {{0x50,0x1f},{0x00}},
    {{0x47,0x13},{0x02}},
    {{0x44,0x07},{0x04}},
    {{0x44,0x0e},{0x00}},
    {{0x46,0x0b},{0x37}},
    {{0x46,0x0c},{0x20}},
    {{0x48,0x27},{0x16}},
    {{0x38,0x24},{0x04}},
    {{0x50,0x00},{0xa7}},
    {{0x50,0x01},{0x83}},
    {{0x51,0x80},{0xff}},
    {{0x51,0x81},{0xf2}},
    {{0x51,0x82},{0x00}},
    {{0x51,0x83},{0x14}},
    {{0x51,0x84},{0x25}},
    {{0x51,0x85},{0x24}},
    {{0x51,0x86},{0x09}},
    {{0x51,0x87},{0x09}},
    {{0x51,0x88},{0x09}},
    {{0x51,0x89},{0x75}},
    {{0x51,0x8a},{0x54}},
    {{0x51,0x8b},{0xe0}},
    {{0x51,0x8c},{0xb2}},
    {{0x51,0x8d},{0x42}},
    {{0x51,0x8e},{0x3d}},
    {{0x51,0x8f},{0x56}},
    {{0x51,0x90},{0x46}},
    {{0x51,0x91},{0xf8}},
    {{0x51,0x92},{0x04}},
    {{0x51,0x93},{0x70}},
    {{0x51,0x94},{0xf0}},
    {{0x51,0x95},{0xf0}},
    {{0x51,0x96},{0x03}},
    {{0x51,0x97},{0x01}},
    {{0x51,0x98},{0x04}},
    {{0x51,0x99},{0x12}},
    {{0x51,0x9a},{0x04}},
    {{0x51,0x9b},{0x00}},
    {{0x51,0x9c},{0x06}},
    {{0x51,0x9d},{0x82}},
    {{0x51,0x9e},{0x38}},
    {{0x53,0x81},{0x1e}},
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
    {{0x55,0x80},{0x02}},
    {{0x55,0x83},{0x40}},
    {{0x55,0x84},{0x10}},
    {{0x55,0x89},{0x10}},
    {{0x55,0x8a},{0x00}},
    {{0x55,0x8b},{0xf8}},
    {{0x58,0x00},{0x23}},
    {{0x58,0x01},{0x14}},
    {{0x58,0x02},{0x0f}},
    {{0x58,0x03},{0x0f}},
    {{0x58,0x04},{0x12}},
    {{0x58,0x05},{0x26}},
    {{0x58,0x06},{0x0c}},
    {{0x58,0x07},{0x08}},
    {{0x58,0x08},{0x05}},
    {{0x58,0x09},{0x05}},
    {{0x58,0x0a},{0x08}},
    {{0x58,0x0b},{0x0d}},
    {{0x58,0x0c},{0x08}},
    {{0x58,0x0d},{0x03}},
    {{0x58,0x0e},{0x00}},
    {{0x58,0x0f},{0x00}},
    {{0x58,0x10},{0x03}},
    {{0x58,0x11},{0x09}},
    {{0x58,0x12},{0x07}},
    {{0x58,0x13},{0x03}},
    {{0x58,0x14},{0x00}},
    {{0x58,0x15},{0x01}},
    {{0x58,0x16},{0x03}},
    {{0x58,0x17},{0x08}},
    {{0x58,0x18},{0x0d}},
    {{0x58,0x19},{0x08}},
    {{0x58,0x1a},{0x05}},
    {{0x58,0x1b},{0x06}},
    {{0x58,0x1c},{0x08}},
    {{0x58,0x1d},{0x0e}},
    {{0x58,0x1e},{0x29}},
    {{0x58,0x1f},{0x17}},
    {{0x58,0x20},{0x11}},
    {{0x58,0x21},{0x11}},
    {{0x58,0x22},{0x15}},
    {{0x58,0x23},{0x28}},
    {{0x58,0x24},{0x46}},
    {{0x58,0x25},{0x26}},
    {{0x58,0x26},{0x08}},
    {{0x58,0x27},{0x26}},
    {{0x58,0x28},{0x64}},
    {{0x58,0x29},{0x26}},
    {{0x58,0x2a},{0x24}},
    {{0x58,0x2b},{0x22}},
    {{0x58,0x2c},{0x24}},
    {{0x58,0x2d},{0x24}},
    {{0x58,0x2e},{0x06}},
    {{0x58,0x2f},{0x22}},
    {{0x58,0x30},{0x40}},
    {{0x58,0x31},{0x42}},
    {{0x58,0x32},{0x24}},
    {{0x58,0x33},{0x26}},
    {{0x58,0x34},{0x24}},
    {{0x58,0x35},{0x22}},
    {{0x58,0x36},{0x22}},
    {{0x58,0x37},{0x26}},
    {{0x58,0x38},{0x44}},
    {{0x58,0x39},{0x24}},
    {{0x58,0x3a},{0x26}},
    {{0x58,0x3b},{0x28}},
    {{0x58,0x3c},{0x42}},
    {{0x58,0x3d},{0xce}},
    {{0x50,0x25},{0x00}},
    {{0x3a,0x0f},{0x30}},
    {{0x3a,0x10},{0x28}},
    {{0x3a,0x1b},{0x30}},
    {{0x3a,0x1e},{0x26}},
    {{0x3a,0x11},{0x60}},
    {{0x3a,0x1f},{0x14}},
    {{0x30,0x08},{0x02}},
    {{0x30,0x35},{0x21}},
    {{0x3c,0x01},{0xb4}},
    {{0x3c,0x00},{0x04}},


};

//subsampling in vertical and horizontal,2x2 binning, 1296x968, 
static struct regval_list sensor_sxga_regs[] =   //SXGA: 1280*960, subsample Off, Scaling On
{
    //capture 5Mega 3.75fps
    { {0x35,0x03}, {0x07}
    },
    {{0x3a,0x00},{0x78}},
    {{0x35,0x0c},{0x00}},
    {{0x35,0x0d},{0x00}},
    {{0x3c,0x07},{0x07}},
    {{0x38,0x14},{0x11}},
    {{0x38,0x15},{0x11}},
    {{0x38,0x03},{0x00}},
    {{0x38,0x07},{0x9f}},
    {{0x38,0x08},{0x0a}},
    {{0x38,0x09},{0x20}},
    {{0x38,0x0a},{0x07}},
    {{0x38,0x0b},{0x98}},
    {{0x38,0x0c},{0x0b}},
    {{0x38,0x0d},{0x1c}},
    {{0x38,0x0e},{0x07}},
    {{0x38,0x0f},{0xb0}},

    {{0x40,0x02},{0xc5}},
    {{0x40,0x05},{0x1a}},
    {{0x53,0x02},{0x30}},//sharpness
    {{0x38,0x13},{0x04}},
    {{0x36,0x18},{0x04}},
    {{0x36,0x12},{0x2b}},
    {{0x37,0x09},{0x12}},
    {{0x37,0x0c},{0x00}},
    {{0x3a,0x02},{0x07}},
    {{0x3a,0x03},{0xb0}},
    {{0x3a,0x0e},{0x06}},
    {{0x3a,0x0d},{0x08}},
    {{0x3a,0x14},{0x07}},
    {{0x3a,0x15},{0xb0}},
    {{0x40,0x04},{0x06}},
    {{0x30,0x35},{0x21}},
    {{0x30,0x36},{0x46}},
    {{0x48,0x37},{0x2c}},
    {{0x50,0x01},{0xa3}},

    //1280*960
    {{0x38,0x08},{0x05}},
    {{0x38,0x09},{0x00}},
    {{0x38,0x0a},{0x03}},
    {{0x38,0x0b},{0xc0}},
};




static struct regval_list sensor_svga_regs[] =   //SVGA: 800*600, ???
{
    { {0x38,0x00}, {0x0 }
    },
    {{0x38,0x01},{0x0 }},
    {{0x38,0x02},{0x0 }},
    {{0x38,0x03},{0x4 }},
    {{0x38,0x04},{0xa }},
    {{0x38,0x05},{0x3f}},
    {{0x38,0x06},{0x7 }},
    {{0x38,0x07},{0x9b}},
    {{0x38,0x08},{0x3 }},
    {{0x38,0x09},{0x20}},
    {{0x38,0x0a},{0x2 }},
    {{0x38,0x0b},{0x58}},
    {{0x38,0x0c},{0x7 }},
    {{0x38,0x0d},{0x68}},
    {{0x38,0x0e},{0x3 }},
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
static struct sensor_win_size
{
    int	width;
    int	height;
    // register list
    struct regval_list *regs;
    int regs_size;
} sensor_win_sizes[] =
{
    /* qsxga: 2592*1944 */
    {
        .width			= 2592,
        .height 		= 1944,
        .regs			= sensor_qsxga_regs,
        .regs_size	= ARRAY_SIZE(sensor_qsxga_regs),
    },

    /* 1080P */
    {
        .width			= 1920,
        .height			= 1080,
        .regs 			= sensor_1080p_regs,
        .regs_size	= ARRAY_SIZE(sensor_1080p_regs),
    },

    /* SXGA */
    {
        .width			= 1280,
        .height 		= 960,
        .regs			= sensor_sxga_regs,
        .regs_size	= ARRAY_SIZE(sensor_sxga_regs),
    },
    /* 720p */
    {
        .width			= 1280,
        .height			= 720,
        .regs 			= sensor_720p_regs,
        .regs_size	= ARRAY_SIZE(sensor_720p_regs),
    },

    /* SVGA */
    {
        .width			= 800,
        .height			= 600,
        .regs				= sensor_svga_regs,
        .regs_size	= ARRAY_SIZE(sensor_svga_regs),
    },
    /* VGA */
    {
        .width			= 640,
        .height			= 480,
        .regs				=  sensor_init_regs, //sensor_vga_regs,
        .regs_size	= ARRAY_SIZE(sensor_init_regs),
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
    ispwidth |= data+1;  //x-end
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
    ispheight |= data+1; //x-end
	ispheight -= offset;
	

//------------------	get output size before scaling, as double offset from isp origin, or offset of origin within isp window

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
if (data&0x04) nighton=1;

address[1]=0x05;
sensor_read(i2cdev, address, &data);
frameinsert=0;
if (data&0x40) frameinsert=1;

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
if (data&0x80) lenc=1;
if (data&0x20) rawg=1;
if (data&0x04) bpc=1;
if (data&0x02) wpc=1;
if (data&0x01) cip=1;

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


/*
precondition:  scaling is on, subsample is off

set output window size by using down-scaling from the whole physical sensor. full view
isp input window is set to max.
if aspect-ratio is 4:3 then just downsclaing is used.
if aspect is not 4:3, x or y sip offset is set to desired aspect ratio. then its downscaled from there.
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
data = (ispx-1)>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x05;
data = (ispx-1); //(0 to n-1!)
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
data = (ispy-1)>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x07;
data = (ispy-1); //(0 to n-1!)
sensor_write(i2cdev, address, &data);	

// set isp offset--not changed as we do just scale without aspect correction

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
data = 0x11;
sensor_write(i2cdev, address, &data);
address[1]=0x15;
sensor_write(i2cdev, address, &data);	

// SDE on(7), scale on(5), UV average off(2), CMX on(1), AWB on(0)
address[0]=0x50;
address[1]=0x01;
data = 0xa3;
sensor_write(i2cdev, address, &data);

return 0;
}

/*
This function changes the area of the sensor to be scanned in. The View.
The size that is scanned in equals the given output window size.
This is also called crobbing or better zooming, as the chosen section looks enlarged, its the section of the full resolution.

The ISP is fed with the selection section of the picture.
This selection can be done by either changing the ISP-input window (select any area in the picture)
or changing the ISP offset (zooming into the center of the picture/above ISP-input window),
or both.

As the scanned sensor area is smaller, we can change the Timing VTS HTS to have faster FPS.

precondition:
HTS VTS at max..to be tested: depends on ISP input size!
scaling off
subsample off

HTS: number of picturepixels + 32 dummypixel (16 in front and 16 an the end)
VTS: number of picturelines + 20 dummy lines (14 in front and 6 at the end)

Framerate HTS * VTS / pxclk: determines exposure and shutter.
1896 * 984 (1,92)= 15 fps  (640 x 480) with subsample of full picture
2844 * 1968 (1,44)= 5 fps  (2592 x 1944) no subsample

entry: 
- ispwidth,ispheight	size of isp-input window (must be >= outwindow). We current cannot give an origin offset to this window (where in the original)
						if either value is 0, then a seperate ISP-input window is not created and just the ISP offset is used, which works the same.(tbtested)
						This window enters into the ISP and influences the FPS.
						The TestBarImage is generated inside the ISP, so this actually gets full size on this setting.(the picture doesnt)
- outwidth,outheight 	size of output window. The desired window size. This Subwindow is in the ISP-Input window.
						if either value is 0, output window is set to max resolution!!!
						This window is the Output of the ISP and does not influence FPS!

Will set ISP input window.
Then set offset to difference isp to output.

exit:_ 0=success, else error
*/
int zoom_image(unsigned int ispwidth, unsigned int ispheight, unsigned int outwidth, unsigned int outheight)
{
unsigned char address[REG_ADDR_STEP], data;
unsigned int ispx, ispy, outx, outy, maxx, maxy;
maxx = 2592; // max ISP input size
maxy = 1944;
address[0]=0x38;

if (!ispwidth || !ispheight || !outwidth || !outheight) //if special settings
{
	if (!ispwidth || !ispheight) // no isp window
	{
		ispwidth = maxx;
		ispheight = maxy;
	}
	
	if (!outwidth || !outheight)
	{
		ispwidth = outwidth =maxx;
		ispheight = outheight = maxy;
	}
}

if ( (outwidth > maxx) || (outheight > maxy) ) return 1; // error
if ( (outwidth > ispwidth) || (outheight > ispheight) || (ispwidth > maxx) || (ispheight > maxy) ) return 1; // error

ispx = (maxx - ispwidth)/2; //x-start of scanning, can be 0!
ispy = (maxy - ispheight)/2; //y-start of scanning

// set isp window -------------------------
//isp x-start
address[1]=0x00;
data = ispx>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x01;
data = ispx;
sensor_write(i2cdev, address, &data);	
//isp x-end
ispx += ispwidth-1; //(0 to n-1!)
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
ispy += ispheight-1; //(0 to n-1!)
address[1]=0x06;
data = ispy>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x07;
data = ispy;
sensor_write(i2cdev, address, &data);	

// set output window -------------------------
outx = (ispwidth - outwidth)/2; //x-offset
outy = (ispheight - outheight)/2; // y-offset

// isp offset:
//out x-offset
address[1]=0x10;
data = outx>>8;
sensor_write(i2cdev, address, &data);
address[1]=0x11;
data = outx;
sensor_write(i2cdev, address, &data);	
//out y-offset
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

// subsample off	
address[1]=0x14;
data = 0x11;
sensor_write(i2cdev, address, &data);
address[1]=0x15;
sensor_write(i2cdev, address, &data);	
// scale off
// SDE on(7), scale off(5), UV average off(2), CMX on(1), AWB on(0)
address[0]=0x50;
address[1]=0x01;
data = 0x83;
sensor_write(i2cdev, address, &data);


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

    case V4L2_CID_AUTOGAIN: // nightmode on/off
        control.autogain = ctrl->value;
        address[0]=0x3A;
        address[1]=0x00;
        ret=sensor_read(i2cdev, address, &data);
        if (ctrl->value) data |= 0x04;
        else data &= ~0x04;
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
        address[0]=0x55; //y gain for contrast
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
auto_exposure is used to enable/disable night mode. check this ?????????????????????

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

    if (info->width > 800 && info->height > 600)
    {
        cp->timeperframe.denominator = 15;
    }
    else
    {
        cp->timeperframe.denominator = 30;
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

    csi_debug(1,"sensor_s_fmt\n");

    // we call try-fmt again here(user should have done that before), just to make sure!, also we get the pointers we need.
    ret = sensor_try_fmt_internal(sd, fmt, &pformat, &psize);
    if (ret)
        return ret; // no match found

// reset the camera first, so we have all default values in the registers
//	sensor_reset(sd, CSI_SUBDEV_RST_PUL);
    // set camera matched windowsize . (if actually one found, yeah always!)
    if (psize->regs)
    {
        ret = sensor_write_array(sd, psize->regs , psize->regs_size);
        if (ret < 0)
            return ret;
    }

    // set the camera videoformat as to best found.
    sensor_write_array(sd, pformat->regs , pformat->regs_size);

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
   return sensor_write_array(sd, sensor_init_regs , ARRAY_SIZE(sensor_init_regs));
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
		
		showinfo();		// show last used settings for debug
		
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
// its called until success is returned from add_driver.
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

// Sensor Docu:>>>


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
	
Video Timing in Pixelclocks:
5Mpixel: 2592 * 1944
Vsync-periodendauer = 	5596692 = one complete frame, depends on number of lines.ie. height
HREF-periodendauer = 2844 (HTS) min.HREF-Pulsdauer + ?2
HREF-Pulsdauer = 2592 (output linelength) ie. width = valid pixeldata



*/
/* ---------------------- ov5640 sensor docu --------------------
Register 3800 - 3807: ISP Window
     define subarea on  physical image sensor. the ORIGIN-top left and Bottom/Right.  default is the whole sensor array!
     visible area = ISP Input Window. - this area is fed into the image sensor processor ISP. the smaller the faster FPS!
	using downscaling:
	if ISP-Window has same aspect ratio as sensor. just define your output size in DVPHO and DVPVO to be downscaled.
	if ISP-Window aspect ratio different to sensor:
	Need to correct ISP-window-hight to match output aspect ratio:
	Conversion Factor CF = DVPHO / DVPVO; ie. Width/Height
	define ISP Input Image Height IH= fe. 1944.
	then ISP Input Image Width IW = IH * CF .
	example for full image:
	Xoffset = 16; no change
	Yoffset = 4 +(2592  - IW)/2
	
	Digital Zoom:
	For any image output size less than 2592x1944, increase  X offset and Y offset in such way that 
	the aspect ratio of input image match with aspect ratio of output image, digital zoom function is 
	implemented by this way.
	ZoomFactor ZF. example 2
	IW = 2592/ZF
	IH = 1944/ZF
	Xoffset = 16 + (2592 - IW)/2
	Yoffset = 4 +  (1944 - IH)/2
	
    */
/*
+++ Understand rolling shutter and exposure:
There are 2 linepointers to the sensor array:
- a line reset pointer that causes the linecells to be erased/reset for next exposure.
- a line readout pointer that selects the linecells to be readout.
The distance (in lines) between these 2 pointers define the exposure time of the sensors cells.
The Time for readout and reset depend on the cameras pixelclock and picture dimensions.
The minimum exposure time is 1 line processing time for readout/reset. (Trow)
The maximum exposure time is the total number of lines of the picture (ie.hight). This is actually a full frame readout!
maximum exposure interval: 1964lines x Trow for ov5640.
Trow is the Linetime in pixelclock-periods.
example we have a resolution of 1600pixel x 1200lines:
Here one line is composed of 1600pixelclock (active picture)(HREF=ON), and 322pixelclock(horizontal blanking) (HREF=OFF).
The blanking is unclear if needed, but at least defines the offtime of the HREF pulse(and thus the time for one line!.
So the total horizontal pixelclocks are 1922.
Depending on your PLL and timing settings, you get a certain Pixelclock frequency. Lets assume we have a pixelclock of 36Mhz.
Then Trow = 39,39usecs. (one line)
The total Frametime results in 66,62msecs (if no additional dummy-lines are added, to adjust Framerate!!).
So the rollingshutter exposuretimes are: minimum=39,39usecs and maximum=66,62msecs.

To get bigger exposute times you need to:
- reduce pixelclock
- add dummy pixel to the line (fine adjust)
- add dummy lines to the frame
However, this reduces the Framerate!
ov5640 supports larger than one frame exposuretimes in registers 0x350C/0x350D !! which adds dummylines to the frame.manual only

Docu on Video Timing and output sizes:
ISC = Image Sensor Core = Physical Sensor, 50/60 Hz detection, analog gain, outputs ADC signal to ISP	
ISP = Image Sensor Processor, receives analog picture, performs image processing, outputs to Fifo and DVP.
DVP = Digital Video Port, 8Bit parallel video output port, as opposed to MIPIport. performs mjeg, CCIR656, hsync pulse if wanted.

Alltogether 3 modes of sensor picturesize change(output size/data conversion)  are available:
- subsampling/binning = full image but only use every second or third pixel for output data.2x2 binnig reduce picsize by 4! or is it 2?
- windowing = crobbing, cut out a subarea of the physical sensor for output (zoom)
- scaling = full image downscale. digital scaler downsamples the whole image for smaler output.

+++ relevant registers in 4 groups:
+ 1: define ISP input size ie. FieldOfView FOV (Subwindow to be scanned from sensor) : 
	 The FOV can be anywhere on the sensorarray!
    The ISP input size defines the total pixel data actually read from sensor pixel array! 
    This area is fed into the image sensor processor ISP. the smaller the faster FPS possible (see Timing +3)!
    {{0x38,0x00},{0x00}},             //sensor window x-start = 0  (default 0) 
    {{0x38,0x01},{0x00}},             //
    {{0x38,0x02},{0x00}},             //sensor window y-start = 0  (default 0)
    {{0x38,0x03},{0x00}},             //
    {{0x38,0x04},{0x0a}},             //sensor window x-end  = a3f=2623    (default 2623) offset in (0 to n-1)
    {{0x38,0x05},{0x3f}},             //
    {{0x38,0x06},{0x07}},             //sensor window y-end = 79f = 1951   (default 1951) offset in (0 to n-1)
    {{0x38,0x07},{0x9f}},             //subwindow = 2623 * 1951 = 2624*1952

+ 2: define the DVP output picture size that is transmitted out on the parallel port:
	It actually tells the DVP how long the HREF Pulse is on. = DVPHO pixelclocks.
	It also tells the DVP, how many HREF Pulses it shall issue = DVPVO lines.
	It thus defines the size of the outputted image.
	It does not change the timing ie. FPS. Hsync period has a variable delay to compensate missing lines. (see +3)
	If scaling is used, the picture asto +4 is downscaled to this setting.
	If no scaling is used, it should match +4. otherwise it crops +4 from its origin. Or if bigger creates a subarea in big window.
    {{0x38,0x08},{0x02}},            //output width from x-start = 280 = 640    DVPHO (default 2592)  = active (high) portion of HREF
    {{0x38,0x09},{0x80}},            //
    {{0x38,0x0a},{0x01}},            //output height from y-start =  1e0 = 480   DVPVO (default 1944)
    {{0x38,0x0b},{0xe0}},

+ 3: This defines the Video Timing, Linetime in Pixelclocks, Frametime in Lines. HREF-period, VSYNC Period.
	It includes non-visible pixels at start/end of sensor and dummy-lines, and blanking. values should be dividable by 4.
	It must not be less than the ISP Inputwindow! or if binning, half of it. i guess?
	 It also controls exposure and sensor readout logic. Its actually used by the ISC to generate HREF and VSYNC.
	Values: ov5640 sensor array = 2624 coloms * 1964 rows. active pixel that can be outputted: 2592 * 1944. minxy-offset=32/8!
	(caution: the CSI interface can only handle 4096 Bytes, thus for yuv422 thats 2048!maybe?)
    {{0x38,0x0c},{0x07}},             //total width = 768 = 1896   HTS (default 2844)  Horizontal total size for 1 line = HREF Period
    {{0x38,0x0d},{0x68}},             //
    {{0x38,0x0e},{0x03}},             //total height = 3d8 = 984   VTS (default 1968)  Vertical total size for 1 frame. VTS*HTS = VSYNC Period
    {{0x38,0x0f},{0xd8}},             //

+ 4: This defines a Subwindow in the ISP Inputwindow.  It defines the size of the output-window if no scaling is used. ie. it crops.
	 If scaling is used, this Subwindow is used to be downscaled to the outputsize as defined in +2.
	 So we have actually 2 methods of "cropping" the physical sensor: +1  and +4. where only +1 will have effect on Framerate!
	 The offsets are applied to the beginning and end of the ISP Window to result in an always centered window.
	 Therefore the offset values must be divided by 2.
    {{0x38,0x10},{0x00}},             //isp x-offset = 10 = 16  H offset(default 16) default min offset = valid array size
    {{0x38,0x11},{0x10}},              //
    {{0x38,0x12},{0x00}},              //isp y-offset = 4 = 4   V offset(default 4)
    {{0x38,0x13},{0x04}},              //resulting isp-input= (2624-32)2592 * (1952-8)1944. 
	
//calculation of HTS VTS:
given: isp input size as to regs 0 to 7.
HTS = IPS-horizontal-size + 10%.  HTS must be dividable by 4!
HTS is the HREF Period. IPS-horizontal-size is HREF ontime.
HREF period (HTS) defines the rolling shutter time of the line! and the framerate. 

VTS = ISP-input-lines + (20232/HTS +1) + (variable delay in HTS units to accomplish correct FPS)
(20232 is the minimum pre and post delay for vsnyc)
VTS must be dividable by 4!?
HSYNC period = HTS*VTS   hsync defines the exposure as it is the time a row can gather light.


+++subsample and binning:
subsample means, scipping pixel horizontal, and skipping lines vertical. Thus reducing image datasize while keeping FOV.
binning means, averaging the lightlevels of adjacent pixel (the ones that are skipped by subsample) into the subsample destination.
Thus considering also the skipped pixels.

A typical subsample with binning of 2x2 will result in halfing the imagesize.

This all happends in the analog sensor readout logic.

Section +1 and +4 define the field of view.
Section +3 and +4 should assume the halfed datasize of image, in active binning.
on the other hand, as binning is in the analog section, the ISP input data is actually half of +1. Then +4 would apply to
the reduced framedata?

subsample/binning registers:
0x3814 = horizontal even/odd increment 0x31 for 2x2
0x3815 = vertical even/odd increment  0x31 for 2x2
0x3821 = horizontal binning on[1] =1  for 2x2


+++ custom settings supplied by omnivision, are used for:
- 50/60Hz detection
- LENC lens correction
- AWB auto white balance

+++ AEC/AGC registers:
exposure:
0x3503 = auto/manual mode select for aec and agc.
(max value for (3500-02)>>4 = 380e-0f(VTS) + 350c-0d(extra dummylines).manual only
0x3500 -02 = 20bit exposure vale in lines<<4. actually 16bit value(skipping lower 4 bits) (autoset in autoaec)
0x350c-0d = 16bit extra dummylines per frame to enlarge exposure. ..!! auto updated!!
0x56a1 = current image average luminance value  High 8 bits. auto updated
aec control range:
all 4 regs compare to 0x56a1. if within controlrange no aec adjust necessary. This implements a hysteresis!
0x3A0F = Stable Range High Limit (enter),  stable region . this is relevant for auto aec!
0x3A10 = Stable Range Low Limit (enter),
0x3A1B = control Range High Limit (go out), control region .
0x3A1E = control Range Low Limit (go out) if yavg within this range, no change, else adjust as to stable range.

defining the average window for YAVG, the picture average luminance value in 0x56a1:(only needed if manual enabled in 0x501D)!!
0x5680 - 01 = 12bit X-start . Horizontal start position for average window
0x5684 - 85 = 12bit X-end
0x5682 - 03 = 11bit Y-start Vertical start position for average window
0x5686 - 87 = 11bit Y-end

gain:
automatic gain control will only activate if aec control reaches its limit. Value is autoset in autoagc!
0x350a-0b = 10bit current analog gain value (auto set in auto agc). maximum gain is 64. range:0-1023. so div 16 gives real gain.
0x3A18-19 = 10bit AGC ceiling/max value

+++ nightmode:
0x3A00[2] = nightmode enable(1)
0x3A05[6] =  insert frame enable(1). increase and decrease step based on frames(1) or lines/bands(0).off=longer exposure!
0x3A05[5] =  exposure step auto(1) or manual(0). step auto-ratio can be set in bits 0-4
0x3A17[1:0]   Gain night threshold = gain value when in nightmode
night mode ceiling:(max exposure when nightmode is used)
0x3A02-03 = nightmode ceiling or 16bit max_exposure60hz. default: 15744
            Only this register must be set to define the maximum wanted exposure in nightmode!!!
            added lines will then be shown in 0x350c-0d.

0x3A14-15 = 16bit max_exposure50hz. default: 3648

+++ banding 50/60Hz detection:
 banding filter = exposure is done in steps of integer multiples of the period of the light source.
- banding(0x3A00[5]) = (1)band function enable, if on, aec is done in steps of "band steps". if off, aec is continuoues.
- bandautodetect(0x3c01[7]) = if on(0), will detect 50 or 60 hz light frequency automaticly. 
  The detected freq can be red from 3C0C_0.
3a08-09 = 50hz band step width, is set approx to VTS automaticly, maybe need to set extra framelines as well.
3a0a-0b = 60hz band step width

+++ things we have to set for each resolution:
+++ nothing+++:-)
In all resolutions we provide auto aec, auto agc, auto 50/60hz detection, possibly auto-nightmode.
- the max exposure value ??

- When night mode is turned on, dummy lines is inserted automatically and the frame rate is decreased.
- nightmode can be left on. does not harm capture during normal light conditions. ie. is not active then.

//+++jpeg capture:
It seems jpeg capture is not supported by the A20 CSI interface.

------------------------------------------------*/
