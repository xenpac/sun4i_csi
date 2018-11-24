 # OV5640-Sunxi-A20

This is a replacement driver for the existing Linux Sunxi CSI Camera Driver.

https://github.com/linux-sunxi/linux-sunxi/tree/stage/sunxi-3.4

directory path: ../linux-bananapi/drivers/media/video/sun4i_csi/...

...to be replaced with this subbranch.

Platform: Banana Pi Version 1 (the first one) Allwinner A20 CSI parallel port, CSI Cameras

This provides a new CSI Driver and a new sub-device driver for the OV5640 Camera.

I have been working on this for the last ..3 years on and off when there was spare time.

The old CSI driver had issues like artifacts first capture being present in the video stream and so on.

The OV5640 Subdevice driver was basicly rewritten and now supports discrete windowsize selection and FPS display.

I tried autofocus but no success with my camera module.


I will include a "docu" subdirectory to include some usefull scripts for compiling the driver and the camera-subdevice, plus a self made
tool to test the camera output. ie. FPS display, parameter selection and display.

comments are welcome, xenpac
