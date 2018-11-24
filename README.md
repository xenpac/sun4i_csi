# OV5640-camera driver for Sunxi-A20

This is a replacement driver for the existing Linux Sunxi CSI Camera Driver "sun4i_csi".
compatible with A10 (sun4i), A12/A13/A10S (sun5i) and A20 (sun7i) SoCs.

Possible kernel sources:

https://github.com/linux-sunxi/linux-sunxi/tree/stage/sunxi-3.4

or

https://github.com/nisenbeck/linux-bananapi

or

https://github.com/LeMaker/linux-sunxi

directory path: ../linux-bananapi/drivers/media/video/sun4i_csi/...

...to be replaced with this subbranch.



Developed on Banana Pi Version 1 (the first one) Allwinner A20 CSI parallel port, using OV5640 camera module.

This provides a new CSI Driver and a new sub-device driver for the OV5640 Camera.

I have been working on this for the last ..3 years on and off when there was spare time.

The old CSI driver had issues like artifacts first capture being present in the video stream and so on.

The OV5640 Subdevice driver was basicly rewritten and now supports discrete windowsize selection and FPS display.

I tried autofocus but no success with my camera module.


I will include a "docu" subdirectory to include some usefull scripts for compiling the driver and the camera-subdevice, plus a self made
tool to test the camera output. ie. FPS display, parameter selection and display.

Howto use (compile on the bananapi):


    apt-get install git build-essential libncurses5-dev u-boot-tools uboot-mkimage   - get compiler and tools
    cd          go home
    git clone https://github.com/nisenbeck/linux-bananapi.git --depth 1    - get kernel source code into new directory linux-bananapi

    cd linux-bananapi/drivers/media/video
    //remove or rename the directory sun4i_csi
    mv sun4i_csi sun4i_csi_bak    or   rm -r sun4i_csi
    git clone https://github.com/xenpac/sun4i_csi.git   -  get new code,  this will create new directory "sun4i_csi" with new code

    cd  - go home
    cd linux-bananapi
    make sun7i_defconfig  -  make the default kernel .config for A20 bananapi
 
    make -j2 uImage modules  -  compile the kernel into uImage, the modules into the .ko files.  -j2 = use 2 threads
    make modules_install    -    copys all generated modules .ko from this compile directory to /lib/modules/kernelversion/
                            -     ie. u have one module branch for each specific kernel-version!

    mv  /boot/bananapi/uImage /boot/bananapi/uImage.ori  -  backup old kernel just in case!

    cp /home/bananapi/linux-bananapi/arch/arm/boot/uImage /boot/bananapi/  -  copy new kernel to boot
 
    reboot  - (into the new kernel)

you now have ov5640 ready on /dev/video0


comments are welcome, xenpac
