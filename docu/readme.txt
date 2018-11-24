some utilitys:

removedev ov5640 - will detach a camera from the system
addev ov5640 - will attach a camera to the system

./makcam.sh will compile and install all updated camera source file as being mentioned in makefile.
./makdrv.sh will compile the csi driver and install it.

./vt will execute the camera test program:
its terminal oriented and responds to your keyboard input. help is available in the program.


Hint:
i added the .ko files in extra. you can try to place them in the /lib/modules/kernel-version/drivers/media/video/sun4i_csi/....
maybe that works as a simple update.