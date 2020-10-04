# ili9325
Raspberry Pi 8 bit parallel ili9325 LCD driver 

This is mostly a fork from the ili9341 driver.

The display identifies its self as a 0x6809 which is the same as 0x9325.  Except a couple of unused registers.
I have had it running at about 16fps on a Pi-Zero.  The code is not neat, but works.



Installing:
   download the source
   run make
   sudo insmod ./ili9325.ko
this will create /dev/fb1.
Use the new framebuffer (https://github.com/notro/fbtft/wiki/Framebuffer-use)

The photos folder shows the different pinout.
