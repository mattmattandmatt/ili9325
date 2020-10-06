// ili9325 LCD for Raspberry Pi
//   chip id = 0x6809, force use = 0x9325

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/interrupt.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/backlight.h>
#include <linux/platform_device.h>
#include <linux/uaccess.h>


#define BLOCKSIZE (4*1024)

// GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13) & (1<<g)) // 0 if LOW, (1<<g) if HIGH

//#define GPIO_PULL *(gpio+37) // Pull up/pull down
//#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

#define DATA0 16 // in aligned order
#define DATA1 17
#define DATA2 18
#define DATA3 19
#define DATA4 20
#define DATA5 21
#define DATA6 22
#define DATA7 23

#define DC 26 // RS
#define CS 06
#define RD 25
#define RW 24
//#define IM0 27 // unused
#define RESET 05

#define PHY_DISPLAY_WIDTH   320 // Real size
#define PHY_DISPLAY_HEIGHT  240
#define DISPLAY_BPP         16  // 5/6/5



// These are copied from Arduino
#define ILI932X_START_OSC          0x00
#define ILI932X_DRIV_OUT_CTRL      0x01
#define ILI932X_DRIV_WAV_CTRL      0x02
#define ILI932X_ENTRY_MOD          0x03
#define ILI932X_RESIZE_CTRL        0x04
#define ILI932X_16BITS_FORMAT      0x05 // Added - might need to remove
#define ILI932X_DISP_CTRL1         0x07
#define ILI932X_DISP_CTRL2         0x08
#define ILI932X_DISP_CTRL3         0x09
#define ILI932X_DISP_CTRL4         0x0A
#define ILI932X_RGB_DISP_IF_CTRL1  0x0C
#define ILI932X_FRM_MARKER_POS     0x0D
#define ILI932X_RGB_DISP_IF_CTRL2  0x0F
#define ILI932X_POW_CTRL1          0x10
#define ILI932X_POW_CTRL2          0x11
#define ILI932X_POW_CTRL3          0x12
#define ILI932X_POW_CTRL4          0x13
#define ILI932X_GRAM_HOR_AD        0x20
#define ILI932X_GRAM_VER_AD        0x21
#define ILI932X_RW_GRAM            0x22
#define ILI932X_POW_CTRL7          0x29
#define ILI932X_FRM_RATE_COL_CTRL  0x2B
#define ILI932X_GAMMA_CTRL1        0x30
#define ILI932X_GAMMA_CTRL2        0x31
#define ILI932X_GAMMA_CTRL3        0x32
#define ILI932X_GAMMA_CTRL4        0x35
#define ILI932X_GAMMA_CTRL5        0x36
#define ILI932X_GAMMA_CTRL6        0x37
#define ILI932X_GAMMA_CTRL7        0x38
#define ILI932X_GAMMA_CTRL8        0x39
#define ILI932X_GAMMA_CTRL9        0x3C
#define ILI932X_GAMMA_CTRL10       0x3D
#define ILI932X_HOR_START_AD       0x50
#define ILI932X_HOR_END_AD         0x51
#define ILI932X_VER_START_AD       0x52
#define ILI932X_VER_END_AD         0x53
#define ILI932X_GATE_SCAN_CTRL1    0x60 // Driver Output Control 2
#define ILI932X_GATE_SCAN_CTRL2    0x61 // Base Image Display Control
#define ILI932X_GATE_SCAN_CTRL9    0x66 // SPI Read/Write Control
#define ILI932X_GATE_SCAN_CTRL3    0x6A // Vertical Scroll Control
#define ILI932X_PART_IMG1_DISP_POS 0x80
#define ILI932X_PART_IMG1_START_AD 0x81
#define ILI932X_PART_IMG1_END_AD   0x82
#define ILI932X_PART_IMG2_DISP_POS 0x83
#define ILI932X_PART_IMG2_START_AD 0x84
#define ILI932X_PART_IMG2_END_AD   0x85
#define ILI932X_PANEL_IF_CTRL1     0x90
#define ILI932X_PANEL_IF_CTRL2     0x92
#define ILI932X_PANEL_IF_CTRL3     0x93
#define ILI932X_PANEL_IF_CTRL4     0x95
#define ILI932X_PANEL_IF_CTRL5     0x97
#define ILI932X_PANEL_IF_CTRL6     0x98 // Unused ?
#define ILI932X_DEEP_STANDBY_CTRL  0xE6
#define TFTLCD_DELAY               0xFF

static const uint16_t ILI932x_regValues[] = {
  ILI932X_START_OSC        , 0x0001, // Start oscillator ? / NoOp / Read-ID
  TFTLCD_DELAY             , 50,     // 50 millisecond delay
  ILI932X_DRIV_OUT_CTRL    , 0x0000, // can Flip Landscape - 0000 0x0x 0000 0000
  ILI932X_DRIV_WAV_CTRL    , 0x0700, // 0x0700
  ILI932X_ENTRY_MOD        , 0x1008, // xx08=Land1 xx38=Land2 xx20=Port1 xx10=Port2-conn-side    xx18=Land-Flip xx28=Land-Flip-Mirr - will be over-ridden
  ILI932X_RESIZE_CTRL      , 0x0000, // 0x0
  ILI932X_16BITS_FORMAT    , 0x0002,
  0x06                     , 0x0,
  ILI932X_DISP_CTRL1       , 0x0,
  ILI932X_DISP_CTRL2       , 0x0202, // front porch, back porch
  ILI932X_DISP_CTRL4       , 0x0,
  0x0B                     , 0x0,
  ILI932X_RGB_DISP_IF_CTRL1, 0x0,
  ILI932X_FRM_MARKER_POS   , 0x0,
  0x0E                     , 0x0,
  ILI932X_RGB_DISP_IF_CTRL2, 0x0, // 0x0
  ILI932X_POW_CTRL1        , 0x0,
  ILI932X_POW_CTRL2        , 0x0007,
  ILI932X_POW_CTRL3        , 0x0,
  ILI932X_POW_CTRL4        , 0x0,
  TFTLCD_DELAY             , 200,
  0x14                     , 0x0,
  0x15                     , 0x0,
  0x16                     , 0x0,
  0x17                     , 0x0,
  0x18                     , 0x0,
  0x19                     , 0x0,
  0x1A                     , 0x0,
  0x1B                     , 0x0,
  0x1C                     , 0x0,
  0x1D                     , 0x0,
  0x1E                     , 0x0,
  0x1F                     , 0x0,
  ILI932X_POW_CTRL1        , 0x1690,
  ILI932X_POW_CTRL2        , 0x0227, // 0x0227
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL3        , 0x001A, // 0x1A
  TFTLCD_DELAY             , 50,
  ILI932X_POW_CTRL4        , 0x1800,
  ILI932X_POW_CTRL7        , 0x002A,
  TFTLCD_DELAY             , 50,
  ILI932X_GAMMA_CTRL1      , 0x0000,
  ILI932X_GAMMA_CTRL2      , 0x0000,
  ILI932X_GAMMA_CTRL3      , 0x0000,
  ILI932X_GAMMA_CTRL4      , 0x0206, // 0x0206
  ILI932X_GAMMA_CTRL5      , 0x0808,
  ILI932X_GAMMA_CTRL6      , 0x0007,
  ILI932X_GAMMA_CTRL7      , 0x0201,
  ILI932X_GAMMA_CTRL8      , 0x0000,
  ILI932X_GAMMA_CTRL9      , 0x0000,
  ILI932X_GAMMA_CTRL10     , 0x0000,
  ILI932X_GRAM_HOR_AD      , 0x0000,
  ILI932X_GRAM_VER_AD      , 0x0000,
  ILI932X_HOR_START_AD     , 0x0000,
  ILI932X_HOR_END_AD       , PHY_DISPLAY_HEIGHT - 1,
  ILI932X_VER_START_AD     , 0X0000,
  ILI932X_VER_END_AD       , PHY_DISPLAY_WIDTH - 1,
  ILI932X_GATE_SCAN_CTRL1  , 0xA700, // Driver Output Control (R60h) // 0xA700
  ILI932X_GATE_SCAN_CTRL2  , 0x0003, // Driver Output Control (R61h) // 0x0003
  ILI932X_GATE_SCAN_CTRL3  , 0x0000, // Driver Output Control (R62h)
  ILI932X_PANEL_IF_CTRL1   , 0X0010, // Panel Interface Control 1 (R90h)
  ILI932X_PANEL_IF_CTRL2   , 0X0000,
  ILI932X_PANEL_IF_CTRL3   , 0X0003,
  ILI932X_PANEL_IF_CTRL4   , 0X1100,
  ILI932X_PANEL_IF_CTRL5   , 0X0000,
  ILI932X_PANEL_IF_CTRL6   , 0X0000,
  ILI932X_DISP_CTRL1       , 0x0133, // Main screen turn on
};


volatile unsigned *gpio;
static unsigned int fps = 25;
static unsigned int rotation = 0x1008; // Landscape
static unsigned int scale = 1;
static unsigned int scaletype = 1;
static uint16_t DISPLAY_WIDTH  = PHY_DISPLAY_WIDTH;
static uint16_t DISPLAY_HEIGHT = PHY_DISPLAY_HEIGHT;



// Set to output
static void gpio_setoutput(char g)    
{
    INP_GPIO(g); // must use INP_GPIO before we can use OUT_GPIO
    OUT_GPIO(g);
}

// Set state 1=high 0=low
static void gpio_setstate(char g,char state)    
{
    (state) ? (GPIO_SET = 1<<g) : (GPIO_CLR = 1<<g) ;
}
/*
static int gpio_read(void)
{
    int value = 0;

    INP_GPIO(DATA0);
    INP_GPIO(DATA1);
    INP_GPIO(DATA2);
    INP_GPIO(DATA3);
    INP_GPIO(DATA4);
    INP_GPIO(DATA5);
    INP_GPIO(DATA6);
    INP_GPIO(DATA7);
    gpio_setstate(RD,0);
udelay(1);
    value |= GET_GPIO(16) ? 0x0100 : 0;
    value |= GET_GPIO(17) ? 0x0200 : 0;
    value |= GET_GPIO(18) ? 0x0400 : 0;
    value |= GET_GPIO(19) ? 0x0800 : 0;
    value |= GET_GPIO(20) ? 0x1000 : 0;
    value |= GET_GPIO(21) ? 0x2000 : 0;
    value |= GET_GPIO(22) ? 0x4000 : 0;
    value |= GET_GPIO(23) ? 0x8000 : 0;
    gpio_setstate(RD,1);
udelay(1);
    gpio_setstate(RD,0);
udelay(1);
    value |= GET_GPIO(16) ? 0x0001 : 0;
    value |= GET_GPIO(17) ? 0x0002 : 0;
    value |= GET_GPIO(18) ? 0x0004 : 0;
    value |= GET_GPIO(19) ? 0x0008 : 0;
    value |= GET_GPIO(20) ? 0x0010 : 0;
    value |= GET_GPIO(21) ? 0x0020 : 0;
    value |= GET_GPIO(22) ? 0x0040 : 0;
    value |= GET_GPIO(23) ? 0x0080 : 0;
    gpio_setstate(RD,1);

    gpio_setoutput(DATA0);
    gpio_setoutput(DATA1);
    gpio_setoutput(DATA2);
    gpio_setoutput(DATA3);
    gpio_setoutput(DATA4);
    gpio_setoutput(DATA5);
    gpio_setoutput(DATA6);
    gpio_setoutput(DATA7);

    return value;
}
*/
// initialization of GPIO
static void tft_init_board(struct fb_info *info)    
{
    //printk(KERN_INFO "fb%d: tft_init_board\n", info->node);
    gpio_setoutput(DATA0);
    gpio_setoutput(DATA1);
    gpio_setoutput(DATA2);
    gpio_setoutput(DATA3);
    gpio_setoutput(DATA4);
    gpio_setoutput(DATA5);
    gpio_setoutput(DATA6);
    gpio_setoutput(DATA7);

    gpio_setoutput(DC);
    gpio_setoutput(CS);
    gpio_setoutput(RD);
    gpio_setoutput(RW);
    //gpio_setoutput(IM0);
    gpio_setoutput(RESET);

    gpio_setstate(DATA0,0);
    gpio_setstate(DATA1,0);
    gpio_setstate(DATA2,0);
    gpio_setstate(DATA3,0);
    gpio_setstate(DATA4,0);
    gpio_setstate(DATA5,0);
    gpio_setstate(DATA6,0);
    gpio_setstate(DATA7,0);

    gpio_setstate(DC,1);
    gpio_setstate(CS,1);
    gpio_setstate(RD,1);
    gpio_setstate(RW,1);
    //gpio_setstate(IM0,1);
    gpio_setstate(RESET,1);

}

// hard reset of the graphic controller and the tft
static void tft_hard_reset(void)    
{
    gpio_setstate(CS,0);
    gpio_setstate(RESET,0);
    msleep(10); // 120
    gpio_setstate(RESET,1);
    msleep(100); // 120
    gpio_setstate(CS,1);
}

static void gpio_set_parallel_data(char data)    
{
/*
    gpio_setstate(DATA0,((data >> 0)  & 0x01));
    gpio_setstate(DATA1,((data >> 1)  & 0x01));
    gpio_setstate(DATA2,((data >> 2)  & 0x01));
    gpio_setstate(DATA3,((data >> 3)  & 0x01));
    gpio_setstate(DATA4,((data >> 4)  & 0x01));
    gpio_setstate(DATA5,((data >> 5)  & 0x01));
    gpio_setstate(DATA6,((data >> 6)  & 0x01));
    gpio_setstate(DATA7,((data >> 7)  & 0x01));
*/
    // This assumes the data bits are in GPIO order
    GPIO_SET =        data << DATA0; // Set the "set"   bits
    GPIO_CLR = (char)~data << DATA0; // Set the "clear" bits
}

// write command
static void tft_command_write16(uint16_t command)    
{
    gpio_setstate(DC,0);
    gpio_set_parallel_data(command >> 8); // always 0
    gpio_setstate(CS,0); // small delay - can be removed if Short & Equal Length wires are used
//ndelay(1); // 10
    gpio_setstate(RW,0);
    gpio_setstate(CS,0); // small delay
//ndelay(1); // 150
    gpio_setstate(RW,1);

    gpio_set_parallel_data(command & 0xff);
    gpio_setstate(CS,0); // small delay
//ndelay(1); // 10
    gpio_setstate(RW,0);
    gpio_setstate(CS,0); // small delay
//ndelay(1); // 150
    gpio_setstate(RW,1);
}

// write data
static void tft_data_write16(uint16_t data)    
{
    gpio_setstate(DC,1);
    gpio_set_parallel_data(data >> 8);
    gpio_setstate(CS,0); // small delay
//ndelay(1); // 10
    gpio_setstate(RW,0);
    gpio_setstate(CS,0); // small delay
//ndelay(1); // 150
    gpio_setstate(RW,1);

    gpio_set_parallel_data(data & 0xff);
    gpio_setstate(CS,0); // small delay
//ndelay(1); // 10
    gpio_setstate(RW,0);
    gpio_setstate(CS,0); // small delay
//ndelay(1); // 150
    gpio_setstate(RW,1);
}

// initialization of ili9325
static void tft_init(struct fb_info *info)    
{
    uint16_t c, d;
    uint16_t i = 0;
    int x, y, rnd;
    //printk(KERN_INFO "fb%d: tft_init\n", info->node);

    gpio_setstate(CS,0);
    // Copied from an Arduino example
    while(i < sizeof(ILI932x_regValues) / sizeof(uint16_t)) {
        c = ILI932x_regValues[i++];
        d = ILI932x_regValues[i++];
        if(c == TFTLCD_DELAY) {
            mdelay(d);
        } else {
            tft_command_write16(c);
            tft_data_write16(d);
            //printk("%x.....%x..%x\n",c,d >> 8,d & 0xff);
        }
    }
    tft_command_write16(ILI932X_ENTRY_MOD); // Rotation
    tft_data_write16(rotation);


    // Randomised Splash screen lines - indicates its loaded
    tft_command_write16(ILI932X_RW_GRAM); // Mem Write
    for (y=0; y < DISPLAY_HEIGHT; y++) {
        get_random_bytes(&rnd, 2);
        for (x=0; x < DISPLAY_WIDTH; x++) {
            tft_data_write16(rnd);
        }
    }
    gpio_setstate(CS,1);
}
/*
// write memory to TFT
static void ili9325_update_display_area(const struct fb_image *image)    
{
    int x,y;
    printk(KERN_INFO "ili9325_update_display_area %d %d\n", image->dx, image->dy);

    gpio_setstate(CS,0);
    // set column
    (ORIENTATION) ? tft_command_write16(ILI932X_GRAM_HOR_AD) : tft_command_write16(ILI932X_GRAM_VER_AD); // ILI9341_PAGEADDRSET ILI9341_COLADDRSET
    //tft_data_write16(image->dx >> 8);
    tft_data_write16(image->dx);
    //tft_data_write16((image->dx + image->width) >> 8);
    tft_data_write16(image->dx + image->width);

    // set row
    (ORIENTATION) ? tft_command_write16(ILI932X_GRAM_VER_AD) : tft_command_write16(ILI932X_GRAM_HOR_AD);
    //tft_data_write16(image->dy >> 8);
    tft_data_write16(image->dy);
    //tft_data_write16((image->dy + image->height) >> 8);
    tft_data_write16(image->dy + image->height);

    tft_command_write16(ILI932X_RW_GRAM); //Memory Write

    if (ORIENTATION == 0) {
        for(y=0;y < image->width ;y++){
            for(x=0;x < image->height ;x++){
                tft_data_write16(image->data[(image->dx * (2 * image->width)) + (image->dy * 2) + 1]);
                //tft_data_write16(image->data[(image->dx * (2 * image->width)) + (image->dy * 2) + 2]);
            }
        }
    } else {
        for(y=0;y < image->width ;y++){
            for(x=0;x < image->height ;x++){
                tft_data_write16(image->data[(image->dx * (2 * image->width)) + (image->dy * 2) + 1]);
                //tft_data_write16(image->data[(image->dx * (2 * image->width)) + (image->dy * 2) + 2]);
            }
        }
    }

    gpio_setstate(CS,1);
}
*/
/*
static void ili9325_update_display_color_area(const struct fb_fillrect *rect)    
{
    int x,y;
    printk(KERN_INFO "ili9325_update_display_color_area %d %d\n", rect->dx, rect->dy);

    gpio_setstate(CS,0);
    // set column
    (ORIENTATION) ? tft_command_write16(ILI932X_GRAM_HOR_AD) : tft_command_write16(ILI932X_GRAM_VER_AD); // ILI9341_PAGEADDRSET ILI9341_COLADDRSET
    //tft_data_write16(rect->dx >> 8);
    tft_data_write16(rect->dx);
    //tft_data_write16((rect->dx + rect->width) >> 8);
    tft_data_write16(rect->dx + rect->width);

    // set row
    (ORIENTATION) ? tft_command_write16(ILI932X_GRAM_VER_AD) : tft_command_write16(ILI932X_GRAM_HOR_AD);
    //tft_data_write16(rect->dy >> 8);
    tft_data_write16(rect->dy);
    //tft_data_write16((rect->dy + rect->height) >> 8);
    tft_data_write16(rect->dy + rect->height);

    tft_command_write16(ILI932X_RW_GRAM); //Memory Write

    if (ORIENTATION == 0) {
        for(y=0;y < rect->width ;y++){
            for(x=0;x < rect->height ;x++){
                tft_data_write16(rect->color);
                //tft_data_write16(rect->color >> 8);
            }
        }
    } else {
        for(y=0;y < rect->height ;y++){
            for(x=0;x < rect->width ;x++){
                tft_data_write16(rect->color);
                //tft_data_write16(rect->color >> 8);
            }
        }
    }

    gpio_setstate(CS,1);
}
*/
static void ili9325_update_display(const struct fb_info *info)    
{
    int x, y, x2, y2;
    int r1, g1, b1;
    uint16_t pixel;

    gpio_setstate(CS,0);
/*  Commented out, but might be needed for other ili9325's ?
    tft_command_write16(ILI932X_GRAM_HOR_AD);
    tft_data_write16(0);
    tft_command_write16(ILI932X_GRAM_VER_AD);
    tft_data_write16(0);
*/
    // Box filtering - works best with scale=2
    if (scaletype == 2) {
        for (y=0; y < DISPLAY_HEIGHT; y+=scale) {
            for (x=0; x < DISPLAY_WIDTH; x+=scale) {
                r1=g1=b1=0;
                for (y2 = y; y2 < (y + scale); y2++) {
                    for (x2 = x; x2 < (x + scale); x2++) {
                        pixel = *((uint16_t*) &(info->screen_base[((DISPLAY_WIDTH * y2) + x2) * 2]) );
                        r1 += (pixel & 0xF800) >> 11;
                        g1 += (pixel & 0x07E0) >> 5;
                        b1 +=  pixel & 0x001F;
                    }
                }
                r1 /= scale * scale;
                g1 /= scale * scale;
                b1 /= scale * scale;
                pixel = (r1 << 11) | (g1 << 5) | b1;
                info->screen_base[(((DISPLAY_WIDTH * y) + x) * 2) + 1] = pixel >> 8; // Back into the buffer
                info->screen_base[(((DISPLAY_WIDTH * y) + x) * 2)    ] = pixel & 0xFF;
            }
        }
    }


    tft_command_write16(ILI932X_RW_GRAM); // Mem Write

    for (y=0; y < DISPLAY_HEIGHT; y+=scale) {
        for (x=0; x < DISPLAY_WIDTH; x+=scale) {
            tft_data_write16( *((uint16_t*) &(info->screen_base[((DISPLAY_WIDTH * y) + x) * 2]) ) );
        }
    }
    gpio_setstate(CS,1);
    //printk(KERN_INFO "fb%d: ili9325_update_display full\n", info->node);
}

static void ili9325_fillrect(struct fb_info *info, const struct fb_fillrect *rect)    
{
    //printk(KERN_INFO "fb%d: ili9325_fillrect\n", info->node);
    //ili9325_update_display_color_area(rect);
    ili9325_update_display(info);
}

static void ili9325_copyarea(struct fb_info *info, const struct fb_copyarea *area)    
{
    //printk(KERN_INFO "fb%d: ili9325_copyarea\n", info->node);
    ili9325_update_display(info);
}

static void ili9325_imageblit(struct fb_info *info, const struct fb_image *image)    
{
    //printk(KERN_INFO "fb%d: ili9325_imageblit\n", info->node);
    //ili9325_update_display_area(image);
    ili9325_update_display(info);
}

static ssize_t ili9325_write(struct fb_info *info, const char __user *buf, size_t count, loff_t *ppos)    
{
    unsigned long p = *ppos;
    void *dst;
    int err = 0;
    unsigned long total_size;
    //printk(KERN_INFO "ili9325_write cnt=%d\n", count);

    if (info->state != FBINFO_STATE_RUNNING)
        return -EPERM;

    total_size = info->screen_size;

    if (total_size == 0)
        total_size = info->fix.smem_len;

    if (p > total_size)
        return -EFBIG;

    if (count > total_size) {
        err = -EFBIG;
        count = total_size;
    }

    if (count + p > total_size) {
        if (!err)
            err = -ENOSPC;

        count = total_size - p;
    }

    dst = (void __force *) (info->screen_base + p);

    if (info->fbops->fb_sync)
        info->fbops->fb_sync(info);

    if (copy_from_user(dst, buf, count))
        err = -EFAULT;

    if  (!err)
        *ppos += count;

    ili9325_update_display(info);

    return (err) ? err : count;
}

static ssize_t ili9325_read(struct fb_info *info, char __user *buf, size_t count, loff_t *ppos)    
{
    unsigned long p = *ppos;
    void *dst;
    int err = 0;
    unsigned long total_size;
    //printk(KERN_INFO "ili9325_read cnt=%d\n", count);

    if (info->state != FBINFO_STATE_RUNNING)
        return -EPERM;

    total_size = info->screen_size;

    if (total_size == 0)
        total_size = info->fix.smem_len;

    if (p > total_size)
        return -EFBIG;

    if (count > total_size) {
        err = -EFBIG;
        count = total_size;
    }

    if (count + p > total_size) {
        if (!err)
            err = -ENOSPC;

        count = total_size - p;
    }

    dst = (void __force *) (info->screen_base + p);

    if (info->fbops->fb_sync)
        info->fbops->fb_sync(info);

    if (copy_from_user(dst, buf, count))
        err = -EFAULT;

    if  (!err)
        *ppos += count;

    return (err) ? err : count;
}


static void ili9325_deferred_io(struct fb_info *info, struct list_head *pagelist)    
{
    ili9325_update_display(info);
}


static struct fb_fix_screeninfo ili9325_fix = {
    .id             = "ili9325",
    .type           = FB_TYPE_PACKED_PIXELS,
    .visual         = FB_VISUAL_TRUECOLOR,
    .accel          = FB_ACCEL_NONE,
    .xpanstep       = 0,
    .ypanstep       = 0,
    .ywrapstep      = 0,
    .line_length    = PHY_DISPLAY_WIDTH * DISPLAY_BPP / 8, // May change in probe()
};


static struct fb_var_screeninfo ili9325_var = {
    .width          = PHY_DISPLAY_WIDTH,
    .height         = PHY_DISPLAY_HEIGHT,
    .bits_per_pixel = DISPLAY_BPP,
    .xres           = PHY_DISPLAY_WIDTH,
    .yres           = PHY_DISPLAY_HEIGHT,
    .xres_virtual   = PHY_DISPLAY_WIDTH,
    .yres_virtual   = PHY_DISPLAY_HEIGHT,
    .activate       = FB_ACTIVATE_NOW,
    .vmode          = FB_VMODE_NONINTERLACED,

    .nonstd         = 0,
    .red.offset     = 11,
    .red.length     = 5,
    .green.offset   = 5,
    .green.length   = 6,
    .blue.offset    = 0,
    .blue.length    = 5,
    .transp.offset  = 0,
    .transp.length  = 0,
};


static struct fb_ops ili9325_ops = {
    .owner          = THIS_MODULE,
    .fb_read        = ili9325_read,
    .fb_write       = ili9325_write,
    .fb_fillrect    = ili9325_fillrect,
    .fb_copyarea    = ili9325_copyarea,
    .fb_imageblit   = ili9325_imageblit,
};

static struct fb_deferred_io ili9325_defio = {
    .delay          = HZ/25,
    .deferred_io    = ili9325_deferred_io,
};


static int ili9325_probe(struct platform_device *pdev)    
{
    struct fb_info *info;
    int retval = -ENOMEM;
    int vmem_size;
    unsigned char *vmem;
    //int i;


    // Rotation
    rotation |= 0x1000; // Adds 0x1000 onto the user input parameters
    if ((rotation & 0x8) == 0) { // Check it is Portrait mode, Landscape is the default
        DISPLAY_HEIGHT = PHY_DISPLAY_WIDTH;
        DISPLAY_WIDTH  = PHY_DISPLAY_HEIGHT;
        ili9325_fix.line_length    = DISPLAY_WIDTH * DISPLAY_BPP / 8;
        ili9325_var.width          = DISPLAY_WIDTH;
        ili9325_var.height         = DISPLAY_HEIGHT;
        ili9325_var.xres           = DISPLAY_WIDTH;
        ili9325_var.yres           = DISPLAY_HEIGHT;
        ili9325_var.xres_virtual   = DISPLAY_WIDTH;
        ili9325_var.yres_virtual   = DISPLAY_HEIGHT;
    }
    // Change Scaling - nb maybe cause an issue if probed twice
    if (scale > 1) {
        DISPLAY_WIDTH  = DISPLAY_WIDTH  * scale;
        DISPLAY_HEIGHT = DISPLAY_HEIGHT * scale;
        ili9325_fix.line_length    = DISPLAY_WIDTH * DISPLAY_BPP / 8;
        ili9325_var.width          = DISPLAY_WIDTH;
        ili9325_var.height         = DISPLAY_HEIGHT;
        ili9325_var.xres           = DISPLAY_WIDTH;
        ili9325_var.yres           = DISPLAY_HEIGHT;
        ili9325_var.xres_virtual   = DISPLAY_WIDTH;
        ili9325_var.yres_virtual   = DISPLAY_HEIGHT;
    } else if (scale < 1) {
        scale = 1;
    }

    vmem_size = ili9325_var.width * ili9325_var.height * ili9325_var.bits_per_pixel / 8;
    vmem = vzalloc(vmem_size);
    if (!vmem) {
        return -ENOMEM;
    }
    memset(vmem, 0, vmem_size);


    info = framebuffer_alloc(0, &pdev->dev);
    if (!info) {
        vfree(vmem);
        return -ENOMEM;
    }


    info->screen_base = (char __force __iomem*)vmem;
    info->fbops = &ili9325_ops;
    info->fix = ili9325_fix;
    info->fix.smem_start = (unsigned long)vmem;
    info->fix.smem_len = vmem_size;
    info->var = ili9325_var;
    info->flags = FBINFO_DEFAULT | FBINFO_VIRTFB;

    info->fbdefio = &ili9325_defio;
    if (0 < fps) {
        info->fbdefio->delay = HZ/fps;
    }

    fb_deferred_io_init(info);

    retval = register_framebuffer(info);
    if (retval < 0) {
        framebuffer_release(info);
        vfree(vmem);
        return retval;
    }

    platform_set_drvdata(pdev, info);

    gpio = ioremap(GPIO_BASE, BLOCKSIZE); // 4K

    tft_init_board(info);
/*
    // Dump the registers before reset/init()
    gpio_setstate(CS,0);
    for(i=0; i<=0x98; i++) {
        tft_command_write16(i); // Reg Num
        gpio_setstate(DC,1);
        printk(KERN_INFO "0x%02X = 0x%04X", i, gpio_read());
    }
    gpio_setstate(CS,1);
*/
    tft_hard_reset();
    tft_init(info);

    printk(KERN_INFO "fb%d: probe ili9325 LCD framebuffer device rot=0x%x,scale=%d\n", info->node, rotation, scale);
    return 0;
}



static int ili9325_remove(struct platform_device *dev)    
{
    struct fb_info *info = platform_get_drvdata(dev);

    if (info) {
        unregister_framebuffer(info);
        fb_deferred_io_cleanup(info);
        vfree((void __force *)info->screen_base);


        iounmap(gpio);

        framebuffer_release(info);
    }
    return 0;
}


static struct platform_driver ili9325_driver = {
    .probe  = ili9325_probe,
    .remove = ili9325_remove,
    .driver = {
        .name   = "ili9325",
    },
};

static struct platform_device *ili9325_device;

static int __init ili9325_init(void)    
{
    int ret = platform_driver_register(&ili9325_driver);
    if (0 == ret) {
        ili9325_device = platform_device_alloc("ili9325", 0);
        if (ili9325_device) {
            ret = platform_device_add(ili9325_device);
        } else {
            ret = -ENOMEM;
        }
        if (0 != ret) {
            platform_device_put(ili9325_device);
            platform_driver_unregister(&ili9325_driver);
        }
    }
    return ret;
}

static void __exit ili9325_exit(void)    
{
    platform_device_unregister(ili9325_device);
    platform_driver_unregister(&ili9325_driver);
}

module_param(fps, uint, 0440); // Read Only
MODULE_PARM_DESC(fps, "Frames per second");
module_param(rotation, uint, 0440);
MODULE_PARM_DESC(rotation, "Rotation: 0x08=Land1 0x38=Land2 0x20=Port1 0x10=Port2");
module_param(scale, uint, 0440);
MODULE_PARM_DESC(scale, "Scale down: 1 2 3 4");
module_param(scaletype, uint, 0440);
MODULE_PARM_DESC(scaletype, "Scale type: 1=Quick, 2=Averaging");

//   Use a spinlock or...
//EXPORT_SYMBOL(my_exported_variable); // In module 1
//extern int my_exported_variable;     // In module 2

module_init(ili9325_init);
module_exit(ili9325_exit);

MODULE_DESCRIPTION("ili9325 LCD framebuffer driver");
MODULE_AUTHOR("Matt & mix-n-match");
MODULE_LICENSE("GPL");
