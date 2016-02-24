/*
 * st7920.c
 *
 *  Created on: 08 янв. 2015 г.
 *      Author: Dmitriy Vostrikov <dsvost@gmail.com>
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/spi/spi.h>
#include <linux/mm.h>
#include <linux/vmalloc.h>
#include <linux/delay.h>
#include <linux/uaccess.h>
#include <st7920fb.h>

/**
 *	st7920fb_release - Optional function. Called when the framebuffer
 *			device is closed.
 *	@info: frame buffer structure that represents a single frame buffer
 *	@user: tell us if the userland (value=1) or the console is accessing
 *	       the framebuffer.
 *
 *	Thus function is called when we close /dev/fb or the framebuffer
 *	console system is released. Usually you don't need this function.
 *	The case where it is usually used is to go from a graphics state
 *	to a text mode state.
 *
 *	Returns negative errno on error, or zero on success.
 */
static int st7920fb_release(struct fb_info *info, int user)
{
    return 0;
}

/*static void st7920_write_cmd(uint8_t rs, uint8_t rw, uint8_t param) {

	if (rs==0) {
		gspi_buf[0] = 0x0F8;
	} else {
		gspi_buf[0] = 0x0FA;
	}

	gspi_buf[1] = (param & 0x0F0);
	gspi_buf[2] = (param<<4);

	spi_write(gspi_dev, gspi_buf, 3);
}*/

static void st7920_set_ex_mode(uint8_t dl, uint8_t re, uint8_t gd_state){
	st7920_write_cmd(0, 0, (1<<5)|(dl<<4)|(re<<2)|(gd_state<<1));
}

/*static void st7920_set_gdram_addr(uint8_t vaddr, uint8_t haddr) {
	st7920_write_cmd(0, 0, vaddr|(1<<7));
	st7920_write_cmd(0, 0, haddr|(1<<7));
}*/

/*static void st7920_write(uint8_t data) {
	st7920_write_cmd(1, 0, data);
}*/

static void st7920_display_clear(void){

	int v,h;

	st7920_write_cmd(0, 0, 1);
	ndelay(10);
	for (v=0;v<32;v++) {
		st7920_set_gdram_addr(v, 0);

		for (h=0;h<32;h++) {
			st7920_write(0);
			ndelay(10);
		}
	}
}

static void st7920_show_logo(const uint8_t *logo_pixmap){
	st7920_write_buf(logo_pixmap, 1024, 0);
}

static void st7920_display_on(void) {
	st7920_set_ex_mode(0, 0, 0);
	ndelay(10);
	st7920_set_ex_mode(0, 1, 1);
	ndelay(10);
	st7920_display_clear();
	ndelay(1);
	st7920_show_logo(rpi_logo);
//	mdelay(10);
//	st7920_show_logo(rd_logo);
}

/*static uint8_t reverse_byte(uint8_t val) {
	uint8_t nval, i, tval;
	nval = 0;
	for (i=0;i<8;i++) {
		nval <<= 1;
		tval = val & (1 << i);
		nval |= (tval) ? 1 : 0;
	}
	return nval;
}
*/

static void st7920_write_buf(const uint8_t *buf, uint32_t size, loff_t *ppos) {

	uint32_t offset = 0;
	int x,y;
	uint8_t val = 0;

	x=0; y=0;
	//.. необходимо рассчитать x, y в соответствии с ppos
//	mutex_lock(&io_lock);
	for (y=0;y<32;y++) {

		st7920_set_gdram_addr(y,0);
//		ndelay(1000);
		for (x=0;x<16;x++) {
			offset = y*16+x;

			if (offset>size)
				goto small_buffer;

			val = (*(buf + offset));
			//st7920_write(reverse_byte(val));
			st7920_write(val);
		}
		for (x=0;x<16;x++) {
			offset = 512+y*16+x;

			if (offset>size)
				break;

			val = (*(buf + offset));
			//st7920_write(reverse_byte(val));
			st7920_write(val);
		}
	}

small_buffer:
//	mutex_unlock(&io_lock);
	return;
}

static void st7920fb_deferred_io(struct fb_info *info,
			    struct list_head *pagelist) {

	gspi_dev = container_of(info->device, struct spi_device, dev);

	if (!gspi_dev) {
		printk(KERN_NOTICE "ST7920: can't cast fb_info to spi_device");
		return;
	}

	st7920_write_buf(info->screen_base, FB_SIZE, 0);
}

static ssize_t st7920fb_write(struct fb_info *info, const char __user *buf,
			    size_t count, loff_t *ppos){
	// it depend on what current framebuffer settings

	int len;
	struct st7920fb_data *fb_data;

	fb_data = info->par;

	gspi_dev = container_of(info->device, struct spi_device, dev);

	if (!gspi_dev) {
		printk(KERN_NOTICE "ST7920: can't cast fb_info to spi_device");
		return -EINVAL;
	}

	len = (*ppos+count) > (info->fix.smem_len - *ppos)
		? (info->fix.smem_len - *ppos) : count;

	len = copy_from_user(info->screen_base +(*ppos), buf, len);
	schedule_delayed_work(&info->deferred_work, 0);

	return len*fb_data->bpp;
}

static ssize_t st7920fb_read(struct fb_info *info, char __user *buf,
			   size_t count, loff_t *ppos)
{
	int len;
	len = (count > (info->fix.smem_len - *ppos)) ? (info->fix.smem_len - *ppos) : count;
	len = copy_to_user(buf, info->screen_base + *ppos, len);
	*ppos += len;

	return len;
}

static int st7920fb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct st7920fb_data *fb_data = info->par;
	__u32 bpp      = var->bits_per_pixel;
	__u32 activate = var->activate;

	/* only allow 1/8 bit depth (8-bit is grayscale) */
	*var = info->var;
	var->activate = activate;
	if (bpp >= 8) {
		var->bits_per_pixel = 8;
		var->red.length     = 8;
		var->green.length   = 8;
		var->blue.length    = 8;
	} else {
		var->bits_per_pixel = 1;
		var->red.length     = 1;
		var->green.length   = 1;
		var->blue.length    = 1;
	}

	fb_data->bpp = bpp;

	return 0;
}


static int st7920fb_set_par(struct fb_info *info) {
	struct st7920fb_data *fbdata = NULL;
	u8 *tmp_fb, *o_fb;
	fbdata	= info->par;
	if (info->var.bits_per_pixel == fbdata->bpp)
		return 0;
	/* switch between 1/8 bit depths */
//	if (info->var.bits_per_pixel != 1 && fbdata->bpp !=8)
//		return -EINVAL;

	o_fb = info->screen_base;
	tmp_fb = kmalloc(FB_SIZE, GFP_KERNEL);
	if (!tmp_fb)
		return -ENOMEM;

	printk(KERN_NOTICE "st7920: before translate FB content");

	/* translate FB content to new bits-per-pixel */
	if (info->var.bits_per_pixel == 1) {
		int i, b;
		for (i = 0; i < FB_SIZE; i++) {
			u8 p = 0;
			for (b = 0; b < 8; b++) {
				p <<= 1;
				p |= o_fb[i] ? 0x01 : 0x00;
			}
			tmp_fb[i] = p;
		}
		memcpy(o_fb, tmp_fb, FB_SIZE);
		info->fix.visual = FB_VISUAL_MONO01;
		info->fix.line_length = FB_LINE_WIDTH;
	} else {
		int i;
		memcpy(tmp_fb, o_fb, FB_SIZE);
		for (i = 0; i < FB_SIZE; i++)
			o_fb[i] = tmp_fb[i/8] & (0x01 << (7 - i % 8)) ? 0xff : 0x00;
		info->fix.visual = FB_VISUAL_PSEUDOCOLOR;
		info->fix.line_length = FB_LINE_WIDTH*8;
	}

	kfree(tmp_fb);
	fbdata->bpp = info->var.bits_per_pixel;

	schedule_delayed_work(&info->deferred_work, 0);

	return 0;
}

static int st7920fb_setcolreg(u_int regno, u_int red, u_int green, u_int blue,
			 u_int transp, struct fb_info *info)
{
	if (regno >= 256)	/* no. of hw registers */
		return 1;
	/*
	 * Program hardware... do anything you want with transp
	 */

	/* grayscale works only partially under directcolor */
	if (info->var.grayscale) {
		/* grayscale = 0.30*R + 0.59*G + 0.11*B */
		red = green = blue =
		    (red * 77 + green * 151 + blue * 28) >> 8;
	}

	/* Directcolor:
	 *   var->{color}.offset contains start of bitfield
	 *   var->{color}.length contains length of bitfield
	 *   {hardwarespecific} contains width of RAMDAC
	 *   cmap[X] is programmed to (X << red.offset) | (X << green.offset) | (X << blue.offset)
	 *   RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Pseudocolor:
	 *    var->{color}.offset is 0 unless the palette index takes less than
	 *                        bits_per_pixel bits and is stored in the upper
	 *                        bits of the pixel value
	 *    var->{color}.length is set so that 1 << length is the number of available
	 *                        palette entries
	 *    cmap is not used
	 *    RAMDAC[X] is programmed to (red, green, blue)
	 *
	 * Truecolor:
	 *    does not use DAC. Usually 3 are present.
	 *    var->{color}.offset contains start of bitfield
	 *    var->{color}.length contains length of bitfield
	 *    cmap is programmed to (red << red.offset) | (green << green.offset) |
	 *                      (blue << blue.offset) | (transp << transp.offset)
	 *    RAMDAC does not exist
	 */
#define CNVT_TOHW(val,width) ((((val)<<(width))+0x7FFF-(val))>>16)
	switch (info->fix.visual) {
	case FB_VISUAL_TRUECOLOR:
	case FB_VISUAL_PSEUDOCOLOR:
		red = CNVT_TOHW(red, info->var.red.length);
		green = CNVT_TOHW(green, info->var.green.length);
		blue = CNVT_TOHW(blue, info->var.blue.length);
		transp = CNVT_TOHW(transp, info->var.transp.length);
		break;
	case FB_VISUAL_DIRECTCOLOR:
		red = CNVT_TOHW(red, 8);	/* expect 8 bit DAC */
		green = CNVT_TOHW(green, 8);
		blue = CNVT_TOHW(blue, 8);
		/* hey, there is bug in transp handling... */
		transp = CNVT_TOHW(transp, 8);
		break;
	}
#undef CNVT_TOHW
	/* Truecolor has hardware independent palette */
	if (info->fix.visual == FB_VISUAL_TRUECOLOR) {
		u32 v;

		if (regno >= 16)
			return 1;

		v = (red << info->var.red.offset) |
		    (green << info->var.green.offset) |
		    (blue << info->var.blue.offset) |
		    (transp << info->var.transp.offset);
		switch (info->var.bits_per_pixel) {
		case 8:
			break;
		case 16:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		case 24:
		case 32:
			((u32 *) (info->pseudo_palette))[regno] = v;
			break;
		}
		return 0;
	}
	return 0;
}


static void st7920fb_fillrect(struct fb_info *info, const struct fb_fillrect *rect) {
	// sys_fillrect(info, rect);
	cfb_fillrect(info, rect);
	schedule_delayed_work(&info->deferred_work, 0);
}

static void st7920fb_copyarea(struct fb_info *info, const struct fb_copyarea *region) {
	//sys_copyarea(info, region);
	cfb_copyarea(info, region);
	schedule_delayed_work(&info->deferred_work, 0);
}

static void st7920fb_imageblit(struct fb_info *info, const struct fb_image *image) {
//	sys_imageblit(info, image);
	cfb_imageblit(info, image);
	schedule_delayed_work(&info->deferred_work, 0);
}

static struct fb_ops st7920fb_ops = {
	.owner			= THIS_MODULE,
	.fb_release		= st7920fb_release,
	.fb_fillrect	= /*cfb_fillrect*/ st7920fb_fillrect,
	.fb_copyarea	= /*cfb_copyarea*/ st7920fb_copyarea,
	.fb_imageblit	= /*cfb_imageblit*/ st7920fb_imageblit,
//	.fb_ioctl		= st7920fb_ioctl,
	.fb_write		= st7920fb_write,
	.fb_read		= st7920fb_read,
	.fb_check_var	= st7920fb_check_var,
	.fb_set_par		= st7920fb_set_par,
	.fb_setcolreg	= st7920fb_setcolreg,
};

static struct fb_deferred_io st7920fb_defio = {
		.delay			= HZ/8,
//		.delay			= 1,
		.deferred_io	= st7920fb_deferred_io,
};

static int st7920fb_probe(struct spi_device *pdev)
{
	struct fb_info *info;
	unsigned char *videomemory;
	struct st7920fb_data *fbdata = NULL;
    struct fb_videomode fbmode;

    struct device *device = &pdev->dev;
    pdev->bits_per_word = 8;
    pdev->max_speed_hz = 1 * 1000 * 1000;
//	pdev->max_speed_hz = 600 * 1000;
    pdev->mode = SPI_MODE_3 | SPI_CS_HIGH; //SPI_MODE_3

    spi_setup(pdev);

    info = framebuffer_alloc(0, device);

    if (!info)
	    goto out;

    info->fbops = &st7920fb_ops;
    info->fix = st7920fb_fix;
    info->var = st7920fb_var;
	info->flags = FBINFO_DEFAULT | FBINFO_VIRTFB;

	videomemory = vzalloc(info->fix.smem_len*8);
	if (!videomemory)
		goto mem_fault;

    info->screen_base = (char __force __iomem *)videomemory;

    info->screen_size = info->fix.smem_len;

    st7920fb_fix.mmio_start = (unsigned long)(char __force __iomem *)videomemory;
    st7920fb_fix.mmio_len = FB_SIZE;

//	pallete = vmalloc(256*sizeof(u32));
//	int i;
//	for (i=0;i<256;i++)
//		pallete[i] = i>0 && i<16 ? 0xff : 0;
//	info->pseudo_palette = pallete;
//
	info->fbdefio = &st7920fb_defio;

	fbdata = kzalloc(sizeof(struct st7920fb_data), GFP_KERNEL);
	if (!fbdata)
		goto mem_fault;

	memset(fbdata,0,sizeof(fbdata));
	fbdata->bpp = 1;
	info->par = fbdata;

    //INIT_LIST_HEAD(info->modelist)

  // fb_alloc_cmap(&info->cmap, 256, 0);

    spi_set_drvdata(pdev, info);

	gspi_dev = container_of(info->device, struct spi_device, dev);

	if (!gspi_dev) {
		printk(KERN_NOTICE "ST7920: can't cast fb_info to spi_device");
		goto out;
	}

	gspi_buf = kzalloc(3, GFP_KERNEL|GFP_DMA);
	st7920_display_on();

//	mutex_init(&io_lock);

    if (register_framebuffer(info) < 0)
    	goto register_fault;

	/* add selected videomode to modelist */
	fb_var_to_videomode(&fbmode, &info->var);
	fb_add_videomode(&fbmode, &info->modelist);

	fb_deferred_io_init(info);

	info->state = FBINFO_STATE_RUNNING;

    printk(KERN_NOTICE "fb%d: %s frame buffer device\n", info->node,
	   info->fix.id);

    return 0;

register_fault:
//	fb_dealloc_cmap(&info->cmap);
mem_fault:
	framebuffer_release(info);
out:
	printk(KERN_NOTICE "st7920 failed");
	return -EINVAL;
}

static int st7920fb_remove(struct spi_device *pdev)
{
	struct fb_info *info = spi_get_drvdata(pdev);

	if (info) {
		if (!unregister_framebuffer(info))
			return -EINVAL;

	//fb_dealloc_cmap(&info->cmap);
		fb_deferred_io_cleanup(info);
		kfree(info->par);
		framebuffer_release(info);
		kfree(gspi_buf);
	}

	return 0;
}

#define st7920fb_suspend NULL
#define st7920fb_resume NULL
//#endif /* CONFIG_PM */

static struct spi_driver st7920fb_driver = {
	.probe = st7920fb_probe,
	.remove = st7920fb_remove,
	.suspend = st7920fb_suspend,
	.resume = st7920fb_resume,
	.driver = {
		.name = "st7920fb",
		.owner = THIS_MODULE,
	},
};

module_spi_driver(st7920fb_driver);


MODULE_LICENSE("GPL");
MODULE_AUTHOR("Dmitriy Vostrikov <dsvost@gmail.com>");
MODULE_DESCRIPTION("Linux Framebuffer implementation for GLCD on ST7920 controller");
