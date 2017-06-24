#include <stdint.h>
#include <stdlib.h>
#include <stddef.h>
#include <stdbool.h>

#include <stm32f10x_conf.h>

#include <FreeRTOS.h>
#include <task.h>
#include <projdefs.h>

#include "diag/Trace.h"

#include "defs_OV7670.h"
#include "OV7670.h"


// from http://elixir.free-electrons.com/linux/v4.11.6/source/drivers/media/i2c/ov7670.c#L187


static void ov7670_sccb_init()
{
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// GPIO config
	GPIO_InitTypeDef portInit;
	GPIO_StructInit(&portInit);
	portInit.GPIO_Mode =  GPIO_Mode_AF_OD;
	portInit.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	portInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &portInit);

	// I2C config
	I2C_InitTypeDef i2c;
	I2C_StructInit(&i2c);
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_OwnAddress1 = 0x00;
	//i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_Ack = I2C_Ack_Disable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c.I2C_ClockSpeed = 1000;
	I2C_Init(I2C1, &i2c);
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
	I2C_Cmd(I2C1, ENABLE);


	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
	I2C_AcknowledgeConfig(I2C1, DISABLE);
}


static int ov7670_read(unsigned char reg, unsigned char *value)
{
	uint32_t timeout = 0x7FFFFF;
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		if ((timeout--) == 0) {
			trace_printf("Busy Timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send start bit
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			trace_printf("Start bit Timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send slave address (camera write address)
	I2C_Send7bitAddress(I2C1, OV7670_SCCB_WRITE_ADDR, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			trace_printf("Slave address timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send register address
	I2C_SendData(I2C1, reg);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			trace_printf("Register timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}



	// Send start again
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			trace_printf("Start bit Timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send slave address (camera read address)
	I2C_Send7bitAddress(I2C1, OV7670_SCCB_WRITE_ADDR, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			trace_printf("Slave address timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Read reg value
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if ((timeout--) == 0) {
			trace_printf("Value timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}
	*value = I2C_ReceiveData(I2C1);

	// Send stop bit
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 0;

}


static int ov7670_write(unsigned char reg, unsigned char value)
{
	uint32_t timeout = 0x7FFFFF;
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		if ((timeout--) == 0) {
			trace_printf("Busy Timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send start bit
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			trace_printf("Start bit Timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send slave address (camera write address)
	I2C_Send7bitAddress(I2C1, OV7670_SCCB_WRITE_ADDR, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			trace_printf("Slave address timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send register address
	I2C_SendData(I2C1, reg);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			trace_printf("Register timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send new register value
	I2C_SendData(I2C1, value);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			trace_printf("Value timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send stop bit
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 0;
}


static int ov7670_write_array(const struct regval_list *vals)
{
	while (vals->reg_num != 0xff || vals->value != 0xff) {
		int ret = ov7670_write(vals->reg_num, vals->value);
		if (ret < 0)
			return ret;
		vals++;
	}
	return 0;
}


inline static void msleep(int cnt)
{
	vTaskDelay(cnt / portTICK_PERIOD_MS);
}

/*
 * Information we maintain about a known sensor.
 */
struct ov7670_format_struct;  /* coming later */
struct ov7670_info {
	struct ov7670_format_struct *fmt;  /* Current format */
	//int min_width;			/* Filter out smaller sizes */
	//int min_height;			/* Filter out smaller sizes */
	int clock_speed;		/* External clock speed (MHz) */
	uint8_t clkrc;			/* Clock divider value */
	//bool use_smbus;			/* Use smbus I/O instead of I2C */
	//bool pll_bypass;
	bool pclk_hb_disable;
	//const struct ov7670_devtype *devtype; /* Device specifics */
};

static struct ov7670_info _info;


/*
 * Here we'll try to encapsulate the changes for just the output
 * video format.
 *
 * RGB656 and YUV422 come from OV; RGB444 is homebrewed.
 *
 * IMPORTANT RULE: the first entry must be for COM7, see ov7670_s_fmt for why.
 */

struct ov7670_win_size {
	int	width;
	int	height;
	unsigned char com7_bit;
	int	hstart;		/* Start/stop values for the camera.  Note */
	int	hstop;		/* that they do not always make complete */
	int	vstart;		/* sense to humans, but evidently the sensor */
	int	vstop;		/* will do the right thing... */
	const struct regval_list *regs; /* Regs to tweak */
};


/*
 * Then there is the issue of window sizes.  Try to capture the info here.
 */
static struct ov7670_win_size ov7670_win_sizes[] = {
	/* VGA */
	{
		.width		= VGA_WIDTH,
		.height		= VGA_HEIGHT,
		.com7_bit	= COM7_FMT_VGA,
		.hstart		= 158,	/* These values from */
		.hstop		=  14,	/* Omnivision */
		.vstart		=  10,
		.vstop		= 490,
		.regs		= NULL,
	},
	/* CIF */
	{
		.width		= CIF_WIDTH,
		.height		= CIF_HEIGHT,
		.com7_bit	= COM7_FMT_CIF,
		.hstart		= 170,	/* Empirically determined */
		.hstop		=  90,
		.vstart		=  14,
		.vstop		= 494,
		.regs		= NULL,
	},
	/* QVGA */
	{
		.width		= QVGA_WIDTH,
		.height		= QVGA_HEIGHT,
		.com7_bit	= COM7_FMT_QVGA,
		.hstart		= 168,	/* Empirically determined */
		.hstop		=  24,
		.vstart		=  12,
		.vstop		= 492,
		.regs		= NULL,
	},
	/* QCIF */
	{
		.width		= QCIF_WIDTH,
		.height		= QCIF_HEIGHT,
		.com7_bit	= COM7_FMT_VGA, /* see comment above */
		.hstart		= 456,	/* Empirically determined */
		.hstop		=  24,
		.vstart		=  14,
		.vstop		= 494,
		.regs		= ov7670_qcif_regs,
	}
};


/*
 * Store a set of start/stop values into the camera.
 */
static int _ov7670_set_hw(int hstart, int hstop,	int vstart, int vstop)
{
	int ret;
	unsigned char v;
/*
 * Horizontal: 11 bits, top 8 live in hstart and hstop.  Bottom 3 of
 * hstart are in href[2:0], bottom 3 of hstop in href[5:3].  There is
 * a mystery "edge offset" value in the top two bits of href.
 */
	ret =  ov7670_write(REG_HSTART, (hstart >> 3) & 0xff);
	ret += ov7670_write(REG_HSTOP, (hstop >> 3) & 0xff);
	ret += ov7670_read(REG_HREF, &v);
	v = (v & 0xc0) | ((hstop & 0x7) << 3) | (hstart & 0x7);
	msleep(10);
	ret += ov7670_write(REG_HREF, v);
/*
 * Vertical: similar arrangement, but only 10 bits.
 */
	ret += ov7670_write(REG_VSTART, (vstart >> 2) & 0xff);
	ret += ov7670_write(REG_VSTOP, (vstop >> 2) & 0xff);
	ret += ov7670_read(REG_VREF, &v);
	v = (v & 0xf0) | ((vstop & 0x3) << 2) | (vstart & 0x3);
	msleep(10);
	ret += ov7670_write(REG_VREF, v);
	return ret;
}


/*
 * Store information about the video data format.  The color matrix
 * is deeply tied into the format, so keep the relevant values here.
 * The magic matrix numbers come from OmniVision.
 */
static struct ov7670_format_struct {
	const struct regval_list *regs;
	int cmatrix[CMATRIX_LEN];
} ov7670_formats[] = {
	{
		.regs 		= ov7670_fmt_yuv422,
		.cmatrix	= { 128, -128, 0, -34, -94, 128 },
	},
	{
		.regs		= ov7670_fmt_rgb444,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
	},
	{
		.regs		= ov7670_fmt_rgb565,
		.cmatrix	= { 179, -179, 0, -61, -176, 228 },
	},
	{
		.regs 		= ov7670_fmt_raw,
		.cmatrix	= { 0, 0, 0, 0, 0, 0 },
	},
};



/*
 * Stuff that knows about the sensor.
 */
int ov7670_reset(void)
{
	ov7670_write(REG_COM7, COM7_RESET);
	msleep(1);
	return 0;
}


static int ov7670_detect(void)
{
	unsigned char v;
	int ret;

	ret = ov7670_write_array(ov7670_default_regs);
	if (ret < 0)
		return ret;
	ret = ov7670_read(REG_MIDH, &v);
	if (ret < 0)
		return ret;
	if (v != 0x7f) /* OV manuf. id. */
		return -pdFREERTOS_ERRNO_ENODEV;
	ret = ov7670_read(REG_MIDL, &v);
	if (ret < 0)
		return ret;
	if (v != 0xa2)
		return -pdFREERTOS_ERRNO_ENODEV;
	/*
	 * OK, we know we have an OmniVision chip...but which one?
	 */
	ret = ov7670_read(REG_PID, &v);
	if (ret < 0)
		return ret;
	if (v != 0x76)  /* PID + VER = 0x76 / 0x73 */
		return -pdFREERTOS_ERRNO_ENODEV;
	ret = ov7670_read(REG_VER, &v);
	if (ret < 0)
		return ret;
	if (v != 0x73)  /* PID + VER = 0x76 / 0x73 */
		return -pdFREERTOS_ERRNO_ENODEV;
	return 0;
}


int ov7670_init(struct ov7670_config * config)
{
	ov7670_sccb_init();

	struct ov7670_info * const info = &_info;
	struct v4l2_fract tpf;
	int ret;
	info->clock_speed = 30; /* default: a guess */

	/*
	 * Must apply configuration before initializing device, because it
	 * selects I/O method.
	 */
	//info->min_width = config->min_width;
	//info->min_height = config->min_height;
	//info->use_smbus = config->use_smbus;

	if (config->clock_speed)
		info->clock_speed = config->clock_speed;

	/*
	 * It should be allowed for ov7670 too when it is migrated to
	 * the new frame rate formula.
	 */
	if (config->pclk_hb_disable)
		info->pclk_hb_disable = true;

	/* Make sure it's an ov7670 */
	ret = ov7670_detect();
	if (ret) {
		return ret;
	}

	info->fmt = &ov7670_formats[0];
	info->clkrc = 0;

	/* Set default frame rate to 30 fps */
	tpf.numerator = 1;
	tpf.denominator = 30;
	ov7670_set_framerate_legacy(&tpf);

	if (info->pclk_hb_disable)
		ov7670_write(REG_COM10, COM10_PCLK_HB);

	return 0;
}



int ov7670_set_fmt(ov7670_fmt_key_t fmt, ov7670_window_size_key_t ws)
{

	unsigned char com7;
	int ret;
	struct ov7670_format_struct * ovfmt = &ov7670_formats[fmt];
	struct ov7670_win_size * wsize = &ov7670_win_sizes[ws];

	/*
	 * COM7 is a pain in the ass, it doesn't like to be read then
	 * quickly written afterward.  But we have everything we need
	 * to set it absolutely here, as long as the format-specific
	 * register sets list it first.
	 */
	com7 = ovfmt->regs[0].value;
	com7 |= wsize->com7_bit;
	ov7670_write(REG_COM7, com7);
	/*
	 * Now write the rest of the array.  Also store start/stops
	 */
	ov7670_write_array(ovfmt->regs + 1);
	_ov7670_set_hw(wsize->hstart, wsize->hstop, wsize->vstart,
			wsize->vstop);
	ret = 0;
	if (wsize->regs)
		ret = ov7670_write_array(wsize->regs);

	/*
	 * If we're running RGB565, we must rewrite clkrc after setting
	 * the other parameters or the image looks poor.  If we're *not*
	 * doing RGB565, we must not rewrite clkrc or the image looks
	 * *really* poor.
	 *
	 * (Update) Now that we retain clkrc state, we should be able
	 * to write it unconditionally, and that will make the frame
	 * rate persistent too.
	 */
	/*
	if (ret == 0)
		ret = ov7670_write(REG_CLKRC, info->clkrc);
	*/
	return ret;
}


void ov7670_get_framerate_legacy(struct v4l2_fract *tpf)
{
	struct ov7670_info * const info = &_info;

	tpf->numerator = 1;
	tpf->denominator = info->clock_speed;
	if ((info->clkrc & CLK_EXT) == 0 && (info->clkrc & CLK_SCALE) > 1)
		tpf->denominator /= (info->clkrc & CLK_SCALE);
}


int ov7670_set_framerate_legacy(struct v4l2_fract *tpf)
{
	struct ov7670_info * const info = &_info;
	int div;

	if (tpf->numerator == 0 || tpf->denominator == 0)
		div = 1;  /* Reset to full rate */
	else
		div = (tpf->numerator * info->clock_speed) / tpf->denominator;
	if (div == 0)
		div = 1;
	else if (div > CLK_SCALE)
		div = CLK_SCALE;
	info->clkrc = (info->clkrc & 0x80) | div;
	tpf->numerator = 1;
	tpf->denominator = info->clock_speed / div;
	return ov7670_write(REG_CLKRC, info->clkrc);
}
