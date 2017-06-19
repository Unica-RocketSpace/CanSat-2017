#include <stdbool.h>
#include <stdint.h>

#include <stm32f10x_conf.h>

#include "OV7670.h"
#include "defs_OV7670.h"
#include "defs_hw.h"
#include "sccb.h"
#include "tim_dma_control.h"

#define VSYNC_INT 2
#define HREF_INT 1

#define MODE_RGB444	0
#define MODE_RGB555	1
#define MODE_RGB565	2
#define MODE_YUV 	3




static uint8_t _transfer_regvals(struct regval_list *list)
{
	uint8_t ret = 0;
	uint8_t i = 0;

	for(;;) {
		// end marker check
		if ((list[i].reg_num == OV7670_EM) && (list[i].value == OV7670_EM)) {
			return 1;
		}

		ret = sccb_write(list[i].reg_num, list[i].value);
		if (!ret) {
			return i;
		}

		// delay for reset command
		if ((list[i].reg_num == REG_COM7) && (list[i].value == COM7_RESET)) {
			_delay_ms(200);
		}

		i++;
	}

	return 0;
}


static void _camera_reset() {
	sccb_write(OV7670_I2C_ADDR, REG_COM7, COM7_RESET);
	_delay_ms(200);
}


void ov7670_init() {
	sccb_init();
	tdcs_init();

	ov7670_reset(MODE_YUV);
}


uint8_t ov7670_reset(ov7670_mode_t mode)
{
	uint8_t ret = 0;

	init_camera_reset();
	_transfer_regvals(ov7670_qqvga)

	switch (mode) {
	case OV7670_MODE_RGB444:
		_transfer_regvals(ov7670_fmt_rgb444);
		if (ret != 1) return ret;
		break;
	case OV7670_MODE_RGB555:
		ret = init_rgb555_qqvga();
		if (ret != 1) return ret;
		break;
	case OV7670_MODE_RGB565:
		ret = init_rgb565_qqvga();
		if (ret != 1) return ret;
		break;
	case OV7670_MODE_YUV:
		ret = init_yuv_qqvga();
		if (ret != 1) return ret;
		break;

	}
	return ret;
}

