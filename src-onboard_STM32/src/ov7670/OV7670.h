/*
 * registers.h
 *
 *  Created on: 15 июня 2017 г.
 *      Author: developer
 */

#ifndef OV7670_OV7670_H_
#define OV7670_OV7670_H_

#include <stdint.h>
#include <stdbool.h>


typedef enum
{
	OV7670_FMT_KEY_YUV422 = 0,
	OV7670_FMT_KEY_RGB444,
	OV7670_FMT_KEY_RGB565,
	OV7670_FMT_KEY_RAW,
} ov7670_fmt_key_t;


typedef enum
{
	OV7670_WINDOW_SIZE_KEY_VGA = 0,
	OV7670_WINDOW_SIZE_KEY_CIF,
	OV7670_WINDOW_SIZE_KEY_QVGA,
	OV7670_WINDOW_SIZE_KEY_QCIF,
} ov7670_window_size_key_t;

struct v4l2_fract {
	uint32_t   numerator;
	uint32_t   denominator;
};

struct ov7670_config {
	//int min_width;			/* Filter out smaller sizes */
	//int min_height;			/* Filter out smaller sizes */
	int clock_speed;		/* External clock speed (MHz) */
	//bool pll_bypass;		/* Choose whether to bypass the PLL */
	bool pclk_hb_disable;	/* Disable toggling pixclk during horizontal blanking */
};


int ov7670_init(struct ov7670_config * config);
int ov7670_reset(void);
int ov7670_set_fmt(ov7670_fmt_key_t fmt, ov7670_window_size_key_t ws);
void ov7670_get_framerate_legacy(struct v4l2_fract *tpf);
int ov7670_set_framerate_legacy(struct v4l2_fract *tpf);

#endif /* OV7670_OV7670_H_ */
