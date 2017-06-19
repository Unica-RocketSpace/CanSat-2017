#ifndef OV7670_H_
#define OV7670_H_


typedef enum
{
	OV7670_MODE_RGB444 = 0,
	OV7670_MODE_RGB555 = 1,
	OV7670_MODE_RGB565 = 2,
	OV7670_MODE_YUV	= 3,
} ov7670_mode_t;

void ov7670_init();
uint8_t ov7670_reset(ov7670_mode_t mode);

uint8_t transfer_regvals(struct regval_list *list);
uint8_t init_yuv_qvga();
void init_camera_reset();
uint8_t init_default_values();


#endif // OV7670_H_
