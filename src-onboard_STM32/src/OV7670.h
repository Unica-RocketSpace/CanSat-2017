/*
 * Ov7670.h
 *
 *  Created on: Aug 26, 2013
 *      Author: arndtjenssen
 */

#ifndef OV7670_H_
#define OV7670_H_

#include <stdint.h>
#include "OV7670_config.h"
//#include "fifo.h"
//#include "SimpleI2C.h"
//#include <HardwareSerial.h>

#define VSYNC_INT 2
#define HREF_INT 1

#define MODE_RGB444	0
#define MODE_RGB555	1
#define MODE_RGB565 	2
#define MODE_YUV 		3

#define SIZEX (160)
#define SIZEY (120)

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))


typedef struct
{
	volatile bool	capture_request;
	volatile bool	capture_done;
	volatile bool	busy;
	uint8_t init_success;
	volatile uint16_t	line_counter;
	volatile uint16_t	last_line_counter;
	volatile uint8_t	camera_mode;
	uint16_t bufferPos;
	bool edgeEnhacementEnabled;
	bool denoiseEnabled;
	uint8_t buffer[SIZEX * 3];

} OV7670_struct;

extern OV7670_struct OV7670;

//PUBLIC FUNCTIONS
void OV7670_init();
uint8_t OV7670_reset(uint8_t mode);
void OV7670_capture_image();
//void OV7670_setSerial(HardwareSerial *s);
void OV7670_vsync_handler();
void OV7670_href_handler();
void OV7670_nightMode(bool enable);
void OV7670_contrast(int8_t);
void OV7670_brightness(int8_t);
void OV7670_specialEffect(uint8_t);
void OV7670_edgeEnhancement(uint8_t);
void OV7670_denoise(uint8_t);

//PRIVATE FUNCTIONS
void OV7670_read_stop();
void OV7670_capture_next();
uint8_t OV7670_captured();
uint8_t OV7670_read_one_byte();
uint8_t OV7670_transfer_regvals(struct regval_list *list);
uint8_t OV7670_init_rgb444_qvga();
uint8_t OV7670_init_rgb555_qvga();
uint8_t OV7670_init_rgb565_qvga();
uint8_t OV7670_init_yuv_qvga();
void OV7670_init_negative_vsync();
void OV7670_init_camera_reset();
uint8_t OV7670_init_default_values();

#endif /* OV7670_H_ */
