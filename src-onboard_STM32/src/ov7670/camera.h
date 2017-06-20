/*
 * fifo.h
 *
 *  Created on: 20 июня 2017 г.
 *      Author: Vasiliy
 */

#ifndef OV7670_CAMERA_H_
#define OV7670_CAMERA_H_

#include <stddef.h>
#include <stdint.h>

int camera_init();

void camera_wait_vsync();

void camera_fifo_reset_write();
void camera_fifo_start_capture(size_t href_count);

void camera_fifo_reset_read();
void camera_fifo_read(void * buffer, size_t bufferSize);


#endif /* OV7670_FIFO_H_ */
