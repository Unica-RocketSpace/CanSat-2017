/*
 * hw_defs.h
 *
 *  Created on: 19 июн. 2017 г.
 *      Author: snork
 */

#ifndef OV7670_DEFS_HW_H_
#define OV7670_DEFS_HW_H_

#include <stm32f10x_conf.h>

//Порт A
#define VIDEO_DATA		GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7
#define VIDEO_RRST		GPIO_Pin_9
#define VIDEO_WRST		GPIO_Pin_10
#define VIDEO_HREF		GPIO_Pin_11
#define VIDEO_VSYNC		GPIO_Pin_12

//Порт B
#define VIDEO_OE		GPIO_Pin_0
#define VIDEO_WE		GPIO_Pin_1
#define VIDEO_CLK		GPIO_Pin_10



#endif /* OV7670_DEFS_HW_H_ */
