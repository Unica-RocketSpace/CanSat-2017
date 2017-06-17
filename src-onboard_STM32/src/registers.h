/*
 * registers.h
 *
 *  Created on: 15 июня 2017 г.
 *      Author: developer
 */

#ifndef REGISTERS_H_
#define REGISTERS_H_

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


#endif /* REGISTERS_H_ */
