/*
 * sccb.h
 *
 *  Created on: 20 июня 2017 г.
 *      Author: developer
 */

#ifndef OV7670_SCCB_H_
#define OV7670_SCCB_H_

#define SIO_C_Pin			6
#define SIO_D_Pin			7

#define SCCB_CLOCK_SPEED	100000

typedef struct
{
	uint32_t sccb_ClockSpeed;

}sccb_InitTypeDef;



#endif /* OV7670_SCCB_H_ */
