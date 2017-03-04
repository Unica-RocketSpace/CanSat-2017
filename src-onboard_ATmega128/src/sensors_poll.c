/*

 * sensors_poll.c
 *
 *  Created on: 26 нояб. 2016 г.
 *      Author: developer
 */

#include <avr/io.h>
#include <stdio.h>
#include <rscs/i2c.h>
#include <stdlib.h>
#include <rscs/spi.h>
#include <avr/interrupt.h>

#include "ADXL345.h"
#include "I2C_sensors.h"
#include "sensors_poll.h"


//ИНИЦИАЛИЗАЦИЯ ШИНЫ SPI (временно)
rscs_spi_bus_t spi1_;
rscs_spi_bus_t * spi1 = &spi1_;

ISR(INT4_vect)
{
	sawsarp(MPU9255_ADR, 116, FIFO_data, 6);
}

ISR(INT6_vect)
{
	ADXL345_GetGXYZ(&ADXL_raw_data[0],	&ADXL_raw_data[1],	&ADXL_raw_data[2],
					&ADXL_G_data[0],	&ADXL_G_data[1],	&ADXL_G_data[2]);

}


