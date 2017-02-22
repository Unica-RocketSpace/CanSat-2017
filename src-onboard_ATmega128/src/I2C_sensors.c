/*
 * I2C_sensors.c
 *
 *  Created on: 21 янв. 2017 г.
 *      Author: developer
 */

#include <avr/io.h>
#include <stdio.h>
#include <rscs/i2c.h>
#include <stdlib.h>

#include "I2C_sensors.h"

rscs_i2c_bus_t i2c_;
rscs_i2c_bus_t * i22c = &i2c_;

uint8_t FIFO_data[3];

void sawsarp(uint8_t adr, uint8_t data_write, uint8_t *data_read, uint8_t n_read)
{
	do {
		rscs_i2c_start(i22c);
		if (rscs_i2c_send_slaw(i22c, adr, 0))	{rscs_i2c_reset(i22c);continue;}
		if (rscs_i2c_write(i22c, &data_write, 1))	{rscs_i2c_reset(i22c);continue;}
		rscs_i2c_start(i22c);
		if (rscs_i2c_send_slaw(i22c, adr, 1))	{rscs_i2c_reset(i22c);continue;}
		if (rscs_i2c_read(i22c, data_read, n_read, 1))	{rscs_i2c_reset(i22c);continue;}
		rscs_i2c_stop(i22c);
		break;
	}
	while(1);
}

void sawwp(uint8_t adr, uint8_t data_write1, uint8_t data_write2)
{
	do {
		rscs_i2c_start(i22c);
		if (rscs_i2c_send_slaw(i22c, adr, 0))	{rscs_i2c_reset(i22c);continue;}
		if (rscs_i2c_write(i22c, &data_write1, 1))	{rscs_i2c_reset(i22c);continue;}
		if (rscs_i2c_send_slaw(i22c, adr, 0))	{rscs_i2c_reset(i22c);continue;}
		if (rscs_i2c_write(i22c, &data_write2, 1))	{rscs_i2c_reset(i22c);continue;}
		rscs_i2c_stop(i22c);
		break;
	}
	while(1);
}

void I22C_init()
{
	sawwp(MPU9255_ADR, 26,	0b01000000);	//config
	sawwp(MPU9255_ADR, 27,	0b00001000);	//gyro config
	sawwp(MPU9255_ADR, 28,	0x00); 			//accel config
	sawwp(MPU9255_ADR, 29,	0x00);			//accel config 2
	sawwp(MPU9255_ADR, 35,	0b01110000);	//FIFO enable
	sawwp(MPU9255_ADR, 56,	0b01000001);	//interrupt enable
	sawwp(MPU9255_ADR, 106,	0b01000100);	//user control
	sawwp(MPU9255_ADR, 107,	0b00000001);	//power managment 1
	sawwp(MPU9255_ADR, 108,	0b00111000);	//power managment 2s
}

void I22C_gyro_read()
{
	sawwp(MPU9255_ADR, 106,	0b01000100);	//reset FIFO
	EIMSK |= (1 << 4);						//Разрешаем прерывания INT4 от гироскопа
	EIMSK &= ~(1 << 4);						//Запрещаем прерывания INT4 от гироскопа ?????????????????
}

/*void I22C_magnetometer_read()
{
	sawwp(KOMPASS_ADR, 106,	0b01000100);
}*/
