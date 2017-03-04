/*
 * I2C_sensors.h
 *
 *  Created on: 21 янв. 2017 г.
 *      Author: developer
 */

#ifndef I2C_SENSORS_H_
#define I2C_SENSORS_H_

#define MPU9255_ADR 0b1101000	// если AD0 = 0, иначе адрес 1101001
#define KOMPASS_ADR 0b0001100	// если CAD1 = CAD0 = 0, иначе адрес другой; режим pass-through mode
#define PRESSURE_ADR

extern rscs_i2c_bus_t i2c_;
extern rscs_i2c_bus_t * i2c;

void sawsarp(uint8_t adr, uint8_t data_write, uint8_t *data_read, uint8_t n_read);	//I2C read-write
void sawwp(uint8_t adr, uint8_t data_write1, uint8_t data_write2);					//I2C multiple write
void I22C_init();		//initialisation of the I2C wire
void I22C_gyro_read();	//reading data of the gyroscope

#endif /* I2C_SENSORS_H_ */
