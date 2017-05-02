/*
 * MPU9255.h
 *
 *  Created on: 21 янв. 2017 г.
 *      Author: developer
 */

#ifndef MPU9255_H_
#define MPU9255_H_

#include <rscs/i2c.h>

#define MPU9255_ADR 0b1101000	// если AD0 = 0, иначе адрес 1101001
#define KOMPASS_ADR 0b0001100	// если CAD1 = CAD0 = 0, иначе адрес другой; режим pass-through mode

typedef enum
{
	GYRO_AND_ACCEL = 0b1101000,
	COMPASS = 0b0001100,

}MPU9255_address;


rscs_e MPU9255_read_register(MPU9255_address adr, uint8_t reg_address, uint8_t *data_read, uint8_t n_read);	//I2C read-write
rscs_e MPU9255_write_regiter(MPU9255_address adr, uint8_t reg_address, uint8_t data);					//I2C multiple write

rscs_e MPU9255_read_imu(int16_t * raw_accel_XYZ, int16_t * raw_gyro_XYZ);	//чтение данных с акселерометра и гироскопа
rscs_e MPU9255_read_compass(int16_t * raw_compass_XYZ);					//чтение данных с магнитометра


rscs_e MPU9255_recalc_accel(const int16_t * raw_accel_XYZ, float * accel_XYZ);	//перевод показаний акселерометра в единицы g
rscs_e MPU9255_recalc_gyro(const int16_t * raw_gyro_XYZ, float * gyro_XYZ);		//перевод показаний гироскопа в единицы dps
rscs_e MPU9255_recalc_compass(const int16_t * raw_compass_XYZ, float * compass_XYZ);

void MPU9255_init();		//initialisation of the I2C wire


#endif /* MPU9255_H_ */
