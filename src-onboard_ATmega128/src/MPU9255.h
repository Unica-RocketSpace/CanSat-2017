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
//#define MPU9255_ACCEL_SCALE_FACTOR	0.00062217
#define MPU9255_GYRO_SCALE_FACTOR	0.00013323
#define MPU9255_ACCEL_SCALE_FACTOR	0.00055217

#define X_ACCEL_OFFSET		0x003F//( 0x7FC1 )
#define Y_ACCEL_OFFSET		0x003E//( 0x7FC2 )
#define Z_ACCEL_OFFSET		0x0067//( 0x7F99 )
#define X_ACCEL_KOEFF		1.0796414
#define Y_ACCEL_KOEFF		1.0833494
#define Z_ACCEL_KOEFF		1.0717212

#define GOTO_END_IF_ERROR(op) if ((error = op) != RSCS_E_NONE) goto end;
#define ACCEL_RANGE 1					//2g - 00, 4g - 01, 8g - 10, 16g - 11
#define GYRO_RANGE 1

typedef enum
{
	GYRO_AND_ACCEL = 0b1101000,
	COMPASS = 0b0001100,

}MPU9255_address;


rscs_e MPU9255_read_register(MPU9255_address adr, uint8_t reg_address, uint8_t *data_read, uint8_t n_read);	//I2C read-write
rscs_e MPU9255_write_register(MPU9255_address adr, uint8_t reg_address, uint8_t data);					//I2C multiple write

rscs_e MPU9255_read_imu(int16_t * raw_accel_XYZ, int16_t * raw_gyro_XYZ);	//чтение данных с акселерометра и гироскопа
rscs_e MPU9255_read_compass(int16_t * raw_compass_XYZ);					//чтение данных с магнитометра


void MPU9255_recalc_accel(const int16_t * raw_accel_XYZ, float * accel_XYZ);	//перевод показаний акселерометра в единицы g
void MPU9255_recalc_gyro(const int16_t * raw_gyro_XYZ, float * gyro_XYZ);		//перевод показаний гироскопа в единицы dps
void MPU9255_recalc_compass(const int16_t * raw_compass_XYZ, float * compass_XYZ);

void MPU9255_init();		//initialisation of the I2C wire


#endif /* MPU9255_H_ */
