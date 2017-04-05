/*
 * MPU9255.c
 *
 *  Created on: 21 янв. 2017 г.
 *      Author: developer
 */

#include <avr/io.h>
#include <stdio.h>
#include <rscs/i2c.h>
#include <stdlib.h>
#include <rscs/stdext/stdio.h>
#include <math.h>

#include "MPU9255.h"

uint8_t FIFO_data[3];

#define MPU9255_ACCEL_SCALE_FACTOR	0.0062217
#define MPU9255_GYRO_SCALE_FACTOR	0.00013323
#define GOTO_END_IF_ERROR(op) if ((error = op) != RSCS_E_NONE) goto end;

rscs_e MPU9255_read_register(MPU9255_address adr, uint8_t reg_address, uint8_t *data_read, uint8_t n_read)
{
	rscs_e error;

	GOTO_END_IF_ERROR(rscs_i2c_start());
	GOTO_END_IF_ERROR(rscs_i2c_send_slaw(adr, rscs_i2c_slaw_write));
	GOTO_END_IF_ERROR(rscs_i2c_write(&reg_address, 1));

	GOTO_END_IF_ERROR(rscs_i2c_start());
	GOTO_END_IF_ERROR(rscs_i2c_send_slaw(adr, rscs_i2c_slaw_read));

	GOTO_END_IF_ERROR(rscs_i2c_read(data_read, n_read, 1));

end:
	rscs_i2c_stop();
	return error;
}

rscs_e MPU9255_write_register(MPU9255_address adr, uint8_t reg_address, uint8_t data)
{
	rscs_e error;

	GOTO_END_IF_ERROR(rscs_i2c_start());
	GOTO_END_IF_ERROR(rscs_i2c_send_slaw(adr, rscs_i2c_slaw_write));
	GOTO_END_IF_ERROR(rscs_i2c_write(&reg_address, 1));
	GOTO_END_IF_ERROR(rscs_i2c_write(&data, 1));

end:
	rscs_i2c_stop();
	return error;
}


void MPU9255_init()
{
	MPU9255_write_register(GYRO_AND_ACCEL,	107,	0b10000000);	//RESET

	MPU9255_write_register(GYRO_AND_ACCEL,	25,		0b00000001);	//Sample Rate Divider
	MPU9255_write_register(GYRO_AND_ACCEL,	26,		0b00000001);	//config (DLPF = 001)
	MPU9255_write_register(GYRO_AND_ACCEL,	28,		0b00001000); 	//accel config (rate 4g = 01)
	MPU9255_write_register(GYRO_AND_ACCEL,	29,		0b00000001);	//accel config 2 (Fch_b = 00, DLPF = 001)
	MPU9255_write_register(GYRO_AND_ACCEL,	35,		0b00000000);	//FIFO enable
	MPU9255_write_register(GYRO_AND_ACCEL,	56,		0b00000000);	//interrupt enable (int disable = 0)
	MPU9255_write_register(GYRO_AND_ACCEL,	106,	0b00000000);	//user control
	MPU9255_write_register(GYRO_AND_ACCEL,	107,	0b00000001);	//power managment 1
	MPU9255_write_register(GYRO_AND_ACCEL,	108,	0b00000000);	//power managment 2
	MPU9255_write_register(GYRO_AND_ACCEL,	27,		0b00010000);	//gyro config (rate 500dps = 01, Fch_b = 00)


	MPU9255_write_register(GYRO_AND_ACCEL,	55,		0b00000010);	//режим bypass on
	MPU9255_write_register(COMPASS,		  	0x0A,	0b00010110);	//control 1
	MPU9255_write_register(GYRO_AND_ACCEL,	55,		0b00000000);	//режим bypass off
}


rscs_e MPU9255_read_imu(int16_t * raw_accel_XYZ, int16_t * raw_gyro_XYZ)
{
	rscs_e error = 0;

	GOTO_END_IF_ERROR(MPU9255_read_register(GYRO_AND_ACCEL, 59, (uint8_t*)raw_accel_XYZ, 6));		//чтение данных с акселерометра
	GOTO_END_IF_ERROR(MPU9255_read_register(GYRO_AND_ACCEL, 67, (uint8_t*)raw_gyro_XYZ, 6));	//чтение данных с гироскопа

end:
	return error;
}


rscs_e MPU9255_read_compass(int16_t * raw_compass_XYZ)
{
	rscs_e error = 0;

	uint8_t dummy;

	GOTO_END_IF_ERROR(MPU9255_write_register(GYRO_AND_ACCEL, 55, 0b00000010));	//режим bypass on
	GOTO_END_IF_ERROR(MPU9255_read_register(COMPASS, 0x02, &dummy, 1));
	GOTO_END_IF_ERROR(MPU9255_read_register(COMPASS, 0x03, (uint8_t*)raw_compass_XYZ, 6));
	GOTO_END_IF_ERROR(MPU9255_read_register(COMPASS, 0x09, &dummy, 1));
	GOTO_END_IF_ERROR(MPU9255_write_register(GYRO_AND_ACCEL, 55, 0b00000000));	//режим bypass off

end:
	return error;
}


rscs_e MPU9255_recalc_accel(int16_t * raw_accel_XYZ, float * accel_XYZ)
{
	rscs_e error = 0;

	uint8_t  range = 1;
	uint8_t  a_range_reg;	//диапазон измерений акселерометра из регистра

	/*создание указателей на первые элементы массивов*/
	int16_t *first_raw_accel_XYZ = (int16_t*)raw_accel_XYZ;
	float *first_accel_XYZ = (float*)accel_XYZ;

	GOTO_END_IF_ERROR(MPU9255_read_register(GYRO_AND_ACCEL, 28, &a_range_reg, 1));
	for (size_t i = 0; i < ((a_range_reg & 0b00011000) >> 3); i++) {range = range * 2;}

	for (int i = 0; i < 3; i++)
	{
		first_accel_XYZ[i] = (float)(first_raw_accel_XYZ[i]) * MPU9255_ACCEL_SCALE_FACTOR * range;
	}

end:
	return error;

}


rscs_e MPU9255_recalc_gyro(int16_t * raw_gyro_XYZ, float * gyro_XYZ)
{
	rscs_e error = 0;

	uint8_t  range = 1;
	uint8_t  g_range_reg;	//диапазон измерений гироскопа из регистра
	int16_t *first_raw_gyro_XYZ = (int16_t*)raw_gyro_XYZ;
	float *first_gyro_XYZ = (float*)gyro_XYZ;


	GOTO_END_IF_ERROR(MPU9255_read_register(GYRO_AND_ACCEL, 27, &g_range_reg, 1));
	for (size_t i = 0; i < ((g_range_reg & 0b00011000) >> 3); i++) {range = range * 2;}

	for (int i = 0; i < 3; i++)
	{
		first_gyro_XYZ[i] = (float)(first_raw_gyro_XYZ[i]) * MPU9255_GYRO_SCALE_FACTOR * range;
	}

end:
	return error;

}


/*rscs_e MPU9255_accel_gyro_data(uint16_t * data_read, float * data)
{
	rscs_e error = 0;
	GOTO_END_IF_ERROR(MPU9255_data_read(GYRO_AND_ACCEL, data_read));
	uint8_t  range = 1;
	uint8_t  a_range_reg;	//диапазон измерений акселерометра из регистра
	uint8_t  g_range_reg;	//диапазон измерений гироскопа из регистра
	uint16_t data_tmp;

	uint8_t * data_read_byte = (uint8_t *)data_read;
	float * data_float = data;


	GOTO_END_IF_ERROR(MPU9255_read_register(GYRO_AND_ACCEL, 28, &a_range_reg, 1));
	for (size_t i = 0; i < ((a_range_reg & 0b00011000) >> 3); i++) {range = range * 2;}

	//преобразуем данные акселерометра по оси X
	data_tmp = (*(data_read_byte + 0) << 8) + *(data_read_byte + 1);

	if (data_tmp >> 15) *(data_float + 0) = - ((float)(~(data_tmp - 1) * MPU9255_ACCEL_SCALE_FACTOR * range));
	else *(data_float + 0) = ((float)data_tmp * MPU9255_ACCEL_SCALE_FACTOR * range);

	//преобразуем данные акселерометра по оси Y
	data_tmp = (*(data_read_byte + 2) << 8) + *(data_read_byte + 3);

	if (data_tmp >> 15) *(data_float + 1) = - ((float)(~(data_tmp - 1) * MPU9255_ACCEL_SCALE_FACTOR * range));
	else *(data_float + 1) = ((float)data_tmp * MPU9255_ACCEL_SCALE_FACTOR * range);

	//преобразуем данные акселерометра по оси Z
	data_tmp = (*(data_read_byte + 4) << 8) + *(data_read_byte + 5);

	if (data_tmp >> 15) *(data_float + 2) = - ((float)(~(data_tmp - 1) * MPU9255_ACCEL_SCALE_FACTOR * range));
	else *(data_float + 2) = ((float)data_tmp * MPU9255_ACCEL_SCALE_FACTOR * range);



	//GOTO_END_IF_ERROR(MPU9255_write_register(GYRO_AND_ACCEL, 27, 0b00011000));
	GOTO_END_IF_ERROR(MPU9255_read_register(GYRO_AND_ACCEL, 27, &g_range_reg, 1));
	for (size_t i = 0; i < ((g_range_reg & 0b00011000) >> 3); i++) {range = range * 2;}

	//преобразуем данные гироскопа по оси X
	data_tmp = (*(data_read_byte + 6) << 8) + *(data_read_byte + 7);

	if (data_tmp >> 15) *(data_float + 3) = - ((float)(~(data_tmp - 1) * MPU9255_GYRO_SCALE_FACTOR * range));
	else *(data_float + 3) = ((float)data_tmp * MPU9255_GYRO_SCALE_FACTOR * range);

	//преобразуем данные гироскопа по оси Y
	data_tmp = (*(data_read_byte + 8) << 8) + *(data_read_byte + 9);

	if (data_tmp >> 15) *(data_float + 4) = - ((float)(~(data_tmp - 1) * MPU9255_GYRO_SCALE_FACTOR * range));
	else *(data_float + 4) = ((float)data_tmp * MPU9255_GYRO_SCALE_FACTOR * range);

	//преобразуем данные гироскопа по оси Z
	data_tmp = (*(data_read_byte + 10) << 8) + *(data_read_byte + 11);

	if (data_tmp >> 15) *(data_float + 5) = - ((float)(~(data_tmp - 1) * MPU9255_GYRO_SCALE_FACTOR * range));
	else *(data_float + 5) = ((float)data_tmp * MPU9255_GYRO_SCALE_FACTOR * range);

	end:
	return error;
}*/

rscs_e MPU9255_recalc_compass(int16_t * raw_compass_XYZ, float * compass_XYZ)
{
	rscs_e error = 0;
	float x, y, z;
	int16_t *first_raw_compass_XYZ = (int16_t*)raw_compass_XYZ;
	float *first_compass_XYZ = (float*)compass_XYZ;


	float length = sqrt(pow(*(first_raw_compass_XYZ + 0), 2) + pow(*(first_raw_compass_XYZ + 1), 2) + pow(*(first_raw_compass_XYZ + 2), 2));

	x = ((float)*(first_raw_compass_XYZ + 1) / length);
	y = ((float)*(first_raw_compass_XYZ + 0)/ length);
	z = - ((float)*(first_raw_compass_XYZ + 2)/ length);

	*(first_compass_XYZ + 0) = x;
	*(first_compass_XYZ + 1) = y;
	*(first_compass_XYZ + 2) = z;

//	float * data_float = data;

//	//преобразуем данные компаса по оси X
//	data_tmp = (*(data_read_byte + 0) << 8) + *(data_read_byte + 1);
//
//	if (data_tmp >> 15) *(data_float + 0) = - ((float)((~(data_tmp - 1)) / 32.760));
//	else *(data_float + 0) = ((float)data_tmp / 32.760);
//
//	//преобразуем данные компаса по оси Y
//	data_tmp = (*(data_read_byte + 2) << 8) + *(data_read_byte + 3);
//
//	if (data_tmp >> 15) *(data_float + 1) = - ((float)((~(data_tmp - 1)) / 32.760));
//	else *(data_float + 1) = ((float)data_tmp / 32.760);
//
//	//преобразуем данные компаса по оси Z
//	data_tmp = (*(data_read_byte + 4) << 8) + *(data_read_byte + 5);
//
//	if (data_tmp >> 15) *(data_float + 2) = - ((float)((~(data_tmp - 1)) / 32.760));
//	else *(data_float + 2) = ((float)data_tmp / 32.760);

	return error;
}
