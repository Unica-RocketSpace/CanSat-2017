/*
 * MPU9255.c
 *
 *  Created on: 21 янв. 2017 г.
 *      Author: developer
 */

#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <util/delay.h>

#include <rscs/stdext/stdio.h>
#include <rscs/i2c.h>

#include "kinematic_unit.h"
#include "MPU9255.h"


rscs_e MPU9255_read_register(MPU9255_address adr, uint8_t reg_address, uint8_t *data_read, uint8_t n_read)
{
	rscs_e error;

	/*error = rscs_i2c_start();
	printf("i2c_start = %d\n", error);*/
	GOTO_END_IF_ERROR(rscs_i2c_start());

	/*error = rscs_i2c_send_slaw(adr, rscs_i2c_slaw_read);
	printf("i2c_slaw_read = %d\n", error);*/
	GOTO_END_IF_ERROR(rscs_i2c_send_slaw(adr, rscs_i2c_slaw_write));

	/*error = rscs_i2c_write(&reg_address, 1);
	printf("i2c_write_address = %d\n", error);*/
	GOTO_END_IF_ERROR(rscs_i2c_write(&reg_address, 1));

	/*error = rscs_i2c_start();
	printf("i2c_start = %d\n", error);*/
	GOTO_END_IF_ERROR(rscs_i2c_start());

	/*error = rscs_i2c_send_slaw(adr, rscs_i2c_slaw_read);
	printf("i2c_slaw_read = %d\n", error);*/
	GOTO_END_IF_ERROR(rscs_i2c_send_slaw(adr, rscs_i2c_slaw_read));

	/*error = rscs_i2c_read(data_read, n_read, 1);
	printf("i2c_read_error = %d\n", error);*/
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
	MPU9255_write_register(GYRO_AND_ACCEL,	28,		(0b00000000 | (ACCEL_RANGE << 3))); 	//accel config (rate 4g = 01)
	MPU9255_write_register(GYRO_AND_ACCEL,	29,		0b00000001);	//accel config 2 (Fch_b = 00, DLPF = 001)
	MPU9255_write_register(GYRO_AND_ACCEL,	35,		0b00000000);	//FIFO enable (not enabled)
	MPU9255_write_register(GYRO_AND_ACCEL,	56,		0b00000000);	//interrupt enable (int disable = 0)
	MPU9255_write_register(GYRO_AND_ACCEL,	106,	0b00000000);	//user control
	MPU9255_write_register(GYRO_AND_ACCEL,	107,	0b00000001);	//power managment 1
	MPU9255_write_register(GYRO_AND_ACCEL,	108,	0b00000000);	//power managment 2
	MPU9255_write_register(GYRO_AND_ACCEL,	27,		(0b00000000 | (GYRO_RANGE << 4)) );	//gyro config (rate 500dps = 01, Fch_b = 00)


	MPU9255_write_register(GYRO_AND_ACCEL,	55,		0b00000010);	//режим bypass on
	MPU9255_write_register(COMPASS,		  	0x0A,	0b00010110);	//control 1
	MPU9255_write_register(GYRO_AND_ACCEL,	55,		0b00000000);	//режим bypass off
}


static int16_t _swapBytesI16(int16_t value)
{
	uint8_t * value_ptr = (uint8_t*)&value;
	uint8_t tmp = value_ptr[0];
	value_ptr[0] = value_ptr[1];
	value_ptr[1] = tmp;

	return value;
}


rscs_e MPU9255_read_imu(int16_t * raw_accel_XYZ, int16_t * raw_gyro_XYZ)
{
	rscs_e error = 0;

	GOTO_END_IF_ERROR(MPU9255_read_register(GYRO_AND_ACCEL, 59, (uint8_t*)raw_accel_XYZ, 6));	//чтение данных с акселерометра
	GOTO_END_IF_ERROR(MPU9255_read_register(GYRO_AND_ACCEL, 67, (uint8_t*)raw_gyro_XYZ, 6));	//чтение данных с гироскопа


	for (int i = 0; i < 3; i++)
		raw_accel_XYZ[i] = _swapBytesI16(raw_accel_XYZ[i]);

	for (int i = 0; i < 3; i++)
		raw_gyro_XYZ[i] = _swapBytesI16(raw_gyro_XYZ[i]);

end:
	return error;
}


rscs_e MPU9255_read_compass(int16_t * raw_compass_XYZ)
{
	rscs_e error = 0;

	uint8_t dummy;

	GOTO_END_IF_ERROR(MPU9255_write_register(GYRO_AND_ACCEL, 55, 0b00000010));	//режим bypass on
	MPU9255_read_register(COMPASS, 0x02, &dummy, 1);
	//printf("read_dummy_error = %d\n", read_dummy_error);
	GOTO_END_IF_ERROR(MPU9255_read_register(COMPASS, 0x02, &dummy, 1));


	if ((dummy && 0x01) != 1)
	{
		STATE.state &= ~(1 << 1);		//магнитометр не готов
		GOTO_END_IF_ERROR(MPU9255_write_register(GYRO_AND_ACCEL, 55, 0b00000000));	//режим bypass off
		error = -7;
		goto end;
	}

	STATE.state |= (1 << 1);	////магнитометр готов
	GOTO_END_IF_ERROR(MPU9255_read_register(COMPASS, 0x03, (uint8_t*)raw_compass_XYZ, 6));
	GOTO_END_IF_ERROR(MPU9255_read_register(COMPASS, 0x09, &dummy, 1));
	GOTO_END_IF_ERROR(MPU9255_write_register(GYRO_AND_ACCEL, 55, 0b00000000));	//режим bypass off

end:
	return error;
}


void MPU9255_recalc_accel(const int16_t * raw_accel_XYZ, float * accel_XYZ)
{
	for (int i = 0; i < 3; i++)
		accel_XYZ[i] = (float)(raw_accel_XYZ[i]) * MPU9255_ACCEL_SCALE_FACTOR * pow(2, ACCEL_RANGE);
}


void MPU9255_recalc_gyro(const int16_t * raw_gyro_XYZ, float * gyro_XYZ)
{
	for (int i = 0; i < 3; i++)
		gyro_XYZ[i] = (float)(raw_gyro_XYZ[i]) * MPU9255_GYRO_SCALE_FACTOR * pow(2, GYRO_RANGE);
}

void MPU9255_recalc_compass(const int16_t * raw_compass_XYZ, float * compass_XYZ)
{
	float x, y, z;

	float length = sqrt(pow(*(raw_compass_XYZ + 0), 2) + pow(*(raw_compass_XYZ + 1), 2) + pow(*(raw_compass_XYZ + 2), 2));

	x = (float)raw_compass_XYZ[1] / length;
	y = (float)raw_compass_XYZ[0] / length;
	z = - (float)raw_compass_XYZ[2] / length;

	compass_XYZ[0] = x;
	compass_XYZ[1] = y;
	compass_XYZ[2] = z;
}
