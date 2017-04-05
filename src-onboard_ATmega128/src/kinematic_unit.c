/*
 * kinematic_unit.c
 *
 *  Created on: 23 февр. 2017 г.
 *      Author: RaKetov
 */

#include <math.h>
#include <rscs/timeservice.h>
#include <rscs/ds18b20.h>				//ДРАЙВЕР ДАТЧИКА ТЕМПЕРАТУРЫ DS18B20
//#include <rscs/bmp180.h>				//ДРАЙВЕР ДАТЧИКА ДАВЛЕНИЯ BMP180

//#include "ADXL345.h"					//ДРАЙВЕР АКСЕЛЕРОМЕТРА	ADXL345
#include "MPU9255.h"					//ДРАЙВЕР ГИРОСКОПА, АКСЕЛЕРОМЕТРА и КОМПАСА (MPU9255)
#include "kinematic_unit.h"


void kinematicInit()
{
	state STATE = {	{0},	//ускорения в единицах g (в ССК)
					{0},	//угловые скорости в degps (в ССК)
					{0},	//косинусы углов вектора магнитного поля с осями ССК
					0,			//давление

					0, 0, 0,	//ускорения в м/с^2 (ИСК)
					0, 0, 0,	//скорости в м/с (ИСК)
					0, 0, 0,	//перемещения в м (ИСК)
					0, 0, 0,	//угловые скорости в 1/с (ИСК)

					0, 0, 0,	//косинусы углов оси Х ИСК с осями ССК
					0, 0, 0,	//косинусы углов оси Y ИСК с осями ССК
					0, 0, 0,	//косинусы углов оси Z ИСК с осями ССК
					0
					};

	transmit_data TRANSMIT_DATA = {	{0},
									{0},
									{0}
	};


}


void set_ISC_offset()
{
	rscs_e error1, error2;
	int16_t dummy = 0;
	int16_t accel_raw_XYZ[3];
	float accel_XYZ[3];

	error1 = MPU9255_read_imu(accel_raw_XYZ, &dummy);
	error2 = MPU9255_recalc_accel(accel_raw_XYZ, accel_XYZ);

	while ((error1 & error2) != 0)
	{
		float *first_accel_XYZ = (float*)accel_XYZ;

		float g_vect = sqrt(pow(*(first_accel_XYZ + 0), 2) + pow(*(first_accel_XYZ + 1), 2) + pow(*(first_accel_XYZ + 2), 2));

		ISC_OFFSET.fOFFSET_X = - ((float)*(first_accel_XYZ + 0) / g_vect);
		ISC_OFFSET.fOFFSET_Y = - ((float)*(first_accel_XYZ + 1) / g_vect);
		ISC_OFFSET.fOFFSET_Z = - ((float)*(first_accel_XYZ + 2) / g_vect);

		error1 = MPU9255_read_imu(accel_raw_XYZ, &dummy);
		error2 = MPU9255_recalc_accel(accel_raw_XYZ, accel_XYZ);
	}
}

void ISC_recalc()
{
	STATE.fZX1 = ISC_OFFSET.fOFFSET_X;
	STATE.fZY1 = ISC_OFFSET.fOFFSET_Y;
	STATE.fZZ1 = ISC_OFFSET.fOFFSET_Z;




}

void set_magn_dir()
{
	rscs_e error1, error2;
	int16_t compass_raw_XYZ[3];
	float compass_XYZ[3];

	error1 = MPU9255_read_compass(compass_raw_XYZ);
	error2 = MPU9255_recalc_compass(compass_raw_XYZ, compass_XYZ);

	while ((error1 & error2) != 0)
	{
		float * first_compass_XYZ = (float*)compass_XYZ;

		//нахождение длины вектора магнитного поля (вектор B)
		float B_vect = sqrt(pow(*(first_compass_XYZ + 0), 2) + pow(*(first_compass_XYZ + 1), 2) + pow(*(first_compass_XYZ + 2), 2));

		MAGN_DIR.fBX = ((float)*(first_compass_XYZ + 0) / B_vect);
		MAGN_DIR.fBY = ((float)*(first_compass_XYZ + 1) / B_vect);
		MAGN_DIR.fBZ = ((float)*(first_compass_XYZ + 2) / B_vect);

		error1 = MPU9255_read_compass(compass_raw_XYZ);
		error2 = MPU9255_recalc_compass(compass_raw_XYZ, compass_XYZ);
	}
}

void recon_AGC_STATE_TRANSMIT_DATA()
{

	MPU9255_read_imu(TRANSMIT_DATA.aTransmitXYZ, TRANSMIT_DATA.gTransmitXYZ);
	MPU9255_read_compass(TRANSMIT_DATA.cTransmitXYZ);

	MPU9255_recalc_accel(TRANSMIT_DATA.aTransmitXYZ, STATE.aRelatedXYZ);
	MPU9255_recalc_gyro(TRANSMIT_DATA.gTransmitXYZ, STATE.gRelatedXYZ);
	MPU9255_recalc_compass(TRANSMIT_DATA.cTransmitXYZ, STATE.cRelatedXYZ);

	//TODO: написать опрос барометра

	STATE.Time = rscs_time_get();

}

void calculate_height()
{
	STATE.sZ = 44330 * (1 - pow((STATE.pressure / ZERO_PRESSURE), 1/5.225));
}


void trajectoryConstruction()
{

	float dt = (rscs_time_get() - STATE.Time) / 1000;

	//определение угловых скоростей (в ИСК)
	STATE.wX = STATE.gRelatedXYZ[0] * STATE.fXX1 + STATE.gRelatedXYZ[1] * STATE.fXY1 + STATE.gRelatedXYZ[2] * STATE.fXZ1;
	STATE.wY = STATE.gRelatedXYZ[0] * STATE.fYX1 + STATE.gRelatedXYZ[1] * STATE.fYY1 + STATE.gRelatedXYZ[2] * STATE.fYZ1;
	STATE.wZ = STATE.gRelatedXYZ[0] * STATE.fZX1 + STATE.gRelatedXYZ[1] * STATE.fZY1 + STATE.gRelatedXYZ[2] * STATE.fZZ1;


	//определение углов между осями ИСК и ССК
	rotation_matrix ROT_M = { 0 };

	ROT_M.fXX1 = STATE.fXX1 + (STATE.fXY1 * STATE.wZ - STATE.fXZ1 * STATE.wY) * dt;
	ROT_M.fXY1 = STATE.fXY1 + (STATE.fXZ1 * STATE.wX - STATE.fXX1 * STATE.wZ) * dt;
	ROT_M.fXZ1 = STATE.fXZ1 + (STATE.fXX1 * STATE.wY - STATE.fXY1 * STATE.wX) * dt;
	ROT_M.fYX1 = STATE.fYX1 + (STATE.fYY1 * STATE.wZ - STATE.fYZ1 * STATE.wY) * dt;
	ROT_M.fYY1 = STATE.fYY1 + (STATE.fYZ1 * STATE.wX - STATE.fYX1 * STATE.wZ) * dt;
	ROT_M.fYZ1 = STATE.fYZ1 + (STATE.fYX1 * STATE.wY - STATE.fYY1 * STATE.wX) * dt;

	ROT_M.fZZ1 = ROT_M.fXX1 * ROT_M.fYY1 - ROT_M.fXY1 * ROT_M.fYX1;
	ROT_M.fZX1 = (ROT_M.fXX1 * ROT_M.fXZ1 + ROT_M.fYX1 * ROT_M.fYZ1) / ROT_M.fZZ1;
	ROT_M.fZY1 = (ROT_M.fXY1 * ROT_M.fXZ1 + ROT_M.fYY1 * ROT_M.fYZ1) / ROT_M.fZZ1;


	//обновление функций углов в STATE
	STATE.fXX1 = ROT_M.fXX1;
	STATE.fXY1 = ROT_M.fXY1;
	STATE.fXZ1 = ROT_M.fXZ1;
	STATE.fYX1 = ROT_M.fYX1;
	STATE.fYY1 = ROT_M.fYY1;
	STATE.fYZ1 = ROT_M.fYZ1;
	STATE.fZX1 = ROT_M.fZX1;
	STATE.fZY1 = ROT_M.fZY1;
	STATE.fZZ1 = ROT_M.fZZ1;


	//определение ускорений
	STATE.aX = STATE.aRelatedXYZ[0] * STATE.fXX1 + STATE.aRelatedXYZ[1] * STATE.fXY1 + STATE.aRelatedXYZ[2] * STATE.fXZ1;
	STATE.aY = STATE.aRelatedXYZ[0] * STATE.fYX1 + STATE.aRelatedXYZ[1] * STATE.fYY1 + STATE.aRelatedXYZ[2] * STATE.fYZ1;
	STATE.aZ = STATE.aRelatedXYZ[0] * STATE.fZX1 + STATE.aRelatedXYZ[1] * STATE.fZY1 + STATE.aRelatedXYZ[2] * STATE.fZZ1;


	//определение скоростей
	STATE.vX = STATE.vX + STATE.aX * dt;
	STATE.vY = STATE.vY + STATE.aY * dt;
	STATE.vZ = STATE.vZ + STATE.aZ * dt;


	//расчет перемещений
	STATE.sX = STATE.sX + STATE.vX * dt + STATE.aX * dt * dt / 2;
	STATE.sY = STATE.sY + STATE.vY * dt + STATE.aY * dt * dt / 2;
	calculate_height();	//записывает в STATE.sZ значение высоты

	//TODO: STATE.pressure = BMP_recalc

}


void getTranslations (float * translations)
{
	float * first_translations = (float*)translations;

	*(first_translations + 0) = STATE.sX;
	*(first_translations + 1) = STATE.sY;
	*(first_translations + 2) = STATE.sZ;
}


void getAngVelocity (float * angVelocity)
{
	float * first_angVelocity = (float*)angVelocity;

	*(first_angVelocity + 0) = STATE.wX;
	*(first_angVelocity + 1) = STATE.wY;
	*(first_angVelocity + 2) = STATE.wZ;

}


void getRotationMatrix (float * RotationMatrix)
{
	float * first_RotationMatrix = (float*)RotationMatrix;

	*(first_RotationMatrix + 0) = STATE.fXX1;
	*(first_RotationMatrix + 1) = STATE.fXY1;
	*(first_RotationMatrix + 2) = STATE.fXZ1;
	*(first_RotationMatrix + 3) = STATE.fYX1;
	*(first_RotationMatrix + 4) = STATE.fYY1;
	*(first_RotationMatrix + 5) = STATE.fYZ1;
	*(first_RotationMatrix + 6) = STATE.fZX1;
	*(first_RotationMatrix + 7) = STATE.fZY1;
	*(first_RotationMatrix + 8) = STATE.fZZ1;

}



