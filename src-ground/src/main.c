/*
 * main.c
 *
 *  Created on: 31 марта 2017 г.
 *      Author: developer
 */

#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <sofa.h>

#include "package_struct.h"
#include "recalculation.h"
#include "kinematic_unit.h"
#include "analysis.h"


package PACKAGE;
package * PACKAGE_ADDR = &PACKAGE;


/*проверка целостности пакета (сравнением контрольной суммы)*/
bool check_package(package * pack)
{
	uint16_t CS = 0;
	uint32_t tCS = 0;
	uint8_t * first_pack = (uint8_t*)pack;

	for (int i = 0; i < sizeof(package) - sizeof(pack->CS); i++)
	{
		tCS = CS + *(first_pack + i);
		CS = (uint16_t)(tCS & 0x0000FFFF);
	}

	if (CS == pack->CS)
		return true;
	else
		return false;

}

/*заполнение файла некоторыми данными; получает на вход путь файла, который надо заполнить*/
void make_fake_data(const char * filepath)
{
	FILE * file = fopen(filepath, "wb");

	for (size_t i = 0; i < 10; i++)
	{
		package packet;
		packet.marker = 0xFF;
		packet.number = i;
		packet.pressure = 0x00;
		//packet.temp = 0x00;
		packet.aXYZ[0] = 0x0000;
		packet.aXYZ[1] = 0x0000;
		packet.aXYZ[2] = 0x0000;

		packet.gXYZ[0] = 0x0000;
		packet.gXYZ[1] = 0x0000;
		packet.gXYZ[2] = 0x0000;

		packet.cXYZ[0] = 0x0000;
		packet.cXYZ[1] = 0x0000;
		packet.cXYZ[2] = 0x0000;

		packet.time = 0x0000;
		packet.state = 0x0000;

		uint16_t cs = 0;
		for (int i = 0; i < sizeof(package) - sizeof(packet.CS); i++)
		{
			const uint8_t * packet_first_byte = (const uint8_t*)&packet;
			uint32_t tCS = cs + *(packet_first_byte + i);
			cs = (uint16_t)(tCS & 0x0000FFFF);
		}

		packet.CS = cs;

		/*size_t writen = */fwrite(&packet, sizeof(packet), 1, file);
	}

	fclose(file);
}

void printf_state(FILE * file_, long package_number)
{
	fprintf(file_, "%ld, ", package_number);
	fprintf(file_, "%f, ", (float)DEVICE_STATE.Time / 1000);
	fprintf(file_, "%f, %f, %f, ", DEVICE_STATE.aRelatedXYZ[0], DEVICE_STATE.aRelatedXYZ[1], DEVICE_STATE.aRelatedXYZ[2]);
	fprintf(file_, "%f, ", iauPm(DEVICE_STATE.aRelatedXYZ));
	fprintf(file_, "%f, %f, %f, ", DEVICE_STATE.gRelatedXYZ[0], DEVICE_STATE.gRelatedXYZ[1], DEVICE_STATE.gRelatedXYZ[2]);
	fprintf(file_, "%f, ", iauPm(DEVICE_STATE.gRelatedXYZ));
	fprintf(file_, "%f, %f, %f, ", DEVICE_STATE.a_XYZ[0], DEVICE_STATE.a_XYZ[1], DEVICE_STATE.a_XYZ[2]);
	fprintf(file_, "%f, ", iauPm(DEVICE_STATE.a_XYZ));
	fprintf(file_, "%f, %f, %f, ", DEVICE_STATE.v_XYZ[0], DEVICE_STATE.v_XYZ[1], DEVICE_STATE.v_XYZ[2]);
	fprintf(file_, "%f, ", iauPm(DEVICE_STATE.v_XYZ));
	fprintf(file_, "%f, %f, %f, ", DEVICE_STATE.s_XYZ[0], DEVICE_STATE.s_XYZ[1], DEVICE_STATE.s_XYZ[2]);
	fprintf(file_, "%f, ", iauPm(DEVICE_STATE.s_XYZ));
	fprintf(file_, "%f, %f, %f, ", DEVICE_STATE.w_XYZ[0], DEVICE_STATE.w_XYZ[1], DEVICE_STATE.w_XYZ[2]);
	fprintf(file_, "%f, ", iauPm(DEVICE_STATE.w_XYZ));
	fprintf(file_, "%f, %f, %f, %f, %f, %f, %f, %f, %f\n",	DEVICE_STATE.f_XYZ[0][0],
													DEVICE_STATE.f_XYZ[0][1],
													DEVICE_STATE.f_XYZ[0][2],
													DEVICE_STATE.f_XYZ[1][0],
													DEVICE_STATE.f_XYZ[1][1],
													DEVICE_STATE.f_XYZ[1][2],
													DEVICE_STATE.f_XYZ[2][0],
													DEVICE_STATE.f_XYZ[2][1],
													DEVICE_STATE.f_XYZ[2][2]);
}
void printf_first_string_state(FILE * file_)
{
	fprintf(file_, "package, ");
	fprintf(file_, "time, ");
	fprintf(file_, "Accelerometer-X, ");
	fprintf(file_, "Accelerometer-Y, ");
	fprintf(file_, "Accelerometer-Z, ");
	fprintf(file_, "Accelerometer, ");
	fprintf(file_, "Gyroscope-X, ");
	fprintf(file_, "Gyroscope-Y, ");
	fprintf(file_, "Gyroscope-Z, ");
	fprintf(file_, "Gyroscope, ");
	fprintf(file_, "Accelerations-X, ");
	fprintf(file_, "Accelerations-Y, ");
	fprintf(file_, "Accelerations-Z, ");
	fprintf(file_, "Accelerations, ");
	fprintf(file_, "Velocities-X, ");
	fprintf(file_, "Velocities-Y, ");
	fprintf(file_, "Velocities-Z, ");
	fprintf(file_, "Velocities, ");
	fprintf(file_, "Translations-X, ");
	fprintf(file_, "Translations-Y, ");
	fprintf(file_, "Translations-Z, ");
	fprintf(file_, "Translations, ");
	fprintf(file_, "Angle velocities-X, ");
	fprintf(file_, "Angle velocities-Y, ");
	fprintf(file_, "Angle velocities-Z, ");
	fprintf(file_, "Angle velocities, ");
	fprintf(file_, "Rotation Matrix\n");
}

float G_vector;


int main()
{
	const char fake_package_path[] = "/home/developer/git/CanSat-2017/src-ground/RAW.bin";
	//make_fake_data(fake_package_path);

	FILE * f_raw = fopen(fake_package_path, "rb");		//файл с сырыми значениями
	FILE * f_ready_txt = fopen("/home/developer/git/CanSat-2017/src-ground/YouCanReadThis_DATA.txt","w");		//файл с преобразованными значениями
	FILE * f_ready_csv = fopen("/home/developer/git/CanSat-2017/src-ground/ready_to_plot.csv","w");

	if (f_raw == NULL)
	{
		printf("ERROR");								//выводит ERROR если невозможно открыть файл
		return 0;
	}

	fseek(f_raw, 0, SEEK_END);							//переносим указатель в конец файла
	int pointer = ftell(f_raw);							//возвращаем функцией ftell длину файла в pointer

	printf("file size: %d\n", pointer);

	/*выделяем в динамической памяти массив элементов uint8_t длины pointer и создаем указатель на него	*/
	uint8_t * pack_uint = (uint8_t*)malloc(pointer);


	fseek(f_raw, 0, SEEK_SET);							//перемещаем указатель в начало файла

	/*читаем из f_raw в созданный массив pack_uint один раз pointer бит	*/
	fread(pack_uint, pointer, 1, f_raw);


	//Инициализация структуры STATE
	kinematicInit();
	printf_first_string_state(f_ready_csv);

	int i;
	int counter = 0;
	for (i = 0; i < pointer; i++)
	{
		if (pack_uint[i] == 0xFA && pack_uint[i + 1] == 0xFA)	//если находит маркер структуры (0xFF), начинает анализ пакета
		{
			package * pack = (package*)(pack_uint + i);
			if (check_package(pack))
			{
				recalc_accel((int16_t*)(pack->aXYZ), DEVICE_STATE.aRelatedXYZ);
				recalc_gyro((int16_t*)(pack->gXYZ), DEVICE_STATE.gRelatedXYZ);
				recalc_compass((int16_t*)(pack->cXYZ), DEVICE_STATE.cRelatedXYZ);

				//Фильтрация шума
				apply_NoiseFilter(DEVICE_STATE.gRelatedXYZ, GYRO_NOISE, 3);
				apply_NoiseFilter(DEVICE_STATE.aRelatedXYZ, ACCEL_NOISE, 3);

				//определение угловых скоростей (в ИСК)
				RSC_to_ISC_recalc(DEVICE_STATE.gRelatedXYZ, DEVICE_STATE.w_XYZ);

				solveByRungeKutta((float)(DEVICE_STATE.Time - DEVICE_STATE.previousTime) / 1000, DEVICE_STATE.f_XYZ_prev[0], DEVICE_STATE.f_XYZ[0]);
				solveByRungeKutta((float)(DEVICE_STATE.Time - DEVICE_STATE.previousTime) / 1000, DEVICE_STATE.f_XYZ_prev[1], DEVICE_STATE.f_XYZ[1]);

				//находим третью строку матрицы поворота из условия ортогональности векторов
				iauPxp(DEVICE_STATE.f_XYZ[0], DEVICE_STATE.f_XYZ[1], DEVICE_STATE.f_XYZ[2]);

				set_cos_to_1(&DEVICE_STATE.f_XYZ[0][0]);
				set_cos_to_1(&DEVICE_STATE.f_XYZ[0][1]);
				set_cos_to_1(&DEVICE_STATE.f_XYZ[0][2]);
				set_cos_to_1(&DEVICE_STATE.f_XYZ[1][0]);
				set_cos_to_1(&DEVICE_STATE.f_XYZ[1][1]);
				set_cos_to_1(&DEVICE_STATE.f_XYZ[1][2]);
				set_cos_to_1(&DEVICE_STATE.f_XYZ[2][0]);
				set_cos_to_1(&DEVICE_STATE.f_XYZ[2][1]);
				set_cos_to_1(&DEVICE_STATE.f_XYZ[2][2]);

				//определение ускорений
				RSC_to_ISC_recalc(DEVICE_STATE.aRelatedXYZ, DEVICE_STATE.a_XYZ);

				G_vector += iauPm(DEVICE_STATE.a_XYZ);
				counter++;
			}
		}
	}
	G_vector = G_vector / counter;
	printf("Counter = %d\n", counter);
	printf("G_ = %f\n", G_vector);
	bool matrix_set = 0;
	for (i = 0; i < pointer; i++)
	{
		if ((pack_uint[i] == 0xFE && pack_uint[i + 1] == 0xFE) && (matrix_set == 0))
		{
			first_dev_matrix * device_first_matrix = (first_dev_matrix*)(pack_uint + i + 2);
			for (int i = 0; i < 3; i++)
				for (int j = 0; j < 3; j++)
				{
					DEVICE_STATE.f_XYZ[i][j] = device_first_matrix->f_XYZ[i][j];
					DEVICE_STATE.f_XYZ_prev[i][j] = device_first_matrix->f_XYZ[i][j];
				}
		}
		if (pack_uint[i] == 0xFA && pack_uint[i + 1] == 0xFA)	//если находит маркер структуры (0xFF), начинает анализ пакета
		{
			matrix_set = 1;
			package * pack = (package*)(pack_uint + i);
			if (check_package(pack))
			{
				/*global_data DATA =
				{
						.pressure = recalc_bmp280Pressure(pack->pressure),
						.temp_bmp280 = recalc_bmp280Temp(pack->temp_bmp280),
						.temp_ds18b20 = recalc_ds18b20Temp(pack->temp_ds18b20),
						.time = (float)pack->time / 1000
				};
				recalc_adxl345(pack->a_adxl345, DATA.a_adxl345);
				recalc_accel((int16_t*)(pack->aXYZ), DATA.accel_XYZ);
				recalc_gyro((int16_t*)(pack->gXYZ), DATA.gyro_XYZ);
				recalc_compass((int16_t*)(pack->cXYZ), DATA.compass_XYZ);
				*/

				printf("Пакет %d записан \n", pack->number);
				/*---------------------------------------------------------------------*/
				DEVICE_STATE.pressure = recalc_bmp280Pressure(pack->pressure);
				DEVICE_STATE.temp_bmp280 = recalc_bmp280Temp(pack->temp_bmp280);
				DEVICE_STATE.temp_ds18b20 = recalc_ds18b20Temp(pack->temp_ds18b20);
				DEVICE_STATE.Time = pack->time;
				recalc_adxl345(pack->a_adxl345, DEVICE_STATE.aALT_XYZ);
				recalc_accel((int16_t*)(pack->aXYZ), DEVICE_STATE.aRelatedXYZ);
				recalc_gyro((int16_t*)(pack->gXYZ), DEVICE_STATE.gRelatedXYZ);
				recalc_compass((int16_t*)(pack->cXYZ), DEVICE_STATE.cRelatedXYZ);

				//Фильтрация шума
				apply_NoiseFilter(DEVICE_STATE.gRelatedXYZ, GYRO_NOISE, 3);
				apply_NoiseFilter(DEVICE_STATE.aRelatedXYZ, ACCEL_NOISE, 3);

				construct_trajectory(G_vector);
				printf_state(f_ready_csv, pack->number);
				/*---------------------------------------------------------------------*/

				//content_FPrint(stdout, pack, &DATA, ALL);
				//content_FPrint(f_ready_txt, pack, &DEVICE_STATE, ALL);
				//content_FPrint(f_ready_csv, pack, &DATA, ALL | CSV);

			}
			else
				printf("Пакет не прошел проверку \n");
		}
	}


	fclose(f_raw);
	fclose(f_ready_txt);
	fclose(f_ready_csv);
	free(pack_uint);
	return 0;
}

