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

#include "package_struct.h"
#include "recalculation.h"
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


int main()
{
	const char fake_package_path[] = "/home/developer/git/CanSat-2017/src-ground/RAW2.log";
	make_fake_data(fake_package_path);

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


	int i;
	for (i = 0; i < pointer; i++)
	{
		if (pack_uint[i] == 0xFF)						//если находит маркер структуры (0xFF), начинает анализ пакета
		{
			package * pack = (package*)(pack_uint + i);
			if (check_package(pack))
			{
				global_data DATA =
				{
						.pressure = recalc_bmp280Pressure(pack->pressure, pack->temp_bmp280),
						.temp_bmp280 = recalc_bmp280Temp(pack->temp_bmp280),
						.temp_ds18b20 = recalc_ds18b20Temp(pack->temp_ds18b20),
						.time = pack->time / 1000
				};
				recalc_accel((int16_t*)(pack->aXYZ), DATA.accel_XYZ);
				recalc_gyro((int16_t*)(pack->gXYZ), DATA.gyro_XYZ);
				recalc_compass((int16_t*)(pack->cXYZ), DATA.compass_XYZ);

				content_FPrint(f_ready_txt, pack, &DATA, ALL);
				content_FPrint(f_ready_csv, pack, &DATA, ALL | CSV);

			}
			else
				printf("Пакет %d не прошел проверку \n", i);
		}


	}


	fclose(f_raw);
	fclose(f_ready_txt);
	fclose(f_ready_csv);
	free(pack_uint);
	return 0;
}

/*	float accel_XYZ[3] = {0}; float gyro_XYZ[3] = {0}; float compass_XYZ[3] = {0};
	printf("============================\n");
	printf("ПАКЕТ %d \n\n", pack->number);
	fprintf(f_ready, "============================\n");
	fprintf(f_ready, "ПАКЕТ %d\n\n", pack->number);

	//TODO: добавить функцию пересчета давления
	//TODO: добавить функцию пересчета температуры

	/ *запись ускорений* /
	recalc_accel((int16_t*)(pack->aXYZ), accel_XYZ);
			//вывод
	printf("Ускорение по оси X: %f м/с^2\n", accel_XYZ[0]);
	printf("Ускорение по оси Y: %f м/с^2\n", accel_XYZ[1]);
	printf("Ускорение по оси Z: %f м/с^2\n", accel_XYZ[2]);
	printf("\n");
			//запись в файл
	fprintf(f_ready, "Показания акселерометра (XYZ):\n");
	fprintf(f_ready, "Ускорение по оси X: %f м/с^2\n", accel_XYZ[0]);
	fprintf(f_ready, "Ускорение по оси Y: %f м/с^2\n", accel_XYZ[1]);
	fprintf(f_ready, "Ускорение по оси Z: %f м/с^2\n", accel_XYZ[2]);
	//fwrite(accel_XYZ, 3, 1, f_ready);
	fprintf(f_ready, "\n");
	/*_____________()_______________* /


	/ *запись угловых скоростей* /
	recalc_gyro((int16_t*)(pack->gXYZ), gyro_XYZ);
			//вывод
	printf("Угловая скорость по оси X: %f рад/с\n", gyro_XYZ[0]);
	printf("Угловая скорость по оси Y: %f рад/с\n", gyro_XYZ[1]);
	printf("Угловая скорость по оси Z: %f рад/с\n", gyro_XYZ[2]);
	printf("\n");
			//запись в файл
	fprintf(f_ready, "Показания гироскопа (XYZ):\n");
	fwrite(gyro_XYZ, 1, 3, f_ready);
	fprintf(f_ready, "\n");
	/*_____________()_______________* /


	/ *запись направляющих косинусов вектора манитного поля* /
	recalc_compass((int16_t*)(pack->cXYZ), compass_XYZ);
			//вывод
	printf("cos угла (B,OX): %f\n", compass_XYZ[0]);
	printf("cos угла (B,OY): %f\n", compass_XYZ[1]);
	printf("cos угла (B,OZ): %f\n", compass_XYZ[2]);
	printf("\n");
			//запись в файл
	fprintf(f_ready, "cos углов (B,XYZ):\n");
	fwrite(compass_XYZ, 3, 1, f_ready);
	compass_XYZ[0] = 0;
	compass_XYZ[1] = 0;
	compass_XYZ[2] = 0;
	/ *_____________()_______________* /

	/ *запись состояния* /
	printf("Состояние: %d\n", pack->state);
	fprintf(f_ready, "Состояние: %d\n", pack->state);

	/ *запись времени* /
	float time = pack->time / 1000;
	printf("Время передачи: %f c\n", time);
	fprintf(f_ready, "Время передачи: %f c\n", time);
	time = 0;


	printf("============================\n");
	fprintf(f_ready, "============================\n");*/
