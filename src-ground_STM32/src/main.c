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
#include <math.h>

#define START_FRAME		0xFF7C0000
#define END_ROW			0xFF7C00FF
#define END_FRAME		0x007CFFFF

#define INV_START_FRAME	0x0000FF7C
#define INV_END_ROW		0x00FFFF7C
#define INV_END_FRAME	0xFFFF007C

#define MARKER_SIZE		4

const char FRAME_HEADER[] = "P6 256 256 255 ";
const char SPACE[] = " ";

//ЗАПОЛНЯЕТ ФАЙЛ СГЕНЕРИРОВАННЫМИ ВИДЕОДАННЫМИ
//ПАРАМЕТР	* path		- указатель на путь к файлу, в который будут записаны видеоданные
void generateStream(const char * path)
{
	FILE * file = fopen(path, "wb");

	uint16_t data[256][256];
	uint32_t start_frame	= INV_START_FRAME;
	uint32_t end_row		= INV_END_ROW;
	uint32_t end_frame		= INV_END_FRAME;

	for(int k = 0; k < 150; k++)
	{
		fwrite(&start_frame, MARKER_SIZE, 1, file);
		for (int i = 0; i < 256; i++)
		{
			for (int j = 0; j < 256; j++)
			{
				data[i][j] = (i + 1 + k) * (j + 1 + k);
				fwrite(&data[i][j], 2, 1, file);
			}

			fwrite(&end_row, MARKER_SIZE, 1, file);
		}
		fwrite(&end_frame, MARKER_SIZE, 1, file);
	}


	fclose(file);
}

//ПРЕОБРАЗУЕТ НОМЕР КАДРА (int) В СТРОКУ С НОМЕРОМ КАДРА (char)
//ПАРАМЕТР	number		- номер кадра
//ПАРАМЕТР	* name		- указатель на строку[length], в которую будет записан номер кадра
//ПАРАМЕТР	lenght		- длина строки
void setFrameNumber(int number, char * name, int length)
{
	int figure;
	for (int i = 0; i < length; i++)
	{
		figure = (number % (int)(pow(10, i + 1))) / (int)(pow(10, i));
			name[(length - 1) - i] =  0x30 + figure;
	}
}

//СОЗДАЕТ КАДР ИЗ ПОТОКА ВИДЕОДАННЫХ
//ВОЗВРАЩАЕТ количество обработанных байт видеопотока или -1, если видеопоток закончился
//ПАРАМЕТР	* stream	- указатель на массив, содержащий данные видеопотока
//ПАРАМЕТР	length		- длина массива
//ПАРАМЕТР	pointer		- номер элемента массива, с которого начнется поиск кадра
//ПАРАМЕТР	* path_frame- указатель на путь к файлу, в который будет записан
int createFrame(uint16_t * stream, int length, int pointer, const char * path_frame)
{
	FILE * file_frame  = fopen(path_frame, "wb");		//файл со сформиррованным кадром
	int k = 0;

	for(int i = pointer; i < (length / 2); i++)
	{
		if ((	(stream[i + 0] << 16) |
				(stream[i + 1] << 0 )	) == START_FRAME)
		{
			i = i + 2;

			uint8_t red, green, blue;
			fwrite(FRAME_HEADER, strlen(FRAME_HEADER), 1, file_frame);
			printf("Найдено начало кадра\n");
			for(; i < length; i++)
			{
				if ((	(stream[i + 0] << 16) |
						(stream[i + 1] << 0 )	) == END_ROW)
				{
					i = i + 2;
				}
				if ((	(stream[i + 0] << 16) |
						(stream[i + 1] << 0 )	) == END_FRAME)
				{
					printf("Найден конец кадра [записано %d пикс.]\n", k);
					fclose(file_frame);
					return i + 2;
				}

				red = (uint16_t)(stream[i] >> 11);
				red = (uint8_t)(red * 255 / 31);
				green = (uint16_t)((uint16_t)(stream[i] << 5) >> 10);
				green = (uint8_t)(green * 255 / 63);
				blue = (uint16_t)((uint16_t)(stream[i] << 11) >> 11);
				blue = (uint8_t)(blue * 255 / 31);

				fwrite(&red, 1, 1, file_frame);
				fwrite(SPACE, 1, 1, file_frame);
				fwrite(&green, 1, 1, file_frame);
				fwrite(SPACE, 1, 1, file_frame);
				fwrite(&blue, 1, 1, file_frame);
				fwrite(SPACE, 1, 1, file_frame);
				k++;
			}

		}

	}
fclose(file_frame);
return -1;
}


//MAIN*****************
int main()
{
	const char	path_stream[]	= "/home/developer/git/CanSat-2017/src-ground_STM32/video.bin";
	char 		path_folder[]	= "/home/developer/Рабочий стол/Frames/frame-";
	const char	expansion[]		= ".ppm";

	char number[4];
	int n = 0;
	char path_frame [strlen(path_folder) + strlen(number) + strlen(expansion) + 1];

	//СОЗДАЕМ ФАЙЛ С ВИДЕОПОТОКОМ
	FILE * file_stream = fopen(path_stream, "r");		//файл с видеопотоком
	generateStream(path_stream);
	if (file_stream == NULL)
	{
		printf("ERROR");								//выводит ERROR если невозможно открыть файл
		return 0;
	}
	fseek(file_stream, 0, SEEK_END);					//переносим указатель в конец файла
	int pointer = ftell(file_stream);					//возвращаем функцией ftell длину файла в pointer
	printf("Размер потока видео: %d байт(а)\n", pointer);



	//выделяем в динамической памяти массив элементов uint16_t длины pointer и создаем указатель на него
	uint16_t * stream_uint = (uint16_t*)malloc(pointer);

	fseek(file_stream, 0, SEEK_SET);					//перемещаем указатель в начало файла
	fread(stream_uint, 2 * pointer, 1, file_stream);	//читаем из file_stream в созданный массив stream_uint один раз pointer бит

	int state = 0;
	while(1)
	{
		if (state == -1)
		{
			printf("Кадра № %d записан не полностью.\nКОНЕЦ ВИДЕОПОТОКА\n", n - 1);
			break;
		}
		setFrameNumber(n, number, 4);
		strcpy(path_frame, path_folder);
		strncat(path_frame, number, 4);
		strcat(path_frame, expansion);

		printf("\n===================================\n");
		printf("Путь к кадру № %d: ", n);
		printf(path_frame);
		printf("\n");

		state = createFrame(stream_uint, pointer, state, path_frame);
		n++;
	}


	fclose(file_stream);
	free(stream_uint);

	return 0;
}
