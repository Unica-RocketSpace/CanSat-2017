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

/*заполнение файла некоторыми данными; получает на вход путь файла, который надо заполнить*/
void generateStream(const char * path)
{
	FILE * file = fopen(path, "wb");

	uint16_t data[10][10];
	uint32_t start_frame	= INV_START_FRAME;
	uint32_t end_row		= INV_END_ROW;
	uint32_t end_frame		= INV_END_FRAME;

	fwrite(&start_frame, MARKER_SIZE, 1, file);

	for (int i = 0; i < 10; i++)
	{
		for (int j = 0; j < 10; j++)
		{
			data[i][j] = (i + 1) * (j + 1);
			fwrite(&data[i][j], 2, 1, file);
		}

		fwrite(&end_row, MARKER_SIZE, 1, file);
	}

	fwrite(&end_frame, MARKER_SIZE, 1, file);

	fclose(file);
}


void createFrame(const char * path_stream, char * path_frame, uint16_t frame_n)
{
	//TODO:Написать преобразование пути path_frame с добавлением номера кадра (например, path/frame-0001.ppm)

	FILE * stream = fopen(path_stream, "r");
	FILE * frame  = fopen(path_frame, "wb");


	fclose(stream);
	fclose(frame);
}


int main()
{
	const char path_stream[] = "/home/developer/git/CanSat-2017/src-ground_STM32/video.bin";
	const char path_frame[]  = "/home/developer/git/CanSat-2017/src-ground_STM32/frame.ppm";

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

	FILE * file_frame  = fopen(path_frame, "wb");		//файл со сформиррованным кадром

	for(int i = 0; i < (pointer / 2); i++)
	{
		if ((	(stream_uint[i + 0] << 16) |
				(stream_uint[i + 1] << 0 )	) == START_FRAME)
		{
			i = i + 2;
			int k = 0;
			uint8_t red, green, blue;
			fwrite(FRAME_HEADER, strlen(FRAME_HEADER), 1, file_frame);
			//fwrite(&FRAME_HEADER, 1, 11, file_frame);
			printf("Найдено начало кадра\n");
			for(; i < pointer; i++)
			{
				if ((	(stream_uint[i + 0] << 16) |
						(stream_uint[i + 1] << 0 )	) == END_ROW)
				{
					printf("Найден конец строки [записано %d пикс.]\n", k);
					i = i + 2;
				}
				if ((	(stream_uint[i + 0] << 16) |
						(stream_uint[i + 1] << 0 )	) == END_FRAME)
				{
					printf("Найден конец кадра [записано %d пикс.]\n", k);
					goto end;
				}

				red = (uint16_t)(stream_uint[i] >> 11);
				red = (uint8_t)(red * 255 / 31);

				green = (uint16_t)((uint16_t)(stream_uint[i] << 5) >> 10);
				green = (uint8_t)(green * 255 / 63);


				blue = (uint16_t)((uint16_t)(stream_uint[i] << 11) >> 11);
				blue = (uint8_t)(blue * 255 / 31);

				printf("[k] = %d\n", k);
				printf("red: %d, ", red);
				printf("green: %d, ", green);
				printf("blue: %d\n", blue);

				fwrite(&red, 1, 1, file_frame);
				fwrite(&green, 1, 1, file_frame);
				fwrite(&blue, 1, 1, file_frame);
				k++;
			}

		}

	}
end:
	fclose(file_stream);
	fclose(file_frame);
	free(stream_uint);

	return 0;
}
