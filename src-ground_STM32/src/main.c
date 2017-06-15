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

#define INV_START_FRAME		0xFF7C0000
#define INV_END_ROW			0xFF7C00FF
#define INV_END_FRAME			0x007CFFFF

#define START_FRAME			0x0000FF7C
#define END_ROW				0x00FFFF7C
#define END_FRAME				0xFFFF007C

#define MARKER_SIZE		4

const char FRAME_HEADER[] = "P6 640 300 255 ";
char mode[10];
const char color_mode[] = "color";
const char mono_mode[] = "mono";
int video_number;

//ЗАПОЛНЯЕТ ФАЙЛ СГЕНЕРИРОВАННЫМИ ВИДЕОДАННЫМИ
//ПАРАМЕТР	* path		- указатель на путь к файлу, в который будут записаны видеоданные
void generateStream(const char * path)
{
	FILE * file = fopen(path, "wb");

	uint16_t frame_width = 320;
	uint16_t frame_height = 240;
	uint16_t data[frame_width][frame_height];
	uint32_t start_frame	= INV_START_FRAME;
	uint32_t end_row		= INV_END_ROW;
	uint32_t end_frame		= INV_END_FRAME;

	int I = 0;
	for(int k = 0; k < 150; k++)
	{
		fwrite(&start_frame, MARKER_SIZE, 1, file);
		for (int i = 0; i < frame_width; i++)
		{
			for (int j = 0; j < frame_height; j++)
			{
				data[i][j] = 0b0000011111100000 + k;
				fwrite(&data[i][j], 2, 1, file);
				I++;
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

			int16_t Y = 0, Cb = 0, Cr = 0;
			int16_t red, green, blue;
			fwrite(FRAME_HEADER, strlen(FRAME_HEADER), 1, file_frame);
			printf("Найдено начало кадра [пиксель номер %d]\n", i - 2);
			for(; i < length; i++)
			{
				/*if ((	(stream[i + 0] << 16) |
						(stream[i + 1] << 0 )	) == END_ROW)
				{
					i = i + 2;
				}*/
				if ((	(stream[i + 0] << 16) |
						(stream[i + 1] << 0 )	) == END_FRAME)
				{
					printf("Найден конец кадра [записано %d пикс.]\n", k);
					fclose(file_frame);
					return i + 2;
				}


				if (!strcmp(mode, color_mode))
				{
					Y = (uint8_t)stream[i];
					if (i % 2 == 0)
					{
						Cb = (uint8_t)(stream[i] >> 8);
						Cr = (uint8_t)(stream[i+1] >> 8);
					}

					red = round(Y + 1.402 * (Cr - 128));
					green = round(Y - 0.34414 * (Cb - 128) - 0.71414 * (Cr - 128));
					blue = round(Y + 1.772 * (Cb - 128));

					if (red > 255)	red = 255;		if (red < 0)	red = 0;
					if (green > 255)green = 255;	if (green < 0)	green = 0;
					if (blue > 255)	blue = 255;		if (blue < 0)	blue = 0;
				}

				else if (!strcmp(mode, mono_mode))
				{
					uint8_t k_temp = (uint8_t)stream[i];
					stream[i] = stream[i] >> 8 ;
					stream[i] = (k_temp << 8) | stream[i];
					red = (uint8_t)(stream[i] >> 8);
					green = (uint8_t)(stream[i] >> 8);
					blue = (uint8_t)(stream[i] >> 8);
				}
				else {printf("ERROR");break;}

				/*red = (uint16_t)(stream[i] >> 11);
				red = (uint8_t)(red * 255 / 31);
				green = (uint16_t)((uint16_t)(stream[i] << 5) >> 10);
				green = (uint8_t)(green * 255 / 63);
				blue = (uint16_t)((uint16_t)(stream[i] << 11) >> 11);
				blue = (uint8_t)(blue * 255 / 31);*/
				/*uint8_t k_temp = (uint8_t)stream[i];
				stream[i] = stream[i] >> 8 ;
				stream[i] = (k_temp << 8) | stream[i];
				red = (uint8_t)(stream[i] >> 8);
				green = (uint8_t)(stream[i] >> 8);
				blue = (uint8_t)(stream[i] >> 8);*/

				/*red = (uint8_t)(stream[i]);
				green = (uint8_t)(stream[i]);
				blue = (uint8_t)(stream[i]);*/

				fwrite(&red, 1, 1, file_frame);
				fwrite(&green, 1, 1, file_frame);
				fwrite(&blue, 1, 1, file_frame);
				//k++;
			}

		}

	}
fclose(file_frame);
return -1;
}

void set_mode_()
{
	printf("ВВЕДИТЕ РЕЖИМ ОБРАБОТКИ ВИДЕО\n");
	scanf("%s", mode);
}

void set_number()
{
	printf("ВВЕДИТЕ НОМЕР ВИДЕО\n");
	scanf("%d", &video_number);
}

//MAIN*****************
int main()
{
	//const char path_stream[]	= "/media/developer/SDHC/VIDEO-3.BIN";
	char 		path_folder[]	= "/home/developer/Рабочий стол/Frames/frame-";
	const char	extension[]		= ".ppm";

	char default_path[] = "/media/developer/SDHC/VIDEO-";
	char video_extension[] = ".BIN";

	set_mode_();
	while(((strcmp(mode, color_mode) != 0) & (strcmp(mode, mono_mode) != 0)))
	{
		if (((strcmp(mode, color_mode) != 0) & (strcmp(mode, mono_mode) != 0)))		//проверям совпадение введенного режима с возможными
			printf("MODE_ERROR\n");
		set_mode_();
	}
	int i = 0;
	int num = video_number;
	while(num >= 1)
	{
		num = num / 10;
		i++;
	}

number_l:
	set_number();
	/*создаем путь к видео*/
	char path_stream[strlen(default_path) + strlen(video_extension) + i];
	sprintf(path_stream, "%s%d%s", default_path, video_number, video_extension);
	printf("%s\n", path_stream);

	char number[4];
	int n = 0;
	char path_frame [strlen(path_folder) + strlen(number) + strlen(extension) + 1];

	//СОЗДАЕМ ФАЙЛ С ВИДЕОПОТОКОМ
	FILE * file_stream = fopen(path_stream, "r");		//файл с видеопотоком
	//generateStream(path_stream);
	if (file_stream == NULL)
	{
		printf("OPEN_ERROR\n");								//выводит ERROR если невозможно открыть файл
		goto number_l;
		return 0;
	}
	fseek(file_stream, 0, SEEK_END);					//переносим указатель в конец файла
	int pointer = ftell(file_stream);					//возвращаем функцией ftell длину файла в pointer
	printf("Размер потока видео: %d байт(а)\n", pointer);


	//выделяем в динамической памяти массив элементов uint16_t длины pointer и создаем указатель на него
	uint16_t * stream_uint = (uint16_t*)malloc(pointer);

	fseek(file_stream, 0, SEEK_SET);					//перемещаем указатель в начало файла
	fread(stream_uint, 2 * pointer-1, 1, file_stream);	//читаем из file_stream в созданный массив stream_uint один раз pointer бит

	int state = 0;
	while(1)
	{
		if (state == -1)
		{
			printf("Кадр № %d записан не полностью.\nКОНЕЦ ВИДЕОПОТОКА\n", n - 1);
			break;
		}
		setFrameNumber(n, number, 4);
		strcpy(path_frame, path_folder);
		strncat(path_frame, number, 4);
		strcat(path_frame, extension);

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
