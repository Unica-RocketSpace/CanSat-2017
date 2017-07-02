#include <stdio.h>
#include <stdlib.h>

#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>

#include <stm32f10x_conf.h>
#include <core_cm3.h>

#include "diag/Trace.h"

#include "BlinkLed.h"
#include "ov7670/camera.h"
#include "sd.h"
#include "dump.h"


#define LINES_TO_SAVE_OFFSET	(90)
#define LINES_TO_SAVE			(300)
#define BUFFER_SIZE				(1280*6)
#define BUFFER_CNT 				(2)

#define LINES_PER_BUFFER		(BUFFER_SIZE/640/2)		//640*300*2/BUFFER_SIZE


typedef enum
{
	OV7670_START_FRAME	= 0xFF7C0000,
	OV7670_END_FRAME	= 0x007CFFFF,
	OV7670_SE_FRAME		= 0xFF7C0000007CFFFF	//end-start
} ov7670_flag;

static const ov7670_flag OV7670_FLAG = OV7670_SE_FRAME;
static const char filename[] = "video-";

static QueueHandle_t _fullBufferQ;
static StaticQueue_t _fullBufferQ_ob;
static uint8_t _fullBufferQ_data[BUFFER_CNT * sizeof(uint8_t *)];

static QueueHandle_t _emptyBufferQ;
static StaticQueue_t _emptyBufferQ_ob;
static uint8_t _emptyBufferQ_data[BUFFER_CNT * sizeof(uint8_t *)];

static StackType_t _cam_task_stack[120];
static StaticTask_t _cam_taskb_ob;

static StackType_t _sd_task_stack[400];
static StaticTask_t _sd_task_ob;

static dump_state_t stream_file;
static uint8_t _buffers[BUFFER_CNT][BUFFER_SIZE];


void blink_led()
{
	static bool led_st = false;
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, led_st ? RESET : SET);
	led_st = !led_st;
}


void cam_task(void* args);


void sd_task(void * args)
{
	(void)args;

	dump_init(&stream_file, filename);

	_emptyBufferQ = xQueueCreateStatic(BUFFER_CNT, sizeof(uint8_t*), _emptyBufferQ_data,
			&_emptyBufferQ_ob);

	_fullBufferQ = xQueueCreateStatic(BUFFER_CNT, sizeof(uint8_t*), _fullBufferQ_data,
			&_fullBufferQ_ob);

	for (size_t i = 0; i < BUFFER_CNT; i++)
	{
		uint8_t * buffPtr = _buffers[i];
		xQueueSendToBack(_emptyBufferQ, &buffPtr, portMAX_DELAY);
	}

	xTaskCreateStatic(cam_task, "cam", sizeof(_cam_task_stack)/sizeof(_cam_task_stack[0]), NULL, 1,
			_cam_task_stack, &_cam_taskb_ob);


	dump(&stream_file, &OV7670_FLAG, 8);
	size_t lines_written = 0;
	while(1)
	{
		uint8_t * buffer;
		//Пишем на SD флаг начала кадра
		xQueueReceive(_fullBufferQ, &buffer, portMAX_DELAY);
		dump(&stream_file, buffer, BUFFER_SIZE);
		xQueueSend(_emptyBufferQ, &buffer, portMAX_DELAY);
		lines_written += LINES_PER_BUFFER;
		if (lines_written == LINES_TO_SAVE)
		{
			dump(&stream_file, &OV7670_FLAG, 8);
			blink_led();
			lines_written = 0;
		}
	}
}


void cam_task(void* args)
{
	(void)args;

	camera_init();

again:
	//taskENTER_CRITICAL();
	camera_wait_vsync();
	//taskEXIT_CRITICAL();
	camera_fifo_reset_write();
	camera_fifo_start_capture(LINES_TO_SAVE_OFFSET, LINES_TO_SAVE);

	// и читаем и пишем на флешку
	camera_fifo_reset_read();


	size_t buff_cnt = 0;
	while (buff_cnt*LINES_PER_BUFFER < LINES_TO_SAVE)
	{
		// ждем, пока в FIFO появится достаточно данных
		while (LINES_TO_SAVE - camera_fifo_lines_left() < buff_cnt*LINES_PER_BUFFER) {}

		// Если в фифо уже накопилось данных хотябы на один буфер
		// начинаем выгрузку на SD карту
		{
			uint8_t * buffer;
			xQueueReceive(_emptyBufferQ, &buffer, portMAX_DELAY);
			camera_fifo_read(buffer, BUFFER_SIZE);
			xQueueSend(_fullBufferQ, &buffer, portMAX_DELAY);

			buff_cnt++;
		}
	}

	// продолжаем
	goto again;
}


int main(int argc, char* argv[])
{
	(void)argc, (void)argv;
	blink_led_init();
	xTaskCreateStatic(sd_task, "sd", sizeof(_sd_task_stack)/sizeof(_sd_task_stack[0]), NULL, 2, _sd_task_stack, &_sd_task_ob);
	vTaskStartScheduler();

	return 0;
}



