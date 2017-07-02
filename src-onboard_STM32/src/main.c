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
#define BUFFER_SIZE				(1280*14)

#define LINES_PER_BUFFER		(BUFFER_SIZE/640/2)		//640*300*2/BUFFER_SIZE


typedef enum
{
	OV7670_START_FRAME	= 0xFF7C0000,
	OV7670_END_FRAME	= 0x007CFFFF,
	OV7670_SE_FRAME		= 0xFF7C0000007CFFFF	//end-start
} ov7670_flag;

static const ov7670_flag OV7670_FLAG = OV7670_SE_FRAME;
static const char filename[] = "video-";

static dump_state_t stream_file;
static uint8_t _buffer[BUFFER_SIZE];


void blink_led()
{
	static bool led_st = false;
	GPIO_WriteBit(GPIOC, GPIO_Pin_13, led_st ? RESET : SET);
	led_st = !led_st;
}



int main(int argc, char* argv[])
{
	(void)argc, (void)argv;
	blink_led_init();
	camera_init();
	dump_init(&stream_file, filename);

again:
	camera_wait_vsync();
	camera_fifo_reset_write();
	camera_fifo_start_capture(LINES_TO_SAVE_OFFSET, LINES_TO_SAVE);

	// ждем, пока в FIFO появится достаточно данных
	while (camera_fifo_lines_left() != 0) {}

	// и читаем и пишем на флешку
	camera_fifo_reset_read();

	dump(&stream_file, &OV7670_FLAG, 8);
	size_t buff_cnt = 0;
	while (buff_cnt*LINES_PER_BUFFER < LINES_TO_SAVE)
	{
		camera_fifo_reset_read();
		camera_fifo_read(_buffer, BUFFER_SIZE);
		dump(&stream_file, _buffer, BUFFER_SIZE);
		buff_cnt++;
	}

	blink_led();
	// продолжаем
	goto again;

	return 0;
}



