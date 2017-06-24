/*#include <stdio.h>
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
#include "ov7670/sccb.h"
#include "ov7670/defs_OV7670.h"


#define BUFFER_SIZE			(1280*4)
#define BUFFER_CNT 			(2)
#define BUFFER_READ_COUNT	(640*300*2/BUFFER_SIZE)		//640*300*2/BUFFER_SIZE

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
	//Пишем на SD флаг начала кадра
	dump(&stream_file, &OV7670_FLAG, 8);

	//_emptyBufferQ = xQueueCreate(BUFFER_CNT, sizeof(uint8_t*));
	//_fullBufferQ = xQueueCreate(BUFFER_CNT, sizeof(uint8_t*));
	_emptyBufferQ = xQueueCreateStatic(BUFFER_CNT, sizeof(uint8_t*), _emptyBufferQ_data,
			&_emptyBufferQ_ob);

	_fullBufferQ = xQueueCreateStatic(BUFFER_CNT, sizeof(uint8_t*), _fullBufferQ_data,
			&_fullBufferQ_ob);

	for (size_t i = 0; i < BUFFER_CNT; i++)
	{
		uint8_t * buffPtr = _buffers[i];
		xQueueSendToBack(_emptyBufferQ, &buffPtr, portMAX_DELAY);
	}

	//xTaskCreate(cam_task, "cam", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreateStatic(cam_task, "cam", sizeof(_cam_task_stack)/sizeof(_cam_task_stack[0]), NULL, 1,
			_cam_task_stack, &_cam_taskb_ob);

	while(1)
	{
		int dump_buffers_cnt = 0;
		dump_buffers_cnt++;
		uint8_t * buffer;
		xQueueReceive(_fullBufferQ, &buffer, portMAX_DELAY);
		dump(&stream_file, buffer, sizeof(buffer));
		xQueueSend(_emptyBufferQ, &buffer, portMAX_DELAY);


		if (dump_buffers_cnt == 50)
			dump(&stream_file, &OV7670_FLAG, 8);

	}
}

int ov7670_read_(uint8_t reg, uint8_t * data)
{
	uint32_t timeout = 0x7FFFFF;
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		if ((timeout--) == 0) {
			trace_printf("Busy Timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send start bit
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			trace_printf("Start bit Timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send slave address (camera write address)
	I2C_Send7bitAddress(I2C1, OV7670_SCCB_WRITE_ADDR, I2C_Direction_Transmitter);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			trace_printf("Slave address timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send register address
	I2C_SendData(I2C1, reg);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			trace_printf("Register timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}



	// Send start again
	I2C_GenerateSTART(I2C1, ENABLE);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			trace_printf("Start bit Timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Send slave address (camera read address)
	I2C_Send7bitAddress(I2C1, OV7670_SCCB_WRITE_ADDR, I2C_Direction_Receiver);
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			trace_printf("Slave address timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}

	// Read reg value
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
		if ((timeout--) == 0) {
			trace_printf("Value timeout\r\n");
			return -pdFREERTOS_ERRNO_ECANCELED;
		}
	}
	*data = I2C_ReceiveData(I2C1);

	// Send stop bit
	I2C_GenerateSTOP(I2C1, ENABLE);
	return 0;

}

void cam_task(void* args)
{
	(void)args;

	camera_init();

again:
	taskENTER_CRITICAL();
	camera_wait_vsync();
	taskEXIT_CRITICAL();
	camera_fifo_reset_write();
	camera_fifo_start_capture(BUFFER_READ_COUNT);

	// и читаем и пишем на флешку
	camera_fifo_reset_read();

	size_t buff_cnt = 0;
	while (buff_cnt < BUFFER_READ_COUNT)
	{
		if (camera_fifo_lines_left() <= BUFFER_READ_COUNT - 2)
		{
			uint8_t * buffer;
			xQueueReceive(_emptyBufferQ, &buffer, portMAX_DELAY);
			camera_fifo_read(buffer, BUFFER_SIZE);
			xQueueSend(_fullBufferQ, &buffer, portMAX_DELAY);

			buff_cnt++;
		}
	}

	blink_led();

	// продолжаем
	goto again;
}

void task()
{
	blink_led_init();
	camera_init();
	uint8_t pid;
	while(1)
	{
		blink_led();
		ov7670_read_(0x0a, &pid);
		trace_printf("pid: %d", pid);
	}
}

int main(int argc, char* argv[])
{
	(void)argc, (void)argv;
	static StaticTask_t _task_ob;
	static StackType_t _task_stack[50];


	xTaskCreateStatic(task, "1", configMINIMAL_STACK_SIZE, NULL, 1, _task_stack, &_task_ob);

	vTaskStartScheduler();


	return 0;
}


int main_x(int argc, char* argv[])
{
	(void)argc, (void)argv;
	blink_led_init();

	//xTaskCreate(sd_task, "sd", 3*configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreateStatic(sd_task, "sd", sizeof(_sd_task_stack)/sizeof(_sd_task_stack[0]),
			NULL, 1, _sd_task_stack, &_sd_task_ob);


	vTaskStartScheduler();

	return 0;
}



*/
