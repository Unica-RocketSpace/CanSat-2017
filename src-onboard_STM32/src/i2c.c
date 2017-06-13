/*
 * bmp280_local.c
 *
 *  Created on: 3 июн. 2017 г.
 *      Author: snork
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <stm32f10x_conf.h>
#include <stm32f10x_i2c.h>

#pragma GCC diagnostic push // очень много варнингов на эту тему от фриртоса и fatfs
#pragma GCC diagnostic ignored "-Wpadded"
//#include <FreeRTOS.h>
//#include <task.h>
#pragma GCC diagnostic pop




#define I2C_TIMEOUT 0xFFFF

//Макрос для возможности обработки ошибок
#define GOTO_END_IF_ERROR(OP) error = OP; if(error != 0) goto end;


inline static int8_t _wait_for_i2c_event(I2C_TypeDef * bus, uint32_t event)
{

   for (size_t count = 0; count < I2C_TIMEOUT; count++)
   {
      bool retval;

      retval = (I2C_CheckEvent(bus, event) == SUCCESS);

      if (retval)
         return 0;
   }

   return -1;
}


static int8_t _i2c_start(I2C_TypeDef * bus)
{
	int8_t error;

	I2C_GenerateSTART(bus, ENABLE);
	GOTO_END_IF_ERROR(_wait_for_i2c_event(bus, I2C_EVENT_MASTER_MODE_SELECT)); // EV 8_2

end:
	return error;
}

static int8_t _i2c_send_slaw(I2C_TypeDef * bus, uint8_t addr, uint8_t direction)
{
	int8_t error = 0;
	// отправляем адрес
	I2C_Send7bitAddress(bus, addr, direction);
	if (direction == I2C_Direction_Transmitter)
	{
		GOTO_END_IF_ERROR(_wait_for_i2c_event(bus, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); 	//EV-6
	}
	else
	{
		GOTO_END_IF_ERROR(_wait_for_i2c_event(bus, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)); 	//EV-9
	}

	(void)I2C1->SR2;

end:
	return error;
}


static int8_t _i2c_write(I2C_TypeDef * bus, const void * buffer_, size_t buffersize)
{
	int8_t error = 0;

	const uint8_t * buffer = (const uint8_t*)buffer_;
	while(buffersize--)
	{
		I2C_SendData(bus, *(buffer++));
		GOTO_END_IF_ERROR(_wait_for_i2c_event(bus, I2C_EVENT_MASTER_BYTE_TRANSMITTED)); // EV 8_2
	}

end:
	return error;
}


static int8_t _i2c_read(I2C_TypeDef * bus, void * buffer_, size_t buffersize, bool nackAtEnd)
{
	int8_t error = 0;

	I2C_AcknowledgeConfig(bus, ENABLE);
	uint8_t * buffer = (uint8_t*)buffer_;
	while(buffersize--)
	{
		if (buffersize == 1 && nackAtEnd)
			I2C_AcknowledgeConfig(bus, DISABLE);

		GOTO_END_IF_ERROR(_wait_for_i2c_event(bus, I2C_EVENT_MASTER_BYTE_RECEIVED)); // EV 8_2
		*(buffer++) = I2C_ReceiveData(bus);
	}

end:
	return error;
}


static void _i2c_stop(I2C_TypeDef * bus)
{
	I2C_GenerateSTOP(bus, ENABLE);
}




int8_t  i2c_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int8_t error;
	GOTO_END_IF_ERROR(_i2c_start(I2C1));
	GOTO_END_IF_ERROR(_i2c_send_slaw(I2C1, dev_addr, I2C_Direction_Transmitter));
	GOTO_END_IF_ERROR(_i2c_write(I2C1, &reg_addr, 1));
	GOTO_END_IF_ERROR(_i2c_write(I2C1, reg_data, cnt));

end:
	_i2c_stop(I2C1);
	return error;
}


int8_t i2c_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
	int8_t error;

	GOTO_END_IF_ERROR(_i2c_start(I2C1));
	GOTO_END_IF_ERROR(_i2c_send_slaw(I2C1, dev_addr, I2C_Direction_Transmitter));
	GOTO_END_IF_ERROR(_i2c_write(I2C1, &reg_addr, 1));
	_i2c_stop(I2C1);
	GOTO_END_IF_ERROR(_i2c_start(I2C1));
	GOTO_END_IF_ERROR(_i2c_send_slaw(I2C1, dev_addr, I2C_Direction_Receiver));
	GOTO_END_IF_ERROR(_i2c_read(I2C1, reg_data, cnt, false));


end:
	_i2c_stop(I2C1);
	return error;
}


int i2c_init(void)
{
	// Настройка I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	GPIO_InitTypeDef portInit;
	GPIO_StructInit(&portInit);
	portInit.GPIO_Mode =  GPIO_Mode_AF_OD;
	portInit.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	portInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &portInit);

	I2C_InitTypeDef i2c;
	I2C_StructInit(&i2c);
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c.I2C_ClockSpeed = 400;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_OwnAddress1 = 0x7F;

	I2C_Init(I2C1, &i2c);

	I2C_Cmd(I2C1, ENABLE);
	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);

	s32 com_rslt = ERROR;

	return com_rslt;
}
/************************* END DE-INITIALIZATION **********************/
