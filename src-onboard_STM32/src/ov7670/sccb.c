#include <stdbool.h>

#include <stm32f10x_conf.h>

#include "diag/Trace.h"

#include "sccb.h"

#include "defs_OV7670.h"

void sccb_init(void) {

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_AHBPeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

	// GPIO config
	GPIO_InitTypeDef portInit;
	GPIO_StructInit(&portInit);
	portInit.GPIO_Mode =  GPIO_Mode_AF_OD;
	portInit.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	portInit.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &portInit);


	// I2C config
	I2C_InitTypeDef i2c;
	I2C_StructInit(&i2c);
	i2c.I2C_Mode = I2C_Mode_I2C;
	i2c.I2C_DutyCycle = I2C_DutyCycle_2;
	i2c.I2C_OwnAddress1 = 0x00;
	i2c.I2C_Ack = I2C_Ack_Enable;
	i2c.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	i2c.I2C_ClockSpeed = 100000;
	I2C_Init(I2C1, &i2c);
	I2C_ITConfig(I2C1, I2C_IT_ERR, ENABLE);
	I2C_Cmd(I2C1, ENABLE);
	//I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);
}

bool sccb_write(uint8_t reg_addr, uint8_t data) {
	uint32_t timeout = 0x7FFFFF;

	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY)) {
		if ((timeout--) == 0) {
			trace_printf("Busy Timeout\r\n");
			return true;
		}
	}

	// Send start bit
	I2C_GenerateSTART(I2C1, ENABLE);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT)) {
		if ((timeout--) == 0) {
			trace_printf("Start bit Timeout\r\n");
			return true;
		}
	}

	// Send slave address (camera write address)
	I2C_Send7bitAddress(I2C1, OV7670_I2C_ADDR, I2C_Direction_Transmitter);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		if ((timeout--) == 0) {
			trace_printf("Slave address timeout\r\n");
			return true;
		}
	}

	// Send register address
	I2C_SendData(I2C1, reg_addr);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			trace_printf("Register timeout\r\n");
			return true;
		}
	}

	// Send new register value
	I2C_SendData(I2C1, data);

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
		if ((timeout--) == 0) {
			trace_printf("Value timeout\r\n");
			return true;
		}
	}

	// Send stop bit
	I2C_GenerateSTOP(I2C1, ENABLE);
	return false;
}
