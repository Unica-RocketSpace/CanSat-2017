/*
 * ADXL345.c
 *
 *  Created on: 15 янв. 2017 г.
 *      Author: RaKetov
 */

#include "ADXL345.h"
#include <rscs/spi.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>


char selectedRange     = 0;
char fullResolutionSet = 0;


//ИНИЦИАЛИЗАЦИЯ ШИНЫ SPI (временно)
rscs_spi_bus_t spi1_;
rscs_spi_bus_t * spi1 = &spi1_;



/*Управление линией CS ADXL345*/
void ADXL345_CS_State(bool state)
{
	if (state)
		ADXL345_CS_PORT |= (1 << ADXL345_CS_PIN);
	else
		ADXL345_CS_PORT &= ~(1 << ADXL345_CS_PIN);
}

/*ЧТЕНИЕ ЗНАЧЕНИЯ ИЗ РЕГИСТРА*/
uint8_t ADXL345_GetRegisterValue(uint8_t registerAddress)
{
	uint8_t data = 0;
	registerAddress = ADXL345_SPI_READ | registerAddress;

	ADXL345_CS_State(0);
	rscs_spi_write(spi1, &registerAddress, 1);
	rscs_spi_read(spi1, &data, 1, 0);
	ADXL345_CS_State(1);

    return data;
}

/*ЗАПИСЬ ЗНАЧЕНИЯ В РЕГИСТР*/
void ADXL345_SetRegisterValue(uint8_t registerAddress, uint8_t registerValue)
{
    uint8_t dataBuffer[2] = {0, 0};
    dataBuffer[0] = ADXL345_SPI_WRITE | registerAddress;
	dataBuffer[1] = registerValue;

	ADXL345_CS_State(0);
	rscs_spi_write(spi1, dataBuffer, 2);
	ADXL345_CS_State(1);
}

/*ИНИЦИАЛИЗАЦИЯ ADXL345*/
char ADXL345_Init()
{
	ADXL345_CS_DDR |= (1 << ADXL345_CS_PIN);

	unsigned char status = 1;

    if(ADXL345_GetRegisterValue(ADXL345_DEVID) != 0xE5)
    {
        status = 0;
        return status;
    }

    uint8_t writeBuffer[18] = {	ADXL345_SPI_WRITE | ADXL345_OFSX,			ADXL345_OFSX_DATA,
								ADXL345_SPI_WRITE | ADXL345_OFSY,			ADXL345_OFSY_DATA,
								ADXL345_SPI_WRITE | ADXL345_OFSZ,			ADXL345_OFSZ_DATA,
								ADXL345_SPI_WRITE | ADXL345_BW_RATE,		ADXL345_BW_RATE_DATA,
								ADXL345_SPI_WRITE | ADXL345_POWER_CTL,		ADXL345_POWER_CTL_DATA,
								ADXL345_SPI_WRITE | ADXL345_INT_ENABLE,		ADXL345_INT_ENABLE_DATA,
								ADXL345_SPI_WRITE | ADXL345_INT_MAP,		ADXL345_INT_MAP_DATA,
								ADXL345_SPI_WRITE | ADXL345_DATA_FORMAT,	ADXL345_DATA_FORMAT_DATA,
								ADXL345_SPI_WRITE | ADXL345_FIFO_CTL,		ADXL345_FIFO_CTL_DATA		};

    rscs_spi_write(spi1, writeBuffer, 18);

    return status;
}

/***************************************************************************//**
 * @brief Places the device into standby/measure mode.
 *
 * @param pwrMode - Power mode.
 *			Example: 0x0 - standby mode.
 *				 0x1 - measure mode.
 *
 * @return None.
*******************************************************************************/
void ADXL345_SetPowerMode(unsigned char pwrMode)
{
    unsigned char oldPowerCtl = 0;
    unsigned char newPowerCtl = 0;

    oldPowerCtl = ADXL345_GetRegisterValue(ADXL345_POWER_CTL);
    newPowerCtl = oldPowerCtl & ~ADXL345_PCTL_MEASURE;
    newPowerCtl = newPowerCtl | (pwrMode * ADXL345_PCTL_MEASURE);
    ADXL345_SetRegisterValue(ADXL345_POWER_CTL, newPowerCtl);
}

/* ЧТЕНИЕ ДАННЫХ ADXL345 В БИНАРНОМ ВИДЕ*/
void ADXL345_GetXYZ(uint16_t* xData, uint16_t* yData, uint16_t* zData)
{
    uint8_t firstRegAddress = ADXL345_DATAX0;
    uint8_t readBuffer[6]   = {0, 0, 0, 0, 0, 0};

    firstRegAddress = ADXL345_SPI_READ | ADXL345_SPI_MB | firstRegAddress;

	ADXL345_CS_State(0);
	rscs_spi_write(spi1, &firstRegAddress, 1);
	rscs_spi_read(spi1, readBuffer, 6, 0);
	ADXL345_CS_State(1);

	*xData = (readBuffer[1] << 8) + readBuffer[0];	//X-axis's output binary data
	*yData = (readBuffer[3] << 8) + readBuffer[2];	//Y-axis's output binary data
	*zData = (readBuffer[5] << 8) + readBuffer[4];	//Z-axis's output binary data
}

/* ЧТЕНИЕ ДАННЫХ ADXL345 В БИНАРНОМ ВИДЕ И ПРЕОБРАЗОВАНИЕ В ЕДИНИЦЫ g */
void ADXL345_GetGXYZ(uint16_t* xData, uint16_t* yData, uint16_t* zData, float* xData_G, float* yData_G, float* zData_G)
{
	*xData = 0;  // X-axis's output data, xData - address of the binary X-axis's output data
	*yData = 0;  // Y-axis's output data, yData - address of the binary Y-axis's output data
	*zData = 0;  // Z-axis's output data, zData - address of the binary Z-axis's output data
	uint8_t  range = 1;

	for (size_t i = 0; i < (ADXL345_DATA_FORMAT_DATA & 0x3); i++) {range = range * 2;}

    ADXL345_GetXYZ(xData, yData, zData);

    if(*xData > 0x200) {*xData = 0x400 - *xData;}
    if(*yData > 0x200) {*yData = 0x400 - *yData;}
    if(*zData > 0x200) {*zData = 0x400 - *zData;}

    *xData_G  = (float)((*xData) * ADXL345_SCALE_FACTOR * range);	//X-axis's output transformed data
    *yData_G = (float)((*yData) * ADXL345_SCALE_FACTOR * range);	//Y-axis's output transformed data
    *zData_G = (float)((*zData) * ADXL345_SCALE_FACTOR * range);	//Z-axis's output transformed data
}

/*ЗАДАЕТ СМЕЩЕНИЕ ЗНАЧЕНИЙ ПО ТРЕМ ОСЯМ*/
void ADXL345_SetOffset(uint8_t xOffset, uint8_t yOffset, uint8_t zOffset)
{
	ADXL345_SetRegisterValue(ADXL345_OFSX, xOffset);
	ADXL345_SetRegisterValue(ADXL345_OFSY, yOffset);
	ADXL345_SetRegisterValue(ADXL345_OFSZ, yOffset);
}



