/*
 * ADXL345.h
 *
 *  Created on: 15 янв. 2017 г.
 *      Author: RaKetov
 */

#ifndef ADXL345_H_
#define ADXL345_H_

#include <rscs/spi.h>
#include <stdbool.h>

/* Команды на чтение и запись */
#define ADXL345_SPI_READ        (1 << 7)
#define ADXL345_SPI_WRITE       (0 << 7)
#define ADXL345_SPI_MB          (1 << 6)

/* ADXL345 Адреса регистров */
#define ADXL345_DEVID			0x00 // R	Device ID
#define ADXL345_OFSX            0x1E // R/W X-axis offset.
#define ADXL345_OFSY            0x1F // R/W Y-axis offset.
#define ADXL345_OFSZ            0x20 // R/W Z-axis offset.
#define ADXL345_BW_RATE         0x2C // R/W Data rate and power mode control.
#define ADXL345_POWER_CTL       0x2D // R/W Power saving features control.
#define ADXL345_INT_ENABLE      0x2E // R/W Interrupt enable control.
#define ADXL345_INT_MAP         0x2F // R/W Interrupt mapping control.
#define ADXL345_INT_SOURCE      0x30 // R   Source of interrupts.
#define ADXL345_DATA_FORMAT     0x31 // R/W Data format control.
#define ADXL345_DATAX0          0x32 // R   X-Axis Data 0.
#define ADXL345_DATAX1          0x33 // R   X-Axis Data 1.
#define ADXL345_DATAY0          0x34 // R   Y-Axis Data 0.
#define ADXL345_DATAY1          0x35 // R   Y-Axis Data 1.
#define ADXL345_DATAZ0          0x36 // R   Z-Axis Data 0.
#define ADXL345_DATAZ1          0x37 // R   Z-Axis Data 1.
#define ADXL345_FIFO_CTL        0x38 // R/W FIFO control.
#define ADXL345_FIFO_STATUS     0x39 // R   FIFO status.

/* ADXL345_BW_RATE Определение регистра */
#define ADXL345_LOW_POWER       (1 << 4)
#define ADXL345_RATE(x)         ((x) & 0xF)

/* ADXL345_POWER_CTL Определение регистра */
#define ADXL345_PCTL_LINK       (1 << 5)
#define ADXL345_PCTL_AUTO_SLEEP (1 << 4)
#define ADXL345_PCTL_MEASURE    (1 << 3)
#define ADXL345_PCTL_SLEEP      (1 << 2)
#define ADXL345_PCTL_WAKEUP(x)  ((x) & 0x3)

/* ADXL345_INT_ENABLE / ADXL345_INT_MAP / ADXL345_INT_SOURCE Определение регистра */
#define ADXL345_DATA_READY      (1 << 7)
#define ADXL345_SINGLE_TAP      (1 << 6)
#define ADXL345_DOUBLE_TAP      (1 << 5)
#define ADXL345_ACTIVITY        (1 << 4)
#define ADXL345_INACTIVITY      (1 << 3)
#define ADXL345_FREE_FALL       (1 << 2)
#define ADXL345_WATERMARK       (1 << 1)
#define ADXL345_OVERRUN         (1 << 0)

/* ADXL345_DATA_FORMAT Определение регистра */
#define ADXL345_SELF_TEST       (1 << 7)
#define ADXL345_SPI             (1 << 6)
#define ADXL345_INT_INVERT      (1 << 5)
#define ADXL345_FULL_RES        (1 << 3)
#define ADXL345_JUSTIFY         (1 << 2)
#define ADXL345_RANGE(x)        ((x) & 0x3)

/* ADXL345_RANGE(x) Возможные значения*/
#define ADXL345_RANGE_2G		0
#define ADXL345_RANGE_4G		1
#define ADXL345_RANGE_8G		2
#define ADXL345_RANGE_16G		3

/* ADXL345_FIFO_CTL Определение регистра */
#define ADXL345_FIFO_MODE(x)    (((x) & 0x3) << 6)
#define ADXL345_TRIGGER         (1 << 5)
#define ADXL345_SAMPLES(x)      ((x) & 0x1F)

/* ADXL345_FIFO_MODE(x) Возможные значения */
#define ADXL345_FIFO_BYPASS     0
#define ADXL345_FIFO_FIFO       1
#define ADXL345_FIFO_STREAM     2
#define ADXL345_FIFO_TRIGGER    3

/* ADXL345_FIFO_STATUS Определение регистра */
#define ADXL345_FIFO_TRIG       (1 << 7)
#define ADXL345_ENTRIES(x)      ((x) & 0x3F)

/* ADXL345 Full Resolution Scale Factor */
#define ADXL345_SCALE_FACTOR    0.0039



/***************************************************************************/
/******************** ИНИЦИАЛИЗАЦИЯ ПИНА CS ДЛЯ ADXL345 ********************/
/********************      ЗАДАЁТСЯ ПОЛЬЗОВАТЕЛЕМ       ********************/
/***************************************************************************/

#define ADXL345_CS_DDR			DDRB	// регистр DDR порта, на котором расположен CS пин
#define ADXL345_CS_PORT			PORTB	// регистр PORT порта, на котором расположен CS пин
#define ADXL345_CS_PIN			4		// номер пина CS в порту


/****************************************************************/
/********************   НАСТРОЙКИ ADXL345    ********************/
/******************** ЗАДАЁТСЯ ПОЛЬЗОВАТЕЛЕМ ********************/
/****************************************************************/

#define ADXL345_OFSX_DATA			0x00 // Смещение по оси X
#define ADXL345_OFSY_DATA			0x00 // Смещение по оси Y
#define ADXL345_OFSZ_DATA			0x00 // Смещение по оси Z
#define ADXL345_BW_RATE_DATA		ADXL345_RATE(1010) // LOW_POWER - OFF | 1010 - 100Гц
#define ADXL345_POWER_CTL_DATA		0x00 // LINK - OFF | AUTO_SLEEP - OFF | MEASURE - OFF | SLEEP - OFF | WAKEUP(x) - OFF.
#define ADXL345_INT_ENABLE_DATA		ADXL345_DATA_READY // DATA_READY - ON, остальные - OFF
#define ADXL345_INT_MAP_DATA		0x00 // 0 - все типы прерываний выведены на пин INT1
#define ADXL345_DATA_FORMAT_DATA	ADXL345_RANGE_4G // RANGE_4G - диапазон измерений +-4g, остальные параметры по умолчанию
#define ADXL345_FIFO_CTL_DATA		ADXL345_FIFO_STREAM   // Режим FIFO - STREAM, TRIGGER и SAMPLES не имеют значения.


/************************************************************/
/******************** ОБЪЯВЛЕНИЕ ФУНКЦИЙ ********************/
/************************************************************/

/*Управление линией CS ADXL345
 * Параметр:	state - задает состояние линии CS	(1 - линия поднята)
 * 													(0 - линия опущена) */
void ADXL345_CS_State(bool state);

/* ЧТЕНИЕ ЗНАЧЕНИЯ ИЗ РЕГИСТРА
 * Параметр:	registerAddress - адрес регистра
 * Возвращает:	Значение регистра */
uint8_t ADXL345_GetRegisterValue(uint8_t registerAddress);

/* ЗАПИСЬ ЗНАЧЕНИЯ В РЕГИСТР
 * Параметр:	registerAddress - адрес регистра
 * Параметр:	registerValue - записываемое значение региста */
void ADXL345_SetRegisterValue(uint8_t registerAddress, uint8_t registerValue);

/* ИНИЦИАЛИЗАЦИЯ ADXL345
 * Возвращает:	1 - если есть связь с ADXL345
 * 				0 - если ADXL345 не отвечает */
char ADXL345_Init();

/*! Places the device into standby/measure mode. */
void ADXL345_SetPowerMode(unsigned char pwrMode);

/* ЧТЕНИЕ ДАННЫХ ADXL345 В БИНАРНОМ ВИДЕ
 * Параметр:	хData - переменная, в которую будет записано непреобразованное ускорение по оси X
 * Параметр:	yData - переменная, в которую будет записано непреобразованное ускорение по оси Y
 * Параметр:	zData - переменная, в которую будет записано непреобразованное ускорение по оси Z */
void ADXL345_GetXYZ(uint16_t* xData, uint16_t* yData, uint16_t* zData);

/* ЧТЕНИЕ ДАННЫХ ADXL345 В ЕДИНИЦАХ g
 * Параметр:	хData - переменная, в которую будет записано непреобразованное ускорение по оси X
 * Параметр:	yData - переменная, в которую будет записано непреобразованное ускорение по оси Y
 * Параметр:	zData - переменная, в которую будет записано непреобразованное ускорение по оси Z
 * Параметр:	хData_G - переменная, в которую будет записано преобразованное ускорение по оси X
 * Параметр:	yData_G - переменная, в которую будет записано преобразованное ускорение по оси X
 * Параметр:	zData_G - переменная, в которую будет записано преобразованное ускорение по оси X
 *  */
void ADXL345_GetGXYZ(uint16_t* xData, uint16_t* yData, uint16_t* zData, float* xData_G, float* yData_G, float* zData_G);

/*! Sets an offset value for each axis (Offset Calibration). */
void ADXL345_SetOffset(uint8_t xOffset, uint8_t yOffset, uint8_t zOffset);

#endif /* ADXL345_H_ */
