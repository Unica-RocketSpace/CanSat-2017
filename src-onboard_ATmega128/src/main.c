/*
 * main.c
 *
 *  Created on: 19 нояб. 2016 г.
 *      Author: developer
 */

#include <stdio.h>

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <rscs/uart.h>
#include <rscs/i2c.h>
#include <rscs/spi.h>
#include <rscs/onewire.h>
#include <rscs/bmp280.h>
#include <rscs/ds18b20.h>
#include <rscs/timeservice.h>
#include <rscs/stdext/stdio.h>

#include "timer.h"
#include "radio_transmitter.h"
#include "kinematic_unit.h"
#include "dynamic_unit.h"
#include "MPU9255.h"

void blink_led_init()
{
	DDRG |= (1 << 3);
}

void blink_led()
{
	PORTG ^= (1 << 3);
	_delay_ms(20);
}

void extiInit()
{
	//Настройка внешнего прерывания INT7
	DDRE &= ~(1 << 7);
	EICRB |= (1 << ISC71) | (1 << ISC70);		//Установливаем режим Rising edge
	EIMSK |= (1 << INT7);						//Разрешаем внешнее прерывание
}

void send_calibration_values()
{
	const rscs_bmp280_calibration_values_t * bmp280_cal_values = rscs_bmp280_get_calibration_values(bmp280);
	uint8_t i = 10, mark = 0xFF;
	while(i--)
	{
		rscs_uart_write(uart0, &mark, 1);
		rscs_uart_write(uart0, &bmp280_cal_values, sizeof(bmp280_cal_values));
	}
}


void servo_test()
{
	for(int i = 0; i < 360; i++)
	{
		if (i < 180) setServoAngle(i - 90);
		else if (i == 180)
		{
			setServoAngle(i - 90);
			_delay_ms(500);
		}
		else setServoAngle(270 - i);
		_delay_ms(3);
	}
	_delay_ms(500);
}


void motor_test()
{
	for(int i = 0; i < 800; i++)
	{
		 if (i < 400)
		{
			setWheelSpeed(i - 200);
			_delay_ms(5);
		}
		else if (i == 400)
		{
			setWheelSpeed(i - 200);
			_delay_ms(1000);
		}
		else setWheelSpeed(600 - i);
		_delay_ms(5);
	}
	_delay_ms(500);
}

void motor_test_stop()
{
	setWheelSpeed(200);
	_delay_ms(2000);
	setWheelSpeed(0);
	_delay_ms(4000);
}


void printf_package(uint16_t package_number)
{
	printf("TIME: %f  [s]\n", (float)STATE.Time / 1000);
	printf("=============PACKAGE NUMBER [ %d ]====\n", package_number);
	printf("-----MAIN MISSION-----\n");
	printf("Accelerations (ADXL345): %f, %f, %f  [m/s^2]\n", G_VECT*STATE.aALT_XYZ[0]/1.08, G_VECT*STATE.aALT_XYZ[1]/1.08, G_VECT*STATE.aALT_XYZ[2]/1.17);
	printf("Pressure      (BMP280) : %f  [Pa]\n", STATE.pressure);
	printf("Temperature   (BMP280) : %f  [oC]\n", STATE.temp_bmp280);
	printf("Temperature   (DS18B20): %f  [oC]\n", STATE.temp_ds18b20);

	printf("-----ADDITIONAL MISSION-----\n");
	printf("Accelerations    (MPU9255): %f, %f, %f  [m/s^2]\n", STATE.aRelatedXYZ[0], STATE.aRelatedXYZ[1], STATE.aRelatedXYZ[2]);
	printf("Angle velocities (MPU9255): %f, %f, %f  [1/s]\n", STATE.gRelatedXYZ[0], STATE.gRelatedXYZ[1], STATE.gRelatedXYZ[2]);
	printf("Magnetic vector  (MPU9255): %f, %f, %f  [-]\n", STATE.cRelatedXYZ[0], STATE.cRelatedXYZ[1], STATE.cRelatedXYZ[2]);
	printf("=============END OF PACKAGE=============\n\n");
}

void printf_rotation_matrix()
{
	printf("(%f),   (%f),   (%f)\n", STATE.f_XYZ[0][0],
									 STATE.f_XYZ[0][1],
									 STATE.f_XYZ[0][2]);
	printf("(%f),   (%f),   (%f)\n", STATE.f_XYZ[1][0],
									 STATE.f_XYZ[1][1],
									 STATE.f_XYZ[1][2]);
	printf("(%f),   (%f),   (%f)\n", STATE.f_XYZ[2][0],
									 STATE.f_XYZ[2][1],
									 STATE.f_XYZ[2][2]);
	printf("\n");
}

void printf_rotation_matrix_string()
{
	printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n",	STATE.f_XYZ[0][0],
									 	 	 		STATE.f_XYZ[0][1],
									 	 	 		STATE.f_XYZ[0][2],
									 	 	 		STATE.f_XYZ[1][0],
									 	 	 		STATE.f_XYZ[1][1],
									 	 	 		STATE.f_XYZ[1][2],
									 	 	 		STATE.f_XYZ[2][0],
									 	 	 		STATE.f_XYZ[2][1],
									 	 	 		STATE.f_XYZ[2][2]);
}

void printf_state()
{
	printf("time = %f ns  ==================\n", (float)STATE.Time / 1000);
	printf("Accelerometer\n");
	printf("a_RSC: %f, %f, %f\n", STATE.aRelatedXYZ[0], STATE.aRelatedXYZ[1], STATE.aRelatedXYZ[2]);
	printf("Gyroscope\n");
	printf("w_RSC: %f, %f, %f\n", STATE.gRelatedXYZ[0], STATE.gRelatedXYZ[1], STATE.gRelatedXYZ[2]);
	printf("Accelerations\n");
	printf("a_ISC: %f, %f, %f\n", STATE.a_XYZ[0], STATE.a_XYZ[1], STATE.a_XYZ[2]);
	printf("Velocities\n");
	printf("v_ISC: %f, %f, %f\n", STATE.v_XYZ[0], STATE.v_XYZ[1], STATE.v_XYZ[2]);
	printf("Translations\n");
	printf("s_ISC: %f, %f, %f\n", STATE.s_XYZ[0], STATE.s_XYZ[1], STATE.s_XYZ[2]);
	printf("Angle velocities\n");
	printf("w_ISC: %f, %f, %f\n", STATE.w_XYZ[0], STATE.w_XYZ[1], STATE.w_XYZ[2]);
	printf("Rotation Matrix\n");
	printf_rotation_matrix();
}

int main()
{
	//*****ИНИЦИАЛИЗАЦИЯ БОРТА*****//
	_delay_ms(3000);
	blink_led_init();
	hardwareInit();
	kinematicInit();
	dynamicInit();
	STATE.state = 0;		//обнуляем состояние аппарата (на всякий случай)
	extiInit();

	/*while(1)
		printf("error");*/
	//*****УСТАНОВКА НАЧАЛЬНЫХ ПАРАМЕТРОВ*****//
	set_zero_pressure();	//устанавливаем нулевое давление
	//set_ISC_offset();
	/*for (int i = 0; i < 20; i++)
	{

		uint16_t marker = 0xF7F7;
		rscs_uart_write(uart0, &marker, sizeof(marker));
		rscs_uart_write(uart0, &STATE.f_XYZ, sizeof(STATE.f_XYZ));
		rscs_uart_write(uart0, &STATE.pressure, sizeof(STATE.pressure));
	}*/


	//*****ОЖИДАНИЕ РАСКРЫТИЯ ПАРАШЮТА*****//

	while((STATE.state & (1 << 0)) == 0)
	{//printf("while state = %d\n", STATE.state);
		pull_recon_data();
		send_package();
		blink_led();
	}
	//printf("next state = %d\n", STATE.state & (1 << 0));
	setWheelSpeed(200);
	//set_magn_dir();


	//ТЕСТ СЕРВЫ И ДВИГАТЕЛЯ
	/*while(1)
	{
		//motor_test();
		//servo_test();
		//blink_led();
	}*/


	while(1)
	{
		//*****ИНДИКАЦИЯ РАБОТЫ УСТРОЙСТВ*****//
		blink_led();

		//*****ОПРОС ДАТЧИКОВ*****//
		pull_recon_data();

		//set_magn_dir();
		//construct_trajectory();

		//Отправляем пакет в формате "PRINT"
		//printf_package(p_number);
		//printf_state();

		//Пересчитываем матрицу поворота по данным с магнитометра
		/*if (STATE.state & (1 << 1))
		{
			recalc_ISC();
			printf("recalced matrix:  \n");
			printf_rotation_matrix();
		 */

		//Отправляем пакет в формате "HEX"
		send_package();
		//TIMES.transmition = rscs_time_get() - TIMES.total;
		//TIMES.total = TIMES.total + TIMES.transmition;

	}

	return 0;
}

ISR (INT7_vect)
{
	STATE.state |= (1 << 0);
	//printf("STATE.state = %d\n", STATE.state);
}
