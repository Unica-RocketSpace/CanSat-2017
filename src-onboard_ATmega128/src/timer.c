#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "timer.h"


void timer1PWMInit()
{
	//Включаем на вывод OCR1A и OCR1B
	DDRB |= (1 << 5);
	DDRB |= (1 << 6);

	// настраиваем таймер
	TCCR1A =	(1 << WGM11)  |
				(0 << WGM10)  |
				(1 << COM1A1) |
				(0 << COM1A0) |
				(1 << COM1B1) |
				(0 << COM1B0);

	TCCR1B = 	(1 << WGM13)  |
				(1 << WGM12);

	ICR1 = 20000;	// пототолок таймера
	OCR1A = 0;		// начальная скорость двигателя
	OCR1B = 0;		// начальное положение сервы

	// запускаем таймер (продделитель 8)
	TCCR1B |=	(0 << CS12) |
				(1 << CS11) |
				(0 << CS10);
}
