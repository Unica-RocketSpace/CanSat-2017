#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include "timer.h"
#include "wheel_control.h"

uint8_t cntr;

void timer_init(void)
{
   //Инициируем регистр управления таймером
   TCCR0 = (0 << FOC0 ) |	//Включает принудительное внешнее сравнение
	   (1 << WGM01) |	//Определяет режим таймера (10 - режим CTC)
	   (0 << WGM00) |
	   (0 << COM01) |	//подключает к таймеру пин OC0 (00 - пин отключен)
	   (0 << COM00) |
	   (1 << CS02 ) |	//Выставляет предделитель (111 - предделитель = 1024)
	   (1 << CS01 ) |
	   (1 << CS00 );
   TCNT0 = 0x00;			//Сбрасываем таймер на 0
   OCR0  = 255;			//Номер такта, при котором будет сбрасываться счетчик
   TIMSK |= (1 << OCIE0);		//Разрешает прерывания по сравнению
   cntr = 0;
   sei();
}


ISR(TIMER0_COMP_vect)
{
	cntr++;
	if (cntr == 1)
	{
		wheel_run(1, globalWheelSpeed[0]);
		wheel_run(2, globalWheelSpeed[1]);
		wheel_run(3, globalWheelSpeed[2]);
	}
}


