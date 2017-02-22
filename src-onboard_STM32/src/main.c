//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"

#include "Timer.h"
#include "BlinkLed.h"

// ----------------------------------------------------------------------------
//
// Standalone STM32F1 led blink sample (trace via DEBUG).
//
// In debug configurations, demonstrate how to print a greeting message
// on the trace device. In release configurations the message is
// simply discarded.
//
// Then demonstrates how to blink a led with 1 Hz, using a
// continuous loop and SysTick delays.
//
// Trace support is enabled by adding the TRACE macro definition.
// By default the trace messages are forwarded to the DEBUG output,
// but can be rerouted to any device or completely suppressed, by
// changing the definitions required in system/src/diag/trace_impl.c
// (currently OS_USE_TRACE_SEMIHOSTING_DEBUG/_STDOUT).
//
// The external clock frequency is specified as a preprocessor definition
// passed to the compiler via a command line option (see the 'C/C++ General' ->
// 'Paths and Symbols' -> the 'Symbols' tab, if you want to change it).
// The value selected during project creation was HSE_VALUE=72000000.
//
// Note: The default clock settings take the user defined HSE_VALUE and try
// to reach the maximum possible system clock. For the default 8 MHz input
// the result is guaranteed, but for other values it might not be possible,
// so please adjust the PLL settings in system/src/cmsis/system_stm32f10x.c
//

// ----- Timing definitions -------------------------------------------------

// Keep the LED on for 2/3 of a second.
#define BLINK_ON_TICKS  (TIMER_FREQUENCY_HZ * 3 / 4)
#define BLINK_OFF_TICKS (TIMER_FREQUENCY_HZ - BLINK_ON_TICKS)

// ----- main() ---------------------------------------------------------------

// Sample pragmas to cope with warnings. Please note the related line at
// the end of this function, used to pop the compiler diagnostics status.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

int
main(int argc, char* argv[])
{
  SPI_TypeDef SPI;
  SPI_InitTypeDef InitSPI;
  
  InitSPI.SPI_Direction = SPI_Direction_2Lines_FullDuplex;	//режим передачи по двум каналам MOSI, MISO
  InitSPI.SPI_DataSize	= SPI_DataSize_8b;	//размер пакета 8 бит
  InitSPI.SPI_CPOL		= SPI_CPOL_Low;		//
  InitSPI.SPI_CPHA		= SPI_CPHA_1Edge;	//
  InitSPI.SPI_NSS		= SPI_NSS_Soft;		//управление пином CS программно
  InitSPI.SPI_BaudRatePrescaler	= SPI_BaudRatePrescaler_32;	//Предделитель SCK
  InitSPI.SPI_FirstBit	= SPI_FirstBit_MSB;	//Отправляем сначала старший бит
  InitSPI.SPI_Mode		= SPI_Mode_Master;	//режим - мастер;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
