/*
 * sd_spi.c
 *
 *  Created on: 13 июля 2016 г.
 *      Author: snork
 */

#include "sd.h"
//#include "../include/sd.h"

#include <FreeRTOS.h>
#include <task.h>

#include <string.h>
#include <errno.h>

#include <stm32f10x_rcc.h>
#include <stm32f10x_dma.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_spi.h>

#include "crc.h"

// =========================================================
// Настроечки
// =========================================================
#define SD_SPI SPI2 // настройки пинов и порта сюда пока не вынесены
#define SD_DMA DMA1 // Зависит от устройства

//#define SD_DMA_CHANNEL_RX  DMA1_Channel4
//#define SD_DMA_CHANNEL_TX  DMA1_Channel5
//
//#define SD_DMA_FLAG_RX_TC  DMA1_IT_TC4 // Успешно завершено
//#define SD_DMA_FLAG_TX_TC  DMA1_IT_TC5
//
//#define SD_DMA_FLAG_RX_ER  DMA1_IT_TE4 // Завершено c ошибкой
//#define SD_DMA_FLAG_TX_ER  DMA1_IT_TE5

#define SD_BAUD_RATE_PRESCALER_SLOW SPI_BaudRatePrescaler_256  //!< для первичной прокачки и инициализации SD карты
#define SD_BAUD_RATE_PRESCALER_FAST SPI_BaudRatePrescaler_4    //!< для основной работы. Быстрее не может :c

// =========================================================
// Вспомогательные сущности
// =========================================================
#define SD_R1_IDLE (1 << 0)
#define SD_R1_ILLEGAL_CMD (1 << 2)
#define SD_R1_CRC_ERR (1 << 3)

#define SD_DATA_RESP_MASK           0x1F  // маска ответа SD карты о приеме данных
#define SD_DATA_RESP_DATA_ACCEPTED  0x05  // ответ SD карты - данные получены
#define SD_DATA_RESP_CRC_ERROR      0x0B  // ответ SD карты - ошибка CRC
#define SD_DATA_RESP_WRITE_ERROR    0x0D  // ответ SD карты - ошибка записи

#define SD_TOKEN_SINGLE_DATA 0xFE   // Токен заголовка пакета для CMD17/18/24
#define SD_TOKEN_MULTI_DATA 0xFC    // Токен заголовка пакета для CMD25
#define SD_TOKEN_NO_MORE_DATA 0xFD  // Токен заверешения передачи данных для CMD25

#define GOTO_END_IF_ERROR(X) if ((error = (X)) != SD_ERROR_NONE) goto end;

#define SD_CMD_TEAK_TIMEOUT (100)

// =========================================================
// Статики
// =========================================================

// Тип запроса к SD карте
typedef enum
{
   SD_REQUEST_WRITE = 0x10,
   SD_REQUEST_READ = 0x20
} sd_request_t;

// настройки - чтобы не заполнять полностью каждый раз
static SPI_InitTypeDef _sd_spi_params;
static DMA_InitTypeDef _sd_dma_params;

// Ожидание окончания работы SPI модуля
inline static void _spi_wait(void)
{
   bool notYet = true;
   while (notYet)
   {
      __disable_irq();
      if (SD_SPI->SR & SPI_I2S_FLAG_RXNE)
         notYet = false;
      __enable_irq();
   }
}


 static volatile TaskHandle_t _thisTaskHandle = NULL;


// Запуск DMA для обмена блоками данных с SD картой по SPI (считай асинхронная операция)
// Блоки фиксированного размера 512 байт. Буффер должен быть жив до окончания операции
inline static void _sd_dma_transfer(void * target, sd_request_t direction)
{
   // Чистим настройки DMA с прошлого раза FIXME: можно чистить не все а только конкретный флаг,
   // который мешает перезапустить
   DMA_DeInit(DMA1_Channel4);
   DMA_DeInit(DMA1_Channel5);

   if (SD_REQUEST_WRITE ==  direction)
   {
      // настраиваем DMA канал на RX. Принятые данные скидываем в фиксированную переменную
      // без перемещения каретки
      static uint16_t dummy_rx_buff;
      _sd_dma_params.DMA_MemoryBaseAddr = (uint32_t)&dummy_rx_buff;
      _sd_dma_params.DMA_DIR = DMA_DIR_PeripheralSRC;
      _sd_dma_params.DMA_MemoryInc = DMA_MemoryInc_Disable;
      DMA_Init(DMA1_Channel4, &_sd_dma_params);

      // Теперь на TX. Пишем из данного буфера с перемещением каретки
      _sd_dma_params.DMA_MemoryBaseAddr = (uint32_t)target;
      _sd_dma_params.DMA_DIR = DMA_DIR_PeripheralDST;
      _sd_dma_params.DMA_MemoryInc = DMA_MemoryInc_Enable;
      DMA_Init(DMA1_Channel5, &_sd_dma_params);
   }
   else // чтение
   {
      // настраиваем DMA канал на RX. Собираем данные в буффер
      _sd_dma_params.DMA_MemoryBaseAddr = (uint32_t)target;
      _sd_dma_params.DMA_DIR = DMA_DIR_PeripheralSRC;
      _sd_dma_params.DMA_MemoryInc = DMA_MemoryInc_Enable;
      DMA_Init(DMA1_Channel4, &_sd_dma_params);

      // Теперь на TX. Кормим муодуль 0xFFками
      static const uint16_t dummy_tx_buff = 0xFFFF;
      _sd_dma_params.DMA_MemoryBaseAddr = (uint32_t)&dummy_tx_buff;
      _sd_dma_params.DMA_DIR = DMA_DIR_PeripheralDST;
      _sd_dma_params.DMA_MemoryInc = DMA_MemoryInc_Disable;
      DMA_Init(DMA1_Channel5, &_sd_dma_params);
   }

   _thisTaskHandle = xTaskGetCurrentTaskHandle();

   // настройка завершена, но DMA еще не запущено
   // активируем само DMA
   DMA_Cmd(DMA1_Channel4, ENABLE);
   DMA_Cmd(DMA1_Channel5, ENABLE);

   DMA_ITConfig(DMA1_Channel4, DMA_IT_TC, ENABLE);
   NVIC_SetPriority(DMA1_Channel4_IRQn, configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
   NVIC_EnableIRQ(DMA1_Channel4_IRQn);

   // разрешаем SPI к нему обращаться
   SPI_I2S_DMACmd(SD_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

   // ждем завершения
   uint32_t dummy;
   xTaskNotifyWait(0xFFFFFFFF, 0x00000000, &dummy, portMAX_DELAY);

   // отключаем DMA. FIXME: Впринципе можно этого и не делать, но на всякий - пускай будет
   SPI_I2S_DMACmd(SD_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, DISABLE);

}

void DMA1_Channel4_IRQHandler();

void DMA1_Channel4_IRQHandler()
{
	BaseType_t woken = 0;

	if (SET == DMA_GetITStatus(DMA1_IT_TC4))
	{
		DMA_ClearITPendingBit(DMA1_IT_TC4);
		vTaskNotifyGiveFromISR(_thisTaskHandle, &woken);
		_thisTaskHandle = NULL;
		portYIELD_FROM_ISR(woken);
	}
	else
	{
		abort(); // чет какая-то фигня случилась
	}

	if (DMA_GetITStatus(DMA1_IT_TC5) != SET)
		abort();

}



// =================================================================

void sd_init(void)
{
   // активриуем тактирование необходимой перефириии
   RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
   RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

   // настраиваем пины
   {
      // CS - им рулим вручную
      GPIO_InitTypeDef pb_init;
      pb_init.GPIO_Mode = GPIO_Mode_Out_PP;
      pb_init.GPIO_Pin = GPIO_Pin_12;
      pb_init.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &pb_init);
   }
   {
      // SCK, MOSI
      GPIO_InitTypeDef pb_init;
      pb_init.GPIO_Mode = GPIO_Mode_AF_PP;
      pb_init.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
      pb_init.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &pb_init);
   }
   {
      // MISO
      GPIO_InitTypeDef pb_init;
      pb_init.GPIO_Mode = GPIO_Mode_IN_FLOATING; // GPIO_Mode_AIN почему-то не катит, я не правильно понимаю что оно значит?
      pb_init.GPIO_Pin = GPIO_Pin_14;
      pb_init.GPIO_Speed = GPIO_Speed_50MHz;
      GPIO_Init(GPIOB, &pb_init);
   }

   // настраиваем модуль SPI
   SPI_StructInit(&_sd_spi_params);
   _sd_spi_params.SPI_Mode = SPI_Mode_Master;
   _sd_spi_params.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
   _sd_spi_params.SPI_DataSize = SPI_DataSize_8b;
   _sd_spi_params.SPI_CPOL = SPI_CPOL_Low; // такие параметры требуются для sd карты
   _sd_spi_params.SPI_CPHA = SPI_CPHA_1Edge; // такие параметры требуются для sd карты
   _sd_spi_params.SPI_NSS = SPI_NSS_Soft;
   _sd_spi_params.SPI_BaudRatePrescaler = SD_BAUD_RATE_PRESCALER_FAST; // Сразу ставим быстрый, он будет меняться по ходу дела
   _sd_spi_params.SPI_FirstBit = SPI_FirstBit_MSB; // такие параметры требуются для sd карты
   _sd_spi_params.SPI_CRCPolynomial = 0x0007; // пока не используем, но при других значениях срабатывает ассерт
   SPI_Init(SD_SPI, &_sd_spi_params);
   SPI_Cmd(SD_SPI, ENABLE);
   SPI_NSSInternalSoftwareConfig(SD_SPI, SPI_NSSInternalSoft_Set);

   // общие настройки DMA, остальные будут меняться в зависимости от ситуации
   DMA_StructInit(&_sd_dma_params);
   _sd_dma_params.DMA_PeripheralBaseAddr = (uint32_t)&SD_SPI->DR;
   _sd_dma_params.DMA_PeripheralDataSize = DMA_MemoryDataSize_Byte;
   _sd_dma_params.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
   _sd_dma_params.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
   _sd_dma_params.DMA_BufferSize = 512;
   _sd_dma_params.DMA_Mode = DMA_Mode_Normal;
   _sd_dma_params.DMA_Priority = DMA_Priority_Low;
   _sd_dma_params.DMA_M2M = DMA_M2M_Disable;

   DMA_Cmd(DMA1_Channel4, ENABLE);
   DMA_Cmd(DMA1_Channel5, ENABLE);

   sd_cs(false);
}


void sd_cs(bool state)
{
   GPIO_WriteBit(GPIOB, GPIO_Pin_12, state ? RESET : SET);
}


void sd_exchange(const void * to_send, void * to_recieve, size_t amount)
{
   const uint8_t * to_send_ = (const uint8_t *)to_send;
   uint8_t * to_recieve_ = (uint8_t *)to_recieve;
   for (size_t i = 0; i < amount; i++)
   {
      SD_SPI->DR = to_send_[i];
      _spi_wait();
      to_recieve_[i] = (uint8_t)SD_SPI->DR;
   }
}


void sd_write(const void * to_send, size_t amount)
{
   const uint8_t * to_send_ = (const uint8_t *)to_send;
   for (size_t i = 0; i < amount; i++)
   {
      SD_SPI->DR = to_send_[i];
      _spi_wait();
      register const uint8_t dummy = (uint8_t)SD_SPI->DR;
      (void)dummy;
   }
}


void sd_read(void * to_recieve, size_t amount)
{
   register const uint8_t dummy = 0xFF;
   uint8_t * to_recieve_ = (uint8_t*)to_recieve;

   for (size_t i = 0; i < amount; i++)
   {
      SD_SPI->DR = dummy;
      _spi_wait();
      to_recieve_[i] = (uint8_t)SD_SPI->DR;
   }
}


sd_resp_t sd_response_type(sd_cmd_t cmd)
{
   switch (cmd)
   {
   case SD_CMD0:
   case SD_CMD1:
   case SD_CMD12:
   case SD_CMD17:
   case SD_CMD18:
   case SD_CMD24:
   case SD_CMD55:
   case SD_CMD41:
   case SD_CMD25:
      return SD_R1;
   case SD_CMD8:
      return SD_R7;
   default:
      abort();
      return 0; // не дойдем до сюда
   }
}


size_t sd_response_length(sd_resp_t resp)
{
   switch (resp)
   {
   case SD_R1:
      return 1;
   case SD_R3:
      return 4;
   case SD_R7:
      return 5;
   default:
      abort();
      return 0; // не дойдем до сюда, но компилятор ругается
   }
}


sd_error_t sd_cmd(sd_cmd_t cmd, uint32_t argument, void * response)
{
   const uint8_t * arg = (const uint8_t *)&argument;

   uint8_t data[] = { (uint8_t)((cmd & 0x3F) | 0x40), arg[3], arg[2], arg[1], arg[0], 0x95/*CRC*/};
   // 0x95 - фиксированное значение CRC для CMD0, для которой без CRC никак. Для остальных CRC мы не испольуем
   data[5] = crc7(data, 5);
   sd_write(data, sizeof(data));

   sd_resp_t resp_type = sd_response_type(cmd);
   size_t response_length = sd_response_length(resp_type);

   uint8_t * volatile first_response_byte_ptr = (uint8_t*)response;
   size_t timeleft = SD_CMD_TEAK_TIMEOUT;
   for (; timeleft != 0; timeleft--)
   {
      sd_read(first_response_byte_ptr, 1);
      // пока ответа нет - шина держится в единице (0xFF)
      // первый бит любого ответа должен быть равен 0
      if ((*first_response_byte_ptr & 0x80) == 0)
         break;
   }

   if (0 == timeleft)
      return SD_ERROR_TIMEOUT;

   // дочитываем остальное
   sd_read(first_response_byte_ptr+1, response_length-1);

   return SD_ERROR_NONE;
}


sd_error_t sd_wait_busy(void)
{
   //GPIOC->BRR |= (1 << 13);
   uint8_t buffer;
   do {
      sd_read(&buffer, 1);
   } while(0x00 == buffer);

   //GPIOC->BSRR |= (1 << 13);
   return SD_ERROR_NONE;
}


sd_error_t sd_startup(void)
{
   sd_error_t error = SD_ERROR_NONE;
   sd_card_type_t card_type;

   // Переключаем SPI на медленный режим
   _sd_spi_params.SPI_BaudRatePrescaler = SD_BAUD_RATE_PRESCALER_SLOW; // пока такой, можно будет и побыстрее
   SPI_Init(SD_SPI, &_sd_spi_params);

   // прокачиваем 20 SPI тактов на SD карту при выключенном CS и удерживаю на линии MOSI единицу (0xFF)
   sd_cs(false);
   uint8_t dummy[20];
   memset(dummy, 0xFF, sizeof(dummy));
   sd_write(dummy, sizeof(dummy));

   // Переключаем SPI на высокую скорость
   _sd_spi_params.SPI_BaudRatePrescaler = SD_BAUD_RATE_PRESCALER_FAST; // пока такой, можно будет и побыстрее
   SPI_Init(SD_SPI, &_sd_spi_params);

   sd_cs(true);
   // отправляем карте команду CMDO уже при включенном CS
   uint8_t r1_resp = 0x00;
   GOTO_END_IF_ERROR(sd_cmd(SD_CMD0, 0x00, &r1_resp));
   if (r1_resp & ~SD_R1_IDLE) // должен быть только IDLE бит, без других ошибок
   {
      error = SD_ERROR_WRONG_RESPONSE;
      goto end;
   }

   // отлично - карта в состоние IDLE

   // заставим её проверить напряжение питания
   // ошибки игнорируем, тк эту команду понимают только некоторые SD карты
   // заодно и выясним - какой тип карточки у нас
   uint8_t r7_resp[5];
   sd_cmd(SD_CMD8, 0x000001AA, r7_resp); // 0xAA - спец обязательный паттерн. 0x01 - команда на проверку питания
   if (r7_resp[0] & SD_R1_ILLEGAL_CMD) // карта не поняла команду - это SD1
      card_type = SD_TYPE_SD1;
   else if (0xAA == r7_resp[4]) // карта поняла нашу команду - это SD2 карта
      card_type = SD_TYPE_SD2;
   else // карта ведет себя неадекватно
   {
      error = SD_ERROR_WRONG_RESPONSE;
      goto end;
   }

   // долбим карту командами не включение, пока ей не надоест
   // если это SDHC, она ждет от нас ACMD41

   // для SD2 аргумент команды 41 должен быть 0X40000000, для остальных 0
   uint32_t cmd41_arg = (card_type == SD_TYPE_SD2) ? 0X40000000 : 0;
   do
   {
      GOTO_END_IF_ERROR(sd_cmd(SD_CMD55, 0x00, &r1_resp));
      if (r1_resp & ~SD_R1_IDLE) // должен быть только IDLE бит, без других ошибок
      {
         error = SD_ERROR_WRONG_RESPONSE;
         goto end;
      }

      GOTO_END_IF_ERROR(sd_cmd(SD_CMD41, cmd41_arg, &r1_resp));
      if (r1_resp & ~SD_R1_IDLE) // есть биты ошибки
      {
         if (SD_TYPE_SD2 == card_type) // SD V2 не позволительно тут давать ошибку - с ней что-то не то
         {
            error = SD_ERROR_WRONG_RESPONSE;
            goto end;
         }
         else
         {
            break; // остальные карты могут ошибаться - мы будем пробовать на них CMD1
         }
      }
   } while (r1_resp != 0x00);


   // если это карта, которая не понимает ACMD41 и idle бит все еще стоит - пробуем запустить её через CMD1
   while (r1_resp != 0x00)
   {
      GOTO_END_IF_ERROR(sd_cmd(SD_CMD1, 0x00, &r1_resp));
      if (r1_resp & ~SD_R1_IDLE) // есть биты ошибки
      {
         error = SD_ERROR_WRONG_RESPONSE;
         goto end;
      }
   }

   // все, карта готова
end:
   sd_cs(false);
   return error;
}

sd_error_t sd_block_write_multi(size_t offset, const void * block_start, size_t block_count)
{
   sd_error_t error = SD_ERROR_NONE;
   static const uint16_t dummy = 0xFFFF;
   // определяемся с командой и токенами - запись нескольких блоков или одного
   const uint8_t packet_token = block_count > 1 ? SD_TOKEN_MULTI_DATA : SD_TOKEN_SINGLE_DATA;
   const sd_cmd_t write_cmd = block_count > 1 ? SD_CMD25 : SD_CMD24;

   sd_cs(true);
   sd_wait_busy();

   // отправляем команду на запись
   uint8_t resp = 0x00;
   GOTO_END_IF_ERROR(sd_cmd(write_cmd, (uint32_t)offset, &resp));
   if (resp != 0x00)
   {
      error = SD_ERROR_WRONG_RESPONSE;
      goto end;
   }
   // теперь отправляем специальный "пропускающий" байт
   sd_write(&dummy, 1);

   for (size_t i = 0; i < block_count; i++)
   {
      if (i != 0)
         sd_wait_busy(); // ждем пока карта освободиться

      // заголовок пакета данных
      sd_write(&packet_token, 1);
      // сами данные
      _sd_dma_transfer((void*)((uint8_t*)block_start + 512*i), SD_REQUEST_WRITE);
      // контрольная сумма
      sd_write(&dummy, 2);

      // sd карта отвечает на пакет - слушаем
      sd_read(&resp, 1);
      resp &= SD_DATA_RESP_MASK;
      if (resp != SD_DATA_RESP_DATA_ACCEPTED)
      {
         error = SD_ERROR_WRONG_RESPONSE;
         break;
      }
   }

   if (block_count > 1)
   {
      sd_wait_busy();
      // заканчиваем - отправляем соответсвующий токен
      static const uint8_t stop_tran_token = SD_TOKEN_NO_MORE_DATA;
      sd_write(&stop_tran_token, 1);
      // нужно прогнать еще один пропускающий байт, чтобы карта вошла в состояние busy
      // чтобы начать запись на самой sd карте
      // нужно отправить еще хотябы один байт по SPI в состоянии busy
      // (согласно статье http://elm-chan.org/docs/mmc/mmc_e.html)
      // поэтому отправляем сразу два байта
      sd_write(&dummy, 2);
   }

   //sd_wait_busy();

end:
   sd_cs(false);
   return error;
}


sd_error_t sd_block_read_multi(size_t offset, void * block, size_t block_count)
{
   sd_error_t error = SD_ERROR_NONE;

   // определяемся с командой на чтение в зависимости от количества блоков
   sd_cmd_t read_cmd = block_count > 1 ? SD_CMD18 : SD_CMD17;

   sd_cs(true);
   sd_wait_busy();

   // Подготовка к чтению блока - отправка команды и заголовков
   uint8_t resp_r1 = 0;
   GOTO_END_IF_ERROR(sd_cmd(read_cmd, (uint32_t)offset, &resp_r1));
   if (resp_r1 != 0x00)
   {
      error = SD_ERROR_WRONG_RESPONSE;
      goto end;
   }

   for (size_t i = 0; i < block_count; i++)
   {
      // ждем токена о начале передачи
      uint8_t token;
      do
      {
         sd_read(&token, 1);
      } while (token == 0xFF);

      // проверяем токен
      if (token != SD_TOKEN_SINGLE_DATA)
      {
         error = SD_ERROR_INVALID_DATA_TOKEN;
         break;
      }

      // читаем сам блок
      _sd_dma_transfer((uint8_t*)block + 512*i, SD_REQUEST_READ);

      // читаем CRC
      uint16_t crc;
      sd_read(&crc, 2);

   }

   if (block_count > 1)
   {
      // горшочек - не вари. Засылаем спеиальную команду на остановку
      sd_cmd(SD_CMD12, 0x00, &resp_r1);
      if (resp_r1 != 0x00)
      {
         error = SD_ERROR_WRONG_RESPONSE;
         goto end;
      }
   }

   // карта вошла в состояние busy.
   // когда блоки пишуются, нужно отправить хотябы один байт, чтобы запись реально пошла
   // может быть с чтением такая же ерунда?
   uint8_t dummy;
   sd_read(&dummy, 1);
   //sd_wait_busy();

end:
   sd_cs(false);
   return error;
}



