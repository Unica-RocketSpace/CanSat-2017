/*
 * sd_spi.h
 *
 *  Created on: 14 июля 2016 г.
 *      Author: snork
 */

#ifndef SD_SPI_H_
#define SD_SPI_H_

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>

#include <sys/types.h>

//! Ошибки модуля
typedef enum
{
   SD_ERROR_NONE = 0,
   SD_ERROR_TIMEOUT = 1,
   SD_ERROR_WRONG_RESPONSE = 2,
   SD_ERROR_INVALID_DATA_TOKEN = 3,
   SD_ERROR_WOULD_BLOCK = 0x100,
} sd_error_t;

//! типы sd карт
typedef enum
{
   SD_TYPE_SD1,
   SD_TYPE_SD2,
   SD_TYPE_SDHC,
} sd_card_type_t;

//! Команды SD карты
typedef enum
{
   SD_CMD0 = 0,   // сброс
   SD_CMD1 = 1,   // инициализация
   SD_CMD8 = 8,   // проверка напряжения SD картой
   SD_CMD12 = 12, // остановка многоблочного чтения
   SD_CMD17 = 17, // чтение блока
   SD_CMD18 = 18, // чтение множества блоков
   SD_CMD55 = 55, // преамбула для ACMD команды
   SD_CMD41 = 41, // команда на включение SDHC карты
   SD_CMD24 = 24, // запись блока
   SD_CMD25 = 25, // запись множества блоков
} sd_cmd_t;

//! Типы ответов SD карты
typedef enum
{
   SD_R1,
   SD_R3,
   SD_R7,
} sd_resp_t;

//! Инициализация модуля работы с SD картой (не её самой, только настройка SPI и прочего)
void sd_init(void);

// =========================================================
// Транспортный уровень
// =========================================================
//! Управление линией CS SD карты. True - активна. False - пассивна
void sd_cs(bool state);
//! Обмен с SD картой по SPI, принятые данные игнорируются.
void sd_write(const void * to_send, size_t amount);
//! Обмен с SD картой по SPI. Одновременно на чтение и запись
void sd_exchange(const void * to_send, void * to_recieve, size_t amount);
//! Обмен с SD картой по SPI только на прием. В качестве передаваемых данных передается стабильное 0xFF
void sd_read(void * to_recieve, size_t amount);

// =========================================================
// Уровень команд
// =========================================================
//! тип ответа SD карты в зависимости от типа команды
sd_resp_t sd_response_type(sd_cmd_t cmd);
//! длинна ответа в зависимости от его типа
size_t sd_response_length(sd_resp_t resp);

//! Команда SD карте. Аргумент 4 байта, зависит от команды. Размер ответа зависит от типа команды
/*! По типам ответа к командам - смотри функции sd_response_type и sd_response_length */
sd_error_t sd_cmd(sd_cmd_t cmd, uint32_t argument, void * response);

//! Ожидание снятия флага занятости SD карты
sd_error_t sd_wait_busy(void);

// =========================================================
// Уровень операций
// =========================================================
//! Запуска и нициализация самой SD карты
sd_error_t sd_startup(void);

//! Запись блока 512 байт на SD карту. Очевидно нельзя использовать во время асинхронных операций.
sd_error_t sd_block_write_multi(size_t offset, const void * block_start, size_t block_count);
//! Чтение блока 512 байт на SD карту. Очевидно нельзя использовать во время асинхронных операций.
sd_error_t sd_block_read_multi(size_t offset, void * block_start, size_t block_count);


#endif /* SD_SPI_H_ */
