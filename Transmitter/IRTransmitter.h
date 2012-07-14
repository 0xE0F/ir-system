/*
 * Определения для инфракрасной части передатчика
 */
#ifndef __IR__TRANSMITTER__H__
#define __IR__TRANSMITTER__H__

// Максимальный номер канала
extern const uint32_t MaxChannelNumber;

typedef enum {StatusCode_Ok = 0, StatusCode_NullArgumentReference = 1, StatusCode_ArgumentOutOfRange = 2, StatusCode_Busy = 3, StatusCode_InternalError = 4} StatusCode;

// Проверка текущего состояния передатчика
volatile bool IsIrTransmitting();

// Выдача кода в соответствующий канал
StatusCode SendCodeToChannel(IRCode *code, uint32_t channelID);

// Установка хначения вы выходном канале
void SetOutValueToChannel(const uint32_t channelId, const uint8_t value);

// Установка несущей частоты
void SetCarrierFrequency(const uint32_t value);

#endif // __IR__TRANSMITTER__H__
