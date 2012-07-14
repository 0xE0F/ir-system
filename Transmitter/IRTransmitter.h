/*
 * ����������� ��� ������������ ����� �����������
 */
#ifndef __IR__TRANSMITTER__H__
#define __IR__TRANSMITTER__H__

// ������������ ����� ������
extern const uint32_t MaxChannelNumber;

typedef enum {StatusCode_Ok = 0, StatusCode_NullArgumentReference = 1, StatusCode_ArgumentOutOfRange = 2, StatusCode_Busy = 3, StatusCode_InternalError = 4} StatusCode;

// �������� �������� ��������� �����������
volatile bool IsIrTransmitting();

// ������ ���� � ��������������� �����
StatusCode SendCodeToChannel(IRCode *code, uint32_t channelID);

// ��������� �������� �� �������� ������
void SetOutValueToChannel(const uint32_t channelId, const uint8_t value);

// ��������� ������� �������
void SetCarrierFrequency(const uint32_t value);

#endif // __IR__TRANSMITTER__H__
