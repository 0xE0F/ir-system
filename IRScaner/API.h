/*
 *	������������ ���� � �������� ����������������.
 */
#ifndef _IRDA_API_H_
#define _IRDA_API_H_

#define STORAGE_PAGE_SIZE 512

// ���� ��������
#pragma pack(1)
typedef struct
{
	uint16_t Time;
	uint8_t  Value;
} OnePulse;

#define MaxPulses 166 //((512 - 4 * 3) / sizeof(OnePulse))

// �������� ����
typedef struct
{
	uint32_t ID;
	uint32_t Frequency;
	uint32_t PulsesCount;
	OnePulse Pulses[MaxPulses];
	uint16_t Flags;
} IRCode;
#pragma pack(0)

#if (MaxPulses*3 + 3*4+2) != STORAGE_PAGE_SIZE
#error Size of IRCode NOT EQUAL STORAGE_PAGE_SIZE
#endif


// ������ ���� �� �������
uint32_t StartRecord(IRCode *irCode);
// ��������� ������
uint32_t StopRecord();

// ������ ���� �� ��������� ���������
uint32_t ReadCode(IRCode *code, uint32_t id);
// ������ ���� � ���������� ��������
uint32_t WriteCode(IRCode *code, uint32_t id);

// ������ ���� � ��������������� �����
uint32_t WriteToChannel(IRCode *code, uint32_t channelID);

// ��������� ����� ����
void DebugPrint(IRCode *code);

// ��������� ����������� ������� �� ��������� 30 ��� - 60 ���
// 0 - ���� ������� ��� ���������
uint32_t GetFrequencyInterval(uint32_t value);

// ��������� ������� �������
void SetCarrierFrequency(const uint32_t value);

#endif // _IRDA_API_H_
