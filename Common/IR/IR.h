/*
 * ����� ����������� ��� ������������ �����
 */
#ifndef __IR__H__
#define __IR__H__

#define STORAGE_PAGE_SIZE 2048
#define INTERVAL_SIZE (4)
#define IR_CODE_HEADER_SIZE (4 * 4)

#define INTERVALS_MAX ((STORAGE_PAGE_SIZE - IR_CODE_HEADER_SIZE) / INTERVAL_SIZE) // ������������ ���������� ����������

#pragma pack(1)

// �������� ����
typedef struct
{
	uint32_t ID;
	uint32_t Flags;
	uint32_t Frequency;
	uint32_t IntervalsCount;
	uint32_t Intervals[INTERVALS_MAX];	// ������� - ������� ��� - �������� ������, ��������� - ����� � ���
} IRCode;

#pragma pack(0)

#if (INTERVALS_MAX*INTERVAL_SIZE + IR_CODE_HEADER_SIZE) > STORAGE_PAGE_SIZE
#error Size of IRCode NOT EQUAL STORAGE_PAGE_SIZE
#endif

enum { InvalidFrequencyValue = 0 };

// ���������� ������������� �������
extern const uint32_t DetectFreqMin;
// ����������� ������������� �������
extern const uint32_t DetectFreqMax;

// ��������� ����������� ������� �� ��������� 30 ��� - 60 ���
uint32_t GetFrequencyInterval(uint32_t value);

//��������� �������� �������
void SetTime(uint32_t *interval, const uint32_t time);

// ��������� �������� ���������
void SetValue(uint32_t *interval, const uint8_t value);

// ��������� �������� ������� ���������
uint32_t GetTime(uint32_t interval);

// ��������� �������� ���������
uint8_t GetValue(uint32_t interval);

// ��������� ����� ����
void DebugPrint(IRCode *code);

typedef enum  { CS_OK, CS_ERROR, CS_WARNING} CheckStatus;
// �������� ����
CheckStatus CheckIRCode(IRCode *code);

// ��� - ��������� ��������� �����
typedef enum {CodesEqual = 0, FirstEmpty, SecondEmpty, LengthNotEqual, CodesValueNotEqual, CodesIntervalNotEqual, FreqNotEqual} CompareCodesResult;

// ��������� �����
CompareCodesResult CompareIrCode(IRCode *left, IRCode *rigth);
#endif // __IR__H__
