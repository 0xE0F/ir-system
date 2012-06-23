#ifndef __STORAGE__H__
#define __STORAGE__H__

/* ������������� ��������� */
bool InitStorage(void);

/* ���������� ���� � ��������� */
bool Save(IRCode *code);

/* ������ �� ��������� */
bool Open(const uint32_t id, IRCode *result);

/* ============================================= */
// Debug fucntions
/* ============================================= */

// ����������� ������� ����������
void PrintStorageStatus(void);

// ����������� ����������� ���������
void PrintConentStorage(void);

/* ��������� ����������� ������ ��������� */
void SetStorageDebugMode(bool mode);


#endif //__STORAGE__H__
