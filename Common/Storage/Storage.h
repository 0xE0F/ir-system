#ifndef __STORAGE__H__
#define __STORAGE__H__

/* ������������� ��������� */
bool InitStorage(void);

/* ���������� ���� � ��������� */
bool Save(IRCode *code, const uint16_t number);

/* ������ �� ��������� */
bool Open(IRCode *code, const uint16_t number);

/** �������� ��������� */
void EraseStorage(void);

/** ��� ������� ��������� ������ ��� ������������ ������ */
typedef void (*action_on_enum_t) (char *path, char *fname);

/** ������������ ������ */
FRESULT EnumerateFiles ( char* path, action_on_enum_t func);

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
