#ifndef __STORAGE__H__
#define __STORAGE__H__

/* ============================================= */
// Public API
/* ============================================= */

typedef enum {StorageNoError, StorageNoDevice, StorageWriteProtect, StorageNoFreeSpace, StorageNotFound, StorageInternalError} StorageStatus;

// ������������� ���������
StorageStatus InitStorage(void);

// ���������� ���� � ���������
StorageStatus Save(IRCode *code);

// ������ �� ���������
StorageStatus Open(const uint32_t id, IRCode *resutl);

/* ============================================= */
// Debug fucntions
/* ============================================= */

// ����������� ������� ����������
void PrintStorageStatus(void);

// ����������� ����������� ���������
void PrintConentStorage(void);


#endif //__STORAGE__H__
