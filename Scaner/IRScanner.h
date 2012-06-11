/*
 * ����������� ��� ������������ ����� �������
 */
#ifndef __IR__SCANNER__H__
#define __IR__SCANNER__H__

/* ������������ ���� */
void Scan(IRCode *irCode);


/* ����������� �������� ������������ */
void StopScan(void);

// ������������ ������������
volatile uint32_t IsScanning();


#endif // __IR__SCANNER__H__
