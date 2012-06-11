/*
 * Определения для инфракрасной части сканера
 */
#ifndef __IR__SCANNER__H__
#define __IR__SCANNER__H__

/* Сканирование кода */
void Scan(IRCode *irCode);


/* Прекращение процесса сканирования */
void StopScan(void);

// Производится сканирование
volatile uint32_t IsScanning();


#endif // __IR__SCANNER__H__
