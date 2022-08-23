#ifndef _INIT_H
#define _INIT_H

#define DATA_FLASH_BASE     0x7E00
#define MODE_ID_OFFSET      0
/* CONFIG0: Enable data flash. */
#define CONFIG0             0xFDFFF97E
/* CONFIG1: Set Data Flash Base Address to 0x7E00 (size 512 bytes) */
#define CONFIG1             DATA_FLASH_BASE

void SYS_Init(void);
void CAN_Init(void);
void PWM0_Init(void);
void ADC_Init(void);
void TIMER0_Init(void);
void FMC_Init(void);

#endif