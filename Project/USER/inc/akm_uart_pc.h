#ifndef __AKM_USART_PC_H__
#define __AKM_USART_PC_H__

#include <stdint.h>

void AKM_UART_PC_Init(void);
void AKM_UART_PC_SendData(uint8_t *buf, uint8_t len, uint8_t num);
void UART1SendByte(uint8_t data);

#endif

