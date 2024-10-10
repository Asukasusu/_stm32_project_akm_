#ifndef __AKM_UART_PI_H__
#define __AKM_UART_PI_H__

#include <stdint.h>

#define  BUF_MAX_LEN  (20u) // 不超过255
#define  FRAMLEN_MIN  (6)    // 最小帧长度

typedef struct
{
    uint8_t  head;
    uint8_t  tail;
    uint8_t  count;
    uint8_t  buffer_t[BUF_MAX_LEN];
    uint8_t  buffer[BUF_MAX_LEN];
}USART_Stru;

typedef void (*uart_func)(USART_Stru*);

#pragma pack(push, 4)
typedef struct
{
    uart_func p_fun;
    int16_t *addres;
    // uint16_t last_ticks;
}pfunc;       // 8Byte
#pragma pack(pop)

// extern USART_Stru usart2;

void    AKM_UART_PI_Init(void);
uint8_t USART_Receive_Analysis(USART_Stru* pusart);
void    USART_Transmiss_Datagroups(void);
void UART_ReceDataPack(void);
#endif
