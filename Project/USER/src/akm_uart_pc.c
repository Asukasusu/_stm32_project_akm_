#include "stm32f10x.h"

static uint8_t uart1_tx_buf[60];

void AKM_UART_PC_Init(void){
    GPIO_InitTypeDef GPIO_InitStructure;
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 115200;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_ClearFlag(USART1, USART_FLAG_RXNE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);

    USART_Cmd(USART1, ENABLE);
}



/*
 * @简  述  发送数据协议 
 * @参  数  *buf:   数据
 *           len:   数据长度
 *           num:   帧码
 * @返回值	 无
 */
void AKM_UART_PC_SendData(uint8_t *buf, uint8_t len, uint8_t num) {
    uint8_t i, cnt;
    uint8_t tx_checksum = 0;

    if (len > 50)  return;

    *(uart1_tx_buf+0) = 0xAA;
    *(uart1_tx_buf+1) = 0x55;
    *(uart1_tx_buf+2) = len+5; // 双帧头 帧长 帧码  校验
    *(uart1_tx_buf+3) = num;

    for(i = 0; i < len; i++) {
        *(uart1_tx_buf+4+i) = *(buf+i);
    }
    cnt = 4 + len; // 计算校验和
    for(i = 0; i < cnt; i++) {
        tx_checksum += *(uart1_tx_buf+i);
    }
    *(uart1_tx_buf+(cnt++)) = tx_checksum;

    // 发送数据
    for(i = 0; i < cnt; i++) {
        USART1->DR = (*(uart1_tx_buf+i) & (uint16_t)0x01FF);
        // 传输成功标志位 状态寄存器(USART_SR)(6)TC发送完成标志位
        while(!(USART1->SR & USART_FLAG_TC));
    }
}


/*
 *  USART1发送一个字节
 */
void UART1SendByte(uint8_t data) {
     USART1->DR = (data & (uint16_t)0x01FF);
     while(!(USART1->SR & USART_FLAG_TC));
}
