#include "stm32f10x.h"
#include "taskloop.h"

void AKM_TIME_Init(uint16_t cnt_us) {
    // APB1 外设时钟使能寄存器(RCC_APB1ENR)
    // (4)TIM6EN TIM6时钟开启
    RCC->APB1ENR |= RCC_APB1Periph_TIM6;

    // 自动重装载值
    TIM6->ARR = cnt_us - 1;
    // 定时器分频
    TIM6->PSC = 72 - 1;

    // 清除定时器更新中断标志位
    TIM6->SR  = (uint16_t)(~TIM_FLAG_Update);

    // 中断使能
    TIM6->DIER|= TIM_IT_Update;

    // 使能TIM6
    TIM6->CR1 |= TIM_CR1_CEN;

    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}


void TIM6_IRQHandler(void){

    if (TIM_GetITStatus( TIM6, TIM_IT_Update) != RESET) {
        // LED_Gre_Tgl();
        sysTickUpTimes++;
        TIM_ClearITPendingBit(TIM6 , TIM_IT_Update);
    }
}
