#include "stm32f10x.h"

void AKM_ENCODER_Init(uint16_t cycle) {
    // GPIOB时钟 以及复用功能
    RCC->APB2ENR |= (RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO);
    // TIM3 时钟使能
    RCC->APB1ENR |= RCC_APB1Periph_TIM3;

    uint32_t tmpreg = GPIOB->CRL;
    tmpreg &= 0xFF00FFFF;
    tmpreg |= 0x00440000; // PB4\5 浮空输入模式
    GPIOB->CRL = tmpreg;

    // 引脚功能重映射
    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3 , ENABLE);
    // 
    RCC->APB1RSTR |= RCC_APB1Periph_TIM3;
    RCC->APB1RSTR &=~RCC_APB1Periph_TIM3;

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_ICInitTypeDef TIM_ICInitStructure;

    TIM_TimeBaseStructure.TIM_Prescaler = 0x0;  // No prescaling 
    TIM_TimeBaseStructure.TIM_Period = cycle;  
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;   
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    //Reset counter
    TIM3->CNT = 0;
    TIM_Cmd(TIM3, ENABLE);

    /**/
    GPIO_InitTypeDef GPIO_InitStructure; 
    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    TIM_DeInit(TIM4);

    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 6;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    TIM4->CNT = 0;

    TIM_Cmd(TIM4, ENABLE); 
}

uint16_t AKM_ENCODE_A_GetCount(void){
    return (TIM_GetCounter(TIM3));
}

uint16_t AKM_ENCODE_B_GetCount(void){
    return (TIM_GetCounter(TIM4)); 
}

void AKM_ENCODE_A_SetCounter(uint16_t count)
{
    TIM3->CNT = count;
}

void AKM_ENCODE_B_SetCounter(uint16_t count)
{
    TIM4->CNT = count;
}


