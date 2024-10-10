#include "stm32f10x.h"

void AKM_MOTOR_Init(uint8_t freq){
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA|
    RCC_APB2Periph_AFIO|RCC_APB2Periph_GPIOC, ENABLE);

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed= GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Period = 1999;
    TIM_TimeBaseStructure.TIM_Prescaler = 36/freq - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);

    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;
    
    TIM_OC1Init(TIM5, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM5, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM5, TIM_OCPreload_Enable);

    // TIM_OC3Init(TIM5, TIM_OCInitStructure);
    // TIM_OC3PreloadConfig(TIM5, TIM_OCPreload_Enable);

    // TIM_OC4Init(TIM5, TIM_OCInitStructure);
    // TIM_OC4PreloadConfig(TIM5, TIM_OCPreload_Enable);

    TIM_ARRPreloadConfig(TIM5, ENABLE);
    TIM_Cmd(TIM5, ENABLE);
    // TIM_CtrlPWMOutputs(TIM5, ENABLE);//TIM1 和 TIM8使用

}

void AKM_MOTOR_A_SpeedSet(int16_t speed){
    uint16_t temp = 0;
    if (speed > 0) {
        // GPIOC->BRR |= GPIO_Pin_3;  //置零
        // GPIOC->BSRR|= GPIO_Pin_2;  //置一
        GPIOC->BSRR = (uint32_t)(GPIO_Pin_3<<16 | GPIO_Pin_2);
        temp = speed;
    }
    else if (speed < 0) {
        // GPIOC->BSRR|= GPIO_Pin_3;
        // GPIOC->BRR |= GPIO_Pin_2;
        GPIOC->BSRR = (uint32_t)(GPIO_Pin_2<<16 | GPIO_Pin_3);
        temp = -speed;
    }
    else{
        GPIOC->BSRR = (uint32_t)((GPIO_Pin_3|GPIO_Pin_2)<<16);
        temp = 0;
    }
    temp = temp > 2000 ? 2000 : temp;
    TIM5->CCR2 = temp;
}

void AKM_MOTOR_B_SpeedSet(int16_t speed){
    uint16_t temp = 0;
    if (speed > 0) {
        GPIOC->BSRR = (uint32_t)(GPIO_Pin_1<<16 | GPIO_Pin_0);
        temp = speed;
    }
    else if (speed < 0) {
        GPIOC->BSRR = (uint32_t)(GPIO_Pin_0<<16 | GPIO_Pin_1);
        temp = -speed;
    }
    else{
        GPIOC->BSRR = (uint32_t)((GPIO_Pin_0|GPIO_Pin_1)<<16);
        temp = 0;
    }
    temp = temp > 2000 ? 2000 : temp;
    TIM5->CCR1 = temp;
}


