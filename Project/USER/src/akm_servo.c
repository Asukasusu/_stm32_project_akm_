#include "stm32f10x.h"

void AKM_Servo_Init(void)
{
    //   打开GPIOA、B时钟 复用功能时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO,ENABLE);
    //   打开TIM2时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    //   TIM2 输出引脚重定向至 PA15 - PB3
    GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);

    GPIO_InitTypeDef         GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef        TIM_OCInitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;  //CH2
    //   引脚复用功能
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    //   引脚反转速度
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15; //CH1
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //   50Hz 
    //   预分频器(TIMx_PSC)16位 定时器72分频 工作频率为1MHz 
    TIM_TimeBaseStructure.TIM_Prescaler = 71;
    //   自动重装载寄存器(TIMx_ARR)16位 
    TIM_TimeBaseStructure.TIM_Period = 19999;

    /*  控制寄存器 1(TIMx_CR1) 时钟分频 计数模式  */
    //   采样频率一分频
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    //   向上计数模式
    //   计数器从0计数到自动加载值TIMx_ARR 
    //   然后重新从0开始->计数器溢出->一般产生事件更新(未使用重复计数功能)
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

    //   向上计数时 PWM1: 计数器值(TIMx_CNT)<(TIMx_CCR1)时有效电平，否则无效
    //              PWM2: 计数器值(TIMx_CNT)<(TIMx_CCR1)时无效电平，否则有效
    //   向下计数时 有效性相反
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    //   比较输出使能        CC2E = 1
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 1499;  // 舵机 90度
    //   设置有效电平为高电平 CC2P = 0
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIM2, &TIM_OCInitStructure);// 通道一
    // 改变 捕获/比较寄存器 1(TIMx_CCR1)值时{disable:立即改变 enable:下次改变}
    // 避免PWM占空比紊乱
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);

    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
    // 改变 自动重装载寄存器(TIMx_ARR)值时{disable:立即改变 enable:下次改变}
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    // CEN位(0位) 使能使用计数器
    TIM_Cmd(TIM2, ENABLE);
}


/**
  * 简  述  舵机控制
  * 参  数  angle 舵机角度 范围：0~180度 50Hz
            0.5ms-(-90°)  1ms-(-45°)  1.5ms-(0°)  2ms-(45°)  2.5ms-(90°)
            500           1000        1500        2000       2500
  * 返回值  无
  */
void AKM_SERVO_SetAngle(uint16_t angle){
    if (angle > 1800)  angle = 1800;

    angle = 1.111f*angle + 500;
    // 将angle值写入捕获/比较寄存器 1(TIMx_CCR1) 下次事件更新生效
    TIM_SetCompare1(TIM2, angle);
}


