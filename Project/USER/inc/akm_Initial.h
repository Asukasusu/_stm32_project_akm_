#ifndef __AKM_INITIAL_H__
#define __AKM_INITIAL_H__

#include "headfile.h"

#define  SysTick_MODE_Div8    (1u)
#define  SysTick_MODE_Div     (0)
#define  SysTick_MODE         SysTick_MODE_Div8


#define LED_Red_Off()   GPIOD->BSRR = GPIO_Pin_2
#define LED_Red_On()    GPIOD->BRR  = GPIO_Pin_2
#define LED_Red_Tgl()   if((GPIOD->IDR & GPIO_Pin_2)!=(uint32_t)Bit_RESET){GPIOD->BSRR = GPIO_Pin_2;} else{GPIOD->BRR  = GPIO_Pin_2;}

#define LED_Gre_Off()   GPIOA->BSRR = GPIO_Pin_8
#define LED_Gre_On()    GPIOA->BRR  = GPIO_Pin_8
#define LED_Gre_Tgl()   if((GPIOA->IDR & GPIO_Pin_8)!=(uint32_t)Bit_RESET){GPIOA->BRR  = GPIO_Pin_8;} else{GPIOA->BSRR = GPIO_Pin_8;}

#define BEEP_Off()      GPIOC->BRR  = GPIO_Pin_8;
#define BEEP_On()       GPIOC->BSRR = GPIO_Pin_8;
#define BEEP_Tgl()      if((GPIOC->IDR & GPIO_Pin_8)==(uint32_t)Bit_RESET){GPIOC->BSRR = GPIO_Pin_8;} else{GPIOC->BRR  = GPIO_Pin_8;}


void System_init(void);
void SysTick_Delay_us(uint16_t us);
void SysTick_Delay_ms(uint16_t us);

#endif
