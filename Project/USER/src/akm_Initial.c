#include "akm_Initial.h"

static void SysTick_Init(void)
{
    //  延时函数系统时钟配置
    //  时钟选为8分频 72Mhz/8 = 9Mhz 精度高
#if (SysTick_MODE == SysTick_MODE_Div8)
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

    //  72Mhz
#elif (SysTick_MODE == SysTick_MODE_Div)
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

#endif
}

static void AKM_LED_Init(void)
{
    // 打开APB2上的GPIOA和GPIOD的时钟信号
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //((uint16_t)1u<<2)
    // 推挽输出稳定输出高低电平
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //((uint16_t)1u<<8)
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*
       BSRR(只写)寄存器Set比Reset的级别高 写0无动作
       低16位(0~15) 控制端口输出高电平
       高16位(16~31)控制端口输出低电平

       BRR (只写)寄存器
       低16位(0~15) 写1控制端口输出低电平 写0无动作
       高16位(16~31)保留

       ODR (读写)寄存器 写1端口高电平，0为低电平
       低16位(15~0) 引脚输出数据
       高16位(31~16)保留
    */
    GPIOD->BSRR = (uint16_t)(1u << 2);
    GPIOA->BSRR = (uint16_t)(1u << 8);
}

// void LED_Gre_Tgl(void)   
// {
//     if((GPIOA->IDR & GPIO_Pin_8)!=(uint32_t)Bit_RESET){GPIOA->BRR  = GPIO_Pin_8;} else{GPIOA->BSRR = GPIO_Pin_8;};
// }

static void AKM_BEEP_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode= GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;

    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIOC->BRR = (uint16_t)(1u << 8);
}


/**
 * @简  述   软件毫秒级延时函数
 * @参  数   ms: 延时长度 单位ms
 * @返回值   无
 * @说  明   SysTick->LOAD为24位寄存器  8分频下
 *           最大延时时间nTime <= 0xFFFFFF*8000/SYSCLK
 *           在72MHZ下 ms <= 1864ms
 */
void SysTick_Delay_ms(uint16_t ms)
{

    uint32_t temp = 0x00000000;

    // 重装载值寄存器
    SysTick->LOAD = (uint32_t)(9000 * ms);
    // 递减计数器置零
    SysTick->VAL = 0x000000;
    // SysTick定时器使能
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    do
    {
        temp = SysTick->CTRL;
        // SysTick定时器使能 且 COUNTFLAG位置一
    } while ((temp & 0x01) && !(temp & (1 << 16)));
    // SysTick定时器使能标志位置零
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    // 清空计数器
    SysTick->VAL = 0x000000;
}

void SysTick_Delay_us(uint16_t us)
{

    uint32_t temp = 0x00000000;

    // 重装载值寄存器
    SysTick->LOAD = (uint32_t)(9 * us);
    // 递减计数器置零
    SysTick->VAL = 0x000000;
    // SysTick定时器使能
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    do
    {
        temp = SysTick->CTRL;
        // SysTick定时器使能 且 COUNTFLAG位置一
    } while ((temp & 0x01) && !(temp & (1 << 16)));
    // SysTick定时器使能标志位置零
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    // 清空计数器
    SysTick->VAL = 0x000000;
}

/**
  * @简  述  JTAG模式设置
  * @参  数  mode:jtag,swd模式设置;00,全是能;01,使能SWD;10,全关闭;  
              JTAG_SWD_DISABLE   0X02
              SWD_ENABLE         0X01
              JTAG_SWD_ENABLE    0X00	
  * @返回值  无
  */
static void AX_JTAG_Set(uint8_t mode)
{
    uint32_t temp;

    temp=mode;
    temp<<=25;
    RCC->APB2ENR|=1<<0;     //开启辅助时钟
    AFIO->MAPR&=0XF8FFFFFF; //清楚MARR的[26:24]
    AFIO->MAPR|=temp;       //设置jtag模式
} 

void System_init(void)
{
    uint8_t state;
    // SystemInit();         // 配置系统频率72Mhz

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    AX_JTAG_Set(0x02);
    AX_JTAG_Set(0x01);    // 禁止JTAG功能，把PB3，PB4作为普通IO口使用 ！！！
    SysTick_Init();       // 系统滴答延时
    AKM_Servo_Init();     // 舵机
    AKM_LED_Init();       // LED灯初始化
    AKM_BEEP_Init();      // 蜂鸣器
    AKM_Volt_Init();      // 电压采集初始化

    AKM_MOTOR_Init(10);   // 电机初始化
    

    AKM_UART_PI_Init();   // 与树莓派通信串口
    AKM_UART_PC_Init();   // 调试串口
    AKM_TIME_Init(1000);  // 定时器中断1ms

    Vofa_Init(UART1SendByte);
                          // 编码器初始
    AKM_ENCODER_Init(ENCODER_MID_VALUE<<1);
    AKM_ENCODE_A_SetCounter(ENCODER_MID_VALUE);
    AKM_ENCODE_B_SetCounter(ENCODER_MID_VALUE);
    state = MPU6050_Init();
    MPU6050_DMP_Init();
    if(state == SET) {
        BEEP_On();
        while(1);
        // SysTick_Delay_ms(500);
        // BEEP_Off();
    }
    LED_Gre_On();
    BEEP_On();
    SysTick_Delay_ms(1000);
    BEEP_Off();
    LED_Gre_Off();
}
    // 小端存储方式
    // float a = 1.5;
    // uint8_t *p = (uint8_t *)(&a);
    // if (*p == 0x00) {
    //     LED_Red_On();
    // }
    // else {};
