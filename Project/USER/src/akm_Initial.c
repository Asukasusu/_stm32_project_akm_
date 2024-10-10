#include "akm_Initial.h"

static void SysTick_Init(void)
{
    //  ��ʱ����ϵͳʱ������
    //  ʱ��ѡΪ8��Ƶ 72Mhz/8 = 9Mhz ���ȸ�
#if (SysTick_MODE == SysTick_MODE_Div8)
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);

    //  72Mhz
#elif (SysTick_MODE == SysTick_MODE_Div)
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);

#endif
}

static void AKM_LED_Init(void)
{
    // ��APB2�ϵ�GPIOA��GPIOD��ʱ���ź�
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //((uint16_t)1u<<2)
    // ��������ȶ�����ߵ͵�ƽ
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8; //((uint16_t)1u<<8)
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /*
       BSRR(ֻд)�Ĵ���Set��Reset�ļ���� д0�޶���
       ��16λ(0~15) ���ƶ˿�����ߵ�ƽ
       ��16λ(16~31)���ƶ˿�����͵�ƽ

       BRR (ֻд)�Ĵ���
       ��16λ(0~15) д1���ƶ˿�����͵�ƽ д0�޶���
       ��16λ(16~31)����

       ODR (��д)�Ĵ��� д1�˿ڸߵ�ƽ��0Ϊ�͵�ƽ
       ��16λ(15~0) �����������
       ��16λ(31~16)����
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
 * @��  ��   ������뼶��ʱ����
 * @��  ��   ms: ��ʱ���� ��λms
 * @����ֵ   ��
 * @˵  ��   SysTick->LOADΪ24λ�Ĵ���  8��Ƶ��
 *           �����ʱʱ��nTime <= 0xFFFFFF*8000/SYSCLK
 *           ��72MHZ�� ms <= 1864ms
 */
void SysTick_Delay_ms(uint16_t ms)
{

    uint32_t temp = 0x00000000;

    // ��װ��ֵ�Ĵ���
    SysTick->LOAD = (uint32_t)(9000 * ms);
    // �ݼ�����������
    SysTick->VAL = 0x000000;
    // SysTick��ʱ��ʹ��
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    do
    {
        temp = SysTick->CTRL;
        // SysTick��ʱ��ʹ�� �� COUNTFLAGλ��һ
    } while ((temp & 0x01) && !(temp & (1 << 16)));
    // SysTick��ʱ��ʹ�ܱ�־λ����
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    // ��ռ�����
    SysTick->VAL = 0x000000;
}

void SysTick_Delay_us(uint16_t us)
{

    uint32_t temp = 0x00000000;

    // ��װ��ֵ�Ĵ���
    SysTick->LOAD = (uint32_t)(9 * us);
    // �ݼ�����������
    SysTick->VAL = 0x000000;
    // SysTick��ʱ��ʹ��
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;

    do
    {
        temp = SysTick->CTRL;
        // SysTick��ʱ��ʹ�� �� COUNTFLAGλ��һ
    } while ((temp & 0x01) && !(temp & (1 << 16)));
    // SysTick��ʱ��ʹ�ܱ�־λ����
    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
    // ��ռ�����
    SysTick->VAL = 0x000000;
}

/**
  * @��  ��  JTAGģʽ����
  * @��  ��  mode:jtag,swdģʽ����;00,ȫ����;01,ʹ��SWD;10,ȫ�ر�;  
              JTAG_SWD_DISABLE   0X02
              SWD_ENABLE         0X01
              JTAG_SWD_ENABLE    0X00	
  * @����ֵ  ��
  */
static void AX_JTAG_Set(uint8_t mode)
{
    uint32_t temp;

    temp=mode;
    temp<<=25;
    RCC->APB2ENR|=1<<0;     //��������ʱ��
    AFIO->MAPR&=0XF8FFFFFF; //���MARR��[26:24]
    AFIO->MAPR|=temp;       //����jtagģʽ
} 

void System_init(void)
{
    uint8_t state;
    // SystemInit();         // ����ϵͳƵ��72Mhz

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    AX_JTAG_Set(0x02);
    AX_JTAG_Set(0x01);    // ��ֹJTAG���ܣ���PB3��PB4��Ϊ��ͨIO��ʹ�� ������
    SysTick_Init();       // ϵͳ�δ���ʱ
    AKM_Servo_Init();     // ���
    AKM_LED_Init();       // LED�Ƴ�ʼ��
    AKM_BEEP_Init();      // ������
    AKM_Volt_Init();      // ��ѹ�ɼ���ʼ��

    AKM_MOTOR_Init(10);   // �����ʼ��
    

    AKM_UART_PI_Init();   // ����ݮ��ͨ�Ŵ���
    AKM_UART_PC_Init();   // ���Դ���
    AKM_TIME_Init(1000);  // ��ʱ���ж�1ms

    Vofa_Init(UART1SendByte);
                          // ��������ʼ
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
    // С�˴洢��ʽ
    // float a = 1.5;
    // uint8_t *p = (uint8_t *)(&a);
    // if (*p == 0x00) {
    //     LED_Red_On();
    // }
    // else {};
