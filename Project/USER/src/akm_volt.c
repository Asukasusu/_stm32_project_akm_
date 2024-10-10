#include "stm32f10x.h"
#include "akm_volt.h"
#include "akm_Initial.h"

void AKM_Volt_Init(void)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_ADC2, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    // 时钟配置寄存器(RCC->CFGR) 设置ADC工作频率为72MHz/6=12MHz ADC最大工作频率为14MHz
    RCC_ADCCLKConfig(RCC_PCLK2_Div6);

    // ADC模块初始化函数 将ADC模块的所有寄存器恢复到默认值
    ADC_DeInit(ADC2);

    // ADC控制寄存器(ADC->CR1) (31:24)保留
    //   (19:16) 独立模式
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;

    // ADC控制寄存器(ADC->CR1)(8) 扫描模式 {0单通道转换 1所有通道转换}
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    // ADC控制寄存器(ADC->CR2) (31:24)保留
    // (1) 连续模式 {0单次转换 1连续转化借助DMA实现 }
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    // ADC控制寄存器(ADC->CR2)(19：17) 转换的外部事件 软件触发
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    // ADC控制寄存器(ADC->CR2)(11) 对齐方式 {0 右对齐(低于12位 保低位数据) 左对齐(高于12位 保高位数据)}
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    // 顺序进行规则转换的ADC通道的数目
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC2, &ADC_InitStructure);

    RCC_ADCCLKConfig(RCC_PCLK2_Div6);
    // 指定ADC的规则组通道 规则采样顺序值为1 采样为239.5周期
    // ADC最大频率14MHz  1.5(最小采样周期) + 12.5(转换周期) = 14   1us 故不能大于14Mhz频率 损失ADC精度
    ADC_RegularChannelConfig(ADC2, ADC_Channel_8, 1, ADC_SampleTime_239Cycles5);

    ADC_Cmd(ADC2, ENABLE);

    // 开启复位校准 (ADC->CR2)(3)RSTCAL
    ADC_ResetCalibration(ADC2);
    // 等待复位校准结
    while (ADC_GetResetCalibrationStatus(ADC2));
    // 开启 AD 校准 (ADC->CR2)(2)CAL
    ADC_StartCalibration(ADC2);
    // 等待校准结束
    while (ADC_GetCalibrationStatus(ADC2));
}

static uint16_t AKM_GetVolt_X100(void)
{
    uint16_t Volt, temp;
    ADC2->CR2 |= ((uint32_t)0x00500000); // SWSTART位 (置一 开始转换规则通道)(0 复位状态)
    while (!(ADC2->SR & ADC_FLAG_EOC));  // 等待转换结束 EOC标志位(0为转化完成 1转化完成)

    // 10ko 1ko   电压比10：1 以电池12v电压 ADC引脚输入最大电压约为1.1v ADC读取数值为1365*3636/4095对应(1200)
    temp = ADC2->DR;
    Volt = ((uint16_t)((temp * 3636) >> 12));
    return Volt;
}

uint16_t bat_vol_dat = 0;


// 电源检测
void AKM_ROBOT_BatteryDetect(void)
{
    static uint16_t bat_vol_cnt = 0;

    bat_vol_dat = AKM_GetVolt_X100();

    if (bat_vol_dat < VBAT_VOL_CHG)
    {
        LED_Red_Tgl();
        if (bat_vol_dat < VBAT_VOL_OFF)
        {
            bat_vol_cnt++;
            if (bat_vol_cnt > 200)
            {
                LED_Red_On();
                LED_Gre_Off();
                AKM_MOTOR_A_SpeedSet(0);
                AKM_MOTOR_B_SpeedSet(0);
                do
                {
                    BEEP_On();
                    SysTick_Delay_ms(30);
                    BEEP_Off();
                    SysTick_Delay_ms(1000);
                } while (1);
            }
        }
        else
        {
            bat_vol_cnt = 0;
        }
    }
    else
    {
        LED_Red_Off();
    }
}




