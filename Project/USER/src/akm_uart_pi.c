#include "stm32f10x.h"
#include "akm_uart_pi.h"
#include "akm_control.h"
#include "akm_Initial.h"

// static uint8_t uart2_rx_con=0;    //接收计数器
// static uint8_t uart2_rx_checksum; //帧头部分校验和
// static uint8_t uart2_rx_buf[60];  //接收缓冲
// static uint8_t uart2_tx_buf[60];  //发送缓冲
/*
 *
 *  X-Protocol协议介绍（变帧长）
 *  帧定义: 0xAA 0x55 | 0x0B | 0x01 | 0x03 0xE8 0xFC 0x18 0x00 0x0A | 0x14
 *            帧 头     帧长   帧码               数 据               校验 
 *  帧  头:  双帧头  抗干扰强
 *  帧  长:  根据数据长度设定
 *  帧  码:  用户根据功能设定
 *  数  据:  高位在前长度可变
 *  校验和:  数据累加和的低位（8）
 *  
 */
USART_Stru usart2;

void AKM_UART_PI_Init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

    GPIO_InitTypeDef   GPIO_InitStructure;
    USART_InitTypeDef  USART_InitStructure;
    NVIC_InitTypeDef   NVIC_InitStructure;
    
    //  UART2  PA2-Tx配置推挽复用模式
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    //  UART2  PA3-Tx配置浮空输入模式
    GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    
    /* U(S)ART(通用(同步)异步收发传输器) 串行通信
     * USART 123
     * UART  45
     */


    // 波特率 每秒传输的比特(bit) 位数
    USART_InitStructure.USART_BaudRate            = 115200;
    // 数据位 (USARTx->CR1)(12)M 5~8位逻辑位 设置奇偶校验位 则设置为9bit
    // 采用ASCII码形式 从低位开始传输 时钟定位
    USART_InitStructure.USART_WordLength          = USART_WordLength_8b;
    // 停止位 字符数据结束标志 {1位 1.5位 2位选项}
    USART_InitStructure.USART_StopBits            = USART_StopBits_1;
    // 校验位 {奇检验 偶检验 0检验}    0校验
    USART_InitStructure.USART_Parity              = USART_Parity_No; 
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    // 工作模式 收发
    USART_InitStructure.USART_Mode                = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(USART2, &USART_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel                   = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;           // 抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 1;           // 响应优先级
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;      // 中断使能
    NVIC_Init(&NVIC_InitStructure);

    // 清除标志位 状态寄存器 (USARTx->SR(9-0)) (05-r_w0)RXNE位
    /* 
       当RDR 移位寄存器 的数据转移到 数据寄存器(USART_DR)寄存器中 该位置1
       如果  控制寄存器1(USARTx->CR1(05-rw)) RXNEIE位 为1，则产生中断
       对    (USARTx->DR(8-0rw)) 可以将改位置 0 
     */
    USART_ClearFlag(USART2, USART_FLAG_RXNE);

    // 开启串口接受中断
    /* 
       控制寄存器1(USARTx->CR1(13-0rw))  
       (5)RXNEIE位 {0禁止中断 1(USARTx->SR)(3)ORE或(5)RXNE置1产生中断}
     */
    USART_ITConfig (USART2, USART_IT_RXNE, ENABLE);
    // USART使能 
    // 控制寄存器1(USARTx->CR1(13-0rw))  (13)UE位 
    USART_Cmd(USART2, ENABLE);

}

/*
 * @简  述  发送数据协议 
 * @参  数  *buf:   数据
 *           len:   数据长度
 *           num:   帧码
 * @返回值  无
 */
static void AKM_UART_PI_SendData(uint8_t *buf, uint8_t len, uint8_t num) {
    uint8_t i, cnt;
    uint8_t tx_checksum = 0;
    static uint8_t uart2_tx_buf[60]={0xAA,0x55};

    if (len > 50)  return;

    *(uart2_tx_buf+0) = 0xAA;
    *(uart2_tx_buf+1) = 0x55;
    *(uart2_tx_buf+2) = len+5; // 双帧头 帧长 帧码  校验
    *(uart2_tx_buf+3) = num;

    for(i = 0; i < len; i++) {
        *(uart2_tx_buf+4+i) = *(buf+i);
    }
    cnt = 4 + len; // 计算校验和
    for(i = 0; i < cnt; i++) {
        tx_checksum += *(uart2_tx_buf+i);
    }
    *(uart2_tx_buf+(cnt++)) = tx_checksum;

    // 发送数据
    for(i = 0; i < cnt; i++) {
        USART2->DR = (*(uart2_tx_buf+i) & (uint16_t)0x01FF);
        // 传输成功标志位 状态寄存器(USART_SR)(6)TC发送完成标志位
        while(!(USART2->SR & USART_FLAG_TC));
    }
}

extern uint16_t bat_vol_dat;

/*
 * @简  述  发送数据到树莓派 
 * @参  数  无
 * @返回值  无
 */
void USART_Transmiss_Datagroups(void)
{
    static uint8_t comdata[41];

    // gyro 
    comdata[0]  = (u8)( mpu_data[0] >> 8 );
    comdata[1]  = (u8)( mpu_data[0] );
    comdata[2]  = (u8)( mpu_data[1] >> 8 );
    comdata[3]  = (u8)( mpu_data[1] );
    comdata[4]  = (u8)( mpu_data[2] >> 8 );
    comdata[5]  = (u8)( mpu_data[2] );

    // acce
    comdata[6]  = (u8)( mpu_data[3] >> 8 );
    comdata[7]  = (u8)( mpu_data[3] );
    comdata[8]  = (u8)( mpu_data[4] >> 8 );
    comdata[9]  = (u8)( mpu_data[4] );
    comdata[10] = (u8)( mpu_data[5] >> 8 );
    comdata[11] = (u8)( mpu_data[5] );

    // angle
    comdata[12] = (u8)( mpu_data[6] >> 8 );
    comdata[13] = (u8)( mpu_data[6] );
    comdata[14] = (u8)( mpu_data[7] >> 8 );
    comdata[15] = (u8)( mpu_data[7] );
    comdata[16] = (u8)( mpu_data[8] >> 8 );
    comdata[17] = (u8)( mpu_data[8] );

    // 里程计坐标
    comdata[18] = (u8)( robot_odom[0] >> 8 );
    comdata[19] = (u8)( robot_odom[0] );
    comdata[20] = (u8)( robot_odom[1] >> 8 );
    comdata[21] = (u8)( robot_odom[1] );
    comdata[22] = (u8)( robot_odom[2] >> 8 );
    comdata[23] = (u8)( robot_odom[2] );

    // 里程计变化量
    comdata[24] = (u8)( robot_odom[3] >> 8 );
    comdata[25] = (u8)( robot_odom[3] );
    comdata[26] = (u8)( robot_odom[4] >> 8 );
    comdata[27] = (u8)( robot_odom[4] );
    comdata[28] = (u8)( robot_odom[5] >> 8 );
    comdata[29] = (u8)( robot_odom[5] );

    //编码器当前值和目标值
    comdata[30] = (u8)( encoder_delta[0] >> 8 );
    comdata[31] = (u8)( encoder_delta[0] );
    comdata[32] = (u8)( encoder_delta[1] >> 8 );
    comdata[33] = (u8)( encoder_delta[1] );
    comdata[34] = (u8)( encoder_delta_target[0] >> 8 );
    comdata[35] = (u8)( encoder_delta_target[0] );
    comdata[36] = (u8)( encoder_delta_target[1] >> 8 );
    comdata[37] = (u8)( encoder_delta_target[1] );

    // 电压值
    comdata[38] = (u8)( bat_vol_dat >> 8 );
    comdata[39] = (u8)( bat_vol_dat );

    AKM_UART_PI_SendData(comdata, 40, 0x06);
}


// 接受USART数据
static void USART_Receive_Data(uint8_t data, USART_Stru* pusart) {
    pusart->buffer[pusart->head] = data;
    // 环形存储 0 - (BUF_MAX_LEN-1)中
    pusart->head = (pusart->head + 1)%BUF_MAX_LEN;
    pusart->count++;
    if (pusart->count > BUF_MAX_LEN) {
        pusart->count = 0;
        pusart->head  = 0;
        pusart->tail  = 0;
    }

}

static pfunc p_func[]=
{
    {Control_Speed_Parameters, 0},
    {Control_PID_Parameters,   0},
    {Control_Robot_Parameters, 0}
};

//
static uint8_t USART_Data_Copy(USART_Stru* pusart, uint8_t len) {
    uint8_t i, checksum=0;

    for (i = 0; i < len; i++) {
        pusart->buffer_t[i] = pusart->buffer[(pusart->tail+i)%BUF_MAX_LEN];
    }
    // 计算校验位
    for (i = 0; i < (len-1); i++) {
        checksum += pusart->buffer_t[i];
    }
    if (checksum != pusart->buffer_t[len-1]) {BEEP_On(); return 0;}

    //  运动控制帧
    // if (pusart->buffer_t[3] == 0x11) {
    //     Control_Speed_Parameters(pusart);
    // } 
    // //  PID参数帧
    // else if (pusart->buffer_t[3] == 0x12) {
    //     Control_PID_Parameters(pusart);
    // }
    // //  机器人参数帧
    // else if (pusart->buffer_t[3] == 0x13) {
    //     Control_Robot_Parameters(pusart);
    // }

    // 选择对应的函数
    p_func[pusart->buffer_t[3] - 0x11].p_fun(pusart);
    // typedef pfunc(pusart);    //函数指针可以使用


    BEEP_Off();
    return 1;
}


uint8_t USART_Receive_Analysis(USART_Stru* pusart) {
    // uint8_t i;
    uint8_t count = pusart->count;
    // uint8_t head  = pusart->head;
    uint8_t tail  = pusart->tail;
    uint8_t lenght;

    if (count >= FRAMLEN_MIN) {
        if (pusart->buffer[(tail+0)%BUF_MAX_LEN] == 0xAA &&
            pusart->buffer[(tail+1)%BUF_MAX_LEN] == 0x55) {

               lenght = pusart->buffer[(tail+2)%BUF_MAX_LEN];
               if (count >= lenght) {
                   // 数据包处理
                   USART_Data_Copy(pusart, lenght);

                   pusart->count = pusart->count - lenght;
                   pusart->tail  = (tail+lenght)%BUF_MAX_LEN;
                   return 1;
                   
               }else{
                   return 2;
               }
        }else{
            pusart->count = pusart->count - 1;
            pusart->tail  = (pusart->tail+1)%BUF_MAX_LEN;
            return 3;
        }
    }else{
        return 4;
    }
}

// 串口中断函数
void USART2_IRQHandler(void) {
    uint8_t Res;

    // 控制寄存器 1(USARTx->CR1(13-0rw)) (5)RXNEIE位
    // 状态寄存器 (USARTx->SR(9-0))  (05-r_w0)RXNE位
    // 检测RXNEIE位（中断位）与RXNE位（可读数据）是否为1
    if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) {

        // 读(USARTx->SR(9-0)) (05-r_w0)RXNE位置0
        Res = (USART2->DR & (uint16_t)0x01FF);
        USART_Receive_Data(Res, &usart2);
        USART2->SR = (uint16_t)~USART_FLAG_RXNE; // 多缓存通讯使用
    }
}

void UART_ReceDataPack(void)
{
    USART_Receive_Analysis(&usart2);
}
