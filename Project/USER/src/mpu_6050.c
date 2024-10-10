#include "stm32f10x.h"

#include <stdio.h>
#include <math.h>
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "mpu_6050.h"
#include "akm_Initial.h"

int16_t mpu_data[10];

//  软件模拟IIC
void MPU_IIC_Init(void) {
    GPIO_InitTypeDef  GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_SetBits(GPIOB,GPIO_Pin_10|GPIO_Pin_11);
}

// 产生IIC起始信号
void MPU_IIC_Start(void)
{
    MPU_SDA_OT();
    MPU_IIC_SDA = 1;
    MPU_IIC_SCL = 1;
    SysTick_Delay_us(2);
    MPU_IIC_SDA = 0;
    SysTick_Delay_us(2);
    MPU_IIC_SCL = 0;
}

// 产生IIC停止信号
void MPU_IIC_Stop (void)
{
    MPU_SDA_OT();
    MPU_IIC_SCL = 0;
    MPU_IIC_SDA = 0;
    SysTick_Delay_us(2);
    MPU_IIC_SCL = 1;
    MPU_IIC_SDA = 1;
    SysTick_Delay_us(2);
}

// 等待应答信号
uint8_t MPU_IIC_Wait_Ack(void)
{
    uint8_t ucErrTime = 0;
    MPU_SDA_IN();
    MPU_IIC_SDA = 1;
    SysTick_Delay_us(2);
    MPU_IIC_SCL = 1;
    SysTick_Delay_us(2);
    while (MPU_READ_SDA) {
        ucErrTime++;
        if (ucErrTime > 250) {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_SCL = 0;
    return 0;
}

// 产生ACK应答
void MPU_IIC_Ack(void)
{
    MPU_IIC_SCL = 0;
    MPU_SDA_OT();
    MPU_IIC_SDA = 0;
    SysTick_Delay_us(2);
    MPU_IIC_SCL = 1;
    SysTick_Delay_us(2);
    MPU_IIC_SCL = 0;
}

// 不产生ACK应答
void MPU_IIC_NAck(void)
{
    MPU_IIC_SCL=0;
    MPU_SDA_OT();
    MPU_IIC_SDA=1;
    SysTick_Delay_us(2);
    MPU_IIC_SCL=1;
    SysTick_Delay_us(2);
    MPU_IIC_SCL=0;
}

// IIC发送一个字节
void MPU_IIC_Send_Byte(uint8_t txd)
{
    uint8_t t;
    MPU_SDA_OT();
    MPU_IIC_SCL = 0;
    for (t = 0; t < 8; t++) {
        MPU_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
        MPU_IIC_SCL=1;
        SysTick_Delay_us(2); 
        MPU_IIC_SCL=0;	
        SysTick_Delay_us(2);
    }
}

// IIC读取一个字节
uint8_t MPU_IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive = 0;
    MPU_SDA_IN();
    for (i = 0; i < 8 ; i++) {
        MPU_IIC_SCL=0; 
        SysTick_Delay_us(2);
        MPU_IIC_SCL=1;
        receive <<= 1;
        if(MPU_READ_SDA) receive++;
        SysTick_Delay_us(2);
    }

    if (!ack) MPU_IIC_NAck();
    else      MPU_IIC_Ack();
    return receive;
}

/*
 * @简  述   IIC---写入一个字节
 * @参  数   reg:  寄存器地址
 *           data: 要写入的数据
 * @返回值   0    正常
 *           其他 错误
 * @说  明   
 */
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU6050_ADDR<<1)|0);
    if (MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(data);
    if (MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}

/*
 * @简  述   IIC---读入一个字节
 * @参  数   reg:  寄存器地址
 * @返回值   读取到的数据      
 * @说  明   
 */
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU6050_ADDR<<1)|0);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU6050_ADDR<<1)|1);
    MPU_IIC_Wait_Ack();
    res=MPU_IIC_Read_Byte(0);
    MPU_IIC_Stop();
    return res;
}

/*
 * @简  述   IIC---连续写入字节
 * @参  数   addr:  器件地址
 *           reg :  寄存器地址
 *           len :  写入长度
 *           buf :  数据组
 * @返回值   0    正常
 *           其他 错误
 */
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|0);
    if (MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    for (i = 0; i < len; i++) {
        MPU_IIC_Send_Byte(buf[i]);
        if(MPU_IIC_Wait_Ack()) {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_Stop();
    return 0;
}


/*
 * @简  述   IIC---连续读取字节
 * @参  数   addr:  器件地址
 *           reg :  寄存器地址
 *           len :  写入长度
 *           buf :  存数据组
 * @返回值   0    正常
 *           其他 错误
 */
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|0);
    if(MPU_IIC_Wait_Ack()) {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);
    MPU_IIC_Wait_Ack();
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr<<1)|1);
    MPU_IIC_Wait_Ack();
    while(len) {
        if(len==1)*buf=MPU_IIC_Read_Byte(0);
        else *buf=MPU_IIC_Read_Byte(1);
        len--;buf++;
    }
    MPU_IIC_Stop();
    return 0;
}

// 设置MPU6050数字低通滤波器
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data=0;
    if(lpf>=188)data=1;
    else if(lpf>=98)data=2;
    else if(lpf>=42)data=3;
    else if(lpf>=20)data=4;
    else if(lpf>=10)data=5;
    else data=6; 
    return MPU_Write_Byte(MPU_CFG_REG,data);
}

// 设置MPU6050的采样频率
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if(rate>1000)rate=1000;
    if(rate<4)rate=4;
    data=1000/rate-1;
    // 由采样频率 = 陀螺仪输出频率 / (1+SMPLRT_DIV)
    // SMPLRT_DIV = 陀螺仪输出频率 / 采样频率 - 1
    data=MPU_Write_Byte(MPU_SAMPLE_RATE_REG,data);
    return MPU_Set_LPF(rate/2);
}

uint8_t MPU6050_Init(void)
{
    /* 陀螺仪低频噪声 加速度高频噪声 */
    MPU_IIC_Init();
    // 复位MPU6050
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);
    SysTick_Delay_ms(100);
    // 唤醒MPU6050
    MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);
    MPU_Write_Byte(MPU_GYRO_CFG_REG, 0x18);   // 陀螺仪 ± 2000dps
    MPU_Write_Byte(MPU_ACCEL_CFG_REG,0x00);   // 加速度 ± 2g

    MPU_Write_Byte(MPU_INT_EN_REG,   0X00);   // 关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);   // IIC主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG,  0X00);   // 关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);   // INT引脚低电平有效

    if(MPU_Read_Byte(MPU_DEVICE_ID_REG) == MPU6050_ADDR) {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01); // 设置CLKSEL，PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00); // 加速度计和陀螺仪都工作
        // MPU_Write_Byte(MPU_SAMPLE_RATE_REG,19); // 采用频率 设置为50Hz
        // MPU_Write_Byte(MPU_CFG_REG,4);          // 数字低通滤波
    }else return 1;
    return 0;
}

// X轴陀螺仪
int16_t MPU_Get_GyroData_X(void)
{
    uint8_t buf[2];
    MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,2,buf);
    return ((buf[0]<<8) | buf[1]);
}
// Y轴陀螺仪
int16_t MPU_Get_GyroData_Y(void)
{
    uint8_t buf[2];
    MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_YOUTH_REG,2,buf);
    return ((buf[0]<<8) | buf[1]);
}
// Z轴陀螺仪
int16_t MPU_Get_GyroData_Z(void)
{
    uint8_t buf[2];
    MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_ZOUTH_REG,2,buf);
    return ((buf[0]<<8) | buf[1]);
}
uint8_t MPU_Get_GyroData(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6],res;
    res=MPU_Read_Len(MPU6050_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
    if (res == 0) {
        *gx = ((uint16_t)buf[0]<<8) | buf[1];
        *gy = ((uint16_t)buf[2]<<8) | buf[3];
        *gz = ((uint16_t)buf[4]<<8) | buf[5];
    }
    return res;
}


// X轴加速度
int16_t MPU_Get_AcceData_X(void)
{
    uint8_t buf[2];
    MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,2,buf);
    return ((buf[0]<<8) | buf[1]);
}
// Y轴加速度
int16_t MPU_Get_AcceData_Y(void)
{
    uint8_t buf[2];
    MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_YOUTH_REG,2,buf);
    return ((buf[0]<<8) | buf[1]);
}
// Z轴加速度
int16_t MPU_Get_AcceData_Z(void)
{
    uint8_t buf[2];
    MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_ZOUTH_REG,2,buf);
    return ((buf[0]<<8) | buf[1]);
}
uint8_t MPU_Get_AcceData(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6],res;
    res=MPU_Read_Len(MPU6050_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
    if (res == 0) {
        *ax = ((uint16_t)buf[0]<<8) | buf[1];
        *ay = ((uint16_t)buf[2]<<8) | buf[3];
        *az = ((uint16_t)buf[4]<<8) | buf[5];
    }
    return res;
}

/*------------------------------------------------------------------------------------------------*/

short sensors;
float ax_pitch,ax_roll,ax_yaw; 
float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;

static signed char gyro_orientation[9] = { 1, 0, 0,
                                           0, 1, 0,
                                           0, 0, 1};


static uint16_t inv_row_2_scale(const signed char *row)
{
    unsigned short b;

    if (row[0] > 0)
        b = 0;
    else if (row[0] < 0)
        b = 4;
    else if (row[1] > 0)
        b = 1;
    else if (row[1] < 0)
        b = 5;
    else if (row[2] > 0)
        b = 2;
    else if (row[2] < 0)
        b = 6;
    else
        b = 7;      // error
    return b;
}


static  uint16_t inv_orientation_matrix_to_scalar(const signed char *mtx)
{
    unsigned short scalar;
    scalar = inv_row_2_scale(mtx);
    scalar |= inv_row_2_scale(mtx + 3) << 3;
    scalar |= inv_row_2_scale(mtx + 6) << 6;

    return scalar;
}

static void run_self_test(void)
{
    int result;
    long gyro[3], accel[3];

    result = mpu_run_self_test(gyro, accel);

    if (result == 0x3) 
    {
        /* Test passed. We can trust the gyro data here, so let's push it down
         * to the DMP.
         */
        float sens;
        unsigned short accel_sens;

        mpu_get_gyro_sens(&sens);
        gyro[0] = (long)(gyro[0] * sens);
        gyro[1] = (long)(gyro[1] * sens);
        gyro[2] = (long)(gyro[2] * sens);
        dmp_set_gyro_bias(gyro);

        mpu_get_accel_sens(&accel_sens);
        accel[0] *= accel_sens;
        accel[1] *= accel_sens;
        accel[2] *= accel_sens;
        dmp_set_accel_bias(accel);
    }
}

/*
 * @简  述   MPU6050 DMP初始化
 * @参  数   无
 * @返回值   无
 */
void MPU6050_DMP_Init(void)
{
    uint8_t res=0;

    if(mpu_init() == 0)
    {
        res=mpu_set_sensors(INV_XYZ_GYRO|INV_XYZ_ACCEL);//设置所需要的传感器
        if(res) printf("mpu_set_sensor error\r\n");

        res=mpu_configure_fifo(INV_XYZ_GYRO | INV_XYZ_ACCEL);//设置FIFO
        if(res) printf("mpu_configure_fifo error\r\n"); 

        res=mpu_set_sample_rate(DEFAULT_MPU_HZ);    //设置采样率
        if(res) printf("mpu_set_sample_rate error\r\n");

        res=dmp_load_motion_driver_firmware();      //加载dmp固件
        if(res) printf("dmp_load_motion_driver_firmware error\r\n"); 

        res=dmp_set_orientation(inv_orientation_matrix_to_scalar(gyro_orientation));//设置陀螺仪方向
        if(res) printf("dmp_set_orientation error\r\n");; 

        res=dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT|DMP_FEATURE_TAP|    //设置dmp功能
            DMP_FEATURE_ANDROID_ORIENT|DMP_FEATURE_SEND_RAW_ACCEL|DMP_FEATURE_SEND_CAL_GYRO|
            DMP_FEATURE_GYRO_CAL);
        if(res) printf("dmp_enable_feature error\r\n");

        res=dmp_set_fifo_rate(DEFAULT_MPU_HZ);    //设置DMP输出速率(最大不超过200Hz)
        if(res) printf("dmp_set_fifo_rate error.\r\n");

        run_self_test();  //校准

        res=mpu_set_dmp_state(1);  //使能DMP
        if(res) printf("mpu_set_dmp_state error.\r\n"); 
    }
}

void MPU6050_DMP_GetData(int16_t *data)
{
    unsigned long sensor_timestamp;
    unsigned char more;
    long quat[4];
    dmp_read_fifo(&data[0], &data[3], quat, &sensor_timestamp, &sensors, &more);

    if ( sensors & INV_WXYZ_QUAT )
    {
        q0=quat[0] / q30;
        q1=quat[1] / q30;
        q2=quat[2] / q30;
        q3=quat[3] / q30;

        ax_pitch = asin(-2 * q1 * q3 + 2 * q0* q2)* 57.3; 	//pitch
        ax_roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* 57.3; //roll
        ax_yaw  = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.3;	//yaw

        data[6] = ax_roll*100;
        data[7] = ax_pitch*100;
        data[8] = ax_yaw*100;
    }
}


