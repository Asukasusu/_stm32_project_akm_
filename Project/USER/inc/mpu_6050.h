#ifndef __MPU_6050_H__
#define __MPU_6050_H__

#include <stdint.h>

// IO 操作宏定义
#define BITBAND(addr, bitnum)    ((addr & 0xF0000000)+0x2000000+((addr &0xFFFFF)<<5)+(bitnum<<2)) 
#define MEM_ADDR(addr)           *((volatile unsigned long  *)(addr)) 
#define BIT_ADDR(addr, bitnum)   MEM_ADDR(BITBAND(addr, bitnum)) 

// IO 口地址映射
#define GPIOB_ODR_Addr    (GPIOB_BASE+12) //0x40010C0C
#define GPIOB_IDR_Addr    (GPIOB_BASE+8)  //0x40010C08
#define PBout(n)   BIT_ADDR(GPIOB_ODR_Addr,n)
#define PBin(n)    BIT_ADDR(GPIOB_IDR_Addr,n)

// IO 方向设置
#define    MPU_SDA_IN()    {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)8<<4;}
#define    MPU_SDA_OT()    {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=(u32)3<<4;}

// IO 操作函数
#define    MPU_IIC_SCL        PBout(8)     // SCL
#define    MPU_IIC_SDA        PBout(9)     // SDA
#define    MPU_READ_SDA       PBin(9)      // 输入SDA

#define  MPU_SELF_TESTX_REG        0X0D    // 自检寄存器X
#define  MPU_SELF_TESTY_REG        0X0E    // 自检寄存器Y
#define  MPU_SELF_TESTZ_REG        0X0F    // 自检寄存器Z
#define  MPU_SELF_TESTA_REG        0X10    // 自检寄存器A
#define  MPU_SAMPLE_RATE_REG       0X19    // 采样频率分频器
#define  MPU_CFG_REG               0X1A    // 配置寄存器
#define  MPU_GYRO_CFG_REG          0X1B    // 陀螺仪配置寄存器
#define  MPU_ACCEL_CFG_REG         0X1C    // 加速度计配置寄存器
#define  MPU_MOTION_DET_REG        0X1F    // 运动检测阀值设置寄存器
#define  MPU_FIFO_EN_REG           0X23    // FIFO使能寄存器
#define  MPU_I2CMST_CTRL_REG       0X24    // IIC主机控制寄存器
#define  MPU_I2CSLV0_ADDR_REG      0X25    // IIC从机0器件地址寄存器
#define  MPU_I2CSLV0_REG           0X26    // IIC从机0数据地址寄存器
#define  MPU_I2CSLV0_CTRL_REG      0X27    // IIC从机0控制寄存器
#define  MPU_I2CSLV1_ADDR_REG      0X28    // IIC从机1器件地址寄存器
#define  MPU_I2CSLV1_REG           0X29    // IIC从机1数据地址寄存器
#define  MPU_I2CSLV1_CTRL_REG      0X2A    // IIC从机1控制寄存器
#define  MPU_I2CSLV2_ADDR_REG      0X2B    // IIC从机2器件地址寄存器
#define  MPU_I2CSLV2_REG           0X2C    // IIC从机2数据地址寄存器
#define  MPU_I2CSLV2_CTRL_REG      0X2D    // IIC从机2控制寄存器
#define  MPU_I2CSLV3_ADDR_REG      0X2E    // IIC从机3器件地址寄存器
#define  MPU_I2CSLV3_REG           0X2F    // IIC从机3数据地址寄存器
#define  MPU_I2CSLV3_CTRL_REG      0X30    // IIC从机3控制寄存器
#define  MPU_I2CSLV4_ADDR_REG      0X31    // IIC从机4器件地址寄存器
#define  MPU_I2CSLV4_REG           0X32    // IIC从机4数据地址寄存器
#define  MPU_I2CSLV4_DO_REG        0X33    // IIC从机4写数据寄存器
#define  MPU_I2CSLV4_CTRL_REG      0X34    // IIC从机4控制寄存器
#define  MPU_I2CSLV4_DI_REG        0X35    // IIC从机4读数据寄存器

#define  MPU_I2CMST_STA_REG        0X36    // IIC主机状态寄存器
#define  MPU_INTBP_CFG_REG         0X37    // 中断/旁路设置寄存器
#define  MPU_INT_EN_REG            0X38    // 中断使能寄存器
#define  MPU_INT_STA_REG           0X3A    // 中断状态寄存器

#define  MPU_ACCEL_XOUTH_REG       0X3B    // 加速度值 X轴高8位寄存器
#define  MPU_ACCEL_XOUTL_REG       0X3C    // 加速度值 X轴低8位寄存器
#define  MPU_ACCEL_YOUTH_REG       0X3D    // 加速度值 Y轴高8位寄存器
#define  MPU_ACCEL_YOUTL_REG       0X3E    // 加速度值 Y轴低8位寄存器
#define  MPU_ACCEL_ZOUTH_REG       0X3F    // 加速度值 Z轴高8位寄存器
#define  MPU_ACCEL_ZOUTL_REG       0X40    // 加速度值 Z轴低8位寄存器

#define  MPU_TEMP_OUTH_REG         0X41    // 温度值高8位寄存器
#define  MPU_TEMP_OUTL_REG         0X42    // 加速度低8位寄存器

#define  MPU_GYRO_XOUTH_REG        0X43    // 陀螺仪值 X轴高8位寄存器
#define  MPU_GYRO_XOUTL_REG        0X44    // 陀螺仪值 X轴低8位寄存器
#define  MPU_GYRO_YOUTH_REG        0X45    // 陀螺仪值 Y轴高8位寄存器
#define  MPU_GYRO_YOUTL_REG        0X46    // 陀螺仪值 Y轴低8位寄存器
#define  MPU_GYRO_ZOUTH_REG        0X47    // 陀螺仪值 Z轴高8位寄存器
#define  MPU_GYRO_ZOUTL_REG        0X48    // 陀螺仪值 Z轴低8位寄存器

#define  MPU_I2CSLV0_DO_REG        0X63    // IIC从机0数据寄存器
#define  MPU_I2CSLV1_DO_REG        0X64    // IIC从机1数据寄存器
#define  MPU_I2CSLV2_DO_REG        0X65    // IIC从机2数据寄存器
#define  MPU_I2CSLV3_DO_REG        0X66    // IIC从机3数据寄存器

#define  MPU_I2CMST_DELAY_REG      0X67    // IIC主机延时管理寄存器
#define  MPU_SIGPATH_RST_REG       0X68    // 信号通道复位寄存器
#define  MPU_MDETECT_CTRL_REG      0X69    // 运动检测控制寄存器
#define  MPU_USER_CTRL_REG         0X6A    // 用户控制寄存器
#define  MPU_PWR_MGMT1_REG         0X6B    // 电源管理寄存器1
#define  MPU_PWR_MGMT2_REG         0X6C    // 电源管理寄存器2
#define  MPU_FIFO_CNTH_REG         0X72    // FIFO计数寄存器高8位
#define  MPU_FIFO_CNTL_REG         0X73    // FIFO计数寄存器低8位
#define  MPU_FIFO_RW_REG           0X74    // FIFO读写寄存器
#define  MPU_DEVICE_ID_REG         0X75    // 器件ID寄存器

#define  MPU6050_ADDR              0x68    // 地址

#define DEFAULT_MPU_HZ  (100)
#define q15   32768.0f   
#define q30   1073741824.0f

uint8_t MPU6050_Init(void);

uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

int16_t MPU_Get_GyroData_X(void);
int16_t MPU_Get_GyroData_Y(void);
int16_t MPU_Get_GyroData_Z(void);
uint8_t MPU_Get_GyroData(int16_t *gx, int16_t *gy, int16_t *gz);

int16_t MPU_Get_AcceData_X(void);
int16_t MPU_Get_AcceData_Y(void);
int16_t MPU_Get_AcceData_Z(void);
uint8_t MPU_Get_AcceData(int16_t *ax, int16_t *ay, int16_t *az);


void MPU6050_DMP_Init(void);
void MPU6050_DMP_GetData(int16_t *data);

extern int16_t mpu_data[10];

#endif
