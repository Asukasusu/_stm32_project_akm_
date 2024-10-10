#include "headfile.h"

volatile uint32_t sysTickUpTimes = 0;

/* 接收树莓派发送来的数据包 */
static void TaskDuty_3ms(void)
{
    UART_ReceDataPack();
}

/* 获取IMU三轴 角速度 加速度 角度 */
static void TaskDuty_10ms(void)
{
    MPU6050_DMP_GetData(mpu_data);
}

/* 底盘控制层 与 向树莓派发送数据 */
static void TaskDuty_20ms(void)
{
    AKM_ROBOT_MoveCtrl();
    USART_Transmiss_Datagroups();
}

/* 电池电压检测 */
static void TaskDuty_103ms(void)
{
    AKM_ROBOT_BatteryDetect();
}

static sched_task_t sched_tasks[] = 
{
    {TaskDuty_103ms ,  103, 0},
    {TaskDuty_20ms  ,   20, 0},
    {TaskDuty_10ms  ,   10, 0},
    {TaskDuty_3ms   ,    3, 0}

};

// 根据结构体数组长度 判断线程数量
// sizeof 既是关键字又是运算符
#define TASK_NUM (sizeof(sched_tasks)/sizeof(sched_task_t))
#define TNUMS    65535

/*
 *   参考自成都信息工程大学 智能车WD队
 *   简易任务调度
 */
void TaskLoop_Run(void)
{
    uint8_t index;

    for (index = 0; index < 4; index++) {
        uint16_t tnow = sysTickUpTimes;
        if ((tnow - sched_tasks[index].last_ticks + TNUMS)%TNUMS >= sched_tasks[index].interval_ticks) {
            sched_tasks[index].last_ticks = tnow;
            sched_tasks[index].pfunc();
        }
    }
}

