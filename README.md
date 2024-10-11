#  **stm32 project akm** 
## 简介
塔克创新 阿克曼式ROS机器人的底层控制代码 自用第一版

## STM32
基于STM32F103RC和STM32标准库建立的工程

## 任务简介
3ms&nbsp;&nbsp;&nbsp;&nbsp;执行一次 解析数据包任务  
10ms&nbsp;&nbsp;执行一次 MPU6050的DMP库解算三轴角度  
20ms&nbsp;&nbsp;执行一次 舵机控制、电机控制并向NVIDA Jetson Nano发送一次数据包  
103ms&nbsp;执行一次 电池电源检测函数

## IDE
KEIL5、VScode

## 引用
**作者**[CloverGit](https://github.com/CloverGit/Vofa-Plus-Protocol-Driver) 依据VOFA+的JUSTFLOAT数据协议模式编写的数据发送库  
**作者**[huangcheng188](https://blog.csdn.net/HC_huangcheng/article/details/101762634) 依据FIFO性质编写的串口环形缓冲算法  
**作者**[Kevincoooool](https://github.com/Kevincoooool/2018-NXP-CUIT-WD) 编写的一种任务调度算法
