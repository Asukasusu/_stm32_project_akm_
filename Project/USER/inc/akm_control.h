#ifndef __AKM_CONTROL_H__
#define __AKM_CONTROL_H__

#include "akm_uart_pi.h"
#include "akm_encoder.h"

#define ROBOT_LINEAR_SPEED_LIMIT   5000
#define ROBOT_ANGULAR_SPEED_LIMIT  5000

#define ENCODER_LOW_WRAP  ((ENCODER_MAX - ENCODER_MIN)*0.3+ENCODER_MIN)
#define ENCODER_HIGH_WRAP ((ENCODER_MAX - ENCODER_MIN)*0.7+ENCODER_MIN)

#define PI 3.1415926

#define ANGLE_CAL(X) (PI-((X)-1500)/2000*PI)

typedef struct PID_Ctrl
{
    int16_t (*PID_VeloCtrl)(struct PID_Ctrl *,int16_t);
    void    (*PID_SpeedSet)(int16_t);
    int16_t Kp;
    int16_t Ki;
    int16_t Kd;
    int16_t PWM;
}PID_Ctrl;                           //16 Byte

extern int16_t encoder[4];
extern int16_t encoder_delta[4];
extern int16_t encoder_delta_target[4];
extern int16_t robot_odom[6];

void Control_Speed_Parameters(USART_Stru* pusart);
void Control_PID_Parameters(USART_Stru* pusart);
void Control_Robot_Parameters(USART_Stru* pusart);
int16_t PID_MotorVelocityCtrlA(PID_Ctrl *PIDControl, int16_t error);
int16_t PID_MotorVelocityCtrlB(PID_Ctrl *PIDControl, int16_t error);
void AKM_ROBOT_MoveCtrl(void);
#endif

