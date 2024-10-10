#include "akm_encoder.h"
#include "akm_uart_pi.h"
#include "akm_control.h"
#include "akm_motor.h"
#include "akm_servo.h"
#include "VofaPlus.h"
#include "akm_servo.h"
#include "stm32f10x.h"
#include <math.h>

int16_t encoder_delta[4];
int16_t encoder_delta_target[4];
int16_t robot_target_speed[3];
int16_t robot_odom[6];
// int16_t akm_motor_kp;
// int16_t akm_motor_ki;
// int16_t akm_motor_kd;

float    linear_correction_factor = 1.0;
int16_t  servo_bias = 0;
float    ticks_per_meter = 0;//4418.181461;

static PID_Ctrl CtrlParam[] = 
{
    {PID_MotorVelocityCtrlA, AKM_MOTOR_A_SpeedSet, 0, 0, 0, 0},
    {PID_MotorVelocityCtrlB, AKM_MOTOR_B_SpeedSet, 0, 0, 0, 0}
};

static void AKM_Kinematics_Init(int16_t* robot_params)
{
    linear_correction_factor = (float)robot_params[0] / 1000;
    servo_bias = robot_params[1];

    ticks_per_meter    = (float)ENCODER_RESOLUTION/(WHEEL_DIAMETER*3.1415926*linear_correction_factor);
}

static void AKM_PIDParams_Init(PID_Ctrl* PIDControl, int16_t *input)
{
    PIDControl->Kp = input[0];
    PIDControl->Kd = input[1];
    PIDControl->Ki = input[2];
}

// 运动控制参数
void Control_Speed_Parameters(USART_Stru* pusart)
{
    robot_target_speed[0] = (int16_t)(pusart->buffer_t[4] << 8 | pusart->buffer_t[5]);
    robot_target_speed[1] = (int16_t)(pusart->buffer_t[6] << 8 | pusart->buffer_t[7]);
    robot_target_speed[2] = (int16_t)(pusart->buffer_t[8] << 8 | pusart->buffer_t[9]);

    robot_target_speed[0] = robot_target_speed[0] >  ROBOT_LINEAR_SPEED_LIMIT ?  ROBOT_LINEAR_SPEED_LIMIT : robot_target_speed[0];
    robot_target_speed[0] = robot_target_speed[0] < -ROBOT_LINEAR_SPEED_LIMIT ? -ROBOT_LINEAR_SPEED_LIMIT : robot_target_speed[0];
    robot_target_speed[1] = robot_target_speed[1] >  ROBOT_LINEAR_SPEED_LIMIT ?  ROBOT_LINEAR_SPEED_LIMIT : robot_target_speed[1];
    robot_target_speed[1] = robot_target_speed[1] < -ROBOT_LINEAR_SPEED_LIMIT ? -ROBOT_LINEAR_SPEED_LIMIT : robot_target_speed[1];
    
    robot_target_speed[2] = robot_target_speed[2] >  ROBOT_ANGULAR_SPEED_LIMIT ?  ROBOT_ANGULAR_SPEED_LIMIT : robot_target_speed[2];
    robot_target_speed[2] = robot_target_speed[2] < -ROBOT_ANGULAR_SPEED_LIMIT ? -ROBOT_ANGULAR_SPEED_LIMIT : robot_target_speed[2];

    // float data[2] = {robot_target_speed[0], robot_target_speed[2]};
    // Vofa_JustFloatSendDataAll(data, 2);
}

// PID控制参数
void Control_PID_Parameters(USART_Stru* pusart)
{
    int16_t MotorPidParam[3];
    MotorPidParam[0] = (int16_t)(pusart->buffer_t[4] << 8 | pusart->buffer_t[5]);
    MotorPidParam[1] = (int16_t)(pusart->buffer_t[6] << 8 | pusart->buffer_t[7]);
    MotorPidParam[2] = (int16_t)(pusart->buffer_t[8] << 8 | pusart->buffer_t[9]);

    AKM_PIDParams_Init(&CtrlParam[0], MotorPidParam);
    AKM_PIDParams_Init(&CtrlParam[1], MotorPidParam);
}

// 机器人控制参数
void Control_Robot_Parameters(USART_Stru* pusart)
{
    static int16_t robot_params[2] = {1000, 0};
    robot_params[0] = (int16_t)(pusart->buffer_t[4]<<8 | pusart->buffer_t[5]);
    robot_params[1] = (int16_t)(pusart->buffer_t[6]<<8 | pusart->buffer_t[7]);
    AKM_Kinematics_Init(robot_params);
}

static void AKM_Kinematics_Forward(int16_t *input, int16_t *output)
{
    static double  delta_count[2];
    static double  delta_v_ave[3];
    static int16_t recv_count[2];
    static int32_t wheel_mult[2] = {0};
    static int32_t current_count[2] = {0};

    recv_count[0] = input[0];
    recv_count[1] = input[1];
    
    // 编码器计数溢出处理
    for(int i=0;i<2;i++)
    {
        if(recv_count[i] < ENCODER_LOW_WRAP && current_count[i] > ENCODER_HIGH_WRAP)
            wheel_mult[i]++;
        else if(recv_count[i] > ENCODER_HIGH_WRAP && current_count[i] < ENCODER_LOW_WRAP)
            wheel_mult[i]--;
        else
            wheel_mult[i]=0;
    }

    // 将编码器数值转化为前进的距离 单位m
    for(int i=0;i<2;i++)
    {
        delta_count[i] = (double)(recv_count[i] + wheel_mult[i]*(ENCODER_MAX-ENCODER_MIN)-current_count[i])/ticks_per_meter;
        current_count[i] = recv_count[i];
    }

    // 计算底盘x轴变化距离m与Yaw轴朝向变化rad
    delta_v_ave[0] = (delta_count[0]+delta_count[1])/2.0;  
    delta_v_ave[1] = delta_v_ave[0]*tan(ANGLE_CAL((double)(TIM2->CCR1+(int16_t)(servo_bias*1.111)))) / ACKMAN_WHEEL_DISTANCE;

    // 积分计算 里程计坐标系(odom_frame)下的机器人X,Y,Yaw坐标
    output[0] += (int16_t)((delta_v_ave[0]*cos(((double)output[2]/1000))*1000));
    output[1] += (int16_t)((delta_v_ave[0]*sin(((double)output[2]/1000))*1000));
    output[2] += (int16_t)(delta_v_ave[1]*1000);

    if(output[2] > PI*1000)
        output[2] -= 2*PI*1000;
    else if(output[2] < -PI*1000)
        output[2] += 2*PI*1000;
    
    output[3] = (int16_t)(delta_v_ave[0]*1000);
    output[4] = 0;
    output[5] = (int16_t)(delta_v_ave[1]*1000);
}


static void AKM_Kinematics_Inverse(int16_t *input, int16_t *output)
{
    float x_speed = ((float)input[0])/1000;
    //弧度制转化为角度制
    float servo_angle = ((-(float)input[2]*180)/3141.5926);
    float wheel_velocity[2] = {0};
    
    servo_angle = servo_angle > SERVO_MAX  ?  SERVO_MAX : servo_angle;
    servo_angle = servo_angle < -SERVO_MAX ? -SERVO_MAX : servo_angle;

    x_speed = x_speed > SPEED_MAX  ?  SPEED_MAX : x_speed;
    x_speed = x_speed < -SPEED_MAX ? -SPEED_MAX : x_speed;

    float vel_bias = ACKMAN_WHEEL_TRACK*tan(servo_angle/180*3.1415)/2/ACKMAN_WHEEL_DISTANCE;

    wheel_velocity[0] = x_speed*(1 + vel_bias);
    wheel_velocity[1] = x_speed*(1 - vel_bias);
    
                                     //补偿
    float index = (ticks_per_meter + 0.05f)/PID_RATE;
    output[0] = (int16_t)(wheel_velocity[0] * index);
    output[1] = (int16_t)(wheel_velocity[1] * index);
    output[2] = servo_angle*10 - servo_bias;
}


#define PID_INTEGRAL_UP 1000

int16_t PID_MotorVelocityCtrlA(PID_Ctrl *PIDControl, int16_t error)
{
    static int16_t pwm_out;
    static int16_t bias_last=0, bias_2l=0;// bias_integral=0;
    int16_t bias = error;

    // bias_integral += bias;

    // if(bias_integral> PID_INTEGRAL_UP) bias_integral =  PID_INTEGRAL_UP;
    // if(bias_integral<-PID_INTEGRAL_UP) bias_integral = -PID_INTEGRAL_UP;

    pwm_out += (PIDControl->Kp*(bias - bias_last) + PIDControl->Ki*(bias)*0.1f \
              + PIDControl->Kd*(bias - 2*bias_last + bias_2l))*0.1f;
    
    bias_2l = bias_last;
    bias_last = bias;

    pwm_out = pwm_out > 2000 ? 2000 : pwm_out;
    pwm_out = pwm_out <-2000 ?-2000 : pwm_out;

    return pwm_out;
}

int16_t PID_MotorVelocityCtrlB(PID_Ctrl *PIDControl, int16_t error)
{
    static int16_t pwm_out;
    static int16_t bias_last=0, bias_2l=0;// bias_integral=0;
    int16_t bias = error;//spd_target - spd_current;

    // bias_integral += bias;

    // if(bias_integral> PID_INTEGRAL_UP) bias_integral =  PID_INTEGRAL_UP;
    // if(bias_integral<-PID_INTEGRAL_UP) bias_integral = -PID_INTEGRAL_UP;

    pwm_out += (PIDControl->Kp*(bias - bias_last) + PIDControl->Ki*(bias)*0.1f \
              + PIDControl->Kd*(bias - 2*bias_last + bias_2l))*0.1f;
    
    bias_2l = bias_last;
    bias_last = bias;

    pwm_out = pwm_out > 2000 ? 2000 : pwm_out;
    pwm_out = pwm_out <-2000 ?-2000 : pwm_out;

    return pwm_out;
}





/*
 *   AKM  左轮为A轮 右轮为B轮
 *        前为正    前为负
 */
void AKM_ROBOT_MoveCtrl(void)
{
    static int16_t encoder[4];
    int16_t servo = 0;
    // int16_t motor_pwm[2] = {0};

    // 编码器变化值
    encoder_delta[0] = (AKM_ENCODE_A_GetCount() - ENCODER_MID_VALUE);
    encoder_delta[1] =-(AKM_ENCODE_B_GetCount() - ENCODER_MID_VALUE);

    // 设置编码器中间值
    AKM_ENCODE_A_SetCounter(ENCODER_MID_VALUE);
    AKM_ENCODE_B_SetCounter(ENCODER_MID_VALUE);

    // 编码器累加值
    encoder[0] += encoder_delta[0];
    encoder[1] += encoder_delta[1];

    AKM_Kinematics_Forward(encoder, robot_odom);
    AKM_Kinematics_Inverse(robot_target_speed, encoder_delta_target);

    CtrlParam[0].PWM = PID_MotorVelocityCtrlA(&CtrlParam[0], (encoder_delta_target[0]-encoder_delta[0]));
    CtrlParam[1].PWM = PID_MotorVelocityCtrlB(&CtrlParam[1], (encoder_delta_target[1]-encoder_delta[1]));

    float floatdata[4] = {(float)encoder_delta[0], (float)encoder_delta[1], (float)encoder_delta_target[0], (float)encoder_delta_target[1]};
    Vofa_JustFloatSendDataAll(floatdata, 4);
    AKM_MOTOR_A_SpeedSet(-CtrlParam[0].PWM);
    AKM_MOTOR_B_SpeedSet(-CtrlParam[1].PWM);

    servo = 900 + encoder_delta_target[2];
    
    AKM_SERVO_SetAngle(servo);
}



