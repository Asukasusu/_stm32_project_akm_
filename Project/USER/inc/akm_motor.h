#ifndef  __AKM_MOTOR_H__
#define  __AKM_MOTOR_H__

#include <stdint.h>

void AKM_MOTOR_Init(uint8_t freq);
void AKM_MOTOR_A_SpeedSet(int16_t speed);
void AKM_MOTOR_B_SpeedSet(int16_t speed);

#endif
