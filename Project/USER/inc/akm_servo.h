#ifndef  __AKM_SERVO_H__
#define  __AKM_SERVO_H__

#include <stdint.h>

#define SERVO_MAX   45      // 50

void AKM_Servo_Init(void);
void AKM_SERVO_SetAngle(uint16_t angle);

#endif
