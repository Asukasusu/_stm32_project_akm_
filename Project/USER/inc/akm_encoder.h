#ifndef __AKM_ENCODER_H__
#define __AKM_ENCODER_H__

#include <stdint.h>

#define ENCODER_MID_VALUE       (30000)

#define ENCODER_MAX             (32767)
#define ENCODER_MIN             (-32768)
#define SPEED_MAX               (2)
#define ACKMAN_WHEEL_DISTANCE   (0.185)
#define ACKMAN_WHEEL_TRACK      (0.165)
#define WHEEL_DIAMETER          (0.0634)
#define ENCODER_RESOLUTION      (880)
#define PID_RATE                (50)

void AKM_ENCODER_Init(uint16_t cycle);
uint16_t AKM_ENCODE_A_GetCount(void);
uint16_t AKM_ENCODE_B_GetCount(void);

void AKM_ENCODE_A_SetCounter(uint16_t count);
void AKM_ENCODE_B_SetCounter(uint16_t count);

#endif
