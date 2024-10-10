#ifndef __AKM_VOLT_H__
#define __AKM_VOLT_H__

#include <stdint.h>

#define   VBAT_VOL_CHG    1050
#define   VBAT_VOL_OFF    990

void AKM_Volt_Init(void);
// uint16_t AKM_GetVolt_X100(void);
void AKM_ROBOT_BatteryDetect(void);

#endif
