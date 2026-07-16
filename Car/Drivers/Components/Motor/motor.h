#ifndef __MOTOR_H
#define __MOTOR_H

#include "ti_msp_dl_config.h"

void Motor_Init(void);
void Set_Duty(uint8_t duty, uint8_t channel);
void Load(int8_t motorR, int8_t motorL);

#endif
