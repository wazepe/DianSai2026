#ifndef __GRAY_SENSOR_H
#define __GRAY_SENSOR_H

#include "ti_msp_dl_config.h"

void Gray_Sensor_Init(void);
void Gray_Sensor_RequestDigitalMode(void);
void Gray_Sensor_IRQHandler(uint8_t rxData);
uint8_t Gray_Sensor_GetRaw(void);
uint8_t Gray_Sensor_HasNewFrame(void);
float Gray_Sensor_Read_All(uint8_t *sensor_value, uint8_t mask);

#endif
