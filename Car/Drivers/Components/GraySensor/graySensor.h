#ifndef __GRAY_SENSOR_H
#define __GRAY_SENSOR_H

#include "ti_msp_dl_config.h"

void _select_channel(uint8_t channel);
_Bool Read_OUT_value(void);
float Gray_Sensor_Read_All(uint8_t* sensor_value);

#endif
