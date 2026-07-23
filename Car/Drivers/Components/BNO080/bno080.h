#ifndef _BNO08X_UART_RVC_H_
#define _BNO08X_UART_RVC_H_

#include "ti_msp_dl_config.h"

typedef struct {
    uint8_t index;
    float pitch;
    float roll;
    float yaw;
    int16_t ax;
    int16_t ay;
    int16_t az;
} BNO08X_Data_t;

extern BNO08X_Data_t bno08x_data;

void BNO08X_Init(void);

#endif  /* #ifndef _BNO08X_UART_RVC_H_ */
