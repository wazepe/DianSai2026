#ifndef __ENCODER_H
#define __ENCODER_H

#include "ti_msp_dl_config.h"

typedef enum {
    LEFT_ENCODER,
    RIGHT_ENCODER,
} EncoderChannel;

// 函数声明
void Encoder_Init(void);
int16_t Encoder_GetCount(EncoderChannel ChannelX);

#endif
