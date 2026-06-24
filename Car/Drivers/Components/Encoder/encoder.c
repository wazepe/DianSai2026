#include "encoder.h"

void Encoder_Init(void)
{
    DL_TimerG_startCounter(QEI_L_INST);
    DL_TimerG_startCounter(QEI_R_INST);
}

// 获取编码器计数函数
int16_t Encoder_GetCount(EncoderChannel ChannelX)
{
    if (ChannelX == LEFT_ENCODER) {
        return (int16_t)DL_TimerG_getTimerCount(QEI_L_INST);
    } else if (ChannelX == RIGHT_ENCODER) {
        return (int16_t)DL_TimerG_getTimerCount(QEI_R_INST) * -1;
    }
    return 0;
}
