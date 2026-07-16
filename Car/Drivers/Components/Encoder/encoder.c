#include "encoder.h"

void Encoder_Init(void)
{
    DL_TimerG_startCounter(QEI_L_INST);
    DL_TimerG_startCounter(QEI_R_INST);
}

// 获取编码器计数函数
int16_t Encoder_GetCount(EncoderChannel ChannelX)
{
    int16_t count = 0;
    if (ChannelX == LEFT_ENCODER) {
        count = (int16_t)DL_TimerG_getTimerCount(QEI_L_INST);
        DL_TimerG_setTimerCount(QEI_L_INST, 0);
    } else if (ChannelX == RIGHT_ENCODER) {
        count = (int16_t)DL_TimerG_getTimerCount(QEI_R_INST) * -1;
        DL_TimerG_setTimerCount(QEI_R_INST, 0);
    }
    return count;
}
