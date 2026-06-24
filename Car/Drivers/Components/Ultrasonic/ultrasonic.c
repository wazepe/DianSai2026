#include "ultrasonic.h"

void Ultrasonic_Init(void)
{
    DL_TimerG_startCounter(PWM_ULTRASONIC_INST);
}

uint16_t Read_Ultrasonic(void)
{
    uint16_t distVal = DL_Timer_getCaptureCompareValue(CAPTURE_ULTRASONIC_INST, CAPTURE_ULTRASONIC_CC_IDX) * 0.17;

    if(distVal > 6000)
        distVal = 0;

    return distVal;
}