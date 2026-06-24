#include "motor.h"
#include <stdlib.h>

uint32_t period = 8000;

void Motor_Init(void)
{
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, 0, DL_TIMER_CC_0_INDEX);
    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, 0, DL_TIMER_CC_1_INDEX);

    DL_TimerG_startCounter(PWM_Motor_INST);

    DL_GPIO_clearPins(GPIO_DIR_PORT, GPIO_DIR_A1_PIN | GPIO_DIR_A2_PIN
                                    | GPIO_DIR_B1_PIN | GPIO_DIR_B2_PIN);
}

void Set_Duty(uint8_t duty, uint8_t channel)
{
    uint32_t CompareValue;
    CompareValue = (uint32_t)(period * duty * 0.01f);

    if (channel == 0) {
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, CompareValue, DL_TIMER_CC_1_INDEX);
    } else if (channel == 1) {
        DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, CompareValue, DL_TIMER_CC_0_INDEX);
    }
}

void Load(int8_t motorL, int8_t motorR)         //-100.0~100.0
{
    if(motorL >= 0) {
        DL_GPIO_clearPins(GPIO_DIR_PORT, GPIO_DIR_A2_PIN);
        DL_GPIO_setPins(GPIO_DIR_PORT, GPIO_DIR_A1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_DIR_PORT, GPIO_DIR_A1_PIN);
        DL_GPIO_setPins(GPIO_DIR_PORT, GPIO_DIR_A2_PIN);
    }
    Set_Duty(abs(motorL), 0);

    if(motorR >= 0) {
        DL_GPIO_clearPins(GPIO_DIR_PORT, GPIO_DIR_B2_PIN);
        DL_GPIO_setPins(GPIO_DIR_PORT, GPIO_DIR_B1_PIN);
    } else {
        DL_GPIO_clearPins(GPIO_DIR_PORT, GPIO_DIR_B1_PIN);
        DL_GPIO_setPins(GPIO_DIR_PORT, GPIO_DIR_B2_PIN);
    }
    Set_Duty(abs(motorR), 1);
}
