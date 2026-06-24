#include "ti_msp_dl_config.h"
#include "clock.h"
#include "oled.h"
#include "bno080.h"
#include "encoder.h"
#include "key.h"
#include "motor.h"
#include "ultrasonic.h"
#include "graySensor.h"
#include "blueSerial.h"

int16_t leftEncoderValue = 0, rightEncoderValue = 0;
int8_t leftDuty, rightDuty;

uint8_t keyNum;
uint16_t distVal = 0;

uint8_t gs_data;
float gs_value;666

void EncoderProcess(void)
{
    leftEncoderValue = Encoder_GetCount(LEFT_ENCODER);
    rightEncoderValue = Encoder_GetCount(RIGHT_ENCODER);
}

void keyProcess(void)
{
    if (Key_Check(GPIO_KEY_I_PIN, KEY_SINGLE)) {
        keyNum = 1;
        leftDuty = 10;
        rightDuty = -10;
    }
    if (Key_Check(GPIO_KEY_II_PIN, KEY_SINGLE)) {
        keyNum = 2;
        leftDuty = -10;
        rightDuty = 10;
    }
    if (Key_Check(GPIO_KEY_III_PIN, KEY_SINGLE)) {
        keyNum = 3;
    }
    if (Key_Check(GPIO_KEY_IV_PIN, KEY_SINGLE)) {
        keyNum = 4;
        leftDuty = 0;
        rightDuty = 0;
    }
}

void motorProcess(void)
{
    static int8_t pLeftDuty, pRightDuty;

    // 检查是否有任何变化
    if (pLeftDuty != leftDuty || pRightDuty != rightDuty) {
        pLeftDuty = leftDuty;
        pRightDuty = rightDuty;
        Load(pLeftDuty, pRightDuty);
    }
}

void ultrasonicProcess(void)
{
    distVal = Read_Ultrasonic();
}

void GSProcess(void)
{
    gs_value = Gray_Sensor_Read_All(&gs_data);
}

void oledProcess(void)
{
    OLED_Printf(00, 00, OLED_6X8, "Yaw:%+07.2f", bno08x_data.yaw);

    OLED_Printf(00, 10, OLED_6X8, "LE:%+06d", leftEncoderValue);
    OLED_Printf(64, 10, OLED_6X8, "RE:%+06d", rightEncoderValue);
    OLED_Printf(00, 20, OLED_6X8, "LDuty:%+03d", leftDuty);
    OLED_Printf(64, 20, OLED_6X8, "RDuty:%+03d", rightDuty);

    OLED_Printf(00, 30, OLED_6X8, "KEY%0d", keyNum);

    OLED_Printf(64, 30, OLED_6X8, "D:%04d", distVal);

    OLED_ShowBinNum(00, 40, gs_data, 8, OLED_6X8);
    OLED_Printf(00, 40, OLED_6X8, "GS:%+5.1f", gs_value);
    
    // 主循环是否刷新
    OLED_ShowNum(92, 56, g_sysTick_1ms_u32, 6, OLED_6X8);
    OLED_Update();
}

int main(void)
{
    delay_cycles(CPUCLK_FREQ / 20);  // 延时50ms再初始化各个外设

    SYSCFG_DL_init();

    OLED_Init();
    BNO08X_Init();
    Encoder_Init();
    Motor_Init();
    Ultrasonic_Init();
    BlueSerial_Init();

    BlueSerial_Printf("MSPM0G3519_Init_OK");
    
    while (1)
    {
        EncoderProcess();
        keyProcess();
        motorProcess();
        ultrasonicProcess();
        GSProcess();
        oledProcess();
    }
}
