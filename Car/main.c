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
#include "pid.h"

uint8_t keyNum, flag;
uint16_t distVal = 0;

uint8_t gs_data;
float gs_value;

ParsedData_t pkt;

PID_t leftMotorPID = {
    .Kp = 3.8f,
    .Ki = 0.25f,
    .Kd = 0.0f,

    .OutMax = 100.0f,
    .OutMin = -100.0f,

    .OutOffset = 0.0f,

    .ErrorIntMax = 800.0f,
    .ErrorIntMin = -800.0f,
};

PID_t rightMotorPID = {
    .Kp = 3.8f,
    .Ki = 0.25f,
    .Kd = 0.0f,

    .OutMax = 100.0f,
    .OutMin = -100.0f,

    .OutOffset = 0.0f,

    .ErrorIntMax = 800.0f,
    .ErrorIntMin = -800.0f,
};

void keyProcess(void)
{
    if (Key_Check(GPIO_KEY_I_PIN, KEY_SINGLE)) {
        keyNum = 1;
    }
    if (Key_Check(GPIO_KEY_II_PIN, KEY_SINGLE)) {
        keyNum = 2;
    }
    if (Key_Check(GPIO_KEY_III_PIN, KEY_SINGLE)) {
        keyNum = 3;
    }
    if (Key_Check(GPIO_KEY_IV_PIN, KEY_SINGLE)) {
        keyNum = 4;
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

void BSProcess(void)
{
    if (BlueSerial_IsPacketReady()) {
        if (BlueSerial_ParsePacket(&pkt)) {
            if (strcmp(pkt.fields[0], "slider") == 0) {
                uint8_t val = atoi(pkt.fields[1]);
                if (val == 4) {
                    leftMotorPID.Target = atof(pkt.fields[2]);
                    rightMotorPID.Target = leftMotorPID.Target;
                }
            }
        }
    }

    BlueSerial_Printf("[plot,%f,%f]", leftMotorPID.Actual, rightMotorPID.Actual);
}

void oledProcess(void)
{
    OLED_Printf(00, 00, OLED_6X8, "Yaw:%+07.2f", bno08x_data.yaw);

    OLED_Printf(00, 10, OLED_6X8, "LE:%+06.0f", leftMotorPID.Actual);
    OLED_Printf(64, 10, OLED_6X8, "RE:%+06.0f", rightMotorPID.Actual);

    OLED_Printf(00, 30, OLED_6X8, "KEY%0d", keyNum);

    OLED_Printf(64, 30, OLED_6X8, "D:%04d", distVal);

    OLED_ShowBinNum(00, 40, gs_data, 8, OLED_6X8);
    OLED_Printf(64, 40, OLED_6X8, "GS:%+5.1f", gs_value);
    
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

    PID_Init(&leftMotorPID);
    PID_Init(&rightMotorPID);

    NVIC_EnableIRQ(TIMER_SYS_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_SYS_INST);
    
    while (1)
    {
        keyProcess();
        ultrasonicProcess();
        GSProcess();
        BSProcess();
        oledProcess();
    }
}

void TIMER_SYS_INST_IRQHandler(void)
{
    // 获取实际值
    leftMotorPID.Actual = Encoder_GetCount(LEFT_ENCODER);
    rightMotorPID.Actual = Encoder_GetCount(RIGHT_ENCODER);

    PID_Update(&leftMotorPID);
    PID_Update(&rightMotorPID);

    Load(leftMotorPID.Out, rightMotorPID.Out);
}
