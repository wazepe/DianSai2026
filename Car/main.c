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
#include "lqkj.h"
#include "pid.h"
#include "filter.h"
#include "trap_profile.h"

#define GS_MASK 0x3C  // 0b00111100, 只取中间4路(通道2~5)计算位置

uint8_t keyNum;
uint16_t distVal = 0;

uint8_t gs_data;
float baseSpeed = 0.0f;

uint32_t runTimer_ms = 0;    // 运行计时器(ms)
uint8_t  timerRunning = 0;   // 计时器运行标志
uint8_t  tracingEnabled = 0; // 寻迹使能: 0=关闭, 1=开启

float speed = 20.0f;
float speed2 = 15.0f;
float straightFactor = 0.35f;     // 直线PID削弱系数
uint8_t  curveDebounceCnt = 0;     // 弯道确认计数
uint8_t  curveDebounceThresh = 4;  // 弯道确认阈值(连续非直线次数)
uint16_t curveCount = 0;           // 进入弯道次数

ParsedData_t pkt;
LowPassFilter_t grayFilter;
TrapProfile_t SpeedProfile;

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

PID_t linePID = {
    .Kp = 2.76f,            
    .Ki = 0.015f,
    .Kd = 1.3f,

    // .Kp = 2.50f,            
    // .Ki = 0.01f,
    // .Kd = 1.2f,

    .OutMax = 100.0f,
    .OutMin = -100.0f,

    .OutOffset = 0.0f,

    .ErrorIntMax = 50.0f,
    .ErrorIntMin = -50.0f,

    .Target = 4.5f,
};

Gimbal_t gimbal;
float target_yaw = 0.0f;
float target_pitch = 0.0f;

void keyProcess(void)
{
    if (Key_Check(GPIO_KEY_I_PIN, KEY_SINGLE)) {
        keyNum = 1;
        runTimer_ms = 0;
        timerRunning = 1;
        tracingEnabled = 1;
        curveCount = 0;
        TrapProfile_SpeedMode(&SpeedProfile, speed);
    }
    if (Key_Check(GPIO_KEY_II_PIN, KEY_SINGLE)) {
        keyNum = 2;
        runTimer_ms = 0;
        timerRunning = 1;
        tracingEnabled = 1;
        curveCount = 0;
        TrapProfile_SpeedMode(&SpeedProfile, speed2);
    }
    // if (Key_Check(GPIO_KEY_III_PIN, KEY_SINGLE)) {
    //     keyNum = 3;
    //     runTimer_ms = 0;
    //     timerRunning = 1;
    //     tracingEnabled = 1;
    //     TrapProfile_SpeedMode(&SpeedProfile, 30.0f);
    // }
    if (Key_Check(GPIO_KEY_IV_PIN, KEY_SINGLE)) {
        keyNum = 4;
        timerRunning = 0;
        tracingEnabled = 0;
        TrapProfile_SpeedMode(&SpeedProfile, 0.0f);  // 平滑到0
        leftMotorPID.Out = 0.0f;
        rightMotorPID.Out = 0.0f;
        linePID.Actual = 4.5f;
    }
}


void ultrasonicProcess(void)
{
    distVal = Read_Ultrasonic();
}

void BSProcess(void)
{
    // while (BlueSerial_ReadPacket(&pkt)) {
    //     if (strcmp(pkt.fields[0], "slider") == 0) {
    //         uint8_t val = atoi(pkt.fields[1]);
    //         if (val == 1) {
    //             linePID.Kp = atof(pkt.fields[2]);
    //         } else if (val == 2) {
    //             linePID.Ki = atof(pkt.fields[2]);
    //         } else if (val == 3) {
    //             linePID.Kd = atof(pkt.fields[2]);
    //         } else if (val == 4) {
    //             grayFilter.alpha = atof(pkt.fields[2]);
    //         }
    //     }
    //     if (strcmp(pkt.fields[0], "key") == 0) {
    //         uint8_t keyval = atoi(pkt.fields[1]);
    //         if (keyval == 1){
    //             baseSpeed = 0.0f;
    //             leftMotorPID.Target = 0.0f;
    //             rightMotorPID.Target = 0.0f;
    //             leftMotorPID.Out = 0.0f;
    //             rightMotorPID.Out = 0.0f;
    //             linePID.Actual = 4.5f;
    //         }
    //     }
    // }
    // BlueSerial_Printf("[plot,%f,%f]", linePID.Target, linePID.Actual);
    while (BlueSerial_ReadPacket(&pkt)) {
        BlueSerial_Printf("[%s]", pkt.fields[0]);
        if (strcmp(pkt.fields[0], "slider") == 0) {
            uint8_t val = atoi(pkt.fields[1]);
            if (val == 1) {
                linePID.Kp = atof(pkt.fields[2]);
                // target_pitch = atof(pkt.fields[2]);
            } else if (val == 2) {
                linePID.Ki = atof(pkt.fields[2]);
                // target_yaw = atof(pkt.fields[2]);
            } else if (val == 3) {
                linePID.Kd = atof(pkt.fields[2]);
            } else if (val == 4) {
                speed = atof(pkt.fields[2]);
            } else if (val == 5) {
                straightFactor = atof(pkt.fields[2]);
            } else if (val == 6) {
                curveDebounceThresh = (uint8_t)atoi(pkt.fields[2]);
            }

        }
        if (strcmp(pkt.fields[0], "key") == 0) {
            uint8_t keyval = atoi(pkt.fields[1]);
            if (keyval == 1){
                timerRunning = 0;
                tracingEnabled = 0;
                TrapProfile_SpeedMode(&SpeedProfile, 0.0f);  // ← 加上这行
                leftMotorPID.Out = 0.0f;
                rightMotorPID.Out = 0.0f;
                linePID.Actual = 4.5f;
            }
            if (keyval == 2){
                runTimer_ms = 0;
                timerRunning = 1;
                tracingEnabled = 1;
                TrapProfile_SpeedMode(&SpeedProfile, speed);
            }
        }
    }
    
    
}

void oledProcess(void)
{
    OLED_Printf(00, 0, OLED_6X8, "Spd:%3.1f", speed);
    OLED_Printf(64, 0, OLED_6X8, "Spd2:%3.1f", speed2);

    OLED_Printf(00, 10, OLED_6X8, "Lkp:%5.3f", linePID.Kp);
    OLED_Printf(64, 10, OLED_6X8, "Lki:%5.3f", linePID.Ki);

    OLED_Printf(00, 20, OLED_6X8, "Lkd:%5.3f", linePID.Kd);
    OLED_Printf(64, 20, OLED_6X8, "SF:%4.2f", straightFactor);

    // 显示运行时间 (秒, 保留1位小数)
    OLED_Printf(00, 30, OLED_6X8, "Time:%5.1fs", runTimer_ms / 1000.0f);
    OLED_Printf(64, 30, OLED_6X8, "%s", tracingEnabled ? "TRC:ON" : "TRC:OFF");

    OLED_ShowBinNum(00, 40, gs_data, 8, OLED_6X8);
    OLED_Printf(64, 40, OLED_6X8, "Crv:%d", curveCount);

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

    // Gimbal_Init(&gimbal);

    // // 上电初始化
    // bool init_done = false;
    // while (!init_done) {
    //     init_done = Gimbal_PowerOnInit(&gimbal, g_sysTick_1ms_u32);
    //     Gimbal_Poll(&gimbal, g_sysTick_1ms_u32);
    // }

    PID_Init(&leftMotorPID);
    PID_Init(&rightMotorPID);
    PID_Init(&linePID);

    TrapProfile_Init(&SpeedProfile, 0.01f, 0, 10.0f, 100.0f);
    // 目标固定为黑线中心（8路灰度位置 1~8 的中点）
    linePID.Target = 4.5f;
    linePID.Actual = Gray_Sensor_Read_All(&gs_data, GS_MASK);
    linePID.Actual1 = linePID.Actual;
    LowPassFilter_Init(&grayFilter, 0.7f, 0.1f, linePID.Actual);

    NVIC_EnableIRQ(TIMER_SYS_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_SYS_INST);
    NVIC_EnableIRQ(TIMER_1ms_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_1ms_INST);

    while (1)
    {
        keyProcess();
        ultrasonicProcess();
        BSProcess();
        oledProcess();
    }
}

void TIMER_1ms_INST_IRQHandler(void)
{
   linePID.Actual = LowPassFilter_Update(&grayFilter,Gray_Sensor_Read_All(&gs_data, GS_MASK));
   if (timerRunning) {
       runTimer_ms++;
   }
   // 统计gs_data中1的个数，超过3个认为脱线
   {
       uint8_t ones = gs_data;
       ones = (ones & 0x55) + ((ones >> 1) & 0x55);
       ones = (ones & 0x33) + ((ones >> 2) & 0x33);
       ones = (ones & 0x0F) + ((ones >> 4) & 0x0F);
       if (tracingEnabled && ones > 3) {
           tracingEnabled = 0;
           timerRunning = 0;
           TrapProfile_SpeedMode(&SpeedProfile, 0.0f);
           leftMotorPID.Out = 0.0f;
           rightMotorPID.Out = 0.0f;
           linePID.Actual = 4.5f;
       }
   }
}

void TIMER_SYS_INST_IRQHandler(void)
{
     // 获取实际值
    leftMotorPID.Actual = Encoder_GetCount(LEFT_ENCODER);
    rightMotorPID.Actual = Encoder_GetCount(RIGHT_ENCODER);
    baseSpeed = TrapProfile_SpeedUpdate(&SpeedProfile);

    if (tracingEnabled) {
        // 寻迹
        PID_Update(&linePID);

        // 直线检测：仅中间两路(bit3,bit4)有信号时，减弱调控防止抖动
        // 弯道恢复需连续多次确认，过滤偶发噪声
        // 仅 00011000 进入直线；已在直线时 0x00/0x08/0x10/0x18 保持
        if (gs_data == 0x18) {
            curveDebounceCnt = 0;
        } else if (curveDebounceCnt == 0 && (gs_data == 0x00 || gs_data == 0x08 || gs_data == 0x10)) {
            curveDebounceCnt = 0;
        } else {
            if (curveDebounceCnt < curveDebounceThresh) {
                curveDebounceCnt++;
            }
        }
        if (curveDebounceCnt < curveDebounceThresh) {
            linePID.Out *= straightFactor;
        } else if (curveDebounceCnt == curveDebounceThresh) {
            curveCount++;  // 首次确认弯道，计数+1
            curveDebounceCnt++;  // 防止重复触发
        }

        leftMotorPID.Target  = baseSpeed - linePID.Out;
        rightMotorPID.Target = baseSpeed + linePID.Out;
    } else {
        leftMotorPID.Target  = 0.0f;
        rightMotorPID.Target = 0.0f;
    }

    // 添加限幅保护
    if (leftMotorPID.Target > 60.0f) leftMotorPID.Target = 60.0f;
    if (leftMotorPID.Target < -60.0f) leftMotorPID.Target = -60.0f;

    if (rightMotorPID.Target > 60.0f) rightMotorPID.Target = 60.0f;
    if (rightMotorPID.Target < -60.0f) rightMotorPID.Target = -60.0f;

    PID_Update(&leftMotorPID);
    PID_Update(&rightMotorPID);

    Load(leftMotorPID.Out, rightMotorPID.Out);

    // Gimbal_SetTarget(&gimbal, target_yaw, target_pitch, speed);
}
