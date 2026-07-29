#include "ti_msp_dl_config.h"
#include "clock.h"
#include "oled.h"
#include "bno080.h"
#include "encoder.h"
#include "key.h"
#include "motor.h"
#include "infrared.h"
#include "blueSerial.h"

#include "pid.h"
#include "filter.h"
#include "trap_profile.h"

#define IR_MASK_STRAIGHT  0x3C  // 0b00111100, 直线: 中间4路(通道2~5)
#define IR_MASK_CURVE     0x7E  // 0b01111110, 弯道: 中间6路(通道1~6)
uint8_t ir_curMask = IR_MASK_STRAIGHT;

uint16_t distVal = 0;

uint8_t ir_data;
float baseSpeed = 0.0f;

uint32_t runTimer_ms = 0;    // 运行计时器(ms)
uint8_t  timerRunning = 0;   // 计时器运行标志
uint8_t  tracingEnabled = 0; // 寻迹使能: 0=关闭, 1=开启

float speed = 28.0f;
float speed2 = 20.0f;
float straightFactor = 0.35f;     // 直线PID削弱系数
uint8_t  curveDebounceCnt = 0;     // 弯道确认计数
uint8_t  curveDebounceThresh = 4;  // 弯道确认阈值(连续非直线次数)
uint16_t curveCount = 0;           // 进入弯道次数

ParsedData_t pkt;
LowPassFilter_t irFilter;
TrapProfile_t SpeedProfile;

PID_t leftMotorPID = {
    .Kp = 3.8f,
    .Ki = 0.25f,
    .Kd = 0.0f,

    .OutMax = 100.0f,
    .OutMin = -100.0f,

    .OutOffset = 0.0f,

    .ErrorIntMax = 250.0f,
    .ErrorIntMin = -250.0f,
};

PID_t rightMotorPID = {
    .Kp = 3.8f,
    .Ki = 0.25f,
    .Kd = 0.0f,

    .OutMax = 100.0f,
    .OutMin = -100.0f,

    .OutOffset = 0.0f,

    .ErrorIntMax = 250.0f,
    .ErrorIntMin = -250.0f,
};

PID_t linePID = {
    .Kp = 2.76f,            
    .Ki = 0.015f,
    .Kd = 1.3f,

    .OutMax = 100.0f,
    .OutMin = -100.0f,

    .OutOffset = 0.0f,

    .ErrorIntMax = 50.0f,
    .ErrorIntMin = -50.0f,

    .Target = 4.5f,
};

void keyProcess(void)
{
    if (Key_Check(GPIO_KEY_I_PIN, KEY_SINGLE)) {
        runTimer_ms = 0;
        timerRunning = 1;
        tracingEnabled = 1;
        curveCount = 0;
        TrapProfile_SpeedMode(&SpeedProfile, speed);
    }
    if (Key_Check(GPIO_KEY_II_PIN, KEY_SINGLE)) {
        runTimer_ms = 0;
        timerRunning = 1;
        tracingEnabled = 1;
        curveCount = 0;
        TrapProfile_SpeedMode(&SpeedProfile, speed2);
    }
    if (Key_Check(GPIO_KEY_IV_PIN, KEY_SINGLE)) {
        timerRunning = 0;
        tracingEnabled = 0;
        TrapProfile_SpeedMode(&SpeedProfile, 0.0f);
        leftMotorPID.Out = 0.0f;
        rightMotorPID.Out = 0.0f;
        linePID.Actual = 4.5f;
    }
}

void BSProcess(void)
{
    while (BlueSerial_ReadPacket(&pkt)) {
        if (strcmp(pkt.fields[0], "slider") == 0) {
            uint8_t val = atoi(pkt.fields[1]);
            if (val == 1) {
                linePID.Kp = atof(pkt.fields[2]);
            } else if (val == 2) {
                linePID.Ki = atof(pkt.fields[2]);
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
        else if (strcmp(pkt.fields[0], "key") == 0) {
            uint8_t keyval = atoi(pkt.fields[1]);
            if (keyval == 1){
                timerRunning = 0;
                tracingEnabled = 0;
                TrapProfile_SpeedMode(&SpeedProfile, 0.0f);
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
    // BlueSerial_Printf("[%s]", pkt.fields[0]);
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

    OLED_ShowBinNum(00, 40, ir_data, 8, OLED_6X8);
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
    BlueSerial_Init();

    PID_Init(&leftMotorPID);
    PID_Init(&rightMotorPID);
    PID_Init(&linePID);

    TrapProfile_Init(&SpeedProfile, 0.01f, 0, 10.0f, 100.0f);
    // 目标固定为黑线中心（8路灰度位置 1~8 的中点）
    linePID.Target = 4.5f;
    linePID.Actual = Infrared_Read_All(&ir_data, ir_curMask);
    linePID.Actual1 = linePID.Actual;
    LowPassFilter_Init(&irFilter, 0.68f, 0.1f, linePID.Actual);

    NVIC_EnableIRQ(TIMER_SYS_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_SYS_INST);
    NVIC_EnableIRQ(TIMER_1ms_INST_INT_IRQN);
    DL_TimerG_startCounter(TIMER_1ms_INST);

    while (1)
    {
        keyProcess();
        BSProcess();
        oledProcess();
    }
}

void TIMER_1ms_INST_IRQHandler(void)
{
   linePID.Actual = LowPassFilter_Update(&irFilter, Infrared_Read_All(&ir_data, ir_curMask));
   if (timerRunning) {
       runTimer_ms++;
   }
   // 统计gs_data中1的个数，超过3个认为脱线
   {
       uint8_t ones = ir_data;
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
        if (ir_data == 0x18) {
            curveDebounceCnt = 0;
        } else if (curveDebounceCnt == 0 && (ir_data == 0x00 || ir_data == 0x08 || ir_data == 0x10)) {
            curveDebounceCnt = 0;
        } else {
            if (curveDebounceCnt < curveDebounceThresh) {
                curveDebounceCnt++;
            }
        }
        if (curveDebounceCnt < curveDebounceThresh) {
            linePID.Out *= straightFactor;
            ir_curMask = IR_MASK_STRAIGHT;    // 直线/过渡期用4路
        } else if (curveDebounceCnt == curveDebounceThresh) {
            curveCount++;
            curveDebounceCnt++;
            ir_curMask = IR_MASK_CURVE;       // 确认弯道，切6路
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
}