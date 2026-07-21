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
uint8_t keyNum;
uint16_t distVal = 0;

uint8_t gs_data;
float baseSpeed = 0.0f;
uint8_t lostLineTimes = 0;  //丢线次数计数器

typedef enum {
    MODE_FOLLOW,    // 正常巡线
    MODE_LOST_LINE, // 无黑线
} CarMode_t;

CarMode_t carMode = MODE_LOST_LINE;

#define LOST_LINE_THRESHOLD  5   // 连续5次无黑线才判定为丢线
#define FOLLOW_THRESHOLD     3   // 连续3次有黑线才恢复巡线

#define ANGLE_PID_DEADBAND   0.1f // 角度环输入死区阈值：角度误差在此范围内时 PID 不输出

uint8_t lostLineCnt = 0;
uint8_t followCnt = 0;


ParsedData_t pkt;
LowPassFilter_t grayFilter;
TrapProfile_t AngleProfile;

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

    .OutMax = 100.0f,
    .OutMin = -100.0f,

    .OutOffset = 0.0f,

    .ErrorIntMax = 50.0f,
    .ErrorIntMin = -50.0f,

    .Target = 4.5f,
};

PID_t AnglePID = {
    .Kp = 1.0f,     
    .Ki = 0.0f,     
    .Kd = 0.5f,     

    .OutMax = 30.0f,   
    .OutMin = -30.0f,

    .ErrorIntMax = 200.0f,
    .ErrorIntMin = -200.0f,

    .OutOffset = 0.0f,
};

Gimbal_t gimbal;
float target_yaw = 0.0f;
float target_pitch = 0.0f;
int16_t speed = 200;         // RPM

void keyProcess(void)
{
    if (Key_Check(GPIO_KEY_I_PIN, KEY_SINGLE)) {
        keyNum = 1;
        baseSpeed = 10.0f;
        TrapProfile_SetTarget(&AngleProfile, 38.0f);
    }
    if (Key_Check(GPIO_KEY_II_PIN, KEY_SINGLE)) {
        keyNum = 2;
        baseSpeed = 20.0f;
        TrapProfile_SetTarget(&AngleProfile, 38.5f);
    }
    if (Key_Check(GPIO_KEY_III_PIN, KEY_SINGLE)) {
        keyNum = 3;
        baseSpeed = 30.0f;
        TrapProfile_SetTarget(&AngleProfile, 39.5f);
    }
    if (Key_Check(GPIO_KEY_IV_PIN, KEY_SINGLE)) {
        keyNum = 4;
        baseSpeed = 0.0f;
        leftMotorPID.Out = 0.0f;
        rightMotorPID.Out = 0.0f;
        linePID.Actual = 4.5f;
        lostLineTimes = 0;
    }
}


void ultrasonicProcess(void)
{
    distVal = Read_Ultrasonic();
}

void BSProcess(void)
{
    if (BlueSerial_IsPacketReady()) {
        if (BlueSerial_ParsePacket(&pkt)) {
            if (strcmp(pkt.fields[0], "slider") == 0) {
                uint8_t val = atoi(pkt.fields[1]);
                if (val == 1) {
                    AnglePID.Kp = atof(pkt.fields[2]);
                    // target_pitch = atof(pkt.fields[2]);
                } else if (val == 2) {
                    AnglePID.Ki = atof(pkt.fields[2]);
                    // target_yaw = atof(pkt.fields[2]);
                } else if (val == 3) {
                    AnglePID.Kd = atof(pkt.fields[2]);
                } else if (val == 4) {
                    grayFilter.alpha = atof(pkt.fields[2]);
                } 
            }
            if (strcmp(pkt.fields[0], "key") == 0) {
                uint8_t keyval = atoi(pkt.fields[1]);
                if (keyval == 1){
                    baseSpeed = 0.0f;
                    leftMotorPID.Out = 0.0f;
                    rightMotorPID.Out = 0.0f;
                    linePID.Actual = 4.5f;
                    lostLineTimes = 0;
                }
                if (keyval == 2){
                    TrapProfile_SetTarget(&AngleProfile, 180);
                }
                if (keyval == 3){
                    TrapProfile_SetTarget(&AngleProfile, 0);
                }
            }
        }
    }

    BlueSerial_Printf("[plot,%f,%f]", linePID.Target, linePID.Actual);
}

void oledProcess(void)
{
    OLED_Printf(00, 00, OLED_6X8, "Yaw:%+07.2f", bno08x_data.yaw);

    OLED_Printf(00, 10, OLED_6X8, "kp:%04.2f", AnglePID.Kp);
    OLED_Printf(64, 10, OLED_6X8, "ki:%04.3f", AnglePID.Ki);
    OLED_Printf(74, 00, OLED_6X8, "kd:%04.2f", AnglePID.Kd);

    // OLED_Printf(64, 30, OLED_6X8, "D:%04d", distVal);

    OLED_ShowBinNum(64, 30, gs_data, 8, OLED_6X8);
    OLED_Printf(00, 40, OLED_6X8, "Tar:%5.2f", AnglePID.Target);
    OLED_Printf(64, 40, OLED_6X8, "Act:%5.2f", AnglePID.Actual);

    OLED_Printf(00, 20, OLED_6X8, "time:%d", lostLineTimes);
    OLED_Printf(64, 20, OLED_6X8, "%s", 
        carMode == MODE_FOLLOW ? "FOLLOW" : "LOST");

    OLED_Update();
}

float AngleErrorNormalize(float target, float actual)
{
    float err = target - actual;
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
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

    LowPassFilter_Init(&grayFilter, 0.58f, 0.1f);
    PID_Init(&AnglePID);
    TrapProfile_Init(&AngleProfile,0.01f,180.0f,360.0f,360.0f);
    // 目标固定为黑线中心（8路灰度位置 1~8 的中点）
    linePID.Target = 4.5f;
    linePID.Actual = Gray_Sensor_Read_All(&gs_data);
    linePID.Actual1 = linePID.Actual;

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
   linePID.Actual = LowPassFilter_Update(&grayFilter,Gray_Sensor_Read_All(&gs_data));
}

void TIMER_SYS_INST_IRQHandler(void)
{
     // 获取实际值
    leftMotorPID.Actual = Encoder_GetCount(LEFT_ENCODER);
    rightMotorPID.Actual = Encoder_GetCount(RIGHT_ENCODER);

    AnglePID.Actual = bno08x_data.yaw;

    // 状态转移（带消抖）
    if (gs_data == 0) {
        lostLineCnt++;
        followCnt = 0;
        if (lostLineCnt >= LOST_LINE_THRESHOLD) {
            if (carMode == MODE_FOLLOW) {  // 只在首次进入丢线时设置
                lostLineTimes++;
                if (lostLineTimes % 2 == 1) {
                // 奇数次
                    TrapProfile_Reset(&AngleProfile, bno08x_data.yaw);
                    TrapProfile_SetTarget(&AngleProfile, 142.0f);
                } else {
                    TrapProfile_Reset(&AngleProfile, bno08x_data.yaw);
                    TrapProfile_SetTarget(&AngleProfile, 38.0f);
                }
            }
            carMode = MODE_LOST_LINE;
        }
    } else {
        followCnt++;
        lostLineCnt = 0;
        if (followCnt >= FOLLOW_THRESHOLD) {
            carMode = MODE_FOLLOW;
        }
    }

    // 状态执行
    switch (carMode) {
        case MODE_LOST_LINE: {
            // 无黑线
            AnglePID.Target = TrapProfile_Update(&AngleProfile);
            float err = AngleErrorNormalize(AnglePID.Target, bno08x_data.yaw);
            AnglePID.Actual = AnglePID.Target - err;

            if (err < ANGLE_PID_DEADBAND && err > -ANGLE_PID_DEADBAND) {
                // 角度误差在死区内，PID 不参与调控，输出清零
                AnglePID.Out = 0.0f;
                AnglePID.ErrorInt = 0.0f;
            } else {
                PID_Update(&AnglePID);
            }

            leftMotorPID.Target = baseSpeed + AnglePID.Out;
            rightMotorPID.Target = baseSpeed - AnglePID.Out;
            break;
        }

        case MODE_FOLLOW:
        default:
            // 有黑线：寻迹
            PID_Update(&linePID);
            leftMotorPID.Target  = baseSpeed - linePID.Out;
            rightMotorPID.Target = baseSpeed + linePID.Out;
            break;
    }

    // 添加限幅保护
    if (leftMotorPID.Target > 50.0f) leftMotorPID.Target = 50.0f;
    if (leftMotorPID.Target < -50.0f) leftMotorPID.Target = -50.0f;

    if (rightMotorPID.Target > 50.0f) rightMotorPID.Target = 50.0f;
    if (rightMotorPID.Target < -50.0f) rightMotorPID.Target = -50.0f;
    
    PID_Update(&leftMotorPID);
    PID_Update(&rightMotorPID);

    Load(leftMotorPID.Out, rightMotorPID.Out);

    // Gimbal_SetTarget(&gimbal, target_yaw, target_pitch, speed);
}
