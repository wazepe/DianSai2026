#include "lqkj.h"
#include <string.h>
#include <math.h>

/* ============================================================================
 *  UART 发送
 * ============================================================================ */

void UART_YT_SendData(const uint8_t *data, uint8_t len)
{
    uint32_t sent = 0;
    while (sent < len) {
        uint32_t filled = DL_UART_Main_fillTXFIFO(UART_YT_INST, data + sent, len - sent);
        sent += filled;
        if (sent < len) {
            while (DL_UART_Main_isTXFIFOFull(UART_YT_INST));
        }
    }
    while (DL_UART_Main_isBusy(UART_YT_INST));
}

/* ============================================================================
 *  通用打包工具函数
 * ============================================================================ */

static inline void pack_u16(uint8_t *buf, uint16_t val)
{
    buf[0] = (uint8_t)(val >> 8);
    buf[1] = (uint8_t)(val & 0xFF);
}

static inline void pack_s16(uint8_t *buf, int16_t val) { pack_u16(buf, (uint16_t)val); }

static inline void pack_s32(uint8_t *buf, int32_t val)
{
    buf[0] = (uint8_t)(val >> 24);
    buf[1] = (uint8_t)(val >> 16);
    buf[2] = (uint8_t)(val >> 8);
    buf[3] = (uint8_t)(val & 0xFF);
}

static uint8_t calc_bcc(const uint8_t *data, uint8_t len)
{
    uint8_t bcc = 0;
    for (uint8_t i = 0; i < len; i++) bcc ^= data[i];
    return bcc;
}

// 发送一帧
static void send_frame(uint8_t id, uint8_t func, const uint8_t *data, uint8_t data_len)
{
    uint8_t buf[12];
    uint8_t idx = 0;
    
    buf[idx++] = 0x7A;
    buf[idx++] = id;
    buf[idx++] = func;
    for (uint8_t i = 0; i < data_len; i++) {
        buf[idx++] = data[i];
    }
    
    uint8_t bcc = calc_bcc(buf, idx);       // 先计算 BCC
    buf[idx++] = bcc;                       // 再写入，idx 只被修改一次
    buf[idx++] = 0x7B;
    
    UART_YT_SendData(buf, idx);
}

/* ============================================================================
 *  F32C 单电机驱动
 * ============================================================================ */

void F32C_InitMotor(F32C_Motor_t *motor, uint8_t id, F32C_Mode_t mode)
{
    memset(motor, 0, sizeof(F32C_Motor_t));
    motor->id = id;
    motor->mode = mode;
    motor->accel = F32C_DEFAULT_ACCEL;
}

void F32C_Enable(F32C_Motor_t *motor)
{
    send_frame(motor->id, 0x06, NULL, 0);
    motor->enabled = true;
}

void F32C_Disable(F32C_Motor_t *motor)
{
    send_frame(motor->id, 0x05, NULL, 0);
    motor->enabled = false;
}

void F32C_SetMode(F32C_Motor_t *motor, F32C_Mode_t mode)
{
    uint8_t data[2];
    pack_u16(data, (uint16_t)mode);
    send_frame(motor->id, 0x00, data, 2);
    motor->mode = mode;
}

void F32C_SetAccel(F32C_Motor_t *motor, uint16_t accel)
{
    uint8_t data[2];
    pack_u16(data, accel);
    send_frame(motor->id, 0x07, data, 2);
    motor->accel = accel;
}

void F32C_SetSpeed(F32C_Motor_t *motor, int16_t speed_rpm)
{
    uint8_t data[2];
    pack_s16(data, speed_rpm);
    send_frame(motor->id, 0x01, data, 2);
}

void F32C_SetAngleMulti(F32C_Motor_t *motor, float angle_deg, int16_t speed_rpm)
{
    F32C_SetSpeed(motor, speed_rpm);
    
    int32_t scaled = (int32_t)(angle_deg * F32C_ANGLE_SCALE);
    uint8_t data[4];
    pack_s32(data, scaled);
    send_frame(motor->id, 0x02, data, 4);
}

void F32C_SetAngleSingle(F32C_Motor_t *motor, float angle_deg, int16_t speed_rpm)
{
    F32C_SetSpeed(motor, speed_rpm);
    
    float ang = angle_deg;
    if (ang < 0) ang = 0;
    if (ang > 359.9f) ang = 359.9f;
    uint16_t scaled = (uint16_t)(ang * F32C_ANGLE_SCALE);
    uint8_t data[2];
    pack_u16(data, scaled);
    send_frame(motor->id, 0x03, data, 2);
}

void F32C_ZeroMulti(F32C_Motor_t *motor)  { send_frame(motor->id, 0x09, NULL, 0); }
void F32C_ZeroSingle(F32C_Motor_t *motor) { send_frame(motor->id, 0x0A, NULL, 0); }
void F32C_SaveParam(F32C_Motor_t *motor)  { send_frame(motor->id, 0x08, NULL, 0); }

/* ============================================================================
 *  云台管理 - 初始化命令表（替代13个case的状态机）
 * ============================================================================ */

#define INIT_DELAY_MS   3

typedef struct {
    uint8_t  id;
    uint8_t  func;
    uint16_t data;      // 对于无数据的命令（如Enable），此值忽略
    bool     has_data;  // 是否有数据域
} InitCmd_t;

static const InitCmd_t init_cmd_table[] = {
    {MOTOR_YAW_ID,   0x06, 0,              false},  // Enable Yaw
    {MOTOR_PITCH_ID, 0x06, 0,              false},  // Enable Pitch
    {MOTOR_YAW_ID,   0x00, MODE_MULTI_T,   true },  // Mode Yaw = Multi-Turn
    {MOTOR_PITCH_ID, 0x00, MODE_SINGLE_T,  true },  // Mode Pitch = Single-Turn
    {MOTOR_YAW_ID,   0x07, F32C_DEFAULT_ACCEL, true},  // Accel Yaw
    {MOTOR_PITCH_ID, 0x07, F32C_DEFAULT_ACCEL, true},  // Accel Pitch
};

static uint8_t init_step = 0;
static uint32_t init_wait_tick = 0;

void Gimbal_Init(Gimbal_t *g)
{
    g->sys_tick_ms = 0;
    g->last_tx_tick = 0;
    g->tx_toggle = 0;
    init_step = 0;
    init_wait_tick = 0;
    F32C_InitMotor(&g->yaw, MOTOR_YAW_ID, MODE_MULTI_T);
    F32C_InitMotor(&g->pitch, MOTOR_PITCH_ID, MODE_SINGLE_T);
}

bool Gimbal_PowerOnInit(Gimbal_t *g, uint32_t now_ms)
{
    const uint8_t total_steps = sizeof(init_cmd_table) / sizeof(init_cmd_table[0]);
    
    if (init_step >= total_steps) return true;  // 全部完成
    
    // 等待间隔
    if (init_step > 0 && (now_ms - init_wait_tick) < INIT_DELAY_MS) return false;
    
    // 执行当前步骤
    const InitCmd_t *cmd = &init_cmd_table[init_step];
    if (cmd->has_data) {
        uint8_t data[2];
        pack_u16(data, cmd->data);
        send_frame(cmd->id, cmd->func, data, 2);
    } else {
        send_frame(cmd->id, cmd->func, NULL, 0);
    }
    
    init_wait_tick = now_ms;
    init_step++;
    return false;
}

void Gimbal_SetZero(Gimbal_t *g)
{
    F32C_ZeroMulti(&g->yaw);
    F32C_ZeroSingle(&g->pitch);
}

void Gimbal_SaveAll(Gimbal_t *g)
{
    F32C_SaveParam(&g->yaw);
    F32C_SaveParam(&g->pitch);
}

void Gimbal_SetTarget(Gimbal_t *g, float yaw_deg, float pitch_deg, int16_t speed_rpm)
{
    g->yaw.target_angle = yaw_deg;
    g->yaw.target_speed = speed_rpm;
    g->yaw.need_update = true;
    
    g->pitch.target_angle = pitch_deg;
    g->pitch.target_speed = speed_rpm;
    g->pitch.need_update = true;
}

/* ============================================================================
 *  发送单电机一帧（简化状态机：2状态 + 完成）
 * ============================================================================ */

static void motor_send_frame(F32C_Motor_t *motor, uint32_t now_ms)
{
    if (!motor->need_update) return;
    
    uint8_t data[4];
    uint8_t len = 0;
    uint8_t func = 0;
    
    switch (motor->tx_state) {
        case TX_SEND_SPEED:
            if (motor->target_speed != motor->curr_speed) {
                pack_s16(data, motor->target_speed);
                func = 0x01;
                len = 2;
                motor->curr_speed = motor->target_speed;
            }
            motor->tx_state = TX_SEND_ANGLE;
            break;
            
        case TX_SEND_ANGLE:
            if (fabsf(motor->target_angle - motor->curr_angle) >= 0.05f) {
                if (motor->id == MOTOR_YAW_ID) {
                    // 多圈
                    int32_t scaled = (int32_t)(motor->target_angle * F32C_ANGLE_SCALE);
                    pack_s32(data, scaled);
                    func = 0x02;
                    len = 4;
                } else {
                    // 单圈
                    float ang = motor->target_angle;
                    if (ang < 0) ang = 0;
                    if (ang > 359.9f) ang = 359.9f;
                    pack_u16(data, (uint16_t)(ang * F32C_ANGLE_SCALE));
                    func = 0x03;
                    len = 2;
                }
                motor->curr_angle = motor->target_angle;
            }
            motor->tx_state = TX_DONE;
            break;
            
        case TX_DONE:
        default:
            motor->need_update = false;
            motor->tx_state = TX_SEND_SPEED;  // 准备下一轮
            return;
    }
    
    if (len > 0) {
        send_frame(motor->id, func, data, len);
        motor->last_tx_tick = now_ms;
    }
}

void Gimbal_Poll(Gimbal_t *g, uint32_t now_ms)
{
    g->sys_tick_ms = now_ms;
    
    // 检查 1ms 间隔
    if ((now_ms - g->last_tx_tick) < F32C_CMD_INTERVAL_MS) return;
    
    // 交替发送 yaw 和 pitch
    F32C_Motor_t *motor = (g->tx_toggle == 0) ? &g->yaw : &g->pitch;
    g->tx_toggle ^= 1;
    
    if (motor->need_update) {
        motor_send_frame(motor, now_ms);
        g->last_tx_tick = now_ms;
    }
}

bool Gimbal_IsReady(const Gimbal_t *g)
{
    return !g->yaw.need_update && !g->pitch.need_update;
}