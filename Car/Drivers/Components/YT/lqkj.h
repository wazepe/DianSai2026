#ifndef __LQKJ_H
#define __LQKJ_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/* ============================================================================
 *  配置参数
 * ============================================================================ */

#define F32C_BAUDRATE           115200
#define F32C_DEFAULT_ACCEL      100     // 转/s²

#define MOTOR_YAW_ID            0x01    // 方位轴（多圈）
#define MOTOR_PITCH_ID          0x02    // 俯仰轴（单圈）

#define F32C_CMD_INTERVAL_MS    1       // 电机间最小指令间隔 1ms
#define F32C_ANGLE_SCALE        10      // 角度精度：协议放大 10 倍

/* ============================================================================
 *  数据类型
 * ============================================================================ */

typedef enum {
    MODE_SPEED          = 0x0000,
    MODE_MULTI_T        = 0x0001,
    MODE_SINGLE_T       = 0x0002,
    MODE_MULTI_DIRECT   = 0x0003,
    MODE_SINGLE_DIRECT  = 0x0004,
} F32C_Mode_t;

// 发送状态机：只有2个有效状态 + 完成
typedef enum {
    TX_SEND_SPEED = 0,  // 发速度
    TX_SEND_ANGLE,      // 发角度
    TX_DONE,            // 本轮完成，回到TX_SEND_SPEED开始下一轮
} TxState_t;

typedef struct {
    uint8_t         id;
    F32C_Mode_t     mode;
    bool            enabled;
    
    float           target_angle;
    int16_t         target_speed;
    
    float           curr_angle;
    int16_t         curr_speed;
    
    uint16_t        accel;
    
    TxState_t       tx_state;
    uint32_t        last_tx_tick;
    bool            need_update;
} F32C_Motor_t;

typedef struct {
    F32C_Motor_t yaw;
    F32C_Motor_t pitch;
    uint32_t sys_tick_ms;
    uint32_t last_tx_tick;      // 移入结构体，支持多实例
    uint8_t  tx_toggle;         // 交替发送标志
} Gimbal_t;

/* ============================================================================
 *  API
 * ============================================================================ */

extern void UART_YT_SendData(const uint8_t *data, uint8_t len);

void F32C_InitMotor(F32C_Motor_t *motor, uint8_t id, F32C_Mode_t mode);
void F32C_Enable(F32C_Motor_t *motor);
void F32C_Disable(F32C_Motor_t *motor);
void F32C_SetMode(F32C_Motor_t *motor, F32C_Mode_t mode);
void F32C_SetAccel(F32C_Motor_t *motor, uint16_t accel);
void F32C_SetSpeed(F32C_Motor_t *motor, int16_t speed_rpm);
void F32C_SetAngleMulti(F32C_Motor_t *motor, float angle_deg, int16_t speed_rpm);
void F32C_SetAngleSingle(F32C_Motor_t *motor, float angle_deg, int16_t speed_rpm);
void F32C_ZeroMulti(F32C_Motor_t *motor);
void F32C_ZeroSingle(F32C_Motor_t *motor);
void F32C_SaveParam(F32C_Motor_t *motor);

void Gimbal_Init(Gimbal_t *g);
bool Gimbal_PowerOnInit(Gimbal_t *g, uint32_t now_ms);
void Gimbal_SetZero(Gimbal_t *g);
void Gimbal_SaveAll(Gimbal_t *g);

void Gimbal_SetTarget(Gimbal_t *g, float yaw_deg, float pitch_deg, int16_t speed_rpm);
void Gimbal_Poll(Gimbal_t *g, uint32_t now_ms);
bool Gimbal_IsReady(const Gimbal_t *g);

#endif