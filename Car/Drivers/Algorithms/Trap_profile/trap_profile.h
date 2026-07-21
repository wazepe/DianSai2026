#ifndef __TRAP_PROFILE_H
#define __TRAP_PROFILE_H

#include <stdbool.h>

typedef struct {
    float current;          // 当前位置/角度
    float target;           // 目标位置/角度
    float velocity;         // 当前速度
    float max_velocity;     // 最大速度
    float acceleration;     // 加速度
    float deceleration;     // 减速度（可以与加速度不同）
    float dt;               // 调用周期（秒）
    bool done;              // 是否到达目标
} TrapProfile_t;

void TrapProfile_Init(TrapProfile_t *tp, float dt, float max_vel, float accel, float decel);
void TrapProfile_SetTarget(TrapProfile_t *tp, float target);
float TrapProfile_Update(TrapProfile_t *tp);
bool TrapProfile_IsDone(TrapProfile_t *tp);
void TrapProfile_Reset(TrapProfile_t *tp, float start_pos);

#endif