#include "trap_profile.h"
#include <math.h>

/**
 * 初始化梯形速度规划器
 * @param tp      规划器实例
 * @param dt      调用周期（秒）
 * @param max_vel 最大速度（单位/秒）
 * @param accel   加速度（单位/秒²）
 * @param decel   减速度（单位/秒²，可与加速度不同）
 */
void TrapProfile_Init(TrapProfile_t *tp, float dt, float max_vel, float accel, float decel)
{
    tp->current = 0.0f;
    tp->target = 0.0f;
    tp->velocity = 0.0f;
    tp->max_velocity = max_vel;
    tp->acceleration = accel;
    tp->deceleration = decel;
    tp->dt = dt;
    tp->done = true;  // 初始认为已到位
}

/**
 * 设置新的目标位置
 * 会在内部重新规划梯形速度曲线
 */
void TrapProfile_SetTarget(TrapProfile_t *tp, float target)
{
    tp->target = target;
    tp->done = false;
    // 保持当前速度不变，Update() 中会进行加减速计算
}

/**
 * 每个周期调用一次，返回当前规划的位置
 * 内部实现梯形加减速逻辑
 */
float TrapProfile_Update(TrapProfile_t *tp)
{
    if (tp->done) {
        return tp->current;
    }

     // ========== 角度归一化处理 ==========
    float distance = tp->target - tp->current;

    while (distance > 180.0f) distance -= 360.0f;
    while (distance < -180.0f) distance += 360.0f;
    
    float abs_distance = fabsf(distance);
    float direction;
    if (distance > 0.0f) {
        direction = 1.0f;
    } else if (distance < 0.0f) {
        direction = -1.0f;
    } else {
        direction = 0.0f;
    }

    // ========== 1. 计算减速开始距离 ==========
    // 从当前速度减速到0需要的距离：S = v² / (2 * a)
    float stop_distance = (tp->velocity * tp->velocity) / (2.0f * tp->deceleration);

    // ========== 2. 判断当前处于哪个阶段 ==========
    if (abs_distance <= stop_distance || tp->velocity > tp->max_velocity) {
        // --- 减速阶段 ---
        // 需要开始减速了（或者速度超出最大速度限制）
        if (tp->velocity > 0.0f) {
            tp->velocity -= tp->deceleration * tp->dt;
            if (tp->velocity < 0.0f) tp->velocity = 0.0f;
        }
    } else {
        // --- 加速或匀速阶段 ---
        if (tp->velocity < tp->max_velocity) {
            // 加速
            tp->velocity += tp->acceleration * tp->dt;
            if (tp->velocity > tp->max_velocity) {
                tp->velocity = tp->max_velocity;
            }
        }
        // 否则保持匀速
    }

    // ========== 3. 更新位置 ==========
    if (direction != 0.0f) {
        tp->current += tp->velocity * direction * tp->dt;
    }

     // 保持角度在 [-180, 180] 范围，与 BNO08x yaw 一致
    while (tp->current > 180.0f) tp->current -= 360.0f;
    while (tp->current <= -180.0f) tp->current += 360.0f;

    // ========== 4. 检查是否到达目标 ==========
    // 软着陆：接近目标时按指数减速，避免硬停止导致抖动
    if (abs_distance < 3.0f) {
        tp->velocity *= 0.6f;
        if (fabsf(tp->velocity) < 0.5f || abs_distance < 0.1f) {
            tp->current = tp->target;
            tp->velocity = 0.0f;
            tp->done = true;
        }
    }

    return tp->current;
}

/**
 * 查询是否到达目标
 */
bool TrapProfile_IsDone(TrapProfile_t *tp)
{
    return tp->done;
}

/**
 * 强制重置到某个位置
 */
void TrapProfile_Reset(TrapProfile_t *tp, float start_pos)
{
    tp->current = start_pos;
    tp->target = start_pos;
    tp->velocity = 0.0f;
    tp->done = true;
}