#include "trap_profile.h"
#include <math.h>

void TrapProfile_Init(TrapProfile_t *tp, float dt, float max_vel, float accel, float decel)
{
    tp->current = 0.0f;
    tp->target = 0.0f;
    tp->velocity = 0.0f;
    tp->max_velocity = max_vel;
    tp->acceleration = accel;
    tp->deceleration = decel;
    tp->dt = dt;
    tp->done = true;
}

/**
 * 设置新的目标位置
 * 会在内部重新规划梯形速度曲线
 */
void TrapProfile_SetTarget(TrapProfile_t *tp, float target)
{
    tp->target = target;
    tp->done = false;
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

    // ========== 角度归一化：计算最短路径 ==========
    float distance = tp->target - tp->current;
    while (distance > 180.0f) distance -= 360.0f;
    while (distance < -180.0f) distance += 360.0f;
    
    float abs_distance = fabsf(distance);
    float direction = (distance > 0.0f) - (distance < 0.0f);

    if (direction == 0.0f) {
        tp->velocity = 0.0f;
        tp->done = true;
        return tp->current;
    }

    // ========== 1. 计算减速距离 ==========
    float stop_distance = (tp->velocity * tp->velocity) / (2.0f * tp->deceleration);

    // ========== 2. 速度规划 ==========
    if (abs_distance <= stop_distance) {
        tp->velocity -= tp->deceleration * tp->dt;
        if (tp->velocity < 0.0f) tp->velocity = 0.0f;
    } else if (tp->velocity < tp->max_velocity) {
        tp->velocity += tp->acceleration * tp->dt;
        if (tp->velocity > tp->max_velocity) {
            tp->velocity = tp->max_velocity;
        }
    }

    // ========== 3. 更新位置 ==========
    tp->current += tp->velocity * direction * tp->dt;

    // ========== 4. 到达判断（在归一化之前，避免跳变） ==========
    // 重新计算归一化距离来判断是否到达
    float check_dist = tp->target - tp->current;
    while (check_dist > 180.0f) check_dist -= 360.0f;
    while (check_dist < -180.0f) check_dist += 360.0f;

    if (tp->velocity <= 0.0f && fabsf(check_dist) < 3.0f) {
        tp->current = tp->target;
        tp->velocity = 0.0f;
        tp->done = true;
        return tp->current;
    }


    return tp->current;
}

void TrapProfile_Reset(TrapProfile_t *tp, float start_pos)
{
    tp->current = start_pos;
    tp->target = start_pos;
    tp->velocity = 0.0f;
    tp->done = true;
}

/**
 * 切换到速度模式，设置目标速度
 * current 被复用为"当前平滑速度"
 * velocity 被复用为"速度变化率"（固定为加速度）
 */
void TrapProfile_SpeedMode(TrapProfile_t *tp, float target_speed)
{
    tp->target = target_speed;      // 目标速度
    tp->done = false;
}

/**
 * 速度模式下的更新，返回平滑后的速度值
 * 复用 current 存当前速度，velocity 存变化率
 */
float TrapProfile_SpeedUpdate(TrapProfile_t *tp)
{
    if (tp->done) {
        return tp->current;
    }

    if (tp->current < tp->target) {
        // 加速
        float step = tp->acceleration * tp->dt;
        tp->current += step;
        if (tp->current >= tp->target) {
            tp->current = tp->target;
            tp->done = true;
        }
    } else if (tp->current > tp->target) {
        // 减速（用 deceleration）
        float step = tp->deceleration * tp->dt;
        tp->current -= step;
        if (tp->current <= tp->target) {
            tp->current = tp->target;
            tp->done = true;
        }
    } else {
        tp->done = true;
    }

    return tp->current;
}