#include "filter.h"
#include <math.h>
#include <string.h>

/**
 * 函    数：初始化低通滤波器
 * 参    数：f - 滤波器结构体指针
 * 返 回 值：无
 */
void LowPassFilter_Init(LowPassFilter_t *f,float alpha,float deadband)
{
    memset(f, 0, sizeof(LowPassFilter_t));
    f->alpha = alpha;
    f->deadband = deadband;
}

/**
 * 函    数：更新滤波器（核心函数）
 * 参    数：f - 滤波器结构体指针
 *          raw - 原始输入值
 * 返 回 值：滤波后的输出值
 */
float LowPassFilter_Update(LowPassFilter_t *f, float raw)
{
    // 1. 首次调用，直接初始化为当前值
    if (!f->initialized) {
        f->output = raw;
        f->last_raw = raw;
        f->initialized = 1;
        return f->output;
    }
    
    // 2. 死区处理：微小变化直接忽略
    if (f->deadband > 0.0f) {
        float diff = raw - f->last_raw;
        if (fabsf(diff) < f->deadband) {
            return f->output;  // 变化太小，不更新
        }
    }
    f->last_raw = raw;
    
    // 3. 一阶低通滤波
    float filtered = f->alpha * raw + (1.0f - f->alpha) * f->output;
    
    // 4. 更新输出
    f->output = filtered;
    return f->output;
}