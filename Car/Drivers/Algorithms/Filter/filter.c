#include "filter.h"
#include <math.h>

/**
 * 函    数：初始化低通滤波器
 * 参    数：f         - 滤波器结构体指针
 *          alpha     - 滤波系数（0~1）
 *          deadband  - 死区范围
 *          init_val  - 输出初始值（建议传入第一次采样值）
 * 返 回 值：无
 */
void LowPassFilter_Init(LowPassFilter_t *f, float alpha, float deadband, float init_val)
{
    f->alpha    = alpha;
    f->deadband = deadband;
    f->output   = init_val;
    f->last_raw = init_val;
}

/**
 * 函    数：更新滤波器（核心函数）
 * 参    数：f   - 滤波器结构体指针
 *          raw - 原始输入值
 * 返 回 值：滤波后的输出值
 */
float LowPassFilter_Update(LowPassFilter_t *f, float raw)
{
    /* 死区处理：微小变化直接忽略，但始终更新 last_raw 防止死区窗口漂移 */
    if (f->deadband > 0.0f) {
        float diff = raw - f->last_raw;
        f->last_raw = raw;
        if (fabsf(diff) < f->deadband) {
            return f->output;
        }
    } else {
        f->last_raw = raw;
    }
    
    /* 一阶低通滤波 */
    f->output = f->alpha * raw + (1.0f - f->alpha) * f->output;
    return f->output;
}