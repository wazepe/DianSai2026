#ifndef __FILTER_H
#define __FILTER_H

#include <stdint.h>

// 滤波器结构体
typedef struct {
    float alpha;          // 滤波系数（0~1）
    float output;         // 滤波输出值
    float last_raw;       // 上次原始值（用于死区判断）
    uint8_t initialized;  // 初始化标志
    float deadband;       // 死区范围（可选）
} LowPassFilter_t;

// 函数声明
void LowPassFilter_Init(LowPassFilter_t *f,float alpha,float deadband);
float LowPassFilter_Update(LowPassFilter_t *f, float raw);

#endif