#ifndef STEPPER_H
#define STEPPER_H

#include <stdint.h>

/* UART 实例，与 ti_msp_dl_config.h 里生成的名字一致 */
#define STEPPER_UART     UART_S_INST

/* 电机默认地址 */
#define STEPPER_ADDR     0x01

/* 方向 */
#define STEPPER_CW       0x01     /* 顺时针 */
#define STEPPER_CCW      0x00     /* 逆时针 */

/* 常用细分 */
#define STEPPER_USTEP_16 0x10
#define STEPPER_USTEP_32 0x20

/* 位置控制：angle_x10 = 角度 × 10，如 3600 表示 360.0°；
   speed_x10 = 速度 × 10，如 100 表示 10.0 rad/s */
void stepper_pos_ctrl(uint8_t addr, uint8_t dir, uint16_t angle_x10,
                      uint16_t speed_x10, uint8_t ustep);

/* 停止电机（速度模式发 0） */
void stepper_stop(uint8_t addr);

/* 请求数据反馈（3.6 预留） */
void stepper_request_feedback(uint8_t addr);

#endif