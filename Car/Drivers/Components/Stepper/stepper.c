#include "stepper.h"
#include "ti_msp_dl_config.h"

/* ---------- 内部：FIFO 轮询发单字节 ---------- */
static void stepper_send_byte(uint8_t data)
{
    /* 等待 TX FIFO 不满 */
    while (DL_UART_Main_isTXFIFOFull(STEPPER_UART)) {
        ;
    }
    DL_UART_Main_transmitData(STEPPER_UART, data);
}

/* ---------- 内部：计算 BCC（前 9 字节异或） ---------- */
static uint8_t stepper_calc_bcc(const uint8_t *buf, uint8_t len)
{
    uint8_t bcc = 0;
    for (uint8_t i = 0; i < len; i++) {
        bcc ^= buf[i];
    }
    return bcc;
}

/* ---------- 内部：发送完整 11 字节帧 ---------- */
static void stepper_send_frame(uint8_t addr, uint8_t mode, uint8_t dir,
                               uint8_t ustep, uint16_t param, uint16_t speed)
{
    uint8_t frame[11];

    frame[0] = 0x7B;                        /* 帧头 */
    frame[1] = addr;                        /* 设备地址 */
    frame[2] = mode;                        /* 控制模式 */
    frame[3] = dir;                         /* 转向 */
    frame[4] = ustep;                       /* 细分 */
    frame[5] = (uint8_t)(param >> 8);       /* POS_H */
    frame[6] = (uint8_t)(param & 0xFF);     /* POS_L */
    frame[7] = (uint8_t)(speed >> 8);       /* SPEED_H */
    frame[8] = (uint8_t)(speed & 0xFF);     /* SPEED_L */
    frame[9] = stepper_calc_bcc(frame, 9);  /* BCC 校验 */
    frame[10] = 0x7D;                       /* 帧尾 */

    for (uint8_t i = 0; i < 11; i++) {
        stepper_send_byte(frame[i]);
    }
}

/* ---------- 公共接口 ---------- */
void stepper_pos_ctrl(uint8_t addr, uint8_t dir, uint16_t angle_x10,
                      uint16_t speed_x10, uint8_t ustep)
{
    /* 位置控制模式 = 0x02 */
    stepper_send_frame(addr, 0x02, dir, ustep, angle_x10, speed_x10);
}

void stepper_stop(uint8_t addr)
{
    /* 速度模式发 0 即可停转 */
    stepper_send_frame(addr, 0x01, STEPPER_CW, STEPPER_USTEP_32, 0, 0);
}

void stepper_request_feedback(uint8_t addr)
{
    uint8_t frame[11] = {
        0x7B, addr, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x7D
    };
    frame[9] = stepper_calc_bcc(frame, 9);
    for (uint8_t i = 0; i < 11; i++) {
        stepper_send_byte(frame[i]);
    }
}