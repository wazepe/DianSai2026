#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H

#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#define BLUE_SERIAL_DMA_BUF_SIZE      32   // 覆盖式接收，保持最新数据即可
#define BLUE_SERIAL_DMA_CHAN          DMA_BLE_CHAN_ID

#define BLUE_SERIAL_MAX_STRING_LEN    100  // 最大字符串长度
#define BLUE_SERIAL_MAX_FIELDS        6    // 最大字段数
#define BLUE_SERIAL_FIELD_MAX_LEN     20   // 每个字段最大长度

// 数据包解析状态
typedef enum {
    PACKET_STATE_IDLE,      // 空闲，等待 '['
    PACKET_STATE_RECEIVING, // 正在接收数据
    PACKET_STATE_COMPLETE   // 接收完成，等待处理
} PacketState_t;

// 解析后的数据
typedef struct {
    char fields[BLUE_SERIAL_MAX_FIELDS][BLUE_SERIAL_FIELD_MAX_LEN];
    uint8_t fieldCount;
    uint8_t isValid;
} ParsedData_t;

// DMA 接收相关变量
extern volatile uint8_t  bleDmaRxBuf[BLUE_SERIAL_DMA_BUF_SIZE];
extern volatile uint16_t bleDmaRxLen;      // 本次 timeout 接收到的字节数
extern volatile uint8_t  bleDmaRxReady;    // 数据就绪标志

// 函数声明
void BlueSerial_Init(void);
void BlueSerial_DMA_Init(void);
void BlueSerial_Printf(const char *format, ...);

// 高级接口：轮询 DMA 并解析出一个完整包
// 返回 1：成功读取一包到 outData
// 返回 0：当前无完整包
uint8_t BlueSerial_ReadPacket(ParsedData_t *outData);

// 重置解析器（放弃当前正在接收的包）
void BlueSerial_ResetParser(void);

#endif