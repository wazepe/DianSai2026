#ifndef __BLUE_SERIAL_H
#define __BLUE_SERIAL_H

#include "ti_msp_dl_config.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdarg.h>

#define BLUE_SERIAL_RING_BUFFER_SIZE  100  // 环形缓冲区大小
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

extern uint8_t rxTempBuffer;

// 函数声明
void BlueSerial_Init(void);
void BlueSerial_Printf(const char *format, ...);

// 环形缓冲区操作
uint8_t BlueSerial_Put(uint8_t Byte);
uint8_t BlueSerial_Get(uint8_t *Byte);
uint16_t BlueSerial_Length(void);
void BlueSerial_ClearRingBuffer(void);

// 数据包解析
uint8_t BlueSerial_IsPacketReady(void);
uint8_t BlueSerial_ParsePacket(ParsedData_t *outData);
void BlueSerial_ResetParser(void);

void BlueSerial_IRQHandler(uint8_t RxData);

#endif
