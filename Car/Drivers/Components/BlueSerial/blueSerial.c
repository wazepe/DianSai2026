#include "blueSerial.h"
#include <stdint.h>
#include <stdio.h>

// ==================== 环形缓冲区 ====================
static uint8_t ringBuffer[BLUE_SERIAL_RING_BUFFER_SIZE];
static uint16_t putIndex = 0;
static uint16_t getIndex = 0;

// ==================== 临时接收缓冲区 ====================
uint8_t rxTempBuffer = 0;  // 中断临时存储

// ==================== 数据包解析器 ====================
static char packetBuffer[BLUE_SERIAL_MAX_STRING_LEN];
static uint16_t packetIndex = 0;
static PacketState_t packetState = PACKET_STATE_IDLE;

// ==================== 初始化 ====================
void BlueSerial_Init(void)
{
    // 清空所有缓冲区
    memset(ringBuffer, 0, BLUE_SERIAL_RING_BUFFER_SIZE);
    memset(packetBuffer, 0, BLUE_SERIAL_MAX_STRING_LEN);
    putIndex = 0;
    getIndex = 0;
    packetIndex = 0;
    packetState = PACKET_STATE_IDLE;
    
    // 启动逐字节接收
    NVIC_EnableIRQ(UART_BLE_INST_INT_IRQN);
}

// ==================== 打印函数 ====================
void BlueSerial_Printf(const char *format, ...)
{
    char buffer[100] = "";
    va_list args;
    
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    uint8_t lens = strlen(buffer);
    for (int i = 0; i < lens; i++) {
        DL_UART_Main_transmitDataBlocking(UART_BLE_INST, buffer[i]);
    }
}

// ==================== 环形缓冲区操作 ====================
uint8_t BlueSerial_Put(uint8_t Byte)
{
    uint16_t nextIndex = (putIndex + 1) % BLUE_SERIAL_RING_BUFFER_SIZE;
    if (nextIndex == getIndex) {
        return 1;  // 缓冲区满
    }
    ringBuffer[putIndex] = Byte;
    putIndex = nextIndex;
    return 0;
}

uint8_t BlueSerial_Get(uint8_t *Byte)
{
    if (getIndex == putIndex) {
        *Byte = 0;
        return 1;  // 缓冲区空
    }
    *Byte = ringBuffer[getIndex];
    getIndex = (getIndex + 1) % BLUE_SERIAL_RING_BUFFER_SIZE;
    return 0;
}

uint16_t BlueSerial_Length(void)
{
    return (putIndex + BLUE_SERIAL_RING_BUFFER_SIZE - getIndex) % BLUE_SERIAL_RING_BUFFER_SIZE;
}

void BlueSerial_ClearRingBuffer(void)
{
    memset(ringBuffer, 0, BLUE_SERIAL_RING_BUFFER_SIZE);
    putIndex = 0;
    getIndex = 0;
}

// ==================== 数据包解析（状态机）====================
void BlueSerial_IRQHandler(uint8_t RxData)
{
    // 1. 先存入环形缓冲区（保留原始数据）
    BlueSerial_Put(RxData);
    
    // 2. 状态机解析数据包（实时解析，不依赖主循环消费）
    switch (packetState) {
        case PACKET_STATE_IDLE:
            if (RxData == '[') {
                packetState = PACKET_STATE_RECEIVING;
                packetIndex = 0;
                memset(packetBuffer, 0, BLUE_SERIAL_MAX_STRING_LEN);
            }
            break;
            
        case PACKET_STATE_RECEIVING:
            if (RxData == ']') {
                packetBuffer[packetIndex] = '\0';
                packetState = PACKET_STATE_COMPLETE;
            } else if (packetIndex < BLUE_SERIAL_MAX_STRING_LEN - 1) {
                packetBuffer[packetIndex++] = RxData;
            } else {
                // 缓冲区溢出，丢弃当前包
                packetState = PACKET_STATE_IDLE;
                packetIndex = 0;
            }
            break;
            
        case PACKET_STATE_COMPLETE:
            // 等待主循环处理，新数据暂时忽略（或可以选择丢弃）
            // 也可以在这里做覆盖保护
            break;
    }
}

// 检查是否有完整的包等待处理
uint8_t BlueSerial_IsPacketReady(void)
{
    return (packetState == PACKET_STATE_COMPLETE);
}

// 解析包内容，提取逗号分隔的字段
uint8_t BlueSerial_ParsePacket(ParsedData_t *outData)
{
    if (!outData || packetState != PACKET_STATE_COMPLETE) {
        return 0;  // 解析失败
    }
    
    // 清空输出结构体
    memset(outData, 0, sizeof(ParsedData_t));
    
    uint8_t fieldIndex = 0;
    uint8_t charIndex = 0;
    
    for (uint16_t i = 0; packetBuffer[i] != '\0' && fieldIndex < BLUE_SERIAL_MAX_FIELDS; i++) {
        if (packetBuffer[i] == ',') {
            // 结束当前字段
            outData->fields[fieldIndex][charIndex] = '\0';
            fieldIndex++;
            charIndex = 0;
        } else {
            // 添加字符到当前字段
            if (charIndex < BLUE_SERIAL_FIELD_MAX_LEN - 1) {
                outData->fields[fieldIndex][charIndex++] = packetBuffer[i];
            }
        }
    }
    
    // 保存最后一个字段
    if (fieldIndex < BLUE_SERIAL_MAX_FIELDS && charIndex > 0) {
        outData->fields[fieldIndex][charIndex] = '\0';
        fieldIndex++;
    }
    
    outData->fieldCount = fieldIndex;
    outData->isValid = (fieldIndex > 0);
    
    // 解析完成后重置状态机，准备接收下一个包
    packetState = PACKET_STATE_IDLE;
    packetIndex = 0;
    
    return outData->isValid;
}

// 重置解析器（放弃当前正在接收的包）
void BlueSerial_ResetParser(void)
{
    packetState = PACKET_STATE_IDLE;
    packetIndex = 0;
    memset(packetBuffer, 0, BLUE_SERIAL_MAX_STRING_LEN);
}
