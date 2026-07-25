#include "blueSerial.h"
#include <stdint.h>
#include <stdio.h>

// ==================== DMA 接收缓冲区 ====================
volatile uint8_t  bleDmaRxBuf[BLUE_SERIAL_DMA_BUF_SIZE];
volatile uint16_t bleDmaRxLen = 0;
volatile uint8_t  bleDmaRxReady = 0;

// ==================== 数据包解析器 ====================
static char packetBuffer[BLUE_SERIAL_MAX_STRING_LEN];
static uint16_t packetIndex = 0;
static PacketState_t packetState = PACKET_STATE_IDLE;
static uint16_t consumeIdx = 0;  // 当前在 bleDmaRxBuf 中的消费位置

// ==================== 初始化 ====================
void BlueSerial_Init(void)
{
    // 清空解析缓冲区
    memset(packetBuffer, 0, BLUE_SERIAL_MAX_STRING_LEN);
    packetIndex = 0;
    packetState = PACKET_STATE_IDLE;
    consumeIdx = 0;

    // 启动 DMA 循环接收
    BlueSerial_DMA_Init();
}

// ==================== DMA 接收初始化 ====================
void BlueSerial_DMA_Init(void)
{
    // 1. 配置 DMA 循环接收
    DL_DMA_disableChannel(DMA, BLUE_SERIAL_DMA_CHAN);
    DL_DMA_setSrcAddr(DMA, BLUE_SERIAL_DMA_CHAN, (uint32_t)&UART_BLE_INST->RXDATA);
    DL_DMA_setDestAddr(DMA, BLUE_SERIAL_DMA_CHAN, (uint32_t)bleDmaRxBuf);
    DL_DMA_setTransferSize(DMA, BLUE_SERIAL_DMA_CHAN, BLUE_SERIAL_DMA_BUF_SIZE);

    // 配置为 Repeat Single：UART 每收到 1 字节触发一次 DMA，搬运 1 字节
    // 与 BNO08X 的 Repeat Block（整包触发）不同，蓝牙是流式字节
    DL_DMA_setTransferMode(DMA, BLUE_SERIAL_DMA_CHAN,
                           DL_DMA_FULL_CH_REPEAT_SINGLE_TRANSFER_MODE);

    DL_DMA_enableChannel(DMA, BLUE_SERIAL_DMA_CHAN);

    // 2. UART 中断已由 SysConfig 初始化（IMASK.RTIM=1, DMA_RX_IMASK.RXINT=1）
    //    只需确认 NVIC 已使能（SysConfig 已做，此处保底）
    NVIC_SetPriority(UART_BLE_INST_INT_IRQN, 2);
    NVIC_EnableIRQ(UART_BLE_INST_INT_IRQN);
}

// ==================== 内部：DMA 接收轮询 ====================
// 消费 bleDmaRxBuf[consumeIdx .. bleDmaRxLen-1]，驱动状态机
static void BlueSerial_DMA_Poll(void)
{
    if (!bleDmaRxReady) {
        return;
    }

    // 从上次断点继续消费
    while (consumeIdx < bleDmaRxLen) {
        uint8_t ch = bleDmaRxBuf[consumeIdx];

        // 遇到完整包就暂停，等 ParsePacket 重置状态机后再继续
        if (packetState == PACKET_STATE_COMPLETE) {
            break;
        }

        consumeIdx++;

        switch (packetState) {
            case PACKET_STATE_IDLE:
                if (ch == '[') {
                    packetState = PACKET_STATE_RECEIVING;
                    packetIndex = 0;
                    packetBuffer[0] = '\0';
                }
                break;

            case PACKET_STATE_RECEIVING:
                if (ch == ']') {
                    packetBuffer[packetIndex] = '\0';
                    packetState = PACKET_STATE_COMPLETE;
                } else if (packetIndex < BLUE_SERIAL_MAX_STRING_LEN - 1) {
                    packetBuffer[packetIndex++] = ch;
                } else {
                    // 单包过长，丢弃
                    packetState = PACKET_STATE_IDLE;
                    packetIndex = 0;
                }
                break;

            default:
                break;
        }
    }

    // 本次 DMA 数据全部消费完毕 → 清除 flag，重置消费索引
    if (consumeIdx >= bleDmaRxLen) {
        bleDmaRxReady = 0;
        consumeIdx = 0;
    }
}

// ==================== 内部：检查是否有完整的包 ====================
static uint8_t BlueSerial_IsPacketReady(void)
{
    return (packetState == PACKET_STATE_COMPLETE);
}

// ==================== 内部：解析包内容 ====================
static uint8_t BlueSerial_ParsePacket(ParsedData_t *outData)
{
    if (!outData || packetState != PACKET_STATE_COMPLETE) {
        return 0;
    }

    // 清空输出结构体
    memset(outData, 0, sizeof(ParsedData_t));

    uint8_t fieldIndex = 0;
    uint8_t charIndex = 0;

    for (uint16_t i = 0; packetBuffer[i] != '\0' && fieldIndex < BLUE_SERIAL_MAX_FIELDS; i++) {
        if (packetBuffer[i] == ',') {
            outData->fields[fieldIndex][charIndex] = '\0';
            fieldIndex++;
            charIndex = 0;
        } else {
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

// ==================== 高级接口：读取一个完整包 ====================
uint8_t BlueSerial_ReadPacket(ParsedData_t *outData)
{
    BlueSerial_DMA_Poll();

    if (BlueSerial_IsPacketReady()) {
        return BlueSerial_ParsePacket(outData);
    }
    return 0;
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

// ==================== 重置解析器 ====================
void BlueSerial_ResetParser(void)
{
    packetState = PACKET_STATE_IDLE;
    packetIndex = 0;
    memset(packetBuffer, 0, BLUE_SERIAL_MAX_STRING_LEN);
}