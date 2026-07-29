#include "graySensor.h"

#include <string.h>

#define GS_UART_INST                 UART_0_INST
#define GS_UART_INST_INT_IRQN        UART_0_INST_INT_IRQN
#define GS_FRAME_MAX_LEN             80
#define GS_BLACK_RAW_VALUE           0

static volatile uint8_t gs_rxFrame[GS_FRAME_MAX_LEN];
static volatile uint8_t gs_rxIndex = 0;
static volatile uint8_t gs_receiving = 0;
static volatile uint8_t gs_rawData = 0;
static volatile uint8_t gs_newFrame = 0;

static void Gray_Sensor_SendString(const char *str)
{
    while (*str != '\0') {
        DL_UART_Main_transmitDataBlocking(GS_UART_INST, (uint8_t)*str++);
    }
}

void Gray_Sensor_RequestDigitalMode(void)
{
    Gray_Sensor_SendString("$0,0,1#");
}

void Gray_Sensor_Init(void)
{
    gs_rxIndex = 0;
    gs_receiving = 0;
    gs_rawData = 0;
    gs_newFrame = 0;
    memset((void *)gs_rxFrame, 0, sizeof(gs_rxFrame));

    NVIC_ClearPendingIRQ(GS_UART_INST_INT_IRQN);
    NVIC_EnableIRQ(GS_UART_INST_INT_IRQN);

    delay_cycles(CPUCLK_FREQ);
    Gray_Sensor_RequestDigitalMode();
}

static void Gray_Sensor_ParseFrame(void)
{
    uint8_t parsedRaw = 0;
    uint8_t validCount = 0;

    if (gs_rxIndex < 4 || gs_rxFrame[0] != '$' || gs_rxFrame[1] != 'D') {
        return;
    }

    for (uint8_t i = 2; i + 3 < gs_rxIndex; i++) {
        if (gs_rxFrame[i] != 'x') {
            continue;
        }

        uint8_t index = gs_rxFrame[i + 1];
        if (index < '1' || index > '8') {
            continue;
        }

        uint8_t j = i + 2;
        while (j < gs_rxIndex && gs_rxFrame[j] != ':') {
            j++;
        }
        if (j + 1 >= gs_rxIndex) {
            continue;
        }

        uint8_t value = gs_rxFrame[j + 1];
        if (value != '0' && value != '1') {
            continue;
        }

        if ((uint8_t)(value - '0') == GS_BLACK_RAW_VALUE) {
            parsedRaw |= (uint8_t)(0x01u << (index - '1'));
        }
        validCount++;
    }

    if (validCount == 8) {
        gs_rawData = parsedRaw;
        gs_newFrame = 1;
    }
}

void Gray_Sensor_IRQHandler(uint8_t rxData)
{
    if (rxData == '$') {
        gs_receiving = 1;
        gs_rxIndex = 0;
        gs_rxFrame[gs_rxIndex++] = rxData;
        return;
    }

    if (!gs_receiving) {
        return;
    }

    if (gs_rxIndex >= GS_FRAME_MAX_LEN - 1) {
        gs_receiving = 0;
        gs_rxIndex = 0;
        return;
    }

    gs_rxFrame[gs_rxIndex++] = rxData;

    if (rxData == '#') {
        gs_receiving = 0;
        Gray_Sensor_ParseFrame();
        gs_rxIndex = 0;
    }
}

uint8_t Gray_Sensor_GetRaw(void)
{
    return gs_rawData;
}

uint8_t Gray_Sensor_HasNewFrame(void)
{
    uint8_t hasNew = gs_newFrame;
    gs_newFrame = 0;
    return hasNew;
}

float Gray_Sensor_Read_All(uint8_t *sensor_value, uint8_t mask)
{
    uint8_t count = 0;
    uint8_t activeValue;
    float grayValue = 0.0f;
    static float lastGrayValue = 4.5f;

    if (sensor_value == NULL) {
        return lastGrayValue;
    }

    activeValue = gs_rawData;
    *sensor_value = activeValue;

    for (uint8_t i = 0; i < 8; i++) {
        if ((activeValue & mask & (0x01u << i)) != 0u) {
            count++;
            grayValue += (float)(i + 1);
        }
    }

    if (count == 0) {
        return lastGrayValue;
    }

    grayValue /= (float)count;
    lastGrayValue = grayValue;

    if (grayValue > 8.0f) {
        grayValue = 8.0f;
    }

    return grayValue;
}
