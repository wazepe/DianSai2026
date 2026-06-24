#include "graySensor.h"

#define SENSOR_AD0_WRITE(X) \
    do { \
        if (X) \
            DL_GPIO_setPins(GPIO_GS_PORT, GPIO_GS_AD0_PIN); \
        else \
            DL_GPIO_clearPins(GPIO_GS_PORT, GPIO_GS_AD0_PIN); \
    } while(0)

#define SENSOR_AD1_WRITE(X) \
    do { \
        if (X) \
            DL_GPIO_setPins(GPIO_GS_PORT, GPIO_GS_AD1_PIN); \
        else \
            DL_GPIO_clearPins(GPIO_GS_PORT, GPIO_GS_AD1_PIN); \
    } while(0)

#define SENSOR_AD2_WRITE(X) \
    do { \
        if (X) \
            DL_GPIO_setPins(GPIO_GS_PORT, GPIO_GS_AD2_PIN); \
        else \
            DL_GPIO_clearPins(GPIO_GS_PORT, GPIO_GS_AD2_PIN); \
    } while(0)

// channel: 0 ~ 7
void _select_channel(uint8_t channel)
{
    SENSOR_AD0_WRITE(channel & 0x01);
    SENSOR_AD1_WRITE(channel & 0x02);
    SENSOR_AD2_WRITE(channel & 0x04);
}

_Bool Read_OUT_value(void)
{
    return DL_GPIO_readPins(GPIO_GS_PORT, GPIO_GS_IN_PIN);
}

float Gray_Sensor_Read_All(uint8_t* sensor_value)
{
    uint8_t i, count = 0;
    static float lastGrayVule;
    float grayVule = 0.0f;

    for (i = 0; i < 8; i++) {
        _select_channel(i);
        delay_cycles(CPUCLK_FREQ / 1000000);  // 延时1us 读取数据
        if (Read_OUT_value()) {
            *sensor_value |= (0x01 << i);
            count ++;
            grayVule += (i + 1);
        } else {
            *sensor_value &= ~(0x01 << i);
        }
    }

    if (grayVule < 0.1f) {
        grayVule = lastGrayVule;
        count = 1;
    } else {
        lastGrayVule = grayVule;
    }

    return grayVule / (float)count;
}
