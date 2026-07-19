#include "graySensor.h"

// #define FILTER_ALPHA 0.3f

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
    static float lastGrayVule = 4.5f;
    float _GrayVule = 0.0f, grayVule = 0.0f;

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

    _GrayVule = grayVule / (float)count;

    if (_GrayVule > 8.0f) {
        _GrayVule = 8.0f;
    }

    return _GrayVule;
}

// // 一阶低通滤波
// float Gray_Sensor_Read_Filtered(uint8_t* sensor_value)
// {
//     static float filtered_value = 4.5f;  // 初始为中心值
//     float raw_value = Gray_Sensor_Read_All(sensor_value);

//     filtered_value = FILTER_ALPHA * raw_value + (1 - FILTER_ALPHA) * filtered_value;
//     return filtered_value;
// }
