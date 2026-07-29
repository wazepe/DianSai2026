#include "ti_msp_dl_config.h"
#include "infrared.h"

#define Infrared_ADDRESS                        0x12
#define Infrared_Status_RegAddress              0x30
#define Infrared_Data_Verification_RegAddress   0x01

uint8_t Infrared_Number_Status = 0x00;

void Infrared_WriteReg(uint8_t RegAddress, uint8_t Data)
{
    // 等待I2C控制器空闲
    while (!(DL_I2C_getControllerStatus(I2C_IR_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    
    // 启动传输(写模式)
    DL_I2C_startControllerTransfer(I2C_IR_INST, Infrared_ADDRESS, 
        DL_I2C_CONTROLLER_DIRECTION_TX, 2); // 发送2字节: 寄存器地址和数据
    
    // 发送寄存器地址
    while (DL_I2C_isControllerTXFIFOFull(I2C_IR_INST));
    DL_I2C_transmitControllerData(I2C_IR_INST, RegAddress);
    
    // 发送数据
    while (DL_I2C_isControllerTXFIFOFull(I2C_IR_INST));
    DL_I2C_transmitControllerData(I2C_IR_INST, Data);
    
    // 等待传输完成
    while (!(DL_I2C_getControllerStatus(I2C_IR_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
}

uint8_t Infrared_ReadReg(uint8_t RegAddress)
{
    uint8_t Data = 0;
    
    // 1. 发送寄存器地址(写模式)
    while (!(DL_I2C_getControllerStatus(I2C_IR_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    
    DL_I2C_startControllerTransfer(I2C_IR_INST, Infrared_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1); // 发送1字节: 寄存器地址
    
    while (DL_I2C_isControllerTXFIFOFull(I2C_IR_INST));
    DL_I2C_transmitControllerData(I2C_IR_INST, RegAddress);
    
    // 等待传输完成
    while (!(DL_I2C_getControllerStatus(I2C_IR_INST) & DL_I2C_CONTROLLER_STATUS_IDLE));
    
    // 2. 读取数据(读模式)
    DL_I2C_startControllerTransfer(I2C_IR_INST, Infrared_ADDRESS,
        DL_I2C_CONTROLLER_DIRECTION_RX, 1); // 读取1字节数据
    
    while (DL_I2C_isControllerRXFIFOEmpty(I2C_IR_INST));
    Data = DL_I2C_receiveControllerData(I2C_IR_INST);

    Data = ~Data;
    
    return Data;
}

float Infrared_Read_All(uint8_t *sensor_value, uint8_t mask)
{
    uint8_t count = 0;
    uint8_t activeValue;
    float weightedValue = 0.0f;
    static float lastWeightedValue = 4.5f;

    if (sensor_value == NULL) {
        return lastWeightedValue;
    }

    /* 从红外状态寄存器读取8bit原始值 */
    activeValue = Infrared_ReadReg(Infrared_Status_RegAddress);
    *sensor_value = activeValue;

    for (uint8_t i = 0; i < 8; i++) {
        if ((activeValue & mask & (0x01u << i)) != 0u) {
            count++;
            weightedValue += (float)(i + 1);
        }
    }

    if (count == 0) {
        return lastWeightedValue;
    }

    weightedValue /= (float)count;
    lastWeightedValue = weightedValue;

    if (weightedValue > 8.0f) {
        weightedValue = 8.0f;
    }

    return weightedValue;
}
