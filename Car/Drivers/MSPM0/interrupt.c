#include "ti_msp_dl_config.h"
#include "interrupt.h"
#include "clock.h"
#include "bno080.h"
#include "key.h"
#include "blueSerial.h"

void SysTick_Handler(void)
{
    g_sysTick_1ms_u32 ++;
    
#ifdef GPIO_KEY_PORT
    Key_Tick();
#endif
}

#ifdef UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;
    extern uint8_t bno08x_dmaBuffer[19];

    DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_CH0_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false)
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);

    for(int i=2; i<=14; i++)
        checkSum += bno08x_dmaBuffer[i];

    if((rxSize == 19) && (bno08x_dmaBuffer[0] == 0xAA) && (bno08x_dmaBuffer[1] == 0xAA) && (checkSum == bno08x_dmaBuffer[18]))
    {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (int16_t)((bno08x_dmaBuffer[4]<<8)|bno08x_dmaBuffer[3]) / 100.0;
        bno08x_data.pitch = (int16_t)((bno08x_dmaBuffer[6]<<8)|bno08x_dmaBuffer[5]) / 100.0;
        bno08x_data.roll = (int16_t)((bno08x_dmaBuffer[8]<<8)|bno08x_dmaBuffer[7]) / 100.0;
        bno08x_data.ax = (bno08x_dmaBuffer[10]<<8)|bno08x_dmaBuffer[9];
        bno08x_data.ay = (bno08x_dmaBuffer[12]<<8)|bno08x_dmaBuffer[11];
        bno08x_data.az = (bno08x_dmaBuffer[14]<<8)|bno08x_dmaBuffer[13];
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
}
#endif

#ifdef UART_BLE_INST_IRQHandler
void UART_BLE_INST_IRQHandler(void)
{
    if (DL_UART_Main_getPendingInterrupt(UART_BLE_INST) == DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR) {

        // 1. 停止 DMA 通道
        DL_DMA_disableChannel(DMA, BLUE_SERIAL_DMA_CHAN);

        // 2. 计算本次已接收字节数
        uint16_t rxSize = BLUE_SERIAL_DMA_BUF_SIZE
                        - DL_DMA_getTransferSize(DMA, BLUE_SERIAL_DMA_CHAN);

        // 3. 批量排空 RX FIFO 残留（MSPM0 FIFO 深度 4）
        uint8_t fifoBuf[4];
        uint32_t fifoCnt = DL_UART_drainRXFIFO(UART_BLE_INST, fifoBuf, 4);
        for (uint32_t i = 0; i < fifoCnt && rxSize < BLUE_SERIAL_DMA_BUF_SIZE; i++) {
            bleDmaRxBuf[rxSize++] = fifoBuf[i];
        }

        // 4. 无条件覆盖：调试接口，永远保持最新数据
        if (rxSize > 0) {
            bleDmaRxLen   = rxSize;
            bleDmaRxReady = 1;
        }

        // 5. 重置 DMA 到缓冲区头部（覆盖式，模仿 BNO08X）
        DL_DMA_setDestAddr(DMA, BLUE_SERIAL_DMA_CHAN, (uint32_t)bleDmaRxBuf);
        DL_DMA_setTransferSize(DMA, BLUE_SERIAL_DMA_CHAN, BLUE_SERIAL_DMA_BUF_SIZE);
        DL_DMA_enableChannel(DMA, BLUE_SERIAL_DMA_CHAN);

        // P2.6 兼容性：若实测 RTFG 只触发一次后不再进入，
        // 改用 DL_UART_clearInterruptStatus(UART_BLE_INST, UART_CPU_INT_IMASK_RTFG_SET);
        DL_UART_Main_clearInterruptStatus(UART_BLE_INST,
            DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
    }
}
#endif

// void GROUP1_IRQHandler(void)
// {

// }
