#include "ti_msp_dl_config.h"
#include "clock.h"

volatile uint32_t g_sysTick_1ms_u32;
volatile uint32_t start_time;

int mspm0_delay_ms(uint32_t num_ms)
{
    start_time = g_sysTick_1ms_u32;
    while (g_sysTick_1ms_u32 - start_time < num_ms);
    return 0;
}

int mspm0_get_clock_ms(uint32_t *count)
{
    if (!count)
        return 1;
    count[0] = g_sysTick_1ms_u32;
    return 0;
}
