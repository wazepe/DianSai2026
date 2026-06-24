#ifndef _CLOCK_H_
#define _CLOCK_H_

extern volatile uint32_t g_sysTick_1ms_u32;

int mspm0_delay_ms(uint32_t num_ms);
int mspm0_get_clock_ms(uint32_t *count);

#endif  /* #ifndef _CLOCK_H_ */
