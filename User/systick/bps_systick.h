#ifndef __BPS_SYSTICK_H__
#define __BPS_SYSTICK_H__

#include "stm32f10x.h" 
#include "core_cm3.h"



void Systick_us_delay(uint32_t us);
void Systick_ms_delay(uint32_t ms);
void SysTick_Init(void);

void mdelay(unsigned long nTime);
int get_tick_count(unsigned long *count);
void TimingDelay_Decrement(void);
	void TimeStamp_Increment(void);
#endif

