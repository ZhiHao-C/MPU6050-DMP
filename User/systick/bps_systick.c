#include "bps_systick.h"
//无需对NVIC进行配置，在SysTick中断服务函数SysTick_Handler中也无需对中断标志进行清零
void Systick_us_delay(uint32_t us)
{
	uint32_t i;
	SysTick_Config(SystemCoreClock/1000000);
	for (i=0;i<us;i++)
	{
		while (!(SysTick->CTRL &(1<<16)));
//		SysTick->CTRL &=~(1<<16);//
	}
	SysTick->CTRL &=(~(SysTick_CTRL_ENABLE_Msk));
}



void Systick_ms_delay(uint32_t ms)
{
	uint32_t i;
	SysTick_Config(72000);
	for (i=0;i<ms;i++)
	{
		while (!(SysTick->CTRL &(1<<16)));
//		SysTick->CTRL &=~(1<<16);
	}
	SysTick->CTRL &=(~SysTick_CTRL_ENABLE_Msk);

}


void SysTick_Init(void)//1ms触发一次中断
{
	SysTick_Config(SystemCoreClock/1000);
	SysTick->CTRL &=(~SysTick_CTRL_ENABLE_Msk);//关闭时钟 使用时需要打开
	//SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk//打开时钟操作
}

/****************************** 以下函数为DMP官方库函数使用 **********************************************/

volatile uint32_t g_ul_ms_ticks=0;
static volatile uint32_t TimingDelay=0;


void mdelay(unsigned long nTime)
{
	TimingDelay = nTime;
	while(TimingDelay != 0);
}

int get_tick_count(unsigned long *count)
{
        count[0] = g_ul_ms_ticks;
	return 0;
}

void TimingDelay_Decrement(void)
{
	if (TimingDelay != 0x00)
		TimingDelay--;
}

void TimeStamp_Increment(void)
{
	g_ul_ms_ticks++;
}
