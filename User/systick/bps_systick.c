#include "bps_systick.h"
//�����NVIC�������ã���SysTick�жϷ�����SysTick_Handler��Ҳ������жϱ�־��������
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


void SysTick_Init(void)//1ms����һ���ж�
{
	SysTick_Config(SystemCoreClock/1000);
	SysTick->CTRL &=(~SysTick_CTRL_ENABLE_Msk);//�ر�ʱ�� ʹ��ʱ��Ҫ��
	//SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk//��ʱ�Ӳ���
}

/****************************** ���º���ΪDMP�ٷ��⺯��ʹ�� **********************************************/

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
