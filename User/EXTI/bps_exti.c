#include "bps_exti.h"


static void NVIC_Config(void)//该函数只有在这个文件内容才能被调用其他文件调用不了
{
	NVIC_InitTypeDef NVIC_InitStruct;//设置变量
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//分组为组1（主优先级为1bit 次优先级为3bit）
	NVIC_InitStruct.NVIC_IRQChannel=EXTI_IRQ;//设置引脚来源
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void EXTI_Config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	//配置中断优先级
	NVIC_Config();
	
	//初始化使得GPIOA PA11为输入状态
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStruct.GPIO_Pin=EXTI_GPIO_PIN;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(EXTI_GPIO_PORT, &GPIO_InitStruct);
	//初始化EXTI包括选择模式 触发方式 引脚
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO,ENABLE);//打开AFIO时钟
	GPIO_EXTILineConfig(EXTI_SOURCE_PORT, EXTI_SOURCE_PIN); 
//	EXTI_DeInit();//恢复默认状态
  EXTI_InitStructure.EXTI_Line = EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //上升沿中断
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure); 
}


void EnableInvInterrupt(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
  /* Configure EXTI Line1 */
  EXTI_InitStructure.EXTI_Line = EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}
void DisableInvInterrupt(void)
{
  EXTI_InitTypeDef   EXTI_InitStructure;
    /* Configure EXTI Line1 */
  EXTI_InitStructure.EXTI_Line = EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
  EXTI_InitStructure.EXTI_LineCmd = DISABLE;
  EXTI_Init(&EXTI_InitStructure);
  EXTI_ClearITPendingBit(EXTI_LINE);
}

