#include "bps_exti.h"


static void NVIC_Config(void)//�ú���ֻ��������ļ����ݲ��ܱ����������ļ����ò���
{
	NVIC_InitTypeDef NVIC_InitStruct;//���ñ���
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);//����Ϊ��1�������ȼ�Ϊ1bit �����ȼ�Ϊ3bit��
	NVIC_InitStruct.NVIC_IRQChannel=EXTI_IRQ;//����������Դ
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority=0;
	NVIC_InitStruct.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStruct);
}

void EXTI_Config(void)
{
	EXTI_InitTypeDef EXTI_InitStructure;
	GPIO_InitTypeDef GPIO_InitStruct;
	
	//�����ж����ȼ�
	NVIC_Config();
	
	//��ʼ��ʹ��GPIOA PA11Ϊ����״̬
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
	GPIO_InitStruct.GPIO_Pin=EXTI_GPIO_PIN;
	GPIO_InitStruct.GPIO_Mode=GPIO_Mode_IN_FLOATING;
	GPIO_Init(EXTI_GPIO_PORT, &GPIO_InitStruct);
	//��ʼ��EXTI����ѡ��ģʽ ������ʽ ����
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_AFIO,ENABLE);//��AFIOʱ��
	GPIO_EXTILineConfig(EXTI_SOURCE_PORT, EXTI_SOURCE_PIN); 
//	EXTI_DeInit();//�ָ�Ĭ��״̬
  EXTI_InitStructure.EXTI_Line = EXTI_LINE;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising; //�������ж�
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

