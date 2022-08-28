#ifndef __BPS_EXTI_H__
#define __BPS_EXTI_H__

#include "stm32f10x.h" 

#define             EXTI_GPIO_CLK                        (RCC_APB2Periph_GPIOA )     
#define             EXTI_GPIO_PORT                       GPIOA   
#define             EXTI_GPIO_PIN                        GPIO_Pin_11
#define             EXTI_SOURCE_PORT                     GPIO_PortSourceGPIOA
#define             EXTI_SOURCE_PIN                      GPIO_PinSource11
#define             EXTI_LINE                            EXTI_Line11
#define             EXTI_IRQ                             EXTI15_10_IRQn
#define             EXTI_INT_FUNCTION                    EXTI15_10_IRQHandler
void EXTI_Config(void);
void EnableInvInterrupt(void);
void DisableInvInterrupt(void);
#endif
