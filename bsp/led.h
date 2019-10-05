#ifndef __LED_H
#define __LED_H

#include "stm32f10x.h"
#include "task.h"

#define LED                     1

#define LED_GPIO_RCC           RCC_APB2Periph_GPIOE
#define LED_GPIO_PORT          GPIOE
#define LED0_GPIO_PIN      			GPIO_Pin_0
#define LED1_GPIO_PIN      			GPIO_Pin_1
#define LED2_GPIO_PIN      			GPIO_Pin_2

#define LED_ONOFF(x,GPIO_PIN)     			GPIO_WriteBit(GPIOE,GPIO_PIN,x);

void LED_GPIO_Config(void);	
void LED_Toggle(void);
void LED_SELECT(u8 AircraftMode);
void TIM6_Config(void);
//void NVIC_Config(void);
void TIM6_IRQHandler(void);
#endif
