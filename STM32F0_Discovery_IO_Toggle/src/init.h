/*
 * init.h
 *
 *  Created on: 30.11.2013
 *      Author: didi
 */

#ifndef INIT_H_
#define INIT_H_

#include "stm32f0xx.h"

#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"
#include "stm32f0xx_flash.h"
#include "stm32f0xx_spi.h"
#include "stm32f0xx_tim.h"
#include "stm32f0xx_misc.h"
#include "stm32f0xx_exti.h"
#include "stm32f0xx_syscfg.h"


extern SPI_InitTypeDef 				SPI_InitStruct;
extern TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
extern TIM_OCInitTypeDef  			TIM_OCInitStructure;
extern TIM_ICInitTypeDef  			TIM_ICInitStructure;
extern GPIO_InitTypeDef        		GPIO_InitStructure;
extern USART_InitTypeDef 			USART_InitStructure;
extern USART_ClockInitTypeDef 		USART_ClockInitStruct;
extern NVIC_InitTypeDef 			NVIC_InitStructure;
extern EXTI_InitTypeDef 			EXTI_InitStructure;

extern void init_periph();


#endif /* INIT_H_ */
