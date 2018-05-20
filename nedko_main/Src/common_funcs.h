/*
 * common_funcs.h
 *
 *  Created on: Apr 1, 2018
 *      Author: didi
 */

#ifndef COMMON_FUNCS_H_
#define COMMON_FUNCS_H_
#include "stm32f1xx_hal.h"

#define HIGH 1
#define LOW 0
#ifndef NULL
	#define NULL 0
#endif
#define true 1
#define false 0

void pinMode(GPIO_TypeDef* port, uint16_t pin_num, char pin_mode);
void digitalWrite(GPIO_TypeDef* port, uint16_t pin_number, unsigned char  value);
void shiftOut(GPIO_TypeDef* data_port, uint16_t data_pin, GPIO_TypeDef* clk_port, uint16_t clk_pin, unsigned char data);
unsigned char shiftInOut(GPIO_TypeDef* data_out_port, uint16_t data_out_pin, GPIO_TypeDef* data_in_port, uint16_t data_in_pin, GPIO_TypeDef* clk_port, uint16_t clk_pin, unsigned char data);

#endif /* COMMON_FUNCS_H_ */
