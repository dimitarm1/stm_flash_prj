/*
 * common_funcs.c
 *
 *  Created on: Apr 1, 2018
 *      Author: didi
 */

#include "common_funcs.h"
void pinMode(GPIO_TypeDef* port, uint16_t pin_num, char pin_mode)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Pin = pin_num;
	  GPIO_InitStruct.Mode = pin_mode;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void digitalWrite(GPIO_TypeDef* port, uint16_t pin_number, unsigned char  value)
{
	 HAL_GPIO_WritePin(port, 1<<pin_number, (GPIO_PinState) value);
}

void shiftOut(GPIO_TypeDef* data_port, uint16_t data_pin, GPIO_TypeDef* clk_port, uint16_t clk_pin, unsigned char data)
{
	unsigned char i,val;
	for(i = 0; i< 8; i++)
	{
		HAL_GPIO_WritePin(clk_port, 1<<clk_pin, (GPIO_PinState) 0);
		val = ((data & 0x80) != 0);
		data = data << 1;
		HAL_GPIO_WritePin(data_port, 1<<data_pin, (GPIO_PinState) val);
		HAL_GPIO_WritePin(clk_port, 1<<clk_pin, (GPIO_PinState) 1);
	}
}

unsigned char shiftInOut(GPIO_TypeDef* data_out_port, uint16_t data_out_pin, GPIO_TypeDef* data_in_port, uint16_t data_in_pin, GPIO_TypeDef* clk_port, uint16_t clk_pin, unsigned char data)
{
	unsigned char i,val;
	unsigned char res = 0;
	for(i = 0; i< 8; i++)
	{
		HAL_GPIO_WritePin(clk_port, 1<<clk_pin, (GPIO_PinState) 0);
		val = ((data & 0x80) != 0);
		data = data << 1;
		HAL_GPIO_WritePin(data_out_port, 1<<data_out_pin, (GPIO_PinState) val);
		HAL_GPIO_WritePin(clk_port, 1<<clk_pin, (GPIO_PinState) 1);
		res = (res << 1) | HAL_GPIO_ReadPin(data_in_port, 1<<data_in_pin);
	}
	return res;
}
