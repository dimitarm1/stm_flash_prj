/**
 ******************************************************************************
 * @file    IO_Toggle/main.c
 * @author  MCD Application Team
 * @version V1.0.0
 * @date    23-March-2012
 * @brief   Main program body
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
 *
 * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
 * You may not use this file except in compliance with the License.
 * You may obtain a copy of the License at:
 *
 *        http://www.st.com/software_license_agreement_liberty_v2
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "defines.h"
#include "init.h"

#include "stm32f0xx_iwdg.h"



/** @addtogroup STM32F0_Discovery_Peripheral_Examples
 * @{
 */

/** @addtogroup IO_Toggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define BSRR_VAL        0x0300


/* Private macros ------------------------------------------------------------*/




/* Private variables ---------------------------------------------------------*/


 const uint32_t eeprom_array[512/4] __attribute__ ((section (".eeprom1text")));
 __IO uint32_t TimingDelay;

typedef enum states {state_show_time,state_set_time,state_show_hours,state_enter_service,state_clear_hours,state_address,state_pre_time,state_cool_time,state_ext_mode}states;
 states state;
typedef enum modes {mode_null,mode_clear_hours,mode_set_address,mode_set_pre_time,mode_set_cool_time}modes;
 modes service_mode;
 unsigned char controller_address = 0x0f;
 int curr_status;
 int prev_status;
 int curr_time;
 int flash_mode = 0;

 uint16_t data = 0;
unsigned char  time_to_set = 0;
unsigned int   work_hours[3] = {9,10,30}; //HH HL MM - Hours[2], Minutes[1]
unsigned char  preset_pre_time = 7;
unsigned char  preset_cool_time = 3;
int start_counter = 0;
int last_button = 0;
int prev_button = 0;
int display_data;
int pre_time, main_time, cool_time;
unsigned int Gv_miliseconds = 0;
int Gv_UART_Timeout = 500; // Timeout in mSec
int pre_time_sent = 0, main_time_sent = 0, cool_time_sent = 0;
int rx_state= 0;
int percent_aquafresh = 0, percent_licevi = 0, percent_fan1 = 0, percent_fan2 = 0;
int minute_counter =0;
int zero_crossed = 0;
int aqua_fresh_level = 0;
int volume_level = 5;
int fan_level = 7;

const char tim_pulse_array[] = {1,3,6,9,12,14,17,19,21,23};
 char digits[3];
// for Display:
 int refresh_counter = 0;
 int flash_counter = 0;
// for Display:
 int led_counter = 0;
 int digit_num = 0;
typedef struct time_str{
	uint8_t used_flag :8;
	uint8_t hours_h   :8;
	uint8_t hours_l   :8;
	uint8_t minutes   :8;
}time_str;
typedef struct settings_str{
	uint8_t addresse  :8;
	uint8_t pre_time  :8;
	uint8_t cool_time :8;
	uint8_t unused    :8;
}settings_str;
typedef struct flash_struct{
	time_str time;
	settings_str settings;
}flash_struct;

/* Private function prototypes -----------------------------------------------*/
void SystickDelay(__IO uint32_t nTime);
int ToBCD(int value);
void write_eeprom(void);
void read_eeprom(void);
void TimingDelay_Decrement(void);
void ProcessButtons(void);
void update_status(void);
void set_lamps(int value);
void set_colarium_lamps(int value);
void set_fan1(int value);
void set_fan2(int value);
void set_aquafresh(int value);

/* Global variables ----------------------------------------------------------*/

__IO uint32_t Gv_SystickCounter;


SPI_InitTypeDef 			SPI_InitStruct;
TIM_TimeBaseInitTypeDef  	TIM_TimeBaseStructure;
TIM_OCInitTypeDef  			TIM_OCInitStructure;
TIM_ICInitTypeDef  			TIM_ICInitStructure;
GPIO_InitTypeDef        	GPIO_InitStructure;
EXTI_InitTypeDef   			EXTI_InitStructure;
USART_InitTypeDef 			USART_InitStructure;
USART_ClockInitTypeDef 		USART_ClockInitStruct;
NVIC_InitTypeDef 			NVIC_InitStructure;

/* Private functions ---------------------------------------------------------*/
void Delay(__IO uint32_t nTime)
{
	TimingDelay = nTime;

	while(TimingDelay != 0);
}

void show_level(int level){
	level = level & 0x0F;
	//Digits and bars
	GPIOA->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5 | GPIO_BSRR_BR_8 | GPIO_BSRR_BR_11 | GPIO_BSRR_BR_12;
	GPIOB->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_1 | GPIO_BSRR_BR_2 | GPIO_BSRR_BR_12 | GPIO_BSRR_BR_13 | GPIO_BSRR_BR_14 | GPIO_BSRR_BR_15;
	GPIOC->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5 | GPIO_BSRR_BR_6 | GPIO_BSRR_BR_7 | GPIO_BSRR_BR_10 | GPIO_BSRR_BR_15 ;
	GPIOF->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5;

	// LEDs and bars
	GPIOA->BSRR = GPIO_BSRR_BR_8 | GPIO_BSRR_BR_11 | GPIO_BSRR_BR_12;
	GPIOC->BSRR = GPIO_BSRR_BR_10;

	switch (level) {
	case 0:

		break;
	case 1:
		GPIOA->BSRR = GPIO_BSRR_BS_8 ;
		GPIOC->BSRR = GPIO_BSRR_BS_10;
		break;
	case 2:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOC->BSRR = GPIO_BSRR_BS_10;
		break;
	case 3:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOC->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10;
		break;
	case 4:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR =  GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13;
		GPIOC->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10;
		break;
	case 5:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR =  GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10;
		GPIOF->BSRR = GPIO_BSRR_BS_4 ;
		break;
	case 6:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR =  GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 7:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10 | GPIO_BSRR_BS_15;
		GPIOF->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 8:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10 | GPIO_BSRR_BS_15;
		GPIOF->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 9:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_0 |GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10 | GPIO_BSRR_BS_15;
		GPIOF->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 0x0A:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15 ;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_15;
		GPIOF->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	}
}

void show_digit(int digit){
	digit = digit & 0x0F;
	//Digits and bars
	GPIOA->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5 | GPIO_BSRR_BR_8 | GPIO_BSRR_BR_11 | GPIO_BSRR_BR_12;
	GPIOB->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_1 | GPIO_BSRR_BR_2 | GPIO_BSRR_BR_12 | GPIO_BSRR_BR_13 | GPIO_BSRR_BR_14 | GPIO_BSRR_BR_15;
	GPIOC->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5 | GPIO_BSRR_BR_6 | GPIO_BSRR_BR_7 | GPIO_BSRR_BR_10 | GPIO_BSRR_BR_15 ;
	GPIOF->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5;

	// LEDs and bars
	GPIOA->BSRR = GPIO_BSRR_BR_8 | GPIO_BSRR_BR_11 | GPIO_BSRR_BR_12;
	GPIOC->BSRR = GPIO_BSRR_BR_10;

	switch (digit) {
	case 0:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
		GPIOB->BSRR = GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;

		break;
	case 1:
		GPIOA->BSRR = GPIO_BSRR_BS_3 ;
		GPIOB->BSRR = GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13;
		GPIOC->BSRR = GPIO_BSRR_BS_0;
		GPIOF->BSRR = GPIO_BSRR_BS_4 ;
		break;
	case 2:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 3:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_5 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 4:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4;
		break;
	case 5:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		GPIOF->BSRR = GPIO_BSRR_BS_5;
		break;
	case 6:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_5;
		break;
	case 7:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = GPIO_BSRR_BS_0;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 8:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 9:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 0x0A:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 0x0B:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = 0 ;
		break;
	case 0x0C:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_5;
		break;
	case 0x0D:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = 0 ;
		break;
	case 0x0E:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_5;
		break;
	case 0x0F:
	default:
		//empty
		//                GPIOA->BSRR = GPIO_BSRR_BS_3 ;
		//                GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		//                GPIOC->BSRR = GPIO_BSRR_BS_5;
		//                GPIOF->BSRR = GPIO_BSRR_BS_4;
		break;
	}
}

/*
 * Outputs:
 * PA2 - Digit 0
 * PA1 - Digit 1
 * PA0 - Digit 2
 * PB2,PB1 - a
 * PC6,PC7 - b
 * PC4,PA7 - c
 * PA4,PF5 - d
 * PF4, PA3 - e
 * PB11,PB10 - f
 * PB0,PC5 - g
 * PA6,PA5 - DP
 *
 *
 * A
 * F B
 * G
 * E C
 * D
 * DP
 */

/**
 * @brief  Main program.
 * @param  None
 * @retval None
 */
int main(void)
{

	init_periph();

	if (SysTick_Config(SystemCoreClock / (1000))){
		while(1); // Capture error
	}
	NVIC_SetPriority (SysTick_IRQn, 3);

	/*
	 * commads:
	 * 0 - status 0-free, 1-Working, 2-COOLING, 3-WAITING
	 * 1 - start
	 * 2 - set pre-time
	 * 3 - set cool-time
	 * 4 - stop - may be not implemented in some controllers
	 * 5 - set main time
	 */

	digits[0] = 0;
	digits[1] = 1;
	digits[2] = 2;
	prev_status = 0;
	state = state_show_time;
	pre_time = 0;
	main_time = 0;
	cool_time = 0;
	read_eeprom();
	if(!preset_pre_time || ! preset_cool_time){
		preset_pre_time = 7;
		preset_cool_time = 3;
		write_eeprom();
	}
	GPIOA->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2;
	update_status();
	while (1)
	{
		//		if(USART_GetFlagStatus(USART1, USART_FLAG_BUSY)){
		//			while(1);
		//			preset_pre_time = 7;

		//		}
		//		static int data =  0;
		//		static int lastdata = 0;
		//		data = USART_ReceiveData(USART1);
		//		if(data != lastdata){
		//			lastdata = data;
		//		}
		//		int delay_counter = 0;
		//		IWDG_ReloadCounter();
		//		for (delay_counter = 0; delay_counter<500; delay_counter++);

		ProcessButtons();
		if(pre_time || main_time || cool_time) state = state_show_time; // Working now. Other functions disabled
		switch (state){
		case state_show_time:
		case state_set_time:
			if(!pre_time && ! main_time && !cool_time ){
				flash_mode = 0;
				state = state_set_time;
				display_data = ToBCD(time_to_set);
				//				display_data = ToBCD(last_button);
			} else {
				state = state_show_time;
				//				  time_to_set = 0;
				display_data = ToBCD(main_time);
				//				display_data = ToBCD(last_button);
			}
			break;
		case state_show_hours:

			if( flash_mode != 3){
				flash_mode = 3; // All flashing
			}
			{
				int index = (flash_counter/8)%4;
				if(index<3){
					display_data =  ToBCD(work_hours[index]);
				}
				else {
					display_data = 0x0FF;
				}
			}
			break;
		case state_enter_service:
			display_data = service_mode|0xF0;
			flash_mode = 0;
			break;

		case state_clear_hours:
			flash_mode = 3;
			display_data = 0x00C;
			break;
		case state_address:
			flash_mode = 0;
			display_data = 0xA;
			break;
		case state_pre_time:
			flash_mode = 2;
			display_data = 0x3 | preset_pre_time;;
			break;
		case state_cool_time:
			flash_mode = 2;
			display_data = 0x4 | preset_cool_time;
			break;
		case state_ext_mode:
			flash_mode = 0;
			display_data = 0x5;
			break;
		}
		//		show_digit(display_data);
		if (state == state_show_time){
			if(!(pre_time || main_time || cool_time)){
				// Some Paranoia...
				// turn off all
				set_lamps(0);
				set_colarium_lamps(0);
				percent_fan1 = 0;
				set_fan1(0);
				set_fan2(0);
				set_aquafresh(0);
//				state = state_set_time;
			}
		}
		//
		//		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); // wAIT UNTIL RX BUFFER IS EMPTY
		//		int data =  USART_ReceiveData(USART1);
		//		USART_SendData(USART1,0x80);
		//				while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY
		if(++led_counter>130){
			static int current_button_read = 0;
//			if(flash_counter%80 == 0){
//						data = (STATUS_COOLING<<6)|4;
//						USART_SendData(USART1,data);
//			}
			led_counter = 0;
			digit_num++;
			flash_counter++;
			//		aqua_fresh_level = 0;
			if(digit_num>4) {
				digit_num = 0;
				last_button = current_button_read;
				current_button_read = 0;
			}
			GPIOA->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_2 | GPIO_BSRR_BR_8 | GPIO_BSRR_BR_11 | GPIO_BSRR_BR_12; // Turn off the lights while changing them
			GPIOB->BSRR = GPIO_BSRR_BR_4;
			GPIOC->BSRR = GPIO_BSRR_BR_10 | GPIO_BSRR_BR_13 ;
			GPIOF->BSRR = GPIO_BSRR_BR_1 ;
			GPIOA->BSRR = GPIO_BSRR_BR_8 | GPIO_BSRR_BR_11 | GPIO_BSRR_BR_12;
			if(digit_num<3){
				show_digit(((display_data & 0xFFF)& (0x0F<<(digit_num*4)))>>(digit_num*4));
//				show_digit(last_button>>(digit_num*4));
			} else {
				show_digit(0x8);// DEBUG

			}
			if(flash_mode < 3 ||(flash_counter & 0x40)){

				// Indicate pre_time by movind bars
				if (pre_time){
					volume_level = (flash_counter>>5) & 0x07;
					fan_level = ((flash_counter>>5) & 0x07) +3;
				}
				else {

				}

				switch (digit_num){
				case 0:
					GPIOA->BSRR = GPIO_BSRR_BS_2 ;
					//set Aqua 2 LED
					if(aqua_fresh_level>0){
						GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12;
						GPIOC->BSRR = GPIO_BSRR_BS_10 ;
					}
					current_button_read |= ((!!(GPIOC->IDR & GPIO_IDR_11)) | ((!!(GPIOC->IDR & GPIO_IDR_12))<<1) | \
							((!!(GPIOD->IDR & GPIO_IDR_2))<<2))<<0;
					break;

				case 1:
					GPIOB->BSRR = GPIO_BSRR_BS_4 ;
					//set Aqua 2 LEDs
					if(aqua_fresh_level>1){
						GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12;
						GPIOC->BSRR = GPIO_BSRR_BS_10 ;
					}
					current_button_read |= ((!!(GPIOC->IDR & GPIO_IDR_11)) | ((!!(GPIOC->IDR & GPIO_IDR_12))<<1) | \
							((!!(GPIOD->IDR & GPIO_IDR_2))<<2))<<4;

					break;
				case 2:
					GPIOA->BSRR = GPIO_BSRR_BS_0 ;
					//set Aqua 1 LEDs
					if(aqua_fresh_level>1){
						GPIOA->BSRR = GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12;
					}
					current_button_read |= ((!!(GPIOC->IDR & GPIO_IDR_11)) | ((!!(GPIOC->IDR & GPIO_IDR_12))<<1) | \
							((!!(GPIOD->IDR & GPIO_IDR_2))<<2))<<8;
					break;
				case 3:
					GPIOC->BSRR = GPIO_BSRR_BS_13 ;
					show_level(volume_level);

					break;
				case 4:
				default:
					GPIOF->BSRR = GPIO_BSRR_BS_1 ;
					show_level(fan_level);
					break;
				}
			}
			if (((flash_mode == 1)&& digit_num == 0 && (flash_counter & 0x40)) || ((flash_mode == 2) && (digit_num == 2))){
				GPIOA->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BS_5;
			}
		}
		IWDG_ReloadCounter(); //DEBUG
	}
}

void TIM2_IRQHandler() {
	if((TIM2->SR & TIM_SR_UIF) != 0) // If update flag is set
	{
		TIM2->SR &= ~TIM_SR_UIF; // Interrupt has been handled }
		if(TIM_ICInitStructure.TIM_ICPolarity == TIM_ICPolarity_Rising)
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
		else TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
		TIM_ICInit(TIM2, &TIM_ICInitStructure);
	}
	zero_crossed = 1;

}

int ToBCD(int value){
	int digits[3];
	int result;
	digits[0] = value %10;
	digits[1] = (value/10) % 10;
	digits[2] = (value/100) % 10;
	result = digits[0] | (digits[1]<<4) | (digits[2]<<8);
	return result;
}

int FromBCD(int value){
	int digits[3];
	int result;
	digits[0] = value & 0x0F;
	digits[1] = (value>>4) & 0x0F;
	digits[2] = (value>>8) & 0x0F;
	result = digits[0] + digits[1]*10 + digits[2]*100;
	return result;
}


/**
 * @brief  Manage the activity on buttons
 * @param  None
 * @retval None
 */
void ProcessButtons(void)
{

	if (last_button ){
		if(last_button != prev_button){
			switch(last_button){
			case BUTTON_START:
				if( pre_time ){
					//send_start();
					Gv_miliseconds = 0;
					pre_time = 0;
					state = state_show_time;
					update_status();
				}
				if(!pre_time && ! main_time && !cool_time) {
					if(time_to_set){
						// Send of time moved elsewhere
						pre_time = preset_pre_time;
						main_time = time_to_set;
						cool_time = preset_cool_time;
						state = state_show_time;
						time_to_set = 0;
						Gv_miliseconds = 0;
						update_status();
					} else {
						if (state > state_enter_service){
							// Write EEPROM
							if(state == state_clear_hours){
								work_hours[0] = 0;
								work_hours[1] = 0;
								work_hours[2] = 0;
							}
//							write_eeprom();
//							read_eeprom();
							start_counter = 0;
							service_mode = mode_null;
						} else {
							start_counter = START_COUNTER_TIME;
						}
						state = state_show_hours;
					}
				}
				break;
			case BUTTON_STOP:
				if(curr_status == STATUS_WORKING){
					main_time = 0;
				}
				break;
			case BUTTON_FAN_PLUS:
				if(curr_status == STATUS_WORKING){
					if(fan_level<10){
						fan_level++;
						percent_fan1 = fan_level;
						set_fan1(percent_fan1);
					}
				}
				break;
			case BUTTON_FAN_MINUS:
				if(curr_status == STATUS_WORKING){
					if(fan_level>0){
						fan_level--;
						percent_fan1 = fan_level;
						set_fan1(percent_fan1);
					}
				}
				break;
			case BUTTON_AQUA:
				if(minute_counter){
				}
				break;
			case BUTTON_VOL_PLUS:
				if(curr_status == STATUS_WORKING){
					if(volume_level<10){
						volume_level++;
					}
				}
				break;
			case BUTTON_VOL_MINUS:
				if(curr_status == STATUS_WORKING){
					if(volume_level>0){
						volume_level--;
					}
				}
				break;
			case BUTTON_PLUS:
			{

				if(state == state_show_hours) {
					state = state_set_time;
					start_counter = 0;
				}
				if(state == state_set_time){
					if(time_to_set < 25){
						if(!Gv_UART_Timeout) time_to_set++;
					}
				}
				else if(state > state_enter_service){
					start_counter = EXIT_SERVICE_TIME;
					switch (service_mode){
					case mode_set_address:
						break;
					case mode_set_pre_time:
						if(preset_pre_time<9) preset_pre_time++;
						break;
					case mode_set_cool_time:
						if(preset_cool_time<9) preset_cool_time++;
						break;
					default:
						break;
					}
				}

			}
			break;
			case BUTTON_MINUS:
				if(state == state_show_hours) {
					state = state_set_time;
					start_counter = 0;
				}
				if(state == state_set_time){
					if(time_to_set) {
						if(!Gv_UART_Timeout) time_to_set--;
					}
				}else if(state > state_enter_service){
					start_counter = EXIT_SERVICE_TIME;
					switch (service_mode){
					case mode_set_address:
						break;
					case mode_set_pre_time:
						if(preset_pre_time) preset_pre_time--;
						break;
					case mode_set_cool_time:
						if(preset_cool_time) preset_cool_time--;
						break;
					default:
						break;
					}
				}
				//				if(selected_led_bits & LED_FAN2_L){
				//					if(percent_fan2) percent_fan2 = 0;
				//					set_fan2(percent_fan2);
				//				} else if(selected_led_bits & LED_FAN1_L){
				//					if(percent_fan1) percent_fan1-=25;
				//					set_fan1(percent_fan1);
				//				}
				//				else if(selected_led_bits & LED_LICEVI_L){
				//					if(percent_licevi) percent_licevi=0;
				//					set_licevi_lamps(percent_licevi);
				//					update_status();
				//				}

				break;
			default:
				while(0);
			}
			prev_button =  last_button;
		}
	} else {
		prev_button = last_button;
	}


	if (last_button == BUTTON_START)
	{
		// LED1_ON;
		if(start_counter< (START_COUNTER_TIME+ ENTER_SERVICE_DELAY + 6*SERVICE_NEXT_DELAY)) start_counter++;
		if(start_counter== START_COUNTER_TIME + ENTER_SERVICE_DELAY){
			if(curr_status == STATUS_FREE &&(state < state_enter_service))
			{
				state = state_enter_service;
				service_mode = mode_clear_hours; // Clear Hours
			}
		}
		if(state == state_enter_service){
			if(start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 1*SERVICE_NEXT_DELAY){
				service_mode = mode_set_address; //
			}
			else if(start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 2*SERVICE_NEXT_DELAY){
				service_mode = mode_set_pre_time; //
			}
			else if(start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 3*SERVICE_NEXT_DELAY){
				service_mode = mode_set_cool_time; //
			}
		}
		if(time_to_set && state == state_set_time && start_counter == START_DELAY){
			// Do start
			state = state_show_time;
			//			send_time();
			//			start_counter = 0;
		}
	}
	else
	{
		if(start_counter){
			if(state == state_show_hours){
				start_counter--;
				if(!start_counter){
					state = state_show_time;
				}
			}
			else if(state >= state_enter_service){
				if(state == state_enter_service){
					state = service_mode + state_enter_service;
				}
				start_counter--;
				if(!start_counter){
					state = state_show_time;
				}
			}
			else {
				start_counter = 0;

			}
		}
		if(prev_status != curr_status){
			if (prev_status == STATUS_WAITING && curr_status == STATUS_WORKING){
				work_hours[2]+=time_to_set;
				if(work_hours[2]>59){
					work_hours[2]=work_hours[2]-60;
					work_hours[1]++;
					if(work_hours[1]>99){
						work_hours[1] = 0;
						work_hours[0]++;
					}
				}
				write_eeprom();
				time_to_set = 0;
				percent_aquafresh = 0, percent_licevi = 100, percent_fan1 = 0, percent_fan2 = 100;
				zero_crossed = 0;
				volume_level = 6;
				fan_level = 7;
				percent_fan1 = fan_level * 10;
				set_lamps(100);
				set_colarium_lamps(percent_licevi);
				set_fan1(percent_fan1);
				set_fan2(percent_fan2);
				set_aquafresh(percent_aquafresh);
			}
			if (curr_status == STATUS_COOLING){
				percent_licevi = 0, percent_fan1 = 100, percent_fan2 = 100;
//				set_lamps(0);
				set_colarium_lamps(percent_licevi);
				set_fan1(percent_fan1);
				set_fan2(percent_fan2);
				set_aquafresh(percent_aquafresh);
			}
			if (curr_status == STATUS_FREE){
				percent_aquafresh = 0, percent_licevi = 0, percent_fan1 = 0, percent_fan2 = 0;
//				set_lamps(0);
				set_colarium_lamps(percent_licevi);
				set_fan1(percent_fan1);
				set_fan2(percent_fan2);
				set_aquafresh(percent_aquafresh);
				minute_counter = 0;
			}
			prev_status = curr_status;

		}
		//    LED1_OFF;
	}
}



/**
 * @brief  Add a delay using the Systick
 * @param  nTime Delay in milliseconds.
 * @retval None
 */
void SystickDelay(__IO uint32_t nTime)
{

	Gv_SystickCounter = nTime;
	while (Gv_SystickCounter != 0)
	{
		// The Gv_SystickCounter variable is decremented every ms by the Systick interrupt routine
	}

}

void update_status(void){
	if(pre_time) {
		curr_time = pre_time;
		curr_status = STATUS_WAITING;
	}
	else if(main_time) {
		curr_time = main_time;
		curr_status = STATUS_WORKING;
	}
	else if(cool_time) {
		curr_time = cool_time;
		curr_status = STATUS_COOLING;
	}
	else {
		curr_time = 0;
		curr_status = STATUS_FREE;
	}
}
void TimingDelay_Decrement(void)
{
	if (Gv_SystickCounter != 0x00)
	{
		Gv_SystickCounter--;
	}
	if( ++ refresh_counter>200){
		refresh_counter = 0;
		flash_counter++;
	}
	if(Gv_miliseconds++>60000){
		Gv_miliseconds = 0;
		if (pre_time)pre_time--;
		else if (main_time) {
			main_time--;
			minute_counter ++;
		}
		else if (cool_time) cool_time--;
		update_status();
	}
	if  (Gv_UART_Timeout){
		Gv_UART_Timeout--;
		if(! Gv_UART_Timeout) {
			rx_state = 0;
			pre_time_sent = 0, main_time_sent = 0, cool_time_sent = 0;
		}
	}
//	if(zero_crossed) zero_crossed-=10;
//	if(percent_fan1) set_fan1(zero_crossed && (!(zero_crossed > percent_fan1)));
}


void read_eeprom(void){
	int index = 0;
	flash_struct *flash_mem;
	while((eeprom_array[index]!=0xFFFFFFFFUL)&&(index<(512/4)))index+=2;
	if(index == 0){
		// Load defaults
		flash_mem = 0;
		preset_pre_time = 7;
		preset_cool_time = 3;
		work_hours[0] = 0;
		work_hours[1] = 0;
		work_hours[2] = 0;
		return;
	}
	index-=2;
	flash_mem = (flash_struct*)&eeprom_array[index];
	preset_pre_time = flash_mem->settings.pre_time ;
	preset_cool_time = flash_mem->settings.cool_time;
	work_hours[0] = flash_mem->time.hours_h;
	work_hours[1] = flash_mem->time.hours_l;
	work_hours[2] = flash_mem->time.minutes;
}

void write_eeprom(void){
	int index = 0;
	FLASH_Unlock();
	volatile flash_struct flash_mem;
	uint32_t *p = (uint32_t *)&flash_mem;
	while((eeprom_array[index]!=0xFFFFFFFFUL)&&(index<512))index+=2;
	if(index == 512){
		// No more room. Erase the 4 pages
		FLASH_ErasePage((uint32_t)&eeprom_array[0]);
		FLASH_ErasePage((uint32_t)&eeprom_array[128]);
		FLASH_ErasePage((uint32_t)&eeprom_array[256]);
		FLASH_ErasePage((uint32_t)&eeprom_array[384]);
		index = 0;
	}

	while((eeprom_array[index]!=0xFFFFFFFFUL)&&(index<512))index++;
	if(index == 512){
		display_data = 0xE01;
		for (index = 0; index<20; index++){
		}
	} else {

		flash_mem.settings.pre_time = preset_pre_time;
		flash_mem.settings.cool_time = preset_cool_time;
		flash_mem.settings.addresse = controller_address;
		flash_mem.settings.unused = 0x55;
		flash_mem.time.hours_h = work_hours[0];
		flash_mem.time.hours_l = work_hours[1];
		flash_mem.time.minutes = work_hours[2];
		flash_mem.time.used_flag = 0;
		FLASH_ProgramWord((uint32_t)&eeprom_array[index],p[0]);
		FLASH_ProgramWord((uint32_t)&eeprom_array[index+1],p[1]);
		index = sizeof(flash_mem);
		index ++;
	}
	//	FLASH_Lock();
}
void set_lamps(int value){
	//	while(!zero_crossed);
	if (value)	GPIOC->BSRR = GPIO_BSRR_BS_8;
	else GPIOC->BRR = GPIO_BSRR_BS_8;
}
void set_colarium_lamps(int value){
	//	while(!zero_crossed);
	if (value)	GPIOC->BSRR = GPIO_BSRR_BS_9;
	else GPIOC->BRR = GPIO_BSRR_BS_9;
}
void set_fan2(int value){
	if (value)
		GPIOC->BSRR = GPIO_BSRR_BS_14;
	else
		GPIOC->BSRR = GPIO_BSRR_BR_14;
}

void set_fan1(int value){
//	uint32_t tim_base=5;
	TIM_Cmd(TIM2, DISABLE);

	if (value ) TIM_OCInitStructure.TIM_Pulse = tim_pulse_array[value+1];
	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
//	  One Pulse value = (TIM_Period - TIM_Pulse)/TIM2 counter clock
	TIM_TimeBaseStructure.TIM_Period =  TIM_OCInitStructure.TIM_Pulse + 6;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIM2 PWM2 Mode configuration: Channel4 */
	//for one pulse mode set PWM2, output enable, pulse (1/(t_wanted=TIM_period-TIM_Pulse)), set polarity high
	if (value)	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	else 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	//TIM_Pulse defines the delay value
//	if (TIM_TimeBaseStructure.TIM_Period) TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period-3;
//	else  TIM_OCInitStructure.TIM_Pulse = 3;


	TIM_Cmd(TIM2, ENABLE);
}

void set_aquafresh(int value){
	return;
	if (value)	GPIOB->BSRR = GPIO_BSRR_BS_5;
	else GPIOB->BRR = GPIO_BSRR_BS_5;
}

void usart_receive(void){

	enum rxstates {state_none, state_pre_time, state_main_time, state_cool_time, state_get_checksum};
	//	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
	data =  USART_ReceiveData(USART1);
	Gv_UART_Timeout = 500;
	//pre_time_sent = 0, main_time_sent = 0, cool_time_sent = 0;

	if ((data & 0x80)){
		// Command
		if((data == (0x80U | ((controller_address & 0x0fU)<<3U)))){
			data = (curr_status<<6)|curr_time;
//			data = (STATUS_WORKING<<6)|4;
			USART_SendData(USART1,data);
		}
		else if (data == (0x80U | ((controller_address & 0x0fU)<<3U) | 1U)) //Command 1 - Start
		{
			pre_time = 0;
			update_status();
		}
		else if (data == (0x80U | ((controller_address & 0x0fU)<<3U) | 2U)) //Command 2 == Pre_time_set
		{
			rx_state = state_pre_time;
		}
		else if (data == (0x80U | ((controller_address & 0x0fU)<<3U) | 5U)) //Command 5 == Main_time_set
		{
			rx_state = state_main_time;
		}
		else if (data == (0x80U | ((controller_address & 0x0fU)<<3U) | 3U)) //Command 3 == Cool_time_set
		{
			rx_state = state_cool_time;
		}

	} else if (rx_state){
		// payload
		int time_in_hex = ToBCD(main_time_sent);
		if(rx_state == state_get_checksum){
			unsigned char checksum = (pre_time_sent + cool_time_sent  - time_in_hex - 5) & 0x7F;
			if(	data == checksum){
				pre_time = pre_time_sent;
				main_time = main_time_sent;
				cool_time = cool_time_sent;
				update_status();
				Gv_miliseconds = 0;
			}
			rx_state = 0;
		}
		if(rx_state == state_pre_time){
			pre_time_sent = data;
			rx_state = 0;
		}
		if(rx_state == state_main_time){
			main_time_sent = data;
			rx_state = 0;
		}
		if(rx_state == state_cool_time){
			cool_time_sent = data;
			rx_state = state_get_checksum;
			unsigned char checksum = (pre_time_sent + cool_time_sent  - time_in_hex - 5) & 0x7F;
			data = checksum;
			USART_SendData(USART1,data);
		}


	}
	//	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	//	USART_SendData(USART1,0x80);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
