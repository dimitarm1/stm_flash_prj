/**
 ******************************************************************************
 * @file    IO_Toggle/main.c
 * @author  D. Marinov
 * @version V1.0.0
 * @date    30-06-2016
 * @brief   Main program body
 ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "defines.h"
#include "init.h"
#include "stdint.h"
#include "stm32f0xx_iwdg.h"
#include "eeprom.h"
#include "math.h"


/** @addtogroup STM32F0_Discovery_Peripheral_Examples
 * @{
 */

/** @addtogroup IO_Toggle
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
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
	 uint8_t ext_mode  :8;
	 uint8_t volume_dac:8;
	 uint8_t temperatue_max:8;
	 uint8_t unused_3  :8;
	 uint8_t checksum  :8;
 }settings_str;
 typedef struct flash_struct{
	 time_str time;
	 settings_str settings;
 }flash_struct;

typedef enum ext_modes {ext_mode_none, ext_mode_colarium}ext_modes;
typedef enum lamps_modes {lamps_all, lamps_uv, lamps_colagen}lamps_modes;
typedef enum states {state_show_time,state_set_time,state_show_hours,state_enter_service,state_clear_hours,state_address,state_pre_time,state_cool_time,state_ext_mode, state_volume, state_max_temp}states;
 states state;
typedef enum modes {mode_null,mode_clear_hours,mode_set_address,mode_set_pre_time,mode_set_cool_time, mode_set_ext_mode, mode_set_volume, mode_set_max_temp}modes;
modes service_mode;
typedef enum voice_messages { message_power_up, message_start_working, message_stop_working };
int useUart=0;


/* Private define ------------------------------------------------------------*/
#define BSRR_VAL        0x0300
//#define LICEVI_LAMPI_VMESTO_AQAFRESH // specialna porachka ot 28.02.2016 za star balgarski solarium

/* Private macros ------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
/* Virtual address defined by the user: 0xFFFF value is prohibited */
flash_struct flash_data;
uint16_t VirtAddVarTab[sizeof(flash_data)/2];
//uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0};
uint16_t VarValue = 0;


/* Private function prototypes -----------------------------------------------*/
void SystickDelay(__IO uint32_t nTime);
int ToBCD(int value);
void write_eeprom(void);
void read_eeprom(void);
void TimingDelay_Decrement(void);
void ProcessButtons(void);
void update_status(void);
void set_start_out_signal(int value);
void set_lamps(int value);
void set_colarium_lamps(int value);
void set_fan1(int value);
void set_fan2(int value);
void set_aquafresh(int value);
void set_volume(int value);
void play_message(int index);
void new_read_eeprom(void);
void new_write_eeprom(void);

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
DAC_InitTypeDef             DAC_InitStructure;


uint8_t dac_buffer[2][1024];
const uint32_t eeprom_array[512] __attribute__ ((section (".eeprom1text")));
const char tim_pulse_array[] = {68,63,58,52,46,40,33,26,20,1};
const uint32_t message_sector_offset[] = {00,100,200,300,400,500,600};
const uint32_t message_sector_counts[] = {90,90,90,90,90,90,90};

__IO uint32_t TimingDelay;

unsigned char controller_address = 0x0e;
int curr_status;
int prev_status;
int curr_time;
int flash_mode = 0;
int dac_out_counter;
int dac_ping_pong_state;
int dac_prev_ping_pong_state;
int dac_current_block;
int dac_current_message;
int fade_out_counter;
int fade_in_counter;
int silence_counter;
int colaruim_lamps_state;
int lamps_state;

uint16_t data = 0;
unsigned char  time_to_set = 0;
unsigned int   work_hours[3] = {0,0,0}; //HH HL MM - Hours[2], Minutes[1]
unsigned char  preset_pre_time = 7;
unsigned char  preset_cool_time = 3;
unsigned char  volume_message;
unsigned char  temperature_threshold;
unsigned char  ext_mode = 0;
unsigned char  lamps_mode = 0;
unsigned char  temperature_current;
unsigned char  last_rx_address;
int start_counter = 0;
int last_button = 0;
int prev_button = 0;
int display_data;
int pre_time, main_time, cool_time;
unsigned int Gv_miliseconds = 0;
int Gv_UART_Timeout = 1000; // Timeout in mSec
int pre_time_sent = 0, main_time_sent = 0, cool_time_sent = 0;
int rx_state= 0;
int percent_aquafresh = 0, percent_licevi = 0, percent_fan1 = 0, percent_fan2 = 0;
int minute_counter =0;
int zero_crossed = 0;
int aqua_fresh_level = 0;
volatile int volume_level = 5;
volatile int fan_level = 7;
uint32_t tim_base=7; // Timer 2 time delay
unsigned int external_read = 0;
static int startup_delay = 0;
static int stop_delay = 0;

char digits[3];
// for Display:
int refresh_counter = 0;
int flash_counter = 0;
// for Display:
int led_counter = 0;
int digit_num = 0;
__IO uint32_t LsiFreq = 40000;


/* Private functions ---------------------------------------------------------*/
void Delay(__IO uint32_t nTime)
{
	TimingDelay = nTime;

	while(TimingDelay != 0);
}

void show_level(int level){
	level = level & 0x0F;
	//Digits and bars
	GPIOA->BRR = GPIO_BRR_BR_4 | GPIO_BRR_BR_5 | GPIO_BRR_BR_8 | GPIO_BRR_BR_11 | GPIO_BRR_BR_12;
	GPIOB->BRR = GPIO_BRR_BR_0 | GPIO_BRR_BR_1 | GPIO_BRR_BR_2 | GPIO_BRR_BR_4 | GPIO_BRR_BR_12 | GPIO_BRR_BR_13 | GPIO_BRR_BR_14 | GPIO_BRR_BR_15;
	GPIOC->BRR = GPIO_BRR_BR_4 | GPIO_BRR_BR_5 | GPIO_BRR_BR_6 | GPIO_BRR_BR_7 | GPIO_BRR_BR_10 | GPIO_BRR_BR_15 ;
	GPIOF->BRR = GPIO_BRR_BR_0 | GPIO_BRR_BR_4 | GPIO_BRR_BR_5;

	// LEDs and bars
	GPIOA->BRR = GPIO_BRR_BR_8 | GPIO_BRR_BR_11 | GPIO_BRR_BR_12;
	GPIOC->BRR = GPIO_BRR_BR_10;

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
		GPIOB->BSRR = GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13;
		GPIOC->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10;
		break;
	case 5:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_4 |GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13;
		GPIOC->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10;
		GPIOF->BSRR = GPIO_BSRR_BS_4 ;
		break;
	case 6:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_4 |GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 7:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_4 |GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10 | GPIO_BSRR_BS_15;
		GPIOF->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 8:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_4 |GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10 | GPIO_BSRR_BS_15;
		GPIOF->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 9:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_4 |GPIO_BSRR_BS_0 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 |GPIO_BSRR_BS_10 | GPIO_BSRR_BS_15;
		GPIOF->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 0x0A:
		GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12 ;
		GPIOB->BSRR = GPIO_BSRR_BS_4 |GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15 ;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_15;
		GPIOF->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	}
}

void show_digit(int digit){
	digit = digit & 0x0F;
	//Digits and bars
	GPIOA->BRR = GPIO_BRR_BR_4 | GPIO_BRR_BR_5 | GPIO_BRR_BR_8 | GPIO_BRR_BR_11 | GPIO_BRR_BR_12;
	GPIOB->BRR = GPIO_BRR_BR_0 | GPIO_BRR_BR_1 | GPIO_BRR_BR_2 | GPIO_BRR_BR_4 | GPIO_BRR_BR_12 | GPIO_BRR_BR_13 | GPIO_BRR_BR_14 | GPIO_BRR_BR_15;
	GPIOC->BRR = GPIO_BRR_BR_4 | GPIO_BRR_BR_5 | GPIO_BRR_BR_6 | GPIO_BRR_BR_7 | GPIO_BRR_BR_10 | GPIO_BRR_BR_15 ;
	GPIOF->BRR = GPIO_BRR_BR_0 | GPIO_BRR_BR_4 | GPIO_BRR_BR_5;

	// LEDs and bars
	GPIOA->BRR = GPIO_BRR_BR_8 | GPIO_BRR_BR_11 | GPIO_BRR_BR_12;
	GPIOC->BRR = GPIO_BRR_BR_10;
	if(flash_mode == 3 && (flash_counter & 0x100)) return;
	switch (digit) {
	case 0:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
		GPIOB->BSRR = GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;

		break;
	case 1:
		GPIOA->BSRR = 0;
		GPIOB->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13;
		GPIOC->BSRR = 0;
		GPIOF->BSRR = GPIO_BSRR_BS_4 ;
		break;
	case 2:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 3:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = GPIO_BSRR_BS_5 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 4:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
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
		GPIOB->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14;
		GPIOC->BSRR = 0;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 8:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = 0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 9:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = 0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 0x0A:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5  ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13 | GPIO_BSRR_BS_14 | GPIO_BSRR_BS_15;
		GPIOC->BSRR = 0 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
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
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_12 | GPIO_BSRR_BS_13;
		GPIOC->BSRR = GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4;
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
volatile int stop=1;
int main(void)
{
	static uint8_t counter=0;
	SystemInit();
//	while (stop);
	new_read_eeprom();
	//	/* Unlock the Flash Program Erase controller */

	init_periph();
	DRESULT result = 0;

	if (SysTick_Config(SystemCoreClock / (1000))){
		while(1); // Capture error
	}
	NVIC_SetPriority (SysTick_IRQn, 3);

	/* IWDG timeout equal to 250 ms (the timeout may varies due to LSI frequency
		 dispersion) */
	/* Enable write access to IWDG_PR and IWDG_RLR registers */
	IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);

	/* IWDG counter clock: LSI/32 */
	IWDG_SetPrescaler(IWDG_Prescaler_32);

	/* Set counter reload value to obtain 250ms IWDG TimeOut.
		Counter Reload Value = 250ms/IWDG counter clock period
							  = 250ms / (LSI/32)
							  = 0.25s / (LsiFreq/32)
							  = LsiFreq/(32 * 4)
							  = LsiFreq/128
	   */
	IWDG_SetReload(LsiFreq/128);

	/* Reload IWDG counter */
	IWDG_ReloadCounter();

	/* Enable IWDG (the LSI oscillator will be enabled by hardware) */
	IWDG_Enable();
	//FLASH_Unlock();
	//	/* EEPROM Init */
	//EE_Init();
//	for (counter = 0; counter < sizeof(flash_data); counter++)
//	{
//		VirtAddVarTab[counter] = counter;
//	}
//	new_read_eeprom();
//
//
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
	dac_current_message = -1;
//	read_eeprom();
	if(!preset_pre_time || ! preset_cool_time){
		preset_pre_time = 7;
		preset_cool_time = 3;
		new_write_eeprom();// Paranoia check
	}
	GPIOA->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2;
	//GPIOC->BRR =  GPIO_BSRR_BS_3; // External sound
	GPIOC->BSRR =  GPIO_BSRR_BS_3; // Internal sound
	update_status();
	set_volume(0);

	play_message(message_power_up);

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

		// read external input
		if ((state < state_enter_service) && ((flash_counter>>4)&1)){
			if (controller_address == 15){
				external_read = (external_read << 1) | 	(!!(GPIOA->IDR & GPIO_IDR_10));
				if(!main_time && (!external_read)){
					if(curr_status != STATUS_COOLING){
						main_time = -1;
						cool_time = preset_cool_time;
						Gv_miliseconds = 0;
						flash_mode = 0;
						state = state_set_time;
						update_status();
					}
				}
				if(main_time && !~external_read){
					main_time = 0;
					Gv_miliseconds = 0;
					update_status();
				}
			}
		}
		ProcessButtons();
		switch (state){
		case state_show_time:
		case state_set_time:
			if(!pre_time && ! main_time && !cool_time ){
				flash_mode = 0;
				state = state_set_time;
				display_data = ToBCD(time_to_set);
				//				display_data = ToBCD(last_button); //Debug
			} else {
				state = state_show_time;
				//				  time_to_set = 0;
				display_data = ToBCD(abs(main_time));
				//				display_data = ToBCD(last_button); //Debug
			}
			break;
		case state_show_hours:

			if( flash_mode != 3){
				flash_mode = 3; // All flashing
			}
			{
				int index = ((flash_counter/0x200)+3) %4;
				if(index<3 ){
					display_data =  ToBCD(work_hours[index]);
				}
				else {
					display_data = 0xFFF;
				}
			}
			break;
		case state_enter_service:
			display_data = service_mode|0xF0;
			flash_mode = 0;
			break;

		case state_clear_hours:
//			flash_mode = 3;
			if((flash_counter/0x400)&1){
				display_data = 0xFFC;
			} else {
				display_data = 0xFFF;
			}
			break;
		case state_address:
			flash_mode = 0;
			if((flash_counter/0x400)&1){
				if(controller_address !=15){ // Address 15 reserved for external control
					display_data = controller_address;
				}
				else {
					display_data = 0xEAF;
				}
			} else {
				display_data = 0xFFA;
			}
			break;
		case state_pre_time:
//			flash_mode = 3;
			if((flash_counter/0x400)&1){
				display_data =  preset_pre_time;
			} else {
				display_data = 0xFF3;
			}
			break;
		case state_cool_time:
//			flash_mode = 3;
			if((flash_counter/0x400)&1){
				display_data =  preset_cool_time;
			} else {
				display_data = 0xFF4;
			}
			break;
		case state_ext_mode:
//			flash_mode = 0;
			if((flash_counter/0x400)&1){
				display_data =  ext_mode;
			} else {
				display_data = 0xFF5;
			}
			break;

		case state_volume:
//			flash_mode = 0;
			if((flash_counter/0x400)&1){
				display_data =  volume_message;
			} else {
				display_data = 0xFF6;
			}
			break;

		case state_max_temp:
//			flash_mode = 0;
			if((flash_counter/0x400)&1){
				display_data =  temperature_threshold;
			} else {
				display_data = 0xFF7;
			}
			break;
		}
		//		show_digit(display_data);
		if (state == state_show_time){
			if(!(pre_time || main_time || cool_time)){
				// Some Paranoia...
				// turn off all
				set_lamps(OFF);
				set_colarium_lamps(OFF);
				percent_fan1 = 0;
				set_fan1(0);
				set_fan2(0);
				set_aquafresh(0);
				lamps_mode = lamps_all;

//				state = state_set_time;
			}
		}
		//
		//		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET); // wAIT UNTIL RX BUFFER IS EMPTY
		//		int data =  USART_ReceiveData(USART1);
		//		USART_SendData(USART1,0x80);
		//				while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY
		if(++led_counter>130L){
			static int current_button_read = 0;
			static int button_buffer[16];
			static int read_counter = 0;
//			if(flash_counter%80 == 0){
//						data = (STATUS_COOLING<<6)|4;
//						USART_SendData(USART1,data);
//			}
			led_counter = 0;
			digit_num++;

			//		aqua_fresh_level = 0;
			if(digit_num>4L) {
				digit_num = 0;
				int i;
				button_buffer[read_counter] = current_button_read;
				if(read_counter++ == 15){
					read_counter = 0;
					last_button = button_buffer[0];
					for (i = 1; i<8; i++){
						last_button = last_button & button_buffer[i];
					}
				}
				current_button_read = 0;
			}
			//display_data = ToBCD(dac_current_block*10 + result);
			GPIOA->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_2 ; // Turn off the lights while changing them
			GPIOB->BSRR = 0;
			GPIOC->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_13 ;
			GPIOF->BSRR = GPIO_BSRR_BR_1 ;
			if(digit_num<3){
				show_digit(((display_data & 0xFFF)& (0x0F<<(digit_num*4)))>>(digit_num*4));
//				show_digit(last_button>>(digit_num*4));
			} else {
				//show_digit(0x8);// DEBUG

			}
			{
				switch (digit_num){
				case 0:

					//set Aqua 2 LED
					if(aqua_fresh_level>0){
						GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12;
						GPIOC->BSRR = GPIO_BSRR_BS_10 ;
					}
					GPIOA->BSRR = GPIO_BSRR_BS_2 ;
					current_button_read |= ((!!(GPIOC->IDR & GPIO_IDR_11)) | ((!!(GPIOC->IDR & GPIO_IDR_12))<<1) | \
																		((!!(GPIOD->IDR & GPIO_IDR_2))<<2))<<0;
					break;
				case 1:

					//set Aqua 2 LEDs
					if(aqua_fresh_level>1){
						GPIOA->BSRR = GPIO_BSRR_BS_8 | GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12;
						GPIOC->BSRR = GPIO_BSRR_BS_10 ;
					}
					GPIOC->BSRR = GPIO_BSRR_BS_0 ;
					current_button_read |= ((!!(GPIOC->IDR & GPIO_IDR_11)) | ((!!(GPIOC->IDR & GPIO_IDR_12))<<1) | \
																	((!!(GPIOD->IDR & GPIO_IDR_2))<<2))<<4;
					break;
				case 2:

					//set Aqua 1 LEDs
					if(aqua_fresh_level>1){
						GPIOA->BSRR = GPIO_BSRR_BS_11 | GPIO_BSRR_BS_12;
					}
					GPIOA->BSRR = GPIO_BSRR_BS_0 ;
					current_button_read |= ((!!(GPIOC->IDR & GPIO_IDR_11)) | ((!!(GPIOC->IDR & GPIO_IDR_12))<<1) | \
																	((!!(GPIOD->IDR & GPIO_IDR_2))<<2))<<8;
					break;
				case 3:

					if (pre_time){
						// Indicate pre_time by moving bars
						show_level((flash_counter>>6) & 0x07);
					} else {
						show_level(volume_level);
					}
					GPIOC->BSRR = GPIO_BSRR_BS_13 ;
					break;
				case 4:

					GPIOF->BSRR = GPIO_BSRR_BS_1 ;
					if (pre_time){
						show_level(((flash_counter>>6) & 0x07) +3);
					} else {
						show_level(fan_level);
					}
					break;
				default:
					break;
				}
			}
			if (((flash_mode == 1)&& digit_num == 0 && (flash_counter & 0x40)) || ((flash_mode == 2) && (digit_num == 2))){
				GPIOA->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BS_5;
			}
			flash_counter++;
		}

		if(dac_current_message>=0){
			if(dac_prev_ping_pong_state != dac_ping_pong_state){
				if (dac_current_block < message_sector_counts[dac_current_message]){
					result = disk_read(0, &dac_buffer[dac_prev_ping_pong_state], message_sector_offset[dac_current_message] + dac_current_block, 2) ;
					if(result == 0){
						dac_current_block++;
						dac_current_block++;
						dac_prev_ping_pong_state = dac_ping_pong_state;
					}
					else{
						disk_initialize(0);
					}
				}
			}
		}


		 /* Reload IWDG counter */
		IWDG_ReloadCounter();
	}
}

void TIM2_IRQHandler() {
	if((TIM2->SR & TIM_SR_UIF) != 0) // If update flag is set
	{
		if(TIM_ICInitStructure.TIM_ICPolarity == TIM_ICPolarity_Rising)
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Falling;
			TIM_ICInit(TIM2, &TIM_ICInitStructure);
		}
		else
		{
			TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
			TIM_ICInit(TIM2, &TIM_ICInitStructure);
		}
	}

	zero_crossed = 1;

	if (colaruim_lamps_state)	GPIOC->BSRR = GPIO_BSRR_BS_8;
	else GPIOC->BRR = GPIO_BRR_BR_8;
	if (lamps_state)	GPIOC->BSRR = GPIO_BSRR_BS_9;
	else GPIOC->BRR = GPIO_BRR_BR_9;
	TIM2->SR = 0; // Interrupt has been handled }
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
					fan_level= 7;
					percent_fan1 = fan_level;
					set_fan1(percent_fan1);
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
//						play_message(0);
					} else {
						if (state > state_enter_service){
							// Write EEPROM
							if(state == state_clear_hours){
								work_hours[0] = 0;
								work_hours[1] = 0;
								work_hours[2] = 0;
							}
							new_write_eeprom();
							new_read_eeprom();
							start_counter = 0;
							service_mode = mode_null;
							if (state == state_address){
								init_periph();
							}
						} else {
							start_counter = START_COUNTER_TIME;
						}
						state = state_show_hours;
						flash_counter = 0;
					}
				}
				break;
			case BUTTON_STOP:
				if(curr_status == STATUS_WORKING){
					main_time = 0;
					pre_time = 0;
					update_status();
					percent_fan1 = 10;
					set_fan1(percent_fan1);
					aqua_fresh_level = 0;
					percent_aquafresh = 0;
				}
				if(state >= state_enter_service){
					start_counter = 0;
					state = state_show_time;
					new_write_eeprom();
					new_read_eeprom();
				}
				break;
			case BUTTON_FAN_PLUS:
				if(curr_status == STATUS_WORKING){
					if(fan_level<10L){
						fan_level++;
						percent_fan1 = fan_level;
						set_fan1(percent_fan1);
					}
				}
				break;
			case BUTTON_FAN_MINUS:
				if(curr_status == STATUS_WORKING){
					if(fan_level>1){
						fan_level--;
						percent_fan1 = fan_level;
						set_fan1(percent_fan1);
					}
				}
				break;
			case BUTTON_AQUA:
#ifdef LICEVI_LAMPI_VMESTO_AQAFRESH
				if(minute_counter)
				{
					aqua_fresh_level = 0;
				}
#else
 				aqua_fresh_level++;
 				if (aqua_fresh_level >2)aqua_fresh_level =0;
#endif
				if(minute_counter){
				}
				break;
			case BUTTON_VOL_PLUS:
				if(curr_status != STATUS_WAITING){
					if(volume_level<10L){
						volume_level++;
						set_volume(volume_level);
					}
				}
				break;
			case BUTTON_VOL_MINUS:
				if(curr_status != STATUS_WAITING){
					if(volume_level>0){
						volume_level--;
						set_volume(volume_level);
					}
				}
				break;
			case BUTTON_PLUS:
			{
				if(curr_status == STATUS_WORKING){
					if(lamps_mode == lamps_all){
						set_colarium_lamps(1);
					}
				}
				if(state == state_show_hours) {
					state = state_set_time;
					start_counter = 0;
				}
				if(state == state_set_time){
					if(time_to_set < 25L){
						if((!useUart && (volume_level == 1)) && (controller_address !=15)) time_to_set++;
					}
					else
					{while(1);}
				}
				else if(state > state_enter_service){
					start_counter = EXIT_SERVICE_TIME;
					switch (service_mode){
					case mode_set_address:
						if(controller_address<15) controller_address++;
						break;
					case mode_set_pre_time:
						if(preset_pre_time<9) preset_pre_time++;
						break;
					case mode_set_cool_time:
						if(preset_cool_time<9) preset_cool_time++;
						break;
					case mode_set_ext_mode:
						if(ext_mode < ext_mode_colarium) ext_mode++;
						break;
					case mode_set_volume:
						if(volume_message < 9) volume_message++;
						break;
					case mode_set_max_temp:
						if(temperature_threshold < 99) temperature_threshold++;
						break;
					default:
						break;
					}
				}

			}
			break;
			case BUTTON_MINUS:
				if(curr_status == STATUS_WORKING){
					if(lamps_mode == lamps_all){
						set_colarium_lamps(0);
					}
				}
				if(state == state_show_hours) {
					state = state_set_time;
					start_counter = 0;
				}
				if(state == state_set_time){
					if(time_to_set) {
						if((!useUart) && (controller_address !=15))  time_to_set--;
					}
				}else if(state > state_enter_service){
					start_counter = EXIT_SERVICE_TIME;
					switch (service_mode){
					case mode_set_address:
						if(controller_address) controller_address--;
						break;
					case mode_set_pre_time:
						if(preset_pre_time) preset_pre_time--;
						break;
					case mode_set_cool_time:
						if(preset_cool_time) preset_cool_time--;
						break;
					case mode_set_ext_mode:
						if(ext_mode > 0) ext_mode--;
						break;
					case mode_set_volume:
						if(volume_message > 0) volume_message--;
						break;
					case mode_set_max_temp:
						if(temperature_threshold > 10) temperature_threshold--;
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
		}
	}
	prev_button = last_button;

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
			else if(start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 4*SERVICE_NEXT_DELAY){
				service_mode = mode_set_ext_mode; //
			}
			else if(start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 5*SERVICE_NEXT_DELAY){
				service_mode = mode_set_volume; //
			}
			else if(start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 6*SERVICE_NEXT_DELAY){
				service_mode = mode_set_max_temp; //
			}
		}
		set_start_out_signal(1);
	}
	else
	{
		set_start_out_signal(0);
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
					new_write_eeprom();
					new_read_eeprom();
				}
			}
			else {
				start_counter = 0;

			}
		}
	}
	if(prev_status != curr_status){
		if (curr_status == STATUS_WORKING){
			if(controller_address != 15){
				work_hours[2]+=main_time;
				if(work_hours[2]>59L){
					work_hours[2]=work_hours[2]-60;
					work_hours[1]++;
					if(work_hours[1]>99L){
						work_hours[1] = 0;
						work_hours[0]++;
					}
				}
				new_write_eeprom();
			}
			flash_mode = 0;
			percent_aquafresh = 0, percent_licevi = 100L, percent_fan2 = 100L;
			zero_crossed = 0;
//			volume_level = 6;
			fan_level = 7;
			percent_fan1 = fan_level;
			if(lamps_mode == lamps_all){
				set_lamps(100);
				set_colarium_lamps(100);
			}
			if(lamps_mode == lamps_uv){
				set_lamps(100);
			}
			if(lamps_mode == lamps_colagen){
				set_colarium_lamps(100);
			}
			zero_crossed = 0;
			set_fan1(percent_fan1);
			set_fan2(percent_fan2);
//			set_aquafresh(percent_aquafresh);
			play_message(message_start_working);
			set_volume(volume_level);
		}
		if (curr_status == STATUS_COOLING){
			if(controller_address == 15){
				work_hours[2]+=abs(main_time);
				if(work_hours[2]>59L){
					work_hours[2]=work_hours[2]-60;
					work_hours[1]++;
					if(work_hours[1]>99L){
						work_hours[1] = 0;
						work_hours[0]++;
					}
				}
				new_write_eeprom();
				play_message(message_stop_working);
			}
			percent_licevi = 0, percent_fan1 = 10L, percent_fan2 = 100L;
			percent_aquafresh = 0;
			aqua_fresh_level = 0;
			zero_crossed = 0;
			set_lamps(OFF);
//			zero_crossed = 0;
			stop_delay = 1000;
//			set_colarium_lamps(0);
			fan_level = 10;
			set_fan1(percent_fan1);
			set_fan2(percent_fan2);
			set_aquafresh(0);
			flash_mode = 3;


		}
		if (curr_status == STATUS_FREE){
			percent_aquafresh = 0, percent_licevi = 0, percent_fan1 = 0, percent_fan2 = 0;
			aqua_fresh_level = 0;
			set_lamps(OFF);
			set_colarium_lamps(OFF);
			set_fan1(0);
			set_fan2(OFF);
//			set_aquafresh(percent_aquafresh);
			minute_counter = 0;
			flash_mode = 0;
			lamps_mode = lamps_all;
		}
		prev_status = curr_status;

	}
	//    LED1_OFF;

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
#ifdef LICEVI_LAMPI_VMESTO_AQAFRESH
		aqua_fresh_level = 0;
#endif
	}
	else if(main_time) {
		curr_time = main_time;
		curr_status = STATUS_WORKING;
#ifdef LICEVI_LAMPI_VMESTO_AQAFRESH
		if(!minute_counter)aqua_fresh_level = 2;
#endif
	}
	else if(cool_time) {
		curr_time = cool_time;
		curr_status = STATUS_COOLING;
#ifdef LICEVI_LAMPI_VMESTO_AQAFRESH
		aqua_fresh_level = 0;
#endif
	}
	else {
		curr_time = 0;
		curr_status = STATUS_FREE;
#ifdef LICEVI_LAMPI_VMESTO_AQAFRESH
		aqua_fresh_level = 0;
#endif
	}
}

// Each 1 mS
void TimingDelay_Decrement(void)
{
	if (Gv_SystickCounter != 0x00)
	{
		Gv_SystickCounter--;
	}
	if( ++ refresh_counter>200L){
		refresh_counter = 0;
		flash_counter++;
	}
#ifndef LICEVI_LAMPI_VMESTO_AQAFRESH
	if(aqua_fresh_level == 1){
		if(Gv_miliseconds>59000L ){
			set_aquafresh(1);
		}
		else {
			set_aquafresh(0);
		}
	}
	else if(aqua_fresh_level == 2){
		if(Gv_miliseconds %15000 >14000L ){
			set_aquafresh(1);
		}
		else {
			set_aquafresh(0);
		}
	}
	else {
		set_aquafresh(0);
	}
#else
	if(aqua_fresh_level > 0){
		set_aquafresh(1);
	}
	else {
		set_aquafresh(0);
	}
#endif

	if(fade_in_counter){
		fade_in_counter--;
//		if(!(fade_in_counter%100)){
//			set_volume(fade_in_counter/100);
//		}
		if(!fade_in_counter){
			GPIOC->BSRR =  GPIO_BSRR_BS_3; // Internal sound
			silence_counter = 100;
		}
	}

	if(fade_out_counter){
		fade_out_counter--;
//		if(!(fade_out_counter%100)){
//			set_volume(10 - fade_out_counter/100);
//		}
		if(fade_out_counter<=0){
			TIM_Cmd(TIM14, DISABLE);
		}
		GPIOC->BRR =  GPIO_BRR_BR_3; // External sound
	}
	if(silence_counter)
	{
		silence_counter--;
		if(!silence_counter)
		{
			fade_out_counter = 100;
		}
	}

	if(startup_delay)
	{
		startup_delay--;
		if(!startup_delay && curr_status == STATUS_WORKING)
		{
			set_colarium_lamps(ON);
			set_fan1(percent_fan1);
			set_fan2(ON);
		}
	}
	if(stop_delay)
	{
		stop_delay--;
		if(!stop_delay && curr_status != STATUS_WORKING)
		{
			set_colarium_lamps(OFF);
		}
	}

	if(Gv_miliseconds++>60000L){
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

//void new_read_eeprom(void)
//{
//	uint16_t counter;
//	uint8_t result = 0;
//	flash_struct *flash_mem = &flash_data;
//	for(counter = 0; counter < sizeof(flash_data); counter++)
//	{
//		result += EE_ReadVariable(VirtAddVarTab[counter],  ((uint16_t*)(&flash_data))[counter]);
//	}
//	if(result)
//	{
//		// Set defaults
//		flash_mem->settings.pre_time = 7;
//		flash_mem->settings.cool_time = 3;
//		flash_mem->time.hours_h = 0;
//		flash_mem->time.hours_l = 0;
//		flash_mem->time.minutes = 0;
//		flash_mem->settings.volume_dac = 5;
//		flash_mem->settings.temperatue_max = 90;
//		flash_mem->settings.ext_mode = 0;
//		flash_mem->settings.addresse = 0x0e;
//		new_write_eeprom();
//	}
//
//	preset_pre_time = flash_mem->settings.pre_time ;
//	preset_cool_time = flash_mem->settings.cool_time;
//	controller_address = flash_mem->settings.addresse & 0x0f;
//	work_hours[0] = flash_mem->time.hours_h;
//	work_hours[1] = flash_mem->time.hours_l;
//	work_hours[2] = flash_mem->time.minutes;
//	volume_message = flash_mem->settings.volume_dac;
//	temperature_threshold = flash_mem->settings.temperatue_max;
//	ext_mode = flash_mem->settings.ext_mode;
//
//}
//
//void new_write_eeprom(void)
//{
//	uint16_t counter;
//	for(counter = 0; counter < sizeof(flash_data); counter++)
//	{
//		if(FLASH_COMPLETE != EE_WriteVariable(VirtAddVarTab[counter],  ((uint16_t*)(&flash_data))[counter]))
//		{
//			// Second chance
//			EE_WriteVariable(VirtAddVarTab[counter],  ((uint16_t*)(&flash_data))[counter]);
//		}
//	}
//}

void new_read_eeprom(void){
	int index = 0;
	flash_struct *flash_mem;
	while((eeprom_array[index]!=0xFFFFFFFFUL)&&(index<(512)))index+=(sizeof(flash_struct)/sizeof(uint32_t));
//	for (i = 0; i< 512; i+=2){
//		val = *pMem;
//		if (val == 0xffffffff) break;
//		pMem++;
//	}
//	index = i;
	if(index > 511){
			index = 0;
	}
	if(index == 0){
		// Load defaults
		flash_mem = 0;
		preset_pre_time = 7;
		preset_cool_time = 3;
		work_hours[0] = 0;
		work_hours[1] = 0;
		work_hours[2] = 0;
		volume_message = 5;
		temperature_threshold = 90;
		ext_mode = 0;
		return;
	}
	index-=sizeof(flash_struct)/sizeof(uint32_t);
	flash_mem = (flash_struct*)&eeprom_array[index];
	preset_pre_time = flash_mem->settings.pre_time ;
	preset_cool_time = flash_mem->settings.cool_time;
	controller_address = flash_mem->settings.addresse & 0x0f;
	work_hours[0] = flash_mem->time.hours_h;
	work_hours[1] = flash_mem->time.hours_l;
	work_hours[2] = flash_mem->time.minutes;
	volume_message = flash_mem->settings.volume_dac;
	temperature_threshold = flash_mem->settings.temperatue_max;
	ext_mode = flash_mem->settings.ext_mode;
}

void new_write_eeprom(void){
	int index = 0;
	FLASH_Unlock();
	volatile flash_struct flash_mem;
	uint32_t *p = (uint32_t *)&flash_mem;
	while((eeprom_array[index]!=0xFFFFFFFFUL)&&(index<(512)))index+=3;
	if(index > 511){
		// No more room. Erase the 4 pages
		FLASH_ErasePage((uint32_t)&eeprom_array[0]);
		FLASH_ErasePage((uint32_t)&eeprom_array[128]);
		FLASH_ErasePage((uint32_t)&eeprom_array[256]);
		FLASH_ErasePage((uint32_t)&eeprom_array[384]);
		index = 0;
	}
	while((eeprom_array[index]!=0xFFFFFFFFUL)&&(index<(512)))index+=2;
	if(index >511){
		display_data = 0xE01;
	} else {
		volatile static FLASH_Status sts;
		flash_mem.settings.pre_time = preset_pre_time;
		flash_mem.settings.cool_time = preset_cool_time;
		flash_mem.settings.addresse = controller_address & 0x0f;
		flash_mem.time.hours_h = work_hours[0];
		flash_mem.time.hours_l = work_hours[1];
		flash_mem.time.minutes = work_hours[2];
		flash_mem.time.used_flag = 0;
		flash_mem.settings.volume_dac = volume_message;
		flash_mem.settings.temperatue_max = temperature_threshold;
		flash_mem.settings.ext_mode = ext_mode;
		sts = FLASH_ProgramWord((uint32_t)&eeprom_array[index],p[0]);
		if(sts != FLASH_COMPLETE)
		{
			// second chance
			FLASH_ProgramWord((uint32_t)&eeprom_array[index],p[0]);
		}
		sts = FLASH_ProgramWord((uint32_t)&eeprom_array[index+1],p[1]);
		if(sts != FLASH_COMPLETE)
		{
			// second chance
			FLASH_ProgramWord((uint32_t)&eeprom_array[index+1],p[1]);
		}
		index = sizeof(flash_mem);
		index ++;
		if (sts){
			// Debug code here...
		}
	}
	FLASH_Lock();
}
void set_start_out_signal(int value){
	if (value)	GPIOA->BSRR = GPIO_BSRR_BS_9;
		else GPIOA->BRR = GPIO_BSRR_BS_9;
}

void set_lamps(int value){
	lamps_state = value;
	if(value)
		GPIOC->BRR =  GPIO_BSRR_BS_3; // External sound
	else
		GPIOC->BSRR =  GPIO_BSRR_BS_3; // Internal sound)
}
void set_colarium_lamps(int value){
	colaruim_lamps_state = value;
}
void set_fan2(int value){
	if (value)
		GPIOC->BSRR = GPIO_BSRR_BS_14;
	else
		GPIOC->BSRR = GPIO_BSRR_BR_14;
}

void set_fan1(int value){
	//
//	uint32 counter = 0xFFFFFFF;
	TIM_Cmd(TIM2, DISABLE);

	zero_crossed = 0;

//	while (!zero_crossed && (counter--));
	switch(value){
	case 10:
		tim_base = 60; //Reverse polarity
		break;
	case 9:
		tim_base = 37;
		break;
	case 8:
		tim_base = 43;
		break;
	case 7:
		tim_base = 48;
		break;
	case 6:
		tim_base = 52;
		break;
	case 5:
		tim_base = 56;
		break;
	case 4:
		tim_base = 59;
		break;
	case 3:
		tim_base = 62;
		break;
	case 2:
		tim_base = 63;
		break;
	case 1:
	default:
		tim_base = 65;
		break;
	}
	if(value == 10) TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	else  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_TimeBaseStructure.TIM_Period = tim_base;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* TIM2 PWM2 Mode configuration: Channel4 */
	//for one pulse mode set PWM2, output enable, pulse (1/(t_wanted=TIM_period-TIM_Pulse)), set polarity high
	if (value)	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	else 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	if (TIM_TimeBaseStructure.TIM_Period) TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period-5;
	else  TIM_OCInitStructure.TIM_Pulse = 9;

	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
//	if (value) TIM_Cmd(TIM2, ENABLE);
//	else TIM_Cmd(TIM2, DISABLE);
	TIM_Cmd(TIM2, ENABLE);
}

void set_aquafresh(int value){

	if (value)	GPIOB->BSRR = GPIO_BSRR_BS_5;
	else GPIOB->BRR = GPIO_BSRR_BS_5;
}

void set_volume(int value){
#ifdef PCBVERSION_2
	GPIOC->BRR = GPIO_BSRR_BS_2 | GPIO_BSRR_BS_3;
	if (value >8)	{
		GPIOC->BSRR = GPIO_BSRR_BS_2 | GPIO_BSRR_BS_3;
	}
	else if (value >5)	{
		GPIOC->BSRR =  GPIO_BSRR_BS_3;
	}
	else if (value >2)	{
		GPIOC->BSRR = GPIO_BSRR_BS_2 ;
	}

#else  //PCBVERSION_3
	GPIOB->BRR = GPIO_BRR_BR_6;
	GPIOF->BRR = GPIO_BRR_BR_6 | GPIO_BRR_BR_7;

	if (value >9)	{
		//GPIOB->BSRR = GPIO_BSRR_BS_6;
		//GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		//GPIOF->BSRR = GPIO_BSRR_BS_7;
	}
	else if (value >7)	{
		GPIOB->BSRR = GPIO_BSRR_BS_6;
		//GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		//GPIOF->BSRR = GPIO_BSRR_BS_7;
	}
	else if (value >6)	{
		//GPIOB->BSRR = GPIO_BSRR_BS_6;
		//GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		GPIOF->BSRR = GPIO_BSRR_BS_7;
	}
	else if (value >5)	{
		GPIOB->BSRR = GPIO_BSRR_BS_6;
		//GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		GPIOF->BSRR = GPIO_BSRR_BS_7;
		}
	else if (value >4)	{
		//GPIOB->BSRR = GPIO_BSRR_BS_6;
		GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		//GPIOF->BSRR = GPIO_BSRR_BS_7;
		}
	else if (value >3)	{
		GPIOB->BSRR = GPIO_BSRR_BS_6;
		GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		//GPIOF->BSRR = GPIO_BSRR_BS_7;
		}
	else if (value >1)	{
		//GPIOB->BSRR = GPIO_BSRR_BS_6;
		GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		GPIOF->BSRR = GPIO_BSRR_BS_7;
		}
	else {
		GPIOB->BSRR = GPIO_BSRR_BS_6;
		GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		GPIOF->BSRR = GPIO_BSRR_BS_7;
	}
#endif
}


void usart_receive(void){
	useUart = 1;
	enum rxstates {rx_state_none, rx_state_pre_time, rx_state_main_time, rx_state_cool_time, rx_state_get_checksum};
	//	USART_ITConfig(USART1,USART_IT_RXNE,DISABLE);
	data =  USART_ReceiveData(USART1);

	//pre_time_sent = 0, main_time_sent = 0, cool_time_sent = 0;

	if ((data & 0x80)){
		last_rx_address = (data >> 3U)&0x0f;
		unsigned char addr_is_ok = 0;

		if(ext_mode == ext_mode_colarium){
			if(last_rx_address  == controller_address ){
				if (curr_status != STATUS_FREE ){
					if( lamps_mode == lamps_all) addr_is_ok = 1;
				}
				else {
					addr_is_ok = 1;
				}
			}
			if(last_rx_address  == controller_address + 1){
				if (curr_status != STATUS_FREE ){
					if( lamps_mode == lamps_uv) addr_is_ok = 1;
				}
				else {
					addr_is_ok = 1;
				}
			}
			if(last_rx_address  == controller_address + 2){
				if (curr_status != STATUS_FREE ){
					if( lamps_mode == lamps_colagen) addr_is_ok = 1;
				}
				else {
					addr_is_ok = 1;
				}
			}
		}
		else {
			addr_is_ok = (last_rx_address == controller_address);
		}
		if (!addr_is_ok) return;
		// Command
		if(addr_is_ok){
			Gv_UART_Timeout = 1500;
		}
		if((data & 0x07) == 0x00 ){ // Status
			data = (curr_status<<6)| ToBCD(curr_time);
//			data = (STATUS_WORKING<<6)|4;
			USART_SendData(USART1,data);
		}
		else if ((data & 0x07) == 1) //Command 1 - Start
		{
			pre_time = 0;
			update_status();
		}
		else if ((data & 0x07) == 2)  //Command 2 == Pre_time_set
		{
			rx_state = rx_state_pre_time;
		}
		else if ((data & 0x07) == 5) //Command 5 == Main_time_set
		{
			rx_state = rx_state_main_time;
		}
		else if ((data & 0x07) == 3) //Command 3 == Cool_time_set
		{
			rx_state = rx_state_cool_time;
		}

	} else if (rx_state){
		// payload
		int time_in_hex = ToBCD(main_time_sent);
		if(rx_state == rx_state_get_checksum){
			int checksum = (pre_time_sent + cool_time_sent  - time_in_hex - 5) & 0x7F;
			if(	data == checksum){
				pre_time = pre_time_sent;
				main_time = main_time_sent;
				cool_time = cool_time_sent;
				update_status();
				Gv_miliseconds = 0;
			}
			rx_state = 0;
		}
		if(rx_state == rx_state_pre_time){
			pre_time_sent = data;
			rx_state = 0;
		}
		if(rx_state == rx_state_main_time){
			main_time_sent = FromBCD(data);
			rx_state = 0;
			if (last_rx_address == controller_address)	lamps_mode = lamps_all;
			if (last_rx_address == controller_address+1)lamps_mode = lamps_uv;
			if (last_rx_address == controller_address+2)lamps_mode = lamps_colagen;

		}
		if(rx_state == rx_state_cool_time){
			cool_time_sent = data;
			rx_state = rx_state_get_checksum;
			int checksum = (pre_time_sent + cool_time_sent  - time_in_hex - 5) & 0x7F;
			data = checksum;
			USART_SendData(USART1,data);
		}


	}
	//	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
	//	USART_SendData(USART1,0x80);
}

void play_message(int index){

	disk_initialize(0);
	if(disk_status(0) == STA_NOINIT)
	{
		if(index == message_start_working)
		{
			fade_in_counter =  100;
			//TIM_Cmd(TIM14, ENABLE);
		}
		return;
	}
	dac_ping_pong_state = 0;
	dac_current_message = index;
	DRESULT result;
	dac_out_counter = 0;

	result = disk_read(0, &dac_buffer[dac_ping_pong_state], message_sector_offset[index], 2) ;
	dac_current_block = 0;

	dac_ping_pong_state = 0;
	dac_prev_ping_pong_state = dac_ping_pong_state;
	fade_in_counter =  100;
	if (result) dac_out_counter = dac_out_counter;
	//DAC_Cmd(DAC_Channel_1, ENABLE);
	//TIM6->DIER |= TIM_DIER_UIE; // Enable interrupt on update event
	//TIM_Cmd(TIM6, ENABLE);
	TIM_Cmd(TIM14, ENABLE);
	//while(1)
	{
		if (dac_current_block < message_sector_counts[dac_current_message]){
			if(dac_prev_ping_pong_state != dac_ping_pong_state)
			{
				dac_prev_ping_pong_state = dac_ping_pong_state;
				//TIM14->CCER &= ~TIM_OutputState_Enable;
				disk_read(0, &dac_buffer[dac_prev_ping_pong_state], message_sector_offset[dac_current_message] + dac_current_block, 2) ;
				dac_current_block++;
				dac_current_block++;
				//TIM14->CCER |= TIM_OutputState_Enable;
			}
		}
		//dac_current_block = 0;
	}

//	TIM6->CR1 |= TIM_CR1_CEN;

}


void TIM14_IRQHandler() {
	if(!(TIM14->SR & TIM_SR_UIF) != 0) return;
	// If update flag is set

		static int dummy = 0;
		if (dummy++<10) goto finish_TIM14_isr;
		dummy = 0;//

//		if(fade_in_counter){
//			fade_in_counter--;
//			if(!(fade_in_counter%1000)){
//				set_volume(fade_in_counter/1000);
//			}
//			if(!fade_in_counter){
//				GPIOC->BSRR =  GPIO_BSRR_BS_3; // Internal sound
//				silence_counter = 100000;
//			}
//			goto finish_TIM14_isr;
//		}
//
//		if(fade_out_counter){
//			fade_out_counter--;
//			if(!(fade_out_counter%1000)){
//				set_volume(10 - fade_out_counter/1000);
//			}
//			if(fade_out_counter<=0){
//				TIM_Cmd(TIM14, DISABLE);
//			}
//			GPIOC->BRR =  GPIO_BRR_BR_3; // External sound
//			goto finish_TIM14_isr;
//		}
		unsigned char vol = 10; //10-volume_message%10;
		if(disk_status(0) == STA_NOINIT)
		{
//			if(silence_counter)
//			{
//				silence_counter--;
//				if(!silence_counter)
//				{
//					fade_out_counter = 10000;
//				}
//			}

		}
		else
		{
			TIM_SetCompare1(TIM14, dac_buffer[dac_ping_pong_state][dac_out_counter]/vol);

			if(dac_out_counter<1024){
				dac_out_counter++;
			} else {
				if (dac_current_block >= message_sector_counts[dac_current_message]){
						fade_out_counter = 100;
						dac_current_message = -1;
				}
				dac_out_counter = 0;
				dac_ping_pong_state = !dac_ping_pong_state;
			}
		}

	finish_TIM14_isr:
	TIM14->SR &= ~TIM_SR_UIF; // Interrupt has been handled }
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
