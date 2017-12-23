/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2017 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "max7219.h"
#include "LedControl.h"


#define STATUS_FREE    (0L)
#define STATUS_WAITING (3L)
#define STATUS_WORKING (1L)
#define STATUS_COOLING (2L)
#define START_COUNTER_TIME  (1000000L)
#define ENTER_SERVICE_DELAY (2000000L)
#define SERVICE_NEXT_DELAY  (400000L)
#define EXIT_SERVICE_TIME   (1200000L)
#define START_DELAY         (200000L)


// Buttons
#define BUTTON_PLUS      (0x004)
#define BUTTON_MINUS     (0x010)
#define BUTTON_START     (0x001)
#define BUTTON_STOP      (0x002)

#define BUTTON_AQUA      (0x400)
#define BUTTON_FAN_PLUS  (0x100)
#define BUTTON_FAN_MINUS (0x200)
#define BUTTON_VOL_PLUS  (0x020)
#define BUTTON_VOL_MINUS (0x040)

#define ON 100
#define OFF 0
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int key_states;
LedControl led_control;
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


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
									
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
								
								

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
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

char key_readings[9];
char col_index;
char row_index;
void scan_keys()
{
	row_index++;
	if(row_index > 2)
	{
		row_index = 0;
		col_index++;
		if(col_index > 2)
		{
			col_index = 0;
		}
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

	if(row_index == 0)
	{
		key_readings[row_index + col_index*3] |= (key_readings[row_index + col_index*3] << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
	}
	if(row_index == 1)
	{
		key_readings[row_index + col_index*3] |= (key_readings[row_index + col_index*3] << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
	}
	if(row_index == 2)
	{
		key_readings[row_index + col_index*3] |= (key_readings[row_index + col_index*3] << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
	}


	if(key_readings[row_index + col_index*3] == 0xFF)
	{
		key_states |= 1 << (row_index + col_index*3);
	}
	if(key_readings[row_index + col_index*3] == 0x00)
	{
		key_states &= ~(1 << (row_index + col_index*3));
	}
}

char bar_conversion_table[] = {0x00, 0x00, 0x00, 0x80, 0x81, 0x83, 0x87, 0x8F, 0x9F, 0xBF, 0xFF};

void ShowBarIndicators(unsigned char bar1, unsigned char bar2)
{
	  unsigned char val = 0;
	  if(bar1 > 10) bar1 = 10;
	  if (bar2 > 10) bar2 = 10;
//	  LedControl_setRow(&led_control, 0, 3, (1<<(bar1-3)) - 1);
	  LedControl_setRow(&led_control, 0, 3, bar_conversion_table[bar1]);
	  LedControl_setRow(&led_control, 0, 5, bar_conversion_table[bar2]);
	  if(bar2 > 0) val |= 0x08;
	  if(bar2 > 1) val |= 0x10;
	  if(bar1 > 0) val |= 0x20;
	  if(bar1 > 1) val |= 0x40;
	  LedControl_setRow(&led_control, 0, 4, val);
}

char display_buffer[16];


void show_level(int level){
	level = level & 0x0F;
}
void show_digit(int digit){
	digit = digit & 0x0F;
}

/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */
	char index = 0;

	//  char i;
	//  for (i = 0; i < 8; i++)
	//  {
	//	  display_buffer[i*2] = 0x0;
	//	  display_buffer[i*2+1] = 0;
	//  }
	//  display_buffer[0] = 0;
	//  display_buffer[1] = 9; // No decode mode
	//  display_buffer[2] = 0x7F;
	//  display_buffer[3] = 0x0A; // Intensity
	//  display_buffer[4] = 0x7;
	//  display_buffer[5] = 0x0B; // Scan Limit
	//  display_buffer[6] = 0x1;
	//  display_buffer[7] = 0x0C; // Shutdown register
	//  Display_refresh();
	//  HAL_Delay(100);
	//
	//  for (i = 0; i < 8; i++)
	//  {
	//	  display_buffer[i*2] = 0x33;
	//	  display_buffer[i*2+1] = i;
	//  }
	//  MAX7219_Init();


	//  SPI_Write(0x0C,0x01);        // Normal Operation
	//  SPI_Write(0x09,0xFF);        // Code B Decode for Digit 7 to 0
	//  SPI_Write(0x0B,0x07);        // Scan digit 7 to 0
	//  SPI_Write(0x0A,0x0F);        // Set Default Intensity to Max



	  /* USER CODE END 1 */

	  /* MCU Configuration----------------------------------------------------------*/

	  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */
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
	if (!preset_pre_time || !preset_cool_time) {
		preset_pre_time = 7;
		preset_cool_time = 3;
		new_write_eeprom();// Paranoia check
	}
	update_status();
	//set_volume(0);

	//play_message(message_power_up);

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_ADC1_Init();
	MX_USART1_UART_Init();
	MX_USART3_UART_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();

	/* USER CODE BEGIN 2 */
	LedControl_init(&led_control, 15, GPIOB, 13, GPIOB, 12, GPIOB, 1);
	LedControl_shutdown(&led_control, 0, 0); //Turn ON
	LedControl_setIntensity(&led_control, 0, 8);
	LedControl_clearDisplay(&led_control, 0);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	  //	  HAL_Delay(1000);
	  //	  scan_keys();
	  //	  LedControl_setDigit(&led_control, 0, 0, index & 0x0f, 0);
	  //	  LedControl_setDigit(&led_control, 0, 1, index & 0x0f, 0);
	  //	  LedControl_setDigit(&led_control, 0, 2, index & 0x0f, 0);
	  //	  if((index & 0x0f) < 11)
	  //	  {
	  //	      ShowBarIndicators(index & 0x0f, 0);
	  //	  }
	  //	  LedControl_setRow(&led_control, 0, 3, 1<<(index & 7));
	  //	  LedControl_setRow(&led_control, 0, 5, 1<<(index & 7));
	  //	  LedControl_setRow(&led_control, 0, 6, 1<<(index & 7));
	  //	  MAX7219_DisplayChar(1,index & 0x0f);
	  //	  MAX7219_DisplayChar(2,(index>>4) & 0x0f);
	  //	  index++;
	  //	  HAL_Delay(1000);
	  //	  MAX7219_ShutdownStart();
	  //	  HAL_Delay(1000);
	  //	  MAX7219_ShutdownStop();
	  //	  Display_refresh();
		if ((state < state_enter_service) && ((flash_counter >> 4) & 1))
		{
			if (controller_address == 15) {
				external_read = (external_read << 1) | (!!(GPIOB->IDR & GPIO_IDR_IDR7));
				if (!main_time && (!external_read)) {
					if (curr_status != STATUS_COOLING) {
						main_time = -1;
						cool_time = preset_cool_time;
						Gv_miliseconds = 0;
						flash_mode = 0;
						state = state_set_time;
						update_status();
					}
				}
				if (main_time && !~external_read) {
					main_time = 0;
					Gv_miliseconds = 0;
					update_status();
				}
			}
		}
		ProcessButtons();
		switch (state) {
		case state_show_time:
		case state_set_time:
			if (!pre_time && !main_time && !cool_time) {
				flash_mode = 0;
				state = state_set_time;
				display_data = ToBCD(time_to_set);
				//				display_data = ToBCD(last_button); //Debug
			}
			else {
				state = state_show_time;
				//				  time_to_set = 0;
				display_data = ToBCD(abs(main_time));
				//				display_data = ToBCD(last_button); //Debug
			}
			break;
		case state_show_hours:

			if (flash_mode != 3) {
				flash_mode = 3; // All flashing
			}
			{
				int index = ((flash_counter / 0x200) + 3) % 4;
				if (index < 3) {
					display_data = ToBCD(work_hours[index]);
				}
				else {
					display_data = 0xFFF;
				}
			}
			break;
		case state_enter_service:
			display_data = service_mode | 0xF0;
			flash_mode = 0;
			break;

		case state_clear_hours:
			//			flash_mode = 3;
			if ((flash_counter / 0x400) & 1) {
				display_data = 0xFFC;
			}
			else {
				display_data = 0xFFF;
			}
			break;
		case state_address:
			flash_mode = 0;
			if ((flash_counter / 0x400) & 1) {
				if (controller_address != 15) { // Address 15 reserved for external control
					display_data = controller_address;
				}
				else {
					display_data = 0xEAF;
				}
			}
			else {
				display_data = 0xFFA;
			}
			break;
		case state_pre_time:
			//			flash_mode = 3;
			if ((flash_counter / 0x400) & 1) {
				display_data = preset_pre_time;
			}
			else {
				display_data = 0xFF3;
			}
			break;
		case state_cool_time:
			//			flash_mode = 3;
			if ((flash_counter / 0x400) & 1) {
				display_data = preset_cool_time;
			}
			else {
				display_data = 0xFF4;
			}
			break;
		case state_ext_mode:
			//			flash_mode = 0;
			if ((flash_counter / 0x400) & 1) {
				display_data = ext_mode;
			}
			else {
				display_data = 0xFF5;
			}
			break;

		case state_volume:
			//			flash_mode = 0;
			if ((flash_counter / 0x400) & 1) {
				display_data = volume_message;
			}
			else {
				display_data = 0xFF6;
			}
			break;

		case state_max_temp:
			//			flash_mode = 0;
			if ((flash_counter / 0x400) & 1) {
				display_data = temperature_threshold;
			}
			else {
				display_data = 0xFF7;
			}
			break;
		}
		//		show_digit(display_data);
		if (state == state_show_time) {
			if (!(pre_time || main_time || cool_time)) {
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
		if (pre_time) {
			// Indicate pre_time by moving bars
			ShowBarIndicators((flash_counter >> 2) & 0x07, (flash_counter >> 2) & 0x07);
		}
		else {
			ShowBarIndicators(volume_level, fan_level);
		}
		HAL_Delay(100);

		LedControl_setDigit(&led_control, 0, 0, display_data & 0x0f, 0);
		LedControl_setDigit(&led_control, 0, 1, (display_data>>4) & 0x0f, 0);
		LedControl_setDigit(&led_control, 0, 2, (display_data>>8) & 0x0f, 0);
		flash_counter++;

		/* Reload IWDG counter */
		IWDG_ReloadCounter();

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

	/**Initializes the CPU, AHB and APB busses clocks 
	*/
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
							  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

	/**Configure the Systick interrupt time 
	*/
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick 
	*/
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

	/**Common config 
	*/
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

	/**Configure Regular Channel 
	*/
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OnePulse_Init(&htim1, TIM_OPMODE_SINGLE) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 5000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 2500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* USART1 init function */
static void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

}

/* USART3 init function */
static void MX_USART3_UART_Init(void)
{

  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
	_Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
		* Analog 
		* Input 
		* Output
		* EVENT_OUT
		* EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
						  |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 PA10 
						   PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
						  |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB14 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/**
 * @brief  Manage the activity on buttons
 * @param  None
 * @retval None
 */
void ProcessButtons(void)
{

	if (last_button) {
		if (last_button != prev_button) {
			switch (last_button) {
			case BUTTON_START:
				if (pre_time) {
					//send_start();
					Gv_miliseconds = 0;
					pre_time = 0;
					state = state_show_time;
					update_status();
					fan_level = 7;
					percent_fan1 = fan_level;
					set_fan1(percent_fan1);
				}
				if (!pre_time && !main_time && !cool_time) {
					if (time_to_set) {
						// Send of time moved elsewhere
						pre_time = preset_pre_time;
						main_time = time_to_set;
						cool_time = preset_cool_time;
						state = state_show_time;
						time_to_set = 0;
						Gv_miliseconds = 0;
						update_status();
						//						play_message(0);
					}
					else {
						if (state > state_enter_service) {
							// Write EEPROM
							if (state == state_clear_hours) {
								work_hours[0] = 0;
								work_hours[1] = 0;
								work_hours[2] = 0;
							}
							new_write_eeprom();
							new_read_eeprom();
							start_counter = 0;
							service_mode = mode_null;
							if (state == state_address) {
								init_periph();
							}
						}
						else {
							start_counter = START_COUNTER_TIME;
						}
						state = state_show_hours;
						flash_counter = 0;
					}
				}
				break;
			case BUTTON_STOP:
				if (curr_status == STATUS_WORKING) {
					main_time = 0;
					pre_time = 0;
					update_status();
					percent_fan1 = 10;
					set_fan1(percent_fan1);
					aqua_fresh_level = 0;
					percent_aquafresh = 0;
				}
				if (state >= state_enter_service) {
					start_counter = 0;
					state = state_show_time;
					new_write_eeprom();
					new_read_eeprom();
				}
				break;
			case BUTTON_FAN_PLUS:
				if (curr_status == STATUS_WORKING) {
					if (fan_level < 10L) {
						fan_level++;
						percent_fan1 = fan_level;
						set_fan1(percent_fan1);
					}
				}
				break;
			case BUTTON_FAN_MINUS:
				if (curr_status == STATUS_WORKING) {
					if (fan_level > 1) {
						fan_level--;
						percent_fan1 = fan_level;
						set_fan1(percent_fan1);
					}
				}
				break;
			case BUTTON_AQUA:
#ifdef LICEVI_LAMPI_VMESTO_AQAFRESH
				if (minute_counter)
				{
					aqua_fresh_level = 0;
				}
#else
				aqua_fresh_level++;
				if (aqua_fresh_level > 2)aqua_fresh_level = 0;
#endif
				if (minute_counter) {
				}
				break;
			case BUTTON_VOL_PLUS:
				if (curr_status != STATUS_WAITING) {
					if (volume_level < 10L) {
						volume_level++;
						set_volume(volume_level);
					}
				}
				break;
			case BUTTON_VOL_MINUS:
				if (curr_status != STATUS_WAITING) {
					if (volume_level > 0) {
						volume_level--;
						set_volume(volume_level);
					}
				}
				break;
			case BUTTON_PLUS:
			{
				if (curr_status == STATUS_WORKING) {
					if (lamps_mode == lamps_all) {
						set_colarium_lamps(1);
					}
				}
				if (state == state_show_hours) {
					state = state_set_time;
					start_counter = 0;
				}
				if (state == state_set_time) {
					if (time_to_set < 25L) {
						if ((!useUart && (volume_level == 1)) && (controller_address != 15)) time_to_set++;
					}
					else
					{
						while (1);
					}
				}
				else if (state > state_enter_service) {
					start_counter = EXIT_SERVICE_TIME;
					switch (service_mode) {
					case mode_set_address:
						if (controller_address < 15) controller_address++;
						break;
					case mode_set_pre_time:
						if (preset_pre_time < 9) preset_pre_time++;
						break;
					case mode_set_cool_time:
						if (preset_cool_time < 9) preset_cool_time++;
						break;
					case mode_set_ext_mode:
						if (ext_mode < ext_mode_colarium) ext_mode++;
						break;
					case mode_set_volume:
						if (volume_message < 9) volume_message++;
						break;
					case mode_set_max_temp:
						if (temperature_threshold < 99) temperature_threshold++;
						break;
					default:
						break;
					}
				}

			}
			break;
			case BUTTON_MINUS:
				if (curr_status == STATUS_WORKING) {
					if (lamps_mode == lamps_all) {
						set_colarium_lamps(0);
					}
				}
				if (state == state_show_hours) {
					state = state_set_time;
					start_counter = 0;
				}
				if (state == state_set_time) {
					if (time_to_set) {
						if ((!useUart) && (controller_address != 15))  time_to_set--;
					}
				}
				else if (state > state_enter_service) {
					start_counter = EXIT_SERVICE_TIME;
					switch (service_mode) {
					case mode_set_address:
						if (controller_address) controller_address--;
						break;
					case mode_set_pre_time:
						if (preset_pre_time) preset_pre_time--;
						break;
					case mode_set_cool_time:
						if (preset_cool_time) preset_cool_time--;
						break;
					case mode_set_ext_mode:
						if (ext_mode > 0) ext_mode--;
						break;
					case mode_set_volume:
						if (volume_message > 0) volume_message--;
						break;
					case mode_set_max_temp:
						if (temperature_threshold > 10) temperature_threshold--;
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
				while (0);
			}
		}
	}
	prev_button = last_button;

	if (last_button == BUTTON_START)
	{
		// LED1_ON;
		if (start_counter < (START_COUNTER_TIME + ENTER_SERVICE_DELAY + 6 * SERVICE_NEXT_DELAY)) start_counter++;
		if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY) {
			if (curr_status == STATUS_FREE && (state < state_enter_service))
			{
				state = state_enter_service;
				service_mode = mode_clear_hours; // Clear Hours
			}
		}
		if (state == state_enter_service) {
			if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 1 * SERVICE_NEXT_DELAY) {
				service_mode = mode_set_address; //
			}
			else if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 2 * SERVICE_NEXT_DELAY) {
				service_mode = mode_set_pre_time; //
			}
			else if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 3 * SERVICE_NEXT_DELAY) {
				service_mode = mode_set_cool_time; //
			}
			else if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 4 * SERVICE_NEXT_DELAY) {
				service_mode = mode_set_ext_mode; //
			}
			else if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 5 * SERVICE_NEXT_DELAY) {
				service_mode = mode_set_volume; //
			}
			else if (start_counter == START_COUNTER_TIME + ENTER_SERVICE_DELAY + 6 * SERVICE_NEXT_DELAY) {
				service_mode = mode_set_max_temp; //
			}
		}
		set_start_out_signal(1);
	}
	else
	{
		set_start_out_signal(0);
		if (start_counter) {
			if (state == state_show_hours) {
				start_counter--;
				if (!start_counter) {
					state = state_show_time;
				}
			}
			else if (state >= state_enter_service) {
				if (state == state_enter_service) {
					state = service_mode + state_enter_service;
				}
				start_counter--;
				if (!start_counter) {
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
	if (prev_status != curr_status) {
		if (curr_status == STATUS_WORKING) {
			if (controller_address != 15) {
				work_hours[2] += main_time;
				if (work_hours[2] > 59L) {
					work_hours[2] = work_hours[2] - 60;
					work_hours[1]++;
					if (work_hours[1] > 99L) {
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
			if (lamps_mode == lamps_all) {
				set_lamps(100);
				set_colarium_lamps(100);
			}
			if (lamps_mode == lamps_uv) {
				set_lamps(100);
			}
			if (lamps_mode == lamps_colagen) {
				set_colarium_lamps(100);
			}
			zero_crossed = 0;
			set_fan1(percent_fan1);
			set_fan2(percent_fan2);
			//			set_aquafresh(percent_aquafresh);
			play_message(message_start_working);
			set_volume(volume_level);
		}
		if (curr_status == STATUS_COOLING) {
			if (controller_address == 15) {
				work_hours[2] += abs(main_time);
				if (work_hours[2] > 59L) {
					work_hours[2] = work_hours[2] - 60;
					work_hours[1]++;
					if (work_hours[1] > 99L) {
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
		if (curr_status == STATUS_FREE) {
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

void update_status(void) {
	if (pre_time) {
		curr_time = pre_time;
		curr_status = STATUS_WAITING;
#ifdef LICEVI_LAMPI_VMESTO_AQAFRESH
		aqua_fresh_level = 0;
#endif
	}
	else if (main_time) {
		curr_time = main_time;
		curr_status = STATUS_WORKING;
#ifdef LICEVI_LAMPI_VMESTO_AQAFRESH
		if (!minute_counter)aqua_fresh_level = 2;
#endif
	}
	else if (cool_time) {
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


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
	ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
