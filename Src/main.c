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
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "math.h"
#include "eeprom.h"
#include "defines.h"

//#define LAMPI_KRAKA
#define LICEVI_LAMPI_VMESTO_AQAFRESH
#define STATUS_FREE    (0L)
#define STATUS_WAITING (3L)
#define STATUS_WORKING (1L)
#define STATUS_COOLING (2L)
#define MULTIPLIER     (1)
#define START_COUNTER_TIME  (1000L*MULTIPLIER)
#define ENTER_SERVICE_DELAY (2000L*MULTIPLIER)
#define SERVICE_NEXT_DELAY  (400*MULTIPLIER)
#define EXIT_SERVICE_TIME   (1200L*MULTIPLIER)
#define START_DELAY         (200L*MULTIPLIER)

// Virtual addreses
#define ADDRESS_HOURS_H		1
#define ADDRESS_HOURS_L		2
#define ADDRESS_MINUTES		3
#define ADDRESS_ADDRESSE	4
#define ADDRESS_PRE_TIME	5
#define ADDRESS_COOL_TIME	6
#define ADDRESS_EXT_MODE	7
#define ADDRESS_VOLUME		8
#define ADDRESS_TEMP_MAX	9

// Buttons
#define BUTTON_PLUS      (1<<0x02)
#define BUTTON_MINUS     (1<<0x03)
#define BUTTON_START     (1<<0x00)
#define BUTTON_STOP      (1<<0x01)

#define BUTTON_AQUA      (1<<0x08)
#define BUTTON_FAN_PLUS  (1<<0x06)
#define BUTTON_FAN_MINUS (1<<0x07)
#define BUTTON_VOL_PLUS  (1<<0x04)
#define BUTTON_VOL_MINUS (1<<0x05)

#define ON 100
#define OFF 0
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IWDG_HandleTypeDef hiwdg;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
const uint32_t eeprom_array[512];
int key_states;
LedControl led_control;
__IO uint32_t TimingDelay;
int tim_reload_value = 31000;
int Gv_SystickCounter;
uint32_t g_ADCValue;
uint32_t g_ADCMeanValue;
uint32_t g_MeasurementNumber;
uint32_t g_Temperature;

uint32_t test_data;

unsigned char controller_address = 0x0e;
int curr_status;
int prev_status;
int curr_time;
//int flash_mode = 0;
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
static int auto_exit_fn = 0;

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
int percent_licevi = 0, percent_fan1 = 0, percent_fan2 = 0;
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

/* Application general parameters */
#define VDD_APPLI                      ((uint32_t) 3300)    /* Value of analog voltage supply Vdda (unit: mV) */
#define RANGE_8BITS                    ((uint32_t)  255)    /* Max digital value for a full range of 8 bits */
#define RANGE_12BITS                   ((uint32_t) 4095)    /* Max digital value for a full range of 12 bits */

/* ADC parameters */
#define ADCCONVERTEDVALUES_BUFFER_SIZE ((uint32_t)  256)    /* Size of array containing ADC converted values */
#define ADC_36_6_DEGREE_VALUE (1024 - 0x617)
#define ADC_0_DEGREE_VALUE (1024 - 0xB70)

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_IWDG_Init(void);
static void MX_NVIC_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void read_settings(void);
void write_settings(void);
void read_stored_time(void);
void write_stored_time(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
int ToBCD(int value);
//void write_eeprom(void);
//void read_eeprom(void);
void ProcessButtons(void);
void update_status(void);
void set_start_out_signal(int value);
void set_lamps(int value);
void set_colarium_lamps(int value);
void set_fan1(int value);
void set_fan2(int value);
void set_fan3(int value);
void set_aquafresh(int value);
void set_volume(int value);
void play_message(int index);
void new_read_eeprom(void);
void new_write_eeprom(void);
void show_digit_Ergoline(int digit);
void ProcessButtonsErgoline(void);
void set_clima(int value);
void set_licevi_lamps(int value);
typedef struct time_str{
	 uint16_t hours_h;
	 uint16_t hours_l;
	 uint16_t minutes;
}time_str;
typedef struct settings_str{
	 uint16_t addresse;
	 uint16_t pre_time;
	 uint16_t cool_time;
	 uint16_t ext_mode;
	 uint16_t volume;
	 uint16_t temperatue_max;
}settings_str;
typedef struct flash_struct{
	 time_str time;
	 settings_str settings;
}flash_struct;
flash_struct flash_data;
uint16_t VirtAddVarTab[] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14};//sizeof(flash_data)/2];

typedef enum ext_modes {ext_mode_none, ext_mode_colarium}ext_modes;
typedef enum lamps_modes {lamps_all, lamps_uv, lamps_colagen}lamps_modes;
typedef enum states {state_show_time,state_set_time,state_show_hours,state_enter_service,state_clear_hours,state_address,state_pre_time,state_cool_time,state_ext_mode, state_volume, state_max_temp}states;
states state;
typedef enum modes {mode_null,mode_clear_hours,mode_set_address,mode_set_pre_time,mode_set_cool_time, mode_set_ext_mode, mode_set_volume, mode_set_max_temp}modes;
modes service_mode;
typedef enum voice_messages { message_power_up, message_start_working, message_stop_working }voice_messages;
int useUart=0;

char key_readings[9];
char col_index;
char row_index;
static int led_bits = 0x0;
static int selected_led_bits = 0x0;
static int percent_clima = 0, percent_kraka = 0;//, percent_fan1 = 0, percent_fan2 = 0;
static int flash_mode = 0;
static int last_button_ergoline = 0;
static int prev_button_ergoline = 0;
void scan_keys()
{
	for(col_index = 0; col_index < 3; col_index++)
	{
		if(col_index == 0)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
		}
		if(col_index == 1)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
		}
		if(col_index == 2)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		}
		for(row_index = 0; row_index < 3; row_index++)
		{
			if(row_index == 0)
			{
				key_readings[row_index + col_index*3] = (key_readings[row_index + col_index*3] << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);
			}
			if(row_index == 1)
			{
				key_readings[row_index + col_index*3] = (key_readings[row_index + col_index*3] << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);
			}
			if(row_index == 2)
			{
				key_readings[row_index + col_index*3] = (key_readings[row_index + col_index*3] << 1) | HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5);
			}


			if(key_readings[row_index + col_index*3] == 0x00)
			{
				key_states |= 1 << (row_index + col_index*3);
			}
			if(key_readings[row_index + col_index*3] == 0xFF)
			{
				key_states &= ~(1 << (row_index + col_index*3));
			}
		}
	}
	last_button = key_states;
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
  unsigned char val = 0;
  if (level == 1)
  {
	  val = 0x0f;
  }
  else if (level >= 2)
  {
	  val = 0xff;
  }
  LedControl_setRow(&led_control, 0, 6, val);
}

void show_digit(int digit){
	digit = digit & 0x0F;
}

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
//	char index = 0;
	__disable_irq();
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
		write_settings();// Paranoia check
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
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_IWDG_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();

  /* USER CODE BEGIN 2 */
  __enable_irq();
  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();
  EE_Init();

  read_settings();
  read_stored_time();

  __HAL_UART_ENABLE(&huart1);
//  __HAL_UART_ENABLE(&huart3);
//  HAL_TIM_Base_Start(&htim1);
//  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
  set_fan1(0);
  set_fan2(0);
  set_fan3(0);
  /* Enable the UART Parity Error Interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_PE);

    /* Enable the UART Error Interrupt: (Frame error, noise error, overrun error) */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);

    /* Enable the UART Data Register not empty Interrupt */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
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
		scan_keys();
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
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_9);
		ProcessButtons();
		ProcessButtonsErgoline();
		switch (state) {
		case state_show_time:
		case state_set_time:
			if (!pre_time && !main_time && !cool_time) {
				flash_mode = 0;
				state = state_set_time;
				display_data = ToBCD(time_to_set) | 0xF000;
				//				display_data = ToBCD(last_button); //Debug
			}
			else if (!pre_time && !main_time && cool_time)
			{
				if(flash_counter & 0x20)
				{
					display_data = 0xFFFF;
				}
				else
				{
					display_data = ToBCD(abs(cool_time)) | ((Gv_miliseconds/1000)<< 16);
				}
			}
			else if (pre_time)
			{
				if(!(flash_counter & 0x38))
				{
					display_data = 0xFFFF;
				}
				else
				{
					display_data = ToBCD(abs(pre_time)) | ((Gv_miliseconds/1000)<< 16);
				}
			}
			else {
				state = state_show_time;
				//				  time_to_set = 0;
				display_data = ToBCD(abs(main_time)) | ((Gv_miliseconds/1000)<< 16);
				//				display_data = ToBCD(last_button); //Debug
			}
			break;
		case state_show_hours:

			if (flash_mode != 3) {
				flash_mode = 3; // All flashing
			}
			{
				int index = ((flash_counter / 0x20)) % 8;
				if ((index < 6) && (index & 1)) {
					display_data = ToBCD(work_hours[index/2]);
				}
				else {
					display_data = 0xFFFF;
				}
			}
			break;
		case state_enter_service:
			display_data = service_mode | 0xF0;
			flash_mode = 0;
			break;

		case state_clear_hours:
			//			flash_mode = 3;
			if ((flash_counter / 0x30) & 1) {
				display_data = 0xFFFC;
			}
			else {
				display_data = 0xFFFF;
			}
			break;
		case state_address:
			flash_mode = 0;
			if ((flash_counter / 0x30) & 1) {
				if (controller_address != 15) { // Address 15 reserved for external control
					display_data = controller_address;
				}
				else {
					display_data = 0xFEAF;
				}
			}
			else {
				display_data = 0xFFFA;
			}
			break;
		case state_pre_time:
			//			flash_mode = 3;
			if ((flash_counter / 0x30) & 1) {
				display_data = preset_pre_time | 0xF000;
			}
			else {
				display_data = 0xFFF3;
			}
			break;
		case state_cool_time:
			//			flash_mode = 3;
			if ((flash_counter / 0x30) & 1) {
				display_data = preset_cool_time | 0xF000;
			}
			else {
				display_data = 0xFFF4;
			}
			break;
		case state_ext_mode:
			//			flash_mode = 0;
			if ((flash_counter / 0x30) & 1) {
				display_data = ext_mode | 0xF000;
			}
			else {
				display_data = 0xFFF5;
			}
			break;

		case state_volume:
			//			flash_mode = 0;
			if ((flash_counter / 0x30) & 1) {
				display_data = volume_message;
			}
			else {
				display_data = 0xFFF6;
			}
			break;

		case state_max_temp:
			//			flash_mode = 0;
			if ((flash_counter / 0x30) & 1) {
				display_data = temperature_threshold;
			}
			else {
				display_data = 0xFFF7;
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
				percent_clima = 0;
				percent_kraka = 0;
				percent_fan1 = 0;
				percent_licevi = 0;
				set_fan1(0);
				set_fan2(0);
				set_fan3(0);
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
			ShowBarIndicators((flash_counter >> 4) %10, (flash_counter >> 4) % 10);
		}
		else {
			ShowBarIndicators(volume_level, fan_level);
		}
		show_level(aqua_fresh_level);
		show_digit_Ergoline(display_data);

		HAL_ADC_Start_IT(&hadc1);
		HAL_Delay(10);
//		display_data = 0xFFF;
//		for(int i = 0; i < 9; i++)
//		{
//			if (!key_readings[i])
//			{
//				display_data = i;
//				break;
//			}
//
//		}

		//Uncomment to check temperature
//		if(aqua_fresh_level == 0)
//		{
//		display_data = ToBCD(g_Temperature);
//		}
//		else
//		{
//			display_data = g_ADCMeanValue;
//		}
		LedControl_setDigit(&led_control, 0, 0, display_data & 0x0f, 0);
		LedControl_setDigit(&led_control, 0, 1, (display_data>>4) & 0x0f, 0);
		LedControl_setDigit(&led_control, 0, 2, (display_data>>8) & 0x0f, 0);
		flash_counter++;

		/* Reload IWDG counter */
//		IWDG_ReloadCounter();
		HAL_IWDG_Refresh(&hiwdg);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // Disable external reset connected to PB9

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

    /**Enables the Clock Security System 
    */
  HAL_RCC_EnableCSS();

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/** NVIC Configuration
*/
static void MX_NVIC_Init(void)
{
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
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

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
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
  htim1.Init.Prescaler = 19;
  htim1.Init.CounterMode = TIM_COUNTERMODE_DOWN;
  htim1.Init.Period = 31000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 8000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
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
  sConfigOC.Pulse = 4000;
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
  huart1.Init.BaudRate = 1200;
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
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|Clock_Pin 
                          |GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Data_Pin|Load_Pin|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 PA2 Clock_Pin 
                           PA9 PA11 PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|Clock_Pin 
                          |GPIO_PIN_9|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA3 PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Data_Pin Load_Pin PB11 PB12 
                           PB13 PB15 PB9 */
  GPIO_InitStruct.Pin = Data_Pin|Load_Pin|GPIO_PIN_11|GPIO_PIN_12 
                          |GPIO_PIN_13|GPIO_PIN_15|GPIO_PIN_9;
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
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */

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

void set_lamps(int value){
	lamps_state = value;
	if (value)	GPIOA->BSRR = GPIO_BSRR_BS12;
	else GPIOA->BSRR = GPIO_BSRR_BR12;
}

void set_colarium_lamps(int value){
	colaruim_lamps_state = value;
	if (value)	GPIOA->BSRR = GPIO_BSRR_BS11;
	else GPIOA->BSRR = GPIO_BSRR_BR11;
}

void set_fan2(int value){
	if (value)
	{
		GPIOA->BSRR = GPIO_BSRR_BS9;
	}
	else
	{
		GPIOA->BSRR = GPIO_BSRR_BR9;
	}

}

void set_fan3(int value){
	if (value)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_SET); // Relay ON
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_14, GPIO_PIN_RESET); // Relay OFF
	}

}

void set_fan1(int value){
	//
	GPIO_InitTypeDef GPIO_InitStruct;
//	uint32 counter = 0xFFFFFFF;
//	TIM_Cmd(TIM2, DISABLE);
	int multiplier = 285;
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
#ifndef LAMPI_KRAKA
	case 1:
		value = 0; // Po zelanie na Emil - da moze da se spira ventilatora
		break;
	case 0:
		tim_base = 0;
		break;
#endif
	default:
		tim_base = 63;
		break;
	}
//	if(value == 10) TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
//	else  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
//	TIM_TimeBaseStructure.TIM_Period = tim_base;
//	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
	/* TIM2 PWM2 Mode configuration: Channel4 */
	//for one pulse mode set PWM2, output enable, pulse (1/(t_wanted=TIM_period-TIM_Pulse)), set polarity high
//	if (value)	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
//	else 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
//	if (TIM_TimeBaseStructure.TIM_Period) TIM_OCInitStructure.TIM_Pulse = TIM_TimeBaseStructure.TIM_Period-5;
//	else  TIM_OCInitStructure.TIM_Pulse = 9;

//	TIM_OC4Init(TIM2, &TIM_OCInitStructure);
//	if (value) TIM_Cmd(TIM2, ENABLE);
//	else TIM_Cmd(TIM2, DISABLE);
//	TIM_Cmd(TIM2, ENABLE);
	if(value == 0)
	{
		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
	}
	else
	{
		MX_TIM1_Init();
		HAL_TIM_Base_Start(&htim1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		if(value == 10)
		{
			TIM1->CCR3 = TIM1->ARR -1;
		}
		else
		{
			TIM1->CCR3 = TIM1->ARR -(multiplier*tim_base);
		}
	}

}

void set_aquafresh(int value){

	if (value)	GPIOA->BSRR = GPIO_BSRR_BS15;
	else GPIOA->BSRR = GPIO_BSRR_BR15;
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
//	GPIOB->BRR = GPIO_BRR_BR_6;
//	GPIOF->BRR = GPIO_BRR_BR_6 | GPIO_BRR_BR_7;

	if (value >9)	{
		//GPIOB->BSRR = GPIO_BSRR_BS_6;
		//GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		//GPIOF->BSRR = GPIO_BSRR_BS_7;
	}
	else if (value >7)	{
//		GPIOB->BSRR = GPIO_BSRR_BS_6;
		//GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		//GPIOF->BSRR = GPIO_BSRR_BS_7;
	}
	else if (value >6)	{
		//GPIOB->BSRR = GPIO_BSRR_BS_6;
		//GPIOF->BSRR = GPIO_BSRR_BS_6 ;
//		GPIOF->BSRR = GPIO_BSRR_BS_7;
	}
	else if (value >5)	{
//		GPIOB->BSRR = GPIO_BSRR_BS_6;
		//GPIOF->BSRR = GPIO_BSRR_BS_6 ;
//		GPIOF->BSRR = GPIO_BSRR_BS_7;
		}
	else if (value >4)	{
		//GPIOB->BSRR = GPIO_BSRR_BS_6;
//		GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		//GPIOF->BSRR = GPIO_BSRR_BS_7;
		}
	else if (value >3)	{
//		GPIOB->BSRR = GPIO_BSRR_BS_6;
//		GPIOF->BSRR = GPIO_BSRR_BS_6 ;
		//GPIOF->BSRR = GPIO_BSRR_BS_7;
		}
	else if (value >1)	{
		//GPIOB->BSRR = GPIO_BSRR_BS_6;
//		GPIOF->BSRR = GPIO_BSRR_BS_6 ;
//		GPIOF->BSRR = GPIO_BSRR_BS_7;
		}
	else {
//		GPIOB->BSRR = GPIO_BSRR_BS_6;
//		GPIOF->BSRR = GPIO_BSRR_BS_6 ;
//		GPIOF->BSRR = GPIO_BSRR_BS_7;
	}
#endif
}

void usart1_IT_handler()
{
	 uint32_t isrflags   = READ_REG(USART1->SR);
	 uint32_t cr1its     = READ_REG(USART1->CR1);
//	 uint32_t cr3its     = READ_REG(USART1->CR3);
	 uint32_t errorflags = 0x00U;
//	 uint32_t dmarequest = 0x00U;
	 useUart = 1;
	 enum rxstates {rx_state_none, rx_state_pre_time, rx_state_main_time, rx_state_cool_time, rx_state_get_checksum};

	 /* If no error occurs */
	 errorflags = (isrflags & (uint32_t)(USART_SR_PE | USART_SR_FE | USART_SR_ORE | USART_SR_NE));
	 if(errorflags == RESET)
	 {
		 /* UART in mode Receiver -------------------------------------------------*/
		 if(((isrflags & USART_SR_RXNE) != RESET) && ((cr1its & USART_CR1_RXNEIE) != RESET))
		 {
			 data = USART1->DR & (uint16_t)0x00FF;
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
			 			USART1->DR = data;
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
			 			USART1->DR = data;
			 		}


			 	}
		 }
	 }
	 else
	 {
		 rx_state= 0;
	 }
}


void play_message(int index){

//	disk_initialize(0);
//	if(disk_status(0) == STA_NOINIT)
//	{
//		if(index == message_start_working)
//		{
//			fade_in_counter =  1000;
//			//TIM_Cmd(TIM14, ENABLE);
//		}
//		return;
//	}
//	dac_ping_pong_state = 0;
//	dac_current_message = index;
//	DRESULT result;
//	dac_out_counter = 0;
//
//	result = disk_read(0, &dac_buffer[dac_ping_pong_state], message_sector_offset[index], 2) ;
//	dac_current_block = 0;
//
//	dac_ping_pong_state = 0;
//	dac_prev_ping_pong_state = dac_ping_pong_state;
//	fade_in_counter =  1000;
//	if (result) dac_out_counter = dac_out_counter;
//	//DAC_Cmd(DAC_Channel_1, ENABLE);
//	//TIM6->DIER |= TIM_DIER_UIE; // Enable interrupt on update event
//	//TIM_Cmd(TIM6, ENABLE);
//	TIM_Cmd(TIM14, ENABLE);
//	//while(1)
//	{
//		if (dac_current_block < message_sector_counts[dac_current_message]){
//			if(dac_prev_ping_pong_state != dac_ping_pong_state)
//			{
//				dac_prev_ping_pong_state = dac_ping_pong_state;
//				//TIM14->CCER &= ~TIM_OutputState_Enable;
//				disk_read(0, &dac_buffer[dac_prev_ping_pong_state], message_sector_offset[dac_current_message] + dac_current_block, 2) ;
//				dac_current_block++;
//				dac_current_block++;
//				//TIM14->CCER |= TIM_OutputState_Enable;
//			}
//		}
//		//dac_current_block = 0;
//	}

//	TIM6->CR1 |= TIM_CR1_CEN;

}


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
								write_stored_time();
							}
							else
							{
								write_settings();
							}

							start_counter = 0;
							service_mode = mode_null;

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
				}
				if (state >= state_enter_service) {
					start_counter = 0;
					state = state_show_time;
					write_settings();
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
		if (start_counter < (START_COUNTER_TIME + ENTER_SERVICE_DELAY + 6 * SERVICE_NEXT_DELAY))
		{
			start_counter++;
		}
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
//		set_start_out_signal(1);
	}
	else
	{
//		set_start_out_signal(0);
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
//					new_write_eeprom();
//					new_read_eeprom();
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
				write_stored_time();
			}
			flash_mode = 0;
			percent_licevi = 100L, percent_fan2 = 100;
			zero_crossed = 0;
			//			volume_level = 6;
			fan_level = 7;
			percent_fan1 = fan_level;
			if (lamps_mode == lamps_all) {
				set_lamps(100);
				set_colarium_lamps(0);
				percent_clima = 0;
#ifdef LAMPI_KRAKA
				percent_kraka = 100;
				set_clima(percent_kraka);
#endif
			}
			if (lamps_mode == lamps_uv) {
				set_lamps(100);
			}
			if (lamps_mode == lamps_colagen) {
				set_colarium_lamps(100);
				percent_clima = 100;
				percent_kraka = 100;
			}
			zero_crossed = 0;
			set_fan1(percent_fan1);
			set_fan2(percent_fan2);
			set_fan3(1);
			play_message(message_start_working);
			set_volume(volume_level);
#ifdef LICEVI_LAMPI_VMESTO_AQAFRESH
		if (!minute_counter)aqua_fresh_level = 2;
		percent_licevi = 100;
		set_aquafresh(aqua_fresh_level);
#endif
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
				write_stored_time();
				play_message(message_stop_working);
			}
			percent_licevi = 0, percent_fan1 = 10L, percent_fan2 = 100L;

			aqua_fresh_level = 0;
			zero_crossed = 0;
			set_lamps(OFF);
			//			zero_crossed = 0;
			stop_delay = 1000;
			//			set_colarium_lamps(0);
			fan_level = 10;
			set_fan1(percent_fan1);
			set_fan2(percent_fan2);
			set_fan3(1);
			set_aquafresh(0);
			flash_mode = 3;


		}
		if (curr_status == STATUS_FREE) {
			percent_licevi = 0, percent_fan1 = 0, percent_fan2 = 0;
			aqua_fresh_level = 0;
			set_lamps(OFF);
			set_colarium_lamps(OFF);
			percent_clima = 0;
			percent_kraka = 0;
			set_fan1(0);
			set_fan2(OFF);
			set_fan3(OFF);

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

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if(tim_base == 0)
	{
		htim1.Instance->ARR = 0;
	}
	else
	{
		if(htim1.Instance->CNT > 10 )
		{
			//tim_reload_value++;
		}
		if(htim1.Instance->CNT < 5 )
		{
			//tim_reload_value--;
		}
//		test_data = htim1.Instance->CNT;
		htim1.Instance->ARR = tim_reload_value;
		htim1.Instance->CNT = 1;
	}

}

void HAL_SYSTICK_Callback(void)
{
	if( ++ refresh_counter>200L){
		refresh_counter = 0;
		flash_counter++;
		if(auto_exit_fn && (flash_counter&1))
		{
			auto_exit_fn--;
			if(!auto_exit_fn)
			{
				selected_led_bits &= ~(LED_BUTTONS_MASK);
			}
		}
	}
#ifndef LICEVI_LAMPI_VMESTO_AQAFRESH
	if(aqua_fresh_level == 1){
		if(Gv_miliseconds %30000 >29000L ){
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
			//GPIOC->BSRR =  GPIO_BSRR_BS_3; // Internal sound
			silence_counter = 100;
		}
	}

	if(fade_out_counter){
		fade_out_counter--;
//		if(!(fade_out_counter%100)){
//			set_volume(10 - fade_out_counter/100);
//		}
		if(fade_out_counter<=0){
			//TIM_Cmd(TIM14, DISABLE);
		}
		//GPIOC->BRR =  GPIO_BRR_BR_3; // External sound
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
			percent_clima = 100;
			percent_kraka = 100;
			set_fan1(percent_fan1);
			set_fan2(ON);
			set_fan3(1);
		}
	}
	if(stop_delay)
	{
		stop_delay--;
		if(!stop_delay && curr_status != STATUS_WORKING)
		{
			set_colarium_lamps(OFF);
			percent_clima = 0;
			percent_kraka = 0;
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



void read_settings(void)
{
//	uint16_t counter;
	uint8_t result = 0;
//	flash_struct *flash_mem = &flash_data;
	uint16_t  data;

	result += EE_ReadVariable(ADDRESS_ADDRESSE,  &data);
	controller_address = data;
	result += EE_ReadVariable(ADDRESS_PRE_TIME,  &data);
	preset_pre_time = data;
	result += EE_ReadVariable(ADDRESS_COOL_TIME,  &data);
	preset_cool_time = data;
	result += EE_ReadVariable(ADDRESS_EXT_MODE,  &data);
	ext_mode = data;
	result += EE_ReadVariable(ADDRESS_VOLUME,  &data);
	volume_message = data;
	result += EE_ReadVariable(ADDRESS_TEMP_MAX,  &data);
	temperature_threshold = data;

	if(result)
	{
		// Set defaults
		preset_pre_time = 7;
		preset_cool_time = 3;
		controller_address = 14;
		volume_message = 5;
		temperature_threshold = 80;
		ext_mode = 0;
		work_hours[0] = 0;
		work_hours[1] = 0;
		work_hours[2] = 0;
		write_settings();
		write_stored_time();
	}
}

void read_stored_time(void)
{
	uint16_t  data;
	if(!EE_ReadVariable(ADDRESS_HOURS_H,  &data))
	{
		work_hours[0] = data;
	}
	else
	{
		work_hours[0] = 0;
	}
	if(!EE_ReadVariable(ADDRESS_HOURS_L,  &data))
	{
		work_hours[1] = data;
	}
	else
	{
		work_hours[1] = 0;
	}
	if(!EE_ReadVariable(ADDRESS_MINUTES,  &data))
	{
		work_hours[2] = data;
	}
	else
	{
		work_hours[2] = 0;
	}
}

void write_settings(void)
{
	uint16_t counter;
	for(counter = 0; counter < 3; counter++)
	{
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_ADDRESSE,  controller_address))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_PRE_TIME,  preset_pre_time))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_COOL_TIME,  preset_cool_time))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_EXT_MODE,  ext_mode))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_VOLUME,  volume_message))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_TEMP_MAX,  temperature_threshold))
		{
			// Second chance
			continue;
		}
		break;
	}
}

void write_stored_time(void)
{
	uint16_t counter;
	for(counter = 0; counter < 3; counter++)
	{
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_HOURS_H,  work_hours[0]))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_HOURS_L,  work_hours[1]))
		{
			// Second chance
			continue;
		}
		if(FLASH_COMPLETE != EE_WriteVariable(ADDRESS_MINUTES,  work_hours[2]))
		{
			// Second chance
			continue;
		}
		break;
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* AdcHandle)
  {
	//CorrectedValue = (((RawValue – RawLow) * ReferenceRange) / RawRange) + ReferenceLow
      g_ADCValue = HAL_ADC_GetValue(AdcHandle);
      g_ADCMeanValue = (g_ADCMeanValue*9 + g_ADCValue)/10;
      g_ADCValue = 1024 - g_ADCValue;
      g_MeasurementNumber++;
      g_Temperature = (g_Temperature*9 + (((g_ADCValue - ADC_0_DEGREE_VALUE)*366)/(ADC_36_6_DEGREE_VALUE - ADC_0_DEGREE_VALUE)) + 0)/10;
  }

void SendData16(unsigned short data)
{
	unsigned short counter, i;
	for (counter = 0; counter < 16; counter++)
	{
		HAL_GPIO_WritePin(Clock_GPIO_Port, Clock_Pin, GPIO_PIN_SET);
		for (i = 0; i< 200; i++); // some delay
		if(data & (1<<counter))
		{
			HAL_GPIO_WritePin(Data_GPIO_Port, Data_Pin, GPIO_PIN_RESET);
		}
		else
		{
			HAL_GPIO_WritePin(Data_GPIO_Port, Data_Pin, GPIO_PIN_SET);
		}

		HAL_GPIO_WritePin(Clock_GPIO_Port, Clock_Pin, GPIO_PIN_RESET);
		for (i = 0; i< 20; i++); // some delay
	}
}

unsigned short ReceiveData16(void)
{
	unsigned short counter, i, data;
	data = 0;
	HAL_GPIO_WritePin(Data_GPIO_Port, Data_Pin, GPIO_PIN_SET);
	for (counter = 0; counter < 16; counter++)
	{
		HAL_GPIO_WritePin(Clock_GPIO_Port, Clock_Pin, GPIO_PIN_RESET);
		for (i = 0; i< 200; i++); // some delay
		if(HAL_GPIO_ReadPin(Read_GPIO_Port, Read_Pin) ==  GPIO_PIN_RESET)
		{
			data |= (1<<counter);
		}

		HAL_GPIO_WritePin(Clock_GPIO_Port, Clock_Pin, GPIO_PIN_SET);
		for (i = 0; i< 20; i++); // some delay
	}
	return data;
}



void show_digit_Ergoline(int digit)
{

	int i,j;
	short int digit_data;
//	volatile uint16_t status;
	static int led_bits_tmp; // = led_bits;
	led_bits &= ~((LED_FAN1_1 |  LED_FAN1_2 |  LED_FAN1_3 |  LED_FAN1_4)|
			(LED_FAN2_1 |  LED_FAN2_2 |  LED_FAN2_3 |  LED_FAN2_4)|
			(LED_CLIMA_1 |  LED_CLIMA_2 |  LED_CLIMA_3 |  LED_CLIMA_4));
	if(fan_level>8)
	{
		led_bits |=  LED_FAN1_1 |  LED_FAN1_2 |  LED_FAN1_3 |  LED_FAN1_4;
	}
	else if(fan_level>6)
	{
		led_bits |=   LED_FAN1_2 |  LED_FAN1_3 |  LED_FAN1_4;
	}
	else if(fan_level>4)
	{
		led_bits |=    LED_FAN1_3 |  LED_FAN1_4;
	}
	else if(fan_level>2)
	{
		led_bits |=  LED_FAN1_4;
	}
	if(percent_fan2)
	{
		led_bits |=  LED_FAN2_1 |  LED_FAN2_2 |  LED_FAN2_3 |  LED_FAN2_4;
	}

#ifdef LAMPI_KRAKA
	if(percent_kraka)
	{
		led_bits |=  (LED_KRAKA_1 );
	}
#else
	if(percent_clima)
	{
		led_bits |=  (LED_CLIMA_1 |  LED_CLIMA_2 |  LED_CLIMA_3 |  LED_CLIMA_4);
	}
#endif

	led_bits_tmp = led_bits;
//	if(flash_mode == 2){ // DP cycling
//
//	}
//	if(((flash_mode == 3) ||(flash_mode == 1) )&&(flash_counter & 0x04)){
//		digit |= 0x00FF;
//	}
	if(flash_counter & 0x20){
		led_bits_tmp |= LED_BUTTONS_MASK;
	}
	else
	{
		led_bits_tmp |= (~selected_led_bits) & LED_BUTTONS_MASK;
	}
	//led_bits_tmp = 1 << (time_to_set);
//	digit = ((flash_counter>>6)%32);
//	digit = ToBCD(digit);
	led_bits_tmp = ~led_bits_tmp;
	SendData16(0);  // Flush buffers
	SendData16(0);
	SendData16(0);
	SendData16(0);
//	HAL_GPIO_WritePin(Load_GPIO_Port, Load_Pin, GPIO_PIN_SET); // enable shift FOR BUTTONS
	HAL_GPIO_WritePin(Load_GPIO_Port, Load_Pin, GPIO_PIN_SET); // disable shift FOR BUTTONS / Parallel load

	for (i = 0; i< 200; i++); // some delay
	HAL_GPIO_WritePin(Load_GPIO_Port, Load_Pin, GPIO_PIN_RESET);  // enable shift FOR BUTTONS
//	while(1){ //DEBUG
	for (i = 0; i< 200; i++); // some delay
	// LEDs 1

//	SendData16((~led_bits_tmp)>>16);
//	last_button_ergoline = 0x11111111;
	last_button_ergoline = ReceiveData16() ;
	HAL_GPIO_WritePin(Load_GPIO_Port, Load_Pin, GPIO_PIN_SET); // disable shift FOR BUTTONS / Parallel load

	for (i = 0; i< 200; i++); // some delay
	HAL_GPIO_WritePin(Load_GPIO_Port, Load_Pin, GPIO_PIN_RESET);  // enable shift FOR BUTTONS
//	while(1){ //DEBUG
	for (i = 0; i< 200; i++); // some delay
	last_button_ergoline |= ReceiveData16() ;
//	}
	last_button_ergoline &= 0x7FFF;

	SendData16((~led_bits_tmp)>>16);
	// LEDs 2
	SendData16((~led_bits_tmp )& 0xFFFF);

	// Rightmost 2 digits
	digit_data = digits4[(digit) & 0x0f] | (digits4[(digit>>4) & 0x0f] << 8);

	if(percent_licevi)	digit_data |= 0x0101;
//	digit_data = digit_data<<1;
	SendData16(digit_data);

	// Leftmost 2 digits
	{
		digit_data =  digits4[(digit>>8) & 0x0f]; // | (digits4[(digit >> 4) & 0x0f] << 8);
	}
	if(percent_licevi)	digit_data |= 0x0100;
//	digit_data = ~digit_data;
	if((STATUS_WORKING == curr_status) && (flash_counter & 0x10))
	{
		digit_data |= 0x0001;
	}


//	GPIOB->BSRR = GPIO_BSRR_BS_2; // enable shift FOR BUTTONS
//	for (i = 0; i< 2000; i++);
	SendData16(digit_data);

//	while(SPI_GetReceptionFIFOStatus(SPI1)) last_button = SPI_I2S_ReceiveData16(SPI1);
//	GPIOB->BRR = GPIO_BSRR_BS_2; // disable shift FOR BUTTONS
	for (i = 1; i<16; i++){ // bit 0 is junk
		j = (last_button>>i) & 1;
		if(!j){
			last_button = i;
			break;
		}
	}
	for (i = 0; i< 500; i++);
	HAL_GPIO_WritePin(Load_GPIO_Port, Load_Pin, GPIO_PIN_SET);
	for (i = 0; i< 50; i++);
	HAL_GPIO_WritePin(Load_GPIO_Port, Load_Pin, GPIO_PIN_RESET);
}

/**
 * @brief  Manage the activity on buttons
 * @param  None
 * @retval None
 */
void ProcessButtonsErgoline(void)
{

	if (last_button_ergoline)
	{
		if(last_button_ergoline != prev_button_ergoline){
			switch(last_button_ergoline){
			case BUTTON_START_ERGOLINE:
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
							//write_eeprom();
//							read_eeprom();
							read_settings();
							start_counter = 0;
							service_mode = mode_null;
						} else {
							start_counter = START_COUNTER_TIME;
						}
						state = state_show_hours;
					}
				}


				break;
			case BUTTON_FAN1:
				selected_led_bits &= ~(LED_BUTTONS_MASK ^ LED_FAN1_L);
				selected_led_bits ^= LED_FAN1_L;
				auto_exit_fn = 20;
				break;
			case BUTTON_FAN2:
				selected_led_bits &=  ~(LED_BUTTONS_MASK ^ LED_FAN2_L);
				selected_led_bits ^= LED_FAN2_L;
				auto_exit_fn = 20;
				break;
			case BUTTON_LICEVI:
				if(minute_counter){
					selected_led_bits &=  ~(LED_BUTTONS_MASK ^ LED_LICEVI_L);
					selected_led_bits ^= LED_LICEVI_L;
				}
				auto_exit_fn = 20;
				break;
#ifdef LAMPI_KRAKA
			case BUTTON_LAMPI_KRAKA:
					selected_led_bits &=  ~(LED_BUTTONS_MASK ^ LED_KRAKA_L);
					selected_led_bits ^= LED_KRAKA_L;
					auto_exit_fn = 20;
					break;
#else
			case BUTTON_CLIMA:
				selected_led_bits &=  ~(LED_BUTTONS_MASK ^ LED_CLIMA_L);
				selected_led_bits ^= LED_CLIMA_L;
				auto_exit_fn = 20;
				break;
#endif

			case BUTTON_PLUS_ERGOLINE:
			{
				//				if(  led_bits){
				//					led_bits = led_bits <<1;
				//				} else {
				//					led_bits = 0x01;
				//				}
				auto_exit_fn = 20;

				if(state == state_show_hours) {
					state = state_set_time;
					start_counter = 0;
				}
				if(state == state_set_time){
					if(time_to_set < 32){
						if(!Gv_UART_Timeout)
							time_to_set ++;
						//			if((time_to_set & 0x0F)>9) time_to_set +=6;
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
				} else {
					if(selected_led_bits & LED_FAN2_L){
						if(percent_fan2<100)
						{
							percent_fan2=100;
							set_fan2(percent_fan2);
						}
					} else if(selected_led_bits & LED_FAN1_L){
						if(fan_level<10)
						{
							fan_level++;
							percent_fan1 = fan_level;
							set_fan1(percent_fan1);
						}
					} else if(selected_led_bits & LED_CLIMA_L){
						if(percent_clima<100)
						{
							percent_clima=100;
							set_clima(percent_clima);
						}
					}
					else if(selected_led_bits & LED_KRAKA_L){
						if(percent_kraka<100)
						{
							percent_kraka=100;
							set_clima(percent_kraka);
						}
					}
				}




			}
			break;
			case BUTTON_MINUS_ERGOLINE:
				//				if(  led_bits){
				//					led_bits = led_bits >>1;
				//				} else {
				//					led_bits = 0x01 << 31;
				//				}
				auto_exit_fn = 20;

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
				if(selected_led_bits & LED_FAN2_L){
#ifndef LAMPI_KRAKA
					if(percent_fan2)
					{
						percent_fan2 = 0;
						set_fan2(percent_fan2);
					}
#endif
				} else if(selected_led_bits & LED_FAN1_L){
					if(fan_level>1)
					{
						fan_level--;
						percent_fan1 = fan_level;
						set_fan1(percent_fan1);
					}
				}
				else if(selected_led_bits & LED_LICEVI_L){
					if(percent_licevi && minute_counter)
					{
						percent_licevi=0;
						aqua_fresh_level = 0;
						set_licevi_lamps(percent_licevi);
						update_status();
					}
				}
//				else if(selected_led_bits & LED_CLIMA_L){
//				if(percent_clima)
//				{
//					percent_clima = 0;
//					set_clima(percent_clima);
//				}
				else if(selected_led_bits & LED_KRAKA_L){
					if(percent_kraka)
					{
						percent_kraka = 0;
						set_clima(percent_kraka);
					}
				}

				break;
			default:
				while(0);
			}
			prev_button_ergoline =  last_button_ergoline;
		}
	} else {
		prev_button_ergoline = last_button_ergoline;
	}


	if (last_button_ergoline == BUTTON_START_ERGOLINE)
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
		if(curr_status == STATUS_WAITING && start_counter == START_DELAY){
			// Cancel start
			state = state_show_time;
			time_to_set = 0;
			preset_pre_time = 0;
			preset_cool_time = 0;
//			send_time();
//			read_eeprom();
			read_settings();
			//			start_counter = 0;

		}
		else if (curr_status == STATUS_WORKING && start_counter == START_DELAY)
		{
			main_time = 0;
			pre_time = 0;
			update_status();
			percent_fan1 = 10;
			set_fan1(percent_fan1);
			aqua_fresh_level = 0;
			percent_kraka = 0;
			percent_clima = 0;
			set_clima(percent_kraka);
			set_fan3(1);
		}
	}
//	else
//	{
//		if(start_counter){
//			if(state == state_show_hours){
//				start_counter--;
//				if(!start_counter){
//					state = state_show_time;
//				}
//			}
//			else if(state >= state_enter_service){
//				if(state == state_enter_service){
//					state = service_mode + state_enter_service;
//				}
//				start_counter--;
//				if(!start_counter){
//					state = state_show_time;
//				}
//			}
//			else {
//				start_counter = 0;
//
//			}
//		}
//		if(prev_status != curr_status){
//			if (prev_status == STATUS_WAITING && curr_status == STATUS_WORKING){
//				work_hours[2]+=time_to_set;
//				if(work_hours[2]>59){
//					work_hours[2]=work_hours[2]-60;
//					work_hours[1]++;
//					if(work_hours[1]>99){
//						work_hours[1] = 0;
//						work_hours[0]++;
//					}
//				}
//				//write_eeprom();
//				time_to_set = 0;
//				percent_clima = 0, percent_licevi = 100, percent_fan1 = 0, percent_fan2 = 100;
//				zero_crossed = 0;
//				set_lamps(100);
//				set_licevi_lamps(percent_licevi);
//				set_fan1(percent_fan1);
//				set_fan2(percent_fan2);
//				set_clima(percent_clima);
//			}
//			if (curr_status == STATUS_COOLING){
//				selected_led_bits = 0;
//				percent_licevi = 0, percent_fan1 = 100, percent_fan2 = 100;
//				set_lamps(0);
//				set_licevi_lamps(percent_licevi);
//				set_fan1(percent_fan1);
//				set_fan2(percent_fan2);
//				set_clima(percent_clima);
//			}
//			if (curr_status == STATUS_FREE){
//				percent_clima = 0, percent_licevi = 0, percent_fan1 = 0, percent_fan2 = 0;
//				set_lamps(0);
//				set_licevi_lamps(percent_licevi);
//				set_fan1(percent_fan1);
//				set_fan2(percent_fan2);
//				set_clima(percent_clima);
//				selected_led_bits = 0;
//				minute_counter = 0;
//			}
//			prev_status = curr_status;
//
//		}
//		//    LED1_OFF;
//	}
}

void set_clima(int value)
{
	set_colarium_lamps(value);
}

void set_licevi_lamps(int value)
{
	set_aquafresh(value);
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
//  while(1)
//  {
//  }
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
