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
#include "stm32f0xx.h"
#include "pitches.h"
#include "tsl_types.h"
#include "tsl_touchkey.h"
#include "tsl_user.h"
#include "tsl_conf_stm32f0xx.h"
#include "stm32f0xx_gpio.h"
#include "stm32f0xx_rcc.h"
#include "stm32f0xx_usart.h"


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

#define TEST_TKEY(NB) (MyTKeys[(NB)].p_Data->StateId == TSL_STATEID_DETECT)

#define STATUS_FREE    (0)
#define STATUS_WAITING (3)
#define STATUS_WORKING (1)
#define STATUS_COOLING (2)

/* Private variables ---------------------------------------------------------*/
GPIO_InitTypeDef        GPIO_InitStructure;
static long buzz_counter = 0;
static long pitches[255];
static long durations[255];
static int start_note = 0; // Or current note
static int end_note = 0;
static char digits[3];
static __IO uint32_t TimingDelay;
TSL_tMeas_T measurment;
static int display_data=0;
USART_InitTypeDef USART_InitStructure;
USART_ClockInitTypeDef USART_ClockInitStruct;

typedef enum states {state_show_time,state_set_time,state_show_hours,state_enter_service,state_clear_hours,state_address,
	state_pre_time,state_cool_time,state_ext_mode}states;
static states state;
typedef enum modes {mode_clear_hours,mode_set_address,mode_set_pre_time,mode_set_cool_time}modes;
static modes service_mode;
static unsigned char controller_address = 8; //0x10;
static int curr_status;
static int curr_time;
static int flash_mode = 0;
static unsigned char  time_to_set = 0;
static unsigned int   work_hours[3] = {9,10,30};
static unsigned char  preset_pre_time = 7;
static unsigned char  preset_cool_time = 3;
static int start_counter = 0;
static int counter_hours = 0;
static int flash_counter_prev = 0;

// for Display:
static int led_counter = 0;
static int flash_counter = 0;
static int digit_num = 0;

/* Private function prototypes -----------------------------------------------*/
void push_note(int pitch, int duration);
void ProcessSensors(void);
void SystickDelay(__IO uint32_t nTime);
void TSL_tim_ProcessIT(void);
void ping_status(void);
int ToBCD(int value);
void send_time(void);
void send_start(void);

/* Global variables ----------------------------------------------------------*/

__IO uint32_t Gv_SystickCounter;
extern __IO uint32_t Gv_EOA; // Set by TS interrupt routine to indicate the End Of Acquisition



//const TSL_TouchKeyMethods_T MyTKeys_Methods =
//{
//  TSL_tkey_Init,
//  TSL_tkey_Process
//};

TSL_TouchKeyData_T          MyTKeys_Data[4];        /**< Data (state id, counter, flags, ...) */
TSL_TouchKeyParam_T         MyTKeys_Param[4];       /**< Parameters (thresholds, debounce, ...) */
TSL_ChannelData_T           MyChannels_Data[4];     /**< Channel Data (Meas, Ref, Delta, ...) */
//// "basic" touchkeys: Always placed in ROM
//const TSL_TouchKeyB_T MyTKeys[TSLPRM_TOTAL_TKEYS] =
//{
//  { &MyTKeys_Data[0], &MyTKeys_Param[0], &MyChannels_Data[0] },
//  { &MyTKeys_Data[1], &MyTKeys_Param[1], &MyChannels_Data[1] },
//  { &MyTKeys_Data[2], &MyTKeys_Param[2], &MyChannels_Data[2] }
//};


/* Private functions ---------------------------------------------------------*/
void Delay(__IO uint32_t nTime)
{
  TimingDelay = nTime;

  while(TimingDelay != 0);
}

void show_digit(int digit){
	digit = digit & 0x0F;
	GPIOA->BSRR = GPIO_BSRR_BR_3 | GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5 | GPIO_BSRR_BR_6 | GPIO_BSRR_BR_7;
	GPIOB->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_1 | GPIO_BSRR_BR_2 | GPIO_BSRR_BR_10 | GPIO_BSRR_BR_11;
	GPIOC->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5 | GPIO_BSRR_BR_6 | GPIO_BSRR_BR_7;
	GPIOF->BSRR = GPIO_BSRR_BR_4 | GPIO_BSRR_BR_5;
	switch (digit) {
	case 0:
		GPIOA->BSRR = GPIO_BSRR_BS_3 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 1:
		GPIOA->BSRR = GPIO_BSRR_BS_7;
		GPIOB->BSRR = 0 ;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = 0;
		break;
	case 2:
		GPIOA->BSRR = GPIO_BSRR_BS_3 | GPIO_BSRR_BS_4;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 ;
		GPIOC->BSRR = GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 3:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 ;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = GPIO_BSRR_BS_5;
		break;
	case 4:
		GPIOA->BSRR = GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = 0;
		break;
	case 5:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		GPIOF->BSRR = GPIO_BSRR_BS_5;
		break;
	case 6:
		GPIOA->BSRR = GPIO_BSRR_BS_3 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 7:
		GPIOA->BSRR = GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = 0;
		break;
	case 8:
		GPIOA->BSRR = GPIO_BSRR_BS_3 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 9:
		GPIOA->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = GPIO_BSRR_BS_5;
		break;
	case 0x0A:
		GPIOA->BSRR = GPIO_BSRR_BS_3 | GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = GPIO_BSRR_BS_4;
		break;
	case 0x0B:
		GPIOA->BSRR = GPIO_BSRR_BS_3 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 0x0C:
		GPIOA->BSRR = GPIO_BSRR_BS_3 | GPIO_BSRR_BS_4;
		GPIOB->BSRR = GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = 0 ;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 0x0D:
		GPIOA->BSRR = GPIO_BSRR_BS_3 | GPIO_BSRR_BS_4 | GPIO_BSRR_BS_7;
		GPIOB->BSRR = GPIO_BSRR_BS_0 ;
		GPIOC->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5 | GPIO_BSRR_BS_6 | GPIO_BSRR_BS_7;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 0x0E:
		GPIOA->BSRR = GPIO_BSRR_BS_3 | GPIO_BSRR_BS_4 ;
		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
		GPIOC->BSRR = GPIO_BSRR_BS_5;
		GPIOF->BSRR = GPIO_BSRR_BS_4 | GPIO_BSRR_BS_5;
		break;
	case 0x0F:
	default:
		//empty
//		GPIOA->BSRR = GPIO_BSRR_BS_3 ;
//		GPIOB->BSRR = GPIO_BSRR_BS_0 | GPIO_BSRR_BS_1 | GPIO_BSRR_BS_2 | GPIO_BSRR_BS_10 | GPIO_BSRR_BS_11;
//		GPIOC->BSRR = GPIO_BSRR_BS_5;
//		GPIOF->BSRR = GPIO_BSRR_BS_4;
		break;
	}
}

int get_controller_status(int n){
	int counter = 10000;
	static int sts, data;
	// clear in fifo
	// send conmmand
	while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))	data =  USART_ReceiveData(USART1); // Flush input
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY
	USART_SendData(USART1,0x80 | ((n & 0x0f)<<3) | 0);
//    delta = (delta + 1)& 0xff;
//	USART_SendData(USART1,delta);

//	USART_SendData(USART1,0x0f);
	while(counter){
		//read Rx buffer
		sts = USART_GetFlagStatus(USART1,USART_FLAG_RXNE);
		if(sts) {
			data =  USART_ReceiveData(USART1);
//			while(1);
			return (data);
			break;
		}
		counter--;
	}
	return -1;
}
// Function to get controller's address
void get_address(void){
	int i, result;
	for (i = 0; i<16; i++){
		result = get_controller_status(i);
		if (result!=-1) break;
	}
	controller_address = i;
}


/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
int main(void)
{

	 TSL_Status_enum_T sts = 0;
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
     */

	 /* GPIOA Periph clock enable */
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
  /* GPIOC Periph clock enable */
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  /* GPIOA Periph clock enable */
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
  /* GPIOA Periph clock enable */
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOF, ENABLE);
 /* UART1 Clock enable */
	  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);



	  ///////////////////////////////////////// UART TEST
	  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);







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
 *    A
 *  F   B
 *    G
 *  E   C
 *    D
 *        DP
 */
	  /*
	   *
	   * */

	  /* Configure PA in output push-pull mode (for segments)*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5| GPIO_Pin_6|GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Configure PA0 -  PA2 in output push-pull mode (for Digits 0-2 )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 |GPIO_Pin_1 |GPIO_Pin_2 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);


	  /* Configure PB in output push-pull mode (for segments  )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2|GPIO_Pin_10 | GPIO_Pin_11;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Configure PB6 -  PB7 for UART*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0);

	  /* Configure PC in output push-pull mode (for segments )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5 | GPIO_Pin_6|GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* Configure PB9 in output push-pull mode (for buzzer )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Configure PF4 in output push-pull mode (for segments )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4|GPIO_Pin_5;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOF, &GPIO_InitStructure);

	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_0);
	  GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_0);

	  //Configure USART2 pins:  Rx and Tx ----------------------------
	  GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_6 | GPIO_Pin_7;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  USART_InitStructure.USART_BaudRate = 1200;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_Init(USART1, &USART_InitStructure);
	  USART_InvPinCmd(USART1,USART_InvPin_Tx,ENABLE);

	  USART_Cmd(USART1,ENABLE);

/*
 * commads:
 * 0 - status 0-free, 1-Working, 2-COOLING, 3-WAITING
 * 1 - start
 * 2 - set pre-time
 * 3 - set cool-time
 * 4 - stop - may be not implemented in some controllers
 * 5 - set main time
 */

	  RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // Enable TIM6 clock
	  TIM6->PSC = 41; // Set prescaler to 41999
	  TIM6->ARR = 599; // Set auto-reload to 5999
//	  TIM6->CR1 |= TIM_CR1_OPM; // One pulse mode
	  TIM6->CR1 |= TIM_CR1_ARPE; // Auto reload
	  TIM6->EGR |= TIM_EGR_UG; // Force update
	  TIM6->SR &= ~TIM_SR_UIF; // Clear the update flag
	  TIM6->DIER |= TIM_DIER_UIE; // Enable interrupt on update event
	  NVIC_EnableIRQ(TIM6_DAC_IRQn); // Enable TIM6 IRQ
	  TIM6->CR1 |= TIM_CR1_CEN; // Enable TIM6 counter

	  push_note(F2,3);
	  push_note(E2,3);
	  push_note(C2,3);
	  //============================================================================
	  // Init STMTouch driver
	  //============================================================================
	  if (SysTick_Config(SystemCoreClock / 1000))
	  {
		  /* Capture error */
		  while (1);
	  }
	  TSL_user_Init();


	  digits[0] = 0;
	  digits[1] = 1;
	  digits[2] = 2;

//	  while (1){
//		  while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) != RESET)
//			  USART_SendData(USART1,0x80 );
//	  }
	  while(0){
		  get_controller_status(8);
	  }
//	  get_address();
	  while (1)
	  {
//		  measurment = MyChannels_Data[0].Meas;
//		  display_data = measurment;

		  // Execute STMTouch Driver state machine
		  if ((sts = TSL_user_Action()) == TSL_STATUS_OK)
		  {
			  ProcessSensors(); // Execute sensors related tasks
		  }

		  else
		  {
			  // Execute other tasks...
			  if(controller_address>15){
				  // Try to get controller address continuously
				  get_address();
			  }
		  }
		  ping_status();

		  switch (state){
		  case state_show_time:
		  case state_set_time:
			  if(curr_status == STATUS_FREE ){
				  flash_mode = 0;
				  state = state_set_time;
				  display_data = ToBCD(time_to_set);
			  } else {
				  state = state_show_time;
				  time_to_set = 0;
				  display_data = ToBCD(curr_time);
				  if(curr_status == STATUS_WAITING ){
					  flash_mode = 1; // DP flashing
				  } else  if(curr_status == STATUS_WORKING ){
					  flash_mode = 2; // DP cycling
				  } else  if(curr_status == STATUS_COOLING ){
					  flash_mode = 3; // All flashing
				  } else {
					  display_data = 0xEEE;
					  flash_mode = 3; // All flashing
				  }
			  }
			  break;
		  case state_show_hours:

			  if( flash_mode != 3){
				  flash_mode = 3; // All flashing
				  flash_counter_prev = flash_counter = 0;
			  }
			  {
				  int index = (counter_hours)%4;
				  if(index<3){
					  display_data = ToBCD(work_hours[index]);
				  }
				  else {
					  display_data = 0xFFF;
				  }
				  if(flash_counter_prev != (flash_counter & 0x40)){
					  if(flash_counter_prev) counter_hours++;
					  flash_counter_prev = (flash_counter & 0x40);
				  }
			  }
			  break;
		  case state_enter_service:
			  display_data = service_mode;
		      flash_mode = 3; // All flashing
			  break;

		  case state_clear_hours:
		  	  break;
		  case state_address:
			  break;
		  case state_pre_time:
			  break;
		  case state_cool_time:
			  break;
		  case state_ext_mode:
			  break;
		  }

	  }
}

void push_note(int pitch, int duration){
	end_note = (end_note + 1)&0x3F;
	pitches[end_note] = pitch;
	durations[end_note] = duration;
	TIM6->DIER |= TIM_DIER_UIE; // Enable interrupt on update event
}

void get_next_note(){

	if(start_note == end_note){
		TIM6->DIER &= ~TIM_DIER_UIE; // Disable interrupt on update event
	} else {
		start_note = (start_note + 1)&0x3F;
		TIM6->ARR = pitches[start_note];
		buzz_counter = durations[start_note]*20000/ pitches[start_note];
	}
}

void TIM6_DAC_IRQHandler() {
	if((TIM6->SR & TIM_SR_UIF) != 0) // If update flag is set
		if(buzz_counter){
			buzz_counter--;
			if(buzz_counter & 1)
				GPIOB->BSRR = GPIO_BSRR_BS_9; // Set B9 high
			else
				GPIOB->BRR = GPIO_BSRR_BS_9; // Set B9 low
		}
	TIM6->SR &= ~TIM_SR_UIF; // Interrupt has been handled }
	if(!buzz_counter )	get_next_note();
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

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/**
  * @brief  Manage the activity on sensors when touched/released (example)
  * @param  None
  * @retval None
  */
void ProcessSensors(void)
{

	// TKEY 0
	if (TEST_TKEY(0))
	{
		// LED1_ON;
		if(start_counter<10000) start_counter++;
		if(start_counter== 5000){
			if((!curr_time)&&(state < state_enter_service)){
				push_note(C2,4);
				push_note(E2,4);
				push_note(G2,4);
				push_note(C3,4);
				push_note(G2,4);
				push_note(C3,8);
				state = state_enter_service;
				service_mode = 0;
			}
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
			else {
				start_counter = 0;
			}
		}
		//    LED1_OFF;
	}

	// TKEY 1
	if (TEST_TKEY(2))
	{
		LED2_ON;
	}
	else
	{
		LED2_OFF;
	}



#if USE_LCD > 0
	LcdDisplayStatus();
#endif
}



/**
  * @brief  Executed when a sensor is in Error state
  * @param  None
  * @retval None
  */
void MyTKeys_ErrorStateProcess(void)
{
  // Add here your own processing when a sensor is in Error state
  TSL_tkey_SetStateOff();
  LED1_ON;
  LED2_OFF;

  for (;;)
  {
	display_data = 0xEEE;
    LED1_TOGGLE;
    SystickDelay(100);
    display_data = 0x000;
    SystickDelay(100);
  }
}


/**
  * @brief  Executed when a sensor is in Off state
  * @param  None
  * @retval None
  */
void MyTKeys_OffStateProcess(void)
{
  // Add here your own processing when a sensor is in Off state
}


//-------------------
// CallBack functions
//-------------------

/**
  * @brief  Executed at each timer interruption (option must be enabled)
  * @param  None
  * @retval None
  */
void TSL_CallBack_TimerTick(void)
{
}


#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  LED1_OFF;
  LED2_ON;
  for (;;)
  {
    LED1_TOGGLE;
    LED2_TOGGLE;
    SystickDelay(100);
  }
}
#endif


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

void TimingDelay_Decrement(void)
{
	if (Gv_SystickCounter != 0x00)
	{
		Gv_SystickCounter--;
	}
	TSL_tim_ProcessIT();

	if(++led_counter>6){
		led_counter = 0;
		digit_num++;
		flash_counter++;
		if(digit_num>2) digit_num = 0;
		GPIOA->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_1 | GPIO_BSRR_BR_2; // Turn off the lights while changing them
		show_digit(((display_data & 0xFFF)& (0x0F<<(digit_num*4)))>>(digit_num*4));
		if(flash_mode < 3 ||(flash_counter & 0x40)){
			switch (digit_num){
			case 2:
				GPIOA->BSRR = GPIO_BSRR_BS_2 ;
				GPIOA->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_1;
				break;

			case 1:
				GPIOA->BSRR = GPIO_BSRR_BS_1 ;
				GPIOA->BSRR = GPIO_BSRR_BR_0 | GPIO_BSRR_BR_2;
				break;
			case 0:
			default:
				GPIOA->BSRR = GPIO_BSRR_BS_0 ;
				GPIOA->BSRR = GPIO_BSRR_BR_1 | GPIO_BSRR_BR_2;
				break;
			}
		}
		if (((flash_mode == 1)&& digit_num == 0 && (flash_counter & 0x40)) || ((flash_mode == 2) && (digit_num == 2))){
			GPIOA->BSRR = GPIO_BSRR_BS_6 | GPIO_BSRR_BS_5;
		}
	}
}

void Process_TS_Int(void){
	Gv_EOA = 1 +  (0!=(TSC->ISR & 0x02)); // Indicate Error also by MCEIF (MaxCount Error)

}

void key_pressed_event(void){
	TSL_tkey_DetectStateProcess();

	if(TSL_Globals.This_TKey->p_Data->Change == TSL_STATE_CHANGED){
		if(TSL_Globals.This_TKey->p_Data->StateId == TSL_STATEID_DETECT){
			TSL_Globals.This_TKey->p_Methods->Callback();
		}
	}
}

void KeyPressed_0(void){//START Key(Left)

//	push_note(E2,2);
//	push_note(G2,2);
//	push_note(C3,2);
	if( curr_status == STATUS_WAITING ){
		push_note(C2,2);
		push_note(E2,4);
		send_start();
	}
	if((curr_status == STATUS_FREE)) {
		if(time_to_set){
			state = state_show_time;
			push_note(E2,3);
			push_note(D2,3);
			push_note(C2,3);
			send_time();
		} else {
			state = state_show_hours;
			start_counter = 3000;
		}
	}
}
void KeyPressed_1(void){// STAT Key (Right)
	KeyPressed_0();
}
void KeyPressed_2(void){ // +
	if(state == state_set_time){
		if(time_to_set < 0x99){
			time_to_set ++;
//			if((time_to_set & 0x0F)>9) time_to_set +=6;
		}
	}
	push_note(D2,2);
	push_note(G2,2);
}
void KeyPressed_3(void){ // -

	if(state == state_set_time){
		if(time_to_set) {
			time_to_set--;
		}
	}

//	if((time_to_set & 0x0F)>9) time_to_set -=6;
	push_note(G2,2);
	push_note(D2,2);

}

void ping_status(void){
	static int ping_counter=0;
	// Ping solarium for status
	if(++ping_counter>600){
		ping_counter = 0;
		if(controller_address >15){
			display_data = 0xEEE;
		} else {
			int sts = get_controller_status(controller_address);
			if(sts != -1){
				curr_status = (sts & 0xC0)>>6;
				curr_time = sts & 0x3F;
			} else {
				curr_status = -1;
				curr_time = 0;
			}
		}
		curr_time = FromBCD(curr_time);
	}
}

void send_time(void){
	// Ping solarium for status
	if(controller_address >15){
		curr_status = -1;
		curr_time = 0;
	} else {
		int retry = 0;
		int retry2 =0;
		char data;
		int time_in_hex = ToBCD(time_to_set);
		preset_pre_time = preset_pre_time & 0x7F;
		// checksum: CoolTime + Pre-Time - 5 - MainTime

		while(retry < 20){
			// clear in FIFO
			volatile unsigned char checksum = (preset_pre_time + preset_cool_time  - time_in_hex - 5) & 0x7F;
			volatile unsigned char  remote_check_sum = 220;
			while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))	USART_ReceiveData(USART1); // Flush input
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY

			USART_SendData(USART1,0x80U | ((controller_address & 0x0fU)<<3U) | 2U); //Command 2 == Pre_time_set
			SystickDelay(2);
			USART_SendData(USART1,preset_pre_time);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY


			SystickDelay(2);
			USART_SendData(USART1,0x80U | ((controller_address & 0x0fU)<<3U) | 5U); //Command 5 == Main time set
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY

			SystickDelay(2);
			USART_SendData(USART1,time_in_hex);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY

//			SystickDelay(2);
			retry2 =0;
			while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE) && retry2++<10){
				SystickDelay(2);
			}
			USART_ReceiveData(USART1); //"Read" old time

			SystickDelay(2);
			USART_SendData(USART1,0x80U | ((controller_address & 0x0fU)<<3U) | 3U); //Command 3 == Cool Time set
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY

			SystickDelay(2);
			USART_SendData(USART1,preset_cool_time);
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY


			SystickDelay(4);
			retry2 =0;
			while(!USART_GetFlagStatus(USART1,USART_FLAG_RXNE) && retry2++<10){
				SystickDelay(2);
			}
			if(USART_GetFlagStatus(USART1,USART_FLAG_RXNE)){
				remote_check_sum = USART_ReceiveData(USART1); //"Read" checksum
			}
			SystickDelay(20);
//			checksum = remote_check_sum;
			if(remote_check_sum == checksum){
				SystickDelay(2);
				USART_SendData(USART1,checksum);
				while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY

			}
			SystickDelay(100);
			USART_SendData(USART1,0x80U | ((controller_address & 0x0f)<<3U) | 0); //Command 0 - Get status
			SystickDelay(20);
			data = USART_ReceiveData(USART1); //"Read" status
			if ((data >> 6 ) != 0 ) retry = 22;
			retry ++;
		}

		/**************** OLD Pascal code as example
		CheckSum:=PreTime+CoolTime-DataSent-5;
		    CheckSum:=CheckSum mod 128 ;
		    PurgeComm(hDevice,(PURGE_TXCLEAR or PURGE_RXCLEAR	));
		    while retry<20 do
		     begin
		      for i:=1 to 10 do IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);
		      Data1:=128+Chanel*8+2;
		      IOResult:=WriteFile(hDevice,Data1,1,IOCount,NIL);
		      sleep(2);             // Set PRE-Time
		      IOResult:=WriteFile(hDevice,PreTime,1,IOCount,NIL);
		      Data1:=128+Chanel*8+5;
		      sleep(2);              // Set main time
		      IOResult:=WriteFile(hDevice,Data1,1,IOCount,NIL);
		      sleep(2);
		      IOResult:=WriteFile(hDevice,DataSent,1,IOCount,NIL);
		      sleep(2);
		      IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);   // get old main time
		      Data1:=128+Chanel*8+3; //  Set cool time
		      IOResult:=WriteFile(hDevice,Data1,1,IOCount,NIL);
		      sleep(2);
		      IOResult:=WriteFile(hDevice,CoolTime,1,IOCount,NIL);
		      sleep(4);
		      IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL); // Get checksum?
		      if IOResult and (IOByte=CheckSum) then
		      IOResult:=WriteFile(hDevice,CheckSum,1,IOCount,NIL);
		      sleep(100);
		      Data1:=128+Chanel*8; // Get status command for selected chanel
		      IOResult:=WriteFile(hDevice,Data1,1,IOCount,NIL);
		      sleep(5);
		      IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);
		      IOByte:= IOByte div 64;
		      if IOResult and (IOByte <>0) and
		        ((DataSent=0) or (PreTime > 0) or ((IOByte = 1) and (DataSent >0)))    then
		        begin
		          retry:=22;
		        end
		      else retry:= retry+1;
		      sleep(1);
		      MainForm.Gauge2.Progress:=retry;
		     end;
		     IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);
		     IOResult:=ReadFile(hDevice,IOByte,1,IOCount,NIL);

		*/
	}
}

void send_start(void){
	// Ping solarium for status
	if(controller_address >15){
		curr_status = -1;
		curr_time = 0;
	} else {
			while(USART_GetFlagStatus(USART1,USART_FLAG_RXNE))	USART_ReceiveData(USART1); // Flush input
			while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET); // wAIT UNTIL TX BUFFER IS EMPTY

			USART_SendData(USART1,0x80 | ((controller_address & 0x0f)<<3) | 1); //Command 1 == start
			SystickDelay(2);
			USART_SendData(USART1,0x55);
	}
}

/**
 * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
