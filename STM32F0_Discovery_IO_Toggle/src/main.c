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
static int set_time;
static int current_time;


/* Private function prototypes -----------------------------------------------*/
void push_note(int pitch, int duration);
void TSL_tkey_Init(void); /**< Used to initialize the TouchKey sensor */
void TSL_tkey_Process(void); /**< Used to execute the TouchKey sensor state machine */
void ProcessSensors(void);
void SystickDelay(__IO uint32_t nTime);
void TSL_tim_ProcessIT(void);


/* Global variables ----------------------------------------------------------*/

__IO uint32_t Gv_SystickCounter;
extern __IO uint32_t Gv_EOA; // Set by TS interrupt routine to indicate the End Of Acquisition



//const TSL_TouchKeyMethods_T MyTKeys_Methods =
//{
//  TSL_tkey_Init,
//  TSL_tkey_Process
//};

TSL_TouchKeyData_T           MyTKeys_Data[3];    /**< Data (state id, counter, flags, ...) */
 TSL_TouchKeyParam_T         MyTKeys_Param[3];   /**< Parameters (thresholds, debounce, ...) */
 TSL_ChannelData_T           MyChannels_Data[3];     /**< Channel Data (Meas, Ref, Delta, ...) */
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
	GPIOB->BSRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14 ;
	GPIOC->BSRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15 ;
	switch (digit) {
	case 0:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14;
		break;
	case 1:
		GPIOB->BRR = GPIO_Pin_11|GPIO_Pin_12 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1 ;
		break;
	case 2:
		GPIOB->BRR = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14 ;
		break;
	case 3:
		GPIOB->BRR = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7 ;
		break;
	case 4:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 ;
		break;
	case 5:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 |GPIO_Pin_6|GPIO_Pin_7;
		break;
	case 6:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14 ;
		break;
	case 7:
		GPIOB->BRR = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1 ;
		break;
	case 8:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14 ;
		break;
	case 9:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 ;
		break;
	case 0x0A:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_13|GPIO_Pin_14 ;
		break;
	case 0x0B:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14 ;
		break;
	case 0x0C:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14 ;
		break;
	case 0x0D:
		GPIOB->BRR = GPIO_Pin_11|GPIO_Pin_12 ;
		GPIOC->BRR = GPIO_Pin_0|GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14 ;
		break;
	case 0x0E:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_13|GPIO_Pin_14 ;
		break;
	case 0x0F:
	default:
		GPIOB->BRR = GPIO_Pin_2|GPIO_Pin_10|GPIO_Pin_13|GPIO_Pin_14 ;
		GPIOC->BRR = GPIO_Pin_2|GPIO_Pin_3|GPIO_Pin_13|GPIO_Pin_14 ;
		break;
	}
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

/*
 * Outputs:
 * PA4 - Digit 0
 * PA5 - Digit 1
 * PA6 - Digit 2
 * PB13,PB14 - a
 * PB11,PB12 - b
 * PC0,PC1 - c
 * PC6,PC7 - d
 * PC13, PC14 - e
 * PB2,PB10 - f
 * PC2,PC3 - g
 * PC15 - DP
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

	  /* Configure PA4 -  PA6 in output push-pull mode (for Digits 0-2 )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOA, &GPIO_InitStructure);

	  /* Configure PB0 -  PB3 in output push-pull mode (for segment e )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2|GPIO_Pin_10 | GPIO_Pin_11 | GPIO_Pin_12| GPIO_Pin_13| GPIO_Pin_14;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOB, &GPIO_InitStructure);

	  /* Configure PC13 -  PC15 in output open drain mode (for segment e )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0| GPIO_Pin_1|GPIO_Pin_2|GPIO_Pin_3 | GPIO_Pin_7|GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* Configure PC13 -  PC15 in output open drain mode (for segment e )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 |GPIO_Pin_9 ;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOC, &GPIO_InitStructure);

	  /* Configure PF5 in output push-pull mode (for buzzer )*/
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	  GPIO_Init(GPIOF, &GPIO_InitStructure);



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
//	  TSL_acq_BankConfig(0);
//	  sts = TSL_acq_BankCalibrate(0);
//	  sts = TSL_acq_BankCalibrate(0);
//	  TSL_acq_BankStartAcq();
//	  TSL_acq_BankWaitEOC();
//	  measurment = TSL_acq_GetMeas(0);
	  digits[0] = 0;
	  digits[1] = 1;
	  digits[2] = 2;


	  while (1)
	  {
		  static int prev_digit_num;
		  static int led_counter = 0;
		  static int digit_num = 0;

		  measurment = MyChannels_Data[1].Meas;
		  // Execute STMTouch Driver state machine
		  if ((sts = TSL_user_Action()) == TSL_STATUS_OK)
		  {
			  ProcessSensors(); // Execute sensors related tasks
		  }

		  else
		  {
			  // Execute other tasks...
		  }
		  led_counter++;

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
				GPIOF->BSRR = GPIO_BSRR_BS_5; // Set F5 high
			else
				GPIOF->BRR = GPIO_BSRR_BS_5; // Set F5 low
		}
	TIM6->SR &= ~TIM_SR_UIF; // Interrupt has been handled }
	if(!buzz_counter )	get_next_note();
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
    LED1_ON;
  }
  else
  {
    LED1_OFF;
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
    LED1_TOGGLE;
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
	static int led_counter = 0;
	static int digit_num = 0;
	if(++led_counter>6){
		led_counter = 0;
		digit_num++;
		if(digit_num>2) digit_num = 0;
		GPIOA->BSRR = GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6; // Turn off the lights while changing them
		show_digit(((display_data & 0xFFF)& (0x0F<<(digit_num*4)))>>(digit_num*4));
		switch (digit_num){
		case 2:
			GPIOA->BRR = GPIO_Pin_4 ;
			GPIOA->BSRR = GPIO_Pin_5 | GPIO_Pin_6;
			break;

		case 1:
			GPIOA->BRR = GPIO_Pin_5 ;
			GPIOA->BSRR = GPIO_Pin_4 | GPIO_Pin_6;
			break;
		case 0:
		default:
			GPIOA->BRR = GPIO_Pin_6 ;
			GPIOA->BSRR = GPIO_Pin_4 | GPIO_Pin_5;
			break;
		}
	}
}

void Process_TS_Int(void){
	Gv_EOA = 1;
}

void key_pressed_event(void){
	TSL_tkey_DetectStateProcess();
	 if(TSL_Globals.This_TKey->p_Data->Change == TSL_STATE_CHANGED){
	    	  push_note(C3,2);
	    	  push_note(F2,2);
	    	  TSL_Globals.This_TKey->p_Methods->Callback();
	      }
}

void KeyPressed_0(void){
	if (display_data)display_data--;
	if((display_data & 0x0F)>9) display_data -=6;
}
void KeyPressed_1(void){
	if(display_data < 0x99){
		display_data = (display_data +1);
		if((display_data & 0x0F)>9) display_data +=6;
	}
}
void KeyPressed_2(void){

}
void KeyPressed_3(void){

}




/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
