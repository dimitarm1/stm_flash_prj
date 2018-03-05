/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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
#include "stm32f4xx_hal.h"
#include "N5110.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
WWDG_HandleTypeDef hwwdg;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
/* Virtual address defined by the user: 0xFFFF value is prohibited */
uint16_t VirtAddVarTab[NB_OF_VAR] = {0x0000, 0x0001, 0x0003, 0x0004};
uint16_t VarDataTab[NB_OF_VAR] = {0, 0, 0, 0};
uint16_t VarValue,VarDataTmp = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MX_GPIO_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_WWDG_Init(void);

volatile long int counter1;
volatile int count1;
volatile int count2;
volatile int count3;
volatile int count4;
volatile unsigned char phase2 = 2;
volatile unsigned char phase3 = 3;
int8_t dir1, dir2, dir3, dir4;
int8_t mode; // 0 - tune phase, 1 - normal work

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{
  char buffer[16];  // each character is 6 pixels wide, screen is 84 pixels (84/6 = 14)
	// so can display a string of a maximum 14 characters in length
	// or create formatted strings - ensure they aren't more than 14 characters long
	// first need to initialise display
  int counter = 0;
  int mult = 1;
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Unlock the Flash Program Erase controller */
  HAL_FLASH_Unlock();

  /* EEPROM Init */
  if( EE_Init() != EE_OK)
  {
    Error_Handler();
  }

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
//  MX_WWDG_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  N5110_init();
  N5110_normalMode();      // normal colour mode
  N5110_setBrightness(0.7); // put LED backlight on 50%

  // can directly print strings at specified co-ordinates
  N5110_printString("**STM Nucleo**",0,0);
//  htim1.Instance->CCR1 = 0x01;
//  htim2.Instance->CCR1 = 0x01;
//  htim3.Instance->CCR1 = 0x01;
//  htim4.Instance->CCR1 = 0x01;
//  htim4.Instance->SMCR = 0x03;
//  htim1.Instance->CCER = 0x21;
//  htim1->Instance->DMAR = 0x01;

  count1 = htim8.Init.Period;
  count2 = TIM8->CCR2;
  count3 = TIM8->CCR3;
  count4 = TIM8->CCR4;
  __HAL_TIM_SET_COUNTER(&htim1,count1*4);
  __HAL_TIM_SET_COUNTER(&htim2,count2*4);
  __HAL_TIM_SET_COUNTER(&htim3,count3*4);
  __HAL_TIM_SET_COUNTER(&htim4,count4*4);
  __HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);
  (EE_ReadVariable(VirtAddVarTab[0],  &VarDataTab[0]));
  (EE_ReadVariable(VirtAddVarTab[1],  &VarDataTab[1]));
  (EE_ReadVariable(VirtAddVarTab[2],  &VarDataTab[2]));
  (EE_ReadVariable(VirtAddVarTab[3],  &VarDataTab[3]));
  if(VarDataTab[0] && VarDataTab[0] < 0x3F00)
  {
	  __HAL_TIM_SET_COUNTER(&htim1,VarDataTab[0] * 4 );
  }
  if(VarDataTab[1] && VarDataTab[1] < 0x3F00)
  {
  	  __HAL_TIM_SET_COUNTER(&htim2,VarDataTab[1] * 4 );
  	  TIM8->CCR2 = VarDataTab[1];
  }
  if(VarDataTab[2] && VarDataTab[2] < 0x3F00)
  {
  	  __HAL_TIM_SET_COUNTER(&htim3,VarDataTab[2] * 4 );
  	  TIM8->CCR3 = VarDataTab[2];
  }
  if(VarDataTab[3] && VarDataTab[3] < 0x3F00)
  {
  	  __HAL_TIM_SET_COUNTER(&htim4,VarDataTab[3] * 4 );
  	  TIM8->CCR4 = VarDataTab[3];
  }

  while (1)
  {
  /* USER CODE END WHILE */
	  //counter1 = __HAL_TIM_GET_COUNTER(&htim8);
	  HAL_Delay(150);

      count1=__HAL_TIM_GET_COUNTER(&htim1)/4;
      dir1 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1);
      if( TIM8->ARR != count1)
      {
    	  if(counter > 18)
    	  {
    		  if(mult<5)
    		  {
    			  mult = mult + 1;
    		  }
    		  TIM8->ARR -= (TIM8->ARR - count1)*mult;
    		  count1 = TIM8->ARR;
    		  __HAL_TIM_SET_COUNTER(&htim1,count1 * 4 );
    	  }
		  TIM8->ARR = count1;
    	  counter = 20; // delay write of new value with 7.5 seconds
      }

      count2=__HAL_TIM_GET_COUNTER(&htim2)/4;
      if(count2 > (count1-50))
      {
		  count2 = count1 - 54;
		  __HAL_TIM_SET_COUNTER(&htim2,count2 * 4 );
		  TIM8->CCR2 = count2;
	  }
      dir2 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim2);
      if( TIM8->CCR2 != count2)
      {
    	  if(counter > 18)
		  {
			  if(mult<5)
			  {
				  mult = mult + 1;
			  }
			  TIM8->CCR2 -= (TIM8->CCR2 - count2)*mult;
			  count2 = TIM8->CCR2;
			  __HAL_TIM_SET_COUNTER(&htim2,count2 * 4 );
		  }
		  TIM8->CCR2 = count2;
		  counter = 20; // delay write of new value with ~5 seconds
	  }

      count3=__HAL_TIM_GET_COUNTER(&htim3)/4;
      if(count3 > (count1-52))
      {
      	  count3 = count1 - 52;
       	  __HAL_TIM_SET_COUNTER(&htim3,count3 * 4 );
       	 TIM8->CCR3 = count3;
      }
      dir3 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim3);
      if( TIM8->CCR3 != count3)
      {
    	  if(counter > 18)
		  {
			  if(mult<5)
			  {
				  mult = mult + 1;
			  }
			  TIM8->CCR3 -= (TIM8->CCR3 - count3)*mult;
			  count3 = TIM8->CCR3;
			  __HAL_TIM_SET_COUNTER(&htim3,count3 * 4 );
		  }
   		  TIM8->CCR3 = count3;
   		  counter = 20; // delay write of new value with ~5 seconds
   	  }

      count4 = TIM8->CCR4 = count1 - 50;
/*
      count4=__HAL_TIM_GET_COUNTER(&htim4)/4;
      if(count4 > (count1-50))
      {
    	  count4 = count1 - 50;
    	  __HAL_TIM_SET_COUNTER(&htim4,count4 * 4 );
      }
      dir4 = __HAL_TIM_IS_TIM_COUNTING_DOWN(&htim4);
      if( TIM8->CCR4 != count4)
      {
    	  if(counter > 18)
		  {
			  if(mult<100)
			  {
				  mult = mult * 2;
			  }
			  TIM8->CCR4 -= (TIM8->CCR4 - count4)*mult;
			  count4 = TIM8->CCR4;
			  __HAL_TIM_SET_COUNTER(&htim4,count4 * 4 );
		  }
   		  TIM8->CCR4 = count4;
   		  counter = 20; // delay write of new value with ~5 seconds
   	  }
*/
      int length = sprintf(buffer,"C1 = %06d ",count1); // print formatted data to buffer
      // it is important the format specifier ensures the length will fit in the buffer

      N5110_printString(buffer,1,1);           // display on screen
      sprintf(buffer,"C2 = %06d ",((count2 + (count1-count4))*3600)/count1); // print formatted data to buffer
      N5110_printString(buffer,1,2);           // display on screen
      sprintf(buffer,"C3 = %06d ",((count3 + (count1-count4))*3600)/count1); // print formatted data to buffer
      N5110_printString(buffer,1,3);           // display on screen
      //sprintf(buffer,"C4 = %06d ",((count4 + (count1-count4))*3600)/count1); // print formatted data to buffer
//      N5110_printString(buffer,1,4);           // display on screen

      // can also print individual characters at specified place
      //lcd.printChar('X',5,3);

      // draw a line across the display at y = 40 pixels (origin top-left)
      for (int i = 0; i < WIDTH; i++)
      {
    	  N5110_setPixel(i,50);
      }
      // need to refresh display after setting pixels
      N5110_refresh();
   //   HAL_Delay(500);

      // can also check status of pixels using getPixel(x,y)

//        lcd.clear();
      if (counter == 1)
      {
    	  EE_WriteVariable(VirtAddVarTab[0],  count1);
    	  EE_WriteVariable(VirtAddVarTab[1],  count2);
    	  EE_WriteVariable(VirtAddVarTab[2],  count3);
    	  EE_WriteVariable(VirtAddVarTab[3],  count4);
      }
      if (counter)
      {
    	  counter--;
      }
      if(counter < 18)
      {
    	  mult = 1;
      }
  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
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

/* SPI2 init function */
static void MX_SPI2_Init(void)
{

  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = MAX_COUNT;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  HAL_TIM_Encoder_DeInit(&htim1);
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0x0F;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC2Filter = 0x0F;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_1)!=HAL_OK)
  {
	Error_Handler();
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = MAX_COUNT;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Encoder_DeInit(&htim2);
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0x0F;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC2Filter = 0x0F;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if(HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_1)!=HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = MAX_COUNT;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Encoder_DeInit(&htim3);
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0x0F;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC2Filter = 0x0F;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_1)!=HAL_OK)
  {
    Error_Handler();
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = MAX_COUNT;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Encoder_DeInit(&htim4);
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC1Filter = 0x0F;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV4;
  sConfig.IC2Filter = 0x0F;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if(HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_1)!=HAL_OK)
  {
	Error_Handler();
  }

}

/* TIM8 init function */
static void MX_TIM8_Init(void)
{


  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 7000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_Base_MspInit(&htim8);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_TIM_OC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = htim8.Init.Period/3;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.Pulse = 2*(htim8.Init.Period)/3;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 3*(htim8.Init.Period)/3 - 100;
  if (HAL_TIM_OC_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim8);
  //__HAL_TIM_ENABLE(&htim8);
  HAL_TIM_Base_Start(&htim8);
  HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_2);
  HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_3);
  HAL_TIM_OC_Start(&htim8, TIM_CHANNEL_4);


}

/* WWDG init function */
static void MX_WWDG_Init(void)
{

  hwwdg.Instance = WWDG;
  hwwdg.Init.Prescaler = WWDG_PRESCALER_1;
  hwwdg.Init.Window = 64;
  hwwdg.Init.Counter = 64;
  hwwdg.Init.EWIMode = WWDG_EWI_DISABLE;
  if (HAL_WWDG_Init(&hwwdg) != HAL_OK)
  {
    Error_Handler();
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PA2   ------> USART2_TX
     PA3   ------> USART2_RX
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : USART_TX_Pin USART_RX_Pin */
  GPIO_InitStruct.Pin = USART_TX_Pin|USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : N5110_DC_PIN */
  GPIO_InitStruct.Pin = N5110_DC_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(N5110_DC_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_R_W_Pin */
  GPIO_InitStruct.Pin = N5110_SCE_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(N5110_SCE_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_RESET_Pin */
  GPIO_InitStruct.Pin = N5110_RST_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(N5110_RST_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin : N5110_PWR_PIN */
  GPIO_InitStruct.Pin = N5110_PWR_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(N5110_PWR_PORT, &GPIO_InitStruct);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(N5110_SCE_PORT, N5110_SCE_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level - mode data*/
  HAL_GPIO_WritePin(N5110_DC_PORT, N5110_SCE_PIN, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(N5110_RST_PORT, N5110_RST_PIN, GPIO_PIN_RESET);

  /*Configure GPIO pin : Timer IT monitor */
   GPIO_InitStruct.Pin = GPIO_PIN_10;
   GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
   GPIO_InitStruct.Pull = GPIO_NOPULL;
   GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
   HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
   HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler */ 
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    //HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);

	if(htim->Instance == TIM8)
	{
		HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_10);
		TIM8->CCMR1 = (TIM8->CCMR2 & (~0x7000))|0x4000; // Force inactive state
		TIM8->CCMR1 = (TIM8->CCMR2 & (~0x7000))|0x3000; // Set toggle on match
		phase2 = !phase2;

		TIM8->CCMR2 = (TIM8->CCMR2 & (~0x0070))|0x0040;
		if(phase2)
		{
			TIM8->CCMR2 = (TIM8->CCMR2 & (~0x0070))|0x0030;
		}

		phase3--;
		TIM8->CCMR2 = (TIM8->CCMR2 & (~0x7000))|0x4000;
		if(!phase3)
		{
			phase3 = 3;
			TIM8->CCMR2 = (TIM8->CCMR2 & (~0x7000))|0x3000;
		}
	}
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
