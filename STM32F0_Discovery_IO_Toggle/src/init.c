/*
 * init.c
 *
 *  Created on: 30.11.2013
 *      Author: didi
 *
 *
 *      Hardware pins used:
 *      Lampi  		- GPIOB.13
 *      Licevi 		- GPIOB.14
 *
 *      Ventilator1 (Reg) - Using Timer2
 *
 *      	CH2 	- GPIOA.1 - Zero cross input
 *      	CH4 	- GPIOA.3 - Output
 *
 *   	Ventilator2 - GPIOA.10 -->PF0
 *   	Climatik 	- GPIOA.11 -->PC15
 *
 *   	Indication:
 *
 *   	Shift Enable: GPIOB.2-> xxxx
 *
 *   	SPI1:
 *   		SPI_CLK - GPIOA.5
 *   		MOSI 	- GPIOA.6
 *   		MISO	- GPIOA.7
 *
 *   	USART1:
 *   		TxD	-	GPIOB.6
 *   		RxD	- 	GPIOA.10
 *
 */
#include "init.h"


void init(){
	/*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
	 */

	/* GPIO Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOF, ENABLE);
	/* UART1 Clock enable; SPI1 Clock enable*/
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1 | RCC_APB2Periph_SPI1,ENABLE);


	/* Configure PA in output push-pull mode (for segments)*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3 | GPIO_Pin_12 ; // LATER!| GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PB in output push-pull mode (for segments  )*/
	//	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2  | GPIO_Pin_4  | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Pin = 0;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	/* Configure PC in output push-pull mode (for segments )*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PF4 in output push-pull mode (for segments )*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);


	//	  //Configure SPI pins:   ----------------------------
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//	  GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_0); // SPI1_NSS
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0); // MISO
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0); // MOSI
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0); // SPI_CLK

	//Configure Timer 1 pins:   ----------------------------
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_2); // TIM1_CH1
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_2); // TIM1_CH1N
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_2); // TIM1_CH2N

	//Configure USART1 pins:  Rx and Tx ----------------------------
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1); // USART1_TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1); // USART1_RX

	USART_InitStructure.USART_BaudRate = 1200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	USART_InvPinCmd(USART1,USART_InvPin_Tx,ENABLE);
	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
//	USART1->CR1 |= USART_CR3_EIE;
//	USART1->CR2 |= USART_CR2_TXINV;
//	USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE ;
//	USART_ReceiveData(USART1);
	USART_Cmd(USART1,ENABLE);

	/* Enable USART1 IRQ */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	SPI_StructInit(&SPI_InitStruct);
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_16b;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_256;
	SPI_Init(SPI1,&SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);

	// Zero cross detection timer

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	TIM_TimeBaseStructure.TIM_Prescaler = 4000 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 70 ;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	/* TIM1 PWM2 Mode configuration: Channel4 */
	//for one pulse mode set PWM2, output enable, pulse (1/(t_wanted=TIM_period-TIM_Pulse)), set polarity high
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 65;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);

	/* TIM1 configuration in Input Capture Mode */

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0;


	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	/* One Pulse Mode selection */
	TIM_SelectOnePulseMode(TIM1, TIM_OPMode_Single);

	/* Input Trigger selection */
	TIM_SelectInputTrigger(TIM1, TIM_TS_TI1FP1);

	/* Slave Mode selection: Trigger Mode */
	TIM_SelectSlaveMode(TIM1, TIM_SlaveMode_Trigger);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	/* OPM Bit -> Only one pulse */
	TIM_SelectOnePulseMode (TIM1, TIM_OPMode_Single);
	TIM1->DIER |= TIM_DIER_UIE; // Enable interrupt on update event
	NVIC_EnableIRQ(TIM1_CC_IRQn); // Enable TIM1 IRQ
	/* TIM2 enable counter */
	TIM_Cmd(TIM1, ENABLE);
	SysTick_Config(SystemCoreClock / (1000));
	NVIC_SetPriority (SysTick_IRQn, 3);
}
