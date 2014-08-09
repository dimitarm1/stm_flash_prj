/*
 * init.c
 *
 *  Created on: 30.11.2013
 *      Author: didi
 */
#include "init.h"
extern  unsigned char controller_address;

int selected_I2C_pair = 0;


void init_periph(){
	/*!< At this stage the microcontroller clock setting is already configured,
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
	 */

	/* GPIO Periph clock enable */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA | RCC_AHBPeriph_GPIOB | RCC_AHBPeriph_GPIOC | RCC_AHBPeriph_GPIOD | RCC_AHBPeriph_GPIOF, ENABLE);
	/* Zero cross timr (TIM2) */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
	/* UART1 Clock enable; SPI1 Clock enable*/
//	RCC_APB2PeriphClockCmd( RCC_APB2Periph_SPI1,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* DAC Periph clock enable */



	/* Configure PA in output push-pull mode (for segments)*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_8 | GPIO_Pin_11| GPIO_Pin_12 ; // LATER!| GPIO_Pin_13 | GPIO_Pin_14;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure PB in output push-pull mode (for segments  )*/
	//	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2  | GPIO_Pin_4  | GPIO_Pin_5 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_12 |  GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PC in output push-pull mode (for segments )*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9 | \
						GPIO_Pin_10 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* Configure PC in output open drain for volume regulator*/
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2|GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	//	Keypad inputs
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);


	//	Keypad inputs
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	/* Configure PF4 in output push-pull mode (for segments )*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4 | GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	//Configure SPI pins:   -----------------------------------------------------------------------
	/* in sdcard.c!!
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource15, GPIO_AF_0); // SPI1_NSS
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource7, GPIO_AF_0); // MISO
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_0); // MOSI
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource5, GPIO_AF_0); // SPI_CLK

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
	*/

	// Zero cross
	// Zero cross detection timer
	//Configure Timer 2 pins:   ----------------------------
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//Configure Timer 2 pins:   ----------------------------
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_3;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource1, GPIO_AF_2); // TIM2_CH2
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource3, GPIO_AF_2); // TIM2_CH4

	// Zero cross detection timer

	TIM_TimeBaseStructure.TIM_Prescaler = 4000 ;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_Period = 70 ;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	/* TIM2 PWM2 Mode configuration: Channel4 */
	//for one pulse mode set PWM2, output enable, pulse (1/(t_wanted=TIM_period-TIM_Pulse)), set polarity high
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;
	TIM_OCInitStructure.TIM_Pulse = 65;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	TIM_OC4Init(TIM2, &TIM_OCInitStructure);

	/* TIM2 configuration in Input Capture Mode */

	TIM_ICStructInit(&TIM_ICInitStructure);
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
	TIM_ICInitStructure.TIM_ICFilter = 0;


	TIM_ICInit(TIM2, &TIM_ICInitStructure);

	/* One Pulse Mode selection */
	TIM_SelectOnePulseMode(TIM2, TIM_OPMode_Single);

	/* Input Trigger selection */
	TIM_SelectInputTrigger(TIM2, TIM_TS_TI2FP2);

	/* Slave Mode selection: Trigger Mode */
	TIM_SelectSlaveMode(TIM2, TIM_SlaveMode_Trigger);

	TIM_OC4PreloadConfig(TIM2, TIM_OCPreload_Enable);

	TIM2->DIER |= TIM_DIER_UIE; // Enable interrupt on update event
	NVIC_EnableIRQ(TIM2_IRQn); // Enable TIM2 IRQ

	/* TIM2 enable counter */
	TIM_Cmd(TIM2, ENABLE);

	if(controller_address !=15){ // Address 15 reserved for external control


		//Configure USART1 pins:  Rx and Tx -------------------------------------------------------------
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
		//	USART_InvPinCmd(USART1,USART_InvPin_Rx,ENABLE);
		//	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
		USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
		//	USART1->CR1 |= USART_CR3_EIE;
		//	USART1->CR2 |= USART_CR2_TXINV;
		//	USART1->CR1 |= USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_TE | USART_CR1_UE ;
		//	USART_ReceiveData(USART1);
		USART_Cmd(USART1,ENABLE);
	}
	else {
		//Configure USART1 pins:  Rx and Tx - used to connect to external control
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_9 ;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
		GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_10;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
		GPIO_Init(GPIOA, &GPIO_InitStructure);
	}

	/* Enable USART1 IRQ */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// Timer 6 used for DAC playback
	RCC->APB1ENR |= RCC_APB1ENR_TIM6EN; // Enable TIM6 clock
	TIM6->PSC = 41; // Set prescaler to 41999
	TIM6->ARR = 599; // Set auto-reload to 5999
	// TIM6->CR1 |= TIM_CR1_OPM; // One pulse mode
	TIM6->CR1 |= TIM_CR1_ARPE; // Auto reload
	TIM6->EGR |= TIM_EGR_UG; // Force update
	TIM6->SR &= ~TIM_SR_UIF; // Clear the update flag
	TIM6->DIER |= TIM_DIER_UIE; // Enable interrupt on update event
	NVIC_EnableIRQ(TIM6_DAC_IRQn); // Enable TIM6 IRQ
	//TIM6->CR1 |= TIM_CR1_CEN; // Enable TIM6 counter



	/* Configure PA.04 (DAC_OUT1) as analog */
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* DAC channel1 Configuration */
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);
	DAC_DeInit();
	DAC_StructInit(&DAC_InitStructure);
	DAC_Init(DAC_Channel_1, &DAC_InitStructure);
	disk_initialize(0);
//	   RCC->APB1ENR|=RCC_APB1ENR_I2C1EN ;        //enable clock for I2C1
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C1,ENABLE);
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_I2C2,ENABLE);
	i2c_config_1();
//	if (SysTick_Config(SystemCoreClock / (1000))){
//		while(1); // Capture error
//	}
//	NVIC_SetPriority (SysTick_IRQn, 3);

}


void i2c_config_1(){
	// These channels are used for analog input
	selected_I2C_pair = 1;
	I2C_DeInit(I2C1);
	I2C_DeInit(I2C2);

	/* Un-Configure PB */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PB */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Configure PF */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource8, GPIO_AF_1); //I2C1 SDA
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource9, GPIO_AF_1); //I2C1 SCL

//	Port F has only 1 AF!!
//	GPIO_PinAFConfig(GPIOF, GPIO_PinSource6, GPIO_AF_1); //I2C2 SDA
//	GPIO_PinAFConfig(GPIOF, GPIO_PinSource7, GPIO_AF_1); //I2C2 SCL

	I2C_InitTypeDef i2c_init_str;
	I2C_StructInit(&i2c_init_str);

	I2C_Init(I2C1,&i2c_init_str);
	I2C_Init(I2C2,&i2c_init_str);

}

void i2c_config_2(){
	// These channels are used for DAC out
	selected_I2C_pair = 2;
	I2C_DeInit(I2C1);
	I2C_DeInit(I2C2);

	/* Un-Configure PB */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Un-Configure PF */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 ;
	GPIO_Init(GPIOF, &GPIO_InitStructure);

	/* Configure PB */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_10 | GPIO_Pin_11 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_1); //I2C1 SDA
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_1); //I2C1 SCL

	GPIO_PinAFConfig(GPIOB, GPIO_PinSource11, GPIO_AF_1); //I2C2 SDA
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_1); //I2C2 SCL

	I2C_InitTypeDef i2c_init_str;
	I2C_StructInit(&i2c_init_str);

	I2C_Init(I2C1,&i2c_init_str);
	I2C_Init(I2C2,&i2c_init_str);

}



// From forum https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Discovery/Flat.aspx?RootFolder=%2Fpublic%2FSTe2ecommunities%2Fmcu%2FLists%2FSTM32Discovery%2FI2C%20Wrong&FolderCTID=0x01200200770978C69A1141439FE559EB459D75800084C20D8867EAD444A5987D47BE638E0F&currentviews=563
void I2C_PCF_init()
 {
     RCC->AHBENR|=RCC_AHBENR_GPIOBEN;
     GPIOB->MODER|= GPIO_MODER_MODER6_1|GPIO_MODER_MODER7_1;
     GPIOB->OSPEEDR|=GPIO_OSPEEDER_OSPEEDR6|GPIO_OSPEEDER_OSPEEDR7;//predkosc 50MHZ
     GPIOB->AFR[0]|=0x11000000;    //AF1

     RCC->APB1ENR|=RCC_APB1ENR_I2C1EN ;        //enable clock for I2C1

     I2C1->CR1|=I2C_CR1_PE;                    //set PE
     I2C1->CR1&=~I2C_CR1_PE;                    //reset PE
     while(I2C1->CR1&I2C_CR1_PE);            //while PE ==1

   //  I2C1->TIMINGR|=(PRESC << 28)|(SCLL<<0)|(SCLH<<8)|(SCLDEL<<20)|(SDADEL<<16); //configure for 48Mhz clock

     I2C1->CR1|=I2C_CR1_PE;                    //set PE
 }



