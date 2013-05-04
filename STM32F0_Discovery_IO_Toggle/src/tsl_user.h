/**
  ******************************************************************************
  * @file    STM32F0518_Ex01_3TKeys_EVAL\inc\tsl_user.h
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    22-February-2013
  * @brief   Touch-Sensing user configuration and api file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TSL_USER_H
#define __TSL_USER_H

#include "tsl.h"

// Select to use or not the LCD
#define USE_LCD (1) // 0=No, 1=Yes

// LEDs definition on STM320518-EVAL board
// LED1 = PC10
#define LED1_TOGGLE {GPIOC->ODR ^= 0x0400;}
#define LED1_OFF    {GPIOC->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)10);}
#define LED1_ON     {GPIOC->BSRR = (uint32_t)((uint32_t)1 << ((uint32_t)10 + 16));}
// LED2 = PC11
#define LED2_TOGGLE {GPIOC->ODR ^= 0x0800;}
#define LED2_OFF    {GPIOC->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)11);}
#define LED2_ON     {GPIOC->BSRR = (uint32_t)((uint32_t)1 << ((uint32_t)11 + 16));}
// LED3 = PC12
#define LED3_TOGGLE {GPIOC->ODR ^= 0x1000;}
#define LED3_OFF    {GPIOC->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)12);}
#define LED3_ON     {GPIOC->BSRR = (uint32_t)((uint32_t)1 << ((uint32_t)12 + 16));}
// LED4 = PD2
#define LED4_TOGGLE {GPIOD->ODR ^= 0x0004;}
#define LED4_OFF    {GPIOD->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)2);}
#define LED4_ON     {GPIOD->BSRR = (uint32_t)((uint32_t)1 << ((uint32_t)2 + 16));}

//=======================
// Channel IOs definition
//=======================

#define CHANNEL_0_IO_MSK    (TSL_GROUP6_IO2)
#define CHANNEL_0_GRP_MSK   (TSL_GROUP6)
#define CHANNEL_0_SRC       (5) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_0_DEST      (0) // Index in destination result array

#define CHANNEL_1_IO_MSK    (TSL_GROUP6_IO3)
#define CHANNEL_1_GRP_MSK   (TSL_GROUP6)
#define CHANNEL_1_SRC       (5) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_1_DEST      (1) // Index in destination result array

#define CHANNEL_2_IO_MSK    (TSL_GROUP6_IO4)
#define CHANNEL_2_GRP_MSK   (TSL_GROUP6)
#define CHANNEL_2_SRC       (5) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_2_DEST      (2) // Index in destination result array

//======================
// Shield IOs definition
//======================

#define SHIELD_IO_MSK       (TSL_GROUP3_IO1)

//=================
// Banks definition
//=================

#define BANK_0_NBCHANNELS    (1)
#define BANK_0_MSK_CHANNELS  (CHANNEL_0_IO_MSK  | SHIELD_IO_MSK)
#define BANK_0_MSK_GROUPS    (CHANNEL_0_GRP_MSK) // Only these groups will be acquired

#define BANK_1_NBCHANNELS    (1)
#define BANK_1_MSK_CHANNELS  (CHANNEL_1_IO_MSK  | SHIELD_IO_MSK)
#define BANK_1_MSK_GROUPS    (CHANNEL_1_GRP_MSK) // Only these groups will be acquired

#define BANK_2_NBCHANNELS    (1)
#define BANK_2_MSK_CHANNELS  (CHANNEL_2_IO_MSK  | SHIELD_IO_MSK)
#define BANK_2_MSK_GROUPS    (CHANNEL_2_GRP_MSK) // Only these groups will be acquired

// User Parameters
extern CONST TSL_Bank_T MyBanks[];
extern CONST TSL_TouchKey_T MyTKeys[];
extern CONST TSL_Object_T MyObjects[];
extern TSL_ObjectGroup_T MyObjGroup;

void TSL_user_Init(void);
TSL_Status_enum_T TSL_user_Action(void);
void TSL_user_SetThresholds(void);

#endif /* __TSL_USER_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
