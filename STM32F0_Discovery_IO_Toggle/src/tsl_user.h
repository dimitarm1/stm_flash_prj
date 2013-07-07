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
#define USE_LCD (0) // 0=No, 1=Yes

// LEDs definition on STM32F0-Discovery board
// LED1 = PC8
#define LED1_TOGGLE {GPIOC->ODR ^= 0x0200;}
#define LED1_ON    {GPIOC->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)8);}
#define LED1_OFF   {GPIOC->BRR = (uint32_t)((uint32_t)1 << ((uint32_t)8 ));}
// LED2 = PC9
#define LED2_TOGGLE {GPIOC->ODR ^= 0x0400;}
#define LED2_ON    {GPIOC->BSRR = (uint32_t)((uint32_t)1 << (uint32_t)9);}
#define LED2_OFF     {GPIOC->BRR = (uint32_t)((uint32_t)1 << ((uint32_t)9 ));}
//=======================
// Channel IOs definition
//=======================

#define CHANNEL_0_IO_MSK    (TSL_GROUP4_IO2)
#define CHANNEL_0_GRP_MSK   (TSL_GROUP4)
#define CHANNEL_0_SRC       (3) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_0_DEST      (0) // Index in destination result array

#define CHANNEL_1_IO_MSK    (TSL_GROUP4_IO3)
#define CHANNEL_1_GRP_MSK   (TSL_GROUP4)
#define CHANNEL_1_SRC       (3) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_1_DEST      (1) // Index in destination result array

#define CHANNEL_2_IO_MSK    (TSL_GROUP6_IO3)
#define CHANNEL_2_GRP_MSK   (TSL_GROUP6)
#define CHANNEL_2_SRC       (5) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_2_DEST      (2) // Index in destination result array

#define CHANNEL_3_IO_MSK    (TSL_GROUP6_IO4)
#define CHANNEL_3_GRP_MSK   (TSL_GROUP6)
#define CHANNEL_3_SRC       (5) // Index in source register (TSC->IOGXCR[])
#define CHANNEL_3_DEST      (3) // Index in destination result array

//======================
// Shield IOs definition
//======================

#define SHIELD_IO_MSK       (TSL_GROUP5_IO4)

//=================
// Banks definition
//=================


#define BANK_0_NBCHANNELS    (2)
#define BANK_0_MSK_CHANNELS  (CHANNEL_0_IO_MSK  | CHANNEL_2_IO_MSK )
#define BANK_0_MSK_GROUPS    (CHANNEL_0_GRP_MSK | CHANNEL_2_GRP_MSK) // Only these groups will be acquired


#define BANK_1_NBCHANNELS    (2)
#define BANK_1_MSK_CHANNELS  (CHANNEL_1_IO_MSK  | CHANNEL_3_IO_MSK )
#define BANK_1_MSK_GROUPS    (CHANNEL_1_GRP_MSK | CHANNEL_3_GRP_MSK) // Only these groups will be acquired

//#define BANK_2_NBCHANNELS    (0)
//#define BANK_2_MSK_CHANNELS  (0)
//#define BANK_2_MSK_GROUPS    (0) // Only these groups will be acquired

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
