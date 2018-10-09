/*
 * defines.h
 *
 *  Created on: 30.11.2013
 *      Author: didi
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define LAMPI_KRAKA
//#define STATUS_FREE    (0)
//#define STATUS_WAITING (3)
//#define STATUS_WORKING (1)
//#define STATUS_COOLING (2)
//#define START_COUNTER_TIME  1000
//#define ENTER_SERVICE_DELAY 2000
//#define SERVICE_NEXT_DELAY  200
//#define EXIT_SERVICE_TIME   400
//#define START_DELAY         200

// Buttons
#define BUTTON_START_ERGOLINE    (0x1800)
#define BUTTON_FAN2     (0x0300)
#define BUTTON_FAN1     (0x0600)
#define BUTTON_LICEVI   (0x001)
#define BUTTON_CLIMA    (0x0C00)
#define BUTTON_PLUS_ERGOLINE     (0x030)
#define BUTTON_MINUS_ERGOLINE    (0x060)
#define BUTTON_VOLUME    (0x00C)
#define BUTTON_AQUA_BODY (0x6000)
#define BUTTON_AQUA_HEAD (0x3000)
#define BUTTON_INFO     (0x018)
#define BUTTON_VOICE    (0x003)
#define BUTTON_LAMPI_KRAKA    (0x0C0)

// LEDs
#define LED_CLIMA_L           (1 << 0x00)
#define LED_RELAX_L           (1 << 0x01)
#define LED_VITAL_L           (1 << 0x02)
#define LED_SPRAY_HEAD_L      (1 << 0x03)
#define LED_SPRAY_BODY_L      (1 << 0x04)
#define LED_FAN1_L            (1 << 0x05)
#define LED_FAN2_L            (1 << 0x06)
#define LED_KRAKA_L         (1 << 0x07)
#define LED_KRAKA_1         (1 << 0x08)
#define LED_LICEVI_L          (1 << 0x09)
#define LED_CHANNEL_L         (1 << 0x0A)
#define LED_VOLUME_L          (1 << 0x0B)
#define LED_INFO_L            (1 << 0x0C)
#define LED_INFO_1            (1 << 0x0D)
#define LED_VOICE_GUIDE_0     (1 << 0x0E)
#define LED_VOICE_GUIDE_L     (1 << 0x0F)
#define LED_VITAL_1           (1 << 0x10)
#define LED_RELAX_1           (1 << 0x11)
#define LED_SPRAY_HEAD_1      (1 << 0x12)
#define LED_SPRAY_BODY_1      (1 << 0x13)
#define LED_FAN2_1            (1 << 0x14)
#define LED_FAN2_2            (1 << 0x15)
#define LED_FAN2_3            (1 << 0x16)
#define LED_FAN2_4            (1 << 0x17)
#define LED_FAN1_1            (1 << 0x18)
#define LED_FAN1_2            (1 << 0x19)
#define LED_FAN1_3            (1 << 0x1a)
#define LED_FAN1_4            (1 << 0x1b)
#define LED_CLIMA_4           (1 << 0x1C)
#define LED_CLIMA_3		      (1 << 0x1D)
#define LED_CLIMA_2           (1 << 0x1E)
#define LED_CLIMA_1           (1 << 0x1F)

#ifdef LAMPI_KRAKA
	#define LED_BUTTONS_MASK (LED_FAN1_L | LED_FAN2_L | LED_KRAKA_L)
#else
	#define LED_BUTTONS_MASK (LED_FAN1_L | LED_FAN2_L | LED_CLIMA_L | LED_LICEVI_L)
#endif
#define LED_UNUSED_BUTTON_MASK (LED_VOICE_GUIDE_L|LED_INFO_L|LED_VOLUME_L|LED_CHANNEL_L|\
		LED_KRAKA_L|LED_SPRAY_BODY_L|LED_SPRAY_HEAD_L|LED_VITAL_L|LED_RELAX_L)

// Bit map of 7 segment indicators
#define S_a3 (0x80)
#define S_b3 (0x40)
#define S_c3 (0x20)
#define S_d3 (0x10)
#define S_e3 (0x04)
#define S_f3 (0x08)
#define S_g3 (0x02)



const int digits4[] = {
// 0:
	(S_a3 | S_b3 | S_c3 | S_d3 | S_e3 | S_f3),
// 1:
	(S_b3 | S_c3),
// 2:
	( S_a3 | S_b3 |  S_d3 | S_e3 | S_g3),
// 3:
	( S_a3 | S_b3 | S_c3 | S_d3 | S_g3),
// 4:
	( S_b3 | S_c3 |  S_f3 | S_g3),
// 5:
	( S_a3 | S_c3 | S_d3 |  S_f3 | S_g3),
// 6:
	( S_a3 |  S_c3 | S_d3 | S_e3 | S_f3 | S_g3),
// 7:
	( S_a3 | S_b3 | S_c3 ),
// 8:
	( S_a3 | S_b3 | S_c3 | S_d3 | S_e3 | S_f3 | S_g3),
// 9:
	( S_a3 | S_b3 | S_c3 | S_d3 | S_f3 | S_g3),
// 0x0A:
	( S_a3 | S_b3 | S_c3 |  S_e3 | S_f3 | S_g3),
// 0x0B:
	( S_c3 | S_d3 | S_e3 | S_f3 | S_g3),
// 0x0C:
	( S_a3 |  S_d3 | S_e3 | S_f3 ),
// 0x0D:
	( S_b3 | S_c3 | S_d3 | S_e3 |  S_g3),
// 0x0E:
	( S_a3 |  S_d3 | S_e3 | S_f3 | S_g3),
// 0x0F:
	0, //(S_a3 |  S_e3 | S_f3 | S_g3),
// 0x10: "-"
	(S_g3),
// 0x11: " "
	0
};


// GPIOs
#define Clock_Pin GPIO_PIN_7
#define Clock_GPIO_Port GPIOA
#define Data_Pin GPIO_PIN_1
#define Data_GPIO_Port GPIOB
#define Load_Pin GPIO_PIN_10
#define Load_GPIO_Port GPIOB
#define Read_Pin GPIO_PIN_13
#define Read_GPIO_Port GPIOC


#endif /* DEFINES_H_ */
