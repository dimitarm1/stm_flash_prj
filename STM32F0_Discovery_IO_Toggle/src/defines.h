/*
 * defines.h
 *
 *  Created on: 30.11.2013
 *      Author: didi
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define STATUS_FREE    (0)
#define STATUS_WAITING (3)
#define STATUS_WORKING (1)
#define STATUS_COOLING (2)
#define START_COUNTER_TIME  1000
#define ENTER_SERVICE_DELAY 2000
#define SERVICE_NEXT_DELAY  200
#define EXIT_SERVICE_TIME   400
#define START_DELAY         200

// Buttons
#define BUTTON_START    (0x03)
#define BUTTON_FAN1     (0x06)
#define BUTTON_FAN2     (0x05)
#define BUTTON_LICEVI   (0x0F)
#define BUTTON_CLIMA    (0x04)
#define BUTTON_PLUS     (0x0A)
#define BUTTON_MINUS    (0x09)

// LEDs
#define LED_VOICE_GUIDE_L     (1 << 0x00)
#define LED_VOICE_GUIDE_0     (1 << 0x01)
#define LED_INFO_1            (1 << 0x02)
#define LED_INFO_L            (1 << 0x03)
#define LED_VOLUME_L          (1 << 0x04)
#define LED_CHANNEL_L         (1 << 0x05)
#define LED_LICEVI_L          (1 << 0x06)
#define LED_UNKNOWN_1         (1 << 0x07)
#define LED_UNKNOWN_L         (1 << 0x08)
#define LED_FAN1_L            (1 << 0x09)
#define LED_FAN2_L            (1 << 0x0a)
#define LED_SPRAY_BODY_L      (1 << 0x0b)
#define LED_SPRAY_HEAD_L      (1 << 0x0c)
#define LED_VITAL_L           (1 << 0x0d)
#define LED_RELAX_L           (1 << 0x0e)
#define LED_CLIMA_L           (1 << 0x0f)
#define LED_CLIMA_4           (1 << 0x10)
#define LED_CLIMA_3		      (1 << 0x11)
#define LED_CLIMA_2           (1 << 0x12)
#define LED_CLIMA_1           (1 << 0x13)
#define LED_FAN2_1            (1 << 0x14)
#define LED_FAN2_2            (1 << 0x15)
#define LED_FAN2_3            (1 << 0x16)
#define LED_FAN2_4            (1 << 0x17)
#define LED_FAN1_1            (1 << 0x18)
#define LED_FAN1_2            (1 << 0x19)
#define LED_FAN1_3            (1 << 0x1a)
#define LED_FAN1_4            (1 << 0x1b)
#define LED_SPRAY_BODY_1      (1 << 0x1c)
#define LED_SPRAY_HEAD_1      (1 << 0x1d)
#define LED_RELAX_1           (1 << 0x1e)
#define LED_VITAL_1           (1 << 0x1f)

#define LED_BUTTONS_MASK (LED_FAN1_L | LED_FAN2_L | LED_CLIMA_L | LED_LICEVI_L)
#define LED_UNUSED_BUTTON_MASK (LED_VOICE_GUIDE_L|LED_INFO_L|LED_VOLUME_L|LED_CHANNEL_L|\
		LED_UNKNOWN_L|LED_SPRAY_BODY_L|LED_SPRAY_HEAD_L|LED_VITAL_L|LED_RELAX_L)

// Bit map of 7 segment indicators
#define S_a3 ((1<<16)>>16)
#define S_b3 ((1<<17)>>16)
#define S_c3 ((1<<18)>>16)
#define S_d3 ((1<<19)>>16)
#define S_e3 ((1<<21)>>16)
#define S_f3 ((1<<20)>>16)
#define S_g3 ((1<<22)>>16)

#define S_a4 ((1<<24)>>16)
#define S_b4 ((1<<25)>>16)
#define S_c4 ((1<<26)>>16)
#define S_d4 ((1<<27)>>16)
#define S_e4 ((1<<29)>>16)
#define S_f4 ((1<<28)>>16)
#define S_g4 ((1<<30)>>16)

const int digits3[] = {
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

const int digits4[] = {
// 0:
	(S_a4 | S_b4 | S_c4 | S_d4 | S_e4 | S_f4),
// 1:
	(S_b4 | S_c4),
// 2:
	( S_a4 | S_b4 |  S_d4 | S_e4 | S_g4),
// 4:
	( S_a4 | S_b4 | S_c4 | S_d4 | S_g4),
// 4:
	( S_b4 | S_c4 |  S_f4 | S_g4),
// 5:
	( S_a4 | S_c4 | S_d4 |  S_f4 | S_g4),
// 6:
	( S_a4 |  S_c4 | S_d4 | S_e4 | S_f4 | S_g4),
// 7:
	( S_a4 | S_b4 | S_c4 ),
// 8:
	( S_a4 | S_b4 | S_c4 | S_d4 | S_e4 | S_f4 | S_g4),
// 9:
	( S_a4 | S_b4 | S_c4 | S_d4 | S_f4 | S_g4),
// 0x0A:
	( S_a4 | S_b4 | S_c4 |  S_e4 | S_f4 | S_g4),
// 0x0B:
	( S_c4 | S_d4 | S_e4 | S_f4 | S_g4),
// 0x0C:
	( S_a4 |  S_d4 | S_e4 | S_f4 ),
// 0x0D:
	( S_b4 | S_c4 | S_d4 | S_e4 |  S_g4),
// 0x0E:
	( S_a4 |  S_d4 | S_e4 | S_f4 | S_g4),
// 0x0F:
	0, //(S_a4 |  S_e4 | S_f4 | S_g4),
// 0x10: "-"
	(S_g4),
// 0x11: " "
	0
};



#endif /* DEFINES_H_ */
