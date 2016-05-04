/*
 * defines.h
 *
 *  Created on: 30.11.2013
 *      Author: didi
 */

#ifndef DEFINES_H_
#define DEFINES_H_

#define STATUS_FREE    (0L)
#define STATUS_WAITING (3L)
#define STATUS_WORKING (1L)
#define STATUS_COOLING (2L)
#define START_COUNTER_TIME  (1000000L)
#define ENTER_SERVICE_DELAY (2000000L)
#define SERVICE_NEXT_DELAY  (400000L)
#define EXIT_SERVICE_TIME   (1200000L)
#define START_DELAY         (200000L)

// Buttons
#define BUTTON_PLUS      (0x001)
#define BUTTON_MINUS     (0x002)
#define BUTTON_START     (0x004)
#define BUTTON_COIN      (0x008)

#endif /* DEFINES_H_ */
