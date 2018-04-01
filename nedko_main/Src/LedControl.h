/*
 *    LedControl.h - A library for controling Leds with a MAX7219/MAX7221
 *    Copyright (c) 2007 Eberhard Fahle
 * 
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 * 
 *    This permission notice shall be included in all copies or 
 *    substantial portions of the Software.
 * 
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef LedControl_h
#define LedControl_h

#include "stm32f1xx_hal.h"



#define MSBFIRST 1
#define LSBFIRST 2

typedef struct LedControl {

        /* The array for shifting the data to the devices up to 16 devices*/
        char spidata[32];
        /* Send out a single command to the device */

        /* We keep track of the led-status for all 16 devices in this array */
        char buffer[128];
        /* Data is shifted out of this pin*/
        uint16_t SPI_MOSI_pin;
        GPIO_TypeDef* SPI_MOSI_port;
        /* The clock is signaled on this pin */
        uint16_t SPI_CLK_pin;
        GPIO_TypeDef *SPI_CLK_port;
        /* This one is driven LOW for chip selectzion */
        uint16_t SPI_CS_pin;
        GPIO_TypeDef *SPI_CS_port;
        /* The maximum number of devices we use */
        int maxDevices;
}LedControl;

void LedControl_spiTransfer(int addr, char opcode, char data);


        /* 
         * Create a new controler 
         * Params :
         * dataPin		pin on the Arduino where data gets shifted out
         * clockPin		pin for the clock
         * csPin		pin for selecting the device 
         * numDevices	maximum number of devices that can be controled
         */
void LedControl_init(uint16_t dataPin, GPIO_TypeDef* data_port, uint16_t clkPin,  GPIO_TypeDef* clk_port, uint16_t csPin,  GPIO_TypeDef* cs_port, int numDevices);

        /*
         * Gets the number of devices attached to this LedControl.
         * Returns :
         * int	the number of devices on this LedControl
         */
int LedControl_getDeviceCount();

        /* 
         * Set the shutdown (power saving) mode for the device
         * Params :
         * addr	The address of the display to control
         * status	If true the device goes into power-down mode. Set to false
         *		for normal operation.
         */
void LedControl_shutdown(int addr, unsigned char status);

        /* 
         * Set the number of digits (or rows) to be displayed.
         * See datasheet for sideeffects of the scanlimit on the brightness
         * of the display.
         * Params :
         * addr	address of the display to control
         * limit	number of digits to be displayed (1..8)
         */
void LedControl_setScanLimit(int addr, int limit);

        /* 
         * Set the brightness of the display.
         * Params:
         * addr		the address of the display to control
         * intensity	the brightness of the display. (0..15)
         */
 void LedControl_setIntensity(int addr, int intensity);

        /* 
         * Switch all Leds on the display off. 
         * Params:
         * addr	address of the display to control
         */
 void LedControl_clearDisplay(int addr);

        /* 
         * Set the status of a single Led.
         * Params :
         * addr	address of the display 
         * row	the row of the Led (0..7)
         * col	the column of the Led (0..7)
         * state	If true the led is switched on, 
         *		if false it is switched off
         */
 void LedControl_setLed(int addr, int row, int col, unsigned char state);

        /* 
         * Set all 8 Led's in a row to a new state
         * Params:
         * addr	address of the display
         * row	row which is to be set (0..7)
         * value	each bit set to 1 will light up the
         *		corresponding Led.
         */
 void LedControl_setRow(int addr, int row, unsigned char value);

        /* 
         * Set all 8 Led's in a column to a new state
         * Params:
         * addr	address of the display
         * col	column which is to be set (0..7)
         * value	each bit set to 1 will light up the
         *		corresponding Led.
         */
 void LedControl_setColumn( int col, unsigned char value);

        /* 
         * Display a hexadecimal digit on a 7-Segment Display
         * Params:
         * addr	address of the display
         * digit	the position of the digit on the display (0..7)
         * value	the value to be displayed. (0x00..0x0F)
         * dp	sets the decimal point.
         */
 void LedControl_setDigit(int addr, int digit, unsigned char value, unsigned char dp);

        /* 
         * Display a character on a 7-Segment display.
         * There are only a few characters that make sense here :
         *	'0','1','2','3','4','5','6','7','8','9','0',
         *  'A','b','c','d','E','F','H','L','P',
         *  '.','-','_',' ' 
         * Params:
         * addr	address of the display
         * digit	the position of the character on the display (0..7)
         * value	the character to be displayed. 
         * dp	sets the decimal point.
         */
void LedControl_setChar(int addr, int digit, unsigned char value, unsigned char dp);
void LedControl_writeSprite(int x, int y, unsigned char* sprite);
void LedControl_reload();
void LedControl_printStringWithShift(char* s, int shift_speed);
void LedControl_printCharWithShift(char c, int shift_speed);
void LedControl_shiftLeft(char rotate, char fill_zero);

#endif	//LedControl.h



