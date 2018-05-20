/*
 *    LedControl.cpp - A library for controling Leds with a MAX7219/MAX7221
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


#include "LedControl.h"
#include "common_funcs.h"
#include <string.h>
#include "Font_16x16.h"

//the opcodes for the MAX7221 and MAX7219
#define OP_NOOP   0
#define OP_DIGIT0 1
#define OP_DIGIT1 2
#define OP_DIGIT2 3
#define OP_DIGIT3 4
#define OP_DIGIT4 5
#define OP_DIGIT5 6
#define OP_DIGIT6 7
#define OP_DIGIT7 8
#define OP_DECODEMODE  9
#define OP_INTENSITY   10
#define OP_SCANLIMIT   11
#define OP_SHUTDOWN    12
#define OP_DISPLAYTEST 15

#define BLOCKS_PER_ROW 5

unsigned char DeviceLUT[]={4,3,2,1,0,9,8,7,6,5,10,11};

LedControl led_control;

/*
 * Segments to be switched on for characters and digits on
 * 7-Segment Displays
 */

// Original
//const static char charTable []   = {
//    0B01111110,0B00110000,0B01101101,0B01111001,0B00110011,0B01011011,0B01011111,0B01110000,
//    0B01111111,0B01111011,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,
//    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
//    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
//    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
//    0B00000000,0B00000000,0B00000000,0B00000000,0B10000000,0B00000001,0B10000000,0B00000000,
//    0B01111110,0B00110000,0B01101101,0B01111001,0B00110011,0B01011011,0B01011111,0B01110000,
//    0B01111111,0B01111011,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
//    0B00000000,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,0B00000000,
//    0B00110111,0B00000000,0B00000000,0B00000000,0B00001110,0B00000000,0B00000000,0B00000000,
//    0B01100111,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
//    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00001000,
//    0B00000000,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,0B00000000,
//    0B00110111,0B00000000,0B00000000,0B00000000,0B00001110,0B00000000,0B00010101,0B00011101,
//    0B01100111,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
//    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000
//};

//Rotated display
const static char charTable []   = {
    0B01111110,0B00000110,0B01101101,0B01001111,0B00010111,0B01011011,0B01111011,0B00001110,
    0B01111111,0B01011111,0B00111111,0B01110011,0B01100001,0B01100111,0B01111001,0B00111001,
    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
    0B00000000,0B00000000,0B00000000,0B00000000,0B10000000,0B00000001,0B10000000,0B00000000,
    0B01111110,0B00110000,0B01101101,0B01111001,0B00110011,0B01011011,0B01011111,0B01110000,
    0B01111111,0B01111011,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
    0B00000000,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,0B00000000,
    0B00110111,0B00000000,0B00000000,0B00000000,0B00001110,0B00000000,0B00000000,0B00000000,
    0B01100111,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00001000,
    0B00000000,0B01110111,0B00011111,0B00001101,0B00111101,0B01001111,0B01000111,0B00000000,
    0B00110111,0B00000000,0B00000000,0B00000000,0B00001110,0B00000000,0B00010101,0B00011101,
    0B01100111,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,
    0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000,0B00000000
};

const unsigned char CH[] =
{
  3, 8, 0B00000000, 0B00000000, 0B00000000, 0B00000000, 0B00000000, // space
  1, 8, 0B01011111, 0B00000000, 0B00000000, 0B00000000, 0B00000000, // !
  3, 8, 0B00000011, 0B00000000, 0B00000011, 0B00000000, 0B00000000, // "
  5, 8, 0B00010100, 0B00111110, 0B00010100, 0B00111110, 0B00010100, // #
  4, 8, 0B00100100, 0B01101010, 0B00101011, 0B00010010, 0B00000000, // $
  5, 8, 0B01100011, 0B00010011, 0B00001000, 0B01100100, 0B01100011, // %
  5, 8, 0B00110110, 0B01001001, 0B01010110, 0B00100000, 0B01010000, // &
  1, 8, 0B00000011, 0B00000000, 0B00000000, 0B00000000, 0B00000000, // '
  3, 8, 0B00011100, 0B00100010, 0B01000001, 0B00000000, 0B00000000, // (
  3, 8, 0B01000001, 0B00100010, 0B00011100, 0B00000000, 0B00000000, // )
  5, 8, 0B00101000, 0B00011000, 0B00001110, 0B00011000, 0B00101000, // *
  5, 8, 0B00001000, 0B00001000, 0B00111110, 0B00001000, 0B00001000, // +
  2, 8, 0B10110000, 0B01110000, 0B00000000, 0B00000000, 0B00000000, // ,
  4, 8, 0B00001000, 0B00001000, 0B00001000, 0B00001000, 0B00000000, // -
  2, 8, 0B01100000, 0B01100000, 0B00000000, 0B00000000, 0B00000000, // .
  4, 8, 0B01100000, 0B00011000, 0B00000110, 0B00000001, 0B00000000, // /
  4, 8, 0B00111110, 0B01000001, 0B01000001, 0B00111110, 0B00000000, // 0
  3, 8, 0B01000010, 0B01111111, 0B01000000, 0B00000000, 0B00000000, // 1
  4, 8, 0B01100010, 0B01010001, 0B01001001, 0B01000110, 0B00000000, // 2
  4, 8, 0B00100010, 0B01000001, 0B01001001, 0B00110110, 0B00000000, // 3
  4, 8, 0B00011000, 0B00010100, 0B00010010, 0B01111111, 0B00000000, // 4
  4, 8, 0B00100111, 0B01000101, 0B01000101, 0B00111001, 0B00000000, // 5
  4, 8, 0B00111110, 0B01001001, 0B01001001, 0B00110000, 0B00000000, // 6
  4, 8, 0B01100001, 0B00010001, 0B00001001, 0B00000111, 0B00000000, // 7
  4, 8, 0B00110110, 0B01001001, 0B01001001, 0B00110110, 0B00000000, // 8
  4, 8, 0B00000110, 0B01001001, 0B01001001, 0B00111110, 0B00000000, // 9
  2, 8, 0B01010000, 0B00000000, 0B00000000, 0B00000000, 0B00000000, // :
  2, 8, 0B10000000, 0B01010000, 0B00000000, 0B00000000, 0B00000000, // ;
  3, 8, 0B00010000, 0B00101000, 0B01000100, 0B00000000, 0B00000000, // <
  3, 8, 0B00010100, 0B00010100, 0B00010100, 0B00000000, 0B00000000, // =
  3, 8, 0B01000100, 0B00101000, 0B00010000, 0B00000000, 0B00000000, // >
  4, 8, 0B00000010, 0B01011001, 0B00001001, 0B00000110, 0B00000000, // ?
  5, 8, 0B00111110, 0B01001001, 0B01010101, 0B01011101, 0B00001110, // @
  4, 8, 0B01111110, 0B00010001, 0B00010001, 0B01111110, 0B00000000, // A
  4, 8, 0B01111111, 0B01001001, 0B01001001, 0B00110110, 0B00000000, // B
  4, 8, 0B00111110, 0B01000001, 0B01000001, 0B00100010, 0B00000000, // C
  4, 8, 0B01111111, 0B01000001, 0B01000001, 0B00111110, 0B00000000, // D
  4, 8, 0B01111111, 0B01001001, 0B01001001, 0B01000001, 0B00000000, // E
  4, 8, 0B01111111, 0B00001001, 0B00001001, 0B00000001, 0B00000000, // F
  4, 8, 0B00111110, 0B01000001, 0B01001001, 0B01111010, 0B00000000, // G
  4, 8, 0B01111111, 0B00001000, 0B00001000, 0B01111111, 0B00000000, // H
  3, 8, 0B01000001, 0B01111111, 0B01000001, 0B00000000, 0B00000000, // I
  4, 8, 0B00110000, 0B01000000, 0B01000001, 0B00111111, 0B00000000, // J
  4, 8, 0B01111111, 0B00001000, 0B00010100, 0B01100011, 0B00000000, // K
  4, 8, 0B01111111, 0B01000000, 0B01000000, 0B01000000, 0B00000000, // L
  5, 8, 0B01111111, 0B00000010, 0B00001100, 0B00000010, 0B01111111, // M
  5, 8, 0B01111111, 0B00000100, 0B00001000, 0B00010000, 0B01111111, // N
  4, 8, 0B00111110, 0B01000001, 0B01000001, 0B00111110, 0B00000000, // O
  4, 8, 0B01111111, 0B00001001, 0B00001001, 0B00000110, 0B00000000, // P
  4, 8, 0B00111110, 0B01000001, 0B01000001, 0B10111110, 0B00000000, // Q
  4, 8, 0B01111111, 0B00001001, 0B00001001, 0B01110110, 0B00000000, // R
  4, 8, 0B01000110, 0B01001001, 0B01001001, 0B00110010, 0B00000000, // S
  5, 8, 0B00000001, 0B00000001, 0B01111111, 0B00000001, 0B00000001, // T
  4, 8, 0B00111111, 0B01000000, 0B01000000, 0B00111111, 0B00000000, // U
  5, 8, 0B00001111, 0B00110000, 0B01000000, 0B00110000, 0B00001111, // V
  5, 8, 0B00111111, 0B01000000, 0B00111000, 0B01000000, 0B00111111, // W
  5, 8, 0B01100011, 0B00010100, 0B00001000, 0B00010100, 0B01100011, // X
  5, 8, 0B00000111, 0B00001000, 0B01110000, 0B00001000, 0B00000111, // Y
  4, 8, 0B01100001, 0B01010001, 0B01001001, 0B01000111, 0B00000000, // Z
  2, 8, 0B01111111, 0B01000001, 0B00000000, 0B00000000, 0B00000000, // [
  4, 8, 0B00000001, 0B00000110, 0B00011000, 0B01100000, 0B00000000, // \ backslash
  2, 8, 0B01000001, 0B01111111, 0B00000000, 0B00000000, 0B00000000, // ]
  3, 8, 0B00000010, 0B00000001, 0B00000010, 0B00000000, 0B00000000, // hat
  4, 8, 0B01000000, 0B01000000, 0B01000000, 0B01000000, 0B00000000, // _
  2, 8, 0B00000001, 0B00000010, 0B00000000, 0B00000000, 0B00000000, // `
  4, 8, 0B00100000, 0B01010100, 0B01010100, 0B01111000, 0B00000000, // a
  4, 8, 0B01111111, 0B01000100, 0B01000100, 0B00111000, 0B00000000, // b
  4, 8, 0B00111000, 0B01000100, 0B01000100, 0B00101000, 0B00000000, // c
  4, 8, 0B00111000, 0B01000100, 0B01000100, 0B01111111, 0B00000000, // d
  4, 8, 0B00111000, 0B01010100, 0B01010100, 0B00011000, 0B00000000, // e
  3, 8, 0B00000100, 0B01111110, 0B00000101, 0B00000000, 0B00000000, // f
  4, 8, 0B10011000, 0B10100100, 0B10100100, 0B01111000, 0B00000000, // g
  4, 8, 0B01111111, 0B00000100, 0B00000100, 0B01111000, 0B00000000, // h
  3, 8, 0B01000100, 0B01111101, 0B01000000, 0B00000000, 0B00000000, // i
  4, 8, 0B01000000, 0B10000000, 0B10000100, 0B01111101, 0B00000000, // j
  4, 8, 0B01111111, 0B00010000, 0B00101000, 0B01000100, 0B00000000, // k
  3, 8, 0B01000001, 0B01111111, 0B01000000, 0B00000000, 0B00000000, // l
  5, 8, 0B01111100, 0B00000100, 0B01111100, 0B00000100, 0B01111000, // m
  4, 8, 0B01111100, 0B00000100, 0B00000100, 0B01111000, 0B00000000, // n
  4, 8, 0B00111000, 0B01000100, 0B01000100, 0B00111000, 0B00000000, // o
  4, 8, 0B11111100, 0B00100100, 0B00100100, 0B00011000, 0B00000000, // p
  4, 8, 0B00011000, 0B00100100, 0B00100100, 0B11111100, 0B00000000, // q
  4, 8, 0B01111100, 0B00001000, 0B00000100, 0B00000100, 0B00000000, // r
  4, 8, 0B01001000, 0B01010100, 0B01010100, 0B00100100, 0B00000000, // s
  3, 8, 0B00000100, 0B00111111, 0B01000100, 0B00000000, 0B00000000, // t
  4, 8, 0B00111100, 0B01000000, 0B01000000, 0B01111100, 0B00000000, // u
  5, 8, 0B00011100, 0B00100000, 0B01000000, 0B00100000, 0B00011100, // v
  5, 8, 0B00111100, 0B01000000, 0B00111100, 0B01000000, 0B00111100, // w
  5, 8, 0B01000100, 0B00101000, 0B00010000, 0B00101000, 0B01000100, // x
  4, 8, 0B10011100, 0B10100000, 0B10100000, 0B01111100, 0B00000000, // y
  3, 8, 0B01100100, 0B01010100, 0B01001100, 0B00000000, 0B00000000, // z
  3, 8, 0B00001000, 0B00110110, 0B01000001, 0B00000000, 0B00000000, // {
  1, 8, 0B01111111, 0B00000000, 0B00000000, 0B00000000, 0B00000000, // |
  3, 8, 0B01000001, 0B00110110, 0B00001000, 0B00000000, 0B00000000, // }
  4, 8, 0B00001000, 0B00000100, 0B00001000, 0B00000100, 0B00000000, // ~
};


/* we always wait a bit between updates of the display */
unsigned long delaytime=50;
unsigned char buffer[40];
// Display=the extracted characters with scrolling
void LedControl_printCharWithShift(char c, int shift_speed) {
  if (c < 32) return;
  c -= 32;
  memcpy(buffer, CH + 7 * c, 7);
  LedControl_writeSprite(0, 0, buffer);
//  LedControl_writeSprite(32, 0, buffer);
//  LedControl_setColumn(32 + buffer[0], 0);
//  for (int i = 0; i < buffer[0] + 1; i++)
//  {
//	  HAL_Delay(shift_speed);
//      LedControl_shiftLeft(0, 0);
//  }
}

void LedControl_SetCursor(int posX, int posY) {
	led_control.cursor_pos = posX;
	led_control.vertical_pos = posY;
}

void LedControl_printChar(char c) {
  if (c < 32) return;
  c -= 32;
  memcpy(buffer, CH + 7 * c, 7);
  if((led_control.cursor_pos < 40) && (led_control.cursor_pos +  buffer[0]) > 40)
  {
	  led_control.cursor_pos = 40; // Avoid splitting characters - move to next line
  }
  LedControl_writeSprite(led_control.cursor_pos, led_control.vertical_pos, buffer);
  led_control.cursor_pos += buffer[0]+1; // Advance carret to next position

}

void LedControl_printBigChar(char c) {
	uint16_t offset;
  if (c < 32) return;
  c -= 32;
  buffer[0] = consolas_16ptDescriptors[(unsigned char)c].width;
  buffer[1] = 16; //Height
  offset = consolas_16ptDescriptors[(unsigned char)c].offset;
  memcpy(&buffer[2],&consolas_16ptBitmaps[offset], (2 + buffer[0]/8)*16);

  LedControl_writeBigSprite(led_control.cursor_pos, led_control.vertical_pos, buffer);
  led_control.cursor_pos += buffer[0]+1; // Advance carret to next position

}


// Extract the characters from the text string
void LedControl_printStringWithShift(char* s, int shift_speed) {
  while (*s != 0) {
	  LedControl_printCharWithShift(*s, shift_speed);
      s++;
  }
}

void LedControl_printString(char* s) {
  while (*s != 0) {
	  LedControl_printChar(*s);
      s++;
  }
}

void LedControl_printBigString(char* s) {
  while (*s != 0) {
	  LedControl_printBigChar(*s);
      s++;
  }
}

void LedControl_init(uint16_t dataPin, GPIO_TypeDef* data_port,
		uint16_t clkPin,  GPIO_TypeDef* clk_port, uint16_t csPin,  GPIO_TypeDef* cs_port, int numDevices) {
	led_control.SPI_MOSI_pin=dataPin;
	led_control.SPI_MOSI_port=data_port;
	led_control.SPI_CLK_pin=clkPin;
	led_control.SPI_CLK_port=clk_port;
	led_control.SPI_CS_pin=csPin;
	led_control.SPI_CS_port=cs_port;
    if(numDevices<=0 || numDevices>16 )
        numDevices=16;
    led_control.maxDevices=numDevices;
    pinMode(led_control.SPI_MOSI_port,led_control.SPI_MOSI_pin,GPIO_MODE_OUTPUT_PP);
    pinMode(led_control.SPI_CLK_port, led_control.SPI_CLK_pin, GPIO_MODE_OUTPUT_PP);
    pinMode(led_control.SPI_CS_port, led_control.SPI_CS_pin, GPIO_MODE_OUTPUT_PP);
    digitalWrite(led_control.SPI_CS_port, led_control.SPI_CS_pin,HIGH);

    memset(led_control.buffer, 0, sizeof(led_control.buffer));
    for(int i=0;i<led_control.maxDevices;i++) {
    	LedControl_spiTransfer(i, OP_DISPLAYTEST, 0);
        //scanlimit is set to max on startup
    	LedControl_setScanLimit(i, 7);
        //decode is done in source
        LedControl_spiTransfer(i, OP_DECODEMODE, 0);
        LedControl_clearDisplay(i);
        //we go into shutdown-mode on startup
        LedControl_shutdown(i, 1);
    }
}

int LedControl_getDeviceCount(){
    return led_control.maxDevices;
}

void LedControl_shutdown(int addr, unsigned char status){
    if(addr<0 || addr>=led_control.maxDevices)
        return;
    if(status)
    	LedControl_spiTransfer(addr, OP_SHUTDOWN,0);
    else
    	LedControl_spiTransfer(addr, OP_SHUTDOWN,1);
}

void LedControl_setScanLimit(int addr, int limit) {
    if(addr<0 || addr>=led_control.maxDevices)
        return;
    if(limit>=0 && limit<8)
    	LedControl_spiTransfer(addr, OP_SCANLIMIT,limit);
}

void LedControl_setIntensity(int addr, int intensity) {
    if(addr<0 || addr>=led_control.maxDevices)
        return;
    if(intensity>=0 && intensity<16)	
    	LedControl_spiTransfer(addr, OP_INTENSITY,intensity);
}

void LedControl_clearDisplay(int addr) {
    int offset;

    if(addr<0 || addr>=led_control.maxDevices)
        return;
    offset=addr*8;
    for(int i=0;i<8;i++) {
    	led_control.buffer[offset+i]=0;
    	LedControl_spiTransfer(addr, i+1,led_control.buffer[offset+i]);
    }
}

void LedControl_setLed(int addr, int row, int col, unsigned char state) {
    int offset;
    unsigned char val=0x00;

    if(addr<0 || addr>=led_control.maxDevices)
        return;
    if(row<0 || row>7 || col<0 || col>7)
        return;
    offset=addr*8;
    val=0B10000000 >> col;
    if(state)
    	led_control.buffer[offset+row]=led_control.buffer[offset+row]|val;
    else {
        val=~val;
        led_control.buffer[offset+row]=led_control.buffer[offset+row]&val;
    }
    LedControl_spiTransfer(addr, row+1,led_control.buffer[offset+row]);
}

void LedControl_setRow(int addr, int row, unsigned char value){
    int offset;
    if(addr<0 || addr>=led_control.maxDevices)
        return;
    if(row<0 || row>7)
        return;
    offset=addr*8;
    led_control.buffer[offset+row]=value;
    LedControl_spiTransfer(addr, row+1,led_control.buffer[offset+row]);
}

void LedControl_setColumn( int col, unsigned char value){
    char val;
    int addr = col /8;
    if(addr<0 || addr>=led_control.maxDevices)
        return;
    if(col<0 || col>7) 
        return;
    for(int row=0;row<8;row++) {
        val=value >> (7-row);
        val=val & 0x01;
        LedControl_setLed(addr,row,col,val);
    }
}

void LedControl_setDigit(int addr, int digit, unsigned char value, unsigned char dp) {
    int offset;
    unsigned char v;

    if(addr<0 || addr>=led_control.maxDevices)
        return;
    if(digit<0 || digit>7 || value>15)
        return;
    offset=addr*8;
    v=charTable[value];
    if(dp)
        v|=0B10000000;
    led_control.buffer[offset+digit]=v;
    LedControl_spiTransfer(addr, digit+1, v);
}

void LedControl_setChar(int addr, int digit, unsigned char value, unsigned char dp) {
    int offset;
    unsigned char index,v;

    if(addr<0 || addr>=led_control.maxDevices)
        return;
    if(digit<0 || digit>7)
        return;
    offset=addr*8;
    index=(char)value;
    if(index >127) {
        //no defined beyond index 127, so we use the space char
        index=32;
    }
    v=charTable[index];
    if(dp)
        v|=0B10000000;
    led_control.buffer[offset+digit]=v;
    LedControl_spiTransfer(addr, digit+1, v);
}

void LedControl_spiTransfer(int addr, char opcode, char data) {
    //Create an array with the data to shift out
    int offset=addr*2;
    int maxbytes=32; //led_control->maxDevices*2;

    memset(led_control.spidata, 0, sizeof(led_control.spidata));
    //put our device data into the array
    led_control.spidata[offset+1]=opcode;
    led_control.spidata[offset]=data;
    //enable the line 
    digitalWrite(led_control.SPI_CS_port, led_control.SPI_CS_pin,LOW);
    //Now shift out the data 
    for(int i=maxbytes;i>0;i--)
    //latch the data onto the display
        shiftOut(led_control.SPI_MOSI_port, led_control.SPI_MOSI_pin, led_control.SPI_CLK_port, led_control.SPI_CLK_pin, led_control.spidata[i-1]);
    digitalWrite(led_control.SPI_CS_port, led_control.SPI_CS_pin, HIGH);
}    


void LedControl_bitWrite(unsigned char col, unsigned char row, unsigned char value)
{

	value &= 1;
	if(value)
	{
		value = 0x80 >> (col%8);
	}
	row = row % 8;
	//relocate devices in a row
	col = 8*DeviceLUT[col/8];

	led_control.buffer[row + col] &= ~value;
	led_control.buffer[row + col] |= value;
}

void LedControl_bitBigWrite(unsigned char col, unsigned char row, unsigned char value)
{

	value &= 1;
	if(value)
	{
		value = 0x80 >> (col%8);
	}
	row = row % 16;
	//relocate devices in a row
	col = (col%40) + (row/8 )*40;
	col = 8*DeviceLUT[col/8];
	row = row % 8;

	led_control.buffer[row + col] &= ~value;
	led_control.buffer[row + col] |= value;
}

unsigned char LedControl_bitRead(unsigned char value, unsigned char bit)
{
	return !!(value & (1<<bit));
}

void LedControl_writeSprite_old(int x, int y, unsigned char* sprite)
{
	int w = sprite[0];
	int h = sprite[1];

	if (h == 8 && y == 0)
		for (int i=0; i<w; i++)
		{
			int c = x + i;
			if (c>=0 && c<80)
				LedControl_setColumn(c, sprite[i+2]);
		}
	else
		for (int i=0; i<w; i++)
			for (int j=0; j<h; j++)
			{
				int c = x + i;
				int r = y + j;
				if (c>=0 && c<80 && r>=0 && r<8)
					LedControl_bitWrite(c, r, LedControl_bitRead(sprite[i+2], j));
			}

}

void LedControl_writeSprite(int x, int y, unsigned char* sprite)
{
	int width = sprite[0];
	int height = sprite[1];


	for (int i=0; i<width; i++)
	{
		for (int j=0; j<height; j++)
		{
			int col = x + i;
			int row = y + j;
			if (col>=0 && col<80 && row>=0 && row<8)
				LedControl_bitWrite(col, row, LedControl_bitRead(sprite[i + 2], j));
		}
	}
}

void LedControl_writeBigSprite(int x, int y, unsigned char* sprite)
{
	int width = sprite[0];
	int height = sprite[1];
	unsigned char bit, val;

	for (int i=0; i<width; i++)
	{
		for (int j=0; j<height; j++)
		{
			int col = x + i;
			int row = y + j;
			if (col>=0 && col<40 && row>=0 && row<16)
			{
				if(width < 9)
				{
				    val = sprite[j +  (i/8) + 2];
				}
				else
				{
					val = sprite[j*2 +  (i/8) + 2];
				}
//				if(i < 8)
//				{
//					bit = LedControl_bitRead(0x03, (i%8));
//				}
//				else
//				{
//					bit = 0;
//				}
				bit =  LedControl_bitRead(val, 7 - (i & 0x07));
//				if(col == 7) bit = 1;
//				else bit = 0;
//				LedControl_bitBigWrite(col, row, 1);sprite[((1 + width/8) - i%8) + j* (1 + width/8)]
//				LedControl_bitBigWrite(col, row, LedControl_bitRead(sprite[j*(1 + width/8) + 2 + width/8], (i%8)));
//				LedControl_bitBigWrite(col, row, LedControl_bitRead(sprite[(width - i)/8 + j*(1 + width/8)], (i%8)));
//				LedControl_bitBigWrite(col, row, LedControl_bitRead(sprite[j*2 + (1 - i%2)], (i%8)));
				LedControl_bitBigWrite(col, row, bit);
			}
		}
	}
}

void LedControl_reload()
{
	for (int i=0; i<8; i++)
	{
//		int col = i;
		digitalWrite(led_control.SPI_CS_port, led_control.SPI_CS_pin,LOW);
		for(int j = (led_control.maxDevices-1); j >= 0; j--)
		{
		    shiftOut(led_control.SPI_MOSI_port, led_control.SPI_MOSI_pin, led_control.SPI_CLK_port, led_control.SPI_CLK_pin, i+1);
		    shiftOut(led_control.SPI_MOSI_port, led_control.SPI_MOSI_pin, led_control.SPI_CLK_port, led_control.SPI_CLK_pin, led_control.buffer[j*8+i]);
//			col += 8;
		}
		//latch the data onto the display
		digitalWrite(led_control.SPI_CS_port, led_control.SPI_CS_pin, HIGH);
	}
}

void LedControl_shiftUp(char rotate)
{
	char old = buffer[0];
	int i;
	for (i=0; i<80; i++)
		buffer[i] = buffer[i+1];
	if (rotate) buffer[led_control.maxDevices*8-1] = old;


	LedControl_reload();
}

void LedControl_shiftRight(char rotate, char fill_zero)
{
	int last = led_control.maxDevices*8-1;
	char old = buffer[last];
	int i;
	for (i=79; i>0; i--)
		buffer[i] = led_control.buffer[i-1];
	if (rotate) buffer[0] = old;
	else if (fill_zero) led_control.buffer[0] = 0;

	LedControl_reload();
}

void LedControl_shiftLeft(char rotate, char fill_zero)
{
	for (int i=0; i<led_control.maxDevices; i++)
	{

		char b = buffer[i] & 1;
		buffer[i] >>= 1;
		if (rotate) LedControl_bitWrite(i, 7, b);
	}
//	LedControl_reload();
	if (fill_zero) led_control.buffer[led_control.maxDevices*8-1] = 0;
}

void LedControl_shiftDown(char rotate)
{
	for (int i=0; i<led_control.maxDevices*8; i++)
	{
		char b = buffer[i] & 128;
		buffer[i] <<= 1;
		if (rotate) LedControl_bitWrite(i, 0, b);
	}
	LedControl_reload();
}


