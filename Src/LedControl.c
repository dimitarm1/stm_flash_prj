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
#define HIGH 1
#define LOW 0

void pinMode(GPIO_TypeDef* port, uint16_t pin_num, char pin_mode)
{
	  GPIO_InitTypeDef GPIO_InitStruct;
	  GPIO_InitStruct.Pin = pin_num;
	  GPIO_InitStruct.Mode = pin_mode;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	  HAL_GPIO_Init(port, &GPIO_InitStruct);
}

void digitalWrite(GPIO_TypeDef* port, uint16_t pin_number, unsigned char  value)
{
	 HAL_GPIO_WritePin(port, 1<<pin_number, (GPIO_PinState) value);
}
void shiftOut(GPIO_TypeDef* data_port, uint16_t data_pin, GPIO_TypeDef* clk_port, uint16_t clk_pin, unsigned char data)
{
	unsigned char i,val;
	for(i = 0; i< 8; i++)
	{
		HAL_GPIO_WritePin(clk_port, 1<<clk_pin, (GPIO_PinState) 0);
		val = ((data & 0x80) != 0);
		data = data << 1;
		HAL_GPIO_WritePin(data_port, 1<<data_pin, (GPIO_PinState) val);
		HAL_GPIO_WritePin(clk_port, 1<<clk_pin, (GPIO_PinState) 1);
	}
}

void LedControl_init(LedControl* led_control, uint16_t dataPin, GPIO_TypeDef* data_port,
		uint16_t clkPin,  GPIO_TypeDef* clk_port, uint16_t csPin,  GPIO_TypeDef* cs_port, int numDevices) {
	led_control->SPI_MOSI_pin=dataPin;
	led_control->SPI_MOSI_port=data_port;
	led_control->SPI_CLK_pin=clkPin;
	led_control->SPI_CLK_port=clk_port;
	led_control->SPI_CS_pin=csPin;
	led_control->SPI_CS_port=cs_port;
    if(numDevices<=0 || numDevices>8 )
        numDevices=8;
    led_control->maxDevices=numDevices;
    pinMode(led_control->SPI_MOSI_port,led_control->SPI_MOSI_pin,GPIO_MODE_OUTPUT_PP);
    pinMode(led_control->SPI_CLK_port, led_control->SPI_CLK_pin, GPIO_MODE_OUTPUT_PP);
    pinMode(led_control->SPI_CS_port, led_control->SPI_CS_pin, GPIO_MODE_OUTPUT_PP);
    digitalWrite(led_control->SPI_CS_port, led_control->SPI_CS_pin,HIGH);

    for(int i=0;i<64;i++) 
    	led_control->status[i]=0x00;
    for(int i=0;i<led_control->maxDevices;i++) {
    	LedControl_spiTransfer(led_control, i, OP_DISPLAYTEST, 0);
        //scanlimit is set to max on startup
    	LedControl_setScanLimit(led_control, i, 7);
        //decode is done in source
        LedControl_spiTransfer(led_control, i, OP_DECODEMODE, 0);
        LedControl_clearDisplay(led_control, i);
        //we go into shutdown-mode on startup
        LedControl_shutdown(led_control, i, 1);
    }
}

int LedControl_getDeviceCount(LedControl* led_control){
    return led_control->maxDevices;
}

void LedControl_shutdown(LedControl* led_control, int addr, unsigned char status){
    if(addr<0 || addr>=led_control->maxDevices)
        return;
    if(status)
    	LedControl_spiTransfer(led_control, addr, OP_SHUTDOWN,0);
    else
    	LedControl_spiTransfer(led_control, addr, OP_SHUTDOWN,1);
}

void LedControl_setScanLimit(LedControl* led_control, int addr, int limit) {
    if(addr<0 || addr>=led_control->maxDevices)
        return;
    if(limit>=0 && limit<8)
    	LedControl_spiTransfer(led_control, addr, OP_SCANLIMIT,limit);
}

void LedControl_setIntensity(LedControl* led_control, int addr, int intensity) {
    if(addr<0 || addr>=led_control->maxDevices)
        return;
    if(intensity>=0 && intensity<16)	
    	LedControl_spiTransfer(led_control, addr, OP_INTENSITY,intensity);
}

void LedControl_clearDisplay(LedControl* led_control, int addr) {
    int offset;

    if(addr<0 || addr>=led_control->maxDevices)
        return;
    offset=addr*8;
    for(int i=0;i<8;i++) {
    	led_control->status[offset+i]=0;
    	LedControl_spiTransfer(led_control, addr, i+1,led_control->status[offset+i]);
    }
}

void LedControl_setLed(LedControl* led_control, int addr, int row, int col, unsigned char state) {
    int offset;
    unsigned char val=0x00;

    if(addr<0 || addr>=led_control->maxDevices)
        return;
    if(row<0 || row>7 || col<0 || col>7)
        return;
    offset=addr*8;
    val=0B10000000 >> col;
    if(state)
    	led_control->status[offset+row]=led_control->status[offset+row]|val;
    else {
        val=~val;
        led_control->status[offset+row]=led_control->status[offset+row]&val;
    }
    LedControl_spiTransfer(led_control, addr, row+1,led_control->status[offset+row]);
}

void LedControl_setRow(LedControl* led_control, int addr, int row, unsigned char value){
    int offset;
    if(addr<0 || addr>=led_control->maxDevices)
        return;
    if(row<0 || row>7)
        return;
    offset=addr*8;
    led_control->status[offset+row]=value;
    LedControl_spiTransfer(led_control, addr, row+1,led_control->status[offset+row]);
}

void LedControl_setColumn(LedControl* led_control, int addr, int col, unsigned char value){
    char val;

    if(addr<0 || addr>=led_control->maxDevices)
        return;
    if(col<0 || col>7) 
        return;
    for(int row=0;row<8;row++) {
        val=value >> (7-row);
        val=val & 0x01;
        LedControl_setLed(led_control, addr,row,col,val);
    }
}

void LedControl_setDigit(LedControl* led_control, int addr, int digit, unsigned char value, unsigned char dp) {
    int offset;
    unsigned char v;

    if(addr<0 || addr>=led_control->maxDevices)
        return;
    if(digit<0 || digit>7 || value>15)
        return;
    offset=addr*8;
    v=charTable[value];
    if(dp)
        v|=0B10000000;
    led_control->status[offset+digit]=v;
    LedControl_spiTransfer(led_control, addr, digit+1, v);
}

void LedControl_setChar(LedControl* led_control, int addr, int digit, unsigned char value, unsigned char dp) {
    int offset;
    unsigned char index,v;

    if(addr<0 || addr>=led_control->maxDevices)
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
    led_control->status[offset+digit]=v;
    LedControl_spiTransfer(led_control, addr, digit+1, v);
}

void LedControl_spiTransfer(LedControl* led_control, int addr, char opcode, char data) {
    //Create an array with the data to shift out
    int offset=addr*2;
    int maxbytes=led_control->maxDevices*2;

    for(int i=0;i<maxbytes;i++)
    	led_control->spidata[i]=(unsigned char)0;
    //put our device data into the array
    led_control->spidata[offset+1]=opcode;
    led_control->spidata[offset]=data;
    //enable the line 
    digitalWrite(led_control->SPI_CS_port, led_control->SPI_CS_pin,LOW);
    //Now shift out the data 
    for(int i=maxbytes;i>0;i--)
        shiftOut(led_control->SPI_MOSI_port, led_control->SPI_MOSI_pin, led_control->SPI_CLK_port, led_control->SPI_CLK_pin, led_control->spidata[i-1]);
    //latch the data onto the display
    digitalWrite(led_control->SPI_CS_port, led_control->SPI_CS_pin, HIGH);
}    


