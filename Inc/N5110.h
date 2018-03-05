/**
@file N5110.h

@brief Header file containing member functions and variables

*/

#ifndef N5110_H
#define N5110_H

// Command Bytes - taken from Chris Yan's library
// More information can be found in the display datasheet
// H = 0 - Basic instructions
#define CMD_DC_CLEAR_DISPLAY   0x08
#define CMD_DC_NORMAL_MODE     0x0C
#define CMD_DC_FILL_DISPLAY    0x09
#define CMD_DC_INVERT_VIDEO    0x0D
#define CMD_FS_HORIZONTAL_MODE 0x00
#define CMD_FS_VERTICAL_MODE   0x02
#define CMD_FS_BASIC_MODE      0x00
#define CMD_FS_EXTENDED_MODE   0x01
#define CMD_FS_ACTIVE_MODE     0x00
#define CMD_FS_POWER_DOWN_MODE 0x04
// H = 1 - Extended instructions
#define CMD_TC_TEMP_0          0x04
#define CMD_TC_TEMP_1          0x05
#define CMD_TC_TEMP_2          0x06
#define CMD_TC_TEMP_3          0x07
#define CMD_BI_MUX_24          0x15
#define CMD_BI_MUX_48          0x13
#define CMD_BI_MUX_100         0x10
#define CMD_VOP_6V06           0xB2
#define CMD_VOP_7V38           0xC8

// number of pixels on display
#define WIDTH 84
#define HEIGHT 48
#define BANKS 6


/**
@brief Library for interfacing with Nokia 5110 LCD display (https://www.sparkfun.com/products/10168) using the hardware SPI on the mbed.
@brief The display is powered from a GPIO pin meaning it can be controlled via software.  The LED backlight is also software-controllable (via PWM pin).
@brief Can print characters and strings to the display using the included 5x7 font.
@brief The library also implements a screen buffer so that individual pixels on the display (84 x 48) can be set, cleared and read.
@brief The library can print primitive shapes (lines, circles, rectangles)
@brief Acknowledgements to Chris Yan's Nokia_5110 Library.

@brief Revision 1.2

@author Craig A. Evans
@date   17th March 2015

@brief converted by dmarinov to c code
@date 5 Nov 2016

 *
 * Example:
 * @code


#include "N5110.h"

//    VCC,SCE,RST,D/C,MOSI,SCLK,LED
N5110 lcd(p7,p8,p9,p10,p11,p13,p21);
// Can also power (VCC) directly from VOUT (3.3 V) -
// Can give better performance due to current limitation from GPIO pin

int main()
{
    // first need to initialise display
    N5110_init();

    while(1) {

        // these are default settings so not strictly needed
        N5110_normalMode();      // normal colour mode
        N5110_setBrightness(0.5); // put LED backlight on 50%

        // can directly print strings at specified co-ordinates
        N5110_printString("Hello, World!",0,0);

        char buffer[14];  // each character is 6 pixels wide, screen is 84 pixels (84/6 = 14)
        // so can display a string of a maximum 14 characters in length
        // or create formatted strings - ensure they aren't more than 14 characters long
        int temperature = 27;
        int length = sprintf(buffer,"T = %2d C",temperature); // print formatted data to buffer
        // it is important the format specifier ensures the length will fit in the buffer
        if (length <= 14)  // if string will fit on display
            N5110_printString(buffer,0,1);           // display on screen

        float pressure = 1012.3;  // same idea with floats
        length = sprintf(buffer,"P = %.2f mb",pressure);
        if (length <= 14)
            N5110_printString(buffer,0,2);

        // can also print individual characters at specified place
        N5110_printChar('X',5,3);

        // draw a line across the display at y = 40 pixels (origin top-left)
        for (int i = 0; i < WIDTH; i++) {
            N5110_setPixel(i,40);
        }
        // need to refresh display after setting pixels
        N5110_refresh();

        // can also check status of pixels using getPixel(x,y)

        wait(5.0);
        N5110_clear();            // clear display
        N5110_inverseMode();      // invert colours
        N5110_setBrightness(1.0); // put LED backlight on full

        float array[84];

        for (int i = 0; i < 84; i++) {
            array[i] = 0.5 + 0.5*sin(i*2*3.14/84);
        }

        // can also plot graphs - 84 elements only
        // values must be in range 0.0 - 1.0
        N5110_plotArray(array);
        wait(5.0);
        N5110_clear();
        N5110_normalMode();      // normal colour mode back
        N5110_setBrightness(0.5); // put LED backlight on 50%

        // example of drawing lines
        for (int x = 0; x < WIDTH ; x+=10) {
            // x0,y0,x1,y1,type 0-white,1-black,2-dotted
            N5110_drawLine(0,0,x,HEIGHT,2);
        }
        N5110_refresh();   // need to refresh screen after drawing lines

        wait(5.0);
        N5110_clear();

        // example of how to draw circles
        N5110_drawCircle(WIDTH/2,HEIGHT/2,20,1);  // x,y,radius,black fill
        N5110_drawCircle(WIDTH/2,HEIGHT/2,10,2);  // x,y,radius,white fill
        N5110_drawCircle(WIDTH/2,HEIGHT/2,30,0);  // x,y,radius,transparent with outline
        N5110_refresh();   // need to refresh screen after drawing circles

        wait(5.0);
        N5110_clear();

        // example of how to draw rectangles
        //          origin x,y,width,height,type
        N5110_drawRect(10,10,50,30,1);  // filled black rectangle
        N5110_drawRect(15,15,20,10,2);  // filled white rectange (no outline)
        N5110_drawRect(2,2,70,40,0);    // transparent, just outline
        N5110_refresh();   // need to refresh screen after drawing rects


        wait(5.0);
        N5110_clear();

    }
}


 * @endcode
 */

extern const unsigned char font5x7[480];
void N5110_init();
void N5110_turnOff();
void N5110_clear();
void N5110_normalMode();
void N5110_inversemode();
void N5110_setBrightness(float brightness);
void N5110_printString(const char * str,int x,int y);
void N5110_printString(const char * str,int x,int y);
void N5110_printChar(char c,int x,int y);
void N5110_setPixel(int x, int y);
void N5110_clearPixel(int x, int y);
int  N5110_getPixel(int x, int y);
void N5110_refresh();
void N5110_randomiseBuffer();
void N5110_plotArray(float array[]);
void N5110_drawCircle(int x0,int y0,int radius,int fill);
void N5110_drawLine(int x0,int y0,int x1,int y1,int type);
void N5110_drawRect(int x0,int y0,int width,int height,int fill);



#endif
