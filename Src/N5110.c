
#include "N5110.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdlib.h>

/**  N5110 connected to the specified pins
*
* @param pwr Pin connected to Vcc on the LCD display (pin 1)
* @param sce Pin connected to chip enable (pin 3)
* @param rst Pin connected to reset (pin 4)
* @param dc  Pin connected to data/command select (pin 5)
* @param mosi Pin connected to data input (MOSI) (pin 6)
* @param sclk Pin connected to serial clock (SCLK) (pin 7)
* @param led Pin connected to LED backlight (must be PWM) (pin 8)
*
*/

extern SPI_HandleTypeDef hspi1;

unsigned char buffer[84][6];

static void setXYAddress(int x, int y);
static void turnOn();
static void reset();
static void clearRAM();
static void clearBuffer();
static void sendCommand(unsigned char command);
static void sendData(unsigned char data);

#define N5110_PWR_PORT GPIOB
#define N5110_PWR_PIN N5110_Power_Pin
#define N5110_RST_PORT GPIOB
#define N5110_RST_PIN N5110_Reset_Pin
#define N5110_DC_PORT GPIOB
#define N5110_DC_PIN N5110_DC_Pin
#define N5110_SCE_PORT GPIOB
#define N5110_SCE_PIN N5110_CE_Pin

/** Initialise display
*
*   Powers up the display and turns on backlight (50% brightness default).
*   Sets the display up in horizontal addressing mode and with normal video mode.
*/
void N5110_init()
{
    turnOn();     // power up
    HAL_Delay(10);  // small delay seems to prevent spurious pixels during mbed reset
    reset();      // reset LCD - must be done within 100 ms

    // function set - extended
    sendCommand(0x20 | CMD_FS_ACTIVE_MODE | CMD_FS_HORIZONTAL_MODE | CMD_FS_EXTENDED_MODE);
    // Don't completely understand these parameters - they seem to work as they are
    // Consult the datasheet if you need to change them
    sendCommand(CMD_VOP_7V38);    // operating voltage - these values are from Chris Yan's Library
    sendCommand(CMD_TC_TEMP_2);   // temperature control
    sendCommand(CMD_BI_MUX_48);   // bias

    // function set - basic
    sendCommand(0x20 | CMD_FS_ACTIVE_MODE | CMD_FS_HORIZONTAL_MODE | CMD_FS_BASIC_MODE);
    N5110_normalMode();  // normal video mode by default
    sendCommand(CMD_DC_NORMAL_MODE);  // black on white

    // RAM is undefined at power-up so clear
    clearRAM();

}

/** Turn on normal video mode (default)
  *  Black on white
  */

void N5110_normalMode()
{
    sendCommand(CMD_DC_NORMAL_MODE);

}

/** Turn on inverse video mode (default)
  *  White on black
  */
void N5110_inverseMode()
{
    sendCommand(CMD_DC_INVERT_VIDEO);
}

// function to power up the LCD and backlight
static void turnOn()
{

	HAL_GPIO_WritePin(N5110_PWR_PORT, N5110_PWR_PIN, GPIO_PIN_RESET);
	HAL_Delay(500);
    // set brightness of LED - 0.0 to 1.0 - default is 50%
    HAL_GPIO_WritePin(N5110_PWR_PORT, N5110_PWR_PIN, GPIO_PIN_SET);
	N5110_setBrightness(0.5);
//    pwr->write(1);  // apply power
}

/** Turn off
*
*   Powers down the display and turns of the backlight.
*   Needs to be reinitialised before being re-used.
*/
void N5110_turnOff()
{
	N5110_setBrightness(0.0);  // turn backlight off
    clearRAM();   // clear RAM to ensure specified current consumption
    // send command to ensure we are in basic mode
    sendCommand(0x20 | CMD_FS_ACTIVE_MODE | CMD_FS_HORIZONTAL_MODE | CMD_FS_BASIC_MODE);
    // clear the display
    sendCommand(CMD_DC_CLEAR_DISPLAY);
    // enter the extended mode and power down
    sendCommand(0x20 | CMD_FS_POWER_DOWN_MODE | CMD_FS_HORIZONTAL_MODE | CMD_FS_EXTENDED_MODE);
    // small delay and then turn off the power pin
    HAL_Delay(10);
    HAL_GPIO_WritePin(N5110_PWR_PORT, N5110_PWR_PIN, GPIO_PIN_RESET);
//    pwr->write(0);

}

/** Set Brightness
  *
  *   Sets brightness of LED backlight.
  *   @param brightness - float in range 0.0 to 1.0
  */
// function to change LED backlight brightness
void N5110_setBrightness(float brightness)
{
    // check whether brightness is within range
    if (brightness < 0.0)
        brightness = 0.0;
    if (brightness > 1.0)
        brightness = 1.0;
    // set PWM duty cycle
//    led->write(brightness);
}


// pulse the active low reset line
static void reset()
{
    // reset the LCD
    HAL_GPIO_WritePin(N5110_RST_PORT, N5110_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(N5110_RST_PORT, N5110_RST_PIN, GPIO_PIN_SET);
}

// send a command to the display
static void sendCommand(unsigned char command)
{
	HAL_GPIO_WritePin(N5110_DC_PORT, N5110_DC_PIN, GPIO_PIN_RESET);  // set DC low for command
    HAL_GPIO_WritePin(N5110_SCE_PORT, N5110_SCE_PIN, GPIO_PIN_RESET); // set CE low to begin frame
    HAL_SPI_Transmit(&hspi1,&command,1,100);  // send command
    HAL_GPIO_WritePin(N5110_DC_PORT, N5110_DC_PIN, GPIO_PIN_SET);  // turn back to data by default
    HAL_GPIO_WritePin(N5110_SCE_PORT, N5110_SCE_PIN, GPIO_PIN_SET); // set CE high to end frame (expected for transmission of single byte)

}

// send data to the display at the current XY address
// dc is set to 1 (i.e. data) after sending a command and so should
// be the default mode.
static void sendData(unsigned char data)
{
	HAL_GPIO_WritePin(N5110_SCE_PORT, N5110_SCE_PIN, GPIO_PIN_RESET); // set CE low to begin frame
    HAL_SPI_Transmit(&hspi1,&data,1,100);
    HAL_GPIO_WritePin(N5110_SCE_PORT, N5110_SCE_PIN, GPIO_PIN_SET);  // set CE high to end frame (expected for transmission of single byte)
}

// this function writes 0 to the 504 bytes to clear the RAM
static void clearRAM()
{
    int i;
    uint8_t data = 0;
    HAL_GPIO_WritePin(N5110_SCE_PORT, N5110_SCE_PIN, GPIO_PIN_RESET);   //set CE low to begin frame
    for(i = 0; i < WIDTH * HEIGHT; i++) { // 48 x 84 bits = 504 bytes
    	HAL_SPI_Transmit(&hspi1,&data,1,100);  // send 0's
    }
    HAL_GPIO_WritePin(N5110_SCE_PORT, N5110_SCE_PIN, GPIO_PIN_SET);  // set CE high to end frame

}

// function to set the XY address in RAM for subsequenct data write
static void setXYAddress(int x, int y)
{
    if (x>=0 && x<WIDTH && y>=0 && y<HEIGHT) {  // check within range
        sendCommand(0x80 | x);  // send addresses to display with relevant mask
        sendCommand(0x40 | y);
    }
}


// These functions are used to set, clear and get the value of pixels in the display
// Pixels are addressed in the range of 0 to 47 (y) and 0 to 83 (x).  The refresh()
// function must be called after set and clear in order to update the display

/** Set a Pixel
*
*   This function sets a pixel in the display. A call to N5110_refresh() must be made
*   to update the display to reflect the change in pixels.
*   @param  x - the x co-ordinate of the pixel (0 to 83)
*   @param  y - the y co-ordinate of the pixel (0 to 47)
*/
void N5110_setPixel(int x, int y)
{
    if (x>=0 && x<WIDTH && y>=0 && y<HEIGHT) {  // check within range
        // calculate bank and shift 1 to required position in the data byte
        buffer[x][y/8] |= (1 << y%8);
    }
}

/** Clear a Pixel
  *
  *   This function clears pixel in the display. A call to N5110_refresh() must be made
  *   to update the display to reflect the change in pixels.
  *   @param  x - the x co-ordinate of the pixel (0 to 83)
  *   @param  y - the y co-ordinate of the pixel (0 to 47)
  */
void N5110_clearPixel(int x, int y)
{
    if (x>=0 && x<WIDTH && y>=0 && y<HEIGHT) {  // check within range
        // calculate bank and shift 1 to required position (using bit clear)
        buffer[x][y/8] &= ~(1 << y%8);
    }
}

/** Get a Pixel
  *
  *   This function gets the status of a pixel in the display.
  *   @param  x - the x co-ordinate of the pixel (0 to 83)
  *   @param  y - the y co-ordinate of the pixel (0 to 47)
  *   @returns
  *       0           - pixel is clear
  *       non-zero    - pixel is set
  */
int N5110_getPixel(int x, int y)
{
    if (x>=0 && x<WIDTH && y>=0 && y<HEIGHT) {  // check within range
        // return relevant bank and mask required bit
        return (int) buffer[x][y/8] & (1 << y%8);
        // note this does not necessarily return 1 - a non-zero number represents a pixel
    } else {
        return 0;
    }
}

/** Refresh display
*
*   This functions refreshes the display to reflect the current data in the buffer.
*/
void N5110_refresh()
{
    int i,j;

    setXYAddress(0,0);  // important to set address back to 0,0 before refreshing display
    // address auto increments after printing string, so buffer[0][0] will not coincide
    // with top-left pixel after priting string

    HAL_GPIO_WritePin(N5110_SCE_PORT, N5110_SCE_PIN, GPIO_PIN_RESET);  //set CE low to begin frame

    for(j = 0; j < BANKS; j++) {  // be careful to use correct order (j,i) for horizontal addressing
        for(i = 0; i < WIDTH; i++) {
        	HAL_SPI_Transmit(&hspi1,&buffer[i][j],1,100);   // send buffer
        }
    }
    HAL_GPIO_WritePin(N5110_SCE_PORT, N5110_SCE_PIN, GPIO_PIN_SET); // set CE high to end frame

}

// fills the buffer with random bytes.  Can be used to test the display.
// The rand() function isn't seeded so it probably creates the same pattern everytime
/** Randomise buffer
*
*   This function fills the buffer with random data.  Can be used to test the display.
*   A call to N5110_refresh() must be made to update the display to reflect the change in pixels.
*   The seed is not set and so the generated pattern will probably be the same each time.
*   TODO: Randomise the seed - maybe using the noise on the AnalogIn pins.
*/
void N5110_randomiseBuffer()
{
    int i,j;
    for(j = 0; j < BANKS; j++) {  // be careful to use correct order (j,i) for horizontal addressing
        for(i = 0; i < WIDTH; i++) {
            buffer[i][j] = rand()%256;  // generate random byte
        }
    }

}

/** Print Character
*
*   Sends a character to the display.  Printed at the specified location. Character is cut-off after the 83rd pixel.
*   @param  c - the character to print. Can print ASCII as so printChar('C').
*   @param x - the column number (0 to 83)
*   @param y - the row number (0 to 5) - the display is split into 6 banks - each bank can be considered a row
*/
void N5110_printChar(char c,int x,int y)
{
    if (y>=0 && y<BANKS) {  // check if printing in range of y banks

        for (int i = 0; i < 5 ; i++ ) {
            int pixel_x = x+i;
            if (pixel_x > WIDTH-1)  // ensure pixel isn't outside the buffer size (0 - 83)
                break;
            buffer[pixel_x][y] = font5x7[(c - 32)*5 + i];
            // array is offset by 32 relative to ASCII, each character is 5 pixels wide
        }

        N5110_refresh();  // this sends the buffer to the display and sets address (cursor) back to 0,0
    }
}


/** Print String
*
*   Prints a string of characters to the display. String is cut-off after the 83rd pixel.
*   @param x - the column number (0 to 83)
*   @param y - the row number (0 to 5) - the display is split into 6 banks - each bank can be considered a row
*/
void N5110_printString(const char * str,int x,int y)
{
    if (y>=0 && y<BANKS) {  // check if printing in range of y banks

        int n = 0 ; // counter for number of characters in string
        // loop through string and print character
        while(*str) {

            // writes the character bitmap data to the buffer, so that
            // text and pixels can be displayed at the same time
            for (int i = 0; i < 5 ; i++ ) {
                int pixel_x = x+i+n*6;
                if (pixel_x > WIDTH-1) // ensure pixel isn't outside the buffer size (0 - 83)
                    break;
                buffer[pixel_x][y] = font5x7[(*str - 32)*5 + i];
            }

            str++;  // go to next character in string

            n++;    // increment index

        }

        N5110_refresh();  // this sends the buffer to the display and sets address (cursor) back to 0,0
    }
}

/** Clears
*
*   Clears the screen.
*/
void N5110_clear()
{
    clearBuffer();  // clear the buffer then call the N5110_refresh function
    N5110_refresh();
}

// function to clear the buffer
static void clearBuffer()
{
    int i,j;
    for (i=0; i<WIDTH; i++) {  // loop through the banks and set the buffer to 0
        for (j=0; j<BANKS; j++) {
            buffer[i][j]=0;
        }
    }
}


/** Plot Array
*
*   This function plots a one-dimensional array on the display.
*   @param array[] - y values of the plot. Values should be normalised in the range 0.0 to 1.0. First 84 plotted.
*/
void N5110_plotArray(float array[])
{

    int i;

    for (i=0; i<WIDTH; i++) {  // loop through array
        // elements are normalised from 0.0 to 1.0, so multiply
        // by 47 to convert to pixel range, and subtract from 47
        // since top-left is 0,0 in the display geometry
    	N5110_setPixel(i,47 - (int)(array[i]*47.0));
    }

    N5110_refresh();

}

/** Draw Circle
*
*   This function draws a circle at the specified origin with specified radius to the display.
*   Uses the midpoint circle algorithm.
*   @see http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
*   @param  x0 - x-coordinate of centre
*   @param  y0 - y-coordinate of centre
*   @param  radius - radius of circle in pixels
*   @param  fill - 0 transparent (w/outline), 1 filled black, 2 filled white (wo/outline)
*/
void N5110_drawCircle(int x0,int y0,int radius,int fill)
{
    // from http://en.wikipedia.org/wiki/Midpoint_circle_algorithm
    int x = radius;
    int y = 0;
    int radiusError = 1-x;

    while(x >= y) {

        // if transparent, just draw outline
        if (fill == 0) {
            N5110_setPixel( x + x0,  y + y0);
            N5110_setPixel(-x + x0,  y + y0);
            N5110_setPixel( y + x0,  x + y0);
            N5110_setPixel(-y + x0,  x + y0);
            N5110_setPixel(-y + x0, -x + y0);
            N5110_setPixel( y + x0, -x + y0);
            N5110_setPixel( x + x0, -y + y0);
            N5110_setPixel(-x + x0, -y + y0);
        } else {  // drawing filled circle, so draw lines between points at same y value

            int type = (fill==1) ? 1:0;  // black or white fill

            N5110_drawLine(x+x0,y+y0,-x+x0,y+y0,type);
            N5110_drawLine(y+x0,x+y0,-y+x0,x+y0,type);
            N5110_drawLine(y+x0,-x+y0,-y+x0,-x+y0,type);
            N5110_drawLine(x+x0,-y+y0,-x+x0,-y+y0,type);
        }


        y++;
        if (radiusError<0) {
            radiusError += 2 * y + 1;
        } else {
            x--;
            radiusError += 2 * (y - x) + 1;
        }
    }

}

/** Draw Line
  *
  *   This function draws a line between the specified points using linear interpolation.
  *   @param  x0 - x-coordinate of first point
  *   @param  y0 - y-coordinate of first point
  *   @param  x1 - x-coordinate of last point
  *   @param  y1 - y-coordinate of last point
  *   @param  type - 0 white,1 black,2 dotted
  */
void N5110_drawLine(int x0,int y0,int x1,int y1,int type)
{
    int y_range = y1-y0;  // calc range of y and x
    int x_range = x1-x0;
    int start,stop,step;

    // if dotted line, set step to 2, else step is 1
    step = (type==2) ? 2:1;

    // make sure we loop over the largest range to get the most pixels on the display
    // for instance, if drawing a vertical line (x_range = 0), we need to loop down the y pixels
    // or else we'll only end up with 1 pixel in the x column
    if ( abs(x_range) > abs(y_range) ) {

        // ensure we loop from smallest to largest or else for-loop won't run as expected
        start = x1>x0 ? x0:x1;
        stop =  x1>x0 ? x1:x0;

        // loop between x pixels
        for (int x = start; x<= stop ; x+=step) {
            // do linear interpolation
            int y = y0 + (y1-y0)*(x-x0)/(x1-x0);

            if (type == 0)   // if 'white' line, turn off pixel
            	N5110_clearPixel(x,y);
            else
                N5110_setPixel(x,y);  // else if 'black' or 'dotted' turn on pixel
        }
    } else {

        // ensure we loop from smallest to largest or else for-loop won't run as expected
        start = y1>y0 ? y0:y1;
        stop =  y1>y0 ? y1:y0;

        for (int y = start; y<= stop ; y+=step) {
            // do linear interpolation
            int x = x0 + (x1-x0)*(y-y0)/(y1-y0);

            if (type == 0)   // if 'white' line, turn off pixel
            	N5110_clearPixel(x,y);
            else
                N5110_setPixel(x,y);  // else if 'black' or 'dotted' turn on pixel

        }
    }

}

/** Draw Rectangle
*
*   This function draws a rectangle.
*   @param  x0 - x-coordinate of origin (top-left)
*   @param  y0 - y-coordinate of origin (top-left)
*   @param  width - width of rectangle
*   @param  height - height of rectangle
*   @param  fill - 0 transparent (w/outline), 1 filled black, 2 filled white (wo/outline)
*/
void N5110_drawRect(int x0,int y0,int width,int height,int fill)
{

    if (fill == 0) { // transparent, just outline
    	N5110_drawLine(x0,y0,x0+width,y0,1);  // top
    	N5110_drawLine(x0,y0+height,x0+width,y0+height,1);  // bottom
    	N5110_drawLine(x0,y0,x0,y0+height,1);  // left
    	N5110_drawLine(x0+width,y0,x0+width,y0+height,1);  // right
    } else { // filled rectangle
        int type = (fill==1) ? 1:0;  // black or white fill
        for (int y = y0; y<= y0+height; y++) {  // loop through rows of rectangle
        	N5110_drawLine(x0,y,x0+width,y,type);  // draw line across screen
        }
    }

}


const unsigned char font5x7[480] = {
    0x00, 0x00, 0x00, 0x00, 0x00,// (space)
    0x00, 0x00, 0x5F, 0x00, 0x00,// !
    0x00, 0x07, 0x00, 0x07, 0x00,// "
    0x14, 0x7F, 0x14, 0x7F, 0x14,// #
    0x24, 0x2A, 0x7F, 0x2A, 0x12,// $
    0x23, 0x13, 0x08, 0x64, 0x62,// %
    0x36, 0x49, 0x55, 0x22, 0x50,// &
    0x00, 0x05, 0x03, 0x00, 0x00,// '
    0x00, 0x1C, 0x22, 0x41, 0x00,// (
    0x00, 0x41, 0x22, 0x1C, 0x00,// )
    0x08, 0x2A, 0x1C, 0x2A, 0x08,// *
    0x08, 0x08, 0x3E, 0x08, 0x08,// +
    0x00, 0x50, 0x30, 0x00, 0x00,// ,
    0x08, 0x08, 0x08, 0x08, 0x08,// -
    0x00, 0x60, 0x60, 0x00, 0x00,// .
    0x20, 0x10, 0x08, 0x04, 0x02,// /
    0x3E, 0x51, 0x49, 0x45, 0x3E,// 0
    0x00, 0x42, 0x7F, 0x40, 0x00,// 1
    0x42, 0x61, 0x51, 0x49, 0x46,// 2
    0x21, 0x41, 0x45, 0x4B, 0x31,// 3
    0x18, 0x14, 0x12, 0x7F, 0x10,// 4
    0x27, 0x45, 0x45, 0x45, 0x39,// 5
    0x3C, 0x4A, 0x49, 0x49, 0x30,// 6
    0x01, 0x71, 0x09, 0x05, 0x03,// 7
    0x36, 0x49, 0x49, 0x49, 0x36,// 8
    0x06, 0x49, 0x49, 0x29, 0x1E,// 9
    0x00, 0x36, 0x36, 0x00, 0x00,// :
    0x00, 0x56, 0x36, 0x00, 0x00,// ;
    0x00, 0x08, 0x14, 0x22, 0x41,// <
    0x14, 0x14, 0x14, 0x14, 0x14,// =
    0x41, 0x22, 0x14, 0x08, 0x00,// >
    0x02, 0x01, 0x51, 0x09, 0x06,// ?
    0x32, 0x49, 0x79, 0x41, 0x3E,// @
    0x7E, 0x11, 0x11, 0x11, 0x7E,// A
    0x7F, 0x49, 0x49, 0x49, 0x36,// B
    0x3E, 0x41, 0x41, 0x41, 0x22,// C
    0x7F, 0x41, 0x41, 0x22, 0x1C,// D
    0x7F, 0x49, 0x49, 0x49, 0x41,// E
    0x7F, 0x09, 0x09, 0x01, 0x01,// F
    0x3E, 0x41, 0x41, 0x51, 0x32,// G
    0x7F, 0x08, 0x08, 0x08, 0x7F,// H
    0x00, 0x41, 0x7F, 0x41, 0x00,// I
    0x20, 0x40, 0x41, 0x3F, 0x01,// J
    0x7F, 0x08, 0x14, 0x22, 0x41,// K
    0x7F, 0x40, 0x40, 0x40, 0x40,// L
    0x7F, 0x02, 0x04, 0x02, 0x7F,// M
    0x7F, 0x04, 0x08, 0x10, 0x7F,// N
    0x3E, 0x41, 0x41, 0x41, 0x3E,// O
    0x7F, 0x09, 0x09, 0x09, 0x06,// P
    0x3E, 0x41, 0x51, 0x21, 0x5E,// Q
    0x7F, 0x09, 0x19, 0x29, 0x46,// R
    0x46, 0x49, 0x49, 0x49, 0x31,// S
    0x01, 0x01, 0x7F, 0x01, 0x01,// T
    0x3F, 0x40, 0x40, 0x40, 0x3F,// U
    0x1F, 0x20, 0x40, 0x20, 0x1F,// V
    0x7F, 0x20, 0x18, 0x20, 0x7F,// W
    0x63, 0x14, 0x08, 0x14, 0x63,// X
    0x03, 0x04, 0x78, 0x04, 0x03,// Y
    0x61, 0x51, 0x49, 0x45, 0x43,// Z
    0x00, 0x00, 0x7F, 0x41, 0x41,// [
    0x02, 0x04, 0x08, 0x10, 0x20,// "\"
    0x41, 0x41, 0x7F, 0x00, 0x00,// ]
    0x04, 0x02, 0x01, 0x02, 0x04,// ^
    0x40, 0x40, 0x40, 0x40, 0x40,// _
    0x00, 0x01, 0x02, 0x04, 0x00,// `
    0x20, 0x54, 0x54, 0x54, 0x78,// a
    0x7F, 0x48, 0x44, 0x44, 0x38,// b
    0x38, 0x44, 0x44, 0x44, 0x20,// c
    0x38, 0x44, 0x44, 0x48, 0x7F,// d
    0x38, 0x54, 0x54, 0x54, 0x18,// e
    0x08, 0x7E, 0x09, 0x01, 0x02,// f
    0x08, 0x14, 0x54, 0x54, 0x3C,// g
    0x7F, 0x08, 0x04, 0x04, 0x78,// h
    0x00, 0x44, 0x7D, 0x40, 0x00,// i
    0x20, 0x40, 0x44, 0x3D, 0x00,// j
    0x00, 0x7F, 0x10, 0x28, 0x44,// k
    0x00, 0x41, 0x7F, 0x40, 0x00,// l
    0x7C, 0x04, 0x18, 0x04, 0x78,// m
    0x7C, 0x08, 0x04, 0x04, 0x78,// n
    0x38, 0x44, 0x44, 0x44, 0x38,// o
    0x7C, 0x14, 0x14, 0x14, 0x08,// p
    0x08, 0x14, 0x14, 0x18, 0x7C,// q
    0x7C, 0x08, 0x04, 0x04, 0x08,// r
    0x48, 0x54, 0x54, 0x54, 0x20,// s
    0x04, 0x3F, 0x44, 0x40, 0x20,// t
    0x3C, 0x40, 0x40, 0x20, 0x7C,// u
    0x1C, 0x20, 0x40, 0x20, 0x1C,// v
    0x3C, 0x40, 0x30, 0x40, 0x3C,// w
    0x44, 0x28, 0x10, 0x28, 0x44,// x
    0x0C, 0x50, 0x50, 0x50, 0x3C,// y
    0x44, 0x64, 0x54, 0x4C, 0x44,// z
    0x00, 0x08, 0x36, 0x41, 0x00,// {
    0x00, 0x00, 0x7F, 0x00, 0x00,// |
    0x00, 0x41, 0x36, 0x08, 0x00,// }
    0x08, 0x08, 0x2A, 0x1C, 0x08,// ->
    0x08, 0x1C, 0x2A, 0x08, 0x08 // <-
};
