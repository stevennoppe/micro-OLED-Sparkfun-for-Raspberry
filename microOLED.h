/******************************************************************************
microOLED.cpp

Main source code for the MicroOLED Raspberry Pi Library

Jim Lindblom @ SparkFun Electronics
October 26, 2014
https://github.com/sparkfun/Micro_OLED_Breakout/tree/master/Firmware/Arduino/libraries/SFE_MicroOLED


This file defines the hardware interface(s) for the Micro OLED Breakout. Those
interfaces include SPI, I2C and a parallel bus.


Development environment specifics:
Raspberry Pi
Micro OLED Breakout v1.0

This code was heavily based around the MicroView library, written by GeekAmmo
(https://github.com/geekammo/MicroView-Arduino-Library), and released under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.

Changes were made by Steven Noppe so it is usable on a Raspberry Pi.
4 April, 2017

TODO : 
		- Parallel
		- scrolling horizontal
		- scrolling parameters like offset and speed...
 
This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
******************************************************************************/

#ifndef MICROOLED_H
#define MICROOLED_H

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <linux/i2c-dev.h>              // for the ioctl() function
#include <linux/spi/spidev.h>
#include <unistd.h>						// for the read() and write() function
#include <fcntl.h>						// for the open() function
#include <string.h>						// for memset() function)
#include <time.h>
#include <errno.h>

#include <wiringPi.h>

// macro's
#define swap(a, b) { uint8_t t = a; a = b; b = t; }
#define _BV(v) (1<<(v))
#define pgm_read_byte(addr) (*(const unsigned char *)(addr))

// I2C
#define I2C_ADDRESS_SA0_0				0b0111100		// 0x3C (60)
#define I2C_ADDRESS_SA0_1				0b0111101		// 0x3D	(61)
#define I2C_COMMAND						0x00			// first byte 0x00 if
														// you want to send a 
														// command
#define I2C_DATA						0x40			// first byte 0x40 if
														// you want to send data

#define LCDWIDTH						64
#define LCDHEIGHT						48
#define FONTHEADERSIZE					6

#define ALL								1
#define PAGE							0

#define NORM							0
#define XOR								1

#define BLACK							0
#define WHITE							1

#define DISPLAYOFF						0xAE
#define DISPLAYON						0xAF
#define NORMALDISPLAY					0xA6
#define INVERTDISPLAY					0xA7
#define DISPLAYALLONRESUME				0xA4
#define DISPLAYALLON					0xA5

#define SETCONTRAST						0x81
#define SETDISPLAYOFFSET				0xD3
#define SETCOMPINS						0xDA
#define SETVCOMDESELECT					0xDB
#define SETDISPLAYCLOCKDIV				0xD5
#define SETPRECHARGE					0xD9
#define SETMULTIPLEX					0xA8
#define SETLOWCOLUMN					0x00
#define SETHIGHCOLUMN					0x10
#define SETSTARTLINE					0x40

#define CHARGEPUMP						0x8D
#define SEGREMAP						0xA0
#define COMSCANINC						0xC0
#define COMSCANDEC						0xC8

// Scroll
#define ACTIVATESCROLL 					0x2F
#define DEACTIVATESCROLL 				0x2E
#define SETVERTICALSCROLLAREA 			0xA3
#define RIGHTHORIZONTALSCROLL 			0x26
#define LEFTHORIZONTALSCROLL 			0x27
#define VERTICALRIGHTHORIZONTALSCROLL	0x29
#define VERTICALLEFTHORIZONTALSCROLL	0x2A

typedef enum COMM_MODE
{
	MODE_SPI,
	MODE_I2C,
	MODE_PARALLEL
} micro_oled_mode ;

typedef enum CMD 
{
	CMD_CLEAR,			//0
	CMD_INVERT,			//1
	CMD_CONTRAST,		//2
	CMD_DISPLAY,		//3
	CMD_SETCURSOR,		//4
	CMD_PIXEL,			//5
	CMD_LINE,			//6
	CMD_LINEH,			//7
	CMD_LINEV,			//8
	CMD_RECT,			//9
	CMD_RECTFILL,		//10
	CMD_CIRCLE,			//11
	CMD_CIRCLEFILL,		//12
	CMD_DRAWCHAR,		//13
	CMD_DRAWBITMAP,		//14
	CMD_GETLCDWIDTH,	//15
	CMD_GETLCDHEIGHT,	//16
	CMD_SETCOLOR,		//17
	CMD_SETDRAWMODE		//18
} commCommand_t ;

class microOLED
{
public:
	microOLED() ;
	microOLED(const microOLED& orig) ;
	microOLED(uint8_t rst, uint8_t dc, uint8_t interface_mode) ;  
	virtual ~microOLED() ;
	
	void begin(void) ;
	void clear(uint8_t mode) ;
	void display(void) ;
	
	int command(uint8_t c) ;
	int data(uint8_t c) ;
	
	void invert(bool inv) ;
	void flipVertical(bool flip) ;
	void flipHorizontal(bool flip) ;
	
	uint8_t getLCDHeight(void) ;
	uint8_t getLCDWidth(void) ;
	uint8_t getFontWidth(void) ;
	uint8_t getFontHeight(void) ;
	uint8_t *getScreenBuffer(void) ;
	
	void setColumnAddress(uint8_t add) ;
	void setPageAddress(uint8_t add) ;
	short int setFontType(uint8_t type) ;
	void setColor(uint8_t color) ;
	void setDrawMode(uint8_t mode) ;
	void setCursor(uint8_t x, uint8_t y) ;
	void setContrast(uint8_t contrast) ;
		
	void line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) ;
	void line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, 
			  uint8_t color, uint8_t mode) ;
	void lineH(uint8_t x, uint8_t y, uint8_t width) ;
	void lineH(uint8_t x, uint8_t y, uint8_t width, uint8_t color, 
			   uint8_t mode) ;
	void lineV(uint8_t x, uint8_t y, uint8_t height) ;
	void lineV(uint8_t x, uint8_t y, uint8_t height, uint8_t color, 
			   uint8_t mode) ;
	
	void pixel(uint8_t x, uint8_t y) ;
	void pixel(uint8_t x, uint8_t y, uint8_t color, uint8_t mode) ;
	
	void rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height) ;
	void rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, 
					 uint8_t color , uint8_t mode) ; 
	void rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height) ;
	void rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, 
		          uint8_t color , uint8_t mode) ;
	
	void circle(uint8_t x0, uint8_t y0, uint8_t radius) ;
	void circle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color, 
				uint8_t mode) ;
	void circleFill(uint8_t x0, uint8_t y0, uint8_t radius) ;
	void circleFill(uint8_t x0, uint8_t y0, uint8_t radius, 
						   uint8_t color, uint8_t mode) ;
	
	void drawBitmap(uint8_t * bitArray) ;
	
	size_t printString(const char *ifsh) ;
	size_t print(const char *fmt, ...) ;
	
	size_t writeChar(uint8_t c) ;
	void  drawChar(uint8_t x, uint8_t y, uint8_t c)  ;
	void  drawChar(uint8_t x, uint8_t y, uint8_t c, 
				   uint8_t color, uint8_t mode) ;
	
	void scrollStop(void) ;
	void scrollRight(uint8_t start, uint8_t stop) ;
	void scrollLeft(uint8_t start, uint8_t stop) ;
	void scrollVertRight(uint8_t start, uint8_t stop) ;
	void scrollVertLeft(uint8_t start, uint8_t stop) ;
	
	int waitMiliSec(float miliS) ;	
	
private:
	int					i2c_fd ;
	int					spi_fd ;
	int i2cSetup() ;
	int spiSetup() ;
	
	timespec			waitTime ;
	float				seconds ;
	long long			nanoseconds ;
		
	uint8_t				csPin, 
						dcPin, 
						rstPin ;
	micro_oled_mode		interface ;
	char				i2c_address ;
	// use volatile because these are fixed location port address
	volatile uint8_t	*ssport, 
						*dcport, 
						*ssreg, 
						*dcreg ;
	uint8_t				mosipinmask, 
						sckpinmask, 
						sspinmask, 
						dcpinmask;	
	uint8_t				foreColor, 
						drawMode, 
						fontWidth, 
						fontHeight, 
						fontType, 
						fontStartChar, 
						fontTotalChar,
						cursorX, 
						cursorY ;
	uint16_t			fontMapWidth ;
	static const char *fontsPointer[] ;
} ;

#endif /* MICROOLED_H */

