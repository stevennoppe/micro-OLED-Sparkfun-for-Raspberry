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
		- SPI
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

#include "microOLED.h"

// Add header of the fonts here.  
// Remove as many as possible to conserve FLASH memory.
#include "font5x7.h"
#include "font8x16.h"
#include "fontlargenumber.h"
#include "7segment.h"

// Change the total fonts included
#define TOTALFONTS		4

// Add the font name as declared in the header file.  
// Remove as many as possible to conserve FLASH memory.
const char *microOLED::fontsPointer[]=
{
	"font5x7",
	"font8x16",
	"sevensegment",
	"fontlargenumber"
} ;

/** \brief MicroOLED screen buffer.

Page buffer 64 x 48 divided by 8 = 384 bytes
Page buffer is required because in SPI mode, the host cannot read the 
SSD1306's GDRAM of the controller.  This page buffer serves as a scratch RAM 
for graphical functions.  All drawing function will first be drawn on this 
page buffer, only upon calling display() function will transfer the page 
buffer to the actual LCD controller's memory.
*/
static uint8_t screenmemory[] = 
{
	/* LCD Memory organised in 64 horizontal pixel and 6 rows of byte
	 B  B .............B  -----
	 y  y .............y        \
	 t  t .............t         \
	 e  e .............e          \
	 0  1 .............63          \
	                                \
	 D0 D0.............D0            \
	 D1 D1.............D1            / ROW 0
	 D2 D2.............D2           /
	 D3 D3.............D3          /
	 D4 D4.............D4         /
	 D5 D5.............D5        /
	 D6 D6.............D6       /
	 D7 D7.............D7  ----
	*/

	//SparkFun Electronics LOGO

	// ROW0, BYTE0 to BYTE63
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xE0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0x0F, 0x07, 0x07, 0x06, 0x06, 0x00, 0x80, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// ROW1, BYTE64 to BYTE127
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x80, 0x80, 0x80, 0x80, 0x80, 0x80, 0x81, 0x07, 0x0F, 0x3F, 0x3F, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFC, 0xFC, 0xFC, 0xFE, 0xFF, 0xFF, 0xFF, 0xFC, 0xF8, 0xE0,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// ROW2, BYTE128 to BYTE191
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 
	0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF1, 0xE0, 0xE0, 0xE0, 0xE0, 0xE0, 0xF0, 0xFD, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// ROW3, BYTE192 to BYTE255
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,	
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x1F, 0x07, 0x01,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// ROW4, BYTE256 to BYTE319
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 
	0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0x1F, 0x1F, 0x0F, 0x0F, 0x0F, 0x0F, 
	0x0F, 0x0F, 0x0F, 0x0F, 0x07, 0x07, 0x07, 0x03, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

	// ROW5, BYTE320 to BYTE383
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 
	0x7F, 0x3F, 0x1F, 0x0F, 0x07, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
} ;

microOLED::microOLED()
{
}

microOLED::microOLED(const microOLED& orig)
{
}

microOLED::~microOLED()
{
}

/** \brief MicroOLED Constructor -- 0=SPI, 1=I2C, 2=parallel

	Setup the MicroOLED class, configure the display to be controlled via a
	I2C, SPI or parallel interface.
*/
microOLED::microOLED(uint8_t rst, uint8_t dc, uint8_t interface_mode)
{
	wiringPiSetup() ;
	
	rstPin = rst ;				// Assign reset pin to private class variable
	
	switch (interface_mode)
	{
		case 0:
			//configure SPI
			dcPin = dc;
			interface = MODE_SPI;	// Set interface mode to SPI
			
			break ;
		case 1:
			//configure I2C
			interface = MODE_I2C ;		// Set interface to I2C

			// Set the I2C Address based on whether DC is high (1) or low (0).
			// The pin is pulled low by default, so if it's not explicitly 
			// set to 1, just default to 0.
			if (dc == 1)
				i2c_address = I2C_ADDRESS_SA0_1 ;
			else
				i2c_address = I2C_ADDRESS_SA0_0 ;
			break ;
		
		case 2:
			// configure parallel
			
			break ;
	}
}

void microOLED::begin()
{
	// default 5x7 font
	setFontType(0);
	setColor(WHITE);
	setDrawMode(NORM);
	setCursor(0,0);

	pinMode(dcPin, OUTPUT) ;
	pinMode(rstPin, OUTPUT) ;
	
	// Set up the selected interface:
	if (interface == MODE_SPI)
	{
		//Setup SPI
		spiSetup();
	}	
	else if (interface == MODE_I2C)
	{
		// Setup I2C
		i2cSetup();
	}
	else if (interface == MODE_PARALLEL)
	{
		// setup parallel
		//parallelSetup();
	}

	// Display reset routine
	//pinMode(rstPin, OUTPUT);		// Set RST pin as OUTPUT
	digitalWrite(rstPin, HIGH);		// Initially set RST HIGH
	waitMilliSec(5.0) ;				// VDD (3.3V) goes high at start, 
									// lets just chill for 5 ms
	digitalWrite(rstPin, LOW);		// Bring RST low, reset the display
	waitMilliSec(10.0) ;			// wait 10ms
	digitalWrite(rstPin, HIGH);		// Set RST HIGH, bring out of reset

	// Display Init sequence for 64x48 OLED module
	command(DISPLAYOFF);			// 0xAE
	
	command(SETDISPLAYCLOCKDIV);	// 0xD5
	command(0x80);					// the suggested ratio 0x80

	command(SETMULTIPLEX);			// 0xA8
	command(0x2F);

	command(SETDISPLAYOFFSET);		// 0xD3
	command(0x0);					// no offset

	command(SETSTARTLINE | 0x0);	// line #0

	command(CHARGEPUMP);			// enable charge pump
	command(0x14);

	command(NORMALDISPLAY);			// 0xA6
	command(DISPLAYALLONRESUME);	// 0xA4

	command(SEGREMAP | 0x1);
	command(COMSCANDEC);

	command(SETCOMPINS);			// 0xDA
	command(0x12);

	command(SETCONTRAST);			// 0x81
	command(0x8F);

	command(SETPRECHARGE);			// 0xd9
	command(0xF1);

	command(SETVCOMDESELECT);		// 0xDB
	command(0x40);

	command(DISPLAYON);				//--turn on oled panel
		
	clear(ALL);						// Erase hardware memory inside 
									// the OLED controller to avoid random 
									// data in memory.
}

/** \brief Setup the I2C communication
 
 Setup the I2C communication
 */
int microOLED::i2cSetup()
{
	if ((i2c_fd = open("/dev/i2c-1", O_RDWR)) < 0) 
    // on a Raspberry Pi - model B rev 1 use : '/dev/i2c-0'
    {
		// If it returns < 0 then something was wrong
		// exit function with a -1 : failed to open de port
		printf("Failed to open I2C line!") ;
		
		return -1 ;
    }
	
	return 0 ;
}

/** \brief Setup the I2C communication
 
 Setup the SPI communication
 */
int microOLED::spiSetup()
{
	unsigned int speed = 1000000;
		
	errno = 0 ;
	if ((spi_fd = open("/dev/spidev0.0", O_RDWR)) < 0) 
    {
		fprintf(stderr, "SPI open failed: %s\n", strerror (errno)) ;
		return -1 ;
    }
	
	errno = 0 ;
		
	if (ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0)
	{
		fprintf(stderr, "SPI set speed failed: %s\n", strerror (errno)) ;
		return -1 ;
	}
	
	return 0 ;
}

/** \brief Send the display a command byte

    Send a command via SPI, I2C or parallel	to SSD1306 controller.
	For SPI we set the DC and CS pins here, and call spiTransfer(byte)
	to send the data. For I2C and Parallel we use the write functions
	defined in hardware.cpp to send the data.
*/
int microOLED::command(uint8_t c) 
{
	if (interface == MODE_SPI)
	{
		struct spi_ioc_transfer spi;
		memset (&spi, 0, sizeof (spi)) ;
		
		spi.tx_buf		=  0 ; 
		spi.tx_buf		=  (unsigned long) &c ; 
		spi.rx_buf		=  (unsigned long) &c ;
		spi.len			=	sizeof(c) ;
		spi.speed_hz	=	1000000 ;
		spi.delay_usecs	=	0 ;
		spi.bits_per_word = 8 ;
				
		errno = 0 ;
		if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi) < 0)
		{
			 // failed to send byte
			fprintf(stderr, "SPI send command failed: %s\n", strerror (errno)) ;
			return -1 ;
		}
					
		return 0 ;
	}

	else if (interface == MODE_I2C)
	{
		// Write to our address, make sure it knows we're sending a
		// command by settingb the first byte to 0x00
		ssize_t result ;
		char buf[2] ;
    	
		buf[0]=I2C_COMMAND ;	// = 0x00 => send a command         
		buf[1]=c ;				// = command to send

		if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0)
        {
            // didn't find a slave at that address
			printf("Failed to find a i2c slave!\n") ;
			return -1 ;
        }
        else
        {
			// found the OLED, now send the command
            if ((result = write(i2c_fd, buf, 2)) != 2)
            {
                // failed to send the command
				printf("Failed to send a command to the i2c slave!\n") ;
				return -1 ;
            }
        }
		
		return 0 ;
	}
	else if (interface == MODE_PARALLEL)
	{
		// Write the byte to our parallel interface. Set DC LOW.
		// parallelWrite(c, LOW);
	}
}

/** \brief Send the display a data byte

    Send a data byte via SPI, I2C or parallel to SSD1306 controller.
	For SPI we set the DC and CS pins here, and call spiTransfer(byte)
	to send the data. For I2C and Parallel we use the write functions
	defined in hardware.cpp to send the data.
*/
int microOLED::data(uint8_t c) 
{
	if (interface == MODE_SPI)
	{		
		struct spi_ioc_transfer spi;
		memset (&spi, 0, sizeof (spi)) ;
		
		spi.tx_buf		=  (unsigned long) &c ;
		spi.rx_buf		=  (unsigned long) &c ;
		spi.len			=	sizeof(c) ;
		spi.speed_hz	=	1000000 ;
		spi.delay_usecs	=	0 ;
		spi.bits_per_word = 8 ;
				
		digitalWrite(dcPin, HIGH) ; // high to send data
		errno = 0 ;
		if (ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi) < 0)
		{
			// failed to send byte
			fprintf(stderr, "SPI send data failed: %s\n", strerror (errno)) ;
			return -1 ;
		}
		
		digitalWrite(dcPin, LOW) ; // After sending data set pin back to LOW
		
		return 0 ;
	}
	else if (interface == MODE_I2C)
	{
		// Write to our address, make sure it knows we're sending a
		// data byte by setting the first byte to 0x40
		ssize_t result ;
		char buf[2] ;
    	
		buf[0]=I2C_DATA ;			// = 0x40 => send data
		buf[1]=c ;					// = data

		if (ioctl(i2c_fd, I2C_SLAVE, i2c_address) < 0)
        {
            printf("Failed to find a i2c slave!\n") ;
			return -1 ;
        }
        else
        {
			// found the OLED, now send the data
            if ((result = write(i2c_fd, buf, 2)) != 2)
            {
                // failed to send data
				printf("Failed to send data to the i2c slave!\n") ;
				return -1 ;
            }
        }
	}
	else if (interface == MODE_PARALLEL)
	{
		// Write the byte to our parallel interface. Set DC HIGH.
		// parallelWrite(c, HIGH);
	}
}

/** \brief Clear screen buffer or SSD1306's memory.

    To clear GDRAM inside the LCD controller, pass in the variable 
	mode = ALL and to clear screen page buffer pass in the variable mode = PAGE.
*/
void microOLED::clear(uint8_t mode) 
{
	//	uint8_t page=6, col=0x40;
	if (mode==ALL) {
		for (int i=0;i<8; i++) {
			setPageAddress(i);
			setColumnAddress(0);
			for (int j=0; j<0x80; j++) {
				data(0);
			}
		}
	}
	else
	{
		memset(screenmemory,0,384);			// (64 x 48) / 8 = 384
		//display();
	}
}

/** \brief Transfer display memory.

    Bulk move the screen buffer to the SSD1306 controller's memory so that 
	images/graphics drawn on the screen buffer will be displayed on the OLED.
*/
void microOLED::display(void) 
{
	short int i, j;

	for (i=0; i<6; i++) 
	{
		setPageAddress(i);
		setColumnAddress(0);
		for (j=0;j<0x40;j++) 
		{
			data(screenmemory[i*0x40+j]);
		}
	}
}

/** \brief Set SSD1306 column address.

    Send column address command and address to the SSD1306 OLED controller.
*/
void microOLED::setColumnAddress(uint8_t add) 
{
	command((0x10|(add>>4))+0x02);
	command((0x0f&add));
	return;

}

/** \brief Set SSD1306 page address.

    Send page address command and address to the SSD1306 OLED controller.
*/
void microOLED::setPageAddress(uint8_t add) 
{
	add=0xb0|add ;
	command(add) ;
	return ;
}

/** \brief Set font type.

    Set the current font type number, ie changing to different fonts base 
	on the type provided.
*/
short int microOLED::setFontType(uint8_t type) 
{
	if ((type>=TOTALFONTS) || (type<0))
		return false;

	fontType=type;
	
	switch(fontType)
	{
		case 0:
			fontWidth=font5x7[0];
			fontHeight=font5x7[1];
			fontStartChar=font5x7[2];
			fontTotalChar=font5x7[3];
			fontMapWidth=font5x7[4]*100+font5x7[5]; // two bytes values into integer 16
			break ;
		case 1:
			fontWidth=font8x16[0];
			fontHeight=font8x16[1];
			fontStartChar=font8x16[2];
			fontTotalChar=font8x16[3];
			fontMapWidth=font8x16[4]*100+font8x16[5]; // two bytes values into integer 16
			break ;
		case 2:
			fontWidth=sevensegment[0];
			fontHeight=sevensegment[1];
			fontStartChar=sevensegment[2];
			fontTotalChar=sevensegment[3];
			fontMapWidth=sevensegment[4]*100+sevensegment[5]; // two bytes values into integer 16
			break ;
		case 3:
			fontWidth=fontlargenumber[0];
			fontHeight=fontlargenumber[1];
			fontStartChar=fontlargenumber[2];
			fontTotalChar=fontlargenumber[3];
			fontMapWidth=fontlargenumber[4]*100+fontlargenumber[5]; // two bytes values into integer 16
			break ;
	}
	
	
	/*
	fontWidth=pgm_read_byte(fontsPointer[fontType]+0);
	fontHeight=pgm_read_byte(fontsPointer[fontType]+1);
	fontStartChar=pgm_read_byte(fontsPointer[fontType]+2);
	fontTotalChar=pgm_read_byte(fontsPointer[fontType]+3);
	fontMapWidth=(pgm_read_byte(fontsPointer[fontType]+4)*100)+pgm_read_byte(fontsPointer[fontType]+5); // two bytes values into integer 16
	*/
	return true ;
}

/** \brief Set color.

	Set the current draw's color. Only WHITE and BLACK available.
*/
void microOLED::setColor(uint8_t color) 
{
	foreColor=color ;
}

/** \brief Set draw mode.

    Set current draw mode with NORM or XOR.
*/
void microOLED::setDrawMode(uint8_t mode) 
{
	drawMode=mode ;
}

/** \brief Set cursor position.

	MicroOLED's cursor position to x,y.
*/
void microOLED::setCursor(uint8_t x, uint8_t y) 
{
	cursorX=x ;
	cursorY=y ;
}

/** \brief Draw line.

Draw line using current fore color and current draw mode from x0,y0 to x1,y1 of the screen buffer.
*/
void microOLED::line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1) 
{
	line(x0,y0,x1,y1,foreColor,drawMode);
}

/** \brief Draw line with color and mode.

Draw line using color and mode from x0,y0 to x1,y1 of the screen buffer.
*/
void microOLED::line(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, 
					 uint8_t color, uint8_t mode) 
{
	uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) 
	{
		swap(x0, y0) ;
		swap(x1, y1) ;
	}

	if (x0 > x1) 
	{
		swap(x0, x1) ;
		swap(y0, y1) ;
	}

	uint8_t dx, dy ;
	dx = x1 - x0 ;
	dy = abs(y1 - y0) ;

	int8_t err = dx / 2 ;
	int8_t ystep ;

	if (y0 < y1) 
	{
		ystep = 1 ;
	} 
	else 
	{
		ystep = -1 ;
	}

	for (; x0<x1; x0++) 
	{
		if (steep) 
		{
			pixel(y0, x0, color, mode) ;
		} 
		else 
		{
			pixel(x0, y0, color, mode) ;
		}
		err -= dy ;

		if (err < 0) 
		{
			y0 += ystep ;
			err += dx ;
		}
	}
}

/** \brief Draw pixel.

Draw pixel using the current fore color and current draw mode in the screen buffer's x,y position.
*/
void microOLED::pixel(uint8_t x, uint8_t y) 
{
	pixel(x,y,foreColor,drawMode) ;
}

/** \brief Draw pixel with color and mode.

Draw color pixel in the screen buffer's x,y position with NORM or XOR draw mode.
*/
void microOLED::pixel(uint8_t x, uint8_t y, uint8_t color, uint8_t mode) 
{
	if ((x<0) ||  (x>=LCDWIDTH) || (y<0) || (y>=LCDHEIGHT))
		return ;

	if (mode==XOR) 
	{
		if (color==WHITE)
			screenmemory[x+ (y/8)*LCDWIDTH] ^= _BV((y%8)) ;
	}
	else 
	{
		if (color==WHITE)
			screenmemory[x+ (y/8)*LCDWIDTH] |= _BV((y%8));
		else
			screenmemory[x+ (y/8)*LCDWIDTH] &= ~_BV((y%8));
	}
}

/** \brief Invert display.

	The WHITE color of the display will turn to BLACK and the BLACK will turn to WHITE.
*/
void microOLED::invert(bool inv) 
{
	if (inv)
		command(INVERTDISPLAY);
	else
		command(NORMALDISPLAY);
}

/** \brief Set contrast.

    OLED contract value from 0 to 255. Note: Contrast level is not very obvious.
*/
void microOLED::setContrast(uint8_t contrast) 
{
	if (contrast > 0 || contrast < 255)
	{
		command(SETCONTRAST) ;			// 0x81
		command(contrast) ;
	}
}

/** \brief Get LCD height.

    The height of the LCD return as byte.
*/
uint8_t microOLED::getLCDHeight(void) 
{
	return LCDHEIGHT;
}

/** \brief Get LCD width.

    The width of the LCD return as byte.
*/
uint8_t microOLED::getLCDWidth(void) 
{
	return LCDWIDTH;
}

/** \brief Get font width.

    The cucrrent font's width return as byte.
*/
uint8_t microOLED::getFontWidth(void) 
{
	return fontWidth;
}

/** \brief Get font height.

    The current font's height return as byte.
*/

uint8_t microOLED::getFontHeight(void) 
{
	return fontHeight;
}

/** \brief Override Arduino's Print.

    Arduino's print overridden so that we can use uView.print().
*/
size_t microOLED::writeChar(uint8_t c) 
{
	if (c == '\n') 
	{
		cursorY += fontHeight;
		cursorX  = 0;
	} 
	else if (c == '\r') 
	{
		// skip
	} 
	else 
	{
		drawChar(cursorX, cursorY, c, foreColor, drawMode);
		cursorX += fontWidth+1;
		if ((cursorX > (LCDWIDTH - fontWidth))) 
		{
			cursorY += fontHeight;
			cursorX = 0;
		}
	}
	return 1;
}

size_t microOLED::print(const char *fmt, ...)
{
	char printBuf[1024] = "";
	va_list args ;
	size_t returnValue = 0 ;
	
	va_start(args, fmt) ;
	
	vsnprintf(printBuf, 1024, fmt, args) ;
		
	va_end(args); 
	
	printString(printBuf) ;
	
	return returnValue ;
}

size_t microOLED::printString(const char *ifsh)
{
	const char *p = reinterpret_cast<const char *>(ifsh);
	size_t n = 0;
	while (1) 
	{
	    unsigned char c = pgm_read_byte(p++);
		if (c == 0) 
			break;

		if (writeChar(c)) 
			n++;
    else 
		break;
	}
  return n;
}

/** \brief Draw character.

    Draw character c using current color and current draw mode at x,y.
*/
void  microOLED::drawChar(uint8_t x, uint8_t y, uint8_t c) 
{
	drawChar(x,y,c,foreColor,drawMode);
}

/** \brief Draw character with color and mode.

    Draw character c using color and draw mode at x,y.
*/
void  microOLED::drawChar(uint8_t x, uint8_t y, uint8_t c, 
						  uint8_t color, uint8_t mode) 
{
	// TODO - New routine to take font of any height, at the moment limited to font height in multiple of 8 pixels

	uint8_t rowsToDraw,row, tempC;
	uint8_t i,j,temp;
	uint16_t charPerBitmapRow,charColPositionOnBitmap,charRowPositionOnBitmap,charBitmapStartPosition;

	if ((c<fontStartChar) || (c>(fontStartChar+fontTotalChar-1)))		// no bitmap for the required c
		return;

	tempC=c-fontStartChar;

	// each row (in datasheet is call page) is 8 bits high, 16 bit high character will have 2 rows to be drawn
	rowsToDraw=fontHeight/8;	// 8 is LCD's page size, see SSD1306 datasheet
	if (rowsToDraw<=1) rowsToDraw=1;

	// the following draw function can draw anywhere on the screen, but SLOW pixel by pixel draw
	if (rowsToDraw==1) 
	{
		for  (i=0;i<fontWidth+1;i++) 
		{
			if (i==fontWidth) // this is done in a weird way because for 5x7 font, there is no margin, this code add a margin after col 5
				temp=0;
			else
			{
				//temp=pgm_read_byte(fontsPointer[fontType]+FONTHEADERSIZE+
				//		(tempC*fontWidth)+i);
				switch(fontType)
				{
					case 0:
						temp=font5x7[FONTHEADERSIZE+(tempC*fontWidth)+i] ;
						break ;
					case 1:
						temp=font8x16[FONTHEADERSIZE+(tempC*fontWidth)+i] ;
						break ;
					case 2:
						temp=sevensegment[FONTHEADERSIZE+(tempC*fontWidth)+i] ;
						break ;
					case 3:
						temp=fontlargenumber[FONTHEADERSIZE+(tempC*fontWidth)+i] ;
						break ;
				}
						
			}
			
			for (j=0;j<8;j++) 
			{			
				// 8 is the LCD's page height (see datasheet for explanation)
				if (temp & 0x1) 
				{
					pixel(x+i, y+j, color,mode);
				}
				else 
				{
					pixel(x+i, y+j, !color,mode);
				}

				temp >>=1;
			}
		}
		return;

	}

	// font height over 8 bit
	// take character "0" ASCII 48 as example
	charPerBitmapRow=fontMapWidth/fontWidth;  // 256/8 =32 char per row
	charColPositionOnBitmap=tempC % charPerBitmapRow;  // =16
	charRowPositionOnBitmap=int(tempC/charPerBitmapRow); // =1
	charBitmapStartPosition=(charRowPositionOnBitmap * fontMapWidth 
							 * (fontHeight/8)) + (charColPositionOnBitmap 
							 * fontWidth) ;

	// each row on LCD is 8 bit height (see datasheet for explanation)
	for(row=0;row<rowsToDraw;row++) 
	{
		for (i=0; i<fontWidth;i++) 
		{
			/*temp=pgm_read_byte(fontsPointer[fontType]+
							   FONTHEADERSIZE+(charBitmapStartPosition+i+
							   (row*fontMapWidth)));
			 */
			switch(fontType)
				{
					case 0:
						temp=font5x7[FONTHEADERSIZE+(charBitmapStartPosition+
									 i+(row*fontMapWidth))] ;
						break ;
					case 1:
						temp=font8x16[FONTHEADERSIZE+(charBitmapStartPosition+
									  i+(row*fontMapWidth))] ;
						break ;
					case 2:
						temp=sevensegment[FONTHEADERSIZE+
										  (charBitmapStartPosition+
										  i+(row*fontMapWidth))] ;
						break ;
					case 3:
						temp=fontlargenumber[FONTHEADERSIZE+
											 (charBitmapStartPosition+
											 i+(row*fontMapWidth))] ;
						break ;
				}
			for (j=0;j<8;j++) 
			{			
				// 8 is the LCD's page height (see datasheet for explanation)
				if (temp & 0x1) 
				{
					pixel(x+i,y+j+(row*8), color, mode);
				}
				else 
				{
					pixel(x+i,y+j+(row*8), !color, mode);
				}
				temp >>=1;
			}
		}
	}
}

/** \brief Draw rectangle.

Draw rectangle using current fore color and current draw mode from x,y to x+width,y+height of the screen buffer.
*/
void microOLED::rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height) 
{
	rect(x,y,width,height,foreColor,drawMode);
}

/** \brief Draw rectangle with color and mode.

Draw rectangle using color and mode from x,y to x+width,y+height of the screen buffer.
*/
void microOLED::rect(uint8_t x, uint8_t y, uint8_t width, uint8_t height, 
					 uint8_t color , uint8_t mode) 
{
	uint8_t tempHeight;

	lineH(x,y, width, color, mode);
	lineH(x,y+height-1, width, color, mode);

	tempHeight=height-2;

	// skip drawing vertical lines to avoid overlapping of pixel that will
	// affect XOR plot if no pixel in between horizontal lines
	if (tempHeight<1) 
		return;

	lineV(x,y+1, tempHeight, color, mode);
	lineV(x+width-1, y+1, tempHeight, color, mode);
}

/** \brief Draw horizontal line.

Draw horizontal line using current fore color and current draw mode from x,y 
to x+width,y of the screen buffer.
*/
void microOLED::lineH(uint8_t x, uint8_t y, uint8_t width) 
{
	line(x,y,x+width,y,foreColor,drawMode);
}

/** \brief Draw horizontal line with color and mode.

Draw horizontal line using color and mode from x,y to x+width,y 
of the screen buffer.
*/
void microOLED::lineH(uint8_t x, uint8_t y, uint8_t width, uint8_t color, 
					  uint8_t mode) 
{
	line(x,y,x+width,y,color,mode);
}

/** \brief Draw vertical line.

Draw vertical line using current fore color and current draw mode from x,y 
to x,y+height of the screen buffer.
*/
void microOLED::lineV(uint8_t x, uint8_t y, uint8_t height) 
{
	line(x,y,x,y+height,foreColor,drawMode);
}

/** \brief Draw vertical line with color and mode.

Draw vertical line using color and mode from x,y to x,y+height 
of the screen buffer.
*/
void microOLED::lineV(uint8_t x, uint8_t y, uint8_t height, uint8_t color, 
					  uint8_t mode) 
{
	line(x,y,x,y+height,color,mode);
}

/** \brief Draw filled rectangle.

Draw filled rectangle using current fore color and current draw mode 
from x,y to x+width,y+height of the screen buffer.
*/
void microOLED::rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height) 
{
	rectFill(x,y,width,height,foreColor,drawMode);
}

/** \brief Draw filled rectangle with color and mode.

Draw filled rectangle using color and mode from x,y to x+width,y+height 
of the screen buffer.
*/
void microOLED::rectFill(uint8_t x, uint8_t y, uint8_t width, uint8_t height, 
						 uint8_t color , uint8_t mode) 
{
	// TODO - need to optimise the memory map draw so that this function will not call pixel one by one
	for (int i=x; i<x+width;i++) 
	{
		lineV(i,y, height, color, mode);
	}
}

/** \brief Draw circle.

Draw circle with radius using current fore color and current draw mode at x,y 
of the screen buffer.
*/
void microOLED::circle(uint8_t x0, uint8_t y0, uint8_t radius) 
{
	circle(x0,y0,radius,foreColor,drawMode);
}

/** \brief Draw circle with color and mode.

Draw circle with radius using color and mode at x,y of the screen buffer.
*/
void microOLED::circle(uint8_t x0, uint8_t y0, uint8_t radius, uint8_t color, 
					   uint8_t mode) 
{
	//TODO - find a way to check for no overlapping of pixels so that XOR draw mode will work perfectly
	int8_t f = 1 - radius;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * radius;
	int8_t x = 0;
	int8_t y = radius;

	pixel(x0, y0+radius, color, mode);
	pixel(x0, y0-radius, color, mode);
	pixel(x0+radius, y0, color, mode);
	pixel(x0-radius, y0, color, mode);

	while (x<y) 
	{
		if (f >= 0) 
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		pixel(x0 + x, y0 + y, color, mode);
		pixel(x0 - x, y0 + y, color, mode);
		pixel(x0 + x, y0 - y, color, mode);
		pixel(x0 - x, y0 - y, color, mode);

		pixel(x0 + y, y0 + x, color, mode);
		pixel(x0 - y, y0 + x, color, mode);
		pixel(x0 + y, y0 - x, color, mode);
		pixel(x0 - y, y0 - x, color, mode);
	}
}

/** \brief Draw filled circle.

Draw filled circle with radius using current fore color and current draw mode 
at x,y of the screen buffer.
*/
void microOLED::circleFill(uint8_t x0, uint8_t y0, uint8_t radius) 
{
	circleFill(x0,y0,radius,foreColor,drawMode);
}

/** \brief Draw filled circle with color and mode.

Draw filled circle with radius using color and mode at x,y of the screen buffer.
*/
void microOLED::circleFill(uint8_t x0, uint8_t y0, uint8_t radius, 
						   uint8_t color, uint8_t mode) 
{
	// TODO - - find a way to check for no overlapping of pixels so that XOR draw mode will work perfectly
	int8_t f = 1 - radius;
	int8_t ddF_x = 1;
	int8_t ddF_y = -2 * radius;
	int8_t x = 0;
	int8_t y = radius;

	// Temporary disable fill circle for XOR mode.
	if (mode==XOR) 
		return;

	for (uint8_t i=y0-radius; i<=y0+radius; i++) 
	{
		pixel(x0, i, color, mode);
	}

	while (x<y) 
	{
		if (f >= 0) 
		{
			y--;
			ddF_y += 2;
			f += ddF_y;
		}
		x++;
		ddF_x += 2;
		f += ddF_x;

		for (uint8_t i=y0-y; i<=y0+y; i++) 
		{
			pixel(x0+x, i, color, mode);
			pixel(x0-x, i, color, mode);
		}
		for (uint8_t i=y0-x; i<=y0+x; i++) 
		{
			pixel(x0+y, i, color, mode);
			pixel(x0-y, i, color, mode);
		}
	}
}

/** \brief Stop scrolling.

Stop the scrolling of graphics on the OLED.
*/
void microOLED::scrollStop(void)
{
	command(DEACTIVATESCROLL);
}

/** \brief Right scrolling horizontal.

Set row start to row stop on the OLED to scroll right. 
Refer to http://learn.microview.io/intro/general-overview-of-microview.html 
for explanation of the rows.
*/
void microOLED::scrollRight(uint8_t start, uint8_t stop)
{
	if (stop<start)		// if stop is smaller then start then exit
		return;

	scrollStop();		// need to disable scrolling before starting to avoid memory corrupt
	/*command(RIGHTHORIZONTALSCROLL);
	command(0x00);	// dummy byte
	command(0x00);	// Define PAGE0 as startpage address
	command(0xFF);	// set time interval between each scroll step as 6 frames
	command(0x07);	// Define PAGE7 as endpage address
	command(0xFF);	// set scrolling offset
	//command(0xFF);
	*/
	command(RIGHTHORIZONTALSCROLL);
	command(0x00);
	command(start);		// offset from top ???
	command(0x7);		// set speed : 0x00 -> 0x0F
	command(stop);		// stop row ???
	command(0x00);		// offset on left side??? +/- 20 -> 215
	command(0xFF);
	 
	command(ACTIVATESCROLL);
}

/** \brief Left scrolling horizontal.

Set row start to row stop on the OLED to scroll right. 
Refer to http://learn.microview.io/intro/general-overview-of-microview.html 
for explanation of the rows.
*/
void microOLED::scrollLeft(uint8_t start, uint8_t stop)
{
	if (stop<start)		// if stop is smaller then start then exit
		return;

	scrollStop();		// need to disable scrolling before starting to avoid memory corrupt
	command(LEFTHORIZONTALSCROLL);
	command(0x00);
	command(start);
	command(0x7);		// scroll speed frames , TODO
	command(stop);
	command(0x00);
	command(0xFF);
	command(ACTIVATESCROLL);
}

/** \brief Right scrolling vertical.

Set row start to row stop on the OLED to scroll right. 
Refer to http://learn.microview.io/intro/general-overview-of-microview.html 
for explanation of the rows.
 
TODO : This function doesn't work yet!
*/
void microOLED::scrollVertRight(uint8_t start, uint8_t stop)
{
	if (stop<start)		// if stop is smaller then start then exit
		return;

	scrollStop();		// need to disable scrolling before starting to avoid memory corrupt
	command(VERTICALRIGHTHORIZONTALSCROLL);
	command(0x00);
	command(start);
	command(0x7);		// scroll speed frames , TODO
	command(stop);
	command(0x00);
	command(0xFF);
	command(ACTIVATESCROLL);
}

/** \brief Left scrolling vertical.

Set row start to row stop on the OLED to scroll right. 
Refer to http://learn.microview.io/intro/general-overview-of-microview.html 
for explanation of the rows.
 
TODO : This function doesn't work yet!
*/
void microOLED::scrollVertLeft(uint8_t start, uint8_t stop)
{
	if (stop<start)		// if stop is smaller then start then exit
		return;

	scrollStop();		// need to disable scrolling before starting to avoid memory corrupt
	command(VERTICALLEFTHORIZONTALSCROLL);
	command(0x00);
	command(start);
	command(0x7);		// scroll speed frames , TODO
	command(stop);
	command(0x00);
	command(0xFF);
	command(ACTIVATESCROLL);
}

/** \brief Vertical flip.

Flip the graphics on the OLED vertically.
*/
void microOLED::flipVertical(bool flip) 
{
	if (flip) 
	{
		command(COMSCANINC);
	}
	else 
	{
		command(COMSCANDEC);
	}
}

/** \brief Horizontal flip.

Flip the graphics on the OLED horizontally.
*/
void microOLED::flipHorizontal(bool flip) 
{
	if (flip) 
	{
		command(SEGREMAP | 0x0);
	}
	else 
	{
		command(SEGREMAP | 0x1);
	}
}

/*
Return a pointer to the start of the RAM screen buffer for direct access.
*/
uint8_t *microOLED::getScreenBuffer(void) 
{
	return screenmemory;
}

/*
Draw Bitmap image on screen. The array for the bitmap can be stored in the 
Arduino file, so user don't have to mess with the library files.

To use, create uint8_t array that is 64x48 pixels (384 bytes). 
Then call .drawBitmap and pass it the array.
*/
void microOLED::drawBitmap(uint8_t * bitArray)
{
  for (int i=0; i<(LCDWIDTH * LCDHEIGHT / 8); i++)
    screenmemory[i] = bitArray[i];
}

/** \brief Wait ... milliseconds
 
 Wait ... milliseconds
 */
int microOLED::waitMilliSec(float milliS)
{
	nanoseconds = milliS * 1000000 ;

	// 1 000 000 microseconds = 1 seconds
	waitTime.tv_sec = seconds ;
	waitTime.tv_nsec =  long(nanoseconds) ;	
		
	if (nanosleep(&waitTime, NULL) < 0) 
		return 0 ;
	else
		return 1 ;
}