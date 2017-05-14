
 * Micro OLED breakout - example rotating cube in SPI mode
 *
 * Connection on a Raspberry Pi 3 model B v1.2 :
 * - 3V3		-> 3,3V (pin 1 on RPi)
 * - GND		-> GND (pin 6 on RPi)
 * - D1 		-> GPIO10 - SPI0_MOSI (pin 19 on RPi)
 * - D0			-> GPIO11 - SPI0_SCLK (pin 23 on RPi)
 * - D2			-> not connected
 * - D/C		-> GPIO18 (pin 12 on RPi) - wiringPi 1
 * - RST		-> GPIO17 (pin 11 on RPi) - wiringPi 0
 * - CS			-> GPIO8 - SPI0_CE0_N (pin 24 on RPi)
 *
 * Compiled in Netbeans IDE 8.1
 * set the following option under project properties ->
 * build -> linker -> libraries :
 * -I/usr/local/include -L/usr/local/lib -lwiringPi -lpthread
 */

/* 
 * These examples came from github repository 
 * https://github.com/sparkfun/Micro_OLED_Breakout
 * 
 * The library I used was originally for the Arduino, which I modified 
 * to work on a Raspberry Pi
 *
 * Show text : this will show text and a decimal number in the middle 
 * of the OLED. 
 * 
 */

#include "microOLED.h"

#define PIN_RESET 0		// Connect RST to pin wiringPi 0 (req. for I2C & SPI)
#define PIN_DC    1		// Connect DC to pin wiringPi 1 (required for SPI)

microOLED oled(PIN_RESET, PIN_DC, 0) ;		// 0 = SPI

int main(int argc, char** argv)
{
	// Before you can start using the OLED, call begin() to init
	// all of the pins and configure the OLED.
	oled.begin();
		
	// clear(ALL) will clear out the OLED's graphic memory.
	// clear(PAGE) will clear the page display buffer.
	oled.clear(ALL);  // Clear the display's memory (gets rid of artifacts)
	oled.clear(PAGE) ;
		  
	// prints a text with a decimal number on the OLED
	oled.setFontType(0);
	
	int middleX = oled.getLCDWidth() / 2;
  	int middleY = oled.getLCDHeight() / 2;
	int textLength = 8 ;		// 10 charcaters in "test 123"
	int number = 123 ;
	
	// Try to set the cursor in the middle of the screen
	oled.setCursor(middleX - (oled.getFontWidth() * (textLength/2)),
                   middleY - (oled.getFontHeight() / 2));
  
	oled.print("test %d", number);
	oled.display();
  
	return 0; 
}
