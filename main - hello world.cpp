/*
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
 * Hello World : this will show the sparkfun logo 
 * 
 */

#include "microOLED.h"

#define PIN_RESET 0		// Connect RST to pin wiringPi 0 (req. for I2C & SPI)
#define PIN_DC    1		// Connect DC to pin wiringPi 1 (required for SPI)

microOLED oled(PIN_RESET, PIN_DC, 0) ;		// 0 = SPI

int main(int argc, char** argv)
{
	// These three lines of code are all you need to initialize the
	// OLED and print the splash screen.

	// Before you can start using the OLED, call begin() to init
	// all of the pins and configure the OLED.
	oled.begin();
		
	// clear(ALL) will clear out the OLED's graphic memory.
	// clear(PAGE) will clear the Arduino's display buffer.
	oled.clear(ALL);  // Clear the display's memory (gets rid of artifacts)
	
	// To actually draw anything on the display, you must call the
	// display() function. 
	oled.display() ;   
	
	// the display() function will show the Sparfun logo, because it's encoded 
	// in the library. To clear the memory of the Sparkfun logo call the
	// clear(PAGE) function before display()
	return 0; 
}
