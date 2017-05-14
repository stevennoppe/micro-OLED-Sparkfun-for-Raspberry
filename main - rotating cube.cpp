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
 * Rotating cube : this will show a 3D rotating cube 
 * 
 */

#include "microOLED.h"
#include <math.h>			// needed for rotating cube

#define PIN_RESET 0		// Connect RST to pin wiringPi 0 (req. for I2C & SPI)
#define PIN_DC    1		// Connect DC to pin wiringPi 1 (required for SPI)

microOLED oled(PIN_RESET, PIN_DC, 0) ;		// 0 = SPI

// rotating cube
#define PI	3.1415
void drawCube() ;

int SCREEN_WIDTH = 64 ;
int SCREEN_HEIGHT = 48 ;

float d = 3;
float px[] = 
{ 
	-d,  d,  d, -d, -d,  d,  d, -d 
} ;

float py[] = 
{ 
	-d, -d,  d,  d, -d, -d,  d,  d 
} ;

float pz[] = 
{ 
	-d, -d, -d, -d,  d,  d,  d,  d 
} ;

float p2x[] = 
{
	0,0,0,0,0,0,0,0
} ;

float p2y[] = 
{
	0,0,0,0,0,0,0,0
} ;

float r[] = 
{
	0,0,0
} ;

#define SHAPE_SIZE 600

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
	oled.clear(PAGE) ;// Clear the page memory
	
	while (1)
	{
		drawCube() ;
	}
	
	return 0; 
}

void drawCube()
{
	r[0]=r[0]+PI/180.0; // Add a degree
	r[1]=r[1]+PI/180.0; // Add a degree
	r[2]=r[2]+PI/180.0; // Add a degree

	if (r[0] >= 360.0*PI/180.0) 
		r[0] = 0;

	if (r[1] >= 360.0*PI/180.0) 
		r[1] = 0;

	if (r[2] >= 360.0*PI/180.0) 
		r[2] = 0;

	for (int i=0;i<8;i++)
	{
		float px2 = px[i];
		float py2 = cos(r[0])*py[i] - sin(r[0])*pz[i];
		float pz2 = sin(r[0])*py[i] + cos(r[0])*pz[i];

	    float px3 = cos(r[1])*px2 + sin(r[1])*pz2;
	    float py3 = py2;
	    float pz3 = -sin(r[1])*px2 + cos(r[1])*pz2;

		float ax = cos(r[2])*px3 - sin(r[2])*py3;
		float ay = sin(r[2])*px3 + cos(r[2])*py3;
		float az = pz3-150;

		p2x[i] = SCREEN_WIDTH/2+ax*SHAPE_SIZE/az;
		p2y[i] = SCREEN_HEIGHT/2+ay*SHAPE_SIZE/az;
	}

	oled.clear(PAGE);

	for (int i=0;i<3;i++) 
	{
		oled.line(p2x[i],p2y[i],p2x[i+1],p2y[i+1]);
		oled.line(p2x[i+4],p2y[i+4],p2x[i+5],p2y[i+5]);
	    oled.line(p2x[i],p2y[i],p2x[i+4],p2y[i+4]);
	}    

	oled.line(p2x[3],p2y[3],p2x[0],p2y[0]);
	oled.line(p2x[7],p2y[7],p2x[4],p2y[4]);
	oled.line(p2x[3],p2y[3],p2x[7],p2y[7]);
	oled.display();
}
