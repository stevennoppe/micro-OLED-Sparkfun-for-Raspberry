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
 * Demo : Shows alot of different functions
 * 
 */

#include "microOLED.h"
#include <math.h>
#include <time.h>

#define PI	3.1415

#define PIN_RESET 0		// Connect RST to pin wiringPi 0 (req. for I2C & SPI)
#define PIN_DC    1		// Connect DC to pin wiringPi 1 (required for SPI)

microOLED oled(PIN_RESET, PIN_DC, 0) ;		// 0 = SPI

void pixelExample() ;
void lineExample() ;
void shapeExample() ;
void textExamples() ;
void printTitle(const char *title, int font) ;

int main(int argc, char** argv)
{
	time_t t;
	srand((unsigned) time(&t));
	
	// Before you can start using the OLED, call begin() to init
	// all of the pins and configure the OLED.
	oled.begin();
		
	// clear(ALL) will clear out the OLED's graphic memory.
	// clear(PAGE) will clear the page display buffer.
	oled.clear(ALL);  // Clear the display's memory (gets rid of artifacts)
	oled.display();  // Display what's in the buffer (splashscreen)

	oled.waitMiliSec(1000);     // Delay 1000 ms

	oled.clear(PAGE); // Clear the buffer.
		  
	while(1)
	{
		pixelExample();  // Run the pixel example function
		lineExample();   // Then the line example function
		shapeExample();  // Then the shape example
		textExamples();  // Finally the text example
	}

	return 0; 
}

void pixelExample()
{
	printTitle("Pixels!", 0);
		
	for (int i=0; i<512; i++)
	{
		oled.pixel(rand() %(oled.getLCDWidth()), rand() %(oled.getLCDHeight())) ;
		//oled.pixel(random(oled.getLCDWidth()), random(oled.getLCDHeight()));
		oled.display();
  }
}

void lineExample()
{
	int middleX = oled.getLCDWidth() / 2; // 32
	int middleY = oled.getLCDHeight() / 2; // 24

	int xEnd, yEnd;

	//int lineWidth = min(middleX, middleY);
	int lineWidth = 24 ;
	
	printTitle("Lines!", 1);
	
	for (int i=0; i<3; i++)
	{
		for (int deg=0; deg<360; deg+=15)
		{
			xEnd = lineWidth * cos(deg * PI / 180.0);
			yEnd = lineWidth * sin(deg * PI / 180.0);

			oled.line(middleX, middleY, middleX + xEnd, middleY + yEnd);
			oled.display();
			oled.waitMiliSec(10);
		}

		for (int deg=0; deg<360; deg+=15)
		{
			xEnd = lineWidth * cos(deg * PI / 180.0);
		    yEnd = lineWidth * sin(deg * PI / 180.0);

			oled.line(middleX, middleY, middleX + xEnd, middleY + yEnd, BLACK, NORM);
		    oled.display();
		    oled.waitMiliSec(10);
	    }
	}
}

void shapeExample()
{
	printTitle("Shapes!", 0);

	// Silly pong demo. It takes a lot of work to fake pong...
	int paddleW = 3;  // Paddle width
	int paddleH = 15;  // Paddle height

	// Paddle 0 (left) position coordinates
	int paddle0_Y = (oled.getLCDHeight() / 2) - (paddleH / 2);
	int paddle0_X = 2;

	// Paddle 1 (right) position coordinates
	int paddle1_Y = (oled.getLCDHeight() / 2) - (paddleH / 2);
	int paddle1_X = oled.getLCDWidth() - 3 - paddleW;

	int ball_rad = 2;  // Ball radius

	// Ball position coordinates
	int ball_X = paddle0_X + paddleW + ball_rad;
	int ball_Y = rand() %(1 + ball_rad, oled.getLCDHeight() - ball_rad);//paddle0_Y + ball_rad;
  
	int ballVelocityX = 1;  // Ball left/right velocity
	int ballVelocityY = 1;  // Ball up/down velocity
	int paddle0Velocity = -1;  // Paddle 0 velocity
	int paddle1Velocity = 1;  // Paddle 1 velocity

	//while(ball_X >= paddle0_X + paddleW - 1)
	while ((ball_X - ball_rad > 1) && 
		   (ball_X + ball_rad < oled.getLCDWidth() - 2))
	{
		// Increment ball's position
	    ball_X+=ballVelocityX;
	    ball_Y+=ballVelocityY;

		// Check if the ball is colliding with the left paddle
	    if (ball_X - ball_rad < paddle0_X + paddleW)
		{
			// Check if ball is within paddle's height
			if ((ball_Y > paddle0_Y) && (ball_Y < paddle0_Y + paddleH))
		    {
			    ball_X++;  // Move ball over one to the right
		        ballVelocityX = -ballVelocityX; // Change velocity
			}
	    }

		// Check if the ball hit the right paddle
		if (ball_X + ball_rad > paddle1_X)
	    {
			// Check if ball is within paddle's height
			if ((ball_Y > paddle1_Y) && (ball_Y < paddle1_Y + paddleH))
		    {
				ball_X--;  // Move ball over one to the left
				ballVelocityX = -ballVelocityX; // change velocity
			}
		}
		// Check if the ball hit the top or bottom
	    if ((ball_Y <= ball_rad) || (ball_Y >= (oled.getLCDHeight() - ball_rad - 1)))
	    {
			// Change up/down velocity direction
		    ballVelocityY = -ballVelocityY;

		}

		// Move the paddles up and down
	    paddle0_Y += paddle0Velocity;
		paddle1_Y += paddle1Velocity;

		// Change paddle 0's direction if it hit top/bottom
		if ((paddle0_Y <= 1) || (paddle0_Y > oled.getLCDHeight() - 2 - paddleH))
	    {
			paddle0Velocity = -paddle0Velocity;
	    }

		// Change paddle 1's direction if it hit top/bottom
	    if ((paddle1_Y <= 1) || (paddle1_Y > oled.getLCDHeight() - 2 - paddleH))
	    {
			paddle1Velocity = -paddle1Velocity;
	    }

    // Draw the Pong Field
    oled.clear(PAGE);  // Clear the page

    // Draw an outline of the screen:
    oled.rect(0, 0, oled.getLCDWidth() - 1, oled.getLCDHeight());

    // Draw the center line
    oled.rectFill(oled.getLCDWidth()/2 - 1, 0, 2, oled.getLCDHeight());

    // Draw the Paddles:
    oled.rectFill(paddle0_X, paddle0_Y, paddleW, paddleH);
    oled.rectFill(paddle1_X, paddle1_Y, paddleW, paddleH);

    // Draw the ball:
    oled.circle(ball_X, ball_Y, ball_rad);

    // Actually draw everything on the screen:
    oled.display();

    oled.waitMiliSec(25);  // Delay for visibility
  }
  oled.waitMiliSec(1000);
}

void textExamples()
{
	printTitle("Text!", 1);

	// Demonstrate font 0. 5x8 font
	oled.clear(PAGE);     // Clear the screen
	oled.setFontType(0);  // Set font to type 0
	oled.setCursor(0, 0); // Set cursor to top-left

	// There are 255 possible characters in the font 0 type.
	// Lets run through all of them and print them out!
	for (int i=0; i<=255; i++)
	{
		// You can write byte values and they'll be mapped to
		// their ASCII equivalent character.

		oled.writeChar(i);  // Write a byte out as a character
		oled.display(); // Draw on the screen
		oled.waitMiliSec(10);      // Wait 10ms

		// We can only display 60 font 0 characters at a time.
		// Every 60 characters, pause for a moment. Then clear
		// the page and start over.
		if ((i%60 == 0) && (i != 0))
		{
			oled.waitMiliSec(500);           // Delay 500 ms
			oled.clear(PAGE);     // Clear the page
			oled.setCursor(0, 0); // Set cursor to top-left
		}
	}

	oled.waitMiliSec(500);  // Wait 500ms before next example

	// Demonstrate font 1. 8x16. Let's use the print function
	// to display every character defined in this font.
	oled.setFontType(1);  // Set font to type 1
	oled.clear(PAGE);     // Clear the page
	oled.setCursor(0, 0); // Set cursor to top-left

	// Print can be used to print a string to the screen:
	oled.print(" !\"#$%&'()*+,-./01234");
	oled.display();       // Refresh the display

	oled.waitMiliSec(1000);          // Delay a second and repeat

	oled.clear(PAGE);
	oled.setCursor(0, 0);
    oled.print("56789:;<=>?@ABCDEFGHI");
	oled.display();

	oled.waitMiliSec(1000);
	
	oled.clear(PAGE);
	oled.setCursor(0, 0);
	oled.print("JKLMNOPQRSTUVWXYZ[\\]^");
	oled.display();

	oled.waitMiliSec(1000);

	oled.clear(PAGE);
	oled.setCursor(0, 0);
	oled.print("_`abcdefghijklmnopqrs");
	oled.display();

	oled.waitMiliSec(1000);

	oled.clear(PAGE);
	oled.setCursor(0, 0);
	oled.print("tuvwxyz{|}~");
	oled.display();

	oled.waitMiliSec(1000);

	// Demonstrate font 2. 10x16. Only numbers and '.' are defined. 
	// This font looks like 7-segment displays.
	// Lets use this big-ish font to display readings from the
	// analog pins.

	for (int i=0; i<25; i++)
	{
		oled.clear(PAGE);           // Clear the display
		oled.setCursor(0, 0);       // Set cursor to top-left
		oled.setFontType(0);        // Smallest font
		oled.print("A0: ");         // Print "A0"
		oled.setFontType(2);        // 7-segment font
		int number = 500 ;
		oled.print("%d", number);			// Print a0 reading
		//oled.print(analogRead(A0));  // Print a0 reading

		oled.setCursor(0, 16);       // Set cursor to top-middle-left
		oled.setFontType(0);         // Repeat
		oled.print("A1: ");
		oled.setFontType(2);
	    number = 600 ;
		oled.print("%d", number);			// Print a1 reading
		//oled.print(analogRead(A1));

		oled.setCursor(0, 32);
		oled.setFontType(0);
		oled.print("A2: ");
		oled.setFontType(2);
		number = 400 ;
		oled.print("%d", number);			// Print a2 reading
		//oled.print(analogRead(A2));

		oled.display();
	    oled.waitMiliSec(100);
	}

	// Demonstrate font 3. 12x48. Stopwatch demo.
	oled.setFontType(3);  // Use the biggest font
	int ms = 0;
	int s = 0;
	
	while (s <= 10)
	{
		oled.clear(PAGE);     // Clear the display
		oled.setCursor(0, 0); // Set cursor to top-left

		if (s < 10)
			oled.print("00");   // Print "00" if s is 1 digit

		else if (s < 100)     
			oled.print("0");    // Print "0" if s is 2 digits

		oled.print("%d", s);        // Print s's value
		oled.print(":");      // Print ":"
		oled.print("%d", ms);       // Print ms value
		oled.display();       // Draw on the screen

		ms++;         // Increment ms

		if (ms >= 10) // If ms is >= 10
	    {
			ms = 0;     // Set ms back to 0
			s++;        // and increment s
		}
	}

  // Demonstrate font 4. 31x48. Let's use the print function
  // to display some characters defined in this font.
  oled.setFontType(4);  // Set font to type 4
  oled.clear(PAGE);     // Clear the page
  oled.setCursor(0, 0); // Set cursor to top-left

  // Print can be used to print a string to the screen:
  oled.print("OL");
  oled.display();       // Refresh the display
  oled.waitMiliSec(1000);          // Delay a second and repeat
  oled.clear(PAGE);
  oled.setCursor(0, 0);
  oled.print("ED");
  oled.display();
  oled.waitMiliSec(1000);
}

// Center and print a small title
// This function is quick and dirty. Only works for titles one
// line long.
void printTitle(const char *title, int font)
{
  int middleX = oled.getLCDWidth() / 2;
  int middleY = oled.getLCDHeight() / 2;

  oled.clear(PAGE);
  oled.setFontType(font);
  // Try to set the cursor in the middle of the screen
  oled.setCursor(middleX - (oled.getFontWidth() * (strlen(title)/2)),
                 middleY - (oled.getFontHeight() / 2));

  // Print the title:
  oled.printString(title);
  oled.display();
  oled.waitMiliSec(1500);
  oled.clear(PAGE);
}