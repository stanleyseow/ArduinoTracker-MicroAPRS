// Simple Hello world demo
// Modified by Stanley Seow <stanleyseow@gmail.com>
// For the SVTrackR 0.96" OLED
// This is a small memory text ONLY driver for SSD 1306 graphic driver
//
//
#include <SPI.h>
#include <SSD1306_text.h>
#include <stdlib.h>

// To select begtween HW SPI and soft SPI comment/uncomment the sections
// below and also change the HW_SPI definition in the library .h file.

#if HW_SPI
// Hardware SPI pins include D11=Data and D13=Clk
// Default for 0.96" SPI OLED
#define OLED_DC 7
#define OLED_CS 0
#define OLED_RST 10
SSD1306_text oled(OLED_DC, OLED_RST, OLED_CS);
#else
// Bit Twiddle "soft" SPI pin definitions
#define OLED_DATA 9
#define OLED_CLK 10
#define OLED_DC 11
#define OLED_CS 12
#define OLED_RST 13
SSD1306_text oled(OLED_DATA, OLED_CLK, OLED_DC, OLED_RST, OLED_CS);
#endif

// Methods:
//
// init()      - call once
// clear()     - clears screen
//
// setCursor(row, pixel)    - sets write location to the specified row (0 to 7) and pixel (0 to 127)
//
// setTextSize(size, spacing) - set the text size from 1 to 8 times the 5x7 font with a pixel spacing (0,1,2,3,...)
//
// write(c)    - write a single ascii character
// write(s)    - write a string
//
// print()      - system print function


//------------------------------------------------------------------------------
void setup() {

// Initialize, optionally clear the screen
    oled.init();
    oled.clear();                 // clear screen
    
// Hello world - single sized character at row 0, pixel 0    
    oled.setTextSize(1,1);        // 5x7 characters, pixel spacing = 1
    oled.write("Hello world!");

// Scaled characters, extra spacing
    oled.setCursor(3, 10);        // move cursor to row 3, pixel column 10
    oled.setTextSize(3, 8);       // 3X character size, spacing 5 pixels
    oled.write("Abc");

// Use print()
    float floatVal = 23.792;
    oled.setCursor(6,40);
    oled.setTextSize(2,1);
    oled.print(floatVal,3);
    
// Pseudo-graphics: Draw a box using direct writes  
    oled.setCursor(0, 100);
    oled.sendData(0xFF);
    for (int i=0; i<14; i++) oled.sendData(0x01);
    oled.sendData(0xFF);
    oled.setCursor(1,100);
    oled.sendData(0xFF);
    for (int i=0; i<14; i++) oled.sendData(0x80);
    oled.sendData(0xFF);
}
//------------------------------------------------------------------------------

void loop() {}

