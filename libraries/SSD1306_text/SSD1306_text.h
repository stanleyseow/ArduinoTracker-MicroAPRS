/**
 *  Adafruit SSD1306 library modified by William Greiman for
 *  unbuffered LiquidCrystal character mode.
 *
 * -- Further modified by JBoyton to support character scaling
 * and to allow horizontal positioning of text to any pixel.
 * Vertical text position is still limited to a row.
 * Edited to make specific to Uno R3 and 128x64 size.
 * Also added HW SPI support.
*/

#define HW_SPI true
#define USE_SYSTEM_PRINT true

#include "Arduino.h"

#define SSD1306_LCDWIDTH  128
#define SSD1306_LCDHEIGHT  64

#define SSD1306_SETCONTRAST 0x81
#define SSD1306_DISPLAYALLON_RESUME 0xA4
#define SSD1306_DISPLAYALLON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

#define SSD1306_SETVCOMDETECT 0xDB

#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9

#define SSD1306_SETMULTIPLEX 0xA8

#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10

#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22

#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_SETSTARTPAGE 0XB0
#define SSD1306_MEMORYMODE 0x20

#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8

#define SSD1306_SEGREMAP 0xA0

#define SSD1306_CHARGEPUMP 0x8D

#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2
//------------------------------------------------------------------------------
#if USE_SYSTEM_PRINT
class SSD1306_text : public Print {
#else
class SSD1306_text {
#endif

 public:
#if HW_SPI
  SSD1306_text(int8_t dc, int8_t rst, int8_t cs)
    :dc_(dc), rst_(rst), cs_(cs) {}
#else
  SSD1306_text(int8_t data, int8_t clk, int8_t dc, int8_t rst, int8_t cs)
    :data_(data), clk_(clk), dc_(dc), rst_(rst), cs_(cs) {}
#endif

  void init();
  void clear();
  void setCursor(uint8_t row, uint8_t col);
  void setTextSize(uint8_t size, uint8_t spacing) {textSize_ = size; textSpacing_ = spacing;};
  size_t write(uint8_t c);
  size_t write(const char* s);
#if !USE_SYSTEM_PRINT
  void writeInt(int i);			// slightly smaller than system print()
#endif

  void sendCommand(uint8_t c);
  void sendData(uint8_t c);

 private:
  int8_t col_, row_;		// cursor position
  uint8_t textSize_, textSpacing_;	// text size and horiz char spacing (pixels between)
  int8_t data_, clk_, dc_, rst_, cs_;	// OLED pins

  volatile uint8_t *mosiport, *clkport, *csport, *dcport;
  uint8_t mosipinmask, clkpinmask, cspinmask, dcpinmask;
  
  void spiWrite(uint8_t c);
};

