// some of this code was written by <cstone@pobox.com> originally; 
// it is in the public domain.
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
#include <avr/pgmspace.h>
#include <SSD1306_text.h>
#include "ssdfont.h"
#include <SPI.h>

//------------------------------------------------------------------------------
void SSD1306_text::init() {
  col_ = 0;
  row_ = 0;
  textSize_ = 1;
  textSpacing_ = 1;
  
  // set pin directions
  pinMode(dc_, OUTPUT);
  pinMode(rst_, OUTPUT);
  pinMode(cs_, OUTPUT);
  csport      = portOutputRegister(digitalPinToPort(cs_));
  cspinmask   = digitalPinToBitMask(cs_);
  dcport      = portOutputRegister(digitalPinToPort(dc_));
  dcpinmask   = digitalPinToBitMask(dc_);

#if  HW_SPI
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV2); // 8 MHz
#else // bit twiddle SPI
  pinMode(data_, OUTPUT);
  pinMode(clk_, OUTPUT);
  clkport     = portOutputRegister(digitalPinToPort(clk_));
  clkpinmask  = digitalPinToBitMask(clk_);
  mosiport    = portOutputRegister(digitalPinToPort(data_));
  mosipinmask = digitalPinToBitMask(data_);
#endif

  // Reset
  digitalWrite(rst_, HIGH);
  delay(1);
  digitalWrite(rst_, LOW);
  delay(10);
  digitalWrite(rst_, HIGH);

  // Init sequence for 128x64 OLED module
  sendCommand(SSD1306_DISPLAYOFF);          // 0xAE
  sendCommand(SSD1306_SETDISPLAYCLOCKDIV);  // 0xD5
  sendCommand(0x80);                        // the suggested ratio 0x80
  sendCommand(SSD1306_SETMULTIPLEX);        // 0xA8
  sendCommand(0x3F);
  sendCommand(SSD1306_SETDISPLAYOFFSET);    // 0xD3
  sendCommand(0x0);                         // no offset
  sendCommand(SSD1306_SETSTARTLINE | 0x0);  // line #0
  sendCommand(SSD1306_CHARGEPUMP);          // 0x8D
  sendCommand(0x14);
  sendCommand(SSD1306_MEMORYMODE);          // 0x20
  sendCommand(0x00);			  // was: 0x2 page mode
  sendCommand(SSD1306_SEGREMAP | 0x1);
  sendCommand(SSD1306_COMSCANDEC);
  sendCommand(SSD1306_SETCOMPINS);          // 0xDA
  sendCommand(0x12);
  sendCommand(SSD1306_SETCONTRAST);         // 0x81
  sendCommand(0xCF);
  sendCommand(SSD1306_SETPRECHARGE);        // 0xd9
  sendCommand(0xF1);
  sendCommand(SSD1306_SETVCOMDETECT);       // 0xDB
  sendCommand(0x40);
  sendCommand(SSD1306_DISPLAYALLON_RESUME); // 0xA4
  sendCommand(SSD1306_NORMALDISPLAY);       // 0xA6
  
  sendCommand(SSD1306_DISPLAYON);//--turn on oled panel
}
//------------------------------------------------------------------------------
// clear the screen
void SSD1306_text::clear() {
  sendCommand(SSD1306_COLUMNADDR);
  sendCommand(0);   // Column start address (0 = reset)
  sendCommand(SSD1306_LCDWIDTH-1); // Column end address (127 = reset)

  sendCommand(SSD1306_PAGEADDR);
  sendCommand(0); // Page start address (0 = reset)
  sendCommand(7); // Page end address

  *csport |= cspinmask;
  *dcport |= dcpinmask;
  *csport &= ~cspinmask;

  for (uint16_t i=0; i<(SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); i++) {
    spiWrite(0x00);
  }
  *csport |= cspinmask;
}
//------------------------------------------------------------------------------
void SSD1306_text::setCursor(uint8_t row, uint8_t col) {
  if (row >= SSD1306_LCDHEIGHT/8) {
    row = SSD1306_LCDHEIGHT/8 - 1;
  }
  if (col >= SSD1306_LCDWIDTH) {
    col = SSD1306_LCDWIDTH - 1;
  }
  row_ = row;	// row is 8 pixels tall; must set to byte sized row
  col_ = col;	// col is 1 pixel wide; can set to any pixel column

  sendCommand(SSD1306_SETLOWCOLUMN | (col & 0XF));
  sendCommand(SSD1306_SETHIGHCOLUMN | (col >> 4));
  sendCommand(SSD1306_SETSTARTPAGE | row);
}
//------------------------------------------------------------------------------
size_t SSD1306_text::write(uint8_t c) {
  if (textSize_ == 1) {		// dedicated code since it's 4x faster than scaling

    if (col_ >= SSD1306_LCDWIDTH) return 0;
    col_ += 7;	// x7 font
    if (c < 32 || c > 127) c = 127;
    c -= 32;
    const uint8_t *base = font + 5 * c;
    for (uint8_t i = 0; i < 5; i++ ) {
      uint8_t b =  pgm_read_byte(base + i);
      sendData(b);
    }
    for (uint8_t i=0; i<textSpacing_; i++) {
      if (col_ >= SSD1306_LCDWIDTH) break;
      col_++; 
      sendData(0);	// textSpacing_ pixels of blank space between characters
    }

  } else {                      // scale characters (up to 8X)

    uint8_t sourceSlice, targetSlice, sourceBitMask, targetBitMask, extractedBit, targetBitCount;
    uint8_t startRow = row_;
    uint8_t startCol = col_;
      
    for (uint8_t irow = 0; irow < textSize_; irow++) {
      if (row_+irow > SSD1306_LCDWIDTH - 1) break;
      if (irow > 0) setCursor(startRow+irow, startCol);
      for (uint8_t iSlice=0; iSlice<5; iSlice++) {
        sourceSlice = (uint8_t) pgm_read_byte(font + 5 * (c-32) + iSlice);
        targetSlice = 0;
        targetBitMask = 0x01;
        sourceBitMask = 0x01 << (irow*8/textSize_);
        targetBitCount = textSize_*7 - irow*8;
        do {
          extractedBit = sourceSlice & sourceBitMask;
          for (uint8_t i=0; i<textSize_; i++) {
            if (extractedBit != 0) targetSlice |= targetBitMask;
            targetBitMask <<= 1;
            targetBitCount--;
            if (targetBitCount % textSize_ == 0) {
              sourceBitMask <<= 1;
              break;
            }
            if (targetBitMask == 0) break;
          }
        } while (targetBitMask != 0);
        *csport |= cspinmask;
        *dcport |= dcpinmask;
        *csport &= ~cspinmask;
        for (uint8_t i=0; i<textSize_; i++) {
          spiWrite(targetSlice);
        }
        *csport |= cspinmask;
      }
    }
    setCursor(startRow, startCol + 5*textSize_ + textSpacing_);
  }

  return 1;

}
//------------------------------------------------------------------------------
size_t SSD1306_text::write(const char* s) {
  size_t n = strlen(s);
  for (size_t i = 0; i < n; i++) {
    write(s[i]);
  }
  return n;
}
//------------------------------------------------------------------------------
#if !USE_SYSTEM_PRINT
void SSD1306_text::writeInt(int i) {	// slighly smaller than system print()
    char buffer[7];
    itoa(i, buffer, 10);
    write(buffer);
}
#endif
//------------------------------------------------------------------------------
void SSD1306_text::sendCommand(uint8_t c) {
  *csport |= cspinmask;
  *dcport &= ~dcpinmask;
  *csport &= ~cspinmask;
  spiWrite(c);
  *csport |= cspinmask;
}
//------------------------------------------------------------------------------
void SSD1306_text::sendData(uint8_t c) {
  *csport |= cspinmask;
  *dcport |= dcpinmask;
  *csport &= ~cspinmask;
  spiWrite(c);
  *csport |= cspinmask;
}
//------------------------------------------------------------------------------
inline void SSD1306_text::spiWrite(uint8_t c) {

#if HW_SPI
    (void)SPI.transfer(c);
#else // bit twiddle SPI
    for(uint8_t bit = 0x80; bit; bit >>= 1) {
      *clkport &= ~clkpinmask;
      if(c & bit) *mosiport |=  mosipinmask;
      else        *mosiport &= ~mosipinmask;
      *clkport |=  clkpinmask;
    }
#endif
}
//------------------------------------------------------------------------------
