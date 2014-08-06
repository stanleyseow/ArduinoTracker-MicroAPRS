/*
 Copyright (C) 2014 Stanley Seow <stanleyseow@gmail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 github URL :-
 https://github.com/stanleyseow/ArduinoTracker-MicroAPRS
 
 This sketch configure the MicroAPRS for the proper callsign and ssid and
 read/write data coming in from the MicroAPRS via debug port
 
 Pin 0/1 (rx,tx) connects to Arduino with MicroAPRS firmware
 Pin 8,9 ( rx,tx ) connects to GPS module
 Pin 2,3 connect to debug serial port
 
 Pin 10 - Buzzer during Radio Tx
 
 Date : 03 July 2014
 Written by Stanley Seow
 e-mail : stanleyseow@gmail.com
 Version : 0.2
 
 Pls flash the modem Arduino with http://unsigned.io/microaprs/ with USBtinyISP
 This firmware does NOT have an Arduino bootloader and run pure AVR codes. Once you
 have done this, follow the instructions on the above URL on how to set it up.
 We will call this Arduino/atmega328 as the "modem". 
 
 To use with Arduino Tracker (this sketch), connect the Modem Tx(pin1) to Arduino Rx(pin8) and 
 Modem Rx(pin0) to Arduino Rx(pin9).
 
 I had dropped bytes when using Softwaredebug for the MicroAPRS modem and therefore
 I'm using AltSoftdebug instead. The GPS module is still on Softwaredebug and 
 the hardware debug is still used for debug Monitor.
 
 ***** To use this sketch, pls change your CALLSIGN and SSID below under configModem().
 
 History :-
 03 July 2014 :-
 - Initial released
 
 06 July 2014 :-
 - added checks for speed and idle speed, modify the Txinternal
 - remove all debug Monitor output
 
 12 July 2014 :-
 - Added SmartBeaconing algorithm, Tx when turn is more than 25 deg
 - Reduce the Tx interval
 
 14 July 2014 :-
 - Fixed coordinates conversion from decimal to Deg Min formula
 - Formula to calculate distance from last Tx point so that it will Tx once the max direct distance 
  of 500m is reached
 
 18 July 2014 :-
 - Fixed lastTx checking routine and ensure lastTx is 5 or more secs
 - Check for analog0 button at least 10 secs per Tx
 - Rewrote the DecodeAPRS functions for display to LCD only
 
 1 Aug 2014 :-
 - Ported codes to Arduino Mini Pro 3.3V
 - Due to checksum errors and Mini Pro 3.3V 8Mhz on SoftwareSerial, I swapped GPS ports to AltSoftwareSerial
 
 3 Aug 2014 :-
 - Added #define codes to turn on/off LCD, DEBUG and TFT
 - Added TFT codes for 2.2" SPI TFT ( runs on 3.3V )
 - Added the F() macros to reduce memory usages on static texts
 
 6 Aug 2014
 - Added Course/Speed/Alt into comment field readable by aprs.fi
 - 
 
 TODO :-
 - implement compression / decompression codes for smaller Tx packets
 - Telemetry packets
 - Split comments and location for smaller Tx packets
 
 Bugs :-
 
 - Not splitting callsign and info properly
 - Packets received from Modem is split into two serial read
 - With TFT codes, the AltSoftSerial could not get the speed and course on time with lots of checksum errors
 - 
 
*/

// Needed this to prevent compile error for #defines
#if 1
__asm volatile ("nop");
#endif

// Turn on/off 20x4 LCD
#undef LCD20x4
// Turn on/off debug
#define DEBUG
// Turn on/off 2.2" TFT
#undef TFT22

#ifdef TFT22
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9340.h>
#define _sclk 13
#define _miso 12
#define _mosi 11
#define _cs 7
#define _dc 6
#define _rst 5
Adafruit_ILI9340 tft = Adafruit_ILI9340(_cs, _dc, _rst);
#endif

#ifdef DEBUG 
#include <SoftwareSerial.h>
#endif

#include <AltSoftSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal.h>



#ifdef LCD20x4
// My wiring for LCD on breadboard
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);
#endif

#define VERSION "SVTrackR v0.4 " 

// Altdebug default on UNO is 8-Rx, 9-Tx
//AltSoftSerial ss;
AltSoftSerial ss(8,9);
TinyGPSPlus gps;

#ifdef DEBUG
// Connect to GPS module on pin 9, 10 ( Rx, Tx )
SoftwareSerial debug(2,3);
#endif

// Pin for Tx buzzer
const byte buzzerPin = 4;
unsigned int txCounter = 0;
unsigned long txTimer = 0;
long lastTx = 0;
long txInterval = 60000L;  // Initial 60 secs internal

int lastCourse = 0;
byte lastSpeed = 0;

int previousHeading, currentHeading = 0;
// Initial lat/lng pos, change to your base station coordnates
float lastTxLat = 3.16925, lastTxLng =  101.64972;
float lastTxdistance, homeDistance = 0.0;

const unsigned int MAX_DEBUG_INPUT = 30;

void setup()
{
#ifdef LCD20x4  
  // LCD format is Col,Row for 20x4 LCD
  lcd.begin(20,4);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Arduino OpenTrackR"); 
#endif  

#ifdef TFT22
  tft.begin();    
  tft.setRotation(1);
  tft.setCursor(0,0);
  tft.setTextSize(3); 
  tft.setTextColor(ILI9340_WHITE);  
  tft.print("OpenTrackR"); 
  delay(1000);
  tft.fillScreen(ILI9340_BLACK);

#endif

// This is for buzzer
//pinMode(4, OUTPUT);
  
  Serial.begin(9600);
#ifdef DEBUG
  debug.begin(9600);
#endif
  ss.begin(9600);

#ifdef DEBUG
  debug.flush();
  debug.println();
  debug.println();
  debug.println(F("=========================================="));
  debug.print(F("DEBUG:- ")); 
  debug.println(F(VERSION)); 
  debug.println(F("=========================================="));
  debug.println();
#endif

  // Set a delay for the MicroAPRS to boot up before configuring it
  delay(1000);
  configModem();
  
  txTimer = millis();
 
} // end setup()


///////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
    // Speed in km/h
    byte highSpeed = 70;       // High speed  
    byte lowSpeed = 30;        // Low speed
    char c;
    boolean inputComplete = false;
    int headingDelta = 0;
    const double HOME_LAT = 3.16925 , HOME_LON = 101.64972;
    
#ifdef DEBUG    
    // Send commands from debug serial into hw Serial char by char 
    debug.listen();
    if ( debug.available() > 0  ) {
          processIncomingDebug(debug.read());
    }
#endif    
    
    // Turn on listen() on GPS
    ss.listen();                    
    while ( ss.available() > 0 ) {
      gps.encode(ss.read());
    }
    
///////////////// Triggered by location updates /////////////////////// 
   if ( gps.location.isUpdated() ) { 

     homeDistance = TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          HOME_LAT, 
          HOME_LON);   
          
    lastTxdistance = TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          lastTxLat,
          lastTxLng);
     
    } // endof gps.location.isUpdated()

///////////////// Triggered by time updates /////////////////////// 
// Update LCD every second

   if ( gps.time.isUpdated() ) {   
    
   // Turn on LED 13 when Satellites more than 3  
   // Disable when using TFT / SPI
   if ( gps.satellites.value() > 3 ) {
     digitalWrite(13,HIGH);  
   } else {
     digitalWrite(13,LOW);     
   }

#ifdef TFT22
      //tft.fillScreen(ILI9340_BLACK);
      tft.setCursor(0,0);

      tft.setTextSize(3);      
      tft.setTextColor(ILI9340_CYAN);  
      tft.print("9W2");    
      tft.setTextColor(ILI9340_YELLOW);   
      tft.print("SVT");  
      
      tft.setTextColor(ILI9340_RED);   
      tft.print("APRS");  
      
      tft.setTextColor(ILI9340_GREEN);   
      tft.println("TrackR");  

      tft.setCursor(0,22);
      tft.setTextSize(2);
      tft.setTextColor(ILI9340_WHITE);  
      tft.print("Date:");
      tft.setTextColor(ILI9340_YELLOW);
      tft.print(gps.date.value());
      tft.setTextColor(ILI9340_WHITE);
      tft.print(" Time:");
      tft.setTextColor(ILI9340_YELLOW);
      tft.println(gps.time.value());
      
      tft.setCursor(0,32);
      tft.setTextSize(3);
      tft.setTextColor(ILI9340_WHITE);
      tft.print("Lat:");
      tft.setTextColor(ILI9340_GREEN);  
      tft.println(gps.location.lat(),5);

      tft.setCursor(0,52);      
      tft.setTextColor(ILI9340_WHITE);     
      tft.print("Lng:");
      tft.setTextColor(ILI9340_GREEN);  
      tft.println(gps.location.lng(),5);

      tft.setCursor(0,100);
      tft.setTextSize(2);      
      tft.setTextColor(ILI9340_WHITE);  
      tft.print("Sats:");
      tft.setTextColor(ILI9340_CYAN);
      tft.print(gps.satellites.value());
      
      tft.setTextColor(ILI9340_WHITE); 
      tft.print(" HDOP:");
      tft.setTextColor(ILI9340_CYAN);
      tft.print(gps.hdop.value());
      
      tft.setTextColor(ILI9340_WHITE); 
      tft.print(" Alt:");
      tft.setTextColor(ILI9340_CYAN);
      tft.println((int)gps.altitude.meters());

      tft.setCursor(0,100);
      tft.setTextSize(2);      
      tft.setTextColor(ILI9340_WHITE); 
      tft.print("Speed:");
      tft.setTextColor(ILI9340_CYAN);
      tft.print(gps.speed.kmph());

      tft.setTextColor(ILI9340_WHITE); 
      tft.print(" Deg:");
      tft.setTextColor(ILI9340_CYAN);
      tft.println(gps.course.deg());
      
      tft.setTextSize(1);      
      tft.setTextColor(ILI9340_WHITE); 
      tft.print("Passed:");
      tft.setTextColor(ILI9340_CYAN);
      tft.print(gps.passedChecksum()); 

      tft.setTextColor(ILI9340_WHITE); 
      tft.print(" Failed:");
      tft.setTextColor(ILI9340_CYAN);
      tft.println(gps.failedChecksum());       
#endif

#ifdef LCD20x4
     lcd.setCursor(1,0);
     lcd.print("   ");
     lcd.setCursor(1,0);
     lcd.print(txCounter);

     lcd.setCursor(4,0);
     lcd.print("       ");
     lcd.setCursor(4,0);
     lcd.print(lastTx);
     
     lcd.setCursor(10,0);
     lcd.print("    ");  
     lcd.setCursor(10,0);
     lcd.print((int)lastTxdistance);    

     lcd.setCursor(14,0);
     lcd.print("    ");  
     lcd.setCursor(14,0);
     lcd.print((float)homeDistance/1000,1); 
     
     lcd.setCursor(18,0);   
     lcd.print("  "); 
     lcd.setCursor(18,0);   
     lcd.print(gps.satellites.value());
#endif
     
// Change the Tx internal based on the current speed
// This change will not affect the countdown timer
// Based on HamHUB Smart Beaconing(tm) algorithm

      if ( ((int) gps.speed.kmph()) < 5 ) {
            txInterval = 300000;         // Change Tx internal to 5 mins
       } else if ( ((int) gps.speed.kmph()) < lowSpeed ) {
            txInterval = 60000;          // Change Tx interval to 60
       } else if ( ((int) gps.speed.kmph()) > highSpeed ) {
            txInterval = 30000;          // Change Tx interval to 30 secs
       } else {
        // Interval inbetween low and high speed 
            txInterval =  (int) ( highSpeed / (int) gps.speed.kmph() ) * 30000;       
       } // endif
      
   }  // endof gps.time.isUpdated()
     
    
///////////////// Triggered by course updates /////////////////////// 
     
    if ( gps.course.isUpdated() ) {
      
      // Get headings and heading delta
      currentHeading = (int) gps.course.deg();
      if ( currentHeading >= 180 ) { currentHeading = currentHeading-180; }
      
      headingDelta = (int) ( previousHeading - currentHeading ) % 360;
    } // endof gps.course.isUpdated()

          
 ////////////////////////////////////////////////////////////////////////////////////
 // Check for when to Tx packet
 ////////////////////////////////////////////////////////////////////////////////////
 
  lastTx = 0;
  lastTx = millis() - txTimer;

    if ( (lastTx > 5000)  && (gps.satellites.value() > 3) ) {
        // Check for heading more than 25 degrees
        if ( headingDelta < -25 || headingDelta >  25 ) {
#ifdef DEBUG          
            //debug.println();        
            //debug.print(F("Heading, current:"));      
            //debug.print(currentHeading);
            //debug.print(F(" previous:"));      
            //debug.print(previousHeading);
            //debug.print(F(" delta:"));      
            //debug.println(headingDelta);        
            debug.print(F("*** Heading Change "));
            debug.println(txCounter); 
#endif            
#ifdef LCD20x4            
            lcd.setCursor(0,0);
            lcd.print("H");  
#endif            
            TxtoRadio();
            previousHeading = currentHeading;
            // Reset the txTimer & lastTX for the below if statements
            txTimer = millis(); 
            lastTx = millis() - txTimer;
        } // endif headingDelta
    } // endif lastTx > 5000
    
    if ( (lastTx > 10000) && (gps.satellites.value() > 3) ) {
         // check of the last Tx distance is more than 500m
         if ( lastTxdistance > 500 ) {  
#ifdef DEBUG                     
            debug.println();
            debug.print(F("*** Distance > 500m ")); 
            debug.println(txCounter);  
            debug.print(F("lastTxdistance:"));
            debug.print(lastTxdistance);
#endif          
#ifdef LCD20x4                        
            lcd.setCursor(0,0);
            lcd.print("D");
#endif            
            TxtoRadio();
            lastTxdistance = 0;   // Ensure this value is zero before the next Tx
            // Reset the txTimer & lastTX for the below if statements            
            txTimer = millis(); 
            lastTx = millis() - txTimer;
         } // endif lastTxdistance
    } // endif lastTx > 10000
    
    if ( (lastTx >= txInterval) && ( gps.satellites.value() > 3) ) {
        // Trigger Tx Tracker when Tx interval is reach 
        // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
        if ( lastTxdistance > 20 ) {
#ifdef DEBUG                    
                   debug.println();
                   debug.print(F("lastTx:"));
                   debug.print(lastTx);
                   debug.print(F(" txInterval:"));
                   debug.print(txInterval);     
                   debug.print(F(" lastTxdistance:"));
                   debug.println(lastTxdistance);               
                   debug.print(F("*** txInterval "));  
                   debug.println(txCounter);  

#endif                   
#ifdef LCD20x4            
                   lcd.setCursor(0,0);
                   lcd.print("T");    
#endif                   
                   TxtoRadio(); 
                   
                   // Reset the txTimer & lastTX for the below if statements   
                   txTimer = millis(); 
                   lastTx = millis() - txTimer;
        } // endif lastTxdistance > 20 
    } // endif of check for lastTx > txInterval

    // Check if the analog0 is plugged into 5V and more than 10 secs
    if ( analogRead(0) > 700 && (lastTx > 10000) ) {
#ifdef DEBUG                
                debug.println();             
                debug.println(analogRead(0));
                debug.print(F("*** Button ")); 
                debug.println(txCounter);  
 
#endif                
#ifdef LCD20x4                            
                lcd.setCursor(0,0);
                lcd.print("B");     
#endif                     
                TxtoRadio(); 
                // Reset the txTimer & lastTX for the below if statements  
                txTimer = millis(); 
                lastTx = millis() - txTimer;
     } // endif check analog0


     
} // end loop()

///////////////////////////////////////////////////////////////////////////////////////////////////////

void serialEvent() {
    decodeAPRS(); 
}

void TxtoRadio() {
  
     char tmp[10];
     float latDegMin, lngDegMin = 0.0;
     String latOut, lngOut, cmtOut = "";
  
     txCounter++;
     
     lastTxLat = gps.location.lat();
     lastTxLng = gps.location.lng();

     if ( lastTx >= 5000 ) {
#ifdef DEBUG                 
       debug.print("Time/Date: ");
//       debug.print(gps.date.value());
       byte hour = gps.time.hour() +8; // GMT+8 is my timezone
       byte day = gps.date.day();
       
       if ( hour > 23 ) {
           hour = hour -24;
           day++;
       }  
       if ( hour < 10 ) {
       debug.print("0");       
       }       
       debug.print(hour);         
       debug.print(":");  
       if ( gps.time.minute() < 10 ) {
       debug.print("0");       
       }
       debug.print(gps.time.minute());
       debug.print(":");
       if ( gps.time.second() < 10 ) {
       debug.print("0");       
       }       
       debug.print(gps.time.second());
       debug.print(" ");

       if ( day < 10 ) {
       debug.print("0");
       }         
       debug.print(day);
       debug.print("/");
       if ( gps.date.month() < 10 ) {
       debug.print("0");       
       }
       debug.print(gps.date.month());
       debug.print("/");
       debug.print(gps.date.year());
       debug.println();

       debug.print("GPS: ");
       debug.print(lastTxLat,5);
       debug.print(" ");
       debug.print(lastTxLng,5);
       debug.println();

       debug.print("Sat:");           
       debug.print(gps.satellites.value());
       debug.print(" HDOP:");
       debug.print(gps.hdop.value());
       debug.print(" km/h:");           
       debug.print((int) gps.speed.kmph());
       debug.print(" Head:");
       debug.print(currentHeading);          
       debug.print(" Alt:");
       debug.print(gps.altitude.meters());
       debug.print("m");
       debug.println();

       debug.print(F("Distance(m): Home:"));
       debug.print(homeDistance,2);
       debug.print(" Last:");  
       debug.print(lastTxdistance,2);
       debug.println(); 
     
       debug.print(F("Tx since "));  
       debug.print((float)lastTx/1000); 
       debug.println(" sec");   
#endif             
       // Turn on the buzzer
       digitalWrite(buzzerPin,HIGH);  
          
       latDegMin = convertDegMin(lastTxLat);
       lngDegMin = convertDegMin(lastTxLng);

       dtostrf(latDegMin, 2, 2, tmp );
       latOut.concat("lla0");      // set latitute command with the 0
       latOut.concat(tmp);
       latOut.concat("N");
     
       dtostrf(lngDegMin, 2, 2, tmp );
       lngOut.concat("llo");       // set longtitute command
       lngOut.concat(tmp);
       lngOut.concat("E");
     
       cmtOut.concat("@");
       cmtOut.concat(padding((int)currentHeading,3));
       cmtOut.concat("/");
       cmtOut.concat(padding((int)gps.speed.mph(),3));
       cmtOut.concat("/A=");
       cmtOut.concat(padding((int)gps.altitude.feet(),6));
       cmtOut.concat(" ");
       cmtOut.concat(VERSION);       
       cmtOut.concat((float) readVcc()/1000);
       cmtOut.concat("V ");
       cmtOut.concat(gps.hdop.value());
       cmtOut.concat("/");
       cmtOut.concat(gps.satellites.value());
       

#ifdef DEBUG          
       debug.print("TX STR: ");
       debug.print(latOut);
       debug.print(" ");           
       debug.print(lngOut);
       debug.println();
       debug.print(cmtOut);  
       debug.println(); 
#endif     

       Serial.println(latOut);
       delay(300);
       Serial.println(lngOut);
       delay(300);
       Serial.println(cmtOut);
       delay(300);
                 
       digitalWrite(buzzerPin,LOW);     
       // Reset the txTimer & Tx internal   
    
       txInterval = 60000;
       lastTx = 0;
#ifdef DEBUG               
       debug.print(F("FreeRAM:"));
       debug.print(freeRam());
       debug.print(" Uptime:");
       debug.println((float) millis()/1000);
       debug.println(F("=========================================="));
#endif     
     
     } // endif lastTX
     
} // endof TxtoRadio()

///////////////////////////////////////////////////////////////////////////////////////////////////////

void processDebugData(const char * data) {
  // Send commands to modem
  Serial.println(data);
}  // end of processDebugData
  
///////////////////////////////////////////////////////////////////////////////////////////////////////
  

void processIncomingDebug(const byte inByte) {

  static char input_line [MAX_DEBUG_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
    {
    case '\n':   // end of text
      input_line[input_pos] = 0;  // terminating null byte 
      // terminator reached! Process the data
      processDebugData(input_line);
      // reset buffer for next time
      input_pos = 0;  
      break;
    case '\r':   // discard carriage return
      break;
    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_DEBUG_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;
    }  // end of switch
   
} // endof processIncomingByte  
  
///////////////////////////////////////////////////////////////////////////////////////////////////////


void configModem() {
// Functions to configure the callsign, ssid, path and other settings
// c<callsign>
// sc<ssid>
// pd0 - turn off DST display
// pp0 - turn on PATH display

/*
#ifdef LCD20x4                            
  lcd.setCursor(0,0);
  lcd.print("Configuring modem");
  lcd.setCursor(0,0);
  lcd.print("Setting callsig");
#endif                            
*/

  Serial.println("c9W2SVT");  // Set SRC Callsign ,PLEASE CHANGE THIS TOI YOUR CALLSIGN
  delay(200);
  Serial.println("sc8");      // Set SRC SSID
  delay(200);
  Serial.println("pd0");      // Disable printing DST 
  delay(200);
  Serial.println("pp0");      // Disable printing PATH
  delay(200);
  Serial.println("lsn");      // Set symbol n / Bluedot
  delay(200);
  Serial.println("lts");      // Standard symbol 
  delay(200);
  Serial.println("V1");      // Silent Mode ON 
  delay(200);
  //Serial.println("H");        // Print out the Settings
  //Serial.println("S");        // Save config
  
#ifdef LCD20x4                              
  lcd.setCursor(0,0);
  lcd.print("Done................");
  delay(500); 
  lcd.clear();
#endif                             
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

void decodeAPRS() {
      // Dump whatever on the Serial to LCD line 1
      char c;
      String decoded="";
      int callIndex,callIndex2, dataIndex = 0;

      //debug.println();
      //debug.print("Entering decodeAPRS (");
      //debug.print(millis());
      //debug.println(")");
      while ( Serial.available() > 0 ) {
         c = Serial.read();
//#ifdef DEBUG         
//         debug.print(c);
//#endif         
         decoded.concat(c); 
      }   

#ifdef DEBUG   
      debug.print("Decoded Packets:");
      debug.println(decoded);
#endif   
      callIndex = decoded.indexOf("[");
      callIndex2 = decoded.indexOf("]");
      String callsign = decoded.substring(callIndex+1,callIndex2);
      
      dataIndex = decoded.indexOf("DATA:");
      String data = decoded.substring(dataIndex+6,decoded.length());

      String line1 = data.substring(0,20);
      String line2 = data.substring(21,40);
      
#ifdef LCD20x4                                  
      lcd.setCursor(0,1);
      lcd.print("                    ");
      lcd.setCursor(0,1);
      lcd.print(callsign);
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(0,2);      
      lcd.print(line1);
      lcd.setCursor(0,3);
      lcd.print("                    ");
      lcd.setCursor(0,3);
      lcd.print(line2);
#endif       

#ifdef DEBUG         
         debug.print(F("Callsign:"));
         debug.println(callsign);
         debug.print(F("Data:"));
         debug.println(data);
#endif  

#ifdef TFT22
      tft.drawLine(0,199,319,199, ILI9340_WHITE);
      tft.setCursor(0,200);
      tft.setTextSize(2);      
      tft.setTextColor(ILI9340_GREEN);  
      tft.print("Callsign:");
      tft.setTextColor(ILI9340_CYAN);
      tft.print(callsign);
      
      tft.setTextColor(ILI9340_WHITE);  
      tft.print(" Data:");
      tft.setTextColor(ILI9340_CYAN);
      tft.println(line2);   
      tft.println(line3);      
  
#endif

}

///////////////////////////////////////////////////////////////////////////////////////////////////////

float convertDegMin(float decDeg) {
  
  float DegMin;
  
  int intDeg = decDeg;
  decDeg -= intDeg;
  decDeg *= 60;
  DegMin = ( intDeg*100 ) + decDeg;
 
 return DegMin; 
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

long readVcc() {                 
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);                     // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

String padding( int number, byte width ) {
  String result;
  
  // Prevent a log10(0) = infinity
  int temp = number;
  if (!temp) { temp++; }
    
  for ( int i=0;i<width-(log10(temp))-1;i++) {
       result.concat('0');
  }
  result.concat(number);
  return result;
}

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

