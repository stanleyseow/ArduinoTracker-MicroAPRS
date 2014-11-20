// Current Version
#define VERSION "SVTrackR v0.9 " 

/*

 SVTrackR ( Arduino APRS Tracker )
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
 
 Pin 4 - Buzzer during Radio Tx
 
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
 
 10 Aug 2014
 - Added support for teensy 3.1 (mcu ARM Cortex M4) 
 
 19 Aug 2014
 - Added GPS simulator codes
 - Added Tx status every 10 Tx
 - Added counter for Headins, Time, Distance and Button
 
 19 Sep 2014
 - Modify button pressed to send STATUS & position 
 - If GPS not locked, only sent out STATUS 
 
 24 Sep 2014
 - Added support for BD GPS by modifying the tinyGPSPlus 
 
 15 Oct 2014
 - Added setting PATH1 & PATH2 to the modem 
 
 24 Oct 2014 
 - Added blinking LED 13 during configuring the modem and a beep when completed
 - Minor changes in Status codes
 
 18 Nov 2014 ( v0.9 )
 - Adding OLED text only display to the Tracker
 - Moving LED to pin 6 as OLED used up SPI pins of 13,11,10,7
 - Switch top 2 lines from gps info to tracker info ( packet rx,tx)
 
 TODO :-
 - implement compression / decompression codes for smaller Tx packets
 - Telemetry packets
 
 Bugs :-
 
 - Serial issues :- Packets received from Modem is split into two serial read
 
*/

// Needed this to prevent compile error for #defines
#if 1
__asm volatile ("nop");
#endif

#ifndef _CONFIGURATION_INCLUDED
#define _CONFIGURATION_INCLUDED
#include "config.h"
#endif
 
// GPS libraries
// Choose between GPS and BD
//#include <TinyGPS++BD.h>
#include <TinyGPS++.h>
TinyGPSPlus gps;

// Turn on/off 20x4 LCD
#undef LCD20x4

// Turn on/off debug, on by default on pin 2,3
#define DEBUG

// Turn on/off GPS simulation
#undef GPSSIM

// Turn on/off 0.96" OLED
#define OLED

#ifdef DEBUG 
  #if defined(__arm__) && defined(TEENSYDUINO)
  #else
  #include <SoftwareSerial.h>
  #endif
#endif

#ifdef LCD20x4
// My wiring for LCD on breadboard
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);
#endif


// AltSoftSerial default on UNO is 8,9 (Rx,Tx)
#if defined (__AVR_ATmega328P__) 
#include <AltSoftSerial.h>
  AltSoftSerial ss(8,9);
#else
// Map hw Serial2 to ss for gps port for other platform with hw serial
  #define ss Serial2 
#endif


#ifdef DEBUG
// Connect to GPS module on pin 9, 10 ( Rx, Tx )
  #if defined (__AVR_ATmega328P__) 
    SoftwareSerial debug(2,3);
  #elif defined(__arm__) && defined(TEENSYDUINO)
    #define debug Serial3
  #endif
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Put All global defines here
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
#ifdef LCD20x4  
// Move buzzerPin to pin 10 
const byte buzzerPin = 10;
#else
// default was on pin 4 on the PCB
const byte buzzerPin = 4;
#endif

// Defines for OLED 128x64 
#ifdef OLED
#include <SPI.h>
#include <SSD1306_text.h>
#include <stdlib.h>
// Hardware SPI pins include D11=Data and D13=Clk
// DO = CLK, 13
// DI = MOSI, 11
#define OLED_DC 7 // OLED Label DC
#define OLED_CS 0 // OLED Not used
#define OLED_RST 10 // OLED Label RST
SSD1306_text oled(OLED_DC, OLED_RST, OLED_CS);
// Move ledPin to 6
const byte ledPin = 6;
#else
const byte ledPin = 13;
#endif

// Varables for Packet Decode
const unsigned int MAX_INPUT = 103;
static unsigned int packetDecoded = 0;


unsigned int txCounter = 0;
unsigned long txTimer = 0;
unsigned long lastTx = 0;
unsigned long txInterval = 80000L;  // Initial 80 secs internal

int lastCourse = 0;
byte lastSpeed = 0;
byte buttonPressed = 0;

static unsigned int Hd,Ti,Di,Bn = 0;

int previousHeading, currentHeading = 0;
// Initial lat/lng pos, change to your base station coordnates
float lastTxLat = HOME_LAT;
float lastTxLng = HOME_LON;
float lastTxdistance, homeDistance = 0.0;

// Used in the future for sending messages, commands to the tracker
const unsigned int MAX_DEBUG_INPUT = 30;

//////////////////////////////////////////////////////////////////////////////
// setup()
//////////////////////////////////////////////////////////////////////////////

void setup()
{
  
#ifdef OLED
    oled.init();
    oled.clear();                 // clear screen

    oled.setTextSize(1,1);        // 5x7 characters, pixel spacing = 1
    oled.setCursor(0,0);          // move cursor to row 1, pixel column 100
    oled.write(VERSION);
    
    oled.setTextSize(1,1);        // 5x7 characters, pixel spacing = 1
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    oled.write("by APRS Studio");  
    delay(2000);
#endif

  
#if defined(__arm__) && defined(TEENSYDUINO)
// This is for reading the internal reference voltage
  analogReference(EXTERNAL);
  analogReadResolution(12);
  analogReadAveraging(32);
#endif
  
#ifdef LCD20x4  
  // LCD format is Col,Row for 20x4 LCD
  lcd.begin(20,4);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(VERSION);
  // Insert GPS Simulator codes if defined 
  #ifdef GPSSIM     
    lcd.setCursor(0,3);
    lcd.print("GPS Sim begin"); 
    delay(1000);
    lcd.clear();
  #endif
#endif  


#ifndef LCD20x4  
  // Buzzer uses pin 4, conflicting with 20x4 LCD pins
  pinMode(buzzerPin, OUTPUT);
#endif

  // LED pin on 13, only enable for non-SPI TFT
  pinMode(ledPin,OUTPUT);

// Main serial talks to the MicroModem directly
Serial.begin(9600);

// ss talks to the GPS receiver at 9600 
ss.begin(9600);


#ifdef DEBUG
  debug.begin(9600);
#endif


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
  
#ifdef OLED  
  oled.clear();
  oledLine1();

#endif  

} // end setup()

//////////////////////////////////////////////////////////////////////////////
// loop()
//////////////////////////////////////////////////////////////////////////////

void loop()
{
    // Speed in km/h
    const byte highSpeed = 80;       // High speed  
    const byte lowSpeed = 30;        // Low speed
    char c;
    boolean inputComplete = false;
    int headingDelta = 0;
    
#ifdef DEBUG    
    // Send commands from debug serial into hw Serial char by char 

#if defined (__AVR_ATmega328P__)    
    debug.listen();
#endif  
#endif  

// Turn on listen() on GPS
#if defined (__AVR_ATmega328P__)        
    ss.listen();  
#endif    
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
          
      // Get headings and heading delta
      currentHeading = (int) gps.course.deg();
      if ( currentHeading >= 180 ) { 
      currentHeading = currentHeading-180; 
      }
      headingDelta = (int) ( previousHeading - currentHeading ) % 360;   
     
    } // endof gps.location.isUpdated()

///////////////// Triggered by time updates /////////////////////// 
// Update LCD every second

   if ( gps.time.isUpdated() ) {   
    
   if ( gps.satellites.value() > 3 ) {
     digitalWrite(ledPin,HIGH);  
   } else {
     digitalWrite(ledPin,LOW);     
   }

#ifdef OLED
      oled.setTextSize(1,1);        // 5x7 characters, pixel spacing = 1

  if ( gps.time.second() % 10 == 0 ) {
      oledLine2();
  } else if ( gps.time.second() % 5 == 0 ) {
      oledLine1();
  } 
  


#endif

#ifdef LCD20x4
     lcd.clear();
     lcd.setCursor(1,0);
     lcd.print("   ");
     //lcd.setCursor(1,0);
     //lcd.print(txCounter);

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
    
     lcd.setCursor(0,1);
     lcd.print("H:");
     lcd.print(Hd);
     lcd.print(" T:");
     lcd.print(Ti);
     lcd.print(" D:");
     lcd.print(Di);
     lcd.print(" B:");
     lcd.print(Bn);
     
     lcd.setCursor(0,2);
     lcd.print("S:");
     lcd.print((int) gps.speed.kmph());
     lcd.print(" H:");
     lcd.print((int) gps.course.deg()); 
     lcd.print(" Ttl:");
     lcd.print(txCounter);
 
     lcd.setCursor(0,3);
     lcd.print(gps.location.lat(),5);
     lcd.setCursor(10,3);
     lcd.print(gps.location.lng(),5);
     delay(30); // To see the LCD display, add a little delays here
#endif
     
// Change the Tx internal based on the current speed
// This change will not affect the countdown timer
// Based on HamHUB Smart Beaconing(tm) algorithm

      if ( gps.speed.kmph() < 5 ) {
            txInterval = 300000;         // Change Tx internal to 5 mins
       } else if ( gps.speed.kmph() < lowSpeed ) {
            txInterval = 70000;          // Change Tx interval to 60
       } else if ( gps.speed.kmph() > highSpeed ) {
            txInterval = 30000;          // Change Tx interval to 30 secs
       } else {
        // Interval inbetween low and high speed 
            txInterval = (highSpeed / gps.speed.kmph()) * 30000;       
       } // endif
      
   }  // endof gps.time.isUpdated()
               
 ////////////////////////////////////////////////////////////////////////////////////
 // Check for when to Tx packet
 ////////////////////////////////////////////////////////////////////////////////////
 
  lastTx = 0;
  lastTx = millis() - txTimer;

  // Only check the below if locked satellites < 3
#ifdef GPSSIM
   if ( gps.satellites.value() == 0 ) {
#else
   if ( gps.satellites.value() > 3 ) {
#endif    
    if ( lastTx > 5000 ) {
        // Check for heading more than 25 degrees
        if ( headingDelta < -25 || headingDelta >  25 ) {
              Hd++;
#ifdef DEBUG                
            debug.println(F("*** Heading Change "));
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
    
    if ( lastTx > 10000 ) {
         // check of the last Tx distance is more than 600m
         if ( lastTxdistance > 600 ) {  
              Di++;
#ifdef DEBUG                     
            debug.println();
            debug.println(F("*** Distance > 600m ")); 
            debug.print(F("lastTxdistance:"));
            debug.println(lastTxdistance);
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
    
    if ( lastTx >= txInterval ) {
        // Trigger Tx Tracker when Tx interval is reach 
        // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
        if ( lastTxdistance > 20 ) {
              Ti++;
#ifdef DEBUG                    
                   debug.println();
                   debug.print(F("lastTx:"));
                   debug.print(lastTx);
                   debug.print(F(" txInterval:"));
                   debug.print(txInterval);     
                   debug.print(F(" lastTxdistance:"));
                   debug.println(lastTxdistance);               
                   debug.println(F("*** txInterval "));  

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

    } // Endif check for satellites
    // Check if the analog0 is plugged into 5V and more than 10 secs
    if ( analogRead(0) > 700 && (lastTx > 10000) ) {
          Bn++;
          buttonPressed = 1;
#ifdef DEBUG                
                debug.println();             
                debug.println(analogRead(0));
                debug.println(F("*** Button ")); 
 
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


// Functions for OLED
#ifdef OLED
void oledLine1() {
  
    oled.setCursor(0,0);
    oled.write("                     "); // Clear the old line, 21 spaces
    oled.setCursor(1,0);
    oled.write("                     "); // Clear the old line, 21 spaces
    oled.setCursor(0,0);   
    oled.print('0');
    oled.print(convertDegMin(gps.location.lat()),2);
    oled.print('N');
    oled.print('/');    
    oled.print(convertDegMin(gps.location.lng()),2);
    oled.print('E'); 

    oled.setCursor(0,115);
    oled.print(gps.satellites.value());

    oled.setCursor(1,0);
    oled.print("S:");   
    //oled.print("000"); 
    oled.print((unsigned int)gps.speed.kmph());  
    oled.print(" C:");
    //oled.print("000"); 
    oled.print((unsigned int)gps.course.deg());  
    oled.print(" A:");
    //oled.print("0000"); 
    oled.print((unsigned int)gps.altitude.meters());
} 

void oledLine2() {    
    oled.setCursor(0,0);
    oled.write("                     "); // Clear the old line, 21 spaces
    oled.setCursor(1,0);
    oled.write("                     "); // Clear the old line, 21 spaces
    oled.setCursor(0,0);   
    oled.print("Rx:");
    oled.print(packetDecoded);  
    oled.print(" Tx:");
    oled.print(txCounter);

    oled.setCursor(1,0);       
    oled.print("H:");
    oled.print(Hd);
    oled.print(" T:");
    oled.print(Ti);
    oled.print(" D:");
    oled.print(Di);
    oled.print(" B:");
    oled.print(Bn);  
}
#endif

///////////////////////////////////////////////////////////////////////////////////////////////////////

void serialEvent() {
  
#ifdef DEBUG            
  debug.println("Entering serialEvent...");
#endif

  while ( Serial.available() > 0) {
    processIncomingByte( Serial.read() );
  }
}


///////////////////////////////////////////////////////////////////////////////////////////////////////


void TxtoRadio() {
  
     char tmp[10];
     float latDegMin, lngDegMin = 0.0;
     String latOut, lngOut, cmtOut = "";
     unsigned int Mem = freeRam();
     float Volt = (float) readVcc()/1000;
  
     lastTxLat = gps.location.lat();
     lastTxLng = gps.location.lng();
     
     if ( lastTx >= 5000 ) { // This prevent ANY condition to Tx below 5 secs
#ifdef DEBUG                 
       debug.print("Time/Date: ");
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
       debug.print((unsigned int) gps.speed.kmph());
       debug.print(" Head:");
       debug.print((unsigned int) gps.course.deg());          
       debug.print(" Alt:");
       debug.print((unsigned int) gps.altitude.meters());
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
 

       // Only send status/version every 10 packets to save packet size  
       if ( ( txCounter % 10 == 0 ) || buttonPressed ) {

          float base = TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          HOME_LAT, 
          HOME_LON)/1000;  
          
          float r1 = TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          R1_LAT, 
          R1_LON)/1000;            

          float r2 = TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          R2_LAT, 
          R2_LON)/1000; 
          
          float r3 = TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          R3_LAT, 
          R3_LON)/1000; 

         cmtOut.concat("!>");                
         cmtOut.concat(VERSION);       
         cmtOut.concat(Volt);
         cmtOut.concat("V S:");
         cmtOut.concat(gps.satellites.value());
         cmtOut.concat(" B:");         
         if ( gps.satellites.value() > 3 ) {
             cmtOut.concat(base);
         } else {
             cmtOut.concat("0");
         }    
         cmtOut.concat(" U:");         
         cmtOut.concat((float) millis()/60000);
         cmtOut.concat(" Seq:");         
         cmtOut.concat(txCounter);
#ifdef DEBUG          
       debug.print("TX STR: ");
       debug.print(cmtOut);  
       debug.println(); 
#endif          
        digitalWrite(buzzerPin,HIGH);  // Turn on buzzer
        Serial.println(cmtOut);
        delay(200);   
        digitalWrite(buzzerPin,LOW);   // Turn off buzzer
        delay(2000);   
        cmtOut = ""; 
       } 
        
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
       cmtOut.concat(padding((int) gps.course.deg(),3));
       cmtOut.concat("/");
       cmtOut.concat(padding((int)gps.speed.mph(),3));
       cmtOut.concat("/A=");
       cmtOut.concat(padding((int)gps.altitude.feet(),6));
       cmtOut.concat(" Seq:");
       cmtOut.concat(txCounter);       

#ifdef DEBUG          
       debug.print("TX STR: ");
       debug.print(latOut);  
       debug.print(" ");       
       debug.print(lngOut);  
       debug.print(" ");
       debug.print(cmtOut);  
       debug.println(); 
#endif         
       
       // This condition is ONLY for button pressed ( do not sent out position if not locked )
       if ( gps.satellites.value() > 3 ) {

       Serial.println(latOut);
       delay(200);
       Serial.println(lngOut);
       delay(200);

       digitalWrite(buzzerPin,HIGH); 
       Serial.println(cmtOut);
       delay(200);
       digitalWrite(buzzerPin,LOW);     
       }
       
       // Reset the txTimer & Tx internal   
       txInterval = 80000;
       buttonPressed = 0;
       lastTx = 0;
#ifdef DEBUG               
       debug.print(F("FreeRAM:"));
       debug.print(Mem);
       debug.print(" Uptime:");
       debug.println((float) millis()/1000);
       debug.println(F("=========================================="));
#endif     

       txCounter++;
     } // endif lastTX
     
     
} // endof TxtoRadio()

///////////////////////////////////////////////////////////////////////////////////////////////////////

void processIncomingByte (const byte inByte)
  {
  static char input_line [MAX_INPUT];
  static unsigned int input_pos = 0;

  switch (inByte)
    {

    case '\n':   
      input_line [input_pos] = 0;  // terminating null byte
      if ( input_pos > 0 ) {  // if not zero bytes
      process_data (input_line);
      }
      input_pos = 0;  
      break;

    case '\r':   
      //input_line [input_pos] = 0;  // terminating null byte
      //process_data (input_line);
      //input_pos = 0;      
      break;

    default:
      // keep adding if not full ... allow for terminating null byte
      if (input_pos < (MAX_INPUT - 1))
        input_line [input_pos++] = inByte;
      break;

    }  // end of switch
   
} // end of processIncomingByte

///////////////////////////////////////////////////////////////////////////////////////////////////////

void process_data (char * input){

  String temp, temp2 = "";
  byte callIndex,callIndex2, dataIndex = 0;
  static char callsign[12]="";
  static char data[MAX_INPUT-13]="";
  static boolean nextLine = 0;
 
#ifdef DEBUG            
  debug.print("INPUT (");
  debug.print(strlen(input));
  debug.print("):");
  debug.println(input);
#endif

  temp = input;  
  callIndex = temp.indexOf("[");
  callIndex2 = temp.indexOf("]");
  temp2 = temp.substring(callIndex+1,callIndex2);  
  temp2.toCharArray(callsign,temp2.length()+1);
 
  temp2 = "";
  dataIndex = temp.indexOf("DATA:");
  temp2 = temp.substring(dataIndex+6,temp.length());
  temp2.toCharArray(data,temp2.length()+1);

  if ( strlen(callsign) > 5 ) {  
        packetDecoded++;
  }

#ifdef DEBUG            
  
  debug.print("Callsign:");
  debug.println(callsign); 
  debug.print("Data (");
  debug.print(strlen(data));
  debug.print("):");
  
  debug.println(data);  
#endif

#ifdef OLED
  if ( !nextLine ) {
    oledLine2();
    oled.setCursor(2,0);
    oled.write("            ");
    oled.setCursor(2,0);
    oled.print(callsign);
  
    oled.setCursor(2,65);
    for ( int i=0;i<53;i++) {
      oled.write(' ');
    }
    oled.setCursor(2,65);  
    data[53] = 0;  // Truncate the data to 52 chars
    oled.print(data);
  
  } else {
    oledLine2();
    oled.setCursor(5,0);
    oled.write("            ");
    oled.setCursor(5,0);
    oled.print(callsign);

    oled.setCursor(5,64);  
    for ( int i=0;i<53;i++) {
      oled.write(' ');
    }
    oled.setCursor(5,64);
    data[53] = 0;  // Truncate the data to 52 chars
    oled.print(data);  
  
  }
  // Toggle the nextLine 
  nextLine ^= 1 << 1;  
#endif
  
}  // end of process_data

///////////////////////////////////////////////////////////////////////////////////////////////////////


void configModem() {
// Functions to configure the callsign, ssid, path and other settings
// c<callsign>
// sc<ssid>
// pd0 - turn off DST display
// pp0 - turn on PATH display

#ifdef OLED
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    oled.write("                      "); // Clear the old line, 21 spaces
    oled.setCursor(1,0);
    oled.write("Config modem...");  
#endif    
    
#ifdef LCD20x4                            
  lcd.setCursor(0,1);
  lcd.print("Configuring modem");
#endif
  digitalWrite(ledPin,HIGH);  
  Serial.println("1WIDE1");  // Set PATH1 callsign
  delay(200);
  
  digitalWrite(ledPin,LOW);  
  Serial.println("2WIDE2");  // Set PATH2 callsign
  delay(200);
  
  digitalWrite(ledPin,HIGH);    
  Serial.println("dAPZSVT");  // Set DST Callsign to APRSVTH
  delay(200);

  digitalWrite(ledPin,LOW);    
  Serial.print("c");          // Set SRC Callsign
  Serial.println(MYCALL);     // Set SRC Callsign
  delay(200);
  
  digitalWrite(ledPin,HIGH);    
  Serial.print("sc");         // Set SRC SSID
  Serial.println(CALL_SSID);      // Set SRC SSID
  delay(200);
  
  digitalWrite(ledPin,LOW);    
  Serial.println("pd0");      // Disable printing DST 
  delay(200);

  digitalWrite(ledPin,HIGH);    
  Serial.println("pp0");      // Disable printing PATH
  delay(200);
  
  digitalWrite(ledPin,LOW);    
  Serial.print("ls");      // Set symbol n / Bluedot
  Serial.println(SYMBOL_CHAR);      // Set symbol n / Bluedot
  delay(200);

  digitalWrite(buzzerPin,HIGH);  // Turn on buzzer
  digitalWrite(ledPin,HIGH);    
  Serial.print("lt");      // Standard symbol 
  Serial.println(SYMBOL_TABLE);      // Standard symbol   
  delay(200);

  digitalWrite(buzzerPin,LOW);  
  digitalWrite(ledPin,LOW);    
  Serial.println("V1");      
  delay(200);

#ifdef OLED
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    oled.write("                      "); // Clear the old line, 21 spaces
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    oled.print("Config done...");  
    delay(500);
#endif  
  
#ifdef LCD20x4 

  lcd.setCursor(1,0);
  lcd.print("Done................");
  delay(500);
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
#if defined(__arm__) && defined(TEENSYDUINO)
    extern "C" char* sbrk(int incr);
    result = 1195 * 4096 /analogRead(39);
#else
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);                     // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV  
#endif  

  return result;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////////

int freeRam() {
#if defined(__arm__) && defined(TEENSYDUINO)
  char top;
        return &top - reinterpret_cast<char*>(sbrk(0));
#else  // non ARM, this is AVR
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
#endif  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

