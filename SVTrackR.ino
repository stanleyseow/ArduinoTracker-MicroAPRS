// Current Version
#define VERSION "SVTrackR v1.0 " 

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
 
 1 Dec 2014 ( v1.0 )
 - Added mic-e/compressed packets decoding from MicroAPRS libs
 - Ability to display received messages
  
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
// The packet decoding libs
#include <MicroAPRS.h>

MicroAPRS microaprs = MicroAPRS(&Serial);
// APRS Buffers
#define BUFLEN (260) //original 260
char packet[BUFLEN];
int buflen = 0;
bool showmsg, showstation;

float latitude = 0.0;
float longitude = 0.0;
float wayPointLatitude, wayPointLongitude;
float latitudeRadians, wayPointLatitudeRadians, longitudeRadians, wayPointLongitudeRadians;
float distanceToWaypoint, bearing, deltaLatitudeRadians, deltaLongitudeRadians;
const float pi = 3.14159265;
const int radiusOfEarth = 6371; // in km

// Turn on/off debug, on by default on pin 2,3
#undef DEBUG

// Turn on/off 0.96" OLED
#define OLED

#ifdef DEBUG 
  #if defined(__arm__) && defined(TEENSYDUINO)
  #else
  #include <SoftwareSerial.h>
  #endif
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

const byte buzzerPin = 4;

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

unsigned int mCounter = 0;
unsigned int txCounter = 0;
unsigned long txTimer = 0;
unsigned long lastTx = 0;
unsigned long lastRx = 0;
unsigned long txInterval = 80000L;  // Initial 80 secs internal

int lastCourse = 0;
byte lastSpeed = 0;
byte buttonPressed = 0;

static unsigned int Hd,Ti,Di,Bn = 0;

int previousHeading, currentHeading = 0;
// Initial lat/lng pos, change to your base station coordnates
float lastTxLat = HOME_LAT;
float lastTxLng = HOME_LON;
float lastTxdistance, homeDistance, base = 0.0;

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



// Buzzer uses pin 4
  pinMode(buzzerPin, OUTPUT);

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
  debug.print(F("DEBUG:- ")); 
  debug.println(F(VERSION)); 
  debug.println();
#endif

  // Set a delay for the MicroAPRS to boot up before configuring it
  delay(1000);
  configModem();
  
  Serial.flush();
  
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
    
    
  while (Serial.available() > 0)
  {
	char ch = microaprs.read();
	if (ch == '\n')
	{
		packet[buflen] = 0;
		show_packet();
		buflen = 0;
	}
	else if ((ch > 31 || ch == 0x1c || ch == 0x1d || ch == 0x27) && buflen < BUFLEN)
	{
		// Mic-E uses some non-printing characters
		packet[buflen++] = ch;
	}
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
          
    base = TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          HOME_LAT, 
          HOME_LON)/1000; 
          
     latitude = gps.location.lat();
     longitude = gps.location.lng();   
          
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

  // Swap the 1st 2 lines of display of gps info or rx/tx packets every 5 secs 
  if ( gps.time.second() % 10 == 0 ) {
      oledLine2();
  } else if ( gps.time.second() % 5 == 0 ) {
      oledLine1();
  } 
  
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

   if ( gps.satellites.value() > 3 ) {
    if ( lastTx > 5000 ) {
        // Check for heading more than 25 degrees
        if ( headingDelta < -25 || headingDelta >  25 ) {
              Hd++;
#ifdef DEBUG                
            debug.println(F("*Head "));
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
            debug.println(F("*D > 600m ")); 
            debug.print(F("lastTxd:"));
            debug.println(lastTxdistance);
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
                   debug.print(F(" txI:"));
                   debug.print(txInterval);     
                   debug.print(F(" lastTxd:"));
                   debug.println(lastTxdistance);               
                   debug.println(F("*TxI "));  

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
                debug.println(F("*B ")); 
 
#endif                                   
                TxtoRadio(); 
                // Reset the txTimer & lastTX for the below if statements  
                txTimer = millis(); 
                lastTx = millis() - txTimer;
     } // endif check analog0

     
} // end loop()

// New MicroAPRS function
void show_packet()
{
	char *posit, *pmsgTo, *call, *pmsg;
	char type, pmsgID;
	long lat, lon;
        static boolean nextLine = 0;

	microaprs.decode_posit(packet, &call, &type, &posit, &lon, &lat, &pmsgTo, &pmsg, &pmsgID);

#ifdef DEBUG    
		debug.print("Input:");
		debug.println(packet);
		debug.print("Callsign:");
		debug.print(call);
		debug.print("  t:");
                debug.print(type);                        
		debug.print"  p:");
		debug.print(posit);
		debug.print(" ");
		debug.print(wayPointLatitude,5);
		debug.print("  ");
		debug.print(wayPointLongitude,5);
		debug.print(" ");
		debug.print(bearing, 0);
		debug.print("deg ");
		debug.print(distanceToWaypoint,1);
		debug.println("km");
#endif

	if (type == 58) // 58 = "!" = Message
	{
		if (startsWith(MYCALL, pmsgTo))
		{
                    mCounter++;
                    oled.setCursor(2,0);
                    clear3Line();                        
                    oled.setCursor(2,0);
                    oled.print("F:");
                    oled.print(call);
                    oled.setCursor(3,0);
                    oled.print("M:");
                    oled.print(pmsg);
                    
                    // Beep 3 times
                    digitalWrite(buzzerPin,HIGH);  // Buzz the user
                    delay(100);
                    digitalWrite(buzzerPin,LOW); 
                    delay(100);
                    digitalWrite(buzzerPin,HIGH);  // Buzz the user
                    delay(100);
                    digitalWrite(buzzerPin,LOW); 
                    delay(100);
                    digitalWrite(buzzerPin,HIGH);  // Buzz the user
                    delay(100);
                    digitalWrite(buzzerPin,LOW); 
                    
#ifdef DEBUG                
  		    debug.print("Msg:");
		    debug.println(pmsg);

#endif
                nextLine ^= 1 << 1;  // Toggle nextLine
		}
	}
	else // Not message, decode , calculate and display packets
	{   		
            wayPointLatitude = lat;		
            wayPointLongitude = lon;

            wayPointLatitude = wayPointLatitude / 1000000;
	    wayPointLongitude = wayPointLongitude / 1000000;

	    distanceToWaypoint = calculateDistance();
	    bearing = calculateBearing();
  
            // Check for valid decoded packets
            if ( strlen(call) < 12 ) {
            lastRx = millis();
            packetDecoded++;

                if ( !nextLine ) {
                    oled.setCursor(2,0);
                    clear3Line();
                    oled.setCursor(2,0);
                    oled.print(call);
                    oled.print(" ");
                    if (distanceToWaypoint < 1000 ) {
                      oled.print(bearing, 0);
                      oled.print("d ");                      
                      oled.print(distanceToWaypoint,1); 
                      oled.print("km");
                    }

                    oled.setCursor(3,0);
                    oled.print(posit);
                } else {
                    oled.setCursor(5,0);
                    clear3Line();
                    oled.setCursor(5,0);
                    oled.print(call);
                    oled.print(" ");                    
                    if (distanceToWaypoint < 1000 ) {
                      oled.print(bearing, 0);
                      oled.print("d ");                      
                      oled.print(distanceToWaypoint,1);
                      oled.print("km");                    
                    }
                    
                    oled.setCursor(6,0);
                    oled.print(posit);                  
                }  // endif !nextLine
                
	      } // endif check for valid packets

              // Toggle the nextLine 
              nextLine ^= 1 << 1;  
	}
}

// Functions for OLED
#ifdef OLED
void oledLine1() {
  
    oled.setCursor(0,0);
    clearLine();
    oled.setCursor(1,0);
    clearLine();
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
    clearLine();
    oled.setCursor(1,0);
    clearLine();
    oled.setCursor(0,0);   
    oled.print("Rx:");
    oled.print(packetDecoded);  
    oled.print(" Tx:");
    oled.print(txCounter);
    oled.print(" L:");
    oled.print((unsigned int)(millis()-lastRx)/1000);  // Print the seconds since last Rx

    oled.setCursor(1,0);  
/*    
    oled.print("H:");
    oled.print(Hd);
    oled.print(" T:");
    oled.print(Ti);
    oled.print(" D:");
    oled.print(Di);
    oled.print(" B:");
    oled.print(Bn);  
 */ 
    oled.print("M:");
    oled.print(mCounter);    // Messages received   
    oled.print(" B:");
    oled.print(base,0);     // Only shows km
    oled.print(" U:");
    oled.print((float) millis()/60000,0); // Only shows in minutes
}
#endif

void clearLine() {
     for ( int i=0;i<21;i++) { oled.write(' '); } 
}

void clear3Line() {
     for ( int i=0;i<64;i++) { oled.write(' '); } 
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

       debug.print("G:");
       debug.print(lastTxLat,5);
       debug.print(" ");
       debug.print(lastTxLng,5);
       debug.println();

       debug.print("S:");           
       debug.print(gps.satellites.value());
       debug.print(" H:");
       debug.print(gps.hdop.value());
       debug.print(" km/h:");           
       debug.print((unsigned int) gps.speed.kmph());
       debug.print(" C:");
       debug.print((unsigned int) gps.course.deg());          
       debug.print(" A:");
       debug.print((unsigned int) gps.altitude.meters());
       debug.print("m");
       debug.println();

       debug.print(F("H:"));
       debug.print(homeDistance,2);
       debug.print(" L:");  
       debug.print(lastTxdistance,2);
       debug.println(); 
     
       debug.print(F("Last "));  
       debug.println((float)lastTx/1000); 
#endif             
 

       // Only send status/version every 10 packets to save packet size  
       if ( ( txCounter % 10 == 0 ) || buttonPressed ) {
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
       debug.print("STR: ");
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
       debug.print("STR: ");
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
       debug.print(F("F:"));
       debug.print(Mem);
       debug.print(" U:");
       debug.println((float) millis()/1000);
#endif     

       txCounter++;
     } // endif lastTX
} // endof TxtoRadio()

///////////////////////////////////////////////////////////////////////////////////////////////////////


void configModem() {
// Functions to configure the callsign, ssid, path and other settings

#ifdef OLED
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    clearLine();
    oled.setCursor(1,0);
    oled.write("Config...");  
#endif    
    
  digitalWrite(ledPin,HIGH);  
  Serial.println("1WIDE1");  // Set PATH1 callsign
  delay(100);
  
  digitalWrite(ledPin,LOW);  
  Serial.println("2WIDE2");  // Set PATH2 callsign
  delay(100);
  
  digitalWrite(ledPin,HIGH);    
  Serial.println("dAPZSVT");  // Set DST Callsign to APRSVTH
  delay(100);

  digitalWrite(ledPin,LOW);    
  Serial.print("c");          // Set SRC Callsign
  Serial.println(MYCALL);     // Set SRC Callsign
  delay(100);
  
  digitalWrite(ledPin,HIGH);    
  Serial.print("sc");         // Set SRC SSID
  Serial.println(CALL_SSID);      // Set SRC SSID
  delay(100);
  
  digitalWrite(ledPin,LOW);    
  Serial.println("pd1");      // Ensable printing DST 
  delay(100);

  digitalWrite(ledPin,HIGH);    
  Serial.println("pp0");      // Disable printing PATH
  delay(100);
  
  digitalWrite(ledPin,LOW);    
  Serial.print("ls");      // Set symbol n / Bluedot
  Serial.println(SYMBOL_CHAR);      // Set symbol n / Bluedot
  delay(100);

  digitalWrite(buzzerPin,HIGH);  // Turn on buzzer
  digitalWrite(ledPin,HIGH);    
  Serial.print("lt");      // Standard symbol 
  Serial.println(SYMBOL_TABLE);      // Standard symbol   
  delay(100);

  digitalWrite(buzzerPin,LOW);  
  digitalWrite(ledPin,LOW);    
  Serial.println("V1");      
  delay(100);

#ifdef OLED
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    clearLine();
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    oled.print("Done...");  
    delay(500);
    oled.clear();
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
// convert degrees to radians
void radianConversion()
{
	deltaLatitudeRadians = (wayPointLatitude - latitude) * pi / 180;
	deltaLongitudeRadians = (wayPointLongitude - longitude) * pi / 180;
	latitudeRadians = latitude * pi / 180;
	wayPointLatitudeRadians = wayPointLatitude * pi / 180;
	longitudeRadians = longitude * pi / 180;
	wayPointLongitudeRadians = wayPointLongitude * pi / 180;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// calculate distance from present location to next way point
float calculateDistance()
{
	radianConversion();
	float a = sin(deltaLatitudeRadians / 2) * sin(deltaLatitudeRadians / 2) +
	          sin(deltaLongitudeRadians / 2) * sin(deltaLongitudeRadians / 2) *
	          cos(latitudeRadians) * cos(wayPointLatitudeRadians);
	float c = 2 * atan2(sqrt(a), sqrt(1 - a));
	float d = radiusOfEarth * c;
	return d * 1;                  // distance in kilometers
//  return d * 0.621371192;        // distance in miles
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
// calculate bearing from present location to next way point
float calculateBearing()
{
	radianConversion();
	float y = sin(deltaLongitudeRadians) * cos(wayPointLatitudeRadians);
	float x = cos(latitudeRadians) * sin(wayPointLatitudeRadians) -
	          sin(latitudeRadians) * cos(wayPointLatitudeRadians) * cos(deltaLongitudeRadians);
	bearing = atan2(y, x) / pi * 180;
	if(bearing < 0)
	{
		bearing = 360 + bearing;
	}
	return bearing;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

bool startsWith(const char *pre, const char *str)
{
	size_t lenpre = strlen(pre),
	       lenstr = strlen(str);
	return lenstr < lenpre ? false : strncmp(pre, str, lenpre) == 0;
}
