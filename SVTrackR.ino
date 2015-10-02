// Current Version
#define VERSION "SVTrackR v1.62 " 

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
 24 Dec 2014 ( v1.1 )
 - Added comment parsing codes from latest MicroAPRS libs
 - Added status ">" type parsing
 
 20 Jan 2015 ( v1.2 )
 - Modify Tx position before Tx status with 3 secs delay
 
 25 Jan 2015 ( V1.3 )
 - Added smartDelay for Tx status to display any received packets during the smartDelay period
 - Added 2 short beeps if Rx station is within 500m away
 
  15 Mar 2015 ( v1.4 - lamaral )
 - Fixed the hardcoding of the North and East on the GPS position
 - Added custom comment on APRS packet 
 - Appended "0" if Lat/Lng is less than 10 deg
  
  23 Mar 2015 ( v1.5 - lamaral )
 - Fixed the problem of appending a zero to negative coordinates
 - Created the beep(int) function to save some memory
  
  07 Apr 2015 ( v1.5a - stanleyseow )
  - fixed the Lontitide with hard coded 0, will give coordinates errors if Lontitide is above 100 deg
  - fixed OLED display for above errors
  
  24 Apr 2015 ( v1.6 - stanleyseow )
  - append recevied callsign and distance to the end of comment
  - fixes some txTimer and lastTx codes
  - changed TxtoRadio return value to boolean and reason for Tx trigger
  - removed unused variables  
  
  4 May 2015 ( v1.61 stanleyseow )
  - Change OLED callsign textsize 2
  - single page packet decode
  - minor changes to comment strings

  31 Aug 2015 ( V1.62 stanleyseow )
  - Adding support for I2C 16x2 for Dashboard LCD
  - Added 3 screens for speed/sats/headings, date/pos and other stats
  - Added N/E/S/W into main screen
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

// Turn on/off 16x2 I2C LCD
#define I2C16X2


// ifdef for I2C LCD
#ifdef I2C16X2
	#include <Wire.h>
	#include <LiquidCrystal_I2C.h>
	LiquidCrystal_I2C lcd(0x27,16,2); // 0x20 is adresss for LCC 16x2
#endif


// Turn on/off 0.96" OLED

#undef OLED

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

char *lastCall="";
String rxCallsign="";
unsigned int rxStation;

unsigned int mCounter = 0;
unsigned int txCounter = 0;
unsigned long txTimer = 0;
#ifdef I2C16X2
bool packetDisplay = 0;
unsigned long displayTime = 0;
#endif
unsigned long lastTx = 0;
unsigned long lastRx = 0;
unsigned long txInterval = 80000L;  // Initial 80 secs internal

int lastCourse = 0;
byte lastSpeed = 0;
byte buttonPressed = 0;

// Unused
//static unsigned int Hd,Ti,Di,Bn = 0;

int previousHeading, currentHeading = 0;
// Initial lat/lng pos, change to your base station coordnates
float lastTxLat = HOME_LAT;
float lastTxLng = HOME_LON;
float lastTxdistance, homeDistance, base = 0.0;

// Used in the future for sending messages, commands to the tracker
const unsigned int MAX_DEBUG_INPUT = 30;


#ifdef I2C16X2
/// LCD Large fonts
// the 8 arrays that form each segment of the custom numbers

/*
byte LT[8] = 
{
  B00011,
  B00111,
  B01111,
  B01111,
  B01111,
  B01111,
  B01111,
  B01111
};*/

byte UB[8] =
{
  B11111,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
  B00000
};

byte RT[8] =
{
  B11100,
  B11110,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111
};

byte LL[8] =
{
  B01111,
  B01111,
  B01111,
  B01111,
  B01111,
  B01111,
  B00111,
  B00011
};

byte LB[8] =
{
  B00000,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111
};
/*
byte LR[8] =
{
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11111,
  B11110,
  B11100
};
*/
byte UMB[8] =
{
  B11111,
  B11111,
  B11111,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111
};
byte LMB[8] =
{
  B11111,
  B00000,
  B00000,
  B00000,
  B00000,
  B11111,
  B11111,
  B11111
};

int x = 0;

/// LCD Large fonts
#endif

//////////////////////////////////////////////////////////////////////////////
// setup()
//////////////////////////////////////////////////////////////////////////////

void setup()
{
  
#ifdef I2C16X2
  lcd.init();

  lcd.createChar(1,RT);  
  lcd.createChar(2,UB);
//  lcd.createChar(3,LL);
  lcd.createChar(4,LB);
//  lcd.createChar(5,LR);  
  lcd.createChar(6,UMB);
  lcd.createChar(0,LMB);
//  lcd.createChar(8,LT);
  
  lcd.backlight();
  lcd.begin(16,2);
  
  lcd.print(VERSION);
  delay(1000);
#endif


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





    if ( gps.time.second() % 10 == 0 ) { 
#ifdef OLED
      oled.setTextSize(1,1);        // 5x7 characters, pixel spacing = 1
      oledLine2();
#endif      
  } else if ( gps.time.second() % 5 == 0 ) {
#ifdef OLED    
      oled.setTextSize(1,1);        // 5x7 characters, pixel spacing = 1
      oledLine1();
#endif
  } 
  
// I2C LCD addons
#ifdef I2C16X2
  if ( packetDisplay ) {
     // Do not overwrite the decoded packet
     // display packetDecoded for 10 secs
     if ( millis() - displayTime > 10000 ) {
           packetDisplay = 0;
     }
  } else if ( gps.time.second() > 30 && gps.time.second() < 36 ) {
      lcdScreen3(); // Screen1 is speed / sats
  } else if ( gps.time.second() > 50 && gps.time.second() < 56 ) { 
      lcdScreen2(); // Screen2 is Altitide / bearing / distBase / Uptime / Rx / Tx / Msgs
   } else {
      lcdScreen1(); // Screen1 is speed / sats 
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
 
  lastTx = millis() - txTimer;

  // Only check the below if locked satellites < 3

   if ( gps.satellites.value() > 3 ) {
    if ( lastTx > 5000 ) {
        // Check for heading more than 25 degrees
        if ( (headingDelta < -25 || headingDelta >  25) && lastTxdistance > 5 ) {
            if (TxtoRadio(1)) {
                lastTxdistance = 0;   // Ensure this value is zero before the next Tx
                previousHeading = currentHeading;
            }
        } // endif headingDelta
    } // endif lastTx > 5000
    
    if ( lastTx > 10000 ) {
         // check of the last Tx distance is more than 600m
         if ( lastTxdistance > 600 ) {  
            if ( TxtoRadio(2) ) {
                lastTxdistance = 0;   // Ensure this value is zero before the next Tx
            }
       } // endif lastTxdistance
    } // endif lastTx > 10000
    
    if ( lastTx >= txInterval ) {
        // Trigger Tx Tracker when Tx interval is reach 
        // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
        if ( lastTxdistance > 20 ) {
              TxtoRadio(3); 
        } // endif lastTxdistance > 20 
    } // endif of check for lastTx > txInterval
    
    // Force a tx when 3 unique stations was received 
    // Or 60 secs after last received station to prevent stale info on the distance 
    if (  (rxStation > 2) || (( (rxStation > 0) && ( ((millis()-lastRx)/1000) > 60) )) )  {
           TxtoRadio(4);
     }
   } // Endif check for satellites
     
    // Check if the analog0 is plugged into 5V and more than 10 secs
    if ( analogRead(0) > 700 && (lastTx > 10000) ) {
          buttonPressed = 1;                                  
          TxtoRadio(5); 
     } // endif check analog0

     
} // end loop()

// New MicroAPRS function
void show_packet()
{
	char *posit, *pmsgTo, *call, *pcomment, *pmsg;
	char type, pmsgID;
	long lat, lon;

        // Only displasy if decode is true
	if ( microaprs.decode_posit(packet, &call, &type, &posit, &lon, &lat, &pcomment, &pmsgTo, &pmsg, &pmsgID) ) {

	if (type == 58) // 58 = "!" = Message
	{
		if (startsWith(MYCALL, pmsgTo))
		{
                    mCounter++;
#ifdef OLED
                    oled.setCursor(2,0);
                    clear3Line();                        
                    oled.setCursor(2,0);
                    //oled.print("F:");
                    oled.setTextSize(2,1);
                    oled.print(call);
                    oled.setTextSize(1,1);                    
                    oled.setCursor(4,0);
                    oled.print("M:");
                    oled.print(pmsg);
#endif                    
                    // Beep 3 times
                    beep(3);
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
  
            // Beep twice is station is less than 500m
            if ( distanceToWaypoint < 0.5 ) {
                 beep(2);
            }
            
            // Check for valid decoded packets
            if ( strlen(call) < 12 ) {
            lastRx = millis();
            packetDecoded++;
            
            // Append rx callsign into rxCallsign strings
            // Do not add own callsign
            if ( !startsWith(MYCALL,call) ) {
                // Do not add duplicated callsign from digipeater packets
        	if ( !rxCallsign.startsWith(call) ) {
			rxCallsign.concat(" ");
			rxCallsign.concat(call);
                        lastCall = call;
                        // Only send distance if GPS is locked AND less than 300km away
                        if ( gps.satellites.value() > 3 && distanceToWaypoint < 300 ) {
			    rxCallsign.concat("/");
			    rxCallsign.concat(distanceToWaypoint);
			    rxCallsign.concat("km");
                        } 
			rxStation++;
                 }
	    }

#ifdef OLED

                    oled.setCursor(2,0);
                    clear6Line();
                    oled.setCursor(2,0);
                    oled.setTextSize(2,1);
                    oled.print(call);
                    oled.setTextSize(1,1);  

                    if (distanceToWaypoint < 1000 ) {
                        oled.setCursor(4,0);                    
                        oled.print("B:");
                        oled.print(bearing, 0);
                        oled.print("deg D:");                      
                        oled.print(distanceToWaypoint,1); 
                        oled.print("km");
                        oled.setCursor(5,0);
                        oled.print(pcomment);
                    } else {
                        oled.setCursor(4,0);
                        oled.print(pcomment);
                    }

#endif

#ifdef I2C16X2
          packetDisplay = 1;
          displayTime = millis();
          lcdScreen4(call,pcomment);
#endif
                
	      } // endif check for valid packets
	}
    } // endif microaprs.decode_posit()
}



#ifdef I2C16X2
void lcdScreen1() {
  lcd.clear();
  displayLargeSpeed((unsigned int)gps.speed.kmph());
}

void lcdScreen2() {

  lcd.clear();  
  lcd.setCursor(0,0);
  lcd.print("B:");
  lcd.print(base,0);     // Max 999
  
  lcd.setCursor(6,0);
  lcd.print("T:");      
  lcd.print(txCounter);  // Max 999

   // Max 2 digits10w
 
  lcd.setCursor(12,0);
  lcd.print("R:");      
  lcd.print(packetDecoded);  // Max 99


  //lcd.setCursor(0,1);
  //lcd.print("M:");
  //lcd.print(mCounter);    // Messages received  
  
  //lcd.setCursor(0,1);
  //lcd.print("C:");
  //lcd.print((unsigned int)gps.course.deg());  

  lcd.setCursor(0,1);
  lcd.print("U:");
  lcd.print((float) millis()/60000,0); // Max 999

  lcd.setCursor(6,1);  
  lcd.print("A:");
  lcd.print((unsigned int)gps.altitude.meters()); // Max 9999
   
  //lcd.setCursor(12,1);
  //lcd.print("L:");
  //lcd.print((unsigned int)(millis()-lastRx)/1000);  // Print the seconds since last Rx

}

void lcdScreen3() {
     lcd.clear();  
     lcd.setCursor(0,0);
  
     // Convert to GMT+8
     byte hour = gps.time.hour() +8;
      
     if ( hour > 23 ) {
         hour = 0;
     }  
     if ( hour < 10 ) {
        lcd.print("0");       
     }  
       lcd.print(hour);         
       lcd.print(":");  
     if ( gps.time.minute() < 10 ) {
        lcd.print("0");       
     }
       lcd.print(gps.time.minute());
       lcd.print(":");
     if ( gps.time.second() < 10 ) {
       lcd.print("0");       
     }       
       lcd.print(gps.time.second());


  lcd.print(" H:");
  lcd.print(gps.hdop.value());
  
  lcd.setCursor(0,1);
  lcd.print(gps.location.lat(),4);
  lcd.setCursor(8,1);
  lcd.print(gps.location.lng(),4);
}

void lcdScreen4(char *callsign, char *comment ) {
 
       lcd.setCursor(0,0);
       lcd.clear();
       lcd.print(callsign);
            if (distanceToWaypoint < 100 ) {
                lcd.print(" ");
                lcd.print(distanceToWaypoint,1);
                lcd.print("km");
            }
       lcd.setCursor(0,1);
       comment[16] = 0;
       lcd.print(comment);
}

#endif


// Functions for OLED
#ifdef OLED
void oledLine1() {
  
    oled.setCursor(0,0);
    clearLine();
    oled.setCursor(1,0);
    clearLine();
    oled.setCursor(0,0);   
    
    if ( fabs(gps.location.lat()) < 10 ) { 
          oled.print('0');
    }

    oled.print(convertDegMin(gps.location.lat()),2);
        
    // Determime to print N or S     
    if ( convertDegMin(gps.location.lat()) >= 0 ) {
      oled.print('N');
    } else if (convertDegMin(gps.location.lat()) < 0 ) {
      oled.print('S');
    }  
    
    oled.print('/'); 

    // Append 0 to OLED
    if ( fabs(gps.location.lng()) < 100 ) { 
          oled.print('0');
    }

    // Append 0 to OLED  
    if ( fabs(gps.location.lng()) < 10 ) { 
          oled.print('0');
    }
    
    oled.print(convertDegMin(gps.location.lng()),2);

    // Determime to print E or W         
    if ( convertDegMin(gps.location.lng()) >= 0 ) {
        oled.print('E'); 
    } else if ( convertDegMin(gps.location.lng()) < 0 ) {
        oled.print('W'); 
    }      
    
    oled.setCursor(0,115);
    oled.print(gps.satellites.value());

    oled.setCursor(1,0);
    oled.print("S:");   
    oled.print((unsigned int)gps.speed.kmph());  
    oled.print(" C:");
    oled.print((unsigned int)gps.course.deg());  
    oled.print(" A:");
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

    oled.print("M:");
    oled.print(mCounter);    // Messages received   
    oled.print(" B:");
    oled.print(base,0);     // Only shows km
    oled.print(" U:");
    oled.print((float) millis()/60000,0); // Only shows in minutes
}

void clearLine() {
     for ( int i=0;i<21;i++) { oled.write(' '); } 
}

void clear3Line() {
     for ( int i=0;i<64;i++) { oled.write(' '); } 
}

void clear6Line() {
     for ( int i=0;i<128;i++) { oled.write(' '); } 
}

#endif


///////////////////////////////////////////////////////////////////////////////////////////////////////


boolean TxtoRadio(int type) {
  
     char tmp[10];
     float latDegMin, lngDegMin = 0.0;
     String latOut, lngOut, cmtOut = "";
     unsigned int Mem = freeRam();
     float Volt = (float) readVcc()/1000;
  
     lastTxLat = gps.location.lat();
     lastTxLng = gps.location.lng();
     
     if ( lastTx > 6000 ) { // This prevent ANY condition to Tx below 6 secs
             
       latDegMin = convertDegMin(lastTxLat);
       lngDegMin = convertDegMin(lastTxLng);

       // Convert Lat float to string
       dtostrf(fabs(latDegMin), 2, 2, tmp );
       latOut.concat("lla");      // set latitute command 
       
       // Append 0 if Lat less than 10 
       if ( fabs(lastTxLat) < 10 ) {
       latOut.concat("0");              
       }  
       
       latOut.concat(tmp);      // Actual Lat in DDMM.MM

       // Determine E or W       
       if (latDegMin >= 0) {
           latOut.concat("N");
       } else if (latDegMin < 0) {
           latOut.concat("S");
       }

       // Convert Lng float to string
       dtostrf(fabs(lngDegMin), 2, 2, tmp );
       lngOut.concat("llo");       // set longtitute command
       
       // Append 0 if Lng less than 100
       if ( ( fabs(lastTxLng) < 100) ) {
             lngOut.concat("0");       // set longtitute command
       }     

       // Append 0 if Lng less than 10
       if ( fabs(lastTxLng) < 10 ) {
       latOut.concat("0");      // Append 0 if Lng less than 10         
       }  

       lngOut.concat(tmp);      // Actual Lng in DDDMM.MM

       // Determine E or W
       if (lngDegMin >= 0) {
           lngOut.concat("E");
       } else if (latDegMin < 0) {
           lngOut.concat("W");
       }
     
       cmtOut.concat("@");
       cmtOut.concat(padding((int) gps.course.deg(),3));
       cmtOut.concat("/");
       cmtOut.concat(padding((int)gps.speed.mph(),3));
       cmtOut.concat("/A=");
       cmtOut.concat(padding((int)gps.altitude.feet(),6));
       cmtOut.concat(" Seq:");
       cmtOut.concat(txCounter);
       
       if ( rxCallsign.length() > 0 ) {
           cmtOut.concat(rxCallsign);     // Send out all the Rx callsign & dist  
       }
        
       // Send out the type of Tx trigger
       switch (type) {
         case 1:   cmtOut.concat(" H"); break;
         case 2:   cmtOut.concat(" D"); break;
         case 3:   cmtOut.concat(" T"); break;
         case 4:   cmtOut.concat(" R"); break;
         case 5:   cmtOut.concat(" B"); break;
         default: break;
       }

       cmtOut.concat(" ");       
       cmtOut.concat(COMMENT); 
       

       
       // This condition is ONLY for button pressed ( do not sent out position if not locked )
       if ( gps.satellites.value() > 3 ) {
       Serial.println(latOut);
       delay(200);
       digitalWrite(buzzerPin,HIGH); 
       Serial.println(lngOut);
       delay(200);
       Serial.println(cmtOut);
       digitalWrite(buzzerPin,LOW);     
       delay(50);
       
       // Clear the rxCallsign contents & counters
       rxCallsign="";
       rxStation=0;
       lastRx = 0;
       }
       
        // Only send status/version every 10 packets to save packet size  
        // Sample status display
        // >SVTrackR v1.5a 5.14V S:10 B:0 U:135m Seq:20
        
       if ( ( txCounter % 10 == 0 ) || buttonPressed ) {
  
         digitalWrite(buzzerPin,HIGH);  // Turn on buzzer
         cmtOut = ""; 
         cmtOut.concat("!>");                
         cmtOut.concat(VERSION);       
         cmtOut.concat(Volt);
         cmtOut.concat("V S:");
         cmtOut.concat(gps.satellites.value());
         cmtOut.concat(" B:");         
         if ( gps.satellites.value() > 3 ) {
             cmtOut.concat((unsigned int)base);
         } else {
             cmtOut.concat("0");
         }    
         // R - number of packets decoded
         cmtOut.concat(" R:");         
         cmtOut.concat(packetDecoded);
         
         cmtOut.concat(" U:");         
         cmtOut.concat(millis()/60000);
         cmtOut.concat("m Seq:");         
         cmtOut.concat(txCounter);
 
        delay(200);   
        digitalWrite(buzzerPin,LOW);   // Turn off buzzer
        
        smartDelay(5000);
        Serial.println(cmtOut);
       } //endof txCounter / buttonPressed 
       
       
       // Reset all tx timer & Tx variables   
       txInterval = 80000;
       buttonPressed = 0;    
       txCounter++;
       txTimer = millis(); 
       lastTx = 0;
       
       // Tx success, return TRUE
         return 1;
     } else {
         return 0; 
     }// endif lastTX > 6000
     
     
} // endof TxtoRadio()

///////////////////////////////////////////////////////////////////////////////////////////////////////


void configModem() {
// Functions to configure the callsign, ssid, path and other settings

#ifdef I2C16X2
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Config:");
	lcd.setCursor(0,1);
	lcd.print(MYCALL);
	lcd.print("-");
	lcd.print(CALL_SSID);
#endif

#ifdef OLED
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    clearLine();
    oled.setCursor(1,0);
    oled.print("Config ");  
    oled.print(MYCALL);  
    oled.print("-");  
    oled.print(CALL_SSID);  
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

#ifdef I2C16X2
	lcd.clear();
	lcd.setCursor(0,0);
	lcd.print("Done...");
	lcd.setCursor(0,1);
	lcd.print(MYCALL);
	lcd.print("-");
	lcd.print(CALL_SSID);
	delay(1000);
#endif

#ifdef OLED
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    clearLine();
    oled.setCursor(1,0);          // move cursor to row 1, pixel column 100
    delay(1000);
    oled.print("Done..........");  
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

///////////////////////////////////////////////////////////////////////////////////////////////////////

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
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
    } // end of Serial.available while loop
  } while (millis() - start < ms);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
void beep(int i) {
  while (i>0) {
    digitalWrite(buzzerPin,HIGH);  // Buzz the user
    delay(100);
    digitalWrite(buzzerPin,LOW); 
    delay(100);
    i--; 
  }
}

#ifdef I2C16X2
///////////////////////////////////////////////////////////////////////////////////////////////////////
// LCD Large fonts
void displayLargeSpeed(int num) {
  
  int digit4 =  num / 1000;
  int digit3 = (num % 1000) / 100;
  int digit2 = (num % 100) / 10;
  int digit1 =  num % 10;
  
  
  x = 0;
  switch (digit3) {
  
   case 0: 
       // Do not display first zero
       //custom00();
       break;
   case 1:
       custom1();
       break;   
   case 2:
       custom2();
       break; 
   case 3:
       custom3();
       break; 
   case 4:
       custom4();
       break; 
   case 5:
       custom5();
       break; 
   case 6:
       custom6();
       break; 
   case 7:
       custom7();
       break; 
   case 8:
       custom8();
       break; 
   case 9:
       custom9();
       break;        
   default:
       break;
  }
  
    x = 4;
    switch (digit2) {
  
   case 0: 
       custom00();
       break;
   case 1:
       custom1();
       break;   
   case 2:
       custom2();
       break; 
   case 3:
       custom3();
       break; 
   case 4:
       custom4();
       break; 
   case 5:
       custom5();
       break; 
   case 6:
       custom6();
       break; 
   case 7:
       custom7();
       break; 
   case 8:
       custom8();
       break; 
   case 9:
       custom9();
       break;        
   default:
       break;
  }
  
  x = 8;
    switch (digit1) {
  
   case 0: 
       custom00();
       break;
   case 1:
       custom1();
       break;   
   case 2:
       custom2();
       break; 
   case 3:
       custom3();
       break; 
   case 4:
       custom4();
       break; 
   case 5:
       custom5();
       break; 
   case 6:
       custom6();
       break; 
   case 7:
       custom7();
       break; 
   case 8:
       custom8();
       break; 
   case 9:
       custom9();
       break;        
   default:
       break;
  }
  
    if ( gps.satellites.value() < 10 ) {
          lcd.setCursor(14,0);
          lcd.print(gps.satellites.value());
          lcd.print("s");
    } else {
          lcd.setCursor(13,0);
          lcd.print(gps.satellites.value());
          lcd.print("s");
    }   
    
    //lcd.print("S");

    // Print degree or N/S/E/W headings
    lcd.setCursor(12,1);
    if ( gps.course.deg() < 10 ) {
          lcd.print("00");
          lcd.print(gps.course.deg(),0);
          lcd.print("d");          
    } else if ( gps.course.deg() < 100 ) {
          lcd.print("0");
          lcd.print(gps.course.deg(),0);
          lcd.print("d");
    } else {
          lcd.print(gps.course.deg(),0);
          lcd.print("d");          
    }
      
    
    /*
    unsigned int headingDegrees = (unsigned int)gps.course.deg();

      if (headingDegrees < 0 || headingDegrees > 360) {
	}
	else if (headingDegrees >= 0 && headingDegrees <= 11)
	{
		lcd.print("  N");
	}
	else if (headingDegrees > 349 && headingDegrees <= 360)
	{
  		lcd.print("  N");
	}
	else if (headingDegrees > 11 && headingDegrees <= 34)
	{
		lcd.print("NNE");
	}
	else if (headingDegrees > 34 && headingDegrees <= 56)
	{
		lcd.print(" NE");
	}
	else if (headingDegrees > 56 && headingDegrees <= 79)
	{
		lcd.print("ENE");
	}
	else if (headingDegrees > 79 && headingDegrees <= 101)
	{
		lcd.print("  E");
	}
	else if (headingDegrees > 101 && headingDegrees <= 124)
	{
		lcd.print("ESE");
	}
	else if (headingDegrees > 124 && headingDegrees <= 146)
	{
		lcd.print(" SE");
	}
	else if (headingDegrees > 146 && headingDegrees <= 169)
	{
		lcd.print("SSE");
	}
	else if (headingDegrees > 169 && headingDegrees <= 191)
	{
		lcd.print("  S");
	}
	else if (headingDegrees > 191 && headingDegrees <= 214)
	{
		lcd.print("SSW");
	}
	else if (headingDegrees > 214 && headingDegrees <= 236)
	{
		lcd.print(" SW");
	}
	else if (headingDegrees > 236 && headingDegrees <= 259)
	{
		lcd.print("WSW");
	}
	else if (headingDegrees > 259 && headingDegrees <= 281)
	{
		lcd.print("  W");
	}
	else if (headingDegrees > 281 && headingDegrees <= 304)
	{
		lcd.print("WNW");
	}
	else if (headingDegrees > 304 && headingDegrees <= 326)
	{
		lcd.print(" NW");
	}
	else if (headingDegrees > 326 && headingDegrees <= 349)
	{
		lcd.print("NNW");
	}
*/

}


void custom00()
{ // uses segments to build the number 0
  lcd.setCursor(x, 0); 
  lcd.write(255);  
  lcd.write(2); 
  lcd.write(255);
  lcd.setCursor(x, 1); 
  lcd.write(255);  
  lcd.write(4);  
  lcd.write(255);
}

void custom1()
{
  lcd.setCursor(x,0);
  lcd.write(2);
  lcd.write(255);
  lcd.setCursor(x+1,1);
  lcd.write(255);
}

void custom2()
{
  lcd.setCursor(x,0);
  lcd.write(6);
  lcd.write(6);
  lcd.write(255);
  lcd.setCursor(x, 1);
  lcd.write(255);
  lcd.write(0);
  lcd.write(0);
}

void custom3()
{
  lcd.setCursor(x,0);
  lcd.write(6);
  lcd.write(6);
  lcd.write(255);
  lcd.setCursor(x, 1);
  lcd.write(0);
  lcd.write(0);
  lcd.write(255); 
}

void custom4()
{
  lcd.setCursor(x,0);
  lcd.write(255);
  lcd.write(4);
  lcd.write(255);
  lcd.setCursor(x+2, 1);
  lcd.write(255);
}

void custom5()
{
  lcd.setCursor(x,0);
  lcd.write(255);
  lcd.write(6);
  lcd.write(6);
  lcd.setCursor(x, 1);
  lcd.write(0);
  lcd.write(0);
  lcd.write(255);
}

void custom6()
{
  lcd.setCursor(x,0);
  lcd.write(255);
  lcd.write(6);
  lcd.write(6);
  lcd.setCursor(x, 1);
  lcd.write(255);
  lcd.write(0);
  lcd.write(255);
}

void custom7()
{
  lcd.setCursor(x,0);
  lcd.write(2);
  lcd.write(2);
  lcd.write(255);
  lcd.setCursor(x+1, 1);
  lcd.write(255);
}

void custom8()
{
  lcd.setCursor(x,0);
  lcd.write(255);
  lcd.write(6);
  lcd.write(255);
  lcd.setCursor(x, 1);
  lcd.write(255);
  lcd.write(0);
  lcd.write(255);
}

void custom9()
{
  lcd.setCursor(x,0);
  lcd.write(255);
  lcd.write(6);
  lcd.write(255);
  lcd.setCursor(x+2, 1);
  lcd.write(255);
}

#endif



