/*
 Copyright (C) 2014 Stanley Seow <stanleyseow@gmail.com>

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 
 github URL :-
 https://github.com/stanleyseow/ArduinoTracker-MicroAPRS
 
 */
 
#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal.h>
// My wiring for LCD on breadboard
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);

#define VERSION "Arduino OpenTrackR v0.3 " 

/* 
  
 This sketch configure the MicroAPRS for the proper callsign and ssid and
 read/write data coming in from the MicroAPRS via debug port
 
 Pin 0/1 (rx,tx) connects to Arduino with MicroAPRS firmware
 Pin 2,3 ( rx,tx ) connects to GPS module
 Pin 8,9 connect to debug serial port
 Pin 4,5,6,7,12,11 connects to 20x4 LCD 
 
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
 - added check for speed and idle speed, modify the Txinternal
 - remove all debug Monitor output
 - added check for Rx bytes before sending Tx bytes, delay by 3 secs if have available bytes
 
 12 July 2014 :-
 - Added SmartBeaconing algorithm, Tx when turn is more than 25 deg
 - Reduce the Tx interval lower
 
 14 July 2014 :-
 - Fixed coordinates conversion from decimal to Deg Min formula
 - Formula to calculate distance from last Tx point so that it will Tx once the max distance 
 is reached, 500m
 
 TODO :-

 
 Bugs :-
 
 - During driving/moving, the packet decoding rate is very low
 - Packet are sometimes not decoded properly or only decoded callsign only 
 
*/

TinyGPSPlus gps;

// Altdebug default on UNO is 8-Rx, 9-Tx
AltSoftSerial debug;

// Connect to GPS module on pin 9, 10 ( Rx, Tx )
SoftwareSerial ss(2,3);

unsigned long last = 0UL;

// APRS buffers
String inBuffer = "";
int maxBuffer = 200;
//char charBuffer[199];
byte bufferIndex=0;

byte inputIndex = 0;

unsigned long txTimer = 0;
long lastTx = 0;
long txInterval = 60000L;  // Initial 60 secs internal
int txCounter=0;

// Speed in km/h
byte highSpeed = 80;       // High speed  
byte lowSpeed = 40;        // Low speed

int lastCourse = 0;
byte lastSpeed = 0;

int previousHeading, currentHeading = 0;
// Initial lat/lng pos, change to your base station coordnates
float lastTxLat = 3.16925, lastTxLng =  101.64972;
float lastTxdistance, homeDistance = 0.0;

void setup()
{
  // LCD format is Col,Row for 20x4 LCD
  lcd.begin(20,4);
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print("Arduino OpenTrackR"); 

  //pinMode(10, OUTPUT);
  pinMode(13, OUTPUT);
  
  Serial.begin(9600);
  debug.begin(9600);
  ss.begin(9600);

  debug.flush();
  debug.println();
  debug.println();
  debug.println("==================================");  
  debug.print("DEBUG:- "); 
  debug.println(VERSION); 
  debug.println("==================================");  
  debug.println();

  
  delay(500);
  lcd.clear();
  
  // Set a delay for the MicroAPRS to boot up before configuring it
  delay(1000);
  configModem();
  
  unsigned long txTimer = millis();

} // end setup()


///////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{
    char c;
    boolean inputComplete = false;
    int headingDelta = 0;
    static const double HOME_LAT = 3.16925 , HOME_LON = 101.64972;
    
    //if ( Serial.available() > 0 ) {
    //decodeAPRS(); 
    //}
    
    // Send commands from debug serial into hw Serial char by char 
    debug.listen();
    if ( debug.available() ) {
         c = debug.read();
         Serial.print(c);
    }
    
    ss.listen();                    // Turn on listen() on GPS
    while ( ss.available() > 0 ) {
      gps.encode(ss.read());
    }
    
   // Turn on LED at pin 13 when a GPS lock 
   if ( gps.location.isValid() ) {
        digitalWrite(13,HIGH);
   } else {
        digitalWrite(13,LOW);     
   }
 
 
///////////////// Triggered by location updates /////////////////////// 
   if ( gps.location.isUpdated()) { 

     homeDistance =   TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          HOME_LAT, 
          HOME_LON);         
    lastTxdistance =   TinyGPSPlus::distanceBetween(
          gps.location.lat(),
          gps.location.lng(),
          lastTxLat,
          lastTxLng);
     
    } // endof gps.location.isUpdated()

///////////////// Triggered by time updates /////////////////////// 
// Update LCD every second

   if (gps.time.isUpdated()) {     

     lcd.setCursor(1,0);
     lcd.print("   ");
     lcd.setCursor(1,0);
     lcd.print(txCounter);

     lcd.setCursor(3,0);
     lcd.print("       ");
     lcd.setCursor(3,0);
     lcd.print(lastTx);
     
     // Print to LCD the last TX distance, 3 digits max, 999m
     lcd.setCursor(10,0);
     lcd.print("    ");  
     lcd.setCursor(10,0);
     lcd.print((int)lastTxdistance);    

     // Print to LCD the distance to home, 4 digits max, 9999m
     lcd.setCursor(14,0);
     lcd.print("    ");  
     lcd.setCursor(14,0);
     lcd.print((float)homeDistance/1000,1);    // Format is 12.34 km
     
     // Satellites in view
     lcd.setCursor(18,0);   
     lcd.print("  "); 
     lcd.setCursor(18,0);   
     lcd.print(gps.satellites.value());
     
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
// Formula need to be confirmed
     
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
            debug.println();        
            debug.print("Heading, current:");      
            debug.print(currentHeading);
            debug.print(" previous:");      
            debug.print(previousHeading);
            debug.print(" delta:");      
            debug.println(headingDelta);
        
            debug.println("*** Trigger by Heading Change"); 
            lcd.setCursor(0,0);
            lcd.print("H");        
            txTimer = millis(); 
            TxtoRadio();
            previousHeading = currentHeading;
            lastTx = millis() - txTimer;
        }
    }
    
    if ( (lastTx > 10000) && (gps.satellites.value() > 3) ) {
         // check of the last Tx distance is more than 500m
         if ( lastTxdistance > 500 ) {  
            debug.println();
            debug.println("*** Trigger by distance > 500m"); 
            debug.print("lastTxdistance: ");
            debug.println(lastTxdistance);
            lcd.setCursor(0,0);
            lcd.print("D");
            TxtoRadio();
            lastTxdistance = 0;   // Ensure this value is zero before the next Tx
            lastTx = millis() - txTimer;

         } else if ( analogRead(0) > 700 ) {
           // Check if the analog0 is plugged into 5V
                     debug.println();             
                     debug.println(analogRead(0));
                     debug.println("*** Trigger by button, 10sec delay/Tx");  
                     lcd.setCursor(0,0);
                     lcd.print("B");     
                     txTimer = millis(); 
                     TxtoRadio(); 
                     lastTx = millis() - txTimer;

           }
    }
    
    if ( (lastTx >= txInterval) && ( gps.satellites.value() > 3) ) {
        // Trigger Tx Tracker when Tx interval is reach 
        // Will not Tx if stationary bcos speed < 5 and lastTxDistance < 20
               if ( lastTxdistance > 20 ) {
                   debug.println();
                   debug.print("lastTx:");
                   debug.print(lastTx);
                   debug.print(" txInterval:");
                   debug.print(txInterval);     
                   debug.print(" lastTxdistance:");
                   debug.println(lastTxdistance);
               
                   debug.println("*** Trigger by txInterval and lastTxdistance > 20");   
                   lcd.setCursor(0,0);
                   lcd.print("T");        
                   txTimer = millis(); 
                   TxtoRadio(); 
                   lastTx = millis() - txTimer;
               }      
  } // endif of check for lastTx
   

} // end loop()

///////////////////////////////////////////////////////////////////////////////////////////////////////

void serialEvent() {
    decodeAPRS(); 
}

void TxtoRadio() {
  
     txCounter++;
     
     lastTxLat = gps.location.lat();
     lastTxLng = gps.location.lng();

     debug.print("Date/Time: ");
     debug.print(gps.date.value());
     debug.print(" ");
     debug.println(gps.time.value());
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
     debug.print(" PrevHead:");
     debug.print(previousHeading); 
     debug.print(" Alt:");
     debug.print(gps.altitude.meters());
     debug.print("m");
     debug.println();

     debug.print("Distance(m): Home:");
     debug.print(homeDistance,2);
     debug.print(" Last:");  
     debug.print(lastTxdistance,2);
     debug.println(); 
     
     debug.print("Writing to radio since ");  
     debug.print(lastTx); 
     debug.println(" ms");    
     // Turn on the buzzer
     digitalWrite(10,HIGH);  
          
     char tmp[10];
     float latDegMin, lngDegMin = 0.0;
     String latOut, lngOut, cmtOut = "";
     
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
     
     cmtOut.concat("@ ");
     cmtOut.concat(VERSION);
     cmtOut.concat((float) readVcc()/1000);
     cmtOut.concat("V ");
     cmtOut.concat((int) gps.speed.kmph());
     cmtOut.concat("km/h ");
     cmtOut.concat(gps.hdop.value());
     cmtOut.concat("/");
     cmtOut.concat(gps.satellites.value());

     debug.print("TX STR: ");
     debug.print(latOut);
     debug.print(" ");           
     debug.print(lngOut);
     debug.println();
     debug.print(cmtOut);  
     debug.println(); 

     Serial.println(latOut);
     delay(300);
     Serial.println(lngOut);
     delay(300);
     Serial.println(cmtOut);
     delay(300);
                 
     digitalWrite(10,LOW);     
     // Reset the txTimer & Tx internal   
    
     txInterval = 60000;
     // txTimer = millis(); 
     lastTx = 0;
     debug.print("FreeRAM:");
     debug.println(freeRam());
     debug.println("==========================================");
}

///////////////////////////////////////////////////////////////////////////////////////////////////////


void configModem() {
// Functions to configure the callsign, ssid, path and other settings
// c<callsign>
// sc<ssid>
// pd0 - turn off DST display

  lcd.setCursor(0,0);
  lcd.print("Configuring modem");
  
  lcd.setCursor(0,0);
  lcd.print("Configuring callsig");
  Serial.println("c9W2SVT");  // Set SRC Callsign
  delay(200);
  lcd.setCursor(0,0);
  lcd.print("Configuring ssid");
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
  
  lcd.setCursor(0,0);
  lcd.print("Done................");
  delay(500); 
  lcd.clear();
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////

void decodeAPRS() {
      // Dump whatever on the Serial to LCD line 1
      char c;
      String decoded="";

      //debug.println();
      //debug.print("Entering decodeAPRS (");
      //debug.print(millis());
      //debug.println(")");
      while ( Serial.available() > 0 ) {
         c = Serial.read();
         // For debugging only 
         debug.print(c);
         decoded.concat(c); 
      }   
  
      int callIndex = decoded.indexOf("[");
      int callIndex2 = decoded.indexOf("]");
      String callsign = decoded.substring(callIndex+1,callIndex2);
      
      int dataIndex = decoded.indexOf("DATA:");
      String data = decoded.substring(dataIndex+6,decoded.length());

      String line2 = data.substring(0,20);
      String line3 = data.substring(21,40);
      
      //lcd.clear();
      lcd.setCursor(0,1);
      lcd.print("                    ");
      lcd.setCursor(0,1);
      lcd.print(callsign);
      lcd.setCursor(0,2);
      lcd.print("                    ");
      lcd.setCursor(0,2);      
      lcd.print(line2);
      lcd.setCursor(0,3);
      lcd.print("                    ");
      lcd.setCursor(0,3);
      lcd.print(line3);
      //delay(100);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
/* This is the old codes
void decodeAPRS() {
      char c;
      char endChar = '\n';
      char endChar2 = '\r';
      boolean storeString = true;
      char callsign[12];
      char path[60];
      char data[100];
      char charBuffer[199];

      //debug.println();
      //debug.print("Entering decodeAPRS (");
      //debug.print(millis());
      //debug.println(")");
      while ( Serial.available() > 0 ) {
         c = Serial.read();
         // For debugging only 
         debug.print(c);
         
         // If MAXBUFFER is reach, NULL terminate the string
         if ( bufferIndex > maxBuffer ) {
               charBuffer[maxBuffer] = 0;
               storeString = false;          
         }
         
         // Check for endChar and NULL terminate the string
         if ( c == endChar || c == endChar2 ) {
             charBuffer[bufferIndex] = 0;
             storeString = false;    
         }
         
         if ( storeString ) {
              charBuffer[bufferIndex++] = c;
         }
      }
      
      // Save buffers into Strings if charBuffer is not blank
      if ( !storeString  && (!charBuffer[0] == 0) ) { 
            inBuffer = charBuffer;
      }
      
      if ( inBuffer != "" && !storeString ) {
        debug.println();
        debug.print("RF(");
        debug.print(inBuffer.length());
        debug.print("):");
        debug.println(inBuffer);
        
        // Check for first 3 char is SRC
        if ( inBuffer.substring(0,3) == "SRC" ) {
      
        int firstBracket = inBuffer.indexOf('[');  // Anything in between [ and ] is the callsign & ssid
        int secondBracket = inBuffer.indexOf(']');

        //int secondColon = inBuffer.indexOf(':',secondBracket+1);  
        int secondColon = secondBracket+6;
        // Do not use lastindexOf as the messaging uses : too 
        int thirdColon = inBuffer.indexOf(':',secondColon+1);
        
        // Get the callsign
        String decoded2 = inBuffer.substring(firstBracket+1,secondBracket); // Substring the callsign
        decoded2.toCharArray(callsign,secondBracket+1-firstBracket-1);      // Convert to char array
        
        // Get the path
        String decoded3 = inBuffer.substring(secondColon+2,thirdColon-5);
        decoded3.toCharArray(path,decoded3.length()+1);
        
        // Get the data
        String decoded4 = inBuffer.substring(thirdColon+2,inBuffer.length());
        decoded4.toCharArray(data,decoded4.length()+1);

        debug.print("Callsign (");
        debug.print(strlen(callsign));
        debug.print("):");
        debug.println(callsign);    
             
        debug.print("Path (");
        debug.print(strlen(path));
        debug.print("):");
        debug.println(path);    

        debug.print("Data (");
        debug.print(strlen(data));
        debug.print("):");
        debug.println(data);    
        
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(callsign);
        
        lcd.setCursor(0,1);
        lcd.print("                    ");        
        lcd.setCursor(0,1);
        for ( int i=0;i<20;i++ ) { lcd.print(data[i]); }

        // Print on Line 2
        lcd.setCursor(0,2);
        lcd.print("                    ");        
        lcd.setCursor(0,2);        
        if ( strlen(data) < 40 ) {
            for ( int i=20;i<strlen(data);i++ ) { lcd.print(data[i]); }
        } else {
            for ( int i=20;i<40;i++ ) { lcd.print(data[i]); } // Truncate data till 40 bytes
        }
                 
        firstBracket, secondBracket, secondColon, thirdColon = 0;
        
       } // endif SRC check
       
        // clear the buffers after saving out all the variables like callsign, path and data
        inBuffer = "";      // Clear the buffers
        charBuffer[0] = 0;  // Clear the buffers
        bufferIndex = 0;    // Reset the index
        callsign[0] = 0;
        path[0] = 0;
        data[0] = 0;
       
      } // end storeString
}
*/
float convertDegMin(float decDeg) {
  
  float DegMin;
  
  int intDeg = decDeg;
  decDeg -= intDeg;
  decDeg *= 60;
  //int minDeg = decDeg;
  //decDeg -= minDeg;
  //decDeg *= 60;
  //int secDeg = decDeg;
  
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

int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

