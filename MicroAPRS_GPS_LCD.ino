#include <AltSoftSerial.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <LiquidCrystal.h>
// My wiring for LCD on breadboard
LiquidCrystal lcd(12, 11, 4, 5, 6, 7);

#define VERSION "Arduino MicroAPRS Tracker v0.2 " 

/* 
 
 MicroAPRS Arduino Interface Version 0.2
 
 This sketch configure the MicroAPRS for the proper callsign and ssid and
 read/write data coming in from the MicroAPRS via Serial port
 
 Pin 8/9 (rx,tx) connects to Arduino with MicroAPRS firmware
 Pin 2,3 ( rx,tx ) connects to GPS module
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
 
 I had dropped bytes when using SoftwareSerial for the MicroAPRS modem and therefore
 I'm using AltSoftSerial instead. The GPS module is still on SoftwareSerial and 
 the hardware serial is still used for Serial Monitor.
 
 ***** To use this sketch, pls change your CALLSIGN and SSID below under configModem().
 
 
*/

TinyGPSPlus gps;

// Connect to MicroAPRS on pin 2, 3 ( Rx, Tx )
//SoftwareSerial aprs(8, 9);

// AltSerial default on UNO is 8-Rx, 9-Tx
AltSoftSerial aprs;

// Connect to GPS module on pin 9, 10 ( Rx, Tx )
SoftwareSerial ss(2,3);

unsigned long last = 0UL;

// APRS buffers
String inBuffer = "";
int maxBuffer = 254;
char charBuffer[255];
byte bufferIndex=0;

char callsign[12];
char path[60];
char data[100];

unsigned long Txinterval = 60000;
unsigned long Txtimer;
String lastLat, lastLng;
String latOut,lngOut,cmtOut;
int lastCourse = 0;
int lastSpeed = 0;

void setup()
{
  // LCD format is Col,Row for 20x4 LCD
  lcd.begin(20,4);
  lcd.clear();

  lcd.setCursor(0,0);
  lcd.print(VERSION); 

  Serial.begin(57600);
  
  aprs.begin(9600);
  
  ss.begin(9600);

  Serial.print("ArduAPRS Track v0.2"); 
  Serial.println();
  
  delay(500);
  lcd.clear();
  
  // Set a delay for the MicroAPRS to boot up before configuring it
  delay(1000);
  configModem();
  
  unsigned long Txtimer = millis();

} // end setup()

void loop()
{
    float latDeg;
    float lngDeg;

    
    aprs.listen();                    // Turn on listen() on radio
    decodeAPRS(); 

    ss.listen();                    // Turn on listen() on GPS
    while ( ss.available() > 0 ) {
      gps.encode(ss.read());
    }
    
   if (gps.time.isUpdated()) {
    
     char tmp[10];
     
     latDeg = convertDegMinSec(gps.location.lat());
     lngDeg = convertDegMinSec(gps.location.lng());

     latOut="";
     lngOut="";
     
     dtostrf(latDeg, 2, 2, tmp );
     latOut.concat("lla0");      // set latitute command with the 0
     latOut.concat(tmp);
     latOut.concat("N");
     
     dtostrf(lngDeg, 2, 2, tmp );
     lngOut.concat("llo");       // set longtitute command
     lngOut.concat(tmp);
     lngOut.concat("E");
     
     cmtOut = "";
     cmtOut = "@ ";
     cmtOut.concat(VERSION);
     cmtOut.concat((float) readVcc()/1000);
     cmtOut.concat("V ");
     cmtOut.concat((int) gps.speed.kmph());
     cmtOut.concat("km/h");
     
     //Serial.println(cmtOut);
     
     lcd.setCursor(0,3);
     lcd.print("0");
     lcd.setCursor(1,3);
     lcd.print(convertDegMinSec(gps.location.lat()));  
    
     lcd.setCursor(8,3);
     lcd.print(convertDegMinSec(gps.location.lng()));  

     lcd.setCursor(18,3);   
     lcd.print("  "); 
     lcd.setCursor(18,3);   
     lcd.print(gps.satellites.value());
    
    //lcd.setCursor(16,0);
    //lcd.print(gps.course.deg());
    
    lcd.setCursor(15,0);
    lcd.print(gps.speed.kmph());

// Check for course change more than 30 degrees
//    if ( (gps.course.deg() - lastCourse > 30) || (lastCourse - gps.course.deg() > 30)  ) {
//          TxtoRadio();
//          Txtimer = millis(); 
//    }
    // Save current course as lastCourse
//    lastCourse = gps.course.deg();
    
// Check for higher speed, need to add delays after first Tx
//    if ( gps.speed.kmph() > 40 ) {
//          TxtoRadio();  
//          Txtimer = millis(); 
//    }

    } // endif
    
    
// Trigger Tx Tracker every 1 min     
//
// TODO, only Tx when there is a satellite lock, currently there are no such checks
// TODO, when speed is x, increase Tx frequencies to 30 sec
// TODO, when direction change > 30 deg, increase Tx frequencies to 30 sec
    
    if ( millis() - Txtimer  >= Txinterval ) {
       // Make sure we are not receiving any packets
      if ( aprs.available()==0 ) {          
           if ( !( lastLat.equals(latOut) && lastLng.equals(lngOut) ) ) {
                  TxtoRadio();
           }
       }
     // Copy lat & lng into lastLat,lastLng  
     lastLat="";
     lastLng="";
     lastLat = latOut;
     lastLng = lngOut;   
    } // End Trigger Tx timer

} // end loop()

void TxtoRadio() {
  
     unsigned long lastTx = millis() - Txtimer;
     Serial.print("Writing to radio since ");  
     Serial.print(lastTx); 
     Serial.println(" ms");    
     // Turn on the buzzer
     digitalWrite(10,HIGH);             
     aprs.println(latOut);
     delay(100);
     aprs.println(lngOut);
     delay(100);
     aprs.println(cmtOut);
     delay(300);
                 
     digitalWrite(10,LOW);          
     Txtimer = millis(); 
}

void configModem() {
// Functions to configure the callsign, ssid, path and other settings
// c<callsign>
// sc<ssid>
// pd0 - turn off DST display

  lcd.setCursor(0,0);
  lcd.print("Configuring modem");

  aprs.println("V0");
  delay(200);

  aprs.println("cNOCALL");  // Set your Callsign here, c is the firmware command
  delay(200);
  
  aprs.println("sc8");      // Set your SSID here, I'm using 8, sc is the command
  delay(200);

  aprs.println("pd0");      // Disable printing DST 
  delay(200);

  //aprs.println("ls?");      // Desktop symbol
  //delay(200);
  
  //aprs.println("lts");      // Desktop symbol
  //delay(200);
  
  //aprs.println("V1");      // Silent Mode Enable
  //delay(200);  

  aprs.println("V1");        // Enable Silent more
  delay(200);
  
  //aprs.println("S");        // Save config
  
  lcd.setCursor(0,0);
  lcd.print("Done...........   ");
  delay(500); 
  lcd.clear();
  
}

void decodeAPRS() {
      char c;
      char endChar = '\n';
      char endChar2 = '\r';
      boolean storeString = true;

      while ( aprs.available() > 0 ) {
         c = aprs.read();
         
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
        Serial.println();
        Serial.print("inBuffer:");
        Serial.println(inBuffer);
      
        // Check for first 3 char is SRC
        if ( inBuffer.substring(0,3) == "SRC" ) {
      
        int firstBracket = inBuffer.indexOf('[');  // Anything in between [ and ] is the callsign & ssid
        int secondBracket = inBuffer.indexOf(']');
  
        int secondColon = inBuffer.indexOf(':',secondBracket+1);
        int thirdColon = inBuffer.lastIndexOf(':');
        
        // Get the callsign
        String decoded2 = inBuffer.substring(firstBracket+1,secondBracket); // Substring the callsign
        decoded2.toCharArray(callsign,secondBracket+1-firstBracket-1);      // Convert to char array
        
        // Get the path
        String decoded3 = inBuffer.substring(secondColon+2,thirdColon-5);
        decoded3.toCharArray(path,decoded3.length()+1);

        String decoded4 = inBuffer.substring(thirdColon+2,inBuffer.length());
        decoded4.toCharArray(data,decoded4.length()+1);

        Serial.print("Callsign:");
        Serial.println(callsign);    
        
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print(callsign);
        
        Serial.print("Path:");
        Serial.println(path);    

        Serial.print("Data:");
        Serial.println(data);    

        lcd.setCursor(0,1);
        for ( int i=0;i<20;i++ ) { lcd.print(data[i]); }

        // Print on Line 2
        lcd.setCursor(0,2);
        if ( strlen(data) < 40 ) {
            for ( int i=20;i<strlen(data);i++ ) { lcd.print(data[i]); }
        } else if ( strlen(data) < 60 ) {
            for ( int i=20;i<40;i++ ) { lcd.print(data[i]); }
            
// Reserve last line for GPS info
            // Print on Line 3
            //lcd.setCursor(0,3);
            //for ( int i=40;i<strlen(data);i++ ) { lcd.print(data[i]); }
        } else {
            // Truncate to max 60 chars
            for ( int i=20;i<40;i++ ) { lcd.print(data[i]); }
            
// Reserve last line for GPS info
            // Print on Line 3
            //lcd.setCursor(0,3);
            //for ( int i=40;i<60;i++ ) { lcd.print(data[i]); }
        }
                 
        
       } // endif SRC check
       
        // clear the buffers after saving out all the variables like callsign, path and data
        inBuffer = "";      // Clear the buffers
        charBuffer[0] = 0;  // Clear the buffers
        bufferIndex = 0;    // Reset the index
        callsign[0]=0;
        path[0]=0;
        data[0] = 0;
       
      } // end storeString
}

float convertDegMinSec(float decDeg) {
  
  float DegMinSec;
  
  int intDeg = decDeg;
  decDeg -= intDeg;
  decDeg *= 60;
  int minDeg = decDeg;
  decDeg -= minDeg;
  decDeg *= 60;
  int secDeg = decDeg;
  
  DegMinSec = ( intDeg*100 ) + minDeg + ( (float)secDeg / 100 );
 
 return DegMinSec; 
}

// Fucntion to read Arduino hidden voltmeter
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
