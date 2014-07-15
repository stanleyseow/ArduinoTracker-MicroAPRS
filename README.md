ArduinoTracker-MicroAPRS
========================

 - Date : 03 July 2014
 - Written by Stanley Seow
 - e-mail : stanleyseow@gmail.com
 
 *** Pls refer to the top of the source code for version history and updated instructions.
 

Arduino TrackR interface for MicroAPRS firmware

 This sketch configure the MicroAPRS for the proper callsign and ssid and
 read/write data coming in from the MicroAPRS (another Arduino or atmega328 known as the modem) 
 via Serial port
 
 Connection to Arduino :-
 Pin 0/1 (rx,tx) connects to Arduino with MicroAPRS firmware
 Pin 2,3 ( rx,tx ) connects to GPS module 
 Pin 8,9 ( rx,tx ) connects to a FTDI Serial USB module with OTG calbe to my Android phone with FTDI 
 Terminal software for logging and debugging during the test runs. 
 
 Optional accessories: 20x4 LCD, buzzer & FTDI Serial USB module
 Pin 4,5,6,7,12,11 connects to 20x4 LCD 
 Pin 10 - Buzzer during Radio Tx
 Pin 8,9 - Connect to FTDI Serial USB module for debugging
 
 Instructions :-
 
 Pls flash the modem Arduino with http://unsigned.io/microaprs/ with USBtinyISP or 
 any ISP flashing tools.
 
 This firmware does NOT have an Arduino bootloader and run pure AVR hex. Once you
 have done this, follow the instructions on the above URL on how to set it up.
 You can console into this modem, change or save settings to the EEPROM or even 
 send messages.

 To use with Arduino TrackR (this sketch), connect the Modem Tx(pin1) to Arduino Rx(pin0) and 
 Modem Rx(pin0) to Arduino Rx(pin1).
 
 *** The Serial connection are Rx-Tx and Tx-Rx
 *** Disconnect the Pin 0,1 during Arduino sketch upload
 
 I had experience dropped bytes when using SoftwareSerial for the MicroAPRS modem and therefore
 I'm using Hardware Serial instead. The GPS module is still on SoftwareSerial and 
 the Altserial is still used for Serial Monitor / debug.print() 
 
 ***** To use this sketch, pls change your CALLSIGN and SSID below under configModem().
 
 
 
