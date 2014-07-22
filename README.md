ArduinoTracker-MicroAPRS
========================

 - Date : 03 July 2014
 - Written by Stanley Seow
 - e-mail : stanleyseow@gmail.com
 
 *** Pls refer to the top of the source code for version history and updated instructions.
 

Arduino TrackR interface for MicroAPRS firmware

 This sketch configure the MicroAPRS (the modem) for the proper callsign and ssid and read/write data 
 coming in from the MicroAPRS (another Arduino/atmega328P with bootloader) via Serial port
 
 Connection to Arduino :-
 - Pin 0/1 (rx,tx) connects to MicroAPRS ( Modem.hex firmware )
 - Pin 2,3 ( rx,tx ) connects to GPS module 
 - Pin 8,9 ( rx,tx ) connects to a FTDI Serial USB module with OTG calbe to my Android phone with FTDI 
 Terminal software for logging and debugging during the test runs. 
 
 Optional accessories: 20x4 LCD, buzzer & FTDI Serial USB module
 - Pin 4,5,6,7,12,11 connects to 20x4 LCD 
 - Pin 10 - Buzzer during Radio Tx
 - Pin 8,9 - Connect to FTDI Serial USB module for debugging
 
 Instructions :-
 
 Download the MicroAPRS compiled hex called ( https://github.com/markqvist/MicroAPRS/tree/master/images )
 Modem.hex from the above URL.
 
 As this hex is not compiled from the Arduino IDE, you need to manually upload this hex to the Arduino
 with a bootloader.
 
 Step 1 : Copy the avrdude.conf from Arduino folder to the current directory 
 ( c:\Arduino\hardware\tools\avr\etc\avrdude.conf )
 
 Step 2: 
 The command line are :-
 
 avrdude -C ./avrdude.conf -v -patmega328p -carduino -P /dev/tty.usbmodem1411 -b115200 -D -Uflash:w:Modem.hex:i
 
Below are the sample output during the hex upload process :-

========= Sample Output ============= 


avrdude: AVR device initialized and ready to accept instructions

Reading | ################################################## | 100% 0.00s

avrdude: Device signature = 0x1e950f
avrdude: safemode: lfuse reads as 0
avrdude: safemode: hfuse reads as 0
avrdude: safemode: efuse reads as 0
avrdude: reading input file "Modem.hex"
avrdude: writing flash (28478 bytes):

Writing | ################################################## | 100% 4.56s

avrdude: 28478 bytes of flash written
avrdude: verifying flash memory against Modem.hex:
avrdude: load data flash data from input file Modem.hex:
avrdude: input file Modem.hex contains 28478 bytes
avrdude: reading on-chip flash data:

Reading | ################################################## | 100% 3.65s

avrdude: verifying ...
avrdude: 28478 bytes of flash verified

avrdude: safemode: lfuse reads as 0
avrdude: safemode: hfuse reads as 0
avrdude: safemode: efuse reads as 0
avrdude: safemode: Fuses OK

avrdude done.  Thank you.

========= Sample Output ============= 
 
Once the hex upload is successful, you will see the below prompt if you connect to the modem using 
9600 baud rate.
 
------------------
MicroAPRS v0.2a
unsigned.io/microaprs
Default configuration loaded!
Modem ready
---------------

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
 
 
 
