/**
 * MicroAPRS Library, Modified by DB1NTO
 * Source: ArgentRadioShield Library
 * Copyright 2012 Leigh L. Klotz, Jr. WA5ZNU
 * Released under dual license:
 * MIT License http://www.opensource.org/licenses/mit-license
 * LGPL 3.0: http://www.gnu.org/licenses/lgpl-3.0.txt
 */

#include "Arduino.h"
#include "Stream.h"
#include <MicroAPRS.h>

MicroAPRS::MicroAPRS(Stream *s) {
  stream = s;
  defaultDestination="APOTW1";
}


int MicroAPRS::read() {
  return stream->read();
}

int MicroAPRS::available() {
  return stream->available();
}


boolean MicroAPRS::decode_posit(char *packet, char **pcall, char *ptype, char **pposit, long *plon, long *plat, char **pcomment, char **pmsgTo, char **pmsg, char *pmsgID) {
  char *callsignBegin = packet+5;
  *callsignBegin++ = 0;
  char *callsignEnd = strchr(callsignBegin, ']');
  if (callsignEnd == NULL || callsignEnd-callsignBegin > 16)
    return false;
  *callsignEnd = 0;
  char *call = callsignBegin;
  char *destination = callsignEnd+8;
  char *payload_begin = strchr(destination, ':')+1;
  if (payload_begin == NULL) 
    return false;
  *payload_begin++ = 0;
  // Type is !, =, @, /, ', `.  
  // We only decode positions.
  char type = *payload_begin;
  payload_begin++;
  *pcall = call;
  *ptype = type;

  char *posit = payload_begin;
  {
    if (type == '/' || type == '@') {
      // no compression, but starts with time
      // advanced past type and fall through to pretend '/' and '@' are just '!'
      // advance past "195537h" or '052139z'
      posit += 6;
      // call N5CV packet type / 3721.40N/12204.64W
      posit++;
      posit[19] = 0;
      type = '!';
    }

    if (type == '`' || type == '\'') {
      // MIC-E compression
        char *positEnd = posit+8;
      	if (posit[12] == '}'){
      		positEnd = posit+13;
      	}      	
		char *comment = positEnd++;
		//*positEnd++ = 0;
		*pcomment = comment;
		
      if (! decode_mic_e(destination, posit, plat, plon))
	return false;
    } else if (type == ':') {
		//Message
		char *msgToEnd = strchr(payload_begin, ':');
		*msgToEnd++ = 0;
		char *msgTo = payload_begin;
		
		char *msgEnd = strchr(msgToEnd, '{');
		char *msg = msgToEnd++;
		*msgEnd++ = 0;
		
		char msgID = *msgEnd++;
		
		*pmsgTo = msgTo;
		*pmsg = msg;
		*pmsgID = msgID;
		
		
    } else if (type == '!' || type == '=') {
	// No compression or base91 compression
	// call KC6SSM-5 packet type ! 3754.15NI12216.92W&
	// call N6MON-9 packet type ! 3741.84N/12202.85W
	// call KC6SSM-9 packet type ! 3739.29N/12205.34W>

	// either !/ or !\ and 12 more characters
	// or ![0-9] and 17 more characters
	// if there is a space that terminates it
	// otherwise terminate at 18 chars or eol, whichever is first
	// terminated by space or EOL (length "3741.84N/12202.85W")
	char sym1 = *posit;
	char sym2 = ' ';
	char *positEnd = strchr(posit, ' ');
	char *comment = positEnd+1;
	//*positEnd++ = 0;	
	*pcomment = comment;
	
		if (sym1 == '/' || sym1 == '\\') {
		// Base91 Compressed
		posit[13] = 0;
		decode91(posit, plat, plon, &sym2);
		} else 	if (posit[1] >= '0' && posit[1] <= '9') {
		posit[18] = 0;
		decode_latlon(posit, plat, plon, &sym2);
		} else {
		return false;
		}
    } else if (type == '>' ) {
	// Check for status type
	// Everything after > is a comment
	// e.g. >SVTrackR v1.0 5.24V S:9 B:0.51 U:21.32 Seq:70
	char *comment = posit;
	*pcomment = comment;

	} else {
      return false;
    }
  }
  *pposit = posit;
  return true;
}



void MicroAPRS::decode91(char *data, long *plat, long *plon, char *sym2) {
  // *sym1 = data[0];
  long lon = 0;
  long lat = 0;
  {
    for (byte i = 1; i < 5; i++) {
      lat = lat*91 + (data[i]-33);
    }
  }
  {
    for (byte i = 5; i < 9; i++) {
      lon = lon*91 + (data[i]-33);
    }
  }

  *plat = 90e6 - ((lat * 1e7) + 5) / 3809260;
  *plon = -180e6 + ((lon * 1e7) + 5) / 1904630;
  *sym2 = data[9];
}

#define DIGIT(x) (x-'0')
void MicroAPRS::decode_latlon(char *data, long *plat, long *plon, char *sym2) {
  {
    // 3741.84N/12202.85W
    // 37 41.84N -> 37.69733 -> 3,769,733 ; (+ 3e7 7e6 (/ 4e7 60) (/ 1e6 60) (/ 8e5 60) (/ 4e4 60))
    // 122 02.85W -> -122.04750  -> -12,204,750
    long lat = DIGIT(*data++) * 1e7;
    lat += DIGIT(*data++) * 1e6;
    lat += DIGIT(*data++) * 1e7 / 60;
    lat += DIGIT(*data++) * 1e6 / 60;
    data++;			// '.'
    lat += DIGIT(*data++) * 1e5 / 60;
    lat += DIGIT(*data++) * 1e4 / 60;
    if (*data++=='S') lat = -lat;
    *plat = lat;
  }
  *sym2 = *data++;
  {
    long lon = DIGIT(*data++) * 1e8;
    lon += DIGIT(*data++) * 1e7;
    lon += DIGIT(*data++) * 1e6;
    lon += DIGIT(*data++) * 1e7 / 60;
    lon += DIGIT(*data++) * 1e6 / 60;
    data++;			// '.'
    lon += DIGIT(*data++) * 1e5 / 60;
    lon += DIGIT(*data++) * 1e4 / 60;
    if (*data++=='W') lon = -lon;
    *plon = lon;
  }
}

boolean MicroAPRS::decode_mic_e(char *destination, char *posit, long *plat, long *plon) {
  char *lastdash = strchr(destination, '-');
  if (lastdash != NULL) {
    *lastdash = 0;
  }
  if (strlen(destination) != 6) return false;
        
  // latitude
  {
    int d = ((miceChar(destination[0])) * 10) + (miceChar(destination[1]));
    int m = ((miceChar(destination[2])) * 10) + (miceChar(destination[3]));
    int s = ((miceChar(destination[4])) * 10) + (miceChar(destination[5]));
    long lat = (d*1000000) + (m*1000000 / 60) + (s*1000000 / 6000);

    if (! (miceFlag(destination[3]))) 
      lat = -lat;

    *plat = lat;
  }
    
  // longitude
  {
    int d = posit[0] - 28;
    int m = posit[1] - 28;
    int s = posit[2] - 28;

    if (d < 0 || d > 99 || m < 0 || m > 99 || s < 0 || s > 99) return false;

    if (miceFlag(destination[4])) d += 100;

    if (d >= 190) d -= 190; 
    else if (d >= 180) d -= 80;
    if (m >= 60) m -= 60;

    long lon = (d*1000000) + (m*1000000 / 60) + (s*1000000 / 6000);

    if (miceFlag(destination[5]))
      lon = - lon;
    *plon = lon;
  }
  return true;
}

// From Chris K6DBG
int MicroAPRS::miceChar(char c) {
  c -= 0x30;				   // adjust to be 0 based
  if (c == 0x1c) { c = 0x0a; }		// change 'L' to space
  if (c > 0x10 && c <= 0x1b) { c--; }		// Decrement A-K
  if ((c & 0x0f) == 0x0a) { c &= 0xf0; }	// convert space to 0
  return (c & 0xf);
}

boolean MicroAPRS::miceFlag(char c) {
  return (c > 0x50);
}

