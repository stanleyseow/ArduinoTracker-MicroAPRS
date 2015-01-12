#ifndef MicroAPRS_h
#define MicroAPRS_h

#include "Stream.h"

class MicroAPRS {
 public:
  MicroAPRS(Stream *stream);
  int available();
  int read();
  boolean decode_posit(char *packet, char **pcall, char *ptype, char **pposit, long *plon, long *plat, char **pcomment, char **pmsgTo, char **pmsg, char *pmsgID);
 private:
  char *defaultDestination;
  Stream *stream;
  void decode_latlon(char *data, long *plat, long *plon, char *sym2);
  void decode91(char *data, long *plat, long *plon, char *sym2);
  boolean decode_mic_e(char *destination, char *posit, long *plat, long *plon);
  int miceChar(char c);
  boolean miceFlag(char c);
};

#endif
