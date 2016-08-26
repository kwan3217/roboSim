#ifndef LogCCSDS_h
#define LogCCSDS_h

#include "Log.h"
#include <stdio.h>

class LogCCSDS: public Log {
private:
  static const int n_apid=64;
  static const int maxPktSize=256;
  FILE* stream;
  char buf[maxPktSize];
  int seq[n_apid];
  bool hasDesc[n_apid];
  int ptr;
public:
  LogCCSDS(char* filename);
  virtual ~LogCCSDS();
  virtual void start(char* pktName, int apid);
  virtual void write(char* fieldName, int8_t value);
  virtual void write(char* fieldName, int16_t value);
  virtual void write(char* fieldName, int32_t value);
  virtual void write(char* fieldName, uint8_t value);
  virtual void write(char* fieldName, uint16_t value);
  virtual void write(char* fieldName, uint32_t value);
  virtual void write(char* fieldName, float value);
  virtual void write(char* fieldName, double value);
  virtual void write(char* fieldName, char* value, int len);
  virtual void write(char* fieldName, char* value);
  virtual void end();
};

#endif /* LOG_H_ */
