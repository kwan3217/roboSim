#ifndef LogRecordGyro_h
#define LogRecordGyro_h

#include "Log.h"

#include <stdio.h>

class LogRecordGyro: public Log {
private:
  FILE* stream[2];
  int current_apid;
  bool firstField;
public:
  LogRecordGyro();
  virtual ~LogRecordGyro();
  virtual void startPacket(int apid);
  virtual void write(int8_t value);
  virtual void write(int16_t value);
  virtual void write(int32_t value);
  virtual void write(uint8_t value);
  virtual void write(uint16_t value);
  virtual void write(uint32_t value);
  virtual void write(float value);
  virtual void write(char* value, int len);
  virtual void write(char* value);
  virtual void endPacket(int apid);
  virtual void startDescribe(int apid);
  virtual void endDescribe(int apid);
  virtual void describe(char* name, int type);
};

#endif /* LOG_H_ */
