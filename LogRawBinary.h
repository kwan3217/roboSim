#ifndef LogRawBinary_h
#define LogRawBinary_h

#include "Log.h"
#include <stdio.h>
#include <string.h> //for strlen()

class LogRawBinary: public Log {
private:
  FILE* stream;
public:
  LogRawBinary(char* filename);
  virtual ~LogRawBinary();
  virtual void startPacket(int apid) {};
  virtual void write(int8_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(int16_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(int32_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(uint8_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(uint16_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(uint32_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(float value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char* value, int len) {fwrite(value,len,1,stream);};
  virtual void write(char* value) {fwrite(value,strlen(value),1,stream);};
  virtual void endPacket(int apid) {};
  virtual void startDescribe(int apid) {};
  virtual void endDescribe(int apid) {};
  virtual void describe(char* name, int type) {};
};

#endif /* LOG_H_ */
