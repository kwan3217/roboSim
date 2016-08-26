#ifndef LogRawBinary_h
#define LogRawBinary_h

#include "Log.h"
#include <stdio.h>
#include <string.h> //for strlen()

/** Raw binary logging class. This just dumps data into a file in the native binary format
    of the writing machine. All descriptive information is ignored.
 */
class LogRawBinary: public Log {
private:
  FILE* stream;
public:
  LogRawBinary(char* filename);
  virtual ~LogRawBinary();
  virtual void start(char* pktName, int apid) {};
  virtual void write(char* fieldName, int8_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char* fieldName, int16_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char* fieldName, int32_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char* fieldName, uint8_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char* fieldName, uint16_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char* fieldName, uint32_t value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char* fieldName, float value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char* fieldName, double value) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char* fieldName, char* value, int len) {fwrite(value,len,1,stream);};
  virtual void write(char* fieldName, char* value) {fwrite(value,strlen(value),1,stream);};
  virtual void end() {};
};

#endif /* LOG_H_ */
