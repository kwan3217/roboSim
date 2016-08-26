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
  virtual void start(int apid, char* pktName=nullptr) {};
  virtual void write(int8_t   value,          char* fieldName=nullptr) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(int16_t  value,          char* fieldName=nullptr) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(int32_t  value,          char* fieldName=nullptr) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(uint8_t  value,          char* fieldName=nullptr) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(uint16_t value,          char* fieldName=nullptr) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(uint32_t value,          char* fieldName=nullptr) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(float    value,          char* fieldName=nullptr) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(double   value,          char* fieldName=nullptr) {fwrite(&value,sizeof(value),1,stream);};
  virtual void write(char*    value, int len, char* fieldName=nullptr) {fwrite( value,len,          1,stream);};
  virtual void write(char*    value,          char* fieldName=nullptr) {fwrite( value,strlen(value),1,stream);};
  virtual void end() {};
};

#endif /* LOG_H_ */
