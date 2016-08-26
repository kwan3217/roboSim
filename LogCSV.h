#ifndef LogCSV_h
#define LogCSV_h

#include "Log.h"

#include <stdio.h>

/** Log packets into multiple CSV files, one for each apid. Documentation is in the form of column headers for each file.
 */
class LogCSV: public Log {
private:
  static const int n_apid=64;
  static const int bufSize=256;
  char buf[bufSize];
  FILE* stream[n_apid];
  FILE* fbuf; //Used to write to buf with fprintf
  int current_apid;
  bool hasDoc[n_apid];
  bool firstField;
  void writeDoc(char* fieldName) {
    if(!hasDoc[current_apid] && fieldName!=nullptr) {
	  fprintf(stream[current_apid],firstField?"%s":",%s",fieldName);
	}
  }
public:
  LogCSV(int n_streams, char** filenames, bool* buffer=nullptr);
  virtual ~LogCSV();
  virtual void start(int apid, char* pktName=nullptr);
  virtual void write(int8_t   value, char* fieldName=nullptr);
  virtual void write(int16_t  value, char* fieldName=nullptr);
  virtual void write(int32_t  value, char* fieldName=nullptr);
  virtual void write(uint8_t  value, char* fieldName=nullptr);
  virtual void write(uint16_t value, char* fieldName=nullptr);
  virtual void write(uint32_t value, char* fieldName=nullptr);
  virtual void write(float    value, char* fieldName=nullptr);
  virtual void write(double   value, char* fieldName=nullptr);
  virtual void write(char*    value, int len, char* fieldName=nullptr);
  virtual void write(char*    value, char* fieldName=nullptr);
  virtual void end();
};

#endif /* LOG_H_ */
