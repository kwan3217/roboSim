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
  void writeDoc(const char* fieldName) {
    if(!hasDoc[current_apid] && fieldName!=nullptr) {
	  fprintf(stream[current_apid],firstField?"%s":",%s",fieldName);
	}
  }
public:
  LogCSV(int n_streams, const char* const* filenames, const bool* buffer=nullptr);
  virtual ~LogCSV();
  virtual void start(int apid,                const char* pktName=nullptr);
  virtual void write(int8_t      value,          const char* fieldName=nullptr) {write(( int32_t)value,fieldName);};
  virtual void write(int16_t     value,          const char* fieldName=nullptr) {write(( int32_t)value,fieldName);};
  virtual void write(int32_t     value,          const char* fieldName=nullptr) {writeDoc(fieldName);fprintf(fbuf,firstField?"%d":",%d",value);  firstField=false;};
  virtual void write(uint8_t     value,          const char* fieldName=nullptr) {write((uint32_t)value,fieldName);};
  virtual void write(uint16_t    value,          const char* fieldName=nullptr) {write((uint32_t)value,fieldName);};
  virtual void write(uint32_t    value,          const char* fieldName=nullptr) {writeDoc(fieldName);fprintf(fbuf,firstField?"%u":",%u",value);  firstField=false;};
  virtual void write(float       value,          const char* fieldName=nullptr) {writeDoc(fieldName);fprintf(fbuf,firstField?"%f":",%f",value);  firstField=false;};
  virtual void write(double      value,          const char* fieldName=nullptr) {writeDoc(fieldName);fprintf(fbuf,firstField?"%f":",%f",value);  firstField=false;};
  virtual void write(const char* value, int len, const char* fieldName=nullptr);
  virtual void write(const char* value,          const char* fieldName=nullptr);
  virtual void end();
};

#endif /* LOG_H_ */
