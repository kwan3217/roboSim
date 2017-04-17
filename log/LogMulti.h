#ifndef LogMulti_h
#define LogMulti_h

#include "Log.h"
#include <initializer_list>

/** Log packets into multiple streams
 */
template<int nLogs>
class LogMulti: public Log {
private:
  Log* logs[nLogs];
public:
  LogMulti(std::initializer_list<Log*> Llogs) {auto a=Llogs.begin();for(int i=0;i<nLogs;i++)logs[i]=a[i];};
  virtual ~LogMulti() {};
  virtual void start(int apid, const char* pktName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->start(apid,pktName);};
  virtual void write(int8_t      value,          const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,fieldName);};
  virtual void write(int16_t     value,          const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,fieldName);};
  virtual void write(int32_t     value,          const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,fieldName);};
  virtual void write(uint8_t     value,          const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,fieldName);};
  virtual void write(uint16_t    value,          const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,fieldName);};
  virtual void write(uint32_t    value,          const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,fieldName);};
  virtual void write(float       value,          const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,fieldName);};
  virtual void write(double      value,          const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,fieldName);};
  virtual void write(const char* value, int len, const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,len,fieldName);};
  virtual void write(const char* value,          const char* fieldName=nullptr) {for(int i=0;i<nLogs;i++) logs[i]->write(value,fieldName);};
  virtual void end() {for(int i=0;i<nLogs;i++) logs[i]->end();};
};

#endif /* LOG_H_ */
