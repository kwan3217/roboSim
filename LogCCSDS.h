#ifndef LogCCSDS_h
#define LogCCSDS_h

#include "Log.h"
#include <stdio.h>
#include "buffer.h" //for writeBuf_be<>()
#include <string.h> //for strlen()

class LogCCSDS: public Log {
private:
  static const int n_apid=64;
  static const int maxPktSize=256;
  FILE* stream;
  char pktBuf[maxPktSize];
  int seq[n_apid];
  bool hasDoc[n_apid];
  char docBuf[maxPktSize];
  int pktPtr,docPtr;
  int pktApid,docApid;
  void writeDoc(int type, char* fieldName);
  void writeDoc(          char*   pktName) {writeDoc(0,pktName);};
  void start(char* buf, int& ptr, int apid);
  void write(char* buf, int& ptr, int8_t   value) {buf[ptr]=value;              ptr+=sizeof(value);};
  void write(char* buf, int& ptr, int16_t  value) {writeBuf_be(buf,ptr,value);  ptr+=sizeof(value);};
  void write(char* buf, int& ptr, int32_t  value) {writeBuf_be(buf,ptr,value);  ptr+=sizeof(value);};
  void write(char* buf, int& ptr, uint8_t  value) {buf[ptr]=value;              ptr+=sizeof(value);};
  void write(char* buf, int& ptr, uint16_t value) {writeBuf_be(buf,ptr,value);  ptr+=sizeof(value);};
  void write(char* buf, int& ptr, uint32_t value) {writeBuf_be(buf,ptr,value);  ptr+=sizeof(value);};
  void write(char* buf, int& ptr, float    value) {writeBuf_be(buf,ptr,value);  ptr+=sizeof(value);};
  void write(char* buf, int& ptr, double   value) {writeBuf_be(buf,ptr,value);  ptr+=sizeof(value);};
  void writeBinary(char* buf, int& ptr, char*    value, int len) {for(int i=0;i<len;i++) write(buf,ptr,value[i]);};
  void writeString(char* buf, int& ptr, char*    value) {writeBinary(buf,ptr,value,strlen(value));};
  void end(char* buf, int& ptr, int apid);
public:
  LogCCSDS(char* filename, int LdocApid=0);
  virtual ~LogCCSDS();
  virtual void start(int apid, char* pktName=nullptr) {writeDoc(pktName);start(pktBuf,pktPtr,apid);};
  virtual void write(int8_t   value,          char* fieldName=nullptr) {writeDoc(t_u8    ,fieldName);write(pktBuf,pktPtr,value);};
  virtual void write(int16_t  value,          char* fieldName=nullptr) {writeDoc(t_i16   ,fieldName);write(pktBuf,pktPtr,value);};
  virtual void write(int32_t  value,          char* fieldName=nullptr) {writeDoc(t_i32   ,fieldName);write(pktBuf,pktPtr,value);};
  virtual void write(uint8_t  value,          char* fieldName=nullptr) {writeDoc(t_u8    ,fieldName);write(pktBuf,pktPtr,value);};
  virtual void write(uint16_t value,          char* fieldName=nullptr) {writeDoc(t_u16   ,fieldName);write(pktBuf,pktPtr,value);};
  virtual void write(uint32_t value,          char* fieldName=nullptr) {writeDoc(t_u32   ,fieldName);write(pktBuf,pktPtr,value);};
  virtual void write(float    value,          char* fieldName=nullptr) {writeDoc(t_float ,fieldName);write(pktBuf,pktPtr,value);};
  virtual void write(double   value,          char* fieldName=nullptr) {writeDoc(t_double,fieldName);write(pktBuf,pktPtr,value);};
  virtual void write(char*    value, int len, char* fieldName=nullptr) {writeDoc(t_binary,fieldName);writeBinary(pktBuf,pktPtr,value,len);};
  virtual void write(char*    value,          char* fieldName=nullptr) {writeDoc(t_string,fieldName);writeString(pktBuf,pktPtr,value);};
  virtual void end() {end(pktBuf,pktPtr,pktApid);};
};

#endif /* LOG_H_ */
