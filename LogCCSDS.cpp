#include "LogCCSDS.h"
#include "buffer.h"
#include <string.h>

LogCCSDS::LogCCSDS(char* basename) {
  char filename[256];
  snprintf(filename,sizeof(filename)-1,"%s/%s",recordPath,basename);
  stream=fopen(filename,"w");
  setbuf(stream,nullptr); //Turn off buffering for this file
}

LogCCSDS::~LogCCSDS() {
  fclose(stream);
}

void LogCCSDS::startPacket(int apid) {
  writeBuf_be<uint16_t>(buf,0,apid);
  writeBuf_be<uint16_t>(buf,2,0xC000 | seq[apid]);
  seq[apid]++;
  if(seq[apid]>=0xC000) seq[apid]=0;
  ptr=6;
}

void LogCCSDS::write(int8_t value) {
  buf[ptr]=value;
  ptr++;
}

void LogCCSDS::write(int16_t value) {
  writeBuf_be(buf,ptr,value);
  ptr+=sizeof(value);
}

void LogCCSDS::write(int32_t value) {
  writeBuf_be(buf,ptr,value);
  ptr+=sizeof(value);
}

void LogCCSDS::write(uint8_t value) {
  buf[ptr]=value;
  ptr++;
}

void LogCCSDS::write(uint16_t value) {
  writeBuf_be(buf,ptr,value);
  ptr+=sizeof(value);
}

void LogCCSDS::write(uint32_t value) {
  writeBuf_be(buf,ptr,value);
  ptr+=sizeof(value);
}

void LogCCSDS::write(float value) {
  writeBuf_be(buf,ptr,value);
  ptr+=sizeof(value);
}

void LogCCSDS::write(char* value, int len) {
  for(int i=0;i<len;i++) {
    buf[ptr+i]=value[i];
  }
  ptr+=len;
}

void LogCCSDS::write(char* value) {
  write(value,strlen(value));
}

void LogCCSDS::endPacket() {
  writeBuf_be<uint16_t>(buf,4,ptr-7);
  fwrite(buf,ptr,1,stream);
}
