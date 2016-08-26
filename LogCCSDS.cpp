#include "LogCCSDS.h"

LogCCSDS::LogCCSDS(char* basename, int LdocApid):docApid(LdocApid) {
  char filename[256];
  snprintf(filename,sizeof(filename)-1,"%s/%s",recordPath,basename);
  stream=fopen(filename,"w");
  setbuf(stream,nullptr); //Turn off buffering for this file
}

LogCCSDS::~LogCCSDS() {
  fclose(stream);
}

void LogCCSDS::writeDoc(int type, char* fieldName) {
  if(fieldName==nullptr) return;
  if(hasDoc[pktApid]) return;
  start(docBuf,docPtr,docApid);
  write(docBuf,docPtr,(uint16_t)pktApid);
  write(docBuf,docPtr,(uint16_t)pktPtr);
  write(docBuf,docPtr,(uint8_t)type);
  write(docBuf,docPtr,fieldName);
  end(docBuf,docPtr,docApid);
}

void LogCCSDS::start(char* buf, int& ptr, int apid) {
  writeBuf_be<uint16_t>(buf,0,apid);
  writeBuf_be<uint16_t>(buf,2,0xC000 | seq[apid]);
  seq[apid]++;
  if(seq[apid]>=0xC000) seq[apid]=0;
  ptr=6;
}

void LogCCSDS::end(char* buf, int& ptr, int apid) {
  hasDoc[apid]=true;
  writeBuf_be<uint16_t>(buf,4,ptr-7);
  fwrite(buf,ptr,1,stream);
}
