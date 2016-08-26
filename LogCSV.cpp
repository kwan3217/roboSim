#include "LogCSV.h"

LogCSV::LogCSV(int n_streams, const char* const* basename, const bool* buffer) {
  char filename[256];
  for(int i=0;i<n_streams;i++) if(basename[i]!=nullptr) {
    snprintf(filename,sizeof(filename)-1,"%s/%s",recordPath,basename[i]);
    stream[i]=fopen(filename,"w");
    if(buffer==nullptr || !buffer[i]) setbuf(stream[i],nullptr);//turn off buffering on this file
  } else {
	stream[i]=nullptr;
  }
}

LogCSV::~LogCSV() {
  for(int i=0;i<n_apid;i++) if(stream[i]!=nullptr) fclose(stream[i]);
}

void LogCSV::start(int apid, const char* pktName) {
  current_apid=apid;
  firstField=true;
  if(hasDoc[apid]) {
	fbuf=stream[current_apid];
  } else {
	fbuf=fmemopen(buf,sizeof(buf),"w");
  }
}

void LogCSV::write(const char* value, int len, const char* fieldName) {
  writeDoc(fieldName);
  if(!firstField) fprintf(fbuf,",");
  for(int i=0;i<len;i++) fprintf(fbuf,"%02x",value[i]);
  firstField=false;
}

void LogCSV::write(const char* value, const char* fieldName) {
  writeDoc(fieldName);
  fprintf(fbuf,firstField?"%s":",%s",value);
  firstField=false;
}

void LogCSV::end() {
  if(!hasDoc[current_apid]) {
	fclose(fbuf);
	fputs(buf,stream[current_apid]);
  }
  fprintf(stream[current_apid],"\n");
  hasDoc[current_apid]=true;
}


