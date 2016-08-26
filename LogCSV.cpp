#include "LogCSV.h"

LogCSV::LogCSV(int n_streams, char** basename, bool* buffer) {
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

void LogCSV::start(int apid, char* pktName) {
  current_apid=apid;
  firstField=true;
  if(hasDoc[apid]) {
	fbuf=stream[current_apid];
  } else {
	fbuf=fmemopen(buf,sizeof(buf),"w");
  }
}

void LogCSV::write(int8_t value, char* fieldName) {
  writeDoc(fieldName);
  fprintf(fbuf,firstField?"%d":",%d",value);
  firstField=false;
}

void LogCSV::write(int16_t value, char* fieldName) {
  writeDoc(fieldName);
  fprintf(fbuf,firstField?"%d":",%d",value);
  firstField=false;
}

void LogCSV::write(int32_t value, char* fieldName) {
  writeDoc(fieldName);
  fprintf(fbuf,firstField?"%d":",%d",value);
  firstField=false;
}

void LogCSV::write(uint8_t value, char* fieldName) {
  writeDoc(fieldName);
  fprintf(fbuf,firstField?"%u":",%u",value);
  firstField=false;
}

void LogCSV::write(uint16_t value, char* fieldName) {
  writeDoc(fieldName);
  fprintf(fbuf,firstField?"%u":",%u",value);
  firstField=false;
}

void LogCSV::write(uint32_t value, char* fieldName) {
  writeDoc(fieldName);
  fprintf(fbuf,firstField?"%u":",%u",value);
  firstField=false;
}

void LogCSV::write(float value, char* fieldName) {
  writeDoc(fieldName);
  fprintf(fbuf,firstField?"%f":",%f",value);
  firstField=false;
}

void LogCSV::write(double value, char* fieldName) {
  writeDoc(fieldName);
  fprintf(fbuf,firstField?"%f":",%f",value);
  firstField=false;
}

void LogCSV::write(char* value, int len, char* fieldName) {
  writeDoc(fieldName);
  if(!firstField) fprintf(fbuf,",");
  for(int i=0;i<len;i++) fprintf(fbuf,"%02x",value[i]);
  firstField=false;
}

void LogCSV::write(char* value, char* fieldName) {
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


