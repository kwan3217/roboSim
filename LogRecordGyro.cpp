#include "LogRecordGyro.h"

LogRecordGyro::LogRecordGyro() {
  char filename[256];
  snprintf(filename,sizeof(filename)-1,"%s/record.csv",recordPath);
  stream[0]=fopen(filename,"w");
  snprintf(filename,sizeof(filename)-1,"%s/mpuconfig.csv",recordPath);
  stream[1]=fopen(filename,"w");
  setbuf(stream[1],nullptr); //turn off buffering on this file
}

LogRecordGyro::~LogRecordGyro() {
  fclose(stream[0]);
  fclose(stream[1]);
}

void LogRecordGyro::startPacket(int apid) {
  current_apid=apid;
  firstField=true;
}

void LogRecordGyro::write(int8_t value) {
  fprintf(stream[current_apid],firstField?"%d":",%d",value);
  firstField=false;
}

void LogRecordGyro::write(int16_t value) {
  fprintf(stream[current_apid],firstField?"%d":",%d",value);
  firstField=false;
}

void LogRecordGyro::write(int32_t value) {
  fprintf(stream[current_apid],firstField?"%d":",%d",value);
  firstField=false;
}

void LogRecordGyro::write(uint8_t value) {
  fprintf(stream[current_apid],firstField?"%u":",%u",value);
  firstField=false;
}

void LogRecordGyro::write(uint16_t value) {
  fprintf(stream[current_apid],firstField?"%u":",%u",value);
  firstField=false;
}

void LogRecordGyro::write(uint32_t value) {
  fprintf(stream[current_apid],firstField?"%u":",%u",value);
  firstField=false;
}

void LogRecordGyro::write(float value) {
  fprintf(stream[current_apid],firstField?"%f":",%f",value);
  firstField=false;
}

void LogRecordGyro::write(char* value, int len) {
  if(!firstField) fprintf(stream[current_apid],",");
  for(int i=0;i<len;i++) fprintf(stream[current_apid],"%02x",value[i]);
  firstField=false;
}

void LogRecordGyro::write(char* value) {
  fprintf(stream[current_apid],firstField?"%s":",%s",value);
  firstField=false;
}

void LogRecordGyro::endPacket(int apid) {
  fprintf(stream[current_apid],"\n");
}

void LogRecordGyro::startDescribe(int apid) {
  current_apid=apid;
  firstField=true;
}

void LogRecordGyro::endDescribe(int apid) {
  fprintf(stream[current_apid],"\n");
}

void LogRecordGyro::describe(char* name, int type) {
  fprintf(stream[current_apid],firstField?"%s":",%s",name);
  firstField=false;
}


