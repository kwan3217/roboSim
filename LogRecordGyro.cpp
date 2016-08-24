#include "Log.h"
#include <ctime> //for struct tm
#include <unistd.h> //for readlink
#include <cstring> //for strlen
#include <sys/stat.h> //for mkdir

LogRecordGyro::LogRecordGyro() {
  time_t rawtime;
  struct tm* ptm;
  time(&rawtime);
  ptm=gmtime(&rawtime);
  char buf1[256],buf2[256];
  readlink("/proc/self/exe",buf1,sizeof(buf1)-1);
  for(int i=strlen(buf1)-1;i>0 && buf1[i]!='/';i--) buf1[i]=0;
  snprintf(buf2,sizeof(buf2)-1,"%s/testOutputData/%04d%02d%02dT%02d%02d%02d",buf1,ptm->tm_year+1900,ptm->tm_mon+1,ptm->tm_mday,ptm->tm_hour,ptm->tm_min,ptm->tm_sec);
  mkdir(buf2,0755);
  snprintf(buf1,sizeof(buf1)-1,"%s/record.csv",buf2);
  stream[0]=fopen(buf1,"w");
  snprintf(buf1,sizeof(buf1)-1,"%s/mpuconfig.csv",buf2);
  stream[1]=fopen(buf1,"w");
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
