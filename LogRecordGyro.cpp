#include "Log.h"
#include <ctime> //for struct tm
#include <unistd.h> //for readlink
#include <cstring> //for strlen
#include <sys/stat.h> //for mkdir

static char recordPath[256]={0};
static void getRecordPath() {
  if(recordPath[0]==0) {
    time_t rawtime;
    struct tm* ptm;
    time(&rawtime);
    ptm=gmtime(&rawtime);
    static char exePath[256];
    readlink("/proc/self/exe",exePath,sizeof(exePath)-1);
    int n=strlen(exePath);
    for(int i=n-1;i>0 && exePath[i]!='/';i--) exePath[i]=0;
    snprintf(recordPath,sizeof(recordPath)-1,"%s/testOutputData/%04d%02d%02dT%02d%02d%02d",exePath,ptm->tm_year+1900,ptm->tm_mon+1,ptm->tm_mday,ptm->tm_hour,ptm->tm_min,ptm->tm_sec);
    mkdir(recordPath,0755);
  }
}

LogRecordGyro::LogRecordGyro() {
  char filename[256];
  getRecordPath();
  snprintf(filename,sizeof(filename)-1,"%s/record.csv",recordPath);
  stream[0]=fopen(filename,"w");
  snprintf(filename,sizeof(filename)-1,"%s/mpuconfig.csv",recordPath);
  stream[1]=fopen(filename,"w");
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

LogRawBinary::LogRawBinary(char* basename) {
  char filename[256];
  getRecordPath();
  snprintf(filename,sizeof(filename)-1,"%s/%s",recordPath,basename);
  stream=fopen(filename,"w");
}

LogRawBinary::~LogRawBinary() {
  fclose(stream);
}

