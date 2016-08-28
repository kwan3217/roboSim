#include "HardwarePi.h"
#include "LogCSV.h"
#include "LogRawBinary.h"
#include "LogCCSDS.h"
#include "LogMulti.h"
#include "dump.h"
#include <iostream>
#include <signal.h>

int bandwidth,samplerate,maxt;
int Argc;
char** Argv;

static const int APID_DESC=0;
static const int APID_DATA=1;
static const int APID_ARGV=2;
static const int APID_DUMP=3;
static const int APID_GYROCFG=4;
static const int APID_CCSDS_ID=5;

HardwarePiInterfaceArduino interface;
LogCSV mpuconfigCSV("mpuconfig.csv",false);
LogCSV recordCSV("record.csv",false);
LogRawBinary dumpTBZ("attach.tbz");
LogCCSDS pkt("packets.sds",APID_DESC,APID_CCSDS_ID);
LogMulti<2> mpuconfig({&pkt,&mpuconfigCSV});
LogMulti<2> record({&pkt,&recordCSV});
LogMulti<2> dump({&pkt,&dumpTBZ});

static volatile bool done=false;

void intHandler(int dummy) {
  done=true;
}

void setup() {
  if(Argc>=2) bandwidth =atoi(Argv[1]); else bandwidth=3;
  if(Argc>=3) samplerate=atoi(Argv[2]); else samplerate=0;
  if(Argc>=4) maxt      =atoi(Argv[3]); else maxt=0;
  for(int i=0;i<Argc;i++) {
    mpuconfig.start(APID_ARGV,"CommandLineParameters");
    mpuconfig.write(Argv[i],"Parameter");
    mpuconfig.end();
  }

  interface.mpu.configure(0,0,bandwidth,samplerate);
  char reg[128];
  for(int i=0;i<sizeof(reg);i++) reg[i]=0;
  interface.mpu.readConfig(reg);
  for(int i=0;i<sizeof(reg);i+=16) {
    mpuconfig.start(APID_GYROCFG,"GyroConfig");
    mpuconfig.write(reg+i,16,"registers");
    mpuconfig.end();
  }

  dumpAttach(dump,APID_DUMP,64);
}

void loop() {
  int16_t gyro[3];
  int16_t T;
  float t=interface.time();
  interface.readGyro(gyro,T);
  record.start(APID_DATA,"GyroData");
  record.write(t,"time");
  record.write(T,"Temperature");
  record.write(gyro[0],"gx");
  record.write(gyro[1],"gy");
  record.write(gyro[2],"gz");
  record.end();
  usleep(2000);
  if(maxt>0 && t>maxt) done=true;
}

int main(int argc, char** argv) {
  signal(SIGINT, intHandler); //trap SIGINT (Ctrl-C) so that we exit instead of crashing, thus running the destructors and flushing our logs
  Argc=argc;
  Argv=argv;
  setup();
  while(!done) {
    loop();
  }
}
