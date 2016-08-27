#include "HardwarePi.h"
#include "LogCSV.h"
#include "LogRawBinary.h"
#include "LogCCSDS.h"
#include "dump.h"
#include <iostream>
#include <signal.h>

int bandwidth,samplerate,maxt;
int Argc;
char** Argv;

HardwarePiInterfaceArduino interface;
LogCSV mpuconfigCSV("mpuconfig.csv",false);
LogCSV recordCSV("mpuconfig.csv",false);
LogRawBinary dump("attach.tbz");
LogCCSDS pkt("packets.sds");

static volatile bool done=false;

void intHandler(int dummy) {
  done=true;
}

static const int APID_DESC=0;
static const int APID_DATA=1;
static const int APID_ARGV=2;
static const int APID_DUMP=3;
static const int APID_GYROCFG=4;

void setup() {
  char buf[256];

  if(Argc>=2) bandwidth =atoi(Argv[1]); else bandwidth=3;
  if(Argc>=3) samplerate=atoi(Argv[2]); else samplerate=0;
  if(Argc>=4) maxt      =atoi(Argv[3]); else maxt=0;
  for(int i=0;i<Argc;i++) {
    pkt.start(APID_ARGV,"CommandLineParameters");
    pkt.write(Argv[i],"Parameter");
    pkt.end();
    mpuconfigCSV.start(APID_ARGV,"CommandLineParameters");
    mpuconfigCSV.write(Argv[i],"Parameter");
    mpuconfigCSV.end();
  }

  interface.mpu.configure(0,0,bandwidth,samplerate);
  for(int i=0;i<sizeof(buf);i++) buf[i]=0;
  interface.mpu.readConfig(buf);
  for(int i=0;i<sizeof(buf);i+=16) {
    pkt.start(APID_GYROCFG,"GyroConfig");
    pkt.write(buf+i,16,"registers");
    pkt.end();
    mpuconfigCSV.start(APID_GYROCFG,"GyroConfig");
    mpuconfigCSV.write(buf+i,16,"registers");
    mpuconfigCSV.end();
  }

  dumpAttach(dump,APID_DUMP,64);
  dumpAttach(pkt,APID_DUMP,64);
}

void loop() {
  int16_t gyro[3];
  int16_t T;
  float t=interface.time();
  interface.readGyro(gyro,T);
  pkt.start(APID_DATA,"GyroData");
  pkt.write(t,"time");
  pkt.write(T,"Temperature");
  pkt.write(gyro[0],"gx");
  pkt.write(gyro[1],"gy");
  pkt.write(gyro[2],"gz");
  pkt.end();
  recordCSV.start(APID_DATA,"GyroData");
  recordCSV.write(t,"time");
  recordCSV.write(T,"Temperature");
  recordCSV.write(gyro[0],"gx");
  recordCSV.write(gyro[1],"gy");
  recordCSV.write(gyro[2],"gz");
  recordCSV.end();
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
