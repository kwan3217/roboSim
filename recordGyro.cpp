#include "HardwarePi.h"
#include "LogCSV.h"
#include "LogRawBinary.h"
#include "LogCCSDS.h"
#include "dump.h"
#include <iostream>
#include <signal.h>

int bandwidth=3;
int samplerate=0;
int maxt=0;
int Argc;
char** Argv;

const char* csvFilenames[]={"record.csv","mpuconfig.csv"};

HardwarePiInterfaceArduino interface;
LogCSV csv(sizeof(csvFilenames)/sizeof(char*),csvFilenames);
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

  if(Argc>=2) bandwidth =atoi(Argv[1]);
  if(Argc>=3) samplerate=atoi(Argv[2]);
  if(Argc>=4) maxt      =atoi(Argv[3]);
  for(int i=0;i<Argc;i++) {
    pkt.start(APID_ARGV,"CommandLineParameters");
    pkt.write(Argv[i],"Parameter");
    pkt.end();
  }

  interface.mpu.configure(0,0,bandwidth,samplerate);
  for(int i=0;i<sizeof(buf);i++) buf[i]=0;
  interface.mpu.readConfig(buf);
  for(int i=0;i<sizeof(buf);i+=16) {
    pkt.start(APID_GYROCFG,"GyroConfig");
    pkt.write(buf+i,16,"registers");
    pkt.end();
  }

  dumpAttach(dump,APID_DUMP,64);
  signal(SIGINT, intHandler); //trap SIGINT (Ctrl-C) so that we exit instead of crashing, thus running the destructors and flusing our logs
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
  usleep(2000);
  if(maxt>0 && t>maxt) done=true;
}

int main(int argc, char** argv) {
  Argc=argc;
  Argv=argv;
  setup();
  while(!done) {
    loop();
  }
}
