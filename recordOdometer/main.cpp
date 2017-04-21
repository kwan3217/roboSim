#include "HardwarePi.h"
#include "LogCSV.h"
#include "LogRawBinary.h"
#include "LogCCSDS.h"
#include "LogMulti.h"
#include "attach.h"
#include <iostream>
#include <signal.h>

int bandwidth,samplerate,maxt;
int Argc;
char** Argv;

HardwarePiInterfaceArduino interface;
LogCSV record("record.csv",false);

static volatile bool done=false;

void intHandler(int dummy) {
  done=true;
}

void setup() {
  if(Argc>=2) bandwidth =atoi(Argv[1]); else bandwidth=3;
  if(Argc>=3) samplerate=atoi(Argv[2]); else samplerate=0;
  if(Argc>=4) maxt      =atoi(Argv[3]); else maxt=0;
  for(int i=0;i<Argc;i++) {
    mpuconfig.start(Log::Apids::argv,"CommandLineParameters");
    mpuconfig.write(Argv[i],"Parameter");
    mpuconfig.end();
  }
}

void loop() {
  int16_t gyro[3];
  int16_t acc[3];
  int16_t mag[3];
  bool mag_ok;
  int16_t T;
  fp t=interface.time();
  interface.readMPU(acc,gyro,T);
  mag_ok=interface.readMag(mag);
  record.start(Log::Apids::odometer,"Odometer");
  record.write(t,"time");
  record.end();
  usleep(5000);
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
