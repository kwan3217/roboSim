#include "HardwarePi.h"
#include "LogCSV.h"
#include <signal.h>
#include <stdlib.h>

int Argc;
char** Argv;

int maxt;
HardwarePiInterfaceArduino interface;
LogCSV record("record.csv",false);

static volatile bool done=false;

void intHandler(int dummy) {
  done=true;
}

void setup() {
  if(Argc>=2) maxt      =atoi(Argv[1]); else maxt=0;
}

void loop() {
  uint32_t ts,dt;
  int32_t wc;
  fp t=interface.time();
  if(interface.readOdometer(ts,wc,dt)) {
    record.start(Log::Apids::odometer,"Odometer");
    record.write(t,"time");
    record.write(ts,"timestamp");
    record.write(wc,"wheelcount");
    record.write(dt,"deltat");
    record.end();
  }
  printf("%d,%f,%d,%d,%d,%08x,%08x\n",interface.cksumSent==interface.cksumCalc,t,ts,wc,dt,interface.cksumSent,interface.cksumCalc);
  usleep(50000);
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
