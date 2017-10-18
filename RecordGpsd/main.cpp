#include "HardwarePi.h"
#include "LogCSV.h"
#include "LogRawBinary.h"
#include "LogCCSDS.h"
#include "LogMulti.h"
#include "attach.h"
#include "roboBrain.h"
#include <iostream>
#include <signal.h>
#include <gps.h>
#include <stdlib.h>
#include <errno.h>

int bandwidth,samplerate,maxt;
int Argc;
char** Argv;

HardwarePiInterfaceArduino interface;
LogCCSDS pkt("packets.sds");
roboBrain brain(interface,pkt);

static volatile bool done=false;

void intHandler(int dummy) {
  done=true;
}

void setup() {
  if(Argc>=2) bandwidth =atoi(Argv[1]); else bandwidth=3;
  if(Argc>=3) samplerate=atoi(Argv[2]); else samplerate=0;
  if(Argc>=4) maxt      =atoi(Argv[3]); else maxt=0;
  for(int i=0;i<Argc;i++) {
    pkt.start(Log::Apids::argv,"CommandLineParameters");
    pkt.write(Argv[i],"Parameter");
    pkt.end();
  }
  pkt.start(Log::Apids::epoch,"Time Epoch");
  pkt.write((uint32_t)(interface.epoch().tv_sec),"Epoch Seconds");
  pkt.write((uint32_t)(interface.epoch().tv_nsec),"Epoch Nanoseconds");
  pkt.end();
  interface.mpu.configure(0,0,bandwidth,samplerate);
  char reg[128];
  for(size_t i=0;i<sizeof(reg);i++) reg[i]=0;
  interface.mpu.readConfig(reg);
  for(size_t i=0;i<sizeof(reg);i+=16) {
    pkt.start(Log::Apids::gyroCfg,"GyroConfig");
    pkt.write(reg+i,16,"registers");
    pkt.end();
  }

  for(size_t i=0;i<sizeof(reg);i++) reg[i]=0;
  interface.mpu.ak.readConfig(reg);
  for(size_t i=0;i<sizeof(reg);i+=16) {
    pkt.start(Log::Apids::akCfg,"MagConfig");
    pkt.write(reg+i,16,"registers");
    pkt.end();
  }

  dumpAttach(pkt,64);
}

void loop() {
  static int count=0;
  brain.loop();
  if(count==100) {
    printf("%d,%f,%f,%d,%d\n",interface.button(),interface.time(),brain.getHeading(),brain.getSteeringCmd(),brain.getThrottleCmd());
    count=0;
  }
  count++;
  if(maxt>0 && interface.time()>maxt) done=true;
  usleep(6000);
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
