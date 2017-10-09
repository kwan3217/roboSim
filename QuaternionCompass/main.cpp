#include "HardwarePi.h"
#include "LogCSV.h"
#include "LogRawBinary.h"
#include "LogCCSDS.h"
#include "LogMulti.h"
#include "attach.h"
#include "roboBrain.h"
#include <iostream>
#include <signal.h>

int bandwidth,samplerate,maxt;
int Argc;
char** Argv;

HardwarePiInterfaceArduino interface;
//LogCSV mpuconfigCSV("mpuconfig.csv",false);
//LogCSV recordCSV("record.csv",false);
//LogRawBinary dumpTBZ("attach.tbz");
//LogRawBinary gps("gps.nmea");
//LogCSV ppsCSV("pps.csv",false);
LogCCSDS pkt("packets.sds");
//LogCSV quaternionCSV("quaternion.csv",false,Log::Apids::quaternion);
//LogCSV compassCSV("compass.csv",false,Log::Apids::compass);
//LogCSV averagegCSV("averageg.csv",false,Log::Apids::averageG);
//LogCSV calcOffsetCSV("calcoffset.csv",false,Log::Apids::calcOffset);
//LogCSV setOffsetCSV("setoffset.csv",false,Log::Apids::setOffSet);
//LogMulti<2> mpuconfig({&pkt,&mpuconfigCSV});
LogCCSDS& mpuconfig=pkt;
//LogMulti<2> record({&pkt,&recordCSV});
LogCCSDS& record=pkt;
//LogMulti<2> dump({&pkt,&dumpTBZ});
LogCCSDS& dump=pkt;
//LogMulti<2> pps({&pkt,&ppsCSV});
LogCCSDS& pps=pkt;
//LogMulti<6> logall({&pkt,&quaternionCSV,&averagegCSV,&calcOffsetCSV,&setOffsetCSV,&compassCSV});
LogCCSDS& logall=pkt;
roboBrain brain(0,0,0,interface,pkt);

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
  mpuconfig.start(Log::Apids::epoch,"Time Epoch");
  mpuconfig.write((uint32_t)(interface.epoch().tv_sec),"Epoch Seconds");
  mpuconfig.write((uint32_t)(interface.epoch().tv_nsec),"Epoch Nanoseconds");
  mpuconfig.end();
  interface.mpu.configure(0,0,bandwidth,samplerate);
  char reg[128];
  for(size_t i=0;i<sizeof(reg);i++) reg[i]=0;
  interface.mpu.readConfig(reg);
  for(size_t i=0;i<sizeof(reg);i+=16) {
    mpuconfig.start(Log::Apids::gyroCfg,"GyroConfig");
    mpuconfig.write(reg+i,16,"registers");
    mpuconfig.end();
  }

  for(size_t i=0;i<sizeof(reg);i++) reg[i]=0;
  interface.mpu.ak.readConfig(reg);
  for(size_t i=0;i<sizeof(reg);i+=16) {
    mpuconfig.start(Log::Apids::akCfg,"MagConfig");
    mpuconfig.write(reg+i,16,"registers");
    mpuconfig.end();
  }

  dumpAttach(dump,64);
}

void loop() {
  brain.navigate();
  if(maxt>0 && interface.time()>maxt) done=true;
  fp t_pps;
  if(interface.checkPPS(t_pps)) {
    struct timespec raw_pps=interface.get_raw_pps();
    struct timespec t0=interface.get_raw_t0();
    pps.start(Log::Apids::pps,"PPS");
    pps.write(int32_t(t0.tv_sec),"t0_sec");
    pps.write(int32_t(t0.tv_nsec),"t0_nsec");
    pps.write(int32_t(raw_pps.tv_sec),"pps_sec");
    pps.write(int32_t(raw_pps.tv_nsec),"pps_nsec");
    pps.write(t_pps,"t_pps");
    pps.end();
  }
  usleep(5000);
}

int main(int argc, char** argv) {
  signal(SIGINT, intHandler); //trap SIGINT (Ctrl-C) so that we exit instead of crashing, thus running the destructors and flushing our logs
  Argc=argc;
  Argv=argv;
  //Stone bridge equation test - i^2==j^2==k^2==ijk==-1
  Quaternion i(1,0,0,0);
  printf("i: x=%f,y=%f,z=%f,w=%f\n",i.x(),i.y(),i.z(),i.w());
  Quaternion j(0,1,0,0);
  printf("j: x=%f,y=%f,z=%f,w=%f\n",j.x(),j.y(),j.z(),j.w());
  Quaternion k(0,0,1,0);
  printf("k: x=%f,y=%f,z=%f,w=%f\n",k.x(),k.y(),k.z(),k.w());
  Quaternion i2=i*i;
  printf("i^2: x=%f,y=%f,z=%f,w=%f\n",i2.x(),i2.y(),i2.z(),i2.w());
  Quaternion j2=j*j;
  printf("j^2: x=%f,y=%f,z=%f,w=%f\n",j2.x(),j2.y(),j2.z(),j2.w());
  Quaternion k2=k*k;
  printf("k^2: x=%f,y=%f,z=%f,w=%f\n",k2.x(),k2.y(),k2.z(),k2.w());
  Quaternion ijk=i*j*k;
  printf("ijk: x=%f,y=%f,z=%f,w=%f\n",ijk.x(),ijk.y(),ijk.z(),ijk.w());
  setup();
  while(!done) {
    loop();
  }
}
