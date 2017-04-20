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
LogCSV mpuconfigCSV("mpuconfig.csv",false);
LogCSV recordCSV("record.csv",false);
LogRawBinary dumpTBZ("attach.tbz");
LogRawBinary gps("gps.nmea");
LogCSV ppsCSV("pps.csv",false);
LogCCSDS pkt("packets.sds");
LogMulti<2> mpuconfig({&pkt,&mpuconfigCSV});
LogMulti<2> record({&pkt,&recordCSV});
LogMulti<2> dump({&pkt,&dumpTBZ});
LogMulti<2> pps({&pkt,&ppsCSV});

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
  int16_t gyro[3];
  int16_t acc[3];
  int16_t mag[3];
  bool mag_ok;
  int16_t T;
  fp t=interface.time();
  interface.readMPU(acc,gyro,T);
  mag_ok=interface.readMag(mag);
  record.start(Log::Apids::mpu,"MPUData");
  record.write(t,"time");
  record.write(T,"Temperature");
  record.write(acc[0],"ax");
  record.write(acc[1],"ay");
  record.write(acc[2],"az");
  record.write(gyro[0],"gx");
  record.write(gyro[1],"gy");
  record.write(gyro[2],"gz");
  record.write(uint8_t(mag_ok?1:0),"MagOK");
  record.write(mag[0],"bx");
  record.write(mag[1],"by");
  record.write(mag[2],"bz");
  record.end();
  while(interface.checkNavChar()){
    int8_t ch = interface.readChar();
    gps.write(ch);
  }
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
