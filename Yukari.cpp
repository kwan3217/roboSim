#include "robot.h"
#include "LogCSV.h"
#include "LogRawBinary.h"
#include "HardwarePi.h"
#include "roboBrain.h"
#include "dump.h"
#include "wiringPi.h"
#include <signal.h>
#include <stdlib.h>

LogCSV logC("readOut.csv");
LogRawBinary dump("attach.tbz");
LogRawBinary gps("gps.nmea");
HardwarePiInterfaceArduino hardInterface;
roboBrain controller(309.63,0,0,hardInterface, logC, gps);

volatile bool done=false;

void intHandler(int dummy) {
  hardInterface.throttle.write(150);
  hardInterface.steering.write(150);
  done=true;
}

void setup(){
  signal(SIGINT, intHandler); //trap SIGINT (Ctrl-C) so that we exit instead of crashing, thus running the destructors and flushing our logs
  dumpAttach(dump,0);
  pinMode(18, INPUT);
  pinMode(22, OUTPUT);
}

void loop(){
  int pin18 = digitalRead(18);
  digitalWrite(22, pin18);

  controller.navigate();
  controller.guide();
  controller.control();

  logC.start(0);
  logC.write(hardInterface.time(),"t");
  controller.showVector(logC);
  logC.end();
}

int main(){
	setup();
	while(!done){loop();}
}
