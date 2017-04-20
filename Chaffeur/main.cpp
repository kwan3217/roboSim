#include "robot.h"
#include "LogCSV.h"
#include "LogRawBinary.h"
#include "HardwarePi.h"
#include "roboBrain.h"
#include "dump.h"
#include "wiringPi.h"

#include "OpenLoopGuidance.h"

#include <signal.h>
#include <stdlib.h>



LogCSV logC("readOut.csv");
LogRawBinary dump("attach.tbz");
LogRawBinary gps("gps.nmea");
HardwarePiInterfaceArduino hardInterface;
roboBrain passenger(309.63,0,0,hardInterface, logC, gps);
double t[]           {0,  0,  2,  5,  6,  6,  99999999};
char servoChannel[] {'T','S','T','S','S','T','T'};
int servoCommand[]  {150,110,140,150,190,150,150};
OpenLoopGuidance controller(simHard,t,servoChannel,servoCommand);

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

  passenger.navigate();
  passenger.guide();

  logC.start(0);
  logC.write(hardInterface.time(),"t");
  controller.showVector(logC);
  logC.end();
  controller.navigate();
  controller.guide();
  controller.control();
}

int main(){
	setup();
	while(!done){loop();}
}



void setup(){
}
void loop(){
}

int main(){
  setup();
  while(1) loop();
}
