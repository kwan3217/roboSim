#include "testbrain.h"
#include "robot.h"
#include "LogCSV.h"

LogCSV logC("readOut.csv")
HardwarePiInterfaceArduino hardInterface;
Simulator roboSim(309.63,0,0);
testBrain controller(309.63,0,0,roboSim, hardInterface, logC);

void setup(){ }

void loop(){
	logC.start(0);
	controller.showVector();
	logC.end();
	controller.navigate();
	controller.guide();
	controller.control();
}

int main(){
	setup();
	while(1){loop();}
}