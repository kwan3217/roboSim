#include <iostream>
#include <cmath>
#include "robot.h"
#include "Simulator.h"
#include "roboBrain.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;


int main()
{
//	Simulator::testNMEA();
	cout << "pos.easting, pos.northing, simThrottle.read(), heading,  turnRadius ,,nav.easting, nav.northing,, nowpoint,waypoints[nowpoint].easting, waypoints[nowpoint].northing,desiredHeading,headingChange" << endl; //.csv headers
	
	HardwarePiInterfaceArduino roboInterface;
	
	roboBrain robo = roboBrain(309.63,0,0,roboInterface);
	
	while(true)
	{
		roboInterface.throttle.write(1750);
		if(roboInterface.time() > 2) roboInterface.steering.write(2000);
		else if(roboInterface.time() > 2.5) roboInterface.steering.write(1500);
		else if(roboInterface.time() > 5){
			roboInterface.throttle.write(1500);
			break;
		}
	}
	cout << "\nEND";
	return 0;
}

