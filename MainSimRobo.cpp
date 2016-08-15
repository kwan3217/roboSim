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
	cout << "time,pos.easting, pos.northing, simThrottle.read(), heading,  turnRadius ,,nav.easting, nav.northing,, nowpoint,waypoints[nowpoint].easting, waypoints[nowpoint].northing,desiredHeading,headingChange" << endl; //.csv headers
	
	Simulator roboSim = Simulator(309.63, 40.090586, -105.185485);
	
	roboBrain robo = roboBrain(309.63,0,0,roboSim);
	
	while(true) {
		double dt = .05; //Interval time; simulates amount of time between each function's call
		roboSim.update(dt);
		robo.navigateCompass();
		robo.navigateOdometer();
		robo.navigateGPS();
                robo.guide();
		robo.control();
		if(roboSim.time() > 90) break;
                cout << roboSim.time() << ",";
		roboSim.showVector();
		robo.showVector();
	}
	cout << "\nEND";
	return 0;
}

