#include <iostream>
#include <cmath>
#include "HeaderSimRobo.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;


int main()
{
//	Simulator::testNMEA();
	//cout << "t,easting, northing, spped, heading, turnRadius, desiredHeading,headingChange\n"; //.csv headers
	
	//Simulator roboSim = Simulator(309.63, 40.090586, -105.185485);
	NMEAPlayback roboSim=NMEAPlayback("testInputData/PhoneSittingStill.nmea");
	
	roboBrain robo = roboBrain(309.63,0,0,roboSim);
	
	printf(", easting, northing\n");
	while(true)
	{
		double dt = .05; //Interval time; simulates amount of time between each function's call
//		roboSim.update(dt);
//		robo.update(dt); //contains simulation adjustment and timesteps the servos
		

		//navigate();
		//double headingChange = guide(goal, robo);
//		while(roboSim.checkNavChar()) printf("%c",roboSim.readChar());
//		robo.navigateCompass();
		robo.navigateGPS();
//		robo.control(robo.guide());
		
		//printf("%05.2f, ",roboSim.time());
		//roboSim.showVector();
		robo.showVector();
		//cout << endl;

		if(!roboSim.checkNavChar()) break;
	}
	cout << "END";
	return 0;
}

