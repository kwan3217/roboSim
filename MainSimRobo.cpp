#include "HeaderSimRobo.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;


int main()
{
//	Simulator::testNMEA();
	//cout << "t,easting, northing, spped, heading, turnRadius, desiredHeading,headingChange\n"; //.csv headers
	
	Simulator roboSim = Simulator(309.63, 40.090586, -105.185485);
	
	roboBrain robo = roboBrain(309.63,0,0,roboSim);
	
	while(true)
	{
		double dt = .05; //Interval time; simulates amount of time between each function's call
		roboSim.update(dt);
		robo.navigateCompass();
		robo.navigateOdometer();
		robo.control(robo.guide());
//		robo.navigateCompass();
//		robo.navigateGPS();
		if(roboSim.time() > 30) break;

		roboSim.showVector();
		robo.showVector();
	}
	cout << "\nEND";
	return 0;
}

