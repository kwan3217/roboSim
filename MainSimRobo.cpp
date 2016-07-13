#include <iostream>
#include <cmath>
#include "HeaderSimRobo.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;


int main()
{
//	Simulator::testNMEA();
	cout << "easting, northing, , heading, turnRadius, T-U, T-T\n"; //.csv headers
	
	double totaltime = 0; //Epoch time; takes amount of time since program began
	Simulator roboSim = Simulator(309.63, 40.090586, -105.185485);
	
	roboBrain robo = roboBrain(309.63,0,0,roboSim);
	
	
	while(true)
	{
		double time = .05; //Interval time; simulates amount of time between each function's call
		
		robo.update(time); //contains simulation adjustment and timesteps the servos
		
		roboSim.showVector();
		cout << time << ", ";
		cout << totaltime << "\n";

		//navigate();
		//double headingChange = guide(goal, robo);
//		robo.navigateCompass();
		robo.navigateGPS();
		robo.control(robo.guide());
		
		totaltime += time;
		if(totaltime >= 60)
			break;
	}
	cout << "END";
	return 0;
}

