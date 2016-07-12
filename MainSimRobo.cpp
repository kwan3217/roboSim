#include <iostream>
#include <cmath>
#include "HeaderSimRobo.h"
#include <stdio.h>
#include <stdlib.h>
using namespace std;

//Create the Simulator instance for the other classes
Simulator roboSim = Simulator(343.4, 40.090586, -105.185485);

int main()
{
	cout << "easting, northing, , heading, turnRadius, T-U, T-T\n"; //.csv headers
	
	double totaltime = 0; //Epoch time; takes amount of time since program began
	
	roboBrain robo = roboBrain(343.4, 22.48, 17.64);
	
	
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

