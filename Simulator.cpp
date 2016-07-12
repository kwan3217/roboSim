#include <iostream>
#include <cmath>
#include <stdio.h>
#include "HeaderSimRobo.h"

using namespace std;

Simulator::Simulator(double h, double e, double n)
: heading(h), easting(e), northing(n), turnRadius(0), wheelBase(.3)
{}


//Implement the robot Interface
double Simulator::checkPPS() {}
bool Simulator::checkNavChar() {}
char Simulator::readChar() {}
void Simulator::readGyro(double []) {}

void Simulator::update(const Servo & s, const Servo & t, double c)
{
	if(s.read() == 0.0)
		turnRadius = 0;
	else if (s.read() > 0.0)
		turnRadius = wheelBase * tan( (90 - s.read()) * PI / 180);
	else if (s.read() < 0.0)
		turnRadius = -wheelBase * tan( (90 - s.read()) * PI / 180);
	if(s.read() == 0) //Straight line position setting
	{
		easting += sin(heading*PI/180)*t.read()*c;
		northing += cos(heading*PI/180)*t.read()*c;
	}
	else
	{	//Time is read and placed in turnAngle to represent the angle of the turn
		//made since last position update.
		double turnAngle = c * 180 * t.read()/(PI * turnRadius);
		if(s.read() < 0)
		{
			easting += turnRadius * cos((turnAngle - heading)*PI/180) - turnRadius * cos(heading*PI/180);
			northing += turnRadius * sin((turnAngle - heading)*PI/180) + turnRadius * sin(heading*PI/180);
			heading -= c*180*t.read()/(PI * turnRadius);
			if (heading < 0)
				heading = 360 + heading;
			else if (heading > 360)
				heading -= 360;
		}
		else if(s.read() > 0)
		{
			easting += -turnRadius * cos((turnAngle + heading)*PI/180) + turnRadius * cos(heading*PI/180);
			northing += turnRadius * sin((turnAngle + heading)*PI/180) - turnRadius * sin(heading*PI/180);
			heading += c*180*t.read()/(PI * turnRadius);
			if (heading > 360)
				heading = heading - 360;
		}
	}
}
void Simulator::showVector() const
{
	printf("%10.2lf, %10.2lf", easting + 484150.0, northing + 4437810.0);
	cout << /*double (easting + 484150.0) << ", " << double (northing + 4437810.0) <<*/ ", , " << heading << ", " << turnRadius << ", ";
}

