#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include "HeaderSimRobo.h"

using namespace std;


//+++++++++++++++++++++++++++roboBrain Class Methods

roboBrain::roboBrain(double h, double e, double n, Interface& Linterface):
heading(h), easting(e), northing(n), turnRadius(0),
wheelBase(.3), wayTarget(0),interface(Linterface)
{ }

double roboBrain::guide() const		//-atan(waypoint.northing - northing/waypoint.easting - easting) + 90
{
	const int wpcount = 6;
	static waypoint waypoints[wpcount] = {{12.48, 17.64},{9.03, 29.21}, {63.81, 95.25}, {91.81, 71.44}, {37.80, 6.18}, {15.84, 21.98}};
	static int nowpoint = 1;
	double headingChange;
	if(((waypoints[nowpoint].northing - waypoints[nowpoint - 1].northing)*(waypoints[nowpoint].northing - northing) +
	(waypoints[nowpoint].easting - waypoints[nowpoint - 1].easting)*(waypoints[nowpoint].easting - easting)) < 0)	//dot product -> vector1.northing*vector2.northing + vector1.easting*vector2.easting
	{
		nowpoint += 1;
		cout << endl << "NOWPOINT CHANGE: " << nowpoint << endl;
	}
	if(nowpoint >= wpcount)
	{
		return 400;
	}
	double desiredHeading = -(atan((waypoints[nowpoint].northing - northing)/(waypoints[nowpoint].easting - easting))*180/PI) + 90;

	if(waypoints[nowpoint].easting < easting)
		desiredHeading += 180;
	headingChange = desiredHeading - heading;
	if(headingChange > 180)
	{
		//cout << " headingChange: " << headingChange - 360 << endl;
		return headingChange - 360;
	}
	else if (headingChange < -180)
	{
	//	cout << " headingChange: " << headingChange + 360 << endl;
		return headingChange + 360;
	}
	//cout << " headingChange: " << headingChange << endl;
	return headingChange;
}

void roboBrain::control(double headingChange)
{

	if(headingChange >= 300)
	{
		interface.throttle.write(0);
		interface.steering.write(0);
		return;
	}
	interface.throttle.write(64);
//	if(headingChange > 0)
//	{
	//	if(headingChange > 15)
	//		steering.write(127);
	//	else
	interface.steering.write(headingChange * double (127)/180);
//	}
//	else if(headingChange < 0)
//	{
	//	if(headingChange < -15)
	//		steering.write(-127);
	//	else
//			steering.write(headingChange * double (127)/180);
//	}
//	else
//	{
//		steering.write(0);
//	}
}

void roboBrain::update(double t)
{

	/*
	if(steering.read() == 0.0)
		turnRadius = 0;
	else if (steering.read() > 0.0)
		turnRadius = wheelBase * tan( (90 - steering.read()) * PI / 180);
	else if (steering.read() < 0.0)
		turnRadius = -wheelBase * tan( (90 - steering.read()) * PI / 180);
	if(steering.read() == 0) //Straight line position setting
	{
		easting += sin(heading*PI/180)*throttle.read()*c;
		northing += cos(heading*PI/180)*throttle.read()*c;
	}
	else
	{	//Time is read and placed in turnAngle to represent the angle of the turn
		//made since last position update.
		double turnAngle = c * 180 * throttle.read()/(PI * turnRadius);
		if(steering.read() < 0)
		{
			easting += turnRadius * cos((turnAngle - heading)*PI/180) - turnRadius * cos(heading*PI/180);
			northing += turnRadius * sin((turnAngle - heading)*PI/180) + turnRadius * sin(heading*PI/180);
			heading -= c*180*throttle.read()/(PI * turnRadius);
			if (heading < 0)
				heading = 360 + heading;
			else if (heading > 350)
				heading -= 360;
		}
		else if(steering.read() > 0)
		{
			easting += -turnRadius * cos((turnAngle + heading)*PI/180) + turnRadius * cos(heading*PI/180);
			northing += turnRadius * sin((turnAngle + heading)*PI/180) - turnRadius * sin(heading*PI/180);
			heading += c*180*throttle.read()/(PI * turnRadius);
			if (heading > 360)
				heading = heading - 360;
		}
	}
	*/
}

void roboBrain::navigateCompass()
{
	//Intentionally ugly -- this won't work in general when the interface isn't a Simulator
	heading = (static_cast<Simulator&>(interface)).heading;
}

void roboBrain::navigateGPS()
{
	//Intentionally ugly -- this won't work in general when the interface isn't a Simulator
	northing = (static_cast<Simulator&>(interface)).northing;
	easting = (static_cast<Simulator&>(interface)).easting;
}

void roboBrain::showVector() const
{
	cout << easting << ", " << northing << ", , " << heading << ", " << turnRadius << ", ";
}
