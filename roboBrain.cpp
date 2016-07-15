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

}

void roboBrain::navigateCompass()
{
	//Intentionally ugly -- this won't work in general when the interface isn't a Simulator
	(static_cast<Simulator&>(interface)).cheatHeading(heading);
}

void roboBrain::navigateGPS()
{
	//Intentionally ugly -- this won't work in general when the interface isn't a Simulator
	(static_cast<Simulator&>(interface)).cheatNavigate(easting,northing);
}

void roboBrain::showVector() const
{
	cout << easting << ", " << northing << ", , " << heading << ", " << turnRadius << ", ";
}
