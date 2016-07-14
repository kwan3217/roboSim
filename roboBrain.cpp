#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include "HeaderSimRobo.h"

using namespace std;


//+++++++++++++++++++++++++++roboBrain Class Methods

roboBrain::roboBrain(double h, double e, double n, Interface& Linterface):
heading(h), pos(e, n),  wayTarget(0),interface(Linterface),headingChange(0),desiredHeading(0)
{ }

const waypoint roboBrain::waypoints[] = {
		{   0.00,   0.00},
		{- 26.42,  21.83},
		{- 19.53,  30.55},
		{   0.29,  14.32},
		{  11.72,  28.72},
		{  23.83,  19.39},
		{   9.70,   2.77},
		{   6.24,   5.57},
		{   3.36,   2.49},
		{   6.91,-  0.11},
		{   3.93,-  3.28},
};

double roboBrain::guide() 		//-atan(waypoint.northing - northing/waypoint.easting - easting) + 90
{
	const int wpcount = sizeof(waypoints)/sizeof(waypoint);
	if((waypoints[nowpoint]- waypoints[nowpoint - 1]).dot(waypoints[nowpoint] - pos) < 0)
	{
		nowpoint += 1;
		//cout << endl << "NOWPOINT CHANGE: " << nowpoint << endl;
	}
	if(nowpoint >= wpcount)
	{
		headingChange=400;
		return headingChange;
	}
    desiredHeading = (waypoints[nowpoint]-pos).heading();

	headingChange = desiredHeading - heading;
	if(headingChange > 180)
	{
		//cout << " headingChange: " << headingChange - 360 << endl;
		headingChange -= 360;
	}
	else if (headingChange < -180)
	{
	//	cout << " headingChange: " << headingChange + 360 << endl;
		headingChange += 360;
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
	(static_cast<Simulator&>(interface)).cheatNavigate(pos.easting,pos.northing);
}

void roboBrain::showVector() const
{
	printf("%i,%06.2f,%06.2f,%06.2f, %07.2f, ",nowpoint,waypoints[nowpoint].easting, waypoints[nowpoint].northing,desiredHeading,headingChange);
}
