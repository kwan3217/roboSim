#include <iostream>
#include <cmath>
#include "HeaderSimRobo.h"
using namespace std;

const double PI = 2*acos(0.0);

struct waypoint
{
	double easting;
	double northing;
};

//double guide(waypoint &wp, Robot &r);
//void control(Robot &r, double t);

//Robot robo = Robot(0,0,0);

Simulator roboSim = Simulator(343.4, 22.48, 17.64);

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
		robo.navigateCompass();
		robo.navigateGPS();
		robo.control(robo.guide());
		
		totaltime += time;
		if(totaltime >= 60)
			break;
	}
	cout << "END";
	
	return 0;
}

Simulator::Simulator(double h, double e, double n)
: heading(h), easting(e), northing(n), turnRadius(0), wheelBase(.3)
{}
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

//+++++++++++++++++++++++++++roboBrain Class Methods

roboBrain::roboBrain(double h, double e, double n)
: throttle(-127, 127, -10, 10, 5), steering(-127, 127, -15, 15, 75),
heading(h), easting(e), northing(n), turnRadius(0),
wheelBase(.3), wayTarget(0) 
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
		throttle.write(0);
		steering.write(0);
		return;
	}
	throttle.write(64);
//	if(headingChange > 0)
//	{
	//	if(headingChange > 15)
	//		steering.write(127);
	//	else
			steering.write(headingChange * double (127)/180);
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
	
	steering.timeStep(t);
	throttle.timeStep(t);
	roboSim.update(steering, throttle, t);
	
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
	heading = roboSim.heading;
}

void roboBrain::navigateGPS()
{
	northing = roboSim.northing;
	easting = roboSim.easting;
}

void roboBrain::showVector() const
{
	cout << easting << ", " << northing << ", , " << heading << ", " << throttle.read() <<	", " << turnRadius << ", ";
}
//++++++++++++++++++++++++++++++++++++++++++++++++++Servo methods.

Servo::Servo(int cmdmin, int cmdmax, double physmin, double physmax, double slewrate) : 
cmdmin(cmdmin), cmdmax(cmdmax), physmin(physmin), physmax(physmax), slewrate(slewrate), commanded(0), physical(0)
{
	if(physmax < physmin || cmdmax < cmdmin)
	{
		cerr << "bad servo max/min -- set max greater than or equal to min";
		exit(1);
	}
}
void Servo::write(int n)
{
	if(n > cmdmax)
	{
		n = cmdmax;
	}
	else if (n < cmdmin)
	{
		n = cmdmin;
	}
	else
	{
		commanded = n;
	}
}
void Servo::timeStep(double t)
{
	double commandPhysical = (double(commanded - cmdmin)/(cmdmax - cmdmin)) * (physmax - physmin) + physmin;
	if (physical < commandPhysical)				//physical vs command * command should be 8-bit int, make physical into a proportional double from -15 to 15 degrees
	{
		physical += t * slewrate;
		if (physical > commandPhysical)
			physical = commandPhysical;
	}
	else if (physical > commandPhysical)
	{
		physical -= t * slewrate;
		if (physical < commandPhysical)
			physical = commandPhysical;
	}
}
void Servo::test()
{
	Servo Steering = Servo(1000, 2000, -15, 15, 5);
	double time = 0;
	while(time < 20)
	{
		cout << time << ", " << Steering.read() << endl;
		if(time < 4)
			Steering.write(2000);
		else if(time < 14)
			Steering.write(1000);
		else if(time >= 14)
		{
			Steering.write(1500);
		}
		Steering.timeStep(.05);
		time += .05;
	}
}
