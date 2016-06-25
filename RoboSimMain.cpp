#include <iostream>
#include <cmath>
#include "RoboSimHeader.h"
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

int main()
{
/*	cout << "latitude, longitude, , heading, velocity, turnRadius, T-U, T-T\n"; //.csv headers

	waypoint goal = {30, 80};
	
	double totaltime = 0; //Epoch time; takes amount of time since program began
	
	while(true)
	{
		double time = .05; //Interval time; simulates amount of time between each function's call
		
		robo.update(time);

		robo.showPosition();
		cout << time << ", ";
		cout << totaltime << "\n";
			
		//navigate();
		//double headingChange = guide(goal, robo);
		control(robo,  totaltime);
		
		totaltime += time;
		if(totaltime >= 10)
			break;
	}
	cout << "END";*/
	
	Simulator simtest = Simulator();
	simtest.test();
	
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
			else if (heading > 350)
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
	cout << easting << ", " << northing << ", , " << heading << ", " << turnRadius << ", ";
}
void Simulator::test()
{
	cout << "easting, northing, , heading, turnRadius, velocity, T-U, T-T\n"; //.csv headers

	//waypoint goal = {30, 80};
	
	double totaltime = 0; //Epoch time; takes amount of time since program began
	
	Servo steering = Servo(-128, 128, -15, 15, 10);
	Servo throttle = Servo(-128, 128, -10, 10, 5);
	
	while(true)
	{
		const double time = .05; //Interval time; simulates amount of time between each function's call
		
		if (totaltime < 5)
		{
			throttle.write(128);
		}
		else if (totaltime >= 5 && totaltime <= 5.5)
		{
			steering.write(-128);
		}
		else if (totaltime > 5.5 && totaltime < 6.5)
		{
			steering.write(0);
		}
		else if (totaltime >= 8.5 && totaltime <= 9.0)
		{
			steering.write(128);
		}
		else
		{
			steering.write(0);
		}
		
		update(steering, throttle, time);

		showVector();
		cout << throttle.read() << ", " << time << ", ";
		cout << totaltime << "\n";
			
		//navigate();
		//double headingChange = guide(goal, robo);
		steering.timeStep(time);
		throttle.timeStep(time);
		totaltime += time;
		if(totaltime >= 14)
			break;
	}
	cout << "END";
}

//+++++++++++++++++++++++++++roboBrain Class Methods

double roboBrain::guide(waypoint &wp, Robot &r)
{
	const int wpcount = 2;
	static waypoint waypoints[wpcount] = {{50, 50},{-50, -50}};
	return (-atan((wp.longitude - r.longitude)/(wp.latitude - easting)) + 90) - r.heading;
}

void roboBrain::control(double ac)
{
	static turnTime = 0;
	if (turnTime = 0)
	{
		
	}
}

void roboBrain::update(double t)
{

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

}

void roboBrain::showVector() const
{
	cout << easting << ", " << northing << ", , " << heading << ", " << throttle.read() <<	", " << turnRadius << ", ";
}
*/
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
