#include <iostream>
#include <cmath>
#include "RoboSimHeader.h"
using namespace std;

const double PI = 2*acos(0.0);

struct waypoint
{
	double latitude;
	double longitude;
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
		}
		else if(s.read() > 0)
		{
			easting += turnRadius * cos((turnAngle + heading)*PI/180) + turnRadius * cos(heading*PI/180);
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
	
	Servo steering = Servo(-128, 127, -15, 15, 10);
	Servo throttle = Servo(-128, 127, -10, 10, 5);
	
	while(true)
	{
		const double time = .05; //Interval time; simulates amount of time between each function's call
		
		if (totaltime < 5)
		{
			throttle.write(127);
		}
		else if (totaltime >= 5 && totaltime <= 5.5)
		{
			steering.write(-128);
		}
		else if (totaltime > 5.5 && totaltime < 6.5)
		{
			steering.write(0);
		}
		else if (totaltime >= 6.5 && totaltime < 7.5)
		{
			steering.write(127);
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
		if(totaltime >= 10)
			break;
	}
	cout << "END";
}

//double guide(waypoint &wp, Robot &r)
//{
//	return (-atan((wp.longitude - r.longitude)/(wp.latitude - r.latitude)) + 90) - r.heading;
//}
/*
void control(Robot &r, double t)
{
	if (t < 5)
	{
		r.throttle.write(180);
	}
	else if (t >= 5 && t <= 5.5)
	{
		r.steering.write(80);
	}
	else
	{
		r.steering.write(90);
	}
	
}
*/
//+++++++++++++++++++++++++++Robot Class Methods
/*
Robot::Robot(double h, double lat, double lon)
{
	heading = h;
	latitude = lat;
	longitude = lon;
	throttleVal = 0;
	currentVelocity = 0;
	targetVelocity = 0;
	velocity = 0;
	wheelAngle = 0;
	wheelBase = .3;
	turnRadius = 0;
	rotationSpeed = 57.5;
	steering.Mode('s');
	throttle.Mode('t');
}

void Robot::readThrottle()	//reads changes in throttle and directs new velocity target.
{
	if(throttle.read() != throttleVal)
	{
		throttleVal = throttle.read();
		if(throttleVal < 103.5 && throttleVal > 76.5)
		{
			targetVelocity = 0;
		}
		else if (throttleVal >= 179.9)
		{
			targetVelocity = 10.0;
		}
		else if (throttleVal <= 0)
		{
			targetVelocity = -10.0;
		}
		else if (throttleVal >= 103.5)
		{
			targetVelocity = .1 + (10/76.5)*(throttleVal-103.5);
		}
		else if (throttleVal <= 76.5)
		{
			targetVelocity = -1 - (10/76.5)*(throttleVal+76.5);
		}
		currentVelocity = velocity;
	}
}
void Robot::setVelocity(double t)	//Scales velocity towards target V according to acceleration rate
{
	double velocityChange = targetVelocity - currentVelocity;
	if(velocityChange != 0.0)
		{ //invisible "1" in this equation represents acceleration rate of 10 m/s/s
			velocity += t * velocityChange;
		}
	if ((velocityChange < 0 && velocity < targetVelocity)||(velocityChange > 0 && velocity > targetVelocity))
		velocity = currentVelocity = targetVelocity;
}
void Robot::setPosition(double t)
{
	if(wheelAngle == 0) //Straight line position setting
	{
		latitude += sin(heading*PI/180)*velocity*t;
		longitude += cos(heading*PI/180)*velocity*t;
	}
	else
	{	//Time is read and placed in turnAngle to represent the angle of the turn
		//made since last position update.
		double turnAngle = t * 180 * velocity/(PI * turnRadius);
		if(wheelAngle < 0)
		{
			latitude += turnRadius * cos((turnAngle - heading)*PI/180) - turnRadius * cos(heading*PI/180);
			longitude += turnRadius * sin((turnAngle - heading)*PI/180) + turnRadius * sin(heading*PI/180);
			heading -= t*180*velocity/(PI * turnRadius);
			if (heading < 0)
				heading = 360 + heading;
		}
		else if(wheelAngle > 0)
		{
			latitude += turnRadius * cos((turnAngle + heading)*PI/180) + turnRadius * cos(heading*PI/180);
			longitude += turnRadius * sin((turnAngle + heading)*PI/180) - turnRadius * sin(heading*PI/180);
			heading += t*180*velocity/(PI * turnRadius);
			if (heading > 360)
				heading = heading - 360;
		}
	}
}
void Robot::setWheelAngle(double t)
{
	double targetAngle = steering.read()-90;
	if(targetAngle > wheelAngle)
	{
		wheelAngle += t * rotationSpeed;
		if(wheelAngle > targetAngle)
			wheelAngle = targetAngle;
	}
	else if (targetAngle < wheelAngle)
	{
		wheelAngle -= t * rotationSpeed;
		if(wheelAngle < targetAngle)
			wheelAngle = targetAngle;
	}
	if(wheelAngle < -11.5)
		wheelAngle = -11.5;
	else if(wheelAngle > 11.5)
		wheelAngle = 11.5;
}
void Robot::setTurnRadius()
{	//Pretty straight-forward, setting turn radius by wheel base.
	if(wheelAngle == 0.0)
		turnRadius = 0;
	else if (wheelAngle > 0.0)
		turnRadius = wheelBase * tan( (90 - wheelAngle) * PI / 180);
	else if (wheelAngle < 0.0)
		turnRadius = -wheelBase * tan( (90 - wheelAngle) * PI / 180);
}
void Robot::showPosition() const
{	//Made to be compatible with .csv format, very helpful for copy-pasting to a text document to be converted to csv.
	//Will need to work with fstream to take a couple steps out of the process.
	cout << latitude << ", " << longitude << ", , " << heading << ", " << velocity <<	", " << turnRadius << ", ";
}
void Robot::update(double t)
{
	readThrottle();
	setVelocity(t);
	setWheelAngle(t);
	setTurnRadius();
	setPosition(t);
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
