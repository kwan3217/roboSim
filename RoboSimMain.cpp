#include <iostream>
#include <cmath>
#include "RoboSimHeader.h"
using namespace std;

const double PI = 2*acos(0.0);
#include <ctime>

void control(Robot &r, clock_t t);

Robot robo = Robot(0,0,0);
clock_t totaltime = 0;
clock_t updatetime = 0;

clock_t timer;
int main()
{
	cout << "latitude, longitude, , heading, velocity, turnRadius, T-U, T-T\n"; //.csv headers

	
	timer = clock();//Main timer. In retrospect, while I used the time spent in the program
			//to represent how the actual robot is going to have to work with the
			//time passed since it last ran a given function, this might be a glaring
			//flaw in this code because it makes receiving output from the sim slower.
			//As it stands, it isn't fatal, so I'll worry about it when I'm working with
			//larger courses.
	
	while(true)
	{
		timer = clock() - timer;
		double time = double(timer)/CLOCKS_PER_SEC;
		updatetime += timer;
		totaltime += timer;	
		timer = clock();
		
		
		robo.update(time);
		if(double(updatetime)/CLOCKS_PER_SEC > .05)
		{
			robo.showPosition();
			cout << double(updatetime)/CLOCKS_PER_SEC << ", ";
			cout << double(clock())/CLOCKS_PER_SEC << "\n";
			updatetime = 0;
		}
		//navigate();
		//guide();
		control(robo, totaltime);
		if(totaltime >= 10000)
			break;
	}
	cout << "END";
	return 0;
}

void control(Robot &r, clock_t t)
{
	if (t < 5000)
	{
		r.throttle.write(180);
	}
	else if (t >= 5000 && t <= 5500)
	{
		r.steering.write(80);
	}
	else
	{
		r.steering.write(90);
	}
	
}

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
	{	//Giant weird equations I figured at work one day. Just changes position along a circle according to turn
		//radius, target heading, current heading, etc. As I look over it again, I don't see time used to determine
		//the new position, only the new heading, so I'll need to comb through it.
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
			if (heading > 0)
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

//Servo class functions.

Servo::Servo()
{
	setting = 90;
	mode = ' ';
}

void Servo::Mode(char m)
{	//This shouldn't be a necessary function. Will change it when
	//I don't have more important things to do; other functions
	//currently depend upon this one.
	mode = m;
}

void Servo::write(double n)
{	//Restricts based upon throttle or steering. Will probably change to 8-bit integer
	//later in development, obviously on a basis of priority.
	if (n <= 101.5 && n >= 78.5 && mode == 's')
	{
		setting = n;
	}
	else if (n <= 180 && n >= 0 && mode == 't')
	{
		setting = n;
	}
	else
	{
		cerr << "Invalid servo setting or mode.\n";
		exit(1);
	}
	
}
double Servo::read() const
{
	return setting;
}
