#include <iostream>
#include <cmath>
#include "RoboSimHeader.h"			//Why do I need main in this file when I haven't previously?
using namespace std;

const double PI = 2*acos(0.0);
#include <ctime>

void control(Robot &r, clock_t t);

Robot robo = Robot(0,0,0);			//Basic setup to make everything run the way it's supposed to
clock_t totaltime = 0;
clock_t updatetime = 0;

clock_t timer;						//Main timer declaration
int main()
{
	cout << "latitude, longitude, , heading, velocity, turnRadius, T-U, T-T\n";

	
	timer = clock();				//Main timer.
	
	while(true)						//My little infini-loop to run the robot
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

void Robot::readThrottle()
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
void Robot::setVelocity(double t)								//okay, so how do I model the speed dropping to zero slowly as acceleration drops?
{
	double velocityChange = targetVelocity - currentVelocity;
	if(velocityChange != 0.0)
		{
			velocity += t * velocityChange;
		}
	if ((velocityChange < 0 && velocity < targetVelocity)||(velocityChange > 0 && velocity > targetVelocity))
		velocity = currentVelocity = targetVelocity;
}
void Robot::setPosition(double t)
{
	if(wheelAngle == 0)										//I need to use cos for latitude, sin for longitude. Make latitude like a negative x, remember that heading
	{														//starts at 12:00 with 0 degrees and goes clockwise, because Renee Descartes apparently hates mathematicians
		latitude += sin(heading*PI/180)*velocity*t;
		longitude += cos(heading*PI/180)*velocity*t;				//Convert degrees to radians --> ang*PI/180
	}
	else
	{
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
void Robot::setWheelAngle(double t)							//"t" is the time passed, so this takes the rotationSpeed and multiplies it
{															//by time to check how the wheel is doing on turning to where it's supposed to
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
{
	if(wheelAngle == 0.0)
		turnRadius = 0;
	else if (wheelAngle > 0.0)
		turnRadius = wheelBase * tan( (90 - wheelAngle) * PI / 180);
	else if (wheelAngle < 0.0)
		turnRadius = -wheelBase * tan( (90 - wheelAngle) * PI / 180);
}
void Robot::showPosition() const
{
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


Servo::Servo()
{
	setting = 90;
	mode = ' ';
}

void Servo::Mode(char m)
{
	mode = m;
}

void Servo::write(double n)
{
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
