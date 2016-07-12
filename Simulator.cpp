#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "HeaderSimRobo.h"

using namespace std;

Simulator::Simulator(double h, double Llat0, double Llon0)
: heading(h), lat0(Llat0), lon0(Llon0), turnRadius(0)
{
    generateNewFix();
}


//Implement the robot Interface
double Simulator::checkPPS() {
  return pps;
}
bool Simulator::checkNavChar() {
  int nCharsShouldTransmit=((time-pps)-nmeaDelay)/charTime;
  if(nCharsShouldTransmit<0) nCharsShouldTransmit=0;
  if(nCharsShouldTransmit>strlen(nmea)) nCharsShouldTransmit=strlen(nmea);
  return nCharsShouldTransmit<charsSent;
}
char Simulator::readChar() {
  if(!checkNavChar()) {
	cout << "Not allowed to read when nothing available" << endl;
	exit(2);
  }
  char result=nmea[charsSent];
  charsSent++;
  return result;
}
void Simulator::readGyro(double []) {}

/** Generate a new GPS fix. Update the PPS value, create the RMC sentence, set the pointers for spooling out the sentence */
void Simulator::generateNewFix() {
  pps=floor(time/dpps+0.5);
  int s=pps;
  int m=s/60;
  s=s%60;
  int h=m/60;
  m=m%60;
  double lat=lat0+(northing/re)*180.0/PI;
  char ns=lat>0?'N':'S';
  lat=fabs(lat);
  lat=floor(lat)*100+(lat-floor(lat))*60; //convert to NMEA DDMM.MMMM format
  double lon=lon0+(easting/cos(lat*PI/180.0)/re)*180.0/PI;
  char ew=lon>0?'E':'W';
  lon=fabs(lon);
  lon=floor(lon)*100+(lon-floor(lon))*60; //convert to NMEA DDMM.MMMM format
  double speed=0.0*1.94384449; // Eventually we will read this from the throttle servo, whose physical value is m/s

  sprintf(nmea,"$GPRMC,%02d%02d%02d,A,%010.5f,%c,%011.5f,%c,%05.1f,%05.1f,170916,000.0,W*",h,m,s,lat,ns,lon,ew,speed,heading);
//  cout << nmea << endl;
}

/** Update the actual position and heading of the robot
 * @param s steering servo
 * @param t throttle servo
 * @param dt update time interval in seconds
 */
void Simulator::update(const Servo & s, const Servo & t, double dt) {
	time+=dt; //Update current time
	if((time-pps)>dpps) {
		generateNewFix();
	}
	if(s.read() == 0.0)
		turnRadius = 0;
	else if (s.read() > 0.0)
		turnRadius = wheelBase * tan( (90 - s.read()) * PI / 180);
	else if (s.read() < 0.0)
		turnRadius = -wheelBase * tan( (90 - s.read()) * PI / 180);
	if(s.read() == 0) //Straight line position setting
	{
		easting += sin(heading*PI/180)*t.read()*dt;
		northing += cos(heading*PI/180)*t.read()*dt;
	}
	else
	{	//Time is read and placed in turnAngle to represent the angle of the turn
		//made since last position update.
		double turnAngle = dt * 180 * t.read()/(PI * turnRadius);
		if(s.read() < 0)
		{
			easting += turnRadius * cos((turnAngle - heading)*PI/180) - turnRadius * cos(heading*PI/180);
			northing += turnRadius * sin((turnAngle - heading)*PI/180) + turnRadius * sin(heading*PI/180);
			heading -= dt*180*t.read()/(PI * turnRadius);
			if (heading < 0)
				heading = 360 + heading;
			else if (heading > 360)
				heading -= 360;
		}
		else if(s.read() > 0)
		{
			easting += -turnRadius * cos((turnAngle + heading)*PI/180) + turnRadius * cos(heading*PI/180);
			northing += turnRadius * sin((turnAngle + heading)*PI/180) - turnRadius * sin(heading*PI/180);
			heading += dt*180*t.read()/(PI * turnRadius);
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
