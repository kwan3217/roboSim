#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include "HeaderSimRobo.h"

using namespace std;



roboBrain::roboBrain(double h, double e, double n, Interface& Linterface):
heading(h), pos(e, n),interface(Linterface),headingChange(0),desiredHeading(0),
partCount(0), charsReceived(0), sentenceStart(false)
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

double roboBrain::guide(){
	const int wpcount = sizeof(waypoints)/sizeof(waypoint);
	if((waypoints[nowpoint]- waypoints[nowpoint - 1]).dot(waypoints[nowpoint] - pos) < 0){
		nowpoint += 1;
	}
	if(nowpoint >= wpcount){
		headingChange=400;
		return headingChange;
	}
    desiredHeading = (waypoints[nowpoint]-pos).heading();

	headingChange = desiredHeading - heading;
	if(headingChange > 180){
		headingChange -= 360;
	}
	else if (headingChange < -180){
		headingChange += 360;
	}
	return headingChange;
}

void roboBrain::control(double headingChange){

	if(headingChange >= 300){
		interface.throttle.write(0);
		interface.steering.write(0);
		return;
	}
	interface.throttle.write(64);
	interface.steering.write(headingChange * double (127)/180);
}

void roboBrain::update(double t){

}

void roboBrain::navigateCompass(){
	
	//Intentionally ugly -- this won't work in general when the interface isn't a Simulator
	(static_cast<Simulator&>(interface)).cheatHeading(heading);
}

void roboBrain::navigateGPS(){
	//TO BE CLEANED ONCE COMPLETED --> start by reading in the NMEA sentence from the simulator as possible.
	//				Then look at the received data an pull the latitude, longitude, speed and heading.
	//				Use math to convert latitude and longitude to northing and easting, then place
	//				these values in the robot's data. Once I have finished the basic version, I need to 
	//				account for the .4 second lag and create projected easting, northing data from that.

//	if(interface.checkPPS() != pps){	//if PPS doesn't match, reset and prep for a new sentence
//		sentenceDone = false;
//		charsReceived = 0;
//		partCount = 0;
//		pps = interface.checkPPS();
//	}
//	if(sentenceDone) return;		//if I've taken in an entire sentence, then I don't need to do anything else here.
	while(interface.checkNavChar()){
		char ch = interface.readChar();
		if(ch == '$') sentenceStart = true;
		if(!sentenceStart) break;

		nmeaReceived[charsReceived] = ch //interface.readChar(); //<--Maybe uncomment this later if ch stops being our character tester
		if(nmeaReceived[charsReceived])
		if(nmeaReceived[charsReceived] == ',' || nmeaReceived[charsReceived] == '*' || nmeaReceived[charsReceived] == '$'){
			partitions[partCount] = charsReceived;
			partCount += 1;
		}
		if(partCount == 2)
			if(strncmp(nmeaReceived, "$GPRMC", 6) != 0){
				sentenceStart = false;
				charsReceived = 0;
				partCount = 0;
				break;
			}
		charsReceived += 1;
		if(partCount == checksumSpot && charsReceived == partitions[checksumSpot]){// IGNORING CHECKSUM FOR NOW, REMOVE THIS AND DECOMMENT FOLLOWING BLOCK OF COMMENTS LATER
			sentenceStart = false;
			charsReceived = 0;
			partCount = 0;
//		if(partCount == checksumSpot && charsReceived == partitions[checksumSpot] + 3){	//there should be just two characters received after the checksum asterisk if sentence is done
//			sentenceDone = true;
//			sentenceStart = false;
//			char checksum = 0;
//			for(int i=1;i<checksumSpot;i++) {
//				checksum ^= nmeaReceived[i];
//			}
//			nmeaReceived[checksumSpot + 3] = '\0';
//			if(nmeaReceived[statusSpot + 1] == 'A' && checksum == strtol(nmeaReceived + checksumSpot + 1, NULL, 16)){	//validate checksum
//
				for(int i = 0; i < charsReceived + 1 /*REMOVE +1 AFTER DECOMMENT*/; i++){
					if(nmeaReceived[i] == ',' || nmeaReceived[i] == '*')
						nmeaReceived[i] = '\0';
				}



				//Take data from the desired partitions and use atod functions to translate them into numbers I can use
				double latpos = atof(nmeaReceived + latSpot + 1);
				int degrees = floor(latpos)/100;
				double minutes = (latpos - degrees * 100);
				double latdd = degrees + minutes/60;

				if(nmeaReceived[nsSpot+1] == 'S') latdd = -latdd;


				double longpos = atof(nmeaReceived + longSpot + 1);
				int degrees = floor(longpos)/100;
				double minutes = (longpos - degrees * 100);
				double longdd = degrees + minutes/60;


				if(nmeaReceived[ewSpot+1] == 'W') longdd = -longdd;


				heading = atof(nmeaReceived + headingSpot + 1);
				if(lat0 > 90 && long0 > 180){
					lat0 = latdd;
					long0 = longdd;
				}
				else{
					//compare new lat, long with original, then give a new easting and northing to pos
					pos.northing = (latdd - lat0)*re*PI/180;
					pos.easting = (longdd - long0)*re*cos(lat0)/180;
				}
			}
		}
//	}		DECOMMENT WHEN REINTRODUCING CHECKSUM VALIDATION
	

	//Intentionally ugly -- this won't work in general when the interface isn't a Simulator
	//(static_cast<Simulator&>(interface)).cheatNavigate(pos.easting,pos.northing);
}

void roboBrain::showVector() const{
	printf("%i,%06.2f,%06.2f,%06.2f, %07.2f, ",nowpoint,waypoints[nowpoint].easting, waypoints[nowpoint].northing,desiredHeading,headingChange);
}
