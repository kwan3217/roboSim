/*
 * roboBrain.h
 */

#ifndef ROBOBRAIN_H_
#define ROBOBRAIN_H_

#include "robot.h"

class roboBrain: public Controller //where the robot thinks it is
{
	private:
		enum nmeaParts {	///< constants to track where in the partitions array is the spot for each part of the nmea sentence
			timeSpot,		///< UTC time
			statusSpot,		///< Active/Void
			latSpot,		///< latitude
			nsSpot,			///< north or south of equator
			longSpot,		///< longitude
			ewSpot,			///< east or west of the prime meridian
			speedSpot, 		///< space for speed in knots
			headingSpot, 	///< heading
			dateSpot, 		///< current date in ddmmyyyy
			magSpot,		///< magnetic variation I DON'T KNOW WHAT THIS MEANS YET
			magewSpot,		///< magnetic variation east or west
			modeSpot,		///< mode of nmea sentence
			checksumSpot	///< checksum
		};

		double lat0 = 100;		///< latitude at time 0, initialized to 100 for navigateGPS to set, then compare with new GPS data
		double long0 = 200;		///< longitude at time 0, initialized to 200 for navigateGPS to set, then compare with new GPS data
	    waypoint pos;		///< perceived position
		double heading;		///< Perceived heading
		double desiredHeading;	///< Heading needed for the robot to be on course
		double headingChange;	///< Heading change needed for the robot to be on course
		int nowpoint = 0;	///< Current waypoint for robot to navigate to
		static const waypoint waypoints[];	///< Array of waypoints for the robot
		char nmeaReceived[256];	///< NMEA sentence received by robot
		int charsReceived;	///< Number of characters in NMEA sentence currently received
		double pps = -1;	///< epoch time of the last PPS in seconds, initialized negative to ensure it is not equal with simulator pps at startup
		bool sentenceStart;	///< begin status of the latest NMEA sentence
		int partCount;		///< number of partitions (commas and asterisk) detected in the current sentence
		int partitions[20];	///< locations of the partitions in the NMEA sentence

		int32_t wheelCount; ///< count of sector changes taken last time by the odometer
		uint32_t timeStamp; ///< epoch time(CURRENTLY IN MILLISECONDS) of last time readOdometer() was used
		uint32_t dt;	///< time between most recent call of readOdometer() and the call of readOdometer() previous to that.
	public:
		roboBrain(double h, double e, double n, Interface& Linterface);
		void navigateCompass();	//
		void navigateGPS();
		void navigateOdometer();
		virtual void navigage() {navigateCompass();navigateGPS();navigateOdometer();}
		virtual void guide();
		virtual void control();			//give data to servos, which will then be read by the simulation
		void log() const;		//take data?
		void showVector() const;
};



#endif /* ROBOBRAIN_H_ */
