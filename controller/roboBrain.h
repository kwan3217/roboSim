/*
 * roboBrain.h
 */

#ifndef ROBOBRAIN_H_
#define ROBOBRAIN_H_

#include "Interface.h"
#include "controller.h"
#include "waypoint.h"
#include "Quaternion.h"

class roboBrain: public Controller {
private:
  //Sensor read timing
  fp t;  ///< Epoch time of most recent sensor read (just before sensor read)
  fp t1; ///< Epoch time just after most recent sensor read (t1-t is how long the sensor read took)
  fp ot; ///< Epoch time of previous sensor read
  fp dt; ///< Interval between most recent and previous sensor read
  //Gyrocompass reading
  bool gValid=false;    ///< True if last gyroscope reading was valid
  int16_t g[3];         ///< gyroscope raw reading
  //Average G
  fp offSet[3];
  static const int bufferDiscard = 300;
  static const int bufferMax = 1500;
  int ofBuffer[bufferMax][3];
  int bufferSpot;
  void fillBuffer();
  void setOffSet();
  //Gyrocompass navigation
  fp heading;		///< Perceived heading, degrees east of true north, [0..360)
  //Compass guidance
  fp desiredHeading;	///< Heading needed for the robot to be on course, degrees east of true north, [0..360)
  fp headingChange;	///< Heading change needed for the robot to be on course, degrees right of current heading, [-180,180)
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
    magSpot,		///< magnetic variation (difference between magnetic and true north)
    magewSpot,		///< magnetic variation east or west
    modeSpot,		///< mode of nmea sentence
    checksumSpot	///< checksum
  };
  //Odometer reading
  fp lat0 = 100;		///< latitude at time 0, initialized to 100 for navigateGPS to set, then compare with new GPS data
  fp long0 = 200;		///< longitude at time 0, initialized to 200 for navigateGPS to set, then compare with new GPS data
  fp latdd;		///< latitude at current time
  fp longdd;		///< longitude at current time
  int nowpoint = 0;	///< Current waypoint for robot to navigate to
  static const waypoint waypoints[];	///< Array of waypoints for the robot
  static const int wpcount;
  char nmeaReceived[256];	///< NMEA sentence received by robot
  int charsReceived;	///< Number of characters in NMEA sentence currently received
  fp pps = -1;	///< epoch time of the last PPS in seconds, initialized negative to ensure it is not equal with simulator pps at startup
  bool sentenceStart;	///< begin status of the latest NMEA sentence
  int partCount;		///< number of partitions (commas and asterisk) detected in the current sentence
  int partitions[20];	///< locations of the partitions in the NMEA sentence
  int32_t oldWheelCount,deltaWheelCount;
  waypoint odoDeltaPos;
  int16_t zDN,steerCmd;
  fp yawRate;
  int servoCommand;
  int32_t wheelCount; ///< count of sector changes taken last time by the odometer
  uint32_t timeStamp; ///< epoch time(CURRENTLY IN MILLISECONDS) of last time readOdometer() was used
  uint32_t dtOdometer;	///< time between most recent call of readOdometer() and the call of readOdometer() previous to that.
  //Navigation variables
  Quaternion q;       ///< Current robot orientaiton estimate
  waypoint pos;       ///< Current robot position estimate
  Log& log;
public:
  roboBrain(fp h, fp e, fp n, Interface& Linterface, Log& Llog):
    Controller(Linterface), t(0), ot(0), dt(0), heading(h), pos(e, n),log(Llog) { }
  bool navigateCompass();	//
  void navigateGPS();
  bool navigateOdometer();
  virtual void readSensors();
  virtual void navigate();
  virtual void guide();
  virtual void control();			//give data to servos, which will then be read by the simulation
  fp getHeading() {return heading;};
};

#endif /* ROBOBRAIN_H_ */
