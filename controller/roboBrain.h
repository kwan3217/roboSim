/*
 * roboBrain.h
 */

#ifndef ROBOBRAIN_H_
#define ROBOBRAIN_H_

#include "Interface.h"
#include "controller.h"
#include "waypoint.h"
#include "Quaternion.h"
#include <gps.h>

class roboBrain: public Controller {
private:
  //Sensor read timing
  fp t;  ///< Epoch time of most recent sensor read (just before sensor read)
  fp t1; ///< Epoch time just after most recent sensor read (t1-t is how long the sensor read took)
  fp ot; ///< Epoch time of previous sensor read
  fp dt; ///< Interval between most recent and previous sensor read
  fp tnextodo=0; ///< Time of next robodometer read or write
  const fp dtnextodo=0.1; ///< Time between robodometer read/write
  //Gyrocompass reading
  bool gValid=false;    ///< True if last gyroscope reading was valid
  int16_t g[3];         ///< gyroscope raw reading
  //GPS reading
  fp pps = -1;	///< epoch time of the last PPS in seconds, initialized negative to ensure it is not equal with simulator pps at startup
  double lat;   ///< Latitude of most recent fix
  double lon;   ///< Longitude of most recent fix
  int gps_mode; ///< 0=no gps, 1=no pos, 2=2D pos, 3=3D pos
  fp t_gps_collected;             ///< Epoch time that GPS fix was updated
  double t_gps_valid;             ///< Unix time of GPS fix timestamp (probably identical or close to pps)
  bool hasFixForPPS=false;        ///<true if we have the fix for this PPS
  bool processedFixForPPS=false;  ///<true if we have incorporated this fix into the position estimate
  //button reading
  fp tbutton=9.9e9;
  // Odometer reading
  int32_t oldWheelCount,deltaWheelCount;
  bool odoValid;
  uint32_t t_odo, dt_odo;
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
  //GPS navigation
  double lat0;		///< latitude at go time
  double lon0;		///< longitude at go time
  double clat0;
  waypoint thisFix;              ///< Northing and easting at most recent PPS
  fp tthisFix;                   ///< Epoch time of this fix
  waypoint lastFix;              ///< Northing and easting at previous PPS
  fp tlastFix;                   ///< Epoch time of last fix
  int nowpoint = 0;	///< Current waypoint for robot to navigate to
  static const waypoint waypoints[];	///< Array of waypoints for the robot
  static const int wpcount;
  waypoint odoDeltaPos;
  fp yawRate;
  int32_t wheelCount; ///< count of sector changes taken last time by the odometer
  uint32_t timeStamp; ///< epoch time(CURRENTLY IN MILLISECONDS) of last time readOdometer() was used
  uint32_t dtOdometer;	///< time between most recent call of readOdometer() and the call of readOdometer() previous to that.
  //Navigation variables
  Quaternion q;       ///< Current robot orientaiton estimate
  waypoint pos;       ///< Current robot position estimate
  Log& log;
  //Control Variables
  uint16_t steeringCmd,throttleCmd,mode;
public:
  roboBrain(Interface& Linterface, Log& Llog):
    Controller(Linterface), t(0), ot(0), dt(0),log(Llog) { }
  bool navigateCompass();	//
  void navigateGPS();
  bool navigateOdometer();
  virtual void readSensors();
  virtual void navigate();
  virtual void guide();
  virtual void control();			//give data to servos, which will then be read by the simulation
  fp getHeading() {return heading;};
  uint16_t getSteeringCmd() {return steeringCmd;};
  uint16_t getThrottleCmd() {return throttleCmd;};
  fp getT() {return t;};
};

#endif /* ROBOBRAIN_H_ */
