/*
 * robot.h
 */
#include<stdint.h>
#include <cmath>
#include "Vector.h"

#ifndef ROBOT_H_
#define ROBOT_H_
const double re=6378137.0;     ///< radius of Earth, used to convert between lat/lon and northing/easting
const double wheelRadius = .03175;

class waypoint: public Vector<2,fp> {
public:
  fp& easting()  {return comp[0];}; ///< Associate the name "easting" with component 0
  fp& northing() {return comp[1];}; ///< Associate the name "northing" with component 1
  fp easting() const {return comp[0];}; ///< Get the easting component, used in const context (see http://www.cprogramming.com/tutorial/const_correctness.html)
  fp northing() const {return comp[1];};///< Get the northing component, used in const context
  /** Compute the heading of this waypoint relative to the origin.
   * @return Heading in degrees east of true north, from 0 to almost 360
   */
  fp heading() {
    fp result=atan2(easting(),northing())*180/PI;
    if(result<0) result+=360;
    return result;
  }
  /** Construct a waypoint
   * @param e easting value
   * @param n northing value
   */
  waypoint(fp e=0, fp n=0) {easting()=e;northing()=n;};
  /** Copy constructor */
  waypoint(Vector<2,fp> other):waypoint(other[0],other[1]) {};
};

/** Abstract interface to a servo. This is write-only, because a real physical servo is write-only */
class Servo {
	public:
	/** Command the servo to a particular position
	 * @param n new commanded value in DN
	 */
		virtual void write(int n)=0;
		virtual ~Servo() {};
};

/** Abstract class representing the interface between the robot navigation, guidance, and control (GNC) software
 * and the real or virtual robot it is controlling.
 */
class Interface {
private:
	public:
	    /** Gets time of last PPS in seconds from epoch. You may call this function as often as you like -- it will
	     * only return a new value whenever a new PPS has arrived. This represents the time at which the *next* GPS fix
	     * is valid, IE a PPS pulse is generated, then some time later a new position is transmitted.
	     * @return epoch time of the most recent PPS in seconds
	     */
		virtual double checkPPS() = 0;
		/** Check if a character of GPS data is available
		 * @return true if a character is available, false if not
		 */
		virtual bool checkNavChar() = 0;
		/** Get the next character of GPS data. Only call this if checkNavChar() returns true, return value is undefined
		 * if checkNavChar() is false.
		 * @return one character of GPS data in NMEA format
		 */
		virtual char readChar() = 0;

		/** Get the current time
		 * @return Current epoch time in seconds
		 */
		virtual double time()=0;
		/** Check whether a button is pushed
                 * @param[in] pin WiringPi pin number for the pin to check. The robot will have a button on pin 17
		 * @return true if the button is pushed (pin is low voltage)
		 */
		virtual bool button(int pin=17)=0;
		/**	Read the odometer
		 * @param timeStamp [out] time of last time readOdometer was called in microseconds
		 * @param wheelCount [out] number of sectors read since the odometer was reset
		 * @param dt [out] time since last call of readOdometer in microseconds
		 * 		 */
		virtual void readOdometer(uint32_t &timeStamp, int32_t &wheelCount, uint32_t &dt)=0;
		/** Read the gyroscope
		 * @param g [out] vector of gyroscope readings, in DN. One DN typically represents
		 * a constant fraction of a degree per second rotation rate around each axis. Number
		 * is a raw readout of the sensor, in two's complement integer, in proper axis order,
		 * IE element 0 is X, 1 is Y, and 2 is Z.
		 */
		virtual void readGyro(int g[]) = 0;
		Servo& steering; ///< Reference to steering servo object
		Servo& throttle; ///< Reference to throttle servo object
		/** Construct a robot interface
		 * @param Lsteering reference to servo object which controls steering
		 * @param Lthrottle reference to servo object which controls throttle
		 */
		Interface(Servo& Lsteering, Servo& Lthrottle):steering(Lsteering),throttle(Lthrottle) {};
		/** Destructor. Doesn't do anything explicitly, but it's good form to include a virtual destructor
		 * for any class which has virtual methods.
		 */
		virtual ~Interface() {};
};

#endif /* ROBOT_H_ */
