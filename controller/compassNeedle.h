/*
 * compassNeedle.h
 */

#ifndef COMPASSNEEDLE_H_
#define COMPASSNEEDLE_H_

#include "robot.h"

class compassNeedle: public Controller //where the robot thinks it is
{
	private:
		double epochTime;
		double dt;
		double heading;		///< Perceived heading
		double desiredHeading;	///< Heading needed for the robot to be on course
		double headingChange;	///< Heading change needed for the robot to be on course
		void updateTime();
		int16_t zDN,steerCmd;
		double yawRate;
	public:
		compassNeedle(Interface& Linterface, double h);
		void navigate() {navigateCompass();};
		void navigateCompass();	//
		void guide();
		void control();			//give data to servos, which will then be read by the simulation
		void log() const;		//take data?
		void showVector() const;
};



#endif /* COMPASSNEEDLE_H_ */
