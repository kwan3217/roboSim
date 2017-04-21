/*
 * Playback.h
 */

#ifndef PLAYBACK_H_
#define PLAYBACK_H_

/** Abstract class representing the interface between the robot navigation, guidance, and control (GNC) software
 * and the real or virtual robot it is controlling.
 */
class NMEAPlayback: public Interface {
private:
	FILE* inf;
	double t=0;
	int pps=0;
	char c; ///< Next character to give to robot
	SimServo nullSteering;
	SimServo nullThrottle;
public:
	virtual double checkPPS();
	virtual bool checkNavChar();
	virtual char readChar();
	virtual double time();

	/** (Null) Read the gyroscope
	 */
	virtual void readGyro(int g[]) {};
	/** Construct a robot interface
	 * @param infn
	 * @param Lthrottle reference to servo object which controls throttle
	 */
	NMEAPlayback(char* infn):nullSteering(0,0,0,0,0),nullThrottle(0,0,0,0,0),Interface(nullSteering,nullThrottle) {inf=fopen(infn,"r");};
	/** Destructor */
	virtual ~NMEAPlayback() {fclose(inf);};
};




#endif /* PLAYBACK_H_ */
