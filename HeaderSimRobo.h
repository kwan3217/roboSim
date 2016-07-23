#ifndef ALL_MY_BOTS
#define ALL_MY_BOTS
const double PI = 2*acos(0.0); ///< Circle constant
const double re=6378137.0;     ///< radius of Earth, used to convert between lat/lon and northing/easting

/** Two-dimensional vector, implementing Northing and Easting concept. This should probably be called Vector instead of waypoint */
class waypoint {
public:
	double easting;  ///< Easting coordinate in meters east of origin
	double northing; ///< Northing coordinate in meters north of origin
	/** Add another waypoint to this waypoint, in the vector addition sense
	 * @param rhs the other waypoint
	 * @return a reference to this waypoint, which is now incremented by rhs
	 */
	waypoint& operator+=(const waypoint& rhs) {
	   easting+=rhs.easting;
	   northing+=rhs.northing;
	   return *this;
	}
	/** Subtract another waypoint from this waypoint, in the vector subtraction sense
	 * @param rhs the other waypoint
	 * @return a reference to this waypoint, which is now decremented by rhs
	 */
	waypoint& operator-=(const waypoint& rhs) {
	   easting-=rhs.easting;
	   northing-=rhs.northing;
	   return *this;
	}
	/** Compute the dot product of this waypoint and another waypoint
	 * @param b the other waypoint
	 * @return dot product value
	 */
	double dot(const waypoint& b) {
		return northing*b.northing+easting*b.easting;
	}
	/** Compute the heading of this waypoint relative to the origin.
	 * @return Heading in degrees east of true north, from 0 to almost 360
	 */
	double heading() {
		double result=atan2(easting,northing)*180/PI;
		if(result<0) result+=360;
		return result;
	}
	/** Construct a waypoint
	 * @param e easting value
	 * @param n northing value
	 */
	waypoint(double e, double n):easting(e),northing(n) {};
	waypoint():easting(0),northing(0) {};
};

/** Add two waypoints (vectors) together to produce a new waypoint (vector)
 * @param lhs left-hand-side waypoint
 * @param rhs right-hand-side waypoint
 * @return a new waypoint which is the vector sum of the two input waypoints
 */
inline waypoint operator+(waypoint lhs, const waypoint& rhs) {
  lhs += rhs;
  return lhs;
}

/** Subtract two waypoints (vectors) together to produce a new waypoint (vector)
 * @param lhs left-hand-side waypoint
 * @param rhs right-hand-side waypoint
 * @return a new waypoint which is the vector difference of the two input waypoints
 */
inline waypoint operator-(waypoint lhs, const waypoint& rhs) {
  lhs -= rhs;
  return lhs;
}

/** Abstract interface to a servo. This is write-only, because a real physical servo is write-only */
class Servo {
	public:
	/** Command the servo to a particular position
	 * @param n new commanded value in DN
	 */
		virtual void write(int n)=0;
		virtual ~Servo() {};
};

/** Simulated servo. Keeps track of physical value of servo (steering angle, speed of robot, etc) and manages servo slewing */
class SimServo:public Servo {
private:
	int commanded; ///< Commanded value in DN
	const int cmdmin; ///< Minimum allowed commanded value in DN
	const int cmdmax; ///< Maximum allowed commanded value in DN
	double physical;  ///< Current value of servo in physical units
	const double physmin; ///< Physical value corresponding to minimum possible command
	const double physmax; ///< Physical value corresponding to maximum possible command
	const double slewrate; ///< Rate of change of physical value in response to command changes, in physical units/second
public:
	/** Construct a servo
	 * @param cmdmin value for cmdmin field
	 * @param cmdmax value for cmdmax field
	 * @param physmin value for physmin field
	 * @param physmax value for physmax field
	 * @param slewrate value for slewrate field
	 */
	SimServo(int cmdmin, int cmdmax, double physmin, double physmax, double slewrate);
	virtual void write(int n);
	/** Read the current value of the servo
	 * @return current value of the servo in physical units. May not match commanded value, due to slew rate.
	 */
	double read() const {return physical;}
	/** Step the servo in time. This allows the physical value to slew to the commanded value.
	 * @param t Interval since last timestep in seconds
	 */
	void timeStep(double t);
	/** Run the test case for the servo. This test case can't be evaluated automatically. It should be run through
	 * a chart on a spreadsheet in order for a human to evaluate it.
	 */
	static void test();
	/** Destructor. Doesn't do anything explicitly, but it's good form to include a virtual destructor
	 * for any class which has virtual methods.
	 */
    virtual ~SimServo() {};

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

class roboBrain //where the robot thinks it is
{
	private:
	    waypoint pos;
		double heading;
		double desiredHeading;
		double headingChange;
		int wayTarget;
		Interface& interface;
		int nowpoint = 1;
		static const waypoint waypoints[];

	public:
		roboBrain(double h, double e, double n, Interface& Linterface);
		/** Destructor. Doesn't do anything explicitly, but it's good form to include a virtual destructor
		 * for any class which has virtual methods.
		 */
		virtual ~roboBrain() {};

		void update(double);	//takes time and updates location. For now, it'll just be a copy of simulation's update.

		void navigateCompass();	//
		void navigateGPS();		//generate garbage data based on Simulation. For now, just take Simulation's data

		double guide() ;	//return a heading change, work with current object data. Determine if previous waypoint has been passed
		void control(double);			//give data to servos, which will then be read by the simulation
		void log() const;		//take data?

		void showVector() const;
};

/** Simulated implementation of robot interface. This keeps track of where the robot actually is
 *
 */
class Simulator : public Interface
{
	private:
	    double lat0;            ///< Origin latitude in degrees north of WGS84 equator
	    double lon0;            ///< Origin longitude in degrees east of WGS84 prime meridian
		waypoint pos;           ///< Actual position of robot relative to lat0 and lon0
		double heading;         ///< Actual heading in degrees east of True North
		double turnRadius;      ///< Turning radius in meters, but 0 means straight ahead
		const double wheelBase=0.3; ///< Wheelbase coefficient in meters
		const double ppsInterval=1.0; ///< Position is available at this interval in seconds
		double pps;             ///< epoch time of the last PPS in seconds
		const double dpps=1.0;  ///< interval between GPS fixes in seconds
		const double nmeaDelay=0.4; ///< NMEA sentence isn't available until this long after PPS
		char nmea[256];         ///< NMEA sentence to send to robot
		const double baudRate=4800.0; ///< Simulated baud rate, characters only become available according to this rate
		const double bitsPerChar=10.0; ///< Number of bit times required to send one char, 10 because we have one start bit, 8 data bits, and one stop bit
		const double charTime=1.0/(baudRate/bitsPerChar); ///< Time required to send one char in seconds
		int charsSent;          ///<Number of characters in this sentence which have already been transmitted
		double epochTime;               ///< epoch time - current time relative to start of simulation in seconds

		SimServo simSteering;  ///<Actual instance of SimServo. Constructor points Interface::steering at this.
		SimServo simThrottle;  ///<Actual instance of SimServo. Constructor points Interface::throttle at this.

		void generateNewFix();

		/** Return the current latitude. This is calculated from the current northing and the initial latitude.
		 * @return Latitude in degrees north of WGS84 equator
		 */
		double lat() const {return lat0+(pos.northing/re)*180.0/PI;};
		/** Return the current longitude. This is calculated from the current easting and the initial latitude and longitude.
		 * Note that the cosine factor necessary for converting easting to longitude is calculated from the initial latitude,
		 * not the current latitude. This doesn't matter at the small scale of an AVC course. If you are considering an
		 * area big enough for the change in latitude to make a difference, easting and northing are no longer valid concepts.
		 * @return Longitude in degrees east of WGS84 prime meridian (Note that all longitudes in continental USA are negative).
		 */
		double lon() const {return lon0+(pos.easting/cos(lat0*PI/180.0)/re)*180.0/PI;}
		/** Convert a measurement in degrees to a measurement in degrees and minutes, appropriate for NMEA output
		 * @param deg Angle in decimal degrees
		 * @return Same angle in degrees*100+minutes, so when printed in decimal, the format is DDMM.MMMMM, IE the units and
		 * tens digits are minutes, the higher digits are degrees, and the fractional digits are fraction of minutes.
		 */
		static double deg2dm(double deg) {return floor(deg)*100+(deg-floor(deg))*60;}
	public:
		Simulator(double Lh = 0, double lat = 0, double lon = 0);
		/** Destructor. Doesn't do anything explicitly, but it's good form to include a virtual destructor
		 * for any class which has virtual methods.
		 */
        virtual ~Simulator() {};
		void update(double dt);
		void showVector() const;									//reports data for storage in .csv file
		static void testNMEA(); ///< Test the NMEA generation code
		
		virtual double checkPPS();
		virtual bool checkNavChar(); 
		virtual char readChar();
		virtual void readGyro(int g[]);
		virtual double time() {return epochTime;};
		/** Back-door direct access to navigation state
		 * @param e [out] easting in meters east of initial longitude
		 * @param n [out] northing in meters east of initial latitude
		 */
        void cheatNavigate(double& e, double& n) {e=pos.easting; n=pos.northing;};
		/** Back-door direct access to heading
		 * @param h [out] heading in degrees east of true north
		 */
        void cheatHeading(double& h) {h=heading;};
				
};

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

#endif
