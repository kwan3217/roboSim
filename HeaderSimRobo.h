
#ifndef ALL_MY_BOTS
#define ALL_MY_BOTS
const double PI = 2*acos(0.0);
const double re=6378137.0; ///< radius of Earth, used to convert between lat/lon and northing/easting


struct waypoint {
	double easting;
	double northing;
};

class Servo
{
	private:
		int commanded;
		const int cmdmin;
		const int cmdmax;
		double physical;
		const double physmin;
		const double physmax;
		const double slewrate;
	public:
		Servo(int cmdmin, int cmdmax, double physmin, double physmax, double slewrate);
		void write(int n);
		double read() const {return physical;}
		void timeStep(double t);
		static void test();
};

class roboBrain;

class Interface
{
	public:
		virtual double checkPPS() = 0;
		virtual bool checkNavChar() = 0; 
		virtual char readChar() = 0;
		
		virtual void readGyro(double[]) = 0;
};

class Simulator : public Interface //where the robot actually is
{
	private:
	    double lat0;            ///< Origin latitude in degrees north of WGS84 equator
	    double lon0;            ///< Origin longitude in degrees east of WGS84 prime meridian
		double easting;         ///< Actual easting coordinate of robot in meters east of lon0
		double northing;        ///< Actual northing coordinate of robot in meters north of lat0
		double heading;         ///< Actual heading in degrees east of True North
		double turnRadius;      ///< Turning radius in meters, but 0 means straight ahead
		double time;               ///< epoch time - current time relative to start of simulation in seconds
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

		void generateNewFix();
	public:
		Simulator(double Lh = 0, double lat = 0, double lon = 0);
		void update(const Servo & s, const Servo & t, double c);	//only method needed to tell the sim where it is right now.
		void showVector() const;									//reports data for storage in .csv file
		void test();
		
		virtual double checkPPS();
		virtual bool checkNavChar(); 
		virtual char readChar();
		virtual void readGyro(double []);
				
		friend roboBrain;
};

extern Simulator roboSim; //This is just temporary, we should restructure roboBrain so it holds a reference to an Interface

class roboBrain //where the robot thinks it is
{
	private:
		double easting;
		double northing;
		double heading;
		double turnRadius;
		const double wheelBase;
		int wayTarget;
		
	public:
		roboBrain(double h = 0, double e = 0, double n = 0);
		
		Servo throttle;
		Servo steering;	
		void update(double);	//takes time and updates location. For now, it'll just be a copy of simulation's update.
		
		void navigateCompass();	//
		void navigateGPS();		//generate garbage data based on Simulation. For now, just take Simulation's data
		
		double guide() const;	//return a heading change, work with current object data. Determine if previous waypoint has been passed
		void control(double);			//give data to servos, which will then be read by the simulation
		void log() const;		//take data?
		
		void showVector() const;
		friend Simulator;
};

#endif
