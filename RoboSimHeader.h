
#ifndef ALL_MY_BOTS
#define ALL_MY_BOTS
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

class Simulator //where the robot actually is
{
	private:
		double easting;
		double northing;
		double heading;
		double turnRadius;
		const double wheelBase;
		
		
	public:
		Simulator(double h = 0, double e = 0, double n = 0);
		void update(const Servo & s, const Servo & t, double c);	//only method needed to tell the sim where it is right now.
		void showVector() const;									//reports data for storage in .csv file
		void test();												//for... testing?
		friend roboBrain;
};

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
		void control(double, double);			//give data to servos, which will then be read by the simulation
		void log() const;		//take data?
		
		void showVector() const;
		friend Simulator;
};

#endif
