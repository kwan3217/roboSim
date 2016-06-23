
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

class Simulator
{
	private:
		double easting;
		double northing;
		double heading;
		double turnRadius;
		const double wheelBase;
		
		
	public:
		Simulator(double h = 0, double e = 0, double n = 0);
		void update(const Servo & s, const Servo & t, double c);
		void showVector() const;
		void test();
};

class roboBrain
{
	private:
		double velocity;
		double easting;
		double northing;
		double heading;
		double turnRadius;
		double wheelBase;
		void readThrottle();
		void setPosition(double);
		void setWheelAngle(double);
		void setTurnRadius();
	public:
		Robot(double h = 0, double e = 0, double n = 0);
		Servo throttle;
		Servo steering;	
		void update(double);
		
		void navigateCompass();
		void navigateGps();
		
		double guide() const;
		void control();
		void log() const;
};

#endif
