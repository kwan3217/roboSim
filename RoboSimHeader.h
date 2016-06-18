
#ifndef ALL_MY_BOTS
#define ALL_MY_BOTS
class Servo
{
	private:
		double setting;
		char mode;
	public:
		Servo();
		void Mode(char);
		void write(double n);
		double read() const;
};

class Robot
{
	private:
		double throttleVal;
		double currentVelocity;
		double targetVelocity;
		double velocity;
		double latitude;
		double longitude;
		double heading;
		double wheelAngle;
		double turnRadius;
		double wheelBase;
		double rotationSpeed;
		void readThrottle();
		void setVelocity(double);
		void setPosition(double);
		void setWheelAngle(double);
		void setTurnRadius();
	public:
		Robot(double h = 0, double lat = 0, double lon = 0);
		void update(double);
		void showPosition() const;
		Servo throttle;
		Servo steering;		
};

#endif
