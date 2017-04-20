/*
 * Interface.h
 */
#ifndef PassengerInterface_h
#define PassengerInterface_h

#include "Interface.h"
#include "Log.h"
/** Abstract interface to a servo. This is write-only, because a real physical servo is write-only */
class PassengerServo:public Servo {
private:
  Interface& interface;
  Log& log;
  int channel;
public:
  PassengerServo(Interface& Linterface, Log& Llog, int Lchannel):interface(Linterface),log(Llog),channel(Lchannel) {};
  virtual ~PassengerServo() {};
  /** Command the servo to a particular position
   * @param n new commanded value in DN
   */
  virtual void write(int n) {log.start(Log::Apids::servo,"PassengerServoLog");log.write(interface.time(),"t");log.write(channel,"channel");log.write(n,"cmd");log.end();};
};

/** Abstract class representing the interface between the robot navigation, guidance, and control (GNC) software
 * and the real or virtual robot it is controlling.
 */
class PassengerInterface:public Interface {
private:
  Interface& interface; ///< Use this interface to get to the real sensors
  Log& log;
  PassengerServo psSteer;
  PassengerServo psThrottle;
public:
  virtual bool checkPPS(fp& t)                                                      {return interface.checkPPS(t);};
  virtual bool checkNavChar()                                                       {return interface.checkNavChar();};
  virtual char readChar()                                                           {return interface.readChar();};
  virtual fp time()                                                                 {return interface.time();};
  virtual bool button(int pin=17)                                                   {return interface.button(pin);};
  virtual void readOdometer(uint32_t &timeStamp, int32_t &wheelCount, uint32_t &dt) {       interface.readOdometer(timeStamp, wheelCount, dt);};
  virtual bool readGyro(int16_t g[])                                                {return interface.readGyro(g);};
  virtual bool readAcc(int16_t g[])                                                 {return interface.readAcc(g);};
  virtual bool readMPU(int16_t a[], int16_t g[], int16_t& t)                        {return interface.readMPU(a,g,t);};
  virtual bool readGyro(int16_t g[], int16_t& t)                                    {return interface.readGyro(g,t);};
  virtual bool readMag(int16_t b[])                                                 {return interface.readMag(b);};

  /** Construct a passenger interface
   * @param Linterface reference to Interface object which will be used to echo the real sensors
   * @param Llog reference to Log object used to record servo commands
   */
  PassengerInterface(Interface& Linterface, Log& Llog):
  Interface(psSteer,psThrottle), interface(Linterface), log(Llog), psSteer(*this,log,0),psThrottle(*this,log,1) {};
  /** Destructor. Doesn't do anything explicitly, but it's good form to include a virtual destructor
   * for any class which has virtual methods.
   */
  virtual ~PassengerInterface() {};
};

#endif /* ROBOT_H_ */
