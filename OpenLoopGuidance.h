#ifndef OPENLOOPGUIDANCE_H_
#define OPENLOOPGUIDANCE_H_

#include "robot.h"

class OpenLoopGuidance: public Controller {
private:
  double* t; ///< Time column of the table
  char* servoChannel; ///< Servo Channel column of the table
  int* servoSetting; ///< Servo setting column of the talbe
  int currentRow;
public:
  OpenLoopGuidance(Interface& Linterface, double* Lt, char* LservoChannel, int* LservoSetting):Controller(Linterface),t(Lt),servoChannel(LservoChannel),servoSetting(LservoSetting),currentRow(0) {};
  virtual void control();			//give data to servos, which will then be read by the simulation
  ~OpenLoopGuidance() {};
};

#endif /* OPENLOOPGUIDANCE_H_ */
