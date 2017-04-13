#ifndef SIMHARDWARE_H_
#define SIMHARDWARE_H_

#include "HardwarePi.h"
#include "Simulator.h"
#include <stdio.h>

class SimHardware: public Simulator
{
  private:
    HardwarePiInterfaceArduino hardware;
  public:
    SimHardware(double Lh = 0, double lat = 0, double lon = 0): Simulator(Lh, lat, lon) {};
    virtual bool button(int n = 17) {hardware.button(n);};
    virtual double time() {return hardware.time();};
    virtual void throttle (int n) {printf("throttlestart\n"); hardware.throttle(n); Simulator::throttle(n); printf("throttleend\n");};
    virtual void steering (int n) {printf("steerstart\n");hardware.steering(n); Simulator::steering(n);printf("steerend");};
};

#endif
