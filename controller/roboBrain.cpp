#include <iostream>
#include <cmath>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
//#include "HeaderSimRobo.h"
#include "roboBrain.h"
//#include "Simulator.h" //quick-and-dirty solution to let the cheat compass take information from Simulator class

using namespace std;

const waypoint roboBrain::waypoints[]={
                        {   0.00,   0.00},
                        {- 26.42,  21.83},
                        {- 19.53,  30.55},
                        {   0.29,  14.32},
                        {  11.72,  28.72},
                        {  23.83,  19.39},
                        {   9.70,   2.77},
                        {   6.24,   5.57},
                        {   3.36,   2.49},
                        {   6.91,-  0.11},
                        {   3.93,-  3.28},
                };
const int roboBrain::wpcount=sizeof(roboBrain::waypoints)/sizeof(waypoint);

void roboBrain::guide(){
  if(nowpoint != 0) {
    /*
    fp wp_dot=dot((waypoints[nowpoint]- waypoints[nowpoint - 1]),waypoints[nowpoint] - pos);
    if(wp_dot<0) {
      nowpoint += 1;
    }
    if(nowpoint >= wpcount){
      headingChange=400;
      return;
    }
    desiredHeading = static_cast<waypoint>(waypoints[nowpoint]-pos).heading();

    */
    if(t>20) {
      headingChange=400;
      return;
    }
    desiredHeading=0;
    headingChange = desiredHeading - heading;
    if(headingChange > 180){
      headingChange -= 360;
    } else if (headingChange < -180) {
      headingChange += 360;
    }
    log.start(Log::Apids::guidance,"Guidance");
//    log.write(nowpoint,"nowpoint");
//    log.write(pos.easting(),"pos_e");
//    log.write(pos.northing(),"pos_n");
//    log.write(waypoints[nowpoint].easting(),"wp_e");
//    log.write(waypoints[nowpoint].northing(),"wp_n");
//    log.write(wp_dot,"wp_dot");
    log.write(t,"t");
    log.write(desiredHeading,"desiredHeading");
    log.write(heading,"heading");
    log.write(headingChange,"headingChange");
    log.end();
  }
}

void roboBrain::control(){
  if(nowpoint == 0) {
    //Haven't started yet
    mode=0;
    throttleCmd=1500;
    steeringCmd=1500;
  } else if(headingChange < 300){
    //In progress
    mode=1;
    throttleCmd=1420;
    steeringCmd = (headingChange * 20 * double(50)/180+1500);
    if(steeringCmd > 2000) steeringCmd = 2000;
    if(steeringCmd < 1000) steeringCmd = 1000;
  } else {
    //Finished
    mode=2;
    steeringCmd=1500;
    throttleCmd=1500;
  }
  /*
  interface.steering.write(steeringCmd);
  interface.throttle.write(throttleCmd);
  */
  log.start(Log::Apids::control,"Control");
  log.write(t,"t");
  log.write(mode,"mode");
  log.write(throttleCmd,"throttleCmd");
  log.write(steeringCmd,"steeringCmd");
  log.end();
  interface.steerBoth(steeringCmd,throttleCmd);
}

void roboBrain::setOffSet(){
  int n=0;
  for(int i = 0; i < bufferMax; i++){
    if(i > bufferDiscard) {
      log.start(Log::Apids::calcOffset,"calcOffset");
      log.write(bufferSpot,"bufferSpot");
      log.write(offSet[0],"offSetBefore[0]");
      log.write(offSet[1],"offSetBefore[1]");
      log.write(offSet[2],"offSetBefore[2]");
      log.write(n,"nBefore");
      log.write(ofBuffer[bufferSpot][0],"ofBuffer[0]");
      log.write(ofBuffer[bufferSpot][1],"ofBuffer[1]");
      log.write(ofBuffer[bufferSpot][2],"ofBuffer[2]");
      offSet[0] += ofBuffer[bufferSpot][0];
      offSet[1] += ofBuffer[bufferSpot][1];
      offSet[2] += ofBuffer[bufferSpot][2];
      n++;
      log.write(offSet[0],"offSetAfter[0]");
      log.write(offSet[1],"offSetAfter[1]");
      log.write(offSet[2],"offSetAfter[2]");
      log.write(n,"nAfter");
      log.end();
    }
    bufferSpot--;
    if(bufferSpot < 0) bufferSpot = bufferMax;
  }
  log.start(Log::Apids::setOffSet,"setOffSet");
  log.write(bufferMax,"bufferMax");
  log.write(bufferDiscard,"bufferDiscard");
  log.write(n,"n");
  log.write(offSet[0],"offSetBefore.x");
  log.write(offSet[1],"offSetBefore.y");
  log.write(offSet[2],"offSetBefore.z");
  offSet[0] /= n;
  offSet[1] /= n;
  offSet[2] /= n;
  log.write(offSet[0],"offSetAfter.x");
  log.write(offSet[1],"offSetAfter.y");
  log.write(offSet[2],"offSetAfter.z");
  log.end();
}

bool roboBrain::navigateCompass(){
  if(nowpoint == 0){
    fillBuffer();
    if(interface.button()){
      nowpoint = 1;
      pos = {0,0};
      setOffSet();
    }
    return false;
  } else {
    Vector<3> omega(
      //Convert from DN, through deg/s
      //(with assumed sensitivity of +/-250deg/s FS)
      //to rad/s
      (fp(g[0])-offSet[0])/0x7FFF * 250 * PI/180,
      (fp(g[1])-offSet[1])/0x7FFF * 250 * PI/180,
      (fp(g[2])-offSet[2])/0x7FFF * 250 * PI/180
    );
    q.integrate(omega,dt);              //Update the quaternion based on this gyro reading
    Quaternion nose(1,0,0);
    nose=q.b2r(nose);  //Convert the nose vector from body to world coordinates
    log.start(Log::Apids::quaternion,"quaternion");
    log.write(t,"t");
    log.write(dt,"dt");
    log.write(fp(omega[0]),"omega.x");
    log.write(fp(omega[1]),"omega.y");
    log.write(fp(omega[2]),"omega.z");
    log.write(q.x(),"q.x");
    log.write(q.y(),"q.y");
    log.write(q.z(),"q.z");
    log.write(q.w(),"q.w");
    log.write(nose.x(),"nose.x");
    log.write(nose.y(),"nose.y");
    log.write(nose.z(),"nose.z");
    log.write(nose.w(),"nose.w");
    log.end();
    log.start(Log::Apids::compass,"compass");
    log.write(t,"t");
    log.write(nose.x(),"nose.x");
    log.write(nose.y(),"nose.y");
    heading=atan2(-nose.y(),nose.x())*180/PI;
    if(heading<0) heading+=360;
    if(heading>360) heading-=360;
    log.write(heading,"heading");
    log.end();
  }
  return true;
}

void roboBrain::fillBuffer(){
  interface.readGyro(g);
  ofBuffer[bufferSpot][0] = g[0];
  ofBuffer[bufferSpot][1] = g[1];
  ofBuffer[bufferSpot][2] = g[2];
  log.start(Log::Apids::averageG,"averageG");
  log.write(bufferSpot,"bufferSpot");
  log.write(ofBuffer[bufferSpot][0],"ofBuffer[0]");
  log.write(ofBuffer[bufferSpot][1],"ofBuffer[1]");
  log.write(ofBuffer[bufferSpot][2],"ofBuffer[2]");
  log.end();
  bufferSpot++;
  if(bufferSpot >= 1500) {
    bufferSpot = 0;
    //If button is enabled, don't use the following code
    nowpoint = 1;
    pos = {0,0};
    setOffSet();
  }
}

bool roboBrain::navigateOdometer(){
  odoDeltaPos={sin(heading*PI/180),cos(heading*PI/180)};
  pos+=odoDeltaPos*fp(fp(tickDistance * deltaWheelCount)/4.0);
  return true;
}

void roboBrain::navigateGPS(){

}

void roboBrain::readSensors() {
  //Get timestamp of sensor readings
  ot = t;
  t = interface.time();
  dt = t - ot;
  //Read gyroscope
  gValid=interface.readGyro(g);
  //Read GPS
  if(interface.checkPPS(pps)) {
    log.start(Log::Apids::pps,"PPS");
    log.write(pps,"pps");
    log.end();
    hasFixForPPS=false;
    processedFixForPPS=false;
  }
  if(interface.readGPS(t_gps_valid,lat,lon)) {
    t_gps_collected=t;
    log.start(Log::Apids::gpsd,"GPS");
    log.write(t_gps_collected,"t_gps_collected");
    log.write(t_gps_valid,"t_gps_valid");
    log.write(lat,"lat");
    log.write(lon,"lon");
    hasFixForPPS=true;
  }
  /*
  //read odometer - should be done after GPS, or else there will be an unnecessary 1-cycle latency
  oldWheelCount = wheelCount;
  int32_t tempWheelCount;
  odoValid=interface.readOdometer(t_odo, wheelCount, dt_odo);
  if(odoValid) {
    deltaWheelCount = wheelCount - oldWheelCount;
  }
  */
  //Get timestamp after reading sensors
  t1=interface.time();
  log.start(Log::Apids::gyro,"gyro");
  log.write(t,"t");
  log.write(dt,"dt");
  log.write(g[0],"gx");
  log.write(g[0],"gy");
  log.write(g[0],"gz");
  log.write(t1,"t1");
  log.end();
}

void roboBrain::navigate() {
//  navigateGPS();
//  navigateOdometer();
  navigateCompass();
}

