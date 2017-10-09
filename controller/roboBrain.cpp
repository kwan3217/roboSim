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
    desiredHeading=90;
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
    log.write(desiredHeading,"desiredHeading");
    log.write(heading,"heading");
    log.write(headingChange,"headingChange");
    log.end();
  }
}

void roboBrain::control(){
  int steeringCmd,throttleCmd,mode;
  if(nowpoint == 0) {
    //Haven't started yet
    mode=0;
    throttleCmd=1500;
    steeringCmd=1500;
  } else if(headingChange < 300){
    //In progress
    mode=1;
    throttleCmd=1400;
    servoCommand = (headingChange * 7 * double(50)/180+1500);
    if(servoCommand > 2000) servoCommand = 2000;
    if(servoCommand < 1000) servoCommand = 1000;
    steeringCmd=servoCommand;
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
  log.write(mode,"mode");
  log.write(throttleCmd,"throttleCmd");
  log.write(steeringCmd,"steeringCmd");
  log.end();
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
    log.write(g[0],"g.x");
    log.write(g[1],"g.y");
    log.write(g[2],"g.z");
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
  while(interface.checkNavChar()){
    char ch = interface.readChar();
    //printf("%c",ch);
    if(ch == '$') sentenceStart = true;
    if(!sentenceStart) continue;

    nmeaReceived[charsReceived] = ch; //interface.readChar(); //<--Maybe decomment this later if ch stops being our character tester
    if(nmeaReceived[charsReceived])
      if(nmeaReceived[charsReceived] == ',' || nmeaReceived[charsReceived] == '*'){
	partitions[partCount] = charsReceived;
	partCount += 1;
      }
    charsReceived += 1;
    if(partCount == timeSpot + 1 && charsReceived == partitions[timeSpot] + 1 && strncmp(nmeaReceived, "$GPRMC", 6) != 0){
      nmeaReceived[charsReceived] = '\0';
      log.start(Log::Apids::nmea,"NMEA");
      log.write(nmeaReceived);
      log.end();
      sentenceStart = false;
      charsReceived = 0;
      partCount = 0;
      continue;
    }
    if(partCount == statusSpot + 1 && nmeaReceived[partitions[statusSpot] + 1] == 'V'){
      nmeaReceived[charsReceived] = '\0';
      log.start(Log::Apids::nmea,"NMEA");
      log.write(nmeaReceived);
      log.end();
      sentenceStart = false;	//if status is void, throw away the received data and move forward
      charsReceived = 0;
      partCount = 0;
      continue;
    }
    if(partCount == checksumSpot + 1 && charsReceived == partitions[checksumSpot] + 3){	//there should be just two characters received after the checksum asterisk if sentence is done
      sentenceStart = false;
      char checksum = 0;
      for(int i = 1; i < partitions[checksumSpot]; i++) {
        checksum ^= nmeaReceived[i];
      }
      nmeaReceived[charsReceived] = '\0';
      log.start(Log::Apids::nmea,"NMEA");
      log.write(nmeaReceived);
      log.end();
      if(checksum == strtol(nmeaReceived + partitions[checksumSpot] + 1, NULL, 16)){	//validate checksum
        printf("Parsing RMC data...");
        for(int i = 0; i < charsReceived; i++){
          if(nmeaReceived[i] == ',' || nmeaReceived[i] == '*')
          nmeaReceived[i] = '\0';
        }
        charsReceived = 0;
        partCount = 0;


        //Take data from the desired partitions and use atod function to translate them into numbers I can use
        fp latpos = atof(nmeaReceived + partitions[latSpot] + 1);
        int degrees = floor(latpos)/100;
	fp minutes = (latpos - degrees * 100);
	latdd = degrees + minutes/60;

	if(nmeaReceived[partitions[nsSpot]+1] == 'S') latdd = -latdd;

        fp longpos = atof(nmeaReceived + partitions[longSpot] + 1);
        degrees = floor(longpos)/100;
        minutes = (longpos - degrees * 100);
        longdd = degrees + minutes/60;

        if(nmeaReceived[partitions[ewSpot]+1] == 'W') longdd = -longdd;


        if(lat0 > 90 && long0 > 180){
          lat0 = latdd;
          long0 = longdd;
        } else {
	   //compare new lat, long with original, then give a new easting and northing to pos
           pos.northing() = (latdd - lat0)*re*PI/180;
           pos.easting()  = (longdd - long0)*re*cos(lat0)/180;
        }
        break;
      } else {	//if checksum is invalid, throw away the sentence and keep going
  	charsReceived = 0;
        partCount = 0;
      }
    }
  }
}

void roboBrain::readSensors() {
  //Get timestamp of sensor readings
  ot = t;
  t = interface.time();
  dt = t - ot;
  //Read gyroscope
  gValid=interface.readGyro(g);
  /*
  //read odometer
  oldWheelCount = wheelCount;
  int32_t tempWheelCount;
  odoValid=interface.readOdometer(t_odo, wheelCount, dt_odo);
  if(odoValid) {
    deltaWheelCount = wheelCount - oldWheelCount;
  }
  */
  //Get timestamp after reading sensors
  t1=interface.time();
}

void roboBrain::navigate() {
  navigateGPS();
  navigateOdometer();
  navigateCompass();
}

