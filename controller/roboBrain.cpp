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
    fp wp_dot=dot((waypoints[nowpoint]- waypoints[nowpoint - 1]),waypoints[nowpoint] - pos);
    if(wp_dot<0) {
      nowpoint += 1;
    }
    if(nowpoint >= wpcount){
      headingChange=400;
      return;
    }
    desiredHeading = static_cast<waypoint>(waypoints[nowpoint]-pos).heading();

    headingChange = desiredHeading - heading;
    if(headingChange > 180){
      headingChange -= 360;
    } else if (headingChange < -180) {
      headingChange += 360;
    }
    log.start(Log::Apids::guidance,"Guidance");
    log.write(nowpoint,"nowpoint");
    log.write(pos.easting(),"pos_e");
    log.write(pos.northing(),"pos_n");
    log.write(waypoints[nowpoint].easting(),"wp_e");
    log.write(waypoints[nowpoint].northing(),"wp_n");
    log.write(wp_dot,"wp_dot");
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
    throttleCmd=150;
    steeringCmd=155;
    interface.throttle.write(150);
    interface.steering.write(155);
  } else if(headingChange < 300){
    //In progress
    mode=1;
    throttleCmd=140;
    interface.throttle.write(140);
    servoCommand = (headingChange * 7 * double(50)/180+155);
    if(servoCommand > 200) servoCommand = 200;
    if(servoCommand < 100) servoCommand = 100;
    steeringCmd=servoCommand;
  } else {
    //Finished
    mode=2;
    interface.throttle.write(150);
    interface.steering.write(150);
  }
  interface.steering.write(steeringCmd);
  interface.throttle.write(throttleCmd);
  log.start(Log::Apids::control,"Control");
  log.write(mode,"mode");
  log.write(throttleCmd,"throttleCmd");
  log.write(steeringCmd,"steeringCmd");
  log.end();
}

void roboBrain::setOffSet(){
  for(int i = 0; i < bufferMax; i++){
    if(i > bufferDiscard) offSet += ofBuffer[bufferSpot];
    bufferSpot--;
    if(bufferSpot < 0) bufferSpot = bufferMax;
  }
  offSet /= (bufferMax - bufferDiscard);
  log.start(Log::Apids::setOffSet,"setOffSet");
  log.write(offSet,"offSet");
  log.write(bufferMax,"bufferMax");
  log.write(bufferDiscard,"bufferDiscard");
  log.end();
}

void roboBrain::navigateCompass(){
  updateTime();
  int16_t g[3];
  interface.readGyro(g);
  zDN=g[2];
  yawRate = fp(g[2] - offSet)/ 0x7FFF * 250;
  heading -= yawRate * dt;
  log.start(Log::Apids::compass,"compass");
  log.write(dt,"dt");
  log.write(zDN,"zDN");
  log.write(yawRate,"yawRate");
  log.write(heading,"heading");
  log.end();
}

void roboBrain::updateTime(){
  double oldTime = epochTime;
  epochTime = interface.time();
  dt = epochTime - oldTime;
}

void roboBrain::fillBuffer(){
  int16_t g[3];
  interface.readGyro(g);
  ofBuffer[bufferSpot] = g[2];
  zDN=g[2];
  log.start(Log::Apids::averageG,"averageG");
  log.write(ofBuffer[bufferSpot],"ofBuffer");
  log.write(bufferSpot,"bufferSpot");
  bufferSpot++;
  if(bufferSpot >= 1500) bufferSpot = 0;
}

void roboBrain::navigateOdometer(){
  oldWheelCount = wheelCount;
  int32_t tempWheelCount;
  interface.readOdometer(timeStamp, tempWheelCount, dtOdometer);
  int32_t deltaWheelCount = tempWheelCount - oldWheelCount;
  if (deltaWheelCount>4 || deltaWheelCount<-4) return;
  wheelCount=tempWheelCount;
  odoDeltaPos={sin(heading*PI/180),cos(heading*PI/180)};
  pos+=odoDeltaPos*fp(fp(tickDistance * deltaWheelCount)/4.0);
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

void roboBrain::navigate() {
  updateTime();
  navigateGPS();
  navigateOdometer();
  if(nowpoint == 0){
    fillBuffer();
    if(interface.button()){
      nowpoint = 1;
      pos = {0,0};
      setOffSet();
    }
  } else {
    navigateCompass();
  }
}

void roboBrain::showVector(Log& pkt) {
//  pkt.write(latdd,"lat from gps");
//  pkt.write(longdd,"lon from gps");
  pkt.write(interface.button(), "button status");
  pkt.write(zDN,"zDN");
  pkt.write(offSet, "offset");
  pkt.write(ofBuffer[bufferSpot], "current buffer's value");
  pkt.write(bufferSpot, "buffer spot");
//  pkt.write(wheelCount,"wheelCount");
//  pkt.write(pos.easting(),"est easting");
//  pkt.write(pos.northing(),"est northing");
//  pkt.write(nowpoint,"nowpoint");
//  pkt.write(waypoints[nowpoint].easting(),"waypoint easting");
// pkt.write(waypoints[nowpoint].northing(),"waypoint northing");
  pkt.write(yawRate, "yaw rate");
  pkt.write(heading, "est heading");
  pkt.write(desiredHeading,"desired heading");
  pkt.write(headingChange,"heading change");
  pkt.write(servoCommand, "steering command");
}
