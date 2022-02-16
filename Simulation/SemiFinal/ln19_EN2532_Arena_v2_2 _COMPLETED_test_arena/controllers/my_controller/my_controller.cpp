#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <math.h>
#include <stdio.h>
#include <cstdlib>

#define TIME_STEP 16
#define MAX_SPEED 6
#define BASE_SPEED 3
#define p 0.8
#define d 0.1

using namespace webots;
using namespace std;

Robot *robot = new Robot();

int turnright;
double oerror;

int main(int argc, char **argv) {
  
  char dsNames[3][10] = {"ds_left","ds_fleft","ds_fright"};


  Motor *left_motor = robot->getMotor("leftMotor");
  Motor *right_motor = robot->getMotor("rightMotor");

  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);

  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);

  
  DistanceSensor *ds[3];
  for(int ind=0; ind<3; ++ind){
    ds[ind] = robot->getDistanceSensor(dsNames[ind]);
    ds[ind]->enable(TIME_STEP);
  }


  while (robot->step(TIME_STEP) != -1){
    double leftDS = ds[0]->getValue();
    double frontLeftDS = ds[1]->getValue();
    double frontRightDS = ds[2]->getValue();

    if (leftDS > 50)
    {
      leftDS = 50;
    }
    

    double perror = leftDS - 16;
    double derror = perror - oerror;
    oerror = perror;

    double error = p*perror - d*derror;

    double l = error;
    double r = error;

    if (error > 6){l=6;}
    if (error > 3){r=3;}
    if (error < -3){l=-3;}
    if (error < -6){r=-6;}

    if ( frontLeftDS < 13 or frontRightDS < 13){
      left_motor->setVelocity(-MAX_SPEED);
      right_motor->setVelocity(-MAX_SPEED);
      robot->step(5);
    }
    
    else if ((frontLeftDS < 18 and (frontRightDS < 22 or frontRightDS > 4*frontLeftDS) and leftDS < 38) or turnright){
      turnright=1;
      if (frontLeftDS < 33 and (frontRightDS <44 or frontRightDS > 2*frontLeftDS) and leftDS < 36){
        left_motor->setVelocity(5);
        right_motor->setVelocity(-1);
      }
      else{
        turnright=0;
      }
    }
    
    else if (derror == 0 and leftDS > 40){
      left_motor->setVelocity(BASE_SPEED - l/5);
      right_motor->setVelocity(BASE_SPEED + r/5);
    }
    else if (derror > 0){
      left_motor->setVelocity(BASE_SPEED - l/6);
      right_motor->setVelocity(BASE_SPEED + r/6);
    }
    
    else {
      left_motor->setVelocity(BASE_SPEED - l);
      right_motor->setVelocity(BASE_SPEED + r);
    }
    cout<<leftDS<<"  "<<frontLeftDS<<"   "<<frontRightDS<<endl;

  };

  delete robot;
  return 0;
}

