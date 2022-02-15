#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <math.h>
#include <stdio.h>
#include <cstdlib>

#define TIME_STEP 16
#define MAX_SPEED 6
#define p 0.8
#define d 0.1

using namespace webots;
using namespace std;

Robot *robot = new Robot();

//void reverse();
// void turnRight(double lf, double rf);
// void lwallFollow();
int turnright;
double oerror;
//double lf,rf,le,re;

int main(int argc, char **argv) {
  
  char dsNames[3][10] = {"ds_left","ds_fleft","ds_fright"};


  Motor *left_motor = robot->getMotor("leftMotor");
  Motor *right_motor = robot->getMotor("rightMotor");

  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);

  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);

  // void reverse();{
  //   left_motor->setVelocity(-6);
  //   right_motor->setVelocity(-6);
  //   robot->step(10);
  // }

  // void turnRight(double lf, double rf);{
  //   if (lf < 20 and rf <26){
  //     left_motor->setVelocity(5);
  //     right_motor->setVelocity(-2);
  //   }
  //   else{
  //     turnright=0;
  //   }
  // }

  // void lwallFollow(double le, double re);{
  //   left_motor->setVelocity(3 - le);
  //   right_motor->setVelocity(3 + re);
  // }
  
  DistanceSensor *ds[3];
  for(int ind=0; ind<3; ++ind){
    ds[ind] = robot->getDistanceSensor(dsNames[ind]);
    ds[ind]->enable(TIME_STEP);
  }


  while (robot->step(TIME_STEP) != -1){
    double left_wall = ds[0]->getValue();
    // double right_wall = ds[1]->getValue();
    double rfront_wall = ds[2]->getValue();
    double lfront_wall = ds[1]->getValue();
    // double left_wall1 = ds[4]->getValue();

    if (left_wall > 50)
    {
      left_wall = 50;
    }
    

    double perror = left_wall - 16;
    double derror = perror - oerror;
    oerror = perror;

    double error = p*perror - d*derror;

        
    double l = error;
    double r = error;

    if (error > 6){l=6;}
    if (error > 3){r=3;}
    if (error < -3){l=-3;}
    if (error < -6){r=-6;}

    if ( lfront_wall < 13 or rfront_wall < 13){
      // reverse();
      left_motor->setVelocity(-6);
      right_motor->setVelocity(-6);
      robot->step(5);
    }
    
    else if ((lfront_wall < 18 and (rfront_wall < 22 or rfront_wall > 4*lfront_wall) and left_wall < 38) or turnright){
      turnright=1;
      //turnRight(lfront_wall,rfront_wall);
      if (lfront_wall < 33 and (rfront_wall <44 or rfront_wall > 2*lfront_wall) and left_wall < 36){
        left_motor->setVelocity(5);
        right_motor->setVelocity(-1);
      }
      else{
        turnright=0;
      }
    }
    
    else if (derror == 0 and left_wall > 40){
      left_motor->setVelocity(3 - l/5);
      right_motor->setVelocity(3 + r/5);
    }
    else if (derror > 0){
      left_motor->setVelocity(3 - l/6);
      right_motor->setVelocity(3 + r/6);
    }
    
    else {
      //lwallFollow(l,r);
      left_motor->setVelocity(3 - l);
      right_motor->setVelocity(3 + r);
    }
    cout<<left_wall<<"  "<<lfront_wall<<"   "<<rfront_wall<<endl;

  };

  delete robot;
  return 0;
}

