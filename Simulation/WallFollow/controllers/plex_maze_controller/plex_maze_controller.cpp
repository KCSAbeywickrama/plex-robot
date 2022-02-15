#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <math.h>
#include <stdio.h>
#include <cstdlib>

#define TIME_STEP 16
#define MAX_SPEED 6
#define p 1
#define d 0.66

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
  
  char dsNames[5][10] = {"ds_left","ds_right","ds_fleft","ds_fright","ds_left1"};


  Motor *left_motor = robot->getMotor("motorL");
  Motor *right_motor = robot->getMotor("motorR");

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
  
  DistanceSensor *ds[5];
  for(int ind=0; ind<5; ++ind){
    ds[ind] = robot->getDistanceSensor(dsNames[ind]);
    ds[ind]->enable(TIME_STEP);
  }


  while (robot->step(TIME_STEP) != -1){
    double left_wall = ds[0]->getValue();
    double right_wall = ds[1]->getValue();
    double rfront_wall = ds[3]->getValue();
    double lfront_wall = ds[2]->getValue();
    double left_wall1 = ds[4]->getValue();

    if (left_wall > 70)
    {
      left_wall = 70;
    }
    

    double perror = left_wall - 12;
    double derror = perror - oerror;
    oerror = perror;

    double error = p*perror - d*derror;

        
    double l = error;
    double r = error;

    if (error > 6){l=6;}
    if (error > 3){r=3;}
    if (error < -3){l=-3;}
    if (error < -6){r=-6;}

    if ( lfront_wall < 8 or rfront_wall < 8){
      // reverse();
      left_motor->setVelocity(-6);
      right_motor->setVelocity(-6);
      robot->step(10);
    }
    
    else if ((lfront_wall < 14 and (rfront_wall < 18 or rfront_wall > 4*lfront_wall) and left_wall < 30) or turnright){
      turnright=1;
      //turnRight(lfront_wall,rfront_wall);
      if (lfront_wall < 27 and (rfront_wall <36 or rfront_wall > 2*lfront_wall) and left_wall < 30){
        left_motor->setVelocity(6);
        right_motor->setVelocity(-3);
      }
      else{
        turnright=0;
      }
    }
    
    else if (derror > 0){
      left_motor->setVelocity(3 - l/6.5);
      right_motor->setVelocity(3 + r/6.5);
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

