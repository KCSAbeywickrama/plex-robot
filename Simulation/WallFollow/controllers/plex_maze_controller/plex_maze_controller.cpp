// File:          plex_maze_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <math.h>
#include <stdio.h>

#define TIME_STEP 30
#define MAX_SPEED 5.28
#define PI 3.141592654
#define kW 1
#define a 12
#define b 7
#define e 18

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;

void delayfunction();
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

Robot *robot = new Robot();


void delay_function(float t){
float current_time_1 = float(robot->getTime());
float current_time_2= float(robot->getTime());
do {
current_time_2 = float(robot->getTime());
robot->step(1);
} while(current_time_2 < (current_time_1 + t));
}

int main(int argc, char **argv) {
  // create the Robot in"stance.
  char dsNames[5][10] = {"ds_left","ds_right","ds_fleft","ds_fright","ds_left1"};


  Motor *left_motor = robot->getMotor("motorL");
  Motor *right_motor = robot->getMotor("motorR");

  left_motor->setPosition(INFINITY);
  right_motor->setPosition(INFINITY);

  left_motor->setVelocity(0.0);
  right_motor->setVelocity(0.0);

  double oerror;
  double errorsum;
  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);
  DistanceSensor *ds[5];
  for(int ind=0; ind<5; ++ind)
  {
    ds[ind] = robot->getDistanceSensor(dsNames[ind]);
    ds[ind]->enable(TIME_STEP);
  }

  // double left_speed = MAX_SPEED;
  // double right_speed = MAX_SPEED;

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  while (robot->step(TIME_STEP) != -1){
    // left_motor->setVelocity(06.0);
    // right_motor->setVelocity(06.0);
    // double left_speed = MAX_SPEED;
    // double right_speed = MAX_SPEED;
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();
    //int wall_deted = 0;
    double left_wall = ds[0]->getValue();
    // double right_wall = ds[1]->getValue();
    // double rfront_wall = ds[3]->getValue();
    double lfront_wall = ds[2]->getValue();
    double left_wall1 = ds[4]->getValue();

    int f =0;

    // double thetaR = atan((left_wall-left_wall1)/a) * 180 / PI ;
    // float dR = (left_wall+left_wall1)/2;
    // double alfa = thetaR +  kW*(12-dR);
    // // double vL = MAX_SPEED*(cos(alfa * PI / 180.0)+(b*sin(alfa* PI / 180.0)/e));
    // // double vR = MAX_SPEED*(cos(alfa * PI / 180.0)-(b*sin(alfa * PI / 180.0)/e));
    // double t = (0.04*PI)/MAX_SPEED;
    if(errorsum>2){errorsum=0;}

    double perror = left_wall-11.5;
    double derror = perror-oerror;
    oerror = perror;
    errorsum = errorsum + perror;
    double ierror= 0.5*errorsum +perror;

    double error = perror - 0.6*derror;

    //bool lwall = ((left_wall>24) && (left_wall1 > 24) );

    // Process sensor data here.
    // left_motor->setVelocity(10);
    // right_motor->setVelocity(-10);
    // if (left_wall > 23){
    //   left_motor->setVelocity(-MAX_SPEED);
    //   right_motor->setVelocity(MAX_SPEED);
    // }
    // else{
    //   if (rfront_wall > 11){
    //   left_motor->setVelocity(MAX_SPEED);
    //   right_motor->setVelocity(MAX_SPEED);
    //   }
    //   else{
    //   left_motor->setVelocity(MAX_SPEED);
    //   right_motor->setVelocity(-MAX_SPEED);
    //   }

    // //   /*if (left_corner == true){
    // //     left_speed = MAX_SPEED;
    // //     right_speed = MAX_SPEED/8;
    // //   }
    // //   */
    // }

    // if (left_wall < 9){
    //   left_motor->setVelocity(MAX_SPEED);
    //   right_motor->setVelocity(MAX_SPEED/8);
    // }

    // else if (right_wall < 9)
    // {
    //   left_motor->setVelocity(MAX_SPEED/8);
    //   right_motor->setVelocity(MAX_SPEED);
    // }

    // if (wall_deted == 0){
    //   if (rfront_wall > 11){
    //     left_motor->setVelocity(MAX_SPEED);
    //     right_motor->setVelocity(MAX_SPEED);
    //   }
    //   else{
    //     wall_deted = 1;
    //     left_motor->setVelocity(MAX_SPEED);
    //     right_motor->setVelocity(0.5*MAX_SPEED);
    //   }
    // }
    // else if (rfront_wall > 11)
    // {
    //   int ls = MAX_SPEED - 0.1*((left_wall-11)+0.2*(oleft_wall-11));
    //   int rs = MAX_SPEED + 0.1*((left_wall-11)+0.2*(oleft_wall-11));
    //   left_motor->setVelocity(ls);
    //   right_motor->setVelocity(rs);
    // }
    // else if (rfront_wall < 11)
    // {
    //   left_motor->setVelocity(MAX_SPEED);
    //   right_motor->setVelocity(-MAX_SPEED);
    // }
    

    // float oleft_wall = left_wall;
    // float oright_wall = right_wall;
    // float orfront_wall = rfront_wall;
    // float olfront_wall = lfront_wall;
    // if (left_wall > 23 && left_wall1 > 23 && rfront_wall <= 11){
    //   left_motor->setVelocity(-MAX_SPEED);
    //   right_motor->setVelocity(MAX_SPEED);
    //   delay_function(0.7);
    //   left_motor->setVelocity(MAX_SPEED);
    //   right_motor->setVelocity(MAX_SPEED);
    //   delay_function(1.5);
    // }
    // else{
    //   if (rfront_wall > 11){
    //     left_motor->setVelocity(MAX_SPEED);
    //     right_motor->setVelocity(MAX_SPEED);
    //   }
    //   else{
    //     left_motor->setVelocity(MAX_SPEED);
    //     right_motor->setVelocity(-MAX_SPEED);
    //     delay_function(1);
    //   }

    //   /*if (left_corner == true){
    //     left_speed = MAX_SPEED;
    //     right_speed = MAX_SPEED/8;
    //   }
    //   */
    // if (lwall){
    //   delay_function(0.88*2);
    //   left_motor->setVelocity(-2.5);
    //   right_motor->setVelocity(2.5);
    //   delay_function(1.5708);
    //   left_motor->setVelocity(2.5);
    //   right_motor->setVelocity(2.5);
    //   while ((ds[4]->getValue())>23){
    //     robot->step(1);
    //   }
    //   f=1;
    // }
    // double l = error;
    // double r = error;

    // if (error > 10){l=10;}
    // if (error > 2){r=2;}
    // if (error < -2){l=-2;}
    // if (error < -10){r=-10;}
    // // double vL = 4 -
    // if(f ==0){ 
    //   if(abs(left_wall-left_wall1)>35){
    //     left_motor->setVelocity(2.5);
    //     right_motor->setVelocity(2.5);
    //   }
    //   else{
    //     left_motor->setVelocity(2.5 - l);
    //     right_motor->setVelocity(2.5 + r);
    //   }
    // }
    double l = error;
    double r = error;

    if (error > 6){l=6;}
    if (error > 3){r=3;}
    if (error < -3){l=-3;}
    if (error < -6){r=-6;}
    
    if(lfront_wall < 15){
      left_motor->setVelocity(3);
      right_motor->setVelocity(-3);
      robot->step(5);
    }
    else{
      if (derror > 0){
        left_motor->setVelocity(3-l/6);
        right_motor->setVelocity(3 + r/6);
      }
      else{
        left_motor->setVelocity(3 - l);
        right_motor->setVelocity(3 + r);
      }
    }
    // if(f ==0){ 
    //   if(abs(left_wall-left_wall1)>35){
    //     left_motor->setVelocity(2.5);
    //     right_motor->setVelocity(2.5);
    //   }
    //   else{
    //     left_motor->setVelocity(2.5 - l);
    //     right_motor->setVelocity(2.5 + r);
    //   }
    // }
    // if (left_wall < 9){
    //   left_motor->setVelocity(MAX_SPEED);
    //   right_motor->setVelocity(MAX_SPEED/8);
    // }

    // else if (right_wall < 9)
    // {
    //   left_motor->setVelocity(MAX_SPEED/8);
    //   right_motor->setVelocity(MAX_SPEED);
    // }

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

