// File:          arm.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#define TIME_STEP 64
// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
vector<string> motornames = {"leftSlider", "rightSlider", "handleMotor"};
vector<string> snames = {"leftTouch", "rightTouch"};

/*void gripObject(float ps,bool &objTouch){
    cout << ps<< endl;
    leftTouch->enable(TIME_STEP);
    rightTouch->enable(TIME_STEP);
    handleMotor->setVelocity(1.57);
    handleMotor->setPosition(0);
    leftSlider->setPosition(ps);
    rightSlider->setPosition(ps);
    if(leftTouch->getValue() && rightTouch->getValue()){
      if (ps>=0.04){
        objTouch=false;
        cout << ps <<" done"<< endl;
        objTouch=true;
      }
      else{
        leftMotor->setVelocity(0.01);
      }
    }
    ps+=0.0001;
    if (ps >0.045){
      ps=0;
    }
  }*/

int main(int argc, char **argv)
{
  // create the Robot instance.
  Robot *robot = new Robot();
  Motor *handleMotor = robot->getMotor("handleMotor");
  Motor *handleEncoder = robot->getMotor("handleEncoder");
  Motor *leftMotor = robot->getMotor("leftMotor");
  Motor *rightMotor = robot->getMotor("rightMotor");

  handleMotor->setPosition(INFINITY);
  handleMotor->setVelocity(0.0);

  leftMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);

  rightMotor->setPosition(INFINITY);
  rightMotor->setVelocity(0.0);

  Motor *leftSlider = robot->getMotor("leftSlider");
  Motor *rightSlider = robot->getMotor("rightSlider");

  PositionSensor *leftSliderEncoder = robot->getPositionSensor("leftSliderEncoder");
  PositionSensor *rightSliderEncoder = robot->getPositionSensor("rightSliderEncoder");

  TouchSensor *leftTouch = robot->getTouchSensor("leftTouchSensor");
  TouchSensor *rightTouch = robot->getTouchSensor("rightTouchSensor");

  // rightArmMotor->setPosition(INFINITY);
  // rightArmMotor->setVelocity(0.0);
  // Motor *rightMotor = robot->getMotor("motorRight");
  //  get the time step of the current world.
  // int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller

  bool objTouch = true;
  float ps = 0.005;
  while (robot->step(TIME_STEP) != -1)
  {
    // leftSlider->setVelocity(2.0);
    // rightSlider->setVelocity(2.0);
    // handleEncoder->setPosition(1.57);

    while (robot->step(TIME_STEP) != -1 && objTouch)
    {
      cout << leftSliderEncoder->getValue() << endl;
      leftTouch->enable(TIME_STEP);
      rightTouch->enable(TIME_STEP);
      leftSliderEncoder->enable(TIME_STEP);
      rightSliderEncoder->enable(TIME_STEP);
      handleMotor->setVelocity(1.57);
      handleMotor->setPosition(0);
      leftSlider->setPosition(ps);
      rightSlider->setPosition(ps);
      if (leftTouch->getValue() && rightTouch->getValue())
      {
        if (leftSliderEncoder->getValue() >= 0.035)
        {
          objTouch = false;
          cout << ps << " done" << endl;
          break;
        }
        else
        {
          cout << "turn" << endl;
          leftSlider->setPosition(ps - 0.001);
          rightSlider->setPosition(ps - 0.001);

          // for (int i = 0; i < 100; i++)
          // {
          //   robot->step(TIME_STEP);
          // }
          cout << "delay end" << endl;
          leftMotor->setVelocity(0.1);
          rightMotor->setVelocity(0.025);
          // for (int i = 0; i < 100; i++)
          // {
          //   robot->step(TIME_STEP);
          // }
        }
      }
      ps += 0.001;
      if (leftSliderEncoder->getValue() >= 0.04)
      {
        ps = 0;
      }
    }

    // cout << "out of loop"<< endl;
    //  Read the sensors:
    //  Enter here functions to read sensor data, like:
    //   double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
