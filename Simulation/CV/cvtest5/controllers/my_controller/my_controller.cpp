// File:          my_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
    handleMotor = robot->getMotor("handleMotor");
    handleEncoder = robot->getMotor("handleEncoder");

    leftSlider = robot->getMotor("leftSlider");
    rightSlider = robot->getMotor("rightSlider");

    leftSliderEncoder = robot->getPositionSensor("leftSliderEncoder");
    rightSliderEncoder = robot->getPositionSensor("rightSliderEncoder");
    
    leftEncoder = robot->getPositionSensor("leftEncoder");
    rightEncoder = robot->getPositionSensor("rightEncoder");

    leftTouch = robot->getTouchSensor("leftTouchSensor");
    rightTouch = robot->getTouchSensor("rightTouchSensor");

    handleMotor->setPosition(INFINITY);
    handleMotor->setVelocity(0.0);

    leftMotor = robot->getMotor("leftMotor");
    rightMotor = robot->getMotor("rightMotor");

    leftMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);

    rightMotor->setPosition(INFINITY);
    rightMotor->setVelocity(0.0);
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
