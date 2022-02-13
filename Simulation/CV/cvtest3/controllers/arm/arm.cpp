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
#define TIME_STEP 64
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
  Motor *handleMotor = robot->getMotor("handleMotor");
  Motor *handleEncoder = robot->getMotor("handleEncoder");
  handleMotor->setPosition(INFINITY);
  handleMotor->setVelocity(0.0);
  
  Motor *leftSlider = robot->getMotor("leftSlider");
  Motor *rightSlider = robot->getMotor("rightSlider");
  //rightArmMotor->setPosition(INFINITY);
  //rightArmMotor->setVelocity(0.0);
  //Motor *rightMotor = robot->getMotor("motorRight");
  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
 float ps=0.002;
  while (robot->step(TIME_STEP) != -1) {
  //leftSlider->setVelocity(2.0);
  //rightSlider->setVelocity(2.0);
  //handleEncoder->setPosition(1.57);
  handleMotor->setVelocity(1.57);
  leftSlider->setPosition(ps);
  rightSlider->setPosition(ps);
  ps+=0.0002;
  if (ps >=0.045){
  ps=0;
  }
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
