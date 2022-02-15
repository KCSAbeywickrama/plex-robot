
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>
#include "arm.hpp"

#define TIME_STEP 16
using namespace webots;
using namespace std;
namespace arm
{
  Robot *robot;
  Motor *handleMotor;
  Motor *handleEncoder;
  Motor *leftMotor;
  Motor *rightMotor;
  Motor *leftSlider;
  Motor *rightSlider;
  PositionSensor *leftSliderEncoder;
  PositionSensor *rightSliderEncoder;
  TouchSensor *leftTouch;
  TouchSensor *rightTouch;

  void gripObject(float ps, bool &objTouch)
  {
     while (robot->step(TIME_STEP) != -1 && objTouch)
    {
      //cout << leftSliderEncoder->getValue() << endl;
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
          //cout << " done" << endl;
          break;
        }
        else
        {
          //cout << "turn" << endl;
          leftSlider->setPosition(ps - 0.01);
          rightSlider->setPosition(ps - 0.01);
          leftMotor->setVelocity(0.1);
          rightMotor->setVelocity(0.025);
        }
      }
      ps += 0.001;
      if (leftSliderEncoder->getValue() >= 0.04)
      {
        ps = 0;
      }
    }
  }
  
  
  void init()
  {
    
    robot = new Robot();
    
    handleMotor = robot->getMotor("handleMotor");
    handleEncoder = robot->getMotor("handleEncoder");
    
    leftMotor = robot->getMotor("leftMotor");
    rightMotor = robot->getMotor("rightMotor");
    
    leftSlider = robot->getMotor("leftSlider");
    rightSlider = robot->getMotor("rightSlider");
    
    leftSliderEncoder = robot->getPositionSensor("leftSliderEncoder");
    rightSliderEncoder = robot->getPositionSensor("rightSliderEncoder");

    leftTouch = robot->getTouchSensor("leftTouchSensor");
    rightTouch = robot->getTouchSensor("rightTouchSensor");


    handleMotor->setPosition(INFINITY);
    handleMotor->setVelocity(0.0);

    leftMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);

    rightMotor->setPosition(INFINITY);
    rightMotor->setVelocity(0.0);
  }
}   
    

    
    
    

     

