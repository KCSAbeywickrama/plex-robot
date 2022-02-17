#include "motors.hpp"
#include "arm.hpp"

namespace arm
{
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

  void gripObject(Robot *robot, float ps, int obj)
  {
    float value;
    if (obj==3){value=0.045;}
    else if (obj == 1 ){value=0.04;}
    cout << "gripping object "<<value << endl;
    while (robot->step(TIME_STEP) != -1)
    {
      
      leftTouch->enable(TIME_STEP);
      rightTouch->enable(TIME_STEP);

      leftSliderEncoder->enable(TIME_STEP);
      rightSliderEncoder->enable(TIME_STEP);

      leftMotor->setVelocity(0.0);
      rightMotor->setVelocity(0.0);

      handleMotor->setVelocity(1.57);
      handleMotor->setPosition(1.57);

      cout << leftSliderEncoder->getValue() << endl;
      for (int i = 0; i < 10; i++)
         {
            robot->step(TIME_STEP);
         }
      
      leftSlider->setPosition(ps);
      rightSlider->setPosition(ps);
      if (leftTouch->getValue() && rightTouch->getValue())
      {
        if (leftSliderEncoder->getValue() >= value)
        {
          leftMotor->setVelocity(0.0);
          rightMotor->setVelocity(0.0);
          handleMotor->setVelocity(0.5);
          handleMotor->setPosition(1.0);
          for (int i = 0; i < 10; i++)
         {
            robot->step(TIME_STEP);
         }
          cout << " done gripping" << endl;
          return;
        }
        else
        {
          cout << "turn" << endl;
          leftSlider->setPosition(ps - 0.01);
          rightSlider->setPosition(ps - 0.01);
          leftMotor->setVelocity(2);
          rightMotor->setVelocity(0.025);
           for (int i = 0; i < 10; i++)
           {
             robot->step(TIME_STEP);
           }

        }
      }
      ps += 0.001;
      if (leftSliderEncoder->getValue() >= 0.05)
      {
        ps = 0;
      }
    }
  }

  void init(Robot *robot)
  {
    handleMotor = robot->getMotor("handleMotor");
    handleEncoder = robot->getMotor("handleEncoder");

    leftSlider = robot->getMotor("leftSlider");
    rightSlider = robot->getMotor("rightSlider");

    leftSliderEncoder = robot->getPositionSensor("leftSliderEncoder");
    rightSliderEncoder = robot->getPositionSensor("rightSliderEncoder");

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

    

    cout << "arm init" << endl;
  }
  
}   
    

    
    
    

     

