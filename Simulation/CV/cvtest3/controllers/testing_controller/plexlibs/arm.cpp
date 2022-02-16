
#include "motors.hpp"
#include "arm.hpp"

namespace arm
{
  Motor *handleMotor;
  Motor *handleEncoder;
  Motor *leftSlider;
  Motor *rightSlider;
  PositionSensor *leftSliderEncoder;
  PositionSensor *rightSliderEncoder;
  TouchSensor *leftTouch;
  TouchSensor *rightTouch;

  void gripObject(Robot *robot, float ps, bool &objTouch)
  {
    cout << "gripping" << endl;
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
          // cout << " done" << endl;
          break;
        }
        else
        {
          // cout << "turn" << endl;
          leftSlider->setPosition(ps - 0.01);
          rightSlider->setPosition(ps - 0.01);
          motors::setLeftSpeed(0.1);
          motors::setRightSpeed(0.025);
        }
      }
      ps += 0.001;
      if (leftSliderEncoder->getValue() >= 0.04)
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

    cout << "arm init" << endl;
  }
}
