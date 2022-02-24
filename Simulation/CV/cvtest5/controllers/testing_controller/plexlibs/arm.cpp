#include "arm.hpp"
#include "mosaic.hpp"
namespace arm
{
  Motor *handleMotor;
  Motor *handleEncoder;
  Motor *leftMotor;
  Motor *rightMotor;
  Motor *leftSlider;
  Motor *rightSlider;
  Motor *shooter;
  PositionSensor *leftSliderEncoder;
  PositionSensor *rightSliderEncoder;
  TouchSensor *leftTouch;
  TouchSensor *rightTouch;

  void gripObject(Robot *robot, float ps, int obj)
  {
    float value;
    if (obj == OBJ_BALL)
    {
      value = 0.043;
    }
    else if (obj == OBJ_KEY)
    {
      value = 0.038;
    }
    cout << "gripping object " << value << endl;
    int checkCount = 0;
    int checkCount2 = 0;
    while (robot->step(TIME_STEP) != -1 && checkCount < 25 && checkCount2 < 5)
    {

      leftTouch->enable(TIME_STEP);
      rightTouch->enable(TIME_STEP);

      leftSliderEncoder->enable(TIME_STEP);
      rightSliderEncoder->enable(TIME_STEP);

      leftMotor->setVelocity(0.0);
      rightMotor->setVelocity(0.0);

      handleMotor->setVelocity(1.57);
      handleMotor->setPosition(1.57);

      for (int i = 0; i < 10; i++)
      {
        robot->step(TIME_STEP);
      }

      leftSlider->setPosition(ps);
      rightSlider->setPosition(ps);
      cout << leftSliderEncoder->getValue() << endl;
      if (leftTouch->getValue() && rightTouch->getValue())
      {
        if (leftSliderEncoder->getValue() >= value)
        {
          leftMotor->setVelocity(0.0);
          rightMotor->setVelocity(0.0);
          handleMotor->setVelocity(0.5);
          handleMotor->setPosition(1.57);
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
          leftSlider->setPosition(ps - 0.005);
          rightSlider->setPosition(ps - 0.005);
          leftMotor->setVelocity(1.5);
          rightMotor->setVelocity(0.025);
          for (int i = 0; i < 10; i++)
          {
            robot->step(TIME_STEP);
          }
          checkCount++;
        }
      }
      ps += 0.001;
      if (leftSliderEncoder->getValue() >= 0.05 || leftSliderEncoder->getValue() <= 0.0)
      {
        ps = 0;
        checkCount2++;
      }
    }
  }

  void init(Robot *robot)
  {
    handleMotor = robot->getMotor("handleMotor");
    handleEncoder = robot->getMotor("handleEncoder");

    leftSlider = robot->getMotor("leftSlider");
    rightSlider = robot->getMotor("rightSlider");
    shooter = robot->getMotor("shooterLinearMotor");

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

    handleMotor->setVelocity(1.57);
    handleMotor->setPosition(0.0);

    cout << "arm init" << endl;
  }
  void raise(Robot *robot, float angle)
  {
    // if (object == 0)
    // {
    handleMotor->setVelocity(0.7);
    handleMotor->setPosition(angle);
    //}
    // if (object == 3)
    // {
    //   // handleMotor->setVelocity(1);
    //   // handleMotor->setPosition(0);
    //   handleMotor->setVelocity(0.5);
    //   handleMotor->setPosition(1.2);
    // }
    for (int i = 0; i < 200; i++)
    {
      robot->step(TIME_STEP);
    }
  }

  // void down(Robot *robot){
  //   handleMotor->setVelocity(0.7);
  //   handleMotor->setPosition(1.57);
  // }

  void shoot(Robot *robot)
  {
    handleMotor->setVelocity(1);
    handleMotor->setPosition(1.57);
    for (int i = 0; i < 200; i++)
    {
      robot->step(TIME_STEP);
    }
    leftSlider->setPosition(0.035);
    rightSlider->setPosition(0.035);
    shooter->setPosition(-0.15);
    for (int i = 0; i < 150; i++)
    {
      robot->step(TIME_STEP);
    }
    shooter->setPosition(0);
    leftSlider->setPosition(0.0);
    rightSlider->setPosition(0.0);
    handleMotor->setVelocity(1);
    handleMotor->setPosition(0);
  }
  void ballShoot(Robot *robot, int redBall)
  {
    if (redBall)
    {
      mosaic::turnRight(robot);
      mosaic::goFront(robot, 295);
      mosaic::turnLeft(robot);
      shoot(robot);
    }
    else
    {
      mosaic::turnLeft(robot);
      mosaic::goFront(robot, 295);
      mosaic::turnRight(robot);
      shoot(robot);
    }
  }

}
