#include "motors.hpp"

namespace motors
{
    Motor *leftMotor;
    Motor *rightMotor;

    void init(Robot *robot)
    {
        leftMotor = robot->getMotor("leftMotor");
        rightMotor = robot->getMotor("rightMotor");

        leftMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.0);

        rightMotor->setPosition(INFINITY);
        rightMotor->setVelocity(0.0);
    }

    void setLeftSpeed(float speed)
    {
        leftMotor->setVelocity(speed);
    }

    void setLeftPosition(float position)
    {
        leftMotor->setPosition(position);
    }

    void setRightSpeed(float speed)
    {
        rightMotor->setVelocity(speed);
    }
     void setRightPosition(float position)
    {
        rightMotor->setPosition(position);
    }
}