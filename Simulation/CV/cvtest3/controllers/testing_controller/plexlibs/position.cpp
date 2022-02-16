#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "position.hpp"

#define TIME_STEP 16

using namespace webots;
using namespace std;

namespace position
{
    Robot *robot;
    PositionSensor *leftPosSensor;
    PositionSensor *rightPosSensor;
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

        leftPosSensor = robot->getPositionSensor("leftEncoder");
        rightPosSensor = robot->getPositionSensor("rightEncoder");

        robot->step(TIME_STEP);
    }

    void turnLeft(Robot *robot)
    {
        float rightStart = rightPosSensor->getValue();
        float rightThres = 3.608;
        leftMotor->setVelocity(-2);
        rightMotor->setVelocity(2);

        while (robot->step(TIME_STEP) != -1 && (rightPosSensor->getValue() - rightStart) < rightThres)
            ;

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    void turnRight(Robot *robot)
    {
        float leftStart = leftPosSensor->getValue();
        float leftThres = 3.608;
        leftMotor->setVelocity(2);
        rightMotor->setVelocity(-2);

        while (robot->step(TIME_STEP) != -1 && (leftPosSensor->getValue() - leftStart) < leftThres)
            ;

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    void goFront(Robot *robot, float distance)
    {
        float rad = distance / 30.0;
        float leftStart = leftPosSensor->getValue();
        float rightStart = rightPosSensor->getValue();

        leftMotor->setVelocity(2);
        rightMotor->setVelocity(2);

        while (robot->step(TIME_STEP) != -1)
        {
            if ((leftPosSensor->getValue() - leftStart) >= rad)
            {
                leftMotor->setVelocity(0);
            }
            if ((rightPosSensor->getValue() - rightStart) >= rad)
            {
                rightMotor->setVelocity(0);
            }
            if (((leftPosSensor->getValue() - leftStart) >= rad) && ((rightPosSensor->getValue() - rightStart) >= rad))
            {
                break;
            }
        }
    }
}
