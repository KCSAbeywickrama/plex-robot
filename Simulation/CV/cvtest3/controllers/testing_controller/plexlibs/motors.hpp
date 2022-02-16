#pragma once
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>

using namespace webots;
namespace motors
{
    void init(Robot *robot);
    void setLeftSpeed(float speed);
    void setRightSpeed(float speed);
    void setLeftPosition(float position);
    void setRightPosition(float position);
}