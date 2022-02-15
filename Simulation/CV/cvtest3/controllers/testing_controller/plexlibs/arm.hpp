
#pragma once
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/TouchSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>

#define TIME_STEP 16
using namespace webots;
using namespace std;
namespace arm
{
  void init(Robot *robot);
  void gripObject(Robot *robot, float ps, bool &objTouch);
}