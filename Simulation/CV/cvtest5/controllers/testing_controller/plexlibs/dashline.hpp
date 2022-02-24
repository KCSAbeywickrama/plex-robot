#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 16
#define PATH_MAX_SPEED 6.28
#define PATH_BASE_SPEED 3

using namespace webots;

namespace dashline
{
    void init(Robot *robot, int _redBall);
    void follow(Robot *robot);
}