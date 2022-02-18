#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#define TIME_STEP 16
#define NORM_MAX 6.28
#define NORM_BASE 4

using namespace webots;

namespace dashline
{
    void init(Robot *robot);
    void follow(Robot *robot);
}