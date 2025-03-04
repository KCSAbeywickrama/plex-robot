#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <iostream>
#include <cmath>

#define TIME_STEP 16
#define MAX_SPEED 6
#define BASE_SPEED 3
#define p 0.8
#define d 0.1

using namespace webots;
using namespace std;

namespace wall
{
    void init(Robot *robot);
    void follow(Robot *robot);
}