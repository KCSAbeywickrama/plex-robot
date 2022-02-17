#pragma once
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>
#include <webots/PositionSensor.hpp>

#define TIME_STEP 16

using namespace webots;
using namespace std;

namespace mosaic
{
    void init(Robot *robot);
    void turnLeft(Robot *robot);
    void turnRight(Robot *robot);
    void goFront(Robot *robot, float distance);
}