// File:          testing_controller.cpp
// Author: Team Plex

#include <webots/Robot.hpp>

#include "plexlibs/vision.hpp"
#include "plexlibs/arm.hpp"
#include "plexlibs/navigate.hpp"
#include "plexlibs/mosaic.hpp"
#include "plexlibs/wall.hpp"
#include "plexlibs/keyhole.hpp"
#include "plexlibs/normline.hpp"
#include "plexlibs/dashline.hpp"
#include "plexlibs/mscstps.hpp"

#define TIME_STEP 16

using namespace webots;
using namespace std;

// update this for changing the ball
// int redBall = 1; //for red ball
// int redBall = 0; //for blue ball
int redBall = 0;

int main(int argc, char **argv)
{

  Robot *robot = new Robot();

  while (robot->step(TIME_STEP) != -1)
  {

    arm::init(robot);
    normline::init(robot);
    normline::follow(robot);
    mosaic::init(robot);
    wall::init(robot);
    wall::follow(robot);
    mscstps::run(robot, redBall);
    dashline::init(robot, redBall);
    dashline::follow(robot);
    arm::ballShoot(robot, redBall);

    while (robot->step(TIME_STEP) != -1)
      ;
  };

  delete robot;
  return 0;
}
