// File:          testing_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "plexlibs/vision.hpp"
#include "plexlibs/arm.hpp"
#include "plexlibs/motors.hpp"

#define TIME_STEP 16

using namespace webots;
using namespace std;
using namespace cv;

// using namespace arm;

void getMaxAreaContourId(vector<vector<Point>> contours, int &id, int &area)
{
  double maxArea = 0;
  id = -1;
  for (int j = 0; j < contours.size(); j++)
  {
    double newArea = contourArea(contours.at(j));
    if (newArea > maxArea)
    {
      maxArea = newArea;
      id = j;
    } // End if
  }   // End for

  area = (int)maxArea;
} // End function

int gotoObg()
{
}

int main(int argc, char **argv)
{

  Robot *robot = new Robot();

  int ps = 0.002;
  bool objTouch = true;

  motors::init(robot);
  arm::init(robot);
  arm::gripObject(robot, ps, objTouch);

  while (robot->step(TIME_STEP) != -1)
  {
  };

  delete robot;
  return 0;
}
