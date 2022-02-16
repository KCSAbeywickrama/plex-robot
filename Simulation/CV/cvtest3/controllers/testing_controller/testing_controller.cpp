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
#include "plexlibs/navigate.hpp"


#define TIME_STEP 16

using namespace webots;
using namespace std;
using namespace cv;

int main(int argc, char **argv)
{

  
   Robot *robot = new Robot();
  
  string objName;
  string color = "blue";
  
   while (robot->step(TIME_STEP) != -1)
   {
    arm::init(robot);
    //arm::gripObject(robot,0.001,"ball");
    navigate::init(robot);
    navigate::navigateBall(robot, color) ;
    arm::gripObject(robot,0.001,"ball");
    while (robot->step(TIME_STEP) != -1);

    
  
  };
  // destroyAllWindows();
  delete robot;
  return 0;
}
