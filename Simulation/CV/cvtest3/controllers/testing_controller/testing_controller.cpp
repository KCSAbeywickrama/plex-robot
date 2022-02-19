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
#include <webots/DistanceSensor.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include "plexlibs/vision.hpp"
#include "plexlibs/arm.hpp"
#include "plexlibs/navigate.hpp"
#include "plexlibs/mosaic.hpp"
#include "plexlibs/wall.hpp"
#include "plexlibs/keyhole.hpp"

#define TIME_STEP 16

using namespace webots;
using namespace std;
using namespace cv;
// obj 0-cylinder/box
// obj 1 - box
// obj 2 - cylinder
// obj 3 - ball

int main(int argc, char **argv)
{

  Robot *robot = new Robot();

  // arm::init(robot);
  // mosaic::init(robot);
  // wall::init(robot);
  // wall::follow(robot);
  // mosaic::goWall2Magenta(robot);
  // mosaic::turnLeft(robot);
  // mosaic::turnRight(robot);
  // mosaic::goFront(robot, 500);
  // mosaic::goBack(robot, 200);
  // mosaic::gotoMegenta(robot);

  string objName;
  string color = "red";
  int object;

  while (robot->step(TIME_STEP) != -1)
  {
    //  arm::init(robot);
    //  keyhole::init(robot);
    //  keyhole::goToBox(robot);
    // navigate::init(robot);
    // navigate::navigateBall(robot,color) ;
    //  arm::gripObject(robot,0.001,3);
    //  arm::raise(robot,3);

    //*************************************************************************************************

    arm::init(robot);
    mosaic::init(robot);

    mosaic::goWall2Magenta(robot);
    navigate::init(robot);
    navigate::navigateObject(robot);
    arm::gripObject(robot, 0.001, 0);
    navigate::detectObject(robot, object);
    arm::raise(robot, 0);

    // add function
    mosaic::preAlignKeyHole(robot);
    cout << "chamod out" << endl;
    float value;
    keyhole::init(robot);
    keyhole::frontReading(robot, value);
    cout << "front reading: " << endl;
    if (value > 50)
    {
      mosaic::goFront(robot, value - 50);
    }
    mosaic::turnRight(robot);
    cout << "pos correct" << endl;

    if (object == 1)
    {
      keyhole::goToBox(robot);
      arm::shoot(robot);
      // box to magenta
      mosaic::turnRight(robot);
      mosaic::turnRight(robot);
      mosaic::goFront(robot, 850);
      mosaic::turnLeft(robot);
      mosaic::goFront(robot, 190);
    }
    if (object == 2)
    {
      keyhole::goToCylinder(robot);

      arm::shoot(robot);
      // cylinder to magenta
      mosaic::turnRight(robot);
      mosaic::turnRight(robot);
      mosaic::goFront(robot, 850);
      mosaic::turnLeft(robot);
      mosaic::goFront(robot, 20);
    }
    cout << "object 2" << endl;
    navigate::navigateObject(robot);
    arm::gripObject(robot, 0.001, 0);
    navigate::detectObject(robot, object);
    arm::raise(robot, 0);

    mosaic::preAlignKeyHole(robot);
    cout << "chamod out" << endl;
    keyhole::init(robot);
    keyhole::frontReading(robot, value);
    cout << "front reading: " << endl;
    if (value > 50)
    {
      mosaic::goFront(robot, value - 50);
    }
    mosaic::turnRight(robot);
    cout << "pos correct" << endl;

    if (object == 1)
    {
      keyhole::goToBox(robot);
      arm::shoot(robot);
      // box to magenta
      mosaic::turnRight(robot);
      mosaic::turnRight(robot);
      mosaic::goFront(robot, 850);
      mosaic::turnLeft(robot);
      mosaic::goFront(robot, 190);
    }
    if (object == 2)
    {
      keyhole::goToCylinder(robot);

      arm::shoot(robot);
      // cylinder to magenta
      mosaic::turnRight(robot);
      mosaic::turnRight(robot);
      mosaic::goFront(robot, 850);
      mosaic::turnLeft(robot);
      mosaic::goFront(robot, 20);
    }
    //******************************************************************************
    // mosaic::goFront(robot,850);

    // mosaic::goCyan2Magenta(robot);
    // arm::shoot(robot);

    // from wall to magenta
    // mosaic::goFront(robot, 500);
    // mosaic::turnLeft(robot);
    // mosaic::goFront(robot, 650);
    // mosaic::turnRight(robot);
    // mosaic::goFront(robot,120);

    // from cylinder to magenta
    //  mosaic::turnRight(robot);
    //  mosaic::turnRight(robot);
    //  mosaic::goFront(robot,850);
    //  mosaic::turnLeft(robot);
    //  mosaic::goFront(robot,20);

    // from box to magenta
    // mosaic::turnRight(robot);
    // mosaic::turnRight(robot);
    // mosaic::goFront(robot,850);
    // mosaic::turnLeft(robot);
    // mosaic::goFront(robot,190);

    // //arm::gripObject(robot,0.001,"ball");
    // navigate::init(robot);
    // //navigate::navigateBall(robot, color) ;
    // navigate::navigateObject(robot,objName);
    // arm::gripObject(robot,0.001,1);
    // cout<<"end"<<endl;
    // mosaic::init(robot);

    while (robot->step(TIME_STEP) != -1)
      ;
  };
  // destroyAllWindows();
  delete robot;
  return 0;
}
