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
  DistanceSensor *rLaser = robot->getDistanceSensor("rLaser");
  DistanceSensor *lLaser = robot->getDistanceSensor("lLaser");
  DistanceSensor *fLaser = robot->getDistanceSensor("fLaser");
  lLaser->enable(TIME_STEP);
  rLaser->enable(TIME_STEP);
  fLaser->enable(TIME_STEP);

  // arm::init(robot);
  // mosaic::init(robot);
  // wall::init(robot);
  // wall::follow(robot);
  // mosaic::gotoCentre1(robot);
  // mosaic::turnLeft(robot);
  // mosaic::turnRight(robot);
  // mosaic::goFront(robot, 500);
  // mosaic::goBack(robot, 200);
  // mosaic::gotoMegenta(robot);

  string objName;
  string color = "blue";
  int object;

  while (robot->step(TIME_STEP) != -1)
  {
    cout << "f " << lLaser->getValue() << endl;
    cout << "r " << rLaser->getValue() << endl;
    cout << "l " << fLaser->getValue() << endl;
    // mosaic::goFront(robot,600);

    // arm::init(robot);
    // keyhole::init(robot);
    // keyhole::goToBox(robot);

    // mosaic::init(robot);

    // mosaic::gotoCentre1(robot);
    // navigate::init(robot);
    // navigate::navigateObject(robot);
    // arm::gripObject(robot,0.001,0);
    // navigate::detectObject(robot,object);
    // arm::raise(robot,0);
    // mosaic::turnRight(robot);
    // mosaic::turnRight(robot);
    // mosaic::goFront(robot,400);
    // mosaic::turnRight(robot);
    // mosaic::goFront(robot,850);

    // mosaic::gotoMagenta1(robot);
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

    // while (robot->step(TIME_STEP) != -1)
    //   ;
  };
  // destroyAllWindows();
  delete robot;
  return 0;
}
