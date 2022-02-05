// File:          cvtest2_vs.cpp
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

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  Camera* cam;
  cam= robot->getCamera("cam");
  cam->enable(timeStep);

  int imgWidth = cam->getWidth();
  int imgHeight = cam->getHeight();

  cout << imgWidth << endl;



  

  //for (int x = 0; x < 2; x++)
  //{
  //    for (int y = 0; y < 2; y++)
  //    {
  //        int r = cam->imageGetRed(image, imgWidth, x, y);
  //        cout << r << endl;
  //        // int g = cam->imageGetGreen(image, imgWidth, x, y);
  //        // int b = cam->imageGetBlue(image, imgWidth, x, y);

  //        // printf("red=%d, green=%d, blue=%d", r, g, b);
  //    }
  //}

  while (robot->step(timeStep) != -1) {
	  const unsigned char* image = cam->getImage();
	  cout << image[0] << endl;
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
