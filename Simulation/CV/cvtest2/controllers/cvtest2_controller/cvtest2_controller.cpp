// File:          cvtest2_controller.cpp
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
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;
using namespace std;
using namespace cv;
// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node

int main(int argc, char **argv)
{

  // create the Robot instance.
  Robot *robot = new Robot();
  int timeStep = (int)robot->getBasicTimeStep();

  Camera *camera = robot->getCamera("camera");
  camera->enable(timeStep);

  int imageWidth = camera->getWidth();
  int imageHeight = camera->getHeight();
  int imageLength = 4 * imageWidth * imageHeight * sizeof(unsigned char);
  // const unsigned char *proccImage = (unsigned char *)malloc(imageLength);

  cout << imageWidth << endl;
  printf("ra=%d\n", imageHeight);

  Display *display = robot->getDisplay("display");

  // Motor *wheels[2];
  // char wheelsNames[2][7] = {"motorL", "motorR"};
  // for (int i = 0; i < 2; i++)
  //   wheels[i] = robot->getMotor(wheelsNames[i]);

  // double speedL = 1.5; // [rad/s]
  // wheels[0]->setPosition(INFINITY);
  // wheels[0]->setVelocity(speedL);

  // double speedR = 1.5; // [rad/s]
  // wheels[1]->setPosition(INFINITY);
  // wheels[1]->setVelocity(speedR);

  // for (int x = 0; x < 2; x++)
  // {
  //   for (int y = 0; y < 2; y++)
  //   {
  //     int r = camera->imageGetRed(img, imgWidth, x, y);
  //     cout << r << endl;
  //     // int g = camera->imageGetGreen(img, imgWidth, x, y);
  //     // int b = camera->imageGetBlue(img, imgWidth, x, y);

  //     // printf("red=%d, green=%d, blue=%d", r, g, b);
  //   }
  // }

  const unsigned char *image;
  Mat imageMat = Mat(Size(imageWidth, imageHeight), CV_8UC4);
  Mat imageProccMat;
  while (robot->step(timeStep) != -1)
  {
    image = camera->getImage();
    if (image)
    {
      imageMat.data = (uchar *)image;
      GaussianBlur(imageMat, imageProccMat, Size(9, 9), 0);
      ImageRef *ir = display->imageNew(imageWidth, imageHeight, imageProccMat.data, Display::BGRA);

      display->imagePaste(ir, 0, 0, false);
      display->imageDelete(ir);
    }
  };

  // Enter here exit cleanup code.

  // delete proccImage;
  // delete imageMat;
  delete robot;
  return 0;
}
