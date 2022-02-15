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

#define TIME_STEP 16
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

  //vision::mainFunction();
  //vision::gotoObject();

  // create the Robot instance.
  float p_coefficient = 0.1;
  Robot *robot = new Robot();
  Motor *leftMotor = robot->getMotor("leftMotor");
  Motor *rightMotor = robot->getMotor("rightMotor");
  Motor *handleMotor = robot->getMotor("handleMotor");
  Motor *handleEncoder = robot->getMotor("handleEncoder");
  
  handleMotor->setPosition(INFINITY);
  handleMotor->setVelocity(0.0);
  

  leftMotor->setPosition(INFINITY);
  leftMotor->setVelocity(0.0);

  rightMotor->setPosition(INFINITY);
  rightMotor->setVelocity(0.0);

  Camera *camera = robot->getCamera("cam");
  camera->enable(TIME_STEP);
  const int width = camera->getWidth();
  const int height = camera->getHeight();
  // int imageLength = 4 * width * height * sizeof(unsigned char);
  Display *display = robot->getDisplay("display");

  const unsigned char *image;
  Mat imageMat = Mat(Size(width, height), CV_8UC4);
  Mat imgRGB, imgHSV, final, mask, dis;
  int hmin = 30, smin = 0, vmin = 0;
  int hmax = 88, smax = 255, vmax = 255;
  vector<vector<Point>> contours;
  vector<Point> poly;
  vector<Vec4i> hierarchy;
  RNG rng(12345);

  unsigned int timeCounter = 0;

  bool goingToObg = true;
  bool iscontours = true;
  while (robot->step(TIME_STEP) != -1)
  {
    
    handleMotor->setVelocity(1.57);
    handleMotor->setPosition(-1.57);
    //handleMotor->setPosition(0);
    //handleMotor->setVelocity(1.57);
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
    while (robot->step(TIME_STEP) != -1 && goingToObg)
    {
      
      image = camera->getImage();
      if (image)
      {
      cout<<"image"<<endl;

        // save images
        // if (timeCounter % 50 == 0)
        // {
        //   string filename = "imgs\\" + to_string(timeCounter / 50) + ".png";
        //   cout << "Saving" << filename << ':' << endl;
        //   cout << camera->saveImage(filename, 0) << endl;
        // }
        imageMat.data = (uchar *)image;
        cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
        cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        inRange(imgHSV, lower, upper, mask);

        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
      // vector<Point> c = contours.at(getMaxAreaContourId(contours));
      if (contours.empty()){
        cout<<"not found"<<endl;
        leftMotor->setVelocity(0.1 );
        rightMotor->setVelocity(-0.1);

      }
      else{
        int largestContour, largestContourArea;

        getMaxAreaContourId(contours, largestContour, largestContourArea);

        cout << largestContourArea << ' ';

        if (largestContourArea > width*height/8)
        {
          goingToObg = false;
          handleMotor->setVelocity(1.57);
          handleMotor->setPosition(0);
          leftMotor->setVelocity(0);
          rightMotor->setVelocity(0);
          approxPolyDP(Mat(contours[largestContour]), poly, 8, true);
          cout<<poly.size()<<endl;
          break;
        }

        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
        drawContours(mask, contours, largestContour, color, 2, LINE_8, hierarchy, 0);

        cvtColor(mask, final, COLOR_GRAY2RGB);
        // cvtColor(final, dis, COLOR_RGB2BGRA);
        ImageRef *ir = display->imageNew(width, height, final.data, Display::RGB);

        display->imagePaste(ir, 0, 0, false);
        display->imageDelete(ir);
        Moments mu = moments(contours[largestContour], false);
        
        int centerx = mu.m10 / mu.m00;
        // cout << centerx << ' ';
        float error = width / 2 - centerx;
        cout << error << endl;
        leftMotor->setVelocity((-error * p_coefficient)+1 );
        rightMotor->setVelocity((error * p_coefficient)+1);
       }
     }
    }
    //timeCounter++;
    // Enter here exit cleanup code.
  };
  // destroyAllWindows();
  delete robot;
  return 0;
}
