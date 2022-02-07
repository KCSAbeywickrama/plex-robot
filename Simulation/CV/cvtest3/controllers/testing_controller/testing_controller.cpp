// File:          testing_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include<webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<opencv2/opencv.hpp>


#define TIME_STEP 64
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
int getMaxAreaContourId(vector <vector<cv::Point>> contours) {
    double maxArea = 0;
    int maxAreaContourId = -1;
    for (int j = 0; j < contours.size(); j++) {
        double newArea = cv::contourArea(contours.at(j));
        if (newArea > maxArea) {
            maxArea = newArea;
            maxAreaContourId = j;
        } // End if
    } // End for
    return maxAreaContourId;
} // End function

int main(int argc, char **argv) {
  // create the Robot instance.
  float p_coefficient = 0.1;
  Robot *robot = new Robot();
  Motor *left_motor = robot->getMotor("left_motor");
  Motor *right_motor = robot->getMotor("right_motor");
  
  left_motor-> setPosition(INFINITY);
  left_motor->setVelocity(0.0);
  
  right_motor-> setPosition(INFINITY);
  right_motor->setVelocity(0.0);
  
  Camera *camera = robot->getCamera("cam");
  camera->enable(TIME_STEP);
  const int width = camera->getWidth();
  const int height = camera->getHeight();
  //int imageLength = 4 * width * height * sizeof(unsigned char);
  Display *display = robot->getDisplay("display");
  //printf("red=%d\n", height);
  //display->setAlpha(0.5);
  //display->fillRectangle(10,10,10,10);

  // get the time step of the current world.
  //int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until Webots is stopping the controller
  const unsigned char *image;
  Mat imageMat = Mat(Size(width, height), CV_8UC4);
  Mat imgRGB, imgHSV ,final,mask,dis;
  int hmin = 60, smin = 0, vmin = 0;
  int hmax = 180, smax = 255, vmax = 255;
  vector<vector<Point>> contours;
  vector<Vec4i> hierarchy;
  //Mat imageProccMat;
  while (robot->step(TIME_STEP) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  
 image = camera->getImage();
    if (image)
    {
      imageMat.data = (uchar *)image;
      cvtColor(imageMat,imgRGB,COLOR_BGRA2RGB);
      cvtColor(imgRGB,imgHSV,COLOR_RGB2HSV);
      //namedWindow("ttt");

      /*namedWindow("Trackbars",(640 , 200));
      createTrackbar("Hue Min","Trackbars",&hmin,179);
      createTrackbar("Hue Max","Trackbars",&hmax,179);
      createTrackbar("Sat Min","Trackbars",&smin,255);
      createTrackbar("Sat Max","Trackbars",&smax,255);
      createTrackbar("Val Min","Trackbars",&vmin,255);
      createTrackbar("Val Max","Trackbars",&vmax,255);*/

      Scalar lower(hmin, smin, vmin);
      Scalar upper(hmax, smax, vmax);
      inRange(imgHSV, lower, upper, mask);

      cvtColor(mask,final, COLOR_GRAY2RGB);
      cvtColor(final,dis, COLOR_RGB2BGRA);
      ImageRef *ir = display->imageNew(width, height, final.data, Display::RGB);

      display->imagePaste(ir, 0, 0, false);
      display->imageDelete(ir);

      findContours(mask, contours,hierarchy, RETR_EXTERNAL,CHAIN_APPROX_NONE);
      //vector<Point> c = contours.at(getMaxAreaContourId(contours));
      int largestContour = getMaxAreaContourId(contours);
      Moments mu = moments( contours[largestContour], false );
      int centerx = mu.m10/mu.m00;
      cout<<centerx;
      float error = width/2 - centerx;
      cout<<error<<endl;
    left_motor->setVelocity(- error * p_coefficient);
    right_motor->setVelocity(error * p_coefficient);


      
    
    }

  // Enter here exit cleanup code.
  };
  //destroyAllWindows();
  delete robot;
  return 0;
}
