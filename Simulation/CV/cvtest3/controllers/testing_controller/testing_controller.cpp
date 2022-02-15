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

#define TIME_STEP 16

using namespace webots;
using namespace std;
using namespace cv;
//using namespace arm;

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
  Mat imgRGB, imgHSV, imgAnd , mask, imgDil, imgErode, final;
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
    
    while (robot->step(TIME_STEP) != -1 && goingToObg)
    {
      handleMotor->setVelocity(1.57);
      handleMotor->setPosition(-1.57);
      
      image = camera->getImage();
      if (image)
      {
        
      cout<<"image"<<endl;

        
        imageMat.data = (uchar *)image;
        cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
        cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

        Scalar lower(hmin, smin, vmin);
        Scalar upper(hmax, smax, vmax);
        inRange(imgHSV, lower, upper, mask);

        unsigned char * pData = imgRGB.data;

        Mat outImg(width, height, CV_8UC1);  
        memcpy(outImg.data, pData, sizeof(unsigned char)*width*height);  

        bitwise_and(imgRGB,outImg,imgAnd, mask);
        //cvtColor(mask, final, COLOR_GRAY2RGB);

        ImageRef *ir = display->imageNew(width, height, imgAnd.data, Display::RGB);
        display->imagePaste(ir, 0, 0, false);
        display->imageDelete(ir);
        
        Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));

	      dilate(mask, imgDil, kernel);
	      erode(imgDil, imgErode, kernel);

        //findContours(imgErode, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
      findContours(imgErode, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
      // vector<Point> c = contours.at(getMaxAreaContourId(contours));
      

      if (contours.empty())
      {
        cout<<"not found"<<endl;
        leftMotor->setVelocity(0.5 );
        rightMotor->setVelocity(-0.5);

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
          approxPolyDP(Mat(contours[largestContour]), poly, 1, true);
          cout<<poly.size()<<endl;
          break;
        }
        if (largestContourArea>0)
        {
          Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
          drawContours(imgErode, contours, largestContour, color, 2, LINE_8, hierarchy, 0);

          cvtColor(imgErode, final, COLOR_GRAY2RGB);
          // cvtColor(final, dis, COLOR_RGB2BGRA);
          //ImageRef *ir = display->imageNew(width, height, final.data, Display::RGB);
           

          // display->imagePaste(ir, 0, 0, false);
          // display->imageDelete(ir);
          Moments mu = moments(contours[largestContour], false);
          
          int centerx = mu.m10 / mu.m00;
          // cout << centerx << ' ';
          float error = width / 2 - centerx;
          cout << error << endl;
          leftMotor->setVelocity((-error * p_coefficient)+0.5 );
          rightMotor->setVelocity((error * p_coefficient)+0.5);
       }
       else
       {
        leftMotor->setVelocity(0.1 );
        rightMotor->setVelocity(-0.1);
       }
     }
    }
    //timeCounter++;
  } // Enter here exit cleanup code.
  // int ps = 0.002;
  // bool objTouch = true;
  // arm::init;
  // arm::gripObject(ps,objTouch);
  };
  // destroyAllWindows();
  delete robot;
  return 0;
}
