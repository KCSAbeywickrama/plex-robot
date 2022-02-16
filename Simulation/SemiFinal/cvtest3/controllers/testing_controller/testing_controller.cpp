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
        } 
    } 
    }
int main(int argc, char **argv)
{

  //vision::mainFunction();
  //vision::gotoObject();

  // create the Robot instance.
  // float p_coefficient = 0.1;
   Robot *robot = new Robot();
  // Motor *leftMotor = robot->getMotor("leftMotor");
  // Motor *rightMotor = robot->getMotor("rightMotor");
  // Motor *handleMotor = robot->getMotor("handleMotor");
  // Motor *handleEncoder = robot->getMotor("handleEncoder");
  
  // handleMotor->setPosition(INFINITY);
  // handleMotor->setVelocity(0.0);
  

  // leftMotor->setPosition(INFINITY);
  // leftMotor->setVelocity(0.0);

  // rightMotor->setPosition(INFINITY);
  // rightMotor->setVelocity(0.0);

  // Camera *camera = robot->getCamera("cam");
  // camera->enable(TIME_STEP);
  // const int width = camera->getWidth();
  // const int height = camera->getHeight();
  // // // int imageLength = 4 * width * height * sizeof(unsigned char);
  // Display *display = robot->getDisplay("display");

  // const unsigned char *image;
  // Mat imageMat = Mat(Size(width, height), CV_8UC4);
  // Mat imgAnd = Mat(Size(width, height), CV_8UC4);
  // Mat imgRGB, imgHSV, mask, maskRGB, imgCanny ,imgDil, imgErode ;
  // int hmin = 1, smin = 50, vmin = 0;
  // int hmax = 88, smax = 255, vmax = 255;
  // vector<vector<Point>> contours;
  // vector<Point> poly;
  // vector<Vec4i> hierarchy;
  // RNG rng(12345);

  // unsigned int timeCounter = 0;
  // int ps = 0.002;
  // bool objTouch = true;
  string objName;
  // bool goingToObj = true;
  // bool goingToBall = true;
  // bool iscontours = true;
  // bool cylinder=true;
  // bool box=true;
   while (robot->step(TIME_STEP) != -1)
   {
    arm::init(robot);
    //arm::gripObject(robot,0.001,"ball");
    navigate::init(robot);
    navigate::navigateObject(robot,objName);
    arm::gripObject(robot,0.001,objName);


    
  //   while (robot->step(TIME_STEP) != -1 && goingToObj)
  //   {
  // //     // handleMotor->setVelocity(1.57);
  // //     // handleMotor->setPosition(-1.57);
      
  //     image = camera->getImage();
  //     if (image)
  //     {
        
  //       cout<<"image"<<endl;

        
  //       imageMat.data = (uchar *)image;
  //       cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
  //       cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

  //       Scalar lower(hmin, smin, vmin);
  //       Scalar upper(hmax, smax, vmax);
  //       inRange(imgHSV, lower, upper, mask);
        
        
  //       cvtColor(mask, maskRGB, COLOR_GRAY2RGB);
  //       bitwise_and(imgRGB,maskRGB,imgAnd);
        
  // //       // ImageRef *ir = display->imageNew(width, height, imgAnd.data, Display::RGB);
  // //       // display->imagePaste(ir, 0, 0, false);
  // //       // display->imageDelete(ir);

        
  // //       // Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));

	// //       // dilate(imgCanny, imgDil, kernel);
	// //       // erode(imgDil, imgErode, kernel);

  //        cvtColor(imgAnd, imgErode, COLOR_RGB2GRAY);
  // //       //findContours(imgErode, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
  //       findContours(imgErode, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
  // //     // vector<Point> c = contours.at(getMaxAreaContourId(contours));
      

  //     if (contours.empty())
  //     {
  //       cout<<"not found"<<endl;
  //       leftMotor->setVelocity(0.5 );
  //       rightMotor->setVelocity(-0.5);

  //     }
  //     else{
  //       int largestContour, largestContourArea;

  //       getMaxAreaContourId(contours, largestContour, largestContourArea);
  //       Point extTop   = *max_element(contours[largestContour].begin(), contours[largestContour].end(), 
  //                     [](const Point& lhs, const Point& rhs) {
  //                         return lhs.y < rhs.y;
  //                     });
        
  //       //float distance = extTop.y;

  //       cout << largestContourArea <<' '<< extTop.y << ' ';

  //       if (extTop.y >= 127)
  //       {
  //         goingToObj = false;
  //         // handleMotor->setVelocity(1.57);
  //         // handleMotor->setPosition(0);
  //         leftMotor->setVelocity(0);
  //         rightMotor->setVelocity(0);
  //         approxPolyDP(Mat(contours[largestContour]), poly, 1, true); 
  //         //box=7,cylinder=9
  //         if (poly.size()>=9){objName="cylinder";}
  //         else{objName="Box";}
  //         cout<<poly.size()<<objName<<endl;
  //         break;
  //       }
  //       if (largestContourArea>0)
  //       {
  //         Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
  //         drawContours(imgAnd, contours, largestContour, color, 2, LINE_8, hierarchy, 0);
  //         //circle(imgAnd, extTop, 10, Scalar(0, 0, 255), FILLED);
  //        // cvtColor(imgErode, final, COLOR_GRAY2RGB);
  //        // cvtColor(final, dis, COLOR_RGB2BGRA);
  //         ImageRef *ir = display->imageNew(width, height, imgAnd.data, Display::RGB);
           

  //         display->imagePaste(ir, 0, 0, false);
  //         display->imageDelete(ir);
  //         Moments mu = moments(contours[largestContour], false);
          
  //         int centerx = mu.m10 / mu.m00;
  //         // cout << centerx << ' ';
  //         float error = width / 2 - centerx;
  //         cout << error << endl;
  //         leftMotor->setVelocity((-error * p_coefficient)+0.5 );
  //         rightMotor->setVelocity((error * p_coefficient)+0.5);
  //      }
  //      else
  //      {
  //       leftMotor->setVelocity(0.1 );
  //       rightMotor->setVelocity(-0.1);
  //      }
  //    }
  //   }
  //   //timeCounter++;
  // } // Enter here exit cleanup code.
  
  // while (robot->step(TIME_STEP) != -1 && objTouch)
  // {
  //   arm::init(robot);
  //   arm::gripObject(robot,ps,"object");
  // }
  //cout<<"end"<<endl;
  // leftMotor->setVelocity(0.1 );
  // rightMotor->setVelocity(-0.1);
  };
  // destroyAllWindows();
  delete robot;
  return 0;
}
