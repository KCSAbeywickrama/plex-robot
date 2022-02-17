
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include "plexlibs/vision.hpp"

#define TIME_STEP 16

using namespace webots;
using namespace std;
using namespace cv;

void getFcMask(Mat &img, Mat &mask)
{
  Scalar lower(FC_HMIN, FC_SMIN, FC_VMIN);
  Scalar upper(FC_HMAX, FC_SMAX, FC_VMAX);
  inRange(img, lower, upper, mask);
  // int hmin = 1, smin = 0, vmin = 0;
  // int hmax = 89, smax = 255, vmax = 255;
  // Scalar lower(hmin, smin, vmin);
  // Scalar upper(hmax, smax, vmax);
  // inRange(img, lower, upper, mask);
}

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

  area = (int)maxArea;
}

void printPointVectors(vector<vector<Point>> pointsVect)
{
  for (int i = 0; i < pointsVect.size(); i++)
  {
    for (int j = 0; j < pointsVect[i].size(); j++)
    {
      cout << pointsVect[i][j].x << ':' << pointsVect[i][j].y << ", ";
    }
    cout << endl;
  }
}

int main(int argc, char **argv)
{

  // vision::mainFunction();
  // vision::gotoObject();

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
  leftMotor->setVelocity(-0.0);

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
  Mat imgRGB, imgHSV, imgDis, mask, dis;
  int hmin = 1, smin = 0, vmin = 0;
  int hmax = 89, smax = 255, vmax = 255;
  // hmax = 29; // to not detect yellow floor
  // int hmin = 30, smin = 50, vmin = 60;
  // int hmax = 89, smax = 255, vmax = 120;
  // hmax = 29;
  vector<vector<Point>> contours;
  vector<Point> poly;
  vector<Vec4i> hierarchy;
  RNG rng(12345);

  unsigned int timeCounter = 0;

  bool goingToObg = true;
  bool iscontours = true;

  leftMotor->setVelocity(1);
  rightMotor->setVelocity(-1);
  // rotating
  while (robot->step(TIME_STEP) != -1)
  {
    image = camera->getImage();
    if (image)
    {

      imageMat.data = (uchar *)image;
      cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
      cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

      vision::getFmMask(imgHSV, mask);

      int i1 = 0;
      for (i1 = height - 1; i1 >= 0; i1--)
      {
        uchar *line = mask.ptr<uchar>(i1);
        if (line[0])
          break;
      }

      cout << "i1:" << i1 << endl;

      if (i1 >= 0)
        break;

      cvtColor(mask, imgDis, COLOR_GRAY2RGB);

      ImageRef *ir = display->imageNew(width, height, imgDis.data, Display::RGB);

      display->imagePaste(ir, 0, 0, false);
      display->imageDelete(ir);
    }
  };

  for (int i = 0; i < 10; i++)
  {
    robot->step(TIME_STEP);
  }

  while (robot->step(TIME_STEP) != -1)
  {
    image = camera->getImage();
    if (image)
    {

      imageMat.data = (uchar *)image;
      cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
      cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

      vision::getFmMask(imgHSV, mask);

      int i1 = 0;
      for (i1 = height - 1; i1 >= 0; i1--)
      {
        uchar *line = mask.ptr<uchar>(i1);
        if (line[0])
          break;
      }

      int i2 = 0;
      for (i2 = height - 1; i2 >= 0; i2--)
      {
        uchar *line = mask.ptr<uchar>(i2);
        if (line[width - 1])
          break;
      }

      int error = i1 - i2;
      float p_coefficient = 0.1;
      float d_coefficient = 0.8;
      cout << " i1:" << i1;
      cout << " i2:" << i2;
      cout << " lineerror: ";
      cout << error << endl;

      if (error < 10)
      {
        leftMotor->setVelocity((-error * p_coefficient) + 2);
        rightMotor->setVelocity((error * p_coefficient) + 2);
      }
      else
      {
        leftMotor->setVelocity(1);
        rightMotor->setVelocity(-1);
      }
      // if (error >= 0)
      // {
      //   // leftMotor->setVelocity(1);
      //   // rightMotor->setVelocity(1);
      //   break;
      // }

      cvtColor(mask, imgDis, COLOR_GRAY2RGB);

      ImageRef *ir = display->imageNew(width, height, imgDis.data, Display::RGB);

      display->imagePaste(ir, 0, 0, false);
      display->imageDelete(ir);
    }
    timeCounter++;
  };

  delete robot;
  return 0;
}
