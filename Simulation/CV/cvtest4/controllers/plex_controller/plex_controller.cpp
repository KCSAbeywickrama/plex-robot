
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
  Mat imgRGB, imgHSV, final, mask, dis;
  int hmin = 1, smin = 0, vmin = 0;
  int hmax = 89, smax = 255, vmax = 255;
  hmax = 29; // to not detect yellow floor
  vector<vector<Point>> contours;
  vector<Point> poly;
  vector<Vec4i> hierarchy;
  RNG rng(12345);

  unsigned int timeCounter = 0;

  bool goingToObg = true;
  bool iscontours = true;

  while (robot->step(TIME_STEP) != -1)
  {
    image = camera->getImage();
    if (image)
    {
      cout << "image" << endl;
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

      findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

      drawContours(mask, contours, -1, Scalar(255, 255, 255));

      printPointVectors(contours);
      cvtColor(mask, final, COLOR_GRAY2RGB);
      // cvtColor(final, dis, COLOR_RGB2BGRA);
      ImageRef *ir = display->imageNew(width, height, final.data, Display::RGB);

      display->imagePaste(ir, 0, 0, false);
      display->imageDelete(ir);
      // leftMotor->setVelocity(5);
      // rightMotor->setVelocity(5);
    }
    timeCounter++;
  };

  delete robot;
  return 0;
}
