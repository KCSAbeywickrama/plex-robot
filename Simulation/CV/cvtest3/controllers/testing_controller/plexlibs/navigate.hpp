#pragma once
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

#define TIME_STEP 16
using namespace webots;
using namespace std;
using namespace cv;

namespace navigate
{
    void getMaxAreaContourId(vector<vector<Point>> contours, int &id, int &area);
    void navigateObject(Robot *robot, string &objName);
    void navigateBall(Robot *robot, string color) ;
    void init(Robot *robot);
}