#pragma once

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>
#include <webots/PositionSensor.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#define TIME_STEP 16
#define MOSAIC_SPEED 2

using namespace webots;
using namespace std;
using namespace cv;

namespace mosaic
{
    void init(Robot *robot);
    void turnLeft(Robot *robot);
    void turnRight(Robot *robot);
    void goFront(Robot *robot, float distance);
    void goBack(Robot *robot, float distance);

    void showImgRGB(Mat &img);
    void showImgGray(Mat &img);
    void showFilter(Robot *robot, int color);

    bool notIn(Robot *robot);
    void goWall2MagentaEnc(Robot *robot);
    void goBox2MagentaEnc(Robot *robot);
    void goCylinder2MagentaEnc(Robot *robot);
    void preAlignKeyHole(Robot *robot);
}