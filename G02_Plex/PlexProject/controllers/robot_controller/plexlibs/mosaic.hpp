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
#define TURN_THRES_90 3.608

#define OBJ_KEY 0
#define OBJ_CYLNDR 1
#define OBJ_BOX 2
#define OBJ_BALL 3

using namespace webots;
using namespace std;
using namespace cv;

namespace mosaic
{
    void init(Robot *robot);
    void turnLeft(Robot *robot, float rightThres = TURN_THRES_90);
    void turnRight(Robot *robot, float leftThres = TURN_THRES_90);
    void goFront(Robot *robot, float distance);
    void goBack(Robot *robot, float distance);
    void driftLeft(Robot *robot);
    void lookFromLeft(Robot *robot);
    void lookFromRight(Robot *robot);
    void delay(Robot *robot, int count);

    void showImgRGB(Mat &img);
    void showImgGray(Mat &img);
    void showFilter(Robot *robot, int color);
    // void showCombinedFilter(Robot *robot, int color1, int color2);

    bool notIn(Robot *robot);
    void goWall2MagentaEnc(Robot *robot);
    void goBox2MagentaEnc(Robot *robot);
    void goCylinder2MagentaEnc(Robot *robot);
    void preAlignKeyHole(Robot *robot);
    void alignWhileGoing(Robot *robot, int color, int dis);
    void alignToWall(Robot *robot);
    void exit(Robot *robot);
    void tmpViewFloorPoints(Robot *robot);
    void goYellow2Box(Robot *robot);
    void goYellow2Cylinder(Robot *robot);
    void goHole2Cyan(Robot *robot);
}