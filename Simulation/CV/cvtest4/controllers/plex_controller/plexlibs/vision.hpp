#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define FM_HMIN 150
#define FM_HMAX 220
#define FM_SMIN 65
#define FM_SMAX 255
#define FM_VMIN 162
#define FM_VMAX 255

#define FC_HMIN 90
#define FC_HMAX 133
#define FC_SMIN 59
#define FC_SMAX 255
#define FC_VMIN 25
#define FC_VMAX 255

#define FY_HMIN 30
#define FY_HMAX 89
#define FY_SMIN 50
#define FY_SMAX 255
#define FY_VMIN 60
#define FY_VMAX 120

#define FK_HMIN
#define FK_HMAX
#define FK_SMIN
#define FK_SMAX
#define FK_VMIN
#define FK_VMAX

#define BR_HMIN
#define BR_HMAX
#define BR_SMIN
#define BR_SMAX
#define BR_VMIN
#define BR_VMAX

#define BB_HMIN
#define BB_HMAX
#define BB_SMIN
#define BB_SMAX
#define BB_VMIN
#define BB_VMAX

#define OY_HMIN
#define OY_HMAX
#define OY_SMIN
#define OY_SMAX
#define OY_VMIN
#define OY_VMAX

// #define _HMIN
// #define _HMAX
// #define _SMIN
// #define _SMAX
// #define _VMIN
// #define _VMAX

namespace vision
{

    void gotoObject();
    void mainFunction();
    void getFmMask(Mat &img, Mat &mask);
    void getFcMask(Mat &img, Mat &mask);
    void imageGradient(Mat &img, int width, int height, int &gi, int &gj);
}