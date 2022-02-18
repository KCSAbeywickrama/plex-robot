#pragma once
#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

#define M_HMIN 150
#define M_HMAX 220
#define M_SMIN 65
#define M_SMAX 255
#define M_VMIN 162
#define M_VMAX 255

#define C_HMIN 90
#define C_HMAX 133
#define C_SMIN 59
#define C_SMAX 255
#define C_VMIN 25
#define C_VMAX 255

#define Y_HMIN 30
#define Y_HMAX 85
#define Y_SMIN 50
#define Y_SMAX 255
#define Y_VMIN 50
#define Y_VMAX 255

#define K_HMIN 0
#define K_HMAX 179
#define K_SMIN
#define K_SMAX
#define K_VMIN
#define K_VMAX

#define R_HMIN
#define R_HMAX
#define R_SMIN
#define R_SMAX
#define R_VMIN
#define R_VMAX

#define B_HMIN
#define B_HMAX
#define B_SMIN
#define B_SMAX
#define B_VMIN
#define B_VMAX

#define O_HMIN
#define O_HMAX
#define O_SMIN
#define O_SMAX
#define O_VMIN
#define O_VMAX

// #define _HMIN
// #define _HMAX
// #define _SMIN
// #define _SMAX
// #define _VMIN
// #define _VMAX

#define M 0
#define C 1
#define Y 2

namespace vision
{

    void gotoObject();
    void mainFunction();
    void getMask(int color, Mat &img, Mat &mask);
    void imageGradient(Mat &img, int width, int height, int &gi, int &gj);

}