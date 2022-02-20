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
#define C_HMAX 115
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

#define O_HMIN 5
#define O_HMAX 27
#define O_SMIN 82
#define O_SMAX 255
#define O_VMIN 15
#define O_VMAX 255

#define R_HMIN 0
#define R_HMAX 16
#define R_SMIN 110
#define R_SMAX 255
#define R_VMIN 15
#define R_VMAX 255

#define B_HMIN 110
#define B_HMAX 120
#define B_SMIN 121
#define B_SMAX 255
#define B_VMIN 10
#define B_VMAX 255

#define W_HMIN 0
#define W_HMAX 179
#define W_SMIN 0
#define W_SMAX 20
#define W_VMIN 100
#define W_VMAX 255

#define K_HMIN 0
#define K_HMAX 179
#define K_SMIN 0
#define K_SMAX 71
#define K_VMIN 0
#define K_VMAX 65

// #define _HMIN
// #define _HMAX
// #define _SMIN
// #define _SMAX
// #define _VMIN
// #define _VMAX

#define CLR_M 0
#define CLR_C 1
#define CLR_Y 2
#define CLR_O 3
#define CLR_R 4
#define CLR_B 5
#define CLR_W 6
#define CLR_K 7

namespace vision
{
    void getMask(int color, Mat &img, Mat &mask);
    void getCombindMask(int color1, int color2, Mat &img, Mat &mask);
    void imageGradient(Mat &img, int width, int height, int &gi, int &gj);
}