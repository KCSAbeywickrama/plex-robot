#include "vision.hpp"

namespace vision
{

    uchar minMargin[3][3] = {
        {M_HMIN, M_SMIN, M_VMIN},
        {C_HMIN, C_SMIN, C_VMIN},
        {Y_HMIN, Y_SMIN, Y_VMIN}};

    uchar maxMargin[3][3] = {
        {M_HMAX, M_SMAX, M_VMAX},
        {C_HMAX, C_SMAX, C_VMAX},
        {Y_HMAX, Y_SMAX, Y_VMAX}};

    void gotoObject()
    {
        cout << "vision:gotoObject()" << endl;
    }
    void mainFunction()
    {
        cout << "vision:mainFunction()" << endl;
    }

    void getMask(int color, Mat &img, Mat &mask)
    {
        Scalar lower(minMargin[color][0], minMargin[color][1], minMargin[color][2]);
        Scalar upper(maxMargin[color][0], maxMargin[color][1], maxMargin[color][2]);
        inRange(img, lower, upper, mask);
    }

    void imageGradient(Mat &img, int width, int height, int &gi, int &gj)
    {
        gi = 0;
        gj = 0;

        for (int i = 0; i < height - 1; i++)
        {
            uchar *line0 = img.ptr<uchar>(i);
            uchar *line1 = img.ptr<uchar>(i + 1);
            for (int j = 0; j < width - 1; j++)
            {
                gi += line1[j] - line0[j];
                gj += line0[j + 1] - line0[j];
            }
        }
    }

}
