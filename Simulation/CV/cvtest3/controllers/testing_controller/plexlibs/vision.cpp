#include "vision.hpp"

namespace vision
{

    uchar minMargin[8][3] = {
        {M_HMIN, M_SMIN, M_VMIN},
        {C_HMIN, C_SMIN, C_VMIN},
        {Y_HMIN, Y_SMIN, Y_VMIN},
        {O_HMIN, O_SMIN, O_VMIN},
        {R_HMIN, R_SMIN, R_VMIN},
        {B_HMIN, B_SMIN, B_VMIN},
        {W_HMIN, W_SMIN, W_VMIN},
        {K_HMIN, K_SMIN, K_VMIN},
    };

    uchar maxMargin[8][3] = {
        {M_HMAX, M_SMAX, M_VMAX},
        {C_HMAX, C_SMAX, C_VMAX},
        {Y_HMAX, Y_SMAX, Y_VMAX},
        {O_HMAX, O_SMAX, O_VMAX},
        {R_HMAX, R_SMAX, R_VMAX},
        {B_HMAX, B_SMAX, B_VMAX},
        {W_HMAX, W_SMAX, W_VMAX},
        {K_HMAX, K_SMAX, K_VMAX},
    };

    void getMask(int color, Mat &img, Mat &mask)
    {
        Scalar lower(minMargin[color][0], minMargin[color][1], minMargin[color][2]);
        Scalar upper(maxMargin[color][0], maxMargin[color][1], maxMargin[color][2]);
        inRange(img, lower, upper, mask);
    }

    uchar getMaxVal(uchar val1, uchar val2)
    {
        if (val1 > val2)
            return val1;
        return val2;
    }

    uchar getMinVal(uchar val1, uchar val2)
    {
        if (val1 < val2)
            return val1;
        return val2;
    }

    void getCombindMask(int color1, int color2, Mat &img, Mat &mask)
    {
        Scalar lower(
            getMinVal(minMargin[color1][0], minMargin[color2][0]),
            getMinVal(minMargin[color1][1], minMargin[color2][1]),
            getMinVal(minMargin[color1][2], minMargin[color2][2]));

        Scalar upper(
            getMaxVal(maxMargin[color1][0], maxMargin[color2][0]),
            getMaxVal(maxMargin[color1][1], maxMargin[color2][1]),
            getMaxVal(maxMargin[color1][2], maxMargin[color2][2]));

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
