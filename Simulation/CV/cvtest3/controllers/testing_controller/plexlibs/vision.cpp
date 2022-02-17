#include "vision.hpp"

namespace vision
{
    void gotoObject()
    {
        cout << "vision:gotoObject()" << endl;
    }
    void mainFunction()
    {
        cout << "vision:mainFunction()" << endl;
    }
    void imageGradient(Mat img, int width, int height, int &gi, int &gj)
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
