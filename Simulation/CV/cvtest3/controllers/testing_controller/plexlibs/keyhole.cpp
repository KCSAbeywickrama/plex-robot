#include "keyhole.hpp"

namespace keyhole
{
    const unsigned char *image;
    Mat imageMat = Mat(Size(width, height), CV_8UC4);
    void goToCylinder(Robot *robot, bool &cylinder)
    {
        while (robot->step(TIME_STEP) != -1 && cylinder)
        {
            image = camera->getImage();
            if (image)
            {

            }
      
        }
    }


    void goToBox(Robot *robot, bool &box)
    {
        while (robot->step(TIME_STEP) != -1 && box)
        {

        }
    }
}