#include "keyhole.hpp"

Camera *camera;

namespace keyhole
{
    const unsigned char *image;
    Mat imageMat = Mat(Size(width, height), CV_8UC4);
    void goToCylinder(Robot *robot)
    {
        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {

            }
      
        }
    }


    void goToBox(Robot *robot)
    {
        while (robot->step(TIME_STEP) != -1)
        {

        }
    }
}