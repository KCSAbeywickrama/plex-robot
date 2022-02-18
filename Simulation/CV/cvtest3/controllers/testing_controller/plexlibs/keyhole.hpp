#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Camera.hpp>
#include <webots/Display.hpp>
#include <webots/ImageRef.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>


//#include <webots/TouchSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <iostream>

#define TIME_STEP 16
using namespace webots;
using namespace std;
using namespace cv;

namespace keyhole
{
    void init(Robot *robot);
    void goToCylinder(Robot *robot);
    void goToBox(Robot *robot);
}