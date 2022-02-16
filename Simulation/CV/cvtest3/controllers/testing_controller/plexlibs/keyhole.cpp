#include "keyhole.hpp"

Camera *camera;

namespace keyhole
{
    Motor *leftMotor;
    Motor *rightMotor;
    Motor *handleMotor;
    Motor *handleEncoder;
    Camera *camera;
    Display *display;


    
    void init(Robot *robot)
    {
    // cout<<"navigate init"<<endl;
    // leftMotor = robot->getMotor("leftMotor");
    // rightMotor = robot->getMotor("rightMotor");
    // handleMotor = robot->getMotor("handleMotor");
    // handleEncoder = robot->getMotor("handleEncoder");
    
    // handleMotor->setPosition(INFINITY);
    // handleMotor->setVelocity(0.0);
    

    // leftMotor->setPosition(INFINITY);
    // leftMotor->setVelocity(0.0);

    // rightMotor->setPosition(INFINITY);
    // rightMotor->setVelocity(0.0);

    // camera = robot->getCamera("cam");
    // camera->enable(TIME_STEP);
    // display = robot->getDisplay("display");
    }
    
    void goToCylinder(Robot *robot)
     {
    //     cout<<"keyhole cylinder"<<endl; 
    //     float p_coefficient = 0.1;
    //     int hmin,hmax,smin,smax,vmin,vmax;
    //     const unsigned char *image;
    //     const int width = camera->getWidth();
    //     const int height = camera->getHeight();
    //     Mat imageMat = Mat(Size(width, height), CV_8UC4);
    //     Mat imgAnd = Mat(Size(width, height), CV_8UC4);
    //     Mat imgRGB, imgHSV, mask, maskRGB, imgGray ;//imgCanny ,imgDil, 
    //     vector<vector<Point>> contours;
    //     vector<Vec4i> hierarchy;
    //     while (robot->step(TIME_STEP) != -1)
    //     {
            
      
    //     }
     }


    void goToBox(Robot *robot)
    {
        while (robot->step(TIME_STEP) != -1)
        {

        }
    }
}