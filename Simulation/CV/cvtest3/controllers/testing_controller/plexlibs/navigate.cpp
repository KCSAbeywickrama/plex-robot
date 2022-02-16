#include "navigate.hpp"
//#include<webots/Motor.hpp>

namespace navigate
{
    Motor *leftMotor;
    Motor *rightMotor;
    Motor *handleMotor;
    Motor *handleEncoder;
    Camera *camera;
    Display *display;

    const unsigned char *image;
    Mat imageMat = Mat(Size(width, height), CV_8UC4);
    Mat imgAnd = Mat(Size(width, height), CV_8UC4);
    Mat imgRGB, imgHSV, mask, maskRGB, imgCanny ,imgDil, imgErode ;
    vector<vector<Point>> contours;
    vector<Point> poly;
    vector<Vec4i> hierarchy;
    RNG rng(12345);
    float p_coefficient = 0.1;

    void getMaxAreaContourId(vector<vector<Point>> contours, int &id, int &area)
    {
    double maxArea = 0;
    id = -1;
    for (int j = 0; j < contours.size(); j++)
    {
        double newArea = contourArea(contours.at(j));
        if (newArea > maxArea)
        {
        maxArea = newArea;
        id = j;
        } 
    }   

    area = (int)maxArea;
    }
    
    void navigateObject(Robot *robot, string &objName) 
    {
        int hmin = 1, smin = 50, vmin = 0;
        int hmax = 88, smax = 255, vmax = 255;
        while (robot->step(TIME_STEP) != -1 )
        {
            handleMotor->setVelocity(1.57);
            handleMotor->setPosition(-1.57);
            
            image = camera->getImage();
            if (image)
            {
                imageMat.data = (uchar *)image;
                cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                Scalar lower(hmin, smin, vmin);
                Scalar upper(hmax, smax, vmax);
                inRange(imgHSV, lower, upper, mask);
                
                
                cvtColor(mask, maskRGB, COLOR_GRAY2RGB);
                bitwise_and(imgRGB,maskRGB,imgAnd);

                cvtColor(imgAnd, imgErode, COLOR_RGB2GRAY);
                //findContours(imgErode, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
                findContours(imgErode, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
            
            

                if (contours.empty())
                {
                    cout<<"not found"<<endl;
                    leftMotor->setVelocity(0.5 );
                    rightMotor->setVelocity(-0.5);

                }
                else
                {
                    int largestContour, largestContourArea;

                    getMaxAreaContourId(contours, largestContour, largestContourArea);
                    Point extTop   = *max_element(contours[largestContour].begin(), contours[largestContour].end(), 
                                [](const Point& lhs, const Point& rhs) {
                                    return lhs.y < rhs.y;
                                });

                    cout << largestContourArea <<' '<< extTop.y << ' ';

                    if (extTop.y >= 127)
                    {
                        //goingToObj = false;
                        // handleMotor->setVelocity(1.57);
                        // handleMotor->setPosition(0);
                        leftMotor->setVelocity(0);
                        rightMotor->setVelocity(0);
                        approxPolyDP(Mat(contours[largestContour]), poly, 1, true); 
                        //box = 7,cylinder = 9
                        if (poly.size()>=9){objName="cylinder";}
                        else{objName="Box";}
                        cout<<poly.size()<<objName<<endl;
                        return;
                    }
                    if (largestContourArea>0)
                    {
                        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                        drawContours(imgAnd, contours, largestContour, color, 2, LINE_8, hierarchy, 0);
                        ImageRef *ir = display->imageNew(width, height, imgAnd.data, Display::RGB);
                        

                        display->imagePaste(ir, 0, 0, false);
                        display->imageDelete(ir);
                        Moments mu = moments(contours[largestContour], false);
                        
                        int centerx = mu.m10 / mu.m00;
                        float error = width / 2 - centerx;
                        cout << error << endl;
                        leftMotor->setVelocity((-error * p_coefficient)+0.5 );
                        rightMotor->setVelocity((error * p_coefficient)+0.5);
                    }
                    else
                    {
                        leftMotor->setVelocity(0.1 );
                        rightMotor->setVelocity(-0.1);
                    }
                }
            }

        } 
    }

    void navigateBall(Robot *robot, bool &goingToBall, string color) 
    {   
        int hmin,hmax,smin,smax,vmin,vmax;
        if (color=="blue")
        {
        int hmin = 1, smin = 50, vmin = 0;
        int hmax = 88, smax = 255, vmax = 255;
        }
        else if (color=="red")
        {
        int hmin = 1, smin = 50, vmin = 0;
        int hmax = 88, smax = 255, vmax = 255;
        }

        while (robot->step(TIME_STEP) != -1 && goingToBall)
        {
            handleMotor->setVelocity(1.57);
            handleMotor->setPosition(-1.57);
            
            image = camera->getImage();
            if (image)
            {
                imageMat.data = (uchar *)image;
                cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                Scalar lower(hmin, smin, vmin);
                Scalar upper(hmax, smax, vmax);
                inRange(imgHSV, lower, upper, mask);
                
                
                cvtColor(mask, maskRGB, COLOR_GRAY2RGB);
                bitwise_and(imgRGB,maskRGB,imgAnd);

                cvtColor(imgAnd, imgErode, COLOR_RGB2GRAY);
                //findContours(imgErode, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
                findContours(imgErode, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
            
            

                if (contours.empty())
                {
                    cout<<"not found"<<endl;
                    leftMotor->setVelocity(0.5 );
                    rightMotor->setVelocity(-0.5);

                }
                else
                {
                    int largestContour, largestContourArea;

                    getMaxAreaContourId(contours, largestContour, largestContourArea);
                    Point extTop   = *max_element(contours[largestContour].begin(), contours[largestContour].end(), 
                                [](const Point& lhs, const Point& rhs) {
                                    return lhs.y < rhs.y;
                                });

                    cout << largestContourArea <<' '<< extTop.y << ' ';

                    if (extTop.y >= 127)
                    {
                        goingToBall = false;
                        leftMotor->setVelocity(0);
                        rightMotor->setVelocity(0);
                        handleMotor->setVelocity(1.57);
                        handleMotor->setPosition(0);
                        for (int i = 0; i < 10; i++)
                        {
                            robot->step(TIME_STEP);
                        }
                        break;
                    }
                    if (largestContourArea>0)
                    {
                        Scalar color = Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
                        drawContours(imgAnd, contours, largestContour, color, 2, LINE_8, hierarchy, 0);
                        ImageRef *ir = display->imageNew(width, height, imgAnd.data, Display::RGB);
                        

                        display->imagePaste(ir, 0, 0, false);
                        display->imageDelete(ir);
                        Moments mu = moments(contours[largestContour], false);
                        
                        int centerx = mu.m10 / mu.m00;
                        float error = width / 2 - centerx;
                        cout << error << endl;
                        leftMotor->setVelocity((-error * p_coefficient)+0.5 );
                        rightMotor->setVelocity((error * p_coefficient)+0.5);
                    }
                    else
                    {
                        leftMotor->setVelocity(0.1 );
                        rightMotor->setVelocity(-0.1);
                    }
                }
            }

        } 
    }
}