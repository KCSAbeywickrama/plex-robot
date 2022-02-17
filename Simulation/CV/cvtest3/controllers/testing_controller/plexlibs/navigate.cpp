#include "navigate.hpp"

namespace navigate
{
    Motor *leftMotor;
    Motor *rightMotor;
    Motor *handleMotor;
    Motor *handleEncoder;
    Camera *camera;
    Display *display;


    
    void init(Robot *robot)
    {
    cout<<"navigate init"<<endl;
    leftMotor = robot->getMotor("leftMotor");
    rightMotor = robot->getMotor("rightMotor");
    handleMotor = robot->getMotor("handleMotor");
    handleEncoder = robot->getMotor("handleEncoder");
    
    handleMotor->setPosition(INFINITY);
    handleMotor->setVelocity(0.0);
    

    leftMotor->setPosition(INFINITY);
    leftMotor->setVelocity(0.0);

    rightMotor->setPosition(INFINITY);
    rightMotor->setVelocity(0.0);

    camera = robot->getCamera("cam");
    camera->enable(TIME_STEP);
    display = robot->getDisplay("display");
    }

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
    // void imageGradient(Mat &img, int width, int height, int &gi, int &gj)
    // {
    //     gi = 0;
    //     gj = 0;

    //     for (int i = 0; i < height - 1; i++)
    //     {
    //         uchar *line0 = img.ptr<uchar>(i);
    //         uchar *line1 = img.ptr<uchar>(i + 1);
    //         for (int j = 0; j < width - 1; j++)
    //         {
    //             gi += abs(line1[j] - line0[j]);
    //             gj += abs(line0[j + 1] - line0[j]);
    //         }
    //     }
    // }

    void detectObject(Robot *robot, int &object)
    {
       cout<<"navigateobj"<<endl;
        float p_coefficient = 0.1;
        int hmin,hmax,smin,smax,vmin,vmax;
        const unsigned char *image;
        const int width = camera->getWidth();
        const int height = camera->getHeight();
        Mat imageMat = Mat(Size(width, height), CV_8UC4);
        Mat imgAnd = Mat(Size(width, height), CV_8UC4);
        Mat imgRGB, imgHSV, mask, maskRGB, imgCanny ,imgDil, imgErode, imgGray, finalim ;
        vector<vector<Point>> contours;
        vector<Point> poly;
        vector<Vec4i> hierarchy;
        

        hmin = 1, smin = 50, vmin = 0;
        hmax = 88, smax = 255, vmax = 255;
        
        while (robot->step(TIME_STEP) != -1 )
        {
            // handleMotor->setVelocity(1.57);
            // handleMotor->setPosition(-1.57);
            
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
                

                cvtColor(imgAnd, imgGray, COLOR_RGB2GRAY);
                Canny(imgGray,imgCanny,100,255);
                // int gi,gj;
                // imageGradient(imgCanny, width,height, gi, gj);
                // cout<<"gi:"<<gi<<" gj: "<<gj<<" sum:"<<gi+gj<<endl;
                Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	            dilate(imgCanny, imgDil, kernel);
	            erode(imgDil, imgErode, kernel);
                findContours(imgErode, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

                vector<Vec4i> lines;  
                vector<Vec4i> lines2;
                Mat horizontal = imgErode.clone();
                Mat vertical = imgErode.clone();
                int horizontal_size = horizontal.cols / 30;
                int vertical_size = vertical.rows / 30;
                Mat horizontalStructure = getStructuringElement(MORPH_RECT, Size(horizontal_size, 1));
                Mat verticalStructure = getStructuringElement(MORPH_RECT, Size(1, vertical_size));
                erode(vertical, vertical, verticalStructure, Point(-1, -1));
                dilate(vertical, vertical, verticalStructure, Point(-1, -1));
                erode(horizontal, horizontal, horizontalStructure, Point(-1, -1));
                dilate(horizontal, horizontal, horizontalStructure, Point(-1, -1));
                HoughLinesP(horizontal, lines, 1, CV_PI/180, 20, 5, 5); 
                HoughLinesP(vertical, lines2, 1, CV_PI/180, 20, 5, 25); 
                cout<<"number of hori lines=" <<lines.size() <<endl;
                cout<<"number of ver lines=" <<lines2.size() <<endl;  

                if(lines.size()>=3)
                {
                    object=1;
                    cout<<"identified box"<<endl;
                    return;
                }
                if(lines.size()<3)
                {
                    object=2;
                    cout<<"identified cylinder"<<endl;
                    return;
                }
            }
        }
    }
    
    void navigateObject(Robot *robot) 
    {
        cout<<"navigateobj"<<endl;
        float p_coefficient = 0.1;
        int hmin,hmax,smin,smax,vmin,vmax;
        const unsigned char *image;
        const int width = camera->getWidth();
        const int height = camera->getHeight();
        Mat imageMat = Mat(Size(width, height), CV_8UC4);
        Mat imgAnd = Mat(Size(width, height), CV_8UC4);
        Mat imgRGB, imgHSV, mask, maskRGB, imgCanny ,imgDil, imgErode, imgGray, finalim ;
        vector<vector<Point>> contours;
        vector<Point> poly;
        vector<Vec4i> hierarchy;
        

        hmin = 1, smin = 50, vmin = 0;
        hmax = 88, smax = 255, vmax = 255;
        
        while (robot->step(TIME_STEP) != -1 )
        {
            // handleMotor->setVelocity(1.57);
            // handleMotor->setPosition(-1.57);
            
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
                

                cvtColor(imgAnd, imgGray, COLOR_RGB2GRAY);
                Canny(imgGray,imgCanny,100,255);
                // int gi,gj;
                // imageGradient(imgCanny, width,height, gi, gj);
                // cout<<"gi:"<<gi<<" gj: "<<gj<<" sum:"<<gi+gj<<endl;
                Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
	            dilate(imgCanny, imgDil, kernel);
	            erode(imgDil, imgErode, kernel);
    
                // vector<Vec4f> lines; // will hold the results of the detection
                // HoughLinesP(imgDil, lines, 1, CV_PI/180, 10, 10, 100 );
                // cout<<"lines "<<lines.size()<<endl;
                //findContours(imgGray, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
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
                    int pixel;
                    getMaxAreaContourId(contours, largestContour, largestContourArea);
                    if (largestContourArea>50)
                    {
                        Point extTop   = *max_element(contours[largestContour].begin(), contours[largestContour].end(), 
                                    [](const Point& lhs, const Point& rhs) {
                                        return lhs.y < rhs.y;
                                    });
                        pixel = extTop.y;
                    }
                    else
                    {
                        pixel=10;
                    }

                    cout << largestContourArea <<' '<< pixel<< ' ';

                     
                    if (pixel>= 120)
                
                    {
                        //goingToObj = false;
                        // handleMotor->setVelocity(1.57);
                        // handleMotor->setPosition(0);
                        leftMotor->setVelocity(0);
                        rightMotor->setVelocity(0);
                        // float epsilon = 0.1*arcLength(contours[largestContour],true);
                        // approxPolyDP(Mat(contours[largestContour]), poly, epsilon, true); 
                        // //box = 7,cylinder = 9
                        // if (poly.size()>=18){objName="box";}
                        // else{objName="cylinder";}
                        // cout<<"poly size "<<poly.size()<<objName<<endl;
                        return;
                    }
                    if (largestContourArea>0)
                    {
                        Scalar color = Scalar(0,255,0);
                        
                        cvtColor(imgCanny, finalim, COLOR_GRAY2RGB);
                        drawContours(finalim, contours, largestContour, color, 2, LINE_8, hierarchy, 0);
                        
                        ImageRef *ir = display->imageNew(width, height, finalim.data, Display::RGB);
                        

                        display->imagePaste(ir, 0, 0, false);
                        display->imageDelete(ir);
                        Moments mu = moments(contours[largestContour], false);
                        
                        int centerx = mu.m10 / mu.m00;
                        float error = width / 2 - centerx;
                        cout << error << endl;
                        if (error<30)
                        {
                            leftMotor->setVelocity((-error * p_coefficient)+1 );
                            rightMotor->setVelocity((error * p_coefficient)+1);
                        }
                        else
                        {
                            leftMotor->setVelocity((-error * p_coefficient));
                            rightMotor->setVelocity((error * p_coefficient));   
                        }
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

    void navigateBall(Robot *robot, string color) 
    {  
        cout<<"navigate ball"<<endl; 
        float p_coefficient = 0.1;
        int hmin,hmax,smin,smax,vmin,vmax;
        const unsigned char *image;
        const int width = camera->getWidth();
        const int height = camera->getHeight();
        Mat imageMat = Mat(Size(width, height), CV_8UC4);
        Mat imgAnd = Mat(Size(width, height), CV_8UC4);
        Mat imgRGB, imgHSV, mask, maskRGB, imgGray ;//imgCanny ,imgDil, 
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        
        
        if (color=="blue")
        {
        hmin = 109, smin = 112, vmin = 50;
        hmax = 120, smax = 255, vmax = 255;
        }
        else if (color=="red")
        {
        hmin = 0, smin = 50, vmin = 50;
        hmax = 11, smax = 255, vmax = 255;
        
        }

        while (robot->step(TIME_STEP) != -1 )
        {
            // handleMotor->setVelocity(1.57);
            // handleMotor->setPosition(-1.57);
            
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

                cvtColor(imgAnd, imgGray, COLOR_RGB2GRAY);
                //findContours(imgGray, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
                findContours(imgGray, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_NONE);
            
            

                if (contours.empty())
                {
                    cout<<"not found"<<endl;
                    leftMotor->setVelocity(0.5 );
                    rightMotor->setVelocity(-0.5);

                }
                else
                {
                    int pixel;
                    int largestContour, largestContourArea;

                    getMaxAreaContourId(contours, largestContour, largestContourArea);
                    if (largestContourArea>50)
                    {
                        Point extTop   = *max_element(contours[largestContour].begin(), contours[largestContour].end(), 
                                    [](const Point& lhs, const Point& rhs) {
                                        return lhs.y < rhs.y;
                                    });
                        pixel = extTop.y;
                    }
                    else
                    {
                        pixel=10;
                    }

                    cout << largestContourArea <<' '<< pixel<< ' ';

                     
                    if (pixel>= 120)
                    {
                        
                        leftMotor->setVelocity(0);
                        rightMotor->setVelocity(0);
                        // handleMotor->setVelocity(1.57);
                        // handleMotor->setPosition(0);
                        // for (int i = 0; i < 10; i++)
                        // {
                        //     robot->step(TIME_STEP);
                        // }
                        return;
                    }
                    if (largestContourArea>0)
                    {
                        Scalar color = Scalar(0,255,0);
                        drawContours(imgAnd, contours, largestContour, color, 2, LINE_8, hierarchy, 0);
                        ImageRef *ir = display->imageNew(width, height, imgAnd.data, Display::RGB);
                        

                        display->imagePaste(ir, 0, 0, false);
                        display->imageDelete(ir);
                        Moments mu = moments(contours[largestContour], false);
                        
                        int centerx = mu.m10 / mu.m00;
                        float error = width / 2 - centerx;
                        cout << error << endl;
                        if (error<30)
                        {
                            leftMotor->setVelocity((-error * p_coefficient)+1 );
                            rightMotor->setVelocity((error * p_coefficient)+1);
                        }
                        else
                        {
                            leftMotor->setVelocity((-error * p_coefficient));
                            rightMotor->setVelocity((error * p_coefficient));   
                        }
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
