#include "vision.hpp"
#include "mosaic.hpp"
#include "navigate.hpp"

namespace navigate
{
    Motor *leftMotor;
    Motor *rightMotor;
    Motor *handleMotor;
    Motor *handleEncoder;
    Camera *camera;
    Display *display;
    int imgWidth;
    int imgHeight;

    void init(Robot *robot)
    {
        // cout << "navigate init" << endl;
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

        imgWidth = camera->getWidth();
        imgHeight = camera->getHeight();

        display = robot->getDisplay("display");
    }

    void getMaxAreaContourId(vector<vector<Point>> contours, int &id, int &area)
    {
        double maxArea = 0;
        id = 0;
        for (size_t j = 0; j < contours.size(); j++)
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

    void drawContPoints(Mat &imgRGB, vector<Point> &contour)
    {
        size_t n = contour.size();
        for (size_t j = 0; j < n; j++)
        {
            circle(imgRGB, contour[j], 0, Scalar(0, 255, 0), 2);
        }
    }

    int detectObject(Robot *robot)
    {
        // cout << "detecting object" << endl;

        const unsigned char *image;

        Mat imageMat = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;

        vector<vector<Point>> contours;
        vector<Point> contourPoly;
        vector<Vec4i> hierarchy;

        for (int i = 0; i < 15; i++)
        {
            image = camera->getImage();

            if (image)
            {
                imageMat.data = (uchar *)image;
                cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(CLR_O, imgHSV, mask);

                uchar *line = mask.ptr<uchar>(110);
                bool skip = false;
                for (int j = 0; j < imgWidth; j++)
                {
                    if (line[j])
                    {
                        mosaic::goBack(robot, 10);
                        skip = true;
                        break;
                    }
                }

                if (skip)
                {
                    // cout << "skip" << endl;
                    continue;
                }

                findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

                int largestContour, largestContourArea;
                getMaxAreaContourId(contours, largestContour, largestContourArea);

                approxPolyDP(Mat(contours[largestContour]), contourPoly, 2, true);

                size_t n = contourPoly.size();
                cout << "poly n: " << n << endl;

                // Point p1(0, 112), p2(127, 112);

                // int thickness = 2;

                // // Line drawn using 8 connected
                // // Bresenham algorithm
                // line(imgRGB, p1, p2, Scalar(255, 0, 255),
                //      thickness, LINE_8);

                drawContPoints(imgRGB, contourPoly);
                mosaic::showImgRGB(imgRGB);

                if (n > 6)
                    return OBJ_CYLNDR;

                return OBJ_BOX;
            }
        }
        return -1;
    }

    void navigateObject(Robot *robot)
    {
        // cout << "navigate obj" << endl;
        float p_coefficient = 0.1;
        const unsigned char *image;
        Mat imageMat = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;
        vector<vector<Point>> contours;
        vector<Point> poly;
        vector<Vec4i> hierarchy;

        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();

            if (image)
            {
                imageMat.data = (uchar *)image;
                cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(CLR_O, imgHSV, mask);

                findContours(mask, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

                if (contours.empty())
                {
                    leftMotor->setVelocity(0.5);
                    rightMotor->setVelocity(-0.5);
                }
                else
                {
                    int largestContour, largestContourArea;
                    int pixel;
                    getMaxAreaContourId(contours, largestContour, largestContourArea);
                    if (largestContourArea > 50)
                    {
                        Point extTop = *max_element(contours[largestContour].begin(), contours[largestContour].end(),
                                                    [](const Point &lhs, const Point &rhs)
                                                    {
                                                        return lhs.y < rhs.y;
                                                    });
                        pixel = extTop.y;
                    }
                    else
                    {
                        pixel = 10;
                    }

                    if (pixel >= 100)

                    {
                        leftMotor->setVelocity(0);
                        rightMotor->setVelocity(0);
                        return;
                    }
                    if (largestContourArea > 0)
                    {
                        Moments mu = moments(contours[largestContour], false);

                        int centerx = mu.m10 / mu.m00;
                        float error = imgWidth / 2 - centerx;

                        if (error < 30)
                        {
                            leftMotor->setVelocity((-error * p_coefficient) + 1);
                            rightMotor->setVelocity((error * p_coefficient) + 1);
                        }
                        else
                        {
                            leftMotor->setVelocity((-error * p_coefficient));
                            rightMotor->setVelocity((error * p_coefficient));
                        }

                        cvtColor(mask, imgRGB, COLOR_GRAY2RGB);
                        drawContPoints(imgRGB, contours[largestContour]);
                        mosaic::showImgRGB(imgRGB);
                    }
                    else
                    {
                        leftMotor->setVelocity(0.1);
                        rightMotor->setVelocity(-0.1);
                    }
                }
            }
        }
    }

    void navigateBall(Robot *robot, int redBall)
    {
        // cout << "navigate ball" << endl;
        float p_coefficient = 0.1;
        const unsigned char *image;
        Mat imageMat = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        int clrCode = CLR_R;
        if (redBall != 1)
            clrCode = CLR_B;

        while (robot->step(TIME_STEP) != -1)
        {
            // handleMotor->setVelocity(1.57);
            // handleMotor->setPosition(-1.57);

            image = camera->getImage();
            if (image)
            {
                imageMat.data = (uchar *)image;
                cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(clrCode, imgHSV, mask);

                findContours(mask, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

                if (contours.empty())
                {
                    cout << "not found" << endl;
                    leftMotor->setVelocity(0.5);
                    rightMotor->setVelocity(-0.5);
                }
                else
                {
                    int pixel;
                    int largestContour, largestContourArea;

                    getMaxAreaContourId(contours, largestContour, largestContourArea);
                    if (largestContourArea > 50)
                    {
                        Point extTop = *max_element(contours[largestContour].begin(), contours[largestContour].end(),
                                                    [](const Point &lhs, const Point &rhs)
                                                    {
                                                        return lhs.y < rhs.y;
                                                    });
                        pixel = extTop.y;
                    }
                    else
                    {
                        pixel = 10;
                    }

                    if (pixel >= 120)
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
                    if (largestContourArea > 0)
                    {
                        Moments mu = moments(contours[largestContour], false);

                        int centerx = mu.m10 / mu.m00;
                        float error = imgWidth / 2 - centerx;

                        if (error < 30)
                        {
                            leftMotor->setVelocity((-error * p_coefficient) + 1);
                            rightMotor->setVelocity((error * p_coefficient) + 1);
                        }
                        else
                        {
                            leftMotor->setVelocity((-error * p_coefficient));
                            rightMotor->setVelocity((error * p_coefficient));
                        }

                        Scalar color = Scalar(0, 255, 0);
                        drawContours(imgRGB, contours, largestContour, color, 2, LINE_8, hierarchy, 0);
                        mosaic::showImgRGB(imgRGB);
                    }
                    else
                    {
                        leftMotor->setVelocity(0.1);
                        rightMotor->setVelocity(-0.1);
                    }
                }
            }
        }
    }

    void checkNear(Robot *robot)
    {
        // cout << "check near" << endl;
        int checkCount = 0;
        while (robot->step(TIME_STEP) != -1 && checkCount < 25)
        {
            const unsigned char *image;
            Mat imageMat = Mat(Size(imgWidth, imgHeight), CV_8UC4);
            Mat imgRGB, imgHSV, mask;
            vector<vector<Point>> contours;
            vector<Vec4i> hierarchy;

            image = camera->getImage();

            if (image)
            {
                imageMat.data = (uchar *)image;
                cvtColor(imageMat, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(CLR_O, imgHSV, mask);

                findContours(mask, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

                int largestContour, largestContourArea;
                int pixel;
                getMaxAreaContourId(contours, largestContour, largestContourArea);

                Point extTop = *min_element(contours[largestContour].begin(), contours[largestContour].end(),
                                            [](const Point &lhs, const Point &rhs)
                                            {
                                                return lhs.y < rhs.y;
                                            });
                circle(imgRGB, extTop, 0, Scalar(255, 0, 0), 5);
                mosaic::showImgRGB(imgRGB);
                pixel = extTop.y;
                // cout << "pixel: " << pixel << endl;
                if (pixel <= 65)
                {
                    mosaic::goFront(robot, 5);
                }
                else
                {
                    return;
                }
            }
            checkCount++;
        }
    }
}
