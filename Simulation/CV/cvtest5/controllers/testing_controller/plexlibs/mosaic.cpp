#include "vision.hpp"
#include "mosaic.hpp"

using namespace webots;
using namespace std;

namespace mosaic
{
    PositionSensor *leftPosSensor;
    PositionSensor *rightPosSensor;
    Motor *leftMotor;
    Motor *rightMotor;
    Camera *camera;
    Display *display;
    int imgWidth;
    int imgHeight;

    void init(Robot *robot)
    {
        leftMotor = robot->getMotor("leftMotor");
        rightMotor = robot->getMotor("rightMotor");

        leftMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.0);

        rightMotor->setPosition(INFINITY);
        rightMotor->setVelocity(0.0);

        leftPosSensor = robot->getPositionSensor("leftEncoder");
        rightPosSensor = robot->getPositionSensor("rightEncoder");

        leftPosSensor->enable(TIME_STEP);
        rightPosSensor->enable(TIME_STEP);

        camera = robot->getCamera("cam");
        camera->enable(TIME_STEP);
        display = robot->getDisplay("display");

        imgWidth = camera->getWidth();
        imgHeight = camera->getHeight();

        robot->step(TIME_STEP);
    }

    void turnLeft(Robot *robot, float rightThres)
    {
        float rightStart = rightPosSensor->getValue();
        leftMotor->setVelocity(-MOSAIC_SPEED);
        rightMotor->setVelocity(MOSAIC_SPEED);

        while (robot->step(TIME_STEP) != -1 && (rightPosSensor->getValue() - rightStart) < rightThres)
            ;

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    void turnRight(Robot *robot, float leftThres)
    {
        float leftStart = leftPosSensor->getValue();
        leftMotor->setVelocity(MOSAIC_SPEED);
        rightMotor->setVelocity(-MOSAIC_SPEED);

        while (robot->step(TIME_STEP) != -1 && (leftPosSensor->getValue() - leftStart) < leftThres)
            ;

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    void delay(Robot *robot, int count)
    {
        for (int i = 0; i < count; i++)
        {
            robot->step(TIME_STEP);
        }
    }

    void driftLeft(Robot *robot)
    {
        for (int i = 0; i < 20; i++)
        {
            rightMotor->setVelocity(-2);
            leftMotor->setVelocity(0);
            delay(robot, 5);
            rightMotor->setVelocity(0);
            leftMotor->setVelocity(-2);
            delay(robot, 5);
        }

        rightMotor->setVelocity(0);
        leftMotor->setVelocity(0);
    }

    void lookFromLeft(Robot *robot)
    {

        float leftThres = 6.5;
        float leftStart = leftPosSensor->getValue();
        leftMotor->setVelocity(-MOSAIC_SPEED);
        rightMotor->setVelocity(-MOSAIC_SPEED / 3.0);

        while (robot->step(TIME_STEP) != -1 && (leftStart - leftPosSensor->getValue()) < leftThres)
            ;

        goFront(robot, 300);

        float rightThres = 4.5;
        float rightStart = rightPosSensor->getValue();
        leftMotor->setVelocity(MOSAIC_SPEED);
        rightMotor->setVelocity(-MOSAIC_SPEED);

        while (robot->step(TIME_STEP) != -1 && (rightStart - rightPosSensor->getValue()) < rightThres)
            ;

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    void lookFromRight(Robot *robot)
    {

        float rightThres = 7.5;
        float rightStart = rightPosSensor->getValue();
        rightMotor->setVelocity(-MOSAIC_SPEED);
        leftMotor->setVelocity(-MOSAIC_SPEED / 3.0);

        while (robot->step(TIME_STEP) != -1 && (rightStart - rightPosSensor->getValue()) < rightThres)
            ;

        goFront(robot, 500);
        turnLeft(robot);
        goFront(robot, 250);

        float leftThres = 3.5;
        float leftStart = leftPosSensor->getValue();
        rightMotor->setVelocity(MOSAIC_SPEED);
        leftMotor->setVelocity(-MOSAIC_SPEED);

        while (robot->step(TIME_STEP) != -1 && (leftStart - leftPosSensor->getValue()) < leftThres)
            ;

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    void goFront(Robot *robot, float distance)
    {
        // distance in mm
        cout << "front" << endl;
        float rad = distance / 30.0;
        float leftStart = leftPosSensor->getValue();
        float rightStart = rightPosSensor->getValue();

        leftMotor->setVelocity(MOSAIC_SPEED);
        rightMotor->setVelocity(MOSAIC_SPEED);

        while (robot->step(TIME_STEP) != -1)
        {
            if ((leftPosSensor->getValue() - leftStart) >= rad)
            {
                leftMotor->setVelocity(0);
            }
            if ((rightPosSensor->getValue() - rightStart) >= rad)
            {
                rightMotor->setVelocity(0);
            }
            if (((leftPosSensor->getValue() - leftStart) >= rad) && ((rightPosSensor->getValue() - rightStart) >= rad))
            {
                break;
            }
        }
    }

    void goBack(Robot *robot, float distance)
    {
        float rad = distance / 30.0;
        float leftStart = leftPosSensor->getValue();
        float rightStart = rightPosSensor->getValue();

        leftMotor->setVelocity(-MOSAIC_SPEED);
        rightMotor->setVelocity(-MOSAIC_SPEED);

        while (robot->step(TIME_STEP) != -1)
        {
            if ((leftStart - leftPosSensor->getValue()) >= rad)
            {
                leftMotor->setVelocity(0);
            }
            if ((rightStart - rightPosSensor->getValue()) >= rad)
            {
                rightMotor->setVelocity(0);
            }
            if (((leftStart - leftPosSensor->getValue()) >= rad) && ((rightStart - rightPosSensor->getValue()) >= rad))
            {
                break;
            }
        }
    }

    void showImgRGB(Mat &img)
    {
        ImageRef *ir = display->imageNew(imgWidth, imgHeight, img.data, Display::RGB);
        display->imagePaste(ir, 0, 0, false);
        display->imageDelete(ir);
    }

    void showImgGray(Mat &img)
    {
        Mat _img;
        cvtColor(img, _img, COLOR_GRAY2RGB);
        showImgRGB(_img);
    }

    void showFilter(Robot *robot, int color)
    {
        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;

        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {
                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);
                vision::getMask(color, imgHSV, mask);
                showImgGray(mask);
            }
        }
    }

    // void showCombinedFilter(Robot *robot, int color1, int color2)
    // {
    //     const unsigned char *image;
    //     Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
    //     Mat imgRGB, imgHSV, mask;

    //     while (robot->step(TIME_STEP) != -1)
    //     {
    //         image = camera->getImage();
    //         if (image)
    //         {
    //             imgCam.data = (uchar *)image;
    //             cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
    //             cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);
    //             vision::getCombindMask(color1, color2, imgHSV, mask);
    //             showImgGray(mask);
    //         }
    //     }
    // }

    bool notIn(Robot *robot)
    {

        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;

        image = camera->getImage();
        if (image)
        {
            imgCam.data = (uchar *)image;
            cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
            cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

            vision::getMask(CLR_M, imgHSV, mask);

            int i = 0;
            for (i = imgHeight - 1; i >= 0; i--)
            {
                uchar *line = mask.ptr<uchar>(i);
                if (line[imgWidth / 2])
                    break;
            }
            // cout << " i:" << i;
            // deside where to detect mosaic area
            if (i > 110)
            {
                return false;
            }

            showImgGray(mask);
        }

        return true;
    }

    float clipSpeed(float speed)
    {
        if (speed > MOSAIC_SPEED)
            return MOSAIC_SPEED;
        if (speed < -MOSAIC_SPEED)
            return -MOSAIC_SPEED;
        return speed;
    }

    void rotateRightUntil(Robot *robot, int color)
    {
        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;

        leftMotor->setVelocity(MOSAIC_SPEED);
        rightMotor->setVelocity(-MOSAIC_SPEED);

        // rotating
        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {
                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(color, imgHSV, mask);

                int i1 = 0;
                for (i1 = imgHeight - 1; i1 >= 0; i1--)
                {
                    uchar *line = mask.ptr<uchar>(i1);
                    if (line[0])
                        break;
                }

                cout << "i1:" << i1 << endl;

                if (i1 >= 0)
                    break;

                showImgGray(mask);
            }
        }
    }

    void rotateLeftUntil(Robot *robot, int color)
    {
        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;

        leftMotor->setVelocity(-MOSAIC_SPEED);
        rightMotor->setVelocity(MOSAIC_SPEED);

        // rotating
        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {
                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(color, imgHSV, mask);

                int i1 = 0;
                for (i1 = imgHeight - 1; i1 >= 0; i1--)
                {
                    uchar *line = mask.ptr<uchar>(i1);
                    if (line[imgWidth - 1])
                        break;
                }

                cout << "i1:" << i1 << endl;

                if (i1 >= 0)
                    break;

                showImgGray(mask);
            }
        }
    }

    void alignTo(Robot *robot, int color)
    {
        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;
        // aligning
        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {

                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(color, imgHSV, mask);

                int i1 = 0;
                for (i1 = imgHeight - 1; i1 >= 0; i1--)
                {
                    uchar *line = mask.ptr<uchar>(i1);
                    if (line[0])
                        break;
                }

                int i2 = 0;
                for (i2 = imgHeight - 1; i2 >= 0; i2--)
                {
                    uchar *line = mask.ptr<uchar>(i2);
                    if (line[imgWidth - 1])
                        break;
                }

                int error = i2 - i1;
                error = (error / 2) * 2;
                float p_coefficient = 0.8;
                cout << " i1:" << i1;
                cout << " i2:" << i2;
                cout << " lineerror: ";
                cout << error << endl;

                leftMotor->setVelocity(clipSpeed(error * p_coefficient));
                rightMotor->setVelocity(clipSpeed(-error * p_coefficient));
                if (error == 0)
                {
                    return;
                }

                showImgGray(mask);
            }
        }
    }

    void alignWhileGoing(Robot *robot, int color, int dis)
    {
        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;
        // aligning
        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {

                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(color, imgHSV, mask);

                int i1 = 0;
                for (i1 = imgHeight - 1; i1 >= 0; i1--)
                {
                    uchar *line = mask.ptr<uchar>(i1);
                    if (line[0])
                        break;
                }

                int i2 = 0;
                for (i2 = imgHeight - 1; i2 >= 0; i2--)
                {
                    uchar *line = mask.ptr<uchar>(i2);
                    if (line[imgWidth - 1])
                        break;
                }

                int error = i2 - i1;
                error = (error / 2) * 2;
                float p_coefficient = 0.1;
                cout << " i1:" << i1;
                cout << " i2:" << i2;
                cout << " lineerror: ";
                cout << error << endl;

                leftMotor->setVelocity(clipSpeed(error * p_coefficient + MOSAIC_SPEED));
                rightMotor->setVelocity(clipSpeed(-error * p_coefficient + MOSAIC_SPEED));

                if ((error < 2 && error > -2) && (i1 >= dis || i2 >= dis))
                {
                    leftMotor->setVelocity(0);
                    rightMotor->setVelocity(0);
                    return;
                }

                showImgGray(mask);
            }
        }
    }

    void goUntil(Robot *robot, int color, int dis)
    {
        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, mask;

        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {
                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(color, imgHSV, mask);

                int i = 0;
                for (i = imgHeight - 1; i >= 0; i--)
                {
                    uchar *line = mask.ptr<uchar>(i);
                    if (line[imgWidth / 2])
                        break;
                }
                cout << " i:" << i;

                if (i > dis)
                {
                    leftMotor->setVelocity(0);
                    rightMotor->setVelocity(0);
                    return;
                }
                leftMotor->setVelocity(MOSAIC_SPEED);
                rightMotor->setVelocity(MOSAIC_SPEED);

                showImgGray(mask);
            }
        }
    }

    void goCyan2Magenta(Robot *robot)
    {
        cout << "rotate" << endl;
        rotateRightUntil(robot, CLR_M);
        delay(robot, 10);
        alignWhileGoing(robot, CLR_M, 112);
    }

    void goMagenta2Yellow(Robot *robot)
    {
        cout << "rotate" << endl;
        rotateRightUntil(robot, CLR_Y);
        cout << "go while align" << endl;
        alignWhileGoing(robot, CLR_Y, 110);
    }

    void preAlignKeyHole(Robot *robot)
    {
        goCyan2Magenta(robot);
        goFront(robot, 350);
        goMagenta2Yellow(robot);
        // turnLeft(robot);
    }

    void exit(Robot *robot)
    {
        cout << "rotate" << endl;
        rotateRightUntil(robot, CLR_C);
        alignWhileGoing(robot, CLR_C, 112);
        turnRight(robot);
        alignWhileGoing(robot, CLR_Y, 100);
        turnRight(robot);
        alignWhileGoing(robot, CLR_W, 112);
    }

    void directExit(Robot *robot)
    {
        rotateLeftUntil(robot, CLR_W);
        delay(robot, 100);
        alignWhileGoing(robot, CLR_W, 112);
    }

    void goWall2MagentaEnc(Robot *robot)
    {
        goFront(robot, 500);
        turnLeft(robot);
        goFront(robot, 620);
        turnRight(robot);
        goFront(robot, 120);
    }

    void goBox2MagentaEnc(Robot *robot)
    {
        turnRight(robot);
        turnRight(robot);
        goFront(robot, 850);
        turnLeft(robot);
        goFront(robot, 190);
    }

    void goCylinder2MagentaEnc(Robot *robot)
    {
        turnRight(robot);
        turnRight(robot);
        goFront(robot, 850);
        turnLeft(robot);
        goFront(robot, 20);
    }

    void getFloorEndPoint(Mat &maskFloor, int &i, int &j)
    {
        bool find = true;

        for (i = 0; i < imgHeight && find; i++)
        {
            uchar *line = maskFloor.ptr<uchar>(i);
            for (j = imgWidth - 1; j > 0 && find; j--)
            {
                if (line[j])
                    find = false;
            }
        }

        // int i1 = 0;
        // for (i1 = 0; i1 < imgHeight; i1++)
        // {
        //     uchar *line = maskFloor.ptr<uchar>(i1);
        //     if (line[0])
        //         break;
        // }

        // for (int i0 = 0; i0 < imgHeight; i0++)
        // {
        //     uchar *line = maskFloor.ptr<uchar>(i0);

        //     if (line[0])
        //     {
        //         return;
        //     }

        //     if (line[imgWidth - 1])
        //     {
        //         i = i0;
        //         j = imgWidth - 1;
        //         return;
        //     }
        // }

        // i = imgHeight - 1;
        // j = imgWidth - 1;

        // int i2 = 0;
        // for (i2 = 0; i2 < imgHeight; i2++)
        // {
        //     uchar *line = maskFloor.ptr<uchar>(i2);
        //     if (line[imgWidth - 1])
        //         break;
        // }

        // int di1 = i1 - i;
        // int di2 = i2 - i;

        // if (di1 < 0)
        //     di1 = -di1;
        // if (di2 < 0)
        //     di2 = -di2;

        // int dj1 = j;
        // int dj2 = imgWidth - j - 1;

        // if (dj1 < 1)
        //     dj1 = 1;
        // if (dj2 < 1)
        //     dj2 = 1;

        // float d1 = (float)di1 / (float)dj1;
        // float d2 = (float)di2 / (float)dj2;

        // cout << "d1:" << d1 << " d2:" << d2 << endl;

        // if (d1 > d2)
        // {
        //     i = i2;
        //     j = imgWidth - 1;
        // }
        // if (j < imgWidth / 3)
        //     j = imgWidth - 1;
    }

    void goYellow2Box(Robot *robot)
    {
        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, maskFloor, maskHole;

        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {

                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(CLR_Y, imgHSV, maskFloor);
                int i = imgHeight - 1;
                int j = imgWidth - 1;
                getFloorEndPoint(maskFloor, i, j);

                vision::getMask(CLR_W, imgHSV, maskHole);
                cvtColor(maskHole, imgRGB, COLOR_GRAY2RGB);

                int i1 = 0;
                int j1 = 0;
                int j1m = 0;
                bool find = true;

                for (j1 = 0; (j1 < imgWidth) && find; j1++)
                {
                    for (i1 = 0; i1 < i && find; i1++)
                    {
                        uchar *line = maskHole.ptr<uchar>(i1);
                        if (line[j1])
                            find = false;
                        circle(imgRGB, Point(j1, i1), 0, Scalar(255, 255, 0), 5);
                    }
                }

                j1m = (2 * j1 + (int)(1 * i1)) / 2;

                int error = j1m - (imgWidth / 2);
                float p_coefficient = 0.1;

                cout << "error: ";
                cout << error << endl;

                leftMotor->setVelocity(clipSpeed(error * p_coefficient + MOSAIC_SPEED));
                rightMotor->setVelocity(clipSpeed(-error * p_coefficient + MOSAIC_SPEED));

                circle(imgRGB, Point(j, i), 0, Scalar(0, 255, 0), 3);
                circle(imgRGB, Point(j1, i1), 0, Scalar(255, 0, 0), 3);
                circle(imgRGB, Point(j1 + (int)(1 * i1), i1), 0, Scalar(255, 0, 0), 3);
                circle(imgRGB, Point(j1m, i1), 0, Scalar(255, 0, 255), 3);
                showImgRGB(imgRGB);

                if (i1 > 50)
                {
                    leftMotor->setVelocity(0);
                    rightMotor->setVelocity(0);
                    return;
                }
            }
        }
    }

    void goYellow2Cylinder(Robot *robot)
    {
        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, maskFloor, maskHole;

        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {

                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(CLR_Y, imgHSV, maskFloor);
                int i = imgHeight - 1;
                int j = imgWidth - 1;
                getFloorEndPoint(maskFloor, i, j);

                vision::getMask(CLR_W, imgHSV, maskHole);
                cvtColor(maskHole, imgRGB, COLOR_GRAY2RGB);

                int i2 = 0;
                int j2 = 0;
                int j2m = 0;
                bool find = true;
                for (j2 = (j - 2); j2 > 0 && find; j2--)
                {
                    for (i2 = 0; i2 < i && find; i2++)
                    {
                        uchar *line = maskHole.ptr<uchar>(i2);
                        if (line[j2])
                            find = false;
                        // circle(imgRGB, Point(j2, i2), 0, Scalar(0, 0, 255), 5);
                    }
                }

                // int dj = j - prevJ;
                // prevJ = j;

                // if (dj < 0)
                //     dj = -dj;

                // if (dj > 20)
                // {
                //     leftMotor->setVelocity(MOSAIC_SPEED);
                //     rightMotor->setVelocity(0);
                //     delay(robot, 10);
                //     continue;
                // }

                j2m = (2 * j2 - 3 * i2 / 5) / 2;

                int error = j2m - (imgWidth / 2);
                float p_coefficient = 0.1;
                cout << "error: ";
                cout << error << endl;

                leftMotor->setVelocity(clipSpeed(error * p_coefficient + MOSAIC_SPEED));
                rightMotor->setVelocity(clipSpeed(-error * p_coefficient + MOSAIC_SPEED));

                circle(imgRGB, Point(j, i), 0, Scalar(0, 255, 0), 5);
                circle(imgRGB, Point(j2, i2), 0, Scalar(0, 0, 255), 3);
                circle(imgRGB, Point(j2 - 3 * i2 / 5, i2), 0, Scalar(0, 0, 255), 3);
                circle(imgRGB, Point(j2m, i2), 0, Scalar(0, 255, 255), 3);
                showImgRGB(imgRGB);

                if (i2 > 50)
                {
                    leftMotor->setVelocity(0);
                    rightMotor->setVelocity(0);
                    return;
                }
            }
        }
    }

    void tmpViewFloorPoints(Robot *robot)
    {
        const unsigned char *image;
        Mat imgCam = Mat(Size(imgWidth, imgHeight), CV_8UC4);
        Mat imgRGB, imgHSV, maskFloor, maskHole;

        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {

                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getMask(CLR_Y, imgHSV, maskFloor);
                // Point endpoint = detectWallEndPoint(maskFloor);
                // circle(imgRGB, endpoint, 0, Scalar(0, 255, 0), 5);
                // vector<vector<Point>> contours;
                // vector<Vec4i> hierarchy;
                // findContours(maskFloor, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

                // // draw contours on the original image
                // cvtColor(maskFloor, imgRGB, COLOR_GRAY2RGB);
                // // drawContours(imgRGB, contours, -1, Scalar(0, 255, 0), 2);
                // cout << "contourssize()" << contours.size() << endl;
                // for (size_t i = 0; i < contours.size(); i++)
                // {
                //     cout << "contours[i].size()" << contours[i].size() << endl;
                //     for (size_t j = 0; j < contours[i].size(); j++)
                //     {
                //         circle(imgRGB, contours[i][j], 0, Scalar(0, 255, 0), 2);
                //     }
                // }

                showImgRGB(imgRGB);
            }
        }
    }
}
