#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include "vision.hpp"
#include "mosaic.hpp"

using namespace webots;
using namespace std;

namespace mosaic
{
    Robot *robot;
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

    void turnLeft(Robot *robot)
    {
        float rightStart = rightPosSensor->getValue();
        float rightThres = 3.608;
        leftMotor->setVelocity(-MOSAIC_SPEED);
        rightMotor->setVelocity(MOSAIC_SPEED);

        while (robot->step(TIME_STEP) != -1 && (rightPosSensor->getValue() - rightStart) < rightThres)
            ;

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    void turnRight(Robot *robot)
    {
        float leftStart = leftPosSensor->getValue();
        float leftThres = 3.608;
        leftMotor->setVelocity(2);
        rightMotor->setVelocity(-2);

        while (robot->step(TIME_STEP) != -1 && (leftPosSensor->getValue() - leftStart) < leftThres)
            ;

        leftMotor->setVelocity(0);
        rightMotor->setVelocity(0);
    }

    void goFront(Robot *robot, float distance)
    {
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

    void delay(Robot *robot, int count)
    {
        for (int i = 0; i < count; i++)
        {
            robot->step(TIME_STEP);
        }
    }

    void gotoMegenta(Robot *robot)
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

                vision::getFmMask(imgHSV, mask);

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

        delay(robot, 10);

        while (robot->step(TIME_STEP) != -1)
        {
            image = camera->getImage();
            if (image)
            {

                imgCam.data = (uchar *)image;
                cvtColor(imgCam, imgRGB, COLOR_BGRA2RGB);
                cvtColor(imgRGB, imgHSV, COLOR_RGB2HSV);

                vision::getFmMask(imgHSV, mask);

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
                float p_coefficient = 0.1;
                float d_coefficient = 0.8;
                cout << " i1:" << i1;
                cout << " i2:" << i2;
                cout << " lineerror: ";
                cout << error << endl;

                if (error < 10)
                {
                    if (error == 0 && i1 >= 112)
                    {
                        goFront(robot, 100);
                        break;
                    }
                    leftMotor->setVelocity((error * p_coefficient) + 2);
                    rightMotor->setVelocity((-error * p_coefficient) + 2);
                }
                else
                {
                    leftMotor->setVelocity(MOSAIC_SPEED);
                    rightMotor->setVelocity(-MOSAIC_SPEED);
                }

                showImgGray(mask);
            }
        }
    }
}
