#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/DistanceSensor.hpp>
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

    void delay(Robot *robot, int count)
    {
        for (int i = 0; i < count; i++)
        {
            robot->step(TIME_STEP);
        }
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
        //turnLeft(robot);
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

    void tmpGoHoles(Robot *robot)
    {
        alignWhileGoing(robot, CLR_K, 120);
    }
}
