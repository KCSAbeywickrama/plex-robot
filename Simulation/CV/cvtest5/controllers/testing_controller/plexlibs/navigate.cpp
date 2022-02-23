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

    void init(Robot *robot)
    {
        cout << "navigate init" << endl;
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
        cout << "navigateobj" << endl;
        float p_coefficient = 0.1;
        int hmin, hmax, smin, smax, vmin, vmax;
        const unsigned char *image;
        const int width = camera->getWidth();
        const int height = camera->getHeight();
        Mat imageMat = Mat(Size(width, height), CV_8UC4);
        Mat imgAnd = Mat(Size(width, height), CV_8UC4);
        Mat imgRGB, imgHSV, mask, maskRGB, imgCanny, imgDil, imgErode, imgGray, finalim;
        vector<vector<Point>> contours;
        vector<Point> poly;
        vector<Vec4i> hierarchy;

        // hmin = 5, smin = 82, vmin = 15;
        // hmax = 27, smax = 255, vmax = 255;

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

                // Commented by CSA
                // Scalar lower(hmin, smin, vmin);
                // Scalar upper(hmax, smax, vmax);
                // inRange(imgHSV, lower, upper, mask);

                vision::getMask(CLR_O, imgHSV, mask);

                cvtColor(mask, maskRGB, COLOR_GRAY2RGB);
                bitwise_and(imgRGB, maskRGB, imgAnd);

                cvtColor(imgAnd, imgGray, COLOR_RGB2GRAY);
                Canny(imgGray, imgCanny, 100, 255);
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
                HoughLinesP(horizontal, lines, 1, CV_PI / 180, 5, 10, 5);
                HoughLinesP(vertical, lines2, 1, CV_PI / 180, 25, 5, 25);
                cout << "number of hori lines=" << lines.size() << endl;
                cout << "number of ver lines=" << lines2.size() << endl;

                mosaic::showImgGray(imgErode);

                if (lines.size() >= 3)
                {
                    object = 1;
                    cout << "identified box" << endl;
                    return;
                }
                if (lines.size() < 3)
                {
                    object = 2;
                    cout << "identified cylinder" << endl;
                    return;
                }
            }
        }
    }

    void drawContPoints(Mat &imgRGB, vector<Point> &contour)
    {
        size_t n = contour.size();
        for (size_t j = 0; j < n; j++)
        {
            circle(imgRGB, contour[j], 0, Scalar(255, 0, 0), 2);
        }
    }

    void detectObject2(Robot *robot)
    {
        cout << "detect obg2" << endl;
        float p_coefficient = 0.1;
        const unsigned char *image;
        const int width = camera->getWidth();
        const int height = camera->getHeight();
        Mat imageMat = Mat(Size(width, height), CV_8UC4);
        Mat imgAnd = Mat(Size(width, height), CV_8UC4);
        Mat imgRGB, imgHSV, imgGray, mask, maskRGB, maskGray;
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

                cvtColor(imgRGB, imgGray, COLOR_RGB2GRAY);
                // bitwise_and(imgGray, mask, maskGray);
                // findContours(maskGray, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);
                vector<Vec3f> circles;
                HoughCircles(imgGray, circles, HOUGH_GRADIENT, 1,
                             1,            // change this value to detect circles with different distances to each other
                             100, 15, 0, 0 // change the last two parameters
                                           // (min_radius & max_radius) to detect larger circles
                );

                cout << "circles.size()" << circles.size() << endl;

                for (size_t i = 0; i < circles.size(); i++)
                {
                    Vec3i c = circles[i];
                    Point center = Point(c[0], c[1]);
                    // circle center
                    circle(imgRGB, center, 1, Scalar(0, 100, 100), 3, LINE_AA);
                    // circle outline
                    int radius = c[2];
                    circle(imgRGB, center, radius, Scalar(255, 0, 255), 3, LINE_AA);
                }
                // mosaic::showImgGray(imgGray);
                mosaic::showImgRGB(imgRGB);
            }
        }
    }

    void detectObject3(Robot *robot)
    {
        cout << "detect obg3" << endl;
        float p_coefficient = 0.1;
        const unsigned char *image;
        const int width = camera->getWidth();
        const int height = camera->getHeight();
        Mat imageMat = Mat(Size(width, height), CV_8UC4);
        Mat imgAnd = Mat(Size(width, height), CV_8UC4);
        Mat imgRGB, imgHSV, imgGray, mask, maskRGB, maskGray;
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

                findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

                vector<Point> contourPoly;

                approxPolyDP(Mat(contours[0]), contourPoly, 3, true);

                size_t n = contourPoly.size();
                cout << "poly n: " << n << endl;
                drawContPoints(imgRGB, contourPoly);

                mosaic::showImgRGB(imgRGB);
            }
        }
    }

    void navigateObject(Robot *robot)
    {
        cout << "navigateobj" << endl;
        float p_coefficient = 0.1;
        const unsigned char *image;
        const int width = camera->getWidth();
        const int height = camera->getHeight();
        Mat imageMat = Mat(Size(width, height), CV_8UC4);
        Mat imgAnd = Mat(Size(width, height), CV_8UC4);
        Mat imgRGB, imgHSV, imgGray, mask, maskRGB, maskGray;
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
                cout << "matrix size :" << imageMat.size() << endl;

                vision::getMask(CLR_O, imgHSV, mask);

                findContours(mask, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

                cout << "nav:end of find cont" << endl;

                if (contours.empty())
                {
                    cout << "not found" << endl;
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

                    cout << "area" << largestContourArea << "pixel  " << pixel << ' ';

                    if (pixel >= 120)

                    {
                        // leftMotor->setVelocity(0);
                        // rightMotor->setVelocity(0);
                        // return;
                    }
                    if (largestContourArea > 0)
                    {
                        Scalar color = Scalar(0, 255, 0);

                        cvtColor(mask, imgRGB, COLOR_GRAY2RGB);
                        // drawContours(imgRGB, contours, largestContour, color, 2, LINE_8, hierarchy, 0);

                        drawContPoints(imgRGB, contours[largestContour]);

                        mosaic::showImgRGB(imgRGB);

                        // Moments mu = moments(contours[largestContour], false);

                        // int centerx = mu.m10 / mu.m00;
                        // float error = width / 2 - centerx;
                        // cout << "error :" << error << endl;
                        // if (error < 30)
                        // {
                        //     leftMotor->setVelocity((-error * p_coefficient) + 1);
                        //     rightMotor->setVelocity((error * p_coefficient) + 1);
                        // }
                        // else
                        // {
                        //     leftMotor->setVelocity((-error * p_coefficient));
                        //     rightMotor->setVelocity((error * p_coefficient));
                        // }
                    }
                    else
                    {
                        // leftMotor->setVelocity(0.1);
                        // rightMotor->setVelocity(-0.1);
                    }
                }
            }
        }
    }

    void navigateBall(Robot *robot, int red)
    {
        cout << "navigate ball" << endl;
        float p_coefficient = 0.1;
        int hmin, hmax, smin, smax, vmin, vmax;
        const unsigned char *image;
        const int width = camera->getWidth();
        const int height = camera->getHeight();
        Mat imageMat = Mat(Size(width, height), CV_8UC4);
        Mat imgAnd = Mat(Size(width, height), CV_8UC4);
        Mat imgRGB, imgHSV, mask, maskRGB, imgGray; // imgCanny ,imgDil,
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        int clrCode = CLR_R;
        if (red == 0)
        {
            // hmin = 109, smin = 112, vmin = 50;
            // hmax = 120, smax = 255, vmax = 255;
            clrCode = CLR_B;
        }
        else if (red == 1)
        {
            // hmin = 0, smin = 50, vmin = 50;
            // hmax = 11, smax = 255, vmax = 255;
            clrCode = CLR_R;
        }

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

                // Commented by CSA
                // Scalar lower(hmin, smin, vmin);
                // Scalar upper(hmax, smax, vmax);
                // inRange(imgHSV, lower, upper, mask);
                vision::getMask(clrCode, imgHSV, mask);

                cvtColor(mask, maskRGB, COLOR_GRAY2RGB);
                bitwise_and(imgRGB, maskRGB, imgAnd);

                cvtColor(imgAnd, imgGray, COLOR_RGB2GRAY);
                // findContours(imgGray, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_NONE);
                findContours(imgGray, contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

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

                    cout << largestContourArea << ' ' << pixel << ' ';

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
                        Scalar color = Scalar(0, 255, 0);
                        drawContours(imgAnd, contours, largestContour, color, 2, LINE_8, hierarchy, 0);
                        ImageRef *ir = display->imageNew(width, height, imgAnd.data, Display::RGB);

                        display->imagePaste(ir, 0, 0, false);
                        display->imageDelete(ir);
                        Moments mu = moments(contours[largestContour], false);

                        int centerx = mu.m10 / mu.m00;
                        float error = width / 2 - centerx;
                        cout << error << endl;
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
}
