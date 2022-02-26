#include "mosaic.hpp"
#include "wall.hpp"

namespace wall
{
    Motor *left_motor;
    Motor *right_motor;

    DistanceSensor *ds[3];

    int turnright = 0;
    double oerror = 0;
    int bgin = 0;

    void init(Robot *robot)
    {
        char dsNames[3][10] = {"ds_left", "ds_fleft", "ds_fright"};
        left_motor = robot->getMotor("leftMotor");
        right_motor = robot->getMotor("rightMotor");

        left_motor->setPosition(INFINITY);
        right_motor->setPosition(INFINITY);

        left_motor->setVelocity(0.0);
        right_motor->setVelocity(0.0);

        for (int ind = 0; ind < 3; ++ind)
        {
            ds[ind] = robot->getDistanceSensor(dsNames[ind]);
            ds[ind]->enable(TIME_STEP);
        }
    }

    void follow(Robot *robot)
    {
        while (robot->step(TIME_STEP) != -1 && mosaic::notIn(robot))
        {
            double leftDS = (33.8482 / pow((ds[0]->getValue()), 1.01532)) - 4.50349;
            double frontLeftDS = (33.8482 / pow((ds[1]->getValue()), 1.01532)) - 4.50349;
            double frontRightDS = (33.8482 / pow((ds[2]->getValue()), 1.01532)) - 4.50349;

            if (leftDS > 55)
            {
                leftDS = 55;
            }

            double perror = leftDS - 17;
            double derror = perror - oerror;
            oerror = perror;

            double error = p * perror - d * derror;

            double l = error;
            double r = error;

            if (error > 2 * BASE_SPEED)
            {
                l = 2 * BASE_SPEED;
            }
            if (error > BASE_SPEED)
            {
                r = BASE_SPEED;
            }
            if (error < -BASE_SPEED)
            {
                l = -BASE_SPEED;
            }
            if (error < -2 * BASE_SPEED)
            {
                r = -2 * BASE_SPEED;
            }

            if (bgin == 0)
            {
                int i = 0;
                if (leftDS > 35 || (perror < 3 && perror > -3) || (frontLeftDS < 30 && frontRightDS < 30))
                {
                    bgin = 1;
                }
                else
                {
                    if (perror > 3)
                    {
                        i = 1;
                    }

                    if (perror < -3)
                    {
                        i = -1;
                    }
                    if (i != 0)
                    {
                        left_motor->setVelocity(BASE_SPEED - 3 * i);
                        right_motor->setVelocity(BASE_SPEED + 3 * i);
                        robot->step(400);
                        left_motor->setVelocity(BASE_SPEED);
                        right_motor->setVelocity(BASE_SPEED);
                        robot->step(1000);
                        left_motor->setVelocity(BASE_SPEED + 3 * i);
                        right_motor->setVelocity(BASE_SPEED - 3 * i);
                        robot->step(370);
                        left_motor->setVelocity(BASE_SPEED);
                        right_motor->setVelocity(BASE_SPEED);
                        robot->step(1000);
                        bgin = 1;
                    }

                    if (i == 0)
                    {
                        bgin = 1;
                    }
                }
            }

            if (frontLeftDS < 13 || frontRightDS < 13)
            {
                left_motor->setVelocity(-MAX_SPEED);
                right_motor->setVelocity(-MAX_SPEED);
                robot->step(5);
            }

            else if ((frontLeftDS < 18 && (frontRightDS < 22 || frontRightDS > 4 * frontLeftDS) && leftDS < 38) || turnright)
            {
                turnright = 1;
                if (frontLeftDS < 33 && (frontRightDS < 44 || frontRightDS > 2 * frontLeftDS) && leftDS < 36)
                {
                    left_motor->setVelocity(5);
                    right_motor->setVelocity(-1);
                }
                else
                {
                    turnright = 0;
                }
            }

            else if (derror == 0 && leftDS > 40)
            {
                left_motor->setVelocity(BASE_SPEED - l / 4.5);
                right_motor->setVelocity(BASE_SPEED + r / 4.5);
            }

            else if (derror > 0)
            {
                left_motor->setVelocity(BASE_SPEED - l / 6);
                right_motor->setVelocity(BASE_SPEED + r / 6);
            }

            else
            {
                left_motor->setVelocity(BASE_SPEED - l);
                right_motor->setVelocity(BASE_SPEED + r);
            }

            // cout<<leftDS<<"  "<<frontLeftDS<<"   "<<frontRightDS<<endl;
        }
    }
}