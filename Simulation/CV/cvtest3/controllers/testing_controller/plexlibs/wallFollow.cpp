#include "wallFollow.hpp"

namespace wallFollow
{
    Motor *left_motor;
    Motor *right_motor;

    DistanceSensor *ds[3];

    int turnright;
    double oerror;
    char dsNames[3][10];

    void init(Robot *robot);
    {
        dsNames[3][10] = {"ds_left","ds_fleft","ds_fright"};

        left_motor = robot->getMotor("leftMotor");
        right_motor = robot->getMotor("rightMotor");

        left_motor->setPosition(INFINITY);
        right_motor->setPosition(INFINITY);

        left_motor->setVelocity(0.0);
        right_motor->setVelocity(0.0);

        for(int ind=0; ind<3; ++ind)
        {
            ds[ind] = robot->getDistanceSensor(dsNames[ind]);
            ds[ind]->enable(TIME_STEP);
        }

    }

    void leftWallFollow();
    {
        while (robot->step(TIME_STEP) != -1)
        {
            double leftDS = ds[0]->getValue();
            double frontLeftDS = ds[1]->getValue();
            double frontRightDS = ds[2]->getValue();

            if (leftDS > 50){leftDS = 50;}
        
            double perror = leftDS - 16;
            double derror = perror - oerror;
            oerror = perror;

            double error = p*perror - d*derror;

            double l = error;
            double r = error;

            if (error > 2*BASE_SPEED){l=2*BASE_SPEED;}
            if (error > BASE_SPEED){r=BASE_SPEED;}
            if (error < -BASE_SPEED){l=-BASE_SPEED;}
            if (error < -2*BASE_SPEED){r=-2*BASE_SPEED;}

            if ( frontLeftDS < 13 or frontRightDS < 13)
            {
                left_motor->setVelocity(-MAX_SPEED);
                right_motor->setVelocity(-MAX_SPEED);
                robot->step(5);
            }
        
            else if ((frontLeftDS < 18 and (frontRightDS < 22 or frontRightDS > 4*frontLeftDS) and leftDS < 38) or turnright)
            {
                turnright=1;
                if (frontLeftDS < 33 and (frontRightDS <44 or frontRightDS > 2*frontLeftDS) and leftDS < 36){
                    left_motor->setVelocity(5);
                    right_motor->setVelocity(-1);
                }
                else{turnright=0;}
            }
        
            else if (derror == 0 and leftDS > 40)
            {
                left_motor->setVelocity(BASE_SPEED - l/5);
                right_motor->setVelocity(BASE_SPEED + r/5);
            }

            else if (derror > 0)
            {
                left_motor->setVelocity(BASE_SPEED - l/6);
                right_motor->setVelocity(BASE_SPEED + r/6);
            }
        
            else {
                left_motor->setVelocity(BASE_SPEED - l);
                right_motor->setVelocity(BASE_SPEED + r);
            }
            
            //cout<<leftDS<<"  "<<frontLeftDS<<"   "<<frontRightDS<<endl;
        }
    }
}