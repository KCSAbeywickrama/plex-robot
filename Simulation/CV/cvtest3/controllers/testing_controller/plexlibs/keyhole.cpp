#include <webots/DistanceSensor.hpp>
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include "keyhole.hpp"

Camera *camera;

namespace keyhole
{
    Motor *leftMotor;
    Motor *rightMotor;
    DistanceSensor *rLaser;
    DistanceSensor *lLaser;
    DistanceSensor *fLaser;


    
    void init(Robot *robot)
    {
        leftMotor = robot->getMotor("leftMotor");
        rightMotor = robot->getMotor("rightMotor");

        leftMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.0);

        rightMotor->setPosition(INFINITY);
        rightMotor->setVelocity(0.0);

        DistanceSensor *rLaser = robot->getDistanceSensor("rLaser");
        DistanceSensor *lLaser = robot->getDistanceSensor("lLaser");
        DistanceSensor *fLaser = robot->getDistanceSensor("fLaser");
        lLaser->enable(TIME_STEP);
        rLaser->enable(TIME_STEP);
        fLaser->enable(TIME_STEP);
    }
    
    void goToCylinder(Robot *robot)
    {
        double oerror=0.0;
        while (robot->step(TIME_STEP) != -1)
        {
            
            double leftD= lLaser->getValue();
            double pError = leftD - 355;
            double dError = pError - oerror;
            oerror = pError;

            double error = 1* pError - 0.6 * dError;
            double l = error;
            double r = error;

            if (error > 6){l=6;}
            if (error > 3){r=3;}
            if (error < -3){l=-3;}
            if (error < -6){r=-6;}

            leftMotor->setVelocity(3 - l);
            rightMotor->setVelocity(3 + r);

            if (fLaser->getValue()<=20)
            {
                cout<<"near cylinder"<<endl;
                return;
            }
        }
    }
    void goToBox(Robot *robot)
    {
        double oerror=0.0;
        double leftDo= lLaser->getValue();
        cout<<"value: "<<leftDo<<endl;
        while (robot->step(TIME_STEP) != -1)
        {
            cout<<"insidef"<<endl;
            double leftD= lLaser->getValue();
            cout<<"value: "<<leftD<<endl;
            double pError = leftD - 355;
            cout<<pError<<endl;
            double dError = pError - oerror;
            oerror = pError;

            double error = 1* pError - 0.6 * dError;
            double l = error;
            double r = error;

            if (error > 6){l=6;}
            if (error > 3){r=3;}
            if (error < -3){l=-3;}
            if (error < -6){r=-6;}

            leftMotor->setVelocity(3 - l);
            rightMotor->setVelocity(3 + r);

            if (fLaser->getValue()<=20)
            {
                cout<<"near box"<<endl;
                return;
            }
        }
    
    }
}