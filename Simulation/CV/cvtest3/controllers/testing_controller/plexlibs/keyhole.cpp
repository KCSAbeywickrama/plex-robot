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
    DistanceSensor *s0;
    DistanceSensor *s1;
    DistanceSensor *s2;
    DistanceSensor *s3;
    DistanceSensor *s4;
    DistanceSensor *s5;
    DistanceSensor *s6;
    DistanceSensor *s7;


    
    void init(Robot *robot)
    {
        leftMotor = robot->getMotor("leftMotor");
        rightMotor = robot->getMotor("rightMotor");

        leftMotor->setPosition(INFINITY);
        leftMotor->setVelocity(0.0);

        rightMotor->setPosition(INFINITY);
        rightMotor->setVelocity(0.0);

        rLaser = robot->getDistanceSensor("rLaser");
        lLaser = robot->getDistanceSensor("lLaser");
        fLaser = robot->getDistanceSensor("fLaser");
        s0=robot->getDistanceSensor("s0");
        s1=robot->getDistanceSensor("s1");
        s2=robot->getDistanceSensor("s2");
        s3=robot->getDistanceSensor("s3");
        s4=robot->getDistanceSensor("s4");
        s5=robot->getDistanceSensor("s5");
        s6=robot->getDistanceSensor("s6");
        s7=robot->getDistanceSensor("s7");
        
        lLaser->enable(TIME_STEP);
        rLaser->enable(TIME_STEP);
        fLaser->enable(TIME_STEP);
        s0->enable(TIME_STEP);
        s1->enable(TIME_STEP);
        s2->enable(TIME_STEP);
        s3->enable(TIME_STEP);
        s4->enable(TIME_STEP);
        s5->enable(TIME_STEP);
        s6->enable(TIME_STEP);
        s7->enable(TIME_STEP);
    }
    
    void goToCylinder(Robot *robot)
    {
        double oerror=0.0;
        while (robot->step(TIME_STEP) != -1)
        {
            
            double leftD= lLaser->getValue();

            double pError = leftD - 380;
            double dError = pError - oerror;
            oerror = pError;

            double error = 0.0005* pError + 0.0002 * dError;
            double l = error;
            double r = error;

            if (error > 6){l=6;}
            if (error > 3){r=3;}
            if (error < -3){l=-3;}
            if (error < -6){r=-6;}

            leftMotor->setVelocity(3 - l);
            rightMotor->setVelocity(3 + r);
            cout<<"flaser value: "<<fLaser->getValue()<<endl;

            if (fLaser->getValue()<=300)
            {
                cout<<"near cylinder"<<endl;
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);
                return;
            }
        }
    }
    void goToBox(Robot *robot)
    {
        double oerror=0.0;
        
        while (robot->step(TIME_STEP) != -1)
        {
            cout<<"insidef"<<endl;
            double leftD= lLaser->getValue();
            cout<<"value: "<<leftD<<endl;
            double pError = leftD - 180;
            cout<<"error: "<<pError<<endl;
            double dError = pError - oerror;
            oerror = pError;

           double error = 0.0004* pError+0.00015 * dError;
            //double error = 0.8* pError;//+0.00015 * dError;
            double l = error;
            double r = error;

            if (error > 6){l=6;}
            if (error > 3){r=3;}
            if (error < -3){l=-3;}
            if (error < -6){r=-6;}

            leftMotor->setVelocity(3 - l);
            rightMotor->setVelocity(3 + r);

            cout<<"flaser value: "<<fLaser->getValue()<<endl;
            if (fLaser->getValue()<=300)
            {
                cout<<"near box"<<endl;
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);
                return;
            }
        }
    
    }
    void goToLine(Robot *robot)
    {
        double oerror=0.0;
        
        while (robot->step(TIME_STEP) != -1)
        {
            cout<<"insidef"<<endl;
            double leftD= lLaser->getValue();
            cout<<"value: "<<leftD<<endl;
            double pError = leftD - 760;
            cout<<pError<<endl;
            //double dError = pError - oerror;
            oerror = pError;

            double error = 0.25* pError; //- 0.001 * dError;
            double l = error;
            double r = error;

            if (error > 6){l=6;}
            if (error > 3){r=3;}
            if (error < -3){l=-3;}
            if (error < -6){r=-6;}

            leftMotor->setVelocity(3 - l);
            rightMotor->setVelocity(3 + r);


            if (s0->getValue()<400||s1->getValue()<400||s2->getValue()<400||s3->getValue()<400||s4->getValue()<400||s5->getValue()<400||s6->getValue()<400||s7->getValue()<400)
            {
                cout<<"near line"<<endl;
                leftMotor->setVelocity(0);
                rightMotor->setVelocity(0);
                return;
            }
        }
    
    }
    void frontReading(Robot *robot,float &value)
    {
       value = fLaser->getValue() ;
    }
}